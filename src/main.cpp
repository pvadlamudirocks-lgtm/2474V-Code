#include "main.h"
#include "telemetry.hpp"
#include "anti_tip.hpp"
#include "pros/screen.hpp"
#include "liblvgl/llemu.hpp" // Explicit LCD callback declarations for editor and build.

#include <cstdio>
#include <cerrno>
#include <cstring>
#include <cmath>
#include <atomic>
#include <cstdint>

#include "autons.hpp"
#include "dev_tests.hpp"
#include "navigation.hpp"
#include "boomerang_motion_trace.hpp"
#include "nav_diagnostics.hpp"
#include "nav_sequence.hpp"
#include "EZ-Template/navigation_hooks.hpp"
#include "motion_exit_trace.hpp"
#include "path_motion_trace.hpp"
#include "subsystems.hpp"
#include "preload_motion.hpp"

#include "pose_recorder.hpp"
#include "route_config.hpp"

// 2474V lift recovery build marker: fail the build if an old locked header is used.
// This is a source/build check; it does NOT verify physical motor direction.
static_assert(!mechanism_config::LIFT_ROTATION_REVERSED &&
              mechanism_config::LIFT_P16_SOFTWARE_SIGN == +1.0,
              "V7 requires P16 software sign +1 and native reversal false");
static_assert(!mechanism_config::LIFT_MOTOR_SAFETY_LOCKOUT,
              "LIFT_FIX_V7: wrong/old subsystems.hpp has LIFT_MOTOR_SAFETY_LOCKOUT=true");
static_assert(!mechanism_config::ENABLE_LIFT_DIRECTION_DIAGNOSTIC,
              "LIFT_FIX_V7: passive direction diagnostic still enabled");
static_assert(mechanism_config::LIFT_LIMITS_CALIBRATED,
              "LIFT_FIX_V7: lift limits not configured in lift_calibration.hpp");

namespace {
// ----- Tracking geometry and driver settings -----
// Sensor placement lives here; motion PID settings live in default_constants().
constexpr int HORIZONTAL_TRACKER_PORT = 5;
constexpr int VERTICAL_TRACKER_PORT = 7;
constexpr int IMU_PORT = 20;

constexpr double HORIZONTAL_TRACKER_DIAMETER_IN = 2.000;
constexpr double VERTICAL_TRACKER_DIAMETER_IN = 1.9585;
// EZ applies the side sign: BACK P5 is negative; RIGHT P7 is positive.


constexpr double HORIZONTAL_TRACKER_OFFSET_IN = 2.000;
constexpr double VERTICAL_TRACKER_OFFSET_IN = 0.125;
constexpr double TRACKER_GEAR_RATIO = 1.0;

constexpr double TURN_MULTIPLIER = 1.30;
constexpr double DRIVER_ACTIVE_BRAKE_KP = 3.0;
constexpr int DRIVER_DEADZONE = 5;
constexpr int DRIVER_LOOP_MS = 10;
constexpr int ODOM_DISPLAY_REFRESH_MS = 100;
std::atomic<const char*> development_mode{"DRIVER"};
// Display-only lift troubleshooting: leave mechanism commands and lockout unchanged.
constexpr bool SHOW_LIFT_TELEMETRY_SCREEN = true;
std::atomic<int> lift_button_mask{0}; // 1=L1, 2=L2
std::atomic<bool> lift_detail_page{false}; // Controller D-pad RIGHT toggles page.


bool imu_calibrated = false;
bool driver_coast_mode = false;
void restore_driver_brakes() {
  if (!driver_coast_mode) return;
  chassis.opcontrol_drive_activebrake_set(DRIVER_ACTIVE_BRAKE_KP);
  chassis.drive_brake_set(MOTOR_BRAKE_BRAKE);
  driver_coast_mode = false;
}
pros::Mutex odom_display_mutex;
// The selector and live pose display share the Brain screen.
// These flags decide which one may draw while the mutex protects a screen update.
bool odom_display_frozen = true;
bool odom_display_redraw = true; // Protected by odom_display_mutex.
bool auton_selector_screen_active = false; // Protected by odom_display_mutex.

// Selector and odometry share the existing screen-ownership lock.
void auton_selector_page_down() {
  odom_display_mutex.take();
  if (auton_selector_screen_active) ez::as::page_down();
  odom_display_mutex.give();
}
void auton_selector_page_up() {
  odom_display_mutex.take();
  if (auton_selector_screen_active) ez::as::page_up();
  odom_display_mutex.give();
}

void show_auton_selector() {
  // Recovery mode: never access an uninitialized LCD.
  if (!pros::lcd::is_initialized()) return;

  odom_display_mutex.take();
  odom_display_frozen = true;
  odom_diagnostic_screen_release();
  auton_selector_screen_active = true;
  ez::as::auton_selector.selected_auton_print();
  odom_display_mutex.give();
}


int apply_driver_deadzone(int input) {
  if (input >= -DRIVER_DEADZONE && input <= DRIVER_DEADZONE)
    return 0;
  return input;
}

int clamp_driver_output(double output) {
  if (output > 127.0)
    return 127;
  if (output < -127.0)
    return -127;
  return static_cast<int>(output);
}

void driver_arcade_control(int forward_input, int turn_input) {
  // Mix forward and turn stick inputs into left/right drive commands.
  // The deadzone keeps small stick noise from moving the robot.
  // Native chassis forward leads with FRONT; positive turn is clockwise.
  const int forward = apply_driver_deadzone(forward_input);
  const int turn = apply_driver_deadzone(turn_input);
  const int amplified_turn = clamp_driver_output(-turn * TURN_MULTIPLIER);
  const int left_output = clamp_driver_output(forward + amplified_turn);
  const int right_output = clamp_driver_output(forward - amplified_turn);

  chassis.opcontrol_joystick_threshold_iterate(left_output, right_output);
}

// Hand screen ownership to the single recovery-safe coordinate task.
void show_live_odometry() {
  odom_display_mutex.take();
  auton_selector_screen_active = false;
  odom_display_redraw = true;
  odom_display_frozen = false;
  odom_diagnostic_screen_release();
  odom_display_mutex.give();
}

// Same green coordinate layout used in driver control and autonomous.
// Called only by odom_display_task while holding the screen-ownership mutex.
void display_odom_pose() {
  if (odom_display_redraw) {
    pros::screen::set_pen(0x0000FF00);
    pros::screen::fill_rect(0, 0, 479, 239);
    odom_display_redraw = false;
  }
  if (SHOW_LIFT_TELEMETRY_SCREEN && !pros::competition::is_autonomous()) {
    // Read motor telemetry directly so the Brain screen does not depend on
    // a new telemetry struct declaration being visible through subsystems.hpp.
    struct ScreenMotorTelemetry {
      bool connected;
      long raw_counts;
      double actual_rpm;
      long current_ma;
      double temperature_c;
    };
    struct ScreenLiftTelemetry {
      ScreenMotorTelemetry top_p13;
      ScreenMotorTelemetry left_p14;
      ScreenMotorTelemetry right_p15;
      double p16_deg;
    };
    static pros::Motor screen_p13(mechanism_config::THIRD_LIFT_PORT,
                                  pros::v5::MotorGears::green);
    static pros::Motor screen_p14(mechanism_config::LEFT_LIFT_PORT,
                                  pros::v5::MotorGears::green);
    static pros::Motor screen_p15(mechanism_config::RIGHT_LIFT_PORT,
                                  pros::v5::MotorGears::green);
    const auto read_motor = [](pros::Motor& motor) -> ScreenMotorTelemetry {
      std::uint32_t timestamp = 0;
      const auto raw = motor.get_raw_position(&timestamp);
      return ScreenMotorTelemetry{
          motor.is_installed(),
          static_cast<long>(raw),
          motor.get_actual_velocity(),
          static_cast<long>(motor.get_current_draw()),
          motor.get_temperature()};
    };
    const ScreenLiftTelemetry t{
        read_motor(screen_p13),
        read_motor(screen_p14),
        read_motor(screen_p15),
        lift_position_deg()};
    const int buttons = lift_button_mask.load();
    const bool detail = lift_detail_page.load();
    const bool locked = mechanism_config::LIFT_MOTOR_SAFETY_LOCKOUT ||
                        mechanism_config::ENABLE_LIFT_DIRECTION_DIAGNOSTIC;
    pros::screen::set_eraser(0x0000FF00);
    pros::screen::set_pen(0x00000000);
    if (!detail) {
      pros::screen::print(pros::E_TEXT_MEDIUM, 1,
          "LIFT FIX V8   LOCKOUT:%d       ", locked);
      pros::screen::print(pros::E_TEXT_MEDIUM, 2,
          "L1:%d L2:%d  REQ:%-4s P16:%6.1f   ",
          !!(buttons & 1), !!(buttons & 2),
          buttons == 1 ? "UP" : buttons == 2 ? "DOWN" : "STOP", t.p16_deg);
      pros::screen::print(pros::E_TEXT_MEDIUM, 3,
          "P13 TOP   rpm:%+6.1f mA:%5ld   ",
          t.top_p13.actual_rpm, t.top_p13.current_ma);
      pros::screen::print(pros::E_TEXT_MEDIUM, 4,
          "P14 LEFT  rpm:%+6.1f mA:%5ld   ",
          t.left_p14.actual_rpm, t.left_p14.current_ma);
      pros::screen::print(pros::E_TEXT_MEDIUM, 5,
          "P15 RIGHT rpm:%+6.1f mA:%5ld   ",
          t.right_p15.actual_rpm, t.right_p15.current_ma);
      pros::screen::print(pros::E_TEXT_MEDIUM, 6,
          "CONNECTED P13:%d P14:%d P15:%d ",
          t.top_p13.connected, t.left_p14.connected, t.right_p15.connected);
      pros::screen::print(pros::E_TEXT_MEDIUM, 7,
          "D-PAD RIGHT=DETAIL  %s       ", locked ? "NO POWER" : "CHECK LIFT");
    } else {
      pros::screen::print(pros::E_TEXT_MEDIUM, 1,
          "LIFT DETAIL V8 LOCKOUT:%d     ", locked);
      pros::screen::print(pros::E_TEXT_MEDIUM, 2,
          "P13 RAW:%+8ld TEMP:%5.1fC  ",
          t.top_p13.raw_counts, t.top_p13.temperature_c);
      pros::screen::print(pros::E_TEXT_MEDIUM, 3,
          "P14 RAW:%+8ld TEMP:%5.1fC  ",
          t.left_p14.raw_counts, t.left_p14.temperature_c);
      pros::screen::print(pros::E_TEXT_MEDIUM, 4,
          "P15 RAW:%+8ld TEMP:%5.1fC  ",
          t.right_p15.raw_counts, t.right_p15.temperature_c);
      pros::screen::print(pros::E_TEXT_MEDIUM, 5,
          "CONNECTED 13:%d 14:%d 15:%d  ",
          t.top_p13.connected, t.left_p14.connected, t.right_p15.connected);
      pros::screen::print(pros::E_TEXT_MEDIUM, 6,
          "P16:%8.2f   L1:%d L2:%d   ",
          t.p16_deg, !!(buttons & 1), !!(buttons & 2));
      pros::screen::print(pros::E_TEXT_MEDIUM, 7,
          "D-PAD RIGHT=LIVE  %s         ", locked ? "NO POWER" : "CHECK LIFT");
    }
    return;
  }
  pros::screen::set_eraser(0x0000FF00);
  pros::screen::set_pen(0x00000000);
  pros::screen::print(pros::E_TEXT_MEDIUM, 1, "2474V VALKYRIE");

  const auto pose = nav_pose_get();
  pros::screen::print(pros::E_TEXT_MEDIUM, 2,
                      "X:       %8.2f in   ", pose.x);
  pros::screen::print(pros::E_TEXT_MEDIUM, 3,
                      "Y:       %8.2f in   ", pose.y);
  pros::screen::print(pros::E_TEXT_MEDIUM, 4,
                      "HEADING:  %8.2f deg ", pose.theta);
  const double lift_deg = lift_position_deg();
  if (std::isfinite(lift_deg)) {
    pros::screen::print(pros::E_TEXT_MEDIUM, 5,
                        "LIFT: %8.1f deg     ", lift_deg);
  } else {
    pros::screen::print(pros::E_TEXT_MEDIUM, 5,
                        "LIFT: SENSOR ERROR      ");
  }
  pros::screen::print(pros::E_TEXT_MEDIUM, 6,
                      "IMU: %s             ",
                      imu_calibrated ? "CALIBRATED" : "FAILED");
  if (pros::competition::is_autonomous()) {
    pros::screen::print(pros::E_TEXT_MEDIUM, 7,
                        "AUTON: %-16s", development_mode.load());
  } else {
    pros::screen::print(pros::E_TEXT_MEDIUM, 7,
                        "DRIVER CONTROL          ");
  }
}

void odom_display_task() {
  while (true) {
    odom_display_mutex.take();
    // Never compete with an initialized legacy LCD/LVGL screen or a diagnostic
    // that has explicitly acquired screen ownership.
    if (!pros::competition::is_disabled() &&
        !pros::lcd::is_initialized() && !auton_selector_screen_active &&
        !odom_display_frozen && !odom_diagnostic_screen_is_active())
      display_odom_pose();
    odom_display_mutex.give();
    claw_diagnostic_observe(); // Read-only; enabled only during the stationary claw test.
    pros::delay(ODOM_DISPLAY_REFRESH_MS);
  }
}

}  // namespace

void odom_display_freeze() {
  odom_display_mutex.take();
  odom_display_frozen = true;
  odom_display_mutex.give();
}

// ----- Drivetrain configuration -----
// This is the shared EZ-Template drive used by both driver control and autonomous.
ez::Drive chassis(
    {-6, 4},        // LEFT viewed facing FRONT; positive output moves FRONT-first.
    {-3 , 2},       // RIGHT viewed facing FRONT; positive output moves FRONT-first.
    IMU_PORT,         // Real inertial sensor on Smart Port 20.
    2.75,             // Drive-wheel diameter in inches.
    600,              // Blue motor cartridge speed in RPM.
    600.0 / 450.0);   // External ratio for 450 RPM wheel speed.

// Preserve the physically verified tracking-wheel signs. Offset arguments are
// positive physical magnitudes; the attachment methods apply the side convention.
ez::tracking_wheel horizontal_tracker(HORIZONTAL_TRACKER_PORT,
                                      HORIZONTAL_TRACKER_DIAMETER_IN,
                                      HORIZONTAL_TRACKER_OFFSET_IN,
                                      TRACKER_GEAR_RATIO);

ez::tracking_wheel vertical_tracker(VERTICAL_TRACKER_PORT,
                                    VERTICAL_TRACKER_DIAMETER_IN,
                                    VERTICAL_TRACKER_OFFSET_IN,
                                    TRACKER_GEAR_RATIO);

extern pros::Controller master;


void initialize() {
  // ============================================
  // 2474V STARTUP RECOVERY
  // Driver-control startup without PROS LCD/LVGL
  // ============================================

  // Use the direct Brain screen API only.
  // Do not initialize pros::lcd in recovery mode.
  pros::screen::set_eraser(0x00000000);
  pros::screen::erase();
  pros::screen::set_pen(0x00FFFFFF);
  pros::screen::print(
      pros::E_TEXT_MEDIUM, 1, "2474V RECOVERY START");

  std::printf("RECOVERY: initialize entered\n");
  std::printf("CLAW_DIAG_BUILD=2474V_CLAW_H_TRACE_01 compiled=%s %s\n", __DATE__, __TIME__);
  std::printf("LIFT_BUILD_ID=2474V_GREEN_LIFT_V8 lockout=%d diagnostic=%d limits=%d motor_mV=%d p16_soft_sign=%+.0f\n",
              mechanism_config::LIFT_MOTOR_SAFETY_LOCKOUT,
              mechanism_config::ENABLE_LIFT_DIRECTION_DIAGNOSTIC,
              mechanism_config::LIFT_LIMITS_CALIBRATED,
              mechanism_config::LIFT_MANUAL_MV,
              mechanism_config::LIFT_P16_SOFTWARE_SIGN);
  std::fflush(stdout);

  std::printf("LIFT_CONFIG_V8 gears=GREEN nominal_rpm=200 L1=VELOCITY up_target_rpm=%d L2=VELOCITY down_target_rpm=%d [targets, not measured speed]\n",
              mechanism_config::LIFT_UP_TARGET_RPM, mechanism_config::LIFT_DOWN_TARGET_RPM);

  // 1. Prepare drivetrain control.
  chassis.pid_drive_toggle(true);

  // 2. Initialize existing mechanisms.
  mechanisms_initialize();
  if (mechanism_config::ENABLE_LIFT_STARTUP_ZERO &&
      !mechanism_config::ENABLE_LIFT_DIRECTION_DIAGNOSTIC) {

  // If a physical bottom switch is installed,
  // require it to confirm home.
  // Without a switch, the physical home
  // position must be verified by the team.
  const LiftBottomLimit bottom =
      lift_bottom_limit_read();

  const bool bottom_allows_zero =
      bottom == LiftBottomLimit::Unconfigured ||
      bottom == LiftBottomLimit::Pressed;

  if (bottom_allows_zero &&
      lift_zero_at_current_position()) {

    std::printf(
        "LIFT STARTUP ZERO SET: %.2f deg\n",
        lift_position_deg()
    );

  } else {

    std::printf(
        "LIFT STARTUP ZERO FAILED\n"
    );
  }

} else {

  std::printf(
      "LIFT STARTUP ZERO DISABLED\n"
  );
}
  std::printf("RECOVERY: mechanisms initialized\n");
  std::fflush(stdout);

  // 3. Preserve calibrated tracking-wheel setup.
  chassis.odom_tracker_front_set(&horizontal_tracker);
  chassis.odom_tracker_left_set(&vertical_tracker);

  // 4. Preserve driver-control configuration.
  chassis.opcontrol_curve_buttons_toggle(false);
  chassis.opcontrol_curve_default_set(0.0, 0.0);
  chassis.opcontrol_drive_activebrake_set(
      DRIVER_ACTIVE_BRAKE_KP);

  // 5. Preserve existing PID constants.
  default_constants();


  // 6. Reset drivetrain sensors.
  chassis.drive_sensor_reset();

  // 7. Calibrate the IMU without the EZ-Template
  // loading animation or PROS LCD interface.
  imu_calibrated = chassis.drive_imu_calibrate(false);

  // 8. Enable odometry only if calibration succeeds.
  if (imu_calibrated) {
    chassis.odom_enable(true);
    std::printf("RECOVERY: IMU OK, odometry enabled\n");
  } else {
    chassis.odom_enable(false);
    std::printf("RECOVERY: IMU FAILED, odometry disabled\n");
  }

  std::fflush(stdout);

  // 7. Report successful completion.
  std::printf("RECOVERY: initialize completed\n");
  std::fflush(stdout);

  pros::screen::print(
      pros::E_TEXT_MEDIUM, 3, "RECOVERY STARTUP OK");
  pros::screen::print(
      pros::E_TEXT_MEDIUM, 5, "DRIVER MODE TEST");

  master.rumble(".");

  // Start once, after startup screen writes. Remain frozen until a mode begins.
  // Independent of the competition callbacks, including blocking navigation.
  static pros::Task live_display(odom_display_task, TASK_PRIORITY_MIN,
                                 TASK_STACK_DEPTH_DEFAULT, "Live coordinates");
  (void)live_display;

  }


void disabled() {
  ez::autonomous_emergency_stop();

  auton::lifecycle_stop("disabled");
  restore_driver_brakes();
  show_auton_selector();
}

void competition_initialize() {
  show_auton_selector();
}

// ----- Autonomous mode selection -----
// Change these switches to choose one test mode; all false uses the Brain selector.
// Development bypasses selector dispatch; the installed Brain selector is for finalized routes.


constexpr bool RUN_DEV_TEST = false;
constexpr DevTest ACTIVE_DEV_TEST = DevTest::GOTO_POSE;
constexpr bool RUN_CUSTOM_AUTON_TEST = true;
constexpr bool RUN_ROUTE_SEGMENT_TEST = false;
constexpr auton::RouteSegmentTest ACTIVE_ROUTE_SEGMENT = auton::RouteSegmentTest::SNAPSHOT;
static_assert(static_cast<int>(RUN_DEV_TEST)+static_cast<int>(RUN_CUSTOM_AUTON_TEST)+
              static_cast<int>(RUN_ROUTE_SEGMENT_TEST)<=1,"Select only one development mode");
static_assert(!(RUN_DEV_TEST && RUN_CUSTOM_AUTON_TEST), "Select only one development mode");

void custom_auton_test() {
//   const unsigned began = pros::millis();
//   //bool ok = claw_open();  // Physically CLOSE claw.
//   // if (ok) ok = pulse_toggle();  // Existing 395 ms pulse.
//   // if (ok) ok = lift_boomerang_to(514, 19.42, 7.58, 54.14, 40);
//   // if (ok) ok = preload_lower_for_release(32, 2000);
//   // if (ok) ok = claw_close();  // Physically OPEN claw.
//   // preload_auton_finish_log(began, ok);

// const bool recording = boomerang_motion_trace_arm(10016);


// std::printf("BOOM DLEAD TEST: speed=70 dlead=0.50 recorder=%d\n",
//             recording);
// std::fflush(stdout);

// if (!recording) {
//     std::printf("BOOM DLEAD TEST: recorder unavailable, movement cancelled\n");
//     std::fflush(stdout);
//     return;
// }


// nav_go_to_pose(20.66, 8.15, 67.58, 70);
// const bool report_ok = boomerang_motion_trace_report();

// std::printf("BOOM DLEAD TEST: report result=%d\n", report_ok);
// std::fflush(stdout);


claw_close();
pros::delay(250);

lift_auton_move_to_deg(401.7, 2500, LiftFinish::STOP);

lift_auton_move_to_deg(140.2, 2500, LiftFinish::STOP);

pros::delay(250);
claw_open();

} 

void autonomous() {
  if (mechanism_config::ENABLE_LIFT_DIRECTION_DIAGNOSTIC) {
    ez::drive_stop(&chassis);
    lift_auton_stop();
    std::printf("LIFT_PASSIVE: autonomous suppressed; use driver mode for readings\n");
    return;
  }
    // Temporary diagnostic disabled.
    constexpr bool RUN_STATIONARY_TEST = false;

    std::printf("AUTON BUILD CHECK: BOOMERANG ENABLED V1\n");
    std::fflush(stdout);

    if (RUN_STATIONARY_TEST) {
        std::printf("AUTON DRIVE TEST PAUSED\n");
        std::fflush(stdout);
        return;
    }

    // Keep the rest of your existing autonomous() code
    // below this point unchanged.

    std::printf("AUTON ENTERED\n");                
  ez::drive_stop(&chassis);
  ez::autonomous_begin();  // ----- Autonomous dispatch -----
  // Each development branch returns here so it cannot also run a competition route.
  development_mode = RUN_ROUTE_SEGMENT_TEST ? "ROUTE SEGMENT" : RUN_DEV_TEST ? "DEV TEST" : RUN_CUSTOM_AUTON_TEST ? "CUSTOM TEST" : "COMPETITION";
  show_live_odometry();
  stop_all_mechanisms();
  restore_driver_brakes();            

  // Cleanup on every normal return; disabled()/opcontrol() handle task shutdown.
  struct Cleanup {
    ~Cleanup() { nav_cancel_sequence(); stop_all_mechanisms(); }
  } cleanup;
  if (RUN_CUSTOM_AUTON_TEST) {

    std::printf("CUSTOM TEST RUNNING\n");
    nav_set_start(0, 0, 0);  // Drivetrain reference only; never P16.
    custom_auton_test();
    const auto pose = nav_pose_get();
    std::printf("CUSTOM TEST FINISHED: X=%.2f Y=%.2f HEADING=%.2f\n",
                pose.x, pose.y, pose.theta);

    return;
}

  auton::preflight(false); // Optional mechanism/distance snapshot never blocks unrelated tests.

  if (RUN_ROUTE_SEGMENT_TEST) {
    auton::run_route_segment_test(ACTIVE_ROUTE_SEGMENT,route_config::segment_route);
    return;
  }
  

  if (RUN_DEV_TEST) {
    run_dev_test(ACTIVE_DEV_TEST);
    return;
  }

  nav_print_sensor_health(); // Informational, not a selector gate.
  ez::as::auton_selector.selected_auton_call();
} 

void opcontrol() {
  // ----- Driver control -----
  // This loop owns driver motor commands and services the mechanism state machines.

  auton::lifecycle_stop("opcontrol");
  if (mechanism_config::ENABLE_LIFT_DIRECTION_DIAGNOSTIC) {
    std::printf("LIFT_PASSIVE: support lift; driver controls inactive; exit by disabling flag and rebuilding\n");
    while (!pros::competition::is_disabled() && !pros::competition::is_autonomous()) {
      lift_direction_diagnostic_update();
      pros::delay(20);
    }
    lift_auton_stop();
    return;
  }
  ez::driver_control_ready(); // Authorize driver output only after existing cleanup returns.
  development_mode = pose_recorder::enabled ? "POSE RECORDER" : "DRIVER";
  show_live_odometry();
  chassis.drive_brake_set(MOTOR_BRAKE_BRAKE);
  // EZ 3.2.2 also rebases both active-brake targets to current drive positions.
  // Preserve encoders and odometry; do not depend on the AUTON_RAN flag.
  chassis.opcontrol_drive_activebrake_set(DRIVER_ACTIVE_BRAKE_KP);


driver_coast_mode = false;
telemetry_reset();
mechanisms_preflight(false, false, false);


while (true) {
    if (pros::competition::is_disabled() ||
        pros::competition::is_autonomous()) {
      stop_all_mechanisms();
      restore_driver_brakes();
      return;
    }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP) && !driver_coast_mode) {
      driver_coast_mode = true;
      chassis.opcontrol_drive_activebrake_set(0.0);
      chassis.drive_brake_set(MOTOR_BRAKE_COAST);
    }
    const int left_y = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    const int right_x = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    driver_arcade_control(left_y, -right_x);

    // Temporary full-voltage commissioning: held L1 up, held L2 down.
    // Release or both held stops both motors; no presets or automatic return.
    // L1 requests up; L2 requests down. Motor configuration is unchanged.
    // Manual lift controls.
errno=0;
const auto lift_up_raw=master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
const int lift_up_error=errno;
errno=0;
const auto lift_down_raw=master.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
const int lift_down_error=errno;
// Preserve existing bool conversion; expose read errors without changing controls.
const bool lift_up=lift_up_raw;
const bool lift_down=lift_down_raw;

lift_button_mask.store((lift_up ? 1 : 0) | (lift_down ? 2 : 0));
if (SHOW_LIFT_TELEMETRY_SCREEN &&
    master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
    lift_detail_page.store(!lift_detail_page.load());
}

// Live L1/L2 hold-to-run output. The return value says whether the
// controller accepted the request; it does not prove actual shaft direction.
const bool lift_request_accepted = test_lift_motor_response(lift_up, lift_down);

if (SHOW_LIFT_TELEMETRY_SCREEN) {
    static unsigned last_button_log = 0;
    static int last_button_mask = -1;
    const unsigned now = pros::millis();
    const int current_button_mask=(lift_up?1:0)|(lift_down?2:0);
    if (current_button_mask!=last_button_mask || now - last_button_log >= 500) {
        last_button_mask=current_button_mask;
        last_button_log = now;
        std::printf("LIFT_FIX_V8 L1=%d L2=%d accepted=%d lockout=%d limits=%d P16=%.2f rawL1=%ld rawL2=%ld errnoL1=%d errnoL2=%d\n",
                    lift_up, lift_down, lift_request_accepted,
                    mechanism_config::LIFT_MOTOR_SAFETY_LOCKOUT ||
                    mechanism_config::ENABLE_LIFT_DIRECTION_DIAGNOSTIC,
                    mechanism_config::LIFT_LIMITS_CALIBRATED,
                    lift_position_deg(),static_cast<long>(lift_up_raw),static_cast<long>(lift_down_raw),lift_up_error,lift_down_error);
    }
}


// Y alone moves the lift to its calibrated Intake position.
// The zero reference is established only during startup.


if (!mechanism_config::ENABLE_LIFT_CALIBRATION_MODE &&
    master.get_digital_new_press(
        pros::E_CONTROLLER_DIGITAL_DOWN)) {

    if (!lift_up && !lift_down) {

        if (lift_move_preset(
                lift_calibration::Position::INTAKE)) {
            std::printf("INTAKE PRESET ACCEPTED\n");
        } else {
            std::printf("INTAKE PRESET REJECTED\n");
        }

    } else {
        std::printf("INTAKE REJECTED: release L1 and L2\n");
    }
}

    claw_driver_control(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2),
                        master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1));
    // B and X request opposite toggle directions; both or neither means stop.
    const bool toggle_ccw = master.get_digital(pros::E_CONTROLLER_DIGITAL_B);
    const bool toggle_cw = master.get_digital(pros::E_CONTROLLER_DIGITAL_X);
    if (toggle_ccw && !toggle_cw) toggle_run();
    else if (toggle_cw && !toggle_ccw) toggle_run_reverse();
    else toggle_stop();
    mechanisms_update();
    mechanisms_print_debug();
    pose_recorder::update(master);


telemetry_update(master);

pros::delay(DRIVER_LOOP_MS);
}   
}
















