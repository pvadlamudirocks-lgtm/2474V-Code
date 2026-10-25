#include "dev_tests.hpp"
#include "telemetry.hpp"
#include "auton_actions.hpp"
#include "autons.hpp"
#include "nav_diagnostics.hpp"
#include "nav_sequence.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>

namespace {
constexpr bool TWO_MOVE_CHAINED = true;
}
// ----- Chaining qualification -----
// Compare handoff and final-settle behavior using the existing two-move test.
void auton_two_move_chain_test() {
  nav_set_start(0, 0, 0);
  const double imu = chassis.drive_imu_get();
  if (!nav_validate_pose(0, 0, 0, 0.5, 0.5) || !std::isfinite(imu) ||
      std::fabs(std::remainder(imu - nav_game_to_internal_heading(0), 360.0)) > 0.5) {
    if (NAV_DIAGNOSTICS) {
      std::printf("TWO-MOVE TEST ABORTED: start pose/IMU did not settle.\n");
      nav_show_pose();
      nav_print_pose();
    }
    return;
  }

  NavSequence sequence(NAV_DIAGNOSTICS); // Buffered only; dump after finish/rest.
  const auto first_end = TWO_MOVE_CHAINED ? NavEndMode::CHAIN : NavEndMode::SETTLE;
  if (NAV_DIAGNOSTICS) {
    std::printf("\n========== TWO-MOVE START: %s ==========\n"
                "PUBLIC route: (0,0) -> (0,24) -> (24,24), speed=127, FRONT-first\n"
                "Existing EZ forward chain extension: %.3f in; CHAIN: native release; FINAL SETTLE: <=1 in\n",
                TWO_MOVE_CHAINED ? "CHAINED" : "FULL SETTLE",
                chassis.pid_drive_chain_forward_constant_get());
    nav_print_pose();
  }
  const auto start_pose = nav_pose_get();
  const auto began = pros::millis();
  const bool first_completed = nav_go_to(0, 24, 127, first_end);
  const auto first_elapsed = pros::millis() - began;
  const auto second_began = pros::millis();
  const bool completed = first_completed &&
                         nav_go_to(24, 24, 127, NavEndMode::SETTLE);
  const auto second_elapsed = pros::millis() - second_began;
  const auto elapsed_ms = pros::millis() - began; // Through move 2's full settle/rest.
  const bool finished = sequence.finish();       // Existing final stop/rest cleanup.

  if (NAV_DIAGNOSTICS) {
    std::printf("CHAIN leg 1 release: %lu ms; leg 2 settle: %lu ms (dispatched: %s)\n",
                static_cast<unsigned long>(first_elapsed), static_cast<unsigned long>(second_elapsed),
                first_completed ? "YES" : "NO");
    if (completed && finished)
      nav_print_test_result("CHAIN_TWO_MOVE", start_pose, {24, 24, ez::ANGLE_NOT_SET}, elapsed_ms);
    nav_trace_dump(); // No serial output occurred during either movement.
    if (!first_completed) std::printf("MOVE 2 NOT DISPATCHED: movement 1 failed.\n");
    if (completed && finished) {
      std::printf("\n========== TWO-MOVE CHAIN RESULT (%s) ==========\n"
                  "Elapsed through final settle: %lu ms\n",
                  TWO_MOVE_CHAINED ? "CHAINED" : "FULL SETTLE",
                  static_cast<unsigned long>(elapsed_ms));
      if (nav_debug_result(24, 24)) {
        pros::lcd::set_text(0, "TWO-MOVE CHAIN RESULT");
        pros::lcd::print(6, "ELAPSED: %lu ms", static_cast<unsigned long>(elapsed_ms));
        pros::lcd::set_text(7, TWO_MOVE_CHAINED ? "ROBOT | CHAINED" : "ROBOT | FULL SETTLE");
      }
    } else {
      std::printf("\nTWO-MOVE TEST FAILED: %s ; elapsed to failure=%lu ms.\n"
                  "Current snapshot follows; NOT a successful final result.\n",
                  nav_fault(), static_cast<unsigned long>(elapsed_ms));
      nav_show_pose();
      nav_print_pose();
      pros::lcd::set_text(0, "TWO-MOVE TEST FAILED");
      pros::lcd::set_text(5, nav_fault());
    }
    std::fflush(stdout);
  }
}

namespace {
const char* test_name(DevTest test) {
  switch (test) {
#define NAME(value) case DevTest::value: return #value
    NAME(SENSOR_HEALTH); NAME(START_POSE); NAME(MANUAL_ODOM);
    NAME(DRIVE_FORWARD_12); NAME(TURN_90); NAME(GOTO_Y_12); NAME(GOTO_X_12);
    NAME(REVERSE_12); NAME(RETURN_TO_ORIGIN); NAME(TWO_POINT); NAME(SQUARE);
    NAME(GOTO_POSE); NAME(CHAIN_TWO_MOVE); NAME(PURE_PURSUIT_BASIC); NAME(MIRROR_MATH);
    NAME(STABILITY_MONITOR);
#undef NAME
  }
  return "UNKNOWN";
}

// Common before/after reporting keeps primitive tests easy to compare.
template <typename Motion>
void measured_move(const char* label, ez::pose target, Motion motion) {
  const auto start = nav_pose_get();
  nav_print_snapshot("BEFORE MOVE");
  const auto began = pros::millis();
  motion();
  const auto elapsed = pros::millis() - began;
  nav_print_test_result(label, start, target, elapsed);
}
void point(const char* label, double x, double y) {
  measured_move(label, {x, y, ez::ANGLE_NOT_SET}, [=] { nav_go_to(x, y, 40); });
}
}

// ----- Primitive test dispatch -----
// Run the selected foundation test; real route stages use the separate segment harness.
void run_dev_test(DevTest test) {
  if (test == DevTest::STABILITY_MONITOR) {
    telemetry_monitor_test();
    return;
  }
  const char* name = test_name(test);
  std::printf("\n=== DEV TEST START: %s ===\n", name);
  // Scope marker also covers all safe early returns.
  struct EndMarker {
    const char* name;
    ~EndMarker() {
      std::printf("=== DEV TEST END: %s ===\n", name);
      std::fflush(stdout);
    }
  } end{name};
  if (test == DevTest::CHAIN_TWO_MOVE || test == DevTest::PURE_PURSUIT_BASIC) {
    if (!ENABLE_ADVANCED_DEV_TESTS) {
      std::printf("ADVANCED TEST DISABLED\nPass foundation tests before enabling chaining/Pure Pursuit.\n");
      return;
    }
  }
  if (test == DevTest::SENSOR_HEALTH) {
    nav_print_sensor_health();
    return;
  }
  if (std::strcmp(name, "UNKNOWN") == 0) {
    std::printf("UNKNOWN DEV TEST: no movement dispatched.\n");
    return;
  }
  if (test == DevTest::MIRROR_MATH) {
    auton_actions::run_helper_math_test();
    return;
  }
  // Preflight checks issue no motor commands. Do not seed odometry from failed sensors.
  if (!nav_print_sensor_health()) {
    std::printf("DEV TEST ABORTED: sensors unavailable or IMU calibrating.\n");
    return;
  }
  nav_set_start(0, 0, 0);
  switch (test) {
    case DevTest::START_POSE:
      nav_print_snapshot("START POSE");
      return;
    case DevTest::MANUAL_ODOM: {
      std::printf("Manual push window: 30 seconds; +Y FRONT, +X RIGHT, clockwise heading positive.\n");
      nav_manual_odom_sample(false);
      const auto began = pros::millis();
      while (pros::millis() - began < 30000 && !pros::competition::is_disabled()) {
        nav_manual_odom_sample(true);
        pros::delay(50);
      }
      nav_manual_odom_sample(false);
      return;
    }
    case DevTest::DRIVE_FORWARD_12:
      measured_move(name, {0, 12, ez::ANGLE_NOT_SET}, [] { nav_drive_distance(12, 40); });
      break;
    case DevTest::TURN_90:
      measured_move(name, {0, 0, 90}, [] { nav_turn_to(90, 30); });
      break;
    case DevTest::GOTO_Y_12: point(name, 0, 12); break;
    case DevTest::GOTO_X_12: point(name, 12, 0); break;
    case DevTest::REVERSE_12:
      std::printf("Expected: BACK leads toward Y=-12; FRONT remains facing approximately +Y.\n");
      measured_move(name, {0, -12, ez::ANGLE_NOT_SET}, [] { nav_go_to_reverse(0, -12, 40); });
      break;
    case DevTest::RETURN_TO_ORIGIN:
      point("RETURN_TO_ORIGIN outbound", 0, 12);
      point("RETURN_TO_ORIGIN return FRONT-first", 0, 0);
      break;
    case DevTest::TWO_POINT:
      point("TWO_POINT leg 1", 0, 12);
      point("TWO_POINT leg 2", 12, 12);
      break;
    case DevTest::SQUARE:
      // Later foundation test: ordinary blocking point-to-point wrappers, no chaining.
      point("SQUARE leg 1", 0, 12);
      point("SQUARE leg 2", 12, 12);
      point("SQUARE leg 3", 12, 0);
      point("SQUARE leg 4", 0, 0);
      break;
    case DevTest::GOTO_POSE:
  measured_move(name, {12, 12, 90},
                [] { nav_go_to_pose(12, 12, 90, 40); });
  break;
    case DevTest::CHAIN_TWO_MOVE:
      auton_two_move_chain_test(); // Existing geometry, speed, tracing and chain settings.
      break;
    case DevTest::PURE_PURSUIT_BASIC:
      measured_move(name, {12, 24, ez::ANGLE_NOT_SET}, [] {
        if (!nav_follow_path({{0, 12}, {12, 24}}, 40))
          std::printf("PURE PURSUIT REQUEST REJECTED\n");
      });                                       
      break;
    default: break;
  }
}
