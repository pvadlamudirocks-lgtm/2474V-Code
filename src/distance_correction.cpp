#include "distance_correction.hpp"

#include <cstdint>
#include <cmath>
#include <cstdio>

namespace {
constexpr bool DISTANCE_ENABLE_DEBUG = true;
constexpr bool DISTANCE_ENABLE_MOVEMENT_CORRECTION = true;
constexpr bool DISTANCE_ENABLE_POSE_RESET = true;

// Flip these to -1 if the robot moves or turns the wrong way during testing.
constexpr int DIST_FRONT_CORRECTION_SIGN = 1;
constexpr int DIST_BACK_CORRECTION_SIGN = 1;
constexpr int DIST_FRONT_SQUARE_TURN_SIGN = 1;
constexpr int DIST_BACK_SQUARE_TURN_SIGN = 1;

// TODO: Deferred sensor redesign must use NEW FRONT = old back, NEW BACK =
// old front, NEW LEFT = old right, NEW RIGHT = old left. Fixed field walls
// remain +X, -X, +Y, -Y. Names/ports below are legacy placeholders, unchanged.
// TODO: Placeholder ports. Change all distance sensor ports after wiring.
// Front sensors are useful for intake-side alignment or front-facing references.
pros::Distance frontLeftDistance(8);
pros::Distance frontRightDistance(9);
// Back sensors are priority for scoring alignment because scoring is from back.
pros::Distance backLeftDistance(10);
pros::Distance backRightDistance(11);
// Left/right sensors are useful for wall/reference resets and X/Y drift checks.
pros::Distance leftDistance(12);
pros::Distance rightDistance(13);

constexpr double MM_PER_INCH = 25.4;
constexpr double MIN_VALID_DISTANCE_IN = 1.0;
constexpr double MAX_VALID_DISTANCE_IN = 80.0;
constexpr int MIN_CONFIDENCE = 20;
constexpr int INVALID_DISTANCE_MM = 9999;
constexpr double MAX_DRIVE_CORRECTION_STEP_IN = 3.0;
constexpr double SQUARE_TURN_STEP_DEG = 2.0;
constexpr double MAX_RESET_ANGLE_ERROR_DEG = 25.0;
constexpr double PI = 3.14159265358979323846;

enum class SensorMount { Front, Back, Left, Right };

int abs_int(int value) {
  return value < 0 ? -value : value;
}

int clamp_speed(int speed) {
  const int magnitude = abs_int(speed);
  if (magnitude < 1) return 1;
  if (magnitude > 127) return 127;
  return magnitude;
}

double clamp_double(double value, double min_value, double max_value) {
  if (value < min_value) return min_value;
  if (value > max_value) return max_value;
  return value;
}

double deg_to_rad(double deg) {
  return deg * PI / 180.0;
}

double wrap_angle(double angle_deg) {
  while (angle_deg > 180.0) angle_deg -= 360.0;
  while (angle_deg < -180.0) angle_deg += 360.0;
  return angle_deg;
}

double angle_error(double target_deg, double current_deg) {
  return wrap_angle(target_deg - current_deg);
}

double mount_angle(SensorMount mount) {
  switch (mount) {
    case SensorMount::Front: return 0.0;
    case SensorMount::Right: return 90.0;
    case SensorMount::Back: return 180.0;
    case SensorMount::Left: return -90.0;
  }
  return 0.0;
}

double reference_heading_for_axis(char axis, bool facing_positive_axis) {
  if (axis == 'x' || axis == 'X')
    return facing_positive_axis ? 90.0 : 270.0;
  return facing_positive_axis ? 0.0 : 180.0;
}

bool confidence_valid(pros::Distance &sensor, std::int32_t reading_mm) {
  if (reading_mm <= 200)
    return true;
  const std::int32_t confidence = sensor.get_confidence();
  return confidence != PROS_ERR && confidence >= MIN_CONFIDENCE;
}

bool read_distance_in(pros::Distance &sensor, double &out_in) {
  const std::int32_t reading_mm = sensor.get();
  if (reading_mm == PROS_ERR || reading_mm == INVALID_DISTANCE_MM || reading_mm < 0)
    return false;

  out_in = dist_mm_to_in(static_cast<double>(reading_mm));
  if (out_in < MIN_VALID_DISTANCE_IN || out_in > MAX_VALID_DISTANCE_IN)
    return false;

  return confidence_valid(sensor, reading_mm);
}

bool average_pair(bool (*read_a)(double &), bool (*read_b)(double &), double &out_in) {
  double a = 0.0, b = 0.0;
  if (!read_a(a) || !read_b(b))
    return false;
  out_in = (a + b) / 2.0;
  return true;
}

bool difference_pair(bool (*read_left)(double &), bool (*read_right)(double &), double &out_in) {
  double left = 0.0, right = 0.0;
  if (!read_left(left) || !read_right(right))
    return false;
  out_in = left - right;
  return true;
}

void stop_drive() {
  chassis.drive_set(0, 0);
}

std::uint32_t elapsed_ms(std::uint32_t start_ms) {
  return pros::millis() - start_ms;
}

bool read_front_average(double &out_in) {
  return dist_front_average_in(out_in);
}

bool read_back_average(double &out_in) {
  return dist_back_average_in(out_in);
}

bool correct_distance_with_average(bool (*average_fn)(double &),
                                   double target_distance_in,
                                   double tolerance_in,
                                   int max_speed,
                                   int timeout_ms,
                                   int direction_sign) {
  if (!DISTANCE_ENABLE_MOVEMENT_CORRECTION || timeout_ms <= 0)
    return false;

  tolerance_in = std::fabs(tolerance_in);
  const std::uint32_t start_ms = pros::millis();
  while (elapsed_ms(start_ms) < static_cast<std::uint32_t>(timeout_ms)) {
    double average = 0.0;
    if (!average_fn(average)) {
      stop_drive();
      return false;
    }

    const double error = average - target_distance_in;
    if (std::fabs(error) <= tolerance_in) {
      stop_drive();
      return true;
    }

    const double correction = clamp_double(error * direction_sign,
                                           -MAX_DRIVE_CORRECTION_STEP_IN,
                                           MAX_DRIVE_CORRECTION_STEP_IN);
    chassis.pid_drive_set(correction, clamp_speed(max_speed));
    chassis.pid_wait();
    pros::delay(20);
  }

  stop_drive();
  return false;
}

bool square_with_difference(bool (*difference_fn)(double &),
                            double tolerance_in,
                            int max_turn_speed,
                            int timeout_ms,
                            int turn_sign) {
  if (!DISTANCE_ENABLE_MOVEMENT_CORRECTION || timeout_ms <= 0)
    return false;

  tolerance_in = std::fabs(tolerance_in);
  const std::uint32_t start_ms = pros::millis();
  while (elapsed_ms(start_ms) < static_cast<std::uint32_t>(timeout_ms)) {
    double difference = 0.0;
    if (!difference_fn(difference)) {
      stop_drive();
      return false;
    }

    if (std::fabs(difference) <= tolerance_in) {
      stop_drive();
      return true;
    }

    const double turn_direction = difference > 0.0 ? 1.0 : -1.0;
    const double target_heading = chassis.odom_theta_get() +
                                  turn_direction * turn_sign * SQUARE_TURN_STEP_DEG;
    chassis.pid_turn_set(target_heading, clamp_speed(max_turn_speed));
    chassis.pid_wait();
    pros::delay(20);
  }

  stop_drive();
  return false;
}

bool reset_axis_from_sensor(char axis,
                            bool (*read_fn)(double &),
                            SensorMount mount,
                            double known_reference_coordinate_in,
                            double robot_sensor_offset_in,
                            bool sensor_facing_positive_axis) {
  if (!DISTANCE_ENABLE_POSE_RESET)
    return false;

  double raw_distance = 0.0;
  if (!read_fn(raw_distance))
    return false;

  const double sensor_heading = chassis.odom_theta_get() + mount_angle(mount);
  const double reference_heading = reference_heading_for_axis(axis, sensor_facing_positive_axis);
  if (!dist_angle_safe_for_reset(sensor_heading, reference_heading, MAX_RESET_ANGLE_ERROR_DEG))
    return false;

  const double measured_distance = dist_project_perpendicular(raw_distance,
                                                              sensor_heading,
                                                              reference_heading);
  const double corrected_coordinate = sensor_facing_positive_axis
                                          ? known_reference_coordinate_in - robot_sensor_offset_in - measured_distance
                                          : known_reference_coordinate_in + robot_sensor_offset_in + measured_distance;
  const double current_x = chassis.odom_x_get();
  const double current_y = chassis.odom_y_get();
  const double current_theta = chassis.odom_theta_get();

  if (axis == 'x' || axis == 'X') {
    chassis.odom_xyt_set(corrected_coordinate, current_y, current_theta);
    return true;
  }

  if (axis == 'y' || axis == 'Y') {
    chassis.odom_xyt_set(current_x, corrected_coordinate, current_theta);
    return true;
  }

  return false;
}
}  // namespace

double dist_mm_to_in(double mm) {
  return mm / MM_PER_INCH;
}

double dist_project_perpendicular(double raw_distance_in,
                                  double sensor_world_heading_deg,
                                  double reference_normal_heading_deg) {
  const double error = angle_error(reference_normal_heading_deg, sensor_world_heading_deg);
  return raw_distance_in * std::cos(deg_to_rad(error));
}

bool dist_angle_safe_for_reset(double sensor_world_heading_deg,
                               double reference_normal_heading_deg,
                               double max_angle_error_deg) {
  const double error = std::fabs(angle_error(reference_normal_heading_deg, sensor_world_heading_deg));
  return error <= std::fabs(max_angle_error_deg);
}

bool dist_read_front_left_in(double &out_in) { return read_distance_in(frontLeftDistance, out_in); }
bool dist_read_front_right_in(double &out_in) { return read_distance_in(frontRightDistance, out_in); }
bool dist_read_back_left_in(double &out_in) { return read_distance_in(backLeftDistance, out_in); }
bool dist_read_back_right_in(double &out_in) { return read_distance_in(backRightDistance, out_in); }
bool dist_read_left_in(double &out_in) { return read_distance_in(leftDistance, out_in); }
bool dist_read_right_in(double &out_in) { return read_distance_in(rightDistance, out_in); }

bool dist_front_average_in(double &out_in) {
  return average_pair(dist_read_front_left_in, dist_read_front_right_in, out_in);
}

bool dist_back_average_in(double &out_in) {
  return average_pair(dist_read_back_left_in, dist_read_back_right_in, out_in);
}

bool dist_front_difference_in(double &out_in) {
  return difference_pair(dist_read_front_left_in, dist_read_front_right_in, out_in);
}

bool dist_back_difference_in(double &out_in) {
  return difference_pair(dist_read_back_left_in, dist_read_back_right_in, out_in);
}

void dist_print_debug() {
  if (!DISTANCE_ENABLE_DEBUG)
    return;

  double fl = 0.0, fr = 0.0, bl = 0.0, br = 0.0, left = 0.0, right = 0.0;
  const bool fl_ok = dist_read_front_left_in(fl);
  const bool fr_ok = dist_read_front_right_in(fr);
  const bool bl_ok = dist_read_back_left_in(bl);
  const bool br_ok = dist_read_back_right_in(br);
  const bool left_ok = dist_read_left_in(left);
  const bool right_ok = dist_read_right_in(right);

  std::printf("Distance | FL:%s %.2f | FR:%s %.2f | BL:%s %.2f | BR:%s %.2f | L:%s %.2f | R:%s %.2f\n",
              fl_ok ? "OK" : "BAD", fl,
              fr_ok ? "OK" : "BAD", fr,
              bl_ok ? "OK" : "BAD", bl,
              br_ok ? "OK" : "BAD", br,
              left_ok ? "OK" : "BAD", left,
              right_ok ? "OK" : "BAD", right);

  ez::screen_print("FL " + (fl_ok ? util::to_string_with_precision(fl) : "BAD") +
                       " FR " + (fr_ok ? util::to_string_with_precision(fr) : "BAD") +
                       "\nBL " + (bl_ok ? util::to_string_with_precision(bl) : "BAD") +
                       " BR " + (br_ok ? util::to_string_with_precision(br) : "BAD") +
                       "\nL " + (left_ok ? util::to_string_with_precision(left) : "BAD") +
                       " R " + (right_ok ? util::to_string_with_precision(right) : "BAD"),
                   1);
}

bool dist_front_readings_valid() {
  double average = 0.0;
  return dist_front_average_in(average);
}

bool dist_back_readings_valid() {
  double average = 0.0;
  return dist_back_average_in(average);
}

bool dist_side_readings_valid() {
  double left = 0.0, right = 0.0;
  return dist_read_left_in(left) && dist_read_right_in(right);
}

bool dist_correct_front_distance(double target_distance_in,
                                 double tolerance_in,
                                 int max_speed,
                                 int timeout_ms) {
  return correct_distance_with_average(dist_front_average_in, target_distance_in,
                                       tolerance_in, max_speed, timeout_ms,
                                       DIST_FRONT_CORRECTION_SIGN);
}

bool dist_correct_back_distance(double target_distance_in,
                                double tolerance_in,
                                int max_speed,
                                int timeout_ms) {
  return correct_distance_with_average(dist_back_average_in, target_distance_in,
                                       tolerance_in, max_speed, timeout_ms,
                                       -DIST_BACK_CORRECTION_SIGN);
}

bool dist_square_to_front_surface(double tolerance_in,
                                  int max_turn_speed,
                                  int timeout_ms) {
  return square_with_difference(dist_front_difference_in, tolerance_in,
                                max_turn_speed, timeout_ms,
                                DIST_FRONT_SQUARE_TURN_SIGN);
}

bool dist_square_to_back_surface(double tolerance_in,
                                 int max_turn_speed,
                                 int timeout_ms) {
  return square_with_difference(dist_back_difference_in, tolerance_in,
                                max_turn_speed, timeout_ms,
                                DIST_BACK_SQUARE_TURN_SIGN);
}

bool dist_reset_y_from_front_reference(double known_reference_y_in,
                                       double robot_front_offset_in,
                                       bool robot_facing_positive_y) {
  return reset_axis_from_sensor('y', read_front_average, SensorMount::Front,
                                known_reference_y_in, robot_front_offset_in,
                                robot_facing_positive_y);
}

bool dist_reset_x_from_front_reference(double known_reference_x_in,
                                       double robot_front_offset_in,
                                       bool robot_facing_positive_x) {
  return reset_axis_from_sensor('x', read_front_average, SensorMount::Front,
                                known_reference_x_in, robot_front_offset_in,
                                robot_facing_positive_x);
}

bool dist_reset_y_from_back_reference(double known_reference_y_in,
                                      double robot_back_offset_in,
                                      bool robot_back_facing_positive_y) {
  return reset_axis_from_sensor('y', read_back_average, SensorMount::Back,
                                known_reference_y_in, robot_back_offset_in,
                                robot_back_facing_positive_y);
}

bool dist_reset_x_from_back_reference(double known_reference_x_in,
                                      double robot_back_offset_in,
                                      bool robot_back_facing_positive_x) {
  return reset_axis_from_sensor('x', read_back_average, SensorMount::Back,
                                known_reference_x_in, robot_back_offset_in,
                                robot_back_facing_positive_x);
}

bool dist_reset_x_from_right_reference(double known_reference_x_in,
                                       double robot_right_offset_in,
                                       bool right_sensor_facing_positive_x) {
  return reset_axis_from_sensor('x', dist_read_right_in, SensorMount::Right,
                                known_reference_x_in, robot_right_offset_in,
                                right_sensor_facing_positive_x);
}

bool dist_reset_x_from_left_reference(double known_reference_x_in,
                                      double robot_left_offset_in,
                                      bool left_sensor_facing_positive_x) {
  return reset_axis_from_sensor('x', dist_read_left_in, SensorMount::Left,
                                known_reference_x_in, robot_left_offset_in,
                                left_sensor_facing_positive_x);
}

bool dist_reset_y_from_right_reference(double known_reference_y_in,
                                       double robot_right_offset_in,
                                       bool right_sensor_facing_positive_y) {
  return reset_axis_from_sensor('y', dist_read_right_in, SensorMount::Right,
                                known_reference_y_in, robot_right_offset_in,
                                right_sensor_facing_positive_y);
}

bool dist_reset_y_from_left_reference(double known_reference_y_in,
                                      double robot_left_offset_in,
                                      bool left_sensor_facing_positive_y) {
  return reset_axis_from_sensor('y', dist_read_left_in, SensorMount::Left,
                                known_reference_y_in, robot_left_offset_in,
                                left_sensor_facing_positive_y);
}

bool dist_reset_pose_from_front_reference(char axis,
                                          double known_reference_coordinate_in,
                                          double robot_front_offset_in,
                                          bool robot_facing_positive_axis) {
  return reset_axis_from_sensor(axis, read_front_average, SensorMount::Front,
                                known_reference_coordinate_in, robot_front_offset_in,
                                robot_facing_positive_axis);
}
