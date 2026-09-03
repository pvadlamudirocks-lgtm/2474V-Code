#pragma once

#include "main.h"

double dist_mm_to_in(double mm);
double dist_project_perpendicular(double raw_distance_in,
                                  double sensor_world_heading_deg,
                                  double reference_normal_heading_deg);
bool dist_angle_safe_for_reset(double sensor_world_heading_deg,
                               double reference_normal_heading_deg,
                               double max_angle_error_deg);

bool dist_read_front_left_in(double &out_in);
bool dist_read_front_right_in(double &out_in);
bool dist_read_back_left_in(double &out_in);
bool dist_read_back_right_in(double &out_in);
bool dist_read_left_in(double &out_in);
bool dist_read_right_in(double &out_in);

bool dist_front_average_in(double &out_in);
bool dist_back_average_in(double &out_in);
bool dist_front_difference_in(double &out_in);
bool dist_back_difference_in(double &out_in);

void dist_print_debug();

bool dist_front_readings_valid();
bool dist_back_readings_valid();
bool dist_side_readings_valid();

bool dist_correct_front_distance(double target_distance_in,
                                 double tolerance_in,
                                 int max_speed,
                                 int timeout_ms);
bool dist_correct_back_distance(double target_distance_in,
                                double tolerance_in,
                                int max_speed,
                                int timeout_ms);

bool dist_square_to_front_surface(double tolerance_in,
                                  int max_turn_speed,
                                  int timeout_ms);
bool dist_square_to_back_surface(double tolerance_in,
                                 int max_turn_speed,
                                 int timeout_ms);

bool dist_reset_y_from_front_reference(double known_reference_y_in,
                                       double robot_front_offset_in,
                                       bool robot_facing_positive_y);
bool dist_reset_x_from_front_reference(double known_reference_x_in,
                                       double robot_front_offset_in,
                                       bool robot_facing_positive_x);

bool dist_reset_y_from_back_reference(double known_reference_y_in,
                                      double robot_back_offset_in,
                                      bool robot_back_facing_positive_y);
bool dist_reset_x_from_back_reference(double known_reference_x_in,
                                      double robot_back_offset_in,
                                      bool robot_back_facing_positive_x);

bool dist_reset_x_from_right_reference(double known_reference_x_in,
                                       double robot_right_offset_in,
                                       bool right_sensor_facing_positive_x);
bool dist_reset_x_from_left_reference(double known_reference_x_in,
                                      double robot_left_offset_in,
                                      bool left_sensor_facing_positive_x);
bool dist_reset_y_from_right_reference(double known_reference_y_in,
                                       double robot_right_offset_in,
                                       bool right_sensor_facing_positive_y);
bool dist_reset_y_from_left_reference(double known_reference_y_in,
                                      double robot_left_offset_in,
                                      bool left_sensor_facing_positive_y);

bool dist_reset_pose_from_front_reference(char axis,
                                          double known_reference_coordinate_in,
                                          double robot_front_offset_in,
                                          bool robot_facing_positive_axis);
