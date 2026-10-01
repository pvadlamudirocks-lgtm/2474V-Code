#pragma once
#include "navigation.hpp"
#include <utility>

// Read-only capture of the existing wait's XY/angular results for a pose move.
void pose_exit_trace_begin();
std::pair<ez::exit_output, ez::exit_output> pose_exit_trace_finish();

// Temporary observer: native PID/exit/wait routines remain unchanged.
void motion_exit_trace_arm();
void motion_exit_trace_begin(const ez::odom& movement);
void motion_exit_trace_finish();
void motion_exit_trace_show();
void motion_controller_snapshot_start();
void turn_velocity_trace_begin();
void turn_velocity_trace_record_result(ez::exit_output result);
ez::exit_output turn_velocity_trace_result();
void final_turn_trace_arm(double public_target);
void final_turn_trace_target(double ez_target);
void final_turn_trace_exit(ez::exit_output result);
void final_turn_trace_wait_returned();
void final_turn_trace_nav_returned();
