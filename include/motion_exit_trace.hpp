#pragma once
#include "navigation.hpp"

// Temporary observer: native PID/exit/wait routines remain unchanged.
void motion_exit_trace_arm();
void motion_exit_trace_begin(const ez::odom& movement);
void motion_exit_trace_finish();
void motion_exit_trace_show();
void motion_controller_snapshot_start();
void turn_velocity_trace_begin();
void turn_velocity_trace_record_result(ez::exit_output result);
ez::exit_output turn_velocity_trace_result();
