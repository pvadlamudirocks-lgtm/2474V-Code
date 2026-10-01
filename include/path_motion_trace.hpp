#pragma once

#include "navigation.hpp"

// Explicitly opt in for one motion; ordinary path calls remain silent/unarmed.
void path_motion_trace_arm();
void path_motion_trace_begin(const std::vector<ez::odom>& movements);
void path_motion_trace_exit(ez::PID* pid, ez::exit_output result);
void path_motion_trace_wait_returned();
// Diagnostic harness only: stop residual PID after wait, wait for measured rest,
// then dump paced ASCII. False means rest was not verified; no dump is printed.
bool path_motion_trace_stop_and_dump();
