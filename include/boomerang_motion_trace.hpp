#pragma once
#include "navigation.hpp"
// No startup/default route arming. One autonomous caller owns these APIs.
bool boomerang_motion_trace_arm(unsigned test_id);
void boomerang_motion_trace_disarm();
void boomerang_motion_trace_begin(ez::pose target,int speed);
enum class BoomerangTraceEvent { Dispatch, WaitReturned, StopRequested, StopReturned };
void boomerang_motion_trace_event(BoomerangTraceEvent event);
void boomerang_motion_trace_finish();
void boomerang_motion_trace_cancel();
bool boomerang_motion_trace_dump(); // Explicit post-motion terminal job; no waiting in control callbacks.
// Explicit TEST-ONLY report: capture wait <=1500ms, output caller wait <=30s; never while PID active.
bool boomerang_motion_trace_report();
// Future stage reporting only. Integer IDs/status are caller-defined, buffered.
// No actions dispatched and no automatic definition of route success.
bool boomerang_motion_trace_stage(unsigned stage_id,int status);
// Results refer to the last completed output job, not a host receipt acknowledgement.
struct BoomerangOutputStatus {
  bool attempted=false,success=false;
  int error=0;
  unsigned rows=0;
};
struct BoomerangReportStatus {
  BoomerangOutputStatus terminal,sd;
  char sd_path[96]{};
};
BoomerangReportStatus boomerang_motion_trace_output_status();
bool boomerang_motion_trace_export(bool terminal,bool sd);
bool boomerang_motion_trace_discard(); // Explicitly release retained frozen data.
void boomerang_motion_trace_synthetic_start(); // One stationary 71-row job, no devices read or commanded.
