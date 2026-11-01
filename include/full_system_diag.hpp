#pragma once
namespace ez { class PID; }
void run_full_system_diag();
void full_system_diag_exit(ez::PID* pid, int reason);
bool full_system_diag_recording(); // Blocking caller only; suppress duplicate DRIVE printing.
