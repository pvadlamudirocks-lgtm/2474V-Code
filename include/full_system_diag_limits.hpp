#pragma once
namespace full_diag {
// Passive recording durations only; never gate or supervise drivetrain commands.
inline constexpr unsigned health_ms=2000,countdown_ms=5000,manual_ms=8000,passive_sample_ms=50;
// Logging only: sampling stays at 50 ms regardless of this switch.
inline constexpr bool detailed_trace=false;
inline constexpr unsigned trace_max_rows=8,trace_print_interval_ms=100;
constexpr unsigned trace_index(unsigned row,unsigned count,unsigned selected){
  return selected<=1 ? 0 : row*(count-1)/(selected-1);
}
constexpr bool expired(unsigned now,unsigned start,unsigned budget){return now-start>=budget;}
}
