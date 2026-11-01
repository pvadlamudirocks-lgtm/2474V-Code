#pragma once
namespace ez {
// Existing NavSequence default: ceiling for continuous PID output, including CHAIN.
inline constexpr unsigned movement_timeout_ms = 8000;
// Bounded synchronization budgets, not physical stopping guarantees.
inline constexpr unsigned movement_lock_ms = 10;
inline constexpr unsigned movement_stop_ms = 100;
inline constexpr bool movement_expired(unsigned now, unsigned start) {
  return now - start >= movement_timeout_ms;
}
}
