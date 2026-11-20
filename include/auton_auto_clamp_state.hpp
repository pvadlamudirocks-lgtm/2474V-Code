#pragma once
#include <cstdint>

// Hardware-free decision logic. No output, sleeps, sensor reads, or allocation.
namespace auto_clamp {
enum class State { Disabled, Idle, Armed, Clamped, Timeout, Fault };
struct Config {
  int min_mm, max_mm;
  unsigned readings, sample_ms, freshness_ms, timeout_ms;
  constexpr bool valid() const {
    return min_mm > 0 && max_mm >= min_mm && max_mm < 9999 && readings >= 2 &&
        sample_ms > 0 && freshness_ms >= sample_ms && timeout_ms > freshness_ms &&
        timeout_ms < 0x80000000u &&
        readings <= timeout_ms / sample_ms;
  }
};
struct Detector {
  State state = State::Idle;
  unsigned started = 0, last = 0, count = 0;
  bool pending = false;
  constexpr bool arm(unsigned now, const Config& c, bool allowed) {
    if (state == State::Fault || state == State::Armed) return false;
    count = 0; pending = false;
    if (!allowed || !c.valid()) { state = State::Idle; return false; }
    started = last = now; state = State::Armed; return true;
  }
  constexpr void disarm() {
    count = 0; pending = false;
    if (state == State::Armed) state = State::Idle;
  }
  constexpr bool due(unsigned now, const Config& c) const {
    return state == State::Armed && now - last >= c.sample_ms;
  }
  constexpr bool check(unsigned now, const Config& c, bool allowed) {
    if (state != State::Armed) return false;
    if (!allowed) { disarm(); return false; }
    if (now - started >= c.timeout_ms) {
      state = State::Timeout; count = 0; return false;
    }
    return true;
  }
  constexpr bool sample(unsigned now, const Config& c, bool valid, int mm) {
    if (!check(now,c,true) || pending || !due(now,c)) return false;
    if (now - last > c.freshness_ms) count = 0;
    last = now;
    if (!valid || mm < c.min_mm || mm > c.max_mm) count = 0;
    else ++count;
    pending = count >= c.readings;
    return pending;
  }
  constexpr void finish(bool success) {
    if (!pending || state != State::Armed) return;
    pending = false; count = 0;
    state = success ? State::Clamped : State::Fault;
  }
};
}