#pragma once
#include <cstdint>

// Only explicit powered turn diagnostics arm this deadline. No navigation policy.
namespace geometry_turn {
inline constexpr std::uint32_t maximum_powered_ms = 5000; // Approved emergency cutoff.
enum class Result { Idle, Running, Returned, Timeout, Interrupted, SensorFault };
struct Deadline {
  std::uint32_t started = 0, budget = 0;
  Result result = Result::Idle;
  constexpr void arm(std::uint32_t now, std::uint32_t duration) {
    started = now; budget = duration; result = Result::Running;
  }
  constexpr void update(std::uint32_t now, bool cancelled, bool sensors_ok = true) {
    if (result != Result::Running) return;
    if (cancelled) result = Result::Interrupted;
    else if (static_cast<std::uint32_t>(now - started) >= budget) result = Result::Timeout;
    else if (!sensors_ok) result = Result::SensorFault;
  }
  constexpr void finish(std::uint32_t now, bool cancelled) {
    update(now, cancelled);
    if (result == Result::Running) result = Result::Returned;
  }
  constexpr bool must_stop() const {
    return result == Result::Timeout || result == Result::Interrupted || result == Result::SensorFault;
  }
};
}
