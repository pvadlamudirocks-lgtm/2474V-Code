#pragma once
#include "navigation.hpp"
#include <functional>

// Use only from the autonomous task. No nested scopes or raw chassis commands.
// Destructor finishes a pending chain and stops on all normal C++ return paths.
class NavSequence {
 public:
  explicit NavSequence(bool trace = false, unsigned motion_timeout_ms = 8000);
  ~NavSequence();
  NavSequence(const NavSequence&) = delete;
  NavSequence& operator=(const NavSequence&) = delete;
  bool ok() const;
  bool finish();
 private:
  bool owner_ = false;
};
struct NavProgress {
  ez::pose pose;
  double distance; // Accumulated odometry travel during this movement, inches.
  int waypoint;   // Last original waypoint made active by native lookahead; -1 initially.
  unsigned elapsed_ms;
};
struct NavEvent {
  std::function<bool(const NavProgress&)> when;
  std::function<bool()> action; // Short, nonblocking, mechanisms only; false aborts.
  bool required = true;       // Missing a required event aborts at movement end.
};
// Arm events for the NEXT movement. Consumed once; no controller change here.
bool nav_events(std::vector<NavEvent> events);
bool nav_ok();
const char* nav_fault();
void nav_abort(const char* reason = "caller abort"); // reason must have static lifetime
void nav_trace_marker(); // Buffered harmless mechanism-event marker.
void nav_trace_dump();   // Allowed only after sequence finish and measured rest.

// Internal bridge used only by explicit-end-mode navigation overloads.
namespace nav_detail {
bool move(const std::function<void()>& dispatch, NavEndMode end,
          ez::pose target, double distance = 0, bool encoder = false, bool turn = false);
}

// Competition lifecycle cleanup after the previous autonomous task is stopped.
void nav_cancel_sequence();
