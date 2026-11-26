#pragma once
#include <cstdint>
#include <climits>
namespace claw_distance {
enum class Status { Unconfigured, Disconnected, Error, NoObject, Invalid, LowConfidence, Valid };
constexpr Status classify(bool configured, bool installed, int mm, int error,
                          int confidence, int confidence_error, int minimum_confidence) {
  if (!configured) return Status::Unconfigured;
  if (!installed) return Status::Disconnected;
  if (error || mm == INT32_MAX) return Status::Error;
  if (mm == 9999) return Status::NoObject;
  if (mm <= 0) return Status::Invalid;
  if (mm > 200 && (confidence_error || confidence == INT32_MAX)) return Status::Error;
  if (mm > 200 && confidence < minimum_confidence) return Status::LowConfidence;
  return Status::Valid;
}
// Passive sampling and reporting never call an actuator.
void diagnostic_update();
void update();
void cancel();
bool detected();
// Serialize only claw commands and detector state; never wait on a busy mutex.
// Automatic close re-enters from the SAME task while update owns the gate.
class CommandGuard {
 public:
  CommandGuard();
  ~CommandGuard();
  explicit operator bool() const { return acquired_; }
 private:
  bool acquired_ = false, nested_ = false;
};
}
