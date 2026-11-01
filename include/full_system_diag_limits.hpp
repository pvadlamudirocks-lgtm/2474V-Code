#pragma once
#include <cmath>
namespace full_diag {
enum class Profile { PASSIVE, MOTOR_BENCH, CONTROLLED_MOTION };
// Select a profile here. Bench verification is a human hardware finding, never
// set automatically from encoder signs or a successful software build.
inline constexpr Profile profile=Profile::PASSIVE;
inline constexpr bool motor_bench_physically_verified=false;
inline constexpr bool controlled_motion_enabled=false;
inline constexpr bool tracker_geometry_physically_verified=false;
constexpr bool power_allowed(Profile p,bool bench_verified,bool motion_enabled){
  return p==Profile::MOTOR_BENCH ||
    (p==Profile::CONTROLLED_MOTION && bench_verified && motion_enabled);
}
inline constexpr unsigned health_ms=2000,countdown_ms=5000,manual_ms=8000,passive_sample_ms=50;
// Diagnostic stimuli/abort bounds; not PID tuning or physical qualification.
inline constexpr int pulse_mv=2000, speed=30;
inline constexpr unsigned pulse_ms=250, motion_ms=1500, rest_ms=100, settle_ms=2000;
inline constexpr double turn_deg=15, drive_in=3, max_turn_deg=30, max_travel_in=6;
constexpr bool expired(unsigned now,unsigned start,unsigned budget){return now-start>=budget;}
constexpr bool wrong_turn(double change,double requested){return change*(requested>0 ? 1 : -1)<-3;}
constexpr bool excessive(double angle,double travel){return !std::isfinite(angle)||!std::isfinite(travel)||std::fabs(angle)>max_turn_deg||travel>max_travel_in;}
}
