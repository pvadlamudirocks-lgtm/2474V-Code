#include "auton_log.hpp"
#include "navigation.hpp"
#include <cstdio>
namespace auton {
namespace {
// ----- Buffered route events -----
// Store major events now and print after stopping, so logging does not delay handoffs.
struct Row {
  unsigned ms;
  const char *event, *segment, *motion, *command;
  Result result;
  auton_math::Pose start, target, end;
  int speed;
  double mechanism;
};
constexpr unsigned capacity = 256;
Row rows[capacity];
unsigned count = 0, dropped = 0, run = 0;
const char* route_name = "none";
// Quote labels so commas or quotes in a route name do not break the PC's CSV reader.
void label(const char* s) {
  std::putchar('"');
  if (s) for (; *s; ++s) {
    if (*s == '"') std::putchar('"');
    std::putchar(*s == '\n' || *s == '\r' ? ' ' : *s);
  }
  std::putchar('"');
}
}
const char* result_name(Result r) {
  switch(r) {
#define R(x) case Result::x: return #x
    R(PASS); R(FAIL); R(ABORTED); R(NOT_CONFIGURED); R(PHYSICAL_CALIBRATION_REQUIRED);
    R(TIMEOUT); R(SENSOR_FAULT); R(RETURNED_UNVERIFIED);
#undef R
  }
  return "FAIL";
}
void log_begin(const char* route) { count = dropped = 0; ++run; route_name = route; }
void log_event(const char* event, const char* segment, const char* motion, Result result,
               auton_math::Pose start, auton_math::Pose target, int speed, double mechanism, const char* command) {
  if (count == capacity) { ++dropped; return; }
  const auto p = nav_pose_get();
  rows[count++] = {pros::millis(), event, segment, motion, command, result, start, target,
                   {p.x,p.y,p.theta}, speed, mechanism};
}
// ----- Terminal export -----
// Each row contains the captured pose and error; missing targets remain NaN.
void log_flush() {
  std::printf("AUTONCSV,run,ms,event,route,segment,motion,result,start_x,start_y,start_h,target_x,target_y,target_h,speed,end_x,end_y,end_h,position_error,heading_error,mechanism_position,command\n");
  for (unsigned i=0; i<count; ++i) {
    const auto& r = rows[i];
    std::printf("AUTONCSV,%u,%u,",run,r.ms); label(r.event); std::putchar(',');
    label(route_name); std::putchar(','); label(r.segment); std::putchar(','); label(r.motion);
    std::printf(",%s,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,",
      result_name(r.result),r.start.x,r.start.y,r.start.h,r.target.x,r.target.y,r.target.h,r.speed,
      r.end.x,r.end.y,r.end.h,std::hypot(r.target.x-r.end.x,r.target.y-r.end.y),
      auton_math::error(r.target.h,r.end.h),r.mechanism);
    label(r.command); std::putchar('\n');
  }
  if (dropped) std::printf("AUTON_LOG_DROPPED,%u\n",dropped);
  count = dropped = 0;
  std::fflush(stdout);
}
}
