#pragma once

namespace ez {
class Drive;
// Read-only locals from the native PTP controller, after its motor write.
// No reconstructed controller math and no changes to Drive's object layout.
struct PathControlFrame {
  double signed_target, slew_limit, bias_scale;
  double xy_scaled, angular_scaled, left, right;
  int direction_sign, flipped;
  bool output_enabled;
  double angular_target = 0; // Exact native computed heading target, degrees.
  double target_x = 0, target_y = 0, target_heading = 0;
  bool bias_enabled = false;
  bool slew_left_enabled = false, slew_right_enabled = false;
};
using PathTraceHook = void (*)(Drive*, const PathControlFrame&);
void path_trace_hook_set(PathTraceHook hook);
}  // namespace ez

// Claim/release only an unused observer slot; never replace another diagnostic.
namespace ez { bool path_trace_hook_claim(PathTraceHook hook); void path_trace_hook_release(PathTraceHook hook); }
