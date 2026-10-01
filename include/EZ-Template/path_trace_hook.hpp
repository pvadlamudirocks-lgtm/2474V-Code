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
};
using PathTraceHook = void (*)(Drive*, const PathControlFrame&);
void path_trace_hook_set(PathTraceHook hook);
}  // namespace ez
