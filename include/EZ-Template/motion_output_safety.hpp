#pragma once
#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
namespace ez { namespace output_safety {
enum class Fault : unsigned { None, Path, Slew, State, Output };
class Latch {
 std::atomic<Fault> reason_{Fault::None};
 public:
 void set(Fault reason) { auto empty=Fault::None; reason_.compare_exchange_strong(empty,reason); }
 Fault get() const { return reason_.load(); }
 void reset() { reason_.store(Fault::None); }
};
inline bool speed_valid(double v) { return std::isfinite(v) && v>=0 && v<=127; }
template<class Pose> inline bool pose_valid(const Pose& p) {
 return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.theta);
}
template<class Movement> inline bool movement_valid(const Movement& m) {
 return pose_valid(m.target) && speed_valid(m.max_xy_speed) &&
        (static_cast<int>(m.drive_direction)==0 || static_cast<int>(m.drive_direction)==1) &&
        static_cast<int>(m.turn_behavior)>=0 && static_cast<int>(m.turn_behavior)<=4;
}
template<class Path> inline bool path_valid(const Path& p,size_t minimum=1) {
 if(p.size()<minimum) return false;
 for(const auto& m:p) if(!movement_valid(m)) return false;
 return true;
}
// Permission is lifecycle-owned, not inferred from competition flags alone.
class DriverOutputPermission {
 std::atomic<bool> ready_{false};
 public:
 void revoke() { ready_.store(false); }
 void driver_ready(bool autonomous, bool disabled) { ready_.store(!autonomous && !disabled); }
 bool get() const { return ready_.load(); }
};
inline bool fault_blocks_output(Fault fault, bool driver_ready, bool native_disabled,
                               bool autonomous, bool disabled) {
 return fault!=Fault::None && !(driver_ready && native_disabled && !autonomous && !disabled);
}
// The existing native geometry functions are supplied by the caller. No hardware IO.
// Preserve their finite-input arithmetic; reject invalid intermediates before clamping.
template<class Pose, class Distance, class Project>
bool boomerang_geometry(const Pose& target, const Pose& current, double lead,
                        double maximum, double lookahead, int direction,
                        Distance distance_to_point, Project vector_off_point, Pose& carrot) {
 if(!pose_valid(target) || !pose_valid(current) || !std::isfinite(lead) || lead<0 || lead>1 ||
    !std::isfinite(maximum) || maximum<0 || !std::isfinite(lookahead) || lookahead<0 ||
    (direction!=1 && direction!=-1)) return false;
 const double distance=distance_to_point(target,current);
 if(!std::isfinite(distance) || distance<0) return false;
 double h=distance*lead;
 if(!std::isfinite(h)) return false;
 h=h>maximum?maximum:h;
 h*=direction;
 Pose temp=vector_off_point(-h,target);
 if(!pose_valid(temp)) return false;
 temp.theta=target.theta;
 if(distance<lookahead/2.0) temp=target;
 carrot=temp;
 return true;
}
inline int motor_boundary(int v) { return std::max(-127,std::min(127,v)); }
struct Mix { bool valid=false; double xy=0, angular=0, left=0, right=0, limit=0; };
// Same native vector scaling, with validation before arithmetic and int conversion.
inline Mix mix(double xy,double angular,double bias,bool use_bias,double left_slew,double right_slew,double requested) {
 Mix r;
 if (!speed_valid(requested) || !speed_valid(left_slew) || !speed_valid(right_slew) ||
     !std::isfinite(xy) || !std::isfinite(angular) || (use_bias && !std::isfinite(bias))) return r;
 r.limit=std::min(requested,std::max(left_slew,right_slew));
 xy=std::max(-r.limit,std::min(r.limit,xy));
 if(use_bias) xy*=bias;
 if(!std::isfinite(xy)) return r;
 double faster=std::max(std::abs(xy),std::abs(angular));
 if(faster>r.limit) { const double ratio=r.limit/faster; xy*=ratio; angular*=ratio; }
 r.xy=xy; r.angular=angular;
 r.left=xy+angular; r.right=xy-angular;
 if(!std::isfinite(r.left)||!std::isfinite(r.right)) return Mix{};
 faster=std::max(std::abs(r.left),std::abs(r.right));
 if(faster>r.limit) { const double ratio=r.limit/faster; r.left*=ratio; r.right*=ratio; }
 // Roundoff only: the recorded requested output is this actual bounded value.
 r.left=std::max(-r.limit,std::min(r.limit,r.left));
 r.right=std::max(-r.limit,std::min(r.limit,r.right));
 r.valid=true; return r;
}
}}
namespace ez {
void motion_output_fault(output_safety::Fault reason);
output_safety::Fault motion_output_fault_get();
}
