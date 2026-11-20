// Compile verbatim production function bodies with fake motors/competition services.
// This tests call ordering and stop behavior, not RTOS scheduling or actual devices.
const fs=require('fs'),cp=require('child_process'),os=require('os'),path=require('path');
const zig=process.argv[2];if(!zig)throw Error('Supply zig.exe path');
function body(file,signature){const s=fs.readFileSync(file,'utf8');const a=s.indexOf(signature);if(a<0)throw Error(signature);let i=s.indexOf('{',a),n=1;for(++i;n&&i<s.length;++i){if(s[i]==='{')++n;if(s[i]==='}')--n;}if(n)throw Error('Unclosed body');return s.slice(a,i);}
const task='patches/EZ-Template/pid_tasks.cpp',drive='patches/EZ-Template/drive.cpp';
const functions=[body(task,'void ez::motion_output_fault('),body(task,'ez::output_safety::Fault ez::motion_output_fault_get('),body(task,'bool ez::driver_output_authorized('),body(task,'void ez::driver_control_ready('),body(task,'void ez::autonomous_begin('),body(task,'bool ez::autonomous_cancelled()'),body(drive,'void Drive::private_drive_set('),body(drive,'void Drive::drive_set('),body(task,'void Drive::boomerang_task()'),body('patches/EZ-Template/exit_conditions.cpp','bool ez::wait_service_tick(')];
const pre=String.raw`
#include "EZ-Template/motion_output_safety.hpp"
#include <vector>
#include <cassert>
#include <cstdio>
namespace pros {
namespace competition { bool automatic=false,disabled=false; bool is_autonomous(){return automatic;} bool is_disabled(){return disabled;} }
unsigned millis(){return 2000;}
constexpr int E_CONTROLLER_MASTER=0,E_CONTROLLER_DIGITAL_B=1;
struct Controller { Controller(int){} int get_digital(int){return 0;} };
}
namespace ez {
enum { DISABLE, PURE_PURSUIT, POINT_TO_POINT, FWD=0, REV=1 };
struct pose {double x=0,y=0,theta=0;};
struct odom {pose target;int drive_direction,max_xy_speed;};
namespace util {
double distance_to_point(pose a,pose b){return std::sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y));}
pose vector_off_point(double d,pose p){double r=p.theta*3.14159265358979323846/180.;return {p.x+std::sin(r)*d,p.y+std::cos(r)*d,p.theta};}
}
struct FakeMotor {int* output;void move_voltage(int v){*output=v;}};
struct FakeSlew {bool enabled(){return false;}};
class Drive {
public:
 int mode=PURE_PURSUIT,max_speed=70,pp_index=0,current_drive_direction=FWD;
 int voltage[4]={1,1,1,1},ptp_calls=0,dispatches=0;
 std::vector<FakeMotor> left_motors{{&voltage[0]},{&voltage[1]}},right_motors{{&voltage[2]},{&voltage[3]}};
 std::vector<odom> pp_movements{{{19.94,6.93,56.83},FWD,70}};
 pose odom_target{},current{};double max_boomerang_distance=16,lead=.50,lookahead=7;
 FakeSlew slew_left,slew_right;
 bool pto_check(FakeMotor){return false;}
 void drive_mode_set(int v,bool){mode=v;}
 pose odom_pose_get(){return current;}
 double odom_boomerang_dlead_get(){return lead;}
 double odom_look_ahead_get(){return lookahead;}
 void raw_pid_odom_ptp_set(odom m,bool){++dispatches;odom_target=m.target;}
 void ptp_task(){++ptp_calls;}
 void private_drive_set(int,int);void drive_set(int,int);void boomerang_task();
};
output_safety::Latch output_fault;
output_safety::DriverOutputPermission driver_output_permission;
std::atomic<bool> emergency_stop{false};
bool driver_output_authorized();void driver_control_ready();void autonomous_begin();bool autonomous_cancelled();
using WaitService=bool(*)(Drive*);std::atomic<WaitService> wait_service{nullptr};
bool geometry_turn_deadline_stopped(Drive*){return false;}bool wait_service_tick(Drive*);
}
using namespace ez;
`;
const post=String.raw`
static void stopped(Drive& d){assert(d.mode==DISABLE);for(auto v:d.voltage)assert(v==0);}
int main(){
 Drive d;
 driver_control_ready();assert(driver_output_authorized());
 motion_output_fault(output_safety::Fault::Output);
 assert(!driver_output_authorized());d.drive_set(70,70);stopped(d);
 assert(!wait_service_tick(&d)); // actual native wait-service function sees actual fault publisher
 d.drive_set(0,0);driver_control_ready();d.drive_set(70,-70);
 assert(d.voltage[0]==int(70*(12000.0/127.0))&&d.voltage[2]==int(-70*(12000.0/127.0)));
 assert(motion_output_fault_get()==output_safety::Fault::Output);
 pros::competition::automatic=true;d.drive_set(70,70);stopped(d);
 pros::competition::automatic=false;pros::competition::disabled=true;d.drive_set(70,70);stopped(d);
 pros::competition::disabled=false;autonomous_begin();assert(!driver_output_authorized());
 pros::competition::automatic=true;d.mode=PURE_PURSUIT;d.boomerang_task();assert(d.ptp_calls==1&&d.dispatches==1);
 const double bad=std::numeric_limits<double>::infinity();
 for(int which=0;which<4;++which){Drive broken;autonomous_begin();
  if(which==0)broken.lead=bad;if(which==1)broken.current.x=1e308;
  if(which==2)broken.pp_movements[0].target.theta=1e308;if(which==3)broken.lookahead=bad;
  broken.boomerang_task();stopped(broken);assert(broken.ptp_calls==0);
  assert(motion_output_fault_get()==output_safety::Fault::State);assert(!wait_service_tick(&broken));
  broken.drive_set(127,127);stopped(broken);
 }
 std::puts("PASS verbatim native functions: manual-auton fault suppression, driver cleanup handoff, owner geometry stop, and native wait-service cancellation");
}
`;
const dir=fs.mkdtempSync(path.join(os.tmpdir(),'2474v-c23-harness-'));const file=path.join(dir,'test.cpp'),exe=path.join(dir,'test.exe');fs.writeFileSync(file,pre+'\n'+functions.join('\n')+'\n'+post);
cp.execFileSync(zig,['c++','-std=c++17','-O1','-I'+path.resolve('include'),file,'-o',exe],{stdio:'inherit'});cp.execFileSync(exe,[],{stdio:'inherit'});
