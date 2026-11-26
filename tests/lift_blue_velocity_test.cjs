// Extract the actual shared converter; simulate readings, not physical motor behavior.
const fs=require('fs'),cp=require('child_process'),os=require('os'),path=require('path');
const source=fs.readFileSync('src/subsystems.cpp','utf8'),header=fs.readFileSync('include/subsystems.hpp','utf8');
function constant(name){const m=header.match(new RegExp('constexpr (?:double|int) '+name+' = ([^;]+);'));if(!m)throw Error(name);return m[1];}
const a=source.indexOf('double read_position(bool is_lift) {'),b=source.indexOf('\n}',a)+2;
function extractFunction(name){const a=source.indexOf('bool '+name+'('),start=source.indexOf('{',a);let n=1,b=start+1;for(;n&&b<source.length;++b){if(source[b]=='{')++n;if(source[b]=='}')--n;}return source.slice(a,b);}
const calibration=fs.readFileSync('include/lift_calibration.hpp','utf8');
const max=calibration.match(/MAX_SAFE_DEG = ([^;]+);/)[1];
const predicate=source.match(/native_config_ok = (result == 1[^;]+);/)[1];
const assert=require('assert');
assert(source.includes('owner == LiftOutputOwner::Driver && direction < 0'));
assert(source.includes('lift_full_direction != direction'));
const code=`#include <cmath>
#include <limits>
#include <cstdint>
#include <cerrno>
#undef NDEBUG
#include <cassert>
#include <cstdio>
#include <initializer_list>
constexpr int PROS_ERR=2147483647,ARM_ROTATION_PORT=0;
constexpr double LIFT_SENSOR_DEG_PER_MECHANISM_DEG=${constant('LIFT_SENSOR_DEG_PER_MECHANISM_DEG')},ARM_SENSOR_DEG_PER_MECHANISM_DEG=1;
namespace mechanism_config {
constexpr double LIFT_P16_SOFTWARE_SIGN=${constant('LIFT_P16_SOFTWARE_SIGN')};
constexpr double LIFT_MAX_DEG=${max},LIFT_LOWER_STOP_MARGIN_DEG=${constant('LIFT_LOWER_STOP_MARGIN_DEG')};
constexpr bool ENABLE_LIFT_CALIBRATION_MODE=false;constexpr unsigned MANUAL_COMMAND_TIMEOUT_MS=100;
}
bool p16_native_ready=false;std::int32_t p16_last_raw=PROS_ERR;int p16_last_read_errno=0;
struct Sensor{int raw=0,error=0,reads=0;int get_position(){++reads;errno=error;return raw;}} sensor;
Sensor& lift_sensor(){return sensor;}Sensor& arm_sensor(){return sensor;}
${source.slice(a,b)}
enum class AxisMode{Disabled,Manual,Hold,MoveToTarget,Fault};
struct AxisState {bool initialized=true,homed=true,preload_crossing=false,settling=false,settled=false;int manual_mv=0;AxisMode mode=AxisMode::Disabled;}lift;
bool lift_response_running=false,returning_home=false;
int lift_full_direction=0;unsigned lift_full_last_refresh=0;
constexpr int LIFT_COMMISSIONING_MV=12000;
constexpr bool LIFT_LIMITS_CALIBRATED=true;
enum class LiftOutputOwner{None,Driver,Autonomous};LiftOutputOwner lift_output_owner=LiftOutputOwner::Driver;
enum class LiftBottomLimit{Unconfigured,Pressed,Released,Fault};
LiftBottomLimit lift_bottom_limit_read(){return LiftBottomLimit::Unconfigured;}
namespace pros {namespace v5 {enum class MotorGears {blue};}
unsigned clock=100;unsigned millis(){return clock;}
namespace competition {bool disabled=false,autonomous=false;bool is_disabled(){return disabled;}bool is_autonomous(){return autonomous;}bool is_connected(){return false;}}
struct Motor {int port;Motor(int p,v5::MotorGears):port(p){};bool is_installed();int move_voltage(int);int move_velocity(int);};}
int command[22]{},mode[22]{},failure_port=0,disconnected=0;
bool pros::Motor::is_installed(){return port!=disconnected;}
int pros::Motor::move_voltage(int n){command[port]=n;mode[port]=0;return 1;}
int pros::Motor::move_velocity(int n){command[port]=n;mode[port]=1;return port==failure_port?PROS_ERR:1;}
pros::Motor p13(13,pros::v5::MotorGears::blue),p14(14,pros::v5::MotorGears::blue),p15(15,pros::v5::MotorGears::blue);
pros::Motor& left_lift_motor(){return p14;}pros::Motor& right_lift_motor(){return p15;}pros::Motor& third_lift_motor(){return p13;}
constexpr int THIRD_LIFT_PORT=13,LEFT_LIFT_PORT=14,RIGHT_LIFT_PORT=15;
constexpr bool THIRD_LIFT_REVERSED=true,LEFT_LIFT_REVERSED=false,RIGHT_LIFT_REVERSED=true;
constexpr int LIFT_DOWN_TARGET_RPM=${constant('LIFT_DOWN_TARGET_RPM')};
bool LIFT_MOTOR_SAFETY_LOCKOUT=false,ENABLE_LIFT_DIRECTION_DIAGNOSTIC=false,lift_output_fault=false;
namespace lift_calibration {bool enable_limits=false;int limits=0;}
namespace auton_math {bool lift_direction_allowed(int,double,int){return true;}}
void trace_lift_path(unsigned,const char*,int,double=NAN){}
struct Arm{int move_voltage(int){return 1;}}arm;Arm& arm_motor(){return arm;}
void trace_lift_write(int,long,long,long,int,int,int,bool=false,bool=false){}
${extractFunction('write_lift_pair')}
${extractFunction('write_voltage')}
${extractFunction('disable_axis')}
bool lift_auton_stop(){return disable_axis(lift,true,lift.mode==AxisMode::Fault);}
bool reject_lift_lockout(){if(!LIFT_MOTOR_SAFETY_LOCKOUT)return false;lift_auton_stop();return true;}
using namespace mechanism_config;
${extractFunction('lift_full_command')}
${source.slice(source.indexOf('void lift_update() {'),source.indexOf('  if (!returning_home) { update_axis'))}}

bool driver_buttons(bool up_held,bool down_held) {
${extractFunction('test_lift_motor_response').slice(extractFunction('test_lift_motor_response').indexOf('// Normal match control'))}
bool config(int result,int set_error,int actual,int get_error){return ${predicate};}
int main(){
 p16_native_ready=true;sensor.raw=10000;
 assert(write_voltage(true,-12000,true));
 assert(command[13]==120&&command[14]==-120&&command[15]==120);
 assert(mode[13]==1&&mode[14]==1&&mode[15]==1);
 assert(write_voltage(true,0));for(int p:{13,14,15})assert(command[p]==0&&mode[p]==0);
 assert(write_voltage(true,12000));assert(command[13]==-12000&&command[14]==12000&&command[15]==-12000);
 assert(write_voltage(true,-12000));assert(command[13]==12000&&command[14]==-12000&&command[15]==12000); // Autonomous unchanged.
 for(int raw:{0,199,200}){sensor.raw=raw;assert(write_voltage(true,-12000,true));for(int p:{13,14,15})assert(command[p]==0&&mode[p]==0);}
 sensor.raw=201;assert(write_voltage(true,-12000,true)&&mode[13]==1);
 sensor.raw=PROS_ERR;assert(!write_voltage(true,-12000,true));for(int p:{13,14,15})assert(command[p]==0&&mode[p]==0);
 sensor.raw=10000;
 for(int p:{13,14,15}){failure_port=p;lift_output_fault=false;assert(!write_voltage(true,-12000,true)&&lift_output_fault);for(int q:{13,14,15})assert(command[q]==0&&mode[q]==0);}
 failure_port=0;disconnected=13;assert(!write_voltage(true,-12000,true));for(int p:{13,14,15})assert(command[p]==0&&mode[p]==0);
 disconnected=0;LIFT_MOTOR_SAFETY_LOCKOUT=true;assert(!write_voltage(true,-12000,true));for(int p:{13,14,15})assert(command[p]==0&&mode[p]==0);
 LIFT_MOTOR_SAFETY_LOCKOUT=false;lift_output_fault=false;
 lift.initialized=lift.homed=true;lift.mode=AxisMode::Disabled;
 assert(lift_full_command(-1,LiftOutputOwner::Driver)&&mode[13]==1);
 assert(lift_full_command(1,LiftOutputOwner::Driver)&&mode[13]==0&&command[13]==-12000);
 assert(lift_full_command(-1,LiftOutputOwner::Driver)&&mode[13]==1);
 assert(driver_buttons(false,false));for(int p:{13,14,15})assert(command[p]==0&&mode[p]==0);
 assert(driver_buttons(false,true));assert(driver_buttons(true,true));for(int p:{13,14,15})assert(command[p]==0&&mode[p]==0);
 assert(driver_buttons(false,true));
 pros::clock+=101;lift_update();for(int p:{13,14,15})assert(command[p]==0&&mode[p]==0);
 assert(lift_full_command(-1,LiftOutputOwner::Driver));pros::competition::disabled=true;lift_update();assert(command[13]==0&&lift_output_owner==LiftOutputOwner::None);
 pros::competition::disabled=false;assert(lift_full_command(-1,LiftOutputOwner::Driver));pros::competition::autonomous=true;lift_update();assert(command[13]==0);
 assert(lift_full_command(-1,LiftOutputOwner::Autonomous)&&command[13]==12000&&mode[13]==0);
 lift_auton_stop();pros::competition::autonomous=false;assert(lift_full_command(-1,LiftOutputOwner::Driver));sensor.raw=200;lift_update();assert(command[13]==0);
 sensor.raw=10000;assert(lift_full_command(-1,LiftOutputOwner::Driver));sensor.raw=PROS_ERR;lift_update();assert(command[13]==0&&lift.mode==AxisMode::Fault);
 puts("PASS owner lifecycle: driver-only RPM, direction switch, watchdog, disabled, competition transition, lower cutoff, sensor fault");
 puts("PASS actual blue lift writer: RPM signs, voltage/RPM transitions, zero cancels velocity, P16 cutoff/invalid, each write failure, disconnection, lockout; autonomous voltage unchanged");

}`;
const dir=fs.mkdtempSync(path.join(os.tmpdir(),'p16-v6-'));fs.writeFileSync(path.join(dir,'test.cpp'),code);
cp.execFileSync(process.argv[2],['c++','-std=c++17','-O1','-UNDEBUG',path.join(dir,'test.cpp'),'-o',path.join(dir,'test.exe')],{stdio:'inherit'});
cp.execFileSync(path.join(dir,'test.exe'),[],{stdio:'inherit'});
