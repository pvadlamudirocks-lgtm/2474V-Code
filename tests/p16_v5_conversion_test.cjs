// Extract the actual shared converter; simulate readings, not physical motor behavior.
const fs=require('fs'),cp=require('child_process'),os=require('os'),path=require('path');
const source=fs.readFileSync('src/subsystems.cpp','utf8'),header=fs.readFileSync('include/subsystems.hpp','utf8');
function constant(name){const m=header.match(new RegExp('constexpr (?:double|int) '+name+' = ([^;]+);'));if(!m)throw Error(name);return m[1];}
const a=source.indexOf('double read_position(bool is_lift) {'),b=source.indexOf('\n}',a)+2;
function extractFunction(name){const a=source.indexOf('bool '+name+'('),start=source.indexOf('{',a);let n=1,b=start+1;for(;n&&b<source.length;++b){if(source[b]=='{')++n;if(source[b]=='}')--n;}return source.slice(a,b);}
const calibration=fs.readFileSync('include/lift_calibration.hpp','utf8');
const max=calibration.match(/MAX_SAFE_DEG = ([^;]+);/)[1];
const predicate=source.match(/native_config_ok = (result == 1[^;]+);/)[1];
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
constexpr bool ENABLE_LIFT_CALIBRATION_MODE=false;
}
bool p16_native_ready=false;std::int32_t p16_last_raw=PROS_ERR;int p16_last_read_errno=0;
struct Sensor{int raw=0,error=0,reads=0;int get_position(){++reads;errno=error;return raw;}} sensor;
Sensor& lift_sensor(){return sensor;}Sensor& arm_sensor(){return sensor;}
${source.slice(a,b)}
struct {bool initialized=true,homed=true;}lift;
enum class LiftOutputOwner{None,Driver,Autonomous};LiftOutputOwner lift_output_owner=LiftOutputOwner::Driver;
enum class LiftBottomLimit{Unconfigured,Pressed,Released,Fault};
LiftBottomLimit lift_bottom_limit_read(){return LiftBottomLimit::Unconfigured;}
namespace pros {namespace competition {bool is_connected(){return false;}}}
int last_mv=99999;bool write_lift_pair(int mv){last_mv=mv;return true;}
void trace_lift_path(unsigned,const char*,int,double=NAN){}
struct Arm{int move_voltage(int){return 1;}}arm;Arm& arm_motor(){return arm;}
${extractFunction('write_voltage')}
bool config(int result,int set_error,int actual,int get_error){return ${predicate};}
int main(){
 assert(config(1,0,0,0));assert(!config(1,0,1,0));assert(!config(PROS_ERR,5,0,0));assert(!config(1,0,PROS_ERR,5));assert(!config(1,5,0,0));
 assert(std::isnan(read_position(true))&&sensor.reads==0);
 p16_native_ready=true;sensor.raw=0;assert(read_position(true)==0);
 sensor.raw=7005;assert(std::fabs(read_position(true)-70.05)<1e-9);
 sensor.raw=15205;assert(std::fabs(read_position(true)-152.05)<1e-9);
 sensor.raw=161771;assert(std::fabs(read_position(true)-1617.71)<1e-9);assert(read_position(true)>2&&read_position(true)<3000);
 sensor.raw=3850;assert(read_position(true)==38.5);sensor.raw=200;assert(read_position(true)==2);sensor.raw=300000;assert(read_position(true)==3000);
 sensor.raw=10000;double raised=read_position(true);sensor.raw=5000;assert(read_position(true)<raised&&read_position(true)>0);
 sensor.raw=PROS_ERR;assert(std::isnan(read_position(true)));
 sensor.raw=5000;sensor.error=5;assert(std::isnan(read_position(true))&&p16_last_read_errno==5);
 sensor.error=0;assert(read_position(true)==50);assert(std::isnan(read_position(false)));
 for(int raw:{0,199,200}){sensor.raw=raw;assert(write_voltage(true,-12000)&&last_mv==0);}
 sensor.raw=201;assert(write_voltage(true,-12000)&&last_mv==-12000);
 sensor.raw=15205;assert(write_voltage(true,-12000)&&last_mv==-12000);
 sensor.raw=299999;assert(write_voltage(true,12000)&&last_mv==12000);
 for(int raw:{300000,300001}){sensor.raw=raw;assert(write_voltage(true,12000)&&last_mv==0);assert(write_voltage(true,-12000)&&last_mv==-12000);}
 sensor.raw=-7005;assert(read_position(true)<0);assert(write_voltage(true,-12000)&&last_mv==0);
 sensor.raw=PROS_ERR;assert(!write_voltage(true,-12000)&&last_mv==0);
 p16_native_ready=false;assert(!write_voltage(true,-12000)&&last_mv==0);
 puts("PASS actual V6 converter and native-config predicate: sign, zero, 38.5/2/3000 boundaries AND actual voltage-writer cutoff behavior, lowering, read errors, config mismatch");
}`;
const dir=fs.mkdtempSync(path.join(os.tmpdir(),'p16-v6-'));fs.writeFileSync(path.join(dir,'test.cpp'),code);
cp.execFileSync(process.argv[2],['c++','-std=c++17','-O1','-UNDEBUG',path.join(dir,'test.cpp'),'-o',path.join(dir,'test.exe')],{stdio:'inherit'});
cp.execFileSync(path.join(dir,'test.exe'),[],{stdio:'inherit'});
