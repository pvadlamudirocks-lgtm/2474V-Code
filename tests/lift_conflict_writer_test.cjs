// Extract the actual diagnostic write batch; substitute fake hardware and approval flags.
const fs=require('fs'),cp=require('child_process'),os=require('os'),path=require('path');
const source=fs.readFileSync('diagnostics/lift_conflict/main.cpp','utf8');
const begin=source.indexOf('  auto write=[&]'),end=source.indexOf('  // Zero first.',begin);
if(begin<0||end<0)throw Error('Missing writer');
const dir=fs.mkdtempSync(path.join(os.tmpdir(),'lift-diag-writer-'));
const test=`#include <atomic>
#include <algorithm>
#include <cassert>
#include <cerrno>
#include <cstdio>
constexpr int PROS_ERR=2147483647,LIFT_DIAGNOSTIC_MAX_MV=1000;
bool POWERED_TEST_APPROVED=true;
std::atomic<unsigned> clearance_generation{3};
std::atomic<unsigned> mode_word{3};
namespace pros {namespace competition {bool connected=false,disabled=false,auton=false;bool is_connected(){return connected;}bool is_disabled(){return disabled;}bool is_autonomous(){return auton;}}}
int hook=-1,fail=-1,outputs[3]{},calls=0;bool fail_stop=false;
struct Motor{int n;int move_voltage(int mv){++calls;outputs[n]=mv;if(n==hook&&mv)mode_word.store(4);if((n==fail&&mv)||(fail_stop&&!mv)){errno=EIO;return PROS_ERR;}return 1;}};
struct Sample{int write_errno=0,activation_write_errno=0,cmd=0;bool write_ok=false;};
struct Frame{Sample m[3];bool all_write_ok=false,zero_ok=false;};
int main(){Motor motors[3]={{0},{1},{2}};Frame f;int signs[3]={-1,1,-1};
`+source.slice(begin,end)+`
assert(write(1000,3));assert(outputs[0]==-1000&&outputs[1]==1000&&outputs[2]==-1000);
for(int at=0;at<3;++at){mode_word=3;hook=at;assert(!write(1000,3));assert(write(0,4));assert(f.zero_ok&&outputs[0]==0&&outputs[1]==0&&outputs[2]==0);int old=calls;assert(!write(1000,3));assert(calls==old);}
hook=-1;mode_word=3;fail=1;assert(!write(1000,3));assert(f.m[1].activation_write_errno==EIO);assert(write(0,3));assert(f.zero_ok);fail=-1;
for(int n=0;n<5;++n)assert(write(0,3));
fail_stop=true;assert(!write(0,3));assert(!f.zero_ok);fail_stop=false;assert(write(0,3));
POWERED_TEST_APPROVED=false;int old=calls;assert(!write(1000,3)&&calls==old);assert(write(0,3));
POWERED_TEST_APPROVED=true;clearance_generation=~0u;assert(!write(-1000,3));assert(write(0,3));
clearance_generation=3;pros::competition::disabled=true;assert(!write(1000,3));assert(write(0,3));
std::puts("PASS extracted production writer: generation changes at each motor write, stale generation, polarity/cap, failed command, repeated zero acknowledgement, approval/reference gates, disable");}
`;
fs.writeFileSync(path.join(dir,'test.cpp'),test);
cp.execFileSync(process.argv[2],['c++','-std=c++17','-O1',path.join(dir,'test.cpp'),'-o',path.join(dir,'test.exe')],{stdio:'inherit'});
cp.execFileSync(path.join(dir,'test.exe'),[],{stdio:'inherit'});
