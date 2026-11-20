#include "EZ-Template/motion_output_safety.hpp"
#include <cassert>
#include <cstdio>
#include <random>
using namespace ez::output_safety;
struct Pose { double x,y,theta; };
static double distance(Pose a,Pose b) { return std::sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)); }
static Pose project(double d,Pose p) {
 const double r=p.theta*3.14159265358979323846/180.0;
 return {p.x+std::sin(r)*d,p.y+std::cos(r)*d,p.theta};
}
static Pose previous(Pose t,Pose c,double lead,double maximum,double look,int dir) {
 double h=distance(t,c)*lead;h=h>maximum?maximum:h;h*=dir;
 Pose temp=project(-h,t);temp.theta=t.theta;
 if(distance(t,c)<look/2.0)temp=t;return temp;
}
int main() {
 DriverOutputPermission permission; assert(!permission.get());
 for(int fault=0;fault<=4;++fault)for(int ready=0;ready<2;++ready)
 for(int native=0;native<2;++native)for(int automatic=0;automatic<2;++automatic)for(int disabled=0;disabled<2;++disabled) {
  const bool blocked=fault_blocks_output(static_cast<Fault>(fault),ready,native,automatic,disabled);
  assert(blocked==(fault!=0 && !(ready&&native&&!automatic&&!disabled)));
  for(int requested:{-127,-70,0,70,127}) {
   const int applied=blocked?0:motor_boundary(requested);
   if(blocked)assert(applied==0);if(requested==0)assert(applied==0);
  }
 }
 Latch fault;
 permission.driver_ready(false,false);assert(permission.get());
 permission.revoke(); // autonomous entry, even with driver competition flags
 fault.set(Fault::Output);
 assert(fault_blocks_output(fault.get(),permission.get(),true,false,false));
 // Cleanup without explicit driver takeover must not authorize output.
 assert(!permission.get());
 permission.driver_ready(false,false);assert(!fault_blocks_output(fault.get(),permission.get(),true,false,false));
 permission.revoke(); // fault publication also revokes a prior driver authorization
 assert(fault_blocks_output(fault.get(),permission.get(),true,false,false));
 permission.driver_ready(true,false);assert(!permission.get());
 permission.driver_ready(false,true);assert(!permission.get());
 permission.driver_ready(false,false);assert(permission.get());
 assert(fault.get()==Fault::Output); // driver takeover does not clear fault evidence
 Pose target{19.94,6.93,56.83},start{0,0,0},carrot{};
 assert(boomerang_geometry(target,start,.50,16.,7.,1,distance,project,carrot));
 const auto expected=previous(target,start,.50,16.,7.,1);
 assert(carrot.x==expected.x&&carrot.y==expected.y&&carrot.theta==expected.theta);
 std::mt19937 random(2474);std::uniform_real_distribution<double> xy(-144,144),heading(-720,720),lead(0,1),limit(0,30),look(0,20);
 for(int i=0;i<100000;++i){Pose t{xy(random),xy(random),heading(random)},c{xy(random),xy(random),heading(random)};
  const double l=lead(random),m=limit(random),k=look(random);const int dir=i%2?1:-1;
  assert(boomerang_geometry(t,c,l,m,k,dir,distance,project,carrot));auto old=previous(t,c,l,m,k,dir);
  assert(carrot.x==old.x&&carrot.y==old.y&&carrot.theta==old.theta);
 }
 const double inf=std::numeric_limits<double>::infinity(),nan=std::numeric_limits<double>::quiet_NaN();
 for(double bad:{inf,-inf,nan}) {
  for(int field=0;field<9;++field){auto t=target,c=start;double l=.5,m=16,k=7;
   if(field==0)t.x=bad;if(field==1)t.y=bad;if(field==2)t.theta=bad;
   if(field==3)c.x=bad;if(field==4)c.y=bad;if(field==5)c.theta=bad;
   if(field==6)l=bad;if(field==7)m=bad;if(field==8)k=bad;
   int projected=0;
   assert(!boomerang_geometry(t,c,l,m,k,1,distance,[&](double d,Pose p){++projected;return project(d,p);},carrot));assert(projected==0);
  }
  assert(!boomerang_geometry(target,start,.5,16.,7.,1,[=](Pose,Pose){return bad;},project,carrot));
  assert(!boomerang_geometry(target,start,.5,16.,7.,1,distance,[=](double,Pose p){p.x=bad;return p;},carrot));
 }
 assert(!boomerang_geometry(Pose{1e308,1e308,0},start,.5,16.,7.,1,distance,project,carrot));
 assert(!boomerang_geometry(Pose{0,0,1e308},start,.5,16.,7.,1,distance,project,carrot)); // projection invalid, even inside near-target override
 for(double l:{-.1,1.1})assert(!boomerang_geometry(target,start,l,16.,7.,1,distance,project,carrot));
 assert(!boomerang_geometry(target,start,.5,-1.,7.,1,distance,project,carrot));
 assert(!boomerang_geometry(target,start,.5,16.,-1.,1,distance,project,carrot));
 assert(!boomerang_geometry(target,start,.5,16.,7.,0,distance,project,carrot));
 for(int dir:{-1,1})for(double l:{0.,.5,1.})for(double d:{0.,3.499,3.5,3.501,32.,32.001}) {
  Pose t{0,d,0};assert(boomerang_geometry(t,start,l,16.,7.,dir,distance,project,carrot));auto old=previous(t,start,l,16.,7.,dir);
  assert(carrot.x==old.x&&carrot.y==old.y&&carrot.theta==old.theta);
 }
 std::puts("PASS corrections 2/3: 80 gate states; manual-auton/driver lifecycle; zero commands; invalid geometry; 100000 exact finite-geometry regressions");
}
