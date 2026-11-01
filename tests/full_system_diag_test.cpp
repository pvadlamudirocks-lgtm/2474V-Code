#include "full_system_diag_limits.hpp"
#include <limits>
using namespace full_diag;
static_assert(profile==Profile::PASSIVE);
static_assert(!motor_bench_physically_verified && !controlled_motion_enabled);
static_assert(!power_allowed(Profile::PASSIVE,false,false));
static_assert(!power_allowed(Profile::PASSIVE,true,true));
static_assert(power_allowed(Profile::MOTOR_BENCH,false,false));
static_assert(!power_allowed(Profile::CONTROLLED_MOTION,false,false));
static_assert(!power_allowed(Profile::CONTROLLED_MOTION,false,true));
static_assert(!power_allowed(Profile::CONTROLLED_MOTION,true,false));
static_assert(power_allowed(Profile::CONTROLLED_MOTION,true,true));
static_assert(!power_allowed(static_cast<Profile>(99),true,true));
static_assert(manual_ms/passive_sample_ms+1<=200);
static_assert(health_ms/passive_sample_ms+1<=200);
// Compile-time checks exercise the exact predicates used by the drive-owner
// callback. They do not simulate PROS scheduling, motors or sensor drivers.
static_assert(!expired(249,0,pulse_ms));
static_assert(expired(250,0,pulse_ms));
static_assert(!expired(1499,0,motion_ms));
static_assert(expired(1500,0,motion_ms));
static_assert(expired(1501,0,motion_ms));
constexpr unsigned wrap_start=std::numeric_limits<unsigned>::max()-99;
static_assert(!expired(149,wrap_start,pulse_ms));
static_assert(expired(150,wrap_start,pulse_ms));
static_assert(!wrong_turn(15,15));
static_assert(!wrong_turn(-15,-15));
static_assert(wrong_turn(-4,15));
static_assert(wrong_turn(4,-15));
static_assert(!wrong_turn(-3,15));
static_assert(!excessive(0,0));
static_assert(!excessive(30,6));
static_assert(excessive(30.01,0));
static_assert(excessive(-30.01,0));
static_assert(excessive(0,6.01));
static_assert(excessive(std::numeric_limits<double>::quiet_NaN(),0));
static_assert(excessive(0,std::numeric_limits<double>::infinity()));
static_assert(motion_ms/20+1<=100); // Fixed capture capacity at nominal cadence.
