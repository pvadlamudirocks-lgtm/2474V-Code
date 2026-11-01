#include "EZ-Template/movement_limits.hpp"
#include <limits>
static_assert(!ez::movement_expired(0,0));
static_assert(!ez::movement_expired(7999,0));
static_assert(ez::movement_expired(8000,0));
static_assert(ez::movement_expired(8001,0));
static_assert(!ez::movement_expired(8499,500));
static_assert(ez::movement_expired(8500,500));
constexpr unsigned before_wrap=std::numeric_limits<unsigned>::max()-3999;
static_assert(!ez::movement_expired(3999,before_wrap));
static_assert(ez::movement_expired(4000,before_wrap));
static_assert(ez::movement_lock_ms < ez::movement_stop_ms);
static_assert(ez::movement_stop_ms < ez::movement_timeout_ms);
