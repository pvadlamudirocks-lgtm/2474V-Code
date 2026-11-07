#include "full_system_diag_limits.hpp"
#include <limits>
using namespace full_diag;
static_assert(!expired(1999,0,health_ms));
static_assert(expired(2000,0,health_ms));
static_assert(!expired(7999,0,manual_ms));
static_assert(expired(8000,0,manual_ms));
constexpr unsigned before_wrap=std::numeric_limits<unsigned>::max()-99;
static_assert(!expired(1899,before_wrap,health_ms));
static_assert(expired(1900,before_wrap,health_ms));
static_assert(manual_ms/passive_sample_ms+1<=200);
static_assert(health_ms/passive_sample_ms+1<=200);
static_assert(trace_max_rows==8);
static_assert(trace_index(0,160,8)==0);
static_assert(trace_index(7,160,8)==159);
static_assert(trace_index(0,1,1)==0);
static_assert(trace_index(3,4,4)==3);
static_assert(trace_print_interval_ms>=100);
