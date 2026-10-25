#pragma once
#include "auton_route.hpp"
// Central measured route data. Keep these empty until field recording supplies it.
// Stage callback assignments belong here; callback bodies remain explicit route code.
namespace route_config {
inline const auton::RouteConfig left{"left"};
inline const auton::RouteConfig right{"right"};
inline const auton::RouteConfig safe{"safe"};
inline const auton::RouteConfig skills{"skills"};
// Choose which route the independent harness tests, without selector fallthrough.
inline const auton::RouteConfig& segment_route = left;
}
