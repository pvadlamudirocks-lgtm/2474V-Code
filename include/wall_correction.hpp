#pragma once
#include "auton_route.hpp"
namespace wall_correction {
inline constexpr bool enable_application = false;
inline constexpr std::array<auton_math::Sensor,4> sensors{};
bool available(const auton_math::Sensor& sensor);
void health_snapshot();
auton::Result apply(auton::Session& session, unsigned sensor_index, const auton_math::Wall& wall,
                   double stationary_rpm, unsigned stationary_ms, unsigned timeout_ms);
}
