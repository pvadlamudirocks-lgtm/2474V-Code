#pragma once
#include "stability_math.hpp"
#include <array>
namespace lift_height_math {
// VEX device operating envelope, not a PROS error sentinel or calibrated lift limit.
inline constexpr int DEVICE_MIN_MM = 20, DEVICE_MAX_MM = 2000;
constexpr bool valid_distance(int mm) { return mm >= DEVICE_MIN_MM && mm <= DEVICE_MAX_MM; }
struct Median {
  std::array<double, 3> values{};
  unsigned used = 0, next_index = 0;
  constexpr void reset() { used = next_index = 0; }
  constexpr double add(double mm) {
    if (!stability::finite(mm)) { reset(); return stability::invalid; }
    values[next_index] = mm; next_index = (next_index + 1) % 3;
    if (used < 3) ++used;
    auto sorted = values;
    for (unsigned i = 1; i < used; ++i)
      for (unsigned j = i; j > 0 && sorted[j] < sorted[j-1]; --j) {
        const auto tmp = sorted[j]; sorted[j] = sorted[j-1]; sorted[j-1] = tmp;
      }
    return used == 2 ? (sorted[0] + sorted[1]) / 2 : sorted[used/2];
  }
};
}
