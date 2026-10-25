#include "stability_math.hpp"
#include "telemetry_format.hpp"
#include "lift_height_math.hpp"

// Synthetic fixtures exercise software only; NONE are robot tuning values.
constexpr stability::Settings fixture{true, 5, 10, 20, 3, 6, 15, 2, .1, 2, .5, 12};
using stability::State;
static_assert(stability::transform(12, -1) == -12);
static_assert(stability::transform(-12, 1) == -12);
static_assert(!stability::finite(stability::transform(12, 0)));
static_assert(!stability::finite(stability::transform(stability::invalid, 1)));
static_assert(!stability::finite(stability::transform(std::numeric_limits<double>::infinity(), 1)));
static_assert(stability::valid(fixture));
static_assert(!stability::valid(stability::Settings{}));
static_assert(stability::classify(State::STABLE, {0,0,0,0}, fixture) == State::STABLE);
static_assert(stability::classify(State::STABLE, {6,0,0,0}, fixture) == State::WARNING);
static_assert(stability::classify(State::STABLE, {10,0,0,0}, fixture) == State::RECOVERY);
static_assert(stability::classify(State::RECOVERY, {4,0,0,0}, fixture) == State::RECOVERY);
static_assert(stability::classify(State::RECOVERY, {3,0,0,0}, fixture) == State::STABLE);
static_assert(stability::classify(State::WARNING, {4,0,0,0}, fixture) == State::WARNING);
static_assert(stability::classify(State::STABLE, {8,0,30,0}, fixture) == State::RECOVERY);
static_assert(stability::classify(State::STABLE, {8,0,-30,0}, fixture) == State::WARNING);
static_assert(stability::classify(State::STABLE, {0,16,0,0}, fixture) == State::SEVERE);
static_assert(stability::classify(State::SEVERE, {0,7,0,0}, fixture) == State::SEVERE);
static_assert(stability::classify(State::SEVERE, {0,4,0,0}, fixture) == State::WARNING);
static_assert(stability::classify(State::RECOVERY, {stability::invalid,0,0,0}, fixture) == State::DISABLED);
static_assert(stability::classify(State::STABLE, {0,0,0,0}, {}) == State::DISABLED);
static_assert(stability::correction(State::RECOVERY, {10,0,30,0}, fixture) == 12);
static_assert(stability::correction(State::RECOVERY, {-10,0,-30,0}, fixture) == -12);
static_assert(stability::correction(State::RECOVERY, {10,0,stability::invalid,0}, fixture) == 0);
static_assert(stability::correction(State::STABLE, {0,0,0,0}, fixture) == 0);
static_assert(stability::propose_driver_command({125,42}, State::RECOVERY, {10,0,30,0}, fixture).forward == 127);
static_assert(stability::propose_driver_command({125,42}, State::RECOVERY, {10,0,30,0}, fixture).turn == 42);
static_assert(stability::propose_driver_command({30,42}, State::SEVERE, {0,16,0,0}, fixture).turn == 0);
constexpr bool rates() {
  stability::RateFilter filter;
  if (stability::finite(filter.update(0,.05))) return false; // Warmup cannot command.
  if (filter.update(0,.05) != 0) return false;
  if (filter.update(5,.05) <= 0) return false;
  if (stability::finite(filter.update(5,.3))) return false; // Gap invalidates.
  if (stability::finite(filter.update(5,.05))) return false; // Rewarm after gap.
  filter.reset(); filter.update(179,.05);
  if (filter.update(-179,.05) <= 0 || filter.rate > 40) return false; // Wrap-safe.
  return !stability::finite(filter.update(stability::invalid,.05));
}
static_assert(rates());
constexpr bool median() {
  lift_height_math::Median f;
  if (f.add(400) != 400 || f.add(402) != 401 || f.add(1800) != 402) return false;
  if (f.add(401) != 402 || f.add(800) != 800 || f.add(801) != 800) return false;
  if (stability::finite(f.add(stability::invalid))) return false;
  return f.add(500) == 500;
}
static_assert(median());
static_assert(lift_height_math::valid_distance(20) && lift_height_math::valid_distance(2000));
static_assert(!lift_height_math::valid_distance(19) && !lift_height_math::valid_distance(2001));
static_assert(!lift_height_math::valid_distance(0) && !lift_height_math::valid_distance(-1));
static_assert(!lift_height_math::valid_distance(9999) && !lift_height_math::valid_distance(2147483647));
constexpr bool row_equals(telemetry_format::Row row, const char* expected) {
  unsigned i=0;
  for (; expected[i]; ++i) if (row[i] != expected[i]) return false;
  for (; i<15; ++i) if (row[i] != ' ') return false;
  return row[15] == '\0';
}
constexpr auto typical = telemetry_format::dashboard(301.1,-16.6,14.3,842.4,18.42,true);
static_assert(row_equals(typical[0], "X-16.6\" Y14.3\""));
static_assert(row_equals(typical[1], "H 301d L 842d"));
static_assert(row_equals(typical[2], "Ht 18.42in"));
constexpr auto bad = telemetry_format::dashboard(stability::invalid,stability::invalid,
    stability::invalid,stability::invalid,stability::invalid,false);
static_assert(row_equals(bad[0], "X--\" Y--\"") && row_equals(bad[1], "H --d L --d"));
static_assert(row_equals(bad[2], "Ht --"));
static_assert(row_equals(telemetry_format::dashboard(359.9,0,0,0,18.42,false)[1], "H 0d L 0d"));
static_assert(row_equals(telemetry_format::dashboard(0,0,0,0,18.42,false)[2], "Ht 18.42in"));
static_assert(row_equals(telemetry_format::dashboard(0,0,-999.9,-99999,0,true)[0], "X0.0\" Y-999.9\""));
static_assert(row_equals(telemetry_format::dashboard(0,0,1e30,1e30,0,true)[1], "H 0d L --d"));
static_assert(row_equals(telemetry_format::dashboard(0,-99.9,-99.9,0,0,true)[0], "X-99.9\" Y-99.9\""));
static_assert(row_equals(telemetry_format::dashboard(0,-999.9,-999.9,0,0,true)[0], "X--\" Y-999.9\""));
static_assert(row_equals(telemetry_format::dashboard(301,0,0,99999,0,true)[1], "H 301d L 99999d"));
static_assert(row_equals(telemetry_format::dashboard(301,0,0,999999,0,true)[1], "H 301d L --d"));
constexpr unsigned visible_length(telemetry_format::Row row) {
  unsigned end = 15;
  while (end && row[end-1] == ' ') --end;
  return end;
}
static_assert(visible_length(typical[0]) == 14);
static_assert(visible_length(typical[1]) == 13);
static_assert(visible_length(typical[2]) == 10);
int main() {} // All assertions execute at compile time on a host OR embedded compiler.
