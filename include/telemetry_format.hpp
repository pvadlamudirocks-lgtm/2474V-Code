#pragma once
#include "stability_math.hpp"
#include <array>

namespace telemetry_format {
using Row = std::array<char, 16>; // 15 physical columns plus terminator.
struct Text { std::array<char, 16> data{}; unsigned size = 0; };
constexpr Text number(double value, unsigned decimals, unsigned max_width) {
  Text text;
  if (stability::finite(value) && stability::magnitude(value) < 10000000 && decimals <= 2) {
    const unsigned scale = decimals == 2 ? 100 : decimals == 1 ? 10 : 1;
    unsigned long n = static_cast<unsigned long>(stability::magnitude(value) * scale + .5);
    const unsigned long rounded = n;
    std::array<char, 12> reversed{};
    unsigned digits = 0;
    do { reversed[digits++] = static_cast<char>('0' + n % 10); n /= 10; }
    while (n || digits <= decimals);
    if (value < 0 && rounded != 0) text.data[text.size++] = '-';
    while (digits) {
      text.data[text.size++] = reversed[--digits];
      if (decimals && digits == decimals) text.data[text.size++] = '.';
    }
    if (text.size <= max_width) return text;
  }
  text = Text{};
  text.data[0] = text.data[1] = '-'; text.size = 2;
  return text; // Invalid or too wide: never truncate a number into another value.
}
constexpr Row pair(char a, Text first, char b, Text second, char unit, bool label_space) {
  // Include both unit suffixes and optional label spaces in the 15-column budget.
  const unsigned number_columns = label_space ? 8 : 10;
  if (first.size + second.size > number_columns) {
    if (first.size >= second.size) first = number(stability::invalid, 0, 2);
    else second = number(stability::invalid, 0, 2);
  }
  Row row{}; for (unsigned i = 0; i < 15; ++i) row[i] = ' ';
  unsigned pos = 0; row[pos++] = a;
  if (label_space) row[pos++] = ' ';
  for (unsigned i = 0; i < first.size; ++i) row[pos++] = first.data[i];
  row[pos++] = unit;
  row[pos++] = ' '; row[pos++] = b;
  if (label_space) row[pos++] = ' ';
  for (unsigned i = 0; i < second.size; ++i) row[pos++] = second.data[i];
  row[pos++] = unit;
  return row;
}
constexpr std::array<Row, 3> dashboard(double heading, double x, double y,
                                      double lift, double distance_in, bool /*vertical*/) {
  if (!stability::finite(heading) || heading < 0 || heading >= 360) heading = stability::invalid;
  else if (heading >= 359.5) heading = 0; // Display rounding only, canonical heading unchanged.
  std::array<Row, 3> rows{};
  rows[0] = pair('X', number(x, 1, 7), 'Y', number(y, 1, 7), '"', false);
  // UTF-8 degree-glyph rendering is unverified on V5; use the clear ASCII fallback.
  rows[1] = pair('H', number(heading, 0, 3), 'L', number(lift, 0, 7), 'd', true);
  auto& row = rows[2]; for (unsigned i = 0; i < 15; ++i) row[i] = ' ';
  row[0] = 'H'; row[1] = 't'; row[2] = ' '; unsigned pos = 3;
  const auto height = number(distance_in, 2, 7);
  for (unsigned i = 0; i < height.size; ++i) row[pos++] = height.data[i];
  if (stability::finite(distance_in)) {
    row[pos++] = 'i'; row[pos++] = 'n';
  }
  return rows;
}
}
