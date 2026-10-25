#pragma once
#include <optional>

namespace field_points {
struct FieldPoint { double x; double y; };
struct StartPose { FieldPoint position; double heading; };
inline constexpr bool FIELD_POINTS_MEASURED = false;
// Unset until physically measured in inches in the canonical field frame.
// Future routes must check the guard and each optional before use.
inline constexpr std::optional<StartPose> start_position = std::nullopt;
inline constexpr std::optional<FieldPoint> first_object = std::nullopt;
inline constexpr std::optional<FieldPoint> goal = std::nullopt;
inline constexpr std::optional<FieldPoint> score_position = std::nullopt;
}
