#pragma once

namespace multi_purpose_mpc_ros::mpc_waypoint_preview
{

inline constexpr int kMinOffset = 0;
inline constexpr int kMaxOffset = 2;

struct InputReference
{
  double velocity_mps{0.0};
  double curvature_radpm{0.0};
};

bool is_valid_offset(int offset);

int select_effective_offset(
  int normal_offset,
  int low_speed_offset,
  double current_speed_mps,
  double low_speed_threshold_mps);

int resolve_preview_index(
  int tracking_index,
  int offset,
  int waypoint_count,
  bool circular);

InputReference resolve_input_reference(
  double tracking_velocity_mps,
  double tracking_curvature_radpm,
  double preview_velocity_mps,
  double preview_curvature_radpm);

} // namespace multi_purpose_mpc_ros::mpc_waypoint_preview
