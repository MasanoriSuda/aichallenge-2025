#pragma once

#include <vector>

namespace multi_purpose_mpc_ros::mpc_waypoint_association
{

struct Waypoint
{
  double x_m{};
  double y_m{};
  double heading_rad{};
};

struct Config
{
  bool enabled{false};
  double local_lookbehind_m{8.0};
  double local_lookahead_m{30.0};
  double lost_distance_m{4.0};
  double heading_weight_m_per_rad{2.0};
  double backward_progress_weight{0.25};
  double forward_jump_weight{1.0};
  double minimum_forward_reach_m{1.5};
  double forward_reach_time_scale{4.0};
};

struct Request
{
  double x_m{};
  double y_m{};
  double yaw_rad{};
  double speed_mps{};
  double dt_sec{};
  int previous_index{};
  bool previous_valid{false};
  bool circular{false};
};

struct Result
{
  int index{};
  bool used_global_search{true};
  double distance_m{};
  double heading_error_rad{};
  double signed_progress_m{};
};

Result associate(
  const std::vector<Waypoint> & waypoints,
  const Request & request,
  const Config & config);

}  // namespace multi_purpose_mpc_ros::mpc_waypoint_association

