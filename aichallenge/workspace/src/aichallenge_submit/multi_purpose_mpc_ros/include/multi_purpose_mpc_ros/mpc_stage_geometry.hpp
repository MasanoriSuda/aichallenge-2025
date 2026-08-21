#ifndef MULTI_PURPOSE_MPC_ROS__MPC_STAGE_GEOMETRY_HPP_
#define MULTI_PURPOSE_MPC_ROS__MPC_STAGE_GEOMETRY_HPP_

#include <cstddef>
#include <string>
#include <vector>

namespace multi_purpose_mpc_ros::mpc_stage_geometry
{

struct Point2d
{
  double x_m{};
  double y_m{};
};

struct Stage
{
  int transition_from_waypoint{};
  int state_waypoint{};
  double transition_distance_m{};
  double cumulative_distance_m{};
};

struct Geometry
{
  bool valid{false};
  int tracking_waypoint{};
  bool circular{false};
  std::vector<Stage> stages;
  std::string reject_reason;
};

/// Build the single stage-index contract shared by dynamics, lateral bounds,
/// wall validation and execution certificates.  Stage zero is the state after
/// the transition tracking_waypoint -> tracking_waypoint + 1.
Geometry build(
  const std::vector<Point2d> & path, int tracking_waypoint,
  std::size_t stage_count, bool circular) noexcept;

}  // namespace multi_purpose_mpc_ros::mpc_stage_geometry

#endif  // MULTI_PURPOSE_MPC_ROS__MPC_STAGE_GEOMETRY_HPP_
