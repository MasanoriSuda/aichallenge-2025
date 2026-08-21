#include <multi_purpose_mpc_ros/mpc_stage_geometry.hpp>

#include <cmath>
#include <limits>

namespace multi_purpose_mpc_ros::mpc_stage_geometry
{
namespace
{

int wrap_index(const int index, const int size) noexcept
{
  return ((index % size) + size) % size;
}

}  // namespace

Geometry build(
  const std::vector<Point2d> & path, const int tracking_waypoint,
  const std::size_t stage_count, const bool circular) noexcept
{
  Geometry geometry;
  geometry.tracking_waypoint = tracking_waypoint;
  geometry.circular = circular;
  if (path.size() < 2U) {
    geometry.reject_reason = "path has fewer than two waypoints";
    return geometry;
  }
  if (stage_count == 0U) {
    geometry.reject_reason = "stage count is zero";
    return geometry;
  }
  const int path_size = static_cast<int>(path.size());
  if (tracking_waypoint < 0 || tracking_waypoint >= path_size) {
    geometry.reject_reason = "tracking waypoint is outside path";
    return geometry;
  }
  if (!circular && static_cast<std::size_t>(tracking_waypoint) + stage_count >= path.size()) {
    geometry.reject_reason = "non-circular horizon exceeds path";
    return geometry;
  }

  geometry.stages.reserve(stage_count);
  double cumulative_distance_m = 0.0;
  for (std::size_t stage_index = 0; stage_index < stage_count; ++stage_index) {
    const int unwrapped_from = tracking_waypoint + static_cast<int>(stage_index);
    const int unwrapped_state = unwrapped_from + 1;
    const int from = circular ? wrap_index(unwrapped_from, path_size) : unwrapped_from;
    const int state = circular ? wrap_index(unwrapped_state, path_size) : unwrapped_state;
    const double transition_distance_m = std::hypot(
      path[static_cast<std::size_t>(state)].x_m - path[static_cast<std::size_t>(from)].x_m,
      path[static_cast<std::size_t>(state)].y_m - path[static_cast<std::size_t>(from)].y_m);
    if (!std::isfinite(transition_distance_m) || transition_distance_m < 0.0) {
      geometry.stages.clear();
      geometry.reject_reason = "stage transition distance is invalid";
      return geometry;
    }
    cumulative_distance_m += transition_distance_m;
    if (!std::isfinite(cumulative_distance_m)) {
      geometry.stages.clear();
      geometry.reject_reason = "stage cumulative distance is invalid";
      return geometry;
    }
    geometry.stages.push_back(Stage{
      from, state, transition_distance_m, cumulative_distance_m});
  }
  geometry.valid = true;
  geometry.reject_reason = "ok";
  return geometry;
}

}  // namespace multi_purpose_mpc_ros::mpc_stage_geometry
