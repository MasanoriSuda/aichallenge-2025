#include <multi_purpose_mpc_ros/mpc_stage_geometry.hpp>

#include <algorithm>
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

std::optional<CourseFrameSample> sample_course_frame(
  const std::vector<CourseFrameKnot> & knots,
  const double query_progress_m,
  const double query_tolerance_m) noexcept
{
  constexpr double kProgressToleranceM = 1e-9;
  if (
    knots.size() < 2U || !std::isfinite(query_progress_m) ||
    !std::isfinite(query_tolerance_m) || query_tolerance_m < 0.0)
  {
    return std::nullopt;
  }
  for (std::size_t index = 0U; index < knots.size(); ++index) {
    const auto & knot = knots[index];
    if (
      !std::isfinite(knot.progress_m) || !std::isfinite(knot.x_m) ||
      !std::isfinite(knot.y_m) || !std::isfinite(knot.heading_rad) ||
      (index > 0U &&
      knot.progress_m <= knots[index - 1U].progress_m + kProgressToleranceM))
    {
      return std::nullopt;
    }
  }
  if (
    query_progress_m < knots.front().progress_m - query_tolerance_m ||
    query_progress_m > knots.back().progress_m + query_tolerance_m)
  {
    return std::nullopt;
  }

  if (query_progress_m <= knots.front().progress_m + query_tolerance_m) {
    const auto & knot = knots.front();
    return CourseFrameSample{
      knot.progress_m, knot.x_m, knot.y_m, knot.heading_rad,
      knot.waypoint, knot.waypoint, 0.0};
  }
  if (query_progress_m >= knots.back().progress_m - query_tolerance_m) {
    const auto & knot = knots.back();
    return CourseFrameSample{
      knot.progress_m, knot.x_m, knot.y_m, knot.heading_rad,
      knot.waypoint, knot.waypoint, 0.0};
  }

  const auto upper = std::upper_bound(
    knots.begin(), knots.end(), query_progress_m,
    [](const double progress_m, const CourseFrameKnot & knot) {
      return progress_m < knot.progress_m;
    });
  if (upper == knots.begin() || upper == knots.end()) {
    return std::nullopt;
  }
  const auto lower = std::prev(upper);
  const double segment_progress_m = upper->progress_m - lower->progress_m;
  if (!std::isfinite(segment_progress_m) || segment_progress_m <= kProgressToleranceM) {
    return std::nullopt;
  }
  const double ratio = std::clamp(
    (query_progress_m - lower->progress_m) / segment_progress_m, 0.0, 1.0);
  const double heading_delta = std::atan2(
    std::sin(upper->heading_rad - lower->heading_rad),
    std::cos(upper->heading_rad - lower->heading_rad));
  const double heading = std::atan2(
    std::sin(lower->heading_rad + ratio * heading_delta),
    std::cos(lower->heading_rad + ratio * heading_delta));
  return CourseFrameSample{
    query_progress_m,
    lower->x_m + ratio * (upper->x_m - lower->x_m),
    lower->y_m + ratio * (upper->y_m - lower->y_m),
    heading,
    lower->waypoint,
    upper->waypoint,
    ratio};
}

}  // namespace multi_purpose_mpc_ros::mpc_stage_geometry
