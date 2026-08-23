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

std::optional<std::vector<CourseFollowingPose>> sample_course_following_trajectory(
  const std::vector<CourseFrameKnot> & course_knots,
  const std::vector<FrenetTrajectoryState> & states,
  const double maximum_progress_step_m) noexcept
{
  constexpr double kProgressToleranceM = 1e-9;
  constexpr std::size_t kMaximumSamples = 100000U;
  if (
    course_knots.size() < 2U || states.empty() ||
    !std::isfinite(maximum_progress_step_m) ||
    maximum_progress_step_m <= kProgressToleranceM)
  {
    return std::nullopt;
  }
  for (std::size_t index = 0U; index < states.size(); ++index) {
    const auto & state = states[index];
    if (
      !std::isfinite(state.progress_m) || !std::isfinite(state.lateral_m) ||
      !std::isfinite(state.lag_m) ||
      !std::isfinite(state.heading_offset_rad) ||
      (index > 0U &&
      state.progress_m < states[index - 1U].progress_m - kProgressToleranceM))
    {
      return std::nullopt;
    }
  }

  std::vector<CourseFollowingPose> result;
  const auto append_pose = [&](const FrenetTrajectoryState & state,
      const std::size_t destination_state_index,
      const double segment_ratio) -> bool {
      const auto course = sample_course_frame(
        course_knots, state.progress_m, kProgressToleranceM);
      if (!course.has_value()) {
        return false;
      }
      result.push_back(CourseFollowingPose{
        course->x_m + state.lag_m * std::cos(course->heading_rad) -
        state.lateral_m * std::sin(course->heading_rad),
        course->y_m + state.lag_m * std::sin(course->heading_rad) +
        state.lateral_m * std::cos(course->heading_rad),
        std::remainder(
          course->heading_rad + state.heading_offset_rad,
          2.0 * std::acos(-1.0)),
        destination_state_index, segment_ratio});
      return true;
    };

  result.reserve(states.size());
  if (!append_pose(states.front(), 0U, 1.0)) {
    return std::nullopt;
  }
  for (std::size_t state_index = 1U; state_index < states.size(); ++state_index) {
    const auto & from = states[state_index - 1U];
    const auto & to = states[state_index];
    const double progress_delta_m = to.progress_m - from.progress_m;
    const std::size_t subdivisions = std::max<std::size_t>(
      1U, static_cast<std::size_t>(std::ceil(
        std::max(0.0, progress_delta_m) / maximum_progress_step_m)));
    if (subdivisions > kMaximumSamples - result.size()) {
      return std::nullopt;
    }
    for (std::size_t substep = 1U; substep <= subdivisions; ++substep) {
      const double ratio =
        static_cast<double>(substep) / static_cast<double>(subdivisions);
      const double heading_delta = std::remainder(
        to.heading_offset_rad - from.heading_offset_rad,
        2.0 * std::acos(-1.0));
      const FrenetTrajectoryState interpolated{
        from.progress_m + ratio * progress_delta_m,
        from.lateral_m + ratio * (to.lateral_m - from.lateral_m),
        from.lag_m + ratio * (to.lag_m - from.lag_m),
        from.heading_offset_rad + ratio * heading_delta};
      if (!append_pose(interpolated, state_index, ratio)) {
        return std::nullopt;
      }
    }
  }
  return result;
}

}  // namespace multi_purpose_mpc_ros::mpc_stage_geometry
