#include "multi_purpose_mpc_ros/mpcc_rate_resolved_dynamic_proof.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_dynamic_proof
{
namespace
{

constexpr double kIdentityTolerance = 1e-9;

bool finite_pose(const recovery::Pose2D & pose) noexcept
{
  return std::isfinite(pose.x_m) && std::isfinite(pose.y_m) &&
         std::isfinite(pose.yaw_rad);
}

double wrap_to_pi(const double angle) noexcept
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace

bool observation_valid(const WorldObservation & observation) noexcept
{
  if (
    observation.generation == 0U || !std::isfinite(observation.observed_sec) ||
    observation.observed_sec < 0.0)
  {
    return false;
  }
  std::unordered_set<std::string> ids;
  for (const auto & obstacle : observation.obstacles) {
    if (
      obstacle.id.empty() || !ids.emplace(obstacle.id).second ||
      !std::isfinite(obstacle.circle.x_m) ||
      !std::isfinite(obstacle.circle.y_m) ||
      !std::isfinite(obstacle.circle.velocity_x_mps) ||
      !std::isfinite(obstacle.circle.velocity_y_mps) ||
      !std::isfinite(obstacle.circle.radius_m) ||
      obstacle.circle.radius_m < 0.0)
    {
      return false;
    }
  }
  return true;
}

void observe_pose(
  const recovery::FootprintExtents & footprint,
  const recovery::Pose2D & pose,
  const double elapsed_time_sec,
  const WorldObservation & observation,
  Result & result)
{
  if (!result.valid || !result.clear) {
    return;
  }
  if (result.obstacle_clearance.empty()) {
    result.obstacle_clearance.resize(observation.obstacles.size());
  } else if (result.obstacle_clearance.size() != observation.obstacles.size()) {
    result.valid = false;
    result.clear = false;
    return;
  }
  for (std::size_t index = 0U; index < observation.obstacles.size(); ++index) {
    const auto & obstacle = observation.obstacles[index];
    const auto clearance = recovery::circle_obstacle_clearance_at_time(
      footprint, pose, obstacle.circle, elapsed_time_sec);
    if (!clearance.has_value()) {
      result.valid = false;
      result.clear = false;
      result.blocking_obstacle_id = obstacle.id;
      return;
    }
    ++result.checked_pose_count;
    result.minimum_clearance_m = std::min(
      result.minimum_clearance_m, clearance.value());
    const auto reason = recovery::observe_dynamic_clearance(
      result.obstacle_clearance[index], clearance.value());
    if (reason != recovery::DynamicClearanceRejectReason::None) {
      result.clear = false;
      result.blocking_obstacle_id = obstacle.id;
      return;
    }
  }
}

void observe_segment(
  const recovery::FootprintExtents & footprint,
  const recovery::Pose2D & start,
  const recovery::Pose2D & end,
  const double start_time_sec,
  const double end_time_sec,
  const double swept_step_m,
  const WorldObservation & observation,
  Result & result)
{
  if (!result.valid || !result.clear) {
    return;
  }
  if (
    !finite_pose(start) || !finite_pose(end) || !std::isfinite(start_time_sec) ||
    !std::isfinite(end_time_sec) || end_time_sec < start_time_sec ||
    !std::isfinite(swept_step_m) || swept_step_m <= 0.0)
  {
    result.valid = false;
    result.clear = false;
    return;
  }
  const double yaw_delta = wrap_to_pi(end.yaw_rad - start.yaw_rad);
  const double corner_radius_m = std::hypot(
    std::max(footprint.front_extent_m, footprint.rear_extent_m) +
    footprint.margin_m,
    std::max(footprint.left_extent_m, footprint.right_extent_m) +
    footprint.margin_m);
  double maximum_obstacle_motion_m = 0.0;
  const double duration_sec = end_time_sec - start_time_sec;
  for (const auto & obstacle : observation.obstacles) {
    maximum_obstacle_motion_m = std::max(
      maximum_obstacle_motion_m,
      std::hypot(
        obstacle.circle.velocity_x_mps,
        obstacle.circle.velocity_y_mps) * duration_sec);
  }
  const double relative_motion_bound_m =
    std::hypot(end.x_m - start.x_m, end.y_m - start.y_m) +
    corner_radius_m * std::abs(yaw_delta) + maximum_obstacle_motion_m;
  const double raw_subdivisions = std::ceil(relative_motion_bound_m / swept_step_m);
  if (!std::isfinite(raw_subdivisions) || raw_subdivisions > 1000000.0) {
    result.valid = false;
    result.clear = false;
    return;
  }
  const std::size_t subdivisions = std::max<std::size_t>(
    1U, static_cast<std::size_t>(raw_subdivisions));
  for (std::size_t index = 0U; index <= subdivisions; ++index) {
    const double fraction =
      static_cast<double>(index) / static_cast<double>(subdivisions);
    const recovery::Pose2D pose{
      start.x_m + fraction * (end.x_m - start.x_m),
      start.y_m + fraction * (end.y_m - start.y_m),
      start.yaw_rad + fraction * yaw_delta};
    observe_pose(
      footprint, pose, start_time_sec + fraction * duration_sec,
      observation, result);
    if (!result.valid || !result.clear) {
      return;
    }
  }
}

void observe_timed_path(
  const recovery::FootprintExtents & footprint,
  const std::vector<recovery::Pose2D> & path,
  const std::vector<double> & elapsed_sec,
  const double swept_step_m,
  const WorldObservation & observation,
  Result & result)
{
  if (!result.valid || !result.clear) {
    return;
  }
  if (
    path.empty() || path.size() != elapsed_sec.size() ||
    !std::isfinite(elapsed_sec.front()) ||
    std::abs(elapsed_sec.front()) > kIdentityTolerance)
  {
    result.valid = false;
    result.clear = false;
    return;
  }
  if (path.size() == 1U) {
    observe_pose(footprint, path.front(), elapsed_sec.front(), observation, result);
    return;
  }
  for (std::size_t index = 1U; index < path.size(); ++index) {
    observe_segment(
      footprint, path[index - 1U], path[index], elapsed_sec[index - 1U],
      elapsed_sec[index], swept_step_m, observation, result);
    if (!result.valid || !result.clear) {
      return;
    }
  }
}

void finalize(
  const WorldObservation & observation,
  Result & result)
{
  if (!result.valid || !result.clear) {
    return;
  }
  if (result.obstacle_clearance.size() != observation.obstacles.size()) {
    result.valid = false;
    result.clear = false;
    return;
  }
  for (std::size_t index = 0U; index < observation.obstacles.size(); ++index) {
    const auto reason = recovery::finalize_dynamic_clearance(
      result.obstacle_clearance[index]);
    if (reason != recovery::DynamicClearanceRejectReason::None) {
      result.clear = false;
      result.blocking_obstacle_id = observation.obstacles[index].id;
      return;
    }
  }
}

}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_dynamic_proof
