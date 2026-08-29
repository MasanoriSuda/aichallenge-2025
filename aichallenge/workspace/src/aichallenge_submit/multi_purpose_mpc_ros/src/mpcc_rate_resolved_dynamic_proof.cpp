#include "multi_purpose_mpc_ros/mpcc_rate_resolved_dynamic_proof.hpp"

#include "multi_purpose_mpc_ros/mpcc_rate_resolved_execution_artifact.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_physical_wall.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_shadow.hpp"

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
      result.rejection_reason =
        recovery::DynamicClearanceRejectReason::InvalidObstacle;
      result.rejected_obstacle_id = obstacle.id;
      result.rejected_elapsed_sec = elapsed_time_sec;
      result.rejected_pose = pose;
      return;
    }
    ++result.checked_pose_count;
    if (clearance.value() < result.minimum_clearance_m) {
      result.minimum_clearance_m = clearance.value();
      result.minimum_clearance_obstacle_id = obstacle.id;
      result.minimum_clearance_elapsed_sec = elapsed_time_sec;
      result.minimum_clearance_pose = pose;
    }
    const auto reason = recovery::observe_dynamic_clearance(
      result.obstacle_clearance[index], clearance.value());
    if (reason != recovery::DynamicClearanceRejectReason::None) {
      result.clear = false;
      result.blocking_obstacle_id = obstacle.id;
      result.rejection_reason = reason;
      result.rejected_obstacle_id = obstacle.id;
      result.rejected_elapsed_sec = elapsed_time_sec;
      result.rejected_pose = pose;
      result.rejected_clearance_m = clearance.value();
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
      result.rejection_reason = reason;
      result.rejected_obstacle_id = observation.obstacles[index].id;
      result.rejected_clearance_m =
        result.obstacle_clearance[index].final_clearance_m;
      return;
    }
  }
}

Result evaluate_current_world(
  const mpcc_rate_resolved_shadow::Snapshot & solver_snapshot,
  const mpcc_rate_resolved_physical_wall::Snapshot & physical_snapshot)
{
  namespace artifact = mpcc_rate_resolved_execution_artifact;
  namespace physical_wall = mpcc_rate_resolved_physical_wall;
  namespace shadow = mpcc_rate_resolved_shadow;

  Result result;
  const auto reject_invalid = [&result]() {
      result.valid = false;
      result.clear = false;
      return result;
    };
  if (
    !solver_snapshot.replay_world.has_value() ||
    !artifact::same_identity(
      solver_snapshot.identity, physical_snapshot.identity.artifact) ||
    !physical_wall::snapshot_valid(physical_snapshot))
  {
    return reject_invalid();
  }
  const auto & replay = solver_snapshot.replay_world.value();
  const auto same_pose = [](const recovery::Pose2D & lhs,
      const recovery::Pose2D & rhs) {
      return lhs.x_m == rhs.x_m && lhs.y_m == rhs.y_m &&
             lhs.yaw_rad == rhs.yaw_rad;
    };
  const auto same_footprint = [](const recovery::FootprintExtents & lhs,
      const recovery::FootprintExtents & rhs) {
      return lhs.front_extent_m == rhs.front_extent_m &&
             lhs.rear_extent_m == rhs.rear_extent_m &&
             lhs.left_extent_m == rhs.left_extent_m &&
             lhs.right_extent_m == rhs.right_extent_m &&
             lhs.margin_m == rhs.margin_m;
    };
  if (
    !replay.current || replay.observation_generation == 0U ||
    replay.control_prefix.empty() ||
    replay.control_prefix.size() != replay.control_prefix_elapsed_sec.size() ||
    replay.control_prefix.size() != physical_snapshot.control_prefix.size() ||
    !std::equal(
      replay.control_prefix.begin(), replay.control_prefix.end(),
      physical_snapshot.control_prefix.begin(), same_pose) ||
    !same_pose(replay.current_pose, physical_snapshot.current_pose) ||
    !same_footprint(replay.physical_footprint, physical_snapshot.footprint) ||
    replay.swept_step_m != physical_snapshot.swept_step_m ||
    replay.bound_tolerance_m != physical_snapshot.bound_tolerance_m ||
    replay.wall_grid_fingerprint != physical_snapshot.wall_grid_fingerprint ||
    !std::isfinite(solver_snapshot.control_prediction_origin_sec) ||
    solver_snapshot.control_prediction_origin_sec < replay.observed_sec ||
    physical_snapshot.trajectory.elapsed_time_sec.empty())
  {
    return reject_invalid();
  }

  WorldObservation observation;
  observation.generation = replay.observation_generation;
  observation.observed_sec = replay.observed_sec;
  observation.current = replay.current;
  observation.obstacles.reserve(replay.obstacles.size());
  for (const shadow::ReplayDynamicObstacle & source : replay.obstacles) {
    observation.obstacles.push_back(DynamicObstacle{
      source.id,
      recovery::CircleObstacle{
        source.x_m, source.y_m, source.velocity_x_mps,
        source.velocity_y_mps, source.radius_m}});
  }
  if (!observation_valid(observation)) {
    return reject_invalid();
  }

  observe_timed_path(
    replay.physical_footprint, replay.control_prefix,
    replay.control_prefix_elapsed_sec, replay.swept_step_m,
    observation, result);
  if (!result.valid || !result.clear) {
    return result;
  }
  auto previous_pose = replay.control_prefix.back();
  double previous_time_sec = replay.control_prefix_elapsed_sec.back();
  const double control_origin_age_sec =
    solver_snapshot.control_prediction_origin_sec - replay.observed_sec;
  for (
    std::size_t stage = 0U;
    stage < physical_snapshot.trajectory.elapsed_time_sec.size(); ++stage)
  {
    const auto reconstructed =
      physical_wall::reconstruct_stage_pose(physical_snapshot, stage);
    if (!reconstructed.has_value()) {
      return reject_invalid();
    }
    const double time_sec = control_origin_age_sec +
      physical_snapshot.trajectory.elapsed_time_sec[stage];
    observe_segment(
      replay.physical_footprint, previous_pose, reconstructed->pose,
      previous_time_sec, time_sec, replay.swept_step_m,
      observation, result);
    if (!result.valid || !result.clear) {
      return result;
    }
    previous_pose = reconstructed->pose;
    previous_time_sec = time_sec;
  }
  finalize(observation, result);
  return result;
}

}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_dynamic_proof
