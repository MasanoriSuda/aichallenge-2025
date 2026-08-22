#include "multi_purpose_mpc_ros/canonical_retained_world_revalidation.hpp"

#include "multi_purpose_mpc_ros/mpcc_execution_contract.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace multi_purpose_mpc_ros::canonical_retained_world_revalidation
{
namespace
{

constexpr double kIdentityTolerance = 1e-9;

class FingerprintBuilder
{
public:
  void add_uint64(const std::uint64_t value) noexcept
  {
    for (std::size_t shift = 0U; shift < sizeof(value); ++shift) {
      const auto byte = static_cast<unsigned char>(value >> (8U * shift));
      hash_ ^= static_cast<std::uint64_t>(byte);
      hash_ *= 1099511628211ULL;
    }
  }

  void add_size(const std::size_t value) noexcept
  {
    add_uint64(static_cast<std::uint64_t>(value));
  }

  void add_double(const double value) noexcept
  {
    std::uint64_t bits{};
    static_assert(sizeof(bits) == sizeof(value), "unexpected double width");
    std::memcpy(&bits, &value, sizeof(value));
    add_uint64(bits);
  }

  std::uint64_t value() const noexcept {return hash_ == 0U ? 1U : hash_;}

private:
  std::uint64_t hash_{1469598103934665603ULL};
};

bool finite_pose(const recovery_footprint::Pose2D & pose) noexcept
{
  return std::isfinite(pose.x_m) && std::isfinite(pose.y_m) &&
         std::isfinite(pose.yaw_rad);
}

bool same_pose(
  const recovery_footprint::Pose2D & left,
  const recovery_footprint::Pose2D & right) noexcept
{
  return finite_pose(left) && finite_pose(right) &&
         std::abs(left.x_m - right.x_m) <= kIdentityTolerance &&
         std::abs(left.y_m - right.y_m) <= kIdentityTolerance &&
         std::abs(left.yaw_rad - right.yaw_rad) <= kIdentityTolerance;
}

std::optional<recovery_footprint::Pose2D> reconstruct_pose(
  const std::vector<mpc_stage_geometry::CourseFrameKnot> & knots,
  const plan::CanonicalPredictedState & state)
{
  const auto frame = mpc_stage_geometry::sample_course_frame(
    knots, state.progress_m, kCourseFrameIdentityToleranceM);
  if (!frame.has_value()) {
    return std::nullopt;
  }
  const auto pose = mpcc_execution_contract::reconstruct_planar_pose_from_frenet(
    mpcc_execution_contract::PlanarPose{
        frame->x_m, frame->y_m, frame->heading_rad},
    mpcc_execution_contract::FrenetPose{
        state.lateral_m, state.lag_m, state.heading_offset_rad});
  if (!pose.has_value()) {
    return std::nullopt;
  }
  return recovery_footprint::Pose2D{
    pose->x_m, pose->y_m, pose->yaw_rad};
}

retained::RetainedPathSegmentEvaluation make_segment_evaluation(
  const retained::CurrentExecutionProvenance & current,
  const double start_progress_m, const double end_progress_m,
  const recovery_footprint::PathClearanceResult & wall)
{
  retained::RetainedPathSegmentEvaluation evaluation;
  evaluation.observation_generation = current.observation_generation;
  evaluation.stage_geometry_id = current.stage_geometry_id;
  evaluation.target_obstacle_generation =
    current.target_obstacle_generation;
  evaluation.control_pose_id = current.control_pose_id;
  evaluation.course_frame_window_id = current.course_frame_window_id;
  evaluation.obstacle_tube_id = current.obstacle_tube_id;
  evaluation.start_progress_m = start_progress_m;
  evaluation.end_progress_m = end_progress_m;
  evaluation.checked = wall.valid;
  evaluation.wall_clear = wall.valid && wall.clear;
  evaluation.obstacles_clear = true;
  // The raster API proves non-intersection, not a metric distance. Zero is a
  // conservative certified lower bound and must not be interpreted as a
  // tunable wall margin.
  evaluation.minimum_wall_clearance_m = 0.0;
  evaluation.minimum_obstacle_clearance_m = 0.0;
  return evaluation;
}

}  // namespace

const char * to_string(const CurrentWorldProofReason reason) noexcept
{
  switch (reason) {
    case CurrentWorldProofReason::Accepted:
      return "accepted";
    case CurrentWorldProofReason::InvalidInput:
      return "invalid-input";
    case CurrentWorldProofReason::WindowRejected:
      return "window-rejected";
    case CurrentWorldProofReason::ProgressLiftRejected:
      return "progress-lift-rejected";
    case CurrentWorldProofReason::ControlPoseIdentityMismatch:
      return "control-pose-identity-mismatch";
    case CurrentWorldProofReason::CourseFrameIdentityMismatch:
      return "course-frame-identity-mismatch";
    case CurrentWorldProofReason::ObstacleObservationUnavailable:
      return "obstacle-observation-unavailable";
    case CurrentWorldProofReason::ObstacleTubeIdentityMismatch:
      return "obstacle-tube-identity-mismatch";
    case CurrentWorldProofReason::DynamicObstaclePresent:
      return "dynamic-obstacle-present";
    case CurrentWorldProofReason::CourseFrameUnavailable:
      return "course-frame-unavailable";
    case CurrentWorldProofReason::DelayPrefixBlocked:
      return "delay-prefix-blocked";
    case CurrentWorldProofReason::ConnectorBlocked:
      return "connector-blocked";
    case CurrentWorldProofReason::StagePathBlocked:
      return "stage-path-blocked";
    case CurrentWorldProofReason::ProofRejected:
      return "proof-rejected";
  }
  return "unknown";
}

std::uint64_t fingerprint_control_pose_path(
  const std::vector<recovery_footprint::Pose2D> & measured_to_control_path,
  const recovery_footprint::Pose2D & control_pose) noexcept
{
  if (measured_to_control_path.empty() || !finite_pose(control_pose)) {
    return 0U;
  }
  FingerprintBuilder builder;
  builder.add_size(measured_to_control_path.size());
  for (const auto & pose : measured_to_control_path) {
    if (!finite_pose(pose)) {
      return 0U;
    }
    builder.add_double(pose.x_m);
    builder.add_double(pose.y_m);
    builder.add_double(pose.yaw_rad);
  }
  builder.add_double(control_pose.x_m);
  builder.add_double(control_pose.y_m);
  builder.add_double(control_pose.yaw_rad);
  return builder.value();
}

std::uint64_t fingerprint_course_frame_window(
  const std::vector<mpc_stage_geometry::CourseFrameKnot> & knots) noexcept
{
  if (knots.size() < 2U) {
    return 0U;
  }
  FingerprintBuilder builder;
  builder.add_size(knots.size());
  double previous_progress_m = -std::numeric_limits<double>::infinity();
  for (const auto & knot : knots) {
    if (!std::isfinite(knot.progress_m) ||
      !std::isfinite(knot.x_m) || !std::isfinite(knot.y_m) ||
      !std::isfinite(knot.heading_rad) ||
      knot.progress_m <= previous_progress_m)
    {
      return 0U;
    }
    builder.add_double(knot.progress_m);
    builder.add_double(knot.x_m);
    builder.add_double(knot.y_m);
    builder.add_double(knot.heading_rad);
    builder.add_uint64(static_cast<std::uint64_t>(knot.waypoint));
    previous_progress_m = knot.progress_m;
  }
  return builder.value();
}

std::uint64_t fingerprint_empty_obstacle_observation(
  const std::uint64_t observation_generation,
  const double observation_sec) noexcept
{
  if (observation_generation == 0U || !std::isfinite(observation_sec) ||
    observation_sec < 0.0)
  {
    return 0U;
  }
  FingerprintBuilder builder;
  builder.add_uint64(observation_generation);
  builder.add_double(observation_sec);
  builder.add_size(0U);
  return builder.value();
}

CurrentWorldProofResult build_current_world_retained_proof(
  const plan::CanonicalExecutionPlan & execution_plan,
  const plan::CanonicalExecutionCursor & cursor,
  const CurrentWorldProofRequest & request,
  const recovery_footprint::OccupancyGrid & wall_grid,
  const recovery_footprint::FootprintExtents & footprint)
{
  CurrentWorldProofResult result;
  const auto window = retained::build_retained_execution_window(
    execution_plan, cursor);
  if (!window.window.has_value()) {
    result.reason = CurrentWorldProofReason::WindowRejected;
    return result;
  }
  if (!wall_grid.valid() || !footprint.valid() ||
    request.measured_to_control_path.empty() ||
    !std::isfinite(request.swept_step_m) || request.swept_step_m <= 0.0)
  {
    return result;
  }
  // Classify the dynamic-world boundary before the aggregate provenance
  // check. Otherwise a missing obstacle fingerprint collapses into the
  // generic InvalidInput result and hides whether the world was unobserved,
  // occupied, or observed with a different identity.
  if (request.obstacles.active_vehicle_count != 0U ||
    !request.current.target_id.empty() ||
    request.current.target_obstacle_generation != 0U)
  {
    result.reason = CurrentWorldProofReason::DynamicObstaclePresent;
    return result;
  }
  if (!request.obstacles.current ||
    request.obstacles.observation_generation == 0U ||
    !std::isfinite(request.obstacles.observation_sec) ||
    request.obstacles.observation_sec < 0.0)
  {
    result.reason = CurrentWorldProofReason::ObstacleObservationUnavailable;
    return result;
  }
  if (request.obstacles.tube_id != request.current.obstacle_tube_id ||
    fingerprint_empty_obstacle_observation(
      request.obstacles.observation_generation,
      request.obstacles.observation_sec) != request.obstacles.tube_id)
  {
    result.reason = CurrentWorldProofReason::ObstacleTubeIdentityMismatch;
    return result;
  }
  if (!retained::current_execution_provenance_complete(request.current)) {
    return result;
  }
  const auto lift = retained::lift_progress_to_retained_branch(
    retained::CircularProgressLiftRequest{
      request.measured_course_progress_m,
      window.window->expected_current_progress_m,
      request.current.path_length_m,
      request.progress_continuity_tolerance_m,
      request.current.circular});
  if (lift.reason != retained::CircularProgressLiftReason::Accepted) {
    result.reason = CurrentWorldProofReason::ProgressLiftRejected;
    return result;
  }
  if (!same_pose(request.measured_to_control_path.back(), request.control_pose) ||
    fingerprint_control_pose_path(
      request.measured_to_control_path, request.control_pose) !=
    request.current.control_pose_id)
  {
    result.reason = CurrentWorldProofReason::ControlPoseIdentityMismatch;
    return result;
  }
  if (fingerprint_course_frame_window(request.course_frame_knots) !=
    request.current.course_frame_window_id)
  {
    result.reason = CurrentWorldProofReason::CourseFrameIdentityMismatch;
    return result;
  }
  const auto expected_current_pose = reconstruct_pose(
    request.course_frame_knots, window.window->expected_current_state);
  if (!expected_current_pose.has_value()) {
    result.reason = CurrentWorldProofReason::CourseFrameUnavailable;
    return result;
  }

  retained::RetainedExecutionProofRequest proof_request;
  proof_request.current = request.current;
  proof_request.measured_course_progress_m =
    request.measured_course_progress_m;
  proof_request.progress_continuity_tolerance_m =
    request.progress_continuity_tolerance_m;
  const auto delay_wall = recovery_footprint::evaluate_clear_footprint_path(
    wall_grid, footprint, request.measured_to_control_path,
    request.swept_step_m);
  proof_request.measured_to_control_prefix = make_segment_evaluation(
    request.current, lift.lifted_progress_m, lift.lifted_progress_m,
    delay_wall);
  if (!delay_wall.valid || !delay_wall.clear) {
    result.reason = CurrentWorldProofReason::DelayPrefixBlocked;
    return result;
  }

  const std::vector<recovery_footprint::Pose2D> connector_path{
    request.control_pose, expected_current_pose.value()};
  const auto connector_wall = recovery_footprint::evaluate_clear_footprint_path(
    wall_grid, footprint, connector_path, request.swept_step_m);
  proof_request.control_to_retained_connector = make_segment_evaluation(
    request.current, lift.lifted_progress_m,
    window.window->expected_current_progress_m, connector_wall);
  if (!connector_wall.valid || !connector_wall.clear) {
    result.reason = CurrentWorldProofReason::ConnectorBlocked;
    return result;
  }

  auto previous_pose = expected_current_pose.value();
  proof_request.stage_evaluations.reserve(window.window->samples.size());
  for (std::size_t index = 0U; index < window.window->samples.size(); ++index) {
    const auto & sample = window.window->samples[index];
    const auto endpoint_pose = reconstruct_pose(
      request.course_frame_knots, sample.endpoint);
    if (!endpoint_pose.has_value()) {
      result.reason = CurrentWorldProofReason::CourseFrameUnavailable;
      result.rejected_stage_index = index;
      return result;
    }
    const std::vector<recovery_footprint::Pose2D> stage_path{
      previous_pose, endpoint_pose.value()};
    const auto stage_wall = recovery_footprint::evaluate_clear_footprint_path(
      wall_grid, footprint, stage_path, request.swept_step_m);
    retained::RetainedStageSafetyEvaluation evaluation;
    evaluation.control_stage_index = sample.control_stage_index;
    evaluation.relative_time_sec = sample.relative_time_sec;
    evaluation.segment_duration_sec = sample.segment_duration_sec;
    evaluation.segment_start_progress_m = sample.segment_start_progress_m;
    evaluation.absolute_progress_m = sample.absolute_progress_m;
    evaluation.observation_generation = request.current.observation_generation;
    evaluation.stage_geometry_id = request.current.stage_geometry_id;
    evaluation.target_obstacle_generation =
      request.current.target_obstacle_generation;
    evaluation.course_frame_window_id = request.current.course_frame_window_id;
    evaluation.obstacle_tube_id = request.current.obstacle_tube_id;
    evaluation.course_frame_available = true;
    evaluation.wall_checked = stage_wall.valid;
    evaluation.wall_clear = stage_wall.valid && stage_wall.clear;
    evaluation.obstacle_checked = true;
    evaluation.obstacles_clear = true;
    evaluation.minimum_wall_clearance_m = 0.0;
    evaluation.minimum_obstacle_clearance_m = 0.0;
    proof_request.stage_evaluations.push_back(evaluation);
    if (!stage_wall.valid || !stage_wall.clear) {
      result.reason = CurrentWorldProofReason::StagePathBlocked;
      result.rejected_stage_index = index;
      return result;
    }
    previous_pose = endpoint_pose.value();
  }

  auto proof = retained::build_retained_execution_proof(
    execution_plan, cursor, proof_request);
  result.proof_reason = proof.reason;
  if (!proof.proof.has_value()) {
    result.reason = CurrentWorldProofReason::ProofRejected;
    return result;
  }
  result.reason = CurrentWorldProofReason::Accepted;
  result.proof = std::move(proof.proof);
  return result;
}

}  // namespace multi_purpose_mpc_ros::canonical_retained_world_revalidation
