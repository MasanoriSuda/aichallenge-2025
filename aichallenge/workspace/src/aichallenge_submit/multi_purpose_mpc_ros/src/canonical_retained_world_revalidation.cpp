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

  void add_string(const std::string & value) noexcept
  {
    add_size(value.size());
    for (const unsigned char character : value) {
      add_uint64(static_cast<std::uint64_t>(character));
    }
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

bool follow_observation_shape_valid(
  const FollowDynamicObstacleObservation & observation) noexcept
{
  if (
    observation.target_id.empty() || observation.observation_generation == 0U ||
    !std::isfinite(observation.observation_sec) ||
    observation.observation_sec < 0.0 ||
    !std::isfinite(observation.hard_gap_m) || observation.hard_gap_m < 0.0 ||
    observation.elapsed_time_sec.size() < 2U ||
    observation.elapsed_time_sec.size() !=
    observation.target_relative_progress_m.size())
  {
    return false;
  }
  if (
    std::abs(observation.elapsed_time_sec.front()) > kIdentityTolerance ||
    !std::isfinite(observation.target_relative_progress_m.front()) ||
    observation.target_relative_progress_m.front() < 0.0)
  {
    return false;
  }
  for (std::size_t index = 1U; index < observation.elapsed_time_sec.size(); ++index) {
    if (
      !std::isfinite(observation.elapsed_time_sec[index]) ||
      observation.elapsed_time_sec[index] <=
      observation.elapsed_time_sec[index - 1U] ||
      !std::isfinite(observation.target_relative_progress_m[index]) ||
      observation.target_relative_progress_m[index] + kIdentityTolerance <
      observation.target_relative_progress_m[index - 1U])
    {
      return false;
    }
  }
  return true;
}

std::optional<double> sample_follow_target_progress(
  const FollowDynamicObstacleObservation & observation,
  const double relative_time_sec) noexcept
{
  if (!follow_observation_shape_valid(observation) ||
    !std::isfinite(relative_time_sec) || relative_time_sec < 0.0 ||
    relative_time_sec > observation.elapsed_time_sec.back() + kIdentityTolerance)
  {
    return std::nullopt;
  }
  if (relative_time_sec <= kIdentityTolerance) {
    return observation.target_relative_progress_m.front();
  }
  const auto upper = std::lower_bound(
    observation.elapsed_time_sec.begin(), observation.elapsed_time_sec.end(),
    relative_time_sec);
  if (upper == observation.elapsed_time_sec.end()) {
    return observation.target_relative_progress_m.back();
  }
  const std::size_t upper_index = static_cast<std::size_t>(
    std::distance(observation.elapsed_time_sec.begin(), upper));
  if (std::abs(*upper - relative_time_sec) <= kIdentityTolerance) {
    return observation.target_relative_progress_m[upper_index];
  }
  if (upper_index == 0U) {
    return std::nullopt;
  }
  const std::size_t lower_index = upper_index - 1U;
  const double duration =
    observation.elapsed_time_sec[upper_index] -
    observation.elapsed_time_sec[lower_index];
  const double fraction =
    (relative_time_sec - observation.elapsed_time_sec[lower_index]) / duration;
  return observation.target_relative_progress_m[lower_index] +
    fraction *
    (observation.target_relative_progress_m[upper_index] -
    observation.target_relative_progress_m[lower_index]);
}

bool overtake_corridor_shape_valid(
  const OvertakeDynamicCorridorObservation & observation) noexcept
{
  const std::size_t count = observation.elapsed_time_sec.size();
  if (
    observation.target_id.empty() || observation.observation_generation == 0U ||
    !std::isfinite(observation.observation_sec) ||
    observation.observation_sec < 0.0 || count < 2U ||
    observation.lateral_lower_m.size() != count ||
    observation.lateral_upper_m.size() != count ||
    std::abs(observation.elapsed_time_sec.front()) > kIdentityTolerance)
  {
    return false;
  }
  for (std::size_t index = 0U; index < count; ++index) {
    if (
      !std::isfinite(observation.elapsed_time_sec[index]) ||
      !std::isfinite(observation.lateral_lower_m[index]) ||
      !std::isfinite(observation.lateral_upper_m[index]) ||
      observation.lateral_upper_m[index] + kIdentityTolerance <
      observation.lateral_lower_m[index] ||
      (index > 0U && observation.elapsed_time_sec[index] <=
      observation.elapsed_time_sec[index - 1U]))
    {
      return false;
    }
  }
  return true;
}

struct LateralCorridorSample
{
  double lower_m{};
  double upper_m{};
};

std::optional<LateralCorridorSample> sample_overtake_corridor(
  const OvertakeDynamicCorridorObservation & observation,
  const double relative_time_sec) noexcept
{
  if (
    !overtake_corridor_shape_valid(observation) ||
    !std::isfinite(relative_time_sec) || relative_time_sec < 0.0 ||
    relative_time_sec > observation.elapsed_time_sec.back() +
    kIdentityTolerance)
  {
    return std::nullopt;
  }
  if (relative_time_sec <= kIdentityTolerance) {
    return LateralCorridorSample{
      observation.lateral_lower_m.front(),
      observation.lateral_upper_m.front()};
  }
  const auto upper = std::lower_bound(
    observation.elapsed_time_sec.begin(), observation.elapsed_time_sec.end(),
    relative_time_sec);
  if (upper == observation.elapsed_time_sec.end()) {
    return LateralCorridorSample{
      observation.lateral_lower_m.back(),
      observation.lateral_upper_m.back()};
  }
  const std::size_t upper_index = static_cast<std::size_t>(
    std::distance(observation.elapsed_time_sec.begin(), upper));
  if (std::abs(*upper - relative_time_sec) <= kIdentityTolerance) {
    return LateralCorridorSample{
      observation.lateral_lower_m[upper_index],
      observation.lateral_upper_m[upper_index]};
  }
  if (upper_index == 0U) {
    return std::nullopt;
  }
  const std::size_t lower_index = upper_index - 1U;
  const double duration =
    observation.elapsed_time_sec[upper_index] -
    observation.elapsed_time_sec[lower_index];
  const double fraction =
    (relative_time_sec - observation.elapsed_time_sec[lower_index]) / duration;
  return LateralCorridorSample{
    observation.lateral_lower_m[lower_index] + fraction *
    (observation.lateral_lower_m[upper_index] -
    observation.lateral_lower_m[lower_index]),
    observation.lateral_upper_m[lower_index] + fraction *
    (observation.lateral_upper_m[upper_index] -
    observation.lateral_upper_m[lower_index])};
}

std::optional<double> evaluate_overtake_corridor_segment(
  const OvertakeDynamicCorridorObservation & observation,
  const double start_time_sec, const double end_time_sec,
  const double start_lateral_m, const double end_lateral_m,
  const double tolerance_m) noexcept
{
  if (
    !std::isfinite(start_time_sec) || !std::isfinite(end_time_sec) ||
    !std::isfinite(start_lateral_m) || !std::isfinite(end_lateral_m) ||
    !std::isfinite(tolerance_m) || tolerance_m < 0.0 ||
    end_time_sec + kIdentityTolerance < start_time_sec)
  {
    return std::nullopt;
  }
  std::vector<double> sample_times{start_time_sec};
  for (const double knot_time_sec : observation.elapsed_time_sec) {
    if (
      knot_time_sec > start_time_sec + kIdentityTolerance &&
      knot_time_sec < end_time_sec - kIdentityTolerance)
    {
      sample_times.push_back(knot_time_sec);
    }
  }
  if (end_time_sec > start_time_sec + kIdentityTolerance) {
    sample_times.push_back(end_time_sec);
  }
  double minimum_reserve_m = std::numeric_limits<double>::infinity();
  for (const double sample_time_sec : sample_times) {
    const auto corridor = sample_overtake_corridor(
      observation, sample_time_sec);
    if (!corridor.has_value()) {
      return std::nullopt;
    }
    const double fraction = end_time_sec > start_time_sec + kIdentityTolerance ?
      (sample_time_sec - start_time_sec) /
      (end_time_sec - start_time_sec) : 0.0;
    const double lateral_m =
      start_lateral_m + fraction * (end_lateral_m - start_lateral_m);
    const double lower_reserve_m = lateral_m - corridor->lower_m;
    const double upper_reserve_m = corridor->upper_m - lateral_m;
    const double reserve_m = std::min(lower_reserve_m, upper_reserve_m);
    if (!std::isfinite(reserve_m) || reserve_m + tolerance_m < 0.0) {
      return std::nullopt;
    }
    minimum_reserve_m = std::min(minimum_reserve_m, std::max(0.0, reserve_m));
  }
  return minimum_reserve_m;
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

const char * to_string(const FollowCurrentWorldProofReason reason) noexcept
{
  switch (reason) {
    case FollowCurrentWorldProofReason::Accepted:
      return "accepted";
    case FollowCurrentWorldProofReason::InvalidInput:
      return "invalid-input";
    case FollowCurrentWorldProofReason::WindowRejected:
      return "window-rejected";
    case FollowCurrentWorldProofReason::ProgressLiftRejected:
      return "progress-lift-rejected";
    case FollowCurrentWorldProofReason::ControlPoseIdentityMismatch:
      return "control-pose-identity-mismatch";
    case FollowCurrentWorldProofReason::CourseFrameIdentityMismatch:
      return "course-frame-identity-mismatch";
    case FollowCurrentWorldProofReason::TargetObservationUnavailable:
      return "target-observation-unavailable";
    case FollowCurrentWorldProofReason::TargetIdentityMismatch:
      return "target-identity-mismatch";
    case FollowCurrentWorldProofReason::TargetTubeIdentityMismatch:
      return "target-tube-identity-mismatch";
    case FollowCurrentWorldProofReason::TargetHorizonUnavailable:
      return "target-horizon-unavailable";
    case FollowCurrentWorldProofReason::InitialHardGapViolation:
      return "initial-hard-gap-violation";
    case FollowCurrentWorldProofReason::CourseFrameUnavailable:
      return "course-frame-unavailable";
    case FollowCurrentWorldProofReason::DelayPrefixBlocked:
      return "delay-prefix-blocked";
    case FollowCurrentWorldProofReason::ConnectorBlocked:
      return "connector-blocked";
    case FollowCurrentWorldProofReason::StagePathBlocked:
      return "stage-path-blocked";
    case FollowCurrentWorldProofReason::StageGapViolation:
      return "stage-gap-violation";
    case FollowCurrentWorldProofReason::ProofRejected:
      return "proof-rejected";
  }
  return "unknown";
}

const char * to_string(const OvertakeCurrentWorldProofReason reason) noexcept
{
  switch (reason) {
    case OvertakeCurrentWorldProofReason::Accepted: return "accepted";
    case OvertakeCurrentWorldProofReason::InvalidInput: return "invalid-input";
    case OvertakeCurrentWorldProofReason::WindowRejected: return "window-rejected";
    case OvertakeCurrentWorldProofReason::ProgressLiftRejected:
      return "progress-lift-rejected";
    case OvertakeCurrentWorldProofReason::ControlPoseIdentityMismatch:
      return "control-pose-identity-mismatch";
    case OvertakeCurrentWorldProofReason::CourseFrameIdentityMismatch:
      return "course-frame-identity-mismatch";
    case OvertakeCurrentWorldProofReason::TargetObservationUnavailable:
      return "target-observation-unavailable";
    case OvertakeCurrentWorldProofReason::TargetIdentityMismatch:
      return "target-identity-mismatch";
    case OvertakeCurrentWorldProofReason::ExecutionSideMismatch:
      return "execution-side-mismatch";
    case OvertakeCurrentWorldProofReason::CorridorIdentityMismatch:
      return "corridor-identity-mismatch";
    case OvertakeCurrentWorldProofReason::CorridorHorizonUnavailable:
      return "corridor-horizon-unavailable";
    case OvertakeCurrentWorldProofReason::TargetReleaseUncertified:
      return "target-release-uncertified";
    case OvertakeCurrentWorldProofReason::InitialCorridorViolation:
      return "initial-corridor-violation";
    case OvertakeCurrentWorldProofReason::CourseFrameUnavailable:
      return "course-frame-unavailable";
    case OvertakeCurrentWorldProofReason::DelayPrefixBlocked:
      return "delay-prefix-blocked";
    case OvertakeCurrentWorldProofReason::ConnectorBlocked:
      return "connector-blocked";
    case OvertakeCurrentWorldProofReason::StagePathBlocked:
      return "stage-path-blocked";
    case OvertakeCurrentWorldProofReason::StageCorridorViolation:
      return "stage-corridor-violation";
    case OvertakeCurrentWorldProofReason::ProofRejected:
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

std::uint64_t fingerprint_follow_obstacle_observation(
  const FollowDynamicObstacleObservation & observation) noexcept
{
  if (!follow_observation_shape_valid(observation)) {
    return 0U;
  }
  FingerprintBuilder builder;
  builder.add_string("follow-target-tube-v1");
  builder.add_string(observation.target_id);
  builder.add_uint64(observation.observation_generation);
  builder.add_double(observation.observation_sec);
  builder.add_double(observation.hard_gap_m);
  builder.add_size(observation.elapsed_time_sec.size());
  for (std::size_t index = 0U; index < observation.elapsed_time_sec.size(); ++index) {
    builder.add_double(observation.elapsed_time_sec[index]);
    builder.add_double(observation.target_relative_progress_m[index]);
  }
  return builder.value();
}

std::uint64_t fingerprint_overtake_corridor_observation(
  const OvertakeDynamicCorridorObservation & observation) noexcept
{
  if (!overtake_corridor_shape_valid(observation)) {
    return 0U;
  }
  FingerprintBuilder builder;
  builder.add_string("overtake-current-corridor-v1");
  builder.add_string(observation.target_id);
  builder.add_uint64(observation.observation_generation);
  builder.add_double(observation.observation_sec);
  builder.add_uint64(observation.target_exclusion_encoded ? 1U : 0U);
  builder.add_uint64(observation.release_current_body_clear ? 1U : 0U);
  builder.add_uint64(observation.release_prediction_valid ? 1U : 0U);
  builder.add_uint64(observation.release_predicted_sweep_clear ? 1U : 0U);
  builder.add_size(observation.elapsed_time_sec.size());
  for (std::size_t index = 0U; index < observation.elapsed_time_sec.size(); ++index) {
    builder.add_double(observation.elapsed_time_sec[index]);
    builder.add_double(observation.lateral_lower_m[index]);
    builder.add_double(observation.lateral_upper_m[index]);
  }
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

FollowCurrentWorldProofResult build_follow_current_world_retained_proof(
  const plan::CanonicalExecutionPlan & execution_plan,
  const plan::CanonicalExecutionCursor & cursor,
  const FollowCurrentWorldProofRequest & request,
  const recovery_footprint::OccupancyGrid & wall_grid,
  const recovery_footprint::FootprintExtents & footprint)
{
  FollowCurrentWorldProofResult result;
  const auto window = retained::build_retained_execution_window(
    execution_plan, cursor);
  if (!window.window.has_value()) {
    result.reason = FollowCurrentWorldProofReason::WindowRejected;
    return result;
  }
  if (!wall_grid.valid() || !footprint.valid() ||
    request.measured_to_control_path.empty() ||
    !std::isfinite(request.swept_step_m) || request.swept_step_m <= 0.0)
  {
    return result;
  }
  if (!request.target.current ||
    !follow_observation_shape_valid(request.target))
  {
    result.reason = FollowCurrentWorldProofReason::TargetObservationUnavailable;
    return result;
  }
  if (
    request.current.intent != mpcc_execution_contract::ControlIntent::Follow ||
    request.current.target_id != request.target.target_id ||
    request.current.target_obstacle_generation !=
    request.target.observation_generation ||
    execution_plan.problem.target_id != request.target.target_id ||
    execution_plan.problem.target_obstacle_generation == 0U)
  {
    result.reason = FollowCurrentWorldProofReason::TargetIdentityMismatch;
    return result;
  }
  if (
    request.target.tube_id == 0U ||
    request.current.obstacle_tube_id != request.target.tube_id ||
    fingerprint_follow_obstacle_observation(request.target) !=
    request.target.tube_id)
  {
    result.reason = FollowCurrentWorldProofReason::TargetTubeIdentityMismatch;
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
    result.reason = FollowCurrentWorldProofReason::ProgressLiftRejected;
    return result;
  }
  if (!same_pose(request.measured_to_control_path.back(), request.control_pose) ||
    fingerprint_control_pose_path(
      request.measured_to_control_path, request.control_pose) !=
    request.current.control_pose_id)
  {
    result.reason = FollowCurrentWorldProofReason::ControlPoseIdentityMismatch;
    return result;
  }
  if (fingerprint_course_frame_window(request.course_frame_knots) !=
    request.current.course_frame_window_id)
  {
    result.reason = FollowCurrentWorldProofReason::CourseFrameIdentityMismatch;
    return result;
  }
  const auto expected_current_pose = reconstruct_pose(
    request.course_frame_knots, window.window->expected_current_state);
  if (!expected_current_pose.has_value()) {
    result.reason = FollowCurrentWorldProofReason::CourseFrameUnavailable;
    return result;
  }

  const auto current_target_relative = sample_follow_target_progress(
    request.target, 0.0);
  if (!current_target_relative.has_value()) {
    result.reason = FollowCurrentWorldProofReason::TargetHorizonUnavailable;
    return result;
  }
  const double current_target_absolute =
    lift.lifted_progress_m + current_target_relative.value();
  const double expected_current_ego =
    window.window->expected_current_state.progress_m +
    window.window->expected_current_state.lag_m;
  const double current_gap = std::min(
    current_target_relative.value(),
    current_target_absolute - expected_current_ego);
  result.minimum_gap_m = current_gap;
  if (!std::isfinite(current_gap) ||
    current_gap + kIdentityTolerance < request.target.hard_gap_m)
  {
    result.reason = FollowCurrentWorldProofReason::InitialHardGapViolation;
    return result;
  }

  retained::RetainedExecutionProofRequest proof_request;
  proof_request.current = request.current;
  proof_request.measured_course_progress_m = request.measured_course_progress_m;
  proof_request.progress_continuity_tolerance_m =
    request.progress_continuity_tolerance_m;
  const auto delay_wall = recovery_footprint::evaluate_clear_footprint_path(
    wall_grid, footprint, request.measured_to_control_path,
    request.swept_step_m);
  proof_request.measured_to_control_prefix = make_segment_evaluation(
    request.current, lift.lifted_progress_m, lift.lifted_progress_m,
    delay_wall);
  proof_request.measured_to_control_prefix.obstacles_clear = true;
  proof_request.measured_to_control_prefix.minimum_obstacle_clearance_m =
    current_gap - request.target.hard_gap_m;
  if (!delay_wall.valid || !delay_wall.clear) {
    result.reason = FollowCurrentWorldProofReason::DelayPrefixBlocked;
    return result;
  }

  const std::vector<recovery_footprint::Pose2D> connector_path{
    request.control_pose, expected_current_pose.value()};
  const auto connector_wall = recovery_footprint::evaluate_clear_footprint_path(
    wall_grid, footprint, connector_path, request.swept_step_m);
  proof_request.control_to_retained_connector = make_segment_evaluation(
    request.current, lift.lifted_progress_m,
    window.window->expected_current_progress_m, connector_wall);
  proof_request.control_to_retained_connector.obstacles_clear = true;
  proof_request.control_to_retained_connector.minimum_obstacle_clearance_m =
    current_gap - request.target.hard_gap_m;
  if (!connector_wall.valid || !connector_wall.clear) {
    result.reason = FollowCurrentWorldProofReason::ConnectorBlocked;
    return result;
  }

  auto previous_pose = expected_current_pose.value();
  proof_request.stage_evaluations.reserve(window.window->samples.size());
  for (std::size_t index = 0U; index < window.window->samples.size(); ++index) {
    const auto & sample = window.window->samples[index];
    const auto target_relative = sample_follow_target_progress(
      request.target, sample.relative_time_sec);
    if (!target_relative.has_value()) {
      result.reason = FollowCurrentWorldProofReason::TargetHorizonUnavailable;
      result.rejected_stage_index = index;
      return result;
    }
    const double target_absolute =
      lift.lifted_progress_m + target_relative.value();
    const double ego_absolute =
      sample.endpoint.progress_m + sample.endpoint.lag_m;
    const double gap = target_absolute - ego_absolute;
    result.minimum_gap_m = std::min(result.minimum_gap_m, gap);
    if (!std::isfinite(gap) ||
      gap + kIdentityTolerance < request.target.hard_gap_m)
    {
      result.reason = FollowCurrentWorldProofReason::StageGapViolation;
      result.rejected_stage_index = index;
      return result;
    }
    const auto endpoint_pose = reconstruct_pose(
      request.course_frame_knots, sample.endpoint);
    if (!endpoint_pose.has_value()) {
      result.reason = FollowCurrentWorldProofReason::CourseFrameUnavailable;
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
    evaluation.minimum_obstacle_clearance_m =
      gap - request.target.hard_gap_m;
    proof_request.stage_evaluations.push_back(evaluation);
    if (!stage_wall.valid || !stage_wall.clear) {
      result.reason = FollowCurrentWorldProofReason::StagePathBlocked;
      result.rejected_stage_index = index;
      return result;
    }
    previous_pose = endpoint_pose.value();
  }

  auto proof = retained::build_retained_execution_proof(
    execution_plan, cursor, proof_request);
  result.proof_reason = proof.reason;
  if (!proof.proof.has_value()) {
    result.reason = FollowCurrentWorldProofReason::ProofRejected;
    return result;
  }
  result.reason = FollowCurrentWorldProofReason::Accepted;
  result.proof = std::move(proof.proof);
  return result;
}

OvertakeCurrentWorldProofResult build_overtake_current_world_retained_proof(
  const plan::CanonicalExecutionPlan & execution_plan,
  const plan::CanonicalExecutionCursor & cursor,
  const OvertakeCurrentWorldProofRequest & request,
  const recovery_footprint::OccupancyGrid & wall_grid,
  const recovery_footprint::FootprintExtents & footprint)
{
  OvertakeCurrentWorldProofResult result;
  const auto window = retained::build_retained_execution_window(
    execution_plan, cursor);
  if (!window.window.has_value()) {
    result.reason = OvertakeCurrentWorldProofReason::WindowRejected;
    return result;
  }
  if (
    !wall_grid.valid() || !footprint.valid() ||
    request.measured_to_control_path.empty() ||
    !std::isfinite(request.measured_lateral_m) ||
    !std::isfinite(request.lateral_tolerance_m) ||
    request.lateral_tolerance_m < 0.0 ||
    !std::isfinite(request.swept_step_m) || request.swept_step_m <= 0.0)
  {
    return result;
  }
  if (!request.corridor.current ||
    !overtake_corridor_shape_valid(request.corridor))
  {
    result.reason =
      OvertakeCurrentWorldProofReason::TargetObservationUnavailable;
    return result;
  }
  const bool overtake_intent =
    request.current.intent == mpcc_execution_contract::ControlIntent::ShiftOut ||
    request.current.intent == mpcc_execution_contract::ControlIntent::Pass ||
    request.current.intent == mpcc_execution_contract::ControlIntent::Return;
  if (
    !overtake_intent ||
    request.current.target_id != request.corridor.target_id ||
    request.current.target_obstacle_generation !=
    request.corridor.observation_generation ||
    execution_plan.problem.target_id != request.corridor.target_id ||
    execution_plan.problem.target_obstacle_generation == 0U)
  {
    result.reason = OvertakeCurrentWorldProofReason::TargetIdentityMismatch;
    return result;
  }
  const auto semantic_reason = retained::validate_retained_semantic_identity(
    execution_plan, request.current);
  if (semantic_reason != retained::RetainedExecutionProofReason::Accepted) {
    result.reason =
      semantic_reason == retained::RetainedExecutionProofReason::ExecutionSideMismatch ?
      OvertakeCurrentWorldProofReason::ExecutionSideMismatch :
      OvertakeCurrentWorldProofReason::TargetIdentityMismatch;
    return result;
  }
  if (
    request.corridor.tube_id == 0U ||
    request.current.obstacle_tube_id != request.corridor.tube_id ||
    fingerprint_overtake_corridor_observation(request.corridor) !=
    request.corridor.tube_id)
  {
    result.reason = OvertakeCurrentWorldProofReason::CorridorIdentityMismatch;
    return result;
  }
  if (
    !request.corridor.target_exclusion_encoded &&
    !(request.corridor.release_current_body_clear &&
    request.corridor.release_prediction_valid &&
    request.corridor.release_predicted_sweep_clear))
  {
    result.reason =
      OvertakeCurrentWorldProofReason::TargetReleaseUncertified;
    return result;
  }
  if (!retained::current_execution_provenance_complete(request.current)) {
    return result;
  }
  if (
    window.window->samples.empty() ||
    window.window->samples.back().relative_time_sec >
    request.corridor.elapsed_time_sec.back() + kIdentityTolerance)
  {
    result.reason =
      OvertakeCurrentWorldProofReason::CorridorHorizonUnavailable;
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
    result.reason = OvertakeCurrentWorldProofReason::ProgressLiftRejected;
    return result;
  }
  if (!same_pose(request.measured_to_control_path.back(), request.control_pose) ||
    fingerprint_control_pose_path(
      request.measured_to_control_path, request.control_pose) !=
    request.current.control_pose_id)
  {
    result.reason =
      OvertakeCurrentWorldProofReason::ControlPoseIdentityMismatch;
    return result;
  }
  if (fingerprint_course_frame_window(request.course_frame_knots) !=
    request.current.course_frame_window_id)
  {
    result.reason =
      OvertakeCurrentWorldProofReason::CourseFrameIdentityMismatch;
    return result;
  }
  const auto expected_current_pose = reconstruct_pose(
    request.course_frame_knots, window.window->expected_current_state);
  if (!expected_current_pose.has_value()) {
    result.reason = OvertakeCurrentWorldProofReason::CourseFrameUnavailable;
    return result;
  }
  const auto initial_corridor = evaluate_overtake_corridor_segment(
    request.corridor, 0.0, 0.0, request.measured_lateral_m,
    request.measured_lateral_m, request.lateral_tolerance_m);
  const auto expected_current_corridor = evaluate_overtake_corridor_segment(
    request.corridor, 0.0, 0.0,
    window.window->expected_current_state.lateral_m,
    window.window->expected_current_state.lateral_m,
    request.lateral_tolerance_m);
  if (!initial_corridor.has_value() || !expected_current_corridor.has_value()) {
    result.reason =
      OvertakeCurrentWorldProofReason::InitialCorridorViolation;
    return result;
  }
  result.minimum_corridor_reserve_m = std::min(
    initial_corridor.value(), expected_current_corridor.value());

  retained::RetainedExecutionProofRequest proof_request;
  proof_request.current = request.current;
  proof_request.measured_course_progress_m = request.measured_course_progress_m;
  proof_request.progress_continuity_tolerance_m =
    request.progress_continuity_tolerance_m;
  const auto delay_wall = recovery_footprint::evaluate_clear_footprint_path(
    wall_grid, footprint, request.measured_to_control_path,
    request.swept_step_m);
  proof_request.measured_to_control_prefix = make_segment_evaluation(
    request.current, lift.lifted_progress_m, lift.lifted_progress_m,
    delay_wall);
  proof_request.measured_to_control_prefix.minimum_obstacle_clearance_m =
    initial_corridor.value();
  if (!delay_wall.valid || !delay_wall.clear) {
    result.reason = OvertakeCurrentWorldProofReason::DelayPrefixBlocked;
    return result;
  }

  const std::vector<recovery_footprint::Pose2D> connector_path{
    request.control_pose, expected_current_pose.value()};
  const auto connector_wall = recovery_footprint::evaluate_clear_footprint_path(
    wall_grid, footprint, connector_path, request.swept_step_m);
  proof_request.control_to_retained_connector = make_segment_evaluation(
    request.current, lift.lifted_progress_m,
    window.window->expected_current_progress_m, connector_wall);
  proof_request.control_to_retained_connector.minimum_obstacle_clearance_m =
    std::min(initial_corridor.value(), expected_current_corridor.value());
  if (!connector_wall.valid || !connector_wall.clear) {
    result.reason = OvertakeCurrentWorldProofReason::ConnectorBlocked;
    return result;
  }

  auto previous_pose = expected_current_pose.value();
  double previous_time_sec = 0.0;
  double previous_lateral_m = window.window->expected_current_state.lateral_m;
  proof_request.stage_evaluations.reserve(window.window->samples.size());
  for (std::size_t index = 0U; index < window.window->samples.size(); ++index) {
    const auto & sample = window.window->samples[index];
    const auto corridor_reserve = evaluate_overtake_corridor_segment(
      request.corridor, previous_time_sec, sample.relative_time_sec,
      previous_lateral_m, sample.endpoint.lateral_m,
      request.lateral_tolerance_m);
    if (!corridor_reserve.has_value()) {
      result.reason =
        OvertakeCurrentWorldProofReason::StageCorridorViolation;
      result.rejected_stage_index = index;
      return result;
    }
    result.minimum_corridor_reserve_m = std::min(
      result.minimum_corridor_reserve_m, corridor_reserve.value());
    const auto endpoint_pose = reconstruct_pose(
      request.course_frame_knots, sample.endpoint);
    if (!endpoint_pose.has_value()) {
      result.reason = OvertakeCurrentWorldProofReason::CourseFrameUnavailable;
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
    evaluation.minimum_obstacle_clearance_m = corridor_reserve.value();
    proof_request.stage_evaluations.push_back(evaluation);
    if (!stage_wall.valid || !stage_wall.clear) {
      result.reason = OvertakeCurrentWorldProofReason::StagePathBlocked;
      result.rejected_stage_index = index;
      return result;
    }
    previous_pose = endpoint_pose.value();
    previous_time_sec = sample.relative_time_sec;
    previous_lateral_m = sample.endpoint.lateral_m;
  }

  auto proof = retained::build_retained_execution_proof(
    execution_plan, cursor, proof_request);
  result.proof_reason = proof.reason;
  if (!proof.proof.has_value()) {
    result.reason = OvertakeCurrentWorldProofReason::ProofRejected;
    return result;
  }
  result.reason = OvertakeCurrentWorldProofReason::Accepted;
  result.proof = std::move(proof.proof);
  return result;
}

}  // namespace multi_purpose_mpc_ros::canonical_retained_world_revalidation
