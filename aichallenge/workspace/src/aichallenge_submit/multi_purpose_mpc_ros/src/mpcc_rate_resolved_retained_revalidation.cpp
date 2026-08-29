#include "multi_purpose_mpc_ros/mpcc_rate_resolved_retained_revalidation.hpp"

#include "multi_purpose_mpc_ros/mpc_stage_geometry.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_physical_wall.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_retained_revalidation
{
namespace dynamic_proof = mpcc_rate_resolved_dynamic_proof;
namespace
{

constexpr double kIdentityTolerance = 1e-9;

bool finite_pose(const recovery::Pose2D & pose) noexcept
{
  return std::isfinite(pose.x_m) && std::isfinite(pose.y_m) &&
         std::isfinite(pose.yaw_rad);
}

bool same_pose(
  const recovery::Pose2D & lhs, const recovery::Pose2D & rhs) noexcept
{
  return finite_pose(lhs) && finite_pose(rhs) &&
         std::abs(lhs.x_m - rhs.x_m) <= kIdentityTolerance &&
         std::abs(lhs.y_m - rhs.y_m) <= kIdentityTolerance &&
         std::abs(lhs.yaw_rad - rhs.yaw_rad) <= kIdentityTolerance;
}

bool same_footprint(
  const recovery::FootprintExtents & lhs,
  const recovery::FootprintExtents & rhs) noexcept
{
  return lhs.valid() && rhs.valid() &&
         lhs.front_extent_m == rhs.front_extent_m &&
         lhs.rear_extent_m == rhs.rear_extent_m &&
         lhs.left_extent_m == rhs.left_extent_m &&
         lhs.right_extent_m == rhs.right_extent_m &&
         lhs.margin_m == rhs.margin_m;
}

artifact::PredictedState interpolate_expected_state(
  const artifact::ExecutionArtifact & execution,
  const artifact::Cursor & cursor) noexcept
{
  const auto & start =
    execution.predicted_states[cursor.control_stage_index];
  const auto & end =
    execution.predicted_states[cursor.control_stage_index + 1U];
  const double duration =
    execution.control_stages[cursor.control_stage_index].duration_sec;
  const double fraction = cursor.stage_elapsed_sec / duration;
  const auto interpolate = [fraction](const double lhs, const double rhs) {
      return lhs + fraction * (rhs - lhs);
    };
  return artifact::PredictedState{
    interpolate(start.lateral_m, end.lateral_m),
    interpolate(start.lag_m, end.lag_m),
    interpolate(start.heading_offset_rad, end.heading_offset_rad),
    interpolate(start.velocity_mps, end.velocity_mps),
    interpolate(start.progress_m, end.progress_m),
    interpolate(start.steering_rad, end.steering_rad),
    interpolate(
      start.response_steering_rad, end.response_steering_rad)};
}

struct ExactPhysicalState
{
  double lateral_m{};
  double lag_m{};
  double heading_offset_rad{};
  double velocity_mps{};
  double absolute_progress_m{};
};

ExactPhysicalState interpolate_exact_physical_state(
  const ExactPhysicalState & start, const ExactPhysicalState & end,
  const double fraction) noexcept
{
  const auto interpolate = [fraction](const double lhs, const double rhs) {
      return lhs + fraction * (rhs - lhs);
    };
  return ExactPhysicalState{
    interpolate(start.lateral_m, end.lateral_m),
    interpolate(start.lag_m, end.lag_m),
    interpolate(start.heading_offset_rad, end.heading_offset_rad),
    interpolate(start.velocity_mps, end.velocity_mps),
    interpolate(start.absolute_progress_m, end.absolute_progress_m)};
}

ExactPhysicalState exact_physical_state_at(
  const race_mpcc_foundation::ExactPhysicalExecutionTrajectory & trajectory,
  const std::size_t index) noexcept
{
  return ExactPhysicalState{
    trajectory.lateral_m[index],
    trajectory.lag_m[index],
    trajectory.heading_offset_rad[index],
    trajectory.velocity_mps[index],
    trajectory.progress_m[index]};
}

ExactPhysicalState exact_physical_state_at(
  const physical::Snapshot & source, const std::size_t index) noexcept
{
  return exact_physical_state_at(source.trajectory, index);
}

std::optional<ExactPhysicalState> sample_exact_physical_state(
  const artifact::ExecutionArtifact & execution,
  const physical::Snapshot & source, const double elapsed_sec) noexcept
{
  const auto & trajectory = source.trajectory;
  if (
    !std::isfinite(elapsed_sec) || elapsed_sec < 0.0 ||
    trajectory.elapsed_time_sec.empty() ||
    elapsed_sec > trajectory.elapsed_time_sec.back() + kIdentityTolerance ||
    execution.predicted_states.empty() ||
    std::abs(
      trajectory.progress_origin_m - execution.course_progress_origin_m) >
    std::max(kIdentityTolerance, source.bound_tolerance_m))
  {
    return std::nullopt;
  }

  const auto upper = std::lower_bound(
    trajectory.elapsed_time_sec.begin(), trajectory.elapsed_time_sec.end(),
    elapsed_sec);
  if (upper == trajectory.elapsed_time_sec.end()) {
    return exact_physical_state_at(source, trajectory.elapsed_time_sec.size() - 1U);
  }
  const std::size_t upper_index = static_cast<std::size_t>(
    std::distance(trajectory.elapsed_time_sec.begin(), upper));
  if (std::abs(*upper - elapsed_sec) <= kIdentityTolerance) {
    return exact_physical_state_at(source, upper_index);
  }

  ExactPhysicalState lower_state;
  double lower_time_sec{};
  if (upper_index == 0U) {
    const auto & initial = execution.predicted_states.front();
    lower_state = ExactPhysicalState{
      initial.lateral_m,
      initial.lag_m,
      initial.heading_offset_rad,
      initial.velocity_mps,
      execution.course_progress_origin_m + initial.progress_m};
  } else {
    lower_time_sec = trajectory.elapsed_time_sec[upper_index - 1U];
    lower_state = exact_physical_state_at(source, upper_index - 1U);
  }
  const double duration_sec = *upper - lower_time_sec;
  if (!std::isfinite(duration_sec) || duration_sec <= 0.0) {
    return std::nullopt;
  }
  const double fraction = (elapsed_sec - lower_time_sec) / duration_sec;
  return interpolate_exact_physical_state(
    lower_state, exact_physical_state_at(source, upper_index), fraction);
}

artifact::PredictedState as_predicted_state(
  const ExactPhysicalState & physical_state,
  const artifact::PredictedState & command_state,
  const double progress_origin_m) noexcept
{
  return artifact::PredictedState{
    physical_state.lateral_m,
    physical_state.lag_m,
    physical_state.heading_offset_rad,
    physical_state.velocity_mps,
    physical_state.absolute_progress_m - progress_origin_m,
    command_state.steering_rad,
    command_state.response_steering_rad};
}

struct LiftResult
{
  bool accepted{false};
  double progress_m{std::numeric_limits<double>::quiet_NaN()};
  double difference_m{std::numeric_limits<double>::quiet_NaN()};
  long lap_offset{};
};

LiftResult lift_progress(
  const double measured_progress_m, const double retained_progress_m,
  const double path_length_m, const double tolerance_m,
  const bool circular) noexcept
{
  LiftResult result;
  if (!std::isfinite(measured_progress_m) ||
    !std::isfinite(retained_progress_m) ||
    !std::isfinite(path_length_m) || path_length_m <= 0.0 ||
    !std::isfinite(tolerance_m) || tolerance_m < 0.0)
  {
    return result;
  }
  if (!circular) {
    result.progress_m = measured_progress_m;
    result.difference_m = measured_progress_m - retained_progress_m;
    if (std::abs(measured_progress_m - retained_progress_m) >
      tolerance_m + kIdentityTolerance)
    {
      return result;
    }
    result.accepted = true;
    result.progress_m = measured_progress_m;
    return result;
  }
  if (tolerance_m >= 0.5 * path_length_m) {
    return result;
  }
  const double raw_offset =
    (retained_progress_m - measured_progress_m) / path_length_m;
  if (raw_offset < static_cast<double>(std::numeric_limits<long>::min()) ||
    raw_offset > static_cast<double>(std::numeric_limits<long>::max()))
  {
    return result;
  }
  const long lap_offset = std::lround(raw_offset);
  const double lifted = measured_progress_m +
    static_cast<double>(lap_offset) * path_length_m;
  result.progress_m = lifted;
  result.difference_m = lifted - retained_progress_m;
  result.lap_offset = lap_offset;
  if (std::abs(lifted - retained_progress_m) >
    tolerance_m + kIdentityTolerance)
  {
    return result;
  }
  result.accepted = true;
  return result;
}

std::optional<recovery::Pose2D> reconstruct_pose(
  const physical::Snapshot & source,
  const artifact::PredictedState & state) noexcept
{
  const double absolute_progress_m =
    source.trajectory.progress_origin_m + state.progress_m;
  const auto frame = mpc_stage_geometry::sample_course_frame(
    source.course_frame_knots, absolute_progress_m,
    std::max(kIdentityTolerance, source.bound_tolerance_m));
  if (!frame.has_value()) {
    return std::nullopt;
  }
  const auto world = contract::reconstruct_planar_pose_from_frenet(
    contract::PlanarPose{frame->x_m, frame->y_m, frame->heading_rad},
    contract::FrenetPose{
        state.lateral_m, state.lag_m, state.heading_offset_rad});
  if (!world.has_value()) {
    return std::nullopt;
  }
  return recovery::Pose2D{world->x_m, world->y_m, world->yaw_rad};
}

double wrap_to_pi(const double angle) noexcept
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

bool follow_target_observation_valid(
  const FollowTargetObservation & observation) noexcept
{
  if (
    observation.target_id.empty() || observation.observation_generation == 0U ||
    !std::isfinite(observation.observed_sec) || observation.observed_sec < 0.0 ||
    !std::isfinite(observation.current_target_gap_m) ||
    observation.current_target_gap_m < 0.0 ||
    !std::isfinite(observation.hard_gap_m) || observation.hard_gap_m < 0.0 ||
    !std::isfinite(observation.target_speed_mps) ||
    observation.target_speed_mps < 0.0 ||
    observation.elapsed_time_sec.size() < 2U ||
    observation.elapsed_time_sec.size() !=
    observation.target_progress_from_current_origin_m.size() ||
    !std::isfinite(observation.elapsed_time_sec.front()) ||
    std::abs(observation.elapsed_time_sec.front()) > kIdentityTolerance ||
    !std::isfinite(observation.target_progress_from_current_origin_m.front()))
  {
    return false;
  }
  for (std::size_t index = 1U; index < observation.elapsed_time_sec.size(); ++index) {
    if (
      !std::isfinite(observation.elapsed_time_sec[index]) ||
      observation.elapsed_time_sec[index] <=
      observation.elapsed_time_sec[index - 1U] ||
      !std::isfinite(
        observation.target_progress_from_current_origin_m[index]) ||
      observation.target_progress_from_current_origin_m[index] +
      kIdentityTolerance <
      observation.target_progress_from_current_origin_m[index - 1U])
    {
      return false;
    }
  }
  return true;
}

std::optional<double> sample_follow_target_progress(
  const FollowTargetObservation & observation,
  const double elapsed_time_sec) noexcept
{
  if (!follow_target_observation_valid(observation) ||
    !std::isfinite(elapsed_time_sec) || elapsed_time_sec < 0.0)
  {
    return std::nullopt;
  }
  if (elapsed_time_sec <= kIdentityTolerance) {
    return observation.target_progress_from_current_origin_m.front();
  }
  const auto upper = std::lower_bound(
    observation.elapsed_time_sec.begin(), observation.elapsed_time_sec.end(),
    elapsed_time_sec);
  if (upper == observation.elapsed_time_sec.end()) {
    const double extension_sec =
      elapsed_time_sec - observation.elapsed_time_sec.back();
    const double extended =
      observation.target_progress_from_current_origin_m.back() +
      observation.target_speed_mps * extension_sec;
    if (!std::isfinite(extended) ||
      extended + kIdentityTolerance <
      observation.target_progress_from_current_origin_m.back())
    {
      return std::nullopt;
    }
    return extended;
  }
  const std::size_t upper_index = static_cast<std::size_t>(
    std::distance(observation.elapsed_time_sec.begin(), upper));
  if (std::abs(*upper - elapsed_time_sec) <= kIdentityTolerance) {
    return observation.target_progress_from_current_origin_m[upper_index];
  }
  if (upper_index == 0U) {
    return std::nullopt;
  }
  const std::size_t lower_index = upper_index - 1U;
  const double duration_sec =
    observation.elapsed_time_sec[upper_index] -
    observation.elapsed_time_sec[lower_index];
  const double fraction =
    (elapsed_time_sec - observation.elapsed_time_sec[lower_index]) /
    duration_sec;
  const double sampled =
    observation.target_progress_from_current_origin_m[lower_index] +
    fraction * (
    observation.target_progress_from_current_origin_m[upper_index] -
    observation.target_progress_from_current_origin_m[lower_index]);
  return std::isfinite(sampled) ? std::optional<double>{sampled} : std::nullopt;
}

}  // namespace

std::optional<double> resolve_peer_circle_radius(
  const double forbidden_ego_center_distance_m,
  const recovery::FootprintExtents & ego_footprint,
  const double peer_uncertainty_margin_m) noexcept
{
  if (
    !std::isfinite(forbidden_ego_center_distance_m) ||
    forbidden_ego_center_distance_m < 0.0 || !ego_footprint.valid() ||
    !std::isfinite(peer_uncertainty_margin_m) ||
    peer_uncertainty_margin_m < 0.0)
  {
    return std::nullopt;
  }
  const double ego_body_lateral_extent_m = std::max(
    ego_footprint.left_extent_m, ego_footprint.right_extent_m);
  const double peer_body_radius_m =
    forbidden_ego_center_distance_m - ego_body_lateral_extent_m;
  if (peer_body_radius_m < -kIdentityTolerance) {
    return std::nullopt;
  }
  const double resolved_radius_m =
    std::max(0.0, peer_body_radius_m) + peer_uncertainty_margin_m;
  return std::isfinite(resolved_radius_m) ?
    std::optional<double>{resolved_radius_m} : std::nullopt;
}

std::optional<FollowTargetObservation> build_follow_target_observation(
  const FollowTargetObservationBuildRequest & request) noexcept
{
  if (
    request.target_id.empty() || request.observation_generation == 0U ||
    !std::isfinite(request.observed_sec) || request.observed_sec < 0.0 ||
    !std::isfinite(request.current_target_gap_m) ||
    request.current_target_gap_m < 0.0 ||
    !std::isfinite(request.current_ego_progress_offset_m) ||
    !std::isfinite(request.hard_gap_m) || request.hard_gap_m < 0.0 ||
    !std::isfinite(request.target_speed_mps) || request.target_speed_mps < 0.0 ||
    request.stage_duration_sec.empty() ||
    std::any_of(
      request.stage_duration_sec.begin(), request.stage_duration_sec.end(),
      [](const double duration_sec) {
        return !std::isfinite(duration_sec) || duration_sec <= 0.0;
      }))
  {
    return std::nullopt;
  }

  FollowTargetObservation observation;
  observation.target_id = request.target_id;
  observation.observation_generation = request.observation_generation;
  observation.observed_sec = request.observed_sec;
  observation.current_target_gap_m = request.current_target_gap_m;
  observation.hard_gap_m = request.hard_gap_m;
  observation.target_speed_mps = request.target_speed_mps;
  observation.current = request.current;
  observation.elapsed_time_sec.reserve(request.stage_duration_sec.size() + 1U);
  observation.target_progress_from_current_origin_m.reserve(
    request.stage_duration_sec.size() + 1U);
  double elapsed_sec = 0.0;
  for (std::size_t state = 0U; state <= request.stage_duration_sec.size(); ++state) {
    if (state > 0U) {
      elapsed_sec += request.stage_duration_sec[state - 1U];
    }
    observation.elapsed_time_sec.push_back(elapsed_sec);
    observation.target_progress_from_current_origin_m.push_back(
      request.current_ego_progress_offset_m + request.current_target_gap_m +
      request.target_speed_mps * elapsed_sec);
  }
  return observation;
}

const char * to_string(const Reason reason) noexcept
{
  switch (reason) {
    case Reason::Accepted: return "accepted";
    case Reason::MissingPlan: return "missing-plan";
    case Reason::InvalidPlan: return "invalid-plan";
    case Reason::ExecutionClockInvalid: return "execution-clock-invalid";
    case Reason::CursorUnavailable: return "cursor-unavailable";
    case Reason::IntentMismatch: return "intent-mismatch";
    case Reason::DynamicObservationUnavailable:
      return "dynamic-observation-unavailable";
    case Reason::DynamicObservationInvalid:
      return "dynamic-observation-invalid";
    case Reason::FollowTargetObservationUnavailable:
      return "follow-target-observation-unavailable";
    case Reason::FollowTargetObservationInvalid:
      return "follow-target-observation-invalid";
    case Reason::FollowTargetIdentityMismatch:
      return "follow-target-identity-mismatch";
    case Reason::FollowTargetHorizonUnavailable:
      return "follow-target-horizon-unavailable";
    case Reason::FollowInitialHardGapViolation:
      return "follow-initial-hard-gap-violation";
    case Reason::FollowStageGapViolation:
      return "follow-stage-gap-violation";
    case Reason::DynamicPathInvalid: return "dynamic-path-invalid";
    case Reason::DynamicPathBlocked: return "dynamic-path-blocked";
    case Reason::StaticWorldMismatch: return "static-world-mismatch";
    case Reason::InvalidCurrentState: return "invalid-current-state";
    case Reason::ProgressLiftRejected: return "progress-lift-rejected";
    case Reason::CourseFrameUnavailable: return "course-frame-unavailable";
    case Reason::ActuationRejected: return "actuation-rejected";
    case Reason::SteeringUnreachable: return "steering-unreachable";
    case Reason::ControlPathInvalid: return "control-path-invalid";
    case Reason::DelayPrefixBlocked: return "delay-prefix-blocked";
    case Reason::ConnectorBlocked: return "connector-blocked";
    case Reason::ContinuationRejected: return "continuation-rejected";
    case Reason::ContinuationWallBlocked:
      return "continuation-wall-blocked";
    case Reason::TerminalContingencyUnavailable:
      return "terminal-contingency-unavailable";
    case Reason::Count: break;
  }
  return "unknown";
}

const char * to_string(const ExecutionClockKind kind) noexcept
{
  switch (kind) {
    case ExecutionClockKind::Unknown:
      return "unknown";
    case ExecutionClockKind::BootstrapCandidate:
      return "bootstrap-candidate";
    case ExecutionClockKind::TimeAlignedCandidate:
      return "time-aligned-candidate";
    case ExecutionClockKind::PublishedPlan:
      return "published-plan";
  }
  return "unknown";
}

artifact::Cursor resolve_execution_cursor(
  const artifact::ExecutionArtifact & execution,
  const double current_control_origin_sec,
  const ExecutionClock & clock) noexcept
{
  if (!std::isfinite(current_control_origin_sec) ||
    current_control_origin_sec < 0.0)
  {
    return artifact::Cursor{};
  }
  double elapsed_sec{};
  switch (clock.kind) {
    case ExecutionClockKind::BootstrapCandidate:
      // No artifact command has crossed the publisher and no predecessor plan
      // exists. Candidate age is proof provenance, not executed control.
      elapsed_sec = 0.0;
      break;
    case ExecutionClockKind::TimeAlignedCandidate:
      // The solver labels state zero with prediction_origin_sec, not with the
      // later instant at which its asynchronous result is consumed.  Keeping
      // every unpublished result at cursor zero compares the current vehicle
      // with a past lateral cross-section and makes a moving candidate
      // joinable only after the vehicle stops.  Select the time-aligned suffix
      // here; evaluate() subsequently rebuilds that suffix from the current
      // physical state and proves actuator, wall and obstacle continuity.  No
      // authority is inherited from the skipped, never-published prefix.
      elapsed_sec = std::max(
        0.0, current_control_origin_sec - execution.prediction_origin_sec);
      break;
    case ExecutionClockKind::PublishedPlan:
      if (!std::isfinite(clock.first_published_control_origin_sec) ||
        clock.first_published_control_origin_sec < 0.0 ||
        !std::isfinite(clock.first_published_artifact_elapsed_sec) ||
        clock.first_published_artifact_elapsed_sec < 0.0 ||
        current_control_origin_sec + kIdentityTolerance <
        clock.first_published_control_origin_sec)
      {
        return artifact::Cursor{};
      }
      elapsed_sec = clock.first_published_artifact_elapsed_sec + std::max(
        0.0,
        current_control_origin_sec -
        clock.first_published_control_origin_sec);
      break;
    case ExecutionClockKind::Unknown:
      return artifact::Cursor{};
  }
  return artifact::resolve_cursor(
    execution, execution.prediction_origin_sec + elapsed_sec);
}

const char * to_string(const StaticWallProofScope scope) noexcept
{
  switch (scope) {
    case StaticWallProofScope::FullSuffix:
      return "full-suffix";
    case StaticWallProofScope::CurrentStagePrefix:
      return "current-stage-prefix";
  }
  return "unknown";
}

const char * to_string(const DynamicObstacleProofScope scope) noexcept
{
  switch (scope) {
    case DynamicObstacleProofScope::FullSuffix:
      return "full-suffix";
    case DynamicObstacleProofScope::CurrentStagePrefix:
      return "current-stage-prefix";
  }
  return "unknown";
}

Result evaluate(const Request & request)
{
  Result result;
  result.execution_clock_kind = request.execution_clock.kind;
  result.first_published_control_origin_sec =
    request.execution_clock.first_published_control_origin_sec;
  result.first_published_artifact_elapsed_sec =
    request.execution_clock.first_published_artifact_elapsed_sec;
  if (request.plan == nullptr) {
    return result;
  }
  if (certified::validate(*request.plan) != certified::RejectReason::None) {
    result.reason = Reason::InvalidPlan;
    return result;
  }
  const auto & execution = *request.plan->execution_artifact;
  const auto & source = *request.plan->physical_snapshot;
  const bool published_clock_invalid =
    request.execution_clock.kind == ExecutionClockKind::PublishedPlan &&
    (!std::isfinite(
      request.execution_clock.first_published_control_origin_sec) ||
    request.execution_clock.first_published_control_origin_sec < 0.0 ||
    !std::isfinite(
      request.execution_clock.first_published_artifact_elapsed_sec) ||
    request.execution_clock.first_published_artifact_elapsed_sec < 0.0 ||
    !std::isfinite(request.control_origin_sec) ||
    request.control_origin_sec + kIdentityTolerance <
    request.execution_clock.first_published_control_origin_sec);
  if (
    request.execution_clock.kind == ExecutionClockKind::Unknown ||
    published_clock_invalid)
  {
    result.reason = Reason::ExecutionClockInvalid;
    return result;
  }
  const auto cursor = resolve_execution_cursor(
    execution, request.control_origin_sec, request.execution_clock);
  result.cursor_reason = cursor.reason;
  if (cursor.available) {
    result.cursor_elapsed_sec = cursor.elapsed_sec;
  }
  if (!cursor.available) {
    result.reason = Reason::CursorUnavailable;
    return result;
  }
  if (!artifact::supports_intent(request.current_intent) ||
    request.current_intent != execution.identity.source_context.intent)
  {
    result.reason = Reason::IntentMismatch;
    return result;
  }
  if (!request.obstacles.current) {
    result.reason = Reason::DynamicObservationUnavailable;
    return result;
  }
  if (!dynamic_proof::observation_valid(request.obstacles) ||
    request.obstacles.observed_sec > request.now_sec + kIdentityTolerance)
  {
    result.reason = Reason::DynamicObservationInvalid;
    return result;
  }
  const FollowTargetObservation * follow_target = nullptr;
  if (request.current_intent == contract::ControlIntent::Follow) {
    if (!request.follow_target.has_value() || !request.follow_target->current) {
      result.reason = Reason::FollowTargetObservationUnavailable;
      return result;
    }
    if (!follow_target_observation_valid(request.follow_target.value()) ||
      request.follow_target->observed_sec > request.now_sec + kIdentityTolerance)
    {
      result.reason = Reason::FollowTargetObservationInvalid;
      return result;
    }
    const bool target_present = std::any_of(
      request.obstacles.obstacles.begin(), request.obstacles.obstacles.end(),
      [&request](const DynamicObstacle & obstacle) {
        return obstacle.id == request.follow_target->target_id;
      });
    if (
      execution.identity.source_context.target_id !=
      request.follow_target->target_id ||
      execution.identity.source_context.target_obstacle_generation == 0U ||
      request.follow_target->observation_generation !=
      request.obstacles.generation || !target_present)
    {
      result.reason = Reason::FollowTargetIdentityMismatch;
      return result;
    }
    follow_target = &request.follow_target.value();
    result.follow_target_observation_generation =
      follow_target->observation_generation;
  }
  const bool static_world_matches =
    request.current_wall_grid != nullptr &&
    source.wall_grid_fingerprint != 0U &&
    (request.current_wall_grid.get() == source.wall_grid.get() ||
    recovery::occupancy_grid_fingerprint(*request.current_wall_grid) ==
    source.wall_grid_fingerprint);
  if (!static_world_matches ||
    !same_footprint(request.current_footprint, source.footprint))
  {
    result.reason = Reason::StaticWorldMismatch;
    return result;
  }
  if (request.decision_id == 0U || !std::isfinite(request.now_sec) ||
    !std::isfinite(request.current_speed_mps) ||
    request.current_speed_mps < 0.0 ||
    !std::isfinite(request.control_origin_speed_mps) ||
    request.control_origin_speed_mps < 0.0 ||
    !std::isfinite(request.current_time_steering_rad) ||
    !std::isfinite(request.current_steering_rad) ||
    !std::isfinite(request.current_response_steering_rad) ||
    !std::isfinite(request.previous_published_steering_rad) ||
    !std::isfinite(request.previous_published_command_age_sec) ||
    request.previous_published_command_age_sec < 0.0 ||
    !std::isfinite(request.minimum_acceleration_mps2) ||
    !std::isfinite(request.maximum_acceleration_mps2) ||
    request.minimum_acceleration_mps2 > request.maximum_acceleration_mps2 ||
    !std::isfinite(request.control_origin_sec) ||
    request.control_origin_sec < request.now_sec ||
    request.measured_to_control_path.empty() ||
    request.measured_to_control_path.size() !=
    request.measured_to_control_elapsed_sec.size() ||
    !std::isfinite(request.measured_to_control_elapsed_sec.front()) ||
    std::abs(request.measured_to_control_elapsed_sec.front()) >
    kIdentityTolerance ||
    !std::isfinite(request.measured_to_control_elapsed_sec.back()) ||
    std::abs(
      request.measured_to_control_elapsed_sec.back() -
      (request.control_origin_sec - request.now_sec)) > kIdentityTolerance ||
    !same_pose(request.measured_to_control_path.back(), request.control_pose))
  {
    result.reason = Reason::InvalidCurrentState;
    return result;
  }

  const auto affine_command_state = interpolate_expected_state(execution, cursor);
  const auto exact_physical_state = sample_exact_physical_state(
    execution, source, cursor.elapsed_sec);
  if (!exact_physical_state.has_value()) {
    result.reason = Reason::InvalidPlan;
    return result;
  }
  // The physical certificate owns the executed Frenet pose and velocity.
  // The affine QP state remains authoritative only for the serialized command
  // coordinates which are absent from the nonlinear replay artifact.
  const auto expected = as_predicted_state(
    exact_physical_state.value(), affine_command_state,
    execution.course_progress_origin_m);
  const double expected_absolute_progress_m =
    exact_physical_state->absolute_progress_m;
  const double expected_physical_progress_m =
    expected_absolute_progress_m + exact_physical_state->lag_m;
  result.expected_absolute_progress_m = expected_absolute_progress_m;
  result.expected_physical_progress_m = expected_physical_progress_m;
  result.progress_continuity_tolerance_m =
    request.progress_continuity_tolerance_m;
  result.current_speed_mps = request.current_speed_mps;
  result.control_origin_speed_mps = request.control_origin_speed_mps;
  result.current_time_steering_rad = request.current_time_steering_rad;
  result.current_steering_rad = request.current_steering_rad;
  result.current_response_steering_rad =
    request.current_response_steering_rad;
  result.previous_published_steering_rad =
    request.previous_published_steering_rad;
  const auto lift = lift_progress(
    request.control_origin_physical_progress_m,
    expected_physical_progress_m,
    request.path_length_m, request.progress_continuity_tolerance_m,
    request.circular);
  result.lifted_control_origin_physical_progress_m = lift.progress_m;
  result.progress_difference_m = lift.difference_m;
  if (!lift.accepted) {
    result.reason = Reason::ProgressLiftRejected;
    return result;
  }
  const auto expected_pose = reconstruct_pose(source, expected);
  if (!expected_pose.has_value()) {
    result.reason = Reason::CourseFrameUnavailable;
    return result;
  }
  result.expected_lateral_m = expected.lateral_m;
  result.expected_lag_m = expected.lag_m;
  result.expected_heading_offset_rad = expected.heading_offset_rad;
  result.control_pose = request.control_pose;
  result.expected_current_pose = expected_pose.value();
  result.control_pose_error_m = std::hypot(
    request.control_pose.x_m - expected_pose->x_m,
    request.control_pose.y_m - expected_pose->y_m);
  result.control_yaw_error_rad = wrap_to_pi(
    request.control_pose.yaw_rad - expected_pose->yaw_rad);
  const auto current_course_frame = mpc_stage_geometry::sample_course_frame(
    source.course_frame_knots, expected_absolute_progress_m,
    std::max(kIdentityTolerance, source.bound_tolerance_m));
  if (!current_course_frame.has_value()) {
    result.reason = Reason::CourseFrameUnavailable;
    return result;
  }
  const auto current_frenet = contract::project_planar_pose_to_frenet(
    contract::PlanarPose{
      request.control_pose.x_m, request.control_pose.y_m,
      request.control_pose.yaw_rad},
    contract::PlanarPose{
      current_course_frame->x_m, current_course_frame->y_m,
      current_course_frame->heading_rad});
  if (!current_frenet.has_value()) {
    result.reason = Reason::InvalidCurrentState;
    return result;
  }
  const artifact::PredictedState current_control_state{
    current_frenet->lateral_m,
    current_frenet->lag_m,
    current_frenet->heading_offset_rad,
    request.control_origin_speed_mps,
    expected.progress_m,
    request.current_steering_rad,
    request.current_response_steering_rad};
  result.current_control_state = current_control_state;
  result.current_control_state_available = true;
  const double prediction_delay_sec =
    request.control_origin_sec - request.now_sec;
  if (
    prediction_delay_sec <= kIdentityTolerance &&
    std::abs(request.control_origin_speed_mps - request.current_speed_mps) >
    kIdentityTolerance)
  {
    // With no observation-to-control interval both values describe the same
    // physical state.  Reject contradictory input provenance before any
    // retained trajectory is joined.
    result.reason = Reason::InvalidCurrentState;
    return result;
  }
  auto evaluate_follow_gap = [&] (
      const artifact::PredictedState & state,
      const double elapsed_time_sec) -> std::optional<double>
    {
      if (follow_target == nullptr) {
        return std::numeric_limits<double>::infinity();
      }
      const auto target_progress = sample_follow_target_progress(
        *follow_target, elapsed_time_sec);
      if (!target_progress.has_value()) {
        return std::nullopt;
      }
      const double target_absolute_progress_m =
        lift.progress_m + target_progress.value();
      const double ego_absolute_progress_m =
        execution.course_progress_origin_m + state.progress_m + state.lag_m;
      const double gap_m =
        target_absolute_progress_m - ego_absolute_progress_m;
      return std::isfinite(gap_m) ?
        std::optional<double>{gap_m} : std::nullopt;
    };
  if (follow_target != nullptr) {
    result.follow_minimum_gap_m = follow_target->current_target_gap_m;
    if (
      follow_target->current_target_gap_m + kIdentityTolerance <
      follow_target->hard_gap_m)
    {
      result.reason = Reason::FollowInitialHardGapViolation;
      return result;
    }
    const auto expected_gap = evaluate_follow_gap(
      current_control_state, prediction_delay_sec);
    if (!expected_gap.has_value()) {
      result.reason = Reason::FollowTargetHorizonUnavailable;
      return result;
    }
    ++result.follow_checked_state_count;
    result.follow_minimum_gap_m = std::min(
      result.follow_minimum_gap_m, expected_gap.value());
    if (expected_gap.value() + kIdentityTolerance < follow_target->hard_gap_m) {
      result.reason = Reason::FollowInitialHardGapViolation;
      return result;
    }
  }
  const auto actuation = artifact::extract_actuation(execution, cursor);
  result.actuation_reason = actuation.reason;
  if (!actuation.actuation.has_value()) {
    result.reason = Reason::ActuationRejected;
    return result;
  }
  result.expected_speed_mps = actuation.actuation->predicted_speed_mps;
  result.expected_steering_rad = actuation.actuation->steering_rad;

  // The QP owns one serialized command trajectory.  Revalidate its executable
  // sample against the last command over the actual publication age.  The
  // measured physical angle belongs to response prediction; making it a
  // second command origin re-bases the rate integral every cycle and produces
  // a command trajectory different from the one certified by the QP.
  const double steering_reachability_duration_sec =
    request.previous_published_command_age_sec;
  const double maximum_steering_step_rad =
    execution.maximum_abs_steering_rate_radps *
    steering_reachability_duration_sec + execution.physical_global_tolerance;
  const double steering_difference_rad =
    actuation.actuation->steering_rad -
    request.previous_published_steering_rad;
  const double reachable_steering_lower_rad = std::max(
    -execution.maximum_abs_steering_rad,
    request.previous_published_steering_rad - maximum_steering_step_rad);
  const double reachable_steering_upper_rad = std::min(
    execution.maximum_abs_steering_rad,
    request.previous_published_steering_rad + maximum_steering_step_rad);
  result.steering_difference_rad = steering_difference_rad;
  result.maximum_steering_step_rad = maximum_steering_step_rad;
  result.reachable_steering_lower_rad = reachable_steering_lower_rad;
  result.reachable_steering_upper_rad = reachable_steering_upper_rad;
  result.steering_reachability_duration_sec =
    steering_reachability_duration_sec;
  bool feedback_shadow_mode = false;
  double continuation_initial_steering_rad =
    actuation.actuation->steering_rad;
  if (
    actuation.actuation->steering_rad < reachable_steering_lower_rad ||
    actuation.actuation->steering_rad > reachable_steering_upper_rad)
  {
    result.feedback_shadow_attempted = true;
    const auto feedback = mpcc_latest_state_feedback::solve(
      mpcc_latest_state_feedback::Request{
        request.previous_published_steering_rad,
        actuation.actuation->steering_rad,
        execution.maximum_abs_steering_rad,
        execution.maximum_abs_steering_rate_radps,
        steering_reachability_duration_sec,
        execution.physical_global_tolerance});
    result.feedback_shadow_reason = feedback.reason;
    result.feedback_shadow_steering_rad = feedback.feedback_steering_rad;
    result.feedback_shadow_correction_rad = feedback.correction_rad;
    if (!feedback.available()) {
      result.reason = Reason::SteeringUnreachable;
      return result;
    }
    feedback_shadow_mode = true;
    continuation_initial_steering_rad = feedback.feedback_steering_rad;
  }
  const auto complete_continuation_proof = [&] (const Reason reason) {
      if (feedback_shadow_mode) {
        result.feedback_shadow_proof_reason = reason;
        result.feedback_shadow_proof_available = reason == Reason::Accepted;
        result.reason = Reason::SteeringUnreachable;
      } else {
        result.reason = reason;
      }
      return result;
    };
  const double velocity_reachability_duration_sec =
    request.control_origin_sec - request.now_sec;
  const double velocity_lower_mps = std::max(
    0.0, request.current_speed_mps +
    request.minimum_acceleration_mps2 * velocity_reachability_duration_sec -
    execution.physical_global_tolerance);
  const double velocity_upper_mps = std::max(
    0.0, request.current_speed_mps +
    request.maximum_acceleration_mps2 * velocity_reachability_duration_sec +
    execution.physical_global_tolerance);
  result.velocity_difference_mps =
    actuation.actuation->predicted_speed_mps - request.current_speed_mps;
  result.reachable_velocity_lower_mps = velocity_lower_mps;
  result.reachable_velocity_upper_mps = velocity_upper_mps;
  result.velocity_reachability_duration_sec =
    velocity_reachability_duration_sec;

  // The retained artifact's predicted velocity is an old model state, not a
  // serialized actuator coordinate.  Requiring that state to remain exactly
  // reachable from the new observation creates a second velocity origin and
  // rejects the same fresh state used by the continuation below.  Preserve
  // the discrepancy and historical reachability envelope as diagnostics,
  // then build one current-world command from the fresh control-origin speed
  // and the artifact's still-certified control inputs.
  auto current_world_actuation = actuation.actuation.value();
  current_world_actuation.predicted_speed_mps =
    request.control_origin_speed_mps;
  if (feedback_shadow_mode) {
    current_world_actuation.steering_rad =
      continuation_initial_steering_rad;
  }

  const auto continuation =
    mpcc_rate_resolved_physical_adapter::build_continuation(
    execution, cursor,
    mpcc_rate_resolved_physical_adapter::ContinuationInitialState{
      current_control_state.lateral_m,
      current_control_state.lag_m,
      current_control_state.heading_offset_rad,
      current_control_state.velocity_mps,
      current_control_state.progress_m,
      continuation_initial_steering_rad,
      request.current_response_steering_rad});
  result.continuation_reason = continuation.reason;
  result.continuation_scope = continuation.scope;
  result.continuation_exact_reason = continuation.exact_reason;
  if (feedback_shadow_mode) {
    result.feedback_shadow_continuation_reason = continuation.reason;
    result.feedback_shadow_exact_reason = continuation.exact_reason;
    result.feedback_shadow_continuation_available =
      continuation.exact_trajectory.has_value();
  }
  if (!continuation.exact_trajectory.has_value()) {
    return complete_continuation_proof(Reason::ContinuationRejected);
  }
  const auto & continuation_trajectory =
    continuation.exact_trajectory.value();
  result.proved_control_stage_count = continuation.scope ==
      mpcc_rate_resolved_physical_adapter::ContinuationProofScope::
      CurrentStagePrefix ?
    1U : cursor.remaining_control_stage_count;
  if (
    continuation.scope ==
    mpcc_rate_resolved_physical_adapter::ContinuationProofScope::
    CurrentStagePrefix)
  {
    result.static_wall_scope = StaticWallProofScope::CurrentStagePrefix;
  }

  const auto clearance_footprint = physical::resolve_clearance_footprint(
    source.footprint, source.hard_wall_clearance_m);
  if (!clearance_footprint.has_value()) {
    return complete_continuation_proof(Reason::StaticWorldMismatch);
  }
  const auto delay = recovery::evaluate_clear_footprint_path(
    *source.wall_grid, clearance_footprint.value(),
    request.measured_to_control_path, source.swept_step_m);
  result.delay_path_clearance = delay;
  if (!delay.valid) {
    return complete_continuation_proof(Reason::ControlPathInvalid);
  }
  if (!delay.clear) {
    return complete_continuation_proof(Reason::DelayPrefixBlocked);
  }

  std::vector<recovery::Pose2D> continuation_path;
  continuation_path.reserve(continuation_trajectory.progress_m.size() + 1U);
  continuation_path.push_back(request.control_pose);
  const double current_stage_remaining_sec =
    execution.control_stages[cursor.control_stage_index].duration_sec -
    cursor.stage_elapsed_sec;
  std::size_t current_stage_last_path_index{};

  dynamic_proof::Result dynamic;
  dynamic_proof::observe_timed_path(
    source.footprint, request.measured_to_control_path,
    request.measured_to_control_elapsed_sec, source.swept_step_m,
    request.obstacles, dynamic);
  // Keep an independent proof for exactly the control stage which can be
  // published now.  The full suffix still owns diagnostics and future
  // replanning, but a collision in a later stage must not erase a clear
  // current-stage authority interval.
  dynamic_proof::Result current_stage_dynamic = dynamic;
  auto previous_dynamic_pose = request.control_pose;
  double dynamic_time_sec = prediction_delay_sec;
  for (
    std::size_t sample_index = 0U;
    sample_index < continuation_trajectory.elapsed_time_sec.size();
    ++sample_index)
  {
    const double sample_elapsed_sec =
      continuation_trajectory.elapsed_time_sec[sample_index];
    const auto physical_endpoint = exact_physical_state_at(
      continuation_trajectory, sample_index);
    const auto endpoint_state = as_predicted_state(
      physical_endpoint, current_control_state,
      execution.course_progress_origin_m);
    const auto endpoint_pose = reconstruct_pose(source, endpoint_state);
    if (!endpoint_pose.has_value()) {
      return complete_continuation_proof(Reason::CourseFrameUnavailable);
    }
    continuation_path.push_back(endpoint_pose.value());
    if (
      sample_elapsed_sec <= current_stage_remaining_sec +
      execution.physical_global_tolerance)
    {
      current_stage_last_path_index = continuation_path.size() - 1U;
    }
    const double endpoint_time_sec = prediction_delay_sec +
      sample_elapsed_sec;
    if (follow_target != nullptr) {
      const auto stage_gap = evaluate_follow_gap(
        endpoint_state, endpoint_time_sec);
      if (!stage_gap.has_value()) {
        return complete_continuation_proof(
          Reason::FollowTargetHorizonUnavailable);
      }
      ++result.follow_checked_state_count;
      result.follow_minimum_gap_m = std::min(
        result.follow_minimum_gap_m, stage_gap.value());
      if (stage_gap.value() + kIdentityTolerance < follow_target->hard_gap_m) {
        return complete_continuation_proof(Reason::FollowStageGapViolation);
      }
    }
    if (
      sample_elapsed_sec <= current_stage_remaining_sec +
      execution.physical_global_tolerance)
    {
      dynamic_proof::observe_segment(
        source.footprint, previous_dynamic_pose, endpoint_pose.value(),
        dynamic_time_sec, endpoint_time_sec,
        source.swept_step_m, request.obstacles, current_stage_dynamic);
    }
    dynamic_proof::observe_segment(
      source.footprint, previous_dynamic_pose, endpoint_pose.value(),
      dynamic_time_sec, endpoint_time_sec,
      source.swept_step_m, request.obstacles, dynamic);
    dynamic_time_sec = endpoint_time_sec;
    previous_dynamic_pose = endpoint_pose.value();
    if (!dynamic.valid || !dynamic.clear) {
      break;
    }
  }
  dynamic_proof::finalize(request.obstacles, current_stage_dynamic);
  dynamic_proof::finalize(request.obstacles, dynamic);
  const auto continuation_clearance =
    recovery::evaluate_clear_footprint_path(
    *source.wall_grid, clearance_footprint.value(), continuation_path,
    source.swept_step_m);
  result.continuation_path_clearance = continuation_clearance;
  if (!continuation_clearance.valid) {
    return complete_continuation_proof(Reason::ControlPathInvalid);
  }
  if (!continuation_clearance.clear) {
    if (
      current_stage_last_path_index < 1U ||
      current_stage_last_path_index >= continuation_path.size())
    {
      return complete_continuation_proof(Reason::ContinuationRejected);
    }
    const std::vector<recovery::Pose2D> current_stage_path{
      continuation_path.begin(),
      continuation_path.begin() + current_stage_last_path_index + 1U};
    const auto current_stage_clearance =
      recovery::evaluate_clear_footprint_path(
      *source.wall_grid, clearance_footprint.value(), current_stage_path,
      source.swept_step_m);
    result.current_stage_path_clearance = current_stage_clearance;
    if (!current_stage_clearance.valid) {
      return complete_continuation_proof(Reason::ControlPathInvalid);
    }
    if (!current_stage_clearance.clear) {
      return complete_continuation_proof(Reason::ContinuationWallBlocked);
    }
    result.static_wall_scope = StaticWallProofScope::CurrentStagePrefix;
    result.proved_control_stage_count = 1U;
  }
  result.blocking_obstacle_id = dynamic.blocking_obstacle_id;
  result.dynamic_checked_pose_count = dynamic.checked_pose_count;
  result.minimum_dynamic_clearance_m = dynamic.minimum_clearance_m;
  if (!dynamic.valid) {
    return complete_continuation_proof(Reason::DynamicPathInvalid);
  }
  if (!dynamic.clear) {
    if (!current_stage_dynamic.valid) {
      return complete_continuation_proof(Reason::DynamicPathInvalid);
    }
    if (!current_stage_dynamic.clear) {
      return complete_continuation_proof(Reason::DynamicPathBlocked);
    }
    result.dynamic_obstacle_scope =
      DynamicObstacleProofScope::CurrentStagePrefix;
    result.proved_control_stage_count = 1U;
  }

  const bool partial_normal_proof =
    continuation.scope ==
    mpcc_rate_resolved_physical_adapter::ContinuationProofScope::
    CurrentStagePrefix ||
    result.static_wall_scope == StaticWallProofScope::CurrentStagePrefix ||
    result.dynamic_obstacle_scope ==
    DynamicObstacleProofScope::CurrentStagePrefix;

  race_mpcc_foundation::ExactPhysicalExecutionTrajectory
  terminal_stop_trajectory;
  recovery::PathClearanceResult terminal_stop_clearance;
  dynamic_proof::Result terminal_stop_dynamic;
  if (partial_normal_proof) {
    // Publication is causal: the current serialized command can remain on the
    // actuator for one publisher interval even if the next solve fails. A
    // partial normal prefix therefore receives authority only when that exact
    // interval followed by a max-braking/path-tracking terminal
    // sequence is rebuilt from the current state and proved against this same
    // immutable world observation. The first interval must replay both the
    // acceleration and steering-rate components of the serialized command.
    result.terminal_stop_attempted = true;
    const auto terminal_stop =
      mpcc_rate_resolved_physical_adapter::build_stop_contingency(
      execution, cursor, current_world_actuation,
      mpcc_rate_resolved_physical_adapter::ContinuationInitialState{
        current_control_state.lateral_m,
        current_control_state.lag_m,
        current_control_state.heading_offset_rad,
        current_control_state.velocity_mps,
        current_control_state.progress_m,
        continuation_initial_steering_rad,
        request.current_response_steering_rad},
      request.stop_lateral_policy,
      request.minimum_acceleration_mps2);
    result.terminal_stop_reason = terminal_stop.reason;
    result.terminal_stop_exact_reason = terminal_stop.exact_reason;
    result.terminal_stop_rejected_sample = terminal_stop.rejected_sample;
    result.terminal_stop_publisher_interval_end_steering_rad =
      terminal_stop.publisher_interval_end_steering_rad;
    result.terminal_stop_final_steering_rad =
      terminal_stop.braking_suffix_final_steering_rad;
    if (!terminal_stop.exact_trajectory.has_value()) {
      return complete_continuation_proof(
        Reason::TerminalContingencyUnavailable);
    }
    terminal_stop_trajectory = terminal_stop.exact_trajectory.value();
    std::vector<recovery::Pose2D> terminal_stop_path;
    terminal_stop_path.reserve(
      terminal_stop_trajectory.progress_m.size() + 1U);
    terminal_stop_path.push_back(request.control_pose);
    auto previous_stop_pose = request.control_pose;
    double previous_stop_time_sec = prediction_delay_sec;
    dynamic_proof::observe_timed_path(
      source.footprint, request.measured_to_control_path,
      request.measured_to_control_elapsed_sec, source.swept_step_m,
      request.obstacles, terminal_stop_dynamic);
    for (std::size_t sample_index = 0U;
      sample_index < terminal_stop_trajectory.elapsed_time_sec.size();
      ++sample_index)
    {
      const auto physical_endpoint = exact_physical_state_at(
        terminal_stop_trajectory, sample_index);
      const auto endpoint_state = as_predicted_state(
        physical_endpoint, current_control_state,
        execution.course_progress_origin_m);
      const auto endpoint_pose = reconstruct_pose(source, endpoint_state);
      if (!endpoint_pose.has_value()) {
        return complete_continuation_proof(
          Reason::TerminalContingencyUnavailable);
      }
      const double endpoint_time_sec = prediction_delay_sec +
        terminal_stop_trajectory.elapsed_time_sec[sample_index];
      if (follow_target != nullptr) {
        const auto stage_gap = evaluate_follow_gap(
          endpoint_state, endpoint_time_sec);
        if (!stage_gap.has_value()) {
          return complete_continuation_proof(
            Reason::TerminalContingencyUnavailable);
        }
        ++result.terminal_stop_follow_checked_state_count;
        result.terminal_stop_follow_minimum_gap_m = std::min(
          result.terminal_stop_follow_minimum_gap_m, stage_gap.value());
        if (stage_gap.value() + kIdentityTolerance < follow_target->hard_gap_m) {
          return complete_continuation_proof(
            Reason::TerminalContingencyUnavailable);
        }
      }
      dynamic_proof::observe_segment(
        source.footprint, previous_stop_pose, endpoint_pose.value(),
        previous_stop_time_sec, endpoint_time_sec, source.swept_step_m,
        request.obstacles, terminal_stop_dynamic);
      terminal_stop_path.push_back(endpoint_pose.value());
      previous_stop_pose = endpoint_pose.value();
      previous_stop_time_sec = endpoint_time_sec;
      if (!terminal_stop_dynamic.valid || !terminal_stop_dynamic.clear) {
        break;
      }
    }
    dynamic_proof::finalize(request.obstacles, terminal_stop_dynamic);
    result.terminal_stop_blocking_obstacle_id =
      terminal_stop_dynamic.blocking_obstacle_id;
    result.terminal_stop_dynamic_checked_pose_count =
      terminal_stop_dynamic.checked_pose_count;
    result.terminal_stop_minimum_dynamic_clearance_m =
      terminal_stop_dynamic.minimum_clearance_m;
    if (!terminal_stop_dynamic.valid || !terminal_stop_dynamic.clear) {
      return complete_continuation_proof(
        Reason::TerminalContingencyUnavailable);
    }
    terminal_stop_clearance = recovery::evaluate_clear_footprint_path(
      *source.wall_grid, clearance_footprint.value(), terminal_stop_path,
      source.swept_step_m);
    result.terminal_stop_path_clearance = terminal_stop_clearance;
    if (!terminal_stop_clearance.valid || !terminal_stop_clearance.clear) {
      return complete_continuation_proof(
        Reason::TerminalContingencyUnavailable);
    }
    result.terminal_stop_certified = true;
  }

  auto proved_continuation_trajectory = continuation_trajectory;
  auto proved_stage_end_velocity_mps = continuation.stage_end_velocity_mps;
  auto proved_stage_end_steering_rad = continuation.stage_end_steering_rad;
  if (result.proved_control_stage_count == 1U) {
    const std::size_t proved_sample_count = current_stage_last_path_index;
    const auto retain_proved_samples =
      [proved_sample_count](auto & values) {
        if (values.size() > proved_sample_count) {
          values.resize(proved_sample_count);
        }
      };
    retain_proved_samples(proved_continuation_trajectory.elapsed_time_sec);
    retain_proved_samples(proved_continuation_trajectory.path_distance_m);
    retain_proved_samples(proved_continuation_trajectory.lateral_m);
    retain_proved_samples(proved_continuation_trajectory.lag_m);
    retain_proved_samples(proved_continuation_trajectory.heading_offset_rad);
    retain_proved_samples(proved_continuation_trajectory.velocity_mps);
    retain_proved_samples(proved_continuation_trajectory.progress_m);
    retain_proved_samples(proved_continuation_trajectory.lateral_lower_m);
    retain_proved_samples(proved_continuation_trajectory.lateral_upper_m);
    proved_continuation_trajectory.minimum_lateral_bound_reserve_m =
      std::numeric_limits<double>::infinity();
    for (std::size_t index = 0U; index < proved_sample_count; ++index) {
      proved_continuation_trajectory.minimum_lateral_bound_reserve_m =
        std::min(
          proved_continuation_trajectory.minimum_lateral_bound_reserve_m,
          std::min(
            proved_continuation_trajectory.lateral_m[index] -
            proved_continuation_trajectory.lateral_lower_m[index],
            proved_continuation_trajectory.lateral_upper_m[index] -
            proved_continuation_trajectory.lateral_m[index]));
    }
    proved_stage_end_velocity_mps.resize(1U);
    proved_stage_end_steering_rad.resize(1U);
  }

  if (feedback_shadow_mode) {
    return complete_continuation_proof(Reason::Accepted);
  }

  Proof proof;
  proof.plan = request.plan;
  proof.decision_id = request.decision_id;
  proof.obstacle_generation = request.obstacles.generation;
  proof.observed_sec = request.obstacles.observed_sec;
  proof.observation_origin_sec = request.now_sec;
  proof.control_origin_sec = request.control_origin_sec;
  proof.prediction_delay_sec = prediction_delay_sec;
  proof.cursor = cursor;
  proof.actuation = current_world_actuation;
  proof.expected_current_state = expected;
  proof.expected_current_pose = expected_pose.value();
  proof.expected_absolute_progress_m = expected_absolute_progress_m;
  proof.expected_physical_progress_m = expected_physical_progress_m;
  proof.lifted_control_origin_physical_progress_m = lift.progress_m;
  proof.lap_offset = lift.lap_offset;
  proof.current_time_steering_rad = request.current_time_steering_rad;
  proof.previous_published_steering_rad =
    request.previous_published_steering_rad;
  proof.steering_difference_rad = steering_difference_rad;
  proof.maximum_steering_step_rad = maximum_steering_step_rad;
  proof.reachable_steering_lower_rad = reachable_steering_lower_rad;
  proof.reachable_steering_upper_rad = reachable_steering_upper_rad;
  proof.steering_reachability_duration_sec =
    steering_reachability_duration_sec;
  proof.velocity_difference_mps = result.velocity_difference_mps;
  proof.reachable_velocity_lower_mps = result.reachable_velocity_lower_mps;
  proof.reachable_velocity_upper_mps = result.reachable_velocity_upper_mps;
  proof.velocity_reachability_duration_sec =
    result.velocity_reachability_duration_sec;
  proof.delay_checked_pose_count = delay.checked_pose_count;
  proof.connector_checked_pose_count = 0U;
  proof.static_wall_scope = result.static_wall_scope;
  proof.dynamic_obstacle_scope = result.dynamic_obstacle_scope;
  proof.continuation_scope = result.continuation_scope;
  proof.proved_control_stage_count = result.proved_control_stage_count;
  proof.static_wall_checked_pose_count =
    result.static_wall_scope == StaticWallProofScope::FullSuffix ?
    continuation_clearance.checked_pose_count :
    result.current_stage_path_clearance.checked_pose_count;
  const auto & proved_dynamic =
    result.dynamic_obstacle_scope ==
    DynamicObstacleProofScope::CurrentStagePrefix ?
    current_stage_dynamic : dynamic;
  proof.dynamic_checked_pose_count = proved_dynamic.checked_pose_count;
  proof.minimum_dynamic_clearance_m = proved_dynamic.minimum_clearance_m;
  proof.follow_target_observation_generation =
    result.follow_target_observation_generation;
  proof.follow_checked_state_count = result.follow_checked_state_count;
  proof.follow_minimum_gap_m = result.follow_minimum_gap_m;
  proof.terminal_stop_certified = result.terminal_stop_certified;
  proof.terminal_stop_static_checked_pose_count =
    terminal_stop_clearance.checked_pose_count;
  proof.terminal_stop_dynamic_checked_pose_count =
    terminal_stop_dynamic.checked_pose_count;
  proof.terminal_stop_minimum_dynamic_clearance_m =
    terminal_stop_dynamic.minimum_clearance_m;
  proof.continuation_trajectory =
    std::move(proved_continuation_trajectory);
  proof.terminal_stop_trajectory = std::move(terminal_stop_trajectory);
  proof.continuation_stage_end_velocity_mps =
    std::move(proved_stage_end_velocity_mps);
  proof.continuation_stage_end_steering_rad =
    std::move(proved_stage_end_steering_rad);
  result.reason = Reason::Accepted;
  result.proof = std::move(proof);
  return result;
}

}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_retained_revalidation
