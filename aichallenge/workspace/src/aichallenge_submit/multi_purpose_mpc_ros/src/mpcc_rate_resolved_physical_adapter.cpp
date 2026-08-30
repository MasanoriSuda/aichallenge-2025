#include "multi_purpose_mpc_ros/mpcc_rate_resolved_physical_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_physical_adapter
{
namespace
{

struct NonlinearState
{
  double lateral_m{};
  double lag_m{};
  double heading_offset_rad{};
  double velocity_mps{};
  double progress_m{};
  double steering_rad{};
  double response_steering_rad{};
};

bool finite(const NonlinearState & state) noexcept
{
  return std::isfinite(state.lateral_m) && std::isfinite(state.lag_m) &&
         std::isfinite(state.heading_offset_rad) &&
         std::isfinite(state.velocity_mps) &&
         std::isfinite(state.progress_m) &&
         std::isfinite(state.steering_rad) &&
         std::isfinite(state.response_steering_rad);
}

bool advance_nonlinear_state(
  NonlinearState & state,
  const mpcc_rate_resolved_execution_artifact::ControlStage & control,
  const mpcc_rate_resolved_execution_artifact::ExecutionArtifact & artifact,
  const double step_sec) noexcept
{
  mpcc_rate_resolved::LinearizationRequest request;
  request.reference_lateral_m = state.lateral_m;
  request.reference_lag_m = state.lag_m;
  request.reference_heading_rad = state.heading_offset_rad;
  request.reference_velocity_mps = state.velocity_mps;
  request.reference_progress_m = state.progress_m;
  request.reference_steering_rad = state.steering_rad;
  request.reference_response_steering_rad = state.response_steering_rad;
  request.reference_acceleration_mps2 = control.acceleration_mps2;
  request.reference_steering_rate_radps = control.steering_rate_radps;
  request.reference_virtual_progress_speed_mps =
    control.virtual_progress_speed_mps;
  request.reference_path_curvature_radpm = control.path_curvature_radpm;
  request.wheelbase_m = artifact.wheelbase_m;
  request.yaw_response_gain = artifact.yaw_response_gain;
  request.yaw_response_time_constant_sec =
    artifact.yaw_response_time_constant_sec;
  request.stage_dt_sec = step_sec;
  request.minimum_frenet_denominator = artifact.minimum_frenet_denominator;
  request.minimum_stage_dt_sec = step_sec;
  request.maximum_stage_dt_sec = step_sec;
  const auto transition =
    mpcc_rate_resolved::evaluate_temporal_frenet_transition(request);
  if (!transition.has_value()) {
    return false;
  }
  const auto & next = transition->next_state;
  state.lateral_m = next[mpcc_rate_resolved::kLateralIndex];
  state.lag_m = next[mpcc_rate_resolved::kLagIndex];
  state.heading_offset_rad = next[mpcc_rate_resolved::kHeadingIndex];
  state.velocity_mps = next[mpcc_rate_resolved::kVelocityIndex];
  state.progress_m = next[mpcc_rate_resolved::kProgressIndex];
  state.steering_rad = next[mpcc_rate_resolved::kSteeringIndex];
  state.response_steering_rad =
    next[mpcc_rate_resolved::kResponseSteeringIndex];
  return finite(state);
}

struct SampledCourseGeometry
{
  double curvature_radpm{};
  double lateral_lower_m{};
  double lateral_upper_m{};
};

std::optional<SampledCourseGeometry> sample_course_geometry(
  const StopCourseGeometry & course_geometry,
  const double query_tolerance_m,
  const double local_progress_m) noexcept
{
  if (
    !stop_course_geometry_valid(course_geometry) ||
    !std::isfinite(query_tolerance_m) || query_tolerance_m < 0.0 ||
    !std::isfinite(local_progress_m) ||
    local_progress_m < course_geometry.progress_m.front() - query_tolerance_m ||
    local_progress_m > course_geometry.progress_m.back() + query_tolerance_m)
  {
    return std::nullopt;
  }
  std::size_t stage{};
  while (
    stage + 1U < course_geometry.curvature_radpm.size() &&
    local_progress_m > course_geometry.progress_m[stage + 1U] +
    query_tolerance_m)
  {
    ++stage;
  }
  const double start_progress_m =
    course_geometry.progress_m[stage];
  const double end_progress_m =
    course_geometry.progress_m[stage + 1U];
  const double fraction = std::clamp(
    (local_progress_m - start_progress_m) /
    (end_progress_m - start_progress_m), 0.0, 1.0);
  const auto interpolate = [fraction](const double start, const double end) {
      return start + fraction * (end - start);
    };
  SampledCourseGeometry geometry;
  geometry.curvature_radpm =
    course_geometry.curvature_radpm[stage];
  geometry.lateral_lower_m = interpolate(
    course_geometry.lateral_lower_m[stage],
    course_geometry.lateral_lower_m[stage + 1U]);
  geometry.lateral_upper_m = interpolate(
    course_geometry.lateral_upper_m[stage],
    course_geometry.lateral_upper_m[stage + 1U]);
  if (
    !std::isfinite(geometry.curvature_radpm) ||
    !std::isfinite(geometry.lateral_lower_m) ||
    !std::isfinite(geometry.lateral_upper_m) ||
    geometry.lateral_lower_m > geometry.lateral_upper_m)
  {
    return std::nullopt;
  }
  return geometry;
}

std::optional<double> physical_progress_speed(
  const NonlinearState & state, const double curvature_radpm,
  const double minimum_frenet_denominator) noexcept
{
  const double denominator = 1.0 - curvature_radpm * state.lateral_m;
  if (
    !std::isfinite(denominator) ||
    denominator < minimum_frenet_denominator)
  {
    return std::nullopt;
  }
  const double speed_mps =
    state.velocity_mps * std::cos(state.heading_offset_rad) / denominator;
  if (!std::isfinite(speed_mps) || speed_mps < 0.0) {
    return std::nullopt;
  }
  return speed_mps;
}

}  // namespace

bool stop_lateral_target_profile_valid(
  const StopLateralTargetProfile & profile) noexcept
{
  if (
    profile.progress_m.size() < 2U ||
    profile.lateral_m.size() != profile.progress_m.size())
  {
    return false;
  }
  for (std::size_t index = 0U; index < profile.progress_m.size(); ++index) {
    if (
      !std::isfinite(profile.progress_m[index]) ||
      !std::isfinite(profile.lateral_m[index]) ||
      (index > 0U &&
      profile.progress_m[index] <= profile.progress_m[index - 1U]))
    {
      return false;
    }
  }
  return true;
}

std::optional<double> sample_stop_lateral_target(
  const StopLateralTargetProfile & profile,
  const double progress_m,
  const double tolerance_m) noexcept
{
  if (
    !stop_lateral_target_profile_valid(profile) ||
    !std::isfinite(progress_m) || !std::isfinite(tolerance_m) ||
    tolerance_m < 0.0 ||
    progress_m < profile.progress_m.front() - tolerance_m ||
    progress_m > profile.progress_m.back() + tolerance_m)
  {
    return std::nullopt;
  }
  if (progress_m <= profile.progress_m.front()) {
    return profile.lateral_m.front();
  }
  if (progress_m >= profile.progress_m.back()) {
    return profile.lateral_m.back();
  }
  const auto upper = std::upper_bound(
    profile.progress_m.begin(), profile.progress_m.end(), progress_m);
  if (upper == profile.progress_m.begin() || upper == profile.progress_m.end()) {
    return std::nullopt;
  }
  const std::size_t upper_index = static_cast<std::size_t>(
    std::distance(profile.progress_m.begin(), upper));
  const std::size_t lower_index = upper_index - 1U;
  const double progress_span_m =
    profile.progress_m[upper_index] - profile.progress_m[lower_index];
  const double fraction =
    (progress_m - profile.progress_m[lower_index]) / progress_span_m;
  const double sampled_lateral_m = profile.lateral_m[lower_index] +
    fraction *
    (profile.lateral_m[upper_index] - profile.lateral_m[lower_index]);
  return std::isfinite(sampled_lateral_m) ?
    std::optional<double>{sampled_lateral_m} : std::nullopt;
}

const char * to_string(const RejectReason reason) noexcept
{
  switch (reason) {
    case RejectReason::None: return "none";
    case RejectReason::InvalidArtifact: return "invalid-artifact";
    case RejectReason::IntentMismatch: return "intent-mismatch";
    case RejectReason::StageGeometryMismatch: return "stage-geometry-mismatch";
    case RejectReason::ExactTrajectoryRejected:
      return "exact-trajectory-rejected";
    case RejectReason::Count: break;
  }
  return "unknown";
}

const char * to_string(const ContinuationRejectReason reason) noexcept
{
  switch (reason) {
    case ContinuationRejectReason::None: return "none";
    case ContinuationRejectReason::InvalidArtifact:
      return "invalid-artifact";
    case ContinuationRejectReason::InvalidCursor: return "invalid-cursor";
    case ContinuationRejectReason::InvalidInitialState:
      return "invalid-initial-state";
    case ContinuationRejectReason::InitialLateralBoundRejected:
      return "initial-lateral-bound-rejected";
    case ContinuationRejectReason::NonlinearModelRejected:
      return "nonlinear-model-rejected";
    case ContinuationRejectReason::ActuatorEnvelopeRejected:
      return "actuator-envelope-rejected";
    case ContinuationRejectReason::ExactTrajectoryRejected:
      return "exact-trajectory-rejected";
    case ContinuationRejectReason::Count: break;
  }
  return "unknown";
}

const char * to_string(const ContinuationProofScope scope) noexcept
{
  switch (scope) {
    case ContinuationProofScope::FullSuffix: return "full-suffix";
    case ContinuationProofScope::PublisherIntervalPrefix:
      return "publisher-interval-prefix";
  }
  return "unknown";
}

const char * to_string(const StopContingencyRejectReason reason) noexcept
{
  switch (reason) {
    case StopContingencyRejectReason::None: return "none";
    case StopContingencyRejectReason::InvalidArtifact:
      return "invalid-artifact";
    case StopContingencyRejectReason::InvalidCursor:
      return "invalid-cursor";
    case StopContingencyRejectReason::InvalidInitialState:
      return "invalid-initial-state";
    case StopContingencyRejectReason::InvalidActuation:
      return "invalid-actuation";
    case StopContingencyRejectReason::InvalidBrakingEnvelope:
      return "invalid-braking-envelope";
    case StopContingencyRejectReason::InvalidLateralPolicy:
      return "invalid-lateral-policy";
    case StopContingencyRejectReason::CourseGeometryUnavailable:
      return "course-geometry-unavailable";
    case StopContingencyRejectReason::NonlinearModelRejected:
      return "nonlinear-model-rejected";
    case StopContingencyRejectReason::ActuatorEnvelopeRejected:
      return "actuator-envelope-rejected";
    case StopContingencyRejectReason::ExactTrajectoryRejected:
      return "exact-trajectory-rejected";
    case StopContingencyRejectReason::Count: break;
  }
  return "unknown";
}

Result build(
  const mpcc_rate_resolved_execution_artifact::ExecutionArtifact & artifact,
  const mpcc_execution_contract::ControlIntent current_intent,
  const std::uint64_t current_stage_geometry_id) noexcept
{
  namespace execution = mpcc_rate_resolved_execution_artifact;
  namespace race = race_mpcc_foundation;
  Result result;
  result.artifact_reason = execution::validate(artifact);
  if (result.artifact_reason != execution::RejectReason::None) {
    return result;
  }
  if (artifact.identity.source_context.intent != current_intent) {
    result.reason = RejectReason::IntentMismatch;
    return result;
  }
  if (
    current_stage_geometry_id == 0U ||
    artifact.identity.source_context.stage_geometry_id !=
    current_stage_geometry_id)
  {
    result.reason = RejectReason::StageGeometryMismatch;
    return result;
  }

  const std::size_t state_count = artifact.predicted_states.size();
  std::size_t rollout_sample_count{};
  for (const auto & control : artifact.control_stages) {
    rollout_sample_count += static_cast<std::size_t>(std::max(
      1.0, std::ceil(
        control.duration_sec /
        mpcc_rate_resolved::kMaximumPhysicalIntegrationStepSec)));
  }
  race::ExactPhysicalExecutionTrajectory exact;
  exact.progress_origin_m = artifact.course_progress_origin_m;
  exact.elapsed_time_sec.reserve(rollout_sample_count);
  exact.path_distance_m.reserve(rollout_sample_count);
  exact.lateral_m.reserve(rollout_sample_count);
  exact.lag_m.reserve(rollout_sample_count);
  exact.heading_offset_rad.reserve(rollout_sample_count);
  exact.velocity_mps.reserve(rollout_sample_count);
  exact.progress_m.reserve(rollout_sample_count);
  exact.lateral_lower_m.reserve(rollout_sample_count);
  exact.lateral_upper_m.reserve(rollout_sample_count);
  exact.minimum_lateral_bound_reserve_m =
    std::numeric_limits<double>::infinity();
  const double residual_bound_m = artifact.maximum_constraint_violation + 1e-9;
  exact.velocity_lower_bound_tolerance_mps = residual_bound_m;
  exact.lateral_bound_tolerance_m =
    execution::physical_lateral_bound_tolerance_m(artifact);
  // Preserve the solver certificate diagnostics, but never use its affine
  // state samples as physical wall evidence.  They only prove the assembled
  // QP; the exact command sequence below is independently replayed through
  // the nonlinear Frenet/yaw-response model.
  for (std::size_t state_index = 1U; state_index < state_count; ++state_index) {
    const auto & state = artifact.predicted_states[state_index];
    const auto & previous_state = artifact.predicted_states[state_index - 1U];
    const auto & transition = artifact.control_stages[state_index - 1U];
    const double progress_delta_m = state.progress_m - previous_state.progress_m;
    if (progress_delta_m < result.minimum_progress_delta_m) {
      result.minimum_progress_transition_state =
        static_cast<int>(state_index);
      result.minimum_progress_delta_m = progress_delta_m;
      result.transition_virtual_progress_speed_mps =
        transition.virtual_progress_speed_mps;
      result.transition_duration_sec = transition.duration_sec;
      result.progress_dynamics_defect_m = progress_delta_m -
        transition.virtual_progress_speed_mps * transition.duration_sec;
    }
    result.certified_progress_regression_tolerance_m = std::max(
      result.certified_progress_regression_tolerance_m,
      std::max(
        0.0, residual_bound_m * (1.0 + transition.duration_sec) -
        transition.virtual_progress_lower_mps * transition.duration_sec));
  }
  const auto & initial = artifact.predicted_states.front();
  NonlinearState nonlinear{
    initial.lateral_m, initial.lag_m, initial.heading_offset_rad,
    initial.velocity_mps, initial.progress_m, initial.steering_rad,
    initial.response_steering_rad};
  double elapsed_sec{};
  for (std::size_t stage = 0U; stage < artifact.control_stages.size(); ++stage) {
    const auto & control = artifact.control_stages[stage];
    const auto substep_count = static_cast<std::size_t>(std::max(
      1.0, std::ceil(
        control.duration_sec /
        mpcc_rate_resolved::kMaximumPhysicalIntegrationStepSec)));
    const double step_sec =
      control.duration_sec / static_cast<double>(substep_count);
    const double path_start_m = artifact.nominal_path_distance_m[stage];
    const double path_end_m = artifact.nominal_path_distance_m[stage + 1U];
    const double lower_start_m = artifact.lateral_lower_m[stage];
    const double lower_end_m = artifact.lateral_lower_m[stage + 1U];
    const double upper_start_m = artifact.lateral_upper_m[stage];
    const double upper_end_m = artifact.lateral_upper_m[stage + 1U];
    for (std::size_t substep = 0U; substep < substep_count; ++substep) {
      if (!advance_nonlinear_state(nonlinear, control, artifact, step_sec)) {
        result.reason = RejectReason::ExactTrajectoryRejected;
        result.rejected_stage = static_cast<int>(exact.path_distance_m.size());
        return result;
      }
      elapsed_sec += step_sec;
      const double fraction =
        static_cast<double>(substep + 1U) /
        static_cast<double>(substep_count);
      const double interpolate = 1.0 - fraction;
      const double lower_m = interpolate * lower_start_m + fraction * lower_end_m;
      const double upper_m = interpolate * upper_start_m + fraction * upper_end_m;
      exact.elapsed_time_sec.push_back(elapsed_sec);
      exact.path_distance_m.push_back(
        interpolate * path_start_m + fraction * path_end_m);
      exact.lateral_m.push_back(nonlinear.lateral_m);
      exact.lag_m.push_back(nonlinear.lag_m);
      exact.heading_offset_rad.push_back(nonlinear.heading_offset_rad);
      exact.velocity_mps.push_back(nonlinear.velocity_mps);
      exact.progress_m.push_back(
        artifact.course_progress_origin_m + nonlinear.progress_m);
      exact.lateral_lower_m.push_back(lower_m);
      exact.lateral_upper_m.push_back(upper_m);
      exact.minimum_lateral_bound_reserve_m = std::min(
        exact.minimum_lateral_bound_reserve_m,
        std::min(
          nonlinear.lateral_m - lower_m,
          upper_m - nonlinear.lateral_m));
    }
  }
  if (std::isfinite(exact.minimum_lateral_bound_reserve_m)) {
    exact.minimum_lateral_bound_reserve_m = std::max(
      0.0, exact.minimum_lateral_bound_reserve_m);
  }
  exact.progress_regression_tolerance_m =
    result.certified_progress_regression_tolerance_m;
  const auto validation =
    race::validate_exact_physical_execution_trajectory(exact);
  result.exact_reason = validation.reason;
  result.rejected_stage = validation.stage;
  result.rejected_lateral_m = validation.rejected_lateral_m;
  result.rejected_lateral_lower_m = validation.rejected_lateral_lower_m;
  result.rejected_lateral_upper_m = validation.rejected_lateral_upper_m;
  if (!validation.complete) {
    result.reason = RejectReason::ExactTrajectoryRejected;
    return result;
  }
  result.reason = RejectReason::None;
  result.exact_trajectory = std::move(exact);
  return result;
}

ContinuationResult build_continuation(
  const mpcc_rate_resolved_execution_artifact::ExecutionArtifact & artifact,
  const mpcc_rate_resolved_execution_artifact::Cursor & cursor,
  const ContinuationInitialState & initial_state) noexcept
{
  namespace execution = mpcc_rate_resolved_execution_artifact;
  namespace race = race_mpcc_foundation;
  ContinuationResult result;
  if (execution::validate(artifact) != execution::RejectReason::None) {
    return result;
  }
  if (
    !cursor.available || cursor.sequence != artifact.identity.sequence ||
    cursor.control_stage_index >= artifact.control_stages.size() ||
    cursor.remaining_control_stage_count == 0U ||
    !std::isfinite(cursor.elapsed_sec) || cursor.elapsed_sec < 0.0 ||
    !std::isfinite(cursor.stage_elapsed_sec) || cursor.stage_elapsed_sec < 0.0)
  {
    result.reason = ContinuationRejectReason::InvalidCursor;
    return result;
  }
  const double global_tolerance =
    std::max(1e-9, artifact.physical_global_tolerance);
  const double lateral_tolerance_m =
    execution::physical_lateral_bound_tolerance_m(artifact);
  NonlinearState nonlinear{
    initial_state.lateral_m,
    initial_state.lag_m,
    initial_state.heading_offset_rad,
    initial_state.velocity_mps,
    initial_state.progress_m,
    initial_state.steering_rad,
    initial_state.response_steering_rad};
  if (
    !finite(nonlinear) || !std::isfinite(lateral_tolerance_m) ||
    nonlinear.velocity_mps < -global_tolerance ||
    std::abs(nonlinear.steering_rad) >
    artifact.maximum_abs_steering_rad + global_tolerance ||
    std::abs(nonlinear.response_steering_rad) >
    artifact.maximum_abs_steering_rad + global_tolerance)
  {
    result.reason = ContinuationRejectReason::InvalidInitialState;
    return result;
  }

  const std::size_t first_stage = cursor.control_stage_index;
  const auto & first_control = artifact.control_stages[first_stage];
  if (cursor.stage_elapsed_sec >= first_control.duration_sec - 1e-12) {
    result.reason = ContinuationRejectReason::InvalidCursor;
    return result;
  }
  const double first_stage_remaining_sec =
    first_control.duration_sec - cursor.stage_elapsed_sec;
  if (
    first_stage_remaining_sec + artifact.physical_global_tolerance <
    artifact.publication_interval_sec)
  {
    // The current serialized command is held until the next publication.  A
    // piecewise solver suffix which switches control stages before that point
    // is not evidence for the command which can actually be placed on the
    // wire.  Fail closed and let a fresh artifact or Emergency Stop own this
    // boundary cycle.
    result.reason = ContinuationRejectReason::InvalidCursor;
    return result;
  }
  const double first_fraction = std::clamp(
    cursor.stage_elapsed_sec / first_control.duration_sec, 0.0, 1.0);
  const auto interpolate = [](const double start, const double end,
      const double fraction) noexcept {
      return start + fraction * (end - start);
    };
  const double initial_lower_m = interpolate(
    artifact.lateral_lower_m[first_stage],
    artifact.lateral_lower_m[first_stage + 1U], first_fraction);
  const double initial_upper_m = interpolate(
    artifact.lateral_upper_m[first_stage],
    artifact.lateral_upper_m[first_stage + 1U], first_fraction);
  if (
    nonlinear.lateral_m < initial_lower_m - lateral_tolerance_m ||
    nonlinear.lateral_m > initial_upper_m + lateral_tolerance_m)
  {
    result.reason =
      ContinuationRejectReason::InitialLateralBoundRejected;
    return result;
  }

  std::size_t rollout_sample_count{};
  for (std::size_t stage = first_stage;
    stage < artifact.control_stages.size(); ++stage)
  {
    const double duration_sec = stage == first_stage ?
      artifact.control_stages[stage].duration_sec - cursor.stage_elapsed_sec :
      artifact.control_stages[stage].duration_sec;
    rollout_sample_count += static_cast<std::size_t>(std::max(
      1.0, std::ceil(
        duration_sec /
        mpcc_rate_resolved::kMaximumPhysicalIntegrationStepSec)));
  }

  race::ExactPhysicalExecutionTrajectory exact;
  exact.progress_origin_m = artifact.course_progress_origin_m;
  exact.elapsed_time_sec.reserve(rollout_sample_count);
  exact.path_distance_m.reserve(rollout_sample_count);
  exact.lateral_m.reserve(rollout_sample_count);
  exact.lag_m.reserve(rollout_sample_count);
  exact.heading_offset_rad.reserve(rollout_sample_count);
  exact.velocity_mps.reserve(rollout_sample_count);
  exact.progress_m.reserve(rollout_sample_count);
  exact.lateral_lower_m.reserve(rollout_sample_count);
  exact.lateral_upper_m.reserve(rollout_sample_count);
  exact.minimum_lateral_bound_reserve_m =
    std::numeric_limits<double>::infinity();
  const double residual_bound_m = artifact.maximum_constraint_violation + 1e-9;
  exact.velocity_lower_bound_tolerance_mps = residual_bound_m;
  exact.lateral_bound_tolerance_m = lateral_tolerance_m;
  double elapsed_sec{};
  std::size_t publisher_interval_sample_count{};
  const auto retain_publisher_interval_prefix = [&] () -> bool {
      if (publisher_interval_sample_count < 1U) {
        return false;
      }
      const auto retain_prefix = [publisher_interval_sample_count](auto & values) {
          values.resize(publisher_interval_sample_count);
        };
      retain_prefix(exact.elapsed_time_sec);
      retain_prefix(exact.path_distance_m);
      retain_prefix(exact.lateral_m);
      retain_prefix(exact.lag_m);
      retain_prefix(exact.heading_offset_rad);
      retain_prefix(exact.velocity_mps);
      retain_prefix(exact.progress_m);
      retain_prefix(exact.lateral_lower_m);
      retain_prefix(exact.lateral_upper_m);
      exact.minimum_lateral_bound_reserve_m =
        std::numeric_limits<double>::infinity();
      for (std::size_t index = 0U;
        index < publisher_interval_sample_count; ++index)
      {
        exact.minimum_lateral_bound_reserve_m = std::min(
          exact.minimum_lateral_bound_reserve_m,
          std::min(
            exact.lateral_m[index] - exact.lateral_lower_m[index],
            exact.lateral_upper_m[index] - exact.lateral_m[index]));
      }
      const auto prefix_validation =
        race::validate_exact_physical_execution_trajectory(exact);
      if (!prefix_validation.complete) {
        return false;
      }
      result.scope = ContinuationProofScope::PublisherIntervalPrefix;
      result.stage_end_velocity_mps = {exact.velocity_mps.back()};
      // The first serialized tire angle is held over this prefix. The exact
      // nonlinear state, rather than the SQP's un-serialized rate input, owns
      // the publisher-boundary steering state.
      result.stage_end_steering_rad = {nonlinear.steering_rad};
      result.reason = ContinuationRejectReason::None;
      result.exact_trajectory = std::move(exact);
      return true;
    };
  for (std::size_t stage = first_stage;
    stage < artifact.control_stages.size(); ++stage)
  {
    const auto & control = artifact.control_stages[stage];
    const double consumed_sec = stage == first_stage ?
      cursor.stage_elapsed_sec : 0.0;
    const double duration_sec = control.duration_sec - consumed_sec;
    if (!std::isfinite(duration_sec) || duration_sec <= 0.0) {
      result.reason = ContinuationRejectReason::InvalidCursor;
      return result;
    }
    const double consumed_fraction = consumed_sec / control.duration_sec;
    const double path_start_m = interpolate(
      artifact.nominal_path_distance_m[stage],
      artifact.nominal_path_distance_m[stage + 1U], consumed_fraction);
    const double path_end_m = artifact.nominal_path_distance_m[stage + 1U];
    const double lower_start_m = interpolate(
      artifact.lateral_lower_m[stage],
      artifact.lateral_lower_m[stage + 1U], consumed_fraction);
    const double lower_end_m = artifact.lateral_lower_m[stage + 1U];
    const double upper_start_m = interpolate(
      artifact.lateral_upper_m[stage],
      artifact.lateral_upper_m[stage + 1U], consumed_fraction);
    const double upper_end_m = artifact.lateral_upper_m[stage + 1U];
    double stage_rollout_sec{};
    while (stage_rollout_sec < duration_sec - 1e-12) {
      double step_sec = std::min(
        mpcc_rate_resolved::kMaximumPhysicalIntegrationStepSec,
        duration_sec - stage_rollout_sec);
      if (
        elapsed_sec < artifact.publication_interval_sec - 1e-12 &&
        elapsed_sec + step_sec > artifact.publication_interval_sec)
      {
        step_sec = artifact.publication_interval_sec - elapsed_sec;
      }
      if (!std::isfinite(step_sec) || step_sec <= 0.0) {
        result.reason = ContinuationRejectReason::NonlinearModelRejected;
        result.rejected_stage = static_cast<int>(exact.path_distance_m.size());
        if (retain_publisher_interval_prefix()) {
          return result;
        }
        return result;
      }
      auto applied_control = control;
      if (elapsed_sec < artifact.publication_interval_sec - 1e-12) {
        // AckermannControlCommand serializes tire angle, not steering rate.
        // The already-published angle therefore remains constant until the
        // next publication boundary. Applying the SQP rate here proves a
        // different command from the one that actually crossed the wire.
        applied_control.steering_rate_radps = 0.0;
      }
      if (!advance_nonlinear_state(
          nonlinear, applied_control, artifact, step_sec))
      {
        result.reason = ContinuationRejectReason::NonlinearModelRejected;
        result.rejected_stage = static_cast<int>(exact.path_distance_m.size());
        if (retain_publisher_interval_prefix()) {
          return result;
        }
        return result;
      }
      if (
        nonlinear.velocity_mps < -residual_bound_m ||
        std::abs(nonlinear.steering_rad) >
        artifact.maximum_abs_steering_rad + global_tolerance ||
        std::abs(nonlinear.response_steering_rad) >
        artifact.maximum_abs_steering_rad + global_tolerance)
      {
        result.reason = ContinuationRejectReason::ActuatorEnvelopeRejected;
        result.rejected_stage = static_cast<int>(exact.path_distance_m.size());
        if (retain_publisher_interval_prefix()) {
          return result;
        }
        return result;
      }
      stage_rollout_sec += step_sec;
      elapsed_sec += step_sec;
      const double fraction = stage_rollout_sec / duration_sec;
      const double lower_m = interpolate(lower_start_m, lower_end_m, fraction);
      const double upper_m = interpolate(upper_start_m, upper_end_m, fraction);
      exact.elapsed_time_sec.push_back(elapsed_sec);
      exact.path_distance_m.push_back(
        interpolate(path_start_m, path_end_m, fraction));
      exact.lateral_m.push_back(nonlinear.lateral_m);
      exact.lag_m.push_back(nonlinear.lag_m);
      exact.heading_offset_rad.push_back(nonlinear.heading_offset_rad);
      exact.velocity_mps.push_back(nonlinear.velocity_mps);
      exact.progress_m.push_back(
        artifact.course_progress_origin_m + nonlinear.progress_m);
      exact.lateral_lower_m.push_back(lower_m);
      exact.lateral_upper_m.push_back(upper_m);
      exact.minimum_lateral_bound_reserve_m = std::min(
        exact.minimum_lateral_bound_reserve_m,
        std::min(
          nonlinear.lateral_m - lower_m,
          upper_m - nonlinear.lateral_m));
      if (
        std::abs(
          elapsed_sec - artifact.publication_interval_sec) <=
        artifact.physical_global_tolerance)
      {
        publisher_interval_sample_count = exact.path_distance_m.size();
        result.publisher_interval_end_steering_rad = nonlinear.steering_rad;
        result.publisher_interval_end_response_steering_rad =
          nonlinear.response_steering_rad;
      }
    }
    result.stage_end_velocity_mps.push_back(nonlinear.velocity_mps);
    result.stage_end_steering_rad.push_back(nonlinear.steering_rad);
    exact.progress_regression_tolerance_m = std::max(
      exact.progress_regression_tolerance_m,
      std::max(
        0.0, residual_bound_m * (1.0 + duration_sec) -
        control.virtual_progress_lower_mps * duration_sec));
  }
  if (std::isfinite(exact.minimum_lateral_bound_reserve_m)) {
    exact.minimum_lateral_bound_reserve_m = std::max(
      0.0, exact.minimum_lateral_bound_reserve_m);
  }
  const auto validation =
    race::validate_exact_physical_execution_trajectory(exact);
  result.exact_reason = validation.reason;
  if (!validation.complete) {
    if (
      publisher_interval_sample_count >= 1U && validation.stage >= 0 &&
      static_cast<std::size_t>(validation.stage) >=
      publisher_interval_sample_count)
    {
      if (retain_publisher_interval_prefix()) {
        return result;
      }
    }
    result.reason = ContinuationRejectReason::ExactTrajectoryRejected;
    result.rejected_stage = validation.stage;
    return result;
  }
  result.reason = ContinuationRejectReason::None;
  result.exact_trajectory = std::move(exact);
  return result;
}

StopContingencyResult build_stop_contingency(
  const mpcc_rate_resolved_execution_artifact::ExecutionArtifact & artifact,
  const mpcc_rate_resolved_execution_artifact::Cursor & cursor,
  const mpcc_rate_resolved_execution_artifact::Actuation & current_actuation,
  const ContinuationInitialState & initial_state,
  const StopCourseGeometry & course_geometry,
  const race_mpcc_foundation::StopPathTrackingPolicy & lateral_policy,
  const double minimum_acceleration_mps2,
  const double target_lateral_m,
  const StopLateralTargetProfile * const target_profile) noexcept
{
  namespace execution = mpcc_rate_resolved_execution_artifact;
  namespace race = race_mpcc_foundation;
  StopContingencyResult result;
  if (execution::validate(artifact) != execution::RejectReason::None) {
    return result;
  }
  if (!stop_course_geometry_valid(course_geometry)) {
    result.reason = StopContingencyRejectReason::CourseGeometryUnavailable;
    return result;
  }
  if (
    !cursor.available || cursor.sequence != artifact.identity.sequence ||
    cursor.control_stage_index >= artifact.control_stages.size() ||
    cursor.remaining_control_stage_count == 0U)
  {
    result.reason = StopContingencyRejectReason::InvalidCursor;
    return result;
  }
  NonlinearState nonlinear{
    initial_state.lateral_m, initial_state.lag_m,
    initial_state.heading_offset_rad, initial_state.velocity_mps,
    initial_state.progress_m, initial_state.steering_rad,
    initial_state.response_steering_rad};
  const double tolerance = std::max(1e-9, artifact.physical_global_tolerance);
  if (
    !finite(nonlinear) || nonlinear.velocity_mps < -tolerance ||
    std::abs(nonlinear.steering_rad) >
    artifact.maximum_abs_steering_rad + tolerance ||
    std::abs(nonlinear.response_steering_rad) >
    artifact.maximum_abs_steering_rad + tolerance)
  {
    result.reason = StopContingencyRejectReason::InvalidInitialState;
    return result;
  }
  if (
    current_actuation.sequence != artifact.identity.sequence ||
    current_actuation.control_stage_index != cursor.control_stage_index ||
    !std::isfinite(current_actuation.acceleration_mps2) ||
    !std::isfinite(current_actuation.steering_rate_radps) ||
    !std::isfinite(current_actuation.steering_rad) ||
    std::abs(current_actuation.steering_rad - nonlinear.steering_rad) >
    tolerance ||
    std::abs(current_actuation.steering_rate_radps) >
    artifact.maximum_abs_steering_rate_radps + tolerance ||
    std::abs(current_actuation.steering_rad) >
    artifact.maximum_abs_steering_rad + tolerance)
  {
    result.reason = StopContingencyRejectReason::InvalidActuation;
    return result;
  }
  if (
    !std::isfinite(minimum_acceleration_mps2) ||
    !std::isfinite(target_lateral_m) ||
    minimum_acceleration_mps2 >= 0.0 ||
    (target_profile != nullptr &&
    !stop_lateral_target_profile_valid(*target_profile)))
  {
    result.reason = StopContingencyRejectReason::InvalidBrakingEnvelope;
    return result;
  }
  const auto & current_stage =
    artifact.control_stages[cursor.control_stage_index];
  if (
    current_actuation.acceleration_mps2 <
    current_stage.acceleration_lower_mps2 - tolerance ||
    current_actuation.acceleration_mps2 >
    current_stage.acceleration_upper_mps2 + tolerance ||
    minimum_acceleration_mps2 <
    current_stage.acceleration_lower_mps2 - tolerance ||
    minimum_acceleration_mps2 >
    current_stage.acceleration_upper_mps2 + tolerance)
  {
    result.reason = StopContingencyRejectReason::InvalidBrakingEnvelope;
    return result;
  }

  const double braking_duration_sec =
    std::max(
    0.0, nonlinear.velocity_mps +
    std::max(0.0, current_actuation.acceleration_mps2) *
    artifact.publication_interval_sec) /
    -minimum_acceleration_mps2;
  const std::size_t reserve_count = static_cast<std::size_t>(std::max(
      1.0, std::ceil(
        (artifact.publication_interval_sec + braking_duration_sec) /
        mpcc_rate_resolved::kMaximumPhysicalIntegrationStepSec))) + 1U;
  race::ExactPhysicalExecutionTrajectory exact;
  exact.progress_origin_m = artifact.course_progress_origin_m;
  exact.elapsed_time_sec.reserve(reserve_count);
  exact.path_distance_m.reserve(reserve_count);
  exact.lateral_m.reserve(reserve_count);
  exact.lag_m.reserve(reserve_count);
  exact.heading_offset_rad.reserve(reserve_count);
  exact.velocity_mps.reserve(reserve_count);
  exact.progress_m.reserve(reserve_count);
  exact.lateral_lower_m.reserve(reserve_count);
  exact.lateral_upper_m.reserve(reserve_count);
  exact.velocity_lower_bound_tolerance_mps = tolerance;
  exact.lateral_bound_tolerance_m =
    execution::physical_lateral_bound_tolerance_m(artifact);
  exact.progress_regression_tolerance_m = tolerance;
  exact.minimum_lateral_bound_reserve_m =
    std::numeric_limits<double>::infinity();
  result.actuation_samples.reserve(reserve_count);
  double elapsed_sec{};
  double path_distance_m{};
  std::size_t command_interval_index{};

  const auto initial_geometry = sample_course_geometry(
    course_geometry, tolerance, nonlinear.progress_m);
  if (!initial_geometry.has_value()) {
    result.reason = StopContingencyRejectReason::CourseGeometryUnavailable;
    return result;
  }
  result.initial_lateral_lower_m = initial_geometry->lateral_lower_m;
  result.initial_lateral_upper_m = initial_geometry->lateral_upper_m;

  const auto append_duration = [&] (
      const double requested_duration_sec,
      const double requested_acceleration_mps2,
      const double requested_steering_rate_radps) -> bool
    {
      double remaining_sec = requested_duration_sec;
      while (remaining_sec > 1e-12) {
        double step_sec = std::min(
          mpcc_rate_resolved::kMaximumPhysicalIntegrationStepSec,
          remaining_sec);
        double effective_acceleration_mps2 = requested_acceleration_mps2;
        if (
          nonlinear.velocity_mps <= tolerance &&
          effective_acceleration_mps2 < 0.0)
        {
          effective_acceleration_mps2 = 0.0;
        } else if (
          effective_acceleration_mps2 < 0.0 &&
          nonlinear.velocity_mps + effective_acceleration_mps2 * step_sec < 0.0)
        {
          step_sec = nonlinear.velocity_mps / -effective_acceleration_mps2;
        }
        if (!std::isfinite(step_sec) || step_sec <= 0.0) {
          return false;
        }
        const auto geometry = sample_course_geometry(
          course_geometry, tolerance, nonlinear.progress_m);
        if (!geometry.has_value()) {
          result.reason =
            StopContingencyRejectReason::CourseGeometryUnavailable;
          return false;
        }
        const auto progress_speed = physical_progress_speed(
          nonlinear, geometry->curvature_radpm,
          artifact.minimum_frenet_denominator);
        if (!progress_speed.has_value()) {
          result.reason =
            StopContingencyRejectReason::CourseGeometryUnavailable;
          return false;
        }
        execution::ControlStage control;
        control.acceleration_mps2 = effective_acceleration_mps2;
        control.steering_rate_radps = requested_steering_rate_radps;
        control.virtual_progress_speed_mps = progress_speed.value();
        control.duration_sec = step_sec;
        control.virtual_progress_lower_mps = 0.0;
        control.virtual_progress_upper_mps =
          std::max(progress_speed.value(), tolerance);
        control.acceleration_lower_mps2 = minimum_acceleration_mps2;
        control.acceleration_upper_mps2 =
          current_stage.acceleration_upper_mps2;
        control.path_curvature_radpm = geometry->curvature_radpm;
        const double velocity_before_mps = nonlinear.velocity_mps;
        if (!advance_nonlinear_state(
            nonlinear, control, artifact, step_sec))
        {
          result.reason =
            StopContingencyRejectReason::NonlinearModelRejected;
          return false;
        }
        if (
          nonlinear.velocity_mps < -tolerance ||
          std::abs(nonlinear.steering_rad) >
          artifact.maximum_abs_steering_rad + tolerance ||
          std::abs(nonlinear.response_steering_rad) >
          artifact.maximum_abs_steering_rad + tolerance)
        {
          result.reason =
            StopContingencyRejectReason::ActuatorEnvelopeRejected;
          return false;
        }
        if (nonlinear.velocity_mps < 0.0) {
          nonlinear.velocity_mps = 0.0;
        }
        const auto endpoint_geometry = sample_course_geometry(
          course_geometry, tolerance, nonlinear.progress_m);
        if (!endpoint_geometry.has_value()) {
          result.reason =
            StopContingencyRejectReason::CourseGeometryUnavailable;
          return false;
        }
        elapsed_sec += step_sec;
        remaining_sec -= step_sec;
        path_distance_m += 0.5 *
          (std::max(0.0, velocity_before_mps) + nonlinear.velocity_mps) *
          step_sec;
        exact.elapsed_time_sec.push_back(elapsed_sec);
        exact.path_distance_m.push_back(path_distance_m);
        exact.lateral_m.push_back(nonlinear.lateral_m);
        exact.lag_m.push_back(nonlinear.lag_m);
        exact.heading_offset_rad.push_back(nonlinear.heading_offset_rad);
        exact.velocity_mps.push_back(nonlinear.velocity_mps);
        exact.progress_m.push_back(
          artifact.course_progress_origin_m + nonlinear.progress_m);
        const double approximate_support_violation_m = std::max(
          {0.0,
            endpoint_geometry->lateral_lower_m - nonlinear.lateral_m,
            nonlinear.lateral_m - endpoint_geometry->lateral_upper_m});
        if (approximate_support_violation_m > tolerance) {
          result.approximate_lateral_support_exceeded = true;
          if (
            result.first_approximate_lateral_support_exceeded_sample < 0)
          {
            result.first_approximate_lateral_support_exceeded_sample =
              static_cast<int>(exact.lateral_m.size() - 1U);
          }
          result.maximum_approximate_lateral_support_violation_m = std::max(
            result.maximum_approximate_lateral_support_violation_m,
            approximate_support_violation_m);
        }
        // StopCourseGeometry supplies interpolation and path-tracking support.
        // It is conservative and may exclude an exact-grid-clear physical
        // state. Do not let that approximation pre-empt the downstream swept
        // footprint occupancy-grid proof, which remains the wall authority.
        exact.lateral_lower_m.push_back(std::min(
          endpoint_geometry->lateral_lower_m, nonlinear.lateral_m));
        exact.lateral_upper_m.push_back(std::max(
          endpoint_geometry->lateral_upper_m, nonlinear.lateral_m));
        exact.minimum_lateral_bound_reserve_m = std::min(
          exact.minimum_lateral_bound_reserve_m,
          std::min(
            nonlinear.lateral_m - endpoint_geometry->lateral_lower_m,
            endpoint_geometry->lateral_upper_m - nonlinear.lateral_m));
        result.actuation_samples.push_back(
          StopContingencyResult::ActuationSample{
            elapsed_sec, step_sec, requested_acceleration_mps2,
            effective_acceleration_mps2,
            requested_steering_rate_radps, nonlinear.velocity_mps,
            nonlinear.steering_rad, nonlinear.response_steering_rad,
            geometry->curvature_radpm, progress_speed.value(),
            command_interval_index});
      }
      return true;
    };

  if (!append_duration(
      artifact.publication_interval_sec,
      current_actuation.acceleration_mps2,
      0.0))
  {
    if (result.reason == StopContingencyRejectReason::InvalidArtifact) {
      result.reason = StopContingencyRejectReason::NonlinearModelRejected;
    }
    return result;
  }
  ++command_interval_index;
  result.publisher_interval_end_steering_rad = nonlinear.steering_rad;
  result.publisher_interval_sample_count = result.actuation_samples.size();
  while (nonlinear.velocity_mps > tolerance) {
    const auto geometry = sample_course_geometry(
      course_geometry, tolerance, nonlinear.progress_m);
    if (!geometry.has_value()) {
      result.reason = StopContingencyRejectReason::CourseGeometryUnavailable;
      return result;
    }
    const auto sampled_target_lateral_m = target_profile == nullptr ?
      std::optional<double>{target_lateral_m} :
      sample_stop_lateral_target(
        *target_profile, nonlinear.progress_m, tolerance);
    if (!sampled_target_lateral_m.has_value()) {
      result.reason = StopContingencyRejectReason::InvalidLateralPolicy;
      return result;
    }
    const auto lateral_command =
      race::resolve_stop_path_tracking_command(
      race::StopPathTrackingCommandRequest{
        lateral_policy, nonlinear.lateral_m,
        nonlinear.heading_offset_rad, geometry->curvature_radpm,
        nonlinear.velocity_mps, nonlinear.steering_rad,
        artifact.publication_interval_sec,
        sampled_target_lateral_m.value()});
    if (!lateral_command.has_value()) {
      result.reason = StopContingencyRejectReason::InvalidLateralPolicy;
      return result;
    }
    const double stop_duration_sec = nonlinear.velocity_mps /
      -minimum_acceleration_mps2;
    const double command_duration_sec = std::min(
      artifact.publication_interval_sec, stop_duration_sec);
    if (!append_duration(
        command_duration_sec, minimum_acceleration_mps2,
        lateral_command->steering_rate_radps))
    {
      if (result.reason == StopContingencyRejectReason::InvalidArtifact) {
        result.reason = StopContingencyRejectReason::NonlinearModelRejected;
      }
      return result;
    }
    ++command_interval_index;
  }
  result.braking_suffix_final_steering_rad = nonlinear.steering_rad;
  if (exact.elapsed_time_sec.empty() || nonlinear.velocity_mps > tolerance) {
    result.reason = StopContingencyRejectReason::ExactTrajectoryRejected;
    return result;
  }
  if (
    result.publisher_interval_sample_count == 0U ||
    result.publisher_interval_sample_count > result.actuation_samples.size() ||
    result.actuation_samples.size() != exact.elapsed_time_sec.size())
  {
    result.reason = StopContingencyRejectReason::ExactTrajectoryRejected;
    return result;
  }
  if (std::isfinite(exact.minimum_lateral_bound_reserve_m)) {
    exact.minimum_lateral_bound_reserve_m = std::max(
      0.0, exact.minimum_lateral_bound_reserve_m);
  }
  const auto validation =
    race::validate_exact_physical_execution_trajectory(exact);
  result.exact_reason = validation.reason;
  result.rejected_sample = validation.stage;
  if (!validation.complete) {
    result.reason = StopContingencyRejectReason::ExactTrajectoryRejected;
    return result;
  }
  result.reason = StopContingencyRejectReason::None;
  result.exact_trajectory = std::move(exact);
  return result;
}

StopContingencyResult build_stop_successor(
  const mpcc_rate_resolved_execution_artifact::ExecutionArtifact & artifact,
  const ContinuationInitialState & initial_state,
  const StopCourseGeometry & course_geometry,
  const race_mpcc_foundation::StopPathTrackingPolicy & lateral_policy,
  const double minimum_acceleration_mps2) noexcept
{
  namespace execution = mpcc_rate_resolved_execution_artifact;
  StopContingencyResult rejected;
  if (execution::validate(artifact) != execution::RejectReason::None) {
    return rejected;
  }
  if (
    !std::isfinite(minimum_acceleration_mps2) ||
    minimum_acceleration_mps2 >= 0.0)
  {
    rejected.reason = StopContingencyRejectReason::InvalidBrakingEnvelope;
    return rejected;
  }
  const double tolerance = std::max(1e-9, artifact.physical_global_tolerance);
  const bool braking_owned_by_every_source_stage = std::all_of(
    artifact.control_stages.begin(), artifact.control_stages.end(),
    [minimum_acceleration_mps2, tolerance](
      const execution::ControlStage & stage) {
      return minimum_acceleration_mps2 >=
             stage.acceleration_lower_mps2 - tolerance &&
             minimum_acceleration_mps2 <=
             stage.acceleration_upper_mps2 + tolerance;
    });
  if (!braking_owned_by_every_source_stage) {
    rejected.reason = StopContingencyRejectReason::InvalidBrakingEnvelope;
    return rejected;
  }

  // The successor does not extend or revive an exhausted normal cursor.  A
  // synthetic cursor only transports the immutable source identity and the
  // already-certified physical input envelope into the common Stop builder.
  // Maximum braking begins immediately; the first publisher interval holds
  // the last serialized steering state exactly.
  const execution::Cursor successor_cursor{
    true, execution::CursorReason::Available, artifact.identity.sequence,
    0U, artifact.control_stages.size(), 0.0, 0.0};
  const execution::Actuation successor_actuation{
    artifact.identity.sequence, 0U, initial_state.velocity_mps,
    minimum_acceleration_mps2, 0.0, initial_state.steering_rad,
    artifact.control_stages.front().path_curvature_radpm, 0.0};
  return build_stop_contingency(
    artifact, successor_cursor, successor_actuation, initial_state,
    course_geometry, lateral_policy, minimum_acceleration_mps2);
}

}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_physical_adapter
