#include "multi_purpose_mpc_ros/mpcc_rate_resolved_physical_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_physical_adapter
{
namespace
{

constexpr double kMaximumNonlinearRolloutStepSec = 0.01;

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

double response_steering_after_ramp(
  const double initial_command_rad,
  const double initial_response_rad,
  const double steering_rate_radps,
  const double elapsed_sec,
  const double time_constant_sec) noexcept
{
  const double decay = std::exp(-elapsed_sec / time_constant_sec);
  return initial_command_rad +
         (initial_response_rad - initial_command_rad) * decay +
         steering_rate_radps *
         (elapsed_sec - time_constant_sec * (1.0 - decay));
}

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
  const double response_mid_rad = response_steering_after_ramp(
    state.steering_rad, state.response_steering_rad,
    control.steering_rate_radps, 0.5 * step_sec,
    artifact.yaw_response_time_constant_sec);
  const double response_next_rad = response_steering_after_ramp(
    state.steering_rad, state.response_steering_rad,
    control.steering_rate_radps, step_sec,
    artifact.yaw_response_time_constant_sec);
  const double velocity_mid_mps =
    state.velocity_mps + 0.5 * control.acceleration_mps2 * step_sec;
  const double heading_rate_mid_radps =
    artifact.yaw_response_gain * velocity_mid_mps *
    std::tan(response_mid_rad) / artifact.wheelbase_m -
    control.path_curvature_radpm *
    control.virtual_progress_speed_mps;
  const double heading_mid_rad =
    state.heading_offset_rad + 0.5 * heading_rate_mid_radps * step_sec;
  const double lateral_rate_mid_mps =
    velocity_mid_mps * std::sin(heading_mid_rad);
  const double lateral_mid_m =
    state.lateral_m + 0.5 * lateral_rate_mid_mps * step_sec;
  const double frenet_denominator =
    1.0 - control.path_curvature_radpm * lateral_mid_m;
  if (
    !std::isfinite(response_mid_rad) ||
    !std::isfinite(response_next_rad) ||
    !std::isfinite(velocity_mid_mps) ||
    !std::isfinite(heading_rate_mid_radps) ||
    !std::isfinite(heading_mid_rad) ||
    !std::isfinite(lateral_rate_mid_mps) ||
    !std::isfinite(frenet_denominator) ||
    frenet_denominator < artifact.minimum_frenet_denominator)
  {
    return false;
  }
  const double physical_progress_rate_mps =
    velocity_mid_mps * std::cos(heading_mid_rad) / frenet_denominator;
  state.lateral_m += lateral_rate_mid_mps * step_sec;
  state.lag_m +=
    (physical_progress_rate_mps -
    control.virtual_progress_speed_mps) * step_sec;
  state.heading_offset_rad += heading_rate_mid_radps * step_sec;
  state.velocity_mps += control.acceleration_mps2 * step_sec;
  state.progress_m += control.virtual_progress_speed_mps * step_sec;
  state.steering_rad += control.steering_rate_radps * step_sec;
  state.response_steering_rad = response_next_rad;
  return finite(state);
}

}  // namespace

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
    case ContinuationProofScope::CurrentStagePrefix:
      return "current-stage-prefix";
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
        control.duration_sec / kMaximumNonlinearRolloutStepSec)));
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
        control.duration_sec / kMaximumNonlinearRolloutStepSec)));
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
  const double tolerance = std::max(1e-9, artifact.physical_global_tolerance);
  NonlinearState nonlinear{
    initial_state.lateral_m,
    initial_state.lag_m,
    initial_state.heading_offset_rad,
    initial_state.velocity_mps,
    initial_state.progress_m,
    initial_state.steering_rad,
    initial_state.response_steering_rad};
  if (
    !finite(nonlinear) || nonlinear.velocity_mps < -tolerance ||
    std::abs(nonlinear.steering_rad) >
    artifact.maximum_abs_steering_rad + tolerance ||
    std::abs(nonlinear.response_steering_rad) >
    artifact.maximum_abs_steering_rad + tolerance)
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
    nonlinear.lateral_m < initial_lower_m - tolerance ||
    nonlinear.lateral_m > initial_upper_m + tolerance)
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
      1.0, std::ceil(duration_sec / kMaximumNonlinearRolloutStepSec)));
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
  double elapsed_sec{};
  std::size_t current_stage_sample_count{};
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
    const auto substep_count = static_cast<std::size_t>(std::max(
      1.0, std::ceil(duration_sec / kMaximumNonlinearRolloutStepSec)));
    const double step_sec = duration_sec / static_cast<double>(substep_count);
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
    for (std::size_t substep = 0U; substep < substep_count; ++substep) {
      if (!advance_nonlinear_state(nonlinear, control, artifact, step_sec)) {
        result.reason = ContinuationRejectReason::ExactTrajectoryRejected;
        result.rejected_stage = static_cast<int>(exact.path_distance_m.size());
        return result;
      }
      if (
        nonlinear.velocity_mps < -residual_bound_m ||
        std::abs(nonlinear.steering_rad) >
        artifact.maximum_abs_steering_rad + tolerance ||
        std::abs(nonlinear.response_steering_rad) >
        artifact.maximum_abs_steering_rad + tolerance)
      {
        result.reason = ContinuationRejectReason::ExactTrajectoryRejected;
        result.rejected_stage = static_cast<int>(exact.path_distance_m.size());
        return result;
      }
      elapsed_sec += step_sec;
      const double fraction =
        static_cast<double>(substep + 1U) /
        static_cast<double>(substep_count);
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
    }
    result.stage_end_velocity_mps.push_back(nonlinear.velocity_mps);
    result.stage_end_steering_rad.push_back(nonlinear.steering_rad);
    if (stage == first_stage) {
      current_stage_sample_count = exact.path_distance_m.size();
    }
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
      current_stage_sample_count >= 2U && validation.stage >= 0 &&
      static_cast<std::size_t>(validation.stage) >=
      current_stage_sample_count)
    {
      const auto retain_current_stage_prefix =
        [current_stage_sample_count](auto & values) {
          values.resize(current_stage_sample_count);
        };
      retain_current_stage_prefix(exact.elapsed_time_sec);
      retain_current_stage_prefix(exact.path_distance_m);
      retain_current_stage_prefix(exact.lateral_m);
      retain_current_stage_prefix(exact.lag_m);
      retain_current_stage_prefix(exact.heading_offset_rad);
      retain_current_stage_prefix(exact.velocity_mps);
      retain_current_stage_prefix(exact.progress_m);
      retain_current_stage_prefix(exact.lateral_lower_m);
      retain_current_stage_prefix(exact.lateral_upper_m);
      exact.minimum_lateral_bound_reserve_m =
        std::numeric_limits<double>::infinity();
      for (std::size_t index = 0U; index < current_stage_sample_count; ++index) {
        exact.minimum_lateral_bound_reserve_m = std::min(
          exact.minimum_lateral_bound_reserve_m,
          std::min(
            exact.lateral_m[index] - exact.lateral_lower_m[index],
            exact.lateral_upper_m[index] - exact.lateral_m[index]));
      }
      const auto prefix_validation =
        race::validate_exact_physical_execution_trajectory(exact);
      if (prefix_validation.complete) {
        result.scope = ContinuationProofScope::CurrentStagePrefix;
        result.stage_end_velocity_mps.resize(1U);
        result.stage_end_steering_rad.resize(1U);
        result.reason = ContinuationRejectReason::None;
        result.exact_trajectory = std::move(exact);
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

}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_physical_adapter
