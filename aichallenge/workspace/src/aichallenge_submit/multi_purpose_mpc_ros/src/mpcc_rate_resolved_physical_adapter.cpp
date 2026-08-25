#include "multi_purpose_mpc_ros/mpcc_rate_resolved_physical_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_physical_adapter
{

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
  race::ExactPhysicalExecutionTrajectory exact;
  exact.progress_origin_m = artifact.course_progress_origin_m;
  exact.path_distance_m.reserve(state_count - 1U);
  exact.lateral_m.reserve(state_count - 1U);
  exact.lag_m.reserve(state_count - 1U);
  exact.heading_offset_rad.reserve(state_count - 1U);
  exact.velocity_mps.reserve(state_count - 1U);
  exact.progress_m.reserve(state_count - 1U);
  exact.lateral_lower_m.reserve(state_count - 1U);
  exact.lateral_upper_m.reserve(state_count - 1U);
  exact.minimum_lateral_bound_reserve_m =
    std::numeric_limits<double>::infinity();
  const double residual_bound_m = artifact.maximum_constraint_violation + 1e-9;
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
    const double lower_m = artifact.lateral_lower_m[state_index];
    const double upper_m = artifact.lateral_upper_m[state_index];
    exact.path_distance_m.push_back(
      artifact.nominal_path_distance_m[state_index]);
    exact.lateral_m.push_back(state.lateral_m);
    exact.lag_m.push_back(state.lag_m);
    exact.heading_offset_rad.push_back(state.heading_offset_rad);
    exact.velocity_mps.push_back(state.velocity_mps);
    exact.progress_m.push_back(
      artifact.course_progress_origin_m + state.progress_m);
    exact.lateral_lower_m.push_back(lower_m);
    exact.lateral_upper_m.push_back(upper_m);
    exact.minimum_lateral_bound_reserve_m = std::min(
      exact.minimum_lateral_bound_reserve_m,
      std::min(state.lateral_m - lower_m, upper_m - state.lateral_m));
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

}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_physical_adapter
