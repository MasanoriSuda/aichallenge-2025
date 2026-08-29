#include "multi_purpose_mpc_ros/mpcc_rate_resolved_production_adapter.hpp"

#include "multi_purpose_mpc_ros/mpc_stage_geometry.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_command_candidate.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <utility>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_production_adapter
{
namespace
{

namespace artifact = mpcc_rate_resolved_execution_artifact;
namespace candidate = mpcc_rate_resolved_command_candidate;
namespace certified = mpcc_rate_resolved_certified_plan;
namespace physical = mpcc_rate_resolved_physical_wall;

std::optional<double> project_certified_nonnegative(
  const double value, const double lower_bound_tolerance) noexcept
{
  if (
    !std::isfinite(value) || !std::isfinite(lower_bound_tolerance) ||
    lower_bound_tolerance < 0.0 || value < -lower_bound_tolerance)
  {
    return std::nullopt;
  }
  return std::max(0.0, value);
}

std::optional<std::pair<std::vector<double>, std::vector<double>>>
build_world_prediction(
  const physical::Snapshot & snapshot,
  const race_mpcc_foundation::ExactPhysicalExecutionTrajectory & trajectory)
noexcept
{
  if (
    !physical::snapshot_valid(snapshot) ||
    !race_mpcc_foundation::exact_physical_execution_trajectory_complete(
      trajectory))
  {
    return std::nullopt;
  }
  std::pair<std::vector<double>, std::vector<double>> prediction;
  prediction.first.reserve(trajectory.progress_m.size());
  prediction.second.reserve(trajectory.progress_m.size());
  const double tolerance_m = std::max(1e-9, snapshot.bound_tolerance_m);
  for (
    std::size_t index = 0U; index < trajectory.progress_m.size(); ++index)
  {
    const auto frame = mpc_stage_geometry::sample_course_frame(
      snapshot.course_frame_knots, trajectory.progress_m[index],
      tolerance_m);
    if (!frame.has_value()) {
      return std::nullopt;
    }
    const auto pose = contract::reconstruct_planar_pose_from_frenet(
      contract::PlanarPose{frame->x_m, frame->y_m, frame->heading_rad},
      contract::FrenetPose{
        trajectory.lateral_m[index], trajectory.lag_m[index],
        trajectory.heading_offset_rad[index]});
    if (!pose.has_value()) {
      return std::nullopt;
    }
    prediction.first.push_back(pose->x_m);
    prediction.second.push_back(pose->y_m);
  }
  if (prediction.first.empty()) {
    return std::nullopt;
  }
  return prediction;
}

}  // namespace

const char * to_string(const Reason reason) noexcept
{
  switch (reason) {
    case Reason::Available: return "available";
    case Reason::RetainedProofUnavailable: return "retained-proof-unavailable";
    case Reason::InvalidPlan: return "invalid-plan";
    case Reason::InvalidIdentity: return "invalid-identity";
    case Reason::InvalidPhysicalEvidence: return "invalid-physical-evidence";
    case Reason::InvalidCursor: return "invalid-cursor";
    case Reason::InvalidActuation: return "invalid-actuation";
    case Reason::AuthorityRejected: return "authority-rejected";
    case Reason::CommandRejected: return "command-rejected";
    case Reason::PredictionRejected: return "prediction-rejected";
    case Reason::Count: break;
  }
  return "unknown";
}

Result build(const retained::Result & retained_result) noexcept
{
  Result result;
  if (
    retained_result.reason != retained::Reason::Accepted ||
    !retained_result.proof.has_value())
  {
    return result;
  }
  const auto & proof = retained_result.proof.value();
  if (
    proof.plan == nullptr || proof.plan->execution_artifact == nullptr ||
    proof.plan->physical_snapshot == nullptr ||
    certified::validate(*proof.plan) != certified::RejectReason::None)
  {
    result.reason = Reason::InvalidPlan;
    return result;
  }
  const auto & execution = *proof.plan->execution_artifact;
  const auto & source_context = execution.identity.source_context;
  if (
    !contract::problem_context_complete(source_context) ||
    !artifact::supports_intent(source_context.intent) ||
    source_context.formulation !=
    contract::Formulation::VelocitySteeringYawResponseProgress7State ||
    proof.decision_id == 0U)
  {
    result.reason = Reason::InvalidIdentity;
    return result;
  }
  if (
    proof.plan->physical_outcome != physical::Outcome::Accepted ||
    !physical::snapshot_valid(*proof.plan->physical_snapshot) ||
    !physical::same_identity(
      proof.plan->physical_identity,
      proof.plan->physical_snapshot->identity))
  {
    result.reason = Reason::InvalidPhysicalEvidence;
    return result;
  }
  if (
    !proof.cursor.available ||
    proof.cursor.sequence != execution.identity.sequence ||
    proof.cursor.remaining_control_stage_count == 0U ||
    proof.cursor.control_stage_index >= execution.control_stages.size() ||
    proof.proved_control_stage_count == 0U ||
    proof.proved_control_stage_count >
    proof.cursor.remaining_control_stage_count)
  {
    result.reason = Reason::InvalidCursor;
    return result;
  }
  const bool full_suffix =
    proof.proved_control_stage_count ==
    proof.cursor.remaining_control_stage_count &&
    proof.static_wall_scope == retained::StaticWallProofScope::FullSuffix &&
    proof.dynamic_obstacle_scope ==
    retained::DynamicObstacleProofScope::FullSuffix &&
    proof.continuation_scope ==
    mpcc_rate_resolved_physical_adapter::ContinuationProofScope::FullSuffix;
  const bool bounded_publisher_interval =
    proof.proved_control_stage_count == 1U &&
    proof.terminal_stop_certified &&
    race_mpcc_foundation::exact_physical_execution_trajectory_complete(
      proof.terminal_stop_trajectory);
  if (!full_suffix && !bounded_publisher_interval) {
    result.reason = Reason::InvalidCursor;
    return result;
  }
  const auto command_candidate = candidate::build(retained_result);
  if (!command_candidate.candidate.has_value()) {
    result.reason = Reason::InvalidActuation;
    return result;
  }
  const auto & sampled = command_candidate.candidate.value();
  if (
    sampled.source_context.fingerprint != source_context.fingerprint ||
    sampled.decision_id != proof.decision_id ||
    sampled.artifact_sequence != execution.identity.sequence ||
    sampled.control_stage_index != proof.cursor.control_stage_index)
  {
    result.reason = Reason::InvalidIdentity;
    return result;
  }

  // The physical proof admits lower-bound residuals only inside its recorded
  // solver tolerance.  Convert that certified numerical representation to the
  // exact non-negative actuator representation once, at this boundary.  The
  // immutable execution artifact and its physical evidence remain untouched.
  const double velocity_lower_bound_tolerance_mps =
    proof.plan->physical_snapshot->trajectory
    .velocity_lower_bound_tolerance_mps;
  const auto projected_speed_mps = project_certified_nonnegative(
    sampled.predicted_speed_mps, velocity_lower_bound_tolerance_mps);
  const auto projected_virtual_progress_speed_mps =
    project_certified_nonnegative(
    sampled.virtual_progress_speed_mps,
    execution.physical_global_tolerance);
  if (
    !projected_speed_mps.has_value() ||
    !projected_virtual_progress_speed_mps.has_value())
  {
    result.reason = Reason::InvalidActuation;
    return result;
  }

  const double horizon_duration_sec = std::accumulate(
    execution.control_stages.begin(), execution.control_stages.end(), 0.0,
    [](const double total, const artifact::ControlStage & stage) {
      return total + stage.duration_sec;
    });
  contract::CertifiedMpccSolution solution;
  solution.solution_id = execution.identity.sequence;
  solution.problem_fingerprint = source_context.fingerprint;
  solution.formulation = source_context.formulation;
  solution.solved = true;
  solution.finite = true;
  solution.constraints_satisfied = true;
  solution.maximum_constraint_violation =
    execution.maximum_constraint_violation;
  solution.physical.checked = true;
  solution.physical.wall_clear = true;
  solution.physical.obstacles_clear = true;
  solution.physical.minimum_wall_clearance_m =
    proof.continuation_trajectory.minimum_lateral_bound_reserve_m;
  solution.physical.minimum_obstacle_clearance_m =
    proof.minimum_dynamic_clearance_m;
  // The solution identity remains the original complete horizon; executable
  // suffix length is carried independently by CanonicalNormalCandidate.
  solution.prediction_stage_count = execution.control_stages.size();
  solution.valid_until_sec =
    execution.prediction_origin_sec + horizon_duration_sec;
  if (!contract::solution_certified(solution)) {
    result.reason = Reason::InvalidPhysicalEvidence;
    return result;
  }

  contract::CanonicalNormalCandidate retained_candidate;
  retained_candidate.problem = source_context;
  retained_candidate.solution = solution;
  retained_candidate.executable_control_stage_count =
    proof.proved_control_stage_count;
  retained_candidate.execution_plan_id = execution.identity.sequence;
  retained_candidate.execution_certificate_decision_id = proof.decision_id;
  retained_candidate.execution_first_control_stage_index =
    proof.cursor.control_stage_index;
  retained_candidate.terminal_contingency_certified =
    proof.terminal_stop_certified;
  retained_candidate.execution_physical = solution.physical;
  const auto authority_resolution =
    contract::resolve_canonical_normal_authority(
    contract::CanonicalNormalAuthorityRequest{
      proof.decision_id, proof.observation_origin_sec, {}, retained_candidate,
      source_context.intent});
  if (
    authority_resolution.source !=
    contract::CanonicalNormalAuthoritySource::RetainedCertified ||
    !authority_resolution.problem.has_value() ||
    !authority_resolution.solution.has_value())
  {
    result.reason = Reason::AuthorityRejected;
    return result;
  }
  const auto command_result = contract::build_canonical_normal_command(
    authority_resolution,
    contract::CanonicalActuation{
      projected_speed_mps.value(), sampled.acceleration_mps2,
      sampled.curvature_radpm, sampled.steering_rad,
      projected_virtual_progress_speed_mps.value()});
  if (!command_result.command.has_value()) {
    result.reason = Reason::CommandRejected;
    return result;
  }
  const auto world_prediction = build_world_prediction(
    *proof.plan->physical_snapshot, proof.continuation_trajectory);
  if (!world_prediction.has_value()) {
    result.reason = Reason::PredictionRejected;
    return result;
  }

  Authority authority;
  authority.problem = source_context;
  authority.solution = solution;
  authority.command = command_result.command.value();
  authority.first_control_stage_index = proof.cursor.control_stage_index;
  authority.maximum_abs_steering_rad = std::abs(sampled.steering_rad);
  authority.target_speed_horizon_mps.reserve(
    proof.proved_control_stage_count);
  authority.steering_horizon_rad.reserve(
    proof.proved_control_stage_count);
  if (
    proof.continuation_stage_end_velocity_mps.size() !=
    proof.proved_control_stage_count ||
    proof.continuation_stage_end_steering_rad.size() !=
    proof.proved_control_stage_count)
  {
    result.reason = Reason::PredictionRejected;
    return result;
  }
  for (
    std::size_t offset = 0U;
    offset < proof.proved_control_stage_count; ++offset)
  {
    double speed_mps = projected_speed_mps.value();
    double steering_rad = sampled.steering_rad;
    if (offset > 0U) {
      const auto projected_horizon_speed_mps = project_certified_nonnegative(
        proof.continuation_stage_end_velocity_mps[offset - 1U],
        velocity_lower_bound_tolerance_mps);
      if (!projected_horizon_speed_mps.has_value()) {
        result.reason = Reason::PredictionRejected;
        return result;
      }
      speed_mps = projected_horizon_speed_mps.value();
      steering_rad = proof.continuation_stage_end_steering_rad[offset - 1U];
    }
    if (
      !std::isfinite(steering_rad))
    {
      result.reason = Reason::PredictionRejected;
      return result;
    }
    authority.target_speed_horizon_mps.push_back(speed_mps);
    authority.steering_horizon_rad.push_back(steering_rad);
    authority.maximum_abs_steering_rad = std::max(
      authority.maximum_abs_steering_rad, std::abs(steering_rad));
  }
  authority.world_prediction = world_prediction.value();
  result.reason = Reason::Available;
  result.authority = std::move(authority);
  return result;
}

}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_production_adapter
