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

std::optional<std::pair<std::vector<double>, std::vector<double>>>
build_world_prediction(
  const physical::Snapshot & snapshot, const artifact::Cursor & cursor) noexcept
{
  if (
    !cursor.available || !physical::snapshot_valid(snapshot) ||
    cursor.control_stage_index >= snapshot.trajectory.progress_m.size())
  {
    return std::nullopt;
  }
  std::pair<std::vector<double>, std::vector<double>> prediction;
  const std::size_t remaining =
    snapshot.trajectory.progress_m.size() - cursor.control_stage_index;
  prediction.first.reserve(remaining);
  prediction.second.reserve(remaining);
  const double tolerance_m = std::max(1e-9, snapshot.bound_tolerance_m);
  for (
    std::size_t index = cursor.control_stage_index;
    index < snapshot.trajectory.progress_m.size(); ++index)
  {
    const auto frame = mpc_stage_geometry::sample_course_frame(
      snapshot.course_frame_knots, snapshot.trajectory.progress_m[index],
      tolerance_m);
    if (!frame.has_value()) {
      return std::nullopt;
    }
    const auto pose = contract::reconstruct_planar_pose_from_frenet(
      contract::PlanarPose{frame->x_m, frame->y_m, frame->heading_rad},
      contract::FrenetPose{
        snapshot.trajectory.lateral_m[index],
        snapshot.trajectory.lag_m[index],
        snapshot.trajectory.heading_offset_rad[index]});
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
    contract::Formulation::VelocitySteeringProgress6State ||
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
    proof.cursor.control_stage_index >= execution.control_stages.size())
  {
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
    proof.plan->physical_snapshot->trajectory.minimum_lateral_bound_reserve_m;
  solution.physical.minimum_obstacle_clearance_m =
    proof.minimum_dynamic_clearance_m;
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
    proof.cursor.remaining_control_stage_count;
  retained_candidate.execution_plan_id = execution.identity.sequence;
  retained_candidate.execution_certificate_decision_id = proof.decision_id;
  retained_candidate.execution_first_control_stage_index =
    proof.cursor.control_stage_index;
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
      sampled.predicted_speed_mps, sampled.acceleration_mps2,
      sampled.curvature_radpm, sampled.steering_rad,
      sampled.virtual_progress_speed_mps});
  if (!command_result.command.has_value()) {
    result.reason = Reason::CommandRejected;
    return result;
  }
  const auto world_prediction = build_world_prediction(
    *proof.plan->physical_snapshot, proof.cursor);
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
    proof.cursor.remaining_control_stage_count);
  authority.steering_horizon_rad.reserve(
    proof.cursor.remaining_control_stage_count);
  for (
    std::size_t offset = 0U;
    offset < proof.cursor.remaining_control_stage_count; ++offset)
  {
    double speed_mps = sampled.predicted_speed_mps;
    double steering_rad = sampled.steering_rad;
    if (offset > 0U) {
      const std::size_t state_index =
        proof.cursor.control_stage_index + offset;
      if (state_index >= execution.predicted_states.size()) {
        result.reason = Reason::PredictionRejected;
        return result;
      }
      speed_mps = execution.predicted_states[state_index].velocity_mps;
      steering_rad = execution.predicted_states[state_index].steering_rad;
    }
    if (
      !std::isfinite(speed_mps) || speed_mps < 0.0 ||
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
