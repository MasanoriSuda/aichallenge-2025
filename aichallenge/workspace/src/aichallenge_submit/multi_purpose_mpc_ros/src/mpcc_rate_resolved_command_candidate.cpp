#include "multi_purpose_mpc_ros/mpcc_rate_resolved_command_candidate.hpp"

#include <cmath>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_command_candidate
{

const char * to_string(const Reason reason) noexcept
{
  switch (reason) {
    case Reason::Available:
      return "available";
    case Reason::RetainedProofUnavailable:
      return "retained-proof-unavailable";
    case Reason::InvalidCertifiedPlan:
      return "invalid-certified-plan";
    case Reason::InvalidIdentity:
      return "invalid-identity";
    case Reason::IdentityMismatch:
      return "identity-mismatch";
    case Reason::InvalidActuation:
      return "invalid-actuation";
    case Reason::Count:
      break;
  }
  return "unknown";
}

Result build(const retained::Result & retained_result) noexcept
{
  Result result;
  if (retained_result.reason != retained::Reason::Accepted ||
    !retained_result.proof.has_value())
  {
    return result;
  }
  const auto & proof = retained_result.proof.value();
  if (proof.plan == nullptr || proof.plan->execution_artifact == nullptr ||
    mpcc_rate_resolved_certified_plan::validate(*proof.plan) !=
    mpcc_rate_resolved_certified_plan::RejectReason::None)
  {
    result.reason = Reason::InvalidCertifiedPlan;
    return result;
  }
  const auto & artifact = *proof.plan->execution_artifact;
  const auto & identity = artifact.identity;
  if (!mpcc_rate_resolved_execution_artifact::identity_valid(identity)) {
    result.reason = Reason::InvalidIdentity;
    return result;
  }
  const auto & actuation = proof.actuation;
  if (proof.decision_id == 0U || !proof.cursor.available ||
    proof.cursor.sequence != identity.sequence ||
    actuation.sequence != identity.sequence ||
    actuation.control_stage_index != proof.cursor.control_stage_index ||
    actuation.control_stage_index >= artifact.control_stages.size())
  {
    result.reason = Reason::IdentityMismatch;
    return result;
  }
  if (!std::isfinite(artifact.prediction_origin_sec) ||
    !std::isfinite(actuation.predicted_speed_mps) ||
    !std::isfinite(actuation.acceleration_mps2) ||
    !std::isfinite(actuation.steering_rate_radps) ||
    !std::isfinite(actuation.steering_rad) ||
    !std::isfinite(actuation.curvature_radpm) ||
    !std::isfinite(actuation.virtual_progress_speed_mps))
  {
    result.reason = Reason::InvalidActuation;
    return result;
  }

  Candidate candidate;
  candidate.decision_id = proof.decision_id;
  candidate.artifact_sequence = identity.sequence;
  candidate.source_decision_id = identity.decision_id;
  candidate.source_problem_fingerprint = identity.source_problem_fingerprint;
  candidate.stage_geometry_id = identity.stage_geometry_id;
  candidate.intent = identity.intent;
  candidate.formulation = identity.formulation;
  candidate.control_stage_index = actuation.control_stage_index;
  candidate.prediction_origin_sec = artifact.prediction_origin_sec;
  candidate.predicted_speed_mps = actuation.predicted_speed_mps;
  candidate.acceleration_mps2 = actuation.acceleration_mps2;
  candidate.steering_rate_radps = actuation.steering_rate_radps;
  candidate.steering_rad = actuation.steering_rad;
  candidate.curvature_radpm = actuation.curvature_radpm;
  candidate.virtual_progress_speed_mps = actuation.virtual_progress_speed_mps;
  result.reason = Reason::Available;
  result.candidate = candidate;
  return result;
}

} // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_command_candidate
