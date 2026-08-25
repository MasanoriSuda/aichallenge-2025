#include "multi_purpose_mpc_ros/mpcc_rate_resolved_execution_source.hpp"

#include <cmath>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_execution_source
{

const char * to_string(const RejectReason reason) noexcept
{
  switch (reason) {
    case RejectReason::None: return "none";
    case RejectReason::MissingPlan: return "missing-plan";
    case RejectReason::InvalidCertifiedPlan: return "invalid-certified-plan";
    case RejectReason::UnsupportedIntent: return "unsupported-intent";
    case RejectReason::IntentMismatch: return "intent-mismatch";
    case RejectReason::TargetMismatch: return "target-mismatch";
    case RejectReason::MissionGenerationMismatch:
      return "mission-generation-mismatch";
    case RejectReason::SideMismatch: return "side-mismatch";
    case RejectReason::InvalidSourceTime: return "invalid-source-time";
  }
  return "unknown";
}

Result build(const Request & request)
{
  Result result;
  if (request.plan == nullptr) {
    return result;
  }
  if (
    certified::validate(*request.plan) != certified::RejectReason::None ||
    request.plan->execution_artifact == nullptr ||
    request.plan->physical_snapshot == nullptr)
  {
    result.reason = RejectReason::InvalidCertifiedPlan;
    return result;
  }
  if (
    !contract::canonical_normal_intent_requires_execution_side(request.intent) ||
    request.target_id.empty() || request.mission_generation == 0U ||
    (request.side_sign != -1 && request.side_sign != 1))
  {
    result.reason = RejectReason::UnsupportedIntent;
    return result;
  }

  const auto & artifact = *request.plan->execution_artifact;
  const auto & context = artifact.identity.source_context;
  if (context.intent != request.intent) {
    result.reason = RejectReason::IntentMismatch;
    return result;
  }
  if (context.target_id != request.target_id) {
    result.reason = RejectReason::TargetMismatch;
    return result;
  }
  if (context.intent_generation != request.mission_generation) {
    result.reason = RejectReason::MissionGenerationMismatch;
    return result;
  }
  if (context.execution_side_sign != request.side_sign) {
    result.reason = RejectReason::SideMismatch;
    return result;
  }
  if (
    !std::isfinite(artifact.identity.snapshot_sec) ||
    !std::isfinite(artifact.completed_sec) ||
    artifact.completed_sec + 1e-9 < artifact.identity.snapshot_sec)
  {
    result.reason = RejectReason::InvalidSourceTime;
    return result;
  }

  const auto & exact = request.plan->physical_snapshot->trajectory;
  result.source.artifact_sequence = artifact.identity.sequence;
  result.source.source_context = context;
  result.source.source_snapshot_sec = artifact.identity.snapshot_sec;
  result.source.source_completed_sec = artifact.completed_sec;
  result.source.course_progress_origin_m = exact.progress_origin_m;
  result.source.minimum_lateral_bound_reserve_m =
    exact.minimum_lateral_bound_reserve_m;
  result.source.path_distance_m = exact.path_distance_m;
  result.source.lateral_m = exact.lateral_m;
  result.source.progress_m = exact.progress_m;
  result.reason = RejectReason::None;
  return result;
}

}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_execution_source
