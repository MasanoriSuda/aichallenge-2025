#include "multi_purpose_mpc_ros/mpcc_rate_resolved_execution_artifact.hpp"

#include <cmath>
#include <limits>
#include <utility>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_execution_artifact
{
namespace
{

bool supported_intent(
  const mpcc_execution_contract::ControlIntent intent) noexcept
{
  return intent == mpcc_execution_contract::ControlIntent::Track ||
         intent == mpcc_execution_contract::ControlIntent::Cruise;
}

bool finite_state(const PredictedState & state) noexcept
{
  return std::isfinite(state.lateral_m) && std::isfinite(state.lag_m) &&
         std::isfinite(state.heading_offset_rad) &&
         std::isfinite(state.velocity_mps) &&
         std::isfinite(state.progress_m) &&
         std::isfinite(state.steering_rad);
}

bool finite_control(const ControlStage & control) noexcept
{
  return std::isfinite(control.acceleration_mps2) &&
         std::isfinite(control.steering_rate_radps) &&
         std::isfinite(control.virtual_progress_speed_mps) &&
         std::isfinite(control.duration_sec) && control.duration_sec > 0.0;
}

mpcc_rate_resolved::CertifiedActuationSequenceSampleEvaluation sample_steering(
  const ExecutionArtifact & artifact, const double elapsed_sec) noexcept
{
  std::vector<double> rates;
  std::vector<double> durations;
  rates.reserve(artifact.control_stages.size());
  durations.reserve(artifact.control_stages.size());
  for (const auto & control : artifact.control_stages) {
    rates.push_back(control.steering_rate_radps);
    durations.push_back(control.duration_sec);
  }
  return mpcc_rate_resolved::evaluate_certified_actuation_sequence_sample(
    mpcc_rate_resolved::CertifiedActuationSequenceSampleRequest{
      artifact.semantic_initial_steering_rad, std::move(rates),
      std::move(durations), elapsed_sec, artifact.maximum_abs_steering_rad,
      artifact.wheelbase_m,
      artifact.maximum_normalized_constraint_violation});
}

}  // namespace

bool identity_valid(const Identity & identity) noexcept
{
  return identity.sequence > 0U && identity.decision_id > 0U &&
         identity.source_problem_fingerprint > 0U &&
         identity.stage_geometry_id > 0U && supported_intent(identity.intent) &&
         std::isfinite(identity.snapshot_sec) && identity.snapshot_sec >= 0.0;
}

const char * to_string(const RejectReason reason) noexcept
{
  switch (reason) {
    case RejectReason::None: return "none";
    case RejectReason::InvalidIdentity: return "invalid-identity";
    case RejectReason::InvalidTiming: return "invalid-timing";
    case RejectReason::InvalidLimits: return "invalid-limits";
    case RejectReason::InvalidCertificate: return "invalid-certificate";
    case RejectReason::EmptyHorizon: return "empty-horizon";
    case RejectReason::StateCountMismatch: return "state-count-mismatch";
    case RejectReason::CorridorCountMismatch: return "corridor-count-mismatch";
    case RejectReason::InvalidPredictedState: return "invalid-predicted-state";
    case RejectReason::InvalidControlStage: return "invalid-control-stage";
    case RejectReason::InvalidLateralCorridor: return "invalid-lateral-corridor";
    case RejectReason::InitialSteeringMismatch: return "initial-steering-mismatch";
    case RejectReason::SteeringDynamicsMismatch: return "steering-dynamics-mismatch";
    case RejectReason::SemanticSteeringSequenceRejected:
      return "semantic-steering-sequence-rejected";
  }
  return "unknown";
}

RejectReason validate(const ExecutionArtifact & artifact) noexcept
{
  constexpr double half_pi = 1.57079632679489661923;
  if (!identity_valid(artifact.identity)) {
    return RejectReason::InvalidIdentity;
  }
  if (
    !std::isfinite(artifact.prediction_origin_sec) ||
    !std::isfinite(artifact.completed_sec) ||
    artifact.prediction_origin_sec < 0.0 ||
    artifact.completed_sec < artifact.prediction_origin_sec ||
    std::abs(
      artifact.prediction_origin_sec - artifact.identity.snapshot_sec) > 1e-12)
  {
    return RejectReason::InvalidTiming;
  }
  if (
    !std::isfinite(artifact.semantic_initial_steering_rad) ||
    !std::isfinite(artifact.wheelbase_m) || artifact.wheelbase_m <= 0.0 ||
    !std::isfinite(artifact.maximum_abs_steering_rad) ||
    artifact.maximum_abs_steering_rad <= 0.0 ||
    artifact.maximum_abs_steering_rad >= half_pi ||
    !std::isfinite(artifact.maximum_abs_steering_rate_radps) ||
    artifact.maximum_abs_steering_rate_radps <= 0.0)
  {
    return RejectReason::InvalidLimits;
  }
  if (
    !std::isfinite(artifact.physical_global_tolerance) ||
    artifact.physical_global_tolerance <= 0.0 ||
    !std::isfinite(artifact.maximum_constraint_violation) ||
    artifact.maximum_constraint_violation < 0.0 ||
    !std::isfinite(artifact.maximum_normalized_constraint_violation) ||
    artifact.maximum_normalized_constraint_violation < 0.0 ||
    artifact.maximum_normalized_constraint_violation > 1.0)
  {
    return RejectReason::InvalidCertificate;
  }
  if (artifact.control_stages.empty()) {
    return RejectReason::EmptyHorizon;
  }
  if (
    artifact.control_stages.size() ==
      std::numeric_limits<std::size_t>::max() ||
    artifact.predicted_states.size() != artifact.control_stages.size() + 1U)
  {
    return RejectReason::StateCountMismatch;
  }
  if (
    artifact.lateral_lower_m.size() != artifact.predicted_states.size() ||
    artifact.lateral_upper_m.size() != artifact.predicted_states.size())
  {
    return RejectReason::CorridorCountMismatch;
  }
  const double tolerance = artifact.physical_global_tolerance;
  for (std::size_t index = 0U; index < artifact.predicted_states.size(); ++index) {
    const auto & state = artifact.predicted_states[index];
    if (!finite_state(state)) {
      return RejectReason::InvalidPredictedState;
    }
    const double lower = artifact.lateral_lower_m[index];
    const double upper = artifact.lateral_upper_m[index];
    if (
      !std::isfinite(lower) || !std::isfinite(upper) || upper < lower ||
      state.lateral_m < lower - tolerance ||
      state.lateral_m > upper + tolerance)
    {
      return RejectReason::InvalidLateralCorridor;
    }
    if (
      std::abs(state.steering_rad) >
      artifact.maximum_abs_steering_rad + tolerance)
    {
      return RejectReason::InvalidPredictedState;
    }
  }
  if (
    std::abs(
      artifact.predicted_states.front().steering_rad -
      artifact.semantic_initial_steering_rad) > tolerance)
  {
    return RejectReason::InitialSteeringMismatch;
  }
  double horizon_sec = 0.0;
  for (std::size_t index = 0U; index < artifact.control_stages.size(); ++index) {
    const auto & control = artifact.control_stages[index];
    if (
      !finite_control(control) ||
      std::abs(control.steering_rate_radps) >
      artifact.maximum_abs_steering_rate_radps + tolerance)
    {
      return RejectReason::InvalidControlStage;
    }
    horizon_sec += control.duration_sec;
    if (!std::isfinite(horizon_sec)) {
      return RejectReason::InvalidControlStage;
    }
    const double predicted_next_steering =
      artifact.predicted_states[index].steering_rad +
      control.steering_rate_radps * control.duration_sec;
    if (
      std::abs(
        predicted_next_steering -
        artifact.predicted_states[index + 1U].steering_rad) > tolerance)
    {
      return RejectReason::SteeringDynamicsMismatch;
    }
  }
  if (!sample_steering(artifact, horizon_sec).sample.has_value()) {
    return RejectReason::SemanticSteeringSequenceRejected;
  }
  return RejectReason::None;
}

const char * to_string(const CursorReason reason) noexcept
{
  switch (reason) {
    case CursorReason::Available: return "available";
    case CursorReason::InvalidArtifact: return "invalid-artifact";
    case CursorReason::InvalidTime: return "invalid-time";
    case CursorReason::FutureArtifact: return "future-artifact";
    case CursorReason::Exhausted: return "exhausted";
  }
  return "unknown";
}

Cursor resolve_cursor(
  const ExecutionArtifact & artifact, const double now_sec) noexcept
{
  Cursor cursor;
  cursor.sequence = artifact.identity.sequence;
  if (validate(artifact) != RejectReason::None) {
    return cursor;
  }
  if (!std::isfinite(now_sec) || now_sec < 0.0) {
    cursor.reason = CursorReason::InvalidTime;
    return cursor;
  }
  if (now_sec < artifact.prediction_origin_sec) {
    cursor.reason = CursorReason::FutureArtifact;
    return cursor;
  }
  double elapsed_sec = now_sec - artifact.prediction_origin_sec;
  cursor.elapsed_sec = elapsed_sec;
  for (std::size_t index = 0U; index < artifact.control_stages.size(); ++index) {
    const double duration_sec = artifact.control_stages[index].duration_sec;
    if (elapsed_sec < duration_sec) {
      cursor.available = true;
      cursor.reason = CursorReason::Available;
      cursor.control_stage_index = index;
      cursor.remaining_control_stage_count =
        artifact.control_stages.size() - index;
      cursor.stage_elapsed_sec = elapsed_sec;
      return cursor;
    }
    elapsed_sec -= duration_sec;
  }
  cursor.reason = CursorReason::Exhausted;
  return cursor;
}

const char * to_string(const ActuationReason reason) noexcept
{
  switch (reason) {
    case ActuationReason::Available: return "available";
    case ActuationReason::InvalidArtifact: return "invalid-artifact";
    case ActuationReason::CursorUnavailable: return "cursor-unavailable";
    case ActuationReason::IdentityMismatch: return "identity-mismatch";
    case ActuationReason::InvalidStageIndex: return "invalid-stage-index";
    case ActuationReason::SampleRejected: return "sample-rejected";
    case ActuationReason::NonfiniteActuation: return "nonfinite-actuation";
  }
  return "unknown";
}

ActuationResult extract_actuation(
  const ExecutionArtifact & artifact, const Cursor & cursor) noexcept
{
  ActuationResult result;
  if (validate(artifact) != RejectReason::None) {
    return result;
  }
  if (!cursor.available) {
    result.reason = ActuationReason::CursorUnavailable;
    return result;
  }
  if (cursor.sequence != artifact.identity.sequence) {
    result.reason = ActuationReason::IdentityMismatch;
    return result;
  }
  const std::size_t stage = cursor.control_stage_index;
  if (
    stage >= artifact.control_stages.size() ||
    cursor.remaining_control_stage_count !=
    artifact.control_stages.size() - stage ||
    !std::isfinite(cursor.elapsed_sec) || cursor.elapsed_sec < 0.0 ||
    !std::isfinite(cursor.stage_elapsed_sec) ||
    cursor.stage_elapsed_sec < 0.0 ||
    cursor.stage_elapsed_sec >= artifact.control_stages[stage].duration_sec)
  {
    result.reason = ActuationReason::InvalidStageIndex;
    return result;
  }
  const auto sample = sample_steering(artifact, cursor.elapsed_sec);
  result.sample_reason = sample.reason;
  if (!sample.sample.has_value()) {
    result.reason = ActuationReason::SampleRejected;
    return result;
  }
  const auto & state = artifact.predicted_states[stage];
  const auto & control = artifact.control_stages[stage];
  Actuation actuation;
  actuation.sequence = artifact.identity.sequence;
  actuation.control_stage_index = stage;
  actuation.predicted_speed_mps =
    state.velocity_mps + control.acceleration_mps2 * cursor.stage_elapsed_sec;
  actuation.acceleration_mps2 = control.acceleration_mps2;
  actuation.steering_rate_radps = control.steering_rate_radps;
  actuation.steering_rad = sample.sample->steering_rad;
  actuation.curvature_radpm = sample.sample->curvature_radpm;
  actuation.virtual_progress_speed_mps = control.virtual_progress_speed_mps;
  if (
    !std::isfinite(actuation.predicted_speed_mps) ||
    !std::isfinite(actuation.acceleration_mps2) ||
    !std::isfinite(actuation.steering_rate_radps) ||
    !std::isfinite(actuation.steering_rad) ||
    !std::isfinite(actuation.curvature_radpm) ||
    !std::isfinite(actuation.virtual_progress_speed_mps))
  {
    result.reason = ActuationReason::NonfiniteActuation;
    return result;
  }
  result.reason = ActuationReason::Available;
  result.actuation = actuation;
  return result;
}

}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_execution_artifact
