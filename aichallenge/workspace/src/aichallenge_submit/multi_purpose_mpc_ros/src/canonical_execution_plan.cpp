#include "multi_purpose_mpc_ros/canonical_execution_plan.hpp"

#include <cmath>
#include <limits>
#include <utility>

namespace multi_purpose_mpc_ros::canonical_execution_plan
{
namespace
{

bool finite_state(const CanonicalPredictedState & state) noexcept
{
  return
    std::isfinite(state.lateral_m) && std::isfinite(state.lag_m) &&
    std::isfinite(state.heading_offset_rad) &&
    std::isfinite(state.velocity_mps) && state.velocity_mps >= 0.0 &&
    std::isfinite(state.progress_m);
}

bool finite_control(const CanonicalControlStage & control) noexcept
{
  return
    std::isfinite(control.acceleration_mps2) &&
    std::isfinite(control.curvature_radpm) &&
    std::isfinite(control.virtual_progress_speed_mps) &&
    control.virtual_progress_speed_mps >= 0.0 &&
    std::isfinite(control.duration_sec) && control.duration_sec > 0.0;
}

bool physical_certificate_accepted(
  const contract::PhysicalCertificate & physical) noexcept
{
  return physical.checked && physical.wall_clear && physical.obstacles_clear;
}

}  // namespace

const char * to_string(const CanonicalExecutionPlanRejectReason reason) noexcept
{
  switch (reason) {
    case CanonicalExecutionPlanRejectReason::None: return "none";
    case CanonicalExecutionPlanRejectReason::MissingPlanId: return "missing-plan-id";
    case CanonicalExecutionPlanRejectReason::IncompleteProblem: return "incomplete-problem";
    case CanonicalExecutionPlanRejectReason::UnsupportedIntent: return "unsupported-intent";
    case CanonicalExecutionPlanRejectReason::NoncanonicalFormulation:
      return "noncanonical-formulation";
    case CanonicalExecutionPlanRejectReason::SolutionIdentityMismatch:
      return "solution-identity-mismatch";
    case CanonicalExecutionPlanRejectReason::SolutionNotCertified:
      return "solution-not-certified";
    case CanonicalExecutionPlanRejectReason::InvalidSolveTime: return "invalid-solve-time";
    case CanonicalExecutionPlanRejectReason::EmptyHorizon: return "empty-horizon";
    case CanonicalExecutionPlanRejectReason::StateCountMismatch:
      return "state-count-mismatch";
    case CanonicalExecutionPlanRejectReason::ControlCountMismatch:
      return "control-count-mismatch";
    case CanonicalExecutionPlanRejectReason::CorridorCountMismatch:
      return "corridor-count-mismatch";
    case CanonicalExecutionPlanRejectReason::InvalidPredictedState:
      return "invalid-predicted-state";
    case CanonicalExecutionPlanRejectReason::InvalidControlStage:
      return "invalid-control-stage";
    case CanonicalExecutionPlanRejectReason::InvalidLateralCorridor:
      return "invalid-lateral-corridor";
    case CanonicalExecutionPlanRejectReason::InvalidLateralTrackingReserve:
      return "invalid-lateral-tracking-reserve";
  }
  return "unknown";
}

CanonicalExecutionPlanRejectReason validate_canonical_execution_plan(
  const CanonicalExecutionPlan & plan) noexcept
{
  if (plan.plan_id == 0U) {
    return CanonicalExecutionPlanRejectReason::MissingPlanId;
  }
  if (plan.problem.horizon_steps == 0U) {
    return CanonicalExecutionPlanRejectReason::EmptyHorizon;
  }
  if (!contract::problem_context_complete(plan.problem)) {
    return CanonicalExecutionPlanRejectReason::IncompleteProblem;
  }
  if (!contract::canonical_normal_intent_supported(plan.problem.intent)) {
    return CanonicalExecutionPlanRejectReason::UnsupportedIntent;
  }
  if (
    plan.problem.formulation != contract::Formulation::VelocityProgress5State ||
    plan.solution.formulation != contract::Formulation::VelocityProgress5State)
  {
    return CanonicalExecutionPlanRejectReason::NoncanonicalFormulation;
  }
  if (plan.solution.problem_fingerprint != plan.problem.fingerprint) {
    return CanonicalExecutionPlanRejectReason::SolutionIdentityMismatch;
  }
  if (!contract::solution_certified(plan.solution)) {
    return CanonicalExecutionPlanRejectReason::SolutionNotCertified;
  }
  if (
    !std::isfinite(plan.solved_sec) || plan.solved_sec < 0.0 ||
    plan.solution.valid_until_sec < plan.solved_sec)
  {
    return CanonicalExecutionPlanRejectReason::InvalidSolveTime;
  }
  const std::size_t horizon = plan.problem.horizon_steps;
  if (
    horizon == std::numeric_limits<std::size_t>::max() ||
    plan.predicted_states.size() != horizon + 1U)
  {
    return CanonicalExecutionPlanRejectReason::StateCountMismatch;
  }
  if (
    plan.control_stages.size() != horizon ||
    plan.solution.prediction_stage_count != horizon)
  {
    return CanonicalExecutionPlanRejectReason::ControlCountMismatch;
  }
  if (
    plan.lateral_lower_m.size() != horizon + 1U ||
    plan.lateral_upper_m.size() != horizon + 1U)
  {
    return CanonicalExecutionPlanRejectReason::CorridorCountMismatch;
  }
  if (
    !std::isfinite(plan.required_lateral_tracking_reserve_m) ||
    plan.required_lateral_tracking_reserve_m < 0.0)
  {
    return CanonicalExecutionPlanRejectReason::InvalidLateralTrackingReserve;
  }
  for (std::size_t index = 0U; index < plan.predicted_states.size(); ++index) {
    const auto & state = plan.predicted_states[index];
    if (!finite_state(state)) {
      return CanonicalExecutionPlanRejectReason::InvalidPredictedState;
    }
    const double lower_m = plan.lateral_lower_m[index];
    const double upper_m = plan.lateral_upper_m[index];
    const double required_reserve_m =
      index == 0U ? 0.0 : plan.required_lateral_tracking_reserve_m;
    if (
      !std::isfinite(lower_m) || !std::isfinite(upper_m) ||
      upper_m < lower_m || upper_m - required_reserve_m <
      lower_m + required_reserve_m ||
      state.lateral_m <
      lower_m + required_reserve_m - 1e-5 ||
      state.lateral_m >
      upper_m - required_reserve_m + 1e-5)
    {
      return required_reserve_m > 0.0 ?
        CanonicalExecutionPlanRejectReason::InvalidLateralTrackingReserve :
        CanonicalExecutionPlanRejectReason::InvalidLateralCorridor;
    }
  }
  for (const auto & control : plan.control_stages) {
    if (!finite_control(control)) {
      return CanonicalExecutionPlanRejectReason::InvalidControlStage;
    }
  }
  return CanonicalExecutionPlanRejectReason::None;
}

const char * to_string(const CanonicalExecutionCursorReason reason) noexcept
{
  switch (reason) {
    case CanonicalExecutionCursorReason::Available: return "available";
    case CanonicalExecutionCursorReason::InvalidPlan: return "invalid-plan";
    case CanonicalExecutionCursorReason::InvalidTime: return "invalid-time";
    case CanonicalExecutionCursorReason::FuturePlan: return "future-plan";
    case CanonicalExecutionCursorReason::CertificateExpired:
      return "certificate-expired";
    case CanonicalExecutionCursorReason::Exhausted: return "exhausted";
  }
  return "unknown";
}

CanonicalExecutionCursor resolve_execution_cursor(
  const CanonicalExecutionPlan & plan, const double now_sec) noexcept
{
  CanonicalExecutionCursor cursor;
  cursor.plan_id = plan.plan_id;
  if (
    validate_canonical_execution_plan(plan) !=
    CanonicalExecutionPlanRejectReason::None)
  {
    return cursor;
  }
  if (!std::isfinite(now_sec) || now_sec < 0.0) {
    cursor.reason = CanonicalExecutionCursorReason::InvalidTime;
    return cursor;
  }
  if (now_sec < plan.solved_sec) {
    cursor.reason = CanonicalExecutionCursorReason::FuturePlan;
    return cursor;
  }
  if (now_sec > plan.solution.valid_until_sec) {
    cursor.reason = CanonicalExecutionCursorReason::CertificateExpired;
    return cursor;
  }

  double elapsed_sec = now_sec - plan.solved_sec;
  for (std::size_t index = 0U; index < plan.control_stages.size(); ++index) {
    const double duration_sec = plan.control_stages[index].duration_sec;
    if (elapsed_sec < duration_sec) {
      cursor.available = true;
      cursor.reason = CanonicalExecutionCursorReason::Available;
      cursor.first_control_stage_index = index;
      cursor.remaining_control_stage_count =
        plan.control_stages.size() - index;
      cursor.stage_elapsed_sec = elapsed_sec;
      return cursor;
    }
    elapsed_sec -= duration_sec;
  }
  cursor.reason = CanonicalExecutionCursorReason::Exhausted;
  return cursor;
}

const char * to_string(const CanonicalActuationReason reason) noexcept
{
  switch (reason) {
    case CanonicalActuationReason::Available: return "available";
    case CanonicalActuationReason::InvalidPlan: return "invalid-plan";
    case CanonicalActuationReason::CursorUnavailable: return "cursor-unavailable";
    case CanonicalActuationReason::PlanIdentityMismatch:
      return "plan-identity-mismatch";
    case CanonicalActuationReason::InvalidStageIndex:
      return "invalid-stage-index";
    case CanonicalActuationReason::InvalidWheelbase: return "invalid-wheelbase";
    case CanonicalActuationReason::NonfiniteActuation:
      return "nonfinite-actuation";
  }
  return "unknown";
}

CanonicalActuationResult extract_canonical_actuation(
  const CanonicalExecutionPlan & plan,
  const CanonicalExecutionCursor & cursor,
  const double wheelbase_m) noexcept
{
  CanonicalActuationResult result;
  if (
    validate_canonical_execution_plan(plan) !=
    CanonicalExecutionPlanRejectReason::None)
  {
    return result;
  }
  if (!cursor.available) {
    result.reason = CanonicalActuationReason::CursorUnavailable;
    return result;
  }
  if (cursor.plan_id != plan.plan_id) {
    result.reason = CanonicalActuationReason::PlanIdentityMismatch;
    return result;
  }
  const std::size_t stage = cursor.first_control_stage_index;
  if (
    stage >= plan.control_stages.size() ||
    stage + 1U >= plan.predicted_states.size() ||
    cursor.remaining_control_stage_count != plan.control_stages.size() - stage)
  {
    result.reason = CanonicalActuationReason::InvalidStageIndex;
    return result;
  }
  if (!std::isfinite(wheelbase_m) || wheelbase_m <= 0.0) {
    result.reason = CanonicalActuationReason::InvalidWheelbase;
    return result;
  }

  const auto & control = plan.control_stages[stage];
  const auto & next_state = plan.predicted_states[stage + 1U];
  CanonicalActuation actuation;
  actuation.plan_id = plan.plan_id;
  actuation.control_stage_index = stage;
  actuation.predicted_speed_mps = next_state.velocity_mps;
  actuation.acceleration_mps2 = control.acceleration_mps2;
  actuation.curvature_radpm = control.curvature_radpm;
  actuation.steering_tire_angle_rad = std::atan(
    wheelbase_m * control.curvature_radpm);
  actuation.virtual_progress_speed_mps = control.virtual_progress_speed_mps;
  if (
    !std::isfinite(actuation.predicted_speed_mps) ||
    !std::isfinite(actuation.acceleration_mps2) ||
    !std::isfinite(actuation.curvature_radpm) ||
    !std::isfinite(actuation.steering_tire_angle_rad) ||
    !std::isfinite(actuation.virtual_progress_speed_mps))
  {
    result.reason = CanonicalActuationReason::NonfiniteActuation;
    return result;
  }
  result.reason = CanonicalActuationReason::Available;
  result.actuation = actuation;
  return result;
}

const char * to_string(const CanonicalExecutionPlanStoreReason reason) noexcept
{
  switch (reason) {
    case CanonicalExecutionPlanStoreReason::Accepted: return "accepted";
    case CanonicalExecutionPlanStoreReason::InvalidPlan: return "invalid-plan";
    case CanonicalExecutionPlanStoreReason::StalePlanId: return "stale-plan-id";
  }
  return "unknown";
}

CanonicalExecutionPlanStoreReason CanonicalExecutionPlanStore::replace(
  CanonicalExecutionPlan plan)
{
  if (
    validate_canonical_execution_plan(plan) !=
    CanonicalExecutionPlanRejectReason::None)
  {
    return CanonicalExecutionPlanStoreReason::InvalidPlan;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (plan.plan_id <= latest_accepted_plan_id_) {
    return CanonicalExecutionPlanStoreReason::StalePlanId;
  }
  plan_ = std::make_shared<const CanonicalExecutionPlan>(std::move(plan));
  latest_accepted_plan_id_ = plan_->plan_id;
  return CanonicalExecutionPlanStoreReason::Accepted;
}

std::shared_ptr<const CanonicalExecutionPlan>
CanonicalExecutionPlanStore::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return plan_;
}

bool CanonicalExecutionPlanStore::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  const bool had_plan = static_cast<bool>(plan_);
  plan_.reset();
  return had_plan;
}

bool CanonicalExecutionPlanStore::clear_if_plan_id(
  const std::uint64_t expected_plan_id)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!plan_ || plan_->plan_id != expected_plan_id) {
    return false;
  }
  plan_.reset();
  return true;
}

const char * to_string(const CanonicalCandidateBuildReason reason) noexcept
{
  switch (reason) {
    case CanonicalCandidateBuildReason::Accepted: return "accepted";
    case CanonicalCandidateBuildReason::InvalidPlan: return "invalid-plan";
    case CanonicalCandidateBuildReason::CursorUnavailable:
      return "cursor-unavailable";
    case CanonicalCandidateBuildReason::PlanIdentityMismatch:
      return "plan-identity-mismatch";
    case CanonicalCandidateBuildReason::ExecutionWindowMismatch:
      return "execution-window-mismatch";
    case CanonicalCandidateBuildReason::MissingDecisionIdentity:
      return "missing-decision-identity";
    case CanonicalCandidateBuildReason::DecisionIdentityMismatch:
      return "decision-identity-mismatch";
    case CanonicalCandidateBuildReason::PhysicalCertificateRejected:
      return "physical-certificate-rejected";
  }
  return "unknown";
}

CanonicalCandidateBuildResult build_canonical_normal_candidate(
  const CanonicalExecutionPlan & plan,
  const CanonicalExecutionCursor & cursor,
  const CanonicalExecutionRevalidation & revalidation)
{
  CanonicalCandidateBuildResult result;
  if (
    validate_canonical_execution_plan(plan) !=
    CanonicalExecutionPlanRejectReason::None)
  {
    return result;
  }
  if (!cursor.available) {
    result.reason = CanonicalCandidateBuildReason::CursorUnavailable;
    return result;
  }
  if (cursor.plan_id != plan.plan_id || revalidation.plan_id != plan.plan_id) {
    result.reason = CanonicalCandidateBuildReason::PlanIdentityMismatch;
    return result;
  }
  if (
    revalidation.first_control_stage_index !=
    cursor.first_control_stage_index ||
    revalidation.remaining_control_stage_count !=
    cursor.remaining_control_stage_count)
  {
    result.reason = CanonicalCandidateBuildReason::ExecutionWindowMismatch;
    return result;
  }
  if (revalidation.decision_id == 0U) {
    result.reason = CanonicalCandidateBuildReason::MissingDecisionIdentity;
    return result;
  }
  // This legacy-shaped proof is intentionally fresh-only.  A retained plan
  // belongs to an earlier solver problem and must pass the typed current-
  // observation proof in canonical_retained_revalidation instead.
  if (revalidation.decision_id != plan.problem.decision_id) {
    result.reason = CanonicalCandidateBuildReason::DecisionIdentityMismatch;
    return result;
  }
  if (!physical_certificate_accepted(revalidation.physical)) {
    result.reason = CanonicalCandidateBuildReason::PhysicalCertificateRejected;
    return result;
  }

  contract::CanonicalNormalCandidate candidate;
  candidate.problem = plan.problem;
  candidate.solution = plan.solution;
  candidate.executable_control_stage_count =
    cursor.remaining_control_stage_count;
  candidate.execution_plan_id = plan.plan_id;
  candidate.execution_certificate_decision_id = revalidation.decision_id;
  candidate.execution_first_control_stage_index =
    cursor.first_control_stage_index;
  candidate.execution_physical = revalidation.physical;
  result.reason = CanonicalCandidateBuildReason::Accepted;
  result.candidate = std::move(candidate);
  return result;
}

}  // namespace multi_purpose_mpc_ros::canonical_execution_plan
