#include "multi_purpose_mpc_ros/canonical_execution_plan_adapter.hpp"

#include "multi_purpose_mpc_ros/mpcc_progress.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace multi_purpose_mpc_ros::canonical_execution_plan_adapter
{

const char * to_string(const CanonicalPlanExtractionReason reason) noexcept
{
  switch (reason) {
    case CanonicalPlanExtractionReason::Accepted: return "accepted";
    case CanonicalPlanExtractionReason::InvalidMetadata: return "invalid-metadata";
    case CanonicalPlanExtractionReason::InvalidProgressOrigin:
      return "invalid-progress-origin";
    case CanonicalPlanExtractionReason::StageDurationCountMismatch:
      return "stage-duration-count-mismatch";
    case CanonicalPlanExtractionReason::PrimalSizeMismatch:
      return "primal-size-mismatch";
    case CanonicalPlanExtractionReason::NonfinitePrimal: return "nonfinite-primal";
    case CanonicalPlanExtractionReason::PlanContractRejected:
      return "plan-contract-rejected";
  }
  return "unknown";
}

CanonicalPlanExtractionResult extract_canonical_execution_plan(
  const CanonicalPlanExtractionRequest & request)
{
  CanonicalPlanExtractionResult result;
  if (
    request.plan_id == 0U ||
    !contract::problem_context_complete(request.problem) ||
    request.problem.horizon_steps == 0U ||
    request.problem.formulation != contract::Formulation::VelocityProgress5State ||
    request.solution.formulation != contract::Formulation::VelocityProgress5State ||
    !std::isfinite(request.solved_sec) || request.solved_sec < 0.0)
  {
    return result;
  }
  if (!std::isfinite(request.progress_origin_m)) {
    result.reason = CanonicalPlanExtractionReason::InvalidProgressOrigin;
    return result;
  }

  const std::size_t horizon = request.problem.horizon_steps;
  if (request.stage_duration_sec.size() != horizon) {
    result.reason = CanonicalPlanExtractionReason::StageDurationCountMismatch;
    return result;
  }
  constexpr std::size_t state_dimension =
    static_cast<std::size_t>(mpcc_progress::kExtendedStateDimension);
  constexpr std::size_t input_dimension =
    static_cast<std::size_t>(mpcc_progress::kExtendedInputDimension);
  const std::size_t maximum_index = static_cast<std::size_t>(
    std::numeric_limits<Eigen::Index>::max());
  if (
    horizon > (maximum_index - state_dimension) /
    (state_dimension + input_dimension))
  {
    return result;
  }
  const std::size_t state_value_count = state_dimension * (horizon + 1U);
  const std::size_t expected_value_count =
    state_value_count + input_dimension * horizon;
  if (
    request.extended_primal.size() !=
    static_cast<Eigen::Index>(expected_value_count))
  {
    result.reason = CanonicalPlanExtractionReason::PrimalSizeMismatch;
    return result;
  }
  if (!request.extended_primal.allFinite()) {
    result.reason = CanonicalPlanExtractionReason::NonfinitePrimal;
    return result;
  }

  plan_contract::CanonicalExecutionPlan plan;
  plan.plan_id = request.plan_id;
  plan.problem = request.problem;
  plan.solution = request.solution;
  plan.solved_sec = request.solved_sec;
  plan.predicted_states.reserve(horizon + 1U);
  plan.control_stages.reserve(horizon);
  for (std::size_t stage = 0U; stage <= horizon; ++stage) {
    const Eigen::Index offset = static_cast<Eigen::Index>(stage * state_dimension);
    plan.predicted_states.push_back(plan_contract::CanonicalPredictedState{
      request.extended_primal[
        offset + mpcc_progress::kExtendedLateralIndex],
      request.extended_primal[offset + mpcc_progress::kExtendedLagIndex],
      request.extended_primal[offset + mpcc_progress::kExtendedHeadingIndex],
      request.extended_primal[offset + mpcc_progress::kExtendedVelocityIndex],
      request.extended_primal[offset + mpcc_progress::kExtendedProgressIndex] +
      request.progress_origin_m});
  }
  for (std::size_t stage = 0U; stage < horizon; ++stage) {
    const Eigen::Index offset = static_cast<Eigen::Index>(
      state_value_count + stage * input_dimension);
    plan.control_stages.push_back(plan_contract::CanonicalControlStage{
      request.extended_primal[
        offset + mpcc_progress::kExtendedAccelerationIndex],
      request.extended_primal[offset + mpcc_progress::kExtendedCurvatureIndex],
      request.extended_primal[
        offset + mpcc_progress::kExtendedVirtualProgressSpeedIndex],
      request.stage_duration_sec[stage]});
  }

  result.plan_reject_reason =
    plan_contract::validate_canonical_execution_plan(plan);
  if (
    result.plan_reject_reason !=
    plan_contract::CanonicalExecutionPlanRejectReason::None)
  {
    result.reason = CanonicalPlanExtractionReason::PlanContractRejected;
    return result;
  }
  result.reason = CanonicalPlanExtractionReason::Accepted;
  result.plan = std::move(plan);
  return result;
}

const char * to_string(const FreshCanonicalCommandReason reason) noexcept
{
  switch (reason) {
    case FreshCanonicalCommandReason::Accepted: return "accepted";
    case FreshCanonicalCommandReason::PlanRejected: return "plan-rejected";
    case FreshCanonicalCommandReason::CursorRejected: return "cursor-rejected";
    case FreshCanonicalCommandReason::CandidateRejected: return "candidate-rejected";
    case FreshCanonicalCommandReason::AuthorityRejected: return "authority-rejected";
    case FreshCanonicalCommandReason::ActuationRejected: return "actuation-rejected";
    case FreshCanonicalCommandReason::ActuationMismatch: return "actuation-mismatch";
    case FreshCanonicalCommandReason::CommandRejected: return "command-rejected";
  }
  return "unknown";
}

FreshCanonicalCommandResult build_fresh_canonical_command(
  const FreshCanonicalCommandRequest & request)
{
  FreshCanonicalCommandResult result;
  if (
    request.current_decision_id == 0U || !std::isfinite(request.now_sec) ||
    request.now_sec < 0.0 || !std::isfinite(request.wheelbase_m) ||
    request.wheelbase_m <= 0.0 ||
    !std::isfinite(request.actuation_tolerance) ||
    request.actuation_tolerance < 0.0)
  {
    return result;
  }

  auto extraction = extract_canonical_execution_plan(request.extraction);
  result.extraction_reason = extraction.reason;
  result.plan_reject_reason = extraction.plan_reject_reason;
  if (!extraction.plan.has_value()) {
    return result;
  }
  auto execution_plan = std::move(extraction.plan.value());
  result.plan_extracted = true;
  result.plan_id = execution_plan.plan_id;
  result.cursor = plan_contract::resolve_execution_cursor(
    execution_plan, request.now_sec);
  result.cursor_reason = result.cursor.reason;
  if (!result.cursor.available) {
    result.reason = FreshCanonicalCommandReason::CursorRejected;
    return result;
  }
  result.cursor_available = true;

  const plan_contract::CanonicalExecutionRevalidation revalidation{
    request.current_decision_id,
    execution_plan.plan_id,
    result.cursor.first_control_stage_index,
    result.cursor.remaining_control_stage_count,
    request.extraction.solution.physical};
  const auto candidate = plan_contract::build_canonical_normal_candidate(
    execution_plan, result.cursor, revalidation);
  result.candidate_reason = candidate.reason;
  if (!candidate.candidate.has_value()) {
    result.reason = FreshCanonicalCommandReason::CandidateRejected;
    return result;
  }
  result.candidate_accepted = true;

  const auto authority = contract::resolve_canonical_normal_authority(
    contract::CanonicalNormalAuthorityRequest{
      request.current_decision_id, request.now_sec,
      candidate.candidate.value(), {}, request.current_intent});
  result.authority_source = authority.source;
  result.authority_reason = authority.reason;
  result.fresh_reject_reason = authority.fresh_reject_reason;
  if (authority.source != contract::CanonicalNormalAuthoritySource::FreshCertified) {
    result.reason = FreshCanonicalCommandReason::AuthorityRejected;
    return result;
  }
  result.authority_ready = true;

  const auto stored = plan_contract::extract_canonical_actuation(
    execution_plan, result.cursor, request.wheelbase_m);
  result.actuation_reason = stored.reason;
  if (!stored.actuation.has_value()) {
    result.reason = FreshCanonicalCommandReason::ActuationRejected;
    return result;
  }
  result.actuation_extracted = true;
  const auto & actuation = stored.actuation.value();
  result.maximum_actuation_difference = std::max({
    std::abs(
      actuation.predicted_speed_mps - request.direct_actuation.predicted_speed_mps),
    std::abs(
      actuation.acceleration_mps2 - request.direct_actuation.acceleration_mps2),
    std::abs(
      actuation.curvature_radpm - request.direct_actuation.curvature_radpm),
    std::abs(
      actuation.steering_tire_angle_rad -
      request.direct_actuation.steering_tire_angle_rad),
    std::abs(
      actuation.virtual_progress_speed_mps -
      request.direct_actuation.virtual_progress_speed_mps)});
  if (
    !std::isfinite(result.maximum_actuation_difference) ||
    result.maximum_actuation_difference > request.actuation_tolerance)
  {
    result.reason = FreshCanonicalCommandReason::ActuationMismatch;
    return result;
  }

  const auto command = contract::build_canonical_normal_command(
    authority,
    contract::CanonicalActuation{
      actuation.predicted_speed_mps,
      actuation.acceleration_mps2,
      actuation.curvature_radpm,
      actuation.steering_tire_angle_rad,
      actuation.virtual_progress_speed_mps});
  result.command_reason = command.reason;
  if (!command.command.has_value()) {
    result.reason = FreshCanonicalCommandReason::CommandRejected;
    return result;
  }
  result.plan = std::move(execution_plan);
  result.command = command.command;
  result.reason = FreshCanonicalCommandReason::Accepted;
  return result;
}

}  // namespace multi_purpose_mpc_ros::canonical_execution_plan_adapter
