#include "multi_purpose_mpc_ros/canonical_execution_plan_adapter.hpp"

#include "multi_purpose_mpc_ros/mpcc_progress.hpp"

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

}  // namespace multi_purpose_mpc_ros::canonical_execution_plan_adapter
