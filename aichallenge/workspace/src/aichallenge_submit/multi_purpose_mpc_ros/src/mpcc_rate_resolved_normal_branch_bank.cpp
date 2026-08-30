#include "multi_purpose_mpc_ros/mpcc_rate_resolved_normal_branch_bank.hpp"

#include <cmath>
#include <utility>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_normal_branch_bank
{
namespace
{

namespace contract = mpcc_execution_contract;

bool normal_source_valid(const shadow::Snapshot & source) noexcept
{
  const auto & context = source.identity.source_context;
  return
    artifact::identity_valid(source.identity) &&
    (context.intent == contract::ControlIntent::Cruise ||
    context.intent == contract::ControlIntent::Follow) &&
    context.execution_side_sign == 0 &&
    context.dynamic_obstacle_constraint_active &&
    !context.dynamic_obstacle_id.empty() &&
    context.dynamic_obstacle_generation > 0U;
}

bool same_source_epoch(
  const artifact::Identity & source, const artifact::Identity & candidate,
  const int expected_side_sign) noexcept
{
  const auto & expected = source.source_context;
  const auto & actual = candidate.source_context;
  auto expected_candidate = expected;
  expected_candidate.dynamic_obstacle_side_sign = expected_side_sign;
  expected_candidate.fingerprint = 0U;
  expected_candidate = contract::seal_problem_context(
    std::move(expected_candidate));
  return
    (expected_side_sign == -1 || expected_side_sign == 1) &&
    candidate.sequence == source.sequence &&
    candidate.snapshot_sec == source.snapshot_sec &&
    actual.fingerprint == expected_candidate.fingerprint &&
    actual.decision_id == expected.decision_id &&
    actual.intent == expected.intent &&
    actual.intent_generation == expected.intent_generation &&
    actual.observation_generation == expected.observation_generation &&
    actual.stage_geometry_id == expected.stage_geometry_id &&
    actual.target_obstacle_generation == expected.target_obstacle_generation &&
    actual.target_id == expected.target_id &&
    actual.execution_side_sign == expected.execution_side_sign &&
    actual.dynamic_obstacle_constraint_active &&
    actual.dynamic_obstacle_generation ==
    expected.dynamic_obstacle_generation &&
    actual.dynamic_obstacle_id == expected.dynamic_obstacle_id &&
    actual.dynamic_obstacle_side_sign == expected_side_sign &&
    actual.horizon_steps == expected.horizon_steps &&
    actual.formulation == expected.formulation &&
    actual.state_schema_id == expected.state_schema_id &&
    actual.input_schema_id == expected.input_schema_id &&
    actual.bounds_schema_id == expected.bounds_schema_id &&
    actual.cost_schema_id == expected.cost_schema_id;
}

bool plan_valid_for_source(
  const shadow::Snapshot & source,
  const std::shared_ptr<const certified::CertifiedPlan> & plan,
  const int side_sign) noexcept
{
  return
    plan != nullptr &&
    certified::validate(*plan) == certified::RejectReason::None &&
    plan->execution_artifact != nullptr &&
    same_source_epoch(
      source.identity, plan->execution_artifact->identity, side_sign);
}

}  // namespace

const char * to_string(const ReplaceReason reason) noexcept
{
  switch (reason) {
    case ReplaceReason::Accepted: return "accepted";
    case ReplaceReason::InvalidSource: return "invalid-source";
    case ReplaceReason::InvalidSide: return "invalid-side";
    case ReplaceReason::InvalidNegativePlan: return "invalid-negative-plan";
    case ReplaceReason::InvalidPositivePlan: return "invalid-positive-plan";
    case ReplaceReason::StaleSource: return "stale-source";
  }
  return "unknown";
}

std::shared_ptr<const certified::CertifiedPlan> Snapshot::plan_for_side(
  const int side_sign) const noexcept
{
  if (side_sign < 0) {
    return negative_plan;
  }
  if (side_sign > 0) {
    return positive_plan;
  }
  return nullptr;
}

ReplaceReason Bank::replace(
  const shadow::Snapshot & source,
  std::shared_ptr<const certified::CertifiedPlan> negative_plan,
  std::shared_ptr<const certified::CertifiedPlan> positive_plan)
{
  if (!normal_source_valid(source)) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++invalid_source_count_;
    last_reason_ = ReplaceReason::InvalidSource;
    return last_reason_;
  }
  if (
    negative_plan != nullptr &&
    !plan_valid_for_source(source, negative_plan, -1))
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++invalid_plan_count_;
    last_reason_ = ReplaceReason::InvalidNegativePlan;
    return last_reason_;
  }
  if (
    positive_plan != nullptr &&
    !plan_valid_for_source(source, positive_plan, 1))
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++invalid_plan_count_;
    last_reason_ = ReplaceReason::InvalidPositivePlan;
    return last_reason_;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (source.identity.sequence <= latest_source_sequence_) {
    ++stale_source_count_;
    last_reason_ = ReplaceReason::StaleSource;
    return last_reason_;
  }
  snapshot_.source_identity = source.identity;
  snapshot_.negative_plan = std::move(negative_plan);
  snapshot_.positive_plan = std::move(positive_plan);
  latest_source_sequence_ = source.identity.sequence;
  ++accepted_count_;
  last_reason_ = ReplaceReason::Accepted;
  return last_reason_;
}

ReplaceReason Bank::merge_branch(
  const shadow::Snapshot & source, const int side_sign,
  std::shared_ptr<const certified::CertifiedPlan> plan)
{
  if (!normal_source_valid(source)) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++invalid_source_count_;
    last_reason_ = ReplaceReason::InvalidSource;
    return last_reason_;
  }
  if (side_sign != -1 && side_sign != 1) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++invalid_plan_count_;
    last_reason_ = ReplaceReason::InvalidSide;
    return last_reason_;
  }
  if (plan != nullptr && !plan_valid_for_source(source, plan, side_sign)) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++invalid_plan_count_;
    last_reason_ = side_sign < 0 ?
      ReplaceReason::InvalidNegativePlan :
      ReplaceReason::InvalidPositivePlan;
    return last_reason_;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (source.identity.sequence < latest_source_sequence_) {
    ++stale_source_count_;
    last_reason_ = ReplaceReason::StaleSource;
    return last_reason_;
  }
  if (source.identity.sequence == latest_source_sequence_) {
    if (!artifact::same_identity(snapshot_.source_identity, source.identity)) {
      ++invalid_source_count_;
      last_reason_ = ReplaceReason::InvalidSource;
      return last_reason_;
    }
  } else {
    snapshot_ = Snapshot{};
    snapshot_.source_identity = source.identity;
    latest_source_sequence_ = source.identity.sequence;
  }
  if (side_sign < 0) {
    snapshot_.negative_plan = std::move(plan);
  } else {
    snapshot_.positive_plan = std::move(plan);
  }
  ++accepted_count_;
  last_reason_ = ReplaceReason::Accepted;
  return last_reason_;
}

Snapshot Bank::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_;
}

State Bank::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  State result;
  result.latest_source_sequence = latest_source_sequence_;
  result.accepted_count = accepted_count_;
  result.invalid_source_count = invalid_source_count_;
  result.invalid_plan_count = invalid_plan_count_;
  result.stale_source_count = stale_source_count_;
  result.negative_available = snapshot_.negative_plan != nullptr;
  result.positive_available = snapshot_.positive_plan != nullptr;
  result.last_reason = last_reason_;
  return result;
}

void Bank::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_ = Snapshot{};
  latest_source_sequence_ = 0U;
  last_reason_ = ReplaceReason::InvalidSource;
}

}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_normal_branch_bank
