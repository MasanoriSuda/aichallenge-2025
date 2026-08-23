#include "multi_purpose_mpc_ros/follow_canonical_async.hpp"

#include "multi_purpose_mpc_ros/latest_only_worker.hpp"

#include <cmath>
#include <utility>

namespace multi_purpose_mpc_ros::follow_canonical_async
{

const char * to_string(const ResultValidationReason reason) noexcept
{
  switch (reason) {
    case ResultValidationReason::Accepted:
      return "accepted";
    case ResultValidationReason::InvalidIdentity:
      return "invalid-identity";
    case ResultValidationReason::InvalidTiming:
      return "invalid-timing";
    case ResultValidationReason::InvalidPlanPayload:
      return "invalid-plan-payload";
    case ResultValidationReason::PlanIdentityMismatch:
      return "plan-identity-mismatch";
  }
  return "unknown";
}

ResultValidationReason validate_worker_result(
  const WorkerResult & result) noexcept
{
  const auto & identity = result.identity;
  if (
    identity.sequence == 0U || identity.context_epoch == 0U ||
    identity.snapshot_decision_id == 0U || identity.intent_generation == 0U ||
    identity.target_observation_generation == 0U ||
    identity.problem_fingerprint == 0U || identity.target_id.empty())
  {
    return ResultValidationReason::InvalidIdentity;
  }
  if (
    !std::isfinite(identity.snapshot_sec) || identity.snapshot_sec < 0.0 ||
    !std::isfinite(result.completed_sec) ||
    result.completed_sec < identity.snapshot_sec ||
    !std::isfinite(result.compute_ms) || result.compute_ms < 0.0)
  {
    return ResultValidationReason::InvalidTiming;
  }
  if (result.outcome != WorkerOutcome::PlanAvailable) {
    return result.canonical_plan == nullptr ?
      ResultValidationReason::Accepted :
      ResultValidationReason::InvalidPlanPayload;
  }
  if (result.canonical_plan == nullptr) {
    return ResultValidationReason::InvalidPlanPayload;
  }
  const auto & canonical_plan = *result.canonical_plan;
  if (
    plan::validate_canonical_execution_plan(canonical_plan) !=
    plan::CanonicalExecutionPlanRejectReason::None)
  {
    return ResultValidationReason::InvalidPlanPayload;
  }
  const auto & problem = canonical_plan.problem;
  if (
    problem.decision_id != identity.snapshot_decision_id ||
    problem.intent != mpcc_execution_contract::ControlIntent::Follow ||
    problem.intent_generation != identity.intent_generation ||
    problem.target_obstacle_generation !=
    identity.target_observation_generation ||
    problem.target_id != identity.target_id ||
    problem.fingerprint != identity.problem_fingerprint ||
    canonical_plan.solution.problem_fingerprint != identity.problem_fingerprint)
  {
    return ResultValidationReason::PlanIdentityMismatch;
  }
  return ResultValidationReason::Accepted;
}

const char * to_string(const CurrentIdentityReason reason) noexcept
{
  switch (reason) {
    case CurrentIdentityReason::Accepted:
      return "accepted";
    case CurrentIdentityReason::InvalidCurrentContext:
      return "invalid-current-context";
    case CurrentIdentityReason::ContextEpochMismatch:
      return "context-epoch-mismatch";
    case CurrentIdentityReason::IntentMismatch:
      return "intent-mismatch";
    case CurrentIdentityReason::IntentGenerationMismatch:
      return "intent-generation-mismatch";
    case CurrentIdentityReason::TargetMismatch:
      return "target-mismatch";
    case CurrentIdentityReason::TargetObservationRollback:
      return "target-observation-rollback";
  }
  return "unknown";
}

CurrentIdentityReason validate_current_identity(
  const ResultIdentity & result,
  const std::uint64_t active_context_epoch,
  const mpcc_execution_contract::MpccProblemContext & current) noexcept
{
  if (!mpcc_execution_contract::problem_context_complete(current)) {
    return CurrentIdentityReason::InvalidCurrentContext;
  }
  if (result.context_epoch != active_context_epoch) {
    return CurrentIdentityReason::ContextEpochMismatch;
  }
  if (current.intent != mpcc_execution_contract::ControlIntent::Follow) {
    return CurrentIdentityReason::IntentMismatch;
  }
  if (current.intent_generation != result.intent_generation) {
    return CurrentIdentityReason::IntentGenerationMismatch;
  }
  if (current.target_id != result.target_id) {
    return CurrentIdentityReason::TargetMismatch;
  }
  if (
    current.target_obstacle_generation <
    result.target_observation_generation)
  {
    return CurrentIdentityReason::TargetObservationRollback;
  }
  return CurrentIdentityReason::Accepted;
}

const char * to_string(const PublishReason reason) noexcept
{
  switch (reason) {
    case PublishReason::Accepted:
      return "accepted";
    case PublishReason::InvalidResult:
      return "invalid-result";
    case PublishReason::ContextMismatch:
      return "context-mismatch";
    case PublishReason::SequenceRollback:
      return "sequence-rollback";
    case PublishReason::SequenceNotSubmitted:
      return "sequence-not-submitted";
  }
  return "unknown";
}

void Mailbox::reset_context(const std::uint64_t context_epoch)
{
  std::lock_guard<std::mutex> lock(mutex_);
  context_epoch_ = context_epoch;
  latest_submitted_sequence_ = 0U;
  latest_published_sequence_ = 0U;
  latest_result_.reset();
}

bool Mailbox::register_submission(
  const std::uint64_t context_epoch, const std::uint64_t sequence)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (
    context_epoch == 0U || context_epoch != context_epoch_ || sequence == 0U ||
    sequence <= latest_submitted_sequence_)
  {
    return false;
  }
  latest_submitted_sequence_ = sequence;
  return true;
}

PublishReason Mailbox::publish(WorkerResult result)
{
  if (validate_worker_result(result) != ResultValidationReason::Accepted) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++invalid_result_count_;
    last_publish_reason_ = PublishReason::InvalidResult;
    return PublishReason::InvalidResult;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  const auto & identity = result.identity;
  if (identity.context_epoch != context_epoch_) {
    ++context_mismatch_count_;
    last_publish_reason_ = PublishReason::ContextMismatch;
    return PublishReason::ContextMismatch;
  }
  if (identity.sequence <= latest_published_sequence_) {
    ++sequence_rollback_count_;
    last_publish_reason_ = PublishReason::SequenceRollback;
    return PublishReason::SequenceRollback;
  }
  if (identity.sequence > latest_submitted_sequence_) {
    ++sequence_not_submitted_count_;
    last_publish_reason_ = PublishReason::SequenceNotSubmitted;
    return PublishReason::SequenceNotSubmitted;
  }
  if (!should_publish_latest_only_result(
      LatestOnlyResultPublicationRequest{
        identity.context_epoch,
        context_epoch_,
        identity.sequence,
        latest_submitted_sequence_,
        latest_published_sequence_}))
  {
    ++invalid_result_count_;
    last_publish_reason_ = PublishReason::InvalidResult;
    return PublishReason::InvalidResult;
  }
  ++accepted_count_;
  last_publish_reason_ = PublishReason::Accepted;
  latest_published_sequence_ = identity.sequence;
  latest_result_ = std::move(result);
  return PublishReason::Accepted;
}

std::optional<WorkerResult> Mailbox::latest_after(
  const std::uint64_t consumed_sequence) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (
    !latest_result_.has_value() ||
    latest_result_->identity.sequence <= consumed_sequence)
  {
    return std::nullopt;
  }
  return latest_result_;
}

MailboxState Mailbox::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return MailboxState{
    context_epoch_,
    latest_submitted_sequence_,
    latest_published_sequence_,
    accepted_count_,
    invalid_result_count_,
    context_mismatch_count_,
    sequence_rollback_count_,
    sequence_not_submitted_count_,
    last_publish_reason_,
    latest_result_.has_value()};
}

}  // namespace multi_purpose_mpc_ros::follow_canonical_async
