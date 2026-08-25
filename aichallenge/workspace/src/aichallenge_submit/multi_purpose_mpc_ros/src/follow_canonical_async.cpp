#include "multi_purpose_mpc_ros/canonical_normal_async.hpp"

#include "multi_purpose_mpc_ros/latest_only_worker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace multi_purpose_mpc_ros::canonical_normal_async
{

namespace
{

std::uint64_t next_context_epoch(const std::uint64_t current) noexcept
{
  return current == std::numeric_limits<std::uint64_t>::max() ?
    1U : std::max<std::uint64_t>(1U, current + 1U);
}

}  // namespace

const char * to_string(const ContextLifecycleReason reason) noexcept
{
  switch (reason) {
    case ContextLifecycleReason::Accepted:
      return "accepted";
    case ContextLifecycleReason::InvalidContext:
      return "invalid-context";
    case ContextLifecycleReason::UnsupportedIntent:
      return "unsupported-intent";
  }
  return "unknown";
}

ContextLifecycleResolution resolve_context_lifecycle(
  const ContextLifecycleState & previous,
  const mpcc_execution_contract::MpccProblemContext & context) noexcept
{
  ContextLifecycleResolution result;
  result.next = previous;
  if (!mpcc_execution_contract::canonical_normal_intent_supported(context.intent)) {
    result.reason = ContextLifecycleReason::UnsupportedIntent;
    return result;
  }
  if (!mpcc_execution_contract::problem_context_complete(context)) {
    result.reason = ContextLifecycleReason::InvalidContext;
    return result;
  }
  const bool identity_changed = previous.initialized &&
    (previous.intent != context.intent ||
    previous.intent_generation != context.intent_generation ||
    previous.target_id != context.target_id ||
    previous.execution_side_sign != context.execution_side_sign);
  if (identity_changed) {
    result.next.context_epoch = next_context_epoch(previous.context_epoch);
    result.reset_context = true;
  } else if (result.next.context_epoch == 0U) {
    result.next.context_epoch = 1U;
  }
  result.next.initialized = true;
  result.next.intent = context.intent;
  result.next.intent_generation = context.intent_generation;
  result.next.target_id = context.target_id;
  result.next.execution_side_sign = context.execution_side_sign;
  result.valid = true;
  result.reason = ContextLifecycleReason::Accepted;
  return result;
}

ContextLifecycleState invalidate_context_lifecycle(
  const ContextLifecycleState & previous) noexcept
{
  if (!previous.initialized) {
    return previous;
  }
  ContextLifecycleState result;
  result.context_epoch = next_context_epoch(previous.context_epoch);
  return result;
}

const char * to_string(const SnapshotContextReason reason) noexcept
{
  switch (reason) {
    case SnapshotContextReason::Accepted:
      return "accepted";
    case SnapshotContextReason::InvalidContext:
      return "invalid-context";
    case SnapshotContextReason::IntentMismatch:
      return "intent-mismatch";
    case SnapshotContextReason::DecisionMismatch:
      return "decision-mismatch";
    case SnapshotContextReason::IntentGenerationMismatch:
      return "intent-generation-mismatch";
    case SnapshotContextReason::TargetMismatch:
      return "target-mismatch";
    case SnapshotContextReason::TargetObservationMismatch:
      return "target-observation-mismatch";
    case SnapshotContextReason::ProblemFingerprintMismatch:
      return "problem-fingerprint-mismatch";
  }
  return "unknown";
}

SnapshotContextReason validate_snapshot_context(
  const ResultIdentity & identity,
  const mpcc_execution_contract::MpccProblemContext & snapshot) noexcept
{
  if (!mpcc_execution_contract::problem_context_complete(snapshot)) {
    return SnapshotContextReason::InvalidContext;
  }
  if (
    !mpcc_execution_contract::canonical_normal_intent_supported(identity.intent) ||
    snapshot.intent != identity.intent)
  {
    return SnapshotContextReason::IntentMismatch;
  }
  if (snapshot.decision_id != identity.snapshot_decision_id) {
    return SnapshotContextReason::DecisionMismatch;
  }
  if (snapshot.intent_generation != identity.intent_generation) {
    return SnapshotContextReason::IntentGenerationMismatch;
  }
  if (snapshot.target_id != identity.target_id) {
    return SnapshotContextReason::TargetMismatch;
  }
  if (
    snapshot.target_obstacle_generation !=
    identity.target_observation_generation)
  {
    return SnapshotContextReason::TargetObservationMismatch;
  }
  if (snapshot.fingerprint != identity.problem_fingerprint) {
    return SnapshotContextReason::ProblemFingerprintMismatch;
  }
  return SnapshotContextReason::Accepted;
}

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
  const bool supported_intent =
    mpcc_execution_contract::canonical_normal_intent_supported(identity.intent);
  const bool target_required = supported_intent &&
    mpcc_execution_contract::canonical_normal_intent_requires_target(
    identity.intent);
  if (
    identity.sequence == 0U || identity.context_epoch == 0U ||
    identity.snapshot_decision_id == 0U ||
    !supported_intent || identity.problem_fingerprint == 0U ||
    (target_required &&
    (identity.target_observation_generation == 0U || identity.target_id.empty())))
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
    problem.intent != identity.intent ||
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
  if (current.intent != result.intent) {
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

const char * to_string(const OvertakePreentryPlanReason reason) noexcept
{
  switch (reason) {
    case OvertakePreentryPlanReason::Accepted:
      return "accepted";
    case OvertakePreentryPlanReason::MissingPlan:
      return "missing-plan";
    case OvertakePreentryPlanReason::InvalidExpectedIdentity:
      return "invalid-expected-identity";
    case OvertakePreentryPlanReason::InvalidPlan:
      return "invalid-plan";
    case OvertakePreentryPlanReason::UnsupportedIntent:
      return "unsupported-intent";
    case OvertakePreentryPlanReason::IntentMismatch:
      return "intent-mismatch";
    case OvertakePreentryPlanReason::IntentGenerationMismatch:
      return "intent-generation-mismatch";
    case OvertakePreentryPlanReason::TargetMismatch:
      return "target-mismatch";
    case OvertakePreentryPlanReason::TargetObservationRollback:
      return "target-observation-rollback";
    case OvertakePreentryPlanReason::ExecutionSideMismatch:
      return "execution-side-mismatch";
    case OvertakePreentryPlanReason::CursorUnavailable:
      return "cursor-unavailable";
  }
  return "unknown";
}

OvertakePreentryPlanResolution resolve_overtake_preentry_plan(
  const OvertakePreentryPlanRequest & request) noexcept
{
  OvertakePreentryPlanResolution result;
  if (request.plan == nullptr) {
    return result;
  }
  const bool expected_intent_supported =
    request.expected_intent ==
    mpcc_execution_contract::ControlIntent::ShiftOut ||
    request.expected_intent == mpcc_execution_contract::ControlIntent::Pass;
  if (
    !expected_intent_supported || request.expected_intent_generation == 0U ||
    request.current_target_observation_generation == 0U ||
    request.expected_target_id.empty() ||
    (request.expected_execution_side_sign != -1 &&
    request.expected_execution_side_sign != 1) ||
    !std::isfinite(request.now_sec) || request.now_sec < 0.0)
  {
    result.reason = OvertakePreentryPlanReason::InvalidExpectedIdentity;
    return result;
  }
  if (
    plan::validate_canonical_execution_plan(*request.plan) !=
    plan::CanonicalExecutionPlanRejectReason::None)
  {
    result.reason = OvertakePreentryPlanReason::InvalidPlan;
    return result;
  }
  const auto & problem = request.plan->problem;
  if (
    problem.intent != mpcc_execution_contract::ControlIntent::ShiftOut &&
    problem.intent != mpcc_execution_contract::ControlIntent::Pass)
  {
    result.reason = OvertakePreentryPlanReason::UnsupportedIntent;
    return result;
  }
  if (problem.intent != request.expected_intent) {
    result.reason = OvertakePreentryPlanReason::IntentMismatch;
    return result;
  }
  if (problem.intent_generation != request.expected_intent_generation) {
    result.reason = OvertakePreentryPlanReason::IntentGenerationMismatch;
    return result;
  }
  if (problem.target_id != request.expected_target_id) {
    result.reason = OvertakePreentryPlanReason::TargetMismatch;
    return result;
  }
  if (
    problem.target_obstacle_generation == 0U ||
    problem.target_obstacle_generation >
    request.current_target_observation_generation)
  {
    result.reason = OvertakePreentryPlanReason::TargetObservationRollback;
    return result;
  }
  if (problem.execution_side_sign != request.expected_execution_side_sign) {
    result.reason = OvertakePreentryPlanReason::ExecutionSideMismatch;
    return result;
  }
  const auto cursor = plan::resolve_execution_cursor(
    *request.plan, request.now_sec);
  result.cursor_reason = cursor.reason;
  if (!cursor.available) {
    result.reason = OvertakePreentryPlanReason::CursorUnavailable;
    return result;
  }
  result.admitted = true;
  result.reason = OvertakePreentryPlanReason::Accepted;
  return result;
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
  const auto validation_reason = validate_worker_result(result);
  if (validation_reason != ResultValidationReason::Accepted) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++invalid_result_count_;
    last_validation_reason_ = validation_reason;
    last_publish_reason_ = PublishReason::InvalidResult;
    return PublishReason::InvalidResult;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  last_validation_reason_ = ResultValidationReason::Accepted;
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
    last_validation_reason_,
    last_publish_reason_,
    latest_result_.has_value()};
}

}  // namespace multi_purpose_mpc_ros::canonical_normal_async
