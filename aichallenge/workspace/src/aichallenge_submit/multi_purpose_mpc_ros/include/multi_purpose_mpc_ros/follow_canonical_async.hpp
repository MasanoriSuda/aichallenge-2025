#ifndef MULTI_PURPOSE_MPC_ROS__FOLLOW_CANONICAL_ASYNC_HPP_
#define MULTI_PURPOSE_MPC_ROS__FOLLOW_CANONICAL_ASYNC_HPP_

#include "multi_purpose_mpc_ros/canonical_execution_plan.hpp"

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

namespace multi_purpose_mpc_ros::canonical_normal_async
{

namespace plan = canonical_execution_plan;

struct ContextLifecycleState
{
  std::uint64_t context_epoch{1U};
  bool initialized{false};
  mpcc_execution_contract::ControlIntent intent{
    mpcc_execution_contract::ControlIntent::Unknown};
  std::uint64_t intent_generation{};
  std::string target_id;
  int execution_side_sign{};
};

enum class ContextLifecycleReason
{
  Accepted,
  InvalidContext,
  UnsupportedIntent,
};

const char * to_string(ContextLifecycleReason reason) noexcept;

struct ContextLifecycleResolution
{
  bool valid{false};
  bool reset_context{false};
  ContextLifecycleReason reason{ContextLifecycleReason::InvalidContext};
  ContextLifecycleState next;
};

/// Resolve the mailbox lifetime from semantic identity only. Observation and
/// decision generations belong to individual immutable jobs and deliberately
/// do not roll the epoch while the same intent/target context remains active.
ContextLifecycleResolution resolve_context_lifecycle(
  const ContextLifecycleState & previous,
  const mpcc_execution_contract::MpccProblemContext & context) noexcept;

/// Leaving canonical normal execution invalidates an initialized context once.
/// Repeated invalidation is idempotent so unrelated supervisor cycles do not
/// churn epochs.
ContextLifecycleState invalidate_context_lifecycle(
  const ContextLifecycleState & previous) noexcept;

struct ResultIdentity
{
  std::uint64_t sequence{};
  std::uint64_t context_epoch{};
  std::uint64_t snapshot_decision_id{};
  mpcc_execution_contract::ControlIntent intent{
    mpcc_execution_contract::ControlIntent::Unknown};
  std::uint64_t intent_generation{};
  std::uint64_t target_observation_generation{};
  std::uint64_t problem_fingerprint{};
  std::string target_id;
  double snapshot_sec{};
};

enum class SnapshotContextReason
{
  Accepted,
  InvalidContext,
  IntentMismatch,
  DecisionMismatch,
  IntentGenerationMismatch,
  TargetMismatch,
  TargetObservationMismatch,
  ProblemFingerprintMismatch,
};

const char * to_string(SnapshotContextReason reason) noexcept;

/// A canonical worker must solve the immutable context sealed by the live
/// controller when the job was submitted. Re-deriving intent or authority in
/// the worker would allow one normal problem to be interpreted as another.
SnapshotContextReason validate_snapshot_context(
  const ResultIdentity & identity,
  const mpcc_execution_contract::MpccProblemContext & snapshot) noexcept;

enum class WorkerOutcome
{
  PlanAvailable,
  Rejected,
  Exception,
};

struct WorkerResult
{
  ResultIdentity identity;
  WorkerOutcome outcome{WorkerOutcome::Rejected};
  double completed_sec{};
  double compute_ms{};
  std::string detail;
  std::shared_ptr<const plan::CanonicalExecutionPlan> canonical_plan;
};

enum class ResultValidationReason
{
  Accepted,
  InvalidIdentity,
  InvalidTiming,
  InvalidPlanPayload,
  PlanIdentityMismatch,
};

const char * to_string(ResultValidationReason reason) noexcept;
ResultValidationReason validate_worker_result(
  const WorkerResult & result) noexcept;

enum class CurrentIdentityReason
{
  Accepted,
  InvalidCurrentContext,
  ContextEpochMismatch,
  IntentMismatch,
  IntentGenerationMismatch,
  TargetMismatch,
  TargetObservationRollback,
};

const char * to_string(CurrentIdentityReason reason) noexcept;

/// The observation generation is intentionally allowed to advance: worker
/// plans are expected to complete after newer V2X data arrives. Physical use
/// still requires a separate current-world target-tube/wall proof.
CurrentIdentityReason validate_current_identity(
  const ResultIdentity & result,
  std::uint64_t active_context_epoch,
  const mpcc_execution_contract::MpccProblemContext & current) noexcept;

enum class OvertakePreentryPlanReason
{
  Accepted,
  MissingPlan,
  InvalidExpectedIdentity,
  InvalidPlan,
  UnsupportedIntent,
  IntentMismatch,
  IntentGenerationMismatch,
  TargetMismatch,
  TargetObservationRollback,
  ExecutionSideMismatch,
  CursorUnavailable,
  ActuationUnavailable,
  SteeringContinuityRejected,
};

const char * to_string(OvertakePreentryPlanReason reason) noexcept;

struct OvertakePreentryPlanRequest
{
  std::shared_ptr<const plan::CanonicalExecutionPlan> plan;
  mpcc_execution_contract::ControlIntent expected_intent{
    mpcc_execution_contract::ControlIntent::Unknown};
  std::uint64_t expected_intent_generation{};
  std::uint64_t current_target_observation_generation{};
  std::string expected_target_id;
  int expected_execution_side_sign{};
  double now_sec{};
  double current_steering_tire_angle_rad{};
  double wheelbase_m{};
  double maximum_steering_step_rad{};
};

struct OvertakePreentryPlanResolution
{
  bool admitted{false};
  OvertakePreentryPlanReason reason{
    OvertakePreentryPlanReason::MissingPlan};
  plan::CanonicalExecutionCursorReason cursor_reason{
    plan::CanonicalExecutionCursorReason::InvalidPlan};
  plan::CanonicalActuationReason actuation_reason{
    plan::CanonicalActuationReason::InvalidPlan};
  plan::CanonicalSteeringContinuityResult steering_continuity;
};

/// Admit the exact immutable plan produced by the already-solved pre-entry
/// left/right branch.  The live control cycle must repeat both semantic/lifetime
/// identity and first-actuation reachability before raising Overtake authority;
/// current-world wall and obstacle proof remains a separate mandatory gate.
OvertakePreentryPlanResolution resolve_overtake_preentry_plan(
  const OvertakePreentryPlanRequest & request) noexcept;

enum class PublishReason
{
  Accepted,
  InvalidResult,
  ContextMismatch,
  SequenceRollback,
  SequenceNotSubmitted,
};

const char * to_string(PublishReason reason) noexcept;

struct MailboxState
{
  std::uint64_t context_epoch{};
  std::uint64_t latest_submitted_sequence{};
  std::uint64_t latest_published_sequence{};
  std::uint64_t accepted_count{};
  std::uint64_t invalid_result_count{};
  std::uint64_t context_mismatch_count{};
  std::uint64_t sequence_rollback_count{};
  std::uint64_t sequence_not_submitted_count{};
  ResultValidationReason last_validation_reason{
    ResultValidationReason::InvalidIdentity};
  PublishReason last_publish_reason{PublishReason::InvalidResult};
  bool result_available{false};
};

/// Typed latest-only boundary for canonical normal worker results. Publishing
/// never mutates the canonical execution-plan store; the live controller must
/// separately check current intent/target identity and current-world physical
/// proof before storing or executing a plan.
class Mailbox
{
public:
  void reset_context(std::uint64_t context_epoch);
  bool register_submission(
    std::uint64_t context_epoch, std::uint64_t sequence);
  PublishReason publish(WorkerResult result);
  std::optional<WorkerResult> latest_after(
    std::uint64_t consumed_sequence) const;
  MailboxState state() const;

private:
  mutable std::mutex mutex_;
  std::uint64_t context_epoch_{};
  std::uint64_t latest_submitted_sequence_{};
  std::uint64_t latest_published_sequence_{};
  std::uint64_t accepted_count_{};
  std::uint64_t invalid_result_count_{};
  std::uint64_t context_mismatch_count_{};
  std::uint64_t sequence_rollback_count_{};
  std::uint64_t sequence_not_submitted_count_{};
  ResultValidationReason last_validation_reason_{
    ResultValidationReason::InvalidIdentity};
  PublishReason last_publish_reason_{PublishReason::InvalidResult};
  std::optional<WorkerResult> latest_result_;
};

}  // namespace multi_purpose_mpc_ros::canonical_normal_async

namespace multi_purpose_mpc_ros
{

// Source compatibility for the accepted Follow producer during migration.
// New producers must use canonical_normal_async directly.
namespace follow_canonical_async = canonical_normal_async;

}  // namespace multi_purpose_mpc_ros

#endif  // MULTI_PURPOSE_MPC_ROS__FOLLOW_CANONICAL_ASYNC_HPP_
