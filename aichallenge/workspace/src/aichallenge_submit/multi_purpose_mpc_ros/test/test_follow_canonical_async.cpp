#include "multi_purpose_mpc_ros/canonical_normal_async.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <utility>

namespace
{

namespace async = multi_purpose_mpc_ros::canonical_normal_async;
namespace follow_compat = multi_purpose_mpc_ros::follow_canonical_async;
namespace contract = multi_purpose_mpc_ros::mpcc_execution_contract;
namespace plan = multi_purpose_mpc_ros::canonical_execution_plan;

plan::CanonicalExecutionPlan make_follow_plan(
  const std::uint64_t decision_id = 42U,
  const std::uint64_t intent_generation = 3U,
  const std::uint64_t target_generation = 7U,
  const contract::ControlIntent intent = contract::ControlIntent::Follow)
{
  contract::MpccProblemContext problem;
  problem.decision_id = decision_id;
  problem.intent = intent;
  problem.intent_generation = intent_generation;
  problem.observation_generation = target_generation;
  problem.stage_geometry_id = 11U;
  problem.target_obstacle_generation = target_generation;
  problem.target_id = "d2";
  problem.execution_side_sign =
    contract::canonical_normal_intent_requires_execution_side(intent) ? 1 : 0;
  problem.horizon_steps = 2U;
  problem.formulation = contract::Formulation::VelocityProgress5State;
  problem.state_schema_id = "ey-elag-epsi-v-progress-v1";
  problem.input_schema_id = "accel-curvature-progress-rate-v1";
  problem.bounds_schema_id = "progress-stage-wall-follow-target-v1";
  problem.cost_schema_id = "velocity-progress-follow-gap-v1";
  problem = contract::seal_problem_context(std::move(problem));

  contract::CertifiedMpccSolution solution;
  solution.solution_id = 9U;
  solution.problem_fingerprint = problem.fingerprint;
  solution.formulation = contract::Formulation::VelocityProgress5State;
  solution.solved = true;
  solution.finite = true;
  solution.constraints_satisfied = true;
  solution.maximum_constraint_violation = 0.0;
  solution.physical.checked = true;
  solution.physical.wall_clear = true;
  solution.physical.obstacles_clear = true;
  solution.prediction_stage_count = 2U;
  solution.valid_until_sec = 12.5;

  plan::CanonicalExecutionPlan value;
  value.plan_id = 23U;
  value.problem = problem;
  value.solution = solution;
  value.solved_sec = 10.0;
  value.predicted_states = {
    plan::CanonicalPredictedState{0.10, 0.01, 0.02, 5.0, 100.0},
    plan::CanonicalPredictedState{0.12, 0.02, 0.03, 5.2, 100.5},
    plan::CanonicalPredictedState{0.14, 0.03, 0.04, 5.4, 101.0}};
  value.control_stages = {
    plan::CanonicalControlStage{1.0, 0.02, 5.1, 1.0},
    plan::CanonicalControlStage{0.5, 0.03, 5.3, 1.0}};
  value.lateral_lower_m = {-0.5, -0.5, -0.5};
  value.lateral_upper_m = {0.5, 0.5, 0.5};
  return value;
}

async::WorkerResult make_result(
  const std::uint64_t sequence = 10U,
  const std::uint64_t context_epoch = 7U,
  const contract::ControlIntent intent = contract::ControlIntent::Follow)
{
  auto canonical_plan = std::make_shared<const plan::CanonicalExecutionPlan>(
    make_follow_plan(42U, 3U, 7U, intent));
  async::WorkerResult result;
  result.identity.sequence = sequence;
  result.identity.context_epoch = context_epoch;
  result.identity.snapshot_decision_id = canonical_plan->problem.decision_id;
  result.identity.intent = canonical_plan->problem.intent;
  result.identity.intent_generation = canonical_plan->problem.intent_generation;
  result.identity.target_observation_generation =
    canonical_plan->problem.target_obstacle_generation;
  result.identity.problem_fingerprint = canonical_plan->problem.fingerprint;
  result.identity.target_id = canonical_plan->problem.target_id;
  result.identity.snapshot_sec = 10.0;
  result.outcome = async::WorkerOutcome::PlanAvailable;
  result.completed_sec = 10.02;
  result.compute_ms = 20.0;
  result.canonical_plan = std::move(canonical_plan);
  return result;
}

TEST(CanonicalNormalAsyncSnapshotContext, CarriesExactShiftOutIntent)
{
  const auto result = make_result(
    10U, 7U, contract::ControlIntent::ShiftOut);
  EXPECT_EQ(
    async::validate_snapshot_context(
      result.identity, result.canonical_plan->problem),
    async::SnapshotContextReason::Accepted);
}

TEST(CanonicalNormalAsyncSnapshotContext, RejectsCrossIntentRederivation)
{
  const auto result = make_result(
    10U, 7U, contract::ControlIntent::ShiftOut);
  auto changed = result.canonical_plan->problem;
  changed.intent = contract::ControlIntent::Pass;
  changed = contract::seal_problem_context(std::move(changed));
  EXPECT_EQ(
    async::validate_snapshot_context(result.identity, changed),
    async::SnapshotContextReason::IntentMismatch);
}

TEST(CanonicalNormalAsyncResult, RejectsUnsupportedStopIntent)
{
  const auto result = make_result(
    10U, 7U, contract::ControlIntent::Stop);
  EXPECT_EQ(
    async::validate_worker_result(result),
    async::ResultValidationReason::InvalidIdentity);
}

TEST(CanonicalNormalAsyncResult, AcceptsEverySupportedExactIntent)
{
  for (const auto intent : {
      contract::ControlIntent::Track,
      contract::ControlIntent::Cruise,
      contract::ControlIntent::Follow,
      contract::ControlIntent::ShiftOut,
      contract::ControlIntent::Pass,
      contract::ControlIntent::Return})
  {
    const auto result = make_result(10U, 7U, intent);
    EXPECT_EQ(
      async::validate_worker_result(result),
      async::ResultValidationReason::Accepted)
      << contract::to_string(intent);
  }
}

TEST(FollowCanonicalAsyncCompatibility, OldNamespaceNamesCanonicalTransport)
{
  follow_compat::ResultIdentity identity;
  identity.intent = contract::ControlIntent::Follow;
  EXPECT_EQ(identity.intent, contract::ControlIntent::Follow);
}

TEST(CanonicalNormalAsyncContextLifecycle, ExactPhaseChangeResetsEpoch)
{
  async::ContextLifecycleState state;
  const auto shift_out = async::resolve_context_lifecycle(
    state, make_follow_plan(
      42U, 3U, 7U, contract::ControlIntent::ShiftOut).problem);
  ASSERT_TRUE(shift_out.valid);
  EXPECT_FALSE(shift_out.reset_context);
  ASSERT_TRUE(shift_out.next.initialized);
  EXPECT_EQ(shift_out.next.context_epoch, 1U);

  const auto pass = async::resolve_context_lifecycle(
    shift_out.next, make_follow_plan(
      43U, 3U, 8U, contract::ControlIntent::Pass).problem);
  ASSERT_TRUE(pass.valid);
  EXPECT_TRUE(pass.reset_context);
  EXPECT_EQ(pass.next.context_epoch, 2U);
  EXPECT_EQ(pass.next.intent, contract::ControlIntent::Pass);
}

TEST(CanonicalNormalAsyncContextLifecycle, NewObservationKeepsSameEpoch)
{
  const auto initialized = async::resolve_context_lifecycle(
    async::ContextLifecycleState{}, make_follow_plan(
      42U, 3U, 7U, contract::ControlIntent::ShiftOut).problem);
  ASSERT_TRUE(initialized.valid);
  const auto newer_observation = async::resolve_context_lifecycle(
    initialized.next, make_follow_plan(
      43U, 3U, 8U, contract::ControlIntent::ShiftOut).problem);
  ASSERT_TRUE(newer_observation.valid);
  EXPECT_FALSE(newer_observation.reset_context);
  EXPECT_EQ(
    newer_observation.next.context_epoch,
    initialized.next.context_epoch);
}

TEST(CanonicalNormalAsyncContextLifecycle, SamePhaseSideChangeResetsEpoch)
{
  const auto initialized = async::resolve_context_lifecycle(
    async::ContextLifecycleState{}, make_follow_plan(
      42U, 3U, 7U, contract::ControlIntent::ShiftOut).problem);
  ASSERT_TRUE(initialized.valid);

  auto opposite = make_follow_plan(
    43U, 3U, 8U, contract::ControlIntent::ShiftOut).problem;
  opposite.execution_side_sign = -1;
  opposite = contract::seal_problem_context(std::move(opposite));
  const auto changed = async::resolve_context_lifecycle(
    initialized.next, opposite);
  ASSERT_TRUE(changed.valid);
  EXPECT_TRUE(changed.reset_context);
  EXPECT_EQ(changed.next.context_epoch, initialized.next.context_epoch + 1U);
  EXPECT_EQ(changed.next.execution_side_sign, -1);
}

TEST(CanonicalNormalAsyncContextLifecycle, StopFailsClosed)
{
  const auto resolution = async::resolve_context_lifecycle(
    async::ContextLifecycleState{}, make_follow_plan(
      42U, 3U, 7U, contract::ControlIntent::Stop).problem);
  EXPECT_FALSE(resolution.valid);
  EXPECT_EQ(
    resolution.reason,
    async::ContextLifecycleReason::UnsupportedIntent);
  EXPECT_FALSE(resolution.next.initialized);
}

TEST(CanonicalNormalAsyncContextLifecycle, ExplicitInvalidationAdvancesOnce)
{
  const auto initialized = async::resolve_context_lifecycle(
    async::ContextLifecycleState{}, make_follow_plan().problem);
  ASSERT_TRUE(initialized.valid);
  const auto invalidated =
    async::invalidate_context_lifecycle(initialized.next);
  EXPECT_FALSE(invalidated.initialized);
  EXPECT_EQ(invalidated.context_epoch, 2U);
  const auto repeated = async::invalidate_context_lifecycle(invalidated);
  EXPECT_EQ(repeated.context_epoch, invalidated.context_epoch);
}

}  // namespace

TEST(FollowCanonicalAsyncResult, AcceptsInternallyConsistentImmutablePlan)
{
  EXPECT_EQ(
    async::validate_worker_result(make_result()),
    async::ResultValidationReason::Accepted);
}

TEST(FollowCanonicalAsyncResult, AcceptsFollowWithoutOvertakeMissionGeneration)
{
  auto canonical_plan = make_follow_plan(42U, 0U, 7U);
  auto result = make_result();
  result.identity.intent_generation = 0U;
  result.identity.problem_fingerprint = canonical_plan.problem.fingerprint;
  result.canonical_plan =
    std::make_shared<const plan::CanonicalExecutionPlan>(
    std::move(canonical_plan));
  EXPECT_TRUE(contract::problem_context_complete(result.canonical_plan->problem));
  EXPECT_EQ(
    async::validate_worker_result(result),
    async::ResultValidationReason::Accepted);
}

TEST(FollowCanonicalAsyncSnapshotContext, AcceptsExactSealedFollowContext)
{
  const auto result = make_result();
  EXPECT_EQ(
    async::validate_snapshot_context(
      result.identity, result.canonical_plan->problem),
    async::SnapshotContextReason::Accepted);
}

TEST(FollowCanonicalAsyncSnapshotContext, RejectsWorkerIntentRederivation)
{
  const auto result = make_result();
  auto rederived = result.canonical_plan->problem;
  rederived.intent = contract::ControlIntent::Cruise;
  rederived = contract::seal_problem_context(std::move(rederived));
  EXPECT_EQ(
    async::validate_snapshot_context(result.identity, rederived),
    async::SnapshotContextReason::IntentMismatch);
}

TEST(FollowCanonicalAsyncSnapshotContext, RejectsDifferentSealedProblem)
{
  const auto result = make_result();
  auto different = result.canonical_plan->problem;
  different.stage_geometry_id += 1U;
  different = contract::seal_problem_context(std::move(different));
  EXPECT_EQ(
    async::validate_snapshot_context(result.identity, different),
    async::SnapshotContextReason::ProblemFingerprintMismatch);
}

TEST(FollowCanonicalAsyncResult, RejectsPlanIdentityMismatch)
{
  auto result = make_result();
  result.identity.target_id = "d3";
  EXPECT_EQ(
    async::validate_worker_result(result),
    async::ResultValidationReason::PlanIdentityMismatch);
}

TEST(FollowCanonicalAsyncResult, RejectsFailureCarryingExecutablePlan)
{
  auto result = make_result();
  result.outcome = async::WorkerOutcome::Rejected;
  EXPECT_EQ(
    async::validate_worker_result(result),
    async::ResultValidationReason::InvalidPlanPayload);
}

TEST(FollowCanonicalAsyncCurrentIdentity, AcceptsNewerObservationForLiveProof)
{
  const auto result = make_result();
  auto current = result.canonical_plan->problem;
  current.decision_id = 43U;
  current.observation_generation = 8U;
  current.target_obstacle_generation = 8U;
  current.stage_geometry_id = 12U;
  current = contract::seal_problem_context(std::move(current));
  EXPECT_EQ(
    async::validate_current_identity(result.identity, 7U, current),
    async::CurrentIdentityReason::Accepted);
}

TEST(FollowCanonicalAsyncCurrentIdentity, RejectsChangedIntentAndTarget)
{
  const auto result = make_result();
  auto changed_intent = result.canonical_plan->problem;
  changed_intent.intent_generation = 4U;
  changed_intent = contract::seal_problem_context(std::move(changed_intent));
  EXPECT_EQ(
    async::validate_current_identity(result.identity, 7U, changed_intent),
    async::CurrentIdentityReason::IntentGenerationMismatch);

  auto changed_target = result.canonical_plan->problem;
  changed_target.target_id = "d3";
  changed_target = contract::seal_problem_context(std::move(changed_target));
  EXPECT_EQ(
    async::validate_current_identity(result.identity, 7U, changed_target),
    async::CurrentIdentityReason::TargetMismatch);
}

TEST(FollowCanonicalAsyncCurrentIdentity, RejectsOldEpochAndObservationRollback)
{
  const auto result = make_result();
  EXPECT_EQ(
    async::validate_current_identity(
      result.identity, 8U, result.canonical_plan->problem),
    async::CurrentIdentityReason::ContextEpochMismatch);

  auto rolled_back = result.canonical_plan->problem;
  rolled_back.target_obstacle_generation = 6U;
  rolled_back = contract::seal_problem_context(std::move(rolled_back));
  EXPECT_EQ(
    async::validate_current_identity(result.identity, 7U, rolled_back),
    async::CurrentIdentityReason::TargetObservationRollback);
}

TEST(FollowCanonicalAsyncMailbox, PublishesCompletedResultWithNewerJobQueued)
{
  async::Mailbox mailbox;
  mailbox.reset_context(7U);
  ASSERT_TRUE(mailbox.register_submission(7U, 10U));
  ASSERT_TRUE(mailbox.register_submission(7U, 11U));
  EXPECT_EQ(mailbox.publish(make_result(10U)), async::PublishReason::Accepted);
  const auto result = mailbox.latest_after(9U);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->identity.sequence, 10U);
}

TEST(FollowCanonicalAsyncMailbox, RejectsOldContextWithoutReplacingLatest)
{
  async::Mailbox mailbox;
  mailbox.reset_context(7U);
  ASSERT_TRUE(mailbox.register_submission(7U, 10U));
  ASSERT_EQ(mailbox.publish(make_result(10U)), async::PublishReason::Accepted);

  mailbox.reset_context(8U);
  ASSERT_TRUE(mailbox.register_submission(8U, 11U));
  EXPECT_EQ(
    mailbox.publish(make_result(11U, 7U)),
    async::PublishReason::ContextMismatch);
  EXPECT_FALSE(mailbox.latest_after(0U).has_value());
}

TEST(FollowCanonicalAsyncMailbox, RejectsRollbackAndUnsubmittedSequence)
{
  async::Mailbox mailbox;
  mailbox.reset_context(7U);
  ASSERT_TRUE(mailbox.register_submission(7U, 10U));
  ASSERT_EQ(mailbox.publish(make_result(10U)), async::PublishReason::Accepted);
  EXPECT_EQ(
    mailbox.publish(make_result(9U)),
    async::PublishReason::SequenceRollback);
  EXPECT_EQ(
    mailbox.publish(make_result(11U)),
    async::PublishReason::SequenceNotSubmitted);
  const auto retained = mailbox.latest_after(0U);
  ASSERT_TRUE(retained.has_value());
  EXPECT_EQ(retained->identity.sequence, 10U);
  const auto state = mailbox.state();
  EXPECT_EQ(state.accepted_count, 1U);
  EXPECT_EQ(state.sequence_rollback_count, 1U);
  EXPECT_EQ(state.sequence_not_submitted_count, 1U);
  EXPECT_EQ(state.last_publish_reason, async::PublishReason::SequenceNotSubmitted);
}

TEST(FollowCanonicalAsyncMailbox, AcceptsTypedFailureWithoutPlan)
{
  async::Mailbox mailbox;
  mailbox.reset_context(7U);
  ASSERT_TRUE(mailbox.register_submission(7U, 10U));
  auto result = make_result();
  result.outcome = async::WorkerOutcome::Rejected;
  result.canonical_plan.reset();
  result.detail = "solve rejected";
  EXPECT_EQ(mailbox.publish(std::move(result)), async::PublishReason::Accepted);
  const auto published = mailbox.latest_after(0U);
  ASSERT_TRUE(published.has_value());
  EXPECT_EQ(published->outcome, async::WorkerOutcome::Rejected);
  EXPECT_EQ(published->detail, "solve rejected");
}

TEST(FollowCanonicalAsyncMailbox, ExposesExactPayloadValidationFailure)
{
  async::Mailbox mailbox;
  mailbox.reset_context(7U);
  ASSERT_TRUE(mailbox.register_submission(7U, 10U));
  auto result = make_result();
  result.identity.target_id = "d3";
  EXPECT_EQ(
    mailbox.publish(std::move(result)), async::PublishReason::InvalidResult);
  const auto state = mailbox.state();
  EXPECT_EQ(
    state.last_validation_reason,
    async::ResultValidationReason::PlanIdentityMismatch);
  EXPECT_FALSE(state.result_available);
}
