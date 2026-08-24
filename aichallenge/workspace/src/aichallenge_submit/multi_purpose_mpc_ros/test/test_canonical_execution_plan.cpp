#include "multi_purpose_mpc_ros/canonical_execution_plan.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>

namespace
{

namespace contract =
  multi_purpose_mpc_ros::mpcc_execution_contract;
namespace plan = multi_purpose_mpc_ros::canonical_execution_plan;

contract::MpccProblemContext make_problem(const std::uint64_t decision_id = 42U)
{
  contract::MpccProblemContext problem;
  problem.decision_id = decision_id;
  problem.intent = contract::ControlIntent::Track;
  problem.intent_generation = 3U;
  problem.observation_generation = 7U;
  problem.stage_geometry_id = 11U;
  problem.horizon_steps = 2U;
  problem.formulation = contract::Formulation::VelocityProgress5State;
  problem.state_schema_id = "ey-elag-epsi-v-progress-v1";
  problem.input_schema_id = "accel-curvature-progress-rate-v1";
  problem.bounds_schema_id = "progress-stage-wall-obstacle-v1";
  problem.cost_schema_id = "velocity-progress-v1";
  return contract::seal_problem_context(std::move(problem));
}

contract::CertifiedMpccSolution make_solution(
  const contract::MpccProblemContext & problem)
{
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
  return solution;
}

plan::CanonicalExecutionPlan make_plan(
  const std::uint64_t plan_id = 23U,
  const std::uint64_t decision_id = 42U)
{
  const auto problem = make_problem(decision_id);
  plan::CanonicalExecutionPlan value;
  value.plan_id = plan_id;
  value.problem = problem;
  value.solution = make_solution(problem);
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

plan::CanonicalExecutionPlan make_follow_plan()
{
  auto value = make_plan();
  value.problem.intent = contract::ControlIntent::Follow;
  value.problem.target_id = "d2";
  value.problem.target_obstacle_generation =
    value.problem.observation_generation;
  value.problem.bounds_schema_id = "progress-stage-wall-follow-target-v1";
  value.problem.cost_schema_id = "velocity-progress-follow-gap-v1";
  value.problem = contract::seal_problem_context(std::move(value.problem));
  value.solution.problem_fingerprint = value.problem.fingerprint;
  return value;
}

plan::CanonicalExecutionPlan make_overtake_plan(
  const contract::ControlIntent intent)
{
  auto value = make_plan();
  value.problem.intent = intent;
  value.problem.execution_side_sign = 1;
  value.problem.target_id = "d2";
  value.problem.target_obstacle_generation =
    value.problem.observation_generation;
  value.problem = contract::seal_problem_context(std::move(value.problem));
  value.solution.problem_fingerprint = value.problem.fingerprint;
  return value;
}

contract::PhysicalCertificate make_current_physical_certificate()
{
  contract::PhysicalCertificate certificate;
  certificate.checked = true;
  certificate.wall_clear = true;
  certificate.obstacles_clear = true;
  certificate.minimum_wall_clearance_m = 0.25;
  certificate.minimum_obstacle_clearance_m = 0.20;
  return certificate;
}

}  // namespace

TEST(CanonicalExecutionPlan, AcceptsCompleteFiveStatePlan)
{
  EXPECT_EQ(
    plan::validate_canonical_execution_plan(make_plan()),
    plan::CanonicalExecutionPlanRejectReason::None);
}

TEST(CanonicalExecutionPlan, AcceptsFollowPlanWithTargetProvenance)
{
  const auto value = make_follow_plan();
  ASSERT_TRUE(contract::problem_context_complete(value.problem));
  EXPECT_EQ(
    plan::validate_canonical_execution_plan(value),
    plan::CanonicalExecutionPlanRejectReason::None);

  auto missing_target_provenance = value;
  missing_target_provenance.problem.target_obstacle_generation = 0U;
  missing_target_provenance.problem = contract::seal_problem_context(
    std::move(missing_target_provenance.problem));
  missing_target_provenance.solution.problem_fingerprint =
    missing_target_provenance.problem.fingerprint;
  EXPECT_EQ(
    plan::validate_canonical_execution_plan(missing_target_provenance),
    plan::CanonicalExecutionPlanRejectReason::IncompleteProblem);

  auto missing_target_id = value;
  missing_target_id.problem.target_id.clear();
  missing_target_id.problem.target_obstacle_generation = 0U;
  missing_target_id.problem = contract::seal_problem_context(
    std::move(missing_target_id.problem));
  missing_target_id.solution.problem_fingerprint =
    missing_target_id.problem.fingerprint;
  EXPECT_EQ(
    plan::validate_canonical_execution_plan(missing_target_id),
    plan::CanonicalExecutionPlanRejectReason::IncompleteProblem);
}

TEST(CanonicalExecutionPlan, AcceptsOvertakePlansOnlyWithTargetProvenance)
{
  for (const auto intent : {
      contract::ControlIntent::ShiftOut,
      contract::ControlIntent::Pass,
      contract::ControlIntent::Return})
  {
    const auto value = make_overtake_plan(intent);
    EXPECT_EQ(
      plan::validate_canonical_execution_plan(value),
      plan::CanonicalExecutionPlanRejectReason::None);

    auto missing_target = value;
    missing_target.problem.target_id.clear();
    missing_target.problem.target_obstacle_generation = 0U;
    missing_target.problem = contract::seal_problem_context(
      std::move(missing_target.problem));
    missing_target.solution.problem_fingerprint =
      missing_target.problem.fingerprint;
    EXPECT_EQ(
      plan::validate_canonical_execution_plan(missing_target),
      plan::CanonicalExecutionPlanRejectReason::IncompleteProblem);
  }
}

TEST(CanonicalExecutionPlan, RejectsOvertakeNominalStateOutsideTrackingTube)
{
  auto value = make_overtake_plan(contract::ControlIntent::ShiftOut);
  value.required_lateral_tracking_reserve_m = 0.15;
  // State zero is already observed and only needs to remain physically valid.
  value.predicted_states.front().lateral_m = -0.40;
  EXPECT_EQ(
    plan::validate_canonical_execution_plan(value),
    plan::CanonicalExecutionPlanRejectReason::None);

  value.predicted_states[1].lateral_m = -0.40;
  EXPECT_EQ(
    plan::validate_canonical_execution_plan(value),
    plan::CanonicalExecutionPlanRejectReason::InvalidLateralTrackingReserve);

  value.predicted_states[1].lateral_m = -0.35;
  EXPECT_EQ(
    plan::validate_canonical_execution_plan(value),
    plan::CanonicalExecutionPlanRejectReason::None);

  value.lateral_upper_m[1] = -0.25;
  EXPECT_EQ(
    plan::validate_canonical_execution_plan(value),
    plan::CanonicalExecutionPlanRejectReason::InvalidLateralTrackingReserve);
}

TEST(CanonicalExecutionPlan, RejectsPartialPredictionAndControlSequences)
{
  auto missing_state = make_plan();
  missing_state.predicted_states.pop_back();
  EXPECT_EQ(
    plan::validate_canonical_execution_plan(missing_state),
    plan::CanonicalExecutionPlanRejectReason::StateCountMismatch);

  auto missing_control = make_plan();
  missing_control.control_stages.pop_back();
  EXPECT_EQ(
    plan::validate_canonical_execution_plan(missing_control),
    plan::CanonicalExecutionPlanRejectReason::ControlCountMismatch);
}

TEST(CanonicalExecutionPlan, RejectsEmptyExecutableHorizon)
{
  auto value = make_plan();
  value.problem.horizon_steps = 0U;
  value.problem = contract::seal_problem_context(std::move(value.problem));
  value.solution.problem_fingerprint = value.problem.fingerprint;
  value.solution.prediction_stage_count = 0U;
  value.predicted_states.resize(1U);
  value.control_stages.clear();
  EXPECT_EQ(
    plan::validate_canonical_execution_plan(value),
    plan::CanonicalExecutionPlanRejectReason::EmptyHorizon);
}

TEST(CanonicalExecutionPlan, AdvancesExactCursorWithoutRepeatingFinalStage)
{
  const auto value = make_plan();
  const auto first = plan::resolve_execution_cursor(value, 10.25);
  ASSERT_TRUE(first.available);
  EXPECT_EQ(first.first_control_stage_index, 0U);
  EXPECT_EQ(first.remaining_control_stage_count, 2U);
  EXPECT_DOUBLE_EQ(first.stage_elapsed_sec, 0.25);

  const auto second = plan::resolve_execution_cursor(value, 11.0);
  ASSERT_TRUE(second.available);
  EXPECT_EQ(second.first_control_stage_index, 1U);
  EXPECT_EQ(second.remaining_control_stage_count, 1U);
  EXPECT_DOUBLE_EQ(second.stage_elapsed_sec, 0.0);

  const auto exhausted = plan::resolve_execution_cursor(value, 12.0);
  EXPECT_FALSE(exhausted.available);
  EXPECT_EQ(exhausted.reason, plan::CanonicalExecutionCursorReason::Exhausted);
}

TEST(CanonicalExecutionPlan, RejectsFutureAndExpiredExecutionTime)
{
  auto value = make_plan();
  const auto future = plan::resolve_execution_cursor(value, 9.9);
  EXPECT_FALSE(future.available);
  EXPECT_EQ(future.reason, plan::CanonicalExecutionCursorReason::FuturePlan);

  value.solution.valid_until_sec = 10.4;
  const auto expired = plan::resolve_execution_cursor(value, 10.5);
  EXPECT_FALSE(expired.available);
  EXPECT_EQ(
    expired.reason, plan::CanonicalExecutionCursorReason::CertificateExpired);
}

TEST(CanonicalExecutionPlan, ExtractsExactCurrentFiveStateActuation)
{
  const auto value = make_plan();
  const auto first_cursor = plan::resolve_execution_cursor(value, 10.25);
  const auto first = plan::extract_canonical_actuation(
    value, first_cursor, 1.0);
  ASSERT_EQ(first.reason, plan::CanonicalActuationReason::Available);
  ASSERT_TRUE(first.actuation.has_value());
  EXPECT_EQ(first.actuation->plan_id, 23U);
  EXPECT_EQ(first.actuation->control_stage_index, 0U);
  EXPECT_DOUBLE_EQ(first.actuation->predicted_speed_mps, 5.2);
  EXPECT_DOUBLE_EQ(first.actuation->acceleration_mps2, 1.0);
  EXPECT_DOUBLE_EQ(first.actuation->curvature_radpm, 0.02);
  EXPECT_DOUBLE_EQ(
    first.actuation->steering_tire_angle_rad, std::atan(0.02));
  EXPECT_DOUBLE_EQ(first.actuation->virtual_progress_speed_mps, 5.1);

  const auto second_cursor = plan::resolve_execution_cursor(value, 11.25);
  const auto second = plan::extract_canonical_actuation(
    value, second_cursor, 1.0);
  ASSERT_TRUE(second.actuation.has_value());
  EXPECT_EQ(second.actuation->control_stage_index, 1U);
  EXPECT_DOUBLE_EQ(second.actuation->predicted_speed_mps, 5.4);
  EXPECT_DOUBLE_EQ(second.actuation->acceleration_mps2, 0.5);
  EXPECT_DOUBLE_EQ(second.actuation->virtual_progress_speed_mps, 5.3);
}

TEST(CanonicalExecutionPlan, ActuationRejectsExhaustedOrMismatchedCursor)
{
  const auto value = make_plan();
  const auto exhausted = plan::resolve_execution_cursor(value, 12.0);
  EXPECT_EQ(
    plan::extract_canonical_actuation(value, exhausted, 1.0).reason,
    plan::CanonicalActuationReason::CursorUnavailable);

  auto mismatch = plan::resolve_execution_cursor(value, 10.25);
  mismatch.plan_id = 99U;
  EXPECT_EQ(
    plan::extract_canonical_actuation(value, mismatch, 1.0).reason,
    plan::CanonicalActuationReason::PlanIdentityMismatch);
  EXPECT_EQ(
    plan::extract_canonical_actuation(value, mismatch, 0.0).reason,
    plan::CanonicalActuationReason::PlanIdentityMismatch);
  const auto valid_cursor = plan::resolve_execution_cursor(value, 10.25);
  EXPECT_EQ(
    plan::extract_canonical_actuation(value, valid_cursor, 0.0).reason,
    plan::CanonicalActuationReason::InvalidWheelbase);
}

TEST(CanonicalExecutionPlan, CertifiesSteeringAgainstCurrentActuationHistory)
{
  const auto accepted = plan::certify_canonical_steering_continuity(
    plan::CanonicalSteeringContinuityRequest{-0.09187, -0.07437, 0.0175});
  EXPECT_TRUE(accepted.certified);
  EXPECT_EQ(
    accepted.reason, plan::CanonicalSteeringContinuityReason::Accepted);
  EXPECT_NEAR(accepted.reachable_lower_rad, -0.10937, 1e-12);
  EXPECT_NEAR(accepted.reachable_upper_rad, -0.07437, 1e-12);

  // Deterministic replay of the first discontinuity in
  // output/20260824-200419: the async plan was feasible from its old worker
  // snapshot, but not from the steering command active at publication.
  const auto stale_async = plan::certify_canonical_steering_continuity(
    plan::CanonicalSteeringContinuityRequest{-0.09187, 0.00061, 0.0175});
  EXPECT_FALSE(stale_async.certified);
  EXPECT_EQ(
    stale_async.reason, plan::CanonicalSteeringContinuityReason::Unreachable);
  EXPECT_NEAR(stale_async.steering_difference_rad, 0.09248, 1e-12);
}

TEST(CanonicalExecutionPlan, RejectsMalformedSteeringContinuityInput)
{
  const auto invalid = plan::certify_canonical_steering_continuity(
    plan::CanonicalSteeringContinuityRequest{
      0.0, 0.0, std::numeric_limits<double>::quiet_NaN()});
  EXPECT_FALSE(invalid.certified);
  EXPECT_EQ(
    invalid.reason, plan::CanonicalSteeringContinuityReason::InvalidInput);
}

TEST(CanonicalExecutionPlan, StoreReplacementIsCompleteAndMonotonic)
{
  plan::CanonicalExecutionPlanStore store;
  EXPECT_EQ(
    store.replace(make_plan(23U)),
    plan::CanonicalExecutionPlanStoreReason::Accepted);

  auto invalid = make_plan(24U);
  invalid.control_stages.clear();
  EXPECT_EQ(
    store.replace(std::move(invalid)),
    plan::CanonicalExecutionPlanStoreReason::InvalidPlan);
  ASSERT_TRUE(store.snapshot());
  EXPECT_EQ(store.snapshot()->plan_id, 23U);

  EXPECT_EQ(
    store.replace(make_plan(22U)),
    plan::CanonicalExecutionPlanStoreReason::StalePlanId);
  EXPECT_EQ(store.snapshot()->plan_id, 23U);
  EXPECT_EQ(
    store.replace(make_plan(24U)),
    plan::CanonicalExecutionPlanStoreReason::Accepted);
  EXPECT_EQ(store.snapshot()->plan_id, 24U);
  EXPECT_TRUE(store.clear());
  EXPECT_FALSE(store.snapshot());
  EXPECT_FALSE(store.clear());
  EXPECT_EQ(
    store.replace(make_plan(23U)),
    plan::CanonicalExecutionPlanStoreReason::StalePlanId);
  EXPECT_EQ(
    store.replace(make_plan(25U)),
    plan::CanonicalExecutionPlanStoreReason::Accepted);
  EXPECT_FALSE(store.clear_if_plan_id(24U));
  EXPECT_TRUE(store.clear_if_plan_id(25U));
  EXPECT_FALSE(store.snapshot());
  EXPECT_EQ(
    store.replace(make_plan(23U)),
    plan::CanonicalExecutionPlanStoreReason::StalePlanId);
  EXPECT_FALSE(store.snapshot());
}

TEST(CanonicalExecutionPlan, LegacyShapedProofCannotCreateRetainedCandidate)
{
  const auto value = make_plan(23U, 41U);
  const auto cursor = plan::resolve_execution_cursor(value, 11.0);
  ASSERT_TRUE(cursor.available);
  const plan::CanonicalExecutionRevalidation proof{
    42U, 23U, 1U, 1U, make_current_physical_certificate()};
  const auto built = plan::build_canonical_normal_candidate(value, cursor, proof);
  EXPECT_EQ(
    built.reason,
    plan::CanonicalCandidateBuildReason::DecisionIdentityMismatch);
  EXPECT_FALSE(built.candidate.has_value());
}

TEST(CanonicalExecutionPlan, FreshCandidateUsesSameDecisionAndExactExecutionWindow)
{
  const auto value = make_plan(23U, 42U);
  const auto cursor = plan::resolve_execution_cursor(value, 10.25);
  ASSERT_TRUE(cursor.available);
  const plan::CanonicalExecutionRevalidation proof{
    42U, 23U, 0U, 2U, make_current_physical_certificate()};
  const auto built = plan::build_canonical_normal_candidate(value, cursor, proof);
  ASSERT_EQ(built.reason, plan::CanonicalCandidateBuildReason::Accepted);
  ASSERT_TRUE(built.candidate.has_value());

  const auto resolution = contract::resolve_canonical_normal_authority(
    contract::CanonicalNormalAuthorityRequest{
    42U, 10.25, built.candidate.value(), {},
    contract::ControlIntent::Track});
  EXPECT_EQ(
    resolution.source,
    contract::CanonicalNormalAuthoritySource::FreshCertified);
  EXPECT_EQ(resolution.execution_plan_id, 23U);
  EXPECT_EQ(resolution.execution_first_control_stage_index, 0U);
  EXPECT_EQ(resolution.executable_control_stage_count, 2U);
}

TEST(CanonicalExecutionPlan, CandidateRejectsMismatchedWindowAndUnsafeProof)
{
  const auto value = make_plan();
  const auto cursor = plan::resolve_execution_cursor(value, 10.25);
  ASSERT_TRUE(cursor.available);

  auto mismatched = plan::CanonicalExecutionRevalidation{
    42U, 23U, 1U, 1U, make_current_physical_certificate()};
  EXPECT_EQ(
    plan::build_canonical_normal_candidate(value, cursor, mismatched).reason,
    plan::CanonicalCandidateBuildReason::ExecutionWindowMismatch);

  auto unsafe = plan::CanonicalExecutionRevalidation{
    42U, 23U, 0U, 2U, make_current_physical_certificate()};
  unsafe.physical.wall_clear = false;
  EXPECT_EQ(
    plan::build_canonical_normal_candidate(value, cursor, unsafe).reason,
    plan::CanonicalCandidateBuildReason::PhysicalCertificateRejected);
}
