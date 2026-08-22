#include "multi_purpose_mpc_ros/canonical_execution_plan_adapter.hpp"

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

namespace
{

namespace adapter =
  multi_purpose_mpc_ros::canonical_execution_plan_adapter;
namespace contract =
  multi_purpose_mpc_ros::mpcc_execution_contract;
namespace plan = multi_purpose_mpc_ros::canonical_execution_plan;

contract::MpccProblemContext make_problem()
{
  contract::MpccProblemContext problem;
  problem.decision_id = 42U;
  problem.intent = contract::ControlIntent::Cruise;
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

Eigen::VectorXd make_primal()
{
  Eigen::VectorXd primal(21);
  primal <<
    0.10, 0.01, 0.02, 5.0, 0.0,
    0.20, 0.02, 0.03, 5.2, 0.5,
    0.30, 0.03, 0.04, 5.4, 1.0,
    1.00, 0.02, 5.1,
    0.50, 0.03, 5.3;
  return primal;
}

adapter::CanonicalPlanExtractionRequest make_request()
{
  const auto problem = make_problem();
  adapter::CanonicalPlanExtractionRequest request;
  request.plan_id = 23U;
  request.problem = problem;
  request.solution = make_solution(problem);
  request.solved_sec = 10.0;
  request.progress_origin_m = 100.0;
  request.stage_duration_sec = {0.4, 0.6};
  request.extended_primal = make_primal();
  return request;
}

}  // namespace

TEST(CanonicalExecutionPlanAdapter, ExtractsEveryStateAndInputWithoutLegacyFlattening)
{
  const auto result = adapter::extract_canonical_execution_plan(make_request());
  ASSERT_EQ(result.reason, adapter::CanonicalPlanExtractionReason::Accepted);
  ASSERT_TRUE(result.plan.has_value());
  EXPECT_EQ(
    plan::validate_canonical_execution_plan(result.plan.value()),
    plan::CanonicalExecutionPlanRejectReason::None);
  EXPECT_EQ(result.plan->predicted_states.size(), 3U);
  EXPECT_EQ(result.plan->control_stages.size(), 2U);
  EXPECT_DOUBLE_EQ(result.plan->predicted_states[1].lateral_m, 0.20);
  EXPECT_DOUBLE_EQ(result.plan->predicted_states[1].lag_m, 0.02);
  EXPECT_DOUBLE_EQ(result.plan->predicted_states[1].heading_offset_rad, 0.03);
  EXPECT_DOUBLE_EQ(result.plan->predicted_states[1].velocity_mps, 5.2);
  EXPECT_DOUBLE_EQ(result.plan->predicted_states[1].progress_m, 100.5);
  EXPECT_DOUBLE_EQ(result.plan->control_stages[0].acceleration_mps2, 1.0);
  EXPECT_DOUBLE_EQ(result.plan->control_stages[0].curvature_radpm, 0.02);
  EXPECT_DOUBLE_EQ(
    result.plan->control_stages[0].virtual_progress_speed_mps, 5.1);
  EXPECT_DOUBLE_EQ(result.plan->control_stages[0].duration_sec, 0.4);
}

TEST(CanonicalExecutionPlanAdapter, RejectsMalformedPrimalSize)
{
  auto request = make_request();
  request.extended_primal.conservativeResize(20);
  const auto result = adapter::extract_canonical_execution_plan(request);
  EXPECT_EQ(
    result.reason, adapter::CanonicalPlanExtractionReason::PrimalSizeMismatch);
  EXPECT_FALSE(result.plan.has_value());
}

TEST(CanonicalExecutionPlanAdapter, RejectsNonfinitePrimalAndProgressOrigin)
{
  auto nonfinite = make_request();
  nonfinite.extended_primal[6] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(
    adapter::extract_canonical_execution_plan(nonfinite).reason,
    adapter::CanonicalPlanExtractionReason::NonfinitePrimal);

  auto bad_origin = make_request();
  bad_origin.progress_origin_m = std::numeric_limits<double>::infinity();
  EXPECT_EQ(
    adapter::extract_canonical_execution_plan(bad_origin).reason,
    adapter::CanonicalPlanExtractionReason::InvalidProgressOrigin);
}

TEST(CanonicalExecutionPlanAdapter, RejectsMissingStageTimingAndMetadataMismatch)
{
  auto missing_timing = make_request();
  missing_timing.stage_duration_sec.pop_back();
  EXPECT_EQ(
    adapter::extract_canonical_execution_plan(missing_timing).reason,
    adapter::CanonicalPlanExtractionReason::StageDurationCountMismatch);

  auto bad_metadata = make_request();
  bad_metadata.solution.problem_fingerprint += 1U;
  const auto result = adapter::extract_canonical_execution_plan(bad_metadata);
  EXPECT_EQ(
    result.reason, adapter::CanonicalPlanExtractionReason::PlanContractRejected);
  EXPECT_EQ(
    result.plan_reject_reason,
    plan::CanonicalExecutionPlanRejectReason::SolutionIdentityMismatch);
}

TEST(CanonicalExecutionPlanAdapter, ExtractedPlanCanPopulateStoreAndResolveExactCursor)
{
  auto result = adapter::extract_canonical_execution_plan(make_request());
  ASSERT_TRUE(result.plan.has_value());
  plan::CanonicalExecutionPlanStore store;
  EXPECT_EQ(
    store.replace(std::move(result.plan.value())),
    plan::CanonicalExecutionPlanStoreReason::Accepted);
  const auto snapshot = store.snapshot();
  ASSERT_NE(snapshot, nullptr);
  EXPECT_EQ(snapshot->plan_id, 23U);

  const auto cursor = plan::resolve_execution_cursor(*snapshot, 10.5);
  ASSERT_TRUE(cursor.available);
  EXPECT_EQ(cursor.plan_id, 23U);
  EXPECT_EQ(cursor.first_control_stage_index, 1U);
  EXPECT_EQ(cursor.remaining_control_stage_count, 1U);
  EXPECT_NEAR(cursor.stage_elapsed_sec, 0.1, 1e-12);
  EXPECT_DOUBLE_EQ(snapshot->control_stages[1].acceleration_mps2, 0.5);
  EXPECT_DOUBLE_EQ(snapshot->control_stages[1].virtual_progress_speed_mps, 5.3);
}
