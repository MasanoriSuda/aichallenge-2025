#include "multi_purpose_mpc_ros/mpcc_execution_contract.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace contract = multi_purpose_mpc_ros::mpcc_execution_contract;

namespace
{

contract::MpccProblemContext make_context()
{
  contract::MpccProblemContext context;
  context.decision_id = 42U;
  context.intent = contract::ControlIntent::Pass;
  context.intent_generation = 7U;
  context.observation_generation = 19U;
  context.stage_geometry_id = contract::fingerprint_stage_geometry(
    120, true,
    std::vector<contract::StageGeometryIdentity>{
      {120, 121, 0.50, 0.50},
      {121, 122, 0.55, 1.05}});
  context.target_obstacle_generation = 19U;
  context.target_id = "d2";
  context.horizon_steps = 2U;
  context.formulation = contract::Formulation::VelocityProgress5State;
  context.state_schema_id = "ey-elag-epsi-v-progress-v1";
  context.input_schema_id = "accel-curvature-progress-rate-v1";
  context.bounds_schema_id = "stage-wall-obstacle-v1";
  context.cost_schema_id = "velocity-progress-v1";
  return contract::seal_problem_context(std::move(context));
}

contract::CertifiedMpccSolution make_solution(
  const contract::MpccProblemContext & context)
{
  contract::CertifiedMpccSolution solution;
  solution.solution_id = 11U;
  solution.problem_fingerprint = context.fingerprint;
  solution.formulation = context.formulation;
  solution.solved = true;
  solution.finite = true;
  solution.constraints_satisfied = true;
  solution.maximum_constraint_violation = 1.0e-5;
  solution.physical.checked = true;
  solution.physical.wall_clear = true;
  solution.physical.obstacles_clear = true;
  solution.physical.minimum_wall_clearance_m = 0.25;
  solution.physical.minimum_obstacle_clearance_m = 0.15;
  solution.prediction_stage_count = 2U;
  solution.valid_until_sec = 12.5;
  return solution;
}

}  // namespace

TEST(MpccExecutionContract, FingerprintIsStableAndSensitiveToContext)
{
  const auto first = make_context();
  const auto second = make_context();
  ASSERT_TRUE(contract::problem_context_complete(first));
  EXPECT_EQ(first.fingerprint, second.fingerprint);
  EXPECT_NE(first.fingerprint, 0U);

  auto changed = second;
  changed.intent_generation += 1U;
  EXPECT_FALSE(contract::problem_context_complete(changed));
  changed = contract::seal_problem_context(std::move(changed));
  EXPECT_TRUE(contract::problem_context_complete(changed));
  EXPECT_NE(first.fingerprint, changed.fingerprint);
}

TEST(MpccExecutionContract, StageGeometryFingerprintUsesGeometryContent)
{
  const std::vector<contract::StageGeometryIdentity> stages{
    {8, 9, 0.5, 0.5}, {9, 10, 0.6, 1.1}};
  const auto baseline = contract::fingerprint_stage_geometry(8, true, stages);
  EXPECT_EQ(
    baseline, contract::fingerprint_stage_geometry(8, true, stages));

  auto changed = stages;
  changed[1].cumulative_distance_m = 1.2;
  EXPECT_NE(
    baseline, contract::fingerprint_stage_geometry(8, true, changed));
  EXPECT_NE(
    baseline, contract::fingerprint_stage_geometry(8, false, stages));
}

TEST(MpccExecutionContract, CertifiedSolutionRequiresEveryCertificate)
{
  const auto context = make_context();
  auto solution = make_solution(context);
  EXPECT_TRUE(contract::solution_certified(solution));

  solution.physical.wall_clear = false;
  EXPECT_FALSE(contract::solution_certified(solution));
  solution.physical.wall_clear = true;
  solution.maximum_constraint_violation =
    std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(contract::solution_certified(solution));
}

TEST(MpccExecutionContract, AcceptsMatchingCertifiedNormalDecision)
{
  const auto context = make_context();
  const auto solution = make_solution(context);
  const auto decision = contract::resolve_final_control_decision(
    contract::FinalControlDecisionRequest{
      42U, contract::FinalAuthorityClass::CertifiedNormalSolution,
      "mpc-solution", context, solution});

  EXPECT_TRUE(decision.identity_complete);
  EXPECT_TRUE(decision.canonical_contract_satisfied);
  EXPECT_EQ(decision.problem_fingerprint, context.fingerprint);
  EXPECT_EQ(decision.solution_id, solution.solution_id);
  EXPECT_EQ(decision.reason, "matching-certified-solution");
}

TEST(MpccExecutionContract, RejectsMismatchedSolutionIdentity)
{
  const auto context = make_context();
  auto solution = make_solution(context);
  solution.problem_fingerprint += 1U;
  const auto decision = contract::resolve_final_control_decision(
    contract::FinalControlDecisionRequest{
      42U, contract::FinalAuthorityClass::CertifiedNormalSolution,
      "mpc-solution", context, solution});

  EXPECT_FALSE(decision.identity_complete);
  EXPECT_FALSE(decision.canonical_contract_satisfied);
  EXPECT_EQ(decision.reason, "problem-solution-fingerprint-mismatch");
}

TEST(MpccExecutionContract, IdentifiesCertifiedNoncanonicalFormulation)
{
  auto context = make_context();
  context.formulation = contract::Formulation::ProgressContouring3State;
  context.state_schema_id = "ey-epsi-progress-v1";
  context.input_schema_id = "velocity-curvature-v1";
  context.cost_schema_id = "progress-contouring-v1";
  context = contract::seal_problem_context(std::move(context));
  auto solution = make_solution(context);
  solution.formulation = context.formulation;
  const auto decision = contract::resolve_final_control_decision(
    contract::FinalControlDecisionRequest{
      42U, contract::FinalAuthorityClass::CertifiedNormalSolution,
      "mpc-solution", context, solution});

  EXPECT_TRUE(decision.identity_complete);
  EXPECT_FALSE(decision.canonical_contract_satisfied);
  EXPECT_EQ(decision.reason, "matching-certified-noncanonical-formulation");
}

TEST(MpccExecutionContract, RetainedSolutionKeepsOriginalProblemIdentity)
{
  const auto context = make_context();
  const auto solution = make_solution(context);
  const auto decision = contract::resolve_final_control_decision(
    contract::FinalControlDecisionRequest{
      99U, contract::FinalAuthorityClass::CertifiedNormalSolution,
      "mpc-solution", context, solution, true});

  EXPECT_TRUE(decision.identity_complete);
  EXPECT_TRUE(decision.canonical_contract_satisfied);
  EXPECT_TRUE(decision.retained_solution);
  EXPECT_EQ(decision.problem_fingerprint, context.fingerprint);
  EXPECT_EQ(decision.solution_id, solution.solution_id);
}

TEST(MpccExecutionContract, ExposesLegacyBypassWithoutCallingItCanonical)
{
  const auto context = make_context();
  const auto decision = contract::resolve_final_control_decision(
    contract::FinalControlDecisionRequest{
      42U, contract::FinalAuthorityClass::LegacyNormalBypass,
      "low-speed-direct", context, std::nullopt});

  EXPECT_TRUE(decision.identity_complete);
  EXPECT_FALSE(decision.canonical_contract_satisfied);
  EXPECT_EQ(decision.reason, "legacy-normal-bypass");
  EXPECT_NE(
    contract::format_final_control_decision(decision).find(
      "authority=legacy-normal-bypass"),
    std::string::npos);
}

TEST(MpccExecutionContract, ExplicitOverridesDoNotInventSolverIdentity)
{
  for (const auto authority : {
      contract::FinalAuthorityClass::EmergencyOverride,
      contract::FinalAuthorityClass::RecoveryOverride,
      contract::FinalAuthorityClass::ControlDisabled})
  {
    const auto decision = contract::resolve_final_control_decision(
      contract::FinalControlDecisionRequest{
        77U, authority, "explicit-supervisor", std::nullopt, std::nullopt});
    EXPECT_TRUE(decision.identity_complete);
    EXPECT_TRUE(decision.canonical_contract_satisfied);
    EXPECT_EQ(decision.problem_fingerprint, 0U);
    EXPECT_EQ(decision.solution_id, 0U);
  }
}
