#include "multi_purpose_mpc_ros/mpcc_execution_contract.hpp"

#include <gtest/gtest.h>

#include <cmath>
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
  context.execution_side_sign = 1;
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

contract::MpccProblemContext make_track_context(
  const std::uint64_t decision_id = 42U)
{
  auto context = make_context();
  context.decision_id = decision_id;
  context.intent = contract::ControlIntent::Track;
  context.target_id.clear();
  context.target_obstacle_generation = 0U;
  context.execution_side_sign = 0;
  return contract::seal_problem_context(std::move(context));
}

contract::MpccProblemContext make_follow_context(
  const std::uint64_t decision_id = 42U)
{
  auto context = make_track_context(decision_id);
  context.intent = contract::ControlIntent::Follow;
  context.target_id = "d2";
  context.target_obstacle_generation = context.observation_generation;
  context.bounds_schema_id = "progress-stage-wall-follow-target-v1";
  context.cost_schema_id = "velocity-progress-follow-gap-v1";
  return contract::seal_problem_context(std::move(context));
}

contract::CanonicalNormalCandidate make_canonical_candidate(
  const std::uint64_t decision_id = 42U,
  const std::size_t executable_control_stage_count = 2U,
  const std::uint64_t execution_certificate_decision_id = 0U,
  const std::uint64_t execution_plan_id = 23U)
{
  const auto context = make_track_context(decision_id);
  contract::CanonicalNormalCandidate candidate;
  candidate.problem = context;
  candidate.solution = make_solution(context);
  candidate.executable_control_stage_count = executable_control_stage_count;
  candidate.execution_plan_id = execution_plan_id;
  candidate.execution_certificate_decision_id =
    execution_certificate_decision_id == 0U ? decision_id :
    execution_certificate_decision_id;
  candidate.execution_physical.checked = true;
  candidate.execution_physical.wall_clear = true;
  candidate.execution_physical.obstacles_clear = true;
  return candidate;
}

contract::CanonicalNormalCandidate make_follow_candidate(
  const std::uint64_t decision_id = 42U)
{
  const auto context = make_follow_context(decision_id);
  auto candidate = make_canonical_candidate(decision_id);
  candidate.problem = context;
  candidate.solution = make_solution(context);
  return candidate;
}

contract::CanonicalNormalCandidate make_overtake_candidate(
  const contract::ControlIntent intent,
  const std::uint64_t decision_id = 42U)
{
  auto context = make_context();
  context.decision_id = decision_id;
  context.intent = intent;
  context = contract::seal_problem_context(std::move(context));
  auto candidate = make_canonical_candidate(decision_id);
  candidate.problem = context;
  candidate.solution = make_solution(context);
  return candidate;
}

contract::CanonicalNormalCommand make_normal_command(
  const contract::MpccProblemContext & context,
  const contract::CertifiedMpccSolution & solution,
  const std::uint64_t current_decision_id = 42U,
  const bool retained = false)
{
  return contract::CanonicalNormalCommand{
    current_decision_id,
    23U,
    current_decision_id,
    context.fingerprint,
    solution.solution_id,
    retained ? contract::CanonicalNormalAuthoritySource::RetainedCertified :
    contract::CanonicalNormalAuthoritySource::FreshCertified,
    context.intent,
    context.formulation,
    retained,
    7.5,
    0.8,
    -0.12,
    -0.31,
    7.7};
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

TEST(MpccExecutionContract, CanonicalNormalIntentDomainIsExplicit)
{
  for (const auto intent : {
      contract::ControlIntent::Track,
      contract::ControlIntent::Cruise,
      contract::ControlIntent::Follow,
      contract::ControlIntent::ShiftOut,
      contract::ControlIntent::Pass,
      contract::ControlIntent::Return})
  {
    EXPECT_TRUE(contract::canonical_normal_intent_supported(intent));
  }
  for (const auto intent : {
      contract::ControlIntent::Unknown,
      contract::ControlIntent::Hold,
      contract::ControlIntent::Stop,
      contract::ControlIntent::Rejoin})
  {
    EXPECT_FALSE(contract::canonical_normal_intent_supported(intent));
  }
}

TEST(MpccExecutionContract, CanonicalOvertakeIntentsRequireTargetProvenance)
{
  for (const auto intent : {
      contract::ControlIntent::Follow,
      contract::ControlIntent::ShiftOut,
      contract::ControlIntent::Pass,
      contract::ControlIntent::Return})
  {
    EXPECT_TRUE(contract::canonical_normal_intent_requires_target(intent));
    auto context = make_context();
    context.intent = intent;
    context.target_id.clear();
    context.target_obstacle_generation = 0U;
    context = contract::seal_problem_context(std::move(context));
    EXPECT_FALSE(contract::problem_context_complete(context));
  }

  EXPECT_FALSE(
    contract::canonical_normal_intent_requires_target(
      contract::ControlIntent::Track));
  EXPECT_FALSE(
    contract::canonical_normal_intent_requires_target(
      contract::ControlIntent::Cruise));
}

TEST(MpccExecutionContract, CanonicalOvertakeIdentityRequiresExactSide)
{
  auto missing_side = make_context();
  missing_side.execution_side_sign = 0;
  missing_side = contract::seal_problem_context(std::move(missing_side));
  EXPECT_FALSE(contract::problem_context_complete(missing_side));

  auto invalid_side = make_context();
  invalid_side.execution_side_sign = 2;
  invalid_side = contract::seal_problem_context(std::move(invalid_side));
  EXPECT_FALSE(contract::problem_context_complete(invalid_side));

  auto opposite_side = make_context();
  opposite_side.execution_side_sign = -1;
  opposite_side = contract::seal_problem_context(std::move(opposite_side));
  ASSERT_TRUE(contract::problem_context_complete(opposite_side));
  EXPECT_NE(make_context().fingerprint, opposite_side.fingerprint);

  auto track_with_side = make_track_context();
  track_with_side.execution_side_sign = 1;
  track_with_side = contract::seal_problem_context(std::move(track_with_side));
  EXPECT_FALSE(contract::problem_context_complete(track_with_side));
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

TEST(MpccExecutionContract, EffectiveProgressGeometryRebuildsSeamDistanceAndFingerprint)
{
  const std::vector<contract::StageGeometryIdentity> raw{
    {348, 349, 0.90, 0.90},
    {349, 0, 0.0, 0.90},
    {0, 1, 1.0, 1.90}};
  const std::vector<double> effective{0.90, 0.005, 1.0};

  const auto result = contract::resolve_effective_stage_geometry(
    348, true, raw, effective);

  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->stages.size(), 3U);
  EXPECT_DOUBLE_EQ(result->stages[0].cumulative_distance_m, 0.90);
  EXPECT_DOUBLE_EQ(result->stages[1].transition_distance_m, 0.005);
  EXPECT_DOUBLE_EQ(result->stages[1].cumulative_distance_m, 0.905);
  EXPECT_DOUBLE_EQ(result->stages[2].cumulative_distance_m, 1.905);
  EXPECT_EQ(result->stages[1].transition_from_waypoint, 349);
  EXPECT_EQ(result->stages[1].state_waypoint, 0);
  EXPECT_EQ(
    result->fingerprint,
    contract::fingerprint_stage_geometry(348, true, result->stages));
  EXPECT_NE(
    result->fingerprint,
    contract::fingerprint_stage_geometry(348, true, raw));
}

TEST(MpccExecutionContract, EffectiveProgressGeometryRejectsInvalidDistanceContract)
{
  const std::vector<contract::StageGeometryIdentity> raw{
    {8, 9, 0.5, 0.5}, {9, 10, 0.6, 1.1}};

  EXPECT_FALSE(
    contract::resolve_effective_stage_geometry(8, true, raw, {0.5}).has_value());
  EXPECT_FALSE(
    contract::resolve_effective_stage_geometry(8, true, raw, {0.5, 0.0}).has_value());
  EXPECT_FALSE(
    contract::resolve_effective_stage_geometry(
      8, true, raw,
      {0.5, std::numeric_limits<double>::quiet_NaN()}).has_value());
}

TEST(MpccExecutionContract, PhysicalWallCertificateReasonsAreStable)
{
  EXPECT_STREQ(
    contract::physical_wall_certificate_reason_name(
      contract::PhysicalWallCertificateReason::Accepted),
    "accepted");
  EXPECT_STREQ(
    contract::physical_wall_certificate_reason_name(
      contract::PhysicalWallCertificateReason::LateralBoundViolation),
    "lateral-bound-violation");
  EXPECT_STREQ(
    contract::physical_wall_certificate_reason_name(
      contract::PhysicalWallCertificateReason::HardWallContact),
    "hard-wall-contact");
  EXPECT_STREQ(
    contract::physical_wall_certificate_reason_name(
      contract::PhysicalWallCertificateReason::SweptPathViolation),
    "swept-path-violation");
  EXPECT_STREQ(
    contract::physical_wall_certificate_reason_name(
      contract::PhysicalWallCertificateReason::CurrentPoseWallSampleUnavailable),
    "current-pose-wall-sample-unavailable");
  EXPECT_STREQ(
    contract::physical_wall_certificate_reason_name(
      contract::PhysicalWallCertificateReason::CurrentPoseHardWallContact),
    "current-pose-hard-wall-contact");
}

TEST(MpccExecutionContract, SweptPathFailureOriginSeparatesCurrentPoseFromHorizon)
{
  const auto current = contract::resolve_swept_path_failure_origin(0U, 20U);
  EXPECT_EQ(current.origin, contract::PhysicalWallPathFailureOrigin::CurrentPose);
  EXPECT_EQ(current.stage_index, -1);

  const auto first_stage = contract::resolve_swept_path_failure_origin(1U, 20U);
  EXPECT_EQ(first_stage.origin, contract::PhysicalWallPathFailureOrigin::HorizonStage);
  EXPECT_EQ(first_stage.stage_index, 0);

  const auto last_stage = contract::resolve_swept_path_failure_origin(20U, 20U);
  EXPECT_EQ(last_stage.origin, contract::PhysicalWallPathFailureOrigin::HorizonStage);
  EXPECT_EQ(last_stage.stage_index, 19);

  const auto invalid = contract::resolve_swept_path_failure_origin(21U, 20U);
  EXPECT_EQ(invalid.origin, contract::PhysicalWallPathFailureOrigin::Invalid);
  EXPECT_EQ(invalid.stage_index, -1);

  const auto unrepresentable = contract::resolve_swept_path_failure_origin(
    static_cast<std::size_t>(std::numeric_limits<int>::max()) + 2U,
    static_cast<std::size_t>(std::numeric_limits<int>::max()) + 2U);
  EXPECT_EQ(
    unrepresentable.origin,
    contract::PhysicalWallPathFailureOrigin::Invalid);
  EXPECT_EQ(unrepresentable.stage_index, -1);
}

TEST(MpccExecutionContract, PhysicalWallCertificateDiagnosticPreservesFailureProvenance)
{
  contract::PhysicalWallCertificateDiagnostic diagnostic;
  diagnostic.reason = contract::PhysicalWallCertificateReason::HardWallContact;
  diagnostic.stage_index = 3;
  diagnostic.waypoint_id = 123;
  diagnostic.path_distance_m = 2.5;
  diagnostic.lateral_m = 0.8;
  diagnostic.lag_m = -0.4;
  diagnostic.lower_bound_m = -1.0;
  diagnostic.upper_bound_m = 1.0;
  diagnostic.bound_reserve_m = 0.2;
  diagnostic.heading_offset_rad = 0.1;
  diagnostic.reference_progress_m = 12.5;
  diagnostic.solved_progress_m = 11.8;
  diagnostic.progress_delta_m = -0.7;
  diagnostic.pose_x_m = 4.0;
  diagnostic.pose_y_m = 5.0;
  diagnostic.pose_yaw_rad = 0.6;
  diagnostic.contact_cell_count = 4U;

  const std::string formatted =
    contract::format_physical_wall_certificate_diagnostic(diagnostic);

  EXPECT_NE(formatted.find("reason=hard-wall-contact"), std::string::npos);
  EXPECT_NE(formatted.find("stage=3"), std::string::npos);
  EXPECT_NE(formatted.find("wp=123"), std::string::npos);
  EXPECT_NE(formatted.find("distance=2.500m"), std::string::npos);
  EXPECT_NE(formatted.find("lateral=0.800m"), std::string::npos);
  EXPECT_NE(formatted.find("lag=-0.400m"), std::string::npos);
  EXPECT_NE(formatted.find("bounds=[-1.000,1.000]m"), std::string::npos);
  EXPECT_NE(formatted.find("reserve=0.200m"), std::string::npos);
  EXPECT_NE(formatted.find("heading_offset=0.100rad"), std::string::npos);
  EXPECT_NE(formatted.find("progress=11.800/12.500m"), std::string::npos);
  EXPECT_NE(formatted.find("progress_delta=-0.700m"), std::string::npos);
  EXPECT_NE(formatted.find("pose=(4.000,5.000,0.600)"), std::string::npos);
  EXPECT_NE(formatted.find("contacts=4"), std::string::npos);
}

TEST(MpccExecutionContract, NamesMissingSolvedProgressCourseFrame)
{
  EXPECT_STREQ(
    contract::physical_wall_certificate_reason_name(
      contract::PhysicalWallCertificateReason::CourseFrameUnavailable),
    "course-frame-unavailable");
}

TEST(MpccExecutionContract, ProjectsWorldPoseIntoCompleteFrenetState)
{
  const auto state = contract::project_planar_pose_to_frenet(
    contract::PlanarPose{9.5, 22.0, 0.5 * M_PI + 0.2},
    contract::PlanarPose{10.0, 20.0, 0.5 * M_PI});

  ASSERT_TRUE(state.has_value());
  EXPECT_NEAR(state->lateral_m, 0.5, 1e-12);
  EXPECT_NEAR(state->lag_m, 2.0, 1e-12);
  EXPECT_NEAR(state->heading_offset_rad, 0.2, 1e-12);
}

TEST(MpccExecutionContract, FrenetProjectionAndReconstructionAreInverse)
{
  const contract::PlanarPose course_frame{13.0, -4.0, -2.4};
  const contract::FrenetPose state{0.37, -0.62, 0.19};

  const auto world = contract::reconstruct_planar_pose_from_frenet(
    course_frame, state);
  ASSERT_TRUE(world.has_value());
  const auto round_trip = contract::project_planar_pose_to_frenet(
    world.value(), course_frame);

  ASSERT_TRUE(round_trip.has_value());
  EXPECT_NEAR(round_trip->lateral_m, state.lateral_m, 1e-12);
  EXPECT_NEAR(round_trip->lag_m, state.lag_m, 1e-12);
  EXPECT_NEAR(round_trip->heading_offset_rad, state.heading_offset_rad, 1e-12);
}

TEST(MpccExecutionContract, FrenetPoseContractRejectsNonfiniteInput)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(contract::project_planar_pose_to_frenet(
    contract::PlanarPose{nan, 0.0, 0.0},
    contract::PlanarPose{0.0, 0.0, 0.0}).has_value());
  EXPECT_FALSE(contract::reconstruct_planar_pose_from_frenet(
    contract::PlanarPose{0.0, 0.0, 0.0},
    contract::FrenetPose{0.0, nan, 0.0}).has_value());
}

TEST(MpccExecutionContract, IntegratesFirstStageStraightWithAcceleration)
{
  const auto result = contract::integrate_first_stage_constant_curvature(
    contract::FirstStageKinematicRequest{
      {1.0, 2.0, 0.5}, 4.0, 1.0, 0.0, 0.2, 0.2});

  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->travel_distance_m, 0.82, 1e-12);
  EXPECT_NEAR(result->pose.x_m, 1.0 + 0.82 * std::cos(0.5), 1e-12);
  EXPECT_NEAR(result->pose.y_m, 2.0 + 0.82 * std::sin(0.5), 1e-12);
  EXPECT_NEAR(result->pose.yaw_rad, 0.5, 1e-12);
}

TEST(MpccExecutionContract, IntegratesFirstStageConstantCurvatureArc)
{
  const auto result = contract::integrate_first_stage_constant_curvature(
    contract::FirstStageKinematicRequest{
      {0.0, 0.0, 0.0}, 2.0, 0.0, 0.5, 1.0, 1.0});

  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->travel_distance_m, 2.0, 1e-12);
  EXPECT_NEAR(result->pose.x_m, std::sin(1.0) / 0.5, 1e-12);
  EXPECT_NEAR(result->pose.y_m, (1.0 - std::cos(1.0)) / 0.5, 1e-12);
  EXPECT_NEAR(result->pose.yaw_rad, 1.0, 1e-12);
}

TEST(MpccExecutionContract, FirstStageIntegrationStopsBeforeVelocityReverses)
{
  const auto result = contract::integrate_first_stage_constant_curvature(
    contract::FirstStageKinematicRequest{
      {0.0, 0.0, 0.0}, 1.0, -2.0, 0.0, 1.0, 1.0});

  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->travel_distance_m, 0.25, 1e-12);
  EXPECT_NEAR(result->pose.x_m, 0.25, 1e-12);
  EXPECT_NEAR(result->pose.y_m, 0.0, 1e-12);
}

TEST(MpccExecutionContract, RejectsInvalidFirstStageKinematicInput)
{
  EXPECT_FALSE(contract::integrate_first_stage_constant_curvature(
      contract::FirstStageKinematicRequest{
        {0.0, 0.0, 0.0}, -1.0, 0.0, 0.0, 0.1, 0.1}).has_value());
  EXPECT_FALSE(contract::integrate_first_stage_constant_curvature(
      contract::FirstStageKinematicRequest{
        {0.0, 0.0, 0.0}, 1.0, 0.0, 0.0, 0.1, 0.2}).has_value());
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
  const auto context = make_track_context();
  const auto solution = make_solution(context);
  const auto command = make_normal_command(context, solution);
  const auto decision = contract::resolve_final_control_decision(
    contract::FinalControlDecisionRequest{
      42U, contract::FinalAuthorityClass::CertifiedNormalSolution,
      "mpc-solution", context, solution, false, command});

  EXPECT_TRUE(decision.identity_complete);
  EXPECT_TRUE(decision.canonical_contract_satisfied);
  EXPECT_EQ(decision.problem_fingerprint, context.fingerprint);
  EXPECT_EQ(decision.solution_id, solution.solution_id);
  EXPECT_EQ(decision.execution_plan_id, 23U);
  EXPECT_EQ(decision.execution_certificate_decision_id, 42U);
  EXPECT_EQ(
    decision.canonical_source,
    contract::CanonicalNormalAuthoritySource::FreshCertified);
  EXPECT_EQ(decision.reason, "matching-certified-solution");
}

TEST(MpccExecutionContract, RejectsMismatchedSolutionIdentity)
{
  const auto context = make_track_context();
  auto solution = make_solution(context);
  const auto command = make_normal_command(context, solution);
  solution.problem_fingerprint += 1U;
  const auto decision = contract::resolve_final_control_decision(
    contract::FinalControlDecisionRequest{
      42U, contract::FinalAuthorityClass::CertifiedNormalSolution,
      "mpc-solution", context, solution, false, command});

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
  const auto context = make_track_context();
  const auto solution = make_solution(context);
  const auto command = make_normal_command(context, solution, 99U, true);
  const auto decision = contract::resolve_final_control_decision(
    contract::FinalControlDecisionRequest{
      99U, contract::FinalAuthorityClass::CertifiedNormalSolution,
      "mpc-solution", context, solution, true, command});

  EXPECT_TRUE(decision.identity_complete);
  EXPECT_TRUE(decision.canonical_contract_satisfied);
  EXPECT_TRUE(decision.retained_solution);
  EXPECT_EQ(decision.problem_fingerprint, context.fingerprint);
  EXPECT_EQ(decision.solution_id, solution.solution_id);
  EXPECT_EQ(decision.execution_plan_id, 23U);
  EXPECT_EQ(decision.execution_certificate_decision_id, 99U);
  EXPECT_EQ(
    decision.canonical_source,
    contract::CanonicalNormalAuthoritySource::RetainedCertified);
}

TEST(MpccExecutionContract, RejectsCanonicalFinalDecisionWithoutCommandIdentity)
{
  const auto context = make_track_context();
  const auto solution = make_solution(context);

  const auto decision = contract::resolve_final_control_decision(
    contract::FinalControlDecisionRequest{
      42U, contract::FinalAuthorityClass::CertifiedNormalSolution,
      "mpc-solution", context, solution});

  EXPECT_FALSE(decision.identity_complete);
  EXPECT_FALSE(decision.canonical_contract_satisfied);
  EXPECT_EQ(decision.reason, "missing-canonical-command-identity");
}

TEST(MpccExecutionContract, RejectsRetainedFinalDecisionWithoutCurrentWorldProofIdentity)
{
  const auto context = make_track_context();
  const auto solution = make_solution(context);
  auto command = make_normal_command(context, solution, 99U, true);
  command.execution_certificate_decision_id = 98U;

  const auto decision = contract::resolve_final_control_decision(
    contract::FinalControlDecisionRequest{
      99U, contract::FinalAuthorityClass::CertifiedNormalSolution,
      "mpc-solution", context, solution, true, command});

  EXPECT_FALSE(decision.identity_complete);
  EXPECT_FALSE(decision.canonical_contract_satisfied);
  EXPECT_EQ(decision.reason, "canonical-command-identity-mismatch");
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

TEST(MpccExecutionContract, EmergencyOverridePreservesExplicitSupervisorIntent)
{
  const auto decision = contract::resolve_final_control_decision(
    contract::FinalControlDecisionRequest{
      77U, contract::FinalAuthorityClass::EmergencyOverride,
      "explicit-supervisor", std::nullopt, std::nullopt, false,
      std::nullopt, contract::ControlIntent::Stop});

  EXPECT_TRUE(decision.identity_complete);
  EXPECT_TRUE(decision.canonical_contract_satisfied);
  EXPECT_EQ(decision.intent, contract::ControlIntent::Stop);
  EXPECT_EQ(decision.formulation, contract::Formulation::Unresolved);
  EXPECT_EQ(decision.problem_fingerprint, 0U);
  EXPECT_EQ(decision.solution_id, 0U);
  EXPECT_EQ(decision.reason, "explicit-supervisor-override");
}

TEST(MpccExecutionContract, CanonicalNormalAuthoritySelectsFreshCurrentDecision)
{
  const auto fresh = make_canonical_candidate();
  const auto retained = make_canonical_candidate(41U);
  const auto resolution = contract::resolve_canonical_normal_authority(
    contract::CanonicalNormalAuthorityRequest{
      42U, 12.0, fresh, retained, contract::ControlIntent::Track});

  EXPECT_EQ(
    resolution.source,
    contract::CanonicalNormalAuthoritySource::FreshCertified);
  EXPECT_EQ(
    resolution.reason,
    contract::CanonicalNormalAuthorityReason::FreshCertified);
  EXPECT_EQ(
    resolution.fresh_reject_reason,
    contract::CanonicalNormalCandidateRejectReason::None);
  ASSERT_TRUE(resolution.problem.has_value());
  ASSERT_TRUE(resolution.solution.has_value());
  EXPECT_EQ(resolution.problem->decision_id, 42U);
  EXPECT_EQ(resolution.execution_plan_id, 23U);
  EXPECT_EQ(resolution.execution_certificate_decision_id, 42U);
  EXPECT_EQ(resolution.execution_first_control_stage_index, 0U);
  EXPECT_FALSE(resolution.retained_solution);
}

TEST(MpccExecutionContract, BuildsFreshCanonicalCommandWithoutFlatteningActuation)
{
  const auto resolution = contract::resolve_canonical_normal_authority(
    contract::CanonicalNormalAuthorityRequest{
      42U, 12.0, make_canonical_candidate(), {},
      contract::ControlIntent::Track});
  const contract::CanonicalActuation actuation{
    7.5, 0.8, -0.12, -0.31, 7.7};

  const auto result = contract::build_canonical_normal_command(
    resolution, actuation);

  ASSERT_TRUE(result.command.has_value());
  EXPECT_EQ(result.reason, contract::CanonicalNormalCommandReason::Available);
  EXPECT_EQ(result.command->decision_id, 42U);
  EXPECT_EQ(result.command->execution_plan_id, 23U);
  EXPECT_EQ(
    result.command->source,
    contract::CanonicalNormalAuthoritySource::FreshCertified);
  EXPECT_EQ(result.command->intent, contract::ControlIntent::Track);
  EXPECT_DOUBLE_EQ(result.command->predicted_speed_mps, 7.5);
  EXPECT_DOUBLE_EQ(result.command->acceleration_mps2, 0.8);
  EXPECT_DOUBLE_EQ(result.command->curvature_radpm, -0.12);
  EXPECT_DOUBLE_EQ(result.command->steering_tire_angle_rad, -0.31);
  EXPECT_DOUBLE_EQ(result.command->virtual_progress_speed_mps, 7.7);
}

TEST(MpccExecutionContract, CanonicalNormalAuthorityAcceptsCertifiedFollowIntent)
{
  const auto resolution = contract::resolve_canonical_normal_authority(
    contract::CanonicalNormalAuthorityRequest{
      42U, 12.0, make_follow_candidate(), {},
      contract::ControlIntent::Follow});

  ASSERT_EQ(
    resolution.source,
    contract::CanonicalNormalAuthoritySource::FreshCertified);
  ASSERT_TRUE(resolution.problem.has_value());
  EXPECT_EQ(resolution.problem->intent, contract::ControlIntent::Follow);
  EXPECT_EQ(resolution.problem->target_id, "d2");
  EXPECT_GT(resolution.problem->target_obstacle_generation, 0U);

  const auto result = contract::build_canonical_normal_command(
    resolution, contract::CanonicalActuation{7.0, 0.5, 0.1, 0.2, 6.9});
  ASSERT_TRUE(result.command.has_value());
  EXPECT_EQ(result.command->intent, contract::ControlIntent::Follow);
}

TEST(MpccExecutionContract, CanonicalNormalAuthorityAcceptsExactOvertakeIntent)
{
  for (const auto intent : {
      contract::ControlIntent::ShiftOut,
      contract::ControlIntent::Pass,
      contract::ControlIntent::Return})
  {
    const auto resolution = contract::resolve_canonical_normal_authority(
      contract::CanonicalNormalAuthorityRequest{
        42U, 12.0, make_overtake_candidate(intent), {}, intent});
    ASSERT_EQ(
      resolution.source,
      contract::CanonicalNormalAuthoritySource::FreshCertified);
    ASSERT_TRUE(resolution.problem.has_value());
    EXPECT_EQ(resolution.problem->intent, intent);
    EXPECT_EQ(resolution.problem->target_id, "d2");
    EXPECT_GT(resolution.problem->target_obstacle_generation, 0U);
  }
}

TEST(MpccExecutionContract, BuildsRetainedCanonicalCommandWithOriginalPlanIdentity)
{
  const auto resolution = contract::resolve_canonical_normal_authority(
    contract::CanonicalNormalAuthorityRequest{
      99U, 12.0, {}, make_canonical_candidate(41U, 2U, 99U),
      contract::ControlIntent::Track});
  const contract::CanonicalActuation actuation{
    6.2, -0.4, 0.08, 0.19, 6.0};

  const auto result = contract::build_canonical_normal_command(
    resolution, actuation);

  ASSERT_TRUE(result.command.has_value());
  EXPECT_EQ(result.command->decision_id, 99U);
  EXPECT_EQ(result.command->execution_plan_id, 23U);
  EXPECT_EQ(
    result.command->source,
    contract::CanonicalNormalAuthoritySource::RetainedCertified);
  EXPECT_TRUE(result.command->retained_solution);
}

TEST(MpccExecutionContract, EmergencyAuthorityCannotProduceNormalCommand)
{
  const auto resolution = contract::resolve_canonical_normal_authority(
    contract::CanonicalNormalAuthorityRequest{
      42U, 12.0, {}, {}, contract::ControlIntent::Cruise});

  const auto result = contract::build_canonical_normal_command(
    resolution, contract::CanonicalActuation{1.0, 0.0, 0.0, 0.0, 1.0});

  EXPECT_FALSE(result.command.has_value());
  EXPECT_EQ(
    result.reason,
    contract::CanonicalNormalCommandReason::EmergencyAuthority);
}

TEST(MpccExecutionContract, DetectsPostprocessorMutationOfCanonicalCommand)
{
  const auto resolution = contract::resolve_canonical_normal_authority(
    contract::CanonicalNormalAuthorityRequest{
      42U, 12.0, make_canonical_candidate(), {},
      contract::ControlIntent::Track});
  const auto result = contract::build_canonical_normal_command(
    resolution, contract::CanonicalActuation{7.5, 0.8, -0.12, -0.31, 7.7});
  ASSERT_TRUE(result.command.has_value());

  EXPECT_TRUE(contract::canonical_normal_command_matches_actuation(
      result.command.value(), 7.5, 0.8, -0.31));
  EXPECT_FALSE(contract::canonical_normal_command_matches_actuation(
      result.command.value(), 7.5, 0.79, -0.31));
  EXPECT_FALSE(contract::canonical_normal_command_matches_actuation(
      result.command.value(), 7.5, 0.8, -0.30));
}

TEST(MpccExecutionContract, CanonicalNormalAuthorityFallsBackOnlyToRetainedCanonical)
{
  auto fresh = make_canonical_candidate();
  fresh.solution->physical.wall_clear = false;
  const auto retained = make_canonical_candidate(41U, 2U, 42U);
  const auto resolution = contract::resolve_canonical_normal_authority(
    contract::CanonicalNormalAuthorityRequest{
      42U, 12.0, fresh, retained, contract::ControlIntent::Track});

  EXPECT_EQ(
    resolution.source,
    contract::CanonicalNormalAuthoritySource::RetainedCertified);
  EXPECT_EQ(
    resolution.reason,
    contract::CanonicalNormalAuthorityReason::RetainedCertified);
  EXPECT_EQ(
    resolution.fresh_reject_reason,
    contract::CanonicalNormalCandidateRejectReason::NotCertified);
  ASSERT_TRUE(resolution.problem.has_value());
  EXPECT_EQ(resolution.problem->decision_id, 41U);
  EXPECT_EQ(resolution.execution_plan_id, 23U);
  EXPECT_EQ(resolution.execution_certificate_decision_id, 42U);
  EXPECT_EQ(resolution.execution_first_control_stage_index, 0U);
  EXPECT_TRUE(resolution.retained_solution);
}

TEST(MpccExecutionContract, CanonicalNormalAuthorityRejectsRetainedAcrossIntentChange)
{
  auto fresh = make_canonical_candidate();
  fresh.solution->physical.wall_clear = false;
  const auto retained_track = make_canonical_candidate(41U, 2U, 42U);
  auto request = contract::CanonicalNormalAuthorityRequest{
    42U, 12.0, fresh, retained_track};
  request.current_intent = contract::ControlIntent::Cruise;

  const auto resolution =
    contract::resolve_canonical_normal_authority(request);

  EXPECT_EQ(
    resolution.source,
    contract::CanonicalNormalAuthoritySource::EmergencyStop);
  EXPECT_EQ(
    resolution.retained_reject_reason,
    contract::CanonicalNormalCandidateRejectReason::IntentMismatch);
}

TEST(MpccExecutionContract, CanonicalNormalAuthorityRejectsUnsupportedCurrentIntent)
{
  auto request = contract::CanonicalNormalAuthorityRequest{
    42U, 12.0, make_canonical_candidate(), {}};
  request.current_intent = contract::ControlIntent::Hold;

  const auto resolution =
    contract::resolve_canonical_normal_authority(request);

  EXPECT_EQ(
    resolution.source,
    contract::CanonicalNormalAuthoritySource::EmergencyStop);
  EXPECT_EQ(
    resolution.reason,
    contract::CanonicalNormalAuthorityReason::InvalidRequest);
}

TEST(MpccExecutionContract, CanonicalNormalAuthorityRejectsRetainedWithoutCurrentExecutionProof)
{
  auto fresh = make_canonical_candidate();
  fresh.solution->physical.wall_clear = false;
  const auto retained_with_only_original_proof = make_canonical_candidate(41U);
  const auto resolution = contract::resolve_canonical_normal_authority(
    contract::CanonicalNormalAuthorityRequest{
      42U, 12.0, fresh, retained_with_only_original_proof,
      contract::ControlIntent::Track});

  EXPECT_EQ(
    resolution.source,
    contract::CanonicalNormalAuthoritySource::EmergencyStop);
  EXPECT_EQ(
    resolution.retained_reject_reason,
    contract::CanonicalNormalCandidateRejectReason::
      ExecutionCertificateDecisionMismatch);
}

TEST(MpccExecutionContract, CanonicalNormalAuthorityRequiresExecutionPlanIdentity)
{
  auto fresh = make_canonical_candidate();
  fresh.execution_plan_id = 0U;
  const auto resolution = contract::resolve_canonical_normal_authority(
    contract::CanonicalNormalAuthorityRequest{
      42U, 12.0, fresh, contract::CanonicalNormalCandidate{},
      contract::ControlIntent::Track});

  EXPECT_EQ(
    resolution.source,
    contract::CanonicalNormalAuthoritySource::EmergencyStop);
  EXPECT_EQ(
    resolution.fresh_reject_reason,
    contract::CanonicalNormalCandidateRejectReason::MissingExecutionPlan);
}

TEST(MpccExecutionContract, CanonicalNormalAuthorityRejectsStaleFreshExecutionProof)
{
  auto fresh = make_canonical_candidate();
  fresh.execution_certificate_decision_id = 41U;
  const auto resolution = contract::resolve_canonical_normal_authority(
    contract::CanonicalNormalAuthorityRequest{
      42U, 12.0, fresh, contract::CanonicalNormalCandidate{},
      contract::ControlIntent::Track});

  EXPECT_EQ(
    resolution.source,
    contract::CanonicalNormalAuthoritySource::EmergencyStop);
  EXPECT_EQ(
    resolution.fresh_reject_reason,
    contract::CanonicalNormalCandidateRejectReason::
      ExecutionCertificateDecisionMismatch);
}

TEST(MpccExecutionContract, CanonicalNormalAuthorityRejectsUnsafeExecutionProof)
{
  auto fresh = make_canonical_candidate();
  fresh.execution_physical.wall_clear = false;
  const auto resolution = contract::resolve_canonical_normal_authority(
    contract::CanonicalNormalAuthorityRequest{
      42U, 12.0, fresh, contract::CanonicalNormalCandidate{},
      contract::ControlIntent::Track});

  EXPECT_EQ(
    resolution.source,
    contract::CanonicalNormalAuthoritySource::EmergencyStop);
  EXPECT_EQ(
    resolution.fresh_reject_reason,
    contract::CanonicalNormalCandidateRejectReason::
      ExecutionCertificateNotCertified);
}

TEST(MpccExecutionContract, CanonicalNormalAuthorityRequiresExecutableControl)
{
  const auto fresh = make_canonical_candidate(42U, 0U);
  const auto resolution = contract::resolve_canonical_normal_authority(
    contract::CanonicalNormalAuthorityRequest{
      42U, 12.0, fresh, contract::CanonicalNormalCandidate{},
      contract::ControlIntent::Track});

  EXPECT_EQ(
    resolution.source,
    contract::CanonicalNormalAuthoritySource::EmergencyStop);
  EXPECT_EQ(
    resolution.fresh_reject_reason,
    contract::CanonicalNormalCandidateRejectReason::NoExecutableControl);
}

TEST(MpccExecutionContract, CanonicalNormalAuthorityRejectsExpiredCandidate)
{
  auto fresh = make_canonical_candidate();
  fresh.solution->valid_until_sec = 11.9;
  const auto resolution = contract::resolve_canonical_normal_authority(
    contract::CanonicalNormalAuthorityRequest{
      42U, 12.0, fresh, contract::CanonicalNormalCandidate{},
      contract::ControlIntent::Track});

  EXPECT_EQ(
    resolution.source,
    contract::CanonicalNormalAuthoritySource::EmergencyStop);
  EXPECT_EQ(
    resolution.fresh_reject_reason,
    contract::CanonicalNormalCandidateRejectReason::Expired);
}

TEST(MpccExecutionContract, CanonicalNormalAuthorityRejectsFreshFromOlderDecision)
{
  const auto old_fresh = make_canonical_candidate(41U);
  const auto retained = make_canonical_candidate(40U, 2U, 42U);
  const auto resolution = contract::resolve_canonical_normal_authority(
    contract::CanonicalNormalAuthorityRequest{
      42U, 12.0, old_fresh, retained, contract::ControlIntent::Track});

  EXPECT_EQ(
    resolution.source,
    contract::CanonicalNormalAuthoritySource::RetainedCertified);
  EXPECT_EQ(
    resolution.fresh_reject_reason,
    contract::CanonicalNormalCandidateRejectReason::DecisionMismatch);
}

TEST(MpccExecutionContract, CanonicalNormalAuthorityRejectsMalformedExecutableHorizon)
{
  const auto fresh = make_canonical_candidate(42U, 3U);
  const auto resolution = contract::resolve_canonical_normal_authority(
    contract::CanonicalNormalAuthorityRequest{
      42U, 12.0, fresh, contract::CanonicalNormalCandidate{},
      contract::ControlIntent::Track});

  EXPECT_EQ(
    resolution.source,
    contract::CanonicalNormalAuthoritySource::EmergencyStop);
  EXPECT_EQ(
    resolution.fresh_reject_reason,
    contract::CanonicalNormalCandidateRejectReason::InvalidExecutableHorizon);
}

TEST(MpccExecutionContract, CanonicalNormalAuthorityRejectsMismatchedIdentity)
{
  auto fresh = make_canonical_candidate();
  fresh.solution->problem_fingerprint += 1U;
  const auto resolution = contract::resolve_canonical_normal_authority(
    contract::CanonicalNormalAuthorityRequest{
      42U, 12.0, fresh, contract::CanonicalNormalCandidate{},
      contract::ControlIntent::Track});

  EXPECT_EQ(
    resolution.source,
    contract::CanonicalNormalAuthoritySource::EmergencyStop);
  EXPECT_EQ(
    resolution.fresh_reject_reason,
    contract::CanonicalNormalCandidateRejectReason::IdentityMismatch);
}

TEST(MpccExecutionContract, CanonicalNormalAuthorityRejectsThreeStateTrack)
{
  auto context = make_track_context();
  context.formulation = contract::Formulation::ProgressContouring3State;
  context.state_schema_id = "ey-epsi-progress-v1";
  context.input_schema_id = "velocity-curvature-v1";
  context.cost_schema_id = "progress-contouring-v1";
  context = contract::seal_problem_context(std::move(context));
  auto fresh = make_canonical_candidate();
  fresh.problem = context;
  fresh.solution = make_solution(context);
  const auto resolution = contract::resolve_canonical_normal_authority(
    contract::CanonicalNormalAuthorityRequest{
      42U, 12.0, fresh, contract::CanonicalNormalCandidate{},
      contract::ControlIntent::Track});

  EXPECT_EQ(
    resolution.source,
    contract::CanonicalNormalAuthoritySource::EmergencyStop);
  EXPECT_EQ(
    resolution.fresh_reject_reason,
    contract::CanonicalNormalCandidateRejectReason::NoncanonicalFormulation);
}

TEST(MpccExecutionContract, CanonicalNormalAuthorityRejectsCrossIntentCandidate)
{
  const auto context = make_context();
  auto fresh = make_canonical_candidate();
  fresh.problem = context;
  fresh.solution = make_solution(context);
  const auto resolution = contract::resolve_canonical_normal_authority(
    contract::CanonicalNormalAuthorityRequest{
      42U, 12.0, fresh, contract::CanonicalNormalCandidate{},
      contract::ControlIntent::Track});

  EXPECT_EQ(
    resolution.source,
    contract::CanonicalNormalAuthoritySource::EmergencyStop);
  EXPECT_EQ(
    resolution.fresh_reject_reason,
    contract::CanonicalNormalCandidateRejectReason::IntentMismatch);
}

TEST(MpccExecutionContract, CanonicalNormalAuthorityFailsClosedWithoutCandidate)
{
  const auto resolution = contract::resolve_canonical_normal_authority(
    contract::CanonicalNormalAuthorityRequest{
      42U, 12.0, contract::CanonicalNormalCandidate{},
      contract::CanonicalNormalCandidate{}, contract::ControlIntent::Track});

  EXPECT_EQ(
    resolution.source,
    contract::CanonicalNormalAuthoritySource::EmergencyStop);
  EXPECT_EQ(
    resolution.reason,
    contract::CanonicalNormalAuthorityReason::NoCanonicalCandidate);
  EXPECT_EQ(
    resolution.fresh_reject_reason,
    contract::CanonicalNormalCandidateRejectReason::MissingIdentity);
  EXPECT_EQ(
    resolution.retained_reject_reason,
    contract::CanonicalNormalCandidateRejectReason::MissingIdentity);
  EXPECT_FALSE(resolution.problem.has_value());
  EXPECT_FALSE(resolution.solution.has_value());
}

TEST(MpccExecutionContract, CanonicalNormalAuthorityRejectsInvalidRequestTime)
{
  const auto resolution = contract::resolve_canonical_normal_authority(
    contract::CanonicalNormalAuthorityRequest{
      42U, std::numeric_limits<double>::quiet_NaN(),
      make_canonical_candidate(), contract::CanonicalNormalCandidate{},
      contract::ControlIntent::Track});

  EXPECT_EQ(
    resolution.source,
    contract::CanonicalNormalAuthoritySource::EmergencyStop);
  EXPECT_EQ(
    resolution.reason,
    contract::CanonicalNormalAuthorityReason::InvalidRequest);
}

TEST(MpccExecutionContract, CanonicalPublicationPreservesCertifiedPhysicalSteering)
{
  const auto published = contract::resolve_published_steering_tire_angle(
    0.21, 1.5, true);

  ASSERT_TRUE(published.has_value());
  EXPECT_DOUBLE_EQ(published.value(), 0.21);
}

TEST(MpccExecutionContract, LegacyPublicationRetainsActuatorCalibrationDuringMigration)
{
  const auto published = contract::resolve_published_steering_tire_angle(
    0.21, 1.5, false);

  ASSERT_TRUE(published.has_value());
  EXPECT_DOUBLE_EQ(published.value(), 0.315);
}

TEST(MpccExecutionContract, PublicationRejectsInvalidSteeringContract)
{
  EXPECT_FALSE(contract::resolve_published_steering_tire_angle(
    std::numeric_limits<double>::quiet_NaN(), 1.5, true).has_value());
  EXPECT_FALSE(contract::resolve_published_steering_tire_angle(
    0.21, 0.0, false).has_value());
}

TEST(MpccExecutionContract, CanonicalEmergencyKeepsPhysicalSteeringConvention)
{
  EXPECT_TRUE(contract::canonical_normal_uses_physical_steering(
      false, true, false));
  EXPECT_TRUE(contract::canonical_normal_uses_physical_steering(
      true, false, false));
  EXPECT_FALSE(contract::canonical_normal_uses_physical_steering(
      false, false, false));
  EXPECT_FALSE(contract::canonical_normal_uses_physical_steering(
      true, false, true));
  EXPECT_FALSE(contract::canonical_normal_uses_physical_steering(
      false, true, true));
}
