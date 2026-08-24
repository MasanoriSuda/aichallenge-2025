#include "multi_purpose_mpc_ros/canonical_retained_revalidation.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <utility>

namespace
{

namespace contract = multi_purpose_mpc_ros::mpcc_execution_contract;
namespace plan = multi_purpose_mpc_ros::canonical_execution_plan;
namespace retained = multi_purpose_mpc_ros::canonical_retained_revalidation;

plan::CanonicalExecutionPlan make_plan()
{
  contract::MpccProblemContext problem;
  problem.decision_id = 42U;
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
  value.control_stages = {plan::CanonicalControlStage{1.0, 0.02, 5.1, 1.0},
    plan::CanonicalControlStage{0.5, 0.03, 5.3, 1.0}};
  value.lateral_lower_m = {-0.5, -0.5, -0.5};
  value.lateral_upper_m = {0.5, 0.5, 0.5};
  return value;
}

retained::CurrentExecutionProvenance make_current()
{
  retained::CurrentExecutionProvenance current;
  current.decision_id = 43U;
  current.intent = contract::ControlIntent::Track;
  current.intent_generation = 3U;
  current.observation_generation = 8U;
  current.stage_geometry_id = 12U;
  current.control_pose_id = 21U;
  current.course_frame_window_id = 22U;
  current.obstacle_tube_id = 23U;
  current.observation_sec = 10.6;
  current.path_length_m = 100.0;
  current.circular = true;
  return current;
}

retained::RetainedExecutionProofRequest
make_request(
  const plan::CanonicalExecutionPlan & execution_plan,
  const plan::CanonicalExecutionCursor & cursor)
{
  const auto window =
    retained::build_retained_execution_window(execution_plan, cursor);
  EXPECT_EQ(window.reason, retained::RetainedExecutionWindowReason::Accepted);
  EXPECT_TRUE(window.window.has_value());

  retained::RetainedExecutionProofRequest request;
  request.current = make_current();
  request.measured_course_progress_m = 0.30;
  request.progress_continuity_tolerance_m = 0.20;
  const double lifted_progress_m = 100.30;
  const auto segment = [&](const bool clear) {
      retained::RetainedPathSegmentEvaluation evaluation;
      evaluation.observation_generation = request.current.observation_generation;
      evaluation.stage_geometry_id = request.current.stage_geometry_id;
      evaluation.target_obstacle_generation =
        request.current.target_obstacle_generation;
      evaluation.control_pose_id = request.current.control_pose_id;
      evaluation.course_frame_window_id = request.current.course_frame_window_id;
      evaluation.obstacle_tube_id = request.current.obstacle_tube_id;
      evaluation.start_progress_m = lifted_progress_m;
      evaluation.end_progress_m = lifted_progress_m;
      evaluation.checked = true;
      evaluation.wall_clear = clear;
      evaluation.obstacles_clear = clear;
      evaluation.minimum_wall_clearance_m = clear ? 0.25 : 0.0;
      evaluation.minimum_obstacle_clearance_m = clear ? 0.20 : 0.0;
      return evaluation;
    };
  request.measured_to_control_prefix = segment(true);
  request.control_to_retained_connector = segment(true);
  request.control_to_retained_connector.end_progress_m =
    window.window->expected_current_progress_m;

  for (const auto & sample : window.window->samples) {
    retained::RetainedStageSafetyEvaluation evaluation;
    evaluation.control_stage_index = sample.control_stage_index;
    evaluation.relative_time_sec = sample.relative_time_sec;
    evaluation.segment_duration_sec = sample.segment_duration_sec;
    evaluation.segment_start_progress_m = sample.segment_start_progress_m;
    evaluation.absolute_progress_m = sample.absolute_progress_m;
    evaluation.observation_generation = request.current.observation_generation;
    evaluation.stage_geometry_id = request.current.stage_geometry_id;
    evaluation.target_obstacle_generation =
      request.current.target_obstacle_generation;
    evaluation.course_frame_window_id = request.current.course_frame_window_id;
    evaluation.obstacle_tube_id = request.current.obstacle_tube_id;
    evaluation.course_frame_available = true;
    evaluation.wall_checked = true;
    evaluation.wall_clear = true;
    evaluation.obstacle_checked = true;
    evaluation.obstacles_clear = true;
    evaluation.minimum_wall_clearance_m = 0.25;
    evaluation.minimum_obstacle_clearance_m = 0.20;
    request.stage_evaluations.push_back(evaluation);
  }
  return request;
}

} // namespace

TEST(CanonicalRetainedRevalidation, OvertakeProvenanceRequiresTargetIdentity) {
  auto current = make_current();
  current.intent = contract::ControlIntent::Pass;
  EXPECT_FALSE(retained::current_execution_provenance_complete(current));

  current.target_id = "d2";
  current.target_obstacle_generation = current.observation_generation;
  current.execution_side_sign = 1;
  EXPECT_TRUE(retained::current_execution_provenance_complete(current));

  current.execution_side_sign = 0;
  EXPECT_FALSE(retained::current_execution_provenance_complete(current));
  current.execution_side_sign = 1;

  current.target_obstacle_generation = 0U;
  EXPECT_FALSE(retained::current_execution_provenance_complete(current));
}

TEST(CanonicalRetainedRevalidation, RejectsCrossSideBeforePhysicalProof)
{
  auto execution_plan = make_plan();
  execution_plan.problem.intent = contract::ControlIntent::Pass;
  execution_plan.problem.target_id = "d2";
  execution_plan.problem.target_obstacle_generation = 7U;
  execution_plan.problem.execution_side_sign = 1;
  execution_plan.problem = contract::seal_problem_context(
    std::move(execution_plan.problem));
  execution_plan.solution.problem_fingerprint =
    execution_plan.problem.fingerprint;

  auto current = make_current();
  current.intent = contract::ControlIntent::Pass;
  current.target_id = "d2";
  current.target_obstacle_generation = 8U;
  current.execution_side_sign = -1;
  ASSERT_TRUE(retained::current_execution_provenance_complete(current));
  EXPECT_EQ(
    retained::validate_retained_semantic_identity(execution_plan, current),
    retained::RetainedExecutionProofReason::ExecutionSideMismatch);
}

TEST(
  CanonicalRetainedRevalidation,
  PartialFirstStageUsesRemainingTimeAndNextState) {
  const auto execution_plan = make_plan();
  const auto cursor = plan::resolve_execution_cursor(execution_plan, 10.6);
  ASSERT_TRUE(cursor.available);
  const auto result =
    retained::build_retained_execution_window(execution_plan, cursor);
  ASSERT_EQ(result.reason, retained::RetainedExecutionWindowReason::Accepted)
    << result.detail;
  ASSERT_TRUE(result.window.has_value());
  ASSERT_EQ(result.window->samples.size(), 2U);
  EXPECT_NEAR(result.window->expected_current_progress_m, 100.30, 1e-12);
  EXPECT_NEAR(result.window->expected_current_state.lateral_m, 0.112, 1e-12);
  EXPECT_NEAR(result.window->expected_current_state.lag_m, 0.016, 1e-12);
  EXPECT_NEAR(
    result.window->expected_current_state.heading_offset_rad, 0.026, 1e-12);
  EXPECT_NEAR(result.window->expected_current_state.velocity_mps, 5.12, 1e-12);
  EXPECT_EQ(result.window->samples[0].control_stage_index, 0U);
  EXPECT_EQ(result.window->samples[0].endpoint_state_index, 1U);
  EXPECT_NEAR(result.window->samples[0].segment_duration_sec, 0.4, 1e-12);
  EXPECT_NEAR(result.window->samples[0].relative_time_sec, 0.4, 1e-12);
  EXPECT_DOUBLE_EQ(result.window->samples[0].absolute_progress_m, 100.5);
  EXPECT_EQ(result.window->samples[1].endpoint_state_index, 2U);
  EXPECT_NEAR(result.window->samples[1].relative_time_sec, 1.4, 1e-12);
}

TEST(
  CanonicalRetainedRevalidation,
  CertifiedNumericalProgressResidualRemainsExecutableAndObservable)
{
  auto execution_plan = make_plan();
  execution_plan.solution.maximum_constraint_violation = 1e-5;
  execution_plan.predicted_states[1].progress_m =
    execution_plan.predicted_states[0].progress_m - 2e-6;
  const auto cursor = plan::resolve_execution_cursor(execution_plan, 10.0);
  ASSERT_TRUE(cursor.available);

  const auto result =
    retained::build_retained_execution_window(execution_plan, cursor);

  EXPECT_EQ(
    result.reason,
    retained::RetainedExecutionWindowReason::Accepted);
  ASSERT_TRUE(result.window.has_value());
  EXPECT_DOUBLE_EQ(result.window->progress_evolution_tolerance_m, 1e-5);
  const auto advance = retained::sample_retained_progress_advance(
    result.window.value(), {0.0, 0.5, 1.0});
  ASSERT_TRUE(advance.has_value());
  EXPECT_DOUBLE_EQ((*advance)[0], 0.0);
  EXPECT_DOUBLE_EQ((*advance)[1], 0.0);
  EXPECT_DOUBLE_EQ((*advance)[2], 0.0);
}

TEST(
  CanonicalRetainedRevalidation,
  ProgressBeyondTheCertifiedResidualReportsTheExactPlanDelta)
{
  auto execution_plan = make_plan();
  execution_plan.solution.maximum_constraint_violation = 1e-5;
  execution_plan.predicted_states[1].progress_m =
    execution_plan.predicted_states[0].progress_m - 2e-4;
  const auto cursor = plan::resolve_execution_cursor(execution_plan, 10.0);
  ASSERT_TRUE(cursor.available);

  const auto result =
    retained::build_retained_execution_window(execution_plan, cursor);

  EXPECT_EQ(
    result.reason,
    retained::RetainedExecutionWindowReason::InvalidProgressEvolution);
  EXPECT_FALSE(result.window.has_value());
  EXPECT_NE(result.detail.find("stage=0"), std::string::npos);
  EXPECT_NE(result.detail.find("delta=-0.0002"), std::string::npos);
  EXPECT_NE(
    result.detail.find("certified_max_violation=1e-05"),
    std::string::npos);
}

TEST(
  CanonicalRetainedRevalidation,
  SamplesSelectedPlanProgressInsteadOfWaypointSpacing) {
  const auto execution_plan = make_plan();
  const auto cursor = plan::resolve_execution_cursor(execution_plan, 10.6);
  const auto window =
    retained::build_retained_execution_window(execution_plan, cursor);
  ASSERT_TRUE(window.window.has_value());

  const auto advance = retained::sample_retained_progress_advance(
    window.window.value(), {0.0, 0.2, 0.4, 0.9, 1.4, 2.0});
  ASSERT_TRUE(advance.has_value());
  ASSERT_EQ(advance->size(), 6U);
  EXPECT_NEAR((*advance)[0], 0.0, 1e-12);
  EXPECT_NEAR((*advance)[1], 0.1, 1e-12);
  EXPECT_NEAR((*advance)[2], 0.2, 1e-12);
  EXPECT_NEAR((*advance)[3], 0.45, 1e-12);
  EXPECT_NEAR((*advance)[4], 0.7, 1e-12);
  // Current proof may have a longer base corridor than the remaining plan.
  // The unused tail must hold the final selected-plan progress, not invent
  // additional waypoint-distance motion.
  EXPECT_NEAR((*advance)[5], 0.7, 1e-12);
}

TEST(
  CanonicalRetainedRevalidation,
  SlicesTheSelectedPlansOwnLateralCorridorAtTheCursor)
{
  auto execution_plan = make_plan();
  execution_plan.lateral_lower_m = {-0.50, -0.40, -0.30};
  execution_plan.lateral_upper_m = {0.50, 0.45, 0.40};
  const auto cursor = plan::resolve_execution_cursor(execution_plan, 10.6);
  const auto window =
    retained::build_retained_execution_window(execution_plan, cursor);
  ASSERT_TRUE(window.window.has_value());

  const auto corridor = retained::build_retained_lateral_corridor(
    execution_plan, window.window.value());
  ASSERT_TRUE(corridor.has_value());
  ASSERT_EQ(corridor->relative_time_sec.size(), 3U);
  EXPECT_NEAR(corridor->relative_time_sec[0], 0.0, 1e-12);
  EXPECT_NEAR(corridor->relative_time_sec[1], 0.4, 1e-12);
  EXPECT_NEAR(corridor->relative_time_sec[2], 1.4, 1e-12);
  EXPECT_NEAR(corridor->lower_m[0], -0.44, 1e-12);
  EXPECT_NEAR(corridor->upper_m[0], 0.47, 1e-12);
  EXPECT_NEAR(corridor->lower_m[1], -0.40, 1e-12);
  EXPECT_NEAR(corridor->upper_m[2], 0.40, 1e-12);
}

TEST(
  CanonicalRetainedRevalidation,
  CourseFrameRangeIncludesRetainedStateBehindMeasuredOrigin) {
  const auto execution_plan = make_plan();
  const auto cursor = plan::resolve_execution_cursor(execution_plan, 10.6);
  const auto window =
    retained::build_retained_execution_window(execution_plan, cursor);
  ASSERT_TRUE(window.window.has_value());

  // The plant has advanced six centimetres beyond the retained model state.
  // This remains a continuity-valid plan, so current-world proof needs course
  // geometry behind the newly measured origin as well as ahead of it.
  const auto range = retained::required_course_frame_progress_range(
    window.window.value(), 100.36);
  ASSERT_TRUE(range.has_value());
  EXPECT_NEAR(range->minimum_progress_m, 100.30, 1e-12);
  EXPECT_NEAR(range->maximum_progress_m, 101.00, 1e-12);
}

TEST(
  CanonicalRetainedRevalidation,
  CourseFrameRangeRejectsBrokenRetainedProgressChain) {
  const auto execution_plan = make_plan();
  const auto cursor = plan::resolve_execution_cursor(execution_plan, 10.6);
  auto window = retained::build_retained_execution_window(
    execution_plan, cursor).window;
  ASSERT_TRUE(window.has_value());
  ASSERT_FALSE(window->samples.empty());
  window->samples.front().segment_start_progress_m += 0.01;

  EXPECT_FALSE(retained::sample_retained_progress_advance(
    window.value(), {0.0, 0.2}).has_value());
  EXPECT_FALSE(retained::required_course_frame_progress_range(
    window.value(), 100.36).has_value());
}

TEST(CanonicalRetainedRevalidation, CircularProgressLiftIsExplicitAndUnique) {
  const auto accepted = retained::lift_progress_to_retained_branch(
    retained::CircularProgressLiftRequest{0.30, 100.30, 100.0, 0.20, true});
  EXPECT_EQ(accepted.reason, retained::CircularProgressLiftReason::Accepted);
  EXPECT_NEAR(accepted.lifted_progress_m, 100.30, 1e-12);
  EXPECT_EQ(accepted.lap_offset, 1L);

  const auto discontinuous = retained::lift_progress_to_retained_branch(
    retained::CircularProgressLiftRequest{1.0, 100.30, 100.0, 0.20, true});
  EXPECT_EQ(
    discontinuous.reason,
    retained::CircularProgressLiftReason::Discontinuous);

  const auto ambiguous = retained::lift_progress_to_retained_branch(
    retained::CircularProgressLiftRequest{0.30, 100.30, 100.0, 50.0, true});
  EXPECT_EQ(
    ambiguous.reason,
    retained::CircularProgressLiftReason::AmbiguousBranch);
}

TEST(CanonicalRetainedRevalidation, BuildsSealedCurrentObservationProof) {
  const auto execution_plan = make_plan();
  const auto cursor = plan::resolve_execution_cursor(execution_plan, 10.6);
  const auto request = make_request(execution_plan, cursor);
  const auto result =
    retained::build_retained_execution_proof(execution_plan, cursor, request);
  ASSERT_EQ(result.reason, retained::RetainedExecutionProofReason::Accepted);
  ASSERT_TRUE(result.proof.has_value());
  EXPECT_NE(result.proof->proof_fingerprint, 0U);

  const auto candidate = retained::build_canonical_retained_candidate(
    execution_plan, cursor, request.current, result.proof.value());
  ASSERT_EQ(candidate.reason, retained::RetainedCandidateBuildReason::Accepted);
  ASSERT_TRUE(candidate.candidate.has_value());
  EXPECT_EQ(candidate.candidate->execution_certificate_decision_id, 43U);
  EXPECT_EQ(candidate.candidate->execution_first_control_stage_index, 0U);

  const auto resolution = contract::resolve_canonical_normal_authority(
    contract::CanonicalNormalAuthorityRequest{
    43U, 10.6, contract::CanonicalNormalCandidate{},
    candidate.candidate.value(), contract::ControlIntent::Track});
  EXPECT_EQ(
    resolution.source,
    contract::CanonicalNormalAuthoritySource::RetainedCertified);
  EXPECT_TRUE(resolution.retained_solution);
}

TEST(CanonicalRetainedRevalidation, RejectsStageIndexAliasByProgressIdentity) {
  const auto execution_plan = make_plan();
  const auto cursor = plan::resolve_execution_cursor(execution_plan, 10.6);
  auto request = make_request(execution_plan, cursor);
  request.stage_evaluations[0].absolute_progress_m = 109.0;
  EXPECT_EQ(
    retained::build_retained_execution_proof(execution_plan, cursor, request)
    .reason,
    retained::RetainedExecutionProofReason::StageEvaluationIdentityMismatch);
}

TEST(CanonicalRetainedRevalidation, RejectsChangedCurrentProvenance) {
  const auto execution_plan = make_plan();
  const auto cursor = plan::resolve_execution_cursor(execution_plan, 10.6);
  const auto request = make_request(execution_plan, cursor);
  const auto built =
    retained::build_retained_execution_proof(execution_plan, cursor, request);
  ASSERT_TRUE(built.proof.has_value());

  auto changed = request.current;
  ++changed.observation_generation;
  EXPECT_EQ(
    retained::build_canonical_retained_candidate(
      execution_plan, cursor, changed, built.proof.value())
    .proof_reason,
    retained::RetainedExecutionProofReason::FingerprintMismatch);

  changed = request.current;
  ++changed.stage_geometry_id;
  EXPECT_EQ(
    retained::build_canonical_retained_candidate(
      execution_plan, cursor, changed, built.proof.value())
    .proof_reason,
    retained::RetainedExecutionProofReason::FingerprintMismatch);

  changed = request.current;
  changed.target_id = "d2";
  changed.target_obstacle_generation = 99U;
  EXPECT_EQ(
    retained::build_canonical_retained_candidate(
      execution_plan, cursor, changed, built.proof.value())
    .proof_reason,
    retained::RetainedExecutionProofReason::FingerprintMismatch);
}

TEST(CanonicalRetainedRevalidation, RejectsBlockedPrefixesAndMovingObstacle) {
  const auto execution_plan = make_plan();
  const auto cursor = plan::resolve_execution_cursor(execution_plan, 10.6);

  auto request = make_request(execution_plan, cursor);
  request.measured_to_control_prefix.wall_clear = false;
  EXPECT_EQ(
    retained::build_retained_execution_proof(execution_plan, cursor, request)
    .reason,
    retained::RetainedExecutionProofReason::DelayPrefixRejected);

  request = make_request(execution_plan, cursor);
  request.control_to_retained_connector.wall_clear = false;
  EXPECT_EQ(
    retained::build_retained_execution_proof(execution_plan, cursor, request)
    .reason,
    retained::RetainedExecutionProofReason::ConnectorRejected);

  request = make_request(execution_plan, cursor);
  request.stage_evaluations[0].obstacles_clear = false;
  EXPECT_EQ(
    retained::build_retained_execution_proof(execution_plan, cursor, request)
    .reason,
    retained::RetainedExecutionProofReason::ObstacleRejected);
}

TEST(CanonicalRetainedRevalidation, RejectsConnectorWithoutRetainedEndpointIdentity) {
  const auto execution_plan = make_plan();
  const auto cursor = plan::resolve_execution_cursor(execution_plan, 10.6);
  auto request = make_request(execution_plan, cursor);
  request.measured_course_progress_m = 0.25;
  request.measured_to_control_prefix.start_progress_m = 100.25;
  request.measured_to_control_prefix.end_progress_m = 100.25;
  request.control_to_retained_connector.start_progress_m = 100.25;
  request.control_to_retained_connector.end_progress_m =
    request.control_to_retained_connector.start_progress_m;

  EXPECT_EQ(
    retained::build_retained_execution_proof(execution_plan, cursor, request)
    .reason,
    retained::RetainedExecutionProofReason::PrefixIdentityMismatch);
}

TEST(CanonicalRetainedRevalidation, RejectsMissingInputAndFingerprintMutation) {
  const auto execution_plan = make_plan();
  const auto cursor = plan::resolve_execution_cursor(execution_plan, 10.6);

  auto request = make_request(execution_plan, cursor);
  request.current.control_pose_id = 0U;
  EXPECT_EQ(
    retained::build_retained_execution_proof(execution_plan, cursor, request)
    .reason,
    retained::RetainedExecutionProofReason::InvalidCurrentProvenance);

  request = make_request(execution_plan, cursor);
  request.current.obstacle_tube_id = 0U;
  EXPECT_EQ(
    retained::build_retained_execution_proof(execution_plan, cursor, request)
    .reason,
    retained::RetainedExecutionProofReason::InvalidCurrentProvenance);

  request = make_request(execution_plan, cursor);
  const auto built =
    retained::build_retained_execution_proof(execution_plan, cursor, request);
  ASSERT_TRUE(built.proof.has_value());
  auto mutated = built.proof.value();
  mutated.stage_evaluations[0].absolute_progress_m += 0.1;
  EXPECT_EQ(
    retained::build_canonical_retained_candidate(
      execution_plan, cursor, request.current, mutated)
    .proof_reason,
    retained::RetainedExecutionProofReason::FingerprintMismatch);
}
