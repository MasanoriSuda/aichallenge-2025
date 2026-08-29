#include "multi_purpose_mpc_ros/mpcc_rate_resolved_physical_adapter.hpp"

#include <gtest/gtest.h>

namespace adapter =
  multi_purpose_mpc_ros::mpcc_rate_resolved_physical_adapter;
namespace execution =
  multi_purpose_mpc_ros::mpcc_rate_resolved_execution_artifact;
namespace contract = multi_purpose_mpc_ros::mpcc_execution_contract;

namespace
{

contract::MpccProblemContext source_context()
{
  contract::MpccProblemContext context;
  context.decision_id = 2U;
  context.intent = contract::ControlIntent::Track;
  context.intent_generation = 1U;
  context.observation_generation = 2U;
  context.stage_geometry_id = 4U;
  context.horizon_steps = 2U;
  context.formulation =
    contract::Formulation::VelocitySteeringYawResponseProgress7State;
  context.state_schema_id = "ey-elag-epsi-v-progress-steering-v1";
  context.input_schema_id = "accel-steering-rate-progress-rate-v1";
  context.bounds_schema_id = "stage-wall-v1";
  context.cost_schema_id = "velocity-progress-steering-rate-v1";
  return contract::seal_problem_context(std::move(context));
}

execution::ExecutionArtifact artifact()
{
  execution::ExecutionArtifact value;
  value.identity = execution::Identity{1U, source_context(), 10.0};
  value.prediction_origin_sec = 10.0;
  value.publication_interval_sec = 0.025;
  value.completed_sec = 10.01;
  value.course_progress_origin_m = 50.0;
  value.semantic_initial_steering_rad = 0.10;
  value.semantic_initial_response_steering_rad = 0.10;
  value.wheelbase_m = 2.0;
  value.maximum_abs_steering_rad = 0.60;
  value.maximum_abs_steering_rate_radps = 1.0;
  value.physical_global_tolerance = 1e-6;
  value.maximum_constraint_violation = 1e-8;
  value.maximum_normalized_constraint_violation = 0.1;
  value.predicted_states = {
    {0.0, 0.1, 0.0, 2.0, 0.0, 0.10, 0.10},
    {0.1, 0.0, 0.01, 2.1, 0.2, 0.11, 0.10302380180000528},
    {0.2, 0.0, 0.02, 2.2, 0.4, 0.12, 0.10979124524044208},
  };
  value.control_stages = {
    {1.0, 0.10, 2.0, 0.10, 0.0, 4.0, -3.0, 1.37},
    {1.0, 0.10, 2.0, 0.10, 0.0, 4.0, -3.0, 1.37},
  };
  value.nominal_path_distance_m = {0.0, 0.2, 0.4};
  value.lateral_lower_m = {-1.0, -1.0, -1.0};
  value.lateral_upper_m = {1.0, 1.0, 1.0};
  return value;
}

}  // namespace

TEST(MpccRateResolvedPhysicalAdapter, ReplaysControlsThroughNonlinearModel)
{
  const auto source = artifact();
  const auto result = adapter::build(
    source, contract::ControlIntent::Track,
    source.identity.source_context.stage_geometry_id);
  ASSERT_EQ(result.reason, adapter::RejectReason::None);
  ASSERT_TRUE(result.exact_trajectory.has_value());
  const auto & exact = result.exact_trajectory.value();
  ASSERT_GT(exact.lateral_m.size(), 2U);
  EXPECT_DOUBLE_EQ(exact.progress_origin_m, 50.0);
  EXPECT_GT(exact.path_distance_m.front(), 0.0);
  EXPECT_DOUBLE_EQ(exact.path_distance_m.back(), 0.4);
  EXPECT_NEAR(exact.progress_m.back(), 50.4, 1e-12);
  EXPECT_NEAR(exact.velocity_mps.back(), 2.2, 1e-12);
  EXPECT_GT(exact.minimum_lateral_bound_reserve_m, 0.0);
  EXPECT_LT(exact.minimum_lateral_bound_reserve_m, 1.0);
  EXPECT_DOUBLE_EQ(
    exact.lateral_bound_tolerance_m,
    execution::physical_lateral_bound_tolerance_m(source));
  EXPECT_EQ(result.minimum_progress_transition_state, 1);
  EXPECT_DOUBLE_EQ(result.minimum_progress_delta_m, 0.2);
  EXPECT_DOUBLE_EQ(result.transition_virtual_progress_speed_mps, 2.0);
  EXPECT_DOUBLE_EQ(result.transition_duration_sec, 0.10);
  EXPECT_NEAR(result.progress_dynamics_defect_m, 0.0, 1e-12);
}

TEST(
  MpccRateResolvedPhysicalAdapter,
  MixedUnitGlobalToleranceCannotMaskLateralCorridorViolation)
{
  auto source = artifact();
  source.physical_global_tolerance = 0.10;
  source.maximum_constraint_violation = 1e-8;
  auto result = adapter::build(
    source, contract::ControlIntent::Track,
    source.identity.source_context.stage_geometry_id);
  ASSERT_EQ(result.reason, adapter::RejectReason::None);
  ASSERT_TRUE(result.exact_trajectory.has_value());
  EXPECT_DOUBLE_EQ(
    result.exact_trajectory->lateral_bound_tolerance_m, 1e-5);

  source.lateral_upper_m[1] =
    source.predicted_states[1].lateral_m - 1e-4;

  EXPECT_DOUBLE_EQ(
    execution::physical_lateral_bound_tolerance_m(source), 1e-5);
  result = adapter::build(
    source, contract::ControlIntent::Track,
    source.identity.source_context.stage_geometry_id);
  EXPECT_EQ(result.reason, adapter::RejectReason::InvalidArtifact);
  EXPECT_EQ(
    result.artifact_reason, execution::RejectReason::InvalidLateralCorridor);

  source = artifact();
  source.maximum_constraint_violation = 2e-5;
  EXPECT_NEAR(
    execution::physical_lateral_bound_tolerance_m(source), 2.1e-5, 1e-15);
}

TEST(
  MpccRateResolvedPhysicalAdapter,
  ReplaysPartialSuffixFromCurrentPhysicalState)
{
  const auto source = artifact();
  const auto cursor = execution::resolve_cursor(source, 10.05);
  ASSERT_TRUE(cursor.available);

  const auto result = adapter::build_continuation(
    source, cursor,
    adapter::ContinuationInitialState{
      -0.60, 0.0, 0.0, 2.05, 0.10, 0.105, 0.105});

  ASSERT_EQ(result.reason, adapter::ContinuationRejectReason::None);
  ASSERT_TRUE(result.exact_trajectory.has_value());
  const auto & exact = result.exact_trajectory.value();
  EXPECT_LT(exact.lateral_m.front(), -0.59);
  EXPECT_NEAR(exact.elapsed_time_sec.back(), 0.15, 1e-12);
  EXPECT_NEAR(exact.path_distance_m.back(), 0.40, 1e-12);
  EXPECT_NEAR(exact.progress_m.back(), 50.40, 1e-12);
  ASSERT_EQ(result.stage_end_velocity_mps.size(), 2U);
  ASSERT_EQ(result.stage_end_steering_rad.size(), 2U);
  EXPECT_NEAR(result.stage_end_velocity_mps.back(), 2.20, 1e-12);
  EXPECT_NEAR(result.stage_end_steering_rad.back(), 0.12, 1e-12);
}

TEST(
  MpccRateResolvedPhysicalAdapter,
  CertifiesTheLastExecutableEndpointNearArtifactExhaustion)
{
  const auto source = artifact();
  const auto cursor = execution::resolve_cursor(source, 10.195);
  ASSERT_TRUE(cursor.available);
  ASSERT_EQ(cursor.remaining_control_stage_count, 1U);

  const auto result = adapter::build_continuation(
    source, cursor,
    adapter::ContinuationInitialState{
      0.19, 0.0, 0.019, 2.195, 0.39, 0.1195, 0.109});

  ASSERT_EQ(result.reason, adapter::ContinuationRejectReason::None);
  ASSERT_TRUE(result.exact_trajectory.has_value());
  EXPECT_EQ(result.exact_trajectory->elapsed_time_sec.size(), 1U);
  EXPECT_EQ(result.stage_end_velocity_mps.size(), 1U);
  EXPECT_EQ(result.stage_end_steering_rad.size(), 1U);
}

TEST(
  MpccRateResolvedPhysicalAdapter,
  BuildsPublisherPeriodThenMaximumBrakingStopFromCurrentState)
{
  const auto source = artifact();
  const auto cursor = execution::resolve_cursor(source, 10.05);
  ASSERT_TRUE(cursor.available);
  const auto actuation = execution::extract_actuation(source, cursor);
  ASSERT_TRUE(actuation.actuation.has_value());

  const auto result = adapter::build_stop_contingency(
    source, cursor, actuation.actuation.value(),
    adapter::ContinuationInitialState{
      -0.60, 0.0, 0.0, 2.05, 0.10, 0.105, 0.105},
    -3.0);

  ASSERT_EQ(result.reason, adapter::StopContingencyRejectReason::None);
  ASSERT_TRUE(result.exact_trajectory.has_value());
  const auto & exact = result.exact_trajectory.value();
  ASSERT_FALSE(exact.velocity_mps.empty());
  EXPECT_NEAR(exact.velocity_mps.back(), 0.0, 1e-9);
  EXPECT_GT(exact.elapsed_time_sec.back(), source.publication_interval_sec);
  EXPECT_NEAR(
    exact.velocity_mps.front(),
    2.05 + actuation.actuation->acceleration_mps2 * 0.01, 1e-9);
  EXPECT_TRUE(
    multi_purpose_mpc_ros::race_mpcc_foundation::
    exact_physical_execution_trajectory_complete(exact));
}

TEST(
  MpccRateResolvedPhysicalAdapter,
  RejectsStopWhenConfiguredBrakingExceedsPhysicalEnvelope)
{
  const auto source = artifact();
  const auto cursor = execution::resolve_cursor(source, 10.05);
  ASSERT_TRUE(cursor.available);
  const auto actuation = execution::extract_actuation(source, cursor);
  ASSERT_TRUE(actuation.actuation.has_value());

  const auto result = adapter::build_stop_contingency(
    source, cursor, actuation.actuation.value(),
    adapter::ContinuationInitialState{
      -0.60, 0.0, 0.0, 2.05, 0.10, 0.105, 0.105},
    -4.0);

  EXPECT_EQ(
    result.reason,
    adapter::StopContingencyRejectReason::InvalidBrakingEnvelope);
  EXPECT_FALSE(result.exact_trajectory.has_value());
}

TEST(
  MpccRateResolvedPhysicalAdapter,
  RejectsContinuationOutsideCertifiedLateralCorridor)
{
  const auto source = artifact();
  const auto cursor = execution::resolve_cursor(source, 10.05);
  ASSERT_TRUE(cursor.available);

  const auto result = adapter::build_continuation(
    source, cursor,
    adapter::ContinuationInitialState{
      1.10, 0.0, 0.0, 2.05, 0.10, 0.105, 0.105});

  EXPECT_EQ(
    result.reason,
    adapter::ContinuationRejectReason::InitialLateralBoundRejected);
  EXPECT_FALSE(result.exact_trajectory.has_value());
}

TEST(
  MpccRateResolvedPhysicalAdapter,
  RetainsCurrentStageWhenOnlyLaterContinuationLeavesCorridor)
{
  auto source = artifact();
  source.semantic_initial_steering_rad = 0.35;
  source.semantic_initial_response_steering_rad = 0.35;
  source.predicted_states = {
    {0.0, 0.0, 0.0, 8.0, 0.0, 0.35, 0.35},
    {0.0, 0.0, 0.0, 8.0, 0.4, 0.35, 0.35},
    {0.0, 0.0, 0.0, 8.0, 0.8, 0.35, 0.35},
  };
  source.control_stages = {
    {0.0, 0.0, 8.0, 0.05, 0.0, 10.0, -3.0, 1.37},
    {0.0, 0.0, 8.0, 0.05, 0.0, 10.0, -3.0, 1.37},
  };
  source.nominal_path_distance_m = {0.0, 0.4, 0.8};
  source.lateral_lower_m = {-0.03, -0.03, -0.03};
  source.lateral_upper_m = {0.03, 0.03, 0.03};
  const auto cursor = execution::resolve_cursor(source, 10.001);
  ASSERT_TRUE(cursor.available);

  const auto result = adapter::build_continuation(
    source, cursor,
    adapter::ContinuationInitialState{
      0.0, 0.0, 0.0, 8.0, 0.0, 0.35, 0.35});

  ASSERT_EQ(result.reason, adapter::ContinuationRejectReason::None);
  ASSERT_TRUE(result.exact_trajectory.has_value());
  EXPECT_EQ(
    result.scope, adapter::ContinuationProofScope::CurrentStagePrefix);
  EXPECT_EQ(result.stage_end_velocity_mps.size(), 1U);
  EXPECT_EQ(result.stage_end_steering_rad.size(), 1U);
  EXPECT_LT(result.exact_trajectory->elapsed_time_sec.back(), 0.05);
  EXPECT_LE(result.exact_trajectory->lateral_m.back(), 0.03);
}

TEST(MpccRateResolvedPhysicalAdapter, RejectsLinearizedStatesThatHideNonlinearWallDeparture)
{
  auto source = artifact();
  source.semantic_initial_steering_rad = 0.35;
  source.semantic_initial_response_steering_rad = 0.35;
  source.predicted_states = {
    {0.0, 0.0, 0.0, 8.0, 0.0, 0.35, 0.35},
    {0.0, 0.0, 0.0, 8.0, 4.0, 0.35, 0.35},
    {0.0, 0.0, 0.0, 8.0, 8.0, 0.35, 0.35},
  };
  source.control_stages = {
    {0.0, 0.0, 8.0, 0.50, 0.0, 10.0, -3.0, 1.37},
    {0.0, 0.0, 8.0, 0.50, 0.0, 10.0, -3.0, 1.37},
  };
  source.nominal_path_distance_m = {0.0, 4.0, 8.0};
  source.lateral_lower_m = {-0.20, -0.20, -0.20};
  source.lateral_upper_m = {0.20, 0.20, 0.20};

  const auto result = adapter::build(
    source, contract::ControlIntent::Track,
    source.identity.source_context.stage_geometry_id);

  EXPECT_EQ(result.reason, adapter::RejectReason::ExactTrajectoryRejected);
  EXPECT_FALSE(result.exact_trajectory.has_value());
  EXPECT_GT(result.rejected_lateral_m, result.rejected_lateral_upper_m);
  EXPECT_DOUBLE_EQ(result.rejected_lateral_lower_m, -0.20);
  EXPECT_DOUBLE_EQ(result.rejected_lateral_upper_m, 0.20);
}

TEST(MpccRateResolvedPhysicalAdapter, RejectsCurrentSemanticMismatch)
{
  const auto source = artifact();
  EXPECT_EQ(
    adapter::build(
      source, contract::ControlIntent::Cruise,
      source.identity.source_context.stage_geometry_id).reason,
    adapter::RejectReason::IntentMismatch);
  EXPECT_EQ(
    adapter::build(
      source, contract::ControlIntent::Track,
      source.identity.source_context.stage_geometry_id + 1U).reason,
    adapter::RejectReason::StageGeometryMismatch);
}

TEST(MpccRateResolvedPhysicalAdapter, RejectsInvalidArtifactBeforeConversion)
{
  auto source = artifact();
  source.nominal_path_distance_m[1] = 0.0;
  const auto result = adapter::build(
    source, contract::ControlIntent::Track,
    source.identity.source_context.stage_geometry_id);
  EXPECT_EQ(result.reason, adapter::RejectReason::InvalidArtifact);
  EXPECT_EQ(
    result.artifact_reason, execution::RejectReason::InvalidPathDistance);
  EXPECT_FALSE(result.exact_trajectory.has_value());
}

TEST(MpccRateResolvedPhysicalAdapter, AppliesCertifiedToleranceToInternalProgressInput)
{
  auto source = artifact();
  source.control_stages.front().acceleration_mps2 = 1.3700001;
  auto result = adapter::build(
    source, contract::ControlIntent::Track,
    source.identity.source_context.stage_geometry_id);
  EXPECT_EQ(result.reason, adapter::RejectReason::InvalidArtifact);
  EXPECT_EQ(
    result.artifact_reason,
    execution::RejectReason::InvalidAccelerationControlBounds);

  source = artifact();
  source.control_stages.front().virtual_progress_speed_mps = -5e-7;
  source.control_stages.front().virtual_progress_lower_mps = 0.0;
  source.control_stages.front().virtual_progress_upper_mps = 0.0;
  source.predicted_states[1].progress_m = -5e-8;
  source.predicted_states[2].progress_m = 0.19999995;
  source.maximum_constraint_violation = 5e-7;
  result = adapter::build(
    source, contract::ControlIntent::Track,
    source.identity.source_context.stage_geometry_id);
  EXPECT_EQ(result.reason, adapter::RejectReason::None);
  EXPECT_TRUE(result.exact_trajectory.has_value());

  source = artifact();
  source.control_stages.front().virtual_progress_speed_mps = -2e-6;
  source.control_stages.front().virtual_progress_lower_mps = 0.0;
  source.control_stages.front().virtual_progress_upper_mps = 0.0;
  source.predicted_states[1].progress_m = -2e-7;
  source.predicted_states[2].progress_m = 0.1999998;
  source.maximum_constraint_violation = 2e-6;
  result = adapter::build(
    source, contract::ControlIntent::Track,
    source.identity.source_context.stage_geometry_id);
  EXPECT_EQ(result.reason, adapter::RejectReason::InvalidArtifact);
  EXPECT_EQ(
    result.artifact_reason, execution::RejectReason::InvalidControlStage);
}

TEST(MpccRateResolvedPhysicalAdapter, DoesNotUseAffineVelocityAsPhysicalRollout)
{
  auto source = artifact();
  source.predicted_states[1].velocity_mps = -5e-7;
  source.maximum_constraint_violation = 5e-7;
  auto result = adapter::build(
    source, contract::ControlIntent::Track,
    source.identity.source_context.stage_geometry_id);
  ASSERT_EQ(result.reason, adapter::RejectReason::None);
  ASSERT_TRUE(result.exact_trajectory.has_value());
  EXPECT_NEAR(
    result.exact_trajectory->velocity_mps.front(), 2.01, 1e-12);
  EXPECT_GT(
    result.exact_trajectory->velocity_lower_bound_tolerance_mps, 5e-7);

  source = artifact();
  source.predicted_states[1].velocity_mps = -2e-6;
  source.maximum_constraint_violation = 5e-7;
  result = adapter::build(
    source, contract::ControlIntent::Track,
    source.identity.source_context.stage_geometry_id);
  EXPECT_EQ(result.reason, adapter::RejectReason::InvalidArtifact);
  EXPECT_EQ(
    result.artifact_reason, execution::RejectReason::InvalidPredictedState);
}

TEST(MpccRateResolvedPhysicalAdapter, AcceptsCertifiedProgressRegression)
{
  auto source = artifact();
  source.predicted_states[2].progress_m = 0.19999;
  source.control_stages[1].virtual_progress_speed_mps = 0.0;
  source.maximum_constraint_violation = 1.0e-5;
  const auto result = adapter::build(
    source, contract::ControlIntent::Track,
    source.identity.source_context.stage_geometry_id);
  EXPECT_EQ(result.reason, adapter::RejectReason::None);
  ASSERT_TRUE(result.exact_trajectory.has_value());
  EXPECT_GT(result.certified_progress_regression_tolerance_m, 1.0e-5);
  EXPECT_EQ(
    multi_purpose_mpc_ros::race_mpcc_foundation::
    validate_exact_physical_execution_trajectory(
      result.exact_trajectory.value()).reason,
    multi_purpose_mpc_ros::race_mpcc_foundation::
    ExactPhysicalExecutionTrajectoryReason::Accepted);
  EXPECT_EQ(result.minimum_progress_transition_state, 2);
  EXPECT_NEAR(result.minimum_progress_delta_m, -1.0e-5, 1e-12);
  EXPECT_DOUBLE_EQ(result.transition_virtual_progress_speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(result.transition_duration_sec, 0.10);
  EXPECT_NEAR(result.progress_dynamics_defect_m, -1.0e-5, 1e-12);
}

TEST(MpccRateResolvedPhysicalAdapter, RejectsProgressOutsideSolverCertificate)
{
  auto source = artifact();
  source.predicted_states[2].progress_m = 0.10;
  source.control_stages[1].virtual_progress_speed_mps = 0.0;
  const auto result = adapter::build(
    source, contract::ControlIntent::Track,
    source.identity.source_context.stage_geometry_id);
  EXPECT_EQ(result.reason, adapter::RejectReason::InvalidArtifact);
  EXPECT_EQ(
    result.artifact_reason,
    execution::RejectReason::ProgressDynamicsMismatch);
}
