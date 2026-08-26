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
    contract::Formulation::VelocitySteeringProgress6State;
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
  value.wheelbase_m = 2.0;
  value.maximum_abs_steering_rad = 0.60;
  value.maximum_abs_steering_rate_radps = 1.0;
  value.physical_global_tolerance = 1e-6;
  value.maximum_constraint_violation = 1e-8;
  value.maximum_normalized_constraint_violation = 0.1;
  value.predicted_states = {
    {0.0, 0.1, 0.0, 2.0, 0.0, 0.10},
    {0.1, 0.0, 0.01, 2.1, 0.2, 0.11},
    {0.2, 0.0, 0.02, 2.2, 0.4, 0.12},
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

TEST(MpccRateResolvedPhysicalAdapter, PreservesExactStatesOneThroughHorizon)
{
  const auto source = artifact();
  const auto result = adapter::build(
    source, contract::ControlIntent::Track,
    source.identity.source_context.stage_geometry_id);
  ASSERT_EQ(result.reason, adapter::RejectReason::None);
  ASSERT_TRUE(result.exact_trajectory.has_value());
  const auto & exact = result.exact_trajectory.value();
  ASSERT_EQ(exact.lateral_m.size(), 2U);
  EXPECT_DOUBLE_EQ(exact.progress_origin_m, 50.0);
  EXPECT_DOUBLE_EQ(exact.path_distance_m[0], 0.2);
  EXPECT_DOUBLE_EQ(exact.progress_m[0], 50.2);
  EXPECT_DOUBLE_EQ(exact.lag_m[1], 0.0);
  EXPECT_DOUBLE_EQ(exact.heading_offset_rad[1], 0.02);
  EXPECT_DOUBLE_EQ(exact.velocity_mps[1], 2.2);
  EXPECT_DOUBLE_EQ(exact.minimum_lateral_bound_reserve_m, 0.8);
  EXPECT_EQ(result.minimum_progress_transition_state, 1);
  EXPECT_DOUBLE_EQ(result.minimum_progress_delta_m, 0.2);
  EXPECT_DOUBLE_EQ(result.transition_virtual_progress_speed_mps, 2.0);
  EXPECT_DOUBLE_EQ(result.transition_duration_sec, 0.10);
  EXPECT_NEAR(result.progress_dynamics_defect_m, 0.0, 1e-12);
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

TEST(MpccRateResolvedPhysicalAdapter, PropagatesCertifiedPredictedVelocityResidual)
{
  auto source = artifact();
  source.predicted_states[1].velocity_mps = -5e-7;
  source.maximum_constraint_violation = 5e-7;
  auto result = adapter::build(
    source, contract::ControlIntent::Track,
    source.identity.source_context.stage_geometry_id);
  ASSERT_EQ(result.reason, adapter::RejectReason::None);
  ASSERT_TRUE(result.exact_trajectory.has_value());
  EXPECT_DOUBLE_EQ(
    result.exact_trajectory->velocity_mps.front(), -5e-7);
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
