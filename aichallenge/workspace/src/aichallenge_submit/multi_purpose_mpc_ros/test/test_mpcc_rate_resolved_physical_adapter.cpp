#include "multi_purpose_mpc_ros/mpcc_rate_resolved_physical_adapter.hpp"

#include <gtest/gtest.h>

namespace adapter =
  multi_purpose_mpc_ros::mpcc_rate_resolved_physical_adapter;
namespace execution =
  multi_purpose_mpc_ros::mpcc_rate_resolved_execution_artifact;
namespace contract = multi_purpose_mpc_ros::mpcc_execution_contract;

namespace
{

execution::ExecutionArtifact artifact()
{
  execution::ExecutionArtifact value;
  value.identity = execution::Identity{
    1U, 2U, 3U, 4U, contract::ControlIntent::Track, 10.0};
  value.prediction_origin_sec = 10.0;
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
    {1.0, 0.10, 2.0, 0.10},
    {1.0, 0.10, 2.0, 0.10},
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
    source, contract::ControlIntent::Track, source.identity.stage_geometry_id);
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
}

TEST(MpccRateResolvedPhysicalAdapter, RejectsCurrentSemanticMismatch)
{
  const auto source = artifact();
  EXPECT_EQ(
    adapter::build(
      source, contract::ControlIntent::Cruise,
      source.identity.stage_geometry_id).reason,
    adapter::RejectReason::IntentMismatch);
  EXPECT_EQ(
    adapter::build(
      source, contract::ControlIntent::Track,
      source.identity.stage_geometry_id + 1U).reason,
    adapter::RejectReason::StageGeometryMismatch);
}

TEST(MpccRateResolvedPhysicalAdapter, RejectsInvalidArtifactBeforeConversion)
{
  auto source = artifact();
  source.nominal_path_distance_m[1] = 0.0;
  const auto result = adapter::build(
    source, contract::ControlIntent::Track, source.identity.stage_geometry_id);
  EXPECT_EQ(result.reason, adapter::RejectReason::InvalidArtifact);
  EXPECT_EQ(
    result.artifact_reason, execution::RejectReason::InvalidPathDistance);
  EXPECT_FALSE(result.exact_trajectory.has_value());
}
