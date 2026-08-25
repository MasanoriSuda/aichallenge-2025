#include "multi_purpose_mpc_ros/mpcc_rate_resolved_execution_source.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <utility>

namespace
{

namespace source =
  multi_purpose_mpc_ros::mpcc_rate_resolved_execution_source;
namespace certified =
  multi_purpose_mpc_ros::mpcc_rate_resolved_certified_plan;
namespace execution =
  multi_purpose_mpc_ros::mpcc_rate_resolved_execution_artifact;
namespace physical =
  multi_purpose_mpc_ros::mpcc_rate_resolved_physical_wall;
namespace contract = multi_purpose_mpc_ros::mpcc_execution_contract;
namespace recovery = multi_purpose_mpc_ros::recovery_footprint;

contract::MpccProblemContext context()
{
  contract::MpccProblemContext value;
  value.decision_id = 10U;
  value.observation_generation = 10U;
  value.intent = contract::ControlIntent::ShiftOut;
  value.intent_generation = 7U;
  value.target_id = "d2";
  value.target_obstacle_generation = 44U;
  value.execution_side_sign = -1;
  value.stage_geometry_id = 20U;
  value.horizon_steps = 2U;
  value.formulation =
    contract::Formulation::VelocitySteeringProgress6State;
  value.state_schema_id = "ey-elag-epsi-v-progress-steering-v1";
  value.input_schema_id = "accel-steering-rate-progress-rate-v1";
  value.bounds_schema_id = "stage-wall-v1";
  value.cost_schema_id = "velocity-progress-steering-rate-v1";
  return contract::seal_problem_context(std::move(value));
}

std::shared_ptr<const certified::CertifiedPlan> plan()
{
  execution::ExecutionArtifact artifact;
  artifact.identity = execution::Identity{3U, context(), 12.0};
  artifact.prediction_origin_sec = 12.02;
  artifact.completed_sec = 12.03;
  artifact.course_progress_origin_m = 50.0;
  artifact.semantic_initial_steering_rad = 0.1;
  artifact.wheelbase_m = 2.0;
  artifact.maximum_abs_steering_rad = 0.6;
  artifact.maximum_abs_steering_rate_radps = 1.0;
  artifact.physical_global_tolerance = 1e-6;
  artifact.maximum_constraint_violation = 1e-8;
  artifact.maximum_normalized_constraint_violation = 0.1;
  artifact.predicted_states = {
    {0.0, 0.0, 0.0, 3.0, 0.0, 0.1},
    {-0.1, 0.0, 0.0, 3.1, 0.3, 0.1},
    {-0.2, 0.0, 0.0, 3.2, 0.6, 0.1},
  };
  artifact.control_stages = {
    {1.0, 0.0, 3.0, 0.1, 0.0, 4.0, -3.0, 1.37},
    {1.0, 0.0, 3.0, 0.1, 0.0, 4.0, -3.0, 1.37},
  };
  artifact.nominal_path_distance_m = {0.0, 0.3, 0.6};
  artifact.lateral_lower_m = {-1.0, -1.0, -1.0};
  artifact.lateral_upper_m = {1.0, 1.0, 1.0};

  physical::Snapshot snapshot;
  snapshot.identity.artifact = artifact.identity;
  snapshot.identity.pose_snapshot_id = 101U;
  snapshot.identity.course_frame_window_id = 102U;
  snapshot.identity.captured_sec = 12.0;
  auto grid = std::make_shared<recovery::OccupancyGrid>();
  grid->width = 400U;
  grid->height = 400U;
  grid->resolution_m = 0.1;
  grid->origin_x_m = 30.0;
  grid->origin_y_m = -20.0;
  grid->cells.assign(grid->width * grid->height, recovery::CellState::Free);
  snapshot.wall_grid = std::move(grid);
  snapshot.wall_grid_fingerprint =
    recovery::occupancy_grid_fingerprint(*snapshot.wall_grid);
  snapshot.footprint = {0.1, 0.1, 0.1, 0.1, 0.0};
  snapshot.current_pose = {50.0, 0.0, 0.0};
  snapshot.control_prefix = {snapshot.current_pose};
  snapshot.trajectory.progress_origin_m = 50.0;
  snapshot.trajectory.path_distance_m = {0.3, 0.6};
  snapshot.trajectory.lateral_m = {-0.1, -0.2};
  snapshot.trajectory.lag_m = {0.0, 0.0};
  snapshot.trajectory.heading_offset_rad = {0.0, 0.0};
  snapshot.trajectory.velocity_mps = {3.1, 3.2};
  snapshot.trajectory.progress_m = {50.3, 50.6};
  snapshot.trajectory.lateral_lower_m = {-1.0, -1.0};
  snapshot.trajectory.lateral_upper_m = {1.0, 1.0};
  snapshot.trajectory.minimum_lateral_bound_reserve_m = 0.8;
  snapshot.trajectory.progress_regression_tolerance_m = 1e-6;
  snapshot.course_frame_knots = {
    {49.0, 49.0, 0.0, 0.0, 0},
    {52.0, 52.0, 0.0, 0.0, 3},
  };
  snapshot.bound_tolerance_m = 1e-6;
  snapshot.swept_step_m = 0.05;

  physical::Result physical_result;
  physical_result.identity = snapshot.identity;
  physical_result.outcome = physical::Outcome::Accepted;
  physical_result.diagnostic.reason =
    contract::PhysicalWallCertificateReason::Accepted;
  physical_result.completed_sec = 12.04;
  physical_result.compute_ms = 10.0;
  physical_result.detail = "accepted";

  auto artifact_owner =
    std::make_shared<const execution::ExecutionArtifact>(std::move(artifact));
  return certified::build(
    artifact_owner, snapshot, physical_result).plan;
}

source::Request request(const certified::CertifiedPlan * value)
{
  return source::Request{
    value, contract::ControlIntent::ShiftOut, "d2", 7U, -1};
}

TEST(MpccRateResolvedExecutionSource, ProjectsExactCertifiedSixStatePrefix)
{
  const auto certified_plan = plan();
  ASSERT_NE(certified_plan, nullptr);
  const auto result = source::build(request(certified_plan.get()));
  ASSERT_TRUE(result.accepted());
  EXPECT_EQ(result.source.artifact_sequence, 3U);
  EXPECT_EQ(result.source.source_snapshot_sec, 12.0);
  EXPECT_EQ(result.source.source_completed_sec, 12.03);
  EXPECT_EQ(result.source.course_progress_origin_m, 50.0);
  EXPECT_EQ(result.source.path_distance_m, (std::vector<double>{0.3, 0.6}));
  EXPECT_EQ(result.source.lateral_m, (std::vector<double>{-0.1, -0.2}));
  EXPECT_EQ(result.source.progress_m, (std::vector<double>{50.3, 50.6}));
}

TEST(MpccRateResolvedExecutionSource, RejectsMissionIdentityMismatch)
{
  const auto certified_plan = plan();
  ASSERT_NE(certified_plan, nullptr);
  auto mismatch = request(certified_plan.get());
  mismatch.mission_generation = 8U;
  EXPECT_EQ(
    source::build(mismatch).reason,
    source::RejectReason::MissionGenerationMismatch);
  mismatch = request(certified_plan.get());
  mismatch.side_sign = 1;
  EXPECT_EQ(source::build(mismatch).reason, source::RejectReason::SideMismatch);
  mismatch = request(certified_plan.get());
  mismatch.target_id = "d3";
  EXPECT_EQ(source::build(mismatch).reason, source::RejectReason::TargetMismatch);
}

TEST(MpccRateResolvedExecutionSource, RejectsDifferentIntent)
{
  const auto certified_plan = plan();
  ASSERT_NE(certified_plan, nullptr);
  auto mismatch = request(certified_plan.get());
  mismatch.intent = contract::ControlIntent::Pass;
  EXPECT_EQ(source::build(mismatch).reason, source::RejectReason::IntentMismatch);
}

TEST(MpccRateResolvedExecutionSource, PreservesObservationTimeWithoutRenewalInput)
{
  const auto certified_plan = plan();
  ASSERT_NE(certified_plan, nullptr);
  const auto first = source::build(request(certified_plan.get()));
  const auto second = source::build(request(certified_plan.get()));
  ASSERT_TRUE(first.accepted());
  ASSERT_TRUE(second.accepted());
  EXPECT_EQ(first.source.source_snapshot_sec, second.source.source_snapshot_sec);
  EXPECT_EQ(first.source.artifact_sequence, second.source.artifact_sequence);
}

}  // namespace
