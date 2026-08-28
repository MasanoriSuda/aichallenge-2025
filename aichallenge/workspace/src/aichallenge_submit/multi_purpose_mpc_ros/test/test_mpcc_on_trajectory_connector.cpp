#include "multi_purpose_mpc_ros/mpcc_on_trajectory_connector.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <utility>

namespace
{

namespace connector =
  multi_purpose_mpc_ros::mpcc_on_trajectory_connector;
namespace contract = multi_purpose_mpc_ros::mpcc_execution_contract;
namespace execution =
  multi_purpose_mpc_ros::mpcc_rate_resolved_execution_artifact;
namespace certified =
  multi_purpose_mpc_ros::mpcc_rate_resolved_certified_plan;
namespace physical =
  multi_purpose_mpc_ros::mpcc_rate_resolved_physical_wall;
namespace recovery = multi_purpose_mpc_ros::recovery_footprint;

contract::MpccProblemContext context(const std::uint64_t sequence)
{
  contract::MpccProblemContext value;
  value.decision_id = sequence + 10U;
  value.intent = contract::ControlIntent::Track;
  value.intent_generation = 1U;
  value.observation_generation = sequence + 20U;
  value.stage_geometry_id = sequence + 30U;
  value.horizon_steps = 2U;
  value.formulation =
    contract::Formulation::VelocitySteeringYawResponseProgress7State;
  value.state_schema_id = "ey-elag-epsi-v-progress-steering-v1";
  value.input_schema_id = "accel-steering-rate-progress-rate-v1";
  value.bounds_schema_id = "stage-wall-v1";
  value.cost_schema_id = "velocity-progress-steering-rate-v1";
  return contract::seal_problem_context(std::move(value));
}

execution::ExecutionArtifact artifact(
  const std::uint64_t sequence, const double lateral_offset_m)
{
  execution::ExecutionArtifact value;
  value.identity = execution::Identity{sequence, context(sequence), 10.0};
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
    {lateral_offset_m, 0.0, 0.0, 2.0, 0.0, 0.10, 0.10},
    {lateral_offset_m, 0.0, 0.0, 2.0, 0.2, 0.11,
      0.10302380180000528},
    {lateral_offset_m, 0.0, 0.0, 2.0, 0.4, 0.12,
      0.10979124524044208},
  };
  value.control_stages = {
    {0.0, 0.10, 2.0, 0.10, 0.0, 4.0, -3.0, 1.37},
    {0.0, 0.10, 2.0, 0.10, 0.0, 4.0, -3.0, 1.37},
  };
  value.nominal_path_distance_m = {0.0, 0.2, 0.4};
  value.lateral_lower_m = {-1.0, -1.0, -1.0};
  value.lateral_upper_m = {1.0, 1.0, 1.0};
  return value;
}

physical::Snapshot physical_snapshot(
  const execution::Identity & identity, const double lateral_offset_m)
{
  physical::Snapshot snapshot;
  snapshot.identity.artifact = identity;
  snapshot.identity.pose_snapshot_id = identity.sequence + 100U;
  snapshot.identity.course_frame_window_id = identity.sequence + 200U;
  snapshot.identity.captured_sec = identity.snapshot_sec;
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
  snapshot.current_pose = {50.0, lateral_offset_m, 0.0};
  snapshot.control_prefix = {snapshot.current_pose};
  snapshot.trajectory.progress_origin_m = 50.0;
  snapshot.trajectory.elapsed_time_sec = {0.1, 0.2};
  snapshot.trajectory.path_distance_m = {0.2, 0.4};
  snapshot.trajectory.lateral_m = {lateral_offset_m, lateral_offset_m};
  snapshot.trajectory.lag_m = {0.0, 0.0};
  snapshot.trajectory.heading_offset_rad = {0.0, 0.0};
  snapshot.trajectory.velocity_mps = {2.0, 2.0};
  snapshot.trajectory.progress_m = {50.2, 50.4};
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
  return snapshot;
}

std::shared_ptr<const certified::CertifiedPlan> plan(
  const std::uint64_t sequence, const double lateral_offset_m = 0.0)
{
  auto execution_artifact =
    std::make_shared<const execution::ExecutionArtifact>(
    artifact(sequence, lateral_offset_m));
  const auto snapshot = physical_snapshot(
    execution_artifact->identity, lateral_offset_m);
  physical::Result proof;
  proof.identity = snapshot.identity;
  proof.outcome = physical::Outcome::Accepted;
  proof.diagnostic.reason =
    contract::PhysicalWallCertificateReason::Accepted;
  proof.completed_sec = 10.02;
  proof.compute_ms = 20.0;
  proof.detail = "accepted";
  const auto built = certified::build(execution_artifact, snapshot, proof);
  EXPECT_EQ(built.reason, certified::RejectReason::None);
  return built.plan;
}

connector::Request request(
  const std::shared_ptr<const certified::CertifiedPlan> & parent,
  const std::shared_ptr<const certified::CertifiedPlan> & candidate,
  const double switch_sec = 10.05)
{
  connector::Request value;
  value.parent = parent;
  value.candidate = candidate;
  value.parent_first_published_control_origin_sec = 10.0;
  value.parent_first_published_artifact_elapsed_sec = 0.0;
  value.switch_control_origin_sec = switch_sec;
  value.path_length_m = 100.0;
  value.circular = true;
  return value;
}

TEST(MpccOnTrajectoryConnector, AcceptsCommonCertifiedSwitchState)
{
  const auto result = connector::evaluate(request(plan(1U), plan(2U)));
  EXPECT_TRUE(result.accepted());
  EXPECT_EQ(result.reason, connector::Reason::Accepted);
  EXPECT_NEAR(result.parent_elapsed_sec, 0.05, 1e-12);
  EXPECT_NEAR(result.candidate_elapsed_sec, 0.05, 1e-12);
  EXPECT_NEAR(result.progress_difference_m, 0.0, 1e-12);
  EXPECT_NEAR(result.steering_difference_rad, 0.0, 1e-12);
}

TEST(MpccOnTrajectoryConnector, RejectsDifferentPhysicalTrajectory)
{
  const auto result = connector::evaluate(
    request(plan(1U), plan(2U, 0.10)));
  EXPECT_EQ(result.reason, connector::Reason::StateMismatch);
  EXPECT_NEAR(result.lateral_difference_m, 0.10, 1e-12);
  EXPECT_GT(
    std::abs(result.lateral_difference_m), result.position_tolerance_m);
}

TEST(MpccOnTrajectoryConnector, PreservesFirstPublishedArtifactCursor)
{
  auto value = request(plan(1U), plan(2U), 10.025);
  value.parent_first_published_artifact_elapsed_sec = 0.10;
  const auto result = connector::evaluate(value);
  EXPECT_EQ(result.reason, connector::Reason::StateMismatch);
  EXPECT_NEAR(result.parent_elapsed_sec, 0.125, 1e-12);
  EXPECT_NEAR(result.candidate_elapsed_sec, 0.025, 1e-12);
  EXPECT_GT(result.progress_difference_m, -1.0);
}

TEST(MpccOnTrajectoryConnector, ReportsExhaustedParent)
{
  const auto result = connector::evaluate(
    request(plan(1U), plan(2U), 10.25));
  EXPECT_EQ(result.reason, connector::Reason::ParentCursorUnavailable);
  EXPECT_EQ(result.parent_cursor_reason, execution::CursorReason::Exhausted);
}

TEST(MpccOnTrajectoryConnector, RejectsSwitchBeforeCandidateOrigin)
{
  auto value = request(plan(1U), plan(2U), 9.99);
  value.parent_first_published_control_origin_sec = 9.90;
  const auto result = connector::evaluate(value);
  EXPECT_EQ(result.reason, connector::Reason::SwitchBeforeCandidateOrigin);
}

}  // namespace
