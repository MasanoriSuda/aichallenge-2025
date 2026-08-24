#include "multi_purpose_mpc_ros/mpcc_rate_resolved_retained_revalidation.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <memory>

namespace
{

namespace retained =
  multi_purpose_mpc_ros::mpcc_rate_resolved_retained_revalidation;
namespace certified =
  multi_purpose_mpc_ros::mpcc_rate_resolved_certified_plan;
namespace artifact =
  multi_purpose_mpc_ros::mpcc_rate_resolved_execution_artifact;
namespace physical =
  multi_purpose_mpc_ros::mpcc_rate_resolved_physical_wall;
namespace recovery = multi_purpose_mpc_ros::recovery_footprint;
namespace contract = multi_purpose_mpc_ros::mpcc_execution_contract;

artifact::ExecutionArtifact execution_artifact()
{
  artifact::ExecutionArtifact value;
  value.identity = {
    1U, 11U, 21U, 31U, contract::ControlIntent::Track,
    contract::Formulation::VelocitySteeringProgress6State, 1.0};
  value.prediction_origin_sec = 1.0;
  value.completed_sec = 1.01;
  value.course_progress_origin_m = 50.0;
  value.semantic_initial_steering_rad = 0.10;
  value.wheelbase_m = 2.0;
  value.maximum_abs_steering_rad = 0.60;
  value.maximum_abs_steering_rate_radps = 1.0;
  value.physical_global_tolerance = 1e-6;
  value.maximum_constraint_violation = 1e-8;
  value.maximum_normalized_constraint_violation = 0.1;
  value.predicted_states = {
    {0.0, 0.10, 0.0, 2.0, 0.0, 0.10},
    {0.10, 0.0, 0.0, 2.1, 0.2, 0.11},
    {0.20, 0.0, 0.0, 2.2, 0.4, 0.12},
  };
  value.control_stages = {
    {1.0, 0.10, 2.0, 0.10, 0.0, 4.0},
    {1.0, 0.10, 2.0, 0.10, 0.0, 4.0},
  };
  value.nominal_path_distance_m = {0.0, 0.2, 0.4};
  value.lateral_lower_m = {-1.0, -1.0, -1.0};
  value.lateral_upper_m = {1.0, 1.0, 1.0};
  return value;
}

std::shared_ptr<recovery::OccupancyGrid> free_grid()
{
  auto grid = std::make_shared<recovery::OccupancyGrid>();
  grid->width = 400U;
  grid->height = 400U;
  grid->resolution_m = 0.05;
  grid->origin_x_m = 45.0;
  grid->origin_y_m = -5.0;
  grid->cells.assign(grid->width * grid->height, recovery::CellState::Free);
  return grid;
}

physical::Snapshot source_snapshot(
  const artifact::Identity & identity,
  std::shared_ptr<recovery::OccupancyGrid> grid = free_grid())
{
  physical::Snapshot snapshot;
  snapshot.identity.artifact = identity;
  snapshot.identity.pose_snapshot_id = 101U;
  snapshot.identity.course_frame_window_id = 102U;
  snapshot.identity.captured_sec = 1.0;
  snapshot.wall_grid = std::move(grid);
  snapshot.footprint = {0.05, 0.05, 0.05, 0.05, 0.0};
  snapshot.current_pose = {50.0, 0.0, 0.0};
  snapshot.trajectory.progress_origin_m = 50.0;
  snapshot.trajectory.path_distance_m = {0.2, 0.4};
  snapshot.trajectory.lateral_m = {0.10, 0.20};
  snapshot.trajectory.lag_m = {0.0, 0.0};
  snapshot.trajectory.heading_offset_rad = {0.0, 0.0};
  snapshot.trajectory.velocity_mps = {2.1, 2.2};
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
  snapshot.swept_step_m = 0.02;
  return snapshot;
}

physical::Result accepted_result(const physical::Snapshot & snapshot)
{
  physical::Result result;
  result.identity = snapshot.identity;
  result.outcome = physical::Outcome::Accepted;
  result.diagnostic.reason =
    contract::PhysicalWallCertificateReason::Accepted;
  result.completed_sec = 1.01;
  result.compute_ms = 10.0;
  result.detail = "accepted";
  return result;
}

std::shared_ptr<const certified::CertifiedPlan> certified_plan(
  std::shared_ptr<recovery::OccupancyGrid> grid = free_grid())
{
  auto execution = std::make_shared<const artifact::ExecutionArtifact>(
    execution_artifact());
  const auto snapshot = source_snapshot(execution->identity, std::move(grid));
  const auto built = certified::build(
    execution, snapshot, accepted_result(snapshot));
  EXPECT_EQ(built.reason, certified::RejectReason::None);
  return built.plan;
}

retained::Request accepted_request(
  const std::shared_ptr<const certified::CertifiedPlan> & plan)
{
  retained::Request request;
  request.plan = plan;
  request.decision_id = 100U;
  request.now_sec = 1.05;
  request.control_origin_sec = 1.05;
  request.current_intent = contract::ControlIntent::Track;
  request.measured_course_progress_m = 50.10;
  request.path_length_m = 100.0;
  request.progress_continuity_tolerance_m = 0.20;
  request.circular = true;
  request.control_pose = {50.05, 0.05, 0.0};
  request.measured_to_control_path = {request.control_pose};
  request.measured_to_control_elapsed_sec = {0.0};
  request.current_wall_grid = plan->physical_snapshot->wall_grid;
  request.current_footprint = plan->physical_snapshot->footprint;
  request.obstacles.generation = 7U;
  request.obstacles.observed_sec = 1.05;
  request.obstacles.current = true;
  request.current_speed_mps = 2.05;
  request.current_steering_rad = 0.105;
  request.minimum_acceleration_mps2 = -3.0;
  request.maximum_acceleration_mps2 = 1.0;
  request.publication_interval_sec = 0.025;
  return request;
}

TEST(MpccRateResolvedRetainedRevalidation, AcceptsCurrentWorldJoin)
{
  const auto plan = certified_plan();
  ASSERT_NE(plan, nullptr);
  const auto result = retained::evaluate(accepted_request(plan));
  ASSERT_EQ(result.reason, retained::Reason::Accepted);
  ASSERT_TRUE(result.proof.has_value());
  EXPECT_EQ(result.proof->cursor.control_stage_index, 0U);
  EXPECT_NEAR(result.proof->expected_absolute_progress_m, 50.10, 1e-9);
  EXPECT_EQ(result.proof->obstacle_generation, 7U);
}

TEST(MpccRateResolvedRetainedRevalidation, AcceptsClearDynamicObstacle)
{
  const auto plan = certified_plan();
  auto request = accepted_request(plan);
  request.obstacles.obstacles.push_back(
    {"peer", {50.0, 3.0, 0.0, 0.0, 0.2}});
  const auto result = retained::evaluate(request);
  ASSERT_EQ(result.reason, retained::Reason::Accepted);
  ASSERT_TRUE(result.proof.has_value());
  EXPECT_GT(result.proof->dynamic_checked_pose_count, 0U);
  EXPECT_GT(result.proof->minimum_dynamic_clearance_m, 0.0);
}

TEST(MpccRateResolvedRetainedRevalidation, RejectsIntersectingDynamicObstacle)
{
  const auto plan = certified_plan();
  auto request = accepted_request(plan);
  request.obstacles.obstacles.push_back(
    {"peer", {50.10, 0.10, 0.0, 0.0, 0.2}});
  const auto result = retained::evaluate(request);
  EXPECT_EQ(result.reason, retained::Reason::DynamicPathBlocked);
  EXPECT_EQ(result.blocking_obstacle_id, "peer");
  EXPECT_LT(result.minimum_dynamic_clearance_m, 0.0);
}

TEST(MpccRateResolvedRetainedRevalidation, RejectsFutureCrossingObstacle)
{
  const auto plan = certified_plan();
  auto request = accepted_request(plan);
  request.obstacles.obstacles.push_back(
    // The retained suffix has only 0.15 s left at now=1.05.  Start close
    // enough that the moving circle intersects it inside that exact horizon.
    {"crossing", {50.20, 0.30, 0.0, -1.0, 0.15}});
  EXPECT_EQ(
    retained::evaluate(request).reason,
    retained::Reason::DynamicPathBlocked);
}

TEST(MpccRateResolvedRetainedRevalidation, RejectsObstacleCrossingDuringDelayPrefix)
{
  const auto plan = certified_plan();
  auto request = accepted_request(plan);
  request.now_sec = 1.0;
  request.control_origin_sec = 1.05;
  request.measured_to_control_path = {
    {49.95, 0.05, 0.0}, request.control_pose};
  request.measured_to_control_elapsed_sec = {0.0, 0.05};
  request.obstacles.observed_sec = request.now_sec;
  request.obstacles.obstacles.push_back(
    {"delay-crossing", {50.0, 0.30, 0.0, -10.0, 0.02}});
  const auto result = retained::evaluate(request);
  EXPECT_EQ(result.reason, retained::Reason::DynamicPathBlocked);
  EXPECT_EQ(result.blocking_obstacle_id, "delay-crossing");
}

TEST(MpccRateResolvedRetainedRevalidation, RejectsInconsistentControlTimePrefix)
{
  const auto plan = certified_plan();
  auto request = accepted_request(plan);
  request.control_origin_sec = request.now_sec + 0.05;
  request.measured_to_control_elapsed_sec = {0.0};
  EXPECT_EQ(
    retained::evaluate(request).reason,
    retained::Reason::InvalidCurrentState);
}

TEST(MpccRateResolvedRetainedRevalidation, AcceptsMultipleClearPeers)
{
  const auto plan = certified_plan();
  auto request = accepted_request(plan);
  request.obstacles.obstacles = {
    {"behind", {45.0, 0.0, -1.0, 0.0, 0.2}},
    {"outside", {51.0, 3.0, 0.0, 0.0, 0.2}},
  };
  EXPECT_EQ(retained::evaluate(request).reason, retained::Reason::Accepted);
}

TEST(MpccRateResolvedRetainedRevalidation, RejectsDuplicateObstacleIdentity)
{
  const auto plan = certified_plan();
  auto request = accepted_request(plan);
  request.obstacles.obstacles = {
    {"peer", {45.0, 3.0, 0.0, 0.0, 0.2}},
    {"peer", {46.0, 3.0, 0.0, 0.0, 0.2}},
  };
  EXPECT_EQ(
    retained::evaluate(request).reason,
    retained::Reason::DynamicObservationInvalid);
}

TEST(MpccRateResolvedRetainedRevalidation, RejectsInvalidObstacleMotion)
{
  const auto plan = certified_plan();
  auto request = accepted_request(plan);
  request.obstacles.obstacles.push_back(
    {"peer", {45.0, 3.0, std::numeric_limits<double>::quiet_NaN(), 0.0, 0.2}});
  EXPECT_EQ(
    retained::evaluate(request).reason,
    retained::Reason::DynamicObservationInvalid);
}

TEST(MpccRateResolvedRetainedRevalidation, RejectsUnobservedDynamicWorld)
{
  const auto plan = certified_plan();
  auto request = accepted_request(plan);
  request.obstacles.current = false;
  EXPECT_EQ(
    retained::evaluate(request).reason,
    retained::Reason::DynamicObservationUnavailable);
}

TEST(MpccRateResolvedRetainedRevalidation, RejectsDifferentStaticWorldOwner)
{
  const auto plan = certified_plan();
  auto request = accepted_request(plan);
  request.current_wall_grid = free_grid();
  EXPECT_EQ(
    retained::evaluate(request).reason,
    retained::Reason::StaticWorldMismatch);
}

TEST(MpccRateResolvedRetainedRevalidation, RejectsProgressDiscontinuity)
{
  const auto plan = certified_plan();
  auto request = accepted_request(plan);
  request.measured_course_progress_m = 60.0;
  EXPECT_EQ(
    retained::evaluate(request).reason,
    retained::Reason::ProgressLiftRejected);
}

TEST(MpccRateResolvedRetainedRevalidation, RejectsUnreachableSteering)
{
  const auto plan = certified_plan();
  auto request = accepted_request(plan);
  request.current_steering_rad = -0.10;
  EXPECT_EQ(
    retained::evaluate(request).reason,
    retained::Reason::SteeringUnreachable);
}

TEST(MpccRateResolvedRetainedRevalidation, RejectsUnreachableVelocity)
{
  const auto plan = certified_plan();
  auto request = accepted_request(plan);
  request.current_speed_mps = 4.0;
  EXPECT_EQ(
    retained::evaluate(request).reason,
    retained::Reason::VelocityUnreachable);
}

TEST(MpccRateResolvedRetainedRevalidation, RejectsBlockedConnector)
{
  auto grid = free_grid();
  const auto plan = certified_plan(grid);
  ASSERT_NE(plan, nullptr);
  auto request = accepted_request(plan);
  request.control_pose = {50.05, 0.50, 0.0};
  request.measured_to_control_path = {request.control_pose};
  const auto occupied = grid->world_to_grid(50.05, 0.25);
  ASSERT_TRUE(occupied.has_value());
  grid->cells[occupied->row * grid->width + occupied->column] =
    recovery::CellState::Occupied;
  EXPECT_EQ(
    retained::evaluate(request).reason,
    retained::Reason::ConnectorBlocked);
}

TEST(MpccRateResolvedRetainedRevalidation, RejectsExhaustedArtifact)
{
  const auto plan = certified_plan();
  auto request = accepted_request(plan);
  request.now_sec = 2.0;
  request.control_origin_sec = 2.0;
  const auto result = retained::evaluate(request);
  EXPECT_EQ(result.reason, retained::Reason::CursorUnavailable);
  EXPECT_EQ(result.cursor_reason, artifact::CursorReason::Exhausted);
}

}  // namespace
