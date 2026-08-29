#include "multi_purpose_mpc_ros/mpcc_rate_resolved_dynamic_proof.hpp"

#include "multi_purpose_mpc_ros/mpcc_execution_contract.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_physical_wall.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_shadow.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <memory>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_dynamic_proof
{
namespace
{

namespace contract = mpcc_execution_contract;
namespace shadow = mpcc_rate_resolved_shadow;
namespace wall = mpcc_rate_resolved_physical_wall;

contract::MpccProblemContext source_context()
{
  contract::MpccProblemContext context;
  context.decision_id = 10U;
  context.intent = contract::ControlIntent::ShiftOut;
  context.intent_generation = 1U;
  context.observation_generation = 2U;
  context.target_id = "d2";
  context.target_obstacle_generation = 3U;
  context.execution_side_sign = 1;
  context.stage_geometry_id = 30U;
  context.horizon_steps = 2U;
  context.formulation =
    contract::Formulation::VelocitySteeringYawResponseProgress7State;
  context.state_schema_id = "ey-elag-epsi-v-progress-steering-v1";
  context.input_schema_id = "accel-steering-rate-progress-rate-v1";
  context.bounds_schema_id = "stage-wall-v1";
  context.cost_schema_id = "velocity-progress-steering-rate-v1";
  return contract::seal_problem_context(std::move(context));
}

std::shared_ptr<recovery::OccupancyGrid> free_grid()
{
  auto grid = std::make_shared<recovery::OccupancyGrid>();
  grid->width = 200U;
  grid->height = 200U;
  grid->resolution_m = 0.1;
  grid->origin_x_m = -10.0;
  grid->origin_y_m = -10.0;
  grid->y_axis = recovery::YAxisConvention::RowZeroAtMaximumY;
  grid->cells.assign(grid->width * grid->height, recovery::CellState::Free);
  return grid;
}

wall::Snapshot physical_snapshot()
{
  wall::Snapshot value;
  value.identity.artifact.sequence = 5U;
  value.identity.artifact.source_context = source_context();
  value.identity.artifact.snapshot_sec = 10.0;
  value.identity.pose_snapshot_id = 40U;
  value.identity.course_frame_window_id = 50U;
  value.identity.captured_sec = 10.0;
  value.wall_grid = free_grid();
  value.wall_grid_fingerprint =
    recovery::occupancy_grid_fingerprint(*value.wall_grid);
  value.footprint = recovery::FootprintExtents{0.1, 0.1, 0.1, 0.1, 0.0};
  value.current_pose = recovery::Pose2D{0.0, 0.0, 0.0};
  value.control_prefix = {value.current_pose};
  value.trajectory.progress_origin_m = 0.0;
  value.trajectory.elapsed_time_sec = {0.1, 0.2};
  value.trajectory.path_distance_m = {1.0, 2.0};
  value.trajectory.lateral_m = {0.0, 0.0};
  value.trajectory.lag_m = {0.0, 0.0};
  value.trajectory.heading_offset_rad = {0.0, 0.0};
  value.trajectory.velocity_mps = {2.0, 2.0};
  value.trajectory.progress_m = {1.0, 2.0};
  value.trajectory.lateral_lower_m = {-1.0, -1.0};
  value.trajectory.lateral_upper_m = {1.0, 1.0};
  value.trajectory.minimum_lateral_bound_reserve_m = 1.0;
  value.course_frame_knots = {
    {0.0, 0.0, 0.0, 0.0, 0},
    {3.0, 3.0, 0.0, 0.0, 3},
  };
  value.terminal_stop_course_geometry = {
    {0.0, 1.0, 2.0}, {0.0, 0.0},
    {-1.0, -1.0, -1.0}, {1.0, 1.0, 1.0}};
  value.bound_tolerance_m = 1.0e-6;
  value.swept_step_m = 0.05;
  return value;
}

shadow::Snapshot solver_snapshot(
  const wall::Snapshot & physical, const double obstacle_y_m)
{
  shadow::Snapshot value;
  value.identity = physical.identity.artifact;
  value.control_prediction_origin_sec = 10.0;
  shadow::ReplayWorld replay;
  replay.observation_generation = 3U;
  replay.observed_sec = 10.0;
  replay.current = true;
  replay.current_pose = physical.current_pose;
  replay.control_prefix = physical.control_prefix;
  replay.control_prefix_elapsed_sec = {0.0};
  replay.physical_footprint = physical.footprint;
  replay.wall_grid_fingerprint = physical.wall_grid_fingerprint;
  replay.bound_tolerance_m = physical.bound_tolerance_m;
  replay.swept_step_m = physical.swept_step_m;
  replay.obstacles.push_back(shadow::ReplayDynamicObstacle{
    "d2", 1.5, obstacle_y_m, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.1, 3U});
  value.replay_world = std::move(replay);
  return value;
}

TEST(MpccRateResolvedDynamicProof, RecordsFirstDenseRejectionProvenance)
{
  const recovery::FootprintExtents footprint{0.5, 0.5, 0.4, 0.4, 0.0};
  WorldObservation world;
  world.generation = 3U;
  world.observed_sec = 10.0;
  world.current = true;
  world.obstacles.push_back(
    DynamicObstacle{"d2", recovery::CircleObstacle{2.0, 0.0, 0.0, 0.0, 0.2}});
  Result result;

  observe_segment(
    footprint, recovery::Pose2D{0.0, 0.0, 0.0},
    recovery::Pose2D{3.0, 0.0, 0.0}, 0.0, 3.0, 0.05, world, result);

  EXPECT_TRUE(result.valid);
  EXPECT_FALSE(result.clear);
  EXPECT_EQ(
    result.rejection_reason,
    recovery::DynamicClearanceRejectReason::NewOverlap);
  EXPECT_EQ(result.blocking_obstacle_id, "d2");
  EXPECT_EQ(result.rejected_obstacle_id, "d2");
  EXPECT_TRUE(std::isfinite(result.rejected_elapsed_sec));
  EXPECT_GT(result.rejected_elapsed_sec, 0.0);
  EXPECT_TRUE(std::isfinite(result.rejected_pose.x_m));
  EXPECT_LT(result.rejected_clearance_m, 0.0);
  EXPECT_EQ(result.minimum_clearance_obstacle_id, "d2");
  EXPECT_TRUE(std::isfinite(result.minimum_clearance_elapsed_sec));
  EXPECT_LE(result.minimum_clearance_m, result.rejected_clearance_m);
}

TEST(MpccRateResolvedDynamicProof, CurrentWorldUsesExactSharedWorldTrajectory)
{
  const auto physical = physical_snapshot();
  ASSERT_EQ(wall::evaluate(physical).outcome, wall::Outcome::Accepted);

  const auto blocked = evaluate_current_world(
    solver_snapshot(physical, 0.0), physical);
  EXPECT_TRUE(blocked.valid);
  EXPECT_FALSE(blocked.clear);
  EXPECT_EQ(blocked.blocking_obstacle_id, "d2");
  EXPECT_EQ(
    blocked.rejection_reason,
    recovery::DynamicClearanceRejectReason::NewOverlap);

  const auto clear = evaluate_current_world(
    solver_snapshot(physical, 5.0), physical);
  EXPECT_TRUE(clear.valid);
  EXPECT_TRUE(clear.clear);
  EXPECT_GT(clear.minimum_clearance_m, 0.0);
}

TEST(MpccRateResolvedDynamicProof, CurrentWorldRejectsDivergedPhysicalSource)
{
  auto physical = physical_snapshot();
  auto solver = solver_snapshot(physical, 5.0);
  physical.footprint.left_extent_m += 0.01;
  const auto result = evaluate_current_world(solver, physical);
  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.clear);
}

}  // namespace
}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_dynamic_proof
