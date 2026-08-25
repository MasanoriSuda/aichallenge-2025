#include "multi_purpose_mpc_ros/mpcc_rate_resolved_physical_wall.hpp"

#include <gtest/gtest.h>

#include <memory>

namespace
{

namespace wall =
  multi_purpose_mpc_ros::mpcc_rate_resolved_physical_wall;
namespace contract = multi_purpose_mpc_ros::mpcc_execution_contract;
namespace recovery = multi_purpose_mpc_ros::recovery_footprint;

contract::MpccProblemContext source_context()
{
  contract::MpccProblemContext context;
  context.decision_id = 10U;
  context.intent = contract::ControlIntent::Track;
  context.intent_generation = 1U;
  context.observation_generation = 2U;
  context.stage_geometry_id = 30U;
  context.horizon_steps = 2U;
  context.formulation =
    contract::Formulation::VelocitySteeringProgress6State;
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

wall::Snapshot snapshot()
{
  wall::Snapshot value;
  value.identity.artifact.sequence = 5U;
  value.identity.artifact.source_context = source_context();
  value.identity.artifact.snapshot_sec = 1.0;
  value.identity.pose_snapshot_id = 40U;
  value.identity.course_frame_window_id = 50U;
  value.identity.captured_sec = 1.1;
  value.wall_grid = free_grid();
  value.footprint = recovery::FootprintExtents{0.1, 0.1, 0.1, 0.1, 0.0};
  value.current_pose = recovery::Pose2D{0.0, 0.0, 0.0};
  value.trajectory.progress_origin_m = 0.0;
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
  value.bound_tolerance_m = 1.0e-6;
  value.swept_step_m = 0.05;
  return value;
}

TEST(MpccRateResolvedPhysicalWall, AcceptsCompleteSweptFreePath)
{
  const auto result = wall::evaluate(snapshot());
  EXPECT_EQ(result.outcome, wall::Outcome::Accepted);
  EXPECT_EQ(
    result.diagnostic.reason,
    contract::PhysicalWallCertificateReason::Accepted);
  EXPECT_TRUE(wall::result_valid(result));
}

TEST(MpccRateResolvedPhysicalWall, RejectsWallContactBetweenStages)
{
  auto value = snapshot();
  const auto occupied = value.wall_grid->world_to_grid(1.5, 0.0);
  ASSERT_TRUE(occupied.has_value());
  auto mutable_grid = std::make_shared<recovery::OccupancyGrid>(*value.wall_grid);
  mutable_grid->cells[occupied->row * mutable_grid->width + occupied->column] =
    recovery::CellState::Occupied;
  value.wall_grid = mutable_grid;
  const auto result = wall::evaluate(value);
  EXPECT_EQ(result.outcome, wall::Outcome::SweptWallRejected);
  EXPECT_EQ(
    result.diagnostic.reason,
    contract::PhysicalWallCertificateReason::SweptPathViolation);
}

TEST(MpccRateResolvedPhysicalWall, RejectsUnsealedWorldIdentity)
{
  auto value = snapshot();
  value.identity.pose_snapshot_id = 0U;
  const auto result = wall::evaluate(value);
  EXPECT_EQ(result.outcome, wall::Outcome::InvalidInput);
}

TEST(MpccRateResolvedPhysicalWall, MailboxIsMonotonicAndNonBlocking)
{
  wall::Mailbox mailbox;
  const auto value = snapshot();
  EXPECT_TRUE(mailbox.register_submission(value.identity));
  auto result = wall::evaluate(value);
  EXPECT_EQ(mailbox.publish(result), wall::PublishReason::Accepted);
  const auto latest = mailbox.latest_after(0U);
  ASSERT_TRUE(latest.has_value());
  EXPECT_EQ(latest->identity.pose_snapshot_id, value.identity.pose_snapshot_id);
  EXPECT_FALSE(mailbox.latest_after(value.identity.artifact.sequence).has_value());
  EXPECT_EQ(mailbox.publish(std::move(result)), wall::PublishReason::SequenceRollback);
}

TEST(MpccRateResolvedPhysicalWall, MailboxRejectsMutatedWorldIdentity)
{
  wall::Mailbox mailbox;
  const auto value = snapshot();
  EXPECT_TRUE(mailbox.register_submission(value.identity));
  auto result = wall::evaluate(value);
  ++result.identity.pose_snapshot_id;
  EXPECT_TRUE(wall::result_valid(result));
  EXPECT_EQ(mailbox.publish(std::move(result)), wall::PublishReason::IdentityMismatch);
  const auto state = mailbox.state();
  EXPECT_EQ(state.identity_mismatch_count, 1U);
  EXPECT_EQ(state.accepted_count, 0U);
}

TEST(MpccRateResolvedPhysicalWall, MailboxRejectsSupersededCompletion)
{
  wall::Mailbox mailbox;
  const auto first = snapshot();
  auto second = first;
  second.identity.artifact.sequence = first.identity.artifact.sequence + 1U;
  second.identity.artifact.source_context.decision_id =
    first.identity.artifact.source_context.decision_id + 1U;
  second.identity.artifact.source_context = contract::seal_problem_context(
    std::move(second.identity.artifact.source_context));
  second.identity.captured_sec += 0.1;
  EXPECT_TRUE(mailbox.register_submission(first.identity));
  EXPECT_TRUE(mailbox.register_submission(second.identity));
  EXPECT_EQ(
    mailbox.publish(wall::evaluate(first)), wall::PublishReason::Superseded);
  EXPECT_EQ(mailbox.publish(wall::evaluate(second)), wall::PublishReason::Accepted);
}

}  // namespace
