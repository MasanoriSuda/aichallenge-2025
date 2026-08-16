#include "multi_purpose_mpc_ros/recovery_footprint.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>

namespace
{

namespace recovery = multi_purpose_mpc_ros::recovery_footprint;

constexpr double kPi = 3.14159265358979323846;

recovery::OccupancyGrid make_grid(
  const std::size_t width = 20U, const std::size_t height = 20U,
  const double resolution_m = 1.0)
{
  recovery::OccupancyGrid grid;
  grid.width = width;
  grid.height = height;
  grid.resolution_m = resolution_m;
  grid.origin_x_m = 0.0;
  grid.origin_y_m = 0.0;
  grid.y_axis = recovery::YAxisConvention::RowZeroAtMaximumY;
  grid.cells.assign(width * height, recovery::CellState::Free);
  return grid;
}

void set_world_cell(
  recovery::OccupancyGrid & grid, const double x_m, const double y_m,
  const recovery::CellState state = recovery::CellState::Occupied)
{
  const auto index = grid.world_to_grid(x_m, y_m);
  ASSERT_TRUE(index.has_value());
  grid.cells[index->row * grid.width + index->column] = state;
}

std::size_t world_cell_index(
  const recovery::OccupancyGrid & grid, const double x_m, const double y_m)
{
  const auto index = grid.world_to_grid(x_m, y_m);
  return index->row * grid.width + index->column;
}

recovery::FootprintExtents compact_footprint()
{
  return recovery::FootprintExtents{0.2, 0.2, 0.2, 0.2, 0.0};
}

recovery::ReverseRolloutParameters straight_parameters(const double distance_m = 2.0)
{
  return recovery::ReverseRolloutParameters{
    distance_m,
    0.5,
    0.1,
    1.0,
    0.4};
}

TEST(RecoveryFootprintGrid, UsesExplicitRowMajorYFlipCompatibleWithMpcMap)
{
  auto grid = make_grid(4U, 3U, 0.5);
  grid.origin_x_m = 10.0;
  grid.origin_y_m = 20.0;

  const auto lower_left = grid.world_to_grid(10.0, 20.0);
  ASSERT_TRUE(lower_left.has_value());
  EXPECT_EQ(lower_left->row, 2U);
  EXPECT_EQ(lower_left->column, 0U);

  const auto upper_right = grid.world_to_grid(11.5, 21.0);
  ASSERT_TRUE(upper_right.has_value());
  EXPECT_EQ(upper_right->row, 0U);
  EXPECT_EQ(upper_right->column, 3U);

  const auto world = grid.grid_to_world(0U, 3U);
  ASSERT_TRUE(world.has_value());
  EXPECT_DOUBLE_EQ(world->x_m, 11.5);
  EXPECT_DOUBLE_EQ(world->y_m, 21.0);

  EXPECT_FALSE(grid.world_to_grid(9.70, 20.0).has_value());
  EXPECT_FALSE(grid.world_to_grid(10.0, 21.30).has_value());
}

TEST(RecoveryFootprintSteeringSamples, DividesMaximumAngleDeterministically)
{
  const auto samples = recovery::steering_magnitude_samples(0.25, 5U);
  ASSERT_EQ(samples.size(), 5U);
  EXPECT_NEAR(samples[0], 0.05, 1e-12);
  EXPECT_NEAR(samples[1], 0.10, 1e-12);
  EXPECT_NEAR(samples[2], 0.15, 1e-12);
  EXPECT_NEAR(samples[3], 0.20, 1e-12);
  EXPECT_NEAR(samples[4], 0.25, 1e-12);
  EXPECT_TRUE(recovery::steering_magnitude_samples(0.0, 5U).empty());
  EXPECT_TRUE(recovery::steering_magnitude_samples(0.25, 0U).empty());
  EXPECT_TRUE(recovery::steering_magnitude_samples(
    std::numeric_limits<double>::quiet_NaN(), 5U).empty());
}

TEST(RecoveryFootprintFeasibility, ClearStraightRolloutIsFeasible)
{
  const auto result = recovery::evaluate_reverse_candidate(
    make_grid(), compact_footprint(), recovery::Pose2D{10.0, 10.0, 0.0},
    recovery::ReversePrimitive::Straight, straight_parameters(3.0));

  EXPECT_TRUE(result.feasible);
  EXPECT_EQ(result.reason, recovery::RejectReason::None);
  EXPECT_EQ(result.initial_contact_count, 0U);
  EXPECT_EQ(result.final_contact_count, 0U);
  EXPECT_GT(result.checked_pose_count, result.rollout.size());
}

TEST(RecoveryFootprintStepwiseMode, UsesShortStepsForClearSideWallEvidence)
{
  EXPECT_TRUE(recovery::use_stepwise_escape_mode(
    true, true, 0U, recovery::WallRegion::Left, 0U));
  EXPECT_TRUE(recovery::use_stepwise_escape_mode(
    true, true, 0U, recovery::WallRegion::Right, 0U));
  EXPECT_TRUE(recovery::use_stepwise_escape_mode(
    true, true, 0U, recovery::WallRegion::Mixed, 0U));
  EXPECT_TRUE(recovery::use_stepwise_escape_mode(
    true, true, 0U, recovery::WallRegion::None, 0U));
}

TEST(RecoveryFootprintStepwiseMode, KeepsClearFrontAndRearDirectionSpecific)
{
  EXPECT_FALSE(recovery::use_stepwise_escape_mode(
    true, true, 0U, recovery::WallRegion::Front, 0U));
  EXPECT_FALSE(recovery::use_stepwise_escape_mode(
    true, true, 0U, recovery::WallRegion::Rear, 0U));
  EXPECT_FALSE(recovery::use_stepwise_escape_mode(
    false, true, 0U, recovery::WallRegion::Left, 0U));
}

TEST(RecoveryFootprintStepwiseMode, ContinuousModeBypassesStepsOnlyAfterFootprintIsClear)
{
  EXPECT_FALSE(recovery::use_stepwise_escape_mode(
    true, true, 0U, recovery::WallRegion::Mixed, 0U, true));
  EXPECT_FALSE(recovery::use_stepwise_escape_mode(
    true, true, 0U, recovery::WallRegion::None, 0U, true));
  EXPECT_TRUE(recovery::use_stepwise_escape_mode(
    true, false, 1U, recovery::WallRegion::Mixed, 0U, true));
}

TEST(RecoveryFootprintStepwiseMode, ContinuesAnExistingContactEscape)
{
  EXPECT_TRUE(recovery::use_stepwise_escape_mode(
    true, false, 1U, recovery::WallRegion::Front, 1U));
  EXPECT_TRUE(recovery::use_stepwise_escape_mode(
    true, false, 1U, recovery::WallRegion::Left, 0U));
  EXPECT_FALSE(recovery::use_stepwise_escape_mode(
    true, false, 1U, recovery::WallRegion::Front, 0U));
  EXPECT_FALSE(recovery::use_stepwise_escape_mode(
    true, false, 0U, recovery::WallRegion::Left, 1U));
}

TEST(RecoveryFootprintDynamicClearance, FrontOverlapMaySeparateDuringReverse)
{
  const recovery::FootprintExtents footprint{1.49, 0.51, 0.725, 0.725, 0.05};
  const auto rollout = recovery::generate_reverse_rollout(
    recovery::Pose2D{0.0, 0.0, 0.0}, recovery::ReversePrimitive::Straight,
    recovery::ReverseRolloutParameters{0.4, 0.05, 0.05, 2.14, 0.0});
  ASSERT_TRUE(rollout.valid);

  const auto result = recovery::evaluate_circle_obstacle_clearance(
    footprint, rollout.poses,
    recovery::CircleObstacle{1.76, -0.74, 0.0, 0.0, 1.45}, 4.1);

  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.clear);
  EXPECT_EQ(result.reason, recovery::DynamicClearanceRejectReason::None);
  EXPECT_LT(result.initial_clearance_m, 0.0);
  EXPECT_GT(result.final_clearance_m, result.initial_clearance_m);
}

TEST(RecoveryFootprintDynamicClearance, MovingFrontOverlapMaySeparateDuringReverse)
{
  const recovery::FootprintExtents footprint{1.49, 0.51, 0.725, 0.725, 0.05};
  const auto rollout = recovery::generate_reverse_rollout(
    recovery::Pose2D{0.0, 0.0, 0.0}, recovery::ReversePrimitive::Straight,
    recovery::ReverseRolloutParameters{0.4, 0.05, 0.05, 2.14, 0.0});
  ASSERT_TRUE(rollout.valid);

  // A small positive V2X velocity must not switch this geometry to the coarse
  // rectangular moving corridor. The time-predicted rollout proves that every
  // sample moves the initially overlapping front vehicle farther away.
  const auto result = recovery::evaluate_circle_obstacle_clearance(
    footprint, rollout.poses,
    recovery::CircleObstacle{1.76, -0.74, 0.30, 0.0, 1.45}, 4.1);

  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.clear);
  EXPECT_EQ(result.reason, recovery::DynamicClearanceRejectReason::None);
  EXPECT_LT(result.initial_clearance_m, 0.0);
  EXPECT_GT(result.final_clearance_m, result.initial_clearance_m);
}

TEST(RecoveryFootprintDynamicClearance, ReproducedTailGeometrySeparatesWithSelectedRightTurn)
{
  const recovery::FootprintExtents footprint{1.49, 0.51, 0.725, 0.725, 0.05};
  const auto rollout = recovery::generate_reverse_rollout(
    recovery::Pose2D{0.0, 0.0, 0.0}, recovery::ReversePrimitive::Right,
    recovery::ReverseRolloutParameters{0.4, 0.05, 0.05, 2.14, 0.25});
  ASSERT_TRUE(rollout.valid);

  const auto result = recovery::evaluate_circle_obstacle_clearance(
    footprint, rollout.poses,
    recovery::CircleObstacle{1.76, -0.74, 0.0, 0.0, 1.45}, 4.1);

  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.clear);
  EXPECT_EQ(result.reason, recovery::DynamicClearanceRejectReason::None);
  EXPECT_LT(result.initial_clearance_m, 0.0);
  EXPECT_GT(result.final_clearance_m, result.initial_clearance_m);
}

TEST(RecoveryFootprintDynamicClearance, RearOverlapBlocksWorseningReverse)
{
  const recovery::FootprintExtents footprint{1.49, 0.51, 0.725, 0.725, 0.05};
  const auto rollout = recovery::generate_reverse_rollout(
    recovery::Pose2D{0.0, 0.0, 0.0}, recovery::ReversePrimitive::Straight,
    recovery::ReverseRolloutParameters{0.4, 0.05, 0.05, 2.14, 0.0});
  ASSERT_TRUE(rollout.valid);

  const auto result = recovery::evaluate_circle_obstacle_clearance(
    footprint, rollout.poses,
    recovery::CircleObstacle{-1.76, 0.74, 0.0, 0.0, 1.45}, 4.1);

  EXPECT_TRUE(result.valid);
  EXPECT_FALSE(result.clear);
  EXPECT_EQ(
    result.reason, recovery::DynamicClearanceRejectReason::InitialOverlapWorsened);
  EXPECT_GT(result.rejected_at_distance_m, 0.0);
}

TEST(RecoveryFootprintDynamicClearance, NewOverlapDuringRolloutIsBlocked)
{
  const recovery::FootprintExtents footprint{0.5, 0.5, 0.3, 0.3, 0.0};
  const auto rollout = recovery::generate_reverse_rollout(
    recovery::Pose2D{0.0, 0.0, 0.0}, recovery::ReversePrimitive::Straight,
    recovery::ReverseRolloutParameters{2.0, 0.05, 0.05, 2.14, 0.0});
  ASSERT_TRUE(rollout.valid);

  const auto result = recovery::evaluate_circle_obstacle_clearance(
    footprint, rollout.poses,
    recovery::CircleObstacle{-1.5, 0.0, 0.0, 0.0, 0.2}, 4.1);

  EXPECT_TRUE(result.valid);
  EXPECT_FALSE(result.clear);
  EXPECT_EQ(result.reason, recovery::DynamicClearanceRejectReason::NewOverlap);
  EXPECT_GT(result.rejected_at_distance_m, 0.0);
  EXPECT_LT(result.rejected_at_distance_m, 2.0);
}

TEST(RecoveryFootprintDynamicClearance, InvalidObstacleFailsClosed)
{
  const auto rollout = recovery::generate_reverse_rollout(
    recovery::Pose2D{0.0, 0.0, 0.0}, recovery::ReversePrimitive::Straight,
    recovery::ReverseRolloutParameters{0.4, 0.05, 0.05, 2.14, 0.0});
  ASSERT_TRUE(rollout.valid);

  const auto result = recovery::evaluate_circle_obstacle_clearance(
    compact_footprint(), rollout.poses,
    recovery::CircleObstacle{
      std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 0.0, 0.2},
    4.1);

  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.clear);
  EXPECT_EQ(result.reason, recovery::DynamicClearanceRejectReason::InvalidObstacle);
}

TEST(RecoveryFootprintContactTransition, AllowsOnlyFixedHaloNonIncreasingOccupiedPatch)
{
  auto grid = make_grid();
  set_world_cell(grid, 10.0, 10.0);
  set_world_cell(grid, 11.0, 10.0);
  set_world_cell(grid, 11.0, 11.0);
  set_world_cell(grid, 12.0, 10.0);
  set_world_cell(grid, 13.0, 10.0);
  set_world_cell(grid, 9.0, 10.0, recovery::CellState::Unknown);
  const std::size_t initial = world_cell_index(grid, 10.0, 10.0);
  const std::size_t adjacent = world_cell_index(grid, 11.0, 10.0);
  const std::size_t diagonal = world_cell_index(grid, 11.0, 11.0);
  const std::size_t second_adjacent = world_cell_index(grid, 12.0, 10.0);
  const std::size_t far_on_same_wall = world_cell_index(grid, 13.0, 10.0);
  const std::size_t unknown = world_cell_index(grid, 9.0, 10.0);

  EXPECT_EQ(
    recovery::evaluate_contact_transition(grid, {initial}, {initial}, {adjacent}),
    recovery::RejectReason::None);
  EXPECT_EQ(
    recovery::evaluate_contact_transition(grid, {initial}, {initial}, {diagonal}),
    recovery::RejectReason::None);
  EXPECT_EQ(
    recovery::evaluate_contact_transition(
      grid, {initial, adjacent}, {initial, adjacent}, {adjacent}),
    recovery::RejectReason::None);
  EXPECT_EQ(
    recovery::evaluate_contact_transition(grid, {initial}, {initial}, {initial, adjacent}),
    recovery::RejectReason::ContactWorsened);
  EXPECT_EQ(
    recovery::evaluate_contact_transition(grid, {initial}, {initial}, {far_on_same_wall}),
    recovery::RejectReason::NewContact);
  EXPECT_EQ(
    recovery::evaluate_contact_transition(grid, {initial}, {initial}, {unknown}),
    recovery::RejectReason::Collision);
  EXPECT_EQ(
    recovery::evaluate_contact_transition(grid, {unknown}, {unknown}, {}),
    recovery::RejectReason::Collision);
  EXPECT_EQ(
    recovery::evaluate_contact_transition(grid, {initial}, {initial}, {}),
    recovery::RejectReason::None);
  EXPECT_EQ(
    recovery::evaluate_contact_transition(grid, {}, {}, {initial}),
    recovery::RejectReason::NewContact);
  EXPECT_EQ(
    recovery::evaluate_contact_transition(
      grid, {initial}, {adjacent}, {second_adjacent}),
    recovery::RejectReason::NewContact);
}

TEST(RecoveryFootprintContactTransition, ImprovingStepAllowsBoundedPatchMigration)
{
  auto grid = make_grid();
  set_world_cell(grid, 10.0, 10.0);
  set_world_cell(grid, 11.0, 10.0);
  set_world_cell(grid, 12.0, 10.0);
  set_world_cell(grid, 14.0, 10.0);
  const auto first = world_cell_index(grid, 10.0, 10.0);
  const auto second = world_cell_index(grid, 11.0, 10.0);
  const auto migrated = world_cell_index(grid, 12.0, 10.0);
  const auto disconnected = world_cell_index(grid, 14.0, 10.0);

  EXPECT_EQ(
    recovery::evaluate_improving_contact_transition(
      grid, {first, second}, {second}, {migrated}),
    recovery::RejectReason::None);
  EXPECT_EQ(
    recovery::evaluate_improving_contact_transition(
      grid, {first}, {first}, {first, second}),
    recovery::RejectReason::ContactWorsened);
  EXPECT_EQ(
    recovery::evaluate_improving_contact_transition(
      grid, {first, second}, {second}, {disconnected}),
    recovery::RejectReason::NewContact);
}

TEST(RecoveryFootprintFeasibility, WallBehindRejectsStraightRollout)
{
  auto grid = make_grid();
  set_world_cell(grid, 8.0, 10.0);

  const auto result = recovery::evaluate_reverse_candidate(
    grid, compact_footprint(), recovery::Pose2D{10.0, 10.0, 0.0},
    recovery::ReversePrimitive::Straight, straight_parameters(3.0));

  EXPECT_FALSE(result.feasible);
  EXPECT_EQ(result.reason, recovery::RejectReason::Collision);
  EXPECT_GT(result.rejected_at_distance_m, 0.0);
  EXPECT_LT(result.rejected_at_distance_m, 3.0);
}

TEST(RecoveryFootprintFeasibility, UnknownAndInvalidCellValuesAreOccupied)
{
  auto unknown_grid = make_grid();
  set_world_cell(unknown_grid, 8.0, 10.0, recovery::CellState::Unknown);
  auto result = recovery::evaluate_reverse_candidate(
    unknown_grid, compact_footprint(), recovery::Pose2D{10.0, 10.0, 0.0},
    recovery::ReversePrimitive::Straight, straight_parameters(3.0));
  EXPECT_FALSE(result.feasible);
  EXPECT_EQ(result.reason, recovery::RejectReason::Collision);

  auto invalid_grid = make_grid();
  set_world_cell(
    invalid_grid, 8.0, 10.0, static_cast<recovery::CellState>(42));
  result = recovery::evaluate_reverse_candidate(
    invalid_grid, compact_footprint(), recovery::Pose2D{10.0, 10.0, 0.0},
    recovery::ReversePrimitive::Straight, straight_parameters(3.0));
  EXPECT_FALSE(result.feasible);
  EXPECT_EQ(result.reason, recovery::RejectReason::Collision);
}

TEST(RecoveryFootprintFeasibility, RotationAsymmetricExtentsAndMarginAffectRasterization)
{
  auto grid = make_grid();
  set_world_cell(grid, 5.0, 6.0);
  const recovery::Pose2D pose{5.0, 5.0, kPi / 2.0};
  const recovery::FootprintExtents without_margin{0.4, 0.1, 0.1, 0.1, 0.0};
  const recovery::FootprintExtents with_margin{0.4, 0.1, 0.1, 0.1, 0.2};

  const auto clear_sample = recovery::sample_footprint(grid, without_margin, pose);
  const auto margin_sample = recovery::sample_footprint(grid, with_margin, pose);
  ASSERT_TRUE(clear_sample.valid);
  ASSERT_TRUE(margin_sample.valid);
  EXPECT_TRUE(clear_sample.contact_cells.empty());
  EXPECT_EQ(margin_sample.contact_cells.size(), 1U);
}

TEST(RecoveryFootprintLateralClearance, PullsWallSideTargetTowardReferencePath)
{
  auto grid = make_grid(200U, 200U, 0.1);
  for (std::size_t column = 0U; column < grid.width; ++column) {
    const auto wall = grid.world_to_grid(0.1 * static_cast<double>(column), 10.0);
    ASSERT_TRUE(wall.has_value());
    grid.cells[wall->row * grid.width + wall->column] = recovery::CellState::Occupied;
  }
  const recovery::FootprintExtents footprint{0.5, 0.5, 0.5, 0.5, 0.0};

  const auto result = recovery::clamp_lateral_offset_to_static_map(
    grid, footprint, recovery::Pose2D{10.0, 8.0, 0.0},
    1.40, 0.0, 0.25, 0.05);

  ASSERT_TRUE(result.valid);
  ASSERT_TRUE(result.feasible);
  EXPECT_TRUE(result.adjusted);
  EXPECT_LT(result.lateral_offset_m, 1.40);
  EXPECT_GT(result.lateral_offset_m, 0.0);
  EXPECT_GT(result.checked_pose_count, 1U);
}

TEST(RecoveryFootprintLateralClearance, KeepsAlreadyClearTarget)
{
  auto grid = make_grid(200U, 200U, 0.1);
  for (std::size_t column = 0U; column < grid.width; ++column) {
    const auto wall = grid.world_to_grid(0.1 * static_cast<double>(column), 10.0);
    ASSERT_TRUE(wall.has_value());
    grid.cells[wall->row * grid.width + wall->column] = recovery::CellState::Occupied;
  }

  const auto result = recovery::clamp_lateral_offset_to_static_map(
    grid, recovery::FootprintExtents{0.5, 0.5, 0.5, 0.5, 0.0},
    recovery::Pose2D{10.0, 8.0, 0.0}, 0.50, 0.0, 0.25, 0.05);

  ASSERT_TRUE(result.valid);
  ASSERT_TRUE(result.feasible);
  EXPECT_FALSE(result.adjusted);
  EXPECT_DOUBLE_EQ(result.lateral_offset_m, 0.50);
  EXPECT_EQ(result.checked_pose_count, 1U);
}

TEST(RecoveryFootprintLateralClearance, InvalidSearchFailsClosed)
{
  const auto result = recovery::clamp_lateral_offset_to_static_map(
    make_grid(), compact_footprint(), recovery::Pose2D{10.0, 10.0, 0.0},
    1.0, 0.0, 0.2, 0.0);

  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.feasible);
}

TEST(RecoveryFootprintLateralInterval, SelectsConnectedComponentNearestPreference)
{
  auto grid = make_grid(200U, 200U, 0.1);
  for (std::size_t column = 0U; column < grid.width; ++column) {
    set_world_cell(grid, 0.1 * static_cast<double>(column), 10.0);
  }

  const auto result = recovery::find_clear_lateral_interval(
    grid, recovery::FootprintExtents{0.2, 0.2, 0.2, 0.2, 0.0},
    recovery::Pose2D{10.0, 10.0, 0.0}, -2.0, 2.0, 1.0, 0.0, 0.05);

  ASSERT_TRUE(result.valid);
  ASSERT_TRUE(result.feasible);
  EXPECT_TRUE(result.preferred_lateral_contained);
  EXPECT_GT(result.lower_lateral_offset_m, 0.0);
  EXPECT_GE(result.upper_lateral_offset_m, 1.0);
}

TEST(RecoveryFootprintLateralInterval, AppliesAdditionalWallClearance)
{
  auto grid = make_grid(200U, 200U, 0.1);
  for (std::size_t column = 0U; column < grid.width; ++column) {
    set_world_cell(grid, 0.1 * static_cast<double>(column), 10.0);
  }
  const recovery::Pose2D pose{10.0, 8.0, 0.0};
  const recovery::FootprintExtents footprint{0.5, 0.5, 0.5, 0.5, 0.0};
  const auto physical = recovery::find_clear_lateral_interval(
    grid, footprint, pose, -1.0, 1.5, 0.0, 0.0, 0.05);
  const auto reserved = recovery::find_clear_lateral_interval(
    grid, footprint, pose, -1.0, 1.5, 0.0, 0.25, 0.05);

  ASSERT_TRUE(physical.feasible);
  ASSERT_TRUE(reserved.feasible);
  EXPECT_LT(reserved.upper_lateral_offset_m, physical.upper_lateral_offset_m);
}

TEST(RecoveryFootprintLateralInterval, InvalidInputFailsClosed)
{
  const auto result = recovery::find_clear_lateral_interval(
    make_grid(), compact_footprint(), recovery::Pose2D{10.0, 10.0, 0.0},
    1.0, -1.0, 0.0, 0.0, 0.05);

  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.feasible);
}

TEST(RecoveryFootprintPathClearance, AcceptsClearSweptPath)
{
  const std::vector<recovery::Pose2D> path{
    {5.0, 5.0, 0.0},
    {7.0, 5.5, 0.1},
    {9.0, 6.0, 0.2}};

  const auto result = recovery::evaluate_clear_footprint_path(
    make_grid(200U, 200U, 0.1), compact_footprint(), path, 0.05);

  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.clear);
  EXPECT_EQ(result.reason, recovery::RejectReason::None);
  EXPECT_GT(result.checked_pose_count, path.size());
}

TEST(RecoveryFootprintPathClearance, DetectsCollisionBetweenClearEndpoints)
{
  auto grid = make_grid(200U, 200U, 0.1);
  set_world_cell(grid, 7.0, 5.0);
  const auto footprint = compact_footprint();
  const std::vector<recovery::Pose2D> path{
    {5.0, 5.0, 0.0},
    {9.0, 5.0, 0.0}};
  ASSERT_TRUE(recovery::sample_footprint(grid, footprint, path.front()).contact_cells.empty());
  ASSERT_TRUE(recovery::sample_footprint(grid, footprint, path.back()).contact_cells.empty());

  const auto result =
    recovery::evaluate_clear_footprint_path(grid, footprint, path, 0.05);

  EXPECT_TRUE(result.valid);
  EXPECT_FALSE(result.clear);
  EXPECT_EQ(result.reason, recovery::RejectReason::Collision);
  EXPECT_EQ(result.rejected_path_index, 1U);
  EXPECT_GT(result.checked_pose_count, 1U);
}

TEST(RecoveryFootprintPathClearance, RejectsOutOfMapAndInvalidStep)
{
  const auto grid = make_grid(20U, 20U, 0.1);
  const auto footprint = compact_footprint();
  const std::vector<recovery::Pose2D> leaving_map{
    {1.0, 1.0, 0.0},
    {-1.0, 1.0, 0.0}};

  const auto out_of_map =
    recovery::evaluate_clear_footprint_path(grid, footprint, leaving_map, 0.05);
  EXPECT_TRUE(out_of_map.valid);
  EXPECT_FALSE(out_of_map.clear);
  EXPECT_EQ(out_of_map.reason, recovery::RejectReason::OutOfMap);

  const auto invalid_step =
    recovery::evaluate_clear_footprint_path(grid, footprint, leaving_map, 0.2);
  EXPECT_FALSE(invalid_step.valid);
  EXPECT_FALSE(invalid_step.clear);
  EXPECT_EQ(invalid_step.reason, recovery::RejectReason::InvalidRollout);
}

TEST(RecoveryFootprintFeasibility, CandidateLeavingMapIsRejectedWithoutEdgeClamping)
{
  const auto result = recovery::evaluate_reverse_candidate(
    make_grid(10U, 10U), recovery::FootprintExtents{0.4, 0.4, 0.3, 0.3, 0.0},
    recovery::Pose2D{1.0, 5.0, 0.0}, recovery::ReversePrimitive::Straight,
    straight_parameters(2.0));

  EXPECT_FALSE(result.feasible);
  EXPECT_EQ(result.reason, recovery::RejectReason::OutOfMap);
}

TEST(RecoveryFootprintFeasibility, InitialOutOfMapHasDistinctRejectReason)
{
  const auto result = recovery::evaluate_reverse_candidate(
    make_grid(10U, 10U), recovery::FootprintExtents{0.6, 0.6, 0.4, 0.4, 0.0},
    recovery::Pose2D{0.0, 0.0, kPi / 4.0}, recovery::ReversePrimitive::Straight,
    straight_parameters(1.0));

  EXPECT_FALSE(result.feasible);
  EXPECT_EQ(result.reason, recovery::RejectReason::InitialOutOfMap);
}

TEST(RecoveryFootprintFeasibility, SweptInterpolationFindsCollisionBetweenClearEndpoints)
{
  auto grid = make_grid();
  set_world_cell(grid, 5.0, 10.0);
  auto parameters = straight_parameters(4.0);
  parameters.rollout_step_m = 4.0;
  parameters.swept_step_m = 0.1;

  const auto start_sample = recovery::sample_footprint(
    grid, compact_footprint(), recovery::Pose2D{7.0, 10.0, 0.0});
  const auto end_sample = recovery::sample_footprint(
    grid, compact_footprint(), recovery::Pose2D{3.0, 10.0, 0.0});
  ASSERT_TRUE(start_sample.contact_cells.empty());
  ASSERT_TRUE(end_sample.contact_cells.empty());

  const auto result = recovery::evaluate_reverse_candidate(
    grid, compact_footprint(), recovery::Pose2D{7.0, 10.0, 0.0},
    recovery::ReversePrimitive::Straight, parameters);
  EXPECT_FALSE(result.feasible);
  EXPECT_EQ(result.reason, recovery::RejectReason::Collision);
  EXPECT_GT(result.checked_pose_count, 2U);
}

TEST(RecoveryFootprintInitialContact, ReverseEscapeMayClearExistingContact)
{
  auto grid = make_grid();
  set_world_cell(grid, 6.0, 10.0);
  const recovery::FootprintExtents footprint{0.6, 0.2, 0.2, 0.2, 0.0};

  const auto result = recovery::evaluate_reverse_candidate(
    grid, footprint, recovery::Pose2D{5.0, 10.0, 0.0},
    recovery::ReversePrimitive::Straight, straight_parameters(1.0));

  EXPECT_TRUE(result.feasible);
  EXPECT_EQ(result.initial_contact_count, 1U);
  EXPECT_EQ(result.final_contact_count, 0U);
}

TEST(RecoveryFootprintInitialContact, FullFourMeterReverseMayClearExistingContact)
{
  auto grid = make_grid(150U, 150U, 0.1);
  set_world_cell(grid, 8.6, 5.0);
  const recovery::FootprintExtents footprint{0.6, 0.2, 0.2, 0.2, 0.0};
  auto parameters = straight_parameters(4.0);
  parameters.rollout_step_m = 0.05;
  parameters.swept_step_m = 0.05;

  const auto result = recovery::evaluate_recovery_candidate(
    grid, footprint, recovery::Pose2D{8.0, 5.0, 0.0},
    recovery::ReversePrimitive::Straight, parameters,
    recovery::ContactEscapePolicy::RequireImprovement, 0.05);

  EXPECT_TRUE(result.feasible);
  EXPECT_EQ(result.initial_contact_count, 1U);
  EXPECT_EQ(result.final_contact_count, 0U);
  EXPECT_GT(result.contact_reduction, 0U);
  EXPECT_GE(result.checked_pose_count, 81U);
}

TEST(RecoveryFootprintInitialContact, ReverseCannotPassThroughRearContact)
{
  auto grid = make_grid(20U, 20U, 1.0);
  set_world_cell(grid, 4.0, 10.0);
  recovery::ReverseRolloutParameters parameters{2.3, 0.5, 0.1, 1.0, 0.0};

  const auto result = recovery::evaluate_reverse_candidate(
    grid, recovery::FootprintExtents{0.6, 0.6, 0.2, 0.2, 0.0},
    recovery::Pose2D{5.0, 10.0, 0.0}, recovery::ReversePrimitive::Straight,
    parameters);

  EXPECT_FALSE(result.feasible);
  EXPECT_EQ(result.initial_contact_count, 1U);
  EXPECT_EQ(result.reason, recovery::RejectReason::InitialContactNotForward);
}

TEST(RecoveryFootprintInitialContact, ForwardEscapeMayClearExistingRearContact)
{
  auto grid = make_grid(20U, 20U, 1.0);
  set_world_cell(grid, 4.0, 10.0);
  recovery::ReverseRolloutParameters parameters{1.5, 0.25, 0.1, 1.0, 0.0};

  const auto result = recovery::evaluate_reverse_candidate(
    grid, recovery::FootprintExtents{0.6, 0.6, 0.2, 0.2, 0.0},
    recovery::Pose2D{5.0, 10.0, 0.0}, recovery::ReversePrimitive::ForwardStraight,
    parameters);

  EXPECT_TRUE(result.feasible);
  EXPECT_EQ(result.initial_contact_count, 1U);
  EXPECT_EQ(result.final_contact_count, 0U);
  ASSERT_FALSE(result.rollout.empty());
  EXPECT_GT(result.rollout.back().pose.x_m, 5.0);
}

TEST(RecoveryFootprintInitialContact, ForwardCannotPassThroughFrontContact)
{
  auto grid = make_grid(20U, 20U, 1.0);
  set_world_cell(grid, 6.0, 10.0);
  recovery::ReverseRolloutParameters parameters{1.5, 0.25, 0.1, 1.0, 0.0};

  const auto result = recovery::evaluate_reverse_candidate(
    grid, recovery::FootprintExtents{0.6, 0.6, 0.2, 0.2, 0.0},
    recovery::Pose2D{5.0, 10.0, 0.0}, recovery::ReversePrimitive::ForwardStraight,
    parameters);

  EXPECT_FALSE(result.feasible);
  EXPECT_EQ(result.reason, recovery::RejectReason::InitialContactNotRear);
}

TEST(RecoveryFootprintInitialContact, ShortStepMayRequireMeasuredContactImprovement)
{
  auto grid = make_grid(120U, 120U, 0.1);
  for (const double x_m : {4.4, 4.5, 4.6, 4.7}) {
    for (const double y_m : {4.8, 4.9, 5.0, 5.1, 5.2}) {
      set_world_cell(grid, x_m, y_m);
    }
  }
  recovery::ReverseRolloutParameters parameters{0.2, 0.05, 0.05, 1.0, 0.25};
  const auto result = recovery::evaluate_recovery_candidate(
    grid, recovery::FootprintExtents{0.5, 0.5, 0.5, 0.5, 0.0},
    recovery::Pose2D{5.0, 5.0, 0.0}, recovery::ReversePrimitive::ForwardStraight,
    parameters, recovery::ContactEscapePolicy::RequireImprovement, 0.05);

  EXPECT_TRUE(result.feasible);
  EXPECT_GT(result.initial_contact_count, result.final_contact_count);
  EXPECT_GT(result.contact_reduction, 0U);
  EXPECT_GT(result.final_contact_count, 0U);
}

TEST(RecoveryFootprintInitialContact, CommittedStepMayRemainNonWorseningUntilEndpoint)
{
  auto grid = make_grid(30U, 30U, 1.0);
  set_world_cell(grid, 10.5, 10.5);
  recovery::ReverseRolloutParameters parameters{0.1, 0.1, 0.1, 1.0, 0.0};
  const auto result = recovery::evaluate_recovery_candidate(
    grid, recovery::FootprintExtents{0.2, 0.2, 0.2, 0.2, 0.0},
    recovery::Pose2D{10.5, 10.5, 0.0}, recovery::ReversePrimitive::Straight,
    parameters, recovery::ContactEscapePolicy::AllowNonWorsening, 0.0);

  EXPECT_TRUE(result.feasible);
  EXPECT_GT(result.initial_contact_count, 0U);
  EXPECT_EQ(result.final_contact_count, result.initial_contact_count);
  EXPECT_EQ(result.contact_reduction, 0U);
}

TEST(RecoveryFootprintWallRegion, ClassifiesVehicleRelativeSidesAndFailsClosedAtCorner)
{
  const recovery::Pose2D pose{10.0, 10.0, 0.0};
  const recovery::FootprintExtents footprint{1.0, 1.0, 1.0, 1.0, 0.0};
  const auto classify = [&](const double x_m, const double y_m) {
      auto grid = make_grid(200U, 200U, 0.1);
      set_world_cell(grid, x_m, y_m);
      return recovery::classify_nearby_wall(grid, footprint, pose, 0.5, 0.02);
    };

  EXPECT_EQ(classify(11.2, 10.0).region, recovery::WallRegion::Front);
  EXPECT_EQ(classify(8.8, 10.0).region, recovery::WallRegion::Rear);
  EXPECT_EQ(classify(10.0, 11.2).region, recovery::WallRegion::Left);
  EXPECT_EQ(classify(10.0, 8.8).region, recovery::WallRegion::Right);
  EXPECT_EQ(classify(11.2, 11.2).region, recovery::WallRegion::Mixed);

  const auto clear = recovery::classify_nearby_wall(
    make_grid(200U, 200U, 0.1), footprint, pose, 0.5, 0.02);
  EXPECT_TRUE(clear.valid);
  EXPECT_EQ(clear.region, recovery::WallRegion::None);
}

TEST(RecoveryFootprintInitialContact, NewWallContactDuringEscapeIsRejected)
{
  auto grid = make_grid();
  set_world_cell(grid, 6.0, 10.0);
  set_world_cell(grid, 3.0, 10.0);
  const recovery::FootprintExtents footprint{0.6, 0.2, 0.2, 0.2, 0.0};

  const auto result = recovery::evaluate_reverse_candidate(
    grid, footprint, recovery::Pose2D{5.0, 10.0, 0.0},
    recovery::ReversePrimitive::Straight, straight_parameters(2.5));

  EXPECT_FALSE(result.feasible);
  EXPECT_EQ(result.reason, recovery::RejectReason::NewContact);
}

TEST(RecoveryFootprintInitialContact, CandidateMustClearContactByItsEnd)
{
  auto grid = make_grid();
  set_world_cell(grid, 6.0, 10.0);
  const recovery::FootprintExtents footprint{0.6, 0.2, 0.2, 0.2, 0.0};

  const auto result = recovery::evaluate_reverse_candidate(
    grid, footprint, recovery::Pose2D{5.0, 10.0, 0.0},
    recovery::ReversePrimitive::Straight, straight_parameters(0.05));

  EXPECT_FALSE(result.feasible);
  EXPECT_EQ(result.reason, recovery::RejectReason::InitialContactNotCleared);
  EXPECT_EQ(result.final_contact_count, 1U);
}

TEST(RecoveryFootprintInitialContact, CurvedEscapeIsRejectedWithoutDirectionalProof)
{
  auto grid = make_grid(30U, 30U, 1.0);
  set_world_cell(grid, 11.0, 15.0);
  const recovery::FootprintExtents footprint{0.6, 0.2, 0.2, 0.2, 0.0};
  recovery::ReverseRolloutParameters parameters{
    2.0 * kPi / std::tan(0.7), 0.25, 0.05, 1.0, 0.7};

  const auto result = recovery::evaluate_reverse_candidate(
    grid, footprint, recovery::Pose2D{10.0, 15.0, 0.0},
    recovery::ReversePrimitive::Left, parameters);

  EXPECT_FALSE(result.feasible);
  EXPECT_EQ(result.initial_contact_count, 1U);
  EXPECT_EQ(result.reason, recovery::RejectReason::InvalidRollout);
}

TEST(RecoveryFootprintRollout, CurvedPrimitivesUseReverseBicycleKinematics)
{
  const recovery::Pose2D initial{10.0, 10.0, 0.0};
  auto parameters = straight_parameters(1.0);
  parameters.rollout_step_m = 0.25;

  const auto straight = recovery::generate_reverse_rollout(
    initial, recovery::ReversePrimitive::Straight, parameters);
  const auto left = recovery::generate_reverse_rollout(
    initial, recovery::ReversePrimitive::Left, parameters);
  const auto right = recovery::generate_reverse_rollout(
    initial, recovery::ReversePrimitive::Right, parameters);
  ASSERT_TRUE(straight.valid);
  ASSERT_TRUE(left.valid);
  ASSERT_TRUE(right.valid);

  const auto & straight_end = straight.poses.back().pose;
  const auto & left_end = left.poses.back().pose;
  const auto & right_end = right.poses.back().pose;
  EXPECT_NEAR(straight_end.x_m, 9.0, 1e-12);
  EXPECT_NEAR(straight_end.y_m, 10.0, 1e-12);
  EXPECT_LT(left_end.yaw_rad, 0.0);
  EXPECT_GT(left_end.y_m, 10.0);
  EXPECT_GT(right_end.yaw_rad, 0.0);
  EXPECT_LT(right_end.y_m, 10.0);
  EXPECT_NEAR(left_end.x_m, right_end.x_m, 1e-12);
  EXPECT_NEAR(left_end.y_m - 10.0, 10.0 - right_end.y_m, 1e-12);
}

TEST(RecoveryFootprintRollout, CurvedCandidatesAreCheckedWithTheSameSafetyRules)
{
  auto parameters = straight_parameters(2.0);
  parameters.rollout_step_m = 0.5;
  const auto grid = make_grid(30U, 30U, 1.0);
  const recovery::Pose2D initial{15.0, 15.0, 0.0};

  const auto left = recovery::evaluate_reverse_candidate(
    grid, compact_footprint(), initial, recovery::ReversePrimitive::Left, parameters);
  const auto right = recovery::evaluate_reverse_candidate(
    grid, compact_footprint(), initial, recovery::ReversePrimitive::Right, parameters);
  EXPECT_TRUE(left.feasible);
  EXPECT_TRUE(right.feasible);
  EXPECT_DOUBLE_EQ(left.steering_angle_rad, parameters.steering_angle_rad);
  EXPECT_DOUBLE_EQ(right.steering_angle_rad, -parameters.steering_angle_rad);
  EXPECT_GT(left.checked_pose_count, left.rollout.size());
  EXPECT_GT(right.checked_pose_count, right.rollout.size());
}

TEST(RecoveryFootprintValidation, RejectsInvalidGridFootprintPoseAndRollout)
{
  auto invalid_grid = make_grid();
  invalid_grid.cells.pop_back();
  auto result = recovery::evaluate_reverse_candidate(
    invalid_grid, compact_footprint(), recovery::Pose2D{10.0, 10.0, 0.0},
    recovery::ReversePrimitive::Straight, straight_parameters());
  EXPECT_EQ(result.reason, recovery::RejectReason::InvalidGrid);

  auto invalid_footprint = compact_footprint();
  invalid_footprint.margin_m = -0.1;
  result = recovery::evaluate_reverse_candidate(
    make_grid(), invalid_footprint, recovery::Pose2D{10.0, 10.0, 0.0},
    recovery::ReversePrimitive::Straight, straight_parameters());
  EXPECT_EQ(result.reason, recovery::RejectReason::InvalidFootprint);

  result = recovery::evaluate_reverse_candidate(
    make_grid(), compact_footprint(),
    recovery::Pose2D{std::numeric_limits<double>::quiet_NaN(), 10.0, 0.0},
    recovery::ReversePrimitive::Straight, straight_parameters());
  EXPECT_EQ(result.reason, recovery::RejectReason::InvalidInitialPose);

  auto invalid_parameters = straight_parameters();
  invalid_parameters.reverse_distance_m = 0.0;
  result = recovery::evaluate_reverse_candidate(
    make_grid(), compact_footprint(), recovery::Pose2D{10.0, 10.0, 0.0},
    recovery::ReversePrimitive::Straight, invalid_parameters);
  EXPECT_EQ(result.reason, recovery::RejectReason::InvalidRollout);

  invalid_parameters = straight_parameters();
  invalid_parameters.steering_angle_rad = 0.0;
  result = recovery::evaluate_reverse_candidate(
    make_grid(), compact_footprint(), recovery::Pose2D{10.0, 10.0, 0.0},
    recovery::ReversePrimitive::Left, invalid_parameters);
  EXPECT_EQ(result.reason, recovery::RejectReason::InvalidRollout);

  invalid_parameters = straight_parameters();
  invalid_parameters.swept_step_m = 2.0;
  result = recovery::evaluate_reverse_candidate(
    make_grid(), compact_footprint(), recovery::Pose2D{10.0, 10.0, 0.0},
    recovery::ReversePrimitive::Straight, invalid_parameters);
  EXPECT_EQ(result.reason, recovery::RejectReason::InvalidRollout);

  result = recovery::evaluate_reverse_candidate(
    make_grid(), compact_footprint(), recovery::Pose2D{10.0, 10.0, 0.0},
    static_cast<recovery::ReversePrimitive>(99), straight_parameters());
  EXPECT_EQ(result.reason, recovery::RejectReason::InvalidRollout);
}

TEST(RecoveryFootprintValidation, RejectsUnboundedSamplingRequestsDeterministically)
{
  auto parameters = straight_parameters(10.0);
  parameters.rollout_step_m = 1e-12;
  const auto result = recovery::evaluate_reverse_candidate(
    make_grid(), compact_footprint(), recovery::Pose2D{10.0, 10.0, 0.0},
    recovery::ReversePrimitive::Straight, parameters);

  EXPECT_FALSE(result.feasible);
  EXPECT_EQ(result.reason, recovery::RejectReason::SampleLimitExceeded);
}

TEST(RecoveryFootprintStrings, RejectReasonsAndPrimitivesHaveStableNames)
{
  EXPECT_STREQ(
    recovery::to_string(recovery::RejectReason::NotEvaluated), "not_evaluated");
  EXPECT_STREQ(recovery::to_string(recovery::RejectReason::NewContact), "new_contact");
  EXPECT_STREQ(
    recovery::to_string(recovery::RejectReason::InitialContactNotForward),
    "initial_contact_not_forward");
  EXPECT_STREQ(
    recovery::to_string(recovery::ReversePrimitive::Right), "reverse_right");
  EXPECT_STREQ(
    recovery::to_string(recovery::ReversePrimitive::ForwardLeft), "forward_left");
  EXPECT_STREQ(
    recovery::to_string(recovery::DynamicClearanceRejectReason::InitialOverlapWorsened),
    "initial_overlap_worsened");
  EXPECT_STREQ(
    recovery::to_string(static_cast<recovery::RejectReason>(999)), "unknown");
}

}  // namespace
