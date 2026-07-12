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
  EXPECT_STREQ(recovery::to_string(recovery::RejectReason::NewContact), "new_contact");
  EXPECT_STREQ(
    recovery::to_string(recovery::RejectReason::InitialContactNotForward),
    "initial_contact_not_forward");
  EXPECT_STREQ(
    recovery::to_string(recovery::ReversePrimitive::Right), "reverse_right");
  EXPECT_STREQ(
    recovery::to_string(static_cast<recovery::RejectReason>(999)), "unknown");
}

}  // namespace
