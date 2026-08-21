#include <gtest/gtest.h>

#include <multi_purpose_mpc_ros/mpc_stage_geometry.hpp>

#include <vector>

namespace geometry = multi_purpose_mpc_ros::mpc_stage_geometry;

TEST(MpcStageGeometry, AlignsNonUniformDynamicsAndStateWaypoints)
{
  const std::vector<geometry::Point2d> path{
    {0.0, 0.0}, {1.0, 0.0}, {3.0, 0.0}, {6.0, 0.0}, {10.0, 0.0}};

  const auto result = geometry::build(path, 1, 3U, false);

  ASSERT_TRUE(result.valid) << result.reject_reason;
  ASSERT_EQ(result.stages.size(), 3U);
  EXPECT_EQ(result.stages[0].transition_from_waypoint, 1);
  EXPECT_EQ(result.stages[0].state_waypoint, 2);
  EXPECT_DOUBLE_EQ(result.stages[0].transition_distance_m, 2.0);
  EXPECT_DOUBLE_EQ(result.stages[0].cumulative_distance_m, 2.0);
  EXPECT_EQ(result.stages[1].state_waypoint, 3);
  EXPECT_DOUBLE_EQ(result.stages[1].transition_distance_m, 3.0);
  EXPECT_DOUBLE_EQ(result.stages[1].cumulative_distance_m, 5.0);
  EXPECT_EQ(result.stages[2].state_waypoint, 4);
  EXPECT_DOUBLE_EQ(result.stages[2].transition_distance_m, 4.0);
  EXPECT_DOUBLE_EQ(result.stages[2].cumulative_distance_m, 9.0);
}

TEST(MpcStageGeometry, WrapsCircularHorizonWithoutChangingStageContract)
{
  const std::vector<geometry::Point2d> path{
    {0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}, {0.0, 2.0}};

  const auto result = geometry::build(path, 3, 2U, true);

  ASSERT_TRUE(result.valid) << result.reject_reason;
  ASSERT_EQ(result.stages.size(), 2U);
  EXPECT_EQ(result.stages[0].transition_from_waypoint, 3);
  EXPECT_EQ(result.stages[0].state_waypoint, 0);
  EXPECT_DOUBLE_EQ(result.stages[0].transition_distance_m, 2.0);
  EXPECT_EQ(result.stages[1].transition_from_waypoint, 0);
  EXPECT_EQ(result.stages[1].state_waypoint, 1);
  EXPECT_DOUBLE_EQ(result.stages[1].cumulative_distance_m, 4.0);
}

TEST(MpcStageGeometry, RejectsNonCircularHorizonPastPathEnd)
{
  const std::vector<geometry::Point2d> path{{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}};

  const auto result = geometry::build(path, 1, 2U, false);

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reject_reason, "non-circular horizon exceeds path");
}
