#include <gtest/gtest.h>

#include <multi_purpose_mpc_ros/mpc_stage_geometry.hpp>

#include <cmath>
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

TEST(MpcStageGeometry, SamplesCourseFrameAtSolvedProgressInsteadOfNominalStage)
{
  const std::vector<geometry::CourseFrameKnot> knots{
    {10.0, 0.0, 0.0, 0.0, 25},
    {12.0, 2.0, 0.0, 0.0, 26},
    {14.0, 4.0, 0.0, 0.0, 27}};

  const auto sample = geometry::sample_course_frame(knots, 11.0);

  ASSERT_TRUE(sample.has_value());
  EXPECT_DOUBLE_EQ(sample->progress_m, 11.0);
  EXPECT_DOUBLE_EQ(sample->x_m, 1.0);
  EXPECT_DOUBLE_EQ(sample->y_m, 0.0);
  EXPECT_DOUBLE_EQ(sample->heading_rad, 0.0);
  EXPECT_EQ(sample->lower_waypoint, 25);
  EXPECT_EQ(sample->upper_waypoint, 26);
  EXPECT_DOUBLE_EQ(sample->interpolation_ratio, 0.5);
}

TEST(MpcStageGeometry, InterpolatesHeadingAcrossAngleWrap)
{
  constexpr double kPi = 3.14159265358979323846;
  const std::vector<geometry::CourseFrameKnot> knots{
    {0.0, 0.0, 0.0, 179.0 * kPi / 180.0, 8},
    {2.0, 0.0, 2.0, -179.0 * kPi / 180.0, 9}};

  const auto sample = geometry::sample_course_frame(knots, 1.0);

  ASSERT_TRUE(sample.has_value());
  EXPECT_NEAR(std::abs(sample->heading_rad), kPi, 1e-12);
}

TEST(MpcStageGeometry, RejectsProgressOutsideProvenanceWindow)
{
  const std::vector<geometry::CourseFrameKnot> knots{
    {10.0, 0.0, 0.0, 0.0, 25},
    {12.0, 2.0, 0.0, 0.0, 26}};

  EXPECT_FALSE(geometry::sample_course_frame(knots, 9.9).has_value());
  EXPECT_FALSE(geometry::sample_course_frame(knots, 12.1).has_value());
}

TEST(MpcStageGeometry, ClampsOnlyWithinAcceptedSolverTolerance)
{
  const std::vector<geometry::CourseFrameKnot> knots{
    {10.0, 0.0, 0.0, 0.0, 25},
    {12.0, 2.0, 0.0, 0.0, 26}};

  const auto within_tolerance = geometry::sample_course_frame(
    knots, 9.999, 0.002);
  ASSERT_TRUE(within_tolerance.has_value());
  EXPECT_DOUBLE_EQ(within_tolerance->progress_m, 10.0);
  EXPECT_DOUBLE_EQ(within_tolerance->x_m, 0.0);

  EXPECT_FALSE(geometry::sample_course_frame(
      knots, 9.997, 0.002).has_value());
}
