#include <gtest/gtest.h>

#include <imu_gnss_poser/heading_reference.hpp>

#include <cmath>
#include <limits>
#include <sstream>
#include <vector>

namespace
{

using imu_gnss_poser::InitialPoseCovariance;
using imu_gnss_poser::Point2D;

TEST(HeadingReference, LoadsLegacyXYColumns)
{
  std::istringstream csv{"x,y,z\n1.0,2.0,0.0\n3.0,4.0,0.0\n"};
  const auto points = imu_gnss_poser::load_path_points_csv(csv);
  ASSERT_EQ(points.size(), 2U);
  EXPECT_DOUBLE_EQ(points[0].x, 1.0);
  EXPECT_DOUBLE_EQ(points[0].y, 2.0);
}

TEST(HeadingReference, LoadsMpcTrajectoryColumnsByHeader)
{
  std::istringstream csv{
    "s_m,x_m,y_m,psi_rad\n0.0,10.0,20.0,1.5\n1.0,invalid,21.0,1.6\n2.0,12.0,22.0,1.7\n"};
  const auto points = imu_gnss_poser::load_path_points_csv(csv);
  ASSERT_EQ(points.size(), 2U);
  EXPECT_DOUBLE_EQ(points[0].x, 10.0);
  EXPECT_DOUBLE_EQ(points[0].y, 20.0);
  EXPECT_DOUBLE_EQ(points[1].x, 12.0);
  EXPECT_DOUBLE_EQ(points[1].y, 22.0);
}

TEST(HeadingReference, RejectsCsvWithoutCoordinateColumns)
{
  std::istringstream csv{"s_m,psi_rad\n0.0,1.5\n"};
  EXPECT_THROW(imu_gnss_poser::load_path_points_csv(csv), std::runtime_error);
}

TEST(HeadingReference, FindsClosestFinitePoint)
{
  const std::vector<Point2D> points{
    {0.0, 0.0},
    {std::numeric_limits<double>::quiet_NaN(), 0.0},
    {4.0, 0.0}};
  const auto index = imu_gnss_poser::find_closest_finite_point(points, 3.8, 0.1);
  ASSERT_TRUE(index.has_value());
  EXPECT_EQ(index.value(), 2U);
}

TEST(HeadingReference, UsesForwardNonzeroSegment)
{
  const std::vector<Point2D> points{{0.0, 0.0}, {0.0, 0.0}, {0.0, 2.0}};
  const auto yaw = imu_gnss_poser::compute_path_yaw(points, 0U);
  ASSERT_TRUE(yaw.has_value());
  EXPECT_NEAR(yaw.value(), std::acos(-1.0) * 0.5, 1.0e-12);
}

TEST(HeadingReference, UsesPreviousSegmentAtPathEnd)
{
  const std::vector<Point2D> points{{0.0, 0.0}, {-2.0, 0.0}};
  const auto yaw = imu_gnss_poser::compute_path_yaw(points, 1U);
  ASSERT_TRUE(yaw.has_value());
  EXPECT_NEAR(std::abs(yaw.value()), std::acos(-1.0), 1.0e-12);
}

TEST(HeadingReference, BuildsRacelineAlignedInitialPose)
{
  geometry_msgs::msg::PoseWithCovarianceStamped gnss;
  gnss.header.frame_id = "map";
  gnss.pose.pose.position.x = 0.1;
  gnss.pose.pose.position.y = 0.2;
  gnss.pose.pose.position.z = 6.5;
  gnss.pose.pose.orientation.z = 1.0;
  gnss.pose.pose.orientation.w = 0.0;
  const std::vector<Point2D> points{{0.0, 0.0}, {2.0, 0.0}};

  const auto result = imu_gnss_poser::make_raceline_initial_pose(
    gnss, points, InitialPoseCovariance{0.25, 0.5, 0.75});

  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(result->yaw_rad, 0.0);
  EXPECT_EQ(result->reference_index, 0U);
  EXPECT_EQ(result->pose.header.frame_id, "map");
  EXPECT_DOUBLE_EQ(result->pose.pose.pose.position.z, 6.5);
  EXPECT_DOUBLE_EQ(result->pose.pose.pose.orientation.z, 0.0);
  EXPECT_DOUBLE_EQ(result->pose.pose.pose.orientation.w, 1.0);
  EXPECT_DOUBLE_EQ(result->pose.pose.covariance[7 * 0], 0.25);
  EXPECT_DOUBLE_EQ(result->pose.pose.covariance[7 * 1], 0.5);
  EXPECT_DOUBLE_EQ(result->pose.pose.covariance[7 * 5], 0.75);
}

TEST(HeadingReference, RejectsInvalidInputInsteadOfUsingRawYaw)
{
  geometry_msgs::msg::PoseWithCovarianceStamped gnss;
  gnss.pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  gnss.pose.pose.orientation.z = 1.0;
  const std::vector<Point2D> points{{0.0, 0.0}, {1.0, 0.0}};

  EXPECT_FALSE(
    imu_gnss_poser::make_raceline_initial_pose(
      gnss, points, InitialPoseCovariance{0.25, 0.25, 0.5}).has_value());
  gnss.pose.pose.position.x = 0.0;
  EXPECT_FALSE(
    imu_gnss_poser::make_raceline_initial_pose(
      gnss, {{0.0, 0.0}}, InitialPoseCovariance{0.25, 0.25, 0.5}).has_value());
}

}  // namespace
