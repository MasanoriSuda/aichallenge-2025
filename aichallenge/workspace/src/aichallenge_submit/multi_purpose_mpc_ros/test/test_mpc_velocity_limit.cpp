#include <gtest/gtest.h>

#include <multi_purpose_mpc_ros/mpc_velocity_limit.hpp>

#include <limits>
#include <stdexcept>

TEST(MpcVelocityLimit, RampsDownByReachableDeceleration)
{
  const auto limits = multi_purpose_mpc_ros::mpc_velocity_limit::build_reachable_limits(
    multi_purpose_mpc_ros::mpc_velocity_limit::ReachableLimitRequest{
      5, 4.5, 3.0, 1.35, 0.025});

  ASSERT_EQ(limits.size(), 5U);
  EXPECT_DOUBLE_EQ(limits[0], 4.46625);
  EXPECT_DOUBLE_EQ(limits[1], 4.4325);
  EXPECT_DOUBLE_EQ(limits[4], 4.33125);
}

TEST(MpcVelocityLimit, SettlesAtFinalLimitWithoutUndershoot)
{
  const auto limits = multi_purpose_mpc_ros::mpc_velocity_limit::build_reachable_limits(
    multi_purpose_mpc_ros::mpc_velocity_limit::ReachableLimitRequest{
      20, 1.0, 0.0, 2.0, 0.1});

  ASSERT_EQ(limits.size(), 20U);
  EXPECT_DOUBLE_EQ(limits[0], 0.8);
  EXPECT_DOUBLE_EQ(limits[4], 0.0);
  EXPECT_DOUBLE_EQ(limits.back(), 0.0);
}

TEST(MpcVelocityLimit, KeepsHigherTargetImmediatelyAvailable)
{
  const auto limits = multi_purpose_mpc_ros::mpc_velocity_limit::build_reachable_limits(
    multi_purpose_mpc_ros::mpc_velocity_limit::ReachableLimitRequest{
      3, 2.0, 3.0, 1.35, 0.025});

  ASSERT_EQ(limits.size(), 3U);
  EXPECT_DOUBLE_EQ(limits[0], 3.0);
  EXPECT_DOUBLE_EQ(limits[1], 3.0);
  EXPECT_DOUBLE_EQ(limits[2], 3.0);
}

TEST(MpcVelocityLimit, RejectsInvalidInputs)
{
  auto request = multi_purpose_mpc_ros::mpc_velocity_limit::ReachableLimitRequest{
    3, 2.0, 0.0, 1.35, 0.025};
  request.current_velocity_mps = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(
    multi_purpose_mpc_ros::mpc_velocity_limit::build_reachable_limits(request),
    std::invalid_argument);

  request.current_velocity_mps = 2.0;
  request.time_step_sec = 0.0;
  EXPECT_THROW(
    multi_purpose_mpc_ros::mpc_velocity_limit::build_reachable_limits(request),
    std::invalid_argument);
}
