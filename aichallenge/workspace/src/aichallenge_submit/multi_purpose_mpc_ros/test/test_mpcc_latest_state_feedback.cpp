#include "multi_purpose_mpc_ros/mpcc_latest_state_feedback.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace feedback =
  multi_purpose_mpc_ros::mpcc_latest_state_feedback;

TEST(MpccLatestStateFeedback, KeepsReachablePreparedCommand)
{
  const auto result = feedback::solve(feedback::Request{
    0.10, 0.12, 0.50, 1.0, 0.05, 1.0e-6});
  ASSERT_TRUE(result.available());
  EXPECT_FALSE(result.projected());
  EXPECT_DOUBLE_EQ(result.feedback_steering_rad, 0.12);
  EXPECT_DOUBLE_EQ(result.correction_rad, 0.0);
}

TEST(MpccLatestStateFeedback, ProjectsOntoExistingSlewEnvelope)
{
  const auto result = feedback::solve(feedback::Request{
    0.10, 0.30, 0.50, 1.0, 0.05, 0.0});
  ASSERT_TRUE(result.available());
  EXPECT_TRUE(result.projected());
  EXPECT_NEAR(result.lower_rad, 0.05, 1.0e-12);
  EXPECT_NEAR(result.upper_rad, 0.15, 1.0e-12);
  EXPECT_NEAR(result.feedback_steering_rad, 0.15, 1.0e-12);
  EXPECT_NEAR(result.correction_rad, -0.15, 1.0e-12);
}

TEST(MpccLatestStateFeedback, IntersectsSlewAndAbsoluteBounds)
{
  const auto result = feedback::solve(feedback::Request{
    0.48, 0.60, 0.50, 1.0, 0.05, 0.0});
  ASSERT_TRUE(result.available());
  EXPECT_TRUE(result.projected());
  EXPECT_NEAR(result.lower_rad, 0.43, 1.0e-12);
  EXPECT_NEAR(result.upper_rad, 0.50, 1.0e-12);
  EXPECT_NEAR(result.feedback_steering_rad, 0.50, 1.0e-12);
}

TEST(MpccLatestStateFeedback, RejectsInvalidInput)
{
  const auto result = feedback::solve(feedback::Request{
    0.0, std::numeric_limits<double>::quiet_NaN(), 0.50, 1.0, 0.05, 0.0});
  EXPECT_FALSE(result.available());
  EXPECT_EQ(result.reason, feedback::Reason::InvalidInput);
}
