#include "multi_purpose_mpc_ros/mpcc_latest_state_feedback.hpp"
#include "multi_purpose_mpc_ros/mpcc_wire_command.hpp"

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

namespace wire = multi_purpose_mpc_ros::mpcc_wire_command;

TEST(MpccWireReachability, BothSerializedSlewBoundariesUseAnInwardRepresentableValue)
{
  // Frozen r59 D2 pre948: the real-valued boundary encodes outside the exact
  // wire-difference constraint. Mirror it to cover both steering directions.
  const double gain = 1.435;
  const double step = 0.027671960261689516;
  for (const double sign : {-1.0, 1.0}) {
    const double previous = sign * 0.4613766372203827;
    const double desired = previous / gain - sign * step;
    ASSERT_GT(std::abs(wire::steering(desired, gain) - previous) / gain, step);
    const auto selected = wire::reachable_steering(desired, previous, gain, .6, step);
    ASSERT_TRUE(selected);
    EXPECT_LE(std::abs(wire::steering(*selected, gain) - previous) / gain, step);
    const float adjacent = std::nextafter(static_cast<float>(*selected),
      sign > 0 ? -std::numeric_limits<float>::infinity() : std::numeric_limits<float>::infinity());
    EXPECT_GT(std::abs(wire::steering(adjacent, gain) - previous) / gain, step);
  }
}

TEST(MpccWireReachability, PreservesAlreadyReachableCommandExactly)
{
  const double desired = 0.1234567890123;
  const auto selected = wire::reachable_steering(desired, wire::steering(.12, 1.435), 1.435, .6, .02);
  ASSERT_TRUE(selected);
  EXPECT_DOUBLE_EQ(*selected, desired);
  EXPECT_DOUBLE_EQ(wire::steering(*selected, 1.435), wire::steering(desired, 1.435));
}

TEST(MpccWireReachability, AbsoluteLimitsHoldAfterBothFloatRoundings)
{
  for (const double sign : {-1.0, 1.0}) {
    const auto selected = wire::reachable_steering(sign * .3, 0, 1.435, .3, 1);
    ASSERT_TRUE(selected);
    EXPECT_LE(std::abs(*selected), .3);
    EXPECT_LE(std::abs(wire::steering(*selected, 1.435) / 1.435), .3);
    const float beyond = std::nextafter(static_cast<float>(*selected),
      sign > 0 ? std::numeric_limits<float>::infinity() : -std::numeric_limits<float>::infinity());
    EXPECT_TRUE(std::abs(static_cast<double>(beyond)) > .3 ||
      std::abs(wire::steering(beyond, 1.435) / 1.435) > .3);
  }
}

TEST(MpccWireReachability, EmptyRepresentableSetCannotBecomeAHeldOrClampedCommand)
{
  EXPECT_FALSE(wire::reachable_steering(0, 1, 1, .5, .01));
  // Calibration can skip a final float value; zero slew cannot invent it.
  const double previous = 0.9999998211860657; // third float below1, skipped at gain1.5
  const float nearest = static_cast<float>(previous / 1.5);
  ASSERT_GT(wire::steering(nearest, 1.5), previous);
  ASSERT_LT(wire::steering(std::nextafter(nearest, 0.0F), 1.5), previous);
  EXPECT_FALSE(wire::reachable_steering(previous / 1.5, previous, 1.5, 2, 0));
  const auto zero = wire::reachable_steering(-0.0, 0, 1.435, .6, 0);
  ASSERT_TRUE(zero);
  EXPECT_DOUBLE_EQ(wire::steering(*zero, 1.435), 0);
}

TEST(MpccWireReachability, InvalidInputsHaveNoCommand)
{
  EXPECT_FALSE(wire::reachable_steering(NAN, 0, 1, .6, .1));
  EXPECT_FALSE(wire::reachable_steering(0, .1234567890123, 1, .6, .1));
  EXPECT_FALSE(wire::reachable_steering(0, 0, 0, .6, .1));
  EXPECT_FALSE(wire::reachable_steering(0, 0, 1, .6, -1));
  EXPECT_FALSE(wire::reachable_steering(0, 0, INFINITY, .6, .1));
}
