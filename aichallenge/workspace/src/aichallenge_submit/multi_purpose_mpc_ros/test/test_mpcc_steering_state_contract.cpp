#include "multi_purpose_mpc_ros/mpcc_steering_state_contract.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace steering =
  multi_purpose_mpc_ros::mpcc_steering_state_contract;

namespace
{

steering::Request valid_request()
{
  return steering::Request{
    0.20, 0.24, 0.01, 0.13, 0.50, 0.61, 0.70};
}

TEST(MpccSteeringStateContract, ReachesCommittedInputWithinLatencyPrefix)
{
  const auto result = steering::resolve(valid_request());

  ASSERT_EQ(result.reason, steering::Reason::Available);
  ASSERT_TRUE(result.state.has_value());
  EXPECT_DOUBLE_EQ(result.state->measured_steering_rad, 0.20);
  EXPECT_DOUBLE_EQ(result.state->committed_steering_rad, 0.24);
  EXPECT_DOUBLE_EQ(result.state->projection_duration_sec, 0.14);
  EXPECT_NEAR(result.state->maximum_reachable_step_rad, 0.098, 1e-12);
  EXPECT_NEAR(result.state->prediction_origin_steering_rad, 0.24, 1e-12);
  EXPECT_TRUE(result.state->committed_command_reached);
}

TEST(MpccSteeringStateContract, LimitsCommittedInputByPhysicalSteeringRate)
{
  auto request = valid_request();
  request.committed_steering_rad = 0.40;

  const auto result = steering::resolve(request);

  ASSERT_TRUE(result.state.has_value());
  EXPECT_NEAR(result.state->prediction_origin_steering_rad, 0.298, 1e-12);
  EXPECT_FALSE(result.state->committed_command_reached);
}

TEST(MpccSteeringStateContract, LimitsOppositeDirectionCommittedInput)
{
  auto request = valid_request();
  request.committed_steering_rad = -0.20;

  const auto result = steering::resolve(request);

  ASSERT_TRUE(result.state.has_value());
  EXPECT_NEAR(result.state->prediction_origin_steering_rad, 0.102, 1e-12);
  EXPECT_FALSE(result.state->committed_command_reached);
}

TEST(MpccSteeringStateContract, RejectsMissingCommittedInput)
{
  auto request = valid_request();
  request.committed_steering_rad.reset();

  const auto result = steering::resolve(request);

  EXPECT_EQ(result.reason, steering::Reason::CommittedInputUnavailable);
  EXPECT_FALSE(result.state.has_value());
}

TEST(MpccSteeringStateContract, RejectsStaleObservation)
{
  auto request = valid_request();
  request.observation_age_sec = 0.51;

  const auto result = steering::resolve(request);

  EXPECT_EQ(result.reason, steering::Reason::StaleObservation);
  EXPECT_FALSE(result.state.has_value());
}

TEST(MpccSteeringStateContract, RejectsNonfiniteMeasurement)
{
  auto request = valid_request();
  request.measured_steering_rad =
    std::numeric_limits<double>::quiet_NaN();

  const auto result = steering::resolve(request);

  EXPECT_EQ(result.reason, steering::Reason::InvalidMeasurement);
  EXPECT_FALSE(result.state.has_value());
}

TEST(MpccSteeringStateContract, RejectsNonfiniteCommittedInput)
{
  auto request = valid_request();
  request.committed_steering_rad = std::numeric_limits<double>::infinity();

  const auto result = steering::resolve(request);

  EXPECT_EQ(result.reason, steering::Reason::InvalidMeasurement);
  EXPECT_FALSE(result.state.has_value());
}

}  // namespace
