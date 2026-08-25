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
    0.20, 0.10, 0.01, 0.13, 0.50, 0.61, 0.70};
}

TEST(MpccSteeringStateContract, ProjectsMeasuredStateToControlOrigin)
{
  const auto result = steering::resolve(valid_request());

  ASSERT_EQ(result.reason, steering::Reason::Available);
  ASSERT_TRUE(result.state.has_value());
  EXPECT_DOUBLE_EQ(result.state->measured_steering_rad, 0.20);
  EXPECT_DOUBLE_EQ(result.state->projection_duration_sec, 0.14);
  EXPECT_NEAR(result.state->prediction_origin_steering_rad, 0.214, 1e-12);
  EXPECT_FALSE(result.state->measured_rate_outside_model);
}

TEST(MpccSteeringStateContract, DesiredCommandCannotEnterTheRequest)
{
  auto request = valid_request();
  request.measured_steering_rad = -0.12;
  request.measured_steering_rate_radps = 0.0;

  const auto result = steering::resolve(request);

  ASSERT_TRUE(result.state.has_value());
  EXPECT_DOUBLE_EQ(result.state->prediction_origin_steering_rad, -0.12);
}

TEST(MpccSteeringStateContract, BoundsNoisyMeasuredRateByPhysicalModel)
{
  auto request = valid_request();
  request.measured_steering_rate_radps = 2.0;

  const auto result = steering::resolve(request);

  ASSERT_TRUE(result.state.has_value());
  EXPECT_DOUBLE_EQ(result.state->bounded_steering_rate_radps, 0.70);
  EXPECT_TRUE(result.state->measured_rate_outside_model);
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

}  // namespace
