#include "multi_purpose_mpc_ros/mpcc_steering_state_contract.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace steering =
  multi_purpose_mpc_ros::mpcc_steering_state_contract;

namespace
{

steering::Request valid_request()
{
  return steering::Request{
    0.20, 0.24, 0.01, 0.13, 0.50, 0.61, 0.70, 0.02, 0.05};
}

TEST(
  MpccSteeringStateContract,
  DoesNotApplyNewCommittedCommandBeforeItsPublication)
{
  auto request = valid_request();
  request.observation_age_sec = 0.02;
  request.committed_command_age_sec = 0.004;
  request.committed_steering_rad = 0.40;

  const auto result = steering::resolve(request);

  ASSERT_TRUE(result.state.has_value());
  // Only the 4 ms since command publication belongs to the latest command;
  // applying it over the full 20 ms observation age is non-causal.
  EXPECT_NEAR(
    result.state->committed_command_projection_duration_sec, 0.004, 1e-12);
  EXPECT_NEAR(result.state->current_time_steering_rad, 0.2028, 1e-12);
  EXPECT_NEAR(result.state->prediction_origin_steering_rad, 0.2938, 1e-12);
}

TEST(MpccSteeringStateContract, ReachesPhysicalEquivalentCommandAtDelayOrigin)
{
  const auto result = steering::resolve(valid_request());

  ASSERT_EQ(result.reason, steering::Reason::Available);
  ASSERT_TRUE(result.state.has_value());
  EXPECT_DOUBLE_EQ(result.state->measured_steering_rad, 0.20);
  EXPECT_DOUBLE_EQ(result.state->committed_steering_rad, 0.24);
  EXPECT_DOUBLE_EQ(result.state->committed_command_age_sec, 0.02);
  EXPECT_DOUBLE_EQ(result.state->committed_command_control_age_sec, 0.05);
  EXPECT_NEAR(result.state->current_time_steering_rad, 0.207, 1e-12);
  EXPECT_DOUBLE_EQ(result.state->prediction_delay_sec, 0.13);
  EXPECT_DOUBLE_EQ(result.state->projection_duration_sec, 0.14);
  EXPECT_NEAR(result.state->maximum_reachable_step_rad, 0.098, 1e-12);
  EXPECT_NEAR(result.state->prediction_origin_steering_rad, 0.24, 1e-12);
  EXPECT_TRUE(result.state->committed_command_reached);
}

TEST(MpccSteeringStateContract, BoundsLargePhysicalEquivalentCommand)
{
  auto request = valid_request();
  request.committed_steering_rad = 0.40;

  const auto result = steering::resolve(request);

  ASSERT_TRUE(result.state.has_value());
  EXPECT_NEAR(result.state->prediction_origin_steering_rad, 0.298, 1e-12);
  EXPECT_NEAR(result.state->current_time_steering_rad, 0.207, 1e-12);
  EXPECT_FALSE(result.state->committed_command_reached);
}

TEST(MpccSteeringStateContract, BoundsOppositePhysicalEquivalentCommand)
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

TEST(
  MpccSteeringStateContract,
  NormalizesTheFloat32SerializationOfTheConfiguredModelLimit)
{
  auto request = valid_request();
  request.maximum_abs_steering_rad = 21.0 * std::acos(-1.0) / 180.0;
  const double serialized_limit = static_cast<double>(
    static_cast<float>(request.maximum_abs_steering_rad));
  ASSERT_GT(serialized_limit, request.maximum_abs_steering_rad);
  request.measured_steering_rad = -serialized_limit;
  request.committed_steering_rad = -serialized_limit;

  const auto result = steering::resolve(request);

  ASSERT_EQ(result.reason, steering::Reason::Available);
  ASSERT_TRUE(result.state.has_value());
  EXPECT_DOUBLE_EQ(
    result.state->measured_steering_rad,
    -request.maximum_abs_steering_rad);
  EXPECT_DOUBLE_EQ(
    result.state->committed_steering_rad,
    -request.maximum_abs_steering_rad);
  EXPECT_TRUE(result.state->measured_steering_serialization_projected);
  EXPECT_TRUE(result.state->committed_steering_serialization_projected);
}

TEST(
  MpccSteeringStateContract,
  RejectsSteeringBeyondTheFloat32SerializationEnvelope)
{
  auto request = valid_request();
  request.maximum_abs_steering_rad = 21.0 * std::acos(-1.0) / 180.0;
  const float serialized_limit =
    static_cast<float>(request.maximum_abs_steering_rad);
  request.measured_steering_rad = static_cast<double>(
    std::nextafter(serialized_limit, std::numeric_limits<float>::infinity()));

  const auto result = steering::resolve(request);

  EXPECT_EQ(result.reason, steering::Reason::InvalidMeasurement);
  EXPECT_FALSE(result.state.has_value());
}

}  // namespace
