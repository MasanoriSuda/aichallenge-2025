#include <gtest/gtest.h>

#include <multi_purpose_mpc_ros/mpc_state_prediction.hpp>

#include <cmath>
#include <limits>
#include <stdexcept>

namespace state_prediction = multi_purpose_mpc_ros::mpc_state_prediction;

TEST(MpcStatePrediction, KeepsStateWhenDelayIsZero) {
  const state_prediction::State2D input{1.0, 2.0, 0.3};
  const auto predicted =
    state_prediction::predict_constant_turn_rate(input, 10.0, 0.5, 0.0);

  EXPECT_DOUBLE_EQ(predicted.x, input.x);
  EXPECT_DOUBLE_EQ(predicted.y, input.y);
  EXPECT_DOUBLE_EQ(predicted.yaw, input.yaw);
}

TEST(MpcStatePrediction, PredictsStraightMotionWithoutMutatingInput) {
  const state_prediction::State2D input{2.0, 3.0, 0.5 * std::acos(-1.0)};
  const auto predicted =
    state_prediction::predict_constant_turn_rate(input, 8.0, 0.0, 0.125);

  EXPECT_NEAR(predicted.x, 2.0, 1.0e-12);
  EXPECT_NEAR(predicted.y, 4.0, 1.0e-12);
  EXPECT_NEAR(predicted.yaw, input.yaw, 1.0e-12);
  EXPECT_DOUBLE_EQ(input.x, 2.0);
  EXPECT_DOUBLE_EQ(input.y, 3.0);
}

TEST(MpcStatePrediction, PredictsConstantTurnRateArc) {
  const state_prediction::State2D input{0.0, 0.0, 0.0};
  const auto predicted =
    state_prediction::predict_constant_turn_rate(input, 10.0, 1.0, 0.1);

  EXPECT_NEAR(predicted.x, 10.0 * std::sin(0.1), 1.0e-12);
  EXPECT_NEAR(predicted.y, 10.0 * (1.0 - std::cos(0.1)), 1.0e-12);
  EXPECT_NEAR(predicted.yaw, 0.1, 1.0e-12);
}

TEST(MpcStatePrediction, SupportsReverseMotion) {
  const state_prediction::State2D input{1.0, 0.0, 0.0};
  const auto predicted =
    state_prediction::predict_constant_turn_rate(input, -2.0, 0.0, 0.5);

  EXPECT_NEAR(predicted.x, 0.0, 1.0e-12);
  EXPECT_NEAR(predicted.y, 0.0, 1.0e-12);
}

TEST(MpcStatePrediction, RejectsInvalidInput) {
  const state_prediction::State2D input{0.0, 0.0, 0.0};
  EXPECT_THROW(
    state_prediction::predict_constant_turn_rate(input, 1.0, 0.0, -0.1),
    std::invalid_argument);
  EXPECT_THROW(
    state_prediction::predict_constant_turn_rate(
      input, 1.0, std::numeric_limits<double>::quiet_NaN(), 0.1),
    std::invalid_argument);
}

TEST(MpcStatePrediction, ProjectsLaggedYawResponseAndPose) {
  const state_prediction::State2D input{0.0, 0.0, 0.0};
  const auto predicted = state_prediction::predict_yaw_response(
    input, 8.0, 0.0, 0.0, 0.2, 1.087, 0.75, 0.13, 0.13);

  EXPECT_GT(predicted.response_steering_rad, 0.0);
  EXPECT_LT(predicted.response_steering_rad, 0.2);
  EXPECT_GT(predicted.yaw_rate_radps, 0.0);
  EXPECT_GT(predicted.state.y, 0.0);
  EXPECT_GT(predicted.state.yaw, 0.0);
  EXPECT_GT(predicted.state.x, 0.9);
}

TEST(MpcStatePrediction, ProjectsSpeedAndPoseFromOneAccelerationTimedState) {
  const state_prediction::State2D input{0.0, 0.0, 0.0};
  const auto predicted = state_prediction::predict_accelerating_yaw_response(
    input, 8.0, -2.0, 0.0, 0.0, 0.2, 1.087, 0.75, 0.13, 0.13);

  EXPECT_NEAR(predicted.longitudinal_velocity_mps, 7.74, 1.0e-12);
  EXPECT_GT(predicted.response_steering_rad, 0.0);
  EXPECT_LT(predicted.response_steering_rad, 0.2);
  EXPECT_GT(predicted.yaw_rate_radps, 0.0);
  EXPECT_GT(predicted.state.y, 0.0);
  EXPECT_GT(predicted.state.yaw, 0.0);
  // Constant-acceleration travel is 8*0.13 - 0.5*2*0.13^2 = 1.0231 m.
  // Curvature shortens x slightly but the prediction must not retain the
  // old constant-speed 1.04 m execution distance.
  EXPECT_GT(predicted.state.x, 1.0);
  EXPECT_LT(predicted.state.x, 1.04);
}

TEST(MpcStatePrediction, ExposesTheExactLatencyTrajectoryConsumedByTheStateOrigin) {
  const state_prediction::State2D input{1.0, -2.0, 0.3};
  const auto trajectory =
    state_prediction::predict_accelerating_yaw_response_trajectory(
    input, 8.0, -2.0, 0.0, 0.0, 0.2, 1.087, 0.75, 0.13, 0.13);
  const auto final = state_prediction::predict_accelerating_yaw_response(
    input, 8.0, -2.0, 0.0, 0.0, 0.2, 1.087, 0.75, 0.13, 0.13);

  ASSERT_GT(trajectory.size(), 2U);
  EXPECT_DOUBLE_EQ(trajectory.front().elapsed_sec, 0.0);
  EXPECT_DOUBLE_EQ(trajectory.front().prediction.state.x, input.x);
  EXPECT_DOUBLE_EQ(trajectory.front().prediction.state.y, input.y);
  EXPECT_DOUBLE_EQ(trajectory.front().prediction.state.yaw, input.yaw);
  for (std::size_t index = 1U; index < trajectory.size(); ++index) {
    EXPECT_GT(trajectory[index].elapsed_sec, trajectory[index - 1U].elapsed_sec);
  }
  EXPECT_NEAR(trajectory.back().elapsed_sec, 0.13, 1.0e-12);
  EXPECT_NEAR(trajectory.back().prediction.state.x, final.state.x, 1.0e-12);
  EXPECT_NEAR(trajectory.back().prediction.state.y, final.state.y, 1.0e-12);
  EXPECT_NEAR(trajectory.back().prediction.state.yaw, final.state.yaw, 1.0e-12);
  EXPECT_NEAR(
    trajectory.back().prediction.longitudinal_velocity_mps,
    final.longitudinal_velocity_mps, 1.0e-12);
  EXPECT_NEAR(
    trajectory.back().prediction.response_steering_rad,
    final.response_steering_rad, 1.0e-12);
}

TEST(MpcStatePrediction, AcceleratingYawResponseKeepsTimedStateAtZeroDelay) {
  const state_prediction::State2D input{1.0, 2.0, 0.3};
  const auto predicted = state_prediction::predict_accelerating_yaw_response(
    input, 8.0, -2.0, 0.1, 0.1, 0.2, 1.087, 0.75, 0.13, 0.0);

  EXPECT_DOUBLE_EQ(predicted.state.x, input.x);
  EXPECT_DOUBLE_EQ(predicted.state.y, input.y);
  EXPECT_DOUBLE_EQ(predicted.state.yaw, input.yaw);
  EXPECT_DOUBLE_EQ(predicted.longitudinal_velocity_mps, 8.0);
  EXPECT_DOUBLE_EQ(predicted.response_steering_rad, 0.1);
}

TEST(MpcStatePrediction, YawResponseKeepsStateAtZeroDelay) {
  const state_prediction::State2D input{1.0, 2.0, 0.3};
  const auto predicted = state_prediction::predict_yaw_response(
    input, 8.0, 0.1, 0.1, 0.2, 1.087, 0.75, 0.13, 0.0);
  EXPECT_DOUBLE_EQ(predicted.state.x, input.x);
  EXPECT_DOUBLE_EQ(predicted.state.y, input.y);
  EXPECT_DOUBLE_EQ(predicted.state.yaw, input.yaw);
  EXPECT_DOUBLE_EQ(predicted.response_steering_rad, 0.1);
}

TEST(MpcStatePrediction, RejectsInvalidYawResponseContract) {
  const state_prediction::State2D input{0.0, 0.0, 0.0};
  EXPECT_THROW(
    state_prediction::predict_yaw_response(
      input, 8.0, 0.0, 0.0, 0.2, 1.087, 0.0, 0.13, 0.13),
    std::invalid_argument);
}

TEST(MpcStatePrediction, InfersResponseSteeringFromMeasuredYawRate) {
  const double response_steering = 0.18;
  const double speed_mps = 6.0;
  const double wheelbase_m = 2.0;
  const double yaw_gain = 0.75;
  const double yaw_rate =
    yaw_gain * speed_mps * std::tan(response_steering) / wheelbase_m;
  const auto inferred = state_prediction::infer_response_steering(
    speed_mps, yaw_rate, 0.25, wheelbase_m, yaw_gain, 0.5, 0.6);
  ASSERT_TRUE(inferred.has_value());
  EXPECT_NEAR(inferred->steering_rad, response_steering, 1e-12);
  EXPECT_NEAR(inferred->unconstrained_steering_rad, response_steering, 1e-12);
  EXPECT_FALSE(inferred->projected_to_model_envelope);
}

TEST(MpcStatePrediction, UsesPhysicalSteeringWhenYawInversionIsIllConditioned) {
  const auto inferred = state_prediction::infer_response_steering(
    0.1, 3.0, -0.21, 2.0, 0.75, 0.5, 0.6);
  ASSERT_TRUE(inferred.has_value());
  EXPECT_DOUBLE_EQ(inferred->steering_rad, -0.21);
  EXPECT_DOUBLE_EQ(inferred->unconstrained_steering_rad, -0.21);
  EXPECT_FALSE(inferred->projected_to_model_envelope);
}

TEST(MpcStatePrediction, ProjectsFiniteResponseOutsideReducedModelEnvelope) {
  const auto inferred = state_prediction::infer_response_steering(
    5.0, 10.0, 0.0, 2.0, 0.75, 0.5, 0.6);
  ASSERT_TRUE(inferred.has_value());
  EXPECT_DOUBLE_EQ(inferred->steering_rad, 0.6);
  EXPECT_GT(inferred->unconstrained_steering_rad, 0.6);
  EXPECT_TRUE(inferred->projected_to_model_envelope);
}
