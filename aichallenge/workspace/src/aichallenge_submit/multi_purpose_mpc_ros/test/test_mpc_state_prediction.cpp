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
