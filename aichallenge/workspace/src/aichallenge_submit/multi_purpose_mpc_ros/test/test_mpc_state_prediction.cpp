#include <gtest/gtest.h>

#include <multi_purpose_mpc_ros/mpc_state_prediction.hpp>
#include <multi_purpose_mpc_ros/mpc_longitudinal_prediction.hpp>

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

TEST(MpcLongitudinalPrediction, SavedBrakeTransitionUsesCommittedInputAndResponse) {
  const double residual = .543785436495885 - 1.3295944422392196;
  const auto trajectory = state_prediction::predict_piecewise_yaw_response_trajectory(
    {0.0, 0.0, 0.0}, 1.843041217111899, 0.0, 0.0, 0.0, 1.087, .75, .13,
    {{.015000003, 1.329595923423767 + residual},
      {.114999997, -2.9595959186553955 + residual}});
  ASSERT_FALSE(trajectory.empty());
  EXPECT_NEAR(trajectory.back().elapsed_sec, .13, 1e-12);
  EXPECT_NEAR(trajectory.back().prediction.longitudinal_velocity_mps, 1.4204764674388271, 1e-10);
  for (std::size_t i = 1; i < trajectory.size(); ++i) {
    EXPECT_GT(trajectory[i].elapsed_sec, trajectory[i - 1].elapsed_sec);
  }
}

TEST(MpcLongitudinalPrediction, BothFiltersSeeTheSameAlreadyAppliedBrake) {
  state_prediction::LongitudinalResponseObserver observer(.9, .5);
  ASSERT_TRUE(observer.record_published_command(-.13, 0.0, 1.0));
  EXPECT_FALSE(observer.observe(0.0, 2.0));
  ASSERT_TRUE(observer.observe(.1, 2.1));
  ASSERT_TRUE(observer.record_published_command(.1, .23, -3.0));
  const auto observation = observer.observe(.25, 2.1 + .13 - .02 * 3.0);
  ASSERT_TRUE(observation);
  EXPECT_GT(observation->filtered_measured_acceleration_mps2, 0.0);
  EXPECT_NEAR(observation->response_residual_mps2(), 0.0, 1e-12);
  const auto intervals = observer.prediction_intervals(.25, .13);
  ASSERT_TRUE(intervals);
  ASSERT_EQ(intervals->size(), 1U);
  EXPECT_NEAR(intervals->front().acceleration_mps2, -3.0, 1e-12);
}

TEST(MpcLongitudinalPrediction, PreservesSteadyDragWhileUsingFutureCommandChanges) {
  state_prediction::LongitudinalResponseObserver observer(.9, .5);
  ASSERT_TRUE(observer.record_published_command(-.13, 0.0, 1.0));
  EXPECT_FALSE(observer.observe(0.0, 7.0));
  for (int i = 1; i <= 300; ++i) {
    ASSERT_TRUE(observer.observe(i * .02, 7.0));
  }
  auto intervals = observer.prediction_intervals(6.0, .13);
  ASSERT_TRUE(intervals);
  EXPECT_NEAR(intervals->front().acceleration_mps2, 0.0, 1e-12);
  ASSERT_TRUE(observer.record_published_command(6.0, 6.13, -3.0));
  intervals = observer.prediction_intervals(6.05, .13);
  ASSERT_TRUE(intervals);
  ASSERT_EQ(intervals->size(), 2U);
  EXPECT_NEAR((*intervals)[0].duration_sec, .08, 1e-12);
  EXPECT_NEAR((*intervals)[0].acceleration_mps2, 0.0, 1e-12);
  EXPECT_NEAR((*intervals)[1].acceleration_mps2, -4.0, 1e-12);
}

TEST(MpcLongitudinalPrediction, DuplicateObservationDoesNotInventFilterTime) {
  state_prediction::LongitudinalResponseObserver observer(.9, .5);
  EXPECT_FALSE(observer.observe(1.0, 2.0));
  const auto first = observer.observe(1.1, 2.1);
  ASSERT_TRUE(first);
  const auto duplicate = observer.observe(1.1, 2.105);
  ASSERT_TRUE(duplicate);
  EXPECT_DOUBLE_EQ(duplicate->filtered_measured_acceleration_mps2,
    first->filtered_measured_acceleration_mps2);
  const auto next = observer.observe(1.2, 2.205);
  ASSERT_TRUE(next);
  EXPECT_NEAR(next->filtered_measured_acceleration_mps2, .19, 1e-12);
}

TEST(MpcLongitudinalPrediction, DuplicatePublishedStampKeepsLatestSerializedCommand) {
  state_prediction::LongitudinalResponseObserver observer(.9, .5);
  EXPECT_FALSE(observer.observe(0.0, 2.0));
  ASSERT_TRUE(observer.observe(.1, 2.0));
  ASSERT_TRUE(observer.record_published_command(.1, .23, 1.0));
  ASSERT_TRUE(observer.record_published_command(.1, .23, -3.0));
  EXPECT_EQ(observer.retained_command_count(), 1U);
  EXPECT_FALSE(observer.record_published_command(.1, .24, 1.0));
  const auto intervals = observer.prediction_intervals(.2, .13);
  ASSERT_TRUE(intervals);
  ASSERT_EQ(intervals->size(), 2U);
  EXPECT_DOUBLE_EQ(intervals->back().acceleration_mps2, -3.0);
}

TEST(MpcLongitudinalPrediction, MissingStaleAndFutureObservationsHaveNoPrediction) {
  state_prediction::LongitudinalResponseObserver observer(.9, .5);
  EXPECT_FALSE(observer.prediction_intervals(1.0, .13));
  EXPECT_FALSE(observer.observe(1.0, 2.0));
  EXPECT_FALSE(observer.prediction_intervals(1.0, .13));
  ASSERT_TRUE(observer.observe(1.1, 2.0));
  EXPECT_FALSE(observer.prediction_intervals(1.0, .13));
  EXPECT_FALSE(observer.prediction_intervals(1.61, .13));
  ASSERT_TRUE(observer.record_published_command(1.2, 1.33, -3.0));
  EXPECT_FALSE(observer.prediction_intervals(1.15, .13));
}

TEST(MpcLongitudinalPrediction, ClockResetAndObservationGapRequireNewDerivative) {
  state_prediction::LongitudinalResponseObserver observer(.9, .5);
  EXPECT_FALSE(observer.observe(1.0, 2.0));
  ASSERT_TRUE(observer.observe(1.1, 2.0));
  ASSERT_TRUE(observer.record_published_command(1.1, 1.23, -3.0));
  EXPECT_FALSE(observer.observe(.1, 0.0));
  EXPECT_EQ(observer.retained_command_count(), 0U);
  ASSERT_TRUE(observer.observe(.2, 0.0));
  EXPECT_FALSE(observer.observe(.8, 0.0));
  EXPECT_FALSE(observer.prediction_intervals(.8, .13));
  ASSERT_TRUE(observer.observe(.9, 0.0));
  ASSERT_TRUE(observer.record_published_command(.9, 1.03, 1.0));
  ASSERT_TRUE(observer.record_published_command(.3, .43, -3.0));
  EXPECT_FALSE(observer.prediction_intervals(.3, .13));
}

TEST(MpcLongitudinalPrediction, HistoryRetainsOnlyValidObservationWindowAndPendingInputs) {
  state_prediction::LongitudinalResponseObserver observer(.9, .5);
  for (int i = 0; i < 1000; ++i) {
    ASSERT_TRUE(observer.record_published_command(i * .02, i * .02 + .13, 1.0));
  }
  EXPECT_LE(observer.retained_command_count(), 33U);
  EXPECT_FALSE(observer.record_published_command(20.0, 19.9, 1.0));
  EXPECT_FALSE(observer.record_published_command(20.0, 20.13,
    std::numeric_limits<double>::quiet_NaN()));
}

TEST(MpcLongitudinalPrediction, PiecewiseBrakingReachesRestWithoutReverseMotion) {
  const auto trajectory = state_prediction::predict_piecewise_yaw_response_trajectory(
    {0.0, 0.0, 0.0}, .1, 0.0, 0.0, 0.0, 1.087, .75, .13,
    {{.025, -3.0}, {.105, -3.0}});
  EXPECT_DOUBLE_EQ(trajectory.back().prediction.longitudinal_velocity_mps, 0.0);
  for (std::size_t i = 1; i < trajectory.size(); ++i) {
    EXPECT_GE(trajectory[i].prediction.state.x, trajectory[i - 1].prediction.state.x);
  }
  EXPECT_THROW(state_prediction::predict_piecewise_yaw_response_trajectory(
    {0.0, 0.0, 0.0}, .1, 0.0, 0.0, 0.0, 1.087, .75, .13, {{0.0, -3.0}}),
    std::invalid_argument);
}

TEST(MpcLongitudinalPrediction, ReceivedObservationOwnsClockWhenRosCacheIsOneTickBehind) {
  constexpr double ros_time = 35.444999207;
  constexpr double observation_time = 35.449999207;
  const auto epoch = state_prediction::resolve_control_observation_epoch(
    ros_time, 35.419999208, observation_time, .5);
  ASSERT_TRUE(epoch.valid);
  EXPECT_FALSE(epoch.clock_regressed);
  EXPECT_TRUE(epoch.use_received_observation_stamp);
  state_prediction::LongitudinalResponseObserver observer(.9, .5);
  EXPECT_FALSE(observer.observe(35.439999207, 7.623775546598));
  ASSERT_TRUE(observer.observe(observation_time, 7.623775546598));
  // The old producer's time remains rejected. Correct the producer, not this guard.
  EXPECT_FALSE(observer.prediction_intervals(ros_time, .13));
  const double control_time = epoch.use_received_observation_stamp ? observation_time : ros_time;
  EXPECT_TRUE(observer.prediction_intervals(control_time, .13));
}

TEST(MpcLongitudinalPrediction, CurrentRosClockOwnsDecisionAfterOlderReceivedState) {
  const auto epoch = state_prediction::resolve_control_observation_epoch(2.1, 2.075, 2.08, .5);
  EXPECT_TRUE(epoch.valid);
  EXPECT_FALSE(epoch.use_received_observation_stamp);
  EXPECT_FALSE(epoch.clock_regressed);
}

TEST(MpcLongitudinalPrediction, ClockResetCannotRetainPreviousEpochObservation) {
  const auto epoch = state_prediction::resolve_control_observation_epoch(.1, 35.0, 35.0, .5);
  EXPECT_FALSE(epoch.valid);
  EXPECT_TRUE(epoch.clock_regressed);
  EXPECT_FALSE(epoch.use_received_observation_stamp);
}

TEST(MpcLongitudinalPrediction, ObservationClockSkewOutsideExistingAgeContractStaysInvalid) {
  EXPECT_FALSE(state_prediction::resolve_control_observation_epoch(2.0, 1.9, 2.6, .5).valid);
  EXPECT_FALSE(state_prediction::resolve_control_observation_epoch(2.0, 1.9, 1.4, .5).valid);
  EXPECT_FALSE(state_prediction::resolve_control_observation_epoch(
    2.0, std::nullopt, std::numeric_limits<double>::quiet_NaN(), .5).valid);
}
