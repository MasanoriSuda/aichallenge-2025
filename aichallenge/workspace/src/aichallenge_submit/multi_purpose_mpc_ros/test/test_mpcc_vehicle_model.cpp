#include "mpcc_vehicle_model_fixture.hpp"
#include "multi_purpose_mpc_ros/mpcc_vehicle_prediction.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace vehicle = multi_purpose_mpc_ros::mpcc_vehicle_model;
using multi_purpose_mpc_ros::test::vehicle_model;

TEST(MpccVehicleModel, WireAccelerationIsNotNetBodyAcceleration)
{
  const auto p = vehicle_model();
  const vehicle::State initial{0, 0, 0, 2, 0, 0, 0, 0};
  const auto moved = vehicle::advance(initial, {1, 0}, p, .2);
  ASSERT_TRUE(moved);
  // Independent straight-line ODE: du/dt = sum(traction)*(wire-rolling)-drag*u.
  double traction = 0;
  for (const auto & wheel : p.wheels) traction += wheel.traction_fraction;
  const double equilibrium = traction * (1 - p.rolling_mps2) / p.drag_per_sec;
  const double expected = equilibrium + (2 - equilibrium) * std::exp(-p.drag_per_sec * .2);
  EXPECT_NEAR(moved->state.forward_velocity_mps, expected, 1e-9);
  EXPECT_LT(moved->state.forward_velocity_mps, 2.1);
}

TEST(MpccVehicleModel, NominalStopAndRestartUseTheSameBodyState)
{
  const auto p = vehicle_model();
  vehicle::State state{0, 0, 0, 2, .1, .2, .05, .05};
  for (int step = 0; step < 400; ++step) {
    const auto next = vehicle::advance(state, {-3, 0}, p, .005);
    ASSERT_TRUE(next);
    state = next->state;
  }
  EXPECT_DOUBLE_EQ(state.forward_velocity_mps, 0);
  EXPECT_DOUBLE_EQ(state.lateral_velocity_mps, 0);
  EXPECT_DOUBLE_EQ(state.yaw_rate_radps, 0);
  const auto stationary = vehicle::advance(state, {0, 0}, p, .5);
  ASSERT_TRUE(stationary);
  EXPECT_DOUBLE_EQ(stationary->state.x_m, state.x_m);
  EXPECT_DOUBLE_EQ(stationary->state.y_m, state.y_m);
  const auto restart = vehicle::advance(state, {1, 0}, p, .1);
  ASSERT_TRUE(restart);
  EXPECT_GT(restart->state.forward_velocity_mps, 0);
}

TEST(MpccVehicleModel, ProfileRoundTripAndIdentityBindEveryPhysicalParameter)
{
  const auto p = vehicle_model();
  const auto decoded = vehicle::decode_parameters(vehicle::encode_parameters(p));
  ASSERT_TRUE(decoded);
  EXPECT_EQ(vehicle::fingerprint(p), vehicle::fingerprint(*decoded));
  auto changed = p;
  changed.wheels[3].cornering_per_sec += .1;
  EXPECT_NE(vehicle::fingerprint(p), vehicle::fingerprint(changed));
  changed = p;
  changed.com_forward_m += .01;
  EXPECT_NE(vehicle::fingerprint(p), vehicle::fingerprint(changed));
  EXPECT_EQ(vehicle::fingerprint({}), 0U);
  auto missing = vehicle::encode_parameters(p);
  missing.remove("nominal_settled_contact");
  EXPECT_FALSE(vehicle::decode_parameters(missing));
}

TEST(MpccVehiclePrediction, SeparatesChannelDelayFromMechanicalTireResponse)
{
  const auto p = vehicle_model();
  const vehicle::TimedState initial{1, {0, 0, 0, 2, 0, 0, 0, 0}};
  std::vector<vehicle::PublishedCommand> history{{0, 0, 0}, {1, -3, .2}};
  const auto prediction = vehicle::predict_published_history(initial, 1, 1.2, history, p, 0, .1);
  ASSERT_TRUE(prediction);
  const auto first = vehicle::advance(initial.state, {-3, 0}, p, .1);
  ASSERT_TRUE(first);
  auto delayed = first->state;
  delayed.desired_steering_rad = .2 / p.steering_wire_gain;
  const auto second = vehicle::advance(delayed, {-3, 0}, p, .1);
  ASSERT_TRUE(second);
  EXPECT_NEAR(prediction->control_origin.forward_velocity_mps, second->state.forward_velocity_mps, 1e-12);
  EXPECT_NEAR(prediction->control_origin.tire_steering_rad, second->state.tire_steering_rad, 1e-12);
  EXPECT_NEAR(prediction->control_origin.yaw_rate_radps, second->state.yaw_rate_radps, 1e-12);
  history.push_back({1.01, 1, 0});
  EXPECT_FALSE(vehicle::predict_published_history(initial, 1, 1.2, history, p, 0, .1));
  history = {{1, -3, .2}};
  EXPECT_FALSE(vehicle::predict_published_history(initial, 1, 1.2, history, p, 0, .1));
}
