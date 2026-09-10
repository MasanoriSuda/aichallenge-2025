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

TEST(MpccVehicleModel, StopMapSupportAccountsForWireBodyMismatch)
{
  const auto p = vehicle_model();
  const auto support = vehicle::nominal_stop_map_distance(p, 9.8235232703, 1.37, -3, .025);
  ASSERT_TRUE(support);
  // At decision2804 only19.033846m remained in the original map window.
  // Affine wire=net stopping distance would incorrectly fit in that map.
  EXPECT_GT(*support, 19.0338463966);
  EXPECT_LT(9.8235232703 * 9.8235232703 / 6.0, 19.0338463966);
  EXPECT_FALSE(vehicle::nominal_stop_map_distance(p, 2, 1, 0, .025));
  EXPECT_FALSE(vehicle::nominal_stop_map_distance({}, 2, 1, -3, .025));
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

TEST(MpccVehiclePrediction, DelayedPublicationCannotRetroactivelyBrakeTheBody)
{
  const auto p = vehicle_model();
  const vehicle::TimedState initial{1, {0, 0, 0, 8, 0, 0, 0, 0}};
  std::vector<vehicle::PublishedCommand> history{{0, 1, 0}};
  ASSERT_TRUE(vehicle::record_serialized_publication(history, {1, -3, 0}, 1.15, .63));
  const auto predicted = vehicle::predict_published_history(initial, 1.2, 1.2, history, p, 0, .1);
  ASSERT_TRUE(predicted);
  const auto before_publication = vehicle::advance(initial.state, {1, 0}, p, .15);
  ASSERT_TRUE(before_publication);
  const auto after_publication = vehicle::advance(before_publication->state, {-3, 0}, p, .05);
  ASSERT_TRUE(after_publication);
  EXPECT_NEAR(predicted->current.x_m, after_publication->state.x_m, 1e-10);
  EXPECT_NEAR(predicted->current.forward_velocity_mps,
    after_publication->state.forward_velocity_mps, 1e-10);
  EXPECT_DOUBLE_EQ(history.back().published_sec, 1.15);
}

TEST(MpccVehiclePrediction, PublicationHistoryPreservesCausalityAcrossClockBoundaries)
{
  std::vector<vehicle::PublishedCommand> history;
  ASSERT_TRUE(vehicle::record_serialized_publication(history, {1, 1, 0}, .995, .5));
  EXPECT_DOUBLE_EQ(history.back().published_sec, 1);
  ASSERT_TRUE(vehicle::record_serialized_publication(history, {1, -3, .1}, 1.01, .5));
  ASSERT_TRUE(vehicle::record_serialized_publication(history, {1.01, 0, .2}, 1.01, .5));
  ASSERT_EQ(history.size(), 2U);
  EXPECT_DOUBLE_EQ(history.back().wire_acceleration_mps2, 0);
  EXPECT_DOUBLE_EQ(history.back().wire_steering_rad, .2);
  EXPECT_FALSE(vehicle::record_serialized_publication(history, {NAN, 1, 0}, 2, .5));
  EXPECT_FALSE(vehicle::record_serialized_publication(history, {2, 1, 0}, NAN, .5));
  EXPECT_FALSE(vehicle::record_serialized_publication(history, {2, NAN, 0}, 2, .5));
  EXPECT_FALSE(vehicle::record_serialized_publication(history, {2, 1, 0}, 2, -1));
  EXPECT_EQ(history.size(), 2U);
  ASSERT_TRUE(vehicle::record_serialized_publication(history, {2, 1, 0}, 2.1, .5));
  ASSERT_EQ(history.size(), 2U);
  EXPECT_DOUBLE_EQ(history.front().published_sec, 1.01);
  ASSERT_TRUE(vehicle::record_serialized_publication(history, {.1, -3, 0}, .11, .5));
  ASSERT_EQ(history.size(), 1U);
  EXPECT_DOUBLE_EQ(history.back().published_sec, .11);
}

TEST(MpccVehiclePrediction, NewPacketCannotBorrowTheOldBrakingPrefix)
{
  const auto parameters = vehicle_model();
  const vehicle::ObservationProvenance observation{
    {1, {0, 0, 0, 3.9, 0, 0, 0, 0}}, 1, 1, 1, 1, 1.2, 0, .1,
    {{0, -3, 0}, {1, -3, 0}}};
  const vehicle::PublishedCommand proposal{1, 1, .2};
  const auto prediction = vehicle::predict_prospective_publication(
    observation, proposal, parameters);
  ASSERT_TRUE(prediction);
  const auto first = vehicle::advance(observation.initial.state, {1, 0}, parameters, .1);
  ASSERT_TRUE(first);
  auto steering_applied = first->state;
  steering_applied.desired_steering_rad = static_cast<float>(.2) / parameters.steering_wire_gain;
  const auto second = vehicle::advance(steering_applied, {1, 0}, parameters, .1);
  ASSERT_TRUE(second);
  EXPECT_NEAR(prediction->control_origin.x_m, second->state.x_m, 1e-12);
  EXPECT_NEAR(prediction->control_origin.yaw_rate_radps, second->state.yaw_rate_radps, 1e-12);
  EXPECT_NEAR(prediction->control_origin.forward_velocity_mps,
    second->state.forward_velocity_mps, 1e-12);
  const auto old_prefix = vehicle::predict_published_history(
    observation.initial, 1, 1.2, observation.commands, parameters, 0, .1);
  ASSERT_TRUE(old_prefix);
  EXPECT_GT(prediction->control_origin.forward_velocity_mps,
    old_prefix->control_origin.forward_velocity_mps + .4);
  ASSERT_EQ(prediction->observation.commands.size(), 2U);
  EXPECT_DOUBLE_EQ(prediction->observation.commands.back().wire_acceleration_mps2, -3);
  EXPECT_DOUBLE_EQ(prediction->proposed_packet.wire_acceleration_mps2, 1);
  EXPECT_DOUBLE_EQ(prediction->proposed_packet.wire_steering_rad, static_cast<float>(.2));
  EXPECT_DOUBLE_EQ(prediction->current.forward_velocity_mps, 3.9);
  EXPECT_TRUE(vehicle::publication_packet_matches(
    *prediction, 1, 1, .2 / parameters.steering_wire_gain, parameters.steering_wire_gain));
  EXPECT_FALSE(vehicle::publication_packet_matches(
    *prediction, 1, -3, .2 / parameters.steering_wire_gain, parameters.steering_wire_gain));
  EXPECT_FALSE(vehicle::publication_packet_matches(
    *prediction, 1.01, 1, .2 / parameters.steering_wire_gain, parameters.steering_wire_gain));
}

TEST(MpccVehiclePrediction, ProspectivePacketRequiresTheCapturedPublicationEpoch)
{
  const auto parameters = vehicle_model();
  const vehicle::ObservationProvenance observation{
    {1, {0, 0, 0, 3.9, 0, 0, 0, 0}}, 1, 1, 1, 1, 1.13, 0, .1,
    {{0, -3, 0}}};
  EXPECT_FALSE(vehicle::predict_prospective_publication(observation, {1.01, 1, 0}, parameters));
  EXPECT_FALSE(vehicle::predict_prospective_publication(observation, {.99, 1, 0}, parameters));
  EXPECT_FALSE(vehicle::predict_prospective_publication(observation, {1, NAN, 0}, parameters));
  auto missing = observation;
  missing.commands.clear();
  EXPECT_FALSE(vehicle::predict_prospective_publication(missing, {1, -3, 0}, parameters));
  auto regressed = observation;
  regressed.now_sec = .9;
  EXPECT_FALSE(vehicle::predict_prospective_publication(regressed, {.9, -3, 0}, parameters));
  EXPECT_FALSE(vehicle::predict_prospective_publication(observation, {1, -3, 0}, {}));
}
