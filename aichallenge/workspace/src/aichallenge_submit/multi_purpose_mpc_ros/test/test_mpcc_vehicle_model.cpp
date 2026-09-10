#include "mpcc_vehicle_model_fixture.hpp"
#include "multi_purpose_mpc_ros/mpcc_vehicle_prediction.hpp"
#include "multi_purpose_mpc_ros/mpcc_applied_input_prediction.hpp"

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
  ASSERT_EQ(history.size(), 3U);
  EXPECT_DOUBLE_EQ(history[1].wire_acceleration_mps2, -3);
  EXPECT_DOUBLE_EQ(history[1].wire_steering_rad, .1);
  EXPECT_DOUBLE_EQ(history.back().wire_acceleration_mps2, 0);
  EXPECT_DOUBLE_EQ(history.back().wire_steering_rad, .2);
  EXPECT_FALSE(vehicle::record_serialized_publication(history, {NAN, 1, 0}, 2, .5));
  EXPECT_FALSE(vehicle::record_serialized_publication(history, {2, 1, 0}, NAN, .5));
  EXPECT_FALSE(vehicle::record_serialized_publication(history, {2, NAN, 0}, 2, .5));
  EXPECT_FALSE(vehicle::record_serialized_publication(history, {2, 1, 0}, 2, -1));
  EXPECT_EQ(history.size(), 3U);
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

namespace
{
vehicle::ObservationProvenance applied_observation()
{
  return {{1.4, {0, 0, .2, 2, .02, .05, 0, 0}}, 1.39, 1.38, 1.39,
    1.45, 1.58, 0, .1, {{0, 1, 0}, {1.1, 1, 0}, {1.2, 1, 0}, {1.3, 1, 0}, {1.4, 1, 0}}};
}

vehicle::InputApplicationProfile applied_profile()
{
  return {"test-empirical-packet-window", .25, .25, .1};
}

vehicle::PublishedInputProgram stop_program(const double now)
{
  return {.025, {{now, -3, 0}}, true};
}
}  // namespace

TEST(MpccAppliedInput, EqualEpochPacketsRemainPossibleWhileNominalUsesTheLast)
{
  const auto p = vehicle_model();
  auto observation = applied_observation();
  observation.commands = {{0, 1, 0}, {1.1, 1, 0}, {1.2, -3, -.125}, {1.2, 1, .125}, {1.4, 1, 0}};
  ASSERT_TRUE(vehicle::valid(observation));
  const auto program = stop_program(observation.now_sec);
  const auto range = vehicle::applied_input_bounds(observation.commands, program,
    applied_profile(), 1.4, 1.405);
  ASSERT_TRUE(range);
  EXPECT_DOUBLE_EQ(range->acceleration_mps2.lower, -3);
  EXPECT_DOUBLE_EQ(range->acceleration_mps2.upper, 1);
  EXPECT_DOUBLE_EQ(range->wire_steering_rad.lower, -.125);
  EXPECT_DOUBLE_EQ(range->wire_steering_rad.upper, .125);
  const auto all = vehicle::predict_published_history(observation.initial, observation.now_sec,
    observation.control_origin_sec, observation.commands, p, 0, .1);
  observation.commands.erase(observation.commands.begin() + 2);
  const auto last = vehicle::predict_published_history(observation.initial, observation.now_sec,
    observation.control_origin_sec, observation.commands, p, 0, .1);
  ASSERT_TRUE(all); ASSERT_TRUE(last);
  EXPECT_DOUBLE_EQ(all->control_origin.x_m, last->control_origin.x_m);
  EXPECT_DOUBLE_EQ(all->control_origin.tire_steering_rad, last->control_origin.tire_steering_rad);
}

TEST(MpccAppliedInput, ChannelWindowsPreserveSeparateMechanicalDelayAndCommonTail)
{
  const auto observation = applied_observation();
  const vehicle::PublishedInputProgram program{.025, {{observation.now_sec, -3, .125}}, true};
  const auto initial = vehicle::applied_input_bounds(observation.commands, program,
    applied_profile(), observation.now_sec, observation.now_sec + .005);
  ASSERT_TRUE(initial);
  EXPECT_DOUBLE_EQ(initial->acceleration_mps2.lower, -3);
  EXPECT_DOUBLE_EQ(initial->acceleration_mps2.upper, 1);
  EXPECT_DOUBLE_EQ(initial->wire_steering_rad.lower, 0);
  EXPECT_DOUBLE_EQ(initial->wire_steering_rad.upper, 0);
  const auto delivered = vehicle::applied_input_bounds(observation.commands, program,
    applied_profile(), 1.81, 1.815);
  ASSERT_TRUE(delivered);
  EXPECT_DOUBLE_EQ(delivered->acceleration_mps2.lower, -3);
  EXPECT_DOUBLE_EQ(delivered->acceleration_mps2.upper, -3);
  EXPECT_DOUBLE_EQ(delivered->wire_steering_rad.lower, .125);
  EXPECT_DOUBLE_EQ(delivered->wire_steering_rad.upper, .125);
}

TEST(MpccAppliedInput, MissingHistoryAndUnserializedProgramsCannotAcquireAContext)
{
  const auto p = vehicle_model(); auto observation = applied_observation();
  auto program = stop_program(observation.now_sec);
  program.commands.front().wire_steering_rad = .1;
  EXPECT_FALSE(vehicle::valid(program, observation.now_sec));
  EXPECT_EQ(vehicle::applied_input_context_fingerprint(observation, program, applied_profile(), p), 0U);
  program = stop_program(observation.now_sec);
  observation.commands = {{1.25, 1, 0}, {1.4, 1, 0}};
  ASSERT_TRUE(vehicle::valid(observation));
  const auto missing = vehicle::predict_applied_inputs_to_rest(observation, program, applied_profile(), p);
  EXPECT_EQ(missing.reason, vehicle::AppliedInputRejectReason::HistoryUnavailable);
  EXPECT_FALSE(missing.tube);
  program.commands.front().wire_acceleration_mps2 = 1;
  EXPECT_FALSE(vehicle::valid(program, observation.now_sec));
}

TEST(MpccAppliedInput, ContextBindsEverySourcePacketProfileAndFutureProgram)
{
  const auto p = vehicle_model(); const auto observation = applied_observation();
  const auto program = stop_program(observation.now_sec); const auto profile = applied_profile();
  const auto original = vehicle::applied_input_context_fingerprint(observation, program, profile, p);
  ASSERT_NE(original, 0U);
  auto changed = observation; changed.velocity_source_sec -= .01;
  EXPECT_NE(original, vehicle::applied_input_context_fingerprint(changed, program, profile, p));
  changed = observation; changed.commands.insert(changed.commands.begin() + 3, {1.2, -3, 0});
  EXPECT_NE(original, vehicle::applied_input_context_fingerprint(changed, program, profile, p));
  auto other_profile = profile; other_profile.steering_receipt_age_sec = .2;
  EXPECT_NE(original, vehicle::applied_input_context_fingerprint(observation, program, other_profile, p));
  auto other_program = program;
  other_program.commands.push_back({observation.now_sec + .025, -3, .125});
  EXPECT_NE(original, vehicle::applied_input_context_fingerprint(observation, other_program, profile, p));
}

TEST(MpccAppliedInput, SourceToRestContainsNativeResponsesToOneCommonProgram)
{
  const auto p = vehicle_model(); const auto observation = applied_observation();
  const auto prediction = vehicle::predict_applied_inputs_to_rest(
    observation, stop_program(observation.now_sec), applied_profile(), p);
  ASSERT_EQ(prediction.reason, vehicle::AppliedInputRejectReason::None);
  ASSERT_TRUE(prediction.tube); const auto & tube = *prediction.tube;
  ASSERT_FALSE(tube.source_to_rest.empty());
  EXPECT_GT(tube.publication_body[3].upper, observation.initial.state.forward_velocity_mps);
  EXPECT_GT(tube.rest_sec, observation.now_sec + .25);
  for (int phase = 0; phase < 11; ++phase) {
    auto state = observation.initial.state; state.x_m = 0; state.y_m = 0; state.yaw_rad = 0;
    for (const auto & sample : tube.source_to_rest) {
      // Each oracle uses the identical common packet program. The chosen
      // historical positive input persists for one different admissible phase.
      const bool old_input = sample.begin_sec < observation.now_sec + phase * .025;
      const double acceleration = old_input ? sample.inputs.acceleration_mps2.upper :
        sample.inputs.acceleration_mps2.lower;
      state.desired_steering_rad = 0;
      const auto next = vehicle::advance(state, {acceleration, 0}, p, sample.duration_sec);
      ASSERT_TRUE(next); state = next->state;
      const std::array<double, 8> values{state.x_m, state.y_m, state.yaw_rad, state.forward_velocity_mps,
        state.lateral_velocity_mps, state.yaw_rate_radps, state.desired_steering_rad, state.tire_steering_rad};
      for (std::size_t i = 0; i < values.size(); ++i) {
        EXPECT_GE(values[i], sample.endpoint_body[i].lower);
        EXPECT_LE(values[i], sample.endpoint_body[i].upper);
      }
    }
    EXPECT_DOUBLE_EQ(state.forward_velocity_mps, 0);
    EXPECT_DOUBLE_EQ(state.lateral_velocity_mps, 0);
    EXPECT_DOUBLE_EQ(state.yaw_rate_radps, 0);
    const auto restart = vehicle::advance(state, {1, 0}, p, .1);
    ASSERT_TRUE(restart); EXPECT_GT(restart->state.forward_velocity_mps, 0);
  }
}

TEST(MpccAppliedInput, InitiallyRestingBodyDoesNotDiscardThePendingPositiveInput)
{
  const auto p = vehicle_model(); auto observation = applied_observation();
  observation.initial.state = {};
  const auto prediction = vehicle::predict_applied_inputs_to_rest(
    observation, stop_program(observation.now_sec), applied_profile(), p);
  ASSERT_TRUE(prediction.tube);
  EXPECT_GT(prediction.tube->publication_body[3].upper, 0);
  // The new proposal is already braking. The positive packet was published
  // 50ms earlier, so its actual age window expires before now+250ms.
  EXPECT_GT(prediction.tube->rest_sec,
    observation.commands.back().published_sec + applied_profile().acceleration_age_sec);
  bool pending_positive_after_publication = false;
  for (const auto & sample : prediction.tube->source_to_rest) {
    if (sample.begin_sec >= observation.now_sec && sample.inputs.acceleration_mps2.upper > 0 &&
      sample.endpoint_body[3].upper > 0) pending_positive_after_publication = true;
  }
  EXPECT_TRUE(pending_positive_after_publication);
  EXPECT_DOUBLE_EQ(prediction.tube->source_to_rest.back().endpoint_body[3].upper, 0);
}

TEST(MpccAppliedInput, FuturePacketsCannotFillEarlierGapsInTheDeclaredInputContext)
{
  const vehicle::InputApplicationProfile profile{"test-gap", .125, .25, .1};
  const vehicle::PublishedInputProgram next{.025, {{1.5, -3, .125}}, true};
  EXPECT_FALSE(vehicle::applied_input_bounds(
    {{0, 1, 0}, {1.35, 1, 0}}, next, profile, 1.495, 1.505));
  const vehicle::PublishedInputProgram later{.025, {{1.525, -3, .125}}, true};
  EXPECT_FALSE(vehicle::applied_input_bounds(
    {{0, 1, 0}, {1.377, 1, 0}}, later, profile, 1.5, 1.53));
  EXPECT_TRUE(vehicle::applied_input_bounds(
    {{0, 1, 0}, {1.375, 1, 0}}, next, profile, 1.495, 1.505));
}
