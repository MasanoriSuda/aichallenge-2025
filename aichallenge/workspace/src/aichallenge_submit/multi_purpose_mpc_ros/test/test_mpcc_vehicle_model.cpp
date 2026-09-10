#include "multi_purpose_mpc_ros/detail/mpcc_vehicle_enclosure.hpp"
#include "multi_purpose_mpc_ros/detail/mpcc_footprint_enclosure.hpp"
#include "mpcc_vehicle_model_fixture.hpp"
#include "mpcc_resting_packet_fixture.hpp"
#include "multi_purpose_mpc_ros/mpcc_vehicle_prediction.hpp"
#include "multi_purpose_mpc_ros/mpcc_applied_input_prediction.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <random>

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

TEST(MpccAppliedInputPrediction, RestStillCoversTheFirstCompletePublisherInterval)
{
  namespace vehicle = multi_purpose_mpc_ros::mpcc_vehicle_model;
  const auto model = multi_purpose_mpc_ros::test::vehicle_model();
  vehicle::ObservationProvenance observation{{1, {0, 0, 0, 0, 0, 0, 0, 0}},
    1, 1, 1, 1, 1.13, 0, .1, {}};
  for (int i = 0; i <= 40; ++i) observation.commands.push_back({i * .025, -3, 0});
  const vehicle::PublishedInputProgram program{.025, {{1, -3, 0}}, true};
  const auto prediction = vehicle::predict_applied_inputs_to_rest(
    observation, program, {"complete-first-period", .25, .25, .1}, model);
  ASSERT_TRUE(prediction.tube);
  EXPECT_GE(prediction.tube->rest_sec, 1.025);
  EXPECT_DOUBLE_EQ(prediction.tube->source_to_rest.back().endpoint_body[3].upper, 0);
}

TEST(MpccVehiclePrediction, PublicationBindingUsesBothActualRosFloatBoundaries)
{
  constexpr double physical = -.271520636, gain = 1.435;
  const double actual_wire = static_cast<float>(static_cast<double>(static_cast<float>(physical)) * gain);
  ASSERT_NE(actual_wire, static_cast<float>(physical * gain));
  vehicle::ProspectivePublicationPrediction prediction;
  prediction.observation.now_sec = 1;
  prediction.proposed_packet = {1, -3, actual_wire};
  EXPECT_TRUE(vehicle::publication_packet_matches(prediction, 1, -3, physical, gain));
  prediction.proposed_packet.wire_steering_rad = static_cast<float>(physical * gain);
  EXPECT_FALSE(vehicle::publication_packet_matches(prediction, 1, -3, physical, gain));
}

TEST(MpccVehicleModel, RestEnclosurePreservesStaticPoseAcrossRepeatedInputWindows)
{
  namespace range = vehicle::numerical;
  const auto p = vehicle_model();
  auto body = range::point(vehicle::State{.2, -.3, .15, 0, 0, 0, .1, -.1});
  body[0] = {-.2, .3}; body[1] = {-.4, .5}; body[2] = {-.1, .2};
  body[6] = {-.1, .2};
  const auto initial = body;
  for (int i = 0; i < 1000; ++i) body = range::centered_step(body, {-3, 0}, p, .005, true);
  for (const auto i : {0, 1, 2, 6}) {
    EXPECT_EQ(body[i].lo, initial[i].lo);
    EXPECT_EQ(body[i].hi, initial[i].hi);
  }
  EXPECT_TRUE(range::at_rest(body));
  EXPECT_TRUE(std::isfinite(body[7].lo) && std::isfinite(body[7].hi));
}

TEST(MpccAppliedInput, PublishedPacketsAtRestStayWithinOriginalForwardStateBound)
{
  const auto observation = multi_purpose_mpc_ros::test::resting_packet_observation();
  const auto program = multi_purpose_mpc_ros::test::resting_packet_program();
  const auto prediction = vehicle::predict_applied_inputs_to_rest(
    observation, program, applied_profile(), vehicle_model());
  ASSERT_TRUE(prediction.tube);
  double minimum_u = INFINITY;
  for (const auto & sample : prediction.tube->source_to_rest) {
    if (sample.begin_sec >= observation.now_sec)
      minimum_u = std::min(minimum_u, sample.swept_body[3].lower);
  }
  EXPECT_GE(minimum_u, -0.00741003100948554);
  EXPECT_DOUBLE_EQ(prediction.tube->source_to_rest.back().endpoint_body[3].upper, 0);
}

TEST(MpccAppliedInput, PacketUnionPreservesZeroSmallValuesEqualEpochsAndDelayedTail)
{
  const vehicle::PublishedInputProgram program{.025, {{1.45, -3, .125}}, true};
  const std::vector<vehicle::PublishedCommand> history{
    {0, 1, 0}, {1.1, 1, 0}, {1.2, -3, -.125}, {1.2, 0, .125},
    {1.2, .0001220703125, -.25}, {1.2, -.0001220703125, .25},
    {1.3, 1.25, 0}, {1.4, -.25, .125}};
  const auto profile = applied_profile();
  for (int step = 0; step < 100; ++step) {
    const double begin = 1.4 + step * .005, end = begin + .005;
    const auto bounds = vehicle::applied_input_bounds(history, program, profile, begin, end);
    ASSERT_TRUE(bounds);
    const auto contains_acceleration = [&](double value) {
      for (const auto & group : bounds->acceleration_sign_groups)
        if (group && value >= group->lower && value <= group->upper) return true;
      return false;
    };
    // Independently check packet/channel eligibility. Equal-time members must
    // survive even though only the last is used by nominal history prediction.
    for (const auto * packets : {&history, &program.commands}) {
      for (const auto & packet : *packets) {
        if (packet.published_sec >= begin - profile.acceleration_age_sec &&
          packet.published_sec <= end) {
          EXPECT_TRUE(contains_acceleration(packet.wire_acceleration_mps2));
        }
        if (packet.published_sec >= begin - profile.steering_mechanical_delay_sec -
          profile.steering_receipt_age_sec &&
          packet.published_sec <= end - profile.steering_mechanical_delay_sec) {
          EXPECT_GE(packet.wire_steering_rad, bounds->wire_steering_rad.lower);
          EXPECT_LE(packet.wire_steering_rad, bounds->wire_steering_rad.upper);
        }
      }
    }
    if (step == 0) {
      EXPECT_TRUE(contains_acceleration(0));
      ASSERT_TRUE(bounds->acceleration_sign_groups[0]);
      ASSERT_TRUE(bounds->acceleration_sign_groups[2]);
      EXPECT_DOUBLE_EQ(bounds->acceleration_sign_groups[0]->upper, -.0001220703125);
      EXPECT_DOUBLE_EQ(bounds->acceleration_sign_groups[2]->lower, .0001220703125);
    }
    if (begin > 1.75) {
      EXPECT_TRUE(contains_acceleration(-3));
      EXPECT_FALSE(bounds->acceleration_sign_groups[1]);
      EXPECT_FALSE(bounds->acceleration_sign_groups[2]);
    }
  }
}

TEST(MpccAppliedInput, PacketUnionEnclosesIndependentChannelResponsesAndAllSigns)
{
  const auto p = vehicle_model();
  auto observation = multi_purpose_mpc_ros::test::resting_packet_observation();
  const auto program = multi_purpose_mpc_ros::test::resting_packet_program();
  // Also exercise real zero and near-zero packets. They must remain possible
  // even when including them makes a subsequent physical certificate reject.
  observation.commands.insert(observation.commands.end() - 1,
    {{15.084999662, 0, -.125}, {15.084999662, .0001220703125, .125},
      {15.084999662, -.0001220703125, 0}});
  const auto prediction = vehicle::predict_applied_inputs_to_rest(
    observation, program, applied_profile(), p);
  ASSERT_TRUE(prediction.tube);
  auto initial = observation.initial.state; initial.x_m = initial.y_m = initial.yaw_rad = 0;
  std::vector<vehicle::State> states(128, initial);
  std::mt19937_64 random(20260911); std::uniform_real_distribution<double> unit(0, 1);
  bool zero_seen = false, small_positive_seen = false, small_negative_seen = false;
  for (const auto & sample : prediction.tube->source_to_rest) {
    std::vector<vehicle::ScalarRange> groups;
    for (const auto & group : sample.inputs.acceleration_sign_groups)
      if (group) groups.push_back(*group);
    ASSERT_FALSE(groups.empty());
    for (std::size_t arm = 0; arm < states.size(); ++arm) {
      const auto & group = groups[arm % groups.size()];
      const double af = arm < 12 ? double((arm / groups.size()) % 2) : unit(random);
      const double sf = arm < 12 ? double((arm / (2 * groups.size())) % 2) : unit(random);
      const double acceleration = group.lower + af * (group.upper - group.lower);
      zero_seen = zero_seen || acceleration == 0;
      small_positive_seen = small_positive_seen || acceleration == .0001220703125;
      small_negative_seen = small_negative_seen || acceleration == -.0001220703125;
      auto & state = states[arm];
      state.desired_steering_rad = (sample.inputs.wire_steering_rad.lower + sf *
        (sample.inputs.wire_steering_rad.upper - sample.inputs.wire_steering_rad.lower)) /
        p.steering_wire_gain;
      const auto next = vehicle::advance(state, {acceleration, 0}, p, sample.duration_sec);
      ASSERT_TRUE(next); state = next->state;
      const auto values = vehicle::numerical::values(state);
      for (std::size_t i = 0; i < values.size(); ++i) {
        ASSERT_GE(values[i], sample.endpoint_body[i].lower);
        ASSERT_LE(values[i], sample.endpoint_body[i].upper);
      }
    }
  }
  EXPECT_TRUE(zero_seen); EXPECT_TRUE(small_positive_seen); EXPECT_TRUE(small_negative_seen);
  for (const auto & state : states) {
    EXPECT_DOUBLE_EQ(state.forward_velocity_mps, 0);
    EXPECT_DOUBLE_EQ(state.lateral_velocity_mps, 0);
    EXPECT_DOUBLE_EQ(state.yaw_rate_radps, 0);
  }
}

TEST(MpccAppliedInput, ValidatorRejectsWithoutReturningAPartialStopTube)
{
  const auto observation = applied_observation();
  const auto program = stop_program(observation.now_sec);
  const auto p = vehicle_model();
  std::size_t calls = 0;
  const auto at_publication = vehicle::predict_applied_inputs_to_rest(
    observation, program, applied_profile(), p,
    [&](const vehicle::BodyRanges &, double begin, double end) {
      ++calls;
      EXPECT_DOUBLE_EQ(begin, observation.now_sec);
      EXPECT_DOUBLE_EQ(end, observation.now_sec);
      return false;
    });
  EXPECT_EQ(at_publication.reason, vehicle::AppliedInputRejectReason::ValidationRejected);
  EXPECT_FALSE(at_publication.tube);
  EXPECT_EQ(calls, 1U);
  calls = 0;
  const auto after_publication = vehicle::predict_applied_inputs_to_rest(
    observation, program, applied_profile(), p,
    [&](const vehicle::BodyRanges &, double begin, double end) {
      ++calls;
      EXPECT_GE(begin, observation.now_sec);
      EXPECT_GE(end, begin);
      return calls < 4U;
    });
  EXPECT_EQ(after_publication.reason, vehicle::AppliedInputRejectReason::ValidationRejected);
  EXPECT_FALSE(after_publication.tube);
  EXPECT_EQ(calls, 4U);
}

TEST(MpccAppliedInput, AcceptingValidatorPreservesEveryOriginalSampleAndCompleteRest)
{
  const auto observation = multi_purpose_mpc_ros::test::resting_packet_observation();
  const auto program = multi_purpose_mpc_ros::test::resting_packet_program();
  const auto p = vehicle_model();
  const auto original = vehicle::predict_applied_inputs_to_rest(
    observation, program, applied_profile(), p);
  ASSERT_TRUE(original.tube);
  std::vector<const vehicle::AppliedInputSample *> expected;
  for (const auto & sample : original.tube->source_to_rest)
    if (sample.begin_sec >= observation.now_sec) expected.push_back(&sample);
  std::size_t calls = 0;
  const auto checked = vehicle::predict_applied_inputs_to_rest(
    observation, program, applied_profile(), p,
    [&](const vehicle::BodyRanges & ranges, double begin, double end) {
      const auto * sample = calls == 0 ? nullptr : expected.at(calls - 1);
      const auto & wanted = sample ? sample->swept_body : original.tube->publication_body;
      EXPECT_DOUBLE_EQ(begin, sample ? sample->begin_sec : observation.now_sec);
      EXPECT_DOUBLE_EQ(end, sample ? sample->end_sec : observation.now_sec);
      for (std::size_t i = 0; i < ranges.size(); ++i) {
        EXPECT_DOUBLE_EQ(ranges[i].lower, wanted[i].lower);
        EXPECT_DOUBLE_EQ(ranges[i].upper, wanted[i].upper);
      }
      ++calls;
      return true;
    });
  ASSERT_TRUE(checked.tube);
  EXPECT_EQ(calls, expected.size() + 1U);
  EXPECT_EQ(checked.tube->source_to_rest.size(), original.tube->source_to_rest.size());
  EXPECT_EQ(checked.tube->context_fingerprint, original.tube->context_fingerprint);
  EXPECT_DOUBLE_EQ(checked.tube->rest_sec, original.tube->rest_sec);
  EXPECT_DOUBLE_EQ(checked.tube->source_to_rest.back().endpoint_body[3].upper, 0);
}

TEST(MpccVehicleModel, CapturedPeerSeparatesFromAllFootprintsDespiteEnclosingCorner)
{
  namespace num = vehicle::numerical;
  namespace rec = multi_purpose_mpc_ros::recovery_footprint;
  // dev2-r22 D1 decision1024/source338, first common-program peer rejection.
  // Original full swept pose ranges, source pose, footprint and CA1 peer.
  auto box = num::point(vehicle::State{});
  box[0] = {1.5779542437334122, 2.6294458569680037};
  box[1] = {.091724902986313681, .62188217282684899};
  box[2] = {.20418834353971457, .45957083474776345};
  const rec::Pose2D origin{89628.34931049822, 43132.880683355696, 1.8705564481804007};
  const rec::FootprintExtents original{1.615, .51, .768, .768, .05};
  rec::CircleObstacle peer{89632.56551890414, 43131.87186630296,
    -.6553381184394249, .7952964318769234, 1.9309999998882412,
    -.7593027315952433, .5258057092313208, 1};
  const double t0 = 13.559999728999999 - 12.104999729, t1 = t0 + .005;
  peer.radius_m = num::up(peer.radius_m +
    num::up(peer.maximum_speed(t0, t1) * num::up((t1 - t0) / 2)));
  const auto center = peer.predicted_center((t0 + t1) / 2);
  const auto enclosure = num::footprint(box, original);
  const auto to_world = [&](const rec::Pose2D & p) {
    return rec::Pose2D{
      origin.x_m + std::cos(origin.yaw_rad) * p.x_m - std::sin(origin.yaw_rad) * p.y_m,
      origin.y_m + std::sin(origin.yaw_rad) * p.x_m + std::cos(origin.yaw_rad) * p.y_m,
      origin.yaw_rad + p.yaw_rad};
  };
  const auto enclosing = rec::circle_obstacle_clearance_at_time(
    enclosure.extents, to_world(enclosure.pose), peer, (t0 + t1) / 2);
  ASSERT_TRUE(enclosing);
  EXPECT_LT(*enclosing, 0);
  const auto separated = num::separating_circle_clearance(
    box, original, origin, enclosure, center[0], center[1], peer.radius_m);
  ASSERT_TRUE(separated);
  EXPECT_GT(*separated, .11);
  for (const double x : {box[0].lo, box[0].hi}) {
    for (const double y : {box[1].lo, box[1].hi}) {
      for (int i = 0; i <= 64; ++i) {
        const double yaw = box[2].lo + (box[2].hi - box[2].lo) * i / 64;
        const auto exact = rec::circle_obstacle_clearance_at_time(
          original, to_world({x, y, yaw}), peer, (t0 + t1) / 2);
        ASSERT_TRUE(exact);
        EXPECT_LE(*separated, *exact);
      }
    }
  }
}

TEST(MpccVehicleModel, SeparatingPlaneBoundsIndependentNativeFootprintSamples)
{
  namespace num = vehicle::numerical;
  namespace rec = multi_purpose_mpc_ros::recovery_footprint;
  std::mt19937_64 random(20260911);
  std::uniform_real_distribution<double> unit(0, 1);
  const rec::FootprintExtents original{1.615, .51, .768, .768, .05};
  std::size_t certified = 0;
  for (std::size_t scene = 0; scene < 1000; ++scene) {
    const rec::Pose2D origin{90000 + 10 * unit(random), 40000 + 10 * unit(random),
      2 * M_PI * unit(random) - M_PI};
    auto box = num::point(vehicle::State{});
    for (std::size_t j = 0; j < 3; ++j) {
      const double mid = 2 * unit(random) - 1;
      const double radius = unit(random) * (j == 2 ? 3.2 : .7);
      box[j] = {mid - radius, mid + radius};
    }
    const auto enclosure = num::footprint(box, original);
    const rec::CircleObstacle peer{origin.x_m + 12 * unit(random) - 6,
      origin.y_m + 12 * unit(random) - 6, 0, 0, .1 + 2 * unit(random)};
    const auto separated = num::separating_circle_clearance(
      box, original, origin, enclosure, peer.x_m, peer.y_m, peer.radius_m);
    if (!separated) continue;
    ++certified;
    ASSERT_GE(*separated, 0);
    for (std::size_t sample = 0; sample < 32; ++sample) {
      std::array<double, 3> pose;
      for (std::size_t j = 0; j < 3; ++j) {
        const double fraction = sample < 8 ? static_cast<double>((sample >> j) & 1) : unit(random);
        pose[j] = box[j].lo + (box[j].hi - box[j].lo) * fraction;
      }
      const rec::Pose2D world{
        origin.x_m + std::cos(origin.yaw_rad) * pose[0] - std::sin(origin.yaw_rad) * pose[1],
        origin.y_m + std::sin(origin.yaw_rad) * pose[0] + std::cos(origin.yaw_rad) * pose[1],
        origin.yaw_rad + pose[2]};
      const auto exact = rec::circle_obstacle_clearance_at_time(original, world, peer, 0);
      ASSERT_TRUE(exact);
      EXPECT_LE(*separated, *exact);
    }
  }
  EXPECT_GT(certified, 100U);
}

TEST(MpccVehicleModel, SeparatingPlaneRejectsContactUncertainRotationAndInvalidInput)
{
  namespace num = vehicle::numerical;
  namespace rec = multi_purpose_mpc_ros::recovery_footprint;
  const rec::FootprintExtents original{1.615, .51, .768, .768, .05};
  const rec::Pose2D origin{};
  auto box = num::point(vehicle::State{});
  const auto enclosure = num::footprint(box, original);
  for (const double peer_x : {0., 1.615 + .05 + 1 - 1e-6, 1.615 + .05 + 1}) {
    EXPECT_FALSE(num::separating_circle_clearance(box, original, origin, enclosure, peer_x, 0, 1));
  }
  EXPECT_TRUE(num::separating_circle_clearance(
    box, original, origin, enclosure, 1.615 + .05 + 1 + 1e-6, 0, 1));
  box[2] = {-M_PI, M_PI};
  EXPECT_FALSE(num::separating_circle_clearance(
    box, original, origin, num::footprint(box, original), 2, 0, 1));
  box[0] = {NAN, NAN};
  EXPECT_THROW(num::separating_circle_clearance(
    box, original, origin, enclosure, 5, 0, 1), std::runtime_error);
}


namespace
{
vehicle::AppliedFootprintValidation corner_validation()
{
  const auto offsets = vehicle::numerical::footprint_vertex_offsets(
    {1.615, .51, .768, .768, .2});
  vehicle::AppliedFootprintValidation result;
  for (size_t i = 0; i < offsets.size(); ++i)
    result.local_offsets[i] = {offsets[i].lo, offsets[i].hi};
  result.validate = [](const auto &, const auto &, double, double) {return true;};
  return result;
}
}  // namespace

TEST(MpccAppliedInput, CornerPropagationPreservesBodyPopulationAndFullProgram)
{
  const auto observation = applied_observation();
  const auto program = stop_program(observation.now_sec);
  const auto original = vehicle::predict_applied_inputs_to_rest(
    observation, program, applied_profile(), vehicle_model());
  ASSERT_TRUE(original.tube);
  auto context = corner_validation();
  const auto extra = vehicle::predict_applied_inputs_to_rest(
    observation, program, applied_profile(), vehicle_model(), {}, &context);
  ASSERT_TRUE(extra.tube);
  EXPECT_EQ(extra.tube->context_fingerprint, original.tube->context_fingerprint);
  EXPECT_EQ(extra.tube->rest_sec, original.tube->rest_sec);
  EXPECT_EQ(extra.tube->maximum_body_partitions, original.tube->maximum_body_partitions);
  ASSERT_EQ(extra.tube->source_to_rest.size(), original.tube->source_to_rest.size());
  ASSERT_TRUE(extra.tube->publication_footprint);
  for (size_t k = 0; k < original.tube->source_to_rest.size(); ++k) {
    const auto & a = original.tube->source_to_rest[k];
    const auto & b = extra.tube->source_to_rest[k];
    EXPECT_EQ(a.begin_sec, b.begin_sec); EXPECT_EQ(a.end_sec, b.end_sec);
    ASSERT_TRUE(b.swept_footprint); ASSERT_TRUE(b.endpoint_footprint);
    for (size_t i = 0; i < 8; ++i) {
      EXPECT_EQ(a.endpoint_body[i].lower, b.endpoint_body[i].lower);
      EXPECT_EQ(a.endpoint_body[i].upper, b.endpoint_body[i].upper);
      EXPECT_EQ(a.swept_body[i].lower, b.swept_body[i].lower);
      EXPECT_EQ(a.swept_body[i].upper, b.swept_body[i].upper);
      EXPECT_EQ(original.tube->publication_body[i].lower, extra.tube->publication_body[i].lower);
      EXPECT_EQ(original.tube->publication_body[i].upper, extra.tube->publication_body[i].upper);
    }
  }
}

TEST(MpccAppliedInput, CornersEncloseNativeEndpointsAndContinuousPoseSweeps)
{
  namespace num = vehicle::numerical;
  const auto p = vehicle_model();
  const auto offsets = num::footprint_vertex_offsets({1.615, .51, .768, .768, .2});
  std::mt19937_64 random(20260912);
  std::uniform_real_distribution<double> unit(0, 1);
  size_t checked = 0;
  bool rest_seen = false, launch_seen = false, reverse_seen = false;
  for (double yaw : {-2.9, -.3, 0., 1.7, 3.1}) {
    for (double u : {-.05, -.01, 0., .01, .05, 2.}) {
      vehicle::State initial{0, 0, 0, u, .01, -.05, 0, .04};
      std::vector<num::Box> population{num::point(initial)};
      num::Box seed;
      const auto co = num::cosine(num::I(yaw)), so = num::sine(num::I(yaw));
      for (size_t k = 0; k < 8; k += 2) {
        seed[k] = co * offsets[k] - so * offsets[k+1];
        seed[k+1] = so * offsets[k] + co * offsets[k+1];
      }
      num::CornerPopulation corners{{offsets, yaw}, {seed}, seed};
      std::vector<vehicle::State> oracles(32, initial);
      for (size_t step = 0; step < 30; ++step) {
        const std::vector<num::I> arms = step < 12 ?
          std::vector<num::I>{{-3, -.0001}, {0}, {.0001, 1.37}} :
          std::vector<num::I>{{-3, -3}};
        const double limit = p.maximum_wire_steering_rad / p.steering_wire_gain;
        const num::I steer(-limit, limit);
        for (auto & body : population) body[6] = steer;
        num::Box swept;
        population = num::advance_partitioned_inputs(
          std::move(population), arms, p, .005, &swept, &corners);
        const auto endpoint = num::joined(corners.states);
        for (size_t arm = 0; arm < oracles.size(); ++arm) {
          auto & state = oracles[arm]; const auto before = state;
          const auto & acceleration = arms[arm % arms.size()];
          const double af = arm < 12 ? double((arm / arms.size()) % 2) : unit(random);
          const double sf = arm < 12 ? double((arm / (2 * arms.size())) % 2) : unit(random);
          state.desired_steering_rad = steer.lo + sf * (steer.hi - steer.lo);
          const auto next = vehicle::advance(state,
            {acceleration.lo + af * (acceleration.hi - acceleration.lo), 0}, p, .005);
          ASSERT_TRUE(next); state = next->state;
          rest_seen |= state.forward_velocity_mps == 0;
          launch_seen |= before.forward_velocity_mps == 0 && state.forward_velocity_mps > 0;
          reverse_seen |= state.forward_velocity_mps < 0;
          // Independent scalar model endpoints; the existing physical segment
          // sweep linearly interpolates body XY and yaw between these endpoints.
          for (int part = 0; part <= 8; ++part) {
            const double f = part / 8.;
            const double bx = before.x_m + f * (state.x_m - before.x_m);
            const double by = before.y_m + f * (state.y_m - before.y_m);
            const double angle = yaw + before.yaw_rad + f * (state.yaw_rad - before.yaw_rad);
            for (size_t k = 0; k < 8; k += 2) {
              const double x = (offsets[k].lo + offsets[k].hi) / 2;
              const double y = (offsets[k+1].lo + offsets[k+1].hi) / 2;
              const double X = std::cos(yaw)*bx - std::sin(yaw)*by + std::cos(angle)*x - std::sin(angle)*y;
              const double Y = std::sin(yaw)*bx + std::cos(yaw)*by + std::sin(angle)*x + std::cos(angle)*y;
              ASSERT_GE(X, corners.swept[k].lo); ASSERT_LE(X, corners.swept[k].hi);
              ASSERT_GE(Y, corners.swept[k+1].lo); ASSERT_LE(Y, corners.swept[k+1].hi);
              if (part == 8) {
                ASSERT_GE(X, endpoint[k].lo); ASSERT_LE(X, endpoint[k].hi);
                ASSERT_GE(Y, endpoint[k+1].lo); ASSERT_LE(Y, endpoint[k+1].hi);
              }
              checked += 2;
            }
          }
        }
      }
    }
  }
  EXPECT_EQ(checked, 2073600U);
  EXPECT_TRUE(rest_seen); EXPECT_TRUE(launch_seen); EXPECT_TRUE(reverse_seen);
}

TEST(MpccAppliedInput, InvalidCornerContextAndEitherValidatorFailClosed)
{
  namespace num = vehicle::numerical;
  const auto o = applied_observation(); const auto p = vehicle_model();
  auto context = corner_validation();
  context.local_offsets[3].lower = NAN;
  auto result = vehicle::predict_applied_inputs_to_rest(o, stop_program(o.now_sec),
    applied_profile(), p, {}, &context);
  EXPECT_EQ(result.reason, vehicle::AppliedInputRejectReason::NumericalFailure);
  EXPECT_FALSE(result.tube);
  context = corner_validation(); context.validate = {};
  result = vehicle::predict_applied_inputs_to_rest(o, stop_program(o.now_sec),
    applied_profile(), p, {}, &context);
  EXPECT_EQ(result.reason, vehicle::AppliedInputRejectReason::NumericalFailure);
  EXPECT_FALSE(result.tube);
  for (bool reject_body : {false, true}) {
    size_t calls = 0;
    context = corner_validation();
    context.validate = [&](const auto &, const auto &, double, double) {return reject_body || ++calls < 4;};
    result = vehicle::predict_applied_inputs_to_rest(o, stop_program(o.now_sec),
      applied_profile(), p, [&](const auto &, double, double) {return !reject_body;}, &context);
    EXPECT_EQ(result.reason, vehicle::AppliedInputRejectReason::ValidationRejected);
    EXPECT_FALSE(result.tube);
    EXPECT_EQ(calls, reject_body ? 0U : 4U);
  }
  num::CornerImage image; num::CornerProbe probe{}; num::Box body{}, vertices{};
  EXPECT_THROW(num::centered_step(body, num::I(-3), p, .005, true,
    &probe, nullptr, &image), std::runtime_error);
  probe.origin_yaw = NAN;
  EXPECT_THROW(num::centered_step(body, num::I(-3), p, .005, true,
    &probe, &vertices, &image), std::runtime_error);
  probe.origin_yaw = 0; vertices[0] = {1, -1};
  EXPECT_THROW(num::centered_step(body, num::I(-3), p, .005, true,
    &probe, &vertices, &image), std::runtime_error);
}

TEST(MpccVehicleModel, CornerCellSeparationRejectsContactTangencyAndInvalidGeometry)
{
  namespace num = vehicle::numerical;
  namespace rec = multi_purpose_mpc_ros::recovery_footprint;
  rec::OccupancyGrid grid;
  grid.width = grid.height = 20; grid.resolution_m = .25;
  grid.cells.assign(400, rec::CellState::Unknown);
  const size_t cell = 10 * grid.width + 10;
  grid.y_axis = rec::YAxisConvention::RowZeroAtMaximumY;
  const auto c = grid.grid_to_world(10, 10); ASSERT_TRUE(c);
  for (auto state : {rec::CellState::Unknown, rec::CellState::Occupied}) {
    grid.cells[cell] = state;
    for (double offset : {0., .125, .125 + 1e-9}) {
      num::Box vertices;
      for (size_t k = 0; k < 8; k += 2) {
        vertices[k] = num::I(offset); vertices[k+1] = num::I(0);
      }
      EXPECT_FALSE(num::separating_cell_clearance(vertices, grid, cell, *c));
      vertices[0] = num::I(NAN);
      EXPECT_FALSE(num::separating_cell_clearance(vertices, grid, cell, *c));
    }
  }
  num::Box clear;
  for (size_t k = 0; k < 8; k += 2) {clear[k] = num::I(.125 + 1e-6); clear[k+1] = num::I(0);}
  EXPECT_TRUE(num::separating_cell_clearance(clear, grid, cell, *c));
  EXPECT_FALSE(num::separating_cell_clearance(clear, grid, 400, *c));
  EXPECT_FALSE(num::separating_cell_clearance(clear, grid, cell, {NAN, 0}));
  grid.resolution_m = 0;
  EXPECT_FALSE(num::separating_cell_clearance(clear, grid, cell, *c));
}
