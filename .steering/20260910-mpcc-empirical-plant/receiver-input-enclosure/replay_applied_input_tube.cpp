#define main preserved_certified_plan_driver_main
#include "../replay_certified_plan.cpp"
#undef main
#include "footprint_enclosure.hpp"
#include "multi_purpose_mpc_ros/mpcc_applied_input_prediction.hpp"

#include <chrono>
#include <random>

namespace vehicle = m::mpcc_vehicle_model;
namespace rec = m::recovery_footprint;
using Clock = std::chrono::steady_clock;

enclosure::Box box(const vehicle::BodyRanges & values)
{
  enclosure::Box result;
  for (size_t i = 0; i < values.size(); ++i) result[i] = {values[i].lower, values[i].upper};
  return result;
}

int main(int argc, char ** argv)
{
  if (argc != 3) return 2;
  std::string detail;
  const auto recorded = snap::load_recorded_interaction_snapshot(argv[1], &detail);
  if (!recorded) throw std::runtime_error("source snapshot unavailable: " + detail);
  const auto & source = recorded->source;
  const auto & world = source.replay_world.value();
  const auto & observation = source.request.observation_provenance.value();
  const auto & model = source.request.vehicle_model;
  const vehicle::InputApplicationProfile profile{
    "awsim-2025-empirical-receiver-age-250ms-diagnostic-v1", .25, .25, observation.steering_delay_sec};
  const vehicle::PublishedInputProgram program{source.publication_interval_sec,
    {{observation.now_sec, static_cast<float>(world.terminal_stop_minimum_acceleration_mps2),
      observation.commands.back().wire_steering_rad}}, true};
  const auto started = Clock::now();
  const auto prediction = vehicle::predict_applied_inputs_to_rest(observation, program, profile, model);
  const double prediction_ms = std::chrono::duration<double, std::milli>(Clock::now() - started).count();
  YAML::Node output;
  output["authority"] = false;
  output["meaning"] = "Shared library input-program numerical prediction, original source and sealed world; conditional empirical250ms channels, original mechanical delay; no execution certificate or transport/model-error guarantee.";
  output["decision"] = source.identity.source_context.decision_id;
  output["source_sequence"] = source.identity.sequence;
  output["interaction_fingerprint"] = recorded->interaction_fingerprint;
  output["prediction_reason"] = static_cast<int>(prediction.reason);
  output["prediction_ms"] = prediction_ms;
  output["tube_present"] = prediction.tube.has_value();
  if (prediction.tube) {
    const auto & tube = *prediction.tube;
    output["context_fingerprint"] = tube.context_fingerprint;
    output["rest_sec"] = tube.rest_sec;
    output["duration_from_publication_sec"] = tube.rest_sec - observation.now_sec;
    output["maximum_body_partitions"] = tube.maximum_body_partitions;
    output["source_to_rest_samples"] = tube.source_to_rest.size();
    const auto wall_footprint = m::mpcc_rate_resolved_physical_wall::resolve_clearance_footprint(
      world.physical_footprint, world.hard_wall_clearance_m);
    if (!wall_footprint || !source.wall_grid || world.observed_sec > observation.now_sec) {
      throw std::runtime_error("original world/footprint invalid");
    }
    const auto to_world = [&](const rec::Pose2D & local) {
        return rec::Pose2D{
          tube.coordinate_origin.x_m + std::cos(tube.coordinate_origin.yaw_rad) * local.x_m -
          std::sin(tube.coordinate_origin.yaw_rad) * local.y_m,
          tube.coordinate_origin.y_m + std::sin(tube.coordinate_origin.yaw_rad) * local.x_m +
          std::cos(tube.coordinate_origin.yaw_rad) * local.y_m,
          tube.coordinate_origin.yaw_rad + local.yaw_rad};
      };
    bool wall_clear = true, peer_clear = true;
    double minimum_peer = INFINITY, first_wall = NAN, first_peer = NAN, physical_ms = 0;
    const auto check = [&](const vehicle::BodyRanges & ranges, const double begin, const double end) {
        const auto before = Clock::now();
        const auto wall = enclosure::footprint(box(ranges), *wall_footprint);
        const auto cells = rec::sample_footprint(*source.wall_grid, wall.extents, to_world(wall.pose));
        if (!cells.valid || cells.out_of_map || !cells.contact_cells.empty()) {
          wall_clear = false; if (!std::isfinite(first_wall)) first_wall = begin;
        }
        const auto ego = enclosure::footprint(box(ranges), world.physical_footprint);
        for (const auto & obstacle : world.obstacles) {
          auto circle = obstacle.circle();
          const double t0 = begin - world.observed_sec, t1 = end - world.observed_sec;
          circle.radius_m = enclosure::up(circle.radius_m + enclosure::up(
            circle.maximum_speed(t0, t1) * enclosure::up((end - begin) / 2)));
          const auto clearance = rec::circle_obstacle_clearance_at_time(
            ego.extents, to_world(ego.pose), circle, (t0 + t1) / 2);
          if (!clearance) throw std::runtime_error("original peer invalid");
          minimum_peer = std::min(minimum_peer, *clearance);
          if (*clearance < 0) {peer_clear = false; if (!std::isfinite(first_peer)) first_peer = begin;}
        }
        physical_ms += std::chrono::duration<double, std::milli>(Clock::now() - before).count();
      };
    check(tube.publication_body, observation.now_sec, observation.now_sec);
    auto initial = observation.initial.state; initial.x_m = 0; initial.y_m = 0; initial.yaw_rad = 0;
    std::vector<vehicle::State> oracles(43, initial);
    std::mt19937 random(1031); std::uniform_real_distribution<double> unit(0, 1);
    size_t scalar_checks = 0;
    for (const auto & sample : tube.source_to_rest) {
      if (sample.begin_sec >= observation.now_sec) check(sample.swept_body, sample.begin_sec, sample.end_sec);
      for (size_t j = 0; j < oracles.size(); ++j) {
        auto & state = oracles[j];
        const double fraction = j == 0 ? 0 : (j == 1 ? 1 : unit(random));
        const double acceleration = sample.inputs.acceleration_mps2.lower +
          (sample.inputs.acceleration_mps2.upper - sample.inputs.acceleration_mps2.lower) * fraction;
        const double steering = sample.inputs.wire_steering_rad.lower +
          (sample.inputs.wire_steering_rad.upper - sample.inputs.wire_steering_rad.lower) * fraction;
        state.desired_steering_rad = steering / model.steering_wire_gain;
        const auto next = vehicle::advance(state, {acceleration, 0}, model, sample.duration_sec);
        if (!next) throw std::runtime_error("native oracle invalid");
        state = next->state; const auto values = enclosure::values(state);
        for (size_t i = 0; i < values.size(); ++i) {
          if (values[i] < sample.endpoint_body[i].lower || values[i] > sample.endpoint_body[i].upper) {
            throw std::runtime_error("native oracle outside shared-library tube");
          }
          ++scalar_checks;
        }
      }
    }
    for (const auto & state : oracles) {
      if (state.forward_velocity_mps != 0 || state.lateral_velocity_mps != 0 || state.yaw_rate_radps != 0) {
        throw std::runtime_error("native oracle not at full modeled rest");
      }
    }
    output["wall_clear"] = wall_clear; output["peer_clear"] = peer_clear;
    output["minimum_peer_clearance_m"] = minimum_peer;
    output["first_wall_reject_sec"] = first_wall; output["first_peer_reject_sec"] = first_peer;
    output["physical_ms"] = physical_ms; output["native_scalar_checks"] = scalar_checks;
    output["numerical_envelope_clear_to_rest"] = wall_clear && peer_clear;
  }
  YAML::Emitter emitter; emitter.SetDoublePrecision(17); emitter << output;
  std::ofstream(argv[2]) << emitter.c_str() << '\n'; std::cout << emitter.c_str() << '\n';
  return prediction.tube && output["numerical_envelope_clear_to_rest"].as<bool>() ? 0 : 1;
}
