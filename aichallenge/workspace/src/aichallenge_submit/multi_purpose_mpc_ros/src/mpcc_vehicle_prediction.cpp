#include "multi_purpose_mpc_ros/mpcc_vehicle_prediction.hpp"

#include <algorithm>
#include <cmath>

namespace multi_purpose_mpc_ros::mpcc_vehicle_model
{

bool valid(const ObservationProvenance & v) noexcept
{
  if (!finite(v.initial.state) || !std::isfinite(v.initial.source_sec) ||
    !std::isfinite(v.now_sec) || !std::isfinite(v.control_origin_sec) ||
    v.initial.source_sec < 0 || v.now_sec < v.initial.source_sec ||
    v.control_origin_sec < v.now_sec || v.commands.empty()) return false;
  for (double stamp : {v.velocity_source_sec, v.yaw_rate_source_sec, v.tire_source_sec}) {
    if (!std::isfinite(stamp) || stamp < 0 || stamp > v.initial.source_sec) return false;
  }
  for (double delay : {v.acceleration_delay_sec, v.steering_delay_sec}) {
    if (!std::isfinite(delay) || delay < 0 ||
      v.commands.front().published_sec + delay > v.initial.source_sec) return false;
  }
  double previous = -1;
  for (const auto & command : v.commands) {
    if (!std::isfinite(command.published_sec) || command.published_sec < 0 ||
      command.published_sec <= previous || command.published_sec > v.now_sec ||
      !std::isfinite(command.wire_acceleration_mps2) ||
      !std::isfinite(command.wire_steering_rad)) return false;
    previous = command.published_sec;
  }
  return true;
}

std::optional<PublishedPrediction> predict_published_history(
  const TimedState & initial, const double now_sec, const double control_origin_sec,
  const std::vector<PublishedCommand> & commands, const Parameters & parameters,
  const double acceleration_delay_sec, const double steering_delay_sec) noexcept
{
  if (!valid(parameters) || !finite(initial.state) || commands.empty() ||
    !std::isfinite(initial.source_sec) || initial.source_sec < 0.0 ||
    !std::isfinite(now_sec) || now_sec < initial.source_sec ||
    !std::isfinite(control_origin_sec) || control_origin_sec < now_sec ||
    !std::isfinite(acceleration_delay_sec) || acceleration_delay_sec < 0.0 ||
    !std::isfinite(steering_delay_sec) || steering_delay_sec < 0.0 ||
    (control_origin_sec - initial.source_sec) / parameters.maximum_step_sec > 100000)
  {
    return std::nullopt;
  }
  std::vector<double> boundaries{initial.source_sec, now_sec, control_origin_sec};
  double previous = -1.0;
  for (const auto & command : commands) {
    if (!std::isfinite(command.published_sec) || command.published_sec < 0.0 ||
      command.published_sec <= previous || command.published_sec > now_sec ||
      !std::isfinite(command.wire_acceleration_mps2) ||
      !std::isfinite(command.wire_steering_rad)) return std::nullopt;
    previous = command.published_sec;
    for (const double delay : {acceleration_delay_sec, steering_delay_sec}) {
      const double applied = command.published_sec + delay;
      if (applied > initial.source_sec && applied < control_origin_sec) {
        boundaries.push_back(applied);
      }
    }
  }
  // Both channels require a recorded predecessor at the observation epoch.
  // Assuming an unobserved zero command would manufacture input coverage.
  if (commands.front().published_sec + acceleration_delay_sec > initial.source_sec ||
    commands.front().published_sec + steering_delay_sec > initial.source_sec)
  {
    return std::nullopt;
  }
  std::sort(boundaries.begin(), boundaries.end());
  boundaries.erase(std::unique(boundaries.begin(), boundaries.end()), boundaries.end());
  PublishedPrediction result;
  result.provenance = {initial, initial.source_sec, initial.source_sec, initial.source_sec,
    now_sec, control_origin_sec, acceleration_delay_sec, steering_delay_sec, commands};
  State state = initial.state;
  if (initial.source_sec == now_sec) {
    result.current = state;
    result.current_to_control.push_back({now_sec, state});
  }
  for (std::size_t i = 1; i < boundaries.size(); ++i) {
    const double begin = boundaries[i - 1], end = boundaries[i];
    double acceleration = commands.front().wire_acceleration_mps2;
    double steering = commands.front().wire_steering_rad;
    for (const auto & command : commands) {
      if (command.published_sec + acceleration_delay_sec <= begin) {
        acceleration = command.wire_acceleration_mps2;
      }
      if (command.published_sec + steering_delay_sec <= begin) {
        steering = command.wire_steering_rad;
      }
    }
    state.desired_steering_rad = steering / parameters.steering_wire_gain;
    const auto steps = integration_steps(end - begin, parameters.maximum_step_sec);
    const double dt = (end - begin) / static_cast<double>(steps);
    for (std::size_t step = 0; step < steps; ++step) {
      const auto next = advance(state, {acceleration, 0.0}, parameters, dt);
      if (!next) return std::nullopt;
      state = next->state;
      const double stamp = step + 1 == steps ? end : begin + (step + 1) * dt;
      if (stamp == now_sec) {
        result.current = state;
        result.current_to_control.push_back({stamp, state});
      } else if (stamp > now_sec) {
        result.current_to_control.push_back({stamp, state});
      }
    }
  }
  result.control_origin = state;
  return result.current_to_control.empty() ? std::nullopt :
         std::optional<PublishedPrediction>{std::move(result)};
}

}  // namespace multi_purpose_mpc_ros::mpcc_vehicle_model
