#include "multi_purpose_mpc_ros/mpc_longitudinal_prediction.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace multi_purpose_mpc_ros::mpc_state_prediction
{

ControlObservationEpoch resolve_control_observation_epoch(
  const double ros_clock_sec, const std::optional<double> previous_ros_clock_sec,
  const double received_observation_sec, const double maximum_observation_age_sec) noexcept
{
  if (
    !std::isfinite(ros_clock_sec) ||
    (previous_ros_clock_sec && !std::isfinite(*previous_ros_clock_sec)) ||
    !std::isfinite(received_observation_sec) ||
    !std::isfinite(maximum_observation_age_sec) || maximum_observation_age_sec <= 0.0)
  {
    return {};
  }
  if (previous_ros_clock_sec && ros_clock_sec < *previous_ros_clock_sec) {
    return {false, true, false};
  }
  if (std::abs(received_observation_sec - ros_clock_sec) > maximum_observation_age_sec) {
    return {};
  }
  return {true, false, received_observation_sec > ros_clock_sec};
}

LongitudinalResponseObserver::LongitudinalResponseObserver(
  const double filter_gain, const double maximum_observation_gap_sec)
: filter_gain_(filter_gain), maximum_observation_gap_sec_(maximum_observation_gap_sec)
{
  if (
    !std::isfinite(filter_gain) || filter_gain < 0.0 || filter_gain >= 1.0 ||
    !std::isfinite(maximum_observation_gap_sec) || maximum_observation_gap_sec <= 0.0)
  {
    throw std::invalid_argument("invalid longitudinal response observer configuration");
  }
}

void LongitudinalResponseObserver::reset() noexcept
{
  commands_.clear();
  previous_observation_sec_.reset();
  previous_speed_mps_ = 0.0;
  observation_.reset();
}

bool LongitudinalResponseObserver::record_published_command(
  const double published_sec, const double control_origin_sec, const double acceleration_mps2)
{
  if (
    !std::isfinite(published_sec) || !std::isfinite(control_origin_sec) ||
    control_origin_sec < published_sec || !std::isfinite(acceleration_mps2))
  {
    return false;
  }
  if (!commands_.empty() && published_sec < commands_.back().published_sec) {
    reset();
  }
  const Command command{published_sec, control_origin_sec, acceleration_mps2};
  if (!commands_.empty() && published_sec == commands_.back().published_sec) {
    if (control_origin_sec != commands_.back().control_origin_sec) {
      return false;
    }
    commands_.back() = command;
  } else {
    if (!commands_.empty() && control_origin_sec < commands_.back().control_origin_sec) {
      return false;
    }
    commands_.push_back(command);
  }
  // Keep the active input preceding the oldest still-valid observation, plus
  // all later transitions. A stale observer cannot resurrect the older input.
  const double oldest_valid_sec = published_sec - maximum_observation_gap_sec_;
  auto keep = commands_.begin();
  while (keep + 1 != commands_.end() && (keep + 1)->control_origin_sec <= oldest_valid_sec) {
    ++keep;
  }
  commands_.erase(commands_.begin(), keep);
  return true;
}

double LongitudinalResponseObserver::command_at(const double time_sec) const noexcept
{
  double value = 0.0;
  for (const auto & command : commands_) {
    if (command.control_origin_sec > time_sec) {
      break;
    }
    value = command.acceleration_mps2;
  }
  return value;
}

std::vector<AccelerationInterval> LongitudinalResponseObserver::command_intervals(
  const double begin_sec, const double end_sec) const
{
  std::vector<AccelerationInterval> intervals;
  double time_sec = begin_sec;
  double acceleration_mps2 = command_at(begin_sec);
  for (const auto & command : commands_) {
    if (command.control_origin_sec <= begin_sec) {
      continue;
    }
    if (command.control_origin_sec >= end_sec) {
      break;
    }
    if (command.control_origin_sec > time_sec) {
      intervals.push_back({command.control_origin_sec - time_sec, acceleration_mps2});
    }
    time_sec = command.control_origin_sec;
    acceleration_mps2 = command.acceleration_mps2;
  }
  intervals.push_back({end_sec - time_sec, acceleration_mps2});
  return intervals;
}

std::optional<LongitudinalResponseObservation> LongitudinalResponseObserver::observe(
  const double source_sec, const double speed_mps)
{
  if (!std::isfinite(source_sec) || !std::isfinite(speed_mps) || speed_mps < 0.0) {
    reset();
    return std::nullopt;
  }
  if (previous_observation_sec_ && source_sec < *previous_observation_sec_) {
    reset();
  }
  if (
    !previous_observation_sec_ ||
    source_sec - *previous_observation_sec_ > maximum_observation_gap_sec_)
  {
    previous_observation_sec_ = source_sec;
    previous_speed_mps_ = speed_mps;
    observation_.reset();
    return std::nullopt;
  }
  const double duration_sec = source_sec - *previous_observation_sec_;
  if (duration_sec == 0.0) {
    previous_speed_mps_ = speed_mps;
    if (observation_) {
      observation_->speed_mps = speed_mps;
    }
    return observation_;
  }
  double integral_mps = 0.0;
  for (const auto & interval : command_intervals(*previous_observation_sec_, source_sec)) {
    integral_mps += interval.acceleration_mps2 * interval.duration_sec;
  }
  const double measured = (speed_mps - previous_speed_mps_) / duration_sec;
  const double commanded = integral_mps / duration_sec;
  const double previous_measured = observation_ ?
    observation_->filtered_measured_acceleration_mps2 : 0.0;
  const double previous_commanded = observation_ ?
    observation_->filtered_command_acceleration_mps2 : 0.0;
  observation_ = LongitudinalResponseObservation{
    source_sec, speed_mps,
    filter_gain_ * previous_measured + (1.0 - filter_gain_) * measured,
    filter_gain_ * previous_commanded + (1.0 - filter_gain_) * commanded};
  previous_observation_sec_ = source_sec;
  previous_speed_mps_ = speed_mps;
  if (
    !std::isfinite(observation_->filtered_measured_acceleration_mps2) ||
    !std::isfinite(observation_->filtered_command_acceleration_mps2))
  {
    reset();
    return std::nullopt;
  }
  return observation_;
}

std::optional<std::vector<AccelerationInterval>>
LongitudinalResponseObserver::prediction_intervals(
  const double now_sec, const double duration_sec) const
{
  if (
    !observation_ || !std::isfinite(now_sec) || !std::isfinite(duration_sec) ||
    duration_sec <= 0.0 || !std::isfinite(now_sec + duration_sec) ||
    now_sec < observation_->observed_sec ||
    now_sec - observation_->observed_sec > maximum_observation_gap_sec_ ||
    (!commands_.empty() && commands_.back().published_sec > now_sec))
  {
    return std::nullopt;
  }
  auto intervals = command_intervals(now_sec, now_sec + duration_sec);
  for (auto & interval : intervals) {
    interval.acceleration_mps2 += observation_->response_residual_mps2();
  }
  return intervals;
}

std::vector<TimedYawResponsePrediction> predict_piecewise_yaw_response_trajectory(
  State2D state, double speed_mps, double response_steering_rad,
  const double initial_physical_steering_rad, const double terminal_physical_steering_rad,
  const double wheelbase_m, const double yaw_response_gain,
  const double yaw_response_time_constant_sec,
  const std::vector<AccelerationInterval> & intervals)
{
  double duration_sec = 0.0;
  for (const auto & interval : intervals) {
    if (
      !std::isfinite(interval.duration_sec) || interval.duration_sec <= 0.0 ||
      !std::isfinite(interval.acceleration_mps2))
    {
      throw std::invalid_argument("invalid committed acceleration interval");
    }
    duration_sec += interval.duration_sec;
  }
  if (!std::isfinite(duration_sec) || duration_sec <= 0.0) {
    throw std::invalid_argument("empty committed acceleration trajectory");
  }
  std::vector<TimedYawResponsePrediction> trajectory;
  double elapsed_sec = 0.0;
  for (const auto & interval : intervals) {
    const double next_sec = elapsed_sec + interval.duration_sec;
    const double steering_start = initial_physical_steering_rad +
      (terminal_physical_steering_rad - initial_physical_steering_rad) * elapsed_sec / duration_sec;
    const double steering_end = initial_physical_steering_rad +
      (terminal_physical_steering_rad - initial_physical_steering_rad) * next_sec / duration_sec;
    const auto part = predict_accelerating_yaw_response_trajectory(
      state, speed_mps, interval.acceleration_mps2, response_steering_rad,
      steering_start, steering_end, wheelbase_m, yaw_response_gain,
      yaw_response_time_constant_sec, interval.duration_sec);
    for (std::size_t i = trajectory.empty() ? 0U : 1U; i < part.size(); ++i) {
      trajectory.push_back({elapsed_sec + part[i].elapsed_sec, part[i].prediction});
    }
    state = part.back().prediction.state;
    speed_mps = part.back().prediction.longitudinal_velocity_mps;
    response_steering_rad = part.back().prediction.response_steering_rad;
    elapsed_sec = next_sec;
  }
  return trajectory;
}

}  // namespace multi_purpose_mpc_ros::mpc_state_prediction
