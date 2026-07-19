#include "multi_purpose_mpc_ros/recovery_mpc.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace multi_purpose_mpc_ros::recovery_mpc
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kMinimumFrenetDenominator = 0.10;
constexpr double kTieTolerance = 1e-12;

bool finite(const double value) noexcept
{
  return std::isfinite(value);
}

double wrap_to_pi(const double angle) noexcept
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

bool valid_direction(const Direction direction) noexcept
{
  return direction == Direction::Forward || direction == Direction::Reverse;
}

struct Candidate
{
  double lateral_error_m{};
  double heading_error_rad{};
  double previous_steering_rad{};
  double cost{};
  std::vector<double> steering_sequence_rad;
};

bool candidate_less(const Candidate & lhs, const Candidate & rhs) noexcept
{
  if (std::abs(lhs.cost - rhs.cost) > kTieTolerance) {
    return lhs.cost < rhs.cost;
  }
  const double lhs_first = lhs.steering_sequence_rad.empty() ?
    0.0 : lhs.steering_sequence_rad.front();
  const double rhs_first = rhs.steering_sequence_rad.empty() ?
    0.0 : rhs.steering_sequence_rad.front();
  if (std::abs(std::abs(lhs_first) - std::abs(rhs_first)) > kTieTolerance) {
    return std::abs(lhs_first) < std::abs(rhs_first);
  }
  return lhs_first < rhs_first;
}

std::vector<double> steering_targets(const Config & config)
{
  std::vector<double> samples;
  samples.reserve(config.steering_sample_count);
  if (config.steering_sample_count == 1U) {
    samples.push_back(0.0);
    return samples;
  }
  const double span = 2.0 * config.maximum_steering_angle_rad;
  const double divisor = static_cast<double>(config.steering_sample_count - 1U);
  for (std::size_t i = 0U; i < config.steering_sample_count; ++i) {
    samples.push_back(
      -config.maximum_steering_angle_rad + span * static_cast<double>(i) / divisor);
  }
  return samples;
}

}  // namespace

const char * to_string(const RejectReason reason) noexcept
{
  switch (reason) {
    case RejectReason::None:
      return "none";
    case RejectReason::InvalidConfig:
      return "invalid_config";
    case RejectReason::InvalidRequest:
      return "invalid_request";
    case RejectReason::NoFiniteCandidate:
      return "no_finite_candidate";
  }
  return "unknown";
}

bool config_is_valid(const Config & config) noexcept
{
  return
    config.horizon_steps > 0U && config.horizon_steps <= 32U &&
    config.steering_sample_count >= 3U && config.steering_sample_count <= 33U &&
    config.beam_width > 0U && config.beam_width <= 512U &&
    finite(config.travel_step_m) && config.travel_step_m > 0.0 &&
    finite(config.maximum_steering_angle_rad) &&
    config.maximum_steering_angle_rad > 0.0 &&
    config.maximum_steering_angle_rad < 0.5 * kPi - 1e-6 &&
    finite(config.maximum_steering_change_rad) &&
    config.maximum_steering_change_rad > 0.0 &&
    config.maximum_steering_change_rad <= 2.0 * config.maximum_steering_angle_rad &&
    finite(config.lateral_error_weight) && config.lateral_error_weight >= 0.0 &&
    finite(config.heading_error_weight) && config.heading_error_weight >= 0.0 &&
    finite(config.steering_weight) && config.steering_weight >= 0.0 &&
    finite(config.steering_change_weight) && config.steering_change_weight >= 0.0 &&
    finite(config.terminal_lateral_error_weight) &&
    config.terminal_lateral_error_weight >= 0.0 &&
    finite(config.terminal_heading_error_weight) &&
    config.terminal_heading_error_weight >= 0.0 &&
    (config.lateral_error_weight + config.heading_error_weight +
    config.terminal_lateral_error_weight + config.terminal_heading_error_weight) > 0.0;
}

Result plan(const Config & config, const Request & request)
{
  Result result;
  if (!config_is_valid(config)) {
    result.reason = RejectReason::InvalidConfig;
    return result;
  }
  if (
    !valid_direction(request.direction) || !finite(request.lateral_error_m) ||
    !finite(request.heading_error_rad) || !finite(request.reference_curvature_radpm) ||
    !finite(request.wheelbase_m) || request.wheelbase_m <= 0.0 ||
    !finite(request.initial_steering_tire_angle_rad))
  {
    result.reason = RejectReason::InvalidRequest;
    return result;
  }

  const double signed_step_m = request.direction == Direction::Forward ?
    config.travel_step_m : -config.travel_step_m;
  const double initial_steering_rad = std::clamp(
    request.initial_steering_tire_angle_rad,
    -config.maximum_steering_angle_rad,
    config.maximum_steering_angle_rad);
  std::vector<Candidate> beam{Candidate{
      request.lateral_error_m,
      wrap_to_pi(request.heading_error_rad),
      initial_steering_rad,
      0.0,
      {}}};
  const auto targets = steering_targets(config);

  for (std::size_t step = 0U; step < config.horizon_steps; ++step) {
    std::vector<Candidate> expanded;
    expanded.reserve(beam.size() * targets.size());
    for (const auto & parent : beam) {
      double previous_generated = std::numeric_limits<double>::quiet_NaN();
      for (const double target : targets) {
        const double steering = std::clamp(
          target,
          parent.previous_steering_rad - config.maximum_steering_change_rad,
          parent.previous_steering_rad + config.maximum_steering_change_rad);
        if (finite(previous_generated) && std::abs(steering - previous_generated) <= kTieTolerance) {
          continue;
        }
        previous_generated = steering;

        const double denominator =
          1.0 - request.reference_curvature_radpm * parent.lateral_error_m;
        if (!finite(denominator) || std::abs(denominator) < kMinimumFrenetDenominator) {
          continue;
        }
        Candidate child = parent;
        const double heading_rate_per_m =
          std::tan(steering) / request.wheelbase_m -
          request.reference_curvature_radpm * std::cos(parent.heading_error_rad) / denominator;
        child.lateral_error_m += signed_step_m * std::sin(parent.heading_error_rad);
        child.heading_error_rad = wrap_to_pi(
          parent.heading_error_rad + signed_step_m * heading_rate_per_m);
        child.previous_steering_rad = steering;
        child.steering_sequence_rad.push_back(steering);
        const double steering_change = steering - parent.previous_steering_rad;
        child.cost +=
          config.lateral_error_weight * child.lateral_error_m * child.lateral_error_m +
          config.heading_error_weight * child.heading_error_rad * child.heading_error_rad +
          config.steering_weight * steering * steering +
          config.steering_change_weight * steering_change * steering_change;
        if (
          finite(child.lateral_error_m) && finite(child.heading_error_rad) &&
          finite(child.cost))
        {
          expanded.push_back(std::move(child));
        }
      }
    }
    if (expanded.empty()) {
      result.reason = RejectReason::NoFiniteCandidate;
      return result;
    }
    std::sort(expanded.begin(), expanded.end(), candidate_less);
    if (expanded.size() > config.beam_width) {
      expanded.resize(config.beam_width);
    }
    beam = std::move(expanded);
  }

  for (auto & candidate : beam) {
    candidate.cost +=
      config.terminal_lateral_error_weight *
      candidate.lateral_error_m * candidate.lateral_error_m +
      config.terminal_heading_error_weight *
      candidate.heading_error_rad * candidate.heading_error_rad;
  }
  std::sort(beam.begin(), beam.end(), candidate_less);
  if (beam.empty() || beam.front().steering_sequence_rad.empty() || !finite(beam.front().cost)) {
    result.reason = RejectReason::NoFiniteCandidate;
    return result;
  }

  result.valid = true;
  result.reason = RejectReason::None;
  result.cost = beam.front().cost;
  result.first_steering_tire_angle_rad = beam.front().steering_sequence_rad.front();
  result.terminal_lateral_error_m = beam.front().lateral_error_m;
  result.terminal_heading_error_rad = beam.front().heading_error_rad;
  result.steering_sequence_rad = std::move(beam.front().steering_sequence_rad);
  return result;
}

}  // namespace multi_purpose_mpc_ros::recovery_mpc
