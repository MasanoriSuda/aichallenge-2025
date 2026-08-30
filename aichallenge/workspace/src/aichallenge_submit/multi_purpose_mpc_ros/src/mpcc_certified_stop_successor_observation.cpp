#include "multi_purpose_mpc_ros/mpcc_certified_stop_successor_observation.hpp"

#include "multi_purpose_mpc_ros/mpcc_execution_contract.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>

namespace multi_purpose_mpc_ros::mpcc_certified_stop_successor_observation
{
namespace
{

double interpolate(const double lower, const double upper, const double alpha)
{
  return lower + alpha * (upper - lower);
}

double interpolate_angle(
  const double lower, const double upper, const double alpha)
{
  const double difference = std::atan2(
    std::sin(upper - lower), std::cos(upper - lower));
  return lower + alpha * difference;
}

double angle_error(const double observed, const double expected)
{
  return std::atan2(
    std::sin(observed - expected), std::cos(observed - expected));
}

}  // namespace

const char * to_string(const Reason reason) noexcept
{
  switch (reason) {
    case Reason::Sampled: return "sampled";
    case Reason::InvalidIdentity: return "invalid-identity";
    case Reason::InvalidShape: return "invalid-shape";
    case Reason::InvalidCurrentState: return "invalid-current-state";
    case Reason::TimeOutsideSuccessor: return "time-outside-successor";
  }
  return "unknown";
}

Result evaluate(
  const Published & published,
  const CurrentControlOrigin & current) noexcept
{
  Result result;
  const auto & evidence = published.evidence;
  const auto & elapsed = evidence.exact_trajectory.elapsed_time_sec;
  result.publication_age_sec =
    current.observation_origin_sec - published.publication_sec;
  if (
    evidence.source_decision_id == 0U ||
    current.decision_id <= evidence.source_decision_id ||
    evidence.solution_id == 0U || evidence.problem_fingerprint == 0U ||
    !mpcc_execution_contract::canonical_normal_intent_supported(
      evidence.source_intent) ||
    !std::isfinite(evidence.control_origin_sec))
  {
    return result;
  }
  if (
    elapsed.empty() ||
    evidence.exact_trajectory.velocity_mps.size() != elapsed.size() ||
    evidence.actuation_samples.size() != elapsed.size() ||
    evidence.world_prediction.first.size() != elapsed.size() ||
    evidence.world_prediction.second.size() != elapsed.size() ||
    evidence.world_yaw_rad.size() != elapsed.size() ||
    evidence.publisher_interval_sample_count == 0U ||
    evidence.publisher_interval_sample_count > elapsed.size())
  {
    result.reason = Reason::InvalidShape;
    return result;
  }
  if (
    !current.state_available || !std::isfinite(current.control_origin_sec) ||
    !std::isfinite(current.observation_origin_sec) ||
    !std::isfinite(current.x_m) || !std::isfinite(current.y_m) ||
    !std::isfinite(current.yaw_rad) || !std::isfinite(current.speed_mps) ||
    !std::isfinite(current.steering_rad))
  {
    result.reason = Reason::InvalidCurrentState;
    return result;
  }
  result.successor_elapsed_sec =
    current.control_origin_sec - evidence.control_origin_sec;
  result.publisher_boundary_sec =
    elapsed[evidence.publisher_interval_sample_count - 1U];
  if (
    !std::isfinite(result.successor_elapsed_sec) ||
    result.successor_elapsed_sec < elapsed.front() - 1e-9 ||
    result.successor_elapsed_sec > elapsed.back() + 1e-9)
  {
    result.reason = Reason::TimeOutsideSuccessor;
    return result;
  }

  const auto upper = std::lower_bound(
    elapsed.begin(), elapsed.end(), result.successor_elapsed_sec);
  result.upper_sample_index = upper == elapsed.end() ?
    elapsed.size() - 1U :
    static_cast<std::size_t>(std::distance(elapsed.begin(), upper));
  result.lower_sample_index = result.upper_sample_index == 0U ?
    0U : result.upper_sample_index - 1U;
  const double lower_time = elapsed[result.lower_sample_index];
  const double upper_time = elapsed[result.upper_sample_index];
  result.interpolation_alpha =
    result.upper_sample_index == result.lower_sample_index ||
    upper_time <= lower_time + 1e-12 ? 0.0 :
    std::clamp(
      (result.successor_elapsed_sec - lower_time) /
      (upper_time - lower_time), 0.0, 1.0);
  const auto scalar = [&result](const double lower, const double upper) {
      return interpolate(lower, upper, result.interpolation_alpha);
    };
  const auto angle = [&result](const double lower, const double upper) {
      return interpolate_angle(lower, upper, result.interpolation_alpha);
    };
  result.expected_x_m = scalar(
    evidence.world_prediction.first[result.lower_sample_index],
    evidence.world_prediction.first[result.upper_sample_index]);
  result.expected_y_m = scalar(
    evidence.world_prediction.second[result.lower_sample_index],
    evidence.world_prediction.second[result.upper_sample_index]);
  result.expected_yaw_rad = angle(
    evidence.world_yaw_rad[result.lower_sample_index],
    evidence.world_yaw_rad[result.upper_sample_index]);
  result.expected_speed_mps = scalar(
    evidence.exact_trajectory.velocity_mps[result.lower_sample_index],
    evidence.exact_trajectory.velocity_mps[result.upper_sample_index]);
  result.expected_steering_rad = angle(
    evidence.actuation_samples[result.lower_sample_index].end_steering_rad,
    evidence.actuation_samples[result.upper_sample_index].end_steering_rad);
  result.position_error_m = std::hypot(
    current.x_m - result.expected_x_m, current.y_m - result.expected_y_m);
  result.yaw_error_rad = std::abs(angle_error(
      current.yaw_rad, result.expected_yaw_rad));
  result.speed_error_mps = current.speed_mps - result.expected_speed_mps;
  result.steering_error_rad = angle_error(
    current.steering_rad, result.expected_steering_rad);
  result.reason = Reason::Sampled;
  return result;
}

}  // namespace multi_purpose_mpc_ros::mpcc_certified_stop_successor_observation

