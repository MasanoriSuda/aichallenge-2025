#include "multi_purpose_mpc_ros/mpcc_steering_state_contract.hpp"

#include <algorithm>
#include <cmath>

namespace multi_purpose_mpc_ros::mpcc_steering_state_contract
{

const char * to_string(const Reason reason) noexcept
{
  switch (reason) {
    case Reason::Available: return "available";
    case Reason::InvalidMeasurement: return "invalid-measurement";
    case Reason::InvalidTiming: return "invalid-timing";
    case Reason::StaleObservation: return "stale-observation";
    case Reason::InvalidLimits: return "invalid-limits";
    case Reason::Count: break;
  }
  return "unknown";
}

Result resolve(const Request & request) noexcept
{
  Result result;
  if (
    !std::isfinite(request.measured_steering_rad) ||
    !std::isfinite(request.measured_steering_rate_radps))
  {
    return result;
  }
  if (
    !std::isfinite(request.observation_age_sec) ||
    request.observation_age_sec < 0.0 ||
    !std::isfinite(request.prediction_delay_sec) ||
    request.prediction_delay_sec < 0.0)
  {
    result.reason = Reason::InvalidTiming;
    return result;
  }
  if (
    !std::isfinite(request.maximum_observation_age_sec) ||
    request.maximum_observation_age_sec <= 0.0 ||
    !std::isfinite(request.maximum_abs_steering_rad) ||
    request.maximum_abs_steering_rad <= 0.0 ||
    !std::isfinite(request.maximum_abs_steering_rate_radps) ||
    request.maximum_abs_steering_rate_radps < 0.0)
  {
    result.reason = Reason::InvalidLimits;
    return result;
  }
  if (
    std::abs(request.measured_steering_rad) >
    request.maximum_abs_steering_rad)
  {
    return result;
  }
  if (request.observation_age_sec > request.maximum_observation_age_sec) {
    result.reason = Reason::StaleObservation;
    return result;
  }

  PhysicalState state;
  state.measured_steering_rad = request.measured_steering_rad;
  state.measured_steering_rate_radps = request.measured_steering_rate_radps;
  state.bounded_steering_rate_radps = std::clamp(
    request.measured_steering_rate_radps,
    -request.maximum_abs_steering_rate_radps,
    request.maximum_abs_steering_rate_radps);
  state.measured_rate_outside_model =
    state.bounded_steering_rate_radps !=
    state.measured_steering_rate_radps;
  state.observation_age_sec = request.observation_age_sec;
  state.projection_duration_sec =
    request.observation_age_sec + request.prediction_delay_sec;
  state.prediction_origin_steering_rad = std::clamp(
    state.measured_steering_rad +
    state.bounded_steering_rate_radps * state.projection_duration_sec,
    -request.maximum_abs_steering_rad,
    request.maximum_abs_steering_rad);
  result.reason = Reason::Available;
  result.state = state;
  return result;
}

}  // namespace multi_purpose_mpc_ros::mpcc_steering_state_contract
