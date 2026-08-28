#include "multi_purpose_mpc_ros/mpcc_steering_state_contract.hpp"

#include <algorithm>
#include <cmath>

namespace multi_purpose_mpc_ros::mpcc_steering_state_contract
{
namespace
{

double serialized_float32_limit(const double model_limit) noexcept
{
  // AckermannControlCommand and SteeringReport carry float32 angles. A
  // command clipped to a double model limit can round one float32 ULP outside
  // that same limit. Accept exactly the envelope reachable by serialization;
  // larger finite values remain physical-limit violations.
  return std::max(
    model_limit,
    std::abs(static_cast<double>(static_cast<float>(model_limit))));
}

}  // namespace

const char * to_string(const Reason reason) noexcept
{
  switch (reason) {
    case Reason::Available: return "available";
    case Reason::CommittedInputUnavailable: return "committed-input-unavailable";
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
  if (!request.committed_steering_rad.has_value()) {
    result.reason = Reason::CommittedInputUnavailable;
    return result;
  }
  if (
    !std::isfinite(request.measured_steering_rad) ||
    !std::isfinite(request.committed_steering_rad.value()))
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
    request.maximum_abs_steering_rate_radps < 0.0 ||
    !std::isfinite(request.committed_command_age_sec) ||
    request.committed_command_age_sec < 0.0 ||
    !std::isfinite(request.committed_command_control_age_sec) ||
    request.committed_command_control_age_sec < 0.0)
  {
    result.reason = Reason::InvalidLimits;
    return result;
  }
  const double serialized_limit =
    serialized_float32_limit(request.maximum_abs_steering_rad);
  if (
    std::abs(request.measured_steering_rad) > serialized_limit ||
    std::abs(request.committed_steering_rad.value()) > serialized_limit)
  {
    return result;
  }
  if (request.observation_age_sec > request.maximum_observation_age_sec) {
    result.reason = Reason::StaleObservation;
    return result;
  }

  PhysicalState state;
  state.measured_steering_serialization_projected =
    std::abs(request.measured_steering_rad) >
    request.maximum_abs_steering_rad;
  state.committed_steering_serialization_projected =
    std::abs(request.committed_steering_rad.value()) >
    request.maximum_abs_steering_rad;
  state.measured_steering_rad = std::clamp(
    request.measured_steering_rad, -request.maximum_abs_steering_rad,
    request.maximum_abs_steering_rad);
  state.committed_steering_rad = std::clamp(
    request.committed_steering_rad.value(),
    -request.maximum_abs_steering_rad,
    request.maximum_abs_steering_rad);
  state.observation_age_sec = request.observation_age_sec;
  state.committed_command_age_sec = request.committed_command_age_sec;
  state.committed_command_control_age_sec =
    request.committed_command_control_age_sec;
  state.prediction_delay_sec = request.prediction_delay_sec;
  state.committed_command_projection_duration_sec = std::min(
    request.observation_age_sec, request.committed_command_age_sec);
  state.projection_duration_sec =
    state.committed_command_projection_duration_sec +
    request.prediction_delay_sec;
  if (!std::isfinite(state.projection_duration_sec)) {
    result.reason = Reason::InvalidTiming;
    return result;
  }
  state.maximum_reachable_step_rad =
    request.maximum_abs_steering_rate_radps * state.projection_duration_sec;
  if (!std::isfinite(state.maximum_reachable_step_rad)) {
    result.reason = Reason::InvalidLimits;
    return result;
  }
  const double requested_step_rad =
    state.committed_steering_rad - state.measured_steering_rad;
  const double current_time_maximum_step_rad =
    request.maximum_abs_steering_rate_radps *
    state.committed_command_projection_duration_sec;
  const double current_time_reachable_step_rad = std::clamp(
    requested_step_rad,
    -current_time_maximum_step_rad,
    current_time_maximum_step_rad);
  state.current_time_steering_rad = std::clamp(
    state.measured_steering_rad + current_time_reachable_step_rad,
    -request.maximum_abs_steering_rad,
    request.maximum_abs_steering_rad);
  const double reachable_step_rad = std::clamp(
    requested_step_rad,
    -state.maximum_reachable_step_rad,
    state.maximum_reachable_step_rad);
  state.prediction_origin_steering_rad = std::clamp(
    state.measured_steering_rad + reachable_step_rad,
    -request.maximum_abs_steering_rad,
    request.maximum_abs_steering_rad);
  state.committed_command_reached =
    std::abs(requested_step_rad) <= state.maximum_reachable_step_rad;
  result.reason = Reason::Available;
  result.state = state;
  return result;
}

}  // namespace multi_purpose_mpc_ros::mpcc_steering_state_contract
