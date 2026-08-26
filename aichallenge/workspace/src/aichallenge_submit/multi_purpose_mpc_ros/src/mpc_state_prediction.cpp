#include <multi_purpose_mpc_ros/mpc_state_prediction.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace multi_purpose_mpc_ros::mpc_state_prediction
{
namespace
{

constexpr double kYawRateEpsilon = 1.0e-6;

double wrap_to_pi(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

} // namespace

State2D predict_constant_turn_rate(
  State2D state,
  const double longitudinal_velocity_mps,
  const double yaw_rate_radps,
  const double prediction_delay_sec)
{
  if (!std::isfinite(state.x) || !std::isfinite(state.y) ||
    !std::isfinite(state.yaw) || !std::isfinite(longitudinal_velocity_mps) ||
    !std::isfinite(yaw_rate_radps) || !std::isfinite(prediction_delay_sec) ||
    prediction_delay_sec < 0.0)
  {
    throw std::invalid_argument("invalid MPC state prediction input");
  }
  if (prediction_delay_sec == 0.0) {
    return state;
  }

  const double predicted_yaw =
    state.yaw + yaw_rate_radps * prediction_delay_sec;
  if (std::abs(yaw_rate_radps) < kYawRateEpsilon) {
    const double travel = longitudinal_velocity_mps * prediction_delay_sec;
    state.x += travel * std::cos(state.yaw);
    state.y += travel * std::sin(state.yaw);
  } else {
    const double radius = longitudinal_velocity_mps / yaw_rate_radps;
    state.x += radius * (std::sin(predicted_yaw) - std::sin(state.yaw));
    state.y -= radius * (std::cos(predicted_yaw) - std::cos(state.yaw));
  }
  state.yaw = wrap_to_pi(predicted_yaw);
  return state;
}

std::optional<ResponseSteeringInference> infer_response_steering(
  const double longitudinal_velocity_mps,
  const double measured_yaw_rate_radps,
  const double measured_physical_steering_rad,
  const double wheelbase_m,
  const double yaw_response_gain,
  const double minimum_inversion_speed_mps,
  const double maximum_abs_steering_rad) noexcept
{
  if (
    !std::isfinite(longitudinal_velocity_mps) ||
    !std::isfinite(measured_yaw_rate_radps) ||
    !std::isfinite(measured_physical_steering_rad) ||
    !std::isfinite(wheelbase_m) || wheelbase_m <= 0.0 ||
    !std::isfinite(yaw_response_gain) || yaw_response_gain <= 0.0 ||
    !std::isfinite(minimum_inversion_speed_mps) ||
    minimum_inversion_speed_mps < 0.0 ||
    !std::isfinite(maximum_abs_steering_rad) ||
    maximum_abs_steering_rad <= 0.0 ||
    std::abs(measured_physical_steering_rad) > maximum_abs_steering_rad)
  {
    return std::nullopt;
  }
  if (std::abs(longitudinal_velocity_mps) < minimum_inversion_speed_mps) {
    return ResponseSteeringInference{
      measured_physical_steering_rad, measured_physical_steering_rad, false};
  }
  const double inferred = std::atan(
    measured_yaw_rate_radps * wheelbase_m /
    (yaw_response_gain * std::abs(longitudinal_velocity_mps)));
  if (!std::isfinite(inferred)) {
    return std::nullopt;
  }
  const double projected = std::clamp(
    inferred, -maximum_abs_steering_rad, maximum_abs_steering_rad);
  return ResponseSteeringInference{
    projected, inferred, projected != inferred};
}

YawResponsePrediction predict_yaw_response(
  State2D state,
  const double longitudinal_velocity_mps,
  double response_steering_rad,
  const double initial_physical_steering_rad,
  const double terminal_physical_steering_rad,
  const double wheelbase_m,
  const double yaw_response_gain,
  const double yaw_response_time_constant_sec,
  const double prediction_delay_sec)
{
  return predict_accelerating_yaw_response(
    state, longitudinal_velocity_mps, 0.0, response_steering_rad,
    initial_physical_steering_rad, terminal_physical_steering_rad,
    wheelbase_m, yaw_response_gain, yaw_response_time_constant_sec,
    prediction_delay_sec);
}

YawResponsePrediction predict_accelerating_yaw_response(
  State2D state,
  const double longitudinal_velocity_mps,
  const double longitudinal_acceleration_mps2,
  double response_steering_rad,
  const double initial_physical_steering_rad,
  const double terminal_physical_steering_rad,
  const double wheelbase_m,
  const double yaw_response_gain,
  const double yaw_response_time_constant_sec,
  const double prediction_delay_sec)
{
  const auto trajectory = predict_accelerating_yaw_response_trajectory(
    state, longitudinal_velocity_mps, longitudinal_acceleration_mps2,
    response_steering_rad, initial_physical_steering_rad,
    terminal_physical_steering_rad, wheelbase_m, yaw_response_gain,
    yaw_response_time_constant_sec, prediction_delay_sec);
  return trajectory.back().prediction;
}

std::vector<TimedYawResponsePrediction>
predict_accelerating_yaw_response_trajectory(
  State2D state,
  const double longitudinal_velocity_mps,
  const double longitudinal_acceleration_mps2,
  double response_steering_rad,
  const double initial_physical_steering_rad,
  const double terminal_physical_steering_rad,
  const double wheelbase_m,
  const double yaw_response_gain,
  const double yaw_response_time_constant_sec,
  const double prediction_delay_sec)
{
  constexpr double half_pi = 1.57079632679489661923;
  constexpr double maximum_step_sec = 0.005;
  if (
    !std::isfinite(state.x) || !std::isfinite(state.y) ||
    !std::isfinite(state.yaw) ||
    !std::isfinite(longitudinal_velocity_mps) ||
    longitudinal_velocity_mps < 0.0 ||
    !std::isfinite(longitudinal_acceleration_mps2) ||
    !std::isfinite(response_steering_rad) ||
    std::abs(response_steering_rad) >= half_pi ||
    !std::isfinite(initial_physical_steering_rad) ||
    std::abs(initial_physical_steering_rad) >= half_pi ||
    !std::isfinite(terminal_physical_steering_rad) ||
    std::abs(terminal_physical_steering_rad) >= half_pi ||
    !std::isfinite(wheelbase_m) || wheelbase_m <= 0.0 ||
    !std::isfinite(yaw_response_gain) || yaw_response_gain <= 0.0 ||
    !std::isfinite(yaw_response_time_constant_sec) ||
    yaw_response_time_constant_sec <= 0.0 ||
    !std::isfinite(prediction_delay_sec) || prediction_delay_sec < 0.0)
  {
    throw std::invalid_argument("invalid yaw-response prediction input");
  }

  double elapsed_sec = 0.0;
  double velocity_mps = longitudinal_velocity_mps;
  double yaw_rate_radps = yaw_response_gain * velocity_mps *
    std::tan(response_steering_rad) / wheelbase_m;
  std::vector<TimedYawResponsePrediction> trajectory;
  trajectory.reserve(
    static_cast<std::size_t>(
      std::ceil(prediction_delay_sec / maximum_step_sec)) + 1U);
  trajectory.push_back(TimedYawResponsePrediction{
    0.0,
    YawResponsePrediction{
      state, velocity_mps, response_steering_rad, yaw_rate_radps}});
  while (elapsed_sec < prediction_delay_sec) {
    const double step_sec = std::min(
      maximum_step_sec, prediction_delay_sec - elapsed_sec);
    const double midpoint_fraction = prediction_delay_sec > 0.0 ?
      (elapsed_sec + 0.5 * step_sec) / prediction_delay_sec : 1.0;
    const double physical_steering_rad = initial_physical_steering_rad +
      midpoint_fraction *
      (terminal_physical_steering_rad - initial_physical_steering_rad);
    const double decay = std::exp(
      -step_sec / yaw_response_time_constant_sec);
    const double next_response_steering_rad =
      physical_steering_rad +
      (response_steering_rad - physical_steering_rad) * decay;
    const double midpoint_velocity_mps = std::max(
      0.0, velocity_mps + 0.5 * longitudinal_acceleration_mps2 * step_sec);
    const double next_velocity_mps = std::max(
      0.0, velocity_mps + longitudinal_acceleration_mps2 * step_sec);
    const double next_yaw_rate_radps = yaw_response_gain *
      next_velocity_mps * std::tan(next_response_steering_rad) /
      wheelbase_m;
    const double average_yaw_rate_radps =
      0.5 * (yaw_rate_radps + next_yaw_rate_radps);
    const double midpoint_yaw = state.yaw +
      0.5 * average_yaw_rate_radps * step_sec;
    state.x += midpoint_velocity_mps * std::cos(midpoint_yaw) * step_sec;
    state.y += midpoint_velocity_mps * std::sin(midpoint_yaw) * step_sec;
    state.yaw = wrap_to_pi(
      state.yaw + average_yaw_rate_radps * step_sec);
    response_steering_rad = next_response_steering_rad;
    velocity_mps = next_velocity_mps;
    yaw_rate_radps = next_yaw_rate_radps;
    elapsed_sec += step_sec;
    trajectory.push_back(TimedYawResponsePrediction{
      elapsed_sec,
      YawResponsePrediction{
        state, velocity_mps, response_steering_rad, yaw_rate_radps}});
  }
  return trajectory;
}

} // namespace multi_purpose_mpc_ros::mpc_state_prediction
