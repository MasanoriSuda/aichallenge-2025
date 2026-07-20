#include <multi_purpose_mpc_ros/mpc_state_prediction.hpp>

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

} // namespace multi_purpose_mpc_ros::mpc_state_prediction
