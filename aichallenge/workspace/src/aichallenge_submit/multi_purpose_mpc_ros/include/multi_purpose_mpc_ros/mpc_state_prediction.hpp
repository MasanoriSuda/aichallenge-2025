#pragma once

namespace multi_purpose_mpc_ros::mpc_state_prediction
{

struct State2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

/// Predict a planar pose with a constant longitudinal velocity and yaw rate.
/// The input state is passed by value and is never mutated.
State2D predict_constant_turn_rate(
  State2D state,
  double longitudinal_velocity_mps,
  double yaw_rate_radps,
  double prediction_delay_sec);

} // namespace multi_purpose_mpc_ros::mpc_state_prediction
