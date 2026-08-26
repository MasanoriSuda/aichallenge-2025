#pragma once

#include <optional>
#include <vector>

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

struct YawResponsePrediction
{
  State2D state;
  double longitudinal_velocity_mps{};
  double response_steering_rad{};
  double yaw_rate_radps{};
};

struct TimedYawResponsePrediction
{
  double elapsed_sec{};
  YawResponsePrediction prediction;
};

struct ResponseSteeringInference
{
  double steering_rad{};
  double unconstrained_steering_rad{};
  bool projected_to_model_envelope{false};
};

/// Infer the internal yaw-response steering state from the measured yaw rate.
/// At very low speed yaw-rate inversion is ill-conditioned, so the measured
/// physical steering is returned instead. A finite yaw observation outside
/// the reduced model envelope is projected to that envelope: it is model
/// mismatch evidence, not missing state evidence, and must not revoke normal
/// control authority. Invalid sensor or contract inputs are still rejected.
std::optional<ResponseSteeringInference> infer_response_steering(
  double longitudinal_velocity_mps,
  double measured_yaw_rate_radps,
  double measured_physical_steering_rad,
  double wheelbase_m,
  double yaw_response_gain,
  double minimum_inversion_speed_mps,
  double maximum_abs_steering_rad) noexcept;

/// Project the execution origin with the identified vehicle yaw dynamics.
/// The physical steering target is linearly interpolated between the observed
/// angle and the already committed angle over the transport delay. The
/// response-steering state is the angle whose reduced kinematic curvature
/// produces the observed yaw rate.
YawResponsePrediction predict_yaw_response(
  State2D state,
  double longitudinal_velocity_mps,
  double initial_response_steering_rad,
  double initial_physical_steering_rad,
  double terminal_physical_steering_rad,
  double wheelbase_m,
  double yaw_response_gain,
  double yaw_response_time_constant_sec,
  double prediction_delay_sec);

/// Project pose, speed and yaw response to one common execution origin.
/// The acceleration is measured vehicle response, not a newly optimized
/// command.  Keeping these states on one time base prevents a future pose
/// from being paired with an observation-time speed at asynchronous MPCC
/// admission.
YawResponsePrediction predict_accelerating_yaw_response(
  State2D state,
  double longitudinal_velocity_mps,
  double longitudinal_acceleration_mps2,
  double initial_response_steering_rad,
  double initial_physical_steering_rad,
  double terminal_physical_steering_rad,
  double wheelbase_m,
  double yaw_response_gain,
  double yaw_response_time_constant_sec,
  double prediction_delay_sec);

/// Return the complete measured-to-control-origin trajectory used by the
/// latency compensator.  Consumers which certify the swept physical path must
/// use these samples rather than re-integrating the terminal yaw rate with a
/// second motion model.
std::vector<TimedYawResponsePrediction>
predict_accelerating_yaw_response_trajectory(
  State2D state,
  double longitudinal_velocity_mps,
  double longitudinal_acceleration_mps2,
  double initial_response_steering_rad,
  double initial_physical_steering_rad,
  double terminal_physical_steering_rad,
  double wheelbase_m,
  double yaw_response_gain,
  double yaw_response_time_constant_sec,
  double prediction_delay_sec);

} // namespace multi_purpose_mpc_ros::mpc_state_prediction
