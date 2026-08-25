#ifndef MULTI_PURPOSE_MPC_ROS__MPCC_STEERING_STATE_CONTRACT_HPP_
#define MULTI_PURPOSE_MPC_ROS__MPCC_STEERING_STATE_CONTRACT_HPP_

#include <optional>

namespace multi_purpose_mpc_ros::mpcc_steering_state_contract
{

enum class Reason
{
  Available,
  InvalidMeasurement,
  InvalidTiming,
  StaleObservation,
  InvalidLimits,
  Count,
};

const char * to_string(Reason reason) noexcept;

struct Request
{
  double measured_steering_rad{};
  double measured_steering_rate_radps{};
  double observation_age_sec{};
  double prediction_delay_sec{};
  double maximum_observation_age_sec{};
  double maximum_abs_steering_rad{};
  double maximum_abs_steering_rate_radps{};
};

struct PhysicalState
{
  double measured_steering_rad{};
  double measured_steering_rate_radps{};
  double bounded_steering_rate_radps{};
  double observation_age_sec{};
  double projection_duration_sec{};
  double prediction_origin_steering_rad{};
  bool measured_rate_outside_model{false};
};

struct Result
{
  Reason reason{Reason::InvalidMeasurement};
  std::optional<PhysicalState> state;
};

/// Project one measured steering report onto the same latency-compensated
/// control origin used by the six-state pose.  The desired steering command is
/// deliberately absent from this request: it is not a physical observation.
Result resolve(const Request & request) noexcept;

}  // namespace multi_purpose_mpc_ros::mpcc_steering_state_contract

#endif  // MULTI_PURPOSE_MPC_ROS__MPCC_STEERING_STATE_CONTRACT_HPP_
