#ifndef MULTI_PURPOSE_MPC_ROS__MPCC_STEERING_STATE_CONTRACT_HPP_
#define MULTI_PURPOSE_MPC_ROS__MPCC_STEERING_STATE_CONTRACT_HPP_

#include <optional>

namespace multi_purpose_mpc_ros::mpcc_steering_state_contract
{

enum class Reason
{
  Available,
  CommittedInputUnavailable,
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
  std::optional<double> committed_steering_rad;
  double observation_age_sec{};
  double prediction_delay_sec{};
  double maximum_observation_age_sec{};
  double maximum_abs_steering_rad{};
  double maximum_abs_steering_rate_radps{};
  /// Age of the steering command which is currently committed to the
  /// actuator.  It may be newer than the steering observation and therefore
  /// cannot be applied retrospectively over the full observation age.
  double committed_command_age_sec{};
  /// Age on the ROS control-time axis.  Execution-artifact cursors use this
  /// axis, whereas committed_command_age_sec above belongs to the steady
  /// receipt-time axis used for observation projection.
  double committed_command_control_age_sec{};
};

struct PhysicalState
{
  double measured_steering_rad{};
  double committed_steering_rad{};
  double observation_age_sec{};
  /// Actual elapsed time since committed_steering_rad was published.  This
  /// is the causal command-to-command reachability duration; it is not the
  /// nominal controller period.
  double committed_command_age_sec{};
  double committed_command_control_age_sec{};
  double committed_command_projection_duration_sec{};
  double current_time_steering_rad{};
  double prediction_delay_sec{};
  double projection_duration_sec{};
  double maximum_reachable_step_rad{};
  double prediction_origin_steering_rad{};
  bool committed_command_reached{false};
};

struct Result
{
  Reason reason{Reason::InvalidMeasurement};
  std::optional<PhysicalState> state;
};

/// Project one measured steering report onto the same latency-compensated
/// control origin used by the six-state pose.  The committed command is the
/// zero-order-held actuator input already published during the latency prefix;
/// it is never substituted for the measured physical state.
Result resolve(const Request & request) noexcept;

}  // namespace multi_purpose_mpc_ros::mpcc_steering_state_contract

#endif  // MULTI_PURPOSE_MPC_ROS__MPCC_STEERING_STATE_CONTRACT_HPP_
