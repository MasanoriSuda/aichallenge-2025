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
  /// Finite float32 steering values are normalized back onto the double
  /// model envelope when their only excess comes from ROS serialization of
  /// the configured limit.
  bool measured_steering_serialization_projected{false};
  bool committed_steering_serialization_projected{false};
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
  /// Maximum physical-equivalent desired-angle motion between the measured
  /// report and the delayed execution origin.
  double maximum_reachable_step_rad{};
  /// Rate-bounded physical steering at the delayed execution origin.  The
  /// committed value is in physical-equivalent units, never AWSIM wire units.
  double prediction_origin_steering_rad{};
  bool committed_command_reached{false};
};

struct Result
{
  Reason reason{Reason::InvalidMeasurement};
  std::optional<PhysicalState> state;
};

/// Resolve the observed physical steering onto the latency-compensated
/// six-state origin.  The committed input is a physical-equivalent desired
/// angle whose calibrated wire command reaches the plant after the same delay;
/// wire steering must not be passed to this contract.
Result resolve(const Request & request) noexcept;

}  // namespace multi_purpose_mpc_ros::mpcc_steering_state_contract

#endif  // MULTI_PURPOSE_MPC_ROS__MPCC_STEERING_STATE_CONTRACT_HPP_
