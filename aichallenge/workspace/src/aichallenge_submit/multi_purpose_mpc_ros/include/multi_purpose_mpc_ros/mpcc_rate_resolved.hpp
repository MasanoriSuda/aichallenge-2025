#ifndef MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_HPP_
#define MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_HPP_

#include <Eigen/Dense>

#include <cstddef>
#include <optional>
#include <vector>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved
{

inline constexpr int kStateDimension = 7;
inline constexpr int kInputDimension = 3;
inline constexpr int kLateralIndex = 0;
inline constexpr int kLagIndex = 1;
inline constexpr int kHeadingIndex = 2;
inline constexpr int kVelocityIndex = 3;
inline constexpr int kProgressIndex = 4;
inline constexpr int kSteeringIndex = 5;
inline constexpr int kResponseSteeringIndex = 6;
inline constexpr int kAccelerationIndex = 0;
inline constexpr int kSteeringRateIndex = 1;
inline constexpr int kVirtualProgressSpeedIndex = 2;

struct LinearizationRequest
{
  double reference_lateral_m{};
  double reference_lag_m{};
  double reference_heading_rad{};
  double reference_velocity_mps{};
  double reference_progress_m{};
  double reference_steering_rad{};
  double reference_response_steering_rad{};
  double reference_acceleration_mps2{};
  double reference_steering_rate_radps{};
  double reference_virtual_progress_speed_mps{};
  double reference_path_curvature_radpm{};
  double wheelbase_m{};
  double yaw_response_gain{1.0};
  double yaw_response_time_constant_sec{};
  double stage_dt_sec{};
  double minimum_frenet_denominator{0.20};
  double minimum_stage_dt_sec{0.01};
  double maximum_stage_dt_sec{0.25};
};

struct Linearization
{
  Eigen::Matrix<double, kStateDimension, kStateDimension> state_matrix{
    Eigen::Matrix<double, kStateDimension, kStateDimension>::Identity()};
  Eigen::Matrix<double, kStateDimension, kInputDimension> input_matrix{
    Eigen::Matrix<double, kStateDimension, kInputDimension>::Zero()};
  Eigen::Matrix<double, kStateDimension, 1> equality_offset{
    Eigen::Matrix<double, kStateDimension, 1>::Zero()};
  double stage_dt_sec{};
};

/// Linearize the temporal Frenet bicycle model whose lateral actuator is a
/// steering-rate input. Commanded steering and the tire/yaw response steering
/// are distinct states; the latter follows the former through the identified
/// first-order vehicle yaw response.
std::optional<Linearization> linearize_temporal_frenet(
  const LinearizationRequest & request) noexcept;

struct ActuationSampleRequest
{
  double initial_steering_rad{};
  double steering_rate_radps{};
  double elapsed_sec{};
  double stage_duration_sec{};
  double maximum_abs_steering_rad{};
  double maximum_abs_steering_rate_radps{};
  double wheelbase_m{};
};

struct ActuationSample
{
  double steering_rad{};
  double curvature_radpm{};
};

enum class ActuationSampleReason
{
  Accepted,
  InitialSteeringNonfinite,
  SteeringRateNonfinite,
  ElapsedTimeInvalid,
  StageDurationInvalid,
  StageSequenceInvalid,
  PublicationAfterStageEnd,
  PublicationAfterHorizonEnd,
  SteeringLimitInvalid,
  SteeringRateLimitInvalid,
  WheelbaseInvalid,
  InitialSteeringLimitViolation,
  SteeringRateLimitViolation,
  TerminalSteeringLimitViolation,
  SampledSteeringLimitViolation,
  CurvatureNonfinite,
  SolverCertificateInvalid,
  Count,
};

const char * to_string(ActuationSampleReason reason) noexcept;

struct ActuationSampleEvaluation
{
  ActuationSampleReason reason{ActuationSampleReason::InitialSteeringNonfinite};
  std::optional<ActuationSample> sample;
  double terminal_steering_rad{};
  double sampled_steering_rad{};
};

/// Evaluate the exact fail-closed boundary and preserve its rejection
/// provenance. This function never clamps a solver result.
ActuationSampleEvaluation evaluate_actuation_sample(
  const ActuationSampleRequest & request) noexcept;

/// A publication request whose piecewise rate sequence and predicted stage
/// states have already passed the whole-QP physical row certificate. Semantic
/// current steering is the only integration origin.
struct CertifiedActuationSequenceSampleRequest
{
  double semantic_initial_steering_rad{};
  std::vector<double> certified_steering_rates_radps;
  std::vector<double> stage_durations_sec;
  double elapsed_sec{};
  double maximum_abs_steering_rad{};
  double wheelbase_m{};
  double maximum_normalized_constraint_violation{};
};

struct CertifiedActuationSequenceSampleEvaluation
{
  ActuationSampleReason reason{ActuationSampleReason::StageSequenceInvalid};
  std::optional<ActuationSample> sample;
  double sampled_steering_rad{};
  double certified_horizon_duration_sec{};
  std::size_t sampled_stage_index{};
  double sampled_stage_elapsed_sec{};
};

/// Integrate a certified piecewise-constant rate sequence to the exact
/// publisher time. QP-owned rate/state/dynamics constraints are not duplicated
/// here. Every crossed semantic steering boundary and the actual sample remain
/// fail-closed; no value is clamped.
CertifiedActuationSequenceSampleEvaluation
evaluate_certified_actuation_sequence_sample(
  const CertifiedActuationSequenceSampleRequest & request) noexcept;

/// Resolve an intermediate actuator sample from one certified constant-rate
/// stage. Invalid rate, time, or steering bounds are rejected, never clamped.
std::optional<ActuationSample> sample_actuation(
  const ActuationSampleRequest & request) noexcept;

}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved

#endif  // MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_HPP_
