#ifndef MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_HPP_
#define MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_HPP_

#include <Eigen/Dense>

#include <optional>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved
{

inline constexpr int kStateDimension = 6;
inline constexpr int kInputDimension = 3;
inline constexpr int kLateralIndex = 0;
inline constexpr int kLagIndex = 1;
inline constexpr int kHeadingIndex = 2;
inline constexpr int kVelocityIndex = 3;
inline constexpr int kProgressIndex = 4;
inline constexpr int kSteeringIndex = 5;
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
  double reference_acceleration_mps2{};
  double reference_steering_rate_radps{};
  double reference_virtual_progress_speed_mps{};
  double reference_path_curvature_radpm{};
  double wheelbase_m{};
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
/// steering-rate input and whose steering angle is part of the state.
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

/// Resolve an intermediate actuator sample from one certified constant-rate
/// stage. Invalid rate, time, or steering bounds are rejected, never clamped.
std::optional<ActuationSample> sample_actuation(
  const ActuationSampleRequest & request) noexcept;

}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved

#endif  // MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_HPP_
