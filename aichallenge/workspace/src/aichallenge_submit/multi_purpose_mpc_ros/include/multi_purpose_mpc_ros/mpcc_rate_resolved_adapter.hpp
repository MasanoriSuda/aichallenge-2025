#ifndef MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_ADAPTER_HPP_
#define MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_ADAPTER_HPP_

#include "multi_purpose_mpc_ros/mpcc_rate_resolved_problem.hpp"

#include <Eigen/Dense>

#include <limits>
#include <optional>
#include <vector>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_adapter
{

inline constexpr int kLegacyStateDimension = 5;
inline constexpr int kLegacyInputDimension = 3;
inline constexpr int kLegacyCurvatureIndex = 1;

struct StateStage
{
  Eigen::Matrix<double, kLegacyStateDimension, 1> reference{
    Eigen::Matrix<double, kLegacyStateDimension, 1>::Zero()};
  Eigen::Matrix<double, kLegacyStateDimension, 1> lower{
    Eigen::Matrix<double, kLegacyStateDimension, 1>::Zero()};
  Eigen::Matrix<double, kLegacyStateDimension, 1> upper{
    Eigen::Matrix<double, kLegacyStateDimension, 1>::Zero()};
  Eigen::Matrix<double, kLegacyStateDimension, 1> weight{
    Eigen::Matrix<double, kLegacyStateDimension, 1>::Zero()};
  Eigen::Matrix<double, kLegacyStateDimension, 1> linear_cost{
    Eigen::Matrix<double, kLegacyStateDimension, 1>::Zero()};
};

struct InputStage
{
  Eigen::Matrix<double, kLegacyInputDimension, 1> reference{
    Eigen::Matrix<double, kLegacyInputDimension, 1>::Zero()};
  Eigen::Matrix<double, kLegacyInputDimension, 1> lower{
    Eigen::Matrix<double, kLegacyInputDimension, 1>::Zero()};
  Eigen::Matrix<double, kLegacyInputDimension, 1> upper{
    Eigen::Matrix<double, kLegacyInputDimension, 1>::Zero()};
  Eigen::Matrix<double, kLegacyInputDimension, 1> weight{
    Eigen::Matrix<double, kLegacyInputDimension, 1>::Zero()};
  Eigen::Matrix<double, kLegacyInputDimension, 1> linear_cost{
    Eigen::Matrix<double, kLegacyInputDimension, 1>::Zero()};
  double path_curvature_radpm{};
  double stage_dt_sec{};
};

struct Request
{
  int horizon_steps{};
  Eigen::Matrix<double, kLegacyStateDimension, 1> initial_state{
    Eigen::Matrix<double, kLegacyStateDimension, 1>::Zero()};
  double current_steering_rad{};
  double wheelbase_m{};
  double maximum_abs_steering_rad{};
  double maximum_abs_steering_rate_radps{};
  double minimum_frenet_denominator{0.20};
  double minimum_stage_dt_sec{0.01};
  double maximum_stage_dt_sec{0.25};
  std::vector<StateStage> states;
  std::vector<InputStage> inputs;
  Eigen::Matrix<double, kLegacyInputDimension, 1> previous_input{
    Eigen::Matrix<double, kLegacyInputDimension, 1>::Zero()};
  Eigen::Matrix<double, kLegacyInputDimension, 1> input_delta_weight{
    Eigen::Matrix<double, kLegacyInputDimension, 1>::Zero()};
};

struct Result
{
  mpcc_rate_resolved_problem::AssemblyRequest problem;
  std::vector<double> steering_reference_rad;
  std::vector<double> steering_lower_rad;
  std::vector<double> steering_upper_rad;
  std::vector<double> curvature_to_steering_jacobian_radpm_per_rad;
  double first_steering_rate_physical_lower_radps{};
  double first_steering_rate_physical_upper_radps{};
  double first_steering_rate_solver_lower_radps{};
  double first_steering_rate_solver_upper_radps{};
  double first_steering_rate_certificate_margin_radps{};
};

enum class RejectReason
{
  None,
  InvalidRequest,
  InvalidStateStage,
  InvalidInputStage,
  InitialStateOutsideBounds,
  SteeringBoundsUnavailable,
  SteeringJacobianUnavailable,
  LinearizationUnavailable,
  AccelerationInsetUnavailable,
  SteeringRateInsetUnavailable,
};

struct BuildDiagnostic
{
  RejectReason reason{RejectReason::None};
  int stage{-1};
  int element{-1};
  double value{std::numeric_limits<double>::quiet_NaN()};
  double lower{std::numeric_limits<double>::quiet_NaN()};
  double upper{std::numeric_limits<double>::quiet_NaN()};
};

const char * to_string(RejectReason reason) noexcept;

std::optional<Result> build(
  const Request & request,
  const persistent_osqp::PhysicalConstraintTolerance & solver_tolerance,
  BuildDiagnostic * diagnostic = nullptr) noexcept;

}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_adapter

#endif  // MULTI_PURPOSE_MPC_ROS__MPCC_RATE_RESOLVED_ADAPTER_HPP_
