#pragma once

#include <Eigen/Dense>

#include <optional>
#include <vector>

namespace multi_purpose_mpc_ros::mpcc_progress
{

struct Config
{
  double minimum_reference_speed_mps{0.5};
  double minimum_frenet_denominator{0.20};
  double minimum_stage_dt_sec{0.01};
  double maximum_stage_dt_sec{0.25};
  double trust_region_backward_m{12.0};
  double trust_region_forward_m{2.0};
  double lag_weight{5000.0};
  double terminal_lag_weight{2500.0};
  double progress_reward_weight{2000.0};
  double terminal_progress_reward_weight{5000.0};
};

struct LinearizationRequest
{
  double reference_lateral_m{};
  double reference_heading_rad{};
  double reference_progress_m{};
  double reference_speed_mps{};
  double reference_curvature_radpm{};
  double stage_distance_m{};
  Config config;
};

struct Linearization
{
  Eigen::Matrix3d state_matrix{Eigen::Matrix3d::Identity()};
  Eigen::Matrix<double, 3, 2> input_matrix{Eigen::Matrix<double, 3, 2>::Zero()};
  // QP equality convention:
  //   -x[k+1] + A*x[k] + B*u[k] = equality_offset
  Eigen::Vector3d equality_offset{Eigen::Vector3d::Zero()};
  double stage_dt_sec{};
};

/// Linearize the temporal Frenet kinematic model around one stage reference.
/// State is [e_y, e_psi, s], input is [v, kappa].
std::optional<Linearization> linearize_temporal_frenet(
  const LinearizationRequest & request) noexcept;

/// Build an unwrapped stage progress reference from the measured progress and
/// the existing ReferencePath segment distances. The result has N+1 states.
std::optional<std::vector<double>> build_progress_reference(
  double measured_progress_m, const std::vector<double> & stage_distance_m) noexcept;

struct ProgressBounds
{
  double lower_m{};
  double upper_m{};
};

/// Resolve an asymmetric trust region. The measured progress is always kept in
/// the lower envelope so a hard speed cap cannot make the QP infeasible merely
/// because the nominal progress horizon moved ahead.
std::optional<ProgressBounds> resolve_progress_bounds(
  double measured_progress_m, double reference_progress_m,
  const Config & config) noexcept;

struct ProgressCost
{
  double quadratic_weight{};
  double linear_coefficient{};
};

/// Cost convention is 0.5*w*s^2 + q*s. This represents
/// 0.5*w*(s-s_ref)^2 - reward*s up to an irrelevant constant.
std::optional<ProgressCost> resolve_progress_cost(
  double reference_progress_m, bool terminal, const Config & config) noexcept;

}  // namespace multi_purpose_mpc_ros::mpcc_progress
