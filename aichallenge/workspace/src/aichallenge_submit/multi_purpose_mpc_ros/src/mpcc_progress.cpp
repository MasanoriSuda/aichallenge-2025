#include <multi_purpose_mpc_ros/mpcc_progress.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace multi_purpose_mpc_ros::mpcc_progress
{
namespace
{

bool finite_config(const Config & config) noexcept
{
  return
    std::isfinite(config.minimum_reference_speed_mps) &&
    config.minimum_reference_speed_mps > 0.0 &&
    std::isfinite(config.minimum_frenet_denominator) &&
    config.minimum_frenet_denominator > 0.0 &&
    std::isfinite(config.minimum_stage_dt_sec) &&
    config.minimum_stage_dt_sec > 0.0 &&
    std::isfinite(config.maximum_stage_dt_sec) &&
    config.maximum_stage_dt_sec >= config.minimum_stage_dt_sec &&
    std::isfinite(config.trust_region_backward_m) &&
    config.trust_region_backward_m >= 0.0 &&
    std::isfinite(config.trust_region_forward_m) &&
    config.trust_region_forward_m > 0.0 &&
    std::isfinite(config.lag_weight) && config.lag_weight > 0.0 &&
    std::isfinite(config.terminal_lag_weight) && config.terminal_lag_weight > 0.0 &&
    std::isfinite(config.progress_reward_weight) && config.progress_reward_weight >= 0.0 &&
    std::isfinite(config.terminal_progress_reward_weight) &&
    config.terminal_progress_reward_weight >= 0.0 &&
    config.rti_sqp_iterations >= 1 && config.rti_sqp_iterations <= 3 &&
    std::isfinite(config.rti_sqp_mixing) && config.rti_sqp_mixing > 0.0 &&
    config.rti_sqp_mixing <= 1.0;
}

}  // namespace

std::optional<StageDistanceResolution> resolve_stage_distances(
  const std::vector<double> & raw_stage_distance_m, const Config & config) noexcept
{
  if (!finite_config(config) || raw_stage_distance_m.empty()) {
    return std::nullopt;
  }
  const double minimum_stage_distance =
    config.minimum_reference_speed_mps * config.minimum_stage_dt_sec;
  if (!std::isfinite(minimum_stage_distance) || minimum_stage_distance <= 0.0) {
    return std::nullopt;
  }

  StageDistanceResolution result;
  result.distance_m.reserve(raw_stage_distance_m.size());
  result.minimum_stage_distance_m = minimum_stage_distance;
  for (const double raw_distance : raw_stage_distance_m) {
    if (!std::isfinite(raw_distance) || raw_distance < 0.0) {
      return std::nullopt;
    }
    if (raw_distance < minimum_stage_distance) {
      result.distance_m.push_back(minimum_stage_distance);
      ++result.normalized_stage_count;
    } else {
      result.distance_m.push_back(raw_distance);
    }
  }
  return result;
}

bool progress_origin_discontinuous(
  const double previous_progress_m, const double current_progress_m,
  const double maximum_continuous_step_m) noexcept
{
  if (
    !std::isfinite(previous_progress_m) || !std::isfinite(current_progress_m) ||
    !std::isfinite(maximum_continuous_step_m) || maximum_continuous_step_m <= 0.0)
  {
    return true;
  }
  return std::abs(current_progress_m - previous_progress_m) > maximum_continuous_step_m;
}

std::optional<Linearization> linearize_temporal_frenet(
  const LinearizationRequest & request) noexcept
{
  if (
    !finite_config(request.config) ||
    !std::isfinite(request.reference_lateral_m) ||
    !std::isfinite(request.reference_heading_rad) ||
    !std::isfinite(request.reference_progress_m) ||
    !std::isfinite(request.reference_speed_mps) ||
    !std::isfinite(request.reference_path_curvature_radpm) ||
    !std::isfinite(request.reference_input_curvature_radpm) ||
    !std::isfinite(request.stage_distance_m) || request.stage_distance_m <= 0.0)
  {
    return std::nullopt;
  }

  const double reference_speed = std::max(
    request.config.minimum_reference_speed_mps, request.reference_speed_mps);
  const double stage_dt = std::clamp(
    request.stage_distance_m / reference_speed,
    request.config.minimum_stage_dt_sec,
    request.config.maximum_stage_dt_sec);
  const double lateral = request.reference_lateral_m;
  const double heading = request.reference_heading_rad;
  const double path_curvature = request.reference_path_curvature_radpm;
  const double input_curvature = request.reference_input_curvature_radpm;
  const double denominator = 1.0 - path_curvature * lateral;
  if (
    !std::isfinite(denominator) ||
    denominator <= request.config.minimum_frenet_denominator)
  {
    return std::nullopt;
  }

  const double sin_heading = std::sin(heading);
  const double cos_heading = std::cos(heading);
  const double progress_rate = reference_speed * cos_heading / denominator;
  const double lateral_rate = reference_speed * sin_heading;
  const double heading_rate =
    reference_speed * input_curvature - path_curvature * progress_rate;

  const double ds_dey = reference_speed * cos_heading * path_curvature /
    (denominator * denominator);
  const double ds_depsi = -reference_speed * sin_heading / denominator;
  const double ds_dv = cos_heading / denominator;

  Eigen::Matrix3d continuous_state = Eigen::Matrix3d::Zero();
  continuous_state(0, 1) = reference_speed * cos_heading;
  continuous_state(1, 0) = -path_curvature * ds_dey;
  continuous_state(1, 1) = -path_curvature * ds_depsi;
  continuous_state(2, 0) = ds_dey;
  continuous_state(2, 1) = ds_depsi;

  Eigen::Matrix<double, 3, 2> continuous_input =
    Eigen::Matrix<double, 3, 2>::Zero();
  continuous_input(0, 0) = sin_heading;
  continuous_input(1, 0) = input_curvature - path_curvature * ds_dv;
  continuous_input(1, 1) = reference_speed;
  continuous_input(2, 0) = ds_dv;

  Linearization result;
  result.state_matrix = Eigen::Matrix3d::Identity() + stage_dt * continuous_state;
  result.input_matrix = stage_dt * continuous_input;
  result.stage_dt_sec = stage_dt;

  const Eigen::Vector3d reference_state(
    lateral, heading, request.reference_progress_m);
  const Eigen::Vector2d reference_input(reference_speed, input_curvature);
  const Eigen::Vector3d next_reference(
    lateral + stage_dt * lateral_rate,
    heading + stage_dt * heading_rate,
    request.reference_progress_m + stage_dt * progress_rate);
  result.equality_offset =
    result.state_matrix * reference_state +
    result.input_matrix * reference_input - next_reference;

  if (
    !result.state_matrix.allFinite() || !result.input_matrix.allFinite() ||
    !result.equality_offset.allFinite() || !std::isfinite(result.stage_dt_sec))
  {
    return std::nullopt;
  }
  return result;
}

std::optional<std::vector<double>> build_progress_reference(
  const double measured_progress_m,
  const std::vector<double> & stage_distance_m) noexcept
{
  if (!std::isfinite(measured_progress_m) || stage_distance_m.empty()) {
    return std::nullopt;
  }
  std::vector<double> result(stage_distance_m.size() + 1U, measured_progress_m);
  for (std::size_t stage = 0U; stage < stage_distance_m.size(); ++stage) {
    if (!std::isfinite(stage_distance_m[stage]) || stage_distance_m[stage] <= 0.0) {
      return std::nullopt;
    }
    result[stage + 1U] = result[stage] + stage_distance_m[stage];
    if (!std::isfinite(result[stage + 1U])) {
      return std::nullopt;
    }
  }
  return result;
}

std::optional<ProgressBounds> resolve_progress_bounds(
  const double measured_progress_m, const double reference_progress_m,
  const Config & config) noexcept
{
  if (
    !finite_config(config) || !std::isfinite(measured_progress_m) ||
    !std::isfinite(reference_progress_m))
  {
    return std::nullopt;
  }
  ProgressBounds result;
  result.lower_m = std::min(
    measured_progress_m,
    reference_progress_m - config.trust_region_backward_m);
  result.upper_m = reference_progress_m + config.trust_region_forward_m;
  if (
    !std::isfinite(result.lower_m) || !std::isfinite(result.upper_m) ||
    result.lower_m > result.upper_m)
  {
    return std::nullopt;
  }
  return result;
}

std::optional<ProgressCost> resolve_progress_cost(
  const double reference_progress_m, const bool terminal,
  const Config & config) noexcept
{
  if (!finite_config(config) || !std::isfinite(reference_progress_m)) {
    return std::nullopt;
  }
  const double lag_weight = terminal ? config.terminal_lag_weight : config.lag_weight;
  const double reward = terminal ?
    config.terminal_progress_reward_weight : config.progress_reward_weight;
  ProgressCost result;
  result.quadratic_weight = lag_weight;
  result.linear_coefficient = -lag_weight * reference_progress_m - reward;
  if (
    !std::isfinite(result.quadratic_weight) ||
    !std::isfinite(result.linear_coefficient))
  {
    return std::nullopt;
  }
  return result;
}

std::optional<Eigen::VectorXd> damp_rti_sqp_iterate(
  const Eigen::VectorXd & linearization_point,
  const Eigen::VectorXd & qp_solution, const double alpha) noexcept
{
  if (
    linearization_point.size() <= 0 ||
    linearization_point.size() != qp_solution.size() ||
    !linearization_point.allFinite() || !qp_solution.allFinite() ||
    !std::isfinite(alpha) || alpha <= 0.0 || alpha > 1.0)
  {
    return std::nullopt;
  }
  Eigen::VectorXd result =
    alpha * qp_solution + (1.0 - alpha) * linearization_point;
  if (!result.allFinite()) {
    return std::nullopt;
  }
  return result;
}

}  // namespace multi_purpose_mpc_ros::mpcc_progress
