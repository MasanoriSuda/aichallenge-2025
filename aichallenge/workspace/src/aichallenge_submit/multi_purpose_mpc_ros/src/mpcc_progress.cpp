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
    std::isfinite(config.extended_lag_state_bound_m) &&
    config.extended_lag_state_bound_m > 0.0 &&
    std::isfinite(config.extended_lateral_tracking_weight) &&
    config.extended_lateral_tracking_weight > 0.0 &&
    std::isfinite(config.extended_heading_tracking_weight) &&
    config.extended_heading_tracking_weight > 0.0 &&
    std::isfinite(config.extended_terminal_lateral_tracking_weight) &&
    config.extended_terminal_lateral_tracking_weight > 0.0 &&
    std::isfinite(config.extended_terminal_heading_tracking_weight) &&
    config.extended_terminal_heading_tracking_weight > 0.0 &&
    std::isfinite(config.extended_wall_tracking_reference_reserve_m) &&
    config.extended_wall_tracking_reference_reserve_m >= 0.0 &&
    std::isfinite(config.extended_wall_tracking_minimum_weight_scale) &&
    config.extended_wall_tracking_minimum_weight_scale > 0.0 &&
    config.extended_wall_tracking_minimum_weight_scale <= 1.0 &&
    std::isfinite(config.extended_lag_weight) && config.extended_lag_weight > 0.0 &&
    std::isfinite(config.extended_terminal_lag_weight) &&
    config.extended_terminal_lag_weight > 0.0 &&
    std::isfinite(config.extended_progress_tracking_weight) &&
    config.extended_progress_tracking_weight > 0.0 &&
    std::isfinite(config.extended_terminal_progress_tracking_weight) &&
    config.extended_terminal_progress_tracking_weight > 0.0 &&
    std::isfinite(config.extended_progress_reward_weight) &&
    config.extended_progress_reward_weight >= 0.0 &&
    std::isfinite(config.extended_terminal_progress_reward_weight) &&
    config.extended_terminal_progress_reward_weight >= 0.0 &&
    std::isfinite(config.stage_velocity_weight) && config.stage_velocity_weight > 0.0 &&
    std::isfinite(config.committed_stage_velocity_weight) &&
    config.committed_stage_velocity_weight >= config.stage_velocity_weight &&
    std::isfinite(config.terminal_velocity_weight) &&
    config.terminal_velocity_weight >= 0.0 &&
    std::isfinite(config.committed_terminal_velocity_weight) &&
    config.committed_terminal_velocity_weight >= config.terminal_velocity_weight &&
    std::isfinite(config.acceleration_weight) && config.acceleration_weight > 0.0 &&
    std::isfinite(config.virtual_progress_weight) && config.virtual_progress_weight > 0.0 &&
    std::isfinite(config.acceleration_rate_weight) &&
    config.acceleration_rate_weight >= 0.0 &&
    std::isfinite(config.curvature_rate_weight) && config.curvature_rate_weight >= 0.0 &&
    std::isfinite(config.virtual_progress_rate_weight) &&
    config.virtual_progress_rate_weight >= 0.0 &&
    config.rti_sqp_iterations >= 1 && config.rti_sqp_iterations <= 3 &&
    std::isfinite(config.rti_sqp_mixing) && config.rti_sqp_mixing > 0.0 &&
    config.rti_sqp_mixing <= 1.0 &&
    std::isfinite(config.refinement_minimum_bound_reserve_m) &&
    config.refinement_minimum_bound_reserve_m >= 0.0 &&
    std::isfinite(config.refinement_lateral_defect_m) &&
    config.refinement_lateral_defect_m >= 0.0 &&
    std::isfinite(config.refinement_heading_defect_rad) &&
    config.refinement_heading_defect_rad >= 0.0 &&
    std::isfinite(config.refinement_curvature_radpm) &&
    config.refinement_curvature_radpm >= 0.0 &&
    std::isfinite(config.refinement_start_deadline_ms) &&
    config.refinement_start_deadline_ms > 0.0 &&
    std::isfinite(config.refinement_cold_entry_skip_sec) &&
    config.refinement_cold_entry_skip_sec >= 0.0;
}

}  // namespace

ActivationResolution resolve_activation(const ActivationRequest & request) noexcept
{
  ActivationResolution resolution;
  if (request.overtake_execution_phase) {
    resolution.requested = true;
    resolution.source = ActivationSource::OvertakeExecution;
    return resolution;
  }
  if (request.dynamic_obstacle_escape_active) {
    resolution.requested = true;
    resolution.source = ActivationSource::DynamicObstacleEscape;
    return resolution;
  }
  resolution.source = ActivationSource::NormalIntent;
  return resolution;
}

const char * activation_source_name(const ActivationSource source) noexcept
{
  switch (source) {
    case ActivationSource::NormalIntent:
      return "normal-intent";
    case ActivationSource::OvertakeExecution:
      return "overtake-execution";
    case ActivationSource::DynamicObstacleEscape:
      return "dynamic-obstacle-escape";
  }
  return "unknown";
}

std::optional<FirstCurvatureReachabilityResolution>
resolve_first_curvature_reachability(
  const FirstCurvatureReachabilityRequest & request) noexcept
{
  constexpr double half_pi = 1.57079632679489661923;
  if (
    !std::isfinite(request.input_lower_radpm) ||
    !std::isfinite(request.input_upper_radpm) ||
    request.input_lower_radpm > request.input_upper_radpm ||
    !std::isfinite(request.previous_steering_rad) ||
    !std::isfinite(request.maximum_steering_step_rad) ||
    request.maximum_steering_step_rad < 0.0 ||
    !std::isfinite(request.wheelbase_m) || request.wheelbase_m <= 0.0)
  {
    return std::nullopt;
  }
  const double lower_steering =
    request.previous_steering_rad - request.maximum_steering_step_rad;
  const double upper_steering =
    request.previous_steering_rad + request.maximum_steering_step_rad;
  if (
    lower_steering <= -half_pi || upper_steering >= half_pi ||
    lower_steering > upper_steering)
  {
    return std::nullopt;
  }
  const double rate_lower = std::tan(lower_steering) / request.wheelbase_m;
  const double rate_upper = std::tan(upper_steering) / request.wheelbase_m;
  if (!std::isfinite(rate_lower) || !std::isfinite(rate_upper)) {
    return std::nullopt;
  }
  const double reachable_lower =
    std::max(request.input_lower_radpm, rate_lower);
  const double reachable_upper =
    std::min(request.input_upper_radpm, rate_upper);
  return FirstCurvatureReachabilityResolution{
    rate_lower, rate_upper, reachable_lower, reachable_upper,
    reachable_lower <= reachable_upper};
}

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

const char * reachable_horizon_reason_name(
  const ReachableHorizonReason reason) noexcept
{
  switch (reason) {
    case ReachableHorizonReason::InvalidInput: return "invalid-input";
    case ReachableHorizonReason::NoReachableStage: return "no-reachable-stage";
    case ReachableHorizonReason::ReachabilityLimited:
      return "reachability-limited";
    case ReachableHorizonReason::CompleteHorizon: return "complete-horizon";
  }
  return "unknown";
}

ReachableHorizonResolution resolve_reachable_temporal_horizon(
  const ReachableHorizonRequest & request) noexcept
{
  constexpr double kDistanceToleranceM = 1e-9;
  ReachableHorizonResolution result;
  if (
    !std::isfinite(request.initial_speed_mps) ||
    request.initial_speed_mps < 0.0 ||
    !std::isfinite(request.maximum_acceleration_mps2) ||
    request.maximum_acceleration_mps2 < 0.0 ||
    !std::isfinite(request.maximum_lag_m) || request.maximum_lag_m < 0.0 ||
    request.stage_distance_m.empty() ||
    request.stage_distance_m.size() != request.stage_dt_sec.size())
  {
    return result;
  }

  double elapsed_sec = 0.0;
  double reference_distance_m = 0.0;
  for (std::size_t stage = 0U; stage < request.stage_distance_m.size(); ++stage) {
    const double distance_m = request.stage_distance_m[stage];
    const double dt_sec = request.stage_dt_sec[stage];
    if (
      !std::isfinite(distance_m) || distance_m <= 0.0 ||
      !std::isfinite(dt_sec) || dt_sec <= 0.0)
    {
      return ReachableHorizonResolution{};
    }
    elapsed_sec += dt_sec;
    reference_distance_m += distance_m;
    const double reachable_distance_m =
      request.initial_speed_mps * elapsed_sec +
      0.5 * request.maximum_acceleration_mps2 * elapsed_sec * elapsed_sec +
      request.maximum_lag_m;
    if (
      !std::isfinite(elapsed_sec) || !std::isfinite(reference_distance_m) ||
      !std::isfinite(reachable_distance_m))
    {
      return ReachableHorizonResolution{};
    }
    if (reference_distance_m > reachable_distance_m + kDistanceToleranceM) {
      result.first_unreachable_stage = static_cast<int>(stage);
      break;
    }
    result.horizon_steps = static_cast<int>(stage + 1U);
    result.horizon_duration_sec = elapsed_sec;
    result.horizon_reference_distance_m = reference_distance_m;
    result.maximum_reachable_distance_m = reachable_distance_m;
  }

  if (result.horizon_steps == 0) {
    result.reason = ReachableHorizonReason::NoReachableStage;
    return result;
  }
  result.valid = true;
  result.reason =
    result.horizon_steps == static_cast<int>(request.stage_distance_m.size()) ?
    ReachableHorizonReason::CompleteHorizon :
    ReachableHorizonReason::ReachabilityLimited;
  return result;
}

std::optional<WallAwareTrackingReferenceResolution>
resolve_wall_aware_tracking_reference(
  const WallAwareTrackingReferenceRequest & request) noexcept
{
  if (
    !std::isfinite(request.reference_lateral_m) ||
    !std::isfinite(request.lower_bound_m) ||
    !std::isfinite(request.upper_bound_m) ||
    request.lower_bound_m > request.upper_bound_m ||
    !std::isfinite(request.preferred_reserve_m) ||
    request.preferred_reserve_m < 0.0 ||
    !std::isfinite(request.minimum_weight_scale) ||
    request.minimum_weight_scale <= 0.0 || request.minimum_weight_scale > 1.0)
  {
    return std::nullopt;
  }

  const double corridor_width = request.upper_bound_m - request.lower_bound_m;
  const double achievable_reserve = std::min(
    request.preferred_reserve_m, 0.5 * corridor_width);
  const double interior_lower = request.lower_bound_m + achievable_reserve;
  const double interior_upper = request.upper_bound_m - achievable_reserve;
  const double reference = interior_lower <= interior_upper ?
    std::clamp(request.reference_lateral_m, interior_lower, interior_upper) :
    0.5 * (request.lower_bound_m + request.upper_bound_m);
  const double actual_reserve = std::max(
    0.0,
    std::min(
      reference - request.lower_bound_m,
      request.upper_bound_m - reference));
  const double reserve_ratio = request.preferred_reserve_m > 1e-9 ?
    std::clamp(actual_reserve / request.preferred_reserve_m, 0.0, 1.0) : 1.0;
  const double weight_scale =
    request.minimum_weight_scale +
    (1.0 - request.minimum_weight_scale) * reserve_ratio;
  return WallAwareTrackingReferenceResolution{
    reference, weight_scale, actual_reserve,
    std::abs(reference - request.reference_lateral_m) > 1e-9};
}

std::optional<LateralTrackingTubeBoundsResolution>
resolve_lateral_tracking_tube_bounds(
  const LateralTrackingTubeBoundsRequest & request) noexcept
{
  if (
    !std::isfinite(request.physical_lower_m) ||
    !std::isfinite(request.physical_upper_m) ||
    !std::isfinite(request.required_reserve_m) ||
    request.required_reserve_m < 0.0 ||
    request.physical_upper_m < request.physical_lower_m)
  {
    return std::nullopt;
  }
  const double nominal_lower_m =
    request.physical_lower_m + request.required_reserve_m;
  const double nominal_upper_m =
    request.physical_upper_m - request.required_reserve_m;
  if (
    !std::isfinite(nominal_lower_m) || !std::isfinite(nominal_upper_m) ||
    nominal_upper_m < nominal_lower_m)
  {
    return std::nullopt;
  }
  return LateralTrackingTubeBoundsResolution{
    nominal_lower_m, nominal_upper_m, request.required_reserve_m};
}

LateralTrackingHorizonResolution resolve_lateral_tracking_horizon(
  const std::vector<double> & physical_lower_m,
  const std::vector<double> & physical_upper_m,
  const int configured_horizon_steps, const int minimum_prefix_steps,
  const double required_reserve_m) noexcept
{
  LateralTrackingHorizonResolution result;
  if (
    configured_horizon_steps <= 0 || minimum_prefix_steps <= 0 ||
    minimum_prefix_steps > configured_horizon_steps ||
    physical_lower_m.size() !=
    static_cast<std::size_t>(configured_horizon_steps + 1) ||
    physical_upper_m.size() != physical_lower_m.size() ||
    !std::isfinite(required_reserve_m) || required_reserve_m < 0.0)
  {
    return result;
  }
  if (!resolve_lateral_tracking_tube_bounds(
      LateralTrackingTubeBoundsRequest{
        physical_lower_m.front(), physical_upper_m.front(), 0.0}).has_value())
  {
    return result;
  }
  for (int state = 1; state <= configured_horizon_steps; ++state) {
    if (resolve_lateral_tracking_tube_bounds(
        LateralTrackingTubeBoundsRequest{
          physical_lower_m[static_cast<std::size_t>(state)],
          physical_upper_m[static_cast<std::size_t>(state)],
          required_reserve_m}).has_value())
    {
      continue;
    }
    result.first_unavailable_state = state;
    result.horizon_steps = state - 1;
    if (result.horizon_steps < minimum_prefix_steps) {
      result.reason = LateralTrackingHorizonReason::ImmediateInfeasible;
      return result;
    }
    result.valid = true;
    result.reason = LateralTrackingHorizonReason::BoundedPrefix;
    return result;
  }
  result.valid = true;
  result.horizon_steps = configured_horizon_steps;
  result.reason = LateralTrackingHorizonReason::CompleteHorizon;
  return result;
}

const char * lateral_tracking_horizon_reason_name(
  const LateralTrackingHorizonReason reason) noexcept
{
  switch (reason) {
    case LateralTrackingHorizonReason::InvalidInput:
      return "invalid-input";
    case LateralTrackingHorizonReason::CompleteHorizon:
      return "complete-horizon";
    case LateralTrackingHorizonReason::BoundedPrefix:
      return "bounded-prefix";
    case LateralTrackingHorizonReason::ImmediateInfeasible:
      return "immediate-infeasible";
  }
  return "unknown";
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

std::optional<ExtendedLinearization> linearize_extended_temporal_frenet(
  const ExtendedLinearizationRequest & request) noexcept
{
  if (
    !finite_config(request.config) ||
    !std::isfinite(request.reference_lateral_m) ||
    !std::isfinite(request.reference_lag_m) ||
    !std::isfinite(request.reference_heading_rad) ||
    !std::isfinite(request.reference_velocity_mps) ||
    request.reference_velocity_mps < 0.0 ||
    !std::isfinite(request.reference_progress_m) ||
    !std::isfinite(request.reference_acceleration_mps2) ||
    !std::isfinite(request.reference_path_curvature_radpm) ||
    !std::isfinite(request.reference_input_curvature_radpm) ||
    !std::isfinite(request.reference_virtual_progress_speed_mps) ||
    request.reference_virtual_progress_speed_mps < 0.0 ||
    !std::isfinite(request.stage_dt_sec) ||
    request.stage_dt_sec < request.config.minimum_stage_dt_sec ||
    request.stage_dt_sec > request.config.maximum_stage_dt_sec)
  {
    return std::nullopt;
  }

  const double reference_velocity = std::max(
    request.config.minimum_reference_speed_mps, request.reference_velocity_mps);
  const double stage_dt = request.stage_dt_sec;
  const double lateral = request.reference_lateral_m;
  const double heading = request.reference_heading_rad;
  const double path_curvature = request.reference_path_curvature_radpm;
  const double input_curvature = request.reference_input_curvature_radpm;
  const double virtual_progress_speed = request.reference_virtual_progress_speed_mps;
  const double denominator = 1.0 - path_curvature * lateral;
  if (
    !std::isfinite(denominator) ||
    denominator <= request.config.minimum_frenet_denominator)
  {
    return std::nullopt;
  }

  const double sin_heading = std::sin(heading);
  const double cos_heading = std::cos(heading);
  const double physical_progress_rate = reference_velocity * cos_heading / denominator;
  const double lateral_rate = reference_velocity * sin_heading;
  const double lag_rate = physical_progress_rate - virtual_progress_speed;
  const double heading_rate =
    reference_velocity * input_curvature - path_curvature * virtual_progress_speed;
  const double velocity_rate = request.reference_acceleration_mps2;

  const double ds_dey = reference_velocity * cos_heading * path_curvature /
    (denominator * denominator);
  const double ds_depsi = -reference_velocity * sin_heading / denominator;
  const double ds_dv = cos_heading / denominator;

  using StateMatrix = Eigen::Matrix<double, kExtendedStateDimension, kExtendedStateDimension>;
  using InputMatrix = Eigen::Matrix<double, kExtendedStateDimension, kExtendedInputDimension>;
  using StateVector = Eigen::Matrix<double, kExtendedStateDimension, 1>;
  using InputVector = Eigen::Matrix<double, kExtendedInputDimension, 1>;
  StateMatrix continuous_state = StateMatrix::Zero();
  continuous_state(kExtendedLateralIndex, kExtendedHeadingIndex) =
    reference_velocity * cos_heading;
  continuous_state(kExtendedLateralIndex, kExtendedVelocityIndex) = sin_heading;
  continuous_state(kExtendedLagIndex, kExtendedLateralIndex) = ds_dey;
  continuous_state(kExtendedLagIndex, kExtendedHeadingIndex) = ds_depsi;
  continuous_state(kExtendedLagIndex, kExtendedVelocityIndex) = ds_dv;
  continuous_state(kExtendedHeadingIndex, kExtendedVelocityIndex) = input_curvature;

  InputMatrix continuous_input = InputMatrix::Zero();
  continuous_input(kExtendedLagIndex, kExtendedVirtualProgressSpeedIndex) = -1.0;
  continuous_input(kExtendedHeadingIndex, kExtendedCurvatureIndex) = reference_velocity;
  continuous_input(kExtendedHeadingIndex, kExtendedVirtualProgressSpeedIndex) =
    -path_curvature;
  continuous_input(kExtendedVelocityIndex, kExtendedAccelerationIndex) = 1.0;
  continuous_input(kExtendedProgressIndex, kExtendedVirtualProgressSpeedIndex) = 1.0;

  ExtendedLinearization result;
  result.state_matrix = StateMatrix::Identity() + stage_dt * continuous_state;
  result.input_matrix = stage_dt * continuous_input;
  result.stage_dt_sec = stage_dt;

  const StateVector reference_state = (StateVector() <<
    lateral, request.reference_lag_m, heading, request.reference_velocity_mps,
    request.reference_progress_m).finished();
  const InputVector reference_input = (InputVector() <<
    request.reference_acceleration_mps2, input_curvature,
    virtual_progress_speed).finished();
  const StateVector next_reference = (StateVector() <<
    lateral + stage_dt * lateral_rate,
    request.reference_lag_m + stage_dt * lag_rate,
    heading + stage_dt * heading_rate,
    request.reference_velocity_mps + stage_dt * velocity_rate,
    request.reference_progress_m + stage_dt * virtual_progress_speed).finished();
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

std::optional<VelocityHorizon> resolve_velocity_horizon(
  const VelocityHorizonRequest & request) noexcept
{
  if (
    !finite_config(request.config) || request.reference_velocity_mps.empty() ||
    request.reference_velocity_mps.size() != request.hard_cap_velocity_mps.size())
  {
    return std::nullopt;
  }
  VelocityHorizon result;
  result.reference_velocity_mps.reserve(request.reference_velocity_mps.size());
  result.hard_cap_velocity_mps.reserve(request.hard_cap_velocity_mps.size());
  result.stage_weight.assign(
    request.reference_velocity_mps.size(),
    request.committed_pass ? request.config.committed_stage_velocity_weight :
    request.config.stage_velocity_weight);
  for (std::size_t stage = 0U; stage < request.reference_velocity_mps.size(); ++stage) {
    const double reference = request.reference_velocity_mps[stage];
    const double hard_cap = request.hard_cap_velocity_mps[stage];
    if (
      !std::isfinite(reference) || reference < 0.0 ||
      !std::isfinite(hard_cap) || hard_cap < 0.0)
    {
      return std::nullopt;
    }
    result.hard_cap_velocity_mps.push_back(hard_cap);
    result.reference_velocity_mps.push_back(std::min(reference, hard_cap));
  }
  result.terminal_target_velocity_mps = result.reference_velocity_mps.back();
  result.terminal_weight = request.committed_pass ?
    request.config.committed_terminal_velocity_weight :
    request.config.terminal_velocity_weight;
  if (
    !std::isfinite(result.terminal_target_velocity_mps) ||
    !std::isfinite(result.terminal_weight) || result.terminal_weight < 0.0)
  {
    return std::nullopt;
  }
  return result;
}

std::optional<ActuationProposal> extract_actuation_proposal(
  const Eigen::VectorXd & extended_primal, const int horizon_size,
  const double wheelbase_m) noexcept
{
  if (horizon_size <= 0 || !std::isfinite(wheelbase_m) || wheelbase_m <= 0.0) {
    return std::nullopt;
  }
  const int state_values = kExtendedStateDimension * (horizon_size + 1);
  const int expected_size = state_values + kExtendedInputDimension * horizon_size;
  if (extended_primal.size() != expected_size || !extended_primal.allFinite()) {
    return std::nullopt;
  }

  ActuationProposal proposal;
  proposal.predicted_speed_mps =
    extended_primal[kExtendedStateDimension + kExtendedVelocityIndex];
  proposal.acceleration_mps2 =
    extended_primal[state_values + kExtendedAccelerationIndex];
  proposal.curvature_radpm =
    extended_primal[state_values + kExtendedCurvatureIndex];
  proposal.virtual_progress_speed_mps =
    extended_primal[state_values + kExtendedVirtualProgressSpeedIndex];
  proposal.steering_tire_angle_rad = std::atan(
    wheelbase_m * proposal.curvature_radpm);
  if (
    !std::isfinite(proposal.predicted_speed_mps) ||
    !std::isfinite(proposal.acceleration_mps2) ||
    !std::isfinite(proposal.curvature_radpm) ||
    !std::isfinite(proposal.steering_tire_angle_rad) ||
    !std::isfinite(proposal.virtual_progress_speed_mps))
  {
    return std::nullopt;
  }
  return proposal;
}

const char * extended_constraint_row_kind_name(
  const ExtendedConstraintRowKind kind) noexcept
{
  switch (kind) {
    case ExtendedConstraintRowKind::Invalid:
      return "invalid";
    case ExtendedConstraintRowKind::DynamicsEquality:
      return "dynamics-equality";
    case ExtendedConstraintRowKind::StateBox:
      return "state-box";
    case ExtendedConstraintRowKind::InputBox:
      return "input-box";
    case ExtendedConstraintRowKind::CurvatureRate:
      return "curvature-rate";
    case ExtendedConstraintRowKind::FollowEffectiveGap:
      return "follow-effective-gap";
    case ExtendedConstraintRowKind::ProgressWallLower:
      return "progress-wall-lower";
    case ExtendedConstraintRowKind::ProgressWallUpper:
      return "progress-wall-upper";
  }
  return "unknown";
}

const char * extended_constraint_field_name(
  const ExtendedConstraintField field) noexcept
{
  switch (field) {
    case ExtendedConstraintField::None:
      return "none";
    case ExtendedConstraintField::Lateral:
      return "lateral";
    case ExtendedConstraintField::Lag:
      return "lag";
    case ExtendedConstraintField::Heading:
      return "heading";
    case ExtendedConstraintField::Velocity:
      return "velocity";
    case ExtendedConstraintField::Progress:
      return "progress";
    case ExtendedConstraintField::Acceleration:
      return "acceleration";
    case ExtendedConstraintField::Curvature:
      return "curvature";
    case ExtendedConstraintField::VirtualProgressSpeed:
      return "virtual-progress-speed";
  }
  return "unknown";
}

ExtendedConstraintRowSemantic decode_extended_constraint_row(
  const int row, const int horizon_size, const bool follow_gap_rows,
  const bool progress_wall_rows) noexcept
{
  ExtendedConstraintRowSemantic result;
  if (row < 0 || horizon_size <= 0) {
    return result;
  }
  const int state_rows = kExtendedStateDimension * (horizon_size + 1);
  const int input_rows = kExtendedInputDimension * horizon_size;
  const int variable_rows = state_rows + input_rows;
  const int box_offset = state_rows;
  const int rate_offset = box_offset + variable_rows;
  const int follow_gap_offset = rate_offset + horizon_size;
  const int follow_gap_row_count = follow_gap_rows ? horizon_size + 1 : 0;
  const int progress_wall_offset = follow_gap_offset + follow_gap_row_count;
  const int progress_wall_row_count = progress_wall_rows ? 2 * horizon_size : 0;
  const int constraint_rows = progress_wall_offset + progress_wall_row_count;
  if (row >= constraint_rows) {
    return result;
  }
  const auto state_field = [](const int index) {
      switch (index) {
        case kExtendedLateralIndex:
          return ExtendedConstraintField::Lateral;
        case kExtendedLagIndex:
          return ExtendedConstraintField::Lag;
        case kExtendedHeadingIndex:
          return ExtendedConstraintField::Heading;
        case kExtendedVelocityIndex:
          return ExtendedConstraintField::Velocity;
        case kExtendedProgressIndex:
          return ExtendedConstraintField::Progress;
        default:
          return ExtendedConstraintField::None;
      }
    };
  const auto input_field = [](const int index) {
      switch (index) {
        case kExtendedAccelerationIndex:
          return ExtendedConstraintField::Acceleration;
        case kExtendedCurvatureIndex:
          return ExtendedConstraintField::Curvature;
        case kExtendedVirtualProgressSpeedIndex:
          return ExtendedConstraintField::VirtualProgressSpeed;
        default:
          return ExtendedConstraintField::None;
      }
    };
  result.valid = true;
  if (row < state_rows) {
    result.kind = ExtendedConstraintRowKind::DynamicsEquality;
    result.field = state_field(row % kExtendedStateDimension);
    result.stage = row / kExtendedStateDimension;
    return result;
  }
  if (row < rate_offset) {
    const int variable = row - box_offset;
    if (variable < state_rows) {
      result.kind = ExtendedConstraintRowKind::StateBox;
      result.field = state_field(variable % kExtendedStateDimension);
      result.stage = variable / kExtendedStateDimension;
    } else {
      const int input = variable - state_rows;
      result.kind = ExtendedConstraintRowKind::InputBox;
      result.field = input_field(input % kExtendedInputDimension);
      result.stage = input / kExtendedInputDimension;
    }
    return result;
  }
  if (row < follow_gap_offset) {
    result.kind = ExtendedConstraintRowKind::CurvatureRate;
    result.field = ExtendedConstraintField::Curvature;
    result.stage = row - rate_offset;
  } else if (row < progress_wall_offset) {
    result.kind = ExtendedConstraintRowKind::FollowEffectiveGap;
    result.field = ExtendedConstraintField::Progress;
    result.stage = row - follow_gap_offset;
  } else {
    const int wall_row = row - progress_wall_offset;
    result.kind = wall_row % 2 == 0 ?
      ExtendedConstraintRowKind::ProgressWallLower :
      ExtendedConstraintRowKind::ProgressWallUpper;
    result.field = ExtendedConstraintField::Lateral;
    result.stage = wall_row / 2;
  }
  return result;
}

bool rebase_extended_progress_warm_start(
  Eigen::VectorXd & extended_primal, const int horizon_size,
  const double previous_progress_origin_m,
  const double current_progress_origin_m) noexcept
{
  if (
    horizon_size <= 0 || !extended_primal.allFinite() ||
    !std::isfinite(previous_progress_origin_m) ||
    !std::isfinite(current_progress_origin_m))
  {
    return false;
  }
  const int expected_size =
    kExtendedStateDimension * (horizon_size + 1) +
    kExtendedInputDimension * horizon_size;
  if (extended_primal.size() != expected_size) {
    return false;
  }
  const double origin_delta =
    previous_progress_origin_m - current_progress_origin_m;
  for (int stage = 0; stage < horizon_size + 1; ++stage) {
    extended_primal[stage * kExtendedStateDimension + kExtendedProgressIndex] +=
      origin_delta;
  }
  return extended_primal.allFinite();
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

const char * progress_aligned_wall_bounds_reason_name(
  const ProgressAlignedWallBoundsReason reason) noexcept
{
  switch (reason) {
    case ProgressAlignedWallBoundsReason::NotRequested:
      return "not-requested";
    case ProgressAlignedWallBoundsReason::Accepted:
      return "accepted";
    case ProgressAlignedWallBoundsReason::InvalidInput:
      return "invalid-input";
    case ProgressAlignedWallBoundsReason::NoCoveringSegment:
      return "no-covering-segment";
    case ProgressAlignedWallBoundsReason::CorridorCollapsed:
      return "corridor-collapsed";
  }
  return "unknown";
}

ProgressAlignedWallBoundsResolution resolve_progress_aligned_wall_bounds(
  const ProgressAlignedWallBoundsRequest & request) noexcept
{
  ProgressAlignedWallBoundsResolution result;
  if (!request.active) {
    result.valid = true;
    result.feasible = true;
    result.reason = ProgressAlignedWallBoundsReason::NotRequested;
    return result;
  }

  const std::size_t horizon = request.solved_progress_m.size();
  const bool matching_wall_profile =
    request.reference_progress_m.size() >= 2U &&
    request.reference_progress_m.size() == request.wall_lower_m.size() &&
    request.reference_progress_m.size() == request.wall_upper_m.size();
  if (
    horizon == 0U || !matching_wall_profile ||
    request.current_lower_m.size() != horizon ||
    request.current_upper_m.size() != horizon ||
    request.current_progress_lower_m.size() != horizon ||
    request.current_progress_upper_m.size() != horizon ||
    !std::isfinite(request.boundary_tolerance_m) ||
    request.boundary_tolerance_m < 0.0)
  {
    result.reason = ProgressAlignedWallBoundsReason::InvalidInput;
    return result;
  }

  for (std::size_t sample = 0U; sample < request.reference_progress_m.size(); ++sample) {
    if (
      !std::isfinite(request.reference_progress_m[sample]) ||
      !std::isfinite(request.wall_lower_m[sample]) ||
      !std::isfinite(request.wall_upper_m[sample]) ||
      request.wall_lower_m[sample] > request.wall_upper_m[sample] ||
      (sample > 0U &&
      request.reference_progress_m[sample] <=
      request.reference_progress_m[sample - 1U]))
    {
      result.reason = ProgressAlignedWallBoundsReason::InvalidInput;
      return result;
    }
  }
  for (std::size_t stage = 0U; stage < horizon; ++stage) {
    if (
      !std::isfinite(request.solved_progress_m[stage]) ||
      !std::isfinite(request.current_lower_m[stage]) ||
      !std::isfinite(request.current_upper_m[stage]) ||
      request.current_lower_m[stage] > request.current_upper_m[stage] ||
      !std::isfinite(request.current_progress_lower_m[stage]) ||
      !std::isfinite(request.current_progress_upper_m[stage]) ||
      request.current_progress_lower_m[stage] >
      request.current_progress_upper_m[stage])
    {
      result.reason = ProgressAlignedWallBoundsReason::InvalidInput;
      return result;
    }
  }

  result.progress_lower_m.reserve(horizon);
  result.progress_upper_m.reserve(horizon);
  result.wall_lower_slope.reserve(horizon);
  result.wall_lower_intercept.reserve(horizon);
  result.wall_upper_slope.reserve(horizon);
  result.wall_upper_intercept.reserve(horizon);
  const double first_progress = request.reference_progress_m.front();
  const double last_progress = request.reference_progress_m.back();
  for (std::size_t stage = 0U; stage < horizon; ++stage) {
    const double solved_progress = request.solved_progress_m[stage];
    const std::size_t reference_stage = std::min(
      stage, request.reference_progress_m.size() - 1U);
    result.maximum_progress_mismatch_m = std::max(
      result.maximum_progress_mismatch_m,
      std::abs(solved_progress - request.reference_progress_m[reference_stage]));
    if (
      solved_progress < first_progress - request.boundary_tolerance_m ||
      solved_progress > last_progress + request.boundary_tolerance_m)
    {
      ++result.out_of_range_stage_count;
      result.valid = true;
      result.reason = ProgressAlignedWallBoundsReason::NoCoveringSegment;
      result.first_failure_stage = static_cast<int>(stage);
      return result;
    }

    const double covered_progress = std::clamp(
      solved_progress, first_progress, last_progress);
    auto upper_sample = std::lower_bound(
      request.reference_progress_m.begin(),
      request.reference_progress_m.end(), covered_progress);
    if (upper_sample == request.reference_progress_m.begin()) {
      ++upper_sample;
    } else if (upper_sample == request.reference_progress_m.end()) {
      --upper_sample;
    }
    const std::size_t upper_index = static_cast<std::size_t>(
      std::distance(request.reference_progress_m.begin(), upper_sample));
    const std::size_t lower_index = upper_index - 1U;
    const double segment_lower = request.reference_progress_m[lower_index];
    const double segment_upper = request.reference_progress_m[upper_index];
    const double span = segment_upper - segment_lower;
    if (!std::isfinite(span) || span <= 0.0) {
      result.reason = ProgressAlignedWallBoundsReason::InvalidInput;
      return result;
    }
    const double lower_slope =
      (request.wall_lower_m[upper_index] -
      request.wall_lower_m[lower_index]) / span;
    const double upper_slope =
      (request.wall_upper_m[upper_index] -
      request.wall_upper_m[lower_index]) / span;
    const double lower_intercept =
      request.wall_lower_m[lower_index] - lower_slope * segment_lower;
    const double upper_intercept =
      request.wall_upper_m[lower_index] - upper_slope * segment_lower;
    const double constrained_progress_lower = std::max(
      request.current_progress_lower_m[stage], segment_lower);
    const double constrained_progress_upper = std::min(
      request.current_progress_upper_m[stage], segment_upper);
    if (
      !std::isfinite(lower_slope) || !std::isfinite(upper_slope) ||
      !std::isfinite(lower_intercept) || !std::isfinite(upper_intercept))
    {
      result.reason = ProgressAlignedWallBoundsReason::InvalidInput;
      return result;
    }
    if (constrained_progress_lower > constrained_progress_upper) {
      result.valid = true;
      result.reason = ProgressAlignedWallBoundsReason::CorridorCollapsed;
      result.first_failure_stage = static_cast<int>(stage);
      return result;
    }
    ++result.aligned_stage_count;
    result.progress_lower_m.push_back(constrained_progress_lower);
    result.progress_upper_m.push_back(constrained_progress_upper);
    result.wall_lower_slope.push_back(lower_slope);
    result.wall_lower_intercept.push_back(lower_intercept);
    result.wall_upper_slope.push_back(upper_slope);
    result.wall_upper_intercept.push_back(upper_intercept);
  }

  result.valid = true;
  result.feasible = true;
  result.applied = result.aligned_stage_count == horizon;
  result.reason = ProgressAlignedWallBoundsReason::Accepted;
  return result;
}

const char * progress_profile_support_reason_name(
  const ProgressProfileSupportReason reason) noexcept
{
  switch (reason) {
    case ProgressProfileSupportReason::Accepted:
      return "accepted";
    case ProgressProfileSupportReason::InvalidInput:
      return "invalid-input";
    case ProgressProfileSupportReason::CorridorCollapsed:
      return "corridor-collapsed";
  }
  return "unknown";
}

ProgressProfileSupportIntersection intersect_progress_profile_support(
  const double lower_m, const double upper_m,
  const double profile_lower_m, const double profile_upper_m) noexcept
{
  ProgressProfileSupportIntersection result;
  if (
    !std::isfinite(lower_m) || !std::isfinite(upper_m) ||
    !std::isfinite(profile_lower_m) ||
    !std::isfinite(profile_upper_m) || lower_m > upper_m ||
    profile_lower_m > profile_upper_m)
  {
    return result;
  }
  result.valid = true;
  result.lower_m = std::max(lower_m, profile_lower_m);
  result.upper_m = std::min(upper_m, profile_upper_m);
  if (result.lower_m > result.upper_m) {
    result.reason = ProgressProfileSupportReason::CorridorCollapsed;
    return result;
  }
  result.feasible = true;
  result.applied =
    result.lower_m != lower_m || result.upper_m != upper_m;
  result.reason = ProgressProfileSupportReason::Accepted;
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

bool rti_refinement_cold_load_active(const RtiColdLoadRequest & request) noexcept
{
  if (
    !request.progress_execution_context_active ||
    !std::isfinite(request.cold_entry_skip_sec) ||
    request.cold_entry_skip_sec < 0.0)
  {
    return false;
  }
  const bool cold_entry_active =
    request.cold_entry_skip_sec > 0.0 &&
    std::isfinite(request.now_sec) &&
    std::isfinite(request.mission_start_sec) &&
    request.now_sec >= request.mission_start_sec &&
    request.now_sec - request.mission_start_sec <= request.cold_entry_skip_sec + 1e-9;
  const bool cold_wall_cache_load =
    request.wall_cache_miss_skip_threshold > 0U &&
    request.wall_cache_miss_count >= request.wall_cache_miss_skip_threshold;
  return cold_entry_active || cold_wall_cache_load;
}

RtiRefinementDecision resolve_rti_refinement(
  const RtiRefinementRequest & request) noexcept
{
  if (!request.progress_mode_active || request.configured_iterations <= 1) {
    return RtiRefinementDecision::Disabled;
  }
  if (
    request.configured_iterations > 3 ||
    !std::isfinite(request.minimum_lateral_bound_reserve_m) ||
    !std::isfinite(request.lateral_defect_m) || request.lateral_defect_m < 0.0 ||
    !std::isfinite(request.heading_defect_rad) || request.heading_defect_rad < 0.0 ||
    !std::isfinite(request.maximum_curvature_radpm) ||
    request.maximum_curvature_radpm < 0.0 ||
    !std::isfinite(request.elapsed_ms) || request.elapsed_ms < 0.0 ||
    !std::isfinite(request.minimum_bound_reserve_threshold_m) ||
    request.minimum_bound_reserve_threshold_m < 0.0 ||
    !std::isfinite(request.lateral_defect_threshold_m) ||
    request.lateral_defect_threshold_m < 0.0 ||
    !std::isfinite(request.heading_defect_threshold_rad) ||
    request.heading_defect_threshold_rad < 0.0 ||
    !std::isfinite(request.curvature_threshold_radpm) ||
    request.curvature_threshold_radpm < 0.0 ||
    !std::isfinite(request.refinement_start_deadline_ms) ||
    request.refinement_start_deadline_ms <= 0.0)
  {
    return RtiRefinementDecision::Invalid;
  }
  if (request.cold_load_active) {
    return RtiRefinementDecision::SkipColdLoad;
  }
  if (request.elapsed_ms >= request.refinement_start_deadline_ms) {
    return RtiRefinementDecision::SkipDeadline;
  }
  if (!request.conditional_refinement_enabled) {
    return RtiRefinementDecision::Refine;
  }
  const bool refinement_needed =
    request.minimum_lateral_bound_reserve_m <=
    request.minimum_bound_reserve_threshold_m ||
    request.lateral_defect_m >= request.lateral_defect_threshold_m ||
    request.heading_defect_rad >= request.heading_defect_threshold_rad ||
    request.maximum_curvature_radpm >= request.curvature_threshold_radpm;
  return refinement_needed ?
    RtiRefinementDecision::Refine : RtiRefinementDecision::SkipCondition;
}

ExtendedBranchSelectionResolution select_extended_branch(
  const ExtendedBranchSelectionRequest & request) noexcept
{
  ExtendedBranchSelectionResolution resolution;
  if (
    !std::isfinite(request.minimum_objective_advantage) ||
    request.minimum_objective_advantage < 0.0 ||
    !std::isfinite(request.minimum_lateral_bound_reserve_m) ||
    request.minimum_lateral_bound_reserve_m < 0.0 ||
    (request.current_side_sign != -1 && request.current_side_sign != 0 &&
    request.current_side_sign != 1) ||
    (request.fallback_side_sign != -1 && request.fallback_side_sign != 0 &&
    request.fallback_side_sign != 1))
  {
    return resolution;
  }
  const auto classify = [&request](const ExtendedBranchEvaluation & branch) {
      if (branch.side_sign != -1 && branch.side_sign != 1) {
        return ExtendedBranchEligibility::InvalidSide;
      }
      if (!branch.attempted) {
        return ExtendedBranchEligibility::NotAttempted;
      }
      if (!branch.feasible) {
        return ExtendedBranchEligibility::SolverInfeasible;
      }
      if (!std::isfinite(branch.objective)) {
        return ExtendedBranchEligibility::InvalidObjective;
      }
      if (!std::isfinite(branch.minimum_lateral_bound_reserve_m)) {
        return ExtendedBranchEligibility::InvalidBoundReserve;
      }
      if (!branch.physical_wall_validation_attempted) {
        return ExtendedBranchEligibility::PhysicalWallUnchecked;
      }
      if (!branch.physical_wall_validation_passed) {
        return ExtendedBranchEligibility::PhysicalWallFailed;
      }
      if (
        branch.minimum_lateral_bound_reserve_m + 1e-9 >=
        request.minimum_lateral_bound_reserve_m)
      {
        return ExtendedBranchEligibility::Robust;
      }
      // The configured reserve is an extra robustness preference on top of
      // the QP hard bounds. If no robust branch exists, retain a numerically
      // on-bound solution only after its continuous physical wall contract
      // has independently passed. A wall-unchecked or wall-failed branch is
      // never promoted by this fallback.
      if (
        branch.minimum_lateral_bound_reserve_m >= -1e-9)
      {
        return ExtendedBranchEligibility::PhysicalBoundaryFallback;
      }
      return ExtendedBranchEligibility::InsufficientBoundReserve;
    };
  resolution.left_eligibility = classify(request.left);
  resolution.right_eligibility = classify(request.right);
  const bool left_robust =
    resolution.left_eligibility == ExtendedBranchEligibility::Robust &&
    request.left.side_sign == 1;
  const bool right_robust =
    resolution.right_eligibility == ExtendedBranchEligibility::Robust &&
    request.right.side_sign == -1;
  const bool left_boundary =
    resolution.left_eligibility ==
    ExtendedBranchEligibility::PhysicalBoundaryFallback &&
    request.left.side_sign == 1;
  const bool right_boundary =
    resolution.right_eligibility ==
    ExtendedBranchEligibility::PhysicalBoundaryFallback &&
    request.right.side_sign == -1;
  const auto branch_for_side = [&request](const int side)
    -> const ExtendedBranchEvaluation * {
      if (side == 1 && request.left.side_sign == 1) {
        return &request.left;
      }
      if (side == -1 && request.right.side_sign == -1) {
        return &request.right;
      }
      return nullptr;
    };
  const auto side_eligible = [&](const int side) {
      return side == 1 ? left_robust || left_boundary :
             side == -1 ? right_robust || right_boundary : false;
    };
  const auto side_is_boundary = [&](const int side) {
      return side == 1 ? left_boundary : side == -1 ? right_boundary : false;
    };
  if (
    request.no_return && request.current_side_sign != 0 &&
    side_eligible(request.current_side_sign))
  {
    resolution.valid = true;
    resolution.selected_side_sign = request.current_side_sign;
    resolution.physical_boundary_fallback_used =
      side_is_boundary(request.current_side_sign);
    resolution.reason = ExtendedBranchSelectionReason::NoReturnCurrentSide;
    return resolution;
  }
  // A robust branch always wins over a boundary fallback, regardless of the
  // objective. Only compare boundary branches when no robust branch exists.
  const bool robust_pool_available = left_robust || right_robust;
  const bool left_feasible = robust_pool_available ? left_robust : left_boundary;
  const bool right_feasible = robust_pool_available ? right_robust : right_boundary;
  resolution.physical_boundary_fallback_used =
    !robust_pool_available && (left_feasible || right_feasible);
  const auto side_feasible = [&](const int side) {
      return side == 1 ? left_feasible : side == -1 ? right_feasible : false;
    };
  if (!left_feasible && !right_feasible) {
    return resolution;
  }
  if (left_feasible != right_feasible) {
    resolution.valid = true;
    resolution.selected_side_sign = left_feasible ? 1 : -1;
    resolution.reason = left_feasible ?
      ExtendedBranchSelectionReason::OnlyLeftFeasible :
      ExtendedBranchSelectionReason::OnlyRightFeasible;
    return resolution;
  }

  const double left_objective = request.left.objective;
  const double right_objective = request.right.objective;
  const int objective_best_side = left_objective <= right_objective ? 1 : -1;
  const int other_side = -objective_best_side;
  const auto * best = branch_for_side(objective_best_side);
  const auto * other = branch_for_side(other_side);
  if (best == nullptr || other == nullptr) {
    return resolution;
  }
  resolution.objective_advantage = other->objective - best->objective;
  if (
    request.current_side_sign != 0 && side_feasible(request.current_side_sign) &&
    objective_best_side != request.current_side_sign &&
    resolution.objective_advantage + 1e-9 < request.minimum_objective_advantage)
  {
    resolution.valid = true;
    resolution.selected_side_sign = request.current_side_sign;
    resolution.reason = ExtendedBranchSelectionReason::CurrentSideHysteresis;
    return resolution;
  }
  if (
    std::abs(left_objective - right_objective) <= 1e-9 &&
    request.fallback_side_sign != 0 && side_feasible(request.fallback_side_sign))
  {
    resolution.valid = true;
    resolution.selected_side_sign = request.fallback_side_sign;
    resolution.reason = ExtendedBranchSelectionReason::FallbackTieBreak;
    return resolution;
  }
  resolution.valid = true;
  resolution.selected_side_sign = objective_best_side;
  resolution.reason = ExtendedBranchSelectionReason::LowerObjective;
  return resolution;
}

const char * extended_branch_entry_admission_reason_name(
  const ExtendedBranchEntryAdmissionReason reason) noexcept
{
  switch (reason) {
    case ExtendedBranchEntryAdmissionReason::NotRequired:
      return "not-required";
    case ExtendedBranchEntryAdmissionReason::SelectedBranch:
      return "selected-branch";
    case ExtendedBranchEntryAdmissionReason::SelectionUnavailable:
      return "selection-unavailable";
    case ExtendedBranchEntryAdmissionReason::MissionSideMismatch:
      return "mission-side-mismatch";
    case ExtendedBranchEntryAdmissionReason::InvalidInput:
      return "invalid-input";
  }
  return "unknown";
}

ExtendedBranchEntryAdmissionResolution resolve_extended_branch_entry_admission(
  const ExtendedBranchEntryAdmissionRequest & request) noexcept
{
  ExtendedBranchEntryAdmissionResolution resolution;
  const auto valid_side = [](const int side) {
      return side == -1 || side == 0 || side == 1;
    };
  if (!valid_side(request.selected_side_sign) || !valid_side(request.mission_side_sign)) {
    return resolution;
  }
  resolution.valid = true;
  if (!request.enabled || !request.new_entry) {
    resolution.admitted = true;
    resolution.reason = ExtendedBranchEntryAdmissionReason::NotRequired;
    return resolution;
  }
  if (!request.selection_valid || request.selected_side_sign == 0) {
    resolution.hold_current_path = true;
    resolution.reason = ExtendedBranchEntryAdmissionReason::SelectionUnavailable;
    return resolution;
  }
  if (
    request.mission_side_sign == 0 ||
    request.mission_side_sign != request.selected_side_sign)
  {
    resolution.hold_current_path = true;
    resolution.reason = ExtendedBranchEntryAdmissionReason::MissionSideMismatch;
    return resolution;
  }
  resolution.admitted = true;
  resolution.reason = ExtendedBranchEntryAdmissionReason::SelectedBranch;
  return resolution;
}

const char * extended_branch_selection_reason_name(
  const ExtendedBranchSelectionReason reason) noexcept
{
  switch (reason) {
    case ExtendedBranchSelectionReason::None:
      return "none";
    case ExtendedBranchSelectionReason::OnlyLeftFeasible:
      return "only left feasible";
    case ExtendedBranchSelectionReason::OnlyRightFeasible:
      return "only right feasible";
    case ExtendedBranchSelectionReason::LowerObjective:
      return "lower objective";
    case ExtendedBranchSelectionReason::CurrentSideHysteresis:
      return "current-side hysteresis";
    case ExtendedBranchSelectionReason::NoReturnCurrentSide:
      return "no-return current side";
    case ExtendedBranchSelectionReason::FallbackTieBreak:
      return "fallback tie-break";
  }
  return "unknown";
}

const char * extended_branch_eligibility_name(
  const ExtendedBranchEligibility eligibility) noexcept
{
  switch (eligibility) {
    case ExtendedBranchEligibility::Robust:
      return "robust";
    case ExtendedBranchEligibility::PhysicalBoundaryFallback:
      return "physical-boundary";
    case ExtendedBranchEligibility::InvalidSide:
      return "invalid-side";
    case ExtendedBranchEligibility::NotAttempted:
      return "not-attempted";
    case ExtendedBranchEligibility::SolverInfeasible:
      return "solver-infeasible";
    case ExtendedBranchEligibility::InvalidObjective:
      return "invalid-objective";
    case ExtendedBranchEligibility::InvalidBoundReserve:
      return "invalid-bound-reserve";
    case ExtendedBranchEligibility::PhysicalWallUnchecked:
      return "wall-unchecked";
    case ExtendedBranchEligibility::PhysicalWallFailed:
      return "wall-failed";
    case ExtendedBranchEligibility::InsufficientBoundReserve:
      return "insufficient-bound-reserve";
  }
  return "unknown";
}

std::optional<ExecutionTrajectory> extract_execution_trajectory(
  const Eigen::VectorXd & primal, const int horizon_size,
  const std::vector<double> & path_distance_m,
  const std::vector<double> & lateral_lower_m,
  const std::vector<double> & lateral_upper_m,
  const double bound_tolerance_m,
  ExecutionTrajectoryDiagnostic * const diagnostic) noexcept
{
  constexpr int nx = 3;
  constexpr int nu = 2;
  const auto reject = [diagnostic](
      const ExecutionTrajectoryRejection rejection,
      const int stage = -1) -> std::optional<ExecutionTrajectory> {
      if (diagnostic != nullptr) {
        diagnostic->rejection = rejection;
        diagnostic->stage = stage;
      }
      return std::nullopt;
    };
  if (diagnostic != nullptr) {
    *diagnostic = ExecutionTrajectoryDiagnostic{};
  }
  if (
    horizon_size <= 0 ||
    primal.size() != nx * (horizon_size + 1) + nu * horizon_size ||
    !primal.allFinite() ||
    path_distance_m.size() != static_cast<std::size_t>(horizon_size) ||
    lateral_lower_m.size() != path_distance_m.size() ||
    lateral_upper_m.size() != path_distance_m.size() ||
    !std::isfinite(bound_tolerance_m) || bound_tolerance_m < 0.0)
  {
    return reject(ExecutionTrajectoryRejection::InvalidInput);
  }

  ExecutionTrajectory result;
  result.path_distance_m = path_distance_m;
  result.lateral_m.reserve(path_distance_m.size());
  result.progress_m.reserve(path_distance_m.size());
  result.minimum_lateral_bound_reserve_m =
    std::numeric_limits<double>::infinity();
  double previous_distance = -std::numeric_limits<double>::infinity();
  double previous_progress = primal[2];
  for (int stage = 0; stage < horizon_size; ++stage) {
    const std::size_t index = static_cast<std::size_t>(stage);
    const double distance = path_distance_m[index];
    const double lower = lateral_lower_m[index];
    const double upper = lateral_upper_m[index];
    const double lateral = primal[(stage + 1) * nx];
    const double progress = primal[(stage + 1) * nx + 2];
    if (
      !std::isfinite(distance) || distance < 0.0 ||
      distance <= previous_distance)
    {
      return reject(ExecutionTrajectoryRejection::InvalidPathDistance, stage);
    }
    if (!std::isfinite(lower) || !std::isfinite(upper) || lower > upper) {
      return reject(ExecutionTrajectoryRejection::InvalidLateralBounds, stage);
    }
    if (!std::isfinite(lateral) || !std::isfinite(progress)) {
      return reject(ExecutionTrajectoryRejection::NonFiniteState, stage);
    }
    if (progress + bound_tolerance_m < previous_progress) {
      return reject(ExecutionTrajectoryRejection::ProgressRegressed, stage);
    }
    if (
      lateral < lower - bound_tolerance_m ||
      lateral > upper + bound_tolerance_m)
    {
      return reject(ExecutionTrajectoryRejection::LateralOutOfBounds, stage);
    }
    result.lateral_m.push_back(lateral);
    result.progress_m.push_back(progress);
    result.minimum_lateral_bound_reserve_m = std::min(
      result.minimum_lateral_bound_reserve_m,
      std::min(lateral - lower, upper - lateral));
    previous_distance = distance;
    previous_progress = progress;
  }
  if (
    result.lateral_m.empty() ||
    !std::isfinite(result.minimum_lateral_bound_reserve_m))
  {
    return reject(ExecutionTrajectoryRejection::EmptyTrajectory);
  }
  result.minimum_lateral_bound_reserve_m = std::max(
    0.0, result.minimum_lateral_bound_reserve_m);
  return result;
}

const char * execution_trajectory_rejection_name(
  const ExecutionTrajectoryRejection rejection) noexcept
{
  switch (rejection) {
    case ExecutionTrajectoryRejection::None:
      return "none";
    case ExecutionTrajectoryRejection::InvalidInput:
      return "invalid input";
    case ExecutionTrajectoryRejection::InvalidPathDistance:
      return "invalid path distance";
    case ExecutionTrajectoryRejection::InvalidLateralBounds:
      return "invalid lateral bounds";
    case ExecutionTrajectoryRejection::NonFiniteState:
      return "non-finite state";
    case ExecutionTrajectoryRejection::ProgressRegressed:
      return "progress regressed";
    case ExecutionTrajectoryRejection::LateralOutOfBounds:
      return "lateral out of bounds";
    case ExecutionTrajectoryRejection::EmptyTrajectory:
      return "empty trajectory";
  }
  return "unknown";
}

}  // namespace multi_purpose_mpc_ros::mpcc_progress
