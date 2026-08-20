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
    std::isfinite(config.extended_failure_cooldown_sec) &&
    config.extended_failure_cooldown_sec >= 0.0 &&
    std::isfinite(config.extended_mode_handoff_sec) &&
    config.extended_mode_handoff_sec >= 0.0 &&
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
    !std::isfinite(request.stage_distance_m) || request.stage_distance_m <= 0.0)
  {
    return std::nullopt;
  }

  const double reference_velocity = std::max(
    request.config.minimum_reference_speed_mps, request.reference_velocity_mps);
  const double stage_dt = std::clamp(
    request.stage_distance_m / reference_velocity,
    request.config.minimum_stage_dt_sec,
    request.config.maximum_stage_dt_sec);
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

std::optional<Eigen::VectorXd> convert_extended_solution_to_legacy(
  const Eigen::VectorXd & extended_primal, const int horizon_size,
  const double progress_origin_m) noexcept
{
  if (horizon_size <= 0 || !std::isfinite(progress_origin_m)) {
    return std::nullopt;
  }
  const int extended_state_values = kExtendedStateDimension * (horizon_size + 1);
  const int expected_size =
    extended_state_values + kExtendedInputDimension * horizon_size;
  if (extended_primal.size() != expected_size || !extended_primal.allFinite()) {
    return std::nullopt;
  }
  constexpr int legacy_state_dimension = 3;
  constexpr int legacy_input_dimension = 2;
  const int legacy_state_values = legacy_state_dimension * (horizon_size + 1);
  Eigen::VectorXd result = Eigen::VectorXd::Zero(
    legacy_state_values + legacy_input_dimension * horizon_size);
  for (int stage = 0; stage < horizon_size + 1; ++stage) {
    const int source = stage * kExtendedStateDimension;
    const int target = stage * legacy_state_dimension;
    result[target] = extended_primal[source + kExtendedLateralIndex];
    result[target + 1] = extended_primal[source + kExtendedHeadingIndex];
    result[target + 2] =
      extended_primal[source + kExtendedProgressIndex] + progress_origin_m;
  }
  for (int stage = 0; stage < horizon_size; ++stage) {
    const int source = extended_state_values + stage * kExtendedInputDimension;
    const int target = legacy_state_values + stage * legacy_input_dimension;
    result[target] = extended_primal[(stage + 1) * kExtendedStateDimension +
      kExtendedVelocityIndex];
    result[target + 1] = extended_primal[source + kExtendedCurvatureIndex];
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

bool ExtendedSolverCircuitBreaker::active(const double now_sec) const noexcept
{
  return
    std::isfinite(now_sec) && std::isfinite(disabled_until_sec_) &&
    now_sec < disabled_until_sec_;
}

void ExtendedSolverCircuitBreaker::record_failure(
  const double now_sec, const double cooldown_sec) noexcept
{
  if (
    !std::isfinite(now_sec) || !std::isfinite(cooldown_sec) ||
    cooldown_sec < 0.0)
  {
    reset();
    return;
  }
  disabled_until_sec_ = now_sec + cooldown_sec;
}

void ExtendedSolverCircuitBreaker::record_success() noexcept
{
  reset();
}

void ExtendedSolverCircuitBreaker::reset() noexcept
{
  disabled_until_sec_ = -std::numeric_limits<double>::infinity();
}

double ExtendedSolverCircuitBreaker::disabled_until_sec() const noexcept
{
  return disabled_until_sec_;
}

void ExtendedSolverReentryGate::record_failure() noexcept
{
  requalification_required_ = true;
  consecutive_successes_ = 0U;
}

ExtendedSolverReentryResolution ExtendedSolverReentryGate::record_success(
  const std::size_t required_successes) noexcept
{
  const std::size_t bounded_required = std::max<std::size_t>(1U, required_successes);
  if (!requalification_required_) {
    return ExtendedSolverReentryResolution{true, false, 0U, bounded_required};
  }

  if (consecutive_successes_ < bounded_required) {
    ++consecutive_successes_;
  }
  if (consecutive_successes_ < bounded_required) {
    return ExtendedSolverReentryResolution{
      false, true, consecutive_successes_, bounded_required};
  }

  const std::size_t confirmed_successes = consecutive_successes_;
  reset();
  return ExtendedSolverReentryResolution{
    true, false, confirmed_successes, bounded_required};
}

void ExtendedSolverReentryGate::reset() noexcept
{
  requalification_required_ = false;
  consecutive_successes_ = 0U;
}

bool ExtendedSolverReentryGate::requalification_required() const noexcept
{
  return requalification_required_;
}

std::size_t ExtendedSolverReentryGate::consecutive_successes() const noexcept
{
  return consecutive_successes_;
}

std::optional<ExtendedModeHandoffResolution> ExtendedModeHandoff::resolve_velocity(
  const bool extended_mode, const double now_sec, const double desired_velocity_mps,
  const double current_lower_mps, const double current_upper_mps,
  const double handoff_duration_sec) noexcept
{
  if (
    !std::isfinite(now_sec) || !std::isfinite(desired_velocity_mps) ||
    !std::isfinite(current_lower_mps) || !std::isfinite(current_upper_mps) ||
    current_lower_mps > current_upper_mps ||
    !std::isfinite(handoff_duration_sec) || handoff_duration_sec < 0.0)
  {
    reset();
    return std::nullopt;
  }

  const double bounded_desired = std::clamp(
    desired_velocity_mps, current_lower_mps, current_upper_mps);
  if (!previous_extended_mode_.has_value()) {
    previous_extended_mode_ = extended_mode;
    last_output_velocity_mps_ = bounded_desired;
    return ExtendedModeHandoffResolution{bounded_desired, 1.0, false};
  }

  if (
    !std::isfinite(last_output_velocity_mps_) ||
    (std::isfinite(transition_start_sec_) && now_sec < transition_start_sec_))
  {
    reset();
    previous_extended_mode_ = extended_mode;
    last_output_velocity_mps_ = bounded_desired;
    return ExtendedModeHandoffResolution{bounded_desired, 1.0, false};
  }

  if (previous_extended_mode_.value() != extended_mode) {
    previous_extended_mode_ = extended_mode;
    transition_start_sec_ = now_sec;
    transition_source_velocity_mps_ = std::clamp(
      last_output_velocity_mps_, current_lower_mps, current_upper_mps);
  }

  const bool transition_initialized =
    std::isfinite(transition_start_sec_) &&
    std::isfinite(transition_source_velocity_mps_);
  const double transition_elapsed_sec = transition_initialized ?
    std::max(0.0, now_sec - transition_start_sec_) : 0.0;
  constexpr double kTransitionCompletionToleranceSec = 1e-9;
  const double blend_ratio = !transition_initialized || handoff_duration_sec <= 0.0 ?
    1.0 :
    transition_elapsed_sec + kTransitionCompletionToleranceSec >= handoff_duration_sec ?
    1.0 : std::clamp(
    transition_elapsed_sec / handoff_duration_sec, 0.0, 1.0);
  const double velocity = std::clamp(
    transition_initialized ?
    transition_source_velocity_mps_ +
    blend_ratio * (bounded_desired - transition_source_velocity_mps_) :
    bounded_desired,
    current_lower_mps, current_upper_mps);
  const bool active = transition_initialized && blend_ratio < 1.0;
  if (!active) {
    transition_start_sec_ = -std::numeric_limits<double>::infinity();
    transition_source_velocity_mps_ = std::numeric_limits<double>::quiet_NaN();
  }
  last_output_velocity_mps_ = velocity;
  return ExtendedModeHandoffResolution{velocity, blend_ratio, active};
}

void ExtendedModeHandoff::reset() noexcept
{
  previous_extended_mode_.reset();
  last_output_velocity_mps_ = std::numeric_limits<double>::quiet_NaN();
  transition_source_velocity_mps_ = std::numeric_limits<double>::quiet_NaN();
  transition_start_sec_ = -std::numeric_limits<double>::infinity();
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
  const auto eligible = [&request](const ExtendedBranchEvaluation & branch) {
      return
        (branch.side_sign == -1 || branch.side_sign == 1) &&
        branch.attempted && branch.feasible && std::isfinite(branch.objective) &&
        std::isfinite(branch.minimum_lateral_bound_reserve_m) &&
        branch.minimum_lateral_bound_reserve_m + 1e-9 >=
        request.minimum_lateral_bound_reserve_m;
    };
  const bool left_feasible = eligible(request.left) && request.left.side_sign == 1;
  const bool right_feasible = eligible(request.right) && request.right.side_sign == -1;
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
  const auto side_feasible = [&](const int side) {
      return side == 1 ? left_feasible : side == -1 ? right_feasible : false;
    };
  if (
    request.no_return && request.current_side_sign != 0 &&
    side_feasible(request.current_side_sign))
  {
    resolution.valid = true;
    resolution.selected_side_sign = request.current_side_sign;
    resolution.reason = ExtendedBranchSelectionReason::NoReturnCurrentSide;
    return resolution;
  }
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
