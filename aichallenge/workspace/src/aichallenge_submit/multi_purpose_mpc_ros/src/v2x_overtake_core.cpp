#include "multi_purpose_mpc_ros/v2x_overtake_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace multi_purpose_mpc_ros::v2x_overtake_core
{
namespace
{

bool is_configured_side(const PassSide side) noexcept
{
  return side == PassSide::Left || side == PassSide::Right;
}

bool is_feasible(const SideSelectionRequest & request, const PassSide side) noexcept
{
  if (side == PassSide::Left) {
    return request.left_feasible;
  }
  if (side == PassSide::Right) {
    return request.right_feasible;
  }
  return false;
}

void validate_speed(const double speed_mps, const char * name)
{
  if (!std::isfinite(speed_mps) || speed_mps < 0.0) {
    throw std::invalid_argument(std::string(name) + " must be finite and non-negative");
  }
}

}  // namespace

SpeedLimitResolution resolve_effective_speed_limit(const SpeedLimitRequest & request)
{
  validate_speed(request.normal_speed_mps, "normal speed");
  validate_speed(request.global_hard_cap_mps, "global hard cap");
  if (!std::isfinite(request.start_window_duration_sec) ||
    request.start_window_duration_sec < 0.0)
  {
    throw std::invalid_argument("Start window duration must be finite and non-negative");
  }
  if (request.start_speed_mps.has_value()) {
    validate_speed(*request.start_speed_mps, "Start speed");
  }

  const double normal_speed_mps =
    std::min(request.normal_speed_mps, request.global_hard_cap_mps);
  if (!request.start_speed_mps.has_value() || request.start_window_duration_sec == 0.0) {
    return {normal_speed_mps, StartWindowStatus::NotConfigured};
  }
  if (!request.elapsed_since_start_sec.has_value()) {
    return {normal_speed_mps, StartWindowStatus::AwaitingStart};
  }

  const double elapsed_sec = *request.elapsed_since_start_sec;
  if (!std::isfinite(elapsed_sec) || elapsed_sec < 0.0) {
    return {normal_speed_mps, StartWindowStatus::InvalidElapsed};
  }
  if (elapsed_sec >= request.start_window_duration_sec) {
    return {normal_speed_mps, StartWindowStatus::Expired};
  }

  return {
    std::min(*request.start_speed_mps, request.global_hard_cap_mps),
    StartWindowStatus::Applied};
}

const char * to_string(const StartWindowStatus status) noexcept
{
  switch (status) {
    case StartWindowStatus::NotConfigured:
      return "normal";
    case StartWindowStatus::AwaitingStart:
      return "awaiting_start";
    case StartWindowStatus::InvalidElapsed:
      return "invalid_elapsed";
    case StartWindowStatus::Applied:
      return "start_window";
    case StartWindowStatus::Expired:
      return "expired_normal";
  }
  return "unknown";
}

FollowSpeedLimitResolution resolve_follow_speed_limit(
  const FollowSpeedLimitRequest & request)
{
  validate_speed(request.activation_distance_m, "Follow activation distance");
  validate_speed(request.moving_front_speed_threshold_mps, "moving-front speed threshold");
  validate_speed(request.moving_front_speed_margin_mps, "moving-front speed margin");
  validate_speed(request.moving_front_target_distance_m, "moving-front target distance");
  validate_speed(
    request.moving_front_recovery_speed_margin_mps,
    "moving-front recovery speed margin");
  validate_speed(request.moving_front_distance_gain, "moving-front distance gain");
  validate_speed(request.slow_front_velocity_cap_mps, "slow-front velocity cap");
  validate_speed(request.maximum_speed_mps, "maximum speed");

  if (
    !request.enabled || request.suppressed ||
    !std::isfinite(request.front_distance_m) || request.front_distance_m < 0.0 ||
    (request.activation_distance_m > 0.0 &&
    request.front_distance_m > request.activation_distance_m))
  {
    return {};
  }

  validate_speed(request.slow_front_distance_limit_mps, "slow-front distance limit");
  const bool moving_front =
    std::isfinite(request.front_speed_mps) && request.front_speed_mps >= 0.0 &&
    request.front_speed_mps > request.moving_front_speed_threshold_mps;
  if (!moving_front) {
    return {
      true, false, false, 0.0,
      std::min(request.slow_front_distance_limit_mps, request.slow_front_velocity_cap_mps)};
  }

  double moving_speed_margin_mps = request.moving_front_speed_margin_mps;
  bool clearance_recovery = false;
  if (
    request.moving_front_target_distance_m > 0.0 &&
    request.moving_front_distance_gain > 0.0)
  {
    moving_speed_margin_mps = std::clamp(
      request.moving_front_distance_gain *
      (request.front_distance_m - request.moving_front_target_distance_m),
      -request.moving_front_recovery_speed_margin_mps,
      request.moving_front_speed_margin_mps);
    clearance_recovery =
      moving_speed_margin_mps < request.moving_front_speed_margin_mps;
  }
  const double speed_limit_mps = std::min(
    request.maximum_speed_mps,
    std::max(0.0, request.front_speed_mps + moving_speed_margin_mps));
  return {true, true, clearance_recovery, moving_speed_margin_mps, speed_limit_mps};
}

bool should_apply_generic_follow_cap(
  const GenericFollowCapOwnershipRequest & request) noexcept
{
  return
    !request.front_matches_locked_target ||
    (!request.shiftout_active && !request.pass_active);
}

OvertakeSpeedReferenceResolution resolve_overtake_speed_reference(
  const OvertakeSpeedReferenceRequest & request)
{
  validate_speed(request.base_reference_speed_mps, "base overtake reference");
  validate_speed(request.hard_cap_mps, "overtake hard cap");
  validate_speed(request.front_speed_mps, "front speed");
  validate_speed(request.entry_speed_mps, "overtake entry speed");
  validate_speed(request.shiftout_max_closing_speed_mps, "ShiftOut closing speed");

  const double base_reference =
    std::min(request.base_reference_speed_mps, request.hard_cap_mps);
  if (request.stage == OvertakeSpeedStage::Pass) {
    return {base_reference, false};
  }

  const double front_cap = std::min(
    request.hard_cap_mps,
    std::max(
      request.entry_speed_mps,
      request.front_speed_mps + request.shiftout_max_closing_speed_mps));
  return {std::min(base_reference, front_cap), true};
}

OvertakeSpeedReferenceResolution resolve_start_grid_breakout_speed_reference(
  const StartGridBreakoutSpeedReferenceRequest & request)
{
  return resolve_overtake_speed_reference(
    OvertakeSpeedReferenceRequest{
      request.validated_breakout ? OvertakeSpeedStage::Pass : OvertakeSpeedStage::ShiftOut,
      request.base_reference_speed_mps,
      request.hard_cap_mps,
      request.front_speed_mps,
      request.entry_speed_mps,
      request.shiftout_max_closing_speed_mps});
}

bool has_reached_pass_side_lateral_goal(
  const double current_lateral_m, const double target_lateral_m,
  const double lateral_tolerance_m, const int pass_side_sign) noexcept
{
  if (
    !std::isfinite(current_lateral_m) || !std::isfinite(target_lateral_m) ||
    !std::isfinite(lateral_tolerance_m) || lateral_tolerance_m < 0.0 ||
    pass_side_sign == 0)
  {
    return false;
  }
  return pass_side_sign > 0 ?
    current_lateral_m >= target_lateral_m - lateral_tolerance_m :
    current_lateral_m <= target_lateral_m + lateral_tolerance_m;
}

bool is_shiftout_complete(const ShiftOutCompletionRequest & request) noexcept
{
  if (
    !request.phase_hold_elapsed ||
    !std::isfinite(request.traveled_distance_m) || request.traveled_distance_m < 0.0 ||
    !std::isfinite(request.required_distance_m) || request.required_distance_m < 0.0)
  {
    return false;
  }
  return request.traveled_distance_m >= request.required_distance_m &&
         has_reached_pass_side_lateral_goal(
           request.current_lateral_m, request.target_lateral_m,
           request.lateral_tolerance_m, request.pass_side_sign);
}

bool can_release_overtake_front_cap(
  const OvertakeFrontCapReleaseRequest & request) noexcept
{
  if (
    !request.active_execution_phase || !request.target_seen ||
    !std::isfinite(request.target_longitudinal_m) ||
    !request.lateral_complete || !request.execution_horizon_unconstrained)
  {
    return false;
  }
  const bool lateral_release =
    request.lateral_separation_clear ||
    (request.lateral_separation_release_active &&
    request.lateral_separation_above_reapply_threshold);
  return lateral_release || request.target_longitudinal_m <= 0.0;
}

bool can_exclude_locked_target_from_front_overlap(
  const PassFrontOverlapExclusionRequest & request) noexcept
{
  if (!request.active_execution_phase || !request.locked_target) {
    return false;
  }
  if (request.already_latched) {
    return true;
  }
  return std::isfinite(request.relative_lateral_m) &&
         std::isfinite(request.required_lateral_clearance_m) &&
         request.required_lateral_clearance_m > 0.0 &&
         std::abs(request.relative_lateral_m) >= request.required_lateral_clearance_m;
}

bool can_hold_active_pass_after_gap_loss(const ActivePassGapHoldRequest & request) noexcept
{
  return request.pass_phase && request.lateral_clearance_latched &&
         request.locked_target_seen && !request.locked_target_position_jump;
}

bool can_hold_validated_start_grid_breakout(
  const ValidatedStartGridBreakoutContinuityRequest & request) noexcept
{
  return request.continuing_breakout && request.active_line && request.target_matches &&
         request.locked_target_seen && !request.locked_target_position_jump &&
         !request.explicit_forbidden_wp;
}

bool should_resolve_curve_pass_side(
  const bool soft_curve_forbidden, const bool hard_curve_forbidden,
  const bool explicit_forbidden_wp) noexcept
{
  return !explicit_forbidden_wp && (soft_curve_forbidden || hard_curve_forbidden);
}

ActiveLineGapLossHoldResolution resolve_active_line_gap_loss_hold(
  const ActiveLineGapLossHoldRequest & request) noexcept
{
  ActiveLineGapLossHoldResolution resolution;
  if (
    !request.enabled || !request.active_line || !request.locked_target_seen ||
    request.locked_target_position_jump || !request.transient_gap_failure ||
    request.explicit_forbidden_wp || request.cooldown_active || request.emergency_brake ||
    !std::isfinite(request.now_sec) || !std::isfinite(request.last_valid_gap_sec) ||
    !std::isfinite(request.hold_sec) || request.hold_sec <= 0.0 ||
    request.now_sec < request.last_valid_gap_sec)
  {
    return resolution;
  }

  const double elapsed_sec = request.now_sec - request.last_valid_gap_sec;
  resolution.remaining_sec = std::max(0.0, request.hold_sec - elapsed_sec);
  resolution.active = elapsed_sec <= request.hold_sec;
  return resolution;
}

bool explicit_overtake_line_owns_lateral_plan(
  const OvertakeLateralPlannerOwnershipRequest & request) noexcept
{
  return request.explicit_line_enabled &&
         (request.behavior_requests_overtake || request.line_phase_active);
}

bool should_apply_gap_planner_state_bounds(
  const GapPlannerStateBoundsRequest & request) noexcept
{
  return !request.explicit_line_owns_plan;
}

bool should_block_live_execution_corridor(
  const LiveExecutionCorridorBlockRequest & request) noexcept
{
  return request.raw_corridor_blocked &&
         !(request.pass_phase && request.lateral_clearance_latched);
}

bool should_apply_gap_planner_no_gap_velocity_limit(
  const GapPlannerNoGapVelocityLimitRequest & request) noexcept
{
  const bool behavior_allows_limit =
    !request.follow_behavior || request.follow_limit_enabled;
  return behavior_allows_limit &&
         !request.overtake_fallback_target &&
         !request.committed_execution_corridor_bypass &&
         !request.transient_execution_corridor_hold;
}

bool locked_target_intrudes_pass_side(
  const LockedTargetPassSideIntrusionRequest & request) noexcept
{
  if (
    !request.active_line || request.pass_side_sign == 0 ||
    request.lateral_clearance_latched || !request.target_seen ||
    request.target_position_jump || !std::isfinite(request.target_longitudinal_m) ||
    request.target_longitudinal_m <= 0.0 ||
    std::isnan(request.maximum_guard_longitudinal_m) ||
    request.maximum_guard_longitudinal_m < 0.0 ||
    (std::isfinite(request.maximum_guard_longitudinal_m) &&
    request.target_longitudinal_m > request.maximum_guard_longitudinal_m + 1e-9) ||
    !std::isfinite(request.target_relative_lateral_m) ||
    !std::isfinite(request.ordering_margin_m) || request.ordering_margin_m < 0.0)
  {
    return false;
  }

  // For a right pass (side=-1), target-relative lateral must remain positive:
  // target is left of ego. The signs are mirrored for a left pass.
  const double signed_target_ordering =
    static_cast<double>(request.pass_side_sign) * request.target_relative_lateral_m;
  return signed_target_ordering >= -request.ordering_margin_m;
}

bool can_start_side_overtake(const SideOvertakeEntryRequest & request) noexcept
{
  if (request.continuing_overtake) {
    return true;
  }
  if (
    !std::isfinite(request.target_longitudinal_m) ||
    !std::isfinite(request.rear_tolerance_m) || request.rear_tolerance_m < 0.0)
  {
    return false;
  }
  return request.target_longitudinal_m >= -request.rear_tolerance_m;
}

bool side_only_target_requires_follow(
  const double target_longitudinal_m, const double rear_tolerance_m) noexcept
{
  if (
    !std::isfinite(target_longitudinal_m) ||
    !std::isfinite(rear_tolerance_m) || rear_tolerance_m < 0.0)
  {
    return true;
  }
  return target_longitudinal_m >= -rear_tolerance_m;
}

double resolve_unlatched_pass_closing_speed(
  const double configured_closing_speed_mps,
  const double unlatched_pass_closing_speed_mps,
  const bool pass_phase, const bool lateral_clearance_latched)
{
  validate_speed(configured_closing_speed_mps, "configured overtake closing speed");
  validate_speed(unlatched_pass_closing_speed_mps, "unlatched Pass closing speed");
  if (pass_phase && !lateral_clearance_latched) {
    return std::min(configured_closing_speed_mps, unlatched_pass_closing_speed_mps);
  }
  return configured_closing_speed_mps;
}

double resolve_overtake_line_horizon_progress(
  const OvertakeLineHorizonProgressRequest & request) noexcept
{
  if (request.hold_target) {
    return 1.0;
  }
  if (
    !std::isfinite(request.phase_traveled_m) || request.phase_traveled_m < 0.0 ||
    !std::isfinite(request.horizon_distance_m) || request.horizon_distance_m < 0.0 ||
    !std::isfinite(request.phase_distance_m) || request.phase_distance_m <= 0.0)
  {
    return 0.0;
  }
  const double linear_progress = std::clamp(
    (request.phase_traveled_m + request.horizon_distance_m) /
    request.phase_distance_m,
    0.0, 1.0);
  return linear_progress * linear_progress * (3.0 - 2.0 * linear_progress);
}

double resolve_overtake_line_heading_reference(
  const OvertakeLineHeadingReferenceRequest & request) noexcept
{
  if (
    !std::isfinite(request.previous_lateral_m) ||
    !std::isfinite(request.current_lateral_m) ||
    !std::isfinite(request.delta_s_m) || request.delta_s_m <= 0.0 ||
    !std::isfinite(request.base_curvature_radpm))
  {
    return 0.0;
  }

  const double lateral_gradient =
    (request.current_lateral_m - request.previous_lateral_m) / request.delta_s_m;
  // Frenet offset geometry: dr_offset/ds has tangential component 1-kappa*d
  // and normal component d'. Keep a small signed denominator around a cusp.
  double tangential_scale =
    1.0 - request.base_curvature_radpm * request.current_lateral_m;
  if (std::abs(tangential_scale) < 1e-3) {
    tangential_scale = std::copysign(1e-3, tangential_scale == 0.0 ? 1.0 : tangential_scale);
  }
  return std::atan2(lateral_gradient, tangential_scale);
}

bool should_abort_active_overtake_for_static_wall(
  const bool active_execution_phase, const bool wall_geometry_available,
  const bool sample_valid, const bool sample_out_of_map,
  const bool sample_has_contact) noexcept
{
  return active_execution_phase && wall_geometry_available &&
         (!sample_valid || sample_out_of_map || sample_has_contact);
}

bool static_wall_clamp_requires_overtake_recovery(
  const bool active_execution_phase, const bool static_target_adjusted,
  const double required_lateral_accel_mps2,
  const double maximum_lateral_accel_mps2) noexcept
{
  if (
    !active_execution_phase || !static_target_adjusted ||
    !std::isfinite(maximum_lateral_accel_mps2) ||
    maximum_lateral_accel_mps2 <= 0.0)
  {
    return false;
  }
  return !std::isfinite(required_lateral_accel_mps2) ||
         required_lateral_accel_mps2 > maximum_lateral_accel_mps2;
}

double resolve_pass_side_lateral_goal(const PassSideLateralGoalRequest & request) noexcept
{
  if (
    request.pass_side_sign == 0 ||
    !std::isfinite(request.base_lateral_offset_m) || request.base_lateral_offset_m < 0.0 ||
    !std::isfinite(request.minimum_separation_m) || request.minimum_separation_m < 0.0)
  {
    return 0.0;
  }
  if (request.fixed_lateral_goal_m.has_value()) {
    const double fixed_goal = request.fixed_lateral_goal_m.value();
    if (std::isfinite(fixed_goal)) {
      return fixed_goal;
    }
  }
  const double base_goal = static_cast<double>(request.pass_side_sign) *
    request.base_lateral_offset_m;
  if (!std::isfinite(request.target_lateral_m)) {
    return base_goal;
  }
  const double target_side_goal = request.target_lateral_m +
    static_cast<double>(request.pass_side_sign) * request.minimum_separation_m;
  return request.pass_side_sign > 0 ?
    std::max(base_goal, target_side_goal) : std::min(base_goal, target_side_goal);
}

FeasiblePassSideLateralGoalResolution resolve_feasible_pass_side_lateral_goal(
  const FeasiblePassSideLateralGoalRequest & request) noexcept
{
  FeasiblePassSideLateralGoalResolution resolution;
  if (
    !std::isfinite(request.preferred_goal_m) ||
    !std::isfinite(request.feasible_lower_bound_m) ||
    !std::isfinite(request.feasible_upper_bound_m) ||
    request.feasible_upper_bound_m < request.feasible_lower_bound_m)
  {
    return resolution;
  }

  const auto clamp_to_interval = [](const double value, const double lower, const double upper) {
      return std::min(upper, std::max(lower, value));
    };
  resolution.goal_m = clamp_to_interval(
    request.preferred_goal_m,
    request.feasible_lower_bound_m, request.feasible_upper_bound_m);

  const bool valid_separation_request =
    request.enforce_target_separation &&
    request.pass_side_sign != 0 &&
    std::isfinite(request.target_lateral_m) &&
    std::isfinite(request.minimum_separation_m) &&
    request.minimum_separation_m >= 0.0;
  if (!request.enforce_target_separation) {
    resolution.target_separation_feasible = true;
    return resolution;
  }
  if (!valid_separation_request) {
    return resolution;
  }

  double separated_lower = request.feasible_lower_bound_m;
  double separated_upper = request.feasible_upper_bound_m;
  const double target_side_limit =
    request.target_lateral_m +
    static_cast<double>(request.pass_side_sign) * request.minimum_separation_m;
  if (request.pass_side_sign > 0) {
    separated_lower = std::max(separated_lower, target_side_limit);
  } else {
    separated_upper = std::min(separated_upper, target_side_limit);
  }
  if (separated_upper < separated_lower) {
    return resolution;
  }

  resolution.goal_m = clamp_to_interval(
    request.preferred_goal_m, separated_lower, separated_upper);
  resolution.target_separation_feasible = true;
  return resolution;
}

std::optional<double> resolve_pass_corridor_center(
  const PassCorridorCenterRequest & request) noexcept
{
  if (
    !request.active || !std::isfinite(request.lower_bound_m) ||
    !std::isfinite(request.upper_bound_m) || request.upper_bound_m < request.lower_bound_m)
  {
    return std::nullopt;
  }
  return 0.5 * (request.lower_bound_m + request.upper_bound_m);
}

bool should_return_completed_pass_before_margin_recovery(
  const CompletedPassReturnRequest & request) noexcept
{
  return request.pass_phase &&
         request.lateral_separation_latched &&
         request.target_seen &&
         !request.physical_path_blocked &&
         std::isfinite(request.target_longitudinal_m) &&
         std::isfinite(request.rear_clear_distance_m) &&
         request.rear_clear_distance_m >= 0.0 &&
         request.target_longitudinal_m <= -request.rear_clear_distance_m;
}

bool blocks_overtake_return_corridor(
  const ReturnCorridorOccupancyRequest & request) noexcept
{
  if (
    request.vehicle_is_locked_target || !request.geometry_valid ||
    !std::isfinite(request.ego_lateral_m) ||
    !std::isfinite(request.vehicle_lateral_m) ||
    !std::isfinite(request.vehicle_longitudinal_m) ||
    !std::isfinite(request.lateral_clearance_m) ||
    !std::isfinite(request.rear_clearance_m) ||
    !std::isfinite(request.front_clearance_m) ||
    request.lateral_clearance_m < 0.0 ||
    request.rear_clearance_m < 0.0 ||
    request.front_clearance_m < 0.0)
  {
    return false;
  }

  const double swept_lower =
    std::min(0.0, request.ego_lateral_m) - request.lateral_clearance_m;
  const double swept_upper =
    std::max(0.0, request.ego_lateral_m) + request.lateral_clearance_m;
  const bool inside_lateral_sweep =
    request.vehicle_lateral_m >= swept_lower &&
    request.vehicle_lateral_m <= swept_upper;
  const bool inside_longitudinal_window =
    request.vehicle_longitudinal_m >= -request.rear_clearance_m &&
    request.vehicle_longitudinal_m <= request.front_clearance_m;
  return inside_lateral_sweep && inside_longitudinal_window;
}

bool should_cancel_early_overtake_return(
  const EarlyReturnCancellationRequest & request) noexcept
{
  if (
    !request.return_phase || !request.reacquire_enabled ||
    !request.return_corridor_blocked ||
    !std::isfinite(request.return_elapsed_sec) ||
    !std::isfinite(request.reacquire_window_sec) ||
    !std::isfinite(request.return_progress) ||
    !std::isfinite(request.maximum_return_progress) ||
    request.return_elapsed_sec < 0.0 ||
    request.reacquire_window_sec < 0.0 ||
    request.return_progress < 0.0 ||
    request.maximum_return_progress < 0.0)
  {
    return false;
  }
  return request.return_elapsed_sec <= request.reacquire_window_sec &&
         request.return_progress <= request.maximum_return_progress;
}

AdaptiveShiftOutClosingSpeedResolution resolve_adaptive_shiftout_closing_speed(
  const AdaptiveShiftOutClosingSpeedRequest & request)
{
  validate_speed(request.minimum_closing_speed_mps, "minimum ShiftOut closing speed");
  validate_speed(request.maximum_closing_speed_mps, "maximum ShiftOut closing speed");
  validate_speed(request.front_distance_m, "ShiftOut front distance");
  validate_speed(request.protected_front_distance_m, "protected ShiftOut front distance");
  validate_speed(request.remaining_shiftout_distance_m, "remaining ShiftOut distance");
  validate_speed(request.ego_speed_mps, "ShiftOut ego speed");
  if (request.maximum_closing_speed_mps < request.minimum_closing_speed_mps) {
    throw std::invalid_argument(
            "maximum ShiftOut closing speed must not be below minimum closing speed");
  }
  if (!std::isfinite(request.minimum_speed_mps) || request.minimum_speed_mps <= 0.0) {
    throw std::invalid_argument("minimum ShiftOut speed must be finite and positive");
  }
  if (!std::isfinite(request.minimum_time_sec) || request.minimum_time_sec <= 0.0) {
    throw std::invalid_argument("minimum ShiftOut time must be finite and positive");
  }

  const double speed_for_time = std::max(request.ego_speed_mps, request.minimum_speed_mps);
  const double remaining_time = std::max(
    request.minimum_time_sec, request.remaining_shiftout_distance_m / speed_for_time);
  const double distance_budget = std::max(
    0.0, request.front_distance_m - request.protected_front_distance_m);
  const double raw_closing_speed = distance_budget / remaining_time;
  return {
    std::clamp(
      raw_closing_speed, request.minimum_closing_speed_mps,
      request.maximum_closing_speed_mps),
    remaining_time,
    distance_budget};
}

double advance_prediction_time(const PredictionTimeRequest & request)
{
  if (!std::isfinite(request.elapsed_sec) || request.elapsed_sec < 0.0) {
    throw std::invalid_argument("Prediction elapsed time must be finite and non-negative");
  }
  if (!std::isfinite(request.segment_distance_m) || request.segment_distance_m < 0.0) {
    throw std::invalid_argument("Prediction segment distance must be finite and non-negative");
  }
  validate_speed(request.predicted_speed_mps, "predicted path speed");
  if (!std::isfinite(request.minimum_speed_mps) || request.minimum_speed_mps <= 0.0) {
    throw std::invalid_argument("Prediction minimum speed must be finite and positive");
  }
  if (!std::isfinite(request.maximum_time_sec) || request.maximum_time_sec < 0.0) {
    throw std::invalid_argument("Prediction maximum time must be finite and non-negative");
  }

  const double segment_time = request.segment_distance_m /
    std::max(request.predicted_speed_mps, request.minimum_speed_mps);
  return std::min(request.maximum_time_sec, request.elapsed_sec + segment_time);
}

CourseAlignedPrediction resolve_course_aligned_prediction(
  const CourseAlignedPredictionRequest & request) noexcept
{
  CourseAlignedPrediction result{
    false, request.fallback_longitudinal_m, request.fallback_lateral_m};
  if (
    !request.enabled || !request.projection_valid ||
    !std::isfinite(request.target_forward_distance_m) ||
    !std::isfinite(request.target_lateral_m) ||
    !std::isfinite(request.target_along_track_speed_mps) ||
    request.target_along_track_speed_mps < 0.0 ||
    !std::isfinite(request.horizon_time_sec) || request.horizon_time_sec < 0.0 ||
    !std::isfinite(request.ego_horizon_course_distance_m) ||
    request.ego_horizon_course_distance_m < 0.0)
  {
    return result;
  }

  const double longitudinal =
    request.target_forward_distance_m +
    request.target_along_track_speed_mps * request.horizon_time_sec -
    request.ego_horizon_course_distance_m;
  if (!std::isfinite(longitudinal)) {
    return result;
  }
  result.used_course_alignment = true;
  result.longitudinal_m = longitudinal;
  result.lateral_m = request.target_lateral_m;
  return result;
}

CourseLateralPrediction resolve_course_lateral_prediction(
  const CourseLateralPredictionRequest & request) noexcept
{
  CourseLateralPrediction result{
    false, request.fallback_lateral_m, 0.0, 0.0};
  if (
    !request.enabled || !request.current_projection_valid ||
    !request.previous_projection_valid ||
    !std::isfinite(request.current_lateral_m) ||
    !std::isfinite(request.previous_lateral_m) ||
    !std::isfinite(request.sample_interval_sec) ||
    request.sample_interval_sec <= 0.0 ||
    !std::isfinite(request.sample_age_sec) || request.sample_age_sec < 0.0 ||
    !std::isfinite(request.horizon_time_sec) || request.horizon_time_sec < 0.0 ||
    !std::isfinite(request.velocity_deadband_mps) ||
    request.velocity_deadband_mps < 0.0 ||
    !std::isfinite(request.maximum_velocity_mps) ||
    request.maximum_velocity_mps < 0.0)
  {
    return result;
  }

  const double raw_velocity =
    (request.current_lateral_m - request.previous_lateral_m) /
    request.sample_interval_sec;
  if (!std::isfinite(raw_velocity)) {
    return result;
  }
  const double applied_velocity =
    std::abs(raw_velocity) <= request.velocity_deadband_mps ?
    0.0 :
    std::clamp(
      raw_velocity, -request.maximum_velocity_mps, request.maximum_velocity_mps);
  const double lateral =
    request.current_lateral_m +
    applied_velocity * (request.sample_age_sec + request.horizon_time_sec);
  if (!std::isfinite(lateral)) {
    return result;
  }

  result.used_course_lateral_velocity = true;
  result.lateral_m = lateral;
  result.raw_lateral_velocity_mps = raw_velocity;
  result.applied_lateral_velocity_mps = applied_velocity;
  return result;
}

double resolve_vehicle_relative_lateral(
  const VehicleRelativeLateralRequest & request) noexcept
{
  if (
    request.course_projection_used && std::isfinite(request.vehicle_course_lateral_m) &&
    std::isfinite(request.ego_course_lateral_m))
  {
    return request.vehicle_course_lateral_m - request.ego_course_lateral_m;
  }
  return std::isfinite(request.local_relative_lateral_m) ?
         request.local_relative_lateral_m : std::numeric_limits<double>::infinity();
}

ForwardCourseProjection project_forward_course_progress(
  const std::vector<CoursePoint> & path, const ForwardCourseProjectionRequest & request)
{
  if (path.size() < 2U) {
    throw std::invalid_argument("Course projection requires at least two path points");
  }
  const auto finite = [](const double value) {return std::isfinite(value);};
  if (
    !finite(request.origin_x_m) || !finite(request.origin_y_m) ||
    !finite(request.target_x_m) || !finite(request.target_y_m) ||
    !finite(request.target_vx_mps) || !finite(request.target_vy_mps))
  {
    throw std::invalid_argument("Course projection pose and velocity must be finite");
  }
  if (
    !finite(request.lookbehind_distance_m) || request.lookbehind_distance_m < 0.0 ||
    !finite(request.lookahead_distance_m) || request.lookahead_distance_m <= 0.0 ||
    !finite(request.max_cross_track_distance_m) ||
    request.max_cross_track_distance_m <= 0.0)
  {
    throw std::invalid_argument(
            "Course projection distances must be finite with positive lookahead/cross-track");
  }
  if (
    request.preferred_target_path_progress_m.has_value() &&
    (!finite(request.preferred_target_path_progress_m.value()) ||
    !finite(request.max_target_path_progress_change_m) ||
    request.max_target_path_progress_change_m < 0.0))
  {
    throw std::invalid_argument(
            "Preferred target progress requires finite progress and non-negative change");
  }
  for (const auto & point : path) {
    if (!finite(point.x_m) || !finite(point.y_m)) {
      throw std::invalid_argument("Course projection path points must be finite");
    }
  }

  const std::size_t point_count = path.size();
  const auto next_index = [&](const std::size_t index) {
      if (index + 1U < point_count) {
        return index + 1U;
      }
      return request.circular ? 0U : index;
    };
  const auto previous_index = [&](const std::size_t index) {
      if (index > 0U) {
        return index - 1U;
      }
      return request.circular ? point_count - 1U : index;
    };
  const auto segment_length = [&](const std::size_t from, const std::size_t to) {
      return std::hypot(path[to].x_m - path[from].x_m, path[to].y_m - path[from].y_m);
    };
  std::vector<double> path_progress_m(point_count, 0.0);
  for (std::size_t index = 1U; index < point_count; ++index) {
    const double length_m = segment_length(index - 1U, index);
    path_progress_m[index] =
      path_progress_m[index - 1U] +
      (finite(length_m) && length_m > 1e-9 ? length_m : 0.0);
  }
  double total_path_length_m = path_progress_m.back();
  if (request.circular) {
    const double closure_length_m = segment_length(point_count - 1U, 0U);
    if (finite(closure_length_m) && closure_length_m > 1e-9) {
      total_path_length_m += closure_length_m;
    }
  }

  std::size_t anchor = request.circular ?
    request.start_index % point_count : std::min(request.start_index, point_count - 1U);
  double distance_before_anchor_m = 0.0;
  for (std::size_t step = 0U;
    step + 1U < point_count &&
    distance_before_anchor_m + 1e-9 < request.lookbehind_distance_m; ++step)
  {
    const std::size_t previous = previous_index(anchor);
    if (previous == anchor) {
      break;
    }
    const double length_m = segment_length(previous, anchor);
    anchor = previous;
    // Runtime ReferencePath may contain an appended circular closure point.
    // Treat a zero-length closure or repeated interior sample as no progress;
    // rejecting the entire control cycle would turn a harmless path encoding
    // detail into a permanent MPC fallback at the lap seam.
    if (!finite(length_m) || length_m <= 1e-9) {
      continue;
    }
    distance_before_anchor_m += length_m;
  }

  struct Candidate
  {
    bool valid{false};
    double score{std::numeric_limits<double>::infinity()};
    double progress_m{};
    double lateral_m{};
    double cross_track_m{};
    double tangent_x{};
    double tangent_y{};
    std::size_t segment_index{};
    double path_progress_m{};
  };
  Candidate origin;
  Candidate target;
  const double target_speed_mps = std::hypot(request.target_vx_mps, request.target_vy_mps);
  double segment_start_progress_m = -distance_before_anchor_m;
  std::size_t current = anchor;
  const std::size_t maximum_segments = request.circular ? point_count : point_count - 1U;

  const auto consider = [&](
      Candidate & best, const double px, const double py, const std::size_t from,
      const std::size_t to, const double length_m, const double start_progress_m,
      const bool target_candidate) {
      const double tangent_x = (path[to].x_m - path[from].x_m) / length_m;
      const double tangent_y = (path[to].y_m - path[from].y_m) / length_m;
      const double dx = px - path[from].x_m;
      const double dy = py - path[from].y_m;
      const double along_m = std::clamp(dx * tangent_x + dy * tangent_y, 0.0, length_m);
      const double projected_x = path[from].x_m + along_m * tangent_x;
      const double projected_y = path[from].y_m + along_m * tangent_y;
      const double residual_x = px - projected_x;
      const double residual_y = py - projected_y;
      const double lateral_m = -tangent_y * residual_x + tangent_x * residual_y;
      const double cross_track_m = std::hypot(residual_x, residual_y);
      if (cross_track_m > request.max_cross_track_distance_m) {
        return;
      }
      const double target_path_progress_m = path_progress_m[from] + along_m;
      if (target_candidate && request.preferred_target_path_progress_m.has_value()) {
        double progress_change_m =
          target_path_progress_m - request.preferred_target_path_progress_m.value();
        if (request.circular && total_path_length_m > 1e-9) {
          progress_change_m = std::remainder(progress_change_m, total_path_length_m);
        }
        if (
          std::abs(progress_change_m) >
          request.max_target_path_progress_change_m + 1e-9)
        {
          return;
        }
      }
      double score = cross_track_m * cross_track_m;
      if (target_candidate && target_speed_mps > 0.25) {
        const double direction_alignment =
          (request.target_vx_mps * tangent_x + request.target_vy_mps * tangent_y) /
          target_speed_mps;
        if (direction_alignment < -0.2) {
          return;
        }
        score += 0.04 * (1.0 - std::clamp(direction_alignment, -1.0, 1.0));
      }
      const double progress_m = start_progress_m + along_m;
      if (
        !best.valid || score + 1e-12 < best.score ||
        (std::abs(score - best.score) <= 1e-12 && progress_m < best.progress_m))
      {
        best.valid = true;
        best.score = score;
        best.progress_m = progress_m;
        best.lateral_m = lateral_m;
        best.cross_track_m = cross_track_m;
        best.tangent_x = tangent_x;
        best.tangent_y = tangent_y;
        best.segment_index = from;
        best.path_progress_m = target_path_progress_m;
      }
    };

  for (std::size_t step = 0U; step < maximum_segments; ++step) {
    const std::size_t next = next_index(current);
    if (next == current) {
      break;
    }
    const double length_m = segment_length(current, next);
    if (!finite(length_m) || length_m <= 1e-9) {
      current = next;
      continue;
    }
    consider(
      origin, request.origin_x_m, request.origin_y_m, current, next, length_m,
      segment_start_progress_m, false);
    consider(
      target, request.target_x_m, request.target_y_m, current, next, length_m,
      segment_start_progress_m, true);
    segment_start_progress_m += length_m;
    current = next;
    if (segment_start_progress_m > request.lookahead_distance_m + length_m) {
      break;
    }
  }

  ForwardCourseProjection result;
  if (!origin.valid || !target.valid) {
    return result;
  }
  result.forward_distance_m = target.progress_m - origin.progress_m;
  if (
    result.forward_distance_m < -request.lookbehind_distance_m - 1e-9 ||
    result.forward_distance_m > request.lookahead_distance_m + 1e-9)
  {
    return ForwardCourseProjection{};
  }
  result.valid = true;
  result.lateral_m = target.lateral_m;
  result.along_track_speed_mps = std::max(
    0.0,
    request.target_vx_mps * target.tangent_x +
    request.target_vy_mps * target.tangent_y);
  result.cross_track_distance_m = target.cross_track_m;
  result.segment_index = target.segment_index;
  result.target_path_progress_m = target.path_progress_m;
  return result;
}

bool is_course_progress_continuity_constraint_rejection(
  const bool continuity_constraint_applied, const bool constrained_projection_valid,
  const bool unconstrained_projection_valid) noexcept
{
  return continuity_constraint_applied &&
         !constrained_projection_valid &&
         unconstrained_projection_valid;
}

bool is_relative_course_progress_continuous(
  const RelativeCourseProgressContinuityRequest & request) noexcept
{
  if (
    !std::isfinite(request.previous_longitudinal_m) ||
    !std::isfinite(request.observed_longitudinal_m) ||
    !std::isfinite(request.elapsed_sec) || request.elapsed_sec < 0.0 ||
    !std::isfinite(request.ego_speed_mps) || request.ego_speed_mps < 0.0 ||
    !std::isfinite(request.target_speed_mps) || request.target_speed_mps < 0.0 ||
    !std::isfinite(request.tolerance_m) || request.tolerance_m < 0.0)
  {
    return false;
  }
  const double maximum_change_m =
    request.tolerance_m +
    (request.ego_speed_mps + request.target_speed_mps) * request.elapsed_sec;
  return std::abs(request.observed_longitudinal_m - request.previous_longitudinal_m) <=
         maximum_change_m + 1e-9;
}

PassCompletionResolution resolve_pass_completion(const PassCompletionRequest & request)
{
  if (
    std::isnan(request.distance_to_hard_curve_m) || request.distance_to_hard_curve_m < 0.0)
  {
    throw std::invalid_argument("Hard-curve distance must be non-negative or positive infinity");
  }
  validate_speed(request.curve_buffer_m, "hard-curve buffer");
  validate_speed(request.front_distance_m, "front distance");
  validate_speed(request.front_speed_mps, "front speed");
  if (!std::isfinite(request.planned_ego_speed_mps) || request.planned_ego_speed_mps <= 0.0) {
    throw std::invalid_argument("Planned ego speed must be finite and positive");
  }
  validate_speed(request.return_clear_distance_m, "return clear distance");
  validate_speed(request.minimum_shift_distance_m, "minimum ShiftOut distance");
  validate_speed(request.merge_buffer_m, "merge buffer");
  if (
    !std::isfinite(request.minimum_relative_speed_mps) ||
    request.minimum_relative_speed_mps <= 0.0)
  {
    throw std::invalid_argument("Minimum relative speed must be finite and positive");
  }

  PassCompletionResolution resolution;
  resolution.available_distance_m = std::isinf(request.distance_to_hard_curve_m) ?
    request.distance_to_hard_curve_m :
    std::max(0.0, request.distance_to_hard_curve_m - request.curve_buffer_m);
  resolution.relative_speed_mps = request.planned_ego_speed_mps - request.front_speed_mps;
  resolution.required_distance_m = std::numeric_limits<double>::infinity();
  if (
    resolution.relative_speed_mps + 1e-9 <
    request.minimum_relative_speed_mps)
  {
    return resolution;
  }

  const double relative_gain = request.front_distance_m + request.return_clear_distance_m;
  const double pass_time = relative_gain / resolution.relative_speed_mps;
  const double pass_distance = request.planned_ego_speed_mps * pass_time;
  resolution.required_distance_m =
    std::max(request.minimum_shift_distance_m, pass_distance) + request.merge_buffer_m;
  resolution.feasible =
    resolution.available_distance_m + 1e-9 >= resolution.required_distance_m;
  return resolution;
}

bool can_override_completion_for_curve_entry(
  const CurveEntryCompletionOverrideRequest & request) noexcept
{
  if (
    !request.curve_entry_allowed || request.line_committed ||
    !request.front_vehicle_seen ||
    !std::isfinite(request.front_distance_m) || request.front_distance_m < 0.0 ||
    !std::isfinite(request.maximum_front_distance_m) ||
    request.maximum_front_distance_m < 0.0 ||
    !std::isfinite(request.ego_speed_mps) || request.ego_speed_mps < 0.0 ||
    !std::isfinite(request.front_speed_mps) || request.front_speed_mps < 0.0 ||
    !std::isfinite(request.minimum_relative_speed_mps) ||
    request.minimum_relative_speed_mps < 0.0)
  {
    return false;
  }
  return request.front_distance_m <= request.maximum_front_distance_m + 1e-9 &&
         request.ego_speed_mps - request.front_speed_mps + 1e-9 >=
         request.minimum_relative_speed_mps;
}

bool can_precommit_inner_curve_line(
  const InnerCurvePrecommitRequest & request) noexcept
{
  if (
    !request.enabled || !request.inner_curve_entry_allowed || request.line_committed ||
    !request.front_vehicle_seen || request.emergency_brake_required ||
    !std::isfinite(request.front_distance_m) || request.front_distance_m < 0.0 ||
    !std::isfinite(request.minimum_front_distance_m) ||
    request.minimum_front_distance_m < 0.0 ||
    !std::isfinite(request.maximum_front_distance_m) ||
    request.maximum_front_distance_m < request.minimum_front_distance_m ||
    !std::isfinite(request.continuous_open_distance_m) ||
    request.continuous_open_distance_m < 0.0 ||
    !std::isfinite(request.minimum_open_distance_m) ||
    request.minimum_open_distance_m < 0.0 ||
    !std::isfinite(request.ego_speed_mps) || request.ego_speed_mps < 0.0 ||
    !std::isfinite(request.front_speed_mps) || request.front_speed_mps < 0.0 ||
    !std::isfinite(request.minimum_relative_speed_mps))
  {
    return false;
  }

  return request.front_distance_m + 1e-9 >= request.minimum_front_distance_m &&
         request.front_distance_m <= request.maximum_front_distance_m + 1e-9 &&
         request.continuous_open_distance_m + 1e-9 >= request.minimum_open_distance_m &&
         request.ego_speed_mps - request.front_speed_mps + 1e-9 >=
         request.minimum_relative_speed_mps;
}

bool overtake_completion_policy_allows_execution(
  const OvertakeCompletionPermissionRequest & request) noexcept
{
  return
    request.completion_feasible || request.curve_continuation_allowed ||
    can_override_completion_for_curve_entry(
    CurveEntryCompletionOverrideRequest{
      request.curve_entry_allowed,
      request.line_committed,
      request.front_vehicle_seen,
      request.front_distance_m,
      request.maximum_front_distance_m,
      request.ego_speed_mps,
      request.front_speed_mps,
      request.minimum_relative_speed_mps});
}

OvertakeGuardPhaseResolution resolve_overtake_guard_phase(
  const OvertakeGuardPhaseRequest & request)
{
  validate_speed(request.entry_min_front_distance_m, "overtake entry front distance");
  validate_speed(
    request.continuation_min_front_distance_m, "overtake continuation front distance");
  if (request.continuation_min_front_distance_m > request.entry_min_front_distance_m) {
    throw std::invalid_argument(
            "Overtake continuation front distance must not exceed entry front distance");
  }

  if (request.continuing_overtake) {
    return {request.continuation_min_front_distance_m, false};
  }
  return {request.entry_min_front_distance_m, true};
}

bool can_continue_overtake_in_soft_curve(
  const OvertakeCurveContinuationRequest & request) noexcept
{
  return request.continuation_enabled &&
         request.continuing_overtake &&
         request.soft_curve_forbidden &&
         !request.hard_curve_forbidden &&
         !request.cooldown_active &&
         !request.emergency_brake &&
         (!request.inner_curve_pass || request.inner_soft_curve_enabled);
}

OuterCurveOvertakeResolution resolve_outer_curve_overtake(
  const OuterCurveOvertakeRequest & request) noexcept
{
  OuterCurveOvertakeResolution resolution;
  const bool outside_curve =
    request.pass_side_sign != 0 && request.inner_curve_pass_side != 0 &&
    request.pass_side_sign != request.inner_curve_pass_side;
  if (
    !outside_curve || request.explicit_forbidden_wp || request.cooldown_active ||
    request.emergency_brake)
  {
    return resolution;
  }

  resolution.entry_allowed =
    request.entry_enabled && !request.continuing_overtake &&
    request.soft_curve_forbidden && !request.hard_curve_forbidden &&
    request.gap_available;
  resolution.hard_entry_allowed =
    request.entry_enabled && request.hard_entry_enabled &&
    !request.continuing_overtake && request.hard_curve_forbidden &&
    request.gap_available;
  resolution.hard_continuation_allowed =
    request.hard_continuation_enabled && request.continuing_overtake &&
    request.hard_curve_forbidden && request.gap_available &&
    request.locked_target_seen;
  return resolution;
}

InnerCurveOvertakeResolution resolve_inner_curve_overtake(
  const InnerCurveOvertakeRequest & request) noexcept
{
  InnerCurveOvertakeResolution resolution;
  const bool inside_curve =
    request.pass_side_sign != 0 && request.inner_curve_pass_side != 0 &&
    request.pass_side_sign == request.inner_curve_pass_side;
  if (
    !inside_curve || request.explicit_forbidden_wp || request.cooldown_active ||
    request.emergency_brake)
  {
    return resolution;
  }

  resolution.entry_allowed =
    request.entry_enabled && !request.continuing_overtake &&
    request.soft_curve_forbidden && !request.hard_curve_forbidden &&
    request.gap_available;
  resolution.hard_entry_allowed =
    request.entry_enabled && request.hard_entry_enabled &&
    !request.continuing_overtake && request.hard_curve_forbidden &&
    request.gap_available;
  resolution.hard_continuation_allowed =
    request.hard_continuation_enabled && request.continuing_overtake &&
    request.hard_curve_forbidden && request.gap_available &&
    request.locked_target_seen;
  return resolution;
}

ActiveHardCurveContinuationResolution resolve_active_hard_curve_continuation(
  const ActiveHardCurveContinuationRequest & request)
{
  ActiveHardCurveContinuationResolution resolution;
  if (
    !request.enabled || !request.continuing_overtake || !request.pass_phase ||
    !request.locked_target_seen || !request.hard_curve_ahead ||
    request.explicit_forbidden_wp || request.cooldown_active || request.emergency_brake)
  {
    return resolution;
  }

  resolution.completion = resolve_pass_completion(request.completion);
  resolution.allowed =
    resolution.completion.feasible || request.lateral_clearance_latched;
  return resolution;
}

SideSelection select_pass_side(const SideSelectionRequest & request) noexcept
{
  if (is_configured_side(request.locked)) {
    if (is_feasible(request, request.locked)) {
      return {request.locked, SideSelectionReason::Locked};
    }
    return {PassSide::None, SideSelectionReason::LockedUnavailable};
  }

  if (!is_configured_side(request.preferred)) {
    return {PassSide::None, SideSelectionReason::InvalidPreference};
  }
  if (is_feasible(request, request.preferred)) {
    return {request.preferred, SideSelectionReason::Preferred};
  }

  const PassSide alternate = opposite_side(request.preferred);
  if (request.allow_alternate && is_feasible(request, alternate)) {
    return {alternate, SideSelectionReason::Alternate};
  }
  if (is_feasible(request, alternate)) {
    return {PassSide::None, SideSelectionReason::PreferredUnavailable};
  }
  return {PassSide::None, SideSelectionReason::NoFeasibleSide};
}

double score_overtake_side_quality(
  const OvertakeSideQualityCandidate & candidate) noexcept
{
  if (
    !is_configured_side(candidate.side) || !candidate.feasible ||
    !std::isfinite(candidate.side_clearance_m) || candidate.side_clearance_m < 0.0 ||
    !std::isfinite(candidate.corridor_width_m) || candidate.corridor_width_m < 0.0 ||
    !std::isfinite(candidate.continuous_open_distance_m) ||
    candidate.continuous_open_distance_m < 0.0 ||
    !std::isfinite(candidate.required_lateral_accel_mps2) ||
    candidate.required_lateral_accel_mps2 < 0.0)
  {
    return -std::numeric_limits<double>::infinity();
  }

  const double lateral_room_m = candidate.corridor_width_m > 1e-9 ?
    std::min(candidate.side_clearance_m, candidate.corridor_width_m) :
    candidate.side_clearance_m;
  const double continuity_bonus_m =
    0.10 * std::min(candidate.continuous_open_distance_m, 10.0);
  const double lateral_accel_penalty_m =
    0.10 * std::min(candidate.required_lateral_accel_mps2, 10.0);
  return lateral_room_m + continuity_bonus_m - lateral_accel_penalty_m;
}

OvertakeSideQualitySelection select_overtake_side_by_quality(
  const OvertakeSideQualitySelectionRequest & request) noexcept
{
  OvertakeSideQualitySelection result;
  result.left_score = score_overtake_side_quality(request.left);
  result.right_score = score_overtake_side_quality(request.right);
  const bool left_feasible = std::isfinite(result.left_score);
  const bool right_feasible = std::isfinite(result.right_score);
  const auto score_for = [&](const PassSide side) {
      return side == PassSide::Left ? result.left_score :
             side == PassSide::Right ? result.right_score :
             -std::numeric_limits<double>::infinity();
    };
  const auto feasible = [&](const PassSide side) {
      return side == PassSide::Left ? left_feasible :
             side == PassSide::Right ? right_feasible : false;
    };

  if (is_configured_side(request.locked)) {
    if (!request.allow_locked_reselection) {
      result.side = feasible(request.locked) ? request.locked : PassSide::None;
      result.reason = feasible(request.locked) ?
        SideSelectionReason::Locked : SideSelectionReason::LockedUnavailable;
      return result;
    }

    const PassSide alternate = opposite_side(request.locked);
    if (!feasible(request.locked)) {
      result.side = feasible(alternate) ? alternate : PassSide::None;
      result.reason = feasible(alternate) ?
        SideSelectionReason::LockedQualitySwitch : SideSelectionReason::LockedUnavailable;
      return result;
    }
    const double minimum_advantage = std::max(0.0, request.minimum_score_advantage);
    if (
      feasible(alternate) &&
      score_for(alternate) > score_for(request.locked) + minimum_advantage + 1e-9)
    {
      result.side = alternate;
      result.reason = SideSelectionReason::LockedQualitySwitch;
      return result;
    }
    result.side = request.locked;
    result.reason = SideSelectionReason::Locked;
    return result;
  }

  if (!is_configured_side(request.preferred)) {
    result.reason = SideSelectionReason::InvalidPreference;
    return result;
  }
  if (!left_feasible && !right_feasible) {
    result.reason = SideSelectionReason::NoFeasibleSide;
    return result;
  }
  if (left_feasible != right_feasible) {
    result.side = left_feasible ? PassSide::Left : PassSide::Right;
    result.reason = result.side == request.preferred ?
      SideSelectionReason::Preferred : SideSelectionReason::Alternate;
    return result;
  }

  const double difference = result.left_score - result.right_score;
  const double minimum_advantage = std::max(0.0, request.minimum_score_advantage);
  if (difference > minimum_advantage + 1e-9) {
    result.side = PassSide::Left;
    result.reason = SideSelectionReason::HigherQuality;
  } else if (difference < -minimum_advantage - 1e-9) {
    result.side = PassSide::Right;
    result.reason = SideSelectionReason::HigherQuality;
  } else {
    result.side = request.preferred;
    result.reason = SideSelectionReason::Preferred;
  }
  return result;
}

SideSelection select_curve_attack_side(const CurveAttackSideRequest & request) noexcept
{
  if (is_configured_side(request.locked_side)) {
    return select_pass_side(
      SideSelectionRequest{
        request.inner_side, request.locked_side,
        request.left_feasible, request.right_feasible, false});
  }

  if (
    !is_configured_side(request.inner_side) ||
    !std::isfinite(request.minimum_inner_open_distance_m) ||
    request.minimum_inner_open_distance_m < 0.0)
  {
    return {PassSide::None, SideSelectionReason::InvalidPreference};
  }

  const double inner_open_distance = request.inner_side == PassSide::Left ?
    request.left_continuous_open_distance_m : request.right_continuous_open_distance_m;
  const bool inner_sufficient =
    is_feasible(
      SideSelectionRequest{
        request.inner_side, PassSide::None,
        request.left_feasible, request.right_feasible, false},
      request.inner_side) &&
    std::isfinite(inner_open_distance) && inner_open_distance >= 0.0 &&
    inner_open_distance + 1e-9 >= request.minimum_inner_open_distance_m;
  if (inner_sufficient) {
    return {request.inner_side, SideSelectionReason::Preferred};
  }

  const PassSide outside_side = opposite_side(request.inner_side);
  const bool outside_feasible = outside_side == PassSide::Left ?
    request.left_feasible : request.right_feasible;
  if (outside_feasible) {
    return {outside_side, SideSelectionReason::Alternate};
  }
  return {PassSide::None, SideSelectionReason::NoFeasibleSide};
}

PassSide opposite_side(const PassSide side) noexcept
{
  if (side == PassSide::Left) {
    return PassSide::Right;
  }
  if (side == PassSide::Right) {
    return PassSide::Left;
  }
  return PassSide::None;
}

bool selected_pass_side_ordering_conflict(
  const bool active_shiftout, const int pass_side_sign, const bool target_seen,
  const bool target_position_jump, const double target_longitudinal_m,
  const double maximum_guard_longitudinal_m, const double target_relative_lateral_m,
  const double ordering_margin_m) noexcept
{
  if (
    !active_shiftout || pass_side_sign == 0 || !target_seen || target_position_jump ||
    !std::isfinite(target_longitudinal_m) || target_longitudinal_m <= 0.0 ||
    !std::isfinite(maximum_guard_longitudinal_m) ||
    maximum_guard_longitudinal_m < 0.0 ||
    target_longitudinal_m > maximum_guard_longitudinal_m + 1e-9 ||
    !std::isfinite(target_relative_lateral_m) ||
    !std::isfinite(ordering_margin_m) || ordering_margin_m < 0.0)
  {
    return false;
  }

  return static_cast<double>(pass_side_sign) * target_relative_lateral_m >
         ordering_margin_m + 1e-9;
}

EarlyShiftOutSideReplanResolution resolve_early_shiftout_side_replan(
  const EarlyShiftOutSideReplanRequest & request) noexcept
{
  EarlyShiftOutSideReplanResolution result;
  if (
    !request.enabled || !request.shiftout_phase || request.lateral_clearance_latched ||
    !is_configured_side(request.locked_side) ||
    !std::isfinite(request.lateral_progress_m) || request.lateral_progress_m < 0.0 ||
    !std::isfinite(request.maximum_lateral_progress_m) ||
    request.maximum_lateral_progress_m < 0.0 ||
    !std::isfinite(request.traveled_distance_m) || request.traveled_distance_m < 0.0 ||
    !std::isfinite(request.maximum_traveled_distance_m) ||
    request.maximum_traveled_distance_m < 0.0 ||
    !std::isfinite(request.candidate_stable_sec) || request.candidate_stable_sec < 0.0 ||
    !std::isfinite(request.required_stable_sec) || request.required_stable_sec < 0.0)
  {
    return result;
  }

  result.inside_switch_window =
    request.side_switch_permitted &&
    request.lateral_progress_m <= request.maximum_lateral_progress_m + 1e-9 &&
    request.traveled_distance_m <= request.maximum_traveled_distance_m + 1e-9;
  if (request.candidate_stable_sec + 1e-9 < request.required_stable_sec) {
    return result;
  }

  const bool alternate_feasible =
    request.candidate_feasible &&
    is_configured_side(request.candidate_side) &&
    request.candidate_side != request.locked_side;
  if (result.inside_switch_window && alternate_feasible) {
    result.action = EarlyShiftOutSideReplanAction::Switch;
  } else if (request.selected_side_conflict) {
    result.action = EarlyShiftOutSideReplanAction::Abort;
  }
  return result;
}

bool is_v2x_behavior_session_active(
  const bool state_tracking_enabled, const bool race_started,
  const bool start_grid_ready_rollout) noexcept
{
  return !state_tracking_enabled || race_started || start_grid_ready_rollout;
}

bool can_start_low_speed_bypass(const LowSpeedBypassCandidateRequest & request) noexcept
{
  if (
    !request.enabled || !request.candidate_vehicle_present || request.cooldown_active ||
    request.start_grid_stop_suppressed ||
    !std::isfinite(request.vehicle_speed_mps) || request.vehicle_speed_mps < 0.0 ||
    !std::isfinite(request.maximum_vehicle_speed_mps) ||
    request.maximum_vehicle_speed_mps < 0.0 ||
    request.vehicle_speed_mps > request.maximum_vehicle_speed_mps ||
    !std::isfinite(request.forward_distance_m) || request.forward_distance_m < 0.0 ||
    !std::isfinite(request.minimum_prepare_distance_m) ||
    request.minimum_prepare_distance_m < 0.0 ||
    !std::isfinite(request.maximum_entry_distance_m) ||
    request.maximum_entry_distance_m < request.minimum_prepare_distance_m)
  {
    return false;
  }

  const bool curve_policy_allows =
    !request.overtake_forbidden || request.continuing ||
    (request.ignore_soft_curve_forbidden && !request.explicit_forbidden_wp);
  return curve_policy_allows &&
         request.forward_distance_m >= request.minimum_prepare_distance_m &&
         request.forward_distance_m <= request.maximum_entry_distance_m;
}

StoppedCandidateConfirmationResult update_stopped_candidate_confirmation(
  const StoppedCandidateConfirmationRequest & request) noexcept
{
  StoppedCandidateConfirmationResult result;
  if (
    !request.candidate_present || !std::isfinite(request.observation_sec) ||
    request.previous_count < 0 || request.required_count < 1 ||
    !std::isfinite(request.maximum_observation_gap_sec) ||
    request.maximum_observation_gap_sec < 0.0)
  {
    return result;
  }

  if (
    request.same_candidate && std::isfinite(request.previous_observation_sec) &&
    request.observation_sec == request.previous_observation_sec)
  {
    result.observation_count = request.previous_count;
    result.confirmed = result.observation_count >= request.required_count;
    result.last_observation_sec = request.previous_observation_sec;
    return result;
  }

  const bool observation_contiguous =
    request.same_candidate && std::isfinite(request.previous_observation_sec) &&
    request.observation_sec > request.previous_observation_sec &&
    request.observation_sec - request.previous_observation_sec <=
    request.maximum_observation_gap_sec;
  result.observation_count = observation_contiguous ?
    std::min(request.required_count, request.previous_count + 1) : 1;
  result.confirmed = result.observation_count >= request.required_count;
  result.last_observation_sec = request.observation_sec;
  return result;
}

bool should_yield_overtake_line_to_stopped_bypass(
  const StoppedVehicleLineOwnershipRequest & request) noexcept
{
  if (
    request.low_speed_behavior_active || request.low_speed_candidate ||
    request.low_speed_direct_control_active)
  {
    return true;
  }
  const bool stopped_front =
    request.has_front_vehicle &&
    std::isfinite(request.front_distance_m) && request.front_distance_m >= 0.0 &&
    std::isfinite(request.front_speed_mps) && request.front_speed_mps >= 0.0 &&
    std::isfinite(request.maximum_stopped_speed_mps) &&
    request.maximum_stopped_speed_mps >= 0.0 &&
    std::isfinite(request.stopped_detection_distance_m) &&
    request.stopped_detection_distance_m >= 0.0 &&
    request.front_speed_mps <= request.maximum_stopped_speed_mps &&
    request.front_distance_m <= request.stopped_detection_distance_m;
  return stopped_front && !request.overtake_behavior_active;
}

PassSide select_reachable_low_speed_pass_side(
  const LowSpeedPassSideRequest & request) noexcept
{
  const auto valid = [](const LowSpeedPassSideCandidate & candidate) {
      return candidate.feasible && std::isfinite(candidate.target_lateral_m) &&
             std::isfinite(candidate.width_m) && candidate.width_m >= 0.0;
    };
  const bool left_valid = valid(request.left);
  const bool right_valid = valid(request.right);
  if (left_valid && !right_valid) {
    return PassSide::Left;
  }
  if (right_valid && !left_valid) {
    return PassSide::Right;
  }
  if (!left_valid || !std::isfinite(request.current_lateral_m)) {
    return PassSide::None;
  }

  const double left_transition =
    std::abs(request.left.target_lateral_m - request.current_lateral_m);
  const double right_transition =
    std::abs(request.right.target_lateral_m - request.current_lateral_m);
  constexpr double kSelectionTolerance = 1.0e-6;
  if (left_transition + kSelectionTolerance < right_transition) {
    return PassSide::Left;
  }
  if (right_transition + kSelectionTolerance < left_transition) {
    return PassSide::Right;
  }
  if (request.left.width_m > request.right.width_m + kSelectionTolerance) {
    return PassSide::Left;
  }
  return PassSide::Right;
}

bool has_entered_low_speed_pass_corridor(
  const double current_lateral_m, const double lower_m, const double upper_m,
  const double tolerance_m) noexcept
{
  if (
    !std::isfinite(current_lateral_m) || !std::isfinite(lower_m) ||
    !std::isfinite(upper_m) || !std::isfinite(tolerance_m) ||
    lower_m > upper_m || tolerance_m < 0.0)
  {
    return false;
  }
  return current_lateral_m >= lower_m - tolerance_m &&
         current_lateral_m <= upper_m + tolerance_m;
}

double resolve_low_speed_pass_velocity(
  const double pass_velocity_mps, const double shift_velocity_mps,
  const bool corridor_entered)
{
  validate_speed(pass_velocity_mps, "low-speed pass velocity");
  validate_speed(shift_velocity_mps, "low-speed shift velocity");
  return corridor_entered ? pass_velocity_mps :
         std::min(pass_velocity_mps, shift_velocity_mps);
}

double resolve_low_speed_direct_control_velocity(
  const LowSpeedDirectControlPhase phase,
  const double shift_velocity_mps,
  const double pass_velocity_mps,
  const double rejoin_velocity_mps,
  const double maximum_velocity_mps)
{
  validate_speed(shift_velocity_mps, "low-speed direct shift velocity");
  validate_speed(pass_velocity_mps, "low-speed direct pass velocity");
  validate_speed(rejoin_velocity_mps, "low-speed direct rejoin velocity");
  validate_speed(maximum_velocity_mps, "low-speed direct maximum velocity");
  double selected_velocity_mps = shift_velocity_mps;
  switch (phase) {
    case LowSpeedDirectControlPhase::Shift:
      selected_velocity_mps = shift_velocity_mps;
      break;
    case LowSpeedDirectControlPhase::Pass:
      selected_velocity_mps = pass_velocity_mps;
      break;
    case LowSpeedDirectControlPhase::Rejoin:
      selected_velocity_mps = rejoin_velocity_mps;
      break;
  }
  return std::min(selected_velocity_mps, maximum_velocity_mps);
}

double resolve_low_speed_shift_steering(
  const LowSpeedShiftSteeringRequest & request)
{
  if (
    !std::isfinite(request.current_lateral_m) ||
    !std::isfinite(request.current_heading_error_rad) ||
    !std::isfinite(request.target_lateral_m) ||
    !std::isfinite(request.reference_curvature_radpm) ||
    !std::isfinite(request.wheelbase_m) || request.wheelbase_m <= 0.0 ||
    !std::isfinite(request.max_steering_rad) || request.max_steering_rad < 0.0 ||
    !std::isfinite(request.lateral_gain) || request.lateral_gain < 0.0 ||
    !std::isfinite(request.heading_gain) || request.heading_gain < 0.0)
  {
    throw std::invalid_argument("invalid low-speed shift steering request");
  }

  const double lateral_error = request.current_lateral_m - request.target_lateral_m;
  const double target_curvature =
    request.reference_curvature_radpm - request.lateral_gain * lateral_error -
    request.heading_gain * request.current_heading_error_rad;
  const double target_steering = std::atan(request.wheelbase_m * target_curvature);
  return std::clamp(
    target_steering, -request.max_steering_rad, request.max_steering_rad);
}

double limit_low_speed_shift_steering_by_lateral_acceleration(
  const double target_steering_rad, const double current_speed_mps,
  const double wheelbase_m, const double maximum_lateral_acceleration_mps2,
  const double steering_command_gain)
{
  if (
    !std::isfinite(target_steering_rad) ||
    !std::isfinite(current_speed_mps) || current_speed_mps < 0.0 ||
    !std::isfinite(wheelbase_m) || wheelbase_m <= 0.0 ||
    !std::isfinite(maximum_lateral_acceleration_mps2) ||
    maximum_lateral_acceleration_mps2 < 0.0 ||
    !std::isfinite(steering_command_gain) || steering_command_gain <= 0.0)
  {
    throw std::invalid_argument("invalid low-speed lateral acceleration limit request");
  }
  if (current_speed_mps <= std::numeric_limits<double>::epsilon()) {
    return target_steering_rad;
  }

  const double maximum_tire_angle_rad = std::atan(
    wheelbase_m * maximum_lateral_acceleration_mps2 /
    (current_speed_mps * current_speed_mps));
  const double maximum_controller_angle_rad =
    maximum_tire_angle_rad / steering_command_gain;
  return std::clamp(
    target_steering_rad, -maximum_controller_angle_rad,
    maximum_controller_angle_rad);
}

bool is_low_speed_shift_complete(
  const double current_lateral_m, const double current_heading_error_rad,
  const double target_lateral_m, const double lateral_tolerance_m,
  const double heading_tolerance_rad) noexcept
{
  if (
    !std::isfinite(current_lateral_m) || !std::isfinite(current_heading_error_rad) ||
    !std::isfinite(target_lateral_m) || !std::isfinite(lateral_tolerance_m) ||
    !std::isfinite(heading_tolerance_rad) || lateral_tolerance_m < 0.0 ||
    heading_tolerance_rad < 0.0)
  {
    return false;
  }
  return std::abs(current_lateral_m - target_lateral_m) <= lateral_tolerance_m &&
         std::abs(current_heading_error_rad) <= heading_tolerance_rad;
}

bool should_begin_low_speed_shift_rejoin(
  const bool has_front_vehicle, const bool has_side_vehicle,
  const bool has_clearance_vehicle, const double clear_duration_sec,
  const double required_clear_duration_sec) noexcept
{
  if (
    !std::isfinite(clear_duration_sec) || clear_duration_sec < 0.0 ||
    !std::isfinite(required_clear_duration_sec) || required_clear_duration_sec < 0.0)
  {
    return false;
  }
  return !has_front_vehicle && !has_side_vehicle && !has_clearance_vehicle &&
         clear_duration_sec >= required_clear_duration_sec;
}

bool should_release_low_speed_shift_control(
  const bool pose_settled, const bool has_front_vehicle,
  const bool has_side_vehicle, const bool has_clearance_vehicle,
  const double clear_duration_sec, const double required_clear_duration_sec) noexcept
{
  return pose_settled && should_begin_low_speed_shift_rejoin(
    has_front_vehicle, has_side_vehicle, has_clearance_vehicle,
    clear_duration_sec, required_clear_duration_sec);
}

bool can_hold_committed_execution_after_behavior_drop(
  const CommittedExecutionContinuityRequest & request) noexcept
{
  return request.active_execution_phase &&
         request.target_progress_continuous &&
         request.target_ahead &&
         !request.target_pass_side_intrusion &&
         !request.live_execution_corridor_blocked &&
         !request.explicit_forbidden_waypoint &&
         !request.emergency_front_risk;
}

ContinuityAction resolve_target_continuity(const ContinuityRequest & request)
{
  if (std::isnan(request.target_age_sec) || request.target_age_sec < 0.0) {
    throw std::invalid_argument("target age must be finite and non-negative");
  }
  if (!std::isfinite(request.target_hold_sec) || request.target_hold_sec < 0.0) {
    throw std::invalid_argument("target hold must be finite and non-negative");
  }
  if (request.solver_recovery_requested || request.target_position_jump) {
    return ContinuityAction::Recovery;
  }
  if (!std::isfinite(request.target_age_sec)) {
    return ContinuityAction::Recovery;
  }
  if (request.rear_clear_confirmed && !request.side_vehicle_present) {
    return ContinuityAction::Return;
  }
  if (
    request.rear_clear_observed ||
    (request.target_seen && request.target_not_ahead) ||
    (!request.target_seen && request.target_age_sec <= request.target_hold_sec) ||
    (request.target_seen && request.active_execution_latched))
  {
    return ContinuityAction::Hold;
  }
  return ContinuityAction::Recovery;
}

bool can_reacquire_during_return(const ReacquireRequest & request) noexcept
{
  return request.enabled && request.stable_target_id && request.same_target &&
         request.same_side && request.gap_available && request.execution_allowed &&
         std::isfinite(request.return_elapsed_sec) && request.return_elapsed_sec >= 0.0 &&
         std::isfinite(request.reacquire_window_sec) && request.reacquire_window_sec >= 0.0 &&
         request.return_elapsed_sec <= request.reacquire_window_sec &&
         std::isfinite(request.return_progress) && request.return_progress >= 0.0 &&
         std::isfinite(request.max_return_progress) && request.max_return_progress >= 0.0 &&
         request.return_progress <= request.max_return_progress;
}

bool can_reacquire_during_recovery(const RecoveryReacquireRequest & request) noexcept
{
  return request.enabled && request.phase_hold_elapsed && request.stable_target_id &&
         request.same_target && request.target_progress_continuous && request.same_side &&
         !request.target_rear_clear && request.gap_available && request.execution_allowed &&
         request.solver_ready;
}

ForwardDistanceResolution integrate_forward_distance(const ForwardDistanceRequest & request)
{
  if (
    !std::isfinite(request.accumulated_distance_m) ||
    request.accumulated_distance_m < 0.0)
  {
    throw std::invalid_argument("accumulated forward distance must be finite and non-negative");
  }
  if (
    !std::isfinite(request.max_observation_gap_sec) ||
    request.max_observation_gap_sec <= 0.0)
  {
    throw std::invalid_argument("maximum observation gap must be finite and positive");
  }

  ForwardDistanceResolution resolution{request.accumulated_distance_m, false};
  if (
    !std::isfinite(request.forward_speed_mps) || !std::isfinite(request.delta_sec) ||
    request.delta_sec < 0.0 || request.delta_sec > request.max_observation_gap_sec)
  {
    return resolution;
  }

  resolution.accumulated_distance_m +=
    std::max(0.0, request.forward_speed_mps) * request.delta_sec;
  resolution.observation_accepted = true;
  return resolution;
}

CommittedPassProgressWatchdogResolution update_committed_pass_progress_watchdog(
  const CommittedPassProgressWatchdogRequest & request) noexcept
{
  CommittedPassProgressWatchdogResolution resolution{
    request.best_target_longitudinal_m,
    request.progress_checkpoint_distance_m,
    false,
    false,
    false};
  if (
    !request.active ||
    !std::isfinite(request.target_longitudinal_m) ||
    !std::isfinite(request.traveled_distance_m) ||
    request.traveled_distance_m < 0.0 ||
    !std::isfinite(request.minimum_progress_m) ||
    request.minimum_progress_m <= 0.0 ||
    !std::isfinite(request.maximum_without_progress_distance_m) ||
    request.maximum_without_progress_distance_m <= 0.0)
  {
    return resolution;
  }

  resolution.observation_accepted = true;
  const bool previous_state_valid =
    std::isfinite(request.best_target_longitudinal_m) &&
    std::isfinite(request.progress_checkpoint_distance_m) &&
    request.progress_checkpoint_distance_m >= 0.0 &&
    request.traveled_distance_m >= request.progress_checkpoint_distance_m;
  if (!previous_state_valid) {
    resolution.best_target_longitudinal_m = request.target_longitudinal_m;
    resolution.progress_checkpoint_distance_m = request.traveled_distance_m;
    return resolution;
  }

  if (
    request.target_longitudinal_m <=
    request.best_target_longitudinal_m - request.minimum_progress_m)
  {
    resolution.best_target_longitudinal_m = request.target_longitudinal_m;
    resolution.progress_checkpoint_distance_m = request.traveled_distance_m;
    resolution.progressed = true;
    return resolution;
  }

  resolution.timed_out =
    request.traveled_distance_m - request.progress_checkpoint_distance_m >=
    request.maximum_without_progress_distance_m;
  return resolution;
}

RecoveryVelocityLimitResolution resolve_recovery_velocity_limit(
  const RecoveryVelocityLimitRequest & request)
{
  validate_speed(request.configured_velocity_limit_mps, "Recovery velocity limit");
  if (request.configured_velocity_limit_mps <= 0.0) {
    throw std::invalid_argument("Recovery velocity limit must be positive");
  }
  if (
    request.moving_follow_profile_available &&
    (std::isnan(request.moving_follow_velocity_limit_mps) ||
    request.moving_follow_velocity_limit_mps < 0.0))
  {
    throw std::invalid_argument(
            "Recovery moving Follow velocity limit must be non-negative or positive infinity");
  }
  if (request.solver_recovery_active || !request.moving_follow_profile_available) {
    return {request.configured_velocity_limit_mps, false};
  }
  return {request.moving_follow_velocity_limit_mps, true};
}

RecoveryPolicyResolution resolve_recovery_policy(const RecoveryPolicyRequest & request)
{
  validate_speed(request.configured_velocity_limit_mps, "Recovery velocity limit");
  if (request.configured_velocity_limit_mps <= 0.0) {
    throw std::invalid_argument("Recovery velocity limit must be positive");
  }
  if (!std::isfinite(request.target_distance_m) || request.target_distance_m <= 0.0) {
    throw std::invalid_argument("Recovery target distance must be finite and positive");
  }
  if (!std::isfinite(request.lateral_completion_m) || request.lateral_completion_m < 0.0) {
    throw std::invalid_argument("Recovery lateral completion must be finite and non-negative");
  }
  if (!std::isfinite(request.stall_timeout_sec) || request.stall_timeout_sec <= 0.0) {
    throw std::invalid_argument("Recovery stall timeout must be finite and positive");
  }
  if (!std::isfinite(request.timeout_sec) || request.timeout_sec <= 0.0) {
    throw std::invalid_argument("Recovery timeout must be finite and positive");
  }
  if (request.stall_timeout_sec > request.timeout_sec) {
    throw std::invalid_argument("Recovery stall timeout must not exceed total timeout");
  }

  RecoveryPolicyResolution resolution{request.configured_velocity_limit_mps,
    RecoveryExitReason::Active};
  if (
    !std::isfinite(request.elapsed_sec) || request.elapsed_sec < 0.0 ||
    !std::isfinite(request.traveled_distance_m) || request.traveled_distance_m < 0.0 ||
    !std::isfinite(request.lateral_error_m) ||
    !std::isfinite(request.stalled_sec) || request.stalled_sec < 0.0)
  {
    resolution.exit_reason = RecoveryExitReason::InvalidObservation;
    return resolution;
  }
  if (std::abs(request.lateral_error_m) <= request.lateral_completion_m) {
    resolution.exit_reason = RecoveryExitReason::LateralComplete;
    return resolution;
  }
  if (request.traveled_distance_m >= request.target_distance_m) {
    resolution.exit_reason = RecoveryExitReason::DistanceComplete;
    return resolution;
  }
  if (request.stalled_sec >= request.stall_timeout_sec) {
    resolution.exit_reason = RecoveryExitReason::Stalled;
    return resolution;
  }
  if (request.elapsed_sec >= request.timeout_sec) {
    resolution.exit_reason = RecoveryExitReason::TimedOut;
  }
  return resolution;
}

StallWatchdogResolution update_stall_watchdog(const StallWatchdogRequest & request)
{
  if (!std::isfinite(request.speed_threshold_mps) || request.speed_threshold_mps < 0.0) {
    throw std::invalid_argument("stall speed threshold must be finite and non-negative");
  }
  if (!std::isfinite(request.timeout_sec) || request.timeout_sec <= 0.0) {
    throw std::invalid_argument("stall timeout must be finite and positive");
  }
  if (
    !std::isfinite(request.max_observation_gap_sec) ||
    request.max_observation_gap_sec <= 0.0)
  {
    throw std::invalid_argument("stall observation gap must be finite and positive");
  }

  const double inactive = std::numeric_limits<double>::quiet_NaN();
  StallWatchdogResolution resolution{inactive, inactive, 0.0, false, false};
  if (!std::isfinite(request.now_sec) || !std::isfinite(request.speed_mps)) {
    return resolution;
  }

  resolution.update_sec = request.now_sec;
  resolution.observation_accepted = true;
  if (!request.active || std::abs(request.speed_mps) > request.speed_threshold_mps) {
    return resolution;
  }

  const bool observation_continuous =
    std::isfinite(request.previous_update_sec) &&
    request.now_sec >= request.previous_update_sec &&
    request.now_sec - request.previous_update_sec <= request.max_observation_gap_sec;
  const bool previous_stall_valid =
    std::isfinite(request.stall_since_sec) &&
    request.stall_since_sec <= request.now_sec;
  resolution.stall_since_sec = observation_continuous && previous_stall_valid ?
    request.stall_since_sec : request.now_sec;
  resolution.stalled_sec = std::max(0.0, request.now_sec - resolution.stall_since_sec);
  resolution.timed_out = resolution.stalled_sec >= request.timeout_sec;
  return resolution;
}

const char * to_string(const RecoveryExitReason reason) noexcept
{
  switch (reason) {
    case RecoveryExitReason::Active:
      return "active";
    case RecoveryExitReason::DistanceComplete:
      return "distance complete";
    case RecoveryExitReason::LateralComplete:
      return "lateral complete";
    case RecoveryExitReason::Stalled:
      return "stalled";
    case RecoveryExitReason::TimedOut:
      return "timed out";
    case RecoveryExitReason::InvalidObservation:
      return "invalid observation";
  }
  return "unknown";
}

FrontHazardHoldResolution update_front_hazard_hold(const FrontHazardHoldRequest & request)
{
  if (!std::isfinite(request.now_sec)) {
    throw std::invalid_argument("Front hazard hold time must be finite");
  }
  if (!std::isfinite(request.hold_sec) || request.hold_sec < 0.0) {
    throw std::invalid_argument("Front hazard hold duration must be finite and non-negative");
  }
  if (!std::isfinite(request.current_until_sec)) {
    throw std::invalid_argument("Front hazard hold deadline must be finite");
  }

  if (!request.enabled || request.target_rear_clear || request.target_observed_safe) {
    return {false, request.now_sec, 0.0};
  }

  double until_sec = request.current_until_sec;
  if (request.hazard_observed) {
    until_sec = std::max(until_sec, request.now_sec + request.hold_sec);
  }
  const bool active = request.now_sec < until_sec;
  return {
    active,
    active ? until_sec : request.now_sec,
    active ? std::max(0.0, until_sec - request.now_sec) : 0.0};
}

FrontDangerAction resolve_front_danger_action(const FrontDangerActionRequest & request)
{
  if (
    !std::isfinite(request.moving_front_speed_threshold_mps) ||
    request.moving_front_speed_threshold_mps < 0.0)
  {
    throw std::invalid_argument(
            "Moving-front speed threshold must be finite and non-negative");
  }
  if (
    !std::isfinite(request.moving_front_hard_distance_m) ||
    request.moving_front_hard_distance_m < 0.0)
  {
    throw std::invalid_argument(
            "Moving-front hard distance must be finite and non-negative");
  }
  if (request.emergency_brake) {
    return FrontDangerAction::SafetyBrake;
  }
  const bool moving_front =
    std::isfinite(request.front_speed_mps) && request.front_speed_mps >= 0.0 &&
    request.front_speed_mps > request.moving_front_speed_threshold_mps;
  if (
    moving_front && request.moving_front_hard_distance_m > 0.0 &&
    (!std::isfinite(request.front_distance_m) || request.front_distance_m < 0.0 ||
    request.front_distance_m <= request.moving_front_hard_distance_m))
  {
    return FrontDangerAction::SafetyBrake;
  }
  if (!request.inside_stopping_distance) {
    return FrontDangerAction::None;
  }
  if (moving_front) {
    return FrontDangerAction::RelativeSpeedLimit;
  }
  return FrontDangerAction::SafetyBrake;
}

const char * to_string(const FrontDangerAction action) noexcept
{
  switch (action) {
    case FrontDangerAction::None:
      return "None";
    case FrontDangerAction::RelativeSpeedLimit:
      return "RelativeSpeedLimit";
    case FrontDangerAction::SafetyBrake:
      return "SafetyBrake";
  }
  return "Unknown";
}

double arm_solver_cooldown(const SolverCooldownRequest & request)
{
  if (!std::isfinite(request.now_sec)) {
    throw std::invalid_argument("Solver cooldown time must be finite");
  }
  if (!std::isfinite(request.duration_sec) || request.duration_sec < 0.0) {
    throw std::invalid_argument("Solver cooldown duration must be finite and non-negative");
  }
  if (
    std::isnan(request.current_until_sec) ||
    request.current_until_sec == std::numeric_limits<double>::infinity())
  {
    throw std::invalid_argument("Solver cooldown deadline must be finite or negative infinity");
  }
  if (request.duration_sec == 0.0) {
    return request.current_until_sec;
  }

  const double candidate_until_sec = request.now_sec + request.duration_sec;
  if (!std::isfinite(candidate_until_sec)) {
    throw std::invalid_argument("Solver cooldown deadline overflowed");
  }
  return std::max(request.current_until_sec, candidate_until_sec);
}

bool is_solver_cooldown_active(
  const double now_sec, const double cooldown_until_sec) noexcept
{
  return std::isfinite(now_sec) && std::isfinite(cooldown_until_sec) &&
         now_sec < cooldown_until_sec;
}

SolverReentryGateResolution update_solver_reentry_gate(
  const SolverReentryGateRequest & request)
{
  if (request.consecutive_successes < 0) {
    throw std::invalid_argument("Solver re-entry success count must be non-negative");
  }
  if (request.required_successes <= 0) {
    throw std::invalid_argument("Solver re-entry required successes must be positive");
  }
  if (request.arm) {
    return {true, 0, false};
  }
  if (!request.blocked) {
    return {false, 0, false};
  }
  if (!request.solver_succeeded) {
    return {true, 0, false};
  }

  const int successes =
    request.consecutive_successes >= request.required_successes ?
    request.required_successes : request.consecutive_successes + 1;
  if (!request.cooldown_active && successes >= request.required_successes) {
    return {false, 0, true};
  }
  return {true, successes, false};
}

double rate_limit_solver_fallback_steering_toward_neutral(
  const SolverFallbackSteeringRequest & request)
{
  if (!std::isfinite(request.current_steering_rad)) {
    throw std::invalid_argument("Solver fallback steering must be finite");
  }
  if (!std::isfinite(request.max_steering_rad) || request.max_steering_rad < 0.0) {
    throw std::invalid_argument("Solver fallback maximum steering must be finite and non-negative");
  }
  if (!std::isfinite(request.steer_rate_radps) || request.steer_rate_radps < 0.0) {
    throw std::invalid_argument("Solver fallback steering rate must be finite and non-negative");
  }
  if (!std::isfinite(request.step_sec) || request.step_sec < 0.0) {
    throw std::invalid_argument("Solver fallback time step must be finite and non-negative");
  }

  const double steering = std::clamp(
    request.current_steering_rad, -request.max_steering_rad, request.max_steering_rad);
  const double max_step = request.steer_rate_radps * request.step_sec;
  if (!std::isfinite(max_step)) {
    throw std::invalid_argument("Solver fallback steering step overflowed");
  }
  if (std::abs(steering) <= max_step) {
    return 0.0;
  }
  return steering - std::copysign(max_step, steering);
}

bool should_neutralize_solver_fallback_steering(
  const SolverFallbackNeutralizationRequest & request)
{
  if (request.consecutive_failures < 0) {
    throw std::invalid_argument("Solver fallback failure count must be non-negative");
  }
  if (request.steering_hold_cycles < 0) {
    throw std::invalid_argument("Solver fallback steering hold cycles must be non-negative");
  }
  return request.force_neutralize ||
         request.consecutive_failures > request.steering_hold_cycles;
}

bool can_try_unvalidated_overtake_fallback(
  const UnvalidatedOvertakeFallbackRequest & request) noexcept
{
  return !request.start_grid_breakout_attempt &&
         !request.geometric_gap_available &&
         request.side_clearance_available &&
         !request.emergency_brake;
}

OffsetCurveFeasibilityResult evaluate_offset_curve_feasibility(
  const OffsetCurveFeasibilityRequest & request)
{
  if (
    !std::isfinite(request.reference_curvature_radpm) ||
    !std::isfinite(request.lateral_offset_m) ||
    !std::isfinite(request.max_abs_curvature_radpm) ||
    request.max_abs_curvature_radpm <= 0.0 ||
    !std::isfinite(request.min_frenet_denominator) ||
    request.min_frenet_denominator <= 0.0)
  {
    throw std::invalid_argument("Invalid offset-curve feasibility request");
  }

  const double denominator =
    1.0 - request.reference_curvature_radpm * request.lateral_offset_m;
  if (denominator <= request.min_frenet_denominator) {
    return {false, std::numeric_limits<double>::infinity(), denominator};
  }
  const double offset_curvature = request.reference_curvature_radpm / denominator;
  return {
    std::abs(offset_curvature) <= request.max_abs_curvature_radpm,
    offset_curvature,
    denominator};
}

double score_start_grid_corridor(const StartGridCorridorScoreRequest & request)
{
  if (
    !std::isfinite(request.corridor_center_ey) ||
    !std::isfinite(request.corridor_width) || request.corridor_width < 0.0 ||
    !std::isfinite(request.ego_ey))
  {
    throw std::invalid_argument("Start-grid corridor geometry must be finite and non-negative");
  }
  return 10.0 * std::abs(request.corridor_center_ey - request.ego_ey) -
         0.1 * request.corridor_width;
}

double conservative_rectangle_lateral_half_extent(
  const double length_m, const double width_m)
{
  if (
    !std::isfinite(length_m) || length_m < 0.0 ||
    !std::isfinite(width_m) || width_m < 0.0)
  {
    throw std::invalid_argument(
            "Rectangle length and width must be finite and non-negative");
  }
  return 0.5 * std::hypot(length_m, width_m);
}

WallCorridorGeometryResult evaluate_wall_corridor_geometry(
  const WallCorridorGeometryRequest & request)
{
  if (
    !std::isfinite(request.lower) || !std::isfinite(request.upper) ||
    request.upper < request.lower ||
    !std::isfinite(request.vehicle_extra_inflation_m) ||
    request.vehicle_extra_inflation_m < 0.0 ||
    !std::isfinite(request.wall_clearance_m) || request.wall_clearance_m < 0.0 ||
    !std::isfinite(request.minimum_width_m) || request.minimum_width_m < 0.0)
  {
    throw std::invalid_argument(
            "Wall-corridor bounds and margins must be finite, ordered, and non-negative");
  }

  double lower = request.lower;
  double upper = request.upper;
  const bool wall_vehicle_corridor =
    request.lower_is_vehicle != request.upper_is_vehicle;
  if (!request.lower_is_vehicle) {
    lower += request.wall_clearance_m;
  } else if (wall_vehicle_corridor) {
    lower += request.vehicle_extra_inflation_m;
  }
  if (!request.upper_is_vehicle) {
    upper -= request.wall_clearance_m;
  } else if (wall_vehicle_corridor) {
    upper -= request.vehicle_extra_inflation_m;
  }

  const bool feasible = upper >= lower && upper - lower >= request.minimum_width_m;
  return {feasible, lower, upper};
}

bool is_start_grid_boundary_candidate(const StartGridBoundaryCandidateRequest & request)
{
  if (
    !std::isfinite(request.forward_distance_m) ||
    !std::isfinite(request.lookbehind_distance_m) || request.lookbehind_distance_m < 0.0 ||
    !std::isfinite(request.lookahead_distance_m) || request.lookahead_distance_m < 0.0)
  {
    throw std::invalid_argument(
            "Start-grid boundary progress and windows must be finite and non-negative");
  }
  return request.forward_distance_m >= -request.lookbehind_distance_m &&
         request.forward_distance_m <= request.lookahead_distance_m;
}

bool is_inter_vehicle_corridor_rear_clear(
  const InterVehicleRearClearRequest & request)
{
  if (
    !std::isfinite(request.return_clear_distance_m) ||
    request.return_clear_distance_m < 0.0)
  {
    throw std::invalid_argument(
            "Inter-vehicle return-clear distance must be finite and non-negative");
  }
  return request.lower_vehicle_seen && request.upper_vehicle_seen &&
         std::isfinite(request.lower_longitudinal_m) &&
         std::isfinite(request.upper_longitudinal_m) &&
         request.lower_longitudinal_m <= -request.return_clear_distance_m &&
         request.upper_longitudinal_m <= -request.return_clear_distance_m;
}

}  // namespace multi_purpose_mpc_ros::v2x_overtake_core
