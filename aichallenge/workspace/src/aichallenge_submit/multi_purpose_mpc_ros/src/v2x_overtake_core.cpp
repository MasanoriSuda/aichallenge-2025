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

double predict_lateral_with_velocity_decay(
  const double initial_lateral_m, const double lateral_velocity_mps,
  const double time_sec, const double prediction_horizon_sec,
  const double decay_time_sec) noexcept
{
  const double prediction_time = std::min(
    std::max(0.0, time_sec), std::max(0.0, prediction_horizon_sec));
  if (std::isfinite(decay_time_sec) && decay_time_sec > 1e-6) {
    return initial_lateral_m + lateral_velocity_mps * decay_time_sec *
           (1.0 - std::exp(-prediction_time / decay_time_sec));
  }
  return initial_lateral_m + lateral_velocity_mps * prediction_time;
}

}  // namespace

bool V2XPeerIdentityTracker::observe_valid_message(
  const std::vector<std::string> & vehicle_ids)
{
  std::unordered_set<std::string> validated_ids;
  for (const auto & vehicle_id : vehicle_ids) {
    if (vehicle_id.empty() || !validated_ids.emplace(vehicle_id).second) {
      return false;
    }
  }
  learned_vehicle_ids_.insert(validated_ids.begin(), validated_ids.end());
  return true;
}

bool V2XPeerIdentityTracker::is_complete(
  const std::vector<std::string> & vehicle_ids) const
{
  if (
    learned_vehicle_ids_.empty() ||
    vehicle_ids.size() != learned_vehicle_ids_.size())
  {
    return false;
  }

  std::unordered_set<std::string> observed_ids;
  for (const auto & vehicle_id : vehicle_ids) {
    if (
      vehicle_id.empty() || !observed_ids.emplace(vehicle_id).second ||
      learned_vehicle_ids_.find(vehicle_id) == learned_vehicle_ids_.end())
    {
      return false;
    }
  }
  return true;
}

std::size_t V2XPeerIdentityTracker::learned_vehicle_count() const noexcept
{
  return learned_vehicle_ids_.size();
}

void V2XPeerIdentityTracker::reset() noexcept
{
  learned_vehicle_ids_.clear();
}

bool is_v2x_receipt_age_fresh(
  const double age_sec, const double timeout_sec,
  const double future_tolerance_sec) noexcept
{
  return
    std::isfinite(age_sec) &&
    std::isfinite(timeout_sec) && timeout_sec >= 0.0 &&
    std::isfinite(future_tolerance_sec) && future_tolerance_sec >= 0.0 &&
    age_sec >= -future_tolerance_sec && age_sec <= timeout_sec;
}

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
  const bool committed_pass_speed_hold =
    request.committed_pass_speed_hold_allowed &&
    request.lateral_separation_release_active &&
    request.lateral_separation_above_reapply_threshold;
  if (
    !request.active_execution_phase || !request.target_seen ||
    !std::isfinite(request.target_longitudinal_m) ||
    (!request.lateral_complete && !committed_pass_speed_hold))
  {
    return false;
  }
  const bool lateral_release =
    request.lateral_separation_clear ||
    (request.lateral_separation_release_active &&
    request.lateral_separation_above_reapply_threshold);
  const bool constrained_horizon_initial_release =
    request.constrained_horizon_release_allowed &&
    request.lateral_separation_clear;
  const bool constrained_horizon_release_hold =
    request.constrained_horizon_release_allowed &&
    request.lateral_separation_release_active &&
    request.lateral_separation_above_reapply_threshold;
  if (
    !request.execution_horizon_unconstrained &&
    !constrained_horizon_initial_release &&
    !constrained_horizon_release_hold &&
    !committed_pass_speed_hold)
  {
    return false;
  }
  return lateral_release || request.target_longitudinal_m <= 0.0;
}

bool should_apply_committed_pass_speed_floor(
  const CommittedPassSpeedFloorRequest & request) noexcept
{
  const bool physically_committed_pass =
    request.pass_phase &&
    request.lateral_complete &&
    request.front_cap_released &&
    request.lateral_exclusion_latched &&
    request.lateral_separation_above_reapply_threshold;
  // ShiftOut can remain active after the kart has physically moved clear because its line goal
  // and traveled-distance completion checks have not both converged yet. Preserve progress only
  // while full *current* separation is observed; this deliberately does not release the cap or
  // rely on the historical Pass latch.
  const bool physically_cleared_shiftout =
    request.shiftout_phase &&
    request.current_lateral_separation_clear &&
    request.lateral_separation_above_reapply_threshold &&
    !request.target_position_jump;
  return
    request.enabled &&
    (physically_committed_pass || physically_cleared_shiftout) &&
    request.target_seen &&
    request.execution_path_physically_feasible &&
    !request.actual_wall_contact &&
    std::isfinite(request.target_speed_mps) &&
    std::isfinite(request.slow_target_max_speed_mps) &&
    request.slow_target_max_speed_mps >= 0.0 &&
    request.target_speed_mps <= request.slow_target_max_speed_mps &&
    std::isfinite(request.configured_min_speed_mps) &&
    request.configured_min_speed_mps > 0.0;
}

bool course_frame_body_footprints_remain_separated(
  const CourseFrameFootprintSweepRequest & request) noexcept
{
  if (
    !std::isfinite(request.current_longitudinal_m) ||
    !std::isfinite(request.current_lateral_m) ||
    !std::isfinite(request.predicted_longitudinal_m) ||
    !std::isfinite(request.predicted_lateral_m) ||
    !std::isfinite(request.longitudinal_clearance_m) ||
    !std::isfinite(request.lateral_clearance_m) ||
    request.longitudinal_clearance_m <= 0.0 ||
    request.lateral_clearance_m <= 0.0)
  {
    return false;
  }

  // Find the open time interval where one relative coordinate lies inside its body extent.
  // The intersection of both coordinate intervals is a swept body overlap. Strict interval
  // comparison deliberately treats an exact boundary touch as separated.
  const auto inside_interval = [](
      const double current, const double predicted, const double clearance,
      double & enter, double & exit) noexcept
    {
      const double delta = predicted - current;
      if (std::abs(delta) <= std::numeric_limits<double>::epsilon()) {
        if (std::abs(current) >= clearance) {
          return false;
        }
        enter = -std::numeric_limits<double>::infinity();
        exit = std::numeric_limits<double>::infinity();
        return true;
      }
      const double first = (-clearance - current) / delta;
      const double second = (clearance - current) / delta;
      enter = std::min(first, second);
      exit = std::max(first, second);
      return true;
    };

  double longitudinal_enter = 0.0;
  double longitudinal_exit = 0.0;
  if (!inside_interval(
      request.current_longitudinal_m, request.predicted_longitudinal_m,
      request.longitudinal_clearance_m, longitudinal_enter, longitudinal_exit))
  {
    return true;
  }
  double lateral_enter = 0.0;
  double lateral_exit = 0.0;
  if (!inside_interval(
      request.current_lateral_m, request.predicted_lateral_m,
      request.lateral_clearance_m, lateral_enter, lateral_exit))
  {
    return true;
  }

  const double overlap_enter = std::max({0.0, longitudinal_enter, lateral_enter});
  const double overlap_exit = std::min({1.0, longitudinal_exit, lateral_exit});
  return overlap_enter >= overlap_exit;
}

PredictedFootprintOverlapConfirmation update_predicted_footprint_overlap_confirmation(
  const PredictedFootprintOverlapConfirmationRequest & request) noexcept
{
  PredictedFootprintOverlapConfirmation result;
  if (
    !request.monitor_active || !std::isfinite(request.now_sec) ||
    !std::isfinite(request.confirm_sec) || request.confirm_sec < 0.0)
  {
    return result;
  }

  result.overlap_since_sec =
    std::isfinite(request.overlap_since_sec) &&
    request.overlap_since_sec <= request.now_sec ?
    request.overlap_since_sec : request.now_sec;
  result.elapsed_sec = std::max(0.0, request.now_sec - result.overlap_since_sec);
  result.confirmed = result.elapsed_sec + 1e-9 >= request.confirm_sec;
  return result;
}

CommittedPassPolicyResolution resolve_committed_pass_policy(
  const CommittedPassPolicyRequest & request) noexcept
{
  CommittedPassPolicyResolution resolution;
  resolution.active_execution = request.shiftout_phase || request.pass_phase;
  const bool minimum_motion_base_guard =
    resolution.active_execution && request.minimum_motion_corridor_active &&
    request.target_seen && !request.locked_target_position_jump &&
    !request.actual_wall_contact;
  const bool minimum_motion_pass_guard =
    request.pass_phase && minimum_motion_base_guard;
  const bool minimum_motion_shiftout_guard =
    request.shiftout_phase && request.validated_frozen_plan &&
    minimum_motion_base_guard;
  const bool minimum_motion_sweep_clear =
    request.current_body_footprints_separated &&
    request.footprint_prediction_valid &&
    request.predicted_body_footprint_sweep_separated;
  // Robust separation implies physical separation.  Keep that implication in
  // the pure policy so existing callers remain fail-safe while allowing the
  // controller to provide the wider physical-only hold envelope explicitly.
  const bool current_body_physically_separated =
    request.current_body_footprints_physically_separated ||
    request.current_body_footprints_separated;
  const bool predicted_sweep_physically_separated =
    request.predicted_body_footprint_sweep_physically_separated ||
    request.predicted_body_footprint_sweep_separated;
  const bool minimum_motion_physical_sweep_clear =
    current_body_physically_separated && request.footprint_prediction_valid &&
    predicted_sweep_physically_separated;
  const bool minimum_motion_side_by_side_geometry =
    request.current_body_footprints_separated &&
    std::isfinite(request.target_longitudinal_m) &&
    std::isfinite(request.body_longitudinal_clearance_m) &&
    request.body_longitudinal_clearance_m > 0.0 &&
    request.target_longitudinal_m <= request.body_longitudinal_clearance_m;
  resolution.minimum_motion_side_by_side_escape_active =
    minimum_motion_pass_guard && minimum_motion_side_by_side_geometry &&
    (request.prior_front_cap_release_active ||
    (request.lateral_complete && request.execution_path_physically_feasible));
  resolution.minimum_motion_predicted_overlap_grace_active =
    minimum_motion_pass_guard && request.prior_front_cap_release_active &&
    current_body_physically_separated &&
    request.footprint_prediction_valid &&
    !predicted_sweep_physically_separated &&
    !request.predicted_body_footprint_overlap_confirmed &&
    !resolution.minimum_motion_side_by_side_escape_active;
  resolution.minimum_motion_shiftout_predicted_overlap_grace_active =
    minimum_motion_shiftout_guard && request.prior_front_cap_release_active &&
    request.execution_path_physically_feasible &&
    request.current_body_footprints_separated &&
    request.footprint_prediction_valid &&
    !request.predicted_body_footprint_sweep_separated &&
    !request.predicted_body_footprint_overlap_confirmed;
  // A single V2X sample on the body-boundary must not immediately hand
  // longitudinal ownership back to Follow. This grace is hold-only and is
  // available solely to an already released competition-simulation Pass.
  // Initial acquisition still requires a currently separated footprint sweep.
  resolution.minimum_motion_current_overlap_grace_active =
    request.committed_pass_attack_mode_enabled &&
    minimum_motion_pass_guard && request.prior_front_cap_release_active &&
    !current_body_physically_separated &&
    !request.current_body_footprint_overlap_confirmed &&
    request.footprint_prediction_valid &&
    request.execution_path_physically_feasible;
  // Competition-simulation attack mode is deliberately hold-only. It cannot
  // acquire an initial release from an unsafe prediction. Once the validated
  // Pass has been released, however, a predicted future overlap alone must not
  // hand speed ownership back to Follow. Current 2D overlap, wall contact,
  // target discontinuity and an infeasible execution path still revoke it.
  resolution.minimum_motion_attack_hold_active =
    request.committed_pass_attack_mode_enabled &&
    minimum_motion_pass_guard && request.prior_front_cap_release_active &&
    current_body_physically_separated &&
    request.execution_path_physically_feasible;
  resolution.minimum_motion_physical_clear_hold_active =
    minimum_motion_pass_guard && request.prior_front_cap_release_active &&
    request.execution_path_physically_feasible &&
    minimum_motion_physical_sweep_clear;
  resolution.minimum_motion_shiftout_release_allowed =
    minimum_motion_shiftout_guard &&
    request.execution_path_physically_feasible && minimum_motion_sweep_clear;
  resolution.minimum_motion_footprint_release_allowed =
    minimum_motion_pass_guard && request.lateral_complete &&
    request.execution_path_physically_feasible && minimum_motion_sweep_clear;
  const bool minimum_motion_shiftout_hold =
    minimum_motion_shiftout_guard &&
    request.execution_path_physically_feasible &&
    (minimum_motion_sweep_clear ||
    resolution.minimum_motion_shiftout_predicted_overlap_grace_active);
  const bool minimum_motion_pass_hold =
    minimum_motion_pass_guard &&
    (resolution.minimum_motion_physical_clear_hold_active ||
    ((current_body_physically_separated &&
    (minimum_motion_sweep_clear ||
    resolution.minimum_motion_side_by_side_escape_active ||
    resolution.minimum_motion_predicted_overlap_grace_active ||
    resolution.minimum_motion_attack_hold_active)) ||
    resolution.minimum_motion_current_overlap_grace_active));
  resolution.minimum_motion_footprint_hold_active =
    request.prior_front_cap_release_active &&
    (minimum_motion_shiftout_hold || minimum_motion_pass_hold);
  resolution.constrained_horizon_release_allowed =
    request.pass_phase &&
    request.lateral_exclusion_latched &&
    request.execution_path_physically_feasible &&
    !request.actual_wall_contact;
  // Horizon feasibility remains mandatory to acquire a release, but it must not revoke an
  // already committed, body-clear Pass. Path limits and the Recovery FSM continue to own wall
  // and lateral-acceleration safety independently from this locked-target speed cap.
  const bool legacy_committed_pass_speed_hold_allowed =
    request.pass_phase &&
    request.lateral_exclusion_latched &&
    !request.actual_wall_contact &&
    request.prior_front_cap_release_active &&
    request.locked_target_body_lateral_clear &&
    !request.locked_target_position_jump;
  resolution.committed_pass_speed_hold_allowed =
    legacy_committed_pass_speed_hold_allowed ||
    resolution.minimum_motion_footprint_hold_active;

  const bool legacy_front_cap_release_ready = can_release_overtake_front_cap(
    OvertakeFrontCapReleaseRequest{
      resolution.active_execution,
      request.lateral_complete,
      request.execution_horizon_unconstrained,
      request.lateral_separation_clear,
      request.prior_front_cap_release_active,
      request.lateral_separation_above_reapply_threshold,
      resolution.constrained_horizon_release_allowed,
      legacy_committed_pass_speed_hold_allowed,
      request.target_seen,
      request.target_longitudinal_m});
  // The validated minimum-motion corridor owns release acquisition and reapplication.
  // Do not let the legacy lateral-only threshold bypass a predicted body overlap.
  resolution.front_cap_release_ready = request.preserve_validated_breakout_line ||
    (request.minimum_motion_corridor_active ?
    (resolution.minimum_motion_shiftout_release_allowed ||
    resolution.minimum_motion_footprint_release_allowed ||
    resolution.minimum_motion_side_by_side_escape_active ||
    resolution.minimum_motion_footprint_hold_active) :
    legacy_front_cap_release_ready);
  resolution.front_cap_state_update_required =
    resolution.active_execution &&
    !request.preserve_validated_breakout_line &&
    resolution.front_cap_release_ready != request.prior_front_cap_release_active;
  resolution.committed_pass_speed_hold_active =
    resolution.front_cap_release_ready &&
    resolution.committed_pass_speed_hold_allowed &&
    (resolution.minimum_motion_footprint_hold_active ||
    request.lateral_separation_above_reapply_threshold) &&
    (!request.lateral_complete ||
    (!request.execution_horizon_unconstrained &&
    !resolution.constrained_horizon_release_allowed));
  resolution.constrained_horizon_front_cap_release_active =
    resolution.front_cap_release_ready &&
    !request.execution_horizon_unconstrained &&
    resolution.constrained_horizon_release_allowed;
  resolution.committed_pass_speed_floor_active =
    should_apply_committed_pass_speed_floor(
    CommittedPassSpeedFloorRequest{
      request.committed_pass_speed_floor_enabled,
      request.shiftout_phase && !request.preserve_validated_breakout_line,
      request.pass_phase,
      request.lateral_complete,
      resolution.front_cap_release_ready,
      request.lateral_exclusion_latched,
      request.lateral_separation_clear,
      request.lateral_separation_above_reapply_threshold,
      request.locked_target_position_jump,
      request.target_seen,
      request.execution_path_physically_feasible,
      request.actual_wall_contact,
      request.target_speed_mps,
      request.slow_target_max_speed_mps,
      request.committed_pass_min_speed_mps});
  resolution.committed_shiftout_speed_floor_active =
    resolution.committed_pass_speed_floor_active &&
    request.shiftout_phase &&
    !request.preserve_validated_breakout_line;

  if (!resolution.front_cap_state_update_required) {
    return resolution;
  }
  if (resolution.front_cap_release_ready) {
    resolution.transition_reason = resolution.minimum_motion_side_by_side_escape_active ?
      CommittedPassFrontCapTransitionReason::MinimumMotionSideBySideForwardEscape :
      resolution.minimum_motion_shiftout_release_allowed ?
      CommittedPassFrontCapTransitionReason::MinimumMotionFootprintSweepClear :
      resolution.minimum_motion_footprint_release_allowed ?
      CommittedPassFrontCapTransitionReason::MinimumMotionFootprintSweepClear :
      !request.execution_horizon_unconstrained ?
      CommittedPassFrontCapTransitionReason::ConstrainedFeasiblePassHorizonAccepted :
      request.lateral_separation_clear ?
      CommittedPassFrontCapTransitionReason::LateralGoalAndExecutionHorizonClear :
      CommittedPassFrontCapTransitionReason::TargetNoLongerAhead;
  } else if (request.minimum_motion_corridor_active) {
    resolution.transition_reason = !request.target_seen ?
      CommittedPassFrontCapTransitionReason::LockedTargetUnavailable :
      request.locked_target_position_jump ?
      CommittedPassFrontCapTransitionReason::LockedTargetPositionJump :
      request.actual_wall_contact ?
      CommittedPassFrontCapTransitionReason::ActualWallContact :
      !current_body_physically_separated ?
      CommittedPassFrontCapTransitionReason::CurrentFootprintOverlap :
      !request.footprint_prediction_valid ?
      CommittedPassFrontCapTransitionReason::FootprintPredictionUnavailable :
      !predicted_sweep_physically_separated ?
      CommittedPassFrontCapTransitionReason::PredictedFootprintOverlap :
      !request.lateral_complete ?
      CommittedPassFrontCapTransitionReason::LateralGoalIncomplete :
      !request.execution_horizon_unconstrained ?
      CommittedPassFrontCapTransitionReason::ExecutionHorizonConstrained :
      CommittedPassFrontCapTransitionReason::LateralClearanceBelowReapplyThreshold;
  } else {
    resolution.transition_reason = !request.target_seen ?
      CommittedPassFrontCapTransitionReason::LockedTargetUnavailable :
      !request.lateral_complete ?
      CommittedPassFrontCapTransitionReason::LateralGoalIncomplete :
      !request.execution_horizon_unconstrained ?
      CommittedPassFrontCapTransitionReason::ExecutionHorizonConstrained :
      CommittedPassFrontCapTransitionReason::LateralClearanceBelowReapplyThreshold;
  }
  return resolution;
}

const char * to_string(const CommittedPassFrontCapTransitionReason reason) noexcept
{
  switch (reason) {
    case CommittedPassFrontCapTransitionReason::None:
      return "none";
    case CommittedPassFrontCapTransitionReason::MinimumMotionFootprintSweepClear:
      return "minimum-motion body footprint sweep clear";
    case CommittedPassFrontCapTransitionReason::MinimumMotionSideBySideForwardEscape:
      return "minimum-motion side-by-side forward escape";
    case CommittedPassFrontCapTransitionReason::ConstrainedFeasiblePassHorizonAccepted:
      return "physical lateral clearance; constrained feasible Pass horizon accepted";
    case CommittedPassFrontCapTransitionReason::LateralGoalAndExecutionHorizonClear:
      return "lateral goal and execution horizon clear";
    case CommittedPassFrontCapTransitionReason::TargetNoLongerAhead:
      return "lateral goal complete and locked target no longer ahead";
    case CommittedPassFrontCapTransitionReason::CurrentFootprintOverlap:
      return "current body footprints overlap";
    case CommittedPassFrontCapTransitionReason::PredictedFootprintOverlap:
      return "predicted body footprint sweep overlaps";
    case CommittedPassFrontCapTransitionReason::FootprintPredictionUnavailable:
      return "body footprint prediction unavailable";
    case CommittedPassFrontCapTransitionReason::LockedTargetPositionJump:
      return "locked target position jump";
    case CommittedPassFrontCapTransitionReason::ActualWallContact:
      return "actual wall contact";
    case CommittedPassFrontCapTransitionReason::LockedTargetUnavailable:
      return "locked target unavailable";
    case CommittedPassFrontCapTransitionReason::LateralGoalIncomplete:
      return "lateral goal incomplete";
    case CommittedPassFrontCapTransitionReason::ExecutionHorizonConstrained:
      return "execution horizon constrained";
    case CommittedPassFrontCapTransitionReason::LateralClearanceBelowReapplyThreshold:
      return "lateral clearance below reapply threshold";
  }
  return "unknown";
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

LiveExecutionCorridorHoldReferenceResolution
resolve_live_execution_corridor_hold_reference(
  const LiveExecutionCorridorHoldReferenceRequest & request) noexcept
{
  LiveExecutionCorridorHoldReferenceResolution resolution;
  if (std::isfinite(request.last_valid_corridor_sec)) {
    resolution.reference_sec = request.last_valid_corridor_sec;
  }

  const bool pass_phase_reference_available =
    request.pass_phase && request.current_body_footprints_separated &&
    std::isfinite(request.pass_phase_start_sec);
  if (
    pass_phase_reference_available &&
    (!std::isfinite(resolution.reference_sec) ||
    request.pass_phase_start_sec > resolution.reference_sec))
  {
    resolution.reference_sec = request.pass_phase_start_sec;
    resolution.pass_phase_reference_used = true;
  }
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

PassCommitStageResolution resolve_pass_commit_stage(
  const PassCommitStageRequest & request) noexcept
{
  PassCommitStageResolution resolution;
  if (
    !std::isfinite(request.side_by_side_no_return_front_distance_m) ||
    request.side_by_side_no_return_front_distance_m < 0.0)
  {
    return resolution;
  }

  resolution.valid = true;
  if (request.rear_clear) {
    resolution.stage = PassCommitStage::RearClear;
    return resolution;
  }
  if (!request.frozen_execution_active) {
    resolution.stage = PassCommitStage::Selectable;
    resolution.side_replan_allowed = true;
    return resolution;
  }
  if (
    request.lateral_clearance_latched || request.forward_completion_latched ||
    (std::isfinite(request.target_front_distance_m) &&
    request.target_front_distance_m + 1e-9 <
    request.side_by_side_no_return_front_distance_m))
  {
    resolution.stage = PassCommitStage::SideBySideCommitted;
    return resolution;
  }

  resolution.stage = PassCommitStage::ShiftCommitted;
  resolution.side_replan_allowed = true;
  return resolution;
}

const char * to_string(const PassCommitStage stage) noexcept
{
  switch (stage) {
    case PassCommitStage::Selectable:
      return "selectable";
    case PassCommitStage::ShiftCommitted:
      return "shift_committed";
    case PassCommitStage::SideBySideCommitted:
      return "side_by_side_committed";
    case PassCommitStage::RearClear:
      return "rear_clear";
  }
  return "unknown";
}

bool resolve_cross_side_no_return_latch(
  const CrossSideNoReturnLatchRequest & request) noexcept
{
  if (request.previously_latched) {
    return true;
  }
  if (!request.execution_active) {
    return false;
  }
  return request.observed_stage == PassCommitStage::SideBySideCommitted ||
         request.observed_stage == PassCommitStage::RearClear ||
         request.safe_separation_active || request.side_replacement_committed;
}

double resolve_cross_side_minimum_speed_requirement(
  const double current_ego_speed_mps, const double target_speed_mps) noexcept
{
  if (
    !std::isfinite(current_ego_speed_mps) || current_ego_speed_mps < 0.0 ||
    !std::isfinite(target_speed_mps) || target_speed_mps < 0.0)
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::min(current_ego_speed_mps, target_speed_mps);
}

CrossSideMissionReplacementResolution resolve_cross_side_mission_replacement(
  const CrossSideMissionReplacementRequest & request) noexcept
{
  CrossSideMissionReplacementResolution resolution;
  const bool tactical_rearm_active =
    request.tactical_no_return_rearmed && request.safe_separation_active;
  const auto finite_non_negative = [](const double value) {
      return std::isfinite(value) && value >= 0.0;
    };
  const auto valid_budget = [](const double value) {
      return (std::isfinite(value) && value >= 0.0) ||
             value == std::numeric_limits<double>::infinity();
    };

  if (!request.active_execution) {
    resolution.valid = true;
    resolution.reason = CrossSideMissionReplacementReason::Inactive;
    return resolution;
  }
  if (!request.side_changed) {
    resolution.valid = true;
    resolution.admitted = true;
    resolution.reason = CrossSideMissionReplacementReason::SameSide;
    return resolution;
  }
  if (
    (request.no_return_latched || !request.before_no_return) &&
    !tactical_rearm_active)
  {
    resolution.valid = true;
    resolution.reason = CrossSideMissionReplacementReason::NoReturn;
    return resolution;
  }
  if (
    request.safe_separation_active &&
    !tactical_rearm_active)
  {
    resolution.valid = true;
    resolution.reason = CrossSideMissionReplacementReason::SafeSeparation;
    return resolution;
  }
  if (!request.candidate_feasible) {
    resolution.valid = true;
    resolution.reason = CrossSideMissionReplacementReason::CandidateInfeasible;
    return resolution;
  }
  if (!request.rear_clear_prediction_checked) {
    resolution.valid = true;
    resolution.reason = CrossSideMissionReplacementReason::RearClearUnchecked;
    return resolution;
  }
  if (!request.rear_clear_prediction_feasible) {
    resolution.valid = true;
    resolution.reason = CrossSideMissionReplacementReason::RearClearInfeasible;
    return resolution;
  }
  if (
    request.candidate_requires_additional_side_transition &&
    !request.candidate_additional_side_transition_preflight_validated)
  {
    resolution.valid = true;
    resolution.reason =
      CrossSideMissionReplacementReason::AdditionalSideTransitionRequired;
    return resolution;
  }
  if (
    !finite_non_negative(request.predicted_rear_clear_time_sec) ||
    !finite_non_negative(request.predicted_rear_clear_distance_m) ||
    !finite_non_negative(request.predicted_rear_clear_speed_mps) ||
    !finite_non_negative(request.predicted_minimum_ego_speed_mps) ||
    !finite_non_negative(request.minimum_rear_clear_speed_mps) ||
    !finite_non_negative(request.minimum_ego_speed_mps) ||
    !finite_non_negative(request.minimum_path_wall_clearance_m) ||
    !finite_non_negative(request.minimum_required_path_wall_clearance_m) ||
    !valid_budget(request.remaining_time_budget_sec) ||
    !valid_budget(request.remaining_distance_budget_m))
  {
    resolution.reason = CrossSideMissionReplacementReason::InvalidPrediction;
    return resolution;
  }

  resolution.valid = true;
  if (
    request.minimum_path_wall_clearance_m + 1e-9 <
    request.minimum_required_path_wall_clearance_m)
  {
    resolution.reason = CrossSideMissionReplacementReason::WallReserveInsufficient;
    return resolution;
  }
  if (
    request.predicted_rear_clear_time_sec >
    request.remaining_time_budget_sec + 1e-9)
  {
    resolution.reason = CrossSideMissionReplacementReason::TimeBudgetExceeded;
    return resolution;
  }
  if (
    request.predicted_rear_clear_distance_m >
    request.remaining_distance_budget_m + 1e-9)
  {
    resolution.reason = CrossSideMissionReplacementReason::DistanceBudgetExceeded;
    return resolution;
  }
  if (
    request.predicted_minimum_ego_speed_mps + 1e-9 <
    request.minimum_ego_speed_mps)
  {
    resolution.reason = CrossSideMissionReplacementReason::MinimumSpeedInsufficient;
    return resolution;
  }
  if (
    request.predicted_rear_clear_speed_mps + 1e-9 <
    request.minimum_rear_clear_speed_mps)
  {
    resolution.reason = CrossSideMissionReplacementReason::RearClearSpeedInsufficient;
    return resolution;
  }

  resolution.admitted = true;
  resolution.restart_shiftout = request.pass_phase;
  resolution.reason = CrossSideMissionReplacementReason::Admitted;
  return resolution;
}

const char * to_string(const CrossSideMissionReplacementReason reason) noexcept
{
  switch (reason) {
    case CrossSideMissionReplacementReason::None:
      return "none";
    case CrossSideMissionReplacementReason::Inactive:
      return "inactive";
    case CrossSideMissionReplacementReason::SameSide:
      return "same_side";
    case CrossSideMissionReplacementReason::NoReturn:
      return "no_return";
    case CrossSideMissionReplacementReason::SafeSeparation:
      return "safe_separation";
    case CrossSideMissionReplacementReason::CandidateInfeasible:
      return "candidate_infeasible";
    case CrossSideMissionReplacementReason::RearClearUnchecked:
      return "rear_clear_unchecked";
    case CrossSideMissionReplacementReason::RearClearInfeasible:
      return "rear_clear_infeasible";
    case CrossSideMissionReplacementReason::AdditionalSideTransitionRequired:
      return "additional_side_transition_required";
    case CrossSideMissionReplacementReason::InvalidPrediction:
      return "invalid_prediction";
    case CrossSideMissionReplacementReason::WallReserveInsufficient:
      return "wall_reserve_insufficient";
    case CrossSideMissionReplacementReason::TimeBudgetExceeded:
      return "time_budget_exceeded";
    case CrossSideMissionReplacementReason::DistanceBudgetExceeded:
      return "distance_budget_exceeded";
    case CrossSideMissionReplacementReason::MinimumSpeedInsufficient:
      return "minimum_speed_insufficient";
    case CrossSideMissionReplacementReason::RearClearSpeedInsufficient:
      return "rear_clear_speed_insufficient";
    case CrossSideMissionReplacementReason::Admitted:
      return "admitted";
  }
  return "unknown";
}

bool early_pass_side_intrusion_risk(
  const EarlyPassSideIntrusionRiskRequest & request) noexcept
{
  if (
    !request.enabled || request.stage != PassCommitStage::ShiftCommitted ||
    (request.pass_side_sign != -1 && request.pass_side_sign != 1) ||
    !request.target_seen || request.target_position_jump ||
    !request.current_body_footprints_separated ||
    !request.footprint_prediction_valid ||
    !std::isfinite(request.current_target_relative_lateral_m) ||
    !std::isfinite(request.predicted_target_relative_lateral_m) ||
    !std::isfinite(request.ordering_margin_m) || request.ordering_margin_m < 0.0)
  {
    return false;
  }

  if (!request.predicted_body_footprint_sweep_separated) {
    return true;
  }

  const double side = static_cast<double>(request.pass_side_sign);
  const double current_ordering = side * request.current_target_relative_lateral_m;
  const double predicted_ordering = side * request.predicted_target_relative_lateral_m;
  const bool moving_toward_selected_line = predicted_ordering > current_ordering + 1e-9;
  const double warning_boundary = -2.0 * request.ordering_margin_m;
  return moving_toward_selected_line &&
         predicted_ordering + 1e-9 >= warning_boundary;
}

bool should_defer_direct_pass_prediction_handoff(
  const DirectPassPredictionHandoffRequest & request) noexcept
{
  return request.direct_pass_entry && request.selected_mission_frozen &&
         !request.locked_target_seen && request.target_id_available;
}

OvertakeEntryStageResolution resolve_overtake_entry_stage(
  const OvertakeEntryStageRequest & request) noexcept
{
  if (request.direct_base_line_pass) {
    return {
      OvertakeEntryStage::Pass,
      OvertakeEntryStageReason::BaseLineDirectPass};
  }
  if (request.direct_tiny_shift_pass) {
    return {
      OvertakeEntryStage::Pass,
      OvertakeEntryStageReason::TinyShiftDirectPass};
  }
  if (request.direct_same_side_resume) {
    return {
      OvertakeEntryStage::Pass,
      OvertakeEntryStageReason::SameSideResumePass};
  }
  if (request.safety_pause_resume_pass) {
    return {
      OvertakeEntryStage::Pass,
      OvertakeEntryStageReason::SafetyPauseResumePass};
  }
  if (request.safety_pause_resume) {
    return {
      OvertakeEntryStage::ShiftOut,
      OvertakeEntryStageReason::SafetyPauseShiftOut};
  }
  if (request.resuming_paused_mission) {
    return {
      OvertakeEntryStage::ShiftOut,
      OvertakeEntryStageReason::PausedMissionShiftOut};
  }
  return {
    OvertakeEntryStage::ShiftOut,
    OvertakeEntryStageReason::NewMissionShiftOut};
}

const char * to_string(const OvertakeEntryStageReason reason) noexcept
{
  switch (reason) {
    case OvertakeEntryStageReason::NewMissionShiftOut:
      return "overtake selected";
    case OvertakeEntryStageReason::PausedMissionShiftOut:
      return "committed pass revalidated after pause";
    case OvertakeEntryStageReason::SafetyPauseShiftOut:
      return "SafetyBrake-paused pass resumed through ShiftOut";
    case OvertakeEntryStageReason::BaseLineDirectPass:
      return "validated base racing line already clear";
    case OvertakeEntryStageReason::TinyShiftDirectPass:
      return "validated tiny-shift corridor already clear";
    case OvertakeEntryStageReason::SameSideResumePass:
      return "committed pass resumed on validated same side";
    case OvertakeEntryStageReason::SafetyPauseResumePass:
      return "SafetyBrake-paused pass resumed after lateral clearance";
  }
  return "unknown overtake entry stage";
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

OvertakeMissionPathResolution resolve_overtake_mission_path(
  const OvertakeMissionPathRequest & request) noexcept
{
  OvertakeMissionPathResolution resolution;
  if (
    !std::isfinite(request.path_distance_m) || request.path_distance_m < 0.0 ||
    !std::isfinite(request.start_lateral_m) ||
    !std::isfinite(request.pass_lateral_m) ||
    !std::isfinite(request.return_lateral_m) ||
    !std::isfinite(request.shift_distance_m) || request.shift_distance_m <= 0.0 ||
    !std::isfinite(request.pass_distance_m) || request.pass_distance_m < 0.0 ||
    !std::isfinite(request.return_distance_m) || request.return_distance_m <= 0.0)
  {
    return resolution;
  }

  const double pass_start_m = request.shift_distance_m;
  const double return_start_m = pass_start_m + request.pass_distance_m;
  resolution.total_distance_m = return_start_m + request.return_distance_m;
  resolution.valid = true;
  if (request.path_distance_m < pass_start_m) {
    resolution.stage = OvertakeMissionPathStage::ShiftOut;
    const double progress = resolve_overtake_line_horizon_progress(
      OvertakeLineHorizonProgressRequest{
        false, 0.0, request.path_distance_m, request.shift_distance_m});
    resolution.lateral_target_m = request.start_lateral_m +
      progress * (request.pass_lateral_m - request.start_lateral_m);
    return resolution;
  }
  if (request.path_distance_m < return_start_m) {
    resolution.stage = OvertakeMissionPathStage::Pass;
    resolution.lateral_target_m = request.pass_lateral_m;
    return resolution;
  }
  if (request.path_distance_m < resolution.total_distance_m) {
    resolution.stage = OvertakeMissionPathStage::Return;
    const double progress = resolve_overtake_line_horizon_progress(
      OvertakeLineHorizonProgressRequest{
        false, 0.0, request.path_distance_m - return_start_m,
        request.return_distance_m});
    resolution.lateral_target_m = request.pass_lateral_m +
      progress * (request.return_lateral_m - request.pass_lateral_m);
    return resolution;
  }
  resolution.stage = OvertakeMissionPathStage::Complete;
  resolution.lateral_target_m = request.return_lateral_m;
  return resolution;
}

RecedingHorizonLateralResolution optimize_receding_horizon_lateral_trajectory(
  const RecedingHorizonLateralRequest & request) noexcept
{
  RecedingHorizonLateralResolution resolution;
  if (!request.enabled) {
    resolution.valid = true;
    return resolution;
  }
  if (
    !std::isfinite(request.current_lateral_m) ||
    !std::isfinite(request.reference_weight) || request.reference_weight < 0.0 ||
    !std::isfinite(request.warm_start_weight) || request.warm_start_weight < 0.0 ||
    !std::isfinite(request.slope_weight) || request.slope_weight < 0.0 ||
    !std::isfinite(request.curvature_weight) || request.curvature_weight < 0.0 ||
    !std::isfinite(request.current_anchor_weight) || request.current_anchor_weight < 0.0 ||
    !std::isfinite(request.relaxation) || request.relaxation <= 0.0 ||
    request.relaxation > 1.0 || request.iterations == 0U ||
    request.samples.size() < 2U)
  {
    return resolution;
  }

  double previous_distance_m = -std::numeric_limits<double>::infinity();
  for (const auto & sample : request.samples) {
    if (
      !std::isfinite(sample.path_distance_m) || sample.path_distance_m < 0.0 ||
      sample.path_distance_m + 1e-9 < previous_distance_m ||
      !std::isfinite(sample.lower_bound_m) ||
      !std::isfinite(sample.upper_bound_m) ||
      sample.upper_bound_m + 1e-9 < sample.lower_bound_m ||
      !std::isfinite(sample.reference_lateral_m) ||
      !std::isfinite(sample.warm_start_lateral_m))
    {
      return RecedingHorizonLateralResolution{};
    }
    previous_distance_m = sample.path_distance_m;
  }

  const std::size_t count = request.samples.size();
  std::vector<double> targets(count, 0.0);
  for (std::size_t i = 0U; i < count; ++i) {
    const auto & sample = request.samples[i];
    targets[i] = std::clamp(
      sample.warm_start_lateral_m,
      sample.lower_bound_m, sample.upper_bound_m);
  }

  // Each update is the exact minimizer of the local quadratic surrogate,
  // relaxed before projection.  Alternating sweep direction removes the
  // directional bias of an in-place Gauss-Seidel pass while retaining its
  // fast convergence for the short MPC horizon.
  const auto update_sample = [&](const std::size_t i) {
      const auto & sample = request.samples[i];
      double weighted_target =
        request.reference_weight * sample.reference_lateral_m +
        request.warm_start_weight * sample.warm_start_lateral_m;
      double total_weight = request.reference_weight + request.warm_start_weight;
      if (i == 0U && request.current_anchor_weight > 0.0) {
        weighted_target += request.current_anchor_weight * request.current_lateral_m;
        total_weight += request.current_anchor_weight;
      }
      if (request.slope_weight > 0.0 && i > 0U) {
        weighted_target += request.slope_weight * targets[i - 1U];
        total_weight += request.slope_weight;
      }
      if (request.slope_weight > 0.0 && i + 1U < count) {
        weighted_target += request.slope_weight * targets[i + 1U];
        total_weight += request.slope_weight;
      }
      if (request.curvature_weight > 0.0 && i >= 2U) {
        weighted_target += request.curvature_weight *
          (2.0 * targets[i - 1U] - targets[i - 2U]);
        total_weight += request.curvature_weight;
      }
      if (request.curvature_weight > 0.0 && i + 2U < count) {
        weighted_target += request.curvature_weight *
          (2.0 * targets[i + 1U] - targets[i + 2U]);
        total_weight += request.curvature_weight;
      }
      if (request.curvature_weight > 0.0 && i > 0U && i + 1U < count) {
        // The centre coefficient of (x[i+1] - 2*x[i] + x[i-1])^2
        // contributes four times the endpoint coefficient.
        weighted_target += 2.0 * request.curvature_weight *
          (targets[i - 1U] + targets[i + 1U]);
        total_weight += 4.0 * request.curvature_weight;
      }
      if (total_weight <= 1e-12) {
        return;
      }
      const double local_minimum = weighted_target / total_weight;
      const double relaxed = targets[i] +
        request.relaxation * (local_minimum - targets[i]);
      targets[i] = std::clamp(
        relaxed, sample.lower_bound_m, sample.upper_bound_m);
    };

  for (std::size_t iteration = 0U; iteration < request.iterations; ++iteration) {
    if (iteration % 2U == 0U) {
      for (std::size_t i = 0U; i < count; ++i) {
        update_sample(i);
      }
    } else {
      for (std::size_t i = count; i-- > 0U;) {
        update_sample(i);
      }
    }
  }

  double objective = 0.0;
  double maximum_adjustment_m = 0.0;
  for (std::size_t i = 0U; i < count; ++i) {
    const auto & sample = request.samples[i];
    const double reference_error = targets[i] - sample.reference_lateral_m;
    const double warm_start_error = targets[i] - sample.warm_start_lateral_m;
    objective += request.reference_weight * reference_error * reference_error;
    objective += request.warm_start_weight * warm_start_error * warm_start_error;
    maximum_adjustment_m = std::max(
      maximum_adjustment_m, std::abs(reference_error));
    if (i == 0U) {
      const double anchor_error = targets[i] - request.current_lateral_m;
      objective += request.current_anchor_weight * anchor_error * anchor_error;
    }
    if (i > 0U) {
      const double slope = targets[i] - targets[i - 1U];
      objective += request.slope_weight * slope * slope;
    }
    if (i > 1U) {
      const double curvature =
        targets[i] - 2.0 * targets[i - 1U] + targets[i - 2U];
      objective += request.curvature_weight * curvature * curvature;
    }
  }

  resolution.valid = std::isfinite(objective);
  resolution.feasible = resolution.valid;
  resolution.lateral_targets_m = std::move(targets);
  resolution.objective = objective;
  resolution.maximum_reference_adjustment_m = maximum_adjustment_m;
  return resolution;
}

RecedingHorizonTargetBoundsResolution resolve_receding_horizon_target_bounds(
  const RecedingHorizonTargetBoundsRequest & request) noexcept
{
  RecedingHorizonTargetBoundsResolution resolution;
  const auto finite = [](const double value) {return std::isfinite(value);};
  if (
    (request.pass_side_sign != -1 && request.pass_side_sign != 1) ||
    !finite(request.wall_lower_bound_m) || !finite(request.wall_upper_bound_m) ||
    !finite(request.trust_lower_bound_m) || !finite(request.trust_upper_bound_m) ||
    !finite(request.target_lateral_m) ||
    !finite(request.robust_center_separation_m) ||
    !finite(request.configured_center_separation_m) ||
    !finite(request.physical_center_separation_m) ||
    request.wall_upper_bound_m + 1e-9 < request.wall_lower_bound_m ||
    request.robust_center_separation_m < 0.0 ||
    request.configured_center_separation_m < 0.0 ||
    request.physical_center_separation_m < 0.0)
  {
    return resolution;
  }

  const double physical_separation = request.physical_center_separation_m;
  const double configured_separation = std::max(
    physical_separation, request.configured_center_separation_m);
  const double robust_separation = std::max(
    configured_separation, request.robust_center_separation_m);
  const auto try_bounds = [&](const double base_lower, const double base_upper,
      const double separation, const bool degraded, const bool physical,
      const bool trust_expanded) {
      if (!finite(base_lower) || !finite(base_upper) || base_upper + 1e-9 < base_lower) {
        return false;
      }
      double lower = base_lower;
      double upper = base_upper;
      if (request.pass_side_sign > 0) {
        lower = std::max(lower, request.target_lateral_m + separation);
      } else {
        upper = std::min(upper, request.target_lateral_m - separation);
      }
      if (upper + 1e-9 < lower) {
        return false;
      }
      resolution.valid = true;
      resolution.robust_degraded = degraded;
      resolution.physical_separation_used = physical;
      resolution.trust_region_expanded = trust_expanded;
      resolution.lower_bound_m = lower;
      resolution.upper_bound_m = upper;
      resolution.applied_center_separation_m = separation;
      return true;
    };

  if (try_bounds(
      request.trust_lower_bound_m, request.trust_upper_bound_m,
      robust_separation, false, false, false))
  {
    return resolution;
  }
  if (!request.allow_robust_degradation) {
    return resolution;
  }
  if (
    configured_separation + 1e-9 < robust_separation &&
    try_bounds(
      request.trust_lower_bound_m, request.trust_upper_bound_m,
      configured_separation, true, false, false))
  {
    return resolution;
  }
  if (
    physical_separation + 1e-9 < configured_separation &&
    try_bounds(
      request.trust_lower_bound_m, request.trust_upper_bound_m,
      physical_separation, true, true, false))
  {
    return resolution;
  }
  (void)try_bounds(
    request.wall_lower_bound_m, request.wall_upper_bound_m,
    physical_separation, true, true, true);
  return resolution;
}

RecedingHorizonWarmStartResolution resample_receding_horizon_warm_start(
  const RecedingHorizonWarmStartRequest & request) noexcept
{
  RecedingHorizonWarmStartResolution resolution;
  if (
    !std::isfinite(request.forward_progress_m) || request.forward_progress_m < 0.0 ||
    request.previous_path_distances_m.size() < 2U ||
    request.previous_path_distances_m.size() !=
    request.previous_lateral_targets_m.size() ||
    request.current_path_distances_m.empty() ||
    request.current_path_distances_m.size() !=
    request.current_fallback_targets_m.size())
  {
    return resolution;
  }

  const auto strictly_increasing_finite = [](const std::vector<double> & values) {
      double previous = -std::numeric_limits<double>::infinity();
      for (const double value : values) {
        if (!std::isfinite(value) || value < 0.0 || value <= previous + 1e-9) {
          return false;
        }
        previous = value;
      }
      return true;
    };
  if (
    !strictly_increasing_finite(request.previous_path_distances_m) ||
    !strictly_increasing_finite(request.current_path_distances_m))
  {
    return resolution;
  }
  for (const double value : request.previous_lateral_targets_m) {
    if (!std::isfinite(value)) {
      return resolution;
    }
  }
  for (const double value : request.current_fallback_targets_m) {
    if (!std::isfinite(value)) {
      return resolution;
    }
  }

  resolution.valid = true;
  resolution.lateral_targets_m = request.current_fallback_targets_m;
  const double previous_end_m = request.previous_path_distances_m.back();
  for (std::size_t i = 0U; i < request.current_path_distances_m.size(); ++i) {
    const double query_m =
      request.forward_progress_m + request.current_path_distances_m[i];
    if (query_m > previous_end_m + 1e-9) {
      continue;
    }

    const auto upper = std::lower_bound(
      request.previous_path_distances_m.begin(),
      request.previous_path_distances_m.end(), query_m);
    if (upper == request.previous_path_distances_m.begin()) {
      resolution.lateral_targets_m[i] =
        request.previous_lateral_targets_m.front();
      resolution.used_previous_solution = true;
      continue;
    }
    if (upper == request.previous_path_distances_m.end()) {
      resolution.lateral_targets_m[i] =
        request.previous_lateral_targets_m.back();
      resolution.used_previous_solution = true;
      continue;
    }

    const std::size_t upper_index = static_cast<std::size_t>(
      std::distance(request.previous_path_distances_m.begin(), upper));
    const std::size_t lower_index = upper_index - 1U;
    const double lower_distance =
      request.previous_path_distances_m[lower_index];
    const double upper_distance =
      request.previous_path_distances_m[upper_index];
    const double ratio = std::clamp(
      (query_m - lower_distance) / (upper_distance - lower_distance),
      0.0, 1.0);
    resolution.lateral_targets_m[i] =
      request.previous_lateral_targets_m[lower_index] + ratio *
      (request.previous_lateral_targets_m[upper_index] -
      request.previous_lateral_targets_m[lower_index]);
    resolution.used_previous_solution = true;
  }
  return resolution;
}

bool can_retain_receding_horizon_execution_lease(
  const RecedingHorizonExecutionLeaseRequest & request) noexcept
{
  if (
    !request.enabled || !request.active_execution_phase ||
    !request.mission_path_frozen || !request.mission_side_valid ||
    !request.last_solution_feasible || !request.mission_generation_matches ||
    !request.mission_side_matches || !request.target_continuous_or_leased ||
    request.target_position_jump || request.target_course_progress_rejected ||
    request.execution_corridor_blocked || request.explicit_forbidden_waypoint ||
    request.emergency_front_risk || request.solver_recovery_requested ||
    request.hard_wall_fault || !std::isfinite(request.now_sec) ||
    !std::isfinite(request.last_feasible_sec) ||
    !std::isfinite(request.maximum_age_sec) || request.maximum_age_sec <= 0.0 ||
    request.now_sec + 1e-9 < request.last_feasible_sec)
  {
    return false;
  }
  return request.now_sec - request.last_feasible_sec <=
         request.maximum_age_sec + 1e-9;
}

OvertakeBodyClearDeadlineResolution resolve_overtake_body_clear_deadline(
  const OvertakeBodyClearDeadlineRequest & request) noexcept
{
  OvertakeBodyClearDeadlineResolution resolution;
  if (!request.enabled) {
    resolution.valid = true;
    resolution.feasible = true;
    return resolution;
  }

  auto mission_origin_request = request.mission_path;
  mission_origin_request.path_distance_m = 0.0;
  const auto mission_origin = resolve_overtake_mission_path(mission_origin_request);
  if (
    !mission_origin.valid ||
    !std::isfinite(request.target_longitudinal_m) ||
    !std::isfinite(request.ego_speed_mps) || request.ego_speed_mps <= 1e-6 ||
    !std::isfinite(request.target_speed_mps) || request.target_speed_mps < 0.0 ||
    !std::isfinite(request.target_lateral_m) ||
    !std::isfinite(request.target_lateral_velocity_mps) ||
    !std::isfinite(request.target_lateral_prediction_horizon_sec) ||
    request.target_lateral_prediction_horizon_sec < 0.0 ||
    std::isnan(request.target_lateral_velocity_decay_time_sec) ||
    request.target_lateral_velocity_decay_time_sec < 0.0 ||
    !std::isfinite(request.lateral_clearance_m) || request.lateral_clearance_m < 0.0 ||
    !std::isfinite(request.hard_longitudinal_distance_m) ||
    request.hard_longitudinal_distance_m < 0.0 ||
    !std::isfinite(request.deadline_margin_sec) || request.deadline_margin_sec < 0.0 ||
    request.sample_count < 2U)
  {
    return resolution;
  }

  resolution.valid = true;
  resolution.checked = true;
  const double closing_speed_mps = request.ego_speed_mps - request.target_speed_mps;
  if (
    request.target_longitudinal_m <= request.hard_longitudinal_distance_m + 1e-9)
  {
    resolution.hard_distance_time_sec = 0.0;
  } else if (closing_speed_mps > 1e-6) {
    resolution.hard_distance_time_sec =
      (request.target_longitudinal_m - request.hard_longitudinal_distance_m) /
      closing_speed_mps;
  }

  const auto target_lateral_at = [&](const double time_sec) {
      return predict_lateral_with_velocity_decay(
        request.target_lateral_m, request.target_lateral_velocity_mps,
        time_sec, request.target_lateral_prediction_horizon_sec,
        request.target_lateral_velocity_decay_time_sec);
    };
  const double initial_lateral_separation =
    std::abs(target_lateral_at(0.0) - mission_origin.lateral_target_m);
  resolution.currently_laterally_clear =
    initial_lateral_separation + 1e-9 >= request.lateral_clearance_m;
  if (resolution.currently_laterally_clear) {
    resolution.body_clear_time_sec = 0.0;
    resolution.body_clear_distance_m = 0.0;
    resolution.feasible = true;
    return resolution;
  }

  double previous_distance_m = 0.0;
  double previous_time_sec = 0.0;
  double previous_margin_m =
    initial_lateral_separation - request.lateral_clearance_m;
  for (std::size_t i = 1U; i <= request.sample_count; ++i) {
    const double ratio =
      static_cast<double>(i) / static_cast<double>(request.sample_count);
    const double distance_m = request.mission_path.shift_distance_m * ratio;
    auto path_request = request.mission_path;
    path_request.path_distance_m = distance_m;
    const auto path = resolve_overtake_mission_path(path_request);
    if (!path.valid) {
      resolution.valid = false;
      resolution.feasible = false;
      return resolution;
    }
    const double time_sec = distance_m / request.ego_speed_mps;
    const double margin_m =
      std::abs(target_lateral_at(time_sec) - path.lateral_target_m) -
      request.lateral_clearance_m;
    if (margin_m >= -1e-9 && previous_margin_m < 0.0) {
      const double denominator = margin_m - previous_margin_m;
      const double interpolation = denominator > 1e-12 ?
        std::clamp(-previous_margin_m / denominator, 0.0, 1.0) : 1.0;
      resolution.body_clear_time_sec = previous_time_sec +
        interpolation * (time_sec - previous_time_sec);
      resolution.body_clear_distance_m = previous_distance_m +
        interpolation * (distance_m - previous_distance_m);
      break;
    }
    previous_distance_m = distance_m;
    previous_time_sec = time_sec;
    previous_margin_m = margin_m;
  }

  if (!std::isfinite(resolution.body_clear_time_sec)) {
    return resolution;
  }
  resolution.feasible =
    !std::isfinite(resolution.hard_distance_time_sec) ||
    resolution.body_clear_time_sec + request.deadline_margin_sec <=
    resolution.hard_distance_time_sec + 1e-9;
  return resolution;
}

OvertakeEntryDeadlineMarginResolution resolve_overtake_entry_deadline_margin(
  const OvertakeEntryDeadlineMarginRequest & request) noexcept
{
  OvertakeEntryDeadlineMarginResolution resolution;
  if (
    !std::isfinite(request.base_margin_sec) || request.base_margin_sec < 0.0 ||
    (request.pass_side_sign != -1 && request.pass_side_sign != 1) ||
    !std::isfinite(request.target_lateral_velocity_mps) ||
    !std::isfinite(request.intrusion_gain_sec_per_mps) ||
    request.intrusion_gain_sec_per_mps < 0.0 ||
    !std::isfinite(request.maximum_extra_margin_sec) ||
    request.maximum_extra_margin_sec < 0.0)
  {
    return resolution;
  }

  resolution.valid = true;
  resolution.target_intrusion_speed_mps = std::max(
    0.0,
    static_cast<double>(request.pass_side_sign) *
    request.target_lateral_velocity_mps);
  resolution.extra_margin_sec = std::min(
    request.maximum_extra_margin_sec,
    request.intrusion_gain_sec_per_mps *
    resolution.target_intrusion_speed_mps);
  resolution.effective_margin_sec =
    request.base_margin_sec + resolution.extra_margin_sec;
  return resolution;
}

OvertakeKinematicRolloutResolution resolve_overtake_kinematic_rollout(
  const OvertakeKinematicRolloutRequest & request) noexcept
{
  OvertakeKinematicRolloutResolution resolution;
  if (!request.enabled) {
    resolution.valid = true;
    resolution.feasible = true;
    return resolution;
  }

  auto mission_origin_request = request.mission_path;
  mission_origin_request.path_distance_m = 0.0;
  const auto mission_origin = resolve_overtake_mission_path(mission_origin_request);
  if (
    !mission_origin.valid ||
    !std::isfinite(request.target_longitudinal_m) ||
    !std::isfinite(request.current_ego_speed_mps) || request.current_ego_speed_mps < 0.0 ||
    !std::isfinite(request.target_speed_mps) || request.target_speed_mps < 0.0 ||
    !std::isfinite(request.candidate_closing_speed_mps) ||
    request.candidate_closing_speed_mps < 0.0 ||
    !std::isfinite(request.maximum_ego_speed_mps) || request.maximum_ego_speed_mps <= 0.0 ||
    !std::isfinite(request.maximum_acceleration_mps2) ||
    request.maximum_acceleration_mps2 < 0.0 ||
    !std::isfinite(request.maximum_deceleration_mps2) ||
    request.maximum_deceleration_mps2 < 0.0 ||
    !std::isfinite(request.control_delay_sec) || request.control_delay_sec < 0.0 ||
    !std::isfinite(request.target_lateral_m) ||
    !std::isfinite(request.target_lateral_velocity_mps) ||
    !std::isfinite(request.target_lateral_prediction_horizon_sec) ||
    request.target_lateral_prediction_horizon_sec < 0.0 ||
    !std::isfinite(request.target_longitudinal_acceleration_mps2) ||
    !std::isfinite(request.target_longitudinal_acceleration_horizon_sec) ||
    request.target_longitudinal_acceleration_horizon_sec < 0.0 ||
    std::isnan(request.target_lateral_velocity_decay_time_sec) ||
    request.target_lateral_velocity_decay_time_sec < 0.0 ||
    !std::isfinite(request.lateral_clearance_m) || request.lateral_clearance_m < 0.0 ||
    !std::isfinite(request.hard_longitudinal_distance_m) ||
    request.hard_longitudinal_distance_m < 0.0 ||
    !std::isfinite(request.deadline_margin_sec) || request.deadline_margin_sec < 0.0 ||
    !std::isfinite(request.time_step_sec) || request.time_step_sec <= 0.0 ||
    !std::isfinite(request.maximum_time_sec) || request.maximum_time_sec <= 0.0 ||
    (request.rear_clear_prediction_enabled &&
    (!std::isfinite(request.rear_clear_distance_m) || request.rear_clear_distance_m < 0.0)))
  {
    return resolution;
  }

  double previous_cap_distance = -std::numeric_limits<double>::infinity();
  for (const auto & sample : request.speed_caps) {
    if (
      !std::isfinite(sample.path_distance_m) || sample.path_distance_m < 0.0 ||
      !std::isfinite(sample.speed_cap_mps) || sample.speed_cap_mps < 0.0 ||
      !std::isfinite(sample.course_progress_ratio) ||
      sample.course_progress_ratio <= 0.0 ||
      sample.path_distance_m + 1e-9 < previous_cap_distance)
    {
      return resolution;
    }
    previous_cap_distance = sample.path_distance_m;
  }

  const auto speed_cap_at = [&](const double path_distance_m) {
      double cap = request.maximum_ego_speed_mps;
      if (request.speed_caps.empty()) {
        return cap;
      }
      if (path_distance_m <= request.speed_caps.front().path_distance_m + 1e-9) {
        return std::min(cap, request.speed_caps.front().speed_cap_mps);
      }
      for (std::size_t i = 1U; i < request.speed_caps.size(); ++i) {
        const auto & previous = request.speed_caps[i - 1U];
        const auto & current = request.speed_caps[i];
        if (path_distance_m <= current.path_distance_m + 1e-9) {
          const double span = current.path_distance_m - previous.path_distance_m;
          const double ratio = span > 1e-9 ?
            std::clamp((path_distance_m - previous.path_distance_m) / span, 0.0, 1.0) :
            1.0;
          const double interpolated = previous.speed_cap_mps +
            ratio * (current.speed_cap_mps - previous.speed_cap_mps);
          return std::min(cap, interpolated);
        }
      }
      return std::min(cap, request.speed_caps.back().speed_cap_mps);
    };
  const auto course_progress_ratio_at = [&](const double path_distance_m) {
      if (request.speed_caps.empty()) {
        return 1.0;
      }
      if (path_distance_m <= request.speed_caps.front().path_distance_m + 1e-9) {
        return request.speed_caps.front().course_progress_ratio;
      }
      for (std::size_t i = 1U; i < request.speed_caps.size(); ++i) {
        const auto & previous = request.speed_caps[i - 1U];
        const auto & current = request.speed_caps[i];
        if (path_distance_m <= current.path_distance_m + 1e-9) {
          const double span = current.path_distance_m - previous.path_distance_m;
          const double ratio = span > 1e-9 ?
            std::clamp((path_distance_m - previous.path_distance_m) / span, 0.0, 1.0) :
            1.0;
          return previous.course_progress_ratio +
                 ratio * (current.course_progress_ratio - previous.course_progress_ratio);
        }
      }
      return request.speed_caps.back().course_progress_ratio;
    };
  const auto target_lateral_at = [&](const double time_sec) {
      return predict_lateral_with_velocity_decay(
        request.target_lateral_m, request.target_lateral_velocity_mps,
        time_sec, request.target_lateral_prediction_horizon_sec,
        request.target_lateral_velocity_decay_time_sec);
    };

  resolution.valid = true;
  resolution.checked = true;
  resolution.rear_clear_checked = request.rear_clear_prediction_enabled;
  double ego_speed_mps = request.current_ego_speed_mps;
  resolution.minimum_ego_speed_mps = ego_speed_mps;
  double ego_course_distance_m = 0.0;
  double ego_physical_distance_m = 0.0;
  double mission_distance_m = 0.0;
  double target_course_distance_m = request.target_longitudinal_m;
  double target_speed_mps = request.target_speed_mps;
  double previous_time_sec = 0.0;
  double previous_mission_distance_m = 0.0;
  double previous_ego_physical_distance_m = 0.0;
  double previous_ego_speed_mps = ego_speed_mps;
  double previous_lateral_margin_m =
    std::abs(target_lateral_at(0.0) - mission_origin.lateral_target_m) -
    request.lateral_clearance_m;
  double previous_hard_margin_m =
    request.target_longitudinal_m - request.hard_longitudinal_distance_m;
  double previous_rear_clear_margin_m =
    request.target_longitudinal_m + request.rear_clear_distance_m;
  resolution.currently_laterally_clear = previous_lateral_margin_m >= -1e-9;
  if (request.mission_path.shift_distance_m <= 1e-9) {
    resolution.shift_complete_time_sec = 0.0;
    resolution.shift_complete_target_longitudinal_m =
      request.target_longitudinal_m;
    resolution.pass_target_clearance_checked = true;
    resolution.minimum_pass_target_surface_clearance_m =
      previous_lateral_margin_m;
  }
  if (resolution.currently_laterally_clear) {
    resolution.body_clear_time_sec = 0.0;
    resolution.body_clear_distance_m = 0.0;
  }
  if (previous_hard_margin_m <= 1e-9) {
    resolution.hard_distance_time_sec = 0.0;
  }
  if (
    request.rear_clear_prediction_enabled &&
    previous_rear_clear_margin_m <= 1e-9)
  {
    resolution.rear_clear_time_sec = 0.0;
    resolution.rear_clear_ego_distance_m = 0.0;
    resolution.rear_clear_mission_distance_m = 0.0;
    resolution.rear_clear_ego_speed_mps = ego_speed_mps;
  }

  const std::size_t maximum_steps = static_cast<std::size_t>(
    std::ceil(request.maximum_time_sec / request.time_step_sec));
  for (std::size_t step = 0U; step < maximum_steps; ++step) {
    const double step_start_sec = static_cast<double>(step) * request.time_step_sec;
    const double step_end_sec = std::min(
      request.maximum_time_sec, step_start_sec + request.time_step_sec);
    const double step_duration_sec = step_end_sec - step_start_sec;
    if (step_duration_sec <= 0.0) {
      break;
    }

    const double hold_duration_sec = std::clamp(
      request.control_delay_sec - step_start_sec, 0.0, step_duration_sec);
    const double controlled_duration_sec = step_duration_sec - hold_duration_sec;
    const double commanded_speed_mps = std::min(
      speed_cap_at(ego_course_distance_m),
      std::min(
        request.maximum_ego_speed_mps,
        target_speed_mps + request.candidate_closing_speed_mps));
    double next_ego_speed_mps = ego_speed_mps;
    if (controlled_duration_sec > 0.0) {
      if (commanded_speed_mps > ego_speed_mps) {
        next_ego_speed_mps = std::min(
          commanded_speed_mps,
          ego_speed_mps + request.maximum_acceleration_mps2 * controlled_duration_sec);
      } else {
        next_ego_speed_mps = std::max(
          commanded_speed_mps,
          ego_speed_mps - request.maximum_deceleration_mps2 * controlled_duration_sec);
      }
    }
    const double held_distance_m = ego_speed_mps * hold_duration_sec;
    const double controlled_distance_m =
      0.5 * (ego_speed_mps + next_ego_speed_mps) * controlled_duration_sec;
    const double physical_ego_distance_m = held_distance_m + controlled_distance_m;
    const double course_progress_ratio = course_progress_ratio_at(ego_course_distance_m);
    ego_course_distance_m += physical_ego_distance_m * course_progress_ratio;
    ego_physical_distance_m += physical_ego_distance_m;
    mission_distance_m += controlled_distance_m;
    const double target_acceleration_duration_sec = std::clamp(
      request.target_longitudinal_acceleration_horizon_sec - step_start_sec,
      0.0, step_duration_sec);
    double next_target_speed_mps = target_speed_mps;
    if (target_acceleration_duration_sec > 0.0) {
      next_target_speed_mps = std::max(
        0.0,
        target_speed_mps + request.target_longitudinal_acceleration_mps2 *
        target_acceleration_duration_sec);
    }
    target_course_distance_m +=
      0.5 * (target_speed_mps + next_target_speed_mps) *
      target_acceleration_duration_sec +
      next_target_speed_mps *
      (step_duration_sec - target_acceleration_duration_sec);
    target_speed_mps = next_target_speed_mps;
    ego_speed_mps = next_ego_speed_mps;
    resolution.minimum_ego_speed_mps = std::min(
      resolution.minimum_ego_speed_mps, ego_speed_mps);

    auto path_request = request.mission_path;
    path_request.path_distance_m = std::min(
      mission_distance_m, request.mission_path.shift_distance_m);
    const auto path = resolve_overtake_mission_path(path_request);
    if (!path.valid) {
      return OvertakeKinematicRolloutResolution{};
    }
    const double lateral_margin_m =
      std::abs(target_lateral_at(step_end_sec) - path.lateral_target_m) -
      request.lateral_clearance_m;
    if (mission_distance_m + 1e-9 >= request.mission_path.shift_distance_m) {
      resolution.pass_target_clearance_checked = true;
      if (std::isnan(resolution.minimum_pass_target_surface_clearance_m)) {
        resolution.minimum_pass_target_surface_clearance_m = lateral_margin_m;
      } else {
        resolution.minimum_pass_target_surface_clearance_m = std::min(
          resolution.minimum_pass_target_surface_clearance_m,
          lateral_margin_m);
      }
    }
    if (
      !std::isfinite(resolution.body_clear_time_sec) &&
      lateral_margin_m >= -1e-9 && previous_lateral_margin_m < 0.0)
    {
      const double denominator = lateral_margin_m - previous_lateral_margin_m;
      const double interpolation = denominator > 1e-12 ?
        std::clamp(-previous_lateral_margin_m / denominator, 0.0, 1.0) : 1.0;
      resolution.body_clear_time_sec = previous_time_sec +
        interpolation * (step_end_sec - previous_time_sec);
      resolution.body_clear_distance_m = previous_mission_distance_m +
        interpolation * (mission_distance_m - previous_mission_distance_m);
    }

    const double lateral_motion_time_sec = std::max(
      0.15, step_end_sec - request.control_delay_sec);
    const double required_lateral_accel_mps2 =
      2.0 * std::abs(path.lateral_target_m - mission_origin.lateral_target_m) /
      (lateral_motion_time_sec * lateral_motion_time_sec);
    resolution.max_required_lateral_accel_mps2 = std::max(
      resolution.max_required_lateral_accel_mps2, required_lateral_accel_mps2);

    const double relative_longitudinal_m =
      target_course_distance_m - ego_course_distance_m;
    if (
      !std::isfinite(resolution.shift_complete_time_sec) &&
      mission_distance_m + 1e-9 >= request.mission_path.shift_distance_m)
    {
      // Use the end of the first 50 ms integration step that reaches the
      // lateral-ramp distance. This is conservative for a closing ego and is
      // sufficient for caller-side no-side-by-side transition admission.
      resolution.shift_complete_time_sec = step_end_sec;
      resolution.shift_complete_target_longitudinal_m = relative_longitudinal_m;
    }
    const double hard_margin_m =
      relative_longitudinal_m - request.hard_longitudinal_distance_m;
    if (
      !std::isfinite(resolution.hard_distance_time_sec) &&
      hard_margin_m <= 1e-9 && previous_hard_margin_m > 0.0)
    {
      const double denominator = previous_hard_margin_m - hard_margin_m;
      const double interpolation = denominator > 1e-12 ?
        std::clamp(previous_hard_margin_m / denominator, 0.0, 1.0) : 1.0;
      resolution.hard_distance_time_sec = previous_time_sec +
        interpolation * (step_end_sec - previous_time_sec);
    }

    const double rear_clear_margin_m =
      relative_longitudinal_m + request.rear_clear_distance_m;
    if (
      request.rear_clear_prediction_enabled &&
      !std::isfinite(resolution.rear_clear_time_sec) &&
      rear_clear_margin_m <= 1e-9 && previous_rear_clear_margin_m > 0.0)
    {
      const double denominator = previous_rear_clear_margin_m - rear_clear_margin_m;
      const double interpolation = denominator > 1e-12 ?
        std::clamp(previous_rear_clear_margin_m / denominator, 0.0, 1.0) : 1.0;
      resolution.rear_clear_time_sec = previous_time_sec +
        interpolation * (step_end_sec - previous_time_sec);
      resolution.rear_clear_ego_distance_m = previous_ego_physical_distance_m +
        interpolation * (ego_physical_distance_m - previous_ego_physical_distance_m);
      resolution.rear_clear_mission_distance_m = previous_mission_distance_m +
        interpolation * (mission_distance_m - previous_mission_distance_m);
      resolution.rear_clear_ego_speed_mps = previous_ego_speed_mps +
        interpolation * (ego_speed_mps - previous_ego_speed_mps);
    }

    previous_time_sec = step_end_sec;
    previous_mission_distance_m = mission_distance_m;
    previous_ego_physical_distance_m = ego_physical_distance_m;
    previous_ego_speed_mps = ego_speed_mps;
    previous_lateral_margin_m = lateral_margin_m;
    previous_hard_margin_m = hard_margin_m;
    previous_rear_clear_margin_m = rear_clear_margin_m;
    const bool body_deadline_resolved =
      std::isfinite(resolution.body_clear_time_sec) &&
      std::isfinite(resolution.hard_distance_time_sec);
    const bool rear_clear_resolved =
      !request.rear_clear_prediction_enabled ||
      std::isfinite(resolution.rear_clear_time_sec);
    if (body_deadline_resolved && rear_clear_resolved)
    {
      break;
    }
  }

  resolution.ego_distance_at_horizon_m = ego_physical_distance_m;
  resolution.ego_speed_at_horizon_mps = ego_speed_mps;
  resolution.rear_clear_feasible =
    request.rear_clear_prediction_enabled &&
    std::isfinite(resolution.rear_clear_time_sec) &&
    resolution.rear_clear_time_sec <= request.maximum_time_sec + 1e-9;
  if (std::isfinite(resolution.body_clear_time_sec)) {
    resolution.deadline_slack_sec =
      std::isfinite(resolution.hard_distance_time_sec) ?
      resolution.hard_distance_time_sec - resolution.body_clear_time_sec :
      std::numeric_limits<double>::infinity();
    resolution.feasible =
      !std::isfinite(resolution.hard_distance_time_sec) ||
      resolution.body_clear_time_sec + request.deadline_margin_sec <=
      resolution.hard_distance_time_sec + 1e-9;
  }
  return resolution;
}

OvertakeDynamicPassDistanceResolution resolve_overtake_dynamic_pass_distance(
  const OvertakeDynamicPassDistanceRequest & request) noexcept
{
  OvertakeDynamicPassDistanceResolution resolution;
  const auto valid_limit = [](const double value) {
      return !std::isnan(value) && value >= 0.0;
    };
  if (
    !std::isfinite(request.shift_distance_m) || request.shift_distance_m < 0.0 ||
    !std::isfinite(request.minimum_pass_distance_m) ||
    request.minimum_pass_distance_m < 0.0 ||
    !std::isfinite(request.rear_clear_ego_distance_m) ||
    request.rear_clear_ego_distance_m < 0.0 ||
    !std::isfinite(request.rear_clear_ego_speed_mps) ||
    request.rear_clear_ego_speed_mps < 0.0 ||
    !std::isfinite(request.rear_clear_confirm_sec) || request.rear_clear_confirm_sec < 0.0 ||
    !std::isfinite(request.control_delay_sec) || request.control_delay_sec < 0.0 ||
    !std::isfinite(request.runtime_completion_reserve_distance_m) ||
    request.runtime_completion_reserve_distance_m < 0.0 ||
    !valid_limit(request.soft_pass_distance_limit_m) ||
    !valid_limit(request.hard_pass_distance_limit_m) ||
    request.hard_pass_distance_limit_m + 1e-9 < request.minimum_pass_distance_m)
  {
    return resolution;
  }

  resolution.valid = true;
  resolution.confirmation_reserve_distance_m =
    request.rear_clear_ego_speed_mps *
    (request.rear_clear_confirm_sec + request.control_delay_sec);
  resolution.required_pass_distance_m = std::max(
    request.minimum_pass_distance_m,
    std::max(0.0, request.rear_clear_ego_distance_m - request.shift_distance_m) +
    resolution.confirmation_reserve_distance_m +
    request.runtime_completion_reserve_distance_m);
  resolution.soft_limit_exceeded =
    resolution.required_pass_distance_m > request.soft_pass_distance_limit_m + 1e-9;
  resolution.feasible =
    resolution.required_pass_distance_m <= request.hard_pass_distance_limit_m + 1e-9;
  resolution.bounded_pass_distance_m = resolution.feasible ?
    resolution.required_pass_distance_m : request.hard_pass_distance_limit_m;
  return resolution;
}

OvertakeRuntimeContinuationReserveResolution resolve_overtake_runtime_continuation_reserve(
  const OvertakeRuntimeContinuationReserveRequest & request) noexcept
{
  OvertakeRuntimeContinuationReserveResolution resolution;
  if (
    !std::isfinite(request.configured_course_role_reserve_distance_m) ||
    request.configured_course_role_reserve_distance_m < 0.0 ||
    !std::isfinite(request.revalidation_lead_distance_m) ||
    request.revalidation_lead_distance_m < 0.0 ||
    !std::isfinite(request.completion_distance_margin_m) ||
    request.completion_distance_margin_m < 0.0)
  {
    return resolution;
  }

  resolution.valid = true;
  resolution.reserve_distance_m = std::max(
    request.configured_course_role_reserve_distance_m,
    request.revalidation_lead_distance_m + request.completion_distance_margin_m);
  return resolution;
}

DynamicPredictionTimingResolution resolve_dynamic_prediction_timing(
  const DynamicPredictionTimingRequest & request) noexcept
{
  DynamicPredictionTimingResolution resolution;
  if (
    !std::isfinite(request.planner_now_sec) ||
    !std::isfinite(request.source_age_sec) || request.source_age_sec < 0.0 ||
    !std::isfinite(request.prediction_horizon_sec) || request.prediction_horizon_sec < 0.0)
  {
    return resolution;
  }
  resolution.valid = true;
  resolution.prediction_epoch_sec = request.planner_now_sec - request.source_age_sec;
  resolution.expiry_sec =
    resolution.prediction_epoch_sec + request.prediction_horizon_sec;
  resolution.remaining_sec = std::max(0.0, resolution.expiry_sec - request.planner_now_sec);
  return resolution;
}

CommitClockProjectionResolution resolve_commit_clock_projection(
  const CommitClockProjectionRequest & request) noexcept
{
  CommitClockProjectionResolution resolution;
  if (
    !std::isfinite(request.planner_clock_start_sec) ||
    !std::isfinite(request.monotonic_start_sec) ||
    !std::isfinite(request.monotonic_commit_sec) ||
    request.monotonic_commit_sec < request.monotonic_start_sec)
  {
    return resolution;
  }
  resolution.elapsed_sec = request.monotonic_commit_sec - request.monotonic_start_sec;
  resolution.commit_clock_sec = request.planner_clock_start_sec + resolution.elapsed_sec;
  resolution.valid = std::isfinite(resolution.commit_clock_sec);
  return resolution;
}

RearClearReplanWindowResolution resolve_rear_clear_replan_window(
  const RearClearReplanWindowRequest & request) noexcept
{
  RearClearReplanWindowResolution resolution;
  if (
    !std::isfinite(request.static_valid_until_pass_m) ||
    request.static_valid_until_pass_m < 0.0 ||
    !std::isfinite(request.pass_traveled_m) || request.pass_traveled_m < 0.0 ||
    !std::isfinite(request.revalidation_lead_distance_m) ||
    request.revalidation_lead_distance_m < 0.0)
  {
    return resolution;
  }

  resolution.valid = true;
  resolution.remaining_committed_distance_m = std::max(
    0.0, request.static_valid_until_pass_m - request.pass_traveled_m);
  if (!request.prediction_checked || !request.prediction_feasible) {
    return resolution;
  }
  if (
    !std::isfinite(request.required_rear_clear_pass_m) ||
    request.required_rear_clear_pass_m < 0.0)
  {
    return RearClearReplanWindowResolution{};
  }

  resolution.beyond_committed_horizon =
    request.required_rear_clear_pass_m >
    request.static_valid_until_pass_m + 1e-9;
  resolution.replan_due =
    resolution.beyond_committed_horizon &&
    resolution.remaining_committed_distance_m <=
    request.revalidation_lead_distance_m + 1e-9;
  return resolution;
}

PassHorizonAction resolve_pass_horizon_action(
  const PassHorizonDecisionRequest & request) noexcept
{
  const auto non_negative = [](const double value) {
      return std::isfinite(value) && value >= 0.0;
    };
  const auto non_negative_limit = [](const double value) {
      return !std::isnan(value) && value >= 0.0;
    };
  if (!request.enabled || !request.pass_active) {
    return PassHorizonAction::Keep;
  }
  if (
    !non_negative(request.pass_traveled_m) || !non_negative(request.pass_elapsed_sec) ||
    !non_negative_limit(request.static_valid_until_pass_m) ||
    !non_negative_limit(request.dynamic_valid_until_pass_m) ||
    !std::isfinite(request.dynamic_time_remaining_sec) ||
    !non_negative_limit(request.absolute_distance_limit_m) ||
    !non_negative_limit(request.absolute_time_limit_sec) ||
    !non_negative(request.revalidation_lead_distance_m) ||
    !non_negative(request.revalidation_lead_time_sec) ||
    !non_negative(request.hold_elapsed_sec) || !non_negative(request.hold_traveled_m) ||
    !non_negative(request.hold_max_sec) || !non_negative(request.hold_max_distance_m) ||
    request.extension_count < 0 || request.maximum_extension_count < 0)
  {
    return PassHorizonAction::Abort;
  }
  if (request.rear_clear_confirmed && request.return_corridor_available) {
    return PassHorizonAction::Return;
  }
  if (
    request.pass_traveled_m >= request.absolute_distance_limit_m - 1e-9 ||
    request.pass_elapsed_sec >= request.absolute_time_limit_sec - 1e-9)
  {
    return PassHorizonAction::Abort;
  }

  const bool hold_expired = request.hold_active &&
    (request.hold_elapsed_sec >= request.hold_max_sec - 1e-9 ||
    request.hold_traveled_m >= request.hold_max_distance_m - 1e-9);
  if (request.hold_active) {
    return !hold_expired && request.short_horizon_safe ?
      PassHorizonAction::EnterHold : PassHorizonAction::Abort;
  }

  const double effective_valid_until_m = std::min(
    request.static_valid_until_pass_m, request.dynamic_valid_until_pass_m);
  const double distance_slack_m = effective_valid_until_m - request.pass_traveled_m;
  const bool revalidation_due =
    request.predicted_overlap_replan_required ||
    request.rear_clear_replan_required ||
    distance_slack_m <= request.revalidation_lead_distance_m + 1e-9 ||
    request.dynamic_time_remaining_sec <= request.revalidation_lead_time_sec + 1e-9;
  if (!revalidation_due) {
    return PassHorizonAction::Keep;
  }
  if (request.extension_count < request.maximum_extension_count) {
    return PassHorizonAction::RequestSameSideExtension;
  }
  if (
    request.short_horizon_safe &&
    request.rear_clear_replan_required &&
    request.longitudinal_refresh_available)
  {
    return PassHorizonAction::RequestLongitudinalRefresh;
  }
  const bool admitted_static_path_remaining =
    request.static_valid_until_pass_m > request.pass_traveled_m + 1e-9;
  if (
    request.short_horizon_safe &&
    request.longitudinal_refresh_available &&
    !request.predicted_overlap_replan_required &&
    admitted_static_path_remaining)
  {
    // A fresh measured-speed rollout still fits inside the immutable lateral
    // path. Consuming the admitted static horizon is not another geometric
    // extension and must not force SafeSeparation merely because the short
    // prediction TTL entered its proactive lead window.
    return PassHorizonAction::Keep;
  }
  return request.short_horizon_safe ?
    PassHorizonAction::EnterHold : PassHorizonAction::Abort;
}

bool can_lease_stopped_side_pass_prediction(
  const StoppedSidePassPredictionLeaseRequest & request) noexcept
{
  const auto finite_non_negative = [](const double value) {
      return std::isfinite(value) && value >= 0.0;
    };
  const auto valid_limit = [](const double value) {
      return !std::isnan(value) && value >= 0.0;
    };
  if (
    !finite_non_negative(request.maximum_stopped_target_speed_mps) ||
    !finite_non_negative(request.maximum_absolute_target_longitudinal_m) ||
    !finite_non_negative(request.target_observation_age_sec) ||
    !finite_non_negative(request.last_clear_prediction_age_sec) ||
    !finite_non_negative(request.lease_elapsed_sec) ||
    !finite_non_negative(request.lease_traveled_m) ||
    !finite_non_negative(request.maximum_lease_sec) ||
    !finite_non_negative(request.maximum_lease_distance_m) ||
    !finite_non_negative(request.pass_elapsed_sec) ||
    !finite_non_negative(request.pass_traveled_m) ||
    !valid_limit(request.absolute_pass_time_limit_sec) ||
    !valid_limit(request.absolute_pass_distance_limit_m))
  {
    return false;
  }
  if (
    !request.enabled || !request.pass_active || !request.mission_path_frozen ||
    request.refresh_failure_reason != PassRefreshFailureReason::TargetPredictionUnavailable ||
    !request.target_continuous || !request.course_progress_accepted ||
    !request.execution_corridor_clear ||
    !request.current_body_footprints_separated ||
    request.actual_wall_physical_contact || request.actual_wall_margin_blocked ||
    request.actual_wall_sample_unavailable || request.emergency_brake ||
    request.solver_recovery_active || !std::isfinite(request.target_speed_mps) ||
    !std::isfinite(request.target_longitudinal_m))
  {
    return false;
  }

  return request.target_speed_mps <=
         request.maximum_stopped_target_speed_mps + 1e-9 &&
         std::abs(request.target_longitudinal_m) <=
         request.maximum_absolute_target_longitudinal_m + 1e-9 &&
         request.target_observation_age_sec < request.maximum_lease_sec - 1e-9 &&
         request.last_clear_prediction_age_sec < request.maximum_lease_sec - 1e-9 &&
         request.lease_elapsed_sec < request.maximum_lease_sec - 1e-9 &&
         request.lease_traveled_m < request.maximum_lease_distance_m - 1e-9 &&
         request.pass_elapsed_sec < request.absolute_pass_time_limit_sec - 1e-9 &&
         request.pass_traveled_m < request.absolute_pass_distance_limit_m - 1e-9;
}

const char * to_string(const PassRefreshFailureReason reason) noexcept
{
  switch (reason) {
    case PassRefreshFailureReason::None:
      return "none";
    case PassRefreshFailureReason::TargetPredictionUnavailable:
      return "target_prediction_unavailable";
    case PassRefreshFailureReason::TargetDiscontinuous:
      return "target_discontinuous";
    case PassRefreshFailureReason::CourseProgressRejected:
      return "course_progress_rejected";
    case PassRefreshFailureReason::ExecutionCorridorBlocked:
      return "execution_corridor_blocked";
    case PassRefreshFailureReason::PredictedOverlap:
      return "predicted_overlap";
    case PassRefreshFailureReason::InvalidInput:
      return "invalid_input";
    case PassRefreshFailureReason::AbsoluteBudgetExhausted:
      return "absolute_budget_exhausted";
    case PassRefreshFailureReason::WallOrBodyFault:
      return "wall_or_body_fault";
    case PassRefreshFailureReason::Other:
      return "other";
  }
  return "unknown";
}

bool can_hold_pass_during_refresh_replan(
  const PassRefreshReplanGraceRequest & request) noexcept
{
  const auto finite_non_negative = [](const double value) {
      return std::isfinite(value) && value >= 0.0;
    };
  const auto valid_limit = [](const double value) {
      return !std::isnan(value) && value >= 0.0;
    };
  if (
    !finite_non_negative(request.grace_elapsed_sec) ||
    !finite_non_negative(request.grace_traveled_m) ||
    !finite_non_negative(request.maximum_grace_sec) ||
    !finite_non_negative(request.maximum_grace_distance_m) ||
    !finite_non_negative(request.pass_elapsed_sec) ||
    !finite_non_negative(request.pass_traveled_m) ||
    !valid_limit(request.static_valid_until_pass_m) ||
    !valid_limit(request.absolute_pass_time_limit_sec) ||
    !valid_limit(request.absolute_pass_distance_limit_m))
  {
    return false;
  }

  const bool replannable_failure =
    request.refresh_failure_reason == PassRefreshFailureReason::ExecutionCorridorBlocked ||
    request.refresh_failure_reason == PassRefreshFailureReason::WallOrBodyFault ||
    request.refresh_failure_reason == PassRefreshFailureReason::Other;
  if (
    !request.enabled || !request.pass_active || !request.mission_path_frozen ||
    !replannable_failure || !request.target_continuous ||
    !request.course_progress_accepted || !request.fresh_target_prediction_available ||
    !request.short_horizon_safe || !request.current_body_footprints_separated ||
    !request.footprint_prediction_valid ||
    !request.predicted_body_footprint_sweep_separated ||
    request.predicted_overlap_replan_required || request.execution_corridor_blocked ||
    request.actual_wall_physical_contact || request.actual_wall_margin_blocked ||
    request.actual_wall_sample_unavailable || request.emergency_brake ||
    request.solver_recovery_active)
  {
    return false;
  }

  return request.maximum_grace_sec > 1e-9 &&
         request.maximum_grace_distance_m > 1e-9 &&
         request.grace_elapsed_sec < request.maximum_grace_sec - 1e-9 &&
         request.grace_traveled_m < request.maximum_grace_distance_m - 1e-9 &&
         request.pass_traveled_m < request.static_valid_until_pass_m - 1e-9 &&
         request.pass_elapsed_sec < request.absolute_pass_time_limit_sec - 1e-9 &&
         request.pass_traveled_m < request.absolute_pass_distance_limit_m - 1e-9;
}

StoppedPredictionLeaseSpeedResolution resolve_stopped_prediction_lease_speed(
  const StoppedPredictionLeaseSpeedRequest & request) noexcept
{
  StoppedPredictionLeaseSpeedResolution resolution;
  resolution.target_velocity_reference_mps =
    request.existing_target_velocity_reference_mps;
  if (!request.active) {
    resolution.valid =
      !std::isnan(request.existing_target_velocity_reference_mps) &&
      request.existing_target_velocity_reference_mps >= 0.0;
    return resolution;
  }

  resolution.active = true;
  const bool valid =
    std::isfinite(request.current_speed_mps) && request.current_speed_mps >= 0.0 &&
    std::isfinite(request.lease_start_speed_mps) &&
    request.lease_start_speed_mps >= 0.0 &&
    !std::isnan(request.existing_target_velocity_reference_mps) &&
    request.existing_target_velocity_reference_mps >= 0.0 &&
    !std::isnan(request.maximum_speed_mps) && request.maximum_speed_mps >= 0.0;
  if (!valid) {
    resolution.target_velocity_reference_mps = 0.0;
    return resolution;
  }

  resolution.valid = true;
  resolution.target_velocity_reference_mps = std::min({
    request.current_speed_mps,
    request.lease_start_speed_mps,
    request.existing_target_velocity_reference_mps,
    request.maximum_speed_mps});
  return resolution;
}

PassContinuationPreflightPolicyResolution resolve_pass_continuation_preflight_policy(
  const PassContinuationPreflightPolicyRequest & request) noexcept
{
  PassContinuationPreflightPolicyResolution resolution;
  // This resolver is used only after a mission has entered Pass. Validate the
  // committed side through rear-clear here; Return is rebuilt from the actual
  // state after rear-clear instead of invalidating a safe ongoing pass early.
  resolution.include_return_path = false;
  resolution.footprint_continuation_active =
    request.longitudinal_refresh &&
    request.short_horizon_safe &&
    request.target_observation_continuous &&
    request.current_body_footprints_separated &&
    request.predicted_body_footprint_available &&
    (request.predicted_body_footprint_sweep_separated ||
    !request.predicted_body_overlap_confirmed);
  resolution.enforce_target_center_separation =
    !resolution.footprint_continuation_active;
  resolution.enforce_outer_role_continuity =
    !resolution.footprint_continuation_active;
  return resolution;
}

PassOuterHorizonResolution evaluate_pass_outer_horizon(
  const PassOuterHorizonRequest & request) noexcept
{
  PassOuterHorizonResolution resolution;
  if (!request.enabled) {
    resolution.valid = true;
    resolution.feasible = true;
    resolution.outer_strategy = request.outer_strategy_committed;
    return resolution;
  }
  if (
    (request.pass_side_sign != -1 && request.pass_side_sign != 1) ||
    !std::isfinite(request.significant_curvature_radpm) ||
    request.significant_curvature_radpm < 0.0 ||
    !std::isfinite(request.validation_distance_m) ||
    request.validation_distance_m < 0.0)
  {
    return resolution;
  }

  double previous_distance_m = -std::numeric_limits<double>::infinity();
  for (const auto & sample : request.samples) {
    if (
      !std::isfinite(sample.path_distance_m) || sample.path_distance_m < 0.0 ||
      !std::isfinite(sample.reference_curvature_radpm) ||
      sample.path_distance_m + 1e-9 < previous_distance_m)
    {
      return PassOuterHorizonResolution{};
    }
    previous_distance_m = sample.path_distance_m;
  }

  resolution.valid = true;
  resolution.feasible = true;
  resolution.outer_strategy = request.outer_strategy_committed;
  bool strategy_classified = request.outer_strategy_committed;
  for (const auto & sample : request.samples) {
    if (sample.path_distance_m > request.validation_distance_m + 1e-9) {
      break;
    }
    if (
      std::abs(sample.reference_curvature_radpm) <=
      request.significant_curvature_radpm + 1e-12)
    {
      continue;
    }
    if (!std::isfinite(resolution.first_significant_curve_distance_m)) {
      resolution.first_significant_curve_distance_m = sample.path_distance_m;
    }
    const int inner_side = sample.reference_curvature_radpm > 0.0 ? 1 : -1;
    if (!strategy_classified && request.infer_outer_strategy) {
      resolution.outer_strategy = request.pass_side_sign != inner_side;
      strategy_classified = true;
      if (!resolution.outer_strategy) {
        // This mission deliberately starts as an inside attack. The outside
        // continuity contract does not silently turn that separate strategy
        // into a rejection rule.
        return resolution;
      }
    }
    if (resolution.outer_strategy && request.pass_side_sign == inner_side) {
      resolution.feasible = false;
      resolution.role_reversal = true;
      resolution.first_role_reversal_distance_m = sample.path_distance_m;
      return resolution;
    }
  }
  return resolution;
}

const char * to_string(const PassSideCourseRole role) noexcept
{
  switch (role) {
    case PassSideCourseRole::Inner:
      return "inner";
    case PassSideCourseRole::Outer:
      return "outer";
    case PassSideCourseRole::Unknown:
    default:
      return "unknown";
  }
}

PassSideRearClearRoleResolution evaluate_pass_side_rear_clear_role(
  const PassSideRearClearRoleRequest & request) noexcept
{
  PassSideRearClearRoleResolution resolution;
  if (
    (request.pass_side_sign != -1 && request.pass_side_sign != 1) ||
    !std::isfinite(request.significant_curvature_radpm) ||
    request.significant_curvature_radpm < 0.0 ||
    !std::isfinite(request.predicted_rear_clear_distance_m) ||
    request.predicted_rear_clear_distance_m < 0.0 ||
    !std::isfinite(request.reserve_distance_m) || request.reserve_distance_m < 0.0)
  {
    return resolution;
  }

  const double evaluation_distance_m =
    request.predicted_rear_clear_distance_m + request.reserve_distance_m;
  double previous_distance_m = -std::numeric_limits<double>::infinity();
  for (const auto & sample : request.samples) {
    if (
      !std::isfinite(sample.path_distance_m) || sample.path_distance_m < 0.0 ||
      !std::isfinite(sample.reference_curvature_radpm) ||
      sample.path_distance_m + 1e-9 < previous_distance_m)
    {
      return PassSideRearClearRoleResolution{};
    }
    previous_distance_m = sample.path_distance_m;
  }
  if (
    request.samples.empty() ||
    request.samples.back().path_distance_m + 1e-9 < evaluation_distance_m)
  {
    return resolution;
  }

  resolution.valid = true;
  PassSideCourseRole current_role = PassSideCourseRole::Unknown;
  for (const auto & sample : request.samples) {
    if (sample.path_distance_m > evaluation_distance_m + 1e-9) {
      break;
    }
    if (
      std::abs(sample.reference_curvature_radpm) <=
      request.significant_curvature_radpm + 1e-12)
    {
      continue;
    }

    const int inner_side = sample.reference_curvature_radpm > 0.0 ? 1 : -1;
    const PassSideCourseRole sample_role = request.pass_side_sign == inner_side ?
      PassSideCourseRole::Inner : PassSideCourseRole::Outer;
    if (resolution.entry_role == PassSideCourseRole::Unknown) {
      resolution.entry_role = sample_role;
    } else if (
      current_role != PassSideCourseRole::Unknown && sample_role != current_role &&
      !std::isfinite(resolution.first_role_reversal_distance_m))
    {
      resolution.first_role_reversal_distance_m = sample.path_distance_m;
    }
    current_role = sample_role;
    resolution.rear_clear_role = sample_role;
  }

  resolution.outer_to_inner_before_rear_clear =
    resolution.entry_role == PassSideCourseRole::Outer &&
    resolution.rear_clear_role == PassSideCourseRole::Inner;
  resolution.inner_to_outer_at_rear_clear =
    resolution.entry_role == PassSideCourseRole::Inner &&
    resolution.rear_clear_role == PassSideCourseRole::Outer;
  return resolution;
}

ContinuousOuterReplanResolution evaluate_continuous_outer_replan(
  const ContinuousOuterReplanRequest & request) noexcept
{
  ContinuousOuterReplanResolution resolution;
  if (!request.enabled) {
    resolution.valid = true;
    return resolution;
  }
  if (
    (request.current_side_sign != -1 && request.current_side_sign != 1) ||
    !std::isfinite(request.significant_curvature_radpm) ||
    request.significant_curvature_radpm < 0.0 ||
    !std::isfinite(request.lookahead_distance_m) ||
    request.lookahead_distance_m <= 0.0 ||
    !std::isfinite(request.minimum_opposite_curve_distance_m) ||
    request.minimum_opposite_curve_distance_m <= 0.0)
  {
    return resolution;
  }

  double previous_distance_m = -std::numeric_limits<double>::infinity();
  for (const auto & sample : request.samples) {
    if (
      !std::isfinite(sample.path_distance_m) || sample.path_distance_m < 0.0 ||
      !std::isfinite(sample.reference_curvature_radpm) ||
      sample.path_distance_m + 1e-9 < previous_distance_m)
    {
      return ContinuousOuterReplanResolution{};
    }
    previous_distance_m = sample.path_distance_m;
  }

  resolution.valid = true;
  for (std::size_t start_index = 0U; start_index < request.samples.size(); ++start_index) {
    const auto & start = request.samples[start_index];
    if (start.path_distance_m > request.lookahead_distance_m + 1e-9) {
      break;
    }
    if (
      std::abs(start.reference_curvature_radpm) <=
      request.significant_curvature_radpm + 1e-12)
    {
      continue;
    }
    const int inner_side = start.reference_curvature_radpm > 0.0 ? 1 : -1;
    const int desired_outer_side = -inner_side;
    if (desired_outer_side == request.current_side_sign) {
      continue;
    }

    double confirmed_until_m = start.path_distance_m;
    for (std::size_t i = start_index + 1U; i < request.samples.size(); ++i) {
      const auto & sample = request.samples[i];
      if (sample.path_distance_m > request.lookahead_distance_m + 1e-9) {
        confirmed_until_m = request.lookahead_distance_m;
        break;
      }
      if (
        std::abs(sample.reference_curvature_radpm) >
        request.significant_curvature_radpm + 1e-12)
      {
        const int sample_inner_side =
          sample.reference_curvature_radpm > 0.0 ? 1 : -1;
        if (-sample_inner_side != desired_outer_side) {
          break;
        }
      }
      confirmed_until_m = sample.path_distance_m;
    }
    const double confirmed_distance_m = std::max(
      0.0, confirmed_until_m - start.path_distance_m);
    if (
      confirmed_distance_m + 1e-9 <
      request.minimum_opposite_curve_distance_m)
    {
      continue;
    }

    resolution.replan_required = true;
    resolution.desired_outer_side_sign = desired_outer_side;
    resolution.first_opposite_curve_distance_m = start.path_distance_m;
    resolution.confirmed_opposite_curve_distance_m = confirmed_distance_m;
    return resolution;
  }
  return resolution;
}

ScheduledOuterTransitionResolution resolve_scheduled_outer_transition(
  const ScheduledOuterTransitionRequest & request) noexcept
{
  ScheduledOuterTransitionResolution resolution;
  if (!request.enabled) {
    resolution.valid = true;
    resolution.feasible = true;
    return resolution;
  }
  if (
    (request.current_side_sign != -1 && request.current_side_sign != 1) ||
    !std::isfinite(request.minimum_shift_distance_m) ||
    request.minimum_shift_distance_m <= 0.0 ||
    !std::isfinite(request.maximum_shift_distance_m) ||
    request.maximum_shift_distance_m < request.minimum_shift_distance_m)
  {
    return resolution;
  }

  resolution.valid = true;
  if (!request.outer_strategy || !request.role_reversal) {
    resolution.feasible = true;
    return resolution;
  }
  if (
    !std::isfinite(request.body_clear_distance_m) ||
    request.body_clear_distance_m < 0.0 ||
    !std::isfinite(request.role_reversal_distance_m) ||
    request.role_reversal_distance_m < 0.0)
  {
    resolution.valid = false;
    return resolution;
  }

  resolution.transition_required = true;
  resolution.desired_side_sign = -request.current_side_sign;
  resolution.deadline_distance_m = request.role_reversal_distance_m;
  const double maximum_distance_start_m = std::max(
    0.0,
    request.role_reversal_distance_m - request.maximum_shift_distance_m);
  resolution.start_distance_m = std::max(
    request.body_clear_distance_m, maximum_distance_start_m);
  resolution.available_shift_distance_m = std::max(
    0.0,
    resolution.deadline_distance_m - resolution.start_distance_m);
  resolution.feasible =
    resolution.available_shift_distance_m + 1e-9 >=
    request.minimum_shift_distance_m;
  return resolution;
}

ScheduledOuterTransitionRuntimeBudgetResolution
resolve_scheduled_outer_transition_runtime_budget(
  const ScheduledOuterTransitionRuntimeBudgetRequest & request) noexcept
{
  ScheduledOuterTransitionRuntimeBudgetResolution resolution;
  if (
    !std::isfinite(request.remaining_window_distance_m) ||
    request.remaining_window_distance_m < 0.0 ||
    !std::isfinite(request.admission_nominal_shift_distance_m) ||
    request.admission_nominal_shift_distance_m < 0.0 ||
    !std::isfinite(request.configured_maximum_shift_distance_m) ||
    request.configured_maximum_shift_distance_m < 0.0 ||
    std::isnan(request.remaining_absolute_pass_distance_m) ||
    request.remaining_absolute_pass_distance_m < 0.0 ||
    !std::isfinite(request.minimum_shift_distance_m) ||
    request.minimum_shift_distance_m <= 0.0 ||
    !std::isfinite(request.remaining_pass_reserve_m) ||
    request.remaining_pass_reserve_m < 0.0)
  {
    return resolution;
  }

  resolution.valid = true;
  resolution.admission_nominal_shift_distance_m =
    request.admission_nominal_shift_distance_m;
  const double absolute_budget =
    std::isinf(request.remaining_absolute_pass_distance_m) ?
    std::numeric_limits<double>::infinity() :
    std::max(
    0.0,
    request.remaining_absolute_pass_distance_m - request.remaining_pass_reserve_m);
  resolution.available_shift_distance_m = std::min({
    request.remaining_window_distance_m,
    request.configured_maximum_shift_distance_m,
    absolute_budget});
  resolution.feasible =
    resolution.available_shift_distance_m + 1e-9 >=
    request.minimum_shift_distance_m;
  return resolution;
}

FrozenOuterTransitionGoalResolution resolve_frozen_outer_transition_goal(
  const FrozenOuterTransitionGoalRequest & request) noexcept
{
  FrozenOuterTransitionGoalResolution resolution;
  if (
    (request.desired_side_sign != -1 && request.desired_side_sign != 1) ||
    !std::isfinite(request.source_goal_m) ||
    !std::isfinite(request.feasible_lower_m) ||
    !std::isfinite(request.feasible_upper_m) ||
    request.feasible_upper_m < request.feasible_lower_m ||
    !std::isfinite(request.minimum_role_offset_m) ||
    request.minimum_role_offset_m < 0.0 ||
    std::isnan(request.maximum_lateral_adjustment_m) ||
    request.maximum_lateral_adjustment_m < 0.0)
  {
    return resolution;
  }

  resolution.valid = true;
  double role_lower = request.feasible_lower_m;
  double role_upper = request.feasible_upper_m;
  if (request.desired_side_sign > 0) {
    role_lower = std::max(role_lower, request.minimum_role_offset_m);
  } else {
    role_upper = std::min(role_upper, -request.minimum_role_offset_m);
  }
  if (role_upper + 1e-9 < role_lower) {
    return resolution;
  }

  const double mirrored_goal = -request.source_goal_m;
  resolution.goal_m = std::clamp(mirrored_goal, role_lower, role_upper);
  resolution.lateral_adjustment_m = std::abs(
    resolution.goal_m - request.source_goal_m);
  const bool role_satisfied =
    static_cast<double>(request.desired_side_sign) * resolution.goal_m + 1e-9 >=
    request.minimum_role_offset_m;
  resolution.feasible = role_satisfied &&
    resolution.lateral_adjustment_m <=
    request.maximum_lateral_adjustment_m + 1e-9;
  return resolution;
}

DynamicCorridorGoalResolution resolve_dynamic_corridor_goal(
  const DynamicCorridorGoalRequest & request) noexcept
{
  DynamicCorridorGoalResolution resolution;
  const auto finite_non_negative = [](const double value) {
      return std::isfinite(value) && value >= 0.0;
    };
  if (!request.enabled || !request.pass_active) {
    resolution.valid = true;
    return resolution;
  }
  if (
    (request.pass_side_sign != -1 && request.pass_side_sign != 1) ||
    !std::isfinite(request.current_goal_m) ||
    !std::isfinite(request.desired_goal_m) ||
    !std::isfinite(request.feasible_lower_m) ||
    !std::isfinite(request.feasible_upper_m) ||
    request.feasible_upper_m + 1e-9 < request.feasible_lower_m ||
    !finite_non_negative(request.minimum_role_offset_m) ||
    !finite_non_negative(request.minimum_adjustment_m) ||
    !finite_non_negative(request.maximum_adjustment_m) ||
    request.maximum_adjustment_m + 1e-9 < request.minimum_adjustment_m)
  {
    return resolution;
  }

  double role_lower = request.feasible_lower_m;
  double role_upper = request.feasible_upper_m;
  if (request.pass_side_sign > 0) {
    role_lower = std::max(role_lower, request.minimum_role_offset_m);
  } else {
    role_upper = std::min(role_upper, -request.minimum_role_offset_m);
  }
  resolution.valid = true;
  if (role_upper + 1e-9 < role_lower) {
    return resolution;
  }

  const double bounded_desired = std::clamp(
    request.desired_goal_m, role_lower, role_upper);
  const double bounded_delta = std::clamp(
    bounded_desired - request.current_goal_m,
    -request.maximum_adjustment_m, request.maximum_adjustment_m);
  resolution.goal_m = std::clamp(
    request.current_goal_m + bounded_delta, role_lower, role_upper);
  resolution.lateral_adjustment_m = std::abs(
    resolution.goal_m - request.current_goal_m);
  resolution.feasible =
    static_cast<double>(request.pass_side_sign) * resolution.goal_m + 1e-9 >=
    request.minimum_role_offset_m &&
    resolution.lateral_adjustment_m <= request.maximum_adjustment_m + 1e-9;
  resolution.update_required = resolution.feasible &&
    resolution.lateral_adjustment_m + 1e-9 >= request.minimum_adjustment_m;
  return resolution;
}

SameSideExtensionCommitResolution evaluate_same_side_extension_commit(
  const SameSideExtensionCommitRequest & request) noexcept
{
  SameSideExtensionCommitResolution resolution;
  const auto finite_non_negative = [](const double value) {
      return std::isfinite(value) && value >= 0.0;
    };
  const auto valid_limit = [](const double value) {
      return !std::isnan(value) && value >= 0.0;
    };
  if (!request.pass_or_hold_active) {
    resolution.reason = SameSideExtensionCommitReason::PassInactive;
    return resolution;
  }
  if (!request.target_matches) {
    resolution.reason = SameSideExtensionCommitReason::TargetMismatch;
    return resolution;
  }
  if (!request.side_matches) {
    resolution.reason = SameSideExtensionCommitReason::SideMismatch;
    return resolution;
  }
  if (!request.replacement_path_valid) {
    resolution.reason = SameSideExtensionCommitReason::ReplacementPathInvalid;
    return resolution;
  }
  if (
    request.source_generation == 0U ||
    request.source_generation != request.current_generation)
  {
    resolution.reason = SameSideExtensionCommitReason::GenerationMismatch;
    return resolution;
  }
  if (
    !std::isfinite(request.planner_generated_at_sec) ||
    !std::isfinite(request.commit_now_sec) ||
    !finite_non_negative(request.planner_result_max_age_sec) ||
    !std::isfinite(request.prediction_expiry_sec) ||
    !finite_non_negative(request.current_effective_valid_until_pass_m) ||
    !finite_non_negative(request.current_pass_hold_distance_m) ||
    !finite_non_negative(request.replacement_static_valid_until_pass_m) ||
    !finite_non_negative(request.replacement_dynamic_valid_until_pass_m) ||
    !finite_non_negative(request.replacement_pass_hold_distance_m) ||
    !valid_limit(request.absolute_pass_distance_limit_m) ||
    !std::isfinite(request.current_goal_lateral_m) ||
    !std::isfinite(request.replacement_goal_lateral_m) ||
    !valid_limit(request.maximum_lateral_adjustment_m))
  {
    resolution.reason = SameSideExtensionCommitReason::InvalidInput;
    return resolution;
  }
  const double planner_result_age_sec =
    request.commit_now_sec - request.planner_generated_at_sec;
  if (
    planner_result_age_sec < -1e-9 ||
    planner_result_age_sec > request.planner_result_max_age_sec + 1e-9)
  {
    resolution.reason = SameSideExtensionCommitReason::PlannerResultStale;
    return resolution;
  }
  if (request.commit_now_sec >= request.prediction_expiry_sec - 1e-9) {
    resolution.reason = SameSideExtensionCommitReason::PredictionExpired;
    return resolution;
  }
  if (
    request.replacement_pass_hold_distance_m >
    request.absolute_pass_distance_limit_m + 1e-9)
  {
    resolution.reason = SameSideExtensionCommitReason::AbsoluteDistanceExceeded;
    return resolution;
  }
  if (
    request.require_pass_distance_advance &&
    request.replacement_pass_hold_distance_m <=
    request.current_pass_hold_distance_m + 1e-9)
  {
    resolution.reason = SameSideExtensionCommitReason::PassDistanceNotAdvanced;
    return resolution;
  }
  if (
    request.replacement_pass_hold_distance_m + 1e-9 <
    request.replacement_static_valid_until_pass_m)
  {
    resolution.reason = SameSideExtensionCommitReason::StaticCoverageInsufficient;
    return resolution;
  }
  if (
    std::abs(
      request.replacement_goal_lateral_m - request.current_goal_lateral_m) >
    request.maximum_lateral_adjustment_m + 1e-9)
  {
    resolution.reason = SameSideExtensionCommitReason::LateralAdjustmentExceeded;
    return resolution;
  }

  // Dynamic validity is a short V2X prediction window regenerated on every
  // observation. Freshness is enforced by prediction_expiry_sec above. Do not
  // reject a statically advancing replacement merely because its distance
  // horizon is a few centimetres shorter at the newer ego speed.
  resolution.accepted = true;
  resolution.reason = SameSideExtensionCommitReason::Accepted;
  return resolution;
}

bool can_commit_same_side_extension(
  const SameSideExtensionCommitRequest & request) noexcept
{
  return evaluate_same_side_extension_commit(request).accepted;
}

const char * to_string(const SameSideExtensionCommitReason reason) noexcept
{
  switch (reason) {
    case SameSideExtensionCommitReason::Accepted:
      return "accepted";
    case SameSideExtensionCommitReason::PassInactive:
      return "Pass inactive";
    case SameSideExtensionCommitReason::TargetMismatch:
      return "target mismatch";
    case SameSideExtensionCommitReason::SideMismatch:
      return "side mismatch";
    case SameSideExtensionCommitReason::ReplacementPathInvalid:
      return "replacement path invalid";
    case SameSideExtensionCommitReason::GenerationMismatch:
      return "generation mismatch";
    case SameSideExtensionCommitReason::InvalidInput:
      return "invalid commit input";
    case SameSideExtensionCommitReason::PlannerResultStale:
      return "planner result stale";
    case SameSideExtensionCommitReason::PredictionExpired:
      return "prediction expired";
    case SameSideExtensionCommitReason::AbsoluteDistanceExceeded:
      return "absolute distance exceeded";
    case SameSideExtensionCommitReason::PassDistanceNotAdvanced:
      return "Pass distance not advanced";
    case SameSideExtensionCommitReason::StaticCoverageInsufficient:
      return "static coverage insufficient";
    case SameSideExtensionCommitReason::LateralAdjustmentExceeded:
      return "lateral adjustment exceeded";
  }
  return "unknown";
}

MissionTotalBudgetResolution resolve_mission_total_budget(
  const MissionTotalBudgetRequest & request) noexcept
{
  MissionTotalBudgetResolution resolution;
  if (!request.enabled || !request.mission_active) {
    return resolution;
  }
  if (
    !std::isfinite(request.elapsed_sec) || request.elapsed_sec < 0.0 ||
    std::isnan(request.maximum_duration_sec) ||
    request.maximum_duration_sec < 0.0)
  {
    resolution.expired = true;
    resolution.action = MissionTotalBudgetAction::Abort;
    return resolution;
  }
  if (request.elapsed_sec + 1e-9 < request.maximum_duration_sec) {
    resolution.action = MissionTotalBudgetAction::Keep;
    return resolution;
  }
  resolution.expired = true;
  resolution.action = request.rear_clear_confirmed && request.return_corridor_available ?
    MissionTotalBudgetAction::Return : MissionTotalBudgetAction::Abort;
  return resolution;
}

double resolve_mission_total_start_sec(
  const bool mission_active, const double current_start_sec, const double now_sec) noexcept
{
  if (!mission_active || std::isfinite(current_start_sec)) {
    return current_start_sec;
  }
  return std::isfinite(now_sec) ? now_sec : current_start_sec;
}

CommittedPassForwardCompletionResolution resolve_committed_pass_forward_completion(
  const CommittedPassForwardCompletionRequest & request) noexcept
{
  CommittedPassForwardCompletionResolution resolution;
  const auto finite_non_negative = [](const double value) {
      return std::isfinite(value) && value >= 0.0;
    };
  const bool physical_runtime_guard =
    request.enabled && request.pass_active && request.minimum_motion_corridor_active &&
    request.prior_front_cap_release_active && request.target_seen &&
    request.target_continuity_valid && !request.target_position_jump &&
    !request.actual_wall_contact && !request.wall_sample_unavailable &&
    !request.target_pass_side_intrusion && !request.emergency_brake &&
    !request.solver_recovery_active &&
    std::isfinite(request.target_longitudinal_m);
  // Current overlap is independently debounced from future-sweep overlap.
  // Once tactical no-return has been crossed, a single body-boundary sample
  // must not make the physical guard fail before ContactContinuation can
  // classify a confirmed contact. This grace alone never acquires forward
  // completion; it only keeps the same-side SafeSeparation bridge available.
  resolution.current_overlap_grace_active =
    physical_runtime_guard && request.side_by_side_committed &&
    !request.current_body_footprints_separated &&
    !request.current_body_footprint_overlap_confirmed;
  const bool base_guard_without_predicted_geometry =
    physical_runtime_guard && request.footprint_prediction_valid &&
    finite_non_negative(request.maximum_front_distance_m) &&
    request.completion_prediction_valid &&
    finite_non_negative(request.predicted_required_forward_distance_m) &&
    finite_non_negative(request.predicted_completion_time_sec) &&
    finite_non_negative(request.maximum_completion_distance_m);
  resolution.predicted_overlap_grace_active =
    base_guard_without_predicted_geometry && request.already_latched &&
    !request.predicted_body_footprint_sweep_separated &&
    !request.predicted_body_footprint_overlap_confirmed;
  const bool predicted_geometry_acceptable =
    request.predicted_body_footprint_sweep_separated ||
    resolution.predicted_overlap_grace_active;
  const bool base_guard =
    base_guard_without_predicted_geometry && predicted_geometry_acceptable;
  const bool current_geometry_acceptable =
    request.current_body_footprints_separated ||
    resolution.current_overlap_grace_active;
  if (base_guard) {
    resolution.required_forward_distance_m =
      request.predicted_required_forward_distance_m;
    resolution.required_completion_time_sec = request.predicted_completion_time_sec;
    resolution.rear_clear_distance_feasible =
      request.completion_rear_clear_feasible &&
      resolution.required_forward_distance_m <=
      request.maximum_completion_distance_m + 1e-9;
  }
  resolution.active =
    base_guard && current_geometry_acceptable &&
    (request.already_latched ||
    (resolution.rear_clear_distance_feasible &&
    request.target_longitudinal_m <= request.maximum_front_distance_m + 1e-9));
  return resolution;
}

PassCompletionRolloutSpeedResolution resolve_pass_completion_rollout_speed(
  const PassCompletionRolloutSpeedRequest & request) noexcept
{
  PassCompletionRolloutSpeedResolution resolution;
  if (
    !std::isfinite(request.nominal_closing_speed_mps) ||
    request.nominal_closing_speed_mps < 0.0 ||
    !std::isfinite(request.target_speed_mps) || request.target_speed_mps < 0.0 ||
    !std::isfinite(request.maximum_ego_speed_mps) ||
    request.maximum_ego_speed_mps <= 0.0)
  {
    return resolution;
  }

  resolution.valid = true;
  resolution.closing_speed_mps = request.nominal_closing_speed_mps;
  const bool full_speed_coupling_allowed =
    request.enabled && request.pass_active &&
    request.full_speed_forward_escape_enabled && request.front_cap_released &&
    request.current_body_footprints_separated &&
    request.footprint_prediction_valid &&
    request.predicted_body_footprint_sweep_separated &&
    !request.execution_corridor_blocked && !request.hard_fault;
  if (full_speed_coupling_allowed) {
    resolution.full_speed_coupled = true;
    resolution.closing_speed_mps = std::max(
      resolution.closing_speed_mps,
      std::max(0.0, request.maximum_ego_speed_mps - request.target_speed_mps));
  }
  resolution.ego_speed_target_mps = std::min(
    request.maximum_ego_speed_mps,
    request.target_speed_mps + resolution.closing_speed_mps);
  return resolution;
}

DynamicCompletionExtensionResolution resolve_dynamic_completion_extension(
  const DynamicCompletionExtensionRequest & request) noexcept
{
  DynamicCompletionExtensionResolution resolution;
  const auto finite_non_negative = [](const double value) {
      return std::isfinite(value) && value >= 0.0;
    };
  const auto non_negative_bound = [](const double value) {
      return !std::isnan(value) && value >= 0.0;
    };
  if (
    !finite_non_negative(request.required_forward_distance_m) ||
    !finite_non_negative(request.forward_speed_mps) ||
    !finite_non_negative(request.absolute_elapsed_sec) ||
    !finite_non_negative(request.absolute_traveled_m) ||
    !non_negative_bound(request.absolute_maximum_duration_sec) ||
    !non_negative_bound(request.absolute_maximum_distance_m))
  {
    return resolution;
  }

  resolution.remaining_absolute_time_sec =
    std::isfinite(request.absolute_maximum_duration_sec) ?
    std::max(
    0.0, request.absolute_maximum_duration_sec - request.absolute_elapsed_sec) :
    std::numeric_limits<double>::infinity();
  resolution.remaining_absolute_distance_m =
    std::isfinite(request.absolute_maximum_distance_m) ?
    std::max(
    0.0, request.absolute_maximum_distance_m - request.absolute_traveled_m) :
    std::numeric_limits<double>::infinity();
  if (request.required_forward_distance_m <= 1e-9) {
    resolution.required_completion_time_sec = 0.0;
  } else if (request.forward_speed_mps > 1e-6) {
    resolution.required_completion_time_sec =
      request.required_forward_distance_m / request.forward_speed_mps;
  }
  resolution.allowed =
    request.enabled && request.forward_escape_allowed &&
    request.fresh_forward_progress && request.forward_completion_latched &&
    request.required_forward_distance_m <=
    resolution.remaining_absolute_distance_m + 1e-9 &&
    resolution.required_completion_time_sec <=
    resolution.remaining_absolute_time_sec + 1e-9;
  return resolution;
}

RearwardPassCompletionContextResolution resolve_rearward_pass_completion_context(
  const RearwardPassCompletionContextRequest & request) noexcept
{
  RearwardPassCompletionContextResolution resolution;
  const bool rearward_context_valid =
    request.pass_active &&
    request.commit_stage == PassCommitStage::SideBySideCommitted &&
    request.target_seen && request.target_matches &&
    request.target_continuity_valid &&
    std::isfinite(request.target_longitudinal_m) &&
    request.target_longitudinal_m <= 0.0;
  if (!rearward_context_valid) {
    return resolution;
  }

  resolution.rearward_target = true;
  resolution.contact_tail_eligible =
    request.forward_completion_latched && request.fresh_forward_progress;
  resolution.separated_tail_candidate =
    request.current_body_footprints_separated &&
    !request.execution_corridor_blocked;
  resolution.separated_tail_physical_safe =
    resolution.separated_tail_candidate &&
    request.footprint_prediction_valid &&
    request.predicted_body_footprint_sweep_separated;
  resolution.separated_tail_progress_allowed =
    resolution.separated_tail_candidate && request.fresh_forward_progress;
  return resolution;
}

PassShortHorizonGuardResolution resolve_pass_short_horizon_guard(
  const PassShortHorizonGuardRequest & request) noexcept
{
  PassShortHorizonGuardResolution resolution;
  if (!request.hard_guard_safe) {
    return resolution;
  }
  const bool rearward_completion_geometry_valid =
    request.forward_completion_latched &&
    request.current_body_footprints_separated &&
    !request.execution_corridor_blocked;
  const bool physical_rearward_completion_active =
    request.side_by_side_rearward_completion_safe &&
    rearward_completion_geometry_valid;
  resolution.rearward_completion_progress_active =
    request.side_by_side_rearward_progress_completion_allowed &&
    request.fresh_forward_progress && rearward_completion_geometry_valid;
  if (request.predictive_guard_safe) {
    resolution.safe = true;
    resolution.rearward_completion_active =
      physical_rearward_completion_active ||
      resolution.rearward_completion_progress_active;
    return resolution;
  }
  const bool finite_grace_window =
    std::isfinite(request.predictive_guard_loss_elapsed_sec) &&
    request.predictive_guard_loss_elapsed_sec >= 0.0 &&
    std::isfinite(request.maximum_prediction_grace_sec) &&
    request.maximum_prediction_grace_sec >= 0.0;
  resolution.prediction_grace_active =
    request.forward_completion_latched &&
    request.current_body_footprints_separated &&
    !request.execution_corridor_blocked &&
    request.fresh_forward_progress && finite_grace_window &&
    request.predictive_guard_loss_elapsed_sec <=
    request.maximum_prediction_grace_sec + 1e-9;
  resolution.rearward_completion_prediction_grace_active =
    request.side_by_side_rearward_completion_prediction_grace_allowed &&
    resolution.prediction_grace_active;
  resolution.rearward_completion_active =
    physical_rearward_completion_active ||
    resolution.rearward_completion_progress_active ||
    resolution.rearward_completion_prediction_grace_active;
  resolution.safe =
    resolution.prediction_grace_active || resolution.rearward_completion_active;
  return resolution;
}

double resolve_active_pass_elapsed(
  const double accumulated_sec, const bool pass_active,
  const double active_segment_start_sec, const double now_sec) noexcept
{
  const double completed_sec = std::isfinite(accumulated_sec) ?
    std::max(0.0, accumulated_sec) : 0.0;
  if (
    !pass_active || !std::isfinite(active_segment_start_sec) ||
    !std::isfinite(now_sec))
  {
    return completed_sec;
  }
  return completed_sec + std::max(0.0, now_sec - active_segment_start_sec);
}

SpeedPreservingTacticalRevalidationResolution
resolve_speed_preserving_tactical_revalidation(
  const SpeedPreservingTacticalRevalidationRequest & request) noexcept
{
  SpeedPreservingTacticalRevalidationResolution resolution;
  const auto finite_non_negative = [](const double value) {
      return std::isfinite(value) && value >= 0.0;
    };
  if (
    !std::isfinite(request.target_longitudinal_m) ||
    !finite_non_negative(request.maximum_absolute_longitudinal_m) ||
    !finite_non_negative(request.elapsed_sec) ||
    !finite_non_negative(request.traveled_m) ||
    !finite_non_negative(request.maximum_duration_sec) ||
    !finite_non_negative(request.maximum_distance_m))
  {
    return resolution;
  }

  resolution.remaining_sec = std::max(
    0.0, request.maximum_duration_sec - request.elapsed_sec);
  resolution.remaining_distance_m = std::max(
    0.0, request.maximum_distance_m - request.traveled_m);
  const bool within_longitudinal_window =
    std::abs(request.target_longitudinal_m) <=
    request.maximum_absolute_longitudinal_m + 1e-9;
  const bool inside_budget =
    request.elapsed_sec < request.maximum_duration_sec - 1e-9 &&
    request.traveled_m < request.maximum_distance_m - 1e-9;
  resolution.active =
    request.enabled && request.safe_separation_active && request.pass_committed &&
    request.target_continuous && request.current_body_footprints_separated &&
    request.footprint_prediction_valid && !request.execution_corridor_blocked &&
    !request.hard_fault && !request.rear_clear_confirmed &&
    within_longitudinal_window && inside_budget;
  return resolution;
}

RobustOvertakeClearanceResolution resolve_robust_overtake_clearance(
  const RobustOvertakeClearanceRequest & request) noexcept
{
  RobustOvertakeClearanceResolution resolution;
  const auto finite_non_negative = [](const double value) {
      return std::isfinite(value) && value >= 0.0;
    };
  if (
    !finite_non_negative(request.ego_speed_mps) ||
    !finite_non_negative(request.absolute_curvature_radpm) ||
    !finite_non_negative(request.physical_target_center_separation_m) ||
    !finite_non_negative(request.configured_target_center_separation_m) ||
    !finite_non_negative(request.target_surface_base_m) ||
    !finite_non_negative(request.target_speed_gain_sec) ||
    !finite_non_negative(request.target_curvature_gain_m2) ||
    !finite_non_negative(request.target_surface_max_m) ||
    !finite_non_negative(request.hard_wall_clearance_m) ||
    !finite_non_negative(request.wall_base_reserve_m) ||
    !finite_non_negative(request.wall_speed_gain_sec) ||
    !finite_non_negative(request.wall_curvature_gain_m2) ||
    !finite_non_negative(request.wall_reserve_max_m))
  {
    return resolution;
  }

  const double configured_target_separation = std::max(
    request.physical_target_center_separation_m,
    request.configured_target_center_separation_m);
  if (!request.enabled) {
    resolution.valid = true;
    resolution.target_center_separation_m = configured_target_separation;
    resolution.target_surface_clearance_m = std::max(
      0.0,
      configured_target_separation - request.physical_target_center_separation_m);
    resolution.wall_planning_clearance_m = request.hard_wall_clearance_m;
    return resolution;
  }

  const double target_surface_request =
    request.target_surface_base_m +
    request.target_speed_gain_sec * request.ego_speed_mps +
    request.target_curvature_gain_m2 * request.absolute_curvature_radpm;
  resolution.target_surface_clearance_m = std::clamp(
    target_surface_request, 0.0, request.target_surface_max_m);
  resolution.target_center_separation_m = std::max(
    configured_target_separation,
    request.physical_target_center_separation_m +
    resolution.target_surface_clearance_m);

  const double wall_reserve_request =
    request.wall_base_reserve_m +
    request.wall_speed_gain_sec * request.ego_speed_mps +
    request.wall_curvature_gain_m2 * request.absolute_curvature_radpm;
  resolution.wall_tracking_reserve_m = std::clamp(
    wall_reserve_request, 0.0, request.wall_reserve_max_m);
  resolution.wall_planning_clearance_m =
    request.hard_wall_clearance_m + resolution.wall_tracking_reserve_m;
  resolution.valid = true;
  return resolution;
}

bool can_retain_safe_trajectory_prefix(
  const SafeTrajectoryPrefixLeaseRequest & request) noexcept
{
  const auto finite_non_negative = [](const double value) {
      return std::isfinite(value) && value >= 0.0;
    };
  const auto non_negative_bound = [](const double value) {
      return !std::isnan(value) && value >= 0.0;
    };
  if (
    !std::isfinite(request.target_longitudinal_m) ||
    !finite_non_negative(request.maximum_front_distance_m) ||
    !finite_non_negative(request.validated_prefix_remaining_m) ||
    !finite_non_negative(request.minimum_validated_prefix_m) ||
    !finite_non_negative(request.absolute_elapsed_sec) ||
    !finite_non_negative(request.absolute_traveled_m) ||
    !non_negative_bound(request.absolute_maximum_duration_sec) ||
    !non_negative_bound(request.absolute_maximum_distance_m))
  {
    return false;
  }

  return
    request.enabled && request.safe_separation_active && request.pass_committed &&
    request.mission_path_frozen && request.target_continuous &&
    request.current_body_footprints_separated && request.footprint_prediction_valid &&
    request.predicted_body_footprint_sweep_separated &&
    !request.execution_corridor_blocked && !request.hard_fault &&
    !request.rear_clear_confirmed && request.fresh_forward_progress &&
    request.target_longitudinal_m <= request.maximum_front_distance_m + 1e-9 &&
    request.validated_prefix_remaining_m + 1e-9 >=
    request.minimum_validated_prefix_m &&
    request.absolute_elapsed_sec < request.absolute_maximum_duration_sec - 1e-9 &&
    request.absolute_traveled_m < request.absolute_maximum_distance_m - 1e-9;
}

bool can_continue_latched_forward_escape(
  const LatchedForwardEscapeContinuationRequest & request) noexcept
{
  return
    request.enabled && request.safe_separation_active && request.pass_committed &&
    request.forward_completion_latched && request.target_continuous &&
    request.current_body_footprints_separated &&
    request.footprint_prediction_valid && !request.execution_corridor_blocked &&
    !request.hard_fault && !request.rear_clear_confirmed;
}

bool can_return_from_tactical_revalidation(
  const TacticalRevalidationReturnRequest & request) noexcept
{
  if (
    !std::isfinite(request.target_longitudinal_m) ||
    !std::isfinite(request.minimum_front_distance_m) ||
    request.minimum_front_distance_m < 0.0)
  {
    return false;
  }
  return
    request.enabled && request.target_continuous &&
    request.current_body_footprints_separated &&
    request.footprint_prediction_valid &&
    request.predicted_body_footprint_sweep_separated &&
    request.return_corridor_available && !request.execution_corridor_blocked &&
    !request.hard_fault &&
    request.target_longitudinal_m + 1e-9 >= request.minimum_front_distance_m;
}

SafeSeparationResolution resolve_safe_separation(
  const SafeSeparationRequest & request) noexcept
{
  SafeSeparationResolution resolution;
  if (!request.enabled || !request.active) {
    return resolution;
  }
  const auto finite_non_negative = [](const double value) {
      return std::isfinite(value) && value >= 0.0;
    };
  const auto non_negative_bound = [](const double value) {
      return !std::isnan(value) && value >= 0.0;
    };
  if (
    !request.target_seen || !std::isfinite(request.target_longitudinal_m) ||
    !finite_non_negative(request.target_speed_mps) ||
    !finite_non_negative(request.speed_delta_mps) ||
    !finite_non_negative(request.maximum_ego_speed_mps) ||
    !finite_non_negative(request.front_clear_distance_m) ||
    !finite_non_negative(request.front_clear_elapsed_sec) ||
    !finite_non_negative(request.front_clear_confirm_sec) ||
    !finite_non_negative(request.elapsed_sec) ||
    !finite_non_negative(request.traveled_m) ||
    !finite_non_negative(request.maximum_duration_sec) ||
    !finite_non_negative(request.maximum_distance_m) ||
    !finite_non_negative(request.ego_speed_mps) ||
    !finite_non_negative(request.forward_escape_max_front_distance_m) ||
    !finite_non_negative(request.absolute_elapsed_sec) ||
    !finite_non_negative(request.absolute_traveled_m) ||
    !non_negative_bound(request.absolute_maximum_duration_sec) ||
    !non_negative_bound(request.absolute_maximum_distance_m) ||
    ((request.rearward_progress_loss_disengage_enabled ||
    request.rearward_progress_loss_disengage_active) &&
    (!finite_non_negative(request.rearward_progress_loss_progress_age_sec) ||
    !finite_non_negative(request.rearward_progress_loss_stale_sec) ||
    !finite_non_negative(request.rearward_progress_loss_regression_m) ||
    !finite_non_negative(request.rearward_progress_loss_minimum_regression_m) ||
    !finite_non_negative(request.rearward_progress_loss_disengage_elapsed_sec) ||
    !finite_non_negative(request.rearward_progress_loss_disengage_max_sec) ||
    !finite_non_negative(request.rearward_progress_loss_disengage_speed_delta_mps))))
  {
    resolution.action = SafeSeparationAction::Abort;
    resolution.reason = SafeSeparationReason::InvalidInput;
    return resolution;
  }
  if (request.rear_clear_confirmed && request.return_corridor_available) {
    resolution.action = SafeSeparationAction::Return;
    resolution.reason = SafeSeparationReason::RearClear;
    return resolution;
  }
  const bool rearward_progress_loss_disengage_candidate =
    request.rearward_progress_loss_disengage_enabled &&
    request.commit_stage == PassCommitStage::SideBySideCommitted &&
    request.forward_completion_latched &&
    request.target_longitudinal_m < 0.0 &&
    request.rearward_progress_loss_progress_age_sec + 1e-9 >=
    request.rearward_progress_loss_stale_sec &&
    request.rearward_progress_loss_regression_m + 1e-9 >=
    request.rearward_progress_loss_minimum_regression_m;
  const bool rearward_progress_loss_disengage_active =
    request.rearward_progress_loss_disengage_active ||
    rearward_progress_loss_disengage_candidate;
  if (rearward_progress_loss_disengage_active) {
    if (!request.rearward_progress_loss_physical_hold_safe) {
      resolution.action = SafeSeparationAction::Abort;
      resolution.reason = SafeSeparationReason::ShortHorizonUnsafe;
      return resolution;
    }
    if (
      request.target_longitudinal_m >= request.front_clear_distance_m - 1e-9 &&
      request.front_clear_elapsed_sec >= request.front_clear_confirm_sec - 1e-9)
    {
      resolution.action = SafeSeparationAction::RecoverBehind;
      resolution.reason = SafeSeparationReason::TargetClearAhead;
      return resolution;
    }
    if (
      request.rearward_progress_loss_disengage_elapsed_sec >=
      request.rearward_progress_loss_disengage_max_sec - 1e-9)
    {
      resolution.action = SafeSeparationAction::Abort;
      resolution.reason = SafeSeparationReason::RearwardProgressLossDisengagementTimeout;
      return resolution;
    }
    resolution.action = SafeSeparationAction::KeepSameSide;
    resolution.reason = SafeSeparationReason::RearwardProgressLossDisengagement;
    resolution.target_velocity_reference_mps = std::min(
      request.maximum_ego_speed_mps,
      std::max(
        0.0,
        request.target_speed_mps -
        request.rearward_progress_loss_disengage_speed_delta_mps));
    resolution.signed_closing_speed_mps =
      resolution.target_velocity_reference_mps - request.target_speed_mps;
    return resolution;
  }
  if (!request.short_horizon_safe) {
    resolution.action = SafeSeparationAction::Abort;
    resolution.reason = SafeSeparationReason::ShortHorizonUnsafe;
    return resolution;
  }
  const bool forward_escape_active =
    request.forward_escape_allowed &&
    (request.forward_completion_latched ||
    request.target_longitudinal_m <=
    request.forward_escape_max_front_distance_m + 1e-9);
  const auto apply_forward_escape_velocity = [&]() {
      resolution.forward_escape_active = true;
      resolution.full_speed_forward_escape_active =
        request.full_speed_forward_escape_enabled;
      resolution.target_velocity_reference_mps =
        request.full_speed_forward_escape_enabled ?
        request.maximum_ego_speed_mps :
        std::min(
        request.maximum_ego_speed_mps,
        std::max(
          request.ego_speed_mps,
          request.target_speed_mps + request.speed_delta_mps));
      resolution.signed_closing_speed_mps =
      resolution.target_velocity_reference_mps - request.target_speed_mps;
    };
  const bool side_by_side_rear_clear_completion_active =
    request.side_by_side_rearward_completion_allowed &&
    request.commit_stage == PassCommitStage::SideBySideCommitted &&
    request.forward_completion_latched && forward_escape_active &&
    request.target_longitudinal_m <= 0.0;
  const bool absolute_distance_limit_reached =
    request.absolute_traveled_m >= request.absolute_maximum_distance_m - 1e-9;
  const bool absolute_time_limit_reached =
    request.absolute_elapsed_sec >= request.absolute_maximum_duration_sec - 1e-9;
  // A committed side-by-side escape can begin just before the overall Pass
  // bound. Let that already-active local window finish instead of dropping to
  // Recovery beside the target. The local window cannot be re-armed once an
  // absolute bound has been crossed.
  if (
    absolute_distance_limit_reached &&
    (!forward_escape_active || side_by_side_rear_clear_completion_active))
  {
    resolution.action = SafeSeparationAction::Abort;
    resolution.reason = SafeSeparationReason::AbsoluteDistanceLimit;
    return resolution;
  }
  if (
    absolute_time_limit_reached &&
    (!forward_escape_active || side_by_side_rear_clear_completion_active))
  {
    resolution.action = SafeSeparationAction::Abort;
    resolution.reason = SafeSeparationReason::AbsoluteTimeLimit;
    return resolution;
  }

  const bool local_distance_limit_reached =
    request.traveled_m >= request.maximum_distance_m - 1e-9;
  const bool local_time_limit_reached =
    request.elapsed_sec >= request.maximum_duration_sec - 1e-9;
  const bool rearward_progress_time_grace_active =
    request.rearward_progress_time_grace_enabled && forward_escape_active &&
    request.target_longitudinal_m < 0.0 &&
    request.fresh_forward_progress &&
    !absolute_distance_limit_reached;
  if (
    local_distance_limit_reached || local_time_limit_reached)
  {
    if (
      side_by_side_rear_clear_completion_active &&
      !absolute_distance_limit_reached && !absolute_time_limit_reached)
    {
      resolution.action = SafeSeparationAction::KeepSameSide;
      resolution.reason = SafeSeparationReason::SideBySideRearClearCompletion;
      apply_forward_escape_velocity();
      return resolution;
    }
    if (
      local_time_limit_reached && !local_distance_limit_reached &&
      rearward_progress_time_grace_active)
    {
      resolution.action = SafeSeparationAction::KeepSameSide;
      resolution.reason = SafeSeparationReason::RearwardProgressTimeGrace;
      apply_forward_escape_velocity();
      return resolution;
    }
    const bool absolute_limit_reached =
      absolute_distance_limit_reached || absolute_time_limit_reached;
    if (
      forward_escape_active &&
      (request.forward_progress_extension_allowed ||
      request.dynamic_completion_extension_allowed) &&
      !absolute_limit_reached)
    {
      resolution.action = SafeSeparationAction::KeepSameSide;
      resolution.progress_extension_requested = true;
      resolution.reason = request.forward_progress_extension_allowed ?
        SafeSeparationReason::ProgressExtension :
        SafeSeparationReason::DynamicCompletionExtension;
      apply_forward_escape_velocity();
      return resolution;
    }
    resolution.action = SafeSeparationAction::Abort;
    resolution.reason = local_distance_limit_reached ?
      SafeSeparationReason::LocalDistanceLimit : SafeSeparationReason::LocalTimeLimit;
    return resolution;
  }

  resolution.action = SafeSeparationAction::KeepSameSide;
  if (forward_escape_active) {
    apply_forward_escape_velocity();
    return resolution;
  }
  if (
    request.target_longitudinal_m >= request.front_clear_distance_m - 1e-9 &&
    request.front_clear_elapsed_sec >= request.front_clear_confirm_sec - 1e-9)
  {
    if (request.commit_stage != PassCommitStage::SideBySideCommitted) {
      resolution.action = SafeSeparationAction::RecoverBehind;
      resolution.reason = SafeSeparationReason::TargetClearAhead;
      return resolution;
    }
  }
  if (request.target_longitudinal_m >= 0.0) {
    if (request.commit_stage == PassCommitStage::SideBySideCommitted) {
      resolution.target_velocity_reference_mps = std::min(
        request.maximum_ego_speed_mps,
        std::max(request.ego_speed_mps, request.target_speed_mps));
      resolution.signed_closing_speed_mps =
        resolution.target_velocity_reference_mps - request.target_speed_mps;
    } else {
      resolution.target_velocity_reference_mps = std::min(
        request.maximum_ego_speed_mps,
        std::max(0.0, request.target_speed_mps - request.speed_delta_mps));
      resolution.signed_closing_speed_mps = -std::min(
        request.speed_delta_mps, request.target_speed_mps);
    }
  } else {
    resolution.target_velocity_reference_mps = std::min(
      request.maximum_ego_speed_mps,
      request.target_speed_mps + request.speed_delta_mps);
    resolution.signed_closing_speed_mps =
      resolution.target_velocity_reference_mps - request.target_speed_mps;
  }
  return resolution;
}

bool can_reselect_from_safe_separation(
  const SafeSeparationTacticalReselectRequest & request) noexcept
{
  if (
    !std::isfinite(request.target_longitudinal_m) ||
    !std::isfinite(request.minimum_front_distance_m) ||
    request.minimum_front_distance_m < 0.0)
  {
    return false;
  }
  return
    request.enabled && request.safe_separation_active &&
    !request.forward_escape_allowed && request.target_continuous &&
    request.current_body_footprints_separated &&
    request.footprint_prediction_valid &&
    request.predicted_body_footprint_sweep_separated &&
    !request.execution_corridor_blocked && !request.hard_fault &&
    !request.rear_clear_confirmed &&
    request.target_longitudinal_m + 1e-9 >= request.minimum_front_distance_m;
}

SoftMissionAbortAction resolve_soft_mission_abort(
  const SoftMissionAbortRequest & request) noexcept
{
  if (
    !std::isfinite(request.target_longitudinal_m) ||
    !std::isfinite(request.minimum_front_distance_m) ||
    request.minimum_front_distance_m < 0.0)
  {
    return SoftMissionAbortAction::Recovery;
  }
  const bool physically_clear_follow =
    request.soft_failure && !request.hard_fault && request.target_continuous &&
    request.current_body_footprints_separated &&
    request.footprint_prediction_valid &&
    request.predicted_body_footprint_sweep_separated &&
    !request.execution_corridor_blocked && !request.rear_clear_confirmed &&
    request.target_longitudinal_m + 1e-9 >= request.minimum_front_distance_m;
  return physically_clear_follow ?
    SoftMissionAbortAction::SpeedPreservingFollow :
    SoftMissionAbortAction::Recovery;
}

MissionAlignedSafeSeparationBudgetResolution resolve_mission_aligned_safe_separation_budget(
  const MissionAlignedSafeSeparationBudgetRequest & request) noexcept
{
  MissionAlignedSafeSeparationBudgetResolution resolution;
  const auto finite_non_negative = [](const double value) {
      return std::isfinite(value) && value >= 0.0;
    };
  const auto non_negative_bound = [](const double value) {
      return !std::isnan(value) && value >= 0.0;
    };
  if (
    !finite_non_negative(request.configured_maximum_duration_sec) ||
    !finite_non_negative(request.configured_maximum_distance_m) ||
    !finite_non_negative(request.current_pass_traveled_m) ||
    !finite_non_negative(request.completion_distance_margin_m) ||
    !finite_non_negative(request.completion_time_margin_sec) ||
    !finite_non_negative(request.forward_speed_mps) ||
    !finite_non_negative(request.absolute_elapsed_sec) ||
    !finite_non_negative(request.absolute_traveled_m) ||
    !non_negative_bound(request.absolute_maximum_duration_sec) ||
    !non_negative_bound(request.absolute_maximum_distance_m))
  {
    return resolution;
  }

  resolution.valid = true;
  const double remaining_absolute_distance =
    std::isfinite(request.absolute_maximum_distance_m) ?
    std::max(
    0.0, request.absolute_maximum_distance_m - request.absolute_traveled_m) :
    std::numeric_limits<double>::infinity();
  const double remaining_absolute_time =
    std::isfinite(request.absolute_maximum_duration_sec) ?
    std::max(0.0, request.absolute_maximum_duration_sec - request.absolute_elapsed_sec) :
    std::numeric_limits<double>::infinity();
  double requested_distance = request.configured_maximum_distance_m;
  if (
    request.enabled && std::isfinite(request.predicted_rear_clear_pass_m) &&
    request.predicted_rear_clear_pass_m >= request.current_pass_traveled_m)
  {
    requested_distance = std::max(
      requested_distance,
      request.predicted_rear_clear_pass_m - request.current_pass_traveled_m +
      request.completion_distance_margin_m);
    resolution.mission_aligned = true;
  }
  resolution.maximum_distance_m = std::min(requested_distance, remaining_absolute_distance);
  const double speed_for_time = std::max(1.0, request.forward_speed_mps);
  const double requested_time = resolution.mission_aligned ?
    std::max(
    request.configured_maximum_duration_sec,
    resolution.maximum_distance_m / speed_for_time +
    request.completion_time_margin_sec) :
    request.configured_maximum_duration_sec;
  resolution.maximum_duration_sec = std::min(requested_time, remaining_absolute_time);
  return resolution;
}

RecoverableSideContactResolution resolve_recoverable_side_contact(
  const RecoverableSideContactRequest & request) noexcept
{
  RecoverableSideContactResolution resolution;
  const auto finite_non_negative = [](const double value) {
      return std::isfinite(value) && value >= 0.0;
    };
  if (
    !request.enabled || !request.pass_active || !request.target_seen ||
    !request.target_continuity_valid ||
    (!request.current_body_overlap_confirmed && !request.impact_event_confirmed &&
    !(request.previously_active && request.evidence_dropout_held)) ||
    !request.relative_heading_valid || !request.wall_clearance_sufficient ||
    (request.pass_side_sign != -1 && request.pass_side_sign != 1) ||
    !std::isfinite(request.relative_heading_rad) ||
    !std::isfinite(request.target_longitudinal_m) ||
    !std::isfinite(request.relative_lateral_m) ||
    !finite_non_negative(request.longitudinal_closing_speed_mps) ||
    !finite_non_negative(request.ego_speed_mps) ||
    !finite_non_negative(request.contact_elapsed_sec) ||
    !finite_non_negative(request.maximum_duration_sec) ||
    !finite_non_negative(request.rearward_completion_maximum_duration_sec) ||
    !finite_non_negative(request.initial_progress_grace_sec) ||
    !finite_non_negative(request.maximum_absolute_longitudinal_m) ||
    !finite_non_negative(request.minimum_absolute_lateral_m) ||
    !finite_non_negative(request.maximum_longitudinal_closing_speed_mps) ||
    !finite_non_negative(request.maximum_absolute_lateral_velocity_mps) ||
    !finite_non_negative(request.maximum_release_absolute_lateral_velocity_mps) ||
    !finite_non_negative(request.minimum_ego_speed_mps) ||
    !finite_non_negative(request.maximum_absolute_relative_heading_rad) ||
    !finite_non_negative(request.body_longitudinal_clearance_m) ||
    !finite_non_negative(request.body_lateral_clearance_m) ||
    !finite_non_negative(request.lateral_separation_bias_m))
  {
    return resolution;
  }
  if (
    std::abs(request.relative_heading_rad) >
    request.maximum_absolute_relative_heading_rad + 1e-9)
  {
    return resolution;
  }
  if (request.relative_lateral_velocity_valid) {
    if (!std::isfinite(request.relative_lateral_velocity_mps)) {
      return resolution;
    }
    const double entry_lateral_velocity_limit =
      request.maximum_absolute_lateral_velocity_mps;
    const double release_lateral_velocity_limit = std::max(
      entry_lateral_velocity_limit,
      request.maximum_release_absolute_lateral_velocity_mps);
    const double active_lateral_velocity_limit = request.previously_active ?
      release_lateral_velocity_limit : entry_lateral_velocity_limit;
    const double absolute_lateral_velocity =
      std::abs(request.relative_lateral_velocity_mps);
    if (absolute_lateral_velocity > active_lateral_velocity_limit + 1e-9) {
      return resolution;
    }
    resolution.lateral_velocity_hysteresis_active =
      request.previously_active &&
      absolute_lateral_velocity > entry_lateral_velocity_limit + 1e-9;
  }
  const bool target_on_opposite_side =
    static_cast<double>(request.pass_side_sign) * request.relative_lateral_m < 0.0;
  const double longitudinal_penetration_m =
    request.body_longitudinal_clearance_m - std::abs(request.target_longitudinal_m);
  const double lateral_penetration_m =
    request.body_lateral_clearance_m - std::abs(request.relative_lateral_m);
  resolution.side_contact_normal =
    longitudinal_penetration_m >= -1e-9 && lateral_penetration_m >= -1e-9 &&
    lateral_penetration_m <= longitudinal_penetration_m + 1e-9;
  const bool side_contact_geometry =
    target_on_opposite_side && resolution.side_contact_normal &&
    std::abs(request.relative_lateral_m) >= request.minimum_absolute_lateral_m - 1e-9 &&
    std::abs(request.target_longitudinal_m) <=
    request.maximum_absolute_longitudinal_m + 1e-9 &&
    request.longitudinal_closing_speed_mps <=
    request.maximum_longitudinal_closing_speed_mps + 1e-9;
  resolution.initial_progress_grace_active =
    request.contact_elapsed_sec <= request.initial_progress_grace_sec + 1e-9;
  const auto rearward_completion_context = resolve_rearward_pass_completion_context(
    RearwardPassCompletionContextRequest{
      request.pass_active,
      request.commit_stage,
      request.target_seen,
      true,
      request.target_continuity_valid,
      request.target_longitudinal_m,
      request.forward_completion_latched,
      request.fresh_forward_progress});
  resolution.rearward_completion_active =
    side_contact_geometry &&
    request.ego_speed_mps >= request.minimum_ego_speed_mps - 1e-9 &&
    rearward_completion_context.contact_tail_eligible &&
    request.contact_elapsed_sec <=
    request.rearward_completion_maximum_duration_sec + 1e-9;
  resolution.impact_event_used =
    request.impact_event_confirmed && !request.current_body_overlap_confirmed;
  resolution.evidence_dropout_active =
    request.previously_active && request.evidence_dropout_held &&
    !request.current_body_overlap_confirmed && !request.impact_event_confirmed;
  resolution.active =
    (side_contact_geometry &&
    request.ego_speed_mps >= request.minimum_ego_speed_mps - 1e-9 &&
    request.contact_elapsed_sec <= request.maximum_duration_sec + 1e-9 &&
    (resolution.initial_progress_grace_active || request.fresh_forward_progress)) ||
    resolution.rearward_completion_active;
  if (resolution.active) {
    resolution.lateral_separation_bias_m =
      static_cast<double>(request.pass_side_sign) * request.lateral_separation_bias_m;
  }
  return resolution;
}

WallBoundedContactSeparationResolution resolve_wall_bounded_contact_separation(
  const WallBoundedContactSeparationRequest & request) noexcept
{
  WallBoundedContactSeparationResolution resolution;
  if (!std::isfinite(request.base_goal_m)) {
    return resolution;
  }
  resolution.goal_m = request.base_goal_m;
  if (!request.active) {
    resolution.valid = true;
    return resolution;
  }
  if (
    (request.pass_side_sign != -1 && request.pass_side_sign != 1) ||
    !std::isfinite(request.requested_bias_m) || request.requested_bias_m < 0.0)
  {
    return resolution;
  }
  resolution.requested_signed_bias_m =
    static_cast<double>(request.pass_side_sign) * request.requested_bias_m;
  if (
    !request.feasible_interval_available ||
    !std::isfinite(request.feasible_lower_m) ||
    !std::isfinite(request.feasible_upper_m) ||
    request.feasible_upper_m < request.feasible_lower_m)
  {
    resolution.valid = true;
    resolution.wall_limited = request.requested_bias_m > 1e-9;
    return resolution;
  }

  const double requested_goal =
    request.base_goal_m + resolution.requested_signed_bias_m;
  double bounded_goal = request.base_goal_m;
  if (request.pass_side_sign > 0) {
    // Never use contact separation to pull an already out-of-bounds goal
    // farther toward the wall.
    bounded_goal = std::max(
      request.base_goal_m,
      std::min(requested_goal, request.feasible_upper_m));
  } else {
    bounded_goal = std::min(
      request.base_goal_m,
      std::max(requested_goal, request.feasible_lower_m));
  }
  resolution.valid = true;
  resolution.goal_m = bounded_goal;
  resolution.applied_signed_bias_m = bounded_goal - request.base_goal_m;
  resolution.wall_limited =
    std::abs(
    resolution.applied_signed_bias_m - resolution.requested_signed_bias_m) > 1e-9;
  return resolution;
}

const char * to_string(const SafeSeparationAction action) noexcept
{
  switch (action) {
    case SafeSeparationAction::Inactive:
      return "inactive";
    case SafeSeparationAction::KeepSameSide:
      return "keep same side";
    case SafeSeparationAction::Return:
      return "return";
    case SafeSeparationAction::RecoverBehind:
      return "recover behind";
    case SafeSeparationAction::Abort:
      return "abort";
  }
  return "unknown";
}

const char * to_string(const SafeSeparationReason reason) noexcept
{
  switch (reason) {
    case SafeSeparationReason::None:
      return "none";
    case SafeSeparationReason::InvalidInput:
      return "invalid input";
    case SafeSeparationReason::RearClear:
      return "rear clear";
    case SafeSeparationReason::ShortHorizonUnsafe:
      return "short horizon unsafe";
    case SafeSeparationReason::LocalTimeLimit:
      return "local time limit";
    case SafeSeparationReason::LocalDistanceLimit:
      return "local distance limit";
    case SafeSeparationReason::AbsoluteTimeLimit:
      return "absolute Pass time limit";
    case SafeSeparationReason::AbsoluteDistanceLimit:
      return "absolute Pass distance limit";
    case SafeSeparationReason::TargetClearAhead:
      return "target clear ahead";
    case SafeSeparationReason::ProgressExtension:
      return "fresh forward progress extension";
    case SafeSeparationReason::DynamicCompletionExtension:
      return "dynamic completion extension";
    case SafeSeparationReason::RearwardProgressTimeGrace:
      return "rearward progress time grace";
    case SafeSeparationReason::SideBySideRearClearCompletion:
      return "side-by-side rear-clear completion";
    case SafeSeparationReason::RearwardProgressLossDisengagement:
      return "rearward progress-loss disengagement";
    case SafeSeparationReason::RearwardProgressLossDisengagementTimeout:
      return "rearward progress-loss disengagement timeout";
  }
  return "unknown";
}

SameSideReplanShiftDistanceResolution resolve_same_side_replan_shift_distance(
  const SameSideReplanShiftDistanceRequest & request) noexcept
{
  SameSideReplanShiftDistanceResolution resolution;
  if (
    !std::isfinite(request.current_lateral_m) ||
    !std::isfinite(request.goal_lateral_m) ||
    !std::isfinite(request.planning_speed_mps) || request.planning_speed_mps < 0.0 ||
    !std::isfinite(request.maximum_lateral_accel_mps2) ||
    request.maximum_lateral_accel_mps2 <= 0.0 ||
    !std::isfinite(request.minimum_shift_distance_m) ||
    request.minimum_shift_distance_m < 0.0 ||
    !std::isfinite(request.maximum_shift_distance_m) ||
    request.maximum_shift_distance_m < request.minimum_shift_distance_m ||
    !std::isfinite(request.distance_margin_m) || request.distance_margin_m < 0.0)
  {
    return resolution;
  }

  resolution.valid = true;
  resolution.lateral_adjustment_m = std::abs(
    request.goal_lateral_m - request.current_lateral_m);
  resolution.required_time_sec = std::sqrt(
    2.0 * resolution.lateral_adjustment_m /
    request.maximum_lateral_accel_mps2);
  resolution.required_distance_m =
    request.planning_speed_mps * resolution.required_time_sec;
  resolution.shift_distance_m = std::clamp(
    resolution.required_distance_m + request.distance_margin_m,
    request.minimum_shift_distance_m, request.maximum_shift_distance_m);
  resolution.feasible =
    resolution.required_distance_m <= request.maximum_shift_distance_m + 1e-9;
  return resolution;
}

OvertakeMissionDynamicCorridorResolution resolve_overtake_mission_dynamic_corridor(
  const OvertakeMissionDynamicCorridorRequest & request) noexcept
{
  OvertakeMissionDynamicCorridorResolution resolution;
  if (
    std::isnan(request.candidate_goal_lower_m) ||
    std::isnan(request.candidate_goal_upper_m) ||
    request.candidate_goal_upper_m < request.candidate_goal_lower_m ||
    std::isnan(request.maximum_validation_distance_m) ||
    request.maximum_validation_distance_m < 0.0)
  {
    return resolution;
  }

  auto path_template = request.mission_path;
  path_template.path_distance_m = 0.0;
  const auto mission_origin = resolve_overtake_mission_path(path_template);
  if (!mission_origin.valid) {
    return resolution;
  }

  resolution.valid = true;
  resolution.feasible = true;
  resolution.goal_lower_m = request.candidate_goal_lower_m;
  resolution.goal_upper_m = request.candidate_goal_upper_m;
  constexpr double kCoefficientEpsilon = 1e-9;
  constexpr double kCorridorEpsilon = 1e-9;
  const double maximum_validation_distance_m = std::min(
    mission_origin.total_distance_m, request.maximum_validation_distance_m);

  for (std::size_t i = 0; i < request.samples.size(); ++i) {
    const auto & sample = request.samples[i];
    if (!sample.active) {
      continue;
    }
    if (
      !std::isfinite(sample.path_distance_m) || sample.path_distance_m < 0.0 ||
      !std::isfinite(sample.lower_lateral_m) ||
      !std::isfinite(sample.upper_lateral_m) ||
      sample.upper_lateral_m < sample.lower_lateral_m)
    {
      resolution.valid = false;
      resolution.feasible = false;
      return resolution;
    }
    if (sample.path_distance_m > maximum_validation_distance_m + kCorridorEpsilon) {
      continue;
    }

    auto zero_goal_path = request.mission_path;
    zero_goal_path.path_distance_m = sample.path_distance_m;
    zero_goal_path.pass_lateral_m = 0.0;
    const auto zero_goal = resolve_overtake_mission_path(zero_goal_path);
    auto unit_goal_path = zero_goal_path;
    unit_goal_path.pass_lateral_m = 1.0;
    const auto unit_goal = resolve_overtake_mission_path(unit_goal_path);
    if (!zero_goal.valid || !unit_goal.valid) {
      resolution.valid = false;
      resolution.feasible = false;
      return resolution;
    }

    resolution.observed = true;
    ++resolution.checked_sample_count;
    const double intercept = zero_goal.lateral_target_m;
    const double coefficient = unit_goal.lateral_target_m - intercept;
    if (std::abs(coefficient) <= kCoefficientEpsilon) {
      if (
        intercept < sample.lower_lateral_m - kCorridorEpsilon ||
        intercept > sample.upper_lateral_m + kCorridorEpsilon)
      {
        resolution.feasible = false;
        resolution.first_conflict_index = i;
        resolution.first_conflict_distance_m = sample.path_distance_m;
        resolution.first_conflict_lateral_m = intercept;
        resolution.first_conflict_lower_m = sample.lower_lateral_m;
        resolution.first_conflict_upper_m = sample.upper_lateral_m;
        return resolution;
      }
      continue;
    }

    double sample_goal_lower =
      (sample.lower_lateral_m - intercept) / coefficient;
    double sample_goal_upper =
      (sample.upper_lateral_m - intercept) / coefficient;
    if (sample_goal_upper < sample_goal_lower) {
      std::swap(sample_goal_lower, sample_goal_upper);
    }
    resolution.goal_lower_m = std::max(resolution.goal_lower_m, sample_goal_lower);
    resolution.goal_upper_m = std::min(resolution.goal_upper_m, sample_goal_upper);
    if (resolution.goal_upper_m + kCorridorEpsilon < resolution.goal_lower_m) {
      resolution.feasible = false;
      resolution.first_conflict_index = i;
      resolution.first_conflict_distance_m = sample.path_distance_m;
      const double diagnostic_goal = std::clamp(
        request.mission_path.pass_lateral_m,
        request.candidate_goal_lower_m, request.candidate_goal_upper_m);
      resolution.first_conflict_lateral_m = intercept + coefficient * diagnostic_goal;
      resolution.first_conflict_lower_m = sample.lower_lateral_m;
      resolution.first_conflict_upper_m = sample.upper_lateral_m;
      return resolution;
    }
  }
  return resolution;
}

OvertakeMissionCorridorAdmissionResolution resolve_overtake_mission_corridor_admission(
  const OvertakeMissionCorridorAdmissionRequest & request) noexcept
{
  OvertakeMissionCorridorAdmissionResolution resolution;
  const bool static_interval_valid =
    std::isfinite(request.static_goal_lower_m) &&
    std::isfinite(request.static_goal_upper_m) &&
    request.static_goal_lower_m <= request.static_goal_upper_m;
  if (!request.dynamic_corridor.valid || !static_interval_valid) {
    return resolution;
  }

  resolution.valid = true;
  if (!request.entry_gap_available) {
    return resolution;
  }

  if (request.dynamic_corridor.observed) {
    const bool dynamic_interval_valid =
      request.dynamic_corridor.feasible &&
      std::isfinite(request.dynamic_corridor.goal_lower_m) &&
      std::isfinite(request.dynamic_corridor.goal_upper_m) &&
      request.dynamic_corridor.goal_lower_m <= request.dynamic_corridor.goal_upper_m;
    if (!dynamic_interval_valid) {
      return resolution;
    }
    resolution.feasible = true;
    resolution.source = OvertakeMissionCorridorSource::DynamicObservation;
    resolution.goal_lower_m = request.dynamic_corridor.goal_lower_m;
    resolution.goal_upper_m = request.dynamic_corridor.goal_upper_m;
    return resolution;
  }

  resolution.feasible = true;
  resolution.source = OvertakeMissionCorridorSource::StaticWallFallback;
  resolution.goal_lower_m = request.static_goal_lower_m;
  resolution.goal_upper_m = request.static_goal_upper_m;
  return resolution;
}

const char * to_string(const OvertakeMissionCorridorSource source) noexcept
{
  switch (source) {
    case OvertakeMissionCorridorSource::DynamicObservation:
      return "dynamic";
    case OvertakeMissionCorridorSource::StaticWallFallback:
      return "static_fallback";
    case OvertakeMissionCorridorSource::None:
    default:
      return "none";
  }
}

StaticFallbackEntryMotionAdmissionResolution
resolve_static_fallback_entry_motion_admission(
  const StaticFallbackEntryMotionAdmissionRequest & request) noexcept
{
  constexpr double kLateralShiftEpsilon = 1e-9;
  StaticFallbackEntryMotionAdmissionResolution resolution;
  if (
    request.corridor_source == OvertakeMissionCorridorSource::None ||
    !std::isfinite(request.lateral_shift_m) || request.lateral_shift_m < 0.0 ||
    !std::isfinite(request.maximum_lateral_shift_m) ||
    request.maximum_lateral_shift_m < 0.0)
  {
    return resolution;
  }

  resolution.valid = true;
  resolution.guard_applied =
    request.guard_enabled && request.new_mission_entry &&
    request.corridor_source == OvertakeMissionCorridorSource::StaticWallFallback;
  resolution.admitted =
    !resolution.guard_applied ||
    request.lateral_shift_m <=
    request.maximum_lateral_shift_m + kLateralShiftEpsilon;
  return resolution;
}

bool is_full_track_transition_admitted(
  const bool full_track_transition_before_rear_clear,
  const bool scheduled_transition_validated) noexcept
{
  return
    !full_track_transition_before_rear_clear ||
    scheduled_transition_validated;
}

OvertakePassPlan build_overtake_pass_plan(
  const OvertakePassPlanRequest & request) noexcept
{
  OvertakePassPlan plan;
  const auto & mission = request.candidate;
  if (
    !mission.feasible || (mission.pass_side_sign != -1 && mission.pass_side_sign != 1) ||
    !std::isfinite(request.start_lateral_m) ||
    !std::isfinite(request.return_lateral_m) ||
    !std::isfinite(mission.goal_lateral_m) ||
    !std::isfinite(mission.shift_distance_m) || mission.shift_distance_m < 0.5 ||
    !std::isfinite(mission.pass_hold_distance_m) || mission.pass_hold_distance_m < 0.5 ||
    !std::isfinite(mission.return_distance_m) || mission.return_distance_m < 0.5 ||
    !std::isfinite(mission.closing_speed_mps) || mission.closing_speed_mps < 0.0 ||
    (mission.body_clear_deadline_checked && !mission.body_clear_deadline_feasible) ||
    (mission.rear_clear_prediction_checked && !mission.rear_clear_prediction_feasible) ||
    (mission.outer_transition_required &&
    !mission.outer_transition_preflight_validated))
  {
    return plan;
  }

  plan.path = OvertakeMissionPathRequest{
    0.0,
    request.start_lateral_m,
    mission.goal_lateral_m,
    request.return_lateral_m,
    mission.shift_distance_m,
    mission.pass_hold_distance_m,
    mission.return_distance_m};
  const auto origin = resolve_overtake_mission_path(plan.path);
  if (!origin.valid) {
    return plan;
  }

  plan.valid = true;
  plan.mission = mission;
  return plan;
}

std::vector<double> build_overtake_closing_speed_candidates(
  const double minimum_closing_speed_mps,
  const double maximum_closing_speed_mps) noexcept
{
  std::vector<double> candidates;
  if (
    !std::isfinite(minimum_closing_speed_mps) || minimum_closing_speed_mps < 0.0 ||
    !std::isfinite(maximum_closing_speed_mps) ||
    maximum_closing_speed_mps < minimum_closing_speed_mps)
  {
    return candidates;
  }

  constexpr double kEpsilon = 1e-9;
  const auto add_candidate = [&](const double candidate) {
      const bool duplicate = std::any_of(
        candidates.begin(), candidates.end(),
        [&](const double existing) {return std::abs(existing - candidate) <= kEpsilon;});
      if (!duplicate) {
        candidates.push_back(candidate);
      }
    };
  add_candidate(minimum_closing_speed_mps);
  add_candidate(0.5 * (minimum_closing_speed_mps + maximum_closing_speed_mps));
  add_candidate(maximum_closing_speed_mps);
  return candidates;
}

OvertakeMissionHorizonProgressEvaluation evaluate_overtake_mission_horizon_progress(
  const OvertakeMissionHorizonProgressRequest & request) noexcept
{
  OvertakeMissionHorizonProgressEvaluation evaluation;
  if (!request.enabled) {
    evaluation.valid = true;
    evaluation.hard_feasible = true;
    evaluation.reject_reason = OvertakeMissionHorizonProgressRejectReason::Disabled;
    evaluation.score = 0.0;
    return evaluation;
  }

  const auto finite_non_negative = [](const double value) {
      return std::isfinite(value) && value >= 0.0;
    };
  const auto weight_is_valid = [&](const double value) {
      return finite_non_negative(value);
    };
  if (
    !finite_non_negative(request.rear_clear_time_budget_sec) ||
    request.rear_clear_time_budget_sec <= 0.0 ||
    !finite_non_negative(request.rear_clear_distance_budget_m) ||
    request.rear_clear_distance_budget_m <= 0.0 ||
    !finite_non_negative(request.reference_speed_mps) ||
    request.reference_speed_mps <= 0.0 ||
    !finite_non_negative(request.maximum_closing_speed_mps) ||
    request.maximum_closing_speed_mps <= 0.0 ||
    !finite_non_negative(request.lateral_motion_scale_m) ||
    request.lateral_motion_scale_m <= 0.0 ||
    !finite_non_negative(request.maximum_lateral_accel_mps2) ||
    request.maximum_lateral_accel_mps2 <= 0.0 ||
    !weight_is_valid(request.weights.rear_clear_time) ||
    !weight_is_valid(request.weights.rear_clear_distance) ||
    !weight_is_valid(request.weights.retained_speed) ||
    !weight_is_valid(request.weights.closing_speed) ||
    !weight_is_valid(request.weights.lateral_motion_penalty) ||
    !weight_is_valid(request.weights.lateral_accel_penalty))
  {
    evaluation.reject_reason = OvertakeMissionHorizonProgressRejectReason::InvalidInput;
    return evaluation;
  }
  evaluation.valid = true;
  evaluation.checked = true;

  const auto & candidate = request.candidate;
  if (!candidate.rear_clear_prediction_checked) {
    evaluation.reject_reason =
      OvertakeMissionHorizonProgressRejectReason::RearClearUnchecked;
    return evaluation;
  }
  if (
    !candidate.rear_clear_prediction_feasible ||
    !finite_non_negative(candidate.predicted_rear_clear_time_sec) ||
    !finite_non_negative(candidate.predicted_rear_clear_ego_distance_m) ||
    !finite_non_negative(candidate.predicted_rear_clear_speed_mps) ||
    !finite_non_negative(candidate.predicted_minimum_ego_speed_mps) ||
    !finite_non_negative(candidate.closing_speed_mps) ||
    !finite_non_negative(candidate.lateral_shift_m) ||
    !finite_non_negative(candidate.max_required_lateral_accel_mps2))
  {
    evaluation.reject_reason =
      OvertakeMissionHorizonProgressRejectReason::RearClearInfeasible;
    return evaluation;
  }
  if (
    candidate.predicted_rear_clear_time_sec >
    request.rear_clear_time_budget_sec + 1e-9)
  {
    evaluation.reject_reason =
      OvertakeMissionHorizonProgressRejectReason::RearClearTimeBudget;
    return evaluation;
  }
  const double pass_origin_shift_distance = candidate.direct_pass ? 0.0 :
    candidate.shift_distance_m;
  const double rear_clear_pass_distance_m = std::max(
    0.0,
    candidate.predicted_rear_clear_ego_distance_m - pass_origin_shift_distance);
  if (
    rear_clear_pass_distance_m >
    request.rear_clear_distance_budget_m + 1e-9)
  {
    evaluation.reject_reason =
      OvertakeMissionHorizonProgressRejectReason::RearClearDistanceBudget;
    return evaluation;
  }

  evaluation.hard_feasible = true;
  evaluation.reject_reason = OvertakeMissionHorizonProgressRejectReason::None;
  evaluation.rear_clear_time_progress = std::clamp(
    1.0 - candidate.predicted_rear_clear_time_sec /
    request.rear_clear_time_budget_sec, 0.0, 1.0);
  evaluation.rear_clear_distance_progress = std::clamp(
    1.0 - rear_clear_pass_distance_m /
    request.rear_clear_distance_budget_m, 0.0, 1.0);
  evaluation.retained_speed = std::clamp(
    candidate.predicted_minimum_ego_speed_mps / request.reference_speed_mps,
    0.0, 1.0);
  evaluation.closing_speed_progress = std::clamp(
    candidate.closing_speed_mps / request.maximum_closing_speed_mps,
    0.0, 1.0);
  evaluation.lateral_motion_cost = std::clamp(
    candidate.lateral_shift_m / request.lateral_motion_scale_m, 0.0, 1.0);
  evaluation.lateral_accel_cost = std::clamp(
    candidate.max_required_lateral_accel_mps2 /
    request.maximum_lateral_accel_mps2, 0.0, 1.0);
  evaluation.score =
    request.weights.rear_clear_time * evaluation.rear_clear_time_progress +
    request.weights.rear_clear_distance * evaluation.rear_clear_distance_progress +
    request.weights.retained_speed * evaluation.retained_speed +
    request.weights.closing_speed * evaluation.closing_speed_progress -
    request.weights.lateral_motion_penalty * evaluation.lateral_motion_cost -
    request.weights.lateral_accel_penalty * evaluation.lateral_accel_cost;
  return evaluation;
}

OvertakeMissionCandidateSelection select_overtake_mission_candidate(
  const OvertakeMissionCandidateSelectionRequest & request) noexcept
{
  OvertakeMissionCandidateSelection selection;
  constexpr double kEpsilon = 1e-9;
  const auto finite_positive = [](const double value) {
      return std::isfinite(value) && value > 0.0;
    };
  const auto finite_non_negative = [](const double value) {
      return std::isfinite(value) && value >= 0.0;
    };
  const bool horizon_request_valid = !request.horizon_progress_enabled ||
    (finite_positive(request.horizon_progress_time_budget_sec) &&
    finite_positive(request.horizon_progress_distance_budget_m) &&
    finite_positive(request.horizon_progress_reference_speed_mps) &&
    finite_positive(request.horizon_progress_maximum_closing_speed_mps) &&
    finite_positive(request.horizon_progress_lateral_motion_scale_m) &&
    finite_positive(request.horizon_progress_maximum_lateral_accel_mps2) &&
    finite_non_negative(request.horizon_progress_weights.rear_clear_time) &&
    finite_non_negative(request.horizon_progress_weights.rear_clear_distance) &&
    finite_non_negative(request.horizon_progress_weights.retained_speed) &&
    finite_non_negative(request.horizon_progress_weights.closing_speed) &&
    finite_non_negative(request.horizon_progress_weights.lateral_motion_penalty) &&
    finite_non_negative(request.horizon_progress_weights.lateral_accel_penalty));
  if (
    !std::isfinite(request.minimum_deadline_slack_sec) ||
    request.minimum_deadline_slack_sec < 0.0 ||
    !std::isfinite(request.minimum_clearance_advantage_m) ||
    request.minimum_clearance_advantage_m < 0.0 ||
    !std::isfinite(request.minimum_interaction_clearance_advantage_m) ||
    request.minimum_interaction_clearance_advantage_m < 0.0 ||
    !horizon_request_valid)
  {
    return selection;
  }
  selection.valid = true;

  const auto deadline_slack = [](const OvertakeMissionCandidate & candidate) {
      if (!std::isnan(candidate.body_clear_deadline_slack_sec)) {
        return candidate.body_clear_deadline_slack_sec;
      }
      if (
        candidate.body_clear_deadline_checked &&
        !std::isnan(candidate.predicted_hard_distance_time_sec) &&
        !std::isnan(candidate.predicted_body_clear_time_sec))
      {
        if (
          std::isinf(candidate.predicted_hard_distance_time_sec) &&
          std::isfinite(candidate.predicted_body_clear_time_sec))
        {
          return std::numeric_limits<double>::infinity();
        }
        if (
          std::isfinite(candidate.predicted_hard_distance_time_sec) &&
          std::isfinite(candidate.predicted_body_clear_time_sec))
        {
          return candidate.predicted_hard_distance_time_sec -
                 candidate.predicted_body_clear_time_sec;
        }
      }
      return -std::numeric_limits<double>::infinity();
    };

  const auto numerically_valid = [&](const OvertakeMissionCandidate & candidate) {
      const auto non_negative_or_infinity = [](const double value) {
          return !std::isnan(value) && value >= 0.0;
        };
      const bool finite_deadline_result =
        std::isfinite(candidate.predicted_body_clear_time_sec) &&
        candidate.predicted_body_clear_time_sec >= 0.0 &&
        std::isfinite(candidate.predicted_body_clear_distance_m) &&
        candidate.predicted_body_clear_distance_m >= 0.0;
      const bool rejected_deadline_result =
        !candidate.body_clear_deadline_feasible &&
        !std::isnan(candidate.predicted_body_clear_time_sec) &&
        candidate.predicted_body_clear_time_sec >= 0.0 &&
        !std::isnan(candidate.predicted_body_clear_distance_m) &&
        candidate.predicted_body_clear_distance_m >= 0.0;
      const bool deadline_valid = !candidate.body_clear_deadline_checked ||
        ((finite_deadline_result || rejected_deadline_result) &&
        !std::isnan(candidate.predicted_hard_distance_time_sec) &&
        candidate.predicted_hard_distance_time_sec >= 0.0);
      const bool closing_speed_valid =
        std::isnan(candidate.closing_speed_mps) ||
        (std::isfinite(candidate.closing_speed_mps) && candidate.closing_speed_mps >= 0.0);
      const bool slack_valid =
        !candidate.body_clear_deadline_checked ||
        !std::isnan(deadline_slack(candidate));
      const bool rear_clear_valid = !candidate.rear_clear_prediction_checked ||
        (std::isfinite(candidate.predicted_rear_clear_time_sec) &&
        candidate.predicted_rear_clear_time_sec >= 0.0 &&
        std::isfinite(candidate.predicted_rear_clear_ego_distance_m) &&
        candidate.predicted_rear_clear_ego_distance_m >= 0.0 &&
        std::isfinite(candidate.predicted_rear_clear_speed_mps) &&
        candidate.predicted_rear_clear_speed_mps >= 0.0 &&
        std::isfinite(candidate.pass_hold_distance_m) &&
        candidate.pass_hold_distance_m >= 0.0 &&
        std::isfinite(candidate.return_distance_m) &&
        candidate.return_distance_m >= 0.0 &&
        std::isfinite(candidate.static_valid_until_pass_m) &&
        candidate.static_valid_until_pass_m >= 0.0 &&
        std::isfinite(candidate.dynamic_valid_until_pass_m) &&
        candidate.dynamic_valid_until_pass_m >= 0.0 &&
        std::isfinite(candidate.planner_generated_at_sec) &&
        std::isfinite(candidate.prediction_source_age_sec) &&
        candidate.prediction_source_age_sec >= 0.0 &&
        std::isfinite(candidate.prediction_epoch_sec) &&
        std::isfinite(candidate.prediction_horizon_sec) &&
        candidate.prediction_horizon_sec >= 0.0 &&
        std::isfinite(candidate.dynamic_valid_until_sec));
      const bool progress_speed_valid = !request.horizon_progress_enabled ||
        (std::isfinite(candidate.predicted_minimum_ego_speed_mps) &&
        candidate.predicted_minimum_ego_speed_mps >= 0.0);
      const bool pass_target_clearance_valid =
        !candidate.pass_target_clearance_checked ||
        std::isfinite(
        candidate.predicted_minimum_pass_target_surface_clearance_m);
      const auto course_role_valid = [](const PassSideCourseRole role) {
          return role == PassSideCourseRole::Unknown ||
                 role == PassSideCourseRole::Inner ||
                 role == PassSideCourseRole::Outer;
        };
      const bool rear_clear_course_role_valid =
        !candidate.rear_clear_course_role_checked ||
        (course_role_valid(candidate.entry_course_role) &&
        course_role_valid(candidate.rear_clear_course_role) &&
        !std::isnan(candidate.first_course_role_reversal_distance_m) &&
        candidate.first_course_role_reversal_distance_m >= 0.0 &&
        (!candidate.inner_to_outer_at_rear_clear ||
        (candidate.entry_course_role == PassSideCourseRole::Inner &&
        candidate.rear_clear_course_role == PassSideCourseRole::Outer)));
      const bool outer_transition_valid = !candidate.outer_transition_required ||
        (candidate.outer_transition_preflight_validated &&
        (candidate.outer_transition_side_sign == -1 ||
        candidate.outer_transition_side_sign == 1) &&
        candidate.outer_transition_side_sign != candidate.pass_side_sign &&
        std::isfinite(candidate.outer_transition_start_pass_m) &&
        candidate.outer_transition_start_pass_m >= 0.0 &&
        std::isfinite(candidate.outer_transition_deadline_pass_m) &&
        candidate.outer_transition_deadline_pass_m + 1e-9 >=
        candidate.outer_transition_start_pass_m &&
        std::isfinite(candidate.outer_transition_goal_lateral_m) &&
        static_cast<double>(candidate.outer_transition_side_sign) *
        candidate.outer_transition_goal_lateral_m > 0.0 &&
        std::isfinite(candidate.outer_transition_shift_distance_m) &&
        candidate.outer_transition_shift_distance_m >= 0.5 &&
        candidate.outer_transition_shift_distance_m <=
        candidate.outer_transition_deadline_pass_m -
        candidate.outer_transition_start_pass_m + 1e-9);
      return deadline_valid && closing_speed_valid && slack_valid && rear_clear_valid &&
             progress_speed_valid && pass_target_clearance_valid &&
             rear_clear_course_role_valid && outer_transition_valid &&
             non_negative_or_infinity(candidate.minimum_path_wall_clearance_m) &&
             non_negative_or_infinity(candidate.minimum_path_corridor_width_m) &&
             non_negative_or_infinity(candidate.minimum_return_wall_clearance_m) &&
             std::isfinite(candidate.shift_distance_m) &&
             candidate.shift_distance_m >= 0.0 &&
             std::isfinite(candidate.goal_lateral_m) &&
             std::isfinite(candidate.lateral_shift_m) &&
             candidate.lateral_shift_m >= 0.0 &&
             std::isfinite(candidate.max_required_lateral_accel_mps2) &&
             candidate.max_required_lateral_accel_mps2 >= 0.0;
    };
  const auto physical_reserve = [](const OvertakeMissionCandidate & candidate) {
      double reserve = std::numeric_limits<double>::infinity();
      if (std::isfinite(candidate.minimum_path_wall_clearance_m)) {
        reserve = std::min(reserve, candidate.minimum_path_wall_clearance_m);
      }
      if (std::isfinite(candidate.minimum_path_corridor_width_m)) {
        reserve = std::min(reserve, 0.5 * candidate.minimum_path_corridor_width_m);
      }
      if (std::isfinite(candidate.minimum_return_wall_clearance_m)) {
        reserve = std::min(reserve, candidate.minimum_return_wall_clearance_m);
      }
      return reserve;
    };
  const auto interaction_reserve = [&](const OvertakeMissionCandidate & candidate) {
      double reserve = physical_reserve(candidate);
      if (
        candidate.pass_target_clearance_checked &&
        std::isfinite(
          candidate.predicted_minimum_pass_target_surface_clearance_m))
      {
        reserve = std::min(
          reserve,
          candidate.predicted_minimum_pass_target_surface_clearance_m);
      }
      return reserve;
    };
  const auto better = [&](
      const OvertakeMissionCandidate & candidate,
      const OvertakeMissionHorizonProgressEvaluation & candidate_progress,
      const OvertakeMissionCandidate & incumbent,
      const OvertakeMissionHorizonProgressEvaluation & incumbent_progress) {
      if (
        candidate.body_clear_deadline_checked !=
        incumbent.body_clear_deadline_checked)
      {
        return candidate.body_clear_deadline_checked;
      }
      if (
        candidate.body_clear_deadline_checked &&
        incumbent.body_clear_deadline_checked)
      {
        if (
          candidate.body_clear_deadline_feasible !=
          incumbent.body_clear_deadline_feasible)
        {
          return candidate.body_clear_deadline_feasible;
        }
        const double candidate_slack = deadline_slack(candidate);
        const double incumbent_slack = deadline_slack(incumbent);
        const bool candidate_has_reserve =
          candidate.body_clear_deadline_feasible &&
          candidate_slack + kEpsilon >= request.minimum_deadline_slack_sec;
        const bool incumbent_has_reserve =
          incumbent.body_clear_deadline_feasible &&
          incumbent_slack + kEpsilon >= request.minimum_deadline_slack_sec;
        if (candidate_has_reserve != incumbent_has_reserve) {
          return candidate_has_reserve;
        }
        if (!candidate_has_reserve && !incumbent_has_reserve) {
          if (candidate_slack > incumbent_slack + kEpsilon) {
            return true;
          }
          if (incumbent_slack > candidate_slack + kEpsilon) {
            return false;
          }
        }
      }
      if (
        request.rear_clear_side_selection_enabled &&
        candidate.rear_clear_course_role_checked &&
        incumbent.rear_clear_course_role_checked &&
        candidate.full_track_transition_before_rear_clear !=
        incumbent.full_track_transition_before_rear_clear)
      {
        return !candidate.full_track_transition_before_rear_clear;
      }
      if (
        candidate.pass_target_clearance_checked &&
        incumbent.pass_target_clearance_checked)
      {
        const double candidate_interaction_reserve = interaction_reserve(candidate);
        const double incumbent_interaction_reserve = interaction_reserve(incumbent);
        if (
          std::isfinite(candidate_interaction_reserve) &&
          std::isfinite(incumbent_interaction_reserve))
        {
          if (
            candidate_interaction_reserve > incumbent_interaction_reserve +
            request.minimum_interaction_clearance_advantage_m + kEpsilon)
          {
            return true;
          }
          if (
            incumbent_interaction_reserve > candidate_interaction_reserve +
            request.minimum_interaction_clearance_advantage_m + kEpsilon)
          {
            return false;
          }
        }
      }
      const double candidate_physical_reserve = physical_reserve(candidate);
      const double incumbent_physical_reserve = physical_reserve(incumbent);
      if (
        std::isfinite(candidate_physical_reserve) &&
        std::isfinite(incumbent_physical_reserve))
      {
        if (
          candidate_physical_reserve > incumbent_physical_reserve +
          request.minimum_clearance_advantage_m + kEpsilon)
        {
          return true;
        }
        if (
          incumbent_physical_reserve > candidate_physical_reserve +
          request.minimum_clearance_advantage_m + kEpsilon)
        {
          return false;
        }
      }
      if (request.horizon_progress_enabled) {
        if (candidate_progress.score > incumbent_progress.score + kEpsilon) {
          return true;
        }
        if (incumbent_progress.score > candidate_progress.score + kEpsilon) {
          return false;
        }
      }
      if (
        candidate.rear_clear_prediction_checked &&
        incumbent.rear_clear_prediction_checked)
      {
        if (
          candidate.predicted_rear_clear_time_sec + kEpsilon <
          incumbent.predicted_rear_clear_time_sec)
        {
          return true;
        }
        if (
          incumbent.predicted_rear_clear_time_sec + kEpsilon <
          candidate.predicted_rear_clear_time_sec)
        {
          return false;
        }
      }
      if (candidate.direct_pass != incumbent.direct_pass) {
        return candidate.direct_pass;
      }
      if (
        candidate.body_clear_deadline_checked &&
        incumbent.body_clear_deadline_checked)
      {
        if (
          candidate.predicted_body_clear_time_sec + kEpsilon <
          incumbent.predicted_body_clear_time_sec)
        {
          return true;
        }
        if (
          incumbent.predicted_body_clear_time_sec + kEpsilon <
          candidate.predicted_body_clear_time_sec)
        {
          return false;
        }
      }
      if (candidate.shift_distance_m + kEpsilon < incumbent.shift_distance_m) {
        return true;
      }
      if (incumbent.shift_distance_m + kEpsilon < candidate.shift_distance_m) {
        return false;
      }
      if (candidate.lateral_shift_m + kEpsilon < incumbent.lateral_shift_m) {
        return true;
      }
      if (incumbent.lateral_shift_m + kEpsilon < candidate.lateral_shift_m) {
        return false;
      }
      if (
        candidate.max_required_lateral_accel_mps2 + kEpsilon <
        incumbent.max_required_lateral_accel_mps2)
      {
        return true;
      }
      if (
        incumbent.max_required_lateral_accel_mps2 + kEpsilon <
        candidate.max_required_lateral_accel_mps2)
      {
        return false;
      }
      if (
        std::isfinite(candidate.closing_speed_mps) &&
        std::isfinite(incumbent.closing_speed_mps))
      {
        if (candidate.closing_speed_mps > incumbent.closing_speed_mps + kEpsilon) {
          return true;
        }
        if (incumbent.closing_speed_mps > candidate.closing_speed_mps + kEpsilon) {
          return false;
        }
      }
      if (
        request.rear_clear_side_selection_enabled &&
        candidate.rear_clear_course_role_checked &&
        incumbent.rear_clear_course_role_checked)
      {
        const bool candidate_exits_outer =
          candidate.rear_clear_course_role == PassSideCourseRole::Outer;
        const bool incumbent_exits_outer =
          incumbent.rear_clear_course_role == PassSideCourseRole::Outer;
        if (candidate_exits_outer != incumbent_exits_outer) {
          return candidate_exits_outer;
        }
      }
      return std::abs(candidate.goal_lateral_m) + kEpsilon <
             std::abs(incumbent.goal_lateral_m);
    };

  for (std::size_t i = 0; i < request.candidates.size(); ++i) {
    const auto & candidate = request.candidates[i];
    if (!numerically_valid(candidate)) {
      ++selection.invalid_candidate_count;
      continue;
    }
    const auto horizon_progress = evaluate_overtake_mission_horizon_progress(
      OvertakeMissionHorizonProgressRequest{
        request.horizon_progress_enabled,
        candidate,
        request.horizon_progress_time_budget_sec,
        request.horizon_progress_distance_budget_m,
        request.horizon_progress_reference_speed_mps,
        request.horizon_progress_maximum_closing_speed_mps,
        request.horizon_progress_lateral_motion_scale_m,
        request.horizon_progress_maximum_lateral_accel_mps2,
        request.horizon_progress_weights});
    if (!horizon_progress.valid) {
      selection.valid = false;
      selection.found = false;
      selection.selected_index = std::numeric_limits<std::size_t>::max();
      selection.candidate = OvertakeMissionCandidate{};
      selection.horizon_progress = OvertakeMissionHorizonProgressEvaluation{};
      return selection;
    }
    if (
      !candidate.feasible ||
      (request.horizon_progress_enabled && !horizon_progress.hard_feasible))
    {
      continue;
    }
    if (
      !selection.found ||
      better(candidate, horizon_progress, selection.candidate, selection.horizon_progress))
    {
      selection.found = true;
      selection.selected_index = i;
      selection.candidate = candidate;
      selection.horizon_progress = horizon_progress;
    }
  }
  return selection;
}

const char * to_string(const MpccLiteShadowBranch branch) noexcept
{
  switch (branch) {
    case MpccLiteShadowBranch::None:
      return "none";
    case MpccLiteShadowBranch::Left:
      return "left";
    case MpccLiteShadowBranch::Right:
      return "right";
    case MpccLiteShadowBranch::CurrentSideHold:
      return "hold";
    case MpccLiteShadowBranch::Return:
      return "return";
  }
  return "unknown";
}

const char * to_string(const MpccLiteShadowRejectReason reason) noexcept
{
  switch (reason) {
    case MpccLiteShadowRejectReason::None:
      return "none";
    case MpccLiteShadowRejectReason::Disabled:
      return "disabled";
    case MpccLiteShadowRejectReason::InvalidRequest:
      return "invalid_request";
    case MpccLiteShadowRejectReason::Unavailable:
      return "unavailable";
    case MpccLiteShadowRejectReason::PlanningUnavailable:
      return "planning_unavailable";
    case MpccLiteShadowRejectReason::HardConstraint:
      return "hard_constraint";
    case MpccLiteShadowRejectReason::InvalidCandidate:
      return "invalid_candidate";
    case MpccLiteShadowRejectReason::MissionInfeasible:
      return "mission_infeasible";
    case MpccLiteShadowRejectReason::ProgressiveEntryIncomplete:
      return "progressive_entry_incomplete";
    case MpccLiteShadowRejectReason::RearClearUnchecked:
      return "rear_clear_unchecked";
    case MpccLiteShadowRejectReason::RearClearInfeasible:
      return "rear_clear_infeasible";
    case MpccLiteShadowRejectReason::RearClearTimeBudget:
      return "rear_clear_time_budget";
    case MpccLiteShadowRejectReason::RearClearDistanceBudget:
      return "rear_clear_distance_budget";
    case MpccLiteShadowRejectReason::TargetClearanceUnchecked:
      return "target_clearance_unchecked";
    case MpccLiteShadowRejectReason::OuterTransitionUnvalidated:
      return "outer_transition_unvalidated";
    case MpccLiteShadowRejectReason::RuntimeHardFault:
      return "runtime_hard_fault";
    case MpccLiteShadowRejectReason::MissionTotalTimeBudget:
      return "mission_total_time_budget";
    case MpccLiteShadowRejectReason::SafeSeparationTimeBudget:
      return "safe_separation_time_budget";
    case MpccLiteShadowRejectReason::SafeSeparationDistanceBudget:
      return "safe_separation_distance_budget";
    case MpccLiteShadowRejectReason::ReturnNotAdmitted:
      return "return_not_admitted";
    case MpccLiteShadowRejectReason::ReturnCorridorBlocked:
      return "return_corridor_blocked";
  }
  return "unknown";
}

MpccLiteShadowCandidate build_mpcc_lite_shadow_mission_candidate(
  const MpccLiteShadowMissionCandidateRequest & request) noexcept
{
  MpccLiteShadowCandidate candidate;
  candidate.branch = request.branch;
  candidate.assessed = request.assessed;
  const auto reject = [&](const MpccLiteShadowRejectReason reason) {
    candidate.admission_reject_reason = reason;
    return candidate;
  };
  if (request.branch == MpccLiteShadowBranch::None) {
    return reject(MpccLiteShadowRejectReason::InvalidCandidate);
  }
  if (!request.assessed) {
    return reject(MpccLiteShadowRejectReason::Unavailable);
  }
  if (!request.mission.has_value()) {
    return reject(MpccLiteShadowRejectReason::PlanningUnavailable);
  }

  const auto & mission = request.mission.value();
  candidate.available = true;
  candidate.rear_clear_required = true;
  candidate.rear_clear_feasible = mission.rear_clear_prediction_feasible;
  candidate.predicted_rear_clear_time_sec = mission.predicted_rear_clear_time_sec;
  candidate.predicted_rear_clear_distance_m = mission.predicted_rear_clear_ego_distance_m;
  candidate.predicted_minimum_speed_mps = mission.predicted_minimum_ego_speed_mps;
  const auto finite_reserve = [](const double metric, const double fallback) {
      return std::isfinite(metric) ? std::max(0.0, metric) : std::max(0.0, fallback);
    };
  candidate.minimum_wall_clearance_m = std::min(
    finite_reserve(mission.minimum_path_wall_clearance_m, request.fallback_wall_clearance_m),
    std::min(
      finite_reserve(
        mission.minimum_return_wall_clearance_m, request.fallback_wall_clearance_m),
      0.5 * finite_reserve(
        mission.minimum_path_corridor_width_m,
        2.0 * request.fallback_wall_clearance_m)));
  candidate.minimum_target_clearance_m = finite_reserve(
    mission.predicted_minimum_pass_target_surface_clearance_m,
    request.fallback_target_clearance_m);
  candidate.maximum_lateral_accel_mps2 = mission.max_required_lateral_accel_mps2;
  candidate.lateral_motion_m = mission.lateral_shift_m;

  if (!mission.feasible) {
    return reject(MpccLiteShadowRejectReason::MissionInfeasible);
  }
  if (mission.progressive_entry) {
    return reject(MpccLiteShadowRejectReason::ProgressiveEntryIncomplete);
  }
  if (!mission.rear_clear_prediction_checked) {
    return reject(MpccLiteShadowRejectReason::RearClearUnchecked);
  }
  if (!mission.rear_clear_prediction_feasible) {
    return reject(MpccLiteShadowRejectReason::RearClearInfeasible);
  }
  if (!mission.pass_target_clearance_checked) {
    return reject(MpccLiteShadowRejectReason::TargetClearanceUnchecked);
  }
  if (mission.outer_transition_required && !mission.outer_transition_preflight_validated) {
    return reject(MpccLiteShadowRejectReason::OuterTransitionUnvalidated);
  }
  if (request.runtime_hard_fault) {
    return reject(MpccLiteShadowRejectReason::RuntimeHardFault);
  }
  constexpr double kEpsilon = 1e-9;
  if (
    request.mission_time_budget_active &&
    (!std::isfinite(request.mission_time_remaining_sec) ||
    mission.predicted_rear_clear_time_sec >
    request.mission_time_remaining_sec + kEpsilon))
  {
    return reject(MpccLiteShadowRejectReason::MissionTotalTimeBudget);
  }
  if (
    request.safe_separation_budget_active &&
    (!std::isfinite(request.safe_separation_time_remaining_sec) ||
    mission.predicted_rear_clear_time_sec >
    request.safe_separation_time_remaining_sec + kEpsilon))
  {
    return reject(MpccLiteShadowRejectReason::SafeSeparationTimeBudget);
  }
  if (
    request.safe_separation_budget_active &&
    (!std::isfinite(request.safe_separation_distance_remaining_m) ||
    mission.predicted_rear_clear_ego_distance_m >
    request.safe_separation_distance_remaining_m + kEpsilon))
  {
    return reject(MpccLiteShadowRejectReason::SafeSeparationDistanceBudget);
  }
  candidate.hard_feasible = true;
  candidate.admission_reject_reason = MpccLiteShadowRejectReason::None;
  return candidate;
}

MpccLiteShadowRejectReason resolve_mpcc_lite_shadow_return_admission(
  const MpccLiteShadowReturnAdmissionRequest & request) noexcept
{
  if (!request.phase_relevant) {
    return MpccLiteShadowRejectReason::Unavailable;
  }
  if (request.runtime_hard_fault) {
    return MpccLiteShadowRejectReason::RuntimeHardFault;
  }
  if (request.return_corridor_blocked) {
    return MpccLiteShadowRejectReason::ReturnCorridorBlocked;
  }
  if (!request.return_active && !request.rear_clear_confirmed) {
    return MpccLiteShadowRejectReason::ReturnNotAdmitted;
  }
  return MpccLiteShadowRejectReason::None;
}

MpccLiteShadowResolution evaluate_mpcc_lite_shadow(
  const MpccLiteShadowRequest & request) noexcept
{
  MpccLiteShadowResolution resolution;
  const auto finite_positive = [](const double value) {
      return std::isfinite(value) && value > 0.0;
    };
  const auto finite_non_negative = [](const double value) {
      return std::isfinite(value) && value >= 0.0;
    };
  const auto valid_weight = [&](const double value) {
      return finite_non_negative(value);
    };
  if (!request.enabled) {
    resolution.valid = true;
    return resolution;
  }
  if (
    !finite_positive(request.rear_clear_time_budget_sec) ||
    !finite_positive(request.rear_clear_distance_budget_m) ||
    !finite_positive(request.reference_speed_mps) ||
    !finite_positive(request.reference_wall_clearance_m) ||
    !finite_positive(request.reference_target_clearance_m) ||
    !finite_positive(request.lateral_motion_scale_m) ||
    !finite_positive(request.maximum_lateral_accel_mps2) ||
    !valid_weight(request.weights.rear_clear_time) ||
    !valid_weight(request.weights.rear_clear_distance) ||
    !valid_weight(request.weights.retained_speed) ||
    !valid_weight(request.weights.wall_clearance) ||
    !valid_weight(request.weights.target_clearance) ||
    !valid_weight(request.weights.lateral_motion_penalty) ||
    !valid_weight(request.weights.lateral_accel_penalty) ||
    !valid_weight(request.weights.branch_switch_penalty))
  {
    return resolution;
  }
  resolution.valid = true;
  constexpr double kEpsilon = 1e-9;
  for (const auto & candidate : request.candidates) {
    MpccLiteShadowEvaluation evaluation;
    evaluation.checked = true;
    evaluation.candidate = candidate;
    if (candidate.branch == MpccLiteShadowBranch::None) {
      evaluation.reject_reason = MpccLiteShadowRejectReason::InvalidCandidate;
    } else if (!candidate.available) {
      evaluation.valid = true;
      evaluation.reject_reason =
        candidate.admission_reject_reason == MpccLiteShadowRejectReason::None ?
        MpccLiteShadowRejectReason::Unavailable : candidate.admission_reject_reason;
    } else if (!candidate.hard_feasible) {
      evaluation.valid = true;
      evaluation.reject_reason =
        candidate.admission_reject_reason == MpccLiteShadowRejectReason::None ?
        MpccLiteShadowRejectReason::HardConstraint : candidate.admission_reject_reason;
    } else {
      const bool numeric_valid =
        finite_non_negative(candidate.predicted_minimum_speed_mps) &&
        finite_non_negative(candidate.minimum_wall_clearance_m) &&
        finite_non_negative(candidate.minimum_target_clearance_m) &&
        finite_non_negative(candidate.maximum_lateral_accel_mps2) &&
        finite_non_negative(candidate.lateral_motion_m) &&
        (!candidate.rear_clear_required ||
        (finite_non_negative(candidate.predicted_rear_clear_time_sec) &&
        finite_non_negative(candidate.predicted_rear_clear_distance_m)));
      if (!numeric_valid) {
        evaluation.reject_reason = MpccLiteShadowRejectReason::InvalidCandidate;
      } else if (candidate.rear_clear_required && !candidate.rear_clear_feasible) {
        evaluation.valid = true;
        evaluation.reject_reason = MpccLiteShadowRejectReason::RearClearInfeasible;
      } else if (
        candidate.rear_clear_required &&
        candidate.predicted_rear_clear_time_sec >
        request.rear_clear_time_budget_sec + kEpsilon)
      {
        evaluation.valid = true;
        evaluation.reject_reason = MpccLiteShadowRejectReason::RearClearTimeBudget;
      } else if (
        candidate.rear_clear_required &&
        candidate.predicted_rear_clear_distance_m >
        request.rear_clear_distance_budget_m + kEpsilon)
      {
        evaluation.valid = true;
        evaluation.reject_reason = MpccLiteShadowRejectReason::RearClearDistanceBudget;
      } else {
        evaluation.valid = true;
        evaluation.hard_feasible = true;
        evaluation.reject_reason = MpccLiteShadowRejectReason::None;
        evaluation.rear_clear_time_progress = candidate.rear_clear_required ?
          std::clamp(
          1.0 - candidate.predicted_rear_clear_time_sec /
          request.rear_clear_time_budget_sec, 0.0, 1.0) : 1.0;
        evaluation.rear_clear_distance_progress = candidate.rear_clear_required ?
          std::clamp(
          1.0 - candidate.predicted_rear_clear_distance_m /
          request.rear_clear_distance_budget_m, 0.0, 1.0) : 1.0;
        evaluation.retained_speed = std::clamp(
          candidate.predicted_minimum_speed_mps / request.reference_speed_mps,
          0.0, 1.0);
        evaluation.wall_clearance_reserve = std::clamp(
          candidate.minimum_wall_clearance_m /
          request.reference_wall_clearance_m, 0.0, 1.0);
        evaluation.target_clearance_reserve = std::clamp(
          candidate.minimum_target_clearance_m /
          request.reference_target_clearance_m, 0.0, 1.0);
        evaluation.lateral_motion_cost = std::clamp(
          candidate.lateral_motion_m / request.lateral_motion_scale_m, 0.0, 1.0);
        evaluation.lateral_accel_cost = std::clamp(
          candidate.maximum_lateral_accel_mps2 /
          request.maximum_lateral_accel_mps2, 0.0, 1.0);
        evaluation.branch_switch_cost =
          request.active_branch != MpccLiteShadowBranch::None &&
          candidate.branch != request.active_branch ? 1.0 : 0.0;
        evaluation.score =
          request.weights.rear_clear_time * evaluation.rear_clear_time_progress +
          request.weights.rear_clear_distance * evaluation.rear_clear_distance_progress +
          request.weights.retained_speed * evaluation.retained_speed +
          request.weights.wall_clearance * evaluation.wall_clearance_reserve +
          request.weights.target_clearance * evaluation.target_clearance_reserve -
          request.weights.lateral_motion_penalty * evaluation.lateral_motion_cost -
          request.weights.lateral_accel_penalty * evaluation.lateral_accel_cost -
          request.weights.branch_switch_penalty * evaluation.branch_switch_cost;
      }
    }

    if (candidate.branch == request.active_branch) {
      resolution.active_evaluation = evaluation;
    }
    if (evaluation.valid && evaluation.hard_feasible) {
      const bool better_score = !resolution.found ||
        evaluation.score > resolution.best.score + kEpsilon;
      const bool equal_score = resolution.found &&
        std::abs(evaluation.score - resolution.best.score) <= kEpsilon;
      const bool active_tie_break = equal_score &&
        candidate.branch == request.active_branch &&
        resolution.best.candidate.branch != request.active_branch;
      const bool lower_motion_tie_break = equal_score && !active_tie_break &&
        candidate.lateral_motion_m + kEpsilon <
        resolution.best.candidate.lateral_motion_m;
      if (better_score || active_tie_break || lower_motion_tie_break) {
        resolution.found = true;
        resolution.best = evaluation;
      }
    }
    resolution.evaluations.push_back(evaluation);
  }
  resolution.agrees_with_active_branch = resolution.found &&
    resolution.best.candidate.branch == request.active_branch;
  return resolution;
}

bool can_reuse_mpcc_lite_shadow_last_feasible(
  const MpccLiteShadowLeaseRequest & request) noexcept
{
  return request.has_last_feasible && request.target_matches &&
         request.mission_generation_matches && request.phase_matches &&
         request.side_matches && std::isfinite(request.now_sec) &&
         std::isfinite(request.last_feasible_sec) &&
         std::isfinite(request.maximum_age_sec) &&
         request.maximum_age_sec >= 0.0 &&
         request.now_sec + 1e-9 >= request.last_feasible_sec &&
         request.now_sec - request.last_feasible_sec <=
         request.maximum_age_sec + 1e-9;
}

bool should_arm_overtake_side_retry_block(
  const OvertakeSideRetryFailureClass failure_class) noexcept
{
  return failure_class ==
         OvertakeSideRetryFailureClass::PhysicalOrCommittedFailure;
}

RuntimeWallPreplanResolution resolve_runtime_wall_preplan(
  const RuntimeWallPreplanRequest & request) noexcept
{
  RuntimeWallPreplanResolution resolution;
  const bool mission_side_valid =
    request.mission_side_sign == -1 || request.mission_side_sign == 1;
  const bool candidate_side_valid = !request.fresh_candidate_available ||
    request.candidate_side_sign == -1 || request.candidate_side_sign == 1;
  if (
    !std::isfinite(request.now_sec) ||
    (!std::isfinite(request.last_replan_sec) &&
    request.last_replan_sec != -std::numeric_limits<double>::infinity()) ||
    !std::isfinite(request.cooldown_sec) || request.cooldown_sec < 0.0 ||
    !std::isfinite(request.warning_elapsed_sec) || request.warning_elapsed_sec < 0.0 ||
    !std::isfinite(request.fallback_delay_sec) || request.fallback_delay_sec < 0.0 ||
    request.replan_count < 0 || request.maximum_replan_count < 0 ||
    !candidate_side_valid)
  {
    return resolution;
  }
  resolution.valid = true;
  if (
    !request.enabled || !request.active_execution ||
    !request.warning_margin_blocked)
  {
    return resolution;
  }
  if (
    !mission_side_valid || request.hard_wall_fault ||
    !request.target_continuous || !request.current_body_separated ||
    !request.target_prediction_valid)
  {
    return resolution;
  }
  if (
    std::isfinite(request.last_replan_sec) &&
    request.now_sec - request.last_replan_sec < request.cooldown_sec)
  {
    if (
      request.warning_elapsed_sec + 1e-9 >= request.fallback_delay_sec &&
      request.speed_preserving_return_available)
    {
      resolution.action = RuntimeWallPreplanAction::ReturnToBaseLine;
    }
    return resolution;
  }
  if (
    request.fresh_candidate_available &&
    request.replan_count < request.maximum_replan_count &&
    request.candidate_side_sign == request.mission_side_sign)
  {
    resolution.action = RuntimeWallPreplanAction::ReplaceWithFreshSameSide;
  } else if (request.warning_elapsed_sec < request.fallback_delay_sec) {
    resolution.action = RuntimeWallPreplanAction::RequestFreshSameSideCandidate;
  } else if (
    request.center_contraction_available &&
    request.replan_count < request.maximum_replan_count)
  {
    resolution.action = RuntimeWallPreplanAction::ContractTowardCenter;
  } else if (request.speed_preserving_return_available) {
    resolution.action = RuntimeWallPreplanAction::ReturnToBaseLine;
  } else if (request.replan_count < request.maximum_replan_count) {
    resolution.action = RuntimeWallPreplanAction::RequestFreshSameSideCandidate;
  }
  return resolution;
}

bool should_throttle_cross_side_replacement_retry(
  const CrossSideReplacementRetryThrottleRequest & request) noexcept
{
  if (
    !request.side_changed ||
    (request.candidate_side_sign != -1 && request.candidate_side_sign != 1) ||
    request.candidate_side_sign != request.rejected_side_sign ||
    !std::isfinite(request.candidate_goal_lateral_m) ||
    !std::isfinite(request.rejected_goal_lateral_m) ||
    !std::isfinite(request.now_sec) ||
    !std::isfinite(request.rejected_at_sec) ||
    !std::isfinite(request.cooldown_sec) || request.cooldown_sec <= 0.0 ||
    !std::isfinite(request.goal_change_tolerance_m) ||
    request.goal_change_tolerance_m < 0.0)
  {
    return false;
  }
  const double elapsed_sec = request.now_sec - request.rejected_at_sec;
  return elapsed_sec >= 0.0 && elapsed_sec < request.cooldown_sec &&
         std::abs(
    request.candidate_goal_lateral_m - request.rejected_goal_lateral_m) <=
         request.goal_change_tolerance_m + 1e-9;
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

ReachableLateralTargetResolution resolve_reachable_lateral_target(
  const ReachableLateralTargetRequest & request) noexcept
{
  ReachableLateralTargetResolution result;
  if (
    !std::isfinite(request.current_lateral_m) ||
    !std::isfinite(request.desired_lateral_m) ||
    !std::isfinite(request.time_to_target_sec) || request.time_to_target_sec <= 0.0 ||
    !std::isfinite(request.maximum_lateral_accel_mps2) ||
    request.maximum_lateral_accel_mps2 <= 0.0 ||
    !std::isfinite(request.initial_lateral_velocity_mps))
  {
    return result;
  }

  result.valid = true;
  const double zero_acceleration_lateral_m =
    request.current_lateral_m +
    request.initial_lateral_velocity_mps * request.time_to_target_sec;
  const double lateral_delta =
    request.desired_lateral_m - zero_acceleration_lateral_m;
  result.required_lateral_accel_mps2 =
    2.0 * std::abs(lateral_delta) /
    (request.time_to_target_sec * request.time_to_target_sec);
  result.target_lateral_m = request.desired_lateral_m;
  if (result.required_lateral_accel_mps2 <= request.maximum_lateral_accel_mps2) {
    return result;
  }

  const double reachable_shift =
    0.5 * request.maximum_lateral_accel_mps2 *
    request.time_to_target_sec * request.time_to_target_sec;
  result.target_lateral_m = zero_acceleration_lateral_m +
    std::copysign(reachable_shift, lateral_delta);
  result.required_lateral_accel_mps2 = request.maximum_lateral_accel_mps2;
  result.limited = true;
  return result;
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

PassLateralGoalPolicyResolution resolve_pass_lateral_goal_policy(
  const PassLateralGoalPolicyRequest & request) noexcept
{
  PassLateralGoalPolicyResolution resolution;
  resolution.preferred_goal_m = resolve_pass_side_lateral_goal(
    PassSideLateralGoalRequest{
      request.pass_side_sign,
      request.base_lateral_offset_m,
      request.pass_goal_target_lateral_m,
      request.minimum_separation_m,
      request.fixed_lateral_goal_m});
  resolution.execution_goal_m = resolution.preferred_goal_m;
  resolution.target_separation_feasible = !request.enforce_target_separation;
  if (!request.feasible_interval_available) {
    return resolution;
  }

  const auto feasible_goal = resolve_feasible_pass_side_lateral_goal(
    FeasiblePassSideLateralGoalRequest{
      request.pass_side_sign,
      resolution.preferred_goal_m,
      request.separation_target_lateral_m,
      request.minimum_separation_m,
      request.feasible_lower_bound_m,
      request.feasible_upper_bound_m,
      request.enforce_target_separation});
  resolution.execution_goal_m = feasible_goal.goal_m;
  resolution.target_separation_feasible = feasible_goal.target_separation_feasible;
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

MinimumLateralMotionGoalResolution resolve_minimum_lateral_motion_goal(
  const MinimumLateralMotionGoalRequest & request) noexcept
{
  MinimumLateralMotionGoalResolution resolution;
  if (
    !std::isfinite(request.base_line_lateral_m) ||
    !std::isfinite(request.current_lateral_m) ||
    !std::isfinite(request.feasible_lower_bound_m) ||
    !std::isfinite(request.feasible_upper_bound_m) ||
    !std::isfinite(request.preferred_target_clearance_buffer_m) ||
    request.preferred_target_clearance_buffer_m < 0.0 ||
    request.feasible_upper_bound_m < request.feasible_lower_bound_m)
  {
    return resolution;
  }

  if (
    request.preferred_target_clearance_buffer_m > 0.0 &&
    request.pass_side_sign != -1 && request.pass_side_sign != 1)
  {
    return resolution;
  }

  double preferred_lower_bound_m = request.feasible_lower_bound_m;
  double preferred_upper_bound_m = request.feasible_upper_bound_m;
  const double corridor_width_m =
    request.feasible_upper_bound_m - request.feasible_lower_bound_m;
  resolution.applied_target_clearance_buffer_m = std::min(
    request.preferred_target_clearance_buffer_m,
    0.5 * corridor_width_m);
  if (request.pass_side_sign > 0) {
    preferred_lower_bound_m += resolution.applied_target_clearance_buffer_m;
  } else if (request.pass_side_sign < 0) {
    preferred_upper_bound_m -= resolution.applied_target_clearance_buffer_m;
  }

  resolution.valid = true;
  resolution.base_line_clear =
    preferred_lower_bound_m <= request.base_line_lateral_m + 1e-9 &&
    request.base_line_lateral_m <= preferred_upper_bound_m + 1e-9;
  resolution.current_position_clear =
    preferred_lower_bound_m <= request.current_lateral_m + 1e-9 &&
    request.current_lateral_m <= preferred_upper_bound_m + 1e-9;
  resolution.goal_m = std::min(
    preferred_upper_bound_m,
    std::max(preferred_lower_bound_m, request.base_line_lateral_m));
  resolution.required_shift_m = std::abs(resolution.goal_m - request.current_lateral_m);
  return resolution;
}

MinimumMotionDirectPassResolution resolve_minimum_motion_direct_pass(
  const MinimumMotionDirectPassRequest & request) noexcept
{
  MinimumMotionDirectPassResolution resolution;
  if (
    !std::isfinite(request.lateral_shift_m) || request.lateral_shift_m < 0.0 ||
    !std::isfinite(request.maximum_tiny_shift_m) || request.maximum_tiny_shift_m < 0.0)
  {
    return resolution;
  }

  resolution.valid = true;
  resolution.base_line_direct_pass = request.base_line_direct_pass;
  resolution.tiny_shift_direct_pass =
    !request.base_line_direct_pass && request.tiny_shift_enabled &&
    request.current_position_clear && request.body_clear_at_entry &&
    request.lateral_shift_m <= request.maximum_tiny_shift_m + 1e-9;
  resolution.direct_pass =
    resolution.base_line_direct_pass || resolution.tiny_shift_direct_pass;
  return resolution;
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

UnseparatedClosingReserveResolution resolve_unseparated_closing_reserve(
  const UnseparatedClosingReserveRequest & request)
{
  UnseparatedClosingReserveResolution resolution;
  resolution.closing_speed_limit_mps = request.current_closing_speed_limit_mps;
  resolution.eligible =
    !request.current_body_footprints_separated &&
    std::isfinite(request.target_longitudinal_m) &&
    request.target_longitudinal_m > 0.0;
  if (!resolution.eligible) {
    return resolution;
  }

  resolution.protected_front_distance_m =
    std::max(
    std::max(0.0, request.moving_front_hard_distance_m),
    std::max(0.0, request.body_longitudinal_clearance_m)) +
    std::max(0.0, request.reserve_distance_m);
  const auto adaptive = resolve_adaptive_shiftout_closing_speed(
    AdaptiveShiftOutClosingSpeedRequest{
      0.0,
      request.current_closing_speed_limit_mps,
      std::max(0.0, request.target_longitudinal_m),
      resolution.protected_front_distance_m,
      request.remaining_lateral_execution_distance_m,
      request.ego_speed_mps,
      request.minimum_speed_mps,
      request.minimum_time_sec});
  resolution.closing_speed_limit_mps = adaptive.closing_speed_mps;
  resolution.limited =
    resolution.closing_speed_limit_mps +
    std::max(0.0, request.limiting_tolerance_mps) <
    request.current_closing_speed_limit_mps;
  return resolution;
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

OpponentMotionFilterResolution update_opponent_motion_filter(
  const OpponentMotionFilterRequest & request) noexcept
{
  OpponentMotionFilterResolution result;
  if (
    !std::isfinite(request.observed_velocity_x_mps) ||
    !std::isfinite(request.observed_velocity_y_mps) ||
    !std::isfinite(request.sample_interval_sec) || request.sample_interval_sec <= 0.0 ||
    !std::isfinite(request.velocity_gain) || request.velocity_gain < 0.0 ||
    request.velocity_gain > 1.0 ||
    !std::isfinite(request.acceleration_gain) || request.acceleration_gain < 0.0 ||
    request.acceleration_gain > 1.0 ||
    !std::isfinite(request.maximum_acceleration_mps2) ||
    request.maximum_acceleration_mps2 < 0.0)
  {
    return result;
  }

  if (!request.previous_estimate_valid) {
    result.valid = true;
    result.velocity_x_mps = request.observed_velocity_x_mps;
    result.velocity_y_mps = request.observed_velocity_y_mps;
    return result;
  }
  if (
    !std::isfinite(request.previous_velocity_x_mps) ||
    !std::isfinite(request.previous_velocity_y_mps) ||
    !std::isfinite(request.previous_acceleration_x_mps2) ||
    !std::isfinite(request.previous_acceleration_y_mps2))
  {
    return result;
  }

  const double velocity_gain = request.velocity_gain;
  result.velocity_x_mps = request.previous_velocity_x_mps + velocity_gain *
    (request.observed_velocity_x_mps - request.previous_velocity_x_mps);
  result.velocity_y_mps = request.previous_velocity_y_mps + velocity_gain *
    (request.observed_velocity_y_mps - request.previous_velocity_y_mps);
  const double observed_acceleration_x =
    (result.velocity_x_mps - request.previous_velocity_x_mps) /
    request.sample_interval_sec;
  const double observed_acceleration_y =
    (result.velocity_y_mps - request.previous_velocity_y_mps) /
    request.sample_interval_sec;
  const double acceleration_gain = request.acceleration_gain;
  result.acceleration_x_mps2 = request.previous_acceleration_x_mps2 + acceleration_gain *
    (observed_acceleration_x - request.previous_acceleration_x_mps2);
  result.acceleration_y_mps2 = request.previous_acceleration_y_mps2 + acceleration_gain *
    (observed_acceleration_y - request.previous_acceleration_y_mps2);
  const double acceleration_magnitude = std::hypot(
    result.acceleration_x_mps2, result.acceleration_y_mps2);
  if (
    acceleration_magnitude > request.maximum_acceleration_mps2 &&
    acceleration_magnitude > 1e-9)
  {
    const double scale = request.maximum_acceleration_mps2 / acceleration_magnitude;
    result.acceleration_x_mps2 *= scale;
    result.acceleration_y_mps2 *= scale;
  }
  result.valid =
    std::isfinite(result.velocity_x_mps) && std::isfinite(result.velocity_y_mps) &&
    std::isfinite(result.acceleration_x_mps2) &&
    std::isfinite(result.acceleration_y_mps2);
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
    request.completion_feasible || request.validated_full_mission || request.line_committed ||
    request.curve_continuation_allowed ||
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

OvertakeEntrySpeedReadiness update_overtake_entry_speed_readiness(
  const OvertakeEntrySpeedReadinessRequest & request) noexcept
{
  OvertakeEntrySpeedReadiness result;
  if (
    !request.monitor_active || !std::isfinite(request.now_sec) ||
    !std::isfinite(request.ego_speed_mps) || request.ego_speed_mps < 0.0 ||
    !std::isfinite(request.target_speed_mps) || request.target_speed_mps < 0.0 ||
    !std::isfinite(request.minimum_relative_speed_mps) ||
    !std::isfinite(request.confirm_sec) || request.confirm_sec < 0.0)
  {
    return result;
  }

  result.relative_speed_mps = request.ego_speed_mps - request.target_speed_mps;
  if (result.relative_speed_mps + 1e-9 < request.minimum_relative_speed_mps) {
    return result;
  }

  result.ready_since_sec =
    request.same_target && std::isfinite(request.ready_since_sec) &&
    request.ready_since_sec <= request.now_sec ?
    request.ready_since_sec : request.now_sec;
  result.stable_sec = std::max(0.0, request.now_sec - result.ready_since_sec);
  result.ready = result.stable_sec + 1e-9 >= request.confirm_sec;
  return result;
}

OvertakeEntryPrearmWindowResolution update_overtake_entry_prearm_window(
  const OvertakeEntryPrearmWindowRequest & request) noexcept
{
  OvertakeEntryPrearmWindowResolution result;
  if (
    !request.monitor_active || !std::isfinite(request.now_sec) ||
    !std::isfinite(request.ego_speed_mps) || request.ego_speed_mps < 0.0 ||
    !std::isfinite(request.maximum_duration_sec) ||
    request.maximum_duration_sec <= 0.0 ||
    !std::isfinite(request.maximum_distance_m) ||
    request.maximum_distance_m <= 0.0 ||
    !std::isfinite(request.maximum_observation_gap_sec) ||
    request.maximum_observation_gap_sec <= 0.0)
  {
    return result;
  }

  const bool prior_window_valid =
    request.same_target && std::isfinite(request.start_sec) &&
    std::isfinite(request.last_update_sec) && request.start_sec <= request.now_sec &&
    request.last_update_sec <= request.now_sec && request.traveled_m >= 0.0 &&
    std::isfinite(request.traveled_m);
  const double observation_dt = prior_window_valid ?
    std::max(0.0, request.now_sec - request.last_update_sec) : 0.0;
  const bool observation_continuous =
    prior_window_valid &&
    observation_dt <= request.maximum_observation_gap_sec + 1e-9;

  result.start_sec = observation_continuous ? request.start_sec : request.now_sec;
  result.last_update_sec = request.now_sec;
  result.traveled_m = observation_continuous ?
    request.traveled_m + request.ego_speed_mps * observation_dt : 0.0;
  result.elapsed_sec = std::max(0.0, request.now_sec - result.start_sec);
  result.timed_out =
    result.elapsed_sec + 1e-9 >= request.maximum_duration_sec ||
    result.traveled_m + 1e-9 >= request.maximum_distance_m;
  result.active = !result.timed_out;
  return result;
}

OvertakeEngagementLeaseResolution resolve_overtake_engagement_lease(
  const OvertakeEngagementLeaseRequest & request) noexcept
{
  OvertakeEngagementLeaseResolution result;
  if (
    !request.enabled || !request.hard_guard_clear || request.explicit_disengage ||
    !std::isfinite(request.now_sec) || !std::isfinite(request.maximum_hold_sec) ||
    request.maximum_hold_sec < 0.0)
  {
    result.clear_target = request.explicit_disengage || !request.hard_guard_clear;
    return result;
  }

  if (request.current_target_relevant) {
    result.active = true;
    result.last_relevant_sec = request.now_sec;
    result.remaining_sec = request.maximum_hold_sec;
    return result;
  }

  if (!request.prior_target_engaged || !std::isfinite(request.last_relevant_sec)) {
    result.clear_target = true;
    return result;
  }
  const double age_sec = request.now_sec - request.last_relevant_sec;
  if (age_sec < 0.0 || age_sec > request.maximum_hold_sec + 1e-9) {
    result.clear_target = true;
    return result;
  }

  result.active = true;
  result.hold_active = true;
  result.last_relevant_sec = request.last_relevant_sec;
  result.remaining_sec = std::max(0.0, request.maximum_hold_sec - age_sec);
  return result;
}

OvertakeEntryPrearmValidationLeaseResolution resolve_overtake_entry_prearm_validation_lease(
  const OvertakeEntryPrearmValidationLeaseRequest & request) noexcept
{
  OvertakeEntryPrearmValidationLeaseResolution result;
  if (
    !request.hard_guard_clear || !std::isfinite(request.now_sec) ||
    !std::isfinite(request.maximum_hold_sec) || request.maximum_hold_sec < 0.0)
  {
    return result;
  }

  if (request.current_mission_validated) {
    result.monitor_active = true;
    result.last_validated_sec = request.now_sec;
    result.remaining_sec = request.maximum_hold_sec;
    return result;
  }

  if (!request.same_target || !std::isfinite(request.last_validated_sec)) {
    return result;
  }
  const double age_sec = request.now_sec - request.last_validated_sec;
  if (age_sec < 0.0 || age_sec > request.maximum_hold_sec + 1e-9) {
    return result;
  }

  result.monitor_active = true;
  result.hold_active = true;
  result.last_validated_sec = request.last_validated_sec;
  result.remaining_sec = std::max(0.0, request.maximum_hold_sec - age_sec);
  return result;
}

bool can_use_overtake_entry_setup_prearm(
  const OvertakeEntrySetupPrearmRequest & request) noexcept
{
  if (
    !std::isfinite(request.front_distance_m) || request.front_distance_m < 0.0 ||
    !std::isfinite(request.minimum_front_distance_m) ||
    request.minimum_front_distance_m < 0.0)
  {
    return false;
  }
  return request.setup_candidate_available &&
         !request.complete_mission_available && request.monitor_active &&
         request.hard_guard_clear && request.front_vehicle_seen &&
         request.front_distance_m + 1e-9 >= request.minimum_front_distance_m;
}

NewOvertakeEntryAdmissionResolution resolve_new_overtake_entry_admission(
  const NewOvertakeEntryAdmissionRequest & request) noexcept
{
  NewOvertakeEntryAdmissionResolution resolution;
  resolution.execution_allowed =
    !request.overtake_requested || request.execution_committed ||
    request.behavior_handoff_active || request.entry_speed_ready ||
    request.immediate_execution_override;
  resolution.prearm_active =
    request.overtake_requested && !resolution.execution_allowed &&
    request.validated_mission_ready;
  return resolution;
}

bool can_override_entry_speed_for_stationary_blocker(
  const StationaryBlockerEntryOverrideRequest & request) noexcept
{
  const bool finite_non_negative_geometry =
    std::isfinite(request.front_speed_mps) && request.front_speed_mps >= 0.0 &&
    std::isfinite(request.maximum_stopped_speed_mps) &&
    request.maximum_stopped_speed_mps >= 0.0 &&
    std::isfinite(request.front_distance_m) && request.front_distance_m >= 0.0 &&
    std::isfinite(request.minimum_entry_distance_m) &&
    request.minimum_entry_distance_m >= 0.0;
  return request.enabled && request.validated_mission_ready &&
         request.hard_guard_clear && request.front_vehicle_seen &&
         request.stopped_evidence_matches_target &&
         request.required_stopped_observation_count > 0 &&
         request.stopped_observation_count >= request.required_stopped_observation_count &&
         finite_non_negative_geometry &&
         request.front_speed_mps <= request.maximum_stopped_speed_mps + 1e-9 &&
         request.front_distance_m + 1e-9 >= request.minimum_entry_distance_m;
}

bool can_use_urgent_entry_for_slow_blocker(
  const SlowBlockerUrgentEntryOverrideRequest & request) noexcept
{
  const bool finite_non_negative_input =
    std::isfinite(request.relative_speed_mps) &&
    std::isfinite(request.minimum_relative_speed_mps) &&
    request.minimum_relative_speed_mps >= 0.0 &&
    std::isfinite(request.stable_sec) && request.stable_sec >= 0.0 &&
    std::isfinite(request.minimum_stable_sec) && request.minimum_stable_sec >= 0.0 &&
    std::isfinite(request.front_speed_mps) && request.front_speed_mps >= 0.0 &&
    std::isfinite(request.maximum_front_speed_mps) &&
    request.maximum_front_speed_mps >= 0.0 &&
    std::isfinite(request.front_distance_m) && request.front_distance_m >= 0.0 &&
    std::isfinite(request.minimum_entry_distance_m) &&
    request.minimum_entry_distance_m >= 0.0 &&
    std::isfinite(request.maximum_entry_distance_m) &&
    request.maximum_entry_distance_m >= request.minimum_entry_distance_m;
  return request.enabled && request.validated_mission_ready &&
         request.hard_guard_clear && request.front_vehicle_seen &&
         finite_non_negative_input &&
         request.relative_speed_mps + 1e-9 >= request.minimum_relative_speed_mps &&
         request.stable_sec + 1e-9 >= request.minimum_stable_sec &&
         request.front_speed_mps <= request.maximum_front_speed_mps + 1e-9 &&
         request.front_distance_m + 1e-9 >= request.minimum_entry_distance_m &&
         request.front_distance_m <= request.maximum_entry_distance_m + 1e-9;
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

SideSelection select_minimum_lateral_motion_side(
  const MinimumLateralMotionSideSelectionRequest & request) noexcept
{
  const auto valid_candidate = [](const MinimumLateralMotionSideCandidate & candidate) {
      return is_configured_side(candidate.side) && candidate.feasible &&
             std::isfinite(candidate.required_shift_m) &&
             candidate.required_shift_m >= 0.0;
    };
  const bool left_feasible =
    request.left.side == PassSide::Left && valid_candidate(request.left);
  const bool right_feasible =
    request.right.side == PassSide::Right && valid_candidate(request.right);

  if (!is_configured_side(request.preferred)) {
    return {PassSide::None, SideSelectionReason::InvalidPreference};
  }
  if (!left_feasible && !right_feasible) {
    return {PassSide::None, SideSelectionReason::NoFeasibleSide};
  }
  if (left_feasible != right_feasible) {
    const PassSide selected = left_feasible ? PassSide::Left : PassSide::Right;
    return {
      selected,
      selected == request.preferred ?
      SideSelectionReason::Preferred : SideSelectionReason::Alternate};
  }

  if (request.left.base_line_clear != request.right.base_line_clear) {
    return {
      request.left.base_line_clear ? PassSide::Left : PassSide::Right,
      SideSelectionReason::HigherQuality};
  }
  const double inner_preference_max_extra_shift_m =
    std::isfinite(request.inner_preference_max_extra_shift_m) ?
    std::max(0.0, request.inner_preference_max_extra_shift_m) : 0.0;
  const double inner_preference_min_corridor_width_m =
    std::isfinite(request.inner_preference_min_corridor_width_m) ?
    std::max(0.0, request.inner_preference_min_corridor_width_m) :
    std::numeric_limits<double>::infinity();
  const double inner_preference_min_open_distance_m =
    std::isfinite(request.inner_preference_min_open_distance_m) ?
    std::max(0.0, request.inner_preference_min_open_distance_m) :
    std::numeric_limits<double>::infinity();
  if (
    is_configured_side(request.inner_side) &&
    inner_preference_max_extra_shift_m > 1e-9)
  {
    const auto & inner = request.inner_side == PassSide::Left ? request.left : request.right;
    const auto & outer = request.inner_side == PassSide::Left ? request.right : request.left;
    const bool inner_space_sufficient =
      std::isfinite(inner.corridor_width_m) && inner.corridor_width_m >= 0.0 &&
      inner.corridor_width_m + 1e-9 >= inner_preference_min_corridor_width_m &&
      std::isfinite(inner.continuous_open_distance_m) &&
      inner.continuous_open_distance_m >= 0.0 &&
      inner.continuous_open_distance_m + 1e-9 >= inner_preference_min_open_distance_m;
    if (
      inner_space_sufficient &&
      inner.required_shift_m <=
      outer.required_shift_m + inner_preference_max_extra_shift_m + 1e-9)
    {
      return {request.inner_side, SideSelectionReason::InnerPreference};
    }
  }
  const double shift_difference =
    request.left.required_shift_m - request.right.required_shift_m;
  if (shift_difference < -1e-9) {
    return {PassSide::Left, SideSelectionReason::HigherQuality};
  }
  if (shift_difference > 1e-9) {
    return {PassSide::Right, SideSelectionReason::HigherQuality};
  }
  return {request.preferred, SideSelectionReason::Preferred};
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

PassManeuverCandidateAssessment assess_pass_maneuver_candidate(
  const PassManeuverCandidateRequest & request) noexcept
{
  PassManeuverCandidateAssessment result;
  result.side = request.side;
  result.mission = request.mission;
  if (!is_configured_side(request.side)) {
    return result;
  }

  result.valid = true;
  result.plan_available = request.mission.has_value();
  result.feasible = result.plan_available && request.gap_available &&
    request.execution_allowed && !request.side_conflict && request.runtime_sweep_clear;
  if (!request.mission.has_value()) {
    return result;
  }

  const auto & mission = request.mission.value();
  double reserve = std::numeric_limits<double>::infinity();
  if (std::isfinite(mission.minimum_path_wall_clearance_m)) {
    reserve = std::min(reserve, mission.minimum_path_wall_clearance_m);
  }
  if (std::isfinite(mission.minimum_path_corridor_width_m)) {
    reserve = std::min(reserve, 0.5 * mission.minimum_path_corridor_width_m);
  }
  if (std::isfinite(mission.minimum_return_wall_clearance_m)) {
    reserve = std::min(reserve, mission.minimum_return_wall_clearance_m);
  }
  result.physical_reserve_m = reserve;
  result.predicted_rear_clear_time_sec = mission.predicted_rear_clear_time_sec;
  result.predicted_minimum_speed_mps = mission.predicted_minimum_ego_speed_mps;
  result.horizon_progress_score = mission.horizon_progress_score;
  return result;
}

OpponentSideManeuverComparison compare_opponent_side_maneuvers(
  const OpponentSideManeuverComparisonRequest & request) noexcept
{
  OpponentSideManeuverComparison result;
  const bool dynamic_settings_valid =
    !request.dynamic_ranking_enabled ||
    (std::isfinite(request.minimum_rear_clear_time_advantage_sec) &&
    request.minimum_rear_clear_time_advantage_sec >= 0.0 &&
    std::isfinite(request.minimum_progress_score_advantage) &&
    request.minimum_progress_score_advantage >= 0.0 &&
    std::isfinite(request.maximum_reserve_regression_m) &&
    request.maximum_reserve_regression_m >= 0.0 &&
    std::isfinite(request.maximum_rear_clear_time_regression_sec) &&
    request.maximum_rear_clear_time_regression_sec >= 0.0 &&
    std::isfinite(request.maximum_minimum_speed_regression_mps) &&
    request.maximum_minimum_speed_regression_mps >= 0.0);
  if (
    !request.current.valid || !request.alternate.valid ||
    !is_configured_side(request.current.side) ||
    !is_configured_side(request.alternate.side) ||
    request.current.side == request.alternate.side ||
    std::isnan(request.current.physical_reserve_m) ||
    std::isnan(request.alternate.physical_reserve_m) ||
    !std::isfinite(request.minimum_reserve_advantage_m) ||
    request.minimum_reserve_advantage_m < 0.0 || !dynamic_settings_valid)
  {
    return result;
  }

  result.valid = true;
  result.current_feasible = request.current.feasible;
  result.alternate_feasible = request.alternate.feasible;
  if (
    std::isinf(request.current.physical_reserve_m) &&
    std::isinf(request.alternate.physical_reserve_m))
  {
    result.physical_reserve_advantage_m = 0.0;
  } else {
    result.physical_reserve_advantage_m =
      request.alternate.physical_reserve_m - request.current.physical_reserve_m;
  }
  if (!result.alternate_feasible) {
    return result;
  }
  if (!result.current_feasible) {
    result.alternate_preferred = true;
    result.preference_reason = OpponentSideManeuverPreferenceReason::CurrentInfeasible;
    return result;
  }

  if (!request.dynamic_ranking_enabled) {
    result.alternate_preferred =
      result.physical_reserve_advantage_m + 1e-9 >=
      request.minimum_reserve_advantage_m;
    if (result.alternate_preferred) {
      result.preference_reason =
        OpponentSideManeuverPreferenceReason::PhysicalReserveAdvantage;
    }
    return result;
  }

  const bool rear_clear_time_available =
    std::isfinite(request.current.predicted_rear_clear_time_sec) &&
    std::isfinite(request.alternate.predicted_rear_clear_time_sec);
  const bool minimum_speed_available =
    std::isfinite(request.current.predicted_minimum_speed_mps) &&
    std::isfinite(request.alternate.predicted_minimum_speed_mps);
  const bool progress_score_available =
    std::isfinite(request.current.horizon_progress_score) &&
    std::isfinite(request.alternate.horizon_progress_score);
  result.dynamic_metrics_compared =
    rear_clear_time_available || minimum_speed_available || progress_score_available;
  if (rear_clear_time_available) {
    result.rear_clear_time_advantage_sec =
      request.current.predicted_rear_clear_time_sec -
      request.alternate.predicted_rear_clear_time_sec;
  }
  if (minimum_speed_available) {
    result.minimum_speed_advantage_mps =
      request.alternate.predicted_minimum_speed_mps -
      request.current.predicted_minimum_speed_mps;
  }
  if (progress_score_available) {
    result.horizon_progress_score_advantage =
      request.alternate.horizon_progress_score -
      request.current.horizon_progress_score;
  }

  const bool reserve_regression_allowed =
    result.physical_reserve_advantage_m + 1e-9 >=
    -request.maximum_reserve_regression_m;
  const bool rear_clear_time_regression_allowed =
    !rear_clear_time_available ||
    result.rear_clear_time_advantage_sec + 1e-9 >=
    -request.maximum_rear_clear_time_regression_sec;
  const bool minimum_speed_regression_allowed =
    !minimum_speed_available ||
    result.minimum_speed_advantage_mps + 1e-9 >=
    -request.maximum_minimum_speed_regression_mps;
  const bool regressions_allowed =
    reserve_regression_allowed && rear_clear_time_regression_allowed &&
    minimum_speed_regression_allowed;
  if (!regressions_allowed) {
    return result;
  }

  const bool rear_clear_time_wins =
    rear_clear_time_available &&
    result.rear_clear_time_advantage_sec + 1e-9 >=
    request.minimum_rear_clear_time_advantage_sec;
  const bool progress_score_wins =
    progress_score_available &&
    result.horizon_progress_score_advantage + 1e-9 >=
    request.minimum_progress_score_advantage;
  const bool physical_reserve_wins =
    result.physical_reserve_advantage_m + 1e-9 >=
    request.minimum_reserve_advantage_m;
  result.alternate_preferred =
    rear_clear_time_wins || progress_score_wins || physical_reserve_wins;
  if (rear_clear_time_wins) {
    result.preference_reason =
      OpponentSideManeuverPreferenceReason::RearClearTimeAdvantage;
  } else if (progress_score_wins) {
    result.preference_reason =
      OpponentSideManeuverPreferenceReason::HorizonProgressAdvantage;
  } else if (physical_reserve_wins) {
    result.preference_reason =
      OpponentSideManeuverPreferenceReason::PhysicalReserveAdvantage;
  }
  return result;
}

const char * to_string(const OpponentSideManeuverPreferenceReason reason) noexcept
{
  switch (reason) {
    case OpponentSideManeuverPreferenceReason::None:
      return "none";
    case OpponentSideManeuverPreferenceReason::CurrentInfeasible:
      return "current infeasible";
    case OpponentSideManeuverPreferenceReason::RearClearTimeAdvantage:
      return "rear-clear time advantage";
    case OpponentSideManeuverPreferenceReason::HorizonProgressAdvantage:
      return "horizon progress advantage";
    case OpponentSideManeuverPreferenceReason::PhysicalReserveAdvantage:
      return "physical reserve advantage";
  }
  return "unknown";
}

SideReplanDebounceResolution update_side_replan_debounce(
  const SideReplanDebounceRequest & request) noexcept
{
  SideReplanDebounceResolution result;
  if (
    !std::isfinite(request.now_sec) ||
    (request.opportunity_active && !is_configured_side(request.candidate_side)) ||
    (request.pending_side != PassSide::None &&
    !is_configured_side(request.pending_side)) ||
    (!std::isfinite(request.pending_since_sec) &&
    !std::isnan(request.pending_since_sec)))
  {
    return result;
  }

  result.valid = true;
  if (!request.opportunity_active) {
    result.changed = request.pending_side != PassSide::None;
    return result;
  }

  result.pending_side = request.candidate_side;
  const bool starts_new_opportunity =
    request.pending_side != request.candidate_side ||
    !std::isfinite(request.pending_since_sec);
  result.pending_since_sec = starts_new_opportunity ?
    request.now_sec : request.pending_since_sec;
  result.stable_sec = std::max(0.0, request.now_sec - result.pending_since_sec);
  result.changed = starts_new_opportunity;
  return result;
}

OpponentSideReplanResolution resolve_opponent_side_replan(
  const OpponentSideReplanRequest & request) noexcept
{
  OpponentSideReplanResolution result;
  if (!request.enabled) {
    result.reason = OpponentSideReplanReason::Disabled;
    return result;
  }

  const bool configured_sides =
    is_configured_side(request.current_side) &&
    is_configured_side(request.alternate_side) &&
    request.current_side != request.alternate_side;
  const bool valid_numeric_input =
    std::isfinite(request.target_front_distance_m) &&
    std::isfinite(request.no_return_front_distance_m) &&
    request.no_return_front_distance_m >= 0.0 &&
    request.replacement_count >= 0 && request.maximum_replacements >= 0 &&
    !std::isnan(request.current_physical_reserve_m) &&
    !std::isnan(request.alternate_physical_reserve_m) &&
    std::isfinite(request.minimum_reserve_advantage_m) &&
    request.minimum_reserve_advantage_m >= 0.0 &&
    std::isfinite(request.candidate_stable_sec) &&
    request.candidate_stable_sec >= 0.0 &&
    std::isfinite(request.required_stable_sec) &&
    request.required_stable_sec >= 0.0 &&
    (!request.maneuver_ranking_checked ||
    !request.alternate_maneuver_preferred ||
    request.maneuver_preference_reason !=
    OpponentSideManeuverPreferenceReason::None);
  if (!configured_sides || !valid_numeric_input) {
    result.reason = OpponentSideReplanReason::InvalidInput;
    return result;
  }
  if (!request.frozen_execution_active) {
    result.action = OpponentSideReplanAction::BlockedByNoReturn;
    result.reason = OpponentSideReplanReason::InactivePhase;
    return result;
  }
  if (!request.target_continuous || request.target_position_jump) {
    result.action = OpponentSideReplanAction::BlockedByNoReturn;
    result.reason = OpponentSideReplanReason::TargetInvalid;
    return result;
  }
  if (request.rear_clear) {
    result.action = OpponentSideReplanAction::BlockedByNoReturn;
    result.reason = OpponentSideReplanReason::RearClear;
    return result;
  }
  if (!request.current_body_footprints_separated) {
    result.action = OpponentSideReplanAction::BlockedByNoReturn;
    result.reason = OpponentSideReplanReason::BodyOverlap;
    return result;
  }
  if (!request.footprint_prediction_valid) {
    result.action = OpponentSideReplanAction::BlockedByNoReturn;
    result.reason = OpponentSideReplanReason::PredictedOverlap;
    return result;
  }
  if (request.target_front_distance_m + 1e-9 < request.no_return_front_distance_m) {
    result.action = OpponentSideReplanAction::BlockedByNoReturn;
    result.reason = OpponentSideReplanReason::TargetTooClose;
    return result;
  }
  if (request.replacement_count >= request.maximum_replacements) {
    result.action = OpponentSideReplanAction::BlockedByNoReturn;
    result.reason = OpponentSideReplanReason::ReplacementLimit;
    return result;
  }

  result.eligible = true;
  if (!request.alternate_plan_feasible) {
    result.action = request.current_plan_feasible ?
      OpponentSideReplanAction::KeepCurrent :
      OpponentSideReplanAction::FallbackSameSide;
    result.reason = OpponentSideReplanReason::AlternateUnavailable;
    return result;
  }

  if (
    std::isinf(request.alternate_physical_reserve_m) &&
    std::isinf(request.current_physical_reserve_m))
  {
    result.physical_reserve_advantage_m = 0.0;
  } else {
    result.physical_reserve_advantage_m =
      request.alternate_physical_reserve_m - request.current_physical_reserve_m;
  }
  // A predicted overlap on the frozen side is a reason to assess and prefer a
  // complete alternate mission before no-return, not a reason to suppress
  // alternate-side evaluation altogether.
  const bool current_infeasible =
    !request.current_plan_feasible ||
    !request.predicted_body_footprint_sweep_separated;
  const bool reserve_advantage =
    result.physical_reserve_advantage_m + 1e-9 >=
    request.minimum_reserve_advantage_m;
  result.replacement_requested = request.maneuver_ranking_checked ?
    request.alternate_maneuver_preferred :
    (current_infeasible || reserve_advantage);
  if (!result.replacement_requested) {
    result.action = OpponentSideReplanAction::KeepCurrent;
    result.reason = OpponentSideReplanReason::CurrentPlanRetained;
    return result;
  }
  if (request.candidate_stable_sec + 1e-9 < request.required_stable_sec) {
    result.action = OpponentSideReplanAction::WaitForStability;
    result.reason = OpponentSideReplanReason::StabilityPending;
    return result;
  }

  result.action = OpponentSideReplanAction::ReplaceWithAlternate;
  if (current_infeasible) {
    result.reason = OpponentSideReplanReason::CurrentPlanInfeasible;
  } else if (
    request.maneuver_ranking_checked &&
    request.maneuver_preference_reason ==
    OpponentSideManeuverPreferenceReason::RearClearTimeAdvantage)
  {
    result.reason = OpponentSideReplanReason::RearClearTimeAdvantage;
  } else if (
    request.maneuver_ranking_checked &&
    request.maneuver_preference_reason ==
    OpponentSideManeuverPreferenceReason::HorizonProgressAdvantage)
  {
    result.reason = OpponentSideReplanReason::HorizonProgressAdvantage;
  } else {
    result.reason = OpponentSideReplanReason::PhysicalReserveAdvantage;
  }
  return result;
}

const char * to_string(const OpponentSideReplanAction action) noexcept
{
  switch (action) {
    case OpponentSideReplanAction::Inactive:
      return "inactive";
    case OpponentSideReplanAction::KeepCurrent:
      return "keep current";
    case OpponentSideReplanAction::WaitForStability:
      return "wait for stability";
    case OpponentSideReplanAction::ReplaceWithAlternate:
      return "replace with alternate";
    case OpponentSideReplanAction::FallbackSameSide:
      return "fallback same side";
    case OpponentSideReplanAction::BlockedByNoReturn:
      return "blocked by no-return";
  }
  return "unknown";
}

const char * to_string(const OpponentSideReplanReason reason) noexcept
{
  switch (reason) {
    case OpponentSideReplanReason::None:
      return "none";
    case OpponentSideReplanReason::Disabled:
      return "disabled";
    case OpponentSideReplanReason::InvalidInput:
      return "invalid input";
    case OpponentSideReplanReason::InactivePhase:
      return "inactive phase";
    case OpponentSideReplanReason::TargetInvalid:
      return "target invalid";
    case OpponentSideReplanReason::BodyOverlap:
      return "body overlap";
    case OpponentSideReplanReason::PredictedOverlap:
      return "predicted overlap";
    case OpponentSideReplanReason::TargetTooClose:
      return "target too close";
    case OpponentSideReplanReason::RearClear:
      return "rear clear";
    case OpponentSideReplanReason::ReplacementLimit:
      return "replacement limit";
    case OpponentSideReplanReason::AlternateUnavailable:
      return "alternate unavailable";
    case OpponentSideReplanReason::CurrentPlanRetained:
      return "current plan retained";
    case OpponentSideReplanReason::StabilityPending:
      return "stability pending";
    case OpponentSideReplanReason::CurrentPlanInfeasible:
      return "current plan infeasible";
    case OpponentSideReplanReason::RearClearTimeAdvantage:
      return "rear-clear time advantage";
    case OpponentSideReplanReason::HorizonProgressAdvantage:
      return "horizon progress advantage";
    case OpponentSideReplanReason::PhysicalReserveAdvantage:
      return "physical reserve advantage";
  }
  return "unknown";
}

LastFeasibleManeuverResolution resolve_last_feasible_maneuver(
  const LastFeasibleManeuverRequest & request) noexcept
{
  LastFeasibleManeuverResolution resolution;
  if (!request.enabled || !request.soft_failure) {
    return resolution;
  }

  const bool current_age_valid =
    !request.current_candidate_available ||
    (std::isfinite(request.current_candidate_age_sec) &&
    request.current_candidate_age_sec >= 0.0);
  const bool alternate_age_valid =
    !request.alternate_candidate_available ||
    (std::isfinite(request.alternate_candidate_age_sec) &&
    request.alternate_candidate_age_sec >= 0.0);
  if (
    !std::isfinite(request.maximum_candidate_age_sec) ||
    request.maximum_candidate_age_sec < 0.0 ||
    !current_age_valid || !alternate_age_valid)
  {
    resolution.action = LastFeasibleManeuverAction::Unavailable;
    return resolution;
  }

  if (
    request.hard_fault || !request.target_continuous ||
    !request.current_body_footprints_separated)
  {
    resolution.action = LastFeasibleManeuverAction::BlockedByHardFault;
    return resolution;
  }

  const bool current_fresh =
    request.current_candidate_available &&
    request.current_candidate_motion_fresh &&
    request.current_candidate_age_sec <= request.maximum_candidate_age_sec;
  const bool alternate_fresh =
    request.alternate_candidate_available &&
    (request.alternate_candidate_stable ||
    request.allow_unstable_alternate_reselection) &&
    request.alternate_candidate_motion_fresh &&
    request.alternate_candidate_age_sec <= request.maximum_candidate_age_sec;

  if (
    alternate_fresh &&
    (request.before_no_return || request.tactical_no_return_rearmed))
  {
    resolution.action = LastFeasibleManeuverAction::ReuseAlternate;
    resolution.replacement_requested = true;
    resolution.alternate_selected = true;
    resolution.selected_candidate_age_sec = request.alternate_candidate_age_sec;
    return resolution;
  }
  if (current_fresh) {
    resolution.action = LastFeasibleManeuverAction::ReuseCurrent;
    resolution.replacement_requested = true;
    resolution.selected_candidate_age_sec = request.current_candidate_age_sec;
    return resolution;
  }
  if (
    alternate_fresh && !request.before_no_return &&
    !request.tactical_no_return_rearmed)
  {
    resolution.action = LastFeasibleManeuverAction::BlockedByNoReturn;
    return resolution;
  }
  if (
    request.current_candidate_available ||
    (request.alternate_candidate_available &&
    (request.alternate_candidate_stable ||
    request.allow_unstable_alternate_reselection)))
  {
    resolution.action = LastFeasibleManeuverAction::Stale;
    return resolution;
  }
  resolution.action = LastFeasibleManeuverAction::Unavailable;
  return resolution;
}

LastFeasibleCacheUpdateResolution resolve_last_feasible_cache_update(
  const LastFeasibleCacheUpdateRequest & request) noexcept
{
  LastFeasibleCacheUpdateResolution resolution;
  resolution.clear_existing = !request.identity_matches || request.hard_invalid;
  resolution.store_candidate = request.candidate_feasible && !request.hard_invalid;
  resolution.retain_existing =
    request.identity_matches && !request.hard_invalid && !request.candidate_feasible;
  return resolution;
}

const char * to_string(const LastFeasibleManeuverAction action) noexcept
{
  switch (action) {
    case LastFeasibleManeuverAction::Inactive:
      return "inactive";
    case LastFeasibleManeuverAction::ReuseCurrent:
      return "reuse current";
    case LastFeasibleManeuverAction::ReuseAlternate:
      return "reuse alternate";
    case LastFeasibleManeuverAction::Unavailable:
      return "unavailable";
    case LastFeasibleManeuverAction::Stale:
      return "stale";
    case LastFeasibleManeuverAction::BlockedByHardFault:
      return "blocked by hard fault";
    case LastFeasibleManeuverAction::BlockedByNoReturn:
      return "blocked by no-return";
  }
  return "unknown";
}

bool should_observe_locked_target_geometry(
  const LockedTargetGeometryObservationRequest & request) noexcept
{
  const bool execution_phase = request.shiftout_phase || request.pass_phase;
  const bool paused_frozen_mission =
    request.follow_prepare_phase && request.mission_path_frozen;
  return
    (execution_phase || paused_frozen_mission) &&
    request.target_id_available && request.vehicle_matches_target;
}

bool can_enter_dynamic_mission_wait(
  const DynamicMissionWaitAdmissionRequest & request) noexcept
{
  const bool replacement_window_available =
    request.same_side_replacement_allowed ||
    (request.before_no_return && request.replacement_count_available);
  return
    request.enabled && request.active_execution_phase &&
    request.mission_path_frozen && request.target_id_available &&
    request.target_continuous && request.current_body_footprints_separated &&
    request.footprint_prediction_valid && replacement_window_available &&
    !request.hard_fault &&
    !request.rear_clear_confirmed;
}

DynamicMissionWaitResolution resolve_dynamic_mission_wait(
  const DynamicMissionWaitRequest & request) noexcept
{
  DynamicMissionWaitResolution resolution;
  if (!request.enabled || !request.wait_active) {
    resolution.reason = DynamicMissionWaitReason::Disabled;
    return resolution;
  }
  if (request.hard_fault) {
    resolution.action = DynamicMissionWaitAction::Recovery;
    resolution.reason = DynamicMissionWaitReason::HardFault;
    return resolution;
  }
  if (!request.target_continuous || request.target_position_jump) {
    resolution.action = DynamicMissionWaitAction::Recovery;
    resolution.reason = DynamicMissionWaitReason::TargetInvalid;
    return resolution;
  }
  if (!request.current_body_footprints_separated) {
    resolution.action = DynamicMissionWaitAction::Recovery;
    resolution.reason = DynamicMissionWaitReason::BodyOverlap;
    return resolution;
  }
  if (request.rear_clear_confirmed) {
    resolution.action = DynamicMissionWaitAction::Return;
    resolution.reason = DynamicMissionWaitReason::RearClear;
    return resolution;
  }
  if (
    request.alternate_replacement_allowed &&
    request.alternate_replacement_ready)
  {
    resolution.action = DynamicMissionWaitAction::ReplaceWithAlternate;
    resolution.reason = DynamicMissionWaitReason::AlternatePlanReady;
    return resolution;
  }
  if (request.current_replacement_ready) {
    resolution.action = DynamicMissionWaitAction::ReplaceWithCurrent;
    resolution.reason = DynamicMissionWaitReason::CurrentPlanRecovered;
    return resolution;
  }
  if (!request.assessment_completed) {
    resolution.action = DynamicMissionWaitAction::Hold;
    resolution.reason = DynamicMissionWaitReason::WaitingForAssessment;
    return resolution;
  }
  if (request.current_mission_invalidated) {
    // Invalidation is the expected entry state for a dynamic Mission wait: it
    // prevents the failed generation from being resumed. Keep waiting for a
    // fully preflighted replacement until the controller-owned time/distance
    // lease expires; hard faults above still fail closed immediately.
    resolution.action = DynamicMissionWaitAction::Hold;
    resolution.reason = DynamicMissionWaitReason::CurrentMissionInvalidated;
    return resolution;
  }
  if (request.current_plan_feasible) {
    resolution.action = DynamicMissionWaitAction::ResumeCurrent;
    resolution.reason = DynamicMissionWaitReason::CurrentPlanRecovered;
    return resolution;
  }
  resolution.action = DynamicMissionWaitAction::Hold;
  resolution.reason = DynamicMissionWaitReason::BothPlansUnavailable;
  return resolution;
}

const char * to_string(const DynamicMissionWaitAction action) noexcept
{
  switch (action) {
    case DynamicMissionWaitAction::Inactive:
      return "inactive";
    case DynamicMissionWaitAction::Hold:
      return "hold";
    case DynamicMissionWaitAction::ResumeCurrent:
      return "resume current";
    case DynamicMissionWaitAction::ReplaceWithCurrent:
      return "replace with current";
    case DynamicMissionWaitAction::ReplaceWithAlternate:
      return "replace with alternate";
    case DynamicMissionWaitAction::Return:
      return "return";
    case DynamicMissionWaitAction::Recovery:
      return "recovery";
  }
  return "unknown";
}

const char * to_string(const DynamicMissionWaitReason reason) noexcept
{
  switch (reason) {
    case DynamicMissionWaitReason::None:
      return "none";
    case DynamicMissionWaitReason::Disabled:
      return "disabled";
    case DynamicMissionWaitReason::WaitingForAssessment:
      return "waiting for assessment";
    case DynamicMissionWaitReason::BothPlansUnavailable:
      return "both plans unavailable";
    case DynamicMissionWaitReason::CurrentPlanRecovered:
      return "current plan recovered";
    case DynamicMissionWaitReason::CurrentMissionInvalidated:
      return "current Mission generation invalidated";
    case DynamicMissionWaitReason::AlternatePlanReady:
      return "alternate plan ready";
    case DynamicMissionWaitReason::RearClear:
      return "rear clear";
    case DynamicMissionWaitReason::TargetInvalid:
      return "target invalid";
    case DynamicMissionWaitReason::BodyOverlap:
      return "body overlap";
    case DynamicMissionWaitReason::HardFault:
      return "hard fault";
  }
  return "unknown";
}

OvertakeMissionOwnershipResolution resolve_overtake_mission_ownership(
  const OvertakeMissionOwnershipRequest & request) noexcept
{
  OvertakeMissionOwnershipResolution resolution;
  resolution.committed_execution_active = request.shiftout_phase || request.pass_phase;
  resolution.committed_pass_active = request.pass_phase;
  resolution.paused_mission_active = request.follow_prepare_phase;
  resolution.mission_active =
    resolution.committed_execution_active || resolution.paused_mission_active ||
    request.return_phase || request.recovery_phase;
  resolution.behavior_continuation_assessment_active =
    resolution.committed_execution_active || resolution.paused_mission_active ||
    request.previous_behavior_overtake;
  resolution.behavior_entry_assessment_active =
    !resolution.behavior_continuation_assessment_active;
  resolution.generic_follow_owns_locked_target_speed =
    should_apply_generic_follow_cap(
    GenericFollowCapOwnershipRequest{
      request.shiftout_phase, request.pass_phase, request.front_matches_locked_target});
  resolution.overtake_line_owns_locked_target_speed =
    !resolution.generic_follow_owns_locked_target_speed;
  return resolution;
}

CommittedBehaviorOwnershipGuardResolution resolve_committed_behavior_ownership_guards(
  const CommittedBehaviorOwnershipGuardRequest & request) noexcept
{
  CommittedBehaviorOwnershipGuardResolution resolution;
  resolution.target_identity_available =
    request.locked_target_seen || request.target_identity_continuous;
  resolution.hard_fault_present =
    request.locked_target_position_jump ||
    request.locked_target_course_progress_rejected ||
    request.locked_target_pass_side_intrusion ||
    request.explicit_forbidden_waypoint ||
    request.emergency_front_risk ||
    request.solver_recovery_requested;
  resolution.ownership_allowed =
    resolution.target_identity_available && !resolution.hard_fault_present;
  return resolution;
}

CommittedPassGeometryOwnershipResolution resolve_committed_pass_geometry_ownership(
  const CommittedPassGeometryOwnershipRequest & request) noexcept
{
  CommittedPassGeometryOwnershipResolution resolution;
  resolution.pass_release_latched =
    request.lateral_exclusion_latched ||
    request.minimum_motion_front_cap_release_latched;
  resolution.body_clear_handoff_owns_pass =
    request.body_clear_handoff_active &&
    request.current_body_footprints_separated &&
    !request.current_body_footprint_overlap_confirmed;
  resolution.current_overlap_grace_active =
    request.committed_pass_attack_mode_enabled &&
    request.minimum_motion_front_cap_release_latched &&
    !request.current_body_footprints_separated &&
    !request.current_body_footprint_overlap_confirmed;
  resolution.recoverable_side_contact_owns_pass =
    request.recoverable_side_contact_active;
  resolution.pass_authority_available =
    resolution.pass_release_latched || resolution.body_clear_handoff_owns_pass ||
    resolution.recoverable_side_contact_owns_pass;
  resolution.current_geometry_acceptable =
    request.current_body_footprints_separated ||
    resolution.current_overlap_grace_active ||
    resolution.recoverable_side_contact_owns_pass;
  resolution.ownership_allowed =
    resolution.pass_authority_available && resolution.current_geometry_acceptable;
  return resolution;
}

bool can_preserve_committed_pass_behavior(
  const CommittedPassBehaviorOwnershipRequest & request) noexcept
{
  const auto common_guards = resolve_committed_behavior_ownership_guards(
    CommittedBehaviorOwnershipGuardRequest{
      request.locked_target_seen,
      request.target_identity_continuous,
      request.locked_target_position_jump,
      request.locked_target_course_progress_rejected,
      request.locked_target_pass_side_intrusion,
      request.explicit_forbidden_waypoint,
      request.emergency_front_risk,
      request.solver_recovery_requested});
  const auto geometry_ownership = resolve_committed_pass_geometry_ownership(
    CommittedPassGeometryOwnershipRequest{
      request.lateral_exclusion_latched,
      request.minimum_motion_front_cap_release_latched,
      request.current_body_footprints_separated,
      request.current_body_footprint_overlap_confirmed,
      request.committed_pass_attack_mode_enabled,
      request.body_clear_handoff_active,
      request.recoverable_side_contact_active});
  return request.committed_pass_active &&
         request.validated_fixed_line &&
         request.mission_side_valid &&
         common_guards.ownership_allowed &&
         geometry_ownership.ownership_allowed;
}

bool can_preserve_committed_shiftout_behavior(
  const CommittedShiftOutBehaviorOwnershipRequest & request) noexcept
{
  const auto common_guards = resolve_committed_behavior_ownership_guards(
    CommittedBehaviorOwnershipGuardRequest{
      request.locked_target_seen,
      request.target_identity_continuous,
      request.locked_target_position_jump,
      request.locked_target_course_progress_rejected,
      request.locked_target_pass_side_intrusion,
      request.explicit_forbidden_waypoint,
      request.emergency_front_risk,
      request.solver_recovery_requested});
  return request.committed_shiftout_active &&
         request.validated_fixed_line &&
         request.mission_side_valid &&
         request.body_clear_deadline_checked &&
         request.body_clear_handoff_active &&
         common_guards.ownership_allowed;
}

BodyClearExecutionHandoffResolution resolve_body_clear_execution_handoff(
  const BodyClearExecutionHandoffRequest & request) noexcept
{
  BodyClearExecutionHandoffResolution resolution;
  if (!request.committed_execution_active) {
    resolution.valid = true;
    resolution.release_reason =
      BodyClearExecutionHandoffReleaseReason::InactiveExecution;
    return resolution;
  }
  if (
    !request.body_clear_deadline_checked ||
    !request.body_clear_deadline_feasible ||
    !std::isfinite(request.now_sec) ||
    !std::isfinite(request.hard_deadline_sec) ||
    !std::isfinite(request.execution_margin_sec) ||
    request.execution_margin_sec < 0.0)
  {
    resolution.expired =
      request.body_clear_deadline_checked &&
      request.body_clear_deadline_feasible;
    resolution.release_reason =
      BodyClearExecutionHandoffReleaseReason::InvalidDeadline;
    return resolution;
  }

  resolution.valid = true;
  resolution.satisfied = request.current_body_footprints_separated;
  resolution.effective_deadline_sec = request.hard_deadline_sec;
  if (
    std::isfinite(request.live_hard_gap_ttc_sec) &&
    request.live_hard_gap_ttc_sec >= 0.0)
  {
    const double live_deadline_sec = request.now_sec + std::max(
      0.0, request.live_hard_gap_ttc_sec - request.execution_margin_sec);
    if (live_deadline_sec < resolution.effective_deadline_sec) {
      resolution.effective_deadline_sec = live_deadline_sec;
      resolution.live_deadline_contracted = true;
    }
  }
  resolution.remaining_sec = std::max(
    0.0, resolution.effective_deadline_sec - request.now_sec);
  resolution.expired = request.now_sec > resolution.effective_deadline_sec;
  if (request.current_body_footprint_overlap_confirmed) {
    resolution.release_reason =
      BodyClearExecutionHandoffReleaseReason::CurrentOverlap;
  } else if (request.ordinary_pass_ownership_latched) {
    resolution.release_reason =
      BodyClearExecutionHandoffReleaseReason::NormalPassOwnership;
  } else if (resolution.expired) {
    resolution.release_reason = BodyClearExecutionHandoffReleaseReason::Expired;
  } else {
    resolution.active = true;
  }
  return resolution;
}

const char * to_string(const BodyClearExecutionHandoffReleaseReason reason) noexcept
{
  switch (reason) {
    case BodyClearExecutionHandoffReleaseReason::None:
      return "none";
    case BodyClearExecutionHandoffReleaseReason::InactiveExecution:
      return "inactive_execution";
    case BodyClearExecutionHandoffReleaseReason::InvalidDeadline:
      return "invalid_deadline";
    case BodyClearExecutionHandoffReleaseReason::NormalPassOwnership:
      return "normal_latch";
    case BodyClearExecutionHandoffReleaseReason::CurrentOverlap:
      return "current_overlap";
    case BodyClearExecutionHandoffReleaseReason::Expired:
      return "expiry";
  }
  return "unknown";
}

BodyClearHandoffSpeedReferenceResolution resolve_body_clear_handoff_speed_reference(
  const BodyClearHandoffSpeedReferenceRequest & request) noexcept
{
  BodyClearHandoffSpeedReferenceResolution resolution;
  if (
    !std::isfinite(request.current_speed_mps) || request.current_speed_mps < 0.0 ||
    std::isnan(request.maximum_speed_mps) || request.maximum_speed_mps < 0.0 ||
    !std::isfinite(request.allowed_closing_speed_mps) ||
    request.allowed_closing_speed_mps < 0.0)
  {
    return resolution;
  }

  resolution.valid = true;
  const bool prediction_clear =
    request.footprint_prediction_valid &&
    request.predicted_body_footprint_sweep_separated;
  resolution.hold_active =
    request.handoff_active && request.current_body_footprints_separated &&
    !prediction_clear;
  if (!resolution.hold_active) {
    return resolution;
  }

  double reference_mps = request.current_speed_mps;
  if (std::isfinite(request.target_speed_mps) && request.target_speed_mps >= 0.0) {
    reference_mps = std::min(
      reference_mps,
      request.target_speed_mps + request.allowed_closing_speed_mps);
  }
  if (std::isfinite(request.maximum_speed_mps)) {
    reference_mps = std::min(reference_mps, request.maximum_speed_mps);
  }
  resolution.target_velocity_reference_mps = std::max(0.0, reference_mps);
  return resolution;
}

OvertakeExecutionSideResolution resolve_overtake_execution_side(
  const OvertakeExecutionSideRequest & request) noexcept
{
  if (request.resuming_paused_mission && request.mission_side_sign != 0) {
    return {request.mission_side_sign, OvertakeExecutionSideSource::MissionLock};
  }
  if (request.resuming_paused_mission && request.behavior_side_sign != 0) {
    return {request.behavior_side_sign, OvertakeExecutionSideSource::BehaviorRevalidation};
  }
  if (request.mission_side_sign != 0) {
    return {request.mission_side_sign, OvertakeExecutionSideSource::MissionLock};
  }
  if (request.behavior_side_sign != 0) {
    return {request.behavior_side_sign, OvertakeExecutionSideSource::BehaviorSelection};
  }
  return {};
}

double clamp_paused_resume_goal_outward(
  const int mission_side_sign, const double current_lateral_m,
  const double candidate_goal_lateral_m) noexcept
{
  if (!std::isfinite(current_lateral_m)) {
    return candidate_goal_lateral_m;
  }
  if (
    !is_configured_side(static_cast<PassSide>(mission_side_sign)) ||
    !std::isfinite(candidate_goal_lateral_m))
  {
    return current_lateral_m;
  }
  return mission_side_sign > 0 ?
         std::max(current_lateral_m, candidate_goal_lateral_m) :
         std::min(current_lateral_m, candidate_goal_lateral_m);
}

bool can_resume_paused_pass_directly(
  const PausedPassDirectResumeRequest & request) noexcept
{
  if (
    !request.resuming_paused_mission ||
    !is_configured_side(static_cast<PassSide>(request.mission_side_sign)) ||
    request.behavior_side_sign != request.mission_side_sign ||
    !request.execution_corridor_valid || !request.target_seen ||
    request.target_position_jump || !request.target_lateral_prediction_valid ||
    !std::isfinite(request.target_relative_lateral_m) ||
    !std::isfinite(request.target_predicted_relative_lateral_m) ||
    !std::isfinite(request.required_lateral_clearance_m) ||
    request.required_lateral_clearance_m < 0.0 ||
    !std::isfinite(request.current_lateral_m) ||
    !std::isfinite(request.goal_lateral_m))
  {
    return false;
  }

  const double side = static_cast<double>(request.mission_side_sign);
  const double goal_shift_m = request.goal_lateral_m - request.current_lateral_m;
  const double target_goal_relative_lateral_m =
    request.target_relative_lateral_m - goal_shift_m;
  const double target_predicted_goal_relative_lateral_m =
    request.target_predicted_relative_lateral_m - goal_shift_m;
  const bool current_on_committed_side = side * request.current_lateral_m >= -1e-9;
  const bool goal_on_committed_side = side * request.goal_lateral_m >= -1e-9;
  const bool goal_does_not_retreat_inward = side * goal_shift_m >= -1e-9;
  // A positive mission side means ego passes to the left, so the target must
  // be laterally to the right of ego (negative relative lateral), and vice
  // versa. Check current ego pose, the proposed goal, and the predicted target
  // at that goal. Absolute clearance alone could resume Pass with ego still on
  // the target-facing side of the committed corridor.
  const auto lateral_clear_on_committed_side = [&](const double relative_lateral_m) {
      return side * relative_lateral_m <=
             -request.required_lateral_clearance_m + 1e-9;
    };
  const bool current_lateral_clear =
    lateral_clear_on_committed_side(request.target_relative_lateral_m);
  const bool goal_lateral_clear =
    lateral_clear_on_committed_side(target_goal_relative_lateral_m);
  const bool predicted_goal_lateral_clear =
    lateral_clear_on_committed_side(target_predicted_goal_relative_lateral_m);
  return current_on_committed_side && goal_on_committed_side &&
         goal_does_not_retreat_inward && current_lateral_clear &&
         goal_lateral_clear && predicted_goal_lateral_clear;
}

PausedExecutionResumeAction resolve_paused_execution_resume(
  const PausedExecutionResumeRequest & request) noexcept
{
  const bool resumable_origin =
    request.origin == PausedExecutionOrigin::ShiftOut ||
    request.origin == PausedExecutionOrigin::Pass;
  const bool body_clear_execution_ready =
    request.direct_pass_lateral_clear ||
    (request.body_clear_deadline_checked && request.body_clear_deadline_feasible);
  if (
    !request.safety_brake_pause || !resumable_origin ||
    request.dynamic_mission_wait_active || !request.validated_frozen_path ||
    !request.mission_side_valid || !body_clear_execution_ready || !request.target_seen ||
    request.target_position_jump || request.target_course_progress_discontinuity ||
    request.target_pass_side_intrusion || request.forbidden_waypoint ||
    request.emergency_front_risk || request.solver_recovery_requested ||
    request.mission_invalidated || request.physical_path_hard_fault)
  {
    return PausedExecutionResumeAction::Hold;
  }
  return request.direct_pass_lateral_clear ?
         PausedExecutionResumeAction::ResumePass :
         PausedExecutionResumeAction::ResumeShiftOut;
}

const char * to_string(const PausedExecutionResumeAction action) noexcept
{
  switch (action) {
    case PausedExecutionResumeAction::Hold:
      return "Hold";
    case PausedExecutionResumeAction::ResumeShiftOut:
      return "ResumeShiftOut";
    case PausedExecutionResumeAction::ResumePass:
      return "ResumePass";
  }
  return "Unknown";
}

const char * to_string(const OvertakeLineTransitionAction action) noexcept
{
  switch (action) {
    case OvertakeLineTransitionAction::None:
      return "None";
    case OvertakeLineTransitionAction::RecoverPhysicalWallContact:
      return "RecoverPhysicalWallContact";
    case OvertakeLineTransitionAction::RejectEntryWallMargin:
      return "RejectEntryWallMargin";
    case OvertakeLineTransitionAction::ResumePassForReturnCorridorBlocker:
      return "ResumePassForReturnCorridorBlocker";
    case OvertakeLineTransitionAction::HoldPassForRearClearBeforeWallMarginRecovery:
      return "HoldPassForRearClearBeforeWallMarginRecovery";
    case OvertakeLineTransitionAction::ReturnBeforeWallMarginRecovery:
      return "ReturnBeforeWallMarginRecovery";
    case OvertakeLineTransitionAction::HoldCompletedPassForReturnCorridor:
      return "HoldCompletedPassForReturnCorridor";
    case OvertakeLineTransitionAction::RecoverWallMargin:
      return "RecoverWallMargin";
    case OvertakeLineTransitionAction::ReplanEarlyShiftOutSide:
      return "ReplanEarlyShiftOutSide";
    case OvertakeLineTransitionAction::RecoverOccupiedPassSide:
      return "RecoverOccupiedPassSide";
    case OvertakeLineTransitionAction::ReturnRearClear:
      return "ReturnRearClear";
    case OvertakeLineTransitionAction::RecoverLongitudinalProgress:
      return "RecoverLongitudinalProgress";
  }
  return "Unknown";
}

OvertakeLineTransitionAction resolve_overtake_line_transition(
  const OvertakeLineTransitionRequest & request) noexcept
{
  if (request.actual_wall_physical_contact) {
    return OvertakeLineTransitionAction::RecoverPhysicalWallContact;
  }
  if (
    request.actual_wall_margin_blocked &&
    request.starting_execution_phase &&
    !request.active_execution_phase)
  {
    return OvertakeLineTransitionAction::RejectEntryWallMargin;
  }
  if (request.cancel_early_return_for_corridor_blocker) {
    return OvertakeLineTransitionAction::ResumePassForReturnCorridorBlocker;
  }
  if (
    request.actual_wall_margin_blocked &&
    request.completed_pass_ready_to_return_before_margin_recovery &&
    !request.actual_wall_sample_unavailable)
  {
    return request.rear_clear_confirmed ?
      OvertakeLineTransitionAction::ReturnBeforeWallMarginRecovery :
      OvertakeLineTransitionAction::HoldPassForRearClearBeforeWallMarginRecovery;
  }
  if (
    request.actual_wall_margin_blocked &&
    request.completed_pass_waiting_for_return_corridor &&
    !request.actual_wall_sample_unavailable)
  {
    return OvertakeLineTransitionAction::HoldCompletedPassForReturnCorridor;
  }
  if (request.actual_wall_margin_blocked) {
    return OvertakeLineTransitionAction::RecoverWallMargin;
  }
  if (
    request.side_replan_ready &&
    request.shiftout_phase &&
    request.behavior_overtake &&
    request.side_replan_candidate_sign != 0 &&
    request.side_replan_candidate_sign != request.mission_side_sign)
  {
    return OvertakeLineTransitionAction::ReplanEarlyShiftOutSide;
  }
  if (request.side_replan_abort && request.shiftout_phase) {
    return OvertakeLineTransitionAction::RecoverOccupiedPassSide;
  }
  if (
    request.pass_phase &&
    request.rear_clear_confirmed &&
    !request.return_corridor_blocked)
  {
    return OvertakeLineTransitionAction::ReturnRearClear;
  }
  if (request.pass_progress_watchdog_timed_out) {
    return OvertakeLineTransitionAction::RecoverLongitudinalProgress;
  }
  return OvertakeLineTransitionAction::None;
}

bool should_log_overtake_line_transition_action(
  const OvertakeLineTransitionAction action,
  const OvertakeLineTransitionAction previous_action) noexcept
{
  return
    action != OvertakeLineTransitionAction::None &&
    action != previous_action;
}

bool is_v2x_behavior_session_active(
  const bool state_tracking_enabled, const bool race_started,
  const bool start_grid_ready_rollout) noexcept
{
  return !state_tracking_enabled || race_started || start_grid_ready_rollout;
}

bool can_start_low_speed_bypass(const LowSpeedBypassCandidateRequest & request) noexcept
{
  const bool committed_handoff =
    request.committed_overtake_execution_active &&
    request.committed_overtake_handoff_safe && !request.continuing;
  if (
    !request.enabled || !request.candidate_vehicle_present || request.cooldown_active ||
    request.start_grid_stop_suppressed ||
    (request.committed_overtake_execution_active && !request.continuing &&
    !request.committed_overtake_handoff_safe) ||
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
  // A safe same-target handoff already owns a validated lateral mission. It
  // must not lose the stopped-vehicle path merely because confirmation became
  // complete after the ordinary 3 m preparation boundary. A new bypass still
  // uses the configured minimum distance.
  const double minimum_entry_distance_m =
    committed_handoff ? 0.0 : request.minimum_prepare_distance_m;
  return curve_policy_allows &&
         request.forward_distance_m >= minimum_entry_distance_m &&
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
    request.low_speed_behavior_active || request.low_speed_direct_control_active)
  {
    return true;
  }
  if (request.committed_pass_mission_active) {
    return false;
  }
  if (request.low_speed_candidate) {
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

bool should_try_alternate_low_speed_pass_side(
  const bool automatic_side_selection, const bool primary_side_feasible,
  const int primary_side_sign) noexcept
{
  return automatic_side_selection && !primary_side_feasible && primary_side_sign != 0;
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

LowSpeedDirectControlPhase resolve_low_speed_direct_control_entry_phase(
  const bool /*pass_corridor_entered*/) noexcept
{
  return LowSpeedDirectControlPhase::Shift;
}

bool can_enter_low_speed_direct_pass(
  const LowSpeedDirectPassAdmissionRequest & request) noexcept
{
  return request.phase == LowSpeedDirectControlPhase::Shift &&
         request.pass_corridor_entered && request.pose_settled &&
         request.target_seen && !request.target_position_jump &&
         request.current_body_footprints_separated &&
         request.footprint_prediction_valid &&
         request.predicted_body_footprint_sweep_separated;
}

bool can_update_low_speed_direct_pass_side(
  const LowSpeedDirectControlPhase phase, const int current_side_sign,
  const int candidate_side_sign) noexcept
{
  if (
    (candidate_side_sign != -1 && candidate_side_sign != 1) ||
    (current_side_sign != -1 && current_side_sign != 0 && current_side_sign != 1))
  {
    return false;
  }
  return current_side_sign == 0 || candidate_side_sign == current_side_sign ||
         phase == LowSpeedDirectControlPhase::Shift;
}

const char * to_string(const LowSpeedRetainedPassRejectReason reason) noexcept
{
  switch (reason) {
    case LowSpeedRetainedPassRejectReason::None:
      return "none";
    case LowSpeedRetainedPassRejectReason::StaticPathInfeasible:
      return "static path infeasible";
    case LowSpeedRetainedPassRejectReason::TargetIdentityUnavailable:
      return "target identity unavailable";
    case LowSpeedRetainedPassRejectReason::TargetNotSeen:
      return "locked target not seen";
    case LowSpeedRetainedPassRejectReason::TargetPositionJump:
      return "locked target position jump";
    case LowSpeedRetainedPassRejectReason::CurrentBodyOverlap:
      return "current body footprint overlap";
    case LowSpeedRetainedPassRejectReason::PredictionUnavailable:
      return "target footprint prediction unavailable";
    case LowSpeedRetainedPassRejectReason::PredictedFootprintOverlap:
      return "predicted body footprint sweep overlap";
    case LowSpeedRetainedPassRejectReason::SideOrderingConflict:
      return "locked target crossed committed pass side";
    case LowSpeedRetainedPassRejectReason::InvalidGeometry:
      return "invalid target geometry";
  }
  return "unknown";
}

LowSpeedRetainedPassValidationResult resolve_low_speed_retained_pass_validation(
  const LowSpeedRetainedPassValidationRequest & request) noexcept
{
  const auto reject = [](const LowSpeedRetainedPassRejectReason reason) {
      return LowSpeedRetainedPassValidationResult{false, reason};
    };
  if (!request.static_path_feasible) {
    return reject(LowSpeedRetainedPassRejectReason::StaticPathInfeasible);
  }
  if (!request.target_identity_available) {
    return reject(LowSpeedRetainedPassRejectReason::TargetIdentityUnavailable);
  }
  if (!request.target_seen) {
    return reject(LowSpeedRetainedPassRejectReason::TargetNotSeen);
  }
  if (request.target_position_jump) {
    return reject(LowSpeedRetainedPassRejectReason::TargetPositionJump);
  }
  if (!request.current_body_footprints_separated) {
    return reject(LowSpeedRetainedPassRejectReason::CurrentBodyOverlap);
  }
  if (!request.footprint_prediction_valid) {
    return reject(LowSpeedRetainedPassRejectReason::PredictionUnavailable);
  }
  if (!request.predicted_body_footprint_sweep_separated) {
    return reject(LowSpeedRetainedPassRejectReason::PredictedFootprintOverlap);
  }
  if (
    (request.pass_side_sign != -1 && request.pass_side_sign != 1) ||
    !std::isfinite(request.target_relative_lateral_m) ||
    !std::isfinite(request.predicted_target_relative_lateral_m) ||
    !std::isfinite(request.ordering_margin_m) || request.ordering_margin_m < 0.0)
  {
    return reject(LowSpeedRetainedPassRejectReason::InvalidGeometry);
  }
  const double side = static_cast<double>(request.pass_side_sign);
  if (
    side * request.target_relative_lateral_m > request.ordering_margin_m + 1e-9 ||
    side * request.predicted_target_relative_lateral_m >
    request.ordering_margin_m + 1e-9)
  {
    return reject(LowSpeedRetainedPassRejectReason::SideOrderingConflict);
  }
  return {true, LowSpeedRetainedPassRejectReason::None};
}

bool should_stop_low_speed_direct_control_for_corridor(
  const LowSpeedDirectCorridorStopRequest & request) noexcept
{
  if (!request.direct_control_active || request.rejoin_active) {
    return false;
  }
  if (request.local_path_active) {
    return !request.local_path_feasible;
  }
  const bool has_relevant_vehicle =
    request.has_front_vehicle || request.has_side_vehicle ||
    request.has_clearance_vehicle;
  if (!has_relevant_vehicle) {
    // The clear-hold and Rejoin state machine owns this transition. A brief
    // observation gap must not turn into a hard stop.
    return false;
  }
  const bool completing_validated_pass =
    request.phase == LowSpeedDirectControlPhase::Pass &&
    !request.has_front_vehicle &&
    (request.has_side_vehicle || request.has_clearance_vehicle) &&
    request.retained_pass_path_feasible;
  return !completing_validated_pass;
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

LowSpeedDirectControlEntryFeasibility
resolve_low_speed_direct_control_entry_feasibility(
  const LowSpeedDirectControlEntryFeasibilityRequest & request) noexcept
{
  LowSpeedDirectControlEntryFeasibility resolution;
  if (
    !std::isfinite(request.current_speed_mps) || request.current_speed_mps < 0.0 ||
    !std::isfinite(request.shift_speed_mps) || request.shift_speed_mps < 0.0 ||
    !std::isfinite(request.maximum_deceleration_mps2) ||
    request.maximum_deceleration_mps2 <= 0.0 ||
    !std::isfinite(request.forward_distance_m) || request.forward_distance_m < 0.0 ||
    !std::isfinite(request.front_reserve_m) || request.front_reserve_m < 0.0 ||
    !std::isfinite(request.control_latency_sec) || request.control_latency_sec < 0.0)
  {
    return resolution;
  }

  resolution.valid = true;
  resolution.available_distance_m = std::max(
    0.0, request.forward_distance_m - request.front_reserve_m);
  const double braking_distance_m = request.current_speed_mps > request.shift_speed_mps ?
    (request.current_speed_mps * request.current_speed_mps -
    request.shift_speed_mps * request.shift_speed_mps) /
    (2.0 * request.maximum_deceleration_mps2) : 0.0;
  resolution.required_distance_m =
    request.current_speed_mps * request.control_latency_sec +
    std::max(0.0, braking_distance_m);
  resolution.feasible =
    resolution.required_distance_m <= resolution.available_distance_m + 1e-9;
  return resolution;
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

LowSpeedDirectSteeringBounds resolve_low_speed_direct_steering_bounds(
  const double previous_steering_rad, const double nominal_curve_steering_rad,
  const double maximum_steering_rad, const double maximum_steering_step_rad,
  const double current_speed_mps, const double wheelbase_m,
  const double maximum_lateral_acceleration_mps2,
  const double steering_command_gain)
{
  if (
    !std::isfinite(previous_steering_rad) ||
    !std::isfinite(nominal_curve_steering_rad) ||
    !std::isfinite(maximum_steering_rad) || maximum_steering_rad < 0.0 ||
    !std::isfinite(maximum_steering_step_rad) || maximum_steering_step_rad < 0.0)
  {
    throw std::invalid_argument("invalid low-speed steering bounds request");
  }
  const double bounded_previous = std::clamp(
    previous_steering_rad, -maximum_steering_rad, maximum_steering_rad);
  const double bounded_nominal = std::clamp(
    nominal_curve_steering_rad, -maximum_steering_rad, maximum_steering_rad);
  const double correction_limit = std::abs(
    limit_low_speed_shift_steering_by_lateral_acceleration(
      maximum_steering_rad, current_speed_mps, wheelbase_m,
      maximum_lateral_acceleration_mps2, steering_command_gain));
  const double curve_lower = std::max(
    -maximum_steering_rad, bounded_nominal - correction_limit);
  const double curve_upper = std::min(
    maximum_steering_rad, bounded_nominal + correction_limit);
  const double rate_lower = std::max(
    -maximum_steering_rad, bounded_previous - maximum_steering_step_rad);
  const double rate_upper = std::min(
    maximum_steering_rad, bounded_previous + maximum_steering_step_rad);
  const double intersection_lower = std::max(curve_lower, rate_lower);
  const double intersection_upper = std::min(curve_upper, rate_upper);
  if (intersection_lower <= intersection_upper) {
    return {intersection_lower, intersection_upper};
  }

  // The previous command can be outside the curve-centred envelope after a
  // contact or a planner handoff. Preserve the rate limit for this cycle while
  // making monotonic progress toward the nominal curve steering.
  const double rate_limited_nominal = std::clamp(
    bounded_nominal, rate_lower, rate_upper);
  return {rate_limited_nominal, rate_limited_nominal};
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
  if (
    request.rear_clear_confirmed &&
    (!request.side_vehicle_present ||
    (!request.target_seen && request.rear_clear_from_last_observation)))
  {
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
  return request.enabled && request.return_owner_allows_reacquire &&
         request.stable_target_id && request.same_target && request.same_side &&
         request.gap_available && request.execution_allowed &&
         !request.rear_clear_confirmed_latched &&
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
         request.solver_ready && request.replacement_mission_available &&
         request.replacement_deadline_checked && request.replacement_deadline_feasible &&
         request.replacement_goal_available;
}

bool should_suppress_completed_target_reacquire(
  const CompletedTargetReacquireSuppressionRequest & request) noexcept
{
  return request.completed_target_block_active &&
         request.candidate_matches_completed_target &&
         !request.committed_mission_active &&
         std::isfinite(request.now_sec) &&
         std::isfinite(request.block_until_sec) &&
         request.now_sec < request.block_until_sec;
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

PausedMissionExpiryReason resolve_paused_mission_expiry(
  const PausedMissionExpiryRequest & request) noexcept
{
  if (!request.follow_prepare_active) {
    return PausedMissionExpiryReason::Active;
  }
  if (
    std::isfinite(request.timeout_sec) && request.timeout_sec > 0.0 &&
    std::isfinite(request.elapsed_sec) && request.elapsed_sec >= request.timeout_sec)
  {
    return PausedMissionExpiryReason::TimeLimit;
  }
  if (
    std::isfinite(request.maximum_distance_m) && request.maximum_distance_m > 0.0 &&
    std::isfinite(request.traveled_distance_m) &&
    request.traveled_distance_m >= request.maximum_distance_m)
  {
    return PausedMissionExpiryReason::DistanceLimit;
  }
  return PausedMissionExpiryReason::Active;
}

PausedMissionTerminalResolution resolve_paused_mission_terminal(
  const PausedMissionTerminalRequest & request) noexcept
{
  if (!request.follow_prepare_active) {
    return {};
  }

  const auto expiry = resolve_paused_mission_expiry(
    PausedMissionExpiryRequest{
      true,
      request.elapsed_sec,
      request.traveled_distance_m,
      request.timeout_sec,
      request.maximum_distance_m});
  if (expiry == PausedMissionExpiryReason::TimeLimit) {
    return {
      PausedMissionTerminalAction::Expire,
      PausedMissionTerminalReason::TimeLimit};
  }
  if (expiry == PausedMissionExpiryReason::DistanceLimit) {
    return {
      PausedMissionTerminalAction::Expire,
      PausedMissionTerminalReason::DistanceLimit};
  }
  if (request.target_position_jump) {
    return {
      PausedMissionTerminalAction::Recovery,
      PausedMissionTerminalReason::TargetPositionJump};
  }
  if (request.target_course_progress_discontinuity) {
    return {
      PausedMissionTerminalAction::Recovery,
      PausedMissionTerminalReason::TargetCourseProgressDiscontinuity};
  }
  // Preserve the existing diagnostic priority when stale and forbidden are
  // observed in the same cycle: an explicit course restriction is actionable.
  if (request.forbidden_waypoint) {
    return {
      PausedMissionTerminalAction::Recovery,
      PausedMissionTerminalReason::ForbiddenWaypoint};
  }
  if (request.target_stale) {
    return {
      PausedMissionTerminalAction::Recovery,
      PausedMissionTerminalReason::TargetStale};
  }
  if (request.rear_clear_confirmed) {
    return {
      PausedMissionTerminalAction::Return,
      PausedMissionTerminalReason::RearClear};
  }
  return {};
}

const char * to_string(const PausedMissionTerminalReason reason) noexcept
{
  switch (reason) {
    case PausedMissionTerminalReason::None:
      return "none";
    case PausedMissionTerminalReason::TimeLimit:
      return "time_limit";
    case PausedMissionTerminalReason::DistanceLimit:
      return "distance_limit";
    case PausedMissionTerminalReason::TargetPositionJump:
      return "target_position_jump";
    case PausedMissionTerminalReason::TargetCourseProgressDiscontinuity:
      return "target_course_progress_discontinuity";
    case PausedMissionTerminalReason::TargetStale:
      return "target_stale";
    case PausedMissionTerminalReason::ForbiddenWaypoint:
      return "forbidden_waypoint";
    case PausedMissionTerminalReason::RearClear:
      return "rear_clear";
  }
  return "unknown";
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

bool should_retain_pass_mission_after_recovery(
  const RecoveryMissionRetentionRequest & request) noexcept
{
  return request.normal_recovery_complete &&
         !request.mission_retention_forbidden &&
         !request.solver_recovery_active &&
         !request.actual_wall_physical_contact &&
         request.locked_target_seen &&
         !request.target_position_jump &&
         !request.overtake_forbidden_waypoint &&
         std::isfinite(request.target_longitudinal_m) &&
         request.target_longitudinal_m > -request.return_clear_distance_m;
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

FrontHazardTargetContinuityResolution resolve_front_hazard_target_continuity(
  const FrontHazardTargetContinuityRequest & request) noexcept
{
  if (
    !request.held_target_matches || !request.observation_valid ||
    !std::isfinite(request.self_distance_m) || request.self_distance_m < 0.0 ||
    !std::isfinite(request.rear_clear_distance_m) || request.rear_clear_distance_m < 0.0 ||
    !std::isfinite(request.danger_lateral_range_m) || request.danger_lateral_range_m < 0.0)
  {
    return {};
  }

  const bool local_lateral_conflict =
    std::isfinite(request.local_relative_lateral_m) &&
    std::abs(request.local_relative_lateral_m) <= request.danger_lateral_range_m;
  const bool course_lateral_conflict =
    request.course_progress_valid &&
    std::isfinite(request.course_relative_lateral_m) &&
    std::abs(request.course_relative_lateral_m) <= request.danger_lateral_range_m;
  const bool near_field_conflict =
    request.self_distance_m <= request.rear_clear_distance_m &&
    (local_lateral_conflict || course_lateral_conflict);

  double rear_longitudinal_m = request.local_longitudinal_m;
  if (
    request.course_progress_valid &&
    std::isfinite(request.course_longitudinal_m))
  {
    rear_longitudinal_m = request.course_longitudinal_m;
  }
  const bool rear_clear =
    std::isfinite(rear_longitudinal_m) &&
    rear_longitudinal_m <= -request.rear_clear_distance_m &&
    !near_field_conflict;
  return {near_field_conflict, rear_clear};
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

bool can_suppress_committed_corridor_front_danger(
  const CommittedCorridorFrontDangerSuppressionRequest & request) noexcept
{
  const bool current_overlap_grace_active =
    request.committed_pass_attack_mode_enabled && request.pass_phase &&
    request.prior_front_cap_release_active &&
    !request.current_body_footprints_separated &&
    !request.current_body_footprint_overlap_confirmed &&
    request.footprint_prediction_valid;
  const bool current_geometry_acceptable =
    request.current_body_footprints_separated || current_overlap_grace_active ||
    request.recoverable_side_contact_active;
  const bool attack_path_acceptable =
    request.committed_pass_attack_mode_enabled && request.pass_phase &&
    request.prior_front_cap_release_active &&
    current_geometry_acceptable;
  const bool validated_body_clear_handoff_path_acceptable =
    request.committed_pass_attack_mode_enabled &&
    request.validated_body_clear_handoff_active &&
    request.current_body_footprints_separated;
  const bool predicted_path_acceptable =
    request.predicted_body_footprint_sweep_separated ||
    (request.prior_front_cap_release_active &&
    (!request.predicted_body_footprint_overlap_confirmed ||
    request.minimum_motion_side_by_side_escape_active)) ||
    attack_path_acceptable || validated_body_clear_handoff_path_acceptable;
  return request.enabled && request.active_shiftout_or_pass &&
         request.nearest_front_matches_locked_target && request.validated_fixed_corridor &&
         !request.inter_vehicle_corridor && request.target_seen &&
         !request.target_position_jump && current_geometry_acceptable &&
         (request.footprint_prediction_valid || attack_path_acceptable ||
         validated_body_clear_handoff_path_acceptable) &&
         predicted_path_acceptable;
}

CommittedPassBodyGeometryResolution resolve_committed_pass_body_geometry(
  const CommittedPassBodyGeometryRequest & request) noexcept
{
  CommittedPassBodyGeometryResolution resolution;
  resolution.body_longitudinal_clearance_m =
    0.5 * std::max(0.0, request.ego_vehicle_length_m) +
    0.5 * std::max(0.0, request.target_vehicle_length_m);
  resolution.side_by_side_escape_active =
    request.pass_phase &&
    request.current_body_footprints_separated &&
    std::isfinite(request.target_longitudinal_m) &&
    resolution.body_longitudinal_clearance_m > 0.0 &&
    request.target_longitudinal_m <= resolution.body_longitudinal_clearance_m;
  resolution.raw_predicted_body_overlap =
    request.footprint_prediction_valid &&
    !request.predicted_body_footprint_sweep_separated;
  const bool prediction_confirmation_phase =
    request.pass_phase ||
    (request.shiftout_phase && request.validated_shiftout_body_clear_deadline);
  resolution.predicted_overlap_confirmation_eligible =
    prediction_confirmation_phase &&
    request.minimum_motion_corridor_active &&
    request.prior_front_cap_release_active &&
    request.target_seen &&
    !request.target_position_jump &&
    request.current_body_footprints_separated &&
    resolution.raw_predicted_body_overlap &&
    (!resolution.side_by_side_escape_active || request.forward_completion_latched);
  return resolution;
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
