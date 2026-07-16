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
    (!request.target_seen && request.target_age_sec <= request.target_hold_sec))
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

}  // namespace multi_purpose_mpc_ros::v2x_overtake_core
