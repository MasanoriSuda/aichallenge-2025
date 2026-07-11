#include "multi_purpose_mpc_ros/v2x_overtake_core.hpp"

#include <algorithm>
#include <cmath>
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

}  // namespace multi_purpose_mpc_ros::v2x_overtake_core
