#include <multi_purpose_mpc_ros/mpc_velocity_limit.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace multi_purpose_mpc_ros::mpc_velocity_limit
{

std::vector<double> build_reachable_limits(const ReachableLimitRequest & request)
{
  if (request.horizon_size < 0 ||
    !std::isfinite(request.current_velocity_mps) || request.current_velocity_mps < 0.0 ||
    !std::isfinite(request.target_velocity_mps) || request.target_velocity_mps < 0.0 ||
    !std::isfinite(request.max_deceleration_mps2) || request.max_deceleration_mps2 < 0.0 ||
    !std::isfinite(request.time_step_sec) || request.time_step_sec <= 0.0)
  {
    throw std::invalid_argument("invalid reachable velocity limit request");
  }

  std::vector<double> limits(static_cast<std::size_t>(request.horizon_size));
  for (int index = 0; index < request.horizon_size; ++index) {
    const double elapsed_sec = request.time_step_sec * static_cast<double>(index + 1);
    const double reachable_velocity = std::max(
      0.0,
      request.current_velocity_mps - request.max_deceleration_mps2 * elapsed_sec);
    limits[static_cast<std::size_t>(index)] =
      std::max(request.target_velocity_mps, reachable_velocity);
  }
  return limits;
}

SolverFailureCrawlDecision resolve_solver_failure_crawl(
  const SolverFailureCrawlRequest & request) noexcept
{
  SolverFailureCrawlDecision decision;
  if (
    !request.simulation_environment || !request.enabled || !request.control_enabled ||
    !request.solver_fallback || !request.unrestricted_cruise ||
    request.front_vehicle_detected || !request.current_static_footprint_clear ||
    !std::isfinite(request.lateral_error_m) ||
    !std::isfinite(request.heading_error_rad) ||
    !std::isfinite(request.max_lateral_error_m) ||
    request.max_lateral_error_m < 0.0 ||
    !std::isfinite(request.max_heading_error_rad) ||
    request.max_heading_error_rad < 0.0 ||
    std::abs(request.lateral_error_m) > request.max_lateral_error_m ||
    std::abs(request.heading_error_rad) > request.max_heading_error_rad ||
    !std::isfinite(request.configured_speed_mps) ||
    request.configured_speed_mps <= 0.0 ||
    !std::isfinite(request.effective_speed_limit_mps) ||
    request.effective_speed_limit_mps <= 0.0)
  {
    return decision;
  }

  decision.target_speed_mps = std::min(
    request.configured_speed_mps, request.effective_speed_limit_mps);
  decision.active = decision.target_speed_mps > 0.0;
  return decision;
}

SolverFailureContinuationDecision resolve_solver_failure_continuation(
  const SolverFailureContinuationRequest & request) noexcept
{
  SolverFailureContinuationDecision decision;
  if (
    !request.simulation_environment || !request.enabled ||
    !request.control_enabled || !request.solver_fallback)
  {
    decision.block_reason = SolverFailureContinuationBlockReason::Disabled;
    return decision;
  }
  if (!request.dynamic_obstacle_escape_active) {
    decision.block_reason = SolverFailureContinuationBlockReason::NotDynamicEscape;
    return decision;
  }
  if (request.emergency_active) {
    decision.block_reason = SolverFailureContinuationBlockReason::EmergencyActive;
    return decision;
  }
  if (
    request.consecutive_failure_count <= 0 || request.maximum_hold_cycles <= 0 ||
    request.consecutive_failure_count > request.maximum_hold_cycles)
  {
    decision.block_reason =
      SolverFailureContinuationBlockReason::FailureBudgetExceeded;
    return decision;
  }
  if (!request.current_static_footprint_clear) {
    decision.block_reason =
      SolverFailureContinuationBlockReason::StaticFootprintUnsafe;
    return decision;
  }
  if (!request.execution_path_validated) {
    decision.block_reason =
      SolverFailureContinuationBlockReason::ExecutionPathUnvalidated;
    return decision;
  }
  if (!request.tracking_envelope_valid) {
    decision.block_reason =
      SolverFailureContinuationBlockReason::TrackingEnvelopeUnsafe;
    return decision;
  }
  if (
    !std::isfinite(request.current_speed_mps) || request.current_speed_mps < 0.0 ||
    !std::isfinite(request.effective_speed_limit_mps) ||
    request.effective_speed_limit_mps <= 0.0)
  {
    decision.block_reason = SolverFailureContinuationBlockReason::InvalidSpeed;
    return decision;
  }

  decision.target_speed_mps = std::min(
    request.current_speed_mps, request.effective_speed_limit_mps);
  decision.active = decision.target_speed_mps > 0.0;
  decision.block_reason = decision.active ?
    SolverFailureContinuationBlockReason::None :
    SolverFailureContinuationBlockReason::InvalidSpeed;
  return decision;
}

const char * to_string(const SolverFailureContinuationBlockReason reason) noexcept
{
  switch (reason) {
    case SolverFailureContinuationBlockReason::None:
      return "none";
    case SolverFailureContinuationBlockReason::Disabled:
      return "disabled";
    case SolverFailureContinuationBlockReason::NotDynamicEscape:
      return "not-dynamic-escape";
    case SolverFailureContinuationBlockReason::EmergencyActive:
      return "emergency-active";
    case SolverFailureContinuationBlockReason::FailureBudgetExceeded:
      return "failure-budget-exceeded";
    case SolverFailureContinuationBlockReason::StaticFootprintUnsafe:
      return "static-footprint-unsafe";
    case SolverFailureContinuationBlockReason::ExecutionPathUnvalidated:
      return "execution-path-unvalidated";
    case SolverFailureContinuationBlockReason::TrackingEnvelopeUnsafe:
      return "tracking-envelope-unsafe";
    case SolverFailureContinuationBlockReason::InvalidSpeed:
      return "invalid-speed";
  }
  return "unknown";
}

}  // namespace multi_purpose_mpc_ros::mpc_velocity_limit
