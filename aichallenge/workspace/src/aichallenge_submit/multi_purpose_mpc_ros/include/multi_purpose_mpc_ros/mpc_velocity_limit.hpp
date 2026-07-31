#pragma once

#include <vector>

namespace multi_purpose_mpc_ros::mpc_velocity_limit
{

struct ReachableLimitRequest
{
  int horizon_size{0};
  double current_velocity_mps{0.0};
  double target_velocity_mps{0.0};
  double max_deceleration_mps2{0.0};
  double time_step_sec{0.0};
};

struct SolverFailureCrawlRequest
{
  bool simulation_environment{false};
  bool enabled{false};
  bool control_enabled{false};
  bool solver_fallback{false};
  bool unrestricted_cruise{false};
  bool front_vehicle_detected{false};
  bool current_static_footprint_clear{false};
  double lateral_error_m{0.0};
  double heading_error_rad{0.0};
  double max_lateral_error_m{0.0};
  double max_heading_error_rad{0.0};
  double configured_speed_mps{0.0};
  double effective_speed_limit_mps{0.0};
};

struct SolverFailureCrawlDecision
{
  bool active{false};
  double target_speed_mps{0.0};
};

/// Build a velocity upper-bound envelope that can be reached without exceeding
/// the configured longitudinal deceleration limit.
std::vector<double> build_reachable_limits(const ReachableLimitRequest & request);

/// Select a simulation-only fail-operational crawl after an MPC solve failure.
/// The decision fails closed unless normal V2X Cruise reports no front vehicle,
/// path-tracking errors remain inside the recovery rejoin envelope, and the
/// current static-map footprint is clear.
SolverFailureCrawlDecision resolve_solver_failure_crawl(
  const SolverFailureCrawlRequest & request) noexcept;

}  // namespace multi_purpose_mpc_ros::mpc_velocity_limit
