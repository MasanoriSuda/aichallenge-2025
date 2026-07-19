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

/// Build a velocity upper-bound envelope that can be reached without exceeding
/// the configured longitudinal deceleration limit.
std::vector<double> build_reachable_limits(const ReachableLimitRequest & request);

}  // namespace multi_purpose_mpc_ros::mpc_velocity_limit
