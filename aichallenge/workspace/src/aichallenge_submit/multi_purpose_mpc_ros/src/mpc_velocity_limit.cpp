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

}  // namespace multi_purpose_mpc_ros::mpc_velocity_limit
