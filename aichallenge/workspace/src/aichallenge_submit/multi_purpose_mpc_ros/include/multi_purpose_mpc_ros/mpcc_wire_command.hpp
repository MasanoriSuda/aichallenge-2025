#pragma once

#include <cmath>
#include <limits>

namespace multi_purpose_mpc_ros::mpcc_wire_command {
/// The raw ROS message first serializes physical steering to float32, then
/// calibration is applied, then the final ROS field serializes to float32.
/// All prediction/program producers and the publisher use this exact order.
inline double steering(double physical_rad, double gain) noexcept {
  if (!std::isfinite(physical_rad) || !std::isfinite(gain) || gain <= 0)
    return std::numeric_limits<double>::quiet_NaN();
  const float raw = static_cast<float>(physical_rad);
  const float final = static_cast<float>(static_cast<double>(raw) * gain);
  return std::isfinite(final) ? static_cast<double>(final)
                              : std::numeric_limits<double>::quiet_NaN();
}
} // namespace multi_purpose_mpc_ros::mpcc_wire_command
