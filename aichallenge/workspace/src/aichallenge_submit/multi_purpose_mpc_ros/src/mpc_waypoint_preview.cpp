#include <multi_purpose_mpc_ros/mpc_waypoint_preview.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace multi_purpose_mpc_ros::mpc_waypoint_preview
{
namespace
{

constexpr double kStraightCurvatureEpsilon = 1.0e-12;

} // namespace

bool is_valid_offset(const int offset)
{
  return offset >= kMinOffset && offset <= kMaxOffset;
}

int select_effective_offset(
  const int normal_offset,
  const int low_speed_offset,
  const double current_speed_mps,
  const double low_speed_threshold_mps)
{
  if (!is_valid_offset(normal_offset) || !is_valid_offset(low_speed_offset)) {
    throw std::invalid_argument("MPC waypoint preview offset must be within [0, 2]");
  }
  if (!std::isfinite(current_speed_mps) || current_speed_mps < 0.0 ||
    !std::isfinite(low_speed_threshold_mps) || low_speed_threshold_mps < 0.0)
  {
    throw std::invalid_argument("invalid MPC waypoint preview speed input");
  }

  if (low_speed_threshold_mps > 0.0 && current_speed_mps <= low_speed_threshold_mps) {
    return low_speed_offset;
  }
  return normal_offset;
}

int resolve_preview_index(
  const int tracking_index,
  const int offset,
  const int waypoint_count,
  const bool circular)
{
  if (waypoint_count <= 0) {
    throw std::invalid_argument("MPC waypoint preview requires a non-empty path");
  }
  if (tracking_index < 0 || tracking_index >= waypoint_count) {
    throw std::out_of_range("MPC tracking waypoint index is outside the path");
  }
  if (!is_valid_offset(offset)) {
    throw std::invalid_argument("MPC waypoint preview offset must be within [0, 2]");
  }

  const int candidate_index = tracking_index + offset;
  if (circular) {
    return candidate_index % waypoint_count;
  }
  return std::clamp(candidate_index, 0, waypoint_count - 1);
}

InputReference resolve_input_reference(
  const double tracking_velocity_mps,
  const double tracking_curvature_radpm,
  const double preview_velocity_mps,
  const double preview_curvature_radpm)
{
  if (!std::isfinite(tracking_velocity_mps) || tracking_velocity_mps < 0.0 ||
    !std::isfinite(preview_velocity_mps) || preview_velocity_mps < 0.0 ||
    !std::isfinite(tracking_curvature_radpm) || !std::isfinite(preview_curvature_radpm))
  {
    throw std::invalid_argument("invalid MPC waypoint input reference");
  }

  InputReference reference;
  // Preview may request earlier braking, but must not accelerate before the
  // tracking waypoint reaches the higher target speed.
  reference.velocity_mps = std::min(tracking_velocity_mps, preview_velocity_mps);
  reference.curvature_radpm = tracking_curvature_radpm;

  const bool tracking_is_straight =
    std::abs(tracking_curvature_radpm) <= kStraightCurvatureEpsilon;
  const bool same_turn_direction =
    (tracking_curvature_radpm > 0.0 && preview_curvature_radpm > 0.0) ||
    (tracking_curvature_radpm < 0.0 && preview_curvature_radpm < 0.0);
  const bool preview_requests_more_steering =
    std::abs(preview_curvature_radpm) > std::abs(tracking_curvature_radpm);
  if ((tracking_is_straight || same_turn_direction) && preview_requests_more_steering) {
    // Apply preview only when entering/tightening the same turn. Keeping the
    // tracking curvature on exit prevents an offset-dependent early unwind.
    reference.curvature_radpm = preview_curvature_radpm;
  }
  return reference;
}

} // namespace multi_purpose_mpc_ros::mpc_waypoint_preview
