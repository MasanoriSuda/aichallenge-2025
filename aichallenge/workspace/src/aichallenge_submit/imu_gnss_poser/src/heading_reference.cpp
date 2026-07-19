#include <imu_gnss_poser/heading_reference.hpp>

#include <cmath>
#include <limits>

namespace imu_gnss_poser
{
namespace
{

constexpr double kMinimumSegmentLengthSquared = 1.0e-6;

bool finite_point(const Point2D & point) noexcept
{
  return std::isfinite(point.x) && std::isfinite(point.y);
}

bool finite_nonnegative(const double value) noexcept
{
  return std::isfinite(value) && value >= 0.0;
}

std::optional<double> segment_yaw(
  const Point2D & from, const Point2D & to) noexcept
{
  if (!finite_point(from) || !finite_point(to)) {
    return std::nullopt;
  }
  const double dx = to.x - from.x;
  const double dy = to.y - from.y;
  const double length_squared = dx * dx + dy * dy;
  if (!std::isfinite(length_squared) || length_squared <= kMinimumSegmentLengthSquared) {
    return std::nullopt;
  }
  return std::atan2(dy, dx);
}

}  // namespace

std::optional<std::size_t> find_closest_finite_point(
  const std::vector<Point2D> & points, const double query_x, const double query_y) noexcept
{
  if (!std::isfinite(query_x) || !std::isfinite(query_y)) {
    return std::nullopt;
  }

  std::optional<std::size_t> best_index;
  double best_distance_squared = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < points.size(); ++index) {
    if (!finite_point(points[index])) {
      continue;
    }
    const double dx = points[index].x - query_x;
    const double dy = points[index].y - query_y;
    const double distance_squared = dx * dx + dy * dy;
    if (std::isfinite(distance_squared) && distance_squared < best_distance_squared) {
      best_distance_squared = distance_squared;
      best_index = index;
    }
  }
  return best_index;
}

std::optional<double> compute_path_yaw(
  const std::vector<Point2D> & points, const std::size_t index) noexcept
{
  if (index >= points.size()) {
    return std::nullopt;
  }
  for (std::size_t segment = index; segment + 1 < points.size(); ++segment) {
    const auto yaw = segment_yaw(points[segment], points[segment + 1]);
    if (yaw.has_value()) {
      return yaw;
    }
  }
  for (std::size_t segment = index; segment > 0; --segment) {
    const auto yaw = segment_yaw(points[segment - 1], points[segment]);
    if (yaw.has_value()) {
      return yaw;
    }
  }
  return std::nullopt;
}

std::optional<RacelineInitialPose> make_raceline_initial_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped & gnss_pose,
  const std::vector<Point2D> & points,
  const InitialPoseCovariance & covariance) noexcept
{
  const auto & position = gnss_pose.pose.pose.position;
  if (
    points.size() < 2U || !std::isfinite(position.x) || !std::isfinite(position.y) ||
    !std::isfinite(position.z) || !finite_nonnegative(covariance.x) ||
    !finite_nonnegative(covariance.y) || !finite_nonnegative(covariance.yaw))
  {
    return std::nullopt;
  }

  const auto reference_index = find_closest_finite_point(points, position.x, position.y);
  if (!reference_index.has_value()) {
    return std::nullopt;
  }
  const auto yaw = compute_path_yaw(points, reference_index.value());
  if (!yaw.has_value() || !std::isfinite(yaw.value())) {
    return std::nullopt;
  }

  RacelineInitialPose result;
  result.pose.header = gnss_pose.header;
  result.pose.pose.pose.position = position;
  result.pose.pose.pose.orientation.x = 0.0;
  result.pose.pose.pose.orientation.y = 0.0;
  result.pose.pose.pose.orientation.z = std::sin(yaw.value() * 0.5);
  result.pose.pose.pose.orientation.w = std::cos(yaw.value() * 0.5);
  result.pose.pose.covariance[7 * 0] = covariance.x;
  result.pose.pose.covariance[7 * 1] = covariance.y;
  result.pose.pose.covariance[7 * 5] = covariance.yaw;
  result.yaw_rad = yaw.value();
  result.reference_index = reference_index.value();
  return result;
}

}  // namespace imu_gnss_poser

