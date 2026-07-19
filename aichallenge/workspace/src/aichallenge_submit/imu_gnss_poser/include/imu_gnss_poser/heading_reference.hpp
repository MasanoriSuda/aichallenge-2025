#pragma once

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <cstddef>
#include <istream>
#include <optional>
#include <vector>

namespace imu_gnss_poser
{

struct Point2D
{
  double x{};
  double y{};
};

struct InitialPoseCovariance
{
  double x{};
  double y{};
  double yaw{};
};

struct RacelineInitialPose
{
  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  double yaw_rad{};
  std::size_t reference_index{};
};

std::vector<Point2D> load_path_points_csv(std::istream & input);

std::optional<std::size_t> find_closest_finite_point(
  const std::vector<Point2D> & points, double query_x, double query_y) noexcept;

std::optional<double> compute_path_yaw(
  const std::vector<Point2D> & points, std::size_t index) noexcept;

std::optional<RacelineInitialPose> make_raceline_initial_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped & gnss_pose,
  const std::vector<Point2D> & points,
  const InitialPoseCovariance & covariance) noexcept;

}  // namespace imu_gnss_poser
