#pragma once

#include <cstddef>
#include <optional>
#include <vector>

namespace multi_purpose_mpc_ros::runtime_speed_profile
{

struct Edge
{
  std::size_t start{};
  std::size_t end{};
  double distance_m{};
};

struct Parameters
{
  double acceleration_max_mps2{};
  double deceleration_max_mps2{};
  double velocity_min_mps{};
  double velocity_max_mps{};
  double lateral_acceleration_max_mps2{};
  std::optional<double> terminal_velocity_mps;
  double tolerance{1e-9};
  std::size_t max_iterations{1000U};
};

struct Result
{
  std::vector<double> velocity_mps;
  std::size_t iterations{};
};

std::optional<Result> compute(
  const std::vector<double> & curvature_radpm,
  const std::vector<Edge> & edges,
  const Parameters & parameters);

}  // namespace multi_purpose_mpc_ros::runtime_speed_profile
