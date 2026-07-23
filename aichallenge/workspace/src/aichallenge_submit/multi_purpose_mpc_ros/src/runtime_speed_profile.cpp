#include "multi_purpose_mpc_ros/runtime_speed_profile.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace multi_purpose_mpc_ros::runtime_speed_profile
{
namespace
{

double reachable_velocity(
  const double start_velocity, const double acceleration, const double distance)
{
  return std::sqrt(
    std::max(0.0, start_velocity * start_velocity + 2.0 * acceleration * distance));
}

bool finite_nonnegative(const double value)
{
  return std::isfinite(value) && value >= 0.0;
}

}  // namespace

std::optional<Result> compute(
  const std::vector<double> & curvature_radpm,
  const std::vector<Edge> & edges,
  const Parameters & parameters)
{
  if (
    curvature_radpm.size() < 2U ||
    !finite_nonnegative(parameters.acceleration_max_mps2) ||
    !finite_nonnegative(parameters.deceleration_max_mps2) ||
    !finite_nonnegative(parameters.velocity_min_mps) ||
    !finite_nonnegative(parameters.velocity_max_mps) ||
    !finite_nonnegative(parameters.lateral_acceleration_max_mps2) ||
    parameters.velocity_min_mps > parameters.velocity_max_mps ||
    !std::isfinite(parameters.tolerance) || parameters.tolerance <= 0.0 ||
    parameters.max_iterations == 0U)
  {
    return std::nullopt;
  }
  if (
    parameters.terminal_velocity_mps.has_value() &&
    (!finite_nonnegative(parameters.terminal_velocity_mps.value()) ||
    parameters.terminal_velocity_mps.value() > parameters.velocity_max_mps))
  {
    return std::nullopt;
  }

  std::vector<double> velocity;
  velocity.reserve(curvature_radpm.size());
  constexpr double kCurvatureEpsilon = 1e-12;
  for (const double curvature : curvature_radpm) {
    if (!std::isfinite(curvature)) {
      return std::nullopt;
    }
    const double lateral_limit = std::sqrt(
      parameters.lateral_acceleration_max_mps2 /
      std::max(std::abs(curvature), kCurvatureEpsilon));
    const double limit = std::min(parameters.velocity_max_mps, lateral_limit);
    if (!std::isfinite(limit) || limit + parameters.tolerance < parameters.velocity_min_mps) {
      return std::nullopt;
    }
    velocity.push_back(limit);
  }

  for (const auto & edge : edges) {
    if (
      edge.start >= velocity.size() || edge.end >= velocity.size() ||
      !std::isfinite(edge.distance_m) || edge.distance_m <= 0.0)
    {
      return std::nullopt;
    }
  }
  if (parameters.terminal_velocity_mps.has_value()) {
    velocity.back() = std::min(velocity.back(), parameters.terminal_velocity_mps.value());
  }

  for (std::size_t iteration = 1U; iteration <= parameters.max_iterations; ++iteration) {
    double maximum_change = 0.0;
    for (const auto & edge : edges) {
      const double limit = reachable_velocity(
        velocity[edge.start], parameters.acceleration_max_mps2, edge.distance_m);
      if (limit < velocity[edge.end]) {
        maximum_change = std::max(maximum_change, velocity[edge.end] - limit);
        velocity[edge.end] = limit;
      }
    }
    for (auto edge = edges.rbegin(); edge != edges.rend(); ++edge) {
      const double limit = reachable_velocity(
        velocity[edge->end], parameters.deceleration_max_mps2, edge->distance_m);
      if (limit < velocity[edge->start]) {
        maximum_change = std::max(maximum_change, velocity[edge->start] - limit);
        velocity[edge->start] = limit;
      }
    }
    if (maximum_change <= parameters.tolerance) {
      if (std::any_of(
          velocity.begin(), velocity.end(),
          [&parameters](const double value) {
            return !std::isfinite(value) ||
                   value + parameters.tolerance < parameters.velocity_min_mps;
          }))
      {
        return std::nullopt;
      }
      return Result{std::move(velocity), iteration};
    }
  }
  return std::nullopt;
}

}  // namespace multi_purpose_mpc_ros::runtime_speed_profile
