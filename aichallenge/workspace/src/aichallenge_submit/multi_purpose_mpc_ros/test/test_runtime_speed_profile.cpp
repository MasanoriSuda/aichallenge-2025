#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "multi_purpose_mpc_ros/runtime_speed_profile.hpp"

namespace
{

using multi_purpose_mpc_ros::runtime_speed_profile::Edge;
using multi_purpose_mpc_ros::runtime_speed_profile::Parameters;
using multi_purpose_mpc_ros::runtime_speed_profile::compute;

TEST(RuntimeSpeedProfile, UsesSquaredSpeedAccelerationAndBrakingConstraints)
{
  const std::vector<double> curvature{0.0, 0.0, 1.0, 0.0};
  const std::vector<Edge> edges{{0U, 1U, 2.0}, {1U, 2U, 2.0}, {2U, 3U, 2.0}};
  Parameters parameters;
  parameters.acceleration_max_mps2 = 1.0;
  parameters.deceleration_max_mps2 = 2.0;
  parameters.velocity_min_mps = 0.0;
  parameters.velocity_max_mps = 10.0;
  parameters.lateral_acceleration_max_mps2 = 4.0;

  const auto result = compute(curvature, edges, parameters);

  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->velocity_mps.size(), curvature.size());
  EXPECT_NEAR(result->velocity_mps[2], 2.0, 1e-12);
  EXPECT_LE(
    std::pow(result->velocity_mps[1], 2.0) -
    std::pow(result->velocity_mps[2], 2.0),
    2.0 * parameters.deceleration_max_mps2 * edges[1].distance_m + 1e-12);
  EXPECT_LE(
    std::pow(result->velocity_mps[3], 2.0) -
    std::pow(result->velocity_mps[2], 2.0),
    2.0 * parameters.acceleration_max_mps2 * edges[2].distance_m + 1e-12);
}

TEST(RuntimeSpeedProfile, RelaxesAcrossACircularSeam)
{
  const std::vector<double> curvature{1.0, 0.0, 0.0};
  const std::vector<Edge> edges{{0U, 1U, 1.0}, {1U, 2U, 1.0}, {2U, 0U, 1.0}};
  Parameters parameters;
  parameters.acceleration_max_mps2 = 1.0;
  parameters.deceleration_max_mps2 = 1.0;
  parameters.velocity_min_mps = 0.0;
  parameters.velocity_max_mps = 10.0;
  parameters.lateral_acceleration_max_mps2 = 1.0;

  const auto result = compute(curvature, edges, parameters);

  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->velocity_mps[0], 1.0, 1e-12);
  EXPECT_LE(
    std::pow(result->velocity_mps[2], 2.0) -
    std::pow(result->velocity_mps[0], 2.0),
    2.0 * parameters.deceleration_max_mps2 + 1e-12);
}

TEST(RuntimeSpeedProfile, AppliesOpenPathTerminalVelocity)
{
  const std::vector<double> curvature{0.0, 0.0, 0.0};
  const std::vector<Edge> edges{{0U, 1U, 1.0}, {1U, 2U, 1.0}};
  Parameters parameters;
  parameters.acceleration_max_mps2 = 1.0;
  parameters.deceleration_max_mps2 = 1.0;
  parameters.velocity_min_mps = 0.0;
  parameters.velocity_max_mps = 10.0;
  parameters.lateral_acceleration_max_mps2 = 10.0;
  parameters.terminal_velocity_mps = 0.0;

  const auto result = compute(curvature, edges, parameters);

  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(result->velocity_mps.back(), 0.0);
  EXPECT_NEAR(result->velocity_mps[1], std::sqrt(2.0), 1e-12);
}

}  // namespace
