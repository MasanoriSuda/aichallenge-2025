#include "multi_purpose_mpc_ros/recovery_mpc.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace recovery_mpc = multi_purpose_mpc_ros::recovery_mpc;

namespace
{

recovery_mpc::Request request(
  const recovery_mpc::Direction direction, const double lateral_error_m,
  const double heading_error_rad, const double curvature_radpm = 0.0)
{
  recovery_mpc::Request value;
  value.direction = direction;
  value.lateral_error_m = lateral_error_m;
  value.heading_error_rad = heading_error_rad;
  value.reference_curvature_radpm = curvature_radpm;
  value.wheelbase_m = 2.0;
  value.initial_steering_tire_angle_rad = 0.0;
  return value;
}

}  // namespace

TEST(RecoveryMpc, ForwardPlanSteersTowardPath)
{
  const auto result = recovery_mpc::plan(
    recovery_mpc::Config{},
    request(recovery_mpc::Direction::Forward, 1.0, 0.0));

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.reason, recovery_mpc::RejectReason::None);
  EXPECT_LT(result.first_steering_tire_angle_rad, 0.0);
  EXPECT_LT(std::abs(result.terminal_lateral_error_m), 1.0);
}

TEST(RecoveryMpc, ReversePlanUsesSignedBicycleDynamics)
{
  const auto result = recovery_mpc::plan(
    recovery_mpc::Config{},
    request(recovery_mpc::Direction::Reverse, 1.0, 0.0));

  ASSERT_TRUE(result.valid);
  EXPECT_LT(result.first_steering_tire_angle_rad, 0.0);
  EXPECT_LT(std::abs(result.terminal_lateral_error_m), 1.0);
}

TEST(RecoveryMpc, ReverseHeadingCorrectionHasOppositeSteeringSense)
{
  const auto forward = recovery_mpc::plan(
    recovery_mpc::Config{},
    request(recovery_mpc::Direction::Forward, 0.0, 0.35));
  const auto reverse = recovery_mpc::plan(
    recovery_mpc::Config{},
    request(recovery_mpc::Direction::Reverse, 0.0, 0.35));

  ASSERT_TRUE(forward.valid);
  ASSERT_TRUE(reverse.valid);
  EXPECT_LT(forward.first_steering_tire_angle_rad, 0.0);
  EXPECT_GT(reverse.first_steering_tire_angle_rad, 0.0);
}

TEST(RecoveryMpc, CurvedReferenceProducesFeedForwardSteering)
{
  auto config = recovery_mpc::Config{};
  config.maximum_steering_change_rad = config.maximum_steering_angle_rad;
  const auto result = recovery_mpc::plan(
    config,
    request(recovery_mpc::Direction::Forward, 0.0, 0.0, 0.10));

  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.first_steering_tire_angle_rad, 0.0);
}

TEST(RecoveryMpc, FirstActionRespectsSteeringChangeBound)
{
  auto config = recovery_mpc::Config{};
  config.maximum_steering_change_rad = 0.04;
  auto value = request(recovery_mpc::Direction::Forward, 2.0, 0.0);
  value.initial_steering_tire_angle_rad = 0.20;
  const auto result = recovery_mpc::plan(config, value);

  ASSERT_TRUE(result.valid);
  EXPECT_LE(
    std::abs(result.first_steering_tire_angle_rad - value.initial_steering_tire_angle_rad),
    config.maximum_steering_change_rad + 1e-12);
}

TEST(RecoveryMpc, InvalidConfigFailsClosed)
{
  auto config = recovery_mpc::Config{};
  config.horizon_steps = 0U;
  const auto result = recovery_mpc::plan(
    config,
    request(recovery_mpc::Direction::Forward, 0.0, 0.0));

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, recovery_mpc::RejectReason::InvalidConfig);
  EXPECT_STREQ(recovery_mpc::to_string(result.reason), "invalid_config");
}
