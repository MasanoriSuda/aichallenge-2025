#include "multi_purpose_mpc_ros_cpp/ref_velocity_config.hpp"

#include <gtest/gtest.h>

namespace multi_purpose_mpc_ros_cpp
{

TEST(RefVelocityConfig, ReturnsConfiguredVelocityForClosedSections)
{
  RefVelocityConfig config(TEST_REF_VEL_YAML_PATH);

  EXPECT_FALSE(config.empty());
  EXPECT_DOUBLE_EQ(config.get_ref_vel_kmph(0), 25.0);
  EXPECT_DOUBLE_EQ(config.get_ref_vel_kmph(10), 30.0);
  EXPECT_DOUBLE_EQ(config.get_ref_vel_kmph(45), 20.0);
  EXPECT_DOUBLE_EQ(config.get_ref_vel_kmph(90), 25.0);
}

TEST(RefVelocityConfig, EmptyPathKeepsConfigEmpty)
{
  RefVelocityConfig config("");

  EXPECT_TRUE(config.empty());
}

}  // namespace multi_purpose_mpc_ros_cpp
