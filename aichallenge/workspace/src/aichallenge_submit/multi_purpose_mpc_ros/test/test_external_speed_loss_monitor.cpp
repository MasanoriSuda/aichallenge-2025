#include <multi_purpose_mpc_ros/external_speed_loss_monitor.hpp>

#include <gtest/gtest.h>

#include <limits>

namespace external_speed_loss =
  multi_purpose_mpc_ros::external_speed_loss;

TEST(ExternalSpeedLossMonitor, IgnoresNominalBrakingInsideDiagnosticEnvelope)
{
  external_speed_loss::Monitor monitor;
  EXPECT_FALSE(monitor.update({1.0, 10.0}, -3.0).has_value());
  EXPECT_FALSE(monitor.update({1.1, 9.7}, -3.0).has_value());
}

TEST(ExternalSpeedLossMonitor, ReportsAbruptLossOutsideConfiguredControlEnvelope)
{
  external_speed_loss::Monitor monitor;
  EXPECT_FALSE(monitor.update({1.0, 10.0}, -3.0).has_value());
  const auto event = monitor.update({1.12, 1.4}, -3.0);
  ASSERT_TRUE(event.has_value());
  EXPECT_DOUBLE_EQ(event->previous_speed_mps, 10.0);
  EXPECT_DOUBLE_EQ(event->current_speed_mps, 1.4);
  EXPECT_NEAR(event->interval_sec, 0.12, 1e-12);
  EXPECT_LT(event->observed_acceleration_mps2, -70.0);
  EXPECT_DOUBLE_EQ(event->reportable_loss_threshold_mps, 1.0);
}

TEST(ExternalSpeedLossMonitor, RebasesAcrossStaleOrNonfiniteSamples)
{
  external_speed_loss::Monitor monitor;
  EXPECT_FALSE(monitor.update({1.0, 10.0}, -3.0).has_value());
  EXPECT_FALSE(monitor.update({2.0, 1.0}, -3.0).has_value());
  EXPECT_FALSE(
    monitor.update({2.1, std::numeric_limits<double>::quiet_NaN()}, -3.0).has_value());
  EXPECT_FALSE(monitor.update({2.2, 0.0}, -3.0).has_value());
}
