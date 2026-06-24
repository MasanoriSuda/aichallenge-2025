#include "multi_purpose_mpc_ros_cpp/controller.hpp"

#include <gtest/gtest.h>

namespace multi_purpose_mpc_ros_cpp
{

TEST(LapTracker, ReportsCompletedLapsWithFallbackDelta)
{
  LapTracker tracker;

  EXPECT_FALSE(tracker.update(0, 0.0, 0));
  EXPECT_FALSE(tracker.update(1, 12.5, 2));

  const auto lap1 = tracker.update(2, 25.0, 5);
  ASSERT_TRUE(lap1.has_value());
  EXPECT_EQ(lap1->completed_lap, 1);
  EXPECT_DOUBLE_EQ(lap1->lap_time_sec, 12.5);
  EXPECT_EQ(lap1->lap_fallbacks, 5U);
  EXPECT_EQ(lap1->total_fallbacks, 5U);

  const auto lap2 = tracker.update(3, 38.0, 8);
  ASSERT_TRUE(lap2.has_value());
  EXPECT_EQ(lap2->completed_lap, 2);
  EXPECT_DOUBLE_EQ(lap2->lap_time_sec, 25.0);
  EXPECT_EQ(lap2->lap_fallbacks, 3U);
  EXPECT_EQ(lap2->total_fallbacks, 8U);
}

}  // namespace multi_purpose_mpc_ros_cpp
