#include <gtest/gtest.h>

#include <multi_purpose_mpc_ros/mpc_waypoint_association.hpp>

#include <cmath>
#include <limits>
#include <vector>

namespace association = multi_purpose_mpc_ros::mpc_waypoint_association;

TEST(MpcWaypointAssociation, PrefersContinuousBranchAtHairpin)
{
  const std::vector<association::Waypoint> waypoints{
    {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0},
    {3.0, 0.0, 0.0}, {3.0, 1.0, 1.5707963267948966},
    {2.0, 1.0, 3.141592653589793}, {1.0, 1.0, 3.141592653589793},
    {0.0, 1.0, 3.141592653589793}};
  association::Config config;
  config.enabled = true;
  config.local_lookbehind_m = 2.0;
  config.local_lookahead_m = 4.0;
  config.lost_distance_m = 2.0;
  config.heading_weight_m_per_rad = 2.0;
  association::Request request;
  request.x_m = 1.0;
  request.y_m = 0.55;
  request.yaw_rad = 0.0;
  request.speed_mps = 5.0;
  request.dt_sec = 0.025;
  request.previous_index = 1;
  request.previous_valid = true;

  const auto result = association::associate(waypoints, request, config);
  EXPECT_EQ(result.index, 1);
  EXPECT_FALSE(result.used_global_search);
}

TEST(MpcWaypointAssociation, PreservesCircularProgressAcrossLapBoundary)
{
  const std::vector<association::Waypoint> waypoints{
    {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {3.0, 0.0, 0.0}};
  association::Config config;
  config.enabled = true;
  config.local_lookbehind_m = 2.0;
  config.local_lookahead_m = 4.0;
  config.lost_distance_m = 2.0;
  association::Request request;
  request.x_m = 0.05;
  request.y_m = 0.0;
  request.yaw_rad = 0.0;
  request.speed_mps = 3.0;
  request.dt_sec = 0.025;
  request.previous_index = 3;
  request.previous_valid = true;
  request.circular = true;

  const auto result = association::associate(waypoints, request, config);
  EXPECT_EQ(result.index, 0);
  EXPECT_FALSE(result.used_global_search);
  EXPECT_GT(result.signed_progress_m, 0.0);
}

TEST(MpcWaypointAssociation, FallsBackToGlobalOnlyWhenLocalTrackIsLost)
{
  const std::vector<association::Waypoint> waypoints{
    {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0},
    {100.0, 0.0, 0.0}, {101.0, 0.0, 0.0}};
  association::Config config;
  config.enabled = true;
  config.local_lookbehind_m = 1.0;
  config.local_lookahead_m = 2.0;
  config.lost_distance_m = 3.0;
  association::Request request;
  request.x_m = 100.1;
  request.y_m = 0.0;
  request.yaw_rad = 0.0;
  request.speed_mps = 0.0;
  request.dt_sec = 0.025;
  request.previous_index = 1;
  request.previous_valid = true;

  const auto result = association::associate(waypoints, request, config);
  EXPECT_EQ(result.index, 3);
  EXPECT_TRUE(result.used_global_search);
}

TEST(MpcWaypointAssociation, RejectsInvalidInputs)
{
  association::Config config;
  association::Request request;
  EXPECT_THROW(association::associate({}, request, config), std::invalid_argument);

  const std::vector<association::Waypoint> waypoints{{0.0, 0.0, 0.0}};
  request.x_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(association::associate(waypoints, request, config), std::invalid_argument);
}

