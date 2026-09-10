#include <multi_purpose_mpc_ros/prediction_markers.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <map>
#include <utility>
#include <vector>

namespace
{
using Marker = visualization_msgs::msg::Marker;
using Array = visualization_msgs::msg::MarkerArray;
using multi_purpose_mpc_ros::prediction_markers::build;

TEST(PredictionMarkers, RoundTripPreservesEveryPointAndDisplayStyle)
{
  std::vector<double> x(924), y(924);
  for (std::size_t i = 0; i < x.size(); ++i) {
    x[i] = 0.031 * i;
    y[i] = std::sin(0.009 * i);
  }
  std_msgs::msg::ColorRGBA color;
  color.r = 0.2F;
  color.g = 0.4F;
  color.b = 0.8F;
  color.a = 0.7F;
  const auto original = build(x, y, color);
  rclcpp::Serialization<Array> serializer;
  rclcpp::SerializedMessage wire;
  serializer.serialize_message(&original, &wire);
  Array restored;
  serializer.deserialize_message(&wire, &restored);
  ASSERT_EQ(restored.markers.size(), 2U);
  const auto & marker = restored.markers[1];
  ASSERT_EQ(marker.type, Marker::SPHERE_LIST);
  ASSERT_EQ(marker.points.size(), x.size());
  EXPECT_EQ(marker.header.frame_id, "map");
  EXPECT_EQ(marker.ns, "mpc_pred");
  EXPECT_EQ(marker.pose.position, geometry_msgs::msg::Point());
  EXPECT_DOUBLE_EQ(marker.pose.orientation.w, 1.0);
  EXPECT_DOUBLE_EQ(marker.pose.orientation.x, 0.0);
  EXPECT_DOUBLE_EQ(marker.pose.orientation.y, 0.0);
  EXPECT_DOUBLE_EQ(marker.pose.orientation.z, 0.0);
  EXPECT_DOUBLE_EQ(marker.scale.x, 0.5);
  EXPECT_DOUBLE_EQ(marker.scale.y, 0.5);
  EXPECT_DOUBLE_EQ(marker.scale.z, 0.5);
  EXPECT_EQ(marker.color, color);
  EXPECT_TRUE(marker.colors.empty());
  for (std::size_t i = 0; i < x.size(); ++i) {
    EXPECT_DOUBLE_EQ(marker.points[i].x, x[i]);
    EXPECT_DOUBLE_EQ(marker.points[i].y, y[i]);
    EXPECT_DOUBLE_EQ(marker.points[i].z, 0.0);
  }
}

TEST(PredictionMarkers, ReplacementRetiresLegacyIdsAndShrinkingOrEmptyTrajectories)
{
  std::map<std::pair<std::string, int>, Marker> display;
  for (int i = 0; i < 924; ++i) {
    Marker legacy;
    legacy.ns = "mpc_pred";
    legacy.id = i;
    legacy.type = Marker::SPHERE;
    display[{legacy.ns, legacy.id}] = legacy;
  }
  // Apply ROS Marker actions as a subscriber, including replacement by (ns,id).
  const auto apply = [&display](const Array & message) {
      for (const auto & marker : message.markers) {
        if (marker.action == Marker::DELETEALL) {
          display.clear();
        } else if (marker.action == Marker::ADD) {
          display[{marker.ns, marker.id}] = marker;
        }
      }
    };
  const std_msgs::msg::ColorRGBA color;
  apply(build({1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}, color));
  ASSERT_EQ(display.size(), 1U);
  ASSERT_EQ(display.begin()->second.points.size(), 3U);
  apply(build({7.0, 8.0}, {9.0}, color));
  ASSERT_EQ(display.size(), 1U);
  ASSERT_EQ(display.begin()->second.points.size(), 1U);
  EXPECT_DOUBLE_EQ(display.begin()->second.points.front().x, 7.0);
  EXPECT_DOUBLE_EQ(display.begin()->second.points.front().y, 9.0);
  apply(build({}, {}, color));
  ASSERT_EQ(display.size(), 1U);
  EXPECT_TRUE(display.begin()->second.points.empty());
}
}  // namespace
