#ifndef MULTI_PURPOSE_MPC_ROS__PREDICTION_MARKERS_HPP_
#define MULTI_PURPOSE_MPC_ROS__PREDICTION_MARKERS_HPP_

#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <utility>
#include <vector>

namespace multi_purpose_mpc_ros::prediction_markers
{

// Each message replaces the complete display on the controller-owned topics.
// DELETEALL also retires individual SPHERE IDs left by the previous representation.
inline visualization_msgs::msg::MarkerArray build(
  const std::vector<double> & x, const std::vector<double> & y,
  const std_msgs::msg::ColorRGBA & color)
{
  using Marker = visualization_msgs::msg::Marker;
  visualization_msgs::msg::MarkerArray result;
  result.markers.reserve(2);
  Marker clear;
  clear.header.frame_id = "map";
  clear.action = Marker::DELETEALL;
  result.markers.push_back(std::move(clear));

  Marker points;
  points.header.frame_id = "map";
  points.ns = "mpc_pred";
  points.id = 0;
  points.type = Marker::SPHERE_LIST;
  points.action = Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.scale.x = 0.5;
  points.scale.y = 0.5;
  points.scale.z = 0.5;
  points.color = color;
  points.points.resize(std::min(x.size(), y.size()));
  for (std::size_t i = 0; i < points.points.size(); ++i) {
    points.points[i].x = x[i];
    points.points[i].y = y[i];
  }
  result.markers.push_back(std::move(points));
  return result;
}

}  // namespace multi_purpose_mpc_ros::prediction_markers

#endif  // MULTI_PURPOSE_MPC_ROS__PREDICTION_MARKERS_HPP_
