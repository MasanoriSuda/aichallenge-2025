#ifndef SIMPLE_MPC_HPP_
#define SIMPLE_MPC_HPP_

// mpc_node.cpp（ROS2ノード化・initとspin分割のベース）
#include "reference_path.hpp"
#include "spatial_bicycle_models.hpp"
#include "MPC.hpp"
#include "OdometryInput.hpp"

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>

namespace simple_mpc {

using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PointStamped;
using geometry_msgs::msg::Twist;
using nav_msgs::msg::Odometry;

class SimpleMpc : public rclcpp::Node {
 public:
  explicit SimpleMpc();
  
  // subscribers
  rclcpp::Subscription<Odometry>::SharedPtr sub_kinematics_;
  rclcpp::Subscription<Trajectory>::SharedPtr sub_trajectory_;
  
  // publishers
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_cmd_;
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_raw_cmd_;
  rclcpp::Publisher<PointStamped>::SharedPtr pub_lookahead_point_;  

  // timer
  rclcpp::TimerBase::SharedPtr timer_;

  // updated by subscribers
  Trajectory::SharedPtr trajectory_;
  Odometry::SharedPtr odometry_;

    // メンバ変数
  std::shared_ptr<ReferencePath> reference_path;
  std::shared_ptr<BicycleModel> car;
  std::shared_ptr<MPC> mpc;
  std::vector<std::vector<double>> log_;  // ⬅️ これが必要
  std::vector<double> wp_speed;
  int N = 30;
  int WAYPOINT_NUM = 10;
  double total_s;


  // pure pursuit parameters
  //const double wheel_base_;
  //const double lookahead_gain_;
  //const double lookahead_min_distance_;
  //const double speed_proportional_gain_;
  //const bool use_external_target_vel_;
  //const double external_target_vel_;
  //const double steering_tire_angle_gain_;


 private:
  rclcpp::Time prev_time_;
  rclcpp::Time start_time_;
  double max_speed_;
  bool is_initialized;
  bool is_nearrest_0;
 void onTimer();
  bool subscribeMessageAvailable();
};

}  // namespace simple_mpc

#endif  // SIMPLE_MPC_HPP_