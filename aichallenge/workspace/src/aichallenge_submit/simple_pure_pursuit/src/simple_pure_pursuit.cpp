#include "simple_pure_pursuit/simple_pure_pursuit.hpp"

#include <motion_utils/motion_utils.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <tf2/utils.h>

#include <algorithm>

namespace simple_pure_pursuit
{

using motion_utils::findNearestIndex;
using tier4_autoware_utils::calcLateralDeviation;
using tier4_autoware_utils::calcYawDeviation;

SimplePurePursuit::SimplePurePursuit()
: Node("simple_pure_pursuit"),
  // initialize parameters
  wheel_base_(declare_parameter<float>("wheel_base", 2.14)),
  lookahead_gain_(declare_parameter<float>("lookahead_gain", 1.0)),
  lookahead_min_distance_(declare_parameter<float>("lookahead_min_distance", 1.0)),
  speed_proportional_gain_(declare_parameter<float>("speed_proportional_gain", 1.0)),
  use_external_target_vel_(declare_parameter<bool>("use_external_target_vel", false)),
  external_target_vel_(declare_parameter<float>("external_target_vel", 0.0)),
  steering_tire_angle_gain_(declare_parameter<float>("steering_tire_angle_gain", 1.0))
{
  pub_cmd_ = create_publisher<AckermannControlCommand>("output/control_cmd", 1);
  pub_raw_cmd_ = create_publisher<AckermannControlCommand>("output/raw_control_cmd", 1);
  pub_lookahead_point_ = create_publisher<PointStamped>("/control/debug/lookahead_point", 1);

  const auto bv_qos = rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort();
  sub_kinematics_ = create_subscription<Odometry>(
    "input/kinematics", bv_qos, [this](const Odometry::SharedPtr msg) { odometry_ = msg; });
  sub_trajectory_ = create_subscription<Trajectory>(
    "input/trajectory", bv_qos, [this](const Trajectory::SharedPtr msg) { trajectory_ = msg; });

  using namespace std::literals::chrono_literals;
  timer_ =
    rclcpp::create_timer(this, get_clock(), 10ms, std::bind(&SimplePurePursuit::onTimer, this));
  is_reached = false;
}

AckermannControlCommand zeroAckermannControlCommand(rclcpp::Time stamp)
{
  AckermannControlCommand cmd;
  cmd.stamp = stamp;
  cmd.longitudinal.stamp = stamp;
  cmd.longitudinal.speed = 0.0;
  cmd.longitudinal.acceleration = 0.0;
  cmd.lateral.stamp = stamp;
  cmd.lateral.steering_tire_angle = 0.0;
  return cmd;
}

void SimplePurePursuit::onTimer()
{
  // check data
  if (!subscribeMessageAvailable()) {
    return;
  }

  auto odom_pred = odometry_;

    //current pos
  double x = odometry_->pose.pose.position.x;
  double y = odometry_->pose.pose.position.y;
  double yaw = tf2::getYaw(odometry_->pose.pose.orientation);
  double pred_dt =0.20;

  //current spped
  // 車体座標の並進速度（m/s）
  const double vx = odometry_->twist.twist.linear.x; // 前方向
  //const double vy = odometry_->twist.twist.linear.y; // 横方向（多くの車は≈0）
  // 回頭角速度（rad/s）
  const double omega = odometry_->twist.twist.angular.z;
  const double v_tmp = vx;

  double pred_x,pred_y,pred_yaw;
  if (std::abs(omega) < 1e-6) {
      // ほぼ直進
      const double dx = v_tmp * pred_dt * std::cos(yaw);
      const double dy = v_tmp * pred_dt * std::sin(yaw);
      pred_x = x + dx;
      pred_y= y + dy;
      pred_yaw = yaw;
  } else {
      // 円弧解（厳密）
      const double yaw2 = yaw + omega * pred_dt;
      const double R = v_tmp / omega; // 旋回半径（符号付き）
      const double dx =  R * (std::sin(yaw2) - std::sin(yaw));
      const double dy = -R * (std::cos(yaw2) - std::cos(yaw));
      pred_x = x + dx;
      pred_y = y + dy;
      pred_yaw = yaw2;
  }


  tf2::Quaternion q_new;
  q_new.setRPY(0, 0, pred_yaw);
  odom_pred->pose.pose.position.x = pred_x;
  odom_pred->pose.pose.position.y = pred_y;
  odom_pred->pose.pose.orientation = tf2::toMsg(q_new);

  size_t closet_traj_point_idx =
    findNearestIndex(trajectory_->points, odom_pred->pose.pose.position);
    size_t hoge_idx;
    if(closet_traj_point_idx == 90 && is_reached == false){
      is_reached = true;
    }

    if(is_reached == true){
      hoge_idx = 7;
    } else {
      hoge_idx = 7;
    }

    if(closet_traj_point_idx > 240 && closet_traj_point_idx < 250){
      hoge_idx = 7;
    }
    hoge_idx = 0;

    closet_traj_point_idx = (closet_traj_point_idx + hoge_idx) % trajectory_->points.size();

  // publish zero command
  AckermannControlCommand cmd = zeroAckermannControlCommand(get_clock()->now());

  if (0) {
    cmd.longitudinal.speed = 0.0;
    cmd.longitudinal.acceleration = -10.0;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "reached to the goal");
  } else {
    // get closest trajectory point from current position
    TrajectoryPoint closet_traj_point = trajectory_->points.at(closet_traj_point_idx);

    // calc longitudinal speed and acceleration
    double target_longitudinal_vel =
      use_external_target_vel_ ? external_target_vel_ : closet_traj_point.longitudinal_velocity_mps;
    //double current_longitudinal_vel = odom_pred->twist.twist.linear.x;

    #if 0
    cmd.longitudinal.speed = 35.0 * (34.9/32.5) /3.6;
    double v_current = std::hypot(odometry_->twist.twist.linear.x, odometry_->twist.twist.linear.y);
    cmd.longitudinal.acceleration =  (34.90 * (35.0/32.5) / 3.6) - v_current;
    #else
    cmd.longitudinal.speed = 6.0 / 3.6;
    double v_current = std::hypot(odometry_->twist.twist.linear.x, odometry_->twist.twist.linear.y);
    cmd.longitudinal.acceleration =  6.0 / 3.6 - v_current;
    #endif


    // calc lateral control
    //// calc lookahead distance
    double lookahead_distance = lookahead_gain_ * target_longitudinal_vel + lookahead_min_distance_;
    const double yaw = tf2::getYaw(odom_pred->pose.pose.orientation);
    double rear_x = odom_pred->pose.pose.position.x - (wheel_base_/2.0)*std::cos(yaw);
    double rear_y = odom_pred->pose.pose.position.y - (wheel_base_/2.0)*std::sin(yaw);

    //// search lookahead point
    auto lookahead_point_itr = std::find_if(
      trajectory_->points.begin() + closet_traj_point_idx, trajectory_->points.end(),
      [&](const TrajectoryPoint & point) {
        return std::hypot(point.pose.position.x - rear_x, point.pose.position.y - rear_y) >=
               lookahead_distance;
      });
    if (lookahead_point_itr == trajectory_->points.end()) {
      lookahead_point_itr = trajectory_->points.end() - 1;
    }
    double lookahead_point_x = lookahead_point_itr->pose.position.x;
    double lookahead_point_y = lookahead_point_itr->pose.position.y;

    geometry_msgs::msg::PointStamped lookahead_point_msg;
    lookahead_point_msg.header.stamp = get_clock()->now();
    lookahead_point_msg.header.frame_id = "map";
    lookahead_point_msg.point.x = lookahead_point_x;
    lookahead_point_msg.point.y = lookahead_point_y;
    lookahead_point_msg.point.z = closet_traj_point.pose.position.z;
    pub_lookahead_point_->publish(lookahead_point_msg);

    // calc steering angle for lateral control
    double alpha = std::atan2(lookahead_point_y - rear_y, lookahead_point_x - rear_x) -
                   tf2::getYaw(odom_pred->pose.pose.orientation);
    cmd.lateral.steering_tire_angle =
      steering_tire_angle_gain_ * std::atan2(2.0 * wheel_base_ * std::sin(alpha), lookahead_distance);
  }
  pub_cmd_->publish(cmd);
  cmd.lateral.steering_tire_angle /=  steering_tire_angle_gain_;
  pub_raw_cmd_->publish(cmd);
}

bool SimplePurePursuit::subscribeMessageAvailable()
{
  if (!odometry_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "odometry is not available");
    return false;
  }
  if (!trajectory_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "trajectory is not available");
    return false;
  }
  if (trajectory_->points.empty()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/,  "trajectory points is empty");
      return false;
    }
  return true;
}
}  // namespace simple_pure_pursuit

int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simple_pure_pursuit::SimplePurePursuit>());
  rclcpp::shutdown();
  return 0;
}
