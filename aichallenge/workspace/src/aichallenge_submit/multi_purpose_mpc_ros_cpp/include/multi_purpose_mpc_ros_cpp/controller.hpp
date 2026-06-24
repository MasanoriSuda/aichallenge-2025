#ifndef MULTI_PURPOSE_MPC_ROS_CPP__CONTROLLER_HPP_
#define MULTI_PURPOSE_MPC_ROS_CPP__CONTROLLER_HPP_

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <multi_purpose_mpc_ros_msgs/msg/ackermann_control_boost_command.hpp>
#include <multi_purpose_mpc_ros_msgs/msg/path_constraints.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>

#include <array>
#include <optional>
#include <memory>
#include <string>
#include <vector>

namespace multi_purpose_mpc_ros_cpp
{

struct ReferencePathConfig
{
  bool update_by_topic{};
  std::string csv_path;
  double resolution{};
  int smoothing_distance{};
  double max_width{};
  bool circular{};
  bool use_path_constraints_topic{};
  bool use_border_cells_topic{};
};

struct BicycleModelConfig
{
  double length{};
  double width{};
};

struct MpcConfig
{
  int N{};
  std::array<double, 3> q{};
  std::array<double, 2> r{};
  std::array<double, 3> qn{};
  double v_max_kmph{};
  double a_min{};
  double a_max{};
  double ay_max{};
  double delta_max_deg{};
  double steer_rate_max{};
  double control_rate_hz{};
  double steering_tire_angle_gain_var{};
  double accel_low_pass_gain{};
  double steer_low_pass_gain{};
  int wp_id_offset{};
  bool use_max_kappa_pred{};
};

struct ControllerConfig
{
  bool save_config{};
  bool animation_enabled{};
  std::string map_yaml_path;
  ReferencePathConfig reference_path;
  BicycleModelConfig bicycle_model;
  MpcConfig mpc;
  std::string config_path;
  std::string ref_vel_path;
};

ControllerConfig load_controller_config(const std::string & config_path, const std::string & ref_vel_path);
double kmh_to_mps(double kmh);
double deg_to_rad(double deg);

class RefVelocityConfig;
class OsqpMpcCore;
struct SolveOutput;

struct LapStatusUpdate
{
  int completed_lap{};
  double lap_time_sec{};
  std::size_t lap_fallbacks{};
  std::size_t total_fallbacks{};
};

class LapTracker
{
public:
  std::optional<LapStatusUpdate> update(int laps, double lap_time_sec, std::size_t total_fallbacks);

private:
  bool initialized_{false};
  int current_lap_{1};
  double last_lap_time_sec_{0.0};
  std::size_t last_logged_fallback_count_{0};
};

class CppMpcControllerNode : public rclcpp::Node
{
public:
  explicit CppMpcControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CppMpcControllerNode() override;

private:
  using AckermannControlCommand = autoware_auto_control_msgs::msg::AckermannControlCommand;
  using AckermannControlBoostCommand = multi_purpose_mpc_ros_msgs::msg::AckermannControlBoostCommand;
  using Odometry = nav_msgs::msg::Odometry;
  using PathConstraints = multi_purpose_mpc_ros_msgs::msg::PathConstraints;

  void setup_interfaces();
  void setup_runtime_parameters();
  void on_timer();
  void on_stop_request(const std_msgs::msg::Empty::SharedPtr msg);
  void on_control_mode_request(const std_msgs::msg::Bool::SharedPtr msg);
  void on_path_constraints(const PathConstraints::SharedPtr msg);
  void on_awsim_status(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void on_condition(const std_msgs::msg::Int32::SharedPtr msg);
  bool requires_path_constraints() const;
  rcl_interfaces::msg::SetParametersResult on_parameter_update(
    const std::vector<rclcpp::Parameter> & parameters);

  AckermannControlCommand create_raw_control_command(
    const rclcpp::Time & stamp, const std::array<double, 2> & u, double acc) const;
  void publish_control_command(
    const rclcpp::Time & stamp, const std::array<double, 2> & u, double acc, bool boost_mode);

  static double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q);
  static double clamp(double value, double min_value, double max_value);

  ControllerConfig config_;
  bool use_sim_time_{false};
  bool use_boost_acceleration_{false};
  bool use_obstacle_avoidance_{false};
  bool use_stats_{false};
  bool enable_control_{true};

  std::unique_ptr<RefVelocityConfig> ref_velocity_config_;
  std::unique_ptr<OsqpMpcCore> mpc_core_;

  rclcpp::Publisher<AckermannControlCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr command_raw_pub_;
  rclcpp::Publisher<AckermannControlBoostCommand>::SharedPtr boost_command_pub_;

  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr control_mode_request_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_request_sub_;
  rclcpp::Subscription<PathConstraints>::SharedPtr path_constraints_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr awsim_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr condition_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  Odometry::SharedPtr odom_;
  std::vector<double> path_constraint_upper_bounds_;
  std::vector<double> path_constraint_lower_bounds_;
  int path_constraint_rows_{0};
  int path_constraint_cols_{0};
  bool path_constraints_received_{false};

  std::array<double, 2> last_u_{0.0, 0.0};
  double last_acc_{0.0};
  rclcpp::Time last_control_time_{0, 0, RCL_ROS_TIME};
  std::size_t last_total_fallback_count_{0};
  LapTracker lap_tracker_;
  std::optional<int> last_condition_;
};

}  // namespace multi_purpose_mpc_ros_cpp

#endif  // MULTI_PURPOSE_MPC_ROS_CPP__CONTROLLER_HPP_
