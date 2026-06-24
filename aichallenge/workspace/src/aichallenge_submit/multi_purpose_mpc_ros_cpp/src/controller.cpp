#include "multi_purpose_mpc_ros_cpp/controller.hpp"

#include "multi_purpose_mpc_ros_cpp/osqp_mpc.hpp"
#include "multi_purpose_mpc_ros_cpp/ref_velocity_config.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <string>

namespace multi_purpose_mpc_ros_cpp
{

namespace
{

template<typename T>
std::array<T, 3> as_array3(const YAML::Node & node)
{
  if (!node || !node.IsSequence() || node.size() != 3) {
    throw std::runtime_error("Expected a YAML sequence with 3 elements.");
  }
  return {node[0].as<T>(), node[1].as<T>(), node[2].as<T>()};
}

template<typename T>
std::array<T, 2> as_array2(const YAML::Node & node)
{
  if (!node || !node.IsSequence() || node.size() != 2) {
    throw std::runtime_error("Expected a YAML sequence with 2 elements.");
  }
  return {node[0].as<T>(), node[1].as<T>()};
}

}  // namespace

double kmh_to_mps(double kmh)
{
  return kmh / 3.6;
}

double deg_to_rad(double deg)
{
  return deg * M_PI / 180.0;
}

std::optional<LapStatusUpdate> LapTracker::update(
  int laps, double lap_time_sec, std::size_t total_fallbacks)
{
  if (!initialized_) {
    current_lap_ = laps == 0 ? 1 : laps;
    initialized_ = true;
  }

  std::optional<LapStatusUpdate> update_result;
  if (laps > current_lap_) {
    update_result = LapStatusUpdate{
      current_lap_,
      last_lap_time_sec_,
      total_fallbacks - last_logged_fallback_count_,
      total_fallbacks};
    current_lap_ = laps;
    last_logged_fallback_count_ = total_fallbacks;
  }

  last_lap_time_sec_ = lap_time_sec;
  return update_result;
}

ControllerConfig load_controller_config(const std::string & config_path, const std::string & ref_vel_path)
{
  const auto root = YAML::LoadFile(config_path);

  ControllerConfig config;
  config.config_path = config_path;
  config.ref_vel_path = ref_vel_path;

  config.save_config = root["common"]["save_config"].as<bool>(true);
  config.animation_enabled = root["sim_logger"]["animation_enabled"].as<bool>(false);
  config.map_yaml_path = root["map"]["yaml_path"].as<std::string>();

  const auto reference_path = root["reference_path"];
  config.reference_path.update_by_topic = reference_path["update_by_topic"].as<bool>();
  config.reference_path.csv_path = reference_path["csv_path"].as<std::string>();
  config.reference_path.resolution = reference_path["resolution"].as<double>();
  config.reference_path.smoothing_distance = reference_path["smoothing_distance"].as<int>();
  config.reference_path.max_width = reference_path["max_width"].as<double>();
  config.reference_path.circular = reference_path["circular"].as<bool>();
  config.reference_path.use_path_constraints_topic = reference_path["use_path_constraints_topic"].as<bool>();
  config.reference_path.use_border_cells_topic = reference_path["use_border_cells_topic"].as<bool>();

  config.bicycle_model.length = root["bicycle_model"]["length"].as<double>();
  config.bicycle_model.width = root["bicycle_model"]["width"].as<double>();

  const auto mpc = root["mpc"];
  config.mpc.N = mpc["N"].as<int>();
  config.mpc.q = as_array3<double>(mpc["Q"]);
  config.mpc.r = as_array2<double>(mpc["R"]);
  config.mpc.qn = as_array3<double>(mpc["QN"]);
  config.mpc.v_max_kmph = mpc["v_max"].as<double>();
  config.mpc.a_min = mpc["a_min"].as<double>();
  config.mpc.a_max = mpc["a_max"].as<double>();
  config.mpc.ay_max = mpc["ay_max"].as<double>();
  config.mpc.delta_max_deg = mpc["delta_max_deg"].as<double>();
  config.mpc.steer_rate_max = mpc["steer_rate_max"].as<double>();
  config.mpc.control_rate_hz = mpc["control_rate"].as<double>();
  config.mpc.steering_tire_angle_gain_var = mpc["steering_tire_angle_gain_var"].as<double>();
  config.mpc.accel_low_pass_gain = mpc["accel_low_pass_gain"].as<double>();
  config.mpc.steer_low_pass_gain = mpc["steer_low_pass_gain"].as<double>();
  config.mpc.wp_id_offset = mpc["wp_id_offset"].as<int>();
  config.mpc.use_max_kappa_pred = mpc["use_max_kappa_pred"].as<bool>();

  return config;
}

CppMpcControllerNode::CppMpcControllerNode(const rclcpp::NodeOptions & options)
: Node("mpc_controller", options)
{
  declare_parameter("config_path", "");
  declare_parameter("ref_vel_path", "");
  declare_parameter("use_boost_acceleration", false);
  declare_parameter("use_obstacle_avoidance", false);
  declare_parameter("use_stats", false);

  const auto config_path = get_parameter("config_path").as_string();
  const auto ref_vel_path = get_parameter("ref_vel_path").as_string();
  if (config_path.empty()) {
    throw std::runtime_error("The 'config_path' parameter must be set.");
  }

  get_parameter("use_sim_time", use_sim_time_);
  use_boost_acceleration_ = get_parameter("use_boost_acceleration").as_bool();
  use_obstacle_avoidance_ = get_parameter("use_obstacle_avoidance").as_bool();
  use_stats_ = get_parameter("use_stats").as_bool();

  config_ = load_controller_config(config_path, ref_vel_path);
  if (config_.reference_path.update_by_topic) {
    throw std::runtime_error(
            "multi_purpose_mpc_ros_cpp does not support reference_path.update_by_topic.");
  }
  if (config_.reference_path.use_border_cells_topic) {
    throw std::runtime_error(
            "multi_purpose_mpc_ros_cpp does not support reference_path.use_border_cells_topic.");
  }
  if (!ref_vel_path.empty()) {
    ref_velocity_config_ = std::make_unique<RefVelocityConfig>(ref_vel_path);
  }
  mpc_core_ = std::make_unique<OsqpMpcCore>(config_);

  setup_runtime_parameters();
  setup_interfaces();

  const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / config_.mpc.control_rate_hz));
  timer_ = create_wall_timer(timer_period, std::bind(&CppMpcControllerNode::on_timer, this));
}

CppMpcControllerNode::~CppMpcControllerNode() = default;

void CppMpcControllerNode::setup_runtime_parameters()
{
  declare_parameter("v_max", config_.mpc.v_max_kmph);
  declare_parameter("steering_tire_angle_gain_var", config_.mpc.steering_tire_angle_gain_var);
  declare_parameter("Q0", config_.mpc.q[0]);
  declare_parameter("Q1", config_.mpc.q[1]);
  declare_parameter("Q2", config_.mpc.q[2]);
  declare_parameter("R0", config_.mpc.r[0]);
  declare_parameter("R1", config_.mpc.r[1]);
  declare_parameter("QN0", config_.mpc.qn[0]);
  declare_parameter("QN1", config_.mpc.qn[1]);
  declare_parameter("QN2", config_.mpc.qn[2]);
  declare_parameter("ay_max", config_.mpc.ay_max);
  declare_parameter("accel_low_pass_gain", config_.mpc.accel_low_pass_gain);
  declare_parameter("steer_low_pass_gain", config_.mpc.steer_low_pass_gain);
  declare_parameter("wp_id_offset", config_.mpc.wp_id_offset);

  parameter_callback_handle_ = add_on_set_parameters_callback(
    std::bind(&CppMpcControllerNode::on_parameter_update, this, std::placeholders::_1));
}

void CppMpcControllerNode::setup_interfaces()
{
  if (use_boost_acceleration_) {
    boost_command_pub_ = create_publisher<AckermannControlBoostCommand>("/boost_commander/command", 1);
  } else {
    command_pub_ = create_publisher<AckermannControlCommand>("/control/command/control_cmd", 1);
    command_raw_pub_ = create_publisher<AckermannControlCommand>("/control/command/control_cmd_raw", 1);
  }

  odom_sub_ = create_subscription<Odometry>(
    "/localization/kinematic_state", 1,
    [this](const Odometry::SharedPtr msg) { odom_ = msg; });

  control_mode_request_sub_ = create_subscription<std_msgs::msg::Bool>(
    "control/control_mode_request_topic", 1,
    std::bind(&CppMpcControllerNode::on_control_mode_request, this, std::placeholders::_1));

  stop_request_sub_ = create_subscription<std_msgs::msg::Empty>(
    "/control/mpc/stop_request", 1,
    std::bind(&CppMpcControllerNode::on_stop_request, this, std::placeholders::_1));

  if (use_sim_time_) {
    awsim_status_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/awsim/status", 1,
      std::bind(&CppMpcControllerNode::on_awsim_status, this, std::placeholders::_1));
    condition_sub_ = create_subscription<std_msgs::msg::Int32>(
      "/aichallenge/pitstop/condition", 1,
      std::bind(&CppMpcControllerNode::on_condition, this, std::placeholders::_1));
  }

  if (requires_path_constraints()) {
    const auto path_constraints_qos = rclcpp::QoS(1).reliable().transient_local();
    path_constraints_sub_ = create_subscription<PathConstraints>(
      "/path_constraints_provider/path_constraints", path_constraints_qos,
      std::bind(&CppMpcControllerNode::on_path_constraints, this, std::placeholders::_1));
  }
}

bool CppMpcControllerNode::requires_path_constraints() const
{
  return use_obstacle_avoidance_ && config_.reference_path.use_path_constraints_topic;
}

void CppMpcControllerNode::on_stop_request(const std_msgs::msg::Empty::SharedPtr)
{
  if (enable_control_) {
    RCLCPP_WARN(get_logger(), "Stop request received");
    enable_control_ = false;
  }
}

void CppMpcControllerNode::on_control_mode_request(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data && !enable_control_) {
    RCLCPP_INFO(get_logger(), "Control mode request received");
    enable_control_ = true;
  }
}

void CppMpcControllerNode::on_path_constraints(const PathConstraints::SharedPtr msg)
{
  path_constraint_upper_bounds_.assign(msg->upper_bounds.begin(), msg->upper_bounds.end());
  path_constraint_lower_bounds_.assign(msg->lower_bounds.begin(), msg->lower_bounds.end());
  path_constraint_rows_ = msg->rows;
  path_constraint_cols_ = msg->cols;
  path_constraints_received_ = true;
  mpc_core_->set_path_constraints(
    path_constraint_upper_bounds_,
    path_constraint_lower_bounds_,
    path_constraint_rows_,
    path_constraint_cols_);
}

void CppMpcControllerNode::on_awsim_status(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 3) {
    return;
  }

  const auto laps = static_cast<int>(msg->data[1]);
  const auto lap_time_sec = static_cast<double>(msg->data[2]);
  const auto update = lap_tracker_.update(laps, lap_time_sec, last_total_fallback_count_);
  if (!update) {
    return;
  }

  RCLCPP_INFO(
    get_logger(),
    "Lap %d completed! Lap time: %.3f s, lap_fallbacks=%zu, total_fallbacks=%zu",
    update->completed_lap,
    update->lap_time_sec,
    update->lap_fallbacks,
    update->total_fallbacks);
}

void CppMpcControllerNode::on_condition(const std_msgs::msg::Int32::SharedPtr msg)
{
  if (!last_condition_) {
    last_condition_ = msg->data;
    return;
  }

  if (msg->data - *last_condition_ > 30) {
    RCLCPP_WARN(get_logger(), "Collision detected!");
  }
  last_condition_ = msg->data;
}

rcl_interfaces::msg::SetParametersResult CppMpcControllerNode::on_parameter_update(
  const std::vector<rclcpp::Parameter> & parameters)
{
  for (const auto & param : parameters) {
    if (param.get_name() == "v_max" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      config_.mpc.v_max_kmph = param.as_double();
      mpc_core_->update_v_max(kmh_to_mps(config_.mpc.v_max_kmph));
      mpc_core_->set_reference_velocity_all(kmh_to_mps(config_.mpc.v_max_kmph));
    } else if (
      param.get_name() == "steering_tire_angle_gain_var" &&
      param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
    {
      config_.mpc.steering_tire_angle_gain_var = param.as_double();
    } else if (param.get_name() == "Q0") {
      config_.mpc.q[0] = param.as_double();
      mpc_core_->update_q(config_.mpc.q);
    } else if (param.get_name() == "Q1") {
      config_.mpc.q[1] = param.as_double();
      mpc_core_->update_q(config_.mpc.q);
    } else if (param.get_name() == "Q2") {
      config_.mpc.q[2] = param.as_double();
      mpc_core_->update_q(config_.mpc.q);
    } else if (param.get_name() == "R0") {
      config_.mpc.r[0] = param.as_double();
      mpc_core_->update_r(config_.mpc.r);
    } else if (param.get_name() == "R1") {
      config_.mpc.r[1] = param.as_double();
      mpc_core_->update_r(config_.mpc.r);
    } else if (param.get_name() == "QN0") {
      config_.mpc.qn[0] = param.as_double();
      mpc_core_->update_qn(config_.mpc.qn);
    } else if (param.get_name() == "QN1") {
      config_.mpc.qn[1] = param.as_double();
      mpc_core_->update_qn(config_.mpc.qn);
    } else if (param.get_name() == "QN2") {
      config_.mpc.qn[2] = param.as_double();
      mpc_core_->update_qn(config_.mpc.qn);
    } else if (param.get_name() == "ay_max") {
      config_.mpc.ay_max = param.as_double();
      mpc_core_->update_ay_max(config_.mpc.ay_max);
    } else if (param.get_name() == "accel_low_pass_gain") {
      config_.mpc.accel_low_pass_gain = param.as_double();
    } else if (param.get_name() == "steer_low_pass_gain") {
      config_.mpc.steer_low_pass_gain = param.as_double();
    } else if (param.get_name() == "wp_id_offset") {
      config_.mpc.wp_id_offset = static_cast<int>(param.as_int());
      mpc_core_->update_wp_id_offset(config_.mpc.wp_id_offset);
    }
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

double CppMpcControllerNode::yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  const auto sqx = q.x * q.x;
  const auto sqy = q.y * q.y;
  const auto sqz = q.z * q.z;
  const auto sqw = q.w * q.w;
  const auto norm = sqx + sqy + sqz + sqw;

  if (norm <= 1.0e-12) {
    return 0.0;
  }

  const auto sarg = -2.0 * (q.x * q.z - q.w * q.y) / norm;
  if (sarg <= -0.99999) {
    return -2.0 * std::atan2(q.y, q.x);
  }
  if (sarg >= 0.99999) {
    return 2.0 * std::atan2(q.y, q.x);
  }
  return std::atan2(2.0 * (q.x * q.y + q.w * q.z), sqw + sqx - sqy - sqz);
}

double CppMpcControllerNode::clamp(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(value, max_value));
}

CppMpcControllerNode::AckermannControlCommand CppMpcControllerNode::create_raw_control_command(
  const rclcpp::Time & stamp, const std::array<double, 2> & u, double acc) const
{
  AckermannControlCommand cmd;
  cmd.stamp = stamp;
  cmd.longitudinal.stamp = stamp;
  cmd.longitudinal.speed = u[0];
  cmd.longitudinal.acceleration = acc;
  cmd.lateral.stamp = stamp;
  cmd.lateral.steering_tire_angle = u[1];
  // hack デバッグ
  //if (abs(u[1]) < deg_to_rad(3.0)) {
  //  cmd.lateral.steering_tire_angle = 0.0;
 // }

  cmd.lateral.steering_tire_rotation_rate = 2.0;
  return cmd;
}

void CppMpcControllerNode::publish_control_command(
  const rclcpp::Time & stamp, const std::array<double, 2> & u, double acc, bool boost_mode)
{
  auto raw_cmd = create_raw_control_command(stamp, u, acc);

  if (!use_boost_acceleration_ && command_raw_pub_) {
    command_raw_pub_->publish(raw_cmd);
  }

  raw_cmd.lateral.steering_tire_angle *= config_.mpc.steering_tire_angle_gain_var;

  if (use_boost_acceleration_ && boost_command_pub_) {
    AckermannControlBoostCommand boost_cmd;
    boost_cmd.command = raw_cmd;
    boost_cmd.boost_mode = boost_mode;
    boost_command_pub_->publish(boost_cmd);
    return;
  }

  if (command_pub_) {
    command_pub_->publish(raw_cmd);
  }
}

void CppMpcControllerNode::on_timer()
{
  if (!odom_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "odometry is not available");
    return;
  }

  if (requires_path_constraints() && !path_constraints_received_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000, "path constraints are not available");
    return;
  }

  const auto now = this->now();
  const auto dt = last_control_time_.nanoseconds() == 0 ?
    0.0 : std::max(0.0, (now - last_control_time_).seconds());
  last_control_time_ = now;

  const auto & pose = odom_->pose.pose.position;
  const auto yaw = yaw_from_quaternion(odom_->pose.pose.orientation);
  const auto v = odom_->twist.twist.linear.x;
  const auto yaw_rate = odom_->twist.twist.angular.z;

  // 遅延補償: τ分だけ前方外挿
  constexpr double tau = 0.3;  // seconds (GNSS delay)
  const auto compensated_x = pose.x + v * std::cos(yaw) * tau;
  const auto compensated_y = pose.y + v * std::sin(yaw) * tau;
  const auto compensated_yaw = yaw + yaw_rate * tau;

  mpc_core_->update_states(compensated_x, compensated_y, compensated_yaw);

  auto solve_output = mpc_core_->solve(use_stats_);
  last_total_fallback_count_ = solve_output.fallback_count_total;
  if (solve_output.used_fallback) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "OSQP solve failed, using fallback control. total_fallbacks=%zu depth=%zu status=%ld exit=%ld iter=%ld msg=%s",
      solve_output.fallback_count_total,
      solve_output.fallback_depth,
      static_cast<long>(solve_output.solver_status),
      static_cast<long>(solve_output.solver_exit_flag),
      static_cast<long>(solve_output.solver_iterations),
      solve_output.solver_status_message.c_str());
  }

  auto u = solve_output.control;
  const auto max_delta = solve_output.max_delta_rad;

  if (ref_velocity_config_ && !ref_velocity_config_->empty()) {
    const auto ref_vel_mps = std::min(
      kmh_to_mps(ref_velocity_config_->get_ref_vel_kmph(mpc_core_->control_waypoint_id())),
      kmh_to_mps(config_.mpc.v_max_kmph));
    mpc_core_->update_v_max(ref_vel_mps);
    mpc_core_->set_reference_velocity_all(ref_vel_mps);
  }

  if (!enable_control_) {
    if (last_u_[0] < 0.5) {
      u[0] = 0.0;
    } else {
      const auto decel_v = last_u_[0] + config_.mpc.a_min * dt;
      u[0] = clamp(decel_v, 0.0, kmh_to_mps(config_.mpc.v_max_kmph));
    }
  }

  const auto current_speed = odom_->twist.twist.linear.x;
  double acc = 0.0;
  bool boost_mode = false;

  if (use_boost_acceleration_) {
    if (
      std::abs(current_speed) > kmh_to_mps(44.0) ||
      (std::abs(current_speed) > kmh_to_mps(38.0) && std::abs(max_delta) > deg_to_rad(12.0)))
    {
      boost_mode = false;
      acc = config_.mpc.a_min / 3.0 * 2.0;
    } else if (
      std::abs(current_speed) > kmh_to_mps(41.0) ||
      std::abs(u[1]) > deg_to_rad(10.0))
    {
      boost_mode = false;
      acc = config_.mpc.a_max;
    } else {
      boost_mode = true;
      acc = 500.0;
    }
  } else {
    acc = 100.0 * (u[0] - current_speed);
    acc = clamp(acc, config_.mpc.a_min, config_.mpc.a_max);
  }

  acc = last_acc_ + (acc - last_acc_) * config_.mpc.accel_low_pass_gain;
  u[1] = last_u_[1] + (u[1] - last_u_[1]) * config_.mpc.steer_low_pass_gain;

  last_acc_ = acc;
  last_u_ = u;

  // hack: デバッグ用に3.2固定
  if (acc > 0.0) {
    acc = 3.2;
  }

  mpc_core_->drive(current_speed, u[1]);
  publish_control_command(now, u, acc, boost_mode);
}

}  // namespace multi_purpose_mpc_ros_cpp
