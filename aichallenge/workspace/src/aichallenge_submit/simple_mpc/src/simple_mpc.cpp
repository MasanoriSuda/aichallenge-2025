#include "simple_mpc.hpp"

#include <motion_utils/motion_utils.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>
#include <motion_utils/trajectory/trajectory.hpp>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  // 必須!!

#include <algorithm>


namespace simple_mpc
{

using motion_utils::findNearestIndex;
using tier4_autoware_utils::calcLateralDeviation;
using tier4_autoware_utils::calcYawDeviation;

SimpleMpc::SimpleMpc()
: Node("simple_mpc")
  // initialize parameters
{
  pub_cmd_ = create_publisher<AckermannControlCommand>("/control/command/control_cmd", 1);
  pub_raw_cmd_ = create_publisher<AckermannControlCommand>("output/raw_control_cmd", 1);
  pub_lookahead_point_ = create_publisher<PointStamped>("/control/debug/lookahead_point", 1);

  const auto bv_qos = rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort();
  sub_kinematics_ = create_subscription<Odometry>(
    "/localization/kinematic_state", bv_qos, [this](const Odometry::SharedPtr msg) { odometry_ = msg; });
  sub_trajectory_ = create_subscription<Trajectory>(
    "/planning/scenario_planning/trajectory", bv_qos, [this](const Trajectory::SharedPtr msg) { trajectory_ = msg; });

  using namespace std::literals::chrono_literals;
  timer_ =
    rclcpp::create_timer(this, get_clock(), 10ms, std::bind(&SimpleMpc::onTimer, this));

    std::vector<double> x = {0, 1}, y = {0, 1};
    reference_path = std::make_shared<ReferencePath>(x, y, 0.2, 3.0, 3.0, false);

    car = std::make_shared<BicycleModel>(reference_path, 1.087, 1.45, 0.01);
  
    // odometry から OdometryInput を生成して車両にセット、暫定
    geometry_msgs::msg::Quaternion q;
    q.w = 1.0;  // 単位クォータニオン（回転なし）
    q.x = 0.0;
    q.y = 0.0;
    q.z = 0.0;

    OdometryInput odom_input;
    odom_input.x =  0.0;
    odom_input.y = 0.0;
    odom_input.yaw = tf2::getYaw(q);
    odom_input.v = 0.0;
    car->set_pose_from_odom(odom_input);

    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(3, 3);
    Q(0, 0) = 30.0;  // 横ずれは見るけど、50は過剰
    Q(1, 1) = 80.0;   // 進行方向ずれも見る（e_psi、これがないと蛇行する）
    Q(2, 2) = 0.0;   // tまたはsはそのままでOK

    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(2, 2);
    R(0, 0) = 200.0;   // 舵角の変化にコスト（抑制する）
    R(1, 1) = 3000.0;   // 今回vは固定 or補間なら無視でもOK

    Eigen::MatrixXd QN = Eigen::MatrixXd::Identity(3, 3);
    QN(0, 0) = 40.0;  // 横ずれは見るけど、50は過剰
    QN(1, 1) = 120.0;   // 進行方向ずれも見る（e_psi、これがないと蛇行する）
    QN(2, 2) = 0.0;   // tまたはsはそのままでOK

    double v_max = 35.0 / 3.6;//todo:debug
    double delta_max = 0.66;
    double ay_max = 10.0;

    std::map<std::string, Eigen::VectorXd> input_constraints = {
        {"umin", (Eigen::Vector2d() << 0.0, -std::tan(delta_max) / car->get_length()).finished()},
        {"umax", (Eigen::Vector2d() << v_max, std::tan(delta_max) / car->get_length()).finished()}
    };

    std::map<std::string, Eigen::VectorXd> state_constraints = {
        {"xmin", (Eigen::Vector3d() << -1e6, -1e6, -1e6).finished()},
        {"xmax", (Eigen::Vector3d() << 1e6, 1e6, 1e6).finished()}
    };

    mpc = std::make_shared<MPC>(car, N, Q, R, QN, state_constraints, input_constraints, ay_max);

    wp_speed = std::vector<double>(10000, 0.0);  // 固定 or 別途補間に置き換え

    prev_time_ = this->get_clock()->now();
    start_time_ = this->get_clock()->now();
    is_initialized = false;
    is_nearrest_0 = false;
    max_speed_ = 0.0;
    total_s = 0.0;
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

void SimpleMpc::onTimer()
{
  if (!subscribeMessageAvailable()) return;

  //double warm_time = 0.0;
  rclcpp::Time now = this->get_clock()->now();
  //double dt_ms = (now - prev_time_).nanoseconds() / 1e6;
  //RCLCPP_INFO(this->get_logger(), "周期: %.3f ms", dt_ms);
  if (is_initialized == false){
    start_time_ = this->get_clock()->now();
  }
  //double elapsed_sec = (now - start_time_).seconds();
  //RCLCPP_INFO(this->get_logger(), "経過時間: %.3f ms", elapsed_sec);

size_t idx = motion_utils::findNearestIndex(trajectory_->points, odometry_->pose.pose.position);

// 角度差で前後を判定
const auto & traj_pose = trajectory_->points[idx].pose;
const double odom_yaw = tf2::getYaw(odometry_->pose.pose.orientation);

// 前方方向ベクトルと目標点の差
const double dx = traj_pose.position.x - odometry_->pose.pose.position.x;
const double dy = traj_pose.position.y - odometry_->pose.pose.position.y;
const double heading_x = std::cos(odom_yaw);
const double heading_y = std::sin(odom_yaw);
const double dir = dx * heading_x + dy * heading_y;

if (dir < 0.0 && idx + 1 < trajectory_->points.size()) {
  idx += 1;
}

size_t idx_offset; 
double v_pref = std::hypot(odometry_->twist.twist.linear.x, odometry_->twist.twist.linear.y);
if(v_pref > 32.0 / 3.6){
  idx_offset = 7;
} else {
  idx_offset = 5;
}

idx = (idx+idx_offset) % trajectory_->points.size();
  size_t closet_traj_point_idx = idx;

  if (closet_traj_point_idx == 0){
    is_nearrest_0 = true;
  }

  //current pos
  double x = odometry_->pose.pose.position.x;
  double y = odometry_->pose.pose.position.y;
  double yaw = tf2::getYaw(odometry_->pose.pose.orientation);
  double pred_dt =0.15;
  double v_pref2 = std::hypot(odometry_->twist.twist.linear.x, odometry_->twist.twist.linear.y);

  if(v_pref2 < 32.0){
    pred_dt =0.125;
  }

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

  OdometryInput odom_input;
  odom_input.x = pred_x;
  odom_input.y = pred_y;
  odom_input.yaw = pred_yaw;
  double v_current = std::hypot(odometry_->twist.twist.linear.x, odometry_->twist.twist.linear.y);
  odom_input.v = v_current;
  car->set_pose_from_odom(odom_input);

  // trajectory から x, y を抽出
  std::vector<double> xs, ys;

  //odoと先頭トラジェクトリの中間を確保する
#if 0  
  if (elapsed_sec < warm_time) {
    const auto & pt = trajectory_->points[adj_index];
    double mid_x = 0.5 * (pt.pose.position.x + odom_input.x);
    double mid_y = 0.5 * (pt.pose.position.y + odom_input.y);
    xs.push_back(mid_x);
    ys.push_back(mid_y);
  } else {
    const auto & pt = trajectory_->points[closet_traj_point_idx];
    double mid_x = 0.5 * (pt.pose.position.x + odom_input.x);
    double mid_y = 0.5 * (pt.pose.position.y + odom_input.y);
    xs.push_back(mid_x);
    ys.push_back(mid_y);
  }
#endif  
  for (size_t i = 0; i < 20; ++i) {
    size_t idx = (closet_traj_point_idx + i) % trajectory_->points.size();
    if (idx == 0){
      continue;
    }
    const auto & pt = trajectory_->points[idx];
    xs.push_back(pt.pose.position.x);
    ys.push_back(pt.pose.position.y);
  }

  if (xs.size() == 0){
    const auto & pt = trajectory_->points[0];
    xs.push_back(pt.pose.position.x);
    ys.push_back(pt.pose.position.y);
  }

#if 0  
  double dx_tmp = odom_input.x - xs[0];
  double dy_tmp = odom_input.y - ys[0];
  double dist = std::sqrt(dx_tmp * dx_tmp + dy_tmp * dy_tmp);
  RCLCPP_INFO(this->get_logger(), "dist to traj.front: %.3f", dist);
#endif


  //RCLCPP_INFO(this->get_logger(), "Trajectory size = %zu", trajectory_->points.size());
#if 0
  RCLCPP_INFO(this->get_logger(),
      "closet_traj_point_idx = %ld",
      closet_traj_point_idx );
  RCLCPP_INFO(this->get_logger(),
      "MPC trajectory start: x= %.2f,y= %.2f",
      xs[0] , ys[0] );
#endif

double dt = 0.01;  // 制御周期 [s]
  const double acc_max =3.2;  // 加速 [m/s²]
// reference path を更新（仮名 update → 本当は set_points や reset_path など）
  reference_path->update_hoge(xs, ys,v_current,acc_max);  // ← 実装済み関数名に置き換えてください

  // 制御量計算
  Eigen::Vector2d u = mpc->get_control(odom_input, reference_path->get_all_waypoints());
  //car->drive(u);

  // --- ここに最小構成のkappa計算を埋め込み ---
  auto hypot2 = [](double a, double b){ return std::sqrt(a*a + b*b); };
  constexpr double EPS = 1e-9;
  const size_t n = xs.size();
  std::vector<double> kappa(n, 0.0);

  if (n >= 3) {
      for (size_t i = 1; i + 1 < n; ++i) {
          double x1 = xs[i-1], y1 = ys[i-1];
          double x2 = xs[i],   y2 = ys[i];
          double x3 = xs[i+1], y3 = ys[i+1];

          double a = hypot2(x2 - x1, y2 - y1);
          double b = hypot2(x3 - x2, y3 - y2);
          double c = hypot2(x3 - x1, y3 - y1);

          double cross = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
          double area2 = std::abs(cross);
          double denom = a * b * c;

          if (denom >= EPS) {
              double k = 2.0 * (area2 / denom);
              kappa[i] = (cross >= 0.0) ? +k : -k; // 左:+ 右:-
          }
      }
      // 端点コピー
      kappa[0] = kappa[1];
      kappa[n-1] = kappa[n-2];
  }
  //RCLCPP_INFO(this->get_logger(), "最高速度: %.3f m/s",  max_speed_);

  // コマンドメッセージ生成
  auto stamp = get_clock()->now();
  AckermannControlCommand cmd;
  cmd.stamp = stamp;
  cmd.lateral.stamp = stamp;
  cmd.longitudinal.stamp = stamp;

  double v = u[0];
  if(idx>165 && idx <175){
    v =31.0 /3.6;
  }
  #if 0
  if(u[1] < 0.1){
    u[1] = 0.0;
  }
  #endif
  double acc = (v - v_current) / dt;

  if (acc > acc_max) acc = acc_max;

  cmd.longitudinal.speed = v;
  cmd.longitudinal.acceleration = acc;
  cmd.lateral.steering_tire_angle = std::atan(u[1] * car->get_length());  // κL → δ

  #if 0
  double kappa = u[1];
  if(abs(kappa) < 0.25){
    kappa = 0.0;
  }
  double speed = u[0];
  double delta = std::atan(kappa * car->get_length());

  RCLCPP_INFO(this->get_logger(),
      "MPC control: speed = %.2f [m/s], kappa = %.3f [1/m], delta = %.3f [rad]",
      speed, kappa, delta);
  #endif

  if(total_s< -0.01){
    cmd.longitudinal.speed = v;
    cmd.longitudinal.acceleration = acc;
    cmd.lateral.steering_tire_angle = std::atan(0.0);  // κL → δ
    total_s +=0.01;
  }
  // パブリッシュ
  pub_cmd_->publish(cmd);
  // auto raw_cmd = cmd;
  //raw_cmd.lateral.steering_tire_angle /= steering_tire_angle_gain_;  // ゲイン未定義なら1.0でよい
  //pub_raw_cmd_->publish(raw_cmd);

  // ログ（任意）
  //log_.push_back({t_, odom_input.x, odom_input.y, car->get_s(), u[0], u[1]});
  //t_ += car->get_Ts();
}


bool SimpleMpc::subscribeMessageAvailable()
{
  if (!odometry_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "odometry is not available");
    return false;
  }
  if (!trajectory_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "trajectory is not available");
    return false;
  }
  return true;
}
}  // namespace simple_mpc

int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<simple_mpc::SimpleMpc>());
  rclcpp::shutdown();
  return 0;
}