#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <multi_purpose_mpc_ros/awsim_boost_start_dash.hpp>
#include <multi_purpose_mpc_ros/mpc_velocity_limit.hpp>
#include <multi_purpose_mpc_ros/path_core.hpp>
#include <multi_purpose_mpc_ros/recovery_footprint.hpp>
#include <multi_purpose_mpc_ros/recovery_mpc.hpp>
#include <multi_purpose_mpc_ros/start_grid_grace.hpp>
#include <multi_purpose_mpc_ros/stuck_recovery_core.hpp>
#include <multi_purpose_mpc_ros/v2x_overtake_core.hpp>
#include <multi_purpose_mpc_ros_msgs/msg/ackermann_control_boost_command.hpp>
#include <multi_purpose_mpc_ros_msgs/msg/border_cells.hpp>
#include <multi_purpose_mpc_ros_msgs/msg/path_constraints.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <v2x_msgs/msg/v2_x_vehicle_position_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core.hpp>
#include <osqp.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace multi_purpose_mpc_ros
{
namespace
{

using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_vehicle_msgs::msg::GearCommand;
using autoware_auto_vehicle_msgs::msg::GearReport;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose2D;
using geometry_msgs::msg::Quaternion;
using geometry_msgs::msg::Vector3;
using multi_purpose_mpc_ros_msgs::msg::AckermannControlBoostCommand;
using multi_purpose_mpc_ros_msgs::msg::BorderCells;
using multi_purpose_mpc_ros_msgs::msg::PathConstraints;
using nav_msgs::msg::Odometry;
using std_msgs::msg::Bool;
using std_msgs::msg::ColorRGBA;
using std_msgs::msg::Empty;
using std_msgs::msg::Float32MultiArray;
using std_msgs::msg::Int32;
using std_msgs::msg::String;
using v2x_msgs::msg::V2XVehiclePositionArray;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;
using SteadyClock = std::chrono::steady_clock;
namespace path_core = ::multi_purpose_mpc_ros::path_core;
namespace awsim_boost = ::multi_purpose_mpc_ros::awsim_boost;
namespace mpc_velocity_limit = ::multi_purpose_mpc_ros::mpc_velocity_limit;
namespace overtake_core = ::multi_purpose_mpc_ros::v2x_overtake_core;
namespace recovery_footprint = ::multi_purpose_mpc_ros::recovery_footprint;
namespace recovery_mpc = ::multi_purpose_mpc_ros::recovery_mpc;
namespace start_grid_grace = ::multi_purpose_mpc_ros::start_grid_grace;
namespace stuck_recovery = ::multi_purpose_mpc_ros::stuck_recovery;

constexpr double kEps = 1e-12;
constexpr double kPi = 3.14159265358979323846;

double clip(const double value, const double min_value, const double max_value)
{
  return std::min(std::max(value, min_value), max_value);
}

double smoothstep(const double value)
{
  const double t = clip(value, 0.0, 1.0);
  return t * t * (3.0 - 2.0 * t);
}

double wrap_to_pi(const double angle)
{
  return path_core::wrap_to_pi(angle);
}

double kmh_to_m_per_sec(const double kmh)
{
  return kmh / 3.6;
}

stuck_recovery::Gear recovery_gear_from_report(const std::uint8_t report)
{
  if (report == GearReport::NEUTRAL) {
    return stuck_recovery::Gear::Neutral;
  }
  if (report == GearReport::DRIVE) {
    return stuck_recovery::Gear::Drive;
  }
  if (report == GearReport::REVERSE) {
    return stuck_recovery::Gear::Reverse;
  }
  return stuck_recovery::Gear::Unknown;
}

std::optional<std::uint8_t> gear_command_value(const stuck_recovery::Gear gear)
{
  switch (gear) {
    case stuck_recovery::Gear::Neutral:
      return GearCommand::NEUTRAL;
    case stuck_recovery::Gear::Drive:
      return GearCommand::DRIVE;
    case stuck_recovery::Gear::Reverse:
      return GearCommand::REVERSE;
    case stuck_recovery::Gear::NoCommand:
    case stuck_recovery::Gear::Unknown:
      return std::nullopt;
  }
  return std::nullopt;
}

std::optional<int> ros_domain_id_from_env()
{
  const char * value = std::getenv("ROS_DOMAIN_ID");
  if (value == nullptr) {
    return std::nullopt;
  }
  try {
    std::size_t pos = 0;
    const int domain_id = std::stoi(value, &pos);
    if (pos == std::string(value).size()) {
      return domain_id;
    }
  } catch (const std::exception &) {
  }
  return std::nullopt;
}

double stamp_to_seconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

double steady_seconds(const SteadyClock::time_point time_point)
{
  return std::chrono::duration<double>(time_point.time_since_epoch()).count();
}

std::vector<std::string> split_csv_line(const std::string & line)
{
  std::vector<std::string> out;
  std::stringstream ss(line);
  std::string item;
  while (std::getline(ss, item, ',')) {
    out.push_back(item);
  }
  return out;
}

std::unordered_map<std::string, std::vector<double>> read_csv_columns(const std::string & path)
{
  std::ifstream ifs(path);
  if (!ifs.is_open()) {
    throw std::runtime_error("File not found: " + path);
  }

  std::string header_line;
  if (!std::getline(ifs, header_line)) {
    throw std::runtime_error("CSV is empty: " + path);
  }

  const auto headers = split_csv_line(header_line);
  std::unordered_map<std::string, std::vector<double>> columns;
  for (const auto & header : headers) {
    columns.emplace(header, std::vector<double>{});
  }

  std::string line;
  while (std::getline(ifs, line)) {
    if (line.empty()) {
      continue;
    }
    const auto values = split_csv_line(line);
    for (std::size_t i = 0; i < headers.size() && i < values.size(); ++i) {
      try {
        columns.at(headers[i]).push_back(std::stod(values[i]));
      } catch (const std::exception &) {
        columns.at(headers[i]).push_back(0.0);
      }
    }
  }

  return columns;
}

struct OsqpSolveResult
{
  Eigen::VectorXd solution;
  c_int status{};
  double max_constraint_violation{};
};

struct OsqpSolveOutcome
{
  std::optional<OsqpSolveResult> result;
  std::string failure_detail;
};

OsqpSolveOutcome osqp_failure(std::string detail)
{
  return OsqpSolveOutcome{std::nullopt, std::move(detail)};
}

std::string describe_osqp_info(const OSQPInfo * info)
{
  if (info == nullptr) {
    return "info=unavailable";
  }
  std::ostringstream detail;
  detail << "status=" << info->status
         << ", status_val=" << info->status_val
         << ", iter=" << info->iter
         << ", pri_res=" << info->pri_res
         << ", dua_res=" << info->dua_res;
  return detail.str();
}

struct CscDeleter
{
  void operator()(csc * matrix) const noexcept
  {
    c_free(matrix);
  }
};

struct OsqpWorkspaceDeleter
{
  void operator()(OSQPWorkspace * workspace) const noexcept
  {
    if (workspace != nullptr) {
      static_cast<void>(osqp_cleanup(workspace));
    }
  }
};

bool sparse_values_are_finite(const Eigen::SparseMatrix<double> & matrix)
{
  for (int i = 0; i < matrix.nonZeros(); ++i) {
    if (!std::isfinite(matrix.valuePtr()[i])) {
      return false;
    }
  }
  return true;
}

OsqpSolveOutcome solve_osqp(
  Eigen::SparseMatrix<double> P, Eigen::SparseMatrix<double> A, const Eigen::VectorXd & q,
  const Eigen::VectorXd & l, const Eigen::VectorXd & u)
{
  P.makeCompressed();
  A.makeCompressed();

  if (
    P.rows() <= 0 || P.rows() != P.cols() || q.size() != P.cols() || A.rows() <= 0 ||
    A.cols() != P.cols() || A.rows() != l.size() || l.size() != u.size() ||
    !sparse_values_are_finite(P) || !sparse_values_are_finite(A) || !q.allFinite())
  {
    return osqp_failure("stage=validation, reason=invalid dimensions or non-finite matrix/vector");
  }
  for (int i = 0; i < l.size(); ++i) {
    if (std::isnan(l[i]) || std::isnan(u[i]) || l[i] > u[i]) {
      std::ostringstream detail;
      detail << "stage=validation, reason=invalid bounds, index=" << i
             << ", lower=" << l[i] << ", upper=" << u[i];
      return osqp_failure(detail.str());
    }
  }

  std::vector<c_float> p_x(P.nonZeros());
  std::vector<c_int> p_i(P.nonZeros());
  std::vector<c_int> p_p(P.cols() + 1);
  for (int i = 0; i < P.nonZeros(); ++i) {
    p_x[i] = static_cast<c_float>(P.valuePtr()[i]);
    p_i[i] = static_cast<c_int>(P.innerIndexPtr()[i]);
  }
  for (int i = 0; i < P.cols() + 1; ++i) {
    p_p[i] = static_cast<c_int>(P.outerIndexPtr()[i]);
  }

  std::vector<c_float> a_x(A.nonZeros());
  std::vector<c_int> a_i(A.nonZeros());
  std::vector<c_int> a_p(A.cols() + 1);
  for (int i = 0; i < A.nonZeros(); ++i) {
    a_x[i] = static_cast<c_float>(A.valuePtr()[i]);
    a_i[i] = static_cast<c_int>(A.innerIndexPtr()[i]);
  }
  for (int i = 0; i < A.cols() + 1; ++i) {
    a_p[i] = static_cast<c_int>(A.outerIndexPtr()[i]);
  }

  std::vector<c_float> q_data(q.size());
  std::vector<c_float> l_data(l.size());
  std::vector<c_float> u_data(u.size());
  for (int i = 0; i < q.size(); ++i) {
    q_data[i] = static_cast<c_float>(q[i]);
  }
  for (int i = 0; i < l.size(); ++i) {
    l_data[i] = static_cast<c_float>(l[i]);
    u_data[i] = static_cast<c_float>(u[i]);
  }

  std::unique_ptr<csc, CscDeleter> P_csc(csc_matrix(
      static_cast<c_int>(P.rows()), static_cast<c_int>(P.cols()),
      static_cast<c_int>(P.nonZeros()), p_x.data(), p_i.data(), p_p.data()));
  std::unique_ptr<csc, CscDeleter> A_csc(csc_matrix(
      static_cast<c_int>(A.rows()), static_cast<c_int>(A.cols()),
      static_cast<c_int>(A.nonZeros()), a_x.data(), a_i.data(), a_p.data()));
  if (!P_csc || !A_csc) {
    return osqp_failure("stage=csc, reason=matrix allocation failed");
  }

  OSQPData data;
  data.n = static_cast<c_int>(P.cols());
  data.m = static_cast<c_int>(A.rows());
  data.P = P_csc.get();
  data.A = A_csc.get();
  data.q = q_data.data();
  data.l = l_data.data();
  data.u = u_data.data();

  OSQPSettings settings;
  osqp_set_default_settings(&settings);
  settings.verbose = false;

  OSQPWorkspace * raw_work = nullptr;
  const c_int setup_exit_flag = osqp_setup(&raw_work, &data, &settings);
  std::unique_ptr<OSQPWorkspace, OsqpWorkspaceDeleter> work(raw_work);
  if (setup_exit_flag != 0 || !work) {
    std::ostringstream detail;
    detail << "stage=setup, exit_flag=" << setup_exit_flag;
    return osqp_failure(detail.str());
  }
  const c_int solve_exit_flag = osqp_solve(work.get());
  if (solve_exit_flag != 0) {
    std::ostringstream detail;
    detail << "stage=solve, exit_flag=" << solve_exit_flag << ", "
           << describe_osqp_info(work->info);
    return osqp_failure(detail.str());
  }
  if (work->info == nullptr) {
    return osqp_failure("stage=solve, reason=missing solver info");
  }
  if (
    work->info->status_val != OSQP_SOLVED &&
    work->info->status_val != OSQP_SOLVED_INACCURATE)
  {
    return osqp_failure("stage=status, " + describe_osqp_info(work->info));
  }
  if (work->solution == nullptr || work->solution->x == nullptr) {
    return osqp_failure(
      "stage=solution, reason=missing primal solution, " + describe_osqp_info(work->info));
  }

  const c_int status = work->info->status_val;
  Eigen::VectorXd solution(P.cols());
  for (int i = 0; i < solution.size(); ++i) {
    solution[i] = static_cast<double>(work->solution->x[i]);
  }
  if (!solution.allFinite()) {
    return osqp_failure(
      "stage=solution, reason=non-finite primal solution, " + describe_osqp_info(work->info));
  }

  const Eigen::VectorXd constraint_values = A * solution;
  if (constraint_values.size() != l.size() || !constraint_values.allFinite()) {
    return osqp_failure(
      "stage=constraint_check, reason=invalid projected constraints, " +
      describe_osqp_info(work->info));
  }

  double max_constraint_violation = 0.0;
  double max_projected_abs = 0.0;
  for (int i = 0; i < constraint_values.size(); ++i) {
    double projected_value = constraint_values[i];
    if (std::isfinite(l[i])) {
      projected_value = std::max(projected_value, l[i]);
    }
    if (std::isfinite(u[i])) {
      projected_value = std::min(projected_value, u[i]);
    }
    max_constraint_violation =
      std::max(max_constraint_violation, std::abs(constraint_values[i] - projected_value));
    max_projected_abs = std::max(max_projected_abs, std::abs(projected_value));
  }
  const double constraint_scale =
    std::max(constraint_values.cwiseAbs().maxCoeff(), max_projected_abs);
  const double inaccurate_multiplier = status == OSQP_SOLVED_INACCURATE ? 10.0 : 1.0;
  const double constraint_tolerance =
    inaccurate_multiplier *
    (static_cast<double>(settings.eps_abs) +
    static_cast<double>(settings.eps_rel) * constraint_scale);
  if (max_constraint_violation > constraint_tolerance) {
    std::ostringstream detail;
    detail << "stage=constraint_check, max_violation=" << max_constraint_violation
           << ", tolerance=" << constraint_tolerance << ", "
           << describe_osqp_info(work->info);
    return osqp_failure(detail.str());
  }

  return OsqpSolveOutcome{
    OsqpSolveResult{solution, status, max_constraint_violation}, std::string{}};
}

std::pair<std::vector<double>, std::vector<double>> load_waypoints(const std::string & csv_path)
{
  const auto columns = read_csv_columns(csv_path);
  return {columns.at("wp_x"), columns.at("wp_y")};
}

std::pair<std::vector<double>, std::vector<double>> load_ref_path(
  const std::string & csv_path, const bool circular)
{
  auto points = path_core::load_reference_path_csv(csv_path);
  if (circular) {
    path_core::normalize_circular_endpoint(points, 1e-3);
  }

  std::vector<double> x;
  std::vector<double> y;
  x.reserve(points.size());
  y.reserve(points.size());
  for (const auto & point : points) {
    x.push_back(point.x_m);
    y.push_back(point.y_m);
  }
  return {std::move(x), std::move(y)};
}

double yaw_from_quaternion(const Quaternion & q)
{
  const double sqx = q.x * q.x;
  const double sqy = q.y * q.y;
  const double sqz = q.z * q.z;
  const double sqw = q.w * q.w;
  const double sarg = -2.0 * (q.x * q.z - q.w * q.y) / (sqx + sqy + sqz + sqw);

  if (sarg <= -0.99999) {
    return -2.0 * std::atan2(q.y, q.x);
  }
  if (sarg >= 0.99999) {
    return 2.0 * std::atan2(q.y, q.x);
  }
  return std::atan2(2.0 * (q.x * q.y + q.w * q.z), sqw + sqx - sqy - sqz);
}

Pose2D odom_to_pose_2d(const Odometry & odom)
{
  Pose2D pose;
  pose.x = odom.pose.pose.position.x;
  pose.y = odom.pose.pose.position.y;
  pose.theta = yaw_from_quaternion(odom.pose.pose.orientation);
  return pose;
}

struct Obstacle
{
  double cx{};
  double cy{};
  double radius{};
};

struct Map
{
  explicit Map(const std::string & map_yaml_path)
  {
    const YAML::Node map_data = YAML::LoadFile(map_yaml_path);
    threshold_occupied = map_data["occupied_thresh"].as<double>();
    threshold_free = map_data["free_thresh"].as<double>();
    negate = map_data["negate"] ? map_data["negate"].as<int>() : 0;
    resolution = map_data["resolution"].as<double>();
    origin = map_data["origin"].as<std::vector<double>>();

    const std::filesystem::path yaml_path(map_yaml_path);
    const auto pgm_file_path = yaml_path.parent_path() / map_data["image"].as<std::string>();
    cv::Mat image = cv::imread(pgm_file_path.string(), cv::IMREAD_UNCHANGED);
    if (image.empty()) {
      throw std::runtime_error("Failed to read map image: " + pgm_file_path.string());
    }
    if (image.channels() > 1) {
      std::vector<cv::Mat> channels;
      cv::split(image, channels);
      image = channels.front();
    }

    image.convertTo(data, CV_64F);
    double min_value{};
    double max_value{};
    cv::minMaxLoc(data, &min_value, &max_value);
    if (max_value > 1.0) {
      data /= max_value;
    }
    raw_normalized_data = data.clone();
    process_map();
    height = data.rows;
    width = data.cols;
    data_backup = data.clone();
  }

  std::pair<int, int> w2m(const double x, const double y) const
  {
    int dx = static_cast<int>((x - origin.at(0)) / resolution + 0.5);
    int dy = static_cast<int>((height - 1) - (y - origin.at(1)) / resolution + 0.5);
    dx = std::clamp(dx, 0, width - 1);
    dy = std::clamp(dy, 0, height - 1);
    return {dx, dy};
  }

  std::pair<double, double> m2w(const double dx, const double dy) const
  {
    const double x = static_cast<int>(dx + 0.5) * resolution + origin.at(0);
    const double y = (height - 1 - static_cast<int>(dy + 0.5)) * resolution + origin.at(1);
    return {x, y};
  }

  void process_map()
  {
    for (int r = 0; r < data.rows; ++r) {
      for (int c = 0; c < data.cols; ++c) {
        data.at<double>(r, c) = data.at<double>(r, c) >= threshold_occupied ? 1.0 : 0.0;
      }
    }
    remove_small_holes();
    data.convertTo(data, CV_8S);
  }

  void remove_small_holes()
  {
    cv::Mat visited = cv::Mat::zeros(data.rows, data.cols, CV_8U);
    constexpr int area_threshold = 5;
    const int dr[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
    const int dc[8] = {-1, 0, 1, -1, 1, -1, 0, 1};

    for (int r = 0; r < data.rows; ++r) {
      for (int c = 0; c < data.cols; ++c) {
        if (visited.at<unsigned char>(r, c) || data.at<double>(r, c) != 0.0) {
          continue;
        }
        std::vector<std::pair<int, int>> cells;
        std::vector<std::pair<int, int>> stack{{r, c}};
        visited.at<unsigned char>(r, c) = 1;
        bool touches_border = false;

        while (!stack.empty()) {
          const auto [cr, cc] = stack.back();
          stack.pop_back();
          cells.emplace_back(cr, cc);
          if (cr == 0 || cc == 0 || cr == data.rows - 1 || cc == data.cols - 1) {
            touches_border = true;
          }
          for (int i = 0; i < 8; ++i) {
            const int nr = cr + dr[i];
            const int nc = cc + dc[i];
            if (nr < 0 || nc < 0 || nr >= data.rows || nc >= data.cols) {
              continue;
            }
            if (visited.at<unsigned char>(nr, nc) || data.at<double>(nr, nc) != 0.0) {
              continue;
            }
            visited.at<unsigned char>(nr, nc) = 1;
            stack.emplace_back(nr, nc);
          }
        }

        if (!touches_border && static_cast<int>(cells.size()) < area_threshold) {
          for (const auto & cell : cells) {
            data.at<double>(cell.first, cell.second) = 1.0;
          }
        }
      }
    }
  }

  void reset_map()
  {
    data = data_backup.clone();
    obstacles.clear();
  }

  void add_obstacles(const std::vector<Obstacle> & new_obstacles)
  {
    obstacles.insert(obstacles.end(), new_obstacles.begin(), new_obstacles.end());
    for (const auto & obstacle : new_obstacles) {
      const int radius_px = static_cast<int>(std::ceil(obstacle.radius / resolution));
      const auto [cx_px, cy_px] = w2m(obstacle.cx, obstacle.cy);
      for (int yy = -radius_px; yy < radius_px; ++yy) {
        for (int xx = -radius_px; xx < radius_px; ++xx) {
          if (xx * xx + yy * yy > radius_px * radius_px) {
            continue;
          }
          const int r = cy_px + yy;
          const int c = cx_px + xx;
          if (r >= 0 && c >= 0 && r < data.rows && c < data.cols) {
            data.at<signed char>(r, c) = 0;
          }
        }
      }
    }
  }

  double threshold_occupied{};
  double threshold_free{};
  int negate{};
  cv::Mat data;
  cv::Mat data_backup;
  cv::Mat raw_normalized_data;
  int height{};
  int width{};
  double resolution{};
  std::vector<double> origin;
  std::vector<Obstacle> obstacles;
};

std::vector<std::pair<int, int>> line_cells(const int x0, const int y0, const int x1, const int y1)
{
  std::vector<std::pair<int, int>> cells;
  int x = x0;
  int y = y0;
  const int dx = std::abs(x1 - x0);
  const int dy = -std::abs(y1 - y0);
  const int sx = x0 < x1 ? 1 : -1;
  const int sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;

  while (true) {
    cells.emplace_back(x, y);
    if (x == x1 && y == y1) {
      break;
    }
    const int e2 = 2 * err;
    if (e2 >= dy) {
      err += dy;
      x += sx;
    }
    if (e2 <= dx) {
      err += dx;
      y += sy;
    }
  }
  return cells;
}

struct Waypoint
{
  double x{};
  double y{};
  double psi{};
  double kappa{};
  double v_ref{};
  double ub{};
  double lb{};
  double ub_sm{};
  double lb_sm{};
  std::pair<double, double> static_upper_cell;
  std::pair<double, double> static_lower_cell;
  std::pair<double, double> dynamic_upper_cell;
  std::pair<double, double> dynamic_lower_cell;

  double distance_to(const Waypoint & other) const
  {
    return std::hypot(x - other.x, y - other.y);
  }
};

struct BorderCellsData
{
  int current_wp_id{-1};
  std::vector<std::vector<std::pair<double, double>>> dynamic_upper_bounds;
  std::vector<std::vector<std::pair<double, double>>> dynamic_lower_bounds;
};

struct ReferencePath
{
  ReferencePath(
    Map * map_ptr, const std::vector<double> & wp_x, const std::vector<double> & wp_y,
    const double resolution_in, const int smoothing_distance_in, const double max_width,
    const bool circular_in)
  : map(map_ptr),
    resolution(resolution_in),
    smoothing_distance(smoothing_distance_in),
    circular(circular_in)
  {
    if (wp_x.size() != wp_y.size() || wp_x.size() < 2U) {
      throw std::invalid_argument("Reference path requires matching x/y arrays with at least 2 points");
    }
    if (smoothing_distance < 0) {
      throw std::invalid_argument("Reference path smoothing_distance must be non-negative");
    }
    org_wp_x = wp_x;
    org_wp_y = wp_y;
    waypoints = construct_path(wp_x, wp_y);
    n_waypoints = static_cast<int>(waypoints.size());
    compute_length();
    compute_width(max_width);
  }

  std::vector<Waypoint> construct_path(std::vector<double> wp_x, std::vector<double> wp_y)
  {
    if (circular) {
      const int append_count = std::min<int>(smoothing_distance * 3, wp_x.size());
      wp_x.insert(wp_x.end(), wp_x.begin(), wp_x.begin() + append_count);
      wp_y.insert(wp_y.end(), wp_y.begin(), wp_y.begin() + append_count);
    }

    std::vector<std::size_t> n_wp;
    for (std::size_t i = 0; i + 1 < wp_x.size(); ++i) {
      const double d = std::hypot(wp_x[i + 1] - wp_x[i], wp_y[i + 1] - wp_y[i]);
      if (!std::isfinite(d) || d <= path_core::kMinimumSegmentLengthM) {
        throw std::runtime_error(
                "Reference path contains a degenerate segment at index " +
                std::to_string(i));
      }
      // Keep the legacy waypoint density until distance-based horizon and
      // waypoint-parameter migration are implemented together.  The ceil
      // helper is tested in path_core but is not a production default yet.
      n_wp.push_back(std::max<std::size_t>(1U, static_cast<std::size_t>(d / resolution)));
    }

    const double gp_x = wp_x.back();
    const double gp_y = wp_y.back();
    std::vector<double> interp_x;
    std::vector<double> interp_y;
    for (std::size_t i = 0; i + 1 < wp_x.size(); ++i) {
      for (std::size_t j = 0; j < n_wp[i]; ++j) {
        const double t = static_cast<double>(j) / static_cast<double>(n_wp[i]);
        interp_x.push_back(wp_x[i] + (wp_x[i + 1] - wp_x[i]) * t);
        interp_y.push_back(wp_y[i] + (wp_y[i + 1] - wp_y[i]) * t);
      }
    }
    interp_x.push_back(gp_x);
    interp_y.push_back(gp_y);

    std::vector<std::pair<double, double>> smoothed;
    for (int i = smoothing_distance; i < static_cast<int>(interp_x.size()) - smoothing_distance; ++i) {
      double sx = 0.0;
      double sy = 0.0;
      int count = 0;
      for (int j = i - smoothing_distance; j <= i + smoothing_distance; ++j) {
        sx += interp_x[j];
        sy += interp_y[j];
        ++count;
      }
      smoothed.emplace_back(sx / count, sy / count);
    }
    if (smoothed.size() < 4U) {
      throw std::runtime_error("Reference path contains fewer than 3 waypoints after smoothing");
    }
    return construct_waypoints(smoothed);
  }

  std::vector<Waypoint> construct_waypoints(const std::vector<std::pair<double, double>> & coords)
  {
    std::vector<Waypoint> out;
    for (std::size_t i = 0; i + 1 < coords.size(); ++i) {
      const auto current = coords[i];
      const auto next = coords[i + 1];
      const double dx = next.first - current.first;
      const double dy = next.second - current.second;
      const double psi = std::atan2(dy, dx);
      const double dist_ahead = std::hypot(dx, dy);
      double kappa = 0.0;
      if (i != 0) {
        const auto prev = coords[i - 1];
        const double angle_behind = std::atan2(current.second - prev.second, current.first - prev.first);
        const double angle_dif = wrap_to_pi(psi - angle_behind);
        kappa = angle_dif / (dist_ahead + kEps);
      }
      Waypoint wp;
      wp.x = current.first;
      wp.y = current.second;
      wp.psi = psi;
      wp.kappa = kappa;
      out.push_back(wp);
    }
    return out;
  }

  void compute_length()
  {
    segment_lengths.clear();
    segment_lengths.push_back(0.0);
    length = 0.0;
    for (std::size_t i = 0; i + 1 < waypoints.size(); ++i) {
      const double segment = waypoints[i + 1].distance_to(waypoints[i]);
      segment_lengths.push_back(segment);
      length += segment;
    }
  }

  std::pair<double, std::pair<double, double>> get_min_width(
    const double wp_x_w, const double wp_y_w, const int wp_x, const int wp_y, const int t_x,
    const int t_y, const double max_width)
  {
    double min_width = max_width;
    std::pair<double, double> min_cell = map->m2w(t_x, t_y);
    std::vector<int> path_x;
    std::vector<int> path_y;

    for (int i = -1; i < 2; ++i) {
      for (int j = -1; j < 2; ++j) {
        const int txi = std::clamp(t_x + i, 0, map->width - 1);
        const int tyj = std::clamp(t_y + j, 0, map->height - 1);
        const auto cells = line_cells(wp_x, wp_y, txi, tyj);
        for (std::size_t idx = 0; idx < cells.size(); ++idx) {
          const auto [x, y] = cells[idx];
          if (map->data.at<signed char>(y, x) == 0) {
            min_cell = map->m2w(x, y);
            min_width = std::hypot(wp_x_w - min_cell.first, wp_y_w - min_cell.second);
            return {min_width, min_cell};
          }
          path_x.push_back(x);
          path_y.push_back(y);
        }
      }
    }

    if (!path_x.empty()) {
      double best = std::numeric_limits<double>::infinity();
      int best_index = 0;
      for (std::size_t i = 0; i < path_x.size(); ++i) {
        const double d = std::hypot(static_cast<double>(path_x[i] - t_x), static_cast<double>(path_y[i] - t_y));
        if (d < best) {
          best = d;
          best_index = static_cast<int>(i);
        }
      }
      min_cell = map->m2w(path_x[best_index], path_y[best_index]);
      min_width = std::hypot(wp_x_w - min_cell.first, wp_y_w - min_cell.second);
    }
    return {min_width, min_cell};
  }

  void compute_width(const double max_width)
  {
    for (auto & wp : waypoints) {
      const double left_angle = wrap_to_pi(wp.psi + kPi / 2.0);
      const double right_angle = wrap_to_pi(wp.psi - kPi / 2.0);
      const auto [wp_x, wp_y] = map->w2m(wp.x, wp.y);
      const auto [wp_x_w, wp_y_w] = map->m2w(wp_x, wp_y);

      const auto [t_lx, t_ly] =
        map->w2m(wp_x_w + max_width * std::cos(left_angle), wp_y_w + max_width * std::sin(left_angle));
      const auto left = get_min_width(wp_x_w, wp_y_w, wp_x, wp_y, t_lx, t_ly, max_width);

      const auto [t_rx, t_ry] =
        map->w2m(wp_x_w + max_width * std::cos(right_angle), wp_y_w + max_width * std::sin(right_angle));
      const auto right = get_min_width(wp_x_w, wp_y_w, wp_x, wp_y, t_rx, t_ry, max_width);

      wp.ub = left.first;
      wp.lb = -1.0 * right.first;
      wp.static_upper_cell = left.second;
      wp.static_lower_cell = right.second;
    }
    reset_dynamic_constraints();
  }

  void reset_dynamic_constraints()
  {
    for (auto & wp : waypoints) {
      wp.dynamic_upper_cell = wp.static_upper_cell;
      wp.dynamic_lower_cell = wp.static_lower_cell;
      wp.ub_sm = wp.ub;
      wp.lb_sm = wp.lb;
    }
  }

  void set_v_ref(const std::vector<double> & v_ref)
  {
    const std::size_t count = std::min(v_ref.size(), waypoints.size());
    for (std::size_t i = 0; i < count; ++i) {
      waypoints[i].v_ref = v_ref[i];
    }
  }

  bool compute_speed_profile(
    const double a_min_in, const double a_max_in, const double v_min_in, const double v_max_in,
    const double ay_max)
  {
    const int N = n_waypoints - 1;
    if (N < 2) {
      return false;
    }

    Eigen::VectorXd a_min = Eigen::VectorXd::Constant(N - 1, a_min_in);
    Eigen::VectorXd a_max = Eigen::VectorXd::Constant(N - 1, a_max_in);
    Eigen::VectorXd v_min = Eigen::VectorXd::Constant(N, v_min_in);
    Eigen::VectorXd v_max = Eigen::VectorXd::Constant(N, v_max_in);

    std::vector<Eigen::Triplet<double>> d_triplets;
    for (int i = 0; i < N; ++i) {
      const auto & current = get_waypoint(i);
      const auto & next = get_waypoint(i + 1);
      const double li = next.distance_to(current);
      const double v_max_dyn = std::sqrt(ay_max / (std::abs(current.kappa) + kEps));
      if (v_max_dyn < v_max[i]) {
        v_max[i] = v_max_dyn;
      }
      if (i < N - 1) {
        d_triplets.emplace_back(i, i, -1.0 / (2.0 * li));
        d_triplets.emplace_back(i, i + 1, 1.0 / (2.0 * li));
      }
    }
    for (int i = 0; i < N; ++i) {
      d_triplets.emplace_back(N - 1 + i, i, 1.0);
    }

    Eigen::SparseMatrix<double> D(N - 1 + N, N);
    D.setFromTriplets(d_triplets.begin(), d_triplets.end());
    Eigen::SparseMatrix<double> P(N, N);
    P.setIdentity();
    Eigen::VectorXd q = -1.0 * v_max;
    Eigen::VectorXd l(N - 1 + N);
    Eigen::VectorXd u(N - 1 + N);
    l << a_min, v_min;
    u << a_max, v_max;

    const auto outcome = solve_osqp(P, D, q, l, u);
    if (!outcome.result.has_value() || outcome.result->solution.size() != N) {
      return false;
    }
    const Eigen::VectorXd solution = outcome.result->solution;
    for (int i = 0; i < N; ++i) {
      waypoints[i].v_ref = solution[i];
    }
    waypoints.back().v_ref = circular ? waypoints[waypoints.size() - 2].v_ref : 0.0;
    return true;
  }

  const Waypoint & get_waypoint(int wp_id) const
  {
    if (wp_id >= n_waypoints && circular) {
      wp_id = wp_id % n_waypoints;
    } else if (wp_id >= n_waypoints && !circular) {
      wp_id = n_waypoints - 1;
    }
    return waypoints.at(wp_id);
  }

  Waypoint & get_waypoint_mutable(int wp_id)
  {
    if (wp_id >= n_waypoints && circular) {
      wp_id = wp_id % n_waypoints;
    } else if (wp_id >= n_waypoints && !circular) {
      wp_id = n_waypoints - 1;
    }
    return waypoints.at(wp_id);
  }

  void update_simple_path_constraints(const int N, const double safety_margin)
  {
    path_constraints_upper.assign(n_waypoints - 1, std::vector<double>(N, 0.0));
    path_constraints_lower.assign(n_waypoints - 1, std::vector<double>(N, 0.0));
    border_cells.dynamic_upper_bounds.assign(
      n_waypoints - 1, std::vector<std::pair<double, double>>(N, {0.0, 0.0}));
    border_cells.dynamic_lower_bounds.assign(
      n_waypoints - 1, std::vector<std::pair<double, double>>(N, {0.0, 0.0}));

    for (int wp_id = 0; wp_id < n_waypoints - 1; ++wp_id) {
      for (int n = 0; n < N; ++n) {
        const auto & wp = get_waypoint(wp_id + n);
        double ub_sm = wp.ub - safety_margin;
        double lb_sm = wp.lb + safety_margin;
        if (ub_sm < lb_sm) {
          ub_sm = 0.0;
          lb_sm = 0.0;
        }
        const double angle_ub = wrap_to_pi(kPi / 2.0 + wp.psi);
        const double angle_lb = wrap_to_pi(-kPi / 2.0 + wp.psi);
        const std::pair<double, double> ub_sm_ls{
          wp.x + ub_sm * std::cos(angle_ub), wp.y + ub_sm * std::sin(angle_ub)};
        const std::pair<double, double> lb_sm_ls{
          wp.x - lb_sm * std::cos(angle_lb), wp.y - lb_sm * std::sin(angle_lb)};
        path_constraints_upper[wp_id][n] = ub_sm;
        path_constraints_lower[wp_id][n] = lb_sm;
        border_cells.dynamic_upper_bounds[wp_id][n] = ub_sm_ls;
        border_cells.dynamic_lower_bounds[wp_id][n] = lb_sm_ls;
      }
    }
  }

  bool set_path_constraints(
    const std::vector<float> & upper_bounds, const std::vector<float> & lower_bounds,
    const int rows, const int cols)
  {
    if (rows <= 0 || cols <= 0) {
      return false;
    }
    if (
      upper_bounds.size() != static_cast<std::size_t>(rows * cols) ||
      lower_bounds.size() != static_cast<std::size_t>(rows * cols)) {
      return false;
    }

    path_constraints_upper.assign(rows, std::vector<double>(cols, 0.0));
    path_constraints_lower.assign(rows, std::vector<double>(cols, 0.0));
    for (int row = 0; row < rows; ++row) {
      for (int col = 0; col < cols; ++col) {
        const std::size_t index = static_cast<std::size_t>(row * cols + col);
        path_constraints_upper[row][col] = static_cast<double>(upper_bounds[index]);
        path_constraints_lower[row][col] = static_cast<double>(lower_bounds[index]);
      }
    }
    return true;
  }

  bool set_border_cells(
    const std::vector<float> & dynamic_upper_bounds,
    const std::vector<float> & dynamic_lower_bounds, const int rows, const int cols)
  {
    if (rows <= 0 || cols <= 0) {
      return false;
    }
    if (
      dynamic_upper_bounds.size() != static_cast<std::size_t>(rows * cols * 2) ||
      dynamic_lower_bounds.size() != static_cast<std::size_t>(rows * cols * 2)) {
      return false;
    }

    border_cells.dynamic_upper_bounds.assign(
      rows, std::vector<std::pair<double, double>>(cols, {0.0, 0.0}));
    border_cells.dynamic_lower_bounds.assign(
      rows, std::vector<std::pair<double, double>>(cols, {0.0, 0.0}));
    for (int row = 0; row < rows; ++row) {
      for (int col = 0; col < cols; ++col) {
        const std::size_t index = static_cast<std::size_t>((row * cols + col) * 2);
        border_cells.dynamic_upper_bounds[row][col] = {
          static_cast<double>(dynamic_upper_bounds[index]),
          static_cast<double>(dynamic_upper_bounds[index + 1])};
        border_cells.dynamic_lower_bounds[row][col] = {
          static_cast<double>(dynamic_lower_bounds[index]),
          static_cast<double>(dynamic_lower_bounds[index + 1])};
      }
    }
    return true;
  }

  Map * map{};
  std::vector<double> org_wp_x;
  std::vector<double> org_wp_y;
  double resolution{};
  int smoothing_distance{};
  bool circular{};
  std::vector<Waypoint> waypoints;
  int n_waypoints{};
  double length{};
  std::vector<double> segment_lengths;
  std::vector<std::vector<double>> path_constraints_upper;
  std::vector<std::vector<double>> path_constraints_lower;
  BorderCellsData border_cells;
};

struct TemporalState
{
  double x{};
  double y{};
  double psi{};
};

struct SimpleSpatialState
{
  double e_y{};
  double e_psi{};
  double t{};
};

struct BicycleModel
{
  BicycleModel(
    ReferencePath * ref_path, const double length_in, const double width_in,
    const double safety_margin_scale_in, const double Ts_in,
    const double min_linearization_speed_mps_in)
  : length(length_in),
    width(width_in),
    safety_margin_scale(std::max(0.0, safety_margin_scale_in)),
    safety_margin(compute_safety_margin()),
    reference_path(ref_path),
    Ts(Ts_in),
    min_linearization_speed_mps(min_linearization_speed_mps_in)
  {
    current_waypoint = &reference_path->waypoints.at(wp_id);
    temporal_state = s2t(*current_waypoint, spatial_state);
  }

  double compute_safety_margin() const
  {
    return width / std::sqrt(2.0) * safety_margin_scale;
  }

  TemporalState s2t(const Waypoint & reference_waypoint, const SimpleSpatialState & reference_state) const
  {
    TemporalState state;
    state.x = reference_waypoint.x - reference_state.e_y * std::sin(reference_waypoint.psi);
    state.y = reference_waypoint.y + reference_state.e_y * std::cos(reference_waypoint.psi);
    state.psi = reference_waypoint.psi + reference_state.e_psi;
    return state;
  }

  TemporalState s2t(const Waypoint & reference_waypoint, const Eigen::Vector3d & reference_state) const
  {
    TemporalState state;
    state.x = reference_waypoint.x - reference_state[0] * std::sin(reference_waypoint.psi);
    state.y = reference_waypoint.y + reference_state[0] * std::cos(reference_waypoint.psi);
    state.psi = reference_waypoint.psi + reference_state[1];
    return state;
  }

  SimpleSpatialState t2s(const Waypoint & reference_waypoint, const TemporalState & reference_state) const
  {
    SimpleSpatialState state;
    state.e_y = std::cos(reference_waypoint.psi) * (reference_state.y - reference_waypoint.y) -
                std::sin(reference_waypoint.psi) * (reference_state.x - reference_waypoint.x);
    state.e_psi = wrap_to_pi(reference_state.psi - reference_waypoint.psi);
    state.t = 0.0;
    return state;
  }

  void drive(const Eigen::Vector2d & u)
  {
    const double v = u[0];
    const double delta = u[1];
    const double x_dot = v * std::cos(temporal_state.psi);
    const double y_dot = v * std::sin(temporal_state.psi);
    const double psi_dot = v / length * std::tan(delta);
    temporal_state.x += x_dot * Ts;
    temporal_state.y += y_dot * Ts;
    temporal_state.psi += psi_dot * Ts;

    const double s_dot = 1.0 / (1.0 - spatial_state.e_y * current_waypoint->kappa) * v *
                         std::cos(spatial_state.e_psi);
    s += s_dot * Ts;
  }

  void get_current_waypoint()
  {
    std::vector<double> length_cum(segment_lengths_size());
    double acc = 0.0;
    for (std::size_t i = 0; i < reference_path->segment_lengths.size(); ++i) {
      acc += reference_path->segment_lengths[i];
      length_cum[i] = acc;
    }
    const auto it = std::upper_bound(length_cum.begin(), length_cum.end(), s);
    int next_wp_id = static_cast<int>(std::distance(length_cum.begin(), it));
    if (next_wp_id == static_cast<int>(length_cum.size())) {
      wp_id = static_cast<int>(length_cum.size()) - 1;
      current_waypoint = &reference_path->waypoints.at(wp_id);
      return;
    }
    const int prev_wp_id = next_wp_id - 1;
    const double s_next = length_cum[next_wp_id];
    const double s_prev = length_cum[std::max(prev_wp_id, 0)];
    if (std::abs(s - s_next) < std::abs(s - s_prev)) {
      wp_id = next_wp_id;
    } else {
      wp_id = prev_wp_id;
    }
    wp_id = std::clamp(wp_id, 0, reference_path->n_waypoints - 1);
    current_waypoint = &reference_path->waypoints.at(wp_id);
  }

  std::size_t segment_lengths_size() const
  {
    return reference_path->segment_lengths.size();
  }

  int get_closest_waypoint(const double x, const double y) const
  {
    double best = std::numeric_limits<double>::infinity();
    int best_id = 0;
    for (int i = 0; i < reference_path->n_waypoints; ++i) {
      const auto & wp = reference_path->waypoints[i];
      const double d = std::hypot(wp.x - x, wp.y - y);
      if (d < best) {
        best = d;
        best_id = i;
      }
    }
    return best_id;
  }

  double get_s_at_waypoint(const int id) const
  {
    double acc = 0.0;
    for (int i = 0; i <= id && i < static_cast<int>(reference_path->segment_lengths.size()); ++i) {
      acc += reference_path->segment_lengths[i];
    }
    return acc;
  }

  void update_reference_path(ReferencePath * ref_path)
  {
    reference_path = ref_path;
    wp_id = get_closest_waypoint(temporal_state.x, temporal_state.y);
    s = get_s_at_waypoint(wp_id);
    current_waypoint = &reference_path->waypoints.at(wp_id);
  }

  void update_states(const double x, const double y, const double psi)
  {
    temporal_state.x = x;
    temporal_state.y = y;
    temporal_state.psi = psi;
    wp_id = get_closest_waypoint(temporal_state.x, temporal_state.y);
    s = get_s_at_waypoint(wp_id);
    current_waypoint = &reference_path->waypoints.at(wp_id);
  }

  std::tuple<Eigen::Vector3d, Eigen::Matrix3d, Eigen::Matrix<double, 3, 2>> linearize(
    const double v_ref, const double kappa_ref, const double delta_s) const
  {
    Eigen::Vector3d f;
    Eigen::Matrix3d A;
    Eigen::Matrix<double, 3, 2> B;
    A.row(0) = Eigen::Vector3d(1.0, delta_s, 0.0);
    A.row(1) = Eigen::Vector3d(-std::pow(kappa_ref, 2.0) * delta_s, 1.0, 0.0);
    B.row(0) = Eigen::Vector2d(0.0, 0.0);
    B.row(1) = Eigen::Vector2d(0.0, delta_s);
    if (std::abs(v_ref) < min_linearization_speed_mps) {
      A.row(2) = Eigen::Vector3d(0.0, 0.0, 1.0);
      B.row(2) = Eigen::Vector2d(0.0, 0.0);
      f = Eigen::Vector3d(0.0, 0.0, 0.0);
    } else {
      A.row(2) = Eigen::Vector3d(-kappa_ref / v_ref * delta_s, 0.0, 1.0);
      B.row(2) = Eigen::Vector2d(-1.0 / (v_ref * v_ref) * delta_s, 0.0);
      f = Eigen::Vector3d(0.0, 0.0, 1.0 / v_ref * delta_s);
    }
    return {f, A, B};
  }

  double length{};
  double width{};
  double safety_margin_scale{};
  double safety_margin{};
  ReferencePath * reference_path{};
  double s{0.0};
  double Ts{};
  double min_linearization_speed_mps{0.5};
  int wp_id{0};
  Waypoint * current_waypoint{};
  SimpleSpatialState spatial_state;
  TemporalState temporal_state;
};

struct V2XGapPlannerConfig
{
  bool enabled{false};
  double vehicle_radius{1.25};
  double vehicle_length{2.0};
  double prediction_margin{0.2};
  double prediction_time{3.0};
  bool prediction_use_path_time{false};
  double prediction_min_ego_speed{1.0};
  double prediction_max_ego_speed{std::numeric_limits<double>::infinity()};
  double timeout_sec{1.0};
  double position_jump_threshold{5.0};
  double v_max_safety{30.0};
  double self_filter_radius{1.5};
  double min_gap_width{1.8};
  double target_bias{1.0};
  double no_gap_target_velocity{0.0};
  double wall_clearance_margin{0.0};
  double vehicle_side_target_margin{0.0};
  double wall_avoidance_bias{0.0};
  bool vehicle_vehicle_gap_enabled{true};
  double vehicle_vehicle_gap_min_distance{0.0};
  double vehicle_vehicle_gap_min_width{0.0};
  bool multi_front_gap_enabled{true};
  double multi_front_gap_distance{0.0};
  std::string low_speed_pass_side{"auto"};
  double low_speed_pass_ramp_ratio{1.0};
  bool overtake_target_ramp_enabled{false};
  double overtake_target_ramp_ratio{0.7};
};

struct OvertakeLineConfig
{
  bool enabled{false};
  double shift_distance{8.0};
  double pass_distance{8.0};
  double return_distance{10.0};
  double lateral_offset{1.2};
  double target_bias{0.8};
  double min_wall_clearance{0.8};
  double max_lateral_accel{2.5};
  double max_target_change{0.25};
  double return_clear_distance{4.0};
  double phase_hold_time{0.3};
  double target_hold_sec{0.0};
  double clear_confirm_sec{0.0};
  bool reacquire_enabled{false};
  double reacquire_window_sec{0.0};
  double reacquire_max_return_progress{0.0};
  bool recovery_velocity_limit_enabled{true};
  double recovery_velocity{5.0};
  double recovery_stall_speed{0.15};
  double recovery_stall_timeout_sec{1.0};
  double recovery_timeout_sec{5.0};
  double recovery_max_observation_gap_sec{0.2};
  double solver_cooldown_sec{2.0};
  int solver_failure_abort_cycles{3};
  int solver_recovery_success_cycles{20};
  bool debug_log_enabled{false};
};

struct V2XBehaviorConfig
{
  bool enabled{false};
  bool debug_log_enabled{false};
  double debug_log_period_sec{1.0};
  bool front_progress_detection_enabled{false};
  double front_progress_detection_distance{0.0};
  double front_progress_lookbehind_distance{3.0};
  bool front_hazard_hold_enabled{false};
  double front_hazard_hold_sec{0.0};
  double front_hazard_rear_clear_distance{4.0};
  double follow_distance{8.0};
  double safety_brake_distance{3.0};
  double safety_brake_margin{2.0};
  bool follow_gap_planner_enabled{false};
  bool follow_gap_planner_no_gap_speed_limit_enabled{false};
  bool follow_gap_planner_respect_overtake_forbidden{true};
  bool follow_speed_limit_enabled{false};
  /// 0 preserves the legacy behavior: cap every detected Follow target.
  double follow_speed_limit_distance{0.0};
  double follow_velocity{5.0};
  bool follow_preposition_enabled{false};
  double follow_preposition_offset{0.0};
  double follow_preposition_min_side_clearance{1.2};
  double follow_preposition_target_bias{0.25};
  double follow_preposition_ramp_ratio{1.5};
  bool front_decel_guard_enabled{true};
  double front_decel_guard_distance{9.0};
  double front_decel_guard_ttc{1.5};
  double front_decel_guard_speed_margin{0.5};
  double front_decel_guard_min_closing_speed{1.5};
  bool front_decel_guard_curve_include_slow_front{false};
  double front_decel_guard_curve_lateral_margin{0.0};
  double front_decel_guard_curve_lookahead_distance{0.0};
  double front_decel_guard_curve_distance{16.0};
  double front_decel_guard_curve_ttc{3.0};
  bool front_risk_arbitration_enabled{false};
  bool front_risk_brake_prepare_limit_enabled{false};
  bool front_risk_avoid_candidate_limit_enabled{true};
  double front_risk_comfort_decel{2.0};
  double front_risk_hard_decel{4.0};
  double front_risk_emergency_decel{6.0};
  double front_risk_distance_margin{3.0};
  double front_risk_min_closing_speed{0.5};
  double front_risk_prepare_time{1.5};
  bool front_risk_curve_limit_enabled{false};
  double front_risk_curve_limit_required_decel{1.2};
  double front_risk_curve_limit_decel{1.4};
  double front_risk_curve_limit_speed_margin{0.5};
  double safety_brake_velocity{0.0};
  double overtake_min_gap_width{2.0};
  double overtake_max_curvature{0.05};
  bool overtake_block_inner_curve_pass{false};
  bool overtake_outer_curve_entry_enabled{false};
  bool overtake_outer_curve_hard_continuation_enabled{false};
  bool overtake_inner_curve_entry_enabled{false};
  bool overtake_inner_curve_hard_continuation_enabled{false};
  double overtake_forbidden_curve_lookahead_distance{0.0};
  bool overtake_guard_enabled{true};
  double overtake_guard_min_gap_width{2.5};
  int overtake_guard_min_gap_points{3};
  double overtake_guard_min_prepare_distance{8.0};
  double overtake_guard_max_lateral_shift{1.2};
  bool overtake_guard_reachable_gap_enabled{false};
  double overtake_guard_max_lateral_accel{2.0};
  double overtake_guard_min_gap_time{0.8};
  double overtake_guard_min_speed_for_reachable{1.0};
  double overtake_guard_min_front_distance{3.0};
  double overtake_continue_min_front_distance{3.0};
  bool overtake_close_follow_enabled{false};
  double overtake_close_follow_min_front_distance{1.5};
  double overtake_close_follow_max_closing_speed{0.8};
  double overtake_close_follow_min_side_clearance{2.0};
  bool overtake_before_curve_enabled{false};
  double overtake_before_curve_max_front_speed{8.0};
  double overtake_before_curve_min_speed_advantage{1.0};
  double overtake_start_curve_clearance_distance{0.0};
  bool overtake_continue_in_forbidden_enabled{false};
  bool overtake_continue_inner_soft_curve_enabled{false};
  bool overtake_active_hard_curve_completion_enabled{false};
  double overtake_active_hard_curve_rear_clear_distance{0.5};
  double overtake_active_hard_curve_buffer_distance{0.5};
  bool overtake_front_velocity_limit_enabled{true};
  bool overtake_fallback_ignore_soft_curve_forbidden{false};
  double overtake_fallback_min_side_clearance{1.0};
  bool overtake_curve_cooldown_enabled{false};
  double overtake_curve_cooldown_sec{0.0};
  bool side_overtake_enabled{false};
  bool side_overtake_ignore_soft_curve_forbidden{true};
  double side_overtake_entry_rear_tolerance{0.5};
  double overtake_gap_lookahead_distance{0.0};
  bool overtake_try_both_sides{false};
  double overtake_velocity_advantage{0.0};
  bool overtake_stage_speed_enabled{false};
  bool overtake_shiftout_adaptive_closing_speed_enabled{false};
  double overtake_shiftout_min_closing_speed{1.0};
  double overtake_shiftout_max_closing_speed{1.0};
  double overtake_pass_unlatched_max_closing_speed{0.5};
  double overtake_shiftout_adaptive_min_time_sec{0.5};
  double overtake_pass_front_overlap_lateral_clearance{0.0};
  bool overtake_completion_guard_enabled{false};
  double overtake_completion_hard_curvature{0.12};
  double overtake_completion_lookahead_distance{80.0};
  double overtake_completion_curve_buffer_distance{2.0};
  double overtake_completion_merge_buffer_distance{3.0};
  double overtake_completion_min_relative_speed{0.5};
  double moving_front_speed_threshold{1.0};
  double moving_follow_speed_margin{2.0};
  double moving_follow_hard_distance{0.0};
  double moving_follow_target_distance{0.0};
  double moving_follow_recovery_speed_margin{0.0};
  double moving_follow_distance_gain{0.0};
  double moving_safety_brake_distance{1.5};
  double moving_safety_brake_margin{1.0};
  double moving_safety_brake_time_headway{0.3};
  double start_grid_grace_time{0.0};
  bool require_gap_for_overtake{true};
  bool low_speed_avoidance_enabled{false};
  bool low_speed_avoidance_ignore_soft_curve_forbidden{false};
  bool low_speed_local_path_enabled{false};
  double low_speed_avoidance_distance{8.0};
  double low_speed_avoidance_lookahead_distance{18.0};
  double low_speed_avoidance_velocity{2.0};
  double low_speed_avoidance_shift_velocity{1.0};
  double low_speed_avoidance_shift_lateral_gain{0.4};
  double low_speed_avoidance_shift_heading_gain{1.3};
  double low_speed_avoidance_shift_lateral_tolerance{0.4};
  double low_speed_avoidance_shift_heading_tolerance{0.2};
  double low_speed_avoidance_shift_clear_hold_sec{2.0};
  double low_speed_avoidance_max_front_speed{1.0};
  double low_speed_avoidance_min_prepare_distance{0.0};
  double low_speed_avoidance_min_gap_width{1.5};
  int low_speed_avoidance_min_gap_points{2};
  double low_speed_avoidance_clear_distance{8.0};
  double low_speed_avoidance_stall_speed{0.15};
  double low_speed_avoidance_stall_timeout_sec{1.5};
  double low_speed_avoidance_stall_cooldown_sec{3.0};
  double low_speed_avoidance_stall_max_observation_gap_sec{0.2};
  double low_speed_local_path_pass_clearance{3.0};
  double low_speed_local_path_return_distance{6.0};
  bool low_speed_local_path_invert_target{false};
  double state_hold_time{0.5};
  std::vector<std::pair<int, int>> overtake_forbidden_wp_ranges;
  OvertakeLineConfig overtake_line;
};

enum class V2XBehaviorState
{
  Cruise,
  Follow,
  Overtake,
  LowSpeedAvoidance,
  SafetyBrake,
};

const char * to_string(const V2XBehaviorState state)
{
  switch (state) {
    case V2XBehaviorState::Cruise:
      return "Cruise";
    case V2XBehaviorState::Follow:
      return "Follow";
    case V2XBehaviorState::Overtake:
      return "Overtake";
    case V2XBehaviorState::LowSpeedAvoidance:
      return "LowSpeedAvoidance";
    case V2XBehaviorState::SafetyBrake:
      return "SafetyBrake";
  }
  return "Unknown";
}

enum class OvertakeLinePhase
{
  Idle,
  FollowPrepare,
  ShiftOut,
  Pass,
  Return,
  Recovery,
};

const char * to_string(const OvertakeLinePhase phase)
{
  switch (phase) {
    case OvertakeLinePhase::Idle:
      return "Idle";
    case OvertakeLinePhase::FollowPrepare:
      return "FollowPrepare";
    case OvertakeLinePhase::ShiftOut:
      return "ShiftOut";
    case OvertakeLinePhase::Pass:
      return "Pass";
    case OvertakeLinePhase::Return:
      return "Return";
    case OvertakeLinePhase::Recovery:
      return "Recovery";
  }
  return "Unknown";
}

int behavior_restriction_rank(const V2XBehaviorState state)
{
  switch (state) {
    case V2XBehaviorState::Cruise:
      return 0;
    case V2XBehaviorState::Overtake:
      return 1;
    case V2XBehaviorState::Follow:
      return 2;
    case V2XBehaviorState::LowSpeedAvoidance:
      return 3;
    case V2XBehaviorState::SafetyBrake:
      return 4;
  }
  return 0;
}

enum class FrontRiskLevel
{
  Clear,
  Follow,
  BrakePrepare,
  AvoidCandidate,
  EmergencyBrake,
};

const char * to_string(const FrontRiskLevel level)
{
  switch (level) {
    case FrontRiskLevel::Clear:
      return "Clear";
    case FrontRiskLevel::Follow:
      return "Follow";
    case FrontRiskLevel::BrakePrepare:
      return "BrakePrepare";
    case FrontRiskLevel::AvoidCandidate:
      return "AvoidCandidate";
    case FrontRiskLevel::EmergencyBrake:
      return "EmergencyBrake";
  }
  return "Unknown";
}

struct GapPlannerOutput
{
  bool active{false};
  bool feasible{true};
  bool pass_corridor_enforced{false};
  int pass_side_sign{0};
  double pass_target_ey{0.0};
  std::vector<double> lb;
  std::vector<double> ub;
  std::vector<double> target_ey;
  std::vector<bool> target_active;
  double target_velocity_limit{std::numeric_limits<double>::infinity()};
  std::string reject_reason;
};

struct FrontRiskMetrics
{
  bool valid{false};
  double front_distance{std::numeric_limits<double>::infinity()};
  double front_speed{std::numeric_limits<double>::infinity()};
  double ego_speed{0.0};
  double relative_speed{0.0};
  double available_distance{std::numeric_limits<double>::infinity()};
  double required_decel{0.0};
  double ttc{std::numeric_limits<double>::infinity()};
};

struct ReachableGapMetrics
{
  bool valid{false};
  double first_gap_distance{std::numeric_limits<double>::infinity()};
  double first_gap_time{std::numeric_limits<double>::infinity()};
  double first_lateral_shift{0.0};
  double max_lateral_shift{0.0};
  double max_required_lateral_accel{0.0};
  std::size_t max_required_lateral_accel_index{0};
};

struct V2XBehaviorOutput
{
  V2XBehaviorState state{V2XBehaviorState::Cruise};
  bool allow_gap_planner{false};
  bool follow_gap_planner_allowed{true};
  std::size_t active_vehicle_count{0};
  bool has_front_vehicle{false};
  bool front_progress_used{false};
  bool has_side_vehicle{false};
  bool has_low_speed_clearance_vehicle{false};
  bool has_danger_vehicle{false};
  v2x_overtake_core::FrontDangerAction front_danger_action{
    v2x_overtake_core::FrontDangerAction::None};
  bool front_hazard_hold_active{false};
  double front_hazard_hold_remaining_sec{0.0};
  std::string front_hazard_hold_target_id;
  bool start_grid_grace_active{false};
  bool start_grid_stop_suppressed{false};
  bool low_speed_avoidance_candidate{false};
  bool low_speed_avoidance_gap_blocked{false};
  bool low_speed_avoidance_stalled{false};
  bool low_speed_avoidance_cooldown_active{false};
  bool follow_speed_limit_active{false};
  bool follow_speed_limit_moving_front{false};
  bool moving_front_clearance_limit_active{false};
  double moving_front_clearance_speed_margin{0.0};
  bool overtake_forbidden{false};
  bool overtake_forbidden_wp{false};
  bool front_decel_curve_guard{false};
  bool overtake_zone_allows{false};
  bool overtake_start_curve_blocked{false};
  bool before_curve_overtake_allowed{false};
  bool continuing_overtake_allowed{false};
  bool overtake_hard_curve_blocked{false};
  bool active_hard_curve_continuation_allowed{false};
  bool outer_curve_entry_allowed{false};
  bool outer_curve_hard_continuation_allowed{false};
  bool inner_curve_entry_allowed{false};
  bool inner_curve_hard_continuation_allowed{false};
  bool overtake_inner_curve_pass{false};
  bool overtake_completion_feasible{true};
  bool overtake_speed_front_cap_applied{false};
  bool overtake_shiftout_adaptive_speed_applied{false};
  bool overtake_front_cap_release_ready{false};
  bool overtake_gap_available{false};
  bool overtake_fallback_target{false};
  bool overtake_cooldown_active{false};
  int overtake_pass_side_sign{0};
  int overtake_plan_N{0};
  double overtake_side_clearance{0.0};
  double overtake_completion_available_distance{std::numeric_limits<double>::infinity()};
  double overtake_completion_required_distance{0.0};
  double overtake_completion_relative_speed{std::numeric_limits<double>::infinity()};
  double overtake_shiftout_closing_speed_limit{std::numeric_limits<double>::infinity()};
  double overtake_shiftout_remaining_time{std::numeric_limits<double>::infinity()};
  double active_hard_curve_distance{std::numeric_limits<double>::infinity()};
  double active_hard_curve_available_distance{0.0};
  double active_hard_curve_required_distance{std::numeric_limits<double>::infinity()};
  bool overtake_left_gap_available{false};
  bool overtake_right_gap_available{false};
  std::string overtake_left_reason;
  std::string overtake_right_reason;
  std::string target_vehicle_id;
  bool locked_target_seen{false};
  bool locked_target_position_jump{false};
  double locked_target_longitudinal{std::numeric_limits<double>::infinity()};
  double locked_target_lateral{std::numeric_limits<double>::infinity()};
  double locked_target_speed{std::numeric_limits<double>::infinity()};
  double locked_target_receipt_sec{-std::numeric_limits<double>::infinity()};
  double front_speed{std::numeric_limits<double>::infinity()};
  double front_lateral{std::numeric_limits<double>::infinity()};
  double ego_speed{0.0};
  FrontRiskMetrics front_risk;
  FrontRiskLevel front_risk_level{FrontRiskLevel::Clear};
  double target_velocity_limit{std::numeric_limits<double>::infinity()};
  double desired_velocity{std::numeric_limits<double>::infinity()};
  double front_distance{std::numeric_limits<double>::infinity()};
  double front_local_longitudinal{std::numeric_limits<double>::infinity()};
  double front_progress_lateral{std::numeric_limits<double>::infinity()};
  double low_speed_avoidance_stalled_sec{0.0};
  std::string reason;
  std::string overtake_block_reason;
};

struct OvertakeLineState
{
  OvertakeLinePhase phase{OvertakeLinePhase::Idle};
  bool pass_front_overlap_exclusion_latched{false};
  int pass_side_sign{0};
  double target_ey{0.0};
  double phase_start_sec{-std::numeric_limits<double>::infinity()};
  double phase_start_ey{0.0};
  double phase_traveled_m{0.0};
  double phase_last_update_sec{-std::numeric_limits<double>::infinity()};
  double recovery_stall_since_sec{std::numeric_limits<double>::quiet_NaN()};
  std::string target_vehicle_id;
  double target_last_seen_sec{-std::numeric_limits<double>::infinity()};
  double target_last_longitudinal{std::numeric_limits<double>::infinity()};
  double target_last_lateral{std::numeric_limits<double>::infinity()};
  double target_last_speed{std::numeric_limits<double>::infinity()};
  double rear_clear_since_sec{std::numeric_limits<double>::quiet_NaN()};
};

struct OvertakeLineOutput
{
  bool active{false};
  std::vector<double> target_ey;
  std::vector<bool> target_active;
  double target_velocity_reference{std::numeric_limits<double>::infinity()};
  double target_velocity_limit{std::numeric_limits<double>::infinity()};
  double closing_speed_limit{std::numeric_limits<double>::infinity()};
  bool front_cap_release_ready{false};
  double max_required_lateral_accel{0.0};
  bool lateral_accel_limited{false};
  bool wall_clearance_limited{false};
};

struct V2XGapPlanner
{
  struct TrackedVehicle
  {
    std::string id;
    double x{};
    double y{};
    double covariance_x{};
    double covariance_y{};
    double stamp_sec{};
    double receipt_sec{};
    double vx{};
    double vy{};
    bool has_sample{false};
    bool position_jump{false};
    bool invalid_velocity{false};
  };

  struct LateralInterval
  {
    double lower{};
    double upper{};

    double width() const
    {
      return upper - lower;
    }

    double center() const
    {
      return 0.5 * (lower + upper);
    }
  };

  struct OccupiedInterval
  {
    double lower{};
    double upper{};
    double distance_from_self{};

    double width() const
    {
      return upper - lower;
    }
  };

  struct GapCandidate
  {
    LateralInterval interval;
    bool lower_is_vehicle{false};
    bool upper_is_vehicle{false};
    double lower_vehicle_distance{std::numeric_limits<double>::infinity()};
    double upper_vehicle_distance{std::numeric_limits<double>::infinity()};

    bool is_vehicle_vehicle_gap() const
    {
      return lower_is_vehicle && upper_is_vehicle;
    }

    double nearest_vehicle_distance() const
    {
      return std::min(lower_vehicle_distance, upper_vehicle_distance);
    }
  };

  explicit V2XGapPlanner(
    const V2XGapPlannerConfig & cfg_in, const bool track_recovery_completeness = false)
  : cfg(cfg_in), track_recovery_completeness_(track_recovery_completeness) {}

  void update(const V2XVehiclePositionArray & msg, const double receipt_sec)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (track_recovery_completeness_) {
      last_message_receipt_sec_ = receipt_sec;
      last_message_vehicle_count_ = msg.vehicles.size();
      last_message_has_empty_id_ = false;
      last_message_has_duplicate_id_ = false;
      last_message_has_invalid_sample_ = false;
    }
    std::unordered_set<std::string> message_ids;
    const double array_stamp = stamp_to_seconds(msg.header.stamp);
    if (track_recovery_completeness_) {
      if (
        msg.header.frame_id != "map" ||
        !stuck_recovery::source_timestamp_is_monotonic(
          array_stamp, last_message_source_stamp_sec_))
      {
        last_message_has_invalid_sample_ = true;
      }
      if (std::isfinite(array_stamp) && array_stamp > 0.0) {
        last_message_source_stamp_sec_ = array_stamp;
      }
    }
    for (const auto & vehicle : msg.vehicles) {
      if (track_recovery_completeness_) {
        if (vehicle.vehicle_id.empty()) {
          last_message_has_empty_id_ = true;
        } else if (!message_ids.emplace(vehicle.vehicle_id).second) {
          last_message_has_duplicate_id_ = true;
        }
        if (
          !std::isfinite(vehicle.position.x) || !std::isfinite(vehicle.position.y) ||
          !std::isfinite(vehicle.covariance.x) || !std::isfinite(vehicle.covariance.y) ||
          vehicle.covariance.x < 0.0 || vehicle.covariance.y < 0.0 ||
          (!vehicle.header.frame_id.empty() && vehicle.header.frame_id != "map"))
        {
          last_message_has_invalid_sample_ = true;
        }
      }
      const std::string id = vehicle.vehicle_id.empty() ? "__unknown__" : vehicle.vehicle_id;
      double sample_stamp = stamp_to_seconds(vehicle.header.stamp);
      if (sample_stamp <= 0.0) {
        sample_stamp = array_stamp > 0.0 ? array_stamp : receipt_sec;
      }

      auto & tracked = vehicles_[id];
      if (track_recovery_completeness_) {
        if (
          !stuck_recovery::source_sample_is_current(
            array_stamp, sample_stamp, cfg.timeout_sec) ||
          !stuck_recovery::source_timestamp_is_monotonic(
            sample_stamp,
            tracked.has_sample ? std::optional<double>{tracked.stamp_sec} : std::nullopt))
        {
          last_message_has_invalid_sample_ = true;
        }
      }
      double vx = 0.0;
      double vy = 0.0;
      bool position_jump = false;
      bool invalid_velocity = false;
      if (tracked.has_sample) {
        const double dt = sample_stamp - tracked.stamp_sec;
        const double dx = vehicle.position.x - tracked.x;
        const double dy = vehicle.position.y - tracked.y;
        const double jump = std::hypot(dx, dy);
        // AWSIM V2X can update near 1 Hz, so a normally moving kart may travel
        // more than the fixed spatial jump threshold between samples. Scale
        // the admissible displacement by dt and the configured safety speed;
        // the fixed threshold remains the minimum tolerance for short-dt
        // localization noise and discontinuities.
        const double admissible_jump = dt > kEps ?
          std::max(cfg.position_jump_threshold, cfg.v_max_safety * dt) :
          cfg.position_jump_threshold;
        position_jump = !std::isfinite(dt) || dt < -kEps || jump > admissible_jump;
        if (dt > kEps && !position_jump) {
          vx = dx / dt;
          vy = dy / dt;
          if (std::hypot(vx, vy) > cfg.v_max_safety) {
            invalid_velocity = true;
            if (track_recovery_completeness_) {
              last_message_has_invalid_sample_ = true;
            }
            vx = 0.0;
            vy = 0.0;
          }
        }
      }

      tracked.id = id;
      tracked.x = vehicle.position.x;
      tracked.y = vehicle.position.y;
      tracked.covariance_x = std::max(0.0, static_cast<double>(vehicle.covariance.x));
      tracked.covariance_y = std::max(0.0, static_cast<double>(vehicle.covariance.y));
      tracked.stamp_sec = sample_stamp;
      tracked.receipt_sec = receipt_sec;
      tracked.vx = vx;
      tracked.vy = vy;
      tracked.has_sample = true;
      tracked.position_jump = position_jump;
      tracked.invalid_velocity = invalid_velocity;
    }
  }

  std::vector<TrackedVehicle> active_vehicles(const double now_sec)
  {
    std::vector<TrackedVehicle> vehicles;
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & kv : vehicles_) {
      const auto & tracked = kv.second;
      if (!tracked.has_sample) {
        continue;
      }
      const double age_sec = now_sec - tracked.receipt_sec;
      if (!std::isfinite(age_sec) || age_sec < 0.0 || age_sec > cfg.timeout_sec) {
        continue;
      }
      vehicles.push_back(tracked);
    }
    return vehicles;
  }

  bool has_complete_message(const double now_sec, const std::size_t expected_vehicle_count)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!track_recovery_completeness_ || !last_message_receipt_sec_.has_value()) {
      return false;
    }
    const double age_sec = now_sec - last_message_receipt_sec_.value();
    return std::isfinite(age_sec) && age_sec >= 0.0 && age_sec <= cfg.timeout_sec &&
           last_message_vehicle_count_ == expected_vehicle_count &&
           !last_message_has_empty_id_ && !last_message_has_duplicate_id_ &&
           !last_message_has_invalid_sample_;
  }

  void reset_recovery_tracking()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    vehicles_.clear();
    last_message_receipt_sec_.reset();
    last_message_source_stamp_sec_.reset();
    last_message_vehicle_count_ = 0U;
    last_message_has_empty_id_ = false;
    last_message_has_duplicate_id_ = false;
    last_message_has_invalid_sample_ = false;
  }

  void reset_low_speed_target_lock()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    low_speed_locked_target_ey_.reset();
    low_speed_locked_side_sign_.reset();
  }

  void lock_low_speed_pass_side(const int pass_side_sign)
  {
    if (pass_side_sign == 0) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    low_speed_locked_side_sign_ = pass_side_sign;
  }

  void reset_low_speed_targets()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    low_speed_locked_target_ey_.reset();
    low_speed_locked_side_sign_.reset();
    last_target_ey_.reset();
    last_logged_local_path_corridor_enforced_.reset();
  }

  GapPlannerOutput plan_stopped_vehicle_local_path(
    const BicycleModel & model, const int ref_wp_id, const int N, const Eigen::VectorXd & base_lb,
    const Eigen::VectorXd & base_ub, const double now_sec, const V2XBehaviorConfig & behavior_cfg,
    const bool update_last_target = true)
  {
    GapPlannerOutput output;
    if (!cfg.enabled || !behavior_cfg.low_speed_local_path_enabled || N <= 0) {
      return output;
    }

    output.lb.assign(N, 0.0);
    output.ub.assign(N, 0.0);
    output.target_ey.assign(N, 0.0);
    output.target_active.assign(N, false);
    for (int i = 0; i < N; ++i) {
      output.lb[i] = base_lb[i];
      output.ub[i] = base_ub[i];
      output.target_ey[i] = 0.0;
    }

    struct ProjectedVehicle
    {
      TrackedVehicle vehicle;
      int wp_id{};
      double s{};
      double lateral{};
      double covariance_margin{};
    };

    const auto vehicles = active_vehicles(now_sec);
    if (vehicles.empty()) {
      return output;
    }

    const int configured_pass_side = low_speed_pass_side_sign();
    int locked_pass_side = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      locked_pass_side = low_speed_locked_side_sign_.value_or(0);
    }

    const double lookahead_distance =
      std::max(behavior_cfg.low_speed_avoidance_distance, behavior_cfg.low_speed_avoidance_lookahead_distance);
    const double scan_resolution = std::max(0.2, model.reference_path->resolution);
    const int scan_steps =
      std::max(N + 1, static_cast<int>(std::ceil(lookahead_distance / scan_resolution)) + 4);
    std::vector<ProjectedVehicle> projected;
    for (const auto & vehicle : vehicles) {
      const double self_distance =
        std::hypot(vehicle.x - model.temporal_state.x, vehicle.y - model.temporal_state.y);
      if (self_distance < cfg.self_filter_radius) {
        continue;
      }
      if (std::hypot(vehicle.vx, vehicle.vy) > behavior_cfg.low_speed_avoidance_max_front_speed) {
        continue;
      }

      double best_distance = std::numeric_limits<double>::infinity();
      int best_wp_id = ref_wp_id;
      for (int offset = 0; offset <= scan_steps; ++offset) {
        const int candidate_wp_id = ref_wp_id + offset;
        const auto & waypoint = model.reference_path->get_waypoint(candidate_wp_id);
        const double distance = std::hypot(vehicle.x - waypoint.x, vehicle.y - waypoint.y);
        if (distance < best_distance) {
          best_distance = distance;
          best_wp_id = candidate_wp_id;
        }
      }

      const auto & waypoint = model.reference_path->get_waypoint(best_wp_id);
      const double dx = vehicle.x - waypoint.x;
      const double dy = vehicle.y - waypoint.y;
      const double along_wp = std::cos(waypoint.psi) * dx + std::sin(waypoint.psi) * dy;
      const double s = forward_path_distance(*model.reference_path, ref_wp_id, best_wp_id) + along_wp;
      if (s <= 0.0 || s > lookahead_distance) {
        continue;
      }

      ProjectedVehicle projected_vehicle;
      projected_vehicle.vehicle = vehicle;
      projected_vehicle.wp_id = best_wp_id;
      projected_vehicle.s = s;
      projected_vehicle.lateral = -std::sin(waypoint.psi) * dx + std::cos(waypoint.psi) * dy;
      projected_vehicle.covariance_margin = std::max(vehicle.covariance_x, vehicle.covariance_y);
      projected.push_back(projected_vehicle);
    }

    if (projected.empty()) {
      return output;
    }

    std::sort(
      projected.begin(), projected.end(),
      [](const ProjectedVehicle & lhs, const ProjectedVehicle & rhs) { return lhs.s < rhs.s; });
    if (projected.front().s > behavior_cfg.low_speed_avoidance_distance) {
      return output;
    }

    struct SideCandidate
    {
      bool feasible{true};
      LateralInterval intersection{
        -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
      double min_width{std::numeric_limits<double>::infinity()};
    };

    // Low-speed stopped-vehicle bypass has its own clearance threshold. Using
    // the general racing gap minimum here made the dedicated parameter unable
    // to relax narrow car-to-wall gaps in Gate2-like layouts.
    const double required_width =
      std::max(0.0, behavior_cfg.low_speed_avoidance_min_gap_width);
    const auto evaluate_side = [&](const int pass_side_sign) {
      SideCandidate candidate;
      for (const auto & vehicle : projected) {
        const auto & waypoint = model.reference_path->get_waypoint(vehicle.wp_id);
        LateralInterval base{waypoint.lb_sm, waypoint.ub_sm};
        if (base.width() <= kEps) {
          base = {waypoint.lb, waypoint.ub};
        }
        const double wall_margin =
          std::min(std::max(0.0, cfg.wall_clearance_margin), std::max(0.0, base.width()) * 0.45);
        const double inflated_radius =
          std::max(0.0, cfg.vehicle_radius + cfg.prediction_margin + vehicle.covariance_margin);
        const double obstacle_lower = vehicle.lateral - inflated_radius;
        const double obstacle_upper = vehicle.lateral + inflated_radius;
        const LateralInterval interval = pass_side_sign < 0 ?
          LateralInterval{base.lower + wall_margin, obstacle_lower} :
          LateralInterval{obstacle_upper, base.upper - wall_margin};
        candidate.min_width = std::min(candidate.min_width, interval.width());
        candidate.intersection.lower = std::max(candidate.intersection.lower, interval.lower);
        candidate.intersection.upper = std::min(candidate.intersection.upper, interval.upper);
        if (
          interval.width() < required_width ||
          candidate.intersection.width() < required_width) {
          candidate.feasible = false;
        }
      }
      return candidate;
    };

    const auto side_target_ey = [&](const SideCandidate & candidate, const int side_sign) {
      const double interval_width = candidate.intersection.width();
      const double vehicle_margin =
        std::min(std::max(0.0, cfg.vehicle_side_target_margin), interval_width * 0.5);
      const double center_target = candidate.intersection.center();
      const double vehicle_side_target = side_sign < 0 ?
        candidate.intersection.upper - vehicle_margin :
        candidate.intersection.lower + vehicle_margin;
      return center_target + cfg.wall_avoidance_bias * (vehicle_side_target - center_target);
    };

    const auto right = evaluate_side(-1);
    const auto left = evaluate_side(1);
    int pass_side_sign = configured_pass_side != 0 ? configured_pass_side : locked_pass_side;
    SideCandidate selected_side;
    if (pass_side_sign != 0) {
      selected_side = pass_side_sign < 0 ? right : left;
    } else {
      if (right.feasible && left.feasible) {
        const auto side = v2x_overtake_core::select_reachable_low_speed_pass_side(
          v2x_overtake_core::LowSpeedPassSideRequest{
            model.spatial_state.e_y,
            {true, side_target_ey(left, 1), left.min_width},
            {true, side_target_ey(right, -1), right.min_width}});
        pass_side_sign = static_cast<int>(side);
        selected_side = pass_side_sign < 0 ? right : left;
      } else if (right.feasible) {
        pass_side_sign = -1;
        selected_side = right;
      } else if (left.feasible) {
        pass_side_sign = 1;
        selected_side = left;
      } else {
        selected_side.feasible = false;
      }
    }

    output.active = true;
    output.feasible = selected_side.feasible && pass_side_sign != 0;
    output.pass_side_sign = pass_side_sign;
    if (!output.feasible) {
      output.target_velocity_limit = std::max(0.0, cfg.no_gap_target_velocity);
      if (update_last_target) {
        reset_low_speed_targets();
      }
      return output;
    }

    const double interval_width = selected_side.intersection.width();
    const double vehicle_margin =
      std::min(std::max(0.0, cfg.vehicle_side_target_margin), interval_width * 0.5);
    const double center_target_ey = selected_side.intersection.center();
    const double vehicle_side_target_ey = pass_side_sign < 0 ?
      selected_side.intersection.upper - vehicle_margin :
      selected_side.intersection.lower + vehicle_margin;
    const double pass_target_ey =
      center_target_ey +
      cfg.wall_avoidance_bias * (vehicle_side_target_ey - center_target_ey);
    const double pass_begin_s =
      std::max(0.5, projected.front().s - 0.5 * std::max(0.0, cfg.vehicle_length));
    const double pass_end_s =
      projected.back().s + 0.5 * std::max(0.0, cfg.vehicle_length) +
      std::max(0.0, behavior_cfg.low_speed_local_path_pass_clearance);
    const double approach_distance =
      std::max(0.5, pass_begin_s * std::max(0.1, cfg.low_speed_pass_ramp_ratio));
    const bool enforce_pass_corridor =
      v2x_overtake_core::has_entered_low_speed_pass_corridor(
        model.spatial_state.e_y, selected_side.intersection.lower,
        selected_side.intersection.upper);
    output.pass_corridor_enforced = enforce_pass_corridor;
    output.pass_target_ey = pass_target_ey;
    for (int i = 0; i < N; ++i) {
      const double point_s = forward_path_distance(*model.reference_path, ref_wp_id, ref_wp_id + i + 1);
      const LateralInterval base_interval{base_lb[i], base_ub[i]};
      LateralInterval local_corridor = selected_side.intersection;
      if (!enforce_pass_corridor || point_s < approach_distance) {
        local_corridor.lower = std::min(model.spatial_state.e_y, selected_side.intersection.lower);
        local_corridor.upper = std::max(model.spatial_state.e_y, selected_side.intersection.upper);
      }
      output.lb[i] = std::max(base_interval.lower, local_corridor.lower);
      output.ub[i] = std::min(base_interval.upper, local_corridor.upper);
      if (output.ub[i] < output.lb[i]) {
        output.feasible = false;
        output.target_velocity_limit = std::max(0.0, cfg.no_gap_target_velocity);
        if (update_last_target) {
          reset_low_speed_targets();
        }
        return output;
      }

      double target_ey = pass_target_ey;
      if (point_s <= approach_distance) {
        const double progress = smoothstep(point_s / approach_distance);
        target_ey = model.spatial_state.e_y + progress * (pass_target_ey - model.spatial_state.e_y);
      }
      output.target_ey[i] = clip(target_ey, output.lb[i], output.ub[i]);
      output.target_active[i] = true;
    }
    output.target_velocity_limit =
      v2x_overtake_core::resolve_low_speed_pass_velocity(
        std::max(0.0, behavior_cfg.low_speed_avoidance_velocity),
        std::max(0.0, behavior_cfg.low_speed_avoidance_shift_velocity),
        enforce_pass_corridor);

    if (update_last_target) {
      if (
        last_logged_local_path_side_ != pass_side_sign ||
        std::abs(last_logged_local_path_target_ey_ - pass_target_ey) > 0.1 ||
        last_logged_local_path_corridor_enforced_ != enforce_pass_corridor) {
        RCLCPP_INFO(
          rclcpp::get_logger("mpc_controller"),
          "V2X local path selected: side=%s, target_ey=%.2f, center_ey=%.2f, "
          "vehicle_side_ey=%.2f, width=%.2f, approach_s=%.2f, pass_s=[%.2f, %.2f], "
          "vehicles=%zu, corridor=%s, required_width=%.2f, "
          "right=%s/min=%.2f/range=[%.2f,%.2f], left=%s/min=%.2f/range=[%.2f,%.2f]",
          pass_side_sign < 0 ? "right" : "left", pass_target_ey,
          center_target_ey, vehicle_side_target_ey, interval_width, approach_distance, pass_begin_s,
          pass_end_s, projected.size(), enforce_pass_corridor ? "hard" : "shift_target",
          required_width, right.feasible ? "ok" : "blocked", right.min_width,
          right.intersection.lower, right.intersection.upper,
          left.feasible ? "ok" : "blocked", left.min_width,
          left.intersection.lower, left.intersection.upper);
        last_logged_local_path_side_ = pass_side_sign;
        last_logged_local_path_target_ey_ = pass_target_ey;
        last_logged_local_path_corridor_enforced_ = enforce_pass_corridor;
      }
      std::lock_guard<std::mutex> lock(mutex_);
      low_speed_locked_side_sign_ = pass_side_sign;
      last_target_ey_ = output.target_ey.empty() ? pass_target_ey : output.target_ey.back();
    }
    return output;
  }

  GapPlannerOutput plan(
    const BicycleModel & model, const int ref_wp_id, const int N, const Eigen::VectorXd & base_lb,
    const Eigen::VectorXd & base_ub, const double now_sec, const bool update_last_target = true,
    const bool allow_vehicle_vehicle_gap = false,
    const double max_self_distance = std::numeric_limits<double>::infinity(),
    const int forced_pass_side_sign = 0,
    const std::optional<double> min_gap_width_override = std::nullopt)
  {
    GapPlannerOutput output;
    if (!cfg.enabled || N <= 0) {
      return output;
    }

    std::vector<TrackedVehicle> vehicles;
    const bool use_corridor_center_desired = allow_vehicle_vehicle_gap && base_lb.size() > 0 &&
                                            base_ub.size() > 0;
    double desired_ey = use_corridor_center_desired ?
      0.5 * (base_lb[0] + base_ub[0]) :
      model.spatial_state.e_y;
    // Gate2-like stopped vehicle rows need the target to be recomputed after each passed vehicle.
    // Keep last_target_ey_ only as a continuity hint; do not pin one absolute lateral target.
    const bool use_low_speed_target_lock = false;
    std::optional<double> low_speed_locked_target_ey;
    const int configured_low_speed_pass_side = low_speed_pass_side_sign();
    int low_speed_pass_side = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (use_low_speed_target_lock && low_speed_locked_target_ey_.has_value()) {
        low_speed_locked_target_ey = low_speed_locked_target_ey_;
        desired_ey = low_speed_locked_target_ey.value();
      } else if (!use_corridor_center_desired && last_target_ey_.has_value()) {
        desired_ey = last_target_ey_.value();
      }
      if (forced_pass_side_sign != 0) {
        low_speed_pass_side = forced_pass_side_sign;
      } else if (allow_vehicle_vehicle_gap) {
        low_speed_pass_side = configured_low_speed_pass_side != 0 ?
          configured_low_speed_pass_side :
          low_speed_locked_side_sign_.value_or(0);
      }
      for (const auto & kv : vehicles_) {
        const auto & tracked = kv.second;
        if (!tracked.has_sample) {
          continue;
        }
        if (now_sec - tracked.receipt_sec > cfg.timeout_sec) {
          continue;
        }
        vehicles.push_back(tracked);
      }
    }

    if (std::isfinite(max_self_distance)) {
      vehicles.erase(
        std::remove_if(
          vehicles.begin(), vehicles.end(),
          [&](const TrackedVehicle & vehicle) {
            return std::hypot(vehicle.x - model.temporal_state.x, vehicle.y - model.temporal_state.y) >
                   max_self_distance;
          }),
        vehicles.end());
    }

    if (vehicles.empty()) {
      return output;
    }

    if (!allow_vehicle_vehicle_gap && should_block_multi_front_gap(model, ref_wp_id, base_lb, base_ub, vehicles)) {
      output.active = true;
      output.feasible = false;
      output.reject_reason = "multi-front policy";
      output.target_velocity_limit = std::max(0.0, cfg.no_gap_target_velocity);
      if (update_last_target) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_target_ey_.reset();
      }
      return output;
    }

    output.lb.assign(N, 0.0);
    output.ub.assign(N, 0.0);
    output.target_ey.assign(N, 0.0);
    output.target_active.assign(N, false);
    bool any_obstacle_in_horizon = false;
    bool feasible = true;
    double selected_first_target = desired_ey;
    bool first_target_selected = false;
    std::size_t selected_first_target_index = 0;
    double path_prediction_time = 0.0;

    for (int i = 0; i < N; ++i) {
      const LateralInterval base{base_lb[i], base_ub[i]};
      output.lb[i] = base.lower;
      output.ub[i] = base.upper;
      output.target_ey[i] = base.center();
      if (base.width() <= kEps) {
        continue;
      }

      const auto & waypoint = model.reference_path->get_waypoint(ref_wp_id + i);
      double horizon_t = std::min(static_cast<double>(i + 1) * model.Ts, cfg.prediction_time);
      if (cfg.prediction_use_path_time) {
        double segment_distance = 0.0;
        if (i == 0) {
          segment_distance = std::hypot(
            waypoint.x - model.temporal_state.x,
            waypoint.y - model.temporal_state.y);
        } else {
          segment_distance = waypoint.distance_to(
            model.reference_path->get_waypoint(ref_wp_id + i - 1));
        }
        path_prediction_time = v2x_overtake_core::advance_prediction_time(
          v2x_overtake_core::PredictionTimeRequest{
            path_prediction_time, segment_distance,
            std::min(std::max(0.0, waypoint.v_ref), cfg.prediction_max_ego_speed),
            cfg.prediction_min_ego_speed, cfg.prediction_time});
        horizon_t = path_prediction_time;
      }
      std::vector<OccupiedInterval> occupied;
      for (const auto & vehicle : vehicles) {
        const double age = std::max(0.0, now_sec - vehicle.stamp_sec);
        const double pred_x = vehicle.x + vehicle.vx * (age + horizon_t);
        const double pred_y = vehicle.y + vehicle.vy * (age + horizon_t);
        const double self_distance =
          std::hypot(pred_x - model.temporal_state.x, pred_y - model.temporal_state.y);
        if (self_distance < cfg.self_filter_radius) {
          continue;
        }

        const double dx = pred_x - waypoint.x;
        const double dy = pred_y - waypoint.y;
        const double longitudinal = std::cos(waypoint.psi) * dx + std::sin(waypoint.psi) * dy;
        const double lateral = -std::sin(waypoint.psi) * dx + std::cos(waypoint.psi) * dy;
        const double covariance_margin = std::max(vehicle.covariance_x, vehicle.covariance_y);
        const double obstacle_radius =
          std::max(0.0, cfg.vehicle_radius + cfg.prediction_margin + covariance_margin);
        const double longitudinal_radius =
          0.5 * std::max(0.0, cfg.vehicle_length) + 0.5 * model.length +
          cfg.prediction_margin + covariance_margin;
        if (std::abs(longitudinal) > longitudinal_radius) {
          continue;
        }
        if (lateral + obstacle_radius < base.lower || lateral - obstacle_radius > base.upper) {
          continue;
        }
        occupied.push_back({lateral - obstacle_radius, lateral + obstacle_radius, self_distance});
      }

      if (occupied.empty()) {
        continue;
      }

      any_obstacle_in_horizon = true;
      const auto free_intervals = compute_free_intervals(
        base, occupied, allow_vehicle_vehicle_gap, min_gap_width_override);
      if (free_intervals.empty()) {
        feasible = false;
        output.reject_reason = allow_vehicle_vehicle_gap ?
          "gap width" : "gap width or vehicle-vehicle policy";
        break;
      }
      const auto pass_side_intervals = filter_by_pass_side(free_intervals, base, low_speed_pass_side);
      if (low_speed_pass_side != 0 && pass_side_intervals.empty()) {
        feasible = false;
        output.reject_reason = "pass-side gap unavailable";
        break;
      }
      const auto & selectable_intervals =
        pass_side_intervals.empty() ? free_intervals : pass_side_intervals;
      double selection_desired_ey = desired_ey;
      const bool prefer_locked_target = use_low_speed_target_lock && low_speed_locked_target_ey.has_value();
      if (prefer_locked_target) {
        selection_desired_ey = low_speed_locked_target_ey.value();
      }
      const bool prefer_wide_gap =
        allow_vehicle_vehicle_gap && low_speed_pass_side == 0 && !prefer_locked_target;
      const auto selected = select_interval(
        selectable_intervals, selection_desired_ey, prefer_wide_gap, prefer_locked_target);
      if (allow_vehicle_vehicle_gap && low_speed_pass_side == 0) {
        low_speed_pass_side = infer_pass_side_sign(base, selected.interval);
      }
      if ((allow_vehicle_vehicle_gap || forced_pass_side_sign != 0) && output.pass_side_sign == 0) {
        output.pass_side_sign = low_speed_pass_side;
      }
      const auto adjusted = apply_wall_clearance(base, selected.interval, false);
      output.lb[i] = adjusted.lower;
      output.ub[i] = adjusted.upper;
      output.target_ey[i] = prefer_locked_target ?
        clip(low_speed_locked_target_ey.value(), adjusted.lower, adjusted.upper) :
        select_target_ey(base, selected.interval, adjusted, allow_vehicle_vehicle_gap);
      output.target_active[i] = true;
      desired_ey = output.target_ey[i];
      if (!first_target_selected) {
        selected_first_target = output.target_ey[i];
        selected_first_target_index = static_cast<std::size_t>(i);
        first_target_selected = true;
      }
    }

    if (!any_obstacle_in_horizon) {
      if (allow_vehicle_vehicle_gap && update_last_target && low_speed_pass_side != 0) {
        output.active = true;
        output.feasible = true;
        apply_low_speed_pass_side_hold(output, model.spatial_state.e_y, low_speed_pass_side);
        {
          std::lock_guard<std::mutex> lock(mutex_);
          low_speed_locked_side_sign_ = low_speed_pass_side;
          if (!output.target_ey.empty()) {
            last_target_ey_ = output.target_ey.back();
          }
        }
        return output;
      }
      if (use_low_speed_target_lock && update_last_target) {
        std::lock_guard<std::mutex> lock(mutex_);
        low_speed_locked_target_ey_.reset();
      }
      return GapPlannerOutput{};
    }

    output.active = true;
    output.feasible = feasible;
    if (!feasible) {
      output.target_velocity_limit = std::max(0.0, cfg.no_gap_target_velocity);
    }

    if (feasible && use_low_speed_target_lock) {
      if (!low_speed_locked_target_ey.has_value() && first_target_selected) {
        low_speed_locked_target_ey = selected_first_target;
      }
      if (low_speed_locked_target_ey.has_value()) {
        apply_locked_target(output, low_speed_locked_target_ey.value());
      }
    }
    if (feasible && allow_vehicle_vehicle_gap && update_last_target && first_target_selected) {
      output.pass_side_sign = low_speed_pass_side;
      apply_low_speed_pass_ramp(output, model.spatial_state.e_y, selected_first_target);
    }
    if (
      feasible && !allow_vehicle_vehicle_gap && update_last_target && first_target_selected &&
      cfg.overtake_target_ramp_enabled) {
      apply_overtake_target_ramp(
        output, model.spatial_state.e_y, selected_first_target, selected_first_target_index);
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (feasible && update_last_target) {
        if (use_low_speed_target_lock && low_speed_locked_target_ey.has_value()) {
          low_speed_locked_target_ey_ = low_speed_locked_target_ey.value();
          last_target_ey_ = low_speed_locked_target_ey.value();
        } else {
          last_target_ey_ = selected_first_target;
        }
        if (allow_vehicle_vehicle_gap && low_speed_pass_side != 0) {
          low_speed_locked_side_sign_ = low_speed_pass_side;
        }
      } else if (!feasible && update_last_target) {
        last_target_ey_.reset();
        if (use_low_speed_target_lock) {
          low_speed_locked_target_ey_.reset();
        }
        if (allow_vehicle_vehicle_gap) {
          low_speed_locked_side_sign_.reset();
        }
      }
    }
    return output;
  }

  V2XGapPlannerConfig cfg;

private:
  double forward_path_distance(const ReferencePath & path, const int start_wp_id, const int end_wp_id) const
  {
    if (end_wp_id <= start_wp_id) {
      return 0.0;
    }

    double distance = 0.0;
    for (int wp_id = start_wp_id; wp_id < end_wp_id; ++wp_id) {
      const auto & current = path.get_waypoint(wp_id);
      const auto & next = path.get_waypoint(wp_id + 1);
      distance += next.distance_to(current);
    }
    return distance;
  }

  bool should_block_multi_front_gap(
    const BicycleModel & model, const int ref_wp_id, const Eigen::VectorXd & base_lb,
    const Eigen::VectorXd & base_ub, const std::vector<TrackedVehicle> & vehicles) const
  {
    if (cfg.multi_front_gap_enabled || cfg.multi_front_gap_distance <= 0.0) {
      return false;
    }
    if (base_lb.size() == 0 || base_ub.size() == 0) {
      return false;
    }

    const auto & waypoint = model.reference_path->get_waypoint(ref_wp_id);
    const double cos_yaw = std::cos(waypoint.psi);
    const double sin_yaw = std::sin(waypoint.psi);
    const double lateral_range =
      std::max({std::abs(base_lb[0]), std::abs(base_ub[0]), model.width}) +
      cfg.vehicle_radius + cfg.prediction_margin;

    int front_vehicle_count = 0;
    for (const auto & vehicle : vehicles) {
      const double self_distance =
        std::hypot(vehicle.x - model.temporal_state.x, vehicle.y - model.temporal_state.y);
      if (self_distance < cfg.self_filter_radius) {
        continue;
      }

      const double dx = vehicle.x - model.temporal_state.x;
      const double dy = vehicle.y - model.temporal_state.y;
      const double longitudinal = cos_yaw * dx + sin_yaw * dy;
      const double lateral = -sin_yaw * dx + cos_yaw * dy;
      if (longitudinal <= 0.0 || longitudinal > cfg.multi_front_gap_distance) {
        continue;
      }
      if (std::abs(lateral) > lateral_range) {
        continue;
      }
      ++front_vehicle_count;
      if (front_vehicle_count >= 2) {
        return true;
      }
    }
    return false;
  }

  std::vector<GapCandidate> compute_free_intervals(
    const LateralInterval & base, std::vector<OccupiedInterval> occupied,
    const bool allow_vehicle_vehicle_gap,
    const std::optional<double> min_gap_width_override) const
  {
    std::vector<OccupiedInterval> merged;
    for (auto & interval : occupied) {
      interval.lower = std::max(interval.lower, base.lower);
      interval.upper = std::min(interval.upper, base.upper);
      if (interval.upper <= interval.lower) {
        continue;
      }
      merged.push_back(interval);
    }
    std::sort(
      merged.begin(), merged.end(),
      [](const OccupiedInterval & lhs, const OccupiedInterval & rhs) {
        return lhs.lower < rhs.lower;
      });

    std::vector<OccupiedInterval> compacted;
    for (const auto & interval : merged) {
      if (compacted.empty() || interval.lower > compacted.back().upper) {
        compacted.push_back(interval);
        continue;
      }
      auto & last = compacted.back();
      last.upper = std::max(last.upper, interval.upper);
      last.distance_from_self = std::min(last.distance_from_self, interval.distance_from_self);
    }

    std::vector<GapCandidate> free_intervals;
    double cursor = base.lower;
    bool cursor_from_vehicle = false;
    double cursor_vehicle_distance = std::numeric_limits<double>::infinity();
    for (const auto & interval : compacted) {
      if (interval.lower > cursor) {
        free_intervals.push_back(
          {{cursor, interval.lower}, cursor_from_vehicle, true, cursor_vehicle_distance,
            interval.distance_from_self});
      }
      cursor = std::max(cursor, interval.upper);
      cursor_from_vehicle = true;
      cursor_vehicle_distance = interval.distance_from_self;
    }
    if (cursor < base.upper) {
      free_intervals.push_back(
        {{cursor, base.upper}, cursor_from_vehicle, false, cursor_vehicle_distance,
          std::numeric_limits<double>::infinity()});
    }

    std::vector<GapCandidate> filtered;
    const double min_width = min_gap_width_override.has_value() ?
      std::max(0.0, min_gap_width_override.value()) : std::max(0.0, cfg.min_gap_width);
    const double vehicle_vehicle_min_width = allow_vehicle_vehicle_gap ?
      min_width :
      std::max(min_width, std::max(0.0, cfg.vehicle_vehicle_gap_min_width));
    for (const auto & candidate : free_intervals) {
      double required_width = min_width;
      if (candidate.is_vehicle_vehicle_gap()) {
        if (!cfg.vehicle_vehicle_gap_enabled && !allow_vehicle_vehicle_gap) {
          continue;
        }
        required_width = vehicle_vehicle_min_width;
        if (
          !allow_vehicle_vehicle_gap && cfg.vehicle_vehicle_gap_min_distance > 0.0 &&
          candidate.nearest_vehicle_distance() < cfg.vehicle_vehicle_gap_min_distance) {
          continue;
        }
      }
      if (candidate.interval.width() >= required_width) {
        filtered.push_back(candidate);
      }
    }
    return filtered;
  }

  void apply_locked_target(GapPlannerOutput & output, const double target_ey) const
  {
    for (std::size_t i = 0; i < output.target_ey.size(); ++i) {
      if (i >= output.lb.size() || i >= output.ub.size()) {
        continue;
      }
      if (i >= output.target_active.size() || !output.target_active[i]) {
        continue;
      }
      if (output.ub[i] - output.lb[i] <= kEps) {
        continue;
      }
      output.target_ey[i] = clip(target_ey, output.lb[i], output.ub[i]);
    }
  }

  int low_speed_pass_side_sign() const
  {
    if (cfg.low_speed_pass_side == "left") {
      return 1;
    }
    if (cfg.low_speed_pass_side == "right") {
      return -1;
    }
    return 0;
  }

  int infer_pass_side_sign(const LateralInterval & base, const LateralInterval & selected) const
  {
    const double delta = selected.center() - base.center();
    if (delta > kEps) {
      return 1;
    }
    if (delta < -kEps) {
      return -1;
    }
    return 0;
  }

  std::vector<GapCandidate> filter_by_pass_side(
    const std::vector<GapCandidate> & intervals, const LateralInterval & base,
    const int pass_side_sign) const
  {
    if (pass_side_sign == 0) {
      return {};
    }

    std::vector<GapCandidate> filtered;
    const double base_center = base.center();
    for (const auto & candidate : intervals) {
      const double delta = candidate.interval.center() - base_center;
      if ((pass_side_sign > 0 && delta >= -kEps) || (pass_side_sign < 0 && delta <= kEps)) {
        filtered.push_back(candidate);
      }
    }
    return filtered;
  }

  void apply_low_speed_pass_ramp(
    GapPlannerOutput & output, const double start_ey, const double target_ey) const
  {
    const std::size_t count = std::min({output.lb.size(), output.ub.size(), output.target_ey.size()});
    if (count == 0) {
      return;
    }
    const double ramp_ratio = std::max(0.1, cfg.low_speed_pass_ramp_ratio);
    for (std::size_t i = 0; i < count; ++i) {
      if (output.ub[i] - output.lb[i] <= kEps) {
        continue;
      }
      const double progress = clip(
        (static_cast<double>(i) + 1.0) / (static_cast<double>(count) * ramp_ratio), 0.0, 1.0);
      const double ramp_target = start_ey + progress * (target_ey - start_ey);
      output.target_ey[i] = clip(ramp_target, output.lb[i], output.ub[i]);
      if (i < output.target_active.size()) {
        output.target_active[i] = true;
      }
    }
  }

  void apply_overtake_target_ramp(
    GapPlannerOutput & output, const double start_ey, const double target_ey,
    const std::size_t target_index) const
  {
    const std::size_t count =
      std::min({output.lb.size(), output.ub.size(), output.target_ey.size(), target_index + 1});
    if (count == 0) {
      return;
    }
    const double ramp_ratio = std::max(0.1, cfg.overtake_target_ramp_ratio);
    const double denominator = std::max(1.0, static_cast<double>(count) * ramp_ratio);
    for (std::size_t i = 0; i < count; ++i) {
      if (output.ub[i] - output.lb[i] <= kEps) {
        continue;
      }
      const double progress = clip((static_cast<double>(i) + 1.0) / denominator, 0.0, 1.0);
      const double ramp_target = start_ey + progress * (target_ey - start_ey);
      output.target_ey[i] = clip(ramp_target, output.lb[i], output.ub[i]);
      if (i < output.target_active.size()) {
        output.target_active[i] = true;
      }
    }
  }

  void apply_low_speed_pass_side_hold(
    GapPlannerOutput & output, const double start_ey, const int pass_side_sign) const
  {
    const std::size_t count = std::min({output.lb.size(), output.ub.size(), output.target_ey.size()});
    if (count == 0 || pass_side_sign == 0) {
      return;
    }
    for (std::size_t i = 0; i < count; ++i) {
      if (output.ub[i] - output.lb[i] <= kEps) {
        continue;
      }
      const double center = 0.5 * (output.lb[i] + output.ub[i]);
      const double side_edge = pass_side_sign > 0 ? output.ub[i] : output.lb[i];
      const double side_target = 0.5 * (center + side_edge);
      const double ramp_ratio = std::max(0.1, cfg.low_speed_pass_ramp_ratio);
      const double progress = clip(
        (static_cast<double>(i) + 1.0) / (static_cast<double>(count) * ramp_ratio), 0.0, 1.0);
      const double ramp_target = start_ey + progress * (side_target - start_ey);
      output.target_ey[i] = clip(ramp_target, output.lb[i], output.ub[i]);
      if (i < output.target_active.size()) {
        output.target_active[i] = true;
      }
    }
  }

  GapCandidate select_interval(
    const std::vector<GapCandidate> & intervals, const double desired_ey,
    const bool prefer_wide_gap, const bool prefer_containing_desired) const
  {
    auto best = intervals.front();
    double best_score = std::numeric_limits<double>::infinity();
    for (const auto & candidate : intervals) {
      const auto & interval = candidate.interval;
      const double center = interval.center();
      const bool contains_desired =
        interval.lower - kEps <= desired_ey && desired_ey <= interval.upper + kEps;
      const double distance_to_desired = contains_desired ?
        0.0 :
        std::min(std::abs(desired_ey - interval.lower), std::abs(desired_ey - interval.upper));
      const double score = prefer_containing_desired ?
        (contains_desired ? 0.0 : 1000.0) + 8.0 * distance_to_desired +
          0.2 * std::abs(center - desired_ey) - 0.2 * interval.width() :
        prefer_wide_gap ?
        0.6 * std::abs(center - desired_ey) + 0.2 * std::abs(center) - 2.0 * interval.width() :
        4.0 * std::abs(center - desired_ey) + 0.5 * std::abs(center) - 0.1 * interval.width();
      if (score < best_score) {
        best = candidate;
        best_score = score;
      }
    }
    return best;
  }

  LateralInterval apply_wall_clearance(
    const LateralInterval & base, const LateralInterval & selected,
    const bool skip_wall_clearance) const
  {
    LateralInterval adjusted = selected;
    if (skip_wall_clearance) {
      return adjusted;
    }
    const double margin = std::min(std::max(0.0, cfg.wall_clearance_margin), selected.width() * 0.5);
    if (margin <= kEps) {
      return adjusted;
    }
    const bool lower_is_wall = std::abs(selected.lower - base.lower) <= kEps;
    const bool upper_is_wall = std::abs(selected.upper - base.upper) <= kEps;
    if (lower_is_wall && !upper_is_wall) {
      adjusted.lower += margin;
    } else if (upper_is_wall && !lower_is_wall) {
      adjusted.upper -= margin;
    }
    return adjusted;
  }

  double select_target_ey(
    const LateralInterval & base, const LateralInterval & selected,
    const LateralInterval & adjusted, const bool prefer_gap_center) const
  {
    const double center = adjusted.center();
    const double bias = prefer_gap_center ? 0.0 : clip(cfg.wall_avoidance_bias, 0.0, 1.0);
    if (bias <= kEps || adjusted.width() <= kEps) {
      return center;
    }

    const bool lower_is_wall = std::abs(selected.lower - base.lower) <= kEps;
    const bool upper_is_wall = std::abs(selected.upper - base.upper) <= kEps;
    double vehicle_side_target = center;
    const double vehicle_margin =
      std::min(std::max(0.0, cfg.vehicle_side_target_margin), adjusted.width() * 0.5);
    if (lower_is_wall && !upper_is_wall) {
      vehicle_side_target = adjusted.upper - vehicle_margin;
    } else if (upper_is_wall && !lower_is_wall) {
      vehicle_side_target = adjusted.lower + vehicle_margin;
    } else {
      return center;
    }

    const double target = (1.0 - bias) * center + bias * vehicle_side_target;
    return clip(target, adjusted.lower, adjusted.upper);
  }

  std::mutex mutex_;
  std::unordered_map<std::string, TrackedVehicle> vehicles_;
  std::optional<double> last_message_receipt_sec_;
  std::optional<double> last_message_source_stamp_sec_;
  std::size_t last_message_vehicle_count_{0U};
  bool last_message_has_empty_id_{false};
  bool last_message_has_duplicate_id_{false};
  bool last_message_has_invalid_sample_{false};
  bool track_recovery_completeness_{false};
  std::optional<double> last_target_ey_;
  std::optional<double> low_speed_locked_target_ey_;
  std::optional<int> low_speed_locked_side_sign_;
  int last_logged_local_path_side_{0};
  double last_logged_local_path_target_ey_{std::numeric_limits<double>::infinity()};
  std::optional<bool> last_logged_local_path_corridor_enforced_;
};

struct MpcConfig
{
  int N{};
  Eigen::Vector3d Q;
  Eigen::Vector2d R;
  Eigen::Vector3d QN;
  double global_v_max{};
  double global_v_max_kmh{};
  double v_max{};
  double v_max_kmh{};
  double a_min{};
  double a_max{};
  double ay_max{};
  double delta_max{};
  double steer_rate_max{};
  double control_rate{};
  int solver_failure_steering_hold_cycles{4};
  double odom_timeout_sec{0.5};
  double min_linearization_speed_mps{0.5};
  double steering_tire_angle_gain_var{};
  double accel_low_pass_gain{};
  double steer_low_pass_gain{};
  int wp_id_offset{};
  int wp_id_low_offset{};
  double wp_id_low_speed{};
  double wp_id_low_speed_kmh{};
  double center_bias{1.0};
  double safety_margin_scale{1.0};
  int ros_domain_id{-1};
  bool domain_v_max_applied{false};
  bool domain_a_max_applied{false};
  double domain_start_v_max{std::numeric_limits<double>::infinity()};
  double domain_start_v_max_kmh{std::numeric_limits<double>::infinity()};
  double domain_start_v_max_duration{0.0};
  bool domain_start_v_max_applied{false};
  V2XGapPlannerConfig v2x_gap;
  V2XBehaviorConfig v2x_behavior;
  bool use_max_kappa_pred{};
};

struct MpcProblem
{
  Eigen::VectorXd q;
  Eigen::VectorXd l;
  Eigen::VectorXd u;
  Eigen::SparseMatrix<double> P;
  Eigen::SparseMatrix<double> A;
  int N{};
  int base_wp_id{};
  int planning_wp_id{};
  int ref_wp_id{};
};

struct MPC
{
  MPC(
    BicycleModel * model_in, const MpcConfig & cfg_in, const bool use_obstacle_avoidance_in,
    const bool use_path_constraints_topic_in)
  : model(model_in),
    cfg(cfg_in),
    start_grid_grace_guard_(cfg_in.v2x_behavior.start_grid_grace_time),
    use_obstacle_avoidance(use_obstacle_avoidance_in),
    use_path_constraints_topic(use_path_constraints_topic_in),
    current_control(Eigen::VectorXd::Zero(2 * cfg_in.N))
  {
    (void)use_obstacle_avoidance;
    (void)use_path_constraints_topic;
    model->reference_path->update_simple_path_constraints(cfg.N, model->safety_margin);
  }

  void set_gap_planner(V2XGapPlanner * planner)
  {
    gap_planner = planner;
  }

  void update_v_max(const double v_max)
  {
    cfg.v_max = v_max;
  }

  void update_ay_max(const double ay_max)
  {
    cfg.ay_max = ay_max;
  }

  void update_wp_id_offset(const int wp_id_offset)
  {
    cfg.wp_id_offset = wp_id_offset;
  }

  void update_current_speed(const double current_speed_mps)
  {
    current_speed_mps_ = std::max(0.0, current_speed_mps);
  }

  start_grid_grace::Transition arm_start_grid_grace(const double start_sec)
  {
    return start_grid_grace_guard_.arm(start_sec);
  }

  start_grid_grace::Transition prepare_start_grid_grace()
  {
    return start_grid_grace_guard_.prepare();
  }

  start_grid_grace::Transition clear_start_grid_grace()
  {
    const auto transition = start_grid_grace_guard_.clear();
    if (transition == start_grid_grace::Transition::Cleared) {
      start_grid_stop_suppressed_ = false;
      start_grid_emergency_override_logged_ = false;
      start_grid_initial_target_id_.reset();
    }
    return transition;
  }

  void reset_control_history(const double steering)
  {
    previous_steering = std::isfinite(steering) ?
      clip(steering, -std::abs(cfg.delta_max), std::abs(cfg.delta_max)) : 0.0;
    current_control = Eigen::VectorXd::Zero(2 * std::max(0, cfg.N));
    for (int index = 0; index < cfg.N; ++index) {
      current_control[2 * index + 1] = previous_steering;
    }
    current_prediction.first.clear();
    current_prediction.second.clear();
    failure_fallback_speed_ = 0.0;
    infeasibility_counter = 0;
    overtake_infeasibility_counter_ = 0;
    last_control_was_fallback_ = true;
  }

  bool last_control_was_fallback() const
  {
    return last_control_was_fallback_;
  }

  const V2XBehaviorOutput & last_v2x_behavior_output() const
  {
    return last_v2x_behavior_output_;
  }

  void reset_after_external_maneuver(const double now_sec, const double steering)
  {
    reset_control_history(steering);
    v2x_behavior_state = V2XBehaviorState::Cruise;
    v2x_behavior_state_initialized = false;
    last_v2x_behavior_state_change_sec = now_sec;
    low_speed_avoidance_stall_since_sec_ = std::numeric_limits<double>::quiet_NaN();
    low_speed_avoidance_stall_last_update_sec_ = std::numeric_limits<double>::quiet_NaN();
    low_speed_avoidance_stall_cooldown_until_sec_ = now_sec;
    front_hazard_hold_until_sec_ = now_sec;
    front_hazard_hold_target_id_.clear();
    overtake_locked_target_ey_.reset();
    overtake_locked_side_sign_ = 0;
    overtake_entry_speed_.reset();
    overtake_solver_recovery_active_ = false;
    overtake_curve_cooldown_until_sec_ = now_sec;
    reset_overtake_line_state(now_sec, "external recovery completed");
    if (gap_planner != nullptr) {
      gap_planner->reset_low_speed_targets();
    }
    last_v2x_behavior_output_ = V2XBehaviorOutput{};
  }

  V2XBehaviorOutput evaluate_v2x_behavior(
    const int ref_wp_id, const int N, const Eigen::VectorXd & lb, const Eigen::VectorXd & ub,
    const double now_sec)
  {
    V2XBehaviorOutput output;
    if (!cfg.v2x_behavior.enabled || gap_planner == nullptr || N <= 0) {
      return output;
    }

    const auto start_grid_evaluation = start_grid_grace_guard_.evaluate(now_sec);
    const bool start_grid_grace_active = start_grid_evaluation.active;
    if (start_grid_evaluation.transition == start_grid_grace::Transition::Expired) {
      start_grid_initial_target_id_.reset();
      RCLCPP_INFO(
        rclcpp::get_logger("mpc_controller"),
        "Start grid grace expired after %.2f s",
        start_grid_evaluation.elapsed_sec);
    } else if (
      start_grid_evaluation.transition == start_grid_grace::Transition::ClockRejected)
    {
      start_grid_initial_target_id_.reset();
      RCLCPP_WARN(
        rclcpp::get_logger("mpc_controller"),
        "Start grid grace rejected because the ROS clock is invalid or moved backwards");
    }
    output.start_grid_grace_active = start_grid_grace_active;
    output.ego_speed = current_speed_mps_;
    const bool front_hazard_hold_session_active =
      start_grid_evaluation.phase == start_grid_grace::Phase::Disabled ||
      start_grid_evaluation.phase == start_grid_grace::Phase::Grace ||
      start_grid_evaluation.phase == start_grid_grace::Phase::Expired;
    if (!front_hazard_hold_session_active) {
      front_hazard_hold_until_sec_ = now_sec;
      front_hazard_hold_target_id_.clear();
    }
    const bool low_speed_avoidance_cooldown_active =
      std::isfinite(now_sec) && now_sec < low_speed_avoidance_stall_cooldown_until_sec_;
    output.low_speed_avoidance_cooldown_active = low_speed_avoidance_cooldown_active;

    const auto update_front_hazard_hold = [
      &output, this, now_sec, front_hazard_hold_session_active](
        const bool hazard_observed, const bool target_rear_clear,
        const std::string & observed_target_id, const bool target_observed_safe) {
        if (hazard_observed && !observed_target_id.empty()) {
          front_hazard_hold_target_id_ = observed_target_id;
        }
        const auto resolution = v2x_overtake_core::update_front_hazard_hold(
          v2x_overtake_core::FrontHazardHoldRequest{
            cfg.v2x_behavior.front_hazard_hold_enabled &&
            front_hazard_hold_session_active,
            hazard_observed,
            target_rear_clear,
            now_sec,
            front_hazard_hold_until_sec_,
            cfg.v2x_behavior.front_hazard_hold_sec,
            target_observed_safe});
        front_hazard_hold_until_sec_ = resolution.until_sec;
        output.front_hazard_hold_active = resolution.active;
        output.front_hazard_hold_remaining_sec = resolution.remaining_sec;
        output.front_hazard_hold_target_id = front_hazard_hold_target_id_;
        if (!resolution.active) {
          front_hazard_hold_target_id_.clear();
        }
        return resolution.active;
      };

    const auto vehicles = gap_planner->active_vehicles(now_sec);
    output.active_vehicle_count = vehicles.size();
    if (vehicles.empty()) {
      update_start_grid_suppression_diagnostics(
        false, false, std::string{}, std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(), 0.0);
      if (update_front_hazard_hold(false, false, std::string{}, false)) {
        output.state = V2XBehaviorState::SafetyBrake;
        output.target_vehicle_id = output.front_hazard_hold_target_id;
        output.reason = "front hazard target temporarily missing";
      } else {
        output.reason = "no active vehicles";
      }
      return commit_v2x_behavior_state(output, now_sec);
    }

    const bool overtake_forbidden_wp = is_overtake_forbidden_wp(ref_wp_id);
    const bool overtake_forbidden =
      overtake_forbidden_wp ||
      is_overtake_forbidden_curvature(
        ref_wp_id, N, cfg.v2x_behavior.overtake_forbidden_curve_lookahead_distance);
    const bool front_decel_curve_guard =
      overtake_forbidden_wp ||
      is_overtake_forbidden_curvature(
        ref_wp_id, N, cfg.v2x_behavior.front_decel_guard_curve_lookahead_distance);
    const bool overtake_cooldown_active =
      cfg.v2x_behavior.overtake_curve_cooldown_enabled &&
      now_sec < overtake_curve_cooldown_until_sec_;
    output.overtake_forbidden = overtake_forbidden;
    output.overtake_forbidden_wp = overtake_forbidden_wp;
    output.front_decel_curve_guard = front_decel_curve_guard;
    output.overtake_cooldown_active = overtake_cooldown_active;
    const auto & waypoint = model->reference_path->get_waypoint(ref_wp_id);
    const double cos_yaw = std::cos(waypoint.psi);
    const double sin_yaw = std::sin(waypoint.psi);
    const double corridor_lateral_range =
      std::max({std::abs(lb[0]), std::abs(ub[0]), model->width}) +
      cfg.v2x_gap.vehicle_radius + cfg.v2x_gap.prediction_margin;
    const double side_longitudinal_range =
      model->length + cfg.v2x_gap.vehicle_radius + cfg.v2x_gap.prediction_margin;
    const double danger_lateral_range = cfg.v2x_gap.vehicle_radius + cfg.v2x_gap.prediction_margin;
    const double front_lateral_range =
      start_grid_grace::resolve_front_lateral_range(
      start_grid_grace::FrontLateralRangeContext{
        start_grid_grace_active,
        front_decel_curve_guard,
        corridor_lateral_range,
        danger_lateral_range,
        std::max(0.0, cfg.v2x_behavior.front_decel_guard_curve_lateral_margin)});
    const double brake_decel = std::max(kEps, std::abs(cfg.a_min));
    const double stopped_stop_distance =
      current_speed_mps_ * current_speed_mps_ / (2.0 * brake_decel) +
      std::max(0.0, cfg.v2x_behavior.safety_brake_margin);
    const double stopped_safety_brake_distance =
      std::max(cfg.v2x_behavior.safety_brake_distance, stopped_stop_distance);
    double front_detection_distance =
      std::max(cfg.v2x_behavior.follow_distance, stopped_safety_brake_distance);
    if (front_decel_curve_guard) {
      front_detection_distance = std::max(
        front_detection_distance,
        std::max(0.0, cfg.v2x_behavior.front_decel_guard_curve_distance));
    }
    if (cfg.v2x_behavior.front_progress_detection_enabled) {
      front_detection_distance = std::max(
        front_detection_distance, cfg.v2x_behavior.front_progress_detection_distance);
    }

    std::vector<v2x_overtake_core::CoursePoint> course_progress_path;
    if (cfg.v2x_behavior.front_progress_detection_enabled) {
      course_progress_path.reserve(
        static_cast<std::size_t>(model->reference_path->n_waypoints));
      for (int wp_id = 0; wp_id < model->reference_path->n_waypoints; ++wp_id) {
        const auto & course_waypoint = model->reference_path->get_waypoint(wp_id);
        course_progress_path.push_back({course_waypoint.x, course_waypoint.y});
      }
    }

    bool has_front_vehicle = false;
    bool has_danger_vehicle = false;
    bool has_side_vehicle = false;
    bool held_target_rear_clear = false;
    bool has_low_speed_clearance_vehicle = false;
    double nearest_front_distance = std::numeric_limits<double>::infinity();
    double nearest_front_speed = std::numeric_limits<double>::infinity();
    double nearest_front_lateral = std::numeric_limits<double>::infinity();
    double nearest_front_local_longitudinal = std::numeric_limits<double>::infinity();
    bool nearest_front_progress_used = false;
    std::string nearest_front_id;
    std::string nearest_side_id;
    double nearest_side_abs_longitudinal = std::numeric_limits<double>::infinity();
    double nearest_side_course_longitudinal = std::numeric_limits<double>::infinity();
    const bool continuing_low_speed_avoidance =
      v2x_behavior_state_initialized && v2x_behavior_state == V2XBehaviorState::LowSpeedAvoidance;
    const bool track_low_speed_clearance =
      continuing_low_speed_avoidance || low_speed_shift_control_was_active_;

    for (const auto & vehicle : vehicles) {
      const double self_distance =
        std::hypot(vehicle.x - model->temporal_state.x, vehicle.y - model->temporal_state.y);
      if (self_distance < cfg.v2x_gap.self_filter_radius) {
        continue;
      }

      const double dx = vehicle.x - model->temporal_state.x;
      const double dy = vehicle.y - model->temporal_state.y;
      const double longitudinal = cos_yaw * dx + sin_yaw * dy;
      const double lateral = -sin_yaw * dx + cos_yaw * dy;
      const double vehicle_speed = std::hypot(vehicle.vx, vehicle.vy);
      v2x_overtake_core::ForwardCourseProjection course_projection;
      if (
        cfg.v2x_behavior.front_progress_detection_enabled &&
        !course_progress_path.empty() && std::isfinite(vehicle.x) &&
        std::isfinite(vehicle.y) && std::isfinite(vehicle.vx) &&
        std::isfinite(vehicle.vy))
      {
        course_projection = v2x_overtake_core::project_forward_course_progress(
          course_progress_path,
          v2x_overtake_core::ForwardCourseProjectionRequest{
            static_cast<std::size_t>(std::max(0, model->wp_id)),
            model->reference_path->circular,
            model->temporal_state.x,
            model->temporal_state.y,
            vehicle.x,
            vehicle.y,
            vehicle.vx,
            vehicle.vy,
            cfg.v2x_behavior.front_progress_lookbehind_distance,
            front_detection_distance,
            corridor_lateral_range});
      }
      const bool use_course_progress =
        cfg.v2x_behavior.front_progress_detection_enabled && course_projection.valid;
      const bool front_geometry_valid =
        cfg.v2x_behavior.front_progress_detection_enabled ?
        use_course_progress : std::abs(lateral) <= corridor_lateral_range;
      const double front_longitudinal =
        use_course_progress ? course_projection.forward_distance_m : longitudinal;
      const double front_lateral = use_course_progress ? course_projection.lateral_m : lateral;
      const double front_vehicle_speed = use_course_progress ?
        course_projection.along_track_speed_mps : vehicle_speed;
      if (
        !front_hazard_hold_target_id_.empty() &&
        vehicle.id == front_hazard_hold_target_id_ &&
        longitudinal <= -cfg.v2x_behavior.front_hazard_rear_clear_distance)
      {
        held_target_rear_clear = true;
      }
      if (
        !overtake_line_state_.target_vehicle_id.empty() &&
        vehicle.id == overtake_line_state_.target_vehicle_id)
      {
        output.locked_target_seen = true;
        output.locked_target_position_jump = vehicle.position_jump;
        output.locked_target_longitudinal = front_longitudinal;
        output.locked_target_lateral = front_lateral;
        output.locked_target_speed = front_vehicle_speed;
        output.locked_target_receipt_sec = vehicle.receipt_sec;
      }
      const bool within_local_corridor = std::abs(lateral) <= corridor_lateral_range;
      const bool within_progress_corridor =
        use_course_progress && std::abs(front_lateral) <= corridor_lateral_range;
      if (!within_local_corridor && !within_progress_corridor) {
        continue;
      }

      const double clearance_longitudinal =
        use_course_progress ? front_longitudinal : longitudinal;
      if (
        track_low_speed_clearance &&
        self_distance <= cfg.v2x_behavior.low_speed_avoidance_clear_distance &&
        clearance_longitudinal > -side_longitudinal_range) {
        has_low_speed_clearance_vehicle = true;
      }
      const bool is_locked_pass_target =
        overtake_line_state_.phase == OvertakeLinePhase::Pass &&
        !overtake_line_state_.target_vehicle_id.empty() &&
        vehicle.id == overtake_line_state_.target_vehicle_id;
      // Keep the Pass clearance test in the same course frame as the explicit
      // overtake line.  A vehicle-local tangent rotates rapidly through a
      // hairpin and can make an already established side-by-side separation
      // appear to collapse, which re-enables the generic front brake.  Fall
      // back to the local relative lateral value when course projection is not
      // available.
      const double locked_pass_target_relative_lateral =
        use_course_progress && std::isfinite(model->spatial_state.e_y) ?
        front_lateral - model->spatial_state.e_y : lateral;
      const bool locked_pass_target_laterally_clear =
        v2x_overtake_core::can_exclude_locked_target_from_front_overlap(
        v2x_overtake_core::PassFrontOverlapExclusionRequest{
          overtake_line_state_.phase == OvertakeLinePhase::Pass, is_locked_pass_target,
          locked_pass_target_relative_lateral,
          cfg.v2x_behavior.overtake_pass_front_overlap_lateral_clearance,
          overtake_line_state_.pass_front_overlap_exclusion_latched});
      if (
        locked_pass_target_laterally_clear &&
        !overtake_line_state_.pass_front_overlap_exclusion_latched)
      {
        overtake_line_state_.pass_front_overlap_exclusion_latched = true;
        if (cfg.v2x_behavior.overtake_line.debug_log_enabled) {
          RCLCPP_INFO(
            rclcpp::get_logger("mpc_controller"),
            "OvertakeLine: Pass front-overlap exclusion latched, target=%s, lateral=%.2f, "
            "wp_id=%d",
            vehicle.id.c_str(), locked_pass_target_relative_lateral, model->wp_id);
        }
      }
      // Course-frame lateral values oscillate through a hairpin. Once the locked target has
      // become side-by-side in Pass, keep only that target out of the generic front-brake set
      // until the phase ends. Locked-target continuity and all other front vehicles remain active.
      const bool front_overlap =
        front_geometry_valid && std::abs(front_lateral) <= front_lateral_range &&
        !locked_pass_target_laterally_clear;
      if (
        front_overlap && front_longitudinal > 0.0 &&
        front_longitudinal < front_detection_distance)
      {
        has_front_vehicle = true;
        if (front_longitudinal < nearest_front_distance) {
          nearest_front_distance = front_longitudinal;
          nearest_front_speed = front_vehicle_speed;
          nearest_front_lateral = front_lateral;
          nearest_front_local_longitudinal = longitudinal;
          nearest_front_progress_used = use_course_progress;
          nearest_front_id = vehicle.id;
        }
        const bool moving_front =
          front_vehicle_speed > cfg.v2x_behavior.moving_front_speed_threshold;
        const double closing_speed =
          moving_front ?
          std::max(0.0, current_speed_mps_ - front_vehicle_speed) : current_speed_mps_;
        const double front_stop_distance =
          closing_speed * closing_speed / (2.0 * brake_decel) +
          (moving_front ?
          std::max(0.0, cfg.v2x_behavior.moving_safety_brake_margin) :
          std::max(0.0, cfg.v2x_behavior.safety_brake_margin));
        const double moving_headway_distance = current_speed_mps_ *
          std::max(0.0, cfg.v2x_behavior.moving_safety_brake_time_headway);
        const double front_safety_brake_distance = moving_front ?
          std::max({
            cfg.v2x_behavior.moving_safety_brake_distance, front_stop_distance,
            moving_headway_distance}) :
          std::max(cfg.v2x_behavior.safety_brake_distance, front_stop_distance);
        if (front_longitudinal < front_safety_brake_distance) {
          has_danger_vehicle = true;
        }
      }
      if (within_local_corridor && std::abs(longitudinal) < side_longitudinal_range) {
        has_side_vehicle = true;
        if (std::abs(longitudinal) < nearest_side_abs_longitudinal) {
          nearest_side_abs_longitudinal = std::abs(longitudinal);
          nearest_side_id = vehicle.id;
          nearest_side_course_longitudinal = use_course_progress ?
            front_longitudinal : longitudinal;
        }
      }
    }

    output.front_distance = nearest_front_distance;
    output.front_speed = nearest_front_speed;
    output.front_lateral = nearest_front_lateral;
    output.front_progress_used = nearest_front_progress_used;
    output.front_local_longitudinal = nearest_front_local_longitudinal;
    output.front_progress_lateral = nearest_front_lateral;
    output.has_front_vehicle = has_front_vehicle;
    output.has_side_vehicle = has_side_vehicle;
    output.has_low_speed_clearance_vehicle = has_low_speed_clearance_vehicle;
    output.has_danger_vehicle = has_danger_vehicle;
    output.target_vehicle_id = has_front_vehicle ? nearest_front_id : nearest_side_id;
    if (
      output.locked_target_seen &&
      output.locked_target_longitudinal > -side_longitudinal_range)
    {
      output.target_vehicle_id = overtake_line_state_.target_vehicle_id;
    }
    output.follow_gap_planner_allowed = !overtake_forbidden && !overtake_cooldown_active;
    const FrontRiskMetrics front_risk =
      compute_front_risk(nearest_front_distance, nearest_front_speed);
    const FrontRiskLevel front_risk_level = classify_front_risk(front_risk);
    output.front_risk = front_risk;
    output.front_risk_level = front_risk_level;
    const bool front_risk_emergency = front_risk_level == FrontRiskLevel::EmergencyBrake;
    const auto front_danger_action = v2x_overtake_core::resolve_front_danger_action(
      v2x_overtake_core::FrontDangerActionRequest{
        has_danger_vehicle,
        front_risk_emergency,
        nearest_front_speed,
        cfg.v2x_behavior.moving_front_speed_threshold,
        nearest_front_distance,
        cfg.v2x_behavior.moving_follow_hard_distance});
    output.front_danger_action = front_danger_action;
    const bool front_danger_requires_safety_brake =
      front_danger_action == v2x_overtake_core::FrontDangerAction::SafetyBrake;
    // Follow uses the full configured distance gate below. Independently retain the
    // distance-recovery part of the cap while ShiftOut/early Pass owns the lateral plan.
    // Once the locked target is laterally clear, the existing front-overlap latch removes it
    // from nearest-front selection and this cap no longer prevents pass acceleration.
    const auto moving_front_clearance_limit = v2x_overtake_core::resolve_follow_speed_limit(
      v2x_overtake_core::FollowSpeedLimitRequest{
        cfg.v2x_behavior.follow_speed_limit_enabled,
        false,
        nearest_front_distance,
        cfg.v2x_behavior.follow_speed_limit_distance,
        nearest_front_speed,
        cfg.v2x_behavior.moving_front_speed_threshold,
        cfg.v2x_behavior.moving_follow_speed_margin,
        cfg.v2x_behavior.moving_follow_target_distance,
        cfg.v2x_behavior.moving_follow_recovery_speed_margin,
        cfg.v2x_behavior.moving_follow_distance_gain,
        front_distance_velocity_limit(nearest_front_distance),
        cfg.v2x_behavior.follow_velocity,
        cfg.v_max});
    output.moving_front_clearance_limit_active =
      moving_front_clearance_limit.active &&
      moving_front_clearance_limit.moving_front &&
      moving_front_clearance_limit.moving_front_clearance_recovery;
    output.moving_front_clearance_speed_margin =
      moving_front_clearance_limit.moving_front_speed_margin_mps;
    if (output.moving_front_clearance_limit_active) {
      output.target_velocity_limit = std::min(
        output.target_velocity_limit, moving_front_clearance_limit.speed_limit_mps);
    }
    const bool initial_static_target =
      start_grid_grace_active && has_front_vehicle && has_side_vehicle &&
      !nearest_front_id.empty() && std::isfinite(nearest_front_speed) &&
      nearest_front_speed >= 0.0 &&
      nearest_front_speed <= cfg.v2x_behavior.low_speed_avoidance_max_front_speed;
    if (initial_static_target && !start_grid_initial_target_id_.has_value()) {
      start_grid_initial_target_id_ = nearest_front_id;
    }
    const bool initial_static_target_latched =
      start_grid_initial_target_id_.has_value() &&
      nearest_front_id == start_grid_initial_target_id_.value();
    const start_grid_grace::StaticStopContext start_grid_context{
      start_grid_grace_active,
      has_front_vehicle,
      has_side_vehicle,
      initial_static_target_latched,
      front_risk_emergency,
      nearest_front_speed,
      cfg.v2x_behavior.low_speed_avoidance_max_front_speed,
      cfg.v2x_behavior.moving_front_speed_threshold};
    const bool suppress_start_grid_stop_behavior =
      start_grid_grace::should_suppress_static_stop(start_grid_context);
    auto non_emergency_start_grid_context = start_grid_context;
    non_emergency_start_grid_context.emergency_brake_required = false;
    const bool emergency_overrides_start_grid =
      front_risk_emergency &&
      start_grid_grace::should_suppress_static_stop(non_emergency_start_grid_context);
    output.start_grid_stop_suppressed = suppress_start_grid_stop_behavior;
    update_start_grid_suppression_diagnostics(
      suppress_start_grid_stop_behavior, emergency_overrides_start_grid,
      output.target_vehicle_id, nearest_front_distance, nearest_front_speed,
      front_risk.required_decel);
    if (emergency_overrides_start_grid) {
      update_front_hazard_hold(true, false, nearest_front_id, false);
      output.state = V2XBehaviorState::SafetyBrake;
      output.reason = front_risk_reason(
        "start-grid front risk emergency", front_risk, front_risk_level);
      return commit_v2x_behavior_state(output, now_sec);
    }
    const bool front_hazard_hold_was_active =
      cfg.v2x_behavior.front_hazard_hold_enabled &&
      now_sec < front_hazard_hold_until_sec_;
    const bool held_target_observed_safe =
      front_hazard_hold_was_active && has_front_vehicle &&
      !front_hazard_hold_target_id_.empty() &&
      nearest_front_id == front_hazard_hold_target_id_ &&
      std::isfinite(nearest_front_speed) &&
      nearest_front_speed > cfg.v2x_behavior.moving_front_speed_threshold &&
      current_speed_mps_ <= nearest_front_speed;
    const bool refresh_held_front_hazard =
      front_hazard_hold_was_active &&
      front_danger_requires_safety_brake &&
      !suppress_start_grid_stop_behavior;
    if (update_front_hazard_hold(
        refresh_held_front_hazard, held_target_rear_clear,
        refresh_held_front_hazard ? nearest_front_id : std::string{},
        held_target_observed_safe))
    {
      output.state = V2XBehaviorState::SafetyBrake;
      output.target_vehicle_id = output.front_hazard_hold_target_id;
      output.reason = refresh_held_front_hazard ?
        "front hazard hold refreshed" : "front hazard hold after geometry loss";
      return commit_v2x_behavior_state(output, now_sec);
    }
    const bool low_speed_avoidance_candidate =
      cfg.v2x_behavior.low_speed_avoidance_enabled && has_front_vehicle &&
      !low_speed_avoidance_cooldown_active &&
      !suppress_start_grid_stop_behavior &&
      nearest_front_speed <= cfg.v2x_behavior.low_speed_avoidance_max_front_speed &&
      (!overtake_forbidden || continuing_low_speed_avoidance ||
      (cfg.v2x_behavior.low_speed_avoidance_ignore_soft_curve_forbidden &&
      !overtake_forbidden_wp)) &&
      nearest_front_distance >=
      std::max(0.0, cfg.v2x_behavior.low_speed_avoidance_min_prepare_distance) &&
      nearest_front_distance <= cfg.v2x_behavior.low_speed_avoidance_distance;
    output.low_speed_avoidance_candidate = low_speed_avoidance_candidate;
    bool low_speed_avoidance_gap_blocked = false;
    if (low_speed_avoidance_candidate) {
      const double low_speed_gap_max_self_distance =
        std::max(
          cfg.v2x_behavior.low_speed_avoidance_distance + model->length,
          cfg.v2x_behavior.low_speed_avoidance_lookahead_distance);
      const auto candidate_gap = cfg.v2x_behavior.low_speed_local_path_enabled ?
        gap_planner->plan_stopped_vehicle_local_path(
          *model, ref_wp_id, N, lb, ub, now_sec, cfg.v2x_behavior, false) :
        gap_planner->plan(
          *model, ref_wp_id, N, lb, ub, now_sec, false, true, low_speed_gap_max_self_distance);
      const bool gap_ok = cfg.v2x_behavior.low_speed_local_path_enabled ?
        candidate_gap.active && candidate_gap.feasible :
        has_consecutive_sufficient_gap(
          candidate_gap, cfg.v2x_behavior.low_speed_avoidance_min_gap_width,
          cfg.v2x_behavior.low_speed_avoidance_min_gap_points);
      if (gap_ok) {
        if (candidate_gap.pass_side_sign != 0) {
          gap_planner->lock_low_speed_pass_side(candidate_gap.pass_side_sign);
        }
        output.state = V2XBehaviorState::LowSpeedAvoidance;
        output.reason = candidate_gap.pass_side_sign < 0 ?
          "low-speed front vehicle and right gap available" :
          candidate_gap.pass_side_sign > 0 ?
          "low-speed front vehicle and left gap available" :
          "low-speed front vehicle and gap available";
        return commit_v2x_behavior_state(output, now_sec);
      }
      low_speed_avoidance_gap_blocked = !continuing_low_speed_avoidance;
      output.low_speed_avoidance_gap_blocked = low_speed_avoidance_gap_blocked;
    }

    const bool low_speed_avoidance_hold_candidate =
      cfg.v2x_behavior.low_speed_avoidance_enabled && continuing_low_speed_avoidance &&
      !low_speed_avoidance_cooldown_active &&
      has_front_vehicle && !suppress_start_grid_stop_behavior &&
      nearest_front_speed <= cfg.v2x_behavior.low_speed_avoidance_max_front_speed &&
      (!overtake_forbidden ||
      (cfg.v2x_behavior.low_speed_avoidance_ignore_soft_curve_forbidden &&
      !overtake_forbidden_wp)) &&
      nearest_front_distance <=
      std::max(
        cfg.v2x_behavior.low_speed_avoidance_clear_distance,
        cfg.v2x_behavior.low_speed_avoidance_distance);
    if (low_speed_avoidance_hold_candidate) {
      const double low_speed_gap_max_self_distance =
        std::max(
          cfg.v2x_behavior.low_speed_avoidance_distance + model->length,
          cfg.v2x_behavior.low_speed_avoidance_lookahead_distance);
      const auto hold_gap = cfg.v2x_behavior.low_speed_local_path_enabled ?
        gap_planner->plan_stopped_vehicle_local_path(
          *model, ref_wp_id, N, lb, ub, now_sec, cfg.v2x_behavior, false) :
        gap_planner->plan(
          *model, ref_wp_id, N, lb, ub, now_sec, false, true, low_speed_gap_max_self_distance);
      const bool gap_ok = cfg.v2x_behavior.low_speed_local_path_enabled ?
        hold_gap.active && hold_gap.feasible :
        has_consecutive_sufficient_gap(
          hold_gap, cfg.v2x_behavior.low_speed_avoidance_min_gap_width,
          cfg.v2x_behavior.low_speed_avoidance_min_gap_points);
      if (gap_ok) {
        if (hold_gap.pass_side_sign != 0) {
          gap_planner->lock_low_speed_pass_side(hold_gap.pass_side_sign);
        }
        output.low_speed_avoidance_candidate = true;
        output.state = V2XBehaviorState::LowSpeedAvoidance;
        output.reason = "low-speed avoidance hold";
        return commit_v2x_behavior_state(output, now_sec);
      }
    }

    if (has_front_vehicle && front_risk_level == FrontRiskLevel::EmergencyBrake) {
      update_front_hazard_hold(true, false, nearest_front_id, false);
      output.state = V2XBehaviorState::SafetyBrake;
      output.reason = front_risk_reason("front risk emergency", front_risk, front_risk_level);
      return commit_v2x_behavior_state(output, now_sec);
    }

    if (front_danger_requires_safety_brake && !suppress_start_grid_stop_behavior) {
      update_front_hazard_hold(true, false, nearest_front_id, false);
      output.state = V2XBehaviorState::SafetyBrake;
      const bool moving_front_inside_hard_distance =
        std::isfinite(nearest_front_speed) &&
        nearest_front_speed > cfg.v2x_behavior.moving_front_speed_threshold &&
        cfg.v2x_behavior.moving_follow_hard_distance > 0.0 &&
        std::isfinite(nearest_front_distance) &&
        nearest_front_distance <= cfg.v2x_behavior.moving_follow_hard_distance;
      output.reason = moving_front_inside_hard_distance ?
        "moving front inside hard center distance" :
        "stopped/slow front inside stopping distance";
      return commit_v2x_behavior_state(output, now_sec);
    }

    if (
      has_low_speed_clearance_vehicle && cfg.v2x_behavior.low_speed_avoidance_enabled &&
      continuing_low_speed_avoidance && !low_speed_avoidance_cooldown_active) {
      output.state = V2XBehaviorState::LowSpeedAvoidance;
      output.reason = "low-speed avoidance clearance hold";
      return commit_v2x_behavior_state(output, now_sec);
    }

    if (low_speed_avoidance_gap_blocked) {
      output.state = V2XBehaviorState::Follow;
      output.reason = "low-speed gap unavailable";
      bool front_risk_applied = false;
      if (apply_follow_velocity_limits(
          output, nearest_front_distance, nearest_front_speed, front_decel_curve_guard, front_risk,
          front_risk_level, front_risk_applied)) {
        output.reason += front_risk_applied ?
          " / " + front_risk_reason("front risk brake", front_risk, front_risk_level) :
          output.follow_speed_limit_active ? " / follow speed cap" : " / front decel guard";
      }
      return commit_v2x_behavior_state(output, now_sec);
    }

    const bool active_overtake_line =
      overtake_line_state_.phase == OvertakeLinePhase::ShiftOut ||
      overtake_line_state_.phase == OvertakeLinePhase::Pass;
    const bool continuing_overtake =
      active_overtake_line ||
      (v2x_behavior_state_initialized && v2x_behavior_state == V2XBehaviorState::Overtake);
    bool overtake_completion_feasible = true;
    double overtake_completion_available_distance = std::numeric_limits<double>::infinity();
    double overtake_completion_required_distance = 0.0;
    double overtake_completion_relative_speed = std::numeric_limits<double>::infinity();
    const double distance_to_hard_curve = distance_to_hard_overtake_boundary(
      ref_wp_id, cfg.v2x_behavior.overtake_completion_hard_curvature,
      cfg.v2x_behavior.overtake_completion_lookahead_distance);
    if (
      cfg.v2x_behavior.overtake_completion_guard_enabled && !continuing_overtake &&
      has_front_vehicle && std::isfinite(nearest_front_distance) &&
      std::isfinite(nearest_front_speed))
    {
      const double planned_ego_speed = std::min(
        cfg.v_max,
        std::max({
          current_speed_mps_, std::max(0.0, waypoint.v_ref),
          cfg.v2x_behavior.overtake_completion_min_relative_speed}));
      const auto completion = v2x_overtake_core::resolve_pass_completion(
        v2x_overtake_core::PassCompletionRequest{
          distance_to_hard_curve,
          cfg.v2x_behavior.overtake_completion_curve_buffer_distance,
          std::max(0.0, nearest_front_distance),
          std::max(0.0, nearest_front_speed),
          std::max(kEps, planned_ego_speed),
          cfg.v2x_behavior.overtake_line.return_clear_distance,
          cfg.v2x_behavior.overtake_line.shift_distance,
          cfg.v2x_behavior.overtake_completion_merge_buffer_distance,
          cfg.v2x_behavior.overtake_completion_min_relative_speed});
      overtake_completion_feasible = completion.feasible;
      overtake_completion_available_distance = completion.available_distance_m;
      overtake_completion_required_distance = completion.required_distance_m;
      overtake_completion_relative_speed = completion.relative_speed_mps;
    }
    output.overtake_completion_feasible = overtake_completion_feasible;
    output.overtake_completion_available_distance = overtake_completion_available_distance;
    output.overtake_completion_required_distance = overtake_completion_required_distance;
    output.overtake_completion_relative_speed = overtake_completion_relative_speed;
    const bool soft_overtake_forbidden = overtake_forbidden && !overtake_forbidden_wp;
    const bool hard_overtake_forbidden =
      overtake_forbidden_wp ||
      is_curvature_above_threshold(
        ref_wp_id, N, 0.0, cfg.v2x_behavior.overtake_completion_hard_curvature);
    // During a laterally separated Pass the locked kart is intentionally removed from the
    // generic front-overlap set. Keep using its explicit course projection for the hard-curve
    // completion check; otherwise the lateral-clear transition itself aborts the pass.
    const bool active_target_valid =
      output.locked_target_seen && !output.locked_target_position_jump &&
      std::isfinite(output.locked_target_longitudinal) &&
      std::isfinite(output.locked_target_speed);
    const double hard_curve_accel_distance = std::isfinite(distance_to_hard_curve) ?
      std::max(0.0, distance_to_hard_curve) : 0.0;
    const double reachable_hard_curve_speed = std::sqrt(std::max(
        0.0, current_speed_mps_ * current_speed_mps_ +
        2.0 * std::max(0.0, cfg.a_max) * hard_curve_accel_distance));
    const double hard_curve_reference_speed = std::min(
      cfg.v_max, std::max(current_speed_mps_, std::max(0.0, waypoint.v_ref)));
    const double active_hard_curve_planned_speed = std::max(
      kEps, std::min(hard_curve_reference_speed, reachable_hard_curve_speed));
    const auto active_hard_curve_continuation =
      v2x_overtake_core::resolve_active_hard_curve_continuation(
      v2x_overtake_core::ActiveHardCurveContinuationRequest{
        cfg.v2x_behavior.overtake_active_hard_curve_completion_enabled,
        continuing_overtake,
        overtake_line_state_.phase == OvertakeLinePhase::Pass,
        active_target_valid,
        overtake_line_state_.pass_front_overlap_exclusion_latched,
        hard_overtake_forbidden && !overtake_forbidden_wp &&
        std::isfinite(distance_to_hard_curve),
        overtake_forbidden_wp,
        overtake_cooldown_active,
        front_risk_level == FrontRiskLevel::EmergencyBrake,
        v2x_overtake_core::PassCompletionRequest{
          std::isfinite(distance_to_hard_curve) ? distance_to_hard_curve : 0.0,
          cfg.v2x_behavior.overtake_active_hard_curve_buffer_distance,
          active_target_valid ? std::max(0.0, output.locked_target_longitudinal) : 0.0,
          active_target_valid ? std::max(0.0, output.locked_target_speed) : 0.0,
          active_hard_curve_planned_speed,
          cfg.v2x_behavior.overtake_active_hard_curve_rear_clear_distance,
          0.0,
          0.0,
          cfg.v2x_behavior.overtake_completion_min_relative_speed}});
    const bool active_hard_curve_allowed = active_hard_curve_continuation.allowed;
    const bool effective_hard_overtake_forbidden =
      hard_overtake_forbidden && !active_hard_curve_allowed;
    const int inner_curve_pass_side =
      soft_overtake_forbidden ? curve_inner_pass_side(ref_wp_id, N) : 0;
    const bool continuing_inner_curve_pass =
      is_inner_curve_pass(overtake_locked_side_sign_, inner_curve_pass_side);
    const bool slow_front_overtake_candidate =
      has_front_vehicle &&
      std::isfinite(nearest_front_speed) &&
      nearest_front_speed <= cfg.v2x_behavior.overtake_before_curve_max_front_speed &&
      current_speed_mps_ - nearest_front_speed >=
      cfg.v2x_behavior.overtake_before_curve_min_speed_advantage;
    const bool before_curve_overtake_allowed =
      cfg.v2x_behavior.overtake_before_curve_enabled &&
      !overtake_cooldown_active &&
      soft_overtake_forbidden &&
      !front_decel_curve_guard &&
      slow_front_overtake_candidate &&
      front_risk_level != FrontRiskLevel::EmergencyBrake;
    const bool overtake_start_curve_blocked =
      !continuing_overtake &&
      cfg.v2x_behavior.overtake_start_curve_clearance_distance > kEps &&
      is_overtake_forbidden_curvature(
        ref_wp_id, N, cfg.v2x_behavior.overtake_start_curve_clearance_distance);
    const bool continuing_overtake_allowed =
      v2x_overtake_core::can_continue_overtake_in_soft_curve(
        v2x_overtake_core::OvertakeCurveContinuationRequest{
          cfg.v2x_behavior.overtake_continue_in_forbidden_enabled,
          cfg.v2x_behavior.overtake_continue_inner_soft_curve_enabled,
          continuing_overtake,
          soft_overtake_forbidden,
          effective_hard_overtake_forbidden,
          continuing_inner_curve_pass,
          overtake_cooldown_active,
          front_risk_level == FrontRiskLevel::EmergencyBrake});
    const double fallback_min_side_clearance = std::max(
      cfg.v2x_behavior.overtake_min_gap_width,
      cfg.v2x_behavior.overtake_fallback_min_side_clearance);
    output.overtake_start_curve_blocked = overtake_start_curve_blocked;
    output.before_curve_overtake_allowed = before_curve_overtake_allowed;
    output.continuing_overtake_allowed = continuing_overtake_allowed;
    output.overtake_hard_curve_blocked = hard_overtake_forbidden;
    output.active_hard_curve_continuation_allowed = active_hard_curve_allowed;
    output.active_hard_curve_distance = distance_to_hard_curve;
    output.active_hard_curve_available_distance =
      active_hard_curve_continuation.completion.available_distance_m;
    output.active_hard_curve_required_distance =
      active_hard_curve_continuation.completion.required_distance_m;
    output.overtake_inner_curve_pass = continuing_inner_curve_pass;

    struct SideAssessment
    {
      int side{0};
      bool gap_available{false};
      bool fallback_target{false};
      double side_clearance{0.0};
      std::string reason{"not evaluated"};
    };

    const int locked_pass_side = overtake_locked_side_sign_;
    const int geometric_preferred_pass_side =
      choose_overtake_pass_side(nearest_front_lateral, lb[0], ub[0]);
    const bool prefer_inner_curve_entry =
      cfg.v2x_behavior.overtake_inner_curve_entry_enabled &&
      locked_pass_side == 0 && soft_overtake_forbidden &&
      !hard_overtake_forbidden && inner_curve_pass_side != 0;
    const int preferred_pass_side = prefer_inner_curve_entry ?
      inner_curve_pass_side : geometric_preferred_pass_side;
    const int overtake_plan_N = v2x_overtake_gap_plan_horizon(N);
    output.overtake_plan_N = overtake_plan_N;
    const auto [overtake_lb, overtake_ub] =
      build_v2x_gap_planner_bounds(ref_wp_id, N, lb, ub, overtake_plan_N);

    const auto assess_side = [&](const int side) {
      SideAssessment assessment;
      assessment.side = side;
      assessment.side_clearance =
        overtake_side_clearance(side, nearest_front_lateral, lb[0], ub[0]);
      if (
        !has_front_vehicle &&
        !(has_side_vehicle && cfg.v2x_behavior.side_overtake_enabled))
      {
        assessment.reason = "no relevant overtake target";
        return assessment;
      }
      if (side == 0) {
        assessment.reason = "invalid pass side";
        return assessment;
      }
      if (!cfg.v2x_behavior.require_gap_for_overtake && !cfg.v2x_behavior.overtake_guard_enabled) {
        assessment.gap_available = true;
        assessment.reason = "gap check disabled";
        return assessment;
      }

      const int overtake_plan_N = v2x_overtake_gap_plan_horizon(N);
      const auto candidate_gap =
        gap_planner->plan(
        *model, ref_wp_id, overtake_plan_N, overtake_lb, overtake_ub, now_sec, false, false,
        std::numeric_limits<double>::infinity(), side,
        std::max(
          cfg.v2x_behavior.overtake_min_gap_width,
          cfg.v2x_behavior.overtake_guard_min_gap_width));
      const int planned_pass_side_sign =
        infer_gap_pass_side(candidate_gap, model->spatial_state.e_y);
      if (planned_pass_side_sign != 0 && planned_pass_side_sign != side) {
        assessment.reason = "planner returned opposite pass side";
        return assessment;
      }
      if (cfg.v2x_behavior.overtake_guard_enabled) {
        assessment.gap_available = overtake_guard_allows(
          candidate_gap, ref_wp_id, nearest_front_distance, model->spatial_state.e_y,
          continuing_overtake, assessment.reason);
        if (!assessment.gap_available && !candidate_gap.reject_reason.empty()) {
          assessment.reason = candidate_gap.reject_reason + " / " + assessment.reason;
        }
      } else {
        assessment.gap_available = has_sufficient_overtake_gap(candidate_gap);
        assessment.reason = assessment.gap_available ?
          "front vehicle and geometric gap available" : "overtake gap width";
      }
      if (
        !assessment.gap_available &&
        assessment.side_clearance >= fallback_min_side_clearance &&
        front_risk_level != FrontRiskLevel::EmergencyBrake) {
        std::string fallback_guard_reason;
        if (overtake_fallback_guard_allows(
            side, nearest_front_distance, model->spatial_state.e_y,
            lb[0], ub[0], continuing_overtake, fallback_guard_reason)) {
          assessment.gap_available = true;
          assessment.fallback_target = true;
          std::ostringstream ss;
          ss << "overtake fallback side target"
             << ", side=" << (side > 0 ? "left" : "right")
             << ", clearance=" << assessment.side_clearance
             << ", guard=" << fallback_guard_reason;
          assessment.reason = ss.str();
        } else {
          std::string close_follow_reason;
          auto side_output = output;
          side_output.overtake_pass_side_sign = side;
          side_output.overtake_side_clearance = assessment.side_clearance;
          if (overtake_close_follow_allows(
              side_output, nearest_front_distance, nearest_front_speed, model->spatial_state.e_y,
              lb[0], ub[0], close_follow_reason)) {
            assessment.gap_available = true;
            assessment.fallback_target = true;
            assessment.reason = close_follow_reason;
          } else {
            assessment.reason = fallback_guard_reason + " / " + close_follow_reason;
          }
        }
      }
      return assessment;
    };

    SideAssessment left_assessment;
    SideAssessment right_assessment;
    if (locked_pass_side != 0) {
      const auto locked_assessment = assess_side(locked_pass_side);
      if (locked_pass_side > 0) {
        left_assessment = locked_assessment;
      } else {
        right_assessment = locked_assessment;
      }
    } else {
      const int first_side = preferred_pass_side != 0 ? preferred_pass_side : 1;
      const auto first_assessment = assess_side(first_side);
      if (first_side > 0) {
        left_assessment = first_assessment;
      } else {
        right_assessment = first_assessment;
      }
      if (cfg.v2x_behavior.overtake_try_both_sides) {
        const auto alternate_assessment = assess_side(-first_side);
        if (first_side > 0) {
          right_assessment = alternate_assessment;
        } else {
          left_assessment = alternate_assessment;
        }
      }
    }

    output.overtake_left_gap_available = left_assessment.gap_available;
    output.overtake_right_gap_available = right_assessment.gap_available;
    output.overtake_left_reason = left_assessment.reason;
    output.overtake_right_reason = right_assessment.reason;
    const auto pass_side = [](const int side) {
        return side > 0 ? overtake_core::PassSide::Left :
               side < 0 ? overtake_core::PassSide::Right : overtake_core::PassSide::None;
      };
    const auto resolve_outer_curve_for_side = [&](const SideAssessment & assessment) {
        return v2x_overtake_core::resolve_outer_curve_overtake(
          v2x_overtake_core::OuterCurveOvertakeRequest{
            cfg.v2x_behavior.overtake_outer_curve_entry_enabled,
            cfg.v2x_behavior.overtake_outer_curve_hard_continuation_enabled,
            continuing_overtake,
            soft_overtake_forbidden,
            hard_overtake_forbidden,
            overtake_forbidden_wp,
            overtake_cooldown_active,
            front_risk_level == FrontRiskLevel::EmergencyBrake,
            assessment.gap_available,
            active_target_valid,
            assessment.side,
            inner_curve_pass_side});
      };
    const auto resolve_inner_curve_for_side = [&](const SideAssessment & assessment) {
        return v2x_overtake_core::resolve_inner_curve_overtake(
          v2x_overtake_core::InnerCurveOvertakeRequest{
            cfg.v2x_behavior.overtake_inner_curve_entry_enabled,
            cfg.v2x_behavior.overtake_inner_curve_hard_continuation_enabled,
            continuing_overtake,
            soft_overtake_forbidden,
            hard_overtake_forbidden,
            overtake_forbidden_wp,
            overtake_cooldown_active,
            front_risk_level == FrontRiskLevel::EmergencyBrake,
            assessment.gap_available,
            active_target_valid,
            assessment.side,
            inner_curve_pass_side});
      };
    const auto execution_allowed_for_side = [&](const SideAssessment & assessment) {
        if (!assessment.gap_available || assessment.side == 0) {
          return false;
        }
        const auto outer_curve = resolve_outer_curve_for_side(assessment);
        const auto inner_curve = resolve_inner_curve_for_side(assessment);
        const bool inner_curve_pass =
          is_inner_curve_pass(assessment.side, inner_curve_pass_side);
        const bool active_locked_inner_curve_allowed =
          (continuing_overtake_allowed || active_hard_curve_allowed) &&
          continuing_overtake &&
          locked_pass_side != 0 && assessment.side == locked_pass_side;
        const bool side_fallback_soft_curve_allowed =
          cfg.v2x_behavior.overtake_fallback_ignore_soft_curve_forbidden &&
          soft_overtake_forbidden &&
          assessment.side_clearance >= fallback_min_side_clearance &&
          !inner_curve_pass &&
          front_risk_level != FrontRiskLevel::EmergencyBrake;
        return !overtake_cooldown_active &&
               (!overtake_start_curve_blocked || outer_curve.entry_allowed ||
               outer_curve.hard_continuation_allowed || inner_curve.entry_allowed ||
               inner_curve.hard_continuation_allowed) &&
               (overtake_completion_feasible || outer_curve.entry_allowed ||
               outer_curve.hard_continuation_allowed || inner_curve.entry_allowed ||
               inner_curve.hard_continuation_allowed) &&
               !overtake_forbidden_wp &&
               (!effective_hard_overtake_forbidden ||
               outer_curve.hard_continuation_allowed ||
               inner_curve.hard_continuation_allowed) &&
               (!inner_curve_pass || active_locked_inner_curve_allowed ||
               inner_curve.entry_allowed || inner_curve.hard_continuation_allowed) &&
               (!soft_overtake_forbidden || before_curve_overtake_allowed ||
               continuing_overtake_allowed || active_hard_curve_allowed ||
               outer_curve.entry_allowed || outer_curve.hard_continuation_allowed ||
               inner_curve.entry_allowed || inner_curve.hard_continuation_allowed ||
               side_fallback_soft_curve_allowed);
      };
    const auto select_side = [&](const bool require_execution_permission) {
        return overtake_core::select_pass_side(
          overtake_core::SideSelectionRequest{
            preferred_pass_side != 0 ? pass_side(preferred_pass_side) :
            overtake_core::PassSide::Left,
            pass_side(locked_pass_side),
            left_assessment.gap_available &&
            (!require_execution_permission || execution_allowed_for_side(left_assessment)),
            right_assessment.gap_available &&
            (!require_execution_permission || execution_allowed_for_side(right_assessment)),
            cfg.v2x_behavior.overtake_try_both_sides});
      };
    auto side_selection = select_side(true);
    if (side_selection.side == overtake_core::PassSide::None) {
      // Preserve the geometric candidate and its reason for diagnostics even when curve policy
      // prevents executing either side.
      side_selection = select_side(false);
    }
    output.overtake_pass_side_sign = static_cast<int>(side_selection.side);
    const SideAssessment & selected_assessment = output.overtake_pass_side_sign > 0 ?
      left_assessment : output.overtake_pass_side_sign < 0 ?
      right_assessment :
      (locked_pass_side != 0 ? locked_pass_side : preferred_pass_side) < 0 ?
      right_assessment : left_assessment;
    output.overtake_side_clearance = selected_assessment.side_clearance;
    output.overtake_fallback_target = selected_assessment.fallback_target;
    const auto selected_outer_curve = resolve_outer_curve_for_side(selected_assessment);
    output.outer_curve_entry_allowed = selected_outer_curve.entry_allowed;
    output.outer_curve_hard_continuation_allowed =
      selected_outer_curve.hard_continuation_allowed;
    const auto selected_inner_curve = resolve_inner_curve_for_side(selected_assessment);
    output.inner_curve_entry_allowed = selected_inner_curve.entry_allowed;
    output.inner_curve_hard_continuation_allowed =
      selected_inner_curve.hard_continuation_allowed;
    const bool overtake_gap_available = output.overtake_pass_side_sign != 0 &&
      selected_assessment.gap_available;
    const bool selected_inner_curve_pass =
      is_inner_curve_pass(output.overtake_pass_side_sign, inner_curve_pass_side);
    output.overtake_inner_curve_pass = selected_inner_curve_pass;
    const bool overtake_zone_allows =
      overtake_gap_available && execution_allowed_for_side(selected_assessment);
    output.overtake_zone_allows = overtake_zone_allows;

    std::string overtake_block_reason = !overtake_gap_available ?
      selected_assessment.reason :
      selected_outer_curve.entry_allowed ?
      "outer curve entry and reachable gap" :
      selected_outer_curve.hard_continuation_allowed ?
      "continue locked outer line through hard curve" :
      selected_inner_curve.entry_allowed ?
      "inner curve entry and reachable gap" :
      selected_inner_curve.hard_continuation_allowed ?
      "continue locked inner line through hard curve" :
      overtake_forbidden_wp ? "overtake forbidden wp" :
      overtake_cooldown_active ? "overtake curve cooldown" :
      overtake_start_curve_blocked ? "overtake start too close to curve" :
      !overtake_completion_feasible ? "overtake completion distance" :
      effective_hard_overtake_forbidden ? "overtake hard curve blocked" :
      selected_inner_curve_pass &&
      !(continuing_overtake_allowed || active_hard_curve_allowed) ?
      "overtake inner curve blocked" :
      soft_overtake_forbidden && !overtake_zone_allows ? "overtake forbidden curve" :
      selected_assessment.reason;
    output.overtake_gap_available = overtake_gap_available;
    output.overtake_block_reason = overtake_block_reason;

    if (has_front_vehicle) {
      if (overtake_zone_allows && overtake_gap_available) {
        output.state = V2XBehaviorState::Overtake;
        output.reason = selected_outer_curve.entry_allowed ?
          "outer curve entry / " + selected_assessment.reason :
          selected_outer_curve.hard_continuation_allowed ?
          "continue locked outer line through hard curve / " + selected_assessment.reason :
          selected_inner_curve.entry_allowed ?
          "inner curve entry / " + selected_assessment.reason :
          selected_inner_curve.hard_continuation_allowed ?
          "continue locked inner line through hard curve / " + selected_assessment.reason :
          before_curve_overtake_allowed ?
          "slow front before curve and reachable gap / " + overtake_block_reason :
          active_hard_curve_allowed ?
          "continue active pass before hard curve / " + overtake_block_reason :
          continuing_overtake_allowed ?
          (selected_inner_curve_pass ?
          "continue locked pass in soft inner curve / " + overtake_block_reason :
          "continue overtake in soft forbidden zone / " + overtake_block_reason) :
          cfg.v2x_behavior.overtake_guard_enabled ?
          overtake_block_reason :
          "front vehicle and gap available";
        if (cfg.v2x_behavior.overtake_front_velocity_limit_enabled) {
          bool front_risk_applied = false;
          if (apply_follow_velocity_limits(
              output, nearest_front_distance, nearest_front_speed, front_decel_curve_guard, front_risk,
              front_risk_level, front_risk_applied)) {
            output.reason += front_risk_applied ?
              " / " + front_risk_reason("front risk brake", front_risk, front_risk_level) :
              output.follow_speed_limit_active ? " / follow speed cap" : " / front decel guard";
          }
        }
      } else {
        output.state = V2XBehaviorState::Follow;
        // `before_curve_overtake_allowed` is a relaxation that has already passed.  If
        // execution is still rejected, report the guard that actually rejected it.
        output.reason = overtake_block_reason;
        bool front_risk_applied = false;
        if (apply_follow_velocity_limits(
            output, nearest_front_distance, nearest_front_speed, front_decel_curve_guard, front_risk,
            front_risk_level, front_risk_applied)) {
          output.reason += front_risk_applied ?
            " / " + front_risk_reason("front risk brake", front_risk, front_risk_level) :
            output.follow_speed_limit_active ? " / follow speed cap" : " / front decel guard";
        }
      }
      return commit_v2x_behavior_state(output, now_sec);
    }

    if (has_side_vehicle && cfg.v2x_behavior.side_overtake_enabled) {
      const bool side_overtake_entry_target_valid =
        v2x_overtake_core::can_start_side_overtake(
        v2x_overtake_core::SideOvertakeEntryRequest{
          continuing_overtake, nearest_side_course_longitudinal,
          cfg.v2x_behavior.side_overtake_entry_rear_tolerance});
      const bool side_inner_curve_pass =
        is_inner_curve_pass(output.overtake_pass_side_sign, inner_curve_pass_side);
      const bool active_locked_side_inner_curve_allowed =
        (continuing_overtake_allowed || active_hard_curve_allowed) &&
        continuing_overtake &&
        overtake_locked_side_sign_ != 0 &&
        output.overtake_pass_side_sign == overtake_locked_side_sign_;
      const bool side_outer_curve_allowed =
        output.outer_curve_entry_allowed ||
        output.outer_curve_hard_continuation_allowed;
      const bool side_inner_curve_allowed =
        output.inner_curve_entry_allowed ||
        output.inner_curve_hard_continuation_allowed;
      const bool side_overtake_zone_allows =
        side_overtake_entry_target_valid &&
        !overtake_cooldown_active &&
        (!overtake_start_curve_blocked || side_outer_curve_allowed ||
        side_inner_curve_allowed) &&
        !overtake_forbidden_wp &&
        (!effective_hard_overtake_forbidden ||
        output.outer_curve_hard_continuation_allowed ||
        output.inner_curve_hard_continuation_allowed) &&
        (!side_inner_curve_pass || active_locked_side_inner_curve_allowed ||
        side_inner_curve_allowed) &&
        (!overtake_forbidden ||
        (cfg.v2x_behavior.side_overtake_ignore_soft_curve_forbidden && soft_overtake_forbidden) ||
        continuing_overtake_allowed || active_hard_curve_allowed ||
        side_outer_curve_allowed || side_inner_curve_allowed);
      output.overtake_zone_allows = side_overtake_zone_allows;

      const bool side_overtake_gap_available = output.overtake_gap_available;
      std::string side_overtake_block_reason = !side_overtake_entry_target_valid ?
        "side target already behind" :
        output.inner_curve_entry_allowed ?
        "inner curve entry and reachable gap" :
        output.inner_curve_hard_continuation_allowed ?
        "continue locked inner line through hard curve" :
        output.outer_curve_entry_allowed ?
        "outer curve entry and reachable gap" :
        output.outer_curve_hard_continuation_allowed ?
        "continue locked outer line through hard curve" :
        overtake_forbidden_wp ?
        "overtake forbidden wp" :
        overtake_cooldown_active ?
        "overtake curve cooldown" :
        overtake_start_curve_blocked ?
        "overtake start too close to curve" :
        !overtake_completion_feasible ?
        "overtake completion distance" :
        effective_hard_overtake_forbidden ?
        "overtake hard curve blocked" :
        side_inner_curve_pass && !active_locked_side_inner_curve_allowed ?
        "overtake inner curve blocked" :
        side_overtake_zone_allows ?
        output.overtake_block_reason :
        "overtake forbidden curve";

      output.overtake_gap_available = side_overtake_gap_available;
      output.overtake_block_reason = side_overtake_block_reason;
      if (side_overtake_zone_allows && side_overtake_gap_available) {
        output.state = V2XBehaviorState::Overtake;
        output.reason = output.inner_curve_hard_continuation_allowed ?
          "continue side-by-side inner line through hard curve / " + side_overtake_block_reason :
          output.inner_curve_entry_allowed ?
          "side vehicle inner curve entry / " + side_overtake_block_reason :
          output.outer_curve_hard_continuation_allowed ?
          "continue side-by-side outer line through hard curve / " + side_overtake_block_reason :
          output.outer_curve_entry_allowed ?
          "side vehicle outer curve entry / " + side_overtake_block_reason :
          "side vehicle and reachable gap / " + side_overtake_block_reason;
        return commit_v2x_behavior_state(output, now_sec);
      }
      output.reason = side_overtake_zone_allows ?
        "side vehicle overtake blocked / " + side_overtake_block_reason :
        side_overtake_block_reason;
    }

    if (
      has_side_vehicle && cfg.v2x_behavior.low_speed_avoidance_enabled &&
      v2x_behavior_state_initialized && v2x_behavior_state == V2XBehaviorState::LowSpeedAvoidance &&
      !low_speed_avoidance_cooldown_active) {
      output.state = V2XBehaviorState::LowSpeedAvoidance;
      output.reason = "low-speed avoidance side vehicle";
      return commit_v2x_behavior_state(output, now_sec);
    }

    if (has_side_vehicle) {
      output.state = V2XBehaviorState::Follow;
      if (output.reason.empty()) {
        output.reason = "side vehicle";
      }
      return commit_v2x_behavior_state(output, now_sec);
    }

    output.reason = "no relevant vehicle";
    return commit_v2x_behavior_state(output, now_sec);
  }

  MpcProblem init_problem(
    const int N, const double safety_margin, const double now_sec, const int base_wp_id,
    const int planning_wp_id)
  {
    constexpr int nx = 3;
    constexpr int nu = 2;
    // Once the direct shift controller owns the stopped-vehicle bypass, keep
    // it latched until the entire vehicle pack has cleared. Releasing as soon
    // as e_y/e_psi settle can hand control back to an infeasible MPC problem
    // while the ego vehicle is still alongside the first stopped vehicle.
    low_speed_shift_control_active_ = low_speed_shift_control_was_active_;
    const int nx_N = nx * (N + 1);
    const int nu_N = nu * N;
    const double inf = std::numeric_limits<double>::infinity();

    Eigen::Vector2d umin;
    umin << 0.0, -std::tan(cfg.delta_max) / model->length;
    Eigen::Vector2d umax;
    umax << cfg.v_max, std::tan(cfg.delta_max) / model->length;
    Eigen::Vector3d xmin;
    xmin << -inf, -inf, -inf;
    Eigen::Vector3d xmax;
    xmax << inf, inf, inf;

    Eigen::VectorXd ur = Eigen::VectorXd::Zero(nu_N);
    Eigen::VectorXd xr = Eigen::VectorXd::Zero(nx_N);
    Eigen::VectorXd uq = Eigen::VectorXd::Zero(N * nx);
    Eigen::VectorXd xmin_dyn(nx_N);
    Eigen::VectorXd xmax_dyn(nx_N);
    for (int i = 0; i < N + 1; ++i) {
      xmin_dyn.segment<nx>(i * nx) = xmin;
      xmax_dyn.segment<nx>(i * nx) = xmax;
    }
    Eigen::VectorXd umax_dyn(nu_N);
    Eigen::VectorXd umin_dyn(nu_N);
    for (int i = 0; i < N; ++i) {
      umin_dyn.segment<nu>(i * nu) = umin;
      umax_dyn.segment<nu>(i * nu) = umax;
    }

    Eigen::VectorXd kappa_pred(N);
    for (int i = 0; i < N - 1; ++i) {
      kappa_pred[i] = std::tan(current_control[3 + i * nu]) / model->length;
    }
    kappa_pred[N - 1] = std::tan(current_control[nu * N - 1]) / model->length;

    Eigen::MatrixXd A_dense = Eigen::MatrixXd::Zero(nx_N, nx_N);
    Eigen::MatrixXd B_dense = Eigen::MatrixXd::Zero(nx_N, nu_N);
    for (int n = 0; n < N; ++n) {
      const auto & current_waypoint = model->reference_path->get_waypoint(planning_wp_id + n);
      const auto & next_waypoint = model->reference_path->get_waypoint(planning_wp_id + n + 1);
      const double delta_s = next_waypoint.distance_to(current_waypoint);
      const double kappa_ref = current_waypoint.kappa;
      const double v_ref = clip(current_waypoint.v_ref, umin[0], umax[0]);
      const auto [f, A_lin, B_lin] = model->linearize(v_ref, kappa_ref, delta_s);
      A_dense.block<nx, nx>((n + 1) * nx, n * nx) = A_lin;
      B_dense.block<nx, nu>((n + 1) * nx, n * nu) = B_lin;
      ur.segment<nu>(n * nu) = Eigen::Vector2d(v_ref, kappa_ref);
      uq.segment<nx>(n * nx) = B_lin * Eigen::Vector2d(v_ref, kappa_ref) - f;

      double max_kappa_pred = std::abs(kappa_pred[n]);
      if (cfg.use_max_kappa_pred) {
        for (int i = n; i < N; ++i) {
          max_kappa_pred = std::max(max_kappa_pred, std::abs(kappa_pred[i]));
        }
      }
      const double vmax_dyn = std::sqrt(cfg.ay_max / (max_kappa_pred + 1e-12));
      umax_dyn[nu * n] = std::min(vmax_dyn, umax_dyn[nu * n]);
      ur[nu * n] = std::min(ur[nu * n], umax_dyn[nu * n]);
    }

    int ref_wp_id = 0;
    if (
      model->reference_path->path_constraints_upper.empty() ||
      model->reference_path->path_constraints_lower.empty()) {
      model->reference_path->update_simple_path_constraints(cfg.N, model->safety_margin);
    }
    if (!model->reference_path->path_constraints_upper.empty()) {
      const int constraint_count =
        static_cast<int>(model->reference_path->path_constraints_upper.size());
      const int candidate_ref_wp_id = planning_wp_id + 1;
      ref_wp_id = model->reference_path->circular ?
        ((candidate_ref_wp_id % constraint_count) + constraint_count) % constraint_count :
        std::clamp(candidate_ref_wp_id, 0, constraint_count - 1);
    }
    Eigen::VectorXd ub(N);
    Eigen::VectorXd lb(N);
    for (int i = 0; i < N; ++i) {
      ub[i] = model->reference_path->path_constraints_upper.at(ref_wp_id).at(i);
      lb[i] = model->reference_path->path_constraints_lower.at(ref_wp_id).at(i);
    }
    model->reference_path->border_cells.current_wp_id = ref_wp_id;
    if (model->safety_margin != safety_margin) {
      const double safety_margin_diff = safety_margin - model->safety_margin;
      for (int i = 0; i < N; ++i) {
        ub[i] -= safety_margin_diff;
        lb[i] += safety_margin_diff;
        if (ub[i] < lb[i]) {
          ub[i] = 0.0;
          lb[i] = 0.0;
        }
      }
    }

    const auto behavior_output = evaluate_v2x_behavior(ref_wp_id, N, lb, ub, now_sec);
    const bool low_speed_shift_pose_settled =
      v2x_overtake_core::is_low_speed_shift_complete(
      model->spatial_state.e_y, model->spatial_state.e_psi,
      low_speed_shift_target_ey_,
      cfg.v2x_behavior.low_speed_avoidance_shift_lateral_tolerance,
      cfg.v2x_behavior.low_speed_avoidance_shift_heading_tolerance);
    if (low_speed_shift_control_was_active_) {
      if (
        behavior_output.has_front_vehicle || behavior_output.has_side_vehicle ||
        behavior_output.has_low_speed_clearance_vehicle)
      {
        low_speed_shift_last_relevant_vehicle_sec_ = now_sec;
      }
      const double clear_duration_sec =
        std::isfinite(low_speed_shift_last_relevant_vehicle_sec_) ?
        std::max(0.0, now_sec - low_speed_shift_last_relevant_vehicle_sec_) : 0.0;
      if (
        v2x_overtake_core::should_release_low_speed_shift_control(
          low_speed_shift_pose_settled, behavior_output.has_front_vehicle,
          behavior_output.has_side_vehicle,
          behavior_output.has_low_speed_clearance_vehicle, clear_duration_sec,
          cfg.v2x_behavior.low_speed_avoidance_shift_clear_hold_sec))
      {
        low_speed_shift_control_active_ = false;
      }
    }
    last_v2x_behavior_output_ = behavior_output;
    const bool solver_overtake_cooldown_active =
      v2x_overtake_core::is_solver_cooldown_active(
        now_sec, overtake_solver_cooldown_until_sec_);
    const bool suppress_overtake_after_solver_failures =
      (overtake_solver_recovery_active_ || solver_overtake_cooldown_active ||
       overtake_solver_reentry_blocked_) &&
      behavior_output.state == V2XBehaviorState::Overtake;
    const bool explicit_overtake_line_owns_plan =
      v2x_overtake_core::explicit_overtake_line_owns_lateral_plan(
      v2x_overtake_core::OvertakeLateralPlannerOwnershipRequest{
        cfg.v2x_behavior.overtake_line.enabled,
        behavior_output.state == V2XBehaviorState::Overtake,
        overtake_line_state_.phase == OvertakeLinePhase::ShiftOut ||
        overtake_line_state_.phase == OvertakeLinePhase::Pass});
    const bool use_gap_planner =
      gap_planner != nullptr &&
      (!cfg.v2x_behavior.enabled || behavior_output.allow_gap_planner) &&
      !suppress_overtake_after_solver_failures &&
      !explicit_overtake_line_owns_plan;
    const bool use_low_speed_local_path =
      use_gap_planner && cfg.v2x_behavior.low_speed_local_path_enabled &&
      behavior_output.state == V2XBehaviorState::LowSpeedAvoidance;
    const bool use_overtake_lookahead =
      use_gap_planner && behavior_output.state == V2XBehaviorState::Overtake &&
      !use_low_speed_local_path;
    int overtake_pass_side_sign = behavior_output.overtake_pass_side_sign;
    if (behavior_output.state == V2XBehaviorState::Overtake) {
      if (overtake_locked_side_sign_ != 0) {
        overtake_pass_side_sign = overtake_locked_side_sign_;
      } else if (overtake_pass_side_sign != 0) {
        overtake_locked_side_sign_ = overtake_pass_side_sign;
      }
    }
    const bool use_overtake_side_target =
      behavior_output.state == V2XBehaviorState::Overtake && overtake_pass_side_sign != 0 &&
      !suppress_overtake_after_solver_failures && !explicit_overtake_line_owns_plan;
    const int follow_preposition_inner_side =
      behavior_output.overtake_forbidden && !behavior_output.overtake_forbidden_wp ?
      curve_inner_pass_side(ref_wp_id, N) : 0;
    int follow_preposition_pass_side_sign = behavior_output.overtake_pass_side_sign;
    if (
      follow_preposition_inner_side != 0 &&
      (follow_preposition_pass_side_sign == 0 ||
      is_inner_curve_pass(follow_preposition_pass_side_sign, follow_preposition_inner_side))) {
      follow_preposition_pass_side_sign = -follow_preposition_inner_side;
    }
    const double follow_preposition_side_clearance =
      overtake_side_clearance(
        follow_preposition_pass_side_sign, behavior_output.front_lateral, lb[0], ub[0]);
    const bool follow_preposition_curve_allowed =
      !behavior_output.overtake_forbidden ||
      (!behavior_output.overtake_forbidden_wp &&
      !is_inner_curve_pass(follow_preposition_pass_side_sign, follow_preposition_inner_side));
    const bool follow_preposition_front_distance_ok =
      std::isfinite(behavior_output.front_distance) &&
      behavior_output.front_distance >=
      std::max(0.0, cfg.v2x_behavior.overtake_guard_min_front_distance);
    const bool use_follow_preposition_target =
      cfg.v2x_behavior.follow_preposition_enabled &&
      behavior_output.state == V2XBehaviorState::Follow &&
      behavior_output.has_front_vehicle &&
      follow_preposition_curve_allowed &&
      follow_preposition_front_distance_ok &&
      follow_preposition_pass_side_sign != 0 &&
      follow_preposition_side_clearance >=
      cfg.v2x_behavior.follow_preposition_min_side_clearance &&
      cfg.v2x_behavior.follow_preposition_offset > kEps;
    const int gap_plan_N = use_overtake_lookahead ? v2x_overtake_gap_plan_horizon(N) : N;
    const auto [gap_plan_lb, gap_plan_ub] = use_overtake_lookahead ?
      build_v2x_gap_planner_bounds(ref_wp_id, N, lb, ub, gap_plan_N) :
      std::pair<Eigen::VectorXd, Eigen::VectorXd>{lb, ub};
    const auto planner_output = use_gap_planner ?
      (use_low_speed_local_path ?
      gap_planner->plan_stopped_vehicle_local_path(
        *model, ref_wp_id, N, lb, ub, now_sec, cfg.v2x_behavior, true) :
      gap_planner->plan(
        *model, ref_wp_id, gap_plan_N, gap_plan_lb, gap_plan_ub, now_sec, true,
        behavior_output.state == V2XBehaviorState::LowSpeedAvoidance,
        behavior_output.state == V2XBehaviorState::LowSpeedAvoidance ?
        std::max(
          cfg.v2x_behavior.low_speed_avoidance_distance + model->length,
          cfg.v2x_behavior.low_speed_avoidance_lookahead_distance) :
        std::numeric_limits<double>::infinity(),
        behavior_output.state == V2XBehaviorState::Overtake ? overtake_pass_side_sign : 0,
        behavior_output.state == V2XBehaviorState::Overtake ?
        std::optional<double>{std::max(
          cfg.v2x_behavior.overtake_min_gap_width,
          cfg.v2x_behavior.overtake_guard_min_gap_width)} : std::nullopt)) :
      GapPlannerOutput{};
    if (
      behavior_output.state == V2XBehaviorState::Overtake &&
      std::isfinite(behavior_output.desired_velocity))
    {
      for (int i = 0; i < N; ++i) {
        ur[2 * i] = std::min({
          ur[2 * i], umax_dyn[2 * i], behavior_output.desired_velocity});
      }
    }
    if (std::isfinite(behavior_output.target_velocity_limit)) {
      apply_velocity_limit(umax_dyn, ur, N, behavior_output.target_velocity_limit);
    }
    const bool allow_no_gap_velocity_limit =
      behavior_output.state != V2XBehaviorState::Follow ||
      cfg.v2x_behavior.follow_gap_planner_no_gap_speed_limit_enabled;
    const bool apply_no_gap_velocity_limit =
      allow_no_gap_velocity_limit && !behavior_output.overtake_fallback_target;
    if (planner_output.active) {
      if (planner_output.feasible) {
        for (int i = 0; i < N; ++i) {
          lb[i] = std::max(lb[i], planner_output.lb[i]);
          ub[i] = std::min(ub[i], planner_output.ub[i]);
          if (ub[i] < lb[i]) {
            ub[i] = 0.0;
            lb[i] = 0.0;
          }
        }
      } else if (!planner_output.feasible && apply_no_gap_velocity_limit) {
        const double no_gap_velocity = std::max(0.0, planner_output.target_velocity_limit);
        apply_velocity_limit(umax_dyn, ur, N, no_gap_velocity);
      }
    }
    if (
      std::isfinite(planner_output.target_velocity_limit) &&
      (planner_output.feasible || apply_no_gap_velocity_limit)) {
      apply_velocity_limit(umax_dyn, ur, N, planner_output.target_velocity_limit);
    }
    if (
      use_low_speed_local_path && planner_output.active && planner_output.feasible &&
      !planner_output.pass_corridor_enforced &&
      behavior_output.state == V2XBehaviorState::LowSpeedAvoidance)
    {
      low_speed_shift_control_active_ = true;
      low_speed_shift_target_ey_ = planner_output.pass_target_ey;
      low_speed_shift_velocity_mps_ = std::min(
        cfg.v_max, std::max(0.0, cfg.v2x_behavior.low_speed_avoidance_shift_velocity));
      if (!low_speed_shift_control_was_active_) {
        low_speed_shift_last_relevant_vehicle_sec_ = now_sec;
      }
    }

    const auto overtake_line_output =
      update_overtake_line(behavior_output, ref_wp_id, N, lb, ub, now_sec);
    if (std::isfinite(overtake_line_output.target_velocity_reference)) {
      for (int i = 0; i < N; ++i) {
        ur[2 * i] = std::min({
          ur[2 * i], umax_dyn[2 * i], overtake_line_output.target_velocity_reference});
      }
    }
    if (std::isfinite(overtake_line_output.target_velocity_limit)) {
      apply_velocity_limit(umax_dyn, ur, N, overtake_line_output.target_velocity_limit);
    }
    const bool use_overtake_line_target = overtake_line_output.active;

    if (use_overtake_line_target) {
      overtake_locked_target_ey_.reset();
    } else if (!use_overtake_side_target) {
      overtake_locked_target_ey_.reset();
      overtake_locked_side_sign_ = 0;
      overtake_entry_speed_.reset();
    }
    double overtake_target_ey = 0.0;
    if (!use_overtake_line_target && use_overtake_side_target) {
      const int pass_side_sign = overtake_pass_side_sign;
      const double raw_side_target = overtake_side_target(lb[0], ub[0], pass_side_sign);
      const double current_ey = model->spatial_state.e_y;
      const double directional_target = pass_side_sign > 0 ?
        std::max(raw_side_target, current_ey) :
        std::min(raw_side_target, current_ey);
      if (!overtake_locked_target_ey_.has_value()) {
        overtake_locked_target_ey_ = directional_target;
      } else if (pass_side_sign > 0) {
        overtake_locked_target_ey_ = std::max(overtake_locked_target_ey_.value(), directional_target);
      } else {
        overtake_locked_target_ey_ = std::min(overtake_locked_target_ey_.value(), directional_target);
      }
      overtake_target_ey = overtake_locked_target_ey_.value();
    }

    xmin_dyn[0] = model->spatial_state.e_y;
    xmax_dyn[0] = model->spatial_state.e_y;
    double previous_fallback_ref_ey = model->spatial_state.e_y;
    for (int i = 0; i < N; ++i) {
      xmin_dyn[nx + i * nx] = lb[i];
      xmax_dyn[nx + i * nx] = ub[i];
      const double center_ey = (lb[i] + ub[i]) / 2.0;
      xr[nx + i * nx] = clip(cfg.center_bias * center_ey, lb[i], ub[i]);
      if (
        use_overtake_line_target &&
        i < static_cast<int>(overtake_line_output.target_active.size()) &&
        overtake_line_output.target_active[i]) {
        xr[nx + i * nx] =
          (1.0 - cfg.v2x_behavior.overtake_line.target_bias) * xr[nx + i * nx] +
          cfg.v2x_behavior.overtake_line.target_bias * overtake_line_output.target_ey[i];
      } else if (use_overtake_side_target) {
        const double ramp_ratio = std::max(0.1, cfg.v2x_gap.overtake_target_ramp_ratio);
        const double linear_progress = clip(
          (static_cast<double>(i) + 1.0) / (static_cast<double>(N) * ramp_ratio), 0.0, 1.0);
        const double progress =
          linear_progress * linear_progress * (3.0 - 2.0 * linear_progress);
        const double ramp_target =
          model->spatial_state.e_y +
          progress * (overtake_target_ey - model->spatial_state.e_y);
        const double clipped_target = clip(ramp_target, lb[i], ub[i]);
        const double monotonic_target = overtake_pass_side_sign > 0 ?
          std::max(clipped_target, previous_fallback_ref_ey) :
          std::min(clipped_target, previous_fallback_ref_ey);
        const double fallback_target = clip(monotonic_target, lb[i], ub[i]);
        previous_fallback_ref_ey = fallback_target;
        xr[nx + i * nx] =
          (1.0 - cfg.v2x_gap.target_bias) * xr[nx + i * nx] +
          cfg.v2x_gap.target_bias * fallback_target;
      } else if (use_follow_preposition_target) {
        const double ramp_ratio = cfg.v2x_behavior.follow_preposition_ramp_ratio;
        const double linear_progress = clip(
          (static_cast<double>(i) + 1.0) / (static_cast<double>(N) * ramp_ratio), 0.0, 1.0);
        const double progress =
          linear_progress * linear_progress * (3.0 - 2.0 * linear_progress);
        const double preposition_target =
          static_cast<double>(follow_preposition_pass_side_sign) *
          cfg.v2x_behavior.follow_preposition_offset;
        const double ramp_target =
          model->spatial_state.e_y +
          progress * (preposition_target - model->spatial_state.e_y);
        const double clipped_target = clip(ramp_target, lb[i], ub[i]);
        xr[nx + i * nx] =
          (1.0 - cfg.v2x_behavior.follow_preposition_target_bias) * xr[nx + i * nx] +
          cfg.v2x_behavior.follow_preposition_target_bias * clipped_target;
      } else if (
        planner_output.active && planner_output.feasible &&
        i < static_cast<int>(planner_output.target_active.size()) && planner_output.target_active[i]) {
        const double planner_target_ey =
          use_low_speed_local_path && cfg.v2x_behavior.low_speed_local_path_invert_target ?
          -planner_output.target_ey[i] :
          planner_output.target_ey[i];
        xr[nx + i * nx] =
          (1.0 - cfg.v2x_gap.target_bias) * xr[nx + i * nx] +
          cfg.v2x_gap.target_bias * planner_target_ey;
      }
      xr[nx + i * nx] = clip(xr[nx + i * nx], lb[i], ub[i]);
    }

    validate_mpc_preflight(lb, ub, xr, ur, umax_dyn, N, nx);

    std::vector<Eigen::Triplet<double>> a_triplets;
    for (int row = 0; row < nx_N; ++row) {
      a_triplets.emplace_back(row, row, -1.0);
    }
    for (int r = 0; r < nx_N; ++r) {
      for (int c = 0; c < nx_N; ++c) {
        const double value = A_dense(r, c);
        if (value != 0.0) {
          a_triplets.emplace_back(r, c, value);
        }
      }
    }
    for (int r = 0; r < nx_N; ++r) {
      for (int c = 0; c < nu_N; ++c) {
        const double value = B_dense(r, c);
        if (value != 0.0) {
          a_triplets.emplace_back(r, nx_N + c, value);
        }
      }
    }

    const int ineq_offset = nx_N;
    for (int i = 0; i < nx_N + nu_N; ++i) {
      a_triplets.emplace_back(ineq_offset + i, i, 1.0);
    }

    const int rate_offset = nx_N + nx_N + nu_N;
    a_triplets.emplace_back(rate_offset, nx_N + 1, 1.0);
    for (int i = 0; i < N - 1; ++i) {
      a_triplets.emplace_back(rate_offset + i + 1, nx_N + nu * i + 1, -1.0);
      a_triplets.emplace_back(rate_offset + i + 1, nx_N + nu * (i + 1) + 1, 1.0);
    }

    Eigen::SparseMatrix<double> A_full(nx_N + nx_N + nu_N + N, nx_N + nu_N);
    A_full.setFromTriplets(a_triplets.begin(), a_triplets.end());

    Eigen::Vector3d x0;
    x0 << model->spatial_state.e_y, model->spatial_state.e_psi, model->spatial_state.t;
    Eigen::VectorXd leq(nx_N);
    leq.segment<3>(0) = -x0;
    leq.segment(3, uq.size()) = uq;
    Eigen::VectorXd ueq = leq;

    Eigen::VectorXd lineq_basic(nx_N + nu_N);
    Eigen::VectorXd uineq_basic(nx_N + nu_N);
    lineq_basic << xmin_dyn, umin_dyn;
    uineq_basic << xmax_dyn, umax_dyn;

    const double max_delta_change = cfg.steer_rate_max * model->Ts;
    const double max_kappa_change = std::tan(max_delta_change) / model->length;
    const double previous_kappa = std::tan(previous_steering) / model->length;
    Eigen::VectorXd lineq_rate = Eigen::VectorXd::Constant(N, -max_kappa_change);
    Eigen::VectorXd uineq_rate = Eigen::VectorXd::Constant(N, max_kappa_change);
    lineq_rate[0] = previous_kappa - max_kappa_change;
    uineq_rate[0] = previous_kappa + max_kappa_change;

    Eigen::VectorXd l(leq.size() + lineq_basic.size() + lineq_rate.size());
    Eigen::VectorXd u(ueq.size() + uineq_basic.size() + uineq_rate.size());
    l << leq, lineq_basic, lineq_rate;
    u << ueq, uineq_basic, uineq_rate;

    std::vector<Eigen::Triplet<double>> p_triplets;
    for (int n = 0; n < N; ++n) {
      for (int i = 0; i < 3; ++i) {
        p_triplets.emplace_back(n * 3 + i, n * 3 + i, cfg.Q[i]);
      }
    }
    for (int i = 0; i < 3; ++i) {
      p_triplets.emplace_back(N * 3 + i, N * 3 + i, cfg.QN[i]);
    }
    const int input_offset = nx_N;
    for (int n = 0; n < N; ++n) {
      for (int i = 0; i < 2; ++i) {
        p_triplets.emplace_back(input_offset + n * 2 + i, input_offset + n * 2 + i, cfg.R[i]);
      }
    }
    Eigen::SparseMatrix<double> P(nx_N + nu_N, nx_N + nu_N);
    P.setFromTriplets(p_triplets.begin(), p_triplets.end());

    Eigen::VectorXd q(nx_N + nu_N);
    for (int n = 0; n < N; ++n) {
      for (int i = 0; i < 3; ++i) {
        q[n * 3 + i] = -cfg.Q[i] * xr[n * 3 + i];
      }
    }
    q.segment<3>(N * 3) = -(cfg.QN.asDiagonal() * xr.segment<3>(N * 3));
    for (int n = 0; n < N; ++n) {
      for (int i = 0; i < 2; ++i) {
        q[nx_N + n * 2 + i] = -cfg.R[i] * ur[n * 2 + i];
      }
    }

    return MpcProblem{q, l, u, P, A_full, N, base_wp_id, planning_wp_id, ref_wp_id};
  }

  OsqpSolveOutcome solve_problem(const MpcProblem & problem)
  {
    auto outcome = solve_osqp(problem.P, problem.A, problem.q, problem.l, problem.u);
    if (
      outcome.result.has_value() &&
      outcome.result->solution.size() != problem.P.rows())
    {
      std::ostringstream detail;
      detail << "stage=solution, reason=unexpected solution size, actual="
             << outcome.result->solution.size() << ", expected=" << problem.P.rows();
      return osqp_failure(detail.str());
    }
    return outcome;
  }

  void ensure_current_control_horizon()
  {
    const int required_size = 2 * cfg.N;
    if (required_size <= 0 || current_control.size() >= required_size) {
      return;
    }

    double padding_speed = std::isfinite(current_speed_mps_) ?
      std::max(0.0, current_speed_mps_) : 0.0;
    double padding_steering = std::isfinite(previous_steering) ? previous_steering : 0.0;
    if (current_control.size() >= 2) {
      const double last_speed = current_control[current_control.size() - 2];
      const double last_steering = current_control[current_control.size() - 1];
      if (std::isfinite(last_speed)) {
        padding_speed = std::max(0.0, last_speed);
      }
      if (std::isfinite(last_steering)) {
        padding_steering = last_steering;
      }
    }

    Eigen::VectorXd padded_control = Eigen::VectorXd::Zero(required_size);
    const int copy_size = static_cast<int>(current_control.size());
    if (copy_size > 0) {
      padded_control.head(copy_size) = current_control;
    }
    for (int i = copy_size / 2; i < cfg.N; ++i) {
      padded_control[2 * i] = padding_speed;
      padded_control[2 * i + 1] = padding_steering;
    }
    current_control = std::move(padded_control);
  }

  std::pair<Eigen::Vector2d, double> safe_failure_control(const std::string & reason)
  {
    last_control_was_fallback_ = true;
    current_prediction.first.clear();
    current_prediction.second.clear();

    const double current_speed = std::isfinite(current_speed_mps_) ?
      std::max(0.0, current_speed_mps_) : 0.0;
    double fallback_base_speed = current_speed;
    if (failure_fallback_speed_.has_value() && std::isfinite(failure_fallback_speed_.value())) {
      fallback_base_speed = std::min(fallback_base_speed, failure_fallback_speed_.value());
    }

    const double deceleration = std::isfinite(cfg.a_min) ? std::max(0.0, -cfg.a_min) : 0.0;
    const double time_step = std::isfinite(model->Ts) ? std::max(0.0, model->Ts) : 0.0;
    const double fallback_speed = deceleration > kEps && time_step > kEps ?
      std::max(0.0, fallback_base_speed - deceleration * time_step) : 0.0;
    failure_fallback_speed_ = fallback_speed;

    const int failure_count = infeasibility_counter < std::numeric_limits<int>::max() ?
      infeasibility_counter + 1 : std::numeric_limits<int>::max();

    const double max_steering = std::isfinite(cfg.delta_max) ?
      std::max(0.0, std::abs(cfg.delta_max)) : 0.0;
    const bool overtake_recovery_phase =
      overtake_line_state_.phase == OvertakeLinePhase::Recovery;
    const bool force_neutralize_overtake_fallback =
      overtake_recovery_phase || overtake_solver_recovery_active_ ||
      overtake_solver_reentry_blocked_;
    const bool neutralize_fallback =
      v2x_overtake_core::should_neutralize_solver_fallback_steering(
        v2x_overtake_core::SolverFallbackNeutralizationRequest{
          failure_count, cfg.solver_failure_steering_hold_cycles,
          force_neutralize_overtake_fallback});
    const double fallback_steering = neutralize_fallback ?
      v2x_overtake_core::rate_limit_solver_fallback_steering_toward_neutral(
        v2x_overtake_core::SolverFallbackSteeringRequest{
          std::isfinite(previous_steering) ? previous_steering : 0.0,
          max_steering, std::max(0.0, cfg.steer_rate_max), time_step}) :
      (std::isfinite(previous_steering) ?
       clip(previous_steering, -max_steering, max_steering) : 0.0);
    previous_steering = fallback_steering;

    const int safe_horizon = std::max(0, cfg.N);
    current_control = Eigen::VectorXd::Zero(2 * safe_horizon);
    for (int i = 0; i < safe_horizon; ++i) {
      current_control[2 * i] = fallback_speed;
      current_control[2 * i + 1] = fallback_steering;
    }

    const bool active_overtake_phase =
      overtake_line_state_.phase == OvertakeLinePhase::ShiftOut ||
      overtake_line_state_.phase == OvertakeLinePhase::Pass;
    const bool overtake_solver_relevant_phase =
      active_overtake_phase || overtake_recovery_phase;
    overtake_infeasibility_counter_ = overtake_solver_relevant_phase ?
      overtake_infeasibility_counter_ + 1 : 0;
    if (!overtake_solver_recovery_active_ && overtake_recovery_phase) {
      overtake_solver_recovery_active_ = true;
      RCLCPP_ERROR(
        rclcpp::get_logger("mpc_controller"),
        "MPC overtake Recovery entered solver fallback; re-entry gate requested");
    }
    if (
      !overtake_solver_recovery_active_ &&
      active_overtake_phase &&
      overtake_infeasibility_counter_ >=
      cfg.v2x_behavior.overtake_line.solver_failure_abort_cycles)
    {
      overtake_solver_recovery_active_ = true;
      RCLCPP_ERROR(
        rclcpp::get_logger("mpc_controller"),
        "MPC overtake target aborted after %d consecutive failures; Recovery requested",
        overtake_infeasibility_counter_);
    }
    if (overtake_solver_reentry_blocked_) {
      const auto gate = v2x_overtake_core::update_solver_reentry_gate(
        v2x_overtake_core::SolverReentryGateRequest{
          false, overtake_solver_reentry_blocked_,
          overtake_solver_recovery_success_count_, false, true,
          cfg.v2x_behavior.overtake_line.solver_recovery_success_cycles});
      overtake_solver_reentry_blocked_ = gate.blocked;
      overtake_solver_recovery_success_count_ = gate.consecutive_successes;
    }
    const bool neutralization_just_started =
      !force_neutralize_overtake_fallback && neutralize_fallback &&
      cfg.solver_failure_steering_hold_cycles < std::numeric_limits<int>::max() &&
      failure_count == cfg.solver_failure_steering_hold_cycles + 1;
    if (infeasibility_counter == 0 || neutralization_just_started || failure_count % 10 == 0) {
      RCLCPP_ERROR(
        rclcpp::get_logger("mpc_controller"),
        "MPC control failed; using deceleration fallback: reason=%s, failures=%d, "
        "speed=%.3f, steering=%.3f, steering_mode=%s",
        reason.c_str(), failure_count, fallback_speed, fallback_steering,
        neutralize_fallback ? "neutralize" : "hold");
    }
    infeasibility_counter = failure_count;
    Eigen::Vector2d fallback;
    fallback << fallback_speed, fallback_steering;
    return {fallback, std::abs(fallback_steering)};
  }

  std::pair<Eigen::Vector2d, double> low_speed_shift_control(
    const Waypoint & reference_waypoint)
  {
    const double max_steering = std::max(0.0, std::abs(cfg.delta_max));
    const double target_steering =
      v2x_overtake_core::resolve_low_speed_shift_steering(
      v2x_overtake_core::LowSpeedShiftSteeringRequest{
        model->spatial_state.e_y,
        model->spatial_state.e_psi,
        low_speed_shift_target_ey_,
        reference_waypoint.kappa,
        model->length,
        max_steering,
        cfg.v2x_behavior.low_speed_avoidance_shift_lateral_gain,
        cfg.v2x_behavior.low_speed_avoidance_shift_heading_gain});
    const double max_steering_step =
      std::max(0.0, cfg.steer_rate_max) * std::max(0.0, model->Ts);
    const double steering = clip(
      target_steering, previous_steering - max_steering_step,
      previous_steering + max_steering_step);
    const double speed = std::max(0.0, low_speed_shift_velocity_mps_);

    if (!low_speed_shift_control_was_active_) {
      RCLCPP_INFO(
        rclcpp::get_logger("mpc_controller"),
        "Low-speed pass shift control entered: target_ey=%.2f, speed=%.2f, "
        "lateral_gain=%.2f, heading_gain=%.2f, completion=%.2f m/%.2f rad, "
        "vehicle_clear_hold=%.2f s",
        low_speed_shift_target_ey_, speed,
        cfg.v2x_behavior.low_speed_avoidance_shift_lateral_gain,
        cfg.v2x_behavior.low_speed_avoidance_shift_heading_gain,
        cfg.v2x_behavior.low_speed_avoidance_shift_lateral_tolerance,
        cfg.v2x_behavior.low_speed_avoidance_shift_heading_tolerance,
        cfg.v2x_behavior.low_speed_avoidance_shift_clear_hold_sec);
    }
    low_speed_shift_control_was_active_ = true;
    previous_steering = steering;
    current_prediction.first.clear();
    current_prediction.second.clear();
    current_control = Eigen::VectorXd::Zero(2 * std::max(0, cfg.N));
    for (int i = 0; i < cfg.N; ++i) {
      current_control[2 * i] = speed;
      current_control[2 * i + 1] = steering;
    }
    failure_fallback_speed_.reset();
    infeasibility_counter = 0;
    overtake_infeasibility_counter_ = 0;
    last_control_was_fallback_ = false;
    return {Eigen::Vector2d(speed, steering), std::abs(steering)};
  }

  std::pair<Eigen::Vector2d, double> get_control(const double now_sec)
  {
    constexpr int nx = 3;
    constexpr int nu = 2;
    if (cfg.N < 2) {
      return safe_failure_control("mpc.N must be at least 2");
    }
    try {
      model->get_current_waypoint();
      const int base_wp_id = model->wp_id;
      const int planning_wp_id = get_planning_wp_id(base_wp_id);
      const int remaining_segments =
        std::max(0, model->reference_path->n_waypoints - 1 - planning_wp_id);
      const int N = model->reference_path->circular ?
        cfg.N : std::min(cfg.N, remaining_segments);
      if (N < 2) {
        throw std::runtime_error("MPC horizon has fewer than 2 steps");
      }
      ensure_current_control_horizon();

      const auto & planning_waypoint =
        model->reference_path->get_waypoint(planning_wp_id);
      model->spatial_state = model->t2s(planning_waypoint, model->temporal_state);
      const MpcProblem problem =
        init_problem(N, model->safety_margin, now_sec, base_wp_id, planning_wp_id);
      if (low_speed_shift_control_active_) {
        return low_speed_shift_control(planning_waypoint);
      }
      if (low_speed_shift_control_was_active_) {
        RCLCPP_INFO(
          rclcpp::get_logger("mpc_controller"),
          "Low-speed pass shift control completed: e_y=%.2f, e_psi=%.2f; returning to MPC",
          model->spatial_state.e_y, model->spatial_state.e_psi);
        low_speed_shift_control_was_active_ = false;
        low_speed_shift_last_relevant_vehicle_sec_ =
          std::numeric_limits<double>::quiet_NaN();
      }
      auto outcome = solve_problem(problem);
      if (!outcome.result.has_value()) {
        throw std::runtime_error("OSQP failed: " + outcome.failure_detail);
      }
      Eigen::VectorXd dec = outcome.result->solution;
      Eigen::VectorXd control_signals = dec.tail(N * nu);

      for (int i = 1; i < control_signals.size(); i += 2) {
        control_signals[i] = std::atan(control_signals[i] * model->length);
      }
      const double v = control_signals[0];
      double delta = control_signals[1];
      const double max_delta_change = cfg.steer_rate_max * model->Ts;
      delta = clip(delta, previous_steering - max_delta_change, previous_steering + max_delta_change);
      control_signals[1] = delta;
      if (!std::isfinite(v) || !std::isfinite(delta) || !control_signals.allFinite()) {
        throw std::runtime_error("MPC postprocessed control is not finite");
      }

      auto prediction =
        update_prediction(dec.head((N + 1) * nx), N, problem.planning_wp_id);
      Eigen::Vector2d u(v, delta);

      double max_delta = 0.0;
      const int end = static_cast<int>(control_signals.size() / 3) * 2;
      for (int i = 1; i < end; i += 2) {
        max_delta = std::max(max_delta, std::abs(control_signals[i]));
      }

      previous_steering = delta;
      current_control = std::move(control_signals);
      current_prediction = std::move(prediction);
      if (infeasibility_counter > 0) {
        RCLCPP_INFO(
          rclcpp::get_logger("mpc_controller"),
          "MPC solver recovered after %d consecutive failures", infeasibility_counter);
      }
      failure_fallback_speed_.reset();
      infeasibility_counter = 0;
      overtake_infeasibility_counter_ = 0;
      last_control_was_fallback_ = false;
      if (overtake_solver_reentry_blocked_) {
        const bool cooldown_active = v2x_overtake_core::is_solver_cooldown_active(
          now_sec, overtake_solver_cooldown_until_sec_);
        const auto gate = v2x_overtake_core::update_solver_reentry_gate(
          v2x_overtake_core::SolverReentryGateRequest{
            false, overtake_solver_reentry_blocked_,
            overtake_solver_recovery_success_count_, true, cooldown_active,
            cfg.v2x_behavior.overtake_line.solver_recovery_success_cycles});
        overtake_solver_reentry_blocked_ = gate.blocked;
        overtake_solver_recovery_success_count_ = gate.consecutive_successes;
        if (gate.released) {
          overtake_solver_cooldown_logged_ = false;
          RCLCPP_INFO(
            rclcpp::get_logger("mpc_controller"),
            "OvertakeLine solver re-entry gate released after %d healthy solves",
            cfg.v2x_behavior.overtake_line.solver_recovery_success_cycles);
        }
      }
      last_solved_wp_id = problem.planning_wp_id;
      return {u, max_delta};
    } catch (const std::exception & error) {
      return safe_failure_control(error.what());
    } catch (...) {
      return safe_failure_control("unknown exception");
    }
  }

  std::pair<std::vector<double>, std::vector<double>> update_prediction(
    const Eigen::VectorXd & spatial_state_prediction_flat, const int N,
    const int planning_wp_id)
  {
    std::pair<std::vector<double>, std::vector<double>> out;
    for (int n = 2; n < N; ++n) {
      const auto & associated_waypoint =
        model->reference_path->get_waypoint(planning_wp_id + n);
      Eigen::Vector3d pred_state = spatial_state_prediction_flat.segment<3>(n * 3);
      const auto temporal = model->s2t(associated_waypoint, pred_state);
      out.first.push_back(temporal.x);
      out.second.push_back(temporal.y);
    }
    return out;
  }

  BicycleModel * model{};
  MpcConfig cfg;
  start_grid_grace::Guard start_grid_grace_guard_;
  V2XGapPlanner * gap_planner{};
  bool use_obstacle_avoidance{};
  bool use_path_constraints_topic{};
  V2XBehaviorState v2x_behavior_state{V2XBehaviorState::Cruise};
  V2XBehaviorOutput last_v2x_behavior_output_;
  bool v2x_behavior_state_initialized{false};
  double last_v2x_behavior_state_change_sec{-std::numeric_limits<double>::infinity()};
  double last_v2x_behavior_debug_log_sec_{std::numeric_limits<double>::quiet_NaN()};
  double low_speed_avoidance_stall_since_sec_{std::numeric_limits<double>::quiet_NaN()};
  double low_speed_avoidance_stall_last_update_sec_{std::numeric_limits<double>::quiet_NaN()};
  double low_speed_avoidance_stall_cooldown_until_sec_{
    -std::numeric_limits<double>::infinity()};
  double front_hazard_hold_until_sec_{0.0};
  std::string front_hazard_hold_target_id_;
  bool start_grid_stop_suppressed_{false};
  bool start_grid_emergency_override_logged_{false};
  std::optional<std::string> start_grid_initial_target_id_;
  std::optional<double> overtake_locked_target_ey_;
  int overtake_locked_side_sign_{0};
  OvertakeLineState overtake_line_state_;
  std::optional<double> overtake_entry_speed_;
  bool overtake_solver_recovery_active_{false};
  bool overtake_solver_reentry_blocked_{false};
  int overtake_solver_recovery_success_count_{0};
  double last_overtake_line_debug_log_sec_{std::numeric_limits<double>::quiet_NaN()};
  double overtake_solver_cooldown_until_sec_{-std::numeric_limits<double>::infinity()};
  bool overtake_solver_cooldown_logged_{false};
  double overtake_curve_cooldown_until_sec_{-std::numeric_limits<double>::infinity()};
  bool low_speed_shift_control_active_{false};
  bool low_speed_shift_control_was_active_{false};
  double low_speed_shift_target_ey_{0.0};
  double low_speed_shift_velocity_mps_{0.0};
  double low_speed_shift_last_relevant_vehicle_sec_{
    std::numeric_limits<double>::quiet_NaN()};
  double previous_steering{0.0};
  double current_speed_mps_{0.0};

  int effective_wp_id_offset() const
  {
    if (cfg.wp_id_low_speed > kEps && current_speed_mps_ <= cfg.wp_id_low_speed) {
      return cfg.wp_id_low_offset;
    }
    return cfg.wp_id_offset;
  }

  int get_planning_wp_id(const int base_wp_id) const
  {
    const int candidate_wp_id = base_wp_id + effective_wp_id_offset();
    const int waypoint_count = model->reference_path->n_waypoints;
    if (model->reference_path->circular) {
      return ((candidate_wp_id % waypoint_count) + waypoint_count) % waypoint_count;
    }
    return std::clamp(candidate_wp_id, 0, waypoint_count - 1);
  }
  std::pair<std::vector<double>, std::vector<double>> current_prediction;
  int infeasibility_counter{0};
  int overtake_infeasibility_counter_{0};
  int last_solved_wp_id{0};
  Eigen::VectorXd current_control;
  std::optional<double> failure_fallback_speed_;
  bool last_control_was_fallback_{false};

private:
  void update_start_grid_suppression_diagnostics(
    const bool suppressed, const bool emergency_override, const std::string & target_id,
    const double front_distance, const double front_speed, const double required_decel)
  {
    if (suppressed && !start_grid_stop_suppressed_) {
      RCLCPP_INFO(
        rclcpp::get_logger("mpc_controller"),
        "Start grid static stop suppressed: target=%s, distance=%.2f m, speed=%.2f m/s",
        target_id.c_str(), front_distance, front_speed);
    } else if (!suppressed && start_grid_stop_suppressed_) {
      RCLCPP_INFO(
        rclcpp::get_logger("mpc_controller"),
        "Start grid static stop suppression released: target=%s",
        target_id.c_str());
    }
    start_grid_stop_suppressed_ = suppressed;

    if (emergency_override && !start_grid_emergency_override_logged_) {
      RCLCPP_WARN(
        rclcpp::get_logger("mpc_controller"),
        "Start grid suppression overridden by front-risk emergency: target=%s, "
        "distance=%.2f m, speed=%.2f m/s, required_decel=%.2f m/s^2",
        target_id.c_str(), front_distance, front_speed, required_decel);
    }
    start_grid_emergency_override_logged_ = emergency_override;
  }

  static void validate_mpc_preflight(
    const Eigen::VectorXd & lb, const Eigen::VectorXd & ub, const Eigen::VectorXd & xr,
    const Eigen::VectorXd & ur, const Eigen::VectorXd & umax_dyn, const int N, const int nx)
  {
    if (
      N <= 0 || nx <= 0 || lb.size() < N || ub.size() < N ||
      xr.size() < nx * (N + 1) || ur.size() < 2 * N || umax_dyn.size() < 2 * N)
    {
      throw std::runtime_error("MPC preflight rejected inconsistent horizon sizes");
    }
    for (int i = 0; i < N; ++i) {
      const double lower = lb[i];
      const double upper = ub[i];
      const double target_ey = xr[nx + i * nx];
      const double target_speed = ur[2 * i];
      const double target_kappa = ur[2 * i + 1];
      const double speed_limit = umax_dyn[2 * i];
      if (
        !std::isfinite(lower) || !std::isfinite(upper) || lower > upper ||
        !std::isfinite(target_ey) || target_ey < lower - 1e-6 || target_ey > upper + 1e-6 ||
        !std::isfinite(target_speed) || target_speed < 0.0 ||
        !std::isfinite(speed_limit) || target_speed > speed_limit + 1e-6 ||
        !std::isfinite(target_kappa))
      {
        std::ostringstream message;
        message << "MPC preflight rejected horizon index " << i
                << ": bounds=[" << lower << "," << upper << "]"
                << ", target_ey=" << target_ey
                << ", target_speed=" << target_speed
                << ", speed_limit=" << speed_limit
                << ", target_kappa=" << target_kappa;
        throw std::runtime_error(message.str());
      }
    }
  }

  void apply_velocity_limit(
    Eigen::VectorXd & umax_dyn, Eigen::VectorXd & ur, const int N, const double velocity_limit)
  {
    const auto limits = mpc_velocity_limit::build_reachable_limits(
      mpc_velocity_limit::ReachableLimitRequest{
        N,
        std::max(0.0, current_speed_mps_),
        std::max(0.0, velocity_limit),
        std::max(0.0, -cfg.a_min),
        model->Ts});
    for (int n = 0; n < N; ++n) {
      const double limit = limits[static_cast<std::size_t>(n)];
      umax_dyn[2 * n] = std::min(umax_dyn[2 * n], limit);
      ur[2 * n] = std::min(ur[2 * n], limit);
    }
  }

  void reset_overtake_line_state(const double now_sec, const std::string & reason)
  {
    if (overtake_solver_recovery_active_) {
      const auto gate = v2x_overtake_core::update_solver_reentry_gate(
        v2x_overtake_core::SolverReentryGateRequest{
          true, overtake_solver_reentry_blocked_,
          overtake_solver_recovery_success_count_, false, true,
          cfg.v2x_behavior.overtake_line.solver_recovery_success_cycles});
      overtake_solver_reentry_blocked_ = gate.blocked;
      overtake_solver_recovery_success_count_ = gate.consecutive_successes;
      overtake_solver_cooldown_logged_ = false;
      if (
        std::isfinite(now_sec) &&
        cfg.v2x_behavior.overtake_line.solver_cooldown_sec > 0.0)
      {
        overtake_solver_cooldown_until_sec_ = v2x_overtake_core::arm_solver_cooldown(
          v2x_overtake_core::SolverCooldownRequest{
            now_sec, overtake_solver_cooldown_until_sec_,
            cfg.v2x_behavior.overtake_line.solver_cooldown_sec});
      }
      RCLCPP_WARN(
        rclcpp::get_logger("mpc_controller"),
        "OvertakeLine solver re-entry gate armed: cooldown=%.2f s, "
        "healthy_solves=%d, reason=%s",
        cfg.v2x_behavior.overtake_line.solver_cooldown_sec,
        cfg.v2x_behavior.overtake_line.solver_recovery_success_cycles, reason.c_str());
    }
    if (
      cfg.v2x_behavior.overtake_line.debug_log_enabled &&
      overtake_line_state_.phase != OvertakeLinePhase::Idle) {
      RCLCPP_INFO(
        rclcpp::get_logger("mpc_controller"),
        "OvertakeLine: %s -> Idle, side=%d, wp_id=%d, reason=%s",
        to_string(overtake_line_state_.phase), overtake_line_state_.pass_side_sign, model->wp_id,
        reason.c_str());
    }
    overtake_line_state_ = OvertakeLineState{};
    overtake_line_state_.phase_start_sec = now_sec;
    overtake_solver_recovery_active_ = false;
  }

  void transition_overtake_line_phase(
    const OvertakeLinePhase next_phase, const double now_sec, const double current_ey,
    const int pass_side_sign, const std::string & reason)
  {
    if (overtake_line_state_.phase == next_phase) {
      if (pass_side_sign != 0 && overtake_line_state_.pass_side_sign == 0) {
        overtake_line_state_.pass_side_sign = pass_side_sign;
      }
      return;
    }

    if (cfg.v2x_behavior.overtake_line.debug_log_enabled) {
      RCLCPP_INFO(
        rclcpp::get_logger("mpc_controller"),
        "OvertakeLine: %s -> %s, side=%d, ey=%.2f, wp_id=%d, reason=%s",
        to_string(overtake_line_state_.phase), to_string(next_phase),
        pass_side_sign != 0 ? pass_side_sign : overtake_line_state_.pass_side_sign, current_ey,
        model->wp_id, reason.c_str());
    }

    overtake_line_state_.phase = next_phase;
    if (next_phase != OvertakeLinePhase::Pass) {
      overtake_line_state_.pass_front_overlap_exclusion_latched = false;
    }
    overtake_line_state_.phase_start_sec = now_sec;
    overtake_line_state_.phase_start_ey = current_ey;
    overtake_line_state_.target_ey = current_ey;
    overtake_line_state_.phase_traveled_m = 0.0;
    overtake_line_state_.phase_last_update_sec = now_sec;
    overtake_line_state_.recovery_stall_since_sec =
      next_phase == OvertakeLinePhase::Recovery &&
      current_speed_mps_ <= cfg.v2x_behavior.overtake_line.recovery_stall_speed ?
      now_sec : std::numeric_limits<double>::quiet_NaN();
    if (pass_side_sign != 0) {
      overtake_line_state_.pass_side_sign = pass_side_sign;
    }
  }

  double overtake_line_goal_ey() const
  {
    const int pass_side_sign = overtake_line_state_.pass_side_sign;
    if (
      overtake_line_state_.phase == OvertakeLinePhase::ShiftOut ||
      overtake_line_state_.phase == OvertakeLinePhase::Pass) {
      return overtake_core::resolve_pass_side_lateral_goal(
        overtake_core::PassSideLateralGoalRequest{
          pass_side_sign,
          std::max(0.0, cfg.v2x_behavior.overtake_line.lateral_offset),
          overtake_line_state_.target_last_lateral,
          std::max(0.0, cfg.v2x_behavior.overtake_pass_front_overlap_lateral_clearance)});
    }
    return 0.0;
  }

  double overtake_line_phase_distance() const
  {
    const auto & line_cfg = cfg.v2x_behavior.overtake_line;
    switch (overtake_line_state_.phase) {
      case OvertakeLinePhase::ShiftOut:
        return std::max(0.5, line_cfg.shift_distance);
      case OvertakeLinePhase::Pass:
        return std::max(0.5, line_cfg.pass_distance);
      case OvertakeLinePhase::Return:
      case OvertakeLinePhase::Recovery:
        return std::max(0.5, line_cfg.return_distance);
      case OvertakeLinePhase::Idle:
      case OvertakeLinePhase::FollowPrepare:
        return std::max(0.5, line_cfg.shift_distance);
    }
    return std::max(0.5, line_cfg.shift_distance);
  }

  double overtake_shiftout_remaining_distance(const double current_ey) const
  {
    const double shift_distance =
      std::max(0.5, cfg.v2x_behavior.overtake_line.shift_distance);
    const double distance_remaining = std::max(
      0.0, shift_distance - overtake_line_state_.phase_traveled_m);
    const double goal_ey = overtake_line_goal_ey();
    const double total_lateral_shift =
      std::abs(goal_ey - overtake_line_state_.phase_start_ey);
    if (!std::isfinite(current_ey) || total_lateral_shift <= kEps) {
      return distance_remaining;
    }
    const double lateral_remaining_ratio = clip(
      std::abs(goal_ey - current_ey) / total_lateral_shift, 0.0, 1.0);
    return std::max(distance_remaining, shift_distance * lateral_remaining_ratio);
  }

  double limit_overtake_line_goal_change(const double raw_goal)
  {
    const double max_change = std::max(0.0, cfg.v2x_behavior.overtake_line.max_target_change);
    if (max_change <= kEps) {
      overtake_line_state_.target_ey = raw_goal;
      return raw_goal;
    }
    const double limited_goal = clip(
      raw_goal, overtake_line_state_.target_ey - max_change,
      overtake_line_state_.target_ey + max_change);
    overtake_line_state_.target_ey = limited_goal;
    return limited_goal;
  }

  OvertakeLineOutput update_overtake_line(
    const V2XBehaviorOutput & behavior_output, const int ref_wp_id, const int N,
    const Eigen::VectorXd & lb, const Eigen::VectorXd & ub, const double now_sec)
  {
    OvertakeLineOutput output;
    const auto & line_cfg = cfg.v2x_behavior.overtake_line;
    if (!line_cfg.enabled || N <= 0) {
      reset_overtake_line_state(now_sec, "disabled");
      return output;
    }

    const double current_ey = model->spatial_state.e_y;
    if (
      behavior_output.state == V2XBehaviorState::SafetyBrake ||
      behavior_output.front_risk_level == FrontRiskLevel::EmergencyBrake) {
      reset_overtake_line_state(now_sec, "safety brake");
      return output;
    }
    if (behavior_output.state == V2XBehaviorState::LowSpeedAvoidance) {
      reset_overtake_line_state(now_sec, "low speed avoidance owns target");
      return output;
    }
    const bool stopped_or_low_speed_front =
      behavior_output.has_front_vehicle &&
      std::isfinite(behavior_output.front_distance) &&
      std::isfinite(behavior_output.front_speed) &&
      behavior_output.front_speed <= cfg.v2x_behavior.low_speed_avoidance_max_front_speed &&
      behavior_output.front_distance <= std::max(
        cfg.v2x_behavior.low_speed_avoidance_distance + model->length,
        cfg.v2x_behavior.low_speed_avoidance_lookahead_distance);
    if (behavior_output.low_speed_avoidance_candidate || stopped_or_low_speed_front) {
      reset_overtake_line_state(now_sec, "stopped vehicle bypass owns target");
      return output;
    }

    const bool phase_active = overtake_line_state_.phase != OvertakeLinePhase::Idle;
    const bool locked_target_matches =
      !phase_active || overtake_line_state_.target_vehicle_id.empty() ||
      (behavior_output.locked_target_seen &&
      !behavior_output.locked_target_position_jump &&
      behavior_output.target_vehicle_id == overtake_line_state_.target_vehicle_id);
    const bool solver_cooldown_active = v2x_overtake_core::is_solver_cooldown_active(
      now_sec, overtake_solver_cooldown_until_sec_);
    const bool solver_reentry_suppressed =
      solver_cooldown_active || overtake_solver_reentry_blocked_;
    const bool behavior_requests_overtake =
      behavior_output.state == V2XBehaviorState::Overtake &&
      behavior_output.overtake_pass_side_sign != 0;
    if (solver_reentry_suppressed && behavior_requests_overtake) {
      if (!overtake_solver_cooldown_logged_) {
        RCLCPP_INFO(
          rclcpp::get_logger("mpc_controller"),
          "OvertakeLine start suppressed by solver recovery gate: remaining=%.2f s, "
          "healthy_solves=%d/%d",
          std::max(0.0, overtake_solver_cooldown_until_sec_ - now_sec),
          overtake_solver_recovery_success_count_,
          line_cfg.solver_recovery_success_cycles);
        overtake_solver_cooldown_logged_ = true;
      }
    } else if (!solver_reentry_suppressed) {
      overtake_solver_cooldown_logged_ = false;
    }
    const bool behavior_overtake =
      behavior_requests_overtake && !overtake_solver_recovery_active_ &&
      !solver_reentry_suppressed && locked_target_matches;

    if (
      overtake_line_state_.target_vehicle_id.empty() && behavior_overtake &&
      !behavior_output.target_vehicle_id.empty())
    {
      overtake_line_state_.target_vehicle_id = behavior_output.target_vehicle_id;
      overtake_line_state_.target_last_seen_sec = now_sec;
      overtake_line_state_.target_last_longitudinal = behavior_output.has_front_vehicle ?
        behavior_output.front_distance : 0.0;
      overtake_line_state_.target_last_lateral = std::isfinite(behavior_output.front_lateral) ?
        behavior_output.front_lateral : std::numeric_limits<double>::infinity();
      overtake_line_state_.target_last_speed = std::isfinite(behavior_output.front_speed) ?
        std::max(0.0, behavior_output.front_speed) :
        std::numeric_limits<double>::infinity();
    }
    if (behavior_output.locked_target_seen) {
      overtake_line_state_.target_last_seen_sec = now_sec;
      overtake_line_state_.target_last_longitudinal =
        behavior_output.locked_target_longitudinal;
      if (std::isfinite(behavior_output.locked_target_lateral)) {
        overtake_line_state_.target_last_lateral = behavior_output.locked_target_lateral;
      }
      if (std::isfinite(behavior_output.locked_target_speed)) {
        overtake_line_state_.target_last_speed =
          std::max(0.0, behavior_output.locked_target_speed);
      }
    }
    const double target_age = std::isfinite(overtake_line_state_.target_last_seen_sec) ?
      std::max(0.0, now_sec - overtake_line_state_.target_last_seen_sec) :
      std::numeric_limits<double>::infinity();
    const bool rear_clear_observed =
      behavior_output.locked_target_seen &&
      behavior_output.locked_target_longitudinal <=
      -std::max(0.0, line_cfg.return_clear_distance);
    if (rear_clear_observed) {
      if (!std::isfinite(overtake_line_state_.rear_clear_since_sec)) {
        overtake_line_state_.rear_clear_since_sec = now_sec;
      }
    } else {
      overtake_line_state_.rear_clear_since_sec = std::numeric_limits<double>::quiet_NaN();
    }
    const bool rear_clear_confirmed =
      rear_clear_observed &&
      now_sec - overtake_line_state_.rear_clear_since_sec >=
      std::max(0.0, line_cfg.clear_confirm_sec);

    if (behavior_overtake) {
      const int pass_side_sign = overtake_line_state_.pass_side_sign != 0 ?
        overtake_line_state_.pass_side_sign : behavior_output.overtake_pass_side_sign;
      if (!phase_active || overtake_line_state_.phase == OvertakeLinePhase::FollowPrepare) {
        transition_overtake_line_phase(
          OvertakeLinePhase::ShiftOut, now_sec, current_ey, pass_side_sign,
          "overtake selected");
      } else if (overtake_line_state_.pass_side_sign == 0) {
        overtake_line_state_.pass_side_sign = pass_side_sign;
      } else if (overtake_line_state_.phase == OvertakeLinePhase::Return) {
        const bool stable_target_id =
          !overtake_line_state_.target_vehicle_id.empty() &&
          overtake_line_state_.target_vehicle_id != "__unknown__";
        const bool same_target =
          stable_target_id &&
          behavior_output.target_vehicle_id == overtake_line_state_.target_vehicle_id;
        const bool same_side =
          behavior_output.overtake_pass_side_sign == overtake_line_state_.pass_side_sign;
        const double return_elapsed = std::max(
          0.0, now_sec - overtake_line_state_.phase_start_sec);
        const double return_denominator = std::max(
          0.1, std::abs(overtake_line_state_.phase_start_ey));
        const double return_progress = clip(
          (std::abs(overtake_line_state_.phase_start_ey) - std::abs(current_ey)) /
          return_denominator, 0.0, 1.0);
        if (overtake_core::can_reacquire_during_return(
            overtake_core::ReacquireRequest{
              line_cfg.reacquire_enabled, stable_target_id, same_target, same_side,
              behavior_output.overtake_gap_available, behavior_output.overtake_zone_allows,
              return_elapsed, line_cfg.reacquire_window_sec, return_progress,
              line_cfg.reacquire_max_return_progress}))
        {
          transition_overtake_line_phase(
            OvertakeLinePhase::Pass, now_sec, current_ey,
            overtake_line_state_.pass_side_sign, "same target reacquired during early return");
        }
      }
    } else if (
      overtake_line_state_.phase == OvertakeLinePhase::ShiftOut ||
      overtake_line_state_.phase == OvertakeLinePhase::Pass) {
      const bool active_execution_latched =
        behavior_output.locked_target_seen &&
        behavior_output.locked_target_longitudinal > 0.0 &&
        !behavior_output.overtake_forbidden_wp &&
        (!behavior_output.overtake_hard_curve_blocked ||
        behavior_output.active_hard_curve_continuation_allowed ||
        behavior_output.outer_curve_hard_continuation_allowed ||
        behavior_output.inner_curve_hard_continuation_allowed) &&
        (!behavior_output.overtake_inner_curve_pass ||
        behavior_output.active_hard_curve_continuation_allowed ||
        behavior_output.inner_curve_entry_allowed ||
        behavior_output.inner_curve_hard_continuation_allowed) &&
        (behavior_output.overtake_completion_feasible ||
        behavior_output.outer_curve_hard_continuation_allowed ||
        behavior_output.inner_curve_entry_allowed ||
        behavior_output.inner_curve_hard_continuation_allowed) &&
        !behavior_output.overtake_cooldown_active &&
        behavior_output.front_risk_level != FrontRiskLevel::EmergencyBrake &&
        (!behavior_output.overtake_forbidden ||
        behavior_output.continuing_overtake_allowed ||
        behavior_output.active_hard_curve_continuation_allowed ||
        behavior_output.outer_curve_hard_continuation_allowed ||
        behavior_output.inner_curve_entry_allowed ||
        behavior_output.inner_curve_hard_continuation_allowed);
      const auto continuity = overtake_core::resolve_target_continuity(
        overtake_core::ContinuityRequest{
          overtake_solver_recovery_active_, behavior_output.locked_target_position_jump,
          rear_clear_observed, rear_clear_confirmed, behavior_output.has_side_vehicle,
          behavior_output.locked_target_seen, target_age, line_cfg.target_hold_sec,
          behavior_output.locked_target_seen &&
          behavior_output.locked_target_longitudinal <= 0.0,
          active_execution_latched});
      if (continuity == overtake_core::ContinuityAction::Return) {
        transition_overtake_line_phase(
          OvertakeLinePhase::Return, now_sec, current_ey, overtake_line_state_.pass_side_sign,
          "locked target rear clearance confirmed");
      } else if (continuity == overtake_core::ContinuityAction::Recovery) {
        const char * recovery_reason = overtake_solver_recovery_active_ ?
          "solver failure threshold reached" :
          behavior_output.locked_target_position_jump ? "locked target position jump" :
          behavior_output.locked_target_seen ?
          "locked target no longer executable" : "locked target stale or lost";
        transition_overtake_line_phase(
          OvertakeLinePhase::Recovery, now_sec, current_ey,
          overtake_line_state_.pass_side_sign, recovery_reason);
      }
    } else if (
      overtake_line_state_.phase != OvertakeLinePhase::Return &&
      overtake_line_state_.phase != OvertakeLinePhase::Recovery) {
      reset_overtake_line_state(now_sec, "not overtaking");
      return output;
    }

    const double phase_elapsed =
      std::isfinite(overtake_line_state_.phase_start_sec) ?
      std::max(0.0, now_sec - overtake_line_state_.phase_start_sec) : 0.0;
    const double phase_delta_sec =
      std::isfinite(overtake_line_state_.phase_last_update_sec) ?
      now_sec - overtake_line_state_.phase_last_update_sec :
      std::numeric_limits<double>::quiet_NaN();
    const auto phase_progress = v2x_overtake_core::integrate_forward_distance(
      v2x_overtake_core::ForwardDistanceRequest{
        overtake_line_state_.phase_traveled_m, current_speed_mps_, phase_delta_sec,
        line_cfg.recovery_max_observation_gap_sec});
    overtake_line_state_.phase_traveled_m = phase_progress.accumulated_distance_m;
    overtake_line_state_.phase_last_update_sec = now_sec;
    const bool phase_hold_elapsed =
      phase_elapsed >= std::max(0.0, line_cfg.phase_hold_time);

    double recovery_stalled_sec = 0.0;
    if (overtake_line_state_.phase == OvertakeLinePhase::Recovery) {
      if (!phase_progress.observation_accepted) {
        overtake_line_state_.recovery_stall_since_sec =
          std::numeric_limits<double>::quiet_NaN();
      } else if (current_speed_mps_ <= line_cfg.recovery_stall_speed) {
        if (!std::isfinite(overtake_line_state_.recovery_stall_since_sec)) {
          overtake_line_state_.recovery_stall_since_sec = now_sec;
        }
      } else {
        overtake_line_state_.recovery_stall_since_sec =
          std::numeric_limits<double>::quiet_NaN();
      }
      if (std::isfinite(overtake_line_state_.recovery_stall_since_sec)) {
        recovery_stalled_sec = std::max(
          0.0, now_sec - overtake_line_state_.recovery_stall_since_sec);
      }
    }

    const double lateral_offset = std::max(0.0, line_cfg.lateral_offset);
    const double raw_goal_for_phase = overtake_line_goal_ey();
    double feasible_goal_for_phase = raw_goal_for_phase;
    if (lb.size() > 0 && ub.size() > 0) {
      double feasible_lower = lb[0] + std::max(0.0, line_cfg.min_wall_clearance);
      double feasible_upper = ub[0] - std::max(0.0, line_cfg.min_wall_clearance);
      if (feasible_upper < feasible_lower) {
        feasible_lower = lb[0];
        feasible_upper = ub[0];
      }
      feasible_goal_for_phase = clip(
        raw_goal_for_phase, feasible_lower, feasible_upper);
    }
    const double shiftout_lateral_tolerance = std::max(0.15, 0.25 * lateral_offset);
    if (
      overtake_line_state_.phase == OvertakeLinePhase::ShiftOut &&
      overtake_core::is_shiftout_complete(
        overtake_core::ShiftOutCompletionRequest{
          phase_hold_elapsed, overtake_line_state_.phase_traveled_m,
          std::max(0.5, line_cfg.shift_distance), current_ey,
          feasible_goal_for_phase, shiftout_lateral_tolerance,
          overtake_line_state_.pass_side_sign}))
    {
      transition_overtake_line_phase(
        OvertakeLinePhase::Pass, now_sec, current_ey, overtake_line_state_.pass_side_sign,
        "shift complete");
    }
    if (
      overtake_line_state_.phase == OvertakeLinePhase::Return && phase_hold_elapsed &&
      (overtake_line_state_.phase_traveled_m >= std::max(0.5, line_cfg.return_distance) ||
      std::abs(current_ey) < 0.15)) {
      reset_overtake_line_state(now_sec, "return complete");
      return output;
    }

    v2x_overtake_core::RecoveryPolicyResolution recovery_policy;
    if (overtake_line_state_.phase == OvertakeLinePhase::Recovery) {
      recovery_policy = v2x_overtake_core::resolve_recovery_policy(
        v2x_overtake_core::RecoveryPolicyRequest{
          line_cfg.recovery_velocity, phase_elapsed, overtake_line_state_.phase_traveled_m,
          std::max(0.5, line_cfg.return_distance), current_ey, 0.20,
          recovery_stalled_sec, line_cfg.recovery_stall_timeout_sec,
          line_cfg.recovery_timeout_sec});
      const bool normal_completion_before_hold =
        !phase_hold_elapsed &&
        (recovery_policy.exit_reason ==
        v2x_overtake_core::RecoveryExitReason::DistanceComplete ||
        recovery_policy.exit_reason ==
        v2x_overtake_core::RecoveryExitReason::LateralComplete);
      if (normal_completion_before_hold) {
        recovery_policy.exit_reason = v2x_overtake_core::RecoveryExitReason::Active;
      }
      if (
        recovery_policy.exit_reason != v2x_overtake_core::RecoveryExitReason::Active)
      {
        const std::string reason = std::string("recovery ") +
          v2x_overtake_core::to_string(recovery_policy.exit_reason);
        reset_overtake_line_state(now_sec, reason);
        return output;
      }
    }

    if (overtake_line_state_.phase == OvertakeLinePhase::Idle) {
      return output;
    }

    const double raw_goal = overtake_line_goal_ey();
    const bool lateral_complete =
      overtake_core::has_reached_pass_side_lateral_goal(
        current_ey, feasible_goal_for_phase, shiftout_lateral_tolerance,
        overtake_line_state_.pass_side_sign);
    const bool locked_target_seen = behavior_output.locked_target_seen;
    const double locked_target_longitudinal = locked_target_seen ?
      behavior_output.locked_target_longitudinal :
      overtake_line_state_.target_last_longitudinal;
    const double locked_target_speed =
      locked_target_seen && std::isfinite(behavior_output.locked_target_speed) ?
      behavior_output.locked_target_speed : overtake_line_state_.target_last_speed;
    output.front_cap_release_ready =
      overtake_core::can_release_overtake_front_cap(
      overtake_core::OvertakeFrontCapReleaseRequest{
        overtake_line_state_.phase == OvertakeLinePhase::Pass,
        lateral_complete, locked_target_seen, locked_target_longitudinal});
    if (
      (overtake_line_state_.phase == OvertakeLinePhase::ShiftOut ||
      overtake_line_state_.phase == OvertakeLinePhase::Pass) &&
      !output.front_cap_release_ready &&
      std::isfinite(locked_target_speed))
    {
      double closing_speed_limit =
        std::max(0.0, cfg.v2x_behavior.overtake_shiftout_max_closing_speed);
      closing_speed_limit = overtake_core::resolve_unlatched_pass_closing_speed(
        closing_speed_limit,
        cfg.v2x_behavior.overtake_pass_unlatched_max_closing_speed,
        overtake_line_state_.phase == OvertakeLinePhase::Pass,
        overtake_line_state_.pass_front_overlap_exclusion_latched);
      if (
        overtake_line_state_.phase == OvertakeLinePhase::ShiftOut &&
        cfg.v2x_behavior.overtake_shiftout_adaptive_closing_speed_enabled &&
        std::isfinite(locked_target_longitudinal))
      {
        const auto adaptive_closing =
          overtake_core::resolve_adaptive_shiftout_closing_speed(
          overtake_core::AdaptiveShiftOutClosingSpeedRequest{
            cfg.v2x_behavior.overtake_shiftout_min_closing_speed,
            cfg.v2x_behavior.overtake_shiftout_max_closing_speed,
            std::max(0.0, locked_target_longitudinal),
            cfg.v2x_behavior.overtake_guard_min_front_distance,
            overtake_shiftout_remaining_distance(current_ey),
            std::max(0.0, current_speed_mps_),
            std::max(kEps, cfg.v2x_behavior.overtake_guard_min_speed_for_reachable),
            cfg.v2x_behavior.overtake_shiftout_adaptive_min_time_sec});
        closing_speed_limit = adaptive_closing.closing_speed_mps;
      }
      output.closing_speed_limit = closing_speed_limit;
      // This is an overtake progress reference, not a hard state/input bound.  Applying an
      // abruptly reduced hard bound together with the new lateral ShiftOut corridor can make
      // an otherwise reachable MPC problem numerically infeasible.  Recovery below keeps its
      // dedicated hard limit; active ShiftOut/early Pass only shapes the velocity reference.
      output.target_velocity_reference = std::min(
        cfg.v_max, std::max(0.0, locked_target_speed) + closing_speed_limit);
    }
    const double goal_ey = limit_overtake_line_goal_change(raw_goal);
    const double phase_distance = overtake_line_phase_distance();
    const double min_wall_clearance = std::max(0.0, line_cfg.min_wall_clearance);
    const double max_lateral_accel = std::max(0.0, line_cfg.max_lateral_accel);
    const double speed_for_time = std::max(1.0, current_speed_mps_);

    output.active = true;
    output.target_ey.assign(N, 0.0);
    output.target_active.assign(N, true);
    if (
      overtake_line_state_.phase == OvertakeLinePhase::Recovery &&
      (line_cfg.recovery_velocity_limit_enabled || overtake_solver_recovery_active_))
    {
      output.target_velocity_limit = recovery_policy.velocity_limit_mps;
    }

    for (int i = 0; i < N; ++i) {
      const double distance = horizon_path_distance_to_index(ref_wp_id, static_cast<std::size_t>(i));
      const double progress =
        overtake_core::resolve_overtake_line_horizon_progress(
        overtake_core::OvertakeLineHorizonProgressRequest{
          overtake_line_state_.phase == OvertakeLinePhase::Pass,
          overtake_line_state_.phase_traveled_m, distance, phase_distance});
      double target_ey =
        overtake_line_state_.phase_start_ey +
        progress * (goal_ey - overtake_line_state_.phase_start_ey);

      double lower = lb[i] + min_wall_clearance;
      double upper = ub[i] - min_wall_clearance;
      if (upper < lower) {
        lower = lb[i];
        upper = ub[i];
        output.wall_clearance_limited = true;
      }

      const double unclipped_target = target_ey;
      target_ey = clip(target_ey, lower, upper);
      if (std::abs(target_ey - unclipped_target) > 1e-6) {
        output.wall_clearance_limited = true;
      }

      const double time_to_target = std::max(0.15, distance / speed_for_time);
      const double lateral_shift = std::abs(target_ey - current_ey);
      double required_lateral_accel = 2.0 * lateral_shift / (time_to_target * time_to_target);
      if (max_lateral_accel > kEps && required_lateral_accel > max_lateral_accel) {
        const double direction = target_ey >= current_ey ? 1.0 : -1.0;
        const double limited_shift = 0.5 * max_lateral_accel * time_to_target * time_to_target;
        target_ey = current_ey + direction * limited_shift;
        target_ey = clip(target_ey, lower, upper);
        required_lateral_accel = max_lateral_accel;
        output.lateral_accel_limited = true;
      }
      output.max_required_lateral_accel =
        std::max(output.max_required_lateral_accel, required_lateral_accel);
      output.target_ey[i] = target_ey;
    }

    if (line_cfg.debug_log_enabled) {
      const double period = std::max(0.0, cfg.v2x_behavior.debug_log_period_sec);
      const bool log_now =
        !std::isfinite(last_overtake_line_debug_log_sec_) ||
        period <= kEps ||
        now_sec - last_overtake_line_debug_log_sec_ >= period;
      if (log_now) {
        RCLCPP_INFO(
          rclcpp::get_logger("mpc_controller"),
          "OvertakeLine debug: phase=%s, side=%d, goal=%.2f, first_target=%.2f, "
          "current_ey=%.2f, elapsed=%.2f, traveled=%.2f, stalled=%.2f, "
          "v_ref=%.2f, v_limit=%.2f, closing=%.2f, cap_release=%d, cooldown=%.2f, "
          "max_lat_acc=%.2f, lat_limited=%d, wall_limited=%d",
          to_string(overtake_line_state_.phase), overtake_line_state_.pass_side_sign, goal_ey,
          output.target_ey.empty() ? 0.0 : output.target_ey.front(), current_ey,
          phase_elapsed, overtake_line_state_.phase_traveled_m, recovery_stalled_sec,
          output.target_velocity_reference, output.target_velocity_limit,
          output.closing_speed_limit,
          output.front_cap_release_ready ? 1 : 0,
          std::max(0.0, overtake_solver_cooldown_until_sec_ - now_sec),
          output.max_required_lateral_accel, output.lateral_accel_limited ? 1 : 0,
          output.wall_clearance_limited ? 1 : 0);
        last_overtake_line_debug_log_sec_ = now_sec;
      }
    }

    return output;
  }

  bool is_overtake_forbidden_wp(const int wp_id) const
  {
    for (const auto & range : cfg.v2x_behavior.overtake_forbidden_wp_ranges) {
      const int start = range.first;
      const int end = range.second;
      if (start <= end) {
        if (start <= wp_id && wp_id <= end) {
          return true;
        }
      } else if (wp_id >= start || wp_id <= end) {
        return true;
      }
    }
    return false;
  }

  bool is_curvature_above_threshold(
    const int ref_wp_id, const int N, const double configured_lookahead_distance,
    const double configured_threshold) const
  {
    const double threshold = std::max(0.0, configured_threshold);
    if (threshold <= kEps) {
      return false;
    }
    double max_abs_kappa = 0.0;
    int lookahead_count = N;
    const double lookahead_distance = std::max(0.0, configured_lookahead_distance);
    if (lookahead_distance > kEps) {
      double distance = 0.0;
      lookahead_count = 1;
      const int max_steps = std::max(
        N, static_cast<int>(
          std::ceil(lookahead_distance / std::max(0.1, model->reference_path->resolution))) + 2);
      for (int i = 0; i < max_steps; ++i) {
        const auto & current = model->reference_path->get_waypoint(ref_wp_id + i);
        const auto & next = model->reference_path->get_waypoint(ref_wp_id + i + 1);
        distance += next.distance_to(current);
        lookahead_count = i + 1;
        if (distance >= lookahead_distance) {
          break;
        }
      }
    }
    for (int i = 0; i < lookahead_count; ++i) {
      const auto & waypoint = model->reference_path->get_waypoint(ref_wp_id + i);
      max_abs_kappa = std::max(max_abs_kappa, std::abs(waypoint.kappa));
    }
    return max_abs_kappa > threshold;
  }

  bool is_overtake_forbidden_curvature(
    const int ref_wp_id, const int N, const double configured_lookahead_distance) const
  {
    return is_curvature_above_threshold(
      ref_wp_id, N, configured_lookahead_distance,
      cfg.v2x_behavior.overtake_max_curvature);
  }

  double distance_to_hard_overtake_boundary(
    const int ref_wp_id, const double hard_curvature,
    const double maximum_search_distance) const
  {
    const double threshold = std::max(0.0, hard_curvature);
    const double search_distance = std::max(0.0, maximum_search_distance);
    if (threshold <= kEps || search_distance <= kEps) {
      return std::numeric_limits<double>::infinity();
    }

    const int waypoint_count = model->reference_path->n_waypoints;
    const int max_steps = model->reference_path->circular ?
      waypoint_count : std::max(0, waypoint_count - ref_wp_id);
    double distance = 0.0;
    for (int offset = 0; offset < max_steps; ++offset) {
      const int wp_id = ref_wp_id + offset;
      const int normalized_wp_id = model->reference_path->circular ?
        ((wp_id % waypoint_count) + waypoint_count) % waypoint_count : wp_id;
      const auto & waypoint = model->reference_path->get_waypoint(wp_id);
      if (
        is_overtake_forbidden_wp(normalized_wp_id) ||
        std::abs(waypoint.kappa) > threshold)
      {
        return distance;
      }
      if (offset + 1 >= max_steps) {
        break;
      }
      const auto & next = model->reference_path->get_waypoint(wp_id + 1);
      distance += next.distance_to(waypoint);
      if (distance > search_distance) {
        return std::numeric_limits<double>::infinity();
      }
    }
    return std::numeric_limits<double>::infinity();
  }

  int curve_inner_pass_side(const int ref_wp_id, const int N) const
  {
    if (!cfg.v2x_behavior.overtake_block_inner_curve_pass) {
      return 0;
    }
    const double threshold = std::max(0.0, cfg.v2x_behavior.overtake_max_curvature);
    if (threshold <= kEps) {
      return 0;
    }

    int lookahead_count = N;
    const double lookahead_distance =
      std::max(0.0, cfg.v2x_behavior.overtake_forbidden_curve_lookahead_distance);
    if (lookahead_distance > kEps) {
      double distance = 0.0;
      lookahead_count = 1;
      const int max_steps = std::max(
        N, static_cast<int>(
          std::ceil(lookahead_distance / std::max(0.1, model->reference_path->resolution))) + 2);
      for (int i = 0; i < max_steps; ++i) {
        const auto & current = model->reference_path->get_waypoint(ref_wp_id + i);
        const auto & next = model->reference_path->get_waypoint(ref_wp_id + i + 1);
        distance += next.distance_to(current);
        lookahead_count = i + 1;
        if (distance >= lookahead_distance) {
          break;
        }
      }
    }

    double max_abs_kappa = 0.0;
    double signed_kappa = 0.0;
    for (int i = 0; i < lookahead_count; ++i) {
      const auto & waypoint = model->reference_path->get_waypoint(ref_wp_id + i);
      const double abs_kappa = std::abs(waypoint.kappa);
      if (abs_kappa > max_abs_kappa) {
        max_abs_kappa = abs_kappa;
        signed_kappa = waypoint.kappa;
      }
    }
    if (max_abs_kappa <= threshold || std::abs(signed_kappa) <= kEps) {
      return 0;
    }
    return signed_kappa > 0.0 ? 1 : -1;
  }

  bool is_inner_curve_pass(const int pass_side_sign, const int inner_curve_pass_side) const
  {
    return cfg.v2x_behavior.overtake_block_inner_curve_pass &&
           pass_side_sign != 0 &&
           inner_curve_pass_side != 0 &&
           pass_side_sign == inner_curve_pass_side;
  }

  bool has_sufficient_gap(const GapPlannerOutput & gap_output, const double min_width) const
  {
    if (!gap_output.active || !gap_output.feasible) {
      return false;
    }
    const double required_width = std::max(0.0, min_width);
    for (std::size_t i = 0; i < gap_output.target_active.size(); ++i) {
      if (!gap_output.target_active[i]) {
        continue;
      }
      if (
        i < gap_output.lb.size() && i < gap_output.ub.size() &&
        gap_output.ub[i] - gap_output.lb[i] >= required_width) {
        return true;
      }
    }
    return false;
  }

  int v2x_overtake_gap_plan_horizon(const int N) const
  {
    const double lookahead_distance =
      std::max(0.0, cfg.v2x_behavior.overtake_gap_lookahead_distance);
    if (lookahead_distance <= kEps) {
      return N;
    }
    const int lookahead_steps = static_cast<int>(
      std::ceil(lookahead_distance / std::max(0.1, model->reference_path->resolution))) + 2;
    return std::max(N, lookahead_steps);
  }

  std::pair<Eigen::VectorXd, Eigen::VectorXd> build_v2x_gap_planner_bounds(
    const int ref_wp_id, const int base_N, const Eigen::VectorXd & base_lb,
    const Eigen::VectorXd & base_ub, const int plan_N) const
  {
    Eigen::VectorXd plan_lb(plan_N);
    Eigen::VectorXd plan_ub(plan_N);
    for (int i = 0; i < plan_N; ++i) {
      if (i < base_N) {
        plan_lb[i] = base_lb[i];
        plan_ub[i] = base_ub[i];
        continue;
      }

      const auto & waypoint = model->reference_path->get_waypoint(ref_wp_id + i);
      double lb = waypoint.lb_sm;
      double ub = waypoint.ub_sm;
      if (ub - lb <= kEps) {
        lb = waypoint.lb;
        ub = waypoint.ub;
      }
      if (ub < lb) {
        lb = 0.0;
        ub = 0.0;
      }
      plan_lb[i] = lb;
      plan_ub[i] = ub;
    }
    return {plan_lb, plan_ub};
  }

  bool has_consecutive_sufficient_gap(
    const GapPlannerOutput & gap_output, const double min_width, const int min_points) const
  {
    if (min_points <= 1) {
      return has_sufficient_gap(gap_output, min_width);
    }
    if (!gap_output.active || !gap_output.feasible) {
      return false;
    }
    const double required_width = std::max(0.0, min_width);
    int consecutive = 0;
    for (std::size_t i = 0; i < gap_output.target_active.size(); ++i) {
      const bool gap_ok =
        gap_output.target_active[i] && i < gap_output.lb.size() && i < gap_output.ub.size() &&
        gap_output.ub[i] - gap_output.lb[i] >= required_width;
      if (!gap_ok) {
        consecutive = 0;
        continue;
      }
      ++consecutive;
      if (consecutive >= min_points) {
        return true;
      }
    }
    return false;
  }

  bool has_sufficient_overtake_gap(const GapPlannerOutput & gap_output) const
  {
    return has_sufficient_gap(gap_output, cfg.v2x_behavior.overtake_min_gap_width);
  }

  double horizon_path_distance_to_index(const int ref_wp_id, const std::size_t target_index) const
  {
    double distance = 0.0;
    for (std::size_t i = 0; i <= target_index; ++i) {
      const auto & p0 = model->reference_path->get_waypoint(ref_wp_id + static_cast<int>(i));
      const auto & p1 = model->reference_path->get_waypoint(ref_wp_id + static_cast<int>(i) + 1);
      distance += p0.distance_to(p1);
    }
    return distance;
  }

  ReachableGapMetrics compute_reachable_gap_metrics(
    const GapPlannerOutput & gap_output, const int ref_wp_id, const double current_ey,
    const double required_width, const int min_points) const
  {
    ReachableGapMetrics metrics;
    if (!gap_output.active || !gap_output.feasible) {
      return metrics;
    }

    std::size_t first_sequence_start = 0;
    std::size_t current_sequence_start = 0;
    int consecutive = 0;
    bool found_sequence = false;
    for (std::size_t i = 0; i < gap_output.target_active.size(); ++i) {
      const bool gap_ok =
        gap_output.target_active[i] && i < gap_output.lb.size() && i < gap_output.ub.size() &&
        gap_output.ub[i] - gap_output.lb[i] >= required_width;
      if (!gap_ok) {
        consecutive = 0;
        continue;
      }
      if (consecutive == 0) {
        current_sequence_start = i;
      }
      ++consecutive;
      if (consecutive >= std::max(1, min_points)) {
        first_sequence_start = current_sequence_start;
        found_sequence = true;
        break;
      }
    }
    if (!found_sequence) {
      return metrics;
    }

    const double speed_for_time =
      std::max(std::max(0.0, current_speed_mps_),
        std::max(kEps, cfg.v2x_behavior.overtake_guard_min_speed_for_reachable));
    for (std::size_t i = first_sequence_start; i < gap_output.target_active.size(); ++i) {
      if (!gap_output.target_active[i] || i >= gap_output.lb.size() || i >= gap_output.ub.size()) {
        continue;
      }
      if (gap_output.ub[i] - gap_output.lb[i] < required_width) {
        continue;
      }

      const double distance = horizon_path_distance_to_index(ref_wp_id, i);
      const double target_ey = i < gap_output.target_ey.size() ?
        gap_output.target_ey[i] : 0.5 * (gap_output.lb[i] + gap_output.ub[i]);
      const double lateral_shift = std::abs(target_ey - current_ey);
      const double gap_time = distance / speed_for_time;
      const double required_lateral_accel = gap_time > kEps ?
        2.0 * lateral_shift / (gap_time * gap_time) :
        std::numeric_limits<double>::infinity();

      if (!metrics.valid) {
        metrics.valid = true;
        metrics.first_gap_distance = distance;
        metrics.first_gap_time = gap_time;
        metrics.first_lateral_shift = lateral_shift;
      }
      metrics.max_lateral_shift = std::max(metrics.max_lateral_shift, lateral_shift);
      if (required_lateral_accel > metrics.max_required_lateral_accel) {
        metrics.max_required_lateral_accel = required_lateral_accel;
        metrics.max_required_lateral_accel_index = i;
      }
    }
    return metrics;
  }

  bool overtake_guard_allows(
    const GapPlannerOutput & gap_output, const int ref_wp_id, const double front_distance,
    const double current_ey, const bool continuing_overtake,
    std::string & block_reason) const
  {
    const auto guard_phase = v2x_overtake_core::resolve_overtake_guard_phase(
      v2x_overtake_core::OvertakeGuardPhaseRequest{
        continuing_overtake,
        cfg.v2x_behavior.overtake_guard_min_front_distance,
        cfg.v2x_behavior.overtake_continue_min_front_distance});
    const double min_front_distance = guard_phase.min_front_distance_m;
    if (min_front_distance > kEps && front_distance < min_front_distance) {
      std::ostringstream ss;
      ss << "overtake guard front distance"
         << ", phase=" << (continuing_overtake ? "continue" : "entry")
         << ", fd=" << front_distance
         << ", min=" << min_front_distance;
      block_reason = ss.str();
      return false;
    }

    const double required_width = std::max(
      cfg.v2x_behavior.overtake_min_gap_width,
      cfg.v2x_behavior.overtake_guard_min_gap_width);
    const int min_points = std::max(1, cfg.v2x_behavior.overtake_guard_min_gap_points);
    if (!has_consecutive_sufficient_gap(gap_output, required_width, min_points)) {
      double max_width = 0.0;
      int best_consecutive = 0;
      int consecutive = 0;
      for (std::size_t i = 0; i < gap_output.target_active.size(); ++i) {
        const bool valid =
          gap_output.target_active[i] && i < gap_output.lb.size() && i < gap_output.ub.size();
        const double width = valid ? std::max(0.0, gap_output.ub[i] - gap_output.lb[i]) : 0.0;
        max_width = std::max(max_width, width);
        if (valid && width >= required_width) {
          ++consecutive;
          best_consecutive = std::max(best_consecutive, consecutive);
        } else {
          consecutive = 0;
        }
      }
      std::ostringstream oss;
      oss << "overtake guard gap width"
          << ", max=" << max_width
          << ", req=" << required_width
          << ", run=" << best_consecutive
          << ", min_points=" << min_points;
      block_reason = oss.str();
      return false;
    }

    const ReachableGapMetrics reachable_gap =
      compute_reachable_gap_metrics(gap_output, ref_wp_id, current_ey, required_width, min_points);
    if (!reachable_gap.valid) {
      block_reason = "overtake guard reachable gap missing";
      return false;
    }

    const double min_prepare_distance =
      std::max(0.0, cfg.v2x_behavior.overtake_guard_min_prepare_distance);
    if (
      guard_phase.require_prepare_distance && min_prepare_distance > kEps &&
      reachable_gap.first_gap_distance < min_prepare_distance)
    {
      block_reason = "overtake guard prepare distance";
      return false;
    }

    const double max_allowed_shift =
      std::max(0.0, cfg.v2x_behavior.overtake_guard_max_lateral_shift);
    if (max_allowed_shift > kEps && reachable_gap.max_lateral_shift > max_allowed_shift) {
      block_reason = "overtake guard lateral shift";
      return false;
    }

    if (cfg.v2x_behavior.overtake_guard_reachable_gap_enabled) {
      const double min_gap_time =
        std::max(0.0, cfg.v2x_behavior.overtake_guard_min_gap_time);
      if (min_gap_time > kEps && reachable_gap.first_gap_time < min_gap_time) {
        std::ostringstream ss;
        ss << "overtake guard gap time, t=" << reachable_gap.first_gap_time
           << ", min=" << min_gap_time;
        block_reason = ss.str();
        return false;
      }

      const double max_lateral_accel =
        std::max(0.0, cfg.v2x_behavior.overtake_guard_max_lateral_accel);
      if (
        max_lateral_accel > kEps &&
        reachable_gap.max_required_lateral_accel > max_lateral_accel) {
        std::ostringstream ss;
        ss << "overtake guard lateral accel, ay="
           << reachable_gap.max_required_lateral_accel
           << ", max=" << max_lateral_accel
           << ", i=" << reachable_gap.max_required_lateral_accel_index;
        block_reason = ss.str();
        return false;
      }
    }

    std::ostringstream ss;
    ss << "front vehicle and guarded gap available, first_s="
       << reachable_gap.first_gap_distance
       << ", gap_t=" << reachable_gap.first_gap_time
       << ", ay=" << reachable_gap.max_required_lateral_accel;
    block_reason = ss.str();
    return true;
  }

  bool overtake_close_follow_allows(
    const V2XBehaviorOutput & output, const double front_distance, const double front_speed,
    const double current_ey, const double lower, const double upper, std::string & block_reason) const
  {
    if (!cfg.v2x_behavior.overtake_close_follow_enabled) {
      block_reason = "overtake close-follow disabled";
      return false;
    }
    if (output.overtake_pass_side_sign == 0) {
      block_reason = "overtake close-follow invalid side";
      return false;
    }
    if (upper - lower <= kEps) {
      block_reason = "overtake close-follow invalid bounds";
      return false;
    }
    if (!std::isfinite(front_distance)) {
      block_reason = "overtake close-follow front distance";
      return false;
    }
    if (!std::isfinite(front_speed)) {
      block_reason = "overtake close-follow front speed";
      return false;
    }

    const double min_front_distance =
      std::max(0.0, cfg.v2x_behavior.overtake_close_follow_min_front_distance);
    if (front_distance < min_front_distance) {
      std::ostringstream ss;
      ss << "overtake close-follow front distance"
         << ", fd=" << front_distance
         << ", min=" << min_front_distance;
      block_reason = ss.str();
      return false;
    }

    const double min_side_clearance = std::max(
      cfg.v2x_behavior.overtake_fallback_min_side_clearance,
      cfg.v2x_behavior.overtake_close_follow_min_side_clearance);
    if (output.overtake_side_clearance < min_side_clearance) {
      std::ostringstream ss;
      ss << "overtake close-follow side clearance"
         << ", clearance=" << output.overtake_side_clearance
         << ", min=" << min_side_clearance;
      block_reason = ss.str();
      return false;
    }

    const double closing_speed = std::max(0.0, current_speed_mps_ - std::max(0.0, front_speed));
    const double max_closing_speed =
      std::max(0.0, cfg.v2x_behavior.overtake_close_follow_max_closing_speed);
    if (closing_speed > max_closing_speed) {
      std::ostringstream ss;
      ss << "overtake close-follow closing speed"
         << ", closing=" << closing_speed
         << ", max=" << max_closing_speed;
      block_reason = ss.str();
      return false;
    }

    const double raw_target =
      overtake_side_target(lower, upper, output.overtake_pass_side_sign);
    const double directional_target = output.overtake_pass_side_sign > 0 ?
      std::max(raw_target, current_ey) :
      std::min(raw_target, current_ey);
    const double lateral_shift = std::abs(directional_target - current_ey);
    const double speed_for_time =
      std::max(std::max(0.0, current_speed_mps_),
        std::max(kEps, cfg.v2x_behavior.overtake_guard_min_speed_for_reachable));
    const double gap_time = front_distance / speed_for_time;
    const double min_gap_time = std::max(0.0, cfg.v2x_behavior.overtake_guard_min_gap_time);
    if (min_gap_time > kEps && gap_time < min_gap_time) {
      std::ostringstream ss;
      ss << "overtake close-follow gap time"
         << ", t=" << gap_time
         << ", min=" << min_gap_time;
      block_reason = ss.str();
      return false;
    }

    if (cfg.v2x_behavior.overtake_guard_reachable_gap_enabled) {
      const double max_lateral_accel =
        std::max(0.0, cfg.v2x_behavior.overtake_guard_max_lateral_accel);
      const double required_lateral_accel = gap_time > kEps ?
        2.0 * lateral_shift / (gap_time * gap_time) :
        std::numeric_limits<double>::infinity();
      if (max_lateral_accel > kEps && required_lateral_accel > max_lateral_accel) {
        std::ostringstream ss;
        ss << "overtake close-follow lateral accel"
           << ", ay=" << required_lateral_accel
           << ", max=" << max_lateral_accel
           << ", shift=" << lateral_shift
           << ", t=" << gap_time;
        block_reason = ss.str();
        return false;
      }
    }

    std::ostringstream ss;
    ss << "overtake close-follow side target"
       << ", side=" << (output.overtake_pass_side_sign > 0 ? "left" : "right")
       << ", clearance=" << output.overtake_side_clearance
       << ", fd=" << front_distance
       << ", closing=" << closing_speed
       << ", shift=" << lateral_shift
       << ", t=" << gap_time;
    block_reason = ss.str();
    return true;
  }

  bool overtake_fallback_guard_allows(
    const int pass_side_sign, const double front_distance, const double current_ey,
    const double lower, const double upper, const bool continuing_overtake,
    std::string & block_reason) const
  {
    if (pass_side_sign == 0 || upper - lower <= kEps) {
      block_reason = "overtake fallback guard invalid side";
      return false;
    }
    if (!std::isfinite(front_distance)) {
      block_reason = "overtake fallback guard front distance";
      return false;
    }

    const auto guard_phase = v2x_overtake_core::resolve_overtake_guard_phase(
      v2x_overtake_core::OvertakeGuardPhaseRequest{
        continuing_overtake,
        cfg.v2x_behavior.overtake_guard_min_front_distance,
        cfg.v2x_behavior.overtake_continue_min_front_distance});
    const double min_front_distance = guard_phase.min_front_distance_m;
    const double min_prepare_distance =
      guard_phase.require_prepare_distance ?
      std::max(0.0, cfg.v2x_behavior.overtake_guard_min_prepare_distance) : 0.0;
    const double required_front_distance = std::max(min_front_distance, min_prepare_distance);
    if (required_front_distance > kEps && front_distance < required_front_distance) {
      std::ostringstream ss;
      ss << "overtake fallback guard front distance"
         << ", phase=" << (continuing_overtake ? "continue" : "entry")
         << ", fd=" << front_distance
         << ", min=" << required_front_distance;
      block_reason = ss.str();
      return false;
    }

    const double raw_target = overtake_side_target(lower, upper, pass_side_sign);
    const double directional_target = pass_side_sign > 0 ?
      std::max(raw_target, current_ey) :
      std::min(raw_target, current_ey);
    const double lateral_shift = std::abs(directional_target - current_ey);
    const double max_allowed_shift =
      std::max(0.0, cfg.v2x_behavior.overtake_guard_max_lateral_shift);
    if (max_allowed_shift > kEps && lateral_shift > max_allowed_shift) {
      std::ostringstream ss;
      ss << "overtake fallback guard lateral shift"
         << ", shift=" << lateral_shift
         << ", max=" << max_allowed_shift;
      block_reason = ss.str();
      return false;
    }

    const double speed_for_time =
      std::max(std::max(0.0, current_speed_mps_),
        std::max(kEps, cfg.v2x_behavior.overtake_guard_min_speed_for_reachable));
    const double gap_time = front_distance / speed_for_time;
    const double min_gap_time = std::max(0.0, cfg.v2x_behavior.overtake_guard_min_gap_time);
    if (min_gap_time > kEps && gap_time < min_gap_time) {
      std::ostringstream ss;
      ss << "overtake fallback guard gap time"
         << ", t=" << gap_time
         << ", min=" << min_gap_time;
      block_reason = ss.str();
      return false;
    }

    if (cfg.v2x_behavior.overtake_guard_reachable_gap_enabled) {
      const double max_lateral_accel =
        std::max(0.0, cfg.v2x_behavior.overtake_guard_max_lateral_accel);
      const double required_lateral_accel = gap_time > kEps ?
        2.0 * lateral_shift / (gap_time * gap_time) :
        std::numeric_limits<double>::infinity();
      if (max_lateral_accel > kEps && required_lateral_accel > max_lateral_accel) {
        std::ostringstream ss;
        ss << "overtake fallback guard lateral accel"
           << ", ay=" << required_lateral_accel
           << ", max=" << max_lateral_accel;
        block_reason = ss.str();
        return false;
      }
    }

    std::ostringstream ss;
    ss << "ok"
       << ", fd=" << front_distance
       << ", shift=" << lateral_shift
       << ", t=" << gap_time;
    block_reason = ss.str();
    return true;
  }

  double front_distance_velocity_limit(const double front_distance) const
  {
    if (!std::isfinite(front_distance)) {
      return std::numeric_limits<double>::infinity();
    }
    const double brake_decel = std::max(kEps, std::abs(cfg.a_min));
    const double margin = std::max(0.0, cfg.v2x_behavior.safety_brake_margin);
    const double available_distance = front_distance - margin;
    if (available_distance <= 0.0) {
      return 0.0;
    }
    return std::sqrt(2.0 * brake_decel * available_distance);
  }

  FrontRiskMetrics compute_front_risk(
    const double front_distance, const double front_speed) const
  {
    FrontRiskMetrics metrics;
    if (
      !cfg.v2x_behavior.front_risk_arbitration_enabled ||
      !std::isfinite(front_distance) ||
      !std::isfinite(front_speed)) {
      return metrics;
    }

    metrics.valid = true;
    metrics.front_distance = front_distance;
    metrics.front_speed = std::max(0.0, front_speed);
    metrics.ego_speed = current_speed_mps_;
    metrics.relative_speed = current_speed_mps_ - metrics.front_speed;
    const double delay_distance =
      std::max(0.0, metrics.relative_speed) *
      std::max(0.0, cfg.v2x_behavior.front_risk_prepare_time);
    metrics.available_distance =
      front_distance - std::max(0.0, cfg.v2x_behavior.front_risk_distance_margin) -
      delay_distance;
    if (metrics.relative_speed > kEps) {
      metrics.ttc = front_distance / metrics.relative_speed;
    }
    if (
      metrics.relative_speed > std::max(0.0, cfg.v2x_behavior.front_risk_min_closing_speed)) {
      if (metrics.available_distance <= kEps) {
        metrics.required_decel = std::numeric_limits<double>::infinity();
      } else {
        metrics.required_decel =
          metrics.relative_speed * metrics.relative_speed / (2.0 * metrics.available_distance);
      }
    }
    return metrics;
  }

  FrontRiskLevel classify_front_risk(const FrontRiskMetrics & metrics) const
  {
    if (!metrics.valid) {
      return FrontRiskLevel::Clear;
    }
    if (
      metrics.relative_speed <= std::max(0.0, cfg.v2x_behavior.front_risk_min_closing_speed)) {
      return FrontRiskLevel::Follow;
    }
    if (metrics.available_distance <= kEps) {
      return FrontRiskLevel::EmergencyBrake;
    }
    if (metrics.required_decel >= cfg.v2x_behavior.front_risk_emergency_decel) {
      return FrontRiskLevel::EmergencyBrake;
    }
    if (metrics.required_decel >= cfg.v2x_behavior.front_risk_hard_decel) {
      return FrontRiskLevel::AvoidCandidate;
    }
    if (metrics.required_decel >= cfg.v2x_behavior.front_risk_comfort_decel) {
      return FrontRiskLevel::BrakePrepare;
    }
    return FrontRiskLevel::Follow;
  }

  double front_risk_velocity_limit(
    const FrontRiskMetrics & metrics, const FrontRiskLevel level) const
  {
    if (!metrics.valid || level == FrontRiskLevel::Clear || level == FrontRiskLevel::Follow) {
      return std::numeric_limits<double>::infinity();
    }
    if (
      level == FrontRiskLevel::BrakePrepare &&
      !cfg.v2x_behavior.front_risk_brake_prepare_limit_enabled) {
      return std::numeric_limits<double>::infinity();
    }
    if (
      level == FrontRiskLevel::AvoidCandidate &&
      !cfg.v2x_behavior.front_risk_avoid_candidate_limit_enabled) {
      return std::numeric_limits<double>::infinity();
    }
    if (level == FrontRiskLevel::EmergencyBrake || metrics.available_distance <= kEps) {
      return std::max(0.0, cfg.v2x_behavior.safety_brake_velocity);
    }

    const double allowed_decel = level == FrontRiskLevel::AvoidCandidate ?
      std::max(kEps, cfg.v2x_behavior.front_risk_hard_decel) :
      std::max(kEps, cfg.v2x_behavior.front_risk_comfort_decel);
    const double safe_relative_speed =
      std::sqrt(std::max(0.0, 2.0 * allowed_decel * metrics.available_distance));
    return std::min(cfg.v_max, metrics.front_speed + safe_relative_speed);
  }

  double front_risk_curve_velocity_limit(const FrontRiskMetrics & metrics) const
  {
    if (
      !cfg.v2x_behavior.front_risk_curve_limit_enabled ||
      !metrics.valid ||
      metrics.available_distance <= kEps ||
      metrics.relative_speed <= std::max(0.0, cfg.v2x_behavior.front_risk_min_closing_speed) ||
      metrics.required_decel <
      std::max(0.0, cfg.v2x_behavior.front_risk_curve_limit_required_decel)) {
      return std::numeric_limits<double>::infinity();
    }

    const double allowed_decel =
      std::max(kEps, cfg.v2x_behavior.front_risk_curve_limit_decel);
    const double speed_margin =
      std::max(0.0, cfg.v2x_behavior.front_risk_curve_limit_speed_margin);
    const double safe_relative_speed =
      std::sqrt(std::max(0.0, 2.0 * allowed_decel * metrics.available_distance));
    return std::min(cfg.v_max, metrics.front_speed + speed_margin + safe_relative_speed);
  }

  std::string front_risk_reason(
    const char * prefix, const FrontRiskMetrics & metrics, const FrontRiskLevel level) const
  {
    std::ostringstream ss;
    ss << prefix << ", risk=" << to_string(level)
       << ", required_decel=" << metrics.required_decel
       << ", rel_speed=" << metrics.relative_speed
       << ", available_distance=" << metrics.available_distance;
    return ss.str();
  }

  double front_decel_guard_velocity_limit(
    const double front_distance, const double front_speed, const bool curve_guard) const
  {
    if (
      !cfg.v2x_behavior.front_decel_guard_enabled ||
      !std::isfinite(front_distance) ||
      !std::isfinite(front_speed)) {
      return std::numeric_limits<double>::infinity();
    }
    if (
      front_speed <= cfg.v2x_behavior.moving_front_speed_threshold &&
      (!curve_guard || !cfg.v2x_behavior.front_decel_guard_curve_include_slow_front)) {
      return std::numeric_limits<double>::infinity();
    }

    double guard_distance = std::max(0.0, cfg.v2x_behavior.front_decel_guard_distance);
    double guard_ttc = std::max(0.0, cfg.v2x_behavior.front_decel_guard_ttc);
    if (curve_guard) {
      guard_distance =
        std::max(guard_distance, std::max(0.0, cfg.v2x_behavior.front_decel_guard_curve_distance));
      guard_ttc = std::max(guard_ttc, std::max(0.0, cfg.v2x_behavior.front_decel_guard_curve_ttc));
    }
    const double speed_margin = std::max(0.0, cfg.v2x_behavior.front_decel_guard_speed_margin);
    const double closing_speed = current_speed_mps_ - (front_speed + speed_margin);
    const double min_closing_speed =
      std::max(0.0, cfg.v2x_behavior.front_decel_guard_min_closing_speed);
    if (closing_speed < min_closing_speed) {
      return std::numeric_limits<double>::infinity();
    }
    const double ttc = closing_speed > kEps ?
      front_distance / closing_speed : std::numeric_limits<double>::infinity();
    const bool close_enough = front_distance <= guard_distance;
    const bool ttc_low = ttc <= guard_ttc;
    if (!close_enough && !ttc_low) {
      return std::numeric_limits<double>::infinity();
    }
    return std::min(cfg.v_max, front_speed + speed_margin);
  }

  bool apply_follow_velocity_limits(
    V2XBehaviorOutput & output, const double front_distance, const double front_speed,
    const bool curve_guard, const FrontRiskMetrics & front_risk,
    const FrontRiskLevel front_risk_level, bool & front_risk_applied) const
  {
    front_risk_applied = false;
    bool any_limit_applied = false;
    const bool active_pass_gap_hold =
      v2x_overtake_core::can_hold_active_pass_after_gap_loss(
      v2x_overtake_core::ActivePassGapHoldRequest{
        overtake_line_state_.phase == OvertakeLinePhase::Pass,
        overtake_line_state_.pass_front_overlap_exclusion_latched,
        output.locked_target_seen,
        output.locked_target_position_jump});
    const auto follow_speed_limit = v2x_overtake_core::resolve_follow_speed_limit(
      v2x_overtake_core::FollowSpeedLimitRequest{
        cfg.v2x_behavior.follow_speed_limit_enabled,
        active_pass_gap_hold,
        front_distance,
        cfg.v2x_behavior.follow_speed_limit_distance,
        front_speed,
        cfg.v2x_behavior.moving_front_speed_threshold,
        cfg.v2x_behavior.moving_follow_speed_margin,
        cfg.v2x_behavior.moving_follow_target_distance,
        cfg.v2x_behavior.moving_follow_recovery_speed_margin,
        cfg.v2x_behavior.moving_follow_distance_gain,
        front_distance_velocity_limit(front_distance),
        cfg.v2x_behavior.follow_velocity,
        cfg.v_max});
    output.follow_speed_limit_active = follow_speed_limit.active;
    output.follow_speed_limit_moving_front = follow_speed_limit.moving_front;
    if (follow_speed_limit.active) {
      output.target_velocity_limit =
        std::min(output.target_velocity_limit, follow_speed_limit.speed_limit_mps);
      any_limit_applied = true;
    }

    const double front_risk_limit = front_risk_velocity_limit(front_risk, front_risk_level);
    if (std::isfinite(front_risk_limit)) {
      output.target_velocity_limit = std::min(output.target_velocity_limit, front_risk_limit);
      front_risk_applied = true;
      any_limit_applied = true;
    }

    if (curve_guard) {
      const double front_risk_curve_limit = front_risk_curve_velocity_limit(front_risk);
      if (std::isfinite(front_risk_curve_limit)) {
        output.target_velocity_limit =
          std::min(output.target_velocity_limit, front_risk_curve_limit);
        front_risk_applied = true;
        any_limit_applied = true;
      }
    }

    const double decel_guard_limit =
      front_decel_guard_velocity_limit(front_distance, front_speed, curve_guard);
    if (std::isfinite(decel_guard_limit)) {
      output.target_velocity_limit = std::min(output.target_velocity_limit, decel_guard_limit);
      any_limit_applied = true;
    }
    return any_limit_applied;
  }

  int infer_gap_pass_side(const GapPlannerOutput & gap_output, const double current_ey) const
  {
    if (gap_output.pass_side_sign != 0) {
      return gap_output.pass_side_sign;
    }
    if (!gap_output.active || !gap_output.feasible) {
      return 0;
    }
    const std::size_t count =
      std::min(gap_output.target_active.size(), gap_output.target_ey.size());
    for (std::size_t i = 0; i < count; ++i) {
      if (!gap_output.target_active[i]) {
        continue;
      }
      const double delta = gap_output.target_ey[i] - current_ey;
      if (std::abs(delta) > 0.05) {
        return delta > 0.0 ? 1 : -1;
      }
    }
    return 0;
  }

  int choose_overtake_pass_side(
    const double obstacle_lateral, const double lower, const double upper) const
  {
    if (!std::isfinite(obstacle_lateral) || upper - lower <= kEps) {
      return 0;
    }
    const double obstacle_radius = cfg.v2x_gap.vehicle_radius + cfg.v2x_gap.prediction_margin;
    const double left_clearance = upper - (obstacle_lateral + obstacle_radius);
    const double right_clearance = (obstacle_lateral - obstacle_radius) - lower;
    if (left_clearance > kEps || right_clearance > kEps) {
      return left_clearance >= right_clearance ? 1 : -1;
    }

    const double center = 0.5 * (lower + upper);
    return obstacle_lateral >= center ? -1 : 1;
  }

  double overtake_side_clearance(
    const int pass_side_sign, const double obstacle_lateral, const double lower,
    const double upper) const
  {
    if (pass_side_sign == 0 || !std::isfinite(obstacle_lateral) || upper - lower <= kEps) {
      return 0.0;
    }
    const double obstacle_radius = cfg.v2x_gap.vehicle_radius + cfg.v2x_gap.prediction_margin;
    return pass_side_sign > 0 ?
      upper - (obstacle_lateral + obstacle_radius) :
      (obstacle_lateral - obstacle_radius) - lower;
  }

  double overtake_side_target(
    const double lower, const double upper, const int pass_side_sign) const
  {
    if (upper - lower <= kEps) {
      return 0.0;
    }
    const double center = 0.5 * (lower + upper);
    const double side_edge = pass_side_sign > 0 ? upper : lower;
    return 0.5 * (center + side_edge);
  }

  V2XBehaviorOutput commit_v2x_behavior_state(V2XBehaviorOutput output, const double now_sec)
  {
    const V2XBehaviorState desired_state = output.state;
    V2XBehaviorState final_state = output.state;
    const bool had_previous_state = v2x_behavior_state_initialized;
    const V2XBehaviorState previous_state = v2x_behavior_state;
    const bool low_speed_avoidance_watchdog_active =
      had_previous_state && previous_state == V2XBehaviorState::LowSpeedAvoidance &&
      final_state == V2XBehaviorState::LowSpeedAvoidance;
    const auto low_speed_stall = v2x_overtake_core::update_stall_watchdog(
      v2x_overtake_core::StallWatchdogRequest{
        low_speed_avoidance_watchdog_active,
        current_speed_mps_,
        now_sec,
        low_speed_avoidance_stall_last_update_sec_,
        low_speed_avoidance_stall_since_sec_,
        cfg.v2x_behavior.low_speed_avoidance_stall_speed,
        cfg.v2x_behavior.low_speed_avoidance_stall_timeout_sec,
        cfg.v2x_behavior.low_speed_avoidance_stall_max_observation_gap_sec});
    low_speed_avoidance_stall_last_update_sec_ = low_speed_stall.update_sec;
    low_speed_avoidance_stall_since_sec_ = low_speed_stall.stall_since_sec;
    output.low_speed_avoidance_stalled_sec = low_speed_stall.stalled_sec;
    if (low_speed_avoidance_watchdog_active && low_speed_stall.timed_out) {
      output.low_speed_avoidance_stalled = true;
      low_speed_avoidance_stall_cooldown_until_sec_ = std::max(
        low_speed_avoidance_stall_cooldown_until_sec_,
        now_sec + cfg.v2x_behavior.low_speed_avoidance_stall_cooldown_sec);
      output.low_speed_avoidance_cooldown_active = true;
      final_state =
        output.front_danger_action == v2x_overtake_core::FrontDangerAction::SafetyBrake ?
        V2XBehaviorState::SafetyBrake :
        (output.has_front_vehicle || output.has_side_vehicle ?
        V2XBehaviorState::Follow : V2XBehaviorState::Cruise);
      output.reason = "low-speed avoidance stalled";
    }
    if (v2x_behavior_state_initialized && !output.low_speed_avoidance_stalled) {
      const double elapsed = now_sec - last_v2x_behavior_state_change_sec;
      const bool more_restrictive =
        behavior_restriction_rank(final_state) > behavior_restriction_rank(v2x_behavior_state);
      if (!more_restrictive && elapsed < cfg.v2x_behavior.state_hold_time) {
        final_state = v2x_behavior_state;
      }
    }

    if (final_state == V2XBehaviorState::Overtake) {
      output.allow_gap_planner = true;
      if (!overtake_entry_speed_.has_value()) {
        overtake_entry_speed_ = std::max(0.0, current_speed_mps_);
      } else {
        overtake_entry_speed_ = std::max(
          overtake_entry_speed_.value(), std::max(0.0, current_speed_mps_));
      }
      if (cfg.v2x_behavior.overtake_stage_speed_enabled) {
        const double overtake_target_speed =
          output.locked_target_seen && std::isfinite(output.locked_target_speed) ?
          output.locked_target_speed : std::isfinite(output.front_speed) ?
          output.front_speed : overtake_line_state_.target_last_speed;
        if (std::isfinite(overtake_target_speed)) {
          const double pass_goal_ey =
            static_cast<double>(overtake_line_state_.pass_side_sign) *
            std::max(0.0, cfg.v2x_behavior.overtake_line.lateral_offset);
          const double pass_lateral_tolerance = std::max(
            0.15, 0.25 * std::max(0.0, cfg.v2x_behavior.overtake_line.lateral_offset));
          const bool pass_lateral_complete =
            overtake_core::has_reached_pass_side_lateral_goal(
              model->spatial_state.e_y, pass_goal_ey, pass_lateral_tolerance,
              overtake_line_state_.pass_side_sign);
          output.overtake_front_cap_release_ready =
            overtake_core::can_release_overtake_front_cap(
            overtake_core::OvertakeFrontCapReleaseRequest{
              overtake_line_state_.phase == OvertakeLinePhase::Pass,
              pass_lateral_complete, output.locked_target_seen,
              output.locked_target_longitudinal});
          const bool front_cap_stage = !output.overtake_front_cap_release_ready;
          const bool physical_shiftout_stage =
            overtake_line_state_.phase != OvertakeLinePhase::Pass;
          double closing_speed_limit =
            cfg.v2x_behavior.overtake_shiftout_max_closing_speed;
          closing_speed_limit = overtake_core::resolve_unlatched_pass_closing_speed(
            closing_speed_limit,
            cfg.v2x_behavior.overtake_pass_unlatched_max_closing_speed,
            overtake_line_state_.phase == OvertakeLinePhase::Pass,
            overtake_line_state_.pass_front_overlap_exclusion_latched);
          if (
            physical_shiftout_stage &&
            cfg.v2x_behavior.overtake_shiftout_adaptive_closing_speed_enabled &&
            std::isfinite(output.front_distance))
          {
            const double remaining_shiftout_distance =
              overtake_shiftout_remaining_distance(model->spatial_state.e_y);
            const auto adaptive_closing =
              v2x_overtake_core::resolve_adaptive_shiftout_closing_speed(
              v2x_overtake_core::AdaptiveShiftOutClosingSpeedRequest{
                cfg.v2x_behavior.overtake_shiftout_min_closing_speed,
                cfg.v2x_behavior.overtake_shiftout_max_closing_speed,
                std::max(0.0, output.front_distance),
                cfg.v2x_behavior.overtake_guard_min_front_distance,
                remaining_shiftout_distance,
                std::max(0.0, current_speed_mps_),
                std::max(kEps, cfg.v2x_behavior.overtake_guard_min_speed_for_reachable),
                cfg.v2x_behavior.overtake_shiftout_adaptive_min_time_sec});
            closing_speed_limit = adaptive_closing.closing_speed_mps;
            output.overtake_shiftout_adaptive_speed_applied = true;
            output.overtake_shiftout_closing_speed_limit = closing_speed_limit;
            output.overtake_shiftout_remaining_time = adaptive_closing.remaining_time_sec;
          }
          const double front_based_shiftout_cap =
            std::max(0.0, overtake_target_speed) + closing_speed_limit;
          const double entry_speed_floor =
            output.overtake_shiftout_adaptive_speed_applied ?
            std::min(overtake_entry_speed_.value(), front_based_shiftout_cap) :
            overtake_entry_speed_.value();
          const auto speed_reference = v2x_overtake_core::resolve_overtake_speed_reference(
            v2x_overtake_core::OvertakeSpeedReferenceRequest{
              front_cap_stage ?
              v2x_overtake_core::OvertakeSpeedStage::ShiftOut :
              v2x_overtake_core::OvertakeSpeedStage::Pass,
              cfg.v_max,
              cfg.v_max,
              std::max(0.0, overtake_target_speed),
              entry_speed_floor,
              closing_speed_limit});
          output.desired_velocity = speed_reference.reference_speed_mps;
          output.overtake_speed_front_cap_applied = speed_reference.front_cap_applied;
        } else {
          output.desired_velocity = cfg.v_max;
        }
      } else {
        const double front_based_desired = std::isfinite(output.front_speed) ?
          std::max(0.0, output.front_speed) +
          std::max(0.0, cfg.v2x_behavior.overtake_velocity_advantage) : 0.0;
        output.desired_velocity = std::min(
          cfg.v_max, std::max(overtake_entry_speed_.value(), front_based_desired));
        output.overtake_speed_front_cap_applied = std::isfinite(output.front_speed);
      }
    } else if (final_state == V2XBehaviorState::LowSpeedAvoidance) {
      output.allow_gap_planner = true;
      output.target_velocity_limit = std::min(
        output.target_velocity_limit, std::max(0.0, cfg.v2x_behavior.low_speed_avoidance_velocity));
    } else if (final_state == V2XBehaviorState::Follow) {
      // The explicit Pass line already owns the lateral target. A transient side-gap failure
      // must neither re-enable a second gap-planner target nor restore the generic Follow cap.
      const bool active_pass_gap_hold =
        v2x_overtake_core::can_hold_active_pass_after_gap_loss(
        v2x_overtake_core::ActivePassGapHoldRequest{
          overtake_line_state_.phase == OvertakeLinePhase::Pass,
          overtake_line_state_.pass_front_overlap_exclusion_latched,
          output.locked_target_seen,
          output.locked_target_position_jump});
      const bool follow_gap_planner_allowed =
        !active_pass_gap_hold &&
        (!cfg.v2x_behavior.follow_gap_planner_respect_overtake_forbidden ||
        output.follow_gap_planner_allowed);
      if (cfg.v2x_behavior.follow_gap_planner_enabled && follow_gap_planner_allowed) {
        output.allow_gap_planner = true;
      }
    } else if (final_state == V2XBehaviorState::SafetyBrake) {
      output.target_velocity_limit = std::max(0.0, cfg.v2x_behavior.safety_brake_velocity);
    }

    const bool leaving_overtake =
      had_previous_state && previous_state == V2XBehaviorState::Overtake &&
      final_state != V2XBehaviorState::Overtake;
    const bool curve_related_overtake_exit =
      output.overtake_forbidden || output.front_decel_curve_guard ||
      output.reason.find("curve") != std::string::npos ||
      output.reason.find("inner") != std::string::npos ||
      output.overtake_block_reason.find("curve") != std::string::npos ||
      output.overtake_block_reason.find("inner") != std::string::npos;
    if (
      cfg.v2x_behavior.overtake_curve_cooldown_enabled && leaving_overtake &&
      curve_related_overtake_exit) {
      overtake_curve_cooldown_until_sec_ = std::max(
        overtake_curve_cooldown_until_sec_,
        now_sec + std::max(0.0, cfg.v2x_behavior.overtake_curve_cooldown_sec));
      output.overtake_cooldown_active = true;
      overtake_locked_target_ey_.reset();
      overtake_locked_side_sign_ = 0;
    } else if (cfg.v2x_behavior.overtake_curve_cooldown_enabled) {
      output.overtake_cooldown_active =
        output.overtake_cooldown_active || now_sec < overtake_curve_cooldown_until_sec_;
    }

    if (gap_planner != nullptr && final_state != V2XBehaviorState::LowSpeedAvoidance) {
      if (v2x_behavior_state_initialized && v2x_behavior_state == V2XBehaviorState::LowSpeedAvoidance) {
        gap_planner->reset_low_speed_targets();
      } else {
        gap_planner->reset_low_speed_target_lock();
      }
    }

    if (!v2x_behavior_state_initialized || final_state != v2x_behavior_state) {
      RCLCPP_INFO(
        rclcpp::get_logger("mpc_controller"),
        "V2X behavior: %s -> %s, front_distance=%.2f, wp_id=%d, reason=%s, "
        "block=%s, gap=%d, soft_curve=%d, hard_curve=%d, inner_pass=%d, continue=%d, "
        "hard_continue=%d, outer_entry=%d, outer_hard=%d, "
        "inner_entry=%d, inner_hard=%d, "
        "hard_dist=%.2f, hard_avail=%.2f, hard_req=%.2f",
        v2x_behavior_state_initialized ? to_string(v2x_behavior_state) : "None", to_string(final_state),
        output.front_distance, model->wp_id, output.reason.c_str(),
        output.overtake_block_reason.c_str(), output.overtake_gap_available ? 1 : 0,
        output.overtake_forbidden && !output.overtake_forbidden_wp ? 1 : 0,
        output.overtake_hard_curve_blocked ? 1 : 0,
        output.overtake_inner_curve_pass ? 1 : 0,
        output.continuing_overtake_allowed ? 1 : 0,
        output.active_hard_curve_continuation_allowed ? 1 : 0,
        output.outer_curve_entry_allowed ? 1 : 0,
        output.outer_curve_hard_continuation_allowed ? 1 : 0,
        output.inner_curve_entry_allowed ? 1 : 0,
        output.inner_curve_hard_continuation_allowed ? 1 : 0,
        output.active_hard_curve_distance,
        output.active_hard_curve_available_distance,
        output.active_hard_curve_required_distance);
      v2x_behavior_state = final_state;
      last_v2x_behavior_state_change_sec = now_sec;
      v2x_behavior_state_initialized = true;
    }

    if (cfg.v2x_behavior.debug_log_enabled) {
      const double period = std::max(0.0, cfg.v2x_behavior.debug_log_period_sec);
      const bool log_now =
        !std::isfinite(last_v2x_behavior_debug_log_sec_) ||
        period <= kEps ||
        now_sec - last_v2x_behavior_debug_log_sec_ >= period;
      if (log_now) {
        RCLCPP_INFO(
          rclcpp::get_logger("mpc_controller"),
          "V2X debug: desired=%s, final=%s, allow_gap=%d, limit=%.2f, desired_v=%.2f, "
          "follow_cap=%d, follow_moving=%d, follow_cap_dist=%.2f, "
          "clearance_cap=%d, clearance_margin=%.2f, "
          "speed_cap=%d, cap_release=%d, adaptive_cap=%d, closing=%.2f, "
          "shift_t=%.2f, wp_id=%d, "
          "vehicles=%zu, front=%d, side=%d, danger=%d, danger_action=%s, "
          "grace=%d, grid_suppress=%d, "
          "hazard_hold=%d, hazard_remaining=%.2f, hazard_target=%s, "
          "fd=%.2f, progress=%d, local_fd=%.2f, path_lat=%.2f, fs=%.2f, ego=%.2f, "
          "rel=%.2f, req_dec=%.2f, avail=%.2f, risk=%s, "
          "forbid=%d, forbid_wp=%d, hard_curve=%d, inner_pass=%d, curve_guard=%d, "
          "low_speed=%d, low_speed_block=%d, "
          "low_stall=%.2f, low_timeout=%d, low_cooldown=%d, "
          "zone=%d, start_curve=%d, before_curve=%d, continue=%d, hard_continue=%d, "
          "outer_entry=%d, outer_hard=%d, "
          "inner_entry=%d, inner_hard=%d, "
          "hard_dist=%.2f, hard_avail=%.2f, hard_req=%.2f, completion=%d, "
          "completion_avail=%.2f, completion_req=%.2f, completion_rel=%.2f, "
          "gap=%d, fallback=%d, "
          "cooldown=%d, pass=%d, side_clear=%.2f, plan_N=%d, target=%s, locked_seen=%d, "
          "locked_s=%.2f, left_gap=%d, right_gap=%d, solver_failures=%d, "
          "left_reason=%s, right_reason=%s, reason=%s, block=%s",
          to_string(desired_state), to_string(final_state), output.allow_gap_planner ? 1 : 0,
          output.target_velocity_limit, output.desired_velocity,
          output.follow_speed_limit_active ? 1 : 0,
          output.follow_speed_limit_moving_front ? 1 : 0,
          cfg.v2x_behavior.follow_speed_limit_distance,
          output.moving_front_clearance_limit_active ? 1 : 0,
          output.moving_front_clearance_speed_margin,
          output.overtake_speed_front_cap_applied ? 1 : 0,
          output.overtake_front_cap_release_ready ? 1 : 0,
          output.overtake_shiftout_adaptive_speed_applied ? 1 : 0,
          output.overtake_shiftout_closing_speed_limit,
          output.overtake_shiftout_remaining_time, model->wp_id,
          output.active_vehicle_count,
          output.has_front_vehicle ? 1 : 0, output.has_side_vehicle ? 1 : 0,
          output.has_danger_vehicle ? 1 : 0,
          v2x_overtake_core::to_string(output.front_danger_action),
          output.start_grid_grace_active ? 1 : 0,
          output.start_grid_stop_suppressed ? 1 : 0,
          output.front_hazard_hold_active ? 1 : 0,
          output.front_hazard_hold_remaining_sec,
          output.front_hazard_hold_target_id.c_str(),
          output.front_distance, output.front_progress_used ? 1 : 0,
          output.front_local_longitudinal, output.front_progress_lateral,
          output.front_speed, output.ego_speed,
          output.front_risk.relative_speed, output.front_risk.required_decel,
          output.front_risk.available_distance, to_string(output.front_risk_level),
          output.overtake_forbidden ? 1 : 0, output.overtake_forbidden_wp ? 1 : 0,
          output.overtake_hard_curve_blocked ? 1 : 0,
          output.overtake_inner_curve_pass ? 1 : 0,
          output.front_decel_curve_guard ? 1 : 0,
          output.low_speed_avoidance_candidate ? 1 : 0,
          output.low_speed_avoidance_gap_blocked ? 1 : 0,
          output.low_speed_avoidance_stalled_sec,
          output.low_speed_avoidance_stalled ? 1 : 0,
          output.low_speed_avoidance_cooldown_active ? 1 : 0,
          output.overtake_zone_allows ? 1 : 0,
          output.overtake_start_curve_blocked ? 1 : 0,
          output.before_curve_overtake_allowed ? 1 : 0,
          output.continuing_overtake_allowed ? 1 : 0,
          output.active_hard_curve_continuation_allowed ? 1 : 0,
          output.outer_curve_entry_allowed ? 1 : 0,
          output.outer_curve_hard_continuation_allowed ? 1 : 0,
          output.inner_curve_entry_allowed ? 1 : 0,
          output.inner_curve_hard_continuation_allowed ? 1 : 0,
          output.active_hard_curve_distance,
          output.active_hard_curve_available_distance,
          output.active_hard_curve_required_distance,
          output.overtake_completion_feasible ? 1 : 0,
          output.overtake_completion_available_distance,
          output.overtake_completion_required_distance,
          output.overtake_completion_relative_speed,
          output.overtake_gap_available ? 1 : 0,
          output.overtake_fallback_target ? 1 : 0,
          output.overtake_cooldown_active ? 1 : 0, output.overtake_pass_side_sign,
          output.overtake_side_clearance, output.overtake_plan_N,
          output.target_vehicle_id.c_str(), output.locked_target_seen ? 1 : 0,
          output.locked_target_longitudinal,
          output.overtake_left_gap_available ? 1 : 0,
          output.overtake_right_gap_available ? 1 : 0, infeasibility_counter,
          output.overtake_left_reason.c_str(), output.overtake_right_reason.c_str(),
          output.reason.c_str(), output.overtake_block_reason.c_str());
        last_v2x_behavior_debug_log_sec_ = now_sec;
      }
    }

    if (final_state != V2XBehaviorState::LowSpeedAvoidance) {
      low_speed_avoidance_stall_since_sec_ = std::numeric_limits<double>::quiet_NaN();
      low_speed_avoidance_stall_last_update_sec_ = std::numeric_limits<double>::quiet_NaN();
    }

    output.state = final_state;
    return output;
  }
};

struct RefPathConfig
{
  bool update_by_topic{};
  std::string csv_path;
  bool domain_csv_path_applied{false};
  int domain_csv_path_domain{-1};
  double resolution{};
  int smoothing_distance{};
  double max_width{};
  bool circular{};
  bool use_path_constraints_topic{};
  bool use_border_cells_topic{};
};

struct StuckRecoveryAdapterConfig
{
  stuck_recovery::CoreConfig core;
  bool domain_enabled_applied{false};
  int domain_enabled_domain{-1};
  // A second, explicit latch keeps reverse actuation unavailable until the AWSIM gear and
  // acceleration semantics have been measured. Shadow detection remains usable independently.
  bool reverse_actuation_enabled{false};
  // Multiplied by the positive magnitude emitted by the pure core. Zero is the safe TBD default.
  double reverse_acceleration_sign{0.0};
  // Signed command measured to decelerate while the physical gear remains Reverse.
  double reverse_stop_acceleration_mps2{0.0};
  // Positive achieved deceleration measured in Reverse; used for stopping-distance reserve.
  double verified_reverse_stop_deceleration_mps2{0.0};
  double reverse_control_latency_sec{0.1};
  double max_reverse_pose_step_m{0.25};
  // Non-negative magnitude used by the deterministic Left/Right reverse rollouts.
  double reverse_steering_angle_rad{0.0};
  double boost_status_timeout_sec{0.5};
  // Total entries expected in each V2X array, including self if that simulator includes self.
  // -1 means unknown and therefore fail-closed for reverse actuation.
  int expected_v2x_vehicle_count{-1};
  std::string v2x_self_filter_mode{"unknown"};
  std::string self_vehicle_id;
  double rear_vehicle_radius_m{1.45};
  double rear_prediction_margin_sec{0.1};
  double reverse_escape_distance_m{2.0};
  double forward_escape_distance_m{0.30};
  // Maximum V2X front speed accepted as a stopped vehicle for a decentralized
  // tail-first reverse cascade.
  double coordinated_stop_front_speed_mps{0.20};
  // Large-heading solver failures wait for a reverse corridor instead of
  // switching to the short forward deadlock fallback.
  double solver_reverse_only_heading_error_rad{1.0};
  // Occupancy cells within this additional envelope are used only to infer
  // which vehicle end is against a wall; they do not relax collision checks.
  double wall_direction_search_margin_m{0.50};
  double wall_direction_ambiguity_m{0.02};
  bool side_escape_enabled{true};
  double side_escape_min_contact_reduction_ratio{0.05};
  std::size_t side_escape_steering_samples{5U};
  double front_extent_m{1.49};
  double rear_extent_m{0.51};
  double left_extent_m{0.725};
  double right_extent_m{0.725};
  double footprint_margin_m{0.05};
  double sweep_interpolation_step_m{0.05};
  double rejoin_static_lookahead_m{0.8};
  bool rejoin_feedback_steering_enabled{false};
  double rejoin_lateral_error_gain_rad_per_m{0.0};
  double rejoin_heading_error_gain{0.0};
  double rejoin_max_steering_tire_angle_rad{0.0};
  // Simulation-race fallback: after repeated bounded recovery failures, a
  // contact-free vehicle may enter feedback rejoin even when the short
  // forward swept-footprint prediction intersects a wall.
  std::size_t aggressive_force_rejoin_after_retries{0U};
  // Simulation-only sampled finite-horizon controller. The bounded Recovery
  // supervisor and all static/V2X gates remain authoritative.
  bool recovery_mpc_enabled{false};
  recovery_mpc::Config recovery_mpc_config;
};

stuck_recovery::ReverseActuationCalibration reverse_actuation_calibration(
  const StuckRecoveryAdapterConfig & config)
{
  return stuck_recovery::ReverseActuationCalibration{
    config.reverse_acceleration_sign *
    config.core.supervisor.reverse_acceleration_magnitude_mps2,
    config.reverse_stop_acceleration_mps2,
    config.verified_reverse_stop_deceleration_mps2,
    config.reverse_control_latency_sec};
}

struct Config
{
  bool save_config{};
  bool animation_enabled{};
  std::string map_yaml_path;
  std::string waypoints_csv_path;
  std::string obstacles_csv_path;
  double obstacle_radius{};
  RefPathConfig reference_path;
  double bicycle_length{};
  double bicycle_width{};
  awsim_boost::Config awsim_boost;
  StuckRecoveryAdapterConfig stuck_recovery;
  MpcConfig mpc;
};

struct RecoverySafetySnapshot
{
  bool wall_evidence{false};
  recovery_footprint::WallRegion wall_region{recovery_footprint::WallRegion::Unknown};
  double wall_distance_m{std::numeric_limits<double>::infinity()};
  bool rear_static_clear{false};
  bool rear_v2x_clear{false};
  bool rear_information_complete{false};
  bool boost_inactive_confirmed{false};
  bool v2x_message_complete{false};
  bool current_footprint_clear{false};
  bool rejoin_forward_static_clear{false};
  recovery_footprint::RejectReason rejoin_static_reject_reason{
    recovery_footprint::RejectReason::InvalidGrid};
  double rejoin_static_rejected_at_distance_m{};
  bool collision_worsening{false};
  std::vector<std::size_t> current_contact_cells;
  std::size_t current_contact_count{};
  recovery_footprint::RejectReason static_reject_reason{
    recovery_footprint::RejectReason::InvalidGrid};
  recovery_footprint::RejectReason runtime_contact_reject_reason{
    recovery_footprint::RejectReason::None};
  std::size_t static_initial_contact_count{};
  std::size_t static_maximum_contact_count{};
  std::size_t static_final_contact_count{};
  std::size_t static_checked_pose_count{};
  double static_rejected_at_distance_m{};
  bool reverse_candidate_selected{false};
  bool stepwise_escape{false};
  std::size_t contact_reduction{};
  stuck_recovery::ManeuverDirection maneuver_direction{
    stuck_recovery::ManeuverDirection::Unknown};
  recovery_footprint::ReversePrimitive selected_reverse_primitive{
    recovery_footprint::ReversePrimitive::Straight};
  double selected_reverse_steering_angle_rad{};
  bool recovery_mpc_guidance_used{false};
  double recovery_mpc_desired_steering_angle_rad{};
  double selected_center_min_lateral_m{};
  double selected_center_max_lateral_m{};
  std::string v2x_blocking_vehicle_id;
  std::string v2x_clearance_mode{"none"};
  std::string v2x_clearance_reason{"none"};
  double v2x_initial_clearance_m{std::numeric_limits<double>::infinity()};
  double v2x_minimum_clearance_m{std::numeric_limits<double>::infinity()};
  double v2x_final_clearance_m{std::numeric_limits<double>::infinity()};
  double v2x_rejected_at_distance_m{};
};

Config load_config(const std::string & path)
{
  const YAML::Node root = YAML::LoadFile(path);
  const auto ros_domain_id = ros_domain_id_from_env();
  Config cfg;
  cfg.save_config = root["common"]["save_config"].as<bool>();
  cfg.animation_enabled = root["sim_logger"]["animation_enabled"].as<bool>();
  cfg.map_yaml_path = root["map"]["yaml_path"].as<std::string>();
  cfg.waypoints_csv_path = root["waypoints"]["csv_path"].as<std::string>();
  cfg.obstacles_csv_path = root["obstacles"]["csv_path"].as<std::string>();
  cfg.obstacle_radius = root["obstacles"]["radius"].as<double>();
  const auto ref = root["reference_path"];
  cfg.reference_path.update_by_topic = ref["update_by_topic"].as<bool>();
  cfg.reference_path.csv_path = ref["csv_path"].as<std::string>();
  if (ref["domain_csv_path"] && ros_domain_id.has_value()) {
    for (const auto & item : ref["domain_csv_path"]) {
      if (item.first.as<int>() != ros_domain_id.value()) {
        continue;
      }
      cfg.reference_path.csv_path = item.second.as<std::string>();
      cfg.reference_path.domain_csv_path_applied = true;
      cfg.reference_path.domain_csv_path_domain = ros_domain_id.value();
      break;
    }
  }
  cfg.reference_path.resolution = ref["resolution"].as<double>();
  if (!std::isfinite(cfg.reference_path.resolution) || cfg.reference_path.resolution <= 0.0) {
    throw std::runtime_error("reference_path.resolution must be finite and positive");
  }
  cfg.reference_path.smoothing_distance = ref["smoothing_distance"].as<int>();
  if (cfg.reference_path.smoothing_distance < 0) {
    throw std::runtime_error("reference_path.smoothing_distance must be non-negative");
  }
  cfg.reference_path.max_width = ref["max_width"].as<double>();
  cfg.reference_path.circular = ref["circular"].as<bool>();
  cfg.reference_path.use_path_constraints_topic = ref["use_path_constraints_topic"].as<bool>();
  cfg.reference_path.use_border_cells_topic = ref["use_border_cells_topic"].as<bool>();
  cfg.bicycle_length = root["bicycle_model"]["length"].as<double>();
  cfg.bicycle_width = root["bicycle_model"]["width"].as<double>();

  const auto boost = root["awsim_boost"];
  if (boost) {
    cfg.awsim_boost.enabled = boost["enabled"] ? boost["enabled"].as<bool>() : false;
    std::map<int, bool> domain_enabled;
    const auto domain_enabled_node = boost["domain_enabled"];
    if (domain_enabled_node) {
      if (!domain_enabled_node.IsMap()) {
        throw std::runtime_error("awsim_boost.domain_enabled must be a map");
      }
      for (const auto & item : domain_enabled_node) {
        const int domain_id = item.first.as<int>();
        if (domain_id < 0) {
          throw std::runtime_error(
                  "awsim_boost.domain_enabled keys must be non-negative ROS domain IDs");
        }
        const bool enabled = item.second.as<bool>();
        if (!domain_enabled.emplace(domain_id, enabled).second) {
          throw std::runtime_error(
                  "awsim_boost.domain_enabled contains duplicate ROS domain ID " +
                  std::to_string(domain_id));
        }
      }
    }
    const auto enabled_resolution =
      awsim_boost::resolve_enabled(cfg.awsim_boost.enabled, domain_enabled, ros_domain_id);
    cfg.awsim_boost.enabled = enabled_resolution.enabled;
    cfg.awsim_boost.domain_enabled_applied = enabled_resolution.domain_override_applied;
    cfg.awsim_boost.domain_enabled_domain = enabled_resolution.domain_id;
    const std::string mode = boost["mode"] ?
      boost["mode"].as<std::string>() :
      (cfg.awsim_boost.enabled ? "start_once" : "disabled");
    cfg.awsim_boost.mode = awsim_boost::parse_mode(mode);
    const std::string trigger = boost["trigger"] ?
      boost["trigger"].as<std::string>() : "awsim_start";
    cfg.awsim_boost.trigger = awsim_boost::parse_trigger(trigger);
    cfg.awsim_boost.motion_speed_threshold_mps =
      boost["motion_speed_threshold_mps"] ?
      boost["motion_speed_threshold_mps"].as<double>() : 0.1;
    cfg.awsim_boost.max_trigger_speed_mps =
      boost["max_trigger_speed_mps"] ? boost["max_trigger_speed_mps"].as<double>() : 1.0;
    cfg.awsim_boost.motion_trigger_timeout_sec =
      boost["motion_trigger_timeout_sec"] ?
      boost["motion_trigger_timeout_sec"].as<double>() : 0.5;
    cfg.awsim_boost.status_timeout_sec =
      boost["status_timeout_sec"] ? boost["status_timeout_sec"].as<double>() : 0.5;
    cfg.awsim_boost.confirmation_timeout_sec =
      boost["confirmation_timeout_sec"] ?
      boost["confirmation_timeout_sec"].as<double>() : 2.0;
  }
  if (
    !std::isfinite(cfg.awsim_boost.motion_speed_threshold_mps) ||
    cfg.awsim_boost.motion_speed_threshold_mps <= 0.0)
  {
    throw std::runtime_error(
            "awsim_boost.motion_speed_threshold_mps must be finite and positive");
  }
  if (
    !std::isfinite(cfg.awsim_boost.max_trigger_speed_mps) ||
    cfg.awsim_boost.max_trigger_speed_mps < cfg.awsim_boost.motion_speed_threshold_mps)
  {
    throw std::runtime_error(
            "awsim_boost.max_trigger_speed_mps must be finite and at least the motion threshold");
  }
  if (
    !std::isfinite(cfg.awsim_boost.motion_trigger_timeout_sec) ||
    cfg.awsim_boost.motion_trigger_timeout_sec <= 0.0)
  {
    throw std::runtime_error(
            "awsim_boost.motion_trigger_timeout_sec must be finite and positive");
  }
  if (
    !std::isfinite(cfg.awsim_boost.status_timeout_sec) ||
    cfg.awsim_boost.status_timeout_sec <= 0.0)
  {
    throw std::runtime_error("awsim_boost.status_timeout_sec must be finite and positive");
  }
  if (
    !std::isfinite(cfg.awsim_boost.confirmation_timeout_sec) ||
    cfg.awsim_boost.confirmation_timeout_sec <= 0.0)
  {
    throw std::runtime_error("awsim_boost.confirmation_timeout_sec must be finite and positive");
  }

  const auto recovery = root["stuck_recovery"];
  if (recovery) {
    auto & adapter = cfg.stuck_recovery;
    auto & core = adapter.core;
    core.enabled = recovery["enabled"] ? recovery["enabled"].as<bool>() : false;
    const auto domain_enabled = recovery["domain_enabled"];
    if (domain_enabled) {
      if (!domain_enabled.IsMap()) {
        throw std::runtime_error("stuck_recovery.domain_enabled must be a map");
      }
      std::map<int, bool> values;
      for (const auto & item : domain_enabled) {
        const int domain_id = item.first.as<int>();
        if (domain_id < 0 || !values.emplace(domain_id, item.second.as<bool>()).second) {
          throw std::runtime_error(
                  "stuck_recovery.domain_enabled requires unique non-negative domain IDs");
        }
      }
      if (ros_domain_id.has_value()) {
        const auto override_value = values.find(ros_domain_id.value());
        if (override_value != values.end()) {
          core.enabled = override_value->second;
          adapter.domain_enabled_applied = true;
          adapter.domain_enabled_domain = ros_domain_id.value();
        }
      }
    }
    core.shadow_mode = recovery["shadow_mode"] ? recovery["shadow_mode"].as<bool>() : true;
    core.simulation_only =
      recovery["simulation_only"] ? recovery["simulation_only"].as<bool>() : true;
    adapter.reverse_actuation_enabled = recovery["reverse_actuation_enabled"] ?
      recovery["reverse_actuation_enabled"].as<bool>() : false;
    adapter.reverse_acceleration_sign = recovery["reverse_acceleration_sign"] ?
      recovery["reverse_acceleration_sign"].as<double>() : 0.0;
    adapter.reverse_stop_acceleration_mps2 = recovery["reverse_stop_acceleration_mps2"] ?
      recovery["reverse_stop_acceleration_mps2"].as<double>() : 0.0;
    adapter.verified_reverse_stop_deceleration_mps2 =
      recovery["verified_reverse_stop_deceleration_mps2"] ?
      recovery["verified_reverse_stop_deceleration_mps2"].as<double>() : 0.0;
    adapter.reverse_control_latency_sec = recovery["reverse_control_latency_sec"] ?
      recovery["reverse_control_latency_sec"].as<double>() : adapter.reverse_control_latency_sec;
    adapter.boost_status_timeout_sec = recovery["boost_status_timeout_sec"] ?
      recovery["boost_status_timeout_sec"].as<double>() : 0.5;

    const auto detector = recovery["detector"];
    if (detector) {
      auto & out = core.detector;
      out.solver_fallback_recovery_enabled = detector["solver_fallback_recovery_enabled"] ?
        detector["solver_fallback_recovery_enabled"].as<bool>() :
        out.solver_fallback_recovery_enabled;
      out.solver_fallback_duration_sec = detector["solver_fallback_duration_sec"] ?
        detector["solver_fallback_duration_sec"].as<double>() :
        out.solver_fallback_duration_sec;
      out.solver_evidence_free_recovery_enabled =
        detector["solver_evidence_free_recovery_enabled"] ?
        detector["solver_evidence_free_recovery_enabled"].as<bool>() :
        out.solver_evidence_free_recovery_enabled;
      out.solver_evidence_free_duration_sec =
        detector["solver_evidence_free_duration_sec"] ?
        detector["solver_evidence_free_duration_sec"].as<double>() :
        out.solver_evidence_free_duration_sec;
      out.evidence_free_recovery_enabled = detector["evidence_free_recovery_enabled"] ?
        detector["evidence_free_recovery_enabled"].as<bool>() :
        out.evidence_free_recovery_enabled;
      out.evidence_free_duration_sec = detector["evidence_free_duration_sec"] ?
        detector["evidence_free_duration_sec"].as<double>() :
        out.evidence_free_duration_sec;
      out.coordinated_stop_recovery_enabled =
        detector["coordinated_stop_recovery_enabled"] ?
        detector["coordinated_stop_recovery_enabled"].as<bool>() :
        out.coordinated_stop_recovery_enabled;
      out.coordinated_stop_duration_sec = detector["coordinated_stop_duration_sec"] ?
        detector["coordinated_stop_duration_sec"].as<double>() :
        out.coordinated_stop_duration_sec;
      adapter.coordinated_stop_front_speed_mps =
        detector["coordinated_stop_front_speed_mps"] ?
        detector["coordinated_stop_front_speed_mps"].as<double>() :
        adapter.coordinated_stop_front_speed_mps;
      out.max_observation_gap_sec = detector["max_observation_gap_sec"] ?
        detector["max_observation_gap_sec"].as<double>() :
        out.max_observation_gap_sec;
      out.stopped_speed_mps = detector["stopped_speed_mps"] ?
        detector["stopped_speed_mps"].as<double>() : out.stopped_speed_mps;
      out.moving_speed_mps = detector["moving_speed_mps"] ?
        detector["moving_speed_mps"].as<double>() : out.moving_speed_mps;
      out.forward_intent_speed_mps = detector["forward_intent_speed_mps"] ?
        detector["forward_intent_speed_mps"].as<double>() : out.forward_intent_speed_mps;
      out.forward_intent_acceleration_mps2 = detector["forward_intent_acceleration_mps2"] ?
        detector["forward_intent_acceleration_mps2"].as<double>() :
        out.forward_intent_acceleration_mps2;
      out.stationary_duration_sec = detector["stationary_duration_sec"] ?
        detector["stationary_duration_sec"].as<double>() : out.stationary_duration_sec;
      out.max_pose_displacement_m = detector["max_pose_displacement_m"] ?
        detector["max_pose_displacement_m"].as<double>() : out.max_pose_displacement_m;
      out.max_progress_delta_m = detector["max_progress_delta_m"] ?
        detector["max_progress_delta_m"].as<double>() : out.max_progress_delta_m;
      core.supervisor.awsim_recovery_wait_sec = detector["awsim_recovery_settle_sec"] ?
        detector["awsim_recovery_settle_sec"].as<double>() :
        core.supervisor.awsim_recovery_wait_sec;
    }

    const auto gear = recovery["gear"];
    if (gear) {
      auto & out = core.supervisor;
      out.gear_report_timeout_sec = gear["report_timeout_sec"] ?
        gear["report_timeout_sec"].as<double>() : out.gear_report_timeout_sec;
      out.stop_confirm_sec = gear["stop_confirm_sec"] ?
        gear["stop_confirm_sec"].as<double>() : out.stop_confirm_sec;
      out.gear_command_resend_interval_sec = gear["command_resend_interval_sec"] ?
        gear["command_resend_interval_sec"].as<double>() :
        out.gear_command_resend_interval_sec;
      out.max_gear_command_requests = gear["max_command_requests"] ?
        gear["max_command_requests"].as<std::size_t>() : out.max_gear_command_requests;
    }

    const auto maneuver = recovery["maneuver"];
    if (maneuver) {
      auto & out = core.supervisor;
      out.clearance_wait_timeout_sec = maneuver["clearance_wait_timeout_sec"] ?
        maneuver["clearance_wait_timeout_sec"].as<double>() : out.clearance_wait_timeout_sec;
      out.clearance_safe_stop_recovery_enabled =
        maneuver["clearance_safe_stop_recovery_enabled"] ?
        maneuver["clearance_safe_stop_recovery_enabled"].as<bool>() :
        out.clearance_safe_stop_recovery_enabled;
      out.safe_stop_clear_confirm_sec = maneuver["safe_stop_clear_confirm_sec"] ?
        maneuver["safe_stop_clear_confirm_sec"].as<double>() :
        out.safe_stop_clear_confirm_sec;
      out.aggressive_sim_recovery_enabled = maneuver["aggressive_sim_recovery_enabled"] ?
        maneuver["aggressive_sim_recovery_enabled"].as<bool>() :
        out.aggressive_sim_recovery_enabled;
      out.aggressive_retry_delay_sec = maneuver["aggressive_retry_delay_sec"] ?
        maneuver["aggressive_retry_delay_sec"].as<double>() :
        out.aggressive_retry_delay_sec;
      out.max_reverse_distance_m = maneuver["max_reverse_distance_m"] ?
        maneuver["max_reverse_distance_m"].as<double>() : out.max_reverse_distance_m;
      out.max_reverse_duration_sec = maneuver["max_reverse_duration_sec"] ?
        maneuver["max_reverse_duration_sec"].as<double>() : out.max_reverse_duration_sec;
      out.max_reverse_speed_mps = maneuver["max_reverse_speed_mps"] ?
        maneuver["max_reverse_speed_mps"].as<double>() : out.max_reverse_speed_mps;
      out.reverse_acceleration_magnitude_mps2 = maneuver["reverse_acceleration_magnitude_mps2"] ?
        maneuver["reverse_acceleration_magnitude_mps2"].as<double>() :
        out.reverse_acceleration_magnitude_mps2;
      out.max_forward_distance_m = maneuver["max_forward_distance_m"] ?
        maneuver["max_forward_distance_m"].as<double>() : out.max_forward_distance_m;
      out.max_forward_duration_sec = maneuver["max_forward_duration_sec"] ?
        maneuver["max_forward_duration_sec"].as<double>() : out.max_forward_duration_sec;
      out.max_forward_speed_mps = maneuver["max_forward_speed_mps"] ?
        maneuver["max_forward_speed_mps"].as<double>() : out.max_forward_speed_mps;
      out.forward_acceleration_magnitude_mps2 =
        maneuver["forward_acceleration_magnitude_mps2"] ?
        maneuver["forward_acceleration_magnitude_mps2"].as<double>() :
        out.forward_acceleration_magnitude_mps2;
      out.escape_step_distance_m = maneuver["escape_step_distance_m"] ?
        maneuver["escape_step_distance_m"].as<double>() : out.escape_step_distance_m;
      out.max_escape_steps = maneuver["max_escape_steps"] ?
        maneuver["max_escape_steps"].as<std::size_t>() : out.max_escape_steps;
      out.max_attempts = maneuver["max_attempts"] ?
        maneuver["max_attempts"].as<std::size_t>() : out.max_attempts;
      // Keep accepting the original shared key while the checked-in config
      // migrates to direction-specific escape distances. A rear-wall forward
      // recovery must not inherit the much longer front/side reverse target.
      if (maneuver["escape_distance_m"]) {
        adapter.reverse_escape_distance_m = maneuver["escape_distance_m"].as<double>();
        adapter.forward_escape_distance_m = maneuver["escape_distance_m"].as<double>();
      }
      adapter.reverse_escape_distance_m = maneuver["reverse_escape_distance_m"] ?
        maneuver["reverse_escape_distance_m"].as<double>() :
        adapter.reverse_escape_distance_m;
      adapter.forward_escape_distance_m = maneuver["forward_escape_distance_m"] ?
        maneuver["forward_escape_distance_m"].as<double>() :
        adapter.forward_escape_distance_m;
      adapter.solver_reverse_only_heading_error_rad =
        maneuver["solver_reverse_only_heading_error_rad"] ?
        maneuver["solver_reverse_only_heading_error_rad"].as<double>() :
        adapter.solver_reverse_only_heading_error_rad;
      adapter.max_reverse_pose_step_m = maneuver["max_reverse_pose_step_m"] ?
        maneuver["max_reverse_pose_step_m"].as<double>() :
        adapter.max_reverse_pose_step_m;
      adapter.reverse_steering_angle_rad = maneuver["reverse_steering_angle_rad"] ?
        maneuver["reverse_steering_angle_rad"].as<double>() :
        adapter.reverse_steering_angle_rad;
      adapter.wall_direction_search_margin_m = maneuver["wall_direction_search_margin_m"] ?
        maneuver["wall_direction_search_margin_m"].as<double>() :
        adapter.wall_direction_search_margin_m;
      adapter.wall_direction_ambiguity_m = maneuver["wall_direction_ambiguity_m"] ?
        maneuver["wall_direction_ambiguity_m"].as<double>() :
        adapter.wall_direction_ambiguity_m;
      adapter.side_escape_enabled = maneuver["side_escape_enabled"] ?
        maneuver["side_escape_enabled"].as<bool>() : adapter.side_escape_enabled;
      adapter.side_escape_min_contact_reduction_ratio =
        maneuver["side_escape_min_contact_reduction_ratio"] ?
        maneuver["side_escape_min_contact_reduction_ratio"].as<double>() :
        adapter.side_escape_min_contact_reduction_ratio;
      adapter.side_escape_steering_samples = maneuver["side_escape_steering_samples"] ?
        maneuver["side_escape_steering_samples"].as<std::size_t>() :
        adapter.side_escape_steering_samples;
    }

    const auto footprint = recovery["footprint"];
    if (footprint) {
      adapter.front_extent_m = footprint["front_extent_m"] ?
        footprint["front_extent_m"].as<double>() : adapter.front_extent_m;
      adapter.rear_extent_m = footprint["rear_extent_m"] ?
        footprint["rear_extent_m"].as<double>() : adapter.rear_extent_m;
      adapter.left_extent_m = footprint["left_extent_m"] ?
        footprint["left_extent_m"].as<double>() : adapter.left_extent_m;
      adapter.right_extent_m = footprint["right_extent_m"] ?
        footprint["right_extent_m"].as<double>() : adapter.right_extent_m;
      adapter.footprint_margin_m = footprint["margin_m"] ?
        footprint["margin_m"].as<double>() : adapter.footprint_margin_m;
      adapter.sweep_interpolation_step_m = footprint["sweep_interpolation_step_m"] ?
        footprint["sweep_interpolation_step_m"].as<double>() :
        adapter.sweep_interpolation_step_m;
    }

    const auto rear_safety = recovery["rear_safety"];
    if (rear_safety) {
      adapter.expected_v2x_vehicle_count = rear_safety["expected_v2x_vehicle_count"] ?
        rear_safety["expected_v2x_vehicle_count"].as<int>() :
        adapter.expected_v2x_vehicle_count;
      adapter.v2x_self_filter_mode = rear_safety["self_filter_mode"] ?
        rear_safety["self_filter_mode"].as<std::string>() : adapter.v2x_self_filter_mode;
      adapter.self_vehicle_id = rear_safety["self_vehicle_id"] ?
        rear_safety["self_vehicle_id"].as<std::string>() : adapter.self_vehicle_id;
      adapter.rear_vehicle_radius_m = rear_safety["vehicle_radius_m"] ?
        rear_safety["vehicle_radius_m"].as<double>() : adapter.rear_vehicle_radius_m;
      adapter.rear_prediction_margin_sec = rear_safety["prediction_margin_sec"] ?
        rear_safety["prediction_margin_sec"].as<double>() : adapter.rear_prediction_margin_sec;
    }

    const auto rejoin = recovery["rejoin"];
    if (rejoin) {
      auto & out = core.supervisor;
      out.rejoin_speed_limit_mps = rejoin["speed_limit_mps"] ?
        rejoin["speed_limit_mps"].as<double>() : out.rejoin_speed_limit_mps;
      out.max_rejoin_lateral_error_m = rejoin["max_lateral_error_m"] ?
        rejoin["max_lateral_error_m"].as<double>() : out.max_rejoin_lateral_error_m;
      out.max_rejoin_heading_error_rad = rejoin["max_heading_error_rad"] ?
        rejoin["max_heading_error_rad"].as<double>() : out.max_rejoin_heading_error_rad;
      out.rejoin_confirm_sec = rejoin["confirm_sec"] ?
        rejoin["confirm_sec"].as<double>() : out.rejoin_confirm_sec;
      out.rejoin_timeout_sec = rejoin["timeout_sec"] ?
        rejoin["timeout_sec"].as<double>() : out.rejoin_timeout_sec;
      out.rejoin_solver_recovery_timeout_sec = rejoin["solver_recovery_timeout_sec"] ?
        rejoin["solver_recovery_timeout_sec"].as<double>() :
        out.rejoin_solver_recovery_timeout_sec;
      out.retry_rejoin_blocked_path = rejoin["retry_on_blocked_path"] ?
        rejoin["retry_on_blocked_path"].as<bool>() : out.retry_rejoin_blocked_path;
      out.retry_rejoin_timeout = rejoin["retry_on_timeout"] ?
        rejoin["retry_on_timeout"].as<bool>() : out.retry_rejoin_timeout;
      adapter.rejoin_static_lookahead_m = rejoin["static_lookahead_m"] ?
        rejoin["static_lookahead_m"].as<double>() : adapter.rejoin_static_lookahead_m;
      adapter.rejoin_feedback_steering_enabled = rejoin["feedback_steering_enabled"] ?
        rejoin["feedback_steering_enabled"].as<bool>() :
        adapter.rejoin_feedback_steering_enabled;
      adapter.rejoin_lateral_error_gain_rad_per_m =
        rejoin["lateral_error_gain_rad_per_m"] ?
        rejoin["lateral_error_gain_rad_per_m"].as<double>() :
        adapter.rejoin_lateral_error_gain_rad_per_m;
      adapter.rejoin_heading_error_gain = rejoin["heading_error_gain"] ?
        rejoin["heading_error_gain"].as<double>() : adapter.rejoin_heading_error_gain;
      adapter.rejoin_max_steering_tire_angle_rad =
        rejoin["max_steering_tire_angle_rad"] ?
        rejoin["max_steering_tire_angle_rad"].as<double>() :
        adapter.rejoin_max_steering_tire_angle_rad;
      adapter.aggressive_force_rejoin_after_retries =
        rejoin["aggressive_force_after_retries"] ?
        rejoin["aggressive_force_after_retries"].as<std::size_t>() :
        adapter.aggressive_force_rejoin_after_retries;
      out.cooldown_sec = rejoin["cooldown_sec"] ?
        rejoin["cooldown_sec"].as<double>() : out.cooldown_sec;
    }

    const auto recovery_mpc_node = recovery["recovery_mpc"];
    if (recovery_mpc_node) {
      auto & mpc_config = adapter.recovery_mpc_config;
      adapter.recovery_mpc_enabled = recovery_mpc_node["enabled"] ?
        recovery_mpc_node["enabled"].as<bool>() : adapter.recovery_mpc_enabled;
      mpc_config.horizon_steps = recovery_mpc_node["horizon_steps"] ?
        recovery_mpc_node["horizon_steps"].as<std::size_t>() : mpc_config.horizon_steps;
      mpc_config.steering_sample_count = recovery_mpc_node["steering_sample_count"] ?
        recovery_mpc_node["steering_sample_count"].as<std::size_t>() :
        mpc_config.steering_sample_count;
      mpc_config.beam_width = recovery_mpc_node["beam_width"] ?
        recovery_mpc_node["beam_width"].as<std::size_t>() : mpc_config.beam_width;
      mpc_config.travel_step_m = recovery_mpc_node["travel_step_m"] ?
        recovery_mpc_node["travel_step_m"].as<double>() : mpc_config.travel_step_m;
      mpc_config.maximum_steering_angle_rad =
        recovery_mpc_node["maximum_steering_tire_angle_rad"] ?
        recovery_mpc_node["maximum_steering_tire_angle_rad"].as<double>() :
        mpc_config.maximum_steering_angle_rad;
      mpc_config.maximum_steering_change_rad =
        recovery_mpc_node["maximum_steering_change_rad"] ?
        recovery_mpc_node["maximum_steering_change_rad"].as<double>() :
        mpc_config.maximum_steering_change_rad;
      mpc_config.lateral_error_weight = recovery_mpc_node["lateral_error_weight"] ?
        recovery_mpc_node["lateral_error_weight"].as<double>() :
        mpc_config.lateral_error_weight;
      mpc_config.heading_error_weight = recovery_mpc_node["heading_error_weight"] ?
        recovery_mpc_node["heading_error_weight"].as<double>() :
        mpc_config.heading_error_weight;
      mpc_config.steering_weight = recovery_mpc_node["steering_weight"] ?
        recovery_mpc_node["steering_weight"].as<double>() : mpc_config.steering_weight;
      mpc_config.steering_change_weight = recovery_mpc_node["steering_change_weight"] ?
        recovery_mpc_node["steering_change_weight"].as<double>() :
        mpc_config.steering_change_weight;
      mpc_config.terminal_lateral_error_weight =
        recovery_mpc_node["terminal_lateral_error_weight"] ?
        recovery_mpc_node["terminal_lateral_error_weight"].as<double>() :
        mpc_config.terminal_lateral_error_weight;
      mpc_config.terminal_heading_error_weight =
        recovery_mpc_node["terminal_heading_error_weight"] ?
        recovery_mpc_node["terminal_heading_error_weight"].as<double>() :
        mpc_config.terminal_heading_error_weight;
    }

    const auto finite_non_negative = [](const double value) {
        return std::isfinite(value) && value >= 0.0;
      };
    if (core.supervisor.aggressive_sim_recovery_enabled && !core.simulation_only) {
      throw std::runtime_error(
              "stuck_recovery aggressive simulation recovery requires simulation_only: true");
    }
    if (adapter.recovery_mpc_enabled && !core.simulation_only) {
      throw std::runtime_error(
              "stuck_recovery recovery_mpc requires simulation_only: true");
    }
    if (!recovery_mpc::config_is_valid(adapter.recovery_mpc_config)) {
      throw std::runtime_error("stuck_recovery.recovery_mpc values are invalid");
    }
    const double absolute_reverse_sign = std::abs(adapter.reverse_acceleration_sign);
    if (
      !std::isfinite(adapter.reverse_acceleration_sign) ||
      (absolute_reverse_sign > kEps && std::abs(absolute_reverse_sign - 1.0) > kEps))
    {
      throw std::runtime_error("stuck_recovery.reverse_acceleration_sign must be -1, 0, or +1");
    }
    if (
      adapter.reverse_actuation_enabled &&
      (std::abs(adapter.reverse_acceleration_sign) < kEps ||
      !std::isfinite(adapter.reverse_stop_acceleration_mps2) ||
      std::abs(adapter.reverse_stop_acceleration_mps2) < kEps ||
      adapter.reverse_acceleration_sign * adapter.reverse_stop_acceleration_mps2 >= 0.0 ||
      !std::isfinite(adapter.verified_reverse_stop_deceleration_mps2) ||
      adapter.verified_reverse_stop_deceleration_mps2 <= 0.0 ||
      adapter.expected_v2x_vehicle_count < 0 ||
      adapter.v2x_self_filter_mode == "unknown" ||
      (adapter.v2x_self_filter_mode == "vehicle_id" && adapter.self_vehicle_id.empty())))
    {
      throw std::runtime_error(
              "stuck_recovery reverse actuation requires opposite drive/stop commands, measured "
              "stop deceleration, and an explicit V2X self-filter contract");
    }
    if (
      adapter.v2x_self_filter_mode != "unknown" &&
      adapter.v2x_self_filter_mode != "excluded" &&
      adapter.v2x_self_filter_mode != "vehicle_id")
    {
      throw std::runtime_error(
              "stuck_recovery.rear_safety.self_filter_mode must be unknown, excluded, or vehicle_id");
    }
    if (
      !std::isfinite(adapter.reverse_stop_acceleration_mps2) ||
      !finite_non_negative(adapter.verified_reverse_stop_deceleration_mps2) ||
      !finite_non_negative(adapter.reverse_control_latency_sec) ||
      !std::isfinite(adapter.max_reverse_pose_step_m) ||
      adapter.max_reverse_pose_step_m <= 0.0 ||
      !finite_non_negative(adapter.reverse_steering_angle_rad) ||
      adapter.reverse_steering_angle_rad >= 1.5707963267948966 ||
      !std::isfinite(adapter.boost_status_timeout_sec) ||
      adapter.boost_status_timeout_sec <= 0.0 ||
      adapter.expected_v2x_vehicle_count < -1 ||
      !finite_non_negative(adapter.rear_vehicle_radius_m) ||
      !finite_non_negative(adapter.rear_prediction_margin_sec) ||
      !finite_non_negative(adapter.reverse_escape_distance_m) ||
      !finite_non_negative(adapter.forward_escape_distance_m) ||
      !finite_non_negative(adapter.coordinated_stop_front_speed_mps) ||
      !std::isfinite(adapter.solver_reverse_only_heading_error_rad) ||
      adapter.solver_reverse_only_heading_error_rad <= 0.0 ||
      adapter.solver_reverse_only_heading_error_rad > 3.14159265358979323846 ||
      adapter.reverse_escape_distance_m <= 0.0 ||
      adapter.forward_escape_distance_m <= 0.0 ||
      adapter.reverse_escape_distance_m > core.supervisor.max_reverse_distance_m ||
      adapter.forward_escape_distance_m > core.supervisor.max_forward_distance_m ||
      !finite_non_negative(adapter.wall_direction_search_margin_m) ||
      !finite_non_negative(adapter.wall_direction_ambiguity_m) ||
      !std::isfinite(adapter.side_escape_min_contact_reduction_ratio) ||
      adapter.side_escape_min_contact_reduction_ratio <= 0.0 ||
      adapter.side_escape_min_contact_reduction_ratio > 1.0 ||
      adapter.side_escape_steering_samples == 0U ||
      adapter.side_escape_steering_samples > 16U ||
      !finite_non_negative(adapter.front_extent_m) ||
      !finite_non_negative(adapter.rear_extent_m) ||
      !finite_non_negative(adapter.left_extent_m) ||
      !finite_non_negative(adapter.right_extent_m) ||
      !finite_non_negative(adapter.footprint_margin_m) ||
      !std::isfinite(adapter.sweep_interpolation_step_m) ||
      adapter.sweep_interpolation_step_m <= 0.0 ||
      !std::isfinite(adapter.rejoin_static_lookahead_m) ||
      adapter.rejoin_static_lookahead_m <= 0.0 ||
      !finite_non_negative(adapter.rejoin_lateral_error_gain_rad_per_m) ||
      !finite_non_negative(adapter.rejoin_heading_error_gain) ||
      !finite_non_negative(adapter.rejoin_max_steering_tire_angle_rad) ||
      adapter.rejoin_max_steering_tire_angle_rad >= 1.5707963267948966 ||
      (adapter.rejoin_feedback_steering_enabled &&
      (adapter.rejoin_max_steering_tire_angle_rad <= 0.0 ||
      (adapter.rejoin_lateral_error_gain_rad_per_m <= 0.0 &&
      adapter.rejoin_heading_error_gain <= 0.0))))
    {
      throw std::runtime_error("stuck_recovery adapter values must be finite and within range");
    }
  }

  const auto mpc = root["mpc"];
  cfg.mpc.N = mpc["N"].as<int>();
  if (cfg.mpc.N < 2) {
    throw std::runtime_error(
            "Invalid mpc.N=" + std::to_string(cfg.mpc.N) + "; expected at least 2");
  }
  const auto Q = mpc["Q"].as<std::vector<double>>();
  const auto R = mpc["R"].as<std::vector<double>>();
  const auto QN = mpc["QN"].as<std::vector<double>>();
  cfg.mpc.Q = Eigen::Vector3d(Q.at(0), Q.at(1), Q.at(2));
  cfg.mpc.R = Eigen::Vector2d(R.at(0), R.at(1));
  cfg.mpc.QN = Eigen::Vector3d(QN.at(0), QN.at(1), QN.at(2));
  cfg.mpc.global_v_max_kmh = mpc["v_max"].as<double>();
  if (!std::isfinite(cfg.mpc.global_v_max_kmh) || cfg.mpc.global_v_max_kmh <= 0.0) {
    throw std::runtime_error("mpc.v_max must be finite and positive");
  }
  cfg.mpc.global_v_max = kmh_to_m_per_sec(cfg.mpc.global_v_max_kmh);
  cfg.mpc.v_max_kmh = cfg.mpc.global_v_max_kmh;
  cfg.mpc.v_max = cfg.mpc.global_v_max;
  if (ros_domain_id.has_value()) {
    cfg.mpc.ros_domain_id = ros_domain_id.value();
  }
  if (mpc["domain_v_max"] && ros_domain_id.has_value()) {
    for (const auto & item : mpc["domain_v_max"]) {
      if (item.first.as<int>() != ros_domain_id.value()) {
        continue;
      }
      const double domain_v_max_kmh = item.second.as<double>();
      if (!std::isfinite(domain_v_max_kmh) || domain_v_max_kmh < 0.0) {
        throw std::runtime_error("mpc.domain_v_max values must be finite and non-negative");
      }
      cfg.mpc.v_max_kmh = std::min(cfg.mpc.global_v_max_kmh, domain_v_max_kmh);
      cfg.mpc.v_max = kmh_to_m_per_sec(cfg.mpc.v_max_kmh);
      cfg.mpc.domain_v_max_applied = true;
      break;
    }
  }
  const double domain_start_duration =
    mpc["domain_start_v_max_duration"] ?
    mpc["domain_start_v_max_duration"].as<double>() : 0.0;
  if (!std::isfinite(domain_start_duration) || domain_start_duration < 0.0) {
    throw std::runtime_error(
            "mpc.domain_start_v_max_duration must be finite and non-negative");
  }
  cfg.mpc.domain_start_v_max_duration = domain_start_duration;
  if (mpc["domain_start_v_max"] && ros_domain_id.has_value()) {
    for (const auto & item : mpc["domain_start_v_max"]) {
      if (item.first.as<int>() != ros_domain_id.value()) {
        continue;
      }
      cfg.mpc.domain_start_v_max_kmh = item.second.as<double>();
      if (
        !std::isfinite(cfg.mpc.domain_start_v_max_kmh) ||
        cfg.mpc.domain_start_v_max_kmh < 0.0)
      {
        throw std::runtime_error(
                "mpc.domain_start_v_max values must be finite and non-negative");
      }
      cfg.mpc.domain_start_v_max = kmh_to_m_per_sec(cfg.mpc.domain_start_v_max_kmh);
      cfg.mpc.domain_start_v_max_applied = true;
      break;
    }
  }
  cfg.mpc.a_min = mpc["a_min"].as<double>();
  cfg.mpc.a_max = mpc["a_max"].as<double>();
  if (!std::isfinite(cfg.mpc.a_min) || cfg.mpc.a_min >= 0.0) {
    throw std::runtime_error("mpc.a_min must be finite and negative");
  }
  if (!std::isfinite(cfg.mpc.a_max) || cfg.mpc.a_max < 0.0) {
    throw std::runtime_error("mpc.a_max must be finite and non-negative");
  }
  if (mpc["domain_a_max"] && ros_domain_id.has_value()) {
    for (const auto & item : mpc["domain_a_max"]) {
      if (item.first.as<int>() != ros_domain_id.value()) {
        continue;
      }
      const double domain_a_max = item.second.as<double>();
      if (!std::isfinite(domain_a_max) || domain_a_max < 0.0) {
        throw std::runtime_error("mpc.domain_a_max values must be finite and non-negative");
      }
      cfg.mpc.a_max = domain_a_max;
      cfg.mpc.domain_a_max_applied = true;
      break;
    }
  }
  if (cfg.stuck_recovery.reverse_actuation_enabled) {
    if (!stuck_recovery::reverse_actuation_calibration_is_valid(
        reverse_actuation_calibration(cfg.stuck_recovery), cfg.mpc.a_min, cfg.mpc.a_max))
    {
      throw std::runtime_error(
              "stuck_recovery Reverse calibration is invalid or outside MPC acceleration bounds");
    }
  }
  cfg.mpc.ay_max = mpc["ay_max"].as<double>();
  cfg.mpc.delta_max = mpc["delta_max_deg"].as<double>() * kPi / 180.0;
  if (!std::isfinite(cfg.mpc.ay_max) || cfg.mpc.ay_max <= 0.0) {
    throw std::runtime_error("mpc.ay_max must be finite and positive");
  }
  if (!std::isfinite(cfg.mpc.delta_max) || cfg.mpc.delta_max <= 0.0) {
    throw std::runtime_error("mpc.delta_max_deg must be finite and positive");
  }
  cfg.mpc.steer_rate_max = mpc["steer_rate_max"].as<double>();
  cfg.mpc.control_rate = mpc["control_rate"].as<double>();
  cfg.mpc.solver_failure_steering_hold_cycles =
    mpc["solver_failure_steering_hold_cycles"] ?
    mpc["solver_failure_steering_hold_cycles"].as<int>() : 4;
  cfg.mpc.steering_tire_angle_gain_var = mpc["steering_tire_angle_gain_var"].as<double>();
  if (!std::isfinite(cfg.mpc.steer_rate_max) || cfg.mpc.steer_rate_max < 0.0) {
    throw std::runtime_error("mpc.steer_rate_max must be finite and non-negative");
  }
  if (!std::isfinite(cfg.mpc.control_rate) || cfg.mpc.control_rate <= 0.0) {
    throw std::runtime_error("mpc.control_rate must be finite and positive");
  }
  if (cfg.mpc.solver_failure_steering_hold_cycles < 0) {
    throw std::runtime_error(
            "mpc.solver_failure_steering_hold_cycles must be non-negative");
  }
  if (
    !std::isfinite(cfg.mpc.steering_tire_angle_gain_var) ||
    cfg.mpc.steering_tire_angle_gain_var <= 0.0)
  {
    throw std::runtime_error("mpc.steering_tire_angle_gain_var must be finite and positive");
  }
  cfg.mpc.odom_timeout_sec =
    mpc["odom_timeout_sec"] ? mpc["odom_timeout_sec"].as<double>() : 0.5;
  if (!std::isfinite(cfg.mpc.odom_timeout_sec) || cfg.mpc.odom_timeout_sec <= 0.0) {
    throw std::runtime_error("mpc.odom_timeout_sec must be finite and positive");
  }
  cfg.mpc.min_linearization_speed_mps =
    mpc["min_linearization_speed_mps"] ?
    mpc["min_linearization_speed_mps"].as<double>() : 0.5;
  if (
    !std::isfinite(cfg.mpc.min_linearization_speed_mps) ||
    cfg.mpc.min_linearization_speed_mps <= 0.0)
  {
    throw std::runtime_error("mpc.min_linearization_speed_mps must be finite and positive");
  }
  cfg.mpc.accel_low_pass_gain = mpc["accel_low_pass_gain"].as<double>();
  cfg.mpc.steer_low_pass_gain = mpc["steer_low_pass_gain"].as<double>();
  if (
    !std::isfinite(cfg.mpc.accel_low_pass_gain) || cfg.mpc.accel_low_pass_gain < 0.0 ||
    cfg.mpc.accel_low_pass_gain > 1.0)
  {
    throw std::runtime_error("mpc.accel_low_pass_gain must be finite and within [0, 1]");
  }
  if (
    !std::isfinite(cfg.mpc.steer_low_pass_gain) || cfg.mpc.steer_low_pass_gain < 0.0 ||
    cfg.mpc.steer_low_pass_gain > 1.0)
  {
    throw std::runtime_error("mpc.steer_low_pass_gain must be finite and within [0, 1]");
  }
  cfg.mpc.wp_id_offset = mpc["wp_id_offset"].as<int>();
  cfg.mpc.wp_id_low_offset =
    mpc["wp_id_low_offset"] ? mpc["wp_id_low_offset"].as<int>() : cfg.mpc.wp_id_offset;
  cfg.mpc.wp_id_low_speed_kmh =
    std::max(0.0, mpc["wp_id_low_speed"] ? mpc["wp_id_low_speed"].as<double>() : 0.0);
  cfg.mpc.wp_id_low_speed = kmh_to_m_per_sec(cfg.mpc.wp_id_low_speed_kmh);
  cfg.mpc.center_bias = clip(mpc["center_bias"] ? mpc["center_bias"].as<double>() : 1.0, 0.0, 1.0);
  cfg.mpc.safety_margin_scale = std::max(
    0.0, mpc["safety_margin_scale"] ? mpc["safety_margin_scale"].as<double>() : 1.0);
  const auto legacy_v2x = root["v2x_obstacle_avoidance"];
  cfg.mpc.v2x_gap.enabled =
    mpc["use_v2x_gap_planner"] ? mpc["use_v2x_gap_planner"].as<bool>() : false;
  cfg.mpc.v2x_gap.vehicle_radius = std::max(
    0.0, mpc["v2x_vehicle_radius"] ? mpc["v2x_vehicle_radius"].as<double>() :
    (legacy_v2x && legacy_v2x["vehicle_radius"] ? legacy_v2x["vehicle_radius"].as<double>() : 1.25));
  cfg.mpc.v2x_gap.vehicle_length = std::max(
    0.0, mpc["v2x_vehicle_length"] ? mpc["v2x_vehicle_length"].as<double>() : 2.0);
  cfg.mpc.v2x_gap.prediction_margin = std::max(
    0.0, mpc["v2x_prediction_margin"] ? mpc["v2x_prediction_margin"].as<double>() : 0.2);
  cfg.mpc.v2x_gap.prediction_time = std::max(
    0.0, mpc["v2x_prediction_time"] ? mpc["v2x_prediction_time"].as<double>() : 3.0);
  cfg.mpc.v2x_gap.prediction_use_path_time =
    mpc["v2x_prediction_use_path_time"] ?
    mpc["v2x_prediction_use_path_time"].as<bool>() : false;
  cfg.mpc.v2x_gap.prediction_min_ego_speed = std::max(
    kEps,
    mpc["v2x_prediction_min_ego_speed"] ?
    mpc["v2x_prediction_min_ego_speed"].as<double>() : 1.0);
  cfg.mpc.v2x_gap.prediction_max_ego_speed = cfg.mpc.v_max;
  cfg.mpc.v2x_gap.timeout_sec = std::max(
    0.0, mpc["v2x_timeout_sec"] ? mpc["v2x_timeout_sec"].as<double>() : 1.0);
  cfg.mpc.v2x_gap.position_jump_threshold = std::max(
    0.0,
    mpc["v2x_position_jump_threshold"] ? mpc["v2x_position_jump_threshold"].as<double>() :
    (legacy_v2x && legacy_v2x["position_jump_threshold"] ?
      legacy_v2x["position_jump_threshold"].as<double>() : 5.0));
  cfg.mpc.v2x_gap.v_max_safety = std::max(
    0.0, mpc["v2x_v_max_safety"] ? mpc["v2x_v_max_safety"].as<double>() :
    (legacy_v2x && legacy_v2x["v_max_safety"] ? legacy_v2x["v_max_safety"].as<double>() : 30.0));
  cfg.mpc.v2x_gap.self_filter_radius = std::max(
    0.0, mpc["v2x_self_filter_radius"] ? mpc["v2x_self_filter_radius"].as<double>() :
    std::max(1.0, cfg.bicycle_width));
  cfg.mpc.v2x_gap.min_gap_width = std::max(
    0.0, mpc["gap_min_width"] ? mpc["gap_min_width"].as<double>() : 1.8);
  cfg.mpc.v2x_gap.target_bias = clip(
    mpc["gap_target_bias"] ? mpc["gap_target_bias"].as<double>() : 1.0, 0.0, 1.0);
  cfg.mpc.v2x_gap.no_gap_target_velocity = std::max(
    0.0, mpc["no_gap_target_velocity"] ? mpc["no_gap_target_velocity"].as<double>() : 0.0);
  cfg.mpc.v2x_gap.wall_clearance_margin = std::max(
    0.0, mpc["v2x_wall_clearance_margin"] ? mpc["v2x_wall_clearance_margin"].as<double>() : 0.0);
  cfg.mpc.v2x_gap.vehicle_side_target_margin = std::max(
    0.0,
    mpc["v2x_vehicle_side_target_margin"] ?
    mpc["v2x_vehicle_side_target_margin"].as<double>() : 0.0);
  cfg.mpc.v2x_gap.wall_avoidance_bias = clip(
    mpc["v2x_wall_avoidance_bias"] ? mpc["v2x_wall_avoidance_bias"].as<double>() : 0.0, 0.0, 1.0);
  cfg.mpc.v2x_gap.vehicle_vehicle_gap_enabled =
    mpc["v2x_vehicle_vehicle_gap_enabled"] ?
    mpc["v2x_vehicle_vehicle_gap_enabled"].as<bool>() : true;
  cfg.mpc.v2x_gap.vehicle_vehicle_gap_min_distance = std::max(
    0.0,
    mpc["v2x_vehicle_vehicle_gap_min_distance"] ?
    mpc["v2x_vehicle_vehicle_gap_min_distance"].as<double>() : 0.0);
  cfg.mpc.v2x_gap.vehicle_vehicle_gap_min_width = std::max(
    0.0,
    mpc["v2x_vehicle_vehicle_gap_min_width"] ?
    mpc["v2x_vehicle_vehicle_gap_min_width"].as<double>() : 0.0);
  cfg.mpc.v2x_gap.multi_front_gap_enabled =
    mpc["v2x_multi_front_gap_enabled"] ?
    mpc["v2x_multi_front_gap_enabled"].as<bool>() : true;
  cfg.mpc.v2x_gap.multi_front_gap_distance = std::max(
    0.0,
    mpc["v2x_multi_front_gap_distance"] ?
    mpc["v2x_multi_front_gap_distance"].as<double>() : 0.0);
  cfg.mpc.v2x_gap.low_speed_pass_side =
    mpc["v2x_low_speed_pass_side"] ?
    mpc["v2x_low_speed_pass_side"].as<std::string>() : "auto";
  if (
    cfg.mpc.v2x_gap.low_speed_pass_side != "auto" &&
    cfg.mpc.v2x_gap.low_speed_pass_side != "left" &&
    cfg.mpc.v2x_gap.low_speed_pass_side != "right") {
    cfg.mpc.v2x_gap.low_speed_pass_side = "auto";
  }
  cfg.mpc.v2x_gap.low_speed_pass_ramp_ratio = std::max(
    0.1,
    mpc["v2x_low_speed_pass_ramp_ratio"] ?
    mpc["v2x_low_speed_pass_ramp_ratio"].as<double>() : 1.0);
  cfg.mpc.v2x_gap.overtake_target_ramp_enabled =
    mpc["v2x_overtake_target_ramp_enabled"] ?
    mpc["v2x_overtake_target_ramp_enabled"].as<bool>() : false;
  cfg.mpc.v2x_gap.overtake_target_ramp_ratio = std::max(
    0.1,
    mpc["v2x_overtake_target_ramp_ratio"] ?
    mpc["v2x_overtake_target_ramp_ratio"].as<double>() : 0.7);
  cfg.mpc.v2x_behavior.overtake_line.enabled =
    mpc["v2x_overtake_line_enabled"] ?
    mpc["v2x_overtake_line_enabled"].as<bool>() : false;
  cfg.mpc.v2x_behavior.overtake_line.shift_distance = std::max(
    0.5,
    mpc["v2x_overtake_line_shift_distance"] ?
    mpc["v2x_overtake_line_shift_distance"].as<double>() : 8.0);
  cfg.mpc.v2x_behavior.overtake_line.pass_distance = std::max(
    0.5,
    mpc["v2x_overtake_line_pass_distance"] ?
    mpc["v2x_overtake_line_pass_distance"].as<double>() : 8.0);
  cfg.mpc.v2x_behavior.overtake_line.return_distance = std::max(
    0.5,
    mpc["v2x_overtake_line_return_distance"] ?
    mpc["v2x_overtake_line_return_distance"].as<double>() : 10.0);
  cfg.mpc.v2x_behavior.overtake_line.lateral_offset = std::max(
    0.0,
    mpc["v2x_overtake_line_lateral_offset"] ?
    mpc["v2x_overtake_line_lateral_offset"].as<double>() : 1.2);
  cfg.mpc.v2x_behavior.overtake_line.target_bias = clip(
    mpc["v2x_overtake_line_target_bias"] ?
    mpc["v2x_overtake_line_target_bias"].as<double>() : 0.8, 0.0, 1.0);
  cfg.mpc.v2x_behavior.overtake_line.min_wall_clearance = std::max(
    0.0,
    mpc["v2x_overtake_line_min_wall_clearance"] ?
    mpc["v2x_overtake_line_min_wall_clearance"].as<double>() : 0.8);
  cfg.mpc.v2x_behavior.overtake_line.max_lateral_accel = std::max(
    0.0,
    mpc["v2x_overtake_line_max_lateral_accel"] ?
    mpc["v2x_overtake_line_max_lateral_accel"].as<double>() : 2.5);
  cfg.mpc.v2x_behavior.overtake_line.max_target_change = std::max(
    0.0,
    mpc["v2x_overtake_line_max_target_change"] ?
    mpc["v2x_overtake_line_max_target_change"].as<double>() : 0.25);
  cfg.mpc.v2x_behavior.overtake_line.return_clear_distance = std::max(
    0.0,
    mpc["v2x_overtake_line_return_clear_distance"] ?
    mpc["v2x_overtake_line_return_clear_distance"].as<double>() : 4.0);
  cfg.mpc.v2x_behavior.overtake_line.phase_hold_time = std::max(
    0.0,
    mpc["v2x_overtake_line_phase_hold_time"] ?
    mpc["v2x_overtake_line_phase_hold_time"].as<double>() : 0.3);
  cfg.mpc.v2x_behavior.overtake_line.target_hold_sec = std::max(
    0.0,
    mpc["v2x_overtake_target_hold_sec"] ?
    mpc["v2x_overtake_target_hold_sec"].as<double>() : 0.0);
  cfg.mpc.v2x_behavior.overtake_line.clear_confirm_sec = std::max(
    0.0,
    mpc["v2x_overtake_clear_confirm_sec"] ?
    mpc["v2x_overtake_clear_confirm_sec"].as<double>() : 0.0);
  cfg.mpc.v2x_behavior.overtake_line.reacquire_enabled =
    mpc["v2x_overtake_reacquire_enabled"] ?
    mpc["v2x_overtake_reacquire_enabled"].as<bool>() : false;
  cfg.mpc.v2x_behavior.overtake_line.reacquire_window_sec = std::max(
    0.0,
    mpc["v2x_overtake_reacquire_window_sec"] ?
    mpc["v2x_overtake_reacquire_window_sec"].as<double>() : 0.0);
  cfg.mpc.v2x_behavior.overtake_line.reacquire_max_return_progress = clip(
    mpc["v2x_overtake_reacquire_max_return_progress"] ?
    mpc["v2x_overtake_reacquire_max_return_progress"].as<double>() : 0.0, 0.0, 1.0);
  cfg.mpc.v2x_behavior.overtake_line.recovery_velocity_limit_enabled =
    mpc["v2x_overtake_recovery_velocity_limit_enabled"] ?
    mpc["v2x_overtake_recovery_velocity_limit_enabled"].as<bool>() : true;
  cfg.mpc.v2x_behavior.overtake_line.recovery_velocity = std::max(
    0.0,
    mpc["v2x_overtake_recovery_velocity"] ?
    mpc["v2x_overtake_recovery_velocity"].as<double>() : 5.0);
  cfg.mpc.v2x_behavior.overtake_line.recovery_stall_speed =
    mpc["v2x_overtake_recovery_stall_speed"] ?
    mpc["v2x_overtake_recovery_stall_speed"].as<double>() : 0.15;
  cfg.mpc.v2x_behavior.overtake_line.recovery_stall_timeout_sec =
    mpc["v2x_overtake_recovery_stall_timeout_sec"] ?
    mpc["v2x_overtake_recovery_stall_timeout_sec"].as<double>() : 1.0;
  cfg.mpc.v2x_behavior.overtake_line.recovery_timeout_sec =
    mpc["v2x_overtake_recovery_timeout_sec"] ?
    mpc["v2x_overtake_recovery_timeout_sec"].as<double>() : 5.0;
  cfg.mpc.v2x_behavior.overtake_line.recovery_max_observation_gap_sec =
    mpc["v2x_overtake_recovery_max_observation_gap_sec"] ?
    mpc["v2x_overtake_recovery_max_observation_gap_sec"].as<double>() : 0.2;
  cfg.mpc.v2x_behavior.overtake_line.solver_cooldown_sec =
    mpc["v2x_overtake_solver_cooldown_sec"] ?
    mpc["v2x_overtake_solver_cooldown_sec"].as<double>() : 2.0;
  cfg.mpc.v2x_behavior.overtake_line.solver_failure_abort_cycles = std::max(
    1,
    mpc["v2x_overtake_solver_failure_abort_cycles"] ?
    mpc["v2x_overtake_solver_failure_abort_cycles"].as<int>() : 3);
  cfg.mpc.v2x_behavior.overtake_line.solver_recovery_success_cycles = std::max(
    1,
    mpc["v2x_overtake_solver_recovery_success_cycles"] ?
    mpc["v2x_overtake_solver_recovery_success_cycles"].as<int>() : 20);
  cfg.mpc.v2x_behavior.overtake_line.debug_log_enabled =
    mpc["v2x_overtake_line_debug_log_enabled"] ?
    mpc["v2x_overtake_line_debug_log_enabled"].as<bool>() : false;
  cfg.mpc.v2x_behavior.enabled =
    mpc["use_v2x_behavior_fsm"] ? mpc["use_v2x_behavior_fsm"].as<bool>() : false;
  cfg.mpc.v2x_behavior.debug_log_enabled =
    mpc["v2x_behavior_debug_log_enabled"] ?
    mpc["v2x_behavior_debug_log_enabled"].as<bool>() : false;
  cfg.mpc.v2x_behavior.debug_log_period_sec = std::max(
    0.0,
    mpc["v2x_behavior_debug_log_period_sec"] ?
    mpc["v2x_behavior_debug_log_period_sec"].as<double>() : 1.0);
  cfg.mpc.v2x_behavior.front_progress_detection_enabled =
    mpc["v2x_front_progress_detection_enabled"] ?
    mpc["v2x_front_progress_detection_enabled"].as<bool>() : false;
  cfg.mpc.v2x_behavior.front_progress_detection_distance =
    mpc["v2x_front_progress_detection_distance"] ?
    mpc["v2x_front_progress_detection_distance"].as<double>() : 0.0;
  cfg.mpc.v2x_behavior.front_progress_lookbehind_distance =
    mpc["v2x_front_progress_lookbehind_distance"] ?
    mpc["v2x_front_progress_lookbehind_distance"].as<double>() : 3.0;
  cfg.mpc.v2x_behavior.front_hazard_hold_enabled =
    mpc["v2x_front_hazard_hold_enabled"] ?
    mpc["v2x_front_hazard_hold_enabled"].as<bool>() : false;
  cfg.mpc.v2x_behavior.front_hazard_hold_sec =
    mpc["v2x_front_hazard_hold_sec"] ?
    mpc["v2x_front_hazard_hold_sec"].as<double>() : 0.0;
  cfg.mpc.v2x_behavior.front_hazard_rear_clear_distance =
    mpc["v2x_front_hazard_rear_clear_distance"] ?
    mpc["v2x_front_hazard_rear_clear_distance"].as<double>() : 4.0;
  if (
    !std::isfinite(cfg.mpc.v2x_behavior.front_progress_detection_distance) ||
    cfg.mpc.v2x_behavior.front_progress_detection_distance < 0.0 ||
    (cfg.mpc.v2x_behavior.front_progress_detection_enabled &&
    cfg.mpc.v2x_behavior.front_progress_detection_distance <= 0.0))
  {
    throw std::runtime_error(
            "mpc.v2x_front_progress_detection_distance must be finite and positive when enabled");
  }
  if (
    !std::isfinite(cfg.mpc.v2x_behavior.front_progress_lookbehind_distance) ||
    cfg.mpc.v2x_behavior.front_progress_lookbehind_distance < 0.0)
  {
    throw std::runtime_error(
            "mpc.v2x_front_progress_lookbehind_distance must be finite and non-negative");
  }
  if (
    !std::isfinite(cfg.mpc.v2x_behavior.front_hazard_hold_sec) ||
    cfg.mpc.v2x_behavior.front_hazard_hold_sec < 0.0 ||
    (cfg.mpc.v2x_behavior.front_hazard_hold_enabled &&
    cfg.mpc.v2x_behavior.front_hazard_hold_sec <= 0.0))
  {
    throw std::runtime_error(
            "mpc.v2x_front_hazard_hold_sec must be finite and positive when enabled");
  }
  if (
    !std::isfinite(cfg.mpc.v2x_behavior.front_hazard_rear_clear_distance) ||
    cfg.mpc.v2x_behavior.front_hazard_rear_clear_distance <= 0.0)
  {
    throw std::runtime_error(
            "mpc.v2x_front_hazard_rear_clear_distance must be finite and positive");
  }
  cfg.mpc.v2x_behavior.follow_distance = std::max(
    0.0, mpc["v2x_follow_distance"] ? mpc["v2x_follow_distance"].as<double>() : 8.0);
  cfg.mpc.v2x_behavior.safety_brake_distance = std::max(
    0.0,
    mpc["v2x_safety_brake_distance"] ? mpc["v2x_safety_brake_distance"].as<double>() : 3.0);
  cfg.mpc.v2x_behavior.safety_brake_margin = std::max(
    0.0, mpc["v2x_safety_brake_margin"] ? mpc["v2x_safety_brake_margin"].as<double>() : 2.0);
  cfg.mpc.v2x_behavior.follow_gap_planner_enabled =
    mpc["v2x_follow_gap_planner_enabled"] ?
    mpc["v2x_follow_gap_planner_enabled"].as<bool>() : false;
  cfg.mpc.v2x_behavior.follow_gap_planner_no_gap_speed_limit_enabled =
    mpc["v2x_follow_gap_planner_no_gap_speed_limit_enabled"] ?
    mpc["v2x_follow_gap_planner_no_gap_speed_limit_enabled"].as<bool>() : false;
  cfg.mpc.v2x_behavior.follow_gap_planner_respect_overtake_forbidden =
    mpc["v2x_follow_gap_planner_respect_overtake_forbidden"] ?
    mpc["v2x_follow_gap_planner_respect_overtake_forbidden"].as<bool>() : true;
  cfg.mpc.v2x_behavior.follow_speed_limit_enabled =
    mpc["v2x_follow_speed_limit_enabled"] ?
    mpc["v2x_follow_speed_limit_enabled"].as<bool>() : false;
  cfg.mpc.v2x_behavior.follow_speed_limit_distance =
    mpc["v2x_follow_speed_limit_distance"] ?
    mpc["v2x_follow_speed_limit_distance"].as<double>() : 0.0;
  if (
    !std::isfinite(cfg.mpc.v2x_behavior.follow_speed_limit_distance) ||
    cfg.mpc.v2x_behavior.follow_speed_limit_distance < 0.0)
  {
    throw std::runtime_error(
            "mpc.v2x_follow_speed_limit_distance must be finite and non-negative");
  }
  cfg.mpc.v2x_behavior.follow_velocity = std::max(
    0.0, mpc["v2x_follow_velocity"] ? mpc["v2x_follow_velocity"].as<double>() : 5.0);
  if (!mpc["v2x_overtake_recovery_velocity"]) {
    // Preserve the legacy Recovery cap when loading an older config that predates the
    // dedicated setting. New configs can tune Recovery independently of Follow.
    cfg.mpc.v2x_behavior.overtake_line.recovery_velocity =
      cfg.mpc.v2x_behavior.follow_velocity;
  }
  const auto & overtake_line = cfg.mpc.v2x_behavior.overtake_line;
  if (!std::isfinite(overtake_line.recovery_velocity) || overtake_line.recovery_velocity <= 0.0) {
    throw std::runtime_error("mpc.v2x_overtake_recovery_velocity must be finite and positive");
  }
  if (
    !std::isfinite(overtake_line.recovery_stall_speed) ||
    overtake_line.recovery_stall_speed <= 0.0)
  {
    throw std::runtime_error(
            "mpc.v2x_overtake_recovery_stall_speed must be finite and positive");
  }
  if (
    !std::isfinite(overtake_line.recovery_stall_timeout_sec) ||
    overtake_line.recovery_stall_timeout_sec <= 0.0)
  {
    throw std::runtime_error(
            "mpc.v2x_overtake_recovery_stall_timeout_sec must be finite and positive");
  }
  if (
    !std::isfinite(overtake_line.recovery_timeout_sec) ||
    overtake_line.recovery_timeout_sec <= 0.0)
  {
    throw std::runtime_error(
            "mpc.v2x_overtake_recovery_timeout_sec must be finite and positive");
  }
  if (overtake_line.recovery_stall_timeout_sec > overtake_line.recovery_timeout_sec) {
    throw std::runtime_error(
            "mpc.v2x_overtake_recovery_stall_timeout_sec must not exceed total timeout");
  }
  if (
    !std::isfinite(overtake_line.recovery_max_observation_gap_sec) ||
    overtake_line.recovery_max_observation_gap_sec <= 0.0)
  {
    throw std::runtime_error(
            "mpc.v2x_overtake_recovery_max_observation_gap_sec must be finite and positive");
  }
  if (
    !std::isfinite(overtake_line.solver_cooldown_sec) ||
    overtake_line.solver_cooldown_sec < 0.0)
  {
    throw std::runtime_error(
            "mpc.v2x_overtake_solver_cooldown_sec must be finite and non-negative");
  }
  cfg.mpc.v2x_behavior.follow_preposition_enabled =
    mpc["v2x_follow_preposition_enabled"] ?
    mpc["v2x_follow_preposition_enabled"].as<bool>() : false;
  cfg.mpc.v2x_behavior.follow_preposition_offset = std::max(
    0.0,
    mpc["v2x_follow_preposition_offset"] ?
    mpc["v2x_follow_preposition_offset"].as<double>() : 0.0);
  cfg.mpc.v2x_behavior.follow_preposition_min_side_clearance = std::max(
    0.0,
    mpc["v2x_follow_preposition_min_side_clearance"] ?
    mpc["v2x_follow_preposition_min_side_clearance"].as<double>() : 1.2);
  cfg.mpc.v2x_behavior.follow_preposition_target_bias = clip(
    mpc["v2x_follow_preposition_target_bias"] ?
    mpc["v2x_follow_preposition_target_bias"].as<double>() : 0.25, 0.0, 1.0);
  cfg.mpc.v2x_behavior.follow_preposition_ramp_ratio = std::max(
    0.1,
    mpc["v2x_follow_preposition_ramp_ratio"] ?
    mpc["v2x_follow_preposition_ramp_ratio"].as<double>() : 1.5);
  cfg.mpc.v2x_behavior.front_decel_guard_enabled =
    mpc["v2x_front_decel_guard_enabled"] ?
    mpc["v2x_front_decel_guard_enabled"].as<bool>() : true;
  cfg.mpc.v2x_behavior.front_decel_guard_distance = std::max(
    0.0,
    mpc["v2x_front_decel_guard_distance"] ?
    mpc["v2x_front_decel_guard_distance"].as<double>() : 9.0);
  cfg.mpc.v2x_behavior.front_decel_guard_ttc = std::max(
    0.0,
    mpc["v2x_front_decel_guard_ttc"] ?
    mpc["v2x_front_decel_guard_ttc"].as<double>() : 1.5);
  cfg.mpc.v2x_behavior.front_decel_guard_speed_margin = std::max(
    0.0,
    mpc["v2x_front_decel_guard_speed_margin"] ?
    mpc["v2x_front_decel_guard_speed_margin"].as<double>() : 0.5);
  cfg.mpc.v2x_behavior.front_decel_guard_min_closing_speed = std::max(
    0.0,
    mpc["v2x_front_decel_guard_min_closing_speed"] ?
    mpc["v2x_front_decel_guard_min_closing_speed"].as<double>() : 1.5);
  cfg.mpc.v2x_behavior.front_decel_guard_curve_include_slow_front =
    mpc["v2x_front_decel_guard_curve_include_slow_front"] ?
    mpc["v2x_front_decel_guard_curve_include_slow_front"].as<bool>() : false;
  cfg.mpc.v2x_behavior.front_decel_guard_curve_lateral_margin = std::max(
    0.0,
    mpc["v2x_front_decel_guard_curve_lateral_margin"] ?
    mpc["v2x_front_decel_guard_curve_lateral_margin"].as<double>() : 0.0);
  cfg.mpc.v2x_behavior.front_decel_guard_curve_lookahead_distance = std::max(
    0.0,
    mpc["v2x_front_decel_guard_curve_lookahead_distance"] ?
    mpc["v2x_front_decel_guard_curve_lookahead_distance"].as<double>() : 0.0);
  cfg.mpc.v2x_behavior.front_decel_guard_curve_distance = std::max(
    0.0,
    mpc["v2x_front_decel_guard_curve_distance"] ?
    mpc["v2x_front_decel_guard_curve_distance"].as<double>() : 16.0);
  cfg.mpc.v2x_behavior.front_decel_guard_curve_ttc = std::max(
    0.0,
    mpc["v2x_front_decel_guard_curve_ttc"] ?
    mpc["v2x_front_decel_guard_curve_ttc"].as<double>() : 3.0);
  cfg.mpc.v2x_behavior.front_risk_arbitration_enabled =
    mpc["v2x_front_risk_arbitration_enabled"] ?
    mpc["v2x_front_risk_arbitration_enabled"].as<bool>() : false;
  cfg.mpc.v2x_behavior.front_risk_brake_prepare_limit_enabled =
    mpc["v2x_front_risk_brake_prepare_limit_enabled"] ?
    mpc["v2x_front_risk_brake_prepare_limit_enabled"].as<bool>() : false;
  cfg.mpc.v2x_behavior.front_risk_avoid_candidate_limit_enabled =
    mpc["v2x_front_risk_avoid_candidate_limit_enabled"] ?
    mpc["v2x_front_risk_avoid_candidate_limit_enabled"].as<bool>() : true;
  cfg.mpc.v2x_behavior.front_risk_comfort_decel = std::max(
    0.0,
    mpc["v2x_front_risk_comfort_decel"] ?
    mpc["v2x_front_risk_comfort_decel"].as<double>() : 2.0);
  cfg.mpc.v2x_behavior.front_risk_hard_decel = std::max(
    0.0,
    mpc["v2x_front_risk_hard_decel"] ?
    mpc["v2x_front_risk_hard_decel"].as<double>() : 4.0);
  cfg.mpc.v2x_behavior.front_risk_emergency_decel = std::max(
    0.0,
    mpc["v2x_front_risk_emergency_decel"] ?
    mpc["v2x_front_risk_emergency_decel"].as<double>() : 6.0);
  cfg.mpc.v2x_behavior.front_risk_distance_margin = std::max(
    0.0,
    mpc["v2x_front_risk_distance_margin"] ?
    mpc["v2x_front_risk_distance_margin"].as<double>() : 3.0);
  cfg.mpc.v2x_behavior.front_risk_min_closing_speed = std::max(
    0.0,
    mpc["v2x_front_risk_min_closing_speed"] ?
    mpc["v2x_front_risk_min_closing_speed"].as<double>() : 0.5);
  cfg.mpc.v2x_behavior.front_risk_prepare_time = std::max(
    0.0,
    mpc["v2x_front_risk_prepare_time"] ?
    mpc["v2x_front_risk_prepare_time"].as<double>() : 1.5);
  cfg.mpc.v2x_behavior.front_risk_curve_limit_enabled =
    mpc["v2x_front_risk_curve_limit_enabled"] ?
    mpc["v2x_front_risk_curve_limit_enabled"].as<bool>() : false;
  cfg.mpc.v2x_behavior.front_risk_curve_limit_required_decel = std::max(
    0.0,
    mpc["v2x_front_risk_curve_limit_required_decel"] ?
    mpc["v2x_front_risk_curve_limit_required_decel"].as<double>() : 1.2);
  cfg.mpc.v2x_behavior.front_risk_curve_limit_decel = std::max(
    kEps,
    mpc["v2x_front_risk_curve_limit_decel"] ?
    mpc["v2x_front_risk_curve_limit_decel"].as<double>() : 1.4);
  cfg.mpc.v2x_behavior.front_risk_curve_limit_speed_margin = std::max(
    0.0,
    mpc["v2x_front_risk_curve_limit_speed_margin"] ?
    mpc["v2x_front_risk_curve_limit_speed_margin"].as<double>() : 0.5);
  cfg.mpc.v2x_behavior.safety_brake_velocity = std::max(
    0.0,
    mpc["v2x_safety_brake_velocity"] ? mpc["v2x_safety_brake_velocity"].as<double>() : 0.0);
  cfg.mpc.v2x_behavior.overtake_min_gap_width = std::max(
    0.0, mpc["v2x_overtake_min_gap_width"] ? mpc["v2x_overtake_min_gap_width"].as<double>() : 2.0);
  cfg.mpc.v2x_behavior.overtake_max_curvature = std::max(
    0.0,
    mpc["v2x_overtake_max_curvature"] ? mpc["v2x_overtake_max_curvature"].as<double>() : 0.05);
  cfg.mpc.v2x_behavior.overtake_block_inner_curve_pass =
    mpc["v2x_overtake_block_inner_curve_pass"] ?
    mpc["v2x_overtake_block_inner_curve_pass"].as<bool>() : false;
  cfg.mpc.v2x_behavior.overtake_outer_curve_entry_enabled =
    mpc["v2x_overtake_outer_curve_entry_enabled"] ?
    mpc["v2x_overtake_outer_curve_entry_enabled"].as<bool>() : false;
  cfg.mpc.v2x_behavior.overtake_outer_curve_hard_continuation_enabled =
    mpc["v2x_overtake_outer_curve_hard_continuation_enabled"] ?
    mpc["v2x_overtake_outer_curve_hard_continuation_enabled"].as<bool>() : false;
  cfg.mpc.v2x_behavior.overtake_inner_curve_entry_enabled =
    mpc["v2x_overtake_inner_curve_entry_enabled"] ?
    mpc["v2x_overtake_inner_curve_entry_enabled"].as<bool>() : false;
  cfg.mpc.v2x_behavior.overtake_inner_curve_hard_continuation_enabled =
    mpc["v2x_overtake_inner_curve_hard_continuation_enabled"] ?
    mpc["v2x_overtake_inner_curve_hard_continuation_enabled"].as<bool>() : false;
  cfg.mpc.v2x_behavior.overtake_forbidden_curve_lookahead_distance = std::max(
    0.0,
    mpc["v2x_overtake_forbidden_curve_lookahead_distance"] ?
    mpc["v2x_overtake_forbidden_curve_lookahead_distance"].as<double>() : 0.0);
  cfg.mpc.v2x_behavior.overtake_guard_enabled =
    mpc["v2x_overtake_guard_enabled"] ?
    mpc["v2x_overtake_guard_enabled"].as<bool>() : true;
  cfg.mpc.v2x_behavior.overtake_guard_min_gap_width = std::max(
    0.0,
    mpc["v2x_overtake_guard_min_gap_width"] ?
    mpc["v2x_overtake_guard_min_gap_width"].as<double>() : 2.5);
  cfg.mpc.v2x_behavior.overtake_guard_min_gap_points = std::max(
    1,
    mpc["v2x_overtake_guard_min_gap_points"] ?
    mpc["v2x_overtake_guard_min_gap_points"].as<int>() : 3);
  cfg.mpc.v2x_behavior.overtake_guard_min_prepare_distance = std::max(
    0.0,
    mpc["v2x_overtake_guard_min_prepare_distance"] ?
    mpc["v2x_overtake_guard_min_prepare_distance"].as<double>() : 8.0);
  cfg.mpc.v2x_behavior.overtake_guard_max_lateral_shift = std::max(
    0.0,
    mpc["v2x_overtake_guard_max_lateral_shift"] ?
    mpc["v2x_overtake_guard_max_lateral_shift"].as<double>() : 1.2);
  cfg.mpc.v2x_behavior.overtake_guard_reachable_gap_enabled =
    mpc["v2x_overtake_guard_reachable_gap_enabled"] ?
    mpc["v2x_overtake_guard_reachable_gap_enabled"].as<bool>() : false;
  cfg.mpc.v2x_behavior.overtake_guard_max_lateral_accel = std::max(
    0.0,
    mpc["v2x_overtake_guard_max_lateral_accel"] ?
    mpc["v2x_overtake_guard_max_lateral_accel"].as<double>() : 2.0);
  cfg.mpc.v2x_behavior.overtake_guard_min_gap_time = std::max(
    0.0,
    mpc["v2x_overtake_guard_min_gap_time"] ?
    mpc["v2x_overtake_guard_min_gap_time"].as<double>() : 0.8);
  cfg.mpc.v2x_behavior.overtake_guard_min_speed_for_reachable = std::max(
    kEps,
    mpc["v2x_overtake_guard_min_speed_for_reachable"] ?
    mpc["v2x_overtake_guard_min_speed_for_reachable"].as<double>() : 1.0);
  cfg.mpc.v2x_behavior.overtake_guard_min_front_distance = std::max(
    0.0,
    mpc["v2x_overtake_guard_min_front_distance"] ?
    mpc["v2x_overtake_guard_min_front_distance"].as<double>() : 3.0);
  cfg.mpc.v2x_behavior.overtake_continue_min_front_distance = std::min(
    cfg.mpc.v2x_behavior.overtake_guard_min_front_distance,
    std::max(
      0.0,
      mpc["v2x_overtake_continue_min_front_distance"] ?
      mpc["v2x_overtake_continue_min_front_distance"].as<double>() :
      cfg.mpc.v2x_behavior.overtake_guard_min_front_distance));
  cfg.mpc.v2x_behavior.overtake_close_follow_enabled =
    mpc["v2x_overtake_close_follow_enabled"] ?
    mpc["v2x_overtake_close_follow_enabled"].as<bool>() : false;
  cfg.mpc.v2x_behavior.overtake_close_follow_min_front_distance = std::max(
    0.0,
    mpc["v2x_overtake_close_follow_min_front_distance"] ?
    mpc["v2x_overtake_close_follow_min_front_distance"].as<double>() : 1.5);
  cfg.mpc.v2x_behavior.overtake_close_follow_max_closing_speed = std::max(
    0.0,
    mpc["v2x_overtake_close_follow_max_closing_speed"] ?
    mpc["v2x_overtake_close_follow_max_closing_speed"].as<double>() : 0.8);
  cfg.mpc.v2x_behavior.overtake_close_follow_min_side_clearance = std::max(
    0.0,
    mpc["v2x_overtake_close_follow_min_side_clearance"] ?
    mpc["v2x_overtake_close_follow_min_side_clearance"].as<double>() : 2.0);
  cfg.mpc.v2x_behavior.overtake_before_curve_enabled =
    mpc["v2x_overtake_before_curve_enabled"] ?
    mpc["v2x_overtake_before_curve_enabled"].as<bool>() : false;
  cfg.mpc.v2x_behavior.overtake_before_curve_max_front_speed = std::max(
    0.0,
    mpc["v2x_overtake_before_curve_max_front_speed"] ?
    mpc["v2x_overtake_before_curve_max_front_speed"].as<double>() : 8.0);
  cfg.mpc.v2x_behavior.overtake_before_curve_min_speed_advantage = std::max(
    0.0,
    mpc["v2x_overtake_before_curve_min_speed_advantage"] ?
    mpc["v2x_overtake_before_curve_min_speed_advantage"].as<double>() : 1.0);
  cfg.mpc.v2x_behavior.overtake_start_curve_clearance_distance = std::max(
    0.0,
    mpc["v2x_overtake_start_curve_clearance_distance"] ?
    mpc["v2x_overtake_start_curve_clearance_distance"].as<double>() : 0.0);
  cfg.mpc.v2x_behavior.overtake_continue_in_forbidden_enabled =
    mpc["v2x_overtake_continue_in_forbidden_enabled"] ?
    mpc["v2x_overtake_continue_in_forbidden_enabled"].as<bool>() : false;
  cfg.mpc.v2x_behavior.overtake_continue_inner_soft_curve_enabled =
    mpc["v2x_overtake_continue_inner_soft_curve_enabled"] ?
    mpc["v2x_overtake_continue_inner_soft_curve_enabled"].as<bool>() : false;
  cfg.mpc.v2x_behavior.overtake_active_hard_curve_completion_enabled =
    mpc["v2x_overtake_active_hard_curve_completion_enabled"] ?
    mpc["v2x_overtake_active_hard_curve_completion_enabled"].as<bool>() : false;
  cfg.mpc.v2x_behavior.overtake_active_hard_curve_rear_clear_distance = std::max(
    0.0,
    mpc["v2x_overtake_active_hard_curve_rear_clear_distance"] ?
    mpc["v2x_overtake_active_hard_curve_rear_clear_distance"].as<double>() : 0.5);
  cfg.mpc.v2x_behavior.overtake_active_hard_curve_buffer_distance = std::max(
    0.0,
    mpc["v2x_overtake_active_hard_curve_buffer_distance"] ?
    mpc["v2x_overtake_active_hard_curve_buffer_distance"].as<double>() : 0.5);
  cfg.mpc.v2x_behavior.overtake_front_velocity_limit_enabled =
    mpc["v2x_overtake_front_velocity_limit_enabled"] ?
    mpc["v2x_overtake_front_velocity_limit_enabled"].as<bool>() : true;
  cfg.mpc.v2x_behavior.overtake_fallback_ignore_soft_curve_forbidden =
    mpc["v2x_overtake_fallback_ignore_soft_curve_forbidden"] ?
    mpc["v2x_overtake_fallback_ignore_soft_curve_forbidden"].as<bool>() : false;
  cfg.mpc.v2x_behavior.overtake_fallback_min_side_clearance = std::max(
    0.0,
    mpc["v2x_overtake_fallback_min_side_clearance"] ?
    mpc["v2x_overtake_fallback_min_side_clearance"].as<double>() : 1.0);
  cfg.mpc.v2x_behavior.overtake_curve_cooldown_enabled =
    mpc["v2x_overtake_curve_cooldown_enabled"] ?
    mpc["v2x_overtake_curve_cooldown_enabled"].as<bool>() : false;
  cfg.mpc.v2x_behavior.overtake_curve_cooldown_sec = std::max(
    0.0,
    mpc["v2x_overtake_curve_cooldown_sec"] ?
    mpc["v2x_overtake_curve_cooldown_sec"].as<double>() : 0.0);
  cfg.mpc.v2x_behavior.side_overtake_enabled =
    mpc["v2x_side_overtake_enabled"] ?
    mpc["v2x_side_overtake_enabled"].as<bool>() : false;
  cfg.mpc.v2x_behavior.side_overtake_ignore_soft_curve_forbidden =
    mpc["v2x_side_overtake_ignore_soft_curve_forbidden"] ?
    mpc["v2x_side_overtake_ignore_soft_curve_forbidden"].as<bool>() : true;
  cfg.mpc.v2x_behavior.side_overtake_entry_rear_tolerance =
    mpc["v2x_side_overtake_entry_rear_tolerance"] ?
    mpc["v2x_side_overtake_entry_rear_tolerance"].as<double>() : 0.5;
  if (
    !std::isfinite(cfg.mpc.v2x_behavior.side_overtake_entry_rear_tolerance) ||
    cfg.mpc.v2x_behavior.side_overtake_entry_rear_tolerance < 0.0)
  {
    throw std::runtime_error(
            "mpc.v2x_side_overtake_entry_rear_tolerance must be finite and non-negative");
  }
  cfg.mpc.v2x_behavior.overtake_gap_lookahead_distance = std::max(
    0.0,
    mpc["v2x_overtake_gap_lookahead_distance"] ?
    mpc["v2x_overtake_gap_lookahead_distance"].as<double>() : 0.0);
  cfg.mpc.v2x_behavior.overtake_try_both_sides =
    mpc["v2x_overtake_try_both_sides"] ?
    mpc["v2x_overtake_try_both_sides"].as<bool>() : false;
  cfg.mpc.v2x_behavior.overtake_velocity_advantage = std::max(
    0.0,
    mpc["v2x_overtake_velocity_advantage"] ?
    mpc["v2x_overtake_velocity_advantage"].as<double>() : 0.0);
  cfg.mpc.v2x_behavior.overtake_stage_speed_enabled =
    mpc["v2x_overtake_stage_speed_enabled"] ?
    mpc["v2x_overtake_stage_speed_enabled"].as<bool>() : false;
  cfg.mpc.v2x_behavior.overtake_shiftout_adaptive_closing_speed_enabled =
    mpc["v2x_overtake_shiftout_adaptive_closing_speed_enabled"] ?
    mpc["v2x_overtake_shiftout_adaptive_closing_speed_enabled"].as<bool>() : false;
  cfg.mpc.v2x_behavior.overtake_shiftout_max_closing_speed = std::max(
    0.0,
    mpc["v2x_overtake_shiftout_max_closing_speed"] ?
    mpc["v2x_overtake_shiftout_max_closing_speed"].as<double>() : 1.0);
  cfg.mpc.v2x_behavior.overtake_shiftout_min_closing_speed = std::min(
    cfg.mpc.v2x_behavior.overtake_shiftout_max_closing_speed,
    std::max(
      0.0,
      mpc["v2x_overtake_shiftout_min_closing_speed"] ?
      mpc["v2x_overtake_shiftout_min_closing_speed"].as<double>() :
      cfg.mpc.v2x_behavior.overtake_shiftout_max_closing_speed));
  cfg.mpc.v2x_behavior.overtake_pass_unlatched_max_closing_speed = std::min(
    cfg.mpc.v2x_behavior.overtake_shiftout_max_closing_speed,
    std::max(
      0.0,
      mpc["v2x_overtake_pass_unlatched_max_closing_speed"] ?
      mpc["v2x_overtake_pass_unlatched_max_closing_speed"].as<double>() : 0.5));
  cfg.mpc.v2x_behavior.overtake_shiftout_adaptive_min_time_sec = std::max(
    kEps,
    mpc["v2x_overtake_shiftout_adaptive_min_time_sec"] ?
    mpc["v2x_overtake_shiftout_adaptive_min_time_sec"].as<double>() : 0.5);
  cfg.mpc.v2x_behavior.overtake_pass_front_overlap_lateral_clearance = std::max(
    0.0,
    mpc["v2x_overtake_pass_front_overlap_lateral_clearance"] ?
    mpc["v2x_overtake_pass_front_overlap_lateral_clearance"].as<double>() : 0.0);
  cfg.mpc.v2x_behavior.overtake_completion_guard_enabled =
    mpc["v2x_overtake_completion_guard_enabled"] ?
    mpc["v2x_overtake_completion_guard_enabled"].as<bool>() : false;
  cfg.mpc.v2x_behavior.overtake_completion_hard_curvature = std::max(
    kEps,
    mpc["v2x_overtake_completion_hard_curvature"] ?
    mpc["v2x_overtake_completion_hard_curvature"].as<double>() : 0.12);
  cfg.mpc.v2x_behavior.overtake_completion_lookahead_distance = std::max(
    kEps,
    mpc["v2x_overtake_completion_lookahead_distance"] ?
    mpc["v2x_overtake_completion_lookahead_distance"].as<double>() : 80.0);
  cfg.mpc.v2x_behavior.overtake_completion_curve_buffer_distance = std::max(
    0.0,
    mpc["v2x_overtake_completion_curve_buffer_distance"] ?
    mpc["v2x_overtake_completion_curve_buffer_distance"].as<double>() : 2.0);
  cfg.mpc.v2x_behavior.overtake_completion_merge_buffer_distance = std::max(
    0.0,
    mpc["v2x_overtake_completion_merge_buffer_distance"] ?
    mpc["v2x_overtake_completion_merge_buffer_distance"].as<double>() : 3.0);
  cfg.mpc.v2x_behavior.overtake_completion_min_relative_speed = std::max(
    kEps,
    mpc["v2x_overtake_completion_min_relative_speed"] ?
    mpc["v2x_overtake_completion_min_relative_speed"].as<double>() : 0.5);
  cfg.mpc.v2x_behavior.moving_front_speed_threshold = std::max(
    0.0,
    mpc["v2x_moving_front_speed_threshold"] ?
    mpc["v2x_moving_front_speed_threshold"].as<double>() : 1.0);
  cfg.mpc.v2x_behavior.moving_follow_speed_margin = std::max(
    0.0,
    mpc["v2x_moving_follow_speed_margin"] ?
    mpc["v2x_moving_follow_speed_margin"].as<double>() : 2.0);
  cfg.mpc.v2x_behavior.moving_follow_hard_distance = std::max(
    0.0,
    mpc["v2x_moving_follow_hard_distance"] ?
    mpc["v2x_moving_follow_hard_distance"].as<double>() : 0.0);
  cfg.mpc.v2x_behavior.moving_follow_target_distance = std::max(
    cfg.mpc.v2x_behavior.moving_follow_hard_distance,
    mpc["v2x_moving_follow_target_distance"] ?
    mpc["v2x_moving_follow_target_distance"].as<double>() : 0.0);
  cfg.mpc.v2x_behavior.moving_follow_recovery_speed_margin = std::max(
    0.0,
    mpc["v2x_moving_follow_recovery_speed_margin"] ?
    mpc["v2x_moving_follow_recovery_speed_margin"].as<double>() : 0.0);
  cfg.mpc.v2x_behavior.moving_follow_distance_gain = std::max(
    0.0,
    mpc["v2x_moving_follow_distance_gain"] ?
    mpc["v2x_moving_follow_distance_gain"].as<double>() : 0.0);
  cfg.mpc.v2x_behavior.moving_safety_brake_distance = std::max(
    0.0,
    mpc["v2x_moving_safety_brake_distance"] ?
    mpc["v2x_moving_safety_brake_distance"].as<double>() : 1.5);
  cfg.mpc.v2x_behavior.moving_safety_brake_margin = std::max(
    0.0,
    mpc["v2x_moving_safety_brake_margin"] ?
    mpc["v2x_moving_safety_brake_margin"].as<double>() : 1.0);
  cfg.mpc.v2x_behavior.moving_safety_brake_time_headway = std::max(
    0.0,
    mpc["v2x_moving_safety_brake_time_headway"] ?
    mpc["v2x_moving_safety_brake_time_headway"].as<double>() : 0.3);
  cfg.mpc.v2x_behavior.start_grid_grace_time = std::max(
    0.0,
    mpc["v2x_start_grid_grace_time"] ?
    mpc["v2x_start_grid_grace_time"].as<double>() : 0.0);
  cfg.mpc.v2x_behavior.require_gap_for_overtake =
    mpc["v2x_require_gap_for_overtake"] ?
    mpc["v2x_require_gap_for_overtake"].as<bool>() : true;
  cfg.mpc.v2x_behavior.low_speed_avoidance_enabled =
    mpc["v2x_low_speed_avoidance_enabled"] ?
    mpc["v2x_low_speed_avoidance_enabled"].as<bool>() : false;
  cfg.mpc.v2x_behavior.low_speed_avoidance_ignore_soft_curve_forbidden =
    mpc["v2x_low_speed_avoidance_ignore_soft_curve_forbidden"] ?
    mpc["v2x_low_speed_avoidance_ignore_soft_curve_forbidden"].as<bool>() : false;
  cfg.mpc.v2x_behavior.low_speed_local_path_enabled =
    mpc["use_v2x_local_path_planner"] ?
    mpc["use_v2x_local_path_planner"].as<bool>() : false;
  cfg.mpc.v2x_behavior.low_speed_avoidance_distance = std::max(
    0.0,
    mpc["v2x_low_speed_avoidance_distance"] ?
    mpc["v2x_low_speed_avoidance_distance"].as<double>() : 8.0);
  cfg.mpc.v2x_behavior.low_speed_avoidance_lookahead_distance = std::max(
    0.0,
    mpc["v2x_low_speed_avoidance_lookahead_distance"] ?
    mpc["v2x_low_speed_avoidance_lookahead_distance"].as<double>() : 18.0);
  cfg.mpc.v2x_behavior.low_speed_avoidance_velocity = std::max(
    0.0,
    mpc["v2x_low_speed_avoidance_velocity"] ?
    mpc["v2x_low_speed_avoidance_velocity"].as<double>() : 2.0);
  cfg.mpc.v2x_behavior.low_speed_avoidance_shift_velocity = std::max(
    0.0,
    mpc["v2x_low_speed_avoidance_shift_velocity"] ?
    mpc["v2x_low_speed_avoidance_shift_velocity"].as<double>() : 1.0);
  cfg.mpc.v2x_behavior.low_speed_avoidance_shift_lateral_gain = std::max(
    0.0,
    mpc["v2x_low_speed_avoidance_shift_lateral_gain"] ?
    mpc["v2x_low_speed_avoidance_shift_lateral_gain"].as<double>() : 0.4);
  cfg.mpc.v2x_behavior.low_speed_avoidance_shift_heading_gain = std::max(
    0.0,
    mpc["v2x_low_speed_avoidance_shift_heading_gain"] ?
    mpc["v2x_low_speed_avoidance_shift_heading_gain"].as<double>() : 1.3);
  cfg.mpc.v2x_behavior.low_speed_avoidance_shift_lateral_tolerance = std::max(
    0.0,
    mpc["v2x_low_speed_avoidance_shift_lateral_tolerance"] ?
    mpc["v2x_low_speed_avoidance_shift_lateral_tolerance"].as<double>() : 0.4);
  cfg.mpc.v2x_behavior.low_speed_avoidance_shift_heading_tolerance = std::max(
    0.0,
    mpc["v2x_low_speed_avoidance_shift_heading_tolerance"] ?
    mpc["v2x_low_speed_avoidance_shift_heading_tolerance"].as<double>() : 0.2);
  cfg.mpc.v2x_behavior.low_speed_avoidance_shift_clear_hold_sec = std::max(
    0.0,
    mpc["v2x_low_speed_avoidance_shift_clear_hold_sec"] ?
    mpc["v2x_low_speed_avoidance_shift_clear_hold_sec"].as<double>() : 2.0);
  cfg.mpc.v2x_behavior.low_speed_avoidance_max_front_speed = std::max(
    0.0,
    mpc["v2x_low_speed_avoidance_max_front_speed"] ?
    mpc["v2x_low_speed_avoidance_max_front_speed"].as<double>() : 1.0);
  cfg.mpc.v2x_behavior.low_speed_avoidance_min_prepare_distance = std::max(
    0.0,
    mpc["v2x_low_speed_avoidance_min_prepare_distance"] ?
    mpc["v2x_low_speed_avoidance_min_prepare_distance"].as<double>() : 0.0);
  cfg.mpc.v2x_behavior.low_speed_avoidance_min_gap_width = std::max(
    0.0,
    mpc["v2x_low_speed_avoidance_min_gap_width"] ?
    mpc["v2x_low_speed_avoidance_min_gap_width"].as<double>() : 1.5);
  cfg.mpc.v2x_behavior.low_speed_avoidance_min_gap_points = std::max(
    1,
    mpc["v2x_low_speed_avoidance_min_gap_points"] ?
    mpc["v2x_low_speed_avoidance_min_gap_points"].as<int>() : 2);
  cfg.mpc.v2x_behavior.low_speed_avoidance_clear_distance = std::max(
    0.0,
    mpc["v2x_low_speed_avoidance_clear_distance"] ?
    mpc["v2x_low_speed_avoidance_clear_distance"].as<double>() : 8.0);
  cfg.mpc.v2x_behavior.low_speed_avoidance_stall_speed = std::max(
    0.0,
    mpc["v2x_low_speed_avoidance_stall_speed"] ?
    mpc["v2x_low_speed_avoidance_stall_speed"].as<double>() : 0.15);
  cfg.mpc.v2x_behavior.low_speed_avoidance_stall_timeout_sec = std::max(
    0.01,
    mpc["v2x_low_speed_avoidance_stall_timeout_sec"] ?
    mpc["v2x_low_speed_avoidance_stall_timeout_sec"].as<double>() : 1.5);
  cfg.mpc.v2x_behavior.low_speed_avoidance_stall_cooldown_sec = std::max(
    0.0,
    mpc["v2x_low_speed_avoidance_stall_cooldown_sec"] ?
    mpc["v2x_low_speed_avoidance_stall_cooldown_sec"].as<double>() : 3.0);
  cfg.mpc.v2x_behavior.low_speed_avoidance_stall_max_observation_gap_sec = std::max(
    0.01,
    mpc["v2x_low_speed_avoidance_stall_max_observation_gap_sec"] ?
    mpc["v2x_low_speed_avoidance_stall_max_observation_gap_sec"].as<double>() : 0.2);
  cfg.mpc.v2x_behavior.low_speed_local_path_pass_clearance = std::max(
    0.0,
    mpc["v2x_local_path_pass_clearance"] ?
    mpc["v2x_local_path_pass_clearance"].as<double>() : 3.0);
  cfg.mpc.v2x_behavior.low_speed_local_path_return_distance = std::max(
    0.5,
    mpc["v2x_local_path_return_distance"] ?
    mpc["v2x_local_path_return_distance"].as<double>() : 6.0);
  cfg.mpc.v2x_behavior.low_speed_local_path_invert_target =
    mpc["v2x_local_path_invert_target"] ?
    mpc["v2x_local_path_invert_target"].as<bool>() : false;
  cfg.mpc.v2x_behavior.state_hold_time = std::max(
    0.0, mpc["v2x_state_hold_time"] ? mpc["v2x_state_hold_time"].as<double>() : 0.5);
  if (mpc["v2x_overtake_forbidden_wp_ranges"]) {
    for (const auto & item : mpc["v2x_overtake_forbidden_wp_ranges"]) {
      if (!item.IsSequence() || item.size() < 2) {
        continue;
      }
      cfg.mpc.v2x_behavior.overtake_forbidden_wp_ranges.emplace_back(
        item[0].as<int>(), item[1].as<int>());
    }
  }
  cfg.mpc.use_max_kappa_pred = mpc["use_max_kappa_pred"].as<bool>();
  return cfg;
}

class ReferenceVelocityConfigulator
{
public:
  explicit ReferenceVelocityConfigulator(rclcpp::Node * node, const std::string & ref_vel_config_path)
  : node_(node)
  {
    const YAML::Node root = YAML::LoadFile(ref_vel_config_path);
    const YAML::Node sections = root["ref_vel_configulator"];
    for (const auto & item : sections) {
      const std::string name = item.first.as<std::string>();
      const int wp_id = item.second["wp_id"].as<int>();
      const double ref_vel = item.second["ref_vel"].as<double>();
      wp_id_ref_vel_[wp_id] = ref_vel;
      node_->declare_parameter("ref_vel/" + name + "/wp_id", wp_id);
      node_->declare_parameter("ref_vel/" + name + "/ref_vel", ref_vel);
    }
    node_->declare_parameter("ref_vel/save", false);
  }

  double get_ref_vel(const int current_wp_id) const
  {
    if (wp_id_ref_vel_.empty()) {
      throw std::runtime_error("ref_vel_configulator has no sections");
    }
    std::vector<int> keys;
    keys.reserve(wp_id_ref_vel_.size());
    for (const auto & kv : wp_id_ref_vel_) {
      keys.push_back(kv.first);
    }
    for (std::size_t i = 0; i < keys.size(); ++i) {
      const int start = keys[i];
      const int end = keys[(i + 1) % keys.size()];
      const double target_speed = wp_id_ref_vel_.at(start);
      if (start <= end) {
        if (start <= current_wp_id && current_wp_id < end) {
          return target_speed;
        }
      } else if (current_wp_id >= start || current_wp_id < end) {
        return target_speed;
      }
    }
    throw std::runtime_error("Current waypoint ID does not fall into any section.");
  }

private:
  rclcpp::Node * node_{};
  std::map<int, double> wp_id_ref_vel_;
};

class MPCControllerCpp : public rclcpp::Node
{
public:
  MPCControllerCpp(const std::string & config_path, const std::optional<std::string> & ref_vel_path)
  : Node("mpc_controller"), config_path_(config_path), ref_vel_config_path_(ref_vel_path)
  {
    declare_parameter("use_boost_acceleration", false);
    declare_parameter("use_obstacle_avoidance", false);
    declare_parameter("use_stats", false);
    use_sim_time_ = get_parameter("use_sim_time").as_bool();
    use_bug_acc_ = get_parameter("use_boost_acceleration").as_bool();
    use_obstacle_avoidance_ = get_parameter("use_obstacle_avoidance").as_bool();
    use_stats_ = get_parameter("use_stats").as_bool();

    cfg_ = load_config(config_path_);
    mpc_cfg_ = cfg_.mpc;
    mpc_cfg_.steer_rate_max = mpc_cfg_.steer_rate_max / mpc_cfg_.steering_tire_angle_gain_var;
    stuck_recovery_core_ =
      std::make_unique<stuck_recovery::StuckRecoveryCore>(cfg_.stuck_recovery.core);
    stuck_recovery_actuation_io_enabled_ =
      cfg_.stuck_recovery.core.enabled && !cfg_.stuck_recovery.core.shadow_mode &&
      (!cfg_.stuck_recovery.core.simulation_only || use_sim_time_) &&
      cfg_.stuck_recovery.reverse_actuation_enabled && !use_bug_acc_;
    awsim_boost_guard_ = std::make_unique<awsim_boost::StartDashGuard>(cfg_.awsim_boost);
    awsim_boost_io_enabled_ =
      use_sim_time_ && cfg_.awsim_boost.enabled &&
      cfg_.awsim_boost.mode == awsim_boost::Mode::StartOnce && !use_bug_acc_;
    awsim_state_tracking_enabled_ =
      use_sim_time_ &&
      (awsim_boost_io_enabled_ ||
      cfg_.stuck_recovery.core.enabled ||
      (mpc_cfg_.domain_start_v_max_applied &&
      mpc_cfg_.domain_start_v_max_duration > 0.0) ||
      (mpc_cfg_.v2x_behavior.enabled &&
      mpc_cfg_.v2x_behavior.start_grid_grace_time > 0.0));

    create_map_ref_path_car_mpc();
    setup_parameters_callback();
    setup_pub_sub();
    if (ref_vel_config_path_.has_value()) {
      ref_vel_configulator_ =
        std::make_unique<ReferenceVelocityConfigulator>(this, ref_vel_config_path_.value());
    }

    using namespace std::literals::chrono_literals;
    const auto control_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / std::max(1.0, mpc_cfg_.control_rate)));
    control_timer_ = create_wall_timer(
      control_period, std::bind(&MPCControllerCpp::control, this));
    ref_vel_marker_timer_ = create_wall_timer(10ms, [this]() {
      if (ref_vel_marker_pub_) {
        ref_vel_marker_pub_->publish(MarkerArray{});
        section_marker_pub_->publish(MarkerArray{});
      }
    });

    if (use_sim_time_) {
      RCLCPP_WARN(get_logger(), "use_sim_time is enabled!");
    }
    if (use_bug_acc_) {
      RCLCPP_WARN(get_logger(), "USE_BUG_ACC is enabled!");
    }
    if (cfg_.stuck_recovery.domain_enabled_applied) {
      RCLCPP_INFO(
        get_logger(),
        "Stuck recovery domain_enabled applied: ROS_DOMAIN_ID=%d, enabled=%s",
        cfg_.stuck_recovery.domain_enabled_domain,
        cfg_.stuck_recovery.core.enabled ? "true" : "false");
    }
    RCLCPP_INFO(
      get_logger(),
      "Stuck recovery: mode=%s, reverse_actuation=%s, simulation_only=%s",
      stuck_recovery::to_string(stuck_recovery_core_->execution_mode(use_sim_time_)),
      stuck_recovery_actuation_io_enabled_ ? "enabled" : "disabled",
      cfg_.stuck_recovery.core.simulation_only ? "true" : "false");
    RCLCPP_INFO(
      get_logger(),
      "Stuck recovery coordinated reverse: enabled=%s, stop=%.2f s, "
      "front_speed<=%.2f m/s, solver_evidence_free=%s/%.2f s, "
      "solver_reverse_only_heading>=%.2f rad, steering_samples=%zu, "
      "rejoin_solver_wait=%.2f s, recovery_mpc=%s/%zux%.2f m/beam=%zu, "
      "aggressive_sim=%s/retry=%.2f s/force_rejoin=%zu",
      cfg_.stuck_recovery.core.detector.coordinated_stop_recovery_enabled ? "true" : "false",
      cfg_.stuck_recovery.core.detector.coordinated_stop_duration_sec,
      cfg_.stuck_recovery.coordinated_stop_front_speed_mps,
      cfg_.stuck_recovery.core.detector.solver_evidence_free_recovery_enabled ?
      "true" : "false",
      cfg_.stuck_recovery.core.detector.solver_evidence_free_duration_sec,
      cfg_.stuck_recovery.solver_reverse_only_heading_error_rad,
      cfg_.stuck_recovery.side_escape_steering_samples,
      cfg_.stuck_recovery.core.supervisor.rejoin_solver_recovery_timeout_sec,
      cfg_.stuck_recovery.recovery_mpc_enabled ? "enabled" : "disabled",
      cfg_.stuck_recovery.recovery_mpc_config.horizon_steps,
      cfg_.stuck_recovery.recovery_mpc_config.travel_step_m,
      cfg_.stuck_recovery.recovery_mpc_config.beam_width,
      cfg_.stuck_recovery.core.supervisor.aggressive_sim_recovery_enabled ? "true" : "false",
      cfg_.stuck_recovery.core.supervisor.aggressive_retry_delay_sec,
      cfg_.stuck_recovery.aggressive_force_rejoin_after_retries);
    if (
      cfg_.stuck_recovery.core.enabled && !cfg_.stuck_recovery.core.shadow_mode &&
      !stuck_recovery_actuation_io_enabled_)
    {
      RCLCPP_WARN(
        get_logger(),
        "Stuck recovery takeover is configured but reverse actuation is safety-blocked; "
        "confirmed candidates will be held stopped without a gear command");
    }
    if (cfg_.awsim_boost.domain_enabled_applied) {
      RCLCPP_INFO(
        get_logger(),
        "AWSIM Boost domain_enabled applied: ROS_DOMAIN_ID=%d, enabled=%s",
        cfg_.awsim_boost.domain_enabled_domain,
        cfg_.awsim_boost.enabled ? "true" : "false");
    }
    if (awsim_boost_io_enabled_) {
      RCLCPP_INFO(
        get_logger(),
        "AWSIM 2026 boost enabled: mode=%s, trigger=%s, motion=%.2f..%.2f m/s, "
        "motion_timeout=%.2f s, status_timeout=%.2f s, confirmation_timeout=%.2f s",
        awsim_boost::to_string(cfg_.awsim_boost.mode),
        awsim_boost::to_string(cfg_.awsim_boost.trigger),
        cfg_.awsim_boost.motion_speed_threshold_mps, cfg_.awsim_boost.max_trigger_speed_mps,
        cfg_.awsim_boost.motion_trigger_timeout_sec, cfg_.awsim_boost.status_timeout_sec,
        cfg_.awsim_boost.confirmation_timeout_sec);
    } else if (cfg_.awsim_boost.enabled && !use_sim_time_) {
      RCLCPP_WARN(get_logger(), "AWSIM boost is configured but disabled outside simulation.");
    } else if (cfg_.awsim_boost.enabled && use_bug_acc_) {
      RCLCPP_WARN(
        get_logger(),
        "AWSIM 2026 boost is disabled while legacy use_boost_acceleration is active.");
    } else if (!cfg_.awsim_boost.enabled) {
      RCLCPP_INFO(get_logger(), "AWSIM 2026 boost is disabled by configuration.");
    } else if (cfg_.awsim_boost.mode == awsim_boost::Mode::Disabled) {
      RCLCPP_INFO(get_logger(), "AWSIM 2026 boost mode is disabled.");
    }
    if (use_obstacle_avoidance_) {
      RCLCPP_WARN(get_logger(), "USE_OBSTACLE_AVOIDANCE is enabled!");
    }
    if (cfg_.reference_path.domain_csv_path_applied) {
      RCLCPP_INFO(
        get_logger(), "reference_path.domain_csv_path applied: ROS_DOMAIN_ID=%d, csv_path=%s",
        cfg_.reference_path.domain_csv_path_domain, cfg_.reference_path.csv_path.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "reference_path.csv_path: %s", cfg_.reference_path.csv_path.c_str());
    }
    if (mpc_cfg_.domain_v_max_applied) {
      RCLCPP_INFO(
        get_logger(), "domain_v_max applied: ROS_DOMAIN_ID=%d, v_max=%.2f km/h",
        mpc_cfg_.ros_domain_id, mpc_cfg_.v_max_kmh);
    } else if (mpc_cfg_.ros_domain_id >= 0) {
      RCLCPP_INFO(
        get_logger(), "MPC v_max: ROS_DOMAIN_ID=%d, v_max=%.2f km/h",
        mpc_cfg_.ros_domain_id, mpc_cfg_.v_max_kmh);
    } else {
      RCLCPP_INFO(get_logger(), "MPC v_max: v_max=%.2f km/h", mpc_cfg_.v_max_kmh);
    }
    RCLCPP_INFO(
      get_logger(), "MPC wp_id_offset: normal=%d, low=%d below %.2f km/h",
      mpc_cfg_.wp_id_offset, mpc_cfg_.wp_id_low_offset, mpc_cfg_.wp_id_low_speed_kmh);
    if (mpc_cfg_.domain_a_max_applied) {
      RCLCPP_INFO(
        get_logger(), "domain_a_max applied: ROS_DOMAIN_ID=%d, a_max=%.2f m/s^2",
        mpc_cfg_.ros_domain_id, mpc_cfg_.a_max);
    } else if (mpc_cfg_.ros_domain_id >= 0) {
      RCLCPP_INFO(
        get_logger(), "MPC a_max: ROS_DOMAIN_ID=%d, a_max=%.2f m/s^2",
        mpc_cfg_.ros_domain_id, mpc_cfg_.a_max);
    } else {
      RCLCPP_INFO(get_logger(), "MPC a_max: a_max=%.2f m/s^2", mpc_cfg_.a_max);
    }
    if (mpc_cfg_.domain_start_v_max_applied) {
      RCLCPP_INFO(
        get_logger(),
        "domain_start_v_max applied: ROS_DOMAIN_ID=%d, v_max=%.2f km/h for %.2f sec",
        mpc_cfg_.ros_domain_id, mpc_cfg_.domain_start_v_max_kmh,
        mpc_cfg_.domain_start_v_max_duration);
      if (!use_sim_time_) {
        RCLCPP_WARN(
          get_logger(),
          "domain_start_v_max is configured but disabled outside simulation because "
          "/awsim/state Start is unavailable");
      }
    }
    if (mpc_cfg_.v2x_gap.enabled) {
      RCLCPP_WARN(get_logger(), "USE_V2X_GAP_PLANNER is enabled!");
    }
    RCLCPP_INFO(
      get_logger(),
      "MPC solver fallback steering: hold=%d cycles, neutralize_rate=%.3f rad/s",
      mpc_cfg_.solver_failure_steering_hold_cycles, mpc_cfg_.steer_rate_max);
    if (mpc_cfg_.v2x_behavior.enabled) {
      RCLCPP_WARN(get_logger(), "USE_V2X_BEHAVIOR_FSM is enabled!");
      if (mpc_cfg_.v2x_behavior.debug_log_enabled) {
        RCLCPP_INFO(
          get_logger(), "V2X behavior debug log is enabled: period=%.2f sec",
          mpc_cfg_.v2x_behavior.debug_log_period_sec);
      }
      if (mpc_cfg_.v2x_behavior.front_progress_detection_enabled) {
        RCLCPP_INFO(
          get_logger(),
          "V2X common-progress front detection is enabled: lookahead=%.2f m, "
          "lookbehind=%.2f m",
          mpc_cfg_.v2x_behavior.front_progress_detection_distance,
          mpc_cfg_.v2x_behavior.front_progress_lookbehind_distance);
      }
    }
    if (mpc_cfg_.v2x_behavior.overtake_line.enabled) {
      RCLCPP_INFO(
        get_logger(),
        "V2X overtake line is enabled: offset=%.2f, shift=%.2f, pass=%.2f, "
        "return=%.2f, bias=%.2f, recovery_v=%.2f, stall=%.2f m/s/%.2f s, "
        "timeout=%.2f s, solver_cooldown=%.2f s, solver_healthy=%d cycles",
        mpc_cfg_.v2x_behavior.overtake_line.lateral_offset,
        mpc_cfg_.v2x_behavior.overtake_line.shift_distance,
        mpc_cfg_.v2x_behavior.overtake_line.pass_distance,
        mpc_cfg_.v2x_behavior.overtake_line.return_distance,
        mpc_cfg_.v2x_behavior.overtake_line.target_bias,
        mpc_cfg_.v2x_behavior.overtake_line.recovery_velocity,
        mpc_cfg_.v2x_behavior.overtake_line.recovery_stall_speed,
        mpc_cfg_.v2x_behavior.overtake_line.recovery_stall_timeout_sec,
        mpc_cfg_.v2x_behavior.overtake_line.recovery_timeout_sec,
        mpc_cfg_.v2x_behavior.overtake_line.solver_cooldown_sec,
        mpc_cfg_.v2x_behavior.overtake_line.solver_recovery_success_cycles);
    }
    if (mpc_cfg_.v2x_behavior.low_speed_avoidance_enabled) {
      RCLCPP_INFO(
        get_logger(),
        "V2X low-speed pass: side=%s, ramp_ratio=%.2f, shift_gains=%.2f/%.2f, "
        "stall=%.2f m/s/%.2f s, cooldown=%.2f s, max_gap=%.2f s",
        mpc_cfg_.v2x_gap.low_speed_pass_side.c_str(),
        mpc_cfg_.v2x_gap.low_speed_pass_ramp_ratio,
        mpc_cfg_.v2x_behavior.low_speed_avoidance_shift_lateral_gain,
        mpc_cfg_.v2x_behavior.low_speed_avoidance_shift_heading_gain,
        mpc_cfg_.v2x_behavior.low_speed_avoidance_stall_speed,
        mpc_cfg_.v2x_behavior.low_speed_avoidance_stall_timeout_sec,
        mpc_cfg_.v2x_behavior.low_speed_avoidance_stall_cooldown_sec,
        mpc_cfg_.v2x_behavior.low_speed_avoidance_stall_max_observation_gap_sec);
      if (mpc_cfg_.v2x_behavior.low_speed_local_path_enabled) {
        RCLCPP_INFO(
          get_logger(), "V2X low-speed local path planner: pass_clearance=%.2f, return_distance=%.2f",
          mpc_cfg_.v2x_behavior.low_speed_local_path_pass_clearance,
          mpc_cfg_.v2x_behavior.low_speed_local_path_return_distance);
        if (mpc_cfg_.v2x_behavior.low_speed_local_path_invert_target) {
          RCLCPP_WARN(get_logger(), "V2X low-speed local path target sign is inverted before MPC.");
        }
      }
    }
  }

  ~MPCControllerCpp() override
  {
    publish_zero_command();
  }

private:
  std::string in_pkg_share(const std::string & path) const
  {
    const std::filesystem::path p(config_path_);
    const std::filesystem::path config_dir = p.parent_path();
    const std::filesystem::path share_dir = config_dir.parent_path();
    return (share_dir / path).string();
  }

  void create_map_ref_path_car_mpc()
  {
    map_ = std::make_unique<Map>(in_pkg_share(cfg_.map_yaml_path));
    if (cfg_.stuck_recovery.core.enabled) {
      if (
        map_->origin.size() < 3U || std::abs(map_->origin.at(2)) > kEps ||
        (map_->negate != 0 && map_->negate != 1) ||
        !std::isfinite(map_->threshold_free) || !std::isfinite(map_->threshold_occupied) ||
        map_->threshold_free < 0.0 || map_->threshold_occupied > 1.0 ||
        map_->threshold_free >= map_->threshold_occupied)
      {
        throw std::runtime_error(
                "stuck_recovery supports only yaw-zero maps with valid ROS occupancy thresholds");
      }
      recovery_grid_ = std::make_unique<recovery_footprint::OccupancyGrid>();
      recovery_grid_->width = static_cast<std::size_t>(map_->width);
      recovery_grid_->height = static_cast<std::size_t>(map_->height);
      recovery_grid_->resolution_m = map_->resolution;
      recovery_grid_->origin_x_m = map_->origin.at(0);
      recovery_grid_->origin_y_m = map_->origin.at(1);
      recovery_grid_->y_axis = recovery_footprint::YAxisConvention::RowZeroAtMaximumY;
      recovery_grid_->cells.reserve(recovery_grid_->width * recovery_grid_->height);
      for (int row = 0; row < map_->height; ++row) {
        for (int column = 0; column < map_->width; ++column) {
          const double normalized_value = map_->raw_normalized_data.at<double>(row, column);
          const double occupancy_probability =
            map_->negate == 0 ? 1.0 - normalized_value : normalized_value;
          if (occupancy_probability > map_->threshold_occupied) {
            recovery_grid_->cells.push_back(recovery_footprint::CellState::Occupied);
          } else if (occupancy_probability < map_->threshold_free) {
            recovery_grid_->cells.push_back(recovery_footprint::CellState::Free);
          } else {
            recovery_grid_->cells.push_back(recovery_footprint::CellState::Unknown);
          }
        }
      }
      recovery_footprint_ = recovery_footprint::FootprintExtents{
        cfg_.stuck_recovery.front_extent_m,
        cfg_.stuck_recovery.rear_extent_m,
        cfg_.stuck_recovery.left_extent_m,
        cfg_.stuck_recovery.right_extent_m,
        cfg_.stuck_recovery.footprint_margin_m};
      if (!recovery_grid_->valid() || !recovery_footprint_.valid()) {
        throw std::runtime_error("stuck_recovery map or footprint configuration is invalid");
      }
    }
    std::vector<double> wp_x;
    std::vector<double> wp_y;
    if (!cfg_.reference_path.csv_path.empty()) {
      std::tie(wp_x, wp_y) = load_ref_path(
        in_pkg_share(cfg_.reference_path.csv_path), cfg_.reference_path.circular);
    } else {
      std::tie(wp_x, wp_y) = load_waypoints(in_pkg_share(cfg_.waypoints_csv_path));
    }
    reference_path_ = std::make_unique<ReferencePath>(
      map_.get(), wp_x, wp_y, cfg_.reference_path.resolution, cfg_.reference_path.smoothing_distance,
      cfg_.reference_path.max_width, cfg_.reference_path.circular);
    car_ = std::make_unique<BicycleModel>(
      reference_path_.get(), cfg_.bicycle_length, cfg_.bicycle_width, mpc_cfg_.safety_margin_scale,
      1.0 / cfg_.mpc.control_rate, mpc_cfg_.min_linearization_speed_mps);
    mpc_ = std::make_unique<MPC>(
      car_.get(), mpc_cfg_, use_obstacle_avoidance_, cfg_.reference_path.use_path_constraints_topic);
    const bool mpc_requires_v2x_planner =
      mpc_cfg_.v2x_gap.enabled || mpc_cfg_.v2x_behavior.enabled;
    if (mpc_requires_v2x_planner || cfg_.stuck_recovery.core.enabled) {
      auto planner_cfg = mpc_cfg_.v2x_gap;
      if (mpc_cfg_.v2x_behavior.enabled) {
        planner_cfg.enabled = true;
      }
      v2x_gap_planner_ = std::make_unique<V2XGapPlanner>(
        planner_cfg, cfg_.stuck_recovery.core.enabled);
      if (mpc_requires_v2x_planner) {
        mpc_->set_gap_planner(v2x_gap_planner_.get());
      }
    }
    if (!reference_path_->compute_speed_profile(
        mpc_cfg_.a_min, mpc_cfg_.a_max, 0.0,
        use_bug_acc_ ? kmh_to_m_per_sec(40.0) : mpc_cfg_.v_max, mpc_cfg_.ay_max))
    {
      throw std::runtime_error("Failed to compute the initial reference speed profile");
    }
  }

  void setup_parameters_callback()
  {
    declare_parameter("v_max", cfg_.mpc.v_max * 3.6);
    declare_parameter("steering_tire_angle_gain_var", cfg_.mpc.steering_tire_angle_gain_var);
    declare_parameter("Q0", cfg_.mpc.Q[0]);
    declare_parameter("Q1", cfg_.mpc.Q[1]);
    declare_parameter("Q2", cfg_.mpc.Q[2]);
    declare_parameter("R0", cfg_.mpc.R[0]);
    declare_parameter("R1", cfg_.mpc.R[1]);
    declare_parameter("QN0", cfg_.mpc.QN[0]);
    declare_parameter("QN1", cfg_.mpc.QN[1]);
    declare_parameter("QN2", cfg_.mpc.QN[2]);
    declare_parameter("ay_max", cfg_.mpc.ay_max);
    declare_parameter("accel_low_pass_gain", cfg_.mpc.accel_low_pass_gain);
    declare_parameter("steer_low_pass_gain", cfg_.mpc.steer_low_pass_gain);
    declare_parameter("wp_id_offset", cfg_.mpc.wp_id_offset);
    declare_parameter("wp_id_low_offset", cfg_.mpc.wp_id_low_offset);
    declare_parameter("wp_id_low_speed", cfg_.mpc.wp_id_low_speed_kmh);

    param_callback_handle_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto & param : params) {
          const auto & name = param.get_name();
          const bool is_nonnegative_weight =
            name == "Q0" || name == "Q1" || name == "Q2" || name == "R0" ||
            name == "R1" || name == "QN0" || name == "QN1" || name == "QN2";
          const bool is_checked_double =
            name == "v_max" || name == "steering_tire_angle_gain_var" ||
            name == "ay_max" || name == "accel_low_pass_gain" ||
            name == "steer_low_pass_gain" || name == "wp_id_low_speed" ||
            is_nonnegative_weight;
          if (!is_checked_double) {
            continue;
          }
          if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
            result.successful = false;
            result.reason = name + " must be a double";
            return result;
          }
          const double value = param.as_double();
          const bool invalid =
            !std::isfinite(value) ||
            ((name == "v_max" || name == "wp_id_low_speed" || is_nonnegative_weight) &&
            value < 0.0) ||
            ((name == "steering_tire_angle_gain_var" || name == "ay_max") && value <= 0.0) ||
            ((name == "accel_low_pass_gain" || name == "steer_low_pass_gain") &&
            (value < 0.0 || value > 1.0));
          if (invalid) {
            result.successful = false;
            result.reason = name + " is outside its finite safety range";
            return result;
          }
        }
        for (const auto & param : params) {
          const auto & name = param.get_name();
          if (name == "v_max" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            mpc_cfg_.global_v_max_kmh = param.as_double();
            mpc_cfg_.global_v_max = kmh_to_m_per_sec(param.as_double());
            mpc_cfg_.v_max_kmh = param.as_double();
            const double v_mps = kmh_to_m_per_sec(param.as_double());
            mpc_cfg_.v_max = v_mps;
            mpc_->update_v_max(v_mps);
            reference_path_->set_v_ref(std::vector<double>(reference_path_->waypoints.size(), v_mps));
          } else if (
            name == "steering_tire_angle_gain_var" &&
            param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            mpc_cfg_.steering_tire_angle_gain_var = param.as_double();
          } else if (name == "Q0") {
            mpc_->cfg.Q[0] = param.as_double();
          } else if (name == "Q1") {
            mpc_->cfg.Q[1] = param.as_double();
          } else if (name == "Q2") {
            mpc_->cfg.Q[2] = param.as_double();
          } else if (name == "R0") {
            mpc_->cfg.R[0] = param.as_double();
          } else if (name == "R1") {
            mpc_->cfg.R[1] = param.as_double();
          } else if (name == "QN0") {
            mpc_->cfg.QN[0] = param.as_double();
          } else if (name == "QN1") {
            mpc_->cfg.QN[1] = param.as_double();
          } else if (name == "QN2") {
            mpc_->cfg.QN[2] = param.as_double();
          } else if (name == "ay_max") {
            mpc_cfg_.ay_max = param.as_double();
            mpc_->update_ay_max(param.as_double());
          } else if (name == "accel_low_pass_gain") {
            mpc_cfg_.accel_low_pass_gain = param.as_double();
            mpc_->cfg.accel_low_pass_gain = param.as_double();
          } else if (name == "steer_low_pass_gain") {
            mpc_cfg_.steer_low_pass_gain = param.as_double();
            mpc_->cfg.steer_low_pass_gain = param.as_double();
          } else if (name == "wp_id_offset") {
            mpc_cfg_.wp_id_offset = param.as_int();
            mpc_->update_wp_id_offset(param.as_int());
          } else if (name == "wp_id_low_offset") {
            mpc_cfg_.wp_id_low_offset = param.as_int();
            mpc_->cfg.wp_id_low_offset = param.as_int();
          } else if (name == "wp_id_low_speed") {
            mpc_cfg_.wp_id_low_speed_kmh = std::max(0.0, param.as_double());
            mpc_cfg_.wp_id_low_speed = kmh_to_m_per_sec(mpc_cfg_.wp_id_low_speed_kmh);
            mpc_->cfg.wp_id_low_speed_kmh = mpc_cfg_.wp_id_low_speed_kmh;
            mpc_->cfg.wp_id_low_speed = mpc_cfg_.wp_id_low_speed;
          }
        }
        return result;
      });
  }

  void setup_pub_sub()
  {
    if (use_bug_acc_) {
      boost_command_pub_ = create_publisher<AckermannControlBoostCommand>("/boost_commander/command", 1);
    } else {
      command_pub_ = create_publisher<AckermannControlCommand>("/control/command/control_cmd", 1);
      command_raw_pub_ = create_publisher<AckermannControlCommand>("/control/command/control_cmd_raw", 1);
    }
    mpc_pred_pub_ = create_publisher<MarkerArray>("/mpc/prediction", 1);
    mpc_pred_pub_dummy_ = create_publisher<MarkerArray>(
      "/planning/scenario_planning/lane_driving/motion_planning/obstacle_stop_planner/virtual_wall", 1);
    const auto latching_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
    ref_path_pub_ = create_publisher<MarkerArray>("/mpc/ref_path", latching_qos);
    ref_path_pub_dummy_ = create_publisher<MarkerArray>(
      "/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/bound",
      latching_qos);
    ref_vel_marker_pub_ = create_publisher<MarkerArray>("/ref_vel_marker", latching_qos);
    section_marker_pub_ = create_publisher<MarkerArray>("/section_marker", latching_qos);
    if (awsim_boost_io_enabled_) {
      const auto boost_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
      awsim_boost_pub_ = create_publisher<Float32MultiArray>("/awsim/cmd", boost_qos);
    }
    if (stuck_recovery_actuation_io_enabled_) {
      // Volatile durability prevents a newly joined subscriber from replaying a stale REVERSE.
      gear_command_pub_ = create_publisher<GearCommand>(
        "/control/command/gear_cmd", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    }

    odom_sub_ = create_subscription<Odometry>(
      "/localization/kinematic_state", 1, [this](const Odometry::SharedPtr msg) {
        const auto receipt_time = SteadyClock::now();
        odom_ = msg;
        last_odom_receipt_steady_ = receipt_time;
        const rclcpp::Time source_stamp(msg->header.stamp);
        if (source_stamp.nanoseconds() > 0) {
          if (
            !last_odom_source_stamp_.has_value() ||
            source_stamp.nanoseconds() != last_odom_source_stamp_->nanoseconds())
          {
            last_odom_source_stamp_ = source_stamp;
            last_odom_source_advance_steady_ = receipt_time;
          }
        } else {
          last_odom_source_stamp_.reset();
          last_odom_source_advance_steady_.reset();
        }
      });
    control_mode_request_sub_ = create_subscription<Bool>(
      "control/control_mode_request_topic", 1, [this](const Bool::SharedPtr msg) {
        if (msg->data && !enable_control_) {
          enable_control_ = true;
        }
      });
    const auto trajectory_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    trajectory_sub_ = create_subscription<Trajectory>(
      "planning/scenario_planning/trajectory", trajectory_qos,
      [this](const Trajectory::SharedPtr msg) { trajectory_ = msg; });
    stop_request_sub_ = create_subscription<Empty>(
      "/control/mpc/stop_request", 1, [this](const Empty::SharedPtr) {
        if (enable_control_) {
          RCLCPP_WARN(get_logger(), "Stop request received");
          enable_control_ = false;
        }
      });
    if (cfg_.stuck_recovery.core.enabled) {
      gear_report_sub_ = create_subscription<GearReport>(
        "/vehicle/status/gear_status", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
        [this](const GearReport::SharedPtr msg) {
          const auto next_gear = recovery_gear_from_report(msg->report);
          if (!reported_gear_.has_value() || reported_gear_.value() != next_gear) {
            RCLCPP_INFO(
              get_logger(), "Stuck recovery gear report: raw=%u, gear=%s",
              static_cast<unsigned int>(msg->report), stuck_recovery::to_string(next_gear));
          }
          reported_gear_ = next_gear;
          if (next_gear == stuck_recovery::Gear::Drive) {
            last_commanded_recovery_gear_ = stuck_recovery::Gear::Drive;
            if (recovery_waiting_for_drive_after_reset_) {
              recovery_waiting_for_drive_after_reset_ = false;
              recovery_fault_latched_ = !race_started_;
              recovery_reset_drive_request_count_ = 0U;
              recovery_reset_stopped_since_.reset();
              RCLCPP_INFO(
                get_logger(),
                "Stuck recovery Drive report confirmed; control hold remains=%d until Start",
                recovery_fault_latched_ ? 1 : 0);
            }
          }
          last_gear_report_receipt_steady_ = SteadyClock::now();
        });
    }
    if (use_sim_time_) {
      awsim_status_sub_ = create_subscription<Float32MultiArray>(
        "/awsim/status", 1,
        [this](const Float32MultiArray::SharedPtr msg) {awsim_status_callback(msg);});
      if (awsim_state_tracking_enabled_) {
        awsim_state_sub_ = create_subscription<String>(
          "/awsim/state", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
          [this](const String::SharedPtr msg) {awsim_state_callback(msg);});
      }
      condition_sub_ = create_subscription<Int32>(
        "/aichallenge/pitstop/condition", 1, [this](const Int32::SharedPtr msg) {
          if (!last_condition_.has_value()) {
            last_condition_ = msg->data;
          }
          const int diff_condition = msg->data - last_condition_.value();
          if (diff_condition > 30) {
            last_colliding_time_ = now();
            last_collision_receipt_steady_ = SteadyClock::now();
            RCLCPP_WARN(get_logger(), "Collision detected!");
          }
          last_condition_ = msg->data;
        });
    }
    if (use_obstacle_avoidance_ && cfg_.reference_path.use_path_constraints_topic) {
      path_constraints_sub_ = create_subscription<PathConstraints>(
      "/path_constraints_provider/path_constraints", 1, [this](const PathConstraints::SharedPtr msg) {
          if (
            msg->rows != reference_path_->n_waypoints - 1 || msg->cols != mpc_cfg_.N ||
            !reference_path_->set_path_constraints(
              msg->upper_bounds, msg->lower_bounds, msg->rows, msg->cols))
          {
            RCLCPP_WARN(get_logger(), "Invalid path constraints message ignored");
          }
        });
    }
    if (use_obstacle_avoidance_ && cfg_.reference_path.use_border_cells_topic) {
      border_cells_sub_ = create_subscription<BorderCells>(
      "/path_constraints_provider/border_cells", 1, [this](const BorderCells::SharedPtr msg) {
          if (
            msg->rows != reference_path_->n_waypoints - 1 || msg->cols != mpc_cfg_.N ||
            !reference_path_->set_border_cells(
              msg->dynamic_upper_bounds, msg->dynamic_lower_bounds, msg->rows, msg->cols))
          {
            RCLCPP_WARN(get_logger(), "Invalid border cells message ignored");
          }
        });
    }
    if (
      use_obstacle_avoidance_ || mpc_cfg_.v2x_gap.enabled || mpc_cfg_.v2x_behavior.enabled ||
      cfg_.stuck_recovery.core.enabled)
    {
      v2x_sub_ = create_subscription<V2XVehiclePositionArray>(
        "/v2x/vehicle_positions", 1, [this](const V2XVehiclePositionArray::SharedPtr msg) {
          if (v2x_gap_planner_) {
            v2x_gap_planner_->update(*msg, now().seconds());
          }
        });
    }
  }

  AckermannControlCommand create_ackermann_control_command(
    const rclcpp::Time & stamp, const Eigen::Vector2d & u, const double acc) const
  {
    AckermannControlCommand msg;
    msg.stamp = stamp;
    msg.lateral.stamp = stamp;
    msg.lateral.steering_tire_angle = u[1];
    msg.lateral.steering_tire_rotation_rate = 2.0;
    msg.longitudinal.stamp = stamp;
    msg.longitudinal.speed = u[0];
    msg.longitudinal.acceleration = acc;
    return msg;
  }

  bool command_is_finite(const AckermannControlCommand & command) const
  {
    return
      std::isfinite(command.lateral.steering_tire_angle) &&
      std::isfinite(command.lateral.steering_tire_rotation_rate) &&
      std::isfinite(command.longitudinal.speed) &&
      std::isfinite(command.longitudinal.acceleration);
  }

  bool recovery_may_be_in_reverse() const
  {
    return
      (reported_gear_.has_value() &&
      reported_gear_.value() == stuck_recovery::Gear::Reverse) ||
      (last_commanded_recovery_gear_.has_value() &&
      last_commanded_recovery_gear_.value() == stuck_recovery::Gear::Reverse);
  }

  void publish_failsafe_command(const rclcpp::Time & stamp, const char * reason)
  {
    const bool reverse_possible = recovery_may_be_in_reverse();
    const double normal_safe_deceleration =
      std::isfinite(mpc_cfg_.a_min) && mpc_cfg_.a_min < 0.0 ? mpc_cfg_.a_min : -1.0;
    const double safe_deceleration = reverse_possible ?
      (stuck_recovery_actuation_io_enabled_ ?
      reverse_actuation_calibration(cfg_.stuck_recovery).stop_acceleration_mps2 : 0.0) :
      normal_safe_deceleration;
    const double previous_steering = std::isfinite(last_u_[1]) ? last_u_[1] : 0.0;
    const double max_steering_step =
      std::isfinite(mpc_cfg_.steer_rate_max) && std::isfinite(mpc_cfg_.control_rate) &&
      mpc_cfg_.control_rate > 0.0 ?
      std::max(0.0, mpc_cfg_.steer_rate_max) / mpc_cfg_.control_rate : 0.0;
    const double safe_steering = clip(
      0.0, previous_steering - max_steering_step,
      previous_steering + max_steering_step);
    const Eigen::Vector2d safe_control(0.0, safe_steering);
    auto raw_command = create_ackermann_control_command(
      stamp, safe_control, safe_deceleration);

    if (!command_failsafe_active_) {
      RCLCPP_ERROR(get_logger(), "Control failsafe active: %s", reason);
    }
    command_failsafe_active_ = true;
    if (
      recovery_last_output_.has_value() &&
      recovery_last_output_->state != stuck_recovery::RecoveryState::Normal)
    {
      recovery_fault_latched_ = true;
      recovery_boost_suppressed_for_session_ = true;
    }

    if (use_bug_acc_) {
      AckermannControlBoostCommand boost_command;
      boost_command.command = raw_command;
      boost_command.boost_mode = false;
      boost_command_pub_->publish(boost_command);
    } else {
      command_raw_pub_->publish(raw_command);
      const double steering_gain =
        std::isfinite(mpc_cfg_.steering_tire_angle_gain_var) ?
        mpc_cfg_.steering_tire_angle_gain_var : 1.0;
      raw_command.lateral.steering_tire_angle *= steering_gain;
      if (!command_is_finite(raw_command)) {
        raw_command.lateral.steering_tire_angle = 0.0;
      }
      command_pub_->publish(raw_command);
    }
    last_u_ = safe_control;
    last_acc_ = safe_deceleration;
    if (mpc_) {
      mpc_->reset_control_history(safe_steering);
    }
  }

  bool publish_control_command(
    const rclcpp::Time & stamp, const Eigen::Vector2d & u, const double acc, const bool bug_acc_enabled)
  {
    auto raw_command = create_ackermann_control_command(stamp, u, acc);
    if (use_bug_acc_) {
      if (!command_is_finite(raw_command)) {
        publish_failsafe_command(stamp, "non-finite boost control command rejected");
        return false;
      }
      AckermannControlBoostCommand boost_cmd;
      boost_cmd.command = raw_command;
      boost_cmd.boost_mode = bug_acc_enabled;
      boost_command_pub_->publish(boost_cmd);
      command_failsafe_active_ = false;
      return true;
    }
    auto final_command = raw_command;
    final_command.lateral.steering_tire_angle *= mpc_cfg_.steering_tire_angle_gain_var;
    if (!command_is_finite(raw_command) || !command_is_finite(final_command)) {
      publish_failsafe_command(stamp, "non-finite control command rejected");
      return false;
    }
    command_raw_pub_->publish(raw_command);
    command_pub_->publish(final_command);
    command_failsafe_active_ = false;
    return true;
  }

  void publish_ref_path_marker()
  {
    MarkerArray marker_array;
    Marker base;
    base.header.frame_id = "map";
    base.ns = "ref_path";
    base.type = Marker::LINE_STRIP;
    base.action = Marker::ADD;
    base.pose.position.z = 0.0;
    base.scale.x = 0.2;
    base.color.r = 0.0;
    base.color.g = 0.0;
    base.color.b = 1.0;
    base.color.a = 0.7;
    for (int i = 0; i + 1 < reference_path_->n_waypoints; ++i) {
      Marker m = base;
      m.id = i;
      Point start;
      start.x = reference_path_->waypoints[i].x;
      start.y = reference_path_->waypoints[i].y;
      Point end;
      end.x = reference_path_->waypoints[i + 1].x;
      end.y = reference_path_->waypoints[i + 1].y;
      m.points.push_back(start);
      m.points.push_back(end);
      marker_array.markers.push_back(m);
    }
    ref_path_pub_->publish(marker_array);
    ref_path_pub_dummy_->publish(marker_array);
    ref_path_published_ = true;
  }

  void publish_mpc_pred_marker(const std::vector<double> & x_pred, const std::vector<double> & y_pred)
  {
    MarkerArray marker_array;
    Marker base;
    base.header.frame_id = "map";
    base.ns = "mpc_pred";
    base.type = Marker::SPHERE;
    base.action = Marker::ADD;
    base.pose.position.z = 0.0;
    base.scale = Vector3();
    base.scale.x = 0.5;
    base.scale.y = 0.5;
    base.scale.z = 0.5;
    base.color = pred_marker_color_;
    for (std::size_t i = 0; i < x_pred.size() && i < y_pred.size(); ++i) {
      Marker m = base;
      m.id = static_cast<int>(i);
      m.pose.position.x = x_pred[i];
      m.pose.position.y = y_pred[i];
      marker_array.markers.push_back(m);
    }
    mpc_pred_pub_->publish(marker_array);
    mpc_pred_pub_dummy_->publish(marker_array);
  }

  void awsim_status_callback(const Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() >= 7U && std::isfinite(msg->data[6])) {
      awsim_boost_active_ = msg->data[6] > 0.5F;
      last_awsim_status_receipt_steady_ = SteadyClock::now();
    }
    if (awsim_boost_io_enabled_ && awsim_boost_guard_) {
      const auto previous_phase = awsim_boost_guard_->phase();
      if (!awsim_boost_guard_->on_awsim_status(msg->data, SteadyClock::now())) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Invalid /awsim/status for Boost: expected 7 finite values including indices 5 and 6");
      } else if (
        previous_phase != awsim_boost_guard_->phase() &&
        awsim_boost_guard_->phase() == awsim_boost::Phase::Confirmed)
      {
        RCLCPP_INFO(
          get_logger(), "AWSIM Boost confirmed: remaining=%.0f, is_boosting=%d",
          awsim_boost_guard_->remaining().value_or(-1.0),
          awsim_boost_guard_->is_boosting().value_or(false) ? 1 : 0);
      }
    }
    if (msg->data.size() < 3) {
      return;
    }
    const int laps = static_cast<int>(msg->data[1]);
    const double lap_time = msg->data[2];
    if (laps > current_laps_) {
      RCLCPP_INFO(get_logger(), "Lap %d completed! Lap time: %.3f s", current_laps_, last_lap_time_);
      current_laps_ = laps;
    }
    last_lap_time_ = lap_time;
  }

  void reset_stuck_recovery_session(const char * reason)
  {
    if (stuck_recovery_core_) {
      stuck_recovery_core_->reset_session();
    }
    if (cfg_.stuck_recovery.core.enabled && v2x_gap_planner_) {
      v2x_gap_planner_->reset_recovery_tracking();
    }
    recovery_observation_anchor_pose_.reset();
    recovery_observation_anchor_progress_m_.reset();
    recovery_maneuver_start_pose_.reset();
    recovery_last_reverse_pose_.reset();
    recovery_cumulative_reverse_distance_m_ = 0.0;
    recovery_episode_traveled_distance_m_ = 0.0;
    recovery_reverse_pose_jump_ = false;
    recovery_episode_had_contact_evidence_ = false;
    recovery_aggressive_retry_count_ = 0U;
    recovery_coordinated_stop_episode_ = false;
    recovery_reverse_only_episode_ = false;
    recovery_selected_reverse_primitive_.reset();
    recovery_selected_reverse_steering_angle_rad_.reset();
    recovery_selected_stepwise_escape_ = false;
    recovery_initial_contact_cells_.reset();
    recovery_previous_contact_cells_.reset();
    recovery_last_output_.reset();
    recovery_boost_suppressed_for_session_ = false;
    recovery_reset_applied_ = false;
    recovery_rejoin_hold_cycle_ = false;
    recovery_fault_latched_ = false;
    recovery_waiting_for_drive_after_reset_ = false;
    recovery_reset_drive_request_count_ = 0U;
    recovery_reset_stopped_since_.reset();
    last_recovery_state_.reset();
    last_recovery_reject_reason_.reset();
    last_recovery_execution_mode_.reset();
    last_gear_report_receipt_steady_.reset();
    last_collision_receipt_steady_.reset();
    RCLCPP_INFO(get_logger(), "Stuck recovery session reset: %s", reason);
  }

  void arm_start_grid_grace(const rclcpp::Time & start_time, const char * reason)
  {
    if (
      !mpc_ || !mpc_cfg_.v2x_behavior.enabled ||
      mpc_cfg_.v2x_behavior.start_grid_grace_time <= 0.0)
    {
      return;
    }
    const auto transition = mpc_->arm_start_grid_grace(start_time.seconds());
    if (transition == start_grid_grace::Transition::Armed) {
      RCLCPP_INFO(
        get_logger(), "Start grid grace armed: duration=%.2f s, reason=%s",
        mpc_cfg_.v2x_behavior.start_grid_grace_time, reason);
    } else if (transition == start_grid_grace::Transition::ClockRejected) {
      RCLCPP_WARN(
        get_logger(), "Start grid grace rejected: invalid ROS clock, reason=%s", reason);
    }
  }

  void prepare_start_grid_grace(const char * reason)
  {
    if (
      !mpc_ || !mpc_cfg_.v2x_behavior.enabled ||
      mpc_cfg_.v2x_behavior.start_grid_grace_time <= 0.0)
    {
      return;
    }
    if (mpc_->prepare_start_grid_grace() == start_grid_grace::Transition::Prepared) {
      RCLCPP_INFO(
        get_logger(),
        "Start grid grace prepared: static-grid suppression active until %.2f s after Start, "
        "reason=%s",
        mpc_cfg_.v2x_behavior.start_grid_grace_time, reason);
    }
  }

  void clear_start_grid_grace(const char * reason)
  {
    if (!mpc_ || !mpc_cfg_.v2x_behavior.enabled) {
      return;
    }
    if (mpc_->clear_start_grid_grace() == start_grid_grace::Transition::Cleared) {
      RCLCPP_INFO(get_logger(), "Start grid grace cleared: %s", reason);
    }
  }

  void awsim_state_callback(const String::SharedPtr msg)
  {
    if (!awsim_boost_guard_) {
      return;
    }
    std::string normalized_state = msg->data;
    const auto is_space = [](const unsigned char value) {return std::isspace(value) != 0;};
    const auto first = std::find_if_not(normalized_state.begin(), normalized_state.end(), is_space);
    const auto last = std::find_if_not(
      normalized_state.rbegin(), normalized_state.rend(), is_space).base();
    normalized_state = first < last ? std::string(first, last) : std::string{};
    std::transform(
      normalized_state.begin(), normalized_state.end(), normalized_state.begin(),
      [](const unsigned char value) {return static_cast<char>(std::tolower(value));});
    const rclcpp::Time state_time = now();
    const bool recovery_state_changed = normalized_state != last_awsim_state_;
    if (recovery_state_changed) {
      const bool recovery_was_active =
        recovery_last_output_.has_value() &&
        recovery_last_output_->state != stuck_recovery::RecoveryState::Normal;
      const bool reverse_was_possible = recovery_may_be_in_reverse();
      if (normalized_state == "start") {
        reset_stuck_recovery_session("AWSIM Start entered");
        race_started_ = true;
        if (reverse_was_possible) {
          recovery_waiting_for_drive_after_reset_ = true;
          recovery_fault_latched_ = true;
          recovery_boost_suppressed_for_session_ = true;
          RCLCPP_ERROR(
            get_logger(),
            "AWSIM Start entered while recovery gear may still be Reverse; holding until Drive report");
        }
      } else {
        if (race_started_ || normalized_state == "spawned" || normalized_state == "finish") {
          reset_stuck_recovery_session("AWSIM race inactive or session boundary");
          if (recovery_was_active || reverse_was_possible) {
            recovery_fault_latched_ = true;
            recovery_boost_suppressed_for_session_ = true;
            recovery_waiting_for_drive_after_reset_ = reverse_was_possible;
          }
        }
        race_started_ = false;
      }
      last_awsim_state_ = normalized_state;
    }
    if (
      recovery_state_changed &&
      (normalized_state == "spawned" || normalized_state == "grounded" ||
      normalized_state == "finish"))
    {
      clear_start_grid_grace("AWSIM session boundary");
    }
    if (recovery_state_changed && normalized_state == "ready") {
      prepare_start_grid_grace("AWSIM Ready entered");
    }
    const auto previous_phase = awsim_boost_guard_->phase();
    const auto state_event = awsim_boost_guard_->on_awsim_state(msg->data);
    if (
      previous_phase != awsim_boost_guard_->phase() &&
      awsim_boost_guard_->phase() == awsim_boost::Phase::AwaitingMotion)
    {
      RCLCPP_INFO(
        get_logger(), "AWSIM Boost motion watch prepared: state=%s",
        normalized_state.c_str());
      last_awsim_boost_block_reason_ = awsim_boost::BlockReason::None;
    }
    if (state_event == awsim_boost::StateEvent::StartEntered) {
      domain_start_epoch_ = state_time;
      arm_start_grid_grace(state_time, "AWSIM Start entered");
      last_start_window_status_.reset();
      domain_manual_reset_pending_ = false;
      domain_manual_reset_ready_ = false;
      RCLCPP_INFO(
        get_logger(), "AWSIM race Start entered; domain start-speed window epoch captured");
    } else if (
      state_event == awsim_boost::StateEvent::Finished ||
      state_event == awsim_boost::StateEvent::NewSession)
    {
      clear_start_grid_grace("AWSIM race session event");
      domain_start_epoch_.reset();
      last_start_window_status_.reset();
      domain_manual_reset_pending_ = false;
      domain_manual_reset_ready_ = false;
    } else if (
      normalized_state == "spawned" &&
      (domain_start_epoch_.has_value() || domain_manual_reset_pending_))
    {
      // Some manual reset paths omit Finish. Do not rearm Boost, but require the full
      // Spawned -> Grounded/Ready -> Start progression before starting a new speed window.
      domain_manual_reset_pending_ = true;
      domain_manual_reset_ready_ = false;
    } else if (
      domain_manual_reset_pending_ &&
      (normalized_state == "grounded" || normalized_state == "ready"))
    {
      domain_start_epoch_.reset();
      last_start_window_status_.reset();
      domain_manual_reset_ready_ = true;
    } else if (
      normalized_state == "start" && domain_manual_reset_pending_ &&
      domain_manual_reset_ready_)
    {
      domain_start_epoch_ = state_time;
      arm_start_grid_grace(state_time, "AWSIM manual reset Start entered");
      last_start_window_status_.reset();
      domain_manual_reset_pending_ = false;
      domain_manual_reset_ready_ = false;
      RCLCPP_INFO(
        get_logger(),
        "AWSIM manual reset sequence completed; domain start-speed window epoch captured");
    }
    if (
      previous_phase != awsim_boost_guard_->phase() &&
      awsim_boost_guard_->phase() == awsim_boost::Phase::Armed)
    {
      RCLCPP_INFO(get_logger(), "AWSIM Boost rearmed for a new race session.");
      last_awsim_boost_block_reason_ = awsim_boost::BlockReason::None;
    }
  }

  void maybe_publish_awsim_boost(
    const SteadyClock::time_point steady_now,
    const awsim_boost::TriggerContext & context)
  {
    if (!awsim_boost_io_enabled_ || !awsim_boost_guard_ || !awsim_boost_pub_) {
      return;
    }

    const auto previous_phase = awsim_boost_guard_->phase();
    const auto evaluation = awsim_boost_guard_->evaluate(context, steady_now);
    if (evaluation.motion_detected_now) {
      RCLCPP_INFO(
        get_logger(), "AWSIM Boost launch motion detected: speed=%.3f m/s",
        context.forward_speed_mps);
    }
    if (
      previous_phase != awsim_boost_guard_->phase() &&
      awsim_boost_guard_->phase() == awsim_boost::Phase::UnconfirmedSpent)
    {
      RCLCPP_WARN(
        get_logger(),
        "AWSIM Boost confirmation timed out; the start Boost remains spent and will not be retried.");
    }
    if (
      previous_phase != awsim_boost_guard_->phase() &&
      awsim_boost_guard_->phase() == awsim_boost::Phase::LaunchExpiredSpent)
    {
      RCLCPP_WARN(
        get_logger(), "AWSIM launch Boost skipped: %s, speed=%.3f m/s",
        awsim_boost::to_string(evaluation.reason), context.forward_speed_mps);
    }

    if (evaluation.action == awsim_boost::Action::PublishPulse) {
      Float32MultiArray command;
      command.data = {awsim_boost::kCommandPulseValues[0]};
      awsim_boost_pub_->publish(command);
      command.data = {awsim_boost::kCommandPulseValues[1]};
      awsim_boost_pub_->publish(command);
      RCLCPP_INFO(
        get_logger(),
        "AWSIM launch Boost pulse published once: motion_delay=%.3f s, "
        "remaining_before=%.0f",
        evaluation.motion_elapsed_sec, awsim_boost_guard_->remaining().value_or(-1.0));
      last_awsim_boost_block_reason_ = awsim_boost::BlockReason::None;
      return;
    }

    if (
      evaluation.reason != awsim_boost::BlockReason::None &&
      evaluation.reason != last_awsim_boost_block_reason_)
    {
      RCLCPP_INFO(
        get_logger(), "AWSIM Boost not published: %s",
        awsim_boost::to_string(evaluation.reason));
      last_awsim_boost_block_reason_ = evaluation.reason;
    }
  }

  std::unique_ptr<ReferencePath> create_reference_path_from_autoware_trajectory(
    const Trajectory & trajectory)
  {
    std::vector<double> wp_x;
    std::vector<double> wp_y;
    wp_x.reserve(trajectory.points.size());
    wp_y.reserve(trajectory.points.size());
    for (const auto & point : trajectory.points) {
      wp_x.push_back(point.pose.position.x);
      wp_y.push_back(point.pose.position.y);
    }
    auto ref_path = std::make_unique<ReferencePath>(
      map_.get(), wp_x, wp_y, cfg_.reference_path.resolution, cfg_.reference_path.smoothing_distance,
      cfg_.reference_path.max_width, cfg_.reference_path.circular);
    if (!ref_path->compute_speed_profile(
          mpc_cfg_.a_min, mpc_cfg_.a_max, 0.0, mpc_cfg_.v_max, mpc_cfg_.ay_max)) {
      return nullptr;
    }
    ref_path->update_simple_path_constraints(mpc_cfg_.N, car_->safety_margin);
    return ref_path;
  }

  bool recovery_gear_report_fresh(const SteadyClock::time_point steady_now) const
  {
    if (!reported_gear_.has_value() || !last_gear_report_receipt_steady_.has_value()) {
      return false;
    }
    const double age_sec = std::chrono::duration<double>(
      steady_now - last_gear_report_receipt_steady_.value()).count();
    return std::isfinite(age_sec) && age_sec >= 0.0 &&
           age_sec <= cfg_.stuck_recovery.core.supervisor.gear_report_timeout_sec;
  }

  bool recovery_boost_inactive_confirmed(const SteadyClock::time_point steady_now) const
  {
    if (!awsim_boost_active_.has_value() || !last_awsim_status_receipt_steady_.has_value()) {
      return false;
    }
    const double age_sec = std::chrono::duration<double>(
      steady_now - last_awsim_status_receipt_steady_.value()).count();
    return std::isfinite(age_sec) && age_sec >= 0.0 &&
           age_sec <= cfg_.stuck_recovery.boost_status_timeout_sec &&
           !awsim_boost_active_.value();
  }

  bool recovery_gear_transition_active() const
  {
    if (!stuck_recovery_core_) {
      return false;
    }
    switch (stuck_recovery_core_->supervisor().state()) {
      case stuck_recovery::RecoveryState::ShiftToReverse:
      case stuck_recovery::RecoveryState::WaitReverseReport:
      case stuck_recovery::RecoveryState::ShiftToDrive:
      case stuck_recovery::RecoveryState::WaitDriveReport:
        return true;
      default:
        return false;
    }
  }

  double recovery_progress_delta(const double current_progress_m) const
  {
    if (!recovery_observation_anchor_progress_m_.has_value()) {
      return 0.0;
    }
    double delta = current_progress_m - recovery_observation_anchor_progress_m_.value();
    if (!cfg_.reference_path.circular) {
      return delta;
    }
    double path_length_m = 0.0;
    for (const double segment_length_m : reference_path_->segment_lengths) {
      path_length_m += segment_length_m;
    }
    if (!std::isfinite(path_length_m) || path_length_m <= kEps) {
      return delta;
    }
    while (delta > 0.5 * path_length_m) {
      delta -= path_length_m;
    }
    while (delta < -0.5 * path_length_m) {
      delta += path_length_m;
    }
    return delta;
  }

  std::optional<double> recovery_rejoin_steering_tire_angle(
    const Eigen::Vector2d & normal_u)
  {
    const double steering_gain = mpc_cfg_.steering_tire_angle_gain_var;
    const double mpc_tire_angle_limit_rad =
      std::abs(mpc_cfg_.delta_max * steering_gain);
    if (
      !std::isfinite(steering_gain) || steering_gain <= 0.0 ||
      !std::isfinite(mpc_tire_angle_limit_rad) || mpc_tire_angle_limit_rad <= 0.0)
    {
      return std::nullopt;
    }

    const double configured_limit_rad =
      cfg_.stuck_recovery.rejoin_max_steering_tire_angle_rad;
    const double effective_limit_rad = std::min(
      configured_limit_rad, mpc_tire_angle_limit_rad);
    std::optional<double> target_tire_angle_rad;
    if (cfg_.stuck_recovery.recovery_mpc_enabled && effective_limit_rad > 0.0) {
      auto planner_config = cfg_.stuck_recovery.recovery_mpc_config;
      planner_config.maximum_steering_angle_rad = std::min(
        planner_config.maximum_steering_angle_rad, effective_limit_rad);
      planner_config.maximum_steering_change_rad = std::min(
        planner_config.maximum_steering_change_rad,
        2.0 * planner_config.maximum_steering_angle_rad);
      const auto plan = recovery_mpc::plan(
        planner_config,
        recovery_mpc::Request{
          recovery_mpc::Direction::Forward,
          car_->spatial_state.e_y,
          car_->spatial_state.e_psi,
          car_->current_waypoint->kappa,
          cfg_.bicycle_length,
          last_u_[1] * steering_gain});
      if (plan.valid) {
        target_tire_angle_rad = plan.first_steering_tire_angle_rad;
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 500,
          "Recovery MPC rejoin plan: steering=%.3f rad, terminal_e_y=%.3f m, "
          "terminal_e_psi=%.3f rad, cost=%.3f",
          plan.first_steering_tire_angle_rad, plan.terminal_lateral_error_m,
          plan.terminal_heading_error_rad, plan.cost);
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Recovery MPC rejoin fallback: reason=%s",
          recovery_mpc::to_string(plan.reason));
      }
    }

    if (!target_tire_angle_rad.has_value() &&
      !cfg_.stuck_recovery.rejoin_feedback_steering_enabled)
    {
      const double normal_tire_angle_rad = normal_u[1] * steering_gain;
      if (!std::isfinite(normal_tire_angle_rad)) {
        return std::nullopt;
      }
      return clip(
        normal_tire_angle_rad,
        -mpc_tire_angle_limit_rad,
        mpc_tire_angle_limit_rad);
    }

    if (!target_tire_angle_rad.has_value()) {
      target_tire_angle_rad = stuck_recovery::compute_rejoin_steering_tire_angle(
        stuck_recovery::RejoinSteeringRequest{
          car_->current_waypoint->kappa,
          cfg_.bicycle_length,
          car_->spatial_state.e_y,
          car_->spatial_state.e_psi,
          cfg_.stuck_recovery.rejoin_lateral_error_gain_rad_per_m,
          cfg_.stuck_recovery.rejoin_heading_error_gain,
          effective_limit_rad});
    }
    if (!target_tire_angle_rad.has_value()) {
      return std::nullopt;
    }

    const double target_controller_angle_rad =
      target_tire_angle_rad.value() / steering_gain;
    const double max_controller_step_rad =
      mpc_cfg_.steer_rate_max / mpc_cfg_.control_rate;
    if (
      !std::isfinite(target_controller_angle_rad) ||
      !std::isfinite(max_controller_step_rad) || max_controller_step_rad < 0.0 ||
      !std::isfinite(last_u_[1]))
    {
      return std::nullopt;
    }
    const double limited_controller_angle_rad = clip(
      target_controller_angle_rad,
      last_u_[1] - max_controller_step_rad,
      last_u_[1] + max_controller_step_rad);
    return clip(
      limited_controller_angle_rad * steering_gain,
      -effective_limit_rad,
      effective_limit_rad);
  }

  RecoverySafetySnapshot evaluate_recovery_safety(
    const Pose2D & pose, const SteadyClock::time_point steady_now,
    const double ros_now_sec, const double reverse_distance_to_check_m,
    const double forward_distance_to_check_m,
    const double escape_step_distance_to_check_m,
    const double recovery_heading_error_rad,
    const double rejoin_steering_angle_rad,
    const bool reverse_only,
    const bool evaluate_rollout) const
  {
    RecoverySafetySnapshot snapshot;
    std::optional<recovery_footprint::FeasibilityResult> forward_deadlock_fallback;
    bool forward_deadlock_fallback_stepwise = false;
    std::optional<std::vector<recovery_footprint::RolloutPose>> selected_v2x_rollout;
    if (!recovery_grid_ || !recovery_footprint_.valid()) {
      return snapshot;
    }

    const recovery_footprint::Pose2D recovery_pose{pose.x, pose.y, pose.theta};
    const auto wall_proximity = recovery_footprint::classify_nearby_wall(
      *recovery_grid_, recovery_footprint_, recovery_pose,
      cfg_.stuck_recovery.wall_direction_search_margin_m,
      cfg_.stuck_recovery.wall_direction_ambiguity_m);
    snapshot.wall_region = wall_proximity.region;
    snapshot.wall_distance_m = wall_proximity.nearest_distance_m;
    const auto current_sample = recovery_footprint::sample_footprint(
      *recovery_grid_, recovery_footprint_, recovery_pose);
    snapshot.current_contact_cells = current_sample.contact_cells;
    snapshot.current_contact_count = current_sample.contact_cells.size();
    snapshot.wall_evidence = wall_proximity.valid &&
      wall_proximity.region != recovery_footprint::WallRegion::None &&
      wall_proximity.region != recovery_footprint::WallRegion::Unknown;
    snapshot.current_footprint_clear =
      current_sample.valid && !current_sample.out_of_map && current_sample.contact_cells.empty();
    const auto * initial_contact_patch = recovery_initial_contact_cells_.has_value() ?
      &recovery_initial_contact_cells_.value() : nullptr;
    const auto * previous_contact_patch = recovery_previous_contact_cells_.has_value() ?
      &recovery_previous_contact_cells_.value() :
      initial_contact_patch;
    if (previous_contact_patch != nullptr) {
      snapshot.runtime_contact_reject_reason =
        initial_contact_patch != nullptr && current_sample.valid && !current_sample.out_of_map ?
        (recovery_selected_stepwise_escape_ ?
        recovery_footprint::evaluate_improving_contact_transition(
        *recovery_grid_, *initial_contact_patch, *previous_contact_patch,
        current_sample.contact_cells) :
        recovery_footprint::evaluate_contact_transition(
        *recovery_grid_, *initial_contact_patch, *previous_contact_patch,
        current_sample.contact_cells)) :
        (current_sample.out_of_map ? recovery_footprint::RejectReason::OutOfMap :
        recovery_footprint::RejectReason::InvalidInitialPose);
      snapshot.collision_worsening =
        snapshot.runtime_contact_reject_reason != recovery_footprint::RejectReason::None;
    }

    if (evaluate_rollout) {
      recovery_footprint::ReverseRolloutParameters rollout;
      rollout.rollout_step_m = cfg_.stuck_recovery.sweep_interpolation_step_m;
      rollout.swept_step_m = cfg_.stuck_recovery.sweep_interpolation_step_m;
      rollout.wheelbase_m = cfg_.bicycle_length;
      rollout.steering_angle_rad = cfg_.stuck_recovery.reverse_steering_angle_rad;

      if (snapshot.current_footprint_clear && std::isfinite(rejoin_steering_angle_rad)) {
        auto rejoin_rollout = rollout;
        rejoin_rollout.reverse_distance_m = cfg_.stuck_recovery.rejoin_static_lookahead_m;
        rejoin_rollout.steering_angle_rad = std::abs(rejoin_steering_angle_rad);
        const auto rejoin_primitive = std::abs(rejoin_steering_angle_rad) <= kEps ?
          recovery_footprint::ReversePrimitive::ForwardStraight :
          rejoin_steering_angle_rad > 0.0 ?
          recovery_footprint::ReversePrimitive::ForwardLeft :
          recovery_footprint::ReversePrimitive::ForwardRight;
        const auto rejoin_result = recovery_footprint::evaluate_recovery_candidate(
          *recovery_grid_, recovery_footprint_, recovery_pose, rejoin_primitive,
          rejoin_rollout, recovery_footprint::ContactEscapePolicy::RequireClear, 0.0);
        snapshot.rejoin_forward_static_clear = rejoin_result.feasible;
        snapshot.rejoin_static_reject_reason = rejoin_result.reason;
        snapshot.rejoin_static_rejected_at_distance_m = rejoin_result.rejected_at_distance_m;
      }

      const std::size_t completed_escape_steps = stuck_recovery_core_ != nullptr ?
        stuck_recovery_core_->supervisor().escape_step_count() : 0U;
      const bool stepwise_environment = recovery_footprint::use_stepwise_escape_mode(
        cfg_.stuck_recovery.side_escape_enabled, snapshot.current_footprint_clear,
        current_sample.contact_cells.size(), snapshot.wall_region,
        completed_escape_steps);
      const bool clearance_reassessment_step =
        stepwise_environment && snapshot.current_footprint_clear;
      const bool stepwise_candidate_mode =
        recovery_selected_reverse_primitive_.has_value() ?
        recovery_selected_stepwise_escape_ :
        stepwise_environment;
      const bool committed_stepwise_maneuver =
        stepwise_candidate_mode && recovery_selected_reverse_primitive_.has_value() &&
        stuck_recovery_core_ != nullptr &&
        stuck_recovery_core_->supervisor().escape_step_count() > 0U;

      const auto evaluate_candidate_with_steering =
        [&](const recovery_footprint::ReversePrimitive primitive,
        const double steering_magnitude_rad) {
          auto candidate_rollout = rollout;
          candidate_rollout.steering_angle_rad = steering_magnitude_rad;
          candidate_rollout.reverse_distance_m = stepwise_candidate_mode ?
            std::max(0.0, escape_step_distance_to_check_m) :
            (recovery_footprint::primitive_is_forward(primitive) ?
            std::max(0.0, forward_distance_to_check_m) :
            std::max(0.0, reverse_distance_to_check_m));
          return recovery_footprint::evaluate_recovery_candidate(
            *recovery_grid_, recovery_footprint_, recovery_pose, primitive, candidate_rollout,
            clearance_reassessment_step ?
            recovery_footprint::ContactEscapePolicy::RequireClear :
            stepwise_candidate_mode ?
            (committed_stepwise_maneuver ?
            recovery_footprint::ContactEscapePolicy::AllowNonWorsening :
            recovery_footprint::ContactEscapePolicy::RequireImprovement) :
            recovery_footprint::ContactEscapePolicy::RequireClear,
            stepwise_candidate_mode && !committed_stepwise_maneuver ?
            cfg_.stuck_recovery.side_escape_min_contact_reduction_ratio : 0.0);
        };
      const auto evaluate_candidate =
        [&](const recovery_footprint::ReversePrimitive primitive) {
          const double steering_magnitude_rad =
            recovery_selected_reverse_steering_angle_rad_.has_value() ?
            std::abs(recovery_selected_reverse_steering_angle_rad_.value()) :
            cfg_.stuck_recovery.reverse_steering_angle_rad;
          return evaluate_candidate_with_steering(primitive, steering_magnitude_rad);
        };
      const auto recovery_mpc_guidance =
        [&](const recovery_mpc::Direction direction) -> std::optional<double> {
          if (!cfg_.stuck_recovery.recovery_mpc_enabled ||
            cfg_.stuck_recovery.reverse_steering_angle_rad <= 0.0)
          {
            return std::nullopt;
          }
          auto planner_config = cfg_.stuck_recovery.recovery_mpc_config;
          planner_config.maximum_steering_angle_rad = std::min(
            planner_config.maximum_steering_angle_rad,
            cfg_.stuck_recovery.reverse_steering_angle_rad);
          planner_config.maximum_steering_change_rad = std::min(
            planner_config.maximum_steering_change_rad,
            2.0 * planner_config.maximum_steering_angle_rad);
          const auto plan = recovery_mpc::plan(
            planner_config,
            recovery_mpc::Request{
              direction,
              car_->spatial_state.e_y,
              car_->spatial_state.e_psi,
              car_->current_waypoint->kappa,
              cfg_.bicycle_length,
              0.0});
          if (!plan.valid) {
            return std::nullopt;
          }
          return plan.first_steering_tire_angle_rad;
        };
      const auto desired_reverse_mpc_steering = recovery_mpc_guidance(
        recovery_mpc::Direction::Reverse);
      const auto desired_forward_mpc_steering = recovery_mpc_guidance(
        recovery_mpc::Direction::Forward);
      const auto steering_samples = recovery_footprint::steering_magnitude_samples(
        cfg_.stuck_recovery.reverse_steering_angle_rad,
        cfg_.stuck_recovery.side_escape_steering_samples);
      std::optional<recovery_footprint::FeasibilityResult> first_result;
      std::optional<recovery_footprint::FeasibilityResult> selected_result;
      const auto select_heading_aligned_reverse_candidate = [&]() {
          std::optional<recovery_footprint::FeasibilityResult> best_result;
          double best_score = std::numeric_limits<double>::infinity();
          const auto consider = [&](recovery_footprint::FeasibilityResult result) {
              if (!first_result.has_value()) {
                first_result = result;
              }
              if (!result.feasible || result.rollout.empty()) {
                return;
              }
              const double yaw_delta = wrap_to_pi(
                result.rollout.back().pose.yaw_rad - recovery_pose.yaw_rad);
              const double candidate_heading_error = std::isfinite(recovery_heading_error_rad) ?
                std::abs(wrap_to_pi(recovery_heading_error_rad + yaw_delta)) :
                std::abs(yaw_delta);
              const double score = desired_reverse_mpc_steering.has_value() ?
                std::abs(
                result.steering_angle_rad - desired_reverse_mpc_steering.value()) +
                0.10 * candidate_heading_error : candidate_heading_error;
              if (!best_result.has_value() || score + kEps < best_score) {
                best_score = score;
                best_result = std::move(result);
              }
            };
          if (desired_reverse_mpc_steering.has_value() &&
            !recovery_selected_reverse_primitive_.has_value())
          {
            consider(evaluate_candidate_with_steering(
              recovery_footprint::ReversePrimitive::Straight, 0.0));
            for (const auto primitive : {
                recovery_footprint::ReversePrimitive::Left,
                recovery_footprint::ReversePrimitive::Right})
            {
              for (const double steering_magnitude_rad : steering_samples) {
                consider(evaluate_candidate_with_steering(primitive, steering_magnitude_rad));
              }
            }
            return best_result;
          }
          constexpr std::array<recovery_footprint::ReversePrimitive, 3> kReversePreference{
            recovery_footprint::ReversePrimitive::Straight,
            recovery_footprint::ReversePrimitive::Left,
            recovery_footprint::ReversePrimitive::Right};
          for (const auto primitive : kReversePreference) {
            consider(evaluate_candidate(primitive));
          }
          return best_result;
        };
      const auto select_forward_deadlock_fallback = [&]() {
          std::optional<recovery_footprint::FeasibilityResult> best_result;
          double best_score = std::numeric_limits<double>::infinity();
          const auto consider = [&](recovery_footprint::FeasibilityResult result) {
              if (!result.feasible || result.rollout.empty()) {
                return;
              }
              const double yaw_delta = wrap_to_pi(
                result.rollout.back().pose.yaw_rad - recovery_pose.yaw_rad);
              const double candidate_heading_error = std::isfinite(recovery_heading_error_rad) ?
                std::abs(wrap_to_pi(recovery_heading_error_rad + yaw_delta)) :
                std::abs(yaw_delta);
              const double score = desired_forward_mpc_steering.has_value() ?
                std::abs(
                result.steering_angle_rad - desired_forward_mpc_steering.value()) +
                0.10 * candidate_heading_error : candidate_heading_error;
              if (!best_result.has_value() || score + kEps < best_score) {
                best_score = score;
                best_result = std::move(result);
              }
            };
          if (desired_forward_mpc_steering.has_value() &&
            !recovery_selected_reverse_primitive_.has_value())
          {
            consider(evaluate_candidate_with_steering(
              recovery_footprint::ReversePrimitive::ForwardStraight, 0.0));
            for (const auto primitive : {
                recovery_footprint::ReversePrimitive::ForwardLeft,
                recovery_footprint::ReversePrimitive::ForwardRight})
            {
              for (const double steering_magnitude_rad : steering_samples) {
                consider(evaluate_candidate_with_steering(primitive, steering_magnitude_rad));
              }
            }
            return best_result;
          }
          constexpr std::array<recovery_footprint::ReversePrimitive, 3> kForwardPreference{
            recovery_footprint::ReversePrimitive::ForwardStraight,
            recovery_footprint::ReversePrimitive::ForwardLeft,
            recovery_footprint::ReversePrimitive::ForwardRight};
          for (const auto primitive : kForwardPreference) {
            consider(evaluate_candidate(primitive));
          }
          return best_result;
        };
      if (recovery_selected_reverse_primitive_.has_value()) {
        auto result = evaluate_candidate(recovery_selected_reverse_primitive_.value());
        first_result = result;
        if (result.feasible) {
          selected_result = std::move(result);
        }
      } else if (stepwise_candidate_mode) {
        // Contact escape requires improvement. If the simulator reports a
        // persistent physical obstruction while the occupancy footprint is
        // clear, use the same stop-and-reassess cadence with RequireClear;
        // this avoids rejecting a safe 0.4 m step merely because the longer
        // 3.0 m maximum-distance rollout meets another wall later.
        if (clearance_reassessment_step) {
          selected_result = select_heading_aligned_reverse_candidate();
        } else {
          std::optional<recovery_footprint::FeasibilityResult> best_forward_step;
          const auto consider_step_candidate =
            [&](recovery_footprint::FeasibilityResult result,
            std::optional<recovery_footprint::FeasibilityResult> & best_result) {
              if (!first_result.has_value()) {
                first_result = result;
              }
              const auto & desired_steering = recovery_footprint::primitive_is_forward(
                result.primitive) ? desired_forward_mpc_steering :
                desired_reverse_mpc_steering;
              const auto guidance_score = [&](const auto & candidate) {
                  return desired_steering.has_value() ?
                         std::abs(
                    candidate.steering_angle_rad - desired_steering.value()) : 0.0;
                };
              if (result.feasible &&
                (!best_result.has_value() ||
                result.contact_reduction > best_result->contact_reduction ||
                (result.contact_reduction == best_result->contact_reduction &&
                desired_steering.has_value() &&
                guidance_score(result) + kEps < guidance_score(best_result.value()))))
              {
                best_result = std::move(result);
              }
            };
          consider_step_candidate(evaluate_candidate_with_steering(
            recovery_footprint::ReversePrimitive::Straight, 0.0), selected_result);
          for (const auto primitive : {
              recovery_footprint::ReversePrimitive::Left,
              recovery_footprint::ReversePrimitive::Right})
          {
            for (const double steering_magnitude_rad : steering_samples) {
              consider_step_candidate(evaluate_candidate_with_steering(
                primitive, steering_magnitude_rad), selected_result);
            }
          }
          // Mixed/corner contact does not prove that retreat is the improving
          // direction. Evaluate the same bounded step forward and choose it
          // only when the swept footprint removes more occupied contacts than
          // every reverse candidate. Reverse wins ties. If reverse is better
          // but V2X-blocked, retain the improving forward step as the existing
          // directional deadlock fallback.
          if (!reverse_only) {
            consider_step_candidate(evaluate_candidate_with_steering(
              recovery_footprint::ReversePrimitive::ForwardStraight, 0.0),
              best_forward_step);
            for (const auto primitive : {
                recovery_footprint::ReversePrimitive::ForwardLeft,
                recovery_footprint::ReversePrimitive::ForwardRight})
            {
              for (const double steering_magnitude_rad : steering_samples) {
                consider_step_candidate(evaluate_candidate_with_steering(
                  primitive, steering_magnitude_rad), best_forward_step);
              }
            }
          }
          if (best_forward_step.has_value()) {
            if (
              !selected_result.has_value() ||
              best_forward_step->contact_reduction > selected_result->contact_reduction)
            {
              selected_result = std::move(best_forward_step);
            } else if (!recovery_footprint::primitive_is_forward(selected_result->primitive)) {
              forward_deadlock_fallback = std::move(best_forward_step);
              forward_deadlock_fallback_stepwise = true;
            }
          }
        }
        if (clearance_reassessment_step) {
          auto forward_result = select_forward_deadlock_fallback();
          if (forward_result.has_value()) {
            if (selected_result.has_value()) {
              forward_deadlock_fallback = std::move(forward_result.value());
              forward_deadlock_fallback_stepwise = true;
            } else {
              selected_result = std::move(forward_result.value());
            }
          }
        }
      } else if (
        snapshot.current_footprint_clear &&
        (snapshot.wall_region == recovery_footprint::WallRegion::Left ||
        snapshot.wall_region == recovery_footprint::WallRegion::Right ||
        snapshot.wall_region == recovery_footprint::WallRegion::Mixed))
      {
        // A physical side contact can be reported while the conservative map
        // footprint is still clear. Keep the normal retreat preference, but
        // also retain a statically clear forward escape for the multi-vehicle
        // case where a stopped follower occupies the reverse corridor.
        selected_result = select_heading_aligned_reverse_candidate();
        if (!reverse_only) {
          auto forward_result = select_forward_deadlock_fallback();
          if (forward_result.has_value()) {
            if (selected_result.has_value()) {
              forward_deadlock_fallback = std::move(forward_result.value());
            } else {
              selected_result = std::move(forward_result.value());
            }
          }
        }
      } else if (snapshot.wall_region == recovery_footprint::WallRegion::Rear) {
        if (!reverse_only) {
          auto result = evaluate_candidate(
            recovery_footprint::ReversePrimitive::ForwardStraight);
          first_result = result;
          if (result.feasible) {
            selected_result = std::move(result);
          }
        }
      } else if (snapshot.wall_region == recovery_footprint::WallRegion::Front) {
        if (snapshot.current_footprint_clear) {
          selected_result = select_heading_aligned_reverse_candidate();
        } else {
          constexpr std::array<recovery_footprint::ReversePrimitive, 3> kPrimitivePreference{
            recovery_footprint::ReversePrimitive::Straight,
            recovery_footprint::ReversePrimitive::Left,
            recovery_footprint::ReversePrimitive::Right};
          for (const auto primitive : kPrimitivePreference) {
            auto result = evaluate_candidate(primitive);
            if (!first_result.has_value()) {
              first_result = result;
            }
            if (result.feasible) {
              selected_result = std::move(result);
              break;
            }
          }
        }
      } else if (
        snapshot.wall_region == recovery_footprint::WallRegion::None &&
        snapshot.current_footprint_clear)
      {
        // The physical AWSIM wall and the occupancy map can disagree. An
        // evidence-free detector confirmation is allowed only with a healthy
        // solver and sustained no-progress. Provide a bounded straight retreat
        // when the complete static rollout itself is clear; the core still
        // requires that confirmation and the directional V2X corridor below.
        selected_result = select_heading_aligned_reverse_candidate();
      }

      // Wall classification can change while AWSIM settles. Until actuation is committed, retain
      // a statically safe short forward candidate for every clear-footprint reverse selection.
      // Contact escape is handled above by the stricter RequireImprovement comparison. Any
      // clear-footprint fallback and Front-wall candidate still has to pass the full swept-
      // footprint check, so this does not weaken the collision gate.
      if (
        !reverse_only && snapshot.current_footprint_clear && selected_result.has_value() &&
        !recovery_footprint::primitive_is_forward(selected_result->primitive) &&
        !forward_deadlock_fallback.has_value())
      {
        auto forward_result = select_forward_deadlock_fallback();
        if (forward_result.has_value()) {
          forward_deadlock_fallback = std::move(forward_result.value());
          forward_deadlock_fallback_stepwise = stepwise_candidate_mode;
        }
      }

      snapshot.rear_static_clear = selected_result.has_value();
      if (first_result.has_value()) {
        const auto & diagnostic_result = selected_result.has_value() ?
          selected_result.value() : first_result.value();
        snapshot.static_reject_reason = diagnostic_result.reason;
        snapshot.static_initial_contact_count = diagnostic_result.initial_contact_count;
        snapshot.static_maximum_contact_count = diagnostic_result.maximum_contact_count;
        snapshot.static_final_contact_count = diagnostic_result.final_contact_count;
        snapshot.static_checked_pose_count = diagnostic_result.checked_pose_count;
        snapshot.static_rejected_at_distance_m = diagnostic_result.rejected_at_distance_m;
      }
      if (selected_result.has_value()) {
        snapshot.reverse_candidate_selected = true;
        snapshot.stepwise_escape = stepwise_candidate_mode;
        snapshot.contact_reduction = selected_result->contact_reduction;
        snapshot.selected_reverse_primitive = selected_result->primitive;
        snapshot.maneuver_direction = recovery_footprint::primitive_is_forward(
          selected_result->primitive) ?
          stuck_recovery::ManeuverDirection::Forward :
          stuck_recovery::ManeuverDirection::Reverse;
        snapshot.selected_reverse_steering_angle_rad =
          selected_result->steering_angle_rad;
        const auto & selected_mpc_guidance =
          snapshot.maneuver_direction == stuck_recovery::ManeuverDirection::Forward ?
          desired_forward_mpc_steering : desired_reverse_mpc_steering;
        snapshot.recovery_mpc_guidance_used = selected_mpc_guidance.has_value();
        snapshot.recovery_mpc_desired_steering_angle_rad =
          selected_mpc_guidance.value_or(0.0);
        selected_v2x_rollout = selected_result->rollout;
        const double cos_initial = std::cos(recovery_pose.yaw_rad);
        const double sin_initial = std::sin(recovery_pose.yaw_rad);
        for (const auto & rollout_pose : selected_result->rollout) {
          const double dx = rollout_pose.pose.x_m - recovery_pose.x_m;
          const double dy = rollout_pose.pose.y_m - recovery_pose.y_m;
          const double lateral = -sin_initial * dx + cos_initial * dy;
          snapshot.selected_center_min_lateral_m =
            std::min(snapshot.selected_center_min_lateral_m, lateral);
          snapshot.selected_center_max_lateral_m =
            std::max(snapshot.selected_center_max_lateral_m, lateral);
        }
      }
    }

    if (!evaluate_rollout) {
      return snapshot;
    }

    snapshot.boost_inactive_confirmed = recovery_boost_inactive_confirmed(steady_now);
    snapshot.v2x_message_complete =
      v2x_gap_planner_ != nullptr && cfg_.stuck_recovery.expected_v2x_vehicle_count >= 0 &&
      cfg_.stuck_recovery.v2x_self_filter_mode != "unknown" &&
      v2x_gap_planner_->has_complete_message(
      ros_now_sec,
      static_cast<std::size_t>(cfg_.stuck_recovery.expected_v2x_vehicle_count));
    if (!snapshot.boost_inactive_confirmed || !snapshot.v2x_message_complete)
    {
      return snapshot;
    }

    const auto active_vehicles = v2x_gap_planner_->active_vehicles(ros_now_sec);
    if (cfg_.stuck_recovery.v2x_self_filter_mode == "vehicle_id") {
      const std::size_t self_matches = static_cast<std::size_t>(std::count_if(
          active_vehicles.begin(), active_vehicles.end(), [this](const auto & vehicle) {
            return vehicle.id == cfg_.stuck_recovery.self_vehicle_id;
          }));
      if (self_matches != 1U) {
        snapshot.rear_information_complete = false;
        snapshot.rear_v2x_clear = false;
        return snapshot;
      }
    }
    const auto v2x_corridor_status =
      [&](const stuck_recovery::ManeuverDirection direction,
      const double selected_center_min_lateral_m,
      const double selected_center_max_lateral_m,
      const std::vector<recovery_footprint::RolloutPose> * selected_rollout)
      -> std::pair<bool, bool> {
        snapshot.v2x_blocking_vehicle_id.clear();
        snapshot.v2x_clearance_mode = "none";
        snapshot.v2x_clearance_reason = "none";
        snapshot.v2x_initial_clearance_m = std::numeric_limits<double>::infinity();
        snapshot.v2x_minimum_clearance_m = std::numeric_limits<double>::infinity();
        snapshot.v2x_final_clearance_m = std::numeric_limits<double>::infinity();
        snapshot.v2x_rejected_at_distance_m = 0.0;
        const bool forward_maneuver =
          direction == stuck_recovery::ManeuverDirection::Forward;
        const double prediction_horizon_sec =
          (forward_maneuver ?
          cfg_.stuck_recovery.core.supervisor.max_forward_duration_sec :
          cfg_.stuck_recovery.core.supervisor.max_reverse_duration_sec) +
          cfg_.stuck_recovery.rear_prediction_margin_sec;
        const double cos_yaw = std::cos(pose.theta);
        const double sin_yaw = std::sin(pose.theta);
        // A vehicle that remains strictly behind cannot be hit by a straight
        // forward escape; including the ego rear overhang here creates a
        // permanent deadlock with a conservatively inflated stopped follower.
        const double corridor_min_longitudinal = forward_maneuver ?
          0.0 :
          -cfg_.stuck_recovery.rear_extent_m - cfg_.stuck_recovery.footprint_margin_m -
          cfg_.stuck_recovery.core.supervisor.max_reverse_distance_m;
        const double corridor_max_longitudinal = forward_maneuver ?
          cfg_.stuck_recovery.front_extent_m + cfg_.stuck_recovery.footprint_margin_m +
          cfg_.stuck_recovery.core.supervisor.max_forward_distance_m :
          cfg_.stuck_recovery.front_extent_m + cfg_.stuck_recovery.footprint_margin_m;
        const double corridor_min_lateral =
          -cfg_.stuck_recovery.right_extent_m - cfg_.stuck_recovery.footprint_margin_m +
          selected_center_min_lateral_m;
        const double corridor_max_lateral =
          cfg_.stuck_recovery.left_extent_m + cfg_.stuck_recovery.footprint_margin_m +
          selected_center_max_lateral_m;
        const double vehicle_radius_m = cfg_.stuck_recovery.rear_vehicle_radius_m;

        for (const auto & vehicle : active_vehicles) {
          if (
            cfg_.stuck_recovery.v2x_self_filter_mode == "vehicle_id" &&
            vehicle.id == cfg_.stuck_recovery.self_vehicle_id)
          {
            continue;
          }
          const double dx = vehicle.x - pose.x;
          const double dy = vehicle.y - pose.y;
          if (!std::isfinite(dx) || !std::isfinite(dy) || !std::isfinite(vehicle.vx) ||
            !std::isfinite(vehicle.vy) || vehicle.position_jump || vehicle.invalid_velocity)
          {
            snapshot.v2x_blocking_vehicle_id = vehicle.id;
            snapshot.v2x_clearance_mode = "invalid_v2x";
            snapshot.v2x_clearance_reason = "incomplete_or_invalid_vehicle";
            return {false, false};
          }
          const double predicted_dx = dx + vehicle.vx * prediction_horizon_sec;
          const double predicted_dy = dy + vehicle.vy * prediction_horizon_sec;
          const double current_longitudinal = cos_yaw * dx + sin_yaw * dy;
          const double current_lateral = -sin_yaw * dx + cos_yaw * dy;
          const double predicted_longitudinal =
            cos_yaw * predicted_dx + sin_yaw * predicted_dy;
          const double predicted_lateral =
            -sin_yaw * predicted_dx + cos_yaw * predicted_dy;
          const double uncertainty_margin_m =
            std::max(vehicle.covariance_x, vehicle.covariance_y);
          const double inflated_vehicle_radius_m = vehicle_radius_m + uncertainty_margin_m;
          const double vehicle_speed_mps = std::hypot(vehicle.vx, vehicle.vy);
          if (
            selected_rollout != nullptr && !selected_rollout->empty() &&
            vehicle_speed_mps <= cfg_.stuck_recovery.coordinated_stop_front_speed_mps)
          {
            const auto rollout_clearance =
              recovery_footprint::evaluate_circle_obstacle_clearance(
              recovery_footprint_, *selected_rollout,
              recovery_footprint::CircleObstacle{
                vehicle.x, vehicle.y, vehicle.vx, vehicle.vy,
                inflated_vehicle_radius_m},
              prediction_horizon_sec);
            if (!rollout_clearance.valid) {
              snapshot.v2x_blocking_vehicle_id = vehicle.id;
              snapshot.v2x_clearance_mode = "rollout_separation";
              snapshot.v2x_clearance_reason =
                recovery_footprint::to_string(rollout_clearance.reason);
              return {false, false};
            }
            if (!rollout_clearance.clear) {
              snapshot.v2x_blocking_vehicle_id = vehicle.id;
              snapshot.v2x_clearance_mode = "rollout_separation";
              snapshot.v2x_clearance_reason =
                recovery_footprint::to_string(rollout_clearance.reason);
              snapshot.v2x_initial_clearance_m = rollout_clearance.initial_clearance_m;
              snapshot.v2x_minimum_clearance_m = rollout_clearance.minimum_clearance_m;
              snapshot.v2x_final_clearance_m = rollout_clearance.final_clearance_m;
              snapshot.v2x_rejected_at_distance_m =
                rollout_clearance.rejected_at_distance_m;
              return {true, false};
            }
            continue;
          }
          const double vehicle_min_longitudinal =
            std::min(current_longitudinal, predicted_longitudinal) - inflated_vehicle_radius_m;
          const double vehicle_max_longitudinal =
            std::max(current_longitudinal, predicted_longitudinal) + inflated_vehicle_radius_m;
          const double vehicle_min_lateral =
            std::min(current_lateral, predicted_lateral) - inflated_vehicle_radius_m;
          const double vehicle_max_lateral =
            std::max(current_lateral, predicted_lateral) + inflated_vehicle_radius_m;
          const bool longitudinal_overlap =
            vehicle_min_longitudinal <= corridor_max_longitudinal &&
            vehicle_max_longitudinal >= corridor_min_longitudinal;
          const bool lateral_overlap =
            vehicle_min_lateral <= corridor_max_lateral &&
            vehicle_max_lateral >= corridor_min_lateral;
          if (longitudinal_overlap && lateral_overlap) {
            snapshot.v2x_blocking_vehicle_id = vehicle.id;
            snapshot.v2x_clearance_mode = "moving_corridor";
            snapshot.v2x_clearance_reason = "corridor_overlap";
            return {true, false};
          }
        }
        return {true, true};
      };

    auto corridor_status = v2x_corridor_status(
      snapshot.maneuver_direction, snapshot.selected_center_min_lateral_m,
      snapshot.selected_center_max_lateral_m,
      selected_v2x_rollout.has_value() ? &selected_v2x_rollout.value() : nullptr);
    snapshot.rear_information_complete = corridor_status.first;
    snapshot.rear_v2x_clear = corridor_status.second;

    const bool prefer_alternate_forward =
      cfg_.stuck_recovery.core.supervisor.aggressive_sim_recovery_enabled &&
      recovery_aggressive_retry_count_ % 2U == 1U;
    if (
      snapshot.rear_information_complete && !snapshot.rear_v2x_clear &&
      snapshot.maneuver_direction == stuck_recovery::ManeuverDirection::Reverse &&
      !recovery_selected_reverse_primitive_.has_value() &&
      forward_deadlock_fallback.has_value())
    {
      double forward_min_lateral_m = 0.0;
      double forward_max_lateral_m = 0.0;
      const double cos_initial = std::cos(recovery_pose.yaw_rad);
      const double sin_initial = std::sin(recovery_pose.yaw_rad);
      for (const auto & rollout_pose : forward_deadlock_fallback->rollout) {
        const double dx = rollout_pose.pose.x_m - recovery_pose.x_m;
        const double dy = rollout_pose.pose.y_m - recovery_pose.y_m;
        const double lateral = -sin_initial * dx + cos_initial * dy;
        forward_min_lateral_m = std::min(forward_min_lateral_m, lateral);
        forward_max_lateral_m = std::max(forward_max_lateral_m, lateral);
      }
      const auto forward_corridor_status = v2x_corridor_status(
        stuck_recovery::ManeuverDirection::Forward,
        forward_min_lateral_m, forward_max_lateral_m,
        &forward_deadlock_fallback->rollout);
      if (forward_corridor_status.first && forward_corridor_status.second) {
        const auto & result = forward_deadlock_fallback.value();
        snapshot.static_reject_reason = result.reason;
        snapshot.static_initial_contact_count = result.initial_contact_count;
        snapshot.static_maximum_contact_count = result.maximum_contact_count;
        snapshot.static_final_contact_count = result.final_contact_count;
        snapshot.static_checked_pose_count = result.checked_pose_count;
        snapshot.static_rejected_at_distance_m = result.rejected_at_distance_m;
        snapshot.reverse_candidate_selected = true;
        snapshot.stepwise_escape = forward_deadlock_fallback_stepwise;
        snapshot.contact_reduction = result.contact_reduction;
        snapshot.selected_reverse_primitive = result.primitive;
        snapshot.maneuver_direction = stuck_recovery::ManeuverDirection::Forward;
        snapshot.selected_reverse_steering_angle_rad =
          result.steering_angle_rad;
        snapshot.selected_center_min_lateral_m = forward_min_lateral_m;
        snapshot.selected_center_max_lateral_m = forward_max_lateral_m;
        snapshot.rear_information_complete = true;
        snapshot.rear_v2x_clear = true;
      }
    }
    if (
      prefer_alternate_forward && snapshot.rear_information_complete &&
      snapshot.maneuver_direction == stuck_recovery::ManeuverDirection::Reverse &&
      !recovery_selected_reverse_primitive_.has_value() &&
      forward_deadlock_fallback.has_value())
    {
      double forward_min_lateral_m = 0.0;
      double forward_max_lateral_m = 0.0;
      const double cos_initial = std::cos(recovery_pose.yaw_rad);
      const double sin_initial = std::sin(recovery_pose.yaw_rad);
      for (const auto & rollout_pose : forward_deadlock_fallback->rollout) {
        const double dx = rollout_pose.pose.x_m - recovery_pose.x_m;
        const double dy = rollout_pose.pose.y_m - recovery_pose.y_m;
        const double lateral = -sin_initial * dx + cos_initial * dy;
        forward_min_lateral_m = std::min(forward_min_lateral_m, lateral);
        forward_max_lateral_m = std::max(forward_max_lateral_m, lateral);
      }
      const auto forward_corridor_status = v2x_corridor_status(
        stuck_recovery::ManeuverDirection::Forward,
        forward_min_lateral_m, forward_max_lateral_m,
        &forward_deadlock_fallback->rollout);
      if (forward_corridor_status.first && forward_corridor_status.second) {
        const auto & result = forward_deadlock_fallback.value();
        snapshot.static_reject_reason = result.reason;
        snapshot.static_initial_contact_count = result.initial_contact_count;
        snapshot.static_maximum_contact_count = result.maximum_contact_count;
        snapshot.static_final_contact_count = result.final_contact_count;
        snapshot.static_checked_pose_count = result.checked_pose_count;
        snapshot.static_rejected_at_distance_m = result.rejected_at_distance_m;
        snapshot.reverse_candidate_selected = true;
        snapshot.stepwise_escape = forward_deadlock_fallback_stepwise;
        snapshot.contact_reduction = result.contact_reduction;
        snapshot.selected_reverse_primitive = result.primitive;
        snapshot.maneuver_direction = stuck_recovery::ManeuverDirection::Forward;
        snapshot.selected_reverse_steering_angle_rad = result.steering_angle_rad;
        snapshot.selected_center_min_lateral_m = forward_min_lateral_m;
        snapshot.selected_center_max_lateral_m = forward_max_lateral_m;
        snapshot.rear_information_complete = true;
        snapshot.rear_v2x_clear = true;
      }
    }
    return snapshot;
  }

  void update_recovery_observation_anchor(
    const Pose2D & pose, const double progress_m,
    const stuck_recovery::StuckRejectReason reason)
  {
    const bool keep_window =
      reason == stuck_recovery::StuckRejectReason::ObservationWindowIncomplete ||
      reason == stuck_recovery::StuckRejectReason::MissingCorroboratingEvidence ||
      reason == stuck_recovery::StuckRejectReason::None;
    if (!keep_window) {
      recovery_observation_anchor_pose_ = pose;
      recovery_observation_anchor_progress_m_ = progress_m;
    }
  }

  std::optional<stuck_recovery::CoreOutput> evaluate_stuck_recovery(
    const Pose2D & pose, const double actual_v, const Eigen::Vector2d & normal_u,
    const double normal_acc, const double path_forward_intent_speed_mps,
    const bool mpc_fallback_active, const SteadyClock::time_point steady_now,
    const rclcpp::Time & control_time)
  {
    if (!cfg_.stuck_recovery.core.enabled || !stuck_recovery_core_) {
      return std::nullopt;
    }
    if (!recovery_observation_anchor_pose_.has_value()) {
      recovery_observation_anchor_pose_ = pose;
      recovery_observation_anchor_progress_m_ = car_->s;
    }
    const auto & anchor_pose = recovery_observation_anchor_pose_.value();
    const double pose_displacement_m = std::hypot(pose.x - anchor_pose.x, pose.y - anchor_pose.y);
    const double progress_delta_m = recovery_progress_delta(car_->s);
    const auto supervisor_state = stuck_recovery_core_->supervisor().state();
    const bool recovery_motion_phase =
      recovery_maneuver_start_pose_.has_value() &&
      (supervisor_state == stuck_recovery::RecoveryState::ReverseManeuver ||
      supervisor_state == stuck_recovery::RecoveryState::ForwardManeuver);
    if (recovery_motion_phase) {
      if (recovery_last_reverse_pose_.has_value()) {
        const double pose_step_m = std::hypot(
          pose.x - recovery_last_reverse_pose_->x,
          pose.y - recovery_last_reverse_pose_->y);
        const double yaw_step_rad = std::abs(
          wrap_to_pi(pose.theta - recovery_last_reverse_pose_->theta));
        const double footprint_corner_radius_m = std::hypot(
          std::max(
            recovery_footprint_.front_extent_m + recovery_footprint_.margin_m,
            recovery_footprint_.rear_extent_m + recovery_footprint_.margin_m),
          std::max(
            recovery_footprint_.left_extent_m + recovery_footprint_.margin_m,
            recovery_footprint_.right_extent_m + recovery_footprint_.margin_m));
        const double corner_motion_m = pose_step_m + footprint_corner_radius_m * yaw_step_rad;
        const double maximum_runtime_corner_motion_m = std::min(
          cfg_.stuck_recovery.max_reverse_pose_step_m,
          cfg_.stuck_recovery.sweep_interpolation_step_m);
        if (
          !std::isfinite(corner_motion_m) ||
          corner_motion_m > maximum_runtime_corner_motion_m)
        {
          recovery_reverse_pose_jump_ = true;
        } else {
          recovery_cumulative_reverse_distance_m_ += pose_step_m;
          recovery_episode_traveled_distance_m_ += pose_step_m;
        }
      }
      recovery_last_reverse_pose_ = pose;
    }
    const double traveled_distance_m = recovery_cumulative_reverse_distance_m_;
    const double episode_traveled_distance_m = recovery_episode_traveled_distance_m_;
    double reverse_stopping_reserve_m = 0.0;
    if (cfg_.stuck_recovery.reverse_actuation_enabled) {
      reverse_stopping_reserve_m = stuck_recovery::reverse_stopping_distance_reserve_m(
        reverse_actuation_calibration(cfg_.stuck_recovery), actual_v,
        1.0 / mpc_cfg_.control_rate);
    }
    const bool recovery_context_active =
      supervisor_state != stuck_recovery::RecoveryState::Normal;
    // normal_u[0] can alternate between zero and the path demand while a
    // stopped overtake/recovery target is being rebuilt. Keep the reference
    // path demand as the stable intent source so the no-progress timer is not
    // reset every other control cycle.
    const double requested_forward_speed_mps = std::max(
      {0.0, path_forward_intent_speed_mps, normal_u[0]});
    const auto & behavior = mpc_->last_v2x_behavior_output();
    const bool deliberate_stop =
      behavior.state == V2XBehaviorState::LowSpeedAvoidance ||
      behavior.state == V2XBehaviorState::SafetyBrake ||
      (behavior.state == V2XBehaviorState::Follow && behavior.has_front_vehicle) ||
      behavior.has_danger_vehicle;
    const bool coordinated_stop_candidate =
      cfg_.stuck_recovery.core.detector.coordinated_stop_recovery_enabled &&
      behavior.has_front_vehicle &&
      !behavior.target_vehicle_id.empty() && std::isfinite(behavior.front_speed) &&
      behavior.front_speed <= cfg_.stuck_recovery.coordinated_stop_front_speed_mps &&
      (behavior.state == V2XBehaviorState::SafetyBrake ||
      behavior.state == V2XBehaviorState::Follow);
    const bool coordinated_stop_active =
      coordinated_stop_candidate || recovery_coordinated_stop_episode_;
    bool current_wall_evidence = false;
    if (recovery_grid_ && recovery_footprint_.valid()) {
      const auto wall_proximity = recovery_footprint::classify_nearby_wall(
        *recovery_grid_, recovery_footprint_,
        recovery_footprint::Pose2D{pose.x, pose.y, pose.theta},
        cfg_.stuck_recovery.wall_direction_search_margin_m,
        cfg_.stuck_recovery.wall_direction_ambiguity_m);
      current_wall_evidence =
        wall_proximity.valid &&
        wall_proximity.region != recovery_footprint::WallRegion::None &&
        wall_proximity.region != recovery_footprint::WallRegion::Unknown;
    }
    const bool solver_reverse_only_candidate =
      stuck_recovery::solver_fallback_requires_reverse_only(
      mpc_fallback_active,
      cfg_.stuck_recovery.core.detector.solver_evidence_free_recovery_enabled,
      current_wall_evidence, car_->spatial_state.e_psi,
      cfg_.stuck_recovery.solver_reverse_only_heading_error_rad);
    const bool reverse_only =
      !cfg_.stuck_recovery.core.supervisor.aggressive_sim_recovery_enabled &&
      (recovery_reverse_only_episode_ || coordinated_stop_active ||
      solver_reverse_only_candidate);
    const bool low_speed_recovery_candidate =
      std::abs(actual_v) <= cfg_.stuck_recovery.core.detector.moving_speed_mps &&
      (requested_forward_speed_mps >=
      cfg_.stuck_recovery.core.detector.forward_intent_speed_mps ||
      normal_acc >= cfg_.stuck_recovery.core.detector.forward_intent_acceleration_mps2);
    const double reverse_distance_to_check_m = std::max(
      0.0,
      cfg_.stuck_recovery.core.supervisor.max_reverse_distance_m -
      episode_traveled_distance_m);
    const double forward_distance_to_check_m = std::max(
      0.0,
      cfg_.stuck_recovery.core.supervisor.max_forward_distance_m -
      episode_traveled_distance_m);
    const double escape_step_distance_to_check_m = std::max(
      0.0, cfg_.stuck_recovery.core.supervisor.escape_step_distance_m - traveled_distance_m);
    const auto rejoin_steering_tire_angle = recovery_rejoin_steering_tire_angle(normal_u);
    const double checked_rejoin_steering_tire_angle_rad =
      rejoin_steering_tire_angle.value_or(0.0);
    const auto safety = evaluate_recovery_safety(
      pose, steady_now, control_time.seconds(), reverse_distance_to_check_m,
      forward_distance_to_check_m,
      escape_step_distance_to_check_m,
      car_->spatial_state.e_psi,
      checked_rejoin_steering_tire_angle_rad,
      reverse_only,
      recovery_context_active || low_speed_recovery_candidate);
    const bool collision_hint =
      last_collision_receipt_steady_.has_value() &&
      std::chrono::duration<double>(
      steady_now - last_collision_receipt_steady_.value()).count() < 5.0;
    const bool awsim_recovery_settling =
      last_collision_receipt_steady_.has_value() &&
      std::chrono::duration<double>(
      steady_now - last_collision_receipt_steady_.value()).count() <
      cfg_.stuck_recovery.core.supervisor.awsim_recovery_wait_sec;
    const bool gear_report_fresh = recovery_gear_report_fresh(steady_now);

    stuck_recovery::CoreInput input;
    input.simulation_environment = use_sim_time_;
    input.detector.now_sec = steady_seconds(steady_now);
    input.detector.race_started = race_started_;
    input.detector.control_enabled = enable_control_;
    input.detector.odometry_fresh = true;
    input.detector.solver_fallback = mpc_fallback_active;
    input.detector.deliberate_stop = deliberate_stop || recovery_coordinated_stop_episode_;
    input.detector.coordinated_stop = coordinated_stop_active;
    input.detector.gear_transition_active = recovery_gear_transition_active();
    input.detector.awsim_recovery_settling = awsim_recovery_settling;
    input.detector.signed_speed_mps = actual_v;
    input.detector.requested_forward_speed_mps = requested_forward_speed_mps;
    input.detector.requested_acceleration_mps2 = normal_acc;
    input.detector.pose_displacement_m = pose_displacement_m;
    input.detector.unwrapped_progress_delta_m = progress_delta_m;
    input.detector.wall_evidence = safety.wall_evidence;
    input.detector.collision_hint = collision_hint;
    input.recovery.reported_gear = reported_gear_.value_or(stuck_recovery::Gear::Unknown);
    input.recovery.gear_report_fresh = gear_report_fresh;
    input.recovery.maneuver_direction = safety.maneuver_direction;
    input.recovery.stepwise_escape = safety.stepwise_escape;
    input.recovery.step_contact_improved =
      recovery_initial_contact_cells_.has_value() &&
      (recovery_initial_contact_cells_->empty() ?
      safety.current_footprint_clear :
      safety.current_contact_count < recovery_initial_contact_cells_->size());
    input.recovery.rear_static_clear = safety.rear_static_clear;
    input.recovery.rear_v2x_clear = safety.rear_v2x_clear;
    input.recovery.rear_information_complete = safety.rear_information_complete;
    input.recovery.collision_worsening =
      safety.collision_worsening || recovery_reverse_pose_jump_;
    const double escape_distance_target_m =
      safety.maneuver_direction == stuck_recovery::ManeuverDirection::Forward ?
      cfg_.stuck_recovery.forward_escape_distance_m :
      cfg_.stuck_recovery.reverse_escape_distance_m;
    const bool aggressive_force_rejoin =
      cfg_.stuck_recovery.core.supervisor.aggressive_sim_recovery_enabled &&
      cfg_.stuck_recovery.aggressive_force_rejoin_after_retries > 0U &&
      recovery_aggressive_retry_count_ >=
      cfg_.stuck_recovery.aggressive_force_rejoin_after_retries &&
      safety.current_footprint_clear && rejoin_steering_tire_angle.has_value();
    input.recovery.recovery_escape_confirmed =
      safety.current_footprint_clear &&
      (episode_traveled_distance_m >= escape_distance_target_m || aggressive_force_rejoin);
    input.recovery.rejoin_safe = safety.current_footprint_clear;
    input.recovery.rejoin_forward_clear =
      rejoin_steering_tire_angle.has_value() &&
      (safety.rejoin_forward_static_clear || aggressive_force_rejoin);
    if (aggressive_force_rejoin) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Stuck recovery aggressive forced rejoin: retry=%zu, static=%s, "
        "e_y=%.3f m, e_psi=%.3f rad",
        recovery_aggressive_retry_count_,
        recovery_footprint::to_string(safety.rejoin_static_reject_reason),
        car_->spatial_state.e_y, car_->spatial_state.e_psi);
    }
    input.recovery.awsim_recovery_resolved =
      safety.current_footprint_clear &&
      (recovery_episode_had_contact_evidence_ ||
      std::abs(progress_delta_m) >=
      cfg_.stuck_recovery.core.detector.max_progress_delta_m);
    input.recovery.traveled_distance_m = traveled_distance_m + reverse_stopping_reserve_m;
    input.recovery.episode_traveled_distance_m =
      episode_traveled_distance_m + reverse_stopping_reserve_m;
    input.recovery.reverse_steering_tire_angle_rad =
      safety.selected_reverse_steering_angle_rad;
    input.recovery.rejoin_steering_tire_angle_rad =
      checked_rejoin_steering_tire_angle_rad;
    input.recovery.lateral_error_m = car_->spatial_state.e_y;
    input.recovery.heading_error_rad = car_->spatial_state.e_psi;

    auto output = stuck_recovery_core_->update(input);
    update_recovery_observation_anchor(pose, car_->s, output.detector.reject_reason);
    const auto previous_state = recovery_last_output_.has_value() ?
      recovery_last_output_->state : stuck_recovery::RecoveryState::Normal;
    const bool started_recovery_episode =
      output.state != stuck_recovery::RecoveryState::Normal &&
      previous_state == stuck_recovery::RecoveryState::Normal;
    if (started_recovery_episode) {
      recovery_episode_traveled_distance_m_ = 0.0;
      recovery_initial_contact_cells_ = safety.current_contact_cells;
      recovery_previous_contact_cells_ = safety.current_contact_cells;
      recovery_episode_had_contact_evidence_ =
        safety.wall_evidence || collision_hint || !safety.current_contact_cells.empty();
      recovery_coordinated_stop_episode_ = coordinated_stop_active;
      recovery_reverse_only_episode_ = reverse_only;
      recovery_boost_suppressed_for_session_ = true;
    }
    const bool restarted_escape_after_rejoin =
      previous_state == stuck_recovery::RecoveryState::LowSpeedRejoin &&
      output.state == stuck_recovery::RecoveryState::StopAndConfirm &&
      (output.state_reason == stuck_recovery::RecoveryReason::RejoinTimedOut ||
      output.state_reason == stuck_recovery::RecoveryReason::RejoinPathBlocked);
    if (restarted_escape_after_rejoin) {
      recovery_episode_traveled_distance_m_ = 0.0;
      recovery_maneuver_start_pose_.reset();
      recovery_last_reverse_pose_.reset();
      recovery_cumulative_reverse_distance_m_ = 0.0;
      recovery_reverse_pose_jump_ = false;
      recovery_initial_contact_cells_ = safety.current_contact_cells;
      recovery_previous_contact_cells_ = safety.current_contact_cells;
      recovery_selected_reverse_primitive_.reset();
      recovery_selected_reverse_steering_angle_rad_.reset();
      recovery_selected_stepwise_escape_ = false;
      RCLCPP_INFO(
        get_logger(),
        "Stuck recovery rejoin retry: reason=%s, escape progress reset for bounded reassessment",
        stuck_recovery::to_string(output.state_reason));
    }
    const bool restarted_after_aggressive_stop =
      previous_state == stuck_recovery::RecoveryState::SafeStop &&
      output.state == stuck_recovery::RecoveryState::StopAndConfirm &&
      output.state_reason == stuck_recovery::RecoveryReason::AggressiveRetry;
    if (restarted_after_aggressive_stop) {
      ++recovery_aggressive_retry_count_;
      recovery_episode_traveled_distance_m_ = 0.0;
      recovery_maneuver_start_pose_.reset();
      recovery_last_reverse_pose_.reset();
      recovery_cumulative_reverse_distance_m_ = 0.0;
      recovery_reverse_pose_jump_ = false;
      recovery_initial_contact_cells_ = safety.current_contact_cells;
      recovery_previous_contact_cells_ = safety.current_contact_cells;
      recovery_selected_reverse_primitive_.reset();
      recovery_selected_reverse_steering_angle_rad_.reset();
      recovery_selected_stepwise_escape_ = false;
      RCLCPP_WARN(
        get_logger(),
        "Stuck recovery aggressive retry: cycle=%zu, episode progress and candidate reset",
        recovery_aggressive_retry_count_);
    }
    const bool entered_step_reassessment =
      output.state == stuck_recovery::RecoveryState::CheckClearance &&
      (previous_state == stuck_recovery::RecoveryState::StopAndConfirm ||
      previous_state == stuck_recovery::RecoveryState::StopAndReassess ||
      previous_state == stuck_recovery::RecoveryState::ShiftToDrive ||
      previous_state == stuck_recovery::RecoveryState::WaitDriveReport);
    if (entered_step_reassessment) {
      recovery_selected_reverse_primitive_.reset();
      recovery_selected_reverse_steering_angle_rad_.reset();
      recovery_selected_stepwise_escape_ = false;
      recovery_maneuver_start_pose_.reset();
      recovery_last_reverse_pose_.reset();
      recovery_cumulative_reverse_distance_m_ = 0.0;
      recovery_reverse_pose_jump_ = false;
      recovery_initial_contact_cells_ = safety.current_contact_cells;
      recovery_previous_contact_cells_ = safety.current_contact_cells;
    }
    if (
      output.state != stuck_recovery::RecoveryState::Normal &&
      !entered_step_reassessment &&
      !recovery_selected_reverse_primitive_.has_value() &&
      stuck_recovery::recovery_candidate_commit_allowed(output.state) &&
      safety.reverse_candidate_selected)
    {
      recovery_selected_reverse_primitive_ = safety.selected_reverse_primitive;
      recovery_selected_reverse_steering_angle_rad_ =
        safety.selected_reverse_steering_angle_rad;
      recovery_selected_stepwise_escape_ = safety.stepwise_escape;
      RCLCPP_INFO(
        get_logger(), "Stuck recovery maneuver selected: direction=%s, primitive=%s, "
        "steering=%.3f rad, wall=%s, wall_distance=%.3f m, coordinated=%d, "
        "reverse_only=%d, recovery_mpc=%d, mpc_desired=%.3f rad, aggressive_retry=%zu",
        stuck_recovery::to_string(safety.maneuver_direction),
        recovery_footprint::to_string(safety.selected_reverse_primitive),
        safety.selected_reverse_steering_angle_rad,
        recovery_footprint::to_string(safety.wall_region), safety.wall_distance_m,
        recovery_coordinated_stop_episode_ ? 1 : 0,
        recovery_reverse_only_episode_ ? 1 : 0,
        safety.recovery_mpc_guidance_used ? 1 : 0,
        safety.recovery_mpc_desired_steering_angle_rad,
        recovery_aggressive_retry_count_);
    }
    const bool maneuver_state =
      output.state == stuck_recovery::RecoveryState::ReverseManeuver ||
      output.state == stuck_recovery::RecoveryState::ForwardManeuver;
    const bool previous_maneuver_state =
      previous_state == stuck_recovery::RecoveryState::ReverseManeuver ||
      previous_state == stuck_recovery::RecoveryState::ForwardManeuver;
    const bool resumed_reverse_maneuver =
      output.state == stuck_recovery::RecoveryState::ReverseManeuver &&
      previous_state == stuck_recovery::RecoveryState::WaitReverseReport &&
      recovery_maneuver_start_pose_.has_value();
    const bool started_recovery_maneuver =
      maneuver_state && !previous_maneuver_state && !resumed_reverse_maneuver;
    if (started_recovery_maneuver) {
      recovery_maneuver_start_pose_ = pose;
      recovery_last_reverse_pose_ = pose;
      recovery_cumulative_reverse_distance_m_ = 0.0;
      recovery_reverse_pose_jump_ = false;
      recovery_initial_contact_cells_ = safety.current_contact_cells;
      recovery_previous_contact_cells_ = safety.current_contact_cells;
    } else if (resumed_reverse_maneuver) {
      // Do not count pose drift while clearance was incomplete, and do not
      // reset cumulative progress/contact baselines for the current step.
      recovery_last_reverse_pose_ = pose;
      recovery_reverse_pose_jump_ = false;
    } else if (
      output.state != stuck_recovery::RecoveryState::Normal &&
      !started_recovery_episode && !started_recovery_maneuver &&
      (!recovery_previous_contact_cells_.has_value() ||
      safety.runtime_contact_reject_reason == recovery_footprint::RejectReason::None))
    {
      // Do not absorb a rejected contact transition into the next cycle's safety baseline.
      recovery_previous_contact_cells_ = safety.current_contact_cells;
    } else if (
      output.state == stuck_recovery::RecoveryState::Normal &&
      previous_state != stuck_recovery::RecoveryState::Normal)
    {
      recovery_initial_contact_cells_.reset();
      recovery_previous_contact_cells_.reset();
      recovery_episode_had_contact_evidence_ = false;
      recovery_coordinated_stop_episode_ = false;
      recovery_reverse_only_episode_ = false;
      recovery_aggressive_retry_count_ = 0U;
      recovery_selected_reverse_primitive_.reset();
      recovery_selected_reverse_steering_angle_rad_.reset();
      recovery_selected_stepwise_escape_ = false;
      recovery_episode_traveled_distance_m_ = 0.0;
    }
    if (output.action.reset_normal_control && !recovery_reset_applied_) {
      mpc_->reset_after_external_maneuver(control_time.seconds(), 0.0);
      last_u_ = Eigen::Vector2d::Zero();
      last_acc_ = 0.0;
      recovery_reset_applied_ = true;
      recovery_rejoin_hold_cycle_ = true;
    }
    if (output.state == stuck_recovery::RecoveryState::Normal &&
      output.state_reason == stuck_recovery::RecoveryReason::RejoinComplete)
    {
      recovery_initial_contact_cells_.reset();
      recovery_previous_contact_cells_.reset();
      recovery_maneuver_start_pose_.reset();
      recovery_last_reverse_pose_.reset();
      recovery_cumulative_reverse_distance_m_ = 0.0;
      recovery_episode_traveled_distance_m_ = 0.0;
      recovery_reverse_pose_jump_ = false;
      recovery_selected_reverse_primitive_.reset();
      recovery_selected_reverse_steering_angle_rad_.reset();
      recovery_selected_stepwise_escape_ = false;
      recovery_coordinated_stop_episode_ = false;
      recovery_reverse_only_episode_ = false;
      recovery_reset_applied_ = false;
    }

    if (!last_recovery_execution_mode_.has_value() ||
      last_recovery_execution_mode_.value() != output.execution_mode ||
      !last_recovery_state_.has_value() || last_recovery_state_.value() != output.state ||
      !recovery_last_output_.has_value() ||
      recovery_last_output_->state_reason != output.state_reason)
    {
      RCLCPP_INFO(
        get_logger(),
        "Stuck recovery: mode=%s, state=%s, action=%s, reason=%s, "
        "coordinated=%d, reverse_only=%d, static=%s, "
        "static_contacts=%zu/%zu/%zu, static_reject_at=%.3f m, checked=%zu, "
        "runtime_contact=%s, current_contacts=%zu, corridor_complete=%d, boost_inactive=%d, "
        "v2x_message_complete=%d, corridor_v2x_clear=%d, "
        "v2x_blocker=%s, v2x_gate=%s/%s, v2x_clearance=%.3f/%.3f/%.3f m, "
        "v2x_reject_at=%.3f m, "
        "wall=%s, wall_distance=%.3f, direction=%s, primitive=%s, steering=%.3f, "
        "stepwise=%d, contact_reduction=%zu, step=%zu/%zu, maneuver_distance=%.3f m, "
        "episode_distance=%.3f m, stopping_reserve=%.3f m, escape_target=%.3f m, "
        "escape_confirmed=%d, rejoin_safe=%d, rejoin_path_clear=%d, "
        "rejoin_static=%s, rejoin_reject_at=%.3f m, rejoin_steering=%.3f rad, "
        "e_y=%.3f m, e_psi=%.3f rad",
        stuck_recovery::to_string(output.execution_mode),
        stuck_recovery::to_string(output.state),
        stuck_recovery::to_string(output.action.type),
        stuck_recovery::to_string(output.action.reason),
        recovery_coordinated_stop_episode_ ? 1 : 0,
        recovery_reverse_only_episode_ ? 1 : 0,
        recovery_footprint::to_string(safety.static_reject_reason),
        safety.static_initial_contact_count, safety.static_maximum_contact_count,
        safety.static_final_contact_count, safety.static_rejected_at_distance_m,
        safety.static_checked_pose_count,
        recovery_footprint::to_string(safety.runtime_contact_reject_reason),
        safety.current_contact_count,
        safety.rear_information_complete ? 1 : 0,
        safety.boost_inactive_confirmed ? 1 : 0,
        safety.v2x_message_complete ? 1 : 0, safety.rear_v2x_clear ? 1 : 0,
        safety.v2x_blocking_vehicle_id.c_str(), safety.v2x_clearance_mode.c_str(),
        safety.v2x_clearance_reason.c_str(), safety.v2x_initial_clearance_m,
        safety.v2x_minimum_clearance_m, safety.v2x_final_clearance_m,
        safety.v2x_rejected_at_distance_m,
        recovery_footprint::to_string(safety.wall_region), safety.wall_distance_m,
        stuck_recovery::to_string(safety.maneuver_direction),
        safety.reverse_candidate_selected ?
        recovery_footprint::to_string(safety.selected_reverse_primitive) : "none",
        safety.selected_reverse_steering_angle_rad, safety.stepwise_escape ? 1 : 0,
        safety.contact_reduction,
        stuck_recovery_core_->supervisor().escape_step_count(),
        cfg_.stuck_recovery.core.supervisor.max_escape_steps,
        traveled_distance_m, episode_traveled_distance_m, reverse_stopping_reserve_m,
        escape_distance_target_m, input.recovery.recovery_escape_confirmed ? 1 : 0,
        input.recovery.rejoin_safe ? 1 : 0,
        input.recovery.rejoin_forward_clear ? 1 : 0,
        recovery_footprint::to_string(safety.rejoin_static_reject_reason),
        safety.rejoin_static_rejected_at_distance_m,
        input.recovery.rejoin_steering_tire_angle_rad, input.recovery.lateral_error_m,
        input.recovery.heading_error_rad);
    }
    if (!last_recovery_reject_reason_.has_value() ||
      last_recovery_reject_reason_.value() != output.detector.reject_reason)
    {
      RCLCPP_INFO(
        get_logger(),
        "Stuck detector: verdict=%s, reject=%s, stationary=%.2f s, pose=%.3f m, "
        "progress=%.3f m, evidence=%d, wall=%d, forward_intent=%d, "
        "solver_fallback=%d, fallback_duration=%.2f s, fallback_qualified=%d, "
        "solver_evidence_free_qualified=%d, evidence_free_qualified=%d, "
        "coordinated_stop_qualified=%d, coordinated_candidate=%d",
        stuck_recovery::to_string(output.detector.verdict),
        stuck_recovery::to_string(output.detector.reject_reason),
        output.detector.stationary_duration_sec, output.detector.pose_displacement_m,
        output.detector.progress_delta_m, output.detector.corroborating_evidence ? 1 : 0,
        safety.wall_evidence ? 1 : 0, output.detector.forward_intent ? 1 : 0,
        mpc_fallback_active ? 1 : 0, output.detector.solver_fallback_duration_sec,
        output.detector.solver_fallback_qualified ? 1 : 0,
        output.detector.solver_evidence_free_qualified ? 1 : 0,
        output.detector.evidence_free_qualified ? 1 : 0,
        output.detector.coordinated_stop_qualified ? 1 : 0,
        coordinated_stop_candidate ? 1 : 0);
    }
    if (output.state == stuck_recovery::RecoveryState::LowSpeedRejoin) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 500,
        "Stuck recovery rejoin: target_v=%.3f m/s, actual_v=%.3f m/s, "
        "steering=%.3f rad, e_y=%.3f m, e_psi=%.3f rad, static=%s, v2x_clear=%d",
        output.action.rejoin_speed_limit_mps, actual_v,
        input.recovery.rejoin_steering_tire_angle_rad,
        input.recovery.lateral_error_m, input.recovery.heading_error_rad,
        recovery_footprint::to_string(safety.rejoin_static_reject_reason),
        input.recovery.rear_v2x_clear ? 1 : 0);
    }
    last_recovery_execution_mode_ = output.execution_mode;
    last_recovery_state_ = output.state;
    last_recovery_reject_reason_ = output.detector.reject_reason;
    recovery_last_output_ = output;
    return output;
  }

  void publish_recovery_gear_request(
    const stuck_recovery::RecoveryAction & action, const rclcpp::Time & stamp)
  {
    if (action.requested_gear == stuck_recovery::Gear::NoCommand) {
      return;
    }
    if (!stuck_recovery_actuation_io_enabled_ || !gear_command_pub_) {
      RCLCPP_ERROR(
        get_logger(), "Stuck recovery gear request blocked by reverse actuation safety latch: %s",
        stuck_recovery::to_string(action.requested_gear));
      return;
    }
    const auto command_value = gear_command_value(action.requested_gear);
    if (!command_value.has_value()) {
      RCLCPP_ERROR(get_logger(), "Stuck recovery rejected an invalid gear request");
      return;
    }
    GearCommand command;
    command.stamp = stamp;
    command.command = command_value.value();
    gear_command_pub_->publish(command);
    last_commanded_recovery_gear_ = action.requested_gear;
    RCLCPP_WARN(
      get_logger(), "Stuck recovery gear requested: gear=%s raw=%u",
      stuck_recovery::to_string(action.requested_gear),
      static_cast<unsigned int>(command.command));
  }

  void maybe_request_drive_after_recovery_reset(
    const rclcpp::Time & stamp, const SteadyClock::time_point steady_now,
    const double signed_speed_mps)
  {
    if (!recovery_waiting_for_drive_after_reset_ || !stuck_recovery_actuation_io_enabled_ ||
      !gear_command_pub_ || !std::isfinite(signed_speed_mps))
    {
      return;
    }
    if (std::abs(signed_speed_mps) > cfg_.stuck_recovery.core.supervisor.stop_speed_mps) {
      recovery_reset_stopped_since_.reset();
      return;
    }
    if (!recovery_reset_stopped_since_.has_value()) {
      recovery_reset_stopped_since_ = steady_now;
      return;
    }
    const double stopped_duration_sec = std::chrono::duration<double>(
      steady_now - recovery_reset_stopped_since_.value()).count();
    if (
      stopped_duration_sec < cfg_.stuck_recovery.core.supervisor.stop_confirm_sec ||
      recovery_reset_drive_request_count_ >=
      cfg_.stuck_recovery.core.supervisor.max_gear_command_requests)
    {
      return;
    }
    stuck_recovery::RecoveryAction action;
    action.type = stuck_recovery::RecoveryActionType::RequestDrive;
    action.requested_gear = stuck_recovery::Gear::Drive;
    action.reason = stuck_recovery::RecoveryReason::DriveGearRequested;
    publish_recovery_gear_request(action, stamp);
    ++recovery_reset_drive_request_count_;
  }

  bool apply_stuck_recovery_arbitration(
    const stuck_recovery::CoreOutput & output, const double actual_v,
    const rclcpp::Time & stamp, Eigen::Vector2d & u, double & acc,
    bool & bug_acc_enabled)
  {
    if (!output.actuation_allowed ||
      output.action.type == stuck_recovery::RecoveryActionType::NormalControl)
    {
      return false;
    }

    recovery_boost_suppressed_for_session_ = true;
    bug_acc_enabled = false;
    if (output.action.type == stuck_recovery::RecoveryActionType::LowSpeedRejoin) {
      if (recovery_rejoin_hold_cycle_) {
        const double max_steering_step = mpc_cfg_.steer_rate_max / mpc_cfg_.control_rate;
        u[0] = 0.0;
        u[1] = clip(0.0, last_u_[1] - max_steering_step, last_u_[1] + max_steering_step);
        acc = mpc_cfg_.a_min;
        recovery_rejoin_hold_cycle_ = false;
        return true;
      }
      // LowSpeedRejoin is entered only after the bounded escape distance, Drive report,
      // static swept-footprint, V2X, and solver gates have all passed. The normal MPC
      // can still request zero after a physical contact; taking min(normal, limit) then
      // leaves the vehicle stationary until the rejoin timeout. Treat the configured
      // low-speed value as the recovery target while those gates remain continuously clear.
      u[0] = output.action.rejoin_speed_limit_mps;
      const double steering_gain = mpc_cfg_.steering_tire_angle_gain_var;
      u[1] = output.action.steering_tire_angle_rad / steering_gain;
      acc = clip(100.0 * (u[0] - actual_v), mpc_cfg_.a_min, mpc_cfg_.a_max);
      return true;
    }

    const double max_steering_step = mpc_cfg_.steer_rate_max / mpc_cfg_.control_rate;
    u[0] = 0.0;
    u[1] = clip(0.0, last_u_[1] - max_steering_step, last_u_[1] + max_steering_step);
    const bool reverse_reported_fresh =
      recovery_gear_report_fresh(SteadyClock::now()) && reported_gear_.has_value() &&
      reported_gear_.value() == stuck_recovery::Gear::Reverse;
    const bool drive_reported_fresh =
      recovery_gear_report_fresh(SteadyClock::now()) && reported_gear_.has_value() &&
      reported_gear_.value() == stuck_recovery::Gear::Drive;
    const bool reverse_possible = recovery_may_be_in_reverse();
    acc = reverse_possible ?
      (stuck_recovery_actuation_io_enabled_ ?
      reverse_actuation_calibration(cfg_.stuck_recovery).stop_acceleration_mps2 : 0.0) :
      mpc_cfg_.a_min;

    if (
      output.action.type == stuck_recovery::RecoveryActionType::RequestReverse ||
      output.action.type == stuck_recovery::RecoveryActionType::RequestDrive)
    {
      publish_recovery_gear_request(output.action, stamp);
    } else if (output.action.type == stuck_recovery::RecoveryActionType::ReverseCreep) {
      const bool reverse_speed_safe =
        std::isfinite(actual_v) &&
        cfg_.stuck_recovery.core.supervisor.max_reverse_speed_mps > 0.0 &&
        std::abs(actual_v) < cfg_.stuck_recovery.core.supervisor.max_reverse_speed_mps;
      if (
        !stuck_recovery_actuation_io_enabled_ || !reverse_reported_fresh ||
        !reverse_speed_safe)
      {
        acc = reverse_possible && stuck_recovery_actuation_io_enabled_ ?
          reverse_actuation_calibration(cfg_.stuck_recovery).stop_acceleration_mps2 : 0.0;
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Stuck recovery reverse command blocked: actuation=%d, reverse_report=%d, "
          "speed_safe=%d, speed=%.3f m/s, limit=%.3f m/s",
          stuck_recovery_actuation_io_enabled_ ? 1 : 0, reverse_reported_fresh ? 1 : 0,
          reverse_speed_safe ? 1 : 0, actual_v,
          cfg_.stuck_recovery.core.supervisor.max_reverse_speed_mps);
      } else {
        const double steering_gain =
          std::abs(mpc_cfg_.steering_tire_angle_gain_var) > kEps ?
          mpc_cfg_.steering_tire_angle_gain_var : 1.0;
        u[1] = output.action.steering_tire_angle_rad / steering_gain;
        const auto calibration = reverse_actuation_calibration(cfg_.stuck_recovery);
        acc = calibration.drive_acceleration_mps2;
      }
    } else if (output.action.type == stuck_recovery::RecoveryActionType::ForwardCreep) {
      const bool forward_speed_safe =
        std::isfinite(actual_v) &&
        cfg_.stuck_recovery.core.supervisor.max_forward_speed_mps > 0.0 &&
        std::abs(actual_v) < cfg_.stuck_recovery.core.supervisor.max_forward_speed_mps;
      if (!stuck_recovery_actuation_io_enabled_ || !drive_reported_fresh || !forward_speed_safe) {
        u[0] = 0.0;
        acc = mpc_cfg_.a_min;
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Stuck recovery forward command blocked: actuation=%d, drive_report=%d, "
          "speed_safe=%d, speed=%.3f m/s, limit=%.3f m/s",
          stuck_recovery_actuation_io_enabled_ ? 1 : 0, drive_reported_fresh ? 1 : 0,
          forward_speed_safe ? 1 : 0, actual_v,
          cfg_.stuck_recovery.core.supervisor.max_forward_speed_mps);
      } else {
        const double steering_gain =
          std::abs(mpc_cfg_.steering_tire_angle_gain_var) > kEps ?
          mpc_cfg_.steering_tire_angle_gain_var : 1.0;
        u[0] = cfg_.stuck_recovery.core.supervisor.max_forward_speed_mps;
        u[1] = output.action.steering_tire_angle_rad / steering_gain;
        acc = clip(
          output.action.acceleration_magnitude_mps2, 0.0, mpc_cfg_.a_max);
      }
    }
    return true;
  }

  void control()
  {
    const auto steady_now = SteadyClock::now();
    const auto control_time = now();
    const bool missing_odometry = !odom_ || !last_odom_receipt_steady_.has_value();
    const double odometry_age_sec = missing_odometry ?
      std::numeric_limits<double>::infinity() :
      std::chrono::duration<double>(
      steady_now - last_odom_receipt_steady_.value()).count();
    const double odometry_source_age_sec = last_odom_source_advance_steady_.has_value() ?
      std::chrono::duration<double>(
      steady_now - last_odom_source_advance_steady_.value()).count() : 0.0;
    const bool stale_source_stamp =
      last_odom_source_stamp_.has_value() &&
      odometry_source_age_sec > mpc_cfg_.odom_timeout_sec;
    if (
      missing_odometry || odometry_age_sec > mpc_cfg_.odom_timeout_sec ||
      stale_source_stamp)
    {
      if (!odom_failsafe_active_) {
        RCLCPP_ERROR(
          get_logger(),
          "Odometry is missing or stale: receipt_age=%.3f s, source_age=%.3f s, limit=%.3f s",
          odometry_age_sec, odometry_source_age_sec, mpc_cfg_.odom_timeout_sec);
      }
      odom_failsafe_active_ = true;
      publish_failsafe_command(control_time, "missing or stale odometry");
      if (initialized_) {
        last_t_ = control_time;
      }
      return;
    }
    if (odom_failsafe_active_) {
      RCLCPP_INFO(get_logger(), "Odometry recovered: age=%.3f s", odometry_age_sec);
      odom_failsafe_active_ = false;
    }
    if (recovery_fault_latched_) {
      maybe_request_drive_after_recovery_reset(
        control_time, steady_now, odom_->twist.twist.linear.x);
      publish_failsafe_command(control_time, "stuck recovery fault remains latched until session reset");
      return;
    }
    if (cfg_.reference_path.update_by_topic && !trajectory_) {
      publish_failsafe_command(control_time, "trajectory is not available");
      return;
    }

    const auto pose = odom_to_pose_2d(*odom_);
    const double actual_v = odom_->twist.twist.linear.x;
    if (
      !std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.theta) ||
      !std::isfinite(actual_v))
    {
      publish_failsafe_command(control_time, "non-finite odometry rejected");
      return;
    }
    if (!initialized_) {
      car_->update_states(pose.x, pose.y, pose.theta);
      car_->update_reference_path(reference_path_.get());
      last_t_ = control_time;
      initialized_ = true;
      pred_marker_color_.r = 0.0;
      pred_marker_color_.g = 156.0 / 255.0;
      pred_marker_color_.b = 209.0 / 255.0;
      pred_marker_color_.a = 1.0;
      publish_ref_path_marker();
      RCLCPP_INFO(get_logger(), "START!");
    }

    const auto current_time = control_time;
    const double raw_dt = (current_time - last_t_).seconds();
    const double nominal_dt = 1.0 / mpc_cfg_.control_rate;
    const double dt =
      std::isfinite(raw_dt) && raw_dt > 0.0 && raw_dt <= mpc_cfg_.odom_timeout_sec ?
      raw_dt : nominal_dt;
    if (dt != raw_dt) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Invalid control dt %.6f s; using nominal %.6f s", raw_dt, nominal_dt);
    }
    last_t_ = current_time;
    ++loop_;

    if (loop_ % 100 == 0 && cfg_.reference_path.update_by_topic && trajectory_) {
      try {
        auto new_reference_path = create_reference_path_from_autoware_trajectory(*trajectory_);
        if (!new_reference_path || new_reference_path->n_waypoints < 3) {
          throw std::runtime_error("trajectory did not produce a usable reference path");
        }
        auto previous_reference_path = std::move(reference_path_);
        reference_path_ = std::move(new_reference_path);
        try {
          car_->update_reference_path(reference_path_.get());
        } catch (...) {
          reference_path_ = std::move(previous_reference_path);
          car_->update_reference_path(reference_path_.get());
          throw;
        }
        ref_path_published_ = false;
      } catch (const std::exception & error) {
        RCLCPP_ERROR(
          get_logger(), "Trajectory update rejected; keeping last valid path: %s", error.what());
      }
    }
    if (!ref_path_published_) {
      publish_ref_path_marker();
    }

    bool is_colliding = false;
    if (last_colliding_time_.has_value()) {
      is_colliding = (current_time - last_colliding_time_.value()).seconds() < 5.0;
    }
    (void)is_colliding;

    car_->update_states(pose.x, pose.y, pose.theta);
    mpc_->update_current_speed(std::abs(actual_v));

    std::optional<double> elapsed_since_start;
    if (domain_start_epoch_.has_value()) {
      elapsed_since_start = (current_time - domain_start_epoch_.value()).seconds();
    }
    const auto speed_resolution = overtake_core::resolve_effective_speed_limit(
      overtake_core::SpeedLimitRequest{
        mpc_cfg_.v_max,
        mpc_cfg_.global_v_max,
        mpc_cfg_.domain_start_v_max_applied ?
        std::optional<double>{mpc_cfg_.domain_start_v_max} : std::nullopt,
        mpc_cfg_.domain_start_v_max_duration,
        elapsed_since_start});
    double effective_v_max = speed_resolution.speed_mps;
    if (
      !last_start_window_status_.has_value() ||
      last_start_window_status_.value() != speed_resolution.start_window_status)
    {
      RCLCPP_INFO(
        get_logger(),
        "MPC speed source changed: source=%s, normal=%.2f km/h, "
        "start=%.2f km/h, global=%.2f km/h, effective=%.2f km/h",
        overtake_core::to_string(speed_resolution.start_window_status), mpc_cfg_.v_max_kmh,
        mpc_cfg_.domain_start_v_max_applied ? mpc_cfg_.domain_start_v_max_kmh : -1.0,
        mpc_cfg_.global_v_max_kmh,
        effective_v_max * 3.6);
      last_start_window_status_ = speed_resolution.start_window_status;
    }
    if (ref_vel_configulator_) {
      const double ref_vel_kmph = ref_vel_configulator_->get_ref_vel(mpc_->model->wp_id);
      effective_v_max = std::min(kmh_to_m_per_sec(ref_vel_kmph), effective_v_max);
    }
    mpc_->update_v_max(effective_v_max);
    reference_path_->set_v_ref(std::vector<double>(reference_path_->waypoints.size(), effective_v_max));

    auto [u, max_delta] = mpc_->get_control(current_time.seconds());
    const bool mpc_fallback_active = mpc_->last_control_was_fallback();

    if (!enable_control_) {
      const double last_v_cmd = last_u_[0];
      if (last_v_cmd < 0.5) {
        u[0] = 0.0;
      } else {
        const double decel_v = last_v_cmd + mpc_cfg_.a_min * dt;
        u[0] = clip(decel_v, 0.0, mpc_cfg_.v_max);
      }
    }

    double acc = 0.0;
    bool bug_acc_enabled = false;
    const bool forced_stop_active = mpc_fallback_active || !enable_control_;
    if (forced_stop_active) {
      bug_acc_enabled = false;
      acc = mpc_cfg_.a_min;
    } else if (use_bug_acc_) {
      const auto deg2rad = [](const double deg) { return deg * kPi / 180.0; };
      if (
        std::abs(actual_v) > kmh_to_m_per_sec(44.0) ||
        (std::abs(actual_v) > kmh_to_m_per_sec(38.0) && std::abs(max_delta) > deg2rad(12.0))) {
        bug_acc_enabled = false;
        acc = mpc_cfg_.a_min / 3.0 * 2.0;
        pred_marker_color_.r = 1.0;
        pred_marker_color_.g = 0.0;
        pred_marker_color_.b = 0.0;
        pred_marker_color_.a = 1.0;
      } else if (std::abs(actual_v) > kmh_to_m_per_sec(41.0) || std::abs(u[1]) > deg2rad(10.0)) {
        bug_acc_enabled = false;
        acc = mpc_cfg_.a_max;
        pred_marker_color_.r = 1.0;
        pred_marker_color_.g = 1.0;
        pred_marker_color_.b = 0.0;
        pred_marker_color_.a = 1.0;
      } else {
        bug_acc_enabled = true;
        acc = 500.0;
        pred_marker_color_.r = 0.0;
        pred_marker_color_.g = 156.0 / 255.0;
        pred_marker_color_.b = 209.0 / 255.0;
        pred_marker_color_.a = 1.0;
      }
    } else {
      acc = 100.0 * (u[0] - actual_v);
      acc = clip(acc, mpc_cfg_.a_min, mpc_cfg_.a_max);
    }

    if (!forced_stop_active) {
      acc = last_acc_ + (acc - last_acc_) * mpc_cfg_.accel_low_pass_gain;
    }
    u[1] = last_u_[1] + (u[1] - last_u_[1]) * mpc_cfg_.steer_low_pass_gain;
    acc = clip(acc, mpc_cfg_.a_min, mpc_cfg_.a_max);
    const double max_steering_step = mpc_cfg_.steer_rate_max / mpc_cfg_.control_rate;
    u[1] = clip(
      u[1], last_u_[1] - max_steering_step,
      last_u_[1] + max_steering_step);
    u[1] = clip(u[1], -mpc_cfg_.delta_max, mpc_cfg_.delta_max);
    const auto recovery_output = evaluate_stuck_recovery(
      pose, actual_v, u, acc, effective_v_max, mpc_fallback_active, steady_now, current_time);
    const bool recovery_command_active = recovery_output.has_value() &&
      apply_stuck_recovery_arbitration(
      recovery_output.value(), actual_v, current_time, u, acc, bug_acc_enabled);
    acc = clip(acc, mpc_cfg_.a_min, mpc_cfg_.a_max);
    u[1] = clip(u[1], -mpc_cfg_.delta_max, mpc_cfg_.delta_max);
    if (
      !u.allFinite() || !std::isfinite(acc) ||
      !std::isfinite(u[1] * mpc_cfg_.steering_tire_angle_gain_var))
    {
      publish_failsafe_command(current_time, "non-finite postprocessed control rejected");
      return;
    }
    const bool recovery_uses_normal_model =
      recovery_output.has_value() &&
      recovery_output->action.type == stuck_recovery::RecoveryActionType::LowSpeedRejoin;
    if (!recovery_command_active || recovery_uses_normal_model) {
      car_->drive(Eigen::Vector2d(actual_v, u[1]));
    }
    if (!publish_control_command(current_time, u, acc, bug_acc_enabled)) {
      return;
    }
    awsim_boost::TriggerContext boost_context;
    boost_context.control_enabled = enable_control_;
    boost_context.normal_command_published = true;
    boost_context.failsafe_active =
      forced_stop_active || odom_failsafe_active_ || command_failsafe_active_;
    boost_context.v2x_safety_brake_active =
      mpc_->last_v2x_behavior_output().state == V2XBehaviorState::SafetyBrake;
    boost_context.solver_fallback_active = mpc_fallback_active;
    boost_context.reverse_or_recovery_active =
      recovery_boost_suppressed_for_session_ || recovery_command_active ||
      (recovery_output.has_value() && recovery_output->action.inhibit_boost);
    boost_context.forward_speed_mps = actual_v;
    maybe_publish_awsim_boost(steady_now, boost_context);
    last_acc_ = acc;
    last_u_ = u;

    const auto interval = std::max(1, static_cast<int>(mpc_cfg_.control_rate / 4.0));
    if (!mpc_->current_prediction.first.empty() && loop_ % interval == 0) {
      publish_mpc_pred_marker(mpc_->current_prediction.first, mpc_->current_prediction.second);
    }
  }

  void publish_zero_command()
  {
    if (!rclcpp::ok() || !command_pub_) {
      return;
    }
    const Eigen::Vector2d zero(0.0, 0.0);
    const double shutdown_acceleration =
      recovery_may_be_in_reverse() && stuck_recovery_actuation_io_enabled_ ?
      reverse_actuation_calibration(cfg_.stuck_recovery).stop_acceleration_mps2 : 0.0;
    command_pub_->publish(create_ackermann_control_command(now(), zero, shutdown_acceleration));
  }

  std::string config_path_;
  std::optional<std::string> ref_vel_config_path_;
  Config cfg_;
  MpcConfig mpc_cfg_;
  bool use_sim_time_{};
  bool use_bug_acc_{};
  bool use_obstacle_avoidance_{};
  bool use_stats_{};
  bool awsim_boost_io_enabled_{false};
  bool awsim_state_tracking_enabled_{false};
  bool stuck_recovery_actuation_io_enabled_{false};
  bool race_started_{false};
  bool recovery_boost_suppressed_for_session_{false};
  bool recovery_reset_applied_{false};
  bool recovery_rejoin_hold_cycle_{false};
  bool recovery_fault_latched_{false};
  bool recovery_waiting_for_drive_after_reset_{false};
  std::size_t recovery_reset_drive_request_count_{0U};
  std::optional<SteadyClock::time_point> recovery_reset_stopped_since_;
  bool enable_control_{true};
  bool initialized_{false};
  bool ref_path_published_{false};
  bool odom_failsafe_active_{false};
  bool command_failsafe_active_{false};
  int current_laps_{1};
  double last_lap_time_{0.0};
  std::optional<int> last_condition_;
  std::optional<rclcpp::Time> last_colliding_time_;
  std::optional<SteadyClock::time_point> last_collision_receipt_steady_;
  rclcpp::Time last_t_;
  std::optional<rclcpp::Time> domain_start_epoch_;
  std::optional<overtake_core::StartWindowStatus> last_start_window_status_;
  bool domain_manual_reset_pending_{false};
  bool domain_manual_reset_ready_{false};
  std::string last_awsim_state_;
  int loop_{0};
  double last_acc_{0.0};
  Eigen::Vector2d last_u_{0.0, 0.0};
  ColorRGBA pred_marker_color_;

  std::unique_ptr<Map> map_;
  std::unique_ptr<ReferencePath> reference_path_;
  std::unique_ptr<BicycleModel> car_;
  std::unique_ptr<MPC> mpc_;
  std::unique_ptr<V2XGapPlanner> v2x_gap_planner_;
  std::unique_ptr<ReferenceVelocityConfigulator> ref_vel_configulator_;
  std::unique_ptr<awsim_boost::StartDashGuard> awsim_boost_guard_;
  std::unique_ptr<stuck_recovery::StuckRecoveryCore> stuck_recovery_core_;
  std::unique_ptr<recovery_footprint::OccupancyGrid> recovery_grid_;
  recovery_footprint::FootprintExtents recovery_footprint_;
  std::optional<Pose2D> recovery_observation_anchor_pose_;
  std::optional<double> recovery_observation_anchor_progress_m_;
  std::optional<Pose2D> recovery_maneuver_start_pose_;
  std::optional<Pose2D> recovery_last_reverse_pose_;
  double recovery_cumulative_reverse_distance_m_{0.0};
  double recovery_episode_traveled_distance_m_{0.0};
  bool recovery_reverse_pose_jump_{false};
  bool recovery_episode_had_contact_evidence_{false};
  std::size_t recovery_aggressive_retry_count_{0U};
  bool recovery_coordinated_stop_episode_{false};
  bool recovery_reverse_only_episode_{false};
  std::optional<recovery_footprint::ReversePrimitive> recovery_selected_reverse_primitive_;
  std::optional<double> recovery_selected_reverse_steering_angle_rad_;
  bool recovery_selected_stepwise_escape_{false};
  std::optional<std::vector<std::size_t>> recovery_initial_contact_cells_;
  std::optional<std::vector<std::size_t>> recovery_previous_contact_cells_;
  std::optional<stuck_recovery::CoreOutput> recovery_last_output_;
  std::optional<stuck_recovery::RecoveryState> last_recovery_state_;
  std::optional<stuck_recovery::StuckRejectReason> last_recovery_reject_reason_;
  std::optional<stuck_recovery::ExecutionMode> last_recovery_execution_mode_;
  std::optional<stuck_recovery::Gear> reported_gear_;
  std::optional<stuck_recovery::Gear> last_commanded_recovery_gear_;
  std::optional<bool> awsim_boost_active_;
  std::optional<SteadyClock::time_point> last_gear_report_receipt_steady_;
  std::optional<SteadyClock::time_point> last_awsim_status_receipt_steady_;

  rclcpp::Publisher<AckermannControlCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr command_raw_pub_;
  rclcpp::Publisher<AckermannControlBoostCommand>::SharedPtr boost_command_pub_;
  rclcpp::Publisher<Float32MultiArray>::SharedPtr awsim_boost_pub_;
  rclcpp::Publisher<GearCommand>::SharedPtr gear_command_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr mpc_pred_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr mpc_pred_pub_dummy_;
  rclcpp::Publisher<MarkerArray>::SharedPtr ref_path_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr ref_path_pub_dummy_;
  rclcpp::Publisher<MarkerArray>::SharedPtr ref_vel_marker_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr section_marker_pub_;

  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<Bool>::SharedPtr control_mode_request_sub_;
  rclcpp::Subscription<Trajectory>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<Empty>::SharedPtr stop_request_sub_;
  rclcpp::Subscription<Float32MultiArray>::SharedPtr awsim_status_sub_;
  rclcpp::Subscription<String>::SharedPtr awsim_state_sub_;
  rclcpp::Subscription<GearReport>::SharedPtr gear_report_sub_;
  rclcpp::Subscription<Int32>::SharedPtr condition_sub_;
  rclcpp::Subscription<PathConstraints>::SharedPtr path_constraints_sub_;
  rclcpp::Subscription<BorderCells>::SharedPtr border_cells_sub_;
  rclcpp::Subscription<V2XVehiclePositionArray>::SharedPtr v2x_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr ref_vel_marker_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  Odometry::SharedPtr odom_;
  Trajectory::SharedPtr trajectory_;
  std::optional<SteadyClock::time_point> last_odom_receipt_steady_;
  std::optional<rclcpp::Time> last_odom_source_stamp_;
  std::optional<SteadyClock::time_point> last_odom_source_advance_steady_;
  awsim_boost::BlockReason last_awsim_boost_block_reason_{awsim_boost::BlockReason::None};
};

std::optional<std::string> value_after_arg(const std::vector<std::string> & args, const std::string & name)
{
  for (std::size_t i = 0; i + 1 < args.size(); ++i) {
    if (args[i] == name) {
      return args[i + 1];
    }
  }
  return std::nullopt;
}

}  // namespace
}  // namespace multi_purpose_mpc_ros

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto args = rclcpp::remove_ros_arguments(argc, argv);
  const auto config_path = multi_purpose_mpc_ros::value_after_arg(args, "--config_path");
  const auto ref_vel_path = multi_purpose_mpc_ros::value_after_arg(args, "--ref_vel_path");
  if (!config_path.has_value()) {
    throw std::runtime_error("--config_path is required");
  }
  auto node = std::make_shared<multi_purpose_mpc_ros::MPCControllerCpp>(config_path.value(), ref_vel_path);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
