#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
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
#include <chrono>
#include <cmath>
#include <cstddef>
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
#include <utility>
#include <vector>

namespace multi_purpose_mpc_ros
{
namespace
{

using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_planning_msgs::msg::Trajectory;
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
using v2x_msgs::msg::V2XVehiclePositionArray;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

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
  return std::fmod(angle + kPi, 2.0 * kPi) - kPi;
}

double kmh_to_m_per_sec(const double kmh)
{
  return kmh / 3.6;
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

std::optional<Eigen::VectorXd> solve_osqp(
  Eigen::SparseMatrix<double> P, Eigen::SparseMatrix<double> A, const Eigen::VectorXd & q,
  const Eigen::VectorXd & l, const Eigen::VectorXd & u)
{
  P.makeCompressed();
  A.makeCompressed();

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

  csc * P_csc = csc_matrix(
    static_cast<c_int>(P.rows()), static_cast<c_int>(P.cols()), static_cast<c_int>(P.nonZeros()),
    p_x.data(), p_i.data(), p_p.data());
  csc * A_csc = csc_matrix(
    static_cast<c_int>(A.rows()), static_cast<c_int>(A.cols()), static_cast<c_int>(A.nonZeros()),
    a_x.data(), a_i.data(), a_p.data());

  OSQPData data;
  data.n = static_cast<c_int>(P.cols());
  data.m = static_cast<c_int>(A.rows());
  data.P = P_csc;
  data.A = A_csc;
  data.q = q_data.data();
  data.l = l_data.data();
  data.u = u_data.data();

  OSQPSettings settings;
  osqp_set_default_settings(&settings);
  settings.verbose = false;

  OSQPWorkspace * work = nullptr;
  if (osqp_setup(&work, &data, &settings) != 0 || work == nullptr) {
    c_free(P_csc);
    c_free(A_csc);
    return std::nullopt;
  }
  osqp_solve(work);
  if (work->solution == nullptr || work->solution->x == nullptr) {
    osqp_cleanup(work);
    c_free(P_csc);
    c_free(A_csc);
    return std::nullopt;
  }
  Eigen::VectorXd solution(P.cols());
  for (int i = 0; i < solution.size(); ++i) {
    solution[i] = static_cast<double>(work->solution->x[i]);
  }
  osqp_cleanup(work);
  c_free(P_csc);
  c_free(A_csc);
  return solution;
}

std::pair<std::vector<double>, std::vector<double>> load_waypoints(const std::string & csv_path)
{
  const auto columns = read_csv_columns(csv_path);
  return {columns.at("wp_x"), columns.at("wp_y")};
}

std::pair<std::vector<double>, std::vector<double>> load_ref_path(const std::string & csv_path)
{
  const auto columns = read_csv_columns(csv_path);
  return {columns.at("x_m"), columns.at("y_m")};
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
  cv::Mat data;
  cv::Mat data_backup;
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

    std::vector<int> n_wp;
    for (std::size_t i = 0; i + 1 < wp_x.size(); ++i) {
      const double d = std::hypot(wp_x[i + 1] - wp_x[i], wp_y[i + 1] - wp_y[i]);
      n_wp.push_back(std::max(1, static_cast<int>(d / resolution)));
    }

    const double gp_x = wp_x.back();
    const double gp_y = wp_y.back();
    std::vector<double> interp_x;
    std::vector<double> interp_y;
    for (std::size_t i = 0; i + 1 < wp_x.size(); ++i) {
      for (int j = 0; j < n_wp[i]; ++j) {
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

    const auto solution_opt = solve_osqp(P, D, q, l, u);
    if (!solution_opt.has_value() || solution_opt->size() != N) {
      return false;
    }
    const Eigen::VectorXd solution = solution_opt.value();
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
    const double safety_margin_scale_in, const double Ts_in)
  : length(length_in),
    width(width_in),
    safety_margin_scale(std::max(0.0, safety_margin_scale_in)),
    safety_margin(compute_safety_margin()),
    reference_path(ref_path),
    Ts(Ts_in)
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
    if (v_ref == 0.0) {
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
  bool debug_log_enabled{false};
};

struct V2XBehaviorConfig
{
  bool enabled{false};
  bool debug_log_enabled{false};
  double debug_log_period_sec{1.0};
  double follow_distance{8.0};
  double safety_brake_distance{3.0};
  double safety_brake_margin{2.0};
  bool follow_gap_planner_enabled{false};
  bool follow_gap_planner_no_gap_speed_limit_enabled{false};
  bool follow_gap_planner_respect_overtake_forbidden{true};
  bool follow_speed_limit_enabled{false};
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
  bool overtake_close_follow_enabled{false};
  double overtake_close_follow_min_front_distance{1.5};
  double overtake_close_follow_max_closing_speed{0.8};
  double overtake_close_follow_min_side_clearance{2.0};
  bool overtake_before_curve_enabled{false};
  double overtake_before_curve_max_front_speed{8.0};
  double overtake_before_curve_min_speed_advantage{1.0};
  double overtake_start_curve_clearance_distance{0.0};
  bool overtake_continue_in_forbidden_enabled{false};
  bool overtake_front_velocity_limit_enabled{true};
  bool overtake_fallback_ignore_soft_curve_forbidden{false};
  double overtake_fallback_min_side_clearance{1.0};
  bool overtake_curve_cooldown_enabled{false};
  double overtake_curve_cooldown_sec{0.0};
  bool side_overtake_enabled{false};
  bool side_overtake_ignore_soft_curve_forbidden{true};
  double overtake_gap_lookahead_distance{0.0};
  double moving_front_speed_threshold{1.0};
  double moving_follow_speed_margin{2.0};
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
  double low_speed_avoidance_max_front_speed{1.0};
  double low_speed_avoidance_min_prepare_distance{0.0};
  double low_speed_avoidance_min_gap_width{1.5};
  int low_speed_avoidance_min_gap_points{2};
  double low_speed_avoidance_clear_distance{8.0};
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
  int pass_side_sign{0};
  std::vector<double> lb;
  std::vector<double> ub;
  std::vector<double> target_ey;
  std::vector<bool> target_active;
  double target_velocity_limit{std::numeric_limits<double>::infinity()};
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
  bool has_side_vehicle{false};
  bool has_danger_vehicle{false};
  bool start_grid_grace_active{false};
  bool low_speed_avoidance_candidate{false};
  bool low_speed_avoidance_gap_blocked{false};
  bool overtake_forbidden{false};
  bool overtake_forbidden_wp{false};
  bool front_decel_curve_guard{false};
  bool overtake_zone_allows{false};
  bool overtake_start_curve_blocked{false};
  bool before_curve_overtake_allowed{false};
  bool continuing_overtake_allowed{false};
  bool overtake_gap_available{false};
  bool overtake_fallback_target{false};
  bool overtake_cooldown_active{false};
  int overtake_pass_side_sign{0};
  int overtake_plan_N{0};
  double overtake_side_clearance{0.0};
  double front_speed{std::numeric_limits<double>::infinity()};
  double front_lateral{std::numeric_limits<double>::infinity()};
  double ego_speed{0.0};
  FrontRiskMetrics front_risk;
  FrontRiskLevel front_risk_level{FrontRiskLevel::Clear};
  double target_velocity_limit{std::numeric_limits<double>::infinity()};
  double front_distance{std::numeric_limits<double>::infinity()};
  std::string reason;
  std::string overtake_block_reason;
};

struct OvertakeLineState
{
  OvertakeLinePhase phase{OvertakeLinePhase::Idle};
  int pass_side_sign{0};
  double target_ey{0.0};
  double phase_start_sec{-std::numeric_limits<double>::infinity()};
  double phase_start_ey{0.0};
};

struct OvertakeLineOutput
{
  bool active{false};
  std::vector<double> target_ey;
  std::vector<bool> target_active;
  double target_velocity_limit{std::numeric_limits<double>::infinity()};
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

  explicit V2XGapPlanner(const V2XGapPlannerConfig & cfg_in) : cfg(cfg_in) {}

  void update(const V2XVehiclePositionArray & msg, const double receipt_sec)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const double array_stamp = stamp_to_seconds(msg.header.stamp);
    for (const auto & vehicle : msg.vehicles) {
      const std::string id = vehicle.vehicle_id.empty() ? "__unknown__" : vehicle.vehicle_id;
      double sample_stamp = stamp_to_seconds(vehicle.header.stamp);
      if (sample_stamp <= 0.0) {
        sample_stamp = array_stamp > 0.0 ? array_stamp : receipt_sec;
      }

      auto & tracked = vehicles_[id];
      double vx = 0.0;
      double vy = 0.0;
      if (tracked.has_sample) {
        const double dt = sample_stamp - tracked.stamp_sec;
        const double dx = vehicle.position.x - tracked.x;
        const double dy = vehicle.position.y - tracked.y;
        const double jump = std::hypot(dx, dy);
        if (dt > kEps && jump <= cfg.position_jump_threshold) {
          vx = dx / dt;
          vy = dy / dt;
          if (std::hypot(vx, vy) > cfg.v_max_safety) {
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
      if (now_sec - tracked.receipt_sec > cfg.timeout_sec) {
        continue;
      }
      vehicles.push_back(tracked);
    }
    return vehicles;
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

    const double required_width =
      std::max(0.0, std::max(cfg.min_gap_width, behavior_cfg.low_speed_avoidance_min_gap_width));
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
        if (interval.width() < required_width) {
          candidate.feasible = false;
          return candidate;
        }
        candidate.intersection.lower = std::max(candidate.intersection.lower, interval.lower);
        candidate.intersection.upper = std::min(candidate.intersection.upper, interval.upper);
        candidate.min_width = std::min(candidate.min_width, interval.width());
        if (candidate.intersection.width() < required_width) {
          candidate.feasible = false;
          return candidate;
        }
      }
      return candidate;
    };

    int pass_side_sign = configured_pass_side != 0 ? configured_pass_side : locked_pass_side;
    SideCandidate selected_side;
    if (pass_side_sign != 0) {
      selected_side = evaluate_side(pass_side_sign);
    } else {
      const auto right = evaluate_side(-1);
      const auto left = evaluate_side(1);
      if (right.feasible && left.feasible) {
        pass_side_sign = right.min_width >= left.min_width ? -1 : 1;
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
    for (int i = 0; i < N; ++i) {
      const double point_s = forward_path_distance(*model.reference_path, ref_wp_id, ref_wp_id + i + 1);
      const LateralInterval base_interval{base_lb[i], base_ub[i]};
      LateralInterval local_corridor = selected_side.intersection;
      if (point_s < approach_distance) {
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
      std::max(0.0, behavior_cfg.low_speed_avoidance_velocity);

    if (update_last_target) {
      if (
        last_logged_local_path_side_ != pass_side_sign ||
        std::abs(last_logged_local_path_target_ey_ - pass_target_ey) > 0.1) {
        RCLCPP_INFO(
          rclcpp::get_logger("mpc_controller"),
          "V2X local path selected: side=%s, target_ey=%.2f, center_ey=%.2f, "
          "vehicle_side_ey=%.2f, width=%.2f, approach_s=%.2f, pass_s=[%.2f, %.2f], "
          "vehicles=%zu",
          pass_side_sign < 0 ? "right" : "left", pass_target_ey,
          center_target_ey, vehicle_side_target_ey, interval_width, approach_distance, pass_begin_s,
          pass_end_s, projected.size());
        last_logged_local_path_side_ = pass_side_sign;
        last_logged_local_path_target_ey_ = pass_target_ey;
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
    const int forced_pass_side_sign = 0)
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

    for (int i = 0; i < N; ++i) {
      const LateralInterval base{base_lb[i], base_ub[i]};
      output.lb[i] = base.lower;
      output.ub[i] = base.upper;
      output.target_ey[i] = base.center();
      if (base.width() <= kEps) {
        continue;
      }

      const auto & waypoint = model.reference_path->get_waypoint(ref_wp_id + i);
      const double horizon_t = std::min(static_cast<double>(i + 1) * model.Ts, cfg.prediction_time);
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
      const auto free_intervals = compute_free_intervals(base, occupied, allow_vehicle_vehicle_gap);
      if (free_intervals.empty()) {
        feasible = false;
        break;
      }
      const auto pass_side_intervals = filter_by_pass_side(free_intervals, base, low_speed_pass_side);
      if (low_speed_pass_side != 0 && pass_side_intervals.empty()) {
        feasible = false;
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
    const bool allow_vehicle_vehicle_gap) const
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
    const double min_width = std::max(0.0, cfg.min_gap_width);
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
  std::optional<double> last_target_ey_;
  std::optional<double> low_speed_locked_target_ey_;
  std::optional<int> low_speed_locked_side_sign_;
  int last_logged_local_path_side_{0};
  double last_logged_local_path_target_ey_{std::numeric_limits<double>::infinity()};
};

struct MpcConfig
{
  int N{};
  Eigen::Vector3d Q;
  Eigen::Vector2d R;
  Eigen::Vector3d QN;
  double v_max{};
  double v_max_kmh{};
  double a_min{};
  double a_max{};
  double ay_max{};
  double delta_max{};
  double steer_rate_max{};
  double control_rate{};
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
};

struct MPC
{
  MPC(
    BicycleModel * model_in, const MpcConfig & cfg_in, const bool use_obstacle_avoidance_in,
    const bool use_path_constraints_topic_in)
  : model(model_in),
    cfg(cfg_in),
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

  V2XBehaviorOutput evaluate_v2x_behavior(
    const int ref_wp_id, const int N, const Eigen::VectorXd & lb, const Eigen::VectorXd & ub,
    const double now_sec)
  {
    V2XBehaviorOutput output;
    if (!cfg.v2x_behavior.enabled || gap_planner == nullptr || N <= 0) {
      return output;
    }

    if (!std::isfinite(first_v2x_behavior_eval_sec)) {
      first_v2x_behavior_eval_sec = now_sec;
    }
    const bool start_grid_grace_active =
      cfg.v2x_behavior.start_grid_grace_time > 0.0 &&
      now_sec - first_v2x_behavior_eval_sec < cfg.v2x_behavior.start_grid_grace_time;
    output.start_grid_grace_active = start_grid_grace_active;
    output.ego_speed = current_speed_mps_;

    const auto vehicles = gap_planner->active_vehicles(now_sec);
    output.active_vehicle_count = vehicles.size();
    if (vehicles.empty()) {
      output.reason = "no active vehicles";
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
    const double front_lateral_range = front_decel_curve_guard ?
      std::min(
        corridor_lateral_range,
        danger_lateral_range +
        std::max(0.0, cfg.v2x_behavior.front_decel_guard_curve_lateral_margin)) :
      danger_lateral_range;
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

    bool has_front_vehicle = false;
    bool has_danger_vehicle = false;
    bool has_side_vehicle = false;
    bool has_low_speed_clearance_vehicle = false;
    double nearest_front_distance = std::numeric_limits<double>::infinity();
    double nearest_front_speed = std::numeric_limits<double>::infinity();
    double nearest_front_lateral = std::numeric_limits<double>::infinity();
    const bool continuing_low_speed_avoidance =
      v2x_behavior_state_initialized && v2x_behavior_state == V2XBehaviorState::LowSpeedAvoidance;

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
      if (std::abs(lateral) > corridor_lateral_range) {
        continue;
      }

      if (
        continuing_low_speed_avoidance &&
        self_distance <= cfg.v2x_behavior.low_speed_avoidance_clear_distance &&
        longitudinal > -side_longitudinal_range) {
        has_low_speed_clearance_vehicle = true;
      }
      const bool front_overlap = std::abs(lateral) <= front_lateral_range;
      if (front_overlap && longitudinal > 0.0 && longitudinal < front_detection_distance) {
        const double vehicle_speed = std::hypot(vehicle.vx, vehicle.vy);
        has_front_vehicle = true;
        if (longitudinal < nearest_front_distance) {
          nearest_front_distance = longitudinal;
          nearest_front_speed = vehicle_speed;
          nearest_front_lateral = lateral;
        }
        const bool moving_front =
          vehicle_speed > cfg.v2x_behavior.moving_front_speed_threshold;
        const double closing_speed =
          moving_front ? std::max(0.0, current_speed_mps_ - vehicle_speed) : current_speed_mps_;
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
        if (longitudinal < front_safety_brake_distance) {
          has_danger_vehicle = true;
        }
      }
      if (std::abs(longitudinal) < side_longitudinal_range) {
        has_side_vehicle = true;
      }
    }

    output.front_distance = nearest_front_distance;
    output.front_speed = nearest_front_speed;
    output.front_lateral = nearest_front_lateral;
    output.has_front_vehicle = has_front_vehicle;
    output.has_side_vehicle = has_side_vehicle;
    output.has_danger_vehicle = has_danger_vehicle;
    const bool suppress_start_grid_stop_behavior =
      start_grid_grace_active && has_side_vehicle;
    output.follow_gap_planner_allowed = !overtake_forbidden && !overtake_cooldown_active;
    const FrontRiskMetrics front_risk =
      compute_front_risk(nearest_front_distance, nearest_front_speed);
    const FrontRiskLevel front_risk_level = classify_front_risk(front_risk);
    output.front_risk = front_risk;
    output.front_risk_level = front_risk_level;
    const bool low_speed_avoidance_candidate =
      cfg.v2x_behavior.low_speed_avoidance_enabled && has_front_vehicle &&
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

    if (
      has_front_vehicle && !suppress_start_grid_stop_behavior &&
      front_risk_level == FrontRiskLevel::EmergencyBrake) {
      output.state = V2XBehaviorState::SafetyBrake;
      output.reason = front_risk_reason("front risk emergency", front_risk, front_risk_level);
      return commit_v2x_behavior_state(output, now_sec);
    }

    if (has_danger_vehicle && !suppress_start_grid_stop_behavior) {
      output.state = V2XBehaviorState::SafetyBrake;
      output.reason = "inside stopping distance";
      return commit_v2x_behavior_state(output, now_sec);
    }

    if (
      has_low_speed_clearance_vehicle && cfg.v2x_behavior.low_speed_avoidance_enabled &&
      continuing_low_speed_avoidance) {
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
          " / front decel guard";
      }
      return commit_v2x_behavior_state(output, now_sec);
    }

    const bool continuing_overtake =
      v2x_behavior_state_initialized && v2x_behavior_state == V2XBehaviorState::Overtake;
    const bool soft_overtake_forbidden = overtake_forbidden && !overtake_forbidden_wp;
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
      cfg.v2x_behavior.overtake_continue_in_forbidden_enabled &&
      !overtake_cooldown_active &&
      continuing_overtake &&
      soft_overtake_forbidden &&
      !continuing_inner_curve_pass &&
      front_risk_level != FrontRiskLevel::EmergencyBrake;
    output.overtake_pass_side_sign =
      choose_overtake_pass_side(nearest_front_lateral, lb[0], ub[0]);
    output.overtake_side_clearance =
      overtake_side_clearance(output.overtake_pass_side_sign, nearest_front_lateral, lb[0], ub[0]);
    if (overtake_locked_side_sign_ != 0) {
      output.overtake_pass_side_sign = overtake_locked_side_sign_;
      output.overtake_side_clearance =
        overtake_side_clearance(output.overtake_pass_side_sign, nearest_front_lateral, lb[0], ub[0]);
    }
    const double fallback_min_side_clearance = std::max(
      cfg.v2x_behavior.overtake_min_gap_width,
      cfg.v2x_behavior.overtake_fallback_min_side_clearance);
    const bool fallback_side_clear =
      output.overtake_pass_side_sign != 0 &&
      output.overtake_side_clearance >= fallback_min_side_clearance;
    const bool fallback_inner_curve_pass =
      is_inner_curve_pass(output.overtake_pass_side_sign, inner_curve_pass_side);
    const bool fallback_soft_curve_allowed =
      cfg.v2x_behavior.overtake_fallback_ignore_soft_curve_forbidden &&
      !overtake_cooldown_active &&
      soft_overtake_forbidden &&
      !overtake_forbidden_wp &&
      fallback_side_clear &&
      !fallback_inner_curve_pass &&
      front_risk_level != FrontRiskLevel::EmergencyBrake;
    const bool overtake_zone_allows =
      !overtake_cooldown_active &&
      !overtake_start_curve_blocked &&
      (!overtake_forbidden || before_curve_overtake_allowed || continuing_overtake_allowed ||
      fallback_soft_curve_allowed);
    output.overtake_start_curve_blocked = overtake_start_curve_blocked;
    output.before_curve_overtake_allowed = before_curve_overtake_allowed;
    output.continuing_overtake_allowed = continuing_overtake_allowed;
    output.overtake_zone_allows = overtake_zone_allows;

    bool overtake_gap_available =
      overtake_zone_allows &&
      !cfg.v2x_behavior.require_gap_for_overtake && !cfg.v2x_behavior.overtake_guard_enabled;
    std::string overtake_block_reason = overtake_forbidden_wp ?
      "overtake forbidden wp" :
      overtake_cooldown_active ?
      "overtake curve cooldown" :
      overtake_start_curve_blocked ?
      "overtake start too close to curve" :
      soft_overtake_forbidden ?
      "overtake forbidden curve" :
      "no overtake gap";
    if (
      has_front_vehicle && overtake_zone_allows &&
      (cfg.v2x_behavior.require_gap_for_overtake || cfg.v2x_behavior.overtake_guard_enabled)) {
      const int overtake_plan_N = v2x_overtake_gap_plan_horizon(N);
      output.overtake_plan_N = overtake_plan_N;
      const auto [overtake_lb, overtake_ub] =
        build_v2x_gap_planner_bounds(ref_wp_id, N, lb, ub, overtake_plan_N);
      const auto candidate_gap =
        gap_planner->plan(
        *model, ref_wp_id, overtake_plan_N, overtake_lb, overtake_ub, now_sec, false, false,
        std::numeric_limits<double>::infinity(), output.overtake_pass_side_sign);
      const int planned_pass_side_sign =
        infer_gap_pass_side(candidate_gap, model->spatial_state.e_y);
      if (planned_pass_side_sign != 0) {
        output.overtake_pass_side_sign = planned_pass_side_sign;
        output.overtake_side_clearance =
          overtake_side_clearance(
            output.overtake_pass_side_sign, nearest_front_lateral, lb[0], ub[0]);
      }
      const bool inner_curve_pass =
        is_inner_curve_pass(output.overtake_pass_side_sign, inner_curve_pass_side);
      const bool soft_curve_pass_side_unknown =
        soft_overtake_forbidden && output.overtake_pass_side_sign == 0;
      if (soft_curve_pass_side_unknown) {
        overtake_gap_available = false;
        overtake_block_reason = "overtake pass side unknown in curve";
      } else if (inner_curve_pass) {
        overtake_gap_available = false;
        overtake_block_reason = "overtake inner curve blocked";
      } else if (cfg.v2x_behavior.overtake_guard_enabled) {
        overtake_gap_available = overtake_guard_allows(
          candidate_gap, ref_wp_id, nearest_front_distance, model->spatial_state.e_y,
          overtake_block_reason);
      } else {
        overtake_gap_available = has_sufficient_overtake_gap(candidate_gap);
      }
      if (
        !overtake_gap_available &&
        !inner_curve_pass &&
        output.overtake_pass_side_sign != 0 &&
        output.overtake_side_clearance >= fallback_min_side_clearance &&
        front_risk_level != FrontRiskLevel::EmergencyBrake) {
        std::string fallback_guard_reason;
        if (overtake_fallback_guard_allows(
            output.overtake_pass_side_sign, nearest_front_distance, model->spatial_state.e_y,
            lb[0], ub[0], fallback_guard_reason)) {
          overtake_gap_available = true;
          output.overtake_fallback_target = true;
          std::ostringstream ss;
          ss << "overtake fallback side target"
             << ", side=" << (output.overtake_pass_side_sign > 0 ? "left" : "right")
             << ", clearance=" << output.overtake_side_clearance
             << ", guard=" << fallback_guard_reason;
          overtake_block_reason = ss.str();
        } else {
          std::string close_follow_reason;
          if (overtake_close_follow_allows(
              output, nearest_front_distance, nearest_front_speed, model->spatial_state.e_y,
              lb[0], ub[0], close_follow_reason)) {
            overtake_gap_available = true;
            output.overtake_fallback_target = true;
            overtake_block_reason = close_follow_reason;
          } else {
            overtake_block_reason = fallback_guard_reason + " / " + close_follow_reason;
          }
        }
      }
    }
    output.overtake_gap_available = overtake_gap_available;
    output.overtake_block_reason = overtake_block_reason;

    if (has_front_vehicle) {
      if (overtake_zone_allows && overtake_gap_available) {
        output.state = V2XBehaviorState::Overtake;
        output.reason = before_curve_overtake_allowed ?
          "slow front before curve and reachable gap / " + overtake_block_reason :
          continuing_overtake_allowed ?
          "continue overtake in soft forbidden zone / " + overtake_block_reason :
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
              " / front decel guard";
          }
        }
      } else {
        output.state = V2XBehaviorState::Follow;
        output.reason = overtake_zone_allows ? overtake_block_reason :
          before_curve_overtake_allowed ? "before-curve overtake blocked" :
          overtake_block_reason;
        bool front_risk_applied = false;
        if (apply_follow_velocity_limits(
            output, nearest_front_distance, nearest_front_speed, front_decel_curve_guard, front_risk,
            front_risk_level, front_risk_applied)) {
          output.reason += front_risk_applied ?
            " / " + front_risk_reason("front risk brake", front_risk, front_risk_level) :
            " / front decel guard";
        }
      }
      return commit_v2x_behavior_state(output, now_sec);
    }

    if (has_side_vehicle && cfg.v2x_behavior.side_overtake_enabled) {
      const bool side_overtake_zone_allows =
        !overtake_cooldown_active &&
        !overtake_start_curve_blocked &&
        (!overtake_forbidden ||
        (cfg.v2x_behavior.side_overtake_ignore_soft_curve_forbidden && soft_overtake_forbidden) ||
        continuing_overtake_allowed);
      output.overtake_zone_allows = side_overtake_zone_allows;

      bool side_overtake_gap_available =
        side_overtake_zone_allows &&
        !cfg.v2x_behavior.require_gap_for_overtake && !cfg.v2x_behavior.overtake_guard_enabled;
      std::string side_overtake_block_reason = overtake_forbidden_wp ?
        "overtake forbidden wp" :
        overtake_cooldown_active ?
        "overtake curve cooldown" :
        overtake_start_curve_blocked ?
        "overtake start too close to curve" :
        side_overtake_zone_allows ?
        "no side overtake gap" :
        "overtake forbidden curve";

      if (
        side_overtake_zone_allows &&
        (cfg.v2x_behavior.require_gap_for_overtake || cfg.v2x_behavior.overtake_guard_enabled)) {
        const int overtake_plan_N = v2x_overtake_gap_plan_horizon(N);
        output.overtake_plan_N = overtake_plan_N;
        const auto [overtake_lb, overtake_ub] =
          build_v2x_gap_planner_bounds(ref_wp_id, N, lb, ub, overtake_plan_N);
        const auto candidate_gap =
          gap_planner->plan(
          *model, ref_wp_id, overtake_plan_N, overtake_lb, overtake_ub, now_sec, false, false,
          std::numeric_limits<double>::infinity(), output.overtake_pass_side_sign);
        const int planned_pass_side_sign =
          infer_gap_pass_side(candidate_gap, model->spatial_state.e_y);
        if (planned_pass_side_sign != 0) {
          output.overtake_pass_side_sign = planned_pass_side_sign;
        }
        const bool inner_curve_pass =
          is_inner_curve_pass(output.overtake_pass_side_sign, inner_curve_pass_side);
        const bool soft_curve_pass_side_unknown =
          soft_overtake_forbidden && output.overtake_pass_side_sign == 0;
        if (soft_curve_pass_side_unknown) {
          side_overtake_gap_available = false;
          side_overtake_block_reason = "overtake pass side unknown in curve";
        } else if (inner_curve_pass) {
          side_overtake_gap_available = false;
          side_overtake_block_reason = "overtake inner curve blocked";
        } else if (cfg.v2x_behavior.overtake_guard_enabled) {
          side_overtake_gap_available = overtake_guard_allows(
            candidate_gap, ref_wp_id, std::numeric_limits<double>::infinity(),
            model->spatial_state.e_y, side_overtake_block_reason);
        } else {
          side_overtake_gap_available = has_sufficient_overtake_gap(candidate_gap);
        }
      }

      output.overtake_gap_available = side_overtake_gap_available;
      output.overtake_block_reason = side_overtake_block_reason;
      if (side_overtake_zone_allows && side_overtake_gap_available) {
        output.state = V2XBehaviorState::Overtake;
        output.reason = "side vehicle and reachable gap / " + side_overtake_block_reason;
        return commit_v2x_behavior_state(output, now_sec);
      }
      output.reason = side_overtake_zone_allows ?
        "side vehicle overtake blocked / " + side_overtake_block_reason :
        side_overtake_block_reason;
    }

    if (
      has_side_vehicle && cfg.v2x_behavior.low_speed_avoidance_enabled &&
      v2x_behavior_state_initialized && v2x_behavior_state == V2XBehaviorState::LowSpeedAvoidance) {
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

  MpcProblem init_problem(const int N, const double safety_margin, const double now_sec)
  {
    constexpr int nx = 3;
    constexpr int nu = 2;
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
    kappa_pred[N - 1] = std::tan(current_control[current_control.size() - 1]) / model->length;

    model->wp_id += effective_wp_id_offset();

    Eigen::MatrixXd A_dense = Eigen::MatrixXd::Zero(nx_N, nx_N);
    Eigen::MatrixXd B_dense = Eigen::MatrixXd::Zero(nx_N, nu_N);
    for (int n = 0; n < N; ++n) {
      const auto & current_waypoint = model->reference_path->get_waypoint(model->wp_id + n);
      const auto & next_waypoint = model->reference_path->get_waypoint(model->wp_id + n + 1);
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
    }

    int ref_wp_id = 0;
    if (
      model->reference_path->path_constraints_upper.empty() ||
      model->reference_path->path_constraints_lower.empty()) {
      model->reference_path->update_simple_path_constraints(cfg.N, model->safety_margin);
    }
    if (!model->reference_path->path_constraints_upper.empty()) {
      ref_wp_id = (model->wp_id + 1) % static_cast<int>(model->reference_path->path_constraints_upper.size());
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
    const bool use_gap_planner =
      gap_planner != nullptr &&
      (!cfg.v2x_behavior.enabled || behavior_output.allow_gap_planner);
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
      behavior_output.state == V2XBehaviorState::Overtake && overtake_pass_side_sign != 0;
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
        behavior_output.state == V2XBehaviorState::Overtake ? overtake_pass_side_sign : 0)) :
      GapPlannerOutput{};
    if (std::isfinite(behavior_output.target_velocity_limit)) {
      apply_velocity_limit(umax_dyn, ur, N, behavior_output.target_velocity_limit);
    }
    const bool allow_no_gap_velocity_limit =
      behavior_output.state != V2XBehaviorState::Follow ||
      cfg.v2x_behavior.follow_gap_planner_no_gap_speed_limit_enabled;
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
      } else if (!planner_output.feasible && allow_no_gap_velocity_limit) {
        const double no_gap_velocity = std::max(0.0, planner_output.target_velocity_limit);
        apply_velocity_limit(umax_dyn, ur, N, no_gap_velocity);
      }
    }
    if (
      std::isfinite(planner_output.target_velocity_limit) &&
      (planner_output.feasible || allow_no_gap_velocity_limit)) {
      apply_velocity_limit(umax_dyn, ur, N, planner_output.target_velocity_limit);
    }

    const auto overtake_line_output =
      update_overtake_line(behavior_output, ref_wp_id, N, lb, ub, now_sec);
    if (std::isfinite(overtake_line_output.target_velocity_limit)) {
      apply_velocity_limit(umax_dyn, ur, N, overtake_line_output.target_velocity_limit);
    }
    const bool use_overtake_line_target = overtake_line_output.active;

    if (use_overtake_line_target) {
      overtake_locked_target_ey_.reset();
    } else if (!use_overtake_side_target) {
      overtake_locked_target_ey_.reset();
      overtake_locked_side_sign_ = 0;
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
      xr[nx + i * nx] = cfg.center_bias * center_ey;
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
    }

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

    return MpcProblem{q, l, u, P, A_full, N};
  }

  std::optional<Eigen::VectorXd> solve_problem(const MpcProblem & problem)
  {
    const auto solution_opt = solve_osqp(problem.P, problem.A, problem.q, problem.l, problem.u);
    if (!solution_opt.has_value() || solution_opt->size() != problem.P.rows()) {
      return std::nullopt;
    }
    return solution_opt.value();
  }

  std::pair<Eigen::Vector2d, double> get_control(const double now_sec)
  {
    constexpr int nx = 3;
    constexpr int nu = 2;
    model->get_current_waypoint();
    const int N = model->reference_path->circular ?
      cfg.N :
      std::min(cfg.N, model->reference_path->n_waypoints - model->wp_id);

    model->spatial_state = model->t2s(*model->current_waypoint, model->temporal_state);
    MpcProblem problem = init_problem(N, model->safety_margin, now_sec);

    try {
      auto solution_opt = solve_problem(problem);
      if (!solution_opt.has_value()) {
        throw std::runtime_error("OSQP failed");
      }
      Eigen::VectorXd dec = solution_opt.value();
      Eigen::VectorXd control_signals = dec.tail(N * nu);
      bool all_steers_non_zero = true;
      for (int i = 1; i < control_signals.size(); i += 2) {
        if (control_signals[i] == 0.0) {
          all_steers_non_zero = false;
          break;
        }
      }
      if (!all_steers_non_zero) {
        for (int i = 1; i < 6; ++i) {
          const double relaxed_safety_margin = model->safety_margin * ((5.0 - i) / 5.0);
          problem = init_problem(N, relaxed_safety_margin, now_sec);
          solution_opt = solve_problem(problem);
          if (!solution_opt.has_value()) {
            continue;
          }
          dec = solution_opt.value();
          control_signals = dec.tail(N * nu);
          all_steers_non_zero = true;
          for (int j = 1; j < control_signals.size(); j += 2) {
            if (control_signals[j] == 0.0) {
              all_steers_non_zero = false;
              break;
            }
          }
          if (infeasibility_counter == 0 && all_steers_non_zero) {
            break;
          }
        }
      }

      for (int i = 1; i < control_signals.size(); i += 2) {
        control_signals[i] = std::atan(control_signals[i] * model->length);
      }
      const double v = control_signals[0];
      double delta = control_signals[1];
      const double max_delta_change = cfg.steer_rate_max * model->Ts;
      delta = clip(delta, previous_steering - max_delta_change, previous_steering + max_delta_change);
      previous_steering = delta;
      control_signals[1] = delta;

      current_control = control_signals;
      current_prediction = update_prediction(dec.head((N + 1) * nx), N);
      Eigen::Vector2d u(v, delta);

      double max_delta = 0.0;
      const int end = static_cast<int>(control_signals.size() / 3) * 2;
      for (int i = 1; i < end; i += 2) {
        max_delta = std::max(max_delta, std::abs(control_signals[i]));
      }
      infeasibility_counter = 0;
      last_solved_wp_id = model->wp_id;
      return {u, max_delta};
    } catch (const std::exception &) {
      Eigen::Vector2d u(0.0, 0.0);
      double max_delta = 0.0;
      const int id = nu * (infeasibility_counter + 1);
      if (id + 2 < current_control.size()) {
        u = current_control.segment<2>(id);
        max_delta = std::abs(u[1]);
      }
      ++infeasibility_counter;
      return {u, max_delta};
    }
  }

  std::pair<std::vector<double>, std::vector<double>> update_prediction(
    const Eigen::VectorXd & spatial_state_prediction_flat, const int N)
  {
    std::pair<std::vector<double>, std::vector<double>> out;
    for (int n = 2; n < N; ++n) {
      const auto & associated_waypoint = model->reference_path->get_waypoint(model->wp_id + n);
      Eigen::Vector3d pred_state = spatial_state_prediction_flat.segment<3>(n * 3);
      const auto temporal = model->s2t(associated_waypoint, pred_state);
      out.first.push_back(temporal.x);
      out.second.push_back(temporal.y);
    }
    return out;
  }

  BicycleModel * model{};
  MpcConfig cfg;
  V2XGapPlanner * gap_planner{};
  bool use_obstacle_avoidance{};
  bool use_path_constraints_topic{};
  V2XBehaviorState v2x_behavior_state{V2XBehaviorState::Cruise};
  bool v2x_behavior_state_initialized{false};
  double last_v2x_behavior_state_change_sec{-std::numeric_limits<double>::infinity()};
  double last_v2x_behavior_debug_log_sec_{std::numeric_limits<double>::quiet_NaN()};
  double first_v2x_behavior_eval_sec{std::numeric_limits<double>::infinity()};
  std::optional<double> overtake_locked_target_ey_;
  int overtake_locked_side_sign_{0};
  OvertakeLineState overtake_line_state_;
  double last_overtake_line_debug_log_sec_{std::numeric_limits<double>::quiet_NaN()};
  double overtake_curve_cooldown_until_sec_{-std::numeric_limits<double>::infinity()};
  double previous_steering{0.0};
  double current_speed_mps_{0.0};

  int effective_wp_id_offset() const
  {
    if (cfg.wp_id_low_speed > kEps && current_speed_mps_ <= cfg.wp_id_low_speed) {
      return cfg.wp_id_low_offset;
    }
    return cfg.wp_id_offset;
  }
  std::pair<std::vector<double>, std::vector<double>> current_prediction;
  int infeasibility_counter{0};
  int last_solved_wp_id{0};
  Eigen::VectorXd current_control;

private:
  static void apply_velocity_limit(
    Eigen::VectorXd & umax_dyn, Eigen::VectorXd & ur, const int N, const double velocity_limit)
  {
    const double limit = std::max(0.0, velocity_limit);
    for (int n = 0; n < N; ++n) {
      umax_dyn[2 * n] = std::min(umax_dyn[2 * n], limit);
      ur[2 * n] = std::min(ur[2 * n], limit);
    }
  }

  void reset_overtake_line_state(const double now_sec, const std::string & reason)
  {
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
    overtake_line_state_.phase_start_sec = now_sec;
    overtake_line_state_.phase_start_ey = current_ey;
    overtake_line_state_.target_ey = current_ey;
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
      return static_cast<double>(pass_side_sign) *
             std::max(0.0, cfg.v2x_behavior.overtake_line.lateral_offset);
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

    const bool behavior_overtake =
      behavior_output.state == V2XBehaviorState::Overtake &&
      behavior_output.overtake_pass_side_sign != 0;
    const bool phase_active = overtake_line_state_.phase != OvertakeLinePhase::Idle;

    if (behavior_overtake) {
      const int pass_side_sign = overtake_line_state_.pass_side_sign != 0 ?
        overtake_line_state_.pass_side_sign : behavior_output.overtake_pass_side_sign;
      if (!phase_active || overtake_line_state_.phase == OvertakeLinePhase::FollowPrepare) {
        transition_overtake_line_phase(
          OvertakeLinePhase::ShiftOut, now_sec, current_ey, pass_side_sign,
          "overtake selected");
      } else if (overtake_line_state_.pass_side_sign == 0) {
        overtake_line_state_.pass_side_sign = pass_side_sign;
      }
    } else if (
      overtake_line_state_.phase == OvertakeLinePhase::ShiftOut ||
      overtake_line_state_.phase == OvertakeLinePhase::Pass) {
      if (!behavior_output.has_front_vehicle && !behavior_output.has_side_vehicle) {
        transition_overtake_line_phase(
          OvertakeLinePhase::Return, now_sec, current_ey, overtake_line_state_.pass_side_sign,
          "front clear");
      } else if (behavior_output.has_front_vehicle) {
        transition_overtake_line_phase(
          OvertakeLinePhase::Recovery, now_sec, current_ey, overtake_line_state_.pass_side_sign,
          "overtake gap lost");
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
    const double phase_traveled = std::max(0.0, current_speed_mps_) * phase_elapsed;
    const bool phase_hold_elapsed =
      phase_elapsed >= std::max(0.0, line_cfg.phase_hold_time);

    const double lateral_offset = std::max(0.0, line_cfg.lateral_offset);
    const double raw_goal_for_phase = overtake_line_goal_ey();
    if (
      overtake_line_state_.phase == OvertakeLinePhase::ShiftOut && phase_hold_elapsed &&
      (phase_traveled >= std::max(0.5, line_cfg.shift_distance) ||
      std::abs(current_ey - raw_goal_for_phase) <= std::max(0.15, 0.25 * lateral_offset))) {
      transition_overtake_line_phase(
        OvertakeLinePhase::Pass, now_sec, current_ey, overtake_line_state_.pass_side_sign,
        "shift complete");
    }
    if (
      overtake_line_state_.phase == OvertakeLinePhase::Pass && phase_hold_elapsed &&
      !behavior_output.has_front_vehicle && !behavior_output.has_side_vehicle) {
      transition_overtake_line_phase(
        OvertakeLinePhase::Return, now_sec, current_ey, overtake_line_state_.pass_side_sign,
        "front and side clear");
    }
    if (
      overtake_line_state_.phase == OvertakeLinePhase::Return && phase_hold_elapsed &&
      (phase_traveled >= std::max(0.5, line_cfg.return_distance) || std::abs(current_ey) < 0.15)) {
      reset_overtake_line_state(now_sec, "return complete");
      return output;
    }
    if (
      overtake_line_state_.phase == OvertakeLinePhase::Recovery && phase_hold_elapsed &&
      (phase_traveled >= std::max(0.5, line_cfg.return_distance) || std::abs(current_ey) < 0.20)) {
      reset_overtake_line_state(now_sec, "recovery complete");
      return output;
    }

    if (overtake_line_state_.phase == OvertakeLinePhase::Idle) {
      return output;
    }

    const double raw_goal = overtake_line_goal_ey();
    const double goal_ey = limit_overtake_line_goal_change(raw_goal);
    const double phase_distance = overtake_line_phase_distance();
    const double min_wall_clearance = std::max(0.0, line_cfg.min_wall_clearance);
    const double max_lateral_accel = std::max(0.0, line_cfg.max_lateral_accel);
    const double speed_for_time = std::max(1.0, current_speed_mps_);

    output.active = true;
    output.target_ey.assign(N, 0.0);
    output.target_active.assign(N, true);
    if (overtake_line_state_.phase == OvertakeLinePhase::Recovery) {
      output.target_velocity_limit = std::max(0.0, cfg.v2x_behavior.follow_velocity);
    }

    for (int i = 0; i < N; ++i) {
      const double distance = horizon_path_distance_to_index(ref_wp_id, static_cast<std::size_t>(i));
      const double progress = overtake_line_state_.phase == OvertakeLinePhase::Pass ?
        1.0 : smoothstep(distance / phase_distance);
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
          "current_ey=%.2f, max_lat_acc=%.2f, lat_limited=%d, wall_limited=%d",
          to_string(overtake_line_state_.phase), overtake_line_state_.pass_side_sign, goal_ey,
          output.target_ey.empty() ? 0.0 : output.target_ey.front(), current_ey,
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

  bool is_overtake_forbidden_curvature(
    const int ref_wp_id, const int N, const double configured_lookahead_distance) const
  {
    const double threshold = std::max(0.0, cfg.v2x_behavior.overtake_max_curvature);
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
    const double current_ey, std::string & block_reason) const
  {
    const double min_front_distance =
      std::max(0.0, cfg.v2x_behavior.overtake_guard_min_front_distance);
    if (min_front_distance > kEps && front_distance < min_front_distance) {
      block_reason = "overtake guard front distance";
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
    if (min_prepare_distance > kEps && reachable_gap.first_gap_distance < min_prepare_distance) {
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
    const double lower, const double upper, std::string & block_reason) const
  {
    if (pass_side_sign == 0 || upper - lower <= kEps) {
      block_reason = "overtake fallback guard invalid side";
      return false;
    }
    if (!std::isfinite(front_distance)) {
      block_reason = "overtake fallback guard front distance";
      return false;
    }

    const double min_front_distance =
      std::max(0.0, cfg.v2x_behavior.overtake_guard_min_front_distance);
    const double min_prepare_distance =
      std::max(0.0, cfg.v2x_behavior.overtake_guard_min_prepare_distance);
    const double required_front_distance = std::max(min_front_distance, min_prepare_distance);
    if (required_front_distance > kEps && front_distance < required_front_distance) {
      std::ostringstream ss;
      ss << "overtake fallback guard front distance"
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

  double follow_velocity_limit(const double front_distance, const double front_speed) const
  {
    if (
      std::isfinite(front_speed) &&
      front_speed > cfg.v2x_behavior.moving_front_speed_threshold) {
      return std::min(
        cfg.v_max, front_speed + std::max(0.0, cfg.v2x_behavior.moving_follow_speed_margin));
    }
    return front_distance_velocity_limit(front_distance);
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
    if (cfg.v2x_behavior.follow_speed_limit_enabled) {
      output.target_velocity_limit =
        std::min(output.target_velocity_limit, follow_velocity_limit(front_distance, front_speed));
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
    if (v2x_behavior_state_initialized) {
      const double elapsed = now_sec - last_v2x_behavior_state_change_sec;
      const bool more_restrictive =
        behavior_restriction_rank(final_state) > behavior_restriction_rank(v2x_behavior_state);
      if (!more_restrictive && elapsed < cfg.v2x_behavior.state_hold_time) {
        final_state = v2x_behavior_state;
      }
    }

    if (final_state == V2XBehaviorState::Overtake) {
      output.allow_gap_planner = true;
    } else if (final_state == V2XBehaviorState::LowSpeedAvoidance) {
      output.allow_gap_planner = true;
      output.target_velocity_limit = std::min(
        output.target_velocity_limit, std::max(0.0, cfg.v2x_behavior.low_speed_avoidance_velocity));
    } else if (final_state == V2XBehaviorState::Follow) {
      const bool follow_gap_planner_allowed =
        !cfg.v2x_behavior.follow_gap_planner_respect_overtake_forbidden ||
        output.follow_gap_planner_allowed;
      if (cfg.v2x_behavior.follow_gap_planner_enabled && follow_gap_planner_allowed) {
        output.allow_gap_planner = true;
      }
      if (cfg.v2x_behavior.follow_speed_limit_enabled) {
        output.target_velocity_limit = std::min(
        output.target_velocity_limit, std::max(0.0, cfg.v2x_behavior.follow_velocity));
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
        "V2X behavior: %s -> %s, front_distance=%.2f, wp_id=%d, reason=%s",
        v2x_behavior_state_initialized ? to_string(v2x_behavior_state) : "None", to_string(final_state),
        output.front_distance, model->wp_id, output.reason.c_str());
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
          "V2X debug: desired=%s, final=%s, allow_gap=%d, limit=%.2f, wp_id=%d, "
          "vehicles=%zu, front=%d, side=%d, danger=%d, grace=%d, "
          "fd=%.2f, fs=%.2f, ego=%.2f, rel=%.2f, req_dec=%.2f, avail=%.2f, risk=%s, "
          "forbid=%d, forbid_wp=%d, curve_guard=%d, low_speed=%d, low_speed_block=%d, "
          "zone=%d, start_curve=%d, before_curve=%d, continue=%d, gap=%d, fallback=%d, "
          "cooldown=%d, pass=%d, "
          "side_clear=%.2f, plan_N=%d, reason=%s, block=%s",
          to_string(desired_state), to_string(final_state), output.allow_gap_planner ? 1 : 0,
          output.target_velocity_limit, model->wp_id, output.active_vehicle_count,
          output.has_front_vehicle ? 1 : 0, output.has_side_vehicle ? 1 : 0,
          output.has_danger_vehicle ? 1 : 0, output.start_grid_grace_active ? 1 : 0,
          output.front_distance, output.front_speed, output.ego_speed,
          output.front_risk.relative_speed, output.front_risk.required_decel,
          output.front_risk.available_distance, to_string(output.front_risk_level),
          output.overtake_forbidden ? 1 : 0, output.overtake_forbidden_wp ? 1 : 0,
          output.front_decel_curve_guard ? 1 : 0,
          output.low_speed_avoidance_candidate ? 1 : 0,
          output.low_speed_avoidance_gap_blocked ? 1 : 0,
          output.overtake_zone_allows ? 1 : 0,
          output.overtake_start_curve_blocked ? 1 : 0,
          output.before_curve_overtake_allowed ? 1 : 0,
          output.continuing_overtake_allowed ? 1 : 0,
          output.overtake_gap_available ? 1 : 0,
          output.overtake_fallback_target ? 1 : 0,
          output.overtake_cooldown_active ? 1 : 0, output.overtake_pass_side_sign,
          output.overtake_side_clearance, output.overtake_plan_N,
          output.reason.c_str(), output.overtake_block_reason.c_str());
        last_v2x_behavior_debug_log_sec_ = now_sec;
      }
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
  MpcConfig mpc;
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
  cfg.reference_path.smoothing_distance = ref["smoothing_distance"].as<int>();
  cfg.reference_path.max_width = ref["max_width"].as<double>();
  cfg.reference_path.circular = ref["circular"].as<bool>();
  cfg.reference_path.use_path_constraints_topic = ref["use_path_constraints_topic"].as<bool>();
  cfg.reference_path.use_border_cells_topic = ref["use_border_cells_topic"].as<bool>();
  cfg.bicycle_length = root["bicycle_model"]["length"].as<double>();
  cfg.bicycle_width = root["bicycle_model"]["width"].as<double>();

  const auto mpc = root["mpc"];
  cfg.mpc.N = mpc["N"].as<int>();
  const auto Q = mpc["Q"].as<std::vector<double>>();
  const auto R = mpc["R"].as<std::vector<double>>();
  const auto QN = mpc["QN"].as<std::vector<double>>();
  cfg.mpc.Q = Eigen::Vector3d(Q.at(0), Q.at(1), Q.at(2));
  cfg.mpc.R = Eigen::Vector2d(R.at(0), R.at(1));
  cfg.mpc.QN = Eigen::Vector3d(QN.at(0), QN.at(1), QN.at(2));
  cfg.mpc.v_max_kmh = mpc["v_max"].as<double>();
  cfg.mpc.v_max = kmh_to_m_per_sec(cfg.mpc.v_max_kmh);
  if (ros_domain_id.has_value()) {
    cfg.mpc.ros_domain_id = ros_domain_id.value();
  }
  if (mpc["domain_v_max"] && ros_domain_id.has_value()) {
    for (const auto & item : mpc["domain_v_max"]) {
      if (item.first.as<int>() != ros_domain_id.value()) {
        continue;
      }
      cfg.mpc.v_max_kmh = std::max(0.0, item.second.as<double>());
      cfg.mpc.v_max = kmh_to_m_per_sec(cfg.mpc.v_max_kmh);
      cfg.mpc.domain_v_max_applied = true;
      break;
    }
  }
  cfg.mpc.domain_start_v_max_duration = std::max(
    0.0,
    mpc["domain_start_v_max_duration"] ?
    mpc["domain_start_v_max_duration"].as<double>() : 0.0);
  if (mpc["domain_start_v_max"] && ros_domain_id.has_value()) {
    for (const auto & item : mpc["domain_start_v_max"]) {
      if (item.first.as<int>() != ros_domain_id.value()) {
        continue;
      }
      cfg.mpc.domain_start_v_max_kmh = std::max(0.0, item.second.as<double>());
      cfg.mpc.domain_start_v_max = kmh_to_m_per_sec(cfg.mpc.domain_start_v_max_kmh);
      cfg.mpc.domain_start_v_max_applied = true;
      break;
    }
  }
  cfg.mpc.a_min = mpc["a_min"].as<double>();
  cfg.mpc.a_max = mpc["a_max"].as<double>();
  if (mpc["domain_a_max"] && ros_domain_id.has_value()) {
    for (const auto & item : mpc["domain_a_max"]) {
      if (item.first.as<int>() != ros_domain_id.value()) {
        continue;
      }
      cfg.mpc.a_max = std::max(0.0, item.second.as<double>());
      cfg.mpc.domain_a_max_applied = true;
      break;
    }
  }
  cfg.mpc.ay_max = mpc["ay_max"].as<double>();
  cfg.mpc.delta_max = mpc["delta_max_deg"].as<double>() * kPi / 180.0;
  cfg.mpc.steer_rate_max = mpc["steer_rate_max"].as<double>();
  cfg.mpc.control_rate = mpc["control_rate"].as<double>();
  cfg.mpc.steering_tire_angle_gain_var = mpc["steering_tire_angle_gain_var"].as<double>();
  cfg.mpc.accel_low_pass_gain = mpc["accel_low_pass_gain"].as<double>();
  cfg.mpc.steer_low_pass_gain = mpc["steer_low_pass_gain"].as<double>();
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
  cfg.mpc.v2x_behavior.follow_velocity = std::max(
    0.0, mpc["v2x_follow_velocity"] ? mpc["v2x_follow_velocity"].as<double>() : 5.0);
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
  cfg.mpc.v2x_behavior.overtake_gap_lookahead_distance = std::max(
    0.0,
    mpc["v2x_overtake_gap_lookahead_distance"] ?
    mpc["v2x_overtake_gap_lookahead_distance"].as<double>() : 0.0);
  cfg.mpc.v2x_behavior.moving_front_speed_threshold = std::max(
    0.0,
    mpc["v2x_moving_front_speed_threshold"] ?
    mpc["v2x_moving_front_speed_threshold"].as<double>() : 1.0);
  cfg.mpc.v2x_behavior.moving_follow_speed_margin = std::max(
    0.0,
    mpc["v2x_moving_follow_speed_margin"] ?
    mpc["v2x_moving_follow_speed_margin"].as<double>() : 2.0);
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
    }
    if (mpc_cfg_.v2x_gap.enabled) {
      RCLCPP_WARN(get_logger(), "USE_V2X_GAP_PLANNER is enabled!");
    }
    if (mpc_cfg_.v2x_behavior.enabled) {
      RCLCPP_WARN(get_logger(), "USE_V2X_BEHAVIOR_FSM is enabled!");
      if (mpc_cfg_.v2x_behavior.debug_log_enabled) {
        RCLCPP_INFO(
          get_logger(), "V2X behavior debug log is enabled: period=%.2f sec",
          mpc_cfg_.v2x_behavior.debug_log_period_sec);
      }
    }
    if (mpc_cfg_.v2x_behavior.overtake_line.enabled) {
      RCLCPP_INFO(
        get_logger(),
        "V2X overtake line is enabled: offset=%.2f, shift=%.2f, pass=%.2f, "
        "return=%.2f, bias=%.2f",
        mpc_cfg_.v2x_behavior.overtake_line.lateral_offset,
        mpc_cfg_.v2x_behavior.overtake_line.shift_distance,
        mpc_cfg_.v2x_behavior.overtake_line.pass_distance,
        mpc_cfg_.v2x_behavior.overtake_line.return_distance,
        mpc_cfg_.v2x_behavior.overtake_line.target_bias);
    }
    if (mpc_cfg_.v2x_behavior.low_speed_avoidance_enabled) {
      RCLCPP_INFO(
        get_logger(), "V2X low-speed pass: side=%s, ramp_ratio=%.2f",
        mpc_cfg_.v2x_gap.low_speed_pass_side.c_str(),
        mpc_cfg_.v2x_gap.low_speed_pass_ramp_ratio);
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
    std::vector<double> wp_x;
    std::vector<double> wp_y;
    if (!cfg_.reference_path.csv_path.empty()) {
      std::tie(wp_x, wp_y) = load_ref_path(in_pkg_share(cfg_.reference_path.csv_path));
    } else {
      std::tie(wp_x, wp_y) = load_waypoints(in_pkg_share(cfg_.waypoints_csv_path));
    }
    reference_path_ = std::make_unique<ReferencePath>(
      map_.get(), wp_x, wp_y, cfg_.reference_path.resolution, cfg_.reference_path.smoothing_distance,
      cfg_.reference_path.max_width, cfg_.reference_path.circular);
    car_ = std::make_unique<BicycleModel>(
      reference_path_.get(), cfg_.bicycle_length, cfg_.bicycle_width, mpc_cfg_.safety_margin_scale,
      1.0 / cfg_.mpc.control_rate);
    mpc_ = std::make_unique<MPC>(
      car_.get(), mpc_cfg_, use_obstacle_avoidance_, cfg_.reference_path.use_path_constraints_topic);
    if (mpc_cfg_.v2x_gap.enabled || mpc_cfg_.v2x_behavior.enabled) {
      auto planner_cfg = mpc_cfg_.v2x_gap;
      if (mpc_cfg_.v2x_behavior.enabled) {
        planner_cfg.enabled = true;
      }
      v2x_gap_planner_ = std::make_unique<V2XGapPlanner>(planner_cfg);
      mpc_->set_gap_planner(v2x_gap_planner_.get());
    }
    reference_path_->compute_speed_profile(
      mpc_cfg_.a_min, mpc_cfg_.a_max, 0.0,
      use_bug_acc_ ? kmh_to_m_per_sec(40.0) : mpc_cfg_.v_max, mpc_cfg_.ay_max);
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
        for (const auto & param : params) {
          const auto & name = param.get_name();
          if (name == "v_max" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
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
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
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

    odom_sub_ = create_subscription<Odometry>(
      "/localization/kinematic_state", 1, [this](const Odometry::SharedPtr msg) { odom_ = msg; });
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
    if (use_sim_time_) {
      awsim_status_sub_ = create_subscription<Float32MultiArray>(
        "/awsim/status", 1, [this](const Float32MultiArray::SharedPtr msg) { awsim_status_callback(msg); });
      condition_sub_ = create_subscription<Int32>(
        "/aichallenge/pitstop/condition", 1, [this](const Int32::SharedPtr msg) {
          if (!last_condition_.has_value()) {
            last_condition_ = msg->data;
          }
          const int diff_condition = msg->data - last_condition_.value();
          if (diff_condition > 30) {
            last_colliding_time_ = now();
            RCLCPP_WARN(get_logger(), "Collision detected!");
          }
          last_condition_ = msg->data;
        });
    }
    if (use_obstacle_avoidance_ && cfg_.reference_path.use_path_constraints_topic) {
      path_constraints_sub_ = create_subscription<PathConstraints>(
        "/path_constraints_provider/path_constraints", 1, [this](const PathConstraints::SharedPtr msg) {
          if (!reference_path_->set_path_constraints(msg->upper_bounds, msg->lower_bounds, msg->rows, msg->cols)) {
            RCLCPP_WARN(get_logger(), "Invalid path constraints message ignored");
          }
        });
    }
    if (use_obstacle_avoidance_ && cfg_.reference_path.use_border_cells_topic) {
      border_cells_sub_ = create_subscription<BorderCells>(
        "/path_constraints_provider/border_cells", 1, [this](const BorderCells::SharedPtr msg) {
          if (!reference_path_->set_border_cells(
              msg->dynamic_upper_bounds, msg->dynamic_lower_bounds, msg->rows, msg->cols)) {
            RCLCPP_WARN(get_logger(), "Invalid border cells message ignored");
          }
        });
    }
    if (use_obstacle_avoidance_ || mpc_cfg_.v2x_gap.enabled || mpc_cfg_.v2x_behavior.enabled) {
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

  void publish_control_command(
    const rclcpp::Time & stamp, const Eigen::Vector2d & u, const double acc, const bool bug_acc_enabled)
  {
    auto cmd = create_ackermann_control_command(stamp, u, acc);
    if (use_bug_acc_) {
      AckermannControlBoostCommand boost_cmd;
      boost_cmd.command = cmd;
      boost_cmd.boost_mode = bug_acc_enabled;
      boost_command_pub_->publish(boost_cmd);
      return;
    }
    command_raw_pub_->publish(cmd);
    cmd.lateral.steering_tire_angle *= mpc_cfg_.steering_tire_angle_gain_var;
    command_pub_->publish(cmd);
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

  void control()
  {
    if (!odom_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "odometry is not available");
      return;
    }
    if (cfg_.reference_path.update_by_topic && !trajectory_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "trajectory is not available");
      return;
    }
    if (!initialized_) {
      const auto pose = odom_to_pose_2d(*odom_);
      car_->update_states(pose.x, pose.y, pose.theta);
      car_->update_reference_path(reference_path_.get());
      last_t_ = now();
      t_start_ = last_t_;
      initialized_ = true;
      pred_marker_color_.r = 0.0;
      pred_marker_color_.g = 156.0 / 255.0;
      pred_marker_color_.b = 209.0 / 255.0;
      pred_marker_color_.a = 1.0;
      publish_ref_path_marker();
      RCLCPP_INFO(get_logger(), "START!");
    }

    const auto current_time = now();
    const double dt = (current_time - last_t_).seconds();
    last_t_ = current_time;
    ++loop_;

    if (loop_ % 100 == 0 && cfg_.reference_path.update_by_topic && trajectory_) {
      auto new_reference_path = create_reference_path_from_autoware_trajectory(*trajectory_);
      if (new_reference_path) {
        reference_path_ = std::move(new_reference_path);
        car_->update_reference_path(reference_path_.get());
        ref_path_published_ = false;
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

    const auto pose = odom_to_pose_2d(*odom_);
    const double actual_v = odom_->twist.twist.linear.x;
    car_->update_states(pose.x, pose.y, pose.theta);
    mpc_->update_current_speed(std::abs(actual_v));

    double effective_v_max = mpc_cfg_.v_max;
    if (
      mpc_cfg_.domain_start_v_max_applied &&
      mpc_cfg_.domain_start_v_max_duration > 0.0 &&
      std::isfinite(mpc_cfg_.domain_start_v_max)) {
      const double elapsed_sec = (current_time - t_start_).seconds();
      if (elapsed_sec <= mpc_cfg_.domain_start_v_max_duration) {
        effective_v_max = std::min(effective_v_max, mpc_cfg_.domain_start_v_max);
      }
    }
    if (ref_vel_configulator_) {
      const double ref_vel_kmph = ref_vel_configulator_->get_ref_vel(mpc_->model->wp_id);
      effective_v_max = std::min(kmh_to_m_per_sec(ref_vel_kmph), effective_v_max);
    }
    mpc_->update_v_max(effective_v_max);
    reference_path_->set_v_ref(std::vector<double>(reference_path_->waypoints.size(), effective_v_max));

    auto [u, max_delta] = mpc_->get_control(current_time.seconds());

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
    if (use_bug_acc_) {
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

    acc = last_acc_ + (acc - last_acc_) * mpc_cfg_.accel_low_pass_gain;
    u[1] = last_u_[1] + (u[1] - last_u_[1]) * mpc_cfg_.steer_low_pass_gain;
    last_acc_ = acc;
    last_u_ = u;
    car_->drive(Eigen::Vector2d(actual_v, u[1]));
    publish_control_command(current_time, u, acc, bug_acc_enabled);

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
    command_pub_->publish(create_ackermann_control_command(now(), zero, 0.0));
  }

  std::string config_path_;
  std::optional<std::string> ref_vel_config_path_;
  Config cfg_;
  MpcConfig mpc_cfg_;
  bool use_sim_time_{};
  bool use_bug_acc_{};
  bool use_obstacle_avoidance_{};
  bool use_stats_{};
  bool enable_control_{true};
  bool initialized_{false};
  bool ref_path_published_{false};
  int current_laps_{1};
  double last_lap_time_{0.0};
  std::optional<int> last_condition_;
  std::optional<rclcpp::Time> last_colliding_time_;
  rclcpp::Time t_start_;
  rclcpp::Time last_t_;
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

  rclcpp::Publisher<AckermannControlCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr command_raw_pub_;
  rclcpp::Publisher<AckermannControlBoostCommand>::SharedPtr boost_command_pub_;
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
  rclcpp::Subscription<Int32>::SharedPtr condition_sub_;
  rclcpp::Subscription<PathConstraints>::SharedPtr path_constraints_sub_;
  rclcpp::Subscription<BorderCells>::SharedPtr border_cells_sub_;
  rclcpp::Subscription<V2XVehiclePositionArray>::SharedPtr v2x_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr ref_vel_marker_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  Odometry::SharedPtr odom_;
  Trajectory::SharedPtr trajectory_;
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
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
