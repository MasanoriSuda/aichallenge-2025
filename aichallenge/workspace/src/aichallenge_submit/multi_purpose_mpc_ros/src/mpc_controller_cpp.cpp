#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
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
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
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
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

constexpr double kEps = 1e-12;
constexpr double kPi = 3.14159265358979323846;

double clip(const double value, const double min_value, const double max_value)
{
  return std::min(std::max(value, min_value), max_value);
}

double wrap_to_pi(const double angle)
{
  return std::fmod(angle + kPi, 2.0 * kPi) - kPi;
}

double kmh_to_m_per_sec(const double kmh)
{
  return kmh / 3.6;
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
  BicycleModel(ReferencePath * ref_path, const double length_in, const double width_in, const double Ts_in)
  : length(length_in), width(width_in), safety_margin(compute_safety_margin()), reference_path(ref_path), Ts(Ts_in)
  {
    current_waypoint = &reference_path->waypoints.at(wp_id);
    temporal_state = s2t(*current_waypoint, spatial_state);
  }

  double compute_safety_margin() const
  {
    return width / std::sqrt(2.0);
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
  double safety_margin{};
  ReferencePath * reference_path{};
  double s{0.0};
  double Ts{};
  int wp_id{0};
  Waypoint * current_waypoint{};
  SimpleSpatialState spatial_state;
  TemporalState temporal_state;
};

struct MpcConfig
{
  int N{};
  Eigen::Vector3d Q;
  Eigen::Vector2d R;
  Eigen::Vector3d QN;
  double v_max{};
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
    if (!use_obstacle_avoidance) {
      model->reference_path->update_simple_path_constraints(cfg.N, model->safety_margin);
    }
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

  MpcProblem init_problem(const int N, const double safety_margin)
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

    model->wp_id += cfg.wp_id_offset;

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

    xmin_dyn[0] = model->spatial_state.e_y;
    xmax_dyn[0] = model->spatial_state.e_y;
    for (int i = 0; i < N; ++i) {
      xmin_dyn[nx + i * nx] = lb[i];
      xmax_dyn[nx + i * nx] = ub[i];
      xr[nx + i * nx] = (lb[i] + ub[i]) / 2.0;
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
    for (int i = 0; i < N - 1; ++i) {
      a_triplets.emplace_back(rate_offset + i, nx_N + nu * i + 1, -1.0);
      a_triplets.emplace_back(rate_offset + i, nx_N + nu * (i + 1) + 1, 1.0);
    }

    Eigen::SparseMatrix<double> A_full(nx_N + nx_N + nu_N + (N - 1), nx_N + nu_N);
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
    Eigen::VectorXd lineq_rate = Eigen::VectorXd::Constant(N - 1, -max_delta_change);
    Eigen::VectorXd uineq_rate = Eigen::VectorXd::Constant(N - 1, max_delta_change);

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

  std::pair<Eigen::Vector2d, double> get_control()
  {
    constexpr int nx = 3;
    constexpr int nu = 2;
    model->get_current_waypoint();
    const int N = model->reference_path->circular ?
      cfg.N :
      std::min(cfg.N, model->reference_path->n_waypoints - model->wp_id);

    model->spatial_state = model->t2s(*model->current_waypoint, model->temporal_state);
    MpcProblem problem = init_problem(N, model->safety_margin);

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
          problem = init_problem(N, relaxed_safety_margin);
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
  bool use_obstacle_avoidance{};
  bool use_path_constraints_topic{};
  double previous_steering{0.0};
  std::pair<std::vector<double>, std::vector<double>> current_prediction;
  int infeasibility_counter{0};
  int last_solved_wp_id{0};
  Eigen::VectorXd current_control;
};

struct RefPathConfig
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
  cfg.mpc.v_max = kmh_to_m_per_sec(mpc["v_max"].as<double>());
  cfg.mpc.a_min = mpc["a_min"].as<double>();
  cfg.mpc.a_max = mpc["a_max"].as<double>();
  cfg.mpc.ay_max = mpc["ay_max"].as<double>();
  cfg.mpc.delta_max = mpc["delta_max_deg"].as<double>() * kPi / 180.0;
  cfg.mpc.steer_rate_max = mpc["steer_rate_max"].as<double>();
  cfg.mpc.control_rate = mpc["control_rate"].as<double>();
  cfg.mpc.steering_tire_angle_gain_var = mpc["steering_tire_angle_gain_var"].as<double>();
  cfg.mpc.accel_low_pass_gain = mpc["accel_low_pass_gain"].as<double>();
  cfg.mpc.steer_low_pass_gain = mpc["steer_low_pass_gain"].as<double>();
  cfg.mpc.wp_id_offset = mpc["wp_id_offset"].as<int>();
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
    control_timer_ = create_wall_timer(10ms, std::bind(&MPCControllerCpp::control, this));
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
      reference_path_.get(), cfg_.bicycle_length, cfg_.bicycle_width, 1.0 / cfg_.mpc.control_rate);
    mpc_ = std::make_unique<MPC>(
      car_.get(), mpc_cfg_, use_obstacle_avoidance_, cfg_.reference_path.use_path_constraints_topic);
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

    param_callback_handle_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        for (const auto & param : params) {
          const auto & name = param.get_name();
          if (name == "v_max" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            mpc_cfg_.v_max = param.as_double();
            const double v_mps = kmh_to_m_per_sec(param.as_double());
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
    if (use_obstacle_avoidance_) {
      path_constraints_sub_ = create_subscription<PathConstraints>(
        "/path_constraints_provider/path_constraints", 1, [](const PathConstraints::SharedPtr) {});
      border_cells_sub_ = create_subscription<BorderCells>(
        "/path_constraints_provider/border_cells", 1, [](const BorderCells::SharedPtr) {});
      v2x_sub_ = create_subscription<v2x_msgs::msg::V2XVehiclePositionArray>(
        "/v2x/vehicle_positions", 1, [](const v2x_msgs::msg::V2XVehiclePositionArray::SharedPtr) {});
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

    auto [u, max_delta] = mpc_->get_control();
    if (ref_vel_configulator_) {
      const double ref_vel_kmph = ref_vel_configulator_->get_ref_vel(mpc_->model->wp_id);
      const double ref_vel_mps = std::min(kmh_to_m_per_sec(ref_vel_kmph), mpc_cfg_.v_max);
      mpc_->update_v_max(ref_vel_mps);
      reference_path_->set_v_ref(std::vector<double>(reference_path_->waypoints.size(), ref_vel_mps));
    }

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
  rclcpp::Subscription<v2x_msgs::msg::V2XVehiclePositionArray>::SharedPtr v2x_sub_;
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
