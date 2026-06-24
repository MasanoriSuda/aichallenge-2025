#include "multi_purpose_mpc_ros_cpp/osqp_mpc.hpp"

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SparseCore>
#include <osqp_interface/osqp_interface.hpp>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <algorithm>
#include <array>
#include <cctype>
#include <cstdint>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>

namespace multi_purpose_mpc_ros_cpp
{

namespace
{

using SparseMatrix = Eigen::SparseMatrix<double>;
using Triplet = Eigen::Triplet<double>;
constexpr double kOsqpInfinity = 1.0e10;
constexpr double kOsqpTolerance = 1.0e-3;

struct OccupancyMap
{
  int width{};
  int height{};
  double resolution{};
  std::array<double, 3> origin{};
  std::vector<std::uint8_t> data;

  static OccupancyMap load(const std::string & map_yaml_path)
  {
    const auto map_yaml = YAML::LoadFile(map_yaml_path);
    const auto base_dir = std::filesystem::path(map_yaml_path).parent_path();
    const auto image_path = base_dir / map_yaml["image"].as<std::string>();
    const auto occupied_threshold = map_yaml["occupied_thresh"].as<double>();

    std::ifstream image_stream(image_path, std::ios::binary);
    if (!image_stream) {
      throw std::runtime_error("Failed to open occupancy image: " + image_path.string());
    }

    const auto read_token = [&image_path](std::istream & stream) -> std::string {
        std::string token;
        char ch = '\0';
        while (stream.get(ch)) {
          if (std::isspace(static_cast<unsigned char>(ch))) {
            continue;
          }
          if (ch == '#') {
            stream.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            continue;
          }
          token.push_back(ch);
          break;
        }
        while (stream.get(ch)) {
          if (std::isspace(static_cast<unsigned char>(ch))) {
            break;
          }
          if (ch == '#') {
            stream.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            break;
          }
          token.push_back(ch);
        }
        if (token.empty()) {
          throw std::runtime_error("Unexpected end of PGM header while parsing " + image_path.string());
        }
        return token;
      };

    const auto magic = read_token(image_stream);
    const auto width = std::stoi(read_token(image_stream));
    const auto height = std::stoi(read_token(image_stream));
    const auto max_value = std::stoi(read_token(image_stream));
    if (width <= 0 || height <= 0 || max_value <= 0) {
      throw std::runtime_error("Invalid occupancy image header: " + image_path.string());
    }

    std::vector<std::uint8_t> raw_data(static_cast<std::size_t>(width * height), 0);
    if (magic == "P5") {
      image_stream.read(reinterpret_cast<char *>(raw_data.data()), static_cast<std::streamsize>(raw_data.size()));
      if (image_stream.gcount() != static_cast<std::streamsize>(raw_data.size())) {
        throw std::runtime_error("Failed to read occupancy image payload: " + image_path.string());
      }
    } else if (magic == "P2") {
      for (auto & value : raw_data) {
        value = static_cast<std::uint8_t>(std::stoi(read_token(image_stream)));
      }
    } else {
      throw std::runtime_error("Unsupported occupancy image format: " + magic);
    }

    OccupancyMap map;
    map.width = width;
    map.height = height;
    map.resolution = map_yaml["resolution"].as<double>();
    const auto origin = map_yaml["origin"];
    map.origin = {
      origin[0].as<double>(),
      origin[1].as<double>(),
      origin[2].as<double>()};
    map.data.resize(raw_data.size(), 0);
    for (std::size_t i = 0; i < raw_data.size(); ++i) {
      const auto normalized = static_cast<double>(raw_data[i]) / static_cast<double>(max_value);
      map.data[i] = normalized >= occupied_threshold ? 1U : 0U;
    }
    return map;
  }

  std::pair<int, int> world_to_map(double x, double y) const
  {
    auto dx = static_cast<int>((x - origin[0]) / resolution + 0.5);
    auto dy = static_cast<int>((height - 1) - (y - origin[1]) / resolution + 0.5);
    dx = std::clamp(dx, 0, width - 1);
    dy = std::clamp(dy, 0, height - 1);
    return {dx, dy};
  }

  std::pair<double, double> map_to_world(int dx, int dy) const
  {
    const auto x = static_cast<int>(dx + 0.5) * resolution + origin[0];
    const auto y = (height - 1 - static_cast<int>(dy + 0.5)) * resolution + origin[1];
    return {x, y};
  }

  bool is_free(int x, int y) const
  {
    return data[static_cast<std::size_t>(y * width + x)] != 0U;
  }
};

std::filesystem::path resolve_path(
  const std::filesystem::path & base_dir, const std::string & path_string)
{
  const auto path = std::filesystem::path(path_string);
  return path.is_absolute() ? path : base_dir / path;
}

std::vector<std::pair<int, int>> trace_line(int x0, int y0, int x1, int y1)
{
  std::vector<std::pair<int, int>> points;

  const auto dx = std::abs(x1 - x0);
  const auto sx = x0 < x1 ? 1 : -1;
  const auto dy = -std::abs(y1 - y0);
  const auto sy = y0 < y1 ? 1 : -1;
  auto error = dx + dy;

  while (true) {
    points.emplace_back(x0, y0);
    if (x0 == x1 && y0 == y1) {
      break;
    }
    const auto e2 = 2 * error;
    if (e2 >= dy) {
      error += dy;
      x0 += sx;
    }
    if (e2 <= dx) {
      error += dx;
      y0 += sy;
    }
  }

  return points;
}

std::vector<double> to_std_vector(const Eigen::VectorXd & vector)
{
  return std::vector<double>(vector.data(), vector.data() + vector.size());
}

void write_csv_matrix(const std::filesystem::path & path, const Eigen::MatrixXd & matrix)
{
  std::ofstream ofs(path);
  for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
    for (Eigen::Index col = 0; col < matrix.cols(); ++col) {
      if (col > 0) {
        ofs << ",";
      }
      ofs << matrix(row, col);
    }
    ofs << "\n";
  }
}

void write_csv_vector(const std::filesystem::path & path, const Eigen::VectorXd & vector)
{
  std::ofstream ofs(path);
  for (Eigen::Index i = 0; i < vector.size(); ++i) {
    ofs << vector(i) << "\n";
  }
}

std::vector<std::string> split_csv_line(const std::string & line)
{
  std::vector<std::string> tokens;
  std::stringstream ss(line);
  std::string token;
  while (std::getline(ss, token, ',')) {
    tokens.push_back(token);
  }
  return tokens;
}

std::tuple<std::vector<double>, std::vector<double>> load_ref_path_xy(const std::string & csv_path)
{
  std::ifstream ifs(csv_path);
  if (!ifs) {
    throw std::runtime_error("Failed to open reference path CSV: " + csv_path);
  }

  std::string header;
  std::getline(ifs, header);
  const auto columns = split_csv_line(header);

  int x_index = -1;
  int y_index = -1;
  for (std::size_t i = 0; i < columns.size(); ++i) {
    if (columns[i] == "x_m") {
      x_index = static_cast<int>(i);
    } else if (columns[i] == "y_m") {
      y_index = static_cast<int>(i);
    }
  }

  if (x_index < 0 || y_index < 0) {
    throw std::runtime_error("reference path CSV must contain x_m and y_m columns");
  }

  std::vector<double> x;
  std::vector<double> y;
  std::string line;
  while (std::getline(ifs, line)) {
    if (line.empty()) {
      continue;
    }
    const auto tokens = split_csv_line(line);
    if (
      static_cast<int>(tokens.size()) <= std::max(x_index, y_index) ||
      tokens[x_index].empty() || tokens[y_index].empty())
    {
      continue;
    }
    x.push_back(std::stod(tokens[x_index]));
    y.push_back(std::stod(tokens[y_index]));
  }

  if (x.size() < 2 || y.size() < 2) {
    throw std::runtime_error("reference path CSV must contain at least 2 rows");
  }

  return {x, y};
}

double normalize_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double distance(const Waypoint & lhs, const Waypoint & rhs)
{
  return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y);
}

std::pair<double, std::pair<double, double>> get_min_width_to_boundary(
  const OccupancyMap & map,
  double waypoint_x_world,
  double waypoint_y_world,
  int waypoint_x_map,
  int waypoint_y_map,
  int target_x_map,
  int target_y_map,
  double max_width)
{
  auto min_width = max_width;
  auto min_cell = map.map_to_world(target_x_map, target_y_map);
  auto best_target_distance_sq = std::numeric_limits<double>::max();

  for (int i = -1; i <= 1; ++i) {
    for (int j = -1; j <= 1; ++j) {
      const auto candidate_x = std::clamp(target_x_map + i, 0, map.width - 1);
      const auto candidate_y = std::clamp(target_y_map + j, 0, map.height - 1);
      const auto path_cells = trace_line(waypoint_x_map, waypoint_y_map, candidate_x, candidate_y);

      for (const auto & cell : path_cells) {
        if (!map.is_free(cell.first, cell.second)) {
          min_cell = map.map_to_world(cell.first, cell.second);
          min_width = std::hypot(waypoint_x_world - min_cell.first, waypoint_y_world - min_cell.second);
          return {min_width, min_cell};
        }

        const auto target_distance_sq =
          static_cast<double>((cell.first - target_x_map) * (cell.first - target_x_map) +
          (cell.second - target_y_map) * (cell.second - target_y_map));
        if (target_distance_sq < best_target_distance_sq) {
          best_target_distance_sq = target_distance_sq;
          min_cell = map.map_to_world(cell.first, cell.second);
          min_width = std::hypot(waypoint_x_world - min_cell.first, waypoint_y_world - min_cell.second);
        }
      }
    }
  }

  return {min_width, min_cell};
}

}  // namespace

ReferencePath::ReferencePath(
  const std::vector<double> & wp_x, const std::vector<double> & wp_y,
  double resolution, int smoothing_distance, double max_width, bool circular)
: resolution_(resolution), smoothing_distance_(smoothing_distance), circular_(circular)
{
  waypoints_ = construct_path(wp_x, wp_y);
  compute_lengths();
  assign_default_bounds(max_width);
}

const Waypoint & ReferencePath::get_waypoint(std::size_t wp_id) const
{
  if (waypoints_.empty()) {
    throw std::runtime_error("reference path is empty");
  }

  if (circular_) {
    return waypoints_[wp_id % waypoints_.size()];
  }

  return waypoints_[std::min(wp_id, waypoints_.size() - 1)];
}

std::size_t ReferencePath::size() const
{
  return waypoints_.size();
}

bool ReferencePath::circular() const
{
  return circular_;
}

const std::vector<double> & ReferencePath::segment_lengths() const
{
  return segment_lengths_;
}

const std::vector<double> & ReferencePath::cumulative_lengths() const
{
  return cumulative_lengths_;
}

void ReferencePath::set_v_ref_all(double v_ref)
{
  for (auto & waypoint : waypoints_) {
    waypoint.v_ref = v_ref;
  }
}

void ReferencePath::assign_bounds_from_map(const std::string & map_yaml_path, double max_width)
{
  if (map_yaml_path.empty()) {
    assign_default_bounds(max_width);
    return;
  }

  const auto map = OccupancyMap::load(map_yaml_path);
  for (auto & waypoint : waypoints_) {
    const auto left_angle = normalize_angle(waypoint.psi + M_PI / 2.0);
    const auto right_angle = normalize_angle(waypoint.psi - M_PI / 2.0);

    const auto [waypoint_x_map, waypoint_y_map] = map.world_to_map(waypoint.x, waypoint.y);
    const auto [waypoint_x_world, waypoint_y_world] = map.map_to_world(waypoint_x_map, waypoint_y_map);

    const auto [left_target_x, left_target_y] = map.world_to_map(
      waypoint_x_world + max_width * std::cos(left_angle),
      waypoint_y_world + max_width * std::sin(left_angle));
    const auto [left_width, left_cell] = get_min_width_to_boundary(
      map,
      waypoint_x_world,
      waypoint_y_world,
      waypoint_x_map,
      waypoint_y_map,
      left_target_x,
      left_target_y,
      max_width);

    const auto [right_target_x, right_target_y] = map.world_to_map(
      waypoint_x_world + max_width * std::cos(right_angle),
      waypoint_y_world + max_width * std::sin(right_angle));
    const auto [right_width, right_cell] = get_min_width_to_boundary(
      map,
      waypoint_x_world,
      waypoint_y_world,
      waypoint_x_map,
      waypoint_y_map,
      right_target_x,
      right_target_y,
      max_width);

    static_cast<void>(left_cell);
    static_cast<void>(right_cell);
    waypoint.ub = left_width;
    waypoint.lb = -right_width;
  }
}

std::vector<Waypoint> ReferencePath::construct_path(
  const std::vector<double> & wp_x_input, const std::vector<double> & wp_y_input) const
{
  auto wp_x = wp_x_input;
  auto wp_y = wp_y_input;

  if (circular_ && !wp_x.empty() && smoothing_distance_ > 0) {
    const auto duplicate_count = std::min<std::size_t>(
      wp_x.size(), static_cast<std::size_t>(smoothing_distance_ * 3));
    wp_x.insert(wp_x.end(), wp_x.begin(), wp_x.begin() + duplicate_count);
    wp_y.insert(wp_y.end(), wp_y.begin(), wp_y.begin() + duplicate_count);
  }

  std::vector<int> n_wp;
  n_wp.reserve(wp_x.size() > 0 ? wp_x.size() - 1 : 0);
  for (std::size_t i = 0; i + 1 < wp_x.size(); ++i) {
    const auto length = std::hypot(wp_x[i + 1] - wp_x[i], wp_y[i + 1] - wp_y[i]);
    const auto count = std::max(1, static_cast<int>(length / resolution_));
    n_wp.push_back(count);
  }

  const auto goal_x = wp_x.back();
  const auto goal_y = wp_y.back();
  std::vector<double> interpolated_x;
  std::vector<double> interpolated_y;
  for (std::size_t i = 0; i + 1 < wp_x.size(); ++i) {
    const auto count = n_wp[i];
    for (int j = 0; j < count; ++j) {
      const auto ratio = static_cast<double>(j) / static_cast<double>(count);
      interpolated_x.push_back(wp_x[i] + (wp_x[i + 1] - wp_x[i]) * ratio);
      interpolated_y.push_back(wp_y[i] + (wp_y[i + 1] - wp_y[i]) * ratio);
    }
  }
  interpolated_x.push_back(goal_x);
  interpolated_y.push_back(goal_y);

  std::vector<std::pair<double, double>> smoothed_points;
  if (smoothing_distance_ > 0 && interpolated_x.size() > static_cast<std::size_t>(2 * smoothing_distance_ + 1)) {
    for (
      std::size_t wp_id = static_cast<std::size_t>(smoothing_distance_);
      wp_id + static_cast<std::size_t>(smoothing_distance_) < interpolated_x.size(); ++wp_id)
    {
      double x_sum = 0.0;
      double y_sum = 0.0;
      const auto begin = wp_id - static_cast<std::size_t>(smoothing_distance_);
      const auto end = wp_id + static_cast<std::size_t>(smoothing_distance_);
      for (std::size_t i = begin; i <= end; ++i) {
        x_sum += interpolated_x[i];
        y_sum += interpolated_y[i];
      }
      const auto count = static_cast<double>(end - begin + 1);
      smoothed_points.emplace_back(x_sum / count, y_sum / count);
    }
  } else {
    for (std::size_t i = 0; i < interpolated_x.size(); ++i) {
      smoothed_points.emplace_back(interpolated_x[i], interpolated_y[i]);
    }
  }

  return construct_waypoints(smoothed_points);
}

std::vector<Waypoint> ReferencePath::construct_waypoints(
  const std::vector<std::pair<double, double>> & waypoint_coordinates) const
{
  std::vector<Waypoint> waypoints;
  if (waypoint_coordinates.size() < 2) {
    return waypoints;
  }

  for (std::size_t wp_id = 0; wp_id + 1 < waypoint_coordinates.size(); ++wp_id) {
    const auto current_wp = waypoint_coordinates[wp_id];
    const auto next_wp = waypoint_coordinates[wp_id + 1];
    const auto dx = next_wp.first - current_wp.first;
    const auto dy = next_wp.second - current_wp.second;
    const auto psi = std::atan2(dy, dx);
    const auto dist_ahead = std::hypot(dx, dy);

    double kappa = 0.0;
    if (wp_id > 0) {
      const auto prev_wp = waypoint_coordinates[wp_id - 1];
      const auto dx_back = current_wp.first - prev_wp.first;
      const auto dy_back = current_wp.second - prev_wp.second;
      const auto angle_behind = std::atan2(dy_back, dx_back);
      const auto angle_dif = normalize_angle(psi - angle_behind);
      kappa = angle_dif / (dist_ahead + eps_);
    }

    waypoints.push_back(Waypoint{
      current_wp.first, current_wp.second, psi, kappa, 0.0, 0.0, 0.0});
  }

  return waypoints;
}

void ReferencePath::compute_lengths()
{
  segment_lengths_.clear();
  cumulative_lengths_.clear();
  segment_lengths_.reserve(waypoints_.size());
  cumulative_lengths_.reserve(waypoints_.size());

  double cumulative = 0.0;
  for (std::size_t i = 0; i < waypoints_.size(); ++i) {
    double segment_length = 0.0;
    if (i > 0) {
      segment_length = distance(waypoints_[i - 1], waypoints_[i]);
    }
    segment_lengths_.push_back(segment_length);
    cumulative += segment_length;
    cumulative_lengths_.push_back(cumulative);
  }
}

void ReferencePath::assign_default_bounds(double max_width)
{
  for (auto & waypoint : waypoints_) {
    waypoint.ub = max_width;
    waypoint.lb = -max_width;
  }
}

bool ReferencePath::compute_speed_profile(double a_min, double a_max, double v_max, double ay_max)
{
  const auto horizon = static_cast<int>(waypoints_.size()) - 1;
  if (horizon < 2) {
    return false;
  }

  Eigen::VectorXd lower_bound((horizon - 1) + horizon);
  Eigen::VectorXd upper_bound((horizon - 1) + horizon);
  lower_bound.head(horizon - 1).setConstant(a_min);
  upper_bound.head(horizon - 1).setConstant(a_max);
  lower_bound.tail(horizon).setZero();
  upper_bound.tail(horizon).setConstant(v_max);

  std::vector<Triplet> d_triplets;
  d_triplets.reserve((horizon - 1) * 2 + horizon);
  for (int i = 0; i < horizon; ++i) {
    const auto & current_waypoint = get_waypoint(static_cast<std::size_t>(i));
    const auto & next_waypoint = get_waypoint(static_cast<std::size_t>(i + 1));
    const auto li = distance(current_waypoint, next_waypoint);
    const auto ki = current_waypoint.kappa;

    if (i < horizon - 1 && li > eps_) {
      d_triplets.emplace_back(i, i, -1.0 / (2.0 * li));
      d_triplets.emplace_back(i, i + 1, 1.0 / (2.0 * li));
    }

    const auto vmax_dyn = std::sqrt(ay_max / (std::abs(ki) + eps_));
    if (vmax_dyn < upper_bound[(horizon - 1) + i]) {
      upper_bound[(horizon - 1) + i] = vmax_dyn;
    }
  }
  for (int i = 0; i < horizon; ++i) {
    d_triplets.emplace_back((horizon - 1) + i, i, 1.0);
  }

  SparseMatrix D((horizon - 1) + horizon, horizon);
  D.setFromTriplets(d_triplets.begin(), d_triplets.end());

  SparseMatrix P(horizon, horizon);
  std::vector<Triplet> p_triplets;
  p_triplets.reserve(horizon);
  for (int i = 0; i < horizon; ++i) {
    p_triplets.emplace_back(i, i, 1.0);
  }
  P.setFromTriplets(p_triplets.begin(), p_triplets.end());

  Eigen::VectorXd q = -upper_bound.tail(horizon);

  autoware::common::osqp::OSQPInterface solver(kOsqpTolerance, true);
  solver.updateEpsRel(kOsqpTolerance);
  const auto result = solver.optimize(
    P.toDense(),
    D.toDense(),
    to_std_vector(q),
    to_std_vector(lower_bound),
    to_std_vector(upper_bound));
  const auto solution = std::get<0>(result);
  if (solver.getStatus() <= 0 || static_cast<int>(solution.size()) != horizon)
  {
    set_v_ref_all(v_max);
    return false;
  }

  for (int i = 0; i < horizon; ++i) {
    waypoints_[static_cast<std::size_t>(i)].v_ref = solution[static_cast<std::size_t>(i)];
  }
  if (circular_) {
    waypoints_.back().v_ref = waypoints_[waypoints_.size() - 2].v_ref;
  } else {
    waypoints_.back().v_ref = 0.0;
  }

  return true;
}

BicycleModel::BicycleModel(ReferencePath reference_path, double length, double width, double ts)
: reference_path_(std::move(reference_path)), length_(length), width_(width), ts_(ts)
{
  safety_margin_ = compute_safety_margin();
  if (reference_path_.size() == 0) {
    throw std::runtime_error("reference path must not be empty");
  }
  const auto & waypoint = reference_path_.get_waypoint(0);
  temporal_state_ = {waypoint.x, waypoint.y, waypoint.psi};
}

void BicycleModel::update_states(double x, double y, double psi)
{
  temporal_state_ = {x, y, psi};
  wp_id_ = get_closest_waypoint(x, y);
  s_ = get_s_at_waypoint(wp_id_);
  get_current_waypoint();
  spatial_state_ = temporal_to_spatial(current_waypoint(), temporal_state_);
}

void BicycleModel::drive(double v, double delta)
{
  const auto x_dot = v * std::cos(temporal_state_.psi);
  const auto y_dot = v * std::sin(temporal_state_.psi);
  const auto psi_dot = v / length_ * std::tan(delta);
  temporal_state_.x += x_dot * ts_;
  temporal_state_.y += y_dot * ts_;
  temporal_state_.psi += psi_dot * ts_;

  const auto s_dot = 1.0 / (1.0 - spatial_state_.e_y * current_waypoint().kappa) *
    v * std::cos(spatial_state_.e_psi);
  s_ += s_dot * ts_;
}

void BicycleModel::get_current_waypoint()
{
  const auto & cumulative = reference_path_.cumulative_lengths();
  auto it = std::upper_bound(cumulative.begin(), cumulative.end(), s_);
  if (it == cumulative.end()) {
    wp_id_ = cumulative.empty() ? 0 : cumulative.size() - 1;
  } else {
    const auto next_wp_id = static_cast<std::size_t>(std::distance(cumulative.begin(), it));
    const auto prev_wp_id = next_wp_id == 0 ? 0 : next_wp_id - 1;
    const auto s_next = cumulative[next_wp_id];
    const auto s_prev = cumulative[prev_wp_id];
    wp_id_ = std::abs(s_ - s_next) < std::abs(s_ - s_prev) ? next_wp_id : prev_wp_id;
  }
}

std::size_t BicycleModel::get_closest_waypoint(double x, double y) const
{
  std::size_t closest_wp_id = 0;
  double best_distance_sq = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < reference_path_.size(); ++i) {
    const auto & waypoint = reference_path_.get_waypoint(i);
    const auto dx = waypoint.x - x;
    const auto dy = waypoint.y - y;
    const auto dist_sq = dx * dx + dy * dy;
    if (dist_sq < best_distance_sq) {
      best_distance_sq = dist_sq;
      closest_wp_id = i;
    }
  }
  return closest_wp_id;
}

double BicycleModel::get_s_at_waypoint(std::size_t wp_id) const
{
  if (reference_path_.cumulative_lengths().empty()) {
    return 0.0;
  }
  return reference_path_.cumulative_lengths()[std::min(wp_id, reference_path_.cumulative_lengths().size() - 1)];
}

SpatialState BicycleModel::temporal_to_spatial(
  const Waypoint & reference_waypoint, const TemporalState & state) const
{
  const auto e_y = std::cos(reference_waypoint.psi) * (state.y - reference_waypoint.y) -
    std::sin(reference_waypoint.psi) * (state.x - reference_waypoint.x);
  const auto e_psi = normalize_angle(state.psi - reference_waypoint.psi);
  return {e_y, e_psi, 0.0};
}

std::array<double, 3> BicycleModel::linearization_offset(
  double v_ref, double, double delta_s) const
{
  if (std::abs(v_ref) <= 1.0e-12) {
    return {0.0, 0.0, 0.0};
  }
  return {0.0, 0.0, 1.0 / v_ref * delta_s};
}

std::array<double, 9> BicycleModel::linearization_a(
  double v_ref, double kappa_ref, double delta_s) const
{
  const std::array<double, 3> a1{1.0, delta_s, 0.0};
  const std::array<double, 3> a2{-kappa_ref * kappa_ref * delta_s, 1.0, 0.0};
  std::array<double, 3> a3{0.0, 0.0, 1.0};
  if (std::abs(v_ref) > 1.0e-12) {
    a3 = {-kappa_ref / v_ref * delta_s, 0.0, 1.0};
  }
  return {
    a1[0], a1[1], a1[2],
    a2[0], a2[1], a2[2],
    a3[0], a3[1], a3[2]};
}

std::array<double, 6> BicycleModel::linearization_b(
  double v_ref, double, double delta_s) const
{
  std::array<double, 2> b1{0.0, 0.0};
  std::array<double, 2> b2{0.0, delta_s};
  std::array<double, 2> b3{0.0, 0.0};
  if (std::abs(v_ref) > 1.0e-12) {
    b3 = {-1.0 / (v_ref * v_ref) * delta_s, 0.0};
  }
  return {
    b1[0], b1[1],
    b2[0], b2[1],
    b3[0], b3[1]};
}

ReferencePath & BicycleModel::reference_path()
{
  return reference_path_;
}

const ReferencePath & BicycleModel::reference_path() const
{
  return reference_path_;
}

const Waypoint & BicycleModel::current_waypoint() const
{
  return reference_path_.get_waypoint(wp_id_);
}

const TemporalState & BicycleModel::temporal_state() const
{
  return temporal_state_;
}

const SpatialState & BicycleModel::spatial_state() const
{
  return spatial_state_;
}

double BicycleModel::length() const
{
  return length_;
}

double BicycleModel::width() const
{
  return width_;
}

double BicycleModel::ts() const
{
  return ts_;
}

double BicycleModel::safety_margin() const
{
  return safety_margin_;
}

std::size_t BicycleModel::wp_id() const
{
  return wp_id_;
}

void BicycleModel::set_wp_id(std::size_t wp_id)
{
  wp_id_ = wp_id;
}

double BicycleModel::s() const
{
  return s_;
}

void BicycleModel::set_s(double s)
{
  s_ = s;
}

double BicycleModel::compute_safety_margin() const
{
  return width_ / std::sqrt(2.0);
}

OsqpMpcCore::OsqpMpcCore(const ControllerConfig & config)
: config_(config),
  model_(
    [&config]() {
      const auto config_path = std::filesystem::path(config.config_path);
      const auto package_share = config_path.parent_path().parent_path();
      const auto reference_csv = resolve_path(package_share, config.reference_path.csv_path);
      const auto map_yaml = resolve_path(package_share, config.map_yaml_path);
      auto [wp_x, wp_y] = load_ref_path_xy(reference_csv.string());
      ReferencePath reference_path(
        wp_x, wp_y,
        config.reference_path.resolution,
        config.reference_path.smoothing_distance,
        config.reference_path.max_width,
        config.reference_path.circular);
      reference_path.assign_bounds_from_map(map_yaml.string(), config.reference_path.max_width);
      reference_path.compute_speed_profile(
        config.mpc.a_min,
        config.mpc.a_max,
        kmh_to_mps(config.mpc.v_max_kmph),
        config.mpc.ay_max);
      return reference_path;
    }(),
    config.bicycle_model.length,
    config.bicycle_model.width,
    1.0 / config.mpc.control_rate_hz)
{
  v_max_mps_ = kmh_to_mps(config_.mpc.v_max_kmph);
  q_ = config_.mpc.q;
  r_ = config_.mpc.r;
  qn_ = config_.mpc.qn;
  ay_max_ = config_.mpc.ay_max;
  wp_id_offset_ = config_.mpc.wp_id_offset;
  input_kappa_min_ = -std::tan(deg_to_rad(config_.mpc.delta_max_deg)) / config_.bicycle_model.length;
  input_kappa_max_ = std::tan(deg_to_rad(config_.mpc.delta_max_deg)) / config_.bicycle_model.length;
  max_steering_rate_ = config_.mpc.steer_rate_max / config_.mpc.steering_tire_angle_gain_var;
  current_control_.assign(static_cast<std::size_t>(2 * config_.mpc.N), 0.0);
}

void OsqpMpcCore::update_states(double x, double y, double yaw)
{
  model_.update_states(x, y, yaw);
}

void OsqpMpcCore::update_v_max(double v_max_mps)
{
  v_max_mps_ = v_max_mps;
}

void OsqpMpcCore::update_ay_max(double ay_max)
{
  ay_max_ = ay_max;
}

void OsqpMpcCore::update_q(const std::array<double, 3> & q)
{
  q_ = q;
}

void OsqpMpcCore::update_r(const std::array<double, 2> & r)
{
  r_ = r;
}

void OsqpMpcCore::update_qn(const std::array<double, 3> & qn)
{
  qn_ = qn;
}

void OsqpMpcCore::update_wp_id_offset(int wp_id_offset)
{
  wp_id_offset_ = wp_id_offset;
}

void OsqpMpcCore::set_path_constraints(
  const std::vector<double> & upper_bounds, const std::vector<double> & lower_bounds,
  int rows, int cols)
{
  path_constraint_upper_bounds_ = upper_bounds;
  path_constraint_lower_bounds_ = lower_bounds;
  path_constraint_rows_ = rows;
  path_constraint_cols_ = cols;
}

void OsqpMpcCore::clear_path_constraints()
{
  path_constraint_upper_bounds_.clear();
  path_constraint_lower_bounds_.clear();
  path_constraint_rows_ = 0;
  path_constraint_cols_ = 0;
}

std::pair<double, double> adjust_path_constraint_bounds_for_safety_margin(
  double lower_bound, double upper_bound,
  double nominal_safety_margin, double requested_safety_margin)
{
  const auto safety_margin_diff = requested_safety_margin - nominal_safety_margin;
  upper_bound -= safety_margin_diff;
  lower_bound += safety_margin_diff;
  if (upper_bound < lower_bound) {
    return {0.0, 0.0};
  }
  return {lower_bound, upper_bound};
}

void OsqpMpcCore::set_reference_velocity_all(double v_ref_mps)
{
  model_.reference_path().set_v_ref_all(v_ref_mps);
}

void OsqpMpcCore::drive(double v, double delta)
{
  model_.drive(v, delta);
}

std::size_t OsqpMpcCore::current_waypoint_id() const
{
  return model_.wp_id();
}

std::size_t OsqpMpcCore::control_waypoint_id() const
{
  return model_.wp_id() + static_cast<std::size_t>(std::max(0, wp_id_offset_));
}

std::vector<double> OsqpMpcCore::curvature_prediction(std::size_t horizon) const
{
  std::vector<double> prediction(horizon, 0.0);
  if (current_control_.empty()) {
    return prediction;
  }

  for (std::size_t i = 0; i < horizon; ++i) {
    std::size_t delta_index = current_control_.size() - 1;
    if (i + 1 < horizon && (2 * (i + 1) + 1) < current_control_.size()) {
      delta_index = 2 * (i + 1) + 1;
    }
    prediction[i] = std::tan(current_control_[delta_index]) / model_.length();
  }
  return prediction;
}

std::pair<double, double> OsqpMpcCore::resolve_path_bounds(
  std::size_t stage, std::size_t base_wp_id) const
{
  if (path_constraint_rows_ > 0 && path_constraint_cols_ > 0)
  {
    const auto row_index = config_.reference_path.circular ?
      (base_wp_id + 1) % static_cast<std::size_t>(path_constraint_rows_) :
      std::min(base_wp_id + 1, static_cast<std::size_t>(path_constraint_rows_ - 1));
    const auto col_index = std::min(stage, static_cast<std::size_t>(path_constraint_cols_ - 1));
    const auto flat_index = row_index * static_cast<std::size_t>(path_constraint_cols_) + col_index;
    if (
      flat_index < path_constraint_upper_bounds_.size() &&
      flat_index < path_constraint_lower_bounds_.size())
    {
      auto upper = path_constraint_upper_bounds_[flat_index];
      auto lower = path_constraint_lower_bounds_[flat_index];
      if (upper < lower) {
        upper = 0.0;
        lower = 0.0;
      }
      return {lower, upper};
    }
  }

  const auto & waypoint = model_.reference_path().get_waypoint(base_wp_id + 1 + stage);
  auto upper = waypoint.ub;
  auto lower = waypoint.lb;
  if (upper < lower) {
    upper = 0.0;
    lower = 0.0;
  }
  return {lower, upper};
}

double OsqpMpcCore::clamp(double value, double min_value, double max_value) const
{
  return std::max(min_value, std::min(value, max_value));
}

SolveOutput OsqpMpcCore::solve(bool print_solver_stats)
{
  const auto t_total_start = std::chrono::steady_clock::now();
  SolveOutput output;
  model_.get_current_waypoint();
  output.waypoint_id = model_.wp_id();

  const auto path_size = model_.reference_path().size();
  const auto horizon = config_.reference_path.circular ?
    static_cast<std::size_t>(config_.mpc.N) :
    std::min<std::size_t>(config_.mpc.N, path_size > model_.wp_id() ? path_size - model_.wp_id() - 1 : 0);

  if (horizon == 0) {
    output.used_fallback = true;
    output.fallback_depth = ++infeasibility_counter_;
    ++fallback_count_total_;
    output.fallback_count_total = fallback_count_total_;
    return output;
  }

  const auto spatial_state = model_.temporal_to_spatial(model_.current_waypoint(), model_.temporal_state());
  const auto curvature_pred = curvature_prediction(horizon);
  const auto base_wp_id = model_.wp_id() + static_cast<std::size_t>(std::max(0, wp_id_offset_));

  const std::array<double, 6> relaxation_factors{1.0, 0.8, 0.6, 0.4, 0.2, 0.0};
  bool solved = false;
  Eigen::VectorXd solution;
  std::int64_t last_solver_status = 0;
  std::int64_t last_solver_exit_flag = 0;
  std::int64_t last_solver_iterations = 0;
  std::string last_solver_status_message;
  auto t_setup_end = t_total_start;
  auto t_solve_end = t_total_start;

  for (const auto relaxation : relaxation_factors) {
    const auto safety_margin = model_.safety_margin() * relaxation;

    std::vector<double> lower_bounds(horizon, 0.0);
    std::vector<double> upper_bounds(horizon, 0.0);
    for (std::size_t n = 0; n < horizon; ++n) {
      const auto [lower, upper] = resolve_path_bounds(n, base_wp_id);
      lower_bounds[n] = lower;
      upper_bounds[n] = upper;
      if (path_constraint_rows_ > 0 && path_constraint_cols_ > 0) {
        const auto adjusted_bounds = adjust_path_constraint_bounds_for_safety_margin(
          lower, upper, model_.safety_margin(), safety_margin);
        lower_bounds[n] = adjusted_bounds.first;
        upper_bounds[n] = adjusted_bounds.second;
      } else {
        lower_bounds[n] = lower + (model_.safety_margin() - safety_margin);
        upper_bounds[n] = upper - (model_.safety_margin() - safety_margin);
        if (upper_bounds[n] < lower_bounds[n]) {
          lower_bounds[n] = 0.0;
          upper_bounds[n] = 0.0;
        }
      }
    }

    const auto nx = 3;
    const auto nu = 2;
    const auto state_variables = static_cast<int>(nx * (horizon + 1));
    const auto input_variables = static_cast<int>(nu * horizon);
    const auto total_variables = state_variables + input_variables;
    const auto equality_constraints = state_variables;
    const auto basic_inequalities = total_variables;
    const auto rate_constraints = horizon > 0 ? static_cast<int>(horizon - 1) : 0;
    const auto total_constraints = equality_constraints + basic_inequalities + rate_constraints;

    std::vector<Triplet> p_triplets;
    p_triplets.reserve(state_variables + input_variables);
    for (std::size_t n = 0; n < horizon; ++n) {
      for (int i = 0; i < nx; ++i) {
        p_triplets.emplace_back(static_cast<int>(n * nx + i), static_cast<int>(n * nx + i), q_[i]);
      }
    }
    for (int i = 0; i < nx; ++i) {
      p_triplets.emplace_back(state_variables - nx + i, state_variables - nx + i, qn_[i]);
    }
    for (std::size_t n = 0; n < horizon; ++n) {
      for (int i = 0; i < nu; ++i) {
        const auto index = state_variables + static_cast<int>(n * nu + i);
        p_triplets.emplace_back(index, index, r_[i]);
      }
    }
    SparseMatrix P(total_variables, total_variables);
    P.setFromTriplets(p_triplets.begin(), p_triplets.end());

    Eigen::VectorXd q_vec(total_variables);
    q_vec.setZero();
    Eigen::VectorXd xr(state_variables);
    xr.setZero();
    Eigen::VectorXd ur(input_variables);
    ur.setZero();
    Eigen::VectorXd umin_dyn(input_variables);
    Eigen::VectorXd umax_dyn(input_variables);
    Eigen::VectorXd xmin_dyn(state_variables);
    Eigen::VectorXd xmax_dyn(state_variables);
    xmin_dyn.setConstant(-kOsqpInfinity);
    xmax_dyn.setConstant(kOsqpInfinity);

    std::vector<Triplet> a_triplets;
    a_triplets.reserve(
      equality_constraints * 3 + static_cast<int>(horizon) * 15 + basic_inequalities +
      rate_constraints * 2);

    for (int i = 0; i < state_variables; ++i) {
      a_triplets.emplace_back(i, i, -1.0);
    }

    Eigen::VectorXd leq(equality_constraints);
    Eigen::VectorXd ueq(equality_constraints);
    leq.setZero();
    ueq.setZero();
    leq[0] = -spatial_state.e_y;
    leq[1] = -spatial_state.e_psi;
    leq[2] = -spatial_state.t;
    ueq = leq;

    xmin_dyn[0] = spatial_state.e_y;
    xmax_dyn[0] = spatial_state.e_y;

    for (std::size_t n = 0; n < horizon; ++n) {
      const auto & current_waypoint = model_.reference_path().get_waypoint(base_wp_id + n);
      const auto & next_waypoint = model_.reference_path().get_waypoint(base_wp_id + n + 1);
      const auto delta_s = distance(current_waypoint, next_waypoint);
      const auto kappa_ref = current_waypoint.kappa;
      const auto v_ref = clamp(current_waypoint.v_ref, 0.0, v_max_mps_);

      const auto a = model_.linearization_a(v_ref, kappa_ref, delta_s);
      const auto b = model_.linearization_b(v_ref, kappa_ref, delta_s);
      const auto f = model_.linearization_offset(v_ref, kappa_ref, delta_s);

      for (int row = 0; row < nx; ++row) {
        for (int col = 0; col < nx; ++col) {
          const auto value = a[static_cast<std::size_t>(row * nx + col)];
          if (std::abs(value) > 1.0e-12) {
            a_triplets.emplace_back(
              static_cast<int>((n + 1) * nx + row),
              static_cast<int>(n * nx + col),
              value);
          }
        }
        for (int col = 0; col < nu; ++col) {
          const auto value = b[static_cast<std::size_t>(row * nu + col)];
          if (std::abs(value) > 1.0e-12) {
            a_triplets.emplace_back(
              static_cast<int>((n + 1) * nx + row),
              state_variables + static_cast<int>(n * nu + col),
              value);
          }
        }
      }

      Eigen::Vector3d uq;
      uq[0] = b[0] * v_ref + b[1] * kappa_ref - f[0];
      uq[1] = b[2] * v_ref + b[3] * kappa_ref - f[1];
      uq[2] = b[4] * v_ref + b[5] * kappa_ref - f[2];
      leq.segment(static_cast<int>((n + 1) * nx), nx) = uq;
      ueq.segment(static_cast<int>((n + 1) * nx), nx) = uq;

      ur[static_cast<int>(n * nu)] = v_ref;
      ur[static_cast<int>(n * nu + 1)] = kappa_ref;

      const double vmax_dyn = config_.mpc.use_max_kappa_pred ?
        std::sqrt(ay_max_ / (std::abs(*std::max_element(
          curvature_pred.begin() + static_cast<long>(n), curvature_pred.end(),
          [](double lhs, double rhs) { return std::abs(lhs) < std::abs(rhs); })) + 1.0e-12)) :
        std::sqrt(ay_max_ / (std::abs(curvature_pred[n]) + 1.0e-12));
      umin_dyn[static_cast<int>(n * nu)] = 0.0;
      umin_dyn[static_cast<int>(n * nu + 1)] = input_kappa_min_;
      umax_dyn[static_cast<int>(n * nu)] = std::min(vmax_dyn, v_max_mps_);
      umax_dyn[static_cast<int>(n * nu + 1)] = input_kappa_max_;

      xmin_dyn[static_cast<int>((n + 1) * nx)] = lower_bounds[n];
      xmax_dyn[static_cast<int>((n + 1) * nx)] = upper_bounds[n];
      xr[static_cast<int>((n + 1) * nx)] = 0.5 * (lower_bounds[n] + upper_bounds[n]);
    }

    for (int i = 0; i < state_variables; ++i) {
      q_vec[i] = -((i < state_variables - nx ? q_[i % nx] : qn_[i % nx]) * xr[i]);
    }
    for (int i = 0; i < input_variables; ++i) {
      q_vec[state_variables + i] = -(r_[i % nu] * ur[i]);
    }

    const auto identity_offset = equality_constraints;
    for (int i = 0; i < total_variables; ++i) {
      a_triplets.emplace_back(identity_offset + i, i, 1.0);
    }

    const auto rate_offset = equality_constraints + basic_inequalities;
    for (int i = 0; i < rate_constraints; ++i) {
      a_triplets.emplace_back(rate_offset + i, state_variables + i * nu + 1, -1.0);
      a_triplets.emplace_back(rate_offset + i, state_variables + (i + 1) * nu + 1, 1.0);
    }

    SparseMatrix A(total_constraints, total_variables);
    A.setFromTriplets(a_triplets.begin(), a_triplets.end());

    Eigen::VectorXd l(total_constraints);
    Eigen::VectorXd u(total_constraints);
    l.segment(0, equality_constraints) = leq;
    u.segment(0, equality_constraints) = ueq;
    l.segment(equality_constraints, state_variables) = xmin_dyn;
    u.segment(equality_constraints, state_variables) = xmax_dyn;
    l.segment(equality_constraints + state_variables, input_variables) = umin_dyn;
    u.segment(equality_constraints + state_variables, input_variables) = umax_dyn;

    const auto max_delta_change = max_steering_rate_ * model_.ts();
    for (int i = 0; i < rate_constraints; ++i) {
      l[rate_offset + i] = -max_delta_change;
      u[rate_offset + i] = max_delta_change;
    }

    if (!qp_dump_written_) {
      const auto * dump_dir_env = std::getenv("MPC_QP_DUMP_DIR");
      if (dump_dir_env != nullptr && dump_dir_env[0] != '\0') {
        const auto target_dir = std::filesystem::path(dump_dir_env) / "cpp_mpc";
        std::filesystem::create_directories(target_dir);
        write_csv_matrix(target_dir / "P.csv", Eigen::MatrixXd(P));
        write_csv_vector(target_dir / "q.csv", q_vec);
        write_csv_matrix(target_dir / "A.csv", Eigen::MatrixXd(A));
        write_csv_vector(target_dir / "l.csv", l);
        write_csv_vector(target_dir / "u.csv", u);

        std::ofstream meta(target_dir / "meta.txt");
        meta << "N=" << horizon << "\n";
        meta << "safety_margin=" << safety_margin << "\n";
        meta << "wp_id=" << model_.wp_id() << "\n";
        meta << "e_y=" << spatial_state.e_y << "\n";
        meta << "e_psi=" << spatial_state.e_psi << "\n";
        meta << "t=" << spatial_state.t << "\n";
        qp_dump_written_ = true;
      }
    }

    const auto t_setup_end = std::chrono::steady_clock::now();

    autoware::common::osqp::OSQPInterface solver(kOsqpTolerance, true);
    solver.updateEpsRel(kOsqpTolerance);
    solver.updateVerbose(print_solver_stats);
    const auto result = solver.optimize(
      P.toDense(),
      A.toDense(),
      to_std_vector(q_vec),
      to_std_vector(l),
      to_std_vector(u));
    const auto t_solve_end = std::chrono::steady_clock::now();

    const auto primal = std::get<0>(result);
    last_solver_status = solver.getStatus();
    last_solver_exit_flag = solver.getExitFlag();
    last_solver_iterations = solver.getTakenIter();
    last_solver_status_message = solver.getStatusMessage();
    if (solver.getStatus() <= 0 || static_cast<int>(primal.size()) != total_variables)
    {
      continue;
    }

    solution.resize(total_variables);
    for (int i = 0; i < total_variables; ++i) {
      solution[i] = primal[static_cast<std::size_t>(i)];
    }

    solved = true;
    break;
  }

  // --- 計測ログ出力 ---
  {
    static std::size_t diag_count = 0;
    ++diag_count;
    if (diag_count % 50 == 0) {
      const auto t_total_end = std::chrono::steady_clock::now();
      const auto setup_us = std::chrono::duration_cast<std::chrono::microseconds>(t_setup_end - t_total_start).count();
      const auto solve_us = std::chrono::duration_cast<std::chrono::microseconds>(t_solve_end - t_setup_end).count();
      const auto total_us = std::chrono::duration_cast<std::chrono::microseconds>(t_total_end - t_total_start).count();
      std::printf("[MPC_DIAG_CPP] setup=%.2fms solve=%.2fms total=%.2fms\n",
        setup_us / 1000.0, solve_us / 1000.0, total_us / 1000.0);
    }
  }

  if (solved) {
    std::vector<double> control_signals(static_cast<std::size_t>(2 * horizon), 0.0);
    const auto state_variables = static_cast<int>(3 * (horizon + 1));
    for (std::size_t i = 0; i < control_signals.size(); ++i) {
      control_signals[i] = solution[state_variables + static_cast<int>(i)];
    }

    for (std::size_t i = 1; i < control_signals.size(); i += 2) {
      control_signals[i] = std::atan(control_signals[i] * model_.length());
    }

    const auto v = control_signals[0];
    auto delta = control_signals[1];
    const auto max_delta_change = max_steering_rate_ * model_.ts();
    delta = clamp(delta, previous_steering_ - max_delta_change, previous_steering_ + max_delta_change);
    previous_steering_ = delta;

    current_control_.assign(static_cast<std::size_t>(2 * config_.mpc.N), 0.0);
    std::copy(control_signals.begin(), control_signals.end(), current_control_.begin());

    output.success = true;
    output.control = {v, delta};
    const auto max_delta_count = std::min<std::size_t>(horizon, static_cast<std::size_t>(current_control_.size() / 3));
    double max_delta = 0.0;
    for (std::size_t i = 0; i < max_delta_count; ++i) {
      max_delta = std::max(max_delta, std::abs(current_control_[2 * i + 1]));
    }
    output.max_delta_rad = max_delta;
    output.fallback_depth = infeasibility_counter_;
    output.fallback_count_total = fallback_count_total_;
    infeasibility_counter_ = 0;
    return output;
  }

  const auto id = static_cast<std::size_t>(2 * (infeasibility_counter_ + 1));
  if (id + 2 < current_control_.size()) {
    output.control = {current_control_[id], current_control_[id + 1]};
    output.max_delta_rad = std::abs(output.control[1]);
  }
  output.used_fallback = true;
  output.fallback_depth = ++infeasibility_counter_;
  ++fallback_count_total_;
  output.fallback_count_total = fallback_count_total_;
  output.solver_status = last_solver_status;
  output.solver_exit_flag = last_solver_exit_flag;
  output.solver_iterations = last_solver_iterations;
  output.solver_status_message = last_solver_status_message;
  return output;
}

}  // namespace multi_purpose_mpc_ros_cpp
