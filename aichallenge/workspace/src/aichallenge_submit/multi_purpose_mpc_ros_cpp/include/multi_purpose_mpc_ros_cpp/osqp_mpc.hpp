#ifndef MULTI_PURPOSE_MPC_ROS_CPP__OSQP_MPC_HPP_
#define MULTI_PURPOSE_MPC_ROS_CPP__OSQP_MPC_HPP_

#include "multi_purpose_mpc_ros_cpp/controller.hpp"

#include <array>
#include <cstdint>
#include <cstddef>
#include <string>
#include <vector>

namespace multi_purpose_mpc_ros_cpp
{

struct Waypoint
{
  double x{};
  double y{};
  double psi{};
  double kappa{};
  double v_ref{};
  double lb{};
  double ub{};
};

struct TemporalState
{
  double x{};
  double y{};
  double psi{};
};

struct SpatialState
{
  double e_y{};
  double e_psi{};
  double t{};
};

struct SolveOutput
{
  bool success{false};
  bool used_fallback{false};
  std::size_t waypoint_id{};
  std::array<double, 2> control{0.0, 0.0};
  double max_delta_rad{0.0};
  std::size_t fallback_depth{0};
  std::size_t fallback_count_total{0};
  std::int64_t solver_status{0};
  std::int64_t solver_exit_flag{0};
  std::int64_t solver_iterations{0};
  std::string solver_status_message;
};

std::pair<double, double> adjust_path_constraint_bounds_for_safety_margin(
  double lower_bound, double upper_bound,
  double nominal_safety_margin, double requested_safety_margin);

class ReferencePath
{
public:
  ReferencePath(
    const std::vector<double> & wp_x, const std::vector<double> & wp_y,
    double resolution, int smoothing_distance, double max_width, bool circular);

  const Waypoint & get_waypoint(std::size_t wp_id) const;
  std::size_t size() const;
  bool circular() const;
  const std::vector<double> & segment_lengths() const;
  const std::vector<double> & cumulative_lengths() const;
  void set_v_ref_all(double v_ref);
  void assign_bounds_from_map(const std::string & map_yaml_path, double max_width);
  bool compute_speed_profile(double a_min, double a_max, double v_max, double ay_max);

private:
  std::vector<Waypoint> construct_path(
    const std::vector<double> & wp_x, const std::vector<double> & wp_y) const;
  std::vector<Waypoint> construct_waypoints(
    const std::vector<std::pair<double, double>> & waypoint_coordinates) const;
  void compute_lengths();
  void assign_default_bounds(double max_width);

  double resolution_{};
  int smoothing_distance_{};
  bool circular_{false};
  double eps_{1.0e-12};
  std::vector<Waypoint> waypoints_;
  std::vector<double> segment_lengths_;
  std::vector<double> cumulative_lengths_;
};

class BicycleModel
{
public:
  BicycleModel(ReferencePath reference_path, double length, double width, double ts);

  void update_states(double x, double y, double psi);
  void drive(double v, double delta);
  void get_current_waypoint();
  std::size_t get_closest_waypoint(double x, double y) const;
  double get_s_at_waypoint(std::size_t wp_id) const;
  SpatialState temporal_to_spatial(const Waypoint & reference_waypoint, const TemporalState & state) const;
  std::array<double, 3> linearization_offset(double v_ref, double kappa_ref, double delta_s) const;
  std::array<double, 9> linearization_a(double v_ref, double kappa_ref, double delta_s) const;
  std::array<double, 6> linearization_b(double v_ref, double kappa_ref, double delta_s) const;

  ReferencePath & reference_path();
  const ReferencePath & reference_path() const;
  const Waypoint & current_waypoint() const;
  const TemporalState & temporal_state() const;
  const SpatialState & spatial_state() const;
  double length() const;
  double width() const;
  double ts() const;
  double safety_margin() const;
  std::size_t wp_id() const;
  void set_wp_id(std::size_t wp_id);
  double s() const;
  void set_s(double s);

private:
  double compute_safety_margin() const;

  ReferencePath reference_path_;
  double length_{};
  double width_{};
  double ts_{};
  double safety_margin_{};
  double s_{0.0};
  std::size_t wp_id_{0};
  TemporalState temporal_state_{};
  SpatialState spatial_state_{};
};

class OsqpMpcCore
{
public:
  explicit OsqpMpcCore(const ControllerConfig & config);

  void update_states(double x, double y, double yaw);
  void update_v_max(double v_max_mps);
  void update_ay_max(double ay_max);
  void update_q(const std::array<double, 3> & q);
  void update_r(const std::array<double, 2> & r);
  void update_qn(const std::array<double, 3> & qn);
  void update_wp_id_offset(int wp_id_offset);
  void set_path_constraints(
    const std::vector<double> & upper_bounds, const std::vector<double> & lower_bounds,
    int rows, int cols);
  void clear_path_constraints();
  void set_reference_velocity_all(double v_ref_mps);
  void drive(double v, double delta);
  SolveOutput solve(bool print_solver_stats);

  std::size_t current_waypoint_id() const;
  std::size_t control_waypoint_id() const;

private:
  std::vector<double> curvature_prediction(std::size_t horizon) const;
  std::pair<double, double> resolve_path_bounds(std::size_t stage, std::size_t base_wp_id) const;
  double clamp(double value, double min_value, double max_value) const;

  ControllerConfig config_;
  BicycleModel model_;
  double v_max_mps_{};
  std::array<double, 3> q_;
  std::array<double, 2> r_;
  std::array<double, 3> qn_;
  double ay_max_{};
  int wp_id_offset_{};
  double input_kappa_min_{};
  double input_kappa_max_{};
  double max_steering_rate_{};
  double previous_steering_{0.0};
  std::vector<double> current_control_;
  std::size_t infeasibility_counter_{0};
  std::size_t fallback_count_total_{0};

  std::vector<double> path_constraint_upper_bounds_;
  std::vector<double> path_constraint_lower_bounds_;
  int path_constraint_rows_{0};
  int path_constraint_cols_{0};
  bool qp_dump_written_{false};
};

}  // namespace multi_purpose_mpc_ros_cpp

#endif  // MULTI_PURPOSE_MPC_ROS_CPP__OSQP_MPC_HPP_
