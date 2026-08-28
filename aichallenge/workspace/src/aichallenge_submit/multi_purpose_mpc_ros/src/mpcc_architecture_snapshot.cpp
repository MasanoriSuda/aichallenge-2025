#include "multi_purpose_mpc_ros/mpcc_architecture_snapshot.hpp"

#include "multi_purpose_mpc_ros/mpcc_execution_contract.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_shadow.hpp"

#include <yaml-cpp/yaml.h>

#include <Eigen/Sparse>

#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <set>
#include <sstream>
#include <system_error>
#include <type_traits>
#include <vector>

namespace multi_purpose_mpc_ros::mpcc_architecture_snapshot
{
namespace
{

namespace contract = mpcc_execution_contract;
namespace problem = mpcc_rate_resolved_problem;
namespace shadow = mpcc_rate_resolved_shadow;

std::mutex record_mutex;
std::set<std::string> recorded_failure_keys;

bool is_overtake(const contract::ControlIntent intent) noexcept
{
  return intent == contract::ControlIntent::ShiftOut ||
         intent == contract::ControlIntent::Pass ||
         intent == contract::ControlIntent::Return;
}

template<typename LowerDerived, typename UpperDerived>
bool valid_semantic_bounds(
  const Eigen::MatrixBase<LowerDerived> & lower,
  const Eigen::MatrixBase<UpperDerived> & upper) noexcept
{
  if (lower.size() != upper.size()) {
    return false;
  }
  for (Eigen::Index index = 0; index < lower.size(); ++index) {
    if (
      std::isnan(lower(index)) || std::isnan(upper(index)) ||
      lower(index) == std::numeric_limits<double>::infinity() ||
      upper(index) == -std::numeric_limits<double>::infinity() ||
      lower(index) > upper(index))
    {
      return false;
    }
  }
  return true;
}

std::string safe_component(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(), [](const unsigned char c) {
      return std::isalnum(c) != 0 ? static_cast<char>(std::tolower(c)) : '-';
    });
  while (value.find("--") != std::string::npos) {
    value.replace(value.find("--"), 2U, "-");
  }
  if (!value.empty() && value.front() == '-') {
    value.erase(value.begin());
  }
  if (!value.empty() && value.back() == '-') {
    value.pop_back();
  }
  return value.empty() ? "unknown" : value;
}

YAML::Node vector_node(const Eigen::VectorXd & value)
{
  YAML::Node node(YAML::NodeType::Sequence);
  for (Eigen::Index index = 0; index < value.size(); ++index) {
    node.push_back(value[index]);
  }
  return node;
}

template<typename Derived>
YAML::Node fixed_vector_node(const Eigen::MatrixBase<Derived> & value)
{
  YAML::Node node(YAML::NodeType::Sequence);
  for (Eigen::Index index = 0; index < value.size(); ++index) {
    node.push_back(value(index));
  }
  return node;
}

template<typename Derived>
YAML::Node dense_matrix_node(const Eigen::MatrixBase<Derived> & value)
{
  YAML::Node node;
  node["rows"] = value.rows();
  node["columns"] = value.cols();
  YAML::Node entries(YAML::NodeType::Sequence);
  for (Eigen::Index row = 0; row < value.rows(); ++row) {
    YAML::Node row_node(YAML::NodeType::Sequence);
    for (Eigen::Index column = 0; column < value.cols(); ++column) {
      row_node.push_back(value(row, column));
    }
    entries.push_back(row_node);
  }
  node["values"] = entries;
  return node;
}

YAML::Node sparse_matrix_node(Eigen::SparseMatrix<double> value)
{
  value.makeCompressed();
  YAML::Node node;
  node["rows"] = value.rows();
  node["columns"] = value.cols();
  YAML::Node triplets(YAML::NodeType::Sequence);
  for (int outer = 0; outer < value.outerSize(); ++outer) {
    for (Eigen::SparseMatrix<double>::InnerIterator entry(value, outer);
      entry; ++entry)
    {
      YAML::Node item(YAML::NodeType::Sequence);
      item.push_back(entry.row());
      item.push_back(entry.col());
      item.push_back(entry.value());
      triplets.push_back(item);
    }
  }
  node["triplets"] = triplets;
  return node;
}

YAML::Node std_vector_node(const std::vector<double> & values)
{
  YAML::Node node(YAML::NodeType::Sequence);
  for (const double value : values) {
    node.push_back(value);
  }
  return node;
}

YAML::Node assembly_request_node(const problem::AssemblyRequest & request)
{
  YAML::Node node;
  node["horizon_steps"] = request.horizon_steps;
  node["initial_state"] = fixed_vector_node(request.initial_state);
  YAML::Node linearizations(YAML::NodeType::Sequence);
  for (const auto & linearization : request.linearizations) {
    YAML::Node item;
    item["state_matrix"] = dense_matrix_node(linearization.state_matrix);
    item["input_matrix"] = dense_matrix_node(linearization.input_matrix);
    item["equality_offset"] = fixed_vector_node(
      linearization.equality_offset);
    item["stage_dt_sec"] = linearization.stage_dt_sec;
    linearizations.push_back(item);
  }
  node["linearizations"] = linearizations;
  node["state_reference"] = vector_node(request.state_reference);
  node["state_lower"] = vector_node(request.state_lower);
  node["state_upper"] = vector_node(request.state_upper);
  node["state_weight"] = vector_node(request.state_weight);
  node["input_reference"] = vector_node(request.input_reference);
  node["input_lower"] = vector_node(request.input_lower);
  node["input_upper"] = vector_node(request.input_upper);
  node["input_weight"] = vector_node(request.input_weight);
  node["additional_linear_cost"] = vector_node(
    request.additional_linear_cost);
  node["previous_input"] = fixed_vector_node(request.previous_input);
  node["input_delta_weight"] = fixed_vector_node(
    request.input_delta_weight);
  if (request.steering_rate_prefix_bounds.has_value()) {
    YAML::Node bounds;
    bounds["minimum_cumulative_delta_rad"] =
      request.steering_rate_prefix_bounds->minimum_cumulative_delta_rad;
    bounds["maximum_cumulative_delta_rad"] =
      request.steering_rate_prefix_bounds->maximum_cumulative_delta_rad;
    node["steering_rate_prefix_bounds"] = bounds;
  }
  if (request.progress_aligned_wall_constraints.has_value()) {
    YAML::Node wall;
    wall["lower_slope"] = std_vector_node(
      request.progress_aligned_wall_constraints->lower_slope);
    wall["lower_intercept"] = std_vector_node(
      request.progress_aligned_wall_constraints->lower_intercept);
    wall["upper_slope"] = std_vector_node(
      request.progress_aligned_wall_constraints->upper_slope);
    wall["upper_intercept"] = std_vector_node(
      request.progress_aligned_wall_constraints->upper_intercept);
    node["progress_aligned_wall_constraints"] = wall;
  }
  YAML::Node swept(YAML::NodeType::Sequence);
  for (const auto & constraint : request.swept_lateral_wall_constraints) {
    YAML::Node item;
    item["transition_stage"] = constraint.transition_stage;
    item["destination_ratio"] = constraint.destination_ratio;
    item["lower_m"] = constraint.lower_m;
    item["upper_m"] = constraint.upper_m;
    swept.push_back(item);
  }
  node["swept_lateral_wall_constraints"] = swept;
  YAML::Node obstacles(YAML::NodeType::Sequence);
  for (const auto & constraint : request.dynamic_obstacle_constraints) {
    YAML::Node item;
    item["state_stage"] = constraint.state_stage;
    item["axis"] = constraint.axis ==
      problem::DynamicObstacleConstraintAxis::Lateral ?
      "lateral" : "effective-progress";
    item["lower"] = constraint.lower;
    item["upper"] = constraint.upper;
    obstacles.push_back(item);
  }
  node["dynamic_obstacle_constraints"] = obstacles;
  return node;
}

YAML::Node semantic_request_node(
  const mpcc_rate_resolved_adapter::Request & request)
{
  YAML::Node node;
  node["horizon_steps"] = request.horizon_steps;
  node["initial_state"] = fixed_vector_node(request.initial_state);
  node["current_steering_rad"] = request.current_steering_rad;
  node["current_response_steering_rad"] =
    request.current_response_steering_rad;
  node["wheelbase_m"] = request.wheelbase_m;
  node["yaw_response_gain"] = request.yaw_response_gain;
  node["yaw_response_time_constant_sec"] =
    request.yaw_response_time_constant_sec;
  node["maximum_abs_steering_rad"] = request.maximum_abs_steering_rad;
  node["maximum_abs_steering_rate_radps"] =
    request.maximum_abs_steering_rate_radps;
  node["minimum_frenet_denominator"] = request.minimum_frenet_denominator;
  node["minimum_stage_dt_sec"] = request.minimum_stage_dt_sec;
  node["maximum_stage_dt_sec"] = request.maximum_stage_dt_sec;
  node["previous_input"] = fixed_vector_node(request.previous_input);
  node["input_delta_weight"] = fixed_vector_node(
    request.input_delta_weight);
  YAML::Node states(YAML::NodeType::Sequence);
  for (const auto & state : request.states) {
    YAML::Node item;
    item["reference"] = fixed_vector_node(state.reference);
    item["lower"] = fixed_vector_node(state.lower);
    item["upper"] = fixed_vector_node(state.upper);
    item["weight"] = fixed_vector_node(state.weight);
    item["linear_cost"] = fixed_vector_node(state.linear_cost);
    states.push_back(item);
  }
  node["states"] = states;
  YAML::Node inputs(YAML::NodeType::Sequence);
  for (const auto & input : request.inputs) {
    YAML::Node item;
    item["reference"] = fixed_vector_node(input.reference);
    item["lower"] = fixed_vector_node(input.lower);
    item["upper"] = fixed_vector_node(input.upper);
    item["weight"] = fixed_vector_node(input.weight);
    item["linear_cost"] = fixed_vector_node(input.linear_cost);
    item["path_curvature_radpm"] = input.path_curvature_radpm;
    item["stage_dt_sec"] = input.stage_dt_sec;
    inputs.push_back(item);
  }
  node["inputs"] = inputs;
  return node;
}

YAML::Node exact_problem_node(const problem::Problem & value)
{
  YAML::Node node;
  node["horizon_steps"] = value.horizon_steps;
  node["linear_cost"] = vector_node(value.linear_cost);
  node["lower_bound"] = vector_node(value.lower_bound);
  node["upper_bound"] = vector_node(value.upper_bound);
  node["quadratic_cost"] = sparse_matrix_node(value.quadratic_cost);
  node["constraints"] = sparse_matrix_node(value.constraints);
  node["variable_scaling"] = vector_node(
    value.variable_scaling.physical_units_per_solver_unit);
  return node;
}

YAML::Node warm_start_node(
  const std::optional<persistent_osqp::WarmStart> & value)
{
  YAML::Node node;
  node["available"] = value.has_value();
  if (value.has_value()) {
    node["primal"] = vector_node(value->primal);
    node["dual"] = vector_node(value->dual);
  }
  return node;
}

YAML::Node outcome_node(const persistent_osqp::SolveOutcome & outcome)
{
  YAML::Node node;
  node["result_available"] = outcome.result.has_value();
  node["failure_detail"] = outcome.failure_detail;
  YAML::Node telemetry;
  telemetry["setup_performed"] = outcome.telemetry.setup_performed;
  telemetry["update_performed"] = outcome.telemetry.update_performed;
  telemetry["structural_rebuild"] = outcome.telemetry.structural_rebuild;
  telemetry["update_rebuild"] = outcome.telemetry.update_rebuild;
  telemetry["warm_start_applied"] = outcome.telemetry.warm_start_applied;
  telemetry["warm_start_rejected"] = outcome.telemetry.warm_start_rejected;
  telemetry["cold_reset_after_failure"] =
    outcome.telemetry.cold_reset_after_failure;
  telemetry["maximum_iterations_reached"] =
    outcome.telemetry.maximum_iterations_reached;
  telemetry["setup_ms"] = outcome.telemetry.setup_ms;
  telemetry["update_ms"] = outcome.telemetry.update_ms;
  telemetry["warm_start_ms"] = outcome.telemetry.warm_start_ms;
  telemetry["solve_ms"] = outcome.telemetry.solve_ms;
  telemetry["total_ms"] = outcome.telemetry.total_ms;
  telemetry["iterations"] = outcome.telemetry.iterations;
  telemetry["status"] = outcome.telemetry.status;
  telemetry["objective_value"] = outcome.telemetry.objective_value;
  telemetry["primal_residual"] = outcome.telemetry.primal_residual;
  telemetry["dual_residual"] = outcome.telemetry.dual_residual;
  telemetry["rho_updates"] = outcome.telemetry.rho_updates;
  telemetry["rho_estimate"] = outcome.telemetry.rho_estimate;
  telemetry["absolute_tolerance"] = outcome.telemetry.absolute_tolerance;
  telemetry["relative_tolerance"] = outcome.telemetry.relative_tolerance;
  telemetry["scaling_iterations"] = outcome.telemetry.scaling_iterations;
  telemetry["scaled_termination"] = outcome.telemetry.scaled_termination;
  telemetry["row_tolerance_preconditioned"] =
    outcome.telemetry.row_tolerance_preconditioned;
  telemetry["variable_coordinate_scaled"] =
    outcome.telemetry.variable_coordinate_scaled;
  telemetry["minimum_variable_scale"] =
    outcome.telemetry.minimum_variable_scale;
  telemetry["maximum_variable_scale"] =
    outcome.telemetry.maximum_variable_scale;
  telemetry["maximum_row_scale"] = outcome.telemetry.maximum_row_scale;
  telemetry["physical_constraint_scale"] =
    outcome.telemetry.physical_constraint_scale;
  telemetry["physical_global_tolerance"] =
    outcome.telemetry.physical_global_tolerance;
  node["telemetry"] = telemetry;
  if (outcome.constraint_failure.has_value()) {
    const auto & failure = outcome.constraint_failure.value();
    YAML::Node diagnostic;
    diagnostic["row"] = failure.row;
    diagnostic["value"] = failure.value;
    diagnostic["projected"] = failure.projected;
    diagnostic["lower_bound"] = failure.lower_bound;
    diagnostic["upper_bound"] = failure.upper_bound;
    diagnostic["violation"] = failure.violation;
    diagnostic["tolerance"] = failure.tolerance;
    diagnostic["normalized_violation"] = failure.normalized_violation;
    node["constraint_failure"] = diagnostic;
  }
  if (outcome.result.has_value()) {
    YAML::Node result;
    result["primal"] = vector_node(outcome.result->primal);
    result["dual"] = vector_node(outcome.result->dual);
    result["status"] = outcome.result->status;
    result["maximum_constraint_violation"] =
      outcome.result->maximum_constraint_violation;
    result["maximum_normalized_constraint_violation"] =
      outcome.result->maximum_normalized_constraint_violation;
    result["maximum_normalized_constraint_row"] =
      outcome.result->maximum_normalized_constraint_row;
    node["result"] = result;
  }
  return node;
}

YAML::Node source_node(
  const shadow::Snapshot & source, const std::string & wall_grid_file)
{
  YAML::Node node;
  const auto & identity = source.identity;
  const auto & context = identity.source_context;
  node["sequence"] = identity.sequence;
  node["snapshot_sec"] = identity.snapshot_sec;
  node["control_prediction_origin_sec"] =
    source.control_prediction_origin_sec;
  node["course_progress_origin_m"] = source.course_progress_origin_m;
  node["execution_prefix_steps"] = source.execution_prefix_steps;
  node["publication_interval_sec"] = source.publication_interval_sec;
  YAML::Node problem_context;
  problem_context["decision_id"] = context.decision_id;
  problem_context["intent"] = contract::to_string(context.intent);
  problem_context["intent_generation"] = context.intent_generation;
  problem_context["observation_generation"] = context.observation_generation;
  problem_context["stage_geometry_id"] = context.stage_geometry_id;
  problem_context["target_obstacle_generation"] =
    context.target_obstacle_generation;
  problem_context["target_id"] = context.target_id;
  problem_context["execution_side_sign"] = context.execution_side_sign;
  problem_context["horizon_steps"] = context.horizon_steps;
  problem_context["formulation"] = contract::to_string(context.formulation);
  problem_context["state_schema_id"] = context.state_schema_id;
  problem_context["input_schema_id"] = context.input_schema_id;
  problem_context["bounds_schema_id"] = context.bounds_schema_id;
  problem_context["cost_schema_id"] = context.cost_schema_id;
  problem_context["fingerprint"] = context.fingerprint;
  node["problem_context"] = problem_context;
  node["semantic_request"] = semantic_request_node(source.request);
  node["nominal_path_distance_m"] = std_vector_node(
    source.nominal_path_distance_m);
  node["progress_aligned_wall_refinement_active"] =
    source.progress_aligned_wall_refinement_active;
  node["wall_reference_progress_m"] = std_vector_node(
    source.wall_reference_progress_m);
  node["wall_lower_m"] = std_vector_node(source.wall_lower_m);
  node["wall_upper_m"] = std_vector_node(source.wall_upper_m);
  node["progress_wall_profile_diagnostic"] =
    source.progress_wall_profile_diagnostic;
  node["dynamic_obstacle_refinement_active"] =
    source.dynamic_obstacle_refinement_active;
  node["dynamic_obstacle_pass_side_sign"] =
    source.dynamic_obstacle_pass_side_sign;
  YAML::Node obstacle_stages(YAML::NodeType::Sequence);
  for (const auto & stage : source.dynamic_obstacle_stages) {
    YAML::Node item;
    item["valid"] = stage.valid;
    item["target_progress_m"] = stage.target_progress_m;
    item["target_lateral_m"] = stage.target_lateral_m;
    item["longitudinal_overlap_m"] = stage.longitudinal_overlap_m;
    item["lateral_center_separation_m"] =
      stage.lateral_center_separation_m;
    obstacle_stages.push_back(item);
  }
  node["dynamic_obstacle_stages"] = obstacle_stages;
  node["physical_wall_refinement_active"] =
    source.physical_wall_refinement_active;
  YAML::Node footprint;
  footprint["front_extent_m"] = source.wall_footprint.front_extent_m;
  footprint["rear_extent_m"] = source.wall_footprint.rear_extent_m;
  footprint["left_extent_m"] = source.wall_footprint.left_extent_m;
  footprint["right_extent_m"] = source.wall_footprint.right_extent_m;
  footprint["margin_m"] = source.wall_footprint.margin_m;
  node["wall_footprint"] = footprint;
  YAML::Node course_knots(YAML::NodeType::Sequence);
  for (const auto & knot : source.wall_course_frame_knots) {
    YAML::Node item;
    item["progress_m"] = knot.progress_m;
    item["x_m"] = knot.x_m;
    item["y_m"] = knot.y_m;
    item["heading_rad"] = knot.heading_rad;
    item["waypoint"] = knot.waypoint;
    course_knots.push_back(item);
  }
  node["wall_course_frame_knots"] = course_knots;
  node["wall_lateral_sample_step_m"] = source.wall_lateral_sample_step_m;
  node["wall_heading_bucket_width_rad"] =
    source.wall_heading_bucket_width_rad;
  node["wall_translation_bucket_width_m"] =
    source.wall_translation_bucket_width_m;
  node["wall_boundary_guard_m"] = source.wall_boundary_guard_m;
  YAML::Node grid;
  grid["available"] = source.wall_grid != nullptr;
  if (source.wall_grid != nullptr) {
    grid["width"] = source.wall_grid->width;
    grid["height"] = source.wall_grid->height;
    grid["resolution_m"] = source.wall_grid->resolution_m;
    grid["origin_x_m"] = source.wall_grid->origin_x_m;
    grid["origin_y_m"] = source.wall_grid->origin_y_m;
    grid["y_axis"] = source.wall_grid->y_axis ==
      recovery_footprint::YAxisConvention::RowZeroAtMinimumY ?
      "row-zero-at-minimum-y" : "row-zero-at-maximum-y";
    grid["cell_count"] = source.wall_grid->cells.size();
    grid["payload"] = wall_grid_file;
  }
  node["wall_grid"] = grid;
  YAML::Node replay_world;
  replay_world["available"] = source.replay_world.has_value();
  if (source.replay_world.has_value()) {
    const auto & world = source.replay_world.value();
    replay_world["observation_generation"] = world.observation_generation;
    replay_world["observed_sec"] = world.observed_sec;
    replay_world["current"] = world.current;
    YAML::Node current_pose;
    current_pose["x_m"] = world.current_pose.x_m;
    current_pose["y_m"] = world.current_pose.y_m;
    current_pose["yaw_rad"] = world.current_pose.yaw_rad;
    replay_world["current_pose"] = current_pose;
    YAML::Node control_prefix(YAML::NodeType::Sequence);
    for (const auto & pose : world.control_prefix) {
      YAML::Node item;
      item["x_m"] = pose.x_m;
      item["y_m"] = pose.y_m;
      item["yaw_rad"] = pose.yaw_rad;
      control_prefix.push_back(item);
    }
    replay_world["control_prefix"] = control_prefix;
    replay_world["control_prefix_elapsed_sec"] =
      std_vector_node(world.control_prefix_elapsed_sec);
    YAML::Node physical_footprint;
    physical_footprint["front_extent_m"] =
      world.physical_footprint.front_extent_m;
    physical_footprint["rear_extent_m"] =
      world.physical_footprint.rear_extent_m;
    physical_footprint["left_extent_m"] =
      world.physical_footprint.left_extent_m;
    physical_footprint["right_extent_m"] =
      world.physical_footprint.right_extent_m;
    physical_footprint["margin_m"] = world.physical_footprint.margin_m;
    replay_world["physical_footprint"] = physical_footprint;
    replay_world["wall_grid_fingerprint"] = world.wall_grid_fingerprint;
    replay_world["hard_wall_clearance_m"] = world.hard_wall_clearance_m;
    replay_world["bound_tolerance_m"] = world.bound_tolerance_m;
    replay_world["swept_step_m"] = world.swept_step_m;
    auto obstacles = world.obstacles;
    std::sort(
      obstacles.begin(), obstacles.end(),
      [](const shadow::ReplayDynamicObstacle & lhs,
        const shadow::ReplayDynamicObstacle & rhs) {
        return lhs.id < rhs.id;
      });
    YAML::Node obstacle_nodes(YAML::NodeType::Sequence);
    for (const auto & obstacle : obstacles) {
      YAML::Node item;
      item["id"] = obstacle.id;
      item["x_m"] = obstacle.x_m;
      item["y_m"] = obstacle.y_m;
      item["velocity_x_mps"] = obstacle.velocity_x_mps;
      item["velocity_y_mps"] = obstacle.velocity_y_mps;
      item["acceleration_x_mps2"] = obstacle.acceleration_x_mps2;
      item["acceleration_y_mps2"] = obstacle.acceleration_y_mps2;
      item["covariance_x_m2"] = obstacle.covariance_x_m2;
      item["covariance_y_m2"] = obstacle.covariance_y_m2;
      item["radius_m"] = obstacle.radius_m;
      item["observation_generation"] = obstacle.observation_generation;
      obstacle_nodes.push_back(item);
    }
    replay_world["obstacles"] = obstacle_nodes;
  }
  node["replay_world"] = replay_world;
  return node;
}

std::optional<Eigen::VectorXd> load_vector(const YAML::Node & node)
{
  if (!node || !node.IsSequence()) {
    return std::nullopt;
  }
  Eigen::VectorXd value(static_cast<Eigen::Index>(node.size()));
  for (std::size_t index = 0U; index < node.size(); ++index) {
    value[static_cast<Eigen::Index>(index)] = node[index].as<double>();
  }
  return value;
}

std::optional<Eigen::SparseMatrix<double>> load_sparse(
  const YAML::Node & node)
{
  if (!node || !node["rows"] || !node["columns"] ||
    !node["triplets"] || !node["triplets"].IsSequence())
  {
    return std::nullopt;
  }
  const int rows = node["rows"].as<int>();
  const int columns = node["columns"].as<int>();
  if (rows < 0 || columns < 0) {
    return std::nullopt;
  }
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(node["triplets"].size());
  for (const auto & item : node["triplets"]) {
    if (!item.IsSequence() || item.size() != 3U) {
      return std::nullopt;
    }
    const int row = item[0].as<int>();
    const int column = item[1].as<int>();
    const double value = item[2].as<double>();
    if (row < 0 || row >= rows || column < 0 || column >= columns) {
      return std::nullopt;
    }
    triplets.emplace_back(row, column, value);
  }
  Eigen::SparseMatrix<double> matrix(rows, columns);
  matrix.setFromTriplets(triplets.begin(), triplets.end());
  matrix.makeCompressed();
  return matrix;
}

template<int Size>
bool load_fixed_vector(
  const YAML::Node & node, Eigen::Matrix<double, Size, 1> & value)
{
  if (!node || !node.IsSequence() || node.size() != static_cast<std::size_t>(Size)) {
    return false;
  }
  for (int index = 0; index < Size; ++index) {
    value[index] = node[static_cast<std::size_t>(index)].as<double>();
  }
  return value.allFinite();
}

template<int Size>
bool load_bound_vector(
  const YAML::Node & node, Eigen::Matrix<double, Size, 1> & value)
{
  if (!node || !node.IsSequence() || node.size() != static_cast<std::size_t>(Size)) {
    return false;
  }
  for (int index = 0; index < Size; ++index) {
    value[index] = node[static_cast<std::size_t>(index)].as<double>();
    if (std::isnan(value[index])) {
      return false;
    }
  }
  return true;
}

std::optional<std::vector<double>> load_std_vector(const YAML::Node & node)
{
  if (!node || !node.IsSequence()) {
    return std::nullopt;
  }
  std::vector<double> values;
  values.reserve(node.size());
  for (const auto & item : node) {
    const double value = item.as<double>();
    if (!std::isfinite(value)) {
      return std::nullopt;
    }
    values.push_back(value);
  }
  return values;
}

std::optional<contract::ControlIntent> parse_intent(const std::string & value)
{
  using Intent = contract::ControlIntent;
  if (value == "track") return Intent::Track;
  if (value == "cruise") return Intent::Cruise;
  if (value == "follow") return Intent::Follow;
  if (value == "hold") return Intent::Hold;
  if (value == "stop") return Intent::Stop;
  if (value == "shiftout") return Intent::ShiftOut;
  if (value == "pass") return Intent::Pass;
  if (value == "return") return Intent::Return;
  if (value == "rejoin") return Intent::Rejoin;
  return std::nullopt;
}

std::optional<contract::Formulation> parse_formulation(
  const std::string & value)
{
  using Formulation = contract::Formulation;
  if (value == "velocity-steering-yaw-response-progress-7state") {
    return Formulation::VelocitySteeringYawResponseProgress7State;
  }
  if (value == "solver-derived-bypass") {
    return Formulation::SolverDerivedBypass;
  }
  return std::nullopt;
}

std::optional<mpcc_rate_resolved_adapter::Request> load_semantic_request(
  const YAML::Node & node)
{
  namespace adapter = mpcc_rate_resolved_adapter;
  if (!node || !node.IsMap()) {
    return std::nullopt;
  }
  adapter::Request request;
  request.horizon_steps = node["horizon_steps"].as<int>();
  if (
    request.horizon_steps <= 0 ||
    !load_fixed_vector(node["initial_state"], request.initial_state) ||
    !load_fixed_vector(node["previous_input"], request.previous_input) ||
    !load_fixed_vector(node["input_delta_weight"], request.input_delta_weight))
  {
    return std::nullopt;
  }
  request.current_steering_rad = node["current_steering_rad"].as<double>();
  request.current_response_steering_rad =
    node["current_response_steering_rad"].as<double>();
  request.wheelbase_m = node["wheelbase_m"].as<double>();
  request.yaw_response_gain = node["yaw_response_gain"].as<double>();
  request.yaw_response_time_constant_sec =
    node["yaw_response_time_constant_sec"].as<double>();
  request.maximum_abs_steering_rad =
    node["maximum_abs_steering_rad"].as<double>();
  request.maximum_abs_steering_rate_radps =
    node["maximum_abs_steering_rate_radps"].as<double>();
  request.minimum_frenet_denominator =
    node["minimum_frenet_denominator"].as<double>();
  request.minimum_stage_dt_sec = node["minimum_stage_dt_sec"].as<double>();
  request.maximum_stage_dt_sec = node["maximum_stage_dt_sec"].as<double>();
  const auto states = node["states"];
  const auto inputs = node["inputs"];
  if (
    !states || !states.IsSequence() || !inputs || !inputs.IsSequence() ||
    states.size() != static_cast<std::size_t>(request.horizon_steps + 1) ||
    inputs.size() != static_cast<std::size_t>(request.horizon_steps))
  {
    return std::nullopt;
  }
  request.states.reserve(states.size());
  for (const auto & item : states) {
    adapter::StateStage stage;
    if (
      !load_fixed_vector(item["reference"], stage.reference) ||
      !load_bound_vector(item["lower"], stage.lower) ||
      !load_bound_vector(item["upper"], stage.upper) ||
      !load_fixed_vector(item["weight"], stage.weight) ||
      !load_fixed_vector(item["linear_cost"], stage.linear_cost))
    {
      return std::nullopt;
    }
    request.states.push_back(std::move(stage));
  }
  request.inputs.reserve(inputs.size());
  for (const auto & item : inputs) {
    adapter::InputStage stage;
    if (
      !load_fixed_vector(item["reference"], stage.reference) ||
      !load_bound_vector(item["lower"], stage.lower) ||
      !load_bound_vector(item["upper"], stage.upper) ||
      !load_fixed_vector(item["weight"], stage.weight) ||
      !load_fixed_vector(item["linear_cost"], stage.linear_cost))
    {
      return std::nullopt;
    }
    stage.path_curvature_radpm = item["path_curvature_radpm"].as<double>();
    stage.stage_dt_sec = item["stage_dt_sec"].as<double>();
    request.inputs.push_back(std::move(stage));
  }
  return request;
}

bool finite_pose(const recovery_footprint::Pose2D & pose) noexcept
{
  return std::isfinite(pose.x_m) && std::isfinite(pose.y_m) &&
         std::isfinite(pose.yaw_rad);
}

std::optional<recovery_footprint::Pose2D> load_pose(const YAML::Node & node)
{
  if (!node || !node.IsMap()) {
    return std::nullopt;
  }
  recovery_footprint::Pose2D pose{
    node["x_m"].as<double>(), node["y_m"].as<double>(),
    node["yaw_rad"].as<double>()};
  return finite_pose(pose) ? std::optional{pose} : std::nullopt;
}

std::optional<std::shared_ptr<const recovery_footprint::OccupancyGrid>>
load_wall_grid(
  const YAML::Node & node, const std::filesystem::path & snapshot_file)
{
  if (!node || !node.IsMap() || !node["available"] ||
    !node["available"].as<bool>())
  {
    return std::shared_ptr<const recovery_footprint::OccupancyGrid>{};
  }
  auto grid = std::make_shared<recovery_footprint::OccupancyGrid>();
  grid->width = node["width"].as<std::size_t>();
  grid->height = node["height"].as<std::size_t>();
  grid->resolution_m = node["resolution_m"].as<double>();
  grid->origin_x_m = node["origin_x_m"].as<double>();
  grid->origin_y_m = node["origin_y_m"].as<double>();
  const std::string axis = node["y_axis"].as<std::string>();
  if (axis == "row-zero-at-minimum-y") {
    grid->y_axis = recovery_footprint::YAxisConvention::RowZeroAtMinimumY;
  } else if (axis == "row-zero-at-maximum-y") {
    grid->y_axis = recovery_footprint::YAxisConvention::RowZeroAtMaximumY;
  } else {
    return std::nullopt;
  }
  const std::size_t cell_count = node["cell_count"].as<std::size_t>();
  const std::string payload = node["payload"].as<std::string>();
  if (payload.empty() || cell_count != grid->width * grid->height) {
    return std::nullopt;
  }
  std::ifstream stream(snapshot_file.parent_path() / payload, std::ios::binary);
  if (!stream) {
    return std::nullopt;
  }
  grid->cells.reserve(cell_count);
  for (std::size_t index = 0U; index < cell_count; ++index) {
    std::int8_t value{};
    stream.read(reinterpret_cast<char *>(&value), sizeof(value));
    if (!stream) {
      return std::nullopt;
    }
    if (value < -1 || value > 1) {
      return std::nullopt;
    }
    grid->cells.push_back(
      static_cast<recovery_footprint::CellState>(value));
  }
  if (!grid->valid()) {
    return std::nullopt;
  }
  return std::shared_ptr<const recovery_footprint::OccupancyGrid>{grid};
}

std::optional<shadow::Snapshot> load_source_snapshot(
  const YAML::Node & root, const std::filesystem::path & snapshot_file)
{
  const auto node = root["source"];
  const auto problem_context = node["problem_context"];
  const auto intent = parse_intent(problem_context["intent"].as<std::string>());
  const auto formulation = parse_formulation(
    problem_context["formulation"].as<std::string>());
  auto semantic = load_semantic_request(node["semantic_request"]);
  if (!intent || !formulation || !semantic) {
    return std::nullopt;
  }
  shadow::Snapshot source;
  source.identity.sequence = node["sequence"].as<std::uint64_t>();
  source.identity.snapshot_sec = node["snapshot_sec"].as<double>();
  auto & context = source.identity.source_context;
  context.decision_id = problem_context["decision_id"].as<std::uint64_t>();
  context.intent = intent.value();
  context.intent_generation =
    problem_context["intent_generation"].as<std::uint64_t>();
  context.observation_generation =
    problem_context["observation_generation"].as<std::uint64_t>();
  context.stage_geometry_id =
    problem_context["stage_geometry_id"].as<std::uint64_t>();
  context.target_obstacle_generation =
    problem_context["target_obstacle_generation"].as<std::uint64_t>();
  context.target_id = problem_context["target_id"].as<std::string>();
  context.execution_side_sign = problem_context["execution_side_sign"].as<int>();
  context.horizon_steps = problem_context["horizon_steps"].as<std::size_t>();
  context.formulation = formulation.value();
  context.state_schema_id = problem_context["state_schema_id"].as<std::string>();
  context.input_schema_id = problem_context["input_schema_id"].as<std::string>();
  context.bounds_schema_id = problem_context["bounds_schema_id"].as<std::string>();
  context.cost_schema_id = problem_context["cost_schema_id"].as<std::string>();
  context.fingerprint = problem_context["fingerprint"].as<std::uint64_t>();
  source.control_prediction_origin_sec =
    node["control_prediction_origin_sec"].as<double>();
  source.course_progress_origin_m =
    node["course_progress_origin_m"].as<double>();
  source.execution_prefix_steps = node["execution_prefix_steps"].as<int>();
  source.publication_interval_sec = node["publication_interval_sec"].as<double>();
  source.request = std::move(semantic.value());
  auto nominal = load_std_vector(node["nominal_path_distance_m"]);
  auto wall_progress = load_std_vector(node["wall_reference_progress_m"]);
  auto wall_lower = load_std_vector(node["wall_lower_m"]);
  auto wall_upper = load_std_vector(node["wall_upper_m"]);
  if (!nominal || !wall_progress || !wall_lower || !wall_upper) {
    return std::nullopt;
  }
  source.nominal_path_distance_m = std::move(nominal.value());
  source.progress_aligned_wall_refinement_active =
    node["progress_aligned_wall_refinement_active"].as<bool>();
  source.wall_reference_progress_m = std::move(wall_progress.value());
  source.wall_lower_m = std::move(wall_lower.value());
  source.wall_upper_m = std::move(wall_upper.value());
  source.progress_wall_profile_diagnostic =
    node["progress_wall_profile_diagnostic"].as<std::string>();
  source.dynamic_obstacle_refinement_active =
    node["dynamic_obstacle_refinement_active"].as<bool>();
  source.dynamic_obstacle_pass_side_sign =
    node["dynamic_obstacle_pass_side_sign"].as<int>();
  const auto stages = node["dynamic_obstacle_stages"];
  if (!stages || !stages.IsSequence()) {
    return std::nullopt;
  }
  source.dynamic_obstacle_stages.reserve(stages.size());
  for (const auto & item : stages) {
    source.dynamic_obstacle_stages.push_back(
      mpcc_rate_resolved_dynamic_obstacle::StagePrediction{
        item["valid"].as<bool>(), item["target_progress_m"].as<double>(),
        item["target_lateral_m"].as<double>(),
        item["longitudinal_overlap_m"].as<double>(),
        item["lateral_center_separation_m"].as<double>()});
  }
  source.physical_wall_refinement_active =
    node["physical_wall_refinement_active"].as<bool>();
  const auto footprint = node["wall_footprint"];
  source.wall_footprint.front_extent_m = footprint["front_extent_m"].as<double>();
  source.wall_footprint.rear_extent_m = footprint["rear_extent_m"].as<double>();
  source.wall_footprint.left_extent_m = footprint["left_extent_m"].as<double>();
  source.wall_footprint.right_extent_m = footprint["right_extent_m"].as<double>();
  source.wall_footprint.margin_m = footprint["margin_m"].as<double>();
  const auto knots = node["wall_course_frame_knots"];
  if (!knots || !knots.IsSequence()) {
    return std::nullopt;
  }
  source.wall_course_frame_knots.reserve(knots.size());
  for (const auto & item : knots) {
    source.wall_course_frame_knots.push_back(
      mpc_stage_geometry::CourseFrameKnot{
        item["progress_m"].as<double>(), item["x_m"].as<double>(),
        item["y_m"].as<double>(), item["heading_rad"].as<double>(),
        item["waypoint"].as<int>()});
  }
  source.wall_lateral_sample_step_m =
    node["wall_lateral_sample_step_m"].as<double>();
  source.wall_heading_bucket_width_rad =
    node["wall_heading_bucket_width_rad"].as<double>();
  source.wall_translation_bucket_width_m =
    node["wall_translation_bucket_width_m"].as<double>();
  source.wall_boundary_guard_m = node["wall_boundary_guard_m"].as<double>();
  auto grid = load_wall_grid(node["wall_grid"], snapshot_file);
  if (!grid.has_value()) {
    return std::nullopt;
  }
  source.wall_grid = std::move(grid.value());
  const auto replay = node["replay_world"];
  if (replay && replay["available"] && replay["available"].as<bool>()) {
    shadow::ReplayWorld world;
    world.observation_generation =
      replay["observation_generation"].as<std::uint64_t>();
    world.observed_sec = replay["observed_sec"].as<double>();
    world.current = replay["current"].as<bool>();
    const auto current_pose = load_pose(replay["current_pose"]);
    const auto prefix = replay["control_prefix"];
    if (!current_pose || !prefix || !prefix.IsSequence()) {
      return std::nullopt;
    }
    world.current_pose = current_pose.value();
    world.control_prefix.reserve(prefix.size());
    for (const auto & item : prefix) {
      const auto pose = load_pose(item);
      if (!pose) {
        return std::nullopt;
      }
      world.control_prefix.push_back(pose.value());
    }
    const auto prefix_elapsed = load_std_vector(
      replay["control_prefix_elapsed_sec"]);
    if (!prefix_elapsed.has_value()) {
      return std::nullopt;
    }
    world.control_prefix_elapsed_sec = prefix_elapsed.value();
    const auto physical_footprint = replay["physical_footprint"];
    if (!physical_footprint || !physical_footprint.IsMap()) {
      return std::nullopt;
    }
    world.physical_footprint.front_extent_m =
      physical_footprint["front_extent_m"].as<double>();
    world.physical_footprint.rear_extent_m =
      physical_footprint["rear_extent_m"].as<double>();
    world.physical_footprint.left_extent_m =
      physical_footprint["left_extent_m"].as<double>();
    world.physical_footprint.right_extent_m =
      physical_footprint["right_extent_m"].as<double>();
    world.physical_footprint.margin_m =
      physical_footprint["margin_m"].as<double>();
    world.wall_grid_fingerprint =
      replay["wall_grid_fingerprint"].as<std::uint64_t>();
    world.hard_wall_clearance_m = replay["hard_wall_clearance_m"].as<double>();
    world.bound_tolerance_m = replay["bound_tolerance_m"].as<double>();
    world.swept_step_m = replay["swept_step_m"].as<double>();
    const auto obstacles = replay["obstacles"];
    if (!obstacles || !obstacles.IsSequence()) {
      return std::nullopt;
    }
    world.obstacles.reserve(obstacles.size());
    for (const auto & item : obstacles) {
      world.obstacles.push_back(shadow::ReplayDynamicObstacle{
        item["id"].as<std::string>(), item["x_m"].as<double>(),
        item["y_m"].as<double>(), item["velocity_x_mps"].as<double>(),
        item["velocity_y_mps"].as<double>(),
        item["acceleration_x_mps2"].as<double>(),
        item["acceleration_y_mps2"].as<double>(),
        item["covariance_x_m2"].as<double>(),
        item["covariance_y_m2"].as<double>(), item["radius_m"].as<double>(),
        item["observation_generation"].as<std::uint64_t>()});
    }
    std::sort(
      world.obstacles.begin(), world.obstacles.end(),
      [](const shadow::ReplayDynamicObstacle & lhs,
        const shadow::ReplayDynamicObstacle & rhs) {
        return lhs.id < rhs.id;
      });
    source.replay_world = std::move(world);
  }
  return source;
}

class InteractionFingerprintBuilder
{
public:
  void append_byte(const std::uint8_t value) noexcept
  {
    value_ ^= value;
    value_ *= 1099511628211ULL;
  }

  void append_u64(const std::uint64_t value) noexcept
  {
    for (unsigned int shift = 0U; shift < 64U; shift += 8U) {
      append_byte(static_cast<std::uint8_t>((value >> shift) & 0xffU));
    }
  }

  void append_i64(const std::int64_t value) noexcept
  {
    append_u64(static_cast<std::uint64_t>(value));
  }

  void append_bool(const bool value) noexcept
  {
    append_byte(value ? 1U : 0U);
  }

  void append_double(double value) noexcept
  {
    if (value == 0.0) value = 0.0;
    std::uint64_t bits{};
    std::memcpy(&bits, &value, sizeof(bits));
    append_u64(bits);
  }

  void append_string(const std::string & value) noexcept
  {
    append_u64(static_cast<std::uint64_t>(value.size()));
    for (const unsigned char character : value) append_byte(character);
  }

  template<typename Derived>
  void append_eigen(const Eigen::MatrixBase<Derived> & value) noexcept
  {
    append_u64(static_cast<std::uint64_t>(value.size()));
    for (Eigen::Index index = 0; index < value.size(); ++index) {
      append_double(value(index));
    }
  }

  void append_vector(const std::vector<double> & values) noexcept
  {
    append_u64(static_cast<std::uint64_t>(values.size()));
    for (const double value : values) append_double(value);
  }

  std::uint64_t finish() const noexcept
  {
    return value_ == 0U ? 1U : value_;
  }

private:
  std::uint64_t value_{14695981039346656037ULL};
};

std::string failure_key(
  const shadow::Snapshot & source, const PipelineStage stage,
  const std::string & failure_outcome)
{
  return std::string{contract::to_string(source.identity.source_context.intent)} +
    '|' + to_string(stage) + '|' + failure_outcome;
}

}  // namespace

const char * to_string(const PipelineStage stage) noexcept
{
  switch (stage) {
    case PipelineStage::Initial: return "initial";
    case PipelineStage::SuccessiveLinearization:
      return "successive-linearization";
    case PipelineStage::WallRefinement: return "wall-refinement";
    case PipelineStage::DynamicObstacleRefinement:
      return "dynamic-obstacle-refinement";
    case PipelineStage::PostRefinementLinearization:
      return "post-refinement-linearization";
    case PipelineStage::PhysicalProof: return "physical-proof";
  }
  return "unknown";
}

const char * to_string(const RecordStatus status) noexcept
{
  switch (status) {
    case RecordStatus::Written: return "written";
    case RecordStatus::Duplicate: return "duplicate";
    case RecordStatus::NotOvertake: return "not-overtake";
    case RecordStatus::InvalidInput: return "invalid-input";
    case RecordStatus::IoFailure: return "io-failure";
  }
  return "invalid-input";
}

bool interaction_snapshot_complete(const shadow::Snapshot & source) noexcept
{
  try {
    const auto & request = source.request;
    if (
      source.identity.sequence == 0U ||
      !std::isfinite(source.identity.snapshot_sec) ||
      !std::isfinite(source.control_prediction_origin_sec) ||
      source.control_prediction_origin_sec < source.identity.snapshot_sec ||
      !std::isfinite(source.course_progress_origin_m) ||
      !std::isfinite(source.publication_interval_sec) ||
      source.publication_interval_sec <= 0.0 ||
      !contract::problem_context_complete(source.identity.source_context) ||
      request.horizon_steps <= 0 ||
      source.identity.source_context.horizon_steps !=
      static_cast<std::size_t>(request.horizon_steps) ||
      source.execution_prefix_steps <= 0 ||
      source.execution_prefix_steps > request.horizon_steps ||
      !request.initial_state.allFinite() ||
      !request.previous_input.allFinite() ||
      !request.input_delta_weight.allFinite() ||
      !std::isfinite(request.current_steering_rad) ||
      !std::isfinite(request.current_response_steering_rad) ||
      !std::isfinite(request.wheelbase_m) || request.wheelbase_m <= 0.0 ||
      !std::isfinite(request.yaw_response_gain) ||
      !std::isfinite(request.yaw_response_time_constant_sec) ||
      request.yaw_response_time_constant_sec <= 0.0 ||
      !std::isfinite(request.maximum_abs_steering_rad) ||
      request.maximum_abs_steering_rad <= 0.0 ||
      !std::isfinite(request.maximum_abs_steering_rate_radps) ||
      request.maximum_abs_steering_rate_radps <= 0.0 ||
      !std::isfinite(request.minimum_frenet_denominator) ||
      request.minimum_frenet_denominator <= 0.0 ||
      !std::isfinite(request.minimum_stage_dt_sec) ||
      !std::isfinite(request.maximum_stage_dt_sec) ||
      request.minimum_stage_dt_sec <= 0.0 ||
      request.maximum_stage_dt_sec < request.minimum_stage_dt_sec ||
      request.states.size() !=
      static_cast<std::size_t>(request.horizon_steps + 1) ||
      request.inputs.size() != static_cast<std::size_t>(request.horizon_steps) ||
      source.nominal_path_distance_m.size() !=
      static_cast<std::size_t>(request.horizon_steps + 1))
    {
      return false;
    }
    for (const auto & stage : request.states) {
      if (
        !stage.reference.allFinite() ||
        !valid_semantic_bounds(stage.lower, stage.upper) ||
        !stage.weight.allFinite() ||
        !stage.linear_cost.allFinite() ||
        (stage.weight.array() < 0.0).any())
      {
        return false;
      }
    }
    for (const auto & stage : request.inputs) {
      if (
        !stage.reference.allFinite() ||
        !valid_semantic_bounds(stage.lower, stage.upper) ||
        !stage.weight.allFinite() ||
        !stage.linear_cost.allFinite() ||
        (stage.weight.array() < 0.0).any() ||
        !std::isfinite(stage.path_curvature_radpm) ||
        !std::isfinite(stage.stage_dt_sec) || stage.stage_dt_sec <= 0.0)
      {
        return false;
      }
    }
    if (
      !source.progress_aligned_wall_refinement_active ||
      source.wall_reference_progress_m.size() < 2U ||
      source.wall_reference_progress_m.size() != source.wall_lower_m.size() ||
      source.wall_reference_progress_m.size() != source.wall_upper_m.size() ||
      !source.physical_wall_refinement_active || source.wall_grid == nullptr ||
      !source.wall_grid->valid() || !source.wall_footprint.valid() ||
      source.wall_course_frame_knots.size() < 2U ||
      !std::isfinite(source.wall_lateral_sample_step_m) ||
      source.wall_lateral_sample_step_m <= 0.0 ||
      !std::isfinite(source.wall_heading_bucket_width_rad) ||
      source.wall_heading_bucket_width_rad <= 0.0 ||
      !std::isfinite(source.wall_translation_bucket_width_m) ||
      source.wall_translation_bucket_width_m <= 0.0 ||
      !std::isfinite(source.wall_boundary_guard_m) ||
      source.wall_boundary_guard_m < 0.0 || !source.replay_world.has_value())
    {
      return false;
    }
    for (std::size_t index = 0U; index < source.wall_reference_progress_m.size(); ++index) {
      if (
        !std::isfinite(source.wall_reference_progress_m[index]) ||
        !std::isfinite(source.wall_lower_m[index]) ||
        !std::isfinite(source.wall_upper_m[index]) ||
        source.wall_lower_m[index] > source.wall_upper_m[index] ||
        (index > 0U && source.wall_reference_progress_m[index] <=
        source.wall_reference_progress_m[index - 1U]))
      {
        return false;
      }
    }
    for (std::size_t index = 0U; index < source.nominal_path_distance_m.size(); ++index) {
      if (
        !std::isfinite(source.nominal_path_distance_m[index]) ||
        source.nominal_path_distance_m[index] < 0.0 ||
        (index > 0U && source.nominal_path_distance_m[index] <
        source.nominal_path_distance_m[index - 1U]))
      {
        return false;
      }
    }
    if (
      source.dynamic_obstacle_refinement_active &&
      source.dynamic_obstacle_stages.size() !=
      static_cast<std::size_t>(request.horizon_steps))
    {
      return false;
    }
    for (const auto & stage : source.dynamic_obstacle_stages) {
      if (
        !std::isfinite(stage.target_progress_m) ||
        !std::isfinite(stage.target_lateral_m) ||
        !std::isfinite(stage.longitudinal_overlap_m) ||
        !std::isfinite(stage.lateral_center_separation_m))
      {
        return false;
      }
    }
    for (const auto & knot : source.wall_course_frame_knots) {
      if (
        !std::isfinite(knot.progress_m) || !std::isfinite(knot.x_m) ||
        !std::isfinite(knot.y_m) || !std::isfinite(knot.heading_rad))
      {
        return false;
      }
    }
    const auto & world = source.replay_world.value();
    if (
      !world.current || world.observation_generation == 0U ||
      world.observation_generation !=
      source.identity.source_context.target_obstacle_generation ||
      !std::isfinite(world.observed_sec) || !finite_pose(world.current_pose) ||
      world.control_prefix.empty() ||
      world.control_prefix.size() != world.control_prefix_elapsed_sec.size() ||
      !world.physical_footprint.valid() ||
      std::any_of(
        world.control_prefix.begin(), world.control_prefix.end(),
        [](const auto & pose) {return !finite_pose(pose);}) ||
      std::any_of(
        world.control_prefix_elapsed_sec.begin(),
        world.control_prefix_elapsed_sec.end(),
        [](const double elapsed_sec) {return !std::isfinite(elapsed_sec);}) ||
      std::abs(world.control_prefix_elapsed_sec.front()) > 1e-9 ||
      std::abs(
        world.control_prefix_elapsed_sec.back() -
        (source.control_prediction_origin_sec - world.observed_sec)) > 1e-9 ||
      std::abs(world.control_prefix.front().x_m - world.current_pose.x_m) > 1e-9 ||
      std::abs(world.control_prefix.front().y_m - world.current_pose.y_m) > 1e-9 ||
      std::abs(world.control_prefix.front().yaw_rad - world.current_pose.yaw_rad) > 1e-9 ||
      world.wall_grid_fingerprint == 0U ||
      world.wall_grid_fingerprint !=
      recovery_footprint::occupancy_grid_fingerprint(*source.wall_grid) ||
      !std::isfinite(world.hard_wall_clearance_m) ||
      world.hard_wall_clearance_m < 0.0 ||
      std::abs(
        world.physical_footprint.front_extent_m -
        source.wall_footprint.front_extent_m) > 1e-9 ||
      std::abs(
        world.physical_footprint.rear_extent_m -
        source.wall_footprint.rear_extent_m) > 1e-9 ||
      std::abs(
        world.physical_footprint.left_extent_m +
        world.hard_wall_clearance_m -
        source.wall_footprint.left_extent_m) > 1e-9 ||
      std::abs(
        world.physical_footprint.right_extent_m +
        world.hard_wall_clearance_m -
        source.wall_footprint.right_extent_m) > 1e-9 ||
      std::abs(
        world.physical_footprint.margin_m -
        source.wall_footprint.margin_m) > 1e-9 ||
      !std::isfinite(world.bound_tolerance_m) || world.bound_tolerance_m < 0.0 ||
      !std::isfinite(world.swept_step_m) || world.swept_step_m <= 0.0)
    {
      return false;
    }
    for (std::size_t index = 1U;
      index < world.control_prefix_elapsed_sec.size(); ++index)
    {
      if (
        world.control_prefix_elapsed_sec[index] <=
        world.control_prefix_elapsed_sec[index - 1U])
      {
        return false;
      }
    }
    std::set<std::string> obstacle_ids;
    bool target_present = source.identity.source_context.target_id.empty();
    for (const auto & obstacle : world.obstacles) {
      if (
        obstacle.id.empty() || !obstacle_ids.insert(obstacle.id).second ||
        obstacle.observation_generation != world.observation_generation ||
        !std::isfinite(obstacle.x_m) || !std::isfinite(obstacle.y_m) ||
        !std::isfinite(obstacle.velocity_x_mps) ||
        !std::isfinite(obstacle.velocity_y_mps) ||
        !std::isfinite(obstacle.acceleration_x_mps2) ||
        !std::isfinite(obstacle.acceleration_y_mps2) ||
        !std::isfinite(obstacle.covariance_x_m2) ||
        obstacle.covariance_x_m2 < 0.0 ||
        !std::isfinite(obstacle.covariance_y_m2) ||
        obstacle.covariance_y_m2 < 0.0 ||
        !std::isfinite(obstacle.radius_m) || obstacle.radius_m <= 0.0)
      {
        return false;
      }
      target_present = target_present ||
        obstacle.id == source.identity.source_context.target_id;
    }
    return target_present;
  } catch (...) {
    return false;
  }
}

std::uint64_t fingerprint_interaction_snapshot(
  const shadow::Snapshot & source) noexcept
{
  if (!interaction_snapshot_complete(source)) {
    return 0U;
  }
  InteractionFingerprintBuilder builder;
  builder.append_string("mpcc-interaction-snapshot-v2");
  builder.append_u64(source.identity.sequence);
  builder.append_u64(source.identity.source_context.fingerprint);
  builder.append_double(source.identity.snapshot_sec);
  builder.append_double(source.control_prediction_origin_sec);
  builder.append_i64(source.execution_prefix_steps);
  builder.append_double(source.course_progress_origin_m);
  builder.append_double(source.publication_interval_sec);
  const auto & request = source.request;
  builder.append_i64(request.horizon_steps);
  builder.append_eigen(request.initial_state);
  builder.append_double(request.current_steering_rad);
  builder.append_double(request.current_response_steering_rad);
  builder.append_double(request.wheelbase_m);
  builder.append_double(request.yaw_response_gain);
  builder.append_double(request.yaw_response_time_constant_sec);
  builder.append_double(request.maximum_abs_steering_rad);
  builder.append_double(request.maximum_abs_steering_rate_radps);
  builder.append_double(request.minimum_frenet_denominator);
  builder.append_double(request.minimum_stage_dt_sec);
  builder.append_double(request.maximum_stage_dt_sec);
  builder.append_eigen(request.previous_input);
  builder.append_eigen(request.input_delta_weight);
  builder.append_u64(static_cast<std::uint64_t>(request.states.size()));
  for (const auto & stage : request.states) {
    builder.append_eigen(stage.reference);
    builder.append_eigen(stage.lower);
    builder.append_eigen(stage.upper);
    builder.append_eigen(stage.weight);
    builder.append_eigen(stage.linear_cost);
  }
  builder.append_u64(static_cast<std::uint64_t>(request.inputs.size()));
  for (const auto & stage : request.inputs) {
    builder.append_eigen(stage.reference);
    builder.append_eigen(stage.lower);
    builder.append_eigen(stage.upper);
    builder.append_eigen(stage.weight);
    builder.append_eigen(stage.linear_cost);
    builder.append_double(stage.path_curvature_radpm);
    builder.append_double(stage.stage_dt_sec);
  }
  builder.append_vector(source.nominal_path_distance_m);
  builder.append_vector(source.wall_reference_progress_m);
  builder.append_vector(source.wall_lower_m);
  builder.append_vector(source.wall_upper_m);
  builder.append_bool(source.dynamic_obstacle_refinement_active);
  builder.append_i64(source.dynamic_obstacle_pass_side_sign);
  builder.append_u64(
    static_cast<std::uint64_t>(source.dynamic_obstacle_stages.size()));
  for (const auto & stage : source.dynamic_obstacle_stages) {
    builder.append_bool(stage.valid);
    builder.append_double(stage.target_progress_m);
    builder.append_double(stage.target_lateral_m);
    builder.append_double(stage.longitudinal_overlap_m);
    builder.append_double(stage.lateral_center_separation_m);
  }
  builder.append_double(source.wall_footprint.front_extent_m);
  builder.append_double(source.wall_footprint.rear_extent_m);
  builder.append_double(source.wall_footprint.left_extent_m);
  builder.append_double(source.wall_footprint.right_extent_m);
  builder.append_double(source.wall_footprint.margin_m);
  builder.append_u64(
    static_cast<std::uint64_t>(source.wall_course_frame_knots.size()));
  for (const auto & knot : source.wall_course_frame_knots) {
    builder.append_double(knot.progress_m);
    builder.append_double(knot.x_m);
    builder.append_double(knot.y_m);
    builder.append_double(knot.heading_rad);
    builder.append_i64(knot.waypoint);
  }
  builder.append_double(source.wall_lateral_sample_step_m);
  builder.append_double(source.wall_heading_bucket_width_rad);
  builder.append_double(source.wall_translation_bucket_width_m);
  builder.append_double(source.wall_boundary_guard_m);
  const auto & world = source.replay_world.value();
  builder.append_u64(world.observation_generation);
  builder.append_double(world.observed_sec);
  builder.append_double(world.current_pose.x_m);
  builder.append_double(world.current_pose.y_m);
  builder.append_double(world.current_pose.yaw_rad);
  builder.append_u64(static_cast<std::uint64_t>(world.control_prefix.size()));
  for (const auto & pose : world.control_prefix) {
    builder.append_double(pose.x_m);
    builder.append_double(pose.y_m);
    builder.append_double(pose.yaw_rad);
  }
  builder.append_vector(world.control_prefix_elapsed_sec);
  builder.append_double(world.physical_footprint.front_extent_m);
  builder.append_double(world.physical_footprint.rear_extent_m);
  builder.append_double(world.physical_footprint.left_extent_m);
  builder.append_double(world.physical_footprint.right_extent_m);
  builder.append_double(world.physical_footprint.margin_m);
  builder.append_u64(world.wall_grid_fingerprint);
  builder.append_double(world.hard_wall_clearance_m);
  builder.append_double(world.bound_tolerance_m);
  builder.append_double(world.swept_step_m);
  auto obstacles = world.obstacles;
  std::sort(
    obstacles.begin(), obstacles.end(),
    [](const shadow::ReplayDynamicObstacle & lhs,
      const shadow::ReplayDynamicObstacle & rhs) {return lhs.id < rhs.id;});
  builder.append_u64(static_cast<std::uint64_t>(obstacles.size()));
  for (const auto & obstacle : obstacles) {
    builder.append_string(obstacle.id);
    builder.append_double(obstacle.x_m);
    builder.append_double(obstacle.y_m);
    builder.append_double(obstacle.velocity_x_mps);
    builder.append_double(obstacle.velocity_y_mps);
    builder.append_double(obstacle.acceleration_x_mps2);
    builder.append_double(obstacle.acceleration_y_mps2);
    builder.append_double(obstacle.covariance_x_m2);
    builder.append_double(obstacle.covariance_y_m2);
    builder.append_double(obstacle.radius_m);
    builder.append_u64(obstacle.observation_generation);
  }
  return builder.finish();
}

bool interaction_snapshot_matches_fingerprint(
  const shadow::Snapshot & source,
  const std::uint64_t expected_fingerprint) noexcept
{
  return expected_fingerprint != 0U &&
         fingerprint_interaction_snapshot(source) == expected_fingerprint;
}

RecordResult record_failure(
  const shadow::Snapshot & source,
  const problem::AssemblyRequest & assembly_request,
  const problem::Problem & exact_problem,
  const std::optional<persistent_osqp::WarmStart> & warm_start,
  const persistent_osqp::SolveOutcome & outcome,
  const PipelineStage pipeline_stage,
  const std::string & failure_outcome,
  const std::string & failure_detail,
  const std::filesystem::path & output_root) noexcept
{
  RecordResult result;
  try {
    const auto intent = source.identity.source_context.intent;
    if (!is_overtake(intent)) {
      result.status = RecordStatus::NotOvertake;
      result.detail = "capture is restricted to Overtake intents";
      return result;
    }
    if (
      failure_outcome.empty() || output_root.empty() ||
      exact_problem.horizon_steps <= 0 ||
      exact_problem.linear_cost.size() <= 0 ||
      exact_problem.quadratic_cost.rows() != exact_problem.linear_cost.size() ||
      exact_problem.quadratic_cost.cols() != exact_problem.linear_cost.size() ||
      exact_problem.constraints.cols() != exact_problem.linear_cost.size() ||
      exact_problem.constraints.rows() != exact_problem.lower_bound.size() ||
      exact_problem.lower_bound.size() != exact_problem.upper_bound.size())
    {
      result.status = RecordStatus::InvalidInput;
      result.detail = "invalid exact QP capture request";
      return result;
    }

    const std::string key = failure_key(
      source, pipeline_stage, failure_outcome);
    std::lock_guard<std::mutex> lock(record_mutex);
    if (recorded_failure_keys.count(key) != 0U) {
      result.status = RecordStatus::Duplicate;
      result.detail = key;
      return result;
    }

    std::ostringstream sequence;
    sequence << std::setw(12) << std::setfill('0') << source.identity.sequence;
    const std::string directory_name = sequence.str() + '-' +
      safe_component(contract::to_string(intent)) + '-' +
      safe_component(to_string(pipeline_stage)) + '-' +
      safe_component(failure_outcome);
    const auto final_directory = output_root / directory_name;
    const auto temporary_directory = output_root / (directory_name + ".tmp");
    std::error_code error;
    std::filesystem::create_directories(output_root, error);
    if (error) {
      result.status = RecordStatus::IoFailure;
      result.detail = "cannot create snapshot root: " + error.message();
      return result;
    }
    std::filesystem::remove_all(temporary_directory, error);
    error.clear();
    std::filesystem::create_directories(temporary_directory, error);
    if (error) {
      result.status = RecordStatus::IoFailure;
      result.detail = "cannot create temporary snapshot: " + error.message();
      return result;
    }

    const std::string grid_payload = "wall-grid.bin";
    if (source.wall_grid != nullptr) {
      std::ofstream grid(
        temporary_directory / grid_payload,
        std::ios::binary | std::ios::trunc);
      if (!grid) {
        result.status = RecordStatus::IoFailure;
        result.detail = "cannot open wall-grid payload";
        std::filesystem::remove_all(temporary_directory, error);
        return result;
      }
      for (const auto cell : source.wall_grid->cells) {
        const std::int8_t byte = static_cast<std::int8_t>(cell);
        grid.write(reinterpret_cast<const char *>(&byte), sizeof(byte));
      }
      if (!grid) {
        result.status = RecordStatus::IoFailure;
        result.detail = "cannot write wall-grid payload";
        std::filesystem::remove_all(temporary_directory, error);
        return result;
      }
    }

    YAML::Node root;
    root["schema"] = "mpcc-architecture-failure-snapshot/v1";
    root["pipeline_stage"] = to_string(pipeline_stage);
    root["failure_outcome"] = failure_outcome;
    root["failure_detail"] = failure_detail;
    root["source"] = source_node(
      source, source.wall_grid != nullptr ? grid_payload : "");
    root["interaction_fingerprint"] =
      fingerprint_interaction_snapshot(source);
    root["assembly_request"] = assembly_request_node(assembly_request);
    root["exact_qp"] = exact_problem_node(exact_problem);
    root["warm_start"] = warm_start_node(warm_start);
    root["production_outcome"] = outcome_node(outcome);

    YAML::Emitter emitter;
    emitter.SetDoublePrecision(std::numeric_limits<double>::max_digits10);
    emitter << root;
    if (!emitter.good()) {
      result.status = RecordStatus::IoFailure;
      result.detail = "cannot serialize snapshot: " + emitter.GetLastError();
      std::filesystem::remove_all(temporary_directory, error);
      return result;
    }
    const auto temporary_file = temporary_directory / "snapshot.yaml";
    std::ofstream stream(temporary_file, std::ios::trunc);
    stream << emitter.c_str() << '\n';
    stream.close();
    if (!stream) {
      result.status = RecordStatus::IoFailure;
      result.detail = "cannot write snapshot.yaml";
      std::filesystem::remove_all(temporary_directory, error);
      return result;
    }
    if (std::filesystem::exists(final_directory)) {
      std::filesystem::remove_all(temporary_directory, error);
    } else {
      std::filesystem::rename(temporary_directory, final_directory, error);
      if (error) {
        result.status = RecordStatus::IoFailure;
        result.detail = "cannot publish snapshot atomically: " + error.message();
        std::filesystem::remove_all(temporary_directory, error);
        return result;
      }
    }
    recorded_failure_keys.insert(key);
    result.status = RecordStatus::Written;
    result.snapshot_file = final_directory / "snapshot.yaml";
    result.detail = key;
    return result;
  } catch (const std::exception & exception) {
    result.status = RecordStatus::IoFailure;
    result.detail = exception.what();
    return result;
  } catch (...) {
    result.status = RecordStatus::IoFailure;
    result.detail = "unknown snapshot recorder exception";
    return result;
  }
}

std::optional<RecordedQp> load_recorded_qp(
  const std::filesystem::path & snapshot_file, std::string * detail) noexcept
{
  try {
    const YAML::Node root = YAML::LoadFile(snapshot_file.string());
    if (!root["schema"] ||
      root["schema"].as<std::string>() !=
      "mpcc-architecture-failure-snapshot/v1")
    {
      if (detail != nullptr) {
        *detail = "unsupported snapshot schema";
      }
      return std::nullopt;
    }
    const auto qp = root["exact_qp"];
    auto linear_cost = load_vector(qp["linear_cost"]);
    auto lower_bound = load_vector(qp["lower_bound"]);
    auto upper_bound = load_vector(qp["upper_bound"]);
    auto scaling = load_vector(qp["variable_scaling"]);
    auto quadratic_cost = load_sparse(qp["quadratic_cost"]);
    auto constraints = load_sparse(qp["constraints"]);
    if (
      !linear_cost || !lower_bound || !upper_bound || !scaling ||
      !quadratic_cost || !constraints)
    {
      if (detail != nullptr) {
        *detail = "malformed exact QP payload";
      }
      return std::nullopt;
    }
    RecordedQp recorded;
    recorded.problem.horizon_steps = qp["horizon_steps"].as<int>();
    recorded.problem.linear_cost = std::move(linear_cost.value());
    recorded.problem.lower_bound = std::move(lower_bound.value());
    recorded.problem.upper_bound = std::move(upper_bound.value());
    recorded.problem.quadratic_cost = std::move(quadratic_cost.value());
    recorded.problem.constraints = std::move(constraints.value());
    recorded.problem.variable_scaling.physical_units_per_solver_unit =
      std::move(scaling.value());
    const auto warm = root["warm_start"];
    if (warm && warm["available"] && warm["available"].as<bool>()) {
      auto primal = load_vector(warm["primal"]);
      auto dual = load_vector(warm["dual"]);
      if (!primal || !dual) {
        if (detail != nullptr) {
          *detail = "malformed warm-start payload";
        }
        return std::nullopt;
      }
      recorded.warm_start = persistent_osqp::WarmStart{
        std::move(primal.value()), std::move(dual.value())};
    }
    recorded.intent = root["source"]["problem_context"]["intent"].as<std::string>();
    recorded.pipeline_stage = root["pipeline_stage"].as<std::string>();
    recorded.failure_outcome = root["failure_outcome"].as<std::string>();
    recorded.failure_detail = root["failure_detail"].as<std::string>();
    if (detail != nullptr) {
      *detail = "loaded";
    }
    return recorded;
  } catch (const std::exception & exception) {
    if (detail != nullptr) {
      *detail = exception.what();
    }
    return std::nullopt;
  } catch (...) {
    if (detail != nullptr) {
      *detail = "unknown snapshot loader exception";
    }
    return std::nullopt;
  }
}

std::optional<RecordedInteractionSnapshot> load_recorded_interaction_snapshot(
  const std::filesystem::path & snapshot_file, std::string * detail) noexcept
{
  try {
    const YAML::Node root = YAML::LoadFile(snapshot_file.string());
    if (!root["schema"] ||
      root["schema"].as<std::string>() !=
      "mpcc-architecture-failure-snapshot/v1")
    {
      if (detail != nullptr) *detail = "unsupported snapshot schema";
      return std::nullopt;
    }
    auto source = load_source_snapshot(root, snapshot_file);
    if (!source.has_value() || !interaction_snapshot_complete(source.value())) {
      if (detail != nullptr) *detail = "interaction snapshot incomplete";
      return std::nullopt;
    }
    if (!root["interaction_fingerprint"]) {
      if (detail != nullptr) *detail = "interaction fingerprint unavailable";
      return std::nullopt;
    }
    const std::uint64_t fingerprint =
      root["interaction_fingerprint"].as<std::uint64_t>();
    if (!interaction_snapshot_matches_fingerprint(source.value(), fingerprint)) {
      if (detail != nullptr) *detail = "interaction fingerprint mismatch";
      return std::nullopt;
    }
    std::string qp_detail;
    auto recorded_qp = load_recorded_qp(snapshot_file, &qp_detail);
    if (!recorded_qp.has_value()) {
      if (detail != nullptr) *detail = "exact QP unavailable: " + qp_detail;
      return std::nullopt;
    }
    RecordedInteractionSnapshot recorded;
    recorded.source = std::move(source.value());
    recorded.recorded_qp = std::move(recorded_qp.value());
    recorded.interaction_fingerprint = fingerprint;
    if (detail != nullptr) *detail = "loaded";
    return recorded;
  } catch (const std::exception & exception) {
    if (detail != nullptr) *detail = exception.what();
    return std::nullopt;
  } catch (...) {
    if (detail != nullptr) *detail = "unknown interaction snapshot loader exception";
    return std::nullopt;
  }
}

ReplayResult replay_recorded_qp(
  const std::filesystem::path & snapshot_file,
  const bool use_recorded_warm_start) noexcept
{
  ReplayResult replay;
  replay.warm_start_requested = use_recorded_warm_start;
  auto recorded = load_recorded_qp(snapshot_file, &replay.detail);
  if (!recorded.has_value()) {
    return replay;
  }
  replay.loaded = true;
  replay.warm_start_available = recorded->warm_start.has_value();
  persistent_osqp::PersistentOsqpSolver solver(
    persistent_osqp::ConstraintPreconditioningPolicy::RowToleranceNormalized);
  replay.outcome = solver.solve(
    recorded->problem.quadratic_cost,
    recorded->problem.constraints,
    recorded->problem.linear_cost,
    recorded->problem.lower_bound,
    recorded->problem.upper_bound,
    use_recorded_warm_start ? recorded->warm_start : std::nullopt,
    recorded->problem.variable_scaling);
  replay.detail = replay.outcome.result.has_value() ?
    "solved" : replay.outcome.failure_detail;
  return replay;
}

}  // namespace multi_purpose_mpc_ros::mpcc_architecture_snapshot
