#include "multi_purpose_mpc_ros/mpcc_applied_input_yaml.hpp"
#include "multi_purpose_mpc_ros/mpcc_vehicle_model_yaml.hpp"
#include "multi_purpose_mpc_ros/mpcc_architecture_snapshot.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_retained_revalidation.hpp"

#include <condition_variable>
#include <deque>
#include <thread>

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

constexpr const char * kExactQpSnapshotSchemaV1 =
  "mpcc-architecture-failure-snapshot/v1";
constexpr const char * kInteractionSnapshotSchemaV2 =
  "mpcc-architecture-failure-snapshot/v2";
constexpr const char * kInteractionSnapshotSchemaV3 =
  "mpcc-architecture-failure-snapshot/v3";

bool supported_exact_qp_schema(const std::string & schema) noexcept
{
  return schema == kExactQpSnapshotSchemaV1 ||
         schema == kInteractionSnapshotSchemaV2 ||
         schema == kInteractionSnapshotSchemaV3;
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
    if (
      constraint.axis == problem::DynamicObstacleConstraintAxis::Lateral)
    {
      item["axis"] = "lateral";
    } else if (
      constraint.axis ==
      problem::DynamicObstacleConstraintAxis::EffectiveProgress)
    {
      item["axis"] = "effective-progress";
    } else {
      item["axis"] = "coupled-lateral-progress";
    }
    item["lower"] = constraint.lower;
    item["upper"] = constraint.upper;
    item["lateral_coefficient"] = constraint.lateral_coefficient;
    item["effective_progress_coefficient"] =
      constraint.effective_progress_coefficient;
    if (constraint.physical_state_coefficients) {
      item["physical_state_coefficients"] = fixed_vector_node(*constraint.physical_state_coefficients);
    }
    obstacles.push_back(item);
  }
  node["dynamic_obstacle_constraints"] = obstacles;
  return node;
}

YAML::Node observation_provenance_node(const mpcc_vehicle_model::ObservationProvenance & value)
{
  return mpcc_vehicle_model::encode_observation_provenance(value);
}
std::optional<mpcc_vehicle_model::ObservationProvenance> load_observation_provenance(const YAML::Node & node)
{
  return mpcc_vehicle_model::decode_observation_provenance(node);
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
  node["current_lateral_velocity_mps"] = request.current_lateral_velocity_mps;
  node["current_yaw_rate_radps"] = request.current_yaw_rate_radps;
  node["vehicle_model"] = mpcc_vehicle_model::encode_parameters(request.vehicle_model);
  node["maximum_braking_feasibility"] = request.maximum_braking_feasibility;
  if (request.observation_provenance) {
    node["observation_provenance"] = observation_provenance_node(*request.observation_provenance);
  }
  node["wheelbase_m"] = request.wheelbase_m;
  node["curvature_reference_gain"] = request.curvature_reference_gain;
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
  node["rejected_primal_available"] = outcome.rejected_primal.has_value();
  if (outcome.rejected_primal.has_value()) {
    node["rejected_primal"] = vector_node(outcome.rejected_primal.value());
  }
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
  telemetry["feasibility_equalities_augmented"] =
    outcome.telemetry.feasibility_equalities_augmented;
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

YAML::Node problem_context_node(const contract::MpccProblemContext & context)
{
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
  problem_context["dynamic_obstacle_constraint_active"] =
    context.dynamic_obstacle_constraint_active;
  problem_context["dynamic_obstacle_generation"] =
    context.dynamic_obstacle_generation;
  problem_context["dynamic_obstacle_id"] = context.dynamic_obstacle_id;
  problem_context["dynamic_obstacle_side_sign"] =
    context.dynamic_obstacle_side_sign;
  problem_context["horizon_steps"] = context.horizon_steps;
  problem_context["formulation"] = contract::to_string(context.formulation);
  problem_context["state_schema_id"] = context.state_schema_id;
  problem_context["input_schema_id"] = context.input_schema_id;
  problem_context["bounds_schema_id"] = context.bounds_schema_id;
  problem_context["cost_schema_id"] = context.cost_schema_id;
  problem_context["fingerprint"] = context.fingerprint;
  problem_context["vehicle_model_fingerprint"] = context.vehicle_model_fingerprint;
  problem_context["applied_program_fingerprint"] = context.applied_program_fingerprint;
  return problem_context;
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
  node["problem_context"] = problem_context_node(context);
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
  node["terminal_intent_contract_active"] =
    source.terminal_intent_contract.active;
  if (source.terminal_intent_contract.active) {
    node["terminal_intent_lateral_reference_m"] =
      source.terminal_intent_contract.lateral_reference_m;
    node["terminal_intent_lateral_tolerance_m"] =
      source.terminal_intent_contract.lateral_tolerance_m;
    node["terminal_intent_heading_reference_rad"] =
      source.terminal_intent_contract.heading_reference_rad;
    node["terminal_intent_heading_tolerance_rad"] =
      source.terminal_intent_contract.heading_tolerance_rad;
  }
  node["dynamic_obstacle_refinement_active"] =
    source.dynamic_obstacle_refinement_active;
  node["dynamic_obstacle_pass_side_sign"] =
    source.dynamic_obstacle_pass_side_sign;
  node["dynamic_obstacle_longitudinal_topology"] =
    static_cast<int>(source.dynamic_obstacle_longitudinal_topology);
  node["dynamic_obstacle_forced_first_pass_side_stage"] =
    source.dynamic_obstacle_forced_first_pass_side_stage;
  node["dynamic_obstacle_forced_first_ahead_stage"] =
    source.dynamic_obstacle_forced_first_ahead_stage;
  node["dynamic_obstacle_forced_constraint_fraction"] =
    source.dynamic_obstacle_forced_constraint_fraction;
  node["dynamic_obstacle_forced_diagonal_start_stage"] =
    source.dynamic_obstacle_forced_diagonal_start_stage;
  node["dynamic_obstacle_forced_diagonal_full_side_stage"] =
    source.dynamic_obstacle_forced_diagonal_full_side_stage;
  node["dynamic_obstacle_forced_physical_diagonal"] =
    source.dynamic_obstacle_forced_physical_diagonal;
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
  if (source.terminal_stop_course_geometry) {
    const auto & support = *source.terminal_stop_course_geometry;
    auto saved = node["terminal_stop_course_geometry"];
    saved["progress_m"] = support.progress_m;
    saved["curvature_radpm"] = support.curvature_radpm;
    saved["lateral_lower_m"] = support.lateral_lower_m;
    saved["lateral_upper_m"] = support.lateral_upper_m;
  }
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
    replay_world["terminal_stop_contract_available"] =
      world.terminal_stop_contract_available;
    if (world.terminal_stop_contract_available) {
      const auto & policy = world.terminal_stop_lateral_policy;
      YAML::Node stop_policy;
      stop_policy["wheelbase_m"] = policy.wheelbase_m;
      stop_policy["maximum_abs_steering_rad"] =
        policy.maximum_abs_steering_rad;
      stop_policy["maximum_abs_steering_rate_radps"] =
        policy.maximum_abs_steering_rate_radps;
      stop_policy["maximum_lateral_acceleration_mps2"] =
        policy.maximum_lateral_acceleration_mps2;
      stop_policy["steering_command_gain"] = policy.steering_command_gain;
      stop_policy["lateral_gain"] = policy.lateral_gain;
      stop_policy["heading_gain"] = policy.heading_gain;
      replay_world["terminal_stop_lateral_policy"] = stop_policy;
      replay_world["terminal_stop_minimum_acceleration_mps2"] =
        world.terminal_stop_minimum_acceleration_mps2;
    }
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
      item["acceleration_horizon_sec"] = obstacle.acceleration_horizon_sec;
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

template<int Rows, int Columns>
bool load_fixed_matrix(
  const YAML::Node & node, Eigen::Matrix<double, Rows, Columns> & value)
{
  if (
    !node || !node.IsMap() || !node["rows"] || !node["columns"] ||
    !node["values"] || !node["values"].IsSequence() ||
    node["rows"].as<int>() != Rows ||
    node["columns"].as<int>() != Columns ||
    node["values"].size() != static_cast<std::size_t>(Rows))
  {
    return false;
  }
  for (int row = 0; row < Rows; ++row) {
    const auto entries = node["values"][static_cast<std::size_t>(row)];
    if (!entries.IsSequence() ||
      entries.size() != static_cast<std::size_t>(Columns))
    {
      return false;
    }
    for (int column = 0; column < Columns; ++column) {
      value(row, column) =
        entries[static_cast<std::size_t>(column)].as<double>();
    }
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

std::optional<std::vector<double>> load_std_bound_vector(
  const YAML::Node & node)
{
  if (!node || !node.IsSequence()) {
    return std::nullopt;
  }
  std::vector<double> values;
  values.reserve(node.size());
  for (const auto & item : node) {
    const double value = item.as<double>();
    if (std::isnan(value)) {
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

std::optional<problem::DynamicObstacleConstraintAxis>
parse_dynamic_obstacle_axis(const std::string & value)
{
  if (value == "lateral") {
    return problem::DynamicObstacleConstraintAxis::Lateral;
  }
  if (value == "effective-progress") {
    return problem::DynamicObstacleConstraintAxis::EffectiveProgress;
  }
  if (value == "coupled-lateral-progress") {
    return problem::DynamicObstacleConstraintAxis::CoupledLateralProgress;
  }
  return std::nullopt;
}

std::optional<problem::AssemblyRequest> load_assembly_request(
  const YAML::Node & node)
{
  namespace model = mpcc_rate_resolved;
  if (!node || !node.IsMap() || !node["horizon_steps"]) {
    return std::nullopt;
  }
  problem::AssemblyRequest request;
  request.horizon_steps = node["horizon_steps"].as<int>();
  auto state_reference = load_vector(node["state_reference"]);
  auto state_lower = load_vector(node["state_lower"]);
  auto state_upper = load_vector(node["state_upper"]);
  auto state_weight = load_vector(node["state_weight"]);
  auto input_reference = load_vector(node["input_reference"]);
  auto input_lower = load_vector(node["input_lower"]);
  auto input_upper = load_vector(node["input_upper"]);
  auto input_weight = load_vector(node["input_weight"]);
  auto additional_linear_cost = load_vector(node["additional_linear_cost"]);
  if (
    request.horizon_steps <= 0 ||
    !load_fixed_vector(node["initial_state"], request.initial_state) ||
    !load_fixed_vector(node["previous_input"], request.previous_input) ||
    !load_fixed_vector(
      node["input_delta_weight"], request.input_delta_weight) ||
    !state_reference || !state_lower || !state_upper || !state_weight ||
    !input_reference || !input_lower || !input_upper || !input_weight ||
    !additional_linear_cost)
  {
    return std::nullopt;
  }
  request.state_reference = std::move(state_reference.value());
  request.state_lower = std::move(state_lower.value());
  request.state_upper = std::move(state_upper.value());
  request.state_weight = std::move(state_weight.value());
  request.input_reference = std::move(input_reference.value());
  request.input_lower = std::move(input_lower.value());
  request.input_upper = std::move(input_upper.value());
  request.input_weight = std::move(input_weight.value());
  request.additional_linear_cost =
    std::move(additional_linear_cost.value());

  const auto linearizations = node["linearizations"];
  if (!linearizations || !linearizations.IsSequence()) {
    return std::nullopt;
  }
  request.linearizations.reserve(linearizations.size());
  for (const auto & item : linearizations) {
    model::Linearization linearization;
    if (
      !load_fixed_matrix(
        item["state_matrix"], linearization.state_matrix) ||
      !load_fixed_matrix(
        item["input_matrix"], linearization.input_matrix) ||
      !load_fixed_vector(
        item["equality_offset"], linearization.equality_offset))
    {
      return std::nullopt;
    }
    linearization.stage_dt_sec = item["stage_dt_sec"].as<double>();
    if (!std::isfinite(linearization.stage_dt_sec)) {
      return std::nullopt;
    }
    request.linearizations.push_back(std::move(linearization));
  }

  if (node["steering_rate_prefix_bounds"]) {
    const auto bounds = node["steering_rate_prefix_bounds"];
    request.steering_rate_prefix_bounds =
      problem::SteeringRatePrefixBounds{
      bounds["minimum_cumulative_delta_rad"].as<double>(),
      bounds["maximum_cumulative_delta_rad"].as<double>()};
  }
  if (node["progress_aligned_wall_constraints"]) {
    const auto wall = node["progress_aligned_wall_constraints"];
    auto lower_slope = load_std_vector(wall["lower_slope"]);
    auto lower_intercept = load_std_bound_vector(wall["lower_intercept"]);
    auto upper_slope = load_std_vector(wall["upper_slope"]);
    auto upper_intercept = load_std_bound_vector(wall["upper_intercept"]);
    if (!lower_slope || !lower_intercept || !upper_slope || !upper_intercept) {
      return std::nullopt;
    }
    request.progress_aligned_wall_constraints =
      problem::ProgressAlignedWallConstraints{
      std::move(lower_slope.value()), std::move(lower_intercept.value()),
      std::move(upper_slope.value()), std::move(upper_intercept.value())};
  }

  const auto swept = node["swept_lateral_wall_constraints"];
  const auto obstacles = node["dynamic_obstacle_constraints"];
  if (!swept || !swept.IsSequence() ||
    !obstacles || !obstacles.IsSequence())
  {
    return std::nullopt;
  }
  request.swept_lateral_wall_constraints.reserve(swept.size());
  for (const auto & item : swept) {
    request.swept_lateral_wall_constraints.push_back(
      problem::SweptLateralWallConstraint{
        item["transition_stage"].as<int>(),
        item["destination_ratio"].as<double>(),
        item["lower_m"].as<double>(), item["upper_m"].as<double>()});
  }
  request.dynamic_obstacle_constraints.reserve(obstacles.size());
  for (const auto & item : obstacles) {
    const auto axis = parse_dynamic_obstacle_axis(
      item["axis"].as<std::string>());
    if (!axis.has_value()) {
      return std::nullopt;
    }
    request.dynamic_obstacle_constraints.push_back(
      problem::DynamicObstacleConstraint{
        item["state_stage"].as<int>(), axis.value(),
        item["lower"].as<double>(), item["upper"].as<double>(),
        item["lateral_coefficient"].as<double>(),
        item["effective_progress_coefficient"].as<double>()});
    if (item["physical_state_coefficients"]) {
      const auto coefficients = load_vector(item["physical_state_coefficients"]);
      if (!coefficients || coefficients->size() != mpcc_rate_resolved::kStateDimension ||
        !coefficients->allFinite() || coefficients->isZero(0.0))
      {
        return std::nullopt;
      }
      request.dynamic_obstacle_constraints.back().physical_state_coefficients = *coefficients;
    }
  }
  return request;
}

std::optional<contract::Formulation> parse_formulation(
  const std::string & value)
{
  using Formulation = contract::Formulation;
  if (value == "velocity-steering-yaw-response-progress-7state") {
    return Formulation::VelocitySteeringYawResponseProgress7State;
  }
  if (value == "velocity-steering-tire-body-progress-9state") {
    return Formulation::VelocitySteeringTireBodyProgress9State;
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
  const auto vehicle = mpcc_vehicle_model::decode_parameters(node["vehicle_model"]);
  if (!vehicle) return std::nullopt;
  request.vehicle_model = *vehicle;
  request.maximum_braking_feasibility = node["maximum_braking_feasibility"].as<bool>(false);
  if (node["observation_provenance"]) {
    request.observation_provenance = load_observation_provenance(node["observation_provenance"]);
    if (!request.observation_provenance) return std::nullopt;
  }
  request.current_lateral_velocity_mps = node["current_lateral_velocity_mps"].as<double>();
  request.current_yaw_rate_radps = node["current_yaw_rate_radps"].as<double>();
  request.wheelbase_m = node["wheelbase_m"].as<double>();
  request.curvature_reference_gain = node["curvature_reference_gain"].as<double>();
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
  if (problem_context["dynamic_obstacle_constraint_active"]) {
    context.dynamic_obstacle_constraint_active =
      problem_context["dynamic_obstacle_constraint_active"].as<bool>();
    context.dynamic_obstacle_generation =
      problem_context["dynamic_obstacle_generation"].as<std::uint64_t>();
    context.dynamic_obstacle_id =
      problem_context["dynamic_obstacle_id"].as<std::string>();
    context.dynamic_obstacle_side_sign =
      problem_context["dynamic_obstacle_side_sign"].as<int>();
  }
  context.horizon_steps = problem_context["horizon_steps"].as<std::size_t>();
  context.formulation = formulation.value();
  context.state_schema_id = problem_context["state_schema_id"].as<std::string>();
  context.input_schema_id = problem_context["input_schema_id"].as<std::string>();
  context.bounds_schema_id = problem_context["bounds_schema_id"].as<std::string>();
  context.cost_schema_id = problem_context["cost_schema_id"].as<std::string>();
  context.fingerprint = problem_context["fingerprint"].as<std::uint64_t>();
  context.vehicle_model_fingerprint = problem_context["vehicle_model_fingerprint"].as<std::uint64_t>(0U);
  context.applied_program_fingerprint = problem_context["applied_program_fingerprint"].as<std::uint64_t>(0U);
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
  if (node["terminal_intent_contract_active"]) {
    source.terminal_intent_contract.active =
      node["terminal_intent_contract_active"].as<bool>();
  }
  if (source.terminal_intent_contract.active) {
    if (!node["terminal_intent_lateral_reference_m"] ||
      !node["terminal_intent_lateral_tolerance_m"] ||
      !node["terminal_intent_heading_reference_rad"] ||
      !node["terminal_intent_heading_tolerance_rad"])
    {
      return std::nullopt;
    }
    source.terminal_intent_contract.lateral_reference_m =
      node["terminal_intent_lateral_reference_m"].as<double>();
    source.terminal_intent_contract.lateral_tolerance_m =
      node["terminal_intent_lateral_tolerance_m"].as<double>();
    source.terminal_intent_contract.heading_reference_rad =
      node["terminal_intent_heading_reference_rad"].as<double>();
    source.terminal_intent_contract.heading_tolerance_rad =
      node["terminal_intent_heading_tolerance_rad"].as<double>();
  }
  source.dynamic_obstacle_refinement_active =
    node["dynamic_obstacle_refinement_active"].as<bool>();
  source.dynamic_obstacle_pass_side_sign =
    node["dynamic_obstacle_pass_side_sign"].as<int>();
  if (node["dynamic_obstacle_longitudinal_topology"]) {
    const int topology =
      node["dynamic_obstacle_longitudinal_topology"].as<int>();
    if (topology == 0) {
      source.dynamic_obstacle_longitudinal_topology =
        mpcc_rate_resolved_dynamic_obstacle::LongitudinalTopology::Automatic;
    } else if (topology == 1) {
      source.dynamic_obstacle_longitudinal_topology =
        mpcc_rate_resolved_dynamic_obstacle::LongitudinalTopology::StayBehind;
    } else if (topology == 2) {
      source.dynamic_obstacle_longitudinal_topology =
        mpcc_rate_resolved_dynamic_obstacle::LongitudinalTopology::StayAhead;
    } else {
      return std::nullopt;
    }
  }
  if (node["dynamic_obstacle_forced_first_pass_side_stage"]) {
    source.dynamic_obstacle_forced_first_pass_side_stage =
      node["dynamic_obstacle_forced_first_pass_side_stage"].as<int>();
  }
  if (node["dynamic_obstacle_forced_first_ahead_stage"]) {
    source.dynamic_obstacle_forced_first_ahead_stage =
      node["dynamic_obstacle_forced_first_ahead_stage"].as<int>();
  }
  if (node["dynamic_obstacle_forced_constraint_fraction"]) {
    source.dynamic_obstacle_forced_constraint_fraction =
      node["dynamic_obstacle_forced_constraint_fraction"].as<double>();
  }
  if (node["dynamic_obstacle_forced_diagonal_start_stage"]) {
    source.dynamic_obstacle_forced_diagonal_start_stage =
      node["dynamic_obstacle_forced_diagonal_start_stage"].as<int>();
  }
  if (node["dynamic_obstacle_forced_diagonal_full_side_stage"]) {
    source.dynamic_obstacle_forced_diagonal_full_side_stage =
      node["dynamic_obstacle_forced_diagonal_full_side_stage"].as<int>();
  }
  if (node["dynamic_obstacle_forced_physical_diagonal"]) {
    source.dynamic_obstacle_forced_physical_diagonal =
      node["dynamic_obstacle_forced_physical_diagonal"].as<bool>();
  }
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
  if (const auto saved = node["terminal_stop_course_geometry"]) {
    source.terminal_stop_course_geometry =
      mpcc_rate_resolved_physical_adapter::StopCourseGeometry{
      saved["progress_m"].as<std::vector<double>>(),
      saved["curvature_radpm"].as<std::vector<double>>(),
      saved["lateral_lower_m"].as<std::vector<double>>(),
      saved["lateral_upper_m"].as<std::vector<double>>()};
  }
  if (!source.wall_course_frame_knots.empty()) {
    source.request.course_frame = {
      std::make_shared<const std::vector<mpc_stage_geometry::CourseFrameKnot>>(
        source.wall_course_frame_knots), source.course_progress_origin_m};
  }
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
    world.terminal_stop_contract_available =
      replay["terminal_stop_contract_available"] &&
      replay["terminal_stop_contract_available"].as<bool>();
    if (world.terminal_stop_contract_available) {
      const auto policy = replay["terminal_stop_lateral_policy"];
      if (!policy || !policy.IsMap() ||
        !replay["terminal_stop_minimum_acceleration_mps2"])
      {
        return std::nullopt;
      }
      world.terminal_stop_lateral_policy =
        race_mpcc_foundation::StopPathTrackingPolicy{
        policy["wheelbase_m"].as<double>(),
        policy["maximum_abs_steering_rad"].as<double>(),
        policy["maximum_abs_steering_rate_radps"].as<double>(),
        policy["maximum_lateral_acceleration_mps2"].as<double>(),
        policy["steering_command_gain"].as<double>(),
        policy["lateral_gain"].as<double>(),
        policy["heading_gain"].as<double>()};
      world.terminal_stop_minimum_acceleration_mps2 =
        replay["terminal_stop_minimum_acceleration_mps2"].as<double>();
    }
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
        item["observation_generation"].as<std::uint64_t>(),
        item["acceleration_horizon_sec"] ? item["acceleration_horizon_sec"].as<double>() : 0.0});
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

int physical_homotopy_side(const shadow::Snapshot & source) noexcept
{
  const auto & context = source.identity.source_context;
  return context.execution_side_sign != 0 ? context.execution_side_sign :
         source.dynamic_obstacle_pass_side_sign;
}

const char * physical_homotopy_component(const int side_sign) noexcept
{
  return side_sign > 0 ? "side-positive" :
         side_sign < 0 ? "side-negative" : "side-neutral";
}

std::string failure_key(
  const shadow::Snapshot & source, const PipelineStage stage,
  const std::string & failure_outcome)
{
  return std::string{contract::to_string(source.identity.source_context.intent)} +
    "|side=" + std::to_string(physical_homotopy_side(source)) + '|' +
    to_string(stage) + '|' + failure_outcome;
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
    case RecordStatus::UnsupportedIntent: return "unsupported-intent";
    case RecordStatus::InvalidInput: return "invalid-input";
    case RecordStatus::IoFailure: return "io-failure";
  }
  return "invalid-input";
}

bool interaction_snapshot_complete(const shadow::Snapshot & source) noexcept
{
  try {
    const auto & request = source.request;
    const bool return_intent = source.identity.source_context.intent ==
      contract::ControlIntent::Return;
    const auto & terminal_contract = source.terminal_intent_contract;
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
      !std::isfinite(request.current_lateral_velocity_mps) ||
      !std::isfinite(request.current_yaw_rate_radps) ||
      !mpcc_vehicle_model::valid(request.vehicle_model) ||
      (request.observation_provenance &&
      !mpcc_vehicle_model::valid(*request.observation_provenance)) ||
      source.identity.source_context.vehicle_model_fingerprint !=
      mpcc_vehicle_model::fingerprint(request.vehicle_model) ||
      !std::isfinite(request.wheelbase_m) || request.wheelbase_m <= 0.0 ||
      !std::isfinite(request.curvature_reference_gain) ||
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
      static_cast<std::size_t>(request.horizon_steps + 1) ||
      return_intent != terminal_contract.active ||
      (terminal_contract.active &&
      (!std::isfinite(terminal_contract.lateral_reference_m) ||
      !std::isfinite(terminal_contract.lateral_tolerance_m) ||
      terminal_contract.lateral_tolerance_m < 0.0 ||
      !std::isfinite(terminal_contract.heading_reference_rad) ||
      !std::isfinite(terminal_contract.heading_tolerance_rad) ||
      terminal_contract.heading_tolerance_rad < 0.0)))
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
    const auto & problem_context = source.identity.source_context;
    if (
      source.dynamic_obstacle_refinement_active !=
      problem_context.dynamic_obstacle_constraint_active ||
      (source.dynamic_obstacle_refinement_active &&
      source.dynamic_obstacle_pass_side_sign !=
      problem_context.dynamic_obstacle_side_sign))
    {
      return false;
    }
    const bool forced_longitudinal =
      source.dynamic_obstacle_longitudinal_topology !=
      mpcc_rate_resolved_dynamic_obstacle::LongitudinalTopology::Automatic;
    if (
      (source.dynamic_obstacle_longitudinal_topology !=
      mpcc_rate_resolved_dynamic_obstacle::LongitudinalTopology::Automatic &&
      source.dynamic_obstacle_longitudinal_topology !=
      mpcc_rate_resolved_dynamic_obstacle::LongitudinalTopology::StayBehind &&
      source.dynamic_obstacle_longitudinal_topology !=
      mpcc_rate_resolved_dynamic_obstacle::LongitudinalTopology::StayAhead) ||
      (forced_longitudinal &&
      (!source.dynamic_obstacle_refinement_active ||
      source.dynamic_obstacle_pass_side_sign != 0 ||
      source.dynamic_obstacle_forced_first_pass_side_stage >= 0 ||
      source.dynamic_obstacle_forced_first_ahead_stage >= 0 ||
      source.dynamic_obstacle_forced_diagonal_start_stage >= 0 ||
      source.dynamic_obstacle_forced_diagonal_full_side_stage >= 0 ||
      source.dynamic_obstacle_forced_physical_diagonal)))
    {
      return false;
    }
    if (
      source.dynamic_obstacle_forced_first_pass_side_stage < -1 ||
      source.dynamic_obstacle_forced_first_pass_side_stage >=
      request.horizon_steps ||
      (source.dynamic_obstacle_forced_first_pass_side_stage >= 0 &&
      (!source.dynamic_obstacle_refinement_active ||
      source.dynamic_obstacle_pass_side_sign == 0)))
    {
      return false;
    }
    if (
      source.dynamic_obstacle_forced_first_ahead_stage < -1 ||
      source.dynamic_obstacle_forced_first_ahead_stage >
      request.horizon_steps ||
      (source.dynamic_obstacle_forced_first_ahead_stage >= 0 &&
      (source.dynamic_obstacle_forced_first_pass_side_stage < 0 ||
      source.dynamic_obstacle_forced_first_ahead_stage <=
      source.dynamic_obstacle_forced_first_pass_side_stage)))
    {
      return false;
    }
    if (
      !std::isfinite(source.dynamic_obstacle_forced_constraint_fraction) ||
      source.dynamic_obstacle_forced_constraint_fraction < 0.0 ||
      source.dynamic_obstacle_forced_constraint_fraction > 1.0 ||
      (source.dynamic_obstacle_forced_first_pass_side_stage < 0 &&
      source.dynamic_obstacle_forced_constraint_fraction != 1.0))
    {
      return false;
    }
    const bool diagonal_start_present =
      source.dynamic_obstacle_forced_diagonal_start_stage >= 0;
    const bool diagonal_full_present =
      source.dynamic_obstacle_forced_diagonal_full_side_stage >= 0;
    if (
      source.dynamic_obstacle_forced_diagonal_start_stage < -1 ||
      source.dynamic_obstacle_forced_diagonal_full_side_stage < -1 ||
      diagonal_start_present != diagonal_full_present ||
      (diagonal_start_present &&
      (!source.dynamic_obstacle_refinement_active ||
      source.dynamic_obstacle_pass_side_sign == 0 ||
      source.dynamic_obstacle_forced_first_pass_side_stage >= 0 ||
      source.dynamic_obstacle_forced_diagonal_full_side_stage >=
      request.horizon_steps ||
      source.dynamic_obstacle_forced_diagonal_full_side_stage <
      source.dynamic_obstacle_forced_diagonal_start_stage + 2)))
    {
      return false;
    }
    if (
      source.dynamic_obstacle_forced_physical_diagonal &&
      !diagonal_start_present)
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
    if (source.terminal_stop_course_geometry) {
      const auto & support = *source.terminal_stop_course_geometry;
      if (!mpcc_rate_resolved_physical_adapter::stop_course_geometry_valid(support) ||
        support.progress_m.front() + source.course_progress_origin_m <
        source.wall_course_frame_knots.front().progress_m - 1e-9 ||
        support.progress_m.back() + source.course_progress_origin_m >
        source.wall_course_frame_knots.back().progress_m + 1e-9)
      {
        return false;
      }
    }
    const auto & world = source.replay_world.value();
    if (
      !world.current || world.observation_generation == 0U ||
      (problem_context.dynamic_obstacle_constraint_active &&
      world.observation_generation !=
      problem_context.dynamic_obstacle_generation) ||
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
    if (world.terminal_stop_contract_available) {
      const auto & policy = world.terminal_stop_lateral_policy;
      if (
        !std::isfinite(policy.wheelbase_m) || policy.wheelbase_m <= 0.0 ||
        !std::isfinite(policy.maximum_abs_steering_rad) ||
        policy.maximum_abs_steering_rad <= 0.0 ||
        !std::isfinite(policy.maximum_abs_steering_rate_radps) ||
        policy.maximum_abs_steering_rate_radps <= 0.0 ||
        !std::isfinite(policy.maximum_lateral_acceleration_mps2) ||
        policy.maximum_lateral_acceleration_mps2 <= 0.0 ||
        !std::isfinite(policy.steering_command_gain) ||
        policy.steering_command_gain <= 0.0 ||
        !std::isfinite(policy.lateral_gain) ||
        !std::isfinite(policy.heading_gain) ||
        !std::isfinite(world.terminal_stop_minimum_acceleration_mps2) ||
        world.terminal_stop_minimum_acceleration_mps2 >= 0.0)
      {
        return false;
      }
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
    bool target_present =
      !problem_context.dynamic_obstacle_constraint_active;
    for (const auto & obstacle : world.obstacles) {
      if (
        obstacle.id.empty() || !obstacle_ids.insert(obstacle.id).second ||
        obstacle.observation_generation != world.observation_generation ||
        !std::isfinite(obstacle.x_m) || !std::isfinite(obstacle.y_m) ||
        !std::isfinite(obstacle.velocity_x_mps) ||
        !std::isfinite(obstacle.velocity_y_mps) ||
        !std::isfinite(obstacle.acceleration_x_mps2) ||
        !std::isfinite(obstacle.acceleration_y_mps2) ||
        !std::isfinite(obstacle.acceleration_horizon_sec) || obstacle.acceleration_horizon_sec < 0.0 ||
        !std::isfinite(obstacle.covariance_x_m2) ||
        obstacle.covariance_x_m2 < 0.0 ||
        !std::isfinite(obstacle.covariance_y_m2) ||
        obstacle.covariance_y_m2 < 0.0 ||
        !std::isfinite(obstacle.radius_m) || obstacle.radius_m <= 0.0)
      {
        return false;
      }
      target_present = target_present ||
        obstacle.id == problem_context.dynamic_obstacle_id;
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
  builder.append_string(
    source.replay_world->terminal_stop_contract_available ?
    "mpcc-interaction-snapshot-v3" : "mpcc-interaction-snapshot-v2");
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
  builder.append_double(request.current_lateral_velocity_mps);
  builder.append_double(request.current_yaw_rate_radps);
  builder.append_u64(mpcc_vehicle_model::fingerprint(request.vehicle_model));
  builder.append_bool(request.maximum_braking_feasibility);
  builder.append_bool(request.observation_provenance.has_value());
  if (request.observation_provenance) {
    builder.append_string(YAML::Dump(observation_provenance_node(*request.observation_provenance)));
  }
  builder.append_double(request.wheelbase_m);
  builder.append_double(request.curvature_reference_gain);
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
  if (source.terminal_intent_contract.active) {
    builder.append_u64(0x5445524d494e414cULL);
    builder.append_double(source.terminal_intent_contract.lateral_reference_m);
    builder.append_double(source.terminal_intent_contract.lateral_tolerance_m);
    builder.append_double(source.terminal_intent_contract.heading_reference_rad);
    builder.append_double(source.terminal_intent_contract.heading_tolerance_rad);
  }
  builder.append_bool(source.dynamic_obstacle_refinement_active);
  builder.append_i64(source.dynamic_obstacle_pass_side_sign);
  if (source.dynamic_obstacle_longitudinal_topology !=
    mpcc_rate_resolved_dynamic_obstacle::LongitudinalTopology::Automatic)
  {
    builder.append_u64(0x4c4f4e47544f504fULL);
    builder.append_i64(
      static_cast<std::int64_t>(
        source.dynamic_obstacle_longitudinal_topology));
  }
  // Preserve fingerprints of every frozen production snapshot.  The optional
  // lattice member is appended only when candidate C explicitly selects one.
  if (source.dynamic_obstacle_forced_first_pass_side_stage >= 0) {
    builder.append_bool(true);
    builder.append_i64(source.dynamic_obstacle_forced_first_pass_side_stage);
    builder.append_i64(source.dynamic_obstacle_forced_first_ahead_stage);
    builder.append_double(
      source.dynamic_obstacle_forced_constraint_fraction);
  }
  if (source.dynamic_obstacle_forced_diagonal_start_stage >= 0) {
    builder.append_u64(0x444941474f4e414cULL);
    builder.append_i64(source.dynamic_obstacle_forced_diagonal_start_stage);
    builder.append_i64(
      source.dynamic_obstacle_forced_diagonal_full_side_stage);
  }
  if (source.dynamic_obstacle_forced_physical_diagonal) {
    builder.append_u64(0x5048595344494147ULL);
  }
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
  if (source.terminal_stop_course_geometry) {
    builder.append_string("terminal-stop-map-support-v1");
    const auto & support = *source.terminal_stop_course_geometry;
    for (const auto * values : {&support.progress_m, &support.curvature_radpm,
      &support.lateral_lower_m, &support.lateral_upper_m})
    {
      builder.append_u64(values->size());
      for (const auto value : *values) builder.append_double(value);
    }
  }
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
  if (world.terminal_stop_contract_available) {
    const auto & policy = world.terminal_stop_lateral_policy;
    builder.append_u64(0x53544f5053554646ULL);
    builder.append_double(policy.wheelbase_m);
    builder.append_double(policy.maximum_abs_steering_rad);
    builder.append_double(policy.maximum_abs_steering_rate_radps);
    builder.append_double(policy.maximum_lateral_acceleration_mps2);
    builder.append_double(policy.steering_command_gain);
    builder.append_double(policy.lateral_gain);
    builder.append_double(policy.heading_gain);
    builder.append_double(world.terminal_stop_minimum_acceleration_mps2);
  }
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
    // Preserve old CV recording fingerprints, including their unused acceleration.
    if (obstacle.acceleration_horizon_sec != 0.0) {
      builder.append_string("finite-acceleration-cartesian-peer-v1");
      builder.append_double(obstacle.acceleration_horizon_sec);
    }
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

YAML::Node execution_evidence_node(
  const mpcc_rate_resolved_execution_artifact::ExecutionArtifact & value,
  const PublicationEvidence & publication)
{
  YAML::Node node;
  node["schema"] = "mpcc-published-execution-evidence/v1";
  // Identity is exactly the enclosing source identity (checked before writing).
  node["source_sequence"] = value.identity.sequence;
  node["source_problem_fingerprint"] = value.identity.source_context.fingerprint;
  node["source_snapshot_sec"] = value.identity.snapshot_sec;
  node["coordinate_state_schema"] = value.identity.source_context.state_schema_id;
  node["source_course_frame_bound"] = static_cast<bool>(value.course_frame.knots);
  node["course_frame_progress_origin_m"] = value.course_frame.progress_origin_m;
  node["prediction_origin_sec"] = value.prediction_origin_sec;
  node["publication_interval_sec"] = value.publication_interval_sec;
  node["completed_sec"] = value.completed_sec;
  node["course_progress_origin_m"] = value.course_progress_origin_m;
  node["semantic_initial_steering_rad"] = value.semantic_initial_steering_rad;
  node["semantic_initial_response_steering_rad"] = value.semantic_initial_response_steering_rad;
  if (value.applied_stop_program) node["applied_stop_program"] =
    mpcc_vehicle_model::encode_applied_program_provenance(*value.applied_stop_program);
  const auto & initial = value.semantic_initial_state.value();
  node["semantic_initial_state"]["lateral_m"] = initial.lateral_m;
  node["semantic_initial_state"]["lag_m"] = initial.lag_m;
  node["semantic_initial_state"]["heading_offset_rad"] = initial.heading_offset_rad;
  node["semantic_initial_state"]["velocity_mps"] = initial.velocity_mps;
  node["semantic_initial_state"]["progress_m"] = initial.progress_m;
  node["semantic_initial_state"]["steering_rad"] = initial.steering_rad;
  node["semantic_initial_state"]["response_steering_rad"] = initial.response_steering_rad;
  node["semantic_initial_state"]["lateral_velocity_mps"] = initial.lateral_velocity_mps;
  node["semantic_initial_state"]["yaw_rate_radps"] = initial.yaw_rate_radps;
  node["vehicle_model"] = mpcc_vehicle_model::encode_parameters(value.vehicle_model);
  node["wheelbase_m"] = value.wheelbase_m;
  node["minimum_frenet_denominator"] = value.minimum_frenet_denominator;
  node["maximum_abs_steering_rad"] = value.maximum_abs_steering_rad;
  node["maximum_abs_steering_rate_radps"] = value.maximum_abs_steering_rate_radps;
  node["physical_global_tolerance"] = value.physical_global_tolerance;
  node["terminal_body_rest_required"] = value.terminal_body_rest_required;
  node["maximum_constraint_violation"] = value.maximum_constraint_violation;
  node["maximum_normalized_constraint_violation"] = value.maximum_normalized_constraint_violation;
  node["terminal_intent_contract"]["active"] =
    value.terminal_intent_contract.active;
  node["terminal_intent_contract"]["lateral_reference_m"] =
    value.terminal_intent_contract.lateral_reference_m;
  node["terminal_intent_contract"]["lateral_tolerance_m"] =
    value.terminal_intent_contract.lateral_tolerance_m;
  node["terminal_intent_contract"]["heading_reference_rad"] =
    value.terminal_intent_contract.heading_reference_rad;
  node["terminal_intent_contract"]["heading_tolerance_rad"] =
    value.terminal_intent_contract.heading_tolerance_rad;
  node["terminal_intent_certificate"]["active"] =
    value.terminal_intent_certificate.active;
  node["terminal_intent_certificate"]["solved_horizon_steps"] =
    value.terminal_intent_certificate.solved_horizon_steps;
  node["terminal_intent_certificate"]["solved_lateral_m"] =
    value.terminal_intent_certificate.solved_lateral_m;
  node["terminal_intent_certificate"]["solved_heading_rad"] =
    value.terminal_intent_certificate.solved_heading_rad;
  node["predicted_states"] = YAML::Node(YAML::NodeType::Sequence);
  for (const auto & item : value.predicted_states) {
    YAML::Node entry;
    entry["lateral_m"] = item.lateral_m;
    entry["lag_m"] = item.lag_m;
    entry["heading_offset_rad"] = item.heading_offset_rad;
    entry["velocity_mps"] = item.velocity_mps;
    entry["progress_m"] = item.progress_m;
    entry["steering_rad"] = item.steering_rad;
    entry["response_steering_rad"] = item.response_steering_rad;
    entry["lateral_velocity_mps"] = item.lateral_velocity_mps;
    entry["yaw_rate_radps"] = item.yaw_rate_radps;
    node["predicted_states"].push_back(entry);
  }
  node["control_stages"] = YAML::Node(YAML::NodeType::Sequence);
  for (const auto & item : value.control_stages) {
    YAML::Node entry;
    entry["acceleration_mps2"] = item.acceleration_mps2;
    entry["steering_rate_radps"] = item.steering_rate_radps;
    entry["virtual_progress_speed_mps"] = item.virtual_progress_speed_mps;
    entry["duration_sec"] = item.duration_sec;
    entry["virtual_progress_lower_mps"] = item.virtual_progress_lower_mps;
    entry["virtual_progress_upper_mps"] = item.virtual_progress_upper_mps;
    entry["acceleration_lower_mps2"] = item.acceleration_lower_mps2;
    entry["acceleration_upper_mps2"] = item.acceleration_upper_mps2;
    entry["path_curvature_radpm"] = item.path_curvature_radpm;
    node["control_stages"].push_back(entry);
  }
  node["nominal_path_distance_m"] = value.nominal_path_distance_m;
  node["lateral_lower_m"] = value.lateral_lower_m;
  node["lateral_upper_m"] = value.lateral_upper_m;
  node["publication"]["failure_decision_id"] =
    publication.failure_decision_id;
  node["publication"]["failure_interaction_fingerprint"] =
    publication.failure_interaction_fingerprint;
  node["publication"]["failure_observation_sec"] =
    publication.failure_observation_sec;
  node["publication"]["failure_control_origin_sec"] =
    publication.failure_control_origin_sec;
  node["publication"]["source_kind"] =
    publication.source_kind;
  node["publication"]["publication_decision_id"] =
    publication.publication_decision_id;
  node["publication"]["publication_control_origin_sec"] =
    publication.publication_control_origin_sec;
  node["publication"]["publication_artifact_elapsed_sec"] =
    publication.publication_artifact_elapsed_sec;
  return node;
}

static YAML::Node course_frame_observation_node(
  const std::vector<mpc_stage_geometry::CourseFrameKnot> & knots)
{
  YAML::Node output(YAML::NodeType::Sequence);
  for (const auto & knot : knots) {
    YAML::Node item;
    item["progress_m"] = knot.progress_m;
    item["x_m"] = knot.x_m; item["y_m"] = knot.y_m;
    item["heading_rad"] = knot.heading_rad; item["waypoint"] = knot.waypoint;
    output.push_back(item);
  }
  return output;
}

static YAML::Node certified_plan_evidence_node(
  const std::shared_ptr<const mpcc_rate_resolved_certified_plan::CertifiedPlan> & plan,
  const std::string & payload,
  const recovery_footprint::OccupancyGrid * & physical_grid)
{
  namespace certified = mpcc_rate_resolved_certified_plan;
  YAML::Node node;
  node["schema"] = "mpcc-certified-plan-observation/v1";
  node["authority"] = false;
  if (!plan) {
    node["status"] = "missing";
    node["reason"] = "certified plan unavailable";
    return node;
  }
  const auto validity = certified::validate(*plan);
  if (validity != certified::RejectReason::None) {
    node["status"] = "invalid";
    node["reason"] = certified::to_string(validity);
    return node;
  }
  node["status"] = "present";
  node["reason"] = "immutable artifact and its accepted physical evidence; no solver-source inference";
  node["solver_source_status"] = plan->solver_source_snapshot ? "present" : "absent";
  const auto & artifact = *plan->execution_artifact;
  auto value = execution_evidence_node(artifact, {});
  value.remove("publication");
  value["schema"] = "mpcc-inspected-execution-artifact/v1";
  value["problem_context"] = problem_context_node(artifact.identity.source_context);
  value["course_frame_knots"] = artifact.course_frame.knots ?
    course_frame_observation_node(*artifact.course_frame.knots) :
    YAML::Node(YAML::NodeType::Sequence);
  node["artifact"] = value;
  const auto identity_node = [](const mpcc_rate_resolved_physical_wall::Identity & identity) {
      YAML::Node value;
      value["artifact_sequence"] = identity.artifact.sequence;
      value["artifact_problem_fingerprint"] = identity.artifact.source_context.fingerprint;
      value["artifact_snapshot_sec"] = identity.artifact.snapshot_sec;
      value["pose_snapshot_id"] = identity.pose_snapshot_id;
      value["course_frame_window_id"] = identity.course_frame_window_id;
      value["captured_sec"] = identity.captured_sec;
      return value;
    };
  const auto pose_node = [](const recovery_footprint::Pose2D & pose) {
      YAML::Node value;
      value["x_m"] = pose.x_m; value["y_m"] = pose.y_m; value["yaw_rad"] = pose.yaw_rad;
      return value;
    };
  const auto & physical = *plan->physical_snapshot;
  auto snapshot = node["physical_snapshot"];
  snapshot["identity"] = identity_node(physical.identity);
  snapshot["current_pose"] = pose_node(physical.current_pose);
  snapshot["control_prefix"] = YAML::Node(YAML::NodeType::Sequence);
  for (const auto & pose : physical.control_prefix) {
    snapshot["control_prefix"].push_back(pose_node(pose));
  }
  snapshot["course_frame_knots"] = course_frame_observation_node(physical.course_frame_knots);
  snapshot["footprint"]["front_extent_m"] = physical.footprint.front_extent_m;
  snapshot["footprint"]["rear_extent_m"] = physical.footprint.rear_extent_m;
  snapshot["footprint"]["left_extent_m"] = physical.footprint.left_extent_m;
  snapshot["footprint"]["right_extent_m"] = physical.footprint.right_extent_m;
  snapshot["footprint"]["margin_m"] = physical.footprint.margin_m;
  snapshot["trajectory"]["progress_origin_m"] = physical.trajectory.progress_origin_m;
  snapshot["trajectory"]["elapsed_time_sec"] = physical.trajectory.elapsed_time_sec;
  snapshot["trajectory"]["path_distance_m"] = physical.trajectory.path_distance_m;
  snapshot["trajectory"]["lateral_m"] = physical.trajectory.lateral_m;
  snapshot["trajectory"]["lag_m"] = physical.trajectory.lag_m;
  snapshot["trajectory"]["heading_offset_rad"] = physical.trajectory.heading_offset_rad;
  snapshot["trajectory"]["velocity_mps"] = physical.trajectory.velocity_mps;
  snapshot["trajectory"]["progress_m"] = physical.trajectory.progress_m;
  snapshot["trajectory"]["lateral_lower_m"] = physical.trajectory.lateral_lower_m;
  snapshot["trajectory"]["lateral_upper_m"] = physical.trajectory.lateral_upper_m;
  snapshot["trajectory"]["minimum_lateral_bound_reserve_m"] = physical.trajectory.minimum_lateral_bound_reserve_m;
  snapshot["trajectory"]["progress_regression_tolerance_m"] = physical.trajectory.progress_regression_tolerance_m;
  snapshot["trajectory"]["velocity_lower_bound_tolerance_mps"] = physical.trajectory.velocity_lower_bound_tolerance_mps;
  snapshot["trajectory"]["stationary_path_suffix_allowed"] = physical.trajectory.stationary_path_suffix_allowed;
  snapshot["trajectory"]["stationary_velocity_tolerance_mps"] = physical.trajectory.stationary_velocity_tolerance_mps;
  snapshot["trajectory"]["lateral_bound_tolerance_m"] = physical.trajectory.lateral_bound_tolerance_m;
  snapshot["terminal_stop_course_geometry"]["progress_m"] = physical.terminal_stop_course_geometry.progress_m;
  snapshot["terminal_stop_course_geometry"]["curvature_radpm"] = physical.terminal_stop_course_geometry.curvature_radpm;
  snapshot["terminal_stop_course_geometry"]["lateral_lower_m"] = physical.terminal_stop_course_geometry.lateral_lower_m;
  snapshot["terminal_stop_course_geometry"]["lateral_upper_m"] = physical.terminal_stop_course_geometry.lateral_upper_m;
  snapshot["hard_wall_clearance_m"] = physical.hard_wall_clearance_m;
  snapshot["bound_tolerance_m"] = physical.bound_tolerance_m;
  snapshot["swept_step_m"] = physical.swept_step_m;
  auto grid = snapshot["wall_grid"];
  physical_grid = physical.wall_grid.get();
  grid["available"] = physical_grid != nullptr;
  grid["fingerprint"] = physical.wall_grid_fingerprint;
  if (physical_grid) {
    grid["width"] = physical_grid->width; grid["height"] = physical_grid->height;
    grid["resolution_m"] = physical_grid->resolution_m;
    grid["origin_x_m"] = physical_grid->origin_x_m; grid["origin_y_m"] = physical_grid->origin_y_m;
    grid["y_axis"] = physical_grid->y_axis == recovery_footprint::YAxisConvention::RowZeroAtMinimumY ?
      "row-zero-at-minimum-y" : "row-zero-at-maximum-y";
    grid["cell_count"] = physical_grid->cells.size(); grid["payload"] = payload;
  }
  auto proof = node["physical_proof"];
  proof["identity"] = identity_node(plan->physical_identity);
  proof["outcome"] = mpcc_rate_resolved_physical_wall::to_string(plan->physical_outcome);
  proof["completed_sec"] = plan->physical_completed_sec;
  proof["diagnostic"]["reason"] = contract::physical_wall_certificate_reason_name(plan->physical_diagnostic.reason);
  proof["diagnostic"]["stage_index"] = plan->physical_diagnostic.stage_index;
  proof["diagnostic"]["waypoint_id"] = plan->physical_diagnostic.waypoint_id;
  proof["diagnostic"]["path_distance_m"] = plan->physical_diagnostic.path_distance_m;
  proof["diagnostic"]["lateral_m"] = plan->physical_diagnostic.lateral_m;
  proof["diagnostic"]["lag_m"] = plan->physical_diagnostic.lag_m;
  proof["diagnostic"]["lower_bound_m"] = plan->physical_diagnostic.lower_bound_m;
  proof["diagnostic"]["upper_bound_m"] = plan->physical_diagnostic.upper_bound_m;
  proof["diagnostic"]["bound_reserve_m"] = plan->physical_diagnostic.bound_reserve_m;
  proof["diagnostic"]["heading_offset_rad"] = plan->physical_diagnostic.heading_offset_rad;
  proof["diagnostic"]["reference_progress_m"] = plan->physical_diagnostic.reference_progress_m;
  proof["diagnostic"]["solved_progress_m"] = plan->physical_diagnostic.solved_progress_m;
  proof["diagnostic"]["progress_delta_m"] = plan->physical_diagnostic.progress_delta_m;
  proof["diagnostic"]["pose_x_m"] = plan->physical_diagnostic.pose_x_m;
  proof["diagnostic"]["pose_y_m"] = plan->physical_diagnostic.pose_y_m;
  proof["diagnostic"]["pose_yaw_rad"] = plan->physical_diagnostic.pose_yaw_rad;
  proof["diagnostic"]["out_of_map"] = plan->physical_diagnostic.out_of_map;
  proof["diagnostic"]["contact_cell_count"] = plan->physical_diagnostic.contact_cell_count;
  proof["diagnostic"]["swept_rejected_path_index"] = plan->physical_diagnostic.swept_rejected_path_index;
  proof["diagnostic"]["swept_checked_pose_count"] = plan->physical_diagnostic.swept_checked_pose_count;
  proof["diagnostic"]["swept_rejected_substep"] = plan->physical_diagnostic.swept_rejected_substep;
  proof["diagnostic"]["swept_rejected_subdivision_count"] = plan->physical_diagnostic.swept_rejected_subdivision_count;
  proof["diagnostic"]["swept_rejected_segment_ratio"] = plan->physical_diagnostic.swept_rejected_segment_ratio;
  return node;
}

static bool published_execution_valid(
  const shadow::Snapshot & source,
  const mpcc_rate_resolved_execution_artifact::ExecutionArtifact & artifact,
  const PublicationEvidence & publication)
{
  namespace execution = mpcc_rate_resolved_execution_artifact;
  return !(
      fingerprint_interaction_snapshot(source) == 0U ||
      !execution::same_identity(source.identity, artifact.identity) ||
      execution::validate(artifact) != execution::RejectReason::None ||
      !shadow::semantic_initial_state_matches(source, artifact) ||
      (source.physical_wall_refinement_active &&
      !mpcc_rate_resolved::course_frame_matches(
        artifact.course_frame, source.wall_course_frame_knots,
        source.course_progress_origin_m)) ||
      publication.failure_decision_id == 0U ||
      publication.failure_interaction_fingerprint == 0U ||
      (publication.source_kind != "exact-executed" &&
      publication.source_kind != "current-world-bundle") ||
      publication.publication_decision_id == 0U ||
      !std::isfinite(publication.failure_observation_sec) ||
      !std::isfinite(publication.failure_control_origin_sec) ||
      !std::isfinite(publication.publication_control_origin_sec) ||
      !std::isfinite(publication.publication_artifact_elapsed_sec) ||
      publication.publication_artifact_elapsed_sec < 0.0);
}

static YAML::Node revalidation_evidence_node(
  const mpcc_rate_resolved_retained_revalidation::Request * request,
  const bool same_boundary, const std::string & payload_prefix,
  const recovery_footprint::OccupancyGrid * & inspected_grid,
  const recovery_footprint::OccupancyGrid * & observed_grid,
  const recovery_footprint::OccupancyGrid * & certified_grid)
{
  YAML::Node node;
  node["schema"] = "mpcc-revalidation-observation/v1";
  if (request == nullptr) {
    node["status"] = "missing";
    node["reason"] = "exact evaluation request unavailable";
    return node;
  }
  const auto & r = *request;
  auto value = node["request"];
  value["decision_id"] = r.decision_id;
  value["now_sec"] = r.now_sec;
  value["control_origin_sec"] = r.control_origin_sec;
  value["current_intent"] = contract::to_string(r.current_intent);
  value["publication_prefix_required"] = r.publication_prefix_required;
  value["applied_program_required"] = r.applied_program_required;
  if (r.input_application_profile) value["input_application_profile"] =
    mpcc_vehicle_model::encode_input_application_profile(*r.input_application_profile);
  if (r.publication_prefix && r.plan && r.plan->execution_artifact) {
    const auto & bound = *r.publication_prefix;
    auto prefix = value["prospective_publication"];
    prefix["schema"] = "mpcc-prospective-publication/v1";
    prefix["observation"] = observation_provenance_node(bound.observation);
    prefix["vehicle_model"] = mpcc_vehicle_model::encode_parameters(r.plan->execution_artifact->vehicle_model);
    prefix["vehicle_model_fingerprint"] = bound.vehicle_model_fingerprint;
    prefix["proposed_time_wire_acceleration_wire_steering"] = std::vector<double>{
      bound.proposed_packet.published_sec, bound.proposed_packet.wire_acceleration_mps2,
      bound.proposed_packet.wire_steering_rad};
  }
  using Clock = mpcc_rate_resolved_retained_revalidation::ExecutionClockKind;
  const char * clock = "unknown";
  switch (r.execution_clock.kind) {
    case Clock::Unknown: break;
    case Clock::BootstrapCandidate: clock = "bootstrap-candidate"; break;
    case Clock::TimeAlignedCandidate: clock = "time-aligned-candidate"; break;
    case Clock::PublishedPlan: clock = "published-plan"; break;
  }
  value["execution_clock"]["kind"] = clock;
  value["execution_clock"]["first_published_control_origin_sec"] =
    r.execution_clock.first_published_control_origin_sec;
  value["execution_clock"]["first_published_artifact_elapsed_sec"] =
    r.execution_clock.first_published_artifact_elapsed_sec;
  value["control_origin_physical_progress_m"] = r.control_origin_physical_progress_m;
  value["path_length_m"] = r.path_length_m;
  value["progress_continuity_tolerance_m"] = r.progress_continuity_tolerance_m;
  value["circular"] = r.circular;
  value["current_speed_mps"] = r.current_speed_mps;
  value["control_origin_speed_mps"] = r.control_origin_speed_mps;
  value["current_time_steering_rad"] = r.current_time_steering_rad;
  value["current_steering_rad"] = r.current_steering_rad;
  value["current_response_steering_rad"] = r.current_response_steering_rad;
  value["current_lateral_velocity_mps"] = r.current_lateral_velocity_mps;
  value["current_yaw_rate_radps"] = r.current_yaw_rate_radps;
  value["previous_published_steering_rad"] = r.previous_published_steering_rad;
  value["previous_published_command_age_sec"] = r.previous_published_command_age_sec;
  value["minimum_acceleration_mps2"] = r.minimum_acceleration_mps2;
  value["maximum_acceleration_mps2"] = r.maximum_acceleration_mps2;
  const auto pose_node = [](const recovery_footprint::Pose2D & pose) {
      YAML::Node item;
      item["x_m"] = pose.x_m; item["y_m"] = pose.y_m; item["yaw_rad"] = pose.yaw_rad;
      return item;
    };
  value["control_pose"] = pose_node(r.control_pose);
  value["measured_to_control_path"] = YAML::Node(YAML::NodeType::Sequence);
  for (const auto & pose : r.measured_to_control_path) {
    value["measured_to_control_path"].push_back(pose_node(pose));
  }
  value["measured_to_control_elapsed_sec"] = std_vector_node(r.measured_to_control_elapsed_sec);
  auto footprint = value["current_footprint"];
  footprint["front_extent_m"] = r.current_footprint.front_extent_m;
  footprint["rear_extent_m"] = r.current_footprint.rear_extent_m;
  footprint["left_extent_m"] = r.current_footprint.left_extent_m;
  footprint["right_extent_m"] = r.current_footprint.right_extent_m;
  footprint["margin_m"] = r.current_footprint.margin_m;
  auto policy = value["stop_lateral_policy"];
  policy["wheelbase_m"] = r.stop_lateral_policy.wheelbase_m;
  policy["maximum_abs_steering_rad"] = r.stop_lateral_policy.maximum_abs_steering_rad;
  policy["maximum_abs_steering_rate_radps"] = r.stop_lateral_policy.maximum_abs_steering_rate_radps;
  policy["maximum_lateral_acceleration_mps2"] = r.stop_lateral_policy.maximum_lateral_acceleration_mps2;
  policy["steering_command_gain"] = r.stop_lateral_policy.steering_command_gain;
  policy["lateral_gain"] = r.stop_lateral_policy.lateral_gain;
  policy["heading_gain"] = r.stop_lateral_policy.heading_gain;
  auto obstacles = value["obstacles"];
  obstacles["generation"] = r.obstacles.generation;
  obstacles["observed_sec"] = r.obstacles.observed_sec;
  obstacles["current"] = r.obstacles.current;
  obstacles["obstacles"] = YAML::Node(YAML::NodeType::Sequence);
  for (const auto & peer : r.obstacles.obstacles) {
    YAML::Node item;
    item["id"] = peer.id;
    item["x_m"] = peer.circle.x_m; item["y_m"] = peer.circle.y_m;
    item["velocity_x_mps"] = peer.circle.velocity_x_mps;
    item["velocity_y_mps"] = peer.circle.velocity_y_mps;
    item["acceleration_x_mps2"] = peer.circle.acceleration_x_mps2;
    item["acceleration_y_mps2"] = peer.circle.acceleration_y_mps2;
    item["acceleration_horizon_sec"] = peer.circle.acceleration_horizon_sec;
    item["radius_m"] = peer.circle.radius_m;
    obstacles["obstacles"].push_back(item);
  }
  value["follow_target_available"] = r.follow_target.has_value();
  if (r.follow_target) {
    const auto & target = *r.follow_target;
    auto follow = value["follow_target"];
    follow["target_id"] = target.target_id;
    follow["observation_generation"] = target.observation_generation;
    follow["observed_sec"] = target.observed_sec;
    follow["current_target_gap_m"] = target.current_target_gap_m;
    follow["hard_gap_m"] = target.hard_gap_m;
    follow["target_speed_mps"] = target.target_speed_mps;
    follow["elapsed_time_sec"] = std_vector_node(target.elapsed_time_sec);
    follow["target_progress_from_current_origin_m"] =
      std_vector_node(target.target_progress_from_current_origin_m);
    follow["current"] = target.current;
  }
  auto grid = value["current_wall_grid"];
  grid["available"] = r.current_wall_grid != nullptr;
  if (r.current_wall_grid) {
    observed_grid = r.current_wall_grid.get();
    grid["width"] = observed_grid->width; grid["height"] = observed_grid->height;
    grid["resolution_m"] = observed_grid->resolution_m;
    grid["origin_x_m"] = observed_grid->origin_x_m; grid["origin_y_m"] = observed_grid->origin_y_m;
    grid["y_axis"] = observed_grid->y_axis == recovery_footprint::YAxisConvention::RowZeroAtMinimumY ?
      "row-zero-at-minimum-y" : "row-zero-at-maximum-y";
    grid["cell_count"] = observed_grid->cells.size();
    grid["payload"] = payload_prefix + "revalidation-wall-grid.bin";
  }

  // Status authenticates an observed input/inspected artifact association. It
  // is not a claim that this request or trajectory passed physical proof.
  if (!same_boundary) {
    node["status"] = "invalid";
    node["reason"] = "revalidation request does not match the required decision, clock or source association";
  } else {
    node["status"] = "present";
    node["reason"] = "exact evaluation input; not proof or publication authority";
  }
  node["certified_plan_evidence"] = certified_plan_evidence_node(
    r.plan, payload_prefix + "inspected-certified-wall-grid.bin", certified_grid);
  if (!r.plan || !r.plan->execution_artifact || !r.plan->solver_source_snapshot) {
    node["inspected_plan_status"] = "missing";
    return node;
  }
  const auto & source = *r.plan->solver_source_snapshot;
  const auto & artifact = *r.plan->execution_artifact;
  namespace execution = mpcc_rate_resolved_execution_artifact;
  if (!interaction_snapshot_complete(source) ||
    execution::validate(artifact) != execution::RejectReason::None ||
    !execution::same_identity(source.identity, artifact.identity))
  {
    node["inspected_plan_status"] = "invalid";
    node["inspected_plan_reason"] = "inspected source/artifact identity or shape invalid";
    return node;
  }
  node["inspected_plan_status"] = "present";
  node["inspected_source"] = source_node(source, payload_prefix + "inspected-wall-grid.bin");
  node["inspected_interaction_fingerprint"] = fingerprint_interaction_snapshot(source);
  auto artifact_node = execution_evidence_node(artifact, {});
  artifact_node.remove("publication");
  artifact_node["schema"] = "mpcc-inspected-execution-artifact/v1";
  node["inspected_artifact"] = artifact_node;
  inspected_grid = source.wall_grid.get();
  return node;
}

static RecordResult record_snapshot(
  const shadow::Snapshot & source,
  const problem::AssemblyRequest * const assembly_request,
  const problem::Problem * const exact_problem,
  const std::optional<persistent_osqp::WarmStart> * const warm_start,
  const persistent_osqp::SolveOutcome * const outcome,
  const PipelineStage pipeline_stage,
  const std::string & failure_outcome,
  const std::string & failure_detail,
  const std::filesystem::path & output_root,
  const YAML::Node & execution_evidence = YAML::Node(),
  const YAML::Node & publication_bundle = YAML::Node(),
  const recovery_footprint::OccupancyGrid * const published_grid = nullptr,
  const YAML::Node & revalidation_evidence = YAML::Node(),
  const recovery_footprint::OccupancyGrid * const inspected_grid = nullptr,
  const recovery_footprint::OccupancyGrid * const observed_grid = nullptr,
  const YAML::Node & previous_revalidation_evidence = YAML::Node(),
  const recovery_footprint::OccupancyGrid * const previous_inspected_grid = nullptr,
  const recovery_footprint::OccupancyGrid * const previous_observed_grid = nullptr,
  const recovery_footprint::OccupancyGrid * const published_certified_grid = nullptr,
  const recovery_footprint::OccupancyGrid * const inspected_certified_grid = nullptr,
  const recovery_footprint::OccupancyGrid * const previous_certified_grid = nullptr) noexcept
{
  RecordResult result;
  try {
    const auto intent = source.identity.source_context.intent;
    if (!contract::canonical_normal_intent_supported(intent)) {
      result.status = RecordStatus::UnsupportedIntent;
      result.detail = "capture requires a canonical normal intent";
      return result;
    }
    if (
      failure_outcome.empty() || output_root.empty() ||
      ((exact_problem == nullptr) != (assembly_request == nullptr)) ||
      ((exact_problem == nullptr) != (warm_start == nullptr)) ||
      ((exact_problem == nullptr) != (outcome == nullptr)) ||
      (exact_problem != nullptr &&
      (exact_problem->horizon_steps <= 0 ||
      exact_problem->linear_cost.size() <= 0 ||
      exact_problem->quadratic_cost.rows() != exact_problem->linear_cost.size() ||
      exact_problem->quadratic_cost.cols() != exact_problem->linear_cost.size() ||
      exact_problem->constraints.cols() != exact_problem->linear_cost.size() ||
      exact_problem->constraints.rows() != exact_problem->lower_bound.size() ||
      exact_problem->lower_bound.size() != exact_problem->upper_bound.size())))
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
    const std::uint64_t interaction_fingerprint =
      fingerprint_interaction_snapshot(source);
    std::ostringstream interaction_identity;
    if (interaction_fingerprint != 0U) {
      interaction_identity << '-' << std::hex << std::setw(16) <<
        std::setfill('0') << interaction_fingerprint;
    }
    const std::string directory_name = sequence.str() +
      interaction_identity.str() + '-' +
      safe_component(contract::to_string(intent)) + '-' +
      physical_homotopy_component(physical_homotopy_side(source)) + '-' +
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
    const auto write_grid = [&temporary_directory](
      const recovery_footprint::OccupancyGrid * const grid,
      const std::string & name) {
        if (grid == nullptr) {
          return true;
        }
        std::ofstream stream(temporary_directory / name, std::ios::binary | std::ios::trunc);
        if (!stream) {
          return false;
        }
        for (const auto cell : grid->cells) {
          const std::int8_t byte = static_cast<std::int8_t>(cell);
          stream.write(reinterpret_cast<const char *>(&byte), sizeof(byte));
        }
        stream.close();
        return static_cast<bool>(stream);
      };
    if (!write_grid(source.wall_grid.get(), grid_payload) ||
      !write_grid(published_grid, "published-wall-grid.bin") ||
      !write_grid(inspected_grid, "inspected-wall-grid.bin") ||
      !write_grid(observed_grid, "revalidation-wall-grid.bin") ||
      !write_grid(previous_inspected_grid, "previous-inspected-wall-grid.bin") ||
      !write_grid(previous_observed_grid, "previous-revalidation-wall-grid.bin") ||
      !write_grid(published_certified_grid, "published-certified-wall-grid.bin") ||
      !write_grid(inspected_certified_grid, "inspected-certified-wall-grid.bin") ||
      !write_grid(previous_certified_grid, "previous-inspected-certified-wall-grid.bin"))
    {
      result.status = RecordStatus::IoFailure;
      result.detail = "cannot write atomic world/publication grid payloads";
      std::filesystem::remove_all(temporary_directory, error);
      return result;
    }

    YAML::Node root;
    // v3 additionally seals the exact moving-Stop policy and braking envelope
    // required to prove recursive terminal viability. Older v2 interactions
    // remain loadable but cannot claim that certificate.
    root["schema"] = source.replay_world.has_value() &&
      source.replay_world->terminal_stop_contract_available ?
      kInteractionSnapshotSchemaV3 : kInteractionSnapshotSchemaV2;
    root["exact_qp_available"] = exact_problem != nullptr;
    root["pipeline_stage"] = to_string(pipeline_stage);
    root["failure_outcome"] = failure_outcome;
    root["failure_detail"] = failure_detail;
    root["source"] = source_node(
      source, source.wall_grid != nullptr ? grid_payload : "");
    root["interaction_fingerprint"] =
      fingerprint_interaction_snapshot(source);
    if (execution_evidence.IsMap()) {
      root["execution_evidence"] = execution_evidence;
    }
    if (publication_bundle.IsMap()) {
      root["publication_bundle"] = publication_bundle;
    }
    if (revalidation_evidence.IsMap()) {
      root["revalidation_evidence"] = revalidation_evidence;
    }
    if (previous_revalidation_evidence.IsMap()) {
      root["previous_accepted_revalidation_evidence"] = previous_revalidation_evidence;
    }
    if (exact_problem != nullptr) {
      root["assembly_request"] = assembly_request_node(*assembly_request);
      root["exact_qp"] = exact_problem_node(*exact_problem);
      root["warm_start"] = warm_start_node(*warm_start);
      root["production_outcome"] = outcome_node(*outcome);
    }

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
  return record_snapshot(
    source, &assembly_request, &exact_problem, &warm_start, &outcome,
    pipeline_stage, failure_outcome, failure_detail, output_root);
}

RecordResult record_proof_failure(
  const shadow::Snapshot & source,
  const PipelineStage pipeline_stage,
  const std::string & failure_outcome,
  const std::string & failure_detail,
  const std::filesystem::path & output_root) noexcept
{
  if (fingerprint_interaction_snapshot(source) == 0U) {
    RecordResult result;
    result.status = RecordStatus::InvalidInput;
    result.detail = "incomplete source-only interaction snapshot";
    return result;
  }
  return record_snapshot(
    source, nullptr, nullptr, nullptr, nullptr, pipeline_stage,
    failure_outcome, failure_detail, output_root);
}

RecordResult record_published_execution(
  const shadow::Snapshot & source,
  const mpcc_rate_resolved_execution_artifact::ExecutionArtifact & artifact,
  const PublicationEvidence & publication,
  const std::string & failure_detail,
  const std::filesystem::path & output_root) noexcept
{
  namespace execution = mpcc_rate_resolved_execution_artifact;
  try {
    if (!published_execution_valid(source, artifact, publication)) {
      return {RecordStatus::InvalidInput, {},
        "invalid published execution identity, artifact or clock"};
    }
    return record_snapshot(
      source, nullptr, nullptr, nullptr, nullptr, PipelineStage::PhysicalProof,
      "published-source-at-authority-loss", failure_detail, output_root,
      execution_evidence_node(artifact, publication));
  } catch (const std::exception & exception) {
    return {RecordStatus::IoFailure, {}, exception.what()};
  } catch (...) {
    return {RecordStatus::IoFailure, {}, "unknown execution evidence exception"};
  }
}

const char * to_string(const AuthorityFailureBoundary boundary) noexcept
{
  switch (boundary) {
    case AuthorityFailureBoundary::TerminalContingency:
      return "terminal-contingency-unavailable";
    case AuthorityFailureBoundary::FinalAuthority:
      return "normal-authority-unavailable";
    case AuthorityFailureBoundary::MovingFinalAuthority:
      return "moving-normal-authority-unavailable";
    case AuthorityFailureBoundary::StopAlternateOverrun:
      return "stop-alternate-revalidation-overrun";
  }
  return "unknown";
}

struct FirstAuthorityFailureRecorder::Impl
{
  explicit Impl(Completion callback)
  : completion(std::move(callback)), worker([this]() {run();}) {}

  void run() noexcept
  {
    while (true) {
      AuthorityFailureObservation observation;
      {
        std::unique_lock<std::mutex> lock(mutex);
        condition.wait(lock, [this]() {return stopping || !pending.empty();});
        if (pending.empty()) {
          return;
        }
        observation = std::move(pending.front());
        pending.pop_front();
      }
      // The queued world owns this failure join. Hashing the full immutable
      // snapshot, like serialization and I/O, stays off the control callback.
      auto & publication = observation.published_execution.publication;
      publication.failure_decision_id = observation.current_world.identity.source_context.decision_id;
      publication.failure_interaction_fingerprint =
        fingerprint_interaction_snapshot(observation.current_world);
      publication.failure_observation_sec = observation.current_world.identity.snapshot_sec;
      publication.failure_control_origin_sec = observation.current_world.control_prediction_origin_sec;
      const auto result = record_authority_failure(
        observation.current_world, to_string(observation.boundary),
        observation.detail, observation.published_execution, observation.output_root,
        observation.revalidation_request, observation.previous_accepted_revalidation_request);
      if (completion) {
        try {
          completion(observation, result);
        } catch (...) {
          // A diagnostic callback cannot terminate recording of admitted events.
        }
      }
    }
  }

  Completion completion;
  std::mutex mutex;
  std::condition_variable condition;
  std::deque<AuthorityFailureObservation> pending;
  std::set<std::string> admitted;
  bool stopping{false};
  std::thread worker;
};

FirstAuthorityFailureRecorder::FirstAuthorityFailureRecorder(Completion completion)
: impl_(std::make_unique<Impl>(std::move(completion))) {}

FirstAuthorityFailureRecorder::~FirstAuthorityFailureRecorder()
{
  stop();
}

ObservationAdmission FirstAuthorityFailureRecorder::submit(
  AuthorityFailureObservation observation)
{
  const auto intent = observation.current_world.identity.source_context.intent;
  const int side = physical_homotopy_side(observation.current_world);
  if (!contract::canonical_normal_intent_supported(intent) || side < -1 || side > 1 ||
    (observation.boundary != AuthorityFailureBoundary::TerminalContingency &&
    observation.boundary != AuthorityFailureBoundary::FinalAuthority &&
    observation.boundary != AuthorityFailureBoundary::MovingFinalAuthority &&
    observation.boundary != AuthorityFailureBoundary::StopAlternateOverrun) ||
    observation.output_root.empty() || !interaction_snapshot_complete(observation.current_world))
  {
    return ObservationAdmission::Invalid;
  }
  // The key space is finite: supported intents x three sides x four boundaries.
  // Decisions, target IDs and paths do not create additional queue buckets.
  const auto key = failure_key(
    observation.current_world, PipelineStage::PhysicalProof, to_string(observation.boundary));
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (impl_->stopping) {
    return ObservationAdmission::Stopped;
  }
  if (impl_->admitted.count(key) != 0U) {
    return ObservationAdmission::Duplicate;
  }
  impl_->pending.push_back(std::move(observation));
  try {
    impl_->admitted.insert(key);
  } catch (...) {
    impl_->pending.pop_back();
    throw;
  }
  impl_->condition.notify_one();
  return ObservationAdmission::Queued;
}

void FirstAuthorityFailureRecorder::stop() noexcept
{
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->stopping = true;
  }
  impl_->condition.notify_one();
  if (impl_->worker.joinable()) {
    impl_->worker.join();
  }
}

RecordResult record_authority_failure(
  const shadow::Snapshot & current_world,
  const std::string & failure_outcome,
  const std::string & failure_detail,
  const PublishedExecutionObservation & published_execution,
  const std::filesystem::path & output_root,
  const std::shared_ptr<const mpcc_rate_resolved_retained_revalidation::Request> &
  revalidation_request,
  const std::shared_ptr<const mpcc_rate_resolved_retained_revalidation::Request> &
  previous_accepted_revalidation_request) noexcept
{
  try {
    const auto failure_fingerprint = fingerprint_interaction_snapshot(current_world);
    if (failure_fingerprint == 0U) {
      return {RecordStatus::InvalidInput, {}, "incomplete authority-loss current world"};
    }
    YAML::Node bundle;
    bundle["schema"] = "mpcc-authority-loss-publication/v1";
    const auto & source = published_execution.source;
    const auto & artifact = published_execution.artifact;
    const auto & publication = published_execution.publication;
    const recovery_footprint::OccupancyGrid * grid = nullptr;
    if (source == nullptr || artifact == nullptr) {
      bundle["status"] = "missing";
      bundle["reason"] = source == nullptr ? "published solver source unavailable" :
        "published execution artifact unavailable";
    } else if (
      !published_execution_valid(*source, *artifact, publication) ||
      publication.failure_decision_id != current_world.identity.source_context.decision_id ||
      publication.failure_interaction_fingerprint != failure_fingerprint ||
      publication.failure_observation_sec != current_world.identity.snapshot_sec ||
      publication.failure_control_origin_sec != current_world.control_prediction_origin_sec)
    {
      bundle["status"] = "invalid";
      bundle["reason"] = "published source/artifact/clock does not join this failure world";
    } else {
      bundle["status"] = "present";
      bundle["reason"] = "matching immutable publication and failure world";
      bundle["source"] = source_node(
        *source, source->wall_grid != nullptr ? "published-wall-grid.bin" : "");
      bundle["interaction_fingerprint"] = fingerprint_interaction_snapshot(*source);
      bundle["execution_evidence"] = execution_evidence_node(*artifact, publication);
      grid = source->wall_grid.get();
    }
    const recovery_footprint::OccupancyGrid * published_certified_grid = nullptr;
    const recovery_footprint::OccupancyGrid * inspected_certified_grid = nullptr;
    const recovery_footprint::OccupancyGrid * previous_certified_grid = nullptr;
    auto complete_publication = certified_plan_evidence_node(
      published_execution.certified_plan, "published-certified-wall-grid.bin", published_certified_grid);
    if (complete_publication["status"].as<std::string>() == "present") {
      const bool association = artifact && published_execution.certified_plan &&
        artifact == published_execution.certified_plan->execution_artifact &&
        publication.failure_decision_id == current_world.identity.source_context.decision_id &&
        publication.failure_interaction_fingerprint == failure_fingerprint &&
        publication.failure_observation_sec == current_world.identity.snapshot_sec &&
        publication.failure_control_origin_sec == current_world.control_prediction_origin_sec &&
        (publication.source_kind == "exact-executed" || publication.source_kind == "current-world-bundle") &&
        publication.publication_decision_id > 0U &&
        publication.publication_decision_id < publication.failure_decision_id &&
        std::isfinite(publication.publication_control_origin_sec) &&
        publication.publication_control_origin_sec >= 0.0 &&
        publication.publication_control_origin_sec <= publication.failure_control_origin_sec &&
        std::isfinite(publication.publication_artifact_elapsed_sec) &&
        publication.publication_artifact_elapsed_sec >= 0.0;
      complete_publication["publication"] = execution_evidence_node(
        *published_execution.certified_plan->execution_artifact, publication)["publication"];
      if (!association) {
        complete_publication["status"] = "invalid";
        complete_publication["reason"] = "physical plan does not match actual publication and failure clocks";
      }
    }
    bundle["certified_plan_evidence"] = complete_publication;
    const recovery_footprint::OccupancyGrid * inspected_grid = nullptr;
    const recovery_footprint::OccupancyGrid * observed_grid = nullptr;
    const auto * const current = revalidation_request.get();
    const bool same_boundary = current != nullptr &&
      current->decision_id == current_world.identity.source_context.decision_id &&
      current->now_sec == current_world.identity.snapshot_sec &&
      current->control_origin_sec == current_world.control_prediction_origin_sec;
    const auto revalidation = revalidation_evidence_node(
      current, same_boundary, "", inspected_grid, observed_grid, inspected_certified_grid);
    const auto * const previous = previous_accepted_revalidation_request.get();
    // The predecessor is a separately observed input, never a reconstruction
    // using the failure's newer pose, peers or wall. The caller observed an
    // ordinary Accepted result; this association grants no publication role.
    const bool same_source_predecessor = same_boundary && previous != nullptr &&
      previous->decision_id > 0U && previous->decision_id < current->decision_id &&
      std::isfinite(previous->now_sec) && previous->now_sec <= current->now_sec &&
      std::isfinite(previous->control_origin_sec) &&
      previous->control_origin_sec <= current->control_origin_sec &&
      previous->current_intent == current->current_intent &&
      previous->plan && previous->plan->execution_artifact &&
      current->plan && current->plan->execution_artifact &&
      mpcc_rate_resolved_execution_artifact::same_identity(
        previous->plan->execution_artifact->identity, current->plan->execution_artifact->identity);
    const recovery_footprint::OccupancyGrid * previous_inspected_grid = nullptr;
    const recovery_footprint::OccupancyGrid * previous_observed_grid = nullptr;
    auto previous_revalidation = revalidation_evidence_node(
      previous, same_source_predecessor, "previous-",
      previous_inspected_grid, previous_observed_grid, previous_certified_grid);
    previous_revalidation["observation_role"] = "previous-ordinary-terminal-accepted";
    previous_revalidation["failure_decision_id"] = current_world.identity.source_context.decision_id;
    return record_snapshot(
      current_world, nullptr, nullptr, nullptr, nullptr, PipelineStage::PhysicalProof,
      failure_outcome, failure_detail, output_root, YAML::Node(), bundle, grid,
      revalidation, inspected_grid, observed_grid, previous_revalidation,
      previous_inspected_grid, previous_observed_grid,
      published_certified_grid, inspected_certified_grid, previous_certified_grid);
  } catch (const std::exception & exception) {
    return {RecordStatus::IoFailure, {}, exception.what()};
  } catch (...) {
    return {RecordStatus::IoFailure, {}, "unknown authority-loss bundle exception"};
  }
}

std::optional<RecordedQp> load_recorded_qp(
  const std::filesystem::path & snapshot_file, std::string * detail) noexcept
{
  try {
    const YAML::Node root = YAML::LoadFile(snapshot_file.string());
    if (!root["schema"] ||
      !supported_exact_qp_schema(root["schema"].as<std::string>()))
    {
      if (detail != nullptr) {
        *detail = "unsupported snapshot schema";
      }
      return std::nullopt;
    }
    if (
      root["exact_qp_available"] &&
      !root["exact_qp_available"].as<bool>())
    {
      if (detail != nullptr) {
        *detail = "source-only proof-boundary snapshot has no exact QP";
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
    const auto production_outcome = root["production_outcome"];
    if (
      production_outcome && production_outcome["rejected_primal_available"] &&
      production_outcome["rejected_primal_available"].as<bool>())
    {
      auto rejected_primal = load_vector(
        production_outcome["rejected_primal"]);
      if (!rejected_primal) {
        if (detail != nullptr) {
          *detail = "malformed rejected-primal payload";
        }
        return std::nullopt;
      }
      recorded.rejected_primal = std::move(rejected_primal.value());
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
    if (!root["schema"])
    {
      if (detail != nullptr) *detail = "unsupported snapshot schema";
      return std::nullopt;
    }
    const std::string schema = root["schema"].as<std::string>();
    if (schema == kExactQpSnapshotSchemaV1) {
      if (detail != nullptr) {
        *detail =
          "v1 snapshot has no immutable dynamic-obstacle constraint identity; "
          "exact QP replay remains available";
      }
      return std::nullopt;
    }
    if (
      schema != kInteractionSnapshotSchemaV2 &&
      schema != kInteractionSnapshotSchemaV3)
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
    RecordedInteractionSnapshot recorded;
    recorded.source = std::move(source.value());
    const bool exact_qp_available = !root["exact_qp_available"] ||
      root["exact_qp_available"].as<bool>();
    if (exact_qp_available) {
      auto assembly_request = load_assembly_request(root["assembly_request"]);
      if (!assembly_request.has_value()) {
        if (detail != nullptr) *detail = "assembly request unavailable";
        return std::nullopt;
      }
      std::string qp_detail;
      auto recorded_qp = load_recorded_qp(snapshot_file, &qp_detail);
      if (!recorded_qp.has_value()) {
        if (detail != nullptr) *detail = "exact QP unavailable: " + qp_detail;
        return std::nullopt;
      }
      recorded.assembly_request = std::move(assembly_request.value());
      recorded.recorded_qp = std::move(recorded_qp.value());
    }
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
