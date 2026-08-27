#include "multi_purpose_mpc_ros/mpcc_architecture_snapshot.hpp"

#include "multi_purpose_mpc_ros/mpcc_execution_contract.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_shadow.hpp"

#include <yaml-cpp/yaml.h>

#include <Eigen/Sparse>

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <set>
#include <sstream>
#include <system_error>
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
