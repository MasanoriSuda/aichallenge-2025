#include "multi_purpose_mpc_ros/mpcc_architecture_comparison.hpp"

#include "multi_purpose_mpc_ros/mpc_stage_geometry.hpp"
#include "multi_purpose_mpc_ros/mpcc_execution_contract.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_physical_adapter.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_stop_control_lattice.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_wall_refinement.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <exception>
#include <limits>
#include <sstream>
#include <utility>

namespace multi_purpose_mpc_ros::mpcc_architecture_comparison
{
namespace
{

namespace architecture = mpcc_architecture_snapshot;
namespace contract = mpcc_execution_contract;
namespace dynamic = mpcc_rate_resolved_dynamic_proof;
namespace maneuver = mpcc_stateless_maneuver;
namespace physical = mpcc_rate_resolved_physical_adapter;
namespace problem = mpcc_rate_resolved_problem;
namespace shadow = mpcc_rate_resolved_shadow;
namespace stop_lattice = mpcc_rate_resolved_stop_control_lattice;
namespace wall = mpcc_rate_resolved_physical_wall;
namespace recovery = recovery_footprint;
namespace artifact = mpcc_rate_resolved_execution_artifact;
namespace osqp = persistent_osqp;

maneuver::TerminalResolution resolve_audit_terminal_successor(
  const shadow::Snapshot & source) noexcept
{
  auto resolution = maneuver::resolve_terminal_successor(source);
  if (resolution.accepted) {
    return resolution;
  }
  const auto & context = source.identity.source_context;
  const bool target_free_normal =
    (context.intent == contract::ControlIntent::Track ||
    context.intent == contract::ControlIntent::Cruise) &&
    context.target_id.empty() && context.target_obstacle_generation == 0U &&
    !context.dynamic_obstacle_constraint_active &&
    context.dynamic_obstacle_id.empty() &&
    context.dynamic_obstacle_generation == 0U;
  if (
    !target_free_normal || source.request.states.empty() ||
    source.request.inputs.empty())
  {
    return resolution;
  }
  double maximum_deceleration_mps2 = 0.0;
  for (const auto & input : source.request.inputs) {
    maximum_deceleration_mps2 = std::min(
      maximum_deceleration_mps2,
      input.lower[mpcc_rate_resolved::kAccelerationIndex]);
  }
  if (!std::isfinite(maximum_deceleration_mps2) ||
    maximum_deceleration_mps2 >= -1.0e-9)
  {
    resolution.detail =
      "target-free normal audit has no physical braking authority";
    return resolution;
  }
  const double terminal_lateral_m =
    source.request.states.back().reference[
    mpcc_rate_resolved::kLateralIndex];
  if (!std::isfinite(terminal_lateral_m)) {
    resolution.detail =
      "target-free normal audit terminal lateral reference unavailable";
    return resolution;
  }
  resolution.accepted = true;
  resolution.successor = maneuver::TerminalSuccessor::Replan;
  resolution.stop_suffix.available = true;
  resolution.stop_suffix.hold_lateral_m = terminal_lateral_m;
  resolution.stop_suffix.target_velocity_mps = 0.0;
  resolution.stop_suffix.maximum_deceleration_mps2 =
    maximum_deceleration_mps2;
  resolution.detail = "target-free normal replan with Stop contingency";
  return resolution;
}

void append_fingerprint_u64(
  std::uint64_t & fingerprint, const std::uint64_t value) noexcept
{
  constexpr std::uint64_t prime = 1099511628211ULL;
  for (std::size_t byte = 0U; byte < sizeof(value); ++byte) {
    fingerprint ^= (value >> (8U * byte)) & 0xffU;
    fingerprint *= prime;
  }
}

void append_fingerprint_double(
  std::uint64_t & fingerprint, const double value) noexcept
{
  std::uint64_t bits = 0U;
  static_assert(sizeof(bits) == sizeof(value));
  std::memcpy(&bits, &value, sizeof(bits));
  append_fingerprint_u64(fingerprint, bits);
}

std::uint64_t exact_problem_fingerprint(
  const problem::Problem & qp, const std::uint64_t source_fingerprint) noexcept
{
  std::uint64_t fingerprint = 1469598103934665603ULL;
  append_fingerprint_u64(fingerprint, source_fingerprint);
  append_fingerprint_u64(
    fingerprint, static_cast<std::uint64_t>(qp.horizon_steps));
  const auto append_vector = [&fingerprint](const Eigen::VectorXd & values) {
      append_fingerprint_u64(
        fingerprint, static_cast<std::uint64_t>(values.size()));
      for (Eigen::Index index = 0; index < values.size(); ++index) {
        append_fingerprint_double(fingerprint, values[index]);
      }
    };
  const auto append_sparse = [&fingerprint](
      const Eigen::SparseMatrix<double> & matrix) {
      append_fingerprint_u64(
        fingerprint, static_cast<std::uint64_t>(matrix.rows()));
      append_fingerprint_u64(
        fingerprint, static_cast<std::uint64_t>(matrix.cols()));
      append_fingerprint_u64(
        fingerprint, static_cast<std::uint64_t>(matrix.nonZeros()));
      for (int outer = 0; outer < matrix.outerSize(); ++outer) {
        for (Eigen::SparseMatrix<double>::InnerIterator item(matrix, outer);
          item; ++item)
        {
          append_fingerprint_u64(
            fingerprint, static_cast<std::uint64_t>(item.row()));
          append_fingerprint_u64(
            fingerprint, static_cast<std::uint64_t>(item.col()));
          append_fingerprint_double(fingerprint, item.value());
        }
      }
    };
  append_vector(qp.linear_cost);
  append_vector(qp.lower_bound);
  append_vector(qp.upper_bound);
  append_sparse(qp.quadratic_cost);
  append_sparse(qp.constraints);
  append_vector(qp.variable_scaling.physical_units_per_solver_unit);
  return fingerprint == 0U ? 1U : fingerprint;
}

std::optional<problem::Problem> external_primal_problem(
  const architecture::RecordedInteractionSnapshot & recorded,
  const ExternalPrimalConstraintPolicy policy) noexcept
{
  if (!recorded.recorded_qp.has_value()) {
    return std::nullopt;
  }
  auto qp = recorded.recorded_qp->problem;
  if (
    policy == ExternalPrimalConstraintPolicy::ExactRecorded ||
    policy == ExternalPrimalConstraintPolicy::PhysicalNonlinearOracle)
  {
    return qp;
  }
  const int horizon = recorded.source.request.horizon_steps;
  constexpr int state_dimension = mpcc_rate_resolved::kStateDimension;
  const int state_values = state_dimension * (horizon + 1);
  if (
    horizon <= 0 || qp.lower_bound.size() != qp.upper_bound.size() ||
    qp.lower_bound.size() < state_values + state_values)
  {
    return std::nullopt;
  }
  const int state_element =
    policy == ExternalPrimalConstraintPolicy::OmitWallHeadingBucket ?
    mpcc_rate_resolved::kHeadingIndex : mpcc_rate_resolved::kLagIndex;
  // The recorded QP state-box block starts after the initial-state equality
  // rows. Only the selected artificial post-hoc wall pose bucket is removed;
  // dynamics, actuator, progress/lateral wall and opponent rows are retained.
  // This problem exists only for the architecture oracle below and has no
  // production authority API.
  for (int stage = 0; stage <= horizon; ++stage) {
    const int row = state_values + stage * state_dimension + state_element;
    qp.lower_bound[row] = -std::numeric_limits<double>::infinity();
    qp.upper_bound[row] = std::numeric_limits<double>::infinity();
  }
  return qp;
}

struct ExternalArtifactBuild
{
  std::optional<artifact::ExecutionArtifact> value;
  artifact::RejectReason reject_reason{artifact::RejectReason::InvalidCertificate};
  double maximum_absolute_violation{};
  double maximum_normalized_violation{};
  int maximum_normalized_row{-1};
  std::string detail{"not-evaluated"};
};

ExternalArtifactBuild build_external_artifact(
  const shadow::Snapshot & snapshot, const problem::Problem & qp,
  const Eigen::VectorXd & primal,
  const ExternalPrimalConstraintPolicy policy) noexcept
{
  namespace model = mpcc_rate_resolved;
  ExternalArtifactBuild result;
  const bool enforce_affine_rows =
    policy != ExternalPrimalConstraintPolicy::PhysicalNonlinearOracle;
  const int horizon = snapshot.request.horizon_steps;
  const int execution_horizon = snapshot.execution_prefix_steps;
  const int state_values = model::kStateDimension * (horizon + 1);
  const int variable_count = state_values + model::kInputDimension * horizon;
  const int state_box_row = state_values;
  if (
    horizon <= 0 || execution_horizon <= 0 || execution_horizon > horizon ||
    qp.horizon_steps != horizon || qp.constraints.cols() != variable_count ||
    qp.constraints.rows() != qp.lower_bound.size() ||
    qp.lower_bound.size() != qp.upper_bound.size() ||
    state_box_row + state_values > qp.lower_bound.size() ||
    primal.size() != variable_count || !primal.allFinite() ||
    snapshot.nominal_path_distance_m.size() <
    static_cast<std::size_t>(execution_horizon + 1) ||
    snapshot.request.inputs.size() < static_cast<std::size_t>(execution_horizon))
  {
    result.detail = "external-primal-dimension-contract-mismatch";
    return result;
  }

  shadow::SolverContext tolerance_owner;
  const auto tolerance = tolerance_owner.physical_constraint_tolerance();
  double physical_scale = std::max(1.0, primal.cwiseAbs().maxCoeff());
  if (enforce_affine_rows) {
    const auto residual = osqp::evaluate_constraint_residuals(
      qp.constraints, primal, qp.lower_bound, qp.upper_bound,
      tolerance.absolute, tolerance.relative);
    if (!residual.has_value()) {
      result.detail = "external-primal-residual-evaluation-rejected";
      return result;
    }
    result.maximum_absolute_violation = residual->maximum_absolute_violation;
    result.maximum_normalized_violation = residual->maximum_normalized_violation;
    result.maximum_normalized_row = residual->maximum_normalized_row;
    if (residual->maximum_normalized_violation > 1.0) {
      std::ostringstream detail;
      detail << "external-primal-constraint-rejected/row="
             << residual->maximum_normalized_row
             << "/absolute=" << residual->maximum_absolute_violation
             << "/normalized=" << residual->maximum_normalized_violation;
      result.detail = detail.str();
      return result;
    }

    const Eigen::VectorXd values = qp.constraints * primal;
    double maximum_projected_absolute = 0.0;
    for (Eigen::Index row = 0; row < values.size(); ++row) {
      double projected = values[row];
      if (std::isfinite(qp.lower_bound[row])) {
        projected = std::max(projected, qp.lower_bound[row]);
      }
      if (std::isfinite(qp.upper_bound[row])) {
        projected = std::min(projected, qp.upper_bound[row]);
      }
      maximum_projected_absolute = std::max(
        maximum_projected_absolute, std::abs(projected));
    }
    physical_scale = std::max(
      values.cwiseAbs().maxCoeff(), maximum_projected_absolute);
  }

  artifact::ExecutionArtifact execution;
  execution.identity = snapshot.identity;
  execution.prediction_origin_sec = snapshot.control_prediction_origin_sec;
  execution.publication_interval_sec = snapshot.publication_interval_sec;
  execution.completed_sec = std::max(
    snapshot.identity.snapshot_sec, snapshot.control_prediction_origin_sec);
  execution.course_progress_origin_m = snapshot.course_progress_origin_m;
  execution.semantic_initial_steering_rad =
    snapshot.request.current_steering_rad;
  execution.semantic_initial_response_steering_rad =
    snapshot.request.current_response_steering_rad;
  execution.wheelbase_m = snapshot.request.wheelbase_m;
  execution.yaw_response_gain = snapshot.request.yaw_response_gain;
  execution.yaw_response_time_constant_sec =
    snapshot.request.yaw_response_time_constant_sec;
  execution.minimum_frenet_denominator =
    snapshot.request.minimum_frenet_denominator;
  execution.maximum_abs_steering_rad =
    snapshot.request.maximum_abs_steering_rad;
  execution.maximum_abs_steering_rate_radps =
    snapshot.request.maximum_abs_steering_rate_radps;
  execution.physical_global_tolerance =
    tolerance.absolute + tolerance.relative * physical_scale;
  execution.maximum_constraint_violation =
    result.maximum_absolute_violation;
  execution.maximum_normalized_constraint_violation =
    result.maximum_normalized_violation;
  execution.nominal_path_distance_m.assign(
    snapshot.nominal_path_distance_m.begin(),
    snapshot.nominal_path_distance_m.begin() + execution_horizon + 1);
  execution.predicted_states.reserve(
    static_cast<std::size_t>(execution_horizon + 1));
  execution.lateral_lower_m.reserve(
    static_cast<std::size_t>(execution_horizon + 1));
  execution.lateral_upper_m.reserve(
    static_cast<std::size_t>(execution_horizon + 1));
  std::optional<std::pair<double, double>> physical_oracle_lateral_support;
  if (!enforce_affine_rows) {
    const auto support =
      mpcc_rate_resolved_wall_refinement::
      resolve_pre_refinement_lateral_support(
      mpcc_rate_resolved_wall_refinement::
      PreRefinementLateralSupportRequest{
        true, snapshot.request.initial_state[model::kLateralIndex],
        snapshot.wall_lower_m, snapshot.wall_upper_m});
    if (!support.valid || !support.applied) {
      result.detail =
        "physical-nonlinear-oracle-lateral-support-rejected/" +
        std::string{mpcc_rate_resolved_wall_refinement::to_string(
          support.reason)};
      return result;
    }
    physical_oracle_lateral_support =
      std::pair{support.lower_m, support.upper_m};
  }
  model::StateVector nonlinear_state = model::StateVector::Zero();
  nonlinear_state[model::kLateralIndex] =
    snapshot.request.initial_state[model::kLateralIndex];
  nonlinear_state[model::kLagIndex] =
    snapshot.request.initial_state[model::kLagIndex];
  nonlinear_state[model::kHeadingIndex] =
    snapshot.request.initial_state[model::kHeadingIndex];
  nonlinear_state[model::kVelocityIndex] =
    snapshot.request.initial_state[model::kVelocityIndex];
  nonlinear_state[model::kProgressIndex] =
    snapshot.request.initial_state[model::kProgressIndex];
  nonlinear_state[model::kSteeringIndex] =
    snapshot.request.current_steering_rad;
  nonlinear_state[model::kResponseSteeringIndex] =
    snapshot.request.current_response_steering_rad;
  for (int stage = 0; stage <= execution_horizon; ++stage) {
    const int state = model::kStateDimension * stage;
    if (!enforce_affine_rows && stage > 0) {
      const int input = state_values +
        model::kInputDimension * (stage - 1);
      const auto & semantic = snapshot.request.inputs[
        static_cast<std::size_t>(stage - 1)];
      model::LinearizationRequest transition;
      transition.reference_lateral_m =
        nonlinear_state[model::kLateralIndex];
      transition.reference_lag_m = nonlinear_state[model::kLagIndex];
      transition.reference_heading_rad =
        nonlinear_state[model::kHeadingIndex];
      transition.reference_velocity_mps =
        nonlinear_state[model::kVelocityIndex];
      transition.reference_progress_m =
        nonlinear_state[model::kProgressIndex];
      transition.reference_steering_rad =
        nonlinear_state[model::kSteeringIndex];
      transition.reference_response_steering_rad =
        nonlinear_state[model::kResponseSteeringIndex];
      transition.reference_acceleration_mps2 =
        primal[input + model::kAccelerationIndex];
      transition.reference_steering_rate_radps =
        primal[input + model::kSteeringRateIndex];
      transition.reference_virtual_progress_speed_mps =
        primal[input + model::kVirtualProgressSpeedIndex];
      transition.reference_path_curvature_radpm =
        semantic.path_curvature_radpm;
      transition.wheelbase_m = snapshot.request.wheelbase_m;
      transition.yaw_response_gain = snapshot.request.yaw_response_gain;
      transition.yaw_response_time_constant_sec =
        snapshot.request.yaw_response_time_constant_sec;
      transition.stage_dt_sec = semantic.stage_dt_sec;
      transition.minimum_frenet_denominator =
        snapshot.request.minimum_frenet_denominator;
      transition.minimum_stage_dt_sec = semantic.stage_dt_sec;
      transition.maximum_stage_dt_sec = semantic.stage_dt_sec;
      const auto advanced = model::evaluate_temporal_frenet_transition(
        transition);
      if (!advanced.has_value()) {
        result.detail =
          "physical-nonlinear-oracle-transition-rejected/stage=" +
          std::to_string(stage - 1);
        return result;
      }
      nonlinear_state = advanced->next_state;
    }
    execution.predicted_states.push_back(artifact::PredictedState{
      enforce_affine_rows ? primal[state + model::kLateralIndex] :
      nonlinear_state[model::kLateralIndex],
      enforce_affine_rows ? primal[state + model::kLagIndex] :
      nonlinear_state[model::kLagIndex],
      enforce_affine_rows ? primal[state + model::kHeadingIndex] :
      nonlinear_state[model::kHeadingIndex],
      enforce_affine_rows ? primal[state + model::kVelocityIndex] :
      nonlinear_state[model::kVelocityIndex],
      enforce_affine_rows ? primal[state + model::kProgressIndex] :
      nonlinear_state[model::kProgressIndex],
      enforce_affine_rows ? primal[state + model::kSteeringIndex] :
      nonlinear_state[model::kSteeringIndex],
      enforce_affine_rows ? primal[state + model::kResponseSteeringIndex] :
      nonlinear_state[model::kResponseSteeringIndex]});
    const int lateral_row = state_box_row + state + model::kLateralIndex;
    execution.lateral_lower_m.push_back(
      physical_oracle_lateral_support.has_value() ?
      physical_oracle_lateral_support->first : qp.lower_bound[lateral_row]);
    execution.lateral_upper_m.push_back(
      physical_oracle_lateral_support.has_value() ?
      physical_oracle_lateral_support->second : qp.upper_bound[lateral_row]);
  }
  execution.control_stages.reserve(
    static_cast<std::size_t>(execution_horizon));
  for (int stage = 0; stage < execution_horizon; ++stage) {
    const int input = state_values + model::kInputDimension * stage;
    const auto & semantic = snapshot.request.inputs[static_cast<std::size_t>(stage)];
    execution.control_stages.push_back(artifact::ControlStage{
      primal[input + model::kAccelerationIndex],
      primal[input + model::kSteeringRateIndex],
      primal[input + model::kVirtualProgressSpeedIndex],
      semantic.stage_dt_sec,
      semantic.lower[model::kVirtualProgressSpeedIndex],
      semantic.upper[model::kVirtualProgressSpeedIndex],
      semantic.lower[model::kAccelerationIndex],
      semantic.upper[model::kAccelerationIndex],
      semantic.path_curvature_radpm});
  }
  result.reject_reason = artifact::validate(execution);
  if (result.reject_reason != artifact::RejectReason::None) {
    result.detail = std::string{"external-artifact-validation-rejected/"} +
      artifact::to_string(result.reject_reason);
    return result;
  }
  result.detail = "accepted";
  result.value = std::move(execution);
  return result;
}

struct DynamicModelDiagnostic
{
  bool complete{false};
  double minimum_full_disjunction_reserve_m{
    std::numeric_limits<double>::infinity()};
  int minimum_full_disjunction_reserve_stage{-1};
  double minimum_affine_node_clearance_m{std::numeric_limits<double>::infinity()};
  int minimum_affine_node_clearance_stage{-1};
  double minimum_nonlinear_node_clearance_m{std::numeric_limits<double>::infinity()};
  int minimum_nonlinear_node_clearance_stage{-1};
  double maximum_node_position_error_m{};
  int maximum_node_position_error_stage{-1};
  double preceding_node_clearance_m{std::numeric_limits<double>::quiet_NaN()};
  int preceding_node_clearance_stage{-1};
  double following_node_clearance_m{std::numeric_limits<double>::quiet_NaN()};
  int following_node_clearance_stage{-1};
};

std::optional<recovery::Pose2D> reconstruct_pose(
  const std::vector<mpc_stage_geometry::CourseFrameKnot> & knots,
  const race_mpcc_foundation::ExactPhysicalExecutionTrajectory & trajectory,
  const std::size_t index,
  const double tolerance_m) noexcept
{
  if (
    index >= trajectory.progress_m.size() ||
    index >= trajectory.lateral_m.size() ||
    index >= trajectory.lag_m.size() ||
    index >= trajectory.heading_offset_rad.size())
  {
    return std::nullopt;
  }
  const auto frame = mpc_stage_geometry::sample_course_frame(
    knots, trajectory.progress_m[index], std::max(1e-9, tolerance_m));
  if (!frame.has_value()) {
    return std::nullopt;
  }
  const auto world = contract::reconstruct_planar_pose_from_frenet(
    contract::PlanarPose{frame->x_m, frame->y_m, frame->heading_rad},
    contract::FrenetPose{
      trajectory.lateral_m[index], trajectory.lag_m[index],
      trajectory.heading_offset_rad[index]});
  if (!world.has_value()) {
    return std::nullopt;
  }
  return recovery::Pose2D{world->x_m, world->y_m, world->yaw_rad};
}

std::optional<recovery::Pose2D> reconstruct_affine_pose(
  const std::vector<mpc_stage_geometry::CourseFrameKnot> & knots,
  const mpcc_rate_resolved_execution_artifact::ExecutionArtifact & artifact,
  const std::size_t state_index,
  const double tolerance_m) noexcept
{
  if (state_index >= artifact.predicted_states.size()) {
    return std::nullopt;
  }
  const auto & state = artifact.predicted_states[state_index];
  const auto frame = mpc_stage_geometry::sample_course_frame(
    knots, artifact.course_progress_origin_m + state.progress_m,
    std::max(1e-9, tolerance_m));
  if (!frame.has_value()) {
    return std::nullopt;
  }
  const auto world = contract::reconstruct_planar_pose_from_frenet(
    contract::PlanarPose{frame->x_m, frame->y_m, frame->heading_rad},
    contract::FrenetPose{
      state.lateral_m, state.lag_m, state.heading_offset_rad});
  if (!world.has_value()) {
    return std::nullopt;
  }
  return recovery::Pose2D{world->x_m, world->y_m, world->yaw_rad};
}

DynamicModelDiagnostic diagnose_dynamic_model(
  const shadow::Snapshot & candidate,
  const shadow::ReplayWorld & replay,
  const mpcc_rate_resolved_execution_artifact::ExecutionArtifact & artifact,
  const race_mpcc_foundation::ExactPhysicalExecutionTrajectory & trajectory,
  const std::string & obstacle_id, const int pass_side_sign,
  const double rejected_elapsed_sec) noexcept
{
  DynamicModelDiagnostic result;
  const auto obstacle = std::find_if(
    replay.obstacles.begin(), replay.obstacles.end(),
    [&obstacle_id](const shadow::ReplayDynamicObstacle & item) {
      return item.id == obstacle_id;
    });
  if (
    obstacle == replay.obstacles.end() ||
    artifact.predicted_states.size() != artifact.control_stages.size() + 1U)
  {
    return result;
  }
  const recovery::CircleObstacle circle{
    obstacle->x_m, obstacle->y_m, obstacle->velocity_x_mps,
    obstacle->velocity_y_mps, obstacle->radius_m};
  const double control_origin_age_sec =
    candidate.control_prediction_origin_sec - replay.observed_sec;
  double stage_end_sec = 0.0;
  for (std::size_t stage = 0U; stage < artifact.control_stages.size(); ++stage) {
    stage_end_sec += artifact.control_stages[stage].duration_sec;
    const auto affine_pose = reconstruct_affine_pose(
      candidate.wall_course_frame_knots, artifact, stage + 1U,
      replay.bound_tolerance_m);
    const auto exact_time = std::lower_bound(
      trajectory.elapsed_time_sec.begin(), trajectory.elapsed_time_sec.end(),
      stage_end_sec - 1e-9);
    if (
      !affine_pose.has_value() || exact_time == trajectory.elapsed_time_sec.end())
    {
      return result;
    }
    const auto exact_index = static_cast<std::size_t>(
      std::distance(trajectory.elapsed_time_sec.begin(), exact_time));
    if (std::abs(trajectory.elapsed_time_sec[exact_index] - stage_end_sec) > 1e-7) {
      return result;
    }
    const auto nonlinear_pose = reconstruct_pose(
      candidate.wall_course_frame_knots, trajectory, exact_index,
      replay.bound_tolerance_m);
    if (!nonlinear_pose.has_value()) {
      return result;
    }
    const double world_elapsed_sec = control_origin_age_sec + stage_end_sec;
    const auto affine_clearance = recovery::circle_obstacle_clearance_at_time(
      replay.physical_footprint, affine_pose.value(), circle, world_elapsed_sec);
    const auto nonlinear_clearance = recovery::circle_obstacle_clearance_at_time(
      replay.physical_footprint, nonlinear_pose.value(), circle, world_elapsed_sec);
    if (!affine_clearance.has_value() || !nonlinear_clearance.has_value()) {
      return result;
    }
    if (affine_clearance.value() < result.minimum_affine_node_clearance_m) {
      result.minimum_affine_node_clearance_m = affine_clearance.value();
      result.minimum_affine_node_clearance_stage = static_cast<int>(stage);
    }
    if (nonlinear_clearance.value() < result.minimum_nonlinear_node_clearance_m) {
      result.minimum_nonlinear_node_clearance_m = nonlinear_clearance.value();
      result.minimum_nonlinear_node_clearance_stage = static_cast<int>(stage);
    }
    if (world_elapsed_sec <= rejected_elapsed_sec + 1e-9) {
      result.preceding_node_clearance_m = nonlinear_clearance.value();
      result.preceding_node_clearance_stage = static_cast<int>(stage);
    } else if (result.following_node_clearance_stage < 0) {
      result.following_node_clearance_m = nonlinear_clearance.value();
      result.following_node_clearance_stage = static_cast<int>(stage);
    }
    if (
      (pass_side_sign == -1 || pass_side_sign == 1) &&
      stage < candidate.dynamic_obstacle_stages.size() &&
      candidate.dynamic_obstacle_stages[stage].valid)
    {
      const auto & target = candidate.dynamic_obstacle_stages[stage];
      const auto & state = artifact.predicted_states[stage + 1U];
      const double behind_reserve_m =
        target.target_progress_m - target.longitudinal_overlap_m -
        (state.progress_m + state.lag_m);
      const double side_reserve_m =
        static_cast<double>(pass_side_sign) *
        (state.lateral_m - target.target_lateral_m) -
        target.lateral_center_separation_m;
      const double disjunction_reserve_m = std::max(
        behind_reserve_m, side_reserve_m);
      if (
        disjunction_reserve_m < result.minimum_full_disjunction_reserve_m)
      {
        result.minimum_full_disjunction_reserve_m = disjunction_reserve_m;
        result.minimum_full_disjunction_reserve_stage =
          static_cast<int>(stage);
      }
    }
    const double position_error_m = std::hypot(
      affine_pose->x_m - nonlinear_pose->x_m,
      affine_pose->y_m - nonlinear_pose->y_m);
    if (position_error_m > result.maximum_node_position_error_m) {
      result.maximum_node_position_error_m = position_error_m;
      result.maximum_node_position_error_stage = static_cast<int>(stage);
    }
  }
  result.complete = true;
  return result;
}

wall::Snapshot wall_snapshot(
  const shadow::Snapshot & candidate,
  const shadow::ReplayWorld & replay,
  const race_mpcc_foundation::ExactPhysicalExecutionTrajectory & trajectory)
{
  wall::Snapshot result;
  result.identity.artifact = candidate.identity;
  result.identity.captured_sec = candidate.identity.snapshot_sec;
  result.identity.pose_snapshot_id = wall::fingerprint_control_pose_path(
    replay.control_prefix, replay.control_prefix.back());
  result.identity.course_frame_window_id =
    wall::fingerprint_course_frame_window(candidate.wall_course_frame_knots);
  result.wall_grid = candidate.wall_grid;
  result.wall_grid_fingerprint = replay.wall_grid_fingerprint;
  result.footprint = replay.physical_footprint;
  result.current_pose = replay.current_pose;
  result.control_prefix = replay.control_prefix;
  result.trajectory = trajectory;
  result.course_frame_knots = candidate.wall_course_frame_knots;
  // The comparison arms must prove the same terminal successor as production.
  // Keep this geometry independent from the short executable artifact: it is
  // reconstructed only from the immutable, full-horizon source problem that
  // every arm shares.
  result.terminal_stop_course_geometry.progress_m =
    candidate.wall_reference_progress_m;
  result.terminal_stop_course_geometry.lateral_lower_m =
    candidate.wall_lower_m;
  result.terminal_stop_course_geometry.lateral_upper_m =
    candidate.wall_upper_m;
  result.terminal_stop_course_geometry.curvature_radpm.reserve(
    candidate.request.inputs.size());
  for (const auto & input : candidate.request.inputs) {
    result.terminal_stop_course_geometry.curvature_radpm.push_back(
      input.path_curvature_radpm);
  }
  result.hard_wall_clearance_m = replay.hard_wall_clearance_m;
  // This is a post-solve physical certificate. Its tolerance is sealed by the
  // exact execution trajectory, not by the pre-solve ReplayWorld baseline.
  result.bound_tolerance_m = trajectory.lateral_bound_tolerance_m;
  result.swept_step_m = replay.swept_step_m;
  return result;
}

struct TerminalStopCertificate
{
  bool accepted{false};
  race_mpcc_foundation::ExactPhysicalExecutionTrajectory trajectory;
  wall::Result wall_certificate;
  dynamic::Result dynamic_certificate;
  std::string detail{"not-evaluated"};
};

enum class TerminalStopLateralAuditMode
{
  RacingLine,
  Declared,
  PhysicalRangeScan,
  NormalPathProfile,
  SolvedStopTrajectory,
};

std::optional<physical::StopLateralTargetProfile>
build_normal_path_stop_profile(
  const artifact::ExecutionArtifact & execution) noexcept
{
  if (
    artifact::validate(execution) != artifact::RejectReason::None ||
    execution.predicted_states.size() < 2U)
  {
    return std::nullopt;
  }
  physical::StopLateralTargetProfile profile;
  profile.progress_m.reserve(execution.predicted_states.size());
  profile.lateral_m.reserve(execution.predicted_states.size());
  for (const auto & state : execution.predicted_states) {
    profile.progress_m.push_back(state.progress_m);
    profile.lateral_m.push_back(state.lateral_m);
  }
  return physical::stop_lateral_target_profile_valid(profile) ?
    std::optional<physical::StopLateralTargetProfile>{std::move(profile)} :
    std::nullopt;
}

TerminalStopCertificate certify_terminal_stop(
  const shadow::Snapshot & candidate,
  const artifact::ExecutionArtifact & execution,
  const wall::Snapshot & normal_wall_snapshot,
  const double target_lateral_m = 0.0,
  const physical::StopLateralTargetProfile * const target_profile = nullptr)
{
  TerminalStopCertificate result;
  if (!candidate.replay_world.has_value() ||
    !candidate.replay_world->terminal_stop_contract_available)
  {
    result.detail =
      "terminal Stop contract absent from immutable interaction snapshot";
    return result;
  }
  if (execution.predicted_states.empty()) {
    result.detail = "terminal Stop initial state unavailable";
    return result;
  }
  const auto cursor = artifact::resolve_cursor(
    execution, execution.prediction_origin_sec);
  const auto actuation = artifact::extract_actuation(execution, cursor);
  if (!cursor.available || !actuation.actuation.has_value()) {
    std::ostringstream detail;
    detail << "terminal Stop first actuation unavailable/cursor="
           << artifact::to_string(cursor.reason) << "/actuation="
           << artifact::to_string(actuation.reason);
    result.detail = detail.str();
    return result;
  }
  const auto & initial = execution.predicted_states.front();
  const auto & world = candidate.replay_world.value();
  const auto terminal = physical::build_stop_contingency(
    execution, cursor, actuation.actuation.value(),
    physical::ContinuationInitialState{
      initial.lateral_m, initial.lag_m, initial.heading_offset_rad,
      initial.velocity_mps, initial.progress_m, initial.steering_rad,
      initial.response_steering_rad},
    normal_wall_snapshot.terminal_stop_course_geometry,
    world.terminal_stop_lateral_policy,
    world.terminal_stop_minimum_acceleration_mps2, target_lateral_m,
    target_profile);
  if (!terminal.exact_trajectory.has_value()) {
    std::ostringstream detail;
    detail << "terminal Stop synthesis rejected/reason="
           << physical::to_string(terminal.reason)
           << "/exact=" << static_cast<int>(terminal.exact_reason)
           << "/sample=" << terminal.rejected_sample;
    result.detail = detail.str();
    return result;
  }
  auto terminal_wall_snapshot = normal_wall_snapshot;
  terminal_wall_snapshot.trajectory = terminal.exact_trajectory.value();
  result.wall_certificate = wall::evaluate(terminal_wall_snapshot);
  if (result.wall_certificate.outcome != wall::Outcome::Accepted) {
    std::ostringstream detail;
    detail << "terminal Stop wall rejected/outcome="
           << wall::to_string(result.wall_certificate.outcome)
           << "/detail=" << result.wall_certificate.detail;
    result.detail = detail.str();
    return result;
  }
  result.dynamic_certificate = dynamic::evaluate_current_world(
    candidate, terminal_wall_snapshot);
  if (!result.dynamic_certificate.valid ||
    !result.dynamic_certificate.clear)
  {
    std::ostringstream detail;
    detail << "terminal Stop dynamic rejected/valid="
           << (result.dynamic_certificate.valid ? 1 : 0)
           << "/clear=" << (result.dynamic_certificate.clear ? 1 : 0)
           << "/obstacle="
           << result.dynamic_certificate.blocking_obstacle_id
           << "/minimum="
           << result.dynamic_certificate.minimum_clearance_m;
    result.detail = detail.str();
    return result;
  }
  result.trajectory = terminal.exact_trajectory.value();
  result.accepted = true;
  result.detail = "accepted";
  return result;
}

ArmResult evaluate_arm(
  const Arm arm,
  shadow::Snapshot candidate,
  const std::uint64_t source_fingerprint,
  const std::uint64_t candidate_fingerprint,
  const maneuver::TerminalResolution & successor,
  const int lattice_transition_stage = -1,
  const int lattice_ahead_stage = -1,
  shadow::SolverContext * solver_context = nullptr,
  const bool wall_restoration_audit = false,
  const std::optional<shadow::SolverContext::WallBucketAuditMode>
  wall_bucket_audit_mode = std::nullopt,
  const std::size_t physical_dynamic_sqp_audit_iteration_count = 0U,
  const TerminalStopLateralAuditMode terminal_stop_lateral_audit_mode =
  TerminalStopLateralAuditMode::RacingLine,
  const std::vector<double> * const fixed_steering_rate_radps = nullptr)
{
  ArmResult arm_result;
  arm_result.arm = arm;
  arm_result.source_interaction_fingerprint = source_fingerprint;
  arm_result.candidate_fingerprint = candidate_fingerprint;
  arm_result.lattice_transition_stage = lattice_transition_stage;
  arm_result.lattice_ahead_stage = lattice_ahead_stage;
  arm_result.dynamic_sqp_depth =
    physical_dynamic_sqp_audit_iteration_count;
  if (!successor.accepted) {
    arm_result.stage = Stage::TerminalSuccessorRejected;
    arm_result.detail = successor.detail;
    return arm_result;
  }
  if (!candidate.replay_world.has_value()) {
    arm_result.stage = Stage::SourceRejected;
    arm_result.detail = "replay world unavailable";
    return arm_result;
  }

  // A/B/C use a local context and never share lifecycle state. Candidate D
  // passes one private context only across its bounded offline continuation.
  shadow::SolverContext local_solver;
  auto & solver = solver_context == nullptr ? local_solver : *solver_context;
  const auto solved = fixed_steering_rate_radps != nullptr ?
    solver.evaluate_fixed_steering_rate_audit(
    candidate, *fixed_steering_rate_radps) :
    (physical_dynamic_sqp_audit_iteration_count > 0U ?
    solver.evaluate_physical_dynamic_sqp_audit(
      candidate, physical_dynamic_sqp_audit_iteration_count) :
    (wall_bucket_audit_mode.has_value() ?
    solver.evaluate_wall_bucket_audit(
      candidate, wall_bucket_audit_mode.value()) :
    (wall_restoration_audit ?
    solver.evaluate_wall_feasibility_restoration_audit(candidate) :
    solver.evaluate(candidate))));
  arm_result.solver_outcome = solved.outcome;
  arm_result.solver_compute_ms = solved.compute_ms;
  arm_result.terminal_progress_m = solved.terminal_progress_m;
  arm_result.terminal_velocity_mps = solved.terminal_velocity_mps;
  if (
    solved.outcome != shadow::Outcome::Solved ||
    solved.execution_artifact == nullptr)
  {
    arm_result.stage = Stage::SolverRejected;
    arm_result.detail = solved.detail;
    return arm_result;
  }
  const auto adapted = physical::build(
    *solved.execution_artifact,
    candidate.identity.source_context.intent,
    candidate.identity.source_context.stage_geometry_id);
  if (!adapted.exact_trajectory.has_value()) {
    arm_result.stage = Stage::ExactTrajectoryRejected;
    std::ostringstream detail;
    detail << "adapter=" << physical::to_string(adapted.reason)
           << "/stage=" << adapted.rejected_stage;
    arm_result.detail = detail.str();
    return arm_result;
  }
  auto exact = adapted.exact_trajectory.value();
  arm_result.minimum_lateral_bound_reserve_m =
    exact.minimum_lateral_bound_reserve_m;
  auto physical_snapshot = wall_snapshot(
    candidate, candidate.replay_world.value(), exact);
  auto wall_result = wall::evaluate(physical_snapshot);
  if (wall_result.outcome != wall::Outcome::Accepted) {
    arm_result.stage = Stage::WallProofRejected;
    arm_result.detail = wall_result.detail;
    return arm_result;
  }
  auto dynamic_result = dynamic::evaluate_current_world(
    candidate, physical_snapshot);
  if (!dynamic_result.valid || !dynamic_result.clear) {
    arm_result.stage = Stage::DynamicProofRejected;
    const auto & replay = candidate.replay_world.value();
    const double control_origin_age_sec =
      candidate.control_prediction_origin_sec - replay.observed_sec;
    const bool prefix_scope =
      std::isfinite(dynamic_result.rejected_elapsed_sec) &&
      dynamic_result.rejected_elapsed_sec <= control_origin_age_sec + 1e-9;
    int qp_stage = -1;
    if (
      std::isfinite(dynamic_result.rejected_elapsed_sec) && !prefix_scope)
    {
      const double solver_elapsed_sec =
        dynamic_result.rejected_elapsed_sec - control_origin_age_sec;
      double stage_end_sec = 0.0;
      for (std::size_t stage = 0U; stage < candidate.request.inputs.size(); ++stage) {
        stage_end_sec += candidate.request.inputs[stage].stage_dt_sec;
        if (solver_elapsed_sec <= stage_end_sec + 1e-9) {
          qp_stage = static_cast<int>(stage);
          break;
        }
      }
    }
    const auto model_diagnostic = diagnose_dynamic_model(
      candidate, replay, *solved.execution_artifact, exact,
      dynamic_result.blocking_obstacle_id,
      solved.dynamic_obstacle_resolved_side_sign,
      dynamic_result.rejected_elapsed_sec);
    std::ostringstream detail;
    detail << "obstacle=" << dynamic_result.blocking_obstacle_id
           << "/checked=" << dynamic_result.checked_pose_count
           << "/dynamic_side=" << solved.dynamic_obstacle_resolved_side_sign
           << "/first_side_stage="
           << solved.dynamic_obstacle_first_pass_side_stage
           << "/behind_rows="
           << solved.dynamic_obstacle_stay_behind_row_count
           << "/side_rows=" << solved.dynamic_obstacle_pass_side_row_count
           << "/ahead_rows=" << solved.dynamic_obstacle_ahead_row_count
           << "/diagonal_rows="
           << solved.dynamic_obstacle_diagonal_row_count
           << "/physical_axis="
           << (solved.dynamic_obstacle_physical_axis_support_applied ? 1 : 0)
           << "/physical_diagonal="
           << (solved.dynamic_obstacle_physical_diagonal_guidance_applied ? 1 : 0)
           << "/reason=" << recovery::to_string(dynamic_result.rejection_reason)
           << "/scope=" << (prefix_scope ? "control-prefix" : "candidate")
           << "/qp_stage=" << qp_stage
           << "/rejected_t=" << dynamic_result.rejected_elapsed_sec
           << "/rejected_clearance=" << dynamic_result.rejected_clearance_m
           << "/rejected_pose=[" << dynamic_result.rejected_pose.x_m << ','
           << dynamic_result.rejected_pose.y_m << ','
           << dynamic_result.rejected_pose.yaw_rad << ']'
           << "/minimum_obstacle="
           << dynamic_result.minimum_clearance_obstacle_id
           << "/minimum_t=" << dynamic_result.minimum_clearance_elapsed_sec
           << "/minimum_clearance=" << dynamic_result.minimum_clearance_m;
    if (model_diagnostic.complete) {
      detail << "/affine_node_min="
             << model_diagnostic.minimum_affine_node_clearance_m
             << "@" << model_diagnostic.minimum_affine_node_clearance_stage
             << "/nonlinear_node_min="
             << model_diagnostic.minimum_nonlinear_node_clearance_m
             << "@" << model_diagnostic.minimum_nonlinear_node_clearance_stage
             << "/node_pose_error_max="
             << model_diagnostic.maximum_node_position_error_m
             << "@" << model_diagnostic.maximum_node_position_error_stage
             << "/full_disjunction_min="
             << model_diagnostic.minimum_full_disjunction_reserve_m
             << "@" << model_diagnostic.minimum_full_disjunction_reserve_stage
             << "/rejection_node_bracket="
             << model_diagnostic.preceding_node_clearance_m
             << "@" << model_diagnostic.preceding_node_clearance_stage
             << "->" << model_diagnostic.following_node_clearance_m
             << "@" << model_diagnostic.following_node_clearance_stage;
    } else {
      detail << "/node_model_diagnostic=unavailable";
    }
    arm_result.detail = detail.str();
    return arm_result;
  }

  double terminal_stop_target_lateral_m = 0.0;
  std::size_t terminal_stop_target_attempt_count = 0U;
  TerminalStopCertificate terminal_stop;
  if (
    terminal_stop_lateral_audit_mode ==
    TerminalStopLateralAuditMode::PhysicalRangeScan)
  {
    const auto & terminal_geometry =
      physical_snapshot.terminal_stop_course_geometry;
    const double target_lower_m = *std::min_element(
      terminal_geometry.lateral_lower_m.begin(),
      terminal_geometry.lateral_lower_m.end());
    const double target_upper_m = *std::max_element(
      terminal_geometry.lateral_upper_m.begin(),
      terminal_geometry.lateral_upper_m.end());
    const double target_step_m =
      std::isfinite(candidate.wall_lateral_sample_step_m) &&
      candidate.wall_lateral_sample_step_m > 0.0 ?
      candidate.wall_lateral_sample_step_m : 0.10;
    constexpr std::size_t kMaximumAuditTargets = 128U;
    const std::size_t requested_target_count =
      static_cast<std::size_t>(std::ceil(
        std::max(0.0, target_upper_m - target_lower_m) / target_step_m)) + 1U;
    const std::size_t target_count = std::clamp<std::size_t>(
      requested_target_count, 1U, kMaximumAuditTargets);
    for (std::size_t target_index = 0U;
      target_index < target_count; ++target_index)
    {
      const double fraction = target_count <= 1U ? 0.0 :
        static_cast<double>(target_index) /
        static_cast<double>(target_count - 1U);
      const double target_m = target_lower_m +
        fraction * (target_upper_m - target_lower_m);
      ++terminal_stop_target_attempt_count;
      auto evaluated = certify_terminal_stop(
        candidate, *solved.execution_artifact, physical_snapshot, target_m);
      if (evaluated.accepted) {
        terminal_stop_target_lateral_m = target_m;
        terminal_stop = std::move(evaluated);
        break;
      }
      if (terminal_stop_target_attempt_count == 1U) {
        terminal_stop_target_lateral_m = target_m;
        terminal_stop = std::move(evaluated);
      }
    }
  } else if (
    terminal_stop_lateral_audit_mode ==
    TerminalStopLateralAuditMode::NormalPathProfile)
  {
    terminal_stop_target_attempt_count = 1U;
    const auto profile = build_normal_path_stop_profile(
      *solved.execution_artifact);
    if (!profile.has_value()) {
      terminal_stop.detail = "normal-path Stop profile unavailable";
    } else {
      terminal_stop = certify_terminal_stop(
        candidate, *solved.execution_artifact, physical_snapshot, 0.0,
        &profile.value());
    }
  } else if (
    terminal_stop_lateral_audit_mode ==
    TerminalStopLateralAuditMode::SolvedStopTrajectory)
  {
    terminal_stop_target_attempt_count = 1U;
    const auto & stop_artifact = *solved.execution_artifact;
    arm_result.solved_control_duration_sec.reserve(
      stop_artifact.control_stages.size());
    arm_result.solved_acceleration_mps2.reserve(
      stop_artifact.control_stages.size());
    arm_result.solved_steering_rate_radps.reserve(
      stop_artifact.control_stages.size());
    arm_result.solved_acceleration_min_mps2 =
      std::numeric_limits<double>::infinity();
    arm_result.solved_acceleration_max_mps2 =
      -std::numeric_limits<double>::infinity();
    arm_result.solved_steering_rate_min_radps =
      std::numeric_limits<double>::infinity();
    arm_result.solved_steering_rate_max_radps =
      -std::numeric_limits<double>::infinity();
    double total_duration_sec = 0.0;
    double acceleration_time_integral = 0.0;
    double steering_rate_time_integral = 0.0;
    int previous_nonzero_steering_rate_sign = 0;
    const double control_tolerance = std::max(
      1e-9, stop_artifact.physical_global_tolerance);
    for (std::size_t stage = 0U;
      stage < stop_artifact.control_stages.size(); ++stage)
    {
      const auto & control = stop_artifact.control_stages[stage];
      arm_result.solved_control_duration_sec.push_back(control.duration_sec);
      arm_result.solved_acceleration_mps2.push_back(
        control.acceleration_mps2);
      arm_result.solved_steering_rate_radps.push_back(
        control.steering_rate_radps);
      arm_result.solved_acceleration_min_mps2 = std::min(
        arm_result.solved_acceleration_min_mps2,
        control.acceleration_mps2);
      arm_result.solved_acceleration_max_mps2 = std::max(
        arm_result.solved_acceleration_max_mps2,
        control.acceleration_mps2);
      arm_result.solved_steering_rate_min_radps = std::min(
        arm_result.solved_steering_rate_min_radps,
        control.steering_rate_radps);
      arm_result.solved_steering_rate_max_radps = std::max(
        arm_result.solved_steering_rate_max_radps,
        control.steering_rate_radps);
      total_duration_sec += control.duration_sec;
      acceleration_time_integral +=
        control.acceleration_mps2 * control.duration_sec;
      steering_rate_time_integral +=
        control.steering_rate_radps * control.duration_sec;
      if (stage < candidate.request.inputs.size()) {
        const auto & input = candidate.request.inputs[stage];
        const auto acceleration_bounds =
          mpcc_rate_resolved_adapter::resolve_exact_physical_boundary_bounds(
          input.lower[mpcc_rate_resolved::kAccelerationIndex],
          input.upper[mpcc_rate_resolved::kAccelerationIndex],
          solver.physical_constraint_tolerance());
        if (
          acceleration_bounds.has_value() &&
          (std::abs(
            control.acceleration_mps2 - acceleration_bounds->lower) <=
          control_tolerance ||
          std::abs(
            control.acceleration_mps2 - acceleration_bounds->upper) <=
          control_tolerance))
        {
          ++arm_result.solved_acceleration_bound_count;
        }
        const double physical_rate_lower = stage == 0U ? std::max(
            -candidate.request.maximum_abs_steering_rate_radps,
            (-candidate.request.maximum_abs_steering_rad -
            candidate.request.current_steering_rad) /
            input.stage_dt_sec) :
          -candidate.request.maximum_abs_steering_rate_radps;
        const double physical_rate_upper = stage == 0U ? std::min(
            candidate.request.maximum_abs_steering_rate_radps,
            (candidate.request.maximum_abs_steering_rad -
            candidate.request.current_steering_rad) /
            input.stage_dt_sec) :
          candidate.request.maximum_abs_steering_rate_radps;
        const auto steering_rate_bounds =
          mpcc_rate_resolved_adapter::resolve_exact_physical_boundary_bounds(
          physical_rate_lower, physical_rate_upper,
          solver.physical_constraint_tolerance());
        if (
          steering_rate_bounds.has_value() &&
          (std::abs(
            control.steering_rate_radps - steering_rate_bounds->lower) <=
          control_tolerance ||
          std::abs(
            control.steering_rate_radps - steering_rate_bounds->upper) <=
          control_tolerance))
        {
          ++arm_result.solved_steering_rate_bound_count;
        }
      }
      const int steering_rate_sign =
        control.steering_rate_radps > control_tolerance ? 1 :
        (control.steering_rate_radps < -control_tolerance ? -1 : 0);
      if (
        steering_rate_sign != 0 &&
        previous_nonzero_steering_rate_sign != 0 &&
        steering_rate_sign != previous_nonzero_steering_rate_sign)
      {
        ++arm_result.solved_steering_rate_sign_change_count;
      }
      if (steering_rate_sign != 0) {
        previous_nonzero_steering_rate_sign = steering_rate_sign;
      }
    }
    if (total_duration_sec > 0.0) {
      arm_result.solved_acceleration_time_mean_mps2 =
        acceleration_time_integral / total_duration_sec;
      arm_result.solved_steering_rate_time_mean_radps =
        steering_rate_time_integral / total_duration_sec;
    }
    if (
      exact.velocity_mps.empty() ||
      exact.velocity_mps.back() > std::max(
        1e-9, solved.execution_artifact->physical_global_tolerance))
    {
      terminal_stop.detail = "seven-state Stop did not reach rest";
    } else {
      terminal_stop.accepted = true;
      terminal_stop.trajectory = exact;
      terminal_stop.wall_certificate = wall_result;
      terminal_stop.dynamic_certificate = dynamic_result;
      terminal_stop.detail = "accepted/seven-state-stop";
    }
  } else {
    terminal_stop_target_lateral_m =
      terminal_stop_lateral_audit_mode ==
      TerminalStopLateralAuditMode::Declared &&
      successor.stop_suffix.available ?
      successor.stop_suffix.hold_lateral_m : 0.0;
    terminal_stop_target_attempt_count = 1U;
    terminal_stop = certify_terminal_stop(
      candidate, *solved.execution_artifact, physical_snapshot,
      terminal_stop_target_lateral_m);
  }
  arm_result.terminal_stop_target_lateral_m =
    terminal_stop_target_lateral_m;
  arm_result.terminal_stop_target_attempt_count =
    terminal_stop_target_attempt_count;
  if (!terminal_stop.accepted) {
    arm_result.stage = Stage::TerminalSuccessorRejected;
    std::ostringstream detail;
    detail << terminal_stop.detail << "/stop_target_lateral="
           << terminal_stop_target_lateral_m << "/stop_target_attempts="
           << terminal_stop_target_attempt_count;
    arm_result.detail = detail.str();
    return arm_result;
  }

  ManeuverBundle bundle;
  bundle.target_id = candidate.identity.source_context.target_id;
  bundle.pass_side_sign =
    candidate.identity.source_context.execution_side_sign != 0 ?
    candidate.identity.source_context.execution_side_sign :
    candidate.dynamic_obstacle_pass_side_sign;
  bundle.source_interaction_fingerprint = source_fingerprint;
  bundle.candidate_fingerprint = candidate_fingerprint;
  bundle.exact_trajectory = std::move(exact);
  bundle.wall_certificate = std::move(wall_result);
  bundle.dynamic_certificate = std::move(dynamic_result);
  bundle.terminal_stop_trajectory = terminal_stop.trajectory;
  bundle.terminal_stop_wall_certificate = terminal_stop.wall_certificate;
  bundle.terminal_stop_dynamic_certificate = terminal_stop.dynamic_certificate;
  bundle.terminal_successor = successor.successor;
  bundle.stop_suffix = successor.stop_suffix;
  arm_result.stage = Stage::Accepted;
  if (wall_bucket_audit_mode.has_value()) {
    switch (wall_bucket_audit_mode.value()) {
      case shadow::SolverContext::WallBucketAuditMode::OmitHeading:
        arm_result.detail = "accepted/wall-bucket-audit=omit-heading";
        break;
      case shadow::SolverContext::WallBucketAuditMode::OmitLag:
        arm_result.detail = "accepted/wall-bucket-audit=omit-lag";
        break;
      case shadow::SolverContext::WallBucketAuditMode::OmitPose:
        arm_result.detail = "accepted/wall-bucket-audit=omit-pose";
        break;
      case shadow::SolverContext::WallBucketAuditMode::OmitPoseDirect:
        arm_result.detail = "accepted/wall-bucket-audit=omit-pose-direct";
        break;
    }
  } else {
    arm_result.detail =
      (wall_restoration_audit ||
      physical_dynamic_sqp_audit_iteration_count > 0U) ?
      solved.detail : "accepted";
  }
  arm_result.bundle = std::move(bundle);
  return arm_result;
}

ArmResult rejected_arm(
  const Arm arm, const Stage stage, const std::uint64_t source_fingerprint,
  std::string detail)
{
  ArmResult result;
  result.arm = arm;
  result.stage = stage;
  result.source_interaction_fingerprint = source_fingerprint;
  result.detail = std::move(detail);
  return result;
}

ArmResult evaluate_offline_continuation(
  const Arm arm, const shadow::Snapshot & source,
  const std::uint64_t source_fingerprint, const int side,
  const int transition_stage, const int ahead_stage)
{
  const auto exact_candidate = maneuver::build_disjunction_schedule(
    source, source_fingerprint, side, transition_stage, ahead_stage, 1.0);
  if (!exact_candidate.seed.has_value()) {
    const auto stage =
      exact_candidate.reason ==
      maneuver::RejectReason::TerminalSuccessorUnavailable ?
      Stage::TerminalSuccessorRejected : Stage::CandidateRejected;
    auto rejected = rejected_arm(
      arm, stage, source_fingerprint,
      std::string{maneuver::to_string(exact_candidate.reason)} + ": " +
      exact_candidate.detail);
    rejected.lattice_transition_stage = transition_stage;
    rejected.lattice_ahead_stage = ahead_stage;
    return rejected;
  }
  const auto successor = resolve_audit_terminal_successor(
    exact_candidate.seed->solver_snapshot);
  auto direct = evaluate_arm(
    arm, exact_candidate.seed->solver_snapshot, source_fingerprint,
    exact_candidate.seed->candidate_fingerprint, successor,
    transition_stage, ahead_stage);
  direct.direct_final_attempted = true;
  direct.direct_final_stage = direct.stage;
  if (direct.bundle.has_value()) {
    direct.detail = "direct-final accepted";
    return direct;
  }

  constexpr std::array<double, 4> kContinuationFractions{
    0.0, 0.25, 0.50, 0.75};
  shadow::SolverContext continuation_solver;
  std::size_t solved_step_count = 0U;
  double continuation_compute_ms = 0.0;
  for (const double fraction : kContinuationFractions) {
    const auto intermediate = maneuver::build_disjunction_schedule(
      source, source_fingerprint, side, transition_stage, ahead_stage,
      fraction);
    if (!intermediate.seed.has_value()) {
      auto rejected = rejected_arm(
        arm, Stage::CandidateRejected, source_fingerprint,
        std::string{"continuation candidate rejected at fraction="} +
        std::to_string(fraction) + ": " +
        maneuver::to_string(intermediate.reason) + ": " +
        intermediate.detail);
      rejected.candidate_fingerprint =
        exact_candidate.seed->candidate_fingerprint;
      rejected.lattice_transition_stage = transition_stage;
      rejected.lattice_ahead_stage = ahead_stage;
      rejected.direct_final_attempted = true;
      rejected.direct_final_stage = direct.stage;
      rejected.continuation_attempted = true;
      rejected.continuation_solved_step_count = solved_step_count;
      rejected.continuation_compute_ms = continuation_compute_ms;
      return rejected;
    }
    const auto solved = continuation_solver.evaluate(
      intermediate.seed->solver_snapshot);
    continuation_compute_ms += solved.compute_ms;
    if (
      solved.outcome != shadow::Outcome::Solved ||
      solved.execution_artifact == nullptr)
    {
      ArmResult rejected;
      rejected.arm = arm;
      rejected.stage = Stage::SolverRejected;
      rejected.source_interaction_fingerprint = source_fingerprint;
      rejected.candidate_fingerprint =
        exact_candidate.seed->candidate_fingerprint;
      rejected.solver_outcome = solved.outcome;
      rejected.solver_compute_ms = solved.compute_ms;
      rejected.lattice_transition_stage = transition_stage;
      rejected.lattice_ahead_stage = ahead_stage;
      rejected.direct_final_attempted = true;
      rejected.direct_final_stage = direct.stage;
      rejected.continuation_attempted = true;
      rejected.continuation_solved_step_count = solved_step_count;
      rejected.continuation_compute_ms = continuation_compute_ms;
      std::ostringstream detail;
      detail << "direct=" << to_string(direct.stage)
             << "/continuation_fraction=" << fraction
             << "/continuation=" << solved.detail;
      rejected.detail = detail.str();
      return rejected;
    }
    ++solved_step_count;
  }

  auto continued = evaluate_arm(
    arm, exact_candidate.seed->solver_snapshot, source_fingerprint,
    exact_candidate.seed->candidate_fingerprint, successor,
    transition_stage, ahead_stage, &continuation_solver);
  continuation_compute_ms += continued.solver_compute_ms;
  continued.direct_final_attempted = true;
  continued.direct_final_stage = direct.stage;
  continued.continuation_attempted = true;
  continued.continuation_solved_step_count = solved_step_count;
  continued.continuation_compute_ms = continuation_compute_ms;
  const auto final_detail = continued.detail;
  std::ostringstream detail;
  detail << "direct=" << to_string(direct.stage)
         << "/continuation_steps=" << solved_step_count
         << "/final=" << final_detail;
  continued.detail = detail.str();
  return continued;
}

std::vector<std::pair<int, int>> return_rejoin_schedule_lattice(
  const int horizon)
{
  std::vector<std::pair<int, int>> schedules;
  const auto append = [&schedules, horizon](const int start, const int complete) {
      if (start < 0 || start >= horizon || complete <= start ||
        complete > horizon)
      {
        return;
      }
      const auto value = std::pair{start, complete};
      if (std::find(schedules.begin(), schedules.end(), value) == schedules.end()) {
        schedules.push_back(value);
      }
    };
  for (const int start : {0, horizon / 4, horizon / 2}) {
    for (const int complete : {horizon / 2, 3 * horizon / 4, horizon}) {
      append(start, complete);
    }
  }
  if (schedules.empty() && horizon > 0) {
    append(0, horizon);
  }
  return schedules;
}

ArmResult evaluate_offline_return_rejoin(
  const shadow::Snapshot & source,
  const std::uint64_t source_fingerprint,
  const int rejoin_start_stage, const int rejoin_complete_stage)
{
  const auto candidate = maneuver::build_return_rejoin_schedule(
    source, source_fingerprint, rejoin_start_stage, rejoin_complete_stage);
  if (!candidate.seed.has_value()) {
    const auto stage =
      candidate.reason == maneuver::RejectReason::TerminalSuccessorUnavailable ?
      Stage::TerminalSuccessorRejected : Stage::CandidateRejected;
    auto rejected = rejected_arm(
      Arm::OfflineReturnD, stage, source_fingerprint,
      std::string{maneuver::to_string(candidate.reason)} + ": " +
      candidate.detail);
    rejected.lattice_transition_stage = rejoin_start_stage;
    rejected.lattice_ahead_stage = rejoin_complete_stage;
    return rejected;
  }
  const auto successor = resolve_audit_terminal_successor(
    candidate.seed->solver_snapshot);
  auto direct = evaluate_arm(
    Arm::OfflineReturnD, candidate.seed->solver_snapshot,
    source_fingerprint, candidate.seed->candidate_fingerprint, successor,
    rejoin_start_stage, rejoin_complete_stage);
  direct.direct_final_attempted = true;
  direct.direct_final_stage = direct.stage;
  direct.candidate_source = "return-rejoin-polynomial";
  if (direct.bundle.has_value()) {
    direct.detail = "direct-final accepted";
    return direct;
  }

  constexpr std::size_t kOfflineSqpDepth = 3U;
  auto refined = evaluate_arm(
    Arm::OfflineReturnD, candidate.seed->solver_snapshot,
    source_fingerprint, candidate.seed->candidate_fingerprint, successor,
    rejoin_start_stage, rejoin_complete_stage, nullptr, false, std::nullopt,
    kOfflineSqpDepth);
  refined.direct_final_attempted = true;
  refined.direct_final_stage = direct.stage;
  refined.continuation_attempted = true;
  refined.continuation_solved_step_count = refined.dynamic_sqp_depth;
  refined.continuation_compute_ms = refined.solver_compute_ms;
  refined.candidate_source = "return-rejoin-polynomial";
  refined.detail = std::string{"direct="} + to_string(direct.stage) +
    "/offline_multi_sqp_depth=" + std::to_string(kOfflineSqpDepth) +
    "/final=" + refined.detail;
  return refined;
}

int evidence_rank(Stage stage) noexcept;

ArmResult evaluate_normal_avoidance_lattice_population(
  const Arm arm, const shadow::Snapshot & source,
  const std::uint64_t source_fingerprint, const int side,
  const std::size_t dynamic_sqp_depth)
{
  ArmResult best = rejected_arm(
    arm, Stage::CandidateRejected, source_fingerprint,
    "normal-avoidance lattice was not evaluated");
  int best_rank = -1;
  std::size_t attempted = 0U;
  const int horizon = source.request.horizon_steps;
  for (int transition_stage = 0;
    transition_stage < horizon; ++transition_stage)
  {
    for (int ahead_stage = transition_stage + 1;
      ahead_stage <= horizon; ++ahead_stage)
    {
      ++attempted;
      const auto candidate = maneuver::build_normal_avoidance_schedule(
        source, source_fingerprint, side, transition_stage, ahead_stage);
      ArmResult evaluated;
      if (!candidate.seed.has_value()) {
        const auto stage =
          candidate.reason == maneuver::RejectReason::TerminalSuccessorUnavailable ?
          Stage::TerminalSuccessorRejected : Stage::CandidateRejected;
        evaluated = rejected_arm(
          arm, stage, source_fingerprint,
          std::string{maneuver::to_string(candidate.reason)} + ": " +
          candidate.detail);
        evaluated.lattice_transition_stage = transition_stage;
        evaluated.lattice_ahead_stage = ahead_stage;
      } else {
        const auto successor = resolve_audit_terminal_successor(
          candidate.seed->solver_snapshot);
        evaluated = evaluate_arm(
          arm, candidate.seed->solver_snapshot, source_fingerprint,
          candidate.seed->candidate_fingerprint, successor,
          transition_stage, ahead_stage, nullptr, false, std::nullopt,
          dynamic_sqp_depth);
      }
      evaluated.candidate_source = "normal-avoidance-lattice";
      evaluated.candidate_count = attempted;
      const int rank = evidence_rank(evaluated.stage);
      if (rank > best_rank) {
        best_rank = rank;
        best = std::move(evaluated);
      }
      if (best.stage == Stage::Accepted) {
        best.detail = std::string{"accepted/normal-avoidance-lattice/depth="} +
          std::to_string(dynamic_sqp_depth) + "/attempts=" +
          std::to_string(attempted) + "/" + best.detail;
        return best;
      }
    }
  }
  best.candidate_count = attempted;
  best.detail = std::string{"no certified normal-avoidance lattice/depth="} +
    std::to_string(dynamic_sqp_depth) + "/attempts=" +
    std::to_string(attempted) + "/best=" + best.detail;
  return best;
}

int evidence_rank(const Stage stage) noexcept
{
  switch (stage) {
    case Stage::Accepted: return 6;
    case Stage::DynamicProofRejected: return 5;
    case Stage::WallProofRejected: return 4;
    case Stage::ExactTrajectoryRejected: return 3;
    case Stage::SolverRejected: return 2;
    case Stage::TerminalSuccessorRejected:
    case Stage::CandidateRejected: return 1;
    case Stage::SourceRejected: return 0;
  }
  return -1;
}

ArmResult evaluate_proof_guided_candidate(
  const Arm arm, const shadow::Snapshot & candidate,
  const std::uint64_t source_fingerprint,
  const std::uint64_t candidate_fingerprint,
  const maneuver::TerminalResolution & successor,
  const int lattice_transition_stage,
  const int lattice_ahead_stage)
{
  auto baseline = evaluate_arm(
    arm, candidate, source_fingerprint, candidate_fingerprint, successor,
    lattice_transition_stage, lattice_ahead_stage);
  const auto baseline_stage = baseline.stage;
  if (baseline.bundle.has_value()) {
    baseline.detail = "accepted/proof-guided-depth=0";
    return baseline;
  }

  auto best = std::move(baseline);
  int best_rank = evidence_rank(best.stage);
  constexpr std::size_t kMaximumAuditDepth = 3U;
  for (std::size_t depth = 1U; depth <= kMaximumAuditDepth; ++depth) {
    auto evaluated = evaluate_arm(
      arm, candidate, source_fingerprint, candidate_fingerprint, successor,
      lattice_transition_stage, lattice_ahead_stage, nullptr, false,
      std::nullopt, depth);
    if (evaluated.bundle.has_value()) {
      evaluated.detail = std::string{"accepted/proof-guided-depth="} +
        std::to_string(depth) + "/baseline=" + to_string(baseline_stage);
      return evaluated;
    }
    const int rank = evidence_rank(evaluated.stage);
    if (rank > best_rank) {
      best_rank = rank;
      best = std::move(evaluated);
    }
  }
  best.detail = std::string{"no certified proof-guided iterate/baseline="} +
    to_string(baseline_stage) + "/best_depth=" +
    std::to_string(best.dynamic_sqp_depth) + '/' + best.detail;
  return best;
}

enum class ProductionEvaluationMode
{
  SingleSqp,
  FixedDynamicSqp,
  ProofGuidedDynamicSqp,
};

ArmResult evaluate_production_population(
  const Arm arm, const shadow::Snapshot & source,
  const std::uint64_t source_fingerprint, const int side,
  const ProductionEvaluationMode mode = ProductionEvaluationMode::SingleSqp,
  const TerminalStopLateralAuditMode terminal_stop_lateral_audit_mode =
  TerminalStopLateralAuditMode::RacingLine)
{
  const auto population = maneuver::build_bounded_candidates(source, side);
  if (
    population.reason != maneuver::RejectReason::Accepted ||
    population.candidates.empty())
  {
    const auto stage =
      population.reason == maneuver::RejectReason::TerminalSuccessorUnavailable ?
      Stage::TerminalSuccessorRejected : Stage::CandidateRejected;
    return rejected_arm(
      arm, stage, source_fingerprint,
      std::string{maneuver::to_string(population.reason)} + ": " +
      population.detail);
  }

  // Match the production worker: candidates from one immutable world are
  // evaluated in a bounded order and may share only their private SQP warm
  // start.  The first fully certified candidate wins; no command authority is
  // present in this audit path.
  shadow::SolverContext solver_context;
  ArmResult best = rejected_arm(
    arm, Stage::CandidateRejected, source_fingerprint,
    "bounded current-world population was not evaluated");
  int best_rank = -1;
  std::size_t attempted = 0U;
  for (const auto & candidate : population.candidates) {
    ++attempted;
    const auto successor = resolve_audit_terminal_successor(
      candidate.seed.solver_snapshot);
    const int transition_stage = candidate.seed.solver_snapshot.
      dynamic_obstacle_forced_diagonal_start_stage;
    const int ahead_stage = candidate.seed.solver_snapshot.
      dynamic_obstacle_forced_diagonal_full_side_stage;
    auto evaluated = mode == ProductionEvaluationMode::ProofGuidedDynamicSqp ?
      evaluate_proof_guided_candidate(
      arm, candidate.seed.solver_snapshot, source_fingerprint,
      candidate.seed.candidate_fingerprint, successor, transition_stage,
      ahead_stage) :
      evaluate_arm(
      arm, candidate.seed.solver_snapshot, source_fingerprint,
      candidate.seed.candidate_fingerprint, successor, transition_stage,
      ahead_stage, &solver_context, false, std::nullopt,
      mode == ProductionEvaluationMode::FixedDynamicSqp ? 3U : 0U,
      terminal_stop_lateral_audit_mode);
    evaluated.candidate_source = maneuver::to_string(candidate.kind);
    evaluated.candidate_count = attempted;
    const int rank = evidence_rank(evaluated.stage);
    if (rank > best_rank) {
      best_rank = rank;
      best = std::move(evaluated);
    }
    if (best.stage == Stage::Accepted) {
      best.detail += std::string{"/candidate="} + best.candidate_source;
      return best;
    }
  }
  best.candidate_count = attempted;
  best.detail = std::string{"no certified current-world candidate/best="} +
    best.candidate_source + "/" + best.detail;
  return best;
}

ArmResult evaluate_normal_production_population(
  const Arm arm, const shadow::Snapshot & source,
  const std::uint64_t source_fingerprint, const int side)
{
  const auto population = maneuver::build_normal_avoidance_candidates(source);
  if (
    population.reason != maneuver::RejectReason::Accepted ||
    population.candidates.empty())
  {
    const auto stage =
      population.reason == maneuver::RejectReason::TerminalSuccessorUnavailable ?
      Stage::TerminalSuccessorRejected : Stage::CandidateRejected;
    return rejected_arm(
      arm, stage, source_fingerprint,
      std::string{maneuver::to_string(population.reason)} + ": " +
      population.detail);
  }
  shadow::SolverContext solver_context;
  ArmResult best = rejected_arm(
    arm, Stage::CandidateRejected, source_fingerprint,
    "bounded normal production population was not evaluated");
  int best_rank = -1;
  std::size_t attempted = 0U;
  for (const auto & candidate : population.candidates) {
    if (candidate.seed.pass_side_sign != side) {
      continue;
    }
    ++attempted;
    const auto successor = resolve_audit_terminal_successor(
      candidate.seed.solver_snapshot);
    auto evaluated = evaluate_arm(
      arm, candidate.seed.solver_snapshot, source_fingerprint,
      candidate.seed.candidate_fingerprint, successor,
      candidate.seed.solver_snapshot.
      dynamic_obstacle_forced_first_pass_side_stage,
      candidate.seed.solver_snapshot.
      dynamic_obstacle_forced_first_ahead_stage,
      &solver_context);
    evaluated.candidate_source = maneuver::to_string(candidate.kind);
    evaluated.candidate_count = attempted;
    const int rank = evidence_rank(evaluated.stage);
    if (rank > best_rank) {
      best_rank = rank;
      best = std::move(evaluated);
    }
    if (best.stage == Stage::Accepted) {
      best.detail += std::string{"/candidate="} + best.candidate_source;
      return best;
    }
  }
  best.candidate_count = attempted;
  best.detail = std::string{"no certified bounded normal candidate/best="} +
    best.candidate_source + "/" + best.detail;
  return best;
}

ArmResult evaluate_stop_control_lattice(
  const shadow::Snapshot & maximum_braking_stop,
  const std::uint64_t source_fingerprint,
  const persistent_osqp::PhysicalConstraintTolerance & solver_tolerance)
{
  ArmResult best = rejected_arm(
    Arm::SevenStateStopControlLatticeV, Stage::CandidateRejected,
    source_fingerprint, "Stop control lattice was not evaluated");
  int best_rank = -1;
  std::size_t attempted = 0U;
  const auto population = stop_lattice::build_population(
    maximum_braking_stop, solver_tolerance);
  for (const auto & candidate : population) {
    ++attempted;
    const auto & schedule = candidate.schedule;
    const std::string source = schedule.initial_rate_sign > 0 ?
      "positive-negative-hold" : "negative-positive-hold";
    if (!candidate.accepted()) {
      auto rejected = rejected_arm(
        Arm::SevenStateStopControlLatticeV, Stage::CandidateRejected,
        source_fingerprint, candidate.detail);
      rejected.lattice_transition_stage = schedule.first_switch_stage;
      rejected.lattice_ahead_stage = schedule.second_switch_stage;
      rejected.candidate_source = source;
      rejected.candidate_count = attempted;
      if (best_rank < evidence_rank(rejected.stage)) {
        best_rank = evidence_rank(rejected.stage);
        best = std::move(rejected);
      }
      continue;
    }
    auto candidate_fingerprint =
      architecture::fingerprint_interaction_snapshot(maximum_braking_stop);
    append_fingerprint_u64(
      candidate_fingerprint,
      static_cast<std::uint64_t>(schedule.initial_rate_sign));
    append_fingerprint_u64(
      candidate_fingerprint,
      static_cast<std::uint64_t>(schedule.first_switch_stage));
    append_fingerprint_u64(
      candidate_fingerprint,
      static_cast<std::uint64_t>(schedule.second_switch_stage));
    for (const double steering_rate_radps : schedule.steering_rate_radps) {
      append_fingerprint_double(candidate_fingerprint, steering_rate_radps);
    }
    auto evaluated = evaluate_arm(
      Arm::SevenStateStopControlLatticeV, maximum_braking_stop,
      source_fingerprint, candidate_fingerprint,
      resolve_audit_terminal_successor(maximum_braking_stop),
      schedule.first_switch_stage, schedule.second_switch_stage,
      nullptr, false, std::nullopt, 0U,
      TerminalStopLateralAuditMode::SolvedStopTrajectory,
      &schedule.steering_rate_radps);
    evaluated.candidate_source = source;
    evaluated.candidate_count = attempted;
    const int rank = evidence_rank(evaluated.stage);
    if (rank > best_rank) {
      best_rank = rank;
      best = std::move(evaluated);
    }
    if (best.stage == Stage::Accepted) {
      best.detail += "/candidate=" + best.candidate_source;
      return best;
    }
  }
  best.candidate_count = attempted;
  best.detail = std::string{"no certified Stop control lattice/best="} +
    best.candidate_source + '/' + best.detail;
  return best;
}

}  // namespace

const char * to_string(const Arm arm) noexcept
{
  switch (arm) {
    case Arm::PersistentA: return "persistent-a";
    case Arm::PersistentTargetBoundA2:
      return "persistent-target-bound-a2";
    case Arm::StatelessLeftB: return "stateless-left-b";
    case Arm::StatelessRightB: return "stateless-right-b";
    case Arm::StatelessReturnB: return "stateless-return-b";
    case Arm::RoughLeftC: return "rough-left-c";
    case Arm::RoughRightC: return "rough-right-c";
    case Arm::RoughReturnC: return "rough-return-c";
    case Arm::OfflineLeftD: return "offline-left-d";
    case Arm::OfflineRightD: return "offline-right-d";
    case Arm::OfflineReturnD: return "offline-return-d";
    case Arm::DiagonalLeftE: return "diagonal-left-e";
    case Arm::DiagonalRightE: return "diagonal-right-e";
    case Arm::PhysicalDiagonalLeftF: return "physical-diagonal-left-f";
    case Arm::PhysicalDiagonalRightF: return "physical-diagonal-right-f";
    case Arm::ProductionLeftG: return "production-left-g";
    case Arm::ProductionRightG: return "production-right-g";
    case Arm::ProductionReturnG: return "production-return-g";
    case Arm::WallRestorationH: return "wall-restoration-h";
    case Arm::ExternalPrimalI: return "external-primal-i";
    case Arm::WallOmitHeadingJ: return "wall-omit-heading-j";
    case Arm::WallOmitLagK: return "wall-omit-lag-k";
    case Arm::WallOmitPoseN: return "wall-omit-pose-n";
    case Arm::WallOmitPoseDirectO: return "wall-omit-pose-direct-o";
    case Arm::WallProductionP: return "wall-production-p";
    case Arm::KktEquilibratedQ: return "kkt-equilibrated-q";
    case Arm::DynamicSqpPersistentL: return "dynamic-sqp-persistent-l";
    case Arm::DynamicSqpProductionLeftL:
      return "dynamic-sqp-production-left-l";
    case Arm::DynamicSqpProductionRightL:
      return "dynamic-sqp-production-right-l";
    case Arm::ProofGuidedProductionLeftM:
      return "proof-guided-production-left-m";
    case Arm::ProofGuidedProductionRightM:
      return "proof-guided-production-right-m";
    case Arm::PersistentDeclaredStopR:
      return "persistent-declared-stop-r";
    case Arm::ProductionLeftDeclaredStopR:
      return "production-left-declared-stop-r";
    case Arm::PersistentStopScanS:
      return "persistent-stop-scan-s";
    case Arm::ProductionLeftStopScanS:
      return "production-left-stop-scan-s";
    case Arm::PersistentNormalPathStopT:
      return "persistent-normal-path-stop-t";
    case Arm::ProductionLeftNormalPathStopT:
      return "production-left-normal-path-stop-t";
    case Arm::SevenStateStopU:
      return "seven-state-stop-u";
    case Arm::SevenStateStopControlLatticeV:
      return "seven-state-stop-control-lattice-v";
  }
  return "unknown";
}

const char * to_string(const Stage stage) noexcept
{
  switch (stage) {
    case Stage::Accepted: return "accepted";
    case Stage::SourceRejected: return "source-rejected";
    case Stage::CandidateRejected: return "candidate-rejected";
    case Stage::SolverRejected: return "solver-rejected";
    case Stage::ExactTrajectoryRejected: return "exact-trajectory-rejected";
    case Stage::WallProofRejected: return "wall-proof-rejected";
    case Stage::DynamicProofRejected: return "dynamic-proof-rejected";
    case Stage::TerminalSuccessorRejected:
      return "terminal-successor-rejected";
  }
  return "unknown";
}

Report compare(
  const architecture::RecordedInteractionSnapshot & recorded) noexcept
{
  Report report;
  try {
    const auto & source = recorded.source;
    const auto source_fingerprint = recorded.interaction_fingerprint;
    report.source_interaction_fingerprint = source_fingerprint;
    if (
      !architecture::interaction_snapshot_complete(source) ||
      !architecture::interaction_snapshot_matches_fingerprint(
        source, source_fingerprint))
    {
      report.detail = "source interaction snapshot rejected";
      for (const auto arm :
        {Arm::PersistentA, Arm::PersistentTargetBoundA2,
         Arm::StatelessLeftB, Arm::StatelessRightB,
         Arm::RoughLeftC, Arm::RoughRightC,
         Arm::OfflineLeftD, Arm::OfflineRightD,
         Arm::DiagonalLeftE, Arm::DiagonalRightE,
         Arm::PhysicalDiagonalLeftF, Arm::PhysicalDiagonalRightF,
         Arm::ProductionLeftG, Arm::ProductionRightG,
         Arm::WallRestorationH})
      {
        report.arms.push_back(rejected_arm(
          arm, Stage::SourceRejected, source_fingerprint, report.detail));
      }
      return report;
    }
    report.source_accepted = true;
    report.detail = "accepted";
    const auto persistent_successor = resolve_audit_terminal_successor(source);
    report.arms.push_back(evaluate_arm(
      Arm::PersistentA, source, source_fingerprint, source_fingerprint,
      persistent_successor));

    // Cruise/Follow own no tactical pass side in production. When their
    // automatically selected obstacle branch is infeasible, compare two
    // independently rebuilt current-world side candidates before attributing
    // the failure to physics. This is audit-only: it has no store, mailbox or
    // publisher.  C and D remain architecture-audit-only: they enumerate a
    // smooth current-world normal-avoidance lattice and never enter the
    // production candidate population.
    if (
      source.identity.source_context.intent ==
      contract::ControlIntent::Follow ||
      source.identity.source_context.intent ==
      contract::ControlIntent::Cruise)
    {
      for (const auto & [arm, side] :
        {std::pair{Arm::StatelessLeftB, 1},
         std::pair{Arm::StatelessRightB, -1}})
      {
        const auto rebuilt = maneuver::build_normal_avoidance(
          source, source_fingerprint, side);
        if (!rebuilt.seed.has_value()) {
          const auto stage =
            rebuilt.reason ==
            maneuver::RejectReason::TerminalSuccessorUnavailable ?
            Stage::TerminalSuccessorRejected : Stage::CandidateRejected;
          report.arms.push_back(rejected_arm(
            arm, stage, source_fingerprint,
            std::string{maneuver::to_string(rebuilt.reason)} + ": " +
            rebuilt.detail));
          continue;
        }
        const auto successor = resolve_audit_terminal_successor(
          rebuilt.seed->solver_snapshot);
        report.arms.push_back(evaluate_arm(
          arm, rebuilt.seed->solver_snapshot, source_fingerprint,
          rebuilt.seed->candidate_fingerprint, successor));
      }
      for (const auto & [arm, side] :
        {std::pair{Arm::RoughLeftC, 1},
         std::pair{Arm::RoughRightC, -1}})
      {
        report.arms.push_back(evaluate_normal_avoidance_lattice_population(
          arm, source, source_fingerprint, side, 0U));
      }
      constexpr std::size_t kOfflineNormalSqpDepth = 3U;
      for (const auto & [arm, side] :
        {std::pair{Arm::OfflineLeftD, 1},
         std::pair{Arm::OfflineRightD, -1}})
      {
        report.arms.push_back(evaluate_normal_avoidance_lattice_population(
          arm, source, source_fingerprint, side,
          kOfflineNormalSqpDepth));
      }
      for (const auto & [arm, side] :
        {std::pair{Arm::ProductionLeftG, 1},
         std::pair{Arm::ProductionRightG, -1}})
      {
        report.arms.push_back(evaluate_normal_production_population(
          arm, source, source_fingerprint, side));
      }
      return report;
    }

    const auto target_bound =
      maneuver::bind_current_world_target_preserving_geometry(
      source, source_fingerprint);
    if (!target_bound.seed.has_value()) {
      const auto stage =
        target_bound.reason ==
        maneuver::RejectReason::TerminalSuccessorUnavailable ?
        Stage::TerminalSuccessorRejected : Stage::CandidateRejected;
      report.arms.push_back(rejected_arm(
        Arm::PersistentTargetBoundA2, stage, source_fingerprint,
        std::string{maneuver::to_string(target_bound.reason)} + ": " +
        target_bound.detail));
    } else {
      const auto target_bound_successor = resolve_audit_terminal_successor(
        target_bound.seed->solver_snapshot);
      report.arms.push_back(evaluate_arm(
        Arm::PersistentTargetBoundA2,
        target_bound.seed->solver_snapshot, source_fingerprint,
        target_bound.seed->candidate_fingerprint, target_bound_successor));
    }

    // Return owns one committed homotopy, so pass-side disjunction schedules
    // are not a meaningful C/D comparison. Rebuild a bounded family of smooth
    // current-state-to-endpoint schedules instead. This is observation-only:
    // the production candidate population below remains the unchanged G arm.
    if (source.identity.source_context.intent == contract::ControlIntent::Return) {
      const int source_side =
        source.identity.source_context.execution_side_sign;
      const auto rebuilt = maneuver::build(
        source, source_fingerprint, source_side);
      if (!rebuilt.seed.has_value()) {
        const auto stage =
          rebuilt.reason == maneuver::RejectReason::TerminalSuccessorUnavailable ?
          Stage::TerminalSuccessorRejected : Stage::CandidateRejected;
        report.arms.push_back(rejected_arm(
          Arm::StatelessReturnB, stage, source_fingerprint,
          std::string{maneuver::to_string(rebuilt.reason)} + ": " +
          rebuilt.detail));
      } else {
        const auto successor = resolve_audit_terminal_successor(
          rebuilt.seed->solver_snapshot);
        report.arms.push_back(evaluate_arm(
          Arm::StatelessReturnB, rebuilt.seed->solver_snapshot,
          source_fingerprint, rebuilt.seed->candidate_fingerprint, successor));
      }

      const auto schedules = return_rejoin_schedule_lattice(
        source.request.horizon_steps);
      for (const auto & [start_stage, complete_stage] : schedules) {
        const auto candidate = maneuver::build_return_rejoin_schedule(
          source, source_fingerprint, start_stage, complete_stage);
        if (!candidate.seed.has_value()) {
          const auto stage =
            candidate.reason ==
            maneuver::RejectReason::TerminalSuccessorUnavailable ?
            Stage::TerminalSuccessorRejected : Stage::CandidateRejected;
          auto rejected = rejected_arm(
            Arm::RoughReturnC, stage, source_fingerprint,
            std::string{maneuver::to_string(candidate.reason)} + ": " +
            candidate.detail);
          rejected.lattice_transition_stage = start_stage;
          rejected.lattice_ahead_stage = complete_stage;
          report.arms.push_back(std::move(rejected));
        } else {
          const auto successor = resolve_audit_terminal_successor(
            candidate.seed->solver_snapshot);
          auto evaluated = evaluate_arm(
            Arm::RoughReturnC, candidate.seed->solver_snapshot,
            source_fingerprint, candidate.seed->candidate_fingerprint,
            successor, start_stage, complete_stage);
          evaluated.candidate_source = "return-rejoin-polynomial";
          report.arms.push_back(std::move(evaluated));
        }
        report.arms.push_back(evaluate_offline_return_rejoin(
          source, source_fingerprint, start_stage, complete_stage));
      }

      report.arms.push_back(evaluate_production_population(
        Arm::ProductionReturnG, source, source_fingerprint, source_side));
      report.arms.push_back(evaluate_arm(
        Arm::WallRestorationH, source, source_fingerprint,
        source_fingerprint, persistent_successor, -1, -1, nullptr, true));
      return report;
    }

    for (const auto & [arm, side] :
      {std::pair{Arm::StatelessLeftB, 1},
       std::pair{Arm::StatelessRightB, -1}})
    {
      const auto built = maneuver::build(source, source_fingerprint, side);
      if (!built.seed.has_value()) {
        const auto stage =
          built.reason == maneuver::RejectReason::TerminalSuccessorUnavailable ?
          Stage::TerminalSuccessorRejected : Stage::CandidateRejected;
        report.arms.push_back(rejected_arm(
          arm, stage, source_fingerprint,
          std::string{maneuver::to_string(built.reason)} + ": " + built.detail));
        continue;
      }
      const auto successor = resolve_audit_terminal_successor(
        built.seed->solver_snapshot);
      report.arms.push_back(evaluate_arm(
        arm, built.seed->solver_snapshot, source_fingerprint,
        built.seed->candidate_fingerprint, successor));
    }

    for (const auto & [arm, side] :
      {std::pair{Arm::RoughLeftC, 1},
       std::pair{Arm::RoughRightC, -1}})
    {
      for (int transition_stage = 0;
        transition_stage < source.request.horizon_steps; ++transition_stage)
      {
        for (int ahead_stage = transition_stage + 1;
          ahead_stage <= source.request.horizon_steps; ++ahead_stage)
        {
          const auto built = maneuver::build_lattice(
            source, source_fingerprint, side, transition_stage, ahead_stage);
          if (!built.seed.has_value()) {
            const auto stage =
              built.reason == maneuver::RejectReason::TerminalSuccessorUnavailable ?
              Stage::TerminalSuccessorRejected : Stage::CandidateRejected;
            auto rejected = rejected_arm(
              arm, stage, source_fingerprint,
              std::string{maneuver::to_string(built.reason)} + ": " + built.detail);
            rejected.lattice_transition_stage = transition_stage;
            rejected.lattice_ahead_stage = ahead_stage;
            report.arms.push_back(std::move(rejected));
            continue;
          }
          const auto successor = resolve_audit_terminal_successor(
            built.seed->solver_snapshot);
          report.arms.push_back(evaluate_arm(
            arm, built.seed->solver_snapshot, source_fingerprint,
            built.seed->candidate_fingerprint, successor, transition_stage,
            ahead_stage));
        }
      }
    }

    for (const auto & [arm, side] :
      {std::pair{Arm::OfflineLeftD, 1},
       std::pair{Arm::OfflineRightD, -1}})
    {
      for (int transition_stage = 0;
        transition_stage < source.request.horizon_steps; ++transition_stage)
      {
        for (int ahead_stage = transition_stage + 1;
          ahead_stage <= source.request.horizon_steps; ++ahead_stage)
        {
          report.arms.push_back(evaluate_offline_continuation(
            arm, source, source_fingerprint, side, transition_stage,
            ahead_stage));
        }
      }
    }

    for (const auto & [arm, side] :
      {std::pair{Arm::DiagonalLeftE, 1},
       std::pair{Arm::DiagonalRightE, -1}})
    {
      for (int diagonal_start_stage = 0;
        diagonal_start_stage + 2 < source.request.horizon_steps;
        ++diagonal_start_stage)
      {
        for (int full_side_stage = diagonal_start_stage + 2;
          full_side_stage < source.request.horizon_steps; ++full_side_stage)
        {
          const auto built = maneuver::build_diagonal_schedule(
            source, source_fingerprint, side, diagonal_start_stage,
            full_side_stage);
          if (!built.seed.has_value()) {
            const auto stage =
              built.reason == maneuver::RejectReason::TerminalSuccessorUnavailable ?
              Stage::TerminalSuccessorRejected : Stage::CandidateRejected;
            auto rejected = rejected_arm(
              arm, stage, source_fingerprint,
              std::string{maneuver::to_string(built.reason)} + ": " +
              built.detail);
            rejected.lattice_transition_stage = diagonal_start_stage;
            rejected.lattice_ahead_stage = full_side_stage;
            report.arms.push_back(std::move(rejected));
            continue;
          }
          const auto successor = resolve_audit_terminal_successor(
            built.seed->solver_snapshot);
          report.arms.push_back(evaluate_arm(
            arm, built.seed->solver_snapshot, source_fingerprint,
            built.seed->candidate_fingerprint, successor,
            diagonal_start_stage, full_side_stage));
        }
      }
    }

    for (const auto & [arm, side] :
      {std::pair{Arm::PhysicalDiagonalLeftF, 1},
       std::pair{Arm::PhysicalDiagonalRightF, -1}})
    {
      for (int diagonal_start_stage = 0;
        diagonal_start_stage + 2 < source.request.horizon_steps;
        ++diagonal_start_stage)
      {
        for (int full_side_stage = diagonal_start_stage + 2;
          full_side_stage < source.request.horizon_steps; ++full_side_stage)
        {
          const auto built = maneuver::build_physical_diagonal_schedule(
            source, source_fingerprint, side, diagonal_start_stage,
            full_side_stage);
          if (!built.seed.has_value()) {
            const auto stage =
              built.reason == maneuver::RejectReason::TerminalSuccessorUnavailable ?
              Stage::TerminalSuccessorRejected : Stage::CandidateRejected;
            auto rejected = rejected_arm(
              arm, stage, source_fingerprint,
              std::string{maneuver::to_string(built.reason)} + ": " +
              built.detail);
            rejected.lattice_transition_stage = diagonal_start_stage;
            rejected.lattice_ahead_stage = full_side_stage;
            report.arms.push_back(std::move(rejected));
            continue;
          }
          const auto successor = resolve_audit_terminal_successor(
            built.seed->solver_snapshot);
          report.arms.push_back(evaluate_arm(
            arm, built.seed->solver_snapshot, source_fingerprint,
            built.seed->candidate_fingerprint, successor,
            diagonal_start_stage, full_side_stage));
        }
      }
    }

    for (const auto & [arm, side] :
      {std::pair{Arm::ProductionLeftG, 1},
       std::pair{Arm::ProductionRightG, -1}})
    {
      report.arms.push_back(evaluate_production_population(
        arm, source, source_fingerprint, side));
    }

    // This final arm shares the exact immutable source and all downstream
    // proof gates with A. Its only difference is an explicitly audit-only
    // numerical restoration seed after an affine-infeasible wall refinement.
    // It owns no publisher/store and cannot alter production authority.
    report.arms.push_back(evaluate_arm(
      Arm::WallRestorationH, source, source_fingerprint, source_fingerprint,
      persistent_successor, -1, -1, nullptr, true));
  } catch (const std::exception & exception) {
    report.source_accepted = false;
    report.detail = exception.what();
  } catch (...) {
    report.source_accepted = false;
    report.detail = "unknown architecture comparison exception";
  }
  return report;
}

Report compare_wall_restoration(
  const architecture::RecordedInteractionSnapshot & recorded) noexcept
{
  Report report;
  try {
    const auto & source = recorded.source;
    const auto source_fingerprint = recorded.interaction_fingerprint;
    report.source_interaction_fingerprint = source_fingerprint;
    if (
      !architecture::interaction_snapshot_complete(source) ||
      !architecture::interaction_snapshot_matches_fingerprint(
        source, source_fingerprint))
    {
      report.detail = "source interaction snapshot rejected";
      report.arms.push_back(rejected_arm(
        Arm::WallRestorationH, Stage::SourceRejected, source_fingerprint,
        report.detail));
      return report;
    }
    report.source_accepted = true;
    report.detail = "accepted/wall-restoration-only";
    const auto successor = resolve_audit_terminal_successor(source);
    report.arms.push_back(evaluate_arm(
      Arm::WallRestorationH, source, source_fingerprint, source_fingerprint,
      successor, -1, -1, nullptr, true));
  } catch (const std::exception & exception) {
    report.source_accepted = false;
    report.detail = exception.what();
  } catch (...) {
    report.source_accepted = false;
    report.detail = "unknown wall restoration comparison exception";
  }
  return report;
}

Report compare_physical_dynamic_sqp(
  const architecture::RecordedInteractionSnapshot & recorded) noexcept
{
  Report report;
  try {
    const auto & source = recorded.source;
    const auto source_fingerprint = recorded.interaction_fingerprint;
    report.source_interaction_fingerprint = source_fingerprint;
    if (
      !architecture::interaction_snapshot_complete(source) ||
      !architecture::interaction_snapshot_matches_fingerprint(
        source, source_fingerprint))
    {
      report.detail = "source interaction snapshot rejected";
      for (const auto arm :
        {Arm::PersistentA, Arm::DynamicSqpPersistentL,
         Arm::ProductionLeftG, Arm::DynamicSqpProductionLeftL,
         Arm::ProductionRightG, Arm::DynamicSqpProductionRightL})
      {
        report.arms.push_back(rejected_arm(
          arm, Stage::SourceRejected, source_fingerprint, report.detail));
      }
      return report;
    }
    report.source_accepted = true;
    report.detail = "accepted/physical-dynamic-sqp-only";
    const auto persistent_successor =
      resolve_audit_terminal_successor(source);
    report.arms.push_back(evaluate_arm(
      Arm::PersistentA, source, source_fingerprint, source_fingerprint,
      persistent_successor));
    report.arms.push_back(evaluate_arm(
      Arm::DynamicSqpPersistentL, source, source_fingerprint,
      source_fingerprint, persistent_successor, -1, -1, nullptr, false,
      std::nullopt, 3U));

    report.arms.push_back(evaluate_production_population(
      Arm::ProductionLeftG, source, source_fingerprint, 1));
    report.arms.push_back(evaluate_production_population(
      Arm::DynamicSqpProductionLeftL, source, source_fingerprint, 1,
      ProductionEvaluationMode::FixedDynamicSqp));
    report.arms.push_back(evaluate_production_population(
      Arm::ProductionRightG, source, source_fingerprint, -1));
    report.arms.push_back(evaluate_production_population(
      Arm::DynamicSqpProductionRightL, source, source_fingerprint, -1,
      ProductionEvaluationMode::FixedDynamicSqp));
  } catch (const std::exception & exception) {
    report.source_accepted = false;
    report.detail = exception.what();
  } catch (...) {
    report.source_accepted = false;
    report.detail = "unknown physical dynamic SQP comparison exception";
  }
  return report;
}

Report compare_proof_guided_dynamic_sqp(
  const architecture::RecordedInteractionSnapshot & recorded) noexcept
{
  Report report;
  try {
    const auto & source = recorded.source;
    const auto source_fingerprint = recorded.interaction_fingerprint;
    report.source_interaction_fingerprint = source_fingerprint;
    if (
      !architecture::interaction_snapshot_complete(source) ||
      !architecture::interaction_snapshot_matches_fingerprint(
        source, source_fingerprint))
    {
      report.detail = "source interaction snapshot rejected";
      for (const auto arm :
        {Arm::ProductionLeftG, Arm::ProofGuidedProductionLeftM,
         Arm::ProductionRightG, Arm::ProofGuidedProductionRightM})
      {
        report.arms.push_back(rejected_arm(
          arm, Stage::SourceRejected, source_fingerprint, report.detail));
      }
      return report;
    }
    report.source_accepted = true;
    report.detail = "accepted/proof-guided-dynamic-sqp-only";
    report.arms.push_back(evaluate_production_population(
      Arm::ProductionLeftG, source, source_fingerprint, 1));
    report.arms.push_back(evaluate_production_population(
      Arm::ProofGuidedProductionLeftM, source, source_fingerprint, 1,
      ProductionEvaluationMode::ProofGuidedDynamicSqp));
    report.arms.push_back(evaluate_production_population(
      Arm::ProductionRightG, source, source_fingerprint, -1));
    report.arms.push_back(evaluate_production_population(
      Arm::ProofGuidedProductionRightM, source, source_fingerprint, -1,
      ProductionEvaluationMode::ProofGuidedDynamicSqp));
  } catch (const std::exception & exception) {
    report.source_accepted = false;
    report.detail = exception.what();
  } catch (...) {
    report.source_accepted = false;
    report.detail = "unknown proof-guided dynamic SQP comparison exception";
  }
  return report;
}

Report compare_wall_buckets(
  const architecture::RecordedInteractionSnapshot & recorded) noexcept
{
  Report report;
  try {
    const auto & source = recorded.source;
    const auto source_fingerprint = recorded.interaction_fingerprint;
    report.source_interaction_fingerprint = source_fingerprint;
    if (
      !architecture::interaction_snapshot_complete(source) ||
      !architecture::interaction_snapshot_matches_fingerprint(
        source, source_fingerprint))
    {
      report.detail = "source interaction snapshot rejected";
      for (const auto arm :
        {Arm::WallOmitHeadingJ, Arm::WallOmitLagK, Arm::WallOmitPoseN,
          Arm::WallOmitPoseDirectO, Arm::WallProductionP})
      {
        report.arms.push_back(rejected_arm(
          arm, Stage::SourceRejected, source_fingerprint, report.detail));
      }
      return report;
    }
    report.source_accepted = true;
    report.detail = "accepted/wall-bucket-audit-only";
    const auto successor = resolve_audit_terminal_successor(source);
    report.arms.push_back(evaluate_arm(
      Arm::WallOmitHeadingJ, source, source_fingerprint,
      source_fingerprint, successor, -1, -1, nullptr, false,
      shadow::SolverContext::WallBucketAuditMode::OmitHeading));
    report.arms.push_back(evaluate_arm(
      Arm::WallOmitLagK, source, source_fingerprint,
      source_fingerprint, successor, -1, -1, nullptr, false,
      shadow::SolverContext::WallBucketAuditMode::OmitLag));
    report.arms.push_back(evaluate_arm(
      Arm::WallOmitPoseN, source, source_fingerprint,
      source_fingerprint, successor, -1, -1, nullptr, false,
      shadow::SolverContext::WallBucketAuditMode::OmitPose));
    report.arms.push_back(evaluate_arm(
      Arm::WallOmitPoseDirectO, source, source_fingerprint,
      source_fingerprint, successor, -1, -1, nullptr, false,
      shadow::SolverContext::WallBucketAuditMode::OmitPoseDirect));
    report.arms.push_back(evaluate_arm(
      Arm::WallProductionP, source, source_fingerprint,
      source_fingerprint, successor));
  } catch (const std::exception & exception) {
    report.source_accepted = false;
    report.detail = exception.what();
  } catch (...) {
    report.source_accepted = false;
    report.detail = "unknown wall bucket comparison exception";
  }
  return report;
}

Report verify_external_primal(
  const architecture::RecordedInteractionSnapshot & recorded,
  const Eigen::VectorXd & primal,
  const ExternalPrimalConstraintPolicy policy) noexcept
{
  Report report;
  const auto source_fingerprint = recorded.interaction_fingerprint;
  report.source_interaction_fingerprint = source_fingerprint;
  const auto reject_source = [&report, source_fingerprint](
      const Stage stage, const std::string & detail) {
      report.detail = detail;
      report.arms.push_back(rejected_arm(
        Arm::ExternalPrimalI, stage, source_fingerprint, detail));
      return report;
    };
  try {
    const auto & source = recorded.source;
    if (
      !architecture::interaction_snapshot_complete(source) ||
      !architecture::interaction_snapshot_matches_fingerprint(
        source, source_fingerprint))
    {
      return reject_source(
        Stage::SourceRejected, "source interaction snapshot rejected");
    }
    report.source_accepted = true;
    report.detail = "accepted/external-primal-proof-only";
    if (!recorded.recorded_qp.has_value()) {
      return reject_source(
        Stage::SourceRejected,
        "external primal requires an exact recorded QP; source-only "
        "proof-boundary evidence supports architecture arms only");
    }
    const auto verification_problem = external_primal_problem(recorded, policy);
    if (!verification_problem.has_value()) {
      return reject_source(
        Stage::SourceRejected, "external primal policy problem rejected");
    }
    const auto candidate_fingerprint = exact_problem_fingerprint(
      verification_problem.value(), source_fingerprint);
    const auto successor = resolve_audit_terminal_successor(source);
    if (!successor.accepted) {
      auto rejected = rejected_arm(
        Arm::ExternalPrimalI, Stage::TerminalSuccessorRejected,
        source_fingerprint, successor.detail);
      rejected.candidate_fingerprint = candidate_fingerprint;
      report.arms.push_back(std::move(rejected));
      return report;
    }
    if (!source.replay_world.has_value()) {
      auto rejected = rejected_arm(
        Arm::ExternalPrimalI, Stage::SourceRejected, source_fingerprint,
        "replay world unavailable");
      rejected.candidate_fingerprint = candidate_fingerprint;
      report.arms.push_back(std::move(rejected));
      return report;
    }

    ArmResult arm_result;
    arm_result.arm = Arm::ExternalPrimalI;
    arm_result.source_interaction_fingerprint = source_fingerprint;
    arm_result.candidate_fingerprint = candidate_fingerprint;
    const auto built = build_external_artifact(
      source, verification_problem.value(), primal, policy);
    if (!built.value.has_value()) {
      arm_result.stage = Stage::SolverRejected;
      arm_result.detail = built.detail;
      report.arms.push_back(std::move(arm_result));
      return report;
    }
    arm_result.solver_outcome = shadow::Outcome::Solved;
    arm_result.maximum_external_constraint_violation =
      built.maximum_absolute_violation;
    arm_result.maximum_external_normalized_constraint_violation =
      built.maximum_normalized_violation;
    arm_result.maximum_external_normalized_constraint_row =
      built.maximum_normalized_row;
    arm_result.terminal_progress_m =
      built.value->predicted_states.back().progress_m;
    arm_result.terminal_velocity_mps =
      built.value->predicted_states.back().velocity_mps;

    const auto adapted = physical::build(
      built.value.value(), source.identity.source_context.intent,
      source.identity.source_context.stage_geometry_id);
    if (!adapted.exact_trajectory.has_value()) {
      arm_result.stage = Stage::ExactTrajectoryRejected;
      std::ostringstream detail;
      detail << "adapter=" << physical::to_string(adapted.reason)
             << "/exact=" <<
        race_mpcc_foundation::exact_physical_execution_trajectory_reason_name(
        adapted.exact_reason)
             << "/stage=" << adapted.rejected_stage
             << "/lateral=" << adapted.rejected_lateral_m
             << "/bounds=[" << adapted.rejected_lateral_lower_m
             << ',' << adapted.rejected_lateral_upper_m << ']'
             << "/progress_defect=" << adapted.progress_dynamics_defect_m;
      arm_result.detail = detail.str();
      report.arms.push_back(std::move(arm_result));
      return report;
    }
    auto exact = adapted.exact_trajectory.value();
    arm_result.minimum_lateral_bound_reserve_m =
      exact.minimum_lateral_bound_reserve_m;
    auto physical_snapshot = wall_snapshot(
      source, source.replay_world.value(), exact);
    auto wall_result = wall::evaluate(physical_snapshot);
    if (wall_result.outcome != wall::Outcome::Accepted) {
      arm_result.stage = Stage::WallProofRejected;
      arm_result.detail = wall_result.detail;
      report.arms.push_back(std::move(arm_result));
      return report;
    }
    auto dynamic_result = dynamic::evaluate_current_world(
      source, physical_snapshot);
    if (!dynamic_result.valid || !dynamic_result.clear) {
      arm_result.stage = Stage::DynamicProofRejected;
      std::ostringstream detail;
      detail << "obstacle=" << dynamic_result.blocking_obstacle_id
             << "/reason="
             << recovery::to_string(dynamic_result.rejection_reason)
             << "/rejected_t=" << dynamic_result.rejected_elapsed_sec
             << "/rejected_clearance="
             << dynamic_result.rejected_clearance_m
             << "/minimum_clearance=" << dynamic_result.minimum_clearance_m;
      arm_result.detail = detail.str();
      report.arms.push_back(std::move(arm_result));
      return report;
    }

    const auto terminal_stop = certify_terminal_stop(
      source, built.value.value(), physical_snapshot);
    if (!terminal_stop.accepted) {
      arm_result.stage = Stage::TerminalSuccessorRejected;
      arm_result.detail = terminal_stop.detail;
      report.arms.push_back(std::move(arm_result));
      return report;
    }

    ManeuverBundle bundle;
    bundle.target_id = source.identity.source_context.target_id;
    bundle.pass_side_sign =
      source.identity.source_context.execution_side_sign != 0 ?
      source.identity.source_context.execution_side_sign :
      source.dynamic_obstacle_pass_side_sign;
    bundle.source_interaction_fingerprint = source_fingerprint;
    bundle.candidate_fingerprint = candidate_fingerprint;
    bundle.exact_trajectory = std::move(exact);
    bundle.wall_certificate = std::move(wall_result);
    bundle.dynamic_certificate = std::move(dynamic_result);
    bundle.terminal_stop_trajectory = terminal_stop.trajectory;
    bundle.terminal_stop_wall_certificate = terminal_stop.wall_certificate;
    bundle.terminal_stop_dynamic_certificate = terminal_stop.dynamic_certificate;
    bundle.terminal_successor = successor.successor;
    bundle.stop_suffix = successor.stop_suffix;
    arm_result.stage = Stage::Accepted;
    switch (policy) {
      case ExternalPrimalConstraintPolicy::ExactRecorded:
        arm_result.detail = "accepted/external-primal-exact-proofs";
        break;
      case ExternalPrimalConstraintPolicy::OmitWallHeadingBucket:
        arm_result.detail =
          "accepted/external-primal-omit-wall-heading-bucket/exact-proofs";
        break;
      case ExternalPrimalConstraintPolicy::OmitWallLagBucket:
        arm_result.detail =
          "accepted/external-primal-omit-wall-lag-bucket/exact-proofs";
        break;
      case ExternalPrimalConstraintPolicy::PhysicalNonlinearOracle:
        arm_result.detail =
          "accepted/external-primal-physical-nonlinear-oracle/exact-proofs";
        break;
    }
    arm_result.bundle = std::move(bundle);
    report.arms.push_back(std::move(arm_result));
  } catch (const std::exception & exception) {
    report.source_accepted = false;
    report.detail = exception.what();
  } catch (...) {
    report.source_accepted = false;
    report.detail = "unknown external primal verification exception";
  }
  return report;
}

Report compare_terminal_stop_lateral_contract(
  const architecture::RecordedInteractionSnapshot & recorded) noexcept
{
  Report report;
  try {
    const auto & source = recorded.source;
    const auto source_fingerprint = recorded.interaction_fingerprint;
    report.source_interaction_fingerprint = source_fingerprint;
    if (
      !architecture::interaction_snapshot_complete(source) ||
      !architecture::interaction_snapshot_matches_fingerprint(
        source, source_fingerprint))
    {
      report.detail = "source interaction snapshot rejected";
      for (const auto arm :
        {Arm::PersistentA, Arm::PersistentDeclaredStopR,
         Arm::PersistentStopScanS, Arm::ProductionLeftG,
         Arm::ProductionLeftDeclaredStopR, Arm::ProductionLeftStopScanS,
         Arm::PersistentNormalPathStopT,
         Arm::ProductionLeftNormalPathStopT,
         Arm::SevenStateStopU, Arm::SevenStateStopControlLatticeV})
      {
        report.arms.push_back(rejected_arm(
          arm, Stage::SourceRejected, source_fingerprint, report.detail));
      }
      return report;
    }
    report.source_accepted = true;
    report.detail = "accepted/terminal-stop-lateral-contract-audit-only";
    const auto persistent_successor =
      resolve_audit_terminal_successor(source);
    report.arms.push_back(evaluate_arm(
      Arm::PersistentA, source, source_fingerprint, source_fingerprint,
      persistent_successor));
    report.arms.push_back(evaluate_arm(
      Arm::PersistentDeclaredStopR, source, source_fingerprint,
      source_fingerprint, persistent_successor, -1, -1, nullptr, false,
      std::nullopt, 0U, TerminalStopLateralAuditMode::Declared));
    report.arms.push_back(evaluate_arm(
      Arm::PersistentStopScanS, source, source_fingerprint,
      source_fingerprint, persistent_successor, -1, -1, nullptr, false,
      std::nullopt, 0U, TerminalStopLateralAuditMode::PhysicalRangeScan));
    report.arms.push_back(evaluate_arm(
      Arm::PersistentNormalPathStopT, source, source_fingerprint,
      source_fingerprint, persistent_successor, -1, -1, nullptr, false,
      std::nullopt, 0U, TerminalStopLateralAuditMode::NormalPathProfile));
    report.arms.push_back(evaluate_production_population(
      Arm::ProductionLeftG, source, source_fingerprint, 1));
    report.arms.push_back(evaluate_production_population(
      Arm::ProductionLeftDeclaredStopR, source, source_fingerprint, 1,
      ProductionEvaluationMode::SingleSqp,
      TerminalStopLateralAuditMode::Declared));
    report.arms.push_back(evaluate_production_population(
      Arm::ProductionLeftStopScanS, source, source_fingerprint, 1,
      ProductionEvaluationMode::SingleSqp,
      TerminalStopLateralAuditMode::PhysicalRangeScan));
    report.arms.push_back(evaluate_production_population(
      Arm::ProductionLeftNormalPathStopT, source, source_fingerprint, 1,
      ProductionEvaluationMode::SingleSqp,
      TerminalStopLateralAuditMode::NormalPathProfile));
    // The live independent Stop worker owns a current-world producer that
    // does not depend on a normal execution artifact.  The observation-only
    // comparison must exercise that same hypothesis: requiring a successful
    // normal solve here made normal-solver failures indistinguishable from
    // physical Stop infeasibility.
    shadow::SolverContext stop_source_solver;
    const auto solved_stop =
      stop_lattice::build_current_world_maximum_braking_candidate(
      source, stop_source_solver.physical_constraint_tolerance());
    if (solved_stop.accepted()) {
      report.arms.push_back(evaluate_arm(
        Arm::SevenStateStopU, solved_stop.candidate, source_fingerprint,
        architecture::fingerprint_interaction_snapshot(solved_stop.candidate),
        resolve_audit_terminal_successor(solved_stop.candidate), -1, -1,
        nullptr, false, std::nullopt, 0U,
        TerminalStopLateralAuditMode::SolvedStopTrajectory));
      report.arms.push_back(evaluate_stop_control_lattice(
        solved_stop.candidate, source_fingerprint,
        stop_source_solver.physical_constraint_tolerance()));
    } else {
      report.arms.push_back(rejected_arm(
        Arm::SevenStateStopU, Stage::CandidateRejected, source_fingerprint,
        "current-world seven-state Stop audit candidate unavailable/" +
        solved_stop.detail));
      report.arms.push_back(rejected_arm(
        Arm::SevenStateStopControlLatticeV, Stage::CandidateRejected,
        source_fingerprint,
        "current-world seven-state Stop audit candidate unavailable/" +
        solved_stop.detail));
    }
  } catch (const std::exception & exception) {
    report.source_accepted = false;
    report.detail = exception.what();
  } catch (...) {
    report.source_accepted = false;
    report.detail = "unknown terminal Stop lateral comparison exception";
  }
  return report;
}

Report compare_kkt_equilibration(
  const architecture::RecordedInteractionSnapshot & recorded) noexcept
{
  const auto source_fingerprint = recorded.interaction_fingerprint;
  const auto reject = [source_fingerprint](const std::string & detail) {
      Report report;
      report.source_accepted = true;
      report.source_interaction_fingerprint = source_fingerprint;
      report.detail = "rejected/kkt-equilibration-audit-only";
      report.arms.push_back(rejected_arm(
        Arm::KktEquilibratedQ, Stage::SolverRejected,
        source_fingerprint, detail));
      return report;
    };
  try {
    if (
      !architecture::interaction_snapshot_complete(recorded.source) ||
      !architecture::interaction_snapshot_matches_fingerprint(
        recorded.source, source_fingerprint))
    {
      auto report = reject("source interaction snapshot rejected");
      report.source_accepted = false;
      report.detail = "source interaction snapshot rejected";
      report.arms.front().stage = Stage::SourceRejected;
      return report;
    }
    osqp::PersistentOsqpSolver solver{
      osqp::ConstraintPreconditioningPolicy::
        RowToleranceNormalizedWithInternalEquilibration};
    if (!recorded.recorded_qp.has_value()) {
      return reject("source-only snapshot has no exact QP");
    }
    const auto & qp = recorded.recorded_qp->problem;
    const auto outcome = solver.solve(
      qp.quadratic_cost, qp.constraints, qp.linear_cost,
      qp.lower_bound, qp.upper_bound, recorded.recorded_qp->warm_start,
      qp.variable_scaling);
    if (!outcome.result.has_value()) {
      return reject(outcome.failure_detail);
    }
    auto report = verify_external_primal(
      recorded, outcome.result->primal,
      ExternalPrimalConstraintPolicy::ExactRecorded);
    report.detail = report.arms.size() == 1U &&
      report.arms.front().bundle.has_value() ?
      "accepted/kkt-equilibration-audit-only" :
      "rejected/kkt-equilibration-exact-proof";
    if (report.arms.size() == 1U) {
      report.arms.front().arm = Arm::KktEquilibratedQ;
      report.arms.front().solver_compute_ms = outcome.telemetry.total_ms;
      report.arms.front().detail =
        "kkt-equilibrated/" + report.arms.front().detail;
    }
    return report;
  } catch (const std::exception & exception) {
    return reject(exception.what());
  } catch (...) {
    return reject("unknown kkt equilibration comparison exception");
  }
}

}  // namespace multi_purpose_mpc_ros::mpcc_architecture_comparison
