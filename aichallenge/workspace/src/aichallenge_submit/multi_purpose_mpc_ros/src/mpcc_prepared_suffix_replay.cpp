#include "multi_purpose_mpc_ros/mpcc_architecture_snapshot.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_problem.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_shadow.hpp"
#include "multi_purpose_mpc_ros/persistent_osqp.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <optional>
#include <string>

namespace architecture =
  multi_purpose_mpc_ros::mpcc_architecture_snapshot;
namespace artifact =
  multi_purpose_mpc_ros::mpcc_rate_resolved_execution_artifact;
namespace model = multi_purpose_mpc_ros::mpcc_rate_resolved;
namespace problem = multi_purpose_mpc_ros::mpcc_rate_resolved_problem;
namespace shadow = multi_purpose_mpc_ros::mpcc_rate_resolved_shadow;
namespace solver = multi_purpose_mpc_ros::persistent_osqp;

namespace
{

using Clock = std::chrono::steady_clock;

std::optional<artifact::PredictedState> interpolate_recorded_state(
  const shadow::Snapshot & source, const Eigen::VectorXd & primal,
  const double elapsed_sec)
{
  const int horizon = source.request.horizon_steps;
  const int state_values = model::kStateDimension * (horizon + 1);
  const int variable_count =
    state_values + model::kInputDimension * horizon;
  if (
    horizon <= 0 || primal.size() != variable_count || !primal.allFinite() ||
    !std::isfinite(elapsed_sec) || elapsed_sec < 0.0)
  {
    return std::nullopt;
  }
  double remaining = elapsed_sec;
  int stage = 0;
  while (stage < horizon) {
    const double dt =
      source.request.inputs[static_cast<std::size_t>(stage)].stage_dt_sec;
    if (!std::isfinite(dt) || dt <= 0.0) {
      return std::nullopt;
    }
    if (remaining < dt - 1.0e-9) {
      const double ratio = std::clamp(remaining / dt, 0.0, 1.0);
      const int first = stage * model::kStateDimension;
      const int second = (stage + 1) * model::kStateDimension;
      Eigen::Matrix<double, model::kStateDimension, 1> state =
        (1.0 - ratio) * primal.segment<model::kStateDimension>(first) +
        ratio * primal.segment<model::kStateDimension>(second);
      return artifact::PredictedState{
        state[model::kLateralIndex], state[model::kLagIndex],
        state[model::kHeadingIndex],
        std::max(0.0, state[model::kVelocityIndex]),
        state[model::kProgressIndex], state[model::kSteeringIndex],
        state[model::kResponseSteeringIndex]};
    }
    remaining = std::max(0.0, remaining - dt);
    ++stage;
  }
  return std::nullopt;
}

std::optional<Eigen::Matrix<double, model::kInputDimension, 1>>
active_recorded_input(
  const shadow::Snapshot & source, const Eigen::VectorXd & primal,
  const double elapsed_sec)
{
  const int horizon = source.request.horizon_steps;
  const int state_values = model::kStateDimension * (horizon + 1);
  double remaining = elapsed_sec;
  for (int stage = 0; stage < horizon; ++stage) {
    const double dt =
      source.request.inputs[static_cast<std::size_t>(stage)].stage_dt_sec;
    if (!std::isfinite(dt) || dt <= 0.0) return std::nullopt;
    if (remaining < dt - 1.0e-9) {
      return primal.segment<model::kInputDimension>(
        state_values + stage * model::kInputDimension);
    }
    remaining = std::max(0.0, remaining - dt);
  }
  return std::nullopt;
}

const char * suffix_comparison_classification(
  const shadow::LatestStateFeedbackResult & old_origin,
  const shadow::LatestStateFeedbackResult & time_aligned,
  const shadow::LatestStateFeedbackResult & reachable,
  const shadow::LatestStateFeedbackResult & multi_sqp,
  const shadow::LatestStateFeedbackResult & nonlinear_interior) noexcept
{
  const auto accepted = [](const shadow::LatestStateFeedbackResult & value) {
      return value.reason == shadow::LatestStateFeedbackReason::Accepted;
    };
  const auto model_proof_rejected = [](
      const shadow::LatestStateFeedbackResult & value) {
      return value.reason ==
             shadow::LatestStateFeedbackReason::PhysicalAdapterRejected &&
             value.physical_adapter_reason ==
             multi_purpose_mpc_ros::mpcc_rate_resolved_physical_adapter::
             RejectReason::ExactTrajectoryRejected;
    };
  if (!accepted(old_origin) && accepted(time_aligned)) {
    return "old-origin-clock-defect";
  }
  if (!accepted(old_origin) && !accepted(time_aligned) && accepted(reachable)) {
    return "reachable-candidate-generation-defect";
  }
  if (
    !accepted(old_origin) && !accepted(time_aligned) && !accepted(reachable) &&
    accepted(multi_sqp))
  {
    return "single-sqp-limitation";
  }
  if (!accepted(multi_sqp) && accepted(nonlinear_interior)) {
    return "nonlinear-interior-wall-representation-defect";
  }
  if (accepted(old_origin) && !accepted(time_aligned)) {
    return "time-aligned-suffix-regression";
  }
  if (accepted(time_aligned) && !accepted(reachable)) {
    return "reachable-candidate-regression";
  }
  if (accepted(reachable) && !accepted(multi_sqp)) {
    return "multi-sqp-regression";
  }
  if (accepted(multi_sqp) && !accepted(nonlinear_interior)) {
    return "nonlinear-interior-wall-regression";
  }
  if (
    model_proof_rejected(old_origin) || model_proof_rejected(time_aligned) ||
    model_proof_rejected(reachable) || model_proof_rejected(multi_sqp) ||
    model_proof_rejected(nonlinear_interior))
  {
    return "solve-proof-model-mismatch";
  }
  if (
    !accepted(old_origin) && !accepted(time_aligned) && !accepted(reachable) &&
    !accepted(multi_sqp) && !accepted(nonlinear_interior))
  {
    return "suffix-family-unresolved";
  }
  return "all-suffix-arms-accepted";
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 3) {
    std::cerr <<
      "usage: mpcc_prepared_suffix_replay SNAPSHOT_YAML ELAPSED_SEC\n";
    return 2;
  }
  char * end{};
  const double elapsed_sec = std::strtod(argv[2], &end);
  if (end == argv[2] || *end != '\0' || !std::isfinite(elapsed_sec) ||
    elapsed_sec < 0.0)
  {
    std::cerr << "invalid elapsed time\n";
    return 2;
  }

  std::string load_detail;
  auto recorded = architecture::load_recorded_interaction_snapshot(
    std::filesystem::path{argv[1]}, &load_detail);
  if (!recorded.has_value() ||
    !recorded->recorded_qp.warm_start.has_value())
  {
    std::cerr << "load failed: " << load_detail <<
      ", warm_available=" <<
      (recorded.has_value() &&
      recorded->recorded_qp.warm_start.has_value()) << '\n';
    return 2;
  }
  const auto & recorded_primal =
    recorded->recorded_qp.warm_start->primal;
  const auto latest = interpolate_recorded_state(
    recorded->source, recorded_primal, elapsed_sec);
  const auto previous_input = active_recorded_input(
    recorded->source, recorded_primal, elapsed_sec);
  if (!latest.has_value() || !previous_input.has_value()) {
    std::cerr << "recorded primal cannot provide the requested time probe\n";
    return 2;
  }

  shadow::LatestStateFeedbackPreparation preparation;
  preparation.snapshot = recorded->source;
  preparation.final_problem = recorded->assembly_request;
  preparation.prepared_primal = recorded_primal;
  solver::PersistentOsqpSolver feedback_solver(
    solver::ConstraintPreconditioningPolicy::RowToleranceNormalized);
  const auto build_started = Clock::now();
  const auto feedback = shadow::build_time_aligned_feedback_problem(
    shadow::TimeAlignedFeedbackProblemRequest{
      &preparation,
      recorded->source.control_prediction_origin_sec + elapsed_sec,
      latest.value(), previous_input.value(),
      feedback_solver.physical_constraint_tolerance()});
  const double build_ms = std::chrono::duration<double, std::milli>(
    Clock::now() - build_started).count();

  solver::SolveOutcome feedback_outcome;
  bool assembled = false;
  if (feedback.problem.has_value()) {
    const auto assembled_problem = problem::assemble(feedback.problem.value());
    assembled = assembled_problem.has_value();
    if (assembled_problem.has_value()) {
      feedback_outcome = feedback_solver.solve(
        assembled_problem->quadratic_cost, assembled_problem->constraints,
        assembled_problem->linear_cost, assembled_problem->lower_bound,
        assembled_problem->upper_bound, std::nullopt,
        assembled_problem->variable_scaling);
    }
  }

  shadow::SolverContext full_solver;
  const auto full = full_solver.evaluate(recorded->source);
  shadow::LatestStateFeedbackSolverContext time_aligned_solver;
  shadow::LatestStateFeedbackSolverContext old_origin_solver;
  shadow::LatestStateFeedbackSolverContext reachable_solver;
  shadow::LatestStateFeedbackSolverContext multi_sqp_solver;
  shadow::LatestStateFeedbackSolverContext nonlinear_interior_solver;
  const shadow::LatestStateFeedbackRequest latest_state_request{
    std::make_shared<const shadow::LatestStateFeedbackPreparation>(
      preparation),
    recorded->source.control_prediction_origin_sec + elapsed_sec,
    recorded->source.identity.snapshot_sec + elapsed_sec,
    latest.value(), previous_input.value()};
  const auto old_origin = old_origin_solver.evaluate(latest_state_request);
  const auto time_aligned = time_aligned_solver.evaluate_time_aligned(
    latest_state_request);
  const auto reachable =
    reachable_solver.evaluate_reachable_bridge_time_aligned(
    latest_state_request);
  const auto multi_sqp =
    multi_sqp_solver.evaluate_reachable_bridge_multi_sqp_audit(
    latest_state_request, 4U);
  const auto nonlinear_interior = nonlinear_interior_solver.
    evaluate_reachable_bridge_nonlinear_interior_wall_audit(
    latest_state_request, 4U);
  shadow::LatestStateFeedbackResult current_preparation_time_aligned;
  bool current_preparation_probe_available = false;
  if (full.latest_state_feedback_preparation != nullptr) {
    const auto current_latest = interpolate_recorded_state(
      recorded->source,
      full.latest_state_feedback_preparation->prepared_primal, elapsed_sec);
    const auto current_previous_input = active_recorded_input(
      recorded->source,
      full.latest_state_feedback_preparation->prepared_primal, elapsed_sec);
    if (current_latest.has_value() && current_previous_input.has_value()) {
      shadow::LatestStateFeedbackSolverContext current_preparation_solver;
      current_preparation_time_aligned =
        current_preparation_solver.evaluate_time_aligned(
        shadow::LatestStateFeedbackRequest{
          full.latest_state_feedback_preparation,
          recorded->source.control_prediction_origin_sec + elapsed_sec,
          recorded->source.identity.snapshot_sec + elapsed_sec,
          current_latest.value(), current_previous_input.value()});
      current_preparation_probe_available = true;
    }
  }
  std::cout <<
    "snapshot=" << std::filesystem::path{argv[1]}.generic_string() <<
    " intent=" << recorded->recorded_qp.intent <<
    " pipeline=" << recorded->recorded_qp.pipeline_stage <<
    " elapsed_sec=" << elapsed_sec <<
    " source_horizon=" << recorded->source.request.horizon_steps <<
    " suffix_horizon=" <<
    (feedback.suffix.snapshot.has_value() ?
    feedback.suffix.snapshot->request.horizon_steps : 0) <<
    " build_reason=" << shadow::to_string(feedback.reason) <<
    " build_ms=" << build_ms <<
    " assembled=" << assembled <<
    " feedback_solved=" << feedback_outcome.result.has_value() <<
    " feedback_status=" << feedback_outcome.telemetry.status <<
    " feedback_iterations=" << feedback_outcome.telemetry.iterations <<
    " feedback_solve_ms=" << feedback_outcome.telemetry.total_ms <<
    " bootstrap_feedback_reason=" << shadow::to_string(time_aligned.reason) <<
    " bootstrap_feedback_problem=" <<
    shadow::to_string(time_aligned.time_aligned_problem_reason) <<
    " bootstrap_feedback_iterations=" <<
    time_aligned.solver.iterations <<
    " bootstrap_feedback_solve_ms=" << time_aligned.solver.total_ms <<
    " bootstrap_feedback_compute_ms=" << time_aligned.compute_ms <<
    " current_preparation_probe=" << current_preparation_probe_available <<
    " current_preparation_reason=" <<
    shadow::to_string(current_preparation_time_aligned.reason) <<
    " current_preparation_problem=" << shadow::to_string(
    current_preparation_time_aligned.time_aligned_problem_reason) <<
    " current_preparation_iterations=" <<
    current_preparation_time_aligned.solver.iterations <<
    " current_preparation_solve_ms=" <<
    current_preparation_time_aligned.solver.total_ms <<
    " current_preparation_compute_ms=" <<
    current_preparation_time_aligned.compute_ms <<
    " suffix_old_origin_reason=" << shadow::to_string(old_origin.reason) <<
    " suffix_old_origin_solved=" << old_origin.solved <<
    " suffix_old_origin_physical=" <<
    multi_purpose_mpc_ros::mpcc_rate_resolved_physical_adapter::to_string(
    old_origin.physical_adapter_reason) <<
    " suffix_old_origin_exact=" <<
    multi_purpose_mpc_ros::race_mpcc_foundation::
    exact_physical_execution_trajectory_reason_name(
    old_origin.physical_exact_reason) <<
    " suffix_old_origin_exact_stage=" << old_origin.physical_rejected_stage <<
    " suffix_time_aligned_reason=" <<
    shadow::to_string(time_aligned.reason) <<
    " suffix_time_aligned_solved=" << time_aligned.solved <<
    " suffix_time_aligned_physical=" <<
    multi_purpose_mpc_ros::mpcc_rate_resolved_physical_adapter::to_string(
    time_aligned.physical_adapter_reason) <<
    " suffix_time_aligned_exact=" <<
    multi_purpose_mpc_ros::race_mpcc_foundation::
    exact_physical_execution_trajectory_reason_name(
    time_aligned.physical_exact_reason) <<
    " suffix_time_aligned_exact_stage=" <<
    time_aligned.physical_rejected_stage <<
    " suffix_reachable_reason=" << shadow::to_string(reachable.reason) <<
    " suffix_reachable_problem=" <<
    shadow::to_string(reachable.reachable_bridge_reason) <<
    " suffix_reachable_solved=" << reachable.solved <<
    " suffix_reachable_physical=" <<
    multi_purpose_mpc_ros::mpcc_rate_resolved_physical_adapter::to_string(
    reachable.physical_adapter_reason) <<
    " suffix_reachable_exact=" <<
    multi_purpose_mpc_ros::race_mpcc_foundation::
    exact_physical_execution_trajectory_reason_name(
    reachable.physical_exact_reason) <<
    " suffix_reachable_exact_stage=" << reachable.physical_rejected_stage <<
    " suffix_reachable_exact_lateral=" <<
    reachable.physical_rejected_lateral_m <<
    " suffix_reachable_exact_lower=" <<
    reachable.physical_rejected_lateral_lower_m <<
    " suffix_reachable_exact_upper=" <<
    reachable.physical_rejected_lateral_upper_m <<
    " suffix_reachable_exact_violation=" <<
    reachable.physical_lateral_violation_m <<
    " suffix_reachable_exact_tolerance=" <<
    reachable.physical_lateral_bound_tolerance_m <<
    " suffix_multi_sqp_reason=" << shadow::to_string(multi_sqp.reason) <<
    " suffix_multi_sqp_attempts=" <<
    multi_sqp.latest_state_multi_sqp_attempt_count <<
    " suffix_multi_sqp_solves=" <<
    multi_sqp.latest_state_multi_sqp_solve_count <<
    " suffix_multi_sqp_physical=" <<
    multi_purpose_mpc_ros::mpcc_rate_resolved_physical_adapter::to_string(
    multi_sqp.physical_adapter_reason) <<
    " suffix_multi_sqp_exact=" <<
    multi_purpose_mpc_ros::race_mpcc_foundation::
    exact_physical_execution_trajectory_reason_name(
    multi_sqp.physical_exact_reason) <<
    " suffix_multi_sqp_exact_stage=" <<
    multi_sqp.physical_rejected_stage <<
    " suffix_multi_sqp_exact_lateral=" <<
    multi_sqp.physical_rejected_lateral_m <<
    " suffix_multi_sqp_exact_lower=" <<
    multi_sqp.physical_rejected_lateral_lower_m <<
    " suffix_multi_sqp_exact_upper=" <<
    multi_sqp.physical_rejected_lateral_upper_m <<
    " suffix_multi_sqp_exact_violation=" <<
    multi_sqp.physical_lateral_violation_m <<
    " suffix_multi_sqp_exact_tolerance=" <<
    multi_sqp.physical_lateral_bound_tolerance_m <<
    " suffix_nonlinear_interior_reason=" <<
    shadow::to_string(nonlinear_interior.reason) <<
    " suffix_nonlinear_interior_rows=" <<
    nonlinear_interior.nonlinear_interior_wall_row_count <<
    " suffix_nonlinear_interior_problem=" << shadow::to_string(
    nonlinear_interior.nonlinear_interior_wall_reason) <<
    " suffix_nonlinear_interior_attempts=" <<
    nonlinear_interior.latest_state_multi_sqp_attempt_count <<
    " suffix_nonlinear_interior_solves=" <<
    nonlinear_interior.latest_state_multi_sqp_solve_count <<
    " suffix_nonlinear_interior_status=" <<
    nonlinear_interior.solver.status <<
    " suffix_nonlinear_interior_iterations=" <<
    nonlinear_interior.solver.iterations <<
    " suffix_nonlinear_interior_solve_ms=" <<
    nonlinear_interior.solver.total_ms <<
    " suffix_nonlinear_interior_candidate_violation=" <<
    nonlinear_interior.nonlinear_interior_wall_maximum_candidate_violation_m <<
    " suffix_nonlinear_interior_physical=" <<
    multi_purpose_mpc_ros::mpcc_rate_resolved_physical_adapter::to_string(
    nonlinear_interior.physical_adapter_reason) <<
    " suffix_nonlinear_interior_exact=" <<
    multi_purpose_mpc_ros::race_mpcc_foundation::
    exact_physical_execution_trajectory_reason_name(
    nonlinear_interior.physical_exact_reason) <<
    " suffix_nonlinear_interior_exact_stage=" <<
    nonlinear_interior.physical_rejected_stage <<
    " suffix_nonlinear_interior_exact_violation=" <<
    nonlinear_interior.physical_lateral_violation_m <<
    " suffix_nonlinear_interior_exact_tolerance=" <<
    nonlinear_interior.physical_lateral_bound_tolerance_m <<
    " suffix_classification=" << suffix_comparison_classification(
    old_origin, time_aligned, reachable, multi_sqp, nonlinear_interior) <<
    " full_outcome=" << shadow::to_string(full.outcome) <<
    " full_compute_ms=" << full.compute_ms <<
    " probe=recorded-primal-linear-interpolation" <<
    " detail=" << feedback.detail << '\n';
  // A rejected candidate is the diagnostic result, not a CLI failure. Loading
  // or constructing the immutable time probe still fails earlier with code 2.
  return 0;
}
