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
  std::cout <<
    "intent=" << recorded->recorded_qp.intent <<
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
    " full_outcome=" << shadow::to_string(full.outcome) <<
    " full_compute_ms=" << full.compute_ms <<
    " probe=recorded-primal-linear-interpolation" <<
    " detail=" << feedback.detail << '\n';
  return feedback.reason ==
         shadow::TimeAlignedFeedbackProblemReason::Accepted ? 0 : 1;
}
