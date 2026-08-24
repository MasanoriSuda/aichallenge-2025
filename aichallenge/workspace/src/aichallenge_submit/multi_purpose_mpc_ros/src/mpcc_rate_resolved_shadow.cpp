#include "multi_purpose_mpc_ros/mpcc_rate_resolved_shadow.hpp"

#include "multi_purpose_mpc_ros/mpcc_rate_resolved.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_problem.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_shadow
{
namespace
{

using SteadyClock = std::chrono::steady_clock;

}  // namespace

const char * to_string(const Outcome outcome) noexcept
{
  switch (outcome) {
    case Outcome::BuildRejected:
      return "build-rejected";
    case Outcome::AssemblyRejected:
      return "assembly-rejected";
    case Outcome::SolveRejected:
      return "solve-rejected";
    case Outcome::NonfiniteResult:
      return "nonfinite-result";
    case Outcome::ActuationSampleRejected:
      return "actuation-sample-rejected";
    case Outcome::ArtifactRejected:
      return "artifact-rejected";
    case Outcome::Solved:
      return "solved";
    case Outcome::Exception:
      return "exception";
    case Outcome::Count:
      break;
  }
  return "unknown";
}

bool identity_valid(const Identity & identity) noexcept
{
  return artifact::identity_valid(identity);
}

bool result_valid(const Result & result) noexcept
{
  if (
    result.outcome == Outcome::Count ||
    !artifact::identity_valid(result.identity) ||
    !std::isfinite(result.completed_sec) ||
    result.completed_sec < result.identity.snapshot_sec ||
    !std::isfinite(result.compute_ms) || result.compute_ms < 0.0)
  {
    return false;
  }
  if (result.outcome != Outcome::Solved) {
    if (result.outcome == Outcome::ActuationSampleRejected) {
      return !result.solved && !result.actuation_sampled &&
             result.actuation_sample_reason !=
             mpcc_rate_resolved::ActuationSampleReason::Accepted &&
             result.actuation_sample_reason !=
             mpcc_rate_resolved::ActuationSampleReason::Count;
    }
    if (result.outcome == Outcome::ArtifactRejected) {
      return !result.solved && result.actuation_sampled &&
             result.execution_artifact == nullptr &&
             result.execution_artifact_reject_reason !=
             artifact::RejectReason::None;
    }
    return !result.solved && !result.actuation_sampled &&
           result.actuation_sample_reason ==
           mpcc_rate_resolved::ActuationSampleReason::Count;
  }
  return result.adapter_built && result.assembled && result.solve_attempted &&
         result.solved && result.finite && result.constraints_satisfied &&
         result.actuation_sampled &&
         result.actuation_sample_reason ==
         mpcc_rate_resolved::ActuationSampleReason::Accepted &&
         std::isfinite(result.first_acceleration_mps2) &&
         std::isfinite(result.first_steering_rate_radps) &&
         std::isfinite(result.first_virtual_progress_speed_mps) &&
         std::isfinite(result.initial_steering_rad) &&
         std::isfinite(result.solver_initial_steering_rad) &&
         std::isfinite(result.sampled_steering_rad) &&
         std::isfinite(result.first_steering_rate_physical_lower_radps) &&
         std::isfinite(result.first_steering_rate_physical_upper_radps) &&
         std::isfinite(result.first_steering_rate_solver_lower_radps) &&
         std::isfinite(result.first_steering_rate_solver_upper_radps) &&
         std::isfinite(result.first_steering_rate_certificate_margin_radps) &&
         result.first_steering_rate_certificate_margin_radps >= 0.0 &&
         result.first_steering_rate_physical_lower_radps <=
         result.first_steering_rate_solver_lower_radps &&
         result.first_steering_rate_solver_lower_radps <=
         result.first_steering_rate_solver_upper_radps &&
         result.first_steering_rate_solver_upper_radps <=
         result.first_steering_rate_physical_upper_radps &&
         result.certified_stage_count > 0U &&
         result.sampled_stage_index < result.certified_stage_count &&
         std::isfinite(result.sampled_stage_elapsed_sec) &&
         result.sampled_stage_elapsed_sec >= 0.0 &&
         std::isfinite(result.certified_horizon_duration_sec) &&
         result.certified_horizon_duration_sec > 0.0 &&
         result.publication_interval_sec <=
         result.certified_horizon_duration_sec + 1e-12 &&
         std::isfinite(result.sampled_curvature_radpm) &&
         std::isfinite(result.terminal_velocity_mps) &&
         std::isfinite(result.terminal_progress_m) &&
         std::isfinite(result.terminal_steering_rad) &&
         std::isfinite(result.maximum_constraint_violation) &&
         std::isfinite(result.maximum_normalized_constraint_violation) &&
         result.maximum_normalized_constraint_row >= 0 &&
         result.execution_artifact_reject_reason ==
         artifact::RejectReason::None &&
         result.execution_artifact != nullptr &&
         artifact::validate(*result.execution_artifact) ==
         artifact::RejectReason::None;
}

Result SolverContext::evaluate(const Snapshot & snapshot)
{
  const auto started = SteadyClock::now();
  Result result;
  result.identity = snapshot.identity;
  const auto finish = [&result, &snapshot, &started]() {
      result.compute_ms = std::chrono::duration<double, std::milli>(
        SteadyClock::now() - started).count();
      result.completed_sec = snapshot.identity.snapshot_sec +
        result.compute_ms * 1.0e-3;
      return result;
    };
  if (
    !artifact::identity_valid(snapshot.identity) ||
    !std::isfinite(snapshot.course_progress_origin_m) ||
    snapshot.request.horizon_steps <= 0 ||
    snapshot.nominal_path_distance_m.size() !=
    static_cast<std::size_t>(snapshot.request.horizon_steps) + 1U ||
    !std::isfinite(snapshot.publication_interval_sec) ||
    snapshot.publication_interval_sec <= 0.0)
  {
    result.detail = "invalid shadow snapshot identity/timing";
    return finish();
  }

  const auto adapted = mpcc_rate_resolved_adapter::build(
    snapshot.request, solver_.physical_constraint_tolerance());
  if (!adapted.has_value()) {
    result.detail = "rate-resolved semantic adapter rejected snapshot";
    return finish();
  }
  result.adapter_built = true;

  const auto assembled =
    mpcc_rate_resolved_problem::assemble(adapted->problem);
  if (!assembled.has_value()) {
    result.outcome = Outcome::AssemblyRejected;
    result.detail = "rate-resolved QP assembly rejected snapshot";
    return finish();
  }
  result.assembled = true;
  result.first_steering_rate_physical_lower_radps =
    adapted->first_steering_rate_physical_lower_radps;
  result.first_steering_rate_physical_upper_radps =
    adapted->first_steering_rate_physical_upper_radps;
  result.first_steering_rate_solver_lower_radps =
    adapted->first_steering_rate_solver_lower_radps;
  result.first_steering_rate_solver_upper_radps =
    adapted->first_steering_rate_solver_upper_radps;
  result.first_steering_rate_certificate_margin_radps =
    adapted->first_steering_rate_certificate_margin_radps;
  result.solve_attempted = true;

  std::lock_guard<std::mutex> lock(mutex_);
  const auto outcome = solver_.solve(
    assembled->quadratic_cost, assembled->constraints,
    assembled->linear_cost, assembled->lower_bound, assembled->upper_bound,
    std::nullopt, assembled->variable_scaling);
  result.solver = outcome.telemetry;
  if (!outcome.result.has_value()) {
    result.outcome = Outcome::SolveRejected;
    result.detail = outcome.failure_detail;
    return finish();
  }
  result.solved = true;
  result.finite = outcome.result->primal.allFinite();
  if (!result.finite) {
    result.outcome = Outcome::NonfiniteResult;
    result.detail = "rate-resolved solver returned non-finite primal";
    result.solved = false;
    return finish();
  }
  result.constraints_satisfied = true;
  result.maximum_constraint_violation =
    outcome.result->maximum_constraint_violation;
  result.maximum_normalized_constraint_violation =
    outcome.result->maximum_normalized_constraint_violation;
  result.maximum_normalized_constraint_row =
    outcome.result->maximum_normalized_constraint_row;

  namespace model = mpcc_rate_resolved;
  const int horizon = snapshot.request.horizon_steps;
  const int state_values = model::kStateDimension * (horizon + 1);
  const auto & primal = outcome.result->primal;
  result.initial_steering_rad = snapshot.request.current_steering_rad;
  result.solver_initial_steering_rad = primal[model::kSteeringIndex];
  result.first_acceleration_mps2 =
    primal[state_values + model::kAccelerationIndex];
  result.first_steering_rate_radps =
    primal[state_values + model::kSteeringRateIndex];
  result.first_virtual_progress_speed_mps =
    primal[state_values + model::kVirtualProgressSpeedIndex];
  result.first_stage_duration_sec = snapshot.request.inputs.front().stage_dt_sec;
  result.publication_interval_sec = snapshot.publication_interval_sec;
  result.maximum_abs_steering_rad =
    snapshot.request.maximum_abs_steering_rad;
  result.maximum_abs_steering_rate_radps =
    snapshot.request.maximum_abs_steering_rate_radps;
  const int terminal_state = model::kStateDimension * horizon;
  result.terminal_velocity_mps =
    primal[terminal_state + model::kVelocityIndex];
  result.terminal_progress_m =
    primal[terminal_state + model::kProgressIndex];
  result.terminal_steering_rad =
    primal[terminal_state + model::kSteeringIndex];
  std::vector<double> certified_steering_rates_radps;
  std::vector<double> stage_durations_sec;
  certified_steering_rates_radps.reserve(static_cast<std::size_t>(horizon));
  stage_durations_sec.reserve(static_cast<std::size_t>(horizon));
  for (int stage = 0; stage < horizon; ++stage) {
    const int input_offset =
      state_values + model::kInputDimension * stage;
    certified_steering_rates_radps.push_back(
      primal[input_offset + model::kSteeringRateIndex]);
    stage_durations_sec.push_back(
      snapshot.request.inputs[static_cast<std::size_t>(stage)].stage_dt_sec);
  }
  result.certified_stage_count = certified_steering_rates_radps.size();
  result.calculated_terminal_steering_rad =
    result.initial_steering_rad + result.first_steering_rate_radps *
    result.first_stage_duration_sec;
  const auto sample = model::evaluate_certified_actuation_sequence_sample(
    model::CertifiedActuationSequenceSampleRequest{
      result.initial_steering_rad, std::move(certified_steering_rates_radps),
      std::move(stage_durations_sec), snapshot.publication_interval_sec,
      snapshot.request.maximum_abs_steering_rad,
      snapshot.request.wheelbase_m,
      result.maximum_normalized_constraint_violation});
  result.actuation_sample_reason = sample.reason;
  result.sampled_steering_rad = sample.sampled_steering_rad;
  result.sampled_stage_index = sample.sampled_stage_index;
  result.sampled_stage_elapsed_sec = sample.sampled_stage_elapsed_sec;
  result.certified_horizon_duration_sec =
    sample.certified_horizon_duration_sec;
  if (!sample.sample.has_value()) {
    result.outcome = Outcome::ActuationSampleRejected;
    result.detail = std::string{"actuation sample rejected: "} +
      model::to_string(sample.reason);
    result.solved = false;
    result.constraints_satisfied = false;
    return finish();
  }
  result.actuation_sampled = true;
  result.sampled_steering_rad = sample.sample->steering_rad;
  result.sampled_curvature_radpm = sample.sample->curvature_radpm;

  artifact::ExecutionArtifact execution_artifact;
  execution_artifact.identity = snapshot.identity;
  execution_artifact.prediction_origin_sec = snapshot.identity.snapshot_sec;
  execution_artifact.completed_sec = snapshot.identity.snapshot_sec +
    std::chrono::duration<double>(SteadyClock::now() - started).count();
  execution_artifact.course_progress_origin_m =
    snapshot.course_progress_origin_m;
  execution_artifact.semantic_initial_steering_rad =
    snapshot.request.current_steering_rad;
  execution_artifact.wheelbase_m = snapshot.request.wheelbase_m;
  execution_artifact.maximum_abs_steering_rad =
    snapshot.request.maximum_abs_steering_rad;
  execution_artifact.maximum_abs_steering_rate_radps =
    snapshot.request.maximum_abs_steering_rate_radps;
  execution_artifact.physical_global_tolerance =
    outcome.telemetry.physical_global_tolerance;
  execution_artifact.maximum_constraint_violation =
    outcome.result->maximum_constraint_violation;
  execution_artifact.maximum_normalized_constraint_violation =
    outcome.result->maximum_normalized_constraint_violation;
  execution_artifact.predicted_states.reserve(
    static_cast<std::size_t>(horizon + 1));
  execution_artifact.nominal_path_distance_m =
    snapshot.nominal_path_distance_m;
  execution_artifact.lateral_lower_m.reserve(
    static_cast<std::size_t>(horizon + 1));
  execution_artifact.lateral_upper_m.reserve(
    static_cast<std::size_t>(horizon + 1));
  for (int stage = 0; stage <= horizon; ++stage) {
    const int state_offset = model::kStateDimension * stage;
    execution_artifact.predicted_states.push_back(artifact::PredictedState{
      primal[state_offset + model::kLateralIndex],
      primal[state_offset + model::kLagIndex],
      primal[state_offset + model::kHeadingIndex],
      primal[state_offset + model::kVelocityIndex],
      primal[state_offset + model::kProgressIndex],
      primal[state_offset + model::kSteeringIndex]});
    const auto & semantic =
      snapshot.request.states[static_cast<std::size_t>(stage)];
    execution_artifact.lateral_lower_m.push_back(semantic.lower[0]);
    execution_artifact.lateral_upper_m.push_back(semantic.upper[0]);
  }
  execution_artifact.control_stages.reserve(static_cast<std::size_t>(horizon));
  for (int stage = 0; stage < horizon; ++stage) {
    const int input_offset =
      state_values + model::kInputDimension * stage;
    execution_artifact.control_stages.push_back(artifact::ControlStage{
      primal[input_offset + model::kAccelerationIndex],
      primal[input_offset + model::kSteeringRateIndex],
      primal[input_offset + model::kVirtualProgressSpeedIndex],
      snapshot.request.inputs[static_cast<std::size_t>(stage)].stage_dt_sec});
  }
  result.execution_artifact_reject_reason =
    artifact::validate(execution_artifact);
  if (
    result.execution_artifact_reject_reason !=
    artifact::RejectReason::None)
  {
    result.outcome = Outcome::ArtifactRejected;
    result.detail = std::string{"execution artifact rejected: "} +
      artifact::to_string(result.execution_artifact_reject_reason);
    result.solved = false;
    result.constraints_satisfied = false;
    return finish();
  }
  result.execution_artifact =
    std::make_shared<const artifact::ExecutionArtifact>(
    std::move(execution_artifact));
  result.outcome = Outcome::Solved;
  result.detail = "accepted";
  return finish();
}

const char * to_string(const PublishReason reason) noexcept
{
  switch (reason) {
    case PublishReason::Accepted:
      return "accepted";
    case PublishReason::InvalidResult:
      return "invalid-result";
    case PublishReason::SequenceRollback:
      return "sequence-rollback";
    case PublishReason::SequenceNotSubmitted:
      return "sequence-not-submitted";
  }
  return "unknown";
}

bool Mailbox::register_submission(const std::uint64_t sequence)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (sequence == 0U || sequence <= latest_submitted_sequence_) {
    return false;
  }
  latest_submitted_sequence_ = sequence;
  return true;
}

PublishReason Mailbox::publish(Result result)
{
  if (!result_valid(result)) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++invalid_result_count_;
    last_reason_ = PublishReason::InvalidResult;
    return last_reason_;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (result.identity.sequence <= latest_published_sequence_) {
    ++sequence_rollback_count_;
    last_reason_ = PublishReason::SequenceRollback;
    return last_reason_;
  }
  if (result.identity.sequence > latest_submitted_sequence_) {
    ++sequence_not_submitted_count_;
    last_reason_ = PublishReason::SequenceNotSubmitted;
    return last_reason_;
  }
  latest_published_sequence_ = result.identity.sequence;
  latest_result_ = std::move(result);
  ++accepted_count_;
  last_reason_ = PublishReason::Accepted;
  return last_reason_;
}

std::optional<Result> Mailbox::latest_after(
  const std::uint64_t consumed_sequence) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (
    !latest_result_.has_value() ||
    latest_result_->identity.sequence <= consumed_sequence)
  {
    return std::nullopt;
  }
  return latest_result_;
}

MailboxState Mailbox::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return MailboxState{
    latest_submitted_sequence_, latest_published_sequence_, accepted_count_,
    invalid_result_count_, sequence_rollback_count_,
    sequence_not_submitted_count_, last_reason_, latest_result_.has_value()};
}

}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_shadow
