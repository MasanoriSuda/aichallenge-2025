#include "multi_purpose_mpc_ros/mpcc_rate_resolved_shadow.hpp"

#include "multi_purpose_mpc_ros/mpcc_architecture_snapshot.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_physical_adapter.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_problem.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_wall_refinement.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <utility>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_shadow
{
namespace
{

using SteadyClock = std::chrono::steady_clock;

// The first exact replay is free of extra solver work.  If a physically
// refined candidate still exposes a nonlinear model/proof gap, at most three
// current-problem SQP corrections are allowed before failing closed.  This is
// a deterministic certification budget, not a fallback or a lease extension.
constexpr std::size_t kMaximumPhysicalProofSqpCorrections = 3U;

struct ProgressWallRefinement
{
  mpcc_progress::ProgressAlignedWallBoundsResolution resolution;
  mpcc_rate_resolved_wall_refinement::Result physical;
  std::optional<mpcc_rate_resolved_problem::AssemblyRequest> request;
};

ProgressWallRefinement build_progress_wall_refinement(
  const Snapshot & snapshot,
  const mpcc_rate_resolved_problem::AssemblyRequest & initial_request,
  const Eigen::VectorXd & primal,
  const persistent_osqp::PhysicalConstraintTolerance &
  physical_constraint_tolerance) noexcept
{
  namespace model = mpcc_rate_resolved;
  ProgressWallRefinement result;
  mpcc_progress::ProgressAlignedWallBoundsRequest request;
  request.active = snapshot.progress_aligned_wall_refinement_active;
  request.reference_progress_m = snapshot.wall_reference_progress_m;
  request.wall_lower_m = snapshot.wall_lower_m;
  request.wall_upper_m = snapshot.wall_upper_m;
  if (!request.reference_progress_m.empty()) {
    const double maximum_endpoint_magnitude = std::max(
      std::abs(request.reference_progress_m.front()),
      std::abs(request.reference_progress_m.back()));
    request.boundary_tolerance_m = physical_constraint_tolerance.absolute +
      physical_constraint_tolerance.relative * maximum_endpoint_magnitude;
  }
  if (!request.active) {
    result.resolution =
      mpcc_progress::resolve_progress_aligned_wall_bounds(request);
    return result;
  }

  const int horizon = snapshot.request.horizon_steps;
  const int state_values = model::kStateDimension * (horizon + 1);
  const int variable_count = state_values + model::kInputDimension * horizon;
  if (
    horizon <= 0 || primal.size() != variable_count ||
    initial_request.state_lower.size() != state_values ||
    initial_request.state_upper.size() != state_values)
  {
    result.resolution =
      mpcc_progress::resolve_progress_aligned_wall_bounds(request);
    return result;
  }

  request.solved_progress_m.reserve(static_cast<std::size_t>(horizon));
  request.current_lower_m.reserve(static_cast<std::size_t>(horizon));
  request.current_upper_m.reserve(static_cast<std::size_t>(horizon));
  request.current_progress_lower_m.reserve(static_cast<std::size_t>(horizon));
  request.current_progress_upper_m.reserve(static_cast<std::size_t>(horizon));
  for (int stage = 0; stage < horizon; ++stage) {
    const int state = (stage + 1) * model::kStateDimension;
    request.solved_progress_m.push_back(
      primal[state + model::kProgressIndex]);
    request.current_lower_m.push_back(
      initial_request.state_lower[state + model::kLateralIndex]);
    request.current_upper_m.push_back(
      initial_request.state_upper[state + model::kLateralIndex]);
    request.current_progress_lower_m.push_back(
      initial_request.state_lower[state + model::kProgressIndex]);
    request.current_progress_upper_m.push_back(
      initial_request.state_upper[state + model::kProgressIndex]);
  }
  result.resolution =
    mpcc_progress::resolve_progress_aligned_wall_bounds(request);
  if (
    !result.resolution.valid || !result.resolution.feasible ||
    !result.resolution.applied)
  {
    return result;
  }

  auto refined = initial_request;
  mpcc_rate_resolved_problem::ProgressAlignedWallConstraints wall;
  wall.lower_slope = result.resolution.wall_lower_slope;
  wall.lower_intercept = result.resolution.wall_lower_intercept;
  wall.upper_slope = result.resolution.wall_upper_slope;
  wall.upper_intercept = result.resolution.wall_upper_intercept;
  mpcc_rate_resolved_wall_refinement::Request physical_request;
  physical_request.active = snapshot.physical_wall_refinement_active;
  physical_request.wall_grid = snapshot.wall_grid.get();
  physical_request.footprint = snapshot.wall_footprint;
  physical_request.course_frame_knots = snapshot.wall_course_frame_knots;
  physical_request.course_progress_origin_m = snapshot.course_progress_origin_m;
  physical_request.heading_bucket_width_rad =
    snapshot.wall_heading_bucket_width_rad;
  physical_request.translation_bucket_width_m =
    snapshot.wall_translation_bucket_width_m;
  physical_request.lateral_sample_step_m =
    snapshot.wall_lateral_sample_step_m;
  physical_request.boundary_guard_m = snapshot.wall_boundary_guard_m;
  physical_request.initial_stage =
    mpcc_rate_resolved_wall_refinement::StageRequest{
      primal[model::kProgressIndex],
      primal[model::kLateralIndex],
      primal[model::kLagIndex],
      primal[model::kHeadingIndex],
      initial_request.state_lower[model::kLateralIndex],
      initial_request.state_upper[model::kLateralIndex],
      initial_request.state_lower[model::kLagIndex],
      initial_request.state_upper[model::kLagIndex],
      initial_request.state_lower[model::kHeadingIndex],
      initial_request.state_upper[model::kHeadingIndex],
      initial_request.state_lower[model::kProgressIndex],
      initial_request.state_upper[model::kProgressIndex]};
  physical_request.stages.reserve(static_cast<std::size_t>(horizon));
  for (int stage = 0; stage < horizon; ++stage) {
    const int state = (stage + 1) * model::kStateDimension;
    const auto index = static_cast<std::size_t>(stage);
    refined.state_lower[state + model::kProgressIndex] =
      result.resolution.progress_lower_m[index];
    refined.state_upper[state + model::kProgressIndex] =
      result.resolution.progress_upper_m[index];
    const double solved_progress_m =
      primal[state + model::kProgressIndex];
    physical_request.stages.push_back(
      mpcc_rate_resolved_wall_refinement::StageRequest{
        solved_progress_m,
        primal[state + model::kLateralIndex],
        primal[state + model::kLagIndex],
        primal[state + model::kHeadingIndex],
        std::max(
          initial_request.state_lower[state + model::kLateralIndex],
          wall.lower_slope[index] * solved_progress_m +
          wall.lower_intercept[index]),
        std::min(
          initial_request.state_upper[state + model::kLateralIndex],
          wall.upper_slope[index] * solved_progress_m +
          wall.upper_intercept[index]),
        initial_request.state_lower[state + model::kLagIndex],
        initial_request.state_upper[state + model::kLagIndex],
        initial_request.state_lower[state + model::kHeadingIndex],
        initial_request.state_upper[state + model::kHeadingIndex],
        result.resolution.progress_lower_m[index],
        result.resolution.progress_upper_m[index]});
  }
  result.physical =
    mpcc_rate_resolved_wall_refinement::resolve(physical_request);
  if (snapshot.physical_wall_refinement_active) {
    if (!result.physical.applied ||
      result.physical.stages.size() != static_cast<std::size_t>(horizon))
    {
      return result;
    }
    for (int stage = 0; stage < horizon; ++stage) {
      const int state = (stage + 1) * model::kStateDimension;
      const auto & bounds = result.physical.stages[
        static_cast<std::size_t>(stage)];
      refined.state_lower[state + model::kLateralIndex] =
        bounds.lateral_lower_m;
      refined.state_upper[state + model::kLateralIndex] =
        bounds.lateral_upper_m;
      refined.state_lower[state + model::kLagIndex] = bounds.lag_lower_m;
      refined.state_upper[state + model::kLagIndex] = bounds.lag_upper_m;
      refined.state_lower[state + model::kHeadingIndex] =
        bounds.heading_lower_rad;
      refined.state_upper[state + model::kHeadingIndex] =
        bounds.heading_upper_rad;
      refined.state_lower[state + model::kProgressIndex] =
        bounds.progress_lower_m;
      refined.state_upper[state + model::kProgressIndex] =
        bounds.progress_upper_m;
    }
    refined.swept_lateral_wall_constraints.reserve(
      result.physical.swept_lateral_constraints.size());
    for (const auto & constraint :
      result.physical.swept_lateral_constraints)
    {
      refined.swept_lateral_wall_constraints.push_back(
        mpcc_rate_resolved_problem::SweptLateralWallConstraint{
          constraint.transition_stage,
          constraint.destination_ratio,
          constraint.lateral_lower_m,
          constraint.lateral_upper_m});
    }
  }
  refined.progress_aligned_wall_constraints = std::move(wall);
  result.request = std::move(refined);
  return result;
}

struct ExecutionArtifactBuildResult
{
  std::optional<artifact::ExecutionArtifact> execution_artifact;
  artifact::RejectReason reject_reason{artifact::RejectReason::InvalidCertificate};
  const char * detail{"construction-not-attempted"};
};

ExecutionArtifactBuildResult build_execution_artifact(
  const Snapshot & snapshot,
  const mpcc_rate_resolved_problem::AssemblyRequest & final_problem,
  const persistent_osqp::SolveOutcome & outcome,
  const double completed_sec) noexcept
{
  namespace model = mpcc_rate_resolved;
  if (!outcome.result.has_value()) {
    return {std::nullopt, artifact::RejectReason::InvalidCertificate,
      "missing-solve-result"};
  }
  if (!outcome.result->primal.allFinite()) {
    return {std::nullopt, artifact::RejectReason::InvalidCertificate,
      "nonfinite-solve-result"};
  }
  if (!std::isfinite(completed_sec)) {
    return {std::nullopt, artifact::RejectReason::InvalidTiming,
      "invalid-completion-time"};
  }
  const int horizon = snapshot.request.horizon_steps;
  const int execution_horizon = snapshot.execution_prefix_steps;
  const int state_values = model::kStateDimension * (horizon + 1);
  const int variable_count =
    state_values + model::kInputDimension * horizon;
  if (
    horizon <= 0 || execution_horizon <= 0 ||
    execution_horizon > horizon ||
    outcome.result->primal.size() != variable_count ||
    final_problem.state_lower.size() != state_values ||
    final_problem.state_upper.size() != state_values ||
    snapshot.nominal_path_distance_m.size() <
    static_cast<std::size_t>(execution_horizon + 1))
  {
    return {std::nullopt, artifact::RejectReason::InvalidCertificate,
      "dimension-contract-mismatch"};
  }

  const auto & primal = outcome.result->primal;
  artifact::ExecutionArtifact execution_artifact;
  execution_artifact.identity = snapshot.identity;
  execution_artifact.prediction_origin_sec =
    snapshot.control_prediction_origin_sec;
  execution_artifact.publication_interval_sec =
    snapshot.publication_interval_sec;
  execution_artifact.completed_sec = completed_sec;
  execution_artifact.course_progress_origin_m =
    snapshot.course_progress_origin_m;
  execution_artifact.semantic_initial_steering_rad =
    snapshot.request.current_steering_rad;
  execution_artifact.semantic_initial_response_steering_rad =
    snapshot.request.current_response_steering_rad;
  execution_artifact.wheelbase_m = snapshot.request.wheelbase_m;
  execution_artifact.yaw_response_gain = snapshot.request.yaw_response_gain;
  execution_artifact.yaw_response_time_constant_sec =
    snapshot.request.yaw_response_time_constant_sec;
  execution_artifact.minimum_frenet_denominator =
    snapshot.request.minimum_frenet_denominator;
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
    static_cast<std::size_t>(execution_horizon + 1));
  execution_artifact.nominal_path_distance_m.assign(
    snapshot.nominal_path_distance_m.begin(),
    snapshot.nominal_path_distance_m.begin() + execution_horizon + 1);
  execution_artifact.lateral_lower_m.reserve(
    static_cast<std::size_t>(execution_horizon + 1));
  execution_artifact.lateral_upper_m.reserve(
    static_cast<std::size_t>(execution_horizon + 1));
  for (int stage = 0; stage <= execution_horizon; ++stage) {
    const int state_offset = model::kStateDimension * stage;
    execution_artifact.predicted_states.push_back(
      artifact::PredictedState{
        primal[state_offset + model::kLateralIndex],
        primal[state_offset + model::kLagIndex],
        primal[state_offset + model::kHeadingIndex],
        primal[state_offset + model::kVelocityIndex],
        primal[state_offset + model::kProgressIndex],
        primal[state_offset + model::kSteeringIndex],
        primal[state_offset + model::kResponseSteeringIndex]});
    execution_artifact.lateral_lower_m.push_back(
      final_problem.state_lower[
        state_offset + model::kLateralIndex]);
    execution_artifact.lateral_upper_m.push_back(
      final_problem.state_upper[
        state_offset + model::kLateralIndex]);
  }
  execution_artifact.control_stages.reserve(
    static_cast<std::size_t>(execution_horizon));
  for (int stage = 0; stage < execution_horizon; ++stage) {
    const int input_offset =
      state_values + model::kInputDimension * stage;
    execution_artifact.control_stages.push_back(
      artifact::ControlStage{
        primal[input_offset + model::kAccelerationIndex],
        primal[input_offset + model::kSteeringRateIndex],
        primal[input_offset + model::kVirtualProgressSpeedIndex],
        snapshot.request.inputs[static_cast<std::size_t>(stage)].stage_dt_sec,
        snapshot.request.inputs[static_cast<std::size_t>(stage)].lower[2],
        snapshot.request.inputs[static_cast<std::size_t>(stage)].upper[2],
        snapshot.request.inputs[static_cast<std::size_t>(stage)].lower[0],
        snapshot.request.inputs[static_cast<std::size_t>(stage)].upper[0],
        snapshot.request.inputs[
          static_cast<std::size_t>(stage)].path_curvature_radpm});
  }
  const auto reject_reason = artifact::validate(execution_artifact);
  if (reject_reason != artifact::RejectReason::None) {
    return {std::nullopt, reject_reason, "artifact-validation-rejected"};
  }
  return {std::move(execution_artifact), artifact::RejectReason::None, "accepted"};
}

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
    case Outcome::PhysicalProofRejected:
      return "physical-proof-rejected";
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

const char * to_string(const RecedingWarmStartReason reason) noexcept
{
  switch (reason) {
    case RecedingWarmStartReason::Available:
      return "available";
    case RecedingWarmStartReason::CurrentProblemBootstrap:
      return "current-problem-bootstrap";
    case RecedingWarmStartReason::EmptyCache:
      return "empty-cache";
    case RecedingWarmStartReason::InvalidPrevious:
      return "invalid-previous";
    case RecedingWarmStartReason::InvalidCurrent:
      return "invalid-current";
    case RecedingWarmStartReason::SemanticMismatch:
      return "semantic-mismatch";
    case RecedingWarmStartReason::TimeRegression:
      return "time-regression";
    case RecedingWarmStartReason::HorizonExhausted:
      return "horizon-exhausted";
    case RecedingWarmStartReason::DimensionMismatch:
      return "dimension-mismatch";
  }
  return "unknown";
}

RecedingWarmStartResolution resolve_receding_warm_start(
  const RecedingWarmStartSeed & previous, const Snapshot & current,
  const mpcc_rate_resolved_problem::AssemblyRequest & current_problem,
  const std::size_t current_constraint_count) noexcept
{
  namespace model = mpcc_rate_resolved;
  RecedingWarmStartResolution resolution;
  const auto & previous_context = previous.identity.source_context;
  const auto & current_context = current.identity.source_context;
  if (
    !artifact::identity_valid(previous.identity) ||
    !std::isfinite(previous.control_prediction_origin_sec) ||
    !std::isfinite(previous.course_progress_origin_m) ||
    previous.stage_durations_sec.empty() || !previous.primal.allFinite())
  {
    resolution.reason = RecedingWarmStartReason::InvalidPrevious;
    return resolution;
  }
  if (
    !artifact::identity_valid(current.identity) ||
    !std::isfinite(current.control_prediction_origin_sec) ||
    !std::isfinite(current.course_progress_origin_m) ||
    current_problem.horizon_steps <= 0 ||
    current_constraint_count == 0U || !current_problem.initial_state.allFinite())
  {
    resolution.reason = RecedingWarmStartReason::InvalidCurrent;
    return resolution;
  }
  const std::size_t horizon = static_cast<std::size_t>(
    current_problem.horizon_steps);
  const std::size_t previous_horizon =
    previous_context.horizon_steps;
  if (
    previous_context.formulation !=
    mpcc_execution_contract::Formulation::
    VelocitySteeringYawResponseProgress7State ||
    current_context.formulation != previous_context.formulation ||
    current_context.horizon_steps != horizon ||
    previous_horizon == 0U ||
    previous_context.state_schema_id != current_context.state_schema_id ||
    previous_context.input_schema_id != current_context.input_schema_id ||
    previous_context.bounds_schema_id != current_context.bounds_schema_id ||
    previous_context.cost_schema_id != current_context.cost_schema_id ||
    previous_context.intent != current_context.intent ||
    previous_context.intent_generation != current_context.intent_generation ||
    previous_context.target_id != current_context.target_id ||
    previous_context.execution_side_sign != current_context.execution_side_sign)
  {
    resolution.reason = RecedingWarmStartReason::SemanticMismatch;
    if (
      previous_context.formulation !=
      mpcc_execution_contract::Formulation::
      VelocitySteeringYawResponseProgress7State ||
      current_context.formulation != previous_context.formulation)
    {
      resolution.diagnostic = "formulation";
    } else if (current_context.horizon_steps != horizon) {
      resolution.diagnostic = "current-horizon";
    } else if (previous_horizon == 0U) {
      resolution.diagnostic = "previous-horizon";
    } else if (
      previous_context.state_schema_id != current_context.state_schema_id)
    {
      resolution.diagnostic = "state-schema";
    } else if (
      previous_context.input_schema_id != current_context.input_schema_id)
    {
      resolution.diagnostic = "input-schema";
    } else if (
      previous_context.bounds_schema_id != current_context.bounds_schema_id)
    {
      resolution.diagnostic = "bounds-schema";
    } else if (
      previous_context.cost_schema_id != current_context.cost_schema_id)
    {
      resolution.diagnostic = "cost-schema";
    } else if (previous_context.intent != current_context.intent) {
      resolution.diagnostic = "intent";
    } else if (
      previous_context.intent_generation != current_context.intent_generation)
    {
      resolution.diagnostic = "intent-generation";
    } else if (previous_context.target_id != current_context.target_id) {
      resolution.diagnostic = "target-id";
    } else {
      resolution.diagnostic = "execution-side";
    }
    return resolution;
  }
  if (previous.stage_durations_sec.size() != previous_horizon) {
    resolution.reason = RecedingWarmStartReason::DimensionMismatch;
    return resolution;
  }
  double elapsed_sec = current.control_prediction_origin_sec -
    previous.control_prediction_origin_sec;
  constexpr double kTimeToleranceSec = 1.0e-9;
  if (!std::isfinite(elapsed_sec) || elapsed_sec < -kTimeToleranceSec) {
    resolution.reason = RecedingWarmStartReason::TimeRegression;
    return resolution;
  }
  elapsed_sec = std::max(0.0, elapsed_sec);
  for (const double duration_sec : previous.stage_durations_sec) {
    if (!std::isfinite(duration_sec) || duration_sec <= 0.0) {
      resolution.reason = RecedingWarmStartReason::InvalidPrevious;
      return resolution;
    }
    if (elapsed_sec + kTimeToleranceSec < duration_sec) {
      break;
    }
    elapsed_sec = std::max(0.0, elapsed_sec - duration_sec);
    ++resolution.stage_advance;
  }
  if (resolution.stage_advance >= previous_horizon) {
    resolution.reason = RecedingWarmStartReason::HorizonExhausted;
    return resolution;
  }

  const std::size_t state_stage_count = horizon + 1U;
  const std::size_t previous_state_stage_count = previous_horizon + 1U;
  const std::size_t state_values =
    state_stage_count * static_cast<std::size_t>(model::kStateDimension);
  const std::size_t input_values =
    horizon * static_cast<std::size_t>(model::kInputDimension);
  const std::size_t variable_count = state_values + input_values;
  const std::size_t previous_state_values =
    previous_state_stage_count *
    static_cast<std::size_t>(model::kStateDimension);
  const std::size_t previous_input_values =
    previous_horizon * static_cast<std::size_t>(model::kInputDimension);
  const std::size_t previous_variable_count =
    previous_state_values + previous_input_values;
  if (
    previous.primal.size() !=
    static_cast<Eigen::Index>(previous_variable_count) ||
    current_problem.state_lower.size() !=
    static_cast<Eigen::Index>(state_values) ||
    current_problem.state_upper.size() !=
    static_cast<Eigen::Index>(state_values))
  {
    resolution.reason = RecedingWarmStartReason::DimensionMismatch;
    return resolution;
  }

  Eigen::VectorXd primal = Eigen::VectorXd::Zero(
    static_cast<Eigen::Index>(variable_count));
  for (std::size_t stage = 0U; stage < state_stage_count; ++stage) {
    const std::size_t source_stage = std::min(
      stage + resolution.stage_advance,
      previous_state_stage_count - 1U);
    primal.segment(
      static_cast<Eigen::Index>(stage * model::kStateDimension),
      model::kStateDimension) = previous.primal.segment(
      static_cast<Eigen::Index>(source_stage * model::kStateDimension),
      model::kStateDimension);
  }
  for (std::size_t stage = 0U; stage < horizon; ++stage) {
    const std::size_t source_stage = std::min(
      stage + resolution.stage_advance, previous_horizon - 1U);
    primal.segment(
      static_cast<Eigen::Index>(
        state_values + stage * model::kInputDimension),
      model::kInputDimension) = previous.primal.segment(
      static_cast<Eigen::Index>(
        previous_state_values + source_stage * model::kInputDimension),
      model::kInputDimension);
  }
  const double progress_rebase_m = previous.course_progress_origin_m -
    current.course_progress_origin_m;
  if (!std::isfinite(progress_rebase_m)) {
    resolution.reason = RecedingWarmStartReason::InvalidCurrent;
    return resolution;
  }
  for (std::size_t stage = 0U; stage < state_stage_count; ++stage) {
    primal[static_cast<Eigen::Index>(
      stage * model::kStateDimension + model::kProgressIndex)] +=
      progress_rebase_m;
  }
  primal.head<model::kStateDimension>() = current_problem.initial_state;
  if (!primal.allFinite()) {
    resolution.reason = RecedingWarmStartReason::InvalidPrevious;
    return resolution;
  }
  resolution.reason = RecedingWarmStartReason::Available;
  resolution.diagnostic = "available";
  resolution.warm_start = persistent_osqp::WarmStart{
    std::move(primal),
    Eigen::VectorXd::Zero(
      static_cast<Eigen::Index>(current_constraint_count))};
  return resolution;
}

std::optional<persistent_osqp::WarmStart> build_current_problem_bootstrap(
  const mpcc_rate_resolved_problem::AssemblyRequest & current_problem,
  const std::size_t current_constraint_count,
  const Eigen::VectorXd * preceding_same_problem_primal) noexcept
{
  namespace model = mpcc_rate_resolved;
  const int horizon = current_problem.horizon_steps;
  if (horizon <= 0 || current_constraint_count == 0U) {
    return std::nullopt;
  }
  const int state_values = model::kStateDimension * (horizon + 1);
  const int input_values = model::kInputDimension * horizon;
  const int variable_count = state_values + input_values;
  if (
    current_problem.initial_state.size() != model::kStateDimension ||
    current_problem.state_lower.size() != state_values ||
    current_problem.state_upper.size() != state_values ||
    current_problem.input_reference.size() != input_values ||
    current_problem.input_lower.size() != input_values ||
    current_problem.input_upper.size() != input_values ||
    current_problem.linearizations.size() !=
    static_cast<std::size_t>(horizon) ||
    !current_problem.initial_state.allFinite() ||
    !current_problem.input_reference.allFinite() ||
    !current_problem.input_lower.allFinite() ||
    !current_problem.input_upper.allFinite())
  {
    return std::nullopt;
  }
  const bool use_preceding_inputs =
    preceding_same_problem_primal != nullptr &&
    preceding_same_problem_primal->size() == variable_count &&
    preceding_same_problem_primal->allFinite();

  Eigen::VectorXd primal = Eigen::VectorXd::Zero(variable_count);
  primal.head<model::kStateDimension>() = current_problem.initial_state;
  for (int stage = 0; stage < horizon; ++stage) {
    const int state_offset = stage * model::kStateDimension;
    const int next_state_offset = (stage + 1) * model::kStateDimension;
    const int input_offset = stage * model::kInputDimension;
    const int primal_input_offset = state_values + input_offset;
    Eigen::Matrix<double, model::kInputDimension, 1> input =
      use_preceding_inputs ?
      preceding_same_problem_primal->segment<model::kInputDimension>(
      primal_input_offset) :
      current_problem.input_reference.segment<model::kInputDimension>(
      input_offset);
    for (int element = 0; element < model::kInputDimension; ++element) {
      const double lower = current_problem.input_lower[input_offset + element];
      const double upper = current_problem.input_upper[input_offset + element];
      if (!std::isfinite(lower) || !std::isfinite(upper) || lower > upper) {
        return std::nullopt;
      }
      input[element] = std::clamp(input[element], lower, upper);
    }
    primal.segment<model::kInputDimension>(primal_input_offset) = input;

    const auto & linearization =
      current_problem.linearizations[static_cast<std::size_t>(stage)];
    if (
      !linearization.state_matrix.allFinite() ||
      !linearization.input_matrix.allFinite() ||
      !linearization.equality_offset.allFinite())
    {
      return std::nullopt;
    }
    // Assembly owns the affine equality
    //   -x[k+1] + A*x[k] + B*u[k] = equality_offset.
    // Roll it out exactly so neither a fresh tactical candidate nor an SQP
    // correction begins from state values belonging to superseded equality
    // rows.  A same-problem input prefix is safe to transport because its
    // inputs are re-projected and every state/dual is rebuilt here.
    primal.segment<model::kStateDimension>(next_state_offset) =
      linearization.state_matrix *
      primal.segment<model::kStateDimension>(state_offset) +
      linearization.input_matrix * input - linearization.equality_offset;
  }
  if (!primal.allFinite()) {
    return std::nullopt;
  }
  return persistent_osqp::WarmStart{
    std::move(primal),
    Eigen::VectorXd::Zero(
      static_cast<Eigen::Index>(current_constraint_count))};
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
      const bool sampling_state_valid = result.actuation_sampled ?
        result.actuation_sample_reason ==
        mpcc_rate_resolved::ActuationSampleReason::Accepted :
        result.actuation_sample_reason ==
        mpcc_rate_resolved::ActuationSampleReason::Count;
      return !result.solved && sampling_state_valid &&
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
         result.planning_stage_count >= result.certified_stage_count &&
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
         (!result.dynamic_obstacle_refinement_requested ||
         (result.dynamic_obstacle_refinement_applied &&
         result.dynamic_obstacle_refinement_solved &&
         result.dynamic_obstacle_refinement_reason ==
         mpcc_rate_resolved_dynamic_obstacle::Reason::Applied)) &&
         result.successive_linearization_requested &&
         result.successive_linearization_applied &&
         result.successive_linearization_bootstrap_applied &&
         result.successive_linearization_solved &&
         result.successive_linearization_reason ==
         mpcc_rate_resolved_adapter::RelinearizationReason::Accepted &&
         result.post_refinement_physical_proof_checked ==
         (result.progress_wall_refinement_solved ||
         result.dynamic_obstacle_refinement_solved) &&
         (!result.post_refinement_physical_proof_checked ||
         result.post_refinement_physical_proof_accepted) &&
         (!result.post_refinement_linearization_requested ||
         (result.post_refinement_linearization_applied &&
         result.post_refinement_linearization_bootstrap_applied &&
         result.post_refinement_linearization_solved &&
         result.post_refinement_linearization_count > 0U &&
         result.post_refinement_linearization_reason ==
         mpcc_rate_resolved_adapter::RelinearizationReason::Accepted)) &&
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
  const auto capture_failure = [&result, &snapshot](
      const mpcc_architecture_snapshot::PipelineStage pipeline_stage,
      const mpcc_rate_resolved_problem::AssemblyRequest & request,
      const mpcc_rate_resolved_problem::Problem & problem,
      const std::optional<persistent_osqp::WarmStart> & warm_start,
      const persistent_osqp::SolveOutcome & outcome,
      const std::string & failure_outcome) {
      const auto recorded = mpcc_architecture_snapshot::record_failure(
        snapshot, request, problem, warm_start, outcome, pipeline_stage,
        failure_outcome, result.detail);
      if (recorded.status ==
        mpcc_architecture_snapshot::RecordStatus::Written)
      {
        result.detail += ", architecture_snapshot=" +
          recorded.snapshot_file.generic_string();
      } else if (recorded.status ==
        mpcc_architecture_snapshot::RecordStatus::IoFailure)
      {
        result.detail += ", architecture_snapshot_error=" + recorded.detail;
      }
    };
  const auto finish = [&result, &snapshot, &started]() {
      result.compute_ms = std::chrono::duration<double, std::milli>(
        SteadyClock::now() - started).count();
      result.completed_sec = snapshot.identity.snapshot_sec +
        result.compute_ms * 1.0e-3;
      return result;
    };
  if (
    !artifact::identity_valid(snapshot.identity) ||
    !std::isfinite(snapshot.control_prediction_origin_sec) ||
    snapshot.control_prediction_origin_sec < snapshot.identity.snapshot_sec ||
    !std::isfinite(snapshot.course_progress_origin_m) ||
    snapshot.request.horizon_steps <= 0 ||
    snapshot.execution_prefix_steps <= 0 ||
    snapshot.execution_prefix_steps > snapshot.request.horizon_steps ||
    snapshot.nominal_path_distance_m.size() !=
    static_cast<std::size_t>(snapshot.request.horizon_steps) + 1U ||
    !std::isfinite(snapshot.publication_interval_sec) ||
    snapshot.publication_interval_sec <= 0.0)
  {
    result.detail = "invalid shadow snapshot identity/timing";
    return finish();
  }

  mpcc_rate_resolved_adapter::BuildDiagnostic adapter_diagnostic;
  auto adapted = mpcc_rate_resolved_adapter::build(
    snapshot.request, solver_.physical_constraint_tolerance(),
    &adapter_diagnostic);
  if (!adapted.has_value()) {
    std::ostringstream detail;
    detail << "rate-resolved semantic adapter rejected snapshot: reason="
           << mpcc_rate_resolved_adapter::to_string(adapter_diagnostic.reason)
           << ", stage=" << adapter_diagnostic.stage
           << ", element=" << adapter_diagnostic.element
           << ", value=" << adapter_diagnostic.value
           << ", bounds=[" << adapter_diagnostic.lower << ','
           << adapter_diagnostic.upper << ']';
    result.detail = detail.str();
    return finish();
  }
  result.adapter_built = true;
  if (snapshot.progress_aligned_wall_refinement_active) {
    result.progress_wall_refinement_requested = true;
    const auto lateral_support =
      mpcc_rate_resolved_wall_refinement::
      resolve_pre_refinement_lateral_support(
      mpcc_rate_resolved_wall_refinement::
      PreRefinementLateralSupportRequest{
        true,
        adapted->problem.initial_state[
          mpcc_rate_resolved::kLateralIndex],
        snapshot.wall_lower_m,
        snapshot.wall_upper_m});
    result.pre_refinement_lateral_support_applied = lateral_support.applied;
    result.pre_refinement_lateral_support_reason = lateral_support.reason;
    result.pre_refinement_lateral_support_lower_m = lateral_support.lower_m;
    result.pre_refinement_lateral_support_upper_m = lateral_support.upper_m;
    // The wall profile is indexed by optimized progress. Using its stage-i
    // interval as a state-i box before solving progress creates an artificial
    // instantaneous lateral jump when the course corridor moves between
    // progress samples. Keep only the profile union in the first solve. The
    // progress-aligned/physical refinement below installs the executable hard
    // rows around the solved progress before any artifact can be published.
    if (lateral_support.valid && lateral_support.applied) {
      const int horizon_steps = snapshot.request.horizon_steps;
      for (int stage = 1; stage <= horizon_steps; ++stage) {
        const int state = stage * mpcc_rate_resolved::kStateDimension;
        adapted->problem.state_lower[
          state + mpcc_rate_resolved::kLateralIndex] =
          lateral_support.lower_m;
        adapted->problem.state_upper[
          state + mpcc_rate_resolved::kLateralIndex] =
          lateral_support.upper_m;
      }
    }
    const auto horizon = static_cast<std::size_t>(
      snapshot.request.horizon_steps);
    mpcc_rate_resolved_problem::ProgressAlignedWallConstraints inactive_wall;
    inactive_wall.lower_slope.assign(horizon, 0.0);
    inactive_wall.lower_intercept.assign(
      horizon, -std::numeric_limits<double>::infinity());
    inactive_wall.upper_slope.assign(horizon, 0.0);
    inactive_wall.upper_intercept.assign(
      horizon, std::numeric_limits<double>::infinity());
    adapted->problem.progress_aligned_wall_constraints =
      std::move(inactive_wall);
    if (snapshot.physical_wall_refinement_active) {
      constexpr std::size_t kInteriorSamplesPerTransition = 4U;
      adapted->problem.swept_lateral_wall_constraints.reserve(
        horizon * kInteriorSamplesPerTransition);
      for (std::size_t stage = 0U; stage < horizon; ++stage) {
        for (std::size_t sample = 1U;
          sample <= kInteriorSamplesPerTransition; ++sample)
        {
          adapted->problem.swept_lateral_wall_constraints.push_back(
            mpcc_rate_resolved_problem::SweptLateralWallConstraint{
              static_cast<int>(stage),
              static_cast<double>(sample) /
              static_cast<double>(kInteriorSamplesPerTransition + 1U),
              -std::numeric_limits<double>::infinity(),
              std::numeric_limits<double>::infinity()});
        }
      }
    }
  }

  auto assembled =
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
  result.first_virtual_progress_feasibility =
    mpcc_rate_resolved_problem::analyze_first_stage_input_feasibility(
    adapted->problem,
    mpcc_rate_resolved::kVirtualProgressSpeedIndex);
  result.solve_attempted = true;

  std::lock_guard<std::mutex> lock(mutex_);
  RecedingWarmStartResolution warm_start_resolution;
  if (warm_start_seed_.has_value()) {
    warm_start_resolution = resolve_receding_warm_start(
      warm_start_seed_.value(), snapshot, adapted->problem,
      static_cast<std::size_t>(assembled->lower_bound.size()));
  }
  if (!warm_start_resolution.warm_start.has_value()) {
    const auto previous_reason = warm_start_resolution.reason;
    const auto previous_diagnostic = warm_start_resolution.diagnostic;
    auto bootstrap = build_current_problem_bootstrap(
      adapted->problem,
      static_cast<std::size_t>(assembled->lower_bound.size()));
    if (bootstrap.has_value()) {
      warm_start_resolution.reason =
        RecedingWarmStartReason::CurrentProblemBootstrap;
      warm_start_resolution.diagnostic =
        std::string{"current-problem-bootstrap/previous="} +
        to_string(previous_reason);
      if (
        !previous_diagnostic.empty() &&
        previous_diagnostic != to_string(previous_reason))
      {
        warm_start_resolution.diagnostic += ':' + previous_diagnostic;
      }
      warm_start_resolution.stage_advance = 0U;
      warm_start_resolution.warm_start = std::move(bootstrap);
    }
  }
  result.receding_warm_start_reason = warm_start_resolution.reason;
  result.receding_warm_start_diagnostic = warm_start_resolution.diagnostic;
  result.receding_warm_start_stage_advance =
    warm_start_resolution.stage_advance;
  auto outcome = solver_.solve(
    assembled->quadratic_cost, assembled->constraints,
    assembled->linear_cost, assembled->lower_bound, assembled->upper_bound,
    warm_start_resolution.warm_start, assembled->variable_scaling);
  result.receding_warm_start_applied =
    outcome.telemetry.warm_start_applied;
  result.solver = outcome.telemetry;
  if (!outcome.result.has_value()) {
    result.outcome = Outcome::SolveRejected;
    result.detail = outcome.failure_detail;
    if (outcome.constraint_failure.has_value()) {
      const auto semantic = mpcc_rate_resolved_problem::decode_row(
        outcome.constraint_failure->row, snapshot.request.horizon_steps,
        adapted->problem.steering_rate_prefix_bounds.has_value(),
        adapted->problem.progress_aligned_wall_constraints.has_value(),
        static_cast<int>(
          adapted->problem.swept_lateral_wall_constraints.size()));
      std::ostringstream detail;
      detail << result.detail;
      if (semantic.valid) {
        detail << ", row_semantic=" <<
          mpcc_rate_resolved_problem::row_kind_name(semantic.kind) <<
          "/stage=" << semantic.stage << "/element=" << semantic.element;
      }
      const auto & feasibility = result.first_virtual_progress_feasibility;
      if (feasibility.evaluated) {
        detail << ", first_vtheta=" <<
          (feasibility.separable ? "separable" : "coupled") << '/' <<
          (feasibility.conclusive ?
          (feasibility.feasible ? "feasible" : "empty") :
          "inconclusive") <<
          "/declared:[" << feasibility.declared_lower << ',' <<
          feasibility.declared_upper << "]/implied:[" <<
          feasibility.implied_lower << ',' << feasibility.implied_upper <<
          "]/limit_state:[" << feasibility.limiting_lower_state_element <<
          ',' << feasibility.limiting_upper_state_element << ']';
      }
      result.detail = detail.str();
    }
    capture_failure(
      mpcc_architecture_snapshot::PipelineStage::Initial,
      adapted->problem, assembled.value(),
      warm_start_resolution.warm_start, outcome, "solve-rejected");
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
  // The first QP is deliberately broad in progress-aligned wall space.  Use
  // that unconstricted solution to move the affine dynamics tangent onto the
  // current nonlinear trajectory before building any progress/lag/heading
  // trust bucket.  Reversing this order makes the wall proof own a narrow box
  // around a trajectory that does not satisfy the new affine equalities: the
  // relinearized QP can then be structurally infeasible even though both the
  // provisional trajectory and the wall corridor are individually valid.
  //
  // This is the pre-refinement canonical SQP correction.  Wall and dynamic-
  // obstacle refinements below constrain its solution.  Those refinements can
  // move the primal away from this tangent, so a second current-problem-owned
  // correction is conditionally required after the last physical refinement
  // before exact nonlinear replay may certify the artifact.
  result.successive_linearization_requested = true;
  const auto relinearization =
    mpcc_rate_resolved_adapter::relinearize_around_primal(
    snapshot.request, outcome.result->primal, adapted->problem);
  result.successive_linearization_reason = relinearization.reason;
  result.successive_linearization_failure_stage = relinearization.stage;
  result.successive_linearization_applied = relinearization.applied;
  if (!relinearization.applied) {
    result.outcome = Outcome::AssemblyRejected;
    result.solved = false;
    result.detail = std::string{"rate-resolved successive linearization rejected: "} +
      mpcc_rate_resolved_adapter::to_string(relinearization.reason) +
      "/stage=" + std::to_string(relinearization.stage);
    return finish();
  }
  auto relinearized_assembled =
    mpcc_rate_resolved_problem::assemble(adapted->problem);
  if (!relinearized_assembled.has_value()) {
    result.outcome = Outcome::AssemblyRejected;
    result.solved = false;
    result.detail = "rate-resolved relinearized QP assembly rejected";
    return finish();
  }
  // The provisional primal and dual belong to the preceding affine equality
  // system.  Reusing them after replacing every temporal dynamics row gives
  // OSQP a warm start with stale equality provenance.  Roll out a seed owned
  // by the relinearized problem itself.
  auto relinearized_warm_start = build_current_problem_bootstrap(
    adapted->problem,
    static_cast<std::size_t>(relinearized_assembled->lower_bound.size()),
    &outcome.result->primal);
  if (!relinearized_warm_start.has_value()) {
    result.outcome = Outcome::AssemblyRejected;
    result.solved = false;
    result.detail =
      "rate-resolved relinearized current-problem bootstrap rejected";
    return finish();
  }
  result.successive_linearization_bootstrap_applied = true;
  auto relinearized_outcome = solver_.solve(
    relinearized_assembled->quadratic_cost,
    relinearized_assembled->constraints,
    relinearized_assembled->linear_cost,
    relinearized_assembled->lower_bound,
    relinearized_assembled->upper_bound,
    relinearized_warm_start,
    relinearized_assembled->variable_scaling);
  result.solver = relinearized_outcome.telemetry;
  if (!relinearized_outcome.result.has_value()) {
    result.outcome = Outcome::SolveRejected;
    result.solved = false;
    result.detail = std::string{"rate-resolved relinearized QP rejected: "} +
      relinearized_outcome.failure_detail;
    if (relinearized_outcome.constraint_failure.has_value()) {
      const auto semantic = mpcc_rate_resolved_problem::decode_row(
        relinearized_outcome.constraint_failure->row,
        snapshot.request.horizon_steps,
        adapted->problem.steering_rate_prefix_bounds.has_value(),
        adapted->problem.progress_aligned_wall_constraints.has_value(),
        static_cast<int>(
          adapted->problem.swept_lateral_wall_constraints.size()),
        &adapted->problem.dynamic_obstacle_constraints);
      if (semantic.valid) {
        result.detail += std::string{", row_semantic="} +
          mpcc_rate_resolved_problem::row_kind_name(semantic.kind) +
          "/stage=" + std::to_string(semantic.stage) +
          "/element=" + std::to_string(semantic.element);
      }
    }
    capture_failure(
      mpcc_architecture_snapshot::PipelineStage::SuccessiveLinearization,
      adapted->problem, relinearized_assembled.value(),
      relinearized_warm_start, relinearized_outcome, "solve-rejected");
    return finish();
  }
  if (!relinearized_outcome.result->primal.allFinite()) {
    result.outcome = Outcome::NonfiniteResult;
    result.solved = false;
    result.finite = false;
    result.detail =
      "rate-resolved relinearized solver returned non-finite primal";
    return finish();
  }
  assembled = std::move(relinearized_assembled);
  outcome = std::move(relinearized_outcome);
  result.successive_linearization_solved = true;

  result.progress_wall_refinement_requested =
    snapshot.progress_aligned_wall_refinement_active;
  result.physical_wall_refinement_requested =
    snapshot.physical_wall_refinement_active;
  std::optional<persistent_osqp::WarmStart> last_refinement_warm_start;
  if (snapshot.progress_aligned_wall_refinement_active) {
    auto refinement = build_progress_wall_refinement(
      snapshot, adapted->problem, outcome.result->primal,
      solver_.physical_constraint_tolerance());
    result.progress_wall_refinement_reason = refinement.resolution.reason;
    result.progress_wall_refinement_aligned_stage_count =
      refinement.resolution.aligned_stage_count;
    result.progress_wall_refinement_out_of_range_stage_count =
      refinement.resolution.out_of_range_stage_count;
    result.progress_wall_refinement_first_failure_stage =
      refinement.resolution.first_failure_stage;
    result.progress_wall_refinement_maximum_mismatch_m =
      refinement.resolution.maximum_progress_mismatch_m;
    result.progress_wall_refinement_applied = refinement.request.has_value();
    result.physical_wall_refinement_reason = refinement.physical.reason;
    result.physical_wall_refinement_applied = refinement.physical.applied;
    result.physical_wall_refinement_first_failure_stage =
      refinement.physical.first_failure_stage;
    result.physical_wall_refinement_checked_pose_count =
      refinement.physical.checked_pose_count;
    if (!refinement.request.has_value()) {
      result.outcome = Outcome::AssemblyRejected;
      result.solved = false;
      std::ostringstream detail;
      detail << "rate-resolved wall refinement rejected: progress="
             << mpcc_progress::progress_aligned_wall_bounds_reason_name(
        result.progress_wall_refinement_reason)
             << "/stage=" <<
        result.progress_wall_refinement_first_failure_stage
             << "/profile=" << snapshot.progress_wall_profile_diagnostic;
      if (snapshot.physical_wall_refinement_active) {
        detail << ", physical=" <<
          mpcc_rate_resolved_wall_refinement::to_string(
          result.physical_wall_refinement_reason)
               << "/stage=" <<
          result.physical_wall_refinement_first_failure_stage;
      }
      result.detail = detail.str();
      return finish();
    }
    auto refined_assembled =
      mpcc_rate_resolved_problem::assemble(refinement.request.value());
    if (!refined_assembled.has_value()) {
      result.outcome = Outcome::AssemblyRejected;
      result.solved = false;
      result.detail = "rate-resolved wall-refined QP assembly rejected";
      return finish();
    }
    std::optional<persistent_osqp::WarmStart> warm_start{
      persistent_osqp::WarmStart{
        outcome.result->primal, outcome.result->dual}};
    auto refined_outcome = solver_.solve(
      refined_assembled->quadratic_cost, refined_assembled->constraints,
      refined_assembled->linear_cost, refined_assembled->lower_bound,
      refined_assembled->upper_bound, warm_start,
      refined_assembled->variable_scaling);
    result.solver = refined_outcome.telemetry;
    if (!refined_outcome.result.has_value()) {
      result.outcome = Outcome::SolveRejected;
      result.solved = false;
      result.detail = std::string{"rate-resolved wall-refined QP rejected: "} +
        refined_outcome.failure_detail;
      capture_failure(
        mpcc_architecture_snapshot::PipelineStage::WallRefinement,
        refinement.request.value(), refined_assembled.value(), warm_start,
        refined_outcome, "solve-rejected");
      return finish();
    }
    if (!refined_outcome.result->primal.allFinite()) {
      result.outcome = Outcome::NonfiniteResult;
      result.solved = false;
      result.finite = false;
      result.detail =
        "rate-resolved wall-refined solver returned non-finite primal";
      return finish();
    }
    adapted->problem = std::move(refinement.request.value());
    assembled = std::move(refined_assembled);
    outcome = std::move(refined_outcome);
    last_refinement_warm_start = std::move(warm_start);
    result.progress_wall_refinement_solved = true;
    result.physical_wall_refinement_solved =
      !snapshot.physical_wall_refinement_active ||
      result.physical_wall_refinement_applied;
  }
  result.dynamic_obstacle_refinement_requested =
    snapshot.dynamic_obstacle_refinement_active;
  if (snapshot.dynamic_obstacle_refinement_active) {
    const auto refinement =
      mpcc_rate_resolved_dynamic_obstacle::refine(
      mpcc_rate_resolved_dynamic_obstacle::Request{
        true,
        snapshot.dynamic_obstacle_pass_side_sign,
        snapshot.dynamic_obstacle_stages,
        adapted->problem,
        outcome.result->primal,
        solver_.physical_constraint_tolerance().absolute});
    result.dynamic_obstacle_refinement_reason = refinement.reason;
    result.dynamic_obstacle_refinement_applied = refinement.applied;
    result.dynamic_obstacle_resolved_side_sign =
      refinement.resolved_side_sign;
    result.dynamic_obstacle_first_pass_side_stage =
      refinement.first_pass_side_stage;
    result.dynamic_obstacle_stay_behind_row_count =
      refinement.stay_behind_row_count;
    result.dynamic_obstacle_pass_side_row_count =
      refinement.pass_side_row_count;
    result.dynamic_obstacle_partial_escape_row_count =
      refinement.partial_escape_row_count;
    result.dynamic_obstacle_first_valid_stage =
      refinement.first_valid_stage;
    result.dynamic_obstacle_first_wall_only_progress_m =
      refinement.first_wall_only_progress_m;
    result.dynamic_obstacle_first_wall_only_effective_progress_m =
      refinement.first_wall_only_effective_progress_m;
    result.dynamic_obstacle_first_wall_only_lateral_m =
      refinement.first_wall_only_lateral_m;
    result.dynamic_obstacle_first_target_progress_m =
      refinement.first_target_progress_m;
    result.dynamic_obstacle_first_target_lateral_m =
      refinement.first_target_lateral_m;
    result.dynamic_obstacle_first_stay_behind_margin_m =
      refinement.first_stay_behind_margin_m;
    result.dynamic_obstacle_first_positive_side_margin_m =
      refinement.first_positive_side_margin_m;
    result.dynamic_obstacle_first_negative_side_margin_m =
      refinement.first_negative_side_margin_m;
    if (!refinement.problem.has_value()) {
      result.outcome = Outcome::AssemblyRejected;
      result.solved = false;
      result.detail = std::string{"rate-resolved dynamic-obstacle refinement rejected: "} +
        mpcc_rate_resolved_dynamic_obstacle::to_string(refinement.reason);
      return finish();
    }
    auto refined_assembled =
      mpcc_rate_resolved_problem::assemble(refinement.problem.value());
    if (!refined_assembled.has_value()) {
      result.outcome = Outcome::AssemblyRejected;
      result.solved = false;
      result.detail =
        "rate-resolved dynamic-obstacle QP assembly rejected";
      return finish();
    }
    std::optional<persistent_osqp::WarmStart> warm_start{
      persistent_osqp::WarmStart{
        outcome.result->primal, outcome.result->dual}};
    auto refined_outcome = solver_.solve(
      refined_assembled->quadratic_cost, refined_assembled->constraints,
      refined_assembled->linear_cost, refined_assembled->lower_bound,
      refined_assembled->upper_bound, warm_start,
      refined_assembled->variable_scaling);
    result.solver = refined_outcome.telemetry;
    if (!refined_outcome.result.has_value()) {
      result.outcome = Outcome::SolveRejected;
      result.solved = false;
      result.detail = std::string{"rate-resolved dynamic-obstacle QP rejected: "} +
        refined_outcome.failure_detail;
      if (refined_outcome.constraint_failure.has_value()) {
        const auto semantic = mpcc_rate_resolved_problem::decode_row(
          refined_outcome.constraint_failure->row,
          snapshot.request.horizon_steps,
          refinement.problem->steering_rate_prefix_bounds.has_value(),
          refinement.problem->progress_aligned_wall_constraints.has_value(),
          static_cast<int>(
            refinement.problem->swept_lateral_wall_constraints.size()),
          &refinement.problem->dynamic_obstacle_constraints);
        if (semantic.valid) {
          result.detail += std::string{", row_semantic="} +
            mpcc_rate_resolved_problem::row_kind_name(semantic.kind) +
            "/stage=" + std::to_string(semantic.stage) +
            "/element=" + std::to_string(semantic.element);
        }
      }
      capture_failure(
        mpcc_architecture_snapshot::PipelineStage::
        DynamicObstacleRefinement,
        refinement.problem.value(), refined_assembled.value(), warm_start,
        refined_outcome, "solve-rejected");
      return finish();
    }
    if (!refined_outcome.result->primal.allFinite()) {
      result.outcome = Outcome::NonfiniteResult;
      result.solved = false;
      result.finite = false;
      result.detail =
        "rate-resolved dynamic-obstacle solver returned non-finite primal";
      return finish();
    }
    adapted->problem = std::move(refinement.problem.value());
    assembled = std::move(refined_assembled);
    outcome = std::move(refined_outcome);
    last_refinement_warm_start = std::move(warm_start);
    result.dynamic_obstacle_refinement_solved = true;
  }

  // Physical refinements can move the final primal away from the nonlinear
  // trajectory represented by the pre-refinement tangent.  First replay the
  // exact execution prefix: when it is already physically valid, another QP
  // solve would add latency and a new failure mode without improving proof.
  // Only a demonstrated model/proof gap requests a current-problem SQP
  // correction.  Every correction preserves the complete refined problem and
  // is rechecked by the same exact proof before artifact publication.
  const bool physical_problem_refined =
    result.progress_wall_refinement_solved ||
    result.dynamic_obstacle_refinement_solved;
  mpcc_rate_resolved_physical_adapter::Result post_refinement_proof;
  ExecutionArtifactBuildResult post_refinement_artifact_build;
  const auto evaluate_post_refinement_proof = [&]() {
      post_refinement_artifact_build = build_execution_artifact(
        snapshot, adapted->problem, outcome,
        snapshot.identity.snapshot_sec +
        std::chrono::duration<double>(
          SteadyClock::now() - started).count());
      if (!post_refinement_artifact_build.execution_artifact.has_value()) {
        return mpcc_rate_resolved_physical_adapter::Result{};
      }
      return mpcc_rate_resolved_physical_adapter::build(
        post_refinement_artifact_build.execution_artifact.value(),
        snapshot.identity.source_context.intent,
        snapshot.identity.source_context.stage_geometry_id);
    };
  if (physical_problem_refined) {
    result.post_refinement_physical_proof_checked = true;
    post_refinement_proof = evaluate_post_refinement_proof();
  }
  while (
    physical_problem_refined &&
    post_refinement_proof.reason ==
    mpcc_rate_resolved_physical_adapter::RejectReason::
    ExactTrajectoryRejected &&
    result.post_refinement_linearization_count <
    kMaximumPhysicalProofSqpCorrections)
  {
    result.post_refinement_linearization_requested = true;
    ++result.post_refinement_linearization_count;
    const auto post_refinement_linearization =
      mpcc_rate_resolved_adapter::relinearize_around_primal(
      snapshot.request, outcome.result->primal, adapted->problem);
    result.post_refinement_linearization_reason =
      post_refinement_linearization.reason;
    result.post_refinement_linearization_failure_stage =
      post_refinement_linearization.stage;
    result.post_refinement_linearization_applied =
      post_refinement_linearization.applied;
    if (!post_refinement_linearization.applied) {
      result.outcome = Outcome::AssemblyRejected;
      result.solved = false;
      result.detail =
        std::string{"rate-resolved post-refinement linearization rejected: "} +
        mpcc_rate_resolved_adapter::to_string(
        post_refinement_linearization.reason) +
        "/stage=" + std::to_string(post_refinement_linearization.stage);
      return finish();
    }

    auto post_refinement_assembled =
      mpcc_rate_resolved_problem::assemble(adapted->problem);
    if (!post_refinement_assembled.has_value()) {
      result.outcome = Outcome::AssemblyRejected;
      result.solved = false;
      result.detail =
        "rate-resolved post-refinement relinearized QP assembly rejected";
      return finish();
    }
    auto post_refinement_warm_start = build_current_problem_bootstrap(
      adapted->problem,
      static_cast<std::size_t>(
        post_refinement_assembled->lower_bound.size()),
      &outcome.result->primal);
    if (!post_refinement_warm_start.has_value()) {
      result.outcome = Outcome::AssemblyRejected;
      result.solved = false;
      result.detail =
        "rate-resolved post-refinement current-problem bootstrap rejected";
      return finish();
    }
    result.post_refinement_linearization_bootstrap_applied = true;
    auto post_refinement_outcome = solver_.solve(
      post_refinement_assembled->quadratic_cost,
      post_refinement_assembled->constraints,
      post_refinement_assembled->linear_cost,
      post_refinement_assembled->lower_bound,
      post_refinement_assembled->upper_bound,
      post_refinement_warm_start,
      post_refinement_assembled->variable_scaling);
    result.solver = post_refinement_outcome.telemetry;
    if (!post_refinement_outcome.result.has_value()) {
      result.outcome = Outcome::SolveRejected;
      result.solved = false;
      result.detail =
        std::string{"rate-resolved post-refinement relinearized QP rejected: "} +
        post_refinement_outcome.failure_detail;
      if (post_refinement_outcome.constraint_failure.has_value()) {
        const auto semantic = mpcc_rate_resolved_problem::decode_row(
          post_refinement_outcome.constraint_failure->row,
          snapshot.request.horizon_steps,
          adapted->problem.steering_rate_prefix_bounds.has_value(),
          adapted->problem.progress_aligned_wall_constraints.has_value(),
          static_cast<int>(
            adapted->problem.swept_lateral_wall_constraints.size()),
          &adapted->problem.dynamic_obstacle_constraints);
        if (semantic.valid) {
          result.detail += std::string{", row_semantic="} +
            mpcc_rate_resolved_problem::row_kind_name(semantic.kind) +
            "/stage=" + std::to_string(semantic.stage) +
            "/element=" + std::to_string(semantic.element);
        }
      }
      capture_failure(
        mpcc_architecture_snapshot::PipelineStage::
        PostRefinementLinearization,
        adapted->problem, post_refinement_assembled.value(),
        post_refinement_warm_start, post_refinement_outcome,
        "solve-rejected");
      return finish();
    }
    if (!post_refinement_outcome.result->primal.allFinite()) {
      result.outcome = Outcome::NonfiniteResult;
      result.solved = false;
      result.finite = false;
      result.detail =
        "rate-resolved post-refinement solver returned non-finite primal";
      return finish();
    }
    assembled = std::move(post_refinement_assembled);
    outcome = std::move(post_refinement_outcome);
    last_refinement_warm_start = std::move(post_refinement_warm_start);
    result.post_refinement_linearization_solved = true;
    post_refinement_proof = evaluate_post_refinement_proof();
  }
  if (physical_problem_refined) {
    if (!post_refinement_artifact_build.execution_artifact.has_value()) {
      result.outcome = Outcome::ArtifactRejected;
      result.solved = false;
      result.execution_artifact_reject_reason =
        post_refinement_artifact_build.reject_reason;
      std::ostringstream detail;
      detail << "rate-resolved post-refinement artifact construction rejected: "
             << post_refinement_artifact_build.detail << "/reason="
             << artifact::to_string(post_refinement_artifact_build.reject_reason);
      result.detail = detail.str();
      capture_failure(
        mpcc_architecture_snapshot::PipelineStage::PhysicalProof,
        adapted->problem, assembled.value(), last_refinement_warm_start,
        outcome, "artifact-construction-rejected");
      return finish();
    }
    result.post_refinement_physical_proof_accepted =
      post_refinement_proof.reason ==
      mpcc_rate_resolved_physical_adapter::RejectReason::None;
    if (!result.post_refinement_physical_proof_accepted) {
      result.outcome = Outcome::PhysicalProofRejected;
      result.solved = false;
      std::ostringstream detail;
      detail << "rate-resolved post-refinement physical proof rejected: "
             << mpcc_rate_resolved_physical_adapter::to_string(
        post_refinement_proof.reason)
             << "/exact=" <<
        race_mpcc_foundation::exact_physical_execution_trajectory_reason_name(
        post_refinement_proof.exact_reason)
             << "/stage=" << post_refinement_proof.rejected_stage
             << "/corrections=" <<
        result.post_refinement_linearization_count;
      result.detail = detail.str();
      capture_failure(
        mpcc_architecture_snapshot::PipelineStage::PhysicalProof,
        adapted->problem, assembled.value(), last_refinement_warm_start,
        outcome, "physical-proof-rejected");
      return finish();
    }
  }
  result.solver = outcome.telemetry;
  result.constraints_satisfied = true;
  result.maximum_constraint_violation =
    outcome.result->maximum_constraint_violation;
  result.maximum_normalized_constraint_violation =
    outcome.result->maximum_normalized_constraint_violation;
  result.maximum_normalized_constraint_row =
    outcome.result->maximum_normalized_constraint_row;

  namespace model = mpcc_rate_resolved;
  const int horizon = snapshot.request.horizon_steps;
  const int execution_horizon = snapshot.execution_prefix_steps;
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
  result.planning_stage_count = static_cast<std::size_t>(horizon);
  const int terminal_state = model::kStateDimension * horizon;
  result.terminal_velocity_mps =
    primal[terminal_state + model::kVelocityIndex];
  result.terminal_progress_m =
    primal[terminal_state + model::kProgressIndex];
  result.terminal_steering_rad =
    primal[terminal_state + model::kSteeringIndex];
  std::vector<double> certified_steering_rates_radps;
  std::vector<double> stage_durations_sec;
  certified_steering_rates_radps.reserve(
    static_cast<std::size_t>(execution_horizon));
  stage_durations_sec.reserve(static_cast<std::size_t>(execution_horizon));
  for (int stage = 0; stage < execution_horizon; ++stage) {
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

  auto execution_artifact_build = build_execution_artifact(
    snapshot, adapted->problem, outcome,
    snapshot.identity.snapshot_sec +
    std::chrono::duration<double>(SteadyClock::now() - started).count());
  if (!execution_artifact_build.execution_artifact.has_value()) {
    result.outcome = Outcome::ArtifactRejected;
    result.execution_artifact_reject_reason =
      execution_artifact_build.reject_reason;
    std::ostringstream detail;
    detail << "execution artifact construction rejected: "
           << execution_artifact_build.detail << "/reason="
           << artifact::to_string(execution_artifact_build.reject_reason);
    result.detail = detail.str();
    result.solved = false;
    result.constraints_satisfied = false;
    return finish();
  }
  auto execution_artifact = std::move(execution_artifact_build.execution_artifact);
  result.execution_artifact_reject_reason =
    artifact::validate(execution_artifact.value());
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
    std::move(execution_artifact.value()));
  RecedingWarmStartSeed next_warm_start;
  next_warm_start.identity = snapshot.identity;
  next_warm_start.control_prediction_origin_sec =
    snapshot.control_prediction_origin_sec;
  next_warm_start.course_progress_origin_m =
    snapshot.course_progress_origin_m;
  next_warm_start.stage_durations_sec.reserve(
    snapshot.request.inputs.size());
  for (const auto & input : snapshot.request.inputs) {
    next_warm_start.stage_durations_sec.push_back(input.stage_dt_sec);
  }
  next_warm_start.primal = outcome.result->primal;
  warm_start_seed_ = std::move(next_warm_start);
  result.outcome = Outcome::Solved;
  if (result.progress_wall_refinement_requested) {
    std::ostringstream detail;
    detail << "accepted, progress_wall="
           << mpcc_progress::progress_aligned_wall_bounds_reason_name(
      result.progress_wall_refinement_reason)
           << "/applied=" <<
      (result.progress_wall_refinement_applied ? 1 : 0)
           << "/solved=" <<
      (result.progress_wall_refinement_solved ? 1 : 0)
           << "/mismatch=" <<
      result.progress_wall_refinement_maximum_mismatch_m << "m";
    if (result.physical_wall_refinement_requested) {
      detail << ", physical_wall=" <<
        mpcc_rate_resolved_wall_refinement::to_string(
        result.physical_wall_refinement_reason)
             << "/applied=" <<
        (result.physical_wall_refinement_applied ? 1 : 0)
             << "/solved=" <<
        (result.physical_wall_refinement_solved ? 1 : 0)
             << "/failure_stage=" <<
        result.physical_wall_refinement_first_failure_stage
             << "/samples=" <<
        result.physical_wall_refinement_checked_pose_count;
    }
    detail << ", successive_linearization="
           << mpcc_rate_resolved_adapter::to_string(
      result.successive_linearization_reason)
           << "/applied=" <<
      (result.successive_linearization_applied ? 1 : 0)
           << "/bootstrap=" <<
      (result.successive_linearization_bootstrap_applied ? 1 : 0)
           << "/solved=" <<
      (result.successive_linearization_solved ? 1 : 0);
    detail << ", post_refinement_linearization="
           << mpcc_rate_resolved_adapter::to_string(
      result.post_refinement_linearization_reason)
           << "/requested=" <<
      (result.post_refinement_linearization_requested ? 1 : 0)
           << "/applied=" <<
      (result.post_refinement_linearization_applied ? 1 : 0)
           << "/bootstrap=" <<
      (result.post_refinement_linearization_bootstrap_applied ? 1 : 0)
           << "/solved=" <<
      (result.post_refinement_linearization_solved ? 1 : 0)
           << "/count=" << result.post_refinement_linearization_count
           << "/proof=" <<
      (result.post_refinement_physical_proof_checked ? 1 : 0) << '/'
           << (result.post_refinement_physical_proof_accepted ? 1 : 0);
    result.detail = detail.str();
  } else {
    result.detail = std::string{"accepted, progress_wall=not-requested/"} +
      snapshot.progress_wall_profile_diagnostic +
      ", successive_linearization=" +
      mpcc_rate_resolved_adapter::to_string(
      result.successive_linearization_reason) +
      "/applied=" +
      (result.successive_linearization_applied ? "1" : "0") +
      "/bootstrap=" +
      (result.successive_linearization_bootstrap_applied ? "1" : "0") +
      "/solved=" +
      (result.successive_linearization_solved ? "1" : "0") +
      ", post_refinement_linearization=" +
      mpcc_rate_resolved_adapter::to_string(
      result.post_refinement_linearization_reason) +
      "/requested=" +
      (result.post_refinement_linearization_requested ? "1" : "0") +
      "/applied=" +
      (result.post_refinement_linearization_applied ? "1" : "0") +
      "/bootstrap=" +
      (result.post_refinement_linearization_bootstrap_applied ? "1" : "0") +
      "/solved=" +
      (result.post_refinement_linearization_solved ? "1" : "0") +
      "/count=" +
      std::to_string(result.post_refinement_linearization_count) +
      "/proof=" +
      (result.post_refinement_physical_proof_checked ? "1" : "0") + "/" +
      (result.post_refinement_physical_proof_accepted ? "1" : "0");
  }
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
