#include "multi_purpose_mpc_ros/mpcc_rate_resolved_shadow.hpp"

#include "multi_purpose_mpc_ros/mpcc_architecture_snapshot.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_physical_adapter.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_problem.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_wall_refinement.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
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
// Architecture-audit budget only. These iterates cannot publish and exist to
// distinguish a one-shot wall tangent defect from physical infeasibility.
constexpr std::size_t kMaximumWallRestorationSqpCorrections = 3U;

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
  physical_constraint_tolerance,
  mpcc_rate_resolved_wall_refinement::Cache * wall_refinement_cache,
  const std::optional<SolverContext::WallBucketAuditMode> &
  wall_bucket_audit_mode = std::nullopt) noexcept
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
  physical_request.wall_grid_fingerprint =
    snapshot.replay_world.has_value() ?
    snapshot.replay_world->wall_grid_fingerprint : 0U;
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
    mpcc_rate_resolved_wall_refinement::resolve(
    physical_request, wall_refinement_cache);
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
      // The lateral/progress rows describe the convex wall corridor.  Lag
      // and heading intervals only describe the sampled pose bucket used to
      // construct that approximation; making both hard in production can
      // empty the affine subproblem even when the immutable-world exact
      // proof accepts a trajectory.  They remain available solely for the
      // historical offline A/B arms below.  Production acceptance is owned
      // by the exact nonlinear trajectory and swept physical-wall proof.
      if (
        wall_bucket_audit_mode.has_value() &&
        wall_bucket_audit_mode !=
        SolverContext::WallBucketAuditMode::OmitLag &&
        wall_bucket_audit_mode !=
        SolverContext::WallBucketAuditMode::OmitPose &&
        wall_bucket_audit_mode !=
        SolverContext::WallBucketAuditMode::OmitPoseDirect)
      {
        refined.state_lower[state + model::kLagIndex] = bounds.lag_lower_m;
        refined.state_upper[state + model::kLagIndex] = bounds.lag_upper_m;
      }
      if (
        wall_bucket_audit_mode.has_value() &&
        wall_bucket_audit_mode !=
        SolverContext::WallBucketAuditMode::OmitHeading &&
        wall_bucket_audit_mode !=
        SolverContext::WallBucketAuditMode::OmitPose &&
        wall_bucket_audit_mode !=
        SolverContext::WallBucketAuditMode::OmitPoseDirect)
      {
        refined.state_lower[state + model::kHeadingIndex] =
          bounds.heading_lower_rad;
        refined.state_upper[state + model::kHeadingIndex] =
          bounds.heading_upper_rad;
      }
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

const char * to_string(const LatestStateFeedbackReason reason) noexcept
{
  switch (reason) {
    case LatestStateFeedbackReason::InvalidRequest:
      return "invalid-request";
    case LatestStateFeedbackReason::AssemblyRejected:
      return "assembly-rejected";
    case LatestStateFeedbackReason::BootstrapRejected:
      return "bootstrap-rejected";
    case LatestStateFeedbackReason::SolveRejected:
      return "solve-rejected";
    case LatestStateFeedbackReason::ArtifactRejected:
      return "artifact-rejected";
    case LatestStateFeedbackReason::PhysicalAdapterRejected:
      return "physical-adapter-rejected";
    case LatestStateFeedbackReason::Accepted:
      return "accepted";
    case LatestStateFeedbackReason::Exception:
      return "exception";
    case LatestStateFeedbackReason::Count:
      return "count";
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
    previous_context.execution_side_sign != current_context.execution_side_sign ||
    previous_context.dynamic_obstacle_constraint_active !=
    current_context.dynamic_obstacle_constraint_active ||
    previous_context.dynamic_obstacle_id !=
    current_context.dynamic_obstacle_id ||
    previous_context.dynamic_obstacle_side_sign !=
    current_context.dynamic_obstacle_side_sign)
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
    } else if (
      previous_context.dynamic_obstacle_constraint_active !=
      current_context.dynamic_obstacle_constraint_active)
    {
      resolution.diagnostic = "dynamic-obstacle-active";
    } else if (
      previous_context.dynamic_obstacle_id !=
      current_context.dynamic_obstacle_id)
    {
      resolution.diagnostic = "dynamic-obstacle-id";
    } else if (
      previous_context.dynamic_obstacle_side_sign !=
      current_context.dynamic_obstacle_side_sign)
    {
      resolution.diagnostic = "dynamic-obstacle-side";
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
         (!result.physical_dynamic_sqp_audit_requested ||
         (result.physical_dynamic_sqp_audit_applied &&
         result.physical_dynamic_sqp_audit_solved &&
         result.physical_dynamic_sqp_audit_count ==
         result.physical_dynamic_sqp_audit_iteration_limit)) &&
         result.execution_artifact_reject_reason ==
         artifact::RejectReason::None &&
         result.execution_artifact != nullptr &&
         artifact::validate(*result.execution_artifact) ==
         artifact::RejectReason::None;
}

persistent_osqp::PhysicalConstraintTolerance
SolverContext::physical_constraint_tolerance() const noexcept
{
  return solver_.physical_constraint_tolerance();
}

const char * to_string(const TimeAlignedSuffixReason reason) noexcept
{
  switch (reason) {
    case TimeAlignedSuffixReason::Accepted:
      return "accepted";
    case TimeAlignedSuffixReason::InvalidRequest:
      return "invalid-request";
    case TimeAlignedSuffixReason::TimeRegression:
      return "time-regression";
    case TimeAlignedSuffixReason::HorizonExhausted:
      return "horizon-exhausted";
    case TimeAlignedSuffixReason::ExecutionPrefixExhausted:
      return "execution-prefix-exhausted";
    case TimeAlignedSuffixReason::SubminimumFirstStage:
      return "subminimum-first-stage";
    case TimeAlignedSuffixReason::NominalPathMismatch:
      return "nominal-path-mismatch";
    case TimeAlignedSuffixReason::DynamicObstacleStageMismatch:
      return "dynamic-obstacle-stage-mismatch";
    case TimeAlignedSuffixReason::Count:
      return "count";
  }
  return "unknown";
}

TimeAlignedSuffixResult resolve_time_aligned_suffix(
  const TimeAlignedSuffixRequest & request) noexcept
{
  namespace model = mpcc_rate_resolved;
  TimeAlignedSuffixResult result;
  const auto reject = [&result](
      const TimeAlignedSuffixReason reason, const std::string & detail) {
      result.reason = reason;
      result.detail = detail;
      return result;
    };
  try {
    if (
      request.source == nullptr ||
      !artifact::identity_valid(request.source->identity) ||
      !std::isfinite(request.control_prediction_origin_sec) ||
      !std::isfinite(request.source->control_prediction_origin_sec) ||
      !request.previous_input.allFinite())
    {
      return reject(
        TimeAlignedSuffixReason::InvalidRequest,
        "invalid source identity, timing or previous input");
    }
    const auto & source = *request.source;
    const auto & latest = request.initial_state;
    if (
      !std::isfinite(latest.lateral_m) || !std::isfinite(latest.lag_m) ||
      !std::isfinite(latest.heading_offset_rad) ||
      !std::isfinite(latest.velocity_mps) || latest.velocity_mps < 0.0 ||
      !std::isfinite(latest.progress_m) ||
      !std::isfinite(latest.steering_rad) ||
      !std::isfinite(latest.response_steering_rad) ||
      std::abs(latest.steering_rad) >
      source.request.maximum_abs_steering_rad ||
      std::abs(latest.response_steering_rad) >
      source.request.maximum_abs_steering_rad)
    {
      return reject(
        TimeAlignedSuffixReason::InvalidRequest,
        "invalid latest seven-state observation");
    }
    const int horizon = source.request.horizon_steps;
    if (
      horizon <= 0 ||
      source.request.inputs.size() != static_cast<std::size_t>(horizon) ||
      source.request.states.size() != static_cast<std::size_t>(horizon + 1) ||
      source.execution_prefix_steps <= 0 ||
      source.execution_prefix_steps > horizon)
    {
      return reject(
        TimeAlignedSuffixReason::InvalidRequest,
        "invalid source horizon or execution prefix");
    }
    if (
      source.nominal_path_distance_m.size() !=
      static_cast<std::size_t>(horizon + 1))
    {
      return reject(
        TimeAlignedSuffixReason::NominalPathMismatch,
        "nominal path does not match source horizon");
    }
    if (
      !source.dynamic_obstacle_stages.empty() &&
      source.dynamic_obstacle_stages.size() !=
      static_cast<std::size_t>(horizon))
    {
      return reject(
        TimeAlignedSuffixReason::DynamicObstacleStageMismatch,
        "dynamic obstacle stages do not match source horizon");
    }

    double elapsed_sec = request.control_prediction_origin_sec -
      source.control_prediction_origin_sec;
    constexpr double kTimeEpsilonSec = 1.0e-9;
    if (!std::isfinite(elapsed_sec) || elapsed_sec < -kTimeEpsilonSec) {
      return reject(
        TimeAlignedSuffixReason::TimeRegression,
        "feedback control origin precedes preparation origin");
    }
    elapsed_sec = std::max(0.0, elapsed_sec);
    std::size_t consumed = 0U;
    while (consumed < source.request.inputs.size()) {
      const double stage_dt = source.request.inputs[consumed].stage_dt_sec;
      if (!std::isfinite(stage_dt) || stage_dt <= 0.0) {
        return reject(
          TimeAlignedSuffixReason::InvalidRequest,
          "source contains invalid stage duration");
      }
      if (elapsed_sec < stage_dt - kTimeEpsilonSec) {
        break;
      }
      elapsed_sec = std::max(0.0, elapsed_sec - stage_dt);
      ++consumed;
    }
    result.consumed_stage_count = consumed;
    result.elapsed_in_first_remaining_stage_sec = elapsed_sec;
    if (consumed >= source.request.inputs.size()) {
      return reject(
        TimeAlignedSuffixReason::HorizonExhausted,
        "feedback control origin is outside the prepared horizon");
    }
    if (consumed >= static_cast<std::size_t>(source.execution_prefix_steps)) {
      return reject(
        TimeAlignedSuffixReason::ExecutionPrefixExhausted,
        "feedback control origin is outside the certified execution prefix");
    }
    const double source_first_dt =
      source.request.inputs[consumed].stage_dt_sec;
    const double first_remaining_dt = source_first_dt - elapsed_sec;
    result.first_remaining_stage_duration_sec = first_remaining_dt;
    if (
      !std::isfinite(first_remaining_dt) ||
      first_remaining_dt < source.request.minimum_stage_dt_sec)
    {
      return reject(
        TimeAlignedSuffixReason::SubminimumFirstStage,
        "remaining first stage is shorter than the model contract");
    }

    for (std::size_t index = 0U;
      index < source.nominal_path_distance_m.size(); ++index)
    {
      const double distance = source.nominal_path_distance_m[index];
      if (
        !std::isfinite(distance) ||
        (index > 0U &&
        distance + 1.0e-9 < source.nominal_path_distance_m[index - 1U]))
      {
        return reject(
          TimeAlignedSuffixReason::NominalPathMismatch,
          "nominal path is nonfinite or nonmonotonic");
      }
    }

    Snapshot suffix = source;
    const int suffix_horizon =
      horizon - static_cast<int>(consumed);
    suffix.control_prediction_origin_sec =
      request.control_prediction_origin_sec;
    suffix.request.horizon_steps = suffix_horizon;
    suffix.request.initial_state <<
      latest.lateral_m, latest.lag_m, latest.heading_offset_rad,
      latest.velocity_mps, latest.progress_m;
    suffix.request.current_steering_rad = latest.steering_rad;
    suffix.request.current_response_steering_rad =
      latest.response_steering_rad;
    suffix.request.previous_input[0] =
      request.previous_input[model::kAccelerationIndex];
    suffix.request.previous_input[2] =
      request.previous_input[model::kVirtualProgressSpeedIndex];

    std::vector<mpcc_rate_resolved_adapter::StateStage> states;
    states.reserve(static_cast<std::size_t>(suffix_horizon + 1));
    auto current_stage = source.request.states[consumed];
    current_stage.reference <<
      latest.lateral_m, latest.lag_m, latest.heading_offset_rad,
      latest.velocity_mps, latest.progress_m;
    current_stage.lower = current_stage.reference;
    current_stage.upper = current_stage.reference;
    states.push_back(std::move(current_stage));
    for (std::size_t stage = consumed + 1U;
      stage < source.request.states.size(); ++stage)
    {
      states.push_back(source.request.states[stage]);
    }
    suffix.request.states = std::move(states);

    suffix.request.inputs.assign(
      source.request.inputs.begin() + static_cast<std::ptrdiff_t>(consumed),
      source.request.inputs.end());
    suffix.request.inputs.front().stage_dt_sec = first_remaining_dt;
    suffix.execution_prefix_steps =
      source.execution_prefix_steps - static_cast<int>(consumed);

    const double first_fraction = elapsed_sec / source_first_dt;
    const double current_nominal_distance =
      source.nominal_path_distance_m[consumed] + first_fraction *
      (source.nominal_path_distance_m[consumed + 1U] -
      source.nominal_path_distance_m[consumed]);
    suffix.nominal_path_distance_m.clear();
    suffix.nominal_path_distance_m.reserve(
      static_cast<std::size_t>(suffix_horizon + 1));
    suffix.nominal_path_distance_m.push_back(0.0);
    for (std::size_t stage = consumed + 1U;
      stage < source.nominal_path_distance_m.size(); ++stage)
    {
      const double relative_distance =
        source.nominal_path_distance_m[stage] - current_nominal_distance;
      if (!std::isfinite(relative_distance) || relative_distance < -1.0e-9) {
        return reject(
          TimeAlignedSuffixReason::NominalPathMismatch,
          "feedback origin lies beyond a nominal path stage");
      }
      suffix.nominal_path_distance_m.push_back(
        std::max(0.0, relative_distance));
    }

    if (!source.dynamic_obstacle_stages.empty()) {
      suffix.dynamic_obstacle_stages.assign(
        source.dynamic_obstacle_stages.begin() +
        static_cast<std::ptrdiff_t>(consumed),
        source.dynamic_obstacle_stages.end());
    }
    const auto shift_optional_stage = [consumed](const int stage) {
        if (stage < 0) {
          return -1;
        }
        return std::max(0, stage - static_cast<int>(consumed));
      };
    suffix.dynamic_obstacle_forced_first_pass_side_stage =
      shift_optional_stage(
      source.dynamic_obstacle_forced_first_pass_side_stage);
    suffix.dynamic_obstacle_forced_first_ahead_stage =
      shift_optional_stage(source.dynamic_obstacle_forced_first_ahead_stage);
    suffix.dynamic_obstacle_forced_diagonal_start_stage =
      shift_optional_stage(
      source.dynamic_obstacle_forced_diagonal_start_stage);
    suffix.dynamic_obstacle_forced_diagonal_full_side_stage =
      shift_optional_stage(
      source.dynamic_obstacle_forced_diagonal_full_side_stage);

    auto source_context = suffix.identity.source_context;
    source_context.horizon_steps =
      static_cast<std::size_t>(suffix_horizon);
    suffix.identity.source_context =
      mpcc_execution_contract::seal_problem_context(
      std::move(source_context));

    result.reason = TimeAlignedSuffixReason::Accepted;
    result.snapshot = std::move(suffix);
    result.detail = "time-aligned semantic suffix rebuilt";
    return result;
  } catch (const std::exception & error) {
    return reject(
      TimeAlignedSuffixReason::InvalidRequest,
      std::string{"suffix rebuild exception: "} + error.what());
  } catch (...) {
    return reject(
      TimeAlignedSuffixReason::InvalidRequest,
      "suffix rebuild unknown exception");
  }
}

const char * to_string(const TimeAlignedFeedbackProblemReason reason) noexcept
{
  switch (reason) {
    case TimeAlignedFeedbackProblemReason::Accepted:
      return "accepted";
    case TimeAlignedFeedbackProblemReason::InvalidRequest:
      return "invalid-request";
    case TimeAlignedFeedbackProblemReason::SuffixRejected:
      return "suffix-rejected";
    case TimeAlignedFeedbackProblemReason::PreparationDimensionMismatch:
      return "preparation-dimension-mismatch";
    case TimeAlignedFeedbackProblemReason::SemanticAdapterRejected:
      return "semantic-adapter-rejected";
    case TimeAlignedFeedbackProblemReason::RefinementProvenanceMismatch:
      return "refinement-provenance-mismatch";
    case TimeAlignedFeedbackProblemReason::RelinearizationRejected:
      return "relinearization-rejected";
    case TimeAlignedFeedbackProblemReason::Count:
      return "count";
  }
  return "unknown";
}

TimeAlignedFeedbackProblemResult build_time_aligned_feedback_problem(
  const TimeAlignedFeedbackProblemRequest & request) noexcept
{
  namespace model = mpcc_rate_resolved;
  namespace problem = mpcc_rate_resolved_problem;
  TimeAlignedFeedbackProblemResult result;
  const auto reject = [&result](
      const TimeAlignedFeedbackProblemReason reason,
      const std::string & detail) {
      result.reason = reason;
      result.detail = detail;
      result.problem.reset();
      return result;
    };
  try {
    if (
      request.preparation == nullptr || !request.previous_input.allFinite() ||
      !std::isfinite(request.physical_constraint_tolerance.absolute) ||
      request.physical_constraint_tolerance.absolute < 0.0 ||
      !std::isfinite(request.physical_constraint_tolerance.relative) ||
      request.physical_constraint_tolerance.relative < 0.0)
    {
      return reject(
        TimeAlignedFeedbackProblemReason::InvalidRequest,
        "invalid preparation, previous input or physical tolerance");
    }

    const auto & preparation = *request.preparation;
    result.suffix = resolve_time_aligned_suffix(
      TimeAlignedSuffixRequest{
        &preparation.snapshot, request.control_prediction_origin_sec,
        request.initial_state, request.previous_input});
    if (
      result.suffix.reason != TimeAlignedSuffixReason::Accepted ||
      !result.suffix.snapshot.has_value())
    {
      return reject(
        TimeAlignedFeedbackProblemReason::SuffixRejected,
        std::string{"semantic suffix rejected: "} + result.suffix.detail);
    }

    const int old_horizon = preparation.snapshot.request.horizon_steps;
    const int new_horizon =
      result.suffix.snapshot->request.horizon_steps;
    const int consumed = static_cast<int>(result.suffix.consumed_stage_count);
    constexpr int nx = model::kStateDimension;
    constexpr int nu = model::kInputDimension;
    const int old_state_values = nx * (old_horizon + 1);
    const int old_input_values = nu * old_horizon;
    const int old_variable_count = old_state_values + old_input_values;
    const int new_state_values = nx * (new_horizon + 1);
    const int new_input_values = nu * new_horizon;
    const int new_variable_count = new_state_values + new_input_values;
    const auto & final_problem = preparation.final_problem;
    const bool valid_dimensions =
      old_horizon > 0 && new_horizon > 0 && consumed >= 0 &&
      consumed < old_horizon && new_horizon == old_horizon - consumed &&
      final_problem.horizon_steps == old_horizon &&
      final_problem.linearizations.size() ==
      static_cast<std::size_t>(old_horizon) &&
      final_problem.state_reference.size() == old_state_values &&
      final_problem.state_lower.size() == old_state_values &&
      final_problem.state_upper.size() == old_state_values &&
      final_problem.state_weight.size() == old_state_values &&
      final_problem.input_reference.size() == old_input_values &&
      final_problem.input_lower.size() == old_input_values &&
      final_problem.input_upper.size() == old_input_values &&
      final_problem.input_weight.size() == old_input_values &&
      (final_problem.additional_linear_cost.size() == 0 ||
      final_problem.additional_linear_cost.size() == old_variable_count) &&
      preparation.prepared_primal.size() == old_variable_count &&
      preparation.prepared_primal.allFinite();
    if (!valid_dimensions) {
      return reject(
        TimeAlignedFeedbackProblemReason::PreparationDimensionMismatch,
        "prepared final problem does not match its semantic horizon");
    }

    const auto semantic = mpcc_rate_resolved_adapter::build(
      result.suffix.snapshot->request,
      request.physical_constraint_tolerance);
    if (!semantic.has_value()) {
      return reject(
        TimeAlignedFeedbackProblemReason::SemanticAdapterRejected,
        "latest semantic suffix cannot build the physical input envelope");
    }

    // Validate every refinement row against the old problem before deciding
    // whether it belongs to the unconsumed suffix.  Otherwise an invalid stage
    // can look elapsed and be silently dropped, hiding producer corruption.
    if (final_problem.progress_aligned_wall_constraints.has_value()) {
      const auto & wall =
        final_problem.progress_aligned_wall_constraints.value();
      if (
        wall.lower_slope.size() != static_cast<std::size_t>(old_horizon) ||
        wall.lower_intercept.size() !=
        static_cast<std::size_t>(old_horizon) ||
        wall.upper_slope.size() != static_cast<std::size_t>(old_horizon) ||
        wall.upper_intercept.size() !=
        static_cast<std::size_t>(old_horizon))
      {
        return reject(
          TimeAlignedFeedbackProblemReason::RefinementProvenanceMismatch,
          "progress-wall rows do not match preparation horizon");
      }
    }
    for (const auto & wall : final_problem.swept_lateral_wall_constraints) {
      if (
        wall.transition_stage < 0 ||
        wall.transition_stage >= old_horizon ||
        !std::isfinite(wall.destination_ratio) ||
        wall.destination_ratio <= 0.0 || wall.destination_ratio >= 1.0 ||
        std::isnan(wall.lower_m) || std::isnan(wall.upper_m) ||
        wall.lower_m > wall.upper_m)
      {
        return reject(
          TimeAlignedFeedbackProblemReason::RefinementProvenanceMismatch,
          "swept-wall row has invalid preparation-stage provenance");
      }
    }
    for (const auto & obstacle : final_problem.dynamic_obstacle_constraints) {
      const bool supported_axis =
        obstacle.axis == problem::DynamicObstacleConstraintAxis::Lateral ||
        obstacle.axis ==
        problem::DynamicObstacleConstraintAxis::EffectiveProgress ||
        obstacle.axis ==
        problem::DynamicObstacleConstraintAxis::CoupledLateralProgress;
      const bool valid_coupled =
        obstacle.axis !=
        problem::DynamicObstacleConstraintAxis::CoupledLateralProgress ||
        (std::isfinite(obstacle.lateral_coefficient) &&
        std::isfinite(obstacle.effective_progress_coefficient) &&
        (obstacle.lateral_coefficient != 0.0 ||
        obstacle.effective_progress_coefficient != 0.0));
      if (
        obstacle.state_stage <= 0 ||
        obstacle.state_stage > old_horizon || !supported_axis ||
        !valid_coupled || std::isnan(obstacle.lower) ||
        std::isnan(obstacle.upper) || obstacle.lower > obstacle.upper)
      {
        return reject(
          TimeAlignedFeedbackProblemReason::RefinementProvenanceMismatch,
          "dynamic-obstacle row has invalid preparation-stage provenance");
      }
    }

    problem::AssemblyRequest feedback;
    feedback.horizon_steps = new_horizon;
    feedback.initial_state = semantic->problem.initial_state;
    feedback.linearizations.assign(
      final_problem.linearizations.begin() + consumed,
      final_problem.linearizations.end());
    const int old_state_offset = consumed * nx;
    const int old_input_offset = consumed * nu;
    feedback.state_reference = final_problem.state_reference.segment(
      old_state_offset, new_state_values);
    feedback.state_lower = final_problem.state_lower.segment(
      old_state_offset, new_state_values);
    feedback.state_upper = final_problem.state_upper.segment(
      old_state_offset, new_state_values);
    feedback.state_weight = final_problem.state_weight.segment(
      old_state_offset, new_state_values);
    feedback.input_reference = final_problem.input_reference.segment(
      old_input_offset, new_input_values);
    feedback.input_lower = final_problem.input_lower.segment(
      old_input_offset, new_input_values);
    feedback.input_upper = final_problem.input_upper.segment(
      old_input_offset, new_input_values);
    feedback.input_weight = final_problem.input_weight.segment(
      old_input_offset, new_input_values);
    if (final_problem.additional_linear_cost.size() == old_variable_count) {
      feedback.additional_linear_cost =
        Eigen::VectorXd::Zero(new_variable_count);
      feedback.additional_linear_cost.head(new_state_values) =
        final_problem.additional_linear_cost.segment(
        old_state_offset, new_state_values);
      feedback.additional_linear_cost.tail(new_input_values) =
        final_problem.additional_linear_cost.segment(
        old_state_values + old_input_offset, new_input_values);
    }
    feedback.previous_input = request.previous_input;
    feedback.input_delta_weight = final_problem.input_delta_weight;
    feedback.steering_rate_prefix_bounds =
      semantic->problem.steering_rate_prefix_bounds;

    // x0 and the steering-response chain have changed control origins.  The
    // remaining five-state future boxes still carry the final physical wall
    // and obstacle refinement from preparation and must not be replaced.
    feedback.state_reference.head(nx) =
      semantic->problem.state_reference.head(nx);
    feedback.state_lower.head(nx) = semantic->problem.state_lower.head(nx);
    feedback.state_upper.head(nx) = semantic->problem.state_upper.head(nx);
    feedback.state_weight.head(nx) = semantic->problem.state_weight.head(nx);
    if (feedback.additional_linear_cost.size() == new_variable_count) {
      feedback.additional_linear_cost.head(nx) =
        semantic->problem.additional_linear_cost.head(nx);
    }
    for (int stage = 1; stage <= new_horizon; ++stage) {
      const int state = stage * nx;
      for (const int element :
        {model::kSteeringIndex, model::kResponseSteeringIndex})
      {
        feedback.state_reference[state + element] =
          semantic->problem.state_reference[state + element];
        feedback.state_lower[state + element] =
          semantic->problem.state_lower[state + element];
        feedback.state_upper[state + element] =
          semantic->problem.state_upper[state + element];
        feedback.state_weight[state + element] =
          semantic->problem.state_weight[state + element];
        if (feedback.additional_linear_cost.size() == new_variable_count) {
          feedback.additional_linear_cost[state + element] =
            semantic->problem.additional_linear_cost[state + element];
        }
      }
    }
    feedback.input_reference.head(nu) =
      semantic->problem.input_reference.head(nu);
    feedback.input_lower.head(nu) = semantic->problem.input_lower.head(nu);
    feedback.input_upper.head(nu) = semantic->problem.input_upper.head(nu);
    feedback.input_weight.head(nu) = semantic->problem.input_weight.head(nu);
    if (feedback.additional_linear_cost.size() == new_variable_count) {
      feedback.additional_linear_cost.segment(new_state_values, nu) =
        semantic->problem.additional_linear_cost.segment(new_state_values, nu);
    }

    if (final_problem.progress_aligned_wall_constraints.has_value()) {
      const auto & source_wall =
        final_problem.progress_aligned_wall_constraints.value();
      problem::ProgressAlignedWallConstraints wall;
      wall.lower_slope.assign(
        source_wall.lower_slope.begin() + consumed,
        source_wall.lower_slope.end());
      wall.lower_intercept.assign(
        source_wall.lower_intercept.begin() + consumed,
        source_wall.lower_intercept.end());
      wall.upper_slope.assign(
        source_wall.upper_slope.begin() + consumed,
        source_wall.upper_slope.end());
      wall.upper_intercept.assign(
        source_wall.upper_intercept.begin() + consumed,
        source_wall.upper_intercept.end());
      feedback.progress_aligned_wall_constraints = std::move(wall);
    }
    for (auto wall : final_problem.swept_lateral_wall_constraints) {
      if (wall.transition_stage >= consumed) {
        wall.transition_stage -= consumed;
        feedback.swept_lateral_wall_constraints.push_back(std::move(wall));
      }
    }
    for (auto obstacle : final_problem.dynamic_obstacle_constraints) {
      // state_stage == consumed is the newly observed immutable x0. Exact
      // current-world proof owns it; future QP rows start at suffix stage 1.
      if (obstacle.state_stage > consumed) {
        obstacle.state_stage -= consumed;
        feedback.dynamic_obstacle_constraints.push_back(
          std::move(obstacle));
      }
    }

    result.linearization_primal = Eigen::VectorXd::Zero(new_variable_count);
    result.linearization_primal.head(nx) = feedback.initial_state;
    if (new_horizon > 0) {
      result.linearization_primal.segment(nx, new_horizon * nx) =
        preparation.prepared_primal.segment(
        (consumed + 1) * nx, new_horizon * nx);
      result.linearization_primal.tail(new_input_values) =
        preparation.prepared_primal.segment(
        old_state_values + old_input_offset, new_input_values);
    }
    const auto relinearization =
      mpcc_rate_resolved_adapter::relinearize_around_primal(
      result.suffix.snapshot->request, result.linearization_primal, feedback);
    if (!relinearization.applied) {
      return reject(
        TimeAlignedFeedbackProblemReason::RelinearizationRejected,
        std::string{"suffix relinearization rejected: "} +
        mpcc_rate_resolved_adapter::to_string(relinearization.reason) +
        "/stage=" + std::to_string(relinearization.stage));
    }
    if (!problem::assemble(feedback).has_value()) {
      return reject(
        TimeAlignedFeedbackProblemReason::PreparationDimensionMismatch,
        "time-aligned refined feedback problem failed assembly validation");
    }

    result.reason = TimeAlignedFeedbackProblemReason::Accepted;
    result.problem = std::move(feedback);
    result.detail = "time-aligned prepared-QP suffix rebuilt";
    return result;
  } catch (const std::exception & error) {
    return reject(
      TimeAlignedFeedbackProblemReason::InvalidRequest,
      std::string{"feedback problem rebuild exception: "} + error.what());
  } catch (...) {
    return reject(
      TimeAlignedFeedbackProblemReason::InvalidRequest,
      "feedback problem rebuild unknown exception");
  }
}

persistent_osqp::PhysicalConstraintTolerance
LatestStateFeedbackSolverContext::physical_constraint_tolerance() const noexcept
{
  return solver_.physical_constraint_tolerance();
}

LatestStateFeedbackResult LatestStateFeedbackSolverContext::evaluate(
  const LatestStateFeedbackRequest & request)
{
  namespace model = mpcc_rate_resolved;
  const auto started = SteadyClock::now();
  LatestStateFeedbackResult result;
  if (request.preparation != nullptr) {
    result.identity = request.preparation->snapshot.identity;
  }
  const auto finish = [&]() {
      result.compute_ms = std::chrono::duration<double, std::milli>(
        SteadyClock::now() - started).count();
      return result;
    };
  try {
    if (
      request.preparation == nullptr ||
      !mpcc_rate_resolved_shadow::identity_valid(
        request.preparation->snapshot.identity) ||
      !std::isfinite(request.control_prediction_origin_sec) ||
      !std::isfinite(request.observation_sec) ||
      request.observation_sec < request.preparation->snapshot.identity.snapshot_sec ||
      request.control_prediction_origin_sec < request.observation_sec ||
      !request.previous_input.allFinite())
    {
      result.detail = "invalid feedback timing, identity or previous input";
      return finish();
    }
    const auto & state = request.initial_state;
    Eigen::Matrix<double, model::kStateDimension, 1> initial_state;
    initial_state << state.lateral_m, state.lag_m, state.heading_offset_rad,
      state.velocity_mps, state.progress_m, state.steering_rad,
      state.response_steering_rad;
    const auto & source = request.preparation->snapshot;
    if (
      !initial_state.allFinite() || state.velocity_mps < 0.0 ||
      std::abs(state.steering_rad) >
      source.request.maximum_abs_steering_rad ||
      std::abs(state.response_steering_rad) >
      source.request.maximum_abs_steering_rad ||
      source.request.horizon_steps <= 0 ||
      source.request.inputs.empty())
    {
      result.detail = "invalid latest seven-state feedback state";
      return finish();
    }

    auto feedback_snapshot = source;
    auto feedback_problem = request.preparation->final_problem;
    feedback_snapshot.control_prediction_origin_sec =
      request.control_prediction_origin_sec;
    feedback_snapshot.request.initial_state <<
      state.lateral_m, state.lag_m, state.heading_offset_rad,
      state.velocity_mps, state.progress_m;
    feedback_snapshot.request.current_steering_rad = state.steering_rad;
    feedback_snapshot.request.current_response_steering_rad =
      state.response_steering_rad;
    feedback_problem.initial_state = initial_state;
    feedback_problem.previous_input = request.previous_input;

    const int horizon = feedback_problem.horizon_steps;
    const int state_values = model::kStateDimension * (horizon + 1);
    const int input_values = model::kInputDimension * horizon;
    if (
      horizon != feedback_snapshot.request.horizon_steps ||
      feedback_problem.state_reference.size() != state_values ||
      feedback_problem.state_lower.size() != state_values ||
      feedback_problem.state_upper.size() != state_values ||
      feedback_problem.input_lower.size() != input_values ||
      feedback_problem.input_upper.size() != input_values ||
      request.preparation->prepared_primal.size() !=
      state_values + input_values)
    {
      result.detail = "feedback preparation dimension mismatch";
      return finish();
    }
    feedback_problem.state_reference.head<model::kStateDimension>() =
      initial_state;
    feedback_problem.state_lower.head<model::kStateDimension>() = initial_state;
    feedback_problem.state_upper.head<model::kStateDimension>() = initial_state;

    const auto tolerance = solver_.physical_constraint_tolerance();
    const auto prefix_bounds =
      mpcc_rate_resolved_adapter::resolve_exact_physical_boundary_bounds(
      -source.request.maximum_abs_steering_rad - state.steering_rad,
      source.request.maximum_abs_steering_rad - state.steering_rad,
      tolerance);
    const double first_dt =
      feedback_snapshot.request.inputs.front().stage_dt_sec;
    if (!prefix_bounds.has_value() || !std::isfinite(first_dt) || first_dt <= 0.0) {
      result.detail = "latest-state steering prefix unavailable";
      return finish();
    }
    feedback_problem.steering_rate_prefix_bounds =
      mpcc_rate_resolved_problem::SteeringRatePrefixBounds{
      prefix_bounds->lower, prefix_bounds->upper};
    const double physical_rate_lower = std::max(
      -source.request.maximum_abs_steering_rate_radps,
      (-source.request.maximum_abs_steering_rad - state.steering_rad) /
      first_dt);
    const double physical_rate_upper = std::min(
      source.request.maximum_abs_steering_rate_radps,
      (source.request.maximum_abs_steering_rad - state.steering_rad) /
      first_dt);
    const auto rate_bounds =
      mpcc_rate_resolved_adapter::resolve_exact_physical_boundary_bounds(
      physical_rate_lower, physical_rate_upper, tolerance);
    if (!rate_bounds.has_value()) {
      result.detail = "latest-state first steering-rate bounds unavailable";
      return finish();
    }
    feedback_problem.input_lower[model::kSteeringRateIndex] =
      rate_bounds->lower;
    feedback_problem.input_upper[model::kSteeringRateIndex] =
      rate_bounds->upper;

    auto assembled = mpcc_rate_resolved_problem::assemble(feedback_problem);
    if (!assembled.has_value()) {
      result.reason = LatestStateFeedbackReason::AssemblyRejected;
      result.detail = "latest-state feedback QP assembly rejected";
      return finish();
    }
    result.assembled = true;
    auto bootstrap = build_current_problem_bootstrap(
      feedback_problem,
      static_cast<std::size_t>(assembled->lower_bound.size()),
      &request.preparation->prepared_primal);
    if (!bootstrap.has_value()) {
      result.reason = LatestStateFeedbackReason::BootstrapRejected;
      result.detail = "latest-state feedback bootstrap rejected";
      return finish();
    }

    result.solve_attempted = true;
    persistent_osqp::SolveOutcome outcome;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      outcome = solver_.solve(
        assembled->quadratic_cost, assembled->constraints,
        assembled->linear_cost, assembled->lower_bound,
        assembled->upper_bound, bootstrap, assembled->variable_scaling);
    }
    result.solver = outcome.telemetry;
    if (!outcome.result.has_value()) {
      result.reason = LatestStateFeedbackReason::SolveRejected;
      result.detail = outcome.failure_detail.empty() ?
        "latest-state feedback solve rejected" : outcome.failure_detail;
      return finish();
    }
    result.solved = true;
    result.finite = outcome.result->primal.allFinite();
    // PersistentOsqp exposes a SolveResult only after the original physical
    // constraint rows have passed its residual certificate.
    result.constraints_satisfied = true;
    if (!result.finite || !result.constraints_satisfied) {
      result.reason = LatestStateFeedbackReason::SolveRejected;
      result.detail = "latest-state feedback result not executable";
      return finish();
    }

    const double completed_sec = request.observation_sec +
      std::chrono::duration<double>(SteadyClock::now() - started).count();
    auto artifact_build = build_execution_artifact(
      feedback_snapshot, feedback_problem, outcome, completed_sec);
    result.artifact_reject_reason = artifact_build.reject_reason;
    if (!artifact_build.execution_artifact.has_value()) {
      result.reason = LatestStateFeedbackReason::ArtifactRejected;
      result.detail = artifact_build.detail;
      return finish();
    }
    auto execution_artifact =
      std::move(artifact_build.execution_artifact.value());
    result.physical_adapter_reason =
      mpcc_rate_resolved_physical_adapter::build(
      execution_artifact,
      execution_artifact.identity.source_context.intent,
      execution_artifact.identity.source_context.stage_geometry_id).reason;
    if (
      result.physical_adapter_reason !=
      mpcc_rate_resolved_physical_adapter::RejectReason::None)
    {
      result.reason = LatestStateFeedbackReason::PhysicalAdapterRejected;
      result.detail = "latest-state nonlinear physical adapter rejected";
      return finish();
    }
    result.execution_artifact =
      std::make_shared<const artifact::ExecutionArtifact>(
      std::move(execution_artifact));
    result.reason = LatestStateFeedbackReason::Accepted;
    result.detail = "latest-state feedback QP and physical adapter accepted";
    return finish();
  } catch (const std::exception & error) {
    result.reason = LatestStateFeedbackReason::Exception;
    result.detail = error.what();
    return finish();
  } catch (...) {
    result.reason = LatestStateFeedbackReason::Exception;
    result.detail = "unknown latest-state feedback exception";
    return finish();
  }
}

Result SolverContext::evaluate(const Snapshot & snapshot)
{
  return evaluate_impl(snapshot, false, std::nullopt, 0U);
}

Result SolverContext::evaluate_wall_feasibility_restoration_audit(
  const Snapshot & snapshot)
{
  return evaluate_impl(snapshot, true, std::nullopt, 0U);
}

Result SolverContext::evaluate_wall_bucket_audit(
  const Snapshot & snapshot, const WallBucketAuditMode mode)
{
  return evaluate_impl(snapshot, false, mode, 0U);
}

Result SolverContext::evaluate_physical_dynamic_sqp_audit(
  const Snapshot & snapshot, const std::size_t iteration_count)
{
  return evaluate_impl(snapshot, false, std::nullopt, iteration_count);
}

Result SolverContext::evaluate_impl(
  const Snapshot & snapshot,
  const bool wall_feasibility_restoration_audit,
  const std::optional<WallBucketAuditMode> wall_bucket_audit_mode,
  const std::size_t physical_dynamic_sqp_audit_iteration_count)
{
  const auto started = SteadyClock::now();
  Result result;
  result.identity = snapshot.identity;
  result.wall_feasibility_restoration_requested =
    wall_feasibility_restoration_audit;
  result.physical_dynamic_sqp_audit_requested =
    physical_dynamic_sqp_audit_iteration_count > 0U;
  result.physical_dynamic_sqp_audit_iteration_limit =
    physical_dynamic_sqp_audit_iteration_count;
  const bool wall_bucket_phase_one_enabled =
    wall_bucket_audit_mode.has_value() &&
    wall_bucket_audit_mode.value() !=
    WallBucketAuditMode::OmitPoseDirect;
  const auto accumulate_wall_cache = [&result](
      const mpcc_rate_resolved_wall_refinement::Result & refinement) {
      result.physical_wall_refinement_cache_hit_count +=
        refinement.cache_hit_count;
      result.physical_wall_refinement_cache_miss_count +=
        refinement.cache_miss_count;
      result.physical_wall_refinement_cache_scanned_pose_count +=
        refinement.cache_scanned_pose_count;
    };
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
    physical_dynamic_sqp_audit_iteration_count >
    kMaximumPhysicalProofSqpCorrections)
  {
    result.detail = "physical dynamic SQP audit depth exceeds fixed budget";
    return finish();
  }
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
  // Preserve the broad, relinearized current-world problem.  A wall-only
  // solve below is a physical homotopy witness; its narrow progress buckets
  // must not remove the longitudinal freedom needed by a subsequent
  // stay-behind obstacle constraint.
  const bool coupled_wall_obstacle_refinement =
    snapshot.progress_aligned_wall_refinement_active &&
    snapshot.dynamic_obstacle_refinement_active;
  std::optional<mpcc_rate_resolved_problem::AssemblyRequest>
    broad_problem_before_wall;
  if (coupled_wall_obstacle_refinement) {
    broad_problem_before_wall = adapted->problem;
  }
  if (snapshot.progress_aligned_wall_refinement_active) {
    auto refinement = build_progress_wall_refinement(
      snapshot, adapted->problem, outcome.result->primal,
      solver_.physical_constraint_tolerance(), &wall_refinement_cache_,
      wall_bucket_audit_mode);
    accumulate_wall_cache(refinement.physical);
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
    result.physical_wall_lag_pose_box_applied =
      wall_bucket_audit_mode.has_value() &&
      wall_bucket_audit_mode != WallBucketAuditMode::OmitLag &&
      wall_bucket_audit_mode != WallBucketAuditMode::OmitPose &&
      wall_bucket_audit_mode != WallBucketAuditMode::OmitPoseDirect;
    result.physical_wall_heading_pose_box_applied =
      wall_bucket_audit_mode.has_value() &&
      wall_bucket_audit_mode != WallBucketAuditMode::OmitHeading &&
      wall_bucket_audit_mode != WallBucketAuditMode::OmitPose &&
      wall_bucket_audit_mode != WallBucketAuditMode::OmitPoseDirect;
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
    const auto racing_quadratic_cost = refined_assembled->quadratic_cost;
    const auto racing_linear_cost = refined_assembled->linear_cost;
    if (wall_bucket_phase_one_enabled) {
      // Architecture Phase-I asks whether the bucket-relaxed affine set can
      // produce a physically certifiable trajectory. The racing Hessian can
      // remain badly conditioned even after independent LP feasibility has
      // been established, so project the preceding current-problem iterate
      // with a strictly convex identity objective. No row, bound, tolerance
      // or production solve is changed.
      refined_assembled->quadratic_cost.setIdentity();
      refined_assembled->linear_cost = -outcome.result->primal;
    }
    auto refined_outcome = solver_.solve(
      refined_assembled->quadratic_cost, refined_assembled->constraints,
      refined_assembled->linear_cost, refined_assembled->lower_bound,
      refined_assembled->upper_bound, warm_start,
      refined_assembled->variable_scaling);
    bool bucket_phase_one_solved = false;
    if (
      wall_bucket_phase_one_enabled &&
      refined_outcome.result.has_value() &&
      refined_outcome.result->primal.allFinite())
    {
      bucket_phase_one_solved = true;
      refined_assembled->quadratic_cost = racing_quadratic_cost;
      refined_assembled->linear_cost = racing_linear_cost;
      warm_start = persistent_osqp::WarmStart{
        refined_outcome.result->primal,
        Eigen::VectorXd::Zero(refined_assembled->lower_bound.size())};
      refined_outcome = solver_.solve(
        refined_assembled->quadratic_cost, refined_assembled->constraints,
        refined_assembled->linear_cost, refined_assembled->lower_bound,
        refined_assembled->upper_bound, warm_start,
        refined_assembled->variable_scaling);
    }
    result.solver = refined_outcome.telemetry;
    if (
      !refined_outcome.result.has_value() &&
      wall_feasibility_restoration_audit)
    {
      result.wall_feasibility_restoration_attempted = true;
      // This problem retains every explicit wall/actuation row, but restores
      // the lateral/lag/heading/progress state boxes owned by the post-hoc
      // physical wall bucket. The resulting trajectory is not a certificate:
      // its wall rows were proved at the old pose bucket. It is used only to
      // construct a new nonlinear tangent and fresh wall refinement below.
      auto restoration_problem = refinement.request.value();
      for (int stage = 1; stage <= snapshot.request.horizon_steps; ++stage) {
        const int state = stage * mpcc_rate_resolved::kStateDimension;
        for (const int element : {
            mpcc_rate_resolved::kLateralIndex,
            mpcc_rate_resolved::kLagIndex,
            mpcc_rate_resolved::kHeadingIndex,
            mpcc_rate_resolved::kProgressIndex})
        {
          restoration_problem.state_lower[state + element] =
            adapted->problem.state_lower[state + element];
          restoration_problem.state_upper[state + element] =
            adapted->problem.state_upper[state + element];
        }
      }

      persistent_osqp::SolveOutcome restoration_outcome;
      Eigen::VectorXd restoration_primal = outcome.result->primal;
      bool restoration_failed = false;
      for (std::size_t correction = 0U;
        correction < kMaximumWallRestorationSqpCorrections; ++correction)
      {
        if (correction > 0U) {
          const auto tangent =
            mpcc_rate_resolved_adapter::relinearize_around_primal(
            snapshot.request, restoration_primal, restoration_problem);
          if (!tangent.applied) {
            result.wall_feasibility_restoration_detail =
              std::string{"restoration-tangent-rejected/"} +
              mpcc_rate_resolved_adapter::to_string(tangent.reason) +
              "/stage=" + std::to_string(tangent.stage);
            restoration_failed = true;
            break;
          }
        }
        auto restoration_assembled =
          mpcc_rate_resolved_problem::assemble(restoration_problem);
        if (!restoration_assembled.has_value()) {
          result.wall_feasibility_restoration_detail =
            "restoration-assembly-rejected";
          restoration_failed = true;
          break;
        }
        // Phase-I owns feasibility, not racing performance. Reusing the
        // original objective made an already affine-feasible seed hit OSQP's
        // dual iteration limit. Project the preceding seed onto the unchanged
        // affine constraints with a strictly convex identity objective. This
        // changes no bound or row and the projected result is still forbidden
        // from becoming an artifact.
        restoration_assembled->quadratic_cost.setIdentity();
        restoration_assembled->linear_cost = -restoration_primal;
        auto restoration_warm_start = build_current_problem_bootstrap(
          restoration_problem,
          static_cast<std::size_t>(
            restoration_assembled->lower_bound.size()),
          &restoration_primal);
        if (!restoration_warm_start.has_value()) {
          result.wall_feasibility_restoration_detail =
            "restoration-bootstrap-rejected";
          restoration_failed = true;
          break;
        }
        restoration_outcome = solver_.solve(
          restoration_assembled->quadratic_cost,
          restoration_assembled->constraints,
          restoration_assembled->linear_cost,
          restoration_assembled->lower_bound,
          restoration_assembled->upper_bound,
          restoration_warm_start,
          restoration_assembled->variable_scaling);
        result.solver = restoration_outcome.telemetry;
        if (
          !restoration_outcome.result.has_value() ||
          !restoration_outcome.result->primal.allFinite())
        {
          result.wall_feasibility_restoration_detail =
            std::string{"restoration-solve-rejected/"} +
            restoration_outcome.failure_detail;
          // Preserve the exact Phase-I problem at the common failure capture
          // boundary. Recording the original full refinement here made a
          // `restoration-solve-rejected` snapshot describe the wrong QP and
          // prevented independent feasibility classification.
          refinement.request = restoration_problem;
          refined_assembled = std::move(restoration_assembled);
          refined_outcome = std::move(restoration_outcome);
          warm_start = std::move(restoration_warm_start);
          restoration_failed = true;
          break;
        }
        restoration_primal = restoration_outcome.result->primal;
        ++result.wall_feasibility_restoration_sqp_count;
        result.wall_feasibility_restoration_seed_solved = true;
      }

      if (!restoration_failed &&
        result.wall_feasibility_restoration_seed_solved)
      {
        // Rebuild from the broad semantic problem so none of the relaxed
        // seed's old wall buckets survive. Only its tangent is transported.
        auto restored_tangent_problem = adapted->problem;
        const auto restored_tangent =
          mpcc_rate_resolved_adapter::relinearize_around_primal(
          snapshot.request, restoration_primal, restored_tangent_problem);
        if (!restored_tangent.applied) {
          result.wall_feasibility_restoration_detail =
            std::string{"final-tangent-rejected/"} +
            mpcc_rate_resolved_adapter::to_string(restored_tangent.reason) +
            "/stage=" + std::to_string(restored_tangent.stage);
        } else {
          auto final_refinement = build_progress_wall_refinement(
            snapshot, restored_tangent_problem, restoration_primal,
            solver_.physical_constraint_tolerance(), &wall_refinement_cache_,
            wall_bucket_audit_mode);
          accumulate_wall_cache(final_refinement.physical);
          result.wall_feasibility_restoration_final_refinement_built =
            final_refinement.request.has_value();
          if (!final_refinement.request.has_value()) {
            result.wall_feasibility_restoration_detail =
              std::string{"final-wall-refinement-rejected/progress="} +
              mpcc_progress::progress_aligned_wall_bounds_reason_name(
              final_refinement.resolution.reason) +
              "/physical=" +
              mpcc_rate_resolved_wall_refinement::to_string(
              final_refinement.physical.reason);
          } else {
            auto final_assembled = mpcc_rate_resolved_problem::assemble(
              final_refinement.request.value());
            if (!final_assembled.has_value()) {
              result.wall_feasibility_restoration_detail =
                "final-restored-assembly-rejected";
            } else {
              auto final_warm_start = build_current_problem_bootstrap(
                final_refinement.request.value(),
                static_cast<std::size_t>(
                  final_assembled->lower_bound.size()),
                &restoration_primal);
              if (!final_warm_start.has_value()) {
                result.wall_feasibility_restoration_detail =
                  "final-restored-bootstrap-rejected";
              } else {
                auto final_outcome = solver_.solve(
                  final_assembled->quadratic_cost,
                  final_assembled->constraints,
                  final_assembled->linear_cost,
                  final_assembled->lower_bound,
                  final_assembled->upper_bound,
                  final_warm_start,
                  final_assembled->variable_scaling);
                result.solver = final_outcome.telemetry;
                result.wall_feasibility_restoration_final_solved =
                  final_outcome.result.has_value() &&
                  final_outcome.result->primal.allFinite();
                if (result.wall_feasibility_restoration_final_solved) {
                  result.wall_feasibility_restoration_detail =
                    "accepted-as-seed-and-full-refinement-solved";
                  refinement = std::move(final_refinement);
                  refined_assembled = std::move(final_assembled);
                  refined_outcome = std::move(final_outcome);
                  warm_start = std::move(final_warm_start);
                } else {
                  result.wall_feasibility_restoration_detail =
                    std::string{"final-restored-solve-rejected/"} +
                    final_outcome.failure_detail;
                  refinement = std::move(final_refinement);
                  refined_assembled = std::move(final_assembled);
                  refined_outcome = std::move(final_outcome);
                  warm_start = std::move(final_warm_start);
                }
              }
            }
          }
        }
      }
    }
    if (!refined_outcome.result.has_value()) {
      result.outcome = Outcome::SolveRejected;
      result.solved = false;
      result.detail = std::string{"rate-resolved wall-refined QP rejected: "} +
        refined_outcome.failure_detail;
      if (wall_bucket_audit_mode.has_value()) {
        result.detail += std::string{", wall_bucket_phase_one="} +
          (wall_bucket_phase_one_enabled ?
          (bucket_phase_one_solved ? "solved" : "rejected") : "disabled");
      }
      if (result.wall_feasibility_restoration_attempted) {
        result.detail += std::string{", wall_restoration="} +
          result.wall_feasibility_restoration_detail +
          "/sqp_count=" +
          std::to_string(result.wall_feasibility_restoration_sqp_count);
      }
      capture_failure(
        mpcc_architecture_snapshot::PipelineStage::WallRefinement,
        refinement.request.value(), refined_assembled.value(), warm_start,
        refined_outcome,
        result.wall_feasibility_restoration_attempted ?
        "restored-solve-rejected" : "solve-rejected");
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
  std::optional<mpcc_rate_resolved_dynamic_obstacle::Request>
    physical_dynamic_sqp_request_template;
  if (snapshot.dynamic_obstacle_refinement_active) {
    const auto & source_context = snapshot.identity.source_context;
    if (
      !source_context.dynamic_obstacle_constraint_active ||
      source_context.dynamic_obstacle_id.empty() ||
      source_context.dynamic_obstacle_generation == 0U ||
      source_context.dynamic_obstacle_side_sign !=
      snapshot.dynamic_obstacle_pass_side_sign)
    {
      result.outcome = Outcome::AssemblyRejected;
      result.solved = false;
      result.detail =
        "dynamic obstacle refinement has no matching problem identity";
      return finish();
    }
    mpcc_rate_resolved_dynamic_obstacle::Request dynamic_request;
    dynamic_request.active = true;
    dynamic_request.pass_side_sign =
      snapshot.dynamic_obstacle_pass_side_sign;
    if (snapshot.dynamic_obstacle_forced_first_pass_side_stage >= 0) {
      dynamic_request.forced_first_pass_side_stage =
        snapshot.dynamic_obstacle_forced_first_pass_side_stage;
    }
    if (snapshot.dynamic_obstacle_forced_first_ahead_stage >= 0) {
      dynamic_request.forced_first_ahead_stage =
        snapshot.dynamic_obstacle_forced_first_ahead_stage;
    }
    if (snapshot.dynamic_obstacle_forced_first_pass_side_stage >= 0) {
      dynamic_request.forced_constraint_fraction =
        snapshot.dynamic_obstacle_forced_constraint_fraction;
    }
    if (snapshot.dynamic_obstacle_forced_diagonal_start_stage >= 0) {
      dynamic_request.forced_diagonal_start_stage =
        snapshot.dynamic_obstacle_forced_diagonal_start_stage;
      dynamic_request.forced_diagonal_full_side_stage =
        snapshot.dynamic_obstacle_forced_diagonal_full_side_stage;
    }
    if (snapshot.replay_world.has_value()) {
      const auto & world = snapshot.replay_world.value();
      if (
        !world.current || world.observation_generation == 0U ||
        !source_context.dynamic_obstacle_constraint_active ||
        source_context.dynamic_obstacle_id.empty() ||
        world.observation_generation !=
        source_context.dynamic_obstacle_generation)
      {
        result.outcome = Outcome::AssemblyRejected;
        result.solved = false;
        std::ostringstream detail;
        detail << "physical obstacle world does not match problem identity"
               << ": expected_id="
               << (source_context.dynamic_obstacle_id.empty() ?
               "none" : source_context.dynamic_obstacle_id)
               << ", expected_generation="
               << source_context.dynamic_obstacle_generation
               << ", observed_generation=" << world.observation_generation
               << ", constraint_active="
               << source_context.dynamic_obstacle_constraint_active;
        result.detail = detail.str();
        return finish();
      }
      const auto target = std::find_if(
        world.obstacles.begin(), world.obstacles.end(),
        [&snapshot](const ReplayDynamicObstacle & obstacle) {
          return obstacle.id ==
                 snapshot.identity.source_context.dynamic_obstacle_id;
        });
      if (target == world.obstacles.end()) {
        result.outcome = Outcome::AssemblyRejected;
        result.solved = false;
        result.detail =
          "physical obstacle guidance target is absent from replay world";
        return finish();
      }
      if (
        target->observation_generation != world.observation_generation ||
        !world.physical_footprint.valid() || !std::isfinite(target->radius_m) ||
        target->radius_m < 0.0)
      {
        result.outcome = Outcome::AssemblyRejected;
        result.solved = false;
        result.detail = "physical obstacle geometry provenance is invalid";
        return finish();
      }
      const auto & footprint = world.physical_footprint;
      const auto geometry =
        mpcc_rate_resolved_dynamic_obstacle::PhysicalSeparationGeometry{
        footprint.front_extent_m, footprint.rear_extent_m,
        footprint.left_extent_m, footprint.right_extent_m,
        footprint.margin_m, target->radius_m};
      dynamic_request.physical_separation_geometry = geometry;
      if (snapshot.dynamic_obstacle_forced_physical_diagonal) {
        dynamic_request.forced_physical_separation_geometry = geometry;
        dynamic_request.physical_separation_geometry.reset();
      }
    } else if (snapshot.dynamic_obstacle_forced_physical_diagonal) {
      result.outcome = Outcome::AssemblyRejected;
      result.solved = false;
      result.detail =
        "physical obstacle guidance requires immutable replay world";
      return finish();
    }
    dynamic_request.stages = snapshot.dynamic_obstacle_stages;
    dynamic_request.wall_only_problem = adapted->problem;
    dynamic_request.wall_only_primal = outcome.result->primal;
    dynamic_request.constraint_target_problem = broad_problem_before_wall;
    dynamic_request.separation_tolerance_m =
      solver_.physical_constraint_tolerance().absolute;
    physical_dynamic_sqp_request_template = dynamic_request;
    const auto refinement =
      mpcc_rate_resolved_dynamic_obstacle::refine(
      dynamic_request);
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
    result.dynamic_obstacle_ahead_row_count = refinement.ahead_row_count;
    result.dynamic_obstacle_diagonal_row_count =
      refinement.diagonal_row_count;
    result.dynamic_obstacle_physical_axis_support_applied =
      refinement.physical_axis_support_applied;
    result.dynamic_obstacle_physical_diagonal_guidance_applied =
      refinement.physical_diagonal_guidance_applied;
    result.dynamic_obstacle_forced_constraint_fraction =
      refinement.forced_constraint_fraction;
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
    // The target problem has a different row set from the wall witness.  Its
    // primal seed may borrow controls, but state and dual values are rebuilt
    // from the target problem's own equalities and row count.
    auto warm_start = build_current_problem_bootstrap(
      refinement.problem.value(),
      static_cast<std::size_t>(refined_assembled->lower_bound.size()),
      &outcome.result->primal);
    if (!warm_start.has_value()) {
      result.outcome = Outcome::AssemblyRejected;
      result.solved = false;
      result.detail =
        "rate-resolved dynamic-obstacle current-problem bootstrap rejected";
      return finish();
    }
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

    if (coupled_wall_obstacle_refinement) {
      // The dynamic-aware provisional trajectory is allowed to brake or
      // change progress.  Rebuild wall segments around that trajectory, while
      // retaining the dynamic rows, and solve the actual joint problem that
      // will be certified for publication.
      auto joint_refinement = build_progress_wall_refinement(
        snapshot, adapted->problem, outcome.result->primal,
        solver_.physical_constraint_tolerance(), &wall_refinement_cache_,
        wall_bucket_audit_mode);
      accumulate_wall_cache(joint_refinement.physical);
      result.progress_wall_refinement_reason =
        joint_refinement.resolution.reason;
      result.progress_wall_refinement_aligned_stage_count =
        joint_refinement.resolution.aligned_stage_count;
      result.progress_wall_refinement_out_of_range_stage_count =
        joint_refinement.resolution.out_of_range_stage_count;
      result.progress_wall_refinement_first_failure_stage =
        joint_refinement.resolution.first_failure_stage;
      result.progress_wall_refinement_maximum_mismatch_m =
        joint_refinement.resolution.maximum_progress_mismatch_m;
      result.progress_wall_refinement_applied =
        joint_refinement.request.has_value();
      result.physical_wall_refinement_reason =
        joint_refinement.physical.reason;
      result.physical_wall_refinement_applied =
        joint_refinement.physical.applied;
      result.physical_wall_refinement_first_failure_stage =
        joint_refinement.physical.first_failure_stage;
      result.physical_wall_refinement_checked_pose_count =
        joint_refinement.physical.checked_pose_count;
      if (!joint_refinement.request.has_value()) {
        result.outcome = Outcome::AssemblyRejected;
        result.solved = false;
        result.detail =
          "rate-resolved coupled wall/obstacle refinement rejected";
        return finish();
      }
      auto joint_assembled = mpcc_rate_resolved_problem::assemble(
        joint_refinement.request.value());
      if (!joint_assembled.has_value()) {
        result.outcome = Outcome::AssemblyRejected;
        result.solved = false;
        result.detail =
          "rate-resolved coupled wall/obstacle QP assembly rejected";
        return finish();
      }
      auto joint_warm_start = build_current_problem_bootstrap(
        joint_refinement.request.value(),
        static_cast<std::size_t>(joint_assembled->lower_bound.size()),
        &outcome.result->primal);
      if (!joint_warm_start.has_value()) {
        result.outcome = Outcome::AssemblyRejected;
        result.solved = false;
        result.detail =
          "rate-resolved coupled wall/obstacle bootstrap rejected";
        return finish();
      }
      if (wall_bucket_phase_one_enabled) {
        // Preserve the Phase-I meaning through the final coupled
        // wall/obstacle refinement. The unchanged hard rows and exact proofs,
        // not the racing objective's dual convergence, decide feasibility.
        joint_assembled->quadratic_cost.setIdentity();
        joint_assembled->linear_cost = -outcome.result->primal;
      }
      auto joint_outcome = solver_.solve(
        joint_assembled->quadratic_cost, joint_assembled->constraints,
        joint_assembled->linear_cost, joint_assembled->lower_bound,
        joint_assembled->upper_bound, joint_warm_start,
        joint_assembled->variable_scaling);
      bool joint_bucket_phase_one_solved = false;
      if (
        wall_bucket_phase_one_enabled &&
        joint_outcome.result.has_value() &&
        joint_outcome.result->primal.allFinite())
      {
        joint_bucket_phase_one_solved = true;
        // The Phase-I matrices were installed immediately above. Reassemble
        // the same immutable candidate to restore its racing objective while
        // retaining the Phase-I feasible primal as a cold-dual warm start.
        joint_assembled = mpcc_rate_resolved_problem::assemble(
          joint_refinement.request.value());
        if (joint_assembled.has_value()) {
          joint_warm_start = persistent_osqp::WarmStart{
            joint_outcome.result->primal,
            Eigen::VectorXd::Zero(joint_assembled->lower_bound.size())};
          joint_outcome = solver_.solve(
            joint_assembled->quadratic_cost, joint_assembled->constraints,
            joint_assembled->linear_cost, joint_assembled->lower_bound,
            joint_assembled->upper_bound, joint_warm_start,
            joint_assembled->variable_scaling);
        } else {
          result.outcome = Outcome::AssemblyRejected;
          result.solved = false;
          result.detail =
            "rate-resolved coupled racing QP reassembly rejected";
          return finish();
        }
      }
      result.solver = joint_outcome.telemetry;
      if (!joint_outcome.result.has_value()) {
        result.outcome = Outcome::SolveRejected;
        result.solved = false;
        result.detail =
          std::string{"rate-resolved coupled wall/obstacle QP rejected: "} +
          joint_outcome.failure_detail;
        if (wall_bucket_audit_mode.has_value()) {
          result.detail += std::string{", wall_bucket_phase_one="} +
            (wall_bucket_phase_one_enabled ?
            (joint_bucket_phase_one_solved ? "solved" : "rejected") :
            "disabled");
        }
        if (joint_outcome.constraint_failure.has_value()) {
          const auto semantic = mpcc_rate_resolved_problem::decode_row(
            joint_outcome.constraint_failure->row,
            snapshot.request.horizon_steps,
            joint_refinement.request->steering_rate_prefix_bounds.has_value(),
            joint_refinement.request->progress_aligned_wall_constraints.has_value(),
            static_cast<int>(
              joint_refinement.request->swept_lateral_wall_constraints.size()),
            &joint_refinement.request->dynamic_obstacle_constraints);
          if (semantic.valid) {
            result.detail += std::string{", row_semantic="} +
              mpcc_rate_resolved_problem::row_kind_name(semantic.kind) +
              "/stage=" + std::to_string(semantic.stage) +
              "/element=" + std::to_string(semantic.element);
          }
        }
        capture_failure(
          mpcc_architecture_snapshot::PipelineStage::WallRefinement,
          joint_refinement.request.value(), joint_assembled.value(),
          joint_warm_start, joint_outcome, "coupled-solve-rejected");
        return finish();
      }
      if (!joint_outcome.result->primal.allFinite()) {
        result.outcome = Outcome::NonfiniteResult;
        result.solved = false;
        result.finite = false;
        result.detail =
          "rate-resolved coupled wall/obstacle solver returned non-finite primal";
        return finish();
      }
      adapted->problem = std::move(joint_refinement.request.value());
      assembled = std::move(joint_assembled);
      outcome = std::move(joint_outcome);
      last_refinement_warm_start = std::move(joint_warm_start);
      result.progress_wall_refinement_solved = true;
      result.physical_wall_refinement_solved =
        !snapshot.physical_wall_refinement_active ||
        result.physical_wall_refinement_applied;
    }
  }

  // Architecture audit only: the normal path above performs one obstacle
  // convexification and its later physical correction rebuilds vehicle
  // dynamics only.  This bounded outer loop tests the missing formulation:
  // dynamics, oriented obstacle support and wall rows are rebuilt around one
  // common latest primal before every solve.  evaluate() can never request
  // this branch, so it has no production authority or fallback semantics.
  if (physical_dynamic_sqp_audit_iteration_count > 0U) {
    if (
      !snapshot.dynamic_obstacle_refinement_active ||
      !result.dynamic_obstacle_refinement_solved ||
      !snapshot.replay_world.has_value() ||
      !physical_dynamic_sqp_request_template.has_value() ||
      (!physical_dynamic_sqp_request_template->
      physical_separation_geometry.has_value() &&
      !physical_dynamic_sqp_request_template->
      forced_physical_separation_geometry.has_value()))
    {
      result.outcome = Outcome::AssemblyRejected;
      result.solved = false;
      result.physical_dynamic_sqp_audit_detail =
        "physical dynamic SQP audit requires a solved dynamic candidate";
      result.detail = result.physical_dynamic_sqp_audit_detail;
      return finish();
    }
    result.physical_dynamic_sqp_audit_applied = true;
    for (std::size_t iteration = 0U;
      iteration < physical_dynamic_sqp_audit_iteration_count; ++iteration)
    {
      // Rebuild from the same broad semantic problem on every iteration.
      // Reusing adapted->problem here would carry the preceding narrow wall
      // buckets and swept rows into the next tangent, turning successive
      // convexification into cumulative constraint intersection.
      auto iteration_problem = broad_problem_before_wall.has_value() ?
        broad_problem_before_wall.value() : adapted->problem;
      const auto dynamics =
        mpcc_rate_resolved_adapter::relinearize_around_primal(
        snapshot.request, outcome.result->primal, iteration_problem);
      if (!dynamics.applied) {
        result.outcome = Outcome::AssemblyRejected;
        result.solved = false;
        result.physical_dynamic_sqp_audit_detail =
          std::string{"dynamic-sqp dynamics rejected/"} +
          mpcc_rate_resolved_adapter::to_string(dynamics.reason) +
          "/stage=" + std::to_string(dynamics.stage);
        result.detail = result.physical_dynamic_sqp_audit_detail;
        return finish();
      }

      auto dynamic_request =
        physical_dynamic_sqp_request_template.value();
      dynamic_request.wall_only_problem = iteration_problem;
      dynamic_request.constraint_target_problem = iteration_problem;
      dynamic_request.wall_only_primal = outcome.result->primal;
      const auto dynamic_refinement =
        mpcc_rate_resolved_dynamic_obstacle::refine(dynamic_request);
      if (!dynamic_refinement.problem.has_value()) {
        result.outcome = Outcome::AssemblyRejected;
        result.solved = false;
        result.physical_dynamic_sqp_audit_detail =
          std::string{"dynamic-sqp obstacle rows rejected/"} +
          mpcc_rate_resolved_dynamic_obstacle::to_string(
          dynamic_refinement.reason);
        result.detail = result.physical_dynamic_sqp_audit_detail;
        return finish();
      }
      iteration_problem = dynamic_refinement.problem.value();
      result.dynamic_obstacle_resolved_side_sign =
        dynamic_refinement.resolved_side_sign;
      result.dynamic_obstacle_first_pass_side_stage =
        dynamic_refinement.first_pass_side_stage;
      result.dynamic_obstacle_stay_behind_row_count =
        dynamic_refinement.stay_behind_row_count;
      result.dynamic_obstacle_pass_side_row_count =
        dynamic_refinement.pass_side_row_count;
      result.dynamic_obstacle_ahead_row_count =
        dynamic_refinement.ahead_row_count;
      result.dynamic_obstacle_diagonal_row_count =
        dynamic_refinement.diagonal_row_count;
      result.dynamic_obstacle_physical_axis_support_applied =
        dynamic_refinement.physical_axis_support_applied;
      result.dynamic_obstacle_physical_diagonal_guidance_applied =
        dynamic_refinement.physical_diagonal_guidance_applied;

      if (snapshot.progress_aligned_wall_refinement_active) {
        auto wall_refinement = build_progress_wall_refinement(
          snapshot, iteration_problem, outcome.result->primal,
          solver_.physical_constraint_tolerance(), &wall_refinement_cache_);
        accumulate_wall_cache(wall_refinement.physical);
        if (!wall_refinement.request.has_value()) {
          result.outcome = Outcome::AssemblyRejected;
          result.solved = false;
          result.physical_dynamic_sqp_audit_detail =
            std::string{"dynamic-sqp wall rows rejected/progress="} +
            mpcc_progress::progress_aligned_wall_bounds_reason_name(
            wall_refinement.resolution.reason) + "/physical=" +
            mpcc_rate_resolved_wall_refinement::to_string(
            wall_refinement.physical.reason);
          result.detail = result.physical_dynamic_sqp_audit_detail;
          return finish();
        }
        iteration_problem = std::move(wall_refinement.request.value());
      }

      auto iteration_assembled =
        mpcc_rate_resolved_problem::assemble(iteration_problem);
      if (!iteration_assembled.has_value()) {
        result.outcome = Outcome::AssemblyRejected;
        result.solved = false;
        result.physical_dynamic_sqp_audit_detail =
          "dynamic-sqp assembly rejected";
        result.detail = result.physical_dynamic_sqp_audit_detail;
        return finish();
      }
      auto iteration_warm_start = build_current_problem_bootstrap(
        iteration_problem,
        static_cast<std::size_t>(
          iteration_assembled->lower_bound.size()),
        &outcome.result->primal);
      if (!iteration_warm_start.has_value()) {
        result.outcome = Outcome::AssemblyRejected;
        result.solved = false;
        result.physical_dynamic_sqp_audit_detail =
          "dynamic-sqp current-problem bootstrap rejected";
        result.detail = result.physical_dynamic_sqp_audit_detail;
        return finish();
      }
      auto iteration_outcome = solver_.solve(
        iteration_assembled->quadratic_cost,
        iteration_assembled->constraints,
        iteration_assembled->linear_cost,
        iteration_assembled->lower_bound,
        iteration_assembled->upper_bound,
        iteration_warm_start,
        iteration_assembled->variable_scaling);
      result.solver = iteration_outcome.telemetry;
      if (!iteration_outcome.result.has_value()) {
        result.outcome = Outcome::SolveRejected;
        result.solved = false;
        result.physical_dynamic_sqp_audit_detail =
          std::string{"dynamic-sqp solve rejected/"} +
          iteration_outcome.failure_detail;
        if (iteration_outcome.constraint_failure.has_value()) {
          const auto semantic = mpcc_rate_resolved_problem::decode_row(
            iteration_outcome.constraint_failure->row,
            snapshot.request.horizon_steps,
            iteration_problem.steering_rate_prefix_bounds.has_value(),
            iteration_problem.progress_aligned_wall_constraints.has_value(),
            static_cast<int>(
              iteration_problem.swept_lateral_wall_constraints.size()),
            &iteration_problem.dynamic_obstacle_constraints);
          if (semantic.valid) {
            result.physical_dynamic_sqp_audit_detail +=
              std::string{"/row="} +
              mpcc_rate_resolved_problem::row_kind_name(semantic.kind) +
              "/stage=" + std::to_string(semantic.stage) +
              "/element=" + std::to_string(semantic.element);
          }
        }
        result.detail = result.physical_dynamic_sqp_audit_detail;
        capture_failure(
          mpcc_architecture_snapshot::PipelineStage::
          PostRefinementLinearization,
          iteration_problem, iteration_assembled.value(),
          iteration_warm_start, iteration_outcome,
          "physical-dynamic-sqp-audit-solve-rejected");
        return finish();
      }
      if (!iteration_outcome.result->primal.allFinite()) {
        result.outcome = Outcome::NonfiniteResult;
        result.solved = false;
        result.finite = false;
        result.physical_dynamic_sqp_audit_detail =
          "dynamic-sqp solver returned non-finite primal";
        result.detail = result.physical_dynamic_sqp_audit_detail;
        return finish();
      }
      adapted->problem = std::move(iteration_problem);
      assembled = std::move(iteration_assembled);
      outcome = std::move(iteration_outcome);
      last_refinement_warm_start = std::move(iteration_warm_start);
      ++result.physical_dynamic_sqp_audit_count;
    }
    result.physical_dynamic_sqp_audit_solved = true;
    result.physical_dynamic_sqp_audit_detail =
      "bounded dynamics/obstacle/wall SQP solved";
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
  auto feedback_preparation =
    std::make_shared<LatestStateFeedbackPreparation>();
  feedback_preparation->snapshot = snapshot;
  feedback_preparation->final_problem = adapted->problem;
  feedback_preparation->prepared_primal = outcome.result->primal;
  result.latest_state_feedback_preparation = std::move(feedback_preparation);
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
        result.physical_wall_refinement_checked_pose_count
             << "/cache_hits=" <<
        result.physical_wall_refinement_cache_hit_count
             << "/cache_misses=" <<
        result.physical_wall_refinement_cache_miss_count
             << "/cache_scanned=" <<
        result.physical_wall_refinement_cache_scanned_pose_count
             << "/lag_pose_box=" <<
        (result.physical_wall_lag_pose_box_applied ? 1 : 0)
             << "/heading_pose_box=" <<
        (result.physical_wall_heading_pose_box_applied ? 1 : 0);
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
  if (result.wall_feasibility_restoration_requested) {
    result.detail += std::string{", wall_restoration="} +
      (result.wall_feasibility_restoration_attempted ? "attempted" :
      "not-needed") +
      "/seed=" +
      (result.wall_feasibility_restoration_seed_solved ? "1" : "0") +
      "/sqp_count=" +
      std::to_string(result.wall_feasibility_restoration_sqp_count) +
      "/final_built=" +
      (result.wall_feasibility_restoration_final_refinement_built ? "1" :
      "0") +
      "/final_solved=" +
      (result.wall_feasibility_restoration_final_solved ? "1" : "0") +
      "/detail=" + result.wall_feasibility_restoration_detail;
  }
  if (result.physical_dynamic_sqp_audit_requested) {
    result.detail += std::string{", physical_dynamic_sqp_audit="} +
      (result.physical_dynamic_sqp_audit_applied ? "applied" :
      "not-applied") +
      "/solved=" +
      (result.physical_dynamic_sqp_audit_solved ? "1" : "0") +
      "/limit=" +
      std::to_string(result.physical_dynamic_sqp_audit_iteration_limit) +
      "/count=" +
      std::to_string(result.physical_dynamic_sqp_audit_count) +
      "/detail=" + result.physical_dynamic_sqp_audit_detail;
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
