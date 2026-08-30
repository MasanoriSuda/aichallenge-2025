#include <multi_purpose_mpc_ros/race_mpcc_foundation.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace multi_purpose_mpc_ros::race_mpcc_foundation
{

const char * homotopy_name(const Homotopy homotopy) noexcept
{
  switch (homotopy) {
    case Homotopy::Left:
      return "left";
    case Homotopy::Right:
      return "right";
    case Homotopy::Hold:
      return "hold";
    case Homotopy::Return:
      return "return";
    case Homotopy::None:
      return "none";
  }
  return "unknown";
}

const char * target_provenance_stage_name(
  const TargetProvenanceStage stage) noexcept
{
  switch (stage) {
    case TargetProvenanceStage::None:
      return "none";
    case TargetProvenanceStage::Observed:
      return "observed";
    case TargetProvenanceStage::Locked:
      return "locked";
  }
  return "unknown";
}

const char * target_provenance_reject_reason_name(
  const TargetProvenanceRejectReason reason) noexcept
{
  switch (reason) {
    case TargetProvenanceRejectReason::None:
      return "none";
    case TargetProvenanceRejectReason::InvalidExpected:
      return "invalid-expected";
    case TargetProvenanceRejectReason::InvalidCurrent:
      return "invalid-current";
    case TargetProvenanceRejectReason::TargetMismatch:
      return "target-mismatch";
    case TargetProvenanceRejectReason::SourceRegression:
      return "source-regression";
    case TargetProvenanceRejectReason::ReceiptRegression:
      return "receipt-regression";
    case TargetProvenanceRejectReason::GenerationRegression:
      return "generation-regression";
    case TargetProvenanceRejectReason::StageRegression:
      return "stage-regression";
    case TargetProvenanceRejectReason::ProgressDelta:
      return "progress-delta";
    case TargetProvenanceRejectReason::LateralDelta:
      return "lateral-delta";
  }
  return "unknown";
}

const char * return_transition_admission_reason_name(
  const ReturnTransitionAdmissionReason reason) noexcept
{
  switch (reason) {
    case ReturnTransitionAdmissionReason::Inactive:
      return "inactive";
    case ReturnTransitionAdmissionReason::GeometricPreflightUnavailable:
      return "geometric-preflight-unavailable";
    case ReturnTransitionAdmissionReason::ProposalIncomplete:
      return "proposal-incomplete";
    case ReturnTransitionAdmissionReason::IntentMismatch:
      return "intent-mismatch";
    case ReturnTransitionAdmissionReason::TargetMismatch:
      return "target-mismatch";
    case ReturnTransitionAdmissionReason::MissionGenerationMismatch:
      return "mission-generation-mismatch";
    case ReturnTransitionAdmissionReason::SideMismatch:
      return "side-mismatch";
    case ReturnTransitionAdmissionReason::CurrentWorldRejected:
      return "current-world-rejected";
    case ReturnTransitionAdmissionReason::Admitted:
      return "admitted";
  }
  return "unknown";
}

ReturnTransitionAdmission resolve_return_transition_admission(
  const ReturnTransitionAdmissionRequest & request) noexcept
{
  const auto reject = [](const ReturnTransitionAdmissionReason reason) {
      return ReturnTransitionAdmission{false, reason};
    };
  if (!request.pass_active) {
    return reject(ReturnTransitionAdmissionReason::Inactive);
  }
  if (!request.geometric_preflight_valid) {
    return reject(
      ReturnTransitionAdmissionReason::GeometricPreflightUnavailable);
  }
  if (!request.proposal_complete) {
    return reject(ReturnTransitionAdmissionReason::ProposalIncomplete);
  }
  if (
    request.proposal_intent !=
    mpcc_execution_contract::ControlIntent::Return)
  {
    return reject(ReturnTransitionAdmissionReason::IntentMismatch);
  }
  if (
    request.current_target_id.empty() ||
    request.proposal_target_id != request.current_target_id)
  {
    return reject(ReturnTransitionAdmissionReason::TargetMismatch);
  }
  if (
    request.current_mission_generation == 0U ||
    request.proposal_mission_generation != request.current_mission_generation)
  {
    return reject(
      ReturnTransitionAdmissionReason::MissionGenerationMismatch);
  }
  if (
    (request.current_side_sign != -1 && request.current_side_sign != 1) ||
    request.proposal_side_sign != request.current_side_sign)
  {
    return reject(ReturnTransitionAdmissionReason::SideMismatch);
  }
  if (!request.current_world_certified) {
    return reject(ReturnTransitionAdmissionReason::CurrentWorldRejected);
  }
  return {true, ReturnTransitionAdmissionReason::Admitted};
}

const char * pass_transition_admission_reason_name(
  const PassTransitionAdmissionReason reason) noexcept
{
  switch (reason) {
    case PassTransitionAdmissionReason::Inactive: return "inactive";
    case PassTransitionAdmissionReason::DynamicHorizonUnavailable:
      return "dynamic-horizon-unavailable";
    case PassTransitionAdmissionReason::PhysicalHorizonUnavailable:
      return "physical-horizon-unavailable";
    case PassTransitionAdmissionReason::ProposalIncomplete:
      return "proposal-incomplete";
    case PassTransitionAdmissionReason::IntentMismatch:
      return "intent-mismatch";
    case PassTransitionAdmissionReason::TargetMismatch:
      return "target-mismatch";
    case PassTransitionAdmissionReason::MissionGenerationMismatch:
      return "mission-generation-mismatch";
    case PassTransitionAdmissionReason::SideMismatch:
      return "side-mismatch";
    case PassTransitionAdmissionReason::CurrentWorldRejected:
      return "current-world-rejected";
    case PassTransitionAdmissionReason::Admitted: return "admitted";
  }
  return "unknown";
}

PassTransitionAdmission resolve_pass_transition_admission(
  const PassTransitionAdmissionRequest & request) noexcept
{
  const auto reject = [](const PassTransitionAdmissionReason reason) {
      return PassTransitionAdmission{false, reason};
    };
  if (!request.shiftout_complete) {
    return reject(PassTransitionAdmissionReason::Inactive);
  }
  if (!request.dynamic_horizon_available) {
    return reject(PassTransitionAdmissionReason::DynamicHorizonUnavailable);
  }
  if (!request.physical_horizon_available) {
    return reject(PassTransitionAdmissionReason::PhysicalHorizonUnavailable);
  }
  if (!request.proposal_complete) {
    return reject(PassTransitionAdmissionReason::ProposalIncomplete);
  }
  if (
    request.proposal_intent !=
    mpcc_execution_contract::ControlIntent::Pass)
  {
    return reject(PassTransitionAdmissionReason::IntentMismatch);
  }
  if (
    request.current_target_id.empty() ||
    request.proposal_target_id != request.current_target_id)
  {
    return reject(PassTransitionAdmissionReason::TargetMismatch);
  }
  if (
    request.current_mission_generation == 0U ||
    request.proposal_mission_generation != request.current_mission_generation)
  {
    return reject(PassTransitionAdmissionReason::MissionGenerationMismatch);
  }
  if (
    (request.current_side_sign != -1 && request.current_side_sign != 1) ||
    request.proposal_side_sign != request.current_side_sign)
  {
    return reject(PassTransitionAdmissionReason::SideMismatch);
  }
  if (!request.current_world_certified) {
    return reject(PassTransitionAdmissionReason::CurrentWorldRejected);
  }
  return {true, PassTransitionAdmissionReason::Admitted};
}

namespace
{

bool finite_provenance(const TargetProvenance & provenance) noexcept
{
  return provenance.valid && !provenance.target_id.empty() &&
         std::isfinite(provenance.source_stamp_sec) &&
         std::isfinite(provenance.receipt_sec) &&
         std::isfinite(provenance.course_progress_m) &&
         std::isfinite(provenance.course_lateral_m) &&
         provenance.observation_generation > 0U &&
         provenance.stage != TargetProvenanceStage::None;
}

double circular_delta(
  const double current, const double expected,
  const bool circular, const double path_length_m) noexcept
{
  double delta = current - expected;
  if (!circular || !std::isfinite(path_length_m) || path_length_m <= 0.0) {
    return delta;
  }
  while (delta > 0.5 * path_length_m) {
    delta -= path_length_m;
  }
  while (delta < -0.5 * path_length_m) {
    delta += path_length_m;
  }
  return delta;
}

}  // namespace

TargetProvenanceValidation validate_target_provenance(
  const TargetProvenanceValidationRequest & request) noexcept
{
  TargetProvenanceValidation result;
  if (!finite_provenance(request.expected)) {
    result.reject_reason = TargetProvenanceRejectReason::InvalidExpected;
    return result;
  }
  if (!finite_provenance(request.current)) {
    result.reject_reason = TargetProvenanceRejectReason::InvalidCurrent;
    return result;
  }
  if (request.expected.target_id != request.current.target_id) {
    result.reject_reason = TargetProvenanceRejectReason::TargetMismatch;
    return result;
  }
  if (request.current.source_stamp_sec + 1e-9 < request.expected.source_stamp_sec) {
    result.reject_reason = TargetProvenanceRejectReason::SourceRegression;
    return result;
  }
  if (request.current.receipt_sec + 1e-9 < request.expected.receipt_sec) {
    result.reject_reason = TargetProvenanceRejectReason::ReceiptRegression;
    return result;
  }
  if (request.current.observation_generation < request.expected.observation_generation) {
    result.reject_reason = TargetProvenanceRejectReason::GenerationRegression;
    return result;
  }
  if (
    request.expected.stage == TargetProvenanceStage::Locked &&
    request.current.stage != TargetProvenanceStage::Locked)
  {
    result.reject_reason = TargetProvenanceRejectReason::StageRegression;
    return result;
  }
  result.same_observation =
    request.current.observation_generation == request.expected.observation_generation;
  result.progress_delta_m = circular_delta(
    request.current.course_progress_m, request.expected.course_progress_m,
    request.circular, request.path_length_m);
  result.lateral_delta_m =
    request.current.course_lateral_m - request.expected.course_lateral_m;
  if (
    !std::isfinite(result.progress_delta_m) ||
    result.progress_delta_m < -std::max(0.0, request.maximum_backward_progress_m) - 1e-9 ||
    result.progress_delta_m > std::max(0.0, request.maximum_forward_progress_m) + 1e-9)
  {
    result.reject_reason = TargetProvenanceRejectReason::ProgressDelta;
    return result;
  }
  if (
    !std::isfinite(result.lateral_delta_m) ||
    std::abs(result.lateral_delta_m) >
    std::max(0.0, request.maximum_lateral_change_m) + 1e-9)
  {
    result.reject_reason = TargetProvenanceRejectReason::LateralDelta;
    return result;
  }
  result.valid = true;
  result.reject_reason = TargetProvenanceRejectReason::None;
  return result;
}

const char * exact_physical_execution_trajectory_reason_name(
  const ExactPhysicalExecutionTrajectoryReason reason) noexcept
{
  switch (reason) {
    case ExactPhysicalExecutionTrajectoryReason::Accepted: return "accepted";
    case ExactPhysicalExecutionTrajectoryReason::TooFewStages: return "too-few-stages";
    case ExactPhysicalExecutionTrajectoryReason::InvalidProgressOrigin:
      return "invalid-progress-origin";
    case ExactPhysicalExecutionTrajectoryReason::InvalidProgressRegressionTolerance:
      return "invalid-progress-regression-tolerance";
    case ExactPhysicalExecutionTrajectoryReason::InvalidVelocityLowerBoundTolerance:
      return "invalid-velocity-lower-bound-tolerance";
    case ExactPhysicalExecutionTrajectoryReason::InvalidLateralBoundTolerance:
      return "invalid-lateral-bound-tolerance";
    case ExactPhysicalExecutionTrajectoryReason::InvalidMinimumLateralReserve:
      return "invalid-minimum-lateral-reserve";
    case ExactPhysicalExecutionTrajectoryReason::TimeShapeMismatch:
      return "time-shape-mismatch";
    case ExactPhysicalExecutionTrajectoryReason::LateralShapeMismatch:
      return "lateral-shape-mismatch";
    case ExactPhysicalExecutionTrajectoryReason::LagShapeMismatch:
      return "lag-shape-mismatch";
    case ExactPhysicalExecutionTrajectoryReason::HeadingShapeMismatch:
      return "heading-shape-mismatch";
    case ExactPhysicalExecutionTrajectoryReason::VelocityShapeMismatch:
      return "velocity-shape-mismatch";
    case ExactPhysicalExecutionTrajectoryReason::ProgressShapeMismatch:
      return "progress-shape-mismatch";
    case ExactPhysicalExecutionTrajectoryReason::LowerBoundShapeMismatch:
      return "lower-bound-shape-mismatch";
    case ExactPhysicalExecutionTrajectoryReason::UpperBoundShapeMismatch:
      return "upper-bound-shape-mismatch";
    case ExactPhysicalExecutionTrajectoryReason::InvalidElapsedTime:
      return "invalid-elapsed-time";
    case ExactPhysicalExecutionTrajectoryReason::InvalidPathDistance:
      return "invalid-path-distance";
    case ExactPhysicalExecutionTrajectoryReason::NonFiniteLateral:
      return "non-finite-lateral";
    case ExactPhysicalExecutionTrajectoryReason::NonFiniteLag: return "non-finite-lag";
    case ExactPhysicalExecutionTrajectoryReason::NonFiniteHeading:
      return "non-finite-heading";
    case ExactPhysicalExecutionTrajectoryReason::InvalidVelocity:
      return "invalid-velocity";
    case ExactPhysicalExecutionTrajectoryReason::ProgressRegressed:
      return "progress-regressed";
    case ExactPhysicalExecutionTrajectoryReason::InvalidLateralBounds:
      return "invalid-lateral-bounds";
  }
  return "unknown";
}

ExactPhysicalExecutionTrajectoryValidation
validate_exact_physical_execution_trajectory(
  const ExactPhysicalExecutionTrajectory & trajectory) noexcept
{
  const std::size_t stage_count = trajectory.path_distance_m.size();
  const auto reject = [](
      const ExactPhysicalExecutionTrajectoryReason reason,
      const int stage = -1,
      const double lateral_m = std::numeric_limits<double>::quiet_NaN(),
      const double lower_m = std::numeric_limits<double>::quiet_NaN(),
      const double upper_m = std::numeric_limits<double>::quiet_NaN()) {
      return ExactPhysicalExecutionTrajectoryValidation{
        false, reason, stage, lateral_m, lower_m, upper_m};
    };
  if (stage_count < 1U) {
    return reject(ExactPhysicalExecutionTrajectoryReason::TooFewStages);
  }
  if (!std::isfinite(trajectory.progress_origin_m)) {
    return reject(ExactPhysicalExecutionTrajectoryReason::InvalidProgressOrigin);
  }
  if (
    !std::isfinite(trajectory.progress_regression_tolerance_m) ||
    trajectory.progress_regression_tolerance_m < 0.0)
  {
    return reject(
      ExactPhysicalExecutionTrajectoryReason::InvalidProgressRegressionTolerance);
  }
  if (
    !std::isfinite(trajectory.velocity_lower_bound_tolerance_mps) ||
    trajectory.velocity_lower_bound_tolerance_mps < 0.0)
  {
    return reject(
      ExactPhysicalExecutionTrajectoryReason::InvalidVelocityLowerBoundTolerance);
  }
  if (
    !std::isfinite(trajectory.lateral_bound_tolerance_m) ||
    trajectory.lateral_bound_tolerance_m < 0.0)
  {
    return reject(
      ExactPhysicalExecutionTrajectoryReason::InvalidLateralBoundTolerance);
  }
  if (
    !std::isfinite(trajectory.minimum_lateral_bound_reserve_m) ||
    trajectory.minimum_lateral_bound_reserve_m < 0.0)
  {
    return reject(
      ExactPhysicalExecutionTrajectoryReason::InvalidMinimumLateralReserve);
  }
  if (trajectory.elapsed_time_sec.size() != stage_count) {
    return reject(ExactPhysicalExecutionTrajectoryReason::TimeShapeMismatch);
  }
  if (trajectory.lateral_m.size() != stage_count) {
    return reject(ExactPhysicalExecutionTrajectoryReason::LateralShapeMismatch);
  }
  if (trajectory.lag_m.size() != stage_count) {
    return reject(ExactPhysicalExecutionTrajectoryReason::LagShapeMismatch);
  }
  if (trajectory.heading_offset_rad.size() != stage_count) {
    return reject(ExactPhysicalExecutionTrajectoryReason::HeadingShapeMismatch);
  }
  if (trajectory.velocity_mps.size() != stage_count) {
    return reject(ExactPhysicalExecutionTrajectoryReason::VelocityShapeMismatch);
  }
  if (trajectory.progress_m.size() != stage_count) {
    return reject(ExactPhysicalExecutionTrajectoryReason::ProgressShapeMismatch);
  }
  if (trajectory.lateral_lower_m.size() != stage_count) {
    return reject(ExactPhysicalExecutionTrajectoryReason::LowerBoundShapeMismatch);
  }
  if (trajectory.lateral_upper_m.size() != stage_count) {
    return reject(ExactPhysicalExecutionTrajectoryReason::UpperBoundShapeMismatch);
  }
  double previous_distance_m = -std::numeric_limits<double>::infinity();
  double previous_progress_m = -std::numeric_limits<double>::infinity();
  double previous_elapsed_sec{};
  for (std::size_t stage = 0U; stage < stage_count; ++stage) {
    const double elapsed_sec = trajectory.elapsed_time_sec[stage];
    const double distance_m = trajectory.path_distance_m[stage];
    const double lateral_m = trajectory.lateral_m[stage];
    const double lag_m = trajectory.lag_m[stage];
    const double heading_rad = trajectory.heading_offset_rad[stage];
    const double velocity_mps = trajectory.velocity_mps[stage];
    const double progress_m = trajectory.progress_m[stage];
    const double lower_m = trajectory.lateral_lower_m[stage];
    const double upper_m = trajectory.lateral_upper_m[stage];
    const int stage_index = static_cast<int>(stage);
    if (
      !std::isfinite(elapsed_sec) || elapsed_sec <= previous_elapsed_sec)
    {
      return reject(
        ExactPhysicalExecutionTrajectoryReason::InvalidElapsedTime, stage_index);
    }
    if (!std::isfinite(distance_m) || distance_m <= previous_distance_m) {
      return reject(
        ExactPhysicalExecutionTrajectoryReason::InvalidPathDistance, stage_index);
    }
    if (!std::isfinite(lateral_m)) {
      return reject(
        ExactPhysicalExecutionTrajectoryReason::NonFiniteLateral, stage_index);
    }
    if (!std::isfinite(lag_m)) {
      return reject(ExactPhysicalExecutionTrajectoryReason::NonFiniteLag, stage_index);
    }
    if (!std::isfinite(heading_rad)) {
      return reject(
        ExactPhysicalExecutionTrajectoryReason::NonFiniteHeading, stage_index);
    }
    if (
      !std::isfinite(velocity_mps) ||
      velocity_mps < -trajectory.velocity_lower_bound_tolerance_mps)
    {
      return reject(
        ExactPhysicalExecutionTrajectoryReason::InvalidVelocity, stage_index);
    }
    if (
      !std::isfinite(progress_m) ||
      progress_m + trajectory.progress_regression_tolerance_m <
      previous_progress_m)
    {
      return reject(
        ExactPhysicalExecutionTrajectoryReason::ProgressRegressed, stage_index);
    }
    if (
      !std::isfinite(lower_m) || !std::isfinite(upper_m) || lower_m > upper_m ||
      lateral_m < lower_m - trajectory.lateral_bound_tolerance_m ||
      lateral_m > upper_m + trajectory.lateral_bound_tolerance_m)
    {
      return reject(
        ExactPhysicalExecutionTrajectoryReason::InvalidLateralBounds,
        stage_index, lateral_m, lower_m, upper_m);
    }
    previous_distance_m = distance_m;
    previous_progress_m = progress_m;
    previous_elapsed_sec = elapsed_sec;
  }
  return ExactPhysicalExecutionTrajectoryValidation{
    true, ExactPhysicalExecutionTrajectoryReason::Accepted, -1,
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()};
}

bool exact_physical_execution_trajectory_complete(
  const ExactPhysicalExecutionTrajectory & trajectory) noexcept
{
  return validate_exact_physical_execution_trajectory(trajectory).complete;
}

std::string format_shadow_decision(const ShadowDecision & decision)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(2)
         << "epoch=" << decision.context_epoch
         << ",target=" << (decision.target_id.empty() ? "none" : decision.target_id)
         << ",target_provenance=" << (decision.target_provenance.valid ? 1 : 0)
         << '/' << target_provenance_stage_name(decision.target_provenance.stage)
         << '/' << decision.target_provenance.observation_generation
         << '/' << decision.target_provenance.source_stamp_sec
         << '/' << decision.target_provenance.course_progress_m
         << '/' << decision.target_provenance.course_lateral_m
         << ",geometry=" << (decision.stage_geometry_valid ? "valid" : "invalid")
         << '/' << decision.stage_count << '/' << decision.horizon_distance_m << "m";
  for (const auto & candidate : decision.candidates) {
    stream << ',' << homotopy_name(candidate.homotopy) << '='
           << (candidate.attempted ? 1 : 0) << '/'
           << (candidate.feasible ? 1 : 0) << "/warm="
           << (candidate.warm_start_applied ? 1 : 0) << "/reset="
           << (candidate.solver_context_reset ? 1 : 0) << "/count="
           << candidate.solver_context_solve_count << "/obj="
           << candidate.objective << "/p=" << candidate.terminal_progress_m
           << "/v=" << candidate.terminal_velocity_mps << "/wall="
           << candidate.minimum_wall_reserve_m << "/solve=" << candidate.solve_ms
           << "ms/iter=" << candidate.iterations << "/reason=" << candidate.reason;
  }
  stream << ",selected=" << homotopy_name(decision.selected)
         << ",selection_reason=" << decision.selection_reason
         << ",authority=shadow";
  return stream.str();
}

const char * track_cruise_shadow_eligibility_reason_name(
  const TrackCruiseShadowEligibilityReason reason) noexcept
{
  switch (reason) {
    case TrackCruiseShadowEligibilityReason::Eligible:
      return "eligible";
    case TrackCruiseShadowEligibilityReason::LiveProgressAlreadyActive:
      return "live-progress-already-active";
    case TrackCruiseShadowEligibilityReason::TacticalSnapshot:
      return "tactical-snapshot";
    case TrackCruiseShadowEligibilityReason::IntentNotTrackCruise:
      return "intent-not-track-cruise";
  }
  return "unknown";
}

TrackCruiseShadowEligibility resolve_track_cruise_shadow_eligibility(
  const TrackCruiseShadowEligibilityRequest & request) noexcept
{
  TrackCruiseShadowEligibility result;
  if (request.live_progress_active) {
    result.reason = TrackCruiseShadowEligibilityReason::LiveProgressAlreadyActive;
    return result;
  }
  if (request.tactical_snapshot) {
    result.reason = TrackCruiseShadowEligibilityReason::TacticalSnapshot;
    return result;
  }
  if (
    request.intent != mpcc_execution_contract::ControlIntent::Track &&
    request.intent != mpcc_execution_contract::ControlIntent::Cruise)
  {
    result.reason = TrackCruiseShadowEligibilityReason::IntentNotTrackCruise;
    return result;
  }
  result.eligible = true;
  result.reason = TrackCruiseShadowEligibilityReason::Eligible;
  return result;
}

const char * rejoin_shadow_eligibility_reason_name(
  const RejoinShadowEligibilityReason reason) noexcept
{
  switch (reason) {
    case RejoinShadowEligibilityReason::Eligible:
      return "eligible";
    case RejoinShadowEligibilityReason::LiveProgressAlreadyActive:
      return "live-progress-already-active";
    case RejoinShadowEligibilityReason::TacticalSnapshot:
      return "tactical-snapshot";
    case RejoinShadowEligibilityReason::IntentNotRejoin:
      return "intent-not-rejoin";
  }
  return "unknown";
}

RejoinShadowEligibility resolve_rejoin_shadow_eligibility(
  const RejoinShadowEligibilityRequest & request) noexcept
{
  RejoinShadowEligibility result;
  if (request.live_progress_active) {
    result.reason = RejoinShadowEligibilityReason::LiveProgressAlreadyActive;
    return result;
  }
  if (request.tactical_snapshot) {
    result.reason = RejoinShadowEligibilityReason::TacticalSnapshot;
    return result;
  }
  if (request.intent != mpcc_execution_contract::ControlIntent::Rejoin) {
    result.reason = RejoinShadowEligibilityReason::IntentNotRejoin;
    return result;
  }
  result.eligible = true;
  result.reason = RejoinShadowEligibilityReason::Eligible;
  return result;
}

const char * follow_shadow_eligibility_reason_name(
  const FollowShadowEligibilityReason reason) noexcept
{
  switch (reason) {
    case FollowShadowEligibilityReason::Eligible:
      return "eligible";
    case FollowShadowEligibilityReason::LiveProgressAlreadyActive:
      return "live-progress-already-active";
    case FollowShadowEligibilityReason::TacticalSnapshot:
      return "tactical-snapshot";
    case FollowShadowEligibilityReason::IntentNotFollow:
      return "intent-not-follow";
    case FollowShadowEligibilityReason::NoCoherentFrontObservation:
      return "no-coherent-front-observation";
  }
  return "unknown";
}

FollowShadowEligibility resolve_follow_shadow_eligibility(
  const FollowShadowEligibilityRequest & request) noexcept
{
  FollowShadowEligibility result;
  if (request.live_progress_active) {
    result.reason = FollowShadowEligibilityReason::LiveProgressAlreadyActive;
    return result;
  }
  if (request.tactical_snapshot) {
    result.reason = FollowShadowEligibilityReason::TacticalSnapshot;
    return result;
  }
  if (request.intent != mpcc_execution_contract::ControlIntent::Follow) {
    result.reason = FollowShadowEligibilityReason::IntentNotFollow;
    return result;
  }
  if (!request.coherent_front_observation) {
    result.reason = FollowShadowEligibilityReason::NoCoherentFrontObservation;
    return result;
  }
  result.eligible = true;
  result.reason = FollowShadowEligibilityReason::Eligible;
  return result;
}

const char * overtake_canonical_fresh_shadow_eligibility_reason_name(
  const OvertakeCanonicalFreshShadowEligibilityReason reason) noexcept
{
  switch (reason) {
    case OvertakeCanonicalFreshShadowEligibilityReason::Eligible:
      return "eligible";
    case OvertakeCanonicalFreshShadowEligibilityReason::ProgressContouringInactive:
      return "progress-contouring-inactive";
    case OvertakeCanonicalFreshShadowEligibilityReason::IntentNotOvertakeExecution:
      return "intent-not-overtake-execution";
    case OvertakeCanonicalFreshShadowEligibilityReason::ExecutionContextUnavailable:
      return "execution-context-unavailable";
    case OvertakeCanonicalFreshShadowEligibilityReason::LateralBoundsInvalid:
      return "lateral-bounds-invalid";
  }
  return "unknown";
}

OvertakeCanonicalFreshShadowEligibility
resolve_overtake_canonical_fresh_shadow_eligibility(
  const OvertakeCanonicalFreshShadowEligibilityRequest & request) noexcept
{
  OvertakeCanonicalFreshShadowEligibility result;
  if (!request.progress_contouring_active) {
    result.reason = OvertakeCanonicalFreshShadowEligibilityReason::
      ProgressContouringInactive;
    return result;
  }
  if (
    request.intent != mpcc_execution_contract::ControlIntent::ShiftOut &&
    request.intent != mpcc_execution_contract::ControlIntent::Pass &&
    request.intent != mpcc_execution_contract::ControlIntent::Return)
  {
    result.reason = OvertakeCanonicalFreshShadowEligibilityReason::
      IntentNotOvertakeExecution;
    return result;
  }
  if (!request.execution_context_available) {
    result.reason = OvertakeCanonicalFreshShadowEligibilityReason::
      ExecutionContextUnavailable;
    return result;
  }
  if (!request.lateral_bounds_valid) {
    result.reason = OvertakeCanonicalFreshShadowEligibilityReason::
      LateralBoundsInvalid;
    return result;
  }
  result.eligible = true;
  result.reason = OvertakeCanonicalFreshShadowEligibilityReason::Eligible;
  return result;
}

FollowProductionAction resolve_follow_production_action(
  const mpcc_execution_contract::ControlIntent intent,
  const bool complete_canonical_selection,
  const mpcc_execution_contract::ControlIntent last_published_canonical_intent)
noexcept
{
  if (intent != mpcc_execution_contract::ControlIntent::Follow) {
    return FollowProductionAction::NotOwned;
  }
  if (complete_canonical_selection) {
    return FollowProductionAction::PublishCanonical;
  }
  return last_published_canonical_intent !=
         mpcc_execution_contract::ControlIntent::Follow ?
         FollowProductionAction::SolveTransitionAdmission :
         FollowProductionAction::EmergencyStop;
}

StopAuthorityAction resolve_stop_authority_action(
  const mpcc_execution_contract::ControlIntent intent) noexcept
{
  return intent == mpcc_execution_contract::ControlIntent::Stop ?
    StopAuthorityAction::EmergencyStop : StopAuthorityAction::NotOwned;
}

StopLateralAction resolve_stop_lateral_action(
  const StopLateralActionRequest & request) noexcept
{
  if (!std::isfinite(request.current_speed_mps)) {
    return StopLateralAction::Neutralize;
  }
  if (request.current_speed_mps <= 0.0) {
    return StopLateralAction::HoldAtRest;
  }
  return request.reference_path_target_available ?
    StopLateralAction::TrackReferencePath : StopLateralAction::Neutralize;
}

const char * stop_lateral_action_name(const StopLateralAction action) noexcept
{
  switch (action) {
    case StopLateralAction::HoldAtRest:
      return "hold-at-rest";
    case StopLateralAction::TrackReferencePath:
      return "track-reference-path";
    case StopLateralAction::Neutralize:
      return "neutralize";
  }
  return "unknown";
}

std::optional<StopPathTrackingCommand> resolve_stop_path_tracking_command(
  const StopPathTrackingCommandRequest & request) noexcept
{
  const auto & policy = request.policy;
  if (
    !std::isfinite(policy.wheelbase_m) || policy.wheelbase_m <= 0.0 ||
    !std::isfinite(policy.maximum_abs_steering_rad) ||
    policy.maximum_abs_steering_rad < 0.0 ||
    !std::isfinite(policy.maximum_abs_steering_rate_radps) ||
    policy.maximum_abs_steering_rate_radps < 0.0 ||
    !std::isfinite(policy.maximum_lateral_acceleration_mps2) ||
    policy.maximum_lateral_acceleration_mps2 < 0.0 ||
    !std::isfinite(policy.steering_command_gain) ||
    policy.steering_command_gain <= 0.0 ||
    !std::isfinite(policy.lateral_gain) || policy.lateral_gain < 0.0 ||
    !std::isfinite(policy.heading_gain) || policy.heading_gain < 0.0 ||
    !std::isfinite(request.current_lateral_m) ||
    !std::isfinite(request.target_lateral_m) ||
    !std::isfinite(request.current_heading_error_rad) ||
    !std::isfinite(request.reference_curvature_radpm) ||
    !std::isfinite(request.current_speed_mps) ||
    request.current_speed_mps < 0.0 ||
    !std::isfinite(request.current_steering_rad) ||
    !std::isfinite(request.step_sec) || request.step_sec <= 0.0)
  {
    return std::nullopt;
  }

  const double lateral_error_m =
    request.current_lateral_m - request.target_lateral_m;
  const double target_curvature_radpm =
    request.reference_curvature_radpm -
    policy.lateral_gain * lateral_error_m -
    policy.heading_gain * request.current_heading_error_rad;
  if (!std::isfinite(target_curvature_radpm)) {
    return std::nullopt;
  }
  const double unconstrained_target_steering_rad =
    std::atan(policy.wheelbase_m * target_curvature_radpm);
  double target_steering_rad = std::clamp(
    unconstrained_target_steering_rad,
    -policy.maximum_abs_steering_rad,
    policy.maximum_abs_steering_rad);
  if (request.current_speed_mps > std::numeric_limits<double>::epsilon()) {
    const double maximum_tire_angle_rad = std::atan(
      policy.wheelbase_m * policy.maximum_lateral_acceleration_mps2 /
      (request.current_speed_mps * request.current_speed_mps));
    const double maximum_controller_angle_rad =
      maximum_tire_angle_rad / policy.steering_command_gain;
    target_steering_rad = std::clamp(
      target_steering_rad,
      -maximum_controller_angle_rad,
      maximum_controller_angle_rad);
  }

  const double current_steering_rad = std::clamp(
    request.current_steering_rad,
    -policy.maximum_abs_steering_rad,
    policy.maximum_abs_steering_rad);
  const double maximum_step_rad =
    policy.maximum_abs_steering_rate_radps * request.step_sec;
  if (!std::isfinite(maximum_step_rad)) {
    return std::nullopt;
  }
  const double steering_delta_rad =
    target_steering_rad - current_steering_rad;
  const double steering_rad =
    std::abs(steering_delta_rad) <= maximum_step_rad ?
    target_steering_rad :
    current_steering_rad + std::copysign(
      maximum_step_rad, steering_delta_rad);
  const double steering_rate_radps =
    (steering_rad - current_steering_rad) / request.step_sec;
  if (!std::isfinite(steering_rad) || !std::isfinite(steering_rate_radps)) {
    return std::nullopt;
  }
  return StopPathTrackingCommand{
    unconstrained_target_steering_rad, target_steering_rad, steering_rad,
    steering_rate_radps};
}

const char * stop_shadow_intent_reason_name(
  const StopShadowIntentReason reason) noexcept
{
  switch (reason) {
    case StopShadowIntentReason::NotStop:
      return "not-stop";
    case StopShadowIntentReason::InterruptedOvertake:
      return "interrupted-overtake";
    case StopShadowIntentReason::InterruptedRejoin:
      return "interrupted-rejoin";
    case StopShadowIntentReason::CoherentFront:
      return "coherent-front";
    case StopShadowIntentReason::RaceCruise:
      return "race-cruise";
    case StopShadowIntentReason::PreRaceTrack:
      return "pre-race-track";
  }
  return "unknown";
}

StopShadowIntentResolution resolve_stop_shadow_intent(
  const StopShadowIntentRequest & request) noexcept
{
  StopShadowIntentResolution result;
  if (request.active_intent != mpcc_execution_contract::ControlIntent::Stop) {
    return result;
  }
  const bool interrupted_overtake =
    request.last_published_normal_intent ==
    mpcc_execution_contract::ControlIntent::ShiftOut ||
    request.last_published_normal_intent ==
    mpcc_execution_contract::ControlIntent::Pass ||
    request.last_published_normal_intent ==
    mpcc_execution_contract::ControlIntent::Return;
  if (interrupted_overtake && request.interrupted_overtake_context_available) {
    result.requested = true;
    result.intent = request.last_published_normal_intent;
    result.reason = StopShadowIntentReason::InterruptedOvertake;
    return result;
  }
  if (
    request.last_published_normal_intent ==
    mpcc_execution_contract::ControlIntent::Rejoin &&
    request.interrupted_rejoin_context_available)
  {
    result.requested = true;
    result.intent = mpcc_execution_contract::ControlIntent::Rejoin;
    result.reason = StopShadowIntentReason::InterruptedRejoin;
    return result;
  }
  result.requested = true;
  if (request.coherent_front_observation) {
    result.intent = mpcc_execution_contract::ControlIntent::Follow;
    result.reason = StopShadowIntentReason::CoherentFront;
  } else if (request.race_session_active) {
    result.intent = mpcc_execution_contract::ControlIntent::Cruise;
    result.reason = StopShadowIntentReason::RaceCruise;
  } else {
    result.intent = mpcc_execution_contract::ControlIntent::Track;
    result.reason = StopShadowIntentReason::PreRaceTrack;
  }
  return result;
}

const char * follow_longitudinal_contract_reason_name(
  const FollowLongitudinalContractReason reason) noexcept
{
  switch (reason) {
    case FollowLongitudinalContractReason::Accepted:
      return "accepted";
    case FollowLongitudinalContractReason::IntentNotFollow:
      return "intent-not-follow";
    case FollowLongitudinalContractReason::InvalidTargetIdentity:
      return "invalid-target-identity";
    case FollowLongitudinalContractReason::InvalidTargetObservation:
      return "invalid-target-observation";
    case FollowLongitudinalContractReason::StaleTargetObservation:
      return "stale-target-observation";
    case FollowLongitudinalContractReason::InvalidTargetKinematics:
      return "invalid-target-kinematics";
    case FollowLongitudinalContractReason::InvalidProgressOrigin:
      return "invalid-progress-origin";
    case FollowLongitudinalContractReason::InvalidConfiguration:
      return "invalid-configuration";
    case FollowLongitudinalContractReason::InvalidHorizon:
      return "invalid-horizon";
    case FollowLongitudinalContractReason::InitialHardGapViolation:
      return "initial-hard-gap-violation";
  }
  return "unknown";
}

FollowLongitudinalContract build_follow_longitudinal_contract(
  const FollowLongitudinalContractRequest & request) noexcept
{
  FollowLongitudinalContract result;
  result.target_id = request.target_id;
  result.target_observation_generation = request.target_observation_generation;
  result.current_target_gap_m = request.current_target_relative_progress_m;
  result.current_ego_progress_offset_m = request.current_ego_progress_offset_m;
  result.target_speed_mps = request.target_speed_mps;
  result.hard_gap_m = request.hard_gap_m;
  if (request.intent != mpcc_execution_contract::ControlIntent::Follow) {
    result.reason = FollowLongitudinalContractReason::IntentNotFollow;
    return result;
  }
  if (request.target_id.empty() || request.target_observation_generation == 0U) {
    result.reason = FollowLongitudinalContractReason::InvalidTargetIdentity;
    return result;
  }
  if (
    !std::isfinite(request.target_observation_age_sec) ||
    request.target_observation_age_sec < 0.0 ||
    !std::isfinite(request.maximum_target_observation_age_sec) ||
    request.maximum_target_observation_age_sec < 0.0)
  {
    result.reason = FollowLongitudinalContractReason::InvalidTargetObservation;
    return result;
  }
  if (
    request.target_observation_age_sec >
    request.maximum_target_observation_age_sec + 1e-9)
  {
    result.reason = FollowLongitudinalContractReason::StaleTargetObservation;
    return result;
  }
  const std::array<double, 8U> configuration{
    request.moving_target_speed_threshold_mps,
    request.desired_gap_m,
    request.hard_gap_m,
    request.maximum_closing_speed_mps,
    request.maximum_recovery_speed_mps,
    request.distance_gain_per_sec,
    request.slow_target_velocity_cap_mps,
    request.braking_deceleration_mps2};
  if (
    !std::isfinite(request.current_target_relative_progress_m) ||
    request.current_target_relative_progress_m < 0.0 ||
    !std::isfinite(request.current_ego_speed_mps) ||
    request.current_ego_speed_mps < 0.0 ||
    !std::isfinite(request.target_speed_mps) || request.target_speed_mps < 0.0)
  {
    result.reason = FollowLongitudinalContractReason::InvalidTargetKinematics;
    return result;
  }
  if (!std::isfinite(request.current_ego_progress_offset_m)) {
    result.reason = FollowLongitudinalContractReason::InvalidProgressOrigin;
    return result;
  }
  if (
    !std::all_of(
      configuration.begin(), configuration.end(),
      [](const double value) {return std::isfinite(value) && value >= 0.0;}) ||
    !std::isfinite(request.maximum_velocity_mps) ||
    request.maximum_velocity_mps < 0.0 ||
    request.desired_gap_m + 1e-9 < request.hard_gap_m ||
    request.braking_deceleration_mps2 <= 0.0)
  {
    result.reason = FollowLongitudinalContractReason::InvalidConfiguration;
    return result;
  }
  const std::size_t horizon_steps = request.stage_dt_sec.size();
  if (
    horizon_steps == 0U ||
    request.base_progress_reference_m.size() != horizon_steps + 1U ||
    request.base_progress_upper_m.size() != horizon_steps + 1U ||
    request.base_velocity_reference_mps.size() != horizon_steps ||
    request.base_velocity_upper_mps.size() != horizon_steps)
  {
    result.reason = FollowLongitudinalContractReason::InvalidHorizon;
    return result;
  }
  const auto finite_nonnegative = [](const std::vector<double> & values) {
      return std::all_of(
        values.begin(), values.end(),
        [](const double value) {return std::isfinite(value) && value >= 0.0;});
    };
  if (
    !finite_nonnegative(request.stage_dt_sec) ||
    !finite_nonnegative(request.base_progress_reference_m) ||
    !finite_nonnegative(request.base_progress_upper_m) ||
    !finite_nonnegative(request.base_velocity_reference_mps) ||
    !finite_nonnegative(request.base_velocity_upper_mps) ||
    std::any_of(
      request.stage_dt_sec.begin(), request.stage_dt_sec.end(),
      [](const double value) {return value <= 0.0;}))
  {
    result.reason = FollowLongitudinalContractReason::InvalidHorizon;
    return result;
  }
  if (
    request.current_target_relative_progress_m + 1e-9 < request.hard_gap_m)
  {
    result.reason = FollowLongitudinalContractReason::InitialHardGapViolation;
    return result;
  }

  // Planning must preserve the configured nominal gap whenever it is already
  // available. If the current observation is inside that nominal gap, freeze
  // the current physical gap instead of making state zero infeasible. The
  // independent hard-gap certificate remains the physical failure boundary.
  result.planning_gap_m = std::clamp(
    request.current_target_relative_progress_m,
    request.hard_gap_m, request.desired_gap_m);

  result.elapsed_time_sec.reserve(horizon_steps + 1U);
  result.target_progress_m.reserve(horizon_steps + 1U);
  result.progress_reference_m.reserve(horizon_steps + 1U);
  result.progress_lower_m.reserve(horizon_steps + 1U);
  result.progress_upper_m.reserve(horizon_steps + 1U);
  result.velocity_reference_mps.reserve(horizon_steps);
  result.velocity_upper_mps.reserve(horizon_steps);

  double elapsed_sec = 0.0;
  for (std::size_t state = 0U; state <= horizon_steps; ++state) {
    if (state > 0U) {
      elapsed_sec += request.stage_dt_sec[state - 1U];
    }
    const double target_progress =
      request.current_ego_progress_offset_m +
      request.current_target_relative_progress_m +
      request.target_speed_mps * elapsed_sec;
    const double desired_progress = std::max(
      0.0, target_progress - request.desired_gap_m);
    result.elapsed_time_sec.push_back(elapsed_sec);
    result.target_progress_m.push_back(target_progress);
    result.progress_reference_m.push_back(std::min(
      request.base_progress_reference_m[state], desired_progress));
    // Normal Follow may hold its current progress. Monotonicity is enforced
    // by the non-negative virtual-progress input in the canonical model.
    result.progress_lower_m.push_back(0.0);
    // Generic progress bounds own theta feasibility. The physical Follow gap
    // is a theta+e_lag constraint assembled once by the canonical QP; folding
    // it into theta-only bounds here would apply the same concept twice in
    // incompatible coordinates.
    result.progress_upper_m.push_back(request.base_progress_upper_m[state]);
  }

  const bool moving_target =
    request.target_speed_mps > request.moving_target_speed_threshold_mps;
  for (std::size_t stage = 0U; stage < horizon_steps; ++stage) {
    const std::size_t next_state = stage + 1U;
    const double predicted_gap =
      result.target_progress_m[next_state] -
      request.base_progress_reference_m[next_state];
    double longitudinal_reference = 0.0;
    if (moving_target) {
      const double signed_margin = std::clamp(
        request.distance_gain_per_sec *
        (predicted_gap - request.desired_gap_m),
        -request.maximum_recovery_speed_mps,
        request.maximum_closing_speed_mps);
      longitudinal_reference = std::max(
        0.0, request.target_speed_mps + signed_margin);
    } else {
      const double approach_distance = std::max(
        0.0, predicted_gap - request.desired_gap_m);
      longitudinal_reference = std::min(
        request.slow_target_velocity_cap_mps,
        std::sqrt(
          2.0 * request.braking_deceleration_mps2 * approach_distance));
    }
    const double policy_velocity_upper = std::min({
      request.base_velocity_upper_mps[stage],
      request.maximum_velocity_mps,
      longitudinal_reference});
    // The policy limit is a target, not an instantaneous physical state.  A
    // cap below the fastest speed reachable under maximum braking makes the
    // next state infeasible before the solver has any control authority.
    // Preserve the policy reference while admitting the deterministic braking
    // transition from the measured speed.
    const double minimum_reachable_velocity = std::max(
      0.0, request.current_ego_speed_mps -
      request.braking_deceleration_mps2 * result.elapsed_time_sec[next_state]);
    const double velocity_upper = std::max(
      policy_velocity_upper, minimum_reachable_velocity);
    if (!std::isfinite(velocity_upper) || velocity_upper < 0.0) {
      result.reason = FollowLongitudinalContractReason::InvalidHorizon;
      return result;
    }
    result.velocity_upper_mps.push_back(velocity_upper);
    result.velocity_reference_mps.push_back(std::min(
      request.base_velocity_reference_mps[stage], policy_velocity_upper));
  }

  result.valid = true;
  result.reason = FollowLongitudinalContractReason::Accepted;
  return result;
}

FollowEffectiveGapCertificate evaluate_follow_effective_gap(
  const std::vector<double> & target_progress_m,
  const std::vector<double> & solved_progress_m,
  const std::vector<double> & solved_lag_m,
  const double hard_gap_m, const double tolerance_m) noexcept
{
  FollowEffectiveGapCertificate result;
  if (
    target_progress_m.empty() ||
    target_progress_m.size() != solved_progress_m.size() ||
    target_progress_m.size() != solved_lag_m.size() ||
    !std::isfinite(hard_gap_m) || hard_gap_m < 0.0 ||
    !std::isfinite(tolerance_m) || tolerance_m < 0.0)
  {
    return result;
  }
  for (std::size_t stage = 0U; stage < target_progress_m.size(); ++stage) {
    const double target_progress = target_progress_m[stage];
    const double solved_progress = solved_progress_m[stage];
    const double solved_lag = solved_lag_m[stage];
    if (
      !std::isfinite(target_progress) || !std::isfinite(solved_progress) ||
      !std::isfinite(solved_lag))
    {
      return result;
    }
    const double gap = target_progress - (solved_progress + solved_lag);
    const double violation = std::max(0.0, hard_gap_m - gap);
    result.minimum_gap_m = std::min(result.minimum_gap_m, gap);
    if (violation > result.maximum_violation_m) {
      result.maximum_violation_m = violation;
      result.worst_stage = static_cast<int>(stage);
    }
  }
  result.valid = true;
  result.satisfied = result.maximum_violation_m <= tolerance_m;
  return result;
}

}  // namespace multi_purpose_mpc_ros::race_mpcc_foundation
