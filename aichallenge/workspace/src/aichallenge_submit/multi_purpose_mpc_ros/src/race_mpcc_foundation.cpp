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
    case TrackCruiseShadowEligibilityReason::ProgressMpccDisabled:
      return "progress-mpcc-disabled";
    case TrackCruiseShadowEligibilityReason::MigrationBoundaryInactive:
      return "migration-boundary-inactive";
    case TrackCruiseShadowEligibilityReason::ExtendedDynamicsDisabled:
      return "extended-dynamics-disabled";
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
  if (!request.progress_mpcc_enabled) {
    result.reason = TrackCruiseShadowEligibilityReason::ProgressMpccDisabled;
    return result;
  }
  if (!request.overtake_only_boundary) {
    result.reason = TrackCruiseShadowEligibilityReason::MigrationBoundaryInactive;
    return result;
  }
  if (!request.extended_dynamics_enabled) {
    result.reason = TrackCruiseShadowEligibilityReason::ExtendedDynamicsDisabled;
    return result;
  }
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

const char * follow_shadow_eligibility_reason_name(
  const FollowShadowEligibilityReason reason) noexcept
{
  switch (reason) {
    case FollowShadowEligibilityReason::Eligible:
      return "eligible";
    case FollowShadowEligibilityReason::ProgressMpccDisabled:
      return "progress-mpcc-disabled";
    case FollowShadowEligibilityReason::MigrationBoundaryInactive:
      return "migration-boundary-inactive";
    case FollowShadowEligibilityReason::ExtendedDynamicsDisabled:
      return "extended-dynamics-disabled";
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
  if (!request.progress_mpcc_enabled) {
    result.reason = FollowShadowEligibilityReason::ProgressMpccDisabled;
    return result;
  }
  if (!request.overtake_only_boundary) {
    result.reason = FollowShadowEligibilityReason::MigrationBoundaryInactive;
    return result;
  }
  if (!request.extended_dynamics_enabled) {
    result.reason = FollowShadowEligibilityReason::ExtendedDynamicsDisabled;
    return result;
  }
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
    case OvertakeCanonicalFreshShadowEligibilityReason::ExtendedDynamicsDisabled:
      return "extended-dynamics-disabled";
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
  if (!request.extended_dynamics_enabled) {
    result.reason = OvertakeCanonicalFreshShadowEligibilityReason::
      ExtendedDynamicsDisabled;
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
  const bool complete_canonical_selection) noexcept
{
  if (intent != mpcc_execution_contract::ControlIntent::Follow) {
    return FollowProductionAction::NotOwned;
  }
  return complete_canonical_selection ?
    FollowProductionAction::PublishCanonical :
    FollowProductionAction::EmergencyStop;
}

StopAuthorityAction resolve_stop_authority_action(
  const mpcc_execution_contract::ControlIntent intent) noexcept
{
  return intent == mpcc_execution_contract::ControlIntent::Stop ?
    StopAuthorityAction::EmergencyStop : StopAuthorityAction::NotOwned;
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
    // by the non-negative virtual-progress input in the five-state model.
    result.progress_lower_m.push_back(0.0);
    // Generic progress bounds own theta feasibility. The physical Follow gap
    // is a theta+e_lag constraint assembled once by the five-state QP; folding
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

const char * shadow_warm_start_reset_reason_name(
  const ShadowWarmStartResetReason reason) noexcept
{
  switch (reason) {
    case ShadowWarmStartResetReason::None:
      return "none";
    case ShadowWarmStartResetReason::InitialContext:
      return "initial-context";
    case ShadowWarmStartResetReason::InvalidPreviousContext:
      return "invalid-previous-context";
    case ShadowWarmStartResetReason::InvalidCurrentContext:
      return "invalid-current-context";
    case ShadowWarmStartResetReason::IntentChanged:
      return "intent-changed";
    case ShadowWarmStartResetReason::FormulationChanged:
      return "formulation-changed";
    case ShadowWarmStartResetReason::HorizonChanged:
      return "horizon-changed";
    case ShadowWarmStartResetReason::SchemaChanged:
      return "schema-changed";
    case ShadowWarmStartResetReason::StageGeometryDiscontinuous:
      return "stage-geometry-discontinuous";
  }
  return "unknown";
}

namespace
{

bool shadow_identity_complete(const ShadowWarmStartIdentity & identity) noexcept
{
  const bool intent_valid =
    mpcc_execution_contract::canonical_normal_intent_supported(identity.intent);
  if (
    !intent_valid ||
    identity.formulation !=
    mpcc_execution_contract::Formulation::VelocityProgress5State ||
    identity.horizon_steps == 0U ||
    identity.horizon_steps != identity.stages.size() ||
    identity.state_schema_id.empty() || identity.input_schema_id.empty() ||
    identity.bounds_schema_id.empty() || identity.cost_schema_id.empty() ||
    identity.stage_geometry_id == 0U || identity.stages.empty() ||
    identity.stages.front().transition_from_waypoint != identity.tracking_waypoint)
  {
    return false;
  }
  if (
    identity.stage_geometry_id !=
    mpcc_execution_contract::fingerprint_stage_geometry(
      identity.tracking_waypoint, identity.circular, identity.stages))
  {
    return false;
  }
  for (std::size_t index = 0U; index < identity.stages.size(); ++index) {
    const auto & stage = identity.stages[index];
    if (
      !std::isfinite(stage.transition_distance_m) ||
      stage.transition_distance_m < 0.0 ||
      !std::isfinite(stage.cumulative_distance_m) ||
      stage.cumulative_distance_m < 0.0 ||
      (index > 0U &&
      identity.stages[index - 1U].state_waypoint !=
      stage.transition_from_waypoint))
    {
      return false;
    }
  }
  return true;
}

bool same_schema(
  const ShadowWarmStartIdentity & previous,
  const ShadowWarmStartIdentity & current) noexcept
{
  return
    previous.state_schema_id == current.state_schema_id &&
    previous.input_schema_id == current.input_schema_id &&
    previous.bounds_schema_id == current.bounds_schema_id &&
    previous.cost_schema_id == current.cost_schema_id;
}

bool rolling_stage_geometry_compatible(
  const ShadowWarmStartIdentity & previous,
  const ShadowWarmStartIdentity & current) noexcept
{
  if (previous.stage_geometry_id == current.stage_geometry_id) {
    return true;
  }
  if (previous.circular != current.circular || previous.stages.empty() || current.stages.empty()) {
    return false;
  }
  std::size_t offset = previous.stages.size();
  for (std::size_t index = 0U; index < previous.stages.size(); ++index) {
    if (
      previous.stages[index].transition_from_waypoint ==
      current.stages.front().transition_from_waypoint)
    {
      offset = index;
      break;
    }
  }
  if (offset >= previous.stages.size()) {
    return false;
  }
  const std::size_t overlap = std::min(
    previous.stages.size() - offset, current.stages.size());
  const std::size_t minimum_overlap = std::min<std::size_t>(2U, current.stages.size());
  if (overlap < minimum_overlap) {
    return false;
  }
  for (std::size_t index = 0U; index < overlap; ++index) {
    const auto & old_stage = previous.stages[offset + index];
    const auto & new_stage = current.stages[index];
    if (
      old_stage.transition_from_waypoint != new_stage.transition_from_waypoint ||
      old_stage.state_waypoint != new_stage.state_waypoint ||
      std::abs(old_stage.transition_distance_m - new_stage.transition_distance_m) > 1e-9)
    {
      return false;
    }
  }
  return true;
}

}  // namespace

ShadowWarmStartResolution resolve_shadow_warm_start(
  const std::optional<ShadowWarmStartIdentity> & previous,
  const ShadowWarmStartIdentity & current) noexcept
{
  ShadowWarmStartResolution result;
  if (!shadow_identity_complete(current)) {
    result.reason = ShadowWarmStartResetReason::InvalidCurrentContext;
    return result;
  }
  result.valid = true;
  if (!previous.has_value()) {
    result.reason = ShadowWarmStartResetReason::InitialContext;
    return result;
  }
  if (!shadow_identity_complete(previous.value())) {
    result.reason = ShadowWarmStartResetReason::InvalidPreviousContext;
    return result;
  }
  if (previous->intent != current.intent) {
    result.reason = ShadowWarmStartResetReason::IntentChanged;
    return result;
  }
  if (previous->formulation != current.formulation) {
    result.reason = ShadowWarmStartResetReason::FormulationChanged;
    return result;
  }
  if (previous->horizon_steps != current.horizon_steps) {
    result.reason = ShadowWarmStartResetReason::HorizonChanged;
    return result;
  }
  if (!same_schema(previous.value(), current)) {
    result.reason = ShadowWarmStartResetReason::SchemaChanged;
    return result;
  }
  if (!rolling_stage_geometry_compatible(previous.value(), current)) {
    result.reason = ShadowWarmStartResetReason::StageGeometryDiscontinuous;
    return result;
  }
  result.apply_warm_start = true;
  result.reset_context = false;
  result.reason = ShadowWarmStartResetReason::None;
  return result;
}

}  // namespace multi_purpose_mpc_ros::race_mpcc_foundation
