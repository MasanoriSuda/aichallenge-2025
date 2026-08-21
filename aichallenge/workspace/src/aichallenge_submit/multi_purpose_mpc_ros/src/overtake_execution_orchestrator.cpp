#include "multi_purpose_mpc_ros/overtake_execution_orchestrator.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <utility>

namespace multi_purpose_mpc_ros::overtake_execution_orchestrator {
namespace {

bool phase_has_mission(const Phase phase) noexcept
{
  return phase != Phase::Idle;
}

bool finite_nonnegative(const double value) noexcept
{
  return std::isfinite(value) && value >= 0.0;
}

std::string finite_or(const double value, const char * fallback)
{
  if (!std::isfinite(value)) {
    return fallback;
  }
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(2) << value;
  return stream.str();
}

void append_phase(std::ostringstream & stream, bool & first, const char * phase)
{
  if (!first) {
    stream << "/";
  }
  stream << phase;
  first = false;
}

}  // namespace

CorridorMetrics analyze_corridor(
  const std::vector<double> & lower_m,
  const std::vector<double> & upper_m,
  const std::vector<double> & path_distance_m) noexcept
{
  CorridorMetrics metrics;
  if (lower_m.empty() || lower_m.size() != upper_m.size()) {
    return metrics;
  }
  for (std::size_t i = 0; i < lower_m.size(); ++i) {
    if (
      !std::isfinite(lower_m[i]) || !std::isfinite(upper_m[i]) ||
      upper_m[i] < lower_m[i])
    {
      continue;
    }
    const double width = upper_m[i] - lower_m[i];
    ++metrics.sample_count;
    if (!metrics.valid || width < metrics.minimum_width_m) {
      metrics.valid = true;
      metrics.minimum_width_m = width;
      metrics.minimum_width_index = i;
      metrics.minimum_width_distance_m =
        i < path_distance_m.size() && std::isfinite(path_distance_m[i]) ?
        path_distance_m[i] : static_cast<double>(i);
    }
  }
  return metrics;
}

SpeedWindowResolution normalize_speed_window(
  const double reference_mps, const double limit_mps, const double floor_mps,
  const bool floor_active) noexcept
{
  SpeedWindowResolution result;
  if (
    std::isnan(reference_mps) || std::isnan(limit_mps) ||
    (floor_active && (!std::isfinite(floor_mps) || floor_mps < 0.0)))
  {
    return result;
  }
  result.valid = true;
  result.reference_mps = std::isfinite(reference_mps) ?
    std::max(0.0, reference_mps) : std::numeric_limits<double>::infinity();
  result.limit_mps = std::isfinite(limit_mps) ?
    std::max(0.0, limit_mps) : std::numeric_limits<double>::infinity();
  result.requested_floor_mps = floor_active ? floor_mps : 0.0;
  result.floor_mps = result.requested_floor_mps;
  const double upper_mps = std::min(result.reference_mps, result.limit_mps);
  if (floor_active && std::isfinite(upper_mps) && result.floor_mps > upper_mps) {
    result.floor_mps = upper_mps;
    result.floor_adjusted = true;
  }
  return result;
}

WallClearanceContract resolve_wall_clearance_contract(
  const double physical_clearance_m, const double planning_clearance_m,
  const bool runtime_preplan_enabled, const double runtime_reserve_m) noexcept
{
  WallClearanceContract result;
  if (
    !std::isfinite(physical_clearance_m) || physical_clearance_m < 0.0 ||
    !std::isfinite(planning_clearance_m) || planning_clearance_m < 0.0 ||
    !std::isfinite(runtime_reserve_m) || runtime_reserve_m < 0.0)
  {
    return result;
  }
  result.valid = true;
  result.physical_clearance_m = physical_clearance_m;
  result.planning_clearance_m = std::max(
    physical_clearance_m, planning_clearance_m);
  result.runtime_reserve_m = runtime_preplan_enabled ? runtime_reserve_m : 0.0;
  result.required_clearance_m = std::max(
    result.planning_clearance_m,
    result.physical_clearance_m + result.runtime_reserve_m);
  return result;
}

LateralBoundContractResolution resolve_lateral_bound_contract(
  const double lower_m, const double upper_m) noexcept
{
  LateralBoundContractResolution result;
  result.lower_m = lower_m;
  result.upper_m = upper_m;
  if (!std::isfinite(lower_m) || !std::isfinite(upper_m)) {
    result.reason = LateralBoundContractReason::NonFiniteBound;
    return result;
  }
  result.valid = true;
  if (lower_m > upper_m) {
    result.reason = LateralBoundContractReason::EmptyIntersection;
    return result;
  }
  result.feasible = true;
  result.reason = LateralBoundContractReason::None;
  return result;
}

const char * to_string(const LateralBoundContractReason reason) noexcept
{
  switch (reason) {
    case LateralBoundContractReason::None:
      return "none";
    case LateralBoundContractReason::NonFiniteBound:
      return "non-finite-bound";
    case LateralBoundContractReason::EmptyIntersection:
      return "empty-intersection";
  }
  return "unknown";
}

RuntimeReplacementContractResolution resolve_runtime_replacement_contract(
  const RuntimeReplacementContractRequest & request) noexcept
{
  RuntimeReplacementContractResolution result;
  if (!request.candidate_feasible) {
    result.valid = true;
    result.reason = RuntimeReplacementRejectReason::InvalidCandidate;
    return result;
  }
  if (
    !std::isfinite(request.now_sec) ||
    !std::isfinite(request.dynamic_valid_until_sec))
  {
    result.reason = RuntimeReplacementRejectReason::PredictionInvalid;
    return result;
  }
  result.valid = true;
  if (request.now_sec > request.dynamic_valid_until_sec + 1e-9) {
    result.reason = RuntimeReplacementRejectReason::PredictionExpired;
    return result;
  }
  if (!request.target_clearance_checked) {
    result.reason = RuntimeReplacementRejectReason::TargetClearanceUnchecked;
    return result;
  }
  if (!std::isfinite(request.minimum_target_clearance_m)) {
    result.valid = false;
    result.reason = RuntimeReplacementRejectReason::TargetClearanceInvalid;
    return result;
  }
  if (request.minimum_target_clearance_m < -1e-9) {
    result.reason = RuntimeReplacementRejectReason::TargetOverlap;
    return result;
  }
  if (
    !std::isfinite(request.minimum_path_wall_clearance_m) ||
    !std::isfinite(request.required_path_wall_clearance_m) ||
    request.required_path_wall_clearance_m < 0.0)
  {
    result.valid = false;
    result.reason = RuntimeReplacementRejectReason::WallClearanceUnchecked;
    return result;
  }
  if (
    request.minimum_path_wall_clearance_m + 1e-9 <
    request.required_path_wall_clearance_m)
  {
    result.reason = RuntimeReplacementRejectReason::WallContractShortfall;
    return result;
  }

  result.admitted = true;
  result.reason = RuntimeReplacementRejectReason::None;
  return result;
}

const char * to_string(const RuntimeReplacementRejectReason reason) noexcept
{
  switch (reason) {
    case RuntimeReplacementRejectReason::None:
      return "none";
    case RuntimeReplacementRejectReason::InvalidCandidate:
      return "invalid-candidate";
    case RuntimeReplacementRejectReason::PredictionInvalid:
      return "prediction-invalid";
    case RuntimeReplacementRejectReason::PredictionExpired:
      return "prediction-expired";
    case RuntimeReplacementRejectReason::TargetClearanceUnchecked:
      return "target-clearance-unchecked";
    case RuntimeReplacementRejectReason::TargetClearanceInvalid:
      return "target-clearance-invalid";
    case RuntimeReplacementRejectReason::TargetOverlap:
      return "target-overlap";
    case RuntimeReplacementRejectReason::WallClearanceUnchecked:
      return "wall-clearance-unchecked";
    case RuntimeReplacementRejectReason::WallContractShortfall:
      return "wall-contract-shortfall";
  }
  return "unknown";
}

AuthorityResolution resolve_authority(const AuthorityRequest & request) noexcept
{
  AuthorityResolution result;
  const bool safety_active =
    request.behavior == Behavior::SafetyBrake || request.emergency_brake_active;
  const bool active_phase = phase_has_mission(request.phase);
  result.relevant =
    active_phase || request.episode_id != 0U || !request.target_id.empty() ||
    request.dynamic_obstacle_escape_active || request.dynamic_wait_active ||
    request.contact_continuation_active || request.precontact_escape_active ||
    safety_active;

  result.use_overtake_line_target = request.line_active;
  result.apply_overtake_speed_reference =
    std::isfinite(request.speed_reference_mps);
  result.apply_overtake_speed_limit = std::isfinite(request.speed_limit_mps);
  result.apply_overtake_speed_floor = request.pass_speed_floor_active;
  result.path_source = request.path_source_hint;

  if (safety_active) {
    result.action = Action::SafetyBrake;
    result.reason = "safety-brake-precedence";
  } else if (request.phase == Phase::Recovery) {
    result.action = Action::Recovery;
    result.reason = "overtake-recovery";
  } else if (request.contact_continuation_active || request.precontact_escape_active) {
    result.action = Action::ContactEscape;
    result.reason = request.contact_continuation_active ?
      "contact-continuation" : "precontact-escape";
  } else if (request.dynamic_wait_active) {
    result.action = Action::DynamicWait;
    result.reason = request.dynamic_wait_forward_prefix_active ?
      "dynamic-wait-forward-prefix" : "dynamic-wait-hold";
  } else if (request.phase == Phase::ShiftOut) {
    result.action = Action::ShiftOut;
    result.reason = "committed-shiftout";
  } else if (request.phase == Phase::Pass) {
    result.action = Action::Pass;
    result.reason = "committed-pass";
  } else if (request.phase == Phase::Return) {
    result.action = Action::Return;
    result.reason = "committed-return";
  } else if (request.dynamic_obstacle_escape_active) {
    result.action = Action::DynamicEscape;
    result.reason = "dynamic-obstacle-escape";
  } else if (request.behavior == Behavior::Follow || request.follow_cap_active) {
    result.action = Action::Follow;
    result.reason = "follow";
  }

  if (request.line_active) {
    if (request.phase == Phase::Recovery) {
      result.lateral_owner = LateralOwner::RecoveryLine;
    } else if (
      request.dynamic_wait_active && request.dynamic_wait_lateral_authority_active)
    {
      result.lateral_owner = LateralOwner::DynamicWaitPrefix;
    } else if (
      request.contact_continuation_active || request.precontact_escape_active)
    {
      result.lateral_owner = LateralOwner::ContactEscape;
    } else if (safety_active) {
      result.lateral_owner = LateralOwner::SafetyHold;
    } else {
      result.lateral_owner = LateralOwner::OvertakeLine;
    }
  } else if (request.dynamic_obstacle_escape_active) {
    result.lateral_owner = LateralOwner::DynamicObstacleEscape;
  } else if (request.gap_planner_active) {
    result.lateral_owner = LateralOwner::GapPlanner;
  } else if (safety_active) {
    result.lateral_owner = LateralOwner::SafetyHold;
  }

  if (safety_active) {
    result.path_source = PathSource::SafetyHold;
  } else if (request.phase == Phase::Recovery) {
    result.path_source = PathSource::RecoveryLine;
  } else if (
    request.contact_continuation_active || request.precontact_escape_active)
  {
    result.path_source = PathSource::ContactEscape;
  } else if (
    request.dynamic_wait_active && request.dynamic_wait_lateral_authority_active)
  {
    result.path_source = PathSource::DynamicWaitPrefix;
  } else if (request.dynamic_obstacle_escape_active) {
    result.path_source = PathSource::DynamicObstacleEscape;
  } else if (request.gap_planner_active && !request.line_active) {
    result.path_source = PathSource::GapPlanner;
  } else if (
    request.line_active && request.path_source_hint == PathSource::RacingLine)
  {
    result.path_source = PathSource::FrozenMission;
  } else if (!request.line_active) {
    result.path_source = PathSource::RacingLine;
  }

  if (safety_active) {
    result.longitudinal_owner = LongitudinalOwner::SafetyBrake;
  } else if (request.solver_fallback_active) {
    result.longitudinal_owner = LongitudinalOwner::SolverFallback;
  } else if (request.pass_speed_floor_active) {
    result.longitudinal_owner = LongitudinalOwner::PassFloor;
  } else if (
    result.apply_overtake_speed_reference || result.apply_overtake_speed_limit ||
    request.shiftout_speed_floor_active ||
    (request.phase == Phase::ShiftOut && request.line_active &&
    request.shiftout_speed_contract_expected))
  {
    result.longitudinal_owner = LongitudinalOwner::OvertakeLine;
  } else if (
    request.dynamic_obstacle_escape_active &&
    request.dynamic_obstacle_follow_cap_suppressed)
  {
    result.longitudinal_owner = LongitudinalOwner::DynamicObstacleEscape;
  } else if (request.follow_cap_active) {
    result.longitudinal_owner = LongitudinalOwner::FollowCap;
  }

  if (safety_active && request.line_active) {
    result.conflicts |= SafetyWithActiveLine;
  }
  if (
    safety_active &&
    (request.pass_speed_floor_active || request.shiftout_speed_floor_active))
  {
    result.conflicts |= SafetyWithSpeedFloor;
  }
  if (request.front_cap_release_ready && request.follow_cap_active) {
    result.conflicts |= ReleasedPassWithFollowCap;
  }
  if (
    request.dynamic_wait_active && !request.dynamic_wait_lateral_authority_active &&
    !request.line_active && !safety_active && request.phase != Phase::Recovery)
  {
    result.conflicts |= DynamicWaitWithoutLateralAuthority;
  }
  if (active_phase && request.target_id.empty()) {
    result.conflicts |= ActivePhaseWithoutTarget;
  }
  // DynamicObstacleEscape consumes GapPlanner bounds as one execution chain.
  // Counting both flags as independent owners produces a false conflict even
  // though only DynamicObstacleEscape owns the final lateral command.
  const bool independent_gap_planner =
    request.gap_planner_active && !request.dynamic_obstacle_escape_active;
  const int lateral_source_count =
    (request.line_active ? 1 : 0) +
    (request.dynamic_obstacle_escape_active ? 1 : 0) +
    (independent_gap_planner ? 1 : 0);
  if (lateral_source_count > 1) {
    result.conflicts |= MultipleLateralAuthorities;
  }
  const double effective_upper = std::min(
    request.speed_reference_mps, request.speed_limit_mps);
  if (
    request.pass_speed_floor_active && finite_nonnegative(request.speed_floor_mps) &&
    std::isfinite(effective_upper) &&
    request.speed_floor_mps > effective_upper + 1e-6)
  {
    result.conflicts |= InvalidSpeedWindow;
  }
  if (
    request.line_active &&
    finite_nonnegative(request.wall_contract_required_clearance_m) &&
    std::isfinite(request.wall_contract_minimum_path_clearance_m) &&
    request.wall_contract_minimum_path_clearance_m + 1e-6 <
    request.wall_contract_required_clearance_m)
  {
    result.conflicts |= WallContractShortfall;
  }
  if (
    request.phase == Phase::ShiftOut && request.line_active &&
    request.shiftout_speed_contract_expected &&
    !request.shiftout_speed_contract_active)
  {
    result.conflicts |= ShiftOutWithoutSpeedContract;
  }
  return result;
}

const char * to_string(const Phase phase) noexcept
{
  switch (phase) {
    case Phase::Idle: return "Idle";
    case Phase::ShiftOut: return "ShiftOut";
    case Phase::Pass: return "Pass";
    case Phase::Return: return "Return";
    case Phase::FollowPrepare: return "FollowPrepare";
    case Phase::Recovery: return "Recovery";
  }
  return "Unknown";
}

const char * to_string(const Behavior behavior) noexcept
{
  switch (behavior) {
    case Behavior::Cruise: return "Cruise";
    case Behavior::Follow: return "Follow";
    case Behavior::Overtake: return "Overtake";
    case Behavior::LowSpeedAvoidance: return "LowSpeedAvoidance";
    case Behavior::SafetyBrake: return "SafetyBrake";
  }
  return "Unknown";
}

const char * to_string(const Action action) noexcept
{
  switch (action) {
    case Action::Cruise: return "cruise";
    case Action::Follow: return "follow";
    case Action::DynamicEscape: return "dynamic-escape";
    case Action::ShiftOut: return "shiftout";
    case Action::Pass: return "pass";
    case Action::Return: return "return";
    case Action::DynamicWait: return "dynamic-wait";
    case Action::ContactEscape: return "contact-escape";
    case Action::Recovery: return "recovery";
    case Action::SafetyBrake: return "safety-brake";
  }
  return "unknown";
}

const char * to_string(const LateralOwner owner) noexcept
{
  switch (owner) {
    case LateralOwner::RacingLine: return "racing-line";
    case LateralOwner::GapPlanner: return "gap-planner";
    case LateralOwner::DynamicObstacleEscape: return "dynamic-obstacle-escape";
    case LateralOwner::OvertakeLine: return "overtake-line";
    case LateralOwner::DynamicWaitPrefix: return "dynamic-wait-prefix";
    case LateralOwner::ContactEscape: return "contact-escape";
    case LateralOwner::RecoveryLine: return "recovery-line";
    case LateralOwner::SafetyHold: return "safety-hold";
  }
  return "unknown";
}

const char * to_string(const LongitudinalOwner owner) noexcept
{
  switch (owner) {
    case LongitudinalOwner::RacingLine: return "racing-line";
    case LongitudinalOwner::FollowCap: return "follow-cap";
    case LongitudinalOwner::DynamicObstacleEscape: return "dynamic-obstacle-escape";
    case LongitudinalOwner::OvertakeLine: return "overtake-line";
    case LongitudinalOwner::PassFloor: return "pass-floor";
    case LongitudinalOwner::SolverFallback: return "solver-fallback";
    case LongitudinalOwner::SafetyBrake: return "safety-brake";
  }
  return "unknown";
}

const char * to_string(const PathSource source) noexcept
{
  switch (source) {
    case PathSource::RacingLine: return "racing-line";
    case PathSource::GapPlanner: return "gap-planner";
    case PathSource::DynamicObstacleEscape: return "dynamic-obstacle-escape";
    case PathSource::FrozenMission: return "frozen-mission";
    case PathSource::RecedingHorizon: return "receding-horizon";
    case PathSource::RecedingDp: return "receding-dp";
    case PathSource::DynamicWaitPrefix: return "dynamic-wait-prefix";
    case PathSource::ContactEscape: return "contact-escape";
    case PathSource::RecoveryLine: return "recovery-line";
    case PathSource::SafetyHold: return "safety-hold";
  }
  return "unknown";
}

std::string format_conflicts(const std::uint32_t conflicts)
{
  if (conflicts == NoConflict) {
    return "none";
  }
  std::ostringstream stream;
  bool first = true;
  const auto append = [&](const AuthorityConflict conflict, const char * name) {
      if ((conflicts & conflict) == 0U) {
        return;
      }
      if (!first) {
        stream << "+";
      }
      stream << name;
      first = false;
    };
  append(SafetyWithActiveLine, "safety-with-line");
  append(SafetyWithSpeedFloor, "safety-with-floor");
  append(ReleasedPassWithFollowCap, "released-pass-with-follow-cap");
  append(DynamicWaitWithoutLateralAuthority, "dynamic-wait-without-lateral");
  append(ActivePhaseWithoutTarget, "active-phase-without-target");
  append(MultipleLateralAuthorities, "multiple-lateral-authorities");
  append(InvalidSpeedWindow, "invalid-speed-window");
  append(WallContractShortfall, "wall-contract-shortfall");
  append(ShiftOutWithoutSpeedContract, "shiftout-without-speed-contract");
  return stream.str();
}

std::string categorical_signature(const AuthorityTrace & trace)
{
  std::ostringstream stream;
  stream << (trace.resolution.relevant ? 1 : 0) << "|"
         << trace.request.episode_id << "|" << trace.request.mission_generation << "|"
         << trace.request.target_id << "|" << static_cast<int>(trace.request.phase) << "|"
         << static_cast<int>(trace.request.behavior) << "|"
         << static_cast<int>(trace.resolution.action) << "|"
         << static_cast<int>(trace.resolution.lateral_owner) << "|"
         << static_cast<int>(trace.resolution.longitudinal_owner) << "|"
         << static_cast<int>(trace.resolution.path_source) << "|"
         << trace.request.pass_side_sign << "|"
         << trace.resolution.conflicts << "|"
         << (std::isfinite(trace.request.speed_reference_mps) ? 1 : 0) << "|"
         << (std::isfinite(trace.request.speed_limit_mps) ? 1 : 0) << "|"
         << (trace.request.pass_speed_floor_active ? 1 : 0) << "|"
         << (trace.request.speed_floor_adjusted ? 1 : 0) << "|"
         << (trace.request.corridor_blocked ? 1 : 0) << "|"
         << (trace.request.shiftout_speed_contract_expected ? 1 : 0) << "|"
         << (trace.request.shiftout_speed_contract_active ? 1 : 0) << "|"
         << (trace.request.dynamic_wait_active ? 1 : 0) << "|"
         << (trace.request.contact_continuation_active ? 1 : 0) << "|"
         << (trace.request.precontact_escape_active ? 1 : 0);
  return stream.str();
}

std::string format_authority_trace(const AuthorityTrace & trace)
{
  std::ostringstream stream;
  stream << "Overtake execution authority: decision=" << trace.request.decision_id
         << ", episode=" << trace.request.episode_id
         << ", generation=" << trace.request.mission_generation
         << ", target=" << (trace.request.target_id.empty() ? "<none>" : trace.request.target_id)
         << ", side=" << trace.request.pass_side_sign
         << ", phase=" << to_string(trace.request.phase)
         << ", behavior=" << to_string(trace.request.behavior)
         << ", action=" << to_string(trace.resolution.action)
         << ", lateral_owner=" << to_string(trace.resolution.lateral_owner)
         << ", longitudinal_owner=" << to_string(trace.resolution.longitudinal_owner)
         << ", path_source=" << to_string(trace.resolution.path_source)
         << ", path_age=" << finite_or(trace.request.path_age_sec, "inf") << "s"
         << ", line=" << (trace.request.line_active ? 1 : 0)
         << ", stage_corridor=" << (trace.request.stage_corridor_active ? 1 : 0)
         << ", gap=" << (trace.request.gap_planner_active ? 1 : 0)
         << ", dynamic_escape=" << (trace.request.dynamic_obstacle_escape_active ? 1 : 0)
         << ", mission_wait=" << (trace.request.dynamic_wait_active ? 1 : 0)
         << "/lateral=" <<
    (trace.request.dynamic_wait_lateral_authority_active ? 1 : 0)
         << "/forward=" << (trace.request.dynamic_wait_forward_prefix_active ? 1 : 0)
         << ", contact=" << (trace.request.contact_continuation_active ? 1 : 0)
         << "/precontact=" << (trace.request.precontact_escape_active ? 1 : 0)
         << ", safety="
         << ((trace.request.behavior == Behavior::SafetyBrake ||
              trace.request.emergency_brake_active) ? 1 : 0)
         << ", solver_fallback=" << (trace.request.solver_fallback_active ? 1 : 0)
         << ", speed=" << finite_or(trace.request.speed_reference_mps, "inf")
         << "/" << finite_or(trace.request.speed_limit_mps, "inf")
         << "/" << finite_or(trace.request.speed_floor_mps, "nan")
         << ", floor_request=" <<
    finite_or(trace.request.requested_speed_floor_mps, "nan")
         << "/adjusted=" << (trace.request.speed_floor_adjusted ? 1 : 0)
         << ", shiftout_speed_contract="
         << (trace.request.shiftout_speed_contract_expected ? 1 : 0) << "/"
         << (trace.request.shiftout_speed_contract_active ? 1 : 0)
         << "/ref=" << finite_or(
    trace.request.shiftout_speed_contract_reference_mps, "inf")
         << "/overspeed=" << finite_or(
    trace.request.shiftout_speed_contract_overspeed_mps, "nan") << "m/s"
         << ", front=" << finite_or(trace.request.front_distance_m, "inf")
         << "/safety=" <<
    finite_or(trace.request.dynamic_front_safety_distance_m, "inf")
         << "/protected=" <<
    finite_or(trace.request.protected_front_distance_m, "inf")
         << "m, closing_ref=" <<
    finite_or(trace.request.closing_speed_reference_mps, "inf") << "m/s"
         << ", wall_contract=" <<
    finite_or(trace.request.wall_contract_minimum_path_clearance_m, "inf")
         << "/" <<
    finite_or(trace.request.wall_contract_required_clearance_m, "nan")
         << ", corridor_min="
         << finite_or(trace.constrained_corridor.minimum_width_m, "nan")
         << "@" << finite_or(trace.constrained_corridor.minimum_width_distance_m, "nan")
         << "m, wall_min=" << finite_or(trace.wall_corridor.minimum_width_m, "nan")
         << "m, valid_until=" << finite_or(trace.static_valid_until_m, "nan")
         << "/" << finite_or(trace.dynamic_valid_until_m, "nan")
         << "m, rear_clear=" << finite_or(trace.predicted_rear_clear_m, "inf")
         << "m, ego=" << finite_or(trace.ego_speed_mps, "nan")
         << "m/s, conflict=" << format_conflicts(trace.resolution.conflicts)
         << ", reason=" << trace.resolution.reason
         << ", transition=\""
         << (trace.request.transition_reason.empty() ? "none" :
      trace.request.transition_reason) << "\""
         << ", block=\""
         << (trace.request.blocking_reason.empty() ? "none" :
      trace.request.blocking_reason) << "\""
         << ", wp_id=" << trace.waypoint_id;
  return stream.str();
}

TraceEmission ChangeAwareAuthorityTraceEmitter::update(
  const AuthorityTrace & trace, const double now_sec,
  const double repeat_interval_sec)
{
  TraceEmission emission;
  AuthorityTrace effective_trace = trace;
  if (!trace.resolution.relevant && !was_relevant_) {
    return emission;
  }
  emission.signature = categorical_signature(effective_trace);
  emission.state_changed = emission.signature != last_signature_;
  const bool repeat_due =
    std::isfinite(now_sec) && std::isfinite(last_emit_sec_) &&
    std::isfinite(repeat_interval_sec) && repeat_interval_sec >= 0.0 &&
    now_sec >= last_emit_sec_ && now_sec - last_emit_sec_ >= repeat_interval_sec;
  emission.emit = emission.state_changed || repeat_due;
  emission.conflict = effective_trace.resolution.conflicts != NoConflict;
  if (emission.emit) {
    emission.message = format_authority_trace(effective_trace);
    last_signature_ = emission.signature;
    last_emit_sec_ = now_sec;
  }
  was_relevant_ = trace.resolution.relevant;
  return emission;
}

void ChangeAwareAuthorityTraceEmitter::reset() noexcept
{
  last_signature_.clear();
  last_emit_sec_ = -std::numeric_limits<double>::infinity();
  was_relevant_ = false;
}

FinalControlSource resolve_final_control_source(
  const FinalControlSourceRequest & request) noexcept
{
  if (request.failsafe_active) {
    return FinalControlSource::Failsafe;
  }
  if (request.stuck_recovery_active) {
    return FinalControlSource::StuckRecovery;
  }
  if (!request.control_enabled) {
    return FinalControlSource::ControlDisabled;
  }
  if (request.low_speed_wall_stop_active) {
    return FinalControlSource::LowSpeedWallStop;
  }
  if (request.solver_wall_handoff_hold_active) {
    return FinalControlSource::SolverWallHandoffHold;
  }
  if (request.overtake_wall_admission_hold_active) {
    return FinalControlSource::OvertakeWallAdmissionHold;
  }
  if (request.executed_solution_wall_hold_active) {
    return FinalControlSource::ExecutedSolutionWallHold;
  }
  if (request.solver_bounded_continuation_active) {
    return FinalControlSource::SolverBoundedContinuation;
  }
  if (request.solver_crawl_active) {
    return FinalControlSource::SolverCrawl;
  }
  if (request.solver_fallback_active || request.forced_stop_active) {
    return FinalControlSource::SolverFallback;
  }
  if (request.low_speed_direct_active) {
    return FinalControlSource::LowSpeedDirect;
  }
  return FinalControlSource::MpcSolution;
}

const char * to_string(const FinalControlSource source) noexcept
{
  switch (source) {
    case FinalControlSource::MpcSolution: return "mpc-solution";
    case FinalControlSource::LowSpeedDirect: return "low-speed-direct";
    case FinalControlSource::LowSpeedWallStop: return "low-speed-wall-stop";
    case FinalControlSource::SolverFallback: return "solver-fallback";
    case FinalControlSource::SolverBoundedContinuation:
      return "solver-bounded-continuation";
    case FinalControlSource::SolverWallHandoffHold:
      return "solver-wall-handoff-hold";
    case FinalControlSource::OvertakeWallAdmissionHold:
      return "overtake-wall-admission-hold";
    case FinalControlSource::ExecutedSolutionWallHold:
      return "executed-solution-wall-hold";
    case FinalControlSource::SolverCrawl: return "solver-crawl";
    case FinalControlSource::ControlDisabled: return "control-disabled";
    case FinalControlSource::StuckRecovery: return "stuck-recovery";
    case FinalControlSource::Failsafe: return "failsafe";
  }
  return "unknown";
}

std::string final_control_signature(const FinalControlTrace & trace)
{
  std::ostringstream stream;
  if (trace.authority.has_value()) {
    stream << categorical_signature(trace.authority.value());
  } else {
    stream << "no-authority";
  }
  stream << "|" << static_cast<int>(trace.control_source)
         << "|" << (trace.published ? 1 : 0);
  if (
    trace.control_source == FinalControlSource::SolverFallback ||
    trace.control_source == FinalControlSource::SolverBoundedContinuation ||
    trace.control_source == FinalControlSource::SolverWallHandoffHold ||
    trace.control_source == FinalControlSource::OvertakeWallAdmissionHold ||
    trace.control_source == FinalControlSource::ExecutedSolutionWallHold ||
    trace.control_source == FinalControlSource::Failsafe)
  {
    stream << "|" << trace.solver_reason << "|" << trace.output_reason;
  }
  return stream.str();
}

namespace {

std::string structural_final_control_signature(const FinalControlTrace & trace)
{
  std::ostringstream stream;
  if (trace.authority.has_value()) {
    const auto & authority = trace.authority.value();
    stream << (authority.resolution.relevant ? 1 : 0) << "|"
           << authority.request.episode_id << "|"
           << authority.request.mission_generation << "|"
           << authority.request.target_id << "|"
           << static_cast<int>(authority.request.phase) << "|"
           << static_cast<int>(authority.request.behavior) << "|"
           << static_cast<int>(authority.resolution.action) << "|"
           << static_cast<int>(authority.resolution.lateral_owner) << "|"
           << static_cast<int>(authority.resolution.path_source) << "|"
           << authority.request.pass_side_sign << "|"
           << authority.resolution.conflicts << "|"
           << (authority.request.dynamic_wait_lateral_authority_active ? 1 : 0)
           << "|" << (authority.request.contact_continuation_active ? 1 : 0)
           << "|" << (authority.request.precontact_escape_active ? 1 : 0)
           << "|" << (authority.request.speed_floor_adjusted ? 1 : 0);
  } else {
    stream << "no-authority";
  }
  stream << "|" << static_cast<int>(trace.control_source)
         << "|" << (trace.published ? 1 : 0);
  if (
    trace.control_source == FinalControlSource::SolverFallback ||
    trace.control_source == FinalControlSource::SolverBoundedContinuation ||
    trace.control_source == FinalControlSource::SolverWallHandoffHold ||
    trace.control_source == FinalControlSource::OvertakeWallAdmissionHold ||
    trace.control_source == FinalControlSource::ExecutedSolutionWallHold ||
    trace.control_source == FinalControlSource::Failsafe)
  {
    stream << "|" << trace.solver_reason << "|" << trace.output_reason;
  }
  return stream.str();
}

}  // namespace

std::string format_final_control_trace(const FinalControlTrace & trace)
{
  std::ostringstream stream;
  stream << "Overtake control decision: decision=" << trace.decision_id;
  if (trace.authority.has_value()) {
    const auto & authority = trace.authority.value();
    stream << ", episode=" << authority.request.episode_id
           << ", generation=" << authority.request.mission_generation
           << ", target="
           << (authority.request.target_id.empty() ? "<none>" :
      authority.request.target_id)
           << ", side=" << authority.request.pass_side_sign
           << ", phase=" << to_string(authority.request.phase)
           << ", action=" << to_string(authority.resolution.action)
           << ", lateral_owner=" << to_string(authority.resolution.lateral_owner)
           << ", longitudinal_owner="
           << to_string(authority.resolution.longitudinal_owner)
           << ", path_source=" << to_string(authority.resolution.path_source)
           << ", path_age=" << finite_or(authority.request.path_age_sec, "inf") << "s"
           << ", speed_window="
           << finite_or(authority.request.speed_reference_mps, "inf") << "/"
           << finite_or(authority.request.speed_limit_mps, "inf") << "/"
           << finite_or(authority.request.speed_floor_mps, "nan")
           << ", floor_request="
           << finite_or(authority.request.requested_speed_floor_mps, "nan")
           << "/adjusted=" << (authority.request.speed_floor_adjusted ? 1 : 0)
           << ", front=" << finite_or(authority.request.front_distance_m, "inf")
           << "/safety="
           << finite_or(authority.request.dynamic_front_safety_distance_m, "inf")
           << "/protected="
           << finite_or(authority.request.protected_front_distance_m, "inf")
           << "m, closing_ref="
           << finite_or(authority.request.closing_speed_reference_mps, "inf")
           << "m/s"
           << ", conflict=" << format_conflicts(authority.resolution.conflicts)
           << ", transition=\""
           << (authority.request.transition_reason.empty() ? "none" :
      authority.request.transition_reason) << "\""
           << ", block=\""
           << (authority.request.blocking_reason.empty() ? "none" :
      authority.request.blocking_reason) << "\"";
    stream << ", corridor_min="
           << finite_or(authority.constrained_corridor.minimum_width_m, "nan")
           << "@"
           << finite_or(
      authority.constrained_corridor.minimum_width_distance_m, "nan")
           << "m, wall_min="
           << finite_or(authority.wall_corridor.minimum_width_m, "nan")
           << "m, valid_until=" << finite_or(authority.static_valid_until_m, "nan")
           << "/" << finite_or(authority.dynamic_valid_until_m, "nan")
           << "m, rear_clear=" << finite_or(authority.predicted_rear_clear_m, "inf")
           << "m, wall_contract="
           << finite_or(
      authority.request.wall_contract_minimum_path_clearance_m, "inf")
           << "/"
           << finite_or(authority.request.wall_contract_required_clearance_m, "nan")
           << "m, wp_id=" << authority.waypoint_id;
  } else {
    stream << ", episode=0, target=<none>, phase=unknown, authority=unavailable";
  }
  stream << ", control_source=" << to_string(trace.control_source)
         << ", actual=" << finite_or(trace.actual_speed_mps, "nan") << "m/s"
         << ", command=" << finite_or(trace.target_speed_mps, "nan") << "m/s/"
         << finite_or(trace.acceleration_mps2, "nan") << "m/s2/"
         << finite_or(trace.raw_steering_rad, "nan") << "rad"
         << ", published_steering="
         << finite_or(trace.published_steering_rad, "nan") << "rad"
         << ", published=" << (trace.published ? 1 : 0)
         << ", solver=\""
         << (trace.solver_reason.empty() ? "none" : trace.solver_reason) << "\""
         << ", output=\""
         << (trace.output_reason.empty() ? "none" : trace.output_reason) << "\"";
  return stream.str();
}

FinalTraceEmission ChangeAwareFinalControlTraceEmitter::update(
  const FinalControlTrace & trace, const double now_sec,
  const double repeat_interval_sec)
{
  FinalTraceEmission emission;
  const bool authority_relevant =
    trace.authority.has_value() && trace.authority->resolution.relevant;
  const bool exceptional_source =
    trace.control_source != FinalControlSource::MpcSolution;
  const bool relevant = authority_relevant || exceptional_source;
  if (!relevant && !was_relevant_) {
    return emission;
  }
  const std::string detail_signature = final_control_signature(trace);
  emission.signature = structural_final_control_signature(trace);
  emission.state_changed = emission.signature != last_signature_;
  const bool detail_changed = detail_signature != last_detail_signature_;
  const bool repeat_due =
    std::isfinite(now_sec) && std::isfinite(last_emit_sec_) &&
    std::isfinite(repeat_interval_sec) && repeat_interval_sec >= 0.0 &&
    now_sec >= last_emit_sec_ && now_sec - last_emit_sec_ >= repeat_interval_sec;
  emission.warning =
    trace.control_source == FinalControlSource::SolverFallback ||
    trace.control_source == FinalControlSource::SolverBoundedContinuation ||
    trace.control_source == FinalControlSource::SolverWallHandoffHold ||
    trace.control_source == FinalControlSource::OvertakeWallAdmissionHold ||
    trace.control_source == FinalControlSource::ExecutedSolutionWallHold ||
    trace.control_source == FinalControlSource::LowSpeedWallStop ||
    trace.control_source == FinalControlSource::Failsafe ||
    (trace.authority.has_value() &&
    (trace.authority->resolution.conflicts != NoConflict ||
    trace.authority->request.speed_floor_adjusted));
  if (detail_changed && !emission.state_changed) {
    ++suppressed_normal_change_count_;
  }
  // Every warning category belongs to the structural signature above, so its
  // entry/change is immediate. Routine longitudinal-owner and speed-window
  // category chatter is counted and reported by the next structural event or
  // heartbeat instead of producing one line per control cycle.
  emission.emit = emission.state_changed || repeat_due;
  if (emission.emit) {
    emission.message = format_final_control_trace(trace);
    emission.suppressed_normal_change_count = suppressed_normal_change_count_;
    if (suppressed_normal_change_count_ > 0U) {
      emission.message += ", suppressed_normal_changes=" +
        std::to_string(suppressed_normal_change_count_);
    }
    suppressed_normal_change_count_ = 0U;
    last_signature_ = emission.signature;
    last_emit_sec_ = now_sec;
  }
  last_detail_signature_ = detail_signature;
  was_relevant_ = relevant;
  return emission;
}

void ChangeAwareFinalControlTraceEmitter::reset() noexcept
{
  last_signature_.clear();
  last_detail_signature_.clear();
  last_emit_sec_ = -std::numeric_limits<double>::infinity();
  was_relevant_ = false;
  suppressed_normal_change_count_ = 0U;
}

const char * to_string(const WallRiskState state) noexcept
{
  switch (state) {
    case WallRiskState::Unknown: return "unknown";
    case WallRiskState::Clear: return "clear";
    case WallRiskState::Near: return "near";
    case WallRiskState::Contact: return "contact";
  }
  return "unknown";
}

const char * to_string(const WallHandoffAdmissionReason reason) noexcept
{
  switch (reason) {
    case WallHandoffAdmissionReason::Inactive: return "inactive";
    case WallHandoffAdmissionReason::CurrentFootprintUnavailable:
      return "current-footprint-unavailable";
    case WallHandoffAdmissionReason::CurrentFootprintUnsafe:
      return "current-footprint-unsafe";
    case WallHandoffAdmissionReason::PredictionUnavailable:
      return "prediction-unavailable";
    case WallHandoffAdmissionReason::PredictionInvalid:
      return "prediction-invalid";
    case WallHandoffAdmissionReason::PredictedContact:
      return "predicted-wall-contact";
    case WallHandoffAdmissionReason::PredictedOutOfMap:
      return "predicted-out-of-map";
    case WallHandoffAdmissionReason::InsufficientClearance:
      return "predicted-wall-clearance";
    case WallHandoffAdmissionReason::Requalifying: return "requalifying";
    case WallHandoffAdmissionReason::Accepted: return "accepted";
  }
  return "unknown";
}

WallHandoffAdmissionReason classify_wall_path_admission(
  const WallHandoffAdmissionRequest & request) noexcept
{
  if (!request.current_footprint_valid || request.current_footprint_out_of_map) {
    return WallHandoffAdmissionReason::CurrentFootprintUnavailable;
  }
  if (!request.current_footprint_clear || request.current_contact_count > 0U) {
    return WallHandoffAdmissionReason::CurrentFootprintUnsafe;
  }
  if (!request.prediction.available) {
    return WallHandoffAdmissionReason::PredictionUnavailable;
  }
  if (!request.prediction.valid) {
    return WallHandoffAdmissionReason::PredictionInvalid;
  }
  if (request.prediction.contact) {
    return WallHandoffAdmissionReason::PredictedContact;
  }
  if (request.prediction.out_of_map) {
    return WallHandoffAdmissionReason::PredictedOutOfMap;
  }
  if (
    !std::isfinite(request.required_wall_clearance_m) ||
    request.required_wall_clearance_m < 0.0 ||
    !std::isfinite(request.physical_clearance_tolerance_m) ||
    request.physical_clearance_tolerance_m < 0.0 ||
    std::isnan(request.prediction.minimum_wall_distance_m))
  {
    return WallHandoffAdmissionReason::PredictionInvalid;
  }
  if (
    request.prediction.minimum_wall_distance_m +
    request.physical_clearance_tolerance_m + 1e-9 <
    request.required_wall_clearance_m)
  {
    return WallHandoffAdmissionReason::InsufficientClearance;
  }
  return WallHandoffAdmissionReason::Accepted;
}

WallHandoffAdmissionResolution WallPathAdmissionGate::update(
  const WallHandoffAdmissionRequest & request) noexcept
{
  WallHandoffAdmissionResolution resolution;
  resolution.required_consecutive_valid_cycles =
    std::max(1, request.required_consecutive_valid_cycles);
  if (request.activation_requested && !active_) {
    active_ = true;
    hold_cycles_ = 0;
    consecutive_valid_cycles_ = 0;
    resolution.entered = true;
  }
  if (!active_) {
    resolution.reason = WallHandoffAdmissionReason::Inactive;
    resolution.state_changed =
      previous_reason_ != WallHandoffAdmissionReason::Inactive;
    previous_reason_ = resolution.reason;
    return resolution;
  }

  ++hold_cycles_;
  resolution.active = true;
  resolution.hold_control = true;
  resolution.hold_cycles = hold_cycles_;
  if (!request.observation_updated) {
    resolution.reason = previous_reason_;
    resolution.stop_required = previous_stop_required_;
    resolution.consecutive_valid_cycles = consecutive_valid_cycles_;
    return resolution;
  }
  auto reject = [&] (const WallHandoffAdmissionReason reason, const bool stop) {
      consecutive_valid_cycles_ = 0;
      resolution.reason = reason;
      resolution.stop_required = stop;
    };
  const auto observation_reason = classify_wall_path_admission(request);
  const bool predicted_path_rejected =
    observation_reason == WallHandoffAdmissionReason::PredictionUnavailable ||
    observation_reason == WallHandoffAdmissionReason::PredictionInvalid ||
    observation_reason == WallHandoffAdmissionReason::PredictedContact ||
    observation_reason == WallHandoffAdmissionReason::PredictedOutOfMap ||
    observation_reason == WallHandoffAdmissionReason::InsufficientClearance;
  resolution.replan_required =
    resolution.entered && request.current_footprint_valid &&
    request.current_footprint_clear && predicted_path_rejected;
  resolution.planner_contract_admitted =
    request.planner_wall_contract_available &&
    std::isfinite(request.planner_minimum_corridor_reserve_m) &&
    std::isfinite(request.required_wall_clearance_m) &&
    request.planner_minimum_corridor_reserve_m + 1e-9 >=
    request.required_wall_clearance_m;
  resolution.planner_execution_contract_mismatch =
    resolution.planner_contract_admitted &&
    std::isfinite(request.prediction.minimum_wall_distance_m) &&
    request.prediction.minimum_wall_distance_m +
    std::max(0.0, request.physical_clearance_tolerance_m) + 1e-9 <
    request.required_wall_clearance_m;
  if (
    std::isfinite(request.prediction.minimum_wall_distance_m) &&
    std::isfinite(request.required_wall_clearance_m))
  {
    resolution.physical_clearance_shortfall_m = std::max(
      0.0, request.required_wall_clearance_m -
      request.prediction.minimum_wall_distance_m);
    resolution.boundary_clearance_accepted =
      observation_reason == WallHandoffAdmissionReason::Accepted &&
      resolution.physical_clearance_shortfall_m > 1e-9;
  }
  if (observation_reason != WallHandoffAdmissionReason::Accepted) {
    const bool stop_required =
      observation_reason ==
      WallHandoffAdmissionReason::CurrentFootprintUnavailable ||
      observation_reason == WallHandoffAdmissionReason::CurrentFootprintUnsafe;
    reject(observation_reason, stop_required);
  } else {
    ++consecutive_valid_cycles_;
    if (
      consecutive_valid_cycles_ >=
      resolution.required_consecutive_valid_cycles)
    {
      resolution.reason = WallHandoffAdmissionReason::Accepted;
      resolution.released = true;
      resolution.active = false;
      resolution.hold_control = false;
      active_ = false;
    } else {
      resolution.reason = WallHandoffAdmissionReason::Requalifying;
    }
  }
  resolution.consecutive_valid_cycles = consecutive_valid_cycles_;
  resolution.state_changed = resolution.entered || resolution.released ||
    resolution.reason != previous_reason_;
  previous_reason_ = resolution.reason;
  previous_stop_required_ = resolution.stop_required;
  return resolution;
}

bool WallPathAdmissionGate::active() const noexcept
{
  return active_;
}

void WallPathAdmissionGate::reset() noexcept
{
  active_ = false;
  hold_cycles_ = 0;
  consecutive_valid_cycles_ = 0;
  previous_reason_ = WallHandoffAdmissionReason::Inactive;
  previous_stop_required_ = false;
}

const char * to_string(const RetainedExecutionCursorReason reason) noexcept
{
  switch (reason) {
    case RetainedExecutionCursorReason::Available: return "available";
    case RetainedExecutionCursorReason::InvalidTiming: return "invalid-timing";
    case RetainedExecutionCursorReason::InvalidHorizon: return "invalid-horizon";
    case RetainedExecutionCursorReason::Expired: return "expired";
    case RetainedExecutionCursorReason::HorizonExhausted: return "horizon-exhausted";
  }
  return "unknown";
}

RetainedExecutionCursorResolution resolve_retained_execution_cursor(
  const RetainedExecutionCursorRequest & request) noexcept
{
  RetainedExecutionCursorResolution resolution;
  if (
    !std::isfinite(request.age_sec) || request.age_sec < 0.0 ||
    !std::isfinite(request.sample_period_sec) ||
    request.sample_period_sec <= 0.0 ||
    !std::isfinite(request.maximum_age_sec) || request.maximum_age_sec < 0.0)
  {
    resolution.reason = RetainedExecutionCursorReason::InvalidTiming;
    return resolution;
  }
  if (
    request.control_stage_count == 0U ||
    request.prediction_stage_count == 0U)
  {
    resolution.reason = RetainedExecutionCursorReason::InvalidHorizon;
    return resolution;
  }
  if (request.age_sec > request.maximum_age_sec + 1e-9) {
    resolution.reason = RetainedExecutionCursorReason::Expired;
    return resolution;
  }

  const auto elapsed_stage = static_cast<std::size_t>(std::floor(
      request.age_sec / request.sample_period_sec));
  if (
    elapsed_stage >= request.control_stage_count ||
    elapsed_stage >= request.prediction_stage_count)
  {
    resolution.reason = RetainedExecutionCursorReason::HorizonExhausted;
    return resolution;
  }

  resolution.available = true;
  resolution.control_stage_index = elapsed_stage;
  resolution.prediction_stage_index = elapsed_stage;
  resolution.remaining_control_stages =
    request.control_stage_count - elapsed_stage;
  resolution.remaining_prediction_stages =
    request.prediction_stage_count - elapsed_stage;
  resolution.reason = RetainedExecutionCursorReason::Available;
  return resolution;
}

const char * to_string(const DynamicEscapeAttemptReason reason) noexcept
{
  switch (reason) {
    case DynamicEscapeAttemptReason::Inactive: return "inactive";
    case DynamicEscapeAttemptReason::Started: return "started";
    case DynamicEscapeAttemptReason::PlannerRequested: return "planner-requested";
    case DynamicEscapeAttemptReason::ContinuationRequested:
      return "continuation-requested";
    case DynamicEscapeAttemptReason::TargetLossGrace: return "target-loss-grace";
    case DynamicEscapeAttemptReason::TargetLost: return "target-lost";
    case DynamicEscapeAttemptReason::TargetChanged: return "target-changed";
    case DynamicEscapeAttemptReason::ExplicitRelease: return "explicit-release";
  }
  return "unknown";
}

DynamicEscapeAttemptResolution DynamicEscapeAttemptTracker::update(
  const DynamicEscapeAttemptRequest & request) noexcept
{
  DynamicEscapeAttemptResolution resolution;
  resolution.target_loss_grace_sec =
    std::isfinite(request.target_loss_grace_sec) ?
    std::max(0.0, request.target_loss_grace_sec) : 0.0;
  const bool target_relevant =
    request.target_relevant && !request.target_id.empty();

  const auto fill_active_state = [&]() {
      resolution.attempt_id = attempt_id_;
      resolution.active = active_;
      resolution.target_id = target_id_;
      resolution.lifetime_cycles = lifetime_cycles_;
      resolution.planner_request_cycles = planner_request_cycles_;
      resolution.request_gap_cycles = request_gap_cycles_;
    };
  const auto release = [&](const DynamicEscapeAttemptReason reason) {
      resolution.attempt_id = attempt_id_;
      resolution.previous_target_id = target_id_;
      resolution.target_id = target_id_;
      resolution.released = active_;
      resolution.lifetime_cycles = lifetime_cycles_;
      resolution.planner_request_cycles = planner_request_cycles_;
      resolution.request_gap_cycles = request_gap_cycles_;
      active_ = false;
      attempt_id_ = 0U;
      target_id_.clear();
      last_target_relevant_sec_ = -std::numeric_limits<double>::infinity();
      lifetime_cycles_ = 0;
      planner_request_cycles_ = 0;
      request_gap_cycles_ = 0;
      resolution.reason = reason;
      resolution.state_changed = resolution.released || reason != previous_reason_;
      previous_reason_ = reason;
    };
  const auto start = [&](const bool retargeted) {
      active_ = true;
      attempt_id_ = allocate_attempt_id();
      target_id_ = request.target_id;
      last_target_relevant_sec_ = request.now_sec;
      lifetime_cycles_ = 1;
      planner_request_cycles_ = 1;
      request_gap_cycles_ = 0;
      resolution.started = true;
      resolution.retargeted = retargeted;
      resolution.planning_requested = true;
      resolution.continuation_requested = false;
      resolution.reason = retargeted ?
        DynamicEscapeAttemptReason::TargetChanged :
        DynamicEscapeAttemptReason::Started;
      resolution.state_changed = true;
      fill_active_state();
      previous_reason_ = resolution.reason;
    };

  if (request.explicit_release) {
    release(DynamicEscapeAttemptReason::ExplicitRelease);
    return resolution;
  }

  if (!active_) {
    if (request.planner_requested && target_relevant) {
      start(false);
    } else {
      resolution.reason = DynamicEscapeAttemptReason::Inactive;
      resolution.state_changed =
        previous_reason_ != DynamicEscapeAttemptReason::Inactive;
      previous_reason_ = resolution.reason;
    }
    return resolution;
  }

  if (target_relevant && request.target_id != target_id_) {
    resolution.previous_target_id = target_id_;
    resolution.released = true;
    if (request.planner_requested) {
      start(true);
      resolution.released = true;
      return resolution;
    }
    release(DynamicEscapeAttemptReason::TargetChanged);
    return resolution;
  }

  if (target_relevant) {
    if (std::isfinite(request.now_sec)) {
      last_target_relevant_sec_ = request.now_sec;
    }
    ++lifetime_cycles_;
    resolution.planning_requested = true;
    if (request.planner_requested) {
      ++planner_request_cycles_;
      resolution.reason = DynamicEscapeAttemptReason::PlannerRequested;
    } else {
      ++request_gap_cycles_;
      resolution.held_without_request = true;
      resolution.continuation_requested = true;
      resolution.reason = DynamicEscapeAttemptReason::ContinuationRequested;
    }
    resolution.state_changed = resolution.reason != previous_reason_;
    fill_active_state();
    previous_reason_ = resolution.reason;
    return resolution;
  }

  if (
    std::isfinite(request.now_sec) &&
    std::isfinite(last_target_relevant_sec_) &&
    request.now_sec >= last_target_relevant_sec_)
  {
    resolution.target_loss_age_sec =
      request.now_sec - last_target_relevant_sec_;
  }
  if (resolution.target_loss_age_sec <= resolution.target_loss_grace_sec) {
    ++lifetime_cycles_;
    ++request_gap_cycles_;
    resolution.held_without_request = true;
    resolution.planning_requested = true;
    resolution.continuation_requested = true;
    resolution.reason = DynamicEscapeAttemptReason::TargetLossGrace;
    resolution.state_changed = resolution.reason != previous_reason_;
    fill_active_state();
    previous_reason_ = resolution.reason;
    return resolution;
  }

  release(DynamicEscapeAttemptReason::TargetLost);
  return resolution;
}

bool DynamicEscapeAttemptTracker::active() const noexcept
{
  return active_;
}

std::uint64_t DynamicEscapeAttemptTracker::attempt_id() const noexcept
{
  return attempt_id_;
}

const std::string & DynamicEscapeAttemptTracker::target_id() const noexcept
{
  return target_id_;
}

void DynamicEscapeAttemptTracker::reset() noexcept
{
  attempt_id_ = 0U;
  active_ = false;
  target_id_.clear();
  last_target_relevant_sec_ = -std::numeric_limits<double>::infinity();
  lifetime_cycles_ = 0;
  planner_request_cycles_ = 0;
  request_gap_cycles_ = 0;
  previous_reason_ = DynamicEscapeAttemptReason::Inactive;
}

std::uint64_t DynamicEscapeAttemptTracker::allocate_attempt_id() noexcept
{
  const std::uint64_t allocated = next_attempt_id_++;
  if (next_attempt_id_ == 0U) {
    next_attempt_id_ = 1U;
  }
  return allocated == 0U ? allocate_attempt_id() : allocated;
}

std::string format_dynamic_escape_attempt_trace(
  const DynamicEscapeAttemptRequest & request,
  const DynamicEscapeAttemptResolution & resolution,
  const int waypoint_id)
{
  const char * event = resolution.retargeted ? "retargeted" :
    (resolution.started ? "started" :
    (resolution.released ? "released" :
    (resolution.active ? "heartbeat" : "idle")));
  std::ostringstream stream;
  stream << "Dynamic escape attempt lifecycle: event=" << event
         << ", reason=" << to_string(resolution.reason)
         << ", attempt=" << resolution.attempt_id
         << ", active=" << (resolution.active ? 1 : 0)
         << ", target="
         << (resolution.target_id.empty() ? "none" : resolution.target_id)
         << ", previous_target="
         << (resolution.previous_target_id.empty() ?
    "none" : resolution.previous_target_id)
         << ", planner_requested=" << (request.planner_requested ? 1 : 0)
         << ", effective_planning="
         << (resolution.planning_requested ? 1 : 0)
         << ", continuation="
         << (resolution.continuation_requested ? 1 : 0)
         << ", target_relevant=" << (request.target_relevant ? 1 : 0)
         << ", held_without_request="
         << (resolution.held_without_request ? 1 : 0)
         << ", cycles=" << resolution.lifetime_cycles
         << "/request=" << resolution.planner_request_cycles
         << "/gap=" << resolution.request_gap_cycles
         << ", target_loss_age="
         << finite_or(resolution.target_loss_age_sec, "inf") << "s/"
         << std::fixed << std::setprecision(2)
         << resolution.target_loss_grace_sec << "s"
         << ", wp_id=" << waypoint_id;
  return stream.str();
}

const char * to_string(const DynamicEscapeExitReason reason) noexcept
{
  switch (reason) {
    case DynamicEscapeExitReason::Inactive: return "inactive";
    case DynamicEscapeExitReason::TargetBlocking: return "target-blocking";
    case DynamicEscapeExitReason::RetainedSolutionExpired:
      return "retained-solution-expired";
    case DynamicEscapeExitReason::WallPathPending: return "wall-path-pending";
    case DynamicEscapeExitReason::ReplacementPending:
      return "replacement-pending";
    case DynamicEscapeExitReason::ReplacementExecuting:
      return "replacement-executing";
    case DynamicEscapeExitReason::ReplacementLost:
      return "replacement-lost";
    case DynamicEscapeExitReason::TargetResolvedRequalifying:
      return "target-resolved-requalifying";
    case DynamicEscapeExitReason::TargetResolved: return "target-resolved";
    case DynamicEscapeExitReason::RecoveryOverride: return "recovery-override";
  }
  return "unknown";
}

DynamicEscapeExitResolution DynamicEscapeExitGate::update(
  const DynamicEscapeExitRequest & request) noexcept
{
  DynamicEscapeExitResolution resolution;
  resolution.required_consecutive_resolved_cycles =
    std::max(1, request.required_consecutive_resolved_cycles);

  if (request.recovery_override) {
    const bool was_active = active_;
    reset();
    resolution.released = was_active;
    resolution.state_changed = was_active;
    resolution.reason = DynamicEscapeExitReason::RecoveryOverride;
    return resolution;
  }

  const bool valid_request_attempt = request.escape_attempt_id > 0U;
  const bool attempt_changed =
    request.observation_updated && active_ && valid_request_attempt &&
    latched_attempt_id_ > 0U &&
    request.escape_attempt_id != latched_attempt_id_;
  if (
    request.observation_updated &&
    ((request.activation_requested && !active_) || attempt_changed))
  {
    active_ = true;
    latched_attempt_id_ = request.escape_attempt_id;
    replacement_execution_active_ = false;
    hold_cycles_ = 0;
    consecutive_resolved_cycles_ = 0;
    resolution.entered = true;
    resolution.attempt_changed = attempt_changed;
  } else if (
    request.observation_updated && active_ && latched_attempt_id_ == 0U &&
    valid_request_attempt)
  {
    latched_attempt_id_ = request.escape_attempt_id;
  }
  resolution.latched_attempt_id = latched_attempt_id_;
  if (!active_) {
    resolution.reason = DynamicEscapeExitReason::Inactive;
    resolution.state_changed =
      previous_reason_ != DynamicEscapeExitReason::Inactive;
    previous_reason_ = resolution.reason;
    previous_hold_lateral_control_ = false;
    return resolution;
  }

  ++hold_cycles_;
  resolution.active = true;
  resolution.replacement_execution_active = replacement_execution_active_;
  resolution.hold_cycles = hold_cycles_;
  if (!request.observation_updated) {
    resolution.reason = previous_reason_;
    resolution.hold_lateral_control = previous_hold_lateral_control_;
    resolution.consecutive_resolved_cycles = consecutive_resolved_cycles_;
    return resolution;
  }

  if (
    request.obstacle_blocking && request.replacement_escape_active &&
    request.replacement_escape_admitted)
  {
    resolution.replacement_adopted = !replacement_execution_active_;
    replacement_execution_active_ = true;
    resolution.replacement_execution_active = true;
    resolution.reason = DynamicEscapeExitReason::ReplacementExecuting;
    consecutive_resolved_cycles_ = 0;
  } else if (request.obstacle_blocking) {
    const bool replacement_lost = replacement_execution_active_;
    replacement_execution_active_ = false;
    resolution.replacement_execution_active = false;
    consecutive_resolved_cycles_ = 0;
    resolution.hold_lateral_control = request.retained_solution_available;
    resolution.continuation_required =
      request.attempt_active && request.continuation_planner_requested;
    // TargetBlocking is the reason DynamicEscape exists. Do not report it as
    // a wall/solver failure while the encounter lifecycle is already
    // evaluating a replacement. Failure replan remains reserved for a lost
    // replacement without an active continuation owner.
    resolution.replan_required =
      !resolution.continuation_required &&
      (resolution.entered || replacement_lost);
    if (request.replacement_escape_active) {
      resolution.reason = DynamicEscapeExitReason::ReplacementPending;
    } else if (replacement_lost) {
      resolution.reason = DynamicEscapeExitReason::ReplacementLost;
    } else if (request.retained_solution_available) {
      resolution.reason = DynamicEscapeExitReason::TargetBlocking;
    } else {
      resolution.reason = DynamicEscapeExitReason::RetainedSolutionExpired;
    }
  } else if (!request.wall_path_admitted) {
    consecutive_resolved_cycles_ = 0;
    resolution.hold_lateral_control = request.retained_solution_available;
    resolution.reason = DynamicEscapeExitReason::WallPathPending;
  } else {
    ++consecutive_resolved_cycles_;
    if (
      consecutive_resolved_cycles_ >=
      resolution.required_consecutive_resolved_cycles)
    {
      resolution.released = true;
      resolution.active = false;
      resolution.reason = DynamicEscapeExitReason::TargetResolved;
      active_ = false;
      latched_attempt_id_ = 0U;
      replacement_execution_active_ = false;
    } else {
      resolution.hold_lateral_control = request.retained_solution_available;
      resolution.reason =
        DynamicEscapeExitReason::TargetResolvedRequalifying;
    }
  }

  resolution.consecutive_resolved_cycles = consecutive_resolved_cycles_;
  resolution.state_changed = resolution.entered || resolution.released ||
    resolution.replacement_adopted || resolution.attempt_changed ||
    resolution.reason != previous_reason_ ||
    resolution.hold_lateral_control != previous_hold_lateral_control_ ||
    resolution.continuation_required != previous_continuation_required_;
  previous_reason_ = resolution.reason;
  previous_hold_lateral_control_ = resolution.hold_lateral_control;
  previous_continuation_required_ = resolution.continuation_required;
  return resolution;
}

bool DynamicEscapeExitGate::active() const noexcept
{
  return active_;
}

void DynamicEscapeExitGate::reset() noexcept
{
  active_ = false;
  latched_attempt_id_ = 0U;
  replacement_execution_active_ = false;
  hold_cycles_ = 0;
  consecutive_resolved_cycles_ = 0;
  previous_reason_ = DynamicEscapeExitReason::Inactive;
  previous_hold_lateral_control_ = false;
  previous_continuation_required_ = false;
}

std::string format_dynamic_escape_exit_trace(
  const std::uint64_t decision_id,
  const DynamicEscapeExitRequest & request,
  const DynamicEscapeExitResolution & resolution,
  const std::string & latched_target_id,
  const std::string & observed_target_id,
  const int latched_side_sign,
  const double front_distance_m,
  const double protected_front_distance_m,
  const double closing_speed_mps,
  const double retained_age_sec)
{
  const char * event = resolution.replacement_adopted ?
    "replacement-adopted" :
    (resolution.released ? "released" :
    (resolution.entered ? "entered" : "holding"));
  std::ostringstream stream;
  stream << "Dynamic escape exit contract: decision=" << decision_id
         << ", event=" << event
         << ", reason=" << to_string(resolution.reason)
         << ", attempt=" << request.escape_attempt_id << "/latched="
         << resolution.latched_attempt_id
         << ", active=" << (resolution.active ? 1 : 0)
         << ", hold_lateral="
         << (resolution.hold_lateral_control ? 1 : 0)
         << ", lifecycle=" << (request.attempt_active ? 1 : 0)
         << "/continuation="
         << (request.continuation_planner_requested ? 1 : 0)
         << "/required="
         << (resolution.continuation_required ? 1 : 0)
         << ", replan=" << (resolution.replan_required ? 1 : 0) << "/"
         << (resolution.replan_requested ? 1 : 0)
         << ", replacement="
         << (request.replacement_escape_active ? 1 : 0) << "/admitted="
         << (request.replacement_escape_admitted ? 1 : 0) << "/executing="
         << (resolution.replacement_execution_active ? 1 : 0)
         << ", target=" <<
    (latched_target_id.empty() ? "none" : latched_target_id)
         << "/observed=" <<
    (observed_target_id.empty() ? "none" : observed_target_id)
         << ", side=" << latched_side_sign
         << ", front=" << finite_or(front_distance_m, "inf") << "m"
         << ", protected=" <<
    finite_or(protected_front_distance_m, "inf") << "m"
         << ", closing=" << finite_or(closing_speed_mps, "nan") << "m/s"
         << ", obstacle_blocking=" << (request.obstacle_blocking ? 1 : 0)
         << ", wall_admitted=" << (request.wall_path_admitted ? 1 : 0)
         << ", retained=" <<
    (request.retained_solution_available ? 1 : 0)
         << "/age=" << finite_or(retained_age_sec, "inf") << "s"
         << ", resolved=" << resolution.consecutive_resolved_cycles << "/"
         << resolution.required_consecutive_resolved_cycles
         << ", holds=" << resolution.hold_cycles;
  return stream.str();
}

const char * to_string(const WallPathAdmissionScope scope) noexcept
{
  switch (scope) {
    case WallPathAdmissionScope::SolverHandoff: return "solver-handoff";
    case WallPathAdmissionScope::ActiveOvertake: return "active-overtake";
    case WallPathAdmissionScope::DynamicEscapeExit: return "dynamic-escape-exit";
  }
  return "unknown";
}

std::string format_wall_path_admission_trace(
  const std::uint64_t decision_id,
  const WallPathAdmissionScope scope, const Phase phase,
  const PathSource path_source,
  const WallHandoffAdmissionRequest & request,
  const WallHandoffAdmissionResolution & resolution,
  const double held_speed_mps, const double held_steering_rad)
{
  const char * event = resolution.released ? "released" :
    (resolution.entered ? "entered" :
    (resolution.reason == WallHandoffAdmissionReason::Requalifying ?
    "requalifying" : "blocked"));
  std::ostringstream stream;
  stream << "Wall path admission: decision=" << decision_id
         << ", mission_generation=" << request.mission_generation
         << ", scope=" << to_string(scope)
         << ", phase=" << to_string(phase)
         << ", path_source=" << to_string(path_source)
         << ", event=" << event
         << ", reason=" << to_string(resolution.reason)
         << ", active=" << (resolution.active ? 1 : 0)
         << ", hold=" << (resolution.hold_control ? 1 : 0)
         << ", stop=" << (resolution.stop_required ? 1 : 0)
         << ", replan=" << (resolution.replan_required ? 1 : 0) << "/"
         << (resolution.replan_requested ? 1 : 0)
         << ", valid=" << resolution.consecutive_valid_cycles << "/"
         << resolution.required_consecutive_valid_cycles
         << ", holds=" << resolution.hold_cycles
         << ", current=" << (request.current_footprint_valid ? 1 : 0) << "/"
         << (request.current_footprint_clear ? 1 : 0) << "/contact="
         << request.current_contact_count << "/out="
         << (request.current_footprint_out_of_map ? 1 : 0)
         << ", required="
         << finite_or(request.required_wall_clearance_m, "nan") << "m"
         << ", tolerance="
         << finite_or(request.physical_clearance_tolerance_m, "nan") << "m"
         << ", shortfall="
         << finite_or(resolution.physical_clearance_shortfall_m, "nan") << "m"
         << ", boundary_accept="
         << (resolution.boundary_clearance_accepted ? 1 : 0)
         << ", planner_contract="
         << (request.planner_wall_contract_available ? 1 : 0) << "/admitted="
         << (resolution.planner_contract_admitted ? 1 : 0)
         << "/metric=frenet-corridor-reserve/reserve="
         << finite_or(request.planner_minimum_corridor_reserve_m, "inf") << "m"
         << ", execution_contract_mismatch="
         << (resolution.planner_execution_contract_mismatch ? 1 : 0)
         << ", prediction=" << (request.prediction.available ? 1 : 0) << "/"
         << (request.prediction.valid ? 1 : 0) << "/"
         << request.prediction.sample_count
         << ", execution_metric=physical-footprint-distance"
         << ", path_wall=" << request.prediction.minimum_wall_region << "/"
         << finite_or(request.prediction.minimum_wall_distance_m, "inf")
         << "m@" << request.prediction.minimum_index << "/"
         << finite_or(
    request.prediction.minimum_wall_path_distance_m, "nan")
         << "m/contact=" << (request.prediction.contact ? 1 : 0)
         << "/out=" << (request.prediction.out_of_map ? 1 : 0)
         << ", command=" << finite_or(held_speed_mps, "nan") << "m/s/"
         << finite_or(held_steering_rad, "nan") << "rad";
  return stream.str();
}

const char * to_string(const ExecutedSolutionWallAction action) noexcept
{
  switch (action) {
    case ExecutedSolutionWallAction::Publish: return "publish";
    case ExecutedSolutionWallAction::EntryRollback: return "entry-rollback";
    case ExecutedSolutionWallAction::DynamicReplan: return "dynamic-replan";
    case ExecutedSolutionWallAction::RecoveryReplan: return "recovery-replan";
    case ExecutedSolutionWallAction::HoldCurrentPath: return "hold-current-path";
  }
  return "hold-current-path";
}

ExecutedSolutionWallResolution resolve_executed_solution_wall_action(
  const ExecutedSolutionWallRequest & request) noexcept
{
  ExecutedSolutionWallResolution resolution;
  if (
    !std::isfinite(request.phase_traveled_m) || request.phase_traveled_m < 0.0)
  {
    return resolution;
  }

  resolution.valid = true;
  if (!request.execution_context_active) {
    resolution.publish_solution = request.solution_wall_safe;
    resolution.action = request.solution_wall_safe ?
      ExecutedSolutionWallAction::Publish :
      ExecutedSolutionWallAction::HoldCurrentPath;
    return resolution;
  }
  if (request.solution_wall_safe) {
    resolution.publish_solution = true;
    resolution.action = ExecutedSolutionWallAction::Publish;
    return resolution;
  }

  switch (request.phase) {
    case Phase::ShiftOut:
      resolution.action =
        !request.execution_command_published ?
        ExecutedSolutionWallAction::EntryRollback :
        ExecutedSolutionWallAction::DynamicReplan;
      break;
    case Phase::Pass:
      resolution.action = ExecutedSolutionWallAction::DynamicReplan;
      break;
    case Phase::Return:
      resolution.action = ExecutedSolutionWallAction::RecoveryReplan;
      break;
    case Phase::FollowPrepare:
    case Phase::Recovery:
    case Phase::Idle:
      resolution.action = ExecutedSolutionWallAction::HoldCurrentPath;
      break;
  }
  return resolution;
}

WallRiskState classify_wall_risk(const WallHandoffProbe & probe) noexcept
{
  if (
    probe.current_footprint_out_of_map || probe.current_contact_count > 0U ||
    (probe.current_wall_valid && !probe.current_footprint_clear))
  {
    return WallRiskState::Contact;
  }
  if (!probe.current_wall_valid) {
    return WallRiskState::Unknown;
  }
  if (
    std::isfinite(probe.current_wall_distance_m) &&
    std::isfinite(probe.required_wall_clearance_m) &&
    probe.required_wall_clearance_m >= 0.0 &&
    probe.current_wall_distance_m <= probe.required_wall_clearance_m + 1e-9)
  {
    return WallRiskState::Near;
  }
  return WallRiskState::Clear;
}

namespace {

bool dynamic_handoff_relevant(const WallHandoffProbe & probe) noexcept
{
  return
    probe.dynamic_escape_active || probe.action == Action::DynamicEscape ||
    probe.path_source == PathSource::DynamicObstacleEscape ||
    probe.control_source == FinalControlSource::SolverBoundedContinuation ||
    probe.control_source == FinalControlSource::SolverWallHandoffHold ||
    probe.control_source == FinalControlSource::OvertakeWallAdmissionHold ||
    probe.control_source == FinalControlSource::ExecutedSolutionWallHold;
}

void append_trigger(std::ostringstream & stream, bool & first, const char * trigger)
{
  if (!first) {
    stream << "/";
  }
  stream << trigger;
  first = false;
}

}  // namespace

WallHandoffEvent ChangeAwareWallHandoffTraceEmitter::update(
  const WallHandoffProbe & probe, const double now_sec,
  const double monitor_duration_sec, const double risk_repeat_interval_sec)
{
  WallHandoffEvent event;
  event.previous_action = previous_action_;
  event.previous_lateral_owner = previous_lateral_owner_;
  event.previous_path_source = previous_path_source_;
  event.previous_control_source = previous_control_source_;
  event.risk = classify_wall_risk(probe);

  const bool dynamic_relevant = dynamic_handoff_relevant(probe);
  const bool monitor_was_active =
    std::isfinite(now_sec) && now_sec <= monitor_until_sec_;
  if (
    dynamic_relevant && std::isfinite(now_sec) &&
    std::isfinite(monitor_duration_sec) && monitor_duration_sec >= 0.0)
  {
    monitor_until_sec_ = std::max(
      monitor_until_sec_, now_sec + monitor_duration_sec);
  }
  event.monitor_active = dynamic_relevant || monitor_was_active ||
    (std::isfinite(now_sec) && now_sec <= monitor_until_sec_);
  event.source_changed = initialized_ &&
    (probe.action != previous_action_ ||
    probe.lateral_owner != previous_lateral_owner_ ||
    probe.path_source != previous_path_source_ ||
    probe.control_source != previous_control_source_);
  event.risk_changed = initialized_ && event.risk != previous_risk_;
  const bool entry = dynamic_relevant && (!initialized_ || !previous_dynamic_relevant_);
  const bool transition_touches_dynamic =
    dynamic_relevant || previous_dynamic_relevant_ || monitor_was_active;
  const bool risky =
    event.risk == WallRiskState::Near || event.risk == WallRiskState::Contact;
  const bool risk_repeat_due =
    risky && event.monitor_active && std::isfinite(now_sec) &&
    std::isfinite(risk_repeat_interval_sec) && risk_repeat_interval_sec >= 0.0 &&
    now_sec >= last_risk_emit_sec_ &&
    now_sec - last_risk_emit_sec_ >= risk_repeat_interval_sec;

  event.emit = entry ||
    (event.source_changed && transition_touches_dynamic) ||
    (event.monitor_active && risky && (event.risk_changed || risk_repeat_due));
  event.warning = risky ||
    probe.control_source == FinalControlSource::SolverBoundedContinuation;
  if (event.emit) {
    std::ostringstream trigger;
    bool first = true;
    if (entry) {
      append_trigger(trigger, first, "dynamic-entry");
    }
    if (event.source_changed) {
      append_trigger(trigger, first, "source-transition");
    }
    if (event.risk == WallRiskState::Near) {
      append_trigger(trigger, first, "wall-risk");
    } else if (event.risk == WallRiskState::Contact) {
      append_trigger(trigger, first, "wall-contact");
    }
    event.trigger = first ? "monitor-heartbeat" : trigger.str();
    if (risky && std::isfinite(now_sec)) {
      last_risk_emit_sec_ = now_sec;
    }
  }

  initialized_ = true;
  previous_dynamic_relevant_ = dynamic_relevant;
  previous_action_ = probe.action;
  previous_lateral_owner_ = probe.lateral_owner;
  previous_path_source_ = probe.path_source;
  previous_control_source_ = probe.control_source;
  previous_risk_ = event.risk;
  return event;
}

std::string format_wall_handoff_trace(
  const WallHandoffProbe & probe, const WallHandoffEvent & event,
  const PredictedPathWallMetrics & path_metrics)
{
  std::ostringstream stream;
  stream << "DynamicEscape wall handoff: decision=" << probe.decision_id
         << ", trigger=" << event.trigger
         << ", monitor=" << (event.monitor_active ? 1 : 0)
         << ", from=" << to_string(event.previous_action) << "/"
         << to_string(event.previous_lateral_owner) << "/"
         << to_string(event.previous_path_source) << "/"
         << to_string(event.previous_control_source)
         << ", to=" << to_string(probe.action) << "/"
         << to_string(probe.lateral_owner) << "/"
         << to_string(probe.path_source) << "/"
         << to_string(probe.control_source)
         << ", risk=" << to_string(event.risk)
         << ", current_wall=" << probe.current_wall_region << "/"
         << finite_or(probe.current_wall_distance_m, "inf")
         << "m/contact=" << probe.current_contact_count
         << "/clear=" << (probe.current_footprint_clear ? 1 : 0)
         << "/out=" << (probe.current_footprint_out_of_map ? 1 : 0)
         << ", required=" << finite_or(probe.required_wall_clearance_m, "nan")
         << "m, prediction=" << (path_metrics.available ? 1 : 0) << "/"
         << (path_metrics.valid ? 1 : 0) << "/"
         << path_metrics.sample_count << "/retained="
         << (path_metrics.retained_solution ? 1 : 0)
         << ", path_wall=" << path_metrics.minimum_wall_region << "/"
         << finite_or(path_metrics.minimum_wall_distance_m, "inf") << "m@"
         << path_metrics.minimum_index << "/"
         << finite_or(path_metrics.minimum_wall_path_distance_m, "nan")
         << "m/contact=" << (path_metrics.contact ? 1 : 0)
         << "/out=" << (path_metrics.out_of_map ? 1 : 0)
         << ", pose=" << finite_or(probe.pose_x_m, "nan") << "/"
         << finite_or(probe.pose_y_m, "nan") << "/"
         << finite_or(probe.pose_yaw_rad, "nan")
         << ", tracking=" << finite_or(probe.lateral_error_m, "nan") << "/"
         << finite_or(probe.heading_error_rad, "nan")
         << ", motion=" << finite_or(probe.speed_mps, "nan") << "/"
         << finite_or(probe.yaw_rate_radps, "nan")
         << ", steering=" << finite_or(probe.raw_steering_rad, "nan") << "/"
         << finite_or(probe.published_steering_rad, "nan") << "/"
         << finite_or(probe.previous_published_steering_rad, "nan") << "/delta="
         << finite_or(
    probe.published_steering_rad - probe.previous_published_steering_rad, "nan")
         << ", collision_age=" << finite_or(probe.collision_age_sec, "inf")
         << "s";
  return stream.str();
}

void ChangeAwareWallHandoffTraceEmitter::reset() noexcept
{
  initialized_ = false;
  previous_dynamic_relevant_ = false;
  previous_action_ = Action::Cruise;
  previous_lateral_owner_ = LateralOwner::RacingLine;
  previous_path_source_ = PathSource::RacingLine;
  previous_control_source_ = FinalControlSource::MpcSolution;
  previous_risk_ = WallRiskState::Unknown;
  monitor_until_sec_ = -std::numeric_limits<double>::infinity();
  last_risk_emit_sec_ = -std::numeric_limits<double>::infinity();
}

void EpisodeAccumulator::begin(const EpisodeStart & start)
{
  reset();
  if (start.episode_id == 0U || !std::isfinite(start.now_sec)) {
    return;
  }
  active_ = true;
  start_ = start;
  target_id_ = start.target_id;
}

void EpisodeAccumulator::observe(const EpisodeSample & sample)
{
  if (!active_ || sample.episode_id != start_.episode_id) {
    return;
  }
  if (!sample.target_id.empty()) {
    target_id_ = sample.target_id;
  }
  if (finite_nonnegative(sample.ego_speed_mps)) {
    minimum_speed_mps_ = std::min(minimum_speed_mps_, sample.ego_speed_mps);
  }
  if (sample.constrained_corridor.valid) {
    minimum_constrained_corridor_width_m_ = std::min(
      minimum_constrained_corridor_width_m_,
      sample.constrained_corridor.minimum_width_m);
  }
  if (sample.wall_corridor.valid) {
    minimum_wall_corridor_width_m_ = std::min(
      minimum_wall_corridor_width_m_, sample.wall_corridor.minimum_width_m);
  }
  if (finite_nonnegative(sample.maximum_required_lateral_accel_mps2)) {
    maximum_required_lateral_accel_mps2_ = std::max(
      maximum_required_lateral_accel_mps2_,
      sample.maximum_required_lateral_accel_mps2);
  }
  maximum_mission_generation_ = std::max(
    maximum_mission_generation_, sample.mission_generation);
  phase_mask_ |= 1U << static_cast<unsigned>(sample.phase);
  if (
    !authority_initialized_ || sample.action != previous_action_ ||
    sample.lateral_owner != previous_lateral_owner_ ||
    sample.longitudinal_owner != previous_longitudinal_owner_)
  {
    ++authority_change_count_;
    authority_initialized_ = true;
    previous_action_ = sample.action;
    previous_lateral_owner_ = sample.lateral_owner;
    previous_longitudinal_owner_ = sample.longitudinal_owner;
  }
  if (sample.dynamic_wait_active && !previous_dynamic_wait_active_) {
    ++dynamic_wait_entry_count_;
  }
  if (sample.contact_escape_active && !previous_contact_escape_active_) {
    ++contact_escape_entry_count_;
  }
  previous_dynamic_wait_active_ = sample.dynamic_wait_active;
  previous_contact_escape_active_ = sample.contact_escape_active;
  if (sample.authority_conflicts != NoConflict) {
    ++authority_conflict_sample_count_;
  }
}

std::optional<EpisodeSummary> EpisodeAccumulator::finish(
  const double now_sec, const std::string & final_phase,
  const std::string & final_reason, const int final_waypoint_id)
{
  if (!active_) {
    return std::nullopt;
  }
  EpisodeSummary summary;
  summary.valid = true;
  summary.episode_id = start_.episode_id;
  summary.target_id = target_id_;
  summary.side = start_.side;
  summary.elapsed_sec =
    std::isfinite(now_sec) && now_sec >= start_.now_sec ?
    now_sec - start_.now_sec : 0.0;
  summary.minimum_speed_mps = minimum_speed_mps_;
  summary.minimum_constrained_corridor_width_m =
    minimum_constrained_corridor_width_m_;
  summary.minimum_wall_corridor_width_m = minimum_wall_corridor_width_m_;
  summary.maximum_required_lateral_accel_mps2 =
    maximum_required_lateral_accel_mps2_;
  summary.maximum_mission_generation = maximum_mission_generation_;
  summary.authority_change_count = authority_change_count_;
  summary.dynamic_wait_entry_count = dynamic_wait_entry_count_;
  summary.contact_escape_entry_count = contact_escape_entry_count_;
  summary.authority_conflict_sample_count = authority_conflict_sample_count_;
  std::ostringstream phases;
  bool first = true;
  for (unsigned value = static_cast<unsigned>(Phase::Idle);
    value <= static_cast<unsigned>(Phase::Recovery); ++value)
  {
    if ((phase_mask_ & (1U << value)) != 0U) {
      append_phase(phases, first, to_string(static_cast<Phase>(value)));
    }
  }
  summary.phases = phases.str();
  summary.final_phase = final_phase;
  summary.final_reason = final_reason;
  summary.start_waypoint_id = start_.waypoint_id;
  summary.final_waypoint_id = final_waypoint_id;
  reset();
  return summary;
}

bool EpisodeAccumulator::active() const noexcept
{
  return active_;
}

void EpisodeAccumulator::reset() noexcept
{
  active_ = false;
  start_ = EpisodeStart{};
  target_id_.clear();
  minimum_speed_mps_ = std::numeric_limits<double>::infinity();
  minimum_constrained_corridor_width_m_ = std::numeric_limits<double>::infinity();
  minimum_wall_corridor_width_m_ = std::numeric_limits<double>::infinity();
  maximum_required_lateral_accel_mps2_ = 0.0;
  maximum_mission_generation_ = 0U;
  authority_change_count_ = 0U;
  dynamic_wait_entry_count_ = 0U;
  contact_escape_entry_count_ = 0U;
  authority_conflict_sample_count_ = 0U;
  phase_mask_ = 0U;
  previous_dynamic_wait_active_ = false;
  previous_contact_escape_active_ = false;
  authority_initialized_ = false;
  previous_action_ = Action::Cruise;
  previous_lateral_owner_ = LateralOwner::RacingLine;
  previous_longitudinal_owner_ = LongitudinalOwner::RacingLine;
}

std::string format_episode_summary(const EpisodeSummary & summary)
{
  std::ostringstream stream;
  stream << "Overtake episode summary: episode=" << summary.episode_id
         << ", target=" << (summary.target_id.empty() ? "<none>" : summary.target_id)
         << ", side=" << summary.side
         << ", elapsed=" << finite_or(summary.elapsed_sec, "nan") << "s"
         << ", phases=" << (summary.phases.empty() ? "none" : summary.phases)
         << ", min_speed=" << finite_or(summary.minimum_speed_mps, "nan") << "m/s"
         << ", corridor_min="
         << finite_or(summary.minimum_constrained_corridor_width_m, "nan") << "m"
         << ", wall_min="
         << finite_or(summary.minimum_wall_corridor_width_m, "nan") << "m"
         << ", max_ay="
         << finite_or(summary.maximum_required_lateral_accel_mps2, "nan")
         << "m/s2, generation_max=" << summary.maximum_mission_generation
         << ", authority_changes=" << summary.authority_change_count
         << ", dynamic_waits=" << summary.dynamic_wait_entry_count
         << ", contact_escapes=" << summary.contact_escape_entry_count
         << ", conflict_samples=" << summary.authority_conflict_sample_count
         << ", final_phase=" << summary.final_phase
         << ", reason=\"" << summary.final_reason << "\""
         << ", wp_id=" << summary.start_waypoint_id << "->"
         << summary.final_waypoint_id;
  return stream.str();
}

}  // namespace multi_purpose_mpc_ros::overtake_execution_orchestrator
