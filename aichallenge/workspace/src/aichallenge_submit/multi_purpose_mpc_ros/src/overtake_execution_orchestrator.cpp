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
      request.dynamic_wait_active && request.dynamic_wait_forward_prefix_active)
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
    result.longitudinal_owner = LongitudinalOwner::SafetyBrake;
  } else if (request.solver_fallback_active) {
    result.longitudinal_owner = LongitudinalOwner::SolverFallback;
  } else if (request.pass_speed_floor_active) {
    result.longitudinal_owner = LongitudinalOwner::PassFloor;
  } else if (
    result.apply_overtake_speed_reference || result.apply_overtake_speed_limit ||
    request.shiftout_speed_floor_active)
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
    request.dynamic_wait_active && !request.dynamic_wait_forward_prefix_active &&
    !request.line_active && !safety_active && request.phase != Phase::Recovery)
  {
    result.conflicts |= DynamicWaitWithoutLateralAuthority;
  }
  if (active_phase && request.target_id.empty()) {
    result.conflicts |= ActivePhaseWithoutTarget;
  }
  const int lateral_source_count =
    (request.line_active ? 1 : 0) +
    (request.dynamic_obstacle_escape_active ? 1 : 0) +
    (request.gap_planner_active ? 1 : 0);
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
         << trace.resolution.conflicts << "|"
         << (trace.request.corridor_blocked ? 1 : 0) << "|"
         << (trace.request.dynamic_wait_active ? 1 : 0) << "|"
         << (trace.request.contact_continuation_active ? 1 : 0) << "|"
         << (trace.request.precontact_escape_active ? 1 : 0);
  return stream.str();
}

std::string format_authority_trace(const AuthorityTrace & trace)
{
  std::ostringstream stream;
  stream << "Overtake execution authority: episode=" << trace.request.episode_id
         << ", generation=" << trace.request.mission_generation
         << ", target=" << (trace.request.target_id.empty() ? "<none>" : trace.request.target_id)
         << ", phase=" << to_string(trace.request.phase)
         << ", behavior=" << to_string(trace.request.behavior)
         << ", action=" << to_string(trace.resolution.action)
         << ", lateral_owner=" << to_string(trace.resolution.lateral_owner)
         << ", longitudinal_owner=" << to_string(trace.resolution.longitudinal_owner)
         << ", line=" << (trace.request.line_active ? 1 : 0)
         << ", stage_corridor=" << (trace.request.stage_corridor_active ? 1 : 0)
         << ", gap=" << (trace.request.gap_planner_active ? 1 : 0)
         << ", dynamic_escape=" << (trace.request.dynamic_obstacle_escape_active ? 1 : 0)
         << ", mission_wait=" << (trace.request.dynamic_wait_active ? 1 : 0)
         << "/prefix=" << (trace.request.dynamic_wait_forward_prefix_active ? 1 : 0)
         << ", contact=" << (trace.request.contact_continuation_active ? 1 : 0)
         << "/precontact=" << (trace.request.precontact_escape_active ? 1 : 0)
         << ", safety="
         << ((trace.request.behavior == Behavior::SafetyBrake ||
              trace.request.emergency_brake_active) ? 1 : 0)
         << ", solver_fallback=" << (trace.request.solver_fallback_active ? 1 : 0)
         << ", speed=" << finite_or(trace.request.speed_reference_mps, "inf")
         << "/" << finite_or(trace.request.speed_limit_mps, "inf")
         << "/" << finite_or(trace.request.speed_floor_mps, "nan")
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
