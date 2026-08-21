#include "multi_purpose_mpc_ros/overtake_decision_trace.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>

namespace multi_purpose_mpc_ros::overtake_decision_trace {
namespace {

std::string reason_or(const std::string &reason, const char *fallback) {
  return reason.empty() ? fallback : reason;
}

std::string finite_or_nan(const double value) {
  if (!std::isfinite(value)) {
    return "nan";
  }
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(2) << value;
  return stream.str();
}

std::string index_or_none(const std::size_t index) {
  return index == std::numeric_limits<std::size_t>::max() ?
         "none" : std::to_string(index);
}

std::string candidate_reason(const CandidateTrace &trace) {
  switch (classify_candidate(trace)) {
  case CandidateDisposition::NotEvaluated:
    return "not-evaluated";
  case CandidateDisposition::PlannerInactive:
    return reason_or(trace.planner_reason, "planner-inactive");
  case CandidateDisposition::PlannerRejected:
    return reason_or(trace.planner_reason, "planner-rejected");
  case CandidateDisposition::BridgeRejected:
    return reason_or(trace.bridge_reason, "bridge-rejected");
  case CandidateDisposition::BackedOff:
    return "tracking-solver-backoff";
  case CandidateDisposition::SideMismatch:
    return "forced-side-not-produced";
  case CandidateDisposition::Ready:
    return "ready";
  }
  return "unknown";
}

std::string format_candidate(const CandidateTrace &trace) {
  std::ostringstream stream;
  const auto disposition = classify_candidate(trace);
  stream << "{state=" << to_string(disposition)
         << ",requested_side=" << trace.requested_side
         << ",resolved_side=" << trace.resolved_side << ",reason=\""
         << candidate_reason(trace) << "\"";
  if (trace.evaluated) {
    stream << ",planner=" << (trace.planner_active ? 1 : 0) << "/"
           << (trace.planner_feasible ? 1 : 0)
           << ",bridge=" << (trace.bridge_evaluated ? 1 : 0) << "/"
           << (trace.bridge_feasible ? 1 : 0)
           << ",planner_gate=" << reason_or(trace.planner_reject_gate, "none")
           << ",planner_reject=" << trace.planner_reject_index << "@"
           << finite_or_nan(trace.planner_reject_distance_m) << "m"
           << ",free_intervals=" << trace.planner_free_interval_count
           << ",samples=" << trace.bridge_checked_samples
           << ",reject=" << trace.bridge_reject_index << "@"
           << finite_or_nan(trace.bridge_reject_distance_m) << "m"
           << ",width=" << finite_or_nan(trace.corridor_width_m) << "m"
           << ",adjust=" << finite_or_nan(trace.bridge_maximum_adjustment_m)
           << "m"
           << ",preflight="
           << (trace.static_wall_preflight_evaluated ? 1 : 0) << "/"
           << (trace.static_wall_preflight_feasible ? 1 : 0)
           << ",preflight_mode="
           << reason_or(trace.static_wall_preflight_mode, "not-evaluated")
           << ",preflight_margin_escape="
           << (trace.static_wall_margin_escape_used ? 1 : 0)
           << ",preflight_reason=\""
           << reason_or(trace.static_wall_preflight_reason, "not-evaluated")
           << "\""
           << ",preflight_samples=" << trace.static_wall_execution_samples
           << "/" << trace.static_wall_active_samples
           << ",preflight_range="
           << index_or_none(trace.static_wall_first_active_index) << ":"
           << index_or_none(trace.static_wall_last_active_index)
           << ",preflight_invalid_index="
           << index_or_none(trace.static_wall_invalid_index)
           << ",preflight_poses=" << trace.static_wall_checked_poses
           << ",preflight_raw_poses="
           << trace.static_wall_physical_checked_poses
           << ",preflight_margin_contacts="
           << trace.static_wall_initial_margin_contacts << "/"
           << trace.static_wall_maximum_margin_contacts << "/"
           << trace.static_wall_final_margin_contacts
           << ",preflight_margin_clear="
           << index_or_none(trace.static_wall_margin_clear_path_index) << "@"
           << finite_or_nan(trace.static_wall_margin_clear_distance_m) << "m"
           << ",tracking_contract="
           << (trace.tracking_wall_contract_evaluated ? 1 : 0) << "/"
           << (trace.tracking_wall_contract_valid ? 1 : 0) << "/"
           << (trace.tracking_wall_contract_feasible ? 1 : 0)
           << ",tracking_contract_active="
           << (trace.tracking_wall_contract_active ? 1 : 0)
           << ",tracking_contract_side="
           << trace.tracking_wall_contract_side_sign
           << ",tracking_contract_relaxed="
           << trace.tracking_wall_contract_relaxed_samples
           << ",tracking_contract_full="
           << index_or_none(
             trace.tracking_wall_contract_first_full_margin_index) << "@"
           << finite_or_nan(
             trace.tracking_wall_contract_first_full_margin_distance_m) << "m"
           << ",tracking_contract_max_relax="
           << finite_or_nan(
             trace.tracking_wall_contract_maximum_relaxation_m) << "m"
           << ",tracking_contract_first=["
           << finite_or_nan(trace.tracking_wall_contract_first_lower_m) << ","
           << finite_or_nan(trace.tracking_wall_contract_first_upper_m) << "]"
           << ",tracking_contract_current="
           << finite_or_nan(trace.tracking_wall_contract_current_lateral_m) << "m"
           << ",tracking_contract_reason=\""
           << reason_or(
             trace.tracking_wall_contract_reason, "not-evaluated") << "\""
           << ",forecast=" << (trace.forecast_evaluated ? 1 : 0) << "/"
           << (trace.future_threatened ? 1 : 0)
           << ",forecast_tier=" << trace.forecast_risk_tier
           << ",forecast_reason="
           << reason_or(trace.forecast_reason, "not-evaluated")
           << ",min_reserve="
           << finite_or_nan(trace.minimum_corridor_reserve_m) << "m"
           << ",threat_distance="
           << finite_or_nan(trace.first_threat_distance_m) << "m"
           << ",side_transition="
           << (trace.forced_side_transition_requested ? 1 : 0) << "/"
           << (trace.forced_side_transition_gateway_found ? 1 : 0)
           << "/" << (trace.forced_side_transition_crossing_started ? 1 : 0)
           << "/" << (trace.forced_side_transition_requested_side_reached ? 1 : 0)
           << "/" << (trace.forced_side_transition_certified ? 1 : 0)
           << ",side_transition_prefix="
           << trace.forced_side_transition_prefix_samples
           << ",side_transition_gateway="
           << index_or_none(trace.forced_side_transition_gateway_index) << "@"
           << finite_or_nan(trace.forced_side_transition_gateway_distance_m) << "m"
           << ",side_transition_side_reached="
           << index_or_none(trace.forced_side_transition_side_reached_index) << "@"
           << finite_or_nan(trace.forced_side_transition_side_reached_distance_m) << "m"
           << ",side_transition_certified_until="
           << index_or_none(trace.forced_side_transition_certified_until_index) << "@"
           << finite_or_nan(trace.forced_side_transition_certified_until_distance_m) << "m"
           << ",side_transition_first_disconnect="
           << index_or_none(trace.forced_side_transition_first_disconnect_index) << "@"
           << finite_or_nan(trace.forced_side_transition_first_disconnect_distance_m) << "m"
           << ",side_transition_required_connected="
           << finite_or_nan(
             trace.forced_side_transition_required_connected_distance_m) << "m"
           << ",side_transition_reason="
           << reason_or(trace.forced_side_transition_reason, "not-requested")
           << ",backoff=" << trace.backoff_failures << "/"
           << finite_or_nan(trace.backoff_remaining_sec) << "s";
  }
  stream << "}";
  return stream.str();
}

} // namespace

CandidateDisposition classify_candidate(const CandidateTrace &trace) noexcept {
  if (!trace.evaluated) {
    return CandidateDisposition::NotEvaluated;
  }
  if (!trace.planner_feasible) {
    return CandidateDisposition::PlannerRejected;
  }
  if (!trace.planner_active) {
    return CandidateDisposition::PlannerInactive;
  }
  if (!trace.bridge_evaluated || !trace.bridge_feasible) {
    return CandidateDisposition::BridgeRejected;
  }
  if (trace.backoff_active) {
    return CandidateDisposition::BackedOff;
  }
  if ((trace.requested_side == -1 || trace.requested_side == 1) &&
      trace.resolved_side != trace.requested_side) {
    return CandidateDisposition::SideMismatch;
  }
  if (trace.resolved_side != -1 && trace.resolved_side != 1) {
    return CandidateDisposition::SideMismatch;
  }
  return CandidateDisposition::Ready;
}

const char *to_string(const CandidateDisposition disposition) noexcept {
  switch (disposition) {
  case CandidateDisposition::NotEvaluated:
    return "not-evaluated";
  case CandidateDisposition::PlannerInactive:
    return "planner-inactive";
  case CandidateDisposition::PlannerRejected:
    return "planner-rejected";
  case CandidateDisposition::BridgeRejected:
    return "bridge-rejected";
  case CandidateDisposition::BackedOff:
    return "backed-off";
  case CandidateDisposition::SideMismatch:
    return "side-mismatch";
  case CandidateDisposition::Ready:
    return "ready";
  }
  return "unknown";
}

DecisionOutcome classify_outcome(const DecisionTrace &trace) noexcept {
  if (!trace.requested) {
    return DecisionOutcome::NotRequested;
  }
  if (trace.authority_active) {
    if (!trace.tracking_qualified) {
      return trace.alternate_selected ?
             DecisionOutcome::QualificationPendingAlternate :
             DecisionOutcome::QualificationPendingPrimary;
    }
    return trace.alternate_selected ? DecisionOutcome::ActiveAlternate
                                    : DecisionOutcome::ActivePrimary;
  }
  if (trace.alternate_attempted && !trace.alternate_selected) {
    return DecisionOutcome::AlternateRejected;
  }
  if (classify_candidate(trace.primary) == CandidateDisposition::BackedOff) {
    return DecisionOutcome::PrimaryBackedOff;
  }
  if (classify_candidate(trace.primary) != CandidateDisposition::Ready) {
    return DecisionOutcome::PrimaryRejected;
  }
  return DecisionOutcome::AuthorityRejected;
}

const char *to_string(const DecisionOutcome outcome) noexcept {
  switch (outcome) {
  case DecisionOutcome::NotRequested:
    return "not-requested";
  case DecisionOutcome::PrimaryRejected:
    return "primary-rejected";
  case DecisionOutcome::PrimaryBackedOff:
    return "primary-backed-off";
  case DecisionOutcome::AlternateRejected:
    return "alternate-rejected";
  case DecisionOutcome::AuthorityRejected:
    return "authority-rejected";
  case DecisionOutcome::QualificationPendingPrimary:
    return "qualification-pending-primary";
  case DecisionOutcome::QualificationPendingAlternate:
    return "qualification-pending-alternate";
  case DecisionOutcome::ActivePrimary:
    return "active-primary";
  case DecisionOutcome::ActiveAlternate:
    return "active-alternate";
  }
  return "unknown";
}

std::string categorical_signature(const DecisionTrace &trace) {
  std::ostringstream stream;
  const auto append_candidate = [&stream](const CandidateTrace &candidate) {
    stream << static_cast<int>(classify_candidate(candidate)) << ":"
           << candidate.requested_side << ":" << candidate.resolved_side << ":"
           << candidate_reason(candidate) << ":" << candidate.planner_reject_gate
           << ":" << (candidate.static_wall_preflight_evaluated ? 1 : 0)
           << ":" << (candidate.static_wall_preflight_feasible ? 1 : 0)
           << ":" << candidate.static_wall_preflight_mode
           << ":" << (candidate.static_wall_margin_escape_used ? 1 : 0)
           << ":" << candidate.static_wall_preflight_reason
           << ":" << candidate.static_wall_invalid_index
           << ":" << (candidate.tracking_wall_contract_evaluated ? 1 : 0)
           << ":" << (candidate.tracking_wall_contract_valid ? 1 : 0)
           << ":" << (candidate.tracking_wall_contract_active ? 1 : 0)
           << ":" << (candidate.tracking_wall_contract_feasible ? 1 : 0)
           << ":" << candidate.tracking_wall_contract_side_sign
           << ":" << candidate.tracking_wall_contract_reason
           << ":" << (candidate.forecast_evaluated ? 1 : 0)
           << ":" << (candidate.future_threatened ? 1 : 0)
           << ":" << candidate.forecast_risk_tier
           << ":" << candidate.forecast_reason
           << ":" << (candidate.forced_side_transition_requested ? 1 : 0)
           << ":" << (candidate.forced_side_transition_gateway_found ? 1 : 0)
           << ":" << (candidate.forced_side_transition_crossing_started ? 1 : 0)
           << ":" << (candidate.forced_side_transition_requested_side_reached ? 1 : 0)
           << ":" << (candidate.forced_side_transition_certified ? 1 : 0)
           << ":" << candidate.forced_side_transition_reason
           << ":" << candidate.backoff_failures;
  };
  stream << trace.attempt_id << "|" << trace.mission_episode_id << "|"
         << trace.target_id << "|"
         << static_cast<int>(classify_outcome(trace)) << "|";
  append_candidate(trace.primary);
  stream << "|";
  append_candidate(trace.alternate);
  stream << "|" << (trace.alternate_attempted ? 1 : 0) << "|"
         << (trace.alternate_selected ? 1 : 0) << "|" << trace.authority_reason
         << "|" << (trace.primary_suppressed ? 1 : 0)
         << "|" << trace.primary_suppression_reason
         << "|" << (trace.proactive_alternate ? 1 : 0)
         << "|" << trace.alternate_trigger_reason
         << "|" << trace.branch_selection_reason
         << "|" << (trace.pass_through ? 1 : 0)
         << "|" << trace.final_side << "|" << (trace.tracking_qualified ? 1 : 0)
         << "|" << (trace.follow_cap_suppressed ? 1 : 0);
  return stream.str();
}

std::string format_decision_trace(const DecisionTrace &trace) {
  std::ostringstream stream;
  stream << "Overtake decision trace: stage=planning, attempt="
         << trace.attempt_id << ", mission_episode=" << trace.mission_episode_id
         << ", target="
         << (trace.target_id.empty() ? "<none>" : trace.target_id)
         << ", outcome=" << to_string(classify_outcome(trace))
         << ", primary=" << format_candidate(trace.primary)
         << ", alternate=" << format_candidate(trace.alternate)
         << ", alternate_trigger="
         << reason_or(trace.alternate_trigger_reason, "not-evaluated")
         << ", proactive_alternate=" << (trace.proactive_alternate ? 1 : 0)
         << ", branch_selection="
         << reason_or(trace.branch_selection_reason, "not-evaluated")
         << ", primary_suppressed=" << (trace.primary_suppressed ? 1 : 0)
         << "/" << reason_or(
           trace.primary_suppression_reason, "not-suppressed")
         << ", authority=" << (trace.authority_active ? 1 : 0) << "/"
         << trace.authority_reason
         << ", pass_through=" << (trace.pass_through ? 1 : 0)
         << ", final_side=" << trace.final_side
         << ", shift=" << finite_or_nan(trace.final_shift_m) << "m"
         << ", qualified=" << (trace.tracking_qualified ? 1 : 0)
         << ", follow_cap_suppressed=" << (trace.follow_cap_suppressed ? 1 : 0)
         << ", wp_id=" << trace.waypoint_id;
  return stream.str();
}

TraceEmission
ChangeAwareTraceEmitter::update(const DecisionTrace &trace,
                                const double now_sec,
                                const double repeat_interval_sec) {
  TraceEmission emission;
  if (!trace.requested) {
    return emission;
  }

  emission.signature = categorical_signature(trace);
  emission.state_changed = emission.signature != last_signature_;
  const bool repeat_due =
      std::isfinite(now_sec) && std::isfinite(last_emit_sec_) &&
      std::isfinite(repeat_interval_sec) && repeat_interval_sec >= 0.0 &&
      now_sec >= last_emit_sec_ &&
      now_sec - last_emit_sec_ >= repeat_interval_sec;
  emission.emit = emission.state_changed || repeat_due;
  if (emission.emit) {
    emission.message = format_decision_trace(trace);
    last_signature_ = emission.signature;
    last_emit_sec_ = now_sec;
  }
  return emission;
}

void ChangeAwareTraceEmitter::reset() noexcept {
  last_signature_.clear();
  last_emit_sec_ = -std::numeric_limits<double>::infinity();
}

const char *to_string(const TrackingOutcome outcome) noexcept {
  switch (outcome) {
  case TrackingOutcome::QualificationRejected:
    return "qualification-rejected";
  case TrackingOutcome::Qualified:
    return "qualified";
  case TrackingOutcome::Failed:
    return "failed";
  case TrackingOutcome::Recovered:
    return "recovered";
  }
  return "unknown";
}

std::string format_tracking_trace(const TrackingTrace &trace) {
  std::ostringstream stream;
  stream << "Overtake decision trace: stage=tracking, attempt="
         << trace.attempt_id << ", mission_episode=" << trace.mission_episode_id
         << ", target="
         << (trace.target_id.empty() ? "<none>" : trace.target_id)
         << ", side=" << trace.side
         << ", committed_branch=" << reason_or(trace.committed_branch, "none")
         << ", outcome=" << to_string(trace.outcome)
         << ", failures=" << trace.consecutive_failures
         << ", backoff=" << finite_or_nan(trace.backoff_sec) << "s"
         << ", preflight_mode="
         << reason_or(trace.preflight_mode, "not-evaluated")
         << ", preflight_margin_escape="
         << (trace.preflight_margin_escape_used ? 1 : 0)
         << ", preflight_margin_clear="
         << finite_or_nan(trace.preflight_margin_clear_distance_m) << "m"
         << ", tracking_contract="
         << (trace.tracking_wall_contract_active ? 1 : 0) << "/"
         << reason_or(trace.tracking_wall_contract_reason, "not-evaluated")
         << ", tracking_contract_max_relax="
         << finite_or_nan(
           trace.tracking_wall_contract_maximum_relaxation_m) << "m"
         << ", corridor_width=" << finite_or_nan(trace.corridor_width_m)
         << "m"
         << ", target_adjust="
         << finite_or_nan(trace.maximum_target_adjustment_m) << "m"
         << ", cold_retry=" << (trace.cold_retry_attempted ? 1 : 0)
         << "/" << (trace.cold_retry_succeeded ? 1 : 0)
         << ", qualification_hold="
         << (trace.qualification_hold_available ? 1 : 0) << "/"
         << (trace.qualification_hold_used ? 1 : 0) << "/"
         << finite_or_nan(trace.qualification_hold_speed_mps) << "mps/"
         << finite_or_nan(trace.qualification_hold_steering_rad) << "rad"
         << ", initial_solver_reason=\""
         << reason_or(trace.initial_solver_reason, "none") << "\""
         << ", reason=\"" << reason_or(trace.reason, "none") << "\"";
  return stream.str();
}

std::string categorical_signature(const RuntimeFailoverTrace &trace) {
  std::ostringstream stream;
  const std::string trigger_gate = classify_runtime_failover_trigger(trace.trigger);
  stream << trace.mission_episode_id << "|" << trace.mission_generation << "|"
         << trace.target_id << "|" << trace.phase << "|" << trigger_gate;
  if (trigger_gate == "other") {
    // Unknown triggers must remain observable until they receive a stable
    // category. Known trigger details are excluded to suppress log chatter.
    stream << ":" << trace.trigger;
  }
  stream << "|"
         << (trace.current_feasible ? 1 : 0) << ":"
         << (trace.current_mission_available ? 1 : 0) << ":"
         << (trace.current_ready ? 1 : 0) << "|"
         << (trace.alternate_feasible ? 1 : 0) << ":"
         << (trace.alternate_mission_available ? 1 : 0) << ":"
         << (trace.alternate_stable ? 1 : 0) << ":"
         << (trace.alternate_urgent_admission ? 1 : 0) << ":"
         << (trace.alternate_ready ? 1 : 0) << "|"
         << (trace.cross_side_allowed ? 1 : 0) << "|"
         << (trace.cross_side_lease_active ? 1 : 0) << "|"
         << (trace.no_return ? 1 : 0) << "|"
         << (trace.hard_fault ? 1 : 0) << "|"
         << (trace.forward_prefix_active ? 1 : 0) << "|"
         << (trace.bounded_lateral_escape_active ? 1 : 0) << "|"
         << trace.action << "|" << trace.source;
  return stream.str();
}

std::string classify_runtime_failover_trigger(const std::string &trigger) {
  if (trigger.find("physical target separation conflicts") != std::string::npos) {
    return "target-wall-conflict";
  }
  if (trigger.find("Pass entry physical gate has no valid current-side prefix") !=
      std::string::npos) {
    return "pass-entry-no-prefix";
  }
  if (trigger.find("Pass entry physical wall gate unresolved") != std::string::npos) {
    return "pass-entry-wall-unresolved";
  }
  if (trigger.find("optimized horizon failed physical revalidation") !=
      std::string::npos) {
    return "optimized-horizon-physical";
  }
  if (trigger.find("live overtake corridor unavailable") != std::string::npos) {
    return "live-corridor-unavailable";
  }
  if (trigger.find("locked target entered selected pass-side line") !=
      std::string::npos) {
    return "pass-side-intrusion";
  }
  if (trigger.empty()) {
    return "unknown";
  }
  return "other";
}

std::string format_runtime_failover_trace(const RuntimeFailoverTrace &trace) {
  std::ostringstream stream;
  stream << "Overtake decision trace: stage=runtime-failover, mission_episode="
         << trace.mission_episode_id << ", generation=" << trace.mission_generation
         << ", target=" << (trace.target_id.empty() ? "<none>" : trace.target_id)
         << ", phase=" << reason_or(trace.phase, "unknown")
         << ", trigger_gate=" << classify_runtime_failover_trigger(trace.trigger)
         << ", trigger=\"" << reason_or(trace.trigger, "unknown") << "\""
         << ", source=" << reason_or(trace.source, "resolver")
         << ", current=" << (trace.current_feasible ? 1 : 0) << "/"
         << (trace.current_mission_available ? 1 : 0) << "/"
         << (trace.current_ready ? 1 : 0)
         << "/reason=\"" << reason_or(trace.current_reason, "not-evaluated") << "\""
         << ", alternate=" << (trace.alternate_feasible ? 1 : 0) << "/"
         << (trace.alternate_mission_available ? 1 : 0) << "/"
         << (trace.alternate_stable ? 1 : 0) << "/"
         << (trace.alternate_urgent_admission ? 1 : 0) << "/"
         << (trace.alternate_ready ? 1 : 0)
         << "/reason=\"" << reason_or(trace.alternate_reason, "not-evaluated") << "\""
         << ", cross_side=" << (trace.cross_side_allowed ? 1 : 0)
         << "/lease=" << (trace.cross_side_lease_active ? 1 : 0)
         << ", no_return=" << (trace.no_return ? 1 : 0)
         << ", hard_fault=" << (trace.hard_fault ? 1 : 0)
         << ", prefix=" << (trace.forward_prefix_active ? 1 : 0)
         << ", escape_authority="
         << (trace.bounded_lateral_escape_active ? 1 : 0)
         << ", action=" << reason_or(trace.action, "inactive")
         << ", reason=" << reason_or(trace.reason, "not-evaluated")
         << ", wp_id=" << trace.waypoint_id;
  return stream.str();
}

TraceEmission ChangeAwareRuntimeFailoverTraceEmitter::update(
    const RuntimeFailoverTrace &trace, const double now_sec,
    const double repeat_interval_sec) {
  TraceEmission emission;
  emission.signature = categorical_signature(trace);
  emission.state_changed = emission.signature != last_signature_;
  const bool repeat_due =
      std::isfinite(now_sec) && std::isfinite(last_emit_sec_) &&
      std::isfinite(repeat_interval_sec) && repeat_interval_sec >= 0.0 &&
      now_sec >= last_emit_sec_ &&
      now_sec - last_emit_sec_ >= repeat_interval_sec;
  emission.emit = emission.state_changed || repeat_due;
  if (emission.emit) {
    emission.message = format_runtime_failover_trace(trace);
    last_signature_ = emission.signature;
    last_emit_sec_ = now_sec;
  }
  return emission;
}

void ChangeAwareRuntimeFailoverTraceEmitter::reset() noexcept {
  last_signature_.clear();
  last_emit_sec_ = -std::numeric_limits<double>::infinity();
}

} // namespace multi_purpose_mpc_ros::overtake_decision_trace
