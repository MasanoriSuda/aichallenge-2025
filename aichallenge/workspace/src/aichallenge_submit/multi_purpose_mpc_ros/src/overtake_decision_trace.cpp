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
         << ", side=" << trace.side << ", outcome=" << to_string(trace.outcome)
         << ", failures=" << trace.consecutive_failures
         << ", backoff=" << finite_or_nan(trace.backoff_sec) << "s"
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
         << (trace.no_return ? 1 : 0) << "|"
         << (trace.hard_fault ? 1 : 0) << "|"
         << (trace.forward_prefix_active ? 1 : 0) << "|"
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
         << ", alternate=" << (trace.alternate_feasible ? 1 : 0) << "/"
         << (trace.alternate_mission_available ? 1 : 0) << "/"
         << (trace.alternate_stable ? 1 : 0) << "/"
         << (trace.alternate_urgent_admission ? 1 : 0) << "/"
         << (trace.alternate_ready ? 1 : 0)
         << ", cross_side=" << (trace.cross_side_allowed ? 1 : 0)
         << ", no_return=" << (trace.no_return ? 1 : 0)
         << ", hard_fault=" << (trace.hard_fault ? 1 : 0)
         << ", prefix=" << (trace.forward_prefix_active ? 1 : 0)
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
