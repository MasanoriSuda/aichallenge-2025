#ifndef MULTI_PURPOSE_MPC_ROS__OVERTAKE_DECISION_TRACE_HPP_
#define MULTI_PURPOSE_MPC_ROS__OVERTAKE_DECISION_TRACE_HPP_

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>

namespace multi_purpose_mpc_ros::overtake_decision_trace {

enum class CandidateDisposition {
  NotEvaluated,
  PlannerInactive,
  PlannerRejected,
  BridgeRejected,
  BackedOff,
  SideMismatch,
  Ready,
};

enum class DecisionOutcome {
  NotRequested,
  PrimaryRejected,
  PrimaryBackedOff,
  AlternateRejected,
  AuthorityRejected,
  ActivePrimary,
  ActiveAlternate,
};

struct CandidateTrace {
  bool evaluated{false};
  int requested_side{0};
  int resolved_side{0};
  bool planner_active{false};
  bool planner_feasible{false};
  std::string planner_reason;
  std::string planner_reject_gate{"none"};
  std::size_t planner_reject_index{0U};
  double planner_reject_distance_m{std::numeric_limits<double>::quiet_NaN()};
  std::size_t planner_free_interval_count{0U};
  double corridor_width_m{0.0};
  bool bridge_evaluated{false};
  bool bridge_feasible{false};
  std::string bridge_reason{"not-evaluated"};
  std::size_t bridge_checked_samples{0U};
  std::size_t bridge_reject_index{0U};
  double bridge_reject_distance_m{std::numeric_limits<double>::quiet_NaN()};
  double bridge_maximum_adjustment_m{0.0};
  bool static_wall_preflight_evaluated{false};
  bool static_wall_preflight_feasible{false};
  bool static_wall_margin_escape_used{false};
  std::size_t static_wall_checked_poses{0U};
  std::size_t static_wall_physical_checked_poses{0U};
  std::size_t static_wall_execution_samples{0U};
  std::size_t static_wall_active_samples{0U};
  std::size_t static_wall_first_active_index{std::numeric_limits<std::size_t>::max()};
  std::size_t static_wall_last_active_index{std::numeric_limits<std::size_t>::max()};
  std::size_t static_wall_invalid_index{std::numeric_limits<std::size_t>::max()};
  std::size_t static_wall_initial_margin_contacts{0U};
  std::size_t static_wall_maximum_margin_contacts{0U};
  std::size_t static_wall_final_margin_contacts{0U};
  std::size_t static_wall_margin_clear_path_index{
    std::numeric_limits<std::size_t>::max()};
  double static_wall_margin_clear_distance_m{
    std::numeric_limits<double>::quiet_NaN()};
  std::string static_wall_preflight_mode{"not-evaluated"};
  std::string static_wall_preflight_reason{"not-evaluated"};
  bool backoff_active{false};
  int backoff_failures{0};
  double backoff_remaining_sec{0.0};
};

struct DecisionTrace {
  std::uint64_t attempt_id{0U};
  std::uint64_t mission_episode_id{0U};
  std::string target_id;
  bool requested{false};
  CandidateTrace primary;
  CandidateTrace alternate;
  bool alternate_attempted{false};
  bool alternate_selected{false};
  bool authority_active{false};
  bool pass_through{false};
  std::string authority_reason{"not-evaluated"};
  int final_side{0};
  double final_shift_m{0.0};
  bool tracking_qualified{false};
  bool follow_cap_suppressed{false};
  int waypoint_id{0};
};

struct TraceEmission {
  bool emit{false};
  bool state_changed{false};
  std::string signature;
  std::string message;
};

CandidateDisposition classify_candidate(const CandidateTrace &trace) noexcept;
const char *to_string(CandidateDisposition disposition) noexcept;
DecisionOutcome classify_outcome(const DecisionTrace &trace) noexcept;
const char *to_string(DecisionOutcome outcome) noexcept;
std::string categorical_signature(const DecisionTrace &trace);
std::string format_decision_trace(const DecisionTrace &trace);

/// Suppress 40 Hz duplicates while retaining immediate categorical changes.
/// Continuous values such as remaining backoff time and corridor width are
/// deliberately excluded from the change signature.
class ChangeAwareTraceEmitter {
public:
  TraceEmission update(const DecisionTrace &trace, double now_sec,
                       double repeat_interval_sec = 5.0);
  void reset() noexcept;

private:
  std::string last_signature_;
  double last_emit_sec_{-std::numeric_limits<double>::infinity()};
};

enum class TrackingOutcome {
  Failed,
  Recovered,
};

struct TrackingTrace {
  std::uint64_t attempt_id{0U};
  std::uint64_t mission_episode_id{0U};
  std::string target_id;
  int side{0};
  TrackingOutcome outcome{TrackingOutcome::Failed};
  int consecutive_failures{0};
  double backoff_sec{0.0};
  std::string reason;
};

const char *to_string(TrackingOutcome outcome) noexcept;
std::string format_tracking_trace(const TrackingTrace &trace);

struct RuntimeFailoverTrace {
  std::uint64_t mission_episode_id{0U};
  std::uint64_t mission_generation{0U};
  std::string target_id;
  std::string phase;
  std::string trigger;
  std::string source{"resolver"};
  bool current_feasible{false};
  bool current_mission_available{false};
  bool current_ready{false};
  std::string current_reason{"not-evaluated"};
  bool alternate_feasible{false};
  bool alternate_mission_available{false};
  bool alternate_stable{false};
  bool alternate_urgent_admission{false};
  bool alternate_ready{false};
  std::string alternate_reason{"not-evaluated"};
  bool cross_side_allowed{false};
  bool cross_side_lease_active{false};
  bool no_return{false};
  bool hard_fault{false};
  bool forward_prefix_active{false};
  bool bounded_lateral_escape_active{false};
  std::string action{"inactive"};
  std::string reason{"not-evaluated"};
  int waypoint_id{0};
};

std::string categorical_signature(const RuntimeFailoverTrace &trace);
std::string format_runtime_failover_trace(const RuntimeFailoverTrace &trace);
std::string classify_runtime_failover_trigger(const std::string &trigger);

class ChangeAwareRuntimeFailoverTraceEmitter {
public:
  TraceEmission update(const RuntimeFailoverTrace &trace, double now_sec,
                       double repeat_interval_sec = 5.0);
  void reset() noexcept;

private:
  std::string last_signature_;
  double last_emit_sec_{-std::numeric_limits<double>::infinity()};
};

} // namespace multi_purpose_mpc_ros::overtake_decision_trace

#endif // MULTI_PURPOSE_MPC_ROS__OVERTAKE_DECISION_TRACE_HPP_
