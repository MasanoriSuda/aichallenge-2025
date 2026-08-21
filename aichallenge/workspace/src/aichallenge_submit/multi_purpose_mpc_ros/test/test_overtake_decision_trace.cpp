#include "multi_purpose_mpc_ros/overtake_decision_trace.hpp"

#include <gtest/gtest.h>

#include <string>

namespace {

namespace trace = multi_purpose_mpc_ros::overtake_decision_trace;

trace::CandidateTrace ready_candidate(const int requested_side,
                                      const int resolved_side) {
  trace::CandidateTrace candidate;
  candidate.evaluated = true;
  candidate.requested_side = requested_side;
  candidate.resolved_side = resolved_side;
  candidate.planner_active = true;
  candidate.planner_feasible = true;
  candidate.bridge_evaluated = true;
  candidate.bridge_feasible = true;
  candidate.bridge_reason = "reachable";
  candidate.static_wall_preflight_evaluated = true;
  candidate.static_wall_preflight_feasible = true;
  candidate.static_wall_preflight_reason = "clear";
  return candidate;
}

TEST(OvertakeDecisionTrace, ClassifiesEveryCandidateGate) {
  trace::CandidateTrace candidate;
  EXPECT_EQ(trace::classify_candidate(candidate),
            trace::CandidateDisposition::NotEvaluated);

  candidate.evaluated = true;
  candidate.planner_feasible = true;
  EXPECT_EQ(trace::classify_candidate(candidate),
            trace::CandidateDisposition::PlannerInactive);

  candidate.planner_active = true;
  candidate.planner_feasible = false;
  EXPECT_EQ(trace::classify_candidate(candidate),
            trace::CandidateDisposition::PlannerRejected);

  candidate.planner_feasible = true;
  EXPECT_EQ(trace::classify_candidate(candidate),
            trace::CandidateDisposition::BridgeRejected);

  candidate.bridge_evaluated = true;
  candidate.bridge_feasible = true;
  candidate.requested_side = 1;
  candidate.resolved_side = 1;
  candidate.backoff_active = true;
  EXPECT_EQ(trace::classify_candidate(candidate),
            trace::CandidateDisposition::BackedOff);

  candidate.backoff_active = false;
  candidate.resolved_side = -1;
  EXPECT_EQ(trace::classify_candidate(candidate),
            trace::CandidateDisposition::SideMismatch);

  candidate.resolved_side = 1;
  EXPECT_EQ(trace::classify_candidate(candidate),
            trace::CandidateDisposition::Ready);
}

TEST(OvertakeDecisionTrace,
     ExplainsRejectedAlternateWithoutLosingPrimaryBackoff) {
  trace::DecisionTrace decision;
  decision.attempt_id = 17U;
  decision.mission_episode_id = 9U;
  decision.target_id = "d2";
  decision.requested = true;
  decision.primary = ready_candidate(0, 1);
  decision.primary.backoff_active = true;
  decision.primary.backoff_failures = 3;
  decision.primary.backoff_remaining_sec = 1.75;
  decision.alternate_attempted = true;
  decision.alternate = ready_candidate(-1, 0);
  decision.alternate.bridge_feasible = false;
  decision.alternate.bridge_reason =
      "corridor is outside the reachable lateral envelope";
  decision.authority_reason = "planner-infeasible";

  EXPECT_EQ(trace::classify_outcome(decision),
            trace::DecisionOutcome::AlternateRejected);
  const std::string message = trace::format_decision_trace(decision);
  EXPECT_NE(message.find("outcome=alternate-rejected"), std::string::npos);
  EXPECT_NE(message.find("attempt=17"), std::string::npos);
  EXPECT_NE(message.find("mission_episode=9"), std::string::npos);
  EXPECT_NE(message.find("state=backed-off"), std::string::npos);
  EXPECT_NE(message.find("state=bridge-rejected"), std::string::npos);
  EXPECT_NE(message.find("corridor is outside the reachable lateral envelope"),
            std::string::npos);
}

TEST(OvertakeDecisionTrace, ReportsStaticWallExecutionPreflight) {
  trace::DecisionTrace decision;
  decision.target_id = "d2";
  decision.requested = true;
  decision.primary = ready_candidate(0, 1);
  decision.primary.bridge_feasible = false;
  decision.primary.bridge_reason =
      "static wall execution preflight: occupied";
  decision.primary.static_wall_preflight_feasible = false;
  decision.primary.static_wall_margin_escape_used = true;
  decision.primary.static_wall_preflight_mode = "margin-escape";
  decision.primary.static_wall_preflight_reason = "occupied";
  decision.primary.static_wall_checked_poses = 14U;
  decision.primary.static_wall_physical_checked_poses = 21U;
  decision.primary.static_wall_execution_samples = 7U;
  decision.primary.static_wall_active_samples = 5U;
  decision.primary.static_wall_first_active_index = 1U;
  decision.primary.static_wall_last_active_index = 5U;
  decision.primary.static_wall_invalid_index = 3U;
  decision.primary.static_wall_initial_margin_contacts = 6U;
  decision.primary.static_wall_maximum_margin_contacts = 6U;
  decision.primary.static_wall_final_margin_contacts = 0U;
  decision.primary.static_wall_margin_clear_path_index = 2U;
  decision.primary.static_wall_margin_clear_distance_m = 1.25;
  decision.primary.tracking_wall_contract_evaluated = true;
  decision.primary.tracking_wall_contract_valid = true;
  decision.primary.tracking_wall_contract_active = true;
  decision.primary.tracking_wall_contract_feasible = true;
  decision.primary.tracking_wall_contract_side_sign = 1;
  decision.primary.tracking_wall_contract_relaxed_samples = 2U;
  decision.primary.tracking_wall_contract_first_full_margin_index = 3U;
  decision.primary.tracking_wall_contract_first_full_margin_distance_m = 1.65;
  decision.primary.tracking_wall_contract_maximum_relaxation_m = 0.18;
  decision.primary.tracking_wall_contract_current_lateral_m = 1.20;
  decision.primary.tracking_wall_contract_first_lower_m = -1.0;
  decision.primary.tracking_wall_contract_first_upper_m = 1.18;
  decision.primary.tracking_wall_contract_reason = "margin-inherit";

  const std::string message = trace::format_decision_trace(decision);
  EXPECT_NE(message.find("state=bridge-rejected"), std::string::npos);
  EXPECT_NE(message.find("preflight=1/0"), std::string::npos);
  EXPECT_NE(message.find("preflight_mode=margin-escape"), std::string::npos);
  EXPECT_NE(message.find("preflight_margin_escape=1"), std::string::npos);
  EXPECT_NE(message.find("preflight_reason=\"occupied\""), std::string::npos);
  EXPECT_NE(message.find("preflight_samples=7/5"), std::string::npos);
  EXPECT_NE(message.find("preflight_range=1:5"), std::string::npos);
  EXPECT_NE(message.find("preflight_invalid_index=3"), std::string::npos);
  EXPECT_NE(message.find("preflight_poses=14"), std::string::npos);
  EXPECT_NE(message.find("preflight_raw_poses=21"), std::string::npos);
  EXPECT_NE(message.find("preflight_margin_contacts=6/6/0"), std::string::npos);
  EXPECT_NE(message.find("preflight_margin_clear=2@1.25m"), std::string::npos);
  EXPECT_NE(message.find("tracking_contract=1/1/1"), std::string::npos);
  EXPECT_NE(message.find("tracking_contract_active=1"), std::string::npos);
  EXPECT_NE(message.find("tracking_contract_side=1"), std::string::npos);
  EXPECT_NE(message.find("tracking_contract_relaxed=2"), std::string::npos);
  EXPECT_NE(message.find("tracking_contract_full=3@1.65m"), std::string::npos);
  EXPECT_NE(message.find("tracking_contract_max_relax=0.18m"), std::string::npos);
  EXPECT_NE(message.find("tracking_contract_first=[-1.00,1.18]"), std::string::npos);
  EXPECT_NE(message.find("tracking_contract_current=1.20m"), std::string::npos);
  EXPECT_NE(message.find("tracking_contract_reason=\"margin-inherit\""),
            std::string::npos);
}

TEST(OvertakeDecisionTrace, EmitsOnCategoricalChangeButNotContinuousNoise) {
  trace::ChangeAwareTraceEmitter emitter;
  trace::DecisionTrace decision;
  decision.target_id = "d2";
  decision.requested = true;
  decision.primary = ready_candidate(0, 1);
  decision.authority_active = true;
  decision.authority_reason = "accepted";
  decision.final_side = 1;

  EXPECT_TRUE(emitter.update(decision, 1.0).emit);

  decision.primary.corridor_width_m = 3.25;
  decision.primary.backoff_remaining_sec = 0.42;
  decision.final_shift_m = 0.31;
  decision.waypoint_id = 17;
  EXPECT_FALSE(emitter.update(decision, 1.1).emit);

  decision.primary.static_wall_execution_samples = 8U;
  decision.primary.static_wall_active_samples = 4U;
  EXPECT_FALSE(emitter.update(decision, 1.15).emit);

  decision.primary.static_wall_first_active_index = 1U;
  decision.primary.static_wall_last_active_index = 4U;
  EXPECT_FALSE(emitter.update(decision, 1.18).emit);

  decision.primary.static_wall_invalid_index = 2U;
  EXPECT_TRUE(emitter.update(decision, 1.19).emit);

  decision.tracking_qualified = true;
  EXPECT_TRUE(emitter.update(decision, 1.2).emit);
  decision.primary.planner_reject_gate = "reachable-bridge";
  EXPECT_TRUE(emitter.update(decision, 1.3).emit);
  decision.primary.forecast_evaluated = true;
  decision.primary.future_threatened = true;
  decision.primary.forecast_risk_tier = 1;
  decision.primary.forecast_reason = "corridor-reserve";
  EXPECT_TRUE(emitter.update(decision, 1.4).emit);
  decision.primary.minimum_corridor_reserve_m = 0.08;
  decision.primary.first_threat_distance_m = 4.2;
  EXPECT_FALSE(emitter.update(decision, 1.5).emit);
  decision.proactive_alternate = true;
  decision.alternate_trigger_reason = "corridor-reserve";
  decision.branch_selection_reason = "lower-risk-tier";
  EXPECT_TRUE(emitter.update(decision, 1.6).emit);
  EXPECT_FALSE(emitter.update(decision, 5.9).emit);
  EXPECT_TRUE(emitter.update(decision, 6.7).emit);
}

TEST(OvertakeDecisionTrace, FormatsPredictiveAlternateLifecycle) {
  trace::DecisionTrace decision;
  decision.attempt_id = 23U;
  decision.target_id = "d2";
  decision.requested = true;
  decision.primary = ready_candidate(0, 1);
  decision.primary.forecast_evaluated = true;
  decision.primary.future_threatened = true;
  decision.primary.forecast_risk_tier = 1;
  decision.primary.forecast_reason = "corridor-reserve";
  decision.primary.minimum_corridor_reserve_m = 0.08;
  decision.primary.first_threat_distance_m = 4.5;
  decision.alternate_attempted = true;
  decision.alternate_selected = true;
  decision.alternate = ready_candidate(-1, -1);
  decision.alternate.forecast_evaluated = true;
  decision.alternate.forecast_reason = "none";
  decision.alternate.minimum_corridor_reserve_m = 0.24;
  decision.alternate.forced_side_transition_requested = true;
  decision.alternate.forced_side_transition_gateway_found = true;
  decision.alternate.forced_side_transition_prefix_samples = 3U;
  decision.alternate.forced_side_transition_gateway_index = 3U;
  decision.alternate.forced_side_transition_gateway_distance_m = 2.4;
  decision.alternate.forced_side_transition_deadline_m = 6.0;
  decision.alternate.forced_side_transition_reason = "side-enforced";
  decision.proactive_alternate = true;
  decision.alternate_trigger_reason = "corridor-reserve";
  decision.branch_selection_reason = "lower-risk-tier";

  const auto message = trace::format_decision_trace(decision);
  EXPECT_NE(message.find("forecast=1/1"), std::string::npos);
  EXPECT_NE(message.find("forecast_reason=corridor-reserve"), std::string::npos);
  EXPECT_NE(message.find("min_reserve=0.08m"), std::string::npos);
  EXPECT_NE(message.find("threat_distance=4.50m"), std::string::npos);
  EXPECT_NE(message.find("proactive_alternate=1"), std::string::npos);
  EXPECT_NE(message.find("branch_selection=lower-risk-tier"), std::string::npos);
  EXPECT_NE(message.find("side_transition=1/1"), std::string::npos);
  EXPECT_NE(message.find("side_transition_prefix=3"), std::string::npos);
  EXPECT_NE(message.find("side_transition_gateway=3@2.40m"), std::string::npos);
  EXPECT_NE(message.find("side_transition_deadline=6.00m"), std::string::npos);
  EXPECT_NE(message.find("side_transition_reason=side-enforced"), std::string::npos);

  decision.alternate_selected = false;
  decision.primary_suppressed = true;
  decision.primary_suppression_reason =
    "immediate-wall-threat-without-alternate";
  const auto suppressed = trace::format_decision_trace(decision);
  EXPECT_NE(
    suppressed.find(
      "primary_suppressed=1/immediate-wall-threat-without-alternate"),
    std::string::npos);
}

TEST(OvertakeDecisionTrace, FormatsTrackingFailureAndRecovery) {
  trace::TrackingTrace tracking;
  tracking.attempt_id = 12U;
  tracking.mission_episode_id = 4U;
  tracking.target_id = "d2";
  tracking.side = -1;
  tracking.consecutive_failures = 2;
  tracking.backoff_sec = 1.0;
  tracking.reason = "maximum iterations reached";
  tracking.preflight_mode = "margin-escape";
  tracking.preflight_margin_escape_used = true;
  tracking.preflight_margin_clear_distance_m = 0.45;
  tracking.tracking_wall_contract_active = true;
  tracking.tracking_wall_contract_reason = "footprint-validation-only";
  tracking.corridor_width_m = 2.3;
  tracking.maximum_target_adjustment_m = 0.4;
  tracking.cold_retry_attempted = true;
  tracking.initial_solver_reason = "warm maximum iterations reached";

  const auto failed = trace::format_tracking_trace(tracking);
  EXPECT_NE(failed.find("stage=tracking"), std::string::npos);
  EXPECT_NE(failed.find("attempt=12"), std::string::npos);
  EXPECT_NE(failed.find("mission_episode=4"), std::string::npos);
  EXPECT_NE(failed.find("outcome=failed"), std::string::npos);
  EXPECT_NE(failed.find("preflight_mode=margin-escape"), std::string::npos);
  EXPECT_NE(
    failed.find("tracking_contract=1/footprint-validation-only"),
    std::string::npos);
  EXPECT_NE(failed.find("cold_retry=1/0"), std::string::npos);

  tracking.outcome = trace::TrackingOutcome::Recovered;
  tracking.backoff_sec = 0.0;
  tracking.reason = "valid tracking solution";
  const auto recovered = trace::format_tracking_trace(tracking);
  EXPECT_NE(recovered.find("outcome=recovered"), std::string::npos);
}

TEST(OvertakeDecisionTrace, EmitsRuntimeFailoverOnlyOnCategoricalChange) {
  trace::ChangeAwareRuntimeFailoverTraceEmitter emitter;
  trace::RuntimeFailoverTrace failover;
  failover.mission_episode_id = 7U;
  failover.mission_generation = 3U;
  failover.target_id = "d2";
  failover.phase = "FollowPrepare";
  failover.trigger =
      "Pass entry physical gate has no valid current-side prefix: initial";
  failover.current_feasible = true;
  failover.current_mission_available = true;
  failover.current_ready = true;
  failover.current_reason = "current corridor valid";
  failover.alternate_feasible = true;
  failover.alternate_mission_available = true;
  failover.alternate_stable = false;
  failover.alternate_reason = "stability pending";
  failover.cross_side_allowed = true;
  failover.cross_side_lease_active = true;
  failover.bounded_lateral_escape_active = true;
  failover.action = "replace-current";
  failover.source = "dynamic-wait-resolver";
  failover.reason = "current replacement ready";
  failover.waypoint_id = 31;

  const auto first = emitter.update(failover, 1.0);
  ASSERT_TRUE(first.emit);
  EXPECT_NE(first.message.find("stage=runtime-failover"), std::string::npos);
  EXPECT_NE(first.message.find("trigger_gate=pass-entry-no-prefix"),
            std::string::npos);
  EXPECT_NE(first.message.find("source=dynamic-wait-resolver"),
            std::string::npos);
  EXPECT_NE(first.message.find("current=1/1/1"), std::string::npos);
  EXPECT_NE(first.message.find("alternate=1/1/0/0/0"), std::string::npos);
  EXPECT_NE(first.message.find("cross_side=1/lease=1"), std::string::npos);
  EXPECT_NE(first.message.find("escape_authority=1"), std::string::npos);
  EXPECT_NE(first.message.find("reason=\"current corridor valid\""),
            std::string::npos);

  failover.waypoint_id = 32;
  EXPECT_FALSE(emitter.update(failover, 1.1).emit);
  failover.reason = "asynchronous assessment text changed";
  failover.trigger =
      "Pass entry physical gate has no valid current-side prefix: updated detail";
  EXPECT_FALSE(emitter.update(failover, 1.15).emit);
  failover.alternate_ready = true;
  failover.action = "replace-alternate";
  EXPECT_TRUE(emitter.update(failover, 1.2).emit);
  failover.source = "opponent-side-replan";
  EXPECT_TRUE(emitter.update(failover, 1.3).emit);
}

TEST(OvertakeDecisionTrace, ClassifiesRuntimeFailoverTriggerGates) {
  EXPECT_EQ(
      trace::classify_runtime_failover_trigger(
          "physical target separation conflicts with wall corridor"),
      "target-wall-conflict");
  EXPECT_EQ(
      trace::classify_runtime_failover_trigger(
          "optimized horizon failed physical revalidation"),
      "optimized-horizon-physical");
  EXPECT_EQ(
      trace::classify_runtime_failover_trigger(
          "live overtake corridor unavailable"),
      "live-corridor-unavailable");
  EXPECT_EQ(
      trace::classify_runtime_failover_trigger(
          "locked target entered selected pass-side line"),
      "pass-side-intrusion");
  EXPECT_EQ(trace::classify_runtime_failover_trigger("unclassified detail"),
            "other");
}

} // namespace
