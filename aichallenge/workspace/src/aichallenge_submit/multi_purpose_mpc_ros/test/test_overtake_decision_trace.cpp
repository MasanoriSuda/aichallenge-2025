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
  decision.primary.static_wall_preflight_reason = "occupied";
  decision.primary.static_wall_checked_poses = 14U;
  decision.primary.static_wall_execution_samples = 7U;
  decision.primary.static_wall_active_samples = 5U;
  decision.primary.static_wall_first_active_index = 1U;
  decision.primary.static_wall_last_active_index = 5U;
  decision.primary.static_wall_invalid_index = 3U;

  const std::string message = trace::format_decision_trace(decision);
  EXPECT_NE(message.find("state=bridge-rejected"), std::string::npos);
  EXPECT_NE(message.find("preflight=1/0"), std::string::npos);
  EXPECT_NE(message.find("preflight_reason=\"occupied\""), std::string::npos);
  EXPECT_NE(message.find("preflight_samples=7/5"), std::string::npos);
  EXPECT_NE(message.find("preflight_range=1:5"), std::string::npos);
  EXPECT_NE(message.find("preflight_invalid_index=3"), std::string::npos);
  EXPECT_NE(message.find("preflight_poses=14"), std::string::npos);
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
  EXPECT_FALSE(emitter.update(decision, 5.9).emit);
  EXPECT_TRUE(emitter.update(decision, 6.3).emit);
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

  const auto failed = trace::format_tracking_trace(tracking);
  EXPECT_NE(failed.find("stage=tracking"), std::string::npos);
  EXPECT_NE(failed.find("attempt=12"), std::string::npos);
  EXPECT_NE(failed.find("mission_episode=4"), std::string::npos);
  EXPECT_NE(failed.find("outcome=failed"), std::string::npos);

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
