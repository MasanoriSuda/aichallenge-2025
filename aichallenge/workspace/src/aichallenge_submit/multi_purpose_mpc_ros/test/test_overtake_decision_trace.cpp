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
  decision.episode_id = 9U;
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
  EXPECT_NE(message.find("state=backed-off"), std::string::npos);
  EXPECT_NE(message.find("state=bridge-rejected"), std::string::npos);
  EXPECT_NE(message.find("corridor is outside the reachable lateral envelope"),
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

  decision.tracking_qualified = true;
  EXPECT_TRUE(emitter.update(decision, 1.2).emit);
  EXPECT_FALSE(emitter.update(decision, 5.9).emit);
  EXPECT_TRUE(emitter.update(decision, 6.2).emit);
}

TEST(OvertakeDecisionTrace, FormatsTrackingFailureAndRecovery) {
  trace::TrackingTrace tracking;
  tracking.episode_id = 4U;
  tracking.target_id = "d2";
  tracking.side = -1;
  tracking.consecutive_failures = 2;
  tracking.backoff_sec = 1.0;
  tracking.reason = "maximum iterations reached";

  const auto failed = trace::format_tracking_trace(tracking);
  EXPECT_NE(failed.find("stage=tracking"), std::string::npos);
  EXPECT_NE(failed.find("outcome=failed"), std::string::npos);

  tracking.outcome = trace::TrackingOutcome::Recovered;
  tracking.backoff_sec = 0.0;
  tracking.reason = "valid tracking solution";
  const auto recovered = trace::format_tracking_trace(tracking);
  EXPECT_NE(recovered.find("outcome=recovered"), std::string::npos);
}

} // namespace
