#include "multi_purpose_mpc_ros/mpcc_rate_resolved_shadow.hpp"

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <cmath>

namespace shadow =
  multi_purpose_mpc_ros::mpcc_rate_resolved_shadow;
namespace adapter =
  multi_purpose_mpc_ros::mpcc_rate_resolved_adapter;
namespace contract =
  multi_purpose_mpc_ros::mpcc_execution_contract;

namespace
{

adapter::Request straight_request(const int horizon = 3)
{
  adapter::Request request;
  request.horizon_steps = horizon;
  request.initial_state << 0.0, 0.0, 0.0, 2.0, 0.0;
  request.current_steering_rad = 0.10;
  request.wheelbase_m = 2.0;
  request.maximum_abs_steering_rad = 0.60;
  request.maximum_abs_steering_rate_radps = 0.70;
  request.states.resize(static_cast<std::size_t>(horizon + 1));
  for (int stage = 0; stage <= horizon; ++stage) {
    auto & state = request.states[static_cast<std::size_t>(stage)];
    state.reference << 0.0, 0.0, 0.0, 2.0, 0.2 * stage;
    state.lower << -1.0, -1.0, -1.0, 0.0, -1.0;
    state.upper << 1.0, 1.0, 1.0, 4.0, 2.0;
    state.weight << 2.0, 1.0, 2.0, 1.0, 1.0;
    state.linear_cost[4] = -0.5;
  }
  request.inputs.resize(static_cast<std::size_t>(horizon));
  for (int stage = 0; stage < horizon; ++stage) {
    auto & input = request.inputs[static_cast<std::size_t>(stage)];
    input.reference << 0.0, std::tan(0.10) / 2.0, 2.0;
    input.lower << -1.0, -0.30, 0.0;
    input.upper << 1.0, 0.30, 4.0;
    input.weight << 1.0, 1.0, 1.0;
    input.path_curvature_radpm = 0.0;
    input.stage_dt_sec = 0.10;
  }
  request.previous_input << 0.0, std::tan(0.10) / 2.0, 2.0;
  request.input_delta_weight << 0.2, 0.3, 0.1;
  return request;
}

shadow::Snapshot snapshot(const std::uint64_t sequence = 1U)
{
  shadow::Snapshot result;
  result.identity.sequence = sequence;
  result.identity.decision_id = 42U + sequence;
  result.identity.source_problem_fingerprint = 101U + sequence;
  result.identity.stage_geometry_id = 201U + sequence;
  result.identity.intent = contract::ControlIntent::Track;
  result.identity.snapshot_sec = 10.0 + 0.1 * sequence;
  result.request = straight_request();
  result.publication_interval_sec = 0.025;
  return result;
}

}  // namespace

TEST(MpccRateResolvedShadow, SolvesAndSamplesOnePublicationInterval)
{
  shadow::SolverContext context;
  const auto input = snapshot();
  const auto result = context.evaluate(input);
  ASSERT_EQ(result.outcome, shadow::Outcome::Solved) << result.detail;
  EXPECT_TRUE(shadow::result_valid(result));
  EXPECT_TRUE(result.constraints_satisfied);
  EXPECT_TRUE(result.actuation_sampled);
  EXPECT_NEAR(
    result.sampled_steering_rad,
    result.initial_steering_rad +
    result.first_steering_rate_radps * input.publication_interval_sec,
    1e-9);
  EXPECT_LE(
    std::abs(result.first_steering_rate_radps),
    input.request.maximum_abs_steering_rate_radps + 1e-6);
}

TEST(MpccRateResolvedShadow, RejectsPublicationPeriodBeyondFirstStage)
{
  shadow::SolverContext context;
  auto input = snapshot();
  input.publication_interval_sec = 0.20;
  const auto result = context.evaluate(input);
  EXPECT_EQ(result.outcome, shadow::Outcome::ActuationSampleRejected);
  EXPECT_EQ(
    result.actuation_sample_reason,
    multi_purpose_mpc_ros::mpcc_rate_resolved::ActuationSampleReason::
    PublicationAfterStageEnd);
  EXPECT_DOUBLE_EQ(result.publication_interval_sec, 0.20);
  EXPECT_DOUBLE_EQ(result.first_stage_duration_sec, 0.10);
  EXPECT_TRUE(shadow::result_valid(result));
}

TEST(MpccRateResolvedShadowMailbox, PublishesMonotonicRegisteredResults)
{
  shadow::SolverContext context;
  shadow::Mailbox mailbox;
  ASSERT_TRUE(mailbox.register_submission(1U));
  const auto first = context.evaluate(snapshot(1U));
  EXPECT_EQ(mailbox.publish(first), shadow::PublishReason::Accepted);
  const auto available = mailbox.latest_after(0U);
  ASSERT_TRUE(available.has_value());
  EXPECT_EQ(available->identity.sequence, 1U);
  EXPECT_FALSE(mailbox.latest_after(1U).has_value());

  ASSERT_TRUE(mailbox.register_submission(2U));
  const auto second = context.evaluate(snapshot(2U));
  EXPECT_EQ(mailbox.publish(second), shadow::PublishReason::Accepted);
  EXPECT_EQ(
    mailbox.publish(first), shadow::PublishReason::SequenceRollback);
  const auto state = mailbox.state();
  EXPECT_EQ(state.accepted_count, 2U);
  EXPECT_EQ(state.sequence_rollback_count, 1U);
}

TEST(MpccRateResolvedShadowMailbox, RejectsUnregisteredAndInvalidResults)
{
  shadow::SolverContext context;
  shadow::Mailbox mailbox;
  const auto result = context.evaluate(snapshot(2U));
  EXPECT_EQ(
    mailbox.publish(result), shadow::PublishReason::SequenceNotSubmitted);

  ASSERT_TRUE(mailbox.register_submission(3U));
  auto invalid = context.evaluate(snapshot(3U));
  invalid.identity.source_problem_fingerprint = 0U;
  EXPECT_EQ(mailbox.publish(invalid), shadow::PublishReason::InvalidResult);
}
