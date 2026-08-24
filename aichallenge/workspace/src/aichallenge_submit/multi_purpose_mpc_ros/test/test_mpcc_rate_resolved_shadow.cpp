#include "multi_purpose_mpc_ros/mpcc_rate_resolved_shadow.hpp"

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <cmath>
#include <limits>

namespace shadow =
  multi_purpose_mpc_ros::mpcc_rate_resolved_shadow;
namespace adapter =
  multi_purpose_mpc_ros::mpcc_rate_resolved_adapter;
namespace execution =
  multi_purpose_mpc_ros::mpcc_rate_resolved_execution_artifact;
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
  result.course_progress_origin_m = 50.0;
  result.nominal_path_distance_m = {0.0, 0.2, 0.4, 0.6};
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
  EXPECT_DOUBLE_EQ(result.initial_steering_rad, input.request.current_steering_rad);
  EXPECT_TRUE(std::isfinite(result.solver_initial_steering_rad));
  EXPECT_NEAR(
    result.sampled_steering_rad,
    result.initial_steering_rad +
    result.first_steering_rate_radps * input.publication_interval_sec,
    1e-9);
  EXPECT_LE(
    std::abs(result.first_steering_rate_radps),
    input.request.maximum_abs_steering_rate_radps + 1e-6);
  EXPECT_LT(
    result.first_steering_rate_physical_lower_radps,
    result.first_steering_rate_solver_lower_radps);
  EXPECT_LT(
    result.first_steering_rate_solver_upper_radps,
    result.first_steering_rate_physical_upper_radps);
  EXPECT_GT(result.first_steering_rate_certificate_margin_radps, 0.0);
  EXPECT_GE(
    result.first_steering_rate_radps,
    result.first_steering_rate_solver_lower_radps -
    result.solver.physical_global_tolerance);
  EXPECT_LE(
    result.first_steering_rate_radps,
    result.first_steering_rate_solver_upper_radps +
    result.solver.physical_global_tolerance);
  EXPECT_EQ(result.certified_stage_count, 3U);
  EXPECT_EQ(result.sampled_stage_index, 0U);
  EXPECT_NEAR(result.sampled_stage_elapsed_sec, 0.025, 1e-12);
  EXPECT_NEAR(result.certified_horizon_duration_sec, 0.30, 1e-12);
  ASSERT_NE(result.execution_artifact, nullptr);
  EXPECT_EQ(
    execution::validate(*result.execution_artifact),
    execution::RejectReason::None);
  EXPECT_EQ(result.execution_artifact->predicted_states.size(), 4U);
  EXPECT_EQ(result.execution_artifact->control_stages.size(), 3U);
  EXPECT_EQ(result.execution_artifact->nominal_path_distance_m.size(), 4U);
  EXPECT_EQ(result.execution_artifact->lateral_lower_m.size(), 4U);
  EXPECT_DOUBLE_EQ(result.execution_artifact->course_progress_origin_m, 50.0);
  EXPECT_DOUBLE_EQ(
    result.execution_artifact->semantic_initial_steering_rad,
    input.request.current_steering_rad);
  EXPECT_DOUBLE_EQ(
    result.execution_artifact->control_stages.front().steering_rate_radps,
    result.first_steering_rate_radps);

  auto invalid = result;
  invalid.first_steering_rate_certificate_margin_radps =
    std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(shadow::result_valid(invalid));
}

TEST(MpccRateResolvedShadow, RetainsAndSamplesExactRateResolvedArtifact)
{
  shadow::SolverContext context;
  const auto result = context.evaluate(snapshot());
  ASSERT_EQ(result.outcome, shadow::Outcome::Solved) << result.detail;
  ASSERT_NE(result.execution_artifact, nullptr);
  const auto & artifact = *result.execution_artifact;

  const auto cursor = execution::resolve_cursor(
    artifact, artifact.prediction_origin_sec + 0.15);
  ASSERT_TRUE(cursor.available);
  EXPECT_EQ(cursor.reason, execution::CursorReason::Available);
  EXPECT_EQ(cursor.control_stage_index, 1U);
  EXPECT_EQ(cursor.remaining_control_stage_count, 2U);
  EXPECT_NEAR(cursor.stage_elapsed_sec, 0.05, 1e-12);

  const auto actuation = execution::extract_actuation(
    artifact, cursor);
  ASSERT_TRUE(actuation.actuation.has_value());
  EXPECT_EQ(actuation.reason, execution::ActuationReason::Available);
  EXPECT_EQ(
    actuation.sample_reason,
    multi_purpose_mpc_ros::mpcc_rate_resolved::ActuationSampleReason::Accepted);
  const double expected_steering =
    artifact.semantic_initial_steering_rad +
    artifact.control_stages[0].steering_rate_radps *
    artifact.control_stages[0].duration_sec +
    artifact.control_stages[1].steering_rate_radps * 0.05;
  EXPECT_NEAR(
    actuation.actuation->steering_rad, expected_steering, 1e-9);
  EXPECT_DOUBLE_EQ(
    actuation.actuation->steering_rate_radps,
    artifact.control_stages[1].steering_rate_radps);

  const auto exhausted = execution::resolve_cursor(
    artifact, artifact.prediction_origin_sec + 0.30);
  EXPECT_FALSE(exhausted.available);
  EXPECT_EQ(exhausted.reason, execution::CursorReason::Exhausted);
}

TEST(MpccRateResolvedShadow, RejectsMutatedExecutionArtifactProvenance)
{
  shadow::SolverContext context;
  const auto result = context.evaluate(snapshot());
  ASSERT_EQ(result.outcome, shadow::Outcome::Solved) << result.detail;
  ASSERT_NE(result.execution_artifact, nullptr);

  auto invalid = *result.execution_artifact;
  invalid.identity.source_problem_fingerprint = 0U;
  EXPECT_EQ(
    execution::validate(invalid), execution::RejectReason::InvalidIdentity);

  invalid = *result.execution_artifact;
  invalid.course_progress_origin_m =
    std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(
    execution::validate(invalid),
    execution::RejectReason::InvalidCourseProgressOrigin);

  invalid = *result.execution_artifact;
  invalid.predicted_states.pop_back();
  EXPECT_EQ(
    execution::validate(invalid), execution::RejectReason::StateCountMismatch);

  invalid = *result.execution_artifact;
  invalid.nominal_path_distance_m.pop_back();
  EXPECT_EQ(
    execution::validate(invalid),
    execution::RejectReason::PathDistanceCountMismatch);

  invalid = *result.execution_artifact;
  invalid.nominal_path_distance_m[1] = 0.0;
  EXPECT_EQ(
    execution::validate(invalid), execution::RejectReason::InvalidPathDistance);

  invalid = *result.execution_artifact;
  invalid.predicted_states.front().steering_rad += 0.1;
  EXPECT_EQ(
    execution::validate(invalid), execution::RejectReason::InitialSteeringMismatch);

  invalid = *result.execution_artifact;
  invalid.predicted_states[1].steering_rad += 0.1;
  EXPECT_EQ(
    execution::validate(invalid), execution::RejectReason::SteeringDynamicsMismatch);

  invalid = *result.execution_artifact;
  invalid.lateral_upper_m[1] = invalid.predicted_states[1].lateral_m - 0.1;
  EXPECT_EQ(
    execution::validate(invalid), execution::RejectReason::InvalidLateralCorridor);

  invalid = *result.execution_artifact;
  invalid.maximum_normalized_constraint_violation = 1.01;
  EXPECT_EQ(
    execution::validate(invalid), execution::RejectReason::InvalidCertificate);
}

TEST(MpccRateResolvedShadow, SamplesPublicationPeriodAcrossStageBoundary)
{
  shadow::SolverContext context;
  auto input = snapshot();
  input.publication_interval_sec = 0.20;
  const auto result = context.evaluate(input);
  ASSERT_EQ(result.outcome, shadow::Outcome::Solved) << result.detail;
  EXPECT_TRUE(shadow::result_valid(result));
  EXPECT_EQ(result.sampled_stage_index, 1U);
  EXPECT_NEAR(result.sampled_stage_elapsed_sec, 0.10, 1e-12);
  EXPECT_NEAR(result.certified_horizon_duration_sec, 0.30, 1e-12);
}

TEST(MpccRateResolvedShadow, RejectsPublicationPeriodBeyondCertifiedHorizon)
{
  shadow::SolverContext context;
  auto input = snapshot();
  input.publication_interval_sec = 0.31;
  const auto result = context.evaluate(input);
  EXPECT_EQ(result.outcome, shadow::Outcome::ActuationSampleRejected);
  EXPECT_EQ(
    result.actuation_sample_reason,
    multi_purpose_mpc_ros::mpcc_rate_resolved::ActuationSampleReason::
    PublicationAfterHorizonEnd);
  EXPECT_DOUBLE_EQ(result.publication_interval_sec, 0.31);
  EXPECT_DOUBLE_EQ(result.first_stage_duration_sec, 0.10);
  EXPECT_NEAR(result.certified_horizon_duration_sec, 0.30, 1e-12);
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
