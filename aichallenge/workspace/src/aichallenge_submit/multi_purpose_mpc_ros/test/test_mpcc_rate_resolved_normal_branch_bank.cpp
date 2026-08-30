#include "multi_purpose_mpc_ros/mpcc_rate_resolved_normal_branch_bank.hpp"

#include <gtest/gtest.h>

#include <memory>

namespace
{

namespace bank =
  multi_purpose_mpc_ros::mpcc_rate_resolved_normal_branch_bank;
namespace certified =
  multi_purpose_mpc_ros::mpcc_rate_resolved_certified_plan;
namespace execution =
  multi_purpose_mpc_ros::mpcc_rate_resolved_execution_artifact;
namespace physical =
  multi_purpose_mpc_ros::mpcc_rate_resolved_physical_wall;
namespace shadow = multi_purpose_mpc_ros::mpcc_rate_resolved_shadow;
namespace contract = multi_purpose_mpc_ros::mpcc_execution_contract;
namespace recovery = multi_purpose_mpc_ros::recovery_footprint;

contract::MpccProblemContext source_context(
  const std::uint64_t sequence, const int side_sign)
{
  contract::MpccProblemContext context;
  context.decision_id = sequence + 100U;
  context.intent = contract::ControlIntent::Follow;
  context.intent_generation = 3U;
  context.observation_generation = sequence + 200U;
  context.stage_geometry_id = sequence + 300U;
  context.target_obstacle_generation = sequence + 400U;
  context.target_id = "d2";
  context.dynamic_obstacle_constraint_active = true;
  context.dynamic_obstacle_generation = sequence + 400U;
  context.dynamic_obstacle_id = "d2";
  context.dynamic_obstacle_side_sign = side_sign;
  context.horizon_steps = 2U;
  context.formulation =
    contract::Formulation::VelocitySteeringYawResponseProgress7State;
  context.state_schema_id = "ey-elag-epsi-v-progress-steering-response-v1";
  context.input_schema_id = "accel-steering-rate-progress-rate-v1";
  context.bounds_schema_id = "stage-wall-dynamic-v1";
  context.cost_schema_id = "velocity-progress-steering-rate-v1";
  return contract::seal_problem_context(std::move(context));
}

shadow::Snapshot source_snapshot(const std::uint64_t sequence)
{
  shadow::Snapshot source;
  source.identity = execution::Identity{
    sequence, source_context(sequence, 0), 10.0 + sequence};
  return source;
}

execution::ExecutionArtifact artifact(
  const std::uint64_t sequence, const int side_sign)
{
  execution::ExecutionArtifact value;
  value.identity = execution::Identity{
    sequence, source_context(sequence, side_sign), 10.0 + sequence};
  value.prediction_origin_sec = value.identity.snapshot_sec;
  value.publication_interval_sec = 0.025;
  value.completed_sec = value.prediction_origin_sec + 0.01;
  value.course_progress_origin_m = 50.0;
  value.semantic_initial_steering_rad = 0.10;
  value.semantic_initial_response_steering_rad = 0.10;
  value.wheelbase_m = 2.0;
  value.maximum_abs_steering_rad = 0.60;
  value.maximum_abs_steering_rate_radps = 1.0;
  value.physical_global_tolerance = 1e-6;
  value.maximum_constraint_violation = 1e-8;
  value.maximum_normalized_constraint_violation = 0.1;
  value.predicted_states = {
    {0.0, 0.1, 0.0, 2.0, 0.0, 0.10, 0.10},
    {0.1, 0.0, 0.01, 2.1, 0.2, 0.11, 0.10302380180000528},
    {0.2, 0.0, 0.02, 2.2, 0.4, 0.12, 0.10979124524044208},
  };
  value.control_stages = {
    {1.0, 0.10, 2.0, 0.10, 0.0, 4.0, -3.0, 1.37},
    {1.0, 0.10, 2.0, 0.10, 0.0, 4.0, -3.0, 1.37},
  };
  value.nominal_path_distance_m = {0.0, 0.2, 0.4};
  value.lateral_lower_m = {-1.0, -1.0, -1.0};
  value.lateral_upper_m = {1.0, 1.0, 1.0};
  return value;
}

physical::Snapshot physical_snapshot(const execution::Identity & identity)
{
  physical::Snapshot snapshot;
  snapshot.identity.artifact = identity;
  snapshot.identity.pose_snapshot_id = 101U;
  snapshot.identity.course_frame_window_id = 102U;
  snapshot.identity.captured_sec = identity.snapshot_sec;
  auto grid = std::make_shared<recovery::OccupancyGrid>();
  grid->width = 400U;
  grid->height = 400U;
  grid->resolution_m = 0.1;
  grid->origin_x_m = 30.0;
  grid->origin_y_m = -20.0;
  grid->cells.assign(grid->width * grid->height, recovery::CellState::Free);
  snapshot.wall_grid = std::move(grid);
  snapshot.wall_grid_fingerprint =
    recovery::occupancy_grid_fingerprint(*snapshot.wall_grid);
  snapshot.footprint = {0.1, 0.1, 0.1, 0.1, 0.0};
  snapshot.current_pose = {50.0, 0.0, 0.0};
  snapshot.control_prefix = {snapshot.current_pose};
  snapshot.trajectory.progress_origin_m = 50.0;
  snapshot.trajectory.elapsed_time_sec = {0.1, 0.2};
  snapshot.trajectory.path_distance_m = {0.2, 0.4};
  snapshot.trajectory.lateral_m = {0.1, 0.2};
  snapshot.trajectory.lag_m = {0.0, 0.0};
  snapshot.trajectory.heading_offset_rad = {0.01, 0.02};
  snapshot.trajectory.velocity_mps = {2.1, 2.2};
  snapshot.trajectory.progress_m = {50.2, 50.4};
  snapshot.trajectory.lateral_lower_m = {-1.0, -1.0};
  snapshot.trajectory.lateral_upper_m = {1.0, 1.0};
  snapshot.trajectory.minimum_lateral_bound_reserve_m = 0.8;
  snapshot.trajectory.progress_regression_tolerance_m = 1e-6;
  snapshot.course_frame_knots = {
    {49.0, 49.0, 0.0, 0.0, 0},
    {52.0, 52.0, 0.0, 0.0, 3},
  };
  snapshot.terminal_stop_course_geometry = {
    {0.0, 1.0, 2.0}, {0.0, 0.0},
    {-1.0, -1.0, -1.0}, {1.0, 1.0, 1.0}};
  snapshot.hard_wall_clearance_m = 0.1;
  snapshot.bound_tolerance_m = 1e-6;
  snapshot.swept_step_m = 0.05;
  return snapshot;
}

std::shared_ptr<const certified::CertifiedPlan> plan(
  const std::uint64_t sequence, const int side_sign)
{
  auto execution_artifact =
    std::make_shared<const execution::ExecutionArtifact>(
    artifact(sequence, side_sign));
  const auto physical_source =
    physical_snapshot(execution_artifact->identity);
  physical::Result result;
  result.identity = physical_source.identity;
  result.outcome = physical::Outcome::Accepted;
  result.diagnostic.reason =
    contract::PhysicalWallCertificateReason::Accepted;
  result.completed_sec = execution_artifact->identity.snapshot_sec + 0.02;
  result.compute_ms = 20.0;
  result.detail = "accepted";
  return certified::build(
    execution_artifact, physical_source, result).plan;
}

TEST(NormalBranchBank, PublishesBothBranchesFromOneEpochAtomically)
{
  bank::Bank subject;
  const auto source = source_snapshot(5U);
  const auto negative = plan(5U, -1);
  const auto positive = plan(5U, 1);

  EXPECT_EQ(
    subject.replace(source, negative, positive), bank::ReplaceReason::Accepted);
  const auto stored = subject.snapshot();
  EXPECT_EQ(stored.source_identity.sequence, 5U);
  EXPECT_EQ(stored.plan_for_side(-1), negative);
  EXPECT_EQ(stored.plan_for_side(1), positive);
  EXPECT_TRUE(stored.available());
}

TEST(NormalBranchBank, NewEmptyEpochInvalidatesBothOlderBranches)
{
  bank::Bank subject;
  ASSERT_EQ(
    subject.replace(source_snapshot(5U), plan(5U, -1), plan(5U, 1)),
    bank::ReplaceReason::Accepted);

  EXPECT_EQ(
    subject.replace(source_snapshot(6U), nullptr, nullptr),
    bank::ReplaceReason::Accepted);
  const auto stored = subject.snapshot();
  EXPECT_EQ(stored.source_identity.sequence, 6U);
  EXPECT_EQ(stored.negative_plan, nullptr);
  EXPECT_EQ(stored.positive_plan, nullptr);
  EXPECT_FALSE(stored.available());
}

TEST(NormalBranchBank, MergesIndependentlyCompletedBranchesInEitherOrder)
{
  bank::Bank subject;
  const auto source = source_snapshot(5U);
  const auto negative = plan(5U, -1);
  const auto positive = plan(5U, 1);

  EXPECT_EQ(
    subject.merge_branch(source, 1, positive), bank::ReplaceReason::Accepted);
  auto stored = subject.snapshot();
  EXPECT_EQ(stored.negative_plan, nullptr);
  EXPECT_EQ(stored.positive_plan, positive);

  EXPECT_EQ(
    subject.merge_branch(source, -1, negative), bank::ReplaceReason::Accepted);
  stored = subject.snapshot();
  EXPECT_EQ(stored.negative_plan, negative);
  EXPECT_EQ(stored.positive_plan, positive);
}

TEST(NormalBranchBank, NewerBranchInvalidatesOldPairAndRejectsLateSibling)
{
  bank::Bank subject;
  ASSERT_EQ(
    subject.replace(source_snapshot(5U), plan(5U, -1), plan(5U, 1)),
    bank::ReplaceReason::Accepted);
  const auto newer_positive = plan(6U, 1);

  EXPECT_EQ(
    subject.merge_branch(source_snapshot(6U), 1, newer_positive),
    bank::ReplaceReason::Accepted);
  auto stored = subject.snapshot();
  EXPECT_EQ(stored.source_identity.sequence, 6U);
  EXPECT_EQ(stored.negative_plan, nullptr);
  EXPECT_EQ(stored.positive_plan, newer_positive);

  EXPECT_EQ(
    subject.merge_branch(source_snapshot(5U), -1, plan(5U, -1)),
    bank::ReplaceReason::StaleSource);
  stored = subject.snapshot();
  EXPECT_EQ(stored.source_identity.sequence, 6U);
  EXPECT_EQ(stored.negative_plan, nullptr);
  EXPECT_EQ(stored.positive_plan, newer_positive);
}

TEST(NormalBranchBank, RejectsInvalidMergeSideAndMismatchedEpoch)
{
  bank::Bank subject;
  EXPECT_EQ(
    subject.merge_branch(source_snapshot(5U), 0, nullptr),
    bank::ReplaceReason::InvalidSide);
  EXPECT_EQ(
    subject.merge_branch(source_snapshot(5U), -1, plan(4U, -1)),
    bank::ReplaceReason::InvalidNegativePlan);
  EXPECT_EQ(subject.state().latest_source_sequence, 0U);
}

TEST(NormalBranchBank, RejectsStaleSourceWithoutChangingAtomicEntry)
{
  bank::Bank subject;
  const auto negative = plan(6U, -1);
  const auto positive = plan(6U, 1);
  ASSERT_EQ(
    subject.replace(source_snapshot(6U), negative, positive),
    bank::ReplaceReason::Accepted);

  EXPECT_EQ(
    subject.replace(source_snapshot(5U), plan(5U, -1), plan(5U, 1)),
    bank::ReplaceReason::StaleSource);
  const auto stored = subject.snapshot();
  EXPECT_EQ(stored.source_identity.sequence, 6U);
  EXPECT_EQ(stored.negative_plan, negative);
  EXPECT_EQ(stored.positive_plan, positive);
}

TEST(NormalBranchBank, RejectsPlanFromAnotherEpochOrWrongSide)
{
  bank::Bank subject;
  EXPECT_EQ(
    subject.replace(source_snapshot(5U), plan(4U, -1), plan(5U, 1)),
    bank::ReplaceReason::InvalidNegativePlan);
  EXPECT_EQ(
    subject.replace(source_snapshot(5U), plan(5U, 1), nullptr),
    bank::ReplaceReason::InvalidNegativePlan);
  EXPECT_EQ(subject.state().latest_source_sequence, 0U);
}

TEST(NormalBranchBank, RejectsNonNormalSource)
{
  bank::Bank subject;
  auto source = source_snapshot(5U);
  auto context = source.identity.source_context;
  context.intent = contract::ControlIntent::Pass;
  context.execution_side_sign = 1;
  source.identity.source_context = contract::seal_problem_context(context);

  EXPECT_EQ(
    subject.replace(source, nullptr, nullptr),
    bank::ReplaceReason::InvalidSource);
}

}  // namespace
