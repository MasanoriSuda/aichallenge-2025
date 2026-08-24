#include "multi_purpose_mpc_ros/mpcc_rate_resolved_certified_plan.hpp"

#include <gtest/gtest.h>

#include <memory>

namespace
{

namespace certified =
  multi_purpose_mpc_ros::mpcc_rate_resolved_certified_plan;
namespace execution =
  multi_purpose_mpc_ros::mpcc_rate_resolved_execution_artifact;
namespace physical =
  multi_purpose_mpc_ros::mpcc_rate_resolved_physical_wall;
namespace contract = multi_purpose_mpc_ros::mpcc_execution_contract;
namespace recovery = multi_purpose_mpc_ros::recovery_footprint;

execution::ExecutionArtifact artifact(const std::uint64_t sequence = 1U)
{
  execution::ExecutionArtifact value;
  value.identity = execution::Identity{
    sequence, sequence + 10U, sequence + 20U, sequence + 30U,
    contract::ControlIntent::Track, 10.0 + static_cast<double>(sequence)};
  value.prediction_origin_sec = value.identity.snapshot_sec;
  value.completed_sec = value.prediction_origin_sec + 0.01;
  value.course_progress_origin_m = 50.0;
  value.semantic_initial_steering_rad = 0.10;
  value.wheelbase_m = 2.0;
  value.maximum_abs_steering_rad = 0.60;
  value.maximum_abs_steering_rate_radps = 1.0;
  value.physical_global_tolerance = 1e-6;
  value.maximum_constraint_violation = 1e-8;
  value.maximum_normalized_constraint_violation = 0.1;
  value.predicted_states = {
    {0.0, 0.1, 0.0, 2.0, 0.0, 0.10},
    {0.1, 0.0, 0.01, 2.1, 0.2, 0.11},
    {0.2, 0.0, 0.02, 2.2, 0.4, 0.12},
  };
  value.control_stages = {
    {1.0, 0.10, 2.0, 0.10, 0.0, 4.0},
    {1.0, 0.10, 2.0, 0.10, 0.0, 4.0},
  };
  value.nominal_path_distance_m = {0.0, 0.2, 0.4};
  value.lateral_lower_m = {-1.0, -1.0, -1.0};
  value.lateral_upper_m = {1.0, 1.0, 1.0};
  return value;
}

physical::Result accepted_physical(const execution::Identity & identity)
{
  physical::Result result;
  result.identity.artifact = identity;
  result.identity.pose_snapshot_id = 101U;
  result.identity.course_frame_window_id = 102U;
  result.identity.captured_sec = identity.snapshot_sec;
  result.outcome = physical::Outcome::Accepted;
  result.diagnostic.reason =
    contract::PhysicalWallCertificateReason::Accepted;
  result.completed_sec = identity.snapshot_sec + 0.02;
  result.compute_ms = 20.0;
  result.detail = "accepted";
  return result;
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
  snapshot.footprint = {0.1, 0.1, 0.1, 0.1, 0.0};
  snapshot.current_pose = {50.0, 0.0, 0.0};
  snapshot.trajectory.progress_origin_m = 50.0;
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
  snapshot.bound_tolerance_m = 1e-6;
  snapshot.swept_step_m = 0.05;
  return snapshot;
}

certified::BuildResult build_plan(const std::uint64_t sequence = 1U)
{
  auto value = std::make_shared<const execution::ExecutionArtifact>(
    artifact(sequence));
  return certified::build(
    value, physical_snapshot(value->identity),
    accepted_physical(value->identity));
}

TEST(MpccRateResolvedCertifiedPlan, JoinsExactArtifactAndPhysicalProof)
{
  const auto result = build_plan();
  ASSERT_EQ(result.reason, certified::RejectReason::None);
  ASSERT_NE(result.plan, nullptr);
  EXPECT_EQ(certified::validate(*result.plan), certified::RejectReason::None);
  EXPECT_EQ(result.plan->execution_artifact->identity.sequence, 1U);
  EXPECT_EQ(
    result.plan->physical_diagnostic.reason,
    contract::PhysicalWallCertificateReason::Accepted);
}

TEST(MpccRateResolvedCertifiedPlan, AcceptedAdmissionClearsDiagnosticReason)
{
  certified::Store store;
  auto value = std::make_shared<const execution::ExecutionArtifact>(artifact());
  const auto admission = store.certify_and_replace(
    value, physical_snapshot(value->identity),
    accepted_physical(value->identity));
  EXPECT_TRUE(admission.accepted());
  const auto state = store.state();
  EXPECT_EQ(state.last_certification_reason, certified::RejectReason::None);
  EXPECT_EQ(state.last_reason, certified::StoreReason::Accepted);
}

TEST(MpccRateResolvedCertifiedPlan, RejectsNonAcceptedPhysicalProof)
{
  auto value = std::make_shared<const execution::ExecutionArtifact>(artifact());
  auto proof = accepted_physical(value->identity);
  proof.outcome = physical::Outcome::SweptWallRejected;
  proof.diagnostic.reason =
    contract::PhysicalWallCertificateReason::SweptPathViolation;
  const auto result = certified::build(
    value, physical_snapshot(value->identity), proof);
  EXPECT_EQ(result.reason, certified::RejectReason::PhysicalProofRejected);
  EXPECT_EQ(result.plan, nullptr);
}

TEST(MpccRateResolvedCertifiedPlan, RejectsFullIdentityMismatch)
{
  auto value = std::make_shared<const execution::ExecutionArtifact>(artifact());
  auto proof = accepted_physical(value->identity);
  ++proof.identity.course_frame_window_id;
  // A different world window is still a valid physical result, but mutating
  // the embedded artifact identity must never join to this artifact.
  ++proof.identity.artifact.source_problem_fingerprint;
  const auto result = certified::build(
    value, physical_snapshot(value->identity), proof);
  EXPECT_EQ(result.reason, certified::RejectReason::IdentityMismatch);
  EXPECT_EQ(result.plan, nullptr);
}

TEST(MpccRateResolvedCertifiedPlan, FailedReplacementPreservesAcceptedPlan)
{
  certified::Store store;
  const auto first = build_plan(5U);
  ASSERT_NE(first.plan, nullptr);
  EXPECT_EQ(store.replace(first.plan), certified::StoreReason::Accepted);

  auto malformed = std::make_shared<certified::CertifiedPlan>(*first.plan);
  malformed->physical_outcome = physical::Outcome::StageWallRejected;
  EXPECT_EQ(store.replace(malformed), certified::StoreReason::InvalidPlan);
  const auto retained = store.snapshot();
  ASSERT_NE(retained, nullptr);
  EXPECT_EQ(retained->execution_artifact->identity.sequence, 5U);
}

TEST(MpccRateResolvedCertifiedPlan, AdmissionRecordsTypedCertificationReject)
{
  certified::Store store;
  auto value = std::make_shared<const execution::ExecutionArtifact>(artifact());
  auto proof = accepted_physical(value->identity);
  ++proof.identity.artifact.decision_id;
  const auto admission = store.certify_and_replace(
    value, physical_snapshot(value->identity), proof);
  EXPECT_FALSE(admission.accepted());
  EXPECT_EQ(
    admission.certification_reason, certified::RejectReason::IdentityMismatch);
  const auto state = store.state();
  EXPECT_EQ(state.certification_reject_count, 1U);
  EXPECT_EQ(
    state.last_certification_reason, certified::RejectReason::IdentityMismatch);
  EXPECT_FALSE(state.plan_available);
}

TEST(MpccRateResolvedCertifiedPlan, RejectsSnapshotResultIdentityMismatch)
{
  auto value = std::make_shared<const execution::ExecutionArtifact>(artifact());
  auto snapshot = physical_snapshot(value->identity);
  auto proof = accepted_physical(value->identity);
  ++snapshot.identity.pose_snapshot_id;
  const auto result = certified::build(value, snapshot, proof);
  EXPECT_EQ(
    result.reason, certified::RejectReason::PhysicalSnapshotMismatch);
  EXPECT_EQ(result.plan, nullptr);
}

TEST(MpccRateResolvedCertifiedPlan, RejectsStaleReplacementAndConditionalClear)
{
  certified::Store store;
  const auto newer = build_plan(6U);
  const auto older = build_plan(5U);
  ASSERT_NE(newer.plan, nullptr);
  ASSERT_NE(older.plan, nullptr);
  EXPECT_EQ(store.replace(newer.plan), certified::StoreReason::Accepted);
  EXPECT_EQ(store.replace(older.plan), certified::StoreReason::StaleSequence);
  EXPECT_FALSE(store.clear_if_sequence(5U));
  EXPECT_TRUE(store.clear_if_sequence(6U));
  EXPECT_EQ(store.snapshot(), nullptr);
}

}  // namespace
