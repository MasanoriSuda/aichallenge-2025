#include "multi_purpose_mpc_ros/mpcc_rate_resolved_retained_revalidation.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_production_adapter.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_stop_successor_bundle.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <memory>

namespace
{

namespace retained =
  multi_purpose_mpc_ros::mpcc_rate_resolved_retained_revalidation;
namespace certified =
  multi_purpose_mpc_ros::mpcc_rate_resolved_certified_plan;
namespace artifact =
  multi_purpose_mpc_ros::mpcc_rate_resolved_execution_artifact;
namespace physical =
  multi_purpose_mpc_ros::mpcc_rate_resolved_physical_wall;
namespace recovery = multi_purpose_mpc_ros::recovery_footprint;
namespace contract = multi_purpose_mpc_ros::mpcc_execution_contract;
namespace production =
  multi_purpose_mpc_ros::mpcc_rate_resolved_production_adapter;
namespace stop_bundle =
  multi_purpose_mpc_ros::mpcc_rate_resolved_stop_successor_bundle;

contract::MpccProblemContext source_context(
  const contract::ControlIntent intent = contract::ControlIntent::Track)
{
  contract::MpccProblemContext context;
  context.decision_id = 11U;
  context.intent = intent;
  context.intent_generation = 1U;
  context.observation_generation = 2U;
  if (contract::canonical_normal_intent_requires_target(intent)) {
    context.target_obstacle_generation = 2U;
    context.target_id = "d2";
  }
  if (contract::canonical_normal_intent_requires_execution_side(intent)) {
    context.execution_side_sign = -1;
  }
  context.stage_geometry_id = 31U;
  context.horizon_steps = 2U;
  context.formulation =
    contract::Formulation::VelocitySteeringYawResponseProgress7State;
  context.state_schema_id = "ey-elag-epsi-v-progress-steering-v1";
  context.input_schema_id = "accel-steering-rate-progress-rate-v1";
  context.bounds_schema_id = "stage-wall-v1";
  context.cost_schema_id = "velocity-progress-steering-rate-v1";
  return contract::seal_problem_context(std::move(context));
}

artifact::ExecutionArtifact execution_artifact(
  const contract::ControlIntent intent = contract::ControlIntent::Track)
{
  artifact::ExecutionArtifact value;
  value.identity = {1U, source_context(intent), 1.0};
  value.prediction_origin_sec = 1.0;
  value.publication_interval_sec = 0.025;
  value.completed_sec = 1.01;
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
    {0.0, 0.10, 0.0, 2.0, 0.0, 0.10, 0.10},
    {0.10, 0.0, 0.0, 2.1, 0.2, 0.11, 0.10302380180000528},
    {0.20, 0.0, 0.0, 2.2, 0.4, 0.12, 0.10979124524044208},
  };
  value.control_stages = {
    {1.0, 0.10, 2.0, 0.10, 0.0, 4.0, -3.0, 1.37},
    {1.0, 0.10, 2.0, 0.10, 0.0, 4.0, -3.0, 1.37},
  };
  value.nominal_path_distance_m = {0.0, 0.2, 0.4};
  value.lateral_lower_m = {-1.0, -1.0, -1.0};
  value.lateral_upper_m = {1.0, 1.0, 1.0};
  if (intent == contract::ControlIntent::Return) {
    value.terminal_intent_contract =
      artifact::TerminalIntentContract{true, 0.20, 0.01, 0.0, 0.01};
    value.terminal_intent_certificate =
      artifact::TerminalIntentCertificate{true, 2U, 0.20, 0.0};
  }
  return value;
}

std::shared_ptr<recovery::OccupancyGrid> free_grid()
{
  auto grid = std::make_shared<recovery::OccupancyGrid>();
  grid->width = 400U;
  grid->height = 400U;
  grid->resolution_m = 0.05;
  grid->origin_x_m = 45.0;
  grid->origin_y_m = -5.0;
  grid->cells.assign(grid->width * grid->height, recovery::CellState::Free);
  return grid;
}

physical::Snapshot source_snapshot(
  const artifact::Identity & identity,
  std::shared_ptr<recovery::OccupancyGrid> grid = free_grid())
{
  physical::Snapshot snapshot;
  snapshot.identity.artifact = identity;
  snapshot.identity.pose_snapshot_id = 101U;
  snapshot.identity.course_frame_window_id = 102U;
  snapshot.identity.captured_sec = 1.0;
  snapshot.wall_grid = std::move(grid);
  snapshot.wall_grid_fingerprint =
    recovery::occupancy_grid_fingerprint(*snapshot.wall_grid);
  snapshot.footprint = {0.05, 0.05, 0.05, 0.05, 0.0};
  snapshot.current_pose = {50.0, 0.0, 0.0};
  snapshot.control_prefix = {snapshot.current_pose};
  snapshot.trajectory.progress_origin_m = 50.0;
  snapshot.trajectory.elapsed_time_sec = {0.1, 0.2};
  snapshot.trajectory.path_distance_m = {0.2, 0.4};
  snapshot.trajectory.lateral_m = {0.10, 0.20};
  snapshot.trajectory.lag_m = {0.0, 0.0};
  snapshot.trajectory.heading_offset_rad = {0.0, 0.0};
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
  snapshot.bound_tolerance_m = 1e-6;
  snapshot.swept_step_m = 0.02;
  return snapshot;
}

physical::Result accepted_result(const physical::Snapshot & snapshot)
{
  physical::Result result;
  result.identity = snapshot.identity;
  result.outcome = physical::Outcome::Accepted;
  result.diagnostic.reason =
    contract::PhysicalWallCertificateReason::Accepted;
  result.completed_sec = 1.01;
  result.compute_ms = 10.0;
  result.detail = "accepted";
  return result;
}

std::shared_ptr<const certified::CertifiedPlan> certified_plan(
  std::shared_ptr<recovery::OccupancyGrid> grid = free_grid(),
  const contract::ControlIntent intent = contract::ControlIntent::Track)
{
  auto execution = std::make_shared<const artifact::ExecutionArtifact>(
    execution_artifact(intent));
  const auto snapshot = source_snapshot(execution->identity, std::move(grid));
  const auto built = certified::build(
    execution, snapshot, accepted_result(snapshot));
  EXPECT_EQ(built.reason, certified::RejectReason::None);
  return built.plan;
}

std::shared_ptr<const certified::CertifiedPlan>
certified_plan_with_physical_lag(const double lag_m)
{
  auto execution = std::make_shared<const artifact::ExecutionArtifact>(
    execution_artifact());
  auto snapshot = source_snapshot(execution->identity);
  snapshot.trajectory.lag_m.assign(
    snapshot.trajectory.lag_m.size(), lag_m);
  const auto built = certified::build(
    execution, snapshot, accepted_result(snapshot));
  EXPECT_EQ(built.reason, certified::RejectReason::None);
  return built.plan;
}

retained::Request accepted_request(
  const std::shared_ptr<const certified::CertifiedPlan> & plan)
{
  retained::Request request;
  request.plan = plan;
  request.decision_id = 100U;
  request.now_sec = 1.05;
  request.control_origin_sec = 1.05;
  request.execution_clock = {
    retained::ExecutionClockKind::PublishedPlan, 1.0, 0.0};
  request.current_intent = contract::ControlIntent::Track;
  request.control_origin_physical_progress_m = 50.10;
  request.path_length_m = 100.0;
  request.progress_continuity_tolerance_m = 0.20;
  request.circular = true;
  request.control_pose = {50.05, 0.05, 0.0};
  request.measured_to_control_path = {request.control_pose};
  request.measured_to_control_elapsed_sec = {0.0};
  request.current_wall_grid = plan->physical_snapshot->wall_grid;
  request.current_footprint = plan->physical_snapshot->footprint;
  request.obstacles.generation = 7U;
  request.obstacles.observed_sec = 1.05;
  request.obstacles.current = true;
  request.current_speed_mps = 2.05;
  request.control_origin_speed_mps = 2.05;
  request.current_time_steering_rad = 0.105;
  request.current_steering_rad = 0.105;
  request.current_response_steering_rad = 0.105;
  request.previous_published_steering_rad = 0.105;
  request.previous_published_command_age_sec = 0.025;
  request.stop_lateral_policy = {
    2.0, 0.60, 1.0, 6.0, 1.5, 0.4, 1.3};
  request.minimum_acceleration_mps2 = -3.0;
  request.maximum_acceleration_mps2 = 1.0;
  return request;
}

retained::Request accepted_follow_request()
{
  const auto plan = certified_plan(
    free_grid(), contract::ControlIntent::Follow);
  EXPECT_NE(plan, nullptr);
  auto request = accepted_request(plan);
  request.current_intent = contract::ControlIntent::Follow;
  request.obstacles.obstacles.push_back(
    {"d2", {55.0, 0.0, 2.0, 0.0, 0.2}});
  request.follow_target = retained::FollowTargetObservation{
    "d2",
    request.obstacles.generation,
    request.obstacles.observed_sec,
    5.0,
    3.0,
    2.0,
    {0.0, 0.1, 0.2},
    {5.0, 5.2, 5.4},
    true};
  return request;
}

TEST(
  MpccRateResolvedRetainedRevalidation,
  BuildsFreshFollowEvidenceWithoutAProposedFollowIntent)
{
  const auto observation = retained::build_follow_target_observation(
    retained::FollowTargetObservationBuildRequest{
      "d2", 9U, 2.0, 5.0, 0.2, 3.0, 2.0, {0.1, 0.2}, true});

  ASSERT_TRUE(observation.has_value());
  EXPECT_EQ(observation->target_id, "d2");
  EXPECT_EQ(observation->observation_generation, 9U);
  ASSERT_EQ(observation->elapsed_time_sec.size(), 3U);
  EXPECT_NEAR(observation->elapsed_time_sec[0], 0.0, 1e-9);
  EXPECT_NEAR(observation->elapsed_time_sec[1], 0.1, 1e-9);
  EXPECT_NEAR(observation->elapsed_time_sec[2], 0.3, 1e-9);
  ASSERT_EQ(observation->target_progress_from_current_origin_m.size(), 3U);
  EXPECT_NEAR(observation->target_progress_from_current_origin_m[0], 5.2, 1e-9);
  EXPECT_NEAR(observation->target_progress_from_current_origin_m[1], 5.4, 1e-9);
  EXPECT_NEAR(observation->target_progress_from_current_origin_m[2], 5.8, 1e-9);
}

TEST(
  MpccRateResolvedRetainedRevalidation,
  ConvertsCombinedCenterExclusionToPeerOnlyCircleRadius)
{
  const recovery::FootprintExtents ego{
    1.49, 0.51, 0.725, 0.725, 0.05};

  const auto radius = retained::resolve_peer_circle_radius(
    1.45, ego, 0.05);

  ASSERT_TRUE(radius.has_value());
  EXPECT_NEAR(radius.value(), 0.775, 1e-12);
  const auto clearance = recovery::circle_obstacle_clearance_at_time(
    ego, recovery::Pose2D{0.0, 0.0, 0.0},
    recovery::CircleObstacle{0.0, 1.55, 0.0, 0.0, radius.value()}, 0.0);
  ASSERT_TRUE(clearance.has_value());
  EXPECT_NEAR(clearance.value(), 0.0, 1e-12);
}

TEST(
  MpccRateResolvedRetainedRevalidation,
  RejectsCenterExclusionSmallerThanEgoBodyExtent)
{
  const recovery::FootprintExtents ego{
    1.49, 0.51, 0.725, 0.725, 0.05};

  EXPECT_FALSE(retained::resolve_peer_circle_radius(0.70, ego, 0.05).has_value());
}

TEST(
  MpccRateResolvedRetainedRevalidation,
  RejectsMalformedFreshFollowEvidence)
{
  auto request = retained::FollowTargetObservationBuildRequest{
    "d2", 9U, 2.0, 5.0, 0.2, 3.0, 2.0, {0.1, 0.2}, true};
  request.target_id.clear();
  EXPECT_FALSE(retained::build_follow_target_observation(request).has_value());
  request.target_id = "d2";
  request.stage_duration_sec = {0.1, 0.0};
  EXPECT_FALSE(retained::build_follow_target_observation(request).has_value());
}

TEST(MpccRateResolvedRetainedRevalidation, AcceptsCurrentWorldJoin)
{
  const auto plan = certified_plan();
  ASSERT_NE(plan, nullptr);
  const auto result = retained::evaluate(accepted_request(plan));
  ASSERT_EQ(result.reason, retained::Reason::Accepted);
  ASSERT_TRUE(result.proof.has_value());
  EXPECT_EQ(
    result.proof->static_wall_scope,
    retained::StaticWallProofScope::FullSuffix);
  EXPECT_EQ(result.proof->proved_control_stage_count, 2U);
  EXPECT_EQ(result.proof->cursor.control_stage_index, 0U);
  EXPECT_NEAR(result.cursor_elapsed_sec, 0.05, 1e-9);
  EXPECT_NEAR(result.proof->expected_absolute_progress_m, 50.10, 1e-9);
  EXPECT_NEAR(result.proof->expected_physical_progress_m, 50.15, 1e-9);
  EXPECT_EQ(result.proof->obstacle_generation, 7U);
}

TEST(
  MpccRateResolvedRetainedRevalidation,
  RejectsObstacleOnCurrentStateContinuationInsteadOfOldCertifiedSuffix)
{
  const auto plan = certified_plan();
  ASSERT_NE(plan, nullptr);
  auto request = accepted_request(plan);
  // The retained artifact was certified around y=+0.1..+0.2.  The current
  // control-origin state is now on the other side of the reference path.  A
  // wall-clear chord back to the old state is not evidence that replaying the
  // old control suffix from this state remains dynamically clear.
  request.control_pose = {50.05, -0.60, 0.0};
  request.measured_to_control_path = {request.control_pose};
  request.obstacles.obstacles.push_back(
    // Intersect the exact current control stage.  A later-stage-only
    // intersection is intentionally handled by the receding prefix contract
    // below and therefore is not the invariant under test here.
    {"d2", {50.15, -0.60, 0.0, 0.0, 0.08}});

  const auto result = retained::evaluate(request);

  EXPECT_EQ(result.reason, retained::Reason::DynamicPathBlocked);
  EXPECT_EQ(result.blocking_obstacle_id, "d2");
}

TEST(MpccRateResolvedRetainedRevalidation, AcceptsEveryArtifactOwnedIntent)
{
  const std::vector<contract::ControlIntent> intents{
    contract::ControlIntent::Track,
    contract::ControlIntent::Cruise,
    contract::ControlIntent::ShiftOut,
    contract::ControlIntent::Pass,
    contract::ControlIntent::Return,
    contract::ControlIntent::Rejoin,
  };
  for (const auto intent : intents) {
    SCOPED_TRACE(contract::to_string(intent));
    const auto plan = certified_plan(free_grid(), intent);
    ASSERT_NE(plan, nullptr);
    auto request = accepted_request(plan);
    request.current_intent = intent;
    EXPECT_EQ(retained::evaluate(request).reason, retained::Reason::Accepted);
  }
}

TEST(
  MpccRateResolvedRetainedRevalidation,
  AcceptsFollowOnlyWithCurrentTargetHardGapProof)
{
  const auto result = retained::evaluate(accepted_follow_request());
  ASSERT_EQ(result.reason, retained::Reason::Accepted);
  ASSERT_TRUE(result.proof.has_value());
  EXPECT_EQ(result.proof->follow_target_observation_generation, 7U);
  EXPECT_GT(result.proof->follow_checked_state_count, 0U);
  EXPECT_GE(result.proof->follow_minimum_gap_m, 3.0);
}

TEST(
  MpccRateResolvedRetainedRevalidation,
  ResolvesEveryNormalIntentThroughOneRequestScope)
{
  using Intent = contract::ControlIntent;

  EXPECT_TRUE(artifact::request_scope_available(Intent::Track, true, false, false, false));
  EXPECT_TRUE(artifact::request_scope_available(Intent::Cruise, true, false, false, false));
  EXPECT_TRUE(artifact::request_scope_available(Intent::Follow, false, true, false, false));
  EXPECT_TRUE(artifact::request_scope_available(Intent::ShiftOut, false, false, true, false));
  EXPECT_TRUE(artifact::request_scope_available(Intent::Pass, false, false, true, false));
  EXPECT_TRUE(artifact::request_scope_available(Intent::Return, false, false, true, false));
  EXPECT_TRUE(artifact::request_scope_available(Intent::Rejoin, false, false, false, true));

  EXPECT_FALSE(artifact::request_scope_available(Intent::Follow, true, false, true, true));
  EXPECT_FALSE(artifact::request_scope_available(Intent::Cruise, false, true, true, true));
  EXPECT_FALSE(artifact::request_scope_available(Intent::Rejoin, true, true, true, false));
  EXPECT_FALSE(artifact::request_scope_available(Intent::Unknown, true, true, true, true));
}

TEST(
  MpccRateResolvedRetainedRevalidation,
  RejectsFollowWithoutCurrentTargetProof)
{
  auto request = accepted_follow_request();
  request.follow_target.reset();
  EXPECT_EQ(
    retained::evaluate(request).reason,
    retained::Reason::FollowTargetObservationUnavailable);
}

TEST(
  MpccRateResolvedRetainedRevalidation,
  RejectsFollowTargetGenerationOutsideCurrentWorldSnapshot)
{
  auto request = accepted_follow_request();
  request.follow_target->observation_generation += 1U;
  EXPECT_EQ(
    retained::evaluate(request).reason,
    retained::Reason::FollowTargetIdentityMismatch);
}

TEST(
  MpccRateResolvedRetainedRevalidation,
  RejectsFollowCurrentHardGapViolation)
{
  auto request = accepted_follow_request();
  request.follow_target->current_target_gap_m = 2.9;
  EXPECT_EQ(
    retained::evaluate(request).reason,
    retained::Reason::FollowInitialHardGapViolation);
}

TEST(
  MpccRateResolvedRetainedRevalidation,
  RejectsFollowRetainedStageHardGapViolation)
{
  auto request = accepted_follow_request();
  request.follow_target->current_target_gap_m = 3.05;
  request.follow_target->target_speed_mps = 0.0;
  request.follow_target->target_progress_from_current_origin_m = {
    3.05, 3.05, 3.05};
  EXPECT_EQ(
    retained::evaluate(request).reason,
    retained::Reason::FollowStageGapViolation);
}

TEST(
  MpccRateResolvedRetainedRevalidation,
  UsesObservationToControlDurationForVelocityReachability)
{
  const auto plan = certified_plan();
  auto request = accepted_request(plan);
  request.now_sec = 1.0;
  request.control_origin_sec = 1.05;
  request.current_speed_mps = 2.0;
  request.measured_to_control_path = {
    {50.0, 0.0, 0.0}, request.control_pose};
  request.measured_to_control_elapsed_sec = {0.0, 0.05};
  request.obstacles.observed_sec = request.now_sec;

  const auto result = retained::evaluate(request);

  ASSERT_EQ(result.reason, retained::Reason::Accepted);
  ASSERT_TRUE(result.proof.has_value());
  EXPECT_NEAR(result.proof->velocity_difference_mps, 0.05, 1e-9);
  EXPECT_NEAR(result.proof->reachable_velocity_lower_mps, 1.849999, 1e-9);
  EXPECT_NEAR(result.proof->reachable_velocity_upper_mps, 2.050001, 1e-9);
  EXPECT_NEAR(result.proof->velocity_reachability_duration_sec, 0.05, 1e-9);
}

TEST(MpccRateResolvedRetainedRevalidation, AcceptsClearDynamicObstacle)
{
  const auto plan = certified_plan();
  auto request = accepted_request(plan);
  request.obstacles.obstacles.push_back(
    {"peer", {50.0, 3.0, 0.0, 0.0, 0.2}});
  const auto result = retained::evaluate(request);
  ASSERT_EQ(result.reason, retained::Reason::Accepted);
  ASSERT_TRUE(result.proof.has_value());
  EXPECT_GT(result.proof->dynamic_checked_pose_count, 0U);
  EXPECT_GT(result.proof->minimum_dynamic_clearance_m, 0.0);
}

TEST(MpccRateResolvedRetainedRevalidation, RejectsIntersectingDynamicObstacle)
{
  const auto plan = certified_plan();
  auto request = accepted_request(plan);
  request.obstacles.obstacles.push_back(
    {"peer", {50.10, 0.10, 0.0, 0.0, 0.2}});
  const auto result = retained::evaluate(request);
  EXPECT_EQ(result.reason, retained::Reason::DynamicPathBlocked);
  EXPECT_EQ(result.blocking_obstacle_id, "peer");
  EXPECT_LT(result.minimum_dynamic_clearance_m, 0.0);
}

TEST(
  MpccRateResolvedRetainedRevalidation,
  AcceptsMonotonicEscapeFromInitialInflatedOverlap)
{
  const auto plan = certified_plan();
  auto request = accepted_request(plan);
  // Conservative circle inflation overlaps the current rear corner, but the
  // exact canonical continuation moves monotonically away from it.
  request.obstacles.obstacles.push_back(
    {"initial-overlap", {49.95, 0.05, 0.0, 0.0, 0.10}});

  const auto result = retained::evaluate(request);

  ASSERT_EQ(result.reason, retained::Reason::Accepted);
  ASSERT_TRUE(result.proof.has_value());
  EXPECT_LT(result.proof->minimum_dynamic_clearance_m, 0.0);
  EXPECT_GT(result.proof->dynamic_checked_pose_count, 1U);
}

TEST(
  MpccRateResolvedRetainedRevalidation,
  RejectsCanonicalContinuationThatWorsensInitialInflatedOverlap)
{
  const auto plan = certified_plan();
  auto request = accepted_request(plan);
  request.obstacles.obstacles.push_back(
    {"worsening-overlap", {50.15, 0.05, 0.0, 0.0, 0.10}});

  const auto result = retained::evaluate(request);

  EXPECT_EQ(result.reason, retained::Reason::DynamicPathBlocked);
  EXPECT_EQ(result.blocking_obstacle_id, "worsening-overlap");
}

TEST(
  MpccRateResolvedRetainedRevalidation,
  RequiresCertifiedStopWhenObstacleCrossesAfterPublisherInterval)
{
  const auto plan = certified_plan();
  auto request = accepted_request(plan);
  request.obstacles.obstacles.push_back(
    // The retained suffix has only 0.15 s left at now=1.05.  Start close
    // enough that the moving circle intersects it inside that exact horizon.
    {"crossing", {50.20, 0.30, 0.0, -1.0, 0.15}});
  EXPECT_EQ(
    retained::evaluate(request).reason,
    retained::Reason::TerminalContingencyUnavailable);
}

TEST(
  MpccRateResolvedRetainedRevalidation,
  RejectsClearPublisherIntervalWithoutCertifiedDynamicStopSuffix)
{
  const auto plan = certified_plan();
  auto request = accepted_request(plan);
  // The publisher interval is clear. The peer intersects the continuation
  // later. Receding-horizon authority must certify the exact clear interval
  // which can be published now and require a successor plan,
  // rather than converting a future replanning obligation into an immediate
  // Emergency stop.
  request.obstacles.obstacles.push_back(
    {"later-peer", {50.40, 0.10, 0.0, 0.0, 0.05}});

  const auto result = retained::evaluate(request);

  EXPECT_EQ(
    result.reason, retained::Reason::TerminalContingencyUnavailable);
  EXPECT_FALSE(result.proof.has_value());
  EXPECT_EQ(
    result.dynamic_obstacle_scope,
    retained::DynamicObstacleProofScope::PublisherIntervalPrefix);
  EXPECT_EQ(result.blocking_obstacle_id, "later-peer");
  EXPECT_LT(result.minimum_dynamic_clearance_m, 0.0);
}

TEST(
  MpccRateResolvedRetainedRevalidation,
  AcceptsClearPublisherIntervalWithCurrentWorldCertifiedDynamicStopSuffix)
{
  auto execution = std::make_shared<const artifact::ExecutionArtifact>(
    execution_artifact());
  auto snapshot = source_snapshot(execution->identity);
  snapshot.footprint = {0.001, 0.001, 0.001, 0.001, 0.0};
  const auto built = certified::build(
    execution, snapshot, accepted_result(snapshot));
  ASSERT_EQ(built.reason, certified::RejectReason::None);
  ASSERT_NE(built.plan, nullptr);
  auto request = accepted_request(built.plan);
  request.current_footprint = snapshot.footprint;
  // The fast crossing peer reaches the old second-stage endpoint, but has
  // already crossed the short maximum-braking path when ego arrives there.
  request.obstacles.obstacles.push_back(
    {"later-crossing", {51.1187, 0.0527, -5.0, 0.0, 0.001}});

  const auto result = retained::evaluate(request);

  ASSERT_EQ(result.reason, retained::Reason::Accepted);
  ASSERT_TRUE(result.proof.has_value());
  EXPECT_EQ(
    result.dynamic_obstacle_scope,
    retained::DynamicObstacleProofScope::PublisherIntervalPrefix);
  EXPECT_TRUE(result.terminal_stop_attempted);
  EXPECT_TRUE(result.terminal_stop_certified);
  EXPECT_TRUE(result.proof->terminal_stop_certified);
  EXPECT_TRUE(
    multi_purpose_mpc_ros::race_mpcc_foundation::
    exact_physical_execution_trajectory_complete(
      result.proof->terminal_stop_trajectory));
  EXPECT_EQ(
    result.proof->terminal_stop_actuation_samples.size(),
    result.proof->terminal_stop_trajectory.elapsed_time_sec.size());
  EXPECT_GT(
    result.proof->terminal_stop_publisher_interval_sample_count, 0U);
  EXPECT_LE(
    result.proof->terminal_stop_publisher_interval_sample_count,
    result.proof->terminal_stop_actuation_samples.size());
  EXPECT_GE(result.runtime.pre_continuation_ms, 0.0);
  EXPECT_GT(result.runtime.continuation_build_ms, 0.0);
  EXPECT_GE(result.runtime.continuation_proof_ms, 0.0);
  EXPECT_GE(result.runtime.continuation_delay_wall_ms, 0.0);
  EXPECT_GE(result.runtime.continuation_dynamic_ms, 0.0);
  EXPECT_GE(result.runtime.continuation_wall_ms, 0.0);
  EXPECT_GT(result.runtime.terminal_build_ms, 0.0);
  EXPECT_GT(result.runtime.terminal_dynamic_ms, 0.0);
  EXPECT_GT(result.runtime.terminal_wall_ms, 0.0);
}

TEST(
  MpccRateResolvedRetainedRevalidation,
  DoesNotReuseOldCertifiedStateSuffixAfterTheStateOriginMoves)
{
  auto execution = std::make_shared<const artifact::ExecutionArtifact>(
    execution_artifact());
  auto snapshot = source_snapshot(execution->identity);
  // The old nonlinear certificate reached y=0.8.  It proves the old solve,
  // but after the vehicle has moved it is no longer current state evidence.
  // The retained input suffix is replayed from the current y=0.05 origin, so
  // an obstacle which intersects only the old state suffix must not block it.
  snapshot.trajectory.lateral_m = {0.8, 0.8};
  snapshot.trajectory.minimum_lateral_bound_reserve_m = 0.2;
  const auto built = certified::build(
    execution, snapshot, accepted_result(snapshot));
  ASSERT_EQ(built.reason, certified::RejectReason::None);
  ASSERT_NE(built.plan, nullptr);

  auto request = accepted_request(built.plan);
  request.obstacles.obstacles.push_back(
    {"nonlinear-suffix", {50.20, 0.80, 0.0, 0.0, 0.10}});

  const auto result = retained::evaluate(request);
  EXPECT_EQ(result.reason, retained::Reason::Accepted);
  EXPECT_TRUE(result.blocking_obstacle_id.empty());
}

TEST(MpccRateResolvedRetainedRevalidation, RejectsObstacleCrossingDuringDelayPrefix)
{
  const auto plan = certified_plan();
  auto request = accepted_request(plan);
  request.now_sec = 1.0;
  request.control_origin_sec = 1.05;
  request.measured_to_control_path = {
    {49.95, 0.05, 0.0}, request.control_pose};
  request.measured_to_control_elapsed_sec = {0.0, 0.05};
  request.obstacles.observed_sec = request.now_sec;
  request.obstacles.obstacles.push_back(
    {"delay-crossing", {50.0, 0.30, 0.0, -10.0, 0.02}});
  const auto result = retained::evaluate(request);
  EXPECT_EQ(result.reason, retained::Reason::DynamicPathBlocked);
  EXPECT_EQ(result.blocking_obstacle_id, "delay-crossing");
}

TEST(MpccRateResolvedRetainedRevalidation, RejectsInconsistentControlTimePrefix)
{
  const auto plan = certified_plan();
  auto request = accepted_request(plan);
  request.control_origin_sec = request.now_sec + 0.05;
  request.measured_to_control_elapsed_sec = {0.0};
  EXPECT_EQ(
    retained::evaluate(request).reason,
    retained::Reason::InvalidCurrentState);
}

TEST(MpccRateResolvedRetainedRevalidation, AcceptsMultipleClearPeers)
{
  const auto plan = certified_plan();
  auto request = accepted_request(plan);
  request.obstacles.obstacles = {
    {"behind", {45.0, 0.0, -1.0, 0.0, 0.2}},
    {"outside", {51.0, 3.0, 0.0, 0.0, 0.2}},
  };
  EXPECT_EQ(retained::evaluate(request).reason, retained::Reason::Accepted);
}

TEST(MpccRateResolvedRetainedRevalidation, RejectsDuplicateObstacleIdentity)
{
  const auto plan = certified_plan();
  auto request = accepted_request(plan);
  request.obstacles.obstacles = {
    {"peer", {45.0, 3.0, 0.0, 0.0, 0.2}},
    {"peer", {46.0, 3.0, 0.0, 0.0, 0.2}},
  };
  EXPECT_EQ(
    retained::evaluate(request).reason,
    retained::Reason::DynamicObservationInvalid);
}

TEST(MpccRateResolvedRetainedRevalidation, RejectsInvalidObstacleMotion)
{
  const auto plan = certified_plan();
  auto request = accepted_request(plan);
  request.obstacles.obstacles.push_back(
    {"peer", {45.0, 3.0, std::numeric_limits<double>::quiet_NaN(), 0.0, 0.2}});
  EXPECT_EQ(
    retained::evaluate(request).reason,
    retained::Reason::DynamicObservationInvalid);
}

TEST(MpccRateResolvedRetainedRevalidation, RejectsUnobservedDynamicWorld)
{
  const auto plan = certified_plan();
  auto request = accepted_request(plan);
  request.obstacles.current = false;
  EXPECT_EQ(
    retained::evaluate(request).reason,
    retained::Reason::DynamicObservationUnavailable);
}

TEST(MpccRateResolvedRetainedRevalidation, AcceptsIdenticalStaticWorldDeepCopy)
{
  const auto plan = certified_plan();
  auto request = accepted_request(plan);
  request.current_wall_grid = std::make_shared<recovery::OccupancyGrid>(
    *plan->physical_snapshot->wall_grid);
  EXPECT_EQ(
    retained::evaluate(request).reason,
    retained::Reason::Accepted);
}

TEST(MpccRateResolvedRetainedRevalidation, RejectsChangedStaticWorldDeepCopy)
{
  const auto plan = certified_plan();
  auto request = accepted_request(plan);
  auto changed_grid = std::make_shared<recovery::OccupancyGrid>(
    *plan->physical_snapshot->wall_grid);
  changed_grid->cells.front() = recovery::CellState::Occupied;
  request.current_wall_grid = std::move(changed_grid);
  EXPECT_EQ(
    retained::evaluate(request).reason,
    retained::Reason::StaticWorldMismatch);
}

TEST(MpccRateResolvedRetainedRevalidation, CertifiesCurrentWorldProgressRebase)
{
  const auto plan = certified_plan();
  auto request = accepted_request(plan);
  request.control_origin_physical_progress_m = 60.0;
  const auto result = retained::evaluate(request);
  EXPECT_EQ(result.reason, retained::Reason::Accepted);
  EXPECT_NEAR(result.expected_absolute_progress_m, 50.10, 1e-9);
  EXPECT_NEAR(result.expected_physical_progress_m, 50.15, 1e-9);
  EXPECT_NEAR(
    result.lifted_control_origin_physical_progress_m, 60.0, 1e-9);
  EXPECT_NEAR(result.progress_difference_m, 9.85, 1e-9);
  EXPECT_NEAR(result.progress_continuity_tolerance_m, 0.20, 1e-9);
  EXPECT_NEAR(result.current_speed_mps, 2.05, 1e-9);
  EXPECT_NEAR(result.current_steering_rad, 0.105, 1e-9);
  EXPECT_TRUE(result.progress_rebased);
  ASSERT_TRUE(result.proof.has_value());
  EXPECT_TRUE(result.proof->progress_rebased);
  EXPECT_TRUE(result.proof->stateless_current_world_bundle());
}

TEST(
  MpccRateResolvedRetainedRevalidation,
  ComparesPhysicalProgressInsteadOfDiscreteCourseFrameProgress)
{
  const auto plan = certified_plan_with_physical_lag(0.80);
  ASSERT_NE(plan, nullptr);
  auto request = accepted_request(plan);
  request.now_sec = 1.10;
  request.control_origin_sec = 1.10;
  request.obstacles.observed_sec = request.now_sec;
  request.control_origin_physical_progress_m = 51.0;
  request.control_pose = {51.0, 0.10, 0.0};
  request.measured_to_control_path = {request.control_pose};
  request.current_speed_mps = 2.10;
  request.control_origin_speed_mps = 2.10;
  request.current_time_steering_rad = 0.11;
  request.current_steering_rad = 0.11;
  request.current_response_steering_rad = 0.103;
  request.previous_published_steering_rad = 0.11;

  const auto result = retained::evaluate(request);
  EXPECT_EQ(result.reason, retained::Reason::Accepted);
  ASSERT_TRUE(result.proof.has_value());
  EXPECT_FALSE(result.progress_rebased);
  EXPECT_FALSE(result.proof->progress_rebased);
  EXPECT_NEAR(result.expected_absolute_progress_m, 50.20, 1e-9);
  EXPECT_NEAR(result.expected_physical_progress_m, 51.0, 1e-9);
  EXPECT_NEAR(result.progress_difference_m, 0.0, 1e-9);
}

TEST(
  MpccRateResolvedRetainedRevalidation,
  BuildsCurrentWorldBundleForProvedUnreachablePreparedSteering)
{
  const auto plan = certified_plan();
  auto request = accepted_request(plan);
  request.current_time_steering_rad = -0.10;
  request.previous_published_steering_rad = -0.10;
  request.previous_published_command_age_sec = 0.0;
  const auto result = retained::evaluate(request);
  EXPECT_EQ(result.reason, retained::Reason::Accepted);
  EXPECT_NEAR(result.current_steering_rad, 0.105, 1e-9);
  EXPECT_NEAR(result.current_time_steering_rad, -0.10, 1e-9);
  EXPECT_NEAR(result.previous_published_steering_rad, -0.10, 1e-9);
  // At zero publication age the certified command sample must agree with the
  // last serialized command within solver tolerance.
  EXPECT_NEAR(result.expected_steering_rad, 0.105, 1e-9);
  EXPECT_NEAR(result.steering_difference_rad, 0.205, 1e-9);
  EXPECT_NEAR(result.maximum_steering_step_rad, 0.000001, 1e-9);
  EXPECT_NEAR(result.reachable_steering_lower_rad, -0.100001, 1e-9);
  EXPECT_NEAR(result.reachable_steering_upper_rad, -0.099999, 1e-9);
  EXPECT_NEAR(result.steering_reachability_duration_sec, 0.0, 1e-9);
  EXPECT_TRUE(result.feedback_shadow_attempted);
  EXPECT_EQ(
    result.feedback_shadow_reason,
    multi_purpose_mpc_ros::mpcc_latest_state_feedback::Reason::
    ProjectedToReachableEnvelope);
  EXPECT_NEAR(result.feedback_shadow_steering_rad, -0.099999, 1e-9);
  EXPECT_NEAR(result.feedback_shadow_correction_rad, -0.204999, 1e-9);
  EXPECT_EQ(
    result.feedback_shadow_continuation_reason,
    multi_purpose_mpc_ros::mpcc_rate_resolved_physical_adapter::
    ContinuationRejectReason::None);
  EXPECT_TRUE(result.feedback_shadow_continuation_available);
  EXPECT_EQ(
    result.feedback_shadow_proof_reason, retained::Reason::Accepted);
  EXPECT_TRUE(result.feedback_shadow_proof_available);
  // A complete latest-state connection is a stateless current-world bundle,
  // not an excuse to retain the source artifact's unreachable first command.
  // The common production adapter must receive that exact proved bundle.
  ASSERT_TRUE(result.proof.has_value());
  EXPECT_TRUE(result.proof->latest_state_feedback_bundle);
  EXPECT_EQ(result.reason, retained::Reason::Accepted);
}

TEST(
  MpccRateResolvedRetainedRevalidation,
  BootstrapCandidateStartsAtZeroWithoutExecutedPrefix)
{
  const auto plan = certified_plan();
  auto request = accepted_request(plan);
  request.previous_published_steering_rad = 0.079;
  request.previous_published_command_age_sec = 0.025;
  request.execution_clock = {
    retained::ExecutionClockKind::BootstrapCandidate,
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()};

  const auto bootstrap = retained::evaluate(request);

  ASSERT_EQ(bootstrap.reason, retained::Reason::Accepted);
  ASSERT_TRUE(bootstrap.proof.has_value());
  EXPECT_NEAR(bootstrap.cursor_elapsed_sec, 0.0, 1e-9);
  EXPECT_EQ(bootstrap.proof->cursor.control_stage_index, 0U);
  EXPECT_NEAR(bootstrap.proof->cursor.stage_elapsed_sec, 0.0, 1e-9);
}

TEST(
  MpccRateResolvedRetainedRevalidation,
  TimeAlignedCandidateSplicesAtCurrentControlOriginWithoutPrefixAuthority)
{
  const auto plan = certified_plan();
  auto request = accepted_request(plan);
  request.previous_published_steering_rad = 0.079;
  request.previous_published_command_age_sec = 0.025;

  // With a published origin at 1.0, the source cursor has executed 50 ms and
  // asks for steering 0.105.  The source sample is not directly reachable,
  // but its exact current-world feedback bundle is.
  request.execution_clock = {
    retained::ExecutionClockKind::PublishedPlan, 1.0, 0.0};
  const auto fictitiously_aged = retained::evaluate(request);
  EXPECT_EQ(fictitiously_aged.reason, retained::Reason::Accepted);
  ASSERT_TRUE(fictitiously_aged.proof.has_value());
  EXPECT_TRUE(fictitiously_aged.proof->latest_state_feedback_bundle);
  EXPECT_NEAR(fictitiously_aged.cursor_elapsed_sec, 0.05, 1e-9);

  // A certified candidate which never crossed the publisher receives no
  // authority for its skipped prefix.  It is nevertheless joined at the
  // time-aligned suffix, where the same current-world proof owns the
  // reachable feedback command without claiming the skipped prefix.
  request.execution_clock = {
    retained::ExecutionClockKind::TimeAlignedCandidate,
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()};
  const auto candidate = retained::evaluate(request);
  EXPECT_EQ(candidate.reason, retained::Reason::Accepted);
  ASSERT_TRUE(candidate.proof.has_value());
  EXPECT_TRUE(candidate.proof->latest_state_feedback_bundle);
  EXPECT_NEAR(candidate.cursor_elapsed_sec, 0.05, 1e-9);
}

TEST(
  MpccRateResolvedRetainedRevalidation,
  MovingTimeAlignedCandidateUsesReachableSuffixCrossSection)
{
  auto execution = execution_artifact();
  // State zero is a narrow physical cross-section at the old prediction
  // origin.  The later suffix is deliberately broad.  A cursor-zero join from
  // y=0.05 would be rejected even though the candidate has a currently
  // reachable, fully revalidated suffix.
  execution.lateral_lower_m = {-0.001, -1.0, -1.0};
  execution.lateral_upper_m = {0.001, 1.0, 1.0};
  auto execution_owner =
    std::make_shared<const artifact::ExecutionArtifact>(execution);
  const auto snapshot = source_snapshot(execution_owner->identity);
  const auto built = certified::build(
    execution_owner, snapshot, accepted_result(snapshot));
  ASSERT_EQ(built.reason, certified::RejectReason::None);
  ASSERT_NE(built.plan, nullptr);

  auto request = accepted_request(built.plan);
  request.execution_clock = {
    retained::ExecutionClockKind::TimeAlignedCandidate,
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()};

  const auto result = retained::evaluate(request);

  ASSERT_EQ(result.reason, retained::Reason::Accepted);
  ASSERT_TRUE(result.proof.has_value());
  EXPECT_NEAR(result.cursor_elapsed_sec, 0.05, 1e-9);
  EXPECT_EQ(result.proof->cursor.control_stage_index, 0U);
  EXPECT_NEAR(result.proof->cursor.stage_elapsed_sec, 0.05, 1e-9);
}

TEST(
  MpccRateResolvedRetainedRevalidation,
  PublishedPlanContinuesFromTheCandidateSuffixThatActuallyCrossedThePublisher)
{
  const auto plan = certified_plan();
  ASSERT_NE(plan, nullptr);

  const auto cursor = retained::resolve_execution_cursor(
    *plan->execution_artifact, 1.115,
    retained::ExecutionClock{
      retained::ExecutionClockKind::PublishedPlan,
      1.10,
      0.05});

  ASSERT_TRUE(cursor.available);
  EXPECT_NEAR(cursor.elapsed_sec, 0.065, 1e-9);
  EXPECT_EQ(cursor.control_stage_index, 0U);
  EXPECT_NEAR(cursor.stage_elapsed_sec, 0.065, 1e-9);
}

TEST(
  MpccRateResolvedRetainedRevalidation,
  AdvancesToPublisherCompleteStageAndReprovesCurrentWorldBundle)
{
  const auto plan = certified_plan();
  ASSERT_NE(plan, nullptr);
  auto request = accepted_request(plan);
  request.now_sec = 1.085;
  request.control_origin_sec = 1.085;
  request.obstacles.observed_sec = request.now_sec;
  request.control_origin_physical_progress_m = 50.17;
  request.progress_continuity_tolerance_m = 0.50;
  request.control_pose = {50.17, 0.085, 0.0};
  request.measured_to_control_path = {request.control_pose};
  request.current_speed_mps = 2.085;
  request.control_origin_speed_mps = 2.085;
  request.current_time_steering_rad = 0.1085;
  request.current_steering_rad = 0.1085;
  request.current_response_steering_rad = 0.1025;
  request.previous_published_steering_rad = 0.11;

  const auto result = retained::evaluate(request);

  ASSERT_EQ(result.reason, retained::Reason::Accepted);
  ASSERT_TRUE(result.proof.has_value());
  EXPECT_TRUE(result.publication_stage_advanced);
  EXPECT_EQ(result.source_control_stage_index, 0U);
  EXPECT_EQ(result.command_control_stage_index, 1U);
  EXPECT_NEAR(result.publication_stage_advance_sec, 0.015, 1e-9);
  EXPECT_EQ(result.proof->cursor.control_stage_index, 1U);
  EXPECT_NEAR(result.proof->cursor.stage_elapsed_sec, 0.0, 1e-9);
  EXPECT_TRUE(result.proof->publication_stage_advanced);
  EXPECT_TRUE(result.proof->stateless_current_world_bundle());
  EXPECT_FALSE(result.proof->latest_state_feedback_bundle);
  EXPECT_GE(result.runtime.pre_continuation_ms, 0.0);
  EXPECT_GT(result.runtime.continuation_build_ms, 0.0);
  EXPECT_GE(result.runtime.continuation_proof_ms, 0.0);
  EXPECT_GE(result.runtime.continuation_delay_wall_ms, 0.0);
  EXPECT_GE(result.runtime.continuation_dynamic_ms, 0.0);
  EXPECT_GE(result.runtime.continuation_wall_ms, 0.0);
}

TEST(
  MpccRateResolvedRetainedRevalidation,
  KeepsFinalArtifactExhaustionFailClosed)
{
  const auto plan = certified_plan();
  ASSERT_NE(plan, nullptr);
  auto request = accepted_request(plan);
  request.now_sec = 1.195;
  request.control_origin_sec = 1.195;
  request.obstacles.observed_sec = request.now_sec;
  request.control_origin_physical_progress_m = 50.39;
  request.progress_continuity_tolerance_m = 0.50;
  request.control_pose = {50.39, 0.195, 0.0};
  request.measured_to_control_path = {request.control_pose};
  request.current_speed_mps = 2.195;
  request.control_origin_speed_mps = 2.195;
  request.current_time_steering_rad = 0.1195;
  request.current_steering_rad = 0.1195;
  request.current_response_steering_rad = 0.109;
  request.previous_published_steering_rad = 0.1195;

  const auto result = retained::evaluate(request);

  EXPECT_EQ(result.reason, retained::Reason::ContinuationRejected);
  EXPECT_EQ(
    result.continuation_reason,
    multi_purpose_mpc_ros::mpcc_rate_resolved_physical_adapter::
    ContinuationRejectReason::InvalidCursor);
  EXPECT_FALSE(result.proof.has_value());
}

TEST(
  MpccRateResolvedRetainedRevalidation,
  RejectsMissingExecutionClockOwnership)
{
  const auto plan = certified_plan();
  auto request = accepted_request(plan);
  request.execution_clock = {};
  EXPECT_EQ(
    retained::evaluate(request).reason,
    retained::Reason::ExecutionClockInvalid);
}

TEST(
  MpccRateResolvedRetainedRevalidation,
  RejectsPublishedExecutionClockWithoutCausalOrigin)
{
  const auto plan = certified_plan();
  auto request = accepted_request(plan);
  request.execution_clock = {
    retained::ExecutionClockKind::PublishedPlan,
    std::numeric_limits<double>::quiet_NaN(), 0.0};
  EXPECT_EQ(
    retained::evaluate(request).reason,
    retained::Reason::ExecutionClockInvalid);
}

TEST(
  MpccRateResolvedRetainedRevalidation,
  UsesPublishedCommandStateForSteeringReachability)
{
  const auto plan = certified_plan();
  auto request = accepted_request(plan);
  request.control_origin_sec = request.now_sec + 0.05;
  request.measured_to_control_path = {
    request.control_pose, request.control_pose};
  request.measured_to_control_elapsed_sec = {0.0, 0.05};
  request.current_time_steering_rad = 0.08;
  request.current_steering_rad = 0.105;
  request.previous_published_steering_rad = 0.08;
  request.previous_published_command_age_sec = 0.040;

  // The measured physical state may lag far behind.  Command authority is
  // nevertheless valid because the next serialized sample is reachable from
  // the last serialized command over the actual publication age.
  const auto result = retained::evaluate(request);
  EXPECT_EQ(result.reason, retained::Reason::Accepted);
  ASSERT_TRUE(result.proof.has_value());
  EXPECT_NEAR(result.proof->previous_published_steering_rad, 0.08, 1e-9);
  EXPECT_NEAR(result.proof->steering_difference_rad, 0.030, 1e-9);
  EXPECT_NEAR(result.proof->steering_reachability_duration_sec, 0.040, 1e-9);
}

TEST(
  MpccRateResolvedRetainedRevalidation,
  PreviousCommandAgeOwnsCommandReachability)
{
  const auto plan = certified_plan();
  auto request = accepted_request(plan);
  request.previous_published_steering_rad = 0.075;

  // The command age is the causal time available for the serialized command
  // to move from the predecessor to the retained artifact sample.
  request.previous_published_command_age_sec = 0.040;
  const auto accepted = retained::evaluate(request);
  EXPECT_EQ(accepted.reason, retained::Reason::Accepted);
  ASSERT_TRUE(accepted.proof.has_value());
  EXPECT_NEAR(
    accepted.proof->steering_reachability_duration_sec, 0.040, 1e-9);

  request.previous_published_command_age_sec = 0.010;
  const auto connected = retained::evaluate(request);
  EXPECT_EQ(connected.reason, retained::Reason::Accepted);
  ASSERT_TRUE(connected.proof.has_value());
  EXPECT_TRUE(connected.proof->latest_state_feedback_bundle);
}

TEST(
  MpccRateResolvedRetainedRevalidation,
  RejectsContradictoryVelocityAtIdenticalTimeOrigin)
{
  const auto plan = certified_plan();
  auto request = accepted_request(plan);
  request.current_speed_mps = 4.0;
  const auto result = retained::evaluate(request);
  EXPECT_EQ(result.reason, retained::Reason::InvalidCurrentState);
  EXPECT_FALSE(result.proof.has_value());
}

TEST(
  MpccRateResolvedRetainedRevalidation,
  ReanchorsRetainedVelocityStateToCurrentControlOrigin)
{
  const auto plan = certified_plan();
  auto request = accepted_request(plan);
  request.now_sec = 1.02;
  request.control_origin_sec = 1.15;
  request.obstacles.observed_sec = request.now_sec;
  request.measured_to_control_path = {
    request.control_pose, request.control_pose};
  request.measured_to_control_elapsed_sec = {0.0, 0.13};
  request.control_origin_physical_progress_m = 50.20;
  request.current_speed_mps = 1.8;
  request.control_origin_speed_mps = 1.7;

  // The retained artifact's old velocity state is not an actuator command.
  // The nonlinear continuation is rebuilt from the fresh control-origin
  // velocity, so a model mismatch must not close otherwise valid authority.
  const auto result = retained::evaluate(request);
  EXPECT_EQ(result.reason, retained::Reason::Accepted);
  ASSERT_TRUE(result.proof.has_value());
  EXPECT_DOUBLE_EQ(
    result.proof->actuation.predicted_speed_mps,
    request.control_origin_speed_mps);
  ASSERT_FALSE(result.proof->continuation_trajectory.velocity_mps.empty());
  EXPECT_LT(
    result.proof->continuation_trajectory.velocity_mps.front(),
    result.expected_speed_mps);
}

TEST(
  MpccRateResolvedRetainedRevalidation,
  DoesNotTreatUnusedChordToOldStateAsAnExecutionPath)
{
  auto grid = free_grid();
  const auto plan = certified_plan(grid);
  ASSERT_NE(plan, nullptr);
  auto request = accepted_request(plan);
  request.control_pose = {50.05, 0.50, 0.0};
  request.measured_to_control_path = {request.control_pose};
  const auto occupied = grid->world_to_grid(50.05, 0.25);
  ASSERT_TRUE(occupied.has_value());
  grid->cells[occupied->row * grid->width + occupied->column] =
    recovery::CellState::Occupied;
  EXPECT_EQ(retained::evaluate(request).reason, retained::Reason::Accepted);
}

TEST(
  MpccRateResolvedRetainedRevalidation,
  RequiresCertifiedStopWhenWallBlocksAfterPublisherInterval)
{
  auto grid = free_grid();
  const auto occupied = grid->world_to_grid(50.20, 0.05);
  ASSERT_TRUE(occupied.has_value());
  grid->cells[occupied->row * grid->width + occupied->column] =
    recovery::CellState::Occupied;
  const auto plan = certified_plan(grid);
  ASSERT_NE(plan, nullptr);
  auto request = accepted_request(plan);

  EXPECT_EQ(
    retained::evaluate(request).reason,
    retained::Reason::TerminalContingencyUnavailable);
}

TEST(
  MpccRateResolvedRetainedRevalidation,
  FeedbackBundleUsesSamePublisherIntervalWallBlockWithoutAuthority)
{
  auto grid = free_grid();
  const auto occupied = grid->world_to_grid(50.15, 0.05);
  ASSERT_TRUE(occupied.has_value());
  grid->cells[occupied->row * grid->width + occupied->column] =
    recovery::CellState::Occupied;
  const auto plan = certified_plan(grid);
  ASSERT_NE(plan, nullptr);
  auto request = accepted_request(plan);
  request.current_time_steering_rad = -0.10;
  request.previous_published_steering_rad = -0.10;
  request.previous_published_command_age_sec = 0.0;

  const auto result = retained::evaluate(request);
  EXPECT_EQ(result.reason, retained::Reason::SteeringUnreachable);
  EXPECT_TRUE(result.feedback_shadow_attempted);
  EXPECT_TRUE(result.feedback_shadow_continuation_available);
  EXPECT_EQ(
    result.feedback_shadow_proof_reason,
    retained::Reason::ContinuationWallBlocked);
  EXPECT_FALSE(result.feedback_shadow_proof_available);
  EXPECT_FALSE(result.proof.has_value());
}

TEST(
  MpccRateResolvedRetainedRevalidation,
  RejectsClearPublisherIntervalWithoutCertifiedWallStopSuffix)
{
  auto grid = free_grid();
  // The first continuation endpoint is at approximately (50.20, 0.10) and
  // the second is at approximately (50.41, 0.10).  Block only the second
  // endpoint.  Receding-horizon authority owns the
  // current publisher interval, not an assertion that an old open-loop suffix
  // will remain executable after the next solve.
  const auto occupied = grid->world_to_grid(50.40, 0.10);
  ASSERT_TRUE(occupied.has_value());
  grid->cells[occupied->row * grid->width + occupied->column] =
    recovery::CellState::Occupied;
  const auto plan = certified_plan(grid);
  ASSERT_NE(plan, nullptr);
  auto request = accepted_request(plan);

  const auto result = retained::evaluate(request);
  EXPECT_EQ(
    result.reason, retained::Reason::TerminalContingencyUnavailable);
  EXPECT_FALSE(result.proof.has_value());
  EXPECT_EQ(
    result.static_wall_scope,
    retained::StaticWallProofScope::PublisherIntervalPrefix);
  EXPECT_TRUE(result.publisher_interval_path_clearance.valid);
  EXPECT_TRUE(result.publisher_interval_path_clearance.clear);
  EXPECT_TRUE(result.continuation_path_clearance.valid);
  EXPECT_FALSE(result.continuation_path_clearance.clear);
}

TEST(MpccRateResolvedRetainedRevalidation, RejectsExhaustedArtifact)
{
  const auto plan = certified_plan();
  auto request = accepted_request(plan);
  request.now_sec = 2.0;
  request.control_origin_sec = 2.0;
  const auto result = retained::evaluate(request);
  EXPECT_EQ(result.reason, retained::Reason::CursorUnavailable);
  EXPECT_EQ(result.cursor_reason, artifact::CursorReason::Exhausted);
}

TEST(
  MpccRateResolvedRetainedRevalidation,
  CertifiesCurrentWorldStopAfterNormalPrefixExhaustion)
{
  const auto plan = certified_plan();
  auto request = accepted_request(plan);
  request.now_sec = 2.0;
  request.control_origin_sec = 2.0;
  request.obstacles.observed_sec = request.now_sec;
  request.control_origin_physical_progress_m = 50.45;
  request.control_pose = {50.45, 0.20, 0.0};
  request.measured_to_control_path = {request.control_pose};
  request.control_origin_speed_mps = 2.0;
  request.current_speed_mps = 2.0;
  request.current_time_steering_rad = 0.10;
  request.current_steering_rad = 0.10;
  request.current_response_steering_rad = 0.10;
  request.previous_published_steering_rad = 0.10;

  const auto normal = retained::evaluate(request);
  ASSERT_EQ(normal.reason, retained::Reason::CursorUnavailable);
  ASSERT_EQ(normal.cursor_reason, artifact::CursorReason::Exhausted);

  const auto successor = retained::evaluate_stop_successor(request);
  ASSERT_EQ(successor.reason, retained::StopSuccessorReason::Accepted);
  ASSERT_TRUE(successor.accepted());
  ASSERT_FALSE(successor.exact_trajectory.elapsed_time_sec.empty());
  ASSERT_EQ(
    successor.exact_trajectory.elapsed_time_sec.size(),
    successor.actuation_samples.size());
  EXPECT_NEAR(successor.exact_trajectory.velocity_mps.back(), 0.0, 1e-9);
  EXPECT_TRUE(successor.control_path_clearance.clear);
  EXPECT_TRUE(successor.successor_path_clearance.clear);
  EXPECT_TRUE(successor.dynamic_clearance.clear);
}

TEST(
  MpccRateResolvedRetainedRevalidation,
  ReifiesCurrentWorldStopAsCanonicalSevenStateAuthority)
{
  const auto plan = certified_plan();
  auto request = accepted_request(plan);
  request.now_sec = 2.0;
  request.control_origin_sec = 2.0;
  request.obstacles.observed_sec = request.now_sec;
  request.control_origin_physical_progress_m = 50.45;
  request.control_pose = {50.45, 0.20, 0.0};
  request.measured_to_control_path = {request.control_pose};
  request.control_origin_speed_mps = 2.0;
  request.current_speed_mps = 2.0;
  request.current_time_steering_rad = 0.10;
  request.current_steering_rad = 0.10;
  request.current_response_steering_rad = 0.10;
  request.previous_published_steering_rad = 0.10;

  const auto successor = retained::evaluate_stop_successor(request);
  ASSERT_TRUE(successor.accepted());
  const auto bundle = stop_bundle::build(request, successor, 101U);
  ASSERT_EQ(bundle.reason, stop_bundle::Reason::Available);
  ASSERT_NE(bundle.plan, nullptr);
  ASSERT_EQ(certified::validate(*bundle.plan), certified::RejectReason::None);
  const auto & execution = *bundle.plan->execution_artifact;
  EXPECT_EQ(execution.identity.sequence, 101U);
  EXPECT_EQ(execution.identity.source_context.decision_id, request.decision_id);
  EXPECT_EQ(execution.identity.source_context.intent, request.current_intent);
  ASSERT_FALSE(execution.control_stages.empty());
  EXPECT_NEAR(
    execution.control_stages.front().acceleration_mps2,
    request.minimum_acceleration_mps2, 1e-12);
  EXPECT_GE(
    execution.control_stages.front().duration_sec,
    execution.publication_interval_sec);

  auto joined_request = request;
  joined_request.plan = bundle.plan;
  joined_request.decision_id = request.decision_id + 1U;
  joined_request.obstacles.generation += 1U;
  joined_request.obstacles.observed_sec = joined_request.now_sec;
  joined_request.execution_clock = {
    retained::ExecutionClockKind::TimeAlignedCandidate,
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()};
  const auto joined = retained::evaluate(joined_request);
  ASSERT_EQ(joined.reason, retained::Reason::Accepted);
  const auto authority = production::build(joined);
  ASSERT_EQ(authority.reason, production::Reason::Available);
  ASSERT_TRUE(authority.authority.has_value());
  EXPECT_EQ(authority.authority->command.intent, request.current_intent);
  EXPECT_NEAR(
    authority.authority->command.acceleration_mps2,
    request.minimum_acceleration_mps2, 1e-12);
}

TEST(
  MpccRateResolvedRetainedRevalidation,
  DoesNotRelabelStopEndpointAsReturnCompletion)
{
  const auto plan = certified_plan(
    free_grid(), contract::ControlIntent::Return);
  auto request = accepted_request(plan);
  request.current_intent = contract::ControlIntent::Return;
  request.now_sec = 2.0;
  request.control_origin_sec = 2.0;
  request.obstacles.observed_sec = request.now_sec;
  request.control_origin_physical_progress_m = 50.45;
  request.control_pose = {50.45, 0.20, 0.0};
  request.measured_to_control_path = {request.control_pose};
  request.control_origin_speed_mps = 2.0;
  request.current_speed_mps = 2.0;
  request.current_time_steering_rad = 0.10;
  request.current_steering_rad = 0.10;
  request.current_response_steering_rad = 0.10;
  request.previous_published_steering_rad = 0.10;

  const auto successor = retained::evaluate_stop_successor(request);
  ASSERT_TRUE(successor.accepted());
  const auto bundle = stop_bundle::build(request, successor, 102U);
  EXPECT_EQ(bundle.reason, stop_bundle::Reason::UnsupportedTerminalIntent);
  EXPECT_EQ(bundle.plan, nullptr);
}

TEST(
  MpccRateResolvedRetainedRevalidation,
  ClassifiesStopSuccessorCommandIndexDiscontinuity)
{
  const auto plan = certified_plan();
  auto request = accepted_request(plan);
  request.now_sec = 2.0;
  request.control_origin_sec = 2.0;
  request.obstacles.observed_sec = request.now_sec;
  request.control_origin_physical_progress_m = 50.45;
  request.control_pose = {50.45, 0.20, 0.0};
  request.measured_to_control_path = {request.control_pose};
  request.control_origin_speed_mps = 2.0;
  request.current_speed_mps = 2.0;
  request.current_time_steering_rad = 0.10;
  request.current_steering_rad = 0.10;
  request.current_response_steering_rad = 0.10;
  request.previous_published_steering_rad = 0.10;

  auto successor = retained::evaluate_stop_successor(request);
  ASSERT_TRUE(successor.accepted());
  ASSERT_FALSE(successor.actuation_samples.empty());
  successor.actuation_samples.front().command_interval_index = 1U;

  const auto bundle = stop_bundle::build(request, successor, 103U);
  EXPECT_EQ(bundle.reason, stop_bundle::Reason::InvalidActuationSequence);
  EXPECT_EQ(
    bundle.actuation_detail,
    stop_bundle::ActuationRejectDetail::CommandIndexDiscontinuity);
  EXPECT_EQ(bundle.rejected_index, 0U);
  EXPECT_DOUBLE_EQ(bundle.observed_value, 1.0);
  EXPECT_DOUBLE_EQ(bundle.required_bound, 0.0);
}

TEST(
  MpccRateResolvedRetainedRevalidation,
  ClassifiesStopSuccessorCommandMutationWithinPublisherInterval)
{
  const auto plan = certified_plan();
  auto request = accepted_request(plan);
  request.now_sec = 2.0;
  request.control_origin_sec = 2.0;
  request.obstacles.observed_sec = request.now_sec;
  request.control_origin_physical_progress_m = 50.45;
  request.control_pose = {50.45, 0.20, 0.0};
  request.measured_to_control_path = {request.control_pose};
  request.control_origin_speed_mps = 2.0;
  request.current_speed_mps = 2.0;
  request.current_time_steering_rad = 0.10;
  request.current_steering_rad = 0.10;
  request.current_response_steering_rad = 0.10;
  request.previous_published_steering_rad = 0.10;

  auto successor = retained::evaluate_stop_successor(request);
  ASSERT_TRUE(successor.accepted());
  ASSERT_GE(successor.actuation_samples.size(), 2U);
  ASSERT_EQ(
    successor.actuation_samples[0].command_interval_index,
    successor.actuation_samples[1].command_interval_index);
  successor.actuation_samples[1].acceleration_mps2 += 0.5;

  const auto bundle = stop_bundle::build(request, successor, 104U);
  EXPECT_EQ(bundle.reason, stop_bundle::Reason::InvalidActuationSequence);
  EXPECT_EQ(
    bundle.actuation_detail,
    stop_bundle::ActuationRejectDetail::CommandChangedWithinInterval);
  EXPECT_EQ(bundle.rejected_index, 1U);
  EXPECT_NEAR(bundle.observed_value, 0.5, 1e-12);
}

TEST(
  MpccRateResolvedRetainedRevalidation,
  RejectsCurrentWorldStopWithDifferentIntentIdentity)
{
  const auto plan = certified_plan();
  auto request = accepted_request(plan);
  request.current_intent = contract::ControlIntent::Cruise;

  const auto successor = retained::evaluate_stop_successor(request);
  EXPECT_EQ(
    successor.reason, retained::StopSuccessorReason::InvalidIdentity);
  EXPECT_FALSE(successor.accepted());
}

TEST(
  MpccRateResolvedRetainedRevalidation,
  RejectsCurrentWorldStopWhenWallOccupiesSuccessor)
{
  auto grid = free_grid();
  const auto occupied = grid->world_to_grid(50.65, 0.20);
  ASSERT_TRUE(occupied.has_value());
  grid->cells[occupied->row * grid->width + occupied->column] =
    recovery::CellState::Occupied;
  const auto plan = certified_plan(grid);
  auto request = accepted_request(plan);
  request.now_sec = 2.0;
  request.control_origin_sec = 2.0;
  request.obstacles.observed_sec = request.now_sec;
  request.control_origin_physical_progress_m = 50.45;
  request.control_pose = {50.45, 0.20, 0.0};
  request.measured_to_control_path = {request.control_pose};
  request.control_origin_speed_mps = 2.0;
  request.current_speed_mps = 2.0;
  request.current_time_steering_rad = 0.10;
  request.current_steering_rad = 0.10;
  request.current_response_steering_rad = 0.10;
  request.previous_published_steering_rad = 0.10;

  const auto successor = retained::evaluate_stop_successor(request);
  EXPECT_EQ(
    successor.reason, retained::StopSuccessorReason::StaticPathBlocked);
  EXPECT_FALSE(successor.accepted());
  EXPECT_TRUE(successor.control_path_clearance.clear);
  EXPECT_FALSE(successor.successor_path_clearance.clear);
}

TEST(
  MpccRateResolvedRetainedRevalidation,
  RejectsCurrentWorldStopWhenPeerOccupiesSuccessor)
{
  const auto plan = certified_plan();
  auto request = accepted_request(plan);
  request.now_sec = 2.0;
  request.control_origin_sec = 2.0;
  request.obstacles.observed_sec = request.now_sec;
  request.control_origin_physical_progress_m = 50.45;
  request.control_pose = {50.45, 0.20, 0.0};
  request.measured_to_control_path = {request.control_pose};
  request.control_origin_speed_mps = 2.0;
  request.current_speed_mps = 2.0;
  request.current_time_steering_rad = 0.10;
  request.current_steering_rad = 0.10;
  request.current_response_steering_rad = 0.10;
  request.previous_published_steering_rad = 0.10;
  request.obstacles.obstacles.push_back(
    {"d2", {50.75, 0.20, 0.0, 0.0, 0.10}});

  const auto successor = retained::evaluate_stop_successor(request);
  EXPECT_EQ(
    successor.reason, retained::StopSuccessorReason::DynamicPathBlocked);
  EXPECT_FALSE(successor.accepted());
  EXPECT_FALSE(successor.dynamic_clearance.clear);
  EXPECT_EQ(successor.dynamic_clearance.blocking_obstacle_id, "d2");
}

TEST(
  MpccRateResolvedRetainedRevalidation,
  RejectsRemainingSuffixShorterThanPublisherInterval)
{
  const auto plan = certified_plan();
  auto request = accepted_request(plan);
  request.now_sec = 1.195;
  request.control_origin_sec = 1.195;
  request.obstacles.observed_sec = request.now_sec;
  request.control_origin_physical_progress_m = 50.39;
  request.control_pose = {50.39, 0.195, 0.0};
  request.measured_to_control_path = {request.control_pose};
  request.current_speed_mps = 2.195;
  request.control_origin_speed_mps = 2.195;
  request.current_time_steering_rad = 0.1195;
  request.current_steering_rad = 0.1195;
  request.current_response_steering_rad = 0.109;
  request.previous_published_steering_rad = 0.1195;

  const auto result = retained::evaluate(request);
  EXPECT_EQ(result.reason, retained::Reason::ContinuationRejected);
  EXPECT_FALSE(result.proof.has_value());
}

}  // namespace
