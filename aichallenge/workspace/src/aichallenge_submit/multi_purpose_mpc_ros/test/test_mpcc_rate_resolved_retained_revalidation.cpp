#include "multi_purpose_mpc_ros/mpcc_scheduled_observation.hpp"
#include "multi_purpose_mpc_ros/mpcc_scheduled_dispatch.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_shadow.hpp"
#include "multi_purpose_mpc_ros/mpcc_wire_command.hpp"
#include "mpcc_vehicle_model_fixture.hpp"
#include "mpcc_wall_input_fixture.hpp"
#include "multi_purpose_mpc_ros/detail/mpcc_footprint_enclosure.hpp"
#include "multi_purpose_mpc_ros/mpcc_applied_input_yaml.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_applied_program.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_scheduled.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_retained_revalidation.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_production_adapter.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_stop_successor_bundle.hpp"

#include <gtest/gtest.h>
#include "multi_purpose_mpc_ros/mpcc_architecture_snapshot.hpp"
#include <chrono>
#include <fstream>
#include <thread>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <random>
#include <stdexcept>

TEST(MpccAppliedProgram, CapturedMovingStopRemainsClearOfOriginalWallThroughRest)
{
  namespace vm = multi_purpose_mpc_ros::mpcc_vehicle_model;
  namespace num = vm::numerical;
  namespace rec = multi_purpose_mpc_ros::recovery_footprint;
  namespace fixture = multi_purpose_mpc_ros::test;
  const auto observation = fixture::wall_packet_observation();
  const auto program = fixture::wall_packet_program();
  const auto grid = fixture::wall_packet_grid();
  const auto footprint = multi_purpose_mpc_ros::mpcc_rate_resolved_physical_wall::
    resolve_clearance_footprint({1.615, .51, .768, .768, .05}, .2);
  ASSERT_TRUE(footprint);
  ASSERT_TRUE(grid.valid());
  const auto & origin = observation.initial.state;
  double rejected_sec = NAN;
  std::size_t checked = 0;
  const auto prediction = vm::predict_applied_inputs_to_rest(
    observation, program, {"awsim-2025-empirical-receiver-age-250ms-v1", .25, .25, .1},
    fixture::vehicle_model(),
    [&](const vm::BodyRanges & body, double begin, double) {
      num::Box box;
      for (std::size_t i = 0; i < box.size(); ++i) box[i] = {body[i].lower, body[i].upper};
      const auto enclosed = num::footprint(box, *footprint);
      const auto & p = enclosed.pose;
      const double c = std::cos(origin.yaw_rad), s = std::sin(origin.yaw_rad);
      const rec::Pose2D world{origin.x_m + c*p.x_m - s*p.y_m,
        origin.y_m + s*p.x_m + c*p.y_m, origin.yaw_rad + p.yaw_rad};
      const auto sampled = rec::sample_footprint(grid, enclosed.extents, world);
      ++checked;
      const bool clear = sampled.valid && !sampled.out_of_map && sampled.contact_cells.empty();
      if (!clear) rejected_sec = begin;
      return clear;
    });
  ASSERT_TRUE(prediction.tube) << "wall rejection at " << rejected_sec;
  EXPECT_GT(checked, 200U);
  EXPECT_GT(prediction.tube->rest_sec, program.commands.back().published_sec);
  for (const std::size_t index : {3U, 4U, 5U}) {
    EXPECT_DOUBLE_EQ(prediction.tube->source_to_rest.back().endpoint_body[index].lower, 0);
    EXPECT_DOUBLE_EQ(prediction.tube->source_to_rest.back().endpoint_body[index].upper, 0);
  }
}

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
  context.vehicle_model_fingerprint = multi_purpose_mpc_ros::mpcc_vehicle_model::fingerprint(
    multi_purpose_mpc_ros::test::vehicle_model());
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
    contract::Formulation::VelocitySteeringTireBodyProgress9State;
  context.vehicle_model_fingerprint = multi_purpose_mpc_ros::mpcc_vehicle_model::fingerprint(
    multi_purpose_mpc_ros::test::vehicle_model());
  context.state_schema_id =
    multi_purpose_mpc_ros::mpcc_rate_resolved::kCoordinateStateSchema;
  context.input_schema_id = "accel-steering-rate-progress-rate-v1";
  context.bounds_schema_id = "stage-wall-v1";
  context.cost_schema_id = "velocity-progress-steering-rate-v1";
  return contract::seal_problem_context(std::move(context));
}

artifact::ExecutionArtifact execution_artifact(
  const contract::ControlIntent intent = contract::ControlIntent::Track)
{
  artifact::ExecutionArtifact value;
  value.vehicle_model = multi_purpose_mpc_ros::test::vehicle_model();
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
  value.semantic_initial_state = value.predicted_states.front();
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
  AddsUncertaintyToCompletePeerBodyWithoutEgoSubtraction)
{
  const recovery::FootprintExtents ego{
    1.49, 0.51, 0.725, 0.725, 0.05};

  const auto radius = retained::resolve_peer_circle_radius(1.876, 0.05);

  ASSERT_TRUE(radius.has_value());
  EXPECT_NEAR(radius.value(), 1.926, 1e-12);
  const auto clearance = recovery::circle_obstacle_clearance_at_time(
    ego, recovery::Pose2D{0.0, 0.0, 0.0},
    recovery::CircleObstacle{0.0, 2.701, 0.0, 0.0, radius.value()}, 0.0);
  ASSERT_TRUE(clearance.has_value());
  EXPECT_NEAR(clearance.value(), 0.0, 1e-12);
}

TEST(MpccRateResolvedRetainedRevalidation, FullPeerBodyOverlapCannotHavePositiveClearance)
{
  // Independently measured common kart body: peer GNSS at0, ego base_link at2.2.
  // A peer solid-body projection vertex is inside the ego solid-body projection.
  // The old width-derived circle incorrectly reports+0.822mclearance here.
  const recovery::FootprintExtents ego{1.615, 0.51, 0.768, 0.768, 0.05};
  const auto radius = retained::resolve_peer_circle_radius(1.876, 0.05);
  ASSERT_TRUE(radius);
  const auto clearance = recovery::circle_obstacle_clearance_at_time(
    ego, recovery::Pose2D{2.2, 0.0, 0.0},
    recovery::CircleObstacle{0.0, 0.0, 0.0, 0.0, *radius}, 0.0);
  ASSERT_TRUE(clearance);
  EXPECT_LE(*clearance, 0.0);
}

TEST(
  MpccRateResolvedRetainedRevalidation,
  RejectsInvalidPeerBodyOrUncertainty)
{
  EXPECT_FALSE(retained::resolve_peer_circle_radius(0.0, 0.05));
  EXPECT_FALSE(retained::resolve_peer_circle_radius(-1.0, 0.05));
  EXPECT_FALSE(retained::resolve_peer_circle_radius(1.876, -0.05));
  EXPECT_FALSE(retained::resolve_peer_circle_radius(
      std::numeric_limits<double>::infinity(), 0.05));
  EXPECT_FALSE(retained::resolve_peer_circle_radius(
      1.876, std::numeric_limits<double>::quiet_NaN()));
  EXPECT_FALSE(retained::resolve_peer_circle_radius(
      std::numeric_limits<double>::max(), std::numeric_limits<double>::max()));
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

TEST(MpccRateResolvedRetainedRevalidation, NormalPathObservationCannotExtrapolateOrChangeProduction)
{
  const auto request = accepted_request(certified_plan());
  const auto before = retained::evaluate(request);
  ASSERT_EQ(before.reason, retained::Reason::Accepted);
  // Production has an explicit terminal tail. The observation below still
  // measures only the original solved profile and must not extend that domain.
  EXPECT_TRUE(before.proof->terminal_stop_normal_path_reference);
  EXPECT_EQ(before.terminal_stop_reference_attempts, 1U);
  const auto observation = retained::observe_normal_path_stop(request);
  EXPECT_TRUE(observation.profile_available);
  // The solved path ends at 0.4 m; physical braking needs more distance.
  EXPECT_FALSE(observation.accepted);
  EXPECT_EQ(observation.reason, retained::Reason::TerminalContingencyUnavailable);
  EXPECT_EQ(
    observation.terminal_reason,
    multi_purpose_mpc_ros::mpcc_rate_resolved_physical_adapter::
    StopContingencyRejectReason::InvalidLateralPolicy);
  const auto after = retained::evaluate(request);
  ASSERT_TRUE(after.proof.has_value());
  EXPECT_EQ(after.reason, before.reason);
  EXPECT_EQ(after.proof->actuation.steering_rad, before.proof->actuation.steering_rad);
  EXPECT_EQ(after.proof->terminal_stop_trajectory.lateral_m,
    before.proof->terminal_stop_trajectory.lateral_m);
}

TEST(MpccRateResolvedRetainedRevalidation, NormalPathObservationKeepsCurrentWorldIdentityChecks)
{
  auto request = accepted_request(certified_plan());
  request.current_wall_grid = free_grid();
  auto changed = std::make_shared<recovery::OccupancyGrid>(*request.current_wall_grid);
  changed->cells.front() = recovery::CellState::Occupied;
  request.current_wall_grid = changed;
  const auto observation = retained::observe_normal_path_stop(request);
  EXPECT_TRUE(observation.profile_available);
  EXPECT_FALSE(observation.accepted);
  EXPECT_EQ(observation.reason, retained::Reason::StaticWorldMismatch);
}

TEST(MpccRateResolvedRetainedRevalidation, StopCertificateTracksTheSolvedNormalGeometry)
{
  namespace adapter = multi_purpose_mpc_ros::mpcc_rate_resolved_physical_adapter;
  auto value = execution_artifact();
  const auto execution = std::make_shared<const artifact::ExecutionArtifact>(value);
  const auto snapshot = source_snapshot(execution->identity);
  const auto built = certified::build(execution, snapshot, accepted_result(snapshot));
  ASSERT_EQ(built.reason, certified::RejectReason::None);
  auto request = accepted_request(built.plan);
  // A fresh 1m/s origin stops within the short synthetic normal-path profile
  // under the unchanged production wire-braking bound and native body model.
  request.current_speed_mps = 1.0;
  request.control_origin_speed_mps = 1.0;
  const auto result = retained::evaluate(request);
  ASSERT_TRUE(result.proof.has_value());
  const auto profile = adapter::build_normal_path_stop_profile(*execution);
  EXPECT_TRUE(result.proof->terminal_stop_normal_path_reference);
  EXPECT_EQ(result.terminal_stop_reference_attempts, 1U);
  ASSERT_TRUE(profile.has_value());
  const auto & initial = result.current_control_state;
  const auto expected = adapter::build_stop_contingency(
    *execution, result.proof->cursor, result.proof->actuation,
    adapter::ContinuationInitialState{
      initial.lateral_m, initial.lag_m, initial.heading_offset_rad,
      initial.velocity_mps, initial.progress_m, result.proof->actuation.steering_rad,
      request.current_response_steering_rad},
    snapshot.terminal_stop_course_geometry, request.stop_lateral_policy,
    request.minimum_acceleration_mps2, 0.0, &profile.value());
  ASSERT_TRUE(expected.exact_trajectory.has_value());
  EXPECT_EQ(result.proof->terminal_stop_trajectory.lateral_m,
    expected.exact_trajectory->lateral_m);
  EXPECT_EQ(result.proof->terminal_stop_actuation_samples.back().end_steering_rad,
    expected.actuation_samples.back().end_steering_rad);
}

TEST(MpccRateResolvedRetainedRevalidation, StopReferenceReachesRestBeyondSolvedHorizon)
{
  namespace adapter = multi_purpose_mpc_ros::mpcc_rate_resolved_physical_adapter;
  const auto plan = certified_plan();
  ASSERT_TRUE(plan);
  const auto & execution = *plan->execution_artifact;
  const auto request = accepted_request(plan);
  const auto result = retained::evaluate(request);
  ASSERT_TRUE(result.proof);
  const auto solved_reference = adapter::build_normal_path_stop_profile(execution);
  ASSERT_TRUE(solved_reference);
  const auto & initial = result.current_control_state;
  const auto truncated = adapter::build_stop_contingency(
    execution, result.proof->cursor, result.proof->actuation,
    {initial.lateral_m, initial.lag_m, initial.heading_offset_rad,
      initial.velocity_mps, initial.progress_m, result.proof->actuation.steering_rad,
      request.current_response_steering_rad, request.current_lateral_velocity_mps,
      request.current_yaw_rate_radps},
    plan->physical_snapshot->terminal_stop_course_geometry, request.stop_lateral_policy,
    request.minimum_acceleration_mps2, 0.0, &*solved_reference);
  EXPECT_EQ(truncated.reason, adapter::StopContingencyRejectReason::InvalidLateralPolicy);
  // Braking continues beyond the solved normal horizon. Its reference must
  // explicitly cover that interval without switching to the centerline.
  EXPECT_TRUE(result.terminal_stop_normal_path_reference);
  EXPECT_EQ(result.terminal_stop_reference_attempts, 1U);
  EXPECT_GT(result.proof->terminal_stop_trajectory.progress_m.back(),
    execution.course_progress_origin_m + solved_reference->progress_m.back());
  EXPECT_DOUBLE_EQ(result.proof->terminal_stop_trajectory.velocity_mps.back(), 0.0);
  EXPECT_DOUBLE_EQ(result.proof->terminal_stop_actuation_samples.back().end_lateral_velocity_mps, 0.0);
  EXPECT_DOUBLE_EQ(result.proof->terminal_stop_actuation_samples.back().end_yaw_rate_radps, 0.0);
  EXPECT_TRUE(production::build(result).authority);
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
  EXPECT_TRUE(result.terminal_stop_attempted);
  EXPECT_TRUE(result.terminal_stop_certified);
  EXPECT_TRUE(result.proof->terminal_stop_certified);
  EXPECT_FALSE(result.proof->terminal_stop_trajectory.progress_m.empty());
}

TEST(
  MpccRateResolvedRetainedRevalidation,
  ExactWallClearInitialStateOwnsTerminalStopBeforeApproximateSupport)
{
  auto execution = std::make_shared<const artifact::ExecutionArtifact>(
    execution_artifact());
  auto snapshot = source_snapshot(execution->identity);
  std::fill(
    snapshot.terminal_stop_course_geometry.lateral_lower_m.begin(),
    snapshot.terminal_stop_course_geometry.lateral_lower_m.end(), -0.04);
  const auto built = certified::build(
    execution, snapshot, accepted_result(snapshot));
  ASSERT_EQ(built.reason, certified::RejectReason::None);
  ASSERT_NE(built.plan, nullptr);
  auto request = accepted_request(built.plan);
  request.control_pose = {50.05, -0.05, 0.0};
  request.measured_to_control_path = {request.control_pose};

  const auto result = retained::evaluate(request);

  ASSERT_EQ(result.reason, retained::Reason::Accepted);
  EXPECT_TRUE(result.terminal_stop_approximate_support_exceeded);
  EXPECT_GE(
    result.terminal_stop_first_approximate_support_exceeded_sample, 0);
  EXPECT_GT(
    result.terminal_stop_maximum_approximate_support_violation_m, 0.0);
  EXPECT_TRUE(result.terminal_stop_certified);
  EXPECT_TRUE(result.terminal_stop_path_clearance.valid);
  EXPECT_TRUE(result.terminal_stop_path_clearance.clear);
}

TEST(
  MpccRateResolvedRetainedRevalidation,
  RejectsFullSuffixWhoseCurrentWorldStopIsWallBlocked)
{
  auto grid = free_grid();
  const auto occupied = grid->world_to_grid(50.65, 0.10);
  ASSERT_TRUE(occupied.has_value());
  grid->cells[occupied->row * grid->width + occupied->column] =
    recovery::CellState::Occupied;
  const auto plan = certified_plan(grid);
  ASSERT_NE(plan, nullptr);

  const auto result = retained::evaluate(accepted_request(plan));

  EXPECT_EQ(
    result.static_wall_scope,
    retained::StaticWallProofScope::FullSuffix);
  EXPECT_TRUE(result.continuation_path_clearance.valid);
  EXPECT_TRUE(result.continuation_path_clearance.clear);
  EXPECT_TRUE(result.terminal_stop_attempted);
  EXPECT_FALSE(result.terminal_stop_certified);
  EXPECT_EQ(result.reason, retained::Reason::TerminalContingencyUnavailable);
  EXPECT_FALSE(result.proof.has_value());
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
  UsesCurrentPhysicalOriginWithoutAWireEqualsNetVelocityEnvelope)
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
  const auto cursor = artifact::resolve_cursor(*plan->execution_artifact, request.control_origin_sec);
  const auto historical = artifact::extract_actuation(*plan->execution_artifact, cursor);
  ASSERT_TRUE(historical.actuation);
  EXPECT_NEAR(result.proof->velocity_difference_mps,
    historical.actuation->predicted_speed_mps - request.current_speed_mps, 1e-9);
  EXPECT_TRUE(std::isnan(result.proof->reachable_velocity_lower_mps));
  EXPECT_TRUE(std::isnan(result.proof->reachable_velocity_upper_mps));
  EXPECT_DOUBLE_EQ(result.proof->actuation.predicted_speed_mps, request.control_origin_speed_mps);
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
    {"later-crossing", {51.056, 0.054, -5.0, 0.0, 0.001}});

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

TEST(MpccRateResolvedRetainedRevalidation, MaterializedFeedbackStopExecutesBrakingAfterFirstPublication)
{
  auto request = accepted_request(certified_plan());
  request.previous_published_steering_rad = 0.079;
  request.previous_published_command_age_sec = 0.025;
  const auto original = retained::evaluate(request);
  ASSERT_EQ(original.reason, retained::Reason::Accepted);
  ASSERT_TRUE(original.proof);
  ASSERT_TRUE(original.proof->latest_state_feedback_bundle);
  ASSERT_TRUE(original.terminal_stop_certified);
  ASSERT_FALSE(original.terminal_stop_uses_solved_suffix);
  const auto selected = production::build(original);
  ASSERT_TRUE(selected.authority);
  const auto bundle = stop_bundle::build_certified_terminal(request, original, 101U);
  ASSERT_EQ(bundle.reason, stop_bundle::Reason::Available);
  ASSERT_TRUE(bundle.plan);
  const auto & execution = *bundle.plan->execution_artifact;
  ASSERT_GE(execution.control_stages.size(), 2U);
  EXPECT_DOUBLE_EQ(execution.semantic_initial_steering_rad,
    selected.authority->command.steering_tire_angle_rad);
  EXPECT_DOUBLE_EQ(execution.control_stages.front().acceleration_mps2,
    selected.authority->command.acceleration_mps2);
  EXPECT_DOUBLE_EQ(execution.control_stages[1U].acceleration_mps2, request.minimum_acceleration_mps2);
  EXPECT_TRUE(execution.terminal_body_rest_required);
  const auto native = multi_purpose_mpc_ros::mpcc_rate_resolved_physical_adapter::build(
    execution, request.current_intent, execution.identity.source_context.stage_geometry_id);
  ASSERT_TRUE(native.exact_trajectory);
  EXPECT_DOUBLE_EQ(native.exact_trajectory->velocity_mps.back(), 0.0);
  EXPECT_DOUBLE_EQ(execution.predicted_states.back().lateral_velocity_mps, 0.0);
  EXPECT_DOUBLE_EQ(execution.predicted_states.back().yaw_rate_radps, 0.0);

  auto current = request;
  current.plan = bundle.plan;
  current.execution_clock = {retained::ExecutionClockKind::TimeAlignedCandidate, NAN, NAN};
  const auto joined = retained::evaluate(current);
  ASSERT_EQ(joined.reason, retained::Reason::Accepted);
  ASSERT_TRUE(joined.proof);
  EXPECT_TRUE(joined.terminal_stop_uses_solved_suffix);
  EXPECT_FALSE(joined.proof->stateless_current_world_bundle());
  const auto initial_command = production::build(joined);
  ASSERT_TRUE(initial_command.authority);
  EXPECT_DOUBLE_EQ(initial_command.authority->command.steering_tire_angle_rad,
    selected.authority->command.steering_tire_angle_rad);
  EXPECT_DOUBLE_EQ(initial_command.authority->command.acceleration_mps2,
    selected.authority->command.acceleration_mps2);

  // Advance the observed body by the exact first publisher interval. The
  // second command must be the certified brake, not the old source input.
  const auto & terminal = original.proof->terminal_stop_trajectory;
  const std::size_t i = original.proof->terminal_stop_publisher_interval_sample_count - 1U;
  const auto & state = original.proof->terminal_stop_actuation_samples[i];
  const auto frame = multi_purpose_mpc_ros::mpc_stage_geometry::sample_course_frame(
    bundle.plan->physical_snapshot->course_frame_knots, terminal.progress_m[i]);
  ASSERT_TRUE(frame);
  const auto pose = contract::reconstruct_planar_pose_from_frenet(
    {frame->x_m, frame->y_m, frame->heading_rad},
    {terminal.lateral_m[i], terminal.lag_m[i], terminal.heading_offset_rad[i]});
  ASSERT_TRUE(pose);
  current.execution_clock = {retained::ExecutionClockKind::PublishedPlan, request.control_origin_sec, 0.0};
  current.now_sec += execution.publication_interval_sec;
  current.control_origin_sec = current.now_sec;
  ++current.decision_id;
  ++current.obstacles.generation;
  current.obstacles.observed_sec = current.now_sec;
  current.control_pose = {pose->x_m, pose->y_m, pose->yaw_rad};
  current.measured_to_control_path = {current.control_pose};
  current.control_origin_physical_progress_m = terminal.progress_m[i] + terminal.lag_m[i];
  current.current_speed_mps = current.control_origin_speed_mps = state.end_velocity_mps;
  current.current_steering_rad = current.previous_published_steering_rad =
    selected.authority->command.steering_tire_angle_rad;
  current.current_time_steering_rad = current.current_response_steering_rad = state.end_response_steering_rad;
  current.current_lateral_velocity_mps = state.end_lateral_velocity_mps;
  current.current_yaw_rate_radps = state.end_yaw_rate_radps;
  current.previous_published_command_age_sec = execution.publication_interval_sec;
  const auto next = retained::evaluate(current);
  ASSERT_EQ(next.reason, retained::Reason::Accepted);
  const auto next_command = production::build(next);
  ASSERT_TRUE(next_command.authority);
  EXPECT_DOUBLE_EQ(next_command.authority->command.acceleration_mps2, request.minimum_acceleration_mps2);

  auto wrong_clock = request;
  wrong_clock.now_sec += 0.001;
  EXPECT_EQ(stop_bundle::build_certified_terminal(wrong_clock, original, 102U).reason,
    stop_bundle::Reason::InvalidIdentity);
  auto missing = original;
  missing.proof.reset();
  EXPECT_EQ(stop_bundle::build_certified_terminal(request, missing, 103U).reason,
    stop_bundle::Reason::StopSuccessorUnavailable);
}

TEST(MpccRateResolvedRetainedRevalidation, SerializedStopRejoinsEveryPublishedModelState)
{
  auto request = accepted_request(certified_plan());
  const auto stop = retained::evaluate_stop_successor(request);
  ASSERT_TRUE(stop.accepted());
  const auto built = stop_bundle::build(request, stop, 301U);
  ASSERT_EQ(built.reason, stop_bundle::Reason::Available);
  ASSERT_TRUE(built.plan);
  const auto & execution = *built.plan->execution_artifact;
  ASSERT_TRUE(artifact::serialized_stop_schedule(execution));
  auto invalid = execution;
  invalid.terminal_body_rest_required = false;
  EXPECT_NE(artifact::validate(invalid), artifact::RejectReason::None);
  invalid = execution;
  invalid.control_stages.front().duration_sec *= 0.5;
  EXPECT_NE(artifact::validate(invalid), artifact::RejectReason::None);

  auto current = request;
  current.plan = built.plan;
  current.execution_clock = {retained::ExecutionClockKind::PublishedPlan, request.control_origin_sec, 0.0};
  double elapsed = 0.0;
  std::size_t dense_end = 0U;
  for (std::size_t stage = 0U; stage < execution.control_stages.size(); ++stage) {
    SCOPED_TRACE(stage);
    current.now_sec = current.control_origin_sec = request.control_origin_sec + elapsed;
    current.obstacles.observed_sec = current.now_sec;
    current.previous_published_command_age_sec = execution.publication_interval_sec;
    const auto joined = retained::evaluate(current);
    ASSERT_EQ(joined.reason, retained::Reason::Accepted) << retained::to_string(joined.reason);
    ASSERT_TRUE(joined.terminal_stop_uses_solved_suffix);
    const auto published = production::build(joined);
    ASSERT_TRUE(published.authority);
    EXPECT_DOUBLE_EQ(published.authority->command.steering_tire_angle_rad,
      execution.predicted_states[stage + 1U].steering_rad);
    elapsed += execution.control_stages[stage].duration_sec;
    while (dense_end + 1U < stop.actuation_samples.size() &&
      stop.actuation_samples[dense_end].elapsed_time_sec < elapsed - 1e-9) ++dense_end;
    const auto & sample = stop.actuation_samples[dense_end];
    const auto & trajectory = stop.exact_trajectory;
    const auto frame = multi_purpose_mpc_ros::mpc_stage_geometry::sample_course_frame(
      built.plan->physical_snapshot->course_frame_knots, trajectory.progress_m[dense_end]);
    ASSERT_TRUE(frame);
    const auto pose = contract::reconstruct_planar_pose_from_frenet(
      {frame->x_m, frame->y_m, frame->heading_rad},
      {trajectory.lateral_m[dense_end], trajectory.lag_m[dense_end], trajectory.heading_offset_rad[dense_end]});
    ASSERT_TRUE(pose);
    current.control_pose = {pose->x_m, pose->y_m, pose->yaw_rad};
    current.measured_to_control_path = {current.control_pose};
    current.current_speed_mps = current.control_origin_speed_mps = sample.end_velocity_mps;
    current.current_steering_rad = current.previous_published_steering_rad = sample.end_steering_rad;
    current.current_time_steering_rad = current.current_response_steering_rad = sample.end_response_steering_rad;
    current.current_lateral_velocity_mps = sample.end_lateral_velocity_mps;
    current.current_yaw_rate_radps = sample.end_yaw_rate_radps;
    current.control_origin_physical_progress_m = trajectory.progress_m[dense_end] + trajectory.lag_m[dense_end];
  }
  EXPECT_DOUBLE_EQ(current.current_speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(current.current_lateral_velocity_mps, 0.0);
  EXPECT_DOUBLE_EQ(current.current_yaw_rate_radps, 0.0);
}

TEST(MpccRateResolvedRetainedRevalidation, PublishesCertifiedRestAfterSourceCursorExpires)
{
  auto request = accepted_request(certified_plan());
  request.now_sec = request.control_origin_sec = 2.0;
  request.obstacles.observed_sec = request.now_sec;
  request.control_origin_physical_progress_m = 50.45;
  request.control_pose = {50.45, 0.20, 0.0};
  request.measured_to_control_path = {request.control_pose};
  request.current_speed_mps = request.control_origin_speed_mps = 0.0;
  request.current_lateral_velocity_mps = request.current_yaw_rate_radps = 0.0;
  request.current_time_steering_rad = request.current_response_steering_rad = 0.10;
  request.current_steering_rad = request.previous_published_steering_rad = 0.10;
  ASSERT_EQ(retained::evaluate(request).reason, retained::Reason::CursorUnavailable);
  const auto stop = retained::evaluate_stop_successor(request);
  ASSERT_TRUE(stop.accepted());
  const auto bundle = stop_bundle::build(request, stop, 101U);
  ASSERT_EQ(bundle.reason, stop_bundle::Reason::Available);
  ASSERT_TRUE(bundle.plan);
  const auto & execution = *bundle.plan->execution_artifact;
  ASSERT_EQ(execution.control_stages.size(), 1U);
  EXPECT_EQ(execution.nominal_path_distance_m, (std::vector<double>{0.0, 0.0}));
  EXPECT_DOUBLE_EQ(execution.control_stages.front().duration_sec, execution.publication_interval_sec);
  const auto native = multi_purpose_mpc_ros::mpcc_rate_resolved_physical_adapter::build(
    execution, request.current_intent, execution.identity.source_context.stage_geometry_id);
  ASSERT_TRUE(native.exact_trajectory);
  EXPECT_TRUE(native.exact_trajectory->stationary_path_suffix_allowed);
  EXPECT_DOUBLE_EQ(native.exact_trajectory->stationary_velocity_tolerance_mps, 0.0);
  EXPECT_DOUBLE_EQ(native.exact_trajectory->velocity_mps.back(), 0.0);

  auto current = request;
  current.plan = bundle.plan;
  current.execution_clock = {retained::ExecutionClockKind::TimeAlignedCandidate, NAN, NAN};
  const auto joined = retained::evaluate(current);
  ASSERT_EQ(joined.reason, retained::Reason::Accepted);
  EXPECT_TRUE(joined.terminal_stop_uses_solved_suffix);
  const auto command = production::build(joined);
  ASSERT_TRUE(command.authority);
  EXPECT_DOUBLE_EQ(command.authority->command.predicted_speed_mps, 0.0);
  EXPECT_DOUBLE_EQ(command.authority->command.acceleration_mps2, request.minimum_acceleration_mps2);

  // Repeated distance may not represent moving, rotating or launching body
  // states, nor may an ordinary artifact claim the terminal Stop exception.
  for (int mutation = 0; mutation < 6; ++mutation) {
    auto invalid = execution;
    if (mutation == 0) { invalid.terminal_body_rest_required = false; }
    if (mutation == 1) { invalid.predicted_states.back().velocity_mps = 0.01; }
    if (mutation == 2) { invalid.predicted_states.back().lateral_velocity_mps = 0.01; }
    if (mutation == 3) { invalid.predicted_states.back().yaw_rate_radps = 0.01; }
    if (mutation == 4) { invalid.control_stages.front().acceleration_mps2 = 0.01; }
    if (mutation == 5) { invalid.nominal_path_distance_m.back() = -0.01; }
    EXPECT_EQ(artifact::validate(invalid), mutation == 0 ?
      artifact::RejectReason::InvalidCertificate : artifact::RejectReason::InvalidPathDistance);
  }
  current.obstacles.obstacles.push_back({"occupying-peer", {50.45, 0.20, 0.0, 0.0, 0.2}});
  const auto blocked = retained::evaluate(current);
  EXPECT_NE(blocked.reason, retained::Reason::Accepted);
  EXPECT_FALSE(production::build(blocked).authority);
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
  StopSuccessorPreservesCurrentPoseWhenProgressAssociationHasLag)
{
  auto request = accepted_request(certified_plan());
  request.now_sec = 2.0;
  request.control_origin_sec = 2.0;
  request.obstacles.observed_sec = request.now_sec;
  request.control_origin_physical_progress_m = 50.45;
  request.control_pose = {50.60, 0.20, 0.0};
  request.measured_to_control_path = {request.control_pose};
  request.measured_to_control_elapsed_sec = {0.0};
  request.current_speed_mps = 2.0;
  request.control_origin_speed_mps = 2.0;
  request.current_steering_rad = 0.10;
  request.current_response_steering_rad = 0.10;
  request.previous_published_steering_rad = 0.10;

  const auto successor = retained::evaluate_stop_successor(request);
  ASSERT_TRUE(successor.accepted());
  const auto bundle = stop_bundle::build(request, successor, 101U);
  ASSERT_NE(bundle.plan, nullptr);
  const auto & execution = *bundle.plan->execution_artifact;
  ASSERT_TRUE(execution.semantic_initial_state.has_value());
  const auto & initial = *execution.semantic_initial_state;
  const double course_progress = execution.course_progress_origin_m + initial.progress_m;
  const auto frame = multi_purpose_mpc_ros::mpc_stage_geometry::sample_course_frame(
    bundle.plan->physical_snapshot->course_frame_knots, course_progress);
  ASSERT_TRUE(frame.has_value());
  const auto reconstructed = contract::reconstruct_planar_pose_from_frenet(
    {frame->x_m, frame->y_m, frame->heading_rad},
    {initial.lateral_m, initial.lag_m, initial.heading_offset_rad});
  ASSERT_TRUE(reconstructed.has_value());
  EXPECT_NEAR(reconstructed->x_m, request.control_pose.x_m, 1e-10);
  EXPECT_NEAR(reconstructed->y_m, request.control_pose.y_m, 1e-10);
  EXPECT_NEAR(reconstructed->yaw_rad, request.control_pose.yaw_rad, 1e-10);
}

TEST(
  MpccRateResolvedRetainedRevalidation,
  CompleteCurrentWorldSuffixThroughRestOwnsItsTerminalProof)
{
  auto execution = std::make_shared<artifact::ExecutionArtifact>(execution_artifact());
  execution->physical_global_tolerance = 0.016;
  execution->semantic_initial_steering_rad = 0.0;
  execution->semantic_initial_response_steering_rad = 0.0;
  execution->predicted_states = {
    {0.0, 0.10, 0.0, 2.0, 0.0, 0.0, 0.0},
    {0.0, 0.10, 0.0, 3.2, 3.12, 0.0, 0.0},
    {0.0, 0.10, 0.0, 0.0, 3.12 + 1.6 * (3.2 / 3.0), 0.0, 0.0},
  };
  execution->semantic_initial_state = execution->predicted_states.front();
  execution->control_stages = {
    {1.0, 0.0, 2.6, 1.2, 0.0, 4.0, -3.0, 1.37},
    {-3.0, 0.0, 1.6, 3.2 / 3.0, 0.0, 4.0, -3.0, 1.37},
  };
  execution->nominal_path_distance_m =
    {0.0, 3.12, execution->predicted_states.back().progress_m};
  auto snapshot = source_snapshot(execution->identity);
  snapshot.course_frame_knots.back() = {57.0, 57.0, 0.0, 0.0, 7};
  snapshot.terminal_stop_course_geometry.progress_m = {0.0, 3.0, 7.0};
  const auto exact = multi_purpose_mpc_ros::mpcc_rate_resolved_physical_adapter::build(
    *execution, execution->identity.source_context.intent,
    execution->identity.source_context.stage_geometry_id);
  ASSERT_TRUE(exact.exact_trajectory.has_value());
  snapshot.trajectory = *exact.exact_trajectory;
  snapshot.current_pose = {50.10, 0.0, 0.0};
  snapshot.control_prefix = {snapshot.current_pose};
  const auto plan = certified::build(execution, snapshot, physical::evaluate(snapshot));
  ASSERT_NE(plan.plan, nullptr);
  auto request = accepted_request(plan.plan);
  request.now_sec = execution->prediction_origin_sec;
  request.control_origin_sec = request.now_sec;
  request.execution_clock = {retained::ExecutionClockKind::TimeAlignedCandidate, 0.0, 0.0};
  request.obstacles.observed_sec = request.now_sec;
  request.control_origin_physical_progress_m = 50.10;
  request.control_pose = snapshot.current_pose;
  request.measured_to_control_path = {request.control_pose};
  request.current_speed_mps = 2.0;
  request.control_origin_speed_mps = 2.0;
  request.current_time_steering_rad = 0.0;
  request.current_steering_rad = 0.0;
  request.current_response_steering_rad = 0.0;
  request.previous_published_steering_rad = 0.0;
  request.obstacles.obstacles.push_back({"rear", {49.60, 0.0, 1.5, 0.0, 0.05}});

  const auto result = retained::evaluate(request);
  ASSERT_EQ(result.reason, retained::Reason::Accepted);
  ASSERT_TRUE(result.proof.has_value());
  EXPECT_EQ(result.static_wall_scope, retained::StaticWallProofScope::FullSuffix);
  EXPECT_EQ(result.dynamic_obstacle_scope, retained::DynamicObstacleProofScope::FullSuffix);
  EXPECT_TRUE(result.terminal_stop_certified);
  EXPECT_TRUE(result.terminal_stop_uses_solved_suffix);
  EXPECT_DOUBLE_EQ(result.terminal_stop_publisher_interval_end_steering_rad, 0.0);
  EXPECT_DOUBLE_EQ(result.terminal_stop_final_steering_rad, 0.0);
  EXPECT_EQ(result.proof->terminal_stop_trajectory.elapsed_time_sec,
    result.proof->continuation_trajectory.elapsed_time_sec);
  EXPECT_EQ(result.proof->terminal_stop_trajectory.velocity_mps,
    result.proof->continuation_trajectory.velocity_mps);
  ASSERT_GT(result.proof->terminal_stop_publisher_interval_sample_count, 0U);
  EXPECT_NEAR(result.proof->terminal_stop_actuation_samples[
    result.proof->terminal_stop_publisher_interval_sample_count - 1U].elapsed_time_sec,
    execution->publication_interval_sec, 1e-12);
  EXPECT_NEAR(result.proof->terminal_stop_trajectory.velocity_mps.back(), 0.0, 1e-8);
  const auto authority = production::build(result);
  ASSERT_TRUE(authority.authority.has_value());
  EXPECT_DOUBLE_EQ(authority.authority->command.acceleration_mps2, 1.0);

  // Current-state changes must be replayed. In the absorbing rest model,
  // nearby speeds can still end at rest; they are valid only after fresh proof.
  for (const double speed : {1.95, 2.05}) {
    auto changed = request;
    changed.current_speed_mps = speed;
    changed.control_origin_speed_mps = speed;
    const auto checked = retained::evaluate(changed);
    EXPECT_EQ(checked.reason, retained::Reason::Accepted);
    EXPECT_TRUE(checked.terminal_stop_uses_solved_suffix);
    ASSERT_TRUE(checked.proof);
    EXPECT_DOUBLE_EQ(checked.proof->terminal_stop_trajectory.velocity_mps.back(), 0.0);
  }
  auto slower = request;
  slower.current_speed_mps = 1.5;
  slower.control_origin_speed_mps = 1.5;
  EXPECT_NE(retained::evaluate(slower).reason, retained::Reason::Accepted);
  auto faster_peer = request;
  faster_peer.obstacles.obstacles.front().circle.velocity_x_mps = 2.2;
  EXPECT_NE(retained::evaluate(faster_peer).reason, retained::Reason::Accepted);
  auto residual_motion = request;
  residual_motion.current_speed_mps = 3.0;
  residual_motion.control_origin_speed_mps = 3.0;
  EXPECT_FALSE(retained::evaluate(residual_motion).terminal_stop_uses_solved_suffix);
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

retained::Request bind_test_packet(
  retained::Request request,
  const multi_purpose_mpc_ros::mpcc_vehicle_model::PublishedCommand & packet)
{
  namespace vehicle = multi_purpose_mpc_ros::mpcc_vehicle_model;
  const auto & parameters = request.plan->execution_artifact->vehicle_model;
  vehicle::ObservationProvenance observed{
    {request.now_sec,
      {request.control_pose.x_m, request.control_pose.y_m, request.control_pose.yaw_rad,
        request.current_speed_mps, 0, 0, request.previous_published_steering_rad,
        request.current_response_steering_rad}},
    request.now_sec, request.now_sec, request.now_sec,
    request.now_sec, request.control_origin_sec, 0, .02,
    {{0, -3, request.previous_published_steering_rad * parameters.steering_wire_gain}}};
  const auto predicted = vehicle::predict_prospective_publication(observed, packet, parameters);
  if (!predicted) throw std::runtime_error("invalid synthetic publication fixture");
  const auto & origin = predicted->control_origin;
  request.publication_prefix_required = true;
  request.publication_prefix = predicted;
  request.control_pose = {origin.x_m, origin.y_m, origin.yaw_rad};
  request.control_origin_physical_progress_m = origin.x_m;
  request.current_speed_mps = predicted->current.forward_velocity_mps;
  request.control_origin_speed_mps = origin.forward_velocity_mps;
  request.current_steering_rad = origin.desired_steering_rad;
  request.current_response_steering_rad = origin.tire_steering_rad;
  request.current_lateral_velocity_mps = origin.lateral_velocity_mps;
  request.current_yaw_rate_radps = origin.yaw_rate_radps;
  request.measured_to_control_path.clear();
  request.measured_to_control_elapsed_sec.clear();
  for (const auto & item : predicted->current_to_control) {
    request.measured_to_control_path.push_back({item.state.x_m, item.state.y_m, item.state.yaw_rad});
    request.measured_to_control_elapsed_sec.push_back(item.source_sec - request.now_sec);
  }
  return request;
}

TEST(MpccRateResolvedRetainedRevalidation, NormalPacketOwnsItsPrefixAcrossStageAdvance)
{
  auto original = accepted_request(certified_plan());
  original.control_origin_sec = 1.09;
  const auto packet = retained::prospective_artifact_packet(original);
  ASSERT_TRUE(packet);
  auto request = bind_test_packet(original, *packet);
  auto result = retained::evaluate(request);
  ASSERT_EQ(result.reason, retained::Reason::Accepted);
  ASSERT_TRUE(result.proof);
  EXPECT_TRUE(result.proof->publication_stage_advanced);
  EXPECT_TRUE(production::build(result).authority);

  auto brake = *packet;
  brake.wire_acceleration_mps2 = -3;
  const auto mismatched = retained::evaluate(bind_test_packet(original, brake));
  EXPECT_EQ(mismatched.reason, retained::Reason::PublicationPacketMismatch);
  EXPECT_FALSE(production::build(mismatched).authority);

  auto missing = request;
  missing.publication_prefix.reset();
  EXPECT_EQ(retained::evaluate(missing).reason, retained::Reason::PublicationPrefixUnavailable);
  auto changed_path = request;
  changed_path.measured_to_control_path[1].x_m += .1;
  EXPECT_EQ(retained::evaluate(changed_path).reason, retained::Reason::PublicationPrefixUnavailable);
  result.proof->publication_prefix->proposed_packet.wire_acceleration_mps2 = -3;
  EXPECT_FALSE(production::build(result).authority);
}

TEST(MpccRateResolvedRetainedRevalidation, StopPacketPrefixSurvivesMaterializationAndPublication)
{
  auto original = accepted_request(certified_plan());
  original.control_origin_sec = 1.09;
  const auto & parameters = original.plan->execution_artifact->vehicle_model;
  auto request = bind_test_packet(original,
    {original.now_sec, -3, static_cast<float>(original.previous_published_steering_rad * parameters.steering_wire_gain)});
  const auto stop = retained::evaluate_stop_successor(request);
  ASSERT_TRUE(stop.accepted()) << retained::to_string(stop.reason);
  const auto built = stop_bundle::build(request, stop, 1001U);
  ASSERT_TRUE(built.plan) << stop_bundle::to_string(built.reason);
  request.plan = built.plan;
  request.execution_clock = {retained::ExecutionClockKind::TimeAlignedCandidate, NAN, NAN};
  const auto joined = retained::evaluate(request);
  ASSERT_EQ(joined.reason, retained::Reason::Accepted);
  const auto output = production::build(joined);
  ASSERT_TRUE(output.authority);
  EXPECT_DOUBLE_EQ(output.authority->command.acceleration_mps2, -3);
  EXPECT_TRUE(multi_purpose_mpc_ros::mpcc_vehicle_model::publication_packet_matches(
    *request.publication_prefix, request.now_sec, output.authority->command.acceleration_mps2,
    output.authority->command.steering_tire_angle_rad, parameters.steering_wire_gain));
  auto mutated = joined;
  mutated.proof->publication_prefix->proposed_packet.wire_steering_rad += .02;
  EXPECT_FALSE(production::build(mutated).authority);
}

TEST(MpccRateResolvedRetainedRevalidation, PublishedStopKeepsItsSourceWhenNormalIntentChanges)
{
  auto original = accepted_request(certified_plan(free_grid(), contract::ControlIntent::ShiftOut));
  original.current_intent = contract::ControlIntent::Cruise;
  original.control_origin_sec = 1.09;
  const auto & model = original.plan->execution_artifact->vehicle_model;
  auto request = bind_test_packet(original, {original.now_sec, -3,
    static_cast<float>(original.previous_published_steering_rad * model.steering_wire_gain)});
  EXPECT_EQ(retained::evaluate(request).reason, retained::Reason::IntentMismatch);
  EXPECT_EQ(retained::evaluate_stop_successor(request).reason, retained::StopSuccessorReason::InvalidIdentity);
  const auto stopped = retained::evaluate_published_stop_successor(request);
  ASSERT_TRUE(stopped.result.accepted()) << retained::to_string(stopped.result.reason);
  ASSERT_TRUE(stopped.request);
  EXPECT_EQ(request.current_intent, contract::ControlIntent::Cruise);
  EXPECT_EQ(stopped.request->current_intent, contract::ControlIntent::ShiftOut);
  EXPECT_EQ(stopped.request->plan, request.plan);
  EXPECT_EQ(stopped.request->current_wall_grid, request.current_wall_grid);
  const auto built = stop_bundle::build(*stopped.request, stopped.result, 1001U);
  ASSERT_TRUE(built.plan) << stop_bundle::to_string(built.reason);
  ASSERT_TRUE(built.plan->execution_artifact->terminal_body_rest_required);
  EXPECT_DOUBLE_EQ(built.plan->execution_artifact->predicted_states.back().velocity_mps, 0);
  EXPECT_DOUBLE_EQ(built.plan->execution_artifact->predicted_states.back().lateral_velocity_mps, 0);
  EXPECT_DOUBLE_EQ(built.plan->execution_artifact->predicted_states.back().yaw_rate_radps, 0);
  auto joined_request = *stopped.request;
  joined_request.plan = built.plan;
  joined_request.execution_clock = {retained::ExecutionClockKind::TimeAlignedCandidate, NAN, NAN};
  const auto joined = retained::evaluate(joined_request);
  ASSERT_EQ(joined.reason, retained::Reason::Accepted);
  const auto output = production::build(joined);
  ASSERT_TRUE(output.authority);
  EXPECT_DOUBLE_EQ(output.authority->command.acceleration_mps2, -3);
  EXPECT_EQ(output.authority->command.intent, contract::ControlIntent::ShiftOut);
  EXPECT_EQ(contract::resolve_published_authority_intent(output.authority->command.intent, true),
    contract::ControlIntent::Stop);
  EXPECT_TRUE(multi_purpose_mpc_ros::mpcc_vehicle_model::publication_packet_matches(
    *joined_request.publication_prefix, joined_request.now_sec, output.authority->command.acceleration_mps2,
    output.authority->command.steering_tire_angle_rad, model.steering_wire_gain));
  auto noncausal = request;
  noncausal.execution_clock.kind = retained::ExecutionClockKind::TimeAlignedCandidate;
  EXPECT_FALSE(retained::evaluate_published_stop_successor(noncausal).request);
  noncausal = request;
  noncausal.execution_clock.first_published_control_origin_sec = request.control_origin_sec+.01;
  EXPECT_FALSE(retained::evaluate_published_stop_successor(noncausal).request);
  auto missing = request;
  missing.publication_prefix.reset();
  EXPECT_EQ(retained::evaluate_published_stop_successor(missing).result.reason,
    retained::StopSuccessorReason::InvalidCurrentWorld);
  auto changed_wall = request;
  changed_wall.current_footprint.front_extent_m += .1;
  EXPECT_EQ(retained::evaluate_published_stop_successor(changed_wall).result.reason,
    retained::StopSuccessorReason::StaticWorldMismatch);
}


namespace applied = multi_purpose_mpc_ros::mpcc_rate_resolved_applied_program;
namespace stop_program = multi_purpose_mpc_ros::mpcc_stop_input_program;
namespace vehicle = multi_purpose_mpc_ros::mpcc_vehicle_model;

retained::Request applied_request(const contract::ControlIntent intent = contract::ControlIntent::Track)
{
  auto original = intent == contract::ControlIntent::Follow ? accepted_follow_request() : accepted_request(certified_plan(free_grid(), intent));
  original.current_intent = intent;
  original.control_origin_sec = 1.09;
  const auto packet = retained::prospective_artifact_packet(original);
  if (!packet) throw std::runtime_error("no fixture packet");
  auto request = bind_test_packet(original, *packet);
  // Original fixture's lone time-zero packet is nominal history only. Supply
  // actual serialized packets covering the complete declared receiver window.
  auto observation = request.publication_prefix->observation;
  observation.commands.clear();
  for (int i = 0; i <= 40; ++i) observation.commands.push_back(
    {i * .025, -3, static_cast<float>(original.previous_published_steering_rad *
      original.plan->execution_artifact->vehicle_model.steering_wire_gain)});
  request.publication_prefix = vehicle::predict_prospective_publication(
    observation, *packet, original.plan->execution_artifact->vehicle_model);
  if (!request.publication_prefix) throw std::runtime_error("invalid complete fixture history");
  const auto & predicted = *request.publication_prefix;
  const auto & origin = predicted.control_origin;
  request.control_pose = {origin.x_m, origin.y_m, origin.yaw_rad};
  request.control_origin_physical_progress_m = origin.x_m;
  request.current_speed_mps = predicted.current.forward_velocity_mps;
  request.control_origin_speed_mps = origin.forward_velocity_mps;
  request.current_steering_rad = origin.desired_steering_rad;
  request.current_response_steering_rad = origin.tire_steering_rad;
  request.current_lateral_velocity_mps = origin.lateral_velocity_mps;
  request.current_yaw_rate_radps = origin.yaw_rate_radps;
  request.measured_to_control_path.clear(); request.measured_to_control_elapsed_sec.clear();
  for (const auto & item : predicted.current_to_control) {
    request.measured_to_control_path.push_back({item.state.x_m, item.state.y_m, item.state.yaw_rad});
    request.measured_to_control_elapsed_sec.push_back(item.source_sec - request.now_sec);
  }
  return request;
}

TEST(MpccRateResolvedRetainedRevalidation, AppliedStopBindsOriginalProgramSourceAndCurrentWorld)
{
  const auto request = applied_request();
  const auto nominal = retained::evaluate(request);
  ASSERT_TRUE(nominal.proof) << retained::to_string(nominal.reason);
  const vehicle::InputApplicationProfile profile{"test-receiver", .25, .25, .02};
  const auto result = applied::certify_terminal_stop(request, *nominal.proof, profile);
  ASSERT_TRUE(result.certificate) << static_cast<int>(result.reason) << '/' <<
    static_cast<int>(result.program_reason) << '/' << static_cast<int>(result.prediction_reason);
  const auto & proof = *result.certificate;
  EXPECT_TRUE(proof.matches(request)); EXPECT_TRUE(proof.matches(*nominal.proof));
  EXPECT_GT(proof.tube().rest_sec, request.now_sec + .25);
  EXPECT_DOUBLE_EQ(proof.prepared().nominal_control_origin_sec, request.control_origin_sec);
  EXPECT_DOUBLE_EQ(proof.prepared().program.commands.front().published_sec, request.now_sec);
  EXPECT_DOUBLE_EQ(proof.prepared().program.commands.front().wire_acceleration_mps2,
    request.publication_prefix->proposed_packet.wire_acceleration_mps2);
  auto other = request; other.obstacles.generation++; EXPECT_FALSE(proof.matches(other));
  other = request; other.minimum_acceleration_mps2 -= .1; EXPECT_FALSE(proof.matches(other));
  other = request; other.publication_prefix->observation.commands.back().wire_acceleration_mps2 = 1;
  EXPECT_FALSE(proof.matches(other));
  auto changed = *nominal.proof;
  changed.terminal_stop_actuation_samples.back().acceleration_mps2 = -2;
  EXPECT_FALSE(proof.matches(changed));
  changed = *nominal.proof; changed.actuation.steering_rad += .001;
  EXPECT_FALSE(proof.matches(changed));
}

TEST(MpccRateResolvedRetainedRevalidation, AppliedStopRejectsMissingCoverageAndActualPeerOverlap)
{
  auto request = applied_request();
  const auto nominal = retained::evaluate(request);
  ASSERT_TRUE(nominal.proof);
  const vehicle::InputApplicationProfile profile{"test-receiver", .25, .25, .02};
  auto missing = request; missing.publication_prefix->observation.commands.resize(1);
  auto corresponding = *nominal.proof; corresponding.publication_prefix = missing.publication_prefix;
  const auto unavailable = applied::certify_terminal_stop(missing, corresponding, profile);
  EXPECT_EQ(unavailable.reason, applied::Reason::InputPredictionRejected);
  EXPECT_FALSE(unavailable.certificate);
  const auto & body = request.publication_prefix->observation.initial.state;
  request.obstacles.obstacles.push_back({"new-peer", {body.x_m, body.y_m, 0, 0, .5}});
  // The new world has not earned a nominal certificate; this lower-level
  // negative check still requires the applied physical verifier to reject it.
  const auto collision = applied::certify_terminal_stop(request, *nominal.proof, profile);
  EXPECT_EQ(collision.reason, applied::Reason::PeerRejected);
  EXPECT_FALSE(collision.certificate);
  EXPECT_EQ(collision.rejected_peer_id, "new-peer");
}

TEST(MpccRateResolvedRetainedRevalidation, CommonProgramPreservesDelayedBrakingAndRebasesExplicitly)
{
  stop_program::Request request;
  request.source = execution_artifact().identity;
  request.decision_id = 12;
  request.nominal_control_origin_sec = 1.13;
  request.publication_interval_sec = .025;
  request.first_packet = {1, .5, 0};
  request.steering_wire_gain = 1;
  request.minimum_acceleration_mps2 = -3;
  request.maximum_acceleration_mps2 = 1.37;
  request.maximum_abs_steering_rad = .6;
  request.maximum_abs_steering_rate_radps = 1;
  request.actuator_tolerance = 1e-6;
  for (int i = 0; i < 20; ++i) {
    stop_program::Sample sample;
    sample.elapsed_time_sec = (i + 1) * .005; sample.duration_sec = .005;
    sample.acceleration_mps2 = i < 10 ? .5 : -3;
    sample.steering_rate_radps = 0; sample.end_steering_rad = 0;
    sample.end_velocity_mps = i == 19 ? 0 : .1;
    sample.end_lateral_velocity_mps = sample.end_yaw_rate_radps = 0;
    sample.command_interval_index = i / 5;
    request.nominal_stop_samples.push_back(sample);
  }
  const auto built = stop_program::prepare(request);
  ASSERT_TRUE(built.prepared) << static_cast<int>(built.reason);
  const auto & program = built.prepared->program;
  ASSERT_EQ(program.commands.size(), 3U);
  EXPECT_DOUBLE_EQ(program.commands[0].wire_acceleration_mps2, .5);
  EXPECT_DOUBLE_EQ(program.commands[1].wire_acceleration_mps2, .5);
  EXPECT_DOUBLE_EQ(program.commands[2].wire_acceleration_mps2, -3);
  const auto remaining = stop_program::remaining_program(program, 1.04);
  ASSERT_TRUE(remaining); ASSERT_EQ(remaining->commands.size(), 2U);
  EXPECT_DOUBLE_EQ(remaining->commands[0].published_sec, 1.04);
  EXPECT_DOUBLE_EQ(remaining->commands[0].wire_acceleration_mps2, .5);
  EXPECT_DOUBLE_EQ(remaining->commands[1].published_sec, 1.04 + .025);
  EXPECT_DOUBLE_EQ(remaining->commands[1].wire_acceleration_mps2, -3);
  EXPECT_FALSE(stop_program::remaining_program(program, .99));
  request.first_packet.wire_acceleration_mps2 = -3;
  EXPECT_EQ(stop_program::prepare(request).reason, stop_program::Reason::FirstPacketMismatch);
  request.first_packet.wire_acceleration_mps2 = .5;
  request.nominal_stop_samples.back().end_lateral_velocity_mps = .001;
  EXPECT_EQ(stop_program::prepare(request).reason, stop_program::Reason::NominalRestUnavailable);
}


TEST(MpccRateResolvedRetainedRevalidation, AppliedProgramIsRequiredAtTheCommandBoundary)
{
  auto request = applied_request();
  request.applied_program_required = true;
  EXPECT_FALSE(retained::evaluate(request).proof);
  request.input_application_profile = vehicle::InputApplicationProfile{"test-receiver", .25, .25, .02};
  const auto result = retained::evaluate(request);
  ASSERT_TRUE(result.proof) << retained::to_string(result.reason) << '/' <<
    static_cast<int>(result.applied_program_reason) << '/' <<
    static_cast<int>(result.applied_program_prepare_reason);
  ASSERT_TRUE(result.proof->applied_program);
  EXPECT_TRUE(production::build(result).authority);
  auto removed = result; removed.proof->applied_program.reset();
  EXPECT_FALSE(production::build(removed).authority);
  auto changed = result; changed.proof->terminal_stop_actuation_samples.back().end_steering_rad += .001;
  EXPECT_FALSE(production::build(changed).authority);
  auto different_profile = request;
  different_profile.input_application_profile->steering_receipt_age_sec = .2;
  EXPECT_FALSE(result.proof->applied_program->matches(different_profile));
}

void attach_velocity_source(retained::Request & request, double ceiling)
{
  auto plan = std::make_shared<certified::CertifiedPlan>(*request.plan);
  const auto & execution = *plan->execution_artifact;
  const auto & initial = *execution.semantic_initial_state;
  auto source = std::make_shared<multi_purpose_mpc_ros::mpcc_rate_resolved_shadow::Snapshot>();
  source->identity = execution.identity;
  source->request.vehicle_model = execution.vehicle_model;
  source->request.initial_state << initial.lateral_m, initial.lag_m, initial.heading_offset_rad,
    initial.velocity_mps, initial.progress_m;
  source->request.current_steering_rad = initial.steering_rad;
  source->request.current_response_steering_rad = initial.response_steering_rad;
  source->request.current_lateral_velocity_mps = initial.lateral_velocity_mps;
  source->request.current_yaw_rate_radps = initial.yaw_rate_radps;
  source->request.states.resize(execution.control_stages.size() + 1U);
  for (auto & state : source->request.states) state.upper(3) = ceiling;
  plan->solver_source_snapshot = source;
  request.plan = plan;
}

retained::Request source_horizon_request()
{
  auto request = applied_request();
  request.applied_program_required = true;
  request.input_application_profile = vehicle::InputApplicationProfile{"test-receiver", .25, .25, .02};
  attach_velocity_source(request, 4);
  // A rear peer will approach the current footprint after nominal braking.
  // It remains behind the moving, fully certified vehicle in this free world.
  const auto & current = request.publication_prefix->current;
  const auto & model = request.plan->execution_artifact->vehicle_model;
  const auto next = vehicle::advance(current, {-3, 0}, model, model.maximum_step_sec);
  if (!next) throw std::runtime_error("invalid ranking fixture");
  const double stop_time = current.forward_velocity_mps * model.maximum_step_sec /
    (current.forward_velocity_mps - next->state.forward_velocity_mps);
  const double future = request.control_origin_sec - request.obstacles.observed_sec + stop_time + .25;
  request.obstacles.obstacles.push_back({"rear-approach",
    {request.control_pose.x_m - .5, request.control_pose.y_m, .5 / future, 0, .01}});
  return request;
}

TEST(MpccRateResolvedRetainedRevalidation, SourceHorizonUsesDenseOriginalPacketsAndCarriesVelocityCeiling)
{
  auto request = source_horizon_request();
  ASSERT_EQ(certified::validate(*request.plan), certified::RejectReason::None);
  const auto result = retained::evaluate(request);
  ASSERT_TRUE(result.proof && result.proof->applied_program) << retained::to_string(result.reason);
  ASSERT_TRUE(result.terminal_stop_source_horizon_program);
  EXPECT_EQ(result.terminal_stop_reference_attempts, 1U);
  EXPECT_EQ(result.proof->terminal_stop_forward_velocity_ceiling_mps, std::optional<double>{4});
  const auto & programme = result.proof->applied_program->prepared().program;
  ASSERT_GT(programme.commands.size(), 2U);
  const auto & first = request.publication_prefix->proposed_packet;
  const double period = request.plan->execution_artifact->publication_interval_sec;
  for (std::size_t i = 0; i < programme.commands.size(); ++i) {
    const auto & packet = programme.commands[i];
    EXPECT_DOUBLE_EQ(packet.published_sec, first.published_sec + i * period);
    EXPECT_DOUBLE_EQ(packet.wire_steering_rad, first.wire_steering_rad);
    EXPECT_DOUBLE_EQ(packet.wire_acceleration_mps2,
      i + 1 == programme.commands.size() ? -3 : first.wire_acceleration_mps2);
  }
  const double prefix = programme.commands.back().published_sec - first.published_sec;
  // The normal proof keeps its authorized publisher interval. The separately
  // certified terminal programme must cover every later command through rest.
  EXPECT_LE(result.proof->continuation_trajectory.elapsed_time_sec.back(), period + 1e-12);
  EXPECT_GT(result.proof->terminal_stop_trajectory.elapsed_time_sec.back(), prefix);
  EXPECT_GT(result.proof->applied_program->tube().rest_sec, request.now_sec + prefix);
  const auto & execution = *request.plan->execution_artifact;
  double horizon = -result.proof->cursor.stage_elapsed_sec;
  for (std::size_t i = result.proof->cursor.control_stage_index; i < execution.control_stages.size(); ++i)
    horizon += execution.control_stages[i].duration_sec;
  EXPECT_LE(prefix, horizon + 1e-12);
  EXPECT_GT(prefix + period, horizon);
  EXPECT_TRUE(production::build(result).authority);
  auto changed = result;
  changed.proof->applied_program.reset();
  EXPECT_FALSE(production::build(changed).authority);
  changed = result;
  changed.proof->terminal_stop_trajectory.elapsed_time_sec.back() = prefix;
  EXPECT_FALSE(production::build(changed).authority);
  changed = result;
  changed.proof->terminal_stop_forward_velocity_ceiling_mps.reset();
  EXPECT_FALSE(production::build(changed).authority);
  changed = result; changed.proof->terminal_stop_source_horizon_program = false;
  EXPECT_FALSE(production::build(changed).authority);
  auto other = request; attach_velocity_source(other, 5);
  EXPECT_FALSE(result.proof->applied_program->matches(other));
  const auto stop = stop_bundle::build_certified_terminal(request, result, 20002U);
  ASSERT_TRUE(stop.plan);
  ASSERT_TRUE(stop.plan->execution_artifact->applied_stop_program);
  EXPECT_EQ(stop.plan->execution_artifact->applied_stop_program->forward_velocity_ceiling_mps,
    std::optional<double>{4});
  request.plan = stop.plan;
  request.execution_clock = {retained::ExecutionClockKind::TimeAlignedCandidate, NAN, NAN};
  const auto joined = retained::evaluate(request);
  ASSERT_TRUE(joined.proof && joined.proof->applied_program) << retained::to_string(joined.reason);
  EXPECT_TRUE(joined.terminal_stop_uses_solved_suffix);
  EXPECT_EQ(joined.proof->terminal_stop_forward_velocity_ceiling_mps, std::optional<double>{4});
  EXPECT_TRUE(production::build(joined).authority);
  const auto successor = stop_bundle::build_certified_terminal(request, joined, 20003U);
  ASSERT_TRUE(successor.plan);
  EXPECT_EQ(successor.plan->execution_artifact->applied_stop_program->forward_velocity_ceiling_mps,
    std::optional<double>{4});
}

TEST(MpccRateResolvedRetainedRevalidation, SourceHorizonRejectsMissingMismatchedAndInvalidBounds)
{
  auto request = source_horizon_request();
  EXPECT_EQ(retained::source_horizon_velocity_ceiling(request), std::optional<double>{4});
  auto plan = std::make_shared<certified::CertifiedPlan>(*request.plan);
  auto source = std::make_shared<multi_purpose_mpc_ros::mpcc_rate_resolved_shadow::Snapshot>(*plan->solver_source_snapshot);
  plan->solver_source_snapshot = source; request.plan = plan;
  source->request.states.back().upper(3) = 3;
  EXPECT_EQ(retained::source_horizon_velocity_ceiling(request), std::optional<double>{3});
  source->request.states.back().upper(3) = NAN;
  EXPECT_FALSE(retained::source_horizon_velocity_ceiling(request));
  source->request.states.back().upper(3) = -1;
  EXPECT_FALSE(retained::source_horizon_velocity_ceiling(request));
  source->request.states.back().upper(3) = 4;
  source->identity.sequence++;
  EXPECT_FALSE(retained::source_horizon_velocity_ceiling(request));
  source->identity.sequence--;
  source->request.current_steering_rad += .1;
  EXPECT_FALSE(retained::source_horizon_velocity_ceiling(request));
  plan->solver_source_snapshot.reset();
  EXPECT_FALSE(retained::source_horizon_velocity_ceiling(request));
  const auto legacy = retained::evaluate(request);
  ASSERT_TRUE(legacy.proof);
  EXPECT_FALSE(legacy.terminal_stop_source_horizon_program);
}

TEST(MpccRateResolvedRetainedRevalidation, SourceHorizonCeilingChecksTheWholeAppliedResponse)
{
  auto request = source_horizon_request();
  const auto result = retained::evaluate(request);
  ASSERT_TRUE(result.proof && result.proof->applied_program);
  auto nominal = *result.proof;
  // A lower source ceiling still covers the nominal terminal trajectory but
  // cannot cover the complete receipt-response population at publication.
  double nominal_peak = request.control_origin_speed_mps;
  for (double value : nominal.terminal_stop_trajectory.velocity_mps) nominal_peak = std::max(nominal_peak, value);
  double applied_peak = 0;
  for (const auto & sample : result.proof->applied_program->tube().source_to_rest)
    applied_peak = std::max(applied_peak, sample.swept_body[3].upper);
  ASSERT_GT(applied_peak, nominal_peak);
  const double ceiling = (nominal_peak + applied_peak) / 2;
  attach_velocity_source(request, ceiling);
  nominal.plan = request.plan;
  nominal.terminal_stop_forward_velocity_ceiling_mps = ceiling;
  const auto rejected = applied::certify_terminal_stop(request, nominal, *request.input_application_profile);
  EXPECT_FALSE(rejected.certificate);
  EXPECT_EQ(rejected.reason, applied::Reason::StateBoundRejected);
  EXPECT_EQ(rejected.prediction_reason, vehicle::AppliedInputRejectReason::ValidationRejected);
  nominal.terminal_stop_forward_velocity_ceiling_mps = ceiling + 1;
  EXPECT_EQ(applied::certify_terminal_stop(request, nominal, *request.input_application_profile).reason,
    applied::Reason::InvalidNominalProof);
}

TEST(MpccRateResolvedRetainedRevalidation, AppliedProvenanceVersionsBindAndValidateVelocityCeiling)
{
  auto request = source_horizon_request();
  const auto result = retained::evaluate(request);
  ASSERT_TRUE(result.proof);
  const auto stop = stop_bundle::build_certified_terminal(request, result, 20004U);
  ASSERT_TRUE(stop.plan);
  const auto & execution = *stop.plan->execution_artifact;
  auto provenance = *execution.applied_stop_program;
  // Exercise the retained v1/v2 wire format explicitly. Current live proofs
  // have a nonzero publication window, covered separately by the v3 test.
  provenance.program.maximum_publication_delay_sec = 0;
  provenance.program.nanosecond_clock.reset();
  for (std::size_t i = 0; i < provenance.program.commands.size(); ++i)
    provenance.program.commands[i].published_sec = provenance.observation.now_sec +
      i * provenance.program.publication_interval_sec;
  YAML::Emitter encoded; encoded.SetDoublePrecision(17);
  encoded << vehicle::encode_applied_program_provenance(provenance);
  auto node = YAML::Load(encoded.c_str());
  EXPECT_EQ(node["schema"].as<std::string>(), "applied-stop-provenance-v2");
  const auto decoded = vehicle::decode_applied_program_provenance(node, execution.vehicle_model);
  ASSERT_TRUE(decoded);
  const auto fingerprint = vehicle::applied_program_provenance_fingerprint(provenance, execution.vehicle_model);
  EXPECT_EQ(vehicle::applied_program_provenance_fingerprint(*decoded, execution.vehicle_model), fingerprint);
  node["forward_velocity_ceiling_mps"] = -1;
  EXPECT_FALSE(vehicle::decode_applied_program_provenance(node, execution.vehicle_model));
  node.remove("forward_velocity_ceiling_mps");
  EXPECT_FALSE(vehicle::decode_applied_program_provenance(node, execution.vehicle_model));
  node["forward_velocity_ceiling_mps"] = 4;
  node["schema"] = "applied-stop-provenance-v1";
  EXPECT_FALSE(vehicle::decode_applied_program_provenance(node, execution.vehicle_model));
  node.remove("forward_velocity_ceiling_mps");
  const auto legacy = vehicle::decode_applied_program_provenance(node, execution.vehicle_model);
  ASSERT_TRUE(legacy); EXPECT_FALSE(legacy->forward_velocity_ceiling_mps);
  EXPECT_NE(vehicle::applied_program_provenance_fingerprint(*legacy, execution.vehicle_model), fingerprint);
  node["schema"] = "applied-stop-provenance-v3";
  EXPECT_FALSE(vehicle::decode_applied_program_provenance(node, execution.vehicle_model));
}

TEST(MpccRateResolvedRetainedRevalidation, CommonTerminalCandidatePreservesFirstPacketAndCompleteStop)
{
  auto request = applied_request();
  request.applied_program_required = true;
  request.input_application_profile = vehicle::InputApplicationProfile{"test-receiver", .25, .25, .02};
  const auto result = retained::evaluate(request);
  ASSERT_TRUE(result.proof && result.proof->applied_program);
  ASSERT_TRUE(result.terminal_stop_constant_steering_program);
  ASSERT_TRUE(production::build(result).authority);
  const auto & expected = request.publication_prefix->proposed_packet;
  const auto & programme = result.proof->applied_program->prepared().program;
  ASSERT_GT(programme.commands.size(), 1U);
  EXPECT_DOUBLE_EQ(programme.commands.front().published_sec, expected.published_sec);
  EXPECT_DOUBLE_EQ(programme.commands.front().wire_acceleration_mps2, expected.wire_acceleration_mps2);
  for (const auto & packet : programme.commands)
    EXPECT_DOUBLE_EQ(packet.wire_steering_rad, expected.wire_steering_rad);
  EXPECT_DOUBLE_EQ(programme.commands.back().wire_acceleration_mps2, request.minimum_acceleration_mps2);
  EXPECT_GT(result.proof->applied_program->tube().rest_sec, programme.commands.back().published_sec);
  auto changed = result;
  changed.proof->terminal_stop_constant_steering_program = false;
  EXPECT_FALSE(production::build(changed).authority);
  const auto stop = stop_bundle::build_certified_terminal(request, result, 20001U);
  ASSERT_TRUE(stop.plan);
  request.plan = stop.plan;
  request.execution_clock = {retained::ExecutionClockKind::TimeAlignedCandidate, NAN, NAN};
  const auto joined = retained::evaluate(request);
  ASSERT_TRUE(joined.proof && joined.proof->applied_program);
  EXPECT_TRUE(joined.terminal_stop_uses_solved_suffix);
  EXPECT_FALSE(joined.terminal_stop_constant_steering_program);
  EXPECT_TRUE(production::build(joined).authority);
}

TEST(MpccRateResolvedRetainedRevalidation, MaterializedAppliedStopPreservesProgramThroughEveryResponseRest)
{
  auto request = applied_request(); request.applied_program_required = true;
  request.input_application_profile = vehicle::InputApplicationProfile{"test-receiver", .25, .25, .02};
  const auto result = retained::evaluate(request);
  ASSERT_TRUE(result.proof) << retained::to_string(result.reason);
  ASSERT_TRUE(result.proof->applied_program);
  const auto materialized = stop_bundle::build_certified_terminal(request, result, 1001U);
  ASSERT_TRUE(materialized.plan) << stop_bundle::to_string(materialized.reason) << '/' <<
    stop_bundle::to_string(materialized.actuation_detail);
  const auto & execution = *materialized.plan->execution_artifact;
  ASSERT_TRUE(execution.applied_stop_program);
  EXPECT_NE(execution.identity.source_context.applied_program_fingerprint, 0U);
  EXPECT_EQ(execution.identity.source_context.applied_program_fingerprint,
    vehicle::applied_program_provenance_fingerprint(*execution.applied_stop_program, execution.vehicle_model));
  double duration = 0;
  for (const auto & stage : execution.control_stages) duration += stage.duration_sec;
  EXPECT_GE(duration + 1e-9, result.proof->applied_program->tube().rest_sec - request.now_sec);
  auto joined_request = request; joined_request.plan = materialized.plan;
  joined_request.execution_clock = {retained::ExecutionClockKind::TimeAlignedCandidate, NAN, NAN};
  const auto joined = retained::evaluate(joined_request);
  ASSERT_TRUE(joined.proof) << retained::to_string(joined.reason) << '/' <<
    static_cast<int>(joined.applied_program_reason) << '/' << static_cast<int>(joined.applied_program_prepare_reason);
  ASSERT_TRUE(joined.proof->applied_program);
  EXPECT_TRUE(joined.terminal_stop_uses_solved_suffix);
  EXPECT_TRUE(production::build(joined).authority);
  EXPECT_DOUBLE_EQ(joined.proof->applied_program->prepared().program.commands.front().wire_steering_rad,
    result.proof->applied_program->prepared().program.commands.front().wire_steering_rad);
  YAML::Emitter encoded;
  encoded.SetDoublePrecision(17);
  encoded << vehicle::encode_applied_program_provenance(*execution.applied_stop_program);
  const auto decoded = vehicle::decode_applied_program_provenance(YAML::Load(encoded.c_str()), execution.vehicle_model);
  ASSERT_TRUE(decoded);
  EXPECT_EQ(vehicle::applied_program_provenance_fingerprint(*decoded, execution.vehicle_model),
    execution.identity.source_context.applied_program_fingerprint);
  auto altered = execution;
  auto changed_program = std::make_shared<vehicle::AppliedProgramProvenance>(*execution.applied_stop_program);
  changed_program->program.commands.back().wire_acceleration_mps2 = -2;
  altered.applied_stop_program = changed_program;
  EXPECT_EQ(artifact::validate(altered), artifact::RejectReason::InvalidIdentity);
  altered = execution; altered.applied_stop_program.reset();
  EXPECT_EQ(artifact::validate(altered), artifact::RejectReason::InvalidIdentity);
}


TEST(MpccRateResolvedRetainedRevalidation, SameDecisionStopJoinReusesIdenticalRangesWithoutRetainingItsParent)
{
  auto request = applied_request(); request.applied_program_required = true;
  request.input_application_profile = vehicle::InputApplicationProfile{"test-receiver", .25, .25, .02};
  const auto original = retained::evaluate(request);
  ASSERT_TRUE(original.proof && original.proof->applied_program);
  const auto & parent = original.proof->applied_program;
  EXPECT_FALSE(parent->reused_numerical_tube());
  const auto materialized = stop_bundle::build_certified_terminal(request, original, 1001U);
  ASSERT_TRUE(materialized.plan);
  auto joined_request = request; joined_request.plan = materialized.plan;
  joined_request.execution_clock = {retained::ExecutionClockKind::TimeAlignedCandidate, NAN, NAN};
  const auto owners = parent.use_count();
  const auto joined = retained::evaluate_materialized_stop(joined_request, *parent);
  ASSERT_TRUE(joined.proof && joined.proof->applied_program) << retained::to_string(joined.reason);
  EXPECT_EQ(parent.use_count(), owners);
  const auto & proof = *joined.proof->applied_program;
  EXPECT_TRUE(proof.reused_numerical_tube());
  EXPECT_TRUE(proof.matches(joined_request)); EXPECT_TRUE(proof.matches(*joined.proof));
  EXPECT_FALSE(proof.matches(request)); EXPECT_FALSE(parent->matches(joined_request));
  EXPECT_TRUE(production::build(joined).authority);
  const auto fresh = retained::evaluate(joined_request);
  ASSERT_TRUE(fresh.proof && fresh.proof->applied_program);
  EXPECT_FALSE(fresh.proof->applied_program->reused_numerical_tube());
  const auto & a = proof.tube(); const auto & b = fresh.proof->applied_program->tube();
  EXPECT_EQ(a.context_fingerprint, b.context_fingerprint);
  EXPECT_DOUBLE_EQ(a.rest_sec, b.rest_sec);
  EXPECT_EQ(proof.checked_samples(), fresh.proof->applied_program->checked_samples());
  EXPECT_DOUBLE_EQ(proof.minimum_peer_clearance_m(), fresh.proof->applied_program->minimum_peer_clearance_m());
  const auto same_ranges = [](const auto & x, const auto & y) {
    for (std::size_t i = 0; i < x.size(); ++i) {
      EXPECT_DOUBLE_EQ(x[i].lower, y[i].lower); EXPECT_DOUBLE_EQ(x[i].upper, y[i].upper);
    }
  };
  same_ranges(a.publication_body, b.publication_body);
  ASSERT_TRUE(a.publication_footprint && b.publication_footprint);
  same_ranges(*a.publication_footprint, *b.publication_footprint);
  ASSERT_EQ(a.source_to_rest.size(), b.source_to_rest.size());
  for (std::size_t i = 0; i < a.source_to_rest.size(); ++i) {
    const auto & x = a.source_to_rest[i]; const auto & y = b.source_to_rest[i];
    EXPECT_DOUBLE_EQ(x.begin_sec, y.begin_sec); EXPECT_DOUBLE_EQ(x.end_sec, y.end_sec);
    same_ranges(x.swept_body, y.swept_body); same_ranges(x.endpoint_body, y.endpoint_body);
    ASSERT_TRUE(x.swept_footprint && y.swept_footprint && x.endpoint_footprint && y.endpoint_footprint);
    same_ranges(*x.swept_footprint, *y.swept_footprint);
    same_ranges(*x.endpoint_footprint, *y.endpoint_footprint);
  }
  // A later decision must independently predict, even with identical inputs.
  auto later = joined_request; ++later.decision_id;
  const auto next = retained::evaluate_materialized_stop(later, *parent);
  ASSERT_TRUE(next.proof && next.proof->applied_program) << retained::to_string(next.reason);
  EXPECT_FALSE(next.proof->applied_program->reused_numerical_tube());
  EXPECT_TRUE(production::build(next).authority);
}

TEST(MpccRateResolvedRetainedRevalidation, SameDecisionStopJoinCannotHideMissingHistoryOrNewPeer)
{
  auto request = applied_request(); request.applied_program_required = true;
  request.input_application_profile = vehicle::InputApplicationProfile{"test-receiver", .25, .25, .02};
  const auto original = retained::evaluate(request);
  ASSERT_TRUE(original.proof && original.proof->applied_program);
  const auto materialized = stop_bundle::build_certified_terminal(request, original, 1001U);
  ASSERT_TRUE(materialized.plan);
  request.plan = materialized.plan;
  request.execution_clock = {retained::ExecutionClockKind::TimeAlignedCandidate, NAN, NAN};
  const auto joined = retained::evaluate(request);
  ASSERT_TRUE(joined.proof);
  const auto * parent = original.proof->applied_program.get();
  auto missing = request; missing.publication_prefix->observation.commands.resize(1);
  auto nominal = *joined.proof; nominal.publication_prefix = missing.publication_prefix;
  const auto gap = applied::certify_terminal_stop(missing, nominal, *request.input_application_profile, parent);
  EXPECT_EQ(gap.prediction_reason, vehicle::AppliedInputRejectReason::HistoryUnavailable);
  EXPECT_FALSE(gap.certificate);
  auto collision = request;
  const auto & body = request.publication_prefix->observation.initial.state;
  collision.obstacles.obstacles.push_back({"new-peer", {body.x_m, body.y_m, 0, 0, .5}});
  // Lower-level negatives do not mint a nominal proof for a changed world.
  const auto peer = applied::certify_terminal_stop(collision, *joined.proof,
    *request.input_application_profile, parent);
  EXPECT_EQ(peer.reason, applied::Reason::PeerRejected); EXPECT_FALSE(peer.certificate);
  auto changed_profile = *request.input_application_profile; changed_profile.acceleration_age_sec = .2;
  auto changed = request; changed.input_application_profile = changed_profile;
  const auto independent = applied::certify_terminal_stop(changed, *joined.proof, changed_profile, parent);
  ASSERT_TRUE(independent.certificate) << static_cast<int>(independent.reason);
  EXPECT_FALSE(independent.certificate->reused_numerical_tube());
}

TEST(MpccRateResolvedRetainedRevalidation, AppliedStopChecksFollowGapAcrossTheWholeUncertainBodyPath)
{
  auto request = applied_request(contract::ControlIntent::Follow);
  const auto nominal = retained::evaluate(request);
  ASSERT_TRUE(nominal.proof) << retained::to_string(nominal.reason);
  const vehicle::InputApplicationProfile profile{"test-follow-receiver", .25, .25, .02};
  const auto accepted = applied::certify_terminal_stop(request, *nominal.proof, profile);
  ASSERT_TRUE(accepted.certificate) << static_cast<int>(accepted.reason);
  EXPECT_GE(accepted.certificate->minimum_follow_gap_m(), request.follow_target->hard_gap_m);
  auto unsafe = request;
  unsafe.follow_target->hard_gap_m = 10;
  const auto rejected = applied::certify_terminal_stop(unsafe, *nominal.proof, profile);
  EXPECT_EQ(rejected.reason, applied::Reason::FollowGapRejected);
  EXPECT_FALSE(rejected.certificate);
  EXPECT_FALSE(accepted.certificate->matches(unsafe));
}

}  // namespace

TEST(MpccRateResolvedRetainedRevalidation, SignedObservationNeedsTheCompleteAppliedForwardStopProof)
{
  const auto source = certified_plan();
  const auto & model = source->execution_artifact->vehicle_model;
  const double tolerance = source->execution_artifact->physical_global_tolerance;
  ASSERT_GT(tolerance, 0);
  const auto make_stop = [&](const double measured_u) {
    auto initial = accepted_request(source);
    initial.current_speed_mps = measured_u;
    initial.control_origin_sec = initial.now_sec + .13;
    auto request = bind_test_packet(initial, {initial.now_sec, -3,
      multi_purpose_mpc_ros::mpcc_wire_command::steering(initial.previous_published_steering_rad,
        model.steering_wire_gain)});
    auto observation = request.publication_prefix->observation;
    observation.commands.clear();
    for (int i = 0; i <= 40; ++i) observation.commands.push_back(
      {i * .025, -3, request.publication_prefix->proposed_packet.wire_steering_rad});
    request.publication_prefix = vehicle::predict_prospective_publication(
      observation, request.publication_prefix->proposed_packet, model);
    const auto & prediction = *request.publication_prefix;
    const auto & origin = prediction.control_origin;
    request.control_pose = {origin.x_m, origin.y_m, origin.yaw_rad};
    request.control_origin_physical_progress_m = origin.x_m;
    request.current_speed_mps = prediction.current.forward_velocity_mps;
    request.current_time_steering_rad = prediction.current.tire_steering_rad;
    request.control_origin_speed_mps = origin.forward_velocity_mps;
    request.current_steering_rad = origin.desired_steering_rad;
    request.current_response_steering_rad = origin.tire_steering_rad;
    request.current_lateral_velocity_mps = origin.lateral_velocity_mps;
    request.current_yaw_rate_radps = origin.yaw_rate_radps;
    request.measured_to_control_path.clear();
    request.measured_to_control_elapsed_sec.clear();
    for (const auto & item : prediction.current_to_control) {
      request.measured_to_control_path.push_back({item.state.x_m, item.state.y_m, item.state.yaw_rad});
      request.measured_to_control_elapsed_sec.push_back(item.source_sec - request.now_sec);
    }
    request.applied_program_required = true;
    request.input_application_profile = {"signed-observation-test", .25, .25, .02};
    const auto stop = retained::evaluate_stop_successor(request);
    if (!stop.accepted()) throw std::runtime_error(retained::to_string(stop.reason));
    const auto built = stop_bundle::build(request, stop, 10001U);
    if (!built.plan) throw std::runtime_error(stop_bundle::to_string(built.reason));
    request.plan = built.plan;
    request.execution_clock = {retained::ExecutionClockKind::TimeAlignedCandidate, NAN, NAN};
    return request;
  };
  auto request = make_stop(-tolerance / 2);
  ASSERT_LT(request.current_speed_mps, 0);
  ASSERT_DOUBLE_EQ(request.publication_prefix->observation.initial.state.forward_velocity_mps,
    request.current_speed_mps);
  ASSERT_DOUBLE_EQ(request.control_origin_speed_mps, 0);
  const auto proved = retained::evaluate(request);
  ASSERT_EQ(proved.reason, retained::Reason::Accepted);
  ASSERT_TRUE(proved.proof && proved.proof->applied_program);
  ASSERT_TRUE(production::build(proved).authority);
  auto missing = request;
  missing.publication_prefix.reset();
  EXPECT_FALSE(production::build(retained::evaluate(missing)).authority);
  missing = request;
  missing.applied_program_required = false;
  missing.input_application_profile.reset();
  EXPECT_EQ(retained::evaluate(missing).reason, retained::Reason::InvalidCurrentState);
  const auto reverse = retained::evaluate(make_stop(-2 * tolerance));
  EXPECT_FALSE(production::build(reverse).authority);
  EXPECT_EQ(reverse.applied_program_reason, applied::Reason::StateBoundRejected);
}


TEST(MpccAppliedProgram, CapturedCornerDisplacementSeparatesTheWholeOriginalStop)
{
  namespace vm = multi_purpose_mpc_ros::mpcc_vehicle_model;
  namespace num = vm::numerical;
  namespace fixture = multi_purpose_mpc_ros::test;
  const auto observation = fixture::corner_packet_observation();
  const auto program = fixture::corner_packet_program();
  const auto grid = fixture::wall_packet_grid();
  const auto footprint = physical::resolve_clearance_footprint({1.615, .51, .768, .768, .05}, .2);
  ASSERT_TRUE(footprint);
  const auto & origin = observation.initial.state;
  const auto offsets = num::footprint_vertex_offsets(*footprint);
  vm::AppliedFootprintValidation context;
  for (size_t i = 0; i < 8; ++i) context.local_offsets[i] = {offsets[i].lo, offsets[i].hi};
  // The previous enclosure is a frozen input. Its contact is not a required
  // property of a later, tighter numerical representation.
  const auto old = fixture::corner_packet_checkpoint();
  num::Box old_body, old_corners;
  for (size_t i = 0; i < 8; ++i) {
    old_body[i] = {old.body[i].lower, old.body[i].upper};
    old_corners[i] = {old.corners[i].lower, old.corners[i].upper};
  }
  const auto old_box = num::footprint(old_body, *footprint);
  const auto & op = old_box.pose;
  const double co = std::cos(origin.yaw_rad), so = std::sin(origin.yaw_rad);
  const auto old_cells = recovery::sample_footprint(grid, old_box.extents,
    {origin.x_m + co*op.x_m - so*op.y_m, origin.y_m + so*op.x_m + co*op.y_m,
      origin.yaw_rad + op.yaw_rad});
  ASSERT_EQ(old_cells.contact_cells, std::vector<size_t>{468057U});
  ASSERT_TRUE(num::separating_cell_clearance(old_corners, grid, 468057U, {origin.x_m, origin.y_m}));
  size_t checked = 0;
  context.validate = [&](const vm::BodyRanges &body, const vm::FootprintRanges &corners,
      double, double) {
    num::Box box, vertices;
    for (size_t i = 0; i < 8; ++i) {
      box[i] = {body[i].lower, body[i].upper};
      vertices[i] = {corners[i].lower, corners[i].upper};
    }
    const auto enclosed = num::footprint(box, *footprint);
    const auto & p = enclosed.pose;
    const double co = std::cos(origin.yaw_rad), so = std::sin(origin.yaw_rad);
    const recovery::Pose2D world{origin.x_m + co*p.x_m - so*p.y_m,
      origin.y_m + so*p.x_m + co*p.y_m, origin.yaw_rad + p.yaw_rad};
    const auto cells = recovery::sample_footprint(grid, enclosed.extents, world);
    EXPECT_TRUE(cells.valid); EXPECT_FALSE(cells.out_of_map);
    if (!cells.valid || cells.out_of_map) return false;
    ++checked;
    for (const auto cell : cells.contact_cells)
      if (!num::separating_cell_clearance(vertices, grid, cell, {origin.x_m, origin.y_m})) return false;
    return true;
  };
  const auto prediction = vm::predict_applied_inputs_to_rest(observation, program,
    {"awsim-2025-empirical-receiver-age-250ms-v1", .25, .25, .1},
    fixture::vehicle_model(), {}, &context);
  ASSERT_TRUE(prediction.tube) << static_cast<int>(prediction.reason);
  EXPECT_GT(checked, 200U);
  EXPECT_GT(prediction.tube->rest_sec, observation.now_sec + .25);
  EXPECT_LE(prediction.tube->rest_sec, 11.174999773 + 1e-12);
  for (size_t index : {3U, 4U, 5U}) {
    EXPECT_EQ(prediction.tube->source_to_rest.back().endpoint_body[index].lower, 0);
    EXPECT_EQ(prediction.tube->source_to_rest.back().endpoint_body[index].upper, 0);
  }
}

TEST(MpccRateResolvedRetainedRevalidation, ExtraCornersCannotAuthorizeRealWallsOrMapExit)
{
  const auto original = applied_request();
  const auto nominal = retained::evaluate(original);
  ASSERT_TRUE(nominal.proof);
  const vehicle::InputApplicationProfile profile{"test-receiver", .25, .25, .02};
  for (auto state : {recovery::CellState::Occupied, recovery::CellState::Unknown}) {
    auto request = original;
    auto grid = std::make_shared<recovery::OccupancyGrid>();
    grid->width = request.current_wall_grid->width;
    grid->height = request.current_wall_grid->height;
    grid->resolution_m = request.current_wall_grid->resolution_m;
    grid->origin_x_m = request.current_wall_grid->origin_x_m;
    grid->origin_y_m = request.current_wall_grid->origin_y_m;
    grid->y_axis = request.current_wall_grid->y_axis;
    grid->cells.assign(grid->width * grid->height, state);
    request.current_wall_grid = grid;
    const auto result = applied::certify_terminal_stop(request, *nominal.proof, profile);
    EXPECT_EQ(result.reason, applied::Reason::WallRejected);
    EXPECT_FALSE(result.certificate);
  }
  auto request = original;
  auto grid = std::make_shared<recovery::OccupancyGrid>(*request.current_wall_grid);
  grid->origin_x_m += 1000;
  request.current_wall_grid = grid;
  const auto result = applied::certify_terminal_stop(request, *nominal.proof, profile);
  EXPECT_EQ(result.reason, applied::Reason::WallRejected);
  EXPECT_FALSE(result.certificate);
}


TEST(MpccAppliedProgram, CapturedMovingSpeedDependenceStaysClearForEveryInputThroughRest)
{
  namespace vm = multi_purpose_mpc_ros::mpcc_vehicle_model;
  namespace num = vm::numerical;
  namespace fixture = multi_purpose_mpc_ros::test;
  const auto observation = fixture::speed_partition_observation();
  const auto program = fixture::speed_partition_program();
  const auto model = fixture::vehicle_model();
  const auto grid = fixture::wall_packet_grid();
  const auto footprint = physical::resolve_clearance_footprint({1.615, .51, .768, .768, .05}, .2);
  ASSERT_TRUE(footprint);
  const auto & origin = observation.initial.state;
  const double co = std::cos(origin.yaw_rad), so = std::sin(origin.yaw_rad);
  const auto world = [&](const recovery::Pose2D &p) {
    return recovery::Pose2D{origin.x_m + co*p.x_m - so*p.y_m,
      origin.y_m + so*p.x_m + co*p.y_m, origin.yaw_rad + p.yaw_rad};
  };
  const auto old = fixture::speed_partition_checkpoint();
  num::Box old_body, old_corners;
  for (size_t i = 0; i < 8; ++i) {
    old_body[i] = {old.body[i].lower, old.body[i].upper};
    old_corners[i] = {old.corners[i].lower, old.corners[i].upper};
  }
  const auto old_box = num::footprint(old_body, *footprint);
  const auto old_cells = recovery::sample_footprint(grid, old_box.extents, world(old_box.pose));
  ASSERT_NE(std::find(old_cells.contact_cells.begin(), old_cells.contact_cells.end(), 476325U),
    old_cells.contact_cells.end());
  ASSERT_FALSE(num::separating_cell_clearance(old_corners, grid, 476325U, {origin.x_m, origin.y_m}));
  const auto offsets = num::footprint_vertex_offsets(*footprint);
  vm::AppliedFootprintValidation context;
  for (size_t i = 0; i < 8; ++i) context.local_offsets[i] = {offsets[i].lo, offsets[i].hi};
  size_t checked = 0;
  context.validate = [&](const auto &body, const auto &corners, double, double) {
    num::Box box, vertices;
    for (size_t i = 0; i < 8; ++i) {
      box[i] = {body[i].lower, body[i].upper}; vertices[i] = {corners[i].lower, corners[i].upper};
    }
    const auto b = num::footprint(box, *footprint);
    const auto cells = recovery::sample_footprint(grid, b.extents, world(b.pose));
    if (!cells.valid || cells.out_of_map) return false;
    for (auto cell : cells.contact_cells)
      if (!num::separating_cell_clearance(vertices, grid, cell, {origin.x_m, origin.y_m})) return false;
    ++checked; return true;
  };
  const auto prediction = vm::predict_applied_inputs_to_rest(observation, program,
    {"awsim-2025-empirical-receiver-age-250ms-v1", .25, .25, .1}, model, {}, &context);
  ASSERT_TRUE(prediction.tube) << static_cast<int>(prediction.reason);
  EXPECT_GT(checked, 200U);
  EXPECT_LE(prediction.tube->maximum_body_partitions, 6U);
  EXPECT_GT(prediction.tube->rest_sec, program.commands.back().published_sec + .025);
  auto initial = origin; initial.x_m = initial.y_m = initial.yaw_rad = 0;
  std::vector<vm::State> oracles(128, initial);
  std::mt19937_64 random(20260912); std::uniform_real_distribution<double> unit(0, 1);
  size_t values_checked = 0, step = 0;
  for (const auto &s : prediction.tube->source_to_rest) {
    ASSERT_TRUE(s.swept_footprint); ASSERT_TRUE(s.endpoint_footprint);
    std::vector<vm::ScalarRange> groups;
    for (const auto &g : s.inputs.acceleration_sign_groups) if (g) groups.push_back(*g);
    ASSERT_FALSE(groups.empty());
    for (size_t arm = 0; arm < oracles.size(); ++arm) {
      auto &state = oracles[arm]; const auto before = state;
      const auto &g = groups[arm % groups.size()];
      const double af = arm < 4 ? double(arm % 2) : arm < 8 ? double((step+arm)%2) : unit(random);
      const double sf = arm < 4 ? double(arm / 2) : arm < 8 ? double((step+arm/2)%2) : unit(random);
      state.desired_steering_rad = (s.inputs.wire_steering_rad.lower + sf *
        (s.inputs.wire_steering_rad.upper - s.inputs.wire_steering_rad.lower)) / model.steering_wire_gain;
      const auto next = vm::advance(state, {g.lower + af * (g.upper-g.lower), 0}, model, s.duration_sec);
      ASSERT_TRUE(next); state = next->state;
      const auto values = num::values(state);
      for (size_t i = 0; i < 8; ++i) {
        ASSERT_GE(values[i], s.endpoint_body[i].lower); ASSERT_LE(values[i], s.endpoint_body[i].upper);
        ++values_checked;
      }
      for (int part = 0; part <= 4; ++part) {
        const double f = part / 4.;
        const double bx = before.x_m + f*(state.x_m-before.x_m), by = before.y_m + f*(state.y_m-before.y_m);
        const double angle = origin.yaw_rad + before.yaw_rad + f*(state.yaw_rad-before.yaw_rad);
        for (size_t k = 0; k < 8; k += 2) {
          const double x = (offsets[k].lo+offsets[k].hi)/2, y = (offsets[k+1].lo+offsets[k+1].hi)/2;
          const double X = co*bx-so*by+std::cos(angle)*x-std::sin(angle)*y;
          const double Y = so*bx+co*by+std::sin(angle)*x+std::cos(angle)*y;
          ASSERT_GE(X, (*s.swept_footprint)[k].lower); ASSERT_LE(X, (*s.swept_footprint)[k].upper);
          ASSERT_GE(Y, (*s.swept_footprint)[k+1].lower); ASSERT_LE(Y, (*s.swept_footprint)[k+1].upper);
          values_checked += 2;
        }
      }
    }
    ++step;
  }
  EXPECT_GT(values_checked, 1000000U);
  for (const auto &s : oracles) {
    EXPECT_EQ(s.forward_velocity_mps, 0); EXPECT_EQ(s.lateral_velocity_mps, 0); EXPECT_EQ(s.yaw_rate_radps, 0);
  }
  for (size_t i : {3U, 4U, 5U}) {
    EXPECT_EQ(prediction.tube->source_to_rest.back().endpoint_body[i].lower, 0);
    EXPECT_EQ(prediction.tube->source_to_rest.back().endpoint_body[i].upper, 0);
  }
}


TEST(MpccAppliedProgram, CapturedObliqueWallSeparationPreservesTheEntireInputTube)
{
  namespace vm = multi_purpose_mpc_ros::mpcc_vehicle_model;
  namespace num = vm::numerical;
  namespace fixture = multi_purpose_mpc_ros::test;
  const auto observation = fixture::oriented_wall_observation();
  const auto program = fixture::oriented_wall_program();
  const auto model = fixture::vehicle_model();
  auto grid = fixture::wall_packet_grid();
  const auto footprint = physical::resolve_clearance_footprint({1.615, .51, .768, .768, .05}, .2);
  ASSERT_TRUE(footprint);
  const auto &o = observation.initial.state;
  const recovery::Pose2D origin{o.x_m, o.y_m, o.yaw_rad};
  const double co = std::cos(o.yaw_rad), so = std::sin(o.yaw_rad);
  const auto world = [&](const recovery::Pose2D &p) {
    return recovery::Pose2D{o.x_m + co*p.x_m - so*p.y_m,
      o.y_m + so*p.x_m + co*p.y_m, o.yaw_rad + p.yaw_rad};
  };
  const auto old = fixture::oriented_wall_checkpoint();
  num::Box old_body, old_corners;
  for (size_t i = 0; i < 8; ++i) {
    old_body[i] = {old.body[i].lower, old.body[i].upper};
    old_corners[i] = {old.corners[i].lower, old.corners[i].upper};
  }
  const auto old_box = num::footprint(old_body, *footprint);
  const auto cells = recovery::sample_footprint(grid, old_box.extents, world(old_box.pose));
  ASSERT_NE(std::find(cells.contact_cells.begin(), cells.contact_cells.end(), 476325U),
    cells.contact_cells.end());
  ASSERT_FALSE(num::separating_cell_clearance(old_corners, grid, 476325U, {o.x_m, o.y_m}));
  const auto gap = num::separating_oriented_cell_clearance(old_body, *footprint,
    old_corners, grid, 476325U, origin);
  ASSERT_TRUE(gap); EXPECT_GT(*gap, 0);
  const auto offsets = num::footprint_vertex_offsets(*footprint);
  vm::AppliedFootprintValidation context;
  for (size_t i = 0; i < 8; ++i) context.local_offsets[i] = {offsets[i].lo, offsets[i].hi};
  bool directional = false;
  size_t checked = 0;
  context.validate = [&](const auto &body, const auto &corners, double, double) {
    num::Box box, vertices;
    for (size_t i = 0; i < 8; ++i) {
      box[i] = {body[i].lower, body[i].upper}; vertices[i] = {corners[i].lower, corners[i].upper};
    }
    const auto b = num::footprint(box, *footprint);
    const auto occupied = recovery::sample_footprint(grid, b.extents, world(b.pose));
    if (!occupied.valid || occupied.out_of_map) return false;
    for (auto cell : occupied.contact_cells) {
      if (directional) {
        if (!num::separating_oriented_cell_clearance(box, *footprint, vertices, grid, cell, origin)) return false;
      } else if (!num::separating_cell_clearance(vertices, grid, cell, {o.x_m, o.y_m})) return false;
    }
    ++checked; return true;
  };
  const vm::InputApplicationProfile profile{"awsim-2025-empirical-receiver-age-250ms-v1", .25, .25, .1};
  // The frozen coarse checkpoint above still needs the oriented separator.
  // The current force Jacobian tightens the whole tube enough for either
  // separator. This must preserve the same input programme and complete rest.
  const auto axis_aligned = vm::predict_applied_inputs_to_rest(observation, program, profile, model, {}, &context);
  ASSERT_EQ(axis_aligned.reason, vm::AppliedInputRejectReason::None);
  ASSERT_TRUE(axis_aligned.tube);
  directional = true; checked = 0;
  const auto accepted = vm::predict_applied_inputs_to_rest(observation, program, profile, model, {}, &context);
  ASSERT_TRUE(accepted.tube) << static_cast<int>(accepted.reason);
  EXPECT_GT(checked, 150U);
  EXPECT_NEAR(accepted.tube->rest_sec, 10.764999778, 1e-12);
  const auto wall_validator = context.validate;
  context.validate = [](const auto &, const auto &, double, double) {return true;};
  const auto original = vm::predict_applied_inputs_to_rest(observation, program, profile, model, {}, &context);
  ASSERT_TRUE(original.tube);
  ASSERT_EQ(accepted.tube->source_to_rest.size(), original.tube->source_to_rest.size());
  ASSERT_EQ(axis_aligned.tube->source_to_rest.size(), original.tube->source_to_rest.size());
  for (size_t j = 0; j < accepted.tube->source_to_rest.size(); ++j) {
    const auto &a = accepted.tube->source_to_rest[j], &b = original.tube->source_to_rest[j];
    const auto &axis = axis_aligned.tube->source_to_rest[j];
    for (size_t i = 0; i < 8; ++i) {
      EXPECT_DOUBLE_EQ(axis.swept_body[i].lower, b.swept_body[i].lower);
      EXPECT_DOUBLE_EQ(axis.swept_body[i].upper, b.swept_body[i].upper);
      EXPECT_DOUBLE_EQ((*axis.swept_footprint)[i].lower, (*b.swept_footprint)[i].lower);
      EXPECT_DOUBLE_EQ((*axis.swept_footprint)[i].upper, (*b.swept_footprint)[i].upper);
      EXPECT_DOUBLE_EQ(a.swept_body[i].lower, b.swept_body[i].lower);
      EXPECT_DOUBLE_EQ(a.swept_body[i].upper, b.swept_body[i].upper);
      EXPECT_DOUBLE_EQ(a.endpoint_body[i].lower, b.endpoint_body[i].lower);
      EXPECT_DOUBLE_EQ(a.endpoint_body[i].upper, b.endpoint_body[i].upper);
      EXPECT_DOUBLE_EQ((*a.swept_footprint)[i].lower, (*b.swept_footprint)[i].lower);
      EXPECT_DOUBLE_EQ((*a.swept_footprint)[i].upper, (*b.swept_footprint)[i].upper);
    }
  }
  for (size_t i : {3U, 4U, 5U}) {
    EXPECT_EQ(accepted.tube->source_to_rest.back().endpoint_body[i].lower, 0);
    EXPECT_EQ(accepted.tube->source_to_rest.back().endpoint_body[i].upper, 0);
  }
  // A concrete occupied initial footprint remains a hard rejection with both
  // separators. Precision improvements must never bypass the wall callback.
  std::fill(grid.cells.begin(), grid.cells.end(), recovery::CellState::Occupied);
  const auto contact = recovery::sample_footprint(grid, *footprint, origin);
  ASSERT_TRUE(contact.valid);
  ASSERT_FALSE(contact.contact_cells.empty());
  context.validate = wall_validator;
  for (bool use_directional : {false, true}) {
    directional = use_directional;
    const auto rejected = vm::predict_applied_inputs_to_rest(
      observation, program, profile, model, {}, &context);
    EXPECT_EQ(rejected.reason, vm::AppliedInputRejectReason::ValidationRejected);
    EXPECT_FALSE(rejected.tube);
  }

}


TEST(MpccRateResolvedRetainedRevalidation, PublicationWindowSurvivesStopMaterializationAndEvidence)
{
  auto request = source_horizon_request();
  const auto result = retained::evaluate(request);
  ASSERT_TRUE(result.proof && result.proof->applied_program);
  const auto & program = result.proof->applied_program->prepared().program;
  EXPECT_DOUBLE_EQ(program.maximum_publication_delay_sec, program.publication_interval_sec);
  ASSERT_TRUE(program.nanosecond_clock);
  const auto stop = stop_bundle::build_certified_terminal(request, result, 23001U);
  ASSERT_TRUE(stop.plan && stop.plan->execution_artifact->applied_stop_program);
  const auto & execution = *stop.plan->execution_artifact;
  const auto & provenance = *execution.applied_stop_program;
  const auto remaining = stop_program::remaining_program(provenance.program, request.now_sec + .015);
  ASSERT_TRUE(remaining);
  EXPECT_DOUBLE_EQ(remaining->maximum_publication_delay_sec, program.maximum_publication_delay_sec);
  ASSERT_TRUE(remaining->nanosecond_clock);
  EXPECT_EQ(remaining->nanosecond_clock->interval_ns, program.nanosecond_clock->interval_ns);
  EXPECT_EQ(remaining->nanosecond_clock->maximum_delay_ns, program.nanosecond_clock->maximum_delay_ns);
  YAML::Emitter encoded; encoded.SetDoublePrecision(17);
  encoded << vehicle::encode_applied_program_provenance(provenance);
  auto node = YAML::Load(encoded.c_str());
  EXPECT_EQ(node["schema"].as<std::string>(), "applied-stop-provenance-v4");
  EXPECT_TRUE(node["has_forward_velocity_ceiling"].as<bool>());
  const auto decoded = vehicle::decode_applied_program_provenance(node, execution.vehicle_model);
  ASSERT_TRUE(decoded);
  EXPECT_EQ(vehicle::applied_program_provenance_fingerprint(*decoded, execution.vehicle_model),
    vehicle::applied_program_provenance_fingerprint(provenance, execution.vehicle_model));
  node["schema"] = "applied-stop-provenance-v3";
  EXPECT_FALSE(vehicle::decode_applied_program_provenance(node, execution.vehicle_model));
  node["schema"] = "applied-stop-provenance-v4";
  node["program"]["nanosecond_clock"].remove("first_ns");
  EXPECT_FALSE(vehicle::decode_applied_program_provenance(node, execution.vehicle_model));
  node["program"]["nanosecond_clock"]["first_ns"] = program.nanosecond_clock->first_ns + 1;
  EXPECT_FALSE(vehicle::decode_applied_program_provenance(node, execution.vehicle_model));
  node["program"]["nanosecond_clock"]["first_ns"] = program.nanosecond_clock->first_ns;
  node["program"]["maximum_publication_delay_sec"] = program.publication_interval_sec * 2;
  EXPECT_FALSE(vehicle::decode_applied_program_provenance(node, execution.vehicle_model));
  node["program"]["maximum_publication_delay_sec"] = program.maximum_publication_delay_sec;
  node.remove("forward_velocity_ceiling_mps");
  EXPECT_FALSE(vehicle::decode_applied_program_provenance(node, execution.vehicle_model));
  node["forward_velocity_ceiling_mps"] = *provenance.forward_velocity_ceiling_mps;
  node["has_forward_velocity_ceiling"] = false;
  EXPECT_FALSE(vehicle::decode_applied_program_provenance(node, execution.vehicle_model));
  node["has_forward_velocity_ceiling"] = true;
  node["schema"] = "applied-stop-provenance-v2";
  EXPECT_FALSE(vehicle::decode_applied_program_provenance(node, execution.vehicle_model));
  node["schema"] = "applied-stop-provenance-v4";
  node["program"].remove("maximum_publication_delay_sec");
  EXPECT_FALSE(vehicle::decode_applied_program_provenance(node, execution.vehicle_model));
}

TEST(MpccAppliedProgram, FinalPublicationObservationPreservesImmutableRequestAndDrainsFirstEvents)
{
  namespace capture = multi_purpose_mpc_ros::mpcc_architecture_snapshot;
  const auto request = source_horizon_request();
  const auto result = retained::evaluate(request);
  ASSERT_TRUE(result.proof && result.proof->applied_program);
  const auto certificate = result.proof->applied_program;
  ASSERT_TRUE(certificate->observation_request());
  const auto & exact = *certificate->observation_request();
  ASSERT_EQ(exact.decision_id, request.decision_id);
  const auto & program = certificate->prepared().program;
  const auto & first = program.commands.front();
  const auto directory = std::filesystem::temp_directory_path() /
    ("mpcc-publication-observation-" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()));
  const auto parent_thread = std::this_thread::get_id();
  std::vector<capture::RecordResult> records;
  bool callback_on_worker = true;
  capture::FirstPublicationFailureRecorder recorder(
    [&](const auto &, const auto & record) {
      callback_on_worker &= std::this_thread::get_id() != parent_thread;
      records.push_back(record);
    });
  capture::PublicationFailureObservation observation;
  observation.certificate = certificate;
  observation.decision_id = exact.decision_id;
  observation.nominal_sec = first.published_sec;
  observation.decision_clock_sec = exact.now_sec;
  observation.wire_acceleration_mps2 = first.wire_acceleration_mps2;
  observation.wire_steering_rad = first.wire_steering_rad;
  observation.output_root = directory;
  for (bool moving : {false, true}) {
    for (bool post : {false, true}) {
      observation.moving = moving;
      observation.after_publication = post;
      observation.before_clock_sec = first.published_sec +
        program.maximum_publication_delay_sec + (post ? -.001 : .005);
      observation.after_clock_sec = first.published_sec + program.maximum_publication_delay_sec + .005;
      EXPECT_EQ(recorder.submit(observation), capture::ObservationAdmission::Queued);
      auto later = observation;
      later.decision_id += 1000;
      later.wire_acceleration_mps2 = 123;
      EXPECT_EQ(recorder.submit(later), capture::ObservationAdmission::Duplicate);
    }
  }
  recorder.stop();
  ASSERT_EQ(records.size(), 4U);
  EXPECT_TRUE(callback_on_worker);
  EXPECT_EQ(recorder.submit(observation), capture::ObservationAdmission::Stopped);
  for (const auto & record : records) {
    ASSERT_EQ(record.status, capture::RecordStatus::Written) << record.detail;
    const auto document = YAML::LoadFile(record.snapshot_file.string());
    EXPECT_EQ(document["schema"].as<std::string>(), "mpcc-final-publication-failure/v1");
    EXPECT_FALSE(document["authority"].as<bool>());
    const auto boundary = document["boundary"];
    EXPECT_EQ(boundary["decision_id"].as<std::uint64_t>(), exact.decision_id);
    EXPECT_DOUBLE_EQ(boundary["wire_acceleration_mps2"].as<double>(), first.wire_acceleration_mps2);
    EXPECT_DOUBLE_EQ(boundary["wire_steering_rad"].as<double>(), first.wire_steering_rad);
    EXPECT_TRUE(boundary["final_packet_matches"].as<bool>());
    EXPECT_FALSE(boundary["publication_bracket_admitted"].as<bool>());
    EXPECT_DOUBLE_EQ(boundary["decision_clock_sec"].as<double>(), exact.now_sec);
    EXPECT_DOUBLE_EQ(boundary["after_clock_sec"].as<double>(), observation.after_clock_sec);
    EXPECT_DOUBLE_EQ(document["rest_sec"].as<double>(), certificate->tube().rest_sec);
    const auto evidence = document["revalidation_evidence"];
    EXPECT_EQ(evidence["status"].as<std::string>(), "present");
    EXPECT_EQ(evidence["request"]["decision_id"].as<std::uint64_t>(), exact.decision_id);
    EXPECT_DOUBLE_EQ(evidence["request"]["now_sec"].as<double>(), exact.now_sec);
    EXPECT_DOUBLE_EQ(evidence["request"]["control_origin_sec"].as<double>(), exact.control_origin_sec);
    const auto encoded = vehicle::decode_input_program(document["applied_program"]);
    ASSERT_TRUE(encoded);
    EXPECT_EQ(encoded->commands.size(), program.commands.size());
    EXPECT_DOUBLE_EQ(encoded->maximum_publication_delay_sec, program.maximum_publication_delay_sec);
    for (size_t i = 0; i < program.commands.size(); ++i) {
      EXPECT_DOUBLE_EQ(encoded->commands[i].published_sec, program.commands[i].published_sec);
      EXPECT_DOUBLE_EQ(encoded->commands[i].wire_acceleration_mps2, program.commands[i].wire_acceleration_mps2);
      EXPECT_DOUBLE_EQ(encoded->commands[i].wire_steering_rad, program.commands[i].wire_steering_rad);
    }
    if (exact.current_wall_grid) {
      const auto path = record.snapshot_file.parent_path() / "revalidation-wall-grid.bin";
      std::ifstream file(path, std::ios::binary);
      const std::string bytes{std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
      ASSERT_EQ(bytes.size(), exact.current_wall_grid->cells.size());
      for (size_t i = 0; i < bytes.size(); ++i) {
        EXPECT_EQ(static_cast<std::int8_t>(bytes[i]), static_cast<std::int8_t>(exact.current_wall_grid->cells[i]));
      }
    }
  }
  EXPECT_TRUE(certificate->matches(exact));
  std::filesystem::remove_all(directory);
}

TEST(MpccAppliedProgram, FinalPublicationObservationReportsMissingOrMismatchedEvidence)
{
  namespace capture = multi_purpose_mpc_ros::mpcc_architecture_snapshot;
  const auto directory = std::filesystem::temp_directory_path() /
    ("mpcc-publication-missing-" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()));
  capture::PublicationFailureObservation observation;
  observation.output_root = directory;
  EXPECT_EQ(capture::record_publication_failure(observation).status, capture::RecordStatus::InvalidInput);
  observation.decision_id = 100001;
  auto recorded = capture::record_publication_failure(observation);
  ASSERT_EQ(recorded.status, capture::RecordStatus::Written);
  auto document = YAML::LoadFile(recorded.snapshot_file.string());
  EXPECT_EQ(document["certificate_status"].as<std::string>(), "missing");
  EXPECT_EQ(document["revalidation_evidence"]["status"].as<std::string>(), "missing");
  const auto request = source_horizon_request();
  const auto result = retained::evaluate(request);
  ASSERT_TRUE(result.proof && result.proof->applied_program);
  observation.certificate = result.proof->applied_program;
  ++observation.decision_id;
  recorded = capture::record_publication_failure(observation);
  ASSERT_EQ(recorded.status, capture::RecordStatus::Written);
  document = YAML::LoadFile(recorded.snapshot_file.string());
  EXPECT_EQ(document["revalidation_evidence"]["status"].as<std::string>(), "invalid");
  EXPECT_EQ(document["revalidation_evidence"]["request"]["decision_id"].as<std::uint64_t>(), request.decision_id);
  EXPECT_FALSE(document["boundary"]["final_packet_matches"].as<bool>());
  EXPECT_EQ(capture::record_publication_failure(observation).status, capture::RecordStatus::Duplicate);
  std::filesystem::remove_all(directory);
}


namespace scheduled = multi_purpose_mpc_ros::mpcc_rate_resolved_scheduled;

scheduled::Request scheduled_request(const contract::ControlIntent intent = contract::ControlIntent::Track)
{
  auto observed = applied_request(intent);
  observed.applied_program_required = true;
  observed.input_application_profile = vehicle::InputApplicationProfile{"test-receiver", .25, .25, .02};
  auto observation = observed.publication_prefix->observation;
  for (std::size_t i = 0; i < observation.commands.size(); ++i)
    observation.commands[i].published_sec = static_cast<double>(i * 25000000LL) / 1e9;
  observed.publication_prefix = vehicle::predict_prospective_publication(observation,
    observed.publication_prefix->proposed_packet, observed.plan->execution_artifact->vehicle_model);
  if (!observed.publication_prefix) throw std::runtime_error("invalid scheduled fixture history");
  const auto &forecast = *observed.publication_prefix;
  const auto &origin = forecast.control_origin;
  observed.control_pose = {origin.x_m, origin.y_m, origin.yaw_rad};
  observed.control_origin_physical_progress_m = origin.x_m;
  observed.current_speed_mps = forecast.current.forward_velocity_mps;
  observed.control_origin_speed_mps = origin.forward_velocity_mps;
  observed.current_steering_rad = origin.desired_steering_rad;
  observed.current_response_steering_rad = origin.tire_steering_rad;
  observed.current_lateral_velocity_mps = origin.lateral_velocity_mps;
  observed.current_yaw_rate_radps = origin.yaw_rate_radps;
  observed.measured_to_control_path.clear(); observed.measured_to_control_elapsed_sec.clear();
  for (const auto &sample : forecast.current_to_control) {
    observed.measured_to_control_path.push_back({sample.state.x_m, sample.state.y_m, sample.state.yaw_rad});
    observed.measured_to_control_elapsed_sec.push_back(sample.source_sec - observed.now_sec);
  }
  const double steering = observation.commands.back().wire_steering_rad;
  vehicle::PublishedInputProgram prior{.025, {{1.05, -3, steering}, {1.075, -3, steering}}, true, .025};
  prior.nanosecond_clock = vehicle::publication_nanosecond_clock(1.05, .025, .025);
  return {observed, prior, 0, 2, 1.14, {{0, 0, 0}, 0}};
}

TEST(MpccScheduledProgram, FullProofPreservesWaitingStopAndCannotUseOrdinaryAuthority)
{
  for (const auto intent : {contract::ControlIntent::Track, contract::ControlIntent::Cruise, contract::ControlIntent::Follow}) {
    SCOPED_TRACE(static_cast<int>(intent));
    const auto request = scheduled_request(intent);
    const auto result = scheduled::evaluate(request);
    ASSERT_TRUE(result.applied.certificate) << retained::to_string(result.reason) << '/' <<
      static_cast<int>(result.applied.reason) << '/' << static_cast<int>(result.applied.program_reason) << '/' <<
      static_cast<int>(result.applied.prediction_reason);
    EXPECT_EQ(result.reason, retained::Reason::Accepted);
    EXPECT_FALSE(result.diagnostic.proof);
    const auto &certificate = *result.applied.certificate;
    const auto &nominal = *certificate.nominal();
    EXPECT_EQ(certificate.first_suffix_index(), 2U);
    EXPECT_DOUBLE_EQ(nominal.forecast().observation.now_sec, request.observed.now_sec);
    EXPECT_DOUBLE_EQ(nominal.forecast().publication_sec, 1.1);
    EXPECT_DOUBLE_EQ(nominal.forecast().control_origin_sec, 1.14);
    EXPECT_EQ(nominal.forecast().observation.commands.size(), request.observed.publication_prefix->observation.commands.size());
    EXPECT_GT(certificate.tube().rest_sec, nominal.forecast().control_origin_sec);
    EXPECT_GT(certificate.checked_samples(), 2U);
    ASSERT_GE(certificate.tube().program.commands.size(), 3U);
    for (std::size_t i = 0; i < 2; ++i) {
      EXPECT_DOUBLE_EQ(certificate.tube().program.commands[i].published_sec, request.prior_program.commands[i].published_sec);
      EXPECT_DOUBLE_EQ(certificate.tube().program.commands[i].wire_acceleration_mps2, -3);
      EXPECT_DOUBLE_EQ(certificate.tube().program.commands[i].wire_steering_rad, request.prior_program.commands[i].wire_steering_rad);
    }
    EXPECT_TRUE(nominal.proof().publication_prefix_required);
    EXPECT_TRUE(nominal.proof().applied_program_required);
    EXPECT_FALSE(nominal.proof().publication_prefix);
    EXPECT_FALSE(nominal.proof().applied_program);
    auto escaped = result.diagnostic;
    escaped.proof = nominal.proof();
    EXPECT_FALSE(production::build(escaped).authority);
    EXPECT_FALSE(retained::evaluate(nominal.nominal_view()).proof);
    if (intent == contract::ControlIntent::Follow) {
      ASSERT_TRUE(nominal.nominal_view().follow_target);
      const auto &old = *request.observed.follow_target;
      const auto &future = *nominal.nominal_view().follow_target;
      EXPECT_DOUBLE_EQ(future.observed_sec, old.observed_sec);
      EXPECT_EQ(future.observation_generation, old.observation_generation);
      for (double elapsed : {0.0, .03, .15, .5}) {
        const auto original = retained::follow_target_progress_at(old, .05 + elapsed);
        const auto shifted = retained::follow_target_progress_at(future, elapsed);
        ASSERT_TRUE(original); ASSERT_TRUE(shifted);
        EXPECT_NEAR(*original + request.observed.control_origin_physical_progress_m,
          *shifted + nominal.nominal_view().control_origin_physical_progress_m, 1e-12);
      }
      EXPECT_GE(certificate.minimum_follow_gap_m(), old.hard_gap_m);
    }
  }
}

TEST(MpccScheduledProgram, RejectsUnboundFrameClockHistoryAndMissingRequiredInputs)
{
  const auto original = scheduled_request();
  auto bad = original; bad.progress_frame.progress_m = .001;
  EXPECT_EQ(scheduled::evaluate(bad).reason, retained::Reason::CourseFrameUnavailable);
  bad = original; bad.prior_program.nanosecond_clock.reset();
  bad.prior_program.commands[1].published_sec += 1e-9;
  EXPECT_FALSE(scheduled::evaluate(bad).applied.certificate);
  bad = original; bad.prior_program.repeat_last_until_rest = false; bad.preceding_packet_count = 3;
  EXPECT_FALSE(scheduled::evaluate(bad).applied.certificate);
  bad = original; bad.observed.publication_prefix->observation.commands.resize(1);
  EXPECT_FALSE(scheduled::evaluate(bad).applied.certificate);
  bad = original; bad.observed.publication_prefix->observation.commands.back().published_sec = 1.06;
  EXPECT_FALSE(scheduled::evaluate(bad).applied.certificate);
  bad = original; bad.observed.input_application_profile.reset();
  EXPECT_FALSE(scheduled::evaluate(bad).applied.certificate);
  bad = original; bad.observed.applied_program_required = false;
  EXPECT_FALSE(scheduled::evaluate(bad).applied.certificate);
  bad = original; bad.planned_control_origin_sec = 1.09;
  EXPECT_FALSE(scheduled::evaluate(bad).applied.certificate);
}

TEST(MpccScheduledProgram, RejectsPeerContactAndUnsafeFollowGap)
{
  auto request = scheduled_request();
  const auto &body = request.observed.publication_prefix->observation.initial.state;
  request.observed.obstacles.obstacles.push_back({"waiting-contact", {body.x_m, body.y_m, 0, 0, .2}});
  EXPECT_FALSE(scheduled::evaluate(request).applied.certificate);
  request = scheduled_request(contract::ControlIntent::Follow);
  request.observed.follow_target->hard_gap_m = 6;
  EXPECT_FALSE(scheduled::evaluate(request).applied.certificate);
}


TEST(MpccScheduledProgram, ChecksWaitingContactEvenWhenPeerLeavesBeforeTheNewPacket)
{
  auto request = scheduled_request();
  const auto &body = request.observed.publication_prefix->observation.initial.state;
  request.observed.obstacles.obstacles.push_back({"waiting-only", {body.x_m, body.y_m, 0, 30, .02}});
  const auto result = scheduled::evaluate(request);
  EXPECT_FALSE(result.applied.certificate);
  // Nominal future path is clear. The complete applied proof must independently
  // reject contact at the original observation, before the new first packet.
  EXPECT_EQ(result.reason, retained::Reason::AppliedProgramUnavailable);
  EXPECT_EQ(result.applied.reason, applied::Reason::PeerRejected);
  EXPECT_EQ(result.applied.rejected_peer_id, "waiting-only");
  EXPECT_LE(result.applied.rejected_sec, request.observed.now_sec);
}


TEST(MpccScheduledProgram, MovingPeerPredictionIncludesTheWaitingInterval)
{
  auto request = scheduled_request();
  request.preceding_packet_count = 3;
  request.planned_control_origin_sec = 1.165;
  const auto empty_world = scheduled::evaluate(request);
  ASSERT_TRUE(empty_world.applied.certificate) << retained::to_string(empty_world.reason);
  const auto &future = empty_world.applied.certificate->nominal()->forecast().publication_state;
  // The peer has already left this location when the future packet begins.
  // Its original position is ahead of the ego; the waiting interval is clear.
  request.observed.obstacles.obstacles.push_back({"already-departed", {future.x_m, future.y_m, 0, 30, .005}});
  const auto result = scheduled::evaluate(request);
  EXPECT_TRUE(result.applied.certificate) << retained::to_string(result.reason) << '/' <<
    static_cast<int>(result.applied.reason) << '/' << result.applied.rejected_peer_id << '/' << result.applied.rejected_sec;
  EXPECT_GT(result.diagnostic.minimum_dynamic_clearance_m, 0);
}

TEST(MpccScheduledProgram, FutureGapCannotEraseOriginalObservedHardGapViolation)
{
  auto request = scheduled_request(contract::ControlIntent::Follow);
  request.observed.follow_target->current_target_gap_m = 2;
  const auto result = scheduled::evaluate(request);
  EXPECT_FALSE(result.applied.certificate);
  EXPECT_EQ(result.reason, retained::Reason::FollowInitialHardGapViolation);
}


TEST(MpccScheduledProgram, PreservesOriginalFrameAndFollowObservationIdentity)
{
  auto request = scheduled_request(contract::ControlIntent::Follow);
  auto bad = request; bad.observed.control_pose.y_m += .001;
  EXPECT_FALSE(scheduled::evaluate(bad).applied.certificate);
  bad = request; bad.observed.follow_target->observed_sec = 1.08;
  EXPECT_EQ(scheduled::evaluate(bad).reason, retained::Reason::FollowTargetObservationInvalid);
  bad = request; bad.observed.follow_target->target_id = "different-target";
  EXPECT_FALSE(scheduled::evaluate(bad).applied.certificate);
  bad = request; bad.observed.follow_target->observation_generation++;
  EXPECT_FALSE(scheduled::evaluate(bad).applied.certificate);
  const auto original = scheduled::evaluate(request);
  ASSERT_TRUE(original.applied.certificate);
  request.progress_frame.progress_m = 100;
  request.observed.control_origin_physical_progress_m += 100;
  const auto lifted = scheduled::evaluate(request);
  ASSERT_TRUE(lifted.applied.certificate) << retained::to_string(lifted.reason);
  EXPECT_NEAR(lifted.applied.certificate->minimum_follow_gap_m(),
    original.applied.certificate->minimum_follow_gap_m(), 1e-12);
  EXPECT_NEAR(lifted.applied.certificate->nominal()->original_follow_reference_progress_m(),
    original.applied.certificate->nominal()->original_follow_reference_progress_m(), 1e-12);
}

TEST(MpccScheduledProgram, MissingSolvedStopReferencePreservesScheduledWorldContext)
{
  auto request = scheduled_request();
  auto execution = std::make_shared<artifact::ExecutionArtifact>(*request.observed.plan->execution_artifact);
  // A stationary virtual-progress interval does not define a strictly
  // increasing normal-path lateral reference; the native Stop remains a
  // candidate and must still carry the scheduled full-world obligation.
  execution->predicted_states[1].progress_m = execution->predicted_states.front().progress_m;
  execution->control_stages[0].virtual_progress_speed_mps = 0;
  execution->control_stages[1].virtual_progress_speed_mps = 4;
  ASSERT_EQ(artifact::validate(*execution), artifact::RejectReason::None);
  namespace adapter = multi_purpose_mpc_ros::mpcc_rate_resolved_physical_adapter;
  EXPECT_FALSE(adapter::build_normal_path_stop_profile(*execution));
  const auto snapshot = source_snapshot(execution->identity);
  const auto built = certified::build(execution, snapshot, accepted_result(snapshot));
  ASSERT_TRUE(built.plan);
  request.observed.plan = built.plan;
  const auto &body = request.observed.publication_prefix->observation.initial.state;
  request.observed.obstacles.obstacles.push_back({"waiting-only", {body.x_m, body.y_m, 0, 30, .02}});
  const auto result = scheduled::evaluate(request);
  EXPECT_FALSE(result.applied.certificate);
  EXPECT_EQ(result.reason, retained::Reason::AppliedProgramUnavailable);
  EXPECT_EQ(result.applied.reason, applied::Reason::PeerRejected);
  EXPECT_EQ(result.applied.rejected_peer_id, "waiting-only");
}


vehicle::ObservationProvenance scheduled_fresh_observation(const applied::ScheduledCertificate &certificate)
{
  const auto &forecast = certificate.nominal()->forecast();
  const auto state_at = [&](double time) {
    const auto it = std::find_if(forecast.observation_to_control.begin(), forecast.observation_to_control.end(),
      [time](const vehicle::TimedState &sample) { return sample.source_sec == time; });
    if (it == forecast.observation_to_control.end()) throw std::runtime_error("fixture component epoch unavailable");
    return it->state;
  };
  auto fresh = forecast.observation;
  fresh.now_sec = 1.1; fresh.control_origin_sec = 1.14;
  fresh.initial = {1.1, state_at(1.1)};
  fresh.velocity_source_sec = 1.075;
  fresh.initial.state.forward_velocity_mps = state_at(fresh.velocity_source_sec).forward_velocity_mps;
  fresh.initial.state.lateral_velocity_mps = state_at(fresh.velocity_source_sec).lateral_velocity_mps;
  fresh.yaw_rate_source_sec = 1.07;
  fresh.initial.state.yaw_rate_radps = state_at(fresh.yaw_rate_source_sec).yaw_rate_radps;
  fresh.tire_source_sec = 1.095;
  fresh.initial.state.tire_steering_rad = state_at(fresh.tire_source_sec).tire_steering_rad;
  // This is the physical angle issued before serialization, not an observation
  // of which desired steering the receiver has applied.
  fresh.initial.state.desired_steering_rad = forecast.observation.initial.state.desired_steering_rad;
  fresh.commands.insert(fresh.commands.end(), forecast.nominal_prefix.commands.begin(), forecast.nominal_prefix.commands.begin() + 2);
  return fresh;
}

TEST(MpccScheduledMeasurements, UsesEachSourceEpochWithoutReanchoringTheOriginalPopulation)
{
  const auto result = scheduled::evaluate(scheduled_request());
  ASSERT_TRUE(result.applied.certificate);
  const auto &certificate = *result.applied.certificate;
  const auto original_hash = certificate.tube().context_fingerprint;
  const auto original_now = certificate.tube().observation.now_sec;
  auto fresh = scheduled_fresh_observation(certificate);
  ASSERT_TRUE(vehicle::valid(fresh));
  EXPECT_EQ(scheduled::check_measurement_consistency(certificate, fresh).reason, scheduled::MeasurementReason::Compatible);
  // Braking changes velocity between the velocity sample and the later pose.
  // Substituting the pose-time velocity must fail at the velocity source epoch.
  fresh.initial.state.forward_velocity_mps = certificate.nominal()->forecast().publication_state.forward_velocity_mps;
  const auto wrong_epoch = scheduled::check_measurement_consistency(certificate, fresh);
  EXPECT_EQ(wrong_epoch.reason, scheduled::MeasurementReason::VelocityMismatch);
  EXPECT_DOUBLE_EQ(wrong_epoch.rejected_source_sec, fresh.velocity_source_sec);
  EXPECT_EQ(certificate.tube().context_fingerprint, original_hash);
  EXPECT_DOUBLE_EQ(certificate.tube().observation.now_sec, original_now);
}

TEST(MpccScheduledMeasurements, RejectsInconsistentPublicComponentsAndIssuedSteering)
{
  const auto result = scheduled::evaluate(scheduled_request());
  ASSERT_TRUE(result.applied.certificate);
  const auto &certificate = *result.applied.certificate;
  const auto fresh = scheduled_fresh_observation(certificate);
  auto bad = fresh; bad.initial.state.x_m += 10;
  EXPECT_EQ(scheduled::check_measurement_consistency(certificate, bad).reason, scheduled::MeasurementReason::PoseMismatch);
  bad = fresh; bad.initial.state.yaw_rad += 1;
  EXPECT_EQ(scheduled::check_measurement_consistency(certificate, bad).reason, scheduled::MeasurementReason::PoseMismatch);
  bad = fresh; bad.initial.state.lateral_velocity_mps += 10;
  EXPECT_EQ(scheduled::check_measurement_consistency(certificate, bad).reason, scheduled::MeasurementReason::VelocityMismatch);
  bad = fresh; bad.initial.state.yaw_rate_radps += 10;
  EXPECT_EQ(scheduled::check_measurement_consistency(certificate, bad).reason, scheduled::MeasurementReason::YawRateMismatch);
  bad = fresh; bad.initial.state.tire_steering_rad += 1;
  EXPECT_EQ(scheduled::check_measurement_consistency(certificate, bad).reason, scheduled::MeasurementReason::TireMismatch);
  bad = fresh; bad.initial.state.desired_steering_rad += .01;
  EXPECT_EQ(scheduled::check_measurement_consistency(certificate, bad).reason, scheduled::MeasurementReason::IssuedSteeringMismatch);
  bad = fresh; bad.steering_delay_sec += .01;
  EXPECT_EQ(scheduled::check_measurement_consistency(certificate, bad).reason, scheduled::MeasurementReason::InputDelayMismatch);
  bad = fresh; bad.now_sec = certificate.tube().rest_sec + .1; bad.control_origin_sec = bad.now_sec;
  EXPECT_EQ(scheduled::check_measurement_consistency(certificate, bad).reason, scheduled::MeasurementReason::TimeOutsideProof);
}

TEST(MpccScheduledMeasurements, RepeatedOriginalSamplesRequireExactValuesAndNoClockRegression)
{
  auto request = scheduled_request();
  // Original empirical reconstruction held these earlier component samples at
  // the pose epoch. An unchanged sample retains that original provenance; a
  // changed earlier sample cannot be verified by a tube starting at the pose.
  request.observed.publication_prefix->observation.velocity_source_sec = 1.025;
  request.observed.publication_prefix->observation.yaw_rate_source_sec = 1.02;
  const auto result = scheduled::evaluate(request);
  ASSERT_TRUE(result.applied.certificate);
  const auto &certificate = *result.applied.certificate;
  auto fresh = certificate.tube().observation;
  EXPECT_EQ(scheduled::check_measurement_consistency(certificate, fresh).reason, scheduled::MeasurementReason::Compatible);
  auto changed = fresh; changed.initial.state.forward_velocity_mps = std::nextafter(changed.initial.state.forward_velocity_mps, INFINITY);
  EXPECT_EQ(scheduled::check_measurement_consistency(certificate, changed).reason, scheduled::MeasurementReason::VelocityMismatch);
  changed = fresh; changed.velocity_source_sec = 1.015;
  EXPECT_EQ(scheduled::check_measurement_consistency(certificate, changed).reason, scheduled::MeasurementReason::VelocityMismatch);
  changed = fresh; changed.velocity_source_sec = 1.03;
  EXPECT_EQ(scheduled::check_measurement_consistency(certificate, changed).reason, scheduled::MeasurementReason::VelocityMismatch);
  changed = fresh; changed.yaw_rate_source_sec = 1.015;
  EXPECT_EQ(scheduled::check_measurement_consistency(certificate, changed).reason, scheduled::MeasurementReason::YawRateMismatch);
}


TEST(MpccScheduledMeasurements, ChecksRotatedWorldCoordinatesAndWrappedYaw)
{
  for (double angle : {1.2, std::acos(-1.0) - .001}) {
    SCOPED_TRACE(angle);
    auto request = scheduled_request();
    const auto rotate = [angle](const recovery::Pose2D &pose) {
      return recovery::Pose2D{50 + std::cos(angle) * (pose.x_m - 50) - std::sin(angle) * pose.y_m,
        std::sin(angle) * (pose.x_m - 50) + std::cos(angle) * pose.y_m, pose.yaw_rad + angle};
    };
    auto source = *request.observed.plan->physical_snapshot;
    source.current_pose = rotate(source.current_pose);
    for (auto &pose : source.control_prefix) pose = rotate(pose);
    for (auto &k : source.course_frame_knots) {
      const auto pose = rotate({k.x_m, k.y_m, k.heading_rad});
      k.x_m = pose.x_m; k.y_m = pose.y_m; k.heading_rad = pose.yaw_rad;
    }
    const auto plan = certified::build(request.observed.plan->execution_artifact, source, accepted_result(source));
    ASSERT_TRUE(plan.plan);
    auto &observed = request.observed; observed.plan = plan.plan;
    auto observation = observed.publication_prefix->observation;
    auto &state = observation.initial.state;
    const auto pose = rotate({state.x_m, state.y_m, state.yaw_rad});
    state.x_m = pose.x_m; state.y_m = pose.y_m; state.yaw_rad = pose.yaw_rad;
    observed.publication_prefix = vehicle::predict_prospective_publication(observation,
      observed.publication_prefix->proposed_packet, observed.plan->execution_artifact->vehicle_model);
    ASSERT_TRUE(observed.publication_prefix);
    const auto &forecast = *observed.publication_prefix;
    const auto &origin = forecast.control_origin;
    observed.control_pose = {origin.x_m, origin.y_m, origin.yaw_rad};
    observed.control_origin_physical_progress_m = 50 +
      (std::cos(angle) * (origin.x_m - 50) + std::sin(angle) * origin.y_m);
    request.progress_frame = {{50, 0, angle}, 50};
    observed.current_speed_mps = forecast.current.forward_velocity_mps;
    observed.control_origin_speed_mps = origin.forward_velocity_mps;
    observed.current_steering_rad = origin.desired_steering_rad;
    observed.current_response_steering_rad = origin.tire_steering_rad;
    observed.current_lateral_velocity_mps = origin.lateral_velocity_mps;
    observed.current_yaw_rate_radps = origin.yaw_rate_radps;
    observed.measured_to_control_path.clear(); observed.measured_to_control_elapsed_sec.clear();
    for (const auto &sample : forecast.current_to_control) {
      observed.measured_to_control_path.push_back({sample.state.x_m, sample.state.y_m, sample.state.yaw_rad});
      observed.measured_to_control_elapsed_sec.push_back(sample.source_sec - observed.now_sec);
    }
    const auto result = scheduled::evaluate(request);
    ASSERT_TRUE(result.applied.certificate) << retained::to_string(result.reason);
    const auto &certificate = *result.applied.certificate;
    auto fresh = scheduled_fresh_observation(certificate);
    EXPECT_EQ(scheduled::check_measurement_consistency(certificate, fresh).reason, scheduled::MeasurementReason::Compatible);
    fresh.initial.state.yaw_rad = std::atan2(std::sin(fresh.initial.state.yaw_rad), std::cos(fresh.initial.state.yaw_rad));
    EXPECT_EQ(scheduled::check_measurement_consistency(certificate, fresh).reason, scheduled::MeasurementReason::Compatible);
    fresh.initial.state.x_m += .5;
    EXPECT_EQ(scheduled::check_measurement_consistency(certificate, fresh).reason, scheduled::MeasurementReason::PoseMismatch);
  }
}


TEST(MpccScheduledPrefix, AuthenticatesOriginalOldAndNewPacketsAcrossAdoption)
{
  const auto request = scheduled_request();
  const auto result = scheduled::evaluate(request);
  ASSERT_TRUE(result.applied.certificate);
  const auto &certificate = *result.applied.certificate;
  vehicle::PublishedInputLedger ledger(256);
  for (const auto &packet : certificate.tube().observation.commands)
    ASSERT_TRUE(ledger.record(packet, packet.published_sec, packet.published_sec, 2));
  const auto cursor = ledger.snapshot(); ASSERT_TRUE(cursor);
  std::vector<std::optional<vehicle::PublishedProgramSource>> prior_sources;
  for (std::size_t i = 0; i < 2; ++i) {
    prior_sources.push_back(vehicle::PublishedProgramSource{50, 7, 8, 9, 10 + i});
    const auto &packet = request.prior_program.commands[i];
    ASSERT_TRUE(ledger.record(packet, packet.published_sec, packet.published_sec, 2, prior_sources.back()));
  }
  auto fresh = scheduled_fresh_observation(certificate);
  EXPECT_EQ(scheduled::check_actual_publication_prefix(certificate, ledger, *cursor, fresh, prior_sources, 0),
    scheduled::PrefixReason::Consistent);
  auto wrong_sources = prior_sources; wrong_sources[0]->solution_id++;
  EXPECT_EQ(scheduled::check_actual_publication_prefix(certificate, ledger, *cursor, fresh, wrong_sources, 0),
    scheduled::PrefixReason::ActualPrefixMismatch);
  const auto owner = scheduled::scheduled_program_source(certificate, 0); ASSERT_TRUE(owner);
  EXPECT_EQ(owner->decision_id, request.observed.decision_id);
  EXPECT_EQ(owner->solution_id, request.observed.plan->execution_artifact->identity.sequence);
  EXPECT_EQ(owner->problem_fingerprint, request.observed.plan->execution_artifact->identity.source_context.fingerprint);
  EXPECT_EQ(owner->input_context_fingerprint, certificate.tube().context_fingerprint);
  const auto &packet = certificate.suffix().program.commands.front();
  ASSERT_TRUE(ledger.record(packet, packet.published_sec, packet.published_sec, 2, owner));
  fresh.commands = ledger.history();
  EXPECT_EQ(scheduled::check_actual_publication_prefix(certificate, ledger, *cursor, fresh, prior_sources, 1),
    scheduled::PrefixReason::Consistent);
  EXPECT_EQ(scheduled::check_actual_publication_prefix(certificate, ledger, *cursor, fresh, prior_sources, 0),
    scheduled::PrefixReason::ActualPrefixMismatch);
  EXPECT_FALSE(scheduled::scheduled_program_source(certificate, 10000));
  ledger.reset(); fresh.commands = ledger.history();
  EXPECT_NE(scheduled::check_actual_publication_prefix(certificate, ledger, *cursor, fresh, prior_sources, 1),
    scheduled::PrefixReason::Consistent);
}

TEST(MpccScheduledPrefix, RejectsUnboundHistoryMissingStopAndPostPublicationOverrun)
{
  const auto request = scheduled_request();
  const auto result = scheduled::evaluate(request);
  ASSERT_TRUE(result.applied.certificate);
  const auto &certificate = *result.applied.certificate;
  const std::vector<std::optional<vehicle::PublishedProgramSource>> emergency_prefix(2);
  for (int variant = 0; variant < 4; ++variant) {
    SCOPED_TRACE(variant);
    vehicle::PublishedInputLedger ledger(256);
    for (auto packet : certificate.tube().observation.commands) {
      if (variant == 0 && packet.published_sec == 0) packet.wire_acceleration_mps2 = -2;
      ASSERT_TRUE(ledger.record(packet, packet.published_sec, packet.published_sec, 2));
    }
    const auto cursor = ledger.snapshot(); ASSERT_TRUE(cursor);
    for (std::size_t i = 0; i < 2; ++i) {
      if (variant == 1 && i == 0) continue;
      const auto &packet = request.prior_program.commands[i];
      const auto deadline = vehicle::publication_epoch(request.prior_program, i, true); ASSERT_TRUE(deadline);
      const double after = variant == 2 && i == 1 ?
        static_cast<double>(*vehicle::publication_nanoseconds(*deadline) + 1) / 1e9 : packet.published_sec;
      ASSERT_TRUE(ledger.record(packet, packet.published_sec, after, 2));
    }
    auto fresh = scheduled_fresh_observation(certificate); fresh.commands = ledger.history();
    if (variant == 2) { fresh.now_sec = 1.100000001; fresh.control_origin_sec = 1.14; }
    if (variant == 3) fresh.commands.back().wire_acceleration_mps2 = 1;
    const auto rejected = scheduled::check_actual_publication_prefix(certificate, ledger, *cursor, fresh, emergency_prefix, 0);
    const auto expected = variant == 0 ? scheduled::PrefixReason::OriginalHistoryMismatch :
      variant == 3 ? scheduled::PrefixReason::CurrentHistoryMismatch : scheduled::PrefixReason::ActualPrefixMismatch;
    EXPECT_EQ(rejected, expected);
  }
}

TEST(MpccFollowOriginBoundary, TargetAbsoluteProgressIsIndependentOfEgoFrameOffset)
{
  const double waypoint_progress = 50;
  const double target_progress = 55;
  for (double lag : {-.1, .05, .2}) {
    SCOPED_TRACE(lag);
    const double physical_control_origin = waypoint_progress + lag;
    const double gap = target_progress - physical_control_origin;
    // The peer coordinate comes from independent straight-course geometry.
    // Both live producers now use a physical-origin type without waypoint lag.
    const auto observation = retained::build_physical_origin_follow_target_observation(
      {"d2", 7, 1.05, gap, 3, 2, {.1, .1}, true});
    ASSERT_TRUE(observation);
    for (double elapsed : {0.0, .05, .2, .4}) {
      const auto relative = retained::follow_target_progress_at(*observation, elapsed);
      ASSERT_TRUE(relative);
      EXPECT_NEAR(physical_control_origin + *relative, target_progress + 2 * elapsed, 1e-12);
    }
  }
}


TEST(MpccFollowOriginBoundary, StopCannotBorrowWaypointOffsetAsExtraHardGap)
{
  for (double gap : {3.7, 4.7}) {
    SCOPED_TRACE(gap);
    auto request = applied_request(contract::ControlIntent::Follow);
    request.applied_program_required = true;
    request.input_application_profile = vehicle::InputApplicationProfile{"test-receiver", .25, .25, .02};
    request.obstacles.obstacles[0].circle.x_m = request.control_origin_physical_progress_m + gap;
    request.obstacles.obstacles[0].circle.velocity_x_mps = 0;
    request.follow_target = retained::build_physical_origin_follow_target_observation({"d2", request.obstacles.generation,
      request.obstacles.observed_sec, gap, 3, 0, {.1, .1}, true});
    const auto result = retained::evaluate(request);
    if (gap == 3.7) {
      EXPECT_FALSE(result.proof);
      EXPECT_EQ(result.reason, retained::Reason::TerminalContingencyUnavailable);
    } else {
      ASSERT_TRUE(result.proof) << retained::to_string(result.reason);
      ASSERT_TRUE(result.proof->applied_program);
      EXPECT_GE(result.proof->applied_program->minimum_follow_gap_m(), 3);
    }
  }
}

retained::Request scheduled_fresh_world(const applied::ScheduledCertificate &certificate, double now = 1.1,
  bool suffix_published = false)
{
  auto fresh = certificate.nominal()->observed();
  auto observation = scheduled_fresh_observation(certificate);
  if (suffix_published) {
    observation.commands.push_back(certificate.suffix().program.commands.front());
    observation.initial.state.desired_steering_rad = certificate.nominal()->proof().actuation.steering_rad;
  }
  observation.now_sec = now; observation.control_origin_sec = now + .04;
  fresh.now_sec = now; fresh.control_origin_sec = observation.control_origin_sec;
  fresh.decision_id++;
  fresh.previous_published_steering_rad = observation.initial.state.desired_steering_rad;
  fresh.previous_published_command_age_sec = now - observation.commands.back().published_sec;
  const auto packet = retained::prospective_artifact_packet(fresh);
  if (!packet) throw std::runtime_error("current world fixture packet unavailable");
  fresh.publication_prefix = vehicle::predict_prospective_publication(observation, *packet,
    fresh.plan->execution_artifact->vehicle_model);
  if (!fresh.publication_prefix) throw std::runtime_error("current world fixture prediction unavailable");
  const auto &predicted = *fresh.publication_prefix;
  const auto &origin = predicted.control_origin;
  fresh.control_pose = {origin.x_m, origin.y_m, origin.yaw_rad};
  fresh.control_origin_physical_progress_m = origin.x_m;
  fresh.current_speed_mps = predicted.current.forward_velocity_mps;
  fresh.control_origin_speed_mps = origin.forward_velocity_mps;
  fresh.current_time_steering_rad = predicted.current.tire_steering_rad;
  fresh.current_steering_rad = origin.desired_steering_rad;
  fresh.current_response_steering_rad = origin.tire_steering_rad;
  fresh.current_lateral_velocity_mps = origin.lateral_velocity_mps;
  fresh.current_yaw_rate_radps = origin.yaw_rate_radps;
  fresh.measured_to_control_path.clear(); fresh.measured_to_control_elapsed_sec.clear();
  for (const auto &sample : predicted.current_to_control) {
    fresh.measured_to_control_path.push_back({sample.state.x_m, sample.state.y_m, sample.state.yaw_rad});
    fresh.measured_to_control_elapsed_sec.push_back(sample.source_sec - now);
  }
  const double lead = now - fresh.obstacles.observed_sec;
  for (auto &peer : fresh.obstacles.obstacles) {
    const auto center = peer.circle.predicted_center(lead);
    peer.circle.x_m = center[0]; peer.circle.y_m = center[1];
  }
  fresh.obstacles.generation++; fresh.obstacles.observed_sec = now;
  if (fresh.follow_target) {
    const auto &peer = fresh.obstacles.obstacles.front();
    fresh.follow_target = retained::build_physical_origin_follow_target_observation({peer.id,
      fresh.obstacles.generation, now, peer.circle.x_m - fresh.control_origin_physical_progress_m,
      fresh.follow_target->hard_gap_m, peer.circle.velocity_x_mps, {.1, .1}, true});
  }
  return fresh;
}

TEST(MpccScheduledCurrentWorld, RechecksOnlyTheRemainingOriginalBodyAndCorners)
{
  const auto result = scheduled::evaluate(scheduled_request()); ASSERT_TRUE(result.applied.certificate);
  const auto &certificate = *result.applied.certificate;
  auto fresh = scheduled_fresh_world(certificate, 1.1025);
  // A newly observed obstacle at a past ego location must be checked only
  // against the remaining tube, with nonnegative fresh-peer elapsed times.
  const auto &old = certificate.tube().observation.initial.state;
  fresh.obstacles.obstacles.push_back({"behind", {old.x_m, old.y_m, 0, 0, .01}});
  auto wall = std::make_shared<recovery::OccupancyGrid>(*fresh.current_wall_grid);
  const auto occupy = [&](double x, double y) {
    const auto cell = wall->world_to_grid(x, y);
    ASSERT_TRUE(cell);
    wall->cells[cell->row * wall->width + cell->column] = recovery::CellState::Occupied;
    wall->non_free_integral_index.clear();
  };
  occupy(old.x_m - .04, old.y_m); fresh.current_wall_grid = wall;
  const auto check = scheduled::recheck_remaining_world(certificate, fresh);
  EXPECT_EQ(check.reason, scheduled::CurrentWorldReason::Current) << static_cast<int>(check.physical_reason);
  EXPECT_GT(check.checked_samples, 0U); EXPECT_LT(check.checked_samples, certificate.checked_samples());
  const auto &body = certificate.nominal()->forecast().publication_state;
  occupy(body.x_m, body.y_m);
  const auto contact = scheduled::recheck_remaining_world(certificate, fresh);
  EXPECT_EQ(contact.reason, scheduled::CurrentWorldReason::PhysicalRejected);
  EXPECT_EQ(contact.physical_reason, applied::Reason::WallRejected);
  EXPECT_GE(contact.rejected_sec, fresh.now_sec);
}

TEST(MpccScheduledCurrentWorld, RejectsFreshPeersAndChangedPhysicalContext)
{
  const auto result = scheduled::evaluate(scheduled_request()); ASSERT_TRUE(result.applied.certificate);
  const auto &certificate = *result.applied.certificate;
  const auto original = scheduled_fresh_world(certificate);
  auto fresh = original;
  const auto &body = certificate.nominal()->forecast().publication_state;
  fresh.obstacles.obstacles.push_back({"new-contact", {body.x_m, body.y_m, 0, 0, .1}});
  const auto contact = scheduled::recheck_remaining_world(certificate, fresh);
  EXPECT_EQ(contact.reason, scheduled::CurrentWorldReason::PhysicalRejected);
  EXPECT_EQ(contact.physical_reason, applied::Reason::PeerRejected);
  EXPECT_EQ(contact.rejected_peer_id, "new-contact");
  fresh = original; fresh.current_footprint.front_extent_m += .1;
  EXPECT_EQ(scheduled::recheck_remaining_world(certificate, fresh).reason, scheduled::CurrentWorldReason::InvalidContext);
  fresh = original; fresh.maximum_acceleration_mps2 += .1;
  EXPECT_EQ(scheduled::recheck_remaining_world(certificate, fresh).reason, scheduled::CurrentWorldReason::InvalidContext);
  fresh = original; fresh.stop_lateral_policy.lateral_gain += .1;
  EXPECT_EQ(scheduled::recheck_remaining_world(certificate, fresh).reason, scheduled::CurrentWorldReason::InvalidContext);
  fresh = original; fresh.control_pose.x_m += .1;
  EXPECT_EQ(scheduled::recheck_remaining_world(certificate, fresh).reason, scheduled::CurrentWorldReason::InvalidContext);
  fresh = original; fresh.obstacles.current = false;
  EXPECT_EQ(scheduled::recheck_remaining_world(certificate, fresh).reason, scheduled::CurrentWorldReason::InvalidContext);
}

TEST(MpccScheduledCurrentWorld, BindsFreshFollowForecastAndCircularPhysicalOrigin)
{
  const auto result = scheduled::evaluate(scheduled_request(contract::ControlIntent::Follow));
  ASSERT_TRUE(result.applied.certificate);
  const auto &certificate = *result.applied.certificate;
  const auto original = scheduled_fresh_world(certificate);
  const auto check = scheduled::recheck_remaining_world(certificate, original);
  ASSERT_EQ(check.reason, scheduled::CurrentWorldReason::Current) << static_cast<int>(check.physical_reason);
  EXPECT_GE(check.minimum_follow_gap_m, original.follow_target->hard_gap_m);
  auto fresh = original; fresh.control_origin_physical_progress_m += fresh.path_length_m;
  const auto wrapped = scheduled::recheck_remaining_world(certificate, fresh);
  EXPECT_EQ(wrapped.reason, scheduled::CurrentWorldReason::Current);
  EXPECT_NEAR(wrapped.minimum_follow_gap_m, check.minimum_follow_gap_m, 1e-12);
  fresh = original;
  for (auto &progress : fresh.follow_target->target_progress_from_current_origin_m) progress += .8;
  EXPECT_EQ(scheduled::recheck_remaining_world(certificate, fresh).reason,
    scheduled::CurrentWorldReason::InvalidFollowObservation);
  fresh = original; fresh.control_origin_physical_progress_m += 1;
  EXPECT_EQ(scheduled::recheck_remaining_world(certificate, fresh).reason, scheduled::CurrentWorldReason::FollowOriginUnavailable);
  fresh = original; fresh.follow_target->current_target_gap_m = 2;
  const auto unsafe = scheduled::recheck_remaining_world(certificate, fresh);
  EXPECT_EQ(unsafe.reason, scheduled::CurrentWorldReason::PhysicalRejected);
  EXPECT_EQ(unsafe.physical_reason, applied::Reason::FollowGapRejected);
  fresh = original; fresh.follow_target->observation_generation--;
  EXPECT_EQ(scheduled::recheck_remaining_world(certificate, fresh).reason, scheduled::CurrentWorldReason::InvalidFollowObservation);
  fresh = original; fresh.follow_target->target_id = "different";
  EXPECT_EQ(scheduled::recheck_remaining_world(certificate, fresh).reason, scheduled::CurrentWorldReason::InvalidFollowObservation);
  fresh = original; fresh.obstacles.observed_sec = original.now_sec + .01;
  EXPECT_EQ(scheduled::recheck_remaining_world(certificate, fresh).reason, scheduled::CurrentWorldReason::InvalidContext);
}

// These hand-built numerical fixtures stand in for the solver source. The real
// solver/stateless-copy tests separately check input-generation propagation.
void attach_synthetic_source_generation(scheduled::Request & request,
  const scheduled::ContextSnapshot & generation)
{
  attach_velocity_source(request.observed, 4.0);
  auto plan = std::make_shared<certified::CertifiedPlan>(*request.observed.plan);
  auto source = std::make_shared<multi_purpose_mpc_ros::mpcc_rate_resolved_shadow::Snapshot>(
    *plan->solver_source_snapshot);
  source->normal_context_generation = generation;
  plan->solver_source_snapshot = std::move(source);
  request.observed.plan = std::move(plan);
}

TEST(MpccScheduledContext, RevokedSourceCannotReviveWithOldCopiesOrRestoredValues)
{
  scheduled::ContextOwner owner;
  auto request = scheduled_request();
  attach_synthetic_source_generation(request, owner.capture());
  const auto captured = request.observed.plan->solver_source_snapshot->normal_context_generation;
  const auto result = scheduled::evaluate(request); ASSERT_TRUE(result.applied.certificate);
  const auto &certificate = *result.applied.certificate;
  auto current = certificate.suffix().source.source_context;
  current.decision_id++; current.observation_generation++;
  current = contract::seal_problem_context(current);
  EXPECT_EQ(scheduled::check_current_context(certificate, owner.capture(), current,
    scheduled::ContextUse::NewSource), scheduled::ContextReason::Compatible);
  scheduled::ContextOwner another_session;
  EXPECT_EQ(scheduled::check_current_context(certificate, another_session.capture(), current,
    scheduled::ContextUse::NewSource), scheduled::ContextReason::GenerationChanged);
  owner.invalidate();
  EXPECT_FALSE(captured.valid());
  EXPECT_NE(scheduled::check_current_context(certificate, captured, current,
    scheduled::ContextUse::NewSource), scheduled::ContextReason::Compatible);
  // Identical field values after a policy/reference/reset round trip do not
  // make work captured before that event current again.
  EXPECT_NE(scheduled::check_current_context(certificate, owner.capture(), current,
    scheduled::ContextUse::NewSource), scheduled::ContextReason::Compatible);
  scheduled::ContextSnapshot stopped_owner;
  { scheduled::ContextOwner session; stopped_owner = session.capture(); ASSERT_TRUE(stopped_owner.valid()); }
  EXPECT_FALSE(stopped_owner.valid());
}

TEST(MpccScheduledContext, RequiresBoundGenerationAndCompleteCurrentSemanticIdentity)
{
  scheduled::ContextOwner owner;
  const auto diagnostic = scheduled::evaluate(scheduled_request()); ASSERT_TRUE(diagnostic.applied.certificate);
  EXPECT_EQ(scheduled::check_current_context(*diagnostic.applied.certificate, owner.capture(),
    diagnostic.applied.certificate->suffix().source.source_context, scheduled::ContextUse::NewSource),
    scheduled::ContextReason::MissingGeneration);
  auto request = scheduled_request(contract::ControlIntent::Follow); attach_synthetic_source_generation(request, owner.capture());
  const auto result = scheduled::evaluate(request); ASSERT_TRUE(result.applied.certificate);
  const auto &certificate = *result.applied.certificate;
  const auto original = certificate.suffix().source.source_context;
  auto fresh = original; fresh.decision_id++; fresh.observation_generation++; fresh.target_obstacle_generation++;
  fresh = contract::seal_problem_context(fresh);
  EXPECT_EQ(scheduled::check_current_context(certificate, owner.capture(), fresh,
    scheduled::ContextUse::NewSource), scheduled::ContextReason::Compatible);
  for (int change = 0; change < 13; ++change) {
    SCOPED_TRACE(change); fresh = original;
    switch (change) {
      case 0: fresh.intent = contract::ControlIntent::Cruise; break;
      case 1: fresh.intent_generation++; break;
      case 2: fresh.target_id = "d3"; break;
      case 3: fresh.execution_side_sign = 1; break;
      case 4: fresh.horizon_steps++; break;
      case 5: fresh.formulation = contract::Formulation::SolverDerivedBypass; break;
      case 6: fresh.vehicle_model_fingerprint++; break;
      case 7: fresh.state_schema_id += "-changed"; break;
      case 8: fresh.input_schema_id += "-changed"; break;
      case 9: fresh.bounds_schema_id += "-changed"; break;
      case 10: fresh.cost_schema_id += "-changed"; break;
      case 11:
        fresh.dynamic_obstacle_constraint_active = true; fresh.dynamic_obstacle_id = "d3";
        fresh.dynamic_obstacle_generation = 7; fresh.dynamic_obstacle_side_sign = -1; break;
      case 12: fresh.fingerprint++; break;
    }
    if (change != 12) fresh = contract::seal_problem_context(fresh);
    for (auto use : {scheduled::ContextUse::NewSource, scheduled::ContextUse::PublishedRemainder})
      EXPECT_NE(scheduled::check_current_context(certificate, owner.capture(), fresh, use),
        scheduled::ContextReason::Compatible);
  }
}

TEST(MpccScheduledContext, NewSourceGeometryAndPublishedWindowHaveDistinctObligations)
{
  scheduled::ContextOwner owner;
  auto request = scheduled_request(); attach_synthetic_source_generation(request, owner.capture());
  const auto result = scheduled::evaluate(request); ASSERT_TRUE(result.applied.certificate);
  const auto &certificate = *result.applied.certificate;
  auto fresh = certificate.suffix().source.source_context; fresh.stage_geometry_id++;
  fresh = contract::seal_problem_context(fresh);
  EXPECT_EQ(scheduled::check_current_context(certificate, owner.capture(), fresh,
    scheduled::ContextUse::NewSource), scheduled::ContextReason::GeometryChanged);
  // This necessary context result is not authority: the dispatcher must prove
  // prior publication and every fresh physical/clock/slew condition separately.
  EXPECT_EQ(scheduled::check_current_context(certificate, owner.capture(), fresh,
    scheduled::ContextUse::PublishedRemainder), scheduled::ContextReason::Compatible);
  EXPECT_EQ(scheduled::check_current_context(certificate, owner.capture(), fresh,
    static_cast<scheduled::ContextUse>(-1)), scheduled::ContextReason::InvalidProblem);
}

TEST(MpccScheduledCurrentEvidence, BindsOneFrameAndRequiresEveryIndependentGate)
{
  scheduled::ContextOwner owner;
  auto request = scheduled_request(); attach_synthetic_source_generation(request, owner.capture());
  const auto result = scheduled::evaluate(request); ASSERT_TRUE(result.applied.certificate);
  const auto &certificate = *result.applied.certificate;
  vehicle::PublishedInputLedger ledger(256);
  for (const auto &packet : certificate.tube().observation.commands)
    ASSERT_TRUE(ledger.record(packet, packet.published_sec, packet.published_sec, 2));
  const auto cursor = ledger.snapshot(); ASSERT_TRUE(cursor);
  const std::vector<std::optional<vehicle::PublishedProgramSource>> prior_sources(2);
  for (std::size_t i = 0; i < 2; ++i) {
    const auto &packet = request.prior_program.commands[i];
    ASSERT_TRUE(ledger.record(packet, packet.published_sec, packet.published_sec, 2));
  }
  auto fresh = scheduled_fresh_world(certificate);
  auto context = certificate.suffix().source.source_context;
  context.decision_id = fresh.decision_id; context.observation_generation++;
  context = contract::seal_problem_context(context);
  const auto check = [&] (const retained::Request &world, const contract::MpccProblemContext &problem, std::size_t sent = 0) {
    return scheduled::check_current_evidence(certificate, world, problem, owner.capture(), ledger, *cursor, prior_sources, sent);
  };
  ASSERT_EQ(check(fresh, context).reason, scheduled::CurrentReason::Compatible);
  auto changed = fresh; changed.decision_id++;
  EXPECT_EQ(check(changed, context).reason, scheduled::CurrentReason::InvalidFrame);
  changed = fresh; changed.publication_prefix->observation.now_sec += .001;
  EXPECT_EQ(check(changed, context).reason, scheduled::CurrentReason::InvalidFrame);
  changed = fresh; changed.publication_prefix->observation.initial.state.forward_velocity_mps += 1;
  EXPECT_EQ(check(changed, context).reason, scheduled::CurrentReason::MeasurementRejected);
  changed = fresh;
  const auto &body = certificate.nominal()->forecast().publication_state;
  changed.obstacles.obstacles.push_back({"new-contact", {body.x_m, body.y_m, 0, 0, .1}});
  EXPECT_EQ(check(changed, context).reason, scheduled::CurrentReason::WorldRejected);
  EXPECT_EQ(check(fresh, context, 1).reason, scheduled::CurrentReason::PrefixRejected);
  auto changed_context = context; changed_context.stage_geometry_id++;
  changed_context = contract::seal_problem_context(changed_context);
  EXPECT_EQ(check(fresh, changed_context).reason, scheduled::CurrentReason::ContextRejected);
  // A real authenticated first suffix send changes the context obligation;
  // just passing sent=1 above could not bypass the new-source geometry gate.
  const auto packet = certificate.suffix().program.commands.front();
  const auto source = scheduled::scheduled_program_source(certificate, 0); ASSERT_TRUE(source);
  ASSERT_TRUE(ledger.record(packet, packet.published_sec, packet.published_sec, 2, source));
  fresh = scheduled_fresh_world(certificate, 1.1, true);
  EXPECT_EQ(check(fresh, changed_context, 1).reason, scheduled::CurrentReason::Compatible);
  owner.invalidate();
  EXPECT_EQ(check(fresh, changed_context, 1).reason, scheduled::CurrentReason::ContextRejected);
}

TEST(MpccScheduledContext, LateProofCannotBorrowCurrentGenerationForAnOldSolverSource)
{
  scheduled::ContextOwner owner;
  auto request = scheduled_request();
  attach_synthetic_source_generation(request, owner.capture());
  const auto original = request.observed.plan->solver_source_snapshot;
  ASSERT_TRUE(original->normal_context_generation.valid());
  owner.invalidate();
  const auto current = owner.capture();
  const auto result = scheduled::evaluate(request);
  ASSERT_TRUE(result.applied.certificate);
  EXPECT_FALSE(result.applied.certificate->nominal()->source_context().valid());
  EXPECT_FALSE(result.applied.certificate->nominal()->source_context().same_generation(current));
  EXPECT_NE(scheduled::check_current_context(*result.applied.certificate, current,
    result.applied.certificate->suffix().source.source_context, scheduled::ContextUse::NewSource),
    scheduled::ContextReason::Compatible);
  EXPECT_EQ(request.observed.plan->solver_source_snapshot, original);
}


struct ScheduledDispatchFixture {
  scheduled::ContextOwner owner;
  scheduled::Request request{scheduled_request()};
  std::shared_ptr<const applied::ScheduledCertificate> certificate;
  vehicle::PublishedInputLedger ledger{256};
  std::optional<vehicle::PublishedInputLedger::Snapshot> original_cursor;
  std::vector<std::optional<vehicle::PublishedProgramSource>> prior_sources{2};
  retained::Request fresh;
  contract::MpccProblemContext context;

  explicit ScheduledDispatchFixture(double last_prior_after = 1.075, bool initial_clock_floor = false,
    contract::ControlIntent intent = contract::ControlIntent::Track) : request(scheduled_request(intent)) {
    if (initial_clock_floor) {
      request.preceding_packet_count = 0;
      request.prior_program.commands = {{1.055,-3,
        request.observed.publication_prefix->observation.commands.back().wire_steering_rad}};
      request.prior_program.nanosecond_clock = vehicle::publication_nanosecond_clock(1.055,.025,.025);
      request.planned_control_origin_sec = 1.14;
      prior_sources.clear();
      // The original frame is at1.05, not1.0. Add its real causal-floor
      // predecessor and rebuild all dependent point-prefix values.
      auto &observed = request.observed;
      auto observation = observed.publication_prefix->observation;
      auto last = observation.commands.back(); last.published_sec = 1.05;
      observation.commands.push_back(last);
      auto prediction = vehicle::predict_prospective_publication(observation,
        observed.publication_prefix->proposed_packet, observed.plan->execution_artifact->vehicle_model);
      if (!prediction) throw std::runtime_error("causal predecessor source prefix unavailable");
      const auto &origin = prediction->control_origin;
      observed.control_pose = {origin.x_m,origin.y_m,origin.yaw_rad};
      observed.control_origin_physical_progress_m = origin.x_m;
      observed.current_speed_mps = prediction->current.forward_velocity_mps;
      observed.control_origin_speed_mps = origin.forward_velocity_mps;
      observed.current_time_steering_rad = prediction->current.tire_steering_rad;
      observed.current_steering_rad = origin.desired_steering_rad;
      observed.current_response_steering_rad = origin.tire_steering_rad;
      observed.current_lateral_velocity_mps = origin.lateral_velocity_mps;
      observed.current_yaw_rate_radps = origin.yaw_rate_radps;
      observed.previous_published_command_age_sec = 0;
      observed.measured_to_control_path.clear(); observed.measured_to_control_elapsed_sec.clear();
      for (const auto &sample : prediction->current_to_control) {
        observed.measured_to_control_path.push_back({sample.state.x_m,sample.state.y_m,sample.state.yaw_rad});
        observed.measured_to_control_elapsed_sec.push_back(sample.source_sec-observed.now_sec);
      }
      observed.publication_prefix = std::move(prediction);
    }
    attach_synthetic_source_generation(request, owner.capture());
    const auto proof_result = scheduled::evaluate(request);
    certificate = proof_result.applied.certificate;
    if (!certificate) throw std::runtime_error(std::string{"dispatch fixture certificate unavailable/"} +
      retained::to_string(proof_result.reason) + "/physical:" + std::to_string(static_cast<int>(proof_result.applied.reason)) +
      "/program:" + std::to_string(static_cast<int>(proof_result.applied.program_reason)));
    for (const auto &packet : certificate->tube().observation.commands) {
      const double raw = initial_clock_floor && packet.published_sec == 1.05 ? 1.04 : packet.published_sec;
      if (!ledger.record(packet, raw, raw, 2))
        throw std::runtime_error("dispatch fixture initial ledger unavailable");
    }
    original_cursor = ledger.snapshot();
    for (std::size_t i = 0; i < request.preceding_packet_count; ++i) {
      const auto &packet = request.prior_program.commands[i];
      if (!ledger.record(packet, packet.published_sec, i == 1 ? last_prior_after : packet.published_sec, 2))
        throw std::runtime_error("dispatch fixture prior send unavailable");
    }
    if (initial_clock_floor) {
      fresh = certificate->nominal()->observed();
      fresh.now_sec = 1.055; fresh.control_origin_sec = 1.095; fresh.decision_id++;
      fresh.publication_prefix->observation.now_sec = fresh.now_sec;
      fresh.publication_prefix->observation.control_origin_sec = fresh.control_origin_sec;
      fresh.obstacles.generation++; fresh.obstacles.observed_sec = fresh.now_sec;
    } else {
      fresh = scheduled_fresh_world(*certificate);
    }
    bind_next(0);
  }
  void bind_next(std::size_t index) {
    auto observation = fresh.publication_prefix->observation;
    observation.commands = ledger.history();
    auto packet = certificate->suffix().program.commands.at(
      std::min(index, certificate->suffix().program.commands.size()-1));
    packet.published_sec = fresh.now_sec;
    auto prediction = vehicle::predict_prospective_publication(observation, packet,
      fresh.plan->execution_artifact->vehicle_model);
    if (!prediction) throw std::runtime_error("dispatch fixture current packet prefix unavailable");
    const auto &origin = prediction->control_origin;
    fresh.control_pose = {origin.x_m, origin.y_m, origin.yaw_rad};
    fresh.control_origin_physical_progress_m = origin.x_m;
    fresh.current_speed_mps = prediction->current.forward_velocity_mps;
    fresh.control_origin_speed_mps = origin.forward_velocity_mps;
    fresh.current_time_steering_rad = prediction->current.tire_steering_rad;
    fresh.current_steering_rad = origin.desired_steering_rad;
    fresh.current_response_steering_rad = origin.tire_steering_rad;
    fresh.current_lateral_velocity_mps = origin.lateral_velocity_mps;
    fresh.current_yaw_rate_radps = origin.yaw_rate_radps;
    fresh.measured_to_control_path.clear(); fresh.measured_to_control_elapsed_sec.clear();
    for (const auto &sample : prediction->current_to_control) {
      fresh.measured_to_control_path.push_back({sample.state.x_m,sample.state.y_m,sample.state.yaw_rad});
      fresh.measured_to_control_elapsed_sec.push_back(sample.source_sec-fresh.now_sec);
    }
    fresh.previous_published_command_age_sec = fresh.now_sec-observation.commands.back().published_sec;
    fresh.publication_prefix = std::move(prediction);
    context = certificate->suffix().source.source_context;
    context.decision_id = fresh.decision_id; context.observation_generation++;
    if (contract::canonical_normal_intent_requires_target_observation(context.intent))
      context.target_obstacle_generation = fresh.obstacles.generation;
    if (context.dynamic_obstacle_constraint_active) context.dynamic_obstacle_generation = fresh.obstacles.generation;
    context = contract::seal_problem_context(context);
  }
  scheduled::DispatchResult prepare(std::size_t sent = 0) {
    return scheduled::prepare_dispatch(certificate, fresh, context, owner.capture(),
      ledger, *original_cursor, prior_sources, sent);
  }
};

TEST(MpccScheduledDispatch, RetainsOriginalPhysicalSerializationWitnessesWithTheWireTail)
{
  ScheduledDispatchFixture f;
  const auto &suffix = f.certificate->suffix();
  ASSERT_EQ(suffix.physical_steering_rad.size(), suffix.program.commands.size());
  ASSERT_FALSE(suffix.physical_steering_rad.empty());
  const double gain = f.request.observed.plan->execution_artifact->vehicle_model.steering_wire_gain;
  for (std::size_t index = 0; index < suffix.physical_steering_rad.size(); ++index)
    EXPECT_DOUBLE_EQ(multi_purpose_mpc_ros::mpcc_wire_command::steering(suffix.physical_steering_rad[index], gain),
      suffix.program.commands[index].wire_steering_rad);
  EXPECT_DOUBLE_EQ(suffix.physical_steering_rad.front(),
    f.certificate->nominal()->proof().terminal_stop_actuation_samples.front().end_steering_rad);
}

TEST(MpccScheduledDispatch, BindsCurrentDecisionAndStrictRawPublicationWindow)
{
  ScheduledDispatchFixture f; const auto result=f.prepare();
  ASSERT_TRUE(result.candidate) << static_cast<int>(result.reason) << '/' << static_cast<int>(result.current.reason);
  const auto &candidate=*result.candidate; const auto &packet=candidate.packet();
  EXPECT_EQ(candidate.source().decision_id, f.certificate->suffix().decision_id);
  EXPECT_EQ(candidate.dispatch_decision_id(), f.fresh.decision_id);
  EXPECT_NE(candidate.source().decision_id, candidate.dispatch_decision_id());
  const auto before=[&](double clock, std::uint64_t id, double acc, double steer) {
    return candidate.matches_before_publication(f.ledger, f.owner.capture(), id, clock, acc, steer);
  };
  EXPECT_TRUE(before(1.1,f.fresh.decision_id,packet.wire_acceleration_mps2,packet.wire_steering_rad));
  EXPECT_FALSE(before(1.099999999,f.fresh.decision_id,packet.wire_acceleration_mps2,packet.wire_steering_rad));
  EXPECT_FALSE(before(1.125000001,f.fresh.decision_id,packet.wire_acceleration_mps2,packet.wire_steering_rad));
  EXPECT_FALSE(before(1.1,f.fresh.decision_id+1,packet.wire_acceleration_mps2,packet.wire_steering_rad));
  EXPECT_FALSE(before(1.1,f.fresh.decision_id,packet.wire_acceleration_mps2+1,packet.wire_steering_rad));
  EXPECT_FALSE(before(1.1,f.fresh.decision_id,packet.wire_acceleration_mps2,
    std::nextafter(static_cast<float>(packet.wire_steering_rad),INFINITY)));
  ASSERT_TRUE(f.ledger.record(packet,1.1,1.105,2,candidate.source()));
  EXPECT_TRUE(candidate.matches_after_publication(f.ledger, f.owner.capture()));
  EXPECT_FALSE(before(1.105,f.fresh.decision_id,packet.wire_acceleration_mps2,packet.wire_steering_rad));
}

TEST(MpccScheduledDispatch, ActualLateMissingExtraWrongSourceOrRevokedSendCannotCommit)
{
  for(int variant=0;variant<6;++variant) {
    SCOPED_TRACE(variant); ScheduledDispatchFixture f; const auto result=f.prepare(); ASSERT_TRUE(result.candidate);
    const auto &candidate=*result.candidate; auto source=candidate.source(); const auto packet=candidate.packet();
    if(variant==0) source.packet_index++;
    if(variant!=1) { ASSERT_TRUE(f.ledger.record(packet,1.1,variant==2?1.125000001:1.1,2,source)); }
    if(variant==3) { ASSERT_TRUE(f.ledger.record(packet,1.1,1.1,2,source)); }
    if(variant==4) f.owner.invalidate();
    if(variant==5) f.ledger.reset();
    EXPECT_FALSE(candidate.matches_after_publication(f.ledger, f.owner.capture()));
  }
}

TEST(MpccScheduledDispatch, CurrentWorldAndActualPrefixAreMandatoryBeforePreparingAPacket)
{
  ScheduledDispatchFixture f;
  auto changed=f.fresh; changed.publication_prefix->proposed_packet.wire_acceleration_mps2+=1;
  EXPECT_EQ(scheduled::prepare_dispatch(f.certificate,changed,f.context,f.owner.capture(),f.ledger,
    *f.original_cursor,f.prior_sources,0).reason, scheduled::DispatchReason::InvalidPacket);
  EXPECT_FALSE(f.prepare(1).candidate);
  changed=f.fresh; const auto &body=f.certificate->nominal()->forecast().publication_state;
  changed.obstacles.obstacles.push_back({"new-contact",{body.x_m,body.y_m,0,0,.1}});
  const auto conflict=scheduled::prepare_dispatch(f.certificate,changed,f.context,f.owner.capture(),f.ledger,
    *f.original_cursor,f.prior_sources,0);
  EXPECT_FALSE(conflict.candidate); EXPECT_EQ(conflict.current.reason,scheduled::CurrentReason::WorldRejected);
  const auto ready=f.prepare(); ASSERT_TRUE(ready.candidate);
  f.owner.invalidate(); const auto &packet=ready.candidate->packet();
  EXPECT_FALSE(ready.candidate->matches_before_publication(f.ledger, f.owner.capture(), f.fresh.decision_id, 1.1, packet.wire_acceleration_mps2, packet.wire_steering_rad));
}

TEST(MpccScheduledDispatch, SlewUsesActualPreviousCompletionRatherThanNominalPeriod)
{
  ScheduledDispatchFixture f(1.099999999);
  const auto result=f.prepare(); ASSERT_TRUE(result.candidate) << static_cast<int>(result.reason) << '/' << static_cast<int>(result.current.reason);
  const auto &candidate=*result.candidate;const auto &packet=candidate.packet();
  const double gain=f.fresh.plan->execution_artifact->vehicle_model.steering_wire_gain;
  ASSERT_GT(std::abs(packet.wire_steering_rad-f.ledger.latest_transaction()->nominal.wire_steering_rad)/gain,
    f.fresh.plan->execution_artifact->physical_global_tolerance);
  EXPECT_FALSE(candidate.matches_before_publication(f.ledger, f.owner.capture(), f.fresh.decision_id, 1.1, packet.wire_acceleration_mps2, packet.wire_steering_rad));
}

TEST(MpccScheduledDispatch, NextRealSuffixSlotKeepsOriginalProofAndRequiresTheFirstActualSend)
{
  ScheduledDispatchFixture f;const auto first=f.prepare(); ASSERT_TRUE(first.candidate);
  const auto fingerprint=f.certificate->tube().context_fingerprint;
  ASSERT_TRUE(f.ledger.record(first.candidate->packet(),1.1,1.1,2,first.candidate->source()));
  f.fresh=scheduled_fresh_world(*f.certificate,1.125,true);f.bind_next(1);
  f.context.stage_geometry_id++;f.context=contract::seal_problem_context(f.context);
  const auto next=f.prepare(1);ASSERT_TRUE(next.candidate) << static_cast<int>(next.reason) << '/' << static_cast<int>(next.current.reason);
  EXPECT_EQ(next.candidate->certificate(),f.certificate);
  EXPECT_EQ(next.candidate->certificate()->tube().context_fingerprint,fingerprint);
  EXPECT_EQ(next.candidate->source().packet_index,1U);
  EXPECT_DOUBLE_EQ(next.candidate->packet().published_sec,1.125);
  const auto &packet=next.candidate->packet();
  EXPECT_TRUE(next.candidate->matches_before_publication(f.ledger, f.owner.capture(), f.fresh.decision_id, 1.125, packet.wire_acceleration_mps2, packet.wire_steering_rad));
}


TEST(MpccScheduledDispatch, NominalProofAlreadyBoundsCausalPredecessorWhenRawClockLags)
{
  ScheduledDispatchFixture f(1.075,true);
  const auto prepared=f.prepare(); ASSERT_TRUE(prepared.candidate) << static_cast<int>(prepared.reason) << '/' << static_cast<int>(prepared.current.reason);
  const auto &candidate=*prepared.candidate; const auto &packet=candidate.packet();
  const auto &previous=*f.ledger.latest_transaction();
  const auto &execution=*f.fresh.plan->execution_artifact;
  const double step=std::abs(packet.wire_steering_rad-previous.nominal.wire_steering_rad)/execution.vehicle_model.steering_wire_gain;
  const double tolerance=execution.physical_global_tolerance;
  ASSERT_DOUBLE_EQ(previous.after_clock_sec,1.04);
  ASSERT_DOUBLE_EQ(previous.published.published_sec,1.05);
  const double causal_bound=execution.maximum_abs_steering_rate_radps*(1.055-previous.published.published_sec)+tolerance;
  const double raw_bound=execution.maximum_abs_steering_rate_radps*(1.055-previous.after_clock_sec)+tolerance;
  ASSERT_LT(causal_bound,raw_bound);
  // The first-packet builder already limits this example to the recorded
  // causal predecessor. The suspected unsafe first packet was not reproduced.
  ASSERT_LE(step,causal_bound);
  EXPECT_TRUE(candidate.matches_before_publication(f.ledger, f.owner.capture(), f.fresh.decision_id, 1.055, packet.wire_acceleration_mps2, packet.wire_steering_rad));
}

TEST(MpccScheduledObservation, RebuildsPostSendDependenciesWithoutPublishingTheNextPacket)
{
  ScheduledDispatchFixture fixture;
  const auto dispatch = fixture.prepare(); ASSERT_TRUE(dispatch.candidate);
  const auto first = dispatch.candidate;
  ASSERT_TRUE(fixture.ledger.record(first->packet(), 1.1, 1.1, 2, first->source()));
  auto observation = fixture.fresh.publication_prefix->observation;
  observation.now_sec = 1.105; observation.control_origin_sec = 1.235;
  observation.commands = fixture.ledger.history();
  observation.initial.state.desired_steering_rad = first->physical_steering_rad();
  auto next = fixture.certificate->suffix().program.commands[1]; next.published_sec = observation.now_sec;
  const auto rebuilt = scheduled::bind_current_observation(fixture.fresh, observation,
    scheduled::ProgressFrame{{0,0,0},0}, next);
  ASSERT_TRUE(rebuilt);
  EXPECT_EQ(rebuilt->now_sec, observation.now_sec);
  EXPECT_EQ(rebuilt->control_origin_sec, observation.control_origin_sec);
  EXPECT_EQ(rebuilt->previous_published_steering_rad, first->physical_steering_rad());
  EXPECT_DOUBLE_EQ(rebuilt->previous_published_command_age_sec, observation.now_sec - 1.1);
  EXPECT_EQ(rebuilt->publication_prefix->observation.commands.size(), fixture.ledger.history().size());
  EXPECT_EQ(fixture.ledger.latest_transaction()->nominal.published_sec, 1.1);
  EXPECT_EQ(rebuilt->publication_prefix->observation.initial.source_sec, observation.initial.source_sec);
  EXPECT_EQ(rebuilt->publication_prefix->observation.tire_source_sec, observation.tire_source_sec);
  EXPECT_EQ(rebuilt->control_pose.x_m, rebuilt->publication_prefix->control_origin.x_m);
  EXPECT_EQ(rebuilt->control_origin_physical_progress_m, rebuilt->control_pose.x_m);
  EXPECT_TRUE(retained::current_publication_prefix_matches(*rebuilt));
  auto stale = fixture.fresh; stale.now_sec = observation.now_sec;
  EXPECT_FALSE(retained::current_publication_prefix_matches(stale));
}

TEST(MpccScheduledObservation, ExactFutureEpochCannotBeInsertedIntoAnActualCurrentPrefix)
{
  ScheduledDispatchFixture fixture;
  const auto observation = fixture.fresh.publication_prefix->observation;
  auto packet = fixture.certificate->suffix().program.commands[1];
  ASSERT_GT(packet.published_sec, observation.now_sec);
  EXPECT_FALSE(scheduled::bind_current_observation(fixture.fresh, observation,
    scheduled::ProgressFrame{{0,0,0},0}, packet));
  packet.published_sec = observation.now_sec;
  auto rebuilt = scheduled::bind_current_observation(fixture.fresh, observation,
    scheduled::ProgressFrame{{0,0,0},0}, packet);
  ASSERT_TRUE(rebuilt);
  EXPECT_EQ(rebuilt->publication_prefix->observation.commands.size(), observation.commands.size());
  auto invalid = observation; invalid.commands.back().published_sec = observation.now_sec + 1;
  EXPECT_FALSE(scheduled::bind_current_observation(fixture.fresh, invalid,
    scheduled::ProgressFrame{{0,0,0},0}, packet));
}

TEST(MpccScheduledObservation, LargeMapCoordinatesKeepTheExactCapturedFrameAssociation)
{
  ScheduledDispatchFixture fixture;
  auto observation=fixture.fresh.publication_prefix->observation;
  // r45 D1 decision552: the two plausible addition orders differ by one ULP.
  const scheduled::ProgressFrame frame{{89631.78902878,43128.72920636,2.416480975901258},28.773461394486684};
  observation.initial.state={89631.01605908728,43128.034320916144,2.12649352748336,0,0,0,0,0};
  for(auto &packet:observation.commands) {packet.wire_acceleration_mps2=-3;packet.wire_steering_rad=0;}
  const vehicle::PublishedCommand packet{observation.now_sec,-3,0};
  const auto rebuilt=scheduled::bind_current_observation(fixture.fresh,observation,frame,packet);
  ASSERT_TRUE(rebuilt);
  const auto &pose=rebuilt->control_pose;
  const double dx=std::cos(frame.pose.yaw_rad)*(pose.x_m-frame.pose.x_m);
  const double dy=std::sin(frame.pose.yaw_rad)*(pose.y_m-frame.pose.y_m);
  const double correct=frame.progress_m+(dx+dy);
  ASSERT_NE((frame.progress_m+dx)+dy,correct);
  EXPECT_EQ(rebuilt->control_origin_physical_progress_m,correct);
  EXPECT_EQ(scheduled::project_progress(frame,pose),correct);
  EXPECT_TRUE(retained::current_publication_prefix_matches(*rebuilt));
  auto invalid=frame;invalid.pose.yaw_rad=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(scheduled::project_progress(invalid,pose));
}

TEST(MpccScheduledObservation, CapturedAppointmentsPreserveTheCanonicalNanosecondOrigin)
{
  for (const std::int64_t first_ns : {319999992,334999992}) {
    const double first=static_cast<double>(first_ns)/1e9;
    vehicle::PublishedInputProgram program{.025,{{first,-3,0}},true,.025};
    program.nanosecond_clock=vehicle::publication_nanosecond_clock(first,.025,.025);
    const auto origin=scheduled::publication_control_origin(program,1,.13);
    ASSERT_TRUE(origin);
    const std::int64_t expected_ns=first_ns+25000000+130000000;
    EXPECT_EQ(vehicle::publication_nanoseconds(*origin),expected_ns);
    EXPECT_EQ(*origin,static_cast<double>(expected_ns)/1e9);
    EXPECT_FALSE(vehicle::publication_nanoseconds(static_cast<double>(expected_ns)*1e-9));
    EXPECT_FALSE(scheduled::publication_control_origin(program,1,-.13));
    EXPECT_FALSE(scheduled::publication_control_origin(program,10001,.13));
  }
}

TEST(MpccScheduledObservation, FreshFollowUsesActualPeerPositionOnTheOriginalBranch)
{
  namespace core = multi_purpose_mpc_ros::v2x_overtake_core;
  const std::vector<core::CoursePoint> course{{0,0},{10,0},{10,1},{0,1},{0,2},{10,2}};
  core::ForwardCourseProjectionRequest query{0,false,1,0,8,1.2,1,0,2,40,3,std::nullopt,1.5};
  const scheduled::FollowCourseAnchor anchor{"peer",1,8,1};
  ASSERT_EQ(core::project_forward_course_progress(course,query).segment_index,4U);
  const auto same = scheduled::project_follow_at_observation(course,query,anchor,1);
  ASSERT_TRUE(same.valid); EXPECT_EQ(same.segment_index,0U); EXPECT_DOUBLE_EQ(same.forward_distance_m,7);
  query.origin_x_m = 1.02; query.target_x_m = 8.05;
  const auto fresh = scheduled::project_follow_at_observation(course,query,anchor,1.05);
  ASSERT_TRUE(fresh.valid); EXPECT_EQ(fresh.segment_index,0U);
  EXPECT_NEAR(fresh.forward_distance_m,7.03,1e-12);
  // The retired stale-gap formula advances only ego:7-.02=6.98.
  EXPECT_GT(fresh.forward_distance_m,7-.02);
  EXPECT_FALSE(scheduled::project_follow_at_observation(course,query,anchor,.99).valid);
  query.target_x_m = 3; // Actual discontinuous target cannot borrow the old branch anchor.
  EXPECT_FALSE(scheduled::project_follow_at_observation(course,query,anchor,1.05).valid);
}

TEST(MpccScheduledDispatch, FinalIdentityRequiresTheActualSendAndKeepsBothDecisions)
{
  ScheduledDispatchFixture fixture;
  const auto result=fixture.prepare(); ASSERT_TRUE(result.candidate);
  const auto &dispatch=*result.candidate;
  const auto command=dispatch.canonical_command();
  EXPECT_NE(command.decision_id,command.execution_certificate_decision_id);
  EXPECT_FALSE(dispatch.publication_identity(fixture.ledger, fixture.owner.capture()));
  ASSERT_TRUE(fixture.ledger.record(dispatch.packet(),1.1,1.105,2,dispatch.source()));
  const auto receipt=dispatch.publication_identity(fixture.ledger, fixture.owner.capture());
  ASSERT_TRUE(receipt); EXPECT_TRUE(receipt->matches(command));
  auto changed=command; changed.decision_id++; EXPECT_FALSE(receipt->matches(changed));
  changed=command; changed.execution_certificate_decision_id=command.decision_id; EXPECT_FALSE(receipt->matches(changed));
  changed=command; changed.acceleration_mps2+=1; EXPECT_FALSE(receipt->matches(changed));
  contract::CertifiedMpccSolution solution;
  solution.solution_id=command.solution_id; solution.problem_fingerprint=command.problem_fingerprint;
  solution.formulation=command.formulation; solution.solved=true; solution.finite=true; solution.constraints_satisfied=true;
  solution.maximum_constraint_violation=0; solution.physical={true,true,true,1,2};
  solution.prediction_stage_count=fixture.certificate->suffix().source.source_context.horizon_steps;
  solution.valid_until_sec=fixture.certificate->tube().rest_sec;
  contract::FinalControlDecisionRequest request{command.decision_id,contract::FinalAuthorityClass::CertifiedNormalSolution,
    "scheduled",fixture.certificate->suffix().source.source_context,solution,true,command};
  EXPECT_FALSE(contract::resolve_final_control_decision(request).identity_complete);
  request.scheduled_publication=receipt;
  const auto accepted=contract::resolve_final_control_decision(request);
  EXPECT_TRUE(accepted.identity_complete); EXPECT_TRUE(accepted.canonical_contract_satisfied);
  EXPECT_EQ(accepted.execution_certificate_decision_id,command.execution_certificate_decision_id);
  request.decision_id++; EXPECT_FALSE(contract::resolve_final_control_decision(request).identity_complete);
  fixture.owner.invalidate(); EXPECT_FALSE(dispatch.publication_identity(fixture.ledger, fixture.owner.capture()));
}

TEST(MpccScheduledDispatch, UsesImmutablePlanningClockWhileCurrentCheckOccursInsideTheWindow)
{
  ScheduledDispatchFixture fixture;
  auto observation=fixture.fresh.publication_prefix->observation;
  observation.now_sec=1.11; observation.control_origin_sec=1.15;
  auto packet=fixture.certificate->suffix().program.commands.front(); packet.published_sec=observation.now_sec;
  const auto fresh=scheduled::bind_current_observation(fixture.fresh,observation,{{0,0,0},0},packet);
  ASSERT_TRUE(fresh); fixture.fresh=*fresh; fixture.fresh.obstacles.observed_sec=observation.now_sec;
  const auto result=fixture.prepare(); ASSERT_TRUE(result.candidate);
  const auto &dispatch=*result.candidate;
  EXPECT_LT(fixture.certificate->nominal()->observed().now_sec,dispatch.packet().published_sec);
  EXPECT_LT(dispatch.packet().published_sec,observation.now_sec);
  EXPECT_TRUE(dispatch.matches_before_publication(fixture.ledger,fixture.owner.capture(),fixture.fresh.decision_id,
    1.115,packet.wire_acceleration_mps2,packet.wire_steering_rad));
  EXPECT_FALSE(dispatch.matches_before_publication(fixture.ledger,fixture.owner.capture(),fixture.fresh.decision_id,
    1.099999999,packet.wire_acceleration_mps2,packet.wire_steering_rad));
  EXPECT_FALSE(dispatch.matches_before_publication(fixture.ledger,fixture.owner.capture(),fixture.fresh.decision_id,
    1.125000001,packet.wire_acceleration_mps2,packet.wire_steering_rad));
  ASSERT_TRUE(fixture.ledger.record(dispatch.packet(),1.115,1.12,2,dispatch.source()));
  EXPECT_TRUE(dispatch.matches_after_publication(fixture.ledger,fixture.owner.capture()));
}


TEST(MpccScheduledDispatch, IndependentCurrentPopulationKeepsOriginalProgrammeAndActualWindow)
{
  for (double now : {1.1, 1.115}) {
    SCOPED_TRACE(now);
    ScheduledDispatchFixture f;
    f.fresh = scheduled_fresh_world(*f.certificate, now);
    f.fresh.publication_prefix->observation.initial.state.x_m += .001;
    f.bind_next(0);
    const auto old_hash = f.certificate->tube().context_fingerprint;
    EXPECT_EQ(scheduled::check_measurement_consistency(*f.certificate,
      f.fresh.publication_prefix->observation).reason, scheduled::MeasurementReason::PoseMismatch);
    const auto result = f.prepare();
    ASSERT_TRUE(result.candidate) << static_cast<int>(result.reason) << '/' << static_cast<int>(result.current.world.physical_reason);
    const auto &candidate = *result.candidate;
    ASSERT_TRUE(candidate.current_physical_proof());
    const auto &proof = *candidate.current_physical_proof();
    EXPECT_EQ(proof.original_input_fingerprint(), old_hash);
    EXPECT_NE(candidate.physical_input_fingerprint(), old_hash);
    EXPECT_EQ(candidate.source().input_context_fingerprint, old_hash);
    EXPECT_EQ(proof.observed().decision_id, f.fresh.decision_id);
    EXPECT_EQ(proof.tube().numerical.observation.now_sec, now);
    EXPECT_EQ(proof.first_suffix_index(), 0U);
    EXPECT_EQ(proof.tube().numerical.program.commands.size(), f.certificate->suffix().program.commands.size());
    for (std::size_t i = 0; i < proof.tube().numerical.program.commands.size(); ++i) {
      const auto &a = proof.tube().numerical.program.commands[i];
      const auto &b = f.certificate->suffix().program.commands[i];
      EXPECT_EQ(a.published_sec, b.published_sec);
      EXPECT_EQ(a.wire_acceleration_mps2, b.wire_acceleration_mps2);
      EXPECT_EQ(a.wire_steering_rad, b.wire_steering_rad);
    }
    const auto &packet = candidate.packet();
    EXPECT_TRUE(candidate.matches_before_publication(f.ledger, f.owner.capture(), f.fresh.decision_id,
      now, packet.wire_acceleration_mps2, packet.wire_steering_rad));
    EXPECT_FALSE(candidate.matches_before_publication(f.ledger, f.owner.capture(), f.fresh.decision_id,
      1.125000001, packet.wire_acceleration_mps2, packet.wire_steering_rad));
    ASSERT_TRUE(f.ledger.record(packet, now, now, 2, candidate.source()));
    EXPECT_TRUE(candidate.matches_after_publication(f.ledger, f.owner.capture()));
    f.owner.invalidate();
    EXPECT_FALSE(candidate.matches_after_publication(f.ledger, f.owner.capture()));
  }
}

TEST(MpccScheduledDispatch, IndependentProofStillRejectsWorldContextEpochAndHistoryFailures)
{
  for (int variant = 0; variant < 7; ++variant) {
    SCOPED_TRACE(variant);
    ScheduledDispatchFixture f;
    f.fresh.publication_prefix->observation.initial.state.x_m += .001;
    if (variant == 4) {
      f.fresh.now_sec = 1.125000001;
      f.fresh.publication_prefix->observation.now_sec = f.fresh.now_sec;
      f.fresh.control_origin_sec = f.fresh.now_sec + .04;
      f.fresh.publication_prefix->observation.control_origin_sec = f.fresh.control_origin_sec;
    }
    f.bind_next(0);
    if (variant == 2) f.fresh.publication_prefix->observation.initial.source_sec =
      f.certificate->tube().observation.initial.source_sec;
    if (variant == 3) f.fresh.publication_prefix->observation.velocity_source_sec =
      f.certificate->tube().observation.velocity_source_sec - .001;
    if (variant == 0) f.fresh.obstacles.obstacles.push_back({"current-contact",
      {f.fresh.control_pose.x_m, f.fresh.control_pose.y_m, 0, 0, .2}});
    if (variant == 1) { f.context.cost_schema_id += "-changed"; f.context = contract::seal_problem_context(f.context); }
    if (variant == 5) f.owner.invalidate();
    if (variant == 6) {
      auto extra = f.ledger.history().back(); extra.published_sec = f.fresh.now_sec;
      ASSERT_TRUE(f.ledger.record(extra, f.fresh.now_sec, f.fresh.now_sec, 2));
    }
    EXPECT_FALSE(f.prepare().candidate);
  }
}

TEST(MpccScheduledDispatch, IndependentRemainingSuffixRequiresAuthenticatedActualFirstPacket)
{
  ScheduledDispatchFixture f;
  const auto first = f.prepare(); ASSERT_TRUE(first.candidate);
  ASSERT_TRUE(f.ledger.record(first.candidate->packet(), 1.1, 1.1, 2, first.candidate->source()));
  f.fresh = scheduled_fresh_world(*f.certificate, 1.125, true);
  f.fresh.publication_prefix->observation.initial.state.x_m += .001;
  f.bind_next(1);
  const auto next = f.prepare(1);
  ASSERT_TRUE(next.candidate) << static_cast<int>(next.reason) << '/' << static_cast<int>(next.current.world.physical_reason);
  ASSERT_TRUE(next.candidate->current_physical_proof());
  const auto &physical = *next.candidate->current_physical_proof();
  EXPECT_EQ(physical.first_suffix_index(), 1U);
  EXPECT_EQ(physical.tube().numerical.program.commands.front().published_sec, 1.125);
  EXPECT_EQ(physical.tube().numerical.observation.commands.size(), f.ledger.history().size());
  EXPECT_EQ(next.candidate->source().input_context_fingerprint, f.certificate->tube().context_fingerprint);
  EXPECT_FALSE(f.prepare(0).candidate);
  EXPECT_FALSE(f.prepare(2).candidate);
}


TEST(MpccScheduledDispatch, IndependentFollowProofKeepsFreshPhysicalGapAndRejectsUnsafePeer)
{
  ScheduledDispatchFixture f(1.075, false, contract::ControlIntent::Follow);
  f.fresh.publication_prefix->observation.initial.state.x_m += .001;
  f.bind_next(0);
  ASSERT_TRUE(f.fresh.follow_target);
  f.fresh.follow_target->current_target_gap_m -= .001;
  for (auto &progress : f.fresh.follow_target->target_progress_from_current_origin_m) progress -= .001;
  const auto valid = f.prepare();
  ASSERT_TRUE(valid.candidate) << static_cast<int>(valid.reason) << '/' << static_cast<int>(valid.current.world.reason)
    << '/' << static_cast<int>(valid.current.world.physical_reason);
  ASSERT_TRUE(valid.candidate->current_physical_proof());
  EXPECT_GE(valid.current.world.minimum_follow_gap_m, f.fresh.follow_target->hard_gap_m);
  const double unsafe_gap = f.fresh.follow_target->hard_gap_m - .01;
  const double shift = unsafe_gap - f.fresh.follow_target->current_target_gap_m;
  f.fresh.follow_target->current_target_gap_m = unsafe_gap;
  for (auto &progress : f.fresh.follow_target->target_progress_from_current_origin_m) progress += shift;
  const auto invalid = f.prepare();
  EXPECT_FALSE(invalid.candidate);
  EXPECT_EQ(invalid.current.world.physical_reason, applied::Reason::FollowGapRejected);
}


TEST(MpccScheduledCandidateChoice, SelectsEitherNormalAvoidanceDisjunctWithoutChangingSourceOrObservation)
{
  for (auto intent : {contract::ControlIntent::Cruise, contract::ControlIntent::Follow}) {
    for (int side : {-1, 1}) {
      auto source = scheduled_request(intent).observed.plan->execution_artifact->identity.source_context;
      source.dynamic_obstacle_constraint_active = true;
      source.dynamic_obstacle_id = "d2"; source.dynamic_obstacle_generation = 128;
      source.dynamic_obstacle_side_sign = side; source = contract::seal_problem_context(source);
      auto proposed = source;
      proposed.decision_id = 850; proposed.observation_generation = 850;
      proposed.dynamic_obstacle_generation = 132; proposed.dynamic_obstacle_side_sign = 0;
      if (intent == contract::ControlIntent::Follow) proposed.target_obstacle_generation = 132;
      proposed = contract::seal_problem_context(proposed);
      ASSERT_TRUE(contract::problem_context_complete(source)); ASSERT_TRUE(contract::problem_context_complete(proposed));
      const auto selected = scheduled::select_new_source_context(source, proposed);
      ASSERT_TRUE(selected);
      EXPECT_EQ(selected->dynamic_obstacle_side_sign, side);
      EXPECT_EQ(selected->decision_id, 850U); EXPECT_EQ(selected->observation_generation, 850U);
      EXPECT_EQ(selected->dynamic_obstacle_generation, 132U);
      EXPECT_EQ(selected->stage_geometry_id, proposed.stage_geometry_id);
      EXPECT_NE(selected->fingerprint, proposed.fingerprint);
      EXPECT_EQ(proposed.dynamic_obstacle_side_sign, 0);
      EXPECT_EQ(source.dynamic_obstacle_generation, 128U);
      EXPECT_TRUE(contract::problem_context_complete(*selected));
    }
  }
}

TEST(MpccScheduledCandidateChoice, FixedSideMissionTargetGeometryModelAndSchemaStillReject)
{
  auto source = scheduled_request(contract::ControlIntent::Cruise).observed.plan->execution_artifact->identity.source_context;
  source.dynamic_obstacle_constraint_active = true; source.dynamic_obstacle_id = "d2";
  source.dynamic_obstacle_generation = 128; source.dynamic_obstacle_side_sign = 1;
  source = contract::seal_problem_context(source);
  for (int variant = 0; variant < 10; ++variant) {
    SCOPED_TRACE(variant);
    auto proposed = source; proposed.dynamic_obstacle_side_sign = 0;
    if (variant == 0) proposed.dynamic_obstacle_side_sign = -1;
    if (variant == 1) proposed.dynamic_obstacle_id = "different";
    if (variant == 2) proposed.intent_generation++;
    if (variant == 3) proposed.stage_geometry_id++;
    if (variant == 4) proposed.vehicle_model_fingerprint++;
    if (variant == 5) proposed.cost_schema_id += "-changed";
    if (variant == 6) proposed.bounds_schema_id += "-changed";
    if (variant == 7) proposed.horizon_steps++;
    if (variant == 8) proposed.execution_side_sign = -1;
    proposed = contract::seal_problem_context(proposed);
    if (variant == 9) proposed.fingerprint++;
    EXPECT_FALSE(scheduled::select_new_source_context(source, proposed));
  }
  for (auto intent : {contract::ControlIntent::Track, contract::ControlIntent::ShiftOut,
      contract::ControlIntent::Pass, contract::ControlIntent::Return, contract::ControlIntent::Rejoin,
      contract::ControlIntent::Stop, contract::ControlIntent::Hold}) {
    auto other = source; other.intent = intent; other = contract::seal_problem_context(other);
    auto proposed = other; proposed.dynamic_obstacle_side_sign = 0; proposed = contract::seal_problem_context(proposed);
    EXPECT_FALSE(scheduled::select_new_source_context(other, proposed));
  }
  EXPECT_TRUE(scheduled::select_new_source_context(source, source));
}
