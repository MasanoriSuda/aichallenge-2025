#include "multi_purpose_mpc_ros/mpcc_architecture_snapshot.hpp"

#include "multi_purpose_mpc_ros/mpcc_rate_resolved_shadow.hpp"

#include <gtest/gtest.h>

#include <Eigen/Sparse>

#include <filesystem>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace multi_purpose_mpc_ros::mpcc_architecture_snapshot
{
namespace
{

mpcc_rate_resolved_shadow::Snapshot make_snapshot(
  const mpcc_execution_contract::ControlIntent intent)
{
  mpcc_rate_resolved_shadow::Snapshot snapshot;
  snapshot.identity.sequence = 42U;
  snapshot.identity.snapshot_sec = 12.5;
  snapshot.identity.source_context.intent = intent;
  snapshot.identity.source_context.decision_id = 10U;
  snapshot.identity.source_context.intent_generation = 2U;
  snapshot.identity.source_context.observation_generation = 3U;
  snapshot.identity.source_context.stage_geometry_id = 4U;
  snapshot.identity.source_context.target_obstacle_generation = 5U;
  snapshot.identity.source_context.target_id = "d2";
  snapshot.identity.source_context.execution_side_sign = 1;
  snapshot.identity.source_context.horizon_steps = 1U;
  snapshot.identity.source_context.formulation =
    mpcc_execution_contract::Formulation::
    VelocitySteeringYawResponseProgress7State;
  snapshot.identity.source_context.state_schema_id = "state";
  snapshot.identity.source_context.input_schema_id = "input";
  snapshot.identity.source_context.bounds_schema_id = "bounds";
  snapshot.identity.source_context.cost_schema_id = "cost";
  snapshot.identity.source_context.fingerprint = 99U;
  snapshot.control_prediction_origin_sec = 12.6;
  snapshot.course_progress_origin_m = 3.0;
  snapshot.execution_prefix_steps = 1;
  snapshot.publication_interval_sec = 0.025;
  snapshot.request.horizon_steps = 1;
  snapshot.nominal_path_distance_m = {0.0, 1.0};
  return snapshot;
}

mpcc_rate_resolved_shadow::Snapshot make_interaction_snapshot(
  const mpcc_execution_contract::ControlIntent intent)
{
  auto snapshot = make_snapshot(intent);
  snapshot.identity.source_context =
    mpcc_execution_contract::seal_problem_context(
    snapshot.identity.source_context);
  snapshot.request.initial_state.setZero();
  snapshot.request.current_steering_rad = 0.02;
  snapshot.request.current_response_steering_rad = 0.01;
  snapshot.request.wheelbase_m = 1.0;
  snapshot.request.yaw_response_gain = 1.0;
  snapshot.request.yaw_response_time_constant_sec = 0.1;
  snapshot.request.maximum_abs_steering_rad = 0.5;
  snapshot.request.maximum_abs_steering_rate_radps = 1.0;
  snapshot.request.states.resize(2U);
  snapshot.request.inputs.resize(1U);
  snapshot.request.states[0].lower.setConstant(-10.0);
  snapshot.request.states[0].upper.setConstant(10.0);
  snapshot.request.states[1].lower.setConstant(-10.0);
  snapshot.request.states[1].upper.setConstant(10.0);
  snapshot.request.states[0].lower[2] =
    -std::numeric_limits<double>::infinity();
  snapshot.request.states[0].upper[2] =
    std::numeric_limits<double>::infinity();
  snapshot.request.inputs[0].lower.setConstant(-2.0);
  snapshot.request.inputs[0].upper.setConstant(2.0);
  snapshot.request.inputs[0].stage_dt_sec = 0.1;
  snapshot.progress_aligned_wall_refinement_active = true;
  snapshot.wall_reference_progress_m = {3.0, 4.0};
  snapshot.wall_lower_m = {-2.0, -2.0};
  snapshot.wall_upper_m = {2.0, 2.0};
  snapshot.physical_wall_refinement_active = true;
  auto grid = std::make_shared<recovery_footprint::OccupancyGrid>();
  grid->width = 2U;
  grid->height = 2U;
  grid->resolution_m = 0.5;
  grid->cells = {
    recovery_footprint::CellState::Free,
    recovery_footprint::CellState::Free,
    recovery_footprint::CellState::Free,
    recovery_footprint::CellState::Free};
  snapshot.wall_grid = grid;
  snapshot.wall_footprint.front_extent_m = 1.0;
  snapshot.wall_footprint.rear_extent_m = 1.0;
  snapshot.wall_footprint.left_extent_m = 0.725;
  snapshot.wall_footprint.right_extent_m = 0.725;
  snapshot.wall_course_frame_knots = {
    mpc_stage_geometry::CourseFrameKnot{3.0, 0.0, 0.0, 0.0, 10},
    mpc_stage_geometry::CourseFrameKnot{4.0, 1.0, 0.0, 0.0, 11}};
  snapshot.wall_lateral_sample_step_m = 0.1;
  snapshot.wall_translation_bucket_width_m = 0.1;
  snapshot.dynamic_obstacle_refinement_active = true;
  snapshot.dynamic_obstacle_pass_side_sign = 1;
  snapshot.dynamic_obstacle_stages = {
    mpcc_rate_resolved_dynamic_obstacle::StagePrediction{
      true, 4.0, 0.5, 2.0, 1.5}};
  snapshot.replay_world.emplace();
  // Vehicle-state decisions and V2X obstacle observations are independent
  // streams in production.  Keep their generations distinct so replay
  // completeness cannot accidentally bind the dynamic world to the ego
  // decision generation.
  snapshot.replay_world->observation_generation = 5U;
  snapshot.replay_world->observed_sec = 12.5;
  snapshot.replay_world->current = true;
  snapshot.replay_world->current_pose = {0.0, 0.0, 0.0};
  snapshot.replay_world->control_prefix = {
    recovery_footprint::Pose2D{0.0, 0.0, 0.0},
    recovery_footprint::Pose2D{0.1, 0.0, 0.0}};
  snapshot.replay_world->control_prefix_elapsed_sec = {0.0, 0.1};
  snapshot.replay_world->physical_footprint.front_extent_m = 1.0;
  snapshot.replay_world->physical_footprint.rear_extent_m = 1.0;
  snapshot.replay_world->physical_footprint.left_extent_m = 0.525;
  snapshot.replay_world->physical_footprint.right_extent_m = 0.525;
  snapshot.replay_world->wall_grid_fingerprint =
    recovery_footprint::occupancy_grid_fingerprint(*grid);
  snapshot.replay_world->hard_wall_clearance_m = 0.2;
  snapshot.replay_world->bound_tolerance_m = 1e-5;
  snapshot.replay_world->swept_step_m = 0.1;
  snapshot.replay_world->obstacles.push_back(
    mpcc_rate_resolved_shadow::ReplayDynamicObstacle{
      "d2", 4.0, 0.5, 1.0, 0.0, 0.1, 0.0, 0.02, 0.03, 0.8, 5U});
  return snapshot;
}

mpcc_rate_resolved_problem::Problem make_problem()
{
  mpcc_rate_resolved_problem::Problem problem;
  problem.horizon_steps = 1;
  problem.linear_cost = Eigen::VectorXd::Zero(1);
  problem.lower_bound = Eigen::VectorXd::Constant(1, -1.0);
  problem.upper_bound = Eigen::VectorXd::Constant(1, 1.0);
  problem.quadratic_cost.resize(1, 1);
  problem.constraints.resize(1, 1);
  problem.constraints.setIdentity();
  problem.variable_scaling.physical_units_per_solver_unit =
    Eigen::VectorXd::Ones(1);
  return problem;
}

// Avoid iterators into two different temporary vectors in make_problem.
mpcc_rate_resolved_problem::Problem make_valid_problem()
{
  auto problem = make_problem();
  const std::vector<Eigen::Triplet<double>> diagonal{{0, 0, 1.0}};
  problem.quadratic_cost.setFromTriplets(diagonal.begin(), diagonal.end());
  return problem;
}

mpcc_rate_resolved_problem::AssemblyRequest make_assembly_request()
{
  mpcc_rate_resolved_problem::AssemblyRequest request;
  request.horizon_steps = 1;
  request.state_reference = Eigen::VectorXd::Zero(14);
  request.state_lower = Eigen::VectorXd::Constant(14, -1.0);
  request.state_upper = Eigen::VectorXd::Constant(14, 1.0);
  request.state_weight = Eigen::VectorXd::Ones(14);
  request.input_reference = Eigen::VectorXd::Zero(3);
  request.input_lower = Eigen::VectorXd::Constant(3, -1.0);
  request.input_upper = Eigen::VectorXd::Constant(3, 1.0);
  request.input_weight = Eigen::VectorXd::Ones(3);
  request.additional_linear_cost = Eigen::VectorXd::Zero(17);
  return request;
}

std::filesystem::path output_root(const std::string & name)
{
  return std::filesystem::path{::testing::TempDir()} /
    ("mpcc-architecture-snapshot-" + name);
}

TEST(MpccArchitectureSnapshot, WritesLoadsAndReplaysExactProblem)
{
  const auto root = output_root("roundtrip");
  std::filesystem::remove_all(root);
  const auto snapshot = make_snapshot(
    mpcc_execution_contract::ControlIntent::ShiftOut);
  const auto problem = make_valid_problem();
  const auto request = make_assembly_request();
  const std::optional<persistent_osqp::WarmStart> warm_start{
    persistent_osqp::WarmStart{
      Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1)}};
  persistent_osqp::SolveOutcome production_outcome;
  production_outcome.failure_detail = "recorded-test-failure";

  const auto written = record_failure(
    snapshot, request, problem, warm_start, production_outcome,
    PipelineStage::Initial, "unit-roundtrip", "intentional test evidence",
    root);
  ASSERT_EQ(written.status, RecordStatus::Written) << written.detail;
  ASSERT_TRUE(std::filesystem::exists(written.snapshot_file));

  std::string load_detail;
  const auto loaded = load_recorded_qp(written.snapshot_file, &load_detail);
  ASSERT_TRUE(loaded.has_value()) << load_detail;
  EXPECT_EQ(loaded->intent, "shiftout");
  EXPECT_EQ(loaded->pipeline_stage, "initial");
  ASSERT_TRUE(loaded->warm_start.has_value());
  EXPECT_TRUE(loaded->problem.linear_cost.isApprox(problem.linear_cost));
  EXPECT_TRUE(loaded->problem.lower_bound.isApprox(problem.lower_bound));
  EXPECT_TRUE(loaded->problem.upper_bound.isApprox(problem.upper_bound));
  EXPECT_TRUE(
    Eigen::MatrixXd(loaded->problem.quadratic_cost).isApprox(
      Eigen::MatrixXd(problem.quadratic_cost)));

  const auto warm_replay = replay_recorded_qp(written.snapshot_file, true);
  ASSERT_TRUE(warm_replay.loaded) << warm_replay.detail;
  EXPECT_TRUE(warm_replay.warm_start_available);
  EXPECT_TRUE(warm_replay.outcome.result.has_value()) << warm_replay.detail;
  const auto cold_replay = replay_recorded_qp(written.snapshot_file, false);
  ASSERT_TRUE(cold_replay.loaded) << cold_replay.detail;
  EXPECT_TRUE(cold_replay.outcome.result.has_value()) << cold_replay.detail;
}

TEST(MpccArchitectureSnapshot, RoundTripsReplayReadyInteractionSnapshot)
{
  const auto root = output_root("interaction-roundtrip");
  std::filesystem::remove_all(root);
  const auto snapshot = make_interaction_snapshot(
    mpcc_execution_contract::ControlIntent::Pass);
  const auto written = record_failure(
    snapshot, make_assembly_request(), make_valid_problem(), std::nullopt,
    persistent_osqp::SolveOutcome{}, PipelineStage::Initial,
    "unit-interaction-roundtrip", "intentional replay-ready evidence", root);
  ASSERT_EQ(written.status, RecordStatus::Written) << written.detail;

  std::string detail;
  const auto loaded = load_recorded_interaction_snapshot(
    written.snapshot_file, &detail);
  ASSERT_TRUE(loaded.has_value()) << detail;
  EXPECT_TRUE(interaction_snapshot_complete(loaded->source));
  EXPECT_TRUE(
    interaction_snapshot_matches_fingerprint(
      loaded->source, loaded->interaction_fingerprint));
  ASSERT_TRUE(loaded->source.replay_world.has_value());
  EXPECT_EQ(loaded->source.identity.source_context.target_id, "d2");
  EXPECT_EQ(loaded->source.replay_world->observation_generation, 5U);
  EXPECT_EQ(
    loaded->source.replay_world->control_prefix_elapsed_sec,
    (std::vector<double>{0.0, 0.1}));
  EXPECT_DOUBLE_EQ(
    loaded->source.replay_world->physical_footprint.left_extent_m, 0.525);
  ASSERT_EQ(loaded->source.replay_world->obstacles.size(), 1U);
  EXPECT_EQ(loaded->source.replay_world->obstacles.front().id, "d2");
  EXPECT_EQ(loaded->source.wall_grid->cells.size(), 4U);

  auto vehicle_mutated = loaded->source;
  vehicle_mutated.replay_world->obstacles.front().x_m += 0.01;
  EXPECT_FALSE(
    interaction_snapshot_matches_fingerprint(
      vehicle_mutated, loaded->interaction_fingerprint));

  auto wall_mutated = loaded->source;
  wall_mutated.wall_lower_m.front() -= 0.01;
  EXPECT_FALSE(
    interaction_snapshot_matches_fingerprint(
      wall_mutated, loaded->interaction_fingerprint));

  auto timing_mutated = loaded->source;
  timing_mutated.replay_world->control_prefix_elapsed_sec.back() += 0.01;
  EXPECT_FALSE(
    interaction_snapshot_matches_fingerprint(
      timing_mutated, loaded->interaction_fingerprint));

  auto footprint_mutated = loaded->source;
  footprint_mutated.replay_world->physical_footprint.left_extent_m += 0.01;
  EXPECT_FALSE(
    interaction_snapshot_matches_fingerprint(
      footprint_mutated, loaded->interaction_fingerprint));

  auto identity_mutated = loaded->source;
  ++identity_mutated.identity.source_context.decision_id;
  EXPECT_FALSE(
    interaction_snapshot_matches_fingerprint(
      identity_mutated, loaded->interaction_fingerprint));

  auto semantic_mutated = loaded->source;
  semantic_mutated.request.initial_state[0] += 0.01;
  EXPECT_FALSE(
    interaction_snapshot_matches_fingerprint(
      semantic_mutated, loaded->interaction_fingerprint));
}

TEST(MpccArchitectureSnapshot, OldExactQpSnapshotIsNotInteractionReplayReady)
{
  const auto root = output_root("interaction-incomplete");
  std::filesystem::remove_all(root);
  const auto written = record_failure(
    make_snapshot(mpcc_execution_contract::ControlIntent::ShiftOut),
    make_assembly_request(), make_valid_problem(), std::nullopt,
    persistent_osqp::SolveOutcome{}, PipelineStage::Initial,
    "unit-interaction-incomplete", "missing replay world", root);
  ASSERT_EQ(written.status, RecordStatus::Written) << written.detail;
  EXPECT_TRUE(load_recorded_qp(written.snapshot_file).has_value());

  std::string detail;
  EXPECT_FALSE(
    load_recorded_interaction_snapshot(written.snapshot_file, &detail).has_value());
  EXPECT_EQ(detail, "interaction snapshot incomplete");
}

TEST(MpccArchitectureSnapshot, DeduplicatesFailureFamilyPerProcess)
{
  const auto root = output_root("dedup");
  std::filesystem::remove_all(root);
  const auto snapshot = make_snapshot(
    mpcc_execution_contract::ControlIntent::Pass);
  const auto problem = make_valid_problem();
  const auto request = make_assembly_request();
  const persistent_osqp::SolveOutcome outcome;
  const auto first = record_failure(
    snapshot, request, problem, std::nullopt, outcome,
    PipelineStage::WallRefinement, "unit-dedup", "first", root);
  const auto second = record_failure(
    snapshot, request, problem, std::nullopt, outcome,
    PipelineStage::WallRefinement, "unit-dedup", "second", root);
  EXPECT_EQ(first.status, RecordStatus::Written) << first.detail;
  EXPECT_EQ(second.status, RecordStatus::Duplicate) << second.detail;
}

TEST(MpccArchitectureSnapshot, RefusesNonOvertakeCapture)
{
  const auto snapshot = make_snapshot(
    mpcc_execution_contract::ControlIntent::Cruise);
  const auto result = record_failure(
    snapshot, make_assembly_request(), make_valid_problem(), std::nullopt,
    persistent_osqp::SolveOutcome{}, PipelineStage::Initial,
    "unit-not-overtake", "must not write", output_root("not-overtake"));
  EXPECT_EQ(result.status, RecordStatus::NotOvertake);
}

}  // namespace
}  // namespace multi_purpose_mpc_ros::mpcc_architecture_snapshot
