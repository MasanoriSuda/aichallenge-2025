#include "multi_purpose_mpc_ros/mpcc_architecture_snapshot.hpp"

#include "multi_purpose_mpc_ros/mpcc_rate_resolved_shadow.hpp"

#include <gtest/gtest.h>

#include <Eigen/Sparse>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>
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
  snapshot.identity.source_context.dynamic_obstacle_constraint_active = true;
  snapshot.identity.source_context.dynamic_obstacle_generation = 5U;
  snapshot.identity.source_context.dynamic_obstacle_id = "d2";
  snapshot.identity.source_context.dynamic_obstacle_side_sign = 1;
  snapshot.identity.source_context.horizon_steps = 1U;
  snapshot.identity.source_context.formulation =
    mpcc_execution_contract::Formulation::
    VelocitySteeringYawResponseProgress7State;
  snapshot.identity.source_context.state_schema_id =
    multi_purpose_mpc_ros::mpcc_rate_resolved::kCoordinateStateSchema;
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

TEST(MpccArchitectureSnapshot, PreservesPublishedArtifactAndIndependentClocks)
{
  namespace execution = mpcc_rate_resolved_execution_artifact;
  auto source = make_interaction_snapshot(
    mpcc_execution_contract::ControlIntent::Cruise);
  // The generic snapshot fixture carries an Overtake execution side. Cruise
  // has neutral intent geometry even when a dynamic-obstacle homotopy exists.
  source.identity.source_context.execution_side_sign = 0;
  source.identity.source_context = mpcc_execution_contract::seal_problem_context(
    source.identity.source_context);
  source.request.current_steering_rad = 0.0;
  source.request.current_response_steering_rad = 0.0;
  source.request.initial_state[3] = 2.0;
  execution::ExecutionArtifact artifact;
  artifact.identity = source.identity;
  artifact.prediction_origin_sec = source.control_prediction_origin_sec;
  artifact.publication_interval_sec = 0.025;
  artifact.completed_sec = source.identity.snapshot_sec + 0.01;
  artifact.course_progress_origin_m = source.course_progress_origin_m;
  artifact.course_frame = {
    std::make_shared<const std::vector<mpc_stage_geometry::CourseFrameKnot>>(
      source.wall_course_frame_knots), source.course_progress_origin_m};
  artifact.wheelbase_m = 1.0;
  artifact.maximum_abs_steering_rad = 0.5;
  artifact.maximum_abs_steering_rate_radps = 1.0;
  artifact.physical_global_tolerance = 1e-6;
  artifact.maximum_constraint_violation = 1e-8;
  artifact.maximum_normalized_constraint_violation = 0.1;
  artifact.predicted_states = {
    {0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0, 2.0, 0.2, 0.0, 0.0}};
  artifact.semantic_initial_state = artifact.predicted_states.front();
  artifact.control_stages = {{0.0, 0.0, 2.0, 0.1, 0.0, 4.0, -3.0, 1.37, 0.0}};
  artifact.nominal_path_distance_m = {0.0, 0.2};
  artifact.lateral_lower_m = {-1.0, -1.0};
  artifact.lateral_upper_m = {1.0, 1.0};
  ASSERT_EQ(execution::validate(artifact), execution::RejectReason::None);
  PublicationEvidence publication;
  publication.failure_decision_id = 55U;
  publication.failure_interaction_fingerprint = 12345U;
  publication.failure_observation_sec = 12.61;
  publication.failure_control_origin_sec = 12.74;
  publication.source_kind = "exact-executed";
  publication.publication_decision_id = 54U;
  publication.publication_control_origin_sec = 12.65;
  publication.publication_artifact_elapsed_sec = 0.03712345678901234;
  const auto root = output_root("published-execution");
  std::filesystem::remove_all(root);

  // An unrelated fresh problem cannot be relabelled as the executed source.
  auto wrong_source = source;
  ++wrong_source.identity.sequence;
  EXPECT_EQ(
    record_published_execution(
      wrong_source, artifact, publication, "delay-prefix-blocked", root).status,
    RecordStatus::InvalidInput);
  ASSERT_FALSE(std::filesystem::exists(root));

  auto wrong_frame = artifact;
  auto knots = source.wall_course_frame_knots;
  knots.back().y_m += 0.1;
  wrong_frame.course_frame.knots =
    std::make_shared<const std::vector<mpc_stage_geometry::CourseFrameKnot>>(knots);
  const auto wrong_root = output_root("published-wrong-coordinate-frame");
  std::filesystem::remove_all(wrong_root);
  EXPECT_EQ(record_published_execution(
      source, wrong_frame, publication, "wrong-coordinate-frame", wrong_root).status,
    RecordStatus::InvalidInput);
  std::filesystem::remove_all(wrong_root);
  wrong_frame.course_frame.knots.reset();
  EXPECT_EQ(record_published_execution(
      source, wrong_frame, publication, "missing-coordinate-frame", wrong_root).status,
    RecordStatus::InvalidInput);
  std::filesystem::remove_all(wrong_root);

  const auto written = record_published_execution(
    source, artifact, publication, "delay-prefix-blocked", root);
  ASSERT_EQ(written.status, RecordStatus::Written) << written.detail;
  std::string detail;
  const auto loaded = load_recorded_interaction_snapshot(written.snapshot_file, &detail);
  ASSERT_TRUE(loaded.has_value()) << detail;
  EXPECT_EQ(loaded->source.identity.sequence, artifact.identity.sequence);
  EXPECT_FALSE(loaded->recorded_qp.has_value());
  const auto yaml = YAML::LoadFile(written.snapshot_file.string());
  const auto evidence = yaml["execution_evidence"];
  EXPECT_DOUBLE_EQ(evidence["semantic_initial_state"]["progress_m"].as<double>(), 0.0);
  EXPECT_DOUBLE_EQ(evidence["semantic_initial_state"]["velocity_mps"].as<double>(), 2.0);
  ASSERT_EQ(evidence["schema"].as<std::string>(), "mpcc-published-execution-evidence/v1");
  EXPECT_EQ(evidence["source_problem_fingerprint"].as<std::uint64_t>(),
    artifact.identity.source_context.fingerprint);
  EXPECT_EQ(evidence["coordinate_state_schema"].as<std::string>(),
    mpcc_rate_resolved::kCoordinateStateSchema);
  EXPECT_TRUE(evidence["source_course_frame_bound"].as<bool>());
  ASSERT_TRUE(loaded->source.request.course_frame.knots);
  EXPECT_DOUBLE_EQ(loaded->source.request.course_frame.progress_origin_m,
    artifact.course_frame.progress_origin_m);
  const auto clock = evidence["publication"];
  EXPECT_EQ(clock["failure_interaction_fingerprint"].as<std::uint64_t>(), 12345U);
  EXPECT_DOUBLE_EQ(clock["failure_control_origin_sec"].as<double>(), 12.74);
  EXPECT_DOUBLE_EQ(clock["publication_control_origin_sec"].as<double>(), 12.65);
  EXPECT_DOUBLE_EQ(clock["publication_artifact_elapsed_sec"].as<double>(),
    publication.publication_artifact_elapsed_sec);
  EXPECT_DOUBLE_EQ(evidence["prediction_origin_sec"].as<double>(), 12.6);
  // These are the accepted primal states and controls, not request references.
  ASSERT_EQ(evidence["predicted_states"].size(), 2U);
  EXPECT_DOUBLE_EQ(evidence["predicted_states"][1]["progress_m"].as<double>(), 0.2);
  EXPECT_DOUBLE_EQ(evidence["predicted_states"][1]["velocity_mps"].as<double>(), 2.0);
  ASSERT_EQ(evidence["control_stages"].size(), 1U);
  EXPECT_DOUBLE_EQ(evidence["control_stages"][0]["duration_sec"].as<double>(), 0.1);
  EXPECT_DOUBLE_EQ(evidence["control_stages"][0]["virtual_progress_speed_mps"].as<double>(), 2.0);
  EXPECT_DOUBLE_EQ(evidence["lateral_lower_m"][1].as<double>(), -1.0);
  EXPECT_EQ(record_published_execution(
      source, artifact, publication, "delay-prefix-blocked", root).status,
    RecordStatus::Duplicate);
  std::filesystem::remove_all(root);
}

PublishedExecutionObservation make_publication_observation()
{
  namespace execution = mpcc_rate_resolved_execution_artifact;
  auto source = make_interaction_snapshot(
    mpcc_execution_contract::ControlIntent::Cruise);
  // The generic snapshot fixture carries an Overtake execution side. Cruise
  // has neutral intent geometry even when a dynamic-obstacle homotopy exists.
  source.identity.source_context.execution_side_sign = 0;
  source.identity.source_context = mpcc_execution_contract::seal_problem_context(
    source.identity.source_context);
  source.request.current_steering_rad = 0.0;
  source.request.current_response_steering_rad = 0.0;
  source.request.initial_state[3] = 2.0;
  execution::ExecutionArtifact artifact;
  artifact.identity = source.identity;
  artifact.prediction_origin_sec = source.control_prediction_origin_sec;
  artifact.publication_interval_sec = 0.025;
  artifact.completed_sec = source.identity.snapshot_sec + 0.01;
  artifact.course_progress_origin_m = source.course_progress_origin_m;
  artifact.course_frame = {
    std::make_shared<const std::vector<mpc_stage_geometry::CourseFrameKnot>>(
      source.wall_course_frame_knots), source.course_progress_origin_m};
  artifact.wheelbase_m = 1.0;
  artifact.maximum_abs_steering_rad = 0.5;
  artifact.maximum_abs_steering_rate_radps = 1.0;
  artifact.physical_global_tolerance = 1e-6;
  artifact.maximum_constraint_violation = 1e-8;
  artifact.maximum_normalized_constraint_violation = 0.1;
  artifact.predicted_states = {
    {0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0, 2.0, 0.2, 0.0, 0.0}};
  artifact.semantic_initial_state = artifact.predicted_states.front();
  artifact.control_stages = {{0.0, 0.0, 2.0, 0.1, 0.0, 4.0, -3.0, 1.37, 0.0}};
  artifact.nominal_path_distance_m = {0.0, 0.2};
  artifact.lateral_lower_m = {-1.0, -1.0};
  artifact.lateral_upper_m = {1.0, 1.0};
  PublicationEvidence publication;
  publication.failure_decision_id = 55U;
  publication.failure_interaction_fingerprint = 12345U;
  publication.failure_observation_sec = 12.61;
  publication.failure_control_origin_sec = 12.74;
  publication.source_kind = "exact-executed";
  publication.publication_decision_id = 54U;
  publication.publication_control_origin_sec = 12.65;
  publication.publication_artifact_elapsed_sec = 0.03712345678901234;
  return {std::make_shared<const mpcc_rate_resolved_shadow::Snapshot>(source),
    std::make_shared<const execution::ExecutionArtifact>(artifact), publication};
}

void bind_failure_world(
  PublishedExecutionObservation & observation,
  const mpcc_rate_resolved_shadow::Snapshot & world)
{
  observation.publication.failure_decision_id = world.identity.source_context.decision_id;
  observation.publication.failure_interaction_fingerprint = fingerprint_interaction_snapshot(world);
  observation.publication.failure_observation_sec = world.identity.snapshot_sec;
  observation.publication.failure_control_origin_sec = world.control_prediction_origin_sec;
}

TEST(MpccArchitectureSnapshot, EachWrittenFailureOwnsItsActualPublishedArtifact)
{
  namespace execution = mpcc_rate_resolved_execution_artifact;
  auto observed = make_publication_observation();
  auto world = *make_publication_observation().source;
  const auto root = output_root("atomic-distinct-publications");
  std::filesystem::remove_all(root);
  for (int index = 0; index < 2; ++index) {
    auto source = *observed.source;
    auto artifact = *observed.artifact;
    source.identity.sequence = index == 0 ? 296U : 3834U;
    artifact.identity = source.identity;
    observed.source = std::make_shared<const mpcc_rate_resolved_shadow::Snapshot>(source);
    observed.artifact = std::make_shared<const execution::ExecutionArtifact>(artifact);
    world.identity.sequence = index == 0 ? 851U : 4428U;
    world.identity.source_context.decision_id = world.identity.sequence;
    world.identity.source_context = mpcc_execution_contract::seal_problem_context(
      world.identity.source_context);
    bind_failure_world(observed, world);
    const std::string outcome = index == 0 ? "atomic-startup-loss" : "atomic-terminal-loss";
    const auto recorded = record_authority_failure(world, outcome, "test boundary", observed, root);
    ASSERT_EQ(recorded.status, RecordStatus::Written) << recorded.detail;
    std::string detail;
    const auto current = load_recorded_interaction_snapshot(recorded.snapshot_file, &detail);
    ASSERT_TRUE(current) << detail;
    EXPECT_EQ(current->source.identity.sequence, world.identity.sequence);
    auto node = YAML::LoadFile(recorded.snapshot_file.string());
    const auto bundle = node["publication_bundle"];
    ASSERT_EQ(bundle["status"].as<std::string>(), "present");
    EXPECT_EQ(bundle["schema"].as<std::string>(), "mpcc-authority-loss-publication/v1");
    EXPECT_EQ(bundle["source"]["sequence"].as<std::uint64_t>(), source.identity.sequence);
    const auto evidence = bundle["execution_evidence"];
    EXPECT_EQ(evidence["source_sequence"].as<std::uint64_t>(), artifact.identity.sequence);
    EXPECT_EQ(evidence["publication"]["failure_decision_id"].as<std::uint64_t>(),
      world.identity.sequence);
    EXPECT_EQ(evidence["publication"]["failure_interaction_fingerprint"].as<std::uint64_t>(),
      fingerprint_interaction_snapshot(world));
    EXPECT_DOUBLE_EQ(evidence["control_stages"][0]["duration_sec"].as<double>(),
      artifact.control_stages[0].duration_sec);
    EXPECT_TRUE(std::filesystem::exists(recorded.snapshot_file.parent_path()/"published-wall-grid.bin"));
    EXPECT_FALSE(std::filesystem::exists(recorded.snapshot_file.parent_path().string()+".tmp"));
    // The nested original input round-trips through the unchanged interaction
    // parser, including its own wall payload and exact immutable fingerprint.
    node["source"] = YAML::Clone(bundle["source"]);
    node["interaction_fingerprint"] = bundle["interaction_fingerprint"].as<std::uint64_t>();
    node.remove("publication_bundle");
    const auto extracted = recorded.snapshot_file.parent_path()/"published-source-test.yaml";
    std::ofstream stream(extracted);
    stream << node; stream.close();
    const auto loaded_source = load_recorded_interaction_snapshot(extracted, &detail);
    ASSERT_TRUE(loaded_source) << detail;
    EXPECT_EQ(loaded_source->source.identity.sequence, source.identity.sequence);
    EXPECT_EQ(loaded_source->interaction_fingerprint, fingerprint_interaction_snapshot(source));
    EXPECT_EQ(record_authority_failure(world, outcome, "duplicate boundary", observed, root).status,
      RecordStatus::Duplicate);
  }
  std::filesystem::remove_all(root);
}

TEST(MpccArchitectureSnapshot, InvalidPublicationNeverContaminatesFailureWorld)
{
  namespace execution = mpcc_rate_resolved_execution_artifact;
  const auto world = *make_publication_observation().source;
  const auto root = output_root("atomic-invalid-publications");
  std::filesystem::remove_all(root);
  for (int variant = 0; variant < 6; ++variant) {
    auto observed = make_publication_observation();
    bind_failure_world(observed, world);
    if (variant == 0) {
      auto source = *observed.source; ++source.identity.sequence;
      observed.source = std::make_shared<const mpcc_rate_resolved_shadow::Snapshot>(source);
    } else if (variant == 1) {
      ++observed.publication.failure_interaction_fingerprint;
    } else if (variant == 2) {
      ++observed.publication.failure_decision_id;
    } else if (variant == 3) {
      observed.publication.failure_observation_sec += 0.01;
    } else if (variant == 4) {
      observed.publication.failure_control_origin_sec += 0.01;
    } else {
      auto artifact = *observed.artifact;
      artifact.semantic_initial_state->velocity_mps += 1.0;
      observed.artifact = std::make_shared<const execution::ExecutionArtifact>(artifact);
    }
    const auto recorded = record_authority_failure(world,
      "atomic-invalid-"+std::to_string(variant), "test boundary", observed, root);
    ASSERT_EQ(recorded.status, RecordStatus::Written) << recorded.detail;
    const auto node = YAML::LoadFile(recorded.snapshot_file.string());
    EXPECT_EQ(node["publication_bundle"]["status"].as<std::string>(), "invalid");
    EXPECT_FALSE(node["publication_bundle"]["source"]);
    EXPECT_FALSE(std::filesystem::exists(recorded.snapshot_file.parent_path()/"published-wall-grid.bin"));
    EXPECT_TRUE(load_recorded_interaction_snapshot(recorded.snapshot_file));
  }
  std::filesystem::remove_all(root);
}

TEST(MpccArchitectureSnapshot, MissingPublicationIsExplicitAndCurrentWorldStaysReplayable)
{
  const auto world = *make_publication_observation().source;
  const auto root = output_root("atomic-missing-publication");
  std::filesystem::remove_all(root);
  const auto recorded = record_authority_failure(world,
    "atomic-missing", "bootstrap has no published plan", {}, root);
  ASSERT_EQ(recorded.status, RecordStatus::Written);
  const auto node = YAML::LoadFile(recorded.snapshot_file.string());
  EXPECT_EQ(node["publication_bundle"]["status"].as<std::string>(), "missing");
  EXPECT_FALSE(node["publication_bundle"]["execution_evidence"]);
  EXPECT_TRUE(load_recorded_interaction_snapshot(recorded.snapshot_file));
  auto invalid_world = world;
  invalid_world.request.wheelbase_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(record_authority_failure(invalid_world, "atomic-invalid-world", "invalid",
      {}, root).status, RecordStatus::InvalidInput);
  std::filesystem::remove_all(root);
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
  production_outcome.rejected_primal = Eigen::VectorXd::Constant(1, 0.25);

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
  ASSERT_TRUE(loaded->rejected_primal.has_value());
  EXPECT_DOUBLE_EQ((*loaded->rejected_primal)[0], 0.25);
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
  auto assembly = make_assembly_request();
  mpcc_rate_resolved_problem::DynamicObstacleConstraint plane;
  plane.state_stage = 1;
  plane.upper = 0.4;
  Eigen::Matrix<double, mpcc_rate_resolved::kStateDimension, 1> coefficients;
  coefficients << 0.2, -0.1, 0.3, 0.0, 0.7, 0.0, 0.0;
  plane.physical_state_coefficients = coefficients;
  assembly.dynamic_obstacle_constraints.push_back(plane);
  const auto written = record_failure(
    snapshot, assembly, make_valid_problem(), std::nullopt,
    persistent_osqp::SolveOutcome{}, PipelineStage::Initial,
    "unit-interaction-roundtrip", "intentional replay-ready evidence", root);
  ASSERT_EQ(written.status, RecordStatus::Written) << written.detail;

  std::ostringstream expected_identity;
  expected_identity << std::hex << std::setw(16) << std::setfill('0') <<
    fingerprint_interaction_snapshot(snapshot);
  EXPECT_NE(
    written.snapshot_file.parent_path().filename().string().find(
      expected_identity.str()),
    std::string::npos);

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
  EXPECT_TRUE(
    loaded->source.identity.source_context.dynamic_obstacle_constraint_active);
  EXPECT_EQ(
    loaded->source.identity.source_context.dynamic_obstacle_id, "d2");
  EXPECT_EQ(
    loaded->source.identity.source_context.dynamic_obstacle_generation, 5U);
  EXPECT_EQ(loaded->source.replay_world->observation_generation, 5U);
  EXPECT_EQ(
    loaded->source.replay_world->control_prefix_elapsed_sec,
    (std::vector<double>{0.0, 0.1}));
  EXPECT_DOUBLE_EQ(
    loaded->source.replay_world->physical_footprint.left_extent_m, 0.525);
  ASSERT_EQ(loaded->source.replay_world->obstacles.size(), 1U);
  EXPECT_EQ(loaded->source.replay_world->obstacles.front().id, "d2");
  EXPECT_EQ(loaded->source.wall_grid->cells.size(), 4U);
  ASSERT_TRUE(loaded->assembly_request.has_value());
  ASSERT_FALSE(loaded->assembly_request->dynamic_obstacle_constraints.empty());
  const auto & loaded_plane = loaded->assembly_request->dynamic_obstacle_constraints.back();
  ASSERT_TRUE(loaded_plane.physical_state_coefficients.has_value());
  EXPECT_TRUE(loaded_plane.physical_state_coefficients->isApprox(coefficients, 0.0));
  EXPECT_EQ(loaded->assembly_request->horizon_steps, 1);
  EXPECT_TRUE(
    loaded->assembly_request->state_reference.isApprox(
      make_assembly_request().state_reference));
  EXPECT_TRUE(
    loaded->assembly_request->input_reference.isApprox(
      make_assembly_request().input_reference));

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

TEST(MpccArchitectureSnapshot, RecordsSourceOnlyTerminalProofBoundary)
{
  const auto root = output_root("terminal-proof-boundary");
  std::filesystem::remove_all(root);
  auto snapshot = make_interaction_snapshot(
    mpcc_execution_contract::ControlIntent::Pass);
  ASSERT_TRUE(snapshot.replay_world.has_value());
  auto & world = snapshot.replay_world.value();
  world.terminal_stop_contract_available = true;
  world.terminal_stop_lateral_policy.wheelbase_m = 1.0;
  world.terminal_stop_lateral_policy.maximum_abs_steering_rad = 0.5;
  world.terminal_stop_lateral_policy.maximum_abs_steering_rate_radps = 1.0;
  world.terminal_stop_lateral_policy.maximum_lateral_acceleration_mps2 = 4.0;
  world.terminal_stop_lateral_policy.steering_command_gain = 1.0;
  world.terminal_stop_lateral_policy.lateral_gain = 0.8;
  world.terminal_stop_lateral_policy.heading_gain = 1.2;
  world.terminal_stop_minimum_acceleration_mps2 = -2.0;

  const auto written = record_proof_failure(
    snapshot, PipelineStage::PhysicalProof,
    "terminal-contingency-unavailable",
    "unit terminal Stop wall rejection", root);
  ASSERT_EQ(written.status, RecordStatus::Written) << written.detail;

  std::string detail;
  const auto loaded = load_recorded_interaction_snapshot(
    written.snapshot_file, &detail);
  ASSERT_TRUE(loaded.has_value()) << detail;
  EXPECT_TRUE(interaction_snapshot_complete(loaded->source));
  EXPECT_FALSE(loaded->assembly_request.has_value());
  EXPECT_FALSE(loaded->recorded_qp.has_value());
  ASSERT_TRUE(loaded->source.replay_world.has_value());
  EXPECT_TRUE(
    loaded->source.replay_world->terminal_stop_contract_available);
  EXPECT_DOUBLE_EQ(
    loaded->source.replay_world->terminal_stop_minimum_acceleration_mps2,
    -2.0);
  EXPECT_TRUE(
    interaction_snapshot_matches_fingerprint(
      loaded->source, loaded->interaction_fingerprint));

  EXPECT_FALSE(load_recorded_qp(written.snapshot_file, &detail).has_value());
  EXPECT_EQ(
    detail, "source-only proof-boundary snapshot has no exact QP");
}

TEST(MpccArchitectureSnapshot, RejectsMalformedRecordedAssemblyRequest)
{
  const auto root = output_root("interaction-malformed-assembly");
  std::filesystem::remove_all(root);
  const auto snapshot = make_interaction_snapshot(
    mpcc_execution_contract::ControlIntent::ShiftOut);
  const auto written = record_failure(
    snapshot, make_assembly_request(), make_valid_problem(), std::nullopt,
    persistent_osqp::SolveOutcome{}, PipelineStage::Initial,
    "unit-malformed-assembly", "intentional malformed assembly evidence", root);
  ASSERT_EQ(written.status, RecordStatus::Written) << written.detail;

  auto yaml = YAML::LoadFile(written.snapshot_file.string());
  yaml["assembly_request"]["horizon_steps"] = 0;
  std::ofstream output(written.snapshot_file, std::ios::trunc);
  ASSERT_TRUE(output.good());
  output << yaml;
  output.close();

  std::string detail;
  EXPECT_FALSE(
    load_recorded_interaction_snapshot(written.snapshot_file, &detail).has_value());
  EXPECT_EQ(detail, "assembly request unavailable");
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

TEST(MpccArchitectureSnapshot, V1KeepsExactQpButRejectsInteractionMigration)
{
  const auto root = output_root("v1-exact-qp-only");
  std::filesystem::remove_all(root);
  const auto written = record_failure(
    make_snapshot(mpcc_execution_contract::ControlIntent::ShiftOut),
    make_assembly_request(), make_valid_problem(), std::nullopt,
    persistent_osqp::SolveOutcome{}, PipelineStage::Initial,
    "unit-v1-boundary", "legacy schema boundary", root);
  ASSERT_EQ(written.status, RecordStatus::Written) << written.detail;

  std::ifstream input(written.snapshot_file);
  ASSERT_TRUE(input.good());
  std::ostringstream contents;
  contents << input.rdbuf();
  std::string legacy = contents.str();
  const std::string v2 = "mpcc-architecture-failure-snapshot/v2";
  const auto schema_position = legacy.find(v2);
  ASSERT_NE(schema_position, std::string::npos);
  legacy.replace(
    schema_position, v2.size(), "mpcc-architecture-failure-snapshot/v1");
  const auto legacy_file = written.snapshot_file.parent_path() / "snapshot-v1.yaml";
  std::ofstream output(legacy_file);
  ASSERT_TRUE(output.good());
  output << legacy;
  output.close();

  std::string detail;
  EXPECT_TRUE(load_recorded_qp(legacy_file, &detail).has_value()) << detail;
  EXPECT_FALSE(
    load_recorded_interaction_snapshot(legacy_file, &detail).has_value());
  EXPECT_EQ(
    detail,
    "v1 snapshot has no immutable dynamic-obstacle constraint identity; "
    "exact QP replay remains available");
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

TEST(MpccArchitectureSnapshot, KeepsOppositeFollowHomotopyFailuresDistinct)
{
  const auto root = output_root("follow-homotopy-dedup");
  std::filesystem::remove_all(root);
  auto positive = make_snapshot(
    mpcc_execution_contract::ControlIntent::Follow);
  positive.identity.source_context.execution_side_sign = 0;
  positive.dynamic_obstacle_pass_side_sign = 1;
  auto negative = positive;
  negative.dynamic_obstacle_pass_side_sign = -1;
  const auto problem = make_valid_problem();
  const auto request = make_assembly_request();
  const persistent_osqp::SolveOutcome outcome;

  const auto positive_result = record_failure(
    positive, request, problem, std::nullopt, outcome,
    PipelineStage::PhysicalProof, "unit-follow-homotopy", "positive", root);
  const auto negative_result = record_failure(
    negative, request, problem, std::nullopt, outcome,
    PipelineStage::PhysicalProof, "unit-follow-homotopy", "negative", root);
  const auto negative_duplicate = record_failure(
    negative, request, problem, std::nullopt, outcome,
    PipelineStage::PhysicalProof, "unit-follow-homotopy", "negative-again", root);

  EXPECT_EQ(positive_result.status, RecordStatus::Written)
    << positive_result.detail;
  EXPECT_EQ(negative_result.status, RecordStatus::Written)
    << negative_result.detail;
  EXPECT_EQ(negative_duplicate.status, RecordStatus::Duplicate)
    << negative_duplicate.detail;
  EXPECT_NE(positive_result.snapshot_file, negative_result.snapshot_file);
}

TEST(MpccArchitectureSnapshot, WritesExactQpForFollowFailure)
{
  const auto root = output_root("follow-exact-qp");
  std::filesystem::remove_all(root);
  const auto snapshot = make_snapshot(
    mpcc_execution_contract::ControlIntent::Follow);
  const auto written = record_failure(
    snapshot, make_assembly_request(), make_valid_problem(), std::nullopt,
    persistent_osqp::SolveOutcome{}, PipelineStage::Initial,
    "unit-follow", "canonical Follow failure", root);
  ASSERT_EQ(written.status, RecordStatus::Written) << written.detail;
  std::string detail;
  const auto loaded = load_recorded_qp(written.snapshot_file, &detail);
  ASSERT_TRUE(loaded.has_value()) << detail;
  EXPECT_EQ(loaded->intent, "follow");
  EXPECT_FALSE(
    load_recorded_interaction_snapshot(written.snapshot_file).has_value());
}

TEST(MpccArchitectureSnapshot, RefusesUnsupportedIntentCapture)
{
  const auto snapshot = make_snapshot(
    mpcc_execution_contract::ControlIntent::Stop);
  const auto result = record_failure(
    snapshot, make_assembly_request(), make_valid_problem(), std::nullopt,
    persistent_osqp::SolveOutcome{}, PipelineStage::Initial,
    "unit-unsupported", "must not write", output_root("unsupported"));
  EXPECT_EQ(result.status, RecordStatus::UnsupportedIntent);
}

}  // namespace
}  // namespace multi_purpose_mpc_ros::mpcc_architecture_snapshot
