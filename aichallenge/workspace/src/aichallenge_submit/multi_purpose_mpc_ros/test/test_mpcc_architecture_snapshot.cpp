#include "mpcc_vehicle_model_fixture.hpp"
#include "multi_purpose_mpc_ros/mpcc_architecture_snapshot.hpp"

#include "multi_purpose_mpc_ros/mpcc_rate_resolved_shadow.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_retained_revalidation.hpp"
#include "multi_purpose_mpc_ros/mpcc_scheduled_failure_observation.hpp"

#include <gtest/gtest.h>

#include <Eigen/Sparse>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
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
    VelocitySteeringTireBodyProgress9State;
  snapshot.identity.source_context.vehicle_model_fingerprint = multi_purpose_mpc_ros::mpcc_vehicle_model::fingerprint(
    multi_purpose_mpc_ros::test::vehicle_model());
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
  snapshot.request.curvature_reference_gain = 1.0;
  snapshot.request.vehicle_model = multi_purpose_mpc_ros::test::vehicle_model();
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

mpcc_vehicle_model::ObservationProvenance received_body_selected()
{
  return {{19.8, {1, 2, .1, 2, .2, .3, .05, .04}}, 19.79, 19.8, 19.78,
    20.0, 20.1, 0, .1, {{19.0, 1.0, .1}, {19.9, -3.0, .2}}};
}

ReceivedBodyObservation received_body_fixture(const std::uint64_t decision)
{
  ReceivedBodyObservation value;
  value.control_decision_id = decision;
  // Above double's exact integer range: receipt timestamps must remain integers.
  value.captured_steady_ns = 9007199254740999LL;
  value.now_sec = 20;
  value.pose_source_sec = 19.8;
  value.selected_component_source_sec = {19.79, 19.8, 19.78};
  value.selected_x_y_yaw_u_vy_r_desired_tire = {1, 2, .1, 2, .2, .3, .05, .04};
  value.histories[0] = {{19.79, value.captured_steady_ns - 3, 2, .2},
    {19.85, value.captured_steady_ns - 5, 2.1, .25},
    {20.1, value.captured_steady_ns - 1, 2.2, .27}};
  value.histories[1] = {{19.8, value.captured_steady_ns - 4, .3, 0},
    {19.9, value.captured_steady_ns - 2, .35, 0}};
  value.histories[2] = {{19.78, value.captured_steady_ns - 6, .04, 0}};
  return value;
}

TEST(MpccArchitectureSnapshot, ReceivedBodiesPreserveUnselectedSamplesWithoutChangingIdentity)
{
  const auto root = output_root("received-body-roundtrip");
  std::filesystem::remove_all(root);
  auto snapshot = make_interaction_snapshot(mpcc_execution_contract::ControlIntent::Pass);
  snapshot.request.observation_provenance = received_body_selected();
  const auto identity = fingerprint_interaction_snapshot(snapshot);
  ASSERT_NE(identity, 0U);
  const auto expected = received_body_fixture(99);
  snapshot.received_body_observation = std::make_shared<const ReceivedBodyObservation>(expected);
  EXPECT_EQ(fingerprint_interaction_snapshot(snapshot), identity);
  const auto recorded = record_authority_failure(snapshot, "received-body", "test observation", {}, root);
  ASSERT_EQ(recorded.status, RecordStatus::Written) << recorded.detail;
  auto document = YAML::LoadFile(recorded.snapshot_file.string());
  const auto diagnostic = document["source"]["received_body_observation"];
  ASSERT_TRUE(diagnostic);
  EXPECT_EQ(diagnostic["status"].as<std::string>(), "valid");
  EXPECT_FALSE(diagnostic["authority"].as<bool>());
  EXPECT_TRUE(diagnostic["selected_observation_matches"].as<bool>());
  std::string detail;
  auto loaded = load_recorded_interaction_snapshot(recorded.snapshot_file, &detail);
  ASSERT_TRUE(loaded) << detail;
  ASSERT_TRUE(loaded->source.received_body_observation);
  const auto & actual = *loaded->source.received_body_observation;
  EXPECT_EQ(actual.control_decision_id, 99U);
  EXPECT_NE(actual.control_decision_id, snapshot.identity.sequence);
  EXPECT_EQ(actual.captured_steady_ns, expected.captured_steady_ns);
  for (std::size_t i = 0; i < expected.histories.size(); ++i) {
    ASSERT_EQ(actual.histories[i].size(), expected.histories[i].size());
    for (std::size_t j = 0; j < expected.histories[i].size(); ++j) {
      EXPECT_DOUBLE_EQ(actual.histories[i][j].source_sec, expected.histories[i][j].source_sec);
      EXPECT_EQ(actual.histories[i][j].received_steady_ns, expected.histories[i][j].received_steady_ns);
      EXPECT_DOUBLE_EQ(actual.histories[i][j].first, expected.histories[i][j].first);
      EXPECT_DOUBLE_EQ(actual.histories[i][j].second, expected.histories[i][j].second);
    }
  }
  EXPECT_GT(actual.histories[0][1].source_sec, actual.pose_source_sec);
  EXPECT_GT(actual.histories[0][2].source_sec, actual.now_sec);
  EXPECT_EQ(loaded->interaction_fingerprint, identity);
  // Historical records without this optional observation remain replayable.
  document["source"].remove("received_body_observation");
  const auto legacy = recorded.snapshot_file.parent_path() / "legacy.yaml";
  {std::ofstream stream(legacy); stream << document;}
  loaded = load_recorded_interaction_snapshot(legacy, &detail);
  ASSERT_TRUE(loaded) << detail;
  EXPECT_FALSE(loaded->source.received_body_observation);
  EXPECT_EQ(loaded->interaction_fingerprint, identity);
  std::filesystem::remove_all(root);
}

TEST(MpccArchitectureSnapshot, InvalidReceivedBodiesDoNotDiscardValidReplayEvidence)
{
  const auto root = output_root("received-body-invalid");
  std::filesystem::remove_all(root);
  auto snapshot = make_interaction_snapshot(mpcc_execution_contract::ControlIntent::Pass);
  snapshot.request.observation_provenance = received_body_selected();
  const auto identity = fingerprint_interaction_snapshot(snapshot);
  for (int defect = 0; defect < 4; ++defect) {
    auto value = received_body_fixture(99);
    if (defect == 0) value.histories[0].erase(value.histories[0].begin());
    if (defect == 1) value.histories[0].back().received_steady_ns = value.captured_steady_ns + 1;
    if (defect == 2) value.histories[1][0].first = std::numeric_limits<double>::quiet_NaN();
    if (defect == 3) value.histories[2].resize(257, value.histories[2].front());
    snapshot.received_body_observation = std::make_shared<const ReceivedBodyObservation>(value);
    const auto recorded = record_authority_failure(snapshot, "invalid-" + std::to_string(defect),
      "test invalid optional diagnostic", {}, root);
    ASSERT_EQ(recorded.status, RecordStatus::Written) << recorded.detail;
    auto document = YAML::LoadFile(recorded.snapshot_file.string());
    ASSERT_TRUE(document["source"]["received_body_observation"]);
    EXPECT_EQ(document["source"]["received_body_observation"]["status"].as<std::string>(), "invalid");
    std::string detail;
    const auto loaded = load_recorded_interaction_snapshot(recorded.snapshot_file, &detail);
    ASSERT_TRUE(loaded) << detail;
    EXPECT_FALSE(loaded->source.received_body_observation);
    EXPECT_EQ(loaded->interaction_fingerprint, identity);
    // Also exercise malformed optional input from disk, independently of writer validation.
    document["source"]["received_body_observation"] = "malformed-diagnostic";
    const auto malformed = recorded.snapshot_file.parent_path() / "malformed.yaml";
    {std::ofstream stream(malformed); stream << document;}
    const auto reloaded = load_recorded_interaction_snapshot(malformed, &detail);
    ASSERT_TRUE(reloaded) << detail;
    EXPECT_FALSE(reloaded->source.received_body_observation);
    EXPECT_EQ(reloaded->interaction_fingerprint, identity);
  }
  std::filesystem::remove_all(root);
}

TEST(MpccArchitectureSnapshot, PostMotionStationaryPairSurvivesStartupAndMovingSlots)
{
  const auto root=output_root("post-motion-final-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  std::vector<RecordResult> records;
  const auto parent=std::this_thread::get_id();
  bool worker_only=true;
  FirstPublicationFailureRecorder recorder([&](const auto &,const auto &result) {
    worker_only &= std::this_thread::get_id()!=parent;
    records.push_back(result);
  });
  std::array<PublicationFailureObservation,2> pair;
  for (std::size_t i=0;i<2;++i) {
    auto capture=std::make_shared<ScheduledFailureCapture>();
    capture->boundary="final-current-evidence";
    capture->detail=i==0 ? "programme-unavailable" : "active-rejected";
    pair[i].decision_id=10;
    pair[i].decision_clock_sec=1;
    pair[i].output_root=root/(i==0 ? "due" : "active");
    pair[i].scheduled_capture=capture;
  }
  EXPECT_EQ(recorder.submit_selection_failure(pair),ObservationAdmission::Queued);
  for (auto &o:pair) {o.decision_id=20; o.moving=true; o.decision_clock_sec=2;}
  EXPECT_EQ(recorder.submit_selection_failure(pair),ObservationAdmission::Queued);
  for (auto &o:pair) {o.decision_id=40; o.moving=false; o.decision_clock_sec=4;}
  EXPECT_EQ(recorder.submit_selection_failure(pair),ObservationAdmission::Duplicate);
  const mpcc_vehicle_model::PublishedCommand command{3,-3,.01};
  const mpcc_vehicle_model::PublishedProgramSource source{29,31,37,41,11};
  const PublishedNormalMotionObservation motion{30,2.99,-.2,{17,command,command,3,3.01,source}};
  for (auto &o:pair) {o.prior_normal_motion=motion; o.output_root/="after-normal-motion";}
  EXPECT_EQ(recorder.submit_selection_failure(pair),ObservationAdmission::Queued);
  for (auto &o:pair) o.decision_id=50;
  EXPECT_EQ(recorder.submit_selection_failure(pair),ObservationAdmission::Duplicate);
  recorder.stop();
  ASSERT_EQ(records.size(),6U);
  EXPECT_TRUE(worker_only);
  for (const auto &result:records) {
    ASSERT_EQ(result.status,RecordStatus::Written) << result.detail;
    const auto doc=YAML::LoadFile(result.snapshot_file.string());
    EXPECT_FALSE(doc["authority"].as<bool>());
    const auto boundary=doc["boundary"];
    const auto witness=boundary["prior_normal_motion"];
    if (boundary["decision_id"].as<std::uint64_t>()==40) {
      ASSERT_TRUE(witness);
      EXPECT_FALSE(witness["authority"].as<bool>());
      EXPECT_EQ(witness["decision_id"].as<std::uint64_t>(),30U);
      EXPECT_DOUBLE_EQ(witness["pose_sec"].as<double>(),2.99);
      EXPECT_DOUBLE_EQ(witness["forward_velocity_mps"].as<double>(),-.2);
      const auto actual=witness["actual_publication"];
      EXPECT_EQ(actual["sequence"].as<std::uint64_t>(),17U);
      EXPECT_EQ(actual["source_job_solution_problem_input_index"].as<std::vector<std::uint64_t>>(),
        (std::vector<std::uint64_t>{29,31,37,41,11}));
      EXPECT_DOUBLE_EQ(actual["nominal_wire_acceleration_wire_steering_before_after"][4].as<double>(),3.01);
    } else EXPECT_FALSE(witness);
  }
  std::filesystem::remove_all(root);
}

TEST(MpccArchitectureSnapshot, PostMotionPairRejectsInvalidWitnessWithoutConsumingSlot)
{
  const auto root=output_root("post-motion-invalid-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  FirstPublicationFailureRecorder recorder;
  std::array<PublicationFailureObservation,2> pair;
  const mpcc_vehicle_model::PublishedCommand command{3,-3,.01};
  const mpcc_vehicle_model::PublishedProgramSource source{29,31,37,41,11};
  for (std::size_t i=0;i<2;++i) {
    auto capture=std::make_shared<ScheduledFailureCapture>();
    capture->boundary="final-current-evidence";
    pair[i].decision_id=40; pair[i].decision_clock_sec=4;
    pair[i].output_root=root/(i==0 ? "due" : "active");
    pair[i].scheduled_capture=capture;
    pair[i].prior_normal_motion=PublishedNormalMotionObservation{30,2.99,.2,{17,command,command,3,3.01,source}};
  }
  for (int fault=0;fault<11;++fault) {
    auto invalid=pair;
    auto &o=invalid[1];
    auto &motion=*o.prior_normal_motion;
    switch (fault) {
      case 0: o.prior_normal_motion.reset(); break;
      case 1: motion.decision_id=40; break;
      case 2: motion.publication.after_clock_sec=5; break;
      case 3: motion.forward_velocity_mps=.1; break;
      case 4: motion.forward_velocity_mps=std::numeric_limits<double>::quiet_NaN(); break;
      case 5: motion.publication.source.reset(); break;
      case 6: motion.publication.source->solution_id++; break;
      case 7: motion.publication.sequence++; break;
      case 8: motion.pose_sec=3.02; break;
      case 9: motion.publication.published.wire_steering_rad=0; break;
      case 10: o.moving=true; break;
    }
    EXPECT_EQ(recorder.submit_selection_failure(invalid),ObservationAdmission::Invalid) << fault;
  }
  EXPECT_EQ(recorder.submit(pair[0]),ObservationAdmission::Invalid);
  EXPECT_EQ(recorder.submit_selection_failure(pair),ObservationAdmission::Queued);
  recorder.stop();
  EXPECT_EQ(recorder.submit_selection_failure(pair),ObservationAdmission::Stopped);
  std::filesystem::remove_all(root);
}

PostMotionFinalLossObservation source_loss_fixture()
{
  const mpcc_vehicle_model::PublishedCommand nominal{19.5,-3,.01};
  auto published=nominal; published.published_sec=19.51;
  return {100,19.6,{90,19.4,-.2,{91,nominal,published,19.5,19.51,
    mpcc_vehicle_model::PublishedProgramSource{29,31,37,41,11}}}};
}

TEST(MpccArchitectureSnapshot, SourceAfterFinalLossPreservesBothInitializersAndExactQp)
{
  const auto root=output_root("post-loss-source-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  auto source=make_interaction_snapshot(mpcc_execution_contract::ControlIntent::Cruise);
  source.identity.source_context.execution_side_sign=0;
  source.identity.source_context=mpcc_execution_contract::seal_problem_context(source.identity.source_context);
  source.identity.snapshot_sec=20; source.control_prediction_origin_sec=20.1;
  source.replay_world->observed_sec=20;
  source.request.observation_provenance=received_body_selected();
  auto body=received_body_fixture(103);
  const auto problem=make_valid_problem(); const auto request=make_assembly_request();
  const std::optional<persistent_osqp::WarmStart> warm{persistent_osqp::WarmStart{
    Eigen::VectorXd::Constant(1,.17),Eigen::VectorXd::Constant(1,.23)}};
  persistent_osqp::SolveOutcome failed;
  failed.failure_detail="exact original failure";
  failed.rejected_primal=Eigen::VectorXd::Constant(1,.25);
  for (bool qp:{false,true}) {
    const std::string outcome=qp ? "unit-source-after-loss-qp" : "unit-source-after-loss-physical";
    const auto record=[&](const auto &input) {
      return qp ? record_failure(input,request,problem,warm,failed,PipelineStage::Initial,outcome,"test",root) :
        record_proof_failure(input,PipelineStage::PhysicalProof,outcome,"test",root);
    };
    source.received_body_observation.reset();
    const auto original=record(source);
    ASSERT_EQ(original.status,RecordStatus::Written) << original.detail;
    source.received_body_observation=std::make_shared<const ReceivedBodyObservation>(body);
    EXPECT_EQ(record(source).status,RecordStatus::Duplicate);
    for (int policy:{0,1}) {
      source.request.initial_tangent_policy=static_cast<mpcc_rate_resolved_adapter::InitialTangentPolicy>(policy);
      const auto fingerprint=fingerprint_interaction_snapshot(source);
      ASSERT_NE(fingerprint,0U);
      auto tagged=body; tagged.post_motion_final_loss=std::make_shared<const PostMotionFinalLossObservation>(source_loss_fixture());
      source.received_body_observation=std::make_shared<const ReceivedBodyObservation>(tagged);
      EXPECT_EQ(fingerprint_interaction_snapshot(source),fingerprint);
      const auto written=record(source);
      ASSERT_EQ(written.status,RecordStatus::Written) << written.detail;
      EXPECT_NE(written.snapshot_file.string().find("after-post-motion-final-loss/initializer-"+std::to_string(policy)),std::string::npos);
      std::string detail;
      const auto loaded=load_recorded_interaction_snapshot(written.snapshot_file,&detail);
      ASSERT_TRUE(loaded) << detail;
      EXPECT_EQ(loaded->interaction_fingerprint,fingerprint);
      ASSERT_TRUE(loaded->source.received_body_observation);
      const auto &loss=loaded->source.received_body_observation->post_motion_final_loss;
      ASSERT_TRUE(loss);
      EXPECT_EQ(loss->decision_id,100U);
      EXPECT_EQ(loss->prior_normal_motion.decision_id,90U);
      EXPECT_EQ(loss->prior_normal_motion.publication.source->input_context_fingerprint,41U);
      EXPECT_DOUBLE_EQ(loss->prior_normal_motion.publication.after_clock_sec,19.51);
      if (qp) {
        const auto exact=load_recorded_qp(written.snapshot_file,&detail);
        ASSERT_TRUE(exact) << detail;
        ASSERT_TRUE(exact->warm_start); ASSERT_TRUE(exact->rejected_primal);
        EXPECT_DOUBLE_EQ((*exact->rejected_primal)[0],.25);
        EXPECT_DOUBLE_EQ(exact->warm_start->primal[0],.17);
        EXPECT_DOUBLE_EQ(exact->warm_start->dual[0],.23);
      }
      ++source.identity.sequence;
      EXPECT_EQ(record(source).status,RecordStatus::Duplicate);
    }
  }
  std::filesystem::remove_all(root);
}

TEST(MpccArchitectureSnapshot, SourceAfterFinalLossInvalidMetadataKeepsOriginalReplay)
{
  const auto root=output_root("post-loss-invalid-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  auto source=make_interaction_snapshot(mpcc_execution_contract::ControlIntent::Cruise);
  source.identity.source_context.execution_side_sign=0;
  source.identity.source_context=mpcc_execution_contract::seal_problem_context(source.identity.source_context);
  source.identity.snapshot_sec=20; source.control_prediction_origin_sec=20.1;
  source.replay_world->observed_sec=20; source.request.observation_provenance=received_body_selected();
  auto body=received_body_fixture(103);
  const auto fingerprint=fingerprint_interaction_snapshot(source);
  auto loss=source_loss_fixture(); loss.clock_sec=21;  // future relative to source
  body.post_motion_final_loss=std::make_shared<const PostMotionFinalLossObservation>(loss);
  source.received_body_observation=std::make_shared<const ReceivedBodyObservation>(body);
  const auto record=[&] {return record_proof_failure(source,PipelineStage::PhysicalProof,
    "unit-post-loss-invalid","test",root);};
  const auto original=record();
  ASSERT_EQ(original.status,RecordStatus::Written) << original.detail;
  EXPECT_EQ(original.snapshot_file.string().find("after-post-motion-final-loss"),std::string::npos);
  std::string detail;
  auto loaded=load_recorded_interaction_snapshot(original.snapshot_file,&detail);
  ASSERT_TRUE(loaded) << detail;
  EXPECT_EQ(loaded->interaction_fingerprint,fingerprint);
  ASSERT_TRUE(loaded->source.received_body_observation);
  EXPECT_FALSE(loaded->source.received_body_observation->post_motion_final_loss);
  for (int fault=0;fault<5;++fault) {
    auto invalid=source_loss_fixture();
    switch(fault) {
      case 0: invalid.decision_id=104; break;
      case 1: invalid.prior_normal_motion.decision_id=100; break;
      case 2: invalid.prior_normal_motion.forward_velocity_mps=0; break;
      case 3: invalid.prior_normal_motion.publication.source.reset(); break;
      case 4: invalid.prior_normal_motion.publication.after_clock_sec=20.1; break;
    }
    body.post_motion_final_loss=std::make_shared<const PostMotionFinalLossObservation>(invalid);
    source.received_body_observation=std::make_shared<const ReceivedBodyObservation>(body);
    EXPECT_EQ(record().status,RecordStatus::Duplicate) << fault;
  }
  body.post_motion_final_loss=std::make_shared<const PostMotionFinalLossObservation>(source_loss_fixture());
  source.received_body_observation=std::make_shared<const ReceivedBodyObservation>(body);
  ASSERT_EQ(record().status,RecordStatus::Written);
  // A malformed optional tag on disk cannot discard an otherwise usable source.
  auto doc=YAML::LoadFile(original.snapshot_file.string());
  doc["source"]["received_body_observation"]["post_motion_final_loss"]="malformed";
  // Keep relative world-grid paths anchored to their original capture directory.
  const auto malformed_local=original.snapshot_file.parent_path()/"malformed.yaml";
  {std::ofstream stream(malformed_local); stream<<doc;}
  loaded=load_recorded_interaction_snapshot(malformed_local,&detail);
  ASSERT_TRUE(loaded) << detail;
  EXPECT_EQ(loaded->interaction_fingerprint,fingerprint);
  EXPECT_FALSE(loaded->source.received_body_observation->post_motion_final_loss);
  std::filesystem::remove_all(root);
}

TEST(MpccArchitectureSnapshot, SourceAfterFinalLossReportsIoFailureAndKeepsSlotAvailable)
{
  const auto root=output_root("post-loss-io-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  auto source=make_interaction_snapshot(mpcc_execution_contract::ControlIntent::Cruise);
  source.identity.source_context.execution_side_sign=0;
  source.identity.source_context=mpcc_execution_contract::seal_problem_context(source.identity.source_context);
  source.identity.snapshot_sec=20; source.control_prediction_origin_sec=20.1;
  source.replay_world->observed_sec=20; source.request.observation_provenance=received_body_selected();
  const auto record=[&] {return record_proof_failure(source,PipelineStage::PhysicalProof,
    "unit-post-loss-io","test",root);};
  auto original=record();
  ASSERT_EQ(original.status,RecordStatus::Written) << original.detail;
  auto body=received_body_fixture(103);
  body.post_motion_final_loss=std::make_shared<const PostMotionFinalLossObservation>(source_loss_fixture());
  source.received_body_observation=std::make_shared<const ReceivedBodyObservation>(body);
  const auto blocked=root/"after-post-motion-final-loss";
  {std::ofstream stream(blocked); stream<<"owned test obstruction";}
  EXPECT_EQ(record().status,RecordStatus::IoFailure);
  std::filesystem::remove(blocked);
  EXPECT_EQ(record().status,RecordStatus::Written);
  EXPECT_EQ(record().status,RecordStatus::Duplicate);
  std::filesystem::remove_all(root);
}

TEST(MpccArchitectureSnapshot, ReceivedBodiesKeepProgrammeAndCurrentOwnersSeparate)
{
  const auto root = output_root("received-body-owners");
  std::filesystem::remove_all(root);
  auto original = received_body_fixture(99);
  auto current = received_body_fixture(103);
  current.histories[0].push_back({20.2, current.captured_steady_ns, 2.3, .28});
  auto capture = std::make_shared<ScheduledFailureCapture>();
  capture->source_received_body_observation = std::make_shared<const ReceivedBodyObservation>(original);
  capture->current_received_body_observation = std::make_shared<const ReceivedBodyObservation>(current);
  capture->raw_observation = received_body_selected();
  capture->original.observed.decision_id = 99;
  // A new queue value and later decision cannot mutate already captured source ownership.
  original.histories[0].clear();
  current.control_decision_id = 999;
  PublicationFailureObservation observation;
  observation.decision_id = 103;
  observation.output_root = root;
  observation.scheduled_capture = capture;
  auto recorded = record_publication_failure(observation);
  ASSERT_EQ(recorded.status, RecordStatus::Written) << recorded.detail;
  auto document = YAML::LoadFile(recorded.snapshot_file.string());
  const auto source = document["scheduled"]["source_received_body_observation"];
  const auto now = document["scheduled"]["current_received_body_observation"];
  ASSERT_TRUE(source);
  ASSERT_TRUE(now);
  EXPECT_TRUE(source["control_decision_matches"].as<bool>());
  EXPECT_TRUE(now["control_decision_matches"].as<bool>());
  EXPECT_FALSE(source["selected_observation_matches"].as<bool>());  // no source prefix supplied
  EXPECT_TRUE(now["selected_observation_matches"].as<bool>());
  EXPECT_EQ(source["velocity_u_vy"].size(), 3U);
  EXPECT_EQ(now["velocity_u_vy"].size(), 4U);
  EXPECT_EQ(source["control_decision_id"].as<std::uint64_t>(), 99U);
  EXPECT_EQ(now["control_decision_id"].as<std::uint64_t>(), 103U);
  // Missing programme data must not be filled from current. Stale current data
  // is retained with explicit mismatched decision/observation association.
  capture->source_received_body_observation.reset();
  capture->raw_observation->initial.state.forward_velocity_mps += .1;
  ++observation.decision_id;
  recorded = record_publication_failure(observation);
  ASSERT_EQ(recorded.status, RecordStatus::Written) << recorded.detail;
  document = YAML::LoadFile(recorded.snapshot_file.string());
  EXPECT_EQ(document["scheduled"]["source_received_body_observation"]["status"].as<std::string>(), "missing");
  EXPECT_FALSE(document["scheduled"]["current_received_body_observation"]["control_decision_matches"].as<bool>());
  EXPECT_FALSE(document["scheduled"]["current_received_body_observation"]["selected_observation_matches"].as<bool>());
  std::filesystem::remove_all(root);
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
  artifact.vehicle_model = multi_purpose_mpc_ros::test::vehicle_model();
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
  artifact.vehicle_model = multi_purpose_mpc_ros::test::vehicle_model();
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

TEST(MpccArchitectureSnapshot, FirstBoundaryRecorderPreservesQueuedFinalWorldAndArtifact)
{
  namespace execution = mpcc_rate_resolved_execution_artifact;
  const auto root = output_root("first-authority-boundaries");
  std::filesystem::remove_all(root);
  auto world = *make_publication_observation().source;
  const auto observation = [&](std::uint64_t decision, AuthorityFailureBoundary boundary) {
      AuthorityFailureObservation value;
      value.current_world = world;
      value.current_world.identity.sequence = decision;
      auto & context = value.current_world.identity.source_context;
      context.decision_id = decision;
      context = mpcc_execution_contract::seal_problem_context(context);
      value.boundary = boundary;
      value.output_root = root;
      value.published_execution = make_publication_observation();
      auto source = *value.published_execution.source;
      source.identity.sequence = decision * 10U;
      auto artifact = *value.published_execution.artifact;
      artifact.identity = source.identity;
      value.published_execution.source =
        std::make_shared<const mpcc_rate_resolved_shadow::Snapshot>(source);
      value.published_execution.artifact =
        std::make_shared<const execution::ExecutionArtifact>(artifact);
      bind_failure_world(value.published_execution, value.current_world);
      namespace retained = mpcc_rate_resolved_retained_revalidation;
      auto plan = std::make_shared<mpcc_rate_resolved_certified_plan::CertifiedPlan>();
      plan->solver_source_snapshot = value.published_execution.source;
      plan->execution_artifact = value.published_execution.artifact;
      retained::Request current;
      current.plan = plan;
      current.decision_id = decision;
      current.now_sec = value.current_world.identity.snapshot_sec;
      current.control_origin_sec = value.current_world.control_prediction_origin_sec;
      current.current_intent = context.intent;
      value.revalidation_request = std::make_shared<const retained::Request>(current);
      auto previous = current;
      --previous.decision_id;
      previous.now_sec -= 0.025;
      previous.control_origin_sec -= 0.025;
      value.previous_accepted_revalidation_request =
        std::make_shared<const retained::Request>(previous);
      return value;
    };
  std::promise<void> entered, release;
  auto ready = entered.get_future();
  const auto released = release.get_future().share();
  std::vector<std::pair<std::uint64_t, RecordResult>> completed;
  const auto callback_thread = std::this_thread::get_id();
  bool io_off_callback = true;
  FirstAuthorityFailureRecorder recorder(
    [&](const AuthorityFailureObservation & value, const RecordResult & result) {
      io_off_callback = io_off_callback && std::this_thread::get_id() != callback_thread;
      completed.emplace_back(value.current_world.identity.sequence, result);
      if (value.current_world.identity.sequence == 1001U) {
        entered.set_value();
        released.wait();
      }
    });
  auto invalid = observation(1000U, AuthorityFailureBoundary::TerminalContingency);
  invalid.current_world.wall_grid.reset();
  EXPECT_EQ(recorder.submit(std::move(invalid)), ObservationAdmission::Invalid);
  ASSERT_EQ(recorder.submit(observation(1001U, AuthorityFailureBoundary::TerminalContingency)),
    ObservationAdmission::Queued);
  ready.wait();
  EXPECT_EQ(recorder.submit(observation(1002U, AuthorityFailureBoundary::TerminalContingency)),
    ObservationAdmission::Duplicate);
  EXPECT_EQ(recorder.submit(observation(1002U, AuthorityFailureBoundary::FinalAuthority)),
    ObservationAdmission::Queued);
  EXPECT_EQ(recorder.submit(observation(1003U, AuthorityFailureBoundary::FinalAuthority)),
    ObservationAdmission::Duplicate);
  // A startup failure cannot consume the first moving failure's evidence.
  EXPECT_EQ(recorder.submit(observation(1003U, AuthorityFailureBoundary::MovingFinalAuthority)),
    ObservationAdmission::Queued);
  EXPECT_EQ(recorder.submit(observation(1004U, AuthorityFailureBoundary::MovingFinalAuthority)),
    ObservationAdmission::Duplicate);
  // Capture the slow inspected request before a later successful Stop hides it.
  EXPECT_EQ(recorder.submit(observation(1004U, AuthorityFailureBoundary::StopAlternateOverrun)),
    ObservationAdmission::Queued);
  EXPECT_EQ(recorder.submit(observation(1005U, AuthorityFailureBoundary::StopAlternateOverrun)),
    ObservationAdmission::Duplicate);
  release.set_value();
  recorder.stop();
  ASSERT_EQ(completed.size(), 4U);
  EXPECT_EQ(completed[0].first, 1001U);
  EXPECT_EQ(completed[1].first, 1002U);
  EXPECT_EQ(completed[2].first, 1003U);
  EXPECT_EQ(completed[3].first, 1004U);
  EXPECT_TRUE(io_off_callback);
  ASSERT_EQ(completed[1].second.status, RecordStatus::Written) << completed[1].second.detail;
  const auto node = YAML::LoadFile(completed[1].second.snapshot_file.string());
  EXPECT_EQ(node["source"]["sequence"].as<std::uint64_t>(), 1002U);
  EXPECT_EQ(node["publication_bundle"]["execution_evidence"]["source_sequence"].as<std::uint64_t>(),
    10020U);
  EXPECT_EQ(node["failure_outcome"].as<std::string>(), "normal-authority-unavailable");
  const auto previous = node["previous_accepted_revalidation_evidence"];
  ASSERT_EQ(previous["status"].as<std::string>(), "present");
  EXPECT_EQ(previous["request"]["decision_id"].as<std::uint64_t>(), 1001U);
  EXPECT_EQ(previous["inspected_artifact"]["source_sequence"].as<std::uint64_t>(), 10020U);
  ASSERT_EQ(completed[2].second.status, RecordStatus::Written) << completed[2].second.detail;
  const auto moving = YAML::LoadFile(completed[2].second.snapshot_file.string());
  EXPECT_EQ(moving["source"]["sequence"].as<std::uint64_t>(), 1003U);
  EXPECT_EQ(moving["failure_outcome"].as<std::string>(), "moving-normal-authority-unavailable");
  EXPECT_EQ(moving["publication_bundle"]["execution_evidence"]["source_sequence"].as<std::uint64_t>(),
    10030U);
  ASSERT_EQ(completed[3].second.status, RecordStatus::Written) << completed[3].second.detail;
  const auto slow = YAML::LoadFile(completed[3].second.snapshot_file.string());
  EXPECT_EQ(slow["failure_outcome"].as<std::string>(), "stop-alternate-revalidation-overrun");
  EXPECT_EQ(slow["revalidation_evidence"]["status"].as<std::string>(), "present");
  EXPECT_EQ(slow["revalidation_evidence"]["request"]["decision_id"].as<std::uint64_t>(), 1004U);
  EXPECT_EQ(recorder.submit(observation(1004U, AuthorityFailureBoundary::FinalAuthority)),
    ObservationAdmission::Stopped);
  std::filesystem::remove_all(root);
}

TEST(MpccArchitectureSnapshot, PhysicalPlanWithoutSolverSourceKeepsCompleteObservation)
{
  namespace certified = mpcc_rate_resolved_certified_plan;
  namespace physical = mpcc_rate_resolved_physical_wall;
  namespace retained = mpcc_rate_resolved_retained_revalidation;
  auto published = make_publication_observation();
  auto world = *published.source;
  world.identity.snapshot_sec = 12.8;
  world.identity.source_context.decision_id = 55U;
  world.identity.source_context = mpcc_execution_contract::seal_problem_context(
    world.identity.source_context);
  world.control_prediction_origin_sec = 12.9;
  world.replay_world->observed_sec = 12.8;
  bind_failure_world(published, world);

  // A real physical proof is available even when no solver snapshot exists.
  // This straight-line fixture tests observation loss, not race feasibility.
  physical::Snapshot snapshot;
  snapshot.identity.artifact = published.artifact->identity;
  snapshot.identity.captured_sec = 12.6;
  snapshot.current_pose = {0.0, 0.0, 0.0};
  snapshot.control_prefix = {snapshot.current_pose};
  snapshot.identity.pose_snapshot_id = physical::fingerprint_control_pose_path(
    snapshot.control_prefix, snapshot.current_pose);
  snapshot.course_frame_knots = world.wall_course_frame_knots;
  snapshot.identity.course_frame_window_id = physical::fingerprint_course_frame_window(
    snapshot.course_frame_knots);
  auto grid = std::make_shared<recovery_footprint::OccupancyGrid>();
  grid->width = 40U; grid->height = 40U; grid->resolution_m = 0.5;
  grid->origin_x_m = -10.0; grid->origin_y_m = -10.0;
  grid->cells.assign(1600U, recovery_footprint::CellState::Free);
  snapshot.wall_grid = grid;
  snapshot.wall_grid_fingerprint = recovery_footprint::occupancy_grid_fingerprint(*grid);
  snapshot.footprint = {0.1, 0.1, 0.1, 0.1, 0.01};
  snapshot.hard_wall_clearance_m = 0.05;
  snapshot.bound_tolerance_m = 1e-6;
  snapshot.swept_step_m = 0.05;
  auto & trajectory = snapshot.trajectory;
  trajectory.progress_origin_m = 3.0;
  trajectory.elapsed_time_sec = {0.1}; trajectory.path_distance_m = {0.2};
  trajectory.lateral_m = {0.0}; trajectory.lag_m = {0.0};
  trajectory.heading_offset_rad = {0.0}; trajectory.velocity_mps = {2.0};
  trajectory.progress_m = {3.2}; trajectory.lateral_lower_m = {-1.0};
  trajectory.lateral_upper_m = {1.0}; trajectory.minimum_lateral_bound_reserve_m = 1.0;
  snapshot.terminal_stop_course_geometry = {{0.0, 1.0}, {0.0}, {-1.0, -1.0}, {1.0, 1.0}};
  ASSERT_TRUE(physical::snapshot_valid(snapshot));
  const auto proof = physical::evaluate(snapshot);
  ASSERT_EQ(proof.outcome, physical::Outcome::Accepted);
  const auto built = certified::build(published.artifact, snapshot, proof);
  ASSERT_EQ(built.reason, certified::RejectReason::None);
  ASSERT_NE(built.plan, nullptr);
  ASSERT_EQ(built.plan->solver_source_snapshot, nullptr);
  published.source.reset();
  published.certified_plan = built.plan;

  retained::Request request;
  request.plan = built.plan;
  request.decision_id = 55U; request.now_sec = 12.8; request.control_origin_sec = 12.9;
  request.current_intent = world.identity.source_context.intent;
  const auto root = output_root("physical-plan-without-solver-source");
  std::filesystem::remove_all(root);
  const auto recorded = record_authority_failure(world, "derived-plan-observation", "fixture",
    published, root, std::make_shared<const retained::Request>(request));
  ASSERT_EQ(recorded.status, RecordStatus::Written) << recorded.detail;
  const auto document = YAML::LoadFile(recorded.snapshot_file.string());
  // Legacy solver-source fields remain explicit missing, never fabricated.
  EXPECT_EQ(document["publication_bundle"]["status"].as<std::string>(), "missing");
  for (const auto * role : {"publication_bundle", "revalidation_evidence"}) {
    const auto evidence = document[role]["certified_plan_evidence"];
    ASSERT_TRUE(evidence) << role;
    EXPECT_EQ(evidence["status"].as<std::string>(), "present");
    EXPECT_EQ(evidence["solver_source_status"].as<std::string>(), "absent");
    EXPECT_EQ(evidence["artifact"]["source_sequence"].as<std::uint64_t>(),
      published.artifact->identity.sequence);
    EXPECT_DOUBLE_EQ(evidence["physical_snapshot"]["trajectory"]["path_distance_m"][0].as<double>(), 0.2);
    EXPECT_EQ(evidence["physical_snapshot"]["wall_grid"]["fingerprint"].as<std::uint64_t>(),
      snapshot.wall_grid_fingerprint);
    const auto payload = evidence["physical_snapshot"]["wall_grid"]["payload"].as<std::string>();
    EXPECT_EQ(std::filesystem::file_size(recorded.snapshot_file.parent_path()/payload), grid->cells.size());
  }
  auto invalid_publication = published;
  auto invalid_artifact = std::make_shared<mpcc_rate_resolved_execution_artifact::ExecutionArtifact>(
    *published.artifact);
  invalid_artifact->semantic_initial_state.reset();
  invalid_publication.artifact = invalid_artifact;
  const auto invalid_record = record_authority_failure(world, "invalid-derived-publication", "fixture",
    invalid_publication, root, std::make_shared<const retained::Request>(request));
  ASSERT_EQ(invalid_record.status, RecordStatus::Written) << invalid_record.detail;
  const auto invalid_document = YAML::LoadFile(invalid_record.snapshot_file.string());
  EXPECT_EQ(invalid_document["publication_bundle"]["certified_plan_evidence"]["status"].as<std::string>(),
    "invalid");
  EXPECT_EQ(invalid_document["revalidation_evidence"]["certified_plan_evidence"]["status"].as<std::string>(),
    "present");
  std::filesystem::remove_all(root);
}

TEST(MpccArchitectureSnapshot, ExactRejectedRequestKeepsInspectedAndPublishedArtifactsSeparate)
{
  namespace retained = mpcc_rate_resolved_retained_revalidation;
  namespace execution = mpcc_rate_resolved_execution_artifact;
  auto published = make_publication_observation();
  auto world = *published.source;
  world.identity.sequence = 951U;
  world.identity.source_context.decision_id = 951U;
  world.identity.source_context = mpcc_execution_contract::seal_problem_context(
    world.identity.source_context);
  bind_failure_world(published, world);
  auto inspected_source = *published.source;
  inspected_source.identity.sequence = 379U;
  auto inspected_artifact = *published.artifact;
  inspected_artifact.identity = inspected_source.identity;
  auto plan = std::make_shared<mpcc_rate_resolved_certified_plan::CertifiedPlan>();
  plan->solver_source_snapshot =
    std::make_shared<const mpcc_rate_resolved_shadow::Snapshot>(inspected_source);
  plan->execution_artifact = std::make_shared<const execution::ExecutionArtifact>(inspected_artifact);
  retained::Request request;
  request.plan = plan;
  request.decision_id = 951U;
  request.now_sec = world.identity.snapshot_sec;
  request.control_origin_sec = world.control_prediction_origin_sec;
  request.current_intent = world.identity.source_context.intent;
  request.execution_clock = {retained::ExecutionClockKind::PublishedPlan, 12.51, 0.03712345678901234};
  request.current_speed_mps = 1.718736123456789;
  request.control_origin_speed_mps = 1.861113123456789;
  request.current_time_steering_rad = -0.050562123456789;
  request.current_steering_rad = -0.128003123456789;
  request.current_response_steering_rad = -0.061234567890123;
  request.previous_published_steering_rad = request.current_steering_rad;
  request.previous_published_command_age_sec = 0.014999999;
  request.control_origin_physical_progress_m = 30.677949123456789;
  request.path_length_m = 997.1234567890123;
  request.progress_continuity_tolerance_m = 1.5;
  request.circular = true;
  request.minimum_acceleration_mps2 = -3.0;
  request.maximum_acceleration_mps2 = 1.37;
  const auto & replay = *world.replay_world;
  request.control_pose = replay.control_prefix.back();
  request.measured_to_control_path = replay.control_prefix;
  request.measured_to_control_elapsed_sec = replay.control_prefix_elapsed_sec;
  request.current_wall_grid = world.wall_grid;
  request.current_footprint = replay.physical_footprint;
  request.stop_lateral_policy = replay.terminal_stop_lateral_policy;
  request.obstacles = {replay.observation_generation, replay.observed_sec, {}, replay.current};
  for (const auto & peer : replay.obstacles) {
    request.obstacles.obstacles.push_back({peer.id,
      {peer.x_m, peer.y_m, peer.velocity_x_mps, peer.velocity_y_mps, peer.radius_m}});
  }
  const auto root = output_root("exact-inspected-request");
  std::filesystem::remove_all(root);
  const auto fingerprint = fingerprint_interaction_snapshot(world);
  for (int variant = 0; variant < 4; ++variant) {
    auto observed_request = request;
    if (variant == 1) {
      ++observed_request.decision_id;
    } else if (variant == 2) {
      auto invalid_plan = std::make_shared<mpcc_rate_resolved_certified_plan::CertifiedPlan>(*plan);
      auto invalid_artifact = inspected_artifact;
      ++invalid_artifact.identity.sequence;
      invalid_plan->execution_artifact =
        std::make_shared<const execution::ExecutionArtifact>(invalid_artifact);
      observed_request.plan = invalid_plan;
    } else if (variant == 3) {
      observed_request.plan.reset();
    }
    const auto recorded = record_authority_failure(world, "inspection-fixture-" + std::to_string(variant),
      "Synthetic request fixture; no certified-plan or physical acceptance claim", published, root,
      std::make_shared<const retained::Request>(observed_request));
    ASSERT_EQ(recorded.status, RecordStatus::Written) << recorded.detail;
    std::string detail;
    const auto loaded = load_recorded_interaction_snapshot(recorded.snapshot_file, &detail);
    ASSERT_TRUE(loaded) << detail;
    EXPECT_EQ(loaded->interaction_fingerprint, fingerprint);
    const auto node = YAML::LoadFile(recorded.snapshot_file.string());
    const auto evidence = node["revalidation_evidence"];
    EXPECT_EQ(evidence["schema"].as<std::string>(), "mpcc-revalidation-observation/v1");
    EXPECT_EQ(evidence["status"].as<std::string>(), variant == 1 ? "invalid" : "present");
    EXPECT_EQ(node["publication_bundle"]["execution_evidence"]["source_sequence"].as<std::uint64_t>(),
      published.artifact->identity.sequence);
    EXPECT_DOUBLE_EQ(evidence["request"]["current_time_steering_rad"].as<double>(),
      request.current_time_steering_rad);
    EXPECT_DOUBLE_EQ(evidence["request"]["current_speed_mps"].as<double>(), request.current_speed_mps);
    EXPECT_DOUBLE_EQ(evidence["request"]["previous_published_command_age_sec"].as<double>(),
      request.previous_published_command_age_sec);
    EXPECT_DOUBLE_EQ(evidence["request"]["execution_clock"]["first_published_artifact_elapsed_sec"].as<double>(),
      request.execution_clock.first_published_artifact_elapsed_sec);
    EXPECT_TRUE(std::filesystem::exists(recorded.snapshot_file.parent_path()/"revalidation-wall-grid.bin"));
    if (variant >= 2) {
      EXPECT_EQ(evidence["inspected_plan_status"].as<std::string>(), variant == 2 ? "invalid" : "missing");
      EXPECT_FALSE(evidence["inspected_artifact"]);
    } else {
      EXPECT_EQ(evidence["inspected_plan_status"].as<std::string>(), "present");
      EXPECT_EQ(evidence["inspected_artifact"]["source_sequence"].as<std::uint64_t>(), 379U);
      EXPECT_FALSE(evidence["inspected_artifact"]["publication"]);
      EXPECT_TRUE(std::filesystem::exists(recorded.snapshot_file.parent_path()/"inspected-wall-grid.bin"));
    }
  }
  EXPECT_EQ(fingerprint_interaction_snapshot(world), fingerprint);
  std::filesystem::remove_all(root);
}

TEST(MpccArchitectureSnapshot, PreviousAcceptedRequestKeepsItsOwnWorldAndRejectsFalsePairing)
{
  namespace retained = mpcc_rate_resolved_retained_revalidation;
  auto published = make_publication_observation();
  auto world = *published.source;
  world.identity.source_context.decision_id = 941U;
  world.identity.source_context = mpcc_execution_contract::seal_problem_context(
    world.identity.source_context);
  bind_failure_world(published, world);
  auto plan = std::make_shared<mpcc_rate_resolved_certified_plan::CertifiedPlan>();
  plan->solver_source_snapshot = published.source;
  plan->execution_artifact = published.artifact;
  retained::Request current;
  current.plan = plan;
  current.decision_id = 941U;
  current.now_sec = world.identity.snapshot_sec;
  current.control_origin_sec = world.control_prediction_origin_sec;
  current.current_intent = world.identity.source_context.intent;
  current.current_wall_grid = world.wall_grid;
  current.control_pose = {11.123456789012345, 21.0, -0.07};
  current.current_speed_mps = 1.4016329817392303;
  current.obstacles = {167U, current.now_sec, {{"d2", {31.0, 41.0, -1.0, 1.0, 1.931}}}, true};
  auto previous = current;
  previous.decision_id = 939U;
  previous.now_sec -= 0.025;
  previous.control_origin_sec -= 0.025;
  previous.current_speed_mps = 1.3912345678901234;
  previous.control_pose = {11.023456789012345, 20.0, -0.08};
  previous.obstacles = {166U, previous.now_sec, {{"d2", {30.0, 40.0, -0.9, 1.1, 1.931}}}, true};
  auto old_grid = std::make_shared<recovery_footprint::OccupancyGrid>(*world.wall_grid);
  ASSERT_FALSE(old_grid->cells.empty());
  using Cell = recovery_footprint::CellState;
  old_grid->cells[0] = old_grid->cells[0] == Cell::Free ? Cell::Occupied : Cell::Free;
  previous.current_wall_grid = old_grid;
  const auto root = output_root("previous-ordinary-accepted-request");
  std::filesystem::remove_all(root);
  const auto fingerprint = fingerprint_interaction_snapshot(world);
  for (int variant = 0; variant < 7; ++variant) {
    auto observed = previous;
    if (variant == 1) {
      observed.decision_id = current.decision_id;
    } else if (variant == 2) {
      observed.now_sec = current.now_sec + 0.001;
    } else if (variant == 3) {
      auto other_plan = std::make_shared<mpcc_rate_resolved_certified_plan::CertifiedPlan>(*plan);
      auto other_artifact = *plan->execution_artifact;
      ++other_artifact.identity.sequence;
      other_plan->execution_artifact =
        std::make_shared<const mpcc_rate_resolved_execution_artifact::ExecutionArtifact>(other_artifact);
      observed.plan = other_plan;
    } else if (variant == 4) {
      observed.plan.reset();
    } else if (variant == 5) {
      observed.control_origin_sec = current.control_origin_sec + 0.001;
    }
    const auto recorded = record_authority_failure(world,
      "previous-request-" + std::to_string(variant),
      "Synthetic serialization fixture; no physical acceptance claim", published, root,
      std::make_shared<const retained::Request>(current), variant == 6 ? nullptr :
      std::make_shared<const retained::Request>(observed));
    ASSERT_EQ(recorded.status, RecordStatus::Written) << recorded.detail;
    const auto loaded = load_recorded_interaction_snapshot(recorded.snapshot_file);
    ASSERT_TRUE(loaded);
    EXPECT_EQ(loaded->interaction_fingerprint, fingerprint);
    const auto node = YAML::LoadFile(recorded.snapshot_file.string());
    const auto before = node["previous_accepted_revalidation_evidence"];
    const auto after = node["revalidation_evidence"];
    EXPECT_EQ(after["status"].as<std::string>(), "present");
    EXPECT_EQ(before["status"].as<std::string>(),
      variant == 0 ? "present" : variant == 6 ? "missing" : "invalid");
    EXPECT_DOUBLE_EQ(after["request"]["control_pose"]["x_m"].as<double>(), current.control_pose.x_m);
    if (variant != 6) {
      EXPECT_DOUBLE_EQ(before["request"]["control_pose"]["x_m"].as<double>(), previous.control_pose.x_m);
      EXPECT_DOUBLE_EQ(before["request"]["current_speed_mps"].as<double>(), previous.current_speed_mps);
      EXPECT_EQ(before["request"]["obstacles"]["generation"].as<std::uint64_t>(), 166U);
      EXPECT_EQ(after["request"]["obstacles"]["generation"].as<std::uint64_t>(), 167U);
      EXPECT_EQ(before["request"]["current_wall_grid"]["payload"].as<std::string>(),
        "previous-revalidation-wall-grid.bin");
      const auto byte = [&](const std::string & name) {
          std::ifstream stream(recorded.snapshot_file.parent_path()/name, std::ios::binary);
          return stream.get();
        };
      EXPECT_NE(byte("previous-revalidation-wall-grid.bin"), byte("revalidation-wall-grid.bin"));
    }
    if (variant == 0) {
      EXPECT_EQ(before["inspected_plan_status"].as<std::string>(), "present");
      EXPECT_FALSE(before["inspected_artifact"]["publication"]);
      EXPECT_TRUE(std::filesystem::exists(recorded.snapshot_file.parent_path()/"previous-inspected-wall-grid.bin"));
      EXPECT_DOUBLE_EQ(before["request"]["now_sec"].as<double>(), previous.now_sec);
      EXPECT_DOUBLE_EQ(after["request"]["now_sec"].as<double>(), current.now_sec);
    }
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
  auto snapshot = make_interaction_snapshot(
    mpcc_execution_contract::ControlIntent::Pass);
  snapshot.replay_world->obstacles.front().acceleration_x_mps2 = 0.6;
  snapshot.replay_world->obstacles.front().acceleration_y_mps2 = -0.8;
  snapshot.replay_world->obstacles.front().acceleration_horizon_sec = 1.0;
  snapshot.request.observation_provenance = mpcc_vehicle_model::ObservationProvenance{
    {19.8, {1, 2, .1, 2, .2, .3, .05, .04}}, 19.79, 19.8, 19.78,
    20.0, 20.1, 0, .1, {{19.0, 1.0, .1}, {19.9, -3.0, .2}}};
  snapshot.wall_course_frame_knots.push_back({5.0, 2.0, 0.0, .1, 12});
  snapshot.terminal_stop_course_geometry =
    mpcc_rate_resolved_physical_adapter::StopCourseGeometry{
    {0, 1, 2}, {0, .1}, {-2, -2, -1.5}, {2, 2, 1.5}};
  auto assembly = make_assembly_request();
  mpcc_rate_resolved_problem::DynamicObstacleConstraint plane;
  plane.state_stage = 1;
  plane.upper = 0.4;
  Eigen::Matrix<double, mpcc_rate_resolved::kStateDimension, 1> coefficients;
  coefficients << 0.2, -0.1, 0.3, 0.0, 0.7, 0.0, 0.0, 0.0, 0.0;
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
  EXPECT_DOUBLE_EQ(loaded->source.replay_world->obstacles.front().acceleration_horizon_sec, 1.0);
  auto peer_mutated = loaded->source;
  peer_mutated.replay_world->obstacles.front().acceleration_horizon_sec = 0.0;
  EXPECT_FALSE(interaction_snapshot_matches_fingerprint(peer_mutated, loaded->interaction_fingerprint));
  peer_mutated.replay_world->obstacles.front().acceleration_horizon_sec = -1.0;
  EXPECT_FALSE(interaction_snapshot_complete(peer_mutated));
  ASSERT_TRUE(loaded->source.terminal_stop_course_geometry);
  EXPECT_EQ(loaded->source.terminal_stop_course_geometry->progress_m,
    (std::vector<double>{0, 1, 2}));
  auto support_mutated = loaded->source;
  support_mutated.terminal_stop_course_geometry->curvature_radpm.back() += .01;
  EXPECT_FALSE(interaction_snapshot_matches_fingerprint(
      support_mutated, loaded->interaction_fingerprint));
  support_mutated = loaded->source;
  support_mutated.terminal_stop_course_geometry->progress_m.back() = 3;
  EXPECT_FALSE(interaction_snapshot_complete(support_mutated));
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

  ASSERT_TRUE(loaded->source.request.observation_provenance.has_value());
  EXPECT_DOUBLE_EQ(loaded->source.request.observation_provenance->velocity_source_sec, 19.79);
  EXPECT_DOUBLE_EQ(loaded->source.request.observation_provenance->initial.state.yaw_rate_radps, .3);
  for (bool change_epoch : {false, true}) {
    auto changed = loaded->source;
    auto & provenance = *changed.request.observation_provenance;
    if (change_epoch) provenance.tire_source_sec -= .01;
    else provenance.commands.back().wire_acceleration_mps2 += .1;
    EXPECT_FALSE(interaction_snapshot_matches_fingerprint(changed, loaded->interaction_fingerprint));
  }
  auto invalid = loaded->source;
  invalid.request.observation_provenance->velocity_source_sec = 20.1;
  EXPECT_FALSE(interaction_snapshot_complete(invalid));

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


TEST(MpccArchitectureSnapshot, NativeInitializerIsSerializedAndBoundToFingerprint)
{
  const auto root = output_root("native-initializer-roundtrip");
  std::filesystem::remove_all(root);
  auto snapshot = make_interaction_snapshot(mpcc_execution_contract::ControlIntent::Pass);
  const auto original_fingerprint = fingerprint_interaction_snapshot(snapshot);
  snapshot.request.initial_tangent_policy = mpcc_rate_resolved_adapter::InitialTangentPolicy::ReferenceSteeringWithRestLaunch;
  const auto reference_fingerprint = fingerprint_interaction_snapshot(snapshot);
  ASSERT_NE(reference_fingerprint, 0U);
  EXPECT_NE(reference_fingerprint, original_fingerprint);
  const auto written = record_failure(snapshot,make_assembly_request(),make_valid_problem(),std::nullopt,
    persistent_osqp::SolveOutcome{},PipelineStage::Initial,"native-initializer","intentional test",root);
  ASSERT_EQ(written.status, RecordStatus::Written) << written.detail;
  const auto loaded = load_recorded_interaction_snapshot(written.snapshot_file);
  ASSERT_TRUE(loaded);
  EXPECT_EQ(loaded->source.request.initial_tangent_policy,snapshot.request.initial_tangent_policy);
  EXPECT_EQ(loaded->interaction_fingerprint,reference_fingerprint);
  auto mutated = loaded->source;
  mutated.request.initial_tangent_policy = mpcc_rate_resolved_adapter::InitialTangentPolicy::CurrentSteering;
  EXPECT_FALSE(interaction_snapshot_matches_fingerprint(mutated,reference_fingerprint));
  mutated.request.initial_tangent_policy = static_cast<mpcc_rate_resolved_adapter::InitialTangentPolicy>(99);
  EXPECT_FALSE(interaction_snapshot_complete(mutated));
  auto document = YAML::LoadFile(written.snapshot_file.string());
  document["source"]["semantic_request"]["initial_tangent_policy"] = 99;
  { std::ofstream file(written.snapshot_file); file << document; }
  EXPECT_FALSE(load_recorded_interaction_snapshot(written.snapshot_file));
}

}  // namespace
}  // namespace multi_purpose_mpc_ros::mpcc_architecture_snapshot
