#include "multi_purpose_mpc_ros/mpcc_stateless_maneuver.hpp"

#include "multi_purpose_mpc_ros/mpcc_architecture_snapshot.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <vector>

namespace multi_purpose_mpc_ros::mpcc_stateless_maneuver
{
namespace
{

mpcc_rate_resolved_shadow::Snapshot make_source()
{
  namespace contract = mpcc_execution_contract;
  namespace model = mpcc_rate_resolved;
  namespace shadow = mpcc_rate_resolved_shadow;
  shadow::Snapshot source;
  source.identity.sequence = 9U;
  source.identity.snapshot_sec = 20.0;
  source.identity.source_context.decision_id = 7U;
  source.identity.source_context.intent = contract::ControlIntent::ShiftOut;
  source.identity.source_context.intent_generation = 4U;
  source.identity.source_context.observation_generation = 11U;
  source.identity.source_context.stage_geometry_id = 12U;
  source.identity.source_context.target_obstacle_generation = 13U;
  source.identity.source_context.target_id = "d2";
  source.identity.source_context.execution_side_sign = 1;
  source.identity.source_context.dynamic_obstacle_constraint_active = true;
  source.identity.source_context.dynamic_obstacle_generation = 13U;
  source.identity.source_context.dynamic_obstacle_id = "d2";
  source.identity.source_context.dynamic_obstacle_side_sign = 1;
  source.identity.source_context.horizon_steps = 3U;
  source.identity.source_context.formulation =
    contract::Formulation::VelocitySteeringYawResponseProgress7State;
  source.identity.source_context.state_schema_id = "state-7";
  source.identity.source_context.input_schema_id = "input-3";
  source.identity.source_context.bounds_schema_id = "bounds";
  source.identity.source_context.cost_schema_id = "cost";
  source.identity.source_context = contract::seal_problem_context(
    source.identity.source_context);
  source.control_prediction_origin_sec = 20.1;
  source.execution_prefix_steps = 3;
  source.course_progress_origin_m = 100.0;
  source.nominal_path_distance_m = {0.0, 1.0, 2.0, 3.0};
  source.publication_interval_sec = 0.025;

  auto & request = source.request;
  request.horizon_steps = 3;
  request.initial_state << 0.0, 0.0, 0.0, 4.0, 0.0;
  request.current_steering_rad = 0.0;
  request.current_response_steering_rad = 0.0;
  request.wheelbase_m = 1.0;
  request.yaw_response_gain = 1.0;
  request.yaw_response_time_constant_sec = 0.1;
  request.maximum_abs_steering_rad = 0.5;
  request.maximum_abs_steering_rate_radps = 1.0;
  request.states.resize(4U);
  for (std::size_t index = 0U; index < request.states.size(); ++index) {
    auto & state = request.states[index];
    state.reference << 1.25, 0.2, 0.3, 4.0, static_cast<double>(index);
    state.lower << -2.0, -1.0, -0.6, 0.0, 0.0;
    state.upper << 2.0, 1.0, 0.6, 8.0, 5.0;
    state.weight.setOnes();
  }
  request.states.front().lower = request.initial_state;
  request.states.front().upper = request.initial_state;
  request.inputs.resize(3U);
  for (auto & input : request.inputs) {
    input.reference << 0.0, 0.0, 4.0;
    input.lower << -2.0, -0.5, 0.0;
    input.upper << 1.0, 0.5, 8.0;
    input.weight.setOnes();
    input.stage_dt_sec = 0.1;
  }
  source.progress_aligned_wall_refinement_active = true;
  source.wall_reference_progress_m = {0.0, 1.0, 2.0, 3.0};
  source.wall_lower_m = {-2.0, -2.0, -2.0, -2.0};
  source.wall_upper_m = {2.0, 2.0, 2.0, 2.0};
  source.dynamic_obstacle_refinement_active = true;
  source.dynamic_obstacle_pass_side_sign = 1;
  source.dynamic_obstacle_stages = {
    {true, 1.0, 0.0, 0.6, 0.8},
    {true, 2.0, 0.0, 0.6, 0.8},
    {true, 5.0, 0.0, 0.6, 0.8}};
  source.physical_wall_refinement_active = true;
  auto grid = std::make_shared<recovery_footprint::OccupancyGrid>();
  grid->width = 2U;
  grid->height = 2U;
  grid->resolution_m = 1.0;
  grid->cells.assign(4U, recovery_footprint::CellState::Free);
  source.wall_grid = grid;
  source.wall_footprint.front_extent_m = 1.0;
  source.wall_footprint.rear_extent_m = 1.0;
  source.wall_footprint.left_extent_m = 0.725;
  source.wall_footprint.right_extent_m = 0.725;
  source.wall_course_frame_knots = {
    {99.0, -1.0, 0.0, 0.0, 1},
    {104.0, 4.0, 0.0, 0.0, 2}};
  source.wall_lateral_sample_step_m = 0.1;
  source.wall_translation_bucket_width_m = 0.1;

  source.replay_world.emplace();
  auto & world = source.replay_world.value();
  world.observation_generation = 13U;
  world.observed_sec = 20.0;
  world.current = true;
  world.current_pose = {0.0, 0.0, 0.0};
  world.control_prefix = {{0.0, 0.0, 0.0}, {0.1, 0.0, 0.0}};
  world.control_prefix_elapsed_sec = {0.0, 0.1};
  world.physical_footprint.front_extent_m = 1.0;
  world.physical_footprint.rear_extent_m = 1.0;
  world.physical_footprint.left_extent_m = 0.525;
  world.physical_footprint.right_extent_m = 0.525;
  world.wall_grid_fingerprint =
    recovery_footprint::occupancy_grid_fingerprint(*grid);
  world.hard_wall_clearance_m = 0.2;
  world.bound_tolerance_m = 1e-5;
  world.swept_step_m = 0.1;
  world.obstacles.push_back(
    shadow::ReplayDynamicObstacle{
      "d2", 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.01, 0.8, 13U});
  EXPECT_TRUE(mpcc_architecture_snapshot::interaction_snapshot_complete(source));
  return source;
}

mpcc_rate_resolved_shadow::Snapshot make_return_source()
{
  auto source = make_source();
  source.identity.source_context.intent =
    mpcc_execution_contract::ControlIntent::Return;
  source.identity.source_context.fingerprint = 0U;
  source.identity.source_context =
    mpcc_execution_contract::seal_problem_context(
    source.identity.source_context);
  for (auto & state : source.request.states) {
    state.reference[mpcc_rate_resolved::kLateralIndex] = 0.0;
    state.reference[mpcc_rate_resolved::kHeadingIndex] = 0.0;
  }
  source.terminal_intent_contract.active = true;
  source.terminal_intent_contract.lateral_reference_m = 0.0;
  source.terminal_intent_contract.lateral_tolerance_m = 0.20;
  source.terminal_intent_contract.heading_reference_rad = 0.0;
  source.terminal_intent_contract.heading_tolerance_rad = 0.12;
  EXPECT_TRUE(mpcc_architecture_snapshot::interaction_snapshot_complete(source));
  return source;
}

TEST(MpccStatelessManeuver, BuildsBothSidesWithoutMissionGeometry)
{
  const auto source = make_source();
  const auto source_fingerprint =
    mpcc_architecture_snapshot::fingerprint_interaction_snapshot(source);
  const auto left = build(source, source_fingerprint, 1);
  const auto right = build(source, source_fingerprint, -1);
  ASSERT_TRUE(left.seed.has_value()) << left.detail;
  ASSERT_TRUE(right.seed.has_value()) << right.detail;
  EXPECT_EQ(left.reason, RejectReason::Accepted);
  EXPECT_EQ(right.reason, RejectReason::Accepted);
  EXPECT_EQ(left.seed->terminal_successor, TerminalSuccessor::Replan);
  EXPECT_EQ(right.seed->terminal_successor, TerminalSuccessor::Replan);
  EXPECT_GT(left.seed->lateral_reference_m[1], 0.0);
  EXPECT_LT(right.seed->lateral_reference_m[1], 0.0);
  EXPECT_NE(left.seed->candidate_fingerprint, right.seed->candidate_fingerprint);
}

TEST(MpccStatelessManeuver, ReturnBehindTargetPreservesRejoinReference)
{
  auto source = make_return_source();
  const auto population = build_bounded_candidates(source, 1);

  ASSERT_EQ(population.reason, RejectReason::Accepted) << population.detail;
  ASSERT_EQ(population.candidates.size(), 1U);
  const auto & candidate = population.candidates.front();
  EXPECT_EQ(candidate.kind, CandidateKind::ReturnRejoin);
  EXPECT_EQ(candidate.seed.pass_side_sign, 0);
  EXPECT_EQ(
    candidate.seed.solver_snapshot.dynamic_obstacle_longitudinal_topology,
    mpcc_rate_resolved_dynamic_obstacle::LongitudinalTopology::StayBehind);
  EXPECT_EQ(candidate.seed.solver_snapshot.dynamic_obstacle_pass_side_sign, 0);
  EXPECT_EQ(candidate.seed.lateral_reference_m, std::vector<double>(4U, 0.0));
  const auto & terminal = candidate.seed.solver_snapshot.request.states.back();
  EXPECT_DOUBLE_EQ(terminal.lower[mpcc_rate_resolved::kLateralIndex], -0.20);
  EXPECT_DOUBLE_EQ(terminal.upper[mpcc_rate_resolved::kLateralIndex], 0.20);
  EXPECT_DOUBLE_EQ(terminal.lower[mpcc_rate_resolved::kHeadingIndex], -0.12);
  EXPECT_DOUBLE_EQ(terminal.upper[mpcc_rate_resolved::kHeadingIndex], 0.12);
}

TEST(MpccStatelessManeuver, ReturnAheadOfRearClearTargetStaysAhead)
{
  auto source = make_return_source();
  source.request.initial_state[mpcc_rate_resolved::kProgressIndex] = 3.0;
  source.request.states.front().lower = source.request.initial_state;
  source.request.states.front().upper = source.request.initial_state;
  const auto population = build_bounded_candidates(source, -1);

  ASSERT_EQ(population.reason, RejectReason::Accepted) << population.detail;
  ASSERT_EQ(population.candidates.size(), 1U);
  EXPECT_EQ(
    population.candidates.front().seed.solver_snapshot.
    dynamic_obstacle_longitudinal_topology,
    mpcc_rate_resolved_dynamic_obstacle::LongitudinalTopology::StayAhead);
  EXPECT_EQ(population.candidates.front().seed.pass_side_sign, 0);
}

TEST(MpccStatelessManeuver, AuditReturnScheduleDiscardsCapturedMissionReference)
{
  auto source = make_return_source();
  source.replay_world->obstacles.front().x_m = 3.0;
  source.request.initial_state[mpcc_rate_resolved::kLateralIndex] = 1.0;
  source.request.states.front().lower = source.request.initial_state;
  source.request.states.front().upper = source.request.initial_state;
  for (std::size_t index = 1U; index + 1U < source.request.states.size(); ++index) {
    source.request.states[index].reference[
      mpcc_rate_resolved::kLateralIndex] = -1.5;
  }
  const auto fingerprint =
    mpcc_architecture_snapshot::fingerprint_interaction_snapshot(source);
  const auto production_b = build(source, fingerprint, 1);
  const auto audit_c = build_return_rejoin_schedule(
    source, fingerprint, 0, source.request.horizon_steps);

  ASSERT_TRUE(production_b.seed.has_value()) << production_b.detail;
  ASSERT_TRUE(audit_c.seed.has_value()) << audit_c.detail;
  ASSERT_EQ(audit_c.seed->lateral_reference_m.size(), 4U);
  EXPECT_DOUBLE_EQ(audit_c.seed->lateral_reference_m.front(), 1.0);
  EXPECT_GT(audit_c.seed->lateral_reference_m[1], 0.0);
  EXPECT_LT(audit_c.seed->lateral_reference_m[1], 1.0);
  EXPECT_DOUBLE_EQ(audit_c.seed->lateral_reference_m.back(), 0.0);
  EXPECT_EQ(production_b.seed->lateral_reference_m[1], -1.5);
  EXPECT_NE(
    audit_c.seed->candidate_fingerprint,
    production_b.seed->candidate_fingerprint);
  EXPECT_LT(
    audit_c.seed->solver_snapshot.request.states[1].reference[
      mpcc_rate_resolved::kHeadingIndex], 0.0);
}

TEST(MpccStatelessManeuver, ReturnRejectsUnseparatedAmbiguousRelation)
{
  auto source = make_return_source();
  source.dynamic_obstacle_stages.front().target_progress_m = 0.0;
  source.dynamic_obstacle_stages.front().target_lateral_m = 0.0;
  const auto population = build_bounded_candidates(source, 1);

  EXPECT_EQ(population.reason, RejectReason::DynamicTargetUnavailable);
  EXPECT_TRUE(population.candidates.empty());
  EXPECT_NE(population.detail.find("neither"), std::string::npos);
}

TEST(MpccStatelessManeuver, BuildsDistinctSmoothLatticeTransition)
{
  const auto source = make_source();
  const auto source_fingerprint =
    mpcc_architecture_snapshot::fingerprint_interaction_snapshot(source);
  const auto immediate = build_lattice(source, source_fingerprint, 1, 0, 3);
  const auto delayed = build_lattice(source, source_fingerprint, 1, 2, 3);
  ASSERT_TRUE(immediate.seed.has_value()) << immediate.detail;
  ASSERT_TRUE(delayed.seed.has_value()) << delayed.detail;
  EXPECT_EQ(
    delayed.seed->solver_snapshot.dynamic_obstacle_forced_first_pass_side_stage,
    2);
  EXPECT_EQ(
    delayed.seed->solver_snapshot.dynamic_obstacle_forced_first_ahead_stage,
    3);
  EXPECT_GT(delayed.seed->lateral_reference_m[1], 0.0);
  EXPECT_LT(
    delayed.seed->lateral_reference_m[1],
    immediate.seed->lateral_reference_m[1]);
  EXPECT_DOUBLE_EQ(
    delayed.seed->lateral_reference_m[3],
    immediate.seed->lateral_reference_m[3]);
  EXPECT_NE(
    immediate.seed->candidate_fingerprint,
    delayed.seed->candidate_fingerprint);
}

TEST(MpccStatelessManeuver, SealsDiagonalScheduleWithoutMissionGeometry)
{
  const auto source = make_source();
  const auto source_fingerprint =
    mpcc_architecture_snapshot::fingerprint_interaction_snapshot(source);
  const auto early = build_diagonal_schedule(
    source, source_fingerprint, 1, 0, 2);
  const auto opposite = build_diagonal_schedule(
    source, source_fingerprint, -1, 0, 2);
  ASSERT_TRUE(early.seed.has_value()) << early.detail;
  ASSERT_TRUE(opposite.seed.has_value()) << opposite.detail;
  EXPECT_EQ(
    early.seed->solver_snapshot.dynamic_obstacle_forced_diagonal_start_stage,
    0);
  EXPECT_EQ(
    early.seed->solver_snapshot.
    dynamic_obstacle_forced_diagonal_full_side_stage, 2);
  EXPECT_EQ(
    early.seed->lateral_reference_m,
    build(source, source_fingerprint, 1).seed->lateral_reference_m);
  EXPECT_NE(
    early.seed->candidate_fingerprint,
    opposite.seed->candidate_fingerprint);
  EXPECT_NE(early.seed->candidate_fingerprint, source_fingerprint);
}

TEST(MpccStatelessManeuver, SealsPhysicalDiagonalReplayGeometry)
{
  const auto source = make_source();
  const auto source_fingerprint =
    mpcc_architecture_snapshot::fingerprint_interaction_snapshot(source);
  const auto normalized = build_diagonal_schedule(
    source, source_fingerprint, 1, 0, 2);
  const auto physical = build_physical_diagonal_schedule(
    source, source_fingerprint, 1, 0, 2);
  ASSERT_TRUE(normalized.seed.has_value()) << normalized.detail;
  ASSERT_TRUE(physical.seed.has_value()) << physical.detail;
  EXPECT_TRUE(
    physical.seed->solver_snapshot.
    dynamic_obstacle_forced_physical_diagonal);
  EXPECT_EQ(
    physical.seed->lateral_reference_m,
    normalized.seed->lateral_reference_m);
  EXPECT_NE(
    physical.seed->candidate_fingerprint,
    normalized.seed->candidate_fingerprint);
  EXPECT_NE(physical.seed->candidate_fingerprint, source_fingerprint);
}

TEST(MpccStatelessManeuver, StatelessSeedDeletesCapturedCandidateSchedule)
{
  auto source = make_source();
  source.dynamic_obstacle_forced_diagonal_start_stage = 0;
  source.dynamic_obstacle_forced_diagonal_full_side_stage = 2;
  source.dynamic_obstacle_forced_physical_diagonal = true;
  ASSERT_TRUE(
    mpcc_architecture_snapshot::interaction_snapshot_complete(source));
  const auto source_fingerprint =
    mpcc_architecture_snapshot::fingerprint_interaction_snapshot(source);

  const auto direct = build(source, source_fingerprint, -1);

  ASSERT_TRUE(direct.seed.has_value()) << direct.detail;
  const auto & rebuilt = direct.seed->solver_snapshot;
  EXPECT_EQ(
    rebuilt.dynamic_obstacle_longitudinal_topology,
    mpcc_rate_resolved_dynamic_obstacle::LongitudinalTopology::Automatic);
  EXPECT_EQ(rebuilt.dynamic_obstacle_forced_first_pass_side_stage, -1);
  EXPECT_EQ(rebuilt.dynamic_obstacle_forced_first_ahead_stage, -1);
  EXPECT_DOUBLE_EQ(rebuilt.dynamic_obstacle_forced_constraint_fraction, 1.0);
  EXPECT_EQ(rebuilt.dynamic_obstacle_forced_diagonal_start_stage, -1);
  EXPECT_EQ(rebuilt.dynamic_obstacle_forced_diagonal_full_side_stage, -1);
  EXPECT_FALSE(rebuilt.dynamic_obstacle_forced_physical_diagonal);
}

TEST(MpccStatelessManeuver, BuildsMidHorizonPhysicalDiagonalPopulation)
{
  const auto source = make_source();
  const auto result = build_bounded_candidates(source, -1);

  ASSERT_EQ(result.reason, RejectReason::Accepted) << result.detail;
  ASSERT_EQ(result.candidates.size(), 2U);
  EXPECT_EQ(result.candidates[0].kind, CandidateKind::DirectSide);
  EXPECT_EQ(
    result.candidates[1].kind,
    CandidateKind::MidPhysicalDiagonal);
  EXPECT_EQ(result.candidates[0].seed.pass_side_sign, -1);
  EXPECT_EQ(result.candidates[1].seed.pass_side_sign, -1);
  EXPECT_EQ(
    result.candidates[1].seed.solver_snapshot.
    dynamic_obstacle_forced_diagonal_start_stage, 0);
  EXPECT_EQ(
    result.candidates[1].seed.solver_snapshot.
    dynamic_obstacle_forced_diagonal_full_side_stage, 2);
  EXPECT_TRUE(
    result.candidates[1].seed.solver_snapshot.
    dynamic_obstacle_forced_physical_diagonal);
  EXPECT_NE(
    result.candidates[0].seed.candidate_fingerprint,
    result.candidates[1].seed.candidate_fingerprint);
}

TEST(MpccStatelessManeuver, AddsLateExactDisjunctionForLongHorizon)
{
  auto source = make_source();
  constexpr int horizon = 20;
  const auto state_template = source.request.states.back();
  const auto input_template = source.request.inputs.back();
  const auto obstacle_template = source.dynamic_obstacle_stages.back();
  source.request.horizon_steps = horizon;
  source.request.states.resize(horizon + 1, state_template);
  source.request.inputs.resize(horizon, input_template);
  source.nominal_path_distance_m.resize(horizon + 1);
  source.wall_reference_progress_m.resize(horizon + 1);
  source.wall_lower_m.resize(horizon + 1, -2.0);
  source.wall_upper_m.resize(horizon + 1, 2.0);
  source.dynamic_obstacle_stages.resize(horizon, obstacle_template);
  for (int stage = 0; stage <= horizon; ++stage) {
    const auto index = static_cast<std::size_t>(stage);
    source.request.states[index].reference(4) = static_cast<double>(stage);
    source.nominal_path_distance_m[index] = static_cast<double>(stage);
    source.wall_reference_progress_m[index] = static_cast<double>(stage);
    if (stage < horizon) {
      source.dynamic_obstacle_stages[index].target_progress_m =
        static_cast<double>(stage + 1);
    }
  }
  source.execution_prefix_steps = horizon;
  source.identity.source_context.horizon_steps = horizon;
  source.identity.source_context.fingerprint = 0U;
  source.identity.source_context =
    mpcc_execution_contract::seal_problem_context(
    source.identity.source_context);
  ASSERT_TRUE(
    mpcc_architecture_snapshot::interaction_snapshot_complete(source));

  const auto result = build_bounded_candidates(source, -1);

  ASSERT_EQ(result.reason, RejectReason::Accepted) << result.detail;
  ASSERT_EQ(result.candidates.size(), 4U) << result.detail;
  EXPECT_EQ(
    result.candidates[1].kind,
    CandidateKind::SteeringReachablePhysicalDiagonal);
  EXPECT_EQ(
    result.candidates[1].seed.solver_snapshot.
    dynamic_obstacle_forced_diagonal_start_stage, 0);
  EXPECT_EQ(
    result.candidates[1].seed.solver_snapshot.
    dynamic_obstacle_forced_diagonal_full_side_stage, 6);
  EXPECT_EQ(
    result.candidates[2].seed.solver_snapshot.
    dynamic_obstacle_forced_diagonal_start_stage, 0);
  EXPECT_EQ(
    result.candidates[2].seed.solver_snapshot.
    dynamic_obstacle_forced_diagonal_full_side_stage, 9);
  EXPECT_EQ(result.candidates[3].kind, CandidateKind::LateExactDisjunction);
  EXPECT_EQ(result.candidates[3].seed.pass_side_sign, -1);
  EXPECT_EQ(
    result.candidates[3].seed.solver_snapshot.
    dynamic_obstacle_forced_first_pass_side_stage, 17);
  EXPECT_EQ(
    result.candidates[3].seed.solver_snapshot.
    dynamic_obstacle_forced_first_ahead_stage, 20);
  EXPECT_DOUBLE_EQ(
    result.candidates[3].seed.solver_snapshot.
    dynamic_obstacle_forced_constraint_fraction, 1.0);
  EXPECT_EQ(
    result.candidates[3].seed.solver_snapshot.
    dynamic_obstacle_forced_diagonal_start_stage, -1);
  EXPECT_EQ(
    result.candidates[3].seed.solver_snapshot.
    dynamic_obstacle_forced_diagonal_full_side_stage, -1);
  EXPECT_FALSE(
    result.candidates[3].seed.solver_snapshot.
    dynamic_obstacle_forced_physical_diagonal);
  EXPECT_NE(
    result.candidates[2].seed.candidate_fingerprint,
    result.candidates[3].seed.candidate_fingerprint);
}

TEST(MpccStatelessManeuver, DeduplicatesReachableAndMidpointTopology)
{
  auto source = make_source();
  constexpr int horizon = 20;
  const auto state_template = source.request.states.back();
  const auto input_template = source.request.inputs.back();
  const auto obstacle_template = source.dynamic_obstacle_stages.back();
  source.request.horizon_steps = horizon;
  source.request.current_steering_rad = 0.3;
  source.request.states.resize(horizon + 1, state_template);
  source.request.inputs.resize(horizon, input_template);
  source.nominal_path_distance_m.resize(horizon + 1);
  source.wall_reference_progress_m.resize(horizon + 1);
  source.wall_lower_m.resize(horizon + 1, -2.0);
  source.wall_upper_m.resize(horizon + 1, 2.0);
  source.dynamic_obstacle_stages.resize(horizon, obstacle_template);
  for (int stage = 0; stage <= horizon; ++stage) {
    const auto index = static_cast<std::size_t>(stage);
    source.request.states[index].reference(4) = static_cast<double>(stage);
    source.nominal_path_distance_m[index] = static_cast<double>(stage);
    source.wall_reference_progress_m[index] = static_cast<double>(stage);
    if (stage < horizon) {
      source.dynamic_obstacle_stages[index].target_progress_m =
        static_cast<double>(stage + 1);
    }
  }
  source.execution_prefix_steps = horizon;
  source.identity.source_context.horizon_steps = horizon;
  source.identity.source_context.fingerprint = 0U;
  source.identity.source_context =
    mpcc_execution_contract::seal_problem_context(
    source.identity.source_context);
  ASSERT_TRUE(
    mpcc_architecture_snapshot::interaction_snapshot_complete(source));

  const auto result = build_bounded_candidates(source, -1);

  ASSERT_EQ(result.reason, RejectReason::Accepted) << result.detail;
  ASSERT_EQ(result.candidates.size(), 3U) << result.detail;
  EXPECT_EQ(
    result.candidates[1].kind,
    CandidateKind::SteeringReachablePhysicalDiagonal);
  EXPECT_EQ(
    result.candidates[1].seed.solver_snapshot.
    dynamic_obstacle_forced_diagonal_full_side_stage, 9);
  EXPECT_EQ(result.candidates[2].kind, CandidateKind::LateExactDisjunction);
}

TEST(MpccStatelessManeuver, UsesFiniteTargetTubeBoundaryAsThirdHomotopy)
{
  auto source = make_source();
  constexpr int horizon = 20;
  constexpr int last_valid_target_stage = 13;
  const auto state_template = source.request.states.back();
  const auto input_template = source.request.inputs.back();
  const auto obstacle_template = source.dynamic_obstacle_stages.back();
  source.request.horizon_steps = horizon;
  source.request.states.resize(horizon + 1, state_template);
  source.request.inputs.resize(horizon, input_template);
  source.nominal_path_distance_m.resize(horizon + 1);
  source.wall_reference_progress_m.resize(horizon + 1);
  source.wall_lower_m.resize(horizon + 1, -2.0);
  source.wall_upper_m.resize(horizon + 1, 2.0);
  source.dynamic_obstacle_stages.resize(horizon, obstacle_template);
  for (int stage = 0; stage <= horizon; ++stage) {
    const auto index = static_cast<std::size_t>(stage);
    source.request.states[index].reference(4) = static_cast<double>(stage);
    source.nominal_path_distance_m[index] = static_cast<double>(stage);
    source.wall_reference_progress_m[index] = static_cast<double>(stage);
    if (stage < horizon) {
      source.dynamic_obstacle_stages[index].valid =
        stage <= last_valid_target_stage;
      source.dynamic_obstacle_stages[index].target_progress_m =
        6.0 + 0.2 * static_cast<double>(stage);
      source.dynamic_obstacle_stages[index].longitudinal_overlap_m = 2.0;
    }
  }
  source.execution_prefix_steps = horizon;
  source.identity.source_context.horizon_steps = horizon;
  source.identity.source_context.fingerprint = 0U;
  source.identity.source_context =
    mpcc_execution_contract::seal_problem_context(
    source.identity.source_context);
  ASSERT_TRUE(
    mpcc_architecture_snapshot::interaction_snapshot_complete(source));
  const auto source_fingerprint =
    mpcc_architecture_snapshot::fingerprint_interaction_snapshot(source);
  const auto boundary_candidate = build_physical_diagonal_schedule(
    source, source_fingerprint, -1, 5, last_valid_target_stage + 1);
  ASSERT_TRUE(boundary_candidate.seed.has_value()) << boundary_candidate.detail;

  const auto result = build_bounded_candidates(source, -1);

  ASSERT_EQ(result.reason, RejectReason::Accepted) << result.detail;
  ASSERT_EQ(result.candidates.size(), 4U) << result.detail;
  EXPECT_EQ(
    result.candidates[1].kind,
    CandidateKind::SteeringReachablePhysicalDiagonal);
  EXPECT_EQ(
    result.candidates[3].kind,
    CandidateKind::EncounterBoundaryPhysicalDiagonal);
  EXPECT_EQ(
    result.candidates[3].seed.solver_snapshot.
    dynamic_obstacle_forced_diagonal_start_stage, 5);
  EXPECT_EQ(
    result.candidates[3].seed.solver_snapshot.
    dynamic_obstacle_forced_diagonal_full_side_stage,
    last_valid_target_stage + 1);
  EXPECT_TRUE(
    result.candidates[3].seed.solver_snapshot.
    dynamic_obstacle_forced_physical_diagonal);
  EXPECT_EQ(
    result.candidates[3].seed.solver_snapshot.
    dynamic_obstacle_forced_first_pass_side_stage, -1);
}

TEST(MpccStatelessManeuver, BoundsPopulationWhenDiagonalDoesNotFit)
{
  auto source = make_source();
  source.request.horizon_steps = 2;
  source.request.states.resize(3U);
  source.request.inputs.resize(2U);
  source.identity.source_context.horizon_steps = 2U;
  source.identity.source_context.fingerprint = 0U;
  source.identity.source_context = mpcc_execution_contract::seal_problem_context(
    source.identity.source_context);
  source.execution_prefix_steps = 2;
  source.nominal_path_distance_m.resize(3U);
  source.wall_reference_progress_m.resize(3U);
  source.wall_lower_m.resize(3U);
  source.wall_upper_m.resize(3U);
  // Keep the captured interaction model internally complete while shortening
  // the horizon.  With the first target stage at zero, the earliest physical
  // diagonal needs stage two and therefore cannot fit in a two-input horizon.
  source.dynamic_obstacle_stages.resize(2U);

  const auto result = build_bounded_candidates(source, 1);

  ASSERT_EQ(result.reason, RejectReason::Accepted) << result.detail;
  ASSERT_EQ(result.candidates.size(), 1U);
  EXPECT_EQ(result.candidates.front().kind, CandidateKind::DirectSide);
}

TEST(MpccStatelessManeuver, SealsContinuationWithoutChangingStatelessReference)
{
  const auto source = make_source();
  const auto source_fingerprint =
    mpcc_architecture_snapshot::fingerprint_interaction_snapshot(source);
  const auto witness = build_disjunction_schedule(
    source, source_fingerprint, 1, 0, 3, 0.0);
  const auto middle = build_disjunction_schedule(
    source, source_fingerprint, 1, 0, 3, 0.5);
  const auto exact = build_disjunction_schedule(
    source, source_fingerprint, 1, 0, 3, 1.0);
  ASSERT_TRUE(witness.seed.has_value()) << witness.detail;
  ASSERT_TRUE(middle.seed.has_value()) << middle.detail;
  ASSERT_TRUE(exact.seed.has_value()) << exact.detail;
  EXPECT_EQ(witness.seed->lateral_reference_m, exact.seed->lateral_reference_m);
  EXPECT_DOUBLE_EQ(
    witness.seed->solver_snapshot.
    dynamic_obstacle_forced_constraint_fraction, 0.0);
  EXPECT_DOUBLE_EQ(
    middle.seed->solver_snapshot.
    dynamic_obstacle_forced_constraint_fraction, 0.5);
  EXPECT_DOUBLE_EQ(
    exact.seed->solver_snapshot.
    dynamic_obstacle_forced_constraint_fraction, 1.0);
  EXPECT_NE(
    witness.seed->candidate_fingerprint,
    middle.seed->candidate_fingerprint);
  EXPECT_NE(
    middle.seed->candidate_fingerprint,
    exact.seed->candidate_fingerprint);
}

TEST(MpccStatelessManeuver, IgnoresPersistentMissionLateralAndHeadingReference)
{
  const auto source = make_source();
  auto changed_mission = source;
  for (auto & state : changed_mission.request.states) {
    state.reference[mpcc_rate_resolved::kLateralIndex] = -1.75;
    state.reference[mpcc_rate_resolved::kHeadingIndex] = -0.45;
  }
  const auto original = build(
    source, mpcc_architecture_snapshot::fingerprint_interaction_snapshot(source), 1);
  const auto changed = build(
    changed_mission,
    mpcc_architecture_snapshot::fingerprint_interaction_snapshot(changed_mission), 1);
  ASSERT_TRUE(original.seed.has_value()) << original.detail;
  ASSERT_TRUE(changed.seed.has_value()) << changed.detail;
  EXPECT_EQ(original.seed->lateral_reference_m, changed.seed->lateral_reference_m);
  EXPECT_EQ(
    original.seed->solver_snapshot.request.states[2].reference[
      mpcc_rate_resolved::kHeadingIndex], 0.0);
  EXPECT_EQ(
    changed.seed->solver_snapshot.request.states[2].reference[
      mpcc_rate_resolved::kHeadingIndex], 0.0);
}

TEST(MpccStatelessManeuver, RejectsMissingCanonicalCurrentEpochTargetTube)
{
  auto source = make_source();
  source.dynamic_obstacle_refinement_active = false;
  source.dynamic_obstacle_stages.clear();
  source.identity.source_context.dynamic_obstacle_constraint_active = false;
  source.identity.source_context.dynamic_obstacle_generation = 0U;
  source.identity.source_context.dynamic_obstacle_id.clear();
  source.identity.source_context.dynamic_obstacle_side_sign = 0;
  source.identity.source_context = mpcc_execution_contract::seal_problem_context(
    source.identity.source_context);
  const auto horizon = resolve_canonical_target_horizon(source);
  const auto result = build(
    source, mpcc_architecture_snapshot::fingerprint_interaction_snapshot(source), 1);

  EXPECT_FALSE(horizon.accepted);
  EXPECT_EQ(
    horizon.detail, "canonical current-epoch target tube unavailable");
  EXPECT_FALSE(result.seed.has_value());
  EXPECT_EQ(result.reason, RejectReason::DynamicTargetUnavailable);
}

TEST(MpccStatelessManeuver, RejectsCanonicalTubeWithMismatchedWorldGeneration)
{
  auto source = make_source();
  source.replay_world->observation_generation = 14U;

  const auto horizon = resolve_canonical_target_horizon(source);

  EXPECT_FALSE(horizon.accepted);
  EXPECT_EQ(
    horizon.detail, "canonical target identity does not match ReplayWorld");
}

TEST(MpccStatelessManeuver, CanonicalTubeIsNotReprojectedIntoWallWindow)
{
  auto source = make_source();
  const auto expected_stages = source.dynamic_obstacle_stages;
  // This deliberately makes the legacy constant-global-velocity projection
  // leave the finite course window almost immediately. The canonical tube is
  // already expressed in the current solver epoch and must remain unchanged.
  source.replay_world->obstacles.front().x_m = 3.9;
  source.replay_world->obstacles.front().velocity_x_mps = 120.0;
  source.replay_world->obstacles.front().velocity_y_mps = -80.0;
  source.wall_course_frame_knots = {
    {99.0, -1.0, 0.0, 0.0, 1},
    {104.0, 4.0, 0.0, 0.0, 2}};

  const auto horizon = resolve_canonical_target_horizon(source);
  const auto result = build(
    source, mpcc_architecture_snapshot::fingerprint_interaction_snapshot(source), 1);

  ASSERT_TRUE(horizon.accepted) << horizon.detail;
  ASSERT_TRUE(result.seed.has_value()) << result.detail;
  ASSERT_EQ(horizon.stages.size(), expected_stages.size());
  ASSERT_EQ(
    result.seed->solver_snapshot.dynamic_obstacle_stages.size(),
    expected_stages.size());
  for (std::size_t index = 0U; index < expected_stages.size(); ++index) {
    const auto & expected = expected_stages[index];
    const auto & resolved = horizon.stages[index];
    const auto & candidate =
      result.seed->solver_snapshot.dynamic_obstacle_stages[index];
    EXPECT_EQ(resolved.valid, expected.valid);
    EXPECT_DOUBLE_EQ(resolved.target_progress_m, expected.target_progress_m);
    EXPECT_DOUBLE_EQ(resolved.target_lateral_m, expected.target_lateral_m);
    EXPECT_DOUBLE_EQ(
      resolved.longitudinal_overlap_m, expected.longitudinal_overlap_m);
    EXPECT_DOUBLE_EQ(
      resolved.lateral_center_separation_m,
      expected.lateral_center_separation_m);
    EXPECT_DOUBLE_EQ(candidate.target_progress_m, expected.target_progress_m);
    EXPECT_DOUBLE_EQ(candidate.target_lateral_m, expected.target_lateral_m);
  }
}

TEST(MpccStatelessManeuver, BindsCurrentTargetWithoutChangingCapturedGeometry)
{
  auto source = make_source();
  const auto source_fingerprint =
    mpcc_architecture_snapshot::fingerprint_interaction_snapshot(source);

  const auto result = bind_current_world_target_preserving_geometry(
    source, source_fingerprint);

  ASSERT_TRUE(result.seed.has_value()) << result.detail;
  const auto & candidate = result.seed->solver_snapshot;
  EXPECT_EQ(candidate.identity.sequence, source.identity.sequence);
  EXPECT_EQ(
    candidate.identity.source_context.fingerprint,
    source.identity.source_context.fingerprint);
  EXPECT_TRUE(
    candidate.identity.source_context.dynamic_obstacle_constraint_active);
  ASSERT_EQ(candidate.request.states.size(), source.request.states.size());
  ASSERT_EQ(candidate.request.inputs.size(), source.request.inputs.size());
  for (std::size_t index = 0U; index < source.request.states.size(); ++index) {
    EXPECT_TRUE(candidate.request.states[index].reference.isApprox(
      source.request.states[index].reference, 0.0));
    EXPECT_TRUE(candidate.request.states[index].lower.isApprox(
      source.request.states[index].lower, 0.0));
    EXPECT_TRUE(candidate.request.states[index].upper.isApprox(
      source.request.states[index].upper, 0.0));
  }
  EXPECT_TRUE(candidate.dynamic_obstacle_refinement_active);
  EXPECT_EQ(
    candidate.dynamic_obstacle_pass_side_sign,
    source.identity.source_context.execution_side_sign);
  EXPECT_EQ(candidate.dynamic_obstacle_stages.size(), 3U);
  EXPECT_EQ(result.seed->candidate_fingerprint, source_fingerprint);
}

TEST(MpccStatelessManeuver, UsesCanonicalCurrentEpochTargetStages)
{
  const auto source = make_source();
  auto changed_mission = source;
  for (auto & stage : changed_mission.dynamic_obstacle_stages) {
    stage.target_progress_m += 20.0;
    stage.target_lateral_m -= 4.0;
    stage.longitudinal_overlap_m += 3.0;
    stage.lateral_center_separation_m += 2.0;
  }
  const auto original = build(
    source, mpcc_architecture_snapshot::fingerprint_interaction_snapshot(source), 1);
  const auto changed = build(
    changed_mission,
    mpcc_architecture_snapshot::fingerprint_interaction_snapshot(changed_mission), 1);
  ASSERT_TRUE(original.seed.has_value()) << original.detail;
  ASSERT_TRUE(changed.seed.has_value()) << changed.detail;
  EXPECT_NE(original.seed->lateral_reference_m, changed.seed->lateral_reference_m);
  const auto & original_stages =
    original.seed->solver_snapshot.dynamic_obstacle_stages;
  const auto & changed_stages =
    changed.seed->solver_snapshot.dynamic_obstacle_stages;
  ASSERT_EQ(original_stages.size(), changed_stages.size());
  for (std::size_t index = 0U; index < original_stages.size(); ++index) {
    EXPECT_EQ(original_stages[index].valid, changed_stages[index].valid);
    EXPECT_DOUBLE_EQ(
      changed_stages[index].target_progress_m,
      changed_mission.dynamic_obstacle_stages[index].target_progress_m);
    EXPECT_DOUBLE_EQ(
      changed_stages[index].target_lateral_m,
      changed_mission.dynamic_obstacle_stages[index].target_lateral_m);
    EXPECT_DOUBLE_EQ(
      changed_stages[index].longitudinal_overlap_m,
      changed_mission.dynamic_obstacle_stages[index].longitudinal_overlap_m);
    EXPECT_DOUBLE_EQ(
      changed_stages[index].lateral_center_separation_m,
      changed_mission.dynamic_obstacle_stages[index].
      lateral_center_separation_m);
  }
}

TEST(MpccStatelessManeuver, UsesReplanWithExplicitContingencyStop)
{
  auto source = make_source();
  const auto result = build(
    source, mpcc_architecture_snapshot::fingerprint_interaction_snapshot(source), 1);
  ASSERT_TRUE(result.seed.has_value()) << result.detail;
  EXPECT_EQ(result.seed->terminal_successor, TerminalSuccessor::Replan);
  EXPECT_TRUE(result.seed->stop_suffix.available);
  EXPECT_EQ(result.seed->stop_suffix.target_velocity_mps, 0.0);
  EXPECT_LT(result.seed->stop_suffix.maximum_deceleration_mps2, 0.0);
}

TEST(MpccStatelessManeuver, ReturnAllowsSemanticallyUnboundedLateralInterval)
{
  auto source = make_source();
  source.dynamic_obstacle_stages.back().target_progress_m = 2.0;
  source.request.states.back().lower[mpcc_rate_resolved::kLateralIndex] =
    -std::numeric_limits<double>::infinity();
  source.request.states.back().upper[mpcc_rate_resolved::kLateralIndex] =
    std::numeric_limits<double>::infinity();
  const auto result = build(
    source, mpcc_architecture_snapshot::fingerprint_interaction_snapshot(source), 1);
  ASSERT_TRUE(result.seed.has_value()) << result.detail;
  EXPECT_EQ(result.seed->terminal_successor, TerminalSuccessor::Return);
}

TEST(MpccStatelessManeuver, FollowAvoidanceKeepsNormalIntentAndIdentity)
{
  auto source = make_source();
  source.identity.source_context.intent =
    mpcc_execution_contract::ControlIntent::Follow;
  source.identity.source_context.execution_side_sign = 0;
  source.identity.source_context.fingerprint = 0U;
  source.identity.source_context = mpcc_execution_contract::seal_problem_context(
    source.identity.source_context);
  const auto fingerprint =
    mpcc_architecture_snapshot::fingerprint_interaction_snapshot(source);
  ASSERT_NE(fingerprint, 0U);

  const auto production = build(source, fingerprint, 1);
  EXPECT_EQ(production.reason, RejectReason::UnsupportedIntent);
  EXPECT_FALSE(production.seed.has_value());

  const auto avoidance = build_normal_avoidance(source, fingerprint, 1);
  EXPECT_EQ(avoidance.reason, RejectReason::Accepted) << avoidance.detail;
  ASSERT_TRUE(avoidance.seed.has_value());
  EXPECT_EQ(avoidance.seed->pass_side_sign, 1);
  EXPECT_EQ(
    avoidance.seed->solver_snapshot.identity.source_context.intent,
    mpcc_execution_contract::ControlIntent::Follow);

  const auto population = build_normal_avoidance_candidates(source);
  EXPECT_EQ(population.reason, RejectReason::Accepted) << population.detail;
  ASSERT_GE(population.candidates.size(), 2U);
  EXPECT_LE(population.candidates.size(), 8U);
  EXPECT_TRUE(std::any_of(
    population.candidates.begin(), population.candidates.end(),
    [](const Candidate & candidate) {
      return candidate.seed.pass_side_sign > 0 &&
             candidate.kind == CandidateKind::DirectSide;
    }));
  EXPECT_TRUE(std::any_of(
    population.candidates.begin(), population.candidates.end(),
    [](const Candidate & candidate) {
      return candidate.seed.pass_side_sign < 0 &&
             candidate.kind == CandidateKind::DirectSide;
    }));
  for (const auto & candidate : population.candidates) {
    EXPECT_EQ(
      candidate.seed.solver_snapshot.identity.source_context.intent,
      mpcc_execution_contract::ControlIntent::Follow);
    EXPECT_EQ(
      candidate.seed.solver_snapshot.identity.source_context.execution_side_sign,
      0);
  }
}

TEST(MpccStatelessManeuver, CruiseAvoidanceKeepsNormalIntentAndIdentity)
{
  auto source = make_source();
  source.identity.source_context.intent =
    mpcc_execution_contract::ControlIntent::Cruise;
  source.identity.source_context.execution_side_sign = 0;
  source.identity.source_context.fingerprint = 0U;
  source.identity.source_context = mpcc_execution_contract::seal_problem_context(
    source.identity.source_context);
  const auto fingerprint =
    mpcc_architecture_snapshot::fingerprint_interaction_snapshot(source);
  ASSERT_NE(fingerprint, 0U);

  const auto production = build(source, fingerprint, -1);
  EXPECT_EQ(production.reason, RejectReason::UnsupportedIntent);
  EXPECT_FALSE(production.seed.has_value());

  const auto avoidance = build_normal_avoidance(
    source, fingerprint, -1);
  EXPECT_EQ(avoidance.reason, RejectReason::Accepted) << avoidance.detail;
  ASSERT_TRUE(avoidance.seed.has_value());
  EXPECT_EQ(avoidance.seed->pass_side_sign, -1);
  EXPECT_EQ(
    avoidance.seed->solver_snapshot.identity.source_context.intent,
    mpcc_execution_contract::ControlIntent::Cruise);
  EXPECT_EQ(
    avoidance.seed->solver_snapshot.identity.source_context.execution_side_sign,
    0);
}

TEST(MpccStatelessManeuver, NormalAvoidancePopulationSupportsCruiseAndFollow)
{
  for (const auto intent : {
      mpcc_execution_contract::ControlIntent::Cruise,
      mpcc_execution_contract::ControlIntent::Follow})
  {
    auto source = make_source();
    source.identity.source_context.intent = intent;
    source.identity.source_context.execution_side_sign = 0;
    source.identity.source_context.fingerprint = 0U;
    source.identity.source_context =
      mpcc_execution_contract::seal_problem_context(
      source.identity.source_context);

    const auto population = build_normal_avoidance_candidates(source);

    EXPECT_EQ(population.reason, RejectReason::Accepted) << population.detail;
    ASSERT_GE(population.candidates.size(), 2U);
    EXPECT_LE(population.candidates.size(), 8U);
    std::size_t positive_count = 0U;
    std::size_t negative_count = 0U;
    for (const auto & candidate : population.candidates) {
      positive_count += candidate.seed.pass_side_sign > 0 ? 1U : 0U;
      negative_count += candidate.seed.pass_side_sign < 0 ? 1U : 0U;
      EXPECT_EQ(
        candidate.seed.solver_snapshot.identity.source_context.intent,
        intent);
      EXPECT_EQ(
        candidate.seed.solver_snapshot.identity.source_context.execution_side_sign,
        0);
    }
    EXPECT_GT(positive_count, 0U);
    EXPECT_GT(negative_count, 0U);
  }
}

TEST(MpccStatelessManeuver, NormalAvoidanceScheduleIsSealedAndDataOnly)
{
  auto source = make_source();
  source.identity.source_context.intent =
    mpcc_execution_contract::ControlIntent::Cruise;
  source.identity.source_context.execution_side_sign = 0;
  source.identity.source_context.fingerprint = 0U;
  source.identity.source_context =
    mpcc_execution_contract::seal_problem_context(
    source.identity.source_context);
  const auto fingerprint =
    mpcc_architecture_snapshot::fingerprint_interaction_snapshot(source);

  const auto lattice = build_normal_avoidance_schedule(
    source, fingerprint, 1, 0, source.request.horizon_steps);

  EXPECT_EQ(lattice.reason, RejectReason::Accepted) << lattice.detail;
  ASSERT_TRUE(lattice.seed.has_value());
  EXPECT_NE(lattice.seed->candidate_fingerprint, 0U);
  EXPECT_EQ(
    lattice.seed->solver_snapshot.identity.source_context.intent,
    mpcc_execution_contract::ControlIntent::Cruise);
  EXPECT_EQ(
    lattice.seed->solver_snapshot.identity.source_context.execution_side_sign,
    0);
  EXPECT_EQ(
    lattice.seed->solver_snapshot.dynamic_obstacle_forced_first_pass_side_stage,
    0);
  EXPECT_EQ(
    lattice.seed->solver_snapshot.dynamic_obstacle_forced_first_ahead_stage,
    source.request.horizon_steps);
}

TEST(MpccStatelessManeuver, RejectsInvalidSideAndMixedObservation)
{
  auto source = make_source();
  const auto fingerprint =
    mpcc_architecture_snapshot::fingerprint_interaction_snapshot(source);
  EXPECT_EQ(build(source, fingerprint, 0).reason, RejectReason::InvalidSide);
  EXPECT_EQ(
    build(source, fingerprint + 1U, 1).reason,
    RejectReason::SourceFingerprintMismatch);
  source.replay_world->obstacles.front().observation_generation = 12U;
  EXPECT_EQ(
    build(source, fingerprint, 1).reason, RejectReason::IncompleteSnapshot);
}

TEST(MpccStatelessManeuver, RejectsWhenNoTerminalSuccessorExists)
{
  auto source = make_source();
  source.dynamic_obstacle_stages.back().target_progress_m = 3.0;
  for (auto & input : source.request.inputs) {
    input.lower[mpcc_rate_resolved::kAccelerationIndex] = 0.0;
  }
  const auto result = build(
    source, mpcc_architecture_snapshot::fingerprint_interaction_snapshot(source), 1);
  EXPECT_EQ(result.reason, RejectReason::TerminalSuccessorUnavailable);
  EXPECT_FALSE(result.seed.has_value());
}

}  // namespace
}  // namespace multi_purpose_mpc_ros::mpcc_stateless_maneuver
