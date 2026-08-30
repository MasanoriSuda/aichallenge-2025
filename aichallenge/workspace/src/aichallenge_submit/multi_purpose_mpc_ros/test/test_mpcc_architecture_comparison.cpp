#include "multi_purpose_mpc_ros/mpcc_architecture_comparison.hpp"

#include "multi_purpose_mpc_ros/mpcc_rate_resolved.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_adapter.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_problem.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_stop_control_lattice.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_stop_lattice_shadow.hpp"
#include "multi_purpose_mpc_ros/persistent_osqp.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

namespace multi_purpose_mpc_ros::mpcc_architecture_comparison
{
namespace
{

namespace architecture = mpcc_architecture_snapshot;
namespace contract = mpcc_execution_contract;
namespace model = mpcc_rate_resolved;
namespace recovery = recovery_footprint;
namespace shadow = mpcc_rate_resolved_shadow;
namespace stop_lattice = mpcc_rate_resolved_stop_control_lattice;
namespace stop_lattice_shadow = mpcc_rate_resolved_stop_lattice_shadow;

shadow::Snapshot source_snapshot()
{
  shadow::Snapshot source;
  source.identity.sequence = 9U;
  source.identity.snapshot_sec = 20.0;
  auto & context = source.identity.source_context;
  context.decision_id = 7U;
  context.intent = contract::ControlIntent::ShiftOut;
  context.intent_generation = 4U;
  context.observation_generation = 11U;
  context.stage_geometry_id = 12U;
  context.target_obstacle_generation = 13U;
  context.target_id = "d2";
  context.execution_side_sign = 1;
  context.dynamic_obstacle_constraint_active = true;
  context.dynamic_obstacle_generation = 13U;
  context.dynamic_obstacle_id = "d2";
  context.dynamic_obstacle_side_sign = 1;
  context.horizon_steps = 3U;
  context.formulation =
    contract::Formulation::VelocitySteeringYawResponseProgress7State;
  context.state_schema_id = "state-7";
  context.input_schema_id = "input-3";
  context.bounds_schema_id = "bounds";
  context.cost_schema_id = "cost";
  context = contract::seal_problem_context(context);
  source.control_prediction_origin_sec = 20.1;
  source.execution_prefix_steps = 3;
  source.course_progress_origin_m = 100.0;
  source.nominal_path_distance_m = {0.0, 0.4, 0.8, 1.2};
  source.publication_interval_sec = 0.025;

  auto & request = source.request;
  request.horizon_steps = 3;
  request.initial_state << 0.0, 0.0, 0.0, 2.0, 0.0;
  request.current_steering_rad = 0.0;
  request.current_response_steering_rad = 0.0;
  request.wheelbase_m = 1.0;
  request.yaw_response_gain = 1.0;
  request.yaw_response_time_constant_sec = 0.1;
  request.maximum_abs_steering_rad = 0.5;
  request.maximum_abs_steering_rate_radps = 1.0;
  request.previous_input << 0.0, 0.0, 2.0;
  request.input_delta_weight << 0.1, 0.1, 0.1;
  request.states.resize(4U);
  for (std::size_t index = 0U; index < request.states.size(); ++index) {
    auto & state = request.states[index];
    state.reference << 0.0, 0.0, 0.0, 2.0, 0.4 * index;
    state.lower << -2.0, -1.0, -0.6, 0.0, 0.0;
    state.upper << 2.0, 1.0, 0.6, 5.0, 3.0;
    state.weight << 2.0, 1.0, 2.0, 1.0, 1.0;
  }
  request.states.front().lower = request.initial_state;
  request.states.front().upper = request.initial_state;
  request.inputs.resize(3U);
  for (auto & input : request.inputs) {
    input.reference << 0.0, 0.0, 2.0;
    input.lower << -2.0, -0.5, 0.0;
    input.upper << 1.0, 0.5, 5.0;
    input.weight << 1.0, 1.0, 1.0;
    input.path_curvature_radpm = 0.0;
    input.stage_dt_sec = 0.1;
  }
  source.progress_aligned_wall_refinement_active = true;
  source.wall_reference_progress_m = {0.0, 0.4, 0.8, 1.2};
  source.wall_lower_m = {-2.0, -2.0, -2.0, -2.0};
  source.wall_upper_m = {2.0, 2.0, 2.0, 2.0};
  source.dynamic_obstacle_refinement_active = true;
  source.dynamic_obstacle_pass_side_sign = 1;
  source.dynamic_obstacle_stages = {
    {true, 3.0, 5.0, 0.15, 0.8},
    {true, 3.0, 5.0, 0.15, 0.8},
    {true, 3.0, 5.0, 0.15, 0.8}};
  source.physical_wall_refinement_active = true;
  auto grid = std::make_shared<recovery::OccupancyGrid>();
  grid->width = 400U;
  grid->height = 200U;
  grid->resolution_m = 0.1;
  grid->origin_x_m = -10.0;
  grid->origin_y_m = -10.0;
  grid->y_axis = recovery::YAxisConvention::RowZeroAtMinimumY;
  grid->cells.assign(grid->width * grid->height, recovery::CellState::Free);
  source.wall_grid = grid;
  source.wall_footprint = {1.0, 1.0, 0.725, 0.725, 0.0};
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
  world.control_prefix = {{0.0, 0.0, 0.0}, {0.2, 0.0, 0.0}};
  world.control_prefix_elapsed_sec = {0.0, 0.1};
  world.physical_footprint = {1.0, 1.0, 0.525, 0.525, 0.0};
  world.wall_grid_fingerprint = recovery::occupancy_grid_fingerprint(*grid);
  world.hard_wall_clearance_m = 0.2;
  world.bound_tolerance_m = 1e-5;
  world.swept_step_m = 0.1;
  world.terminal_stop_contract_available = true;
  world.terminal_stop_lateral_policy =
    race_mpcc_foundation::StopPathTrackingPolicy{
    source.request.wheelbase_m, source.request.maximum_abs_steering_rad,
    source.request.maximum_abs_steering_rate_radps, 6.0, 1.0, 0.8, 1.2};
  world.terminal_stop_minimum_acceleration_mps2 = -2.0;
  // The replay obstacle is deliberately clear in this infrastructure test;
  // its identity still seals the same target/world used by every arm.
  world.obstacles.push_back(
    shadow::ReplayDynamicObstacle{
      "d2", 3.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.01, 0.2, 13U});
  EXPECT_TRUE(architecture::interaction_snapshot_complete(source));
  return source;
}

shadow::Snapshot stoppable_source_snapshot()
{
  auto source = source_snapshot();
  constexpr int horizon = 12;
  source.identity.source_context.horizon_steps = horizon;
  source.identity.source_context = contract::seal_problem_context(
    source.identity.source_context);
  source.execution_prefix_steps = 3;
  source.request.horizon_steps = horizon;
  const auto state_template = source.request.states[1];
  const auto input_template = source.request.inputs.front();
  source.request.states.assign(horizon + 1U, state_template);
  source.request.inputs.assign(horizon, input_template);
  source.nominal_path_distance_m.resize(horizon + 1U);
  source.wall_reference_progress_m.resize(horizon + 1U);
  source.wall_lower_m.assign(horizon + 1U, -2.0);
  source.wall_upper_m.assign(horizon + 1U, 2.0);
  for (int stage = 0; stage <= horizon; ++stage) {
    auto & state = source.request.states[stage];
    state.reference << 0.0, 0.0, 0.0, 2.0, 0.2 * stage;
    state.lower << -2.0, -1.0, -0.6, 0.0, 0.0;
    state.upper << 2.0, 1.0, 0.6, 5.0, 5.0;
    state.weight << 2.0, 1.0, 2.0, 1.0, 1.0;
    source.nominal_path_distance_m[stage] = 0.2 * stage;
    source.wall_reference_progress_m[stage] = 0.2 * stage;
  }
  source.request.states.front().lower = source.request.initial_state;
  source.request.states.front().upper = source.request.initial_state;
  source.dynamic_obstacle_stages.assign(
    horizon, source.dynamic_obstacle_stages.front());
  EXPECT_TRUE(architecture::interaction_snapshot_complete(source));
  return source;
}

shadow::Snapshot return_source_snapshot()
{
  auto source = source_snapshot();
  source.identity.source_context.intent = contract::ControlIntent::Return;
  source.identity.source_context.fingerprint = 0U;
  source.identity.source_context = contract::seal_problem_context(
    source.identity.source_context);
  source.request.initial_state[model::kLateralIndex] = 1.0;
  source.request.states.front().lower = source.request.initial_state;
  source.request.states.front().upper = source.request.initial_state;
  for (std::size_t index = 1U; index < source.request.states.size(); ++index) {
    source.request.states[index].reference[model::kLateralIndex] = 1.25;
  }
  source.terminal_intent_contract.active = true;
  source.terminal_intent_contract.lateral_reference_m = 0.0;
  source.terminal_intent_contract.lateral_tolerance_m = 0.20;
  source.terminal_intent_contract.heading_reference_rad = 0.0;
  source.terminal_intent_contract.heading_tolerance_rad = 0.12;
  EXPECT_TRUE(architecture::interaction_snapshot_complete(source));
  return source;
}

architecture::RecordedInteractionSnapshot recorded(
  shadow::Snapshot source)
{
  architecture::RecordedInteractionSnapshot value;
  value.source = std::move(source);
  value.interaction_fingerprint =
    architecture::fingerprint_interaction_snapshot(value.source);
  return value;
}

std::pair<architecture::RecordedInteractionSnapshot, Eigen::VectorXd>
recorded_with_solved_qp(shadow::Snapshot source)
{
  persistent_osqp::PersistentOsqpSolver solver{
    persistent_osqp::ConstraintPreconditioningPolicy::RowToleranceNormalized};
  const auto adapted = mpcc_rate_resolved_adapter::build(
    source.request, solver.physical_constraint_tolerance());
  EXPECT_TRUE(adapted.has_value());
  const auto qp = adapted.has_value() ?
    mpcc_rate_resolved_problem::assemble(adapted->problem) : std::nullopt;
  EXPECT_TRUE(qp.has_value());
  const auto outcome = qp.has_value() ? solver.solve(
    qp->quadratic_cost, qp->constraints, qp->linear_cost,
    qp->lower_bound, qp->upper_bound, std::nullopt, qp->variable_scaling) :
    persistent_osqp::SolveOutcome{};
  EXPECT_TRUE(outcome.result.has_value()) << outcome.failure_detail;

  auto value = recorded(std::move(source));
  if (qp.has_value()) {
    value.recorded_qp.emplace();
    value.recorded_qp->problem = qp.value();
  }
  return {
    std::move(value), outcome.result.has_value() ?
    outcome.result->primal : Eigen::VectorXd{}};
}

TEST(MpccArchitectureComparison, IndependentlyProducesSealedBundles)
{
  const auto report = compare(recorded(source_snapshot()));
  ASSERT_TRUE(report.source_accepted) << report.detail;
  // A + A2 + two B arms + C and D over two homotopies and every valid pair,
  // plus one diagonal E, one physical-diagonal F and one bounded production G
  // result per homotopy for N=3, followed by one audit-only wall-restoration
  // arm over the immutable persistent source.
  ASSERT_EQ(report.arms.size(), 35U);
  EXPECT_EQ(report.arms[0].arm, Arm::PersistentA);
  EXPECT_EQ(report.arms[1].arm, Arm::PersistentTargetBoundA2);
  EXPECT_EQ(report.arms[2].arm, Arm::StatelessLeftB);
  EXPECT_EQ(report.arms[3].arm, Arm::StatelessRightB);
  for (std::size_t index = 0U; index < 4U; ++index) {
    const auto & arm = report.arms[index];
    EXPECT_EQ(arm.source_interaction_fingerprint,
      report.source_interaction_fingerprint);
    ASSERT_EQ(arm.stage, Stage::Accepted) << arm.detail;
    ASSERT_TRUE(arm.bundle.has_value());
    EXPECT_EQ(arm.bundle->source_interaction_fingerprint,
      report.source_interaction_fingerprint);
    EXPECT_EQ(arm.bundle->candidate_fingerprint, arm.candidate_fingerprint);
    EXPECT_EQ(
      arm.bundle->wall_certificate.outcome,
      mpcc_rate_resolved_physical_wall::Outcome::Accepted);
    EXPECT_TRUE(arm.bundle->dynamic_certificate.clear);
  }
  for (std::size_t index = 4U; index < 16U; ++index) {
    const auto & arm = report.arms[index];
    EXPECT_TRUE(arm.arm == Arm::RoughLeftC || arm.arm == Arm::RoughRightC);
    EXPECT_GE(arm.lattice_transition_stage, 0);
    EXPECT_GT(arm.lattice_ahead_stage, arm.lattice_transition_stage);
    EXPECT_LE(arm.lattice_ahead_stage, 3);
    EXPECT_EQ(
      arm.source_interaction_fingerprint,
      report.source_interaction_fingerprint);
  }
  for (std::size_t index = 16U; index < 28U; ++index) {
    const auto & arm = report.arms[index];
    EXPECT_TRUE(arm.arm == Arm::OfflineLeftD || arm.arm == Arm::OfflineRightD);
    EXPECT_GE(arm.lattice_transition_stage, 0);
    EXPECT_GT(arm.lattice_ahead_stage, arm.lattice_transition_stage);
    EXPECT_TRUE(arm.direct_final_attempted);
    EXPECT_EQ(
      arm.source_interaction_fingerprint,
      report.source_interaction_fingerprint);
    if (arm.continuation_attempted) {
      EXPECT_LE(arm.continuation_solved_step_count, 4U);
      EXPECT_GE(arm.continuation_compute_ms, 0.0);
    }
  }
  for (std::size_t index = 28U; index < 30U; ++index) {
    const auto & arm = report.arms[index];
    EXPECT_TRUE(
      arm.arm == Arm::DiagonalLeftE || arm.arm == Arm::DiagonalRightE);
    EXPECT_EQ(arm.lattice_transition_stage, 0);
    EXPECT_EQ(arm.lattice_ahead_stage, 2);
  }
  for (std::size_t index = 30U; index < 32U; ++index) {
    const auto & arm = report.arms[index];
    EXPECT_TRUE(
      arm.arm == Arm::PhysicalDiagonalLeftF ||
      arm.arm == Arm::PhysicalDiagonalRightF);
    EXPECT_EQ(arm.lattice_transition_stage, 0);
    EXPECT_EQ(arm.lattice_ahead_stage, 2);
  }
  for (std::size_t index = 32U; index < 34U; ++index) {
    const auto & arm = report.arms[index];
    EXPECT_TRUE(
      arm.arm == Arm::ProductionLeftG ||
      arm.arm == Arm::ProductionRightG);
    EXPECT_EQ(arm.stage, Stage::Accepted) << arm.detail;
    EXPECT_EQ(arm.candidate_source, "direct-side");
    EXPECT_EQ(arm.candidate_count, 1U);
  }
  const auto & restoration = report.arms[34U];
  EXPECT_EQ(restoration.arm, Arm::WallRestorationH);
  EXPECT_EQ(restoration.stage, Stage::Accepted) << restoration.detail;
  EXPECT_NE(restoration.detail.find("wall_restoration=not-needed"),
    std::string::npos);
  EXPECT_NE(
    report.arms[2].candidate_fingerprint,
    report.arms[3].candidate_fingerprint);
}

TEST(MpccArchitectureComparison, ReturnUsesRejoinSchedulesForCAndD)
{
  const auto report = compare(recorded(return_source_snapshot()));

  ASSERT_TRUE(report.source_accepted) << report.detail;
  ASSERT_EQ(report.arms.size(), 15U);
  EXPECT_EQ(report.arms[0].arm, Arm::PersistentA);
  EXPECT_EQ(report.arms[1].arm, Arm::PersistentTargetBoundA2);
  EXPECT_EQ(report.arms[2].arm, Arm::StatelessReturnB);
  std::size_t rough_count = 0U;
  std::size_t offline_count = 0U;
  for (const auto & arm : report.arms) {
    if (arm.arm == Arm::RoughReturnC) {
      ++rough_count;
      EXPECT_EQ(arm.candidate_source, "return-rejoin-polynomial");
      EXPECT_EQ(arm.detail.find("unsupported-intent"), std::string::npos);
    }
    if (arm.arm == Arm::OfflineReturnD) {
      ++offline_count;
      EXPECT_EQ(arm.candidate_source, "return-rejoin-polynomial");
      EXPECT_TRUE(arm.direct_final_attempted);
      EXPECT_EQ(arm.detail.find("unsupported-intent"), std::string::npos);
    }
  }
  EXPECT_EQ(rough_count, 5U);
  EXPECT_EQ(offline_count, 5U);
  EXPECT_EQ(report.arms[13].arm, Arm::ProductionReturnG);
  EXPECT_EQ(report.arms[14].arm, Arm::WallRestorationH);
}

TEST(
  MpccArchitectureComparison,
  PhysicalDynamicSqpAuditRemainsASeparateObservationArm)
{
  const auto report = compare_physical_dynamic_sqp(
    recorded(source_snapshot()));

  ASSERT_TRUE(report.source_accepted) << report.detail;
  ASSERT_EQ(report.arms.size(), 6U);
  EXPECT_EQ(report.arms[0].arm, Arm::PersistentA);
  EXPECT_EQ(report.arms[1].arm, Arm::DynamicSqpPersistentL);
  EXPECT_EQ(report.arms[2].arm, Arm::ProductionLeftG);
  EXPECT_EQ(report.arms[3].arm, Arm::DynamicSqpProductionLeftL);
  EXPECT_EQ(report.arms[4].arm, Arm::ProductionRightG);
  EXPECT_EQ(report.arms[5].arm, Arm::DynamicSqpProductionRightL);
  EXPECT_NE(
    report.arms[1].detail.find("physical_dynamic_sqp_audit="),
    std::string::npos);
}

TEST(
  MpccArchitectureComparison,
  ProofGuidedSqpRetainsCertifiedDepthZeroCandidate)
{
  const auto report = compare_proof_guided_dynamic_sqp(
    recorded(source_snapshot()));

  ASSERT_TRUE(report.source_accepted) << report.detail;
  ASSERT_EQ(report.arms.size(), 4U);
  EXPECT_EQ(report.arms[0].arm, Arm::ProductionLeftG);
  EXPECT_EQ(report.arms[1].arm, Arm::ProofGuidedProductionLeftM);
  EXPECT_EQ(report.arms[2].arm, Arm::ProductionRightG);
  EXPECT_EQ(report.arms[3].arm, Arm::ProofGuidedProductionRightM);
  ASSERT_TRUE(report.arms[1].bundle.has_value()) << report.arms[1].detail;
  ASSERT_TRUE(report.arms[3].bundle.has_value()) << report.arms[3].detail;
  EXPECT_EQ(report.arms[1].dynamic_sqp_depth, 0U);
  EXPECT_EQ(report.arms[3].dynamic_sqp_depth, 0U);
  EXPECT_NE(
    report.arms[1].detail.find("proof-guided-depth=0"),
    std::string::npos);
  EXPECT_NE(
    report.arms[3].detail.find("proof-guided-depth=0"),
    std::string::npos);
}

TEST(
  MpccArchitectureComparison,
  PersistentTargetBoundArmRestoresOnlyCurrentWorldTargetConstraint)
{
  auto source = source_snapshot();
  source.dynamic_obstacle_refinement_active = false;
  source.dynamic_obstacle_pass_side_sign = 0;
  source.dynamic_obstacle_stages.clear();
  source.identity.source_context.dynamic_obstacle_constraint_active = false;
  source.identity.source_context.dynamic_obstacle_generation = 0U;
  source.identity.source_context.dynamic_obstacle_id.clear();
  source.identity.source_context.dynamic_obstacle_side_sign = 0;
  source.identity.source_context = contract::seal_problem_context(
    source.identity.source_context);
  const auto source_fingerprint =
    architecture::fingerprint_interaction_snapshot(source);

  const auto report = compare(recorded(std::move(source)));

  ASSERT_TRUE(report.source_accepted) << report.detail;
  ASSERT_GE(report.arms.size(), 2U);
  const auto & target_bound = report.arms[1];
  EXPECT_EQ(target_bound.arm, Arm::PersistentTargetBoundA2);
  EXPECT_NE(target_bound.candidate_fingerprint, source_fingerprint);
  EXPECT_EQ(target_bound.stage, Stage::Accepted) << target_bound.detail;
  ASSERT_TRUE(target_bound.bundle.has_value());
  EXPECT_EQ(
    target_bound.bundle->pass_side_sign,
    source_snapshot().identity.source_context.execution_side_sign);
}

TEST(MpccArchitectureComparison, PhysicalDynamicRejectionCannotCreateBundle)
{
  auto source = source_snapshot();
  // The selected target remains feasible and clear to the QP.  A non-target
  // intruder intersects the measured-to-control prefix, which only the common
  // all-obstacle physical proof owns.
  source.replay_world->obstacles.push_back(
    shadow::ReplayDynamicObstacle{
      "d3", 1.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.01, 0.2, 13U});
  const auto report = compare(recorded(std::move(source)));
  ASSERT_TRUE(report.source_accepted);
  ASSERT_GE(report.arms.size(), 4U);
  for (std::size_t index = 0U; index < 4U; ++index) {
    const auto & arm = report.arms[index];
    EXPECT_EQ(arm.stage, Stage::DynamicProofRejected) << arm.detail;
    EXPECT_FALSE(arm.bundle.has_value());
  }
}

TEST(MpccArchitectureComparison, MissingSuccessorRejectsBeforeAuthorityData)
{
  auto source = source_snapshot();
  source.dynamic_obstacle_stages.back().target_progress_m = 1.2;
  for (auto & input : source.request.inputs) {
    input.lower[model::kAccelerationIndex] = 0.0;
  }
  const auto report = compare(recorded(std::move(source)));
  ASSERT_TRUE(report.source_accepted);
  for (const auto & arm : report.arms) {
    EXPECT_EQ(arm.stage, Stage::TerminalSuccessorRejected) << arm.detail;
    EXPECT_FALSE(arm.bundle.has_value());
  }
}

TEST(MpccArchitectureComparison, MissingImmutableStopContractRejectsAllArms)
{
  auto source = source_snapshot();
  source.replay_world->terminal_stop_contract_available = false;
  source.replay_world->terminal_stop_minimum_acceleration_mps2 =
    std::numeric_limits<double>::quiet_NaN();
  const auto report = compare(recorded(std::move(source)));
  ASSERT_TRUE(report.source_accepted) << report.detail;
  ASSERT_GE(report.arms.size(), 4U);
  for (std::size_t index = 0U; index < 4U; ++index) {
    const auto & arm = report.arms[index];
    EXPECT_EQ(arm.stage, Stage::TerminalSuccessorRejected) << arm.detail;
    EXPECT_FALSE(arm.bundle.has_value());
  }
  for (const auto & arm : report.arms) {
    EXPECT_FALSE(arm.bundle.has_value());
  }
}

TEST(MpccArchitectureComparison, FollowComparesOnlyPersistentAndStatelessSides)
{
  auto source = source_snapshot();
  source.identity.source_context.intent = contract::ControlIntent::Follow;
  source.identity.source_context.execution_side_sign = 0;
  source.identity.source_context.fingerprint = 0U;
  source.identity.source_context = contract::seal_problem_context(
    source.identity.source_context);

  const auto report = compare(recorded(std::move(source)));

  ASSERT_TRUE(report.source_accepted) << report.detail;
  ASSERT_EQ(report.arms.size(), 3U);
  EXPECT_EQ(report.arms[0].arm, Arm::PersistentA);
  EXPECT_EQ(report.arms[1].arm, Arm::StatelessLeftB);
  EXPECT_EQ(report.arms[2].arm, Arm::StatelessRightB);
  EXPECT_NE(report.arms[1].stage, Stage::CandidateRejected) << report.arms[1].detail;
  EXPECT_NE(report.arms[2].stage, Stage::CandidateRejected) << report.arms[2].detail;
  EXPECT_NE(report.arms[1].candidate_fingerprint, 0U);
  EXPECT_NE(report.arms[2].candidate_fingerprint, 0U);
}

TEST(MpccArchitectureComparison, CruiseComparesCapturedBranchAndCurrentWorldSides)
{
  auto source = source_snapshot();
  source.identity.source_context.intent = contract::ControlIntent::Cruise;
  source.identity.source_context.execution_side_sign = 0;
  source.identity.source_context.fingerprint = 0U;
  source.identity.source_context = contract::seal_problem_context(
    source.identity.source_context);

  const auto report = compare(recorded(std::move(source)));

  ASSERT_TRUE(report.source_accepted) << report.detail;
  ASSERT_EQ(report.arms.size(), 3U);
  EXPECT_EQ(report.arms[0].arm, Arm::PersistentA);
  EXPECT_EQ(report.arms[1].arm, Arm::StatelessLeftB);
  EXPECT_EQ(report.arms[2].arm, Arm::StatelessRightB);
  EXPECT_NE(report.arms[1].stage, Stage::CandidateRejected) << report.arms[1].detail;
  EXPECT_NE(report.arms[2].stage, Stage::CandidateRejected) << report.arms[2].detail;
  EXPECT_NE(report.arms[1].candidate_fingerprint, 0U);
  EXPECT_NE(report.arms[2].candidate_fingerprint, 0U);
}

TEST(MpccArchitectureComparison, FingerprintMismatchRejectsEveryArm)
{
  auto input = recorded(source_snapshot());
  ++input.interaction_fingerprint;
  const auto report = compare(input);
  EXPECT_FALSE(report.source_accepted);
  ASSERT_EQ(report.arms.size(), 15U);
  for (const auto & arm : report.arms) {
    EXPECT_EQ(arm.stage, Stage::SourceRejected);
    EXPECT_FALSE(arm.bundle.has_value());
  }
}

TEST(MpccArchitectureComparison, WallRestorationOnlyDoesNotEnumerateOtherArms)
{
  const auto report = compare_wall_restoration(recorded(source_snapshot()));
  ASSERT_TRUE(report.source_accepted) << report.detail;
  ASSERT_EQ(report.arms.size(), 1U);
  EXPECT_EQ(report.arms.front().arm, Arm::WallRestorationH);
  EXPECT_EQ(report.arms.front().stage, Stage::Accepted) <<
    report.arms.front().detail;
  EXPECT_TRUE(report.arms.front().bundle.has_value());
  EXPECT_NE(report.arms.front().detail.find("wall_restoration=not-needed"),
    std::string::npos);
}

TEST(MpccArchitectureComparison, WallRestorationOnlyRejectsChangedWorld)
{
  auto input = recorded(source_snapshot());
  ++input.interaction_fingerprint;
  const auto report = compare_wall_restoration(input);
  EXPECT_FALSE(report.source_accepted);
  ASSERT_EQ(report.arms.size(), 1U);
  EXPECT_EQ(report.arms.front().arm, Arm::WallRestorationH);
  EXPECT_EQ(report.arms.front().stage, Stage::SourceRejected);
  EXPECT_FALSE(report.arms.front().bundle.has_value());
}

TEST(MpccArchitectureComparison, WallBucketAuditKeepsExactProofChain)
{
  const auto report = compare_wall_buckets(recorded(source_snapshot()));
  ASSERT_TRUE(report.source_accepted) << report.detail;
  ASSERT_EQ(report.arms.size(), 5U);
  EXPECT_EQ(report.arms[0].arm, Arm::WallOmitHeadingJ);
  EXPECT_EQ(report.arms[1].arm, Arm::WallOmitLagK);
  EXPECT_EQ(report.arms[2].arm, Arm::WallOmitPoseN);
  EXPECT_EQ(report.arms[3].arm, Arm::WallOmitPoseDirectO);
  EXPECT_EQ(report.arms[4].arm, Arm::WallProductionP);
  for (std::size_t index = 0; index < report.arms.size(); ++index) {
    const auto & arm = report.arms[index];
    EXPECT_EQ(arm.stage, Stage::Accepted) << arm.detail;
    ASSERT_TRUE(arm.bundle.has_value());
    EXPECT_EQ(
      arm.bundle->wall_certificate.outcome,
      mpcc_rate_resolved_physical_wall::Outcome::Accepted);
    EXPECT_TRUE(arm.bundle->dynamic_certificate.clear);
    if (index < 4U) {
      EXPECT_NE(arm.detail.find("wall-bucket-audit=omit-"), std::string::npos);
    } else {
      EXPECT_EQ(arm.detail.find("wall-bucket-audit=omit-"), std::string::npos);
    }
  }
}

TEST(
  MpccArchitectureComparison,
  TargetFreeCruiseReachesSolverAndExactProofInsteadOfTargetGate)
{
  auto source = source_snapshot();
  auto & context = source.identity.source_context;
  context.intent = contract::ControlIntent::Cruise;
  context.target_id.clear();
  context.target_obstacle_generation = 0U;
  context.execution_side_sign = 0;
  context.dynamic_obstacle_constraint_active = false;
  context.dynamic_obstacle_generation = 0U;
  context.dynamic_obstacle_id.clear();
  context.dynamic_obstacle_side_sign = 0;
  context.fingerprint = 0U;
  context = contract::seal_problem_context(context);
  source.dynamic_obstacle_refinement_active = false;
  source.dynamic_obstacle_pass_side_sign = 0;
  source.dynamic_obstacle_stages.clear();

  const auto report = compare_wall_buckets(recorded(std::move(source)));

  ASSERT_TRUE(report.source_accepted) << report.detail;
  ASSERT_EQ(report.arms.size(), 5U);
  for (const auto & arm : report.arms) {
    EXPECT_NE(arm.stage, Stage::TerminalSuccessorRejected) << arm.detail;
    EXPECT_EQ(
      arm.detail.find("selected current-world target unavailable"),
      std::string::npos) << arm.detail;
  }
}

TEST(MpccArchitectureComparison, ExternalPrimalNeedsExactRecordedProblem)
{
  const auto report = verify_external_primal(
    recorded(source_snapshot()), Eigen::VectorXd::Zero(1));
  ASSERT_TRUE(report.source_accepted);
  ASSERT_EQ(report.arms.size(), 1U);
  EXPECT_EQ(report.arms.front().arm, Arm::ExternalPrimalI);
  EXPECT_EQ(report.arms.front().stage, Stage::SourceRejected);
  EXPECT_FALSE(report.arms.front().bundle.has_value());
  EXPECT_NE(
    report.arms.front().detail.find("requires an exact recorded QP"),
    std::string::npos);
}

TEST(MpccArchitectureComparison, KktEquilibrationKeepsExactProofChain)
{
  auto input = recorded_with_solved_qp(source_snapshot()).first;
  const auto report = compare_kkt_equilibration(input);

  ASSERT_TRUE(report.source_accepted) << report.detail;
  ASSERT_EQ(report.arms.size(), 1U);
  EXPECT_EQ(report.arms.front().arm, Arm::KktEquilibratedQ);
  EXPECT_EQ(report.arms.front().stage, Stage::Accepted)
    << report.arms.front().detail;
  EXPECT_TRUE(report.arms.front().bundle.has_value());
}

TEST(MpccArchitectureComparison, DeclaredStopLateralAuditHasNoAuthorityEdge)
{
  auto source = source_snapshot();
  source.request.states.back().reference[model::kLateralIndex] = 0.5;
  const auto report = compare_terminal_stop_lateral_contract(
    recorded(std::move(source)));

  ASSERT_TRUE(report.source_accepted) << report.detail;
  ASSERT_EQ(report.arms.size(), 10U);
  EXPECT_EQ(report.arms[0].arm, Arm::PersistentA);
  EXPECT_EQ(report.arms[1].arm, Arm::PersistentDeclaredStopR);
  EXPECT_EQ(report.arms[2].arm, Arm::PersistentStopScanS);
  EXPECT_EQ(report.arms[3].arm, Arm::PersistentNormalPathStopT);
  EXPECT_EQ(report.arms[4].arm, Arm::ProductionLeftG);
  EXPECT_EQ(report.arms[5].arm, Arm::ProductionLeftDeclaredStopR);
  EXPECT_EQ(report.arms[6].arm, Arm::ProductionLeftStopScanS);
  EXPECT_EQ(report.arms[7].arm, Arm::ProductionLeftNormalPathStopT);
  EXPECT_EQ(report.arms[8].arm, Arm::SevenStateStopU);
  EXPECT_EQ(report.arms[9].arm, Arm::SevenStateStopControlLatticeV);
  for (const std::size_t index : {0U, 1U, 2U, 4U, 5U, 6U}) {
    const auto & arm = report.arms[index];
    EXPECT_EQ(arm.stage, Stage::Accepted) << arm.detail;
    ASSERT_TRUE(arm.bundle.has_value());
    EXPECT_TRUE(arm.bundle->stop_suffix.available);
  }
  EXPECT_NEAR(report.arms[1].bundle->stop_suffix.hold_lateral_m, 0.5, 1e-12);
  EXPECT_GE(report.arms[2].terminal_stop_target_attempt_count, 1U);
  EXPECT_GE(report.arms[6].terminal_stop_target_attempt_count, 1U);
  EXPECT_NE(report.arms[3].stage, Stage::SourceRejected);
  EXPECT_NE(report.arms[7].stage, Stage::SourceRejected);
  EXPECT_NE(report.arms[8].stage, Stage::SourceRejected);
  EXPECT_NE(report.arms[9].stage, Stage::SourceRejected);
  EXPECT_GT(report.arms[9].candidate_count, 0U);
  if (report.arms[8].stage == Stage::Accepted) {
    ASSERT_FALSE(report.arms[8].solved_acceleration_mps2.empty());
    EXPECT_LE(report.arms[8].solved_acceleration_max_mps2, 1e-9);
    for (const double acceleration_mps2 :
      report.arms[8].solved_acceleration_mps2)
    {
      EXPECT_LE(acceleration_mps2, 1e-9);
    }
  }
  if (report.arms[9].stage == Stage::Accepted) {
    const auto & lattice = report.arms[9];
    ASSERT_FALSE(lattice.solved_steering_rate_radps.empty());
    ASSERT_EQ(
      lattice.solved_steering_rate_radps.size(),
      lattice.solved_acceleration_mps2.size());
    const double initial_sign =
      lattice.candidate_source == "positive-negative-hold" ? 1.0 : -1.0;
    for (std::size_t stage = 0U;
      stage < lattice.solved_steering_rate_radps.size(); ++stage)
    {
      const double rate = lattice.solved_steering_rate_radps[stage];
      if (static_cast<int>(stage) < lattice.lattice_transition_stage) {
        EXPECT_GT(initial_sign * rate, 0.1);
      } else if (static_cast<int>(stage) < lattice.lattice_ahead_stage) {
        EXPECT_LT(initial_sign * rate, -0.1);
      } else {
        EXPECT_NEAR(rate, 0.0, 1e-6);
      }
    }
    EXPECT_LE(lattice.solved_acceleration_max_mps2, 1e-9);
  }
}

TEST(MpccArchitectureComparison, SharedStopLatticeRebasesMaximumBrakingLaw)
{
  const auto source = source_snapshot();
  shadow::SolverContext solver;
  const auto normal = solver.evaluate(source);
  ASSERT_EQ(normal.outcome, shadow::Outcome::Solved) << normal.detail;
  ASSERT_NE(normal.execution_artifact, nullptr);

  const auto stop = stop_lattice::build_maximum_braking_candidate(
    source, *normal.execution_artifact,
    solver.physical_constraint_tolerance());
  ASSERT_TRUE(stop.accepted()) << stop.detail;
  EXPECT_EQ(
    stop.candidate.execution_prefix_steps,
    stop.candidate.request.horizon_steps);
  ASSERT_EQ(
    stop.candidate.request.states.size(),
    stop.candidate.request.inputs.size() + 1U);
  EXPECT_EQ(
    stop.candidate.request.states.front().lower,
    stop.candidate.request.initial_state);
  EXPECT_EQ(
    stop.candidate.request.states.front().upper,
    stop.candidate.request.initial_state);
  double previous_velocity =
    stop.candidate.request.states.front().lower[model::kVelocityIndex];
  for (std::size_t stage = 1U;
    stage < stop.candidate.request.states.size(); ++stage)
  {
    const auto & state = stop.candidate.request.states[stage];
    EXPECT_DOUBLE_EQ(
      state.lower[model::kVelocityIndex],
      state.upper[model::kVelocityIndex]);
    EXPECT_LE(state.lower[model::kVelocityIndex], previous_velocity + 1e-12);
    previous_velocity = state.lower[model::kVelocityIndex];
  }
  EXPECT_NEAR(previous_velocity, 0.0, 1e-12);
}

TEST(MpccArchitectureComparison, SharedStopLatticePopulationIsDeterministic)
{
  const auto source = source_snapshot();
  shadow::SolverContext solver;
  const auto normal = solver.evaluate(source);
  ASSERT_EQ(normal.outcome, shadow::Outcome::Solved) << normal.detail;
  ASSERT_NE(normal.execution_artifact, nullptr);
  const auto stop = stop_lattice::build_maximum_braking_candidate(
    source, *normal.execution_artifact,
    solver.physical_constraint_tolerance());
  ASSERT_TRUE(stop.accepted()) << stop.detail;

  const auto first = stop_lattice::build_population(
    stop.candidate, solver.physical_constraint_tolerance());
  const auto second = stop_lattice::build_population(
    stop.candidate, solver.physical_constraint_tolerance());
  ASSERT_FALSE(first.empty());
  ASSERT_EQ(first.size(), second.size());
  for (std::size_t index = 0U; index < first.size(); ++index) {
    ASSERT_EQ(first[index].reason, second[index].reason);
    ASSERT_EQ(
      first[index].schedule.steering_rate_radps,
      second[index].schedule.steering_rate_radps);
    EXPECT_EQ(
      first[index].schedule.first_switch_stage,
      second[index].schedule.first_switch_stage);
    EXPECT_EQ(
      first[index].schedule.second_switch_stage,
      second[index].schedule.second_switch_stage);
  }
}

TEST(MpccArchitectureComparison, SharedStopLatticeAnytimeOrderIsCompleteAndFair)
{
  const auto source = stoppable_source_snapshot();
  shadow::SolverContext solver;
  const auto normal = solver.evaluate(source);
  ASSERT_EQ(normal.outcome, shadow::Outcome::Solved) << normal.detail;
  ASSERT_NE(normal.execution_artifact, nullptr);
  const auto stop = stop_lattice::build_maximum_braking_candidate(
    source, *normal.execution_artifact,
    solver.physical_constraint_tolerance());
  ASSERT_TRUE(stop.accepted()) << stop.detail;

  const auto legacy = stop_lattice::build_population(
    stop.candidate, solver.physical_constraint_tolerance());
  const auto anytime = stop_lattice::build_anytime_population(
    stop.candidate, solver.physical_constraint_tolerance());
  const auto repeated = stop_lattice::build_anytime_population(
    stop.candidate, solver.physical_constraint_tolerance());
  ASSERT_EQ(anytime.candidates.size(), legacy.size());
  ASSERT_EQ(anytime.legacy_rank_by_candidate.size(), legacy.size());
  ASSERT_EQ(repeated.candidates.size(), anytime.candidates.size());

  std::vector<bool> observed_legacy_rank(legacy.size(), false);
  for (std::size_t index = 0U; index < anytime.candidates.size(); ++index) {
    const auto legacy_rank = anytime.legacy_rank_by_candidate[index];
    ASSERT_GE(legacy_rank, 1U);
    ASSERT_LE(legacy_rank, legacy.size());
    ASSERT_FALSE(observed_legacy_rank[legacy_rank - 1U]);
    observed_legacy_rank[legacy_rank - 1U] = true;
    const auto & candidate = anytime.candidates[index];
    const auto & original = legacy[legacy_rank - 1U];
    EXPECT_EQ(candidate.reason, original.reason);
    EXPECT_EQ(
      candidate.schedule.steering_rate_radps,
      original.schedule.steering_rate_radps);
    EXPECT_EQ(
      repeated.legacy_rank_by_candidate[index], legacy_rank);
    EXPECT_EQ(
      repeated.candidates[index].schedule.steering_rate_radps,
      candidate.schedule.steering_rate_radps);
  }
  EXPECT_TRUE(std::all_of(
      observed_legacy_rank.begin(), observed_legacy_rank.end(),
      [](const bool observed) {return observed;}));

  ASSERT_GE(anytime.candidates.size(), 4U);
  for (std::size_t index = 0U; index + 1U < anytime.candidates.size();
    index += 2U)
  {
    const auto & first = anytime.candidates[index];
    const auto & second = anytime.candidates[index + 1U];
    ASSERT_TRUE(first.accepted());
    ASSERT_TRUE(second.accepted());
    EXPECT_EQ(
      first.schedule.first_switch_stage,
      second.schedule.first_switch_stage);
    EXPECT_EQ(
      first.schedule.second_switch_stage,
      second.schedule.second_switch_stage);
    EXPECT_EQ(
      first.schedule.initial_rate_sign,
      anytime.preferred_initial_rate_sign);
    EXPECT_EQ(
      second.schedule.initial_rate_sign,
      -anytime.preferred_initial_rate_sign);
  }

  const auto & nominal = anytime.candidates.front().schedule;
  EXPECT_EQ(
    nominal.first_switch_stage,
    std::lround(0.15 * stop.candidate.request.horizon_steps));
  EXPECT_EQ(
    nominal.second_switch_stage,
    std::lround(0.30 * stop.candidate.request.horizon_steps));
  const auto & farthest = anytime.candidates[2U].schedule;
  const auto distance_from_nominal = [&nominal, &stop](
      const stop_lattice::Schedule & schedule) {
      const double horizon = static_cast<double>(
        stop.candidate.request.horizon_steps);
      const double first = static_cast<double>(
        schedule.first_switch_stage - nominal.first_switch_stage) / horizon;
      const double second = static_cast<double>(
        schedule.second_switch_stage - nominal.second_switch_stage) / horizon;
      return first * first + second * second;
    };
  const double selected_distance = distance_from_nominal(farthest);
  for (const auto & candidate : legacy) {
    ASSERT_TRUE(candidate.accepted());
    EXPECT_GE(
      selected_distance + 1e-15,
      distance_from_nominal(candidate.schedule));
  }

  auto negative_continuity = stop.candidate;
  negative_continuity.request.previous_input[model::kSteeringRateIndex] =
    -0.1;
  const auto negative_first = stop_lattice::build_anytime_population(
    negative_continuity, solver.physical_constraint_tolerance());
  ASSERT_FALSE(negative_first.candidates.empty());
  EXPECT_EQ(negative_first.preferred_initial_rate_sign, -1);
  EXPECT_EQ(negative_first.candidates.front().schedule.initial_rate_sign, -1);
}

TEST(MpccArchitectureComparison, LiveStopShadowRejectsMismatchedSource)
{
  const auto source = source_snapshot();
  shadow::SolverContext normal_solver;
  const auto normal = normal_solver.evaluate(source);
  ASSERT_EQ(normal.outcome, shadow::Outcome::Solved) << normal.detail;
  ASSERT_NE(normal.execution_artifact, nullptr);

  auto mismatched = source;
  ++mismatched.identity.sequence;
  shadow::SolverContext stop_solver;
  const auto result = stop_lattice_shadow::evaluate(
    mismatched, *normal.execution_artifact, stop_solver);
  EXPECT_EQ(result.reason, stop_lattice_shadow::Reason::InvalidSource);
  EXPECT_EQ(result.attempted_candidate_count, 0U);
  EXPECT_EQ(result.certified_stop_plan, nullptr);
}

TEST(MpccArchitectureComparison, LiveStopShadowBuildsCertifiedObservation)
{
  const auto source = stoppable_source_snapshot();
  shadow::SolverContext normal_solver;
  const auto normal = normal_solver.evaluate(source);
  ASSERT_EQ(normal.outcome, shadow::Outcome::Solved) << normal.detail;
  ASSERT_NE(normal.execution_artifact, nullptr);

  shadow::SolverContext stop_solver;
  const auto result = stop_lattice_shadow::evaluate(
    source, *normal.execution_artifact, stop_solver);
  ASSERT_EQ(result.reason, stop_lattice_shadow::Reason::Accepted)
    << result.detail;
  ASSERT_TRUE(result.accepted());
  EXPECT_GT(result.attempted_candidate_count, 0U);
  EXPECT_GE(result.population_size, result.attempted_candidate_count);
  EXPECT_GT(result.selected_legacy_rank, 0U);
  EXPECT_EQ(
    mpcc_rate_resolved_certified_plan::validate(
      *result.certified_stop_plan),
      mpcc_rate_resolved_certified_plan::RejectReason::None);
  ASSERT_NE(result.certified_stop_plan->solver_source_snapshot, nullptr);
  EXPECT_TRUE(
    mpcc_rate_resolved_execution_artifact::same_identity(
      result.certified_stop_plan->solver_source_snapshot->identity,
      result.certified_stop_plan->execution_artifact->identity));
  EXPECT_GT(
    result.certified_stop_plan->solver_source_snapshot->
    control_prediction_origin_sec,
    source.control_prediction_origin_sec);
  ASSERT_FALSE(
    result.certified_stop_plan->solver_source_snapshot->request.states.empty());
  EXPECT_DOUBLE_EQ(
    result.certified_stop_plan->solver_source_snapshot->request.states.back().
    lower[model::kVelocityIndex],
    0.0);
  EXPECT_DOUBLE_EQ(
    result.certified_stop_plan->solver_source_snapshot->request.states.back().
    upper[model::kVelocityIndex],
    0.0);
}

TEST(MpccArchitectureComparison, LiveStopShadowStopsAfterSupersededSolve)
{
  const auto source = stoppable_source_snapshot();
  shadow::SolverContext normal_solver;
  const auto normal = normal_solver.evaluate(source);
  ASSERT_EQ(normal.outcome, shadow::Outcome::Solved) << normal.detail;
  ASSERT_NE(normal.execution_artifact, nullptr);

  std::size_t supersession_checks = 0U;
  stop_lattice_shadow::EvaluationControl control;
  control.superseded = [&supersession_checks]() {
      ++supersession_checks;
      return supersession_checks >= 3U;
    };
  shadow::SolverContext stop_solver;
  const auto result = stop_lattice_shadow::evaluate(
    source, *normal.execution_artifact, stop_solver, control);

  EXPECT_EQ(result.reason, stop_lattice_shadow::Reason::Superseded);
  EXPECT_EQ(result.attempted_candidate_count, 1U);
  EXPECT_EQ(result.certified_stop_plan, nullptr);
  EXPECT_EQ(result.detail, "newer observation epoch submitted");
}

TEST(MpccArchitectureComparison, LiveStopShadowMailboxIsMonotonic)
{
  stop_lattice_shadow::Mailbox mailbox;
  stop_lattice_shadow::Result newer;
  newer.source_normal_identity = source_snapshot().identity;
  newer.total_compute_ms = 1.0;
  EXPECT_EQ(
    mailbox.publish(newer),
    stop_lattice_shadow::PublishReason::Accepted);

  stop_lattice_shadow::Result older;
  older.source_normal_identity = source_snapshot().identity;
  older.source_normal_identity.sequence -= 1U;
  older.total_compute_ms = 1.0;
  EXPECT_EQ(
    mailbox.publish(older),
    stop_lattice_shadow::PublishReason::SequenceRollback);
  const auto latest = mailbox.latest_after(0U);
  ASSERT_TRUE(latest.has_value());
  EXPECT_EQ(latest->source_normal_identity.sequence, 9U);
  const auto state = mailbox.state();
  EXPECT_EQ(state.accepted_count, 1U);
  EXPECT_EQ(state.sequence_rollback_count, 1U);
}

TEST(MpccArchitectureComparison, ExternalPrimalUsesExactPhysicalProofChain)
{
  auto [input, primal] = recorded_with_solved_qp(source_snapshot());
  const auto report = verify_external_primal(input, primal);
  ASSERT_TRUE(report.source_accepted) << report.detail;
  ASSERT_EQ(report.arms.size(), 1U);
  const auto & arm = report.arms.front();
  EXPECT_EQ(arm.arm, Arm::ExternalPrimalI);
  EXPECT_EQ(arm.stage, Stage::Accepted) << arm.detail;
  EXPECT_TRUE(arm.bundle.has_value());
  EXPECT_LE(arm.maximum_external_normalized_constraint_violation, 1.0);
  EXPECT_NE(arm.candidate_fingerprint, 0U);
}

TEST(MpccArchitectureComparison, ExternalPrimalCannotBypassExactQpRows)
{
  auto [input, primal] = recorded_with_solved_qp(source_snapshot());
  ASSERT_GT(primal.size(), 0);
  primal[0] += 10.0;
  const auto report = verify_external_primal(input, primal);
  ASSERT_TRUE(report.source_accepted);
  ASSERT_EQ(report.arms.size(), 1U);
  const auto & arm = report.arms.front();
  EXPECT_EQ(arm.stage, Stage::SolverRejected);
  EXPECT_FALSE(arm.bundle.has_value());
  EXPECT_NE(arm.detail.find("constraint-rejected"), std::string::npos);
}

TEST(MpccArchitectureComparison, ExternalPrimalBucketOracleKeepsPhysicalProofs)
{
  auto [input, primal] = recorded_with_solved_qp(source_snapshot());
  for (const auto policy : {
      ExternalPrimalConstraintPolicy::OmitWallHeadingBucket,
      ExternalPrimalConstraintPolicy::OmitWallLagBucket})
  {
    const auto report = verify_external_primal(input, primal, policy);
    ASSERT_TRUE(report.source_accepted) << report.detail;
    ASSERT_EQ(report.arms.size(), 1U);
    const auto & arm = report.arms.front();
    EXPECT_EQ(arm.stage, Stage::Accepted) << arm.detail;
    ASSERT_TRUE(arm.bundle.has_value());
    EXPECT_EQ(
      arm.bundle->wall_certificate.outcome,
      mpcc_rate_resolved_physical_wall::Outcome::Accepted);
    EXPECT_TRUE(arm.bundle->dynamic_certificate.clear);
    EXPECT_NE(arm.detail.find("exact-proofs"), std::string::npos);
  }
}

TEST(MpccArchitectureComparison, PhysicalNonlinearOracleCannotBypassExactProofs)
{
  auto [input, primal] = recorded_with_solved_qp(source_snapshot());
  // PhysicalNonlinearOracle intentionally skips affine residual ownership,
  // so its positive fixture must itself be an exact nonlinear rollout rather
  // than a merely tolerance-valid affine QP iterate.  On this straight,
  // zero-steering fixture, v=2 m/s and v_theta=2 m/s advance progress by
  // exactly 0.2 m per 0.1 s stage while every other state remains constant.
  const int horizon = input.source.request.horizon_steps;
  const int state_values = model::kStateDimension * (horizon + 1);
  ASSERT_EQ(primal.size(), state_values + model::kInputDimension * horizon);
  for (int stage = 0; stage <= horizon; ++stage) {
    const int offset = stage * model::kStateDimension;
    primal.segment(offset, model::kStateDimension).setZero();
    primal[offset + model::kVelocityIndex] = 2.0;
    primal[offset + model::kProgressIndex] = 0.2 * stage;
  }
  for (int stage = 0; stage < horizon; ++stage) {
    const int offset = state_values + stage * model::kInputDimension;
    primal.segment(offset, model::kInputDimension).setZero();
    primal[offset + model::kVirtualProgressSpeedIndex] = 2.0;
  }
  const auto accepted = verify_external_primal(
    input, primal, ExternalPrimalConstraintPolicy::PhysicalNonlinearOracle);
  ASSERT_TRUE(accepted.source_accepted) << accepted.detail;
  ASSERT_EQ(accepted.arms.size(), 1U);
  EXPECT_EQ(accepted.arms.front().stage, Stage::Accepted)
    << accepted.arms.front().detail;
  EXPECT_TRUE(accepted.arms.front().bundle.has_value());

  // The physical oracle deliberately ignores affine predicted-state samples
  // and reconstructs them from the current state plus the control sequence.
  // It must still fail closed on the immutable exact wall proof.
  auto occupied_grid = std::make_shared<recovery::OccupancyGrid>(
    *input.source.wall_grid);
  std::fill(
    occupied_grid->cells.begin(), occupied_grid->cells.end(),
    recovery::CellState::Occupied);
  input.source.wall_grid = occupied_grid;
  input.source.replay_world->wall_grid_fingerprint =
    recovery::occupancy_grid_fingerprint(*occupied_grid);
  input.interaction_fingerprint =
    architecture::fingerprint_interaction_snapshot(input.source);
  const auto rejected = verify_external_primal(
    input, primal, ExternalPrimalConstraintPolicy::PhysicalNonlinearOracle);
  ASSERT_TRUE(rejected.source_accepted) << rejected.detail;
  ASSERT_EQ(rejected.arms.size(), 1U);
  EXPECT_NE(rejected.arms.front().stage, Stage::Accepted);
  EXPECT_FALSE(rejected.arms.front().bundle.has_value());
}

}  // namespace
}  // namespace multi_purpose_mpc_ros::mpcc_architecture_comparison
