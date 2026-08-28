#include "multi_purpose_mpc_ros/mpcc_architecture_comparison.hpp"

#include "multi_purpose_mpc_ros/mpcc_rate_resolved.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_adapter.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_problem.hpp"
#include "multi_purpose_mpc_ros/persistent_osqp.hpp"

#include <gtest/gtest.h>

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
  // The replay obstacle is deliberately clear in this infrastructure test;
  // its identity still seals the same target/world used by every arm.
  world.obstacles.push_back(
    shadow::ReplayDynamicObstacle{
      "d2", 3.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.01, 0.2, 13U});
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
    value.recorded_qp.problem = qp.value();
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

TEST(MpccArchitectureComparison, ExternalPrimalNeedsExactRecordedProblem)
{
  const auto report = verify_external_primal(
    recorded(source_snapshot()), Eigen::VectorXd::Zero(1));
  ASSERT_TRUE(report.source_accepted);
  ASSERT_EQ(report.arms.size(), 1U);
  EXPECT_EQ(report.arms.front().arm, Arm::ExternalPrimalI);
  EXPECT_EQ(report.arms.front().stage, Stage::SolverRejected);
  EXPECT_FALSE(report.arms.front().bundle.has_value());
  EXPECT_NE(
    report.arms.front().detail.find("dimension-contract-mismatch"),
    std::string::npos);
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

}  // namespace
}  // namespace multi_purpose_mpc_ros::mpcc_architecture_comparison
