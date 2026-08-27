#include "multi_purpose_mpc_ros/mpcc_rate_resolved_shadow.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_physical_adapter.hpp"

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <cmath>
#include <limits>

namespace shadow =
  multi_purpose_mpc_ros::mpcc_rate_resolved_shadow;
namespace adapter =
  multi_purpose_mpc_ros::mpcc_rate_resolved_adapter;
namespace model = multi_purpose_mpc_ros::mpcc_rate_resolved;
namespace problem = multi_purpose_mpc_ros::mpcc_rate_resolved_problem;
namespace solver = multi_purpose_mpc_ros::persistent_osqp;
namespace execution =
  multi_purpose_mpc_ros::mpcc_rate_resolved_execution_artifact;
namespace physical =
  multi_purpose_mpc_ros::mpcc_rate_resolved_physical_adapter;
namespace contract =
  multi_purpose_mpc_ros::mpcc_execution_contract;
namespace wall_refinement =
  multi_purpose_mpc_ros::mpcc_rate_resolved_wall_refinement;
namespace recovery =
  multi_purpose_mpc_ros::recovery_footprint;

namespace
{

contract::MpccProblemContext source_context(
  const std::uint64_t decision_id, const std::uint64_t stage_geometry_id)
{
  contract::MpccProblemContext context;
  context.decision_id = decision_id;
  context.intent = contract::ControlIntent::Track;
  context.intent_generation = 1U;
  context.observation_generation = 2U;
  context.stage_geometry_id = stage_geometry_id;
  context.horizon_steps = 3U;
  context.formulation =
    contract::Formulation::VelocitySteeringYawResponseProgress7State;
  context.state_schema_id = "ey-elag-epsi-v-progress-steering-v1";
  context.input_schema_id = "accel-steering-rate-progress-rate-v1";
  context.bounds_schema_id = "stage-wall-v1";
  context.cost_schema_id = "velocity-progress-steering-rate-v1";
  return contract::seal_problem_context(std::move(context));
}

adapter::Request straight_request(const int horizon = 3)
{
  adapter::Request request;
  request.horizon_steps = horizon;
  request.initial_state << 0.0, 0.0, 0.0, 2.0, 0.0;
  request.current_steering_rad = 0.10;
  request.current_response_steering_rad = 0.08;
  request.wheelbase_m = 2.0;
  request.yaw_response_gain = 0.75;
  request.yaw_response_time_constant_sec = 0.13;
  request.maximum_abs_steering_rad = 0.60;
  request.maximum_abs_steering_rate_radps = 0.70;
  request.states.resize(static_cast<std::size_t>(horizon + 1));
  for (int stage = 0; stage <= horizon; ++stage) {
    auto & state = request.states[static_cast<std::size_t>(stage)];
    state.reference << 0.0, 0.0, 0.0, 2.0, 0.2 * stage;
    state.lower << -1.0, -1.0, -1.0, 0.0, -1.0;
    state.upper << 1.0, 1.0, 1.0, 4.0, 2.0;
    state.weight << 2.0, 1.0, 2.0, 1.0, 1.0;
    state.linear_cost[4] = -0.5;
  }
  request.inputs.resize(static_cast<std::size_t>(horizon));
  for (int stage = 0; stage < horizon; ++stage) {
    auto & input = request.inputs[static_cast<std::size_t>(stage)];
    input.reference << 0.0, std::tan(0.10) / 2.0, 2.0;
    input.lower << -1.0, -0.30, 0.0;
    input.upper << 1.0, 0.30, 4.0;
    input.weight << 1.0, 1.0, 1.0;
    input.path_curvature_radpm = 0.0;
    input.stage_dt_sec = 0.10;
  }
  request.previous_input << 0.0, std::tan(0.10) / 2.0, 2.0;
  request.input_delta_weight << 0.2, 0.3, 0.1;
  return request;
}

shadow::Snapshot snapshot(const std::uint64_t sequence = 1U)
{
  shadow::Snapshot result;
  result.identity.sequence = sequence;
  result.identity.source_context =
    source_context(42U + sequence, 201U + sequence);
  result.identity.snapshot_sec = 10.0 + 0.1 * sequence;
  result.control_prediction_origin_sec = result.identity.snapshot_sec + 0.13;
  result.request = straight_request();
  result.execution_prefix_steps = result.request.horizon_steps;
  result.course_progress_origin_m = 50.0;
  result.nominal_path_distance_m = {0.0, 0.2, 0.4, 0.6};
  result.publication_interval_sec = 0.025;
  return result;
}

recovery::OccupancyGrid corridor_grid()
{
  recovery::OccupancyGrid grid;
  grid.width = 240U;
  grid.height = 240U;
  grid.resolution_m = 0.1;
  grid.origin_x_m = 0.0;
  grid.origin_y_m = 0.0;
  grid.y_axis = recovery::YAxisConvention::RowZeroAtMinimumY;
  grid.cells.assign(grid.width * grid.height, recovery::CellState::Free);
  for (std::size_t column = 0U; column < grid.width; ++column) {
    const auto wall = grid.world_to_grid(
      grid.origin_x_m + grid.resolution_m * static_cast<double>(column),
      10.0);
    if (wall.has_value()) {
      grid.cells[wall->row * grid.width + wall->column] =
        recovery::CellState::Occupied;
    }
  }
  return grid;
}

wall_refinement::Request wall_request(
  const recovery::OccupancyGrid & grid, const double heading_rad)
{
  wall_refinement::Request request;
  request.active = true;
  request.wall_grid = &grid;
  request.footprint = recovery::FootprintExtents{1.0, 1.0, 0.2, 0.2, 0.0};
  request.course_frame_knots = {
    {0.0, 5.0, 8.0, 0.0, 0},
    {20.0, 25.0, 8.0, 0.0, 1}};
  request.course_progress_origin_m = 0.0;
  request.heading_bucket_width_rad = 0.025;
  request.translation_bucket_width_m = 0.05;
  request.lateral_sample_step_m = 0.05;
  request.boundary_guard_m = 0.001;
  request.stages.push_back(wall_refinement::StageRequest{
    5.0, 1.4, 0.0, heading_rad,
    -0.5, 1.8, -1.0, 1.0, -1.0, 1.0, 4.0, 6.0});
  return request;
}

}  // namespace

TEST(MpccRateResolvedWallRefinement, ResolvesUnionSupportBeforeProgressIsKnown)
{
  const auto result =
    wall_refinement::resolve_pre_refinement_lateral_support(
    wall_refinement::PreRefinementLateralSupportRequest{
      true, -0.4,
      {-0.8, -0.2, 1.7, 1.6},
      {0.6, 0.9, 2.0, 2.1}});
  ASSERT_TRUE(result.valid);
  ASSERT_TRUE(result.applied);
  EXPECT_EQ(
    result.reason,
    wall_refinement::PreRefinementLateralSupportReason::Accepted);
  EXPECT_DOUBLE_EQ(result.lower_m, -0.8);
  EXPECT_DOUBLE_EQ(result.upper_m, 2.1);
}

TEST(MpccRateResolvedWallRefinement, RejectsMalformedPreRefinementProfile)
{
  const auto result =
    wall_refinement::resolve_pre_refinement_lateral_support(
    wall_refinement::PreRefinementLateralSupportRequest{
      true, 0.0, {-0.5, 0.3}, {0.5, 0.2}});
  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.applied);
  EXPECT_EQ(
    result.reason,
    wall_refinement::PreRefinementLateralSupportReason::InvalidInput);
}

TEST(MpccRateResolvedShadow, DoesNotBindFutureWallSampleBeforeProgressSolve)
{
  shadow::SolverContext context;
  auto input = snapshot();
  for (std::size_t stage = 1U; stage < input.request.states.size(); ++stage) {
    input.request.states[stage].lower[0] = 1.7;
    input.request.states[stage].upper[0] = 1.9;
  }
  input.progress_aligned_wall_refinement_active = true;
  input.wall_reference_progress_m = {0.0, 1.0, 2.0, 3.0};
  input.wall_lower_m = {-0.5, -0.5, 1.7, 1.7};
  input.wall_upper_m = {0.5, 0.5, 1.9, 1.9};
  input.progress_wall_profile_diagnostic = "future-corridor-shift";

  const auto result = context.evaluate(input);
  ASSERT_EQ(result.outcome, shadow::Outcome::Solved) << result.detail;
  EXPECT_TRUE(result.pre_refinement_lateral_support_applied);
  EXPECT_EQ(
    result.pre_refinement_lateral_support_reason,
    wall_refinement::PreRefinementLateralSupportReason::Accepted);
  EXPECT_DOUBLE_EQ(result.pre_refinement_lateral_support_lower_m, -0.5);
  EXPECT_DOUBLE_EQ(result.pre_refinement_lateral_support_upper_m, 1.9);
  EXPECT_TRUE(result.progress_wall_refinement_applied);
  EXPECT_TRUE(result.progress_wall_refinement_solved);
  EXPECT_TRUE(result.post_refinement_physical_proof_checked);
  EXPECT_TRUE(result.post_refinement_physical_proof_accepted);
  EXPECT_EQ(
    result.post_refinement_linearization_requested,
    result.post_refinement_linearization_count > 0U);
  if (result.post_refinement_linearization_requested) {
    EXPECT_TRUE(result.post_refinement_linearization_applied);
    EXPECT_TRUE(result.post_refinement_linearization_bootstrap_applied);
    EXPECT_TRUE(result.post_refinement_linearization_solved);
    EXPECT_EQ(
      result.post_refinement_linearization_reason,
      multi_purpose_mpc_ros::mpcc_rate_resolved_adapter::
      RelinearizationReason::Accepted);
  }
}

TEST(MpccRateResolvedShadow, SolvesAndSamplesOnePublicationInterval)
{
  shadow::SolverContext context;
  const auto input = snapshot();
  const auto result = context.evaluate(input);
  ASSERT_EQ(result.outcome, shadow::Outcome::Solved) << result.detail;
  EXPECT_EQ(
    result.receding_warm_start_reason,
    shadow::RecedingWarmStartReason::CurrentProblemBootstrap);
  EXPECT_TRUE(result.receding_warm_start_applied);
  EXPECT_TRUE(result.successive_linearization_requested);
  EXPECT_TRUE(result.successive_linearization_applied);
  EXPECT_TRUE(result.successive_linearization_bootstrap_applied);
  EXPECT_TRUE(result.successive_linearization_solved);
  EXPECT_EQ(
    result.successive_linearization_reason,
    multi_purpose_mpc_ros::mpcc_rate_resolved_adapter::
    RelinearizationReason::Accepted);
  EXPECT_FALSE(result.post_refinement_linearization_requested);
  EXPECT_FALSE(result.post_refinement_linearization_applied);
  EXPECT_FALSE(result.post_refinement_linearization_bootstrap_applied);
  EXPECT_FALSE(result.post_refinement_linearization_solved);
  EXPECT_FALSE(result.post_refinement_physical_proof_checked);
  EXPECT_FALSE(result.post_refinement_physical_proof_accepted);
  EXPECT_EQ(result.post_refinement_linearization_count, 0U);
  EXPECT_NE(
    result.receding_warm_start_diagnostic.find("previous=empty-cache"),
    std::string::npos);
  EXPECT_TRUE(shadow::result_valid(result));
  EXPECT_TRUE(result.constraints_satisfied);
  EXPECT_TRUE(result.actuation_sampled);
  EXPECT_DOUBLE_EQ(result.initial_steering_rad, input.request.current_steering_rad);
  EXPECT_TRUE(std::isfinite(result.solver_initial_steering_rad));
  EXPECT_NEAR(
    result.sampled_steering_rad,
    result.initial_steering_rad +
    result.first_steering_rate_radps * input.publication_interval_sec,
    1e-9);
  EXPECT_LE(
    std::abs(result.first_steering_rate_radps),
    input.request.maximum_abs_steering_rate_radps + 1e-6);
  EXPECT_LT(
    result.first_steering_rate_physical_lower_radps,
    result.first_steering_rate_solver_lower_radps);
  EXPECT_LT(
    result.first_steering_rate_solver_upper_radps,
    result.first_steering_rate_physical_upper_radps);
  EXPECT_GT(result.first_steering_rate_certificate_margin_radps, 0.0);
  EXPECT_GE(
    result.first_steering_rate_radps,
    result.first_steering_rate_solver_lower_radps -
    result.solver.physical_global_tolerance);
  EXPECT_LE(
    result.first_steering_rate_radps,
    result.first_steering_rate_solver_upper_radps +
    result.solver.physical_global_tolerance);
  EXPECT_EQ(result.certified_stage_count, 3U);
  EXPECT_EQ(result.sampled_stage_index, 0U);
  EXPECT_NEAR(result.sampled_stage_elapsed_sec, 0.025, 1e-12);
  EXPECT_NEAR(result.certified_horizon_duration_sec, 0.30, 1e-12);
  ASSERT_NE(result.execution_artifact, nullptr);
  EXPECT_EQ(
    execution::validate(*result.execution_artifact),
    execution::RejectReason::None);
  EXPECT_EQ(result.execution_artifact->predicted_states.size(), 4U);
  EXPECT_EQ(result.execution_artifact->control_stages.size(), 3U);
  EXPECT_EQ(result.execution_artifact->nominal_path_distance_m.size(), 4U);
  EXPECT_EQ(result.execution_artifact->lateral_lower_m.size(), 4U);
  EXPECT_DOUBLE_EQ(result.execution_artifact->course_progress_origin_m, 50.0);
  EXPECT_DOUBLE_EQ(
    result.execution_artifact->prediction_origin_sec,
    input.control_prediction_origin_sec);
  EXPECT_LT(
    result.execution_artifact->completed_sec,
    result.execution_artifact->prediction_origin_sec);
  EXPECT_DOUBLE_EQ(
    result.execution_artifact->semantic_initial_steering_rad,
    input.request.current_steering_rad);
  EXPECT_DOUBLE_EQ(
    result.execution_artifact->control_stages.front().steering_rate_radps,
    result.first_steering_rate_radps);
  const auto physical_result = physical::build(
    *result.execution_artifact, contract::ControlIntent::Track,
    input.identity.source_context.stage_geometry_id);
  EXPECT_EQ(physical_result.reason, physical::RejectReason::None);
  EXPECT_TRUE(physical_result.exact_trajectory.has_value());

  auto invalid = result;
  invalid.first_steering_rate_certificate_margin_radps =
    std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(shadow::result_valid(invalid));
}

TEST(MpccRateResolvedShadow, CurrentProblemBootstrapOwnsCurrentAffineDynamics)
{
  const auto adapted = adapter::build(
    straight_request(), solver::PhysicalConstraintTolerance{1.0e-3, 1.0e-3});
  ASSERT_TRUE(adapted.has_value());
  const auto assembled = problem::assemble(adapted->problem);
  ASSERT_TRUE(assembled.has_value());
  const auto bootstrap = shadow::build_current_problem_bootstrap(
    adapted->problem,
    static_cast<std::size_t>(assembled->lower_bound.size()));
  ASSERT_TRUE(bootstrap.has_value());
  const auto & primal = bootstrap->primal;
  EXPECT_TRUE(primal.allFinite());
  EXPECT_EQ(bootstrap->dual.size(), assembled->lower_bound.size());
  EXPECT_TRUE(bootstrap->dual.isZero());
  EXPECT_TRUE(
    primal.head<model::kStateDimension>().isApprox(
      adapted->problem.initial_state));

  const int state_values = model::kStateDimension *
    (adapted->problem.horizon_steps + 1);
  for (int stage = 0; stage < adapted->problem.horizon_steps; ++stage) {
    const int state_offset = stage * model::kStateDimension;
    const int next_state_offset = (stage + 1) * model::kStateDimension;
    const int input_offset = stage * model::kInputDimension;
    const auto input = primal.segment<model::kInputDimension>(
      state_values + input_offset);
    for (int element = 0; element < model::kInputDimension; ++element) {
      EXPECT_GE(input[element], adapted->problem.input_lower[input_offset + element]);
      EXPECT_LE(input[element], adapted->problem.input_upper[input_offset + element]);
    }
    const auto & linearization =
      adapted->problem.linearizations[static_cast<std::size_t>(stage)];
    const auto equality_residual =
      -primal.segment<model::kStateDimension>(next_state_offset) +
      linearization.state_matrix *
      primal.segment<model::kStateDimension>(state_offset) +
      linearization.input_matrix * input - linearization.equality_offset;
    EXPECT_LT(equality_residual.lpNorm<Eigen::Infinity>(), 1.0e-12);
  }
}

TEST(
  MpccRateResolvedShadow,
  CurrentProblemBootstrapTransportsOnlySameProblemInputs)
{
  const auto adapted = adapter::build(
    straight_request(), solver::PhysicalConstraintTolerance{1.0e-3, 1.0e-3});
  ASSERT_TRUE(adapted.has_value());
  const auto assembled = problem::assemble(adapted->problem);
  ASSERT_TRUE(assembled.has_value());
  const int state_values = model::kStateDimension *
    (adapted->problem.horizon_steps + 1);
  const int variable_count = state_values + model::kInputDimension *
    adapted->problem.horizon_steps;
  Eigen::VectorXd preceding = Eigen::VectorXd::Constant(variable_count, 99.0);
  for (int stage = 0; stage < adapted->problem.horizon_steps; ++stage) {
    const int input_offset = stage * model::kInputDimension;
    const int primal_input = state_values + input_offset;
    for (int element = 0; element < model::kInputDimension; ++element) {
      preceding[primal_input + element] =
        adapted->problem.input_lower[input_offset + element] +
        0.25 * (adapted->problem.input_upper[input_offset + element] -
        adapted->problem.input_lower[input_offset + element]);
    }
  }

  const auto bootstrap = shadow::build_current_problem_bootstrap(
    adapted->problem,
    static_cast<std::size_t>(assembled->lower_bound.size()), &preceding);
  ASSERT_TRUE(bootstrap.has_value());
  EXPECT_TRUE(
    bootstrap->primal.head<model::kStateDimension>().isApprox(
      adapted->problem.initial_state));
  for (int stage = 0; stage < adapted->problem.horizon_steps; ++stage) {
    const int input_offset = stage * model::kInputDimension;
    const int primal_input = state_values + input_offset;
    EXPECT_TRUE(
      bootstrap->primal.segment<model::kInputDimension>(primal_input).isApprox(
        preceding.segment<model::kInputDimension>(primal_input)));
  }
  EXPECT_TRUE(bootstrap->dual.isZero());
}

TEST(MpccRateResolvedShadow, RecedingWarmStartRebasesProgressAndCurrentState)
{
  const auto previous_snapshot = snapshot(1U);
  auto current = snapshot(2U);
  current.identity.source_context.intent_generation =
    previous_snapshot.identity.source_context.intent_generation;
  current.identity.source_context.stage_geometry_id = 999U;
  current.identity.source_context =
    contract::seal_problem_context(current.identity.source_context);
  current.control_prediction_origin_sec =
    previous_snapshot.control_prediction_origin_sec + 0.025;
  current.course_progress_origin_m = 50.4;
  current.request.initial_state << 0.2, -0.1, 0.05, 2.3, 0.0;

  const auto adapted = adapter::build(
    current.request,
    multi_purpose_mpc_ros::persistent_osqp::PhysicalConstraintTolerance{
      1.0e-4, 1.0e-4});
  ASSERT_TRUE(adapted.has_value());
  const int horizon = current.request.horizon_steps;
  const int state_values =
    multi_purpose_mpc_ros::mpcc_rate_resolved::kStateDimension *
    (horizon + 1);
  const int variable_count = state_values +
    multi_purpose_mpc_ros::mpcc_rate_resolved::kInputDimension * horizon;
  Eigen::VectorXd previous_primal(variable_count);
  for (int index = 0; index < variable_count; ++index) {
    previous_primal[index] = 0.01 * static_cast<double>(index + 1);
  }
  for (int stage = 0; stage <= horizon; ++stage) {
    previous_primal[
      stage * multi_purpose_mpc_ros::mpcc_rate_resolved::kStateDimension +
      multi_purpose_mpc_ros::mpcc_rate_resolved::kProgressIndex] =
      0.3 * stage;
  }
  shadow::RecedingWarmStartSeed previous{
    previous_snapshot.identity,
    previous_snapshot.control_prediction_origin_sec,
    previous_snapshot.course_progress_origin_m,
    {0.10, 0.10, 0.10},
    previous_primal};

  const auto resolution = shadow::resolve_receding_warm_start(
    previous, current, adapted->problem, 123U);

  ASSERT_EQ(resolution.reason, shadow::RecedingWarmStartReason::Available);
  ASSERT_TRUE(resolution.warm_start.has_value());
  EXPECT_EQ(resolution.stage_advance, 0U);
  EXPECT_EQ(resolution.warm_start->dual.size(), 123);
  EXPECT_TRUE(resolution.warm_start->dual.isZero());
  EXPECT_TRUE(
    resolution.warm_start->primal.head<
      multi_purpose_mpc_ros::mpcc_rate_resolved::kStateDimension>().isApprox(
      adapted->problem.initial_state));
  EXPECT_NEAR(
    resolution.warm_start->primal[
      multi_purpose_mpc_ros::mpcc_rate_resolved::kStateDimension +
      multi_purpose_mpc_ros::mpcc_rate_resolved::kProgressIndex],
    -0.1, 1.0e-12);
  EXPECT_DOUBLE_EQ(
    resolution.warm_start->primal[state_values],
    previous_primal[state_values]);
}

TEST(MpccRateResolvedShadow, RecedingWarmStartShiftsOneCompleteStage)
{
  const auto previous_snapshot = snapshot(1U);
  auto current = snapshot(2U);
  current.identity.source_context.intent_generation =
    previous_snapshot.identity.source_context.intent_generation;
  current.identity.source_context.stage_geometry_id = 999U;
  current.identity.source_context =
    contract::seal_problem_context(current.identity.source_context);
  current.control_prediction_origin_sec =
    previous_snapshot.control_prediction_origin_sec + 0.11;
  const auto adapted = adapter::build(
    current.request,
    multi_purpose_mpc_ros::persistent_osqp::PhysicalConstraintTolerance{
      1.0e-4, 1.0e-4});
  ASSERT_TRUE(adapted.has_value());
  const int horizon = current.request.horizon_steps;
  const int state_values =
    multi_purpose_mpc_ros::mpcc_rate_resolved::kStateDimension *
    (horizon + 1);
  const int variable_count = state_values +
    multi_purpose_mpc_ros::mpcc_rate_resolved::kInputDimension * horizon;
  Eigen::VectorXd previous_primal(variable_count);
  for (int index = 0; index < variable_count; ++index) {
    previous_primal[index] = static_cast<double>(index + 1);
  }
  shadow::RecedingWarmStartSeed previous{
    previous_snapshot.identity,
    previous_snapshot.control_prediction_origin_sec,
    previous_snapshot.course_progress_origin_m,
    {0.10, 0.10, 0.10},
    previous_primal};

  const auto resolution = shadow::resolve_receding_warm_start(
    previous, current, adapted->problem, 80U);

  ASSERT_EQ(resolution.reason, shadow::RecedingWarmStartReason::Available);
  ASSERT_TRUE(resolution.warm_start.has_value());
  EXPECT_EQ(resolution.stage_advance, 1U);
  EXPECT_DOUBLE_EQ(
    resolution.warm_start->primal[state_values],
    previous_primal[
      state_values + multi_purpose_mpc_ros::mpcc_rate_resolved::kInputDimension]);
  EXPECT_DOUBLE_EQ(
    resolution.warm_start->primal[
      2 * multi_purpose_mpc_ros::mpcc_rate_resolved::kStateDimension +
      multi_purpose_mpc_ros::mpcc_rate_resolved::kLateralIndex],
    previous_primal[
      3 * multi_purpose_mpc_ros::mpcc_rate_resolved::kStateDimension +
      multi_purpose_mpc_ros::mpcc_rate_resolved::kLateralIndex]);
}

TEST(MpccRateResolvedShadow, RecedingWarmStartResizesAcrossPlanningHorizons)
{
  auto previous_snapshot = snapshot(1U);
  previous_snapshot.identity.source_context.intent = contract::ControlIntent::Pass;
  previous_snapshot.identity.source_context.target_id = "d2";
  previous_snapshot.identity.source_context.target_obstacle_generation = 5U;
  previous_snapshot.identity.source_context.execution_side_sign = -1;
  previous_snapshot.identity.source_context =
    contract::seal_problem_context(previous_snapshot.identity.source_context);

  auto current = snapshot(2U);
  current.request = straight_request(5);
  current.execution_prefix_steps = 2;
  current.nominal_path_distance_m = {0.0, 0.2, 0.4, 0.6, 0.8, 1.0};
  current.identity.source_context = previous_snapshot.identity.source_context;
  current.identity.source_context.decision_id += 1U;
  current.identity.source_context.stage_geometry_id += 1U;
  current.identity.source_context.horizon_steps = 5U;
  current.identity.source_context =
    contract::seal_problem_context(current.identity.source_context);
  current.control_prediction_origin_sec =
    previous_snapshot.control_prediction_origin_sec + 0.025;

  const auto adapted = adapter::build(
    current.request,
    multi_purpose_mpc_ros::persistent_osqp::PhysicalConstraintTolerance{
      1.0e-4, 1.0e-4});
  ASSERT_TRUE(adapted.has_value());
  constexpr int previous_horizon = 3;
  const int previous_state_values =
    multi_purpose_mpc_ros::mpcc_rate_resolved::kStateDimension *
    (previous_horizon + 1);
  const int previous_variable_count = previous_state_values +
    multi_purpose_mpc_ros::mpcc_rate_resolved::kInputDimension *
    previous_horizon;
  Eigen::VectorXd previous_primal(previous_variable_count);
  for (int index = 0; index < previous_variable_count; ++index) {
    previous_primal[index] = static_cast<double>(index + 1);
  }
  shadow::RecedingWarmStartSeed previous{
    previous_snapshot.identity,
    previous_snapshot.control_prediction_origin_sec,
    previous_snapshot.course_progress_origin_m,
    {0.10, 0.10, 0.10},
    previous_primal};

  const auto resolution = shadow::resolve_receding_warm_start(
    previous, current, adapted->problem, 140U);

  ASSERT_EQ(resolution.reason, shadow::RecedingWarmStartReason::Available);
  ASSERT_TRUE(resolution.warm_start.has_value());
  const int current_state_values =
    multi_purpose_mpc_ros::mpcc_rate_resolved::kStateDimension * 6;
  const int current_variable_count = current_state_values +
    multi_purpose_mpc_ros::mpcc_rate_resolved::kInputDimension * 5;
  EXPECT_EQ(resolution.warm_start->primal.size(), current_variable_count);
  EXPECT_TRUE(
    resolution.warm_start->primal.head<
      multi_purpose_mpc_ros::mpcc_rate_resolved::kStateDimension>().isApprox(
      adapted->problem.initial_state));
  EXPECT_DOUBLE_EQ(
    resolution.warm_start->primal[current_state_values],
    previous_primal[previous_state_values]);
  EXPECT_DOUBLE_EQ(
    resolution.warm_start->primal[
      current_state_values +
      4 * multi_purpose_mpc_ros::mpcc_rate_resolved::kInputDimension],
    previous_primal[
      previous_state_values +
      2 * multi_purpose_mpc_ros::mpcc_rate_resolved::kInputDimension]);
}

TEST(MpccRateResolvedShadow, RecedingWarmStartRejectsTacticalIdentityChanges)
{
  auto previous_snapshot = snapshot(1U);
  previous_snapshot.identity.source_context.intent = contract::ControlIntent::Pass;
  previous_snapshot.identity.source_context.target_id = "d2";
  previous_snapshot.identity.source_context.target_obstacle_generation = 5U;
  previous_snapshot.identity.source_context.execution_side_sign = 1;
  previous_snapshot.identity.source_context =
    contract::seal_problem_context(previous_snapshot.identity.source_context);
  auto current = snapshot(2U);
  current.identity.source_context = previous_snapshot.identity.source_context;
  current.identity.source_context.decision_id += 1U;
  current.identity.source_context.target_id = "d3";
  current.identity.source_context =
    contract::seal_problem_context(current.identity.source_context);
  const auto adapted = adapter::build(
    current.request,
    multi_purpose_mpc_ros::persistent_osqp::PhysicalConstraintTolerance{
      1.0e-4, 1.0e-4});
  ASSERT_TRUE(adapted.has_value());
  const int horizon = current.request.horizon_steps;
  const int variable_count =
    multi_purpose_mpc_ros::mpcc_rate_resolved::kStateDimension * (horizon + 1) +
    multi_purpose_mpc_ros::mpcc_rate_resolved::kInputDimension * horizon;
  shadow::RecedingWarmStartSeed previous{
    previous_snapshot.identity,
    previous_snapshot.control_prediction_origin_sec,
    previous_snapshot.course_progress_origin_m,
    {0.10, 0.10, 0.10},
    Eigen::VectorXd::Zero(variable_count)};

  const auto target_changed = shadow::resolve_receding_warm_start(
    previous, current, adapted->problem, 80U);
  EXPECT_EQ(
    target_changed.reason,
    shadow::RecedingWarmStartReason::SemanticMismatch);
  EXPECT_EQ(target_changed.diagnostic, "target-id");
  EXPECT_FALSE(target_changed.warm_start.has_value());

  current.identity.source_context.target_id = "d2";
  current.identity.source_context.execution_side_sign = -1;
  current.identity.source_context =
    contract::seal_problem_context(current.identity.source_context);
  const auto side_changed = shadow::resolve_receding_warm_start(
    previous, current, adapted->problem, 80U);
  EXPECT_EQ(
    side_changed.reason,
    shadow::RecedingWarmStartReason::SemanticMismatch);
  EXPECT_EQ(side_changed.diagnostic, "execution-side");

  current.identity.source_context.execution_side_sign = 1;
  current.identity.source_context.intent_generation += 1U;
  current.identity.source_context =
    contract::seal_problem_context(current.identity.source_context);
  const auto generation_changed = shadow::resolve_receding_warm_start(
    previous, current, adapted->problem, 80U);
  EXPECT_EQ(
    generation_changed.reason,
    shadow::RecedingWarmStartReason::SemanticMismatch);
  EXPECT_EQ(generation_changed.diagnostic, "intent-generation");
}

TEST(MpccRateResolvedShadow, RecedingWarmStartRejectsExhaustedHorizon)
{
  const auto previous_snapshot = snapshot(1U);
  auto current = snapshot(2U);
  current.identity.source_context.intent_generation =
    previous_snapshot.identity.source_context.intent_generation;
  current.identity.source_context =
    contract::seal_problem_context(current.identity.source_context);
  current.control_prediction_origin_sec =
    previous_snapshot.control_prediction_origin_sec + 0.31;
  const auto adapted = adapter::build(
    current.request,
    multi_purpose_mpc_ros::persistent_osqp::PhysicalConstraintTolerance{
      1.0e-4, 1.0e-4});
  ASSERT_TRUE(adapted.has_value());
  const int horizon = current.request.horizon_steps;
  const int variable_count =
    multi_purpose_mpc_ros::mpcc_rate_resolved::kStateDimension * (horizon + 1) +
    multi_purpose_mpc_ros::mpcc_rate_resolved::kInputDimension * horizon;
  shadow::RecedingWarmStartSeed previous{
    previous_snapshot.identity,
    previous_snapshot.control_prediction_origin_sec,
    previous_snapshot.course_progress_origin_m,
    {0.10, 0.10, 0.10},
    Eigen::VectorXd::Zero(variable_count)};

  const auto resolution = shadow::resolve_receding_warm_start(
    previous, current, adapted->problem, 80U);
  EXPECT_EQ(
    resolution.reason,
    shadow::RecedingWarmStartReason::HorizonExhausted);
  EXPECT_FALSE(resolution.warm_start.has_value());
}

TEST(MpccRateResolvedShadow, TradesProgressForReachableOpponentSeparation)
{
  auto input = snapshot();
  input.dynamic_obstacle_refinement_active = true;
  input.dynamic_obstacle_pass_side_sign = 1;
  input.dynamic_obstacle_stages.assign(
    3U,
    multi_purpose_mpc_ros::mpcc_rate_resolved_dynamic_obstacle::StagePrediction{
      true, 0.8, 0.75, 0.20, 0.30});

  shadow::SolverContext solver;
  const auto result = solver.evaluate(input);
  EXPECT_EQ(result.outcome, shadow::Outcome::Solved) << result.detail;
  EXPECT_TRUE(result.dynamic_obstacle_refinement_requested);
  EXPECT_TRUE(result.dynamic_obstacle_refinement_applied);
  EXPECT_TRUE(result.dynamic_obstacle_refinement_solved);
  EXPECT_TRUE(result.post_refinement_physical_proof_checked);
  EXPECT_TRUE(result.post_refinement_physical_proof_accepted);
  EXPECT_EQ(
    result.post_refinement_linearization_requested,
    result.post_refinement_linearization_count > 0U);
  if (result.post_refinement_linearization_requested) {
    EXPECT_TRUE(result.post_refinement_linearization_applied);
    EXPECT_TRUE(result.post_refinement_linearization_bootstrap_applied);
    EXPECT_TRUE(result.post_refinement_linearization_solved);
  }
  EXPECT_GT(result.dynamic_obstacle_stay_behind_row_count, 0U);
  EXPECT_TRUE(shadow::result_valid(result));
}

TEST(
  MpccRateResolvedShadow,
  SolvesCompletePlanningHorizonButPublishesOnlyCertifiedPrefix)
{
  auto input = snapshot();
  input.execution_prefix_steps = 2;

  shadow::SolverContext context;
  const auto result = context.evaluate(input);

  ASSERT_EQ(result.outcome, shadow::Outcome::Solved) << result.detail;
  ASSERT_NE(result.execution_artifact, nullptr);
  EXPECT_EQ(result.planning_stage_count, 3U);
  EXPECT_EQ(result.certified_stage_count, 2U);
  EXPECT_EQ(result.execution_artifact->control_stages.size(), 2U);
  EXPECT_EQ(result.execution_artifact->predicted_states.size(), 3U);
  EXPECT_EQ(result.execution_artifact->nominal_path_distance_m.size(), 3U);
  EXPECT_EQ(result.execution_artifact->lateral_lower_m.size(), 3U);
  EXPECT_EQ(result.execution_artifact->lateral_upper_m.size(), 3U);
  EXPECT_NEAR(result.certified_horizon_duration_sec, 0.20, 1e-12);
  EXPECT_TRUE(shadow::result_valid(result));
}

TEST(MpccRateResolvedShadow, RefinesWallRowsAtTheSolvedProgress)
{
  shadow::SolverContext context;
  auto input = snapshot();
  input.progress_aligned_wall_refinement_active = true;
  input.wall_reference_progress_m = {0.0, 0.3, 0.6, 1.0};
  input.wall_lower_m = {-1.0, -0.8, -0.6, -0.4};
  input.wall_upper_m = {1.0, 0.9, 0.8, 0.7};

  const auto result = context.evaluate(input);
  ASSERT_EQ(result.outcome, shadow::Outcome::Solved) << result.detail;
  EXPECT_TRUE(result.progress_wall_refinement_requested);
  EXPECT_TRUE(result.progress_wall_refinement_applied);
  EXPECT_TRUE(result.progress_wall_refinement_solved);
  EXPECT_EQ(
    result.progress_wall_refinement_reason,
    multi_purpose_mpc_ros::mpcc_progress::
    ProgressAlignedWallBoundsReason::Accepted);
  ASSERT_NE(result.execution_artifact, nullptr);
  EXPECT_GT(result.progress_wall_refinement_aligned_stage_count, 0U);
}

TEST(MpccRateResolvedShadow, RejectsWhenRequestedWallRefinementCannotBeBuilt)
{
  shadow::SolverContext context;
  auto input = snapshot();
  input.progress_aligned_wall_refinement_active = true;
  // The first QP is deliberately allowed to solve without wall rows. A
  // malformed physical-wall profile must not let that provisional solution
  // escape as a production artifact.
  input.wall_reference_progress_m = {0.0};
  input.wall_lower_m = {-1.0};
  input.wall_upper_m = {1.0};

  const auto result = context.evaluate(input);
  EXPECT_EQ(result.outcome, shadow::Outcome::AssemblyRejected);
  EXPECT_FALSE(result.solved);
  EXPECT_TRUE(result.progress_wall_refinement_requested);
  EXPECT_FALSE(result.progress_wall_refinement_applied);
  EXPECT_EQ(
    result.progress_wall_refinement_reason,
    multi_purpose_mpc_ros::mpcc_progress::
    ProgressAlignedWallBoundsReason::InvalidInput);
  EXPECT_EQ(result.execution_artifact, nullptr);
  EXPECT_NE(result.detail.find("wall refinement rejected"), std::string::npos);
}

TEST(MpccRateResolvedShadow, PhysicalWallRefinementRetainsHeadingInCertificate)
{
  const auto grid = corridor_grid();
  auto aligned_request = wall_request(grid, 0.0);
  auto rotated_request = wall_request(grid, 0.5);
  aligned_request.stages.front().lag_lower_m =
    -std::numeric_limits<double>::infinity();
  aligned_request.stages.front().lag_upper_m =
    std::numeric_limits<double>::infinity();
  aligned_request.stages.front().heading_lower_rad =
    -std::numeric_limits<double>::infinity();
  aligned_request.stages.front().heading_upper_rad =
    std::numeric_limits<double>::infinity();
  rotated_request.stages.front().lag_lower_m =
    -std::numeric_limits<double>::infinity();
  rotated_request.stages.front().lag_upper_m =
    std::numeric_limits<double>::infinity();
  rotated_request.stages.front().heading_lower_rad =
    -std::numeric_limits<double>::infinity();
  rotated_request.stages.front().heading_upper_rad =
    std::numeric_limits<double>::infinity();
  const auto aligned = wall_refinement::resolve(aligned_request);
  const auto rotated = wall_refinement::resolve(rotated_request);

  ASSERT_TRUE(aligned.applied) << aligned.detail;
  ASSERT_TRUE(rotated.applied) << rotated.detail;
  ASSERT_EQ(aligned.stages.size(), 1U);
  ASSERT_EQ(rotated.stages.size(), 1U);
  EXPECT_EQ(aligned.swept_lateral_constraints.size(), 4U);
  EXPECT_EQ(rotated.swept_lateral_constraints.size(), 4U);
  EXPECT_LT(
    rotated.stages.front().lateral_upper_m,
    aligned.stages.front().lateral_upper_m - 0.2);
  EXPECT_LE(
    rotated.stages.front().heading_lower_rad, 0.5 + 1e-12);
  EXPECT_GE(
    rotated.stages.front().heading_upper_rad, 0.5 - 1e-12);
  EXPECT_LT(
    rotated.stages.front().heading_upper_rad -
    rotated.stages.front().heading_lower_rad,
    0.026);
  EXPECT_LT(
    rotated.stages.front().lag_upper_m -
    rotated.stages.front().lag_lower_m,
    0.051);
  EXPECT_LT(
    rotated.stages.front().progress_upper_m -
    rotated.stages.front().progress_lower_m,
    0.051);
}

TEST(MpccRateResolvedShadow, RejectsNoncanonicalProblemIdentity)
{
  shadow::SolverContext context;
  auto input = snapshot();
  input.identity.source_context.formulation =
    contract::Formulation::SolverDerivedBypass;
  const auto result = context.evaluate(input);
  EXPECT_EQ(result.outcome, shadow::Outcome::BuildRejected);
  EXPECT_FALSE(shadow::result_valid(result));
}

TEST(MpccRateResolvedShadow, SupportsEveryRateResolvedNormalIntent)
{
  for (const auto intent : {
      contract::ControlIntent::Track,
      contract::ControlIntent::Cruise,
      contract::ControlIntent::ShiftOut,
      contract::ControlIntent::Pass,
      contract::ControlIntent::Return})
  {
    shadow::SolverContext context;
    auto input = snapshot(static_cast<std::uint64_t>(intent) + 10U);
    auto source = input.identity.source_context;
    source.intent = intent;
    const bool overtake_intent =
      contract::canonical_normal_intent_requires_execution_side(intent);
    source.target_id = overtake_intent ? "d2" : "";
    source.target_obstacle_generation =
      overtake_intent ? source.observation_generation : 0U;
    source.execution_side_sign = overtake_intent ? 1 : 0;
    input.identity.source_context =
      contract::seal_problem_context(std::move(source));

    const auto result = context.evaluate(input);
    EXPECT_EQ(result.outcome, shadow::Outcome::Solved)
      << contract::to_string(intent) << ": " << result.detail;
    ASSERT_NE(result.execution_artifact, nullptr)
      << contract::to_string(intent);
    EXPECT_TRUE(execution::identity_valid(result.execution_artifact->identity))
      << contract::to_string(intent);
  }
}

TEST(MpccRateResolvedShadow, RetainsAndSamplesExactRateResolvedArtifact)
{
  shadow::SolverContext context;
  const auto result = context.evaluate(snapshot());
  ASSERT_EQ(result.outcome, shadow::Outcome::Solved) << result.detail;
  ASSERT_NE(result.execution_artifact, nullptr);
  const auto & artifact = *result.execution_artifact;

  const auto cursor = execution::resolve_cursor(
    artifact, artifact.prediction_origin_sec + 0.15);
  ASSERT_TRUE(cursor.available);
  EXPECT_EQ(cursor.reason, execution::CursorReason::Available);
  EXPECT_EQ(cursor.control_stage_index, 1U);
  EXPECT_EQ(cursor.remaining_control_stage_count, 2U);
  EXPECT_NEAR(cursor.stage_elapsed_sec, 0.05, 1e-12);

  const auto actuation = execution::extract_actuation(
    artifact, cursor);
  ASSERT_TRUE(actuation.actuation.has_value());
  EXPECT_EQ(actuation.reason, execution::ActuationReason::Available);
  EXPECT_EQ(
    actuation.sample_reason,
    multi_purpose_mpc_ros::mpcc_rate_resolved::ActuationSampleReason::Accepted);
  const double expected_steering =
    artifact.semantic_initial_steering_rad +
    artifact.control_stages[0].steering_rate_radps *
    artifact.control_stages[0].duration_sec +
    artifact.control_stages[1].steering_rate_radps *
    0.05;
  EXPECT_NEAR(
    actuation.actuation->steering_rad, expected_steering, 1e-9);
  EXPECT_DOUBLE_EQ(
    actuation.actuation->steering_rate_radps,
    artifact.control_stages[1].steering_rate_radps);

  const auto exhausted = execution::resolve_cursor(
    artifact, artifact.prediction_origin_sec + 0.30);
  EXPECT_FALSE(exhausted.available);
  EXPECT_EQ(exhausted.reason, execution::CursorReason::Exhausted);
}

TEST(
  MpccRateResolvedShadow,
  FreshArtifactPublishesFromTheCertifiedPhysicalState)
{
  shadow::SolverContext context;
  const auto result = context.evaluate(snapshot());
  ASSERT_EQ(result.outcome, shadow::Outcome::Solved) << result.detail;
  ASSERT_NE(result.execution_artifact, nullptr);

  const auto & artifact = *result.execution_artifact;
  const auto cursor = execution::resolve_cursor(
    artifact, artifact.prediction_origin_sec);
  ASSERT_TRUE(cursor.available);
  ASSERT_DOUBLE_EQ(cursor.elapsed_sec, 0.0);

  const auto actuation = execution::extract_actuation(
    artifact, cursor);
  ASSERT_TRUE(actuation.actuation.has_value());
  EXPECT_NEAR(
    actuation.actuation->steering_rad,
    artifact.semantic_initial_steering_rad,
    1e-9);
}

TEST(MpccRateResolvedShadow, ArtifactRejectsInvalidPublicationTiming)
{
  shadow::SolverContext context;
  const auto result = context.evaluate(snapshot());
  ASSERT_EQ(result.outcome, shadow::Outcome::Solved) << result.detail;
  ASSERT_NE(result.execution_artifact, nullptr);

  auto missing_interval = *result.execution_artifact;
  missing_interval.publication_interval_sec = 0.0;
  EXPECT_EQ(
    execution::validate(missing_interval),
    execution::RejectReason::InvalidTiming);

  auto beyond_horizon = *result.execution_artifact;
  beyond_horizon.publication_interval_sec = 1.0;
  EXPECT_EQ(
    execution::validate(beyond_horizon),
    execution::RejectReason::InvalidTiming);
}

TEST(MpccRateResolvedShadow, RejectsMutatedExecutionArtifactProvenance)
{
  shadow::SolverContext context;
  const auto result = context.evaluate(snapshot());
  ASSERT_EQ(result.outcome, shadow::Outcome::Solved) << result.detail;
  ASSERT_NE(result.execution_artifact, nullptr);

  auto invalid = *result.execution_artifact;
  invalid.identity.source_context.fingerprint = 0U;
  EXPECT_EQ(
    execution::validate(invalid), execution::RejectReason::InvalidIdentity);

  invalid = *result.execution_artifact;
  invalid.course_progress_origin_m =
    std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(
    execution::validate(invalid),
    execution::RejectReason::InvalidCourseProgressOrigin);

  invalid = *result.execution_artifact;
  invalid.predicted_states.pop_back();
  EXPECT_EQ(
    execution::validate(invalid), execution::RejectReason::StateCountMismatch);

  invalid = *result.execution_artifact;
  invalid.nominal_path_distance_m.pop_back();
  EXPECT_EQ(
    execution::validate(invalid),
    execution::RejectReason::PathDistanceCountMismatch);

  invalid = *result.execution_artifact;
  invalid.nominal_path_distance_m[1] = 0.0;
  EXPECT_EQ(
    execution::validate(invalid), execution::RejectReason::InvalidPathDistance);

  invalid = *result.execution_artifact;
  invalid.predicted_states.front().steering_rad += 0.1;
  EXPECT_EQ(
    execution::validate(invalid), execution::RejectReason::InitialSteeringMismatch);

  invalid = *result.execution_artifact;
  invalid.predicted_states[1].steering_rad += 0.1;
  EXPECT_EQ(
    execution::validate(invalid), execution::RejectReason::SteeringDynamicsMismatch);

  invalid = *result.execution_artifact;
  invalid.lateral_upper_m[1] = invalid.predicted_states[1].lateral_m - 0.1;
  EXPECT_EQ(
    execution::validate(invalid), execution::RejectReason::InvalidLateralCorridor);

  invalid = *result.execution_artifact;
  invalid.maximum_normalized_constraint_violation = 1.01;
  EXPECT_EQ(
    execution::validate(invalid), execution::RejectReason::InvalidCertificate);
}

TEST(MpccRateResolvedShadow, SamplesPublicationPeriodAcrossStageBoundary)
{
  shadow::SolverContext context;
  auto input = snapshot();
  input.publication_interval_sec = 0.20;
  const auto result = context.evaluate(input);
  ASSERT_EQ(result.outcome, shadow::Outcome::Solved) << result.detail;
  EXPECT_TRUE(shadow::result_valid(result));
  EXPECT_EQ(result.sampled_stage_index, 1U);
  EXPECT_NEAR(result.sampled_stage_elapsed_sec, 0.10, 1e-12);
  EXPECT_NEAR(result.certified_horizon_duration_sec, 0.30, 1e-12);
}

TEST(MpccRateResolvedShadow, RejectsPublicationPeriodBeyondCertifiedHorizon)
{
  shadow::SolverContext context;
  auto input = snapshot();
  input.publication_interval_sec = 0.31;
  const auto result = context.evaluate(input);
  EXPECT_EQ(result.outcome, shadow::Outcome::ActuationSampleRejected);
  EXPECT_EQ(
    result.actuation_sample_reason,
    multi_purpose_mpc_ros::mpcc_rate_resolved::ActuationSampleReason::
    PublicationAfterHorizonEnd);
  EXPECT_DOUBLE_EQ(result.publication_interval_sec, 0.31);
  EXPECT_DOUBLE_EQ(result.first_stage_duration_sec, 0.10);
  EXPECT_NEAR(result.certified_horizon_duration_sec, 0.30, 1e-12);
  EXPECT_TRUE(shadow::result_valid(result));
}

TEST(MpccRateResolvedShadowMailbox, PublishesMonotonicRegisteredResults)
{
  shadow::SolverContext context;
  shadow::Mailbox mailbox;
  ASSERT_TRUE(mailbox.register_submission(1U));
  const auto first = context.evaluate(snapshot(1U));
  EXPECT_EQ(mailbox.publish(first), shadow::PublishReason::Accepted);
  const auto available = mailbox.latest_after(0U);
  ASSERT_TRUE(available.has_value());
  EXPECT_EQ(available->identity.sequence, 1U);
  EXPECT_FALSE(mailbox.latest_after(1U).has_value());

  ASSERT_TRUE(mailbox.register_submission(2U));
  const auto second = context.evaluate(snapshot(2U));
  EXPECT_EQ(mailbox.publish(second), shadow::PublishReason::Accepted);
  EXPECT_EQ(
    mailbox.publish(first), shadow::PublishReason::SequenceRollback);
  const auto state = mailbox.state();
  EXPECT_EQ(state.accepted_count, 2U);
  EXPECT_EQ(state.sequence_rollback_count, 1U);
}

TEST(MpccRateResolvedShadowMailbox, RejectsUnregisteredAndInvalidResults)
{
  shadow::SolverContext context;
  shadow::Mailbox mailbox;
  const auto result = context.evaluate(snapshot(2U));
  EXPECT_EQ(
    mailbox.publish(result), shadow::PublishReason::SequenceNotSubmitted);

  ASSERT_TRUE(mailbox.register_submission(3U));
  auto invalid = context.evaluate(snapshot(3U));
  invalid.identity.source_context.fingerprint = 0U;
  EXPECT_EQ(mailbox.publish(invalid), shadow::PublishReason::InvalidResult);
}
