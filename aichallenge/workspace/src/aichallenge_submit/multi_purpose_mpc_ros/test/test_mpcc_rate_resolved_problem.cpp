#include "multi_purpose_mpc_ros/mpcc_rate_resolved_problem.hpp"

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <limits>

namespace problem = multi_purpose_mpc_ros::mpcc_rate_resolved_problem;
namespace model = multi_purpose_mpc_ros::mpcc_rate_resolved;
namespace solver = multi_purpose_mpc_ros::persistent_osqp;

namespace
{

problem::AssemblyRequest straight_request(const int horizon = 3)
{
  problem::AssemblyRequest request;
  request.horizon_steps = horizon;
  request.initial_state << 0.0, 0.0, 0.0, 2.0, 0.0, 0.10, 0.08;
  request.linearizations.reserve(static_cast<std::size_t>(horizon));
  for (int stage = 0; stage < horizon; ++stage) {
    model::LinearizationRequest linearization_request;
    linearization_request.reference_velocity_mps = 2.0;
    linearization_request.reference_progress_m = 0.2 * stage;
    linearization_request.reference_steering_rad = 0.10;
    linearization_request.reference_response_steering_rad = 0.08;
    linearization_request.reference_virtual_progress_speed_mps = 2.0;
    linearization_request.wheelbase_m = 2.0;
    linearization_request.yaw_response_gain = 0.75;
    linearization_request.yaw_response_time_constant_sec = 0.13;
    linearization_request.stage_dt_sec = 0.10;
    const auto linearization = model::linearize_temporal_frenet(
      linearization_request);
    EXPECT_TRUE(linearization.has_value());
    request.linearizations.push_back(linearization.value());
  }
  const int state_values = model::kStateDimension * (horizon + 1);
  const int input_values = model::kInputDimension * horizon;
  request.state_reference = Eigen::VectorXd::Zero(state_values);
  request.state_lower = Eigen::VectorXd::Constant(state_values, -100.0);
  request.state_upper = Eigen::VectorXd::Constant(state_values, 100.0);
  request.state_weight = Eigen::VectorXd::Constant(state_values, 1.0);
  for (int stage = 0; stage <= horizon; ++stage) {
    const int offset = model::kStateDimension * stage;
    request.state_reference[offset + model::kVelocityIndex] = 2.0;
    request.state_reference[offset + model::kProgressIndex] = 0.2 * stage;
    request.state_reference[offset + model::kSteeringIndex] = 0.10;
    request.state_reference[offset + model::kResponseSteeringIndex] = 0.08;
    request.state_lower[offset + model::kVelocityIndex] = 0.0;
    request.state_lower[offset + model::kSteeringIndex] = -0.6;
    request.state_upper[offset + model::kSteeringIndex] = 0.6;
    request.state_lower[offset + model::kResponseSteeringIndex] = -0.6;
    request.state_upper[offset + model::kResponseSteeringIndex] = 0.6;
  }
  request.input_reference = Eigen::VectorXd::Zero(input_values);
  request.input_lower = Eigen::VectorXd::Zero(input_values);
  request.input_upper = Eigen::VectorXd::Zero(input_values);
  request.input_weight = Eigen::VectorXd::Constant(input_values, 1.0);
  for (int stage = 0; stage < horizon; ++stage) {
    const int offset = model::kInputDimension * stage;
    request.input_reference[offset + model::kVirtualProgressSpeedIndex] = 2.0;
    request.input_lower[offset + model::kAccelerationIndex] = -1.0;
    request.input_upper[offset + model::kAccelerationIndex] = 1.0;
    request.input_lower[offset + model::kSteeringRateIndex] = -0.7;
    request.input_upper[offset + model::kSteeringRateIndex] = 0.7;
    request.input_lower[offset + model::kVirtualProgressSpeedIndex] = 0.0;
    request.input_upper[offset + model::kVirtualProgressSpeedIndex] = 4.0;
  }
  request.previous_input << 0.0, 0.0, 2.0;
  request.input_delta_weight << 0.2, 0.3, 0.1;
  return request;
}

}  // namespace

TEST(MpccRateResolvedProblem, ResolvesPreviousInputFromSerializedPublication)
{
  const auto previous_input = problem::resolve_serialized_previous_input(
    problem::SerializedPreviousInputRequest{
      0.75, 4.0, 0.02, 0.05, 0.025});

  ASSERT_TRUE(previous_input.has_value());
  EXPECT_DOUBLE_EQ((*previous_input)[model::kAccelerationIndex], 0.75);
  EXPECT_NEAR(
    (*previous_input)[model::kSteeringRateIndex], 1.2, 1e-12);
  EXPECT_DOUBLE_EQ(
    (*previous_input)[model::kVirtualProgressSpeedIndex], 4.0);
}

TEST(MpccRateResolvedProblem, RejectsIncompleteSerializedPublicationPredecessor)
{
  EXPECT_FALSE(problem::resolve_serialized_previous_input(
      problem::SerializedPreviousInputRequest{
        0.75, 2.0, 0.02, 0.05, 0.0}).has_value());
  EXPECT_FALSE(problem::resolve_serialized_previous_input(
      problem::SerializedPreviousInputRequest{
        std::numeric_limits<double>::quiet_NaN(), 2.0, 0.02, 0.05,
        0.025}).has_value());
}

TEST(MpccRateResolvedProblem, AssemblesExactVariableAndRowLayout)
{
  const auto assembled = problem::assemble(straight_request());
  ASSERT_TRUE(assembled.has_value());
  constexpr int horizon = 3;
  constexpr int state_values = model::kStateDimension * (horizon + 1);
  constexpr int input_values = model::kInputDimension * horizon;
  constexpr int variable_count = state_values + input_values;
  EXPECT_EQ(assembled->quadratic_cost.rows(), variable_count);
  EXPECT_EQ(assembled->constraints.cols(), variable_count);
  EXPECT_EQ(assembled->constraints.rows(), state_values + variable_count);
  EXPECT_EQ(assembled->lower_bound.size(), assembled->constraints.rows());
  EXPECT_EQ(assembled->upper_bound.size(), assembled->constraints.rows());
}

TEST(MpccRateResolvedProblem, AddsOneExactDesiredSteeringPrefixRowPerStage)
{
  auto request = straight_request();
  request.steering_rate_prefix_bounds = problem::SteeringRatePrefixBounds{
    -1.15, 0.05};
  const auto assembled = problem::assemble(request);
  ASSERT_TRUE(assembled.has_value());
  constexpr int horizon = 3;
  constexpr int state_values = model::kStateDimension * (horizon + 1);
  constexpr int input_values = model::kInputDimension * horizon;
  constexpr int variable_count = state_values + input_values;
  EXPECT_EQ(
    assembled->constraints.rows(),
    state_values + variable_count + horizon);
  const int prefix_offset = state_values + variable_count;
  for (int stage = 0; stage < horizon; ++stage) {
    EXPECT_DOUBLE_EQ(assembled->lower_bound[prefix_offset + stage], -1.15);
    EXPECT_DOUBLE_EQ(assembled->upper_bound[prefix_offset + stage], 0.05);
    const auto semantic = problem::decode_row(
      prefix_offset + stage, horizon);
    ASSERT_TRUE(semantic.valid);
    EXPECT_EQ(semantic.kind, problem::RowKind::SteeringRatePrefix);
    EXPECT_EQ(semantic.stage, stage);
    for (int prefix_stage = 0; prefix_stage < horizon; ++prefix_stage) {
      const double expected = prefix_stage <= stage ? 0.10 : 0.0;
      EXPECT_DOUBLE_EQ(
        assembled->constraints.coeff(
          prefix_offset + stage,
          state_values + prefix_stage * model::kInputDimension +
          model::kSteeringRateIndex),
        expected);
    }
  }
}

TEST(MpccRateResolvedProblem, CouplesPhysicalWallBoundsToOptimizedProgress)
{
  auto request = straight_request();
  request.steering_rate_prefix_bounds = problem::SteeringRatePrefixBounds{
    -1.15, 0.05};
  problem::ProgressAlignedWallConstraints wall;
  wall.lower_slope = {0.10, -0.20, 0.30};
  wall.lower_intercept = {-1.0, -0.8, -1.2};
  wall.upper_slope = {-0.05, 0.15, -0.25};
  wall.upper_intercept = {1.0, 0.9, 1.1};
  request.progress_aligned_wall_constraints = wall;

  const auto assembled = problem::assemble(request);
  ASSERT_TRUE(assembled.has_value());
  constexpr int horizon = 3;
  constexpr int state_values = model::kStateDimension * (horizon + 1);
  constexpr int input_values = model::kInputDimension * horizon;
  constexpr int variable_count = state_values + input_values;
  const int wall_offset = state_values + variable_count + horizon;
  EXPECT_EQ(assembled->constraints.rows(), wall_offset + 2 * horizon);
  for (int stage = 0; stage < horizon; ++stage) {
    const int state = (stage + 1) * model::kStateDimension;
    const int lower_row = wall_offset + 2 * stage;
    const int upper_row = lower_row + 1;
    EXPECT_DOUBLE_EQ(
      assembled->constraints.coeff(
        lower_row, state + model::kLateralIndex), 1.0);
    EXPECT_DOUBLE_EQ(
      assembled->constraints.coeff(
        lower_row, state + model::kProgressIndex),
      -wall.lower_slope[static_cast<std::size_t>(stage)]);
    EXPECT_DOUBLE_EQ(
      assembled->lower_bound[lower_row],
      wall.lower_intercept[static_cast<std::size_t>(stage)]);
    EXPECT_TRUE(std::isinf(assembled->upper_bound[lower_row]));
    EXPECT_DOUBLE_EQ(
      assembled->constraints.coeff(
        upper_row, state + model::kLateralIndex), 1.0);
    EXPECT_DOUBLE_EQ(
      assembled->constraints.coeff(
        upper_row, state + model::kProgressIndex),
      -wall.upper_slope[static_cast<std::size_t>(stage)]);
    EXPECT_TRUE(std::isinf(assembled->lower_bound[upper_row]));
    EXPECT_DOUBLE_EQ(
      assembled->upper_bound[upper_row],
      wall.upper_intercept[static_cast<std::size_t>(stage)]);
    const auto lower_semantic = problem::decode_row(
      lower_row, horizon, true, true);
    ASSERT_TRUE(lower_semantic.valid);
    EXPECT_EQ(
      lower_semantic.kind,
      problem::RowKind::ProgressAlignedWallLower);
    EXPECT_EQ(lower_semantic.stage, stage);
  }
}

TEST(MpccRateResolvedProblem, ConstrainsLateralMotionBetweenSparseStages)
{
  auto request = straight_request();
  request.steering_rate_prefix_bounds = problem::SteeringRatePrefixBounds{
    -1.15, 0.05};
  problem::ProgressAlignedWallConstraints wall;
  wall.lower_slope = {0.0, 0.0, 0.0};
  wall.lower_intercept = {-1.0, -1.0, -1.0};
  wall.upper_slope = {0.0, 0.0, 0.0};
  wall.upper_intercept = {1.0, 1.0, 1.0};
  request.progress_aligned_wall_constraints = wall;
  request.swept_lateral_wall_constraints = {
    problem::SweptLateralWallConstraint{1, 0.25, -0.4, 0.6},
    problem::SweptLateralWallConstraint{1, 0.75, -0.2, 0.5}};

  const auto assembled = problem::assemble(request);
  ASSERT_TRUE(assembled.has_value());
  constexpr int horizon = 3;
  constexpr int state_values = model::kStateDimension * (horizon + 1);
  constexpr int input_values = model::kInputDimension * horizon;
  constexpr int variable_count = state_values + input_values;
  const int swept_offset =
    state_values + variable_count + horizon + 2 * horizon;
  ASSERT_EQ(assembled->constraints.rows(), swept_offset + 2);
  const int source = model::kStateDimension;
  const int destination = 2 * model::kStateDimension;
  EXPECT_DOUBLE_EQ(
    assembled->constraints.coeff(
      swept_offset, source + model::kLateralIndex), 0.75);
  EXPECT_DOUBLE_EQ(
    assembled->constraints.coeff(
      swept_offset, destination + model::kLateralIndex), 0.25);
  EXPECT_DOUBLE_EQ(assembled->lower_bound[swept_offset], -0.4);
  EXPECT_DOUBLE_EQ(assembled->upper_bound[swept_offset], 0.6);
  const auto semantic = problem::decode_row(
    swept_offset + 1, horizon, true, true, 2);
  ASSERT_TRUE(semantic.valid);
  EXPECT_EQ(semantic.kind, problem::RowKind::SweptLateralWall);
  EXPECT_EQ(semantic.stage, 1);
}

TEST(MpccRateResolvedProblem, KeepsDynamicObstacleRowsOutOfStateBoxes)
{
  auto request = straight_request();
  request.steering_rate_prefix_bounds = problem::SteeringRatePrefixBounds{
    -1.15, 0.05};
  request.dynamic_obstacle_constraints = {
    problem::DynamicObstacleConstraint{
      1, problem::DynamicObstacleConstraintAxis::EffectiveProgress,
      -std::numeric_limits<double>::infinity(), 0.35},
    problem::DynamicObstacleConstraint{
      2, problem::DynamicObstacleConstraintAxis::Lateral,
      0.75, std::numeric_limits<double>::infinity()}};

  const auto assembled = problem::assemble(request);
  ASSERT_TRUE(assembled.has_value());
  constexpr int horizon = 3;
  constexpr int nx = model::kStateDimension;
  constexpr int state_values = nx * (horizon + 1);
  constexpr int input_values = model::kInputDimension * horizon;
  constexpr int variable_count = state_values + input_values;
  const int obstacle_offset = state_values + variable_count + horizon;
  ASSERT_EQ(assembled->constraints.rows(), obstacle_offset + 2);
  EXPECT_DOUBLE_EQ(
    assembled->constraints.coeff(
      obstacle_offset, nx + model::kProgressIndex), 1.0);
  EXPECT_DOUBLE_EQ(
    assembled->constraints.coeff(
      obstacle_offset, nx + model::kLagIndex), 1.0);
  EXPECT_DOUBLE_EQ(assembled->upper_bound[obstacle_offset], 0.35);
  EXPECT_DOUBLE_EQ(
    assembled->constraints.coeff(
      obstacle_offset + 1, 2 * nx + model::kLateralIndex), 1.0);
  EXPECT_DOUBLE_EQ(assembled->lower_bound[obstacle_offset + 1], 0.75);
  const auto semantic = problem::decode_row(
    obstacle_offset, horizon, true, false, 0,
    &request.dynamic_obstacle_constraints);
  ASSERT_TRUE(semantic.valid);
  EXPECT_EQ(
    semantic.kind,
    problem::RowKind::DynamicObstacleEffectiveProgress);
  EXPECT_EQ(semantic.stage, 1);
}

TEST(MpccRateResolvedProblem, StateZeroEqualityAndSteeringOwnersAreExplicit)
{
  const auto request = straight_request();
  const auto assembled = problem::assemble(request);
  ASSERT_TRUE(assembled.has_value());
  for (int element = 0; element < model::kStateDimension; ++element) {
    EXPECT_DOUBLE_EQ(
      assembled->lower_bound[element], -request.initial_state[element]);
    EXPECT_DOUBLE_EQ(
      assembled->upper_bound[element], -request.initial_state[element]);
  }
  const auto steering_state = problem::decode_row(
    model::kStateDimension * (request.horizon_steps + 1) +
    model::kSteeringIndex,
    request.horizon_steps);
  EXPECT_TRUE(steering_state.valid);
  EXPECT_EQ(steering_state.kind, problem::RowKind::StateBox);
  EXPECT_EQ(steering_state.element, model::kSteeringIndex);

  const int state_values =
    model::kStateDimension * (request.horizon_steps + 1);
  const auto steering_rate = problem::decode_row(
    2 * state_values + model::kSteeringRateIndex,
    request.horizon_steps);
  EXPECT_TRUE(steering_rate.valid);
  EXPECT_EQ(steering_rate.kind, problem::RowKind::InputBox);
  EXPECT_EQ(steering_rate.element, model::kSteeringRateIndex);
}

TEST(MpccRateResolvedProblem, DecodesFirstProgressSpeedInputForCurrentStateDimension)
{
  constexpr int horizon = 20;
  constexpr int state_values = model::kStateDimension * (horizon + 1);
  const auto semantic = problem::decode_row(
    2 * state_values + model::kVirtualProgressSpeedIndex, horizon);
  ASSERT_TRUE(semantic.valid);
  EXPECT_EQ(semantic.kind, problem::RowKind::InputBox);
  EXPECT_EQ(semantic.stage, 0);
  EXPECT_EQ(semantic.element, model::kVirtualProgressSpeedIndex);
}

TEST(MpccRateResolvedProblem, DetectsEmptyFirstProgressSpeedInterval)
{
  auto request = straight_request();
  constexpr int nx = model::kStateDimension;
  const int first_progress = nx + model::kProgressIndex;
  request.state_lower[first_progress] = 0.0;
  request.state_upper[first_progress] = 0.0;
  request.input_lower[model::kVirtualProgressSpeedIndex] = 0.01;
  const auto diagnostic = problem::analyze_first_stage_input_feasibility(
    request, model::kVirtualProgressSpeedIndex);
  ASSERT_TRUE(diagnostic.evaluated);
  EXPECT_FALSE(diagnostic.separable);
  EXPECT_TRUE(diagnostic.conclusive);
  EXPECT_FALSE(diagnostic.feasible);
  EXPECT_DOUBLE_EQ(diagnostic.declared_lower, 0.01);
  EXPECT_NEAR(diagnostic.implied_upper, 0.0, 1e-9);
  EXPECT_EQ(
    diagnostic.limiting_upper_state_element,
    model::kProgressIndex);
}

TEST(MpccRateResolvedProblem, ReportsFeasibleFirstProgressSpeedIntersection)
{
  const auto request = straight_request();
  const auto diagnostic = problem::analyze_first_stage_input_feasibility(
    request, model::kVirtualProgressSpeedIndex);
  ASSERT_TRUE(diagnostic.evaluated);
  EXPECT_FALSE(diagnostic.separable);
  EXPECT_FALSE(diagnostic.conclusive);
  EXPECT_FALSE(diagnostic.feasible);
  EXPECT_LE(diagnostic.implied_lower, diagnostic.implied_upper);
}

TEST(MpccRateResolvedProblem, RejectsMalformedOrInconsistentContracts)
{
  auto request = straight_request();
  request.state_reference.conservativeResize(request.state_reference.size() - 1);
  EXPECT_FALSE(problem::assemble(request).has_value());

  request = straight_request();
  request.input_delta_weight[model::kSteeringRateIndex] = -1.0;
  EXPECT_FALSE(problem::assemble(request).has_value());

  request = straight_request();
  request.state_upper[model::kSteeringIndex] = 0.05;
  EXPECT_FALSE(problem::assemble(request).has_value());

  request = straight_request();
  request.linearizations.front().stage_dt_sec =
    std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(problem::assemble(request).has_value());

  request = straight_request();
  request.additional_linear_cost = Eigen::VectorXd::Zero(1);
  EXPECT_FALSE(problem::assemble(request).has_value());

  request = straight_request();
  const int variables = model::kStateDimension * (request.horizon_steps + 1) +
    model::kInputDimension * request.horizon_steps;
  request.additional_linear_cost = Eigen::VectorXd::Zero(variables);
  request.additional_linear_cost[0] =
    std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(problem::assemble(request).has_value());
}

TEST(MpccRateResolvedProblem, AddsIndependentLinearObjectiveToReferenceCost)
{
  auto request = straight_request();
  const int variables = model::kStateDimension * (request.horizon_steps + 1) +
    model::kInputDimension * request.horizon_steps;
  request.additional_linear_cost = Eigen::VectorXd::Zero(variables);
  constexpr int index = model::kProgressIndex;
  request.state_weight[index] = 2.0;
  request.state_reference[index] = 3.0;
  request.additional_linear_cost[index] = -4.0;
  const auto assembled = problem::assemble(request);
  ASSERT_TRUE(assembled.has_value());
  EXPECT_DOUBLE_EQ(assembled->linear_cost[index], -10.0);
}

TEST(MpccRateResolvedProblem, SolvesStraightProblemWithinActuatorBounds)
{
  const auto request = straight_request();
  auto assembled = problem::assemble(request);
  ASSERT_TRUE(assembled.has_value());
  solver::PersistentOsqpSolver osqp(
    solver::ConstraintPreconditioningPolicy::RowToleranceNormalized);
  const auto outcome = osqp.solve(
    assembled->quadratic_cost, assembled->constraints,
    assembled->linear_cost, assembled->lower_bound,
    assembled->upper_bound, std::nullopt, assembled->variable_scaling);
  ASSERT_TRUE(outcome.result.has_value()) << outcome.failure_detail;
  const int state_values =
    model::kStateDimension * (request.horizon_steps + 1);
  for (int stage = 0; stage < request.horizon_steps; ++stage) {
    const double steering_rate = outcome.result->primal[
      state_values + model::kInputDimension * stage +
      model::kSteeringRateIndex];
    EXPECT_GE(steering_rate, -0.7 - 1e-6);
    EXPECT_LE(steering_rate, 0.7 + 1e-6);
  }
  for (int stage = 0; stage <= request.horizon_steps; ++stage) {
    const double steering = outcome.result->primal[
      model::kStateDimension * stage + model::kSteeringIndex];
    EXPECT_GE(steering, -0.6 - 1e-6);
    EXPECT_LE(steering, 0.6 + 1e-6);
  }
}

TEST(MpccRateResolvedProblem, GenericWarmStartShiftAcceptsSevenByThreeLayout)
{
  constexpr std::size_t horizon = 3U;
  constexpr std::size_t nx =
    static_cast<std::size_t>(model::kStateDimension);
  constexpr std::size_t nu =
    static_cast<std::size_t>(model::kInputDimension);
  const std::size_t variables = nx * (horizon + 1U) + nu * horizon;
  const std::size_t rows = nx * (horizon + 1U) + variables;
  solver::WarmStart warm_start{
    Eigen::VectorXd::LinSpaced(
      static_cast<Eigen::Index>(variables), 0.0,
      static_cast<double>(variables - 1U)),
    Eigen::VectorXd::LinSpaced(
      static_cast<Eigen::Index>(rows), 0.0,
      static_cast<double>(rows - 1U))};
  const auto shifted = solver::shift_mpc_warm_start(
    warm_start, horizon,
    solver::MpcWarmStartLayout{nx, nu, 0U, {}}, 1U);
  ASSERT_TRUE(shifted.has_value());
  EXPECT_EQ(shifted->primal.size(), warm_start.primal.size());
  EXPECT_EQ(shifted->dual.size(), warm_start.dual.size());
  EXPECT_TRUE(shifted->primal.allFinite());
  EXPECT_TRUE(shifted->dual.allFinite());

  const auto expect_shifted_block = [](
      const Eigen::VectorXd & source, const Eigen::VectorXd & destination,
      const std::size_t offset, const std::size_t stage_count,
      const std::size_t stage_dimension)
    {
      for (std::size_t stage = 0U; stage < stage_count; ++stage) {
        const std::size_t source_stage = std::min(stage + 1U, stage_count - 1U);
        for (std::size_t element = 0U; element < stage_dimension; ++element) {
          EXPECT_DOUBLE_EQ(
            destination[static_cast<Eigen::Index>(
              offset + stage * stage_dimension + element)],
            source[static_cast<Eigen::Index>(
              offset + source_stage * stage_dimension + element)]);
        }
      }
    };
  const std::size_t state_values = nx * (horizon + 1U);
  expect_shifted_block(
    warm_start.primal, shifted->primal, 0U, horizon + 1U, nx);
  expect_shifted_block(
    warm_start.primal, shifted->primal, state_values, horizon, nu);
  expect_shifted_block(
    warm_start.dual, shifted->dual, 0U, horizon + 1U, nx);
  expect_shifted_block(
    warm_start.dual, shifted->dual, state_values, horizon + 1U, nx);
  expect_shifted_block(
    warm_start.dual, shifted->dual, 2U * state_values, horizon, nu);
}
