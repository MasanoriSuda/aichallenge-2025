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
  request.initial_state << 0.0, 0.0, 0.0, 2.0, 0.0, 0.10;
  request.linearizations.reserve(static_cast<std::size_t>(horizon));
  for (int stage = 0; stage < horizon; ++stage) {
    const auto linearization = model::linearize_temporal_frenet(
      model::LinearizationRequest{
        0.0, 0.0, 0.0, 2.0, 0.2 * stage, 0.10,
        0.0, 0.0, 2.0, 0.0, 2.0, 0.10});
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
    request.state_lower[offset + model::kVelocityIndex] = 0.0;
    request.state_lower[offset + model::kSteeringIndex] = -0.6;
    request.state_upper[offset + model::kSteeringIndex] = 0.6;
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

TEST(MpccRateResolvedProblem, DecodesRuntimeRow254AsFirstProgressSpeedInput)
{
  constexpr int horizon = 20;
  const auto semantic = problem::decode_row(254, horizon);
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
  ASSERT_TRUE(diagnostic.separable);
  EXPECT_FALSE(diagnostic.feasible);
  EXPECT_DOUBLE_EQ(diagnostic.declared_lower, 0.01);
  EXPECT_NEAR(diagnostic.implied_upper, 0.0, 1e-12);
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
  ASSERT_TRUE(diagnostic.separable);
  EXPECT_TRUE(diagnostic.feasible);
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

TEST(MpccRateResolvedProblem, GenericWarmStartShiftAcceptsSixByThreeLayout)
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
