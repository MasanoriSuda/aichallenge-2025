#include "multi_purpose_mpc_ros/mpcc_rate_resolved_adapter.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace adapter = multi_purpose_mpc_ros::mpcc_rate_resolved_adapter;
namespace model = multi_purpose_mpc_ros::mpcc_rate_resolved;
namespace solver = multi_purpose_mpc_ros::persistent_osqp;

namespace
{

adapter::Request curved_request(const int horizon = 4)
{
  adapter::Request request;
  request.horizon_steps = horizon;
  request.initial_state << 0.0, 0.0, 0.0, 3.0, 0.0;
  request.current_steering_rad = 0.0;
  request.wheelbase_m = 2.5;
  request.maximum_abs_steering_rad = 0.6;
  request.maximum_abs_steering_rate_radps = 1.0;
  request.previous_input << 0.0, 0.0, 3.0;
  request.input_delta_weight << 0.4, 0.2, 0.1;
  request.states.resize(static_cast<std::size_t>(horizon + 1));
  for (int stage = 0; stage <= horizon; ++stage) {
    auto & state = request.states[static_cast<std::size_t>(stage)];
    state.reference << 0.0, 0.0, 0.0, 3.0, 0.3 * stage;
    state.lower << -2.0, -2.0, -1.0, 0.0, -1.0;
    state.upper << 2.0, 2.0, 1.0, 6.0, 10.0;
    state.weight << 10.0, 2.0, 4.0, 5.0, 1.0;
    state.linear_cost << 0.0, 0.0, 0.0, 0.0, -2.0;
  }
  request.inputs.resize(static_cast<std::size_t>(horizon));
  for (auto & input : request.inputs) {
    input.reference << 0.0, 0.08, 3.0;
    input.lower << -1.0, -0.30, 0.0;
    input.upper << 1.0, 0.30, 6.0;
    input.weight << 1.0, 8.0, 1.0;
    input.linear_cost << -0.1, 0.0, -0.2;
    input.path_curvature_radpm = 0.08;
    input.stage_dt_sec = 0.10;
  }
  return request;
}

}  // namespace

TEST(MpccRateResolvedAdapter, PreservesSemanticFieldsAndMovesCurvatureOwnership)
{
  const auto request = curved_request();
  const auto result = adapter::build(request);
  ASSERT_TRUE(result.has_value());
  constexpr int stage = 1;
  const int state_offset = model::kStateDimension * stage;
  const int input_offset = 0;
  const double steering_reference = std::atan(2.5 * 0.08);
  const double jacobian =
    1.0 / (2.5 * std::pow(std::cos(steering_reference), 2.0));
  EXPECT_DOUBLE_EQ(
    result->problem.state_reference[state_offset + model::kLateralIndex],
    request.states[stage].reference[0]);
  EXPECT_DOUBLE_EQ(
    result->problem.state_reference[state_offset + model::kSteeringIndex],
    steering_reference);
  EXPECT_DOUBLE_EQ(
    result->problem.state_lower[state_offset + model::kSteeringIndex],
    -0.6);
  EXPECT_DOUBLE_EQ(
    result->problem.state_upper[state_offset + model::kSteeringIndex],
    0.6);
  EXPECT_NEAR(
    result->problem.state_weight[state_offset + model::kSteeringIndex],
    8.0 * jacobian * jacobian, 1e-12);
  EXPECT_DOUBLE_EQ(
    result->problem.input_reference[
      input_offset + model::kSteeringRateIndex], 0.0);
  EXPECT_DOUBLE_EQ(
    result->problem.input_lower[
      input_offset + model::kSteeringRateIndex], -1.0);
  EXPECT_DOUBLE_EQ(
    result->problem.input_upper[
      input_offset + model::kSteeringRateIndex], 1.0);
  EXPECT_NEAR(
    result->problem.input_weight[
      input_offset + model::kSteeringRateIndex],
    0.2 * std::pow(1.0 / 2.5, 2.0) * 0.1 * 0.1, 1e-12);
  EXPECT_DOUBLE_EQ(
    result->problem.input_delta_weight[model::kSteeringRateIndex], 0.0);
  EXPECT_DOUBLE_EQ(
    result->problem.additional_linear_cost[
      state_offset + model::kProgressIndex], -2.0);
  const int state_values =
    model::kStateDimension * (request.horizon_steps + 1);
  EXPECT_DOUBLE_EQ(
    result->problem.additional_linear_cost[
      state_values + input_offset + model::kAccelerationIndex], -0.1);
  EXPECT_DOUBLE_EQ(
    result->problem.additional_linear_cost[
      state_values + input_offset + model::kSteeringRateIndex], 0.0);
  EXPECT_DOUBLE_EQ(
    result->problem.additional_linear_cost[
      state_values + input_offset + model::kVirtualProgressSpeedIndex], -0.2);
}

TEST(MpccRateResolvedAdapter, KeepsObservedSteeringAsTheOnlyStageZeroValue)
{
  auto request = curved_request();
  request.current_steering_rad = -0.12;
  request.states.front().reference << 1.0, 1.0, 0.2, 4.0, 0.5;
  const auto result = adapter::build(request);
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(
    result->problem.state_reference.head<adapter::kLegacyStateDimension>().
    isApprox(request.initial_state, 0.0));
  EXPECT_DOUBLE_EQ(
    result->problem.initial_state[model::kSteeringIndex], -0.12);
  EXPECT_DOUBLE_EQ(
    result->problem.state_reference[model::kSteeringIndex], -0.12);
  EXPECT_DOUBLE_EQ(
    result->problem.state_lower[model::kSteeringIndex], -0.6);
  EXPECT_DOUBLE_EQ(
    result->problem.state_upper[model::kSteeringIndex], 0.6);
  EXPECT_DOUBLE_EQ(
    result->problem.state_weight[model::kSteeringIndex], 0.0);
}

TEST(MpccRateResolvedAdapter, CurvedSnapshotSolvesWithinPhysicalActuatorBoxes)
{
  const auto adapted = adapter::build(curved_request());
  ASSERT_TRUE(adapted.has_value());
  const auto assembled =
    multi_purpose_mpc_ros::mpcc_rate_resolved_problem::assemble(
    adapted->problem);
  ASSERT_TRUE(assembled.has_value());
  solver::PersistentOsqpSolver osqp(
    solver::ConstraintPreconditioningPolicy::RowToleranceNormalized);
  const auto outcome = osqp.solve(
    assembled->quadratic_cost, assembled->constraints,
    assembled->linear_cost, assembled->lower_bound,
    assembled->upper_bound, std::nullopt, assembled->variable_scaling);
  ASSERT_TRUE(outcome.result.has_value()) << outcome.failure_detail;
  constexpr int horizon = 4;
  const int state_values = model::kStateDimension * (horizon + 1);
  for (int stage = 0; stage <= horizon; ++stage) {
    const int variable =
      model::kStateDimension * stage + model::kSteeringIndex;
    const int box_row = state_values + variable;
    const double tolerance =
      outcome.result->constraint_tolerance[box_row] + 1e-9;
    const double steering = outcome.result->primal[
      variable];
    EXPECT_GE(steering, -0.6 - tolerance);
    EXPECT_LE(steering, 0.6 + tolerance);
  }
  for (int stage = 0; stage < horizon; ++stage) {
    const int variable =
      state_values + model::kInputDimension * stage +
      model::kSteeringRateIndex;
    const int box_row = state_values + variable;
    const double tolerance =
      outcome.result->constraint_tolerance[box_row] + 1e-9;
    const double steering_rate = outcome.result->primal[
      variable];
    EXPECT_GE(steering_rate, -1.0 - tolerance);
    EXPECT_LE(steering_rate, 1.0 + tolerance);
  }
}

TEST(MpccRateResolvedAdapter, RejectsMalformedOrUnphysicalSnapshots)
{
  auto request = curved_request();
  request.states.pop_back();
  EXPECT_FALSE(adapter::build(request).has_value());

  request = curved_request();
  request.wheelbase_m = 0.0;
  EXPECT_FALSE(adapter::build(request).has_value());

  request = curved_request();
  request.current_steering_rad = 0.7;
  EXPECT_FALSE(adapter::build(request).has_value());

  request = curved_request();
  request.inputs.front().stage_dt_sec = 0.0;
  EXPECT_FALSE(adapter::build(request).has_value());

  request = curved_request();
  request.inputs.front().lower[adapter::kLegacyCurvatureIndex] = 0.4;
  request.inputs.front().upper[adapter::kLegacyCurvatureIndex] = 0.3;
  EXPECT_FALSE(adapter::build(request).has_value());

  request = curved_request();
  request.states.front().reference[0] =
    std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(adapter::build(request).has_value());

  request = curved_request();
  request.states.front().linear_cost[0] =
    std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(adapter::build(request).has_value());

  request = curved_request();
  request.inputs.front().linear_cost[adapter::kLegacyCurvatureIndex] = 1.0;
  EXPECT_FALSE(adapter::build(request).has_value());

  request = curved_request();
  request.states.front().upper[0] = -0.1;
  EXPECT_FALSE(adapter::build(request).has_value());
}
