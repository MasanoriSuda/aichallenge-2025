#include "multi_purpose_mpc_ros/mpcc_rate_resolved_adapter.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace adapter = multi_purpose_mpc_ros::mpcc_rate_resolved_adapter;
namespace model = multi_purpose_mpc_ros::mpcc_rate_resolved;
namespace solver = multi_purpose_mpc_ros::persistent_osqp;

namespace
{

constexpr solver::PhysicalConstraintTolerance kSolverTolerance{1e-3, 1e-3};

adapter::Request curved_request(const int horizon = 4)
{
  adapter::Request request;
  request.horizon_steps = horizon;
  request.initial_state << 0.0, 0.0, 0.0, 3.0, 0.0;
  request.current_steering_rad = 0.0;
  request.current_response_steering_rad = 0.0;
  request.wheelbase_m = 2.5;
  request.yaw_response_gain = 0.75;
  request.yaw_response_time_constant_sec = 0.13;
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
  const auto result = adapter::build(request, kSolverTolerance);
  ASSERT_TRUE(result.has_value());
  constexpr int stage = 1;
  const int state_offset = model::kStateDimension * stage;
  const int input_offset = 0;
  const double steering_reference = std::atan(2.5 * 0.08 / 0.75);
  const double jacobian =
    0.75 / (2.5 * std::pow(std::cos(steering_reference), 2.0));
  EXPECT_DOUBLE_EQ(
    result->problem.state_reference[state_offset + model::kLateralIndex],
    request.states[stage].reference[0]);
  EXPECT_DOUBLE_EQ(
    result->problem.state_reference[state_offset + model::kSteeringIndex],
    steering_reference);
  EXPECT_GE(
    result->problem.state_reference[
      state_offset + model::kResponseSteeringIndex],
    0.0);
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
  const double first_rate_margin = (1e-3 + 1e-3 * 1.0) / (1.0 - 1e-3);
  EXPECT_NEAR(
    result->problem.input_lower[
      input_offset + model::kSteeringRateIndex],
    -1.0 + first_rate_margin, 1e-12);
  EXPECT_NEAR(
    result->problem.input_upper[
      input_offset + model::kSteeringRateIndex],
    1.0 - first_rate_margin, 1e-12);
  const double acceleration_margin =
    (1e-3 + 1e-3 * 1.0) / (1.0 - 1e-3);
  EXPECT_NEAR(
    result->problem.input_lower[
      input_offset + model::kAccelerationIndex],
    -1.0 + acceleration_margin, 1e-12);
  EXPECT_NEAR(
    result->problem.input_upper[
      input_offset + model::kAccelerationIndex],
    1.0 - acceleration_margin, 1e-12);
  EXPECT_NEAR(
    result->problem.input_lower[
      input_offset + model::kVirtualProgressSpeedIndex],
    0.0, 1e-12);
  EXPECT_NEAR(
    result->problem.input_upper[
      input_offset + model::kVirtualProgressSpeedIndex],
    6.0, 1e-12);
  EXPECT_NEAR(
    result->first_steering_rate_certificate_margin_radps,
    first_rate_margin, 1e-12);
  EXPECT_NEAR(
    result->problem.input_weight[
      input_offset + model::kSteeringRateIndex],
    0.2 * std::pow(0.75 / 2.5, 2.0) * 0.1 * 0.1, 1e-12);
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

TEST(
  MpccRateResolvedAdapter,
  CurvatureReferenceUsesTheSameYawResponseGainAsTheDynamics)
{
  const auto request = curved_request();
  const auto result = adapter::build(request, kSolverTolerance);
  ASSERT_TRUE(result.has_value());

  constexpr int stage = 1;
  const int state_offset = model::kStateDimension * stage;
  const double steering = result->problem.state_reference[
    state_offset + model::kSteeringIndex];
  const double represented_curvature =
    request.yaw_response_gain * std::tan(steering) / request.wheelbase_m;

  EXPECT_NEAR(
    represented_curvature,
    request.inputs.front().reference[adapter::kLegacyCurvatureIndex], 1e-12);
  EXPECT_NEAR(
    result->problem.state_lower[state_offset + model::kSteeringIndex],
    std::max(-request.maximum_abs_steering_rad, std::atan(
      request.wheelbase_m *
      request.inputs.front().lower[adapter::kLegacyCurvatureIndex] /
      request.yaw_response_gain)),
    1e-12);
  EXPECT_NEAR(
    result->problem.state_upper[state_offset + model::kSteeringIndex],
    std::min(request.maximum_abs_steering_rad, std::atan(
      request.wheelbase_m *
      request.inputs.front().upper[adapter::kLegacyCurvatureIndex] /
      request.yaw_response_gain)),
    1e-12);
}

TEST(
  MpccRateResolvedAdapter,
  KeepsInfeasibleCurvatureReferencesInsideThePhysicalSteeringModel)
{
  auto request = curved_request(8);
  request.maximum_abs_steering_rad = 0.366519143;
  for (auto & input : request.inputs) {
    // This legacy curvature target requires more steering than the actuator
    // can produce.  It is a soft reference, not grounds for deleting the
    // entire canonical normal problem while the vehicle is moving.
    input.reference[adapter::kLegacyCurvatureIndex] = 0.30;
    input.lower[adapter::kLegacyCurvatureIndex] = -0.30;
    input.upper[adapter::kLegacyCurvatureIndex] = 0.30;
  }

  adapter::BuildDiagnostic diagnostic;
  const auto result = adapter::build(request, kSolverTolerance, &diagnostic);

  ASSERT_TRUE(result.has_value())
    << adapter::to_string(diagnostic.reason) << " stage=" << diagnostic.stage
    << " value=" << diagnostic.value;
  for (int stage = 0; stage <= request.horizon_steps; ++stage) {
    const int offset = model::kStateDimension * stage;
    EXPECT_LE(
      std::abs(result->problem.state_reference[offset + model::kSteeringIndex]),
      request.maximum_abs_steering_rad);
    EXPECT_LE(
      std::abs(result->problem.state_reference[
        offset + model::kResponseSteeringIndex]),
      request.maximum_abs_steering_rad);
  }
}

TEST(
  MpccRateResolvedAdapter,
  UsesCommandSteeringAsTheOnlyPrefixReachabilityOrigin)
{
  auto request = curved_request();
  request.current_steering_rad = -0.12;
  request.current_response_steering_rad = -0.08;
  request.states.front().reference << 1.0, 1.0, 0.2, 4.0, 0.5;
  const auto result = adapter::build(request, kSolverTolerance);
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(
    result->problem.state_reference.head<adapter::kLegacyStateDimension>().
    isApprox(request.initial_state, 0.0));
  EXPECT_DOUBLE_EQ(
    result->problem.initial_state[model::kSteeringIndex], -0.12);
  EXPECT_DOUBLE_EQ(
    result->problem.initial_state[model::kResponseSteeringIndex], -0.08);
  EXPECT_DOUBLE_EQ(
    result->problem.state_reference[model::kSteeringIndex], -0.12);
  EXPECT_DOUBLE_EQ(
    result->problem.state_lower[model::kSteeringIndex], -0.6);
  EXPECT_DOUBLE_EQ(
    result->problem.state_upper[model::kSteeringIndex], 0.6);
  ASSERT_TRUE(result->problem.steering_rate_prefix_bounds.has_value());
  // Robustification scales with the largest cumulative physical delta.  From
  // -0.12 rad the positive span to the +0.60 rad actuator bound is 0.72 rad.
  const double prefix_margin = (1e-3 + 1e-3 * 0.72) / (1.0 - 1e-3);
  EXPECT_NEAR(
    result->problem.steering_rate_prefix_bounds->
    minimum_cumulative_delta_rad,
    -0.48 + prefix_margin, 1e-12);
  EXPECT_NEAR(
    result->problem.steering_rate_prefix_bounds->
    maximum_cumulative_delta_rad,
    0.72 - prefix_margin, 1e-12);
  EXPECT_DOUBLE_EQ(
    result->problem.state_weight[model::kSteeringIndex], 0.0);
}

TEST(MpccRateResolvedAdapter, CurvedSnapshotSolvesWithinPhysicalActuatorBoxes)
{
  solver::PersistentOsqpSolver osqp(
    solver::ConstraintPreconditioningPolicy::RowToleranceNormalized);
  const auto adapted = adapter::build(
    curved_request(), osqp.physical_constraint_tolerance());
  ASSERT_TRUE(adapted.has_value());
  const auto assembled =
    multi_purpose_mpc_ros::mpcc_rate_resolved_problem::assemble(
    adapted->problem);
  ASSERT_TRUE(assembled.has_value());
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
    const int input_base =
      state_values + model::kInputDimension * stage +
      model::kAccelerationIndex;
    const double acceleration = outcome.result->primal[input_base];
    const double steering_rate = outcome.result->primal[
      input_base + model::kSteeringRateIndex];
    const double progress_speed = outcome.result->primal[
      input_base + model::kVirtualProgressSpeedIndex];
    EXPECT_GE(acceleration, -1.0);
    EXPECT_LE(acceleration, 1.0);
    EXPECT_GE(progress_speed, 0.0);
    EXPECT_LE(progress_speed, 6.0);
    const int variable = input_base + model::kSteeringRateIndex;
    const int box_row = state_values + variable;
    EXPECT_GT(outcome.result->constraint_tolerance[box_row], 0.0);
    EXPECT_GE(steering_rate, -1.0);
    EXPECT_LE(steering_rate, 1.0);
  }
}

TEST(MpccRateResolvedAdapter, RelinearizesTheSameProblemAroundTheSolvedIterate)
{
  const auto request = curved_request();
  auto adapted = adapter::build(request, kSolverTolerance);
  ASSERT_TRUE(adapted.has_value());
  const int horizon = request.horizon_steps;
  const int state_values = model::kStateDimension * (horizon + 1);
  const int variable_count =
    state_values + model::kInputDimension * horizon;
  Eigen::VectorXd primal = Eigen::VectorXd::Zero(variable_count);
  for (int stage = 0; stage <= horizon; ++stage) {
    const int state = model::kStateDimension * stage;
    primal[state + model::kLateralIndex] = 0.08 * stage;
    primal[state + model::kLagIndex] = -0.02 * stage;
    primal[state + model::kHeadingIndex] = 0.03 * stage;
    primal[state + model::kVelocityIndex] = 3.0 + 0.05 * stage;
    primal[state + model::kProgressIndex] = 0.30 * stage;
    primal[state + model::kSteeringIndex] = 0.04 * stage;
    primal[state + model::kResponseSteeringIndex] = 0.03 * stage;
  }
  for (int stage = 0; stage < horizon; ++stage) {
    const int input = state_values + model::kInputDimension * stage;
    primal[input + model::kAccelerationIndex] = 0.10;
    primal[input + model::kSteeringRateIndex] = 0.15;
    primal[input + model::kVirtualProgressSpeedIndex] = 3.0;
  }
  const auto original_state_lower = adapted->problem.state_lower;
  const auto original_input_upper = adapted->problem.input_upper;
  const auto original_third = adapted->problem.linearizations[2];

  const auto relinearized = adapter::relinearize_around_primal(
    request, primal, adapted->problem);

  ASSERT_TRUE(relinearized.applied);
  EXPECT_EQ(
    relinearized.reason, adapter::RelinearizationReason::Accepted);
  EXPECT_EQ(relinearized.stage, -1);
  EXPECT_TRUE(adapted->problem.state_lower.isApprox(original_state_lower, 0.0));
  EXPECT_TRUE(adapted->problem.input_upper.isApprox(original_input_upper, 0.0));
  EXPECT_FALSE(adapted->problem.linearizations[2].equality_offset.isApprox(
    original_third.equality_offset, 1e-12));
  const auto & tangent = adapted->problem.linearizations[2];
  const int state = 2 * model::kStateDimension;
  const int input = state_values + 2 * model::kInputDimension;
  const Eigen::Matrix<double, model::kStateDimension, 1> reference_state =
    primal.segment<model::kStateDimension>(state);
  const Eigen::Matrix<double, model::kInputDimension, 1> reference_input =
    primal.segment<model::kInputDimension>(input);
  const auto tangent_next = tangent.state_matrix * reference_state +
    tangent.input_matrix * reference_input - tangent.equality_offset;
  EXPECT_TRUE(tangent_next.allFinite());
  EXPECT_GT(tangent_next[model::kProgressIndex],
    reference_state[model::kProgressIndex]);
}

TEST(MpccRateResolvedAdapter, RejectsRelinearizationWithoutACompleteIterate)
{
  const auto request = curved_request();
  auto adapted = adapter::build(request, kSolverTolerance);
  ASSERT_TRUE(adapted.has_value());
  Eigen::VectorXd incomplete = Eigen::VectorXd::Zero(3);

  const auto result = adapter::relinearize_around_primal(
    request, incomplete, adapted->problem);

  EXPECT_FALSE(result.applied);
  EXPECT_EQ(result.reason, adapter::RelinearizationReason::InvalidPrimal);
}

TEST(MpccRateResolvedAdapter, RejectsMalformedOrUnphysicalSnapshots)
{
  auto request = curved_request();
  request.states.pop_back();
  EXPECT_FALSE(adapter::build(request, kSolverTolerance).has_value());

  request = curved_request();
  request.wheelbase_m = 0.0;
  EXPECT_FALSE(adapter::build(request, kSolverTolerance).has_value());

  request = curved_request();
  request.current_steering_rad = 0.7;
  EXPECT_FALSE(adapter::build(request, kSolverTolerance).has_value());

  request = curved_request();
  request.inputs.front().stage_dt_sec = 0.0;
  EXPECT_FALSE(adapter::build(request, kSolverTolerance).has_value());

  request = curved_request();
  request.inputs.front().lower[adapter::kLegacyCurvatureIndex] = 0.4;
  request.inputs.front().upper[adapter::kLegacyCurvatureIndex] = 0.3;
  EXPECT_FALSE(adapter::build(request, kSolverTolerance).has_value());

  request = curved_request();
  request.states.front().reference[0] =
    std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(adapter::build(request, kSolverTolerance).has_value());

  request = curved_request();
  request.states.front().linear_cost[0] =
    std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(adapter::build(request, kSolverTolerance).has_value());

  request = curved_request();
  request.inputs.front().linear_cost[adapter::kLegacyCurvatureIndex] = 1.0;
  EXPECT_FALSE(adapter::build(request, kSolverTolerance).has_value());

  request = curved_request();
  request.states.front().upper[0] = -0.1;
  EXPECT_FALSE(adapter::build(request, kSolverTolerance).has_value());

  request = curved_request();
  request.inputs.front().lower[0] = 0.0;
  request.inputs.front().upper[0] = 0.0;
  EXPECT_FALSE(adapter::build(request, kSolverTolerance).has_value());
}

TEST(MpccRateResolvedAdapter, IdentifiesTheExactInitialStateBoundViolation)
{
  auto request = curved_request();
  request.states.front().upper[model::kProgressIndex] = -0.1;
  adapter::BuildDiagnostic diagnostic;

  EXPECT_FALSE(
    adapter::build(request, kSolverTolerance, &diagnostic).has_value());
  EXPECT_EQ(
    diagnostic.reason, adapter::RejectReason::InitialStateOutsideBounds);
  EXPECT_EQ(diagnostic.stage, 0);
  EXPECT_EQ(diagnostic.element, model::kProgressIndex);
  EXPECT_DOUBLE_EQ(diagnostic.value, 0.0);
  EXPECT_DOUBLE_EQ(diagnostic.lower, -1.0);
  EXPECT_DOUBLE_EQ(diagnostic.upper, -0.1);
}

TEST(MpccRateResolvedAdapter, PreservesAZeroWidthFutureStopState)
{
  auto request = curved_request();
  constexpr int stop_stage = 2;
  request.states[stop_stage].reference[model::kVelocityIndex] = 0.0;
  request.states[stop_stage].lower[model::kVelocityIndex] = 0.0;
  request.states[stop_stage].upper[model::kVelocityIndex] = 0.0;
  adapter::BuildDiagnostic diagnostic;

  const auto result = adapter::build(request, kSolverTolerance, &diagnostic);

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(diagnostic.reason, adapter::RejectReason::None);
  const int state_offset = model::kStateDimension * stop_stage;
  EXPECT_DOUBLE_EQ(
    result->problem.state_lower[state_offset + model::kVelocityIndex], 0.0);
  EXPECT_DOUBLE_EQ(
    result->problem.state_upper[state_offset + model::kVelocityIndex], 0.0);
}

TEST(MpccRateResolvedAdapter, PreservesAZeroWidthVirtualProgressHold)
{
  auto request = curved_request();
  constexpr int hold_stage = 2;
  request.inputs[hold_stage].reference[
    model::kVirtualProgressSpeedIndex] = 0.0;
  request.inputs[hold_stage].lower[model::kVirtualProgressSpeedIndex] = 0.0;
  request.inputs[hold_stage].upper[model::kVirtualProgressSpeedIndex] = 0.0;
  adapter::BuildDiagnostic diagnostic;

  const auto result = adapter::build(request, kSolverTolerance, &diagnostic);

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(diagnostic.reason, adapter::RejectReason::None);
  const int input_offset = model::kInputDimension * hold_stage;
  EXPECT_DOUBLE_EQ(
    result->problem.input_lower[
      input_offset + model::kVirtualProgressSpeedIndex], 0.0);
  EXPECT_DOUBLE_EQ(
    result->problem.input_upper[
      input_offset + model::kVirtualProgressSpeedIndex], 0.0);
}

TEST(MpccRateResolvedAdapter, RejectsAnUncertifiableAccelerationSingleton)
{
  auto request = curved_request();
  constexpr int stage = 2;
  request.inputs[stage].reference[model::kAccelerationIndex] = 0.0;
  request.inputs[stage].lower[model::kAccelerationIndex] = 0.0;
  request.inputs[stage].upper[model::kAccelerationIndex] = 0.0;
  adapter::BuildDiagnostic diagnostic;

  EXPECT_FALSE(
    adapter::build(request, kSolverTolerance, &diagnostic).has_value());
  EXPECT_EQ(
    diagnostic.reason,
    adapter::RejectReason::AccelerationInsetUnavailable);
  EXPECT_EQ(diagnostic.stage, stage);
  EXPECT_EQ(diagnostic.element, model::kAccelerationIndex);
  EXPECT_DOUBLE_EQ(diagnostic.lower, 0.0);
  EXPECT_DOUBLE_EQ(diagnostic.upper, 0.0);
}

TEST(MpccRateResolvedAdapter, RejectsAnUncertifiableSteeringRateSingleton)
{
  auto request = curved_request();
  constexpr int stage = 0;
  request.maximum_abs_steering_rate_radps = 0.0;
  adapter::BuildDiagnostic diagnostic;

  EXPECT_FALSE(
    adapter::build(request, kSolverTolerance, &diagnostic).has_value());
  EXPECT_EQ(
    diagnostic.reason,
    adapter::RejectReason::SteeringRateInsetUnavailable);
  EXPECT_EQ(diagnostic.stage, stage);
  EXPECT_EQ(diagnostic.element, model::kSteeringRateIndex);
  EXPECT_DOUBLE_EQ(diagnostic.lower, 0.0);
  EXPECT_DOUBLE_EQ(diagnostic.upper, 0.0);
}

TEST(MpccRateResolvedAdapter, FirstRateIsRobustlyReachableFromSemanticSteering)
{
  auto request = curved_request();
  request.current_steering_rad = request.maximum_abs_steering_rad;
  const auto result = adapter::build(request, kSolverTolerance);
  ASSERT_TRUE(result.has_value());

  const double margin = (1e-3 + 1e-3 * 1.0) / (1.0 - 1e-3);
  EXPECT_DOUBLE_EQ(result->first_steering_rate_physical_upper_radps, 0.0);
  EXPECT_NEAR(result->first_steering_rate_solver_upper_radps, -margin, 1e-12);
  const double accepted_upper_residual =
    kSolverTolerance.absolute + kSolverTolerance.relative *
    std::abs(result->first_steering_rate_solver_upper_radps);
  EXPECT_LE(
    result->first_steering_rate_solver_upper_radps + accepted_upper_residual,
    result->first_steering_rate_physical_upper_radps);

  request.maximum_abs_steering_rate_radps = 0.001;
  EXPECT_FALSE(adapter::build(request, kSolverTolerance).has_value());

  auto invalid_tolerance = kSolverTolerance;
  invalid_tolerance.relative = 1.0;
  EXPECT_FALSE(adapter::build(curved_request(), invalid_tolerance).has_value());
}

TEST(
  MpccRateResolvedAdapter,
  SteeringRatePrefixHasOnePhysicalStateOrigin)
{
  auto request = curved_request();
  request.current_steering_rad = -0.20;
  const auto result = adapter::build(request, kSolverTolerance);
  ASSERT_TRUE(result.has_value());

  ASSERT_TRUE(result->problem.steering_rate_prefix_bounds.has_value());
  const double prefix_margin = (1e-3 + 1e-3 * 0.8) / (1.0 - 1e-3);
  // Only the physical origin -0.20 is a vehicle state. The exact cumulative
  // delta may use the complete [-0.40, 0.80] physical steering envelope.
  EXPECT_NEAR(
    result->problem.steering_rate_prefix_bounds->
    minimum_cumulative_delta_rad,
    -0.40 + prefix_margin, 1e-12);
  EXPECT_NEAR(
    result->problem.steering_rate_prefix_bounds->
    maximum_cumulative_delta_rad,
    0.80 - prefix_margin, 1e-12);
}
