#include "mpcc_vehicle_model_fixture.hpp"
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

double exact_inset_margin(const double characteristic)
{
  const double accepted_absolute =
    solver::kSolvedInaccurateToleranceMultiplier * kSolverTolerance.absolute;
  const double accepted_relative =
    solver::kSolvedInaccurateToleranceMultiplier * kSolverTolerance.relative;
  return (accepted_absolute + accepted_relative * characteristic) /
    (1.0 - accepted_relative);
}

adapter::Request curved_request(const int horizon = 4)
{
  adapter::Request request;
  request.horizon_steps = horizon;
  request.initial_state << 0.0, 0.0, 0.0, 3.0, 0.0;
  request.current_steering_rad = 0.0;
  request.current_response_steering_rad = 0.0;
  request.wheelbase_m = 2.5;
  request.curvature_reference_gain = 0.75;
  request.vehicle_model = multi_purpose_mpc_ros::test::vehicle_model();
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

TEST(MpccRateResolvedAdapter, ExposesExactExecutablePhysicalBoundary)
{
  const auto bounds = adapter::resolve_exact_physical_boundary_bounds(
    -3.0, 1.37, kSolverTolerance);
  ASSERT_TRUE(bounds.has_value());
  const double expected_margin = exact_inset_margin(3.0);
  EXPECT_NEAR(bounds->lower, -3.0 + expected_margin, 1e-12);
  EXPECT_NEAR(bounds->upper, 1.37 - expected_margin, 1e-12);
  EXPECT_NEAR(bounds->certificate_margin, expected_margin, 1e-12);
}

TEST(MpccRateResolvedAdapter,
     CourseSupportCoversAcceptedNegativeVirtualProgress) {
  auto request = curved_request(1);
  for (auto &state : request.states)
    state.lower[model::kProgressIndex] = 0.0;
  const auto support =
      adapter::resolve_course_frame_progress_support(request, kSolverTolerance);
  ASSERT_TRUE(support);
  constexpr double virtual_speed = -1e-6;
  const double exact_progress =
      virtual_speed * request.inputs.front().stage_dt_sec;
  EXPECT_LT(exact_progress, 0.0);
  EXPECT_LT(support->lower_progress_m, exact_progress);
  EXPECT_GT(support->upper_progress_m,
            request.states.back().upper[model::kProgressIndex]);

  Eigen::SparseMatrix<double> row(1, 1);
  row.insert(0, 0) = 1.0;
  const auto residual = solver::evaluate_constraint_residuals(
      row, Eigen::VectorXd::Constant(1, virtual_speed),
      Eigen::VectorXd::Zero(1), Eigen::VectorXd::Constant(1, 6.0),
      kSolverTolerance.absolute, kSolverTolerance.relative);
  ASSERT_TRUE(residual);
  EXPECT_LT(residual->maximum_normalized_violation, 1.0);

  const std::vector<multi_purpose_mpc_ros::mpc_stage_geometry::CourseFrameKnot>
      forward{{0.0, 0.0, 0.0, 0.0, 1}, {1.0, 1.0, 0.0, 0.0, 2}};
  model::LinearizationRequest transition;
  transition.reference_velocity_mps = 3.0;
  transition.reference_virtual_progress_speed_mps = virtual_speed;
  transition.wheelbase_m = request.wheelbase_m;
  transition.vehicle_model = request.vehicle_model;
  transition.stage_dt_sec = request.inputs.front().stage_dt_sec;
  transition.minimum_stage_dt_sec = transition.stage_dt_sec;
  transition.maximum_stage_dt_sec = transition.stage_dt_sec;
  transition.course_frame = {std::make_shared<const decltype(forward)>(forward),
                             0.0};
  EXPECT_FALSE(model::evaluate_temporal_frenet_transition(transition));
  auto covering = forward;
  covering.insert(covering.begin(), {-1.0, -1.0, 0.0, 0.0, 0});
  ASSERT_GT(support->lower_progress_m, covering.front().progress_m);
  transition.course_frame.knots =
      std::make_shared<const decltype(covering)>(covering);
  const auto replay = model::evaluate_temporal_frenet_transition(transition);
  ASSERT_TRUE(replay);
  EXPECT_NEAR(replay->next_state[model::kProgressIndex], exact_progress, 1e-12);
}

TEST(MpccRateResolvedAdapter, CourseSupportIntegratesInaccurateSingletonRows) {
  auto request = curved_request(20);
  for (auto &state : request.states) {
    state.lower[model::kProgressIndex] = 0.0;
    state.upper[model::kProgressIndex] = 0.0;
  }
  for (auto &input : request.inputs) {
    input.lower[model::kVirtualProgressSpeedIndex] = 0.0;
    input.upper[model::kVirtualProgressSpeedIndex] = 0.0;
    input.stage_dt_sec = 0.25;
  }
  const auto support =
      adapter::resolve_course_frame_progress_support(request, {1e-3, 0.0});
  ASSERT_TRUE(support);
  // A solved-inaccurate zero row may accept -0.009. Integrating those
  // controls can leave the nominal state box even with accepted equality
  // residuals. The map must cover that exact trajectory in both directions.
  EXPECT_LE(support->lower_progress_m, -0.05);
  EXPECT_GE(support->upper_progress_m, 0.05);
  EXPECT_LT(support->lower_progress_m, 20.0 * 0.25 * -0.009);
}

TEST(MpccRateResolvedAdapter, CourseSupportKeepsInitialAndSemanticStateRanges) {
  auto request = curved_request(2);
  request.initial_state[model::kProgressIndex] = 12.0;
  request.states.back().lower[model::kProgressIndex] = -4.0;
  request.states.back().upper[model::kProgressIndex] = 50.0;
  const auto support =
      adapter::resolve_course_frame_progress_support(request, {0.0, 0.0});
  ASSERT_TRUE(support);
  EXPECT_LE(support->lower_progress_m, -4.0);
  EXPECT_GE(support->upper_progress_m, 50.0);
}

TEST(MpccRateResolvedAdapter,
     CourseSupportRejectsUnavailableBoundsAndTolerance) {
  const auto request = curved_request();
  EXPECT_FALSE(
      adapter::resolve_course_frame_progress_support(request, {-1.0, 0.0}));
  EXPECT_FALSE(
      adapter::resolve_course_frame_progress_support(request, {0.0, 0.1}));
  auto invalid = request;
  invalid.inputs.pop_back();
  EXPECT_FALSE(adapter::resolve_course_frame_progress_support(
      invalid, kSolverTolerance));
  invalid = request;
  invalid.inputs.front().stage_dt_sec = 0.0;
  EXPECT_FALSE(adapter::resolve_course_frame_progress_support(
      invalid, kSolverTolerance));
  invalid = request;
  invalid.inputs.front().upper[model::kVirtualProgressSpeedIndex] =
      std::numeric_limits<double>::infinity();
  EXPECT_FALSE(adapter::resolve_course_frame_progress_support(
      invalid, kSolverTolerance));
  invalid = request;
  invalid.states.front().lower[model::kProgressIndex] =
      invalid.states.front().upper[model::kProgressIndex] + 1.0;
  EXPECT_FALSE(adapter::resolve_course_frame_progress_support(
      invalid, kSolverTolerance));
}

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
  const double first_rate_margin = exact_inset_margin(1.0);
  EXPECT_NEAR(
    result->problem.input_lower[
      input_offset + model::kSteeringRateIndex],
    -1.0 + first_rate_margin, 1e-12);
  EXPECT_NEAR(
    result->problem.input_upper[
      input_offset + model::kSteeringRateIndex],
    1.0 - first_rate_margin, 1e-12);
  const double acceleration_margin = first_rate_margin;
  EXPECT_NEAR(
    result->problem.input_lower[
      input_offset + model::kAccelerationIndex],
    -1.0 + acceleration_margin, 1e-12);
  EXPECT_NEAR(
    result->problem.input_upper[
      input_offset + model::kAccelerationIndex],
    1.0 - acceleration_margin, 1e-12);
  const double acceleration_solver_lower =
    result->problem.input_lower[
    input_offset + model::kAccelerationIndex];
  const double acceleration_solver_upper =
    result->problem.input_upper[
    input_offset + model::kAccelerationIndex];
  const double accepted_lower_residual =
    solver::kSolvedInaccurateToleranceMultiplier *
    (kSolverTolerance.absolute + kSolverTolerance.relative *
    std::abs(acceleration_solver_lower));
  const double accepted_upper_residual =
    solver::kSolvedInaccurateToleranceMultiplier *
    (kSolverTolerance.absolute + kSolverTolerance.relative *
    std::abs(acceleration_solver_upper));
  EXPECT_GE(
    acceleration_solver_lower - accepted_lower_residual,
    request.inputs.front().lower[model::kAccelerationIndex]);
  EXPECT_LE(
    acceleration_solver_upper + accepted_upper_residual,
    request.inputs.front().upper[model::kAccelerationIndex]);
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
  CurvatureReferenceConversionPreservesTheExistingCommandBounds)
{
  const auto request = curved_request();
  const auto result = adapter::build(request, kSolverTolerance);
  ASSERT_TRUE(result.has_value());

  constexpr int stage = 1;
  const int state_offset = model::kStateDimension * stage;
  const double steering = result->problem.state_reference[
    state_offset + model::kSteeringIndex];
  const double represented_curvature =
    request.curvature_reference_gain * std::tan(steering) / request.wheelbase_m;

  EXPECT_NEAR(
    represented_curvature,
    request.inputs.front().reference[adapter::kLegacyCurvatureIndex], 1e-12);
  EXPECT_NEAR(
    result->problem.state_lower[state_offset + model::kSteeringIndex],
    std::max(-request.maximum_abs_steering_rad, std::atan(
      request.wheelbase_m *
      request.inputs.front().lower[adapter::kLegacyCurvatureIndex] /
      request.curvature_reference_gain)),
    1e-12);
  EXPECT_NEAR(
    result->problem.state_upper[state_offset + model::kSteeringIndex],
    std::min(request.maximum_abs_steering_rad, std::atan(
      request.wheelbase_m *
      request.inputs.front().upper[adapter::kLegacyCurvatureIndex] /
      request.curvature_reference_gain)),
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
  const double prefix_margin = exact_inset_margin(0.72);
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
  EXPECT_TRUE((adapted->problem.state_lower.array() == original_state_lower.array()).all());
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

TEST(
  MpccRateResolvedAdapter,
  ProjectsCertifiedBoxResidualBeforeSelectingTheNonlinearTangent)
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
    primal[state + model::kVelocityIndex] = 3.0;
    primal[state + model::kProgressIndex] = 0.3 * stage;
  }
  for (int stage = 0; stage < horizon; ++stage) {
    const int input = state_values + model::kInputDimension * stage;
    primal[input + model::kVirtualProgressSpeedIndex] = 3.0;
  }
  // OSQP may return a point this far outside a zero lower box while the row is
  // still certified by its configured residual.  It is not a physical
  // negative speed and must not make the next SQP tangent undefined.
  primal[model::kVelocityIndex] = -1.0e-6;
  primal[state_values + model::kVirtualProgressSpeedIndex] = -1.0e-6;

  const auto result = adapter::relinearize_around_primal(
    request, primal, adapted->problem);

  EXPECT_TRUE(result.applied);
  EXPECT_EQ(result.reason, adapter::RelinearizationReason::Accepted);
}

TEST(MpccRateResolvedAdapter, SelectsTangentTransitionInsideImmutableCourseDomain)
{
  namespace geometry = multi_purpose_mpc_ros::mpc_stage_geometry;
  for (const double last_dt : {0.10, 0.23937229491124654}) {
    SCOPED_TRACE(last_dt);
    auto request = curved_request(2);
    const double terminal_progress = 0.3 + 3.0 * last_dt;
    for (auto & state : request.states) {
      state.lower[4] = 0.0;
      state.upper[4] = terminal_progress;
    }
    request.states.back().reference[4] = terminal_progress;
    for (auto & input : request.inputs) {
      input.path_curvature_radpm = 0.0;
      input.reference[adapter::kLegacyCurvatureIndex] = 0.0;
    }
    request.inputs.back().stage_dt_sec = last_dt;
    request.course_frame = {
      std::make_shared<const std::vector<geometry::CourseFrameKnot>>(
        std::vector<geometry::CourseFrameKnot>{
          {50.0, 89613.0, 43161.0, 0.0, 0},
          {50.3, 89613.3, 43161.0, 0.0, 1},
          {50.0 + terminal_progress, 89613.0 + terminal_progress, 43161.0, 0.0, 2}}),
      50.0};
    auto adapted = adapter::build(request, kSolverTolerance);
    ASSERT_TRUE(adapted.has_value());
    constexpr int state_values = 3 * model::kStateDimension;
    Eigen::VectorXd primal(state_values + 2 * model::kInputDimension);
    primal.head(state_values) = adapted->problem.state_reference;
    primal.tail(2 * model::kInputDimension) = adapted->problem.input_reference;
    // Same failure as source1629: individual boxes are respected at the
    // tangent origin, but its transition endpoint carries a solver residual.
    const int last_input = state_values + model::kInputDimension;
    primal[last_input + model::kVirtualProgressSpeedIndex] += 1.2e-8 / last_dt;
    primal[2 * model::kStateDimension + model::kProgressIndex] += 1.2e-8;
    const Eigen::VectorXd original = primal;
    const auto lower = adapted->problem.state_lower;
    const auto upper = adapted->problem.input_upper;
    const auto original_frame = *request.course_frame.knots;

    const auto result = adapter::relinearize_around_primal(request, primal, adapted->problem);

    ASSERT_TRUE(result.applied);
    EXPECT_TRUE((primal.array() == original.array()).all());
    EXPECT_TRUE((adapted->problem.state_lower.array() == lower.array()).all());
    EXPECT_TRUE((adapted->problem.input_upper.array() == upper.array()).all());
    EXPECT_TRUE(model::course_frame_matches(request.course_frame, original_frame, 50.0));
    // The affine progress equation still exposes the original residual; a
    // tangent-domain selection must not rewrite the raw predicted trajectory.
    const auto & tangent = adapted->problem.linearizations.back();
    const auto next = tangent.state_matrix * primal.segment<model::kStateDimension>(model::kStateDimension) +
      tangent.input_matrix * primal.segment<model::kInputDimension>(last_input) - tangent.equality_offset;
    EXPECT_GT(next[model::kProgressIndex], terminal_progress + 1e-9);
  }
}

TEST(MpccRateResolvedAdapter, InitialTangentRespectsCourseDomainWithoutChangingObjectiveOrBounds)
{
  namespace geometry = multi_purpose_mpc_ros::mpc_stage_geometry;
  auto request = curved_request(2);
  for (auto & input : request.inputs) {
    input.path_curvature_radpm = 0.0;
    input.reference[adapter::kLegacyCurvatureIndex] = 0.0;
  }
  request.inputs.back().reference[model::kVirtualProgressSpeedIndex] = 3.13;
  request.course_frame = {
    std::make_shared<const std::vector<geometry::CourseFrameKnot>>(
      std::vector<geometry::CourseFrameKnot>{
        {50.0, 89613.0, 43161.0, 0.0, 0},
        {50.3, 89613.3, 43161.0, 0.0, 1},
        {50.6, 89613.6, 43161.0, 0.0, 2}}), 50.0};
  // A soft speed reference need not interpolate adjacent stage positions.
  // The old initial adapter evaluates theta=0.3, nu=3.13, dt=0.1 at
  // theta=0.613 outside the immutable frame, before there is any QP iterate.
  const auto built = adapter::build(request, kSolverTolerance);
  ASSERT_TRUE(built.has_value());
  const auto & problem = built->problem;
  const int input = model::kInputDimension;
  EXPECT_DOUBLE_EQ(problem.input_reference[input + model::kVirtualProgressSpeedIndex], 3.13);
  EXPECT_DOUBLE_EQ(problem.input_lower[input + model::kVirtualProgressSpeedIndex], 0.0);
  EXPECT_DOUBLE_EQ(problem.input_upper[input + model::kVirtualProgressSpeedIndex], 6.0);
  EXPECT_DOUBLE_EQ(problem.state_reference[model::kStateDimension + model::kProgressIndex], 0.3);
  EXPECT_DOUBLE_EQ(problem.state_weight[model::kStateDimension + model::kProgressIndex], 1.0);
  const auto & tangent = problem.linearizations.back();
  model::StateVector state = problem.state_reference.segment<model::kStateDimension>(model::kStateDimension);
  model::InputVector control = problem.input_reference.tail<model::kInputDimension>();
  const model::StateVector unchanged_soft_endpoint =
    tangent.state_matrix * state + tangent.input_matrix * control - tangent.equality_offset;
  EXPECT_NEAR(unchanged_soft_endpoint[model::kProgressIndex], 0.613, 1e-9);
  control[model::kVirtualProgressSpeedIndex] = 3.0;
  const model::StateVector supported_endpoint =
    tangent.state_matrix * state + tangent.input_matrix * control - tangent.equality_offset;
  EXPECT_NEAR(supported_endpoint[model::kProgressIndex], 0.6, 1e-9);

  // No valid tangent exists if the declared input cannot stay in the frame.
  request.inputs.back().lower[model::kVirtualProgressSpeedIndex] = 3.1;
  adapter::BuildDiagnostic rejected;
  EXPECT_FALSE(adapter::build(request, kSolverTolerance, &rejected).has_value());
  EXPECT_EQ(rejected.reason, adapter::RejectReason::LinearizationUnavailable);
  EXPECT_EQ(rejected.stage, 1);
}

TEST(MpccRateResolvedAdapter, RejectsEmptyTangentInputAndCourseDomainIntersection)
{
  namespace geometry = multi_purpose_mpc_ros::mpc_stage_geometry;
  auto request = curved_request(2);
  for (auto & input : request.inputs) {
    input.path_curvature_radpm = 0.0;
    input.reference[adapter::kLegacyCurvatureIndex] = 0.0;
  }
  request.course_frame = {
    std::make_shared<const std::vector<geometry::CourseFrameKnot>>(
      std::vector<geometry::CourseFrameKnot>{{0.0, 0.0, 0.0, 0.0, 0}, {0.6, 0.6, 0.0, 0.0, 1}}), 0.0};
  auto adapted = adapter::build(request, kSolverTolerance);
  ASSERT_TRUE(adapted.has_value());
  constexpr int state_values = 3 * model::kStateDimension;
  Eigen::VectorXd primal(state_values + 2 * model::kInputDimension);
  primal.head(state_values) = adapted->problem.state_reference;
  primal.tail(2 * model::kInputDimension) = adapted->problem.input_reference;
  adapted->problem.input_lower[model::kInputDimension + model::kVirtualProgressSpeedIndex] = 4.0;
  const auto original = adapted->problem.linearizations.back().equality_offset;

  const auto result = adapter::relinearize_around_primal(request, primal, adapted->problem);

  EXPECT_FALSE(result.applied);
  EXPECT_EQ(result.reason, adapter::RelinearizationReason::LinearizationUnavailable);
  EXPECT_EQ(result.stage, 1);
  EXPECT_TRUE((adapted->problem.linearizations.back().equality_offset.array() == original.array()).all());
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
  request.inputs.front().lower[0] = 0.0;
  request.inputs.front().upper[0] = 0.0;
  EXPECT_FALSE(adapter::build(request, kSolverTolerance).has_value());
}

TEST(MpccRateResolvedAdapter, InitialStateEqualityOwnsStageZeroBounds)
{
  auto request = curved_request();
  request.states.front().upper[model::kProgressIndex] = -0.1;
  adapter::BuildDiagnostic diagnostic;

  const auto result = adapter::build(request, kSolverTolerance, &diagnostic);

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(diagnostic.reason, adapter::RejectReason::None);
  for (int element = 0; element < adapter::kLegacyStateDimension; ++element) {
    EXPECT_DOUBLE_EQ(
      result->problem.state_lower[element], request.initial_state[element]);
    EXPECT_DOUBLE_EQ(
      result->problem.state_upper[element], request.initial_state[element]);
  }
  const int future_progress =
    model::kStateDimension + model::kProgressIndex;
  EXPECT_DOUBLE_EQ(
    result->problem.state_lower[future_progress],
    request.states[1].lower[model::kProgressIndex]);
  EXPECT_DOUBLE_EQ(
    result->problem.state_upper[future_progress],
    request.states[1].upper[model::kProgressIndex]);
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

  const double margin = exact_inset_margin(1.0);
  EXPECT_DOUBLE_EQ(result->first_steering_rate_physical_upper_radps, 0.0);
  EXPECT_NEAR(result->first_steering_rate_solver_upper_radps, -margin, 1e-12);
  const double accepted_upper_residual =
    solver::kSolvedInaccurateToleranceMultiplier *
    (kSolverTolerance.absolute + kSolverTolerance.relative *
    std::abs(result->first_steering_rate_solver_upper_radps));
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
  const double prefix_margin = exact_inset_margin(0.8);
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

TEST(MpccRateResolvedAdapter, SoftRacingSpeedCannotReplaceTheInitialPhysicalTrajectory)
{
  auto request = curved_request(20);
  request.initial_state << 0.9194310400827391, -0.48353979878083253,
    -0.1091854402811685, 0.2702067330722648, 0.0;
  request.current_steering_rad = 0.32403313324426525;
  request.current_response_steering_rad = 0.32549134574657773;
  request.current_lateral_velocity_mps = 0.012246213131490612;
  request.current_yaw_rate_radps = 0.07926222390056174;
  for (auto & state : request.states) {
    state.reference[model::kVelocityIndex] = request.initial_state[model::kVelocityIndex];
    state.upper[model::kVelocityIndex] = 10.0;
  }
  const auto slow_reference = adapter::build(request, kSolverTolerance);
  ASSERT_TRUE(slow_reference);
  for (auto & state : request.states) {
    state.reference[model::kVelocityIndex] = 9.853461939927946;
  }
  const auto racing_reference = adapter::build(request, kSolverTolerance);
  ASSERT_TRUE(racing_reference);
  EXPECT_DOUBLE_EQ(racing_reference->problem.initial_state[model::kVelocityIndex],
    request.initial_state[model::kVelocityIndex]);
  EXPECT_DOUBLE_EQ(racing_reference->problem.state_reference[
    model::kStateDimension + model::kVelocityIndex], 9.853461939927946);
  for (int stage = 0; stage < request.horizon_steps; ++stage) {
    SCOPED_TRACE(stage);
    const auto & slow = slow_reference->problem.linearizations[stage];
    const auto & racing = racing_reference->problem.linearizations[stage];
    EXPECT_TRUE(slow.state_matrix.isApprox(racing.state_matrix, 1e-12));
    EXPECT_TRUE(slow.input_matrix.isApprox(racing.input_matrix, 1e-12));
    EXPECT_TRUE(slow.equality_offset.isApprox(racing.equality_offset, 1e-12));
  }
}

TEST(MpccRateResolvedAdapter, InitialTangentsShareOneNativeBodyTrajectory)
{
  auto request = curved_request(8);
  request.initial_state << 0.9, -0.4, -0.1, 0.27, 0.0;
  request.current_steering_rad = 0.3;
  request.current_response_steering_rad = 0.31;
  request.current_lateral_velocity_mps = 0.012;
  request.current_yaw_rate_radps = 0.08;
  for (auto & input : request.inputs) {
    input.reference[adapter::kLegacyCurvatureIndex] =
      request.curvature_reference_gain * std::tan(request.current_steering_rad) /
      request.wheelbase_m;
  }
  const auto built = adapter::build(request, kSolverTolerance);
  ASSERT_TRUE(built);
  model::StateVector body = built->problem.initial_state;
  for (int stage = 0; stage < request.horizon_steps; ++stage) {
    SCOPED_TRACE(stage);
    const auto & input = request.inputs[stage];
    const double progress_speed = std::max(0.0,
      body[3] * std::cos(body[2]) - body[7] * std::sin(body[2]));
    const auto exact = model::evaluate_temporal_frenet_transition(
      model::LinearizationRequest{body[0], body[1], body[2], body[3], body[4],
        body[5], body[6], 0.0, 0.0, progress_speed, input.path_curvature_radpm,
        request.wheelbase_m, input.stage_dt_sec, request.minimum_frenet_denominator,
        request.minimum_stage_dt_sec, request.maximum_stage_dt_sec, request.course_frame,
        body[7], body[8], request.vehicle_model});
    ASSERT_TRUE(exact);
    model::InputVector control;
    control << 0.0, 0.0, progress_speed;
    const auto & tangent = built->problem.linearizations[stage];
    const model::StateVector affine = tangent.state_matrix * body +
      tangent.input_matrix * control - tangent.equality_offset;
    EXPECT_TRUE(affine.isApprox(exact->next_state, 1e-8)) <<
      "affine/native error=" << (affine - exact->next_state).transpose();
    body = exact->next_state;
  }
}

TEST(MpccRateResolvedAdapter, FullRestSeedMovesTowardTheSoftSpeedWithinOriginalInputBounds)
{
  auto request = curved_request(8);
  request.initial_state.setZero();
  const auto built = adapter::build(request, kSolverTolerance);
  ASSERT_TRUE(built);
  model::StateVector body = built->problem.initial_state;
  for (int stage = 0; stage < request.horizon_steps; ++stage) {
    SCOPED_TRACE(stage);
    const auto & input = request.inputs[stage];
    const int offset = model::kInputDimension * stage;
    const double acceleration = built->problem.input_upper[offset + model::kAccelerationIndex];
    const double progress_speed = std::max(0.0,
      body[3] * std::cos(body[2]) - body[7] * std::sin(body[2]));
    const auto exact = model::evaluate_temporal_frenet_transition(
      model::LinearizationRequest{body[0], body[1], body[2], body[3], body[4],
        body[5], body[6], acceleration, 0.0, progress_speed, input.path_curvature_radpm,
        request.wheelbase_m, input.stage_dt_sec, request.minimum_frenet_denominator,
        request.minimum_stage_dt_sec, request.maximum_stage_dt_sec, request.course_frame,
        body[7], body[8], request.vehicle_model});
    ASSERT_TRUE(exact);
    // This short horizon cannot reach the 3m/s target with the original <=1m/s2
    // input envelope. The seed therefore uses that bounded acceleration and
    // still cannot jump to the soft target speed.
    ASSERT_LT(body[3] + acceleration * input.stage_dt_sec, 3.0);
    model::InputVector control;
    control << acceleration, 0.0, progress_speed;
    const auto & tangent = built->problem.linearizations[stage];
    const model::StateVector affine = tangent.state_matrix * body +
      tangent.input_matrix * control - tangent.equality_offset;
    EXPECT_TRUE(affine.isApprox(exact->next_state, 1e-8)) <<
      "affine/native error=" << (affine - exact->next_state).transpose();
    EXPECT_DOUBLE_EQ(built->problem.input_reference[offset + model::kAccelerationIndex], 0.0);
    EXPECT_DOUBLE_EQ(built->problem.state_reference[
      (stage + 1) * model::kStateDimension + model::kVelocityIndex], 3.0);
    body = exact->next_state;
  }
}

TEST(MpccRateResolvedAdapter, FullRestWithoutAForwardGoalKeepsTheDeclaredBrakingSeed)
{
  auto request = curved_request();
  request.initial_state.setZero();
  for (auto & state : request.states) {
    state.reference[model::kVelocityIndex] = 3.0;
    state.upper[model::kVelocityIndex] = 0.0;
  }
  for (auto & input : request.inputs) {
    input.reference[model::kAccelerationIndex] = -0.5;
  }
  const auto built = adapter::build(request, kSolverTolerance);
  ASSERT_TRUE(built);
  const model::StateVector rest = model::StateVector::Zero();
  model::InputVector braking;
  braking << -0.5, 0.0, 0.0;
  for (const auto & tangent : built->problem.linearizations) {
    const model::StateVector next = tangent.state_matrix * rest +
      tangent.input_matrix * braking - tangent.equality_offset;
    EXPECT_TRUE(next.isZero(1e-12));
  }
}


TEST(MpccRateResolvedAdapter, ReferenceInitializerUsesNativeSteeringWithoutChangingTheQpObjectiveOrBounds)
{
  auto request = curved_request();
  const auto original = adapter::build(request, kSolverTolerance);
  ASSERT_TRUE(original);
  request.initial_tangent_policy = adapter::InitialTangentPolicy::ReferenceSteeringWithRestLaunch;
  const auto reference = adapter::build(request, kSolverTolerance);
  ASSERT_TRUE(reference);
  const auto & p = reference->problem;
  EXPECT_TRUE((p.state_reference.array() == original->problem.state_reference.array()).all());
  EXPECT_TRUE((p.state_weight.array() == original->problem.state_weight.array()).all());
  EXPECT_TRUE((p.input_reference.array() == original->problem.input_reference.array()).all());
  EXPECT_TRUE((p.input_weight.array() == original->problem.input_weight.array()).all());
  EXPECT_TRUE((p.additional_linear_cost.array() == original->problem.additional_linear_cost.array()).all());
  EXPECT_TRUE((p.state_lower.array() == original->problem.state_lower.array()).all());
  EXPECT_TRUE((p.state_upper.array() == original->problem.state_upper.array()).all());
  EXPECT_TRUE((p.input_lower.array() == original->problem.input_lower.array()).all());
  EXPECT_TRUE((p.input_upper.array() == original->problem.input_upper.array()).all());
  const auto & input = request.inputs.front();
  ASSERT_GT(reference->steering_reference_rad[1] / input.stage_dt_sec,
    p.input_upper[model::kSteeringRateIndex]);
  const auto & x = p.initial_state;
  const auto expected = model::linearize_temporal_frenet(
    model::LinearizationRequest{x[0],x[1],x[2],x[3],x[4],x[5],x[6],
      0.0,p.input_upper[model::kSteeringRateIndex],x[3],input.path_curvature_radpm,
      request.wheelbase_m,input.stage_dt_sec,request.minimum_frenet_denominator,
      request.minimum_stage_dt_sec,request.maximum_stage_dt_sec,request.course_frame,
      x[7],x[8],request.vehicle_model});
  ASSERT_TRUE(expected);
  EXPECT_TRUE(p.linearizations.front().state_matrix.isApprox(expected->state_matrix, 1e-12));
  EXPECT_TRUE(p.linearizations.front().input_matrix.isApprox(expected->input_matrix, 1e-12));
  EXPECT_TRUE(p.linearizations.front().equality_offset.isApprox(expected->equality_offset, 1e-12));
  EXPECT_FALSE(p.linearizations.front().state_matrix.isApprox(original->problem.linearizations.front().state_matrix, 1e-12));
}

TEST(MpccRateResolvedAdapter, ReferenceInitializerCanLaunchAfterNativePredictionReachesRest)
{
  auto request = curved_request();
  request.initial_state.setZero();
  request.initial_state[model::kVelocityIndex] = 0.0037;
  for (auto & input : request.inputs) {
    input.reference[adapter::kLegacyCurvatureIndex] = 0.0;
    input.path_curvature_radpm = 0.0;
  }
  request.initial_tangent_policy = adapter::InitialTangentPolicy::ReferenceSteeringWithRestLaunch;
  const auto built = adapter::build(request, kSolverTolerance);
  ASSERT_TRUE(built);
  const auto & p = built->problem;
  const auto & x = p.initial_state;
  const auto & first = request.inputs.front();
  const auto coast = model::evaluate_temporal_frenet_transition(
    model::LinearizationRequest{x[0],x[1],x[2],x[3],x[4],x[5],x[6],0,0,x[3],0,
      request.wheelbase_m,first.stage_dt_sec,request.minimum_frenet_denominator,
      request.minimum_stage_dt_sec,request.maximum_stage_dt_sec,request.course_frame,
      x[7],x[8],request.vehicle_model});
  ASSERT_TRUE(coast);
  const auto & rest = coast->next_state;
  ASSERT_DOUBLE_EQ(rest[model::kVelocityIndex], 0.0);
  ASSERT_DOUBLE_EQ(rest[model::kLateralVelocityIndex], 0.0);
  ASSERT_DOUBLE_EQ(rest[model::kYawRateIndex], 0.0);
  const double acceleration = p.input_upper[model::kInputDimension + model::kAccelerationIndex];
  const auto launch = model::evaluate_temporal_frenet_transition(
    model::LinearizationRequest{rest[0],rest[1],rest[2],rest[3],rest[4],rest[5],rest[6],
      acceleration,0,0,0,request.wheelbase_m,request.inputs[1].stage_dt_sec,
      request.minimum_frenet_denominator,request.minimum_stage_dt_sec,request.maximum_stage_dt_sec,
      request.course_frame,rest[7],rest[8],request.vehicle_model});
  ASSERT_TRUE(launch);
  ASSERT_GT(launch->next_state[model::kVelocityIndex], 0.0);
  model::InputVector u; u << acceleration, 0.0, 0.0;
  const auto & tangent = p.linearizations[1];
  EXPECT_TRUE((tangent.state_matrix * rest + tangent.input_matrix * u - tangent.equality_offset)
    .isApprox(launch->next_state, 1e-8));
}

TEST(MpccRateResolvedAdapter, UnknownInitializerIsRejected)
{
  auto request = curved_request();
  request.initial_tangent_policy = static_cast<adapter::InitialTangentPolicy>(99);
  EXPECT_FALSE(adapter::build(request, kSolverTolerance));
}
