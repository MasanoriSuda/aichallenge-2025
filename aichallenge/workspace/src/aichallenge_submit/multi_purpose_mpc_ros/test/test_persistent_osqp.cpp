#include <multi_purpose_mpc_ros/persistent_osqp.hpp>

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <cmath>
#include <limits>
#include <optional>
#include <vector>

namespace multi_purpose_mpc_ros::persistent_osqp
{
namespace
{

Eigen::SparseMatrix<double>
diagonal_matrix(const std::vector<double> & diagonal)
{
  Eigen::SparseMatrix<double> matrix(
    static_cast<Eigen::Index>(diagonal.size()),
    static_cast<Eigen::Index>(diagonal.size()));
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(diagonal.size());
  for (std::size_t index = 0U; index < diagonal.size(); ++index) {
    triplets.emplace_back(
      static_cast<Eigen::Index>(index),
      static_cast<Eigen::Index>(index), diagonal[index]);
  }
  matrix.setFromTriplets(triplets.begin(), triplets.end());
  return matrix;
}

Eigen::SparseMatrix<double> identity_constraints(const Eigen::Index dimension)
{
  Eigen::SparseMatrix<double> matrix(dimension, dimension);
  matrix.setIdentity();
  return matrix;
}

Eigen::SparseMatrix<double> scalar_constraint(const double value)
{
  Eigen::SparseMatrix<double> matrix(1, 1);
  const std::vector<Eigen::Triplet<double>> triplets{{0, 0, value}};
  matrix.setFromTriplets(triplets.begin(), triplets.end());
  return matrix;
}

TEST(PersistentOsqpSolver, ExposesImmutablePhysicalConstraintTolerance)
{
  PersistentOsqpSolver solver(
    ConstraintPreconditioningPolicy::RowToleranceNormalized);
  const auto before = solver.physical_constraint_tolerance();
  EXPECT_TRUE(std::isfinite(before.absolute));
  EXPECT_GT(before.absolute, 0.0);
  EXPECT_TRUE(std::isfinite(before.relative));
  EXPECT_GE(before.relative, 0.0);
  EXPECT_LT(before.relative, 1.0);

  const auto outcome = solver.solve(
    diagonal_matrix({1.0}), scalar_constraint(1.0),
    Eigen::VectorXd::Zero(1), Eigen::VectorXd::Constant(1, -1.0),
    Eigen::VectorXd::Constant(1, 1.0));
  ASSERT_TRUE(outcome.result.has_value()) << outcome.failure_detail;
  const auto after = solver.physical_constraint_tolerance();
  EXPECT_DOUBLE_EQ(after.absolute, before.absolute);
  EXPECT_DOUBLE_EQ(after.relative, before.relative);
}

TEST(PersistentOsqpWarmStart, ShiftsMpcPrimalAndDualByOneStage) {
  WarmStart previous;
  previous.primal = Eigen::VectorXd(5);
  previous.primal << 0.0, 1.0, 2.0, 3.0, 4.0;
  previous.dual = Eigen::VectorXd(10);
  previous.dual << 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;

  const auto shifted = shift_mpc_warm_start(previous, 2U, 1U, 1U);

  ASSERT_TRUE(shifted.has_value());
  Eigen::VectorXd expected_primal(5);
  expected_primal << 1.0, 2.0, 2.0, 4.0, 4.0;
  Eigen::VectorXd expected_dual(10);
  expected_dual << 1.0, 2.0, 2.0, 4.0, 5.0, 5.0, 7.0, 7.0, 9.0, 9.0;
  EXPECT_TRUE(shifted->primal.isApprox(expected_primal));
  EXPECT_TRUE(shifted->dual.isApprox(expected_dual));
}

TEST(PersistentOsqpWarmStart, PreservesMpcPrimalAndDualWhenGeometryDidNotAdvance)
{
  WarmStart previous;
  previous.primal = Eigen::VectorXd(5);
  previous.primal << 0.0, 1.0, 2.0, 3.0, 4.0;
  previous.dual = Eigen::VectorXd(10);
  previous.dual << 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;

  const auto shifted = shift_mpc_warm_start(
    previous, 2U, 1U, 1U, {}, 0U);

  ASSERT_TRUE(shifted.has_value());
  EXPECT_TRUE(shifted->primal.isApprox(previous.primal));
  EXPECT_TRUE(shifted->dual.isApprox(previous.dual));
}

TEST(PersistentOsqpWarmStart, ShiftsMpcPrimalAndDualByExactStageAdvance)
{
  WarmStart previous;
  previous.primal = Eigen::VectorXd(7);
  previous.primal << 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;
  previous.dual = Eigen::VectorXd(14);
  previous.dual <<
    0.0, 1.0, 2.0, 3.0,
    4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0,
    11.0, 12.0, 13.0;

  const auto shifted = shift_mpc_warm_start(
    previous, 3U, 1U, 1U, {}, 2U);

  ASSERT_TRUE(shifted.has_value());
  Eigen::VectorXd expected_primal(7);
  expected_primal << 2.0, 3.0, 3.0, 3.0, 6.0, 6.0, 6.0;
  Eigen::VectorXd expected_dual(14);
  expected_dual <<
    2.0, 3.0, 3.0, 3.0,
    6.0, 7.0, 7.0, 7.0,
    10.0, 10.0, 10.0,
    13.0, 13.0, 13.0;
  EXPECT_TRUE(shifted->primal.isApprox(expected_primal));
  EXPECT_TRUE(shifted->dual.isApprox(expected_dual));
}

TEST(PersistentOsqpWarmStart, RejectsStageAdvanceOutsideThePreviousHorizon)
{
  const WarmStart previous{
    Eigen::VectorXd::Zero(5), Eigen::VectorXd::Zero(10)};

  EXPECT_FALSE(
    shift_mpc_warm_start(previous, 2U, 1U, 1U, {}, 3U).has_value());
}

TEST(PersistentOsqpWarmStart, PreservesSingleStageLegacyShiftContract)
{
  WarmStart previous;
  previous.primal = Eigen::VectorXd(3);
  previous.primal << 1.0, 2.0, 3.0;
  previous.dual = Eigen::VectorXd(6);
  previous.dual << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;

  const auto shifted = shift_mpc_warm_start(previous, 1U, 1U, 1U);

  ASSERT_TRUE(shifted.has_value());
  Eigen::VectorXd expected_primal(3);
  expected_primal << 2.0, 2.0, 3.0;
  Eigen::VectorXd expected_dual(6);
  expected_dual << 2.0, 2.0, 4.0, 4.0, 5.0, 6.0;
  EXPECT_TRUE(shifted->primal.isApprox(expected_primal));
  EXPECT_TRUE(shifted->dual.isApprox(expected_dual));
}

TEST(PersistentOsqpWarmStart, RejectsMalformedOrNonFiniteState) {
  WarmStart malformed{Eigen::VectorXd::Zero(5), Eigen::VectorXd::Zero(9)};
  EXPECT_FALSE(shift_mpc_warm_start(malformed, 2U, 1U, 1U).has_value());

  WarmStart non_finite{Eigen::VectorXd::Zero(5), Eigen::VectorXd::Zero(10)};
  non_finite.primal[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(shift_mpc_warm_start(non_finite, 2U, 1U, 1U).has_value());
}

TEST(PersistentOsqpWarmStart, ShiftsDeclaredTrailingStageBlock)
{
  WarmStart previous;
  previous.primal = Eigen::VectorXd(5);
  previous.primal << 0.0, 1.0, 2.0, 3.0, 4.0;
  previous.dual = Eigen::VectorXd(13);
  previous.dual <<
    0.0, 1.0, 2.0,
    3.0, 4.0, 5.0,
    6.0, 7.0,
    8.0, 9.0,
    10.0, 11.0, 12.0;

  const auto shifted = shift_mpc_warm_start(
    previous, 2U, 1U, 1U,
    std::vector<DualStageBlockLayout>{{3U, 1U}});

  ASSERT_TRUE(shifted.has_value());
  Eigen::VectorXd expected_primal(5);
  expected_primal << 1.0, 2.0, 2.0, 4.0, 4.0;
  Eigen::VectorXd expected_dual(13);
  expected_dual <<
    1.0, 2.0, 2.0,
    4.0, 5.0, 5.0,
    7.0, 7.0,
    9.0, 9.0,
    11.0, 12.0, 12.0;
  EXPECT_TRUE(shifted->primal.isApprox(expected_primal));
  EXPECT_TRUE(shifted->dual.isApprox(expected_dual));
}

TEST(PersistentOsqpWarmStart, RejectsUndeclaredOrMalformedTrailingRows)
{
  const WarmStart with_trailing_rows{
    Eigen::VectorXd::Zero(5), Eigen::VectorXd::Zero(13)};

  EXPECT_FALSE(
    shift_mpc_warm_start(with_trailing_rows, 2U, 1U, 1U).has_value());
  EXPECT_FALSE(
    shift_mpc_warm_start(
      with_trailing_rows, 2U, 1U, 1U,
      std::vector<DualStageBlockLayout>{{0U, 1U}}).has_value());
  EXPECT_FALSE(
    shift_mpc_warm_start(
      with_trailing_rows, 2U, 1U, 1U,
      std::vector<DualStageBlockLayout>{{3U, 0U}}).has_value());
}

TEST(CertifiedWarmStartStore, PublishesAndConsumesExactlyOnce)
{
  CertifiedWarmStartStore store;
  WarmStart accepted{
    Eigen::VectorXd::Constant(2, 1.0),
    Eigen::VectorXd::Constant(3, -2.0)};

  ASSERT_TRUE(store.publish(accepted, 10.0, 42.0));
  EXPECT_TRUE(store.available());
  const auto consumed = store.consume_fresh(10.1, 0.5);
  ASSERT_TRUE(consumed.has_value());
  EXPECT_TRUE(consumed->value.primal.isApprox(accepted.primal));
  EXPECT_TRUE(consumed->value.dual.isApprox(accepted.dual));
  EXPECT_DOUBLE_EQ(consumed->progress_origin_m, 42.0);
  EXPECT_FALSE(store.available());
  EXPECT_FALSE(store.consume_fresh(10.2, 0.5).has_value());
}

TEST(CertifiedWarmStartStore, RejectsStaleOrMalformedPublication)
{
  CertifiedWarmStartStore store;
  const WarmStart accepted{
    Eigen::VectorXd::Constant(2, 1.0),
    Eigen::VectorXd::Constant(3, -2.0)};
  ASSERT_TRUE(store.publish(accepted, 10.0, 42.0));
  EXPECT_FALSE(store.consume_fresh(10.6, 0.5).has_value());
  EXPECT_FALSE(store.available());

  WarmStart malformed = accepted;
  malformed.primal[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(store.publish(malformed, 11.0, 43.0));
  EXPECT_FALSE(store.publish(accepted, 11.0, std::numeric_limits<double>::infinity()));
  EXPECT_FALSE(store.available());
}

TEST(PersistentOsqpConstraintResiduals, KeepsMixedUnitRowsOnIndependentScales)
{
  Eigen::SparseMatrix<double> constraints(2, 2);
  constraints.setIdentity();
  Eigen::VectorXd primal(2);
  primal << 0.1, 1000.5;
  Eigen::VectorXd lower(2);
  lower << 0.0, 1000.0;
  const Eigen::VectorXd upper = lower;

  const auto report = evaluate_constraint_residuals(
    constraints, primal, lower, upper, 1e-3, 1e-3);

  ASSERT_TRUE(report.has_value());
  EXPECT_NEAR(report->violation[0], 0.1, 1e-12);
  EXPECT_NEAR(report->violation[1], 0.5, 1e-12);
  EXPECT_NEAR(report->tolerance[0], 0.0011, 1e-12);
  EXPECT_NEAR(report->tolerance[1], 1.0015, 1e-12);
  EXPECT_GT(report->violation[0], report->tolerance[0]);
  EXPECT_LT(report->violation[1], report->tolerance[1]);
  EXPECT_EQ(report->maximum_normalized_row, 0);
  EXPECT_GT(report->maximum_normalized_violation, 90.0);
}

TEST(PersistentOsqpConstraintResiduals, RejectsMalformedToleranceContract)
{
  Eigen::SparseMatrix<double> constraints(1, 1);
  constraints.setIdentity();
  const Eigen::VectorXd primal = Eigen::VectorXd::Zero(1);
  const Eigen::VectorXd bound = Eigen::VectorXd::Zero(1);

  EXPECT_FALSE(evaluate_constraint_residuals(
    constraints, primal, bound, bound, -1.0, 1e-3).has_value());
  EXPECT_FALSE(evaluate_constraint_residuals(
    constraints, primal, bound, bound, 1e-3, 1e-3, 0.0).has_value());
}

TEST(PersistentOsqpConstraintResiduals, ReportsWorstRowInPhysicalCoordinates)
{
  const auto constraints = identity_constraints(2);
  Eigen::VectorXd primal(2);
  primal << 0.397, 500.0;
  Eigen::VectorXd lower(2);
  lower << 0.0, 0.0;
  Eigen::VectorXd upper(2);
  upper << 0.25, 1000.0;

  const auto report = evaluate_constraint_residuals(
    constraints, primal, lower, upper, 1e-3, 1e-3);
  ASSERT_TRUE(report.has_value());
  const auto diagnostic = make_constraint_failure_diagnostic(
    report.value(), constraints * primal, lower, upper);

  ASSERT_TRUE(diagnostic.has_value());
  EXPECT_EQ(diagnostic->row, 0);
  EXPECT_DOUBLE_EQ(diagnostic->value, 0.397);
  EXPECT_DOUBLE_EQ(diagnostic->projected, 0.25);
  EXPECT_DOUBLE_EQ(diagnostic->lower_bound, 0.0);
  EXPECT_DOUBLE_EQ(diagnostic->upper_bound, 0.25);
  EXPECT_NEAR(diagnostic->violation, 0.147, 1e-12);
  EXPECT_NEAR(diagnostic->tolerance, 0.001397, 1e-12);
  EXPECT_GT(diagnostic->normalized_violation, 100.0);
}

TEST(PersistentOsqpScaling, DerivesPhysicalUnitsFromFiniteBoxBounds)
{
  Eigen::VectorXd lower(4);
  lower << -2.0, -std::numeric_limits<double>::infinity(), 0.0, 0.0;
  Eigen::VectorXd upper(4);
  upper << 1.0, std::numeric_limits<double>::infinity(), 20.0, 0.25;

  const auto scaling = derive_box_variable_coordinate_scaling(lower, upper);

  ASSERT_TRUE(scaling.has_value());
  Eigen::VectorXd expected(4);
  expected << 2.0, 1.0, 20.0, 0.25;
  EXPECT_TRUE(
    scaling->physical_units_per_solver_unit.isApprox(expected, 1e-12));
}

TEST(PersistentOsqpScaling, RejectsMalformedBoxBounds)
{
  Eigen::VectorXd lower(2);
  lower << 0.0, 2.0;
  Eigen::VectorXd upper(2);
  upper << 1.0, 1.0;
  EXPECT_FALSE(
    derive_box_variable_coordinate_scaling(lower, upper).has_value());
  EXPECT_FALSE(
    derive_box_variable_coordinate_scaling(
      Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(2)).has_value());
}

TEST(PersistentOsqpSolver, ReusesWorkspaceAndAppliesWarmStart) {
  PersistentOsqpSolver solver;
  const auto quadratic = diagonal_matrix({1.0});
  const auto constraints = identity_constraints(1);
  Eigen::VectorXd lower(1);
  Eigen::VectorXd upper(1);
  lower << 0.0;
  upper << 2.0;
  Eigen::VectorXd first_cost(1);
  first_cost << -1.0;

  const auto first =
    solver.solve(quadratic, constraints, first_cost, lower, upper);

  ASSERT_TRUE(first.result.has_value()) << first.failure_detail;
  EXPECT_TRUE(first.telemetry.setup_performed);
  EXPECT_FALSE(first.telemetry.update_performed);
  EXPECT_NEAR(first.result->primal[0], 1.0, 1e-2);

  Eigen::VectorXd second_cost(1);
  second_cost << -0.5;
  const WarmStart warm_start{first.result->primal, first.result->dual};
  const auto second = solver.solve(
    quadratic, constraints, second_cost, lower,
    upper, warm_start);

  ASSERT_TRUE(second.result.has_value()) << second.failure_detail;
  EXPECT_FALSE(second.telemetry.setup_performed);
  EXPECT_TRUE(second.telemetry.update_performed);
  EXPECT_TRUE(second.telemetry.warm_start_applied);
  EXPECT_NEAR(second.result->primal[0], 0.5, 1e-2);

  const WarmStart malformed_warm_start{Eigen::VectorXd::Zero(2),
    Eigen::VectorXd::Zero(2)};
  const auto third = solver.solve(
    quadratic, constraints, first_cost, lower,
    upper, malformed_warm_start);
  ASSERT_TRUE(third.result.has_value()) << third.failure_detail;
  EXPECT_TRUE(third.telemetry.update_performed);
  EXPECT_TRUE(third.telemetry.warm_start_rejected);
  EXPECT_FALSE(third.telemetry.warm_start_applied);
  EXPECT_NEAR(third.result->primal[0], 1.0, 1e-2);
}

TEST(PersistentOsqpSolver, ReportsSuccessfulConvergenceProvenance)
{
  PersistentOsqpSolver solver;
  const auto outcome = solver.solve(
    diagonal_matrix({1.0}), identity_constraints(1),
    Eigen::VectorXd::Constant(1, -1.0),
    Eigen::VectorXd::Constant(1, 0.0),
    Eigen::VectorXd::Constant(1, 2.0));

  ASSERT_TRUE(outcome.result.has_value()) << outcome.failure_detail;
  EXPECT_TRUE(std::isfinite(outcome.telemetry.primal_residual));
  EXPECT_TRUE(std::isfinite(outcome.telemetry.dual_residual));
  EXPECT_TRUE(std::isfinite(outcome.telemetry.objective_value));
  EXPECT_TRUE(std::isfinite(outcome.telemetry.rho_estimate));
  EXPECT_GT(outcome.telemetry.absolute_tolerance, 0.0);
  EXPECT_GT(outcome.telemetry.relative_tolerance, 0.0);
  EXPECT_GT(outcome.telemetry.scaling_iterations, 0);
  EXPECT_FALSE(outcome.telemetry.scaled_termination);
  EXPECT_FALSE(outcome.telemetry.row_tolerance_preconditioned);
  EXPECT_DOUBLE_EQ(outcome.telemetry.maximum_row_scale, 1.0);
  EXPECT_TRUE(std::isfinite(outcome.telemetry.physical_constraint_scale));
  EXPECT_TRUE(std::isfinite(outcome.telemetry.physical_global_tolerance));
  EXPECT_GT(outcome.telemetry.physical_global_tolerance, 0.0);
  EXPECT_GE(outcome.result->maximum_normalized_constraint_row, -1);
}

TEST(PersistentOsqpSolver, RowToleranceNormalizationClosesMixedUnitToleranceLeak)
{
  const auto quadratic = diagonal_matrix({1.0, 1.0});
  const auto constraints = identity_constraints(2);
  Eigen::VectorXd cost(2);
  cost << -0.4, -500.0;
  Eigen::VectorXd lower(2);
  lower << 0.0, 0.0;
  Eigen::VectorXd upper(2);
  upper << 0.25, 1000.0;

  PersistentOsqpSolver baseline;
  PersistentOsqpSolver normalized(
    ConstraintPreconditioningPolicy::RowToleranceNormalized);
  const auto baseline_outcome = baseline.solve(
    quadratic, constraints, cost, lower, upper);
  const auto normalized_outcome = normalized.solve(
    quadratic, constraints, cost, lower, upper);

  ASSERT_TRUE(baseline_outcome.result.has_value())
    << baseline_outcome.failure_detail;
  ASSERT_TRUE(normalized_outcome.result.has_value())
    << normalized_outcome.failure_detail;
  EXPECT_GT(baseline_outcome.result->primal[0], 0.35);
  EXPECT_GT(
    baseline_outcome.result->maximum_normalized_constraint_violation, 50.0);
  EXPECT_NEAR(normalized_outcome.result->primal[0], 0.25, 5e-3);
  EXPECT_NEAR(normalized_outcome.result->primal[1], 500.0, 5e-2);
  EXPECT_LE(
    normalized_outcome.result->maximum_normalized_constraint_violation, 1.0);
}

TEST(PersistentOsqpSolver, RowToleranceNormalizationUsesDimensionlessRows)
{
  const auto quadratic = diagonal_matrix({1.0, 1.0});
  const auto constraints = identity_constraints(2);
  const Eigen::VectorXd cost = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd lower(2);
  lower << -0.25, 0.0;
  Eigen::VectorXd upper(2);
  upper << 0.25, 20.0;

  PersistentOsqpSolver solver(
    ConstraintPreconditioningPolicy::RowToleranceNormalized);
  const auto outcome = solver.solve(
    quadratic, constraints, cost, lower, upper);

  ASSERT_TRUE(outcome.result.has_value()) << outcome.failure_detail;
  // The zero lower bound owns the strict 0.001 physical tolerance. It maps to
  // OSQP's 0.001 absolute tolerance with scale 1, independent of the 20 m
  // opposite bound. The global relative stopping term is disabled for this
  // already-normalized policy.
  EXPECT_NEAR(outcome.telemetry.maximum_row_scale, 1.0, 1e-9);
  EXPECT_DOUBLE_EQ(outcome.telemetry.relative_tolerance, 0.0);
}

TEST(PersistentOsqpSolver, RowToleranceNormalizationUsesStrictAsymmetricSide)
{
  const auto quadratic = diagonal_matrix({1.0, 1.0});
  const auto constraints = identity_constraints(2);
  const Eigen::VectorXd cost = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd lower(2);
  lower << -3.0, 0.0;
  Eigen::VectorXd upper(2);
  upper << 1.37, 12.0;

  PersistentOsqpSolver solver(
    ConstraintPreconditioningPolicy::RowToleranceNormalized);
  const auto outcome = solver.solve(
    quadratic, constraints, cost, lower, upper);

  ASSERT_TRUE(outcome.result.has_value()) << outcome.failure_detail;
  // A violation is certified against the side it crosses. The zero lower
  // bound therefore owns a 0.001 tolerance even though the opposite bound is
  // 12, and maps directly to OSQP's 0.001 absolute tolerance.
  EXPECT_NEAR(outcome.telemetry.maximum_row_scale, 1.0, 1e-9);
  EXPECT_DOUBLE_EQ(outcome.telemetry.relative_tolerance, 0.0);
}

TEST(PersistentOsqpSolver, ExplicitVariableScalingCertifiesCoupledMixedUnitChain)
{
  constexpr int stage_count = 20;
  constexpr int progress_count = stage_count + 1;
  constexpr int curvature_count = stage_count;
  constexpr int acceleration_index = progress_count + curvature_count;
  constexpr int variable_count = acceleration_index + 1;
  constexpr int dynamics_count = progress_count;
  constexpr int box_count = variable_count;
  constexpr int rate_count = stage_count;
  constexpr int constraint_count = dynamics_count + box_count + rate_count;

  std::vector<double> diagonal(static_cast<std::size_t>(variable_count), 1.0);
  Eigen::VectorXd cost = Eigen::VectorXd::Zero(variable_count);
  for (int stage = 0; stage < progress_count; ++stage) {
    const double reference = static_cast<double>(stage);
    cost[stage] = -reference;
  }
  for (int stage = 0; stage < curvature_count; ++stage) {
    const int index = progress_count + stage;
    diagonal[static_cast<std::size_t>(index)] = 100.0;
    cost[index] = -25.0;
  }
  cost[acceleration_index] = -1.38;

  Eigen::SparseMatrix<double> constraints(constraint_count, variable_count);
  std::vector<Eigen::Triplet<double>> entries;
  entries.emplace_back(0, 0, 1.0);
  for (int stage = 1; stage < progress_count; ++stage) {
    entries.emplace_back(stage, stage - 1, -1.0);
    entries.emplace_back(stage, stage, 1.0);
  }
  for (int index = 0; index < variable_count; ++index) {
    entries.emplace_back(dynamics_count + index, index, 1.0);
  }
  const int rate_offset = dynamics_count + box_count;
  entries.emplace_back(rate_offset, progress_count, 1.0);
  for (int stage = 1; stage < stage_count; ++stage) {
    entries.emplace_back(rate_offset + stage, progress_count + stage - 1, -1.0);
    entries.emplace_back(rate_offset + stage, progress_count + stage, 1.0);
  }
  constraints.setFromTriplets(entries.begin(), entries.end());

  Eigen::VectorXd lower = Eigen::VectorXd::Zero(constraint_count);
  Eigen::VectorXd upper = Eigen::VectorXd::Zero(constraint_count);
  for (int stage = 1; stage < progress_count; ++stage) {
    lower[stage] = 1.0;
    upper[stage] = 1.0;
  }
  const int box_offset = dynamics_count;
  for (int stage = 0; stage < progress_count; ++stage) {
    lower[box_offset + stage] = 0.0;
    upper[box_offset + stage] = 20.0;
  }
  for (int stage = 0; stage < curvature_count; ++stage) {
    lower[box_offset + progress_count + stage] = -0.25;
    upper[box_offset + progress_count + stage] = 0.25;
  }
  lower[box_offset + acceleration_index] = -3.0;
  upper[box_offset + acceleration_index] = 1.37;
  for (int stage = 0; stage < rate_count; ++stage) {
    lower[rate_offset + stage] = -0.001;
    upper[rate_offset + stage] = 0.001;
  }

  PersistentOsqpSolver row_only(
    ConstraintPreconditioningPolicy::RowToleranceNormalized);
  const auto variable_scaling = derive_box_variable_coordinate_scaling(
    lower.segment(box_offset, variable_count),
    upper.segment(box_offset, variable_count));
  ASSERT_TRUE(variable_scaling.has_value());
  const auto outcome = row_only.solve(
    diagonal_matrix(diagonal), constraints, cost, lower, upper,
    std::nullopt, variable_scaling);

  ASSERT_TRUE(outcome.result.has_value()) << outcome.failure_detail;
  EXPECT_LE(outcome.result->maximum_normalized_constraint_violation, 1.0);
  EXPECT_NEAR(outcome.result->primal[acceleration_index], 1.37, 5e-3);
  EXPECT_NEAR(outcome.result->primal[progress_count], 0.001, 3e-3);
}

TEST(PersistentOsqpSolver, NormalizedWarmStartKeepsDualInPhysicalCoordinates)
{
  PersistentOsqpSolver solver(
    ConstraintPreconditioningPolicy::RowToleranceNormalized);
  const auto quadratic = diagonal_matrix({1.0});
  const auto constraints = identity_constraints(1);
  Eigen::VectorXd cost(1);
  cost << -2.0;
  Eigen::VectorXd lower(1);
  lower << 0.0;
  Eigen::VectorXd first_upper(1);
  first_upper << 1.0;

  const auto first = solver.solve(
    quadratic, constraints, cost, lower, first_upper);
  ASSERT_TRUE(first.result.has_value()) << first.failure_detail;
  EXPECT_NEAR(first.result->primal[0], 1.0, 5e-3);
  EXPECT_NEAR(first.result->dual[0], 1.0, 5e-3);

  Eigen::VectorXd second_upper(1);
  second_upper << 0.5;
  const WarmStart warm_start{first.result->primal, first.result->dual};
  const auto second = solver.solve(
    quadratic, constraints, cost, lower, second_upper, warm_start);

  ASSERT_TRUE(second.result.has_value()) << second.failure_detail;
  EXPECT_TRUE(second.telemetry.update_performed);
  EXPECT_TRUE(second.telemetry.warm_start_applied);
  EXPECT_NEAR(second.result->primal[0], 0.5, 5e-3);
  EXPECT_NEAR(second.result->dual[0], 1.5, 5e-3);
  EXPECT_LE(
    second.result->maximum_normalized_constraint_violation, 1.0);
}

TEST(PersistentOsqpSolver, VariableScalingPreservesPhysicalPrimalAndDual)
{
  PersistentOsqpSolver solver(
    ConstraintPreconditioningPolicy::RowToleranceNormalized);
  const auto quadratic = diagonal_matrix({1.0});
  const auto constraints = identity_constraints(1);
  Eigen::VectorXd cost(1);
  cost << -2.0;
  Eigen::VectorXd lower(1);
  lower << 0.0;
  Eigen::VectorXd upper(1);
  upper << 1.0;
  Eigen::VectorXd scale(1);
  scale << 3.0;

  const auto first = solver.solve(
    quadratic, constraints, cost, lower, upper, std::nullopt,
    VariableCoordinateScaling{scale});
  ASSERT_TRUE(first.result.has_value()) << first.failure_detail;
  EXPECT_TRUE(first.telemetry.variable_coordinate_scaled);
  EXPECT_DOUBLE_EQ(first.telemetry.minimum_variable_scale, 3.0);
  EXPECT_DOUBLE_EQ(first.telemetry.maximum_variable_scale, 3.0);
  EXPECT_NEAR(first.result->primal[0], 1.0, 5e-3);
  EXPECT_NEAR(first.result->dual[0], 1.0, 5e-3);

  upper << 0.5;
  const auto second = solver.solve(
    quadratic, constraints, cost, lower, upper,
    WarmStart{first.result->primal, first.result->dual},
    VariableCoordinateScaling{scale});
  ASSERT_TRUE(second.result.has_value()) << second.failure_detail;
  EXPECT_TRUE(second.telemetry.warm_start_applied);
  EXPECT_NEAR(second.result->primal[0], 0.5, 5e-3);
  EXPECT_NEAR(second.result->dual[0], 1.5, 5e-3);
}

TEST(PersistentOsqpSolver, RebuildsWhenSparsityDimensionsChange) {
  PersistentOsqpSolver solver;
  Eigen::VectorXd first_cost(1);
  Eigen::VectorXd first_lower(1);
  Eigen::VectorXd first_upper(1);
  first_cost << -1.0;
  first_lower << 0.0;
  first_upper << 2.0;
  ASSERT_TRUE(
    solver
    .solve(
      diagonal_matrix({1.0}), identity_constraints(1),
      first_cost, first_lower, first_upper)
    .result.has_value());

  Eigen::VectorXd second_cost(2);
  Eigen::VectorXd second_lower(2);
  Eigen::VectorXd second_upper(2);
  second_cost << -1.0, -2.0;
  second_lower << 0.0, 0.0;
  second_upper << 3.0, 3.0;
  const auto rebuilt =
    solver.solve(
    diagonal_matrix({1.0, 1.0}), identity_constraints(2),
    second_cost, second_lower, second_upper);

  ASSERT_TRUE(rebuilt.result.has_value()) << rebuilt.failure_detail;
  EXPECT_TRUE(rebuilt.telemetry.setup_performed);
  EXPECT_TRUE(rebuilt.telemetry.structural_rebuild);
  EXPECT_FALSE(rebuilt.telemetry.update_performed);
}

TEST(PersistentOsqpSolver, ExplicitZeroCanBeUpdatedWithoutStructuralRebuild) {
  PersistentOsqpSolver solver;
  Eigen::VectorXd cost(1);
  Eigen::VectorXd lower(1);
  Eigen::VectorXd upper(1);
  cost << -2.0;
  lower << -std::numeric_limits<double>::infinity();
  upper << std::numeric_limits<double>::infinity();
  const auto zero_constraint = scalar_constraint(0.0);
  ASSERT_EQ(zero_constraint.nonZeros(), 1);

  const auto first = solver.solve(
    diagonal_matrix({1.0}), zero_constraint, cost, lower, upper);
  ASSERT_TRUE(first.result.has_value()) << first.failure_detail;
  EXPECT_NEAR(first.result->primal[0], 2.0, 1e-2);

  lower << -1.0;
  upper << 1.0;
  const auto updated = solver.solve(
    diagonal_matrix({1.0}), scalar_constraint(1.0), cost, lower, upper);
  ASSERT_TRUE(updated.result.has_value()) << updated.failure_detail;
  EXPECT_TRUE(updated.telemetry.update_performed);
  EXPECT_FALSE(updated.telemetry.setup_performed);
  EXPECT_FALSE(updated.telemetry.structural_rebuild);
  EXPECT_NEAR(updated.result->primal[0], 1.0, 1e-2);
}

TEST(PersistentOsqpSolver, FailedSolveForcesNextCycleColdSetup) {
  PersistentOsqpSolver solver;
  Eigen::SparseMatrix<double> infeasible_constraints(2, 1);
  infeasible_constraints.insert(0, 0) = 1.0;
  infeasible_constraints.insert(1, 0) = 1.0;
  infeasible_constraints.makeCompressed();
  Eigen::VectorXd cost(1);
  Eigen::VectorXd infeasible_lower(2);
  Eigen::VectorXd infeasible_upper(2);
  cost << 0.0;
  infeasible_lower << 1.0, -std::numeric_limits<double>::infinity();
  infeasible_upper << std::numeric_limits<double>::infinity(), 0.0;

  const auto failed =
    solver.solve(
    diagonal_matrix({1.0}), infeasible_constraints, cost,
    infeasible_lower, infeasible_upper);

  EXPECT_FALSE(failed.result.has_value());
  EXPECT_TRUE(failed.telemetry.cold_reset_after_failure);
  EXPECT_FALSE(solver.initialized());

  Eigen::VectorXd feasible_lower(1);
  Eigen::VectorXd feasible_upper(1);
  feasible_lower << -1.0;
  feasible_upper << 1.0;
  const auto recovered =
    solver.solve(
    diagonal_matrix({1.0}), identity_constraints(1), cost,
    feasible_lower, feasible_upper);

  ASSERT_TRUE(recovered.result.has_value()) << recovered.failure_detail;
  EXPECT_TRUE(recovered.telemetry.setup_performed);
  EXPECT_FALSE(recovered.telemetry.update_performed);
}

} // namespace
} // namespace multi_purpose_mpc_ros::persistent_osqp
