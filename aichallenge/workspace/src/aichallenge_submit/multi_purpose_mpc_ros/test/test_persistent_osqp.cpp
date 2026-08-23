#include <multi_purpose_mpc_ros/persistent_osqp.hpp>

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>

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
