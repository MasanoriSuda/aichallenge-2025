#include "multi_purpose_mpc_ros/mpcc_rate_resolved_dynamic_obstacle.hpp"

#include <gtest/gtest.h>

#include <array>
#include <limits>

namespace dynamic_obstacle =
  multi_purpose_mpc_ros::mpcc_rate_resolved_dynamic_obstacle;
namespace model = multi_purpose_mpc_ros::mpcc_rate_resolved;
namespace problem = multi_purpose_mpc_ros::mpcc_rate_resolved_problem;

namespace
{

dynamic_obstacle::Request request_with_lateral_suffix()
{
  dynamic_obstacle::Request request;
  request.active = true;
  request.pass_side_sign = 1;
  request.wall_only_problem.horizon_steps = 4;
  const int states = model::kStateDimension * 5;
  const int inputs = model::kInputDimension * 4;
  request.wall_only_primal = Eigen::VectorXd::Zero(states + inputs);
  for (int stage = 0; stage <= 4; ++stage) {
    const int state = stage * model::kStateDimension;
    request.wall_only_primal[state + model::kProgressIndex] = 0.5 * stage;
    request.wall_only_primal[state + model::kLateralIndex] =
      stage < 3 ? 0.1 * stage : 1.0;
  }
  request.stages.assign(4, dynamic_obstacle::StagePrediction{
    true, 2.0, 0.0, 0.8, 0.75});
  return request;
}

}  // namespace

TEST(MpccRateResolvedDynamicObstacle, HoldsProgressUntilLateralSuffixIsReachable)
{
  const auto result = dynamic_obstacle::refine(
    request_with_lateral_suffix());
  ASSERT_TRUE(result.applied);
  ASSERT_TRUE(result.problem.has_value());
  EXPECT_EQ(result.first_pass_side_stage, 2);
  EXPECT_EQ(result.stay_behind_row_count, 2U);
  EXPECT_EQ(result.pass_side_row_count, 2U);
  ASSERT_EQ(result.problem->dynamic_obstacle_constraints.size(), 4U);
  EXPECT_EQ(
    result.problem->dynamic_obstacle_constraints[0].axis,
    problem::DynamicObstacleConstraintAxis::EffectiveProgress);
  EXPECT_DOUBLE_EQ(
    result.problem->dynamic_obstacle_constraints[0].upper, 1.2);
  EXPECT_EQ(
    result.problem->dynamic_obstacle_constraints[2].axis,
    problem::DynamicObstacleConstraintAxis::Lateral);
  EXPECT_DOUBLE_EQ(
    result.problem->dynamic_obstacle_constraints[2].lower, 0.75);
}

TEST(MpccRateResolvedDynamicObstacle, ReturnCanSealStayBehindWithoutPassSide)
{
  auto request = request_with_lateral_suffix();
  request.pass_side_sign = 0;
  request.longitudinal_topology =
    dynamic_obstacle::LongitudinalTopology::StayBehind;

  const auto result = dynamic_obstacle::refine(request);

  ASSERT_TRUE(result.applied);
  ASSERT_TRUE(result.problem.has_value());
  EXPECT_EQ(result.resolved_side_sign, 0);
  EXPECT_EQ(result.stay_behind_row_count, 4U);
  EXPECT_EQ(result.ahead_row_count, 0U);
  for (const auto & constraint : result.problem->dynamic_obstacle_constraints) {
    EXPECT_EQ(
      constraint.axis,
      problem::DynamicObstacleConstraintAxis::EffectiveProgress);
    EXPECT_DOUBLE_EQ(constraint.upper, 1.2);
  }
}

TEST(MpccRateResolvedDynamicObstacle, ReturnCanSealStayAheadWithoutPassSide)
{
  auto request = request_with_lateral_suffix();
  request.pass_side_sign = 0;
  request.longitudinal_topology =
    dynamic_obstacle::LongitudinalTopology::StayAhead;

  const auto result = dynamic_obstacle::refine(request);

  ASSERT_TRUE(result.applied);
  ASSERT_TRUE(result.problem.has_value());
  EXPECT_EQ(result.resolved_side_sign, 0);
  EXPECT_EQ(result.stay_behind_row_count, 0U);
  EXPECT_EQ(result.ahead_row_count, 4U);
  for (const auto & constraint : result.problem->dynamic_obstacle_constraints) {
    EXPECT_EQ(
      constraint.axis,
      problem::DynamicObstacleConstraintAxis::EffectiveProgress);
    EXPECT_DOUBLE_EQ(constraint.lower, 2.8);
  }
}

TEST(MpccRateResolvedDynamicObstacle, AppliesWitnessBranchToCompatibleBroadProblem)
{
  auto request = request_with_lateral_suffix();
  const int state_count = model::kStateDimension * 5;
  request.wall_only_problem.state_lower = Eigen::VectorXd::Constant(
    state_count, -2.0);
  request.wall_only_problem.state_upper = Eigen::VectorXd::Constant(
    state_count, 2.0);
  for (int stage = 1; stage <= 4; ++stage) {
    const int state = stage * model::kStateDimension;
    const double witness_progress = 0.5 * static_cast<double>(stage);
    request.wall_only_problem.state_lower[state + model::kProgressIndex] =
      witness_progress - 0.025;
    request.wall_only_problem.state_upper[state + model::kProgressIndex] =
      witness_progress + 0.025;
  }
  auto broad = request.wall_only_problem;
  for (int stage = 1; stage <= 4; ++stage) {
    const int state = stage * model::kStateDimension;
    broad.state_lower[state + model::kProgressIndex] = 0.0;
    broad.state_upper[state + model::kProgressIndex] = 4.0;
  }
  request.constraint_target_problem = broad;

  const auto result = dynamic_obstacle::refine(request);

  ASSERT_TRUE(result.applied);
  ASSERT_TRUE(result.problem.has_value());
  EXPECT_EQ(result.first_pass_side_stage, 2);
  ASSERT_EQ(result.problem->dynamic_obstacle_constraints.size(), 4U);
  for (int stage = 1; stage <= 4; ++stage) {
    const int state = stage * model::kStateDimension;
    EXPECT_DOUBLE_EQ(
      result.problem->state_lower[state + model::kProgressIndex], 0.0);
    EXPECT_DOUBLE_EQ(
      result.problem->state_upper[state + model::kProgressIndex], 4.0);
  }
  EXPECT_NE(
    result.problem->state_lower[
      model::kStateDimension + model::kProgressIndex],
    request.wall_only_problem.state_lower[
      model::kStateDimension + model::kProgressIndex]);
}

TEST(MpccRateResolvedDynamicObstacle, RejectsIncompatibleConstraintTarget)
{
  auto request = request_with_lateral_suffix();
  request.constraint_target_problem = request.wall_only_problem;
  request.constraint_target_problem->horizon_steps = 3;

  const auto result = dynamic_obstacle::refine(request);

  EXPECT_FALSE(result.applied);
  EXPECT_EQ(result.reason, dynamic_obstacle::Reason::InvalidInput);
  EXPECT_FALSE(result.problem.has_value());
}

TEST(MpccRateResolvedDynamicObstacle, FollowClassifiesStayBehindByEffectiveProgress)
{
  auto request = request_with_lateral_suffix();
  request.pass_side_sign = 0;
  const int state_count = model::kStateDimension * 5;
  request.wall_only_problem.state_lower = Eigen::VectorXd::Constant(
    state_count, -std::numeric_limits<double>::infinity());
  request.wall_only_problem.state_upper = Eigen::VectorXd::Constant(
    state_count, std::numeric_limits<double>::infinity());
  for (int stage = 0; stage <= 4; ++stage) {
    const int state = stage * model::kStateDimension;
    const double raw_progress = 0.5 * static_cast<double>(stage);
    request.wall_only_primal[state + model::kProgressIndex] = raw_progress;
    request.wall_only_primal[state + model::kLagIndex] = -1.0;
    request.wall_only_primal[state + model::kLateralIndex] = 0.70;
    request.wall_only_problem.state_lower[state + model::kProgressIndex] =
      raw_progress;
    request.wall_only_problem.state_upper[state + model::kProgressIndex] =
      4.0;
    request.wall_only_problem.state_lower[state + model::kLagIndex] = -1.0;
    request.wall_only_problem.state_upper[state + model::kLagIndex] = 1.0;
    request.wall_only_problem.state_lower[state + model::kLateralIndex] = -2.0;
    request.wall_only_problem.state_upper[state + model::kLateralIndex] = 2.0;
  }
  request.stages.assign(4, dynamic_obstacle::StagePrediction{
    true, 2.0, 0.0, 0.8, 0.75});

  const auto result = dynamic_obstacle::refine(request);

  ASSERT_TRUE(result.applied);
  ASSERT_TRUE(result.problem.has_value());
  EXPECT_DOUBLE_EQ(result.first_wall_only_progress_m, 0.5);
  EXPECT_DOUBLE_EQ(result.first_wall_only_effective_progress_m, -0.5);
  EXPECT_GT(result.first_stay_behind_margin_m, 0.0);
  EXPECT_EQ(result.resolved_side_sign, 0);
  EXPECT_EQ(result.first_pass_side_stage, -1);
  EXPECT_EQ(result.stay_behind_row_count, 4U);
  EXPECT_EQ(result.pass_side_row_count, 0U);
  for (const auto & constraint : result.problem->dynamic_obstacle_constraints) {
    EXPECT_EQ(
      constraint.axis,
      problem::DynamicObstacleConstraintAxis::EffectiveProgress);
  }
}

TEST(MpccRateResolvedDynamicObstacle, DoesNotTrustOneSeparatedMiddleSample)
{
  auto request = request_with_lateral_suffix();
  const int terminal = 4 * model::kStateDimension;
  request.wall_only_primal[terminal + model::kLateralIndex] = 0.2;

  const auto result = dynamic_obstacle::refine(request);
  ASSERT_TRUE(result.applied);
  EXPECT_EQ(result.first_pass_side_stage, -1);
  EXPECT_EQ(result.stay_behind_row_count, 4U);
  EXPECT_EQ(result.pass_side_row_count, 0U);
}

TEST(MpccRateResolvedDynamicObstacle, CruiseWithoutPassSideUsesCoherentWallOnlySide)
{
  auto request = request_with_lateral_suffix();
  request.pass_side_sign = 0;
  for (int stage = 0; stage <= 4; ++stage) {
    const int state = stage * model::kStateDimension;
    request.wall_only_primal[state + model::kLateralIndex] = 1.0;
  }

  const auto result = dynamic_obstacle::refine(request);

  ASSERT_TRUE(result.applied);
  ASSERT_TRUE(result.problem.has_value());
  EXPECT_EQ(result.resolved_side_sign, 1);
  EXPECT_EQ(result.first_pass_side_stage, 0);
  EXPECT_EQ(result.stay_behind_row_count, 0U);
  EXPECT_EQ(result.pass_side_row_count, 4U);
  ASSERT_EQ(result.problem->dynamic_obstacle_constraints.size(), 4U);
  for (const auto & constraint : result.problem->dynamic_obstacle_constraints) {
    EXPECT_EQ(
      constraint.axis,
      problem::DynamicObstacleConstraintAxis::Lateral);
    EXPECT_DOUBLE_EQ(constraint.lower, 0.75);
  }
}

TEST(MpccRateResolvedDynamicObstacle, CruiseKeepsStayBehindWhenWallOnlyPathIsBehind)
{
  auto request = request_with_lateral_suffix();
  request.pass_side_sign = 0;
  for (int stage = 0; stage <= 4; ++stage) {
    const int state = stage * model::kStateDimension;
    request.wall_only_primal[state + model::kProgressIndex] = 0.1 * stage;
    request.wall_only_primal[state + model::kLateralIndex] = 0.0;
  }

  const auto result = dynamic_obstacle::refine(request);

  ASSERT_TRUE(result.applied);
  ASSERT_TRUE(result.problem.has_value());
  EXPECT_EQ(result.resolved_side_sign, 0);
  EXPECT_EQ(result.first_pass_side_stage, -1);
  EXPECT_EQ(result.stay_behind_row_count, 4U);
  EXPECT_EQ(result.pass_side_row_count, 0U);
}

TEST(MpccRateResolvedDynamicObstacle, CruisePreservesCurrentSideWhenRacingLineCrossesLater)
{
  auto request = request_with_lateral_suffix();
  request.pass_side_sign = 0;
  const std::array<double, 5> lateral_m{{1.0, 1.0, 0.9, 0.4, -0.2}};
  for (int stage = 0; stage <= 4; ++stage) {
    const int state = stage * model::kStateDimension;
    request.wall_only_primal[state + model::kLateralIndex] =
      lateral_m[static_cast<std::size_t>(stage)];
  }

  const auto result = dynamic_obstacle::refine(request);

  ASSERT_TRUE(result.applied);
  ASSERT_TRUE(result.problem.has_value());
  EXPECT_EQ(result.resolved_side_sign, 1);
  EXPECT_EQ(result.first_pass_side_stage, 0);
  EXPECT_EQ(result.stay_behind_row_count, 0U);
  EXPECT_EQ(result.pass_side_row_count, 4U);
  for (const auto & constraint : result.problem->dynamic_obstacle_constraints) {
    EXPECT_EQ(
      constraint.axis,
      problem::DynamicObstacleConstraintAxis::Lateral);
    EXPECT_DOUBLE_EQ(constraint.lower, 0.75);
  }
}

TEST(MpccRateResolvedDynamicObstacle, CruiseDoesNotWeakenUnreachableDisjunctWithoutGeometry)
{
  auto request = request_with_lateral_suffix();
  request.pass_side_sign = 0;
  const int state_count = model::kStateDimension * 5;
  request.wall_only_problem.state_lower = Eigen::VectorXd::Constant(
    state_count, -std::numeric_limits<double>::infinity());
  request.wall_only_problem.state_upper = Eigen::VectorXd::Constant(
    state_count, std::numeric_limits<double>::infinity());
  for (int stage = 0; stage <= 4; ++stage) {
    const int state = stage * model::kStateDimension;
    request.wall_only_primal[state + model::kProgressIndex] =
      0.5 * static_cast<double>(stage);
    request.wall_only_primal[state + model::kLateralIndex] = 0.70;
    request.wall_only_problem.state_lower[state + model::kProgressIndex] = 0.0;
    request.wall_only_problem.state_upper[state + model::kProgressIndex] = 4.0;
    request.wall_only_problem.state_lower[state + model::kLagIndex] = 0.0;
    request.wall_only_problem.state_upper[state + model::kLagIndex] = 0.0;
    request.wall_only_problem.state_lower[state + model::kLateralIndex] = -2.0;
    request.wall_only_problem.state_upper[state + model::kLateralIndex] = 2.0;
  }
  request.stages.assign(4, dynamic_obstacle::StagePrediction{
    true, 0.0, 0.0, 0.8, 0.75});

  const auto result = dynamic_obstacle::refine(request);

  ASSERT_TRUE(result.applied);
  ASSERT_TRUE(result.problem.has_value());
  EXPECT_LT(result.first_stay_behind_margin_m, 0.0);
  EXPECT_LT(result.first_positive_side_margin_m, 0.0);
  EXPECT_EQ(result.resolved_side_sign, 1);
  EXPECT_EQ(result.first_pass_side_stage, -1);
  EXPECT_EQ(result.stay_behind_row_count, 4U);
  EXPECT_EQ(result.pass_side_row_count, 0U);
  for (const auto & constraint : result.problem->dynamic_obstacle_constraints) {
    EXPECT_EQ(
      constraint.axis,
      problem::DynamicObstacleConstraintAxis::EffectiveProgress);
    EXPECT_DOUBLE_EQ(constraint.upper, -0.8);
  }
}

TEST(MpccRateResolvedDynamicObstacle, TacticalSideUsesOnlyCompleteDisjuncts)
{
  auto request = request_with_lateral_suffix();
  request.pass_side_sign = 1;
  const int state_count = model::kStateDimension * 5;
  request.wall_only_problem.state_lower = Eigen::VectorXd::Constant(
    state_count, -std::numeric_limits<double>::infinity());
  request.wall_only_problem.state_upper = Eigen::VectorXd::Constant(
    state_count, std::numeric_limits<double>::infinity());
  for (int stage = 0; stage <= 4; ++stage) {
    const int state = stage * model::kStateDimension;
    request.wall_only_primal[state + model::kProgressIndex] =
      0.5 * static_cast<double>(stage);
    request.wall_only_primal[state + model::kLateralIndex] =
      0.70 + 0.02 * static_cast<double>(stage);
    request.wall_only_problem.state_lower[state + model::kProgressIndex] = 0.0;
    request.wall_only_problem.state_upper[state + model::kProgressIndex] = 4.0;
    request.wall_only_problem.state_lower[state + model::kLagIndex] = 0.0;
    request.wall_only_problem.state_upper[state + model::kLagIndex] = 0.0;
    request.wall_only_problem.state_lower[state + model::kLateralIndex] = -2.0;
    request.wall_only_problem.state_upper[state + model::kLateralIndex] = 2.0;
  }
  request.stages.assign(4, dynamic_obstacle::StagePrediction{
    true, 0.0, 0.0, 0.8, 0.75});

  const auto result = dynamic_obstacle::refine(request);

  ASSERT_TRUE(result.applied);
  ASSERT_TRUE(result.problem.has_value());
  EXPECT_EQ(result.resolved_side_sign, 1);
  EXPECT_EQ(result.first_pass_side_stage, 2);
  EXPECT_EQ(result.stay_behind_row_count, 2U);
  EXPECT_EQ(result.pass_side_row_count, 2U);
  ASSERT_EQ(result.problem->dynamic_obstacle_constraints.size(), 4U);
  EXPECT_EQ(
    result.problem->dynamic_obstacle_constraints[0].axis,
    problem::DynamicObstacleConstraintAxis::EffectiveProgress);
  EXPECT_DOUBLE_EQ(
    result.problem->dynamic_obstacle_constraints[0].upper, -0.8);
  EXPECT_EQ(
    result.problem->dynamic_obstacle_constraints[2].axis,
    problem::DynamicObstacleConstraintAxis::Lateral);
  EXPECT_DOUBLE_EQ(
    result.problem->dynamic_obstacle_constraints[2].lower, 0.75);
}

TEST(MpccRateResolvedDynamicObstacle, UnreachedSideNeverBorrowsWallOnlyWitness)
{
  auto request = request_with_lateral_suffix();
  request.pass_side_sign = 1;
  const int state_count = model::kStateDimension * 5;
  request.wall_only_problem.state_lower = Eigen::VectorXd::Constant(
    state_count, -std::numeric_limits<double>::infinity());
  request.wall_only_problem.state_upper = Eigen::VectorXd::Constant(
    state_count, std::numeric_limits<double>::infinity());
  const std::array<double, 5> lateral_m{{0.70, 0.66, 0.68, 0.72, 0.76}};
  for (int stage = 0; stage <= 4; ++stage) {
    const int state = stage * model::kStateDimension;
    request.wall_only_primal[state + model::kProgressIndex] =
      0.5 * static_cast<double>(stage);
    request.wall_only_primal[state + model::kLateralIndex] =
      lateral_m[static_cast<std::size_t>(stage)];
    request.wall_only_problem.state_lower[state + model::kProgressIndex] = 0.0;
    request.wall_only_problem.state_upper[state + model::kProgressIndex] = 4.0;
    request.wall_only_problem.state_lower[state + model::kLagIndex] = 0.0;
    request.wall_only_problem.state_upper[state + model::kLagIndex] = 0.0;
    request.wall_only_problem.state_lower[state + model::kLateralIndex] = -2.0;
    request.wall_only_problem.state_upper[state + model::kLateralIndex] = 2.0;
  }
  request.stages.assign(4, dynamic_obstacle::StagePrediction{
    true, 0.0, 0.0, 0.8, 0.75});

  const auto result = dynamic_obstacle::refine(request);

  ASSERT_TRUE(result.applied);
  ASSERT_TRUE(result.problem.has_value());
  ASSERT_EQ(result.problem->dynamic_obstacle_constraints.size(), 4U);
  EXPECT_EQ(result.first_pass_side_stage, 3);
  EXPECT_EQ(result.stay_behind_row_count, 3U);
  EXPECT_EQ(result.pass_side_row_count, 1U);
  for (std::size_t index = 0U; index < 3U; ++index) {
    const auto & constraint =
      result.problem->dynamic_obstacle_constraints[index];
    EXPECT_EQ(
      constraint.axis,
      problem::DynamicObstacleConstraintAxis::EffectiveProgress);
    EXPECT_DOUBLE_EQ(constraint.upper, -0.8);
  }
  EXPECT_EQ(
    result.problem->dynamic_obstacle_constraints[3].axis,
    problem::DynamicObstacleConstraintAxis::Lateral);
  EXPECT_DOUBLE_EQ(
    result.problem->dynamic_obstacle_constraints[3].lower, 0.75);
}

TEST(MpccRateResolvedDynamicObstacle, ForcedTransitionUsesOnlyCompleteDisjuncts)
{
  auto request = request_with_lateral_suffix();
  request.pass_side_sign = 1;
  request.forced_first_pass_side_stage = 2;
  request.forced_first_ahead_stage = 3;
  const int state_count = model::kStateDimension * 5;
  request.wall_only_problem.state_lower = Eigen::VectorXd::Constant(
    state_count, -std::numeric_limits<double>::infinity());
  request.wall_only_problem.state_upper = Eigen::VectorXd::Constant(
    state_count, std::numeric_limits<double>::infinity());
  for (int stage = 0; stage <= 4; ++stage) {
    const int state = stage * model::kStateDimension;
    request.wall_only_primal[state + model::kProgressIndex] =
      0.5 * static_cast<double>(stage);
    request.wall_only_primal[state + model::kLateralIndex] =
      0.66 + 0.02 * static_cast<double>(stage);
    request.wall_only_problem.state_lower[state + model::kProgressIndex] = 0.0;
    request.wall_only_problem.state_upper[state + model::kProgressIndex] = 4.0;
    request.wall_only_problem.state_lower[state + model::kLagIndex] = 0.0;
    request.wall_only_problem.state_upper[state + model::kLagIndex] = 0.0;
    request.wall_only_problem.state_lower[state + model::kLateralIndex] = -2.0;
    request.wall_only_problem.state_upper[state + model::kLateralIndex] = 2.0;
  }
  request.stages.assign(4, dynamic_obstacle::StagePrediction{
    true, 0.0, 0.0, 0.8, 0.75});

  const auto result = dynamic_obstacle::refine(request);

  ASSERT_TRUE(result.applied);
  ASSERT_TRUE(result.problem.has_value());
  EXPECT_TRUE(result.forced_transition_applied);
  EXPECT_EQ(result.first_pass_side_stage, 2);
  EXPECT_EQ(result.stay_behind_row_count, 2U);
  EXPECT_EQ(result.pass_side_row_count, 1U);
  EXPECT_EQ(result.ahead_row_count, 1U);
  ASSERT_EQ(result.problem->dynamic_obstacle_constraints.size(), 4U);
  EXPECT_EQ(
    result.problem->dynamic_obstacle_constraints[0].axis,
    problem::DynamicObstacleConstraintAxis::EffectiveProgress);
  EXPECT_EQ(
    result.problem->dynamic_obstacle_constraints[1].axis,
    problem::DynamicObstacleConstraintAxis::EffectiveProgress);
  EXPECT_EQ(
    result.problem->dynamic_obstacle_constraints[2].axis,
    problem::DynamicObstacleConstraintAxis::Lateral);
  EXPECT_DOUBLE_EQ(
    result.problem->dynamic_obstacle_constraints[2].lower, 0.75);
  EXPECT_EQ(
    result.problem->dynamic_obstacle_constraints[3].axis,
    problem::DynamicObstacleConstraintAxis::EffectiveProgress);
  EXPECT_DOUBLE_EQ(
    result.problem->dynamic_obstacle_constraints[3].lower, 0.8);
}

TEST(MpccRateResolvedDynamicObstacle, DiagonalScheduleConnectsExactEndpoints)
{
  auto request = request_with_lateral_suffix();
  request.pass_side_sign = 1;
  request.forced_diagonal_start_stage = 1;
  request.forced_diagonal_full_side_stage = 3;

  const auto result = dynamic_obstacle::refine(request);

  ASSERT_TRUE(result.applied);
  ASSERT_TRUE(result.problem.has_value());
  EXPECT_TRUE(result.forced_transition_applied);
  EXPECT_EQ(result.first_pass_side_stage, 3);
  EXPECT_EQ(result.stay_behind_row_count, 1U);
  EXPECT_EQ(result.diagonal_row_count, 2U);
  EXPECT_EQ(result.pass_side_row_count, 1U);
  ASSERT_EQ(result.problem->dynamic_obstacle_constraints.size(), 4U);
  const auto & behind = result.problem->dynamic_obstacle_constraints[0];
  EXPECT_EQ(
    behind.axis,
    problem::DynamicObstacleConstraintAxis::EffectiveProgress);
  EXPECT_DOUBLE_EQ(behind.upper, 1.2);

  const auto & diagonal_start =
    result.problem->dynamic_obstacle_constraints[1];
  EXPECT_EQ(
    diagonal_start.axis,
    problem::DynamicObstacleConstraintAxis::CoupledLateralProgress);
  EXPECT_NEAR(diagonal_start.lateral_coefficient, 0.0, 1e-12);
  EXPECT_NEAR(diagonal_start.effective_progress_coefficient, -1.25, 1e-12);
  EXPECT_NEAR(diagonal_start.lower, -1.5, 1e-12);

  const auto & diagonal_middle =
    result.problem->dynamic_obstacle_constraints[2];
  EXPECT_EQ(
    diagonal_middle.axis,
    problem::DynamicObstacleConstraintAxis::CoupledLateralProgress);
  EXPECT_GT(diagonal_middle.lateral_coefficient, 0.0);
  EXPECT_LT(diagonal_middle.effective_progress_coefficient, 0.0);

  const auto & full_side =
    result.problem->dynamic_obstacle_constraints[3];
  EXPECT_EQ(
    full_side.axis, problem::DynamicObstacleConstraintAxis::Lateral);
  EXPECT_DOUBLE_EQ(full_side.lower, 0.75);
}

TEST(MpccRateResolvedDynamicObstacle, PhysicalDiagonalUsesBodySupportFunction)
{
  auto request = request_with_lateral_suffix();
  request.pass_side_sign = 1;
  request.forced_diagonal_start_stage = 1;
  request.forced_diagonal_full_side_stage = 3;
  request.forced_physical_separation_geometry =
    dynamic_obstacle::PhysicalSeparationGeometry{
    1.4, 0.5, 0.7, 0.6, 0.1, 0.8};

  const auto result = dynamic_obstacle::refine(request);

  ASSERT_TRUE(result.applied);
  ASSERT_TRUE(result.problem.has_value());
  EXPECT_TRUE(result.physical_diagonal_guidance_applied);
  ASSERT_EQ(result.problem->dynamic_obstacle_constraints.size(), 4U);
  const auto & behind = result.problem->dynamic_obstacle_constraints[0];
  EXPECT_EQ(
    behind.axis,
    problem::DynamicObstacleConstraintAxis::EffectiveProgress);
  EXPECT_NEAR(behind.upper, -0.3, 1e-12);

  const auto & diagonal_start =
    result.problem->dynamic_obstacle_constraints[1];
  EXPECT_EQ(
    diagonal_start.axis,
    problem::DynamicObstacleConstraintAxis::CoupledLateralProgress);
  EXPECT_NEAR(diagonal_start.effective_progress_coefficient, -1.0, 1e-12);
  EXPECT_NEAR(diagonal_start.lateral_coefficient, 0.0, 1e-12);
  EXPECT_NEAR(diagonal_start.lower, 0.3, 1e-12);

  const auto & diagonal_middle =
    result.problem->dynamic_obstacle_constraints[2];
  constexpr double kSqrtHalf = 0.70710678118654752440;
  EXPECT_NEAR(
    diagonal_middle.effective_progress_coefficient, -kSqrtHalf, 1e-12);
  EXPECT_NEAR(diagonal_middle.lateral_coefficient, kSqrtHalf, 1e-12);
  EXPECT_NEAR(
    diagonal_middle.lower,
    (1.5 + 0.7) * kSqrtHalf + 0.8 - 2.0 * kSqrtHalf,
    1e-12);

  const auto & full_side =
    result.problem->dynamic_obstacle_constraints[3];
  EXPECT_EQ(
    full_side.axis, problem::DynamicObstacleConstraintAxis::Lateral);
  EXPECT_NEAR(full_side.lower, 1.5, 1e-12);
}

TEST(MpccRateResolvedDynamicObstacle, OrdinaryRowsUsePhysicalBodySupport)
{
  auto request = request_with_lateral_suffix();
  for (int stage = 3; stage <= 4; ++stage) {
    const int state = stage * model::kStateDimension;
    request.wall_only_primal[state + model::kLateralIndex] = 3.0;
  }
  request.physical_separation_geometry =
    dynamic_obstacle::PhysicalSeparationGeometry{
    1.4, 0.5, 0.7, 0.6, 0.1, 0.8};

  const auto result = dynamic_obstacle::refine(request);

  ASSERT_TRUE(result.applied);
  ASSERT_TRUE(result.problem.has_value());
  EXPECT_TRUE(result.physical_axis_support_applied);
  EXPECT_FALSE(result.physical_diagonal_guidance_applied);
  EXPECT_EQ(result.first_pass_side_stage, 2);
  ASSERT_EQ(result.problem->dynamic_obstacle_constraints.size(), 4U);
  const auto & behind = result.problem->dynamic_obstacle_constraints[0];
  EXPECT_EQ(
    behind.axis,
    problem::DynamicObstacleConstraintAxis::EffectiveProgress);
  // target progress 2.0 - (front 1.4 + margin 0.1 + circle 0.8)
  EXPECT_NEAR(behind.upper, -0.3, 1e-12);
  const auto & side = result.problem->dynamic_obstacle_constraints[2];
  EXPECT_EQ(side.axis, problem::DynamicObstacleConstraintAxis::Lateral);
  // Ego passes on the positive-lateral side, so the target-facing support is
  // the ego right extent: 0.6 + margin 0.1 + circle 0.8.
  EXPECT_NEAR(side.lower, 1.5, 1e-12);
}

TEST(MpccRateResolvedDynamicObstacle, RejectsPhysicalGeometryWithoutDiagonal)
{
  auto request = request_with_lateral_suffix();
  request.forced_physical_separation_geometry =
    dynamic_obstacle::PhysicalSeparationGeometry{
    1.4, 0.5, 0.7, 0.6, 0.1, 0.8};

  const auto result = dynamic_obstacle::refine(request);

  EXPECT_EQ(result.reason, dynamic_obstacle::Reason::InvalidInput);
  EXPECT_FALSE(result.problem.has_value());
}

TEST(MpccRateResolvedDynamicObstacle, InitialOverlapUsesDerivedPhysicalDiagonal)
{
  auto request = request_with_lateral_suffix();
  request.pass_side_sign = 1;
  const int state_count = model::kStateDimension * 5;
  request.wall_only_problem.state_lower = Eigen::VectorXd::Constant(
    state_count, -std::numeric_limits<double>::infinity());
  request.wall_only_problem.state_upper = Eigen::VectorXd::Constant(
    state_count, std::numeric_limits<double>::infinity());
  for (int stage = 1; stage <= 4; ++stage) {
    const int state = stage * model::kStateDimension;
    request.wall_only_problem.state_lower[state + model::kProgressIndex] = 2.0;
    request.wall_only_problem.state_upper[state + model::kProgressIndex] = 4.0;
    request.wall_only_problem.state_lower[state + model::kLagIndex] = 0.0;
    request.wall_only_problem.state_upper[state + model::kLagIndex] = 0.0;
    request.wall_only_problem.state_lower[state + model::kLateralIndex] = -2.0;
    request.wall_only_problem.state_upper[state + model::kLateralIndex] = 2.0;
  }
  request.wall_only_primal[model::kLateralIndex] = 0.1;
  request.physical_separation_geometry =
    dynamic_obstacle::PhysicalSeparationGeometry{
    1.4, 0.5, 0.7, 0.6, 0.1, 0.8};

  const auto result = dynamic_obstacle::refine(request);

  ASSERT_TRUE(result.applied);
  ASSERT_TRUE(result.problem.has_value());
  EXPECT_TRUE(result.physical_diagonal_guidance_applied);
  EXPECT_FALSE(result.forced_transition_applied);
  EXPECT_EQ(result.first_pass_side_stage, 3);
  EXPECT_EQ(result.stay_behind_row_count, 1U);
  EXPECT_EQ(result.diagonal_row_count, 2U);
  EXPECT_EQ(result.pass_side_row_count, 1U);
  ASSERT_EQ(result.problem->dynamic_obstacle_constraints.size(), 4U);
  EXPECT_EQ(
    result.problem->dynamic_obstacle_constraints[0].axis,
    problem::DynamicObstacleConstraintAxis::EffectiveProgress);
  EXPECT_EQ(
    result.problem->dynamic_obstacle_constraints[1].axis,
    problem::DynamicObstacleConstraintAxis::CoupledLateralProgress);
  EXPECT_EQ(
    result.problem->dynamic_obstacle_constraints[3].axis,
    problem::DynamicObstacleConstraintAxis::Lateral);
}

TEST(MpccRateResolvedDynamicObstacle, OfflineContinuationEndsAtExactDisjunction)
{
  auto make_request = []() {
      auto request = request_with_lateral_suffix();
      request.pass_side_sign = 1;
      request.forced_first_pass_side_stage = 2;
      request.forced_first_ahead_stage = 3;
      const int state_count = model::kStateDimension * 5;
      request.wall_only_problem.state_lower = Eigen::VectorXd::Constant(
        state_count, -std::numeric_limits<double>::infinity());
      request.wall_only_problem.state_upper = Eigen::VectorXd::Constant(
        state_count, std::numeric_limits<double>::infinity());
      for (int stage = 0; stage <= 4; ++stage) {
        const int state = stage * model::kStateDimension;
        request.wall_only_primal[state + model::kProgressIndex] =
          0.5 * static_cast<double>(stage);
        request.wall_only_primal[state + model::kLateralIndex] =
          0.66 + 0.02 * static_cast<double>(stage);
        request.wall_only_problem.state_lower[
          state + model::kProgressIndex] = 0.0;
        request.wall_only_problem.state_upper[
          state + model::kProgressIndex] = 4.0;
        request.wall_only_problem.state_lower[state + model::kLagIndex] = 0.0;
        request.wall_only_problem.state_upper[state + model::kLagIndex] = 0.0;
        request.wall_only_problem.state_lower[
          state + model::kLateralIndex] = -2.0;
        request.wall_only_problem.state_upper[
          state + model::kLateralIndex] = 2.0;
      }
      request.stages.assign(4, dynamic_obstacle::StagePrediction{
        true, 0.0, 0.0, 0.8, 0.75});
      return request;
    };

  auto witness_request = make_request();
  witness_request.forced_constraint_fraction = 0.0;
  const auto witness = dynamic_obstacle::refine(witness_request);
  ASSERT_TRUE(witness.problem.has_value());
  EXPECT_DOUBLE_EQ(
    witness.problem->dynamic_obstacle_constraints[0].upper, 0.5);
  EXPECT_DOUBLE_EQ(
    witness.problem->dynamic_obstacle_constraints[1].upper, 1.0);
  EXPECT_DOUBLE_EQ(
    witness.problem->dynamic_obstacle_constraints[2].lower, 0.72);
  EXPECT_DOUBLE_EQ(
    witness.problem->dynamic_obstacle_constraints[3].lower, 2.0);

  auto middle_request = make_request();
  middle_request.forced_constraint_fraction = 0.5;
  const auto middle = dynamic_obstacle::refine(middle_request);
  ASSERT_TRUE(middle.problem.has_value());
  EXPECT_NEAR(
    middle.problem->dynamic_obstacle_constraints[0].upper, -0.15, 1e-12);
  EXPECT_NEAR(
    middle.problem->dynamic_obstacle_constraints[1].upper, 0.10, 1e-12);
  EXPECT_NEAR(
    middle.problem->dynamic_obstacle_constraints[2].lower, 0.735, 1e-12);
  EXPECT_NEAR(
    middle.problem->dynamic_obstacle_constraints[3].lower, 1.40, 1e-12);

  auto exact_request = make_request();
  exact_request.forced_constraint_fraction = 1.0;
  const auto exact = dynamic_obstacle::refine(exact_request);
  ASSERT_TRUE(exact.problem.has_value());
  EXPECT_DOUBLE_EQ(
    exact.problem->dynamic_obstacle_constraints[0].upper, -0.8);
  EXPECT_DOUBLE_EQ(
    exact.problem->dynamic_obstacle_constraints[1].upper, -0.8);
  EXPECT_DOUBLE_EQ(
    exact.problem->dynamic_obstacle_constraints[2].lower, 0.75);
  EXPECT_DOUBLE_EQ(
    exact.problem->dynamic_obstacle_constraints[3].lower, 0.8);
}

TEST(
  MpccRateResolvedDynamicObstacle,
  TacticalSidePreservesAcquiredSeparationWhenWallWitnessReturns)
{
  auto request = request_with_lateral_suffix();
  request.pass_side_sign = -1;
  const std::array<double, 5> lateral_m{{-1.0, -1.0, -0.9, -0.4, 0.2}};
  for (int stage = 0; stage <= 4; ++stage) {
    const int state = stage * model::kStateDimension;
    request.wall_only_primal[state + model::kProgressIndex] =
      0.5 * static_cast<double>(stage);
    request.wall_only_primal[state + model::kLagIndex] = 0.0;
    // The current state has already acquired the selected homotopy.  The
    // obstacle-free witness later returns through the target because it has
    // no dynamic-obstacle rows yet; refinement must not copy that drift.
    request.wall_only_primal[state + model::kLateralIndex] =
      lateral_m[static_cast<std::size_t>(stage)];
  }

  const auto result = dynamic_obstacle::refine(request);

  ASSERT_TRUE(result.applied);
  ASSERT_TRUE(result.problem.has_value());
  EXPECT_EQ(result.resolved_side_sign, -1);
  EXPECT_EQ(result.first_pass_side_stage, 0);
  EXPECT_EQ(result.stay_behind_row_count, 0U);
  EXPECT_EQ(result.pass_side_row_count, 4U);
  ASSERT_EQ(result.problem->dynamic_obstacle_constraints.size(), 4U);
  for (std::size_t index = 0U; index < 4U; ++index) {
    const auto & constraint =
      result.problem->dynamic_obstacle_constraints[index];
    EXPECT_EQ(
      constraint.axis,
      problem::DynamicObstacleConstraintAxis::Lateral);
    EXPECT_DOUBLE_EQ(constraint.upper, -0.75);
  }
}

TEST(MpccRateResolvedDynamicObstacle, RejectsMalformedPredictionInsteadOfDroppingIt)
{
  auto request = request_with_lateral_suffix();
  request.stages[1].target_progress_m =
    std::numeric_limits<double>::quiet_NaN();

  const auto result = dynamic_obstacle::refine(request);
  EXPECT_FALSE(result.applied);
  EXPECT_EQ(result.reason, dynamic_obstacle::Reason::InvalidInput);
  EXPECT_FALSE(result.problem.has_value());
}
