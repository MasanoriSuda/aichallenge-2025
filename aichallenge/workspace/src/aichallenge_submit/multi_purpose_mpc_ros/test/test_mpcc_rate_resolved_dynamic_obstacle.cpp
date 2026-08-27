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
  EXPECT_EQ(result.partial_escape_row_count, 0U);
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

TEST(MpccRateResolvedDynamicObstacle, CruiseRejectsUnreachableStayBehindAndKeepsPartialSide)
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
  EXPECT_EQ(result.first_pass_side_stage, 0);
  EXPECT_EQ(result.stay_behind_row_count, 0U);
  EXPECT_EQ(result.pass_side_row_count, 4U);
  EXPECT_EQ(result.partial_escape_row_count, 4U);
  for (const auto & constraint : result.problem->dynamic_obstacle_constraints) {
    EXPECT_EQ(
      constraint.axis,
      problem::DynamicObstacleConstraintAxis::Lateral);
    EXPECT_DOUBLE_EQ(constraint.lower, 0.70);
  }
}

TEST(MpccRateResolvedDynamicObstacle, TacticalSideUsesTheSamePartialEscapeContract)
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
  EXPECT_EQ(result.first_pass_side_stage, 0);
  EXPECT_EQ(result.stay_behind_row_count, 0U);
  EXPECT_EQ(result.pass_side_row_count, 4U);
  EXPECT_EQ(result.partial_escape_row_count, 2U);
  const std::array<double, 4> expected_lower{{0.72, 0.74, 0.75, 0.75}};
  for (std::size_t index = 0U; index < expected_lower.size(); ++index) {
    EXPECT_EQ(
      result.problem->dynamic_obstacle_constraints[index].axis,
      problem::DynamicObstacleConstraintAxis::Lateral);
    EXPECT_DOUBLE_EQ(
      result.problem->dynamic_obstacle_constraints[index].lower,
      expected_lower[index]);
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
