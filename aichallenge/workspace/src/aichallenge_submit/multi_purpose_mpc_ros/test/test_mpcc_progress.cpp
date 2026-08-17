#include <gtest/gtest.h>

#include <multi_purpose_mpc_ros/mpcc_progress.hpp>

#include <cmath>
#include <vector>

namespace
{

using multi_purpose_mpc_ros::mpcc_progress::Config;
using multi_purpose_mpc_ros::mpcc_progress::LinearizationRequest;

TEST(MpccProgress, StraightLinearizationAdvancesPhysicalProgress)
{
  const auto result = multi_purpose_mpc_ros::mpcc_progress::linearize_temporal_frenet(
    LinearizationRequest{0.0, 0.0, 10.0, 5.0, 0.0, 0.0, 0.5, Config{}});
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->stage_dt_sec, 0.1, 1e-12);
  EXPECT_NEAR(result->state_matrix(0, 1), 0.5, 1e-12);
  EXPECT_NEAR(result->input_matrix(2, 0), 0.1, 1e-12);

  const Eigen::Vector3d state(0.0, 0.0, 10.0);
  const Eigen::Vector2d input(5.0, 0.0);
  const Eigen::Vector3d next =
    result->state_matrix * state + result->input_matrix * input -
    result->equality_offset;
  EXPECT_NEAR(next[0], 0.0, 1e-12);
  EXPECT_NEAR(next[1], 0.0, 1e-12);
  EXPECT_NEAR(next[2], 10.5, 1e-12);
}

TEST(MpccProgress, CurvedReferenceIsAnEquilibriumInContourAndHeading)
{
  const auto result = multi_purpose_mpc_ros::mpcc_progress::linearize_temporal_frenet(
    LinearizationRequest{0.0, 0.0, 20.0, 5.0, 0.1, 0.1, 0.5, Config{}});
  ASSERT_TRUE(result.has_value());
  const Eigen::Vector3d state(0.0, 0.0, 20.0);
  const Eigen::Vector2d input(5.0, 0.1);
  const Eigen::Vector3d next =
    result->state_matrix * state + result->input_matrix * input -
    result->equality_offset;
  EXPECT_NEAR(next[0], 0.0, 1e-12);
  EXPECT_NEAR(next[1], 0.0, 1e-12);
  EXPECT_NEAR(next[2], 20.5, 1e-12);
}

TEST(MpccProgress, RejectsSingularFrenetReference)
{
  const auto result = multi_purpose_mpc_ros::mpcc_progress::linearize_temporal_frenet(
    LinearizationRequest{10.0, 0.0, 0.0, 5.0, 0.1, 0.1, 0.5, Config{}});
  EXPECT_FALSE(result.has_value());
}

TEST(MpccProgress, SeparatesPathAndInputCurvatureDuringRelinearization)
{
  const auto result = multi_purpose_mpc_ros::mpcc_progress::linearize_temporal_frenet(
    LinearizationRequest{0.0, 0.0, 20.0, 5.0, 0.1, 0.15, 0.5, Config{}});
  ASSERT_TRUE(result.has_value());
  const Eigen::Vector3d state(0.0, 0.0, 20.0);
  const Eigen::Vector2d input(5.0, 0.15);
  const Eigen::Vector3d next =
    result->state_matrix * state + result->input_matrix * input -
    result->equality_offset;
  EXPECT_NEAR(next[0], 0.0, 1e-12);
  EXPECT_NEAR(next[1], 0.025, 1e-12);
  EXPECT_NEAR(next[2], 20.5, 1e-12);
}

TEST(MpccProgress, BuildsUnwrappedProgressReference)
{
  const auto result = multi_purpose_mpc_ros::mpcc_progress::build_progress_reference(
    99.5, std::vector<double>{0.4, 0.6, 0.5});
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->size(), 4U);
  EXPECT_NEAR(result->at(0), 99.5, 1e-12);
  EXPECT_NEAR(result->at(3), 101.0, 1e-12);
}

TEST(MpccProgress, NormalizesCircularSeamWithoutExpandingItToNominalSpacing)
{
  Config config;
  config.minimum_reference_speed_mps = 0.5;
  config.minimum_stage_dt_sec = 0.01;
  const auto result = multi_purpose_mpc_ros::mpcc_progress::resolve_stage_distances(
    std::vector<double>{0.999, 0.0, 1.001}, config);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->normalized_stage_count, 1U);
  EXPECT_NEAR(result->minimum_stage_distance_m, 0.005, 1e-12);
  ASSERT_EQ(result->distance_m.size(), 3U);
  EXPECT_NEAR(result->distance_m[0], 0.999, 1e-12);
  EXPECT_NEAR(result->distance_m[1], 0.005, 1e-12);
  EXPECT_NEAR(result->distance_m[2], 1.001, 1e-12);

  const auto progress = multi_purpose_mpc_ros::mpcc_progress::build_progress_reference(
    348.0, result->distance_m);
  ASSERT_TRUE(progress.has_value());
  EXPECT_NEAR(progress->back(), 350.005, 1e-12);
}

TEST(MpccProgress, RejectsCorruptStageDistanceInsteadOfRepairingIt)
{
  const auto result = multi_purpose_mpc_ros::mpcc_progress::resolve_stage_distances(
    std::vector<double>{1.0, -0.1, 1.0}, Config{});
  EXPECT_FALSE(result.has_value());
}

TEST(MpccProgress, DetectsLapProgressWrapForWarmStartReset)
{
  EXPECT_TRUE(multi_purpose_mpc_ros::mpcc_progress::progress_origin_discontinuous(
    348.0, 0.0, 12.0));
  EXPECT_FALSE(multi_purpose_mpc_ros::mpcc_progress::progress_origin_discontinuous(
    348.0, 349.0, 12.0));
}

TEST(MpccProgress, TrustRegionKeepsMeasuredProgressFeasible)
{
  Config config;
  config.trust_region_backward_m = 3.0;
  config.trust_region_forward_m = 1.0;
  const auto result = multi_purpose_mpc_ros::mpcc_progress::resolve_progress_bounds(
    10.0, 20.0, config);
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(result->lower_m, 10.0);
  EXPECT_DOUBLE_EQ(result->upper_m, 21.0);
}

TEST(MpccProgress, ProgressRewardMovesCostAheadOfLagReference)
{
  Config config;
  config.lag_weight = 100.0;
  config.progress_reward_weight = 20.0;
  const auto result = multi_purpose_mpc_ros::mpcc_progress::resolve_progress_cost(
    5.0, false, config);
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(result->quadratic_weight, 100.0);
  EXPECT_DOUBLE_EQ(result->linear_coefficient, -520.0);
  const double unconstrained_optimum =
    -result->linear_coefficient / result->quadratic_weight;
  EXPECT_NEAR(unconstrained_optimum, 5.2, 1e-12);
}

TEST(MpccProgress, DampsRtiSqpLinearizationPoint)
{
  Eigen::VectorXd previous(3);
  previous << 0.0, 2.0, 4.0;
  Eigen::VectorXd solution(3);
  solution << 2.0, 4.0, 8.0;
  const auto result = multi_purpose_mpc_ros::mpcc_progress::damp_rti_sqp_iterate(
    previous, solution, 0.65);
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR((*result)[0], 1.3, 1e-12);
  EXPECT_NEAR((*result)[1], 3.3, 1e-12);
  EXPECT_NEAR((*result)[2], 6.6, 1e-12);
}

TEST(MpccProgress, RejectsMalformedRtiSqpUpdate)
{
  const Eigen::VectorXd finite = Eigen::VectorXd::Ones(3);
  EXPECT_FALSE(multi_purpose_mpc_ros::mpcc_progress::damp_rti_sqp_iterate(
    finite, Eigen::VectorXd::Ones(2), 0.65).has_value());
  EXPECT_FALSE(multi_purpose_mpc_ros::mpcc_progress::damp_rti_sqp_iterate(
    finite, finite, 0.0).has_value());
}

}  // namespace
