#include "multi_purpose_mpc_ros/mpcc_rate_resolved_dynamic_proof.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_dynamic_proof
{
namespace
{

TEST(MpccRateResolvedDynamicProof, RecordsFirstDenseRejectionProvenance)
{
  const recovery::FootprintExtents footprint{0.5, 0.5, 0.4, 0.4, 0.0};
  WorldObservation world;
  world.generation = 3U;
  world.observed_sec = 10.0;
  world.current = true;
  world.obstacles.push_back(
    DynamicObstacle{"d2", recovery::CircleObstacle{2.0, 0.0, 0.0, 0.0, 0.2}});
  Result result;

  observe_segment(
    footprint, recovery::Pose2D{0.0, 0.0, 0.0},
    recovery::Pose2D{3.0, 0.0, 0.0}, 0.0, 3.0, 0.05, world, result);

  EXPECT_TRUE(result.valid);
  EXPECT_FALSE(result.clear);
  EXPECT_EQ(
    result.rejection_reason,
    recovery::DynamicClearanceRejectReason::NewOverlap);
  EXPECT_EQ(result.blocking_obstacle_id, "d2");
  EXPECT_EQ(result.rejected_obstacle_id, "d2");
  EXPECT_TRUE(std::isfinite(result.rejected_elapsed_sec));
  EXPECT_GT(result.rejected_elapsed_sec, 0.0);
  EXPECT_TRUE(std::isfinite(result.rejected_pose.x_m));
  EXPECT_LT(result.rejected_clearance_m, 0.0);
  EXPECT_EQ(result.minimum_clearance_obstacle_id, "d2");
  EXPECT_TRUE(std::isfinite(result.minimum_clearance_elapsed_sec));
  EXPECT_LE(result.minimum_clearance_m, result.rejected_clearance_m);
}

}  // namespace
}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_dynamic_proof
