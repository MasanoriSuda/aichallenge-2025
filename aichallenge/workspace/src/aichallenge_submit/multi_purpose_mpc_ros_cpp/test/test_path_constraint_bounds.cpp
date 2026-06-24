#include <gtest/gtest.h>

#include "multi_purpose_mpc_ros_cpp/osqp_mpc.hpp"

namespace multi_purpose_mpc_ros_cpp
{
namespace
{

TEST(PathConstraintBoundsTest, KeepsNominalBoundsWhenMarginIsUnchanged)
{
  const auto bounds =
    adjust_path_constraint_bounds_for_safety_margin(-0.7, 1.2, 0.5, 0.5);

  EXPECT_DOUBLE_EQ(bounds.first, -0.7);
  EXPECT_DOUBLE_EQ(bounds.second, 1.2);
}

TEST(PathConstraintBoundsTest, RelaxesBoundsWhenRequestedMarginIsSmaller)
{
  const auto bounds =
    adjust_path_constraint_bounds_for_safety_margin(-0.7, 1.2, 0.5, 0.3);

  EXPECT_DOUBLE_EQ(bounds.first, -0.9);
  EXPECT_DOUBLE_EQ(bounds.second, 1.4);
}

TEST(PathConstraintBoundsTest, FallsBackToZeroWhenAdjustedBoundsBecomeInfeasible)
{
  const auto bounds =
    adjust_path_constraint_bounds_for_safety_margin(-0.1, 0.2, 0.5, 0.7);

  EXPECT_DOUBLE_EQ(bounds.first, 0.0);
  EXPECT_DOUBLE_EQ(bounds.second, 0.0);
}

}  // namespace
}  // namespace multi_purpose_mpc_ros_cpp
