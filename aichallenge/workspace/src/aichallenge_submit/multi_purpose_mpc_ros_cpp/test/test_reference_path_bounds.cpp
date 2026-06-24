#include "multi_purpose_mpc_ros_cpp/osqp_mpc.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace multi_purpose_mpc_ros_cpp
{

TEST(ReferencePath, AssignsBoundsFromOccupancyMap)
{
  const std::vector<double> wp_x{1.0, 9.0};
  const std::vector<double> wp_y{5.0, 5.0};

  ReferencePath reference_path(wp_x, wp_y, 1.0, 0, 4.0, false);
  reference_path.assign_bounds_from_map(TEST_SIMPLE_MAP_YAML_PATH, 4.0);

  ASSERT_GT(reference_path.size(), 0U);
  const auto & waypoint = reference_path.get_waypoint(0);
  const auto expected_width = std::sqrt(10.0);
  EXPECT_NEAR(waypoint.ub, expected_width, 1.0e-6);
  EXPECT_NEAR(waypoint.lb, -expected_width, 1.0e-6);
}

}  // namespace multi_purpose_mpc_ros_cpp
