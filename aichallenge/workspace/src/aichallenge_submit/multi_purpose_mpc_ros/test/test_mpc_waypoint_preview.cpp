#include <gtest/gtest.h>

#include <multi_purpose_mpc_ros/mpc_waypoint_preview.hpp>

#include <limits>
#include <stdexcept>

namespace preview = multi_purpose_mpc_ros::mpc_waypoint_preview;

TEST(MpcWaypointPreview, AcceptsOnlySupportedOffsets)
{
  EXPECT_TRUE(preview::is_valid_offset(0));
  EXPECT_TRUE(preview::is_valid_offset(1));
  EXPECT_TRUE(preview::is_valid_offset(2));
  EXPECT_FALSE(preview::is_valid_offset(-1));
  EXPECT_FALSE(preview::is_valid_offset(3));
}

TEST(MpcWaypointPreview, SelectsLowSpeedOffsetAtThreshold)
{
  EXPECT_EQ(preview::select_effective_offset(2, 1, 3.0, 4.0), 1);
  EXPECT_EQ(preview::select_effective_offset(2, 1, 4.0, 4.0), 1);
  EXPECT_EQ(preview::select_effective_offset(2, 1, 4.1, 4.0), 2);
  EXPECT_EQ(preview::select_effective_offset(2, 1, 0.0, 0.0), 2);
}

TEST(MpcWaypointPreview, WrapsCircularPathAtLapBoundary)
{
  EXPECT_EQ(preview::resolve_preview_index(4, 2, 5, true), 1);
  EXPECT_EQ(preview::resolve_preview_index(4, 0, 5, true), 4);
}

TEST(MpcWaypointPreview, ClampsNonCircularPathAtEnd)
{
  EXPECT_EQ(preview::resolve_preview_index(4, 2, 5, false), 4);
  EXPECT_EQ(preview::resolve_preview_index(1, 2, 5, false), 3);
}

TEST(MpcWaypointPreview, RejectsInvalidInputs)
{
  EXPECT_THROW(preview::resolve_preview_index(0, 3, 5, true), std::invalid_argument);
  EXPECT_THROW(preview::resolve_preview_index(0, 0, 0, true), std::invalid_argument);
  EXPECT_THROW(preview::resolve_preview_index(5, 0, 5, true), std::out_of_range);
  EXPECT_THROW(
    preview::select_effective_offset(
      0, 0, std::numeric_limits<double>::quiet_NaN(), 1.0),
    std::invalid_argument);
}

TEST(MpcWaypointPreview, PreviewsEarlierBrakingWithoutEarlierAcceleration)
{
  const auto braking = preview::resolve_input_reference(10.0, 0.0, 7.0, 0.0);
  const auto accelerating = preview::resolve_input_reference(7.0, 0.0, 10.0, 0.0);

  EXPECT_DOUBLE_EQ(braking.velocity_mps, 7.0);
  EXPECT_DOUBLE_EQ(accelerating.velocity_mps, 7.0);
}

TEST(MpcWaypointPreview, PreviewsTurnEntryAndTightening)
{
  const auto turn_entry = preview::resolve_input_reference(8.0, 0.0, 8.0, 0.2);
  const auto tightening = preview::resolve_input_reference(8.0, 0.1, 8.0, 0.2);

  EXPECT_DOUBLE_EQ(turn_entry.curvature_radpm, 0.2);
  EXPECT_DOUBLE_EQ(tightening.curvature_radpm, 0.2);
}

TEST(MpcWaypointPreview, DoesNotUnwindSteeringEarlyOnTurnExit)
{
  const auto left_exit = preview::resolve_input_reference(8.0, 0.2, 8.0, 0.1);
  const auto right_exit = preview::resolve_input_reference(8.0, -0.2, 8.0, -0.1);

  EXPECT_DOUBLE_EQ(left_exit.curvature_radpm, 0.2);
  EXPECT_DOUBLE_EQ(right_exit.curvature_radpm, -0.2);
}

TEST(MpcWaypointPreview, DoesNotPreviewOppositeTurnAcrossSignChange)
{
  const auto reference = preview::resolve_input_reference(8.0, 0.05, 8.0, -0.2);

  EXPECT_DOUBLE_EQ(reference.curvature_radpm, 0.05);
}

TEST(MpcWaypointPreview, RejectsInvalidReference)
{
  EXPECT_THROW(
    preview::resolve_input_reference(
      -1.0, 0.0, 1.0, 0.0),
    std::invalid_argument);
  EXPECT_THROW(
    preview::resolve_input_reference(
      1.0, 0.0, 1.0, std::numeric_limits<double>::infinity()),
    std::invalid_argument);
}
