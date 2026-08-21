#include <gtest/gtest.h>

#include <multi_purpose_mpc_ros/race_mpcc_foundation.hpp>

#include <string>

namespace race = multi_purpose_mpc_ros::race_mpcc_foundation;

namespace
{

race::TargetProvenance provenance(
  const std::uint64_t generation, const double progress, const double lateral,
  const race::TargetProvenanceStage stage = race::TargetProvenanceStage::Observed)
{
  return race::TargetProvenance{
    true, "d2", 10.0 + static_cast<double>(generation), 20.0, progress, lateral,
    generation, stage};
}

}  // namespace

TEST(RaceMpccFoundation, AcceptsSameTargetObservation)
{
  const auto value = provenance(3U, 98.0, 0.4);
  const auto result = race::validate_target_provenance(
    race::TargetProvenanceValidationRequest{
      value, value, true, 100.0, 0.25, 2.0, 0.5});

  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.same_observation);
}

TEST(RaceMpccFoundation, UnwrapsCurrentTargetAcrossCircularSeam)
{
  const auto result = race::validate_target_provenance(
    race::TargetProvenanceValidationRequest{
      provenance(3U, 99.0, 0.4), provenance(4U, 0.5, 0.5),
      true, 100.0, 0.25, 2.0, 0.5});

  EXPECT_TRUE(result.valid);
  EXPECT_NEAR(result.progress_delta_m, 1.5, 1e-9);
  EXPECT_NEAR(result.lateral_delta_m, 0.1, 1e-9);
}

TEST(RaceMpccFoundation, RejectsMovedTargetOutsideCertificateTube)
{
  const auto result = race::validate_target_provenance(
    race::TargetProvenanceValidationRequest{
      provenance(3U, 10.0, -0.2), provenance(4U, 11.0, 0.6),
      false, 100.0, 0.25, 2.0, 0.5});

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reject_reason, race::TargetProvenanceRejectReason::LateralDelta);
}

TEST(RaceMpccFoundation, PromotesObservedTargetToLockedTarget)
{
  const auto result = race::validate_target_provenance(
    race::TargetProvenanceValidationRequest{
      provenance(3U, 10.0, -0.2),
      provenance(4U, 11.0, -0.1, race::TargetProvenanceStage::Locked),
      false, 100.0, 0.25, 2.0, 0.5});

  EXPECT_TRUE(result.valid);
}

TEST(RaceMpccFoundation, RejectsLockedTargetLifecycleRegression)
{
  const auto result = race::validate_target_provenance(
    race::TargetProvenanceValidationRequest{
      provenance(3U, 10.0, -0.2, race::TargetProvenanceStage::Locked),
      provenance(4U, 11.0, -0.1),
      false, 100.0, 0.25, 2.0, 0.5});

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reject_reason, race::TargetProvenanceRejectReason::StageRegression);
}

TEST(RaceMpccFoundation, FormatsAllFourHomotopiesWithShadowAuthority)
{
  race::ShadowDecision decision;
  decision.context_epoch = 4U;
  decision.target_id = "d2";
  decision.target_provenance = provenance(4U, 12.0, 0.2);
  decision.stage_geometry_valid = true;
  decision.stage_count = 20U;
  decision.horizon_distance_m = 18.0;
  decision.candidates[0].homotopy = race::Homotopy::Left;
  decision.candidates[1].homotopy = race::Homotopy::Right;
  decision.candidates[2].homotopy = race::Homotopy::Hold;
  decision.candidates[3].homotopy = race::Homotopy::Return;
  decision.selected = race::Homotopy::Hold;
  decision.selection_reason = "no-executable-pass";

  const std::string output = race::format_shadow_decision(decision);
  EXPECT_NE(output.find("left="), std::string::npos);
  EXPECT_NE(output.find("right="), std::string::npos);
  EXPECT_NE(output.find("target_provenance=1/observed/4"), std::string::npos);
  EXPECT_NE(output.find("hold="), std::string::npos);
  EXPECT_NE(output.find("return="), std::string::npos);
  EXPECT_NE(output.find("authority=shadow"), std::string::npos);
}
