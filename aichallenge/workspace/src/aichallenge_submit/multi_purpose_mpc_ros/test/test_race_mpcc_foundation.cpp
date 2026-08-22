#include <gtest/gtest.h>

#include <multi_purpose_mpc_ros/race_mpcc_foundation.hpp>

#include <string>
#include <vector>

namespace race = multi_purpose_mpc_ros::race_mpcc_foundation;
namespace contract = multi_purpose_mpc_ros::mpcc_execution_contract;

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

TEST(RaceMpccFoundation, EnablesTrackAndCruiseOnlyAtTheExistingMigrationBoundary)
{
  const auto track = race::resolve_track_cruise_shadow_eligibility(
    race::TrackCruiseShadowEligibilityRequest{
      true, true, true, false, false, contract::ControlIntent::Track});
  const auto cruise = race::resolve_track_cruise_shadow_eligibility(
    race::TrackCruiseShadowEligibilityRequest{
      true, true, true, false, false, contract::ControlIntent::Cruise});
  const auto follow = race::resolve_track_cruise_shadow_eligibility(
    race::TrackCruiseShadowEligibilityRequest{
      true, true, true, false, false, contract::ControlIntent::Follow});
  const auto tactical_snapshot = race::resolve_track_cruise_shadow_eligibility(
    race::TrackCruiseShadowEligibilityRequest{
      true, true, true, false, true, contract::ControlIntent::Track});

  EXPECT_TRUE(track.eligible);
  EXPECT_TRUE(cruise.eligible);
  EXPECT_EQ(track.reason, race::TrackCruiseShadowEligibilityReason::Eligible);
  EXPECT_FALSE(follow.eligible);
  EXPECT_EQ(follow.reason, race::TrackCruiseShadowEligibilityReason::IntentNotTrackCruise);
  EXPECT_FALSE(tactical_snapshot.eligible);
  EXPECT_EQ(
    tactical_snapshot.reason,
    race::TrackCruiseShadowEligibilityReason::TacticalSnapshot);
}

namespace
{

race::ShadowWarmStartIdentity shadow_identity(
  const multi_purpose_mpc_ros::mpcc_execution_contract::ControlIntent intent,
  const int tracking_waypoint, const std::vector<int> & state_waypoints)
{
  race::ShadowWarmStartIdentity identity;
  identity.intent = intent;
  identity.formulation = contract::Formulation::VelocityProgress5State;
  identity.horizon_steps = state_waypoints.size();
  identity.state_schema_id = "ey-elag-epsi-v-progress-v1";
  identity.input_schema_id = "accel-curvature-progress-rate-v1";
  identity.bounds_schema_id = "progress-stage-wall-obstacle-v1";
  identity.cost_schema_id = "velocity-progress-v1";
  identity.tracking_waypoint = tracking_waypoint;
  identity.circular = true;
  int from = tracking_waypoint;
  double cumulative = 0.0;
  for (const int state : state_waypoints) {
    cumulative += 1.0;
    identity.stages.push_back(contract::StageGeometryIdentity{
      from, state, 1.0, cumulative});
    from = state;
  }
  identity.stage_geometry_id = contract::fingerprint_stage_geometry(
    identity.tracking_waypoint, identity.circular, identity.stages);
  return identity;
}

}  // namespace

TEST(RaceMpccFoundation, KeepsWarmStartAcrossCompatibleRollingStageGeometry)
{
  const auto previous = shadow_identity(
    contract::ControlIntent::Cruise, 10, {11, 12, 13, 14});
  const auto current = shadow_identity(
    contract::ControlIntent::Cruise, 11, {12, 13, 14, 15});

  const auto result = race::resolve_shadow_warm_start(previous, current);

  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.apply_warm_start);
  EXPECT_FALSE(result.reset_context);
  EXPECT_EQ(result.reason, race::ShadowWarmStartResetReason::None);
}

TEST(RaceMpccFoundation, ResetsWarmStartWhenIntentChanges)
{
  const auto previous = shadow_identity(
    contract::ControlIntent::Track, 10, {11, 12, 13, 14});
  const auto current = shadow_identity(
    contract::ControlIntent::Cruise, 11, {12, 13, 14, 15});

  const auto result = race::resolve_shadow_warm_start(previous, current);

  EXPECT_TRUE(result.valid);
  EXPECT_FALSE(result.apply_warm_start);
  EXPECT_TRUE(result.reset_context);
  EXPECT_EQ(result.reason, race::ShadowWarmStartResetReason::IntentChanged);
}

TEST(RaceMpccFoundation, ResetsWarmStartWhenSchemaOrHorizonChanges)
{
  const auto previous = shadow_identity(
    contract::ControlIntent::Cruise, 10, {11, 12, 13, 14});
  auto schema_changed = shadow_identity(
    contract::ControlIntent::Cruise, 11, {12, 13, 14, 15});
  schema_changed.cost_schema_id = "velocity-progress-v2";
  auto horizon_changed = shadow_identity(
    contract::ControlIntent::Cruise, 11, {12, 13, 14});

  const auto schema_result = race::resolve_shadow_warm_start(previous, schema_changed);
  const auto horizon_result = race::resolve_shadow_warm_start(previous, horizon_changed);

  EXPECT_TRUE(schema_result.reset_context);
  EXPECT_EQ(schema_result.reason, race::ShadowWarmStartResetReason::SchemaChanged);
  EXPECT_TRUE(horizon_result.reset_context);
  EXPECT_EQ(horizon_result.reason, race::ShadowWarmStartResetReason::HorizonChanged);
}

TEST(RaceMpccFoundation, ResetsWarmStartForNonOverlappingStageGeometry)
{
  const auto previous = shadow_identity(
    contract::ControlIntent::Cruise, 10, {11, 12, 13, 14});
  const auto jumped = shadow_identity(
    contract::ControlIntent::Cruise, 30, {31, 32, 33, 34});

  const auto result = race::resolve_shadow_warm_start(previous, jumped);

  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.reset_context);
  EXPECT_EQ(
    result.reason, race::ShadowWarmStartResetReason::StageGeometryDiscontinuous);
}
