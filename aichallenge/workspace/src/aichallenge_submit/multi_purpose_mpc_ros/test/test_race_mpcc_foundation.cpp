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

TEST(RaceMpccFoundation, EnablesFollowShadowWithoutPromotingOtherIntents)
{
  const auto follow = race::resolve_follow_shadow_eligibility(
    race::FollowShadowEligibilityRequest{
      true, true, true, false, false, contract::ControlIntent::Follow, true});
  const auto cruise = race::resolve_follow_shadow_eligibility(
    race::FollowShadowEligibilityRequest{
      true, true, true, false, false, contract::ControlIntent::Cruise, true});
  const auto live = race::resolve_follow_shadow_eligibility(
    race::FollowShadowEligibilityRequest{
      true, true, true, true, false, contract::ControlIntent::Follow, true});
  const auto retained_label_without_front =
    race::resolve_follow_shadow_eligibility(
    race::FollowShadowEligibilityRequest{
      true, true, true, false, false, contract::ControlIntent::Follow, false});

  EXPECT_TRUE(follow.eligible);
  EXPECT_EQ(follow.reason, race::FollowShadowEligibilityReason::Eligible);
  EXPECT_FALSE(cruise.eligible);
  EXPECT_EQ(cruise.reason, race::FollowShadowEligibilityReason::IntentNotFollow);
  EXPECT_FALSE(live.eligible);
  EXPECT_EQ(live.reason, race::FollowShadowEligibilityReason::LiveProgressAlreadyActive);
  EXPECT_FALSE(retained_label_without_front.eligible);
  EXPECT_EQ(
    retained_label_without_front.reason,
    race::FollowShadowEligibilityReason::NoCoherentFrontObservation);
}

TEST(RaceMpccFoundation, EnablesRejoinShadowOnlyAtTheMigrationBoundary)
{
  const auto rejoin = race::resolve_rejoin_shadow_eligibility(
    race::RejoinShadowEligibilityRequest{
      true, true, true, false, false, contract::ControlIntent::Rejoin});
  const auto cruise = race::resolve_rejoin_shadow_eligibility(
    race::RejoinShadowEligibilityRequest{
      true, true, true, false, false, contract::ControlIntent::Cruise});
  const auto live = race::resolve_rejoin_shadow_eligibility(
    race::RejoinShadowEligibilityRequest{
      true, true, true, true, false, contract::ControlIntent::Rejoin});
  const auto tactical = race::resolve_rejoin_shadow_eligibility(
    race::RejoinShadowEligibilityRequest{
      true, true, true, false, true, contract::ControlIntent::Rejoin});

  EXPECT_TRUE(rejoin.eligible);
  EXPECT_EQ(rejoin.reason, race::RejoinShadowEligibilityReason::Eligible);
  EXPECT_FALSE(cruise.eligible);
  EXPECT_EQ(cruise.reason, race::RejoinShadowEligibilityReason::IntentNotRejoin);
  EXPECT_FALSE(live.eligible);
  EXPECT_EQ(live.reason, race::RejoinShadowEligibilityReason::LiveProgressAlreadyActive);
  EXPECT_FALSE(tactical.eligible);
  EXPECT_EQ(tactical.reason, race::RejoinShadowEligibilityReason::TacticalSnapshot);
}

TEST(RaceMpccFoundation, FollowProductionNeverFallsThroughToAnotherNormalOwner)
{
  EXPECT_EQ(
    race::resolve_follow_production_action(
      contract::ControlIntent::Cruise, false),
    race::FollowProductionAction::NotOwned);
  EXPECT_EQ(
    race::resolve_follow_production_action(
      contract::ControlIntent::Follow, true),
    race::FollowProductionAction::PublishCanonical);
  EXPECT_EQ(
    race::resolve_follow_production_action(
      contract::ControlIntent::Follow, false),
    race::FollowProductionAction::EmergencyStop);
}

TEST(RaceMpccFoundation, EnablesOvertakeFreshShadowOnlyForLiveExecutionIntents)
{
  const auto make_request = [](const contract::ControlIntent intent) {
      return race::OvertakeCanonicalFreshShadowEligibilityRequest{
        true, true, intent, true, true};
    };
  for (const auto intent : {
      contract::ControlIntent::ShiftOut,
      contract::ControlIntent::Pass,
      contract::ControlIntent::Return})
  {
    const auto result =
      race::resolve_overtake_canonical_fresh_shadow_eligibility(
      make_request(intent));
    EXPECT_TRUE(result.eligible);
    EXPECT_EQ(
      result.reason,
      race::OvertakeCanonicalFreshShadowEligibilityReason::Eligible);
  }

  const auto cruise =
    race::resolve_overtake_canonical_fresh_shadow_eligibility(
    make_request(contract::ControlIntent::Cruise));
  EXPECT_FALSE(cruise.eligible);
  EXPECT_EQ(
    cruise.reason,
    race::OvertakeCanonicalFreshShadowEligibilityReason::
    IntentNotOvertakeExecution);

  auto missing_context = make_request(contract::ControlIntent::Pass);
  missing_context.execution_context_available = false;
  EXPECT_FALSE(
    race::resolve_overtake_canonical_fresh_shadow_eligibility(
      missing_context).eligible);

  auto invalid_bounds = make_request(contract::ControlIntent::Pass);
  invalid_bounds.lateral_bounds_valid = false;
  EXPECT_FALSE(
    race::resolve_overtake_canonical_fresh_shadow_eligibility(
      invalid_bounds).eligible);
}

TEST(RaceMpccFoundation, StopEmergencyAuthorityNeverBorrowsNormalControl)
{
  EXPECT_EQ(
    race::resolve_stop_authority_action(contract::ControlIntent::Cruise),
    race::StopAuthorityAction::NotOwned);
  EXPECT_EQ(
    race::resolve_stop_authority_action(contract::ControlIntent::Follow),
    race::StopAuthorityAction::NotOwned);
  EXPECT_EQ(
    race::resolve_stop_authority_action(contract::ControlIntent::Hold),
    race::StopAuthorityAction::NotOwned);
  EXPECT_EQ(
    race::resolve_stop_authority_action(contract::ControlIntent::Stop),
    race::StopAuthorityAction::EmergencyStop);
}

namespace
{

race::FollowLongitudinalContractRequest follow_contract_request()
{
  race::FollowLongitudinalContractRequest request;
  request.intent = contract::ControlIntent::Follow;
  request.target_id = "d2";
  request.target_observation_generation = 7U;
  request.target_observation_age_sec = 0.1;
  request.maximum_target_observation_age_sec = 1.0;
  request.current_target_relative_progress_m = 4.0;
  request.current_ego_progress_offset_m = 0.3;
  request.current_ego_speed_mps = 3.0;
  request.target_speed_mps = 3.0;
  request.moving_target_speed_threshold_mps = 0.5;
  request.desired_gap_m = 4.0;
  request.hard_gap_m = 2.05;
  request.maximum_closing_speed_mps = 0.8;
  request.maximum_recovery_speed_mps = 0.6;
  request.distance_gain_per_sec = 1.0;
  request.slow_target_velocity_cap_mps = 5.0;
  request.braking_deceleration_mps2 = 3.0;
  request.maximum_velocity_mps = 11.11;
  request.stage_dt_sec = {0.5, 0.5};
  request.base_progress_reference_m = {0.0, 1.5, 3.0};
  request.base_progress_upper_m = {0.0, 2.0, 4.0};
  request.base_velocity_reference_mps = {6.0, 6.0};
  request.base_velocity_upper_mps = {11.11, 11.11};
  return request;
}

}  // namespace

TEST(RaceMpccFoundation, BuildsMovingFollowContractAtConfiguredGap)
{
  const auto result = race::build_follow_longitudinal_contract(
    follow_contract_request());

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.reason, race::FollowLongitudinalContractReason::Accepted);
  EXPECT_EQ(result.target_id, "d2");
  EXPECT_EQ(result.target_observation_generation, 7U);
  EXPECT_NEAR(result.current_target_gap_m, 4.0, 1e-9);
  EXPECT_NEAR(result.current_ego_progress_offset_m, 0.3, 1e-9);
  EXPECT_NEAR(result.planning_gap_m, 4.0, 1e-9);
  EXPECT_NEAR(result.hard_gap_m, 2.05, 1e-9);
  ASSERT_EQ(result.target_progress_m.size(), 3U);
  EXPECT_NEAR(result.target_progress_m[0], 4.3, 1e-9);
  EXPECT_NEAR(result.target_progress_m[1], 5.8, 1e-9);
  EXPECT_NEAR(result.target_progress_m[2], 7.3, 1e-9);
  EXPECT_NEAR(result.progress_reference_m[0], 0.0, 1e-9);
  EXPECT_NEAR(result.progress_reference_m[1], 1.5, 1e-9);
  EXPECT_NEAR(result.progress_reference_m[2], 3.0, 1e-9);
  EXPECT_NEAR(result.progress_upper_m[0], 0.0, 1e-9);
  EXPECT_NEAR(result.progress_upper_m[1], 2.0, 1e-9);
  EXPECT_NEAR(result.progress_upper_m[2], 4.0, 1e-9);
  ASSERT_EQ(result.velocity_reference_mps.size(), 2U);
  EXPECT_NEAR(result.velocity_reference_mps[0], 3.3, 1e-9);
  EXPECT_NEAR(result.velocity_reference_mps[1], 3.3, 1e-9);
}

TEST(RaceMpccFoundation, EffectiveFollowGapIncludesFrenetLag)
{
  const auto safe = race::evaluate_follow_effective_gap(
    {4.0, 5.0}, {0.0, 2.0}, {0.0, 0.5}, 2.05, 1e-5);
  ASSERT_TRUE(safe.valid);
  EXPECT_TRUE(safe.satisfied);
  EXPECT_NEAR(safe.minimum_gap_m, 2.5, 1e-9);

  // theta alone leaves 2.1 m and would pass the legacy check. The physical
  // along-track state is theta + e_lag, leaving only 1.8 m.
  const auto forward_lag = race::evaluate_follow_effective_gap(
    {4.0}, {1.9}, {0.3}, 2.05, 1e-5);
  ASSERT_TRUE(forward_lag.valid);
  EXPECT_FALSE(forward_lag.satisfied);
  EXPECT_EQ(forward_lag.worst_stage, 0);
  EXPECT_NEAR(forward_lag.minimum_gap_m, 1.8, 1e-9);
  EXPECT_NEAR(forward_lag.maximum_violation_m, 0.25, 1e-9);
}

TEST(RaceMpccFoundation, PreservesCurrentGapWhenAlreadyInsideNominalGap)
{
  auto request = follow_contract_request();
  request.current_target_relative_progress_m = 3.0;
  request.current_ego_progress_offset_m = 0.25;

  const auto result = race::build_follow_longitudinal_contract(request);

  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.planning_gap_m, 3.0, 1e-9);
  EXPECT_NEAR(result.hard_gap_m, 2.05, 1e-9);
  EXPECT_NEAR(result.target_progress_m.front(), 3.25, 1e-9);
  // State zero is theta=0 with e_lag=0.25. The base theta bound remains zero;
  // the explicit theta+lag constraint preserves the measured 3.0 m gap.
  EXPECT_NEAR(result.progress_upper_m.front(), 0.0, 1e-9);
}

TEST(RaceMpccFoundation, EffectiveFollowGapRejectsMalformedEvidence)
{
  EXPECT_FALSE(race::evaluate_follow_effective_gap(
      {4.0}, {1.0, 2.0}, {0.0}, 2.05, 1e-5).valid);
  EXPECT_FALSE(race::evaluate_follow_effective_gap(
      {4.0}, {1.0}, {std::numeric_limits<double>::infinity()},
      2.05, 1e-5).valid);
}

TEST(RaceMpccFoundation, BuildsStoppedTargetContractThatStopsAtDesiredGap)
{
  auto request = follow_contract_request();
  request.current_target_relative_progress_m = 10.0;
  request.target_speed_mps = 0.0;
  request.stage_dt_sec = {0.5, 0.5, 0.5};
  request.base_progress_reference_m = {0.0, 2.0, 4.0, 6.3};
  request.base_progress_upper_m = {0.0, 3.0, 6.0, 9.0};
  request.base_velocity_reference_mps = {6.0, 6.0, 6.0};
  request.base_velocity_upper_mps = {11.11, 11.11, 11.11};

  const auto result = race::build_follow_longitudinal_contract(request);

  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.progress_reference_m.back(), 6.3, 1e-9);
  EXPECT_NEAR(result.progress_upper_m.back(), 9.0, 1e-9);
  EXPECT_GT(result.velocity_reference_mps.front(), 0.0);
  EXPECT_NEAR(result.velocity_reference_mps.back(), 0.0, 1e-6);
}

TEST(RaceMpccFoundation, RampsFollowVelocityCapByReachableBrakingEnvelope)
{
  auto request = follow_contract_request();
  request.current_target_relative_progress_m = 24.0;
  request.current_ego_speed_mps = 9.0;
  request.target_speed_mps = 0.0;
  request.stage_dt_sec = {0.1, 0.1, 0.1};
  request.base_progress_reference_m = {0.0, 0.9, 1.8, 2.7};
  request.base_progress_upper_m = {0.0, 1.0, 2.0, 3.0};
  request.base_velocity_reference_mps = {9.0, 9.0, 9.0};
  request.base_velocity_upper_mps = {11.11, 11.11, 11.11};

  const auto result = race::build_follow_longitudinal_contract(request);

  ASSERT_TRUE(result.valid);
  ASSERT_EQ(result.velocity_upper_mps.size(), 3U);
  ASSERT_EQ(result.velocity_reference_mps.size(), 3U);
  EXPECT_NEAR(result.velocity_upper_mps[0], 8.7, 1e-9);
  EXPECT_NEAR(result.velocity_upper_mps[1], 8.4, 1e-9);
  EXPECT_NEAR(result.velocity_upper_mps[2], 8.1, 1e-9);
  EXPECT_NEAR(result.velocity_reference_mps[0], 5.0, 1e-9);
  EXPECT_NEAR(result.velocity_reference_mps[1], 5.0, 1e-9);
  EXPECT_NEAR(result.velocity_reference_mps[2], 5.0, 1e-9);
}

TEST(RaceMpccFoundation, DoesNotRestrictAnOpeningFollowGap)
{
  auto request = follow_contract_request();
  request.current_target_relative_progress_m = 6.0;
  request.target_speed_mps = 8.0;
  request.stage_dt_sec = {0.5, 0.5};
  request.base_progress_reference_m = {0.0, 2.0, 4.0};
  request.base_progress_upper_m = {0.0, 3.0, 6.0};
  request.base_velocity_reference_mps = {6.0, 6.0};
  request.base_velocity_upper_mps = {11.11, 11.11};

  const auto result = race::build_follow_longitudinal_contract(request);

  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.progress_reference_m[1], 2.0, 1e-9);
  EXPECT_NEAR(result.progress_reference_m[2], 4.0, 1e-9);
  EXPECT_NEAR(result.velocity_reference_mps[0], 6.0, 1e-9);
  EXPECT_NEAR(result.velocity_reference_mps[1], 6.0, 1e-9);
}

TEST(RaceMpccFoundation, RejectsStaleOrDisappearingFollowTarget)
{
  auto stale = follow_contract_request();
  stale.target_observation_age_sec = 1.01;
  const auto stale_result = race::build_follow_longitudinal_contract(stale);
  EXPECT_FALSE(stale_result.valid);
  EXPECT_EQ(
    stale_result.reason,
    race::FollowLongitudinalContractReason::StaleTargetObservation);

  auto missing = follow_contract_request();
  missing.target_observation_generation = 0U;
  const auto missing_result = race::build_follow_longitudinal_contract(missing);
  EXPECT_FALSE(missing_result.valid);
  EXPECT_EQ(
    missing_result.reason,
    race::FollowLongitudinalContractReason::InvalidTargetIdentity);
}

TEST(RaceMpccFoundation, RejectsCurrentHardGapViolationInsteadOfClampingIt)
{
  auto request = follow_contract_request();
  request.current_target_relative_progress_m = 2.0;

  const auto result = race::build_follow_longitudinal_contract(request);

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(
    result.reason,
    race::FollowLongitudinalContractReason::InitialHardGapViolation);
}

TEST(RaceMpccFoundation, SeparatesInvalidTargetKinematicsFromInvalidConfiguration)
{
  auto invalid_target = follow_contract_request();
  invalid_target.current_target_relative_progress_m =
    std::numeric_limits<double>::infinity();
  const auto target_result =
    race::build_follow_longitudinal_contract(invalid_target);
  EXPECT_FALSE(target_result.valid);
  EXPECT_EQ(
    target_result.reason,
    race::FollowLongitudinalContractReason::InvalidTargetKinematics);

  auto invalid_origin = follow_contract_request();
  invalid_origin.current_ego_progress_offset_m =
    std::numeric_limits<double>::quiet_NaN();
  const auto origin_result =
    race::build_follow_longitudinal_contract(invalid_origin);
  EXPECT_FALSE(origin_result.valid);
  EXPECT_EQ(
    origin_result.reason,
    race::FollowLongitudinalContractReason::InvalidProgressOrigin);

  auto invalid_configuration = follow_contract_request();
  invalid_configuration.desired_gap_m = 1.0;
  const auto configuration_result =
    race::build_follow_longitudinal_contract(invalid_configuration);
  EXPECT_FALSE(configuration_result.valid);
  EXPECT_EQ(
    configuration_result.reason,
    race::FollowLongitudinalContractReason::InvalidConfiguration);
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

TEST(RaceMpccFoundation, AcceptsCompatibleFollowWarmStartIdentity)
{
  const auto previous = shadow_identity(
    contract::ControlIntent::Follow, 10, {11, 12, 13, 14});
  const auto current = shadow_identity(
    contract::ControlIntent::Follow, 11, {12, 13, 14, 15});

  const auto result = race::resolve_shadow_warm_start(previous, current);

  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.apply_warm_start);
  EXPECT_FALSE(result.reset_context);
}

TEST(RaceMpccFoundation, AcceptsInitialShiftOutWarmStartIdentity)
{
  const auto current = shadow_identity(
    contract::ControlIntent::ShiftOut, 10, {11, 12, 13, 14});

  const auto result = race::resolve_shadow_warm_start(std::nullopt, current);

  EXPECT_TRUE(result.valid);
  EXPECT_FALSE(result.apply_warm_start);
  EXPECT_TRUE(result.reset_context);
  EXPECT_EQ(result.reason, race::ShadowWarmStartResetReason::InitialContext);
}

TEST(RaceMpccFoundation, ResetsWarmStartAcrossExactOvertakeIntentChange)
{
  const auto previous = shadow_identity(
    contract::ControlIntent::ShiftOut, 10, {11, 12, 13, 14});
  const auto current = shadow_identity(
    contract::ControlIntent::Pass, 11, {12, 13, 14, 15});

  const auto result = race::resolve_shadow_warm_start(previous, current);

  EXPECT_TRUE(result.valid);
  EXPECT_FALSE(result.apply_warm_start);
  EXPECT_TRUE(result.reset_context);
  EXPECT_EQ(result.reason, race::ShadowWarmStartResetReason::IntentChanged);
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

TEST(RaceMpccFoundation, ExactPhysicalTrajectoryRequiresEveryFiveStateField)
{
  race::ExactPhysicalExecutionTrajectory trajectory;
  trajectory.progress_origin_m = 100.0;
  trajectory.path_distance_m = {1.0, 2.0};
  trajectory.lateral_m = {0.2, 0.4};
  trajectory.lag_m = {0.1, -0.1};
  trajectory.heading_offset_rad = {0.15, -0.20};
  trajectory.velocity_mps = {4.0, 4.5};
  trajectory.progress_m = {101.1, 102.2};
  trajectory.lateral_lower_m = {-0.5, -0.5};
  trajectory.lateral_upper_m = {0.8, 0.8};
  trajectory.minimum_lateral_bound_reserve_m = 0.4;

  EXPECT_TRUE(race::exact_physical_execution_trajectory_complete(trajectory));

  trajectory.lag_m.clear();
  EXPECT_FALSE(race::exact_physical_execution_trajectory_complete(trajectory));
  trajectory.lag_m = {0.1, -0.1};
  trajectory.heading_offset_rad.clear();
  EXPECT_FALSE(race::exact_physical_execution_trajectory_complete(trajectory));
  trajectory.heading_offset_rad = {0.15, -0.20};
  trajectory.progress_m.clear();
  EXPECT_FALSE(race::exact_physical_execution_trajectory_complete(trajectory));
}

TEST(RaceMpccFoundation, ExactPhysicalTrajectoryRejectsSemanticDiscontinuity)
{
  race::ExactPhysicalExecutionTrajectory trajectory;
  trajectory.progress_origin_m = 100.0;
  trajectory.path_distance_m = {1.0, 2.0};
  trajectory.lateral_m = {0.2, 0.4};
  trajectory.lag_m = {0.1, -0.1};
  trajectory.heading_offset_rad = {0.15, -0.20};
  trajectory.velocity_mps = {4.0, 4.5};
  trajectory.progress_m = {101.1, 102.2};
  trajectory.lateral_lower_m = {-0.5, -0.5};
  trajectory.lateral_upper_m = {0.8, 0.8};
  trajectory.minimum_lateral_bound_reserve_m = 0.4;

  trajectory.progress_m[1] = 100.5;
  const auto progress = race::validate_exact_physical_execution_trajectory(trajectory);
  EXPECT_FALSE(progress.complete);
  EXPECT_EQ(
    progress.reason,
    race::ExactPhysicalExecutionTrajectoryReason::ProgressRegressed);
  EXPECT_EQ(progress.stage, 1);
  trajectory.progress_m[1] = 102.2;
  trajectory.lateral_lower_m[1] = 0.9;
  const auto bounds = race::validate_exact_physical_execution_trajectory(trajectory);
  EXPECT_FALSE(bounds.complete);
  EXPECT_EQ(
    bounds.reason,
    race::ExactPhysicalExecutionTrajectoryReason::InvalidLateralBounds);
  EXPECT_EQ(bounds.stage, 1);
}

TEST(RaceMpccFoundation, ExactPhysicalTrajectoryReportsNegativeVelocityStage)
{
  race::ExactPhysicalExecutionTrajectory trajectory;
  trajectory.progress_origin_m = 100.0;
  trajectory.path_distance_m = {1.0, 2.0};
  trajectory.lateral_m = {0.2, 0.4};
  trajectory.lag_m = {0.1, -0.1};
  trajectory.heading_offset_rad = {0.15, -0.20};
  trajectory.velocity_mps = {4.0, -1e-8};
  trajectory.progress_m = {101.1, 102.2};
  trajectory.lateral_lower_m = {-0.5, -0.5};
  trajectory.lateral_upper_m = {0.8, 0.8};
  trajectory.minimum_lateral_bound_reserve_m = 0.4;

  const auto validation =
    race::validate_exact_physical_execution_trajectory(trajectory);
  EXPECT_FALSE(validation.complete);
  EXPECT_EQ(
    validation.reason,
    race::ExactPhysicalExecutionTrajectoryReason::InvalidVelocity);
  EXPECT_EQ(validation.stage, 1);
}
