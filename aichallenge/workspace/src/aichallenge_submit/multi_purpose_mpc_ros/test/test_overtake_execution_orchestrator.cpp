#include "multi_purpose_mpc_ros/overtake_execution_orchestrator.hpp"

#include <gtest/gtest.h>

#include <string>
#include <vector>

namespace {

namespace orchestrator =
  multi_purpose_mpc_ros::overtake_execution_orchestrator;
namespace contract = multi_purpose_mpc_ros::mpcc_execution_contract;

TEST(OvertakeExecutionOrchestrator, ResolvesCommittedPassOwners)
{
  orchestrator::AuthorityRequest request;
  request.episode_id = 7U;
  request.mission_generation = 2U;
  request.target_id = "d2";
  request.phase = orchestrator::Phase::Pass;
  request.behavior = orchestrator::Behavior::Overtake;
  request.line_active = true;
  request.stage_corridor_active = true;
  request.pass_speed_floor_active = true;
  request.front_cap_release_ready = true;
  request.speed_reference_mps = 8.0;
  request.speed_limit_mps = 9.0;
  request.speed_floor_mps = 7.5;

  const auto result = orchestrator::resolve_authority(request);
  EXPECT_TRUE(result.relevant);
  EXPECT_EQ(result.action, orchestrator::Action::Pass);
  EXPECT_EQ(result.lateral_owner, orchestrator::LateralOwner::OvertakeLine);
  EXPECT_EQ(result.longitudinal_owner, orchestrator::LongitudinalOwner::PassFloor);
  EXPECT_EQ(result.path_source, orchestrator::PathSource::FrozenMission);
  EXPECT_TRUE(result.use_overtake_line_target);
  EXPECT_TRUE(result.apply_overtake_speed_reference);
  EXPECT_TRUE(result.apply_overtake_speed_limit);
  EXPECT_TRUE(result.apply_overtake_speed_floor);
  EXPECT_EQ(result.conflicts, orchestrator::NoConflict);
}

TEST(OvertakeExecutionOrchestrator, TreatsDynamicEscapeAndGapPlannerAsOneChain)
{
  orchestrator::AuthorityRequest request;
  request.target_id = "d2";
  request.behavior = orchestrator::Behavior::Follow;
  request.gap_planner_active = true;
  request.dynamic_obstacle_escape_active = true;
  request.dynamic_obstacle_follow_cap_suppressed = true;

  const auto result = orchestrator::resolve_authority(request);
  EXPECT_EQ(result.action, orchestrator::Action::DynamicEscape);
  EXPECT_EQ(
    result.lateral_owner, orchestrator::LateralOwner::DynamicObstacleEscape);
  EXPECT_EQ(
    result.path_source, orchestrator::PathSource::DynamicObstacleEscape);
  EXPECT_EQ(result.conflicts & orchestrator::MultipleLateralAuthorities, 0U);
}

TEST(OvertakeExecutionOrchestrator, PromotesValidatedDynamicEscapeIdentity)
{
  orchestrator::CanonicalExecutionIdentityRequest request;
  request.dynamic_escape_active = true;
  request.dynamic_escape_path_validated = true;
  request.dynamic_escape_target_id = "d2";
  request.dynamic_escape_attempt_id = 17U;
  request.dynamic_escape_side_sign = -1;

  const auto result =
    orchestrator::resolve_canonical_execution_identity(request);

  ASSERT_TRUE(result.active);
  EXPECT_EQ(
    result.source,
    orchestrator::CanonicalExecutionIdentitySource::DynamicObstacleEscape);
  EXPECT_EQ(result.target_id, "d2");
  EXPECT_EQ(result.generation, 17U);
  EXPECT_EQ(result.phase, orchestrator::Phase::ShiftOut);
  EXPECT_EQ(result.side_sign, -1);
  EXPECT_TRUE(result.target_exclusion_certified);
}

TEST(OvertakeExecutionOrchestrator, RejectsDynamicEscapeWithoutExecutionSide)
{
  orchestrator::CanonicalExecutionIdentityRequest request;
  request.dynamic_escape_active = true;
  request.dynamic_escape_path_validated = true;
  request.dynamic_escape_target_id = "d2";
  request.dynamic_escape_attempt_id = 17U;

  const auto result =
    orchestrator::resolve_canonical_execution_identity(request);

  EXPECT_FALSE(result.active);
  EXPECT_EQ(
    result.reason,
    orchestrator::CanonicalExecutionIdentityReason::
    MalformedDynamicObstacleEscape);
}

TEST(OvertakeExecutionOrchestrator, KeepsCommittedLineIdentityPrecedence)
{
  orchestrator::CanonicalExecutionIdentityRequest request;
  request.overtake_line_active = true;
  request.overtake_line_target_id = "line-target";
  request.overtake_line_mission_generation = 5U;
  request.overtake_line_phase = orchestrator::Phase::Pass;
  request.overtake_line_side_sign = 1;
  request.overtake_line_traveled_m = 3.5;
  request.overtake_line_target_exclusion_certified = true;
  request.dynamic_escape_active = true;
  request.dynamic_escape_path_validated = true;
  request.dynamic_escape_target_id = "escape-target";
  request.dynamic_escape_attempt_id = 17U;
  request.dynamic_escape_side_sign = -1;

  const auto result =
    orchestrator::resolve_canonical_execution_identity(request);

  ASSERT_TRUE(result.active);
  EXPECT_EQ(
    result.source,
    orchestrator::CanonicalExecutionIdentitySource::OvertakeLine);
  EXPECT_EQ(result.target_id, "line-target");
  EXPECT_EQ(result.generation, 5U);
  EXPECT_EQ(result.phase, orchestrator::Phase::Pass);
  EXPECT_EQ(result.side_sign, 1);
  EXPECT_DOUBLE_EQ(result.traveled_m, 3.5);
}

TEST(OvertakeExecutionOrchestrator, ExposesConflictingAuthorities)
{
  orchestrator::AuthorityRequest request;
  request.episode_id = 4U;
  request.phase = orchestrator::Phase::Pass;
  request.behavior = orchestrator::Behavior::SafetyBrake;
  request.line_active = true;
  request.gap_planner_active = true;
  request.follow_cap_active = true;
  request.front_cap_release_ready = true;
  request.pass_speed_floor_active = true;
  request.speed_reference_mps = 3.0;
  request.speed_limit_mps = 2.0;
  request.speed_floor_mps = 4.0;

  const auto result = orchestrator::resolve_authority(request);
  EXPECT_EQ(result.action, orchestrator::Action::SafetyBrake);
  EXPECT_EQ(result.longitudinal_owner, orchestrator::LongitudinalOwner::SafetyBrake);
  EXPECT_NE(result.conflicts & orchestrator::SafetyWithActiveLine, 0U);
  EXPECT_NE(result.conflicts & orchestrator::SafetyWithSpeedFloor, 0U);
  EXPECT_NE(result.conflicts & orchestrator::ReleasedPassWithFollowCap, 0U);
  EXPECT_NE(result.conflicts & orchestrator::ActivePhaseWithoutTarget, 0U);
  EXPECT_NE(result.conflicts & orchestrator::MultipleLateralAuthorities, 0U);
  EXPECT_NE(result.conflicts & orchestrator::InvalidSpeedWindow, 0U);
  const auto names = orchestrator::format_conflicts(result.conflicts);
  EXPECT_NE(names.find("safety-with-line"), std::string::npos);
  EXPECT_NE(names.find("invalid-speed-window"), std::string::npos);
}

TEST(OvertakeExecutionOrchestrator, ClassifiesDynamicWaitWithoutPrefix)
{
  orchestrator::AuthorityRequest request;
  request.episode_id = 2U;
  request.target_id = "d3";
  request.phase = orchestrator::Phase::FollowPrepare;
  request.behavior = orchestrator::Behavior::Follow;
  request.dynamic_wait_active = true;

  const auto result = orchestrator::resolve_authority(request);
  EXPECT_EQ(result.action, orchestrator::Action::DynamicWait);
  EXPECT_EQ(result.lateral_owner, orchestrator::LateralOwner::RacingLine);
  EXPECT_NE(
    result.conflicts & orchestrator::DynamicWaitWithoutLateralAuthority, 0U);
}

TEST(OvertakeExecutionOrchestrator, DynamicWaitLateralHoldOwnsPathWithoutForwardAuthority)
{
  orchestrator::AuthorityRequest request;
  request.episode_id = 2U;
  request.target_id = "d3";
  request.phase = orchestrator::Phase::FollowPrepare;
  request.behavior = orchestrator::Behavior::Follow;
  request.dynamic_wait_active = true;
  request.dynamic_wait_lateral_authority_active = true;
  request.dynamic_wait_forward_prefix_active = false;
  request.line_active = true;

  const auto result = orchestrator::resolve_authority(request);
  EXPECT_EQ(result.action, orchestrator::Action::DynamicWait);
  EXPECT_EQ(result.lateral_owner, orchestrator::LateralOwner::DynamicWaitPrefix);
  EXPECT_EQ(result.path_source, orchestrator::PathSource::DynamicWaitPrefix);
  EXPECT_EQ(
    result.conflicts & orchestrator::DynamicWaitWithoutLateralAuthority, 0U);
}

TEST(OvertakeExecutionOrchestrator, PreservesLateralHoldDynamicWaitOriginIntent)
{
  orchestrator::AuthorityRequest shiftout;
  shiftout.mission_generation = 2U;
  shiftout.target_id = "d2";
  shiftout.dynamic_wait_active = true;
  shiftout.dynamic_wait_lateral_authority_active = true;
  shiftout.dynamic_wait_origin_phase = orchestrator::Phase::ShiftOut;
  shiftout.line_active = true;
  const auto shiftout_authority = orchestrator::resolve_authority(shiftout);
  const auto shiftout_intent = orchestrator::resolve_canonical_control_intent(
    shiftout, shiftout_authority);

  EXPECT_TRUE(shiftout_intent.valid);
  EXPECT_EQ(
    shiftout_intent.intent,
    multi_purpose_mpc_ros::mpcc_execution_contract::ControlIntent::ShiftOut);
  EXPECT_EQ(
    shiftout_intent.reason,
    orchestrator::CanonicalControlIntentReason::
    LateralHoldDynamicWaitShiftOut);

  auto pass = shiftout;
  pass.dynamic_wait_origin_phase = orchestrator::Phase::Pass;
  const auto pass_authority = orchestrator::resolve_authority(pass);
  const auto pass_intent = orchestrator::resolve_canonical_control_intent(
    pass, pass_authority);
  EXPECT_TRUE(pass_intent.valid);
  EXPECT_EQ(
    pass_intent.intent,
    multi_purpose_mpc_ros::mpcc_execution_contract::ControlIntent::Pass);
  EXPECT_EQ(
    pass_intent.reason,
    orchestrator::CanonicalControlIntentReason::LateralHoldDynamicWaitPass);
}

TEST(OvertakeExecutionOrchestrator, PreservesRollingDynamicWaitOriginIntent)
{
  orchestrator::AuthorityRequest shiftout;
  shiftout.mission_generation = 3U;
  shiftout.target_id = "d2";
  shiftout.dynamic_wait_active = true;
  shiftout.dynamic_wait_forward_prefix_active = true;
  shiftout.dynamic_wait_lateral_authority_active = true;
  shiftout.dynamic_wait_origin_phase = orchestrator::Phase::ShiftOut;
  shiftout.line_active = true;
  const auto shiftout_authority = orchestrator::resolve_authority(shiftout);
  const auto shiftout_intent = orchestrator::resolve_canonical_control_intent(
    shiftout, shiftout_authority);

  EXPECT_TRUE(shiftout_intent.valid);
  EXPECT_EQ(
    shiftout_intent.intent,
    multi_purpose_mpc_ros::mpcc_execution_contract::ControlIntent::ShiftOut);
  EXPECT_EQ(
    shiftout_intent.reason,
    orchestrator::CanonicalControlIntentReason::RollingDynamicWaitShiftOut);

  auto pass = shiftout;
  pass.dynamic_wait_origin_phase = orchestrator::Phase::Pass;
  const auto pass_authority = orchestrator::resolve_authority(pass);
  const auto pass_intent = orchestrator::resolve_canonical_control_intent(
    pass, pass_authority);
  EXPECT_TRUE(pass_intent.valid);
  EXPECT_EQ(
    pass_intent.intent,
    multi_purpose_mpc_ros::mpcc_execution_contract::ControlIntent::Pass);
  EXPECT_EQ(
    pass_intent.reason,
    orchestrator::CanonicalControlIntentReason::RollingDynamicWaitPass);
}

TEST(OvertakeExecutionOrchestrator, RejectsDynamicWaitWithoutExecutableProvenance)
{
  orchestrator::AuthorityRequest missing_lateral;
  missing_lateral.dynamic_wait_active = true;
  const auto missing_authority = orchestrator::resolve_authority(missing_lateral);
  const auto missing_intent = orchestrator::resolve_canonical_control_intent(
    missing_lateral, missing_authority);
  EXPECT_FALSE(missing_intent.valid);
  EXPECT_EQ(
    missing_intent.reason,
    orchestrator::CanonicalControlIntentReason::DynamicWaitWithoutLateralAuthority);

  orchestrator::AuthorityRequest missing_identity;
  missing_identity.dynamic_wait_active = true;
  missing_identity.dynamic_wait_lateral_authority_active = true;
  missing_identity.dynamic_wait_origin_phase = orchestrator::Phase::ShiftOut;
  missing_identity.line_active = true;
  const auto missing_identity_authority =
    orchestrator::resolve_authority(missing_identity);
  const auto missing_identity_intent =
    orchestrator::resolve_canonical_control_intent(
    missing_identity, missing_identity_authority);
  EXPECT_FALSE(missing_identity_intent.valid);
  EXPECT_EQ(
    missing_identity_intent.reason,
    orchestrator::CanonicalControlIntentReason::
    DynamicWaitWithoutMissionIdentity);

  orchestrator::AuthorityRequest unsupported;
  unsupported.mission_generation = 3U;
  unsupported.target_id = "d2";
  unsupported.dynamic_wait_active = true;
  unsupported.dynamic_wait_forward_prefix_active = true;
  unsupported.dynamic_wait_lateral_authority_active = true;
  unsupported.dynamic_wait_origin_phase = orchestrator::Phase::Idle;
  unsupported.line_active = true;
  const auto unsupported_authority = orchestrator::resolve_authority(unsupported);
  const auto unsupported_intent = orchestrator::resolve_canonical_control_intent(
    unsupported, unsupported_authority);
  EXPECT_FALSE(unsupported_intent.valid);
  EXPECT_EQ(
    unsupported_intent.reason,
    orchestrator::CanonicalControlIntentReason::UnsupportedDynamicWaitOrigin);
}

TEST(OvertakeExecutionOrchestrator, ResolvesSafetyAndRacingCanonicalIntents)
{
  orchestrator::AuthorityRequest safety;
  safety.behavior = orchestrator::Behavior::SafetyBrake;
  const auto safety_authority = orchestrator::resolve_authority(safety);
  const auto safety_intent = orchestrator::resolve_canonical_control_intent(
    safety, safety_authority);
  EXPECT_TRUE(safety_intent.valid);
  EXPECT_EQ(
    safety_intent.intent,
    multi_purpose_mpc_ros::mpcc_execution_contract::ControlIntent::Stop);

  orchestrator::AuthorityRequest track;
  track.race_session_active = false;
  const auto track_authority = orchestrator::resolve_authority(track);
  const auto track_intent = orchestrator::resolve_canonical_control_intent(
    track, track_authority);
  EXPECT_TRUE(track_intent.valid);
  EXPECT_EQ(
    track_intent.intent,
    multi_purpose_mpc_ros::mpcc_execution_contract::ControlIntent::Track);

  track.race_session_active = true;
  const auto cruise_authority = orchestrator::resolve_authority(track);
  const auto cruise_intent = orchestrator::resolve_canonical_control_intent(
    track, cruise_authority);
  EXPECT_TRUE(cruise_intent.valid);
  EXPECT_EQ(
    cruise_intent.intent,
    multi_purpose_mpc_ros::mpcc_execution_contract::ControlIntent::Cruise);
}

TEST(OvertakeExecutionOrchestrator, FollowIntentRequiresCurrentFrontEvidence)
{
  orchestrator::AuthorityRequest request;
  request.race_session_active = true;
  request.target_id = "d2";
  request.behavior = orchestrator::Behavior::Follow;

  const auto authority = orchestrator::resolve_authority(request);
  ASSERT_EQ(authority.action, orchestrator::Action::Follow);

  const auto without_front =
    orchestrator::resolve_canonical_control_intent(request, authority);
  EXPECT_TRUE(without_front.valid);
  EXPECT_EQ(without_front.intent, contract::ControlIntent::Cruise);
  EXPECT_EQ(
    without_front.reason,
    orchestrator::CanonicalControlIntentReason::
    FollowWithoutCoherentFrontObservation);

  request.coherent_follow_front_observation = true;
  const auto with_front =
    orchestrator::resolve_canonical_control_intent(request, authority);
  EXPECT_TRUE(with_front.valid);
  EXPECT_EQ(with_front.intent, contract::ControlIntent::Follow);
  EXPECT_EQ(
    with_front.reason,
    orchestrator::CanonicalControlIntentReason::ResolvedAction);
}

TEST(OvertakeExecutionOrchestrator, NormalizesContradictorySpeedFloor)
{
  const auto result = orchestrator::normalize_speed_window(5.78, 8.0, 5.80, true);
  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.floor_adjusted);
  EXPECT_DOUBLE_EQ(result.requested_floor_mps, 5.80);
  EXPECT_DOUBLE_EQ(result.floor_mps, 5.78);

  orchestrator::AuthorityRequest request;
  request.phase = orchestrator::Phase::Pass;
  request.target_id = "d2";
  request.pass_speed_floor_active = true;
  request.speed_reference_mps = result.reference_mps;
  request.speed_limit_mps = result.limit_mps;
  request.speed_floor_mps = result.floor_mps;
  EXPECT_EQ(
    orchestrator::resolve_authority(request).conflicts &
    orchestrator::InvalidSpeedWindow,
    0U);
}

TEST(OvertakeExecutionOrchestrator, AlignsAdmissionAndRuntimeWallClearance)
{
  const auto result = orchestrator::resolve_wall_clearance_contract(
    0.10, 0.15, true, 0.10);
  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.required_clearance_m, 0.20);

  orchestrator::AuthorityRequest request;
  request.phase = orchestrator::Phase::Pass;
  request.target_id = "d2";
  request.line_active = true;
  request.wall_contract_required_clearance_m = result.required_clearance_m;
  request.wall_contract_minimum_path_clearance_m = 0.19;
  EXPECT_NE(
    orchestrator::resolve_authority(request).conflicts &
    orchestrator::WallContractShortfall,
    0U);
}

TEST(OvertakeExecutionOrchestrator, ShiftOutKeepsLongitudinalOwnership)
{
  orchestrator::AuthorityRequest request;
  request.episode_id = 3U;
  request.mission_generation = 1U;
  request.target_id = "d2";
  request.phase = orchestrator::Phase::ShiftOut;
  request.behavior = orchestrator::Behavior::Overtake;
  request.line_active = true;
  request.shiftout_speed_contract_expected = true;
  request.shiftout_speed_contract_active = true;
  request.shiftout_speed_contract_reference_mps = 6.10;
  request.speed_reference_mps = 6.10;

  const auto result = orchestrator::resolve_authority(request);
  EXPECT_EQ(result.longitudinal_owner, orchestrator::LongitudinalOwner::OvertakeLine);
  EXPECT_TRUE(result.apply_overtake_speed_reference);
  EXPECT_EQ(result.conflicts, orchestrator::NoConflict);
}

TEST(OvertakeExecutionOrchestrator, ExposesMissingShiftOutSpeedContract)
{
  orchestrator::AuthorityRequest request;
  request.episode_id = 3U;
  request.mission_generation = 1U;
  request.target_id = "d2";
  request.phase = orchestrator::Phase::ShiftOut;
  request.behavior = orchestrator::Behavior::Overtake;
  request.line_active = true;
  request.shiftout_speed_contract_expected = true;

  const auto result = orchestrator::resolve_authority(request);
  EXPECT_EQ(result.longitudinal_owner, orchestrator::LongitudinalOwner::OvertakeLine);
  EXPECT_NE(
    result.conflicts & orchestrator::ShiftOutWithoutSpeedContract, 0U);
  EXPECT_NE(
    orchestrator::format_conflicts(result.conflicts).find(
      "shiftout-without-speed-contract"),
    std::string::npos);
}

TEST(OvertakeExecutionOrchestrator, RejectsEmptyLateralBoundIntersection)
{
  const auto result = orchestrator::resolve_lateral_bound_contract(0.25, 0.10);
  EXPECT_TRUE(result.valid);
  EXPECT_FALSE(result.feasible);
  EXPECT_EQ(
    result.reason,
    orchestrator::LateralBoundContractReason::EmptyIntersection);
}

TEST(OvertakeExecutionOrchestrator, AcceptsFiniteLateralBoundIntersection)
{
  const auto result = orchestrator::resolve_lateral_bound_contract(-0.4, 0.2);
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.feasible);
  EXPECT_EQ(result.reason, orchestrator::LateralBoundContractReason::None);
  EXPECT_DOUBLE_EQ(result.lower_m, -0.4);
  EXPECT_DOUBLE_EQ(result.upper_m, 0.2);
}

TEST(OvertakeExecutionOrchestrator, AdmitsFreshRuntimeReplacementContract)
{
  orchestrator::RuntimeReplacementContractRequest request;
  request.candidate_feasible = true;
  request.now_sec = 10.0;
  request.dynamic_valid_until_sec = 10.5;
  request.target_clearance_checked = true;
  request.minimum_target_clearance_m = 0.05;
  request.minimum_path_wall_clearance_m = 0.40;
  request.required_path_wall_clearance_m = 0.30;

  const auto result = orchestrator::resolve_runtime_replacement_contract(request);
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.admitted);
  EXPECT_EQ(result.reason, orchestrator::RuntimeReplacementRejectReason::None);
}

TEST(OvertakeExecutionOrchestrator, RejectsExpiredRuntimeReplacement)
{
  orchestrator::RuntimeReplacementContractRequest request;
  request.candidate_feasible = true;
  request.now_sec = 10.6;
  request.dynamic_valid_until_sec = 10.5;
  request.target_clearance_checked = true;
  request.minimum_target_clearance_m = 0.05;
  request.minimum_path_wall_clearance_m = 0.40;
  request.required_path_wall_clearance_m = 0.30;

  const auto result = orchestrator::resolve_runtime_replacement_contract(request);
  EXPECT_TRUE(result.valid);
  EXPECT_FALSE(result.admitted);
  EXPECT_EQ(
    result.reason,
    orchestrator::RuntimeReplacementRejectReason::PredictionExpired);
}

TEST(OvertakeExecutionOrchestrator, RejectsUncheckedOrOverlappingRuntimeTarget)
{
  orchestrator::RuntimeReplacementContractRequest request;
  request.candidate_feasible = true;
  request.now_sec = 10.0;
  request.dynamic_valid_until_sec = 10.5;
  request.minimum_target_clearance_m = 0.05;
  request.minimum_path_wall_clearance_m = 0.40;
  request.required_path_wall_clearance_m = 0.30;

  EXPECT_EQ(
    orchestrator::resolve_runtime_replacement_contract(request).reason,
    orchestrator::RuntimeReplacementRejectReason::TargetClearanceUnchecked);
  request.target_clearance_checked = true;
  request.minimum_target_clearance_m = -0.01;
  EXPECT_EQ(
    orchestrator::resolve_runtime_replacement_contract(request).reason,
    orchestrator::RuntimeReplacementRejectReason::TargetOverlap);
}

TEST(OvertakeExecutionOrchestrator, RejectsRuntimeWallContractShortfall)
{
  orchestrator::RuntimeReplacementContractRequest request;
  request.candidate_feasible = true;
  request.now_sec = 10.0;
  request.dynamic_valid_until_sec = 10.5;
  request.target_clearance_checked = true;
  request.minimum_target_clearance_m = 0.05;
  request.minimum_path_wall_clearance_m = 0.29;
  request.required_path_wall_clearance_m = 0.30;

  const auto result = orchestrator::resolve_runtime_replacement_contract(request);
  EXPECT_TRUE(result.valid);
  EXPECT_FALSE(result.admitted);
  EXPECT_EQ(
    result.reason,
    orchestrator::RuntimeReplacementRejectReason::WallContractShortfall);
}

TEST(OvertakeExecutionOrchestrator, FindsFutureCorridorPinch)
{
  const auto metrics = orchestrator::analyze_corridor(
    std::vector<double>{-2.0, -1.0, -0.2},
    std::vector<double>{2.0, 1.0, 0.3},
    std::vector<double>{1.0, 4.0, 8.0});
  ASSERT_TRUE(metrics.valid);
  EXPECT_EQ(metrics.sample_count, 3U);
  EXPECT_EQ(metrics.minimum_width_index, 2U);
  EXPECT_DOUBLE_EQ(metrics.minimum_width_m, 0.5);
  EXPECT_DOUBLE_EQ(metrics.minimum_width_distance_m, 8.0);
}

TEST(OvertakeExecutionOrchestrator, EmitsOnlyOnAuthorityChangeOrHeartbeat)
{
  orchestrator::AuthorityTrace trace;
  trace.request.episode_id = 1U;
  trace.request.target_id = "d2";
  trace.request.phase = orchestrator::Phase::ShiftOut;
  trace.request.behavior = orchestrator::Behavior::Overtake;
  trace.request.line_active = true;
  trace.resolution = orchestrator::resolve_authority(trace.request);
  trace.ego_speed_mps = 6.0;
  trace.waypoint_id = 10;

  orchestrator::ChangeAwareAuthorityTraceEmitter emitter;
  EXPECT_TRUE(emitter.update(trace, 1.0).emit);
  trace.ego_speed_mps = 6.2;
  trace.waypoint_id = 11;
  EXPECT_FALSE(emitter.update(trace, 1.1).emit);
  trace.request.dynamic_wait_active = true;
  trace.request.dynamic_wait_forward_prefix_active = true;
  trace.resolution = orchestrator::resolve_authority(trace.request);
  EXPECT_TRUE(emitter.update(trace, 1.2).emit);
  EXPECT_FALSE(emitter.update(trace, 5.9).emit);
  EXPECT_TRUE(emitter.update(trace, 6.2).emit);
}

TEST(OvertakeExecutionOrchestrator, SummarizesWholeEpisode)
{
  orchestrator::EpisodeAccumulator accumulator;
  accumulator.begin(orchestrator::EpisodeStart{
    5U, "d2", -1, 10.0, 42, "entry"});

  orchestrator::EpisodeSample shift;
  shift.episode_id = 5U;
  shift.mission_generation = 1U;
  shift.target_id = "d2";
  shift.phase = orchestrator::Phase::ShiftOut;
  shift.action = orchestrator::Action::ShiftOut;
  shift.lateral_owner = orchestrator::LateralOwner::OvertakeLine;
  shift.longitudinal_owner = orchestrator::LongitudinalOwner::OvertakeLine;
  shift.now_sec = 10.1;
  shift.ego_speed_mps = 7.0;
  shift.constrained_corridor = orchestrator::analyze_corridor(
    {-1.0, -0.5}, {1.0, 0.5}, {1.0, 2.0});
  shift.wall_corridor = orchestrator::analyze_corridor(
    {-2.0, -2.0}, {2.0, 2.0}, {1.0, 2.0});
  shift.maximum_required_lateral_accel_mps2 = 4.0;
  accumulator.observe(shift);

  auto wait = shift;
  wait.mission_generation = 3U;
  wait.phase = orchestrator::Phase::FollowPrepare;
  wait.action = orchestrator::Action::DynamicWait;
  wait.lateral_owner = orchestrator::LateralOwner::DynamicWaitPrefix;
  wait.dynamic_wait_active = true;
  wait.ego_speed_mps = 5.5;
  wait.maximum_required_lateral_accel_mps2 = 5.0;
  accumulator.observe(wait);
  accumulator.observe(wait);

  const auto summary = accumulator.finish(
    12.5, "FollowPrepare", "target-wall-conflict", 57);
  ASSERT_TRUE(summary.has_value());
  EXPECT_DOUBLE_EQ(summary->elapsed_sec, 2.5);
  EXPECT_DOUBLE_EQ(summary->minimum_speed_mps, 5.5);
  EXPECT_DOUBLE_EQ(summary->minimum_constrained_corridor_width_m, 1.0);
  EXPECT_EQ(summary->maximum_mission_generation, 3U);
  EXPECT_EQ(summary->authority_change_count, 2U);
  EXPECT_EQ(summary->dynamic_wait_entry_count, 1U);
  const auto message = orchestrator::format_episode_summary(summary.value());
  EXPECT_NE(message.find("episode=5"), std::string::npos);
  EXPECT_NE(message.find("reason=\"target-wall-conflict\""), std::string::npos);
  EXPECT_FALSE(accumulator.active());
}

TEST(OvertakeExecutionOrchestrator, ResolvesFinalControlSourceByOutputPrecedence)
{
  orchestrator::FinalControlSourceRequest request;
  request.solver_fallback_active = true;
  EXPECT_EQ(
    orchestrator::resolve_final_control_source(request),
    orchestrator::FinalControlSource::SolverFallback);

  request.executed_solution_wall_hold_active = true;
  EXPECT_EQ(
    orchestrator::resolve_final_control_source(request),
    orchestrator::FinalControlSource::ExecutedSolutionWallHold);

  request.stuck_recovery_active = true;
  EXPECT_EQ(
    orchestrator::resolve_final_control_source(request),
    orchestrator::FinalControlSource::StuckRecovery);

  request.failsafe_active = true;
  EXPECT_EQ(
    orchestrator::resolve_final_control_source(request),
    orchestrator::FinalControlSource::Failsafe);
}

TEST(OvertakeExecutionOrchestrator, JoinsAuthorityAndPublishedCommandByDecisionId)
{
  orchestrator::AuthorityTrace authority;
  authority.request.decision_id = 42U;
  authority.request.episode_id = 3U;
  authority.request.mission_generation = 2U;
  authority.request.target_id = "d2";
  authority.request.pass_side_sign = -1;
  authority.request.phase = orchestrator::Phase::Pass;
  authority.request.behavior = orchestrator::Behavior::Overtake;
  authority.request.line_active = true;
  authority.request.path_source_hint = orchestrator::PathSource::RecedingDp;
  authority.request.path_age_sec = 0.12;
  authority.request.front_distance_m = 6.5;
  authority.request.dynamic_front_safety_distance_m = 4.2;
  authority.request.protected_front_distance_m = 4.5;
  authority.request.closing_speed_reference_mps = 1.1;
  authority.request.transition_reason = "keep phase";
  authority.request.blocking_reason = "none";
  authority.resolution = orchestrator::resolve_authority(authority.request);

  orchestrator::FinalControlTrace trace;
  trace.decision_id = 42U;
  trace.authority = authority;
  trace.control_source = orchestrator::FinalControlSource::ControlDisabled;
  trace.published = true;
  trace.actual_speed_mps = 6.0;
  trace.target_speed_mps = 7.0;
  trace.acceleration_mps2 = 1.0;
  trace.raw_steering_rad = 0.1;
  trace.published_steering_rad = 0.15;
  trace.solver_reason = "extended-mpcc-solved";
  trace.output_reason = "normal-control-published";
  trace.execution_contract =
    multi_purpose_mpc_ros::mpcc_execution_contract::
    resolve_final_control_decision(
    multi_purpose_mpc_ros::mpcc_execution_contract::
    FinalControlDecisionRequest{
      42U,
      multi_purpose_mpc_ros::mpcc_execution_contract::
      FinalAuthorityClass::ControlDisabled,
      "control-disabled", std::nullopt, std::nullopt});

  orchestrator::ChangeAwareFinalControlTraceEmitter emitter;
  const auto first = emitter.update(trace, 1.0);
  EXPECT_TRUE(first.emit);
  EXPECT_FALSE(first.warning);
  EXPECT_NE(first.message.find("decision=42"), std::string::npos);
  EXPECT_NE(first.message.find("path_source=receding-dp"), std::string::npos);
  EXPECT_NE(
    first.message.find("canonical_intent=pass/resolved-action"),
    std::string::npos);
  EXPECT_NE(
    first.message.find("front=6.50/safety=4.20/protected=4.50m"),
    std::string::npos);
  EXPECT_NE(first.message.find("closing_ref=1.10m/s"), std::string::npos);
  EXPECT_NE(first.message.find("control_source=control-disabled"), std::string::npos);
  EXPECT_NE(first.message.find("identity=complete"), std::string::npos);
  EXPECT_NE(first.message.find("contract_join=1"), std::string::npos);
  trace.decision_id = 43U;
  trace.execution_contract =
    multi_purpose_mpc_ros::mpcc_execution_contract::
    resolve_final_control_decision(
    multi_purpose_mpc_ros::mpcc_execution_contract::
    FinalControlDecisionRequest{
      43U,
      multi_purpose_mpc_ros::mpcc_execution_contract::
      FinalAuthorityClass::ControlDisabled,
      "control-disabled", std::nullopt, std::nullopt});
  EXPECT_FALSE(emitter.update(trace, 1.1).emit);
}

TEST(OvertakeExecutionOrchestrator, AggregatesNormalLongitudinalOwnerChatter)
{
  orchestrator::AuthorityTrace authority;
  authority.request.episode_id = 8U;
  authority.request.target_id = "d2";
  authority.request.phase = orchestrator::Phase::Pass;
  authority.request.behavior = orchestrator::Behavior::Overtake;
  authority.request.line_active = true;
  authority.resolution = orchestrator::resolve_authority(authority.request);

  orchestrator::FinalControlTrace trace;
  trace.authority = authority;
  trace.control_source = orchestrator::FinalControlSource::MpcSolution;
  trace.published = true;

  orchestrator::ChangeAwareFinalControlTraceEmitter emitter;
  EXPECT_TRUE(emitter.update(trace, 1.0, 1.0).emit);

  trace.authority->request.follow_cap_active = true;
  trace.authority->resolution =
    orchestrator::resolve_authority(trace.authority->request);
  EXPECT_FALSE(emitter.update(trace, 1.1, 1.0).emit);

  const auto heartbeat = emitter.update(trace, 2.1, 1.0);
  EXPECT_TRUE(heartbeat.emit);
  EXPECT_EQ(heartbeat.suppressed_normal_change_count, 1U);
  EXPECT_NE(
    heartbeat.message.find("suppressed_normal_changes=1"), std::string::npos);
}

TEST(OvertakeExecutionOrchestrator, ClassifiesCurrentFootprintWallRisk)
{
  orchestrator::WallHandoffProbe probe;
  EXPECT_EQ(
    orchestrator::classify_wall_risk(probe),
    orchestrator::WallRiskState::Unknown);

  probe.current_wall_valid = true;
  probe.current_footprint_clear = true;
  probe.current_wall_distance_m = 0.35;
  probe.required_wall_clearance_m = 0.20;
  EXPECT_EQ(
    orchestrator::classify_wall_risk(probe),
    orchestrator::WallRiskState::Clear);

  probe.current_wall_distance_m = 0.20;
  EXPECT_EQ(
    orchestrator::classify_wall_risk(probe),
    orchestrator::WallRiskState::Near);

  probe.current_contact_count = 1U;
  probe.current_footprint_clear = false;
  EXPECT_EQ(
    orchestrator::classify_wall_risk(probe),
    orchestrator::WallRiskState::Contact);
}

TEST(OvertakeExecutionOrchestrator, KeepsAttemptAcrossSameTargetRequestGap)
{
  orchestrator::DynamicEscapeAttemptTracker tracker;
  orchestrator::DynamicEscapeAttemptRequest request;
  request.planner_requested = true;
  request.target_relevant = true;
  request.target_id = "d2";
  request.now_sec = 10.0;

  const auto started = tracker.update(request);
  ASSERT_TRUE(started.started);
  ASSERT_TRUE(started.active);
  ASSERT_EQ(started.attempt_id, 1U);

  request.planner_requested = false;
  request.now_sec = 10.025;
  const auto held = tracker.update(request);
  EXPECT_TRUE(held.active);
  EXPECT_TRUE(held.held_without_request);
  EXPECT_TRUE(held.planning_requested);
  EXPECT_TRUE(held.continuation_requested);
  EXPECT_FALSE(held.started);
  EXPECT_FALSE(held.released);
  EXPECT_EQ(held.attempt_id, started.attempt_id);
  EXPECT_EQ(held.request_gap_cycles, 1);
  EXPECT_EQ(
    held.reason,
    orchestrator::DynamicEscapeAttemptReason::ContinuationRequested);

  request.planner_requested = true;
  request.now_sec = 10.050;
  const auto resumed = tracker.update(request);
  EXPECT_TRUE(resumed.active);
  EXPECT_FALSE(resumed.started);
  EXPECT_EQ(resumed.attempt_id, started.attempt_id);
  EXPECT_EQ(resumed.planner_request_cycles, 2);
}

TEST(OvertakeExecutionOrchestrator, ReleasesAttemptOnlyAfterTargetLossGrace)
{
  orchestrator::DynamicEscapeAttemptTracker tracker;
  orchestrator::DynamicEscapeAttemptRequest request;
  request.planner_requested = true;
  request.target_relevant = true;
  request.target_id = "d2";
  request.now_sec = 20.0;
  request.target_loss_grace_sec = 0.50;
  const auto started = tracker.update(request);
  ASSERT_TRUE(started.active);

  request.planner_requested = false;
  request.target_relevant = false;
  request.target_id.clear();
  request.now_sec = 20.40;
  const auto grace = tracker.update(request);
  EXPECT_TRUE(grace.active);
  EXPECT_FALSE(grace.released);
  EXPECT_EQ(grace.attempt_id, started.attempt_id);
  EXPECT_NEAR(grace.target_loss_age_sec, 0.40, 1e-9);
  EXPECT_EQ(
    grace.reason,
    orchestrator::DynamicEscapeAttemptReason::TargetLossGrace);

  request.now_sec = 20.51;
  const auto released = tracker.update(request);
  EXPECT_FALSE(released.active);
  EXPECT_TRUE(released.released);
  EXPECT_EQ(released.attempt_id, started.attempt_id);
  EXPECT_EQ(released.target_id, "d2");
  EXPECT_EQ(
    released.reason,
    orchestrator::DynamicEscapeAttemptReason::TargetLost);
  EXPECT_FALSE(tracker.active());
}

TEST(OvertakeExecutionOrchestrator, RetargetsAttemptAtomically)
{
  orchestrator::DynamicEscapeAttemptTracker tracker;
  orchestrator::DynamicEscapeAttemptRequest request;
  request.planner_requested = true;
  request.target_relevant = true;
  request.target_id = "d2";
  request.now_sec = 30.0;
  const auto first = tracker.update(request);
  ASSERT_EQ(first.attempt_id, 1U);

  request.target_id = "d3";
  request.now_sec = 30.1;
  const auto retargeted = tracker.update(request);
  EXPECT_TRUE(retargeted.active);
  EXPECT_TRUE(retargeted.started);
  EXPECT_TRUE(retargeted.released);
  EXPECT_TRUE(retargeted.retargeted);
  EXPECT_EQ(retargeted.previous_target_id, "d2");
  EXPECT_EQ(retargeted.target_id, "d3");
  EXPECT_EQ(retargeted.attempt_id, 2U);
  EXPECT_EQ(
    retargeted.reason,
    orchestrator::DynamicEscapeAttemptReason::TargetChanged);
}

TEST(OvertakeExecutionOrchestrator, FormatsDynamicEscapeAttemptLifecycle)
{
  orchestrator::DynamicEscapeAttemptRequest request;
  request.planner_requested = false;
  request.target_relevant = true;
  request.target_id = "d2";
  request.target_loss_grace_sec = 0.50;
  orchestrator::DynamicEscapeAttemptResolution resolution;
  resolution.attempt_id = 41U;
  resolution.active = true;
  resolution.held_without_request = true;
  resolution.planning_requested = true;
  resolution.continuation_requested = true;
  resolution.target_id = "d2";
  resolution.lifetime_cycles = 16;
  resolution.planner_request_cycles = 6;
  resolution.request_gap_cycles = 10;
  resolution.target_loss_grace_sec = 0.50;
  resolution.reason =
    orchestrator::DynamicEscapeAttemptReason::ContinuationRequested;

  const auto message = orchestrator::format_dynamic_escape_attempt_trace(
    request, resolution, 38);
  EXPECT_NE(message.find("event=heartbeat"), std::string::npos);
  EXPECT_NE(message.find("reason=continuation-requested"), std::string::npos);
  EXPECT_NE(message.find("effective_planning=1"), std::string::npos);
  EXPECT_NE(message.find("continuation=1"), std::string::npos);
  EXPECT_NE(message.find("attempt=41"), std::string::npos);
  EXPECT_NE(message.find("target=d2"), std::string::npos);
  EXPECT_NE(message.find("cycles=16/request=6/gap=10"), std::string::npos);
  EXPECT_NE(message.find("wp_id=38"), std::string::npos);
}

TEST(OvertakeExecutionOrchestrator, MonitorsWallAfterDynamicEscapeHandoff)
{
  orchestrator::ChangeAwareWallHandoffTraceEmitter emitter;
  orchestrator::WallHandoffProbe probe;
  probe.current_wall_valid = true;
  probe.current_footprint_clear = true;
  probe.current_wall_distance_m = 1.0;
  probe.required_wall_clearance_m = 0.20;

  EXPECT_FALSE(emitter.update(probe, 1.0).emit);

  probe.dynamic_escape_active = true;
  probe.action = orchestrator::Action::DynamicEscape;
  probe.lateral_owner = orchestrator::LateralOwner::DynamicObstacleEscape;
  probe.path_source = orchestrator::PathSource::DynamicObstacleEscape;
  const auto entry = emitter.update(probe, 2.0);
  EXPECT_TRUE(entry.emit);
  EXPECT_TRUE(entry.monitor_active);
  EXPECT_TRUE(entry.source_changed);

  probe.dynamic_escape_active = false;
  probe.action = orchestrator::Action::Follow;
  probe.lateral_owner = orchestrator::LateralOwner::RacingLine;
  probe.path_source = orchestrator::PathSource::RacingLine;
  const auto exit = emitter.update(probe, 2.1);
  EXPECT_TRUE(exit.emit);
  EXPECT_TRUE(exit.monitor_active);

  probe.current_wall_distance_m = 0.15;
  const auto near = emitter.update(probe, 2.2);
  EXPECT_TRUE(near.emit);
  EXPECT_TRUE(near.warning);
  EXPECT_EQ(near.risk, orchestrator::WallRiskState::Near);
  EXPECT_FALSE(emitter.update(probe, 2.3).emit);
  EXPECT_TRUE(emitter.update(probe, 2.8).emit);
  EXPECT_FALSE(emitter.update(probe, 4.1).emit);
}

TEST(OvertakeExecutionOrchestrator, FormatsWallHandoffDecisionEvidence)
{
  orchestrator::WallHandoffProbe probe;
  probe.decision_id = 17U;
  probe.action = orchestrator::Action::DynamicEscape;
  probe.lateral_owner = orchestrator::LateralOwner::DynamicObstacleEscape;
  probe.path_source = orchestrator::PathSource::DynamicObstacleEscape;
  probe.current_wall_valid = true;
  probe.current_footprint_clear = true;
  probe.current_wall_region = "Right";
  probe.current_wall_distance_m = 0.18;
  probe.required_wall_clearance_m = 0.20;
  probe.published_steering_rad = 0.15;
  probe.previous_published_steering_rad = 0.10;

  orchestrator::WallHandoffEvent event;
  event.emit = true;
  event.monitor_active = true;
  event.risk = orchestrator::WallRiskState::Near;
  event.trigger = "wall-risk";

  orchestrator::PredictedPathWallMetrics path;
  path.available = true;
  path.valid = true;
  path.sample_count = 8U;
  path.minimum_index = 3U;
  path.minimum_wall_region = "Right";
  path.minimum_wall_distance_m = 0.12;
  path.minimum_wall_path_distance_m = 2.5;

  const auto message = orchestrator::format_wall_handoff_trace(probe, event, path);
  EXPECT_NE(message.find("decision=17"), std::string::npos);
  EXPECT_NE(message.find("trigger=wall-risk"), std::string::npos);
  EXPECT_NE(message.find("path_wall=Right/0.12m@3/2.50m"), std::string::npos);
  EXPECT_NE(message.find("delta=0.05"), std::string::npos);
}

TEST(OvertakeExecutionOrchestrator, PublishesOnlyWallSafeExecutedSolution)
{
  const auto result = orchestrator::resolve_executed_solution_wall_action(
    orchestrator::ExecutedSolutionWallRequest{
      true, true, false, orchestrator::Phase::ShiftOut, 0.0});

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.publish_solution);
  EXPECT_EQ(
    result.action, orchestrator::ExecutedSolutionWallAction::Publish);
}

TEST(OvertakeExecutionOrchestrator, RollsBackUntravelledUnsafeEntry)
{
  const auto result = orchestrator::resolve_executed_solution_wall_action(
    orchestrator::ExecutedSolutionWallRequest{
      true, false, false, orchestrator::Phase::ShiftOut, 0.01});

  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.publish_solution);
  EXPECT_EQ(
    result.action, orchestrator::ExecutedSolutionWallAction::EntryRollback);
}

TEST(OvertakeExecutionOrchestrator, RollsBackUnpublishedEntryDespitePoseProgress)
{
  const auto result = orchestrator::resolve_executed_solution_wall_action(
    orchestrator::ExecutedSolutionWallRequest{
      true, false, false, orchestrator::Phase::ShiftOut, 1.5});

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(
    result.action, orchestrator::ExecutedSolutionWallAction::EntryRollback);
}

TEST(OvertakeExecutionOrchestrator, KeepsLateralOwnershipAfterPublishedEntry)
{
  const auto shiftout = orchestrator::resolve_executed_solution_wall_action(
    orchestrator::ExecutedSolutionWallRequest{
      true, false, true, orchestrator::Phase::ShiftOut, 0.06});
  const auto pass = orchestrator::resolve_executed_solution_wall_action(
    orchestrator::ExecutedSolutionWallRequest{
      true, false, true, orchestrator::Phase::Pass, 2.0});
  const auto returning = orchestrator::resolve_executed_solution_wall_action(
    orchestrator::ExecutedSolutionWallRequest{
      true, false, true, orchestrator::Phase::Return, 1.0});

  ASSERT_TRUE(shiftout.valid);
  ASSERT_TRUE(pass.valid);
  ASSERT_TRUE(returning.valid);
  EXPECT_EQ(
    shiftout.action,
    orchestrator::ExecutedSolutionWallAction::DynamicReplan);
  EXPECT_EQ(
    pass.action,
    orchestrator::ExecutedSolutionWallAction::DynamicReplan);
  EXPECT_EQ(
    returning.action,
    orchestrator::ExecutedSolutionWallAction::RecoveryReplan);
}

}  // namespace
