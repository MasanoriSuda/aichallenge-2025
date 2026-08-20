#include "multi_purpose_mpc_ros/overtake_execution_orchestrator.hpp"

#include <gtest/gtest.h>

#include <string>
#include <vector>

namespace {

namespace orchestrator =
  multi_purpose_mpc_ros::overtake_execution_orchestrator;

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

  request.solver_crawl_active = true;
  EXPECT_EQ(
    orchestrator::resolve_final_control_source(request),
    orchestrator::FinalControlSource::SolverCrawl);

  request.solver_bounded_continuation_active = true;
  EXPECT_EQ(
    orchestrator::resolve_final_control_source(request),
    orchestrator::FinalControlSource::SolverBoundedContinuation);

  request.solver_wall_handoff_hold_active = true;
  EXPECT_EQ(
    orchestrator::resolve_final_control_source(request),
    orchestrator::FinalControlSource::SolverWallHandoffHold);

  request.solver_wall_handoff_hold_active = false;
  request.overtake_wall_admission_hold_active = true;
  EXPECT_EQ(
    orchestrator::resolve_final_control_source(request),
    orchestrator::FinalControlSource::OvertakeWallAdmissionHold);

  request.low_speed_wall_stop_active = true;
  EXPECT_EQ(
    orchestrator::resolve_final_control_source(request),
    orchestrator::FinalControlSource::LowSpeedWallStop);

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
  authority.request.transition_reason = "keep phase";
  authority.request.blocking_reason = "none";
  authority.resolution = orchestrator::resolve_authority(authority.request);

  orchestrator::FinalControlTrace trace;
  trace.decision_id = 42U;
  trace.authority = authority;
  trace.control_source = orchestrator::FinalControlSource::MpcSolution;
  trace.published = true;
  trace.actual_speed_mps = 6.0;
  trace.target_speed_mps = 7.0;
  trace.acceleration_mps2 = 1.0;
  trace.raw_steering_rad = 0.1;
  trace.published_steering_rad = 0.15;
  trace.solver_reason = "extended-mpcc-solved";
  trace.output_reason = "normal-control-published";

  orchestrator::ChangeAwareFinalControlTraceEmitter emitter;
  const auto first = emitter.update(trace, 1.0);
  EXPECT_TRUE(first.emit);
  EXPECT_FALSE(first.warning);
  EXPECT_NE(first.message.find("decision=42"), std::string::npos);
  EXPECT_NE(first.message.find("path_source=receding-dp"), std::string::npos);
  EXPECT_NE(first.message.find("control_source=mpc-solution"), std::string::npos);
  trace.decision_id = 43U;
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

TEST(OvertakeExecutionOrchestrator, RejectsUnsafeRecoveredWallHandoff)
{
  orchestrator::WallPathAdmissionGate gate;
  orchestrator::WallHandoffAdmissionRequest request;
  request.activation_requested = true;
  request.current_footprint_valid = true;
  request.current_footprint_clear = true;
  request.prediction.available = true;
  request.prediction.valid = true;
  request.prediction.contact = true;
  request.prediction.sample_count = 18U;
  request.prediction.minimum_wall_distance_m = 0.0;
  request.required_wall_clearance_m = 0.40;

  const auto rejected = gate.update(request);
  EXPECT_TRUE(rejected.entered);
  EXPECT_TRUE(rejected.active);
  EXPECT_TRUE(rejected.hold_control);
  EXPECT_FALSE(rejected.stop_required);
  EXPECT_EQ(
    rejected.reason,
    orchestrator::WallHandoffAdmissionReason::PredictedContact);
  EXPECT_TRUE(gate.active());
}

TEST(OvertakeExecutionOrchestrator, RequiresConsecutiveSafeWallHandoffs)
{
  orchestrator::WallPathAdmissionGate gate;
  orchestrator::WallHandoffAdmissionRequest request;
  request.activation_requested = true;
  request.current_footprint_valid = true;
  request.current_footprint_clear = true;
  request.prediction.available = true;
  request.prediction.valid = true;
  request.prediction.sample_count = 18U;
  request.prediction.minimum_wall_distance_m = 0.60;
  request.required_wall_clearance_m = 0.40;
  request.required_consecutive_valid_cycles = 2;

  const auto first = gate.update(request);
  EXPECT_TRUE(first.entered);
  EXPECT_TRUE(first.hold_control);
  EXPECT_EQ(first.consecutive_valid_cycles, 1);
  EXPECT_EQ(
    first.reason, orchestrator::WallHandoffAdmissionReason::Requalifying);

  request.activation_requested = false;
  const auto second = gate.update(request);
  EXPECT_TRUE(second.released);
  EXPECT_FALSE(second.active);
  EXPECT_FALSE(second.hold_control);
  EXPECT_EQ(second.consecutive_valid_cycles, 2);
  EXPECT_EQ(second.reason, orchestrator::WallHandoffAdmissionReason::Accepted);
  EXPECT_FALSE(gate.active());
}

TEST(OvertakeExecutionOrchestrator, StaleWallObservationDoesNotAdvanceRelease)
{
  orchestrator::WallPathAdmissionGate gate;
  orchestrator::WallHandoffAdmissionRequest request;
  request.activation_requested = true;
  request.current_footprint_valid = true;
  request.current_footprint_clear = true;
  request.prediction.available = true;
  request.prediction.valid = true;
  request.prediction.minimum_wall_distance_m = 0.60;
  request.required_wall_clearance_m = 0.40;
  request.required_consecutive_valid_cycles = 2;

  const auto first = gate.update(request);
  ASSERT_EQ(first.consecutive_valid_cycles, 1);

  request.activation_requested = false;
  request.observation_updated = false;
  const auto stale = gate.update(request);
  EXPECT_TRUE(stale.hold_control);
  EXPECT_EQ(stale.consecutive_valid_cycles, 1);
  EXPECT_EQ(
    stale.reason, orchestrator::WallHandoffAdmissionReason::Requalifying);

  request.observation_updated = true;
  const auto fresh = gate.update(request);
  EXPECT_TRUE(fresh.released);
  EXPECT_FALSE(fresh.hold_control);
}

TEST(OvertakeExecutionOrchestrator, ClassifiesActiveWallPathBeforeLatch)
{
  orchestrator::WallHandoffAdmissionRequest request;
  request.current_footprint_valid = true;
  request.current_footprint_clear = true;
  request.prediction.available = true;
  request.prediction.valid = true;
  request.prediction.minimum_wall_distance_m = 0.19;
  request.required_wall_clearance_m = 0.20;
  EXPECT_EQ(
    orchestrator::classify_wall_path_admission(request),
    orchestrator::WallHandoffAdmissionReason::InsufficientClearance);

  request.prediction.minimum_wall_distance_m = 0.20;
  EXPECT_EQ(
    orchestrator::classify_wall_path_admission(request),
    orchestrator::WallHandoffAdmissionReason::Accepted);
}

TEST(OvertakeExecutionOrchestrator, NeverTimesOutIntoUnsafeWallHandoff)
{
  orchestrator::WallPathAdmissionGate gate;
  orchestrator::WallHandoffAdmissionRequest request;
  request.activation_requested = true;
  request.current_footprint_valid = true;
  request.current_footprint_clear = true;
  request.prediction.available = true;
  request.prediction.valid = true;
  request.prediction.minimum_wall_distance_m = 0.20;
  request.required_wall_clearance_m = 0.40;

  auto result = gate.update(request);
  request.activation_requested = false;
  for (int cycle = 0; cycle < 100; ++cycle) {
    result = gate.update(request);
  }
  EXPECT_TRUE(result.active);
  EXPECT_TRUE(result.hold_control);
  EXPECT_FALSE(result.released);
  EXPECT_EQ(
    result.reason,
    orchestrator::WallHandoffAdmissionReason::InsufficientClearance);
}

TEST(OvertakeExecutionOrchestrator, StopsWallHandoffWhenCurrentFootprintIsUnsafe)
{
  orchestrator::WallPathAdmissionGate gate;
  orchestrator::WallHandoffAdmissionRequest request;
  request.activation_requested = true;
  request.current_footprint_valid = true;
  request.current_footprint_clear = false;
  request.current_contact_count = 1U;

  const auto result = gate.update(request);
  EXPECT_TRUE(result.hold_control);
  EXPECT_TRUE(result.stop_required);
  EXPECT_EQ(
    result.reason,
    orchestrator::WallHandoffAdmissionReason::CurrentFootprintUnsafe);
}

TEST(OvertakeExecutionOrchestrator, FormatsWallHandoffAdmissionEvidence)
{
  orchestrator::WallHandoffAdmissionRequest request;
  request.current_footprint_valid = true;
  request.current_footprint_clear = true;
  request.prediction.available = true;
  request.prediction.valid = true;
  request.prediction.contact = true;
  request.prediction.sample_count = 18U;
  request.prediction.minimum_wall_region = "Mixed";
  request.prediction.minimum_wall_distance_m = 0.0;
  request.prediction.minimum_index = 3U;
  request.prediction.minimum_wall_path_distance_m = 4.82;
  request.required_wall_clearance_m = 0.40;
  orchestrator::WallHandoffAdmissionResolution resolution;
  resolution.entered = true;
  resolution.active = true;
  resolution.hold_control = true;
  resolution.hold_cycles = 1;
  resolution.required_consecutive_valid_cycles = 2;
  resolution.reason =
    orchestrator::WallHandoffAdmissionReason::PredictedContact;

  const auto message = orchestrator::format_wall_path_admission_trace(
    3175U, orchestrator::WallPathAdmissionScope::ActiveOvertake,
    orchestrator::Phase::Pass, orchestrator::PathSource::RecedingDp,
    request, resolution, 4.1, -0.02);
  EXPECT_NE(message.find("decision=3175"), std::string::npos);
  EXPECT_NE(message.find("scope=active-overtake"), std::string::npos);
  EXPECT_NE(message.find("phase=Pass"), std::string::npos);
  EXPECT_NE(message.find("path_source=receding-dp"), std::string::npos);
  EXPECT_NE(message.find("event=entered"), std::string::npos);
  EXPECT_NE(message.find("reason=predicted-wall-contact"), std::string::npos);
  EXPECT_NE(message.find("path_wall=Mixed/0.00m@3/4.82m"), std::string::npos);
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

}  // namespace
