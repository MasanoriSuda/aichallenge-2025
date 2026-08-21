#ifndef MULTI_PURPOSE_MPC_ROS__OVERTAKE_EXECUTION_ORCHESTRATOR_HPP_
#define MULTI_PURPOSE_MPC_ROS__OVERTAKE_EXECUTION_ORCHESTRATOR_HPP_

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace multi_purpose_mpc_ros::overtake_execution_orchestrator {

enum class Phase {
  Idle,
  ShiftOut,
  Pass,
  Return,
  FollowPrepare,
  Recovery,
};

enum class Behavior {
  Cruise,
  Follow,
  Overtake,
  LowSpeedAvoidance,
  SafetyBrake,
};

enum class Action {
  Cruise,
  Follow,
  DynamicEscape,
  ShiftOut,
  Pass,
  Return,
  DynamicWait,
  ContactEscape,
  Recovery,
  SafetyBrake,
};

enum class LateralOwner {
  RacingLine,
  GapPlanner,
  DynamicObstacleEscape,
  OvertakeLine,
  DynamicWaitPrefix,
  ContactEscape,
  RecoveryLine,
  SafetyHold,
};

enum class LongitudinalOwner {
  RacingLine,
  FollowCap,
  DynamicObstacleEscape,
  OvertakeLine,
  PassFloor,
  SolverFallback,
  SafetyBrake,
};

enum class PathSource {
  RacingLine,
  GapPlanner,
  DynamicObstacleEscape,
  FrozenMission,
  RecedingHorizon,
  RecedingDp,
  DynamicWaitPrefix,
  ContactEscape,
  RecoveryLine,
  SafetyHold,
};

enum AuthorityConflict : std::uint32_t {
  NoConflict = 0U,
  SafetyWithActiveLine = 1U << 0U,
  SafetyWithSpeedFloor = 1U << 1U,
  ReleasedPassWithFollowCap = 1U << 2U,
  DynamicWaitWithoutLateralAuthority = 1U << 3U,
  ActivePhaseWithoutTarget = 1U << 4U,
  MultipleLateralAuthorities = 1U << 5U,
  InvalidSpeedWindow = 1U << 6U,
  WallContractShortfall = 1U << 7U,
};

struct CorridorMetrics {
  bool valid{false};
  std::size_t sample_count{0U};
  std::size_t minimum_width_index{0U};
  double minimum_width_m{std::numeric_limits<double>::infinity()};
  double minimum_width_distance_m{std::numeric_limits<double>::quiet_NaN()};
};

struct SpeedWindowResolution {
  bool valid{false};
  bool floor_adjusted{false};
  double reference_mps{std::numeric_limits<double>::infinity()};
  double limit_mps{std::numeric_limits<double>::infinity()};
  double requested_floor_mps{0.0};
  double floor_mps{0.0};
};

SpeedWindowResolution normalize_speed_window(
  double reference_mps, double limit_mps, double floor_mps,
  bool floor_active) noexcept;

struct WallClearanceContract {
  bool valid{false};
  double physical_clearance_m{0.0};
  double planning_clearance_m{0.0};
  double runtime_reserve_m{0.0};
  double required_clearance_m{0.0};
};

WallClearanceContract resolve_wall_clearance_contract(
  double physical_clearance_m, double planning_clearance_m,
  bool runtime_preplan_enabled, double runtime_reserve_m) noexcept;

enum class LateralBoundContractReason {
  None,
  NonFiniteBound,
  EmptyIntersection,
};

struct LateralBoundContractResolution {
  bool valid{false};
  bool feasible{false};
  double lower_m{std::numeric_limits<double>::quiet_NaN()};
  double upper_m{std::numeric_limits<double>::quiet_NaN()};
  LateralBoundContractReason reason{LateralBoundContractReason::NonFiniteBound};
};

/// Validate the final stage bound before it is encoded into the solver.  An
/// empty intersection is an execution-contract failure, not a zero-width
/// center-line corridor.
LateralBoundContractResolution resolve_lateral_bound_contract(
  double lower_m, double upper_m) noexcept;
const char * to_string(LateralBoundContractReason reason) noexcept;

enum class RuntimeReplacementRejectReason {
  None,
  InvalidCandidate,
  PredictionInvalid,
  PredictionExpired,
  TargetClearanceUnchecked,
  TargetClearanceInvalid,
  TargetOverlap,
  WallClearanceUnchecked,
  WallContractShortfall,
};

struct RuntimeReplacementContractRequest {
  bool candidate_feasible{false};
  double now_sec{std::numeric_limits<double>::quiet_NaN()};
  double dynamic_valid_until_sec{std::numeric_limits<double>::quiet_NaN()};
  bool target_clearance_checked{false};
  double minimum_target_clearance_m{std::numeric_limits<double>::quiet_NaN()};
  double minimum_path_wall_clearance_m{std::numeric_limits<double>::quiet_NaN()};
  double required_path_wall_clearance_m{std::numeric_limits<double>::quiet_NaN()};
};

struct RuntimeReplacementContractResolution {
  bool valid{false};
  bool admitted{false};
  RuntimeReplacementRejectReason reason{
    RuntimeReplacementRejectReason::InvalidCandidate};
};

RuntimeReplacementContractResolution resolve_runtime_replacement_contract(
  const RuntimeReplacementContractRequest & request) noexcept;
const char * to_string(RuntimeReplacementRejectReason reason) noexcept;

CorridorMetrics analyze_corridor(
  const std::vector<double> & lower_m,
  const std::vector<double> & upper_m,
  const std::vector<double> & path_distance_m) noexcept;

struct AuthorityRequest {
  std::uint64_t decision_id{0U};
  std::uint64_t episode_id{0U};
  std::uint64_t mission_generation{0U};
  std::string target_id;
  int pass_side_sign{0};
  Phase phase{Phase::Idle};
  Behavior behavior{Behavior::Cruise};
  PathSource path_source_hint{PathSource::RacingLine};
  double path_age_sec{std::numeric_limits<double>::infinity()};
  bool line_active{false};
  bool stage_corridor_active{false};
  bool gap_planner_active{false};
  bool dynamic_obstacle_escape_active{false};
  bool dynamic_obstacle_follow_cap_suppressed{false};
  bool dynamic_wait_active{false};
  bool dynamic_wait_forward_prefix_active{false};
  bool dynamic_wait_lateral_authority_active{false};
  bool contact_continuation_active{false};
  bool precontact_escape_active{false};
  bool emergency_brake_active{false};
  bool solver_fallback_active{false};
  bool follow_cap_active{false};
  bool front_cap_release_ready{false};
  bool pass_speed_floor_active{false};
  bool shiftout_speed_floor_active{false};
  bool corridor_blocked{false};
  double speed_reference_mps{std::numeric_limits<double>::infinity()};
  double speed_limit_mps{std::numeric_limits<double>::infinity()};
  double speed_floor_mps{0.0};
  double requested_speed_floor_mps{0.0};
  bool speed_floor_adjusted{false};
  double front_distance_m{std::numeric_limits<double>::infinity()};
  double dynamic_front_safety_distance_m{std::numeric_limits<double>::infinity()};
  double protected_front_distance_m{std::numeric_limits<double>::infinity()};
  double closing_speed_reference_mps{std::numeric_limits<double>::infinity()};
  double wall_contract_required_clearance_m{0.0};
  double wall_contract_minimum_path_clearance_m{
    std::numeric_limits<double>::infinity()};
  std::string transition_reason;
  std::string blocking_reason;
};

struct AuthorityResolution {
  bool relevant{false};
  Action action{Action::Cruise};
  LateralOwner lateral_owner{LateralOwner::RacingLine};
  LongitudinalOwner longitudinal_owner{LongitudinalOwner::RacingLine};
  PathSource path_source{PathSource::RacingLine};
  bool use_overtake_line_target{false};
  bool apply_overtake_speed_reference{false};
  bool apply_overtake_speed_limit{false};
  bool apply_overtake_speed_floor{false};
  std::uint32_t conflicts{NoConflict};
  std::string reason{"normal-racing-line"};
};

AuthorityResolution resolve_authority(const AuthorityRequest & request) noexcept;

const char * to_string(Phase phase) noexcept;
const char * to_string(Behavior behavior) noexcept;
const char * to_string(Action action) noexcept;
const char * to_string(LateralOwner owner) noexcept;
const char * to_string(LongitudinalOwner owner) noexcept;
const char * to_string(PathSource source) noexcept;
std::string format_conflicts(std::uint32_t conflicts);

struct AuthorityTrace {
  AuthorityRequest request;
  AuthorityResolution resolution;
  CorridorMetrics constrained_corridor;
  CorridorMetrics wall_corridor;
  double static_valid_until_m{0.0};
  double dynamic_valid_until_m{0.0};
  double predicted_rear_clear_m{std::numeric_limits<double>::infinity()};
  double ego_speed_mps{0.0};
  int waypoint_id{0};
};

struct TraceEmission {
  bool emit{false};
  bool state_changed{false};
  bool conflict{false};
  std::string signature;
  std::string message;
};

std::string categorical_signature(const AuthorityTrace & trace);
std::string format_authority_trace(const AuthorityTrace & trace);

class ChangeAwareAuthorityTraceEmitter {
public:
  TraceEmission update(
    const AuthorityTrace & trace, double now_sec,
    double repeat_interval_sec = 5.0);
  void reset() noexcept;

private:
  std::string last_signature_;
  double last_emit_sec_{-std::numeric_limits<double>::infinity()};
  bool was_relevant_{false};
};

enum class FinalControlSource {
  MpcSolution,
  LowSpeedDirect,
  LowSpeedWallStop,
  SolverFallback,
  SolverBoundedContinuation,
  SolverWallHandoffHold,
  OvertakeWallAdmissionHold,
  ExecutedSolutionWallHold,
  SolverCrawl,
  ControlDisabled,
  StuckRecovery,
  Failsafe,
};

struct FinalControlSourceRequest {
  bool failsafe_active{false};
  bool stuck_recovery_active{false};
  bool control_enabled{true};
  bool low_speed_wall_stop_active{false};
  bool solver_bounded_continuation_active{false};
  bool solver_wall_handoff_hold_active{false};
  bool overtake_wall_admission_hold_active{false};
  bool executed_solution_wall_hold_active{false};
  bool solver_crawl_active{false};
  bool solver_fallback_active{false};
  bool forced_stop_active{false};
  bool low_speed_direct_active{false};
};

FinalControlSource resolve_final_control_source(
  const FinalControlSourceRequest & request) noexcept;
const char * to_string(FinalControlSource source) noexcept;

struct FinalControlTrace {
  std::uint64_t decision_id{0U};
  std::optional<AuthorityTrace> authority;
  FinalControlSource control_source{FinalControlSource::MpcSolution};
  bool published{false};
  double actual_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double target_speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double acceleration_mps2{std::numeric_limits<double>::quiet_NaN()};
  double raw_steering_rad{std::numeric_limits<double>::quiet_NaN()};
  double published_steering_rad{std::numeric_limits<double>::quiet_NaN()};
  std::string solver_reason;
  std::string output_reason;
};

struct FinalTraceEmission {
  bool emit{false};
  bool state_changed{false};
  bool warning{false};
  std::size_t suppressed_normal_change_count{0U};
  std::string signature;
  std::string message;
};

std::string final_control_signature(const FinalControlTrace & trace);
std::string format_final_control_trace(const FinalControlTrace & trace);

class ChangeAwareFinalControlTraceEmitter {
public:
  FinalTraceEmission update(
    const FinalControlTrace & trace, double now_sec,
    double repeat_interval_sec = 5.0);
  void reset() noexcept;

private:
  std::string last_signature_;
  std::string last_detail_signature_;
  double last_emit_sec_{-std::numeric_limits<double>::infinity()};
  bool was_relevant_{false};
  std::size_t suppressed_normal_change_count_{0U};
};

enum class WallRiskState {
  Unknown,
  Clear,
  Near,
  Contact,
};

const char * to_string(WallRiskState state) noexcept;

struct PredictedPathWallMetrics {
  bool available{false};
  bool valid{false};
  bool retained_solution{false};
  bool contact{false};
  bool out_of_map{false};
  std::size_t sample_count{0U};
  std::size_t minimum_index{0U};
  std::string minimum_wall_region{"Unknown"};
  double minimum_wall_distance_m{std::numeric_limits<double>::infinity()};
  double minimum_wall_path_distance_m{
    std::numeric_limits<double>::quiet_NaN()};
};

enum class WallHandoffAdmissionReason {
  Inactive,
  CurrentFootprintUnavailable,
  CurrentFootprintUnsafe,
  PredictionUnavailable,
  PredictionInvalid,
  PredictedContact,
  PredictedOutOfMap,
  InsufficientClearance,
  Requalifying,
  Accepted,
};

const char * to_string(WallHandoffAdmissionReason reason) noexcept;

struct WallHandoffAdmissionRequest {
  std::uint64_t mission_generation{0U};
  bool activation_requested{false};
  bool observation_updated{true};
  bool current_footprint_valid{false};
  bool current_footprint_clear{false};
  bool current_footprint_out_of_map{false};
  std::size_t current_contact_count{0U};
  PredictedPathWallMetrics prediction;
  double required_wall_clearance_m{0.0};
  // A small, explicitly logged tolerance for discretization at a physically
  // clear boundary. It never overrides contact, out-of-map, or invalid data.
  double physical_clearance_tolerance_m{0.0};
  bool planner_wall_contract_available{false};
  // This is the lateral reserve inside the Frenet planning corridor, not a
  // physical vehicle-footprint-to-wall distance.  Keep the semantic explicit
  // so execution telemetry never compares two differently measured distances
  // as though they came from the same sensor/model.
  double planner_minimum_corridor_reserve_m{
    std::numeric_limits<double>::infinity()};
  int required_consecutive_valid_cycles{2};
};

struct WallHandoffAdmissionResolution {
  bool active{false};
  bool entered{false};
  bool released{false};
  bool hold_control{false};
  bool stop_required{false};
  bool replan_required{false};
  bool replan_requested{false};
  bool planner_contract_admitted{false};
  bool planner_execution_contract_mismatch{false};
  bool boundary_clearance_accepted{false};
  double physical_clearance_shortfall_m{0.0};
  bool state_changed{false};
  int hold_cycles{0};
  int consecutive_valid_cycles{0};
  int required_consecutive_valid_cycles{2};
  WallHandoffAdmissionReason reason{WallHandoffAdmissionReason::Inactive};
};

/// Classify one physical wall observation independently of the stateful
/// admission latch. Accepted means that both the current footprint and the
/// predicted path satisfy the requested wall contract.
WallHandoffAdmissionReason classify_wall_path_admission(
  const WallHandoffAdmissionRequest & request) noexcept;

class WallPathAdmissionGate {
public:
  WallHandoffAdmissionResolution update(
    const WallHandoffAdmissionRequest & request) noexcept;
  bool active() const noexcept;
  void reset() noexcept;

private:
  bool active_{false};
  int hold_cycles_{0};
  int consecutive_valid_cycles_{0};
  WallHandoffAdmissionReason previous_reason_{
    WallHandoffAdmissionReason::Inactive};
  bool previous_stop_required_{false};
};

enum class DynamicEscapeAttemptReason {
  Inactive,
  Started,
  PlannerRequested,
  RequestGapHeld,
  TargetLossGrace,
  TargetLost,
  TargetChanged,
  ExplicitRelease,
};

const char * to_string(DynamicEscapeAttemptReason reason) noexcept;

/// One Dynamic Escape attempt represents an encounter with one relevant
/// moving obstacle, not one solver/candidate cycle.  Planner availability may
/// legitimately flicker while a branch is quarantined or rebuilt, so it must
/// not own the attempt lifetime.
struct DynamicEscapeAttemptRequest {
  bool planner_requested{false};
  bool target_relevant{false};
  bool explicit_release{false};
  std::string target_id;
  double now_sec{std::numeric_limits<double>::quiet_NaN()};
  double target_loss_grace_sec{0.50};
};

struct DynamicEscapeAttemptResolution {
  std::uint64_t attempt_id{0U};
  bool active{false};
  bool started{false};
  bool released{false};
  bool retargeted{false};
  bool held_without_request{false};
  bool state_changed{false};
  std::string target_id;
  std::string previous_target_id;
  int lifetime_cycles{0};
  int planner_request_cycles{0};
  int request_gap_cycles{0};
  double target_loss_age_sec{std::numeric_limits<double>::infinity()};
  double target_loss_grace_sec{0.50};
  DynamicEscapeAttemptReason reason{DynamicEscapeAttemptReason::Inactive};
};

class DynamicEscapeAttemptTracker {
public:
  DynamicEscapeAttemptResolution update(
    const DynamicEscapeAttemptRequest & request) noexcept;
  bool active() const noexcept;
  std::uint64_t attempt_id() const noexcept;
  const std::string & target_id() const noexcept;
  void reset() noexcept;

private:
  std::uint64_t allocate_attempt_id() noexcept;

  std::uint64_t next_attempt_id_{1U};
  std::uint64_t attempt_id_{0U};
  bool active_{false};
  std::string target_id_;
  double last_target_relevant_sec_{
    -std::numeric_limits<double>::infinity()};
  int lifetime_cycles_{0};
  int planner_request_cycles_{0};
  int request_gap_cycles_{0};
  DynamicEscapeAttemptReason previous_reason_{
    DynamicEscapeAttemptReason::Inactive};
};

std::string format_dynamic_escape_attempt_trace(
  const DynamicEscapeAttemptRequest & request,
  const DynamicEscapeAttemptResolution & resolution,
  int waypoint_id);

enum class DynamicEscapeExitReason {
  Inactive,
  TargetBlocking,
  RetainedSolutionExpired,
  WallPathPending,
  ReplacementPending,
  ReplacementExecuting,
  ReplacementLost,
  TargetResolvedRequalifying,
  TargetResolved,
  RecoveryOverride,
};

const char * to_string(DynamicEscapeExitReason reason) noexcept;

/// State presented to the Dynamic Escape exit contract.  Wall admission and
/// moving-obstacle completion are deliberately separate: a RacingLine path
/// can be physically clear of the wall while still converging onto the front
/// vehicle that Dynamic Escape was trying to pass.
struct DynamicEscapeExitRequest {
  std::uint64_t escape_attempt_id{0U};
  bool activation_requested{false};
  bool observation_updated{true};
  bool wall_path_admitted{false};
  bool obstacle_blocking{false};
  bool replacement_escape_active{false};
  bool replacement_escape_admitted{false};
  bool retained_solution_available{false};
  bool recovery_override{false};
  int required_consecutive_resolved_cycles{2};
};

struct DynamicEscapeExitResolution {
  std::uint64_t latched_attempt_id{0U};
  bool active{false};
  bool entered{false};
  bool released{false};
  bool hold_lateral_control{false};
  bool replan_required{false};
  bool replan_requested{false};
  bool replacement_adopted{false};
  bool replacement_execution_active{false};
  bool attempt_changed{false};
  bool state_changed{false};
  int hold_cycles{0};
  int consecutive_resolved_cycles{0};
  int required_consecutive_resolved_cycles{2};
  DynamicEscapeExitReason reason{DynamicEscapeExitReason::Inactive};
};

/// Prevents Dynamic Escape from handing lateral authority to RacingLine while
/// the obstacle that caused the escape is still blocking.  The gate never
/// manufactures a control command; it only authorizes a short retained-solution
/// lease and requests a replacement escape plan.
class DynamicEscapeExitGate {
public:
  DynamicEscapeExitResolution update(
    const DynamicEscapeExitRequest & request) noexcept;
  bool active() const noexcept;
  void reset() noexcept;

private:
  bool active_{false};
  std::uint64_t latched_attempt_id_{0U};
  bool replacement_execution_active_{false};
  int hold_cycles_{0};
  int consecutive_resolved_cycles_{0};
  DynamicEscapeExitReason previous_reason_{DynamicEscapeExitReason::Inactive};
  bool previous_hold_lateral_control_{false};
};

std::string format_dynamic_escape_exit_trace(
  std::uint64_t decision_id,
  const DynamicEscapeExitRequest & request,
  const DynamicEscapeExitResolution & resolution,
  const std::string & latched_target_id,
  const std::string & observed_target_id,
  int latched_side_sign,
  double front_distance_m,
  double protected_front_distance_m,
  double closing_speed_mps,
  double retained_age_sec);

enum class WallPathAdmissionScope {
  SolverHandoff,
  ActiveOvertake,
  DynamicEscapeExit,
};

const char * to_string(WallPathAdmissionScope scope) noexcept;

std::string format_wall_path_admission_trace(
  std::uint64_t decision_id,
  WallPathAdmissionScope scope, Phase phase, PathSource path_source,
  const WallHandoffAdmissionRequest & request,
  const WallHandoffAdmissionResolution & resolution,
  double held_speed_mps, double held_steering_rad);

enum class ExecutedSolutionWallAction {
  Publish,
  EntryRollback,
  DynamicReplan,
  RecoveryReplan,
  HoldCurrentPath,
};

const char * to_string(ExecutedSolutionWallAction action) noexcept;

struct ExecutedSolutionWallRequest {
  bool execution_context_active{false};
  bool solution_wall_safe{false};
  bool execution_command_published{false};
  Phase phase{Phase::Idle};
  double phase_traveled_m{0.0};
};

struct ExecutedSolutionWallResolution {
  bool valid{false};
  bool publish_solution{false};
  ExecutedSolutionWallAction action{ExecutedSolutionWallAction::HoldCurrentPath};
};

/// Decide how an exact solver trajectory is handled after the reference path
/// has already passed entry admission.  A ShiftOut whose command has never
/// been published may be rolled back atomically; a published lateral
/// manoeuvre must keep ownership through DynamicReplan or RecoveryReplan.
ExecutedSolutionWallResolution resolve_executed_solution_wall_action(
  const ExecutedSolutionWallRequest & request) noexcept;

struct WallHandoffProbe {
  std::uint64_t decision_id{0U};
  bool dynamic_escape_active{false};
  Action action{Action::Cruise};
  LateralOwner lateral_owner{LateralOwner::RacingLine};
  PathSource path_source{PathSource::RacingLine};
  FinalControlSource control_source{FinalControlSource::MpcSolution};
  bool current_wall_valid{false};
  bool current_footprint_clear{false};
  bool current_footprint_out_of_map{false};
  std::size_t current_contact_count{0U};
  std::string current_wall_region{"Unknown"};
  double current_wall_distance_m{std::numeric_limits<double>::infinity()};
  double required_wall_clearance_m{0.0};
  double pose_x_m{std::numeric_limits<double>::quiet_NaN()};
  double pose_y_m{std::numeric_limits<double>::quiet_NaN()};
  double pose_yaw_rad{std::numeric_limits<double>::quiet_NaN()};
  double lateral_error_m{std::numeric_limits<double>::quiet_NaN()};
  double heading_error_rad{std::numeric_limits<double>::quiet_NaN()};
  double speed_mps{std::numeric_limits<double>::quiet_NaN()};
  double yaw_rate_radps{std::numeric_limits<double>::quiet_NaN()};
  double raw_steering_rad{std::numeric_limits<double>::quiet_NaN()};
  double published_steering_rad{std::numeric_limits<double>::quiet_NaN()};
  double previous_published_steering_rad{
    std::numeric_limits<double>::quiet_NaN()};
  double collision_age_sec{std::numeric_limits<double>::infinity()};
};

struct WallHandoffEvent {
  bool emit{false};
  bool warning{false};
  bool monitor_active{false};
  bool source_changed{false};
  bool risk_changed{false};
  WallRiskState risk{WallRiskState::Unknown};
  Action previous_action{Action::Cruise};
  LateralOwner previous_lateral_owner{LateralOwner::RacingLine};
  PathSource previous_path_source{PathSource::RacingLine};
  FinalControlSource previous_control_source{FinalControlSource::MpcSolution};
  std::string trigger{"none"};
};

WallRiskState classify_wall_risk(const WallHandoffProbe & probe) noexcept;
std::string format_wall_handoff_trace(
  const WallHandoffProbe & probe, const WallHandoffEvent & event,
  const PredictedPathWallMetrics & path_metrics);

class ChangeAwareWallHandoffTraceEmitter {
public:
  WallHandoffEvent update(
    const WallHandoffProbe & probe, double now_sec,
    double monitor_duration_sec = 2.0, double risk_repeat_interval_sec = 0.5);
  void reset() noexcept;

private:
  bool initialized_{false};
  bool previous_dynamic_relevant_{false};
  Action previous_action_{Action::Cruise};
  LateralOwner previous_lateral_owner_{LateralOwner::RacingLine};
  PathSource previous_path_source_{PathSource::RacingLine};
  FinalControlSource previous_control_source_{FinalControlSource::MpcSolution};
  WallRiskState previous_risk_{WallRiskState::Unknown};
  double monitor_until_sec_{-std::numeric_limits<double>::infinity()};
  double last_risk_emit_sec_{-std::numeric_limits<double>::infinity()};
};

struct EpisodeStart {
  std::uint64_t episode_id{0U};
  std::string target_id;
  int side{0};
  double now_sec{0.0};
  int waypoint_id{0};
  std::string reason;
};

struct EpisodeSample {
  std::uint64_t episode_id{0U};
  std::uint64_t mission_generation{0U};
  std::string target_id;
  Phase phase{Phase::Idle};
  Action action{Action::Cruise};
  LateralOwner lateral_owner{LateralOwner::RacingLine};
  LongitudinalOwner longitudinal_owner{LongitudinalOwner::RacingLine};
  double now_sec{0.0};
  double ego_speed_mps{0.0};
  CorridorMetrics constrained_corridor;
  CorridorMetrics wall_corridor;
  double maximum_required_lateral_accel_mps2{0.0};
  bool dynamic_wait_active{false};
  bool contact_escape_active{false};
  std::uint32_t authority_conflicts{NoConflict};
};

struct EpisodeSummary {
  bool valid{false};
  std::uint64_t episode_id{0U};
  std::string target_id;
  int side{0};
  double elapsed_sec{0.0};
  double minimum_speed_mps{std::numeric_limits<double>::infinity()};
  double minimum_constrained_corridor_width_m{
    std::numeric_limits<double>::infinity()};
  double minimum_wall_corridor_width_m{
    std::numeric_limits<double>::infinity()};
  double maximum_required_lateral_accel_mps2{0.0};
  std::uint64_t maximum_mission_generation{0U};
  std::size_t authority_change_count{0U};
  std::size_t dynamic_wait_entry_count{0U};
  std::size_t contact_escape_entry_count{0U};
  std::size_t authority_conflict_sample_count{0U};
  std::string phases;
  std::string final_phase;
  std::string final_reason;
  int start_waypoint_id{0};
  int final_waypoint_id{0};
};

std::string format_episode_summary(const EpisodeSummary & summary);

class EpisodeAccumulator {
public:
  void begin(const EpisodeStart & start);
  void observe(const EpisodeSample & sample);
  std::optional<EpisodeSummary> finish(
    double now_sec, const std::string & final_phase,
    const std::string & final_reason, int final_waypoint_id);
  bool active() const noexcept;
  void reset() noexcept;

private:
  bool active_{false};
  EpisodeStart start_;
  std::string target_id_;
  double minimum_speed_mps_{std::numeric_limits<double>::infinity()};
  double minimum_constrained_corridor_width_m_{
    std::numeric_limits<double>::infinity()};
  double minimum_wall_corridor_width_m_{
    std::numeric_limits<double>::infinity()};
  double maximum_required_lateral_accel_mps2_{0.0};
  std::uint64_t maximum_mission_generation_{0U};
  std::size_t authority_change_count_{0U};
  std::size_t dynamic_wait_entry_count_{0U};
  std::size_t contact_escape_entry_count_{0U};
  std::size_t authority_conflict_sample_count_{0U};
  std::uint32_t phase_mask_{0U};
  bool previous_dynamic_wait_active_{false};
  bool previous_contact_escape_active_{false};
  bool authority_initialized_{false};
  Action previous_action_{Action::Cruise};
  LateralOwner previous_lateral_owner_{LateralOwner::RacingLine};
  LongitudinalOwner previous_longitudinal_owner_{LongitudinalOwner::RacingLine};
};

}  // namespace multi_purpose_mpc_ros::overtake_execution_orchestrator

#endif  // MULTI_PURPOSE_MPC_ROS__OVERTAKE_EXECUTION_ORCHESTRATOR_HPP_
