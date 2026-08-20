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
};

struct CorridorMetrics {
  bool valid{false};
  std::size_t sample_count{0U};
  std::size_t minimum_width_index{0U};
  double minimum_width_m{std::numeric_limits<double>::infinity()};
  double minimum_width_distance_m{std::numeric_limits<double>::quiet_NaN()};
};

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
  double last_emit_sec_{-std::numeric_limits<double>::infinity()};
  bool was_relevant_{false};
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
