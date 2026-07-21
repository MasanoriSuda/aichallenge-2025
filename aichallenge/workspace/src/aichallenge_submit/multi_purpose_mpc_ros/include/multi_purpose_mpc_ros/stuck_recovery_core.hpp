#ifndef MULTI_PURPOSE_MPC_ROS__STUCK_RECOVERY_CORE_HPP_
#define MULTI_PURPOSE_MPC_ROS__STUCK_RECOVERY_CORE_HPP_

#include <cstddef>
#include <optional>

namespace multi_purpose_mpc_ros::stuck_recovery
{

// V2X source stamps can use simulation time while the receiving ROS node uses
// a wall/system clock. Receipt freshness must therefore be checked with the
// receiver clock, and source ordering must be checked only within the source
// clock domain.
bool source_timestamp_is_monotonic(
  double stamp_sec, const std::optional<double> & previous_stamp_sec) noexcept;

bool source_sample_is_current(
  double array_stamp_sec, double sample_stamp_sec, double timeout_sec) noexcept;

struct FaultRetryConfig
{
  bool enabled{false};
  double clear_confirm_sec{0.5};
  double max_observation_gap_sec{0.2};
};

struct FaultRetryInput
{
  double now_sec{};
  bool simulation_environment{false};
  bool race_started{false};
  bool control_enabled{false};
  bool odometry_fresh_and_finite{false};
  bool command_finite{false};
  bool drive_gear_fresh{false};
  bool boost_inactive{false};
  bool v2x_complete{false};
  bool bounded_maneuver_available{false};
  bool collision_worsening{false};
};

class FaultRetryGate
{
public:
  explicit FaultRetryGate(FaultRetryConfig config);

  bool update(const FaultRetryInput & input);
  void reset() noexcept;

private:
  FaultRetryConfig config_;
  std::optional<double> healthy_since_sec_;
  std::optional<double> last_update_sec_;
};

struct CollisionDeliberateStopOverrideRequest
{
  bool enabled{false};
  bool simulation_environment{false};
  bool collision_hint{false};
  bool has_front_vehicle{false};
  bool forward_intent{false};
  double signed_speed_mps{};
  double stopped_speed_mps{};
};

// A recent physical collision is stronger evidence than a transient V2X
// SafetyBrake/Follow classification. This only removes the deliberate-stop
// eligibility veto; the normal no-progress timer and all recovery rollout
// gates remain authoritative.
bool should_override_deliberate_stop_for_collision(
  const CollisionDeliberateStopOverrideRequest & request) noexcept;

// Accept a bounded measurement/stopping error only after the current vehicle
// footprint has become clear. The tolerance must never bypass the footprint
// requirement.
bool recovery_escape_distance_confirmed(
  bool current_footprint_clear, double traveled_distance_m,
  double target_distance_m, double tolerance_m) noexcept;

// Evidence-free solver recovery must remain reverse-only. When current wall
// evidence exists, however, the wall direction selects the safe escape unless
// the heading error is large enough to forbid a short forward maneuver.
bool solver_fallback_requires_reverse_only(
  bool solver_fallback, bool evidence_free_recovery_enabled,
  bool wall_evidence, double heading_error_rad,
  double reverse_only_heading_error_rad) noexcept;

struct RejoinSteeringRequest
{
  double path_curvature_radpm{};
  double wheelbase_m{};
  double lateral_error_m{};
  double heading_error_rad{};
  double lateral_error_gain_rad_per_m{};
  double heading_error_gain{};
  double max_steering_tire_angle_rad{};
};

// Low-speed path-rejoin steering. The feedforward term follows path curvature,
// while both feedback terms drive the signed path errors toward zero.
std::optional<double> compute_rejoin_steering_tire_angle(
  const RejoinSteeringRequest & request) noexcept;

enum class StuckVerdict
{
  NotEligible,
  Moving,
  Suspected,
  Confirmed,
};

enum class StuckRejectReason
{
  None,
  FeatureDisabled,
  RaceNotStarted,
  ControlDisabled,
  OdometryStale,
  InvalidInput,
  NonMonotonicTime,
  SolverFallback,
  SolverFallbackMissingWallEvidence,
  DeliberateStop,
  GearTransition,
  AwsimRecoverySettling,
  NoForwardIntent,
  VehicleMoving,
  PoseProgressing,
  PathProgressing,
  ObservationWindowIncomplete,
  MissingCorroboratingEvidence,
};

struct DetectorConfig
{
  // Allow a persistent MPC fallback to become a stuck candidate only when the
  // normal path still requests forward motion and current footprint-to-wall
  // evidence is present. Disabled by default so transient solver failures
  // retain the legacy fail-safe behavior.
  bool solver_fallback_recovery_enabled{false};
  double solver_fallback_duration_sec{2.0};
  // A persistent solver fallback may recover without wall/contact evidence
  // only after a longer no-motion window. The adapter must restrict this
  // route to reverse-only actuation and still prove rear static/V2X clearance.
  bool solver_evidence_free_recovery_enabled{false};
  double solver_evidence_free_duration_sec{3.0};
  // Optional fallback for a physically stuck vehicle whose simulator wall and
  // occupancy-grid/collision evidence disagree. This never applies while the
  // MPC solver itself is in fallback.
  bool evidence_free_recovery_enabled{false};
  double evidence_free_duration_sec{3.0};
  // Allow a vehicle stopped behind another stopped V2X vehicle to join a
  // reverse-only recovery cascade after a longer confirmation window.
  bool coordinated_stop_recovery_enabled{false};
  double coordinated_stop_duration_sec{3.0};
  // Observation time is continuous only while updates stay within this gap.
  double max_observation_gap_sec{0.25};
  double stopped_speed_mps{0.15};
  double moving_speed_mps{0.25};
  double forward_intent_speed_mps{1.0};
  double forward_intent_acceleration_mps2{0.1};
  double stationary_duration_sec{1.2};
  double max_pose_displacement_m{0.15};
  double max_progress_delta_m{0.20};
};

struct DetectorInput
{
  double now_sec{};
  bool race_started{false};
  bool control_enabled{false};
  bool odometry_fresh{false};
  bool solver_fallback{false};
  bool deliberate_stop{false};
  bool coordinated_stop{false};
  bool gear_transition_active{false};
  bool awsim_recovery_settling{false};
  double signed_speed_mps{};
  double requested_forward_speed_mps{};
  double requested_acceleration_mps2{};
  double pose_displacement_m{};
  double unwrapped_progress_delta_m{};
  bool wall_evidence{false};
  bool collision_hint{false};
};

struct DetectorDecision
{
  StuckVerdict verdict{StuckVerdict::NotEligible};
  StuckRejectReason reject_reason{StuckRejectReason::None};
  double stationary_duration_sec{};
  double pose_displacement_m{};
  double progress_delta_m{};
  double solver_fallback_duration_sec{};
  bool forward_intent{false};
  bool corroborating_evidence{false};
  bool solver_fallback_qualified{false};
  bool solver_evidence_free_qualified{false};
  bool evidence_free_qualified{false};
  bool coordinated_stop_qualified{false};
};

class StuckDetector
{
public:
  explicit StuckDetector(DetectorConfig config);

  DetectorDecision update(const DetectorInput & input);
  void reset() noexcept;

  [[nodiscard]] const DetectorConfig & config() const noexcept;

private:
  DetectorDecision reject(
    const DetectorInput & input, StuckVerdict verdict, StuckRejectReason reason,
    bool forward_intent = false, bool evidence = false) noexcept;
  void reset_observation() noexcept;

  DetectorConfig config_;
  std::optional<double> stationary_since_sec_;
  std::optional<double> solver_fallback_since_sec_;
  std::optional<double> last_update_sec_;
};

// The ROS adapter maps this semantic gear to GearCommand/GearReport constants. No ROS numeric
// constant is intentionally embedded in the pure core.
enum class Gear
{
  NoCommand,
  Unknown,
  Neutral,
  Drive,
  Reverse,
};

enum class ManeuverDirection
{
  Unknown,
  Reverse,
  Forward,
};

enum class RecoveryState
{
  Normal,
  SuspectStuck,
  WaitAwsimRecovery,
  StopAndConfirm,
  CheckClearance,
  WaitForClear,
  ShiftToReverse,
  WaitReverseReport,
  ReverseManeuver,
  ForwardManeuver,
  StopAndReassess,
  StopBeforeDrive,
  ShiftToDrive,
  WaitDriveReport,
  LowSpeedRejoin,
  SafeStop,
};

/// Return true once a recovery candidate has entered an actuation path and may be fixed for the
/// rest of that maneuver. Observation/clearance states must continue to re-evaluate direction.
bool recovery_candidate_commit_allowed(RecoveryState state) noexcept;

/// Reverse direction may be latched after the AWSIM settling state while the concrete
/// primitive/steering remains re-evaluated against the current pose and clearance snapshot.
bool recovery_reverse_intent_latch_allowed(RecoveryState state) noexcept;

struct ReverseDirectionPolicyInput
{
  bool reverse_only_episode{false};
  bool reverse_intent_latched{false};
  bool coordinated_stop_active{false};
  bool solver_reverse_only_candidate{false};
  bool obstacle_reverse_first{false};
  bool forward_fallback_unlocked{false};
};

/// Strict episode reasons and a latched Reverse intent always win. A non-strict obstacle-first
/// request may release Forward candidates only after an explicit failed recovery retry.
bool recovery_reverse_direction_required(
  const ReverseDirectionPolicyInput & input) noexcept;

enum class RecoveryActionType
{
  NormalControl,
  HoldStop,
  RequestReverse,
  ReverseCreep,
  ForwardCreep,
  RequestDrive,
  LowSpeedRejoin,
  SafeStop,
};

enum class RecoveryReason
{
  None,
  Disabled,
  ShadowMode,
  SimulationOnlyBlocked,
  RaceInactive,
  AwaitingStuckConfirmation,
  StuckSuspected,
  StuckConfirmed,
  AwsimRecoveryWaiting,
  AwsimRecoveryResolved,
  StopConfirmationPending,
  ClearanceCheck,
  RearStaticBlocked,
  RearVehicleBlocked,
  RearInformationIncomplete,
  ClearanceWaitTimedOut,
  ManeuverDirectionUnknown,
  AttemptLimitReached,
  ReverseGearRequested,
  ReverseGearConfirmed,
  GearReportMissing,
  GearReportInvalid,
  GearReportTimedOut,
  GearCommandLimitReached,
  ReverseInProgress,
  ReverseDistanceLimit,
  ReverseDurationLimit,
  ReverseSpeedLimit,
  ReverseEscapeConfirmed,
  CollisionWorsening,
  RearHazardAppeared,
  ReverseGearLost,
  ForwardInProgress,
  ForwardDistanceLimit,
  ForwardDurationLimit,
  ForwardSpeedLimit,
  ForwardEscapeConfirmed,
  ForwardHazardAppeared,
  EscapeStepComplete,
  EscapeStepLimitReached,
  ContactNotImproving,
  DriveGearRequested,
  DriveGearConfirmed,
  DriveGearLost,
  EscapeNotConfirmed,
  RejoinInProgress,
  RejoinComplete,
  RejoinTimedOut,
  RejoinUnsafe,
  RejoinPathBlocked,
  SolverRecoveryPending,
  AggressiveRetry,
  CooldownActive,
  OdometryUnsafe,
  SolverUnsafe,
  ControlInterrupted,
  InvalidInput,
  NonMonotonicTime,
  SessionReset,
};

struct SupervisorConfig
{
  double awsim_recovery_wait_sec{1.2};
  double stop_speed_mps{0.05};
  double stop_confirm_sec{0.3};
  double clearance_wait_timeout_sec{1.0};
  bool clearance_safe_stop_recovery_enabled{false};
  double safe_stop_clear_confirm_sec{0.5};
  // Simulation-race mode: motion/solver terminal states are paused and then
  // re-evaluated instead of remaining latched for the rest of the race.
  // CoreConfig::simulation_only must also be true.
  bool aggressive_sim_recovery_enabled{false};
  double aggressive_retry_delay_sec{0.5};
  double gear_report_timeout_sec{0.5};
  double gear_command_resend_interval_sec{0.2};
  std::size_t max_gear_command_requests{1U};
  double max_reverse_distance_m{0.8};
  double max_reverse_duration_sec{2.0};
  // The controller begins its calibrated stop sequence at this measured
  // absolute speed. Keep a margin below the competition-operation ceiling to
  // cover command and vehicle latency. Zero is a fail-safe value that prevents
  // ReverseCreep until an explicit limit is configured.
  double max_reverse_speed_mps{0.0};
  // Positive magnitude only. The node-side gear actuation adapter owns the
  // simulator-specific acceleration sign. Zero is the fail-safe default while
  // that sign is unverified.
  double reverse_acceleration_magnitude_mps2{0.0};
  double max_forward_distance_m{0.6};
  double max_forward_duration_sec{1.5};
  double max_forward_speed_mps{0.8};
  double forward_acceleration_magnitude_mps2{0.5};
  double escape_step_distance_m{0.20};
  std::size_t max_escape_steps{3U};
  std::size_t max_attempts{1U};
  double rejoin_speed_limit_mps{1.0};
  double max_rejoin_lateral_error_m{0.5};
  double max_rejoin_heading_error_rad{0.35};
  double rejoin_confirm_sec{0.3};
  double rejoin_timeout_sec{5.0};
  double rejoin_solver_recovery_timeout_sec{1.0};
  bool retry_rejoin_blocked_path{false};
  bool retry_rejoin_timeout{false};
  double cooldown_sec{1.0};
};

struct ReverseActuationCalibration
{
  double drive_acceleration_mps2{};
  double stop_acceleration_mps2{};
  double verified_stop_deceleration_mps2{};
  double control_latency_sec{};
};

bool reverse_actuation_calibration_is_valid(
  const ReverseActuationCalibration & calibration, double acceleration_min_mps2,
  double acceleration_max_mps2) noexcept;

double reverse_stopping_distance_reserve_m(
  const ReverseActuationCalibration & calibration, double signed_speed_mps,
  double control_period_sec) noexcept;

struct RecoveryInput
{
  double now_sec{};
  DetectorDecision detector;
  bool race_active{false};
  bool control_enabled{false};
  bool odometry_valid{false};
  bool solver_healthy{false};
  bool hard_stop_requested{false};
  bool awsim_recovery_settled{false};
  bool awsim_recovery_resolved{false};
  Gear reported_gear{Gear::Unknown};
  bool gear_report_fresh{false};
  ManeuverDirection maneuver_direction{ManeuverDirection::Unknown};
  bool stepwise_escape{false};
  bool step_contact_improved{false};
  bool rear_static_clear{false};
  bool rear_v2x_clear{false};
  bool rear_information_complete{false};
  bool collision_worsening{false};
  bool recovery_escape_confirmed{false};
  bool rejoin_safe{false};
  bool rejoin_forward_clear{true};
  double signed_speed_mps{};
  // Distance of the current bounded maneuver, including the node-side
  // stopping reserve when used for actuation limits.
  double traveled_distance_m{};
  // Distance accumulated across every bounded maneuver in this recovery
  // episode. This prevents a sequence of short steps from being mistaken for
  // a completed escape after each per-step distance reset.
  double episode_traveled_distance_m{};
  double reverse_steering_tire_angle_rad{};
  // Rate-limited tire angle already checked by the adapter's forward swept footprint.
  double rejoin_steering_tire_angle_rad{};
  double lateral_error_m{};
  double heading_error_rad{};
};

struct RecoveryAction
{
  RecoveryActionType type{RecoveryActionType::NormalControl};
  Gear requested_gear{Gear::NoCommand};
  // This field is a non-negative drive magnitude, not a signed AWSIM command.
  double acceleration_magnitude_mps2{};
  double steering_tire_angle_rad{};
  double rejoin_speed_limit_mps{};
  bool inhibit_boost{false};
  bool reset_normal_control{false};
  RecoveryReason reason{RecoveryReason::None};
};

class RecoverySupervisor
{
public:
  explicit RecoverySupervisor(SupervisorConfig config);

  RecoveryAction update(const RecoveryInput & input);
  void reset_session() noexcept;

  [[nodiscard]] RecoveryState state() const noexcept;
  [[nodiscard]] RecoveryReason state_reason() const noexcept;
  [[nodiscard]] std::size_t attempt_count() const noexcept;
  [[nodiscard]] std::size_t escape_step_count() const noexcept;
  [[nodiscard]] bool safe_stop_latched() const noexcept;
  [[nodiscard]] bool drive_report_will_reassess_or_stop() const noexcept;
  [[nodiscard]] const SupervisorConfig & config() const noexcept;

private:
  RecoveryAction update_normal(const RecoveryInput & input);
  RecoveryAction update_suspect(const RecoveryInput & input);
  RecoveryAction update_wait_awsim(const RecoveryInput & input);
  RecoveryAction update_stop_and_confirm(const RecoveryInput & input);
  RecoveryAction update_check_clearance(const RecoveryInput & input);
  RecoveryAction update_wait_for_clear(const RecoveryInput & input);
  RecoveryAction update_wait_reverse_report(const RecoveryInput & input);
  RecoveryAction update_reverse_maneuver(const RecoveryInput & input);
  RecoveryAction update_forward_maneuver(const RecoveryInput & input);
  RecoveryAction update_stop_and_reassess(const RecoveryInput & input);
  RecoveryAction update_stop_before_drive(const RecoveryInput & input);
  RecoveryAction update_wait_drive_report(const RecoveryInput & input);
  RecoveryAction update_low_speed_rejoin(const RecoveryInput & input);
  RecoveryAction update_safe_stop(const RecoveryInput & input);

  void transition(RecoveryState next, RecoveryReason reason, double now_sec) noexcept;
  RecoveryAction normal_action(RecoveryReason reason = RecoveryReason::None) const noexcept;
  RecoveryAction hold_action(RecoveryReason reason) const noexcept;
  RecoveryAction safe_stop_action(RecoveryReason reason) const noexcept;
  RecoveryAction request_gear_action(Gear gear, RecoveryReason reason, double now_sec) noexcept;
  RecoveryAction reverse_action(
    RecoveryReason reason, double steering_tire_angle_rad) const noexcept;
  RecoveryAction forward_action(
    RecoveryReason reason, double steering_tire_angle_rad) const noexcept;
  RecoveryAction rejoin_action(
    RecoveryReason reason, double steering_tire_angle_rad) noexcept;
  RecoveryReason clearance_reason(const RecoveryInput & input) const noexcept;
  bool clearance_is_safe(const RecoveryInput & input) const noexcept;
  bool stopped_confirmed(const RecoveryInput & input) noexcept;
  bool gear_report_is_valid(Gear gear) const noexcept;
  bool input_is_finite(const RecoveryInput & input) const noexcept;
  double state_elapsed(double now_sec) const noexcept;

  SupervisorConfig config_;
  RecoveryState state_{RecoveryState::Normal};
  RecoveryReason state_reason_{RecoveryReason::SessionReset};
  std::size_t attempt_count_{0U};
  std::size_t escape_step_count_{0U};
  std::size_t gear_command_request_count_{0U};
  std::optional<double> state_entered_sec_;
  std::optional<double> last_update_sec_;
  std::optional<double> stopped_since_sec_;
  std::optional<double> aligned_since_sec_;
  std::optional<double> solver_unhealthy_since_sec_;
  std::optional<double> clearance_safe_since_sec_;
  std::optional<double> last_gear_request_sec_;
  std::optional<double> cooldown_until_sec_;
  bool normal_reset_pending_{false};
  bool active_stepwise_escape_{false};
  bool reassess_after_drive_{false};
  bool safe_stop_after_drive_{false};
  bool escape_confirmed_before_drive_{false};
};

enum class ExecutionMode
{
  Disabled,
  Shadow,
  SimulationOnlyBlocked,
  Active,
};

struct CoreConfig
{
  bool enabled{false};
  bool shadow_mode{true};
  bool simulation_only{true};
  DetectorConfig detector;
  SupervisorConfig supervisor;
};

struct CoreInput
{
  bool simulation_environment{false};
  DetectorInput detector;
  RecoveryInput recovery;
};

struct CoreOutput
{
  DetectorDecision detector;
  RecoveryAction action;
  RecoveryState state{RecoveryState::Normal};
  RecoveryReason state_reason{RecoveryReason::None};
  ExecutionMode execution_mode{ExecutionMode::Disabled};
  bool shadow_candidate{false};
  bool actuation_allowed{false};
};

class StuckRecoveryCore
{
public:
  explicit StuckRecoveryCore(CoreConfig config);

  CoreOutput update(const CoreInput & input);
  void reset_session() noexcept;

  [[nodiscard]] ExecutionMode execution_mode(bool simulation_environment) const noexcept;
  [[nodiscard]] const CoreConfig & config() const noexcept;
  [[nodiscard]] const StuckDetector & detector() const noexcept;
  [[nodiscard]] const RecoverySupervisor & supervisor() const noexcept;

private:
  CoreConfig config_;
  StuckDetector detector_;
  RecoverySupervisor supervisor_;
  bool solver_fallback_recovery_episode_{false};
};

const char * to_string(StuckVerdict verdict) noexcept;
const char * to_string(StuckRejectReason reason) noexcept;
const char * to_string(Gear gear) noexcept;
const char * to_string(ManeuverDirection direction) noexcept;
const char * to_string(RecoveryState state) noexcept;
const char * to_string(RecoveryActionType action) noexcept;
const char * to_string(RecoveryReason reason) noexcept;
const char * to_string(ExecutionMode mode) noexcept;

}  // namespace multi_purpose_mpc_ros::stuck_recovery

#endif  // MULTI_PURPOSE_MPC_ROS__STUCK_RECOVERY_CORE_HPP_
