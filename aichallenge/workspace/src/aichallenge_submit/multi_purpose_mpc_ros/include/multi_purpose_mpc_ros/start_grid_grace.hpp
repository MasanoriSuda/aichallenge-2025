#ifndef MULTI_PURPOSE_MPC_ROS__START_GRID_GRACE_HPP_
#define MULTI_PURPOSE_MPC_ROS__START_GRID_GRACE_HPP_

#include <optional>

namespace multi_purpose_mpc_ros::start_grid_grace {

enum class Phase {
  Disabled,
  WaitingForStart,
  Prepared,
  Grace,
  Expired,
};

enum class Transition {
  None,
  Prepared,
  Armed,
  Expired,
  Cleared,
  ClockRejected,
};

struct Evaluation {
  Phase phase{Phase::Disabled};
  Transition transition{Transition::None};
  bool active{false};
  double elapsed_sec{0.0};
};

struct StaticStopContext {
  bool grace_active{false};
  bool has_front_vehicle{false};
  bool has_side_vehicle{false};
  bool initial_static_target_latched{false};
  bool emergency_brake_required{false};
  double front_speed_mps{0.0};
  double stationary_speed_threshold_mps{0.0};
  double rollout_speed_threshold_mps{0.0};
};

struct FrontLateralRangeContext {
  bool grace_active{false};
  bool curve_guard_active{false};
  double corridor_lateral_range_m{0.0};
  double danger_lateral_range_m{0.0};
  double curve_lateral_margin_m{0.0};
};

struct BreakoutContext {
  bool enabled{false};
  bool grace_active{false};
  bool has_front_vehicle{false};
  bool initial_static_target_latched{false};
  double front_speed_mps{0.0};
  double stationary_speed_threshold_mps{0.0};
};

struct BreakoutContinuationContext {
  bool grace_active{false};
  bool breakout_target_latched{false};
  bool current_front_matches{false};
  bool active_line{false};
  bool line_target_matches{false};
};

struct BreakoutSideContext {
  double ego_lateral_m{0.0};
  double front_lateral_m{0.0};
  double side_deadband_m{0.0};
  /// Previously selected breakout side. Zero means no side has been selected yet.
  int latched_side{0};
};

struct BreakoutSideDecision {
  bool valid{false};
  /// +1 requires left, -1 requires right, and 0 lets the gap planner try both sides.
  int required_side{0};
};

struct BreakoutGapPreferenceContext {
  bool left_available{false};
  bool right_available{false};
  double left_width_m{0.0};
  double right_width_m{0.0};
  /// Side away from the visible front-target stagger. Zero means no preference.
  int stagger_preferred_side{0};
  int fallback_side{0};
};

struct BreakoutLineContext {
  bool breakout_active{false};
  bool behavior_overtake{false};
  bool gap_available{false};
  bool zone_allows{false};
};

enum class DynamicDecisionAction {
  Observe,
  CommitCandidate,
  NoCandidate,
};

struct DynamicDecisionContext {
  bool enabled{false};
  bool candidate_available{false};
  bool peer_motion_observed{false};
  bool emergency_commit{false};
  double elapsed_sec{0.0};
  double peer_motion_elapsed_sec{0.0};
  double candidate_stable_sec{0.0};
  double motion_observation_sec{0.0};
  double max_observation_sec{0.0};
  double min_candidate_stable_sec{0.0};
};

struct DynamicDecisionResolution {
  DynamicDecisionAction action{DynamicDecisionAction::NoCandidate};
  double remaining_sec{0.0};
};

class Guard {
public:
  explicit Guard(double duration_sec);

  Transition prepare();
  Transition arm(double start_sec);
  Transition clear();
  Evaluation evaluate(double now_sec);

  Phase phase() const noexcept;
  double duration_sec() const noexcept;
  std::optional<double> start_sec() const noexcept;

private:
  double duration_sec_{0.0};
  Phase phase_{Phase::Disabled};
  std::optional<double> start_sec_;
};

bool should_suppress_static_stop(const StaticStopContext &context) noexcept;

/// Keep adjacent start-grid vehicles out of the front collision corridor.
/// The wider curve guard is restored as soon as start-grid grace ends.
double resolve_front_lateral_range(const FrontLateralRangeContext &context);

/// Allow a latched, stationary start-grid vehicle to be evaluated for an immediate
/// side breakout. Passing geometry is intentionally checked by the caller.
bool should_attempt_breakout(const BreakoutContext &context) noexcept;

/// Continue a latched breakout during grace, or after grace only while the same target owns an
/// active ShiftOut/Pass line. This prevents the grace timeout from cancelling a maneuver that is
/// already alongside the grid target without allowing a stale latch to start another maneuver.
bool should_continue_breakout(const BreakoutContinuationContext &context) noexcept;

/// Preserve a previously selected side for the whole breakout. Before selection, always leave
/// both sides to the collision-inflated gap planner; the initial grid stagger is not evidence that
/// its side corridor is open. Invalid geometry fails closed through valid=false.
BreakoutSideDecision resolve_breakout_side(const BreakoutSideContext &context) noexcept;

/// Prefer the side opposite the front target's lateral stagger while still
/// letting the gap planner reject that side when it is not feasible.
int resolve_breakout_stagger_preference(double ego_lateral_m,
                                        double front_lateral_m,
                                        double side_deadband_m) noexcept;

/// Prefer a feasible corridor away from the visible target stagger before a
/// side is locked. Availability wins; without visible stagger, width and then
/// geometric fallback break the tie.
int resolve_breakout_gap_preference(
    const BreakoutGapPreferenceContext &context) noexcept;

/// A validated breakout deliberately passes a close grid target. Preserve its explicit line
/// through the front-risk metric; an unavailable gap or blocked execution zone still fails closed.
bool should_preserve_breakout_line(const BreakoutLineContext &context) noexcept;

/// Keep accelerating on the base trajectory while peer motion makes the best start corridor
/// observable. A stable candidate commits after the motion window; the bounded maximum wait
/// prevents an indefinitely undecided launch.
DynamicDecisionResolution resolve_dynamic_breakout_decision(
    const DynamicDecisionContext &context);

const char *to_string(Phase phase) noexcept;
const char *to_string(Transition transition) noexcept;
const char *to_string(DynamicDecisionAction action) noexcept;

} // namespace multi_purpose_mpc_ros::start_grid_grace

#endif // MULTI_PURPOSE_MPC_ROS__START_GRID_GRACE_HPP_
