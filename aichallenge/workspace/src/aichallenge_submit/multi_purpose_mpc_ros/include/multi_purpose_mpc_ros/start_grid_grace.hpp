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
  bool has_side_vehicle{false};
  bool initial_static_target_latched{false};
  double front_speed_mps{0.0};
  double stationary_speed_threshold_mps{0.0};
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

/// Preserve a previously selected side for the whole breakout. Before selection, use a visible
/// grid stagger; when the lateral difference is inside the deadband, leave side selection to the
/// collision-inflated gap planner instead of rejecting the breakout. Invalid geometry fails
/// closed through valid=false.
BreakoutSideDecision resolve_breakout_side(const BreakoutSideContext &context) noexcept;

const char *to_string(Phase phase) noexcept;
const char *to_string(Transition transition) noexcept;

} // namespace multi_purpose_mpc_ros::start_grid_grace

#endif // MULTI_PURPOSE_MPC_ROS__START_GRID_GRACE_HPP_
