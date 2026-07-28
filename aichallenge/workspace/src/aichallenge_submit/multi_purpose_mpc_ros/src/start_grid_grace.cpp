#include <multi_purpose_mpc_ros/start_grid_grace.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace multi_purpose_mpc_ros::start_grid_grace {

Guard::Guard(const double duration_sec)
    : duration_sec_(duration_sec),
      phase_(duration_sec > 0.0 ? Phase::WaitingForStart : Phase::Disabled) {
  if (!std::isfinite(duration_sec) || duration_sec < 0.0) {
    throw std::invalid_argument(
        "start-grid grace duration must be finite and non-negative");
  }
}

Transition Guard::arm(const double start_sec) {
  if (phase_ != Phase::WaitingForStart && phase_ != Phase::Prepared) {
    return Transition::None;
  }
  if (!std::isfinite(start_sec)) {
    phase_ = Phase::Expired;
    start_sec_.reset();
    return Transition::ClockRejected;
  }
  start_sec_ = start_sec;
  phase_ = Phase::Grace;
  return Transition::Armed;
}

Transition Guard::prepare() {
  if (phase_ != Phase::WaitingForStart) {
    return Transition::None;
  }
  phase_ = Phase::Prepared;
  return Transition::Prepared;
}

Transition Guard::clear() {
  if (phase_ == Phase::Disabled) {
    return Transition::None;
  }
  const bool changed =
      phase_ != Phase::WaitingForStart || start_sec_.has_value();
  phase_ = Phase::WaitingForStart;
  start_sec_.reset();
  return changed ? Transition::Cleared : Transition::None;
}

Evaluation Guard::evaluate(const double now_sec) {
  Evaluation evaluation;
  evaluation.phase = phase_;
  if (phase_ == Phase::Prepared) {
    evaluation.active = true;
    return evaluation;
  }
  if (phase_ != Phase::Grace) {
    return evaluation;
  }
  if (!start_sec_.has_value() || !std::isfinite(now_sec) ||
      now_sec < start_sec_.value()) {
    phase_ = Phase::Expired;
    start_sec_.reset();
    evaluation.phase = phase_;
    evaluation.transition = Transition::ClockRejected;
    return evaluation;
  }

  evaluation.elapsed_sec = now_sec - start_sec_.value();
  if (evaluation.elapsed_sec >= duration_sec_) {
    phase_ = Phase::Expired;
    evaluation.phase = phase_;
    evaluation.transition = Transition::Expired;
    return evaluation;
  }

  evaluation.active = true;
  return evaluation;
}

Phase Guard::phase() const noexcept { return phase_; }

double Guard::duration_sec() const noexcept { return duration_sec_; }

std::optional<double> Guard::start_sec() const noexcept { return start_sec_; }

bool should_suppress_static_stop(const StaticStopContext &context) noexcept {
  return context.grace_active && context.has_front_vehicle &&
         context.has_side_vehicle && !context.emergency_brake_required &&
         std::isfinite(context.front_speed_mps) &&
         context.front_speed_mps >= 0.0 &&
         std::isfinite(context.stationary_speed_threshold_mps) &&
         context.stationary_speed_threshold_mps >= 0.0 &&
         std::isfinite(context.rollout_speed_threshold_mps) &&
         context.rollout_speed_threshold_mps >=
             context.stationary_speed_threshold_mps &&
         context.initial_static_target_latched &&
         context.front_speed_mps <= context.rollout_speed_threshold_mps;
}

bool should_suppress_coordinated_recovery(
    const CoordinatedRecoveryContext &context) noexcept {
  return context.grace_active || context.dynamic_observation_active ||
         context.breakout_active;
}

double resolve_front_lateral_range(const FrontLateralRangeContext &context) {
  if (!std::isfinite(context.corridor_lateral_range_m) ||
      context.corridor_lateral_range_m < 0.0 ||
      !std::isfinite(context.danger_lateral_range_m) ||
      context.danger_lateral_range_m < 0.0 ||
      !std::isfinite(context.curve_lateral_margin_m) ||
      context.curve_lateral_margin_m < 0.0) {
    throw std::invalid_argument("invalid front lateral range context");
  }

  if (context.grace_active || !context.curve_guard_active) {
    return context.danger_lateral_range_m;
  }
  return std::min(context.corridor_lateral_range_m,
                  context.danger_lateral_range_m +
                      context.curve_lateral_margin_m);
}

bool should_attempt_breakout(const BreakoutContext &context) noexcept {
  return context.enabled && context.grace_active && context.has_front_vehicle &&
         context.initial_static_target_latched && std::isfinite(context.front_speed_mps) &&
         context.front_speed_mps >= 0.0 &&
         std::isfinite(context.stationary_speed_threshold_mps) &&
         context.stationary_speed_threshold_mps >= 0.0 &&
         context.front_speed_mps <= context.stationary_speed_threshold_mps;
}

bool should_continue_breakout(const BreakoutContinuationContext &context) noexcept {
  if (!context.breakout_target_latched) {
    return false;
  }

  // Before a line is committed, only the currently detected front target may
  // use the start-grid exception. Once the same target owns an active
  // ShiftOut/Pass line, however, lateral separation intentionally removes it
  // from the generic front-overlap set. Keep the latched line in that case so
  // a hard-curve lookahead cannot cancel the pass merely because
  // current_front_matches became false.
  return (context.grace_active && context.current_front_matches) ||
         (context.active_line && context.line_target_matches);
}

BreakoutSideDecision resolve_breakout_side(const BreakoutSideContext &context) noexcept {
  if (!std::isfinite(context.side_deadband_m) || context.side_deadband_m < 0.0 ||
      context.latched_side < -1 || context.latched_side > 1) {
    return {};
  }
  if (context.latched_side != 0) {
    return BreakoutSideDecision{true, context.latched_side};
  }
  if (!std::isfinite(context.ego_lateral_m) ||
      !std::isfinite(context.front_lateral_m)) {
    return {};
  }
  return BreakoutSideDecision{true, 0};
}

int resolve_breakout_stagger_preference(const double ego_lateral_m,
                                        const double front_lateral_m,
                                        const double side_deadband_m) noexcept {
  if (!std::isfinite(ego_lateral_m) || !std::isfinite(front_lateral_m) ||
      !std::isfinite(side_deadband_m) || side_deadband_m < 0.0) {
    return 0;
  }
  const double target_relative_lateral = front_lateral_m - ego_lateral_m;
  if (target_relative_lateral > side_deadband_m) {
    return -1;
  }
  if (target_relative_lateral < -side_deadband_m) {
    return 1;
  }
  return 0;
}

int resolve_breakout_gap_preference(
    const BreakoutGapPreferenceContext &context) noexcept {
  if (context.left_available != context.right_available) {
    return context.left_available ? 1 : -1;
  }
  if (context.left_available && context.right_available &&
      (context.stagger_preferred_side == -1 ||
       context.stagger_preferred_side == 1)) {
    return context.stagger_preferred_side;
  }
  if (context.left_available && context.right_available &&
      std::isfinite(context.left_width_m) &&
      std::isfinite(context.right_width_m)) {
    constexpr double kWidthTieTolerance = 1e-3;
    const double width_delta = context.left_width_m - context.right_width_m;
    if (std::abs(width_delta) > kWidthTieTolerance) {
      return width_delta > 0.0 ? 1 : -1;
    }
  }
  return context.fallback_side >= -1 && context.fallback_side <= 1
             ? context.fallback_side
             : 0;
}

bool should_preserve_breakout_line(const BreakoutLineContext &context) noexcept {
  return context.breakout_active && context.behavior_overtake &&
         context.gap_available && context.zone_allows;
}

DynamicDecisionResolution resolve_dynamic_breakout_decision(
    const DynamicDecisionContext &context) {
  const auto finite_non_negative = [](const double value) {
    return std::isfinite(value) && value >= 0.0;
  };
  if (!finite_non_negative(context.elapsed_sec) ||
      !finite_non_negative(context.peer_motion_elapsed_sec) ||
      !finite_non_negative(context.candidate_stable_sec) ||
      !finite_non_negative(context.motion_observation_sec) ||
      !finite_non_negative(context.max_observation_sec) ||
      !finite_non_negative(context.min_candidate_stable_sec)) {
    throw std::invalid_argument("invalid start-grid dynamic decision timing");
  }

  if (!context.enabled) {
    return {context.candidate_available
                ? DynamicDecisionAction::CommitCandidate
                : DynamicDecisionAction::NoCandidate,
            0.0};
  }
  if (context.emergency_commit) {
    return {context.candidate_available
                ? DynamicDecisionAction::CommitCandidate
                : DynamicDecisionAction::NoCandidate,
            0.0};
  }

  const bool maximum_wait_elapsed =
      context.elapsed_sec >= context.max_observation_sec;
  const bool motion_window_elapsed =
      context.peer_motion_observed &&
      context.peer_motion_elapsed_sec >= context.motion_observation_sec;
  const bool candidate_stable =
      context.candidate_available &&
      context.candidate_stable_sec >= context.min_candidate_stable_sec;
  if (context.candidate_available &&
      (maximum_wait_elapsed || (motion_window_elapsed && candidate_stable))) {
    return {DynamicDecisionAction::CommitCandidate, 0.0};
  }
  if (maximum_wait_elapsed) {
    return {DynamicDecisionAction::NoCandidate, 0.0};
  }

  const double maximum_remaining_sec =
      context.max_observation_sec - context.elapsed_sec;
  double remaining_sec = maximum_remaining_sec;
  if (context.peer_motion_observed) {
    double required_remaining_sec = std::max(
        0.0, context.motion_observation_sec -
                 context.peer_motion_elapsed_sec);
    if (context.candidate_available) {
      required_remaining_sec = std::max(
          required_remaining_sec,
          std::max(0.0, context.min_candidate_stable_sec -
                                context.candidate_stable_sec));
    }
    remaining_sec = std::min(maximum_remaining_sec, required_remaining_sec);
  }
  return {DynamicDecisionAction::Observe, std::max(0.0, remaining_sec)};
}

const char *to_string(const Phase phase) noexcept {
  switch (phase) {
  case Phase::Disabled:
    return "Disabled";
  case Phase::WaitingForStart:
    return "WaitingForStart";
  case Phase::Prepared:
    return "Prepared";
  case Phase::Grace:
    return "Grace";
  case Phase::Expired:
    return "Expired";
  }
  return "Unknown";
}

const char *to_string(const Transition transition) noexcept {
  switch (transition) {
  case Transition::None:
    return "None";
  case Transition::Prepared:
    return "Prepared";
  case Transition::Armed:
    return "Armed";
  case Transition::Expired:
    return "Expired";
  case Transition::Cleared:
    return "Cleared";
  case Transition::ClockRejected:
    return "ClockRejected";
  }
  return "Unknown";
}

const char *to_string(const DynamicDecisionAction action) noexcept {
  switch (action) {
  case DynamicDecisionAction::Observe:
    return "Observe";
  case DynamicDecisionAction::CommitCandidate:
    return "CommitCandidate";
  case DynamicDecisionAction::NoCandidate:
    return "NoCandidate";
  }
  return "Unknown";
}

} // namespace multi_purpose_mpc_ros::start_grid_grace
