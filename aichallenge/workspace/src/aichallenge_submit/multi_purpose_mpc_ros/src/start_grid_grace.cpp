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

} // namespace multi_purpose_mpc_ros::start_grid_grace
