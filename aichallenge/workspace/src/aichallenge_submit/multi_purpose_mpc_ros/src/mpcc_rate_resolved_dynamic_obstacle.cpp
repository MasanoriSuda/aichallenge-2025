#include "multi_purpose_mpc_ros/mpcc_rate_resolved_dynamic_obstacle.hpp"

#include "multi_purpose_mpc_ros/mpcc_rate_resolved.hpp"

#include <cmath>
#include <limits>
#include <utility>

namespace multi_purpose_mpc_ros::mpcc_rate_resolved_dynamic_obstacle
{

const char * to_string(const Reason reason) noexcept
{
  switch (reason) {
    case Reason::NotRequested: return "not-requested";
    case Reason::InvalidInput: return "invalid-input";
    case Reason::NoPredictedEncounter: return "no-predicted-encounter";
    case Reason::Applied: return "applied";
  }
  return "unknown";
}

Result refine(const Request & request) noexcept
{
  namespace model = mpcc_rate_resolved;
  namespace problem = mpcc_rate_resolved_problem;
  Result result;
  if (!request.active) {
    result.reason = Reason::NotRequested;
    return result;
  }
  const int horizon = request.wall_only_problem.horizon_steps;
  const int state_values = model::kStateDimension * (horizon + 1);
  const int variable_count = state_values + model::kInputDimension * horizon;
  if (
    horizon <= 0 || request.pass_side_sign < -1 ||
    request.pass_side_sign > 1 ||
    (request.forced_first_pass_side_stage.has_value() &&
    (request.pass_side_sign == 0 ||
    request.forced_first_pass_side_stage.value() < 0 ||
    request.forced_first_pass_side_stage.value() >= horizon)) ||
    (request.forced_first_ahead_stage.has_value() &&
    (!request.forced_first_pass_side_stage.has_value() ||
    request.forced_first_ahead_stage.value() <=
    request.forced_first_pass_side_stage.value() ||
    request.forced_first_ahead_stage.value() > horizon)) ||
    (request.forced_constraint_fraction.has_value() &&
    (!request.forced_first_pass_side_stage.has_value() ||
    !std::isfinite(request.forced_constraint_fraction.value()) ||
    request.forced_constraint_fraction.value() < 0.0 ||
    request.forced_constraint_fraction.value() > 1.0)) ||
    request.stages.size() != static_cast<std::size_t>(horizon) ||
    request.wall_only_primal.size() != variable_count ||
    !request.wall_only_primal.allFinite() ||
    !std::isfinite(request.separation_tolerance_m) ||
    request.separation_tolerance_m < 0.0)
  {
    result.reason = Reason::InvalidInput;
    return result;
  }
  if (request.constraint_target_problem.has_value()) {
    const auto & target = request.constraint_target_problem.value();
    const auto & witness = request.wall_only_problem;
    const bool compatible =
      target.horizon_steps == horizon &&
      target.linearizations.size() == witness.linearizations.size() &&
      target.state_reference.size() == witness.state_reference.size() &&
      target.state_lower.size() == witness.state_lower.size() &&
      target.state_upper.size() == witness.state_upper.size() &&
      target.state_weight.size() == witness.state_weight.size() &&
      target.input_reference.size() == witness.input_reference.size() &&
      target.input_lower.size() == witness.input_lower.size() &&
      target.input_upper.size() == witness.input_upper.size() &&
      target.input_weight.size() == witness.input_weight.size() &&
      target.additional_linear_cost.size() ==
      witness.additional_linear_cost.size();
    if (!compatible) {
      result.reason = Reason::InvalidInput;
      return result;
    }
  }
  bool any_valid = false;
  for (const auto & stage : request.stages) {
    if (!stage.valid) {
      continue;
    }
    any_valid = true;
    if (
      !std::isfinite(stage.target_progress_m) ||
      !std::isfinite(stage.target_lateral_m) ||
      !std::isfinite(stage.longitudinal_overlap_m) ||
      stage.longitudinal_overlap_m < 0.0 ||
      !std::isfinite(stage.lateral_center_separation_m) ||
      stage.lateral_center_separation_m < 0.0)
    {
      result.reason = Reason::InvalidInput;
      return result;
    }
  }
  if (!any_valid) {
    result.reason = Reason::NoPredictedEncounter;
    return result;
  }

  // Cruise/Follow does not own a tactical side.  It may nevertheless already
  // have one physically coherent wall-only homotopy.  Preserve that homotopy
  // instead of converting a side-by-side target into an impossible
  // stay-behind constraint.  If the wall-only path is still behind, or is not
  // coherently separated on one side for every valid stage, longitudinal
  // stay-behind remains the only convex branch this refinement can prove.
  int resolved_side_sign = request.pass_side_sign;
  bool preserve_current_side = false;
  bool partial_side_escape = false;
  double initial_signed_side_separation_m = 0.0;
  if (resolved_side_sign == 0) {
    bool behind_every_stage = true;
    bool positive_side_every_stage = true;
    bool negative_side_every_stage = true;
    bool stay_behind_box_feasible_every_stage = true;
    bool positive_side_box_feasible_every_stage = true;
    bool negative_side_box_feasible_every_stage = true;
    const bool state_box_available =
      request.wall_only_problem.state_lower.size() == state_values &&
      request.wall_only_problem.state_upper.size() == state_values;
    int first_valid_stage = -1;
    bool first_stage_positive_separated = false;
    bool first_stage_negative_separated = false;
    double initial_relative_lateral_m = 0.0;
    for (int stage = 0; stage < horizon; ++stage) {
      const auto & prediction = request.stages[static_cast<std::size_t>(stage)];
      if (!prediction.valid) {
        continue;
      }
      const int state = (stage + 1) * model::kStateDimension;
      const double progress =
        request.wall_only_primal[state + model::kProgressIndex];
      const double effective_progress = progress +
        request.wall_only_primal[state + model::kLagIndex];
      const double lateral =
        request.wall_only_primal[state + model::kLateralIndex];
      if (first_valid_stage < 0) {
        first_valid_stage = stage;
        initial_relative_lateral_m =
          request.wall_only_primal[model::kLateralIndex] -
          prediction.target_lateral_m;
        result.first_valid_stage = stage;
        result.first_wall_only_progress_m = progress;
        result.first_wall_only_effective_progress_m = effective_progress;
        result.first_wall_only_lateral_m = lateral;
        result.first_target_progress_m = prediction.target_progress_m;
        result.first_target_lateral_m = prediction.target_lateral_m;
        result.first_stay_behind_margin_m =
          prediction.target_progress_m - prediction.longitudinal_overlap_m -
          effective_progress;
        result.first_positive_side_margin_m =
          lateral - (
          prediction.target_lateral_m +
          prediction.lateral_center_separation_m);
        result.first_negative_side_margin_m =
          prediction.target_lateral_m -
          prediction.lateral_center_separation_m - lateral;
        first_stage_positive_separated =
          lateral + request.separation_tolerance_m >=
          prediction.target_lateral_m + prediction.lateral_center_separation_m;
        first_stage_negative_separated =
          lateral - request.separation_tolerance_m <=
          prediction.target_lateral_m - prediction.lateral_center_separation_m;
      }
      behind_every_stage = behind_every_stage &&
        effective_progress <= prediction.target_progress_m -
        prediction.longitudinal_overlap_m + request.separation_tolerance_m;
      positive_side_every_stage = positive_side_every_stage &&
        lateral + request.separation_tolerance_m >=
        prediction.target_lateral_m + prediction.lateral_center_separation_m;
      negative_side_every_stage = negative_side_every_stage &&
        lateral - request.separation_tolerance_m <=
        prediction.target_lateral_m - prediction.lateral_center_separation_m;
      if (state_box_available) {
        const double stay_behind_upper =
          prediction.target_progress_m - prediction.longitudinal_overlap_m;
        const double positive_side_lower =
          prediction.target_lateral_m +
          prediction.lateral_center_separation_m;
        const double negative_side_upper =
          prediction.target_lateral_m -
          prediction.lateral_center_separation_m;
        const double minimum_effective_progress =
          request.wall_only_problem.state_lower[
          state + model::kProgressIndex] +
          request.wall_only_problem.state_lower[state + model::kLagIndex];
        stay_behind_box_feasible_every_stage =
          stay_behind_box_feasible_every_stage &&
          !std::isnan(minimum_effective_progress) &&
          stay_behind_upper + request.separation_tolerance_m >=
          minimum_effective_progress;
        positive_side_box_feasible_every_stage =
          positive_side_box_feasible_every_stage &&
          positive_side_lower - request.separation_tolerance_m <=
          request.wall_only_problem.state_upper[
          state + model::kLateralIndex];
        negative_side_box_feasible_every_stage =
          negative_side_box_feasible_every_stage &&
          negative_side_upper + request.separation_tolerance_m >=
          request.wall_only_problem.state_lower[
          state + model::kLateralIndex];
      }
    }
    if (!behind_every_stage) {
      // Once ego is already body-separated on one side, that current
      // homotopy is the physical state of the world rather than a tactical
      // preference.  Preserve it across the horizon even when the nominal
      // racing line would cross the peer later.  Otherwise a side/rear peer
      // is converted to an impossible stay-behind branch exactly while the
      // canonical path is trying to merge through it.
      if (first_stage_positive_separated && !first_stage_negative_separated) {
        resolved_side_sign = 1;
        preserve_current_side = true;
      } else if (first_stage_negative_separated && !first_stage_positive_separated) {
        resolved_side_sign = -1;
        preserve_current_side = true;
      } else if (positive_side_every_stage && !negative_side_every_stage) {
        resolved_side_sign = 1;
      } else if (negative_side_every_stage && !positive_side_every_stage) {
        resolved_side_sign = -1;
      } else if (
        state_box_available &&
        !stay_behind_box_feasible_every_stage &&
        initial_relative_lateral_m > request.separation_tolerance_m &&
        positive_side_box_feasible_every_stage)
      {
        // The current relative side is already the physical homotopy, even
        // though the conservative footprint has not quite cleared yet.  If
        // stay-behind has an empty intersection with the state box, emitting
        // that row can only create an infeasible QP.  Complete separation on
        // the current side instead; dynamics and the same wall box still
        // decide whether that motion is reachable.
        resolved_side_sign = 1;
        preserve_current_side = true;
        partial_side_escape = true;
        initial_signed_side_separation_m = initial_relative_lateral_m;
      } else if (
        state_box_available &&
        !stay_behind_box_feasible_every_stage &&
        initial_relative_lateral_m < -request.separation_tolerance_m &&
        negative_side_box_feasible_every_stage)
      {
        resolved_side_sign = -1;
        preserve_current_side = true;
        partial_side_escape = true;
        initial_signed_side_separation_m = -initial_relative_lateral_m;
      }
    }
  }

  // An explicit tactical side does not execute the automatic-side block
  // above.  Still capture the same first-stage relation so runtime evidence
  // compares every disjunct under one diagnostic contract.
  if (result.first_valid_stage < 0) {
    for (int stage = 0; stage < horizon; ++stage) {
      const auto & prediction = request.stages[static_cast<std::size_t>(stage)];
      if (!prediction.valid) {
        continue;
      }
      const int state = (stage + 1) * model::kStateDimension;
      const double progress =
        request.wall_only_primal[state + model::kProgressIndex];
      const double effective_progress = progress +
        request.wall_only_primal[state + model::kLagIndex];
      const double lateral =
        request.wall_only_primal[state + model::kLateralIndex];
      result.first_valid_stage = stage;
      result.first_wall_only_progress_m = progress;
      result.first_wall_only_effective_progress_m = effective_progress;
      result.first_wall_only_lateral_m = lateral;
      result.first_target_progress_m = prediction.target_progress_m;
      result.first_target_lateral_m = prediction.target_lateral_m;
      result.first_stay_behind_margin_m =
        prediction.target_progress_m - prediction.longitudinal_overlap_m -
        effective_progress;
      result.first_positive_side_margin_m =
        lateral - (
        prediction.target_lateral_m +
        prediction.lateral_center_separation_m);
      result.first_negative_side_margin_m =
        prediction.target_lateral_m -
        prediction.lateral_center_separation_m - lateral;
      break;
    }
  }

  // An explicitly selected tactical side must preserve separation which the
  // current physical state has already acquired.  The future wall-only
  // witness is allowed to cross the peer because it does not contain the
  // dynamic obstacle yet; using that future crossing to demote the current
  // state back to stay-behind discards the selected homotopy and creates the
  // Pass inward-drift observed before SafetyBrake.  This is deliberately
  // based on stage-zero physical state, not on one separated middle sample.
  if (
    !preserve_current_side && resolved_side_sign != 0 &&
    result.first_valid_stage >= 0)
  {
    const auto & first_prediction = request.stages[
      static_cast<std::size_t>(result.first_valid_stage)];
    initial_signed_side_separation_m =
      static_cast<double>(resolved_side_sign) *
      (request.wall_only_primal[model::kLateralIndex] -
      first_prediction.target_lateral_m);
    if (
      initial_signed_side_separation_m + request.separation_tolerance_m >=
      first_prediction.lateral_center_separation_m)
    {
      preserve_current_side = true;
    }
  }

  // A pass-side switch is accepted only when separation is sustained for the
  // remainder of the valid prediction suffix.  One noisy separated sample
  // must not alternate the disjunctive branch stage by stage.
  int first_pass_side_stage = -1;
  if (preserve_current_side) {
    for (int stage = 0; stage < horizon; ++stage) {
      if (request.stages[static_cast<std::size_t>(stage)].valid) {
        first_pass_side_stage = stage;
        break;
      }
    }
  } else if (resolved_side_sign != 0) {
    bool suffix_separated = true;
    for (int stage = horizon - 1; stage >= 0; --stage) {
      const auto & prediction = request.stages[static_cast<std::size_t>(stage)];
      if (!prediction.valid) {
        continue;
      }
      const int state = (stage + 1) * model::kStateDimension;
      const double lateral =
        request.wall_only_primal[state + model::kLateralIndex];
      const double boundary = prediction.target_lateral_m +
        static_cast<double>(resolved_side_sign) *
        prediction.lateral_center_separation_m;
      const bool separated = resolved_side_sign > 0 ?
        lateral + request.separation_tolerance_m >= boundary :
        lateral - request.separation_tolerance_m <= boundary;
      suffix_separated = suffix_separated && separated;
      if (suffix_separated) {
        first_pass_side_stage = stage;
      }
    }
  }

  // A selected tactical side can meet the same initial-overlap condition as
  // automatic Cruise/Follow.  If a pre-pass stay-behind row has an empty
  // intersection with the canonical state box, retain the current physical
  // side and require a non-worsening, wall-only-demonstrated escape envelope
  // instead of demanding complete lateral separation in one stage.
  if (!partial_side_escape && resolved_side_sign != 0) {
    const bool state_box_available =
      request.wall_only_problem.state_lower.size() == state_values &&
      request.wall_only_problem.state_upper.size() == state_values;
    int first_valid_stage = -1;
    double first_target_lateral_m = 0.0;
    bool prepass_stay_behind_box_feasible = true;
    bool selected_side_box_feasible = true;
    for (int stage = 0; stage < horizon; ++stage) {
      const auto & prediction = request.stages[static_cast<std::size_t>(stage)];
      if (!prediction.valid) {
        continue;
      }
      if (first_valid_stage < 0) {
        first_valid_stage = stage;
        first_target_lateral_m = prediction.target_lateral_m;
      }
      if (!state_box_available) {
        continue;
      }
      const int state = (stage + 1) * model::kStateDimension;
      if (first_pass_side_stage < 0 || stage < first_pass_side_stage) {
        const double stay_behind_upper =
          prediction.target_progress_m - prediction.longitudinal_overlap_m;
        const double minimum_effective_progress =
          request.wall_only_problem.state_lower[
          state + model::kProgressIndex] +
          request.wall_only_problem.state_lower[state + model::kLagIndex];
        prepass_stay_behind_box_feasible =
          prepass_stay_behind_box_feasible &&
          !std::isnan(minimum_effective_progress) &&
          stay_behind_upper + request.separation_tolerance_m >=
          minimum_effective_progress;
      }
      if (resolved_side_sign > 0) {
        const double side_lower = prediction.target_lateral_m +
          prediction.lateral_center_separation_m;
        selected_side_box_feasible = selected_side_box_feasible &&
          side_lower - request.separation_tolerance_m <=
          request.wall_only_problem.state_upper[
          state + model::kLateralIndex];
      } else {
        const double side_upper = prediction.target_lateral_m -
          prediction.lateral_center_separation_m;
        selected_side_box_feasible = selected_side_box_feasible &&
          side_upper + request.separation_tolerance_m >=
          request.wall_only_problem.state_lower[
          state + model::kLateralIndex];
      }
    }
    if (first_valid_stage >= 0 && state_box_available) {
      initial_signed_side_separation_m =
        static_cast<double>(resolved_side_sign) *
        (request.wall_only_primal[model::kLateralIndex] -
        first_target_lateral_m);
      if (
        !prepass_stay_behind_box_feasible && selected_side_box_feasible &&
        initial_signed_side_separation_m > request.separation_tolerance_m)
      {
        partial_side_escape = true;
        preserve_current_side = true;
        first_pass_side_stage = first_valid_stage;
      }
    }
  }

  if (request.forced_first_pass_side_stage.has_value()) {
    // Candidate C chooses the disjunct sequence before SQP refinement.  It
    // may not borrow a partial lateral witness: every stage must prove either
    // complete longitudinal-behind or complete selected-side separation.
    first_pass_side_stage = request.forced_first_pass_side_stage.value();
    partial_side_escape = false;
    result.forced_transition_applied = true;
  }
  const double forced_constraint_fraction =
    request.forced_constraint_fraction.value_or(1.0);
  result.forced_constraint_fraction = forced_constraint_fraction;

  auto refined = request.constraint_target_problem.has_value() ?
    request.constraint_target_problem.value() : request.wall_only_problem;
  refined.dynamic_obstacle_constraints.clear();
  refined.dynamic_obstacle_constraints.reserve(request.stages.size());
  for (int stage = 0; stage < horizon; ++stage) {
    const auto & prediction = request.stages[static_cast<std::size_t>(stage)];
    if (!prediction.valid) {
      continue;
    }
    problem::DynamicObstacleConstraint constraint;
    constraint.state_stage = stage + 1;
    const bool forced_ahead =
      request.forced_first_ahead_stage.has_value() &&
      stage >= request.forced_first_ahead_stage.value();
    if (forced_ahead) {
      constraint.axis =
        problem::DynamicObstacleConstraintAxis::EffectiveProgress;
      const int state = (stage + 1) * model::kStateDimension;
      const double witness_effective_progress_m =
        request.wall_only_primal[state + model::kProgressIndex] +
        request.wall_only_primal[state + model::kLagIndex];
      const double final_lower_m =
        prediction.target_progress_m + prediction.longitudinal_overlap_m;
      constraint.lower = witness_effective_progress_m +
        forced_constraint_fraction *
        (final_lower_m - witness_effective_progress_m);
      ++result.ahead_row_count;
    } else if (first_pass_side_stage >= 0 && stage >= first_pass_side_stage) {
      constraint.axis = problem::DynamicObstacleConstraintAxis::Lateral;
      double required_signed_separation_m =
        prediction.lateral_center_separation_m;
      if (partial_side_escape) {
        const int state = (stage + 1) * model::kStateDimension;
        const double wall_only_signed_separation_m =
          static_cast<double>(resolved_side_sign) *
          (request.wall_only_primal[state + model::kLateralIndex] -
          prediction.target_lateral_m);
        // The wall-only solution is the reachability witness for this
        // convexification.  Do not silently strengthen it with the measured
        // stage-zero separation: steering and yaw-response lag can make a
        // short, bounded decrease unavoidable even though the certified
        // trajectory subsequently separates.  Requiring an instantaneous
        // non-decrease creates a hard row which neither the witness nor the
        // physical model can satisfy.  Full separation remains mandatory as
        // soon as the demonstrated trajectory reaches it.
        required_signed_separation_m = std::min(
          prediction.lateral_center_separation_m,
          wall_only_signed_separation_m);
        if (
          required_signed_separation_m + request.separation_tolerance_m <
          prediction.lateral_center_separation_m)
        {
          ++result.partial_escape_row_count;
        }
      }
      const double boundary = prediction.target_lateral_m +
        static_cast<double>(resolved_side_sign) *
        required_signed_separation_m;
      const int state = (stage + 1) * model::kStateDimension;
      const double witness_lateral_m =
        request.wall_only_primal[state + model::kLateralIndex];
      const double continued_boundary = witness_lateral_m +
        forced_constraint_fraction * (boundary - witness_lateral_m);
      if (resolved_side_sign > 0) {
        constraint.lower = continued_boundary;
      } else {
        constraint.upper = continued_boundary;
      }
      ++result.pass_side_row_count;
    } else {
      constraint.axis =
        problem::DynamicObstacleConstraintAxis::EffectiveProgress;
      const int state = (stage + 1) * model::kStateDimension;
      const double witness_effective_progress_m =
        request.wall_only_primal[state + model::kProgressIndex] +
        request.wall_only_primal[state + model::kLagIndex];
      const double final_upper_m =
        prediction.target_progress_m - prediction.longitudinal_overlap_m;
      constraint.upper = witness_effective_progress_m +
        forced_constraint_fraction *
        (final_upper_m - witness_effective_progress_m);
      ++result.stay_behind_row_count;
    }
    refined.dynamic_obstacle_constraints.push_back(constraint);
  }
  if (refined.dynamic_obstacle_constraints.empty()) {
    result.reason = Reason::NoPredictedEncounter;
    return result;
  }
  result.reason = Reason::Applied;
  result.applied = true;
  result.resolved_side_sign = resolved_side_sign;
  result.first_pass_side_stage = first_pass_side_stage;
  result.problem = std::move(refined);
  return result;
}

}  // namespace multi_purpose_mpc_ros::mpcc_rate_resolved_dynamic_obstacle
