#include "multi_purpose_mpc_ros/mpcc_overtake_sibling_adoption.hpp"

#include <cmath>

namespace multi_purpose_mpc_ros::mpcc_overtake_sibling_adoption {
namespace {

bool active_intent(const contract::ControlIntent intent) noexcept {
  return intent == contract::ControlIntent::ShiftOut ||
         intent == contract::ControlIntent::Pass;
}

bool side_valid(const int side_sign) noexcept {
  return side_sign == -1 || side_sign == 1;
}

bool same_epoch_except_side(const artifact::Identity &source,
                            const artifact::Identity &sibling) noexcept {
  if (source.sequence != sibling.sequence ||
      source.snapshot_sec != sibling.snapshot_sec) {
    return false;
  }
  const auto &expected = source.source_context;
  const auto &actual = sibling.source_context;
  return actual.decision_id == expected.decision_id &&
         actual.intent == expected.intent &&
         actual.intent_generation == expected.intent_generation &&
         actual.observation_generation == expected.observation_generation &&
         actual.stage_geometry_id == expected.stage_geometry_id &&
         actual.target_obstacle_generation ==
             expected.target_obstacle_generation &&
         actual.target_id == expected.target_id &&
         actual.dynamic_obstacle_constraint_active ==
             expected.dynamic_obstacle_constraint_active &&
         actual.dynamic_obstacle_generation ==
             expected.dynamic_obstacle_generation &&
         actual.dynamic_obstacle_id == expected.dynamic_obstacle_id &&
         actual.horizon_steps == expected.horizon_steps &&
         actual.formulation == expected.formulation &&
         actual.state_schema_id == expected.state_schema_id &&
         actual.input_schema_id == expected.input_schema_id &&
         actual.bounds_schema_id == expected.bounds_schema_id &&
         actual.cost_schema_id == expected.cost_schema_id;
}

} // namespace

const char *to_string(const Reason reason) noexcept {
  switch (reason) {
  case Reason::Accepted:
    return "accepted";
  case Reason::SelectedCurrentWorldAuthorityAvailable:
    return "selected-current-world-authority-available";
  case Reason::InactiveExecution:
    return "inactive-execution";
  case Reason::InvalidLiveIdentity:
    return "invalid-live-identity";
  case Reason::HardFault:
    return "hard-fault";
  case Reason::SelectedHomotopyEstablished:
    return "selected-homotopy-established";
  case Reason::NoReturn:
    return "no-return";
  case Reason::ReplacementBudgetExhausted:
    return "replacement-budget-exhausted";
  case Reason::MissingSiblingAuthority:
    return "missing-sibling-authority";
  case Reason::NonStatelessSibling:
    return "non-stateless-sibling";
  case Reason::InvalidSourceIdentity:
    return "invalid-source-identity";
  case Reason::InvalidSiblingIdentity:
    return "invalid-sibling-identity";
  case Reason::SourceLiveMismatch:
    return "source-live-mismatch";
  case Reason::SiblingEpochMismatch:
    return "sibling-epoch-mismatch";
  case Reason::SiblingSideMismatch:
    return "sibling-side-mismatch";
  }
  return "unknown";
}

Resolution resolve(const Request &request) noexcept {
  Resolution result;
  if (request.selected_current_world_authority_available) {
    result.reason = Reason::SelectedCurrentWorldAuthorityAvailable;
    return result;
  }
  if (!request.active_execution || !active_intent(request.live_intent)) {
    result.reason = Reason::InactiveExecution;
    return result;
  }
  if (request.live_target_id.empty() || request.live_mission_generation == 0U ||
      !side_valid(request.live_side_sign)) {
    result.reason = Reason::InvalidLiveIdentity;
    return result;
  }
  if (request.hard_fault) {
    result.reason = Reason::HardFault;
    return result;
  }
  if (request.selected_homotopy_established) {
    result.reason = Reason::SelectedHomotopyEstablished;
    return result;
  }
  if (!request.before_no_return) {
    result.reason = Reason::NoReturn;
    return result;
  }
  if (!request.replacement_budget_available) {
    result.reason = Reason::ReplacementBudgetExhausted;
    return result;
  }
  if (!request.sibling_current_world_authority) {
    result.reason = Reason::MissingSiblingAuthority;
    return result;
  }
  if (!request.sibling_stateless_current_world_bundle) {
    result.reason = Reason::NonStatelessSibling;
    return result;
  }
  if (!artifact::identity_valid(request.source_identity)) {
    result.reason = Reason::InvalidSourceIdentity;
    return result;
  }
  if (!artifact::identity_valid(request.sibling_identity)) {
    result.reason = Reason::InvalidSiblingIdentity;
    return result;
  }
  const auto &source = request.source_identity.source_context;
  if (source.intent != request.live_intent ||
      source.target_id != request.live_target_id ||
      source.intent_generation != request.live_mission_generation ||
      source.execution_side_sign != request.live_side_sign) {
    result.reason = Reason::SourceLiveMismatch;
    return result;
  }
  if (!same_epoch_except_side(request.source_identity,
                              request.sibling_identity)) {
    result.reason = Reason::SiblingEpochMismatch;
    return result;
  }
  const auto &sibling = request.sibling_identity.source_context;
  if (sibling.execution_side_sign != -request.live_side_sign ||
      sibling.dynamic_obstacle_side_sign != -request.live_side_sign) {
    result.reason = Reason::SiblingSideMismatch;
    return result;
  }

  result.accepted = true;
  result.reason = Reason::Accepted;
  result.token = Token{request.source_identity.sequence,
                       request.source_identity.snapshot_sec,
                       request.live_intent,
                       request.live_target_id,
                       request.live_mission_generation,
                       request.live_side_sign,
                       sibling.execution_side_sign};
  return result;
}

bool token_matches_live_state(
    const Token &token, const contract::ControlIntent live_intent,
    const std::string &live_target_id,
    const std::uint64_t live_mission_generation, const int live_side_sign,
    const bool active_execution, const bool selected_homotopy_established,
    const bool before_no_return,
    const bool replacement_budget_available, const bool hard_fault) noexcept {
  return token.source_sequence > 0U &&
         std::isfinite(token.source_snapshot_sec) && active_execution &&
         active_intent(live_intent) && live_intent == token.intent &&
         !live_target_id.empty() && live_target_id == token.target_id &&
         live_mission_generation != 0U &&
         live_mission_generation == token.mission_generation &&
         side_valid(live_side_sign) &&
         live_side_sign == token.previous_side_sign &&
         side_valid(token.adopted_side_sign) &&
         token.adopted_side_sign == -live_side_sign &&
         !selected_homotopy_established && before_no_return &&
         replacement_budget_available && !hard_fault;
}

} // namespace multi_purpose_mpc_ros::mpcc_overtake_sibling_adoption
