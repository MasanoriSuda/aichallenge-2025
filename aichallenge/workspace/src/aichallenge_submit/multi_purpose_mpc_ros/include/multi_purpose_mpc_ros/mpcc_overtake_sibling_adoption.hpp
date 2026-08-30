#ifndef MULTI_PURPOSE_MPC_ROS__MPCC_OVERTAKE_SIBLING_ADOPTION_HPP_
#define MULTI_PURPOSE_MPC_ROS__MPCC_OVERTAKE_SIBLING_ADOPTION_HPP_

#include "multi_purpose_mpc_ros/mpcc_rate_resolved_execution_artifact.hpp"

#include <cstdint>
#include <string>

namespace multi_purpose_mpc_ros::mpcc_overtake_sibling_adoption {

namespace artifact = mpcc_rate_resolved_execution_artifact;
namespace contract = mpcc_execution_contract;

enum class Reason {
  Accepted,
  SelectedAuthorityAvailable,
  InactiveExecution,
  InvalidLiveIdentity,
  HardFault,
  NoReturn,
  ReplacementBudgetExhausted,
  MissingSiblingAuthority,
  NonStatelessSibling,
  InvalidSourceIdentity,
  InvalidSiblingIdentity,
  SourceLiveMismatch,
  SiblingEpochMismatch,
  SiblingSideMismatch,
};

const char *to_string(Reason reason) noexcept;

/// Immutable handoff token carried with one canonical command.  It has no
/// authority by itself; the tactical owner may consume it only after that
/// exact command successfully crosses the publisher boundary.
struct Token {
  std::uint64_t source_sequence{};
  double source_snapshot_sec{};
  contract::ControlIntent intent{contract::ControlIntent::Unknown};
  std::string target_id;
  std::uint64_t mission_generation{};
  int previous_side_sign{};
  int adopted_side_sign{};
};

struct Request {
  bool selected_authority_available{false};
  bool active_execution{false};
  bool hard_fault{false};
  bool before_no_return{false};
  bool replacement_budget_available{false};
  bool sibling_current_world_authority{false};
  bool sibling_stateless_current_world_bundle{false};
  contract::ControlIntent live_intent{contract::ControlIntent::Unknown};
  std::string live_target_id;
  std::uint64_t live_mission_generation{};
  int live_side_sign{};
  artifact::Identity source_identity;
  artifact::Identity sibling_identity;
};

struct Resolution {
  Reason reason{Reason::InactiveExecution};
  bool accepted{false};
  Token token;
};

/// Resolve one explicit cross-side authority transition. Same-epoch pairing
/// is checked independently of the branch bank so the publisher does not
/// trust mutable association alone.
Resolution resolve(const Request &request) noexcept;

/// Revalidate a previously accepted token against live tactical state at the
/// publisher boundary.  No elapsed-time or grace rule participates.
bool token_matches_live_state(
    const Token &token, contract::ControlIntent live_intent,
    const std::string &live_target_id, std::uint64_t live_mission_generation,
    int live_side_sign, bool active_execution, bool before_no_return,
    bool replacement_budget_available, bool hard_fault) noexcept;

} // namespace multi_purpose_mpc_ros::mpcc_overtake_sibling_adoption

#endif // MULTI_PURPOSE_MPC_ROS__MPCC_OVERTAKE_SIBLING_ADOPTION_HPP_
