#include "multi_purpose_mpc_ros/mpcc_overtake_sibling_adoption.hpp"

#include <gtest/gtest.h>

namespace adoption = multi_purpose_mpc_ros::mpcc_overtake_sibling_adoption;
namespace contract = multi_purpose_mpc_ros::mpcc_execution_contract;
namespace artifact =
    multi_purpose_mpc_ros::mpcc_rate_resolved_execution_artifact;

namespace {

artifact::Identity identity(const int side) {
  contract::MpccProblemContext context;
  context.decision_id = 71U;
  context.intent = contract::ControlIntent::ShiftOut;
  context.intent_generation = 4U;
  context.observation_generation = 9U;
  context.stage_geometry_id = 17U;
  context.target_obstacle_generation = 9U;
  context.target_id = "d2";
  context.execution_side_sign = side;
  context.dynamic_obstacle_constraint_active = true;
  context.dynamic_obstacle_generation = 9U;
  context.dynamic_obstacle_id = "d2";
  context.dynamic_obstacle_side_sign = side;
  context.horizon_steps = 20U;
  context.formulation =
      contract::Formulation::VelocitySteeringYawResponseProgress7State;
  context.state_schema_id = "state";
  context.input_schema_id = "input";
  context.bounds_schema_id = "bounds";
  context.cost_schema_id = "cost";
  context = contract::seal_problem_context(std::move(context));
  return artifact::Identity{22U, std::move(context), 10.5};
}

adoption::Request accepted_request() {
  adoption::Request request;
  request.active_execution = true;
  request.before_no_return = true;
  request.replacement_budget_available = true;
  request.sibling_current_world_authority = true;
  request.sibling_stateless_current_world_bundle = true;
  request.live_intent = contract::ControlIntent::ShiftOut;
  request.live_target_id = "d2";
  request.live_mission_generation = 4U;
  request.live_side_sign = -1;
  request.source_identity = identity(-1);
  request.sibling_identity = identity(1);
  return request;
}

TEST(MpccOvertakeSiblingAdoption, AcceptsExactPreNoReturnSibling) {
  const auto result = adoption::resolve(accepted_request());
  ASSERT_TRUE(result.accepted);
  EXPECT_EQ(result.reason, adoption::Reason::Accepted);
  EXPECT_EQ(result.token.target_id, "d2");
  EXPECT_EQ(result.token.previous_side_sign, -1);
  EXPECT_EQ(result.token.adopted_side_sign, 1);
  EXPECT_TRUE(adoption::token_matches_live_state(
      result.token, contract::ControlIntent::ShiftOut, "d2", 4U, -1, true, true,
      true, false));
}

TEST(MpccOvertakeSiblingAdoption, RejectsAfterNoReturn) {
  auto request = accepted_request();
  request.before_no_return = false;
  const auto result = adoption::resolve(request);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, adoption::Reason::NoReturn);
}

TEST(MpccOvertakeSiblingAdoption, RejectsHardFault) {
  auto request = accepted_request();
  request.hard_fault = true;
  const auto result = adoption::resolve(request);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, adoption::Reason::HardFault);
}

TEST(MpccOvertakeSiblingAdoption, RejectsExhaustedReplacementBudget) {
  auto request = accepted_request();
  request.replacement_budget_available = false;
  const auto result = adoption::resolve(request);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, adoption::Reason::ReplacementBudgetExhausted);
}

TEST(MpccOvertakeSiblingAdoption, RejectsMissingCurrentWorldAuthority) {
  auto request = accepted_request();
  request.sibling_current_world_authority = false;
  const auto result = adoption::resolve(request);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, adoption::Reason::MissingSiblingAuthority);
}

TEST(MpccOvertakeSiblingAdoption, RejectsRetainedMissionSibling) {
  auto request = accepted_request();
  request.sibling_stateless_current_world_bundle = false;
  const auto result = adoption::resolve(request);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, adoption::Reason::NonStatelessSibling);
}

TEST(MpccOvertakeSiblingAdoption, RejectsDifferentLiveIntent) {
  auto request = accepted_request();
  request.live_intent = contract::ControlIntent::Pass;
  const auto result = adoption::resolve(request);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, adoption::Reason::SourceLiveMismatch);
}

TEST(MpccOvertakeSiblingAdoption, RejectsDifferentEpoch) {
  auto request = accepted_request();
  request.sibling_identity.snapshot_sec += 0.01;
  const auto result = adoption::resolve(request);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, adoption::Reason::SiblingEpochMismatch);
}

TEST(MpccOvertakeSiblingAdoption, RejectsDifferentTarget) {
  auto request = accepted_request();
  request.sibling_identity.source_context.target_id = "d3";
  request.sibling_identity.source_context.fingerprint = 0U;
  request.sibling_identity.source_context = contract::seal_problem_context(
      std::move(request.sibling_identity.source_context));
  const auto result = adoption::resolve(request);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, adoption::Reason::SiblingEpochMismatch);
}

TEST(MpccOvertakeSiblingAdoption, RejectsSelectedAuthorityStillAvailable) {
  auto request = accepted_request();
  request.selected_authority_available = true;
  const auto result = adoption::resolve(request);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, adoption::Reason::SelectedAuthorityAvailable);
}

TEST(MpccOvertakeSiblingAdoption, PublicationTokenRejectsChangedLiveSide) {
  const auto token = adoption::resolve(accepted_request()).token;
  EXPECT_FALSE(adoption::token_matches_live_state(
      token, contract::ControlIntent::ShiftOut, "d2", 4U, 1, true, true, true,
      false));
}

} // namespace
