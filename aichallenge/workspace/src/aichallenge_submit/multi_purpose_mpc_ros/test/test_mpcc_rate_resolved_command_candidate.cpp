#include "multi_purpose_mpc_ros/mpcc_rate_resolved_command_candidate.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_production_adapter.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <vector>

namespace
{

namespace command = multi_purpose_mpc_ros::mpcc_rate_resolved_command_candidate;
namespace production =
  multi_purpose_mpc_ros::mpcc_rate_resolved_production_adapter;
namespace retained =
  multi_purpose_mpc_ros::mpcc_rate_resolved_retained_revalidation;
namespace artifact =
  multi_purpose_mpc_ros::mpcc_rate_resolved_execution_artifact;
namespace certified = multi_purpose_mpc_ros::mpcc_rate_resolved_certified_plan;
namespace physical = multi_purpose_mpc_ros::mpcc_rate_resolved_physical_wall;
namespace recovery = multi_purpose_mpc_ros::recovery_footprint;
namespace contract = multi_purpose_mpc_ros::mpcc_execution_contract;

contract::MpccProblemContext source_context(
  const contract::ControlIntent intent = contract::ControlIntent::Cruise)
{
  contract::MpccProblemContext context;
  context.decision_id = 11U;
  context.intent = intent;
  context.intent_generation = 2U;
  context.observation_generation = 3U;
  if (contract::canonical_normal_intent_requires_target(intent)) {
    context.target_obstacle_generation = 3U;
    context.target_id = "d2";
  }
  if (contract::canonical_normal_intent_requires_execution_side(intent)) {
    context.execution_side_sign = -1;
  }
  context.stage_geometry_id = 17U;
  context.horizon_steps = 2U;
  context.formulation =
    contract::Formulation::VelocitySteeringProgress6State;
  context.state_schema_id = "ey-elag-epsi-v-progress-steering-v1";
  context.input_schema_id = "accel-steering-rate-progress-rate-v1";
  context.bounds_schema_id = "stage-wall-v1";
  context.cost_schema_id = "velocity-progress-steering-rate-v1";
  return contract::seal_problem_context(std::move(context));
}

std::shared_ptr<const certified::CertifiedPlan> certified_plan(
  const contract::ControlIntent intent = contract::ControlIntent::Cruise)
{
  auto execution = std::make_shared<artifact::ExecutionArtifact>();
  execution->identity = {7U, source_context(intent), 1.0};
  execution->prediction_origin_sec = 1.0;
  execution->completed_sec = 1.01;
  execution->course_progress_origin_m = 50.0;
  execution->semantic_initial_steering_rad = 0.10;
  execution->wheelbase_m = 2.0;
  execution->maximum_abs_steering_rad = 0.60;
  execution->maximum_abs_steering_rate_radps = 1.0;
  execution->physical_global_tolerance = 1e-6;
  execution->maximum_constraint_violation = 1e-8;
  execution->maximum_normalized_constraint_violation = 0.1;
  execution->predicted_states = {
    {0.0, 0.10, 0.0, 2.0, 0.0, 0.10},
    {0.10, 0.0, 0.0, 2.1, 0.2, 0.11},
    {0.20, 0.0, 0.0, 2.2, 0.4, 0.12},
  };
  execution->control_stages = {
    {1.0, 0.10, 2.0, 0.10, 0.0, 4.0, -3.0, 1.37},
    {1.0, 0.10, 2.0, 0.10, 0.0, 4.0, -3.0, 1.37},
  };
  execution->nominal_path_distance_m = {0.0, 0.2, 0.4};
  execution->lateral_lower_m = {-1.0, -1.0, -1.0};
  execution->lateral_upper_m = {1.0, 1.0, 1.0};

  auto grid = std::make_shared<recovery::OccupancyGrid>();
  grid->width = 400U;
  grid->height = 400U;
  grid->resolution_m = 0.05;
  grid->origin_x_m = 45.0;
  grid->origin_y_m = -5.0;
  grid->cells.assign(grid->width * grid->height, recovery::CellState::Free);
  physical::Snapshot snapshot;
  snapshot.identity.artifact = execution->identity;
  snapshot.identity.pose_snapshot_id = 101U;
  snapshot.identity.course_frame_window_id = 102U;
  snapshot.identity.captured_sec = 1.0;
  snapshot.wall_grid = grid;
  snapshot.wall_grid_fingerprint =
    recovery::occupancy_grid_fingerprint(*snapshot.wall_grid);
  snapshot.footprint = {0.05, 0.05, 0.05, 0.05, 0.0};
  snapshot.current_pose = {50.0, 0.0, 0.0};
  snapshot.trajectory.progress_origin_m = 50.0;
  snapshot.trajectory.path_distance_m = {0.2, 0.4};
  snapshot.trajectory.lateral_m = {0.10, 0.20};
  snapshot.trajectory.lag_m = {0.0, 0.0};
  snapshot.trajectory.heading_offset_rad = {0.0, 0.0};
  snapshot.trajectory.velocity_mps = {2.1, 2.2};
  snapshot.trajectory.progress_m = {50.2, 50.4};
  snapshot.trajectory.lateral_lower_m = {-1.0, -1.0};
  snapshot.trajectory.lateral_upper_m = {1.0, 1.0};
  snapshot.trajectory.minimum_lateral_bound_reserve_m = 0.8;
  snapshot.trajectory.progress_regression_tolerance_m = 1e-6;
  snapshot.course_frame_knots = {
    {49.0, 49.0, 0.0, 0.0, 0},
    {52.0, 52.0, 0.0, 0.0, 3},
  };
  snapshot.bound_tolerance_m = 1e-6;
  snapshot.swept_step_m = 0.02;
  physical::Result physical_result;
  physical_result.identity = snapshot.identity;
  physical_result.outcome = physical::Outcome::Accepted;
  physical_result.diagnostic.reason =
    contract::PhysicalWallCertificateReason::Accepted;
  physical_result.completed_sec = 1.01;
  physical_result.compute_ms = 10.0;
  const auto built = certified::build(execution, snapshot, physical_result);
  EXPECT_EQ(built.reason, certified::RejectReason::None);
  return built.plan;
}

retained::Result accepted_result(
  const contract::ControlIntent intent = contract::ControlIntent::Cruise)
{
  retained::Result result;
  result.reason = retained::Reason::Accepted;
  retained::Proof proof;
  proof.plan = certified_plan(intent);
  proof.decision_id = 23U;
  proof.cursor.available = true;
  proof.cursor.sequence = 7U;
  proof.cursor.control_stage_index = 1U;
  proof.cursor.remaining_control_stage_count = 1U;
  proof.observation_origin_sec = 1.05;
  proof.control_origin_sec = 1.05;
  proof.actuation.sequence = 7U;
  proof.actuation.control_stage_index = 1U;
  proof.actuation.predicted_speed_mps = 4.2;
  proof.actuation.acceleration_mps2 = 0.8;
  proof.actuation.steering_rate_radps = -0.2;
  proof.actuation.steering_rad = 0.1;
  proof.actuation.curvature_radpm = 0.05;
  proof.actuation.virtual_progress_speed_mps = 4.1;
  result.proof = proof;
  return result;
}

TEST(RateResolvedCommandCandidate, RejectsMissingRetainedProof) {
  const auto result = command::build(retained::Result{});
  EXPECT_EQ(result.reason, command::Reason::RetainedProofUnavailable);
  EXPECT_FALSE(result.candidate.has_value());
}

TEST(RateResolvedCommandCandidate, RejectsUncertifiedPlan) {
  auto retained_result = accepted_result();
  auto invalid_plan = std::make_shared<certified::CertifiedPlan>();
  retained_result.proof->plan = invalid_plan;
  const auto result = command::build(retained_result);
  EXPECT_EQ(result.reason, command::Reason::InvalidCertifiedPlan);
  EXPECT_FALSE(result.candidate.has_value());
}

TEST(RateResolvedCommandCandidate, PreservesRetainedIdentityAndActuation) {
  const auto result = command::build(accepted_result());
  ASSERT_EQ(result.reason, command::Reason::Available);
  ASSERT_TRUE(result.candidate.has_value());
  const auto & candidate = result.candidate.value();
  EXPECT_EQ(candidate.decision_id, 23U);
  EXPECT_EQ(candidate.artifact_sequence, 7U);
  EXPECT_TRUE(contract::problem_context_complete(candidate.source_context));
  EXPECT_EQ(candidate.source_context.decision_id, 11U);
  EXPECT_EQ(
    candidate.source_context.fingerprint,
    source_context().fingerprint);
  EXPECT_EQ(candidate.source_context.stage_geometry_id, 17U);
  EXPECT_EQ(candidate.source_context.intent, contract::ControlIntent::Cruise);
  EXPECT_EQ(
    candidate.source_context.formulation,
    contract::Formulation::VelocitySteeringProgress6State);
  EXPECT_STREQ(
    contract::to_string(candidate.source_context.formulation),
    "velocity-steering-progress-6state");
  EXPECT_EQ(candidate.control_stage_index, 1U);
  EXPECT_DOUBLE_EQ(candidate.predicted_speed_mps, 4.2);
  EXPECT_DOUBLE_EQ(candidate.acceleration_mps2, 0.8);
  EXPECT_DOUBLE_EQ(candidate.steering_rate_radps, -0.2);
  EXPECT_DOUBLE_EQ(candidate.steering_rad, 0.1);
  EXPECT_DOUBLE_EQ(candidate.curvature_radpm, 0.05);
  EXPECT_DOUBLE_EQ(candidate.virtual_progress_speed_mps, 4.1);
}

TEST(RateResolvedCommandCandidate, RejectsFiveStateArtifactIdentity)
{
  auto retained_result = accepted_result();
  auto mutable_plan = std::make_shared<certified::CertifiedPlan>(
    *retained_result.proof->plan);
  auto mutable_artifact = std::make_shared<artifact::ExecutionArtifact>(
    *mutable_plan->execution_artifact);
  mutable_artifact->identity.source_context.formulation =
    contract::Formulation::VelocityProgress5State;
  mutable_plan->execution_artifact = std::move(mutable_artifact);
  retained_result.proof->plan = std::move(mutable_plan);

  const auto result = command::build(retained_result);
  EXPECT_EQ(result.reason, command::Reason::InvalidCertifiedPlan);
  EXPECT_FALSE(result.candidate.has_value());
}

TEST(RateResolvedProductionAdapter, BuildsCanonicalSixStateAuthority)
{
  const auto result = production::build(accepted_result());
  ASSERT_EQ(result.reason, production::Reason::Available);
  ASSERT_TRUE(result.authority.has_value());
  const auto & authority = result.authority.value();
  EXPECT_TRUE(contract::problem_context_complete(authority.problem));
  EXPECT_TRUE(contract::solution_certified(authority.solution));
  EXPECT_EQ(
    authority.problem.formulation,
    contract::Formulation::VelocitySteeringProgress6State);
  EXPECT_EQ(authority.command.decision_id, 23U);
  EXPECT_EQ(authority.command.execution_plan_id, 7U);
  EXPECT_EQ(authority.command.execution_certificate_decision_id, 23U);
  EXPECT_EQ(
    authority.command.source,
    contract::CanonicalNormalAuthoritySource::RetainedCertified);
  EXPECT_EQ(
    authority.command.formulation,
    contract::Formulation::VelocitySteeringProgress6State);
  EXPECT_TRUE(authority.command.retained_solution);
  EXPECT_DOUBLE_EQ(authority.command.predicted_speed_mps, 4.2);
  EXPECT_DOUBLE_EQ(authority.command.acceleration_mps2, 0.8);
  EXPECT_DOUBLE_EQ(authority.command.steering_tire_angle_rad, 0.1);
  ASSERT_EQ(authority.target_speed_horizon_mps.size(), 1U);
  ASSERT_EQ(authority.steering_horizon_rad.size(), 1U);
  ASSERT_EQ(authority.world_prediction.first.size(), 1U);
  ASSERT_EQ(authority.world_prediction.second.size(), 1U);
}

TEST(RateResolvedProductionAdapter, BuildsEveryArtifactOwnedIntent)
{
  const std::vector<contract::ControlIntent> intents{
    contract::ControlIntent::Track,
    contract::ControlIntent::Cruise,
    contract::ControlIntent::ShiftOut,
    contract::ControlIntent::Pass,
    contract::ControlIntent::Return,
  };
  for (const auto intent : intents) {
    SCOPED_TRACE(contract::to_string(intent));
    const auto result = production::build(accepted_result(intent));
    ASSERT_EQ(result.reason, production::Reason::Available);
    ASSERT_TRUE(result.authority.has_value());
    EXPECT_EQ(result.authority->problem.intent, intent);
    EXPECT_EQ(result.authority->command.intent, intent);
  }
}

TEST(RateResolvedProductionAdapter, FinalTraceAcceptsExactSixStateIdentity)
{
  const auto result = production::build(accepted_result());
  ASSERT_TRUE(result.authority.has_value());
  const auto & authority = result.authority.value();
  const auto decision = contract::resolve_final_control_decision(
    contract::FinalControlDecisionRequest{
      23U, contract::FinalAuthorityClass::CertifiedNormalSolution,
      "mpc-solution", authority.problem, authority.solution, true,
      authority.command});
  EXPECT_TRUE(decision.identity_complete);
  EXPECT_TRUE(decision.canonical_contract_satisfied);
  EXPECT_EQ(decision.reason, "matching-certified-solution");
  EXPECT_EQ(
    decision.formulation,
    contract::Formulation::VelocitySteeringProgress6State);
  EXPECT_EQ(decision.execution_plan_id, 7U);
  EXPECT_EQ(decision.execution_certificate_decision_id, 23U);
}

TEST(RateResolvedProductionAdapter, RejectsBlockedCurrentWorldProof)
{
  auto retained_result = accepted_result();
  retained_result.reason = retained::Reason::DynamicPathBlocked;
  retained_result.proof.reset();
  const auto result = production::build(retained_result);
  EXPECT_EQ(result.reason, production::Reason::RetainedProofUnavailable);
  EXPECT_FALSE(result.authority.has_value());
}

} // namespace
