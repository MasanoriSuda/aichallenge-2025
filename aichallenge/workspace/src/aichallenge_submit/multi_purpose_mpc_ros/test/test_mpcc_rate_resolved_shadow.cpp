#include "multi_purpose_mpc_ros/mpcc_rate_resolved_shadow.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_physical_adapter.hpp"

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <cmath>
#include <limits>
#include <utility>

namespace shadow =
  multi_purpose_mpc_ros::mpcc_rate_resolved_shadow;
namespace adapter =
  multi_purpose_mpc_ros::mpcc_rate_resolved_adapter;
namespace model = multi_purpose_mpc_ros::mpcc_rate_resolved;
namespace problem = multi_purpose_mpc_ros::mpcc_rate_resolved_problem;
namespace solver = multi_purpose_mpc_ros::persistent_osqp;
namespace execution =
  multi_purpose_mpc_ros::mpcc_rate_resolved_execution_artifact;
namespace physical =
  multi_purpose_mpc_ros::mpcc_rate_resolved_physical_adapter;
namespace contract =
  multi_purpose_mpc_ros::mpcc_execution_contract;
namespace wall_refinement =
  multi_purpose_mpc_ros::mpcc_rate_resolved_wall_refinement;
namespace recovery =
  multi_purpose_mpc_ros::recovery_footprint;

namespace
{

contract::MpccProblemContext source_context(
  const std::uint64_t decision_id, const std::uint64_t stage_geometry_id)
{
  contract::MpccProblemContext context;
  context.decision_id = decision_id;
  context.intent = contract::ControlIntent::Track;
  context.intent_generation = 1U;
  context.observation_generation = 2U;
  context.stage_geometry_id = stage_geometry_id;
  context.horizon_steps = 3U;
  context.formulation =
    contract::Formulation::VelocitySteeringYawResponseProgress7State;
  context.state_schema_id = "ey-elag-epsi-v-progress-steering-v1";
  context.input_schema_id = "accel-steering-rate-progress-rate-v1";
  context.bounds_schema_id = "stage-wall-v1";
  context.cost_schema_id = "velocity-progress-steering-rate-v1";
  return contract::seal_problem_context(std::move(context));
}

adapter::Request straight_request(const int horizon = 3)
{
  adapter::Request request;
  request.horizon_steps = horizon;
  request.initial_state << 0.0, 0.0, 0.0, 2.0, 0.0;
  request.current_steering_rad = 0.10;
  request.current_response_steering_rad = 0.08;
  request.wheelbase_m = 2.0;
  request.yaw_response_gain = 0.75;
  request.yaw_response_time_constant_sec = 0.13;
  request.maximum_abs_steering_rad = 0.60;
  request.maximum_abs_steering_rate_radps = 0.70;
  request.states.resize(static_cast<std::size_t>(horizon + 1));
  for (int stage = 0; stage <= horizon; ++stage) {
    auto & state = request.states[static_cast<std::size_t>(stage)];
    state.reference << 0.0, 0.0, 0.0, 2.0, 0.2 * stage;
    state.lower << -1.0, -1.0, -1.0, 0.0, -1.0;
    state.upper << 1.0, 1.0, 1.0, 4.0, 2.0;
    state.weight << 2.0, 1.0, 2.0, 1.0, 1.0;
    state.linear_cost[4] = -0.5;
  }
  request.inputs.resize(static_cast<std::size_t>(horizon));
  for (int stage = 0; stage < horizon; ++stage) {
    auto & input = request.inputs[static_cast<std::size_t>(stage)];
    input.reference << 0.0, std::tan(0.10) / 2.0, 2.0;
    input.lower << -1.0, -0.30, 0.0;
    input.upper << 1.0, 0.30, 4.0;
    input.weight << 1.0, 1.0, 1.0;
    input.path_curvature_radpm = 0.0;
    input.stage_dt_sec = 0.10;
  }
  request.previous_input << 0.0, std::tan(0.10) / 2.0, 2.0;
  request.input_delta_weight << 0.2, 0.3, 0.1;
  return request;
}

adapter::Request narrow_progress_request(const int horizon = 3)
{
  auto request = straight_request(horizon);
  for (int stage = 0; stage <= horizon; ++stage) {
    auto & state = request.states[static_cast<std::size_t>(stage)];
    const double progress_reference_m = 0.2 * static_cast<double>(stage);
    state.reference[model::kProgressIndex] = progress_reference_m;
    state.lower[model::kProgressIndex] = progress_reference_m - 0.025;
    state.upper[model::kProgressIndex] = progress_reference_m + 0.025;
  }
  request.states.front().lower[model::kProgressIndex] = 0.0;
  request.states.front().upper[model::kProgressIndex] = 0.0;
  return request;
}

shadow::Snapshot snapshot(const std::uint64_t sequence = 1U)
{
  shadow::Snapshot result;
  result.identity.sequence = sequence;
  result.identity.source_context =
    source_context(42U + sequence, 201U + sequence);
  result.identity.snapshot_sec = 10.0 + 0.1 * sequence;
  result.control_prediction_origin_sec = result.identity.snapshot_sec + 0.13;
  result.request = straight_request();
  result.execution_prefix_steps = result.request.horizon_steps;
  result.course_progress_origin_m = 50.0;
  result.nominal_path_distance_m = {0.0, 0.2, 0.4, 0.6};
  result.publication_interval_sec = 0.025;
  return result;
}

void bind_dynamic_obstacle_identity(
  shadow::Snapshot & snapshot, const std::string & obstacle_id = "d2",
  const std::uint64_t observation_generation = 7U,
  const int side_sign = 1)
{
  auto & context = snapshot.identity.source_context;
  context.dynamic_obstacle_constraint_active = true;
  context.dynamic_obstacle_id = obstacle_id;
  context.dynamic_obstacle_generation = observation_generation;
  context.dynamic_obstacle_side_sign = side_sign;
  context = contract::seal_problem_context(std::move(context));
}

recovery::OccupancyGrid corridor_grid()
{
  recovery::OccupancyGrid grid;
  grid.width = 240U;
  grid.height = 240U;
  grid.resolution_m = 0.1;
  grid.origin_x_m = 0.0;
  grid.origin_y_m = 0.0;
  grid.y_axis = recovery::YAxisConvention::RowZeroAtMinimumY;
  grid.cells.assign(grid.width * grid.height, recovery::CellState::Free);
  for (std::size_t column = 0U; column < grid.width; ++column) {
    const auto wall = grid.world_to_grid(
      grid.origin_x_m + grid.resolution_m * static_cast<double>(column),
      10.0);
    if (wall.has_value()) {
      grid.cells[wall->row * grid.width + wall->column] =
        recovery::CellState::Occupied;
    }
  }
  return grid;
}

wall_refinement::Request wall_request(
  const recovery::OccupancyGrid & grid, const double heading_rad)
{
  wall_refinement::Request request;
  request.active = true;
  request.wall_grid = &grid;
  request.footprint = recovery::FootprintExtents{1.0, 1.0, 0.2, 0.2, 0.0};
  request.course_frame_knots = {
    {0.0, 5.0, 8.0, 0.0, 0},
    {20.0, 25.0, 8.0, 0.0, 1}};
  request.course_progress_origin_m = 0.0;
  request.heading_bucket_width_rad = 0.025;
  request.translation_bucket_width_m = 0.05;
  request.lateral_sample_step_m = 0.05;
  request.boundary_guard_m = 0.001;
  request.stages.push_back(wall_refinement::StageRequest{
    5.0, 1.4, 0.0, heading_rad,
    -0.5, 1.8, -1.0, 1.0, -1.0, 1.0, 4.0, 6.0});
  return request;
}

}  // namespace

TEST(MpccRateResolvedWallRefinement, ResolvesUnionSupportBeforeProgressIsKnown)
{
  const auto result =
    wall_refinement::resolve_pre_refinement_lateral_support(
    wall_refinement::PreRefinementLateralSupportRequest{
      true, -0.4,
      {-0.8, -0.2, 1.7, 1.6},
      {0.6, 0.9, 2.0, 2.1}});
  ASSERT_TRUE(result.valid);
  ASSERT_TRUE(result.applied);
  EXPECT_EQ(
    result.reason,
    wall_refinement::PreRefinementLateralSupportReason::Accepted);
  EXPECT_DOUBLE_EQ(result.lower_m, -0.8);
  EXPECT_DOUBLE_EQ(result.upper_m, 2.1);
}

TEST(MpccRateResolvedWallRefinement, RejectsMalformedPreRefinementProfile)
{
  const auto result =
    wall_refinement::resolve_pre_refinement_lateral_support(
    wall_refinement::PreRefinementLateralSupportRequest{
      true, 0.0, {-0.5, 0.3}, {0.5, 0.2}});
  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.applied);
  EXPECT_EQ(
    result.reason,
    wall_refinement::PreRefinementLateralSupportReason::InvalidInput);
}

TEST(MpccRateResolvedShadow, DoesNotBindFutureWallSampleBeforeProgressSolve)
{
  shadow::SolverContext context;
  auto input = snapshot();
  for (std::size_t stage = 1U; stage < input.request.states.size(); ++stage) {
    input.request.states[stage].lower[0] = 1.7;
    input.request.states[stage].upper[0] = 1.9;
  }
  input.progress_aligned_wall_refinement_active = true;
  input.wall_reference_progress_m = {0.0, 1.0, 2.0, 3.0};
  input.wall_lower_m = {-0.5, -0.5, 1.7, 1.7};
  input.wall_upper_m = {0.5, 0.5, 1.9, 1.9};
  input.progress_wall_profile_diagnostic = "future-corridor-shift";

  const auto result = context.evaluate(input);
  ASSERT_EQ(result.outcome, shadow::Outcome::Solved) << result.detail;
  EXPECT_TRUE(result.pre_refinement_lateral_support_applied);
  EXPECT_EQ(
    result.pre_refinement_lateral_support_reason,
    wall_refinement::PreRefinementLateralSupportReason::Accepted);
  EXPECT_DOUBLE_EQ(result.pre_refinement_lateral_support_lower_m, -0.5);
  EXPECT_DOUBLE_EQ(result.pre_refinement_lateral_support_upper_m, 1.9);
  EXPECT_TRUE(result.progress_wall_refinement_applied);
  EXPECT_TRUE(result.progress_wall_refinement_solved);
  EXPECT_GT(result.wall_refinement_solver_solve_count, 0U);
  EXPECT_EQ(result.wall_refinement_solver_scaling_iterations, 10);
  EXPECT_FALSE(result.physical_wall_lag_pose_box_applied);
  EXPECT_FALSE(result.physical_wall_heading_pose_box_applied);
  EXPECT_TRUE(result.post_refinement_physical_proof_checked);
  EXPECT_TRUE(result.post_refinement_physical_proof_accepted);
  EXPECT_EQ(
    result.post_refinement_linearization_requested,
    result.post_refinement_linearization_count > 0U);
  if (result.post_refinement_linearization_requested) {
    EXPECT_TRUE(result.post_refinement_linearization_applied);
    EXPECT_TRUE(result.post_refinement_linearization_bootstrap_applied);
    EXPECT_TRUE(result.post_refinement_linearization_solved);
    EXPECT_EQ(
      result.post_refinement_linearization_reason,
      multi_purpose_mpc_ros::mpcc_rate_resolved_adapter::
      RelinearizationReason::Accepted);
  }
}

TEST(MpccRateResolvedShadow, SolvesAndSamplesOnePublicationInterval)
{
  shadow::SolverContext context;
  const auto input = snapshot();
  const auto result = context.evaluate(input);
  ASSERT_EQ(result.outcome, shadow::Outcome::Solved) << result.detail;
  EXPECT_EQ(
    result.receding_warm_start_reason,
    shadow::RecedingWarmStartReason::CurrentProblemBootstrap);
  EXPECT_TRUE(result.receding_warm_start_applied);
  EXPECT_TRUE(result.successive_linearization_requested);
  EXPECT_TRUE(result.successive_linearization_applied);
  EXPECT_TRUE(result.successive_linearization_bootstrap_applied);
  EXPECT_TRUE(result.successive_linearization_solved);
  EXPECT_EQ(
    result.successive_linearization_reason,
    multi_purpose_mpc_ros::mpcc_rate_resolved_adapter::
    RelinearizationReason::Accepted);
  EXPECT_FALSE(result.post_refinement_linearization_requested);
  EXPECT_FALSE(result.post_refinement_linearization_applied);
  EXPECT_FALSE(result.post_refinement_linearization_bootstrap_applied);
  EXPECT_FALSE(result.post_refinement_linearization_solved);
  EXPECT_FALSE(result.post_refinement_physical_proof_checked);
  EXPECT_FALSE(result.post_refinement_physical_proof_accepted);
  EXPECT_EQ(result.post_refinement_linearization_count, 0U);
  EXPECT_FALSE(result.physical_dynamic_sqp_audit_requested);
  EXPECT_FALSE(result.physical_dynamic_sqp_audit_applied);
  EXPECT_FALSE(result.physical_dynamic_sqp_audit_solved);
  EXPECT_EQ(result.physical_dynamic_sqp_audit_iteration_limit, 0U);
  EXPECT_EQ(result.physical_dynamic_sqp_audit_count, 0U);
  EXPECT_NE(
    result.receding_warm_start_diagnostic.find("previous=empty-cache"),
    std::string::npos);
  EXPECT_TRUE(shadow::result_valid(result));
  EXPECT_TRUE(result.constraints_satisfied);
  EXPECT_TRUE(result.actuation_sampled);
  EXPECT_DOUBLE_EQ(result.initial_steering_rad, input.request.current_steering_rad);
  EXPECT_TRUE(std::isfinite(result.solver_initial_steering_rad));
  EXPECT_NEAR(
    result.sampled_steering_rad,
    result.initial_steering_rad +
    result.first_steering_rate_radps * input.publication_interval_sec,
    1e-9);
  EXPECT_LE(
    std::abs(result.first_steering_rate_radps),
    input.request.maximum_abs_steering_rate_radps + 1e-6);
  EXPECT_LT(
    result.first_steering_rate_physical_lower_radps,
    result.first_steering_rate_solver_lower_radps);
  EXPECT_LT(
    result.first_steering_rate_solver_upper_radps,
    result.first_steering_rate_physical_upper_radps);
  EXPECT_GT(result.first_steering_rate_certificate_margin_radps, 0.0);
  EXPECT_GE(
    result.first_steering_rate_radps,
    result.first_steering_rate_solver_lower_radps -
    result.solver.physical_global_tolerance);
  EXPECT_LE(
    result.first_steering_rate_radps,
    result.first_steering_rate_solver_upper_radps +
    result.solver.physical_global_tolerance);
  EXPECT_EQ(result.certified_stage_count, 3U);
  EXPECT_EQ(result.sampled_stage_index, 0U);
  EXPECT_NEAR(result.sampled_stage_elapsed_sec, 0.025, 1e-12);
  EXPECT_NEAR(result.certified_horizon_duration_sec, 0.30, 1e-12);
  ASSERT_NE(result.execution_artifact, nullptr);
  EXPECT_EQ(
    execution::validate(*result.execution_artifact),
    execution::RejectReason::None);
  EXPECT_EQ(result.execution_artifact->predicted_states.size(), 4U);
  EXPECT_EQ(result.execution_artifact->control_stages.size(), 3U);
  EXPECT_EQ(result.execution_artifact->nominal_path_distance_m.size(), 4U);
  EXPECT_EQ(result.execution_artifact->lateral_lower_m.size(), 4U);
  EXPECT_DOUBLE_EQ(result.execution_artifact->course_progress_origin_m, 50.0);
  EXPECT_DOUBLE_EQ(
    result.execution_artifact->prediction_origin_sec,
    input.control_prediction_origin_sec);
  EXPECT_LT(
    result.execution_artifact->completed_sec,
    result.execution_artifact->prediction_origin_sec);
  EXPECT_DOUBLE_EQ(
    result.execution_artifact->semantic_initial_steering_rad,
    input.request.current_steering_rad);
  EXPECT_DOUBLE_EQ(
    result.execution_artifact->control_stages.front().steering_rate_radps,
    result.first_steering_rate_radps);
  ASSERT_NE(result.latest_state_feedback_preparation, nullptr);
  EXPECT_EQ(
    result.latest_state_feedback_preparation->snapshot.identity.sequence,
    input.identity.sequence);
  EXPECT_EQ(
    result.latest_state_feedback_preparation->prepared_primal.size(),
    static_cast<Eigen::Index>(
      model::kStateDimension * (input.request.horizon_steps + 1) +
      model::kInputDimension * input.request.horizon_steps));
  const auto physical_result = physical::build(
    *result.execution_artifact, contract::ControlIntent::Track,
    input.identity.source_context.stage_geometry_id);
  EXPECT_EQ(physical_result.reason, physical::RejectReason::None);
  EXPECT_TRUE(physical_result.exact_trajectory.has_value());

  auto invalid = result;
  invalid.first_steering_rate_certificate_margin_radps =
    std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(shadow::result_valid(invalid));
}

TEST(MpccRateResolvedShadow, DynamicSqpAuditRejectsDepthBeyondFixedBudget)
{
  shadow::SolverContext context;
  const auto result = context.evaluate_physical_dynamic_sqp_audit(
    snapshot(), 4U);

  EXPECT_EQ(result.outcome, shadow::Outcome::BuildRejected);
  EXPECT_TRUE(result.physical_dynamic_sqp_audit_requested);
  EXPECT_FALSE(result.physical_dynamic_sqp_audit_applied);
  EXPECT_FALSE(result.physical_dynamic_sqp_audit_solved);
  EXPECT_EQ(result.physical_dynamic_sqp_audit_iteration_limit, 4U);
  EXPECT_NE(result.detail.find("exceeds fixed budget"), std::string::npos);
}

TEST(MpccRateResolvedShadow, LatestStateFeedbackResolvesOneConsistentArtifact)
{
  shadow::SolverContext preparation_context;
  shadow::LatestStateFeedbackSolverContext feedback_context;
  const auto input = snapshot();
  const auto prepared = preparation_context.evaluate(input);
  ASSERT_EQ(prepared.outcome, shadow::Outcome::Solved) << prepared.detail;
  ASSERT_NE(prepared.latest_state_feedback_preparation, nullptr);

  execution::PredictedState latest_state{
    0.02, 0.0, 0.0, 2.0, 0.10, 0.12, 0.09};
  Eigen::Matrix<double, model::kInputDimension, 1> previous_input;
  previous_input << 0.05, 0.20, 2.0;
  const auto result = feedback_context.evaluate(
    shadow::LatestStateFeedbackRequest{
      prepared.latest_state_feedback_preparation,
      input.control_prediction_origin_sec + 0.20,
      input.control_prediction_origin_sec + 0.07,
      latest_state,
      previous_input});
  ASSERT_EQ(result.reason, shadow::LatestStateFeedbackReason::Accepted)
    << result.detail;
  EXPECT_TRUE(result.assembled);
  EXPECT_TRUE(result.solve_attempted);
  EXPECT_TRUE(result.solved);
  EXPECT_TRUE(result.finite);
  EXPECT_TRUE(result.constraints_satisfied);
  ASSERT_NE(result.execution_artifact, nullptr);
  EXPECT_EQ(
    execution::validate(*result.execution_artifact),
    execution::RejectReason::None);
  ASSERT_FALSE(result.execution_artifact->predicted_states.empty());
  const auto & solved_initial = result.execution_artifact->predicted_states.front();
  const double state_tolerance =
    result.execution_artifact->physical_global_tolerance;
  EXPECT_NEAR(solved_initial.lateral_m, latest_state.lateral_m, state_tolerance);
  EXPECT_NEAR(solved_initial.lag_m, latest_state.lag_m, state_tolerance);
  EXPECT_NEAR(
    solved_initial.heading_offset_rad,
    latest_state.heading_offset_rad, state_tolerance);
  EXPECT_NEAR(
    solved_initial.velocity_mps, latest_state.velocity_mps, state_tolerance);
  EXPECT_NEAR(
    solved_initial.progress_m, latest_state.progress_m, state_tolerance);
  EXPECT_NEAR(
    solved_initial.steering_rad, latest_state.steering_rad, state_tolerance);
  EXPECT_NEAR(
    solved_initial.response_steering_rad,
    latest_state.response_steering_rad, state_tolerance);
  EXPECT_DOUBLE_EQ(
    result.execution_artifact->prediction_origin_sec,
    input.control_prediction_origin_sec + 0.20);
  const auto exact = physical::build(
    *result.execution_artifact, contract::ControlIntent::Track,
    input.identity.source_context.stage_geometry_id);
  EXPECT_EQ(exact.reason, physical::RejectReason::None);
  EXPECT_TRUE(exact.exact_trajectory.has_value());
}

TEST(MpccRateResolvedShadow, LatestStateFeedbackRejectsInvalidLatestState)
{
  shadow::SolverContext preparation_context;
  shadow::LatestStateFeedbackSolverContext feedback_context;
  const auto input = snapshot();
  const auto prepared = preparation_context.evaluate(input);
  ASSERT_NE(prepared.latest_state_feedback_preparation, nullptr);
  execution::PredictedState invalid_state{
    0.0, 0.0, 0.0, 2.0, 0.0,
    input.request.maximum_abs_steering_rad + 0.01, 0.0};
  const auto result = feedback_context.evaluate(
    shadow::LatestStateFeedbackRequest{
      prepared.latest_state_feedback_preparation,
      input.control_prediction_origin_sec + 0.20,
      input.control_prediction_origin_sec + 0.07,
      invalid_state,
      Eigen::Matrix<double, model::kInputDimension, 1>::Zero()});
  EXPECT_EQ(result.reason, shadow::LatestStateFeedbackReason::InvalidRequest);
  EXPECT_FALSE(result.solve_attempted);
  EXPECT_EQ(result.execution_artifact, nullptr);
}

TEST(
  MpccRateResolvedShadow,
  ArchitectureEscapeHatchClassifiesOldOriginFeedbackAsProblemRebuildDefect)
{
  shadow::SolverContext preparation_context;
  shadow::LatestStateFeedbackSolverContext feedback_context;
  auto old_origin = snapshot();
  old_origin.request = narrow_progress_request();
  const auto prepared = preparation_context.evaluate(old_origin);
  ASSERT_EQ(prepared.outcome, shadow::Outcome::Solved) << prepared.detail;
  ASSERT_NE(prepared.latest_state_feedback_preparation, nullptr);

  // The vehicle has advanced 0.40 m relative to the preparation's local
  // progress origin. Replacing only x0 leaves every future progress box and
  // SQP linearization at the old origin, so stage one cannot be reached.
  const execution::PredictedState latest_in_old_origin{
    0.0, 0.0, 0.0, 2.0, 0.40, 0.10, 0.08};
  const auto old_problem_feedback = feedback_context.evaluate(
    shadow::LatestStateFeedbackRequest{
      prepared.latest_state_feedback_preparation,
      old_origin.control_prediction_origin_sec + 0.20,
      old_origin.control_prediction_origin_sec + 0.07,
      latest_in_old_origin,
      old_origin.request.previous_input});
  EXPECT_EQ(
    old_problem_feedback.reason,
    shadow::LatestStateFeedbackReason::SolveRejected)
    << old_problem_feedback.detail;
  EXPECT_EQ(old_problem_feedback.execution_artifact, nullptr);

  // A current-world rebuild represents the same physical state at a new local
  // course-progress origin. It rebuilds all stage bounds, references and
  // linearizations together instead of mixing two time origins.
  shadow::SolverContext current_world_context;
  auto current_world = old_origin;
  current_world.identity.sequence += 1U;
  current_world.identity.snapshot_sec += 0.20;
  current_world.control_prediction_origin_sec += 0.20;
  current_world.course_progress_origin_m += latest_in_old_origin.progress_m;
  current_world.request = narrow_progress_request();
  current_world.identity.source_context.decision_id += 1U;
  current_world.identity.source_context = contract::seal_problem_context(
    std::move(current_world.identity.source_context));
  const auto rebuilt = current_world_context.evaluate(current_world);
  EXPECT_EQ(rebuilt.outcome, shadow::Outcome::Solved) << rebuilt.detail;
  ASSERT_NE(rebuilt.execution_artifact, nullptr);
  EXPECT_DOUBLE_EQ(
    rebuilt.execution_artifact->course_progress_origin_m,
    old_origin.course_progress_origin_m + latest_in_old_origin.progress_m);
}

TEST(
  MpccRateResolvedShadow,
  TimeAlignedSuffixSolvesTheMixedOriginFeedbackCounterexample)
{
  shadow::SolverContext preparation_context;
  shadow::LatestStateFeedbackSolverContext feedback_context;
  auto old_origin = snapshot();
  old_origin.request = narrow_progress_request();
  const auto prepared = preparation_context.evaluate(old_origin);
  ASSERT_EQ(prepared.outcome, shadow::Outcome::Solved) << prepared.detail;
  ASSERT_NE(prepared.latest_state_feedback_preparation, nullptr);

  const execution::PredictedState latest_state{
    0.0, 0.0, 0.0, 2.0, 0.40, 0.10, 0.08};
  const double feedback_origin_sec =
    old_origin.control_prediction_origin_sec + 0.20;
  const auto old_qp = feedback_context.evaluate(
    shadow::LatestStateFeedbackRequest{
      prepared.latest_state_feedback_preparation,
      feedback_origin_sec,
      old_origin.control_prediction_origin_sec + 0.07,
      latest_state,
      old_origin.request.previous_input});
  ASSERT_EQ(old_qp.reason, shadow::LatestStateFeedbackReason::SolveRejected)
    << old_qp.detail;

  const auto suffix = shadow::resolve_time_aligned_suffix(
    shadow::TimeAlignedSuffixRequest{
      &old_origin, feedback_origin_sec, latest_state,
      old_origin.request.previous_input});
  ASSERT_EQ(suffix.reason, shadow::TimeAlignedSuffixReason::Accepted)
    << suffix.detail;
  ASSERT_TRUE(suffix.snapshot.has_value());
  EXPECT_EQ(suffix.consumed_stage_count, 2U);
  EXPECT_NEAR(suffix.first_remaining_stage_duration_sec, 0.10, 1e-12);
  EXPECT_EQ(suffix.snapshot->request.horizon_steps, 1);
  EXPECT_EQ(suffix.snapshot->execution_prefix_steps, 1);
  EXPECT_EQ(suffix.snapshot->request.states.size(), 2U);
  EXPECT_EQ(suffix.snapshot->request.inputs.size(), 1U);
  EXPECT_EQ(suffix.snapshot->nominal_path_distance_m.size(), 2U);
  EXPECT_NEAR(suffix.snapshot->nominal_path_distance_m.front(), 0.0, 1e-12);
  EXPECT_NEAR(suffix.snapshot->nominal_path_distance_m.back(), 0.20, 1e-12);

  shadow::SolverContext suffix_context;
  const auto solved = suffix_context.evaluate(suffix.snapshot.value());
  EXPECT_EQ(solved.outcome, shadow::Outcome::Solved) << solved.detail;
  ASSERT_NE(solved.execution_artifact, nullptr);
  EXPECT_NEAR(
    solved.execution_artifact->predicted_states.front().progress_m,
    latest_state.progress_m,
    solved.execution_artifact->physical_global_tolerance);
}

TEST(
  MpccRateResolvedShadow,
  TimeAlignedSuffixMovesEveryStageIndexedInputWithOneClock)
{
  auto source = snapshot();
  source.dynamic_obstacle_stages = {
    {true, 10.0, 1.0, 0.8, 0.75},
    {true, 20.0, 1.1, 0.8, 0.75},
    {true, 30.0, 1.2, 0.8, 0.75}};
  source.dynamic_obstacle_forced_first_pass_side_stage = 2;
  source.dynamic_obstacle_forced_first_ahead_stage = 3;
  source.dynamic_obstacle_forced_diagonal_start_stage = 1;
  source.dynamic_obstacle_forced_diagonal_full_side_stage = 2;
  const execution::PredictedState latest_state{
    0.1, 0.0, 0.0, 2.0, 0.22, 0.10, 0.08};

  const auto suffix = shadow::resolve_time_aligned_suffix(
    shadow::TimeAlignedSuffixRequest{
      &source, source.control_prediction_origin_sec + 0.11,
      latest_state, source.request.previous_input});

  ASSERT_EQ(suffix.reason, shadow::TimeAlignedSuffixReason::Accepted)
    << suffix.detail;
  ASSERT_TRUE(suffix.snapshot.has_value());
  EXPECT_EQ(suffix.consumed_stage_count, 1U);
  EXPECT_NEAR(suffix.elapsed_in_first_remaining_stage_sec, 0.01, 1e-12);
  EXPECT_NEAR(suffix.first_remaining_stage_duration_sec, 0.09, 1e-12);
  EXPECT_EQ(suffix.snapshot->request.horizon_steps, 2);
  EXPECT_EQ(suffix.snapshot->dynamic_obstacle_stages.size(), 2U);
  EXPECT_DOUBLE_EQ(
    suffix.snapshot->dynamic_obstacle_stages.front().target_progress_m,
    20.0);
  ASSERT_EQ(suffix.snapshot->nominal_path_distance_m.size(), 3U);
  EXPECT_NEAR(suffix.snapshot->nominal_path_distance_m[0], 0.0, 1e-12);
  EXPECT_NEAR(suffix.snapshot->nominal_path_distance_m[1], 0.18, 1e-12);
  EXPECT_NEAR(suffix.snapshot->nominal_path_distance_m[2], 0.38, 1e-12);
  EXPECT_EQ(
    suffix.snapshot->dynamic_obstacle_forced_first_pass_side_stage, 1);
  EXPECT_EQ(suffix.snapshot->dynamic_obstacle_forced_first_ahead_stage, 2);
  EXPECT_EQ(suffix.snapshot->dynamic_obstacle_forced_diagonal_start_stage, 0);
  EXPECT_EQ(
    suffix.snapshot->dynamic_obstacle_forced_diagonal_full_side_stage, 1);
  EXPECT_EQ(
    suffix.snapshot->identity.source_context.horizon_steps, 2U);
  EXPECT_TRUE(contract::problem_context_complete(
      suffix.snapshot->identity.source_context));
}

TEST(MpccRateResolvedShadow, TimeAlignedSuffixRejectsSubminimumRemainder)
{
  const auto source = snapshot();
  const execution::PredictedState latest_state{
    0.0, 0.0, 0.0, 2.0, 0.19, 0.10, 0.08};
  const auto suffix = shadow::resolve_time_aligned_suffix(
    shadow::TimeAlignedSuffixRequest{
      &source, source.control_prediction_origin_sec + 0.095,
      latest_state, source.request.previous_input});
  EXPECT_EQ(
    suffix.reason,
    shadow::TimeAlignedSuffixReason::SubminimumFirstStage);
  EXPECT_FALSE(suffix.snapshot.has_value());
}

TEST(
  MpccRateResolvedShadow,
  TimeAlignedPreparedProblemSolvesTheMixedOriginFeedbackCounterexample)
{
  shadow::SolverContext preparation_context;
  shadow::LatestStateFeedbackSolverContext old_feedback_context;
  auto old_origin = snapshot();
  old_origin.request = narrow_progress_request();
  const auto prepared = preparation_context.evaluate(old_origin);
  ASSERT_EQ(prepared.outcome, shadow::Outcome::Solved) << prepared.detail;
  ASSERT_NE(prepared.latest_state_feedback_preparation, nullptr);

  const execution::PredictedState latest_state{
    0.0, 0.0, 0.0, 2.0, 0.40, 0.10, 0.08};
  const double feedback_origin_sec =
    old_origin.control_prediction_origin_sec + 0.20;
  const auto mixed_origin = old_feedback_context.evaluate(
    shadow::LatestStateFeedbackRequest{
      prepared.latest_state_feedback_preparation, feedback_origin_sec,
      old_origin.control_prediction_origin_sec + 0.07, latest_state,
      old_origin.request.previous_input});
  ASSERT_EQ(
    mixed_origin.reason, shadow::LatestStateFeedbackReason::SolveRejected)
    << mixed_origin.detail;

  const auto feedback = shadow::build_time_aligned_feedback_problem(
    shadow::TimeAlignedFeedbackProblemRequest{
      prepared.latest_state_feedback_preparation.get(), feedback_origin_sec,
      latest_state, old_origin.request.previous_input,
      preparation_context.physical_constraint_tolerance()});
  ASSERT_EQ(
    feedback.reason,
    shadow::TimeAlignedFeedbackProblemReason::Accepted) << feedback.detail;
  ASSERT_TRUE(feedback.problem.has_value());
  ASSERT_TRUE(feedback.suffix.snapshot.has_value());
  EXPECT_EQ(feedback.suffix.consumed_stage_count, 2U);
  EXPECT_EQ(feedback.problem->horizon_steps, 1);
  EXPECT_EQ(feedback.linearization_primal.size(), 17);
  EXPECT_NEAR(
    feedback.problem->initial_state[model::kProgressIndex],
    latest_state.progress_m, 1e-12);

  const auto assembled = problem::assemble(feedback.problem.value());
  ASSERT_TRUE(assembled.has_value());
  solver::PersistentOsqpSolver feedback_solver(
    solver::ConstraintPreconditioningPolicy::RowToleranceNormalized);
  const auto solved = feedback_solver.solve(
    assembled->quadratic_cost, assembled->constraints,
    assembled->linear_cost, assembled->lower_bound, assembled->upper_bound,
    std::nullopt, assembled->variable_scaling);
  EXPECT_TRUE(solved.result.has_value()) << solved.failure_detail;
  if (solved.result.has_value()) {
    EXPECT_TRUE(solved.result->primal.allFinite());
    EXPECT_LE(solved.result->maximum_normalized_constraint_violation, 1.0);
    EXPECT_NEAR(
      solved.result->primal[model::kProgressIndex], latest_state.progress_m,
      solved.telemetry.physical_global_tolerance);
  }
}

TEST(
  MpccRateResolvedShadow,
  TimeAlignedFeedbackSolverProducesOneConsistentSuffixArtifact)
{
  shadow::SolverContext preparation_context;
  shadow::LatestStateFeedbackSolverContext feedback_context;
  auto old_origin = snapshot();
  old_origin.request = narrow_progress_request();
  const auto prepared = preparation_context.evaluate(old_origin);
  ASSERT_EQ(prepared.outcome, shadow::Outcome::Solved) << prepared.detail;
  ASSERT_NE(prepared.latest_state_feedback_preparation, nullptr);

  const execution::PredictedState latest_state{
    0.0, 0.0, 0.0, 2.0, 0.40, 0.10, 0.08};
  const double feedback_origin_sec =
    old_origin.control_prediction_origin_sec + 0.20;
  const shadow::LatestStateFeedbackRequest request{
    prepared.latest_state_feedback_preparation, feedback_origin_sec,
    old_origin.control_prediction_origin_sec + 0.07, latest_state,
    old_origin.request.previous_input};

  const auto mixed_origin = feedback_context.evaluate(request);
  ASSERT_EQ(
    mixed_origin.reason, shadow::LatestStateFeedbackReason::SolveRejected)
    << mixed_origin.detail;

  const auto time_aligned = feedback_context.evaluate_time_aligned(request);
  ASSERT_EQ(
    time_aligned.reason, shadow::LatestStateFeedbackReason::Accepted)
    << time_aligned.detail;
  EXPECT_TRUE(time_aligned.time_aligned_suffix_attempted);
  EXPECT_EQ(
    time_aligned.time_aligned_problem_reason,
    shadow::TimeAlignedFeedbackProblemReason::Accepted);
  EXPECT_EQ(time_aligned.consumed_stage_count, 2U);
  EXPECT_NEAR(
    time_aligned.first_remaining_stage_duration_sec, 0.10, 1e-12);
  ASSERT_NE(time_aligned.execution_artifact, nullptr);
  EXPECT_EQ(
    execution::validate(*time_aligned.execution_artifact),
    execution::RejectReason::None);
  EXPECT_EQ(time_aligned.execution_artifact->control_stages.size(), 1U);
  EXPECT_NEAR(
    time_aligned.execution_artifact->predicted_states.front().progress_m,
    latest_state.progress_m,
    time_aligned.execution_artifact->physical_global_tolerance);
  EXPECT_DOUBLE_EQ(
    time_aligned.execution_artifact->prediction_origin_sec,
    feedback_origin_sec);
  const auto exact = physical::build(
    *time_aligned.execution_artifact, contract::ControlIntent::Track,
    time_aligned.execution_artifact->identity.source_context.stage_geometry_id);
  EXPECT_EQ(exact.reason, physical::RejectReason::None);
  EXPECT_TRUE(exact.exact_trajectory.has_value());
}

TEST(
  MpccRateResolvedShadow,
  ReachableBridgeRebuildsTheTangentByCanonicalNonlinearRollout)
{
  shadow::SolverContext preparation_context;
  auto old_origin = snapshot();
  const auto prepared = preparation_context.evaluate(old_origin);
  ASSERT_EQ(prepared.outcome, shadow::Outcome::Solved) << prepared.detail;
  ASSERT_NE(prepared.latest_state_feedback_preparation, nullptr);
  auto preparation = *prepared.latest_state_feedback_preparation;
  problem::ProgressAlignedWallConstraints progress_wall;
  progress_wall.lower_slope = {0.0, 0.0, 0.0};
  progress_wall.lower_intercept = {-5.0, -5.0, -5.0};
  progress_wall.upper_slope = {0.0, 0.0, 0.0};
  progress_wall.upper_intercept = {5.0, 5.0, 5.0};
  preparation.final_problem.progress_aligned_wall_constraints =
    std::move(progress_wall);
  preparation.final_problem.swept_lateral_wall_constraints = {
    {0, 0.5, -5.0, 5.0}, {1, 0.5, -5.0, 5.0},
    {2, 0.5, -5.0, 5.0}};
  preparation.final_problem.dynamic_obstacle_constraints = {
    {1, problem::DynamicObstacleConstraintAxis::EffectiveProgress,
      -5.0, 5.0, 0.0, 0.0},
    {2, problem::DynamicObstacleConstraintAxis::EffectiveProgress,
      -5.0, 5.0, 0.0, 0.0},
    {3, problem::DynamicObstacleConstraintAxis::EffectiveProgress,
      -5.0, 5.0, 0.0, 0.0}};

  const execution::PredictedState latest_state{
    0.0, 0.0, 0.0, 2.0, 0.04, 0.32, 0.26};
  const double feedback_origin_sec =
    old_origin.control_prediction_origin_sec + 0.02;
  const shadow::TimeAlignedFeedbackProblemRequest request{
    &preparation, feedback_origin_sec,
    latest_state, old_origin.request.previous_input,
    preparation_context.physical_constraint_tolerance()};

  const auto direct = shadow::build_time_aligned_feedback_problem(request);
  ASSERT_EQ(
    direct.reason, shadow::TimeAlignedFeedbackProblemReason::Accepted)
    << direct.detail;
  ASSERT_TRUE(direct.problem.has_value());
  ASSERT_TRUE(direct.suffix.snapshot.has_value());
  const auto bridge =
    shadow::build_reachable_bridge_feedback_problem(request);
  ASSERT_EQ(bridge.reason, shadow::ReachableBridgeReason::Accepted)
    << bridge.detail;
  ASSERT_TRUE(bridge.problem.has_value());
  ASSERT_TRUE(bridge.suffix.snapshot.has_value());
  EXPECT_EQ(
    bridge.rollout_stage_count,
    static_cast<std::size_t>(bridge.problem->horizon_steps));
  EXPECT_GT(bridge.maximum_direct_successor_gap, 0.05);

  const auto & direct_problem = direct.problem.value();
  const auto & bridge_problem = bridge.problem.value();
  EXPECT_TRUE(
    bridge_problem.state_reference.isApprox(
      direct_problem.state_reference, 0.0));
  EXPECT_TRUE(
    bridge_problem.state_lower.isApprox(direct_problem.state_lower, 0.0));
  EXPECT_TRUE(
    bridge_problem.state_upper.isApprox(direct_problem.state_upper, 0.0));
  EXPECT_TRUE(
    bridge_problem.input_reference.isApprox(
      direct_problem.input_reference, 0.0));
  EXPECT_TRUE(
    bridge_problem.input_lower.isApprox(direct_problem.input_lower, 0.0));
  EXPECT_TRUE(
    bridge_problem.input_upper.isApprox(direct_problem.input_upper, 0.0));
  EXPECT_EQ(
    bridge_problem.swept_lateral_wall_constraints.size(),
    direct_problem.swept_lateral_wall_constraints.size());
  EXPECT_EQ(
    bridge_problem.dynamic_obstacle_constraints.size(),
    direct_problem.dynamic_obstacle_constraints.size());
  ASSERT_TRUE(
    bridge_problem.progress_aligned_wall_constraints.has_value());
  ASSERT_TRUE(
    direct_problem.progress_aligned_wall_constraints.has_value());
  EXPECT_EQ(
    bridge_problem.progress_aligned_wall_constraints->lower_intercept,
    direct_problem.progress_aligned_wall_constraints->lower_intercept);
  EXPECT_EQ(
    bridge_problem.progress_aligned_wall_constraints->upper_intercept,
    direct_problem.progress_aligned_wall_constraints->upper_intercept);
  for (std::size_t index = 0U;
    index < bridge_problem.swept_lateral_wall_constraints.size(); ++index)
  {
    const auto & lhs = bridge_problem.swept_lateral_wall_constraints[index];
    const auto & rhs = direct_problem.swept_lateral_wall_constraints[index];
    EXPECT_EQ(lhs.transition_stage, rhs.transition_stage);
    EXPECT_DOUBLE_EQ(lhs.destination_ratio, rhs.destination_ratio);
    EXPECT_DOUBLE_EQ(lhs.lower_m, rhs.lower_m);
    EXPECT_DOUBLE_EQ(lhs.upper_m, rhs.upper_m);
  }
  for (std::size_t index = 0U;
    index < bridge_problem.dynamic_obstacle_constraints.size(); ++index)
  {
    const auto & lhs = bridge_problem.dynamic_obstacle_constraints[index];
    const auto & rhs = direct_problem.dynamic_obstacle_constraints[index];
    EXPECT_EQ(lhs.state_stage, rhs.state_stage);
    EXPECT_EQ(lhs.axis, rhs.axis);
    EXPECT_DOUBLE_EQ(lhs.lower, rhs.lower);
    EXPECT_DOUBLE_EQ(lhs.upper, rhs.upper);
  }

  constexpr int nx = model::kStateDimension;
  constexpr int nu = model::kInputDimension;
  const int horizon = bridge_problem.horizon_steps;
  const int state_values = nx * (horizon + 1);
  for (int stage = 0; stage < horizon; ++stage) {
    const auto state =
      bridge.linearization_primal.segment<nx>(stage * nx);
    const auto input = bridge.linearization_primal.segment<nu>(
      state_values + stage * nu);
    const auto & semantic_input =
      bridge.suffix.snapshot->request.inputs[static_cast<std::size_t>(stage)];
    const auto transition = model::evaluate_temporal_frenet_transition(
      model::LinearizationRequest{
        state[model::kLateralIndex], state[model::kLagIndex],
        state[model::kHeadingIndex], state[model::kVelocityIndex],
        state[model::kProgressIndex], state[model::kSteeringIndex],
        state[model::kResponseSteeringIndex],
        input[model::kAccelerationIndex],
        input[model::kSteeringRateIndex],
        input[model::kVirtualProgressSpeedIndex],
        semantic_input.path_curvature_radpm,
        bridge.suffix.snapshot->request.wheelbase_m,
        bridge.suffix.snapshot->request.yaw_response_gain,
        bridge.suffix.snapshot->request.yaw_response_time_constant_sec,
        semantic_input.stage_dt_sec,
        bridge.suffix.snapshot->request.minimum_frenet_denominator,
        bridge.suffix.snapshot->request.minimum_stage_dt_sec,
        bridge.suffix.snapshot->request.maximum_stage_dt_sec});
    ASSERT_TRUE(transition.has_value());
    EXPECT_TRUE(
      transition->next_state.isApprox(
        bridge.linearization_primal.segment<nx>((stage + 1) * nx),
        1.0e-12));
  }
}

TEST(
  MpccRateResolvedShadow,
  ReachableBridgeEvaluatorRemainsObservationOnlyAndProducesAnArtifact)
{
  shadow::SolverContext preparation_context;
  shadow::LatestStateFeedbackSolverContext feedback_context;
  auto old_origin = snapshot();
  const auto prepared = preparation_context.evaluate(old_origin);
  ASSERT_EQ(prepared.outcome, shadow::Outcome::Solved) << prepared.detail;
  ASSERT_NE(prepared.latest_state_feedback_preparation, nullptr);

  const execution::PredictedState latest_state{
    0.0, 0.0, 0.0, 2.0, 0.04, 0.32, 0.26};
  const shadow::LatestStateFeedbackRequest request{
    prepared.latest_state_feedback_preparation,
    old_origin.control_prediction_origin_sec + 0.02,
    old_origin.control_prediction_origin_sec + 0.01,
    latest_state, old_origin.request.previous_input};

  const auto result =
    feedback_context.evaluate_reachable_bridge_time_aligned(request);
  EXPECT_TRUE(result.time_aligned_suffix_attempted);
  EXPECT_TRUE(result.reachable_bridge_attempted);
  EXPECT_TRUE(result.reachable_bridge_applied);
  EXPECT_EQ(result.reachable_bridge_reason, shadow::ReachableBridgeReason::Accepted);
  EXPECT_EQ(result.reason, shadow::LatestStateFeedbackReason::Accepted)
    << result.detail;
  EXPECT_EQ(
    result.physical_exact_reason,
    multi_purpose_mpc_ros::race_mpcc_foundation::
    ExactPhysicalExecutionTrajectoryReason::Accepted);
  EXPECT_EQ(result.physical_rejected_stage, -1);
  EXPECT_DOUBLE_EQ(result.physical_lateral_violation_m, 0.0);
  ASSERT_NE(result.execution_artifact, nullptr);
  EXPECT_EQ(
    execution::validate(*result.execution_artifact),
    execution::RejectReason::None);
}

TEST(
  MpccRateResolvedShadow,
  NonlinearInteriorWallRowsPreserveTheOriginalProblemAsAnExactPrefix)
{
  shadow::SolverContext preparation_context;
  const auto source = snapshot();
  const auto prepared = preparation_context.evaluate(source);
  ASSERT_EQ(prepared.outcome, shadow::Outcome::Solved) << prepared.detail;
  ASSERT_NE(prepared.latest_state_feedback_preparation, nullptr);
  const shadow::TimeAlignedFeedbackProblemRequest request{
    prepared.latest_state_feedback_preparation.get(),
    source.control_prediction_origin_sec + 0.02,
    execution::PredictedState{0.0, 0.0, 0.0, 2.0, 0.04, 0.10, 0.08},
    source.request.previous_input,
    solver::PersistentOsqpSolver{}.physical_constraint_tolerance()};
  const auto bridge = shadow::build_reachable_bridge_feedback_problem(request);
  ASSERT_EQ(bridge.reason, shadow::ReachableBridgeReason::Accepted)
    << bridge.detail;
  ASSERT_TRUE(bridge.problem.has_value());
  const auto original = problem::assemble(bridge.problem.value());
  ASSERT_TRUE(original.has_value());

  const auto augmented = shadow::build_nonlinear_interior_wall_problem(
    bridge.suffix.snapshot.value(), bridge.problem.value(),
    bridge.linearization_primal);
  ASSERT_EQ(
    augmented.reason, shadow::NonlinearInteriorWallReason::Accepted)
    << augmented.detail;
  ASSERT_TRUE(augmented.problem.has_value());
  EXPECT_EQ(
    augmented.original_row_count,
    static_cast<std::size_t>(original->constraints.rows()));
  std::size_t expected_interior_rows{};
  for (const auto & input : bridge.suffix.snapshot->request.inputs) {
    expected_interior_rows += static_cast<std::size_t>(std::max(
      1.0, std::ceil(
        input.stage_dt_sec /
        model::kMaximumPhysicalIntegrationStepSec))) - 1U;
  }
  EXPECT_EQ(augmented.appended_row_count, expected_interior_rows);
  EXPECT_EQ(
    augmented.problem->constraints.rows(),
    original->constraints.rows() +
    static_cast<int>(augmented.appended_row_count));
  EXPECT_TRUE(
    Eigen::MatrixXd(augmented.problem->constraints).
    topRows(original->constraints.rows()).isApprox(
      Eigen::MatrixXd(original->constraints), 0.0));
  EXPECT_TRUE(
    augmented.problem->lower_bound.head(original->lower_bound.size()).isApprox(
      original->lower_bound, 0.0));
  EXPECT_TRUE(
    augmented.problem->upper_bound.head(original->upper_bound.size()).isApprox(
      original->upper_bound, 0.0));
  const Eigen::VectorXd row_values =
    augmented.problem->constraints * bridge.linearization_primal;
  const auto appended_values = row_values.tail(
    static_cast<Eigen::Index>(augmented.appended_row_count));
  const auto appended_lower = augmented.problem->lower_bound.tail(
    static_cast<Eigen::Index>(augmented.appended_row_count));
  const auto appended_upper = augmented.problem->upper_bound.tail(
    static_cast<Eigen::Index>(augmented.appended_row_count));
  EXPECT_TRUE((appended_values.array() >= appended_lower.array()).all());
  EXPECT_TRUE((appended_values.array() <= appended_upper.array()).all());
  EXPECT_DOUBLE_EQ(augmented.maximum_candidate_violation_m, 0.0);
}

TEST(
  MpccRateResolvedShadow,
  PhysicalProofDenseSampleMappingDistinguishesInteriorAndEndpointSamples)
{
  auto source = snapshot();
  source.request.horizon_steps = 2;
  source.request.inputs.resize(2U);
  source.request.inputs[0].stage_dt_sec = 0.025;
  source.request.inputs[1].stage_dt_sec = 0.01;

  const auto first = shadow::locate_physical_proof_sample(source, 0);
  ASSERT_EQ(first.reason, shadow::PhysicalProofSampleReason::Accepted);
  ASSERT_TRUE(first.sample.has_value());
  EXPECT_EQ(first.sample->transition_stage, 0);
  EXPECT_EQ(first.sample->substep_index, 1U);
  EXPECT_EQ(first.sample->substep_count, 3U);

  const auto second = shadow::locate_physical_proof_sample(source, 1);
  ASSERT_EQ(second.reason, shadow::PhysicalProofSampleReason::Accepted);
  ASSERT_TRUE(second.sample.has_value());
  EXPECT_EQ(second.sample->transition_stage, 0);
  EXPECT_EQ(second.sample->substep_index, 2U);

  const auto first_endpoint =
    shadow::locate_physical_proof_sample(source, 2);
  EXPECT_EQ(
    first_endpoint.reason, shadow::PhysicalProofSampleReason::EndpointSample);
  ASSERT_TRUE(first_endpoint.sample.has_value());
  EXPECT_EQ(first_endpoint.sample->transition_stage, 0);
  EXPECT_EQ(first_endpoint.sample->substep_index, 3U);

  const auto second_endpoint =
    shadow::locate_physical_proof_sample(source, 3);
  EXPECT_EQ(
    second_endpoint.reason, shadow::PhysicalProofSampleReason::EndpointSample);
  ASSERT_TRUE(second_endpoint.sample.has_value());
  EXPECT_EQ(second_endpoint.sample->transition_stage, 1);
  EXPECT_EQ(second_endpoint.sample->substep_index, 1U);

  EXPECT_EQ(
    shadow::locate_physical_proof_sample(source, 4).reason,
    shadow::PhysicalProofSampleReason::OutOfRange);
  EXPECT_EQ(
    shadow::locate_physical_proof_sample(source, -1).reason,
    shadow::PhysicalProofSampleReason::InvalidRequest);
}

TEST(
  MpccRateResolvedShadow,
  StructuredInteriorWallRowsReplaceTheAffineSweptRowsWithinTheirBudget)
{
  shadow::SolverContext preparation_context;
  const auto source = snapshot();
  const auto prepared = preparation_context.evaluate(source);
  ASSERT_EQ(prepared.outcome, shadow::Outcome::Solved) << prepared.detail;
  ASSERT_NE(prepared.latest_state_feedback_preparation, nullptr);
  const shadow::TimeAlignedFeedbackProblemRequest request{
    prepared.latest_state_feedback_preparation.get(),
    source.control_prediction_origin_sec + 0.02,
    execution::PredictedState{0.0, 0.0, 0.0, 2.0, 0.04, 0.10, 0.08},
    source.request.previous_input,
    solver::PersistentOsqpSolver{}.physical_constraint_tolerance()};
  auto bridge = shadow::build_reachable_bridge_feedback_problem(request);
  ASSERT_EQ(bridge.reason, shadow::ReachableBridgeReason::Accepted)
    << bridge.detail;
  ASSERT_TRUE(bridge.problem.has_value());
  ASSERT_TRUE(bridge.suffix.snapshot.has_value());
  constexpr std::size_t kSweptRowsPerTransition = 4U;
  for (int stage = 0; stage < bridge.problem->horizon_steps; ++stage) {
    for (std::size_t sample = 1U;
      sample <= kSweptRowsPerTransition; ++sample)
    {
      bridge.problem->swept_lateral_wall_constraints.push_back(
        problem::SweptLateralWallConstraint{
          stage,
          static_cast<double>(sample) /
          static_cast<double>(kSweptRowsPerTransition + 1U),
          -std::numeric_limits<double>::infinity(),
          std::numeric_limits<double>::infinity()});
    }
  }
  const auto original = problem::assemble(bridge.problem.value());
  ASSERT_TRUE(original.has_value());
  const auto old_swept_rows =
    bridge.problem->swept_lateral_wall_constraints.size();
  ASSERT_GT(old_swept_rows, 0U);

  const auto structured =
    shadow::build_structured_nonlinear_interior_wall_problem(
    bridge.suffix.snapshot.value(), bridge.problem.value(),
    bridge.linearization_primal);
  ASSERT_EQ(
    structured.reason, shadow::NonlinearInteriorWallReason::Accepted)
    << structured.detail;
  ASSERT_TRUE(structured.problem.has_value());
  EXPECT_EQ(structured.replaced_row_count, old_swept_rows);
  EXPECT_GT(structured.appended_row_count, 0U);
  EXPECT_LE(structured.appended_row_count, old_swept_rows);
  EXPECT_EQ(
    structured.problem->constraints.rows(),
    original->constraints.rows() - static_cast<int>(old_swept_rows) +
    static_cast<int>(structured.appended_row_count));
  EXPECT_EQ(
    structured.problem->constraints.cols(), original->constraints.cols());
  EXPECT_EQ(
    structured.problem->quadratic_cost.nonZeros(),
    original->quadratic_cost.nonZeros());
  EXPECT_TRUE(
    structured.problem->linear_cost.isApprox(original->linear_cost, 0.0));
}

TEST(
  MpccRateResolvedShadow,
  SelectedNonlinearInteriorWallCutPreservesTheOriginalProblemAsExactPrefix)
{
  shadow::SolverContext preparation_context;
  const auto source = snapshot();
  const auto prepared = preparation_context.evaluate(source);
  ASSERT_EQ(prepared.outcome, shadow::Outcome::Solved) << prepared.detail;
  ASSERT_NE(prepared.latest_state_feedback_preparation, nullptr);
  const shadow::TimeAlignedFeedbackProblemRequest request{
    prepared.latest_state_feedback_preparation.get(),
    source.control_prediction_origin_sec + 0.02,
    execution::PredictedState{0.0, 0.0, 0.0, 2.0, 0.04, 0.10, 0.08},
    source.request.previous_input,
    solver::PersistentOsqpSolver{}.physical_constraint_tolerance()};
  const auto bridge = shadow::build_reachable_bridge_feedback_problem(request);
  ASSERT_EQ(bridge.reason, shadow::ReachableBridgeReason::Accepted)
    << bridge.detail;
  ASSERT_TRUE(bridge.problem.has_value());
  ASSERT_TRUE(bridge.suffix.snapshot.has_value());
  const auto original = problem::assemble(bridge.problem.value());
  ASSERT_TRUE(original.has_value());
  const double duration_sec =
    bridge.suffix.snapshot->request.inputs.front().stage_dt_sec;
  const auto substep_count = static_cast<std::size_t>(std::max(
    1.0, std::ceil(
      duration_sec / model::kMaximumPhysicalIntegrationStepSec)));
  ASSERT_GT(substep_count, 1U);

  const auto augmented =
    shadow::build_selected_nonlinear_interior_wall_problem(
    bridge.suffix.snapshot.value(), bridge.problem.value(),
    bridge.linearization_primal,
    {shadow::NonlinearInteriorWallSample{0, 1U, substep_count}});
  ASSERT_EQ(
    augmented.reason, shadow::NonlinearInteriorWallReason::Accepted)
    << augmented.detail;
  ASSERT_TRUE(augmented.problem.has_value());
  EXPECT_EQ(augmented.appended_row_count, 1U);
  EXPECT_TRUE(
    Eigen::MatrixXd(augmented.problem->constraints).
    topRows(original->constraints.rows()).isApprox(
      Eigen::MatrixXd(original->constraints), 0.0));
  EXPECT_TRUE(
    augmented.problem->lower_bound.head(original->lower_bound.size()).isApprox(
      original->lower_bound, 0.0));
  EXPECT_TRUE(
    augmented.problem->upper_bound.head(original->upper_bound.size()).isApprox(
      original->upper_bound, 0.0));
}

TEST(
  MpccRateResolvedShadow,
  NonlinearInteriorWallAuditRemainsObservationOnlyAndExactlyProved)
{
  shadow::SolverContext preparation_context;
  const auto source = snapshot();
  const auto prepared = preparation_context.evaluate(source);
  ASSERT_EQ(prepared.outcome, shadow::Outcome::Solved) << prepared.detail;
  ASSERT_NE(prepared.latest_state_feedback_preparation, nullptr);
  const shadow::LatestStateFeedbackRequest request{
    prepared.latest_state_feedback_preparation,
    source.control_prediction_origin_sec + 0.02,
    source.control_prediction_origin_sec + 0.01,
    execution::PredictedState{0.0, 0.0, 0.0, 2.0, 0.04, 0.10, 0.08},
    source.request.previous_input};
  shadow::LatestStateFeedbackSolverContext context;

  const auto result =
    context.evaluate_reachable_bridge_nonlinear_interior_wall_audit(
    request, 4U);
  EXPECT_TRUE(result.nonlinear_interior_wall_audit_requested);
  EXPECT_TRUE(result.nonlinear_interior_wall_audit_applied);
  EXPECT_EQ(
    result.nonlinear_interior_wall_reason,
    shadow::NonlinearInteriorWallReason::Accepted);
  EXPECT_GT(result.nonlinear_interior_wall_row_count, 0U);
  EXPECT_EQ(result.reason, shadow::LatestStateFeedbackReason::Accepted)
    << result.detail;
  ASSERT_NE(result.execution_artifact, nullptr);
  EXPECT_EQ(
    execution::validate(*result.execution_artifact),
    execution::RejectReason::None);
}

TEST(
  MpccRateResolvedShadow,
  StructuredInteriorWallAuditRejectsSnapshotsWithoutSweptWallOwnership)
{
  shadow::SolverContext preparation_context;
  const auto source = snapshot();
  const auto prepared = preparation_context.evaluate(source);
  ASSERT_EQ(prepared.outcome, shadow::Outcome::Solved) << prepared.detail;
  ASSERT_NE(prepared.latest_state_feedback_preparation, nullptr);
  const shadow::LatestStateFeedbackRequest request{
    prepared.latest_state_feedback_preparation,
    source.control_prediction_origin_sec + 0.02,
    source.control_prediction_origin_sec + 0.01,
    execution::PredictedState{0.0, 0.0, 0.0, 2.0, 0.04, 0.10, 0.08},
    source.request.previous_input};
  shadow::LatestStateFeedbackSolverContext context;

  const auto result =
    context.evaluate_reachable_bridge_structured_interior_wall_audit(
    request, 4U);
  EXPECT_TRUE(result.structured_interior_wall_audit_requested);
  EXPECT_FALSE(result.structured_interior_wall_audit_applied);
  EXPECT_FALSE(result.nonlinear_interior_wall_audit_requested);
  EXPECT_FALSE(result.nonlinear_interior_wall_audit_applied);
  EXPECT_EQ(
    result.nonlinear_interior_wall_reason,
    shadow::NonlinearInteriorWallReason::InvalidRequest);
  EXPECT_EQ(result.nonlinear_interior_wall_row_count, 0U);
  EXPECT_EQ(result.reason, shadow::LatestStateFeedbackReason::AssemblyRejected)
    << result.detail;
  EXPECT_EQ(result.execution_artifact, nullptr);
}

TEST(
  MpccRateResolvedShadow,
  PhysicalProofCutPlaneAuditDoesNotAddCutsWhenBaseProofIsAccepted)
{
  shadow::SolverContext preparation_context;
  const auto source = snapshot();
  const auto prepared = preparation_context.evaluate(source);
  ASSERT_EQ(prepared.outcome, shadow::Outcome::Solved) << prepared.detail;
  ASSERT_NE(prepared.latest_state_feedback_preparation, nullptr);
  const shadow::LatestStateFeedbackRequest request{
    prepared.latest_state_feedback_preparation,
    source.control_prediction_origin_sec + 0.02,
    source.control_prediction_origin_sec + 0.01,
    execution::PredictedState{0.0, 0.0, 0.0, 2.0, 0.04, 0.10, 0.08},
    source.request.previous_input};
  shadow::LatestStateFeedbackSolverContext context;

  const auto result =
    context.evaluate_reachable_bridge_physical_proof_cut_plane_audit(
    request, 4U);
  EXPECT_TRUE(result.physical_proof_cut_plane_audit_requested);
  EXPECT_FALSE(result.physical_proof_cut_plane_audit_applied);
  EXPECT_EQ(result.physical_proof_cut_count, 0U);
  EXPECT_EQ(result.reason, shadow::LatestStateFeedbackReason::Accepted)
    << result.detail;
  ASSERT_NE(result.execution_artifact, nullptr);
  EXPECT_EQ(
    execution::validate(*result.execution_artifact),
    execution::RejectReason::None);
}

TEST(
  MpccRateResolvedShadow,
  ReachableBridgeMultiSqpOneIterationIsTheSameCandidateAsC)
{
  shadow::SolverContext preparation_context;
  auto old_origin = snapshot();
  const auto prepared = preparation_context.evaluate(old_origin);
  ASSERT_EQ(prepared.outcome, shadow::Outcome::Solved) << prepared.detail;
  ASSERT_NE(prepared.latest_state_feedback_preparation, nullptr);

  const execution::PredictedState latest_state{
    0.0, 0.0, 0.0, 2.0, 0.04, 0.32, 0.26};
  const shadow::LatestStateFeedbackRequest request{
    prepared.latest_state_feedback_preparation,
    old_origin.control_prediction_origin_sec + 0.02,
    old_origin.control_prediction_origin_sec + 0.01,
    latest_state, old_origin.request.previous_input};
  shadow::LatestStateFeedbackSolverContext c_context;
  shadow::LatestStateFeedbackSolverContext d_context;
  const auto c = c_context.evaluate_reachable_bridge_time_aligned(request);
  const auto d =
    d_context.evaluate_reachable_bridge_multi_sqp_audit(request, 1U);

  ASSERT_EQ(c.reason, shadow::LatestStateFeedbackReason::Accepted) << c.detail;
  ASSERT_EQ(d.reason, c.reason) << d.detail;
  EXPECT_EQ(d.latest_state_multi_sqp_attempt_count, 1U);
  EXPECT_EQ(d.latest_state_multi_sqp_solve_count, 1U);
  EXPECT_EQ(d.physical_adapter_reason, c.physical_adapter_reason);
  ASSERT_NE(c.execution_artifact, nullptr);
  ASSERT_NE(d.execution_artifact, nullptr);
  ASSERT_EQ(
    d.execution_artifact->predicted_states.size(),
    c.execution_artifact->predicted_states.size());
  for (std::size_t stage = 0U;
    stage < c.execution_artifact->predicted_states.size(); ++stage)
  {
    const auto & lhs = c.execution_artifact->predicted_states[stage];
    const auto & rhs = d.execution_artifact->predicted_states[stage];
    EXPECT_DOUBLE_EQ(rhs.lateral_m, lhs.lateral_m);
    EXPECT_DOUBLE_EQ(rhs.lag_m, lhs.lag_m);
    EXPECT_DOUBLE_EQ(rhs.heading_offset_rad, lhs.heading_offset_rad);
    EXPECT_DOUBLE_EQ(rhs.velocity_mps, lhs.velocity_mps);
    EXPECT_DOUBLE_EQ(rhs.progress_m, lhs.progress_m);
    EXPECT_DOUBLE_EQ(rhs.steering_rad, lhs.steering_rad);
    EXPECT_DOUBLE_EQ(
      rhs.response_steering_rad, lhs.response_steering_rad);
  }
}

TEST(
  MpccRateResolvedShadow,
  ReachableBridgeMultiSqpRejectsAnUnboundedAuditRequest)
{
  shadow::SolverContext preparation_context;
  const auto old_origin = snapshot();
  const auto prepared = preparation_context.evaluate(old_origin);
  ASSERT_EQ(prepared.outcome, shadow::Outcome::Solved) << prepared.detail;
  ASSERT_NE(prepared.latest_state_feedback_preparation, nullptr);
  const shadow::LatestStateFeedbackRequest request{
    prepared.latest_state_feedback_preparation,
    old_origin.control_prediction_origin_sec + 0.02,
    old_origin.control_prediction_origin_sec + 0.01,
    execution::PredictedState{0.0, 0.0, 0.0, 2.0, 0.04, 0.10, 0.08},
    old_origin.request.previous_input};
  shadow::LatestStateFeedbackSolverContext context;

  const auto zero =
    context.evaluate_reachable_bridge_multi_sqp_audit(request, 0U);
  const auto excessive = context.evaluate_reachable_bridge_multi_sqp_audit(
    request, shadow::kMaximumLatestStateMultiSqpAuditIterations + 1U);
  EXPECT_EQ(zero.reason, shadow::LatestStateFeedbackReason::InvalidRequest);
  EXPECT_FALSE(zero.solve_attempted);
  EXPECT_EQ(
    excessive.reason, shadow::LatestStateFeedbackReason::InvalidRequest);
  EXPECT_FALSE(excessive.solve_attempted);
}

TEST(
  MpccRateResolvedShadow,
  ReachableBridgeSeparatesDirectSuffixFailureFromPhysicalProof)
{
  shadow::SolverContext preparation_context;
  auto old_origin = snapshot();
  old_origin.request = straight_request(20);
  old_origin.execution_prefix_steps = old_origin.request.horizon_steps;
  old_origin.nominal_path_distance_m.resize(21U);
  for (std::size_t stage = 0U;
    stage < old_origin.nominal_path_distance_m.size(); ++stage)
  {
    old_origin.nominal_path_distance_m[stage] =
      0.2 * static_cast<double>(stage);
  }
  auto context = old_origin.identity.source_context;
  context.horizon_steps = 20U;
  old_origin.identity.source_context =
    contract::seal_problem_context(std::move(context));
  const auto prepared = preparation_context.evaluate(old_origin);
  ASSERT_EQ(prepared.outcome, shadow::Outcome::Solved) << prepared.detail;
  ASSERT_NE(prepared.latest_state_feedback_preparation, nullptr);

  const execution::PredictedState latest_state{
    0.0, 0.0, -0.35, 2.0, 0.04, -0.55, -0.44};
  const shadow::LatestStateFeedbackRequest request{
    prepared.latest_state_feedback_preparation,
    old_origin.control_prediction_origin_sec + 0.02,
    old_origin.control_prediction_origin_sec + 0.01,
    latest_state, old_origin.request.previous_input};
  shadow::LatestStateFeedbackSolverContext direct_context;
  shadow::LatestStateFeedbackSolverContext bridge_context;
  shadow::LatestStateFeedbackSolverContext multi_sqp_context;
  const auto direct = direct_context.evaluate_time_aligned(request);
  const auto bridge =
    bridge_context.evaluate_reachable_bridge_time_aligned(request);
  const auto multi_sqp =
    multi_sqp_context.evaluate_reachable_bridge_multi_sqp_audit(request, 4U);

  EXPECT_EQ(direct.reason, shadow::LatestStateFeedbackReason::SolveRejected)
    << direct.detail;
  EXPECT_TRUE(bridge.reachable_bridge_applied) << bridge.detail;
  EXPECT_TRUE(bridge.solved) << bridge.detail;
  EXPECT_EQ(
    bridge.reason,
    shadow::LatestStateFeedbackReason::PhysicalAdapterRejected)
    << bridge.detail;
  EXPECT_EQ(
    bridge.physical_adapter_reason,
    physical::RejectReason::ExactTrajectoryRejected);
  EXPECT_NE(
    bridge.physical_exact_reason,
    multi_purpose_mpc_ros::race_mpcc_foundation::
    ExactPhysicalExecutionTrajectoryReason::Accepted);
  EXPECT_GE(bridge.physical_rejected_stage, 0);
  EXPECT_TRUE(multi_sqp.latest_state_multi_sqp_audit_requested);
  EXPECT_EQ(multi_sqp.latest_state_multi_sqp_attempt_count, 2U);
  EXPECT_EQ(multi_sqp.latest_state_multi_sqp_solve_count, 1U);
  EXPECT_EQ(multi_sqp.reason, shadow::LatestStateFeedbackReason::SolveRejected)
    << multi_sqp.detail;
  EXPECT_NE(multi_sqp.detail.find("primal infeasible"), std::string::npos);
  EXPECT_EQ(multi_sqp.execution_artifact, nullptr);
}

TEST(
  MpccRateResolvedShadow,
  TimeAlignedPreparedProblemMovesEveryRefinementRowWithOneClock)
{
  shadow::SolverContext preparation_context;
  const auto source = snapshot();
  const auto evaluated = preparation_context.evaluate(source);
  ASSERT_EQ(evaluated.outcome, shadow::Outcome::Solved) << evaluated.detail;
  ASSERT_NE(evaluated.latest_state_feedback_preparation, nullptr);
  auto preparation = *evaluated.latest_state_feedback_preparation;
  problem::ProgressAlignedWallConstraints progress_wall;
  progress_wall.lower_slope = {0.0, 0.0, 0.0};
  progress_wall.lower_intercept = {-1.0, -0.9, -0.8};
  progress_wall.upper_slope = {0.0, 0.0, 0.0};
  progress_wall.upper_intercept = {1.0, 0.9, 0.8};
  preparation.final_problem.progress_aligned_wall_constraints =
    std::move(progress_wall);
  preparation.final_problem.swept_lateral_wall_constraints = {
    {0, 0.5, -1.0, 1.0},
    {1, 0.5, -0.9, 0.9},
    {2, 0.5, -0.8, 0.8}};
  preparation.final_problem.dynamic_obstacle_constraints = {
    {1, problem::DynamicObstacleConstraintAxis::EffectiveProgress,
      -1.0, 4.0, 0.0, 0.0},
    {2, problem::DynamicObstacleConstraintAxis::EffectiveProgress,
      -0.5, 4.0, 0.0, 0.0},
    {3, problem::DynamicObstacleConstraintAxis::EffectiveProgress,
      0.0, 4.0, 0.0, 0.0}};
  const execution::PredictedState latest_state{
    0.0, 0.0, 0.0, 2.0, 0.22, 0.10, 0.08};

  const auto feedback = shadow::build_time_aligned_feedback_problem(
    shadow::TimeAlignedFeedbackProblemRequest{
      &preparation, source.control_prediction_origin_sec + 0.11,
      latest_state, source.request.previous_input,
      preparation_context.physical_constraint_tolerance()});

  ASSERT_EQ(
    feedback.reason,
    shadow::TimeAlignedFeedbackProblemReason::Accepted) << feedback.detail;
  ASSERT_TRUE(feedback.problem.has_value());
  ASSERT_TRUE(
    feedback.problem->progress_aligned_wall_constraints.has_value());
  const auto & wall =
    feedback.problem->progress_aligned_wall_constraints.value();
  ASSERT_EQ(wall.lower_intercept.size(), 2U);
  EXPECT_DOUBLE_EQ(wall.lower_intercept[0], -0.9);
  EXPECT_DOUBLE_EQ(wall.lower_intercept[1], -0.8);
  ASSERT_EQ(feedback.problem->swept_lateral_wall_constraints.size(), 2U);
  EXPECT_EQ(
    feedback.problem->swept_lateral_wall_constraints[0].transition_stage, 0);
  EXPECT_EQ(
    feedback.problem->swept_lateral_wall_constraints[1].transition_stage, 1);
  ASSERT_EQ(feedback.problem->dynamic_obstacle_constraints.size(), 2U);
  EXPECT_EQ(
    feedback.problem->dynamic_obstacle_constraints[0].state_stage, 1);
  EXPECT_EQ(
    feedback.problem->dynamic_obstacle_constraints[1].state_stage, 2);
  EXPECT_TRUE(problem::assemble(feedback.problem.value()).has_value());
}

TEST(
  MpccRateResolvedShadow,
  TimeAlignedPreparedProblemRejectsMalformedRefinementProvenance)
{
  shadow::SolverContext preparation_context;
  const auto source = snapshot();
  const auto evaluated = preparation_context.evaluate(source);
  ASSERT_EQ(evaluated.outcome, shadow::Outcome::Solved) << evaluated.detail;
  ASSERT_NE(evaluated.latest_state_feedback_preparation, nullptr);
  auto preparation = *evaluated.latest_state_feedback_preparation;
  preparation.final_problem.swept_lateral_wall_constraints.push_back(
    {source.request.horizon_steps, 0.5, -1.0, 1.0});
  const execution::PredictedState latest_state{
    0.0, 0.0, 0.0, 2.0, 0.22, 0.10, 0.08};

  const auto feedback = shadow::build_time_aligned_feedback_problem(
    shadow::TimeAlignedFeedbackProblemRequest{
      &preparation, source.control_prediction_origin_sec + 0.11,
      latest_state, source.request.previous_input,
      preparation_context.physical_constraint_tolerance()});

  EXPECT_EQ(
    feedback.reason,
    shadow::TimeAlignedFeedbackProblemReason::RefinementProvenanceMismatch);
  EXPECT_FALSE(feedback.problem.has_value());
}

TEST(MpccRateResolvedShadow, RejectsDynamicWorldFromDifferentObservationEpoch)
{
  shadow::SolverContext context;
  auto input = snapshot();
  auto problem_context = input.identity.source_context;
  problem_context.intent = contract::ControlIntent::Pass;
  problem_context.intent_generation = 3U;
  problem_context.target_id = "d2";
  problem_context.target_obstacle_generation = 7U;
  problem_context.execution_side_sign = 1;
  input.identity.source_context =
    contract::seal_problem_context(std::move(problem_context));
  bind_dynamic_obstacle_identity(input, "d2", 7U, 1);
  input.dynamic_obstacle_refinement_active = true;
  input.dynamic_obstacle_pass_side_sign = 1;
  input.dynamic_obstacle_stages.assign(
    static_cast<std::size_t>(input.request.horizon_steps),
    multi_purpose_mpc_ros::mpcc_rate_resolved_dynamic_obstacle::StagePrediction{
      true, 1.0, 0.0, 0.8, 0.75});
  shadow::ReplayWorld world;
  world.current = true;
  world.observation_generation = 8U;
  input.replay_world = std::move(world);

  const auto result = context.evaluate(input);

  EXPECT_EQ(result.outcome, shadow::Outcome::AssemblyRejected);
  EXPECT_NE(
    result.detail.find("physical obstacle world does not match problem identity"),
    std::string::npos);
  EXPECT_NE(result.detail.find("expected_generation=7"), std::string::npos);
  EXPECT_NE(result.detail.find("observed_generation=8"), std::string::npos);
}

TEST(MpccRateResolvedShadow, CurrentProblemBootstrapOwnsCurrentAffineDynamics)
{
  const auto adapted = adapter::build(
    straight_request(), solver::PhysicalConstraintTolerance{1.0e-3, 1.0e-3});
  ASSERT_TRUE(adapted.has_value());
  const auto assembled = problem::assemble(adapted->problem);
  ASSERT_TRUE(assembled.has_value());
  const auto bootstrap = shadow::build_current_problem_bootstrap(
    adapted->problem,
    static_cast<std::size_t>(assembled->lower_bound.size()));
  ASSERT_TRUE(bootstrap.has_value());
  const auto & primal = bootstrap->primal;
  EXPECT_TRUE(primal.allFinite());
  EXPECT_EQ(bootstrap->dual.size(), assembled->lower_bound.size());
  EXPECT_TRUE(bootstrap->dual.isZero());
  EXPECT_TRUE(
    primal.head<model::kStateDimension>().isApprox(
      adapted->problem.initial_state));

  const int state_values = model::kStateDimension *
    (adapted->problem.horizon_steps + 1);
  for (int stage = 0; stage < adapted->problem.horizon_steps; ++stage) {
    const int state_offset = stage * model::kStateDimension;
    const int next_state_offset = (stage + 1) * model::kStateDimension;
    const int input_offset = stage * model::kInputDimension;
    const auto input = primal.segment<model::kInputDimension>(
      state_values + input_offset);
    for (int element = 0; element < model::kInputDimension; ++element) {
      EXPECT_GE(input[element], adapted->problem.input_lower[input_offset + element]);
      EXPECT_LE(input[element], adapted->problem.input_upper[input_offset + element]);
    }
    const auto & linearization =
      adapted->problem.linearizations[static_cast<std::size_t>(stage)];
    const auto equality_residual =
      -primal.segment<model::kStateDimension>(next_state_offset) +
      linearization.state_matrix *
      primal.segment<model::kStateDimension>(state_offset) +
      linearization.input_matrix * input - linearization.equality_offset;
    EXPECT_LT(equality_residual.lpNorm<Eigen::Infinity>(), 1.0e-12);
  }
}

TEST(
  MpccRateResolvedShadow,
  CurrentProblemBootstrapTransportsOnlySameProblemInputs)
{
  const auto adapted = adapter::build(
    straight_request(), solver::PhysicalConstraintTolerance{1.0e-3, 1.0e-3});
  ASSERT_TRUE(adapted.has_value());
  const auto assembled = problem::assemble(adapted->problem);
  ASSERT_TRUE(assembled.has_value());
  const int state_values = model::kStateDimension *
    (adapted->problem.horizon_steps + 1);
  const int variable_count = state_values + model::kInputDimension *
    adapted->problem.horizon_steps;
  Eigen::VectorXd preceding = Eigen::VectorXd::Constant(variable_count, 99.0);
  for (int stage = 0; stage < adapted->problem.horizon_steps; ++stage) {
    const int input_offset = stage * model::kInputDimension;
    const int primal_input = state_values + input_offset;
    for (int element = 0; element < model::kInputDimension; ++element) {
      preceding[primal_input + element] =
        adapted->problem.input_lower[input_offset + element] +
        0.25 * (adapted->problem.input_upper[input_offset + element] -
        adapted->problem.input_lower[input_offset + element]);
    }
  }

  const auto bootstrap = shadow::build_current_problem_bootstrap(
    adapted->problem,
    static_cast<std::size_t>(assembled->lower_bound.size()), &preceding);
  ASSERT_TRUE(bootstrap.has_value());
  EXPECT_TRUE(
    bootstrap->primal.head<model::kStateDimension>().isApprox(
      adapted->problem.initial_state));
  for (int stage = 0; stage < adapted->problem.horizon_steps; ++stage) {
    const int input_offset = stage * model::kInputDimension;
    const int primal_input = state_values + input_offset;
    EXPECT_TRUE(
      bootstrap->primal.segment<model::kInputDimension>(primal_input).isApprox(
        preceding.segment<model::kInputDimension>(primal_input)));
  }
  EXPECT_TRUE(bootstrap->dual.isZero());
}

TEST(MpccRateResolvedShadow, RecedingWarmStartRebasesProgressAndCurrentState)
{
  const auto previous_snapshot = snapshot(1U);
  auto current = snapshot(2U);
  current.identity.source_context.intent_generation =
    previous_snapshot.identity.source_context.intent_generation;
  current.identity.source_context.stage_geometry_id = 999U;
  current.identity.source_context =
    contract::seal_problem_context(current.identity.source_context);
  current.control_prediction_origin_sec =
    previous_snapshot.control_prediction_origin_sec + 0.025;
  current.course_progress_origin_m = 50.4;
  current.request.initial_state << 0.2, -0.1, 0.05, 2.3, 0.0;

  const auto adapted = adapter::build(
    current.request,
    multi_purpose_mpc_ros::persistent_osqp::PhysicalConstraintTolerance{
      1.0e-4, 1.0e-4});
  ASSERT_TRUE(adapted.has_value());
  const int horizon = current.request.horizon_steps;
  const int state_values =
    multi_purpose_mpc_ros::mpcc_rate_resolved::kStateDimension *
    (horizon + 1);
  const int variable_count = state_values +
    multi_purpose_mpc_ros::mpcc_rate_resolved::kInputDimension * horizon;
  Eigen::VectorXd previous_primal(variable_count);
  for (int index = 0; index < variable_count; ++index) {
    previous_primal[index] = 0.01 * static_cast<double>(index + 1);
  }
  for (int stage = 0; stage <= horizon; ++stage) {
    previous_primal[
      stage * multi_purpose_mpc_ros::mpcc_rate_resolved::kStateDimension +
      multi_purpose_mpc_ros::mpcc_rate_resolved::kProgressIndex] =
      0.3 * stage;
  }
  shadow::RecedingWarmStartSeed previous{
    previous_snapshot.identity,
    previous_snapshot.control_prediction_origin_sec,
    previous_snapshot.course_progress_origin_m,
    {0.10, 0.10, 0.10},
    previous_primal};

  const auto resolution = shadow::resolve_receding_warm_start(
    previous, current, adapted->problem, 123U);

  ASSERT_EQ(resolution.reason, shadow::RecedingWarmStartReason::Available);
  ASSERT_TRUE(resolution.warm_start.has_value());
  EXPECT_EQ(resolution.stage_advance, 0U);
  EXPECT_EQ(resolution.warm_start->dual.size(), 123);
  EXPECT_TRUE(resolution.warm_start->dual.isZero());
  EXPECT_TRUE(
    resolution.warm_start->primal.head<
      multi_purpose_mpc_ros::mpcc_rate_resolved::kStateDimension>().isApprox(
      adapted->problem.initial_state));
  EXPECT_NEAR(
    resolution.warm_start->primal[
      multi_purpose_mpc_ros::mpcc_rate_resolved::kStateDimension +
      multi_purpose_mpc_ros::mpcc_rate_resolved::kProgressIndex],
    -0.1, 1.0e-12);
  EXPECT_DOUBLE_EQ(
    resolution.warm_start->primal[state_values],
    previous_primal[state_values]);
}

TEST(MpccRateResolvedShadow, RecedingWarmStartShiftsOneCompleteStage)
{
  const auto previous_snapshot = snapshot(1U);
  auto current = snapshot(2U);
  current.identity.source_context.intent_generation =
    previous_snapshot.identity.source_context.intent_generation;
  current.identity.source_context.stage_geometry_id = 999U;
  current.identity.source_context =
    contract::seal_problem_context(current.identity.source_context);
  current.control_prediction_origin_sec =
    previous_snapshot.control_prediction_origin_sec + 0.11;
  const auto adapted = adapter::build(
    current.request,
    multi_purpose_mpc_ros::persistent_osqp::PhysicalConstraintTolerance{
      1.0e-4, 1.0e-4});
  ASSERT_TRUE(adapted.has_value());
  const int horizon = current.request.horizon_steps;
  const int state_values =
    multi_purpose_mpc_ros::mpcc_rate_resolved::kStateDimension *
    (horizon + 1);
  const int variable_count = state_values +
    multi_purpose_mpc_ros::mpcc_rate_resolved::kInputDimension * horizon;
  Eigen::VectorXd previous_primal(variable_count);
  for (int index = 0; index < variable_count; ++index) {
    previous_primal[index] = static_cast<double>(index + 1);
  }
  shadow::RecedingWarmStartSeed previous{
    previous_snapshot.identity,
    previous_snapshot.control_prediction_origin_sec,
    previous_snapshot.course_progress_origin_m,
    {0.10, 0.10, 0.10},
    previous_primal};

  const auto resolution = shadow::resolve_receding_warm_start(
    previous, current, adapted->problem, 80U);

  ASSERT_EQ(resolution.reason, shadow::RecedingWarmStartReason::Available);
  ASSERT_TRUE(resolution.warm_start.has_value());
  EXPECT_EQ(resolution.stage_advance, 1U);
  EXPECT_DOUBLE_EQ(
    resolution.warm_start->primal[state_values],
    previous_primal[
      state_values + multi_purpose_mpc_ros::mpcc_rate_resolved::kInputDimension]);
  EXPECT_DOUBLE_EQ(
    resolution.warm_start->primal[
      2 * multi_purpose_mpc_ros::mpcc_rate_resolved::kStateDimension +
      multi_purpose_mpc_ros::mpcc_rate_resolved::kLateralIndex],
    previous_primal[
      3 * multi_purpose_mpc_ros::mpcc_rate_resolved::kStateDimension +
      multi_purpose_mpc_ros::mpcc_rate_resolved::kLateralIndex]);
}

TEST(MpccRateResolvedShadow, RecedingWarmStartResizesAcrossPlanningHorizons)
{
  auto previous_snapshot = snapshot(1U);
  previous_snapshot.identity.source_context.intent = contract::ControlIntent::Pass;
  previous_snapshot.identity.source_context.target_id = "d2";
  previous_snapshot.identity.source_context.target_obstacle_generation = 5U;
  previous_snapshot.identity.source_context.execution_side_sign = -1;
  previous_snapshot.identity.source_context =
    contract::seal_problem_context(previous_snapshot.identity.source_context);

  auto current = snapshot(2U);
  current.request = straight_request(5);
  current.execution_prefix_steps = 2;
  current.nominal_path_distance_m = {0.0, 0.2, 0.4, 0.6, 0.8, 1.0};
  current.identity.source_context = previous_snapshot.identity.source_context;
  current.identity.source_context.decision_id += 1U;
  current.identity.source_context.stage_geometry_id += 1U;
  current.identity.source_context.horizon_steps = 5U;
  current.identity.source_context =
    contract::seal_problem_context(current.identity.source_context);
  current.control_prediction_origin_sec =
    previous_snapshot.control_prediction_origin_sec + 0.025;

  const auto adapted = adapter::build(
    current.request,
    multi_purpose_mpc_ros::persistent_osqp::PhysicalConstraintTolerance{
      1.0e-4, 1.0e-4});
  ASSERT_TRUE(adapted.has_value());
  constexpr int previous_horizon = 3;
  const int previous_state_values =
    multi_purpose_mpc_ros::mpcc_rate_resolved::kStateDimension *
    (previous_horizon + 1);
  const int previous_variable_count = previous_state_values +
    multi_purpose_mpc_ros::mpcc_rate_resolved::kInputDimension *
    previous_horizon;
  Eigen::VectorXd previous_primal(previous_variable_count);
  for (int index = 0; index < previous_variable_count; ++index) {
    previous_primal[index] = static_cast<double>(index + 1);
  }
  shadow::RecedingWarmStartSeed previous{
    previous_snapshot.identity,
    previous_snapshot.control_prediction_origin_sec,
    previous_snapshot.course_progress_origin_m,
    {0.10, 0.10, 0.10},
    previous_primal};

  const auto resolution = shadow::resolve_receding_warm_start(
    previous, current, adapted->problem, 140U);

  ASSERT_EQ(resolution.reason, shadow::RecedingWarmStartReason::Available);
  ASSERT_TRUE(resolution.warm_start.has_value());
  const int current_state_values =
    multi_purpose_mpc_ros::mpcc_rate_resolved::kStateDimension * 6;
  const int current_variable_count = current_state_values +
    multi_purpose_mpc_ros::mpcc_rate_resolved::kInputDimension * 5;
  EXPECT_EQ(resolution.warm_start->primal.size(), current_variable_count);
  EXPECT_TRUE(
    resolution.warm_start->primal.head<
      multi_purpose_mpc_ros::mpcc_rate_resolved::kStateDimension>().isApprox(
      adapted->problem.initial_state));
  EXPECT_DOUBLE_EQ(
    resolution.warm_start->primal[current_state_values],
    previous_primal[previous_state_values]);
  EXPECT_DOUBLE_EQ(
    resolution.warm_start->primal[
      current_state_values +
      4 * multi_purpose_mpc_ros::mpcc_rate_resolved::kInputDimension],
    previous_primal[
      previous_state_values +
      2 * multi_purpose_mpc_ros::mpcc_rate_resolved::kInputDimension]);
}

TEST(MpccRateResolvedShadow, RecedingWarmStartRejectsTacticalIdentityChanges)
{
  auto previous_snapshot = snapshot(1U);
  previous_snapshot.identity.source_context.intent = contract::ControlIntent::Pass;
  previous_snapshot.identity.source_context.target_id = "d2";
  previous_snapshot.identity.source_context.target_obstacle_generation = 5U;
  previous_snapshot.identity.source_context.execution_side_sign = 1;
  previous_snapshot.identity.source_context =
    contract::seal_problem_context(previous_snapshot.identity.source_context);
  auto current = snapshot(2U);
  current.identity.source_context = previous_snapshot.identity.source_context;
  current.identity.source_context.decision_id += 1U;
  current.identity.source_context.target_id = "d3";
  current.identity.source_context =
    contract::seal_problem_context(current.identity.source_context);
  const auto adapted = adapter::build(
    current.request,
    multi_purpose_mpc_ros::persistent_osqp::PhysicalConstraintTolerance{
      1.0e-4, 1.0e-4});
  ASSERT_TRUE(adapted.has_value());
  const int horizon = current.request.horizon_steps;
  const int variable_count =
    multi_purpose_mpc_ros::mpcc_rate_resolved::kStateDimension * (horizon + 1) +
    multi_purpose_mpc_ros::mpcc_rate_resolved::kInputDimension * horizon;
  shadow::RecedingWarmStartSeed previous{
    previous_snapshot.identity,
    previous_snapshot.control_prediction_origin_sec,
    previous_snapshot.course_progress_origin_m,
    {0.10, 0.10, 0.10},
    Eigen::VectorXd::Zero(variable_count)};

  const auto target_changed = shadow::resolve_receding_warm_start(
    previous, current, adapted->problem, 80U);
  EXPECT_EQ(
    target_changed.reason,
    shadow::RecedingWarmStartReason::SemanticMismatch);
  EXPECT_EQ(target_changed.diagnostic, "target-id");
  EXPECT_FALSE(target_changed.warm_start.has_value());

  current.identity.source_context.target_id = "d2";
  current.identity.source_context.execution_side_sign = -1;
  current.identity.source_context =
    contract::seal_problem_context(current.identity.source_context);
  const auto side_changed = shadow::resolve_receding_warm_start(
    previous, current, adapted->problem, 80U);
  EXPECT_EQ(
    side_changed.reason,
    shadow::RecedingWarmStartReason::SemanticMismatch);
  EXPECT_EQ(side_changed.diagnostic, "execution-side");

  current.identity.source_context.execution_side_sign = 1;
  current.identity.source_context.intent_generation += 1U;
  current.identity.source_context =
    contract::seal_problem_context(current.identity.source_context);
  const auto generation_changed = shadow::resolve_receding_warm_start(
    previous, current, adapted->problem, 80U);
  EXPECT_EQ(
    generation_changed.reason,
    shadow::RecedingWarmStartReason::SemanticMismatch);
  EXPECT_EQ(generation_changed.diagnostic, "intent-generation");
}

TEST(MpccRateResolvedShadow, RecedingWarmStartRejectsExhaustedHorizon)
{
  const auto previous_snapshot = snapshot(1U);
  auto current = snapshot(2U);
  current.identity.source_context.intent_generation =
    previous_snapshot.identity.source_context.intent_generation;
  current.identity.source_context =
    contract::seal_problem_context(current.identity.source_context);
  current.control_prediction_origin_sec =
    previous_snapshot.control_prediction_origin_sec + 0.31;
  const auto adapted = adapter::build(
    current.request,
    multi_purpose_mpc_ros::persistent_osqp::PhysicalConstraintTolerance{
      1.0e-4, 1.0e-4});
  ASSERT_TRUE(adapted.has_value());
  const int horizon = current.request.horizon_steps;
  const int variable_count =
    multi_purpose_mpc_ros::mpcc_rate_resolved::kStateDimension * (horizon + 1) +
    multi_purpose_mpc_ros::mpcc_rate_resolved::kInputDimension * horizon;
  shadow::RecedingWarmStartSeed previous{
    previous_snapshot.identity,
    previous_snapshot.control_prediction_origin_sec,
    previous_snapshot.course_progress_origin_m,
    {0.10, 0.10, 0.10},
    Eigen::VectorXd::Zero(variable_count)};

  const auto resolution = shadow::resolve_receding_warm_start(
    previous, current, adapted->problem, 80U);
  EXPECT_EQ(
    resolution.reason,
    shadow::RecedingWarmStartReason::HorizonExhausted);
  EXPECT_FALSE(resolution.warm_start.has_value());
}

TEST(MpccRateResolvedShadow, TradesProgressForReachableOpponentSeparation)
{
  auto input = snapshot();
  bind_dynamic_obstacle_identity(input);
  input.dynamic_obstacle_refinement_active = true;
  input.dynamic_obstacle_pass_side_sign = 1;
  input.dynamic_obstacle_stages.assign(
    3U,
    multi_purpose_mpc_ros::mpcc_rate_resolved_dynamic_obstacle::StagePrediction{
      true, 0.8, 0.75, 0.20, 0.30});

  shadow::SolverContext solver;
  const auto result = solver.evaluate(input);
  EXPECT_EQ(result.outcome, shadow::Outcome::Solved) << result.detail;
  EXPECT_TRUE(result.dynamic_obstacle_refinement_requested);
  EXPECT_TRUE(result.dynamic_obstacle_refinement_applied);
  EXPECT_TRUE(result.dynamic_obstacle_refinement_solved);
  EXPECT_EQ(result.wall_refinement_solver_solve_count, 0U);
  EXPECT_EQ(result.wall_refinement_solver_scaling_iterations, 0);
  EXPECT_TRUE(result.post_refinement_physical_proof_checked);
  EXPECT_TRUE(result.post_refinement_physical_proof_accepted);
  EXPECT_EQ(
    result.post_refinement_linearization_requested,
    result.post_refinement_linearization_count > 0U);
  if (result.post_refinement_linearization_requested) {
    EXPECT_TRUE(result.post_refinement_linearization_applied);
    EXPECT_TRUE(result.post_refinement_linearization_bootstrap_applied);
    EXPECT_TRUE(result.post_refinement_linearization_solved);
  }
  EXPECT_GT(result.dynamic_obstacle_stay_behind_row_count, 0U);
  EXPECT_TRUE(shadow::result_valid(result));
}

TEST(
  MpccRateResolvedShadow,
  CruiseStayBehindUsesDedicatedDynamicObstacleIdentity)
{
  auto input = snapshot();
  auto problem_context = input.identity.source_context;
  problem_context.intent = contract::ControlIntent::Cruise;
  input.identity.source_context =
    contract::seal_problem_context(std::move(problem_context));
  bind_dynamic_obstacle_identity(input, "d2", 7U, 0);
  ASSERT_TRUE(input.identity.source_context.target_id.empty());
  input.dynamic_obstacle_refinement_active = true;
  input.dynamic_obstacle_pass_side_sign = 0;
  input.dynamic_obstacle_stages.assign(
    3U,
    multi_purpose_mpc_ros::mpcc_rate_resolved_dynamic_obstacle::StagePrediction{
      true, 0.8, 0.0, 0.20, 0.30});

  shadow::SolverContext solver;
  const auto result = solver.evaluate(input);

  EXPECT_EQ(result.outcome, shadow::Outcome::Solved) << result.detail;
  EXPECT_TRUE(result.dynamic_obstacle_refinement_requested);
  EXPECT_TRUE(result.dynamic_obstacle_refinement_applied);
  EXPECT_GT(result.dynamic_obstacle_stay_behind_row_count, 0U);
  EXPECT_TRUE(shadow::result_valid(result));
}

TEST(MpccRateResolvedShadow, CouplesDynamicProgressChoiceBeforeFinalWallProof)
{
  auto input = snapshot();
  bind_dynamic_obstacle_identity(input);
  input.progress_aligned_wall_refinement_active = true;
  input.wall_reference_progress_m = {0.0, 0.3, 0.6, 1.0};
  input.wall_lower_m = {-1.0, -0.9, -0.8, -0.7};
  input.wall_upper_m = {1.0, 0.9, 0.8, 0.7};
  input.dynamic_obstacle_refinement_active = true;
  input.dynamic_obstacle_pass_side_sign = 1;
  input.dynamic_obstacle_stages.assign(
    3U,
    multi_purpose_mpc_ros::mpcc_rate_resolved_dynamic_obstacle::StagePrediction{
      true, 0.8, 0.75, 0.20, 0.30});

  shadow::SolverContext solver;
  const auto result = solver.evaluate(input);

  EXPECT_EQ(result.outcome, shadow::Outcome::Solved) << result.detail;
  EXPECT_TRUE(result.progress_wall_refinement_requested);
  EXPECT_TRUE(result.progress_wall_refinement_applied);
  EXPECT_TRUE(result.progress_wall_refinement_solved);
  EXPECT_TRUE(result.dynamic_obstacle_refinement_requested);
  EXPECT_TRUE(result.dynamic_obstacle_refinement_applied);
  EXPECT_TRUE(result.dynamic_obstacle_refinement_solved);
  EXPECT_GT(result.wall_refinement_solver_solve_count, 0U);
  EXPECT_EQ(result.wall_refinement_solver_scaling_iterations, 10);
  EXPECT_TRUE(result.post_refinement_physical_proof_checked);
  EXPECT_TRUE(result.post_refinement_physical_proof_accepted);
  EXPECT_TRUE(shadow::result_valid(result));
}

TEST(
  MpccRateResolvedShadow,
  SolvesCompletePlanningHorizonButPublishesOnlyCertifiedPrefix)
{
  auto input = snapshot();
  input.execution_prefix_steps = 2;

  shadow::SolverContext context;
  const auto result = context.evaluate(input);

  ASSERT_EQ(result.outcome, shadow::Outcome::Solved) << result.detail;
  ASSERT_NE(result.execution_artifact, nullptr);
  EXPECT_EQ(result.planning_stage_count, 3U);
  EXPECT_EQ(result.certified_stage_count, 2U);
  EXPECT_EQ(result.execution_artifact->control_stages.size(), 2U);
  EXPECT_EQ(result.execution_artifact->predicted_states.size(), 3U);
  EXPECT_EQ(result.execution_artifact->nominal_path_distance_m.size(), 3U);
  EXPECT_EQ(result.execution_artifact->lateral_lower_m.size(), 3U);
  EXPECT_EQ(result.execution_artifact->lateral_upper_m.size(), 3U);
  EXPECT_NEAR(result.certified_horizon_duration_sec, 0.20, 1e-12);
  EXPECT_TRUE(shadow::result_valid(result));
}

TEST(MpccRateResolvedShadow, RefinesWallRowsAtTheSolvedProgress)
{
  shadow::SolverContext context;
  auto input = snapshot();
  input.progress_aligned_wall_refinement_active = true;
  input.wall_reference_progress_m = {0.0, 0.3, 0.6, 1.0};
  input.wall_lower_m = {-1.0, -0.8, -0.6, -0.4};
  input.wall_upper_m = {1.0, 0.9, 0.8, 0.7};

  const auto result = context.evaluate(input);
  ASSERT_EQ(result.outcome, shadow::Outcome::Solved) << result.detail;
  EXPECT_TRUE(result.progress_wall_refinement_requested);
  EXPECT_TRUE(result.progress_wall_refinement_applied);
  EXPECT_TRUE(result.progress_wall_refinement_solved);
  EXPECT_EQ(
    result.progress_wall_refinement_reason,
    multi_purpose_mpc_ros::mpcc_progress::
    ProgressAlignedWallBoundsReason::Accepted);
  ASSERT_NE(result.execution_artifact, nullptr);
  EXPECT_GT(result.progress_wall_refinement_aligned_stage_count, 0U);
}

TEST(MpccRateResolvedShadow, RejectsWhenRequestedWallRefinementCannotBeBuilt)
{
  shadow::SolverContext context;
  auto input = snapshot();
  input.progress_aligned_wall_refinement_active = true;
  // The first QP is deliberately allowed to solve without wall rows. A
  // malformed physical-wall profile must not let that provisional solution
  // escape as a production artifact.
  input.wall_reference_progress_m = {0.0};
  input.wall_lower_m = {-1.0};
  input.wall_upper_m = {1.0};

  const auto result = context.evaluate(input);
  EXPECT_EQ(result.outcome, shadow::Outcome::AssemblyRejected);
  EXPECT_FALSE(result.solved);
  EXPECT_TRUE(result.progress_wall_refinement_requested);
  EXPECT_FALSE(result.progress_wall_refinement_applied);
  EXPECT_EQ(
    result.progress_wall_refinement_reason,
    multi_purpose_mpc_ros::mpcc_progress::
    ProgressAlignedWallBoundsReason::InvalidInput);
  EXPECT_EQ(result.execution_artifact, nullptr);
  EXPECT_NE(result.detail.find("wall refinement rejected"), std::string::npos);
}

TEST(MpccRateResolvedShadow, PhysicalWallRefinementRetainsHeadingInCertificate)
{
  const auto grid = corridor_grid();
  auto aligned_request = wall_request(grid, 0.0);
  auto rotated_request = wall_request(grid, 0.5);
  aligned_request.stages.front().lag_lower_m =
    -std::numeric_limits<double>::infinity();
  aligned_request.stages.front().lag_upper_m =
    std::numeric_limits<double>::infinity();
  aligned_request.stages.front().heading_lower_rad =
    -std::numeric_limits<double>::infinity();
  aligned_request.stages.front().heading_upper_rad =
    std::numeric_limits<double>::infinity();
  rotated_request.stages.front().lag_lower_m =
    -std::numeric_limits<double>::infinity();
  rotated_request.stages.front().lag_upper_m =
    std::numeric_limits<double>::infinity();
  rotated_request.stages.front().heading_lower_rad =
    -std::numeric_limits<double>::infinity();
  rotated_request.stages.front().heading_upper_rad =
    std::numeric_limits<double>::infinity();
  const auto aligned = wall_refinement::resolve(aligned_request);
  const auto rotated = wall_refinement::resolve(rotated_request);

  ASSERT_TRUE(aligned.applied) << aligned.detail;
  ASSERT_TRUE(rotated.applied) << rotated.detail;
  ASSERT_EQ(aligned.stages.size(), 1U);
  ASSERT_EQ(rotated.stages.size(), 1U);
  EXPECT_EQ(aligned.swept_lateral_constraints.size(), 4U);
  EXPECT_EQ(rotated.swept_lateral_constraints.size(), 4U);
  EXPECT_LT(
    rotated.stages.front().lateral_upper_m,
    aligned.stages.front().lateral_upper_m - 0.2);
  EXPECT_LE(
    rotated.stages.front().heading_lower_rad, 0.5 + 1e-12);
  EXPECT_GE(
    rotated.stages.front().heading_upper_rad, 0.5 - 1e-12);
  EXPECT_LT(
    rotated.stages.front().heading_upper_rad -
    rotated.stages.front().heading_lower_rad,
    0.026);
  EXPECT_LT(
    rotated.stages.front().lag_upper_m -
    rotated.stages.front().lag_lower_m,
    0.051);
  EXPECT_LT(
    rotated.stages.front().progress_upper_m -
    rotated.stages.front().progress_lower_m,
    0.051);
}

TEST(MpccRateResolvedWallRefinement, ReusesCoveringStaticWallScanEvidence)
{
  const auto grid = corridor_grid();
  const auto request = wall_request(grid, 0.25);
  wall_refinement::Cache cache;

  const auto cold = wall_refinement::resolve(request, &cache);
  const auto warm = wall_refinement::resolve(request, &cache);

  ASSERT_TRUE(cold.applied) << cold.detail;
  ASSERT_TRUE(warm.applied) << warm.detail;
  EXPECT_EQ(cold.cache_hit_count, 0U);
  EXPECT_EQ(cold.cache_miss_count, 1U);
  EXPECT_GT(cold.cache_scanned_pose_count, 0U);
  EXPECT_EQ(warm.cache_hit_count, 1U);
  EXPECT_EQ(warm.cache_miss_count, 0U);
  EXPECT_EQ(warm.cache_scanned_pose_count, 0U);
  EXPECT_EQ(warm.checked_pose_count, cold.checked_pose_count);
  ASSERT_EQ(warm.stages.size(), cold.stages.size());
  EXPECT_DOUBLE_EQ(
    warm.stages.front().lateral_lower_m,
    cold.stages.front().lateral_lower_m);
  EXPECT_DOUBLE_EQ(
    warm.stages.front().lateral_upper_m,
    cold.stages.front().lateral_upper_m);
  ASSERT_EQ(
    warm.swept_lateral_constraints.size(),
    cold.swept_lateral_constraints.size());
  for (std::size_t index = 0U;
    index < warm.swept_lateral_constraints.size(); ++index)
  {
    EXPECT_DOUBLE_EQ(
      warm.swept_lateral_constraints[index].lateral_lower_m,
      cold.swept_lateral_constraints[index].lateral_lower_m);
    EXPECT_DOUBLE_EQ(
      warm.swept_lateral_constraints[index].lateral_upper_m,
      cold.swept_lateral_constraints[index].lateral_upper_m);
  }

  auto subset = request;
  subset.stages.front().lateral_lower_m += 0.05;
  subset.stages.front().lateral_upper_m -= 0.05;
  const auto subset_hit = wall_refinement::resolve(subset, &cache);
  ASSERT_TRUE(subset_hit.applied) << subset_hit.detail;
  EXPECT_EQ(subset_hit.cache_hit_count, 1U);
  EXPECT_EQ(subset_hit.cache_miss_count, 0U);
  EXPECT_EQ(subset_hit.cache_scanned_pose_count, 0U);

  auto expansion = request;
  expansion.stages.front().lateral_lower_m -= 0.05;
  expansion.stages.front().lateral_upper_m += 0.05;
  const auto expanded = wall_refinement::resolve(expansion, &cache);
  ASSERT_TRUE(expanded.applied) << expanded.detail;
  EXPECT_EQ(expanded.cache_hit_count, 0U);
  EXPECT_EQ(expanded.cache_miss_count, 1U);
  EXPECT_GT(expanded.cache_scanned_pose_count, 0U);

  const auto covered_again = wall_refinement::resolve(request, &cache);
  ASSERT_TRUE(covered_again.applied) << covered_again.detail;
  EXPECT_EQ(covered_again.cache_hit_count, 1U);
  EXPECT_EQ(covered_again.cache_miss_count, 0U);
  EXPECT_EQ(covered_again.cache_scanned_pose_count, 0U);
}

TEST(MpccRateResolvedWallRefinement, BoundsStaticWallScanEvidenceCache)
{
  const auto grid = corridor_grid();
  wall_refinement::Cache cache(1U);
  const auto first = wall_refinement::resolve(
    wall_request(grid, 0.0), &cache);
  const auto second = wall_refinement::resolve(
    wall_request(grid, 0.25), &cache);

  ASSERT_TRUE(first.applied) << first.detail;
  ASSERT_TRUE(second.applied) << second.detail;
  EXPECT_EQ(cache.maximum_entries(), 1U);
  EXPECT_EQ(cache.size(), 1U);
  EXPECT_EQ(second.cache_hit_count, 0U);
  EXPECT_EQ(second.cache_miss_count, 1U);
}

TEST(MpccRateResolvedShadow, RejectsNoncanonicalProblemIdentity)
{
  shadow::SolverContext context;
  auto input = snapshot();
  input.identity.source_context.formulation =
    contract::Formulation::SolverDerivedBypass;
  const auto result = context.evaluate(input);
  EXPECT_EQ(result.outcome, shadow::Outcome::BuildRejected);
  EXPECT_FALSE(shadow::result_valid(result));
}

TEST(MpccRateResolvedShadow, SupportsEveryRateResolvedNormalIntent)
{
  for (const auto intent : {
      contract::ControlIntent::Track,
      contract::ControlIntent::Cruise,
      contract::ControlIntent::ShiftOut,
      contract::ControlIntent::Pass,
      contract::ControlIntent::Return})
  {
    shadow::SolverContext context;
    auto input = snapshot(static_cast<std::uint64_t>(intent) + 10U);
    auto source = input.identity.source_context;
    source.intent = intent;
    const bool overtake_intent =
      contract::canonical_normal_intent_requires_execution_side(intent);
    source.target_id = overtake_intent ? "d2" : "";
    source.target_obstacle_generation =
      overtake_intent ? source.observation_generation : 0U;
    source.execution_side_sign = overtake_intent ? 1 : 0;
    input.identity.source_context =
      contract::seal_problem_context(std::move(source));

    const auto result = context.evaluate(input);
    EXPECT_EQ(result.outcome, shadow::Outcome::Solved)
      << contract::to_string(intent) << ": " << result.detail;
    ASSERT_NE(result.execution_artifact, nullptr)
      << contract::to_string(intent);
    EXPECT_TRUE(execution::identity_valid(result.execution_artifact->identity))
      << contract::to_string(intent);
  }
}

TEST(MpccRateResolvedShadow, RetainsAndSamplesExactRateResolvedArtifact)
{
  shadow::SolverContext context;
  const auto result = context.evaluate(snapshot());
  ASSERT_EQ(result.outcome, shadow::Outcome::Solved) << result.detail;
  ASSERT_NE(result.execution_artifact, nullptr);
  const auto & artifact = *result.execution_artifact;

  const auto cursor = execution::resolve_cursor(
    artifact, artifact.prediction_origin_sec + 0.15);
  ASSERT_TRUE(cursor.available);
  EXPECT_EQ(cursor.reason, execution::CursorReason::Available);
  EXPECT_EQ(cursor.control_stage_index, 1U);
  EXPECT_EQ(cursor.remaining_control_stage_count, 2U);
  EXPECT_NEAR(cursor.stage_elapsed_sec, 0.05, 1e-12);

  const auto actuation = execution::extract_actuation(
    artifact, cursor);
  ASSERT_TRUE(actuation.actuation.has_value());
  EXPECT_EQ(actuation.reason, execution::ActuationReason::Available);
  EXPECT_EQ(
    actuation.sample_reason,
    multi_purpose_mpc_ros::mpcc_rate_resolved::ActuationSampleReason::Accepted);
  const double expected_steering =
    artifact.semantic_initial_steering_rad +
    artifact.control_stages[0].steering_rate_radps *
    artifact.control_stages[0].duration_sec +
    artifact.control_stages[1].steering_rate_radps *
    0.05;
  EXPECT_NEAR(
    actuation.actuation->steering_rad, expected_steering, 1e-9);
  EXPECT_DOUBLE_EQ(
    actuation.actuation->steering_rate_radps,
    artifact.control_stages[1].steering_rate_radps);

  const auto exhausted = execution::resolve_cursor(
    artifact, artifact.prediction_origin_sec + 0.30);
  EXPECT_FALSE(exhausted.available);
  EXPECT_EQ(exhausted.reason, execution::CursorReason::Exhausted);
}

TEST(
  MpccRateResolvedShadow,
  FreshArtifactPublishesFromTheCertifiedPhysicalState)
{
  shadow::SolverContext context;
  const auto result = context.evaluate(snapshot());
  ASSERT_EQ(result.outcome, shadow::Outcome::Solved) << result.detail;
  ASSERT_NE(result.execution_artifact, nullptr);

  const auto & artifact = *result.execution_artifact;
  const auto cursor = execution::resolve_cursor(
    artifact, artifact.prediction_origin_sec);
  ASSERT_TRUE(cursor.available);
  ASSERT_DOUBLE_EQ(cursor.elapsed_sec, 0.0);

  const auto actuation = execution::extract_actuation(
    artifact, cursor);
  ASSERT_TRUE(actuation.actuation.has_value());
  EXPECT_NEAR(
    actuation.actuation->steering_rad,
    artifact.semantic_initial_steering_rad,
    1e-9);
}

TEST(MpccRateResolvedShadow, ArtifactRejectsInvalidPublicationTiming)
{
  shadow::SolverContext context;
  const auto result = context.evaluate(snapshot());
  ASSERT_EQ(result.outcome, shadow::Outcome::Solved) << result.detail;
  ASSERT_NE(result.execution_artifact, nullptr);

  auto missing_interval = *result.execution_artifact;
  missing_interval.publication_interval_sec = 0.0;
  EXPECT_EQ(
    execution::validate(missing_interval),
    execution::RejectReason::InvalidTiming);

  auto beyond_horizon = *result.execution_artifact;
  beyond_horizon.publication_interval_sec = 1.0;
  EXPECT_EQ(
    execution::validate(beyond_horizon),
    execution::RejectReason::InvalidTiming);
}

TEST(MpccRateResolvedShadow, RejectsMutatedExecutionArtifactProvenance)
{
  shadow::SolverContext context;
  const auto result = context.evaluate(snapshot());
  ASSERT_EQ(result.outcome, shadow::Outcome::Solved) << result.detail;
  ASSERT_NE(result.execution_artifact, nullptr);

  auto invalid = *result.execution_artifact;
  invalid.identity.source_context.fingerprint = 0U;
  EXPECT_EQ(
    execution::validate(invalid), execution::RejectReason::InvalidIdentity);

  invalid = *result.execution_artifact;
  invalid.course_progress_origin_m =
    std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(
    execution::validate(invalid),
    execution::RejectReason::InvalidCourseProgressOrigin);

  invalid = *result.execution_artifact;
  invalid.predicted_states.pop_back();
  EXPECT_EQ(
    execution::validate(invalid), execution::RejectReason::StateCountMismatch);

  invalid = *result.execution_artifact;
  invalid.nominal_path_distance_m.pop_back();
  EXPECT_EQ(
    execution::validate(invalid),
    execution::RejectReason::PathDistanceCountMismatch);

  invalid = *result.execution_artifact;
  invalid.nominal_path_distance_m[1] = 0.0;
  EXPECT_EQ(
    execution::validate(invalid), execution::RejectReason::InvalidPathDistance);

  invalid = *result.execution_artifact;
  invalid.predicted_states.front().steering_rad += 0.1;
  EXPECT_EQ(
    execution::validate(invalid), execution::RejectReason::InitialSteeringMismatch);

  invalid = *result.execution_artifact;
  invalid.predicted_states[1].steering_rad += 0.1;
  EXPECT_EQ(
    execution::validate(invalid), execution::RejectReason::SteeringDynamicsMismatch);

  invalid = *result.execution_artifact;
  invalid.lateral_upper_m[1] = invalid.predicted_states[1].lateral_m - 0.1;
  EXPECT_EQ(
    execution::validate(invalid), execution::RejectReason::InvalidLateralCorridor);

  invalid = *result.execution_artifact;
  invalid.maximum_normalized_constraint_violation = 1.01;
  EXPECT_EQ(
    execution::validate(invalid), execution::RejectReason::InvalidCertificate);
}

TEST(MpccRateResolvedShadow, SamplesPublicationPeriodAcrossStageBoundary)
{
  shadow::SolverContext context;
  auto input = snapshot();
  input.publication_interval_sec = 0.20;
  const auto result = context.evaluate(input);
  ASSERT_EQ(result.outcome, shadow::Outcome::Solved) << result.detail;
  EXPECT_TRUE(shadow::result_valid(result));
  EXPECT_EQ(result.sampled_stage_index, 1U);
  EXPECT_NEAR(result.sampled_stage_elapsed_sec, 0.10, 1e-12);
  EXPECT_NEAR(result.certified_horizon_duration_sec, 0.30, 1e-12);
}

TEST(MpccRateResolvedShadow, RejectsPublicationPeriodBeyondCertifiedHorizon)
{
  shadow::SolverContext context;
  auto input = snapshot();
  input.publication_interval_sec = 0.31;
  const auto result = context.evaluate(input);
  EXPECT_EQ(result.outcome, shadow::Outcome::ActuationSampleRejected);
  EXPECT_EQ(
    result.actuation_sample_reason,
    multi_purpose_mpc_ros::mpcc_rate_resolved::ActuationSampleReason::
    PublicationAfterHorizonEnd);
  EXPECT_DOUBLE_EQ(result.publication_interval_sec, 0.31);
  EXPECT_DOUBLE_EQ(result.first_stage_duration_sec, 0.10);
  EXPECT_NEAR(result.certified_horizon_duration_sec, 0.30, 1e-12);
  EXPECT_TRUE(shadow::result_valid(result));
}

TEST(MpccRateResolvedShadowMailbox, PublishesMonotonicRegisteredResults)
{
  shadow::SolverContext context;
  shadow::Mailbox mailbox;
  ASSERT_TRUE(mailbox.register_submission(1U));
  const auto first = context.evaluate(snapshot(1U));
  EXPECT_EQ(mailbox.publish(first), shadow::PublishReason::Accepted);
  const auto available = mailbox.latest_after(0U);
  ASSERT_TRUE(available.has_value());
  EXPECT_EQ(available->identity.sequence, 1U);
  EXPECT_FALSE(mailbox.latest_after(1U).has_value());

  ASSERT_TRUE(mailbox.register_submission(2U));
  const auto second = context.evaluate(snapshot(2U));
  EXPECT_EQ(mailbox.publish(second), shadow::PublishReason::Accepted);
  EXPECT_EQ(
    mailbox.publish(first), shadow::PublishReason::SequenceRollback);
  const auto state = mailbox.state();
  EXPECT_EQ(state.accepted_count, 2U);
  EXPECT_EQ(state.sequence_rollback_count, 1U);
}

TEST(MpccRateResolvedShadowMailbox, RejectsUnregisteredAndInvalidResults)
{
  shadow::SolverContext context;
  shadow::Mailbox mailbox;
  const auto result = context.evaluate(snapshot(2U));
  EXPECT_EQ(
    mailbox.publish(result), shadow::PublishReason::SequenceNotSubmitted);

  ASSERT_TRUE(mailbox.register_submission(3U));
  auto invalid = context.evaluate(snapshot(3U));
  invalid.identity.source_context.fingerprint = 0U;
  EXPECT_EQ(mailbox.publish(invalid), shadow::PublishReason::InvalidResult);
}
