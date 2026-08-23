#include "multi_purpose_mpc_ros/canonical_retained_world_revalidation.hpp"

#include <gtest/gtest.h>

#include <utility>

namespace
{

namespace contract = multi_purpose_mpc_ros::mpcc_execution_contract;
namespace plan = multi_purpose_mpc_ros::canonical_execution_plan;
namespace retained = multi_purpose_mpc_ros::canonical_retained_revalidation;
namespace world =
  multi_purpose_mpc_ros::canonical_retained_world_revalidation;
namespace footprint = multi_purpose_mpc_ros::recovery_footprint;
namespace geometry = multi_purpose_mpc_ros::mpc_stage_geometry;

plan::CanonicalExecutionPlan make_plan()
{
  contract::MpccProblemContext problem;
  problem.decision_id = 42U;
  problem.intent = contract::ControlIntent::Track;
  problem.intent_generation = 3U;
  problem.observation_generation = 7U;
  problem.stage_geometry_id = 11U;
  problem.horizon_steps = 2U;
  problem.formulation = contract::Formulation::VelocityProgress5State;
  problem.state_schema_id = "ey-elag-epsi-v-progress-v1";
  problem.input_schema_id = "accel-curvature-progress-rate-v1";
  problem.bounds_schema_id = "progress-stage-wall-obstacle-v1";
  problem.cost_schema_id = "velocity-progress-v1";
  problem = contract::seal_problem_context(std::move(problem));

  contract::CertifiedMpccSolution solution;
  solution.solution_id = 9U;
  solution.problem_fingerprint = problem.fingerprint;
  solution.formulation = contract::Formulation::VelocityProgress5State;
  solution.solved = true;
  solution.finite = true;
  solution.constraints_satisfied = true;
  solution.maximum_constraint_violation = 0.0;
  solution.physical.checked = true;
  solution.physical.wall_clear = true;
  solution.physical.obstacles_clear = true;
  solution.prediction_stage_count = 2U;
  solution.valid_until_sec = 12.5;

  plan::CanonicalExecutionPlan value;
  value.plan_id = 23U;
  value.problem = problem;
  value.solution = solution;
  value.solved_sec = 10.0;
  value.predicted_states = {
    plan::CanonicalPredictedState{0.10, 0.01, 0.02, 5.0, 100.0},
    plan::CanonicalPredictedState{0.12, 0.02, 0.03, 5.2, 100.5},
    plan::CanonicalPredictedState{0.14, 0.03, 0.04, 5.4, 101.0}};
  value.control_stages = {
    plan::CanonicalControlStage{1.0, 0.02, 5.1, 1.0},
    plan::CanonicalControlStage{0.5, 0.03, 5.3, 1.0}};
  return value;
}

footprint::OccupancyGrid make_grid(const footprint::CellState state)
{
  footprint::OccupancyGrid grid;
  grid.width = 120U;
  grid.height = 120U;
  grid.resolution_m = 0.1;
  grid.origin_x_m = 0.0;
  grid.origin_y_m = 0.0;
  grid.cells.assign(grid.width * grid.height, state);
  return grid;
}

world::CurrentWorldProofRequest make_request()
{
  world::CurrentWorldProofRequest request;
  request.current.decision_id = 43U;
  request.current.intent = contract::ControlIntent::Track;
  request.current.intent_generation = 3U;
  request.current.observation_generation = 43U;
  request.current.stage_geometry_id = 12U;
  request.current.observation_sec = 10.6;
  request.current.path_length_m = 100.0;
  request.current.circular = true;
  request.measured_course_progress_m = 0.30;
  request.progress_continuity_tolerance_m = 0.20;
  request.measured_to_control_path = {
    footprint::Pose2D{2.30, 5.10, 0.02},
    footprint::Pose2D{2.31, 5.11, 0.02}};
  request.control_pose = request.measured_to_control_path.back();
  request.course_frame_knots = {
    geometry::CourseFrameKnot{100.0, 2.0, 5.0, 0.0, 0},
    geometry::CourseFrameKnot{100.5, 2.5, 5.0, 0.0, 1},
    geometry::CourseFrameKnot{101.0, 3.0, 5.0, 0.0, 2}};
  request.current.control_pose_id = world::fingerprint_control_pose_path(
    request.measured_to_control_path, request.control_pose);
  request.current.course_frame_window_id =
    world::fingerprint_course_frame_window(request.course_frame_knots);
  request.obstacles.observation_generation =
    request.current.observation_generation;
  request.obstacles.observation_sec = request.current.observation_sec;
  request.obstacles.current = true;
  request.obstacles.tube_id = world::fingerprint_empty_obstacle_observation(
    request.obstacles.observation_generation,
    request.obstacles.observation_sec);
  request.current.obstacle_tube_id = request.obstacles.tube_id;
  request.swept_step_m = 0.05;
  return request;
}

plan::CanonicalExecutionPlan make_follow_plan()
{
  auto value = make_plan();
  value.problem.intent = contract::ControlIntent::Follow;
  value.problem.target_id = "d2";
  value.problem.target_obstacle_generation = 5U;
  value.problem.bounds_schema_id = "progress-stage-wall-follow-target-v1";
  value.problem.cost_schema_id = "velocity-progress-follow-gap-v1";
  value.problem = contract::seal_problem_context(std::move(value.problem));
  value.solution.problem_fingerprint = value.problem.fingerprint;
  return value;
}

world::FollowCurrentWorldProofRequest make_follow_request()
{
  const auto empty = make_request();
  world::FollowCurrentWorldProofRequest request;
  request.current = empty.current;
  request.current.intent = contract::ControlIntent::Follow;
  request.current.target_id = "d2";
  request.current.target_obstacle_generation = 8U;
  request.measured_course_progress_m = empty.measured_course_progress_m;
  request.progress_continuity_tolerance_m =
    empty.progress_continuity_tolerance_m;
  request.measured_to_control_path = empty.measured_to_control_path;
  request.control_pose = empty.control_pose;
  request.course_frame_knots = empty.course_frame_knots;
  request.target.target_id = "d2";
  request.target.observation_generation = 8U;
  request.target.observation_sec = 10.6;
  request.target.hard_gap_m = 3.0;
  request.target.elapsed_time_sec = {0.0, 0.4, 1.4};
  request.target.target_relative_progress_m = {5.0, 5.2, 5.7};
  request.target.current = true;
  request.target.tube_id =
    world::fingerprint_follow_obstacle_observation(request.target);
  request.current.obstacle_tube_id = request.target.tube_id;
  request.swept_step_m = empty.swept_step_m;
  return request;
}

plan::CanonicalExecutionPlan make_overtake_plan()
{
  auto value = make_plan();
  value.problem.intent = contract::ControlIntent::ShiftOut;
  value.problem.execution_side_sign = 1;
  value.problem.target_id = "d2";
  value.problem.target_obstacle_generation = 5U;
  value.problem = contract::seal_problem_context(std::move(value.problem));
  value.solution.problem_fingerprint = value.problem.fingerprint;
  return value;
}

world::OvertakeCurrentWorldProofRequest make_overtake_request()
{
  const auto empty = make_request();
  world::OvertakeCurrentWorldProofRequest request;
  request.current = empty.current;
  request.current.intent = contract::ControlIntent::ShiftOut;
  request.current.execution_side_sign = 1;
  request.current.intent_generation = 3U;
  request.current.target_id = "d2";
  request.current.target_obstacle_generation = 8U;
  request.measured_course_progress_m = empty.measured_course_progress_m;
  request.measured_lateral_m = 0.11;
  request.progress_continuity_tolerance_m =
    empty.progress_continuity_tolerance_m;
  request.measured_to_control_path = empty.measured_to_control_path;
  request.control_pose = empty.control_pose;
  request.course_frame_knots = empty.course_frame_knots;
  request.corridor.target_id = "d2";
  request.corridor.observation_generation = 8U;
  request.corridor.observation_sec = 10.6;
  request.corridor.elapsed_time_sec = {0.0, 0.5, 1.5};
  request.corridor.lateral_lower_m = {-0.5, -0.4, -0.3};
  request.corridor.lateral_upper_m = {0.5, 0.45, 0.4};
  request.corridor.target_exclusion_encoded = true;
  request.corridor.current = true;
  request.corridor.tube_id =
    world::fingerprint_overtake_corridor_observation(request.corridor);
  request.current.obstacle_tube_id = request.corridor.tube_id;
  request.lateral_tolerance_m = 1e-6;
  request.swept_step_m = empty.swept_step_m;
  return request;
}

}  // namespace

TEST(CanonicalRetainedWorldRevalidation, BuildsProofFromCurrentEmptyWorld)
{
  const auto execution_plan = make_plan();
  const auto cursor = plan::resolve_execution_cursor(execution_plan, 10.6);
  const auto request = make_request();
  const footprint::FootprintExtents extents{0.15, 0.15, 0.10, 0.10, 0.0};
  const auto result = world::build_current_world_retained_proof(
    execution_plan, cursor, request, make_grid(footprint::CellState::Free),
    extents);

  ASSERT_EQ(result.reason, world::CurrentWorldProofReason::Accepted);
  ASSERT_TRUE(result.proof.has_value());
  EXPECT_EQ(result.proof->current.decision_id, 43U);
  EXPECT_EQ(result.proof->stage_evaluations.size(), 2U);
}

TEST(CanonicalRetainedWorldRevalidation, RejectsBlockedWallAndDynamicVehicle)
{
  const auto execution_plan = make_plan();
  const auto cursor = plan::resolve_execution_cursor(execution_plan, 10.6);
  const footprint::FootprintExtents extents{0.15, 0.15, 0.10, 0.10, 0.0};

  const auto blocked = world::build_current_world_retained_proof(
    execution_plan, cursor, make_request(),
    make_grid(footprint::CellState::Occupied), extents);
  EXPECT_EQ(blocked.reason, world::CurrentWorldProofReason::DelayPrefixBlocked);

  auto dynamic = make_request();
  dynamic.obstacles.active_vehicle_count = 1U;
  const auto vehicle = world::build_current_world_retained_proof(
    execution_plan, cursor, dynamic,
    make_grid(footprint::CellState::Free), extents);
  EXPECT_EQ(vehicle.reason, world::CurrentWorldProofReason::DynamicObstaclePresent);
}

TEST(CanonicalRetainedWorldRevalidation, RejectsUnsealedCurrentInputs)
{
  const auto execution_plan = make_plan();
  const auto cursor = plan::resolve_execution_cursor(execution_plan, 10.6);
  const footprint::FootprintExtents extents{0.15, 0.15, 0.10, 0.10, 0.0};

  auto changed_pose = make_request();
  changed_pose.control_pose.x_m += 0.1;
  EXPECT_EQ(
    world::build_current_world_retained_proof(
      execution_plan, cursor, changed_pose,
      make_grid(footprint::CellState::Free), extents).reason,
    world::CurrentWorldProofReason::ControlPoseIdentityMismatch);

  auto changed_frame = make_request();
  changed_frame.course_frame_knots[1].y_m += 0.1;
  EXPECT_EQ(
    world::build_current_world_retained_proof(
      execution_plan, cursor, changed_frame,
      make_grid(footprint::CellState::Free), extents).reason,
    world::CurrentWorldProofReason::CourseFrameIdentityMismatch);

  auto stale_obstacle = make_request();
  stale_obstacle.obstacles.current = false;
  EXPECT_EQ(
    world::build_current_world_retained_proof(
      execution_plan, cursor, stale_obstacle,
      make_grid(footprint::CellState::Free), extents).reason,
    world::CurrentWorldProofReason::ObstacleObservationUnavailable);

  auto changed_obstacle_identity = make_request();
  changed_obstacle_identity.obstacles.observation_sec += 0.1;
  EXPECT_EQ(
    world::build_current_world_retained_proof(
      execution_plan, cursor, changed_obstacle_identity,
      make_grid(footprint::CellState::Free), extents).reason,
    world::CurrentWorldProofReason::ObstacleTubeIdentityMismatch);
}

TEST(CanonicalRetainedWorldRevalidation, BuildsFollowProofFromCurrentTargetTube)
{
  const auto execution_plan = make_follow_plan();
  const auto cursor = plan::resolve_execution_cursor(execution_plan, 10.6);
  const auto request = make_follow_request();
  const footprint::FootprintExtents extents{0.15, 0.15, 0.10, 0.10, 0.0};
  const auto result = world::build_follow_current_world_retained_proof(
    execution_plan, cursor, request,
    make_grid(footprint::CellState::Free), extents);

  ASSERT_EQ(result.reason, world::FollowCurrentWorldProofReason::Accepted);
  ASSERT_TRUE(result.proof.has_value());
  EXPECT_EQ(result.proof->current.target_id, "d2");
  EXPECT_EQ(result.proof->current.target_obstacle_generation, 8U);
  EXPECT_EQ(result.proof->stage_evaluations.size(), 2U);
  EXPECT_GT(result.minimum_gap_m, request.target.hard_gap_m);
}

TEST(CanonicalRetainedWorldRevalidation, RejectsFollowTargetIdentityAndTubeMutation)
{
  const auto execution_plan = make_follow_plan();
  const auto cursor = plan::resolve_execution_cursor(execution_plan, 10.6);
  const footprint::FootprintExtents extents{0.15, 0.15, 0.10, 0.10, 0.0};

  auto changed_target = make_follow_request();
  changed_target.target.target_id = "d3";
  changed_target.target.tube_id =
    world::fingerprint_follow_obstacle_observation(changed_target.target);
  changed_target.current.obstacle_tube_id = changed_target.target.tube_id;
  EXPECT_EQ(
    world::build_follow_current_world_retained_proof(
      execution_plan, cursor, changed_target,
      make_grid(footprint::CellState::Free), extents).reason,
    world::FollowCurrentWorldProofReason::TargetIdentityMismatch);

  auto changed_tube = make_follow_request();
  changed_tube.target.target_relative_progress_m.back() += 0.5;
  EXPECT_EQ(
    world::build_follow_current_world_retained_proof(
      execution_plan, cursor, changed_tube,
      make_grid(footprint::CellState::Free), extents).reason,
    world::FollowCurrentWorldProofReason::TargetTubeIdentityMismatch);
}

TEST(CanonicalRetainedWorldRevalidation, RejectsFollowCurrentAndFutureHardGap)
{
  const auto execution_plan = make_follow_plan();
  const auto cursor = plan::resolve_execution_cursor(execution_plan, 10.6);
  const footprint::FootprintExtents extents{0.15, 0.15, 0.10, 0.10, 0.0};

  auto current_gap = make_follow_request();
  current_gap.target.target_relative_progress_m = {2.9, 3.1, 3.6};
  current_gap.target.tube_id =
    world::fingerprint_follow_obstacle_observation(current_gap.target);
  current_gap.current.obstacle_tube_id = current_gap.target.tube_id;
  EXPECT_EQ(
    world::build_follow_current_world_retained_proof(
      execution_plan, cursor, current_gap,
      make_grid(footprint::CellState::Free), extents).reason,
    world::FollowCurrentWorldProofReason::InitialHardGapViolation);

  auto future_gap = make_follow_request();
  future_gap.target.target_relative_progress_m = {5.0, 3.0, 3.0};
  // A regressing target tube is malformed and must fail before it can be used
  // to manufacture a future gap certificate.
  future_gap.target.tube_id = 1U;
  future_gap.current.obstacle_tube_id = future_gap.target.tube_id;
  EXPECT_EQ(
    world::build_follow_current_world_retained_proof(
      execution_plan, cursor, future_gap,
      make_grid(footprint::CellState::Free), extents).reason,
    world::FollowCurrentWorldProofReason::TargetObservationUnavailable);

  auto stage_gap = make_follow_request();
  stage_gap.target.target_relative_progress_m = {5.0, 5.0, 5.0};
  stage_gap.target.hard_gap_m = 4.3;
  stage_gap.target.tube_id =
    world::fingerprint_follow_obstacle_observation(stage_gap.target);
  stage_gap.current.obstacle_tube_id = stage_gap.target.tube_id;
  EXPECT_EQ(
    world::build_follow_current_world_retained_proof(
      execution_plan, cursor, stage_gap,
      make_grid(footprint::CellState::Free), extents).reason,
    world::FollowCurrentWorldProofReason::StageGapViolation);
}

TEST(CanonicalRetainedWorldRevalidation, BuildsOvertakeProofFromCurrentCorridor)
{
  const auto execution_plan = make_overtake_plan();
  const auto cursor = plan::resolve_execution_cursor(execution_plan, 10.6);
  const auto request = make_overtake_request();
  const footprint::FootprintExtents extents{0.15, 0.15, 0.10, 0.10, 0.0};
  const auto result = world::build_overtake_current_world_retained_proof(
    execution_plan, cursor, request,
    make_grid(footprint::CellState::Free), extents);

  ASSERT_EQ(result.reason, world::OvertakeCurrentWorldProofReason::Accepted);
  ASSERT_TRUE(result.proof.has_value());
  EXPECT_EQ(result.proof->current.target_id, "d2");
  EXPECT_EQ(result.proof->current.target_obstacle_generation, 8U);
  EXPECT_EQ(result.proof->stage_evaluations.size(), 2U);
  EXPECT_GT(result.minimum_corridor_reserve_m, 0.0);
}

TEST(CanonicalRetainedWorldRevalidation, RejectsOvertakeIdentityAndCorridorMutation)
{
  const auto execution_plan = make_overtake_plan();
  const auto cursor = plan::resolve_execution_cursor(execution_plan, 10.6);
  const footprint::FootprintExtents extents{0.15, 0.15, 0.10, 0.10, 0.0};

  auto changed_target = make_overtake_request();
  changed_target.corridor.target_id = "d3";
  changed_target.corridor.tube_id =
    world::fingerprint_overtake_corridor_observation(changed_target.corridor);
  changed_target.current.obstacle_tube_id = changed_target.corridor.tube_id;
  EXPECT_EQ(
    world::build_overtake_current_world_retained_proof(
      execution_plan, cursor, changed_target,
      make_grid(footprint::CellState::Free), extents).reason,
    world::OvertakeCurrentWorldProofReason::TargetIdentityMismatch);

  auto changed_side = make_overtake_request();
  changed_side.current.execution_side_sign = -1;
  EXPECT_EQ(
    world::build_overtake_current_world_retained_proof(
      execution_plan, cursor, changed_side,
      make_grid(footprint::CellState::Free), extents).reason,
    world::OvertakeCurrentWorldProofReason::ExecutionSideMismatch);

  auto changed_corridor = make_overtake_request();
  changed_corridor.corridor.lateral_upper_m.back() -= 0.1;
  EXPECT_EQ(
    world::build_overtake_current_world_retained_proof(
      execution_plan, cursor, changed_corridor,
      make_grid(footprint::CellState::Free), extents).reason,
    world::OvertakeCurrentWorldProofReason::CorridorIdentityMismatch);
}

TEST(CanonicalRetainedWorldRevalidation, RejectsOvertakeUncertifiedReleaseAndBlockedStage)
{
  const auto execution_plan = make_overtake_plan();
  const auto cursor = plan::resolve_execution_cursor(execution_plan, 10.6);
  const footprint::FootprintExtents extents{0.15, 0.15, 0.10, 0.10, 0.0};

  auto release = make_overtake_request();
  release.corridor.target_exclusion_encoded = false;
  release.corridor.tube_id =
    world::fingerprint_overtake_corridor_observation(release.corridor);
  release.current.obstacle_tube_id = release.corridor.tube_id;
  EXPECT_EQ(
    world::build_overtake_current_world_retained_proof(
      execution_plan, cursor, release,
      make_grid(footprint::CellState::Free), extents).reason,
    world::OvertakeCurrentWorldProofReason::TargetReleaseUncertified);

  auto blocked = make_overtake_request();
  blocked.corridor.lateral_upper_m = {0.5, 0.11, 0.10};
  blocked.corridor.tube_id =
    world::fingerprint_overtake_corridor_observation(blocked.corridor);
  blocked.current.obstacle_tube_id = blocked.corridor.tube_id;
  EXPECT_EQ(
    world::build_overtake_current_world_retained_proof(
      execution_plan, cursor, blocked,
      make_grid(footprint::CellState::Free), extents).reason,
    world::OvertakeCurrentWorldProofReason::StageCorridorViolation);
}
