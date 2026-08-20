#include <gtest/gtest.h>

#include <multi_purpose_mpc_ros/mpc_velocity_limit.hpp>

#include <limits>
#include <stdexcept>

TEST(MpcVelocityLimit, RampsDownByReachableDeceleration)
{
  const auto limits = multi_purpose_mpc_ros::mpc_velocity_limit::build_reachable_limits(
    multi_purpose_mpc_ros::mpc_velocity_limit::ReachableLimitRequest{
      5, 4.5, 3.0, 1.35, 0.025});

  ASSERT_EQ(limits.size(), 5U);
  EXPECT_DOUBLE_EQ(limits[0], 4.46625);
  EXPECT_DOUBLE_EQ(limits[1], 4.4325);
  EXPECT_DOUBLE_EQ(limits[4], 4.33125);
}

TEST(MpcVelocityLimit, SettlesAtFinalLimitWithoutUndershoot)
{
  const auto limits = multi_purpose_mpc_ros::mpc_velocity_limit::build_reachable_limits(
    multi_purpose_mpc_ros::mpc_velocity_limit::ReachableLimitRequest{
      20, 1.0, 0.0, 2.0, 0.1});

  ASSERT_EQ(limits.size(), 20U);
  EXPECT_DOUBLE_EQ(limits[0], 0.8);
  EXPECT_DOUBLE_EQ(limits[4], 0.0);
  EXPECT_DOUBLE_EQ(limits.back(), 0.0);
}

TEST(MpcVelocityLimit, KeepsHigherTargetImmediatelyAvailable)
{
  const auto limits = multi_purpose_mpc_ros::mpc_velocity_limit::build_reachable_limits(
    multi_purpose_mpc_ros::mpc_velocity_limit::ReachableLimitRequest{
      3, 2.0, 3.0, 1.35, 0.025});

  ASSERT_EQ(limits.size(), 3U);
  EXPECT_DOUBLE_EQ(limits[0], 3.0);
  EXPECT_DOUBLE_EQ(limits[1], 3.0);
  EXPECT_DOUBLE_EQ(limits[2], 3.0);
}

TEST(MpcVelocityLimit, RejectsInvalidInputs)
{
  auto request = multi_purpose_mpc_ros::mpc_velocity_limit::ReachableLimitRequest{
    3, 2.0, 0.0, 1.35, 0.025};
  request.current_velocity_mps = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(
    multi_purpose_mpc_ros::mpc_velocity_limit::build_reachable_limits(request),
    std::invalid_argument);

  request.current_velocity_mps = 2.0;
  request.time_step_sec = 0.0;
  EXPECT_THROW(
    multi_purpose_mpc_ros::mpc_velocity_limit::build_reachable_limits(request),
    std::invalid_argument);
}

TEST(MpcVelocityLimit, EnablesSimulationSolverFailureCrawlOnlyOnClearCruise)
{
  using multi_purpose_mpc_ros::mpc_velocity_limit::SolverFailureCrawlRequest;
  using multi_purpose_mpc_ros::mpc_velocity_limit::resolve_solver_failure_crawl;

  SolverFailureCrawlRequest request;
  request.simulation_environment = true;
  request.enabled = true;
  request.control_enabled = true;
  request.solver_fallback = true;
  request.unrestricted_cruise = true;
  request.current_static_footprint_clear = true;
  request.lateral_error_m = 0.1;
  request.heading_error_rad = -0.2;
  request.max_lateral_error_m = 0.5;
  request.max_heading_error_rad = 0.35;
  request.configured_speed_mps = 1.0;
  request.effective_speed_limit_mps = 0.8;

  const auto enabled = resolve_solver_failure_crawl(request);
  EXPECT_TRUE(enabled.active);
  EXPECT_DOUBLE_EQ(enabled.target_speed_mps, 0.8);

  request.front_vehicle_detected = true;
  EXPECT_FALSE(resolve_solver_failure_crawl(request).active);
  request.front_vehicle_detected = false;
  request.unrestricted_cruise = false;
  EXPECT_FALSE(resolve_solver_failure_crawl(request).active);
}

TEST(MpcVelocityLimit, SolverFailureCrawlFailsClosedOutsideSimulationOrOnInvalidSpeed)
{
  using multi_purpose_mpc_ros::mpc_velocity_limit::SolverFailureCrawlRequest;
  using multi_purpose_mpc_ros::mpc_velocity_limit::resolve_solver_failure_crawl;

  SolverFailureCrawlRequest request;
  request.enabled = true;
  request.control_enabled = true;
  request.solver_fallback = true;
  request.unrestricted_cruise = true;
  request.current_static_footprint_clear = true;
  request.max_lateral_error_m = 0.5;
  request.max_heading_error_rad = 0.35;
  request.configured_speed_mps = 1.0;
  request.effective_speed_limit_mps = 2.0;
  EXPECT_FALSE(resolve_solver_failure_crawl(request).active);

  request.simulation_environment = true;
  request.configured_speed_mps = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(resolve_solver_failure_crawl(request).active);
}

TEST(MpcVelocityLimit, SolverFailureCrawlRequiresSafePathTrackingEnvelope)
{
  using multi_purpose_mpc_ros::mpc_velocity_limit::SolverFailureCrawlRequest;
  using multi_purpose_mpc_ros::mpc_velocity_limit::resolve_solver_failure_crawl;

  SolverFailureCrawlRequest request;
  request.simulation_environment = true;
  request.enabled = true;
  request.control_enabled = true;
  request.solver_fallback = true;
  request.unrestricted_cruise = true;
  request.current_static_footprint_clear = true;
  request.max_lateral_error_m = 0.5;
  request.max_heading_error_rad = 0.35;
  request.configured_speed_mps = 1.0;
  request.effective_speed_limit_mps = 2.0;

  request.lateral_error_m = -0.5;
  request.heading_error_rad = 0.35;
  EXPECT_TRUE(resolve_solver_failure_crawl(request).active);

  request.lateral_error_m = 0.5001;
  EXPECT_FALSE(resolve_solver_failure_crawl(request).active);
  request.lateral_error_m = 0.0;
  request.heading_error_rad = -0.3501;
  EXPECT_FALSE(resolve_solver_failure_crawl(request).active);

  // Regression: P2 entered crawl at these errors, then traveled from the
  // hairpin to an unrecoverable e_y=-4.603 m before Recovery started.
  request.lateral_error_m = 0.628;
  request.heading_error_rad = -0.740;
  EXPECT_FALSE(resolve_solver_failure_crawl(request).active);

  request.heading_error_rad = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(resolve_solver_failure_crawl(request).active);
  request.heading_error_rad = 0.0;
  request.max_lateral_error_m = -0.1;
  EXPECT_FALSE(resolve_solver_failure_crawl(request).active);
}

TEST(MpcVelocityLimit, SolverFailureCrawlRequiresClearCurrentStaticFootprint)
{
  using multi_purpose_mpc_ros::mpc_velocity_limit::SolverFailureCrawlRequest;
  using multi_purpose_mpc_ros::mpc_velocity_limit::resolve_solver_failure_crawl;

  SolverFailureCrawlRequest request;
  request.simulation_environment = true;
  request.enabled = true;
  request.control_enabled = true;
  request.solver_fallback = true;
  request.unrestricted_cruise = true;
  request.current_static_footprint_clear = false;
  request.max_lateral_error_m = 0.5;
  request.max_heading_error_rad = 0.35;
  request.configured_speed_mps = 1.0;
  request.effective_speed_limit_mps = 2.0;

  EXPECT_FALSE(resolve_solver_failure_crawl(request).active);
  request.current_static_footprint_clear = true;
  EXPECT_TRUE(resolve_solver_failure_crawl(request).active);
}

TEST(MpcVelocityLimit, HoldsSafeDynamicEscapeThroughIsolatedSolverFailure)
{
  using multi_purpose_mpc_ros::mpc_velocity_limit::SolverFailureContinuationRequest;
  using multi_purpose_mpc_ros::mpc_velocity_limit::resolve_solver_failure_continuation;

  SolverFailureContinuationRequest request;
  request.simulation_environment = true;
  request.enabled = true;
  request.control_enabled = true;
  request.solver_fallback = true;
  request.dynamic_obstacle_escape_active = true;
  request.current_static_footprint_clear = true;
  request.execution_path_validated = true;
  request.tracking_envelope_valid = true;
  request.consecutive_failure_count = 1;
  request.maximum_hold_cycles = 4;
  request.current_speed_mps = 5.5;
  request.effective_speed_limit_mps = 5.0;

  const auto decision = resolve_solver_failure_continuation(request);
  EXPECT_TRUE(decision.active);
  EXPECT_DOUBLE_EQ(decision.target_speed_mps, 5.0);
  EXPECT_EQ(
    decision.block_reason,
    multi_purpose_mpc_ros::mpc_velocity_limit::
    SolverFailureContinuationBlockReason::None);
}

TEST(MpcVelocityLimit, DynamicEscapeContinuationFailsClosed)
{
  using multi_purpose_mpc_ros::mpc_velocity_limit::SolverFailureContinuationBlockReason;
  using multi_purpose_mpc_ros::mpc_velocity_limit::SolverFailureContinuationRequest;
  using multi_purpose_mpc_ros::mpc_velocity_limit::resolve_solver_failure_continuation;

  SolverFailureContinuationRequest request;
  request.simulation_environment = true;
  request.enabled = true;
  request.control_enabled = true;
  request.solver_fallback = true;
  request.dynamic_obstacle_escape_active = true;
  request.current_static_footprint_clear = true;
  request.execution_path_validated = true;
  request.tracking_envelope_valid = true;
  request.consecutive_failure_count = 1;
  request.maximum_hold_cycles = 4;
  request.current_speed_mps = 5.5;
  request.effective_speed_limit_mps = 10.0;

  request.emergency_active = true;
  EXPECT_EQ(
    resolve_solver_failure_continuation(request).block_reason,
    SolverFailureContinuationBlockReason::EmergencyActive);
  request.emergency_active = false;
  request.consecutive_failure_count = 5;
  EXPECT_EQ(
    resolve_solver_failure_continuation(request).block_reason,
    SolverFailureContinuationBlockReason::FailureBudgetExceeded);
  request.consecutive_failure_count = 1;
  request.current_static_footprint_clear = false;
  EXPECT_EQ(
    resolve_solver_failure_continuation(request).block_reason,
    SolverFailureContinuationBlockReason::StaticFootprintUnsafe);
  request.current_static_footprint_clear = true;
  request.execution_path_validated = false;
  EXPECT_EQ(
    resolve_solver_failure_continuation(request).block_reason,
    SolverFailureContinuationBlockReason::ExecutionPathUnvalidated);
  request.execution_path_validated = true;
  request.tracking_envelope_valid = false;
  EXPECT_EQ(
    resolve_solver_failure_continuation(request).block_reason,
    SolverFailureContinuationBlockReason::TrackingEnvelopeUnsafe);
}
