#include "multi_purpose_mpc_ros/mpcc_certified_stop_successor_observation.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace
{

namespace observation =
  multi_purpose_mpc_ros::mpcc_certified_stop_successor_observation;
namespace contract = multi_purpose_mpc_ros::mpcc_execution_contract;

observation::Published published()
{
  observation::Published published;
  auto & evidence = published.evidence;
  evidence.source_decision_id = 10U;
  evidence.solution_id = 20U;
  evidence.problem_fingerprint = 30U;
  evidence.source_intent = contract::ControlIntent::ShiftOut;
  evidence.control_origin_sec = 1.0;
  evidence.exact_trajectory.elapsed_time_sec = {0.01, 0.02, 0.03};
  evidence.exact_trajectory.velocity_mps = {2.0, 1.9, 1.8};
  evidence.world_prediction.first = {10.0, 10.1, 10.2};
  evidence.world_prediction.second = {1.0, 1.0, 1.0};
  evidence.world_yaw_rad = {3.13, -3.13, -3.12};
  using ActuationSample =
    typename decltype(evidence.actuation_samples)::value_type;
  const auto actuation_sample = [] (
      const double elapsed_sec, const double duration_sec,
      const double acceleration_mps2, const double steering_rate_radps,
      const double velocity_mps, const double steering_rad,
      const std::size_t command_interval_index) {
      ActuationSample sample;
      sample.elapsed_time_sec = elapsed_sec;
      sample.duration_sec = duration_sec;
      sample.acceleration_mps2 = acceleration_mps2;
      sample.effective_acceleration_mps2 = acceleration_mps2;
      sample.steering_rate_radps = steering_rate_radps;
      sample.end_velocity_mps = velocity_mps;
      sample.end_steering_rad = steering_rad;
      sample.end_response_steering_rad = steering_rad;
      sample.path_curvature_radpm = 0.0;
      sample.virtual_progress_speed_mps = velocity_mps;
      sample.command_interval_index = command_interval_index;
      return sample;
    };
  evidence.actuation_samples = {
    actuation_sample(0.01, 0.01, 0.0, 0.1, 2.0, 0.10, 0U),
    actuation_sample(0.02, 0.01, 0.0, 0.1, 1.9, 0.11, 0U),
    actuation_sample(0.03, 0.01, -3.0, 0.1, 1.8, 0.12, 1U),
  };
  evidence.publisher_interval_sample_count = 2U;
  published.publication_sec = 0.98;
  return published;
}

TEST(CertifiedStopSuccessorObservation, SamplesTheNextControlOrigin)
{
  auto source = published();
  const observation::CurrentControlOrigin current{
    11U, true, 1.0, 1.025, 10.15, 1.0, -3.125, 1.85, 0.115};

  const auto result = observation::evaluate(source, current);

  EXPECT_EQ(result.reason, observation::Reason::Sampled);
  EXPECT_EQ(result.lower_sample_index, 1U);
  EXPECT_EQ(result.upper_sample_index, 2U);
  EXPECT_NEAR(result.interpolation_alpha, 0.5, 1e-12);
  EXPECT_NEAR(result.publisher_boundary_sec, 0.02, 1e-12);
  EXPECT_NEAR(result.position_error_m, 0.0, 1e-12);
  EXPECT_NEAR(result.speed_error_mps, 0.0, 1e-12);
  EXPECT_NEAR(result.steering_error_rad, 0.0, 1e-12);
  EXPECT_LT(result.yaw_error_rad, 1e-12);
  EXPECT_FALSE(std::isfinite(result.current_time_steering_error_rad));
  EXPECT_FALSE(std::isfinite(
      result.response_control_origin_steering_error_rad));
  EXPECT_FALSE(std::isfinite(
      result.previous_published_steering_error_rad));
}

TEST(CertifiedStopSuccessorObservation, ClassifiesEachSteeringOwnerIndependently)
{
  auto source = published();
  const observation::CurrentControlOrigin current{
    11U, true, 1.0, 1.025, 10.15, 1.0, -3.125, 1.85,
    0.115, 0.105, 0.125, 0.110};

  const auto result = observation::evaluate(source, current);

  ASSERT_EQ(result.reason, observation::Reason::Sampled);
  EXPECT_NEAR(result.steering_error_rad, 0.0, 1e-12);
  EXPECT_NEAR(result.current_time_steering_error_rad, -0.010, 1e-12);
  EXPECT_NEAR(
    result.response_control_origin_steering_error_rad, 0.010, 1e-12);
  EXPECT_NEAR(result.previous_published_steering_error_rad, -0.005, 1e-12);
}

TEST(CertifiedStopSuccessorObservation, RejectsASecondDecisionAtTheSameOrigin)
{
  auto source = published();
  const observation::CurrentControlOrigin current{
    10U, true, 1.0, 1.02, 10.1, 1.0, -3.13, 1.9, 0.11};

  const auto result = observation::evaluate(source, current);

  EXPECT_EQ(result.reason, observation::Reason::InvalidIdentity);
}

TEST(CertifiedStopSuccessorObservation, RejectsDetachedActuationShape)
{
  auto source = published();
  source.evidence.actuation_samples.pop_back();
  const observation::CurrentControlOrigin current{
    11U, true, 1.0, 1.02, 10.1, 1.0, -3.13, 1.9, 0.11};

  const auto result = observation::evaluate(source, current);

  EXPECT_EQ(result.reason, observation::Reason::InvalidShape);
}

TEST(CertifiedStopSuccessorObservation, RejectsAControlOriginOutsideTheProof)
{
  auto source = published();
  const observation::CurrentControlOrigin current{
    11U, true, 1.0, 1.04, 10.3, 1.0, -3.11, 1.7, 0.13};

  const auto result = observation::evaluate(source, current);

  EXPECT_EQ(result.reason, observation::Reason::TimeOutsideSuccessor);
}

}  // namespace
