#include "multi_purpose_mpc_ros/mpcc_certified_stop_successor_observation.hpp"

#include <gtest/gtest.h>

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
  evidence.actuation_samples = {
    {0.01, 0.01, 0.0, 0.1, 2.0, 0.10},
    {0.02, 0.01, 0.0, 0.1, 1.9, 0.11},
    {0.03, 0.01, -3.0, 0.1, 1.8, 0.12},
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
