// Offline reproduction of the exact controller call sequence at HEAD4b2474da.
// This is the real prediction/observer implementation, not a ROS node test.
#include "multi_purpose_mpc_ros/mpc_longitudinal_prediction.hpp"
#include <gtest/gtest.h>
#include <cmath>
namespace p=multi_purpose_mpc_ros::mpc_state_prediction;
class CapturedStateEpoch:public ::testing::TestWithParam<std::pair<double,double>> {};
TEST_P(CapturedStateEpoch, ClaimsTheActualControlOriginForKnownConstantMotion) {
  const auto [age,yaw_rate]=GetParam();
  const double source=10.,now=source+age,delay=.13,speed=2.,wheelbase=1.087,gain=.75,tau=.13;
  const auto epoch=p::resolve_control_observation_epoch(now,now-.02,source,.5);
  ASSERT_TRUE(epoch.valid);
  p::LongitudinalResponseObserver observer(.9,.5);
  ASSERT_TRUE(observer.record_published_command(9.8,9.8+delay,0.));
  observer.observe(source-.02,speed);
  ASSERT_TRUE(observer.observe(source,speed));
  // The source-to-now estimator precedes the separate fixed actuator delay.
  const auto intervals=observer.prediction_intervals(now,delay);
  ASSERT_TRUE(intervals);
  const double steady_steering=std::atan(yaw_rate*wheelbase/(gain*speed));
  const auto response=p::infer_response_steering(speed,yaw_rate,steady_steering,wheelbase,gain,.5,.3665191429);
  ASSERT_TRUE(response);
  const p::State2D measured{0.,0.,0.};
  const auto aligned=p::predict_constant_twist_observation({source,measured,speed,yaw_rate},now,.5);
  ASSERT_TRUE(aligned);
  const auto trajectory=p::predict_piecewise_yaw_response_trajectory(aligned->state,aligned->longitudinal_velocity_mps,
    response->steering_rad,steady_steering,steady_steering,wheelbase,gain,tau,*intervals);
  const auto expected=p::predict_constant_turn_rate(measured,speed,yaw_rate,now+delay-source);
  const auto & actual=trajectory.back().prediction;
  // Curved position uses midpoint quadrature (6.09nm error at zero age here).
  // Isolate epoch accounting with exact straight displacement and curved yaw;
  // do not conflate that known quadrature error with a clock regression.
  if(yaw_rate==0.) {
    EXPECT_NEAR(actual.state.x,expected.x,1e-9);
    EXPECT_NEAR(actual.state.y,expected.y,1e-9);
  }
  EXPECT_NEAR(aligned->stamp_sec+trajectory.back().elapsed_sec,now+delay,1e-12);
  EXPECT_NEAR(actual.state.yaw,expected.yaw,1e-9);
  EXPECT_DOUBLE_EQ(actual.longitudinal_velocity_mps,speed);
}
INSTANTIATE_TEST_SUITE_P(StraightAndCurve,CapturedStateEpoch,::testing::Values(
  std::pair<double,double>{0.,0.},std::pair<double,double>{.005,0.},std::pair<double,double>{.015,0.},
  std::pair<double,double>{0.,.15},std::pair<double,double>{.005,.15},std::pair<double,double>{.015,.15}));
