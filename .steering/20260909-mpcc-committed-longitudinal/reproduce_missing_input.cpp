// Native regression of the current production call on saved decision 993.
#include "multi_purpose_mpc_ros/mpc_state_prediction.hpp"
#include <cmath>
#include <iomanip>
#include <iostream>
int main() {
  namespace p=multi_purpose_mpc_ros::mpc_state_prediction;
  const auto value=p::predict_accelerating_yaw_response(
    {89631.7453596571,43133.27409346843,2.0297006749887387},
    1.843041217111899,.543785436495885,-.219927,-.126920,-.04855377654426893,
    1.087,.75,.13,.13);
  // Same filter response on observed and committed acceleration; future values
  // are only already published commands. No new delay or fitted coefficient.
  const double disturbance=.543785436495885-1.3295944422392196;
  const double expected=1.843041217111899+
    (1.329595923423767+disturbance)*.015000003+
    (-2.9595959186553955+disturbance)*.114999997;
  std::cout<<std::setprecision(17)<<"current="<<value.longitudinal_velocity_mps
    <<" expected="<<expected<<'\n';
  return std::abs(value.longitudinal_velocity_mps-expected)<1e-8 ? 0 : 1;
}
