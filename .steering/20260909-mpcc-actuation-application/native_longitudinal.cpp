// Native canonical model against the separately decoded rolling-resistance
// plant component. Straight, forward, no saturation, no delay, no yaw/slip.
// Other simulator forces are deliberately not asserted to vanish in a run.
#include "multi_purpose_mpc_ros/mpcc_rate_resolved.hpp"
#include <cmath>
#include <iostream>
#include <iomanip>
int main() {
  namespace m=multi_purpose_mpc_ros::mpcc_rate_resolved;
  int failures=0;
  for(double input:{1.0,-1.0}) {
    m::LinearizationRequest r;
    r.reference_velocity_mps=2.0;r.reference_virtual_progress_speed_mps=2.0;
    r.reference_acceleration_mps2=input;r.wheelbase_m=1.087;
    r.yaw_response_gain=0.75;r.yaw_response_time_constant_sec=0.13;
    r.stage_dt_sec=0.2;
    const auto transition=m::evaluate_temporal_frenet_transition(r);
    if(!transition)return 2;
    // Vehicle.RollingResistanceAcceleration: -sign(v)*min(R,abs(v)/dt).
    const double resistance=-std::copysign(std::min(0.37,std::abs(r.reference_velocity_mps)/r.stage_dt_sec),r.reference_velocity_mps);
    const double physical=r.reference_velocity_mps+(input+resistance)*r.stage_dt_sec;
    const double canonical=transition->next_state[m::kVelocityIndex];
    const double error=canonical-physical;
    std::cout<<std::setprecision(17)<<"wire="<<input<<" rolling_only_physical="<<physical<<" canonical="<<canonical<<" difference="<<error<<'\n';
    failures+=std::abs(error)>1e-9;
  }
  std::cout<<"Mismatch cases="<<failures<<"/2; no full-plant accuracy or runtime repair claim\n";
  return failures?1:0;
}
