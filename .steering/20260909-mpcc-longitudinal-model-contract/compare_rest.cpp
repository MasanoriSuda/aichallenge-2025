// Observation-only model hypotheses; input remains named wire separately.
#include "multi_purpose_mpc_ros/mpcc_rate_resolved.hpp"
#include "multi_purpose_mpc_ros/mpc_longitudinal_prediction.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
namespace m=multi_purpose_mpc_ros::mpcc_rate_resolved;
double run(double initial,double wire,const std::string & arm) {
  double velocity=initial;
  for(int i=0;i<20;++i) {
    double offset=0.;
    if(arm=="flat-response")offset=-0.37;
    if(arm=="motion-opposing-component" && velocity!=0.)
      offset=-std::copysign(std::min(0.37,std::abs(velocity)/0.01),velocity);
    m::LinearizationRequest r;
    r.reference_velocity_mps=velocity;r.reference_acceleration_mps2=wire+offset;
    r.wheelbase_m=1.087;r.yaw_response_gain=.75;r.yaw_response_time_constant_sec=.13;r.stage_dt_sec=.01;
    const auto n=m::evaluate_temporal_frenet_transition(r);
    if(!n)throw std::runtime_error("native transition missing");
    velocity=n->next_state[m::kVelocityIndex];
  }
  return velocity;
}
int main() {
  std::cout<<std::setprecision(17);
  for(const auto & arm:{"wire-equals-net","flat-response","motion-opposing-component"})
    for(const auto & c:{std::pair<double,double>{2.,1.},{2.,-1.},{0.,0.},{0.,.37}})
      std::cout<<"arm="<<arm<<" v0="<<c.first<<" wire="<<c.second<<" v_end="<<run(c.first,c.second,arm)<<'\n';
  // A flat moving-state offset manufactures backwards unforced motion and
  // a false stationary equilibrium at positive wire input. Positive input
  // prevents Vehicle.IsCanSleepInput, while rolling resistance is zero atrest.
  if(!(run(0,0,"flat-response")<0 && std::abs(run(0,.37,"flat-response"))<1e-12 && run(0,.37,"motion-opposing-component")>0))return 2;
  std::cout<<"Flat-response rest invariants rejected; component model is not a full plant or promotion candidate\n";
  multi_purpose_mpc_ros::mpc_state_prediction::LongitudinalResponseObserver observer(.9,.5);
  std::optional<multi_purpose_mpc_ros::mpc_state_prediction::LongitudinalResponseObservation> observation;
  for(int i=0;i<=100;++i) {
    const double time=.02*i;
    if(!observer.record_published_command(time,time+.13,-3.))return 3;
    observation=observer.observe(time,0.);
  }
  if(!observation)return 4;
  const double residual=observation->response_residual_mps2();
  std::cout<<"stationary_brake_residual="<<residual<<" extrapolated_net_acceleration_for_wire1="<<1.+residual<<" local_drive_input_cap=1.37\n";
  if(!(residual>2.9 && 1.+residual>1.37))return 5;
  std::cout<<"Brake-at-rest response is input/state conditional; it is not an independent driving disturbance\n";

}
