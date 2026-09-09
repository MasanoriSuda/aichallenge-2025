// Bounded geometric witness search only. No QP certificate or production authority.
#include "../20260909-mpcc-final-authority-observation/revalidation_input.hpp"
#include "multi_purpose_mpc_ros/mpcc_architecture_snapshot.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_physical_wall.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
int main(int argc,char **argv) {
  if(argc!=3)return 2;
  namespace m=multi_purpose_mpc_ros;
  namespace dyn=m::mpcc_rate_resolved_dynamic_proof;
  const auto root=YAML::LoadFile(argv[1]);std::string detail;
  const auto record=m::mpcc_architecture_snapshot::load_recorded_interaction_snapshot(argv[1],&detail);
  if(!record)throw std::runtime_error(detail);
  const auto & source=record->source;
  const auto request=mpcc_observation::read_request(root["revalidation_evidence"]["request"],std::filesystem::path(argv[1]).parent_path());
  const auto artifact=root["revalidation_evidence"]["inspected_artifact"];
  const double wb=artifact["wheelbase_m"].as<double>(), gain=artifact["yaw_response_gain"].as<double>(),
    tau=artifact["yaw_response_time_constant_sec"].as<double>(),max_delta=artifact["maximum_abs_steering_rad"].as<double>(),
    max_rate=artifact["maximum_abs_steering_rate_radps"].as<double>();
  const auto footprint=m::mpcc_rate_resolved_physical_wall::resolve_clearance_footprint(source.replay_world->physical_footprint,source.replay_world->hard_wall_clearance_m);
  if(!footprint)throw std::runtime_error("invalid clearance footprint");
  YAML::Node report;report["world_fingerprint"]=record->interaction_fingerprint;report["authority"]=false;
  report["limits"]="Independent midpoint physical controls oracle, 5ms step, same exact current Request, actuator/model/wall/all-peer limits. Does not establish original seven-state QP state/progress/terminal constraints, a solved certificate or current publication. Rejections do not establish physical infeasibility.";
  int index=0;
  for(double target_fraction:{-1.,-.5,0.,.5,1.})for(double brake_fraction:{1.,.5,.25})for(double accelerate_sec:{0.,.25,.5}) {
    auto pose=request.control_pose;double v=request.control_origin_speed_mps;
    double delta=request.previous_published_steering_rad,rho=request.current_response_steering_rad;
    std::vector<m::recovery_footprint::Pose2D> path=request.measured_to_control_path;
    dyn::Result dynamic;dyn::observe_timed_path(source.replay_world->physical_footprint,path,request.measured_to_control_elapsed_sec,source.replay_world->swept_step_m,request.obstacles,dynamic);
    const double delay=request.control_origin_sec-request.now_sec;
    YAML::Node trace(YAML::NodeType::Sequence);
    constexpr double h=.005;double first_rest=std::numeric_limits<double>::infinity();
    for(int step=0;step<1000;++step) {
      const double t=step*h;
      const double a=t+1e-10<accelerate_sec ? request.maximum_acceleration_mps2 : std::max(request.minimum_acceleration_mps2*brake_fraction,-v/h);
      const double rate=std::clamp((target_fraction*max_delta-delta)/h,-max_rate,max_rate);
      const double vm=v+.5*h*a,dm=delta+.5*h*rate,rm=rho+.5*h*(gain*delta-rho)/tau;
      const double ym=pose.yaw_rad+.5*h*v*std::tan(rho)/wb;
      const auto previous=pose;
      pose.x_m+=h*vm*std::cos(ym);pose.y_m+=h*vm*std::sin(ym);pose.yaw_rad+=h*vm*std::tan(rm)/wb;
      v+=h*a;delta+=h*rate;rho+=h*(gain*dm-rm)/tau;
      if(!std::isfinite(first_rest) && v<=1e-10)first_rest=t+h;
      path.push_back(pose);
      dyn::observe_segment(source.replay_world->physical_footprint,previous,pose,delay+t,delay+t+h,source.replay_world->swept_step_m,request.obstacles,dynamic);
      YAML::Node n;n["t"]=t+h;n["x"]=pose.x_m;n["y"]=pose.y_m;n["yaw"]=pose.yaw_rad;
      n["v"]=v;n["delta"]=delta;n["rho"]=rho;n["a"]=a;n["rate"]=rate;trace.push_back(n);
    }
    dyn::finalize(request.obstacles,dynamic);
    const auto wall=m::recovery_footprint::evaluate_clear_footprint_path(*request.current_wall_grid,*footprint,path,source.replay_world->swept_step_m);
    YAML::Node row;row["index"]=index++;row["target_fraction"]=target_fraction;row["brake_fraction"]=brake_fraction;row["accelerate_sec"]=accelerate_sec;
    row["wall_clear"]=wall.valid&&wall.clear;row["wall_reject_index"]=wall.rejected_path_index;
    row["dynamic_clear"]=dynamic.valid&&dynamic.clear;row["minimum_dynamic_m"]=dynamic.minimum_clearance_m;
    row["dynamic_min_time"]=dynamic.minimum_clearance_elapsed_sec;row["first_rest_sec"]=first_rest;
    row["physical_clear"]=wall.valid&&wall.clear&&dynamic.valid&&dynamic.clear&&v<=1e-10;
    if(row["physical_clear"].as<bool>())row["trace"]=trace;
    report["arms"].push_back(row);std::cout<<row["index"]<<" physical_clear="<<row["physical_clear"]<<" dynamic_min="<<dynamic.minimum_clearance_m<<" wall="<<row["wall_clear"]<<'\n';
  }
  YAML::Emitter e;e.SetDoublePrecision(std::numeric_limits<double>::max_digits10);e<<report;std::ofstream(argv[2])<<e.c_str()<<'\n';
}
