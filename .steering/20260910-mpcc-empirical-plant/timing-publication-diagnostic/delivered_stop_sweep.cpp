#define main preserved_certified_plan_driver_main
#include "../replay_certified_plan.cpp"
#undef main

int main(int argc,char **argv)
{
  if(argc!=3)return 2;
  const std::filesystem::path path=argv[1];const auto root=YAML::LoadFile(path.string());
  namespace vehicle=m::mpcc_vehicle_model;
  YAML::Node out;out["authority"]=false;
  out["meaning"]="Conditional Stop delivery sweep, not a production delay or uncertainty bound. Preserve captured modeled current body and peer world; an already applied recent positive input may persist until brake delivery. Steering uses its separate historical delay. Five delivery hypotheses do not enclose all timings, receipt loss, model error or queue histories.";
  for(const auto key:{"previous_accepted_revalidation_evidence","revalidation_evidence"}){
    const auto e=root[key];if(!e || !e["request"] || !e["certified_plan_evidence"])continue;
    auto captured=mpcc_observation::read_request(e["request"],path.parent_path());
    captured.plan=read_plan(e["certified_plan_evidence"],path);
    const auto & parameters=captured.plan->execution_artifact->vehicle_model;
    const auto observed=captured.publication_prefix->observation;
    const auto base=vehicle::predict_prospective_publication(observed,
      {captured.now_sec,captured.minimum_acceleration_mps2,
       static_cast<float>(captured.previous_published_steering_rad*parameters.steering_wire_gain)},parameters);
    if(!base)throw std::runtime_error("Stop proposal unavailable");
    double applied=captured.minimum_acceleration_mps2;
    for(const auto & p:observed.commands)if(p.published_sec>=captured.now_sec-.2)
      applied=std::max(applied,p.wire_acceleration_mps2);
    out[key]["decision"]=captured.decision_id;out[key]["capture_status"]=e["status"];
    out[key]["current_body_is_model_estimate"]=true;
    out[key]["hypothetical_already_applied_acceleration"]=applied;
    for(const double lag:{0.,.05,.10,.15,.20}){
      auto r=captured;
      const double origin=std::max(r.control_origin_sec,r.now_sec+lag);
      std::vector<double> boundaries{r.now_sec,origin};
      if(lag>0 && r.now_sec+lag<origin)boundaries.push_back(r.now_sec+lag);
      for(const auto & p:observed.commands){
        const auto stamp=p.published_sec+observed.steering_delay_sec;
        if(stamp>r.now_sec && stamp<origin)boundaries.push_back(stamp);
      }
      if(r.now_sec+observed.steering_delay_sec<origin)
        boundaries.push_back(r.now_sec+observed.steering_delay_sec);
      std::sort(boundaries.begin(),boundaries.end());
      boundaries.erase(std::unique(boundaries.begin(),boundaries.end()),boundaries.end());
      auto state=base->current;
      std::vector<vehicle::TimedState> prefix{{r.now_sec,state}};
      for(size_t j=1;j<boundaries.size();++j){
        const double begin=boundaries[j-1],end=boundaries[j];
        const double a=begin+1e-12>=r.now_sec+lag?r.minimum_acceleration_mps2:applied;
        double steering=observed.commands.front().wire_steering_rad;
        for(const auto & p:observed.commands)if(p.published_sec+observed.steering_delay_sec<=begin)
          steering=p.wire_steering_rad;
        if(begin+1e-12>=r.now_sec+observed.steering_delay_sec)
          steering=base->proposed_packet.wire_steering_rad;
        state.desired_steering_rad=steering/parameters.steering_wire_gain;
        const auto steps=vehicle::integration_steps(end-begin,parameters.maximum_step_sec);
        const auto dt=(end-begin)/static_cast<double>(steps);
        for(size_t k=0;k<steps;++k){
          const auto next=vehicle::advance(state,{a,0},parameters,dt);
          if(!next)throw std::runtime_error("body integration failed");state=next->state;
          prefix.push_back({k+1==steps?end:begin+(k+1)*dt,state});
        }
      }
      bool found=false;
      for(const auto & knot:knots(root["source"]["wall_course_frame_knots"])){
        const double original=knot.progress_m+std::cos(knot.heading_rad)*(r.control_pose.x_m-knot.x_m)+std::sin(knot.heading_rad)*(r.control_pose.y_m-knot.y_m);
        if(std::abs(original-r.control_origin_physical_progress_m)>1e-9)continue;
        if(found)throw std::runtime_error("ambiguous projection");found=true;
        r.control_origin_physical_progress_m=knot.progress_m+std::cos(knot.heading_rad)*(state.x_m-knot.x_m)+std::sin(knot.heading_rad)*(state.y_m-knot.y_m);
        break;
      }
      if(!found)throw std::runtime_error("live projection unavailable");
      r.publication_prefix.reset();r.publication_prefix_required=false;
      r.control_origin_sec=origin;r.control_pose={state.x_m,state.y_m,state.yaw_rad};
      r.current_speed_mps=base->current.forward_velocity_mps;r.control_origin_speed_mps=state.forward_velocity_mps;
      r.current_time_steering_rad=base->current.tire_steering_rad;r.current_steering_rad=state.desired_steering_rad;
      r.current_response_steering_rad=state.tire_steering_rad;r.current_lateral_velocity_mps=state.lateral_velocity_mps;r.current_yaw_rate_radps=state.yaw_rate_radps;
      r.measured_to_control_path.clear();r.measured_to_control_elapsed_sec.clear();
      for(const auto & p:prefix){r.measured_to_control_path.push_back({p.state.x_m,p.state.y_m,p.state.yaw_rad});r.measured_to_control_elapsed_sec.push_back(p.source_sec-r.now_sec);}
      const auto stop=retained::evaluate_stop_successor(r);
      YAML::Node row;row["delivery_after_sec"]=lag;row["origin_sec"]=origin;
      row["origin_speed_mps"]=state.forward_velocity_mps;row["stop"]=retained::to_string(stop.reason);
      row["clearance_m"]=stop.dynamic_clearance.minimum_clearance_m;row["samples"]=stop.actuation_samples.size();
      row["wall_clear"]=stop.successor_path_clearance.clear;out[key]["hypotheses"].push_back(row);
    }
  }
  YAML::Emitter e;e.SetDoublePrecision(17);e<<out;std::ofstream(argv[2])<<e.c_str()<<'\n';std::cout<<e.c_str()<<'\n';
}
