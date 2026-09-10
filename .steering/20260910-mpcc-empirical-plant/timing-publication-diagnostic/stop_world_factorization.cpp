#define main preserved_certified_plan_driver_main
#include "../replay_certified_plan.cpp"
#undef main

int main(int argc, char **argv)
{
  if (argc != 3) return 2;
  const std::filesystem::path path=argv[1];
  const auto root=YAML::LoadFile(path.string());
  const auto captured=root["revalidation_evidence"];
  const auto previous=root["previous_accepted_revalidation_evidence"];
  auto current=mpcc_observation::read_request(captured["request"],path.parent_path());
  current.plan=read_plan(captured["certified_plan_evidence"],path);
  const auto old=mpcc_observation::read_request(previous["request"],path.parent_path());
  const auto & a=*current.plan->execution_artifact;
  const auto initial=a.semantic_initial_state.value();
  namespace adapter=m::mpcc_rate_resolved_physical_adapter;
  const auto full=adapter::build_continuation(a,artifact::resolve_cursor(a,a.prediction_origin_sec),
    {initial.lateral_m,initial.lag_m,initial.heading_offset_rad,initial.velocity_mps,
     initial.progress_m,initial.steering_rad,initial.response_steering_rad,
     initial.lateral_velocity_mps,initial.yaw_rate_radps});
  if(!full.exact_trajectory)throw std::runtime_error("model trajectory unavailable");
  const double elapsed=current.execution_clock.first_published_artifact_elapsed_sec+
    current.control_origin_sec-current.execution_clock.first_published_control_origin_sec;
  const auto & trajectory=*full.exact_trajectory;
  const auto it=std::find_if(trajectory.elapsed_time_sec.begin(),trajectory.elapsed_time_sec.end(),
    [elapsed](double t){return std::abs(t-elapsed)<1e-8;});
  if(it==trajectory.elapsed_time_sec.end())throw std::runtime_error("exact modeled cursor unavailable");
  const auto i=std::distance(trajectory.elapsed_time_sec.begin(),it);
  const auto & sample=full.actuation_samples.at(i);
  const auto frame=m::mpc_stage_geometry::sample_course_frame(
    current.plan->physical_snapshot->course_frame_knots,trajectory.progress_m[i]);
  if(!frame)throw std::runtime_error("course frame unavailable");
  const decltype(current.control_pose) nominal_pose{
    frame->x_m+std::cos(frame->heading_rad)*trajectory.lag_m[i]-std::sin(frame->heading_rad)*trajectory.lateral_m[i],
    frame->y_m+std::sin(frame->heading_rad)*trajectory.lag_m[i]+std::cos(frame->heading_rad)*trajectory.lateral_m[i],
    frame->heading_rad+trajectory.heading_offset_rad[i]};
  YAML::Node out;out["authority"]=false;out["source"]=a.identity.sequence;
  out["meaning"]="Four physical-origin diagnostics with zero observation delay/prefix; actual449 artifact, modeled vs current ego, previous955 vs current956 peers with original observation epochs. Not full publication authority or an exact955-to956 revalidation pair.";
  out["elapsed_sec"]=elapsed;out["actual_origin_speed"]=current.control_origin_speed_mps;
  out["model_origin_speed"]=trajectory.velocity_mps[i];
  for(const bool nominal:{false,true})for(const bool old_peer:{false,true}){
    auto r=current;r.now_sec=r.control_origin_sec;r.publication_prefix.reset();
    r.publication_prefix_required=false;
    if(old_peer)r.obstacles=old.obstacles;
    if(nominal){
      bool found=false;
      for(const auto & knot:knots(root["source"]["wall_course_frame_knots"])){
        const auto project=[&](const auto & p){return knot.progress_m+std::cos(knot.heading_rad)*(p.x_m-knot.x_m)+std::sin(knot.heading_rad)*(p.y_m-knot.y_m);};
        if(std::abs(project(current.control_pose)-current.control_origin_physical_progress_m)>1e-9)continue;
        if(found)throw std::runtime_error("ambiguous projection");found=true;
        r.control_origin_physical_progress_m=project(nominal_pose);
      }
      if(!found)throw std::runtime_error("live tangent projection unavailable");
      r.control_pose=nominal_pose;r.control_origin_speed_mps=trajectory.velocity_mps[i];
      r.current_steering_rad=sample.end_steering_rad;
      r.current_response_steering_rad=sample.end_response_steering_rad;
      r.current_lateral_velocity_mps=sample.end_lateral_velocity_mps;
      r.current_yaw_rate_radps=sample.end_yaw_rate_radps;
    }
    r.current_speed_mps=r.control_origin_speed_mps;
    r.current_time_steering_rad=r.current_response_steering_rad;
    r.measured_to_control_path={r.control_pose};r.measured_to_control_elapsed_sec={0.};
    const auto checked=retained::evaluate(r);const auto stop=retained::evaluate_stop_successor(r);
    YAML::Node row;row["ego"]=nominal?"modeled":"current";row["peers"]=old_peer?"previous955":"current956";
    row["peer_generation"]=r.obstacles.generation;row["peer_observed_sec"]=r.obstacles.observed_sec;
    row["retained"]=retained::to_string(checked.reason);row["terminal_clearance"]=checked.terminal_stop_minimum_dynamic_clearance_m;
    row["stop"]=retained::to_string(stop.reason);row["stop_clearance"]=stop.dynamic_clearance.minimum_clearance_m;
    row["stop_samples"]=stop.actuation_samples.size();out["arms"].push_back(row);
  }
  YAML::Emitter e;e.SetDoublePrecision(17);e<<out;std::ofstream(argv[2])<<e.c_str()<<'\n';std::cout<<e.c_str()<<'\n';
}
