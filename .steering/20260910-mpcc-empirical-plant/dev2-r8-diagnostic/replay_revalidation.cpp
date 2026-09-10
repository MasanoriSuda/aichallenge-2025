// Bounded diagnostic only. No production authority or solver invocation.
#define main preserved_certified_plan_driver_main
#include "../replay_certified_plan.cpp"
#undef main
#include "multi_purpose_mpc_ros/mpcc_vehicle_prediction.hpp"

namespace model = m::mpcc_vehicle_model;
YAML::Node describe(const retained::Request & request)
{
  const auto normal = retained::evaluate(request);
  const auto stop = retained::evaluate_stop_successor(request);
  YAML::Node row;
  row["decision"] = request.decision_id;
  row["control_speed"] = request.control_origin_speed_mps;
  row["normal"] = retained::to_string(normal.reason);
  row["normal_terminal_clearance"] = normal.terminal_stop_minimum_dynamic_clearance_m;
  row["stop"] = retained::to_string(stop.reason);
  row["stop_clearance"] = stop.dynamic_clearance.minimum_clearance_m;
  row["stop_reject_elapsed"] = stop.dynamic_clearance.rejected_elapsed_sec;
  row["stop_reject_pose"].push_back(stop.dynamic_clearance.rejected_pose.x_m);
  row["stop_reject_pose"].push_back(stop.dynamic_clearance.rejected_pose.y_m);
  row["stop_wall"] = stop.successor_path_clearance.valid && stop.successor_path_clearance.clear;
  row["stop_samples"] = stop.actuation_samples.size();
  if(stop.accepted()) {
    const auto built=bundle::build(request,stop,10000U+request.decision_id);
    row["stop_bundle"]=bundle::to_string(built.reason);
    if(built.plan) {
      auto bound=request;bound.plan=built.plan;
      bound.execution_clock={retained::ExecutionClockKind::TimeAlignedCandidate,NAN,NAN};
      const auto joined=retained::evaluate(bound);
      const auto packet=m::mpcc_rate_resolved_production_adapter::build(joined);
      row["stop_bundle_join"]=retained::to_string(joined.reason);
      row["stop_bundle_production"]=m::mpcc_rate_resolved_production_adapter::to_string(packet.reason);
      if(packet.authority) {
        row["stop_packet_acceleration"]=packet.authority->command.acceleration_mps2;
        row["stop_packet_steering"]=packet.authority->command.steering_tire_angle_rad;
        row["stop_packet_speed"]=packet.authority->command.predicted_speed_mps;
      }
    }
  }
  const auto command=m::mpcc_rate_resolved_production_adapter::build(normal);
  if(command.authority) {
    row["normal_first_acceleration"] = command.authority->command.acceleration_mps2;
    row["normal_first_steering"] = command.authority->command.steering_tire_angle_rad;
  }
  return row;
}
int main(int argc, char ** argv)
{
  if (argc != 3) return 2;
  const std::filesystem::path path=argv[1];const auto root=YAML::LoadFile(path.string());
  const auto load_request=[&](const char * key) {
    const auto n=root[key];auto r=mpcc_observation::read_request(n["request"],path.parent_path());
    r.plan=read_plan(n["certified_plan_evidence"],path);return r;
  };
  const auto previous=load_request("previous_accepted_revalidation_evidence");
  const auto current=load_request("revalidation_evidence");
  YAML::Node out;out["authority"]=false;out["solver_invocations"]=0;
  out["meaning"]="Exact baselines plus named counterfactuals; peer epoch projection preserves finite-acceleration remaining horizon; no future observations";
  out["previous_exact"]=describe(previous);out["current_exact"]=describe(current);
  auto crossed=current;
  crossed.obstacles=previous.obstacles;
  const auto dt=current.now_sec-previous.obstacles.observed_sec;
  for(auto & peer:crossed.obstacles.obstacles) {
    auto & c=peer.circle;const auto point=c.predicted_center(dt);
    const auto h=std::min(dt,c.acceleration_horizon_sec);
    c.x_m=point[0];c.y_m=point[1];c.velocity_x_mps+=c.acceleration_x_mps2*h;
    c.velocity_y_mps+=c.acceleration_y_mps2*h;c.acceleration_horizon_sec-=h;
  }
  crossed.obstacles.observed_sec=current.now_sec;
  out["current_previous_peer_advanced"]=describe(crossed);
  crossed=current;crossed.control_origin_speed_mps=previous.control_origin_speed_mps;
  out["current_previous_control_speed_only"]=describe(crossed);
  crossed=previous;crossed.control_origin_speed_mps=current.control_origin_speed_mps;
  out["previous_current_control_speed_only"]=describe(crossed);

  const auto p=root["source"]["semantic_request"]["observation_provenance"];
  const auto state=p["initial_x_y_yaw_u_vy_r_desired_tire"].as<std::vector<double>>();
  model::TimedState initial{p["pose_source_sec"].as<double>(),
    {state[0],state[1],state[2],state[3],state[4],state[5],state[6],state[7]}};
  std::vector<model::PublishedCommand> history;
  for(const auto & row:p["published_time_wire_acceleration_wire_steering"])
    history.push_back({row[0].as<double>(),row[1].as<double>(),row[2].as<double>()});
  const auto params=current.plan->execution_artifact->vehicle_model;
  const auto predict=[&](const auto & commands) {
    return model::predict_published_history(initial,current.now_sec,current.control_origin_sec,commands,params,
      p["nominal_acceleration_application_delay_sec"].as<double>(),p["nominal_steering_application_delay_sec"].as<double>());
  };
  const auto baseline=predict(history);if(!baseline)throw std::runtime_error("native provenance replay failed");
  out["native_provenance_control_speed_error"]=baseline->control_origin.forward_velocity_mps-current.control_origin_speed_mps;
  out["native_provenance_control_position_error"]=std::hypot(baseline->control_origin.x_m-current.control_pose.x_m,baseline->control_origin.y_m-current.control_pose.y_m);
  for(const double acceleration:{-3.0,1.0477030277252197}) {
    auto commands=history;commands.push_back({current.now_sec,acceleration,history.back().wire_steering_rad});
    const auto prediction=predict(commands);if(!prediction)throw std::runtime_error("prospective input replay failed");
    auto request=current;const auto & state=prediction->control_origin;
    request.control_pose={state.x_m,state.y_m,state.yaw_rad};
    request.current_speed_mps=prediction->current.forward_velocity_mps;
    request.control_origin_speed_mps=state.forward_velocity_mps;
    request.current_response_steering_rad=state.tire_steering_rad;
    request.current_lateral_velocity_mps=state.lateral_velocity_mps;
    request.current_yaw_rate_radps=state.yaw_rate_radps;
    request.measured_to_control_path.clear();request.measured_to_control_elapsed_sec.clear();
    for(const auto & item:prediction->current_to_control) {
      request.measured_to_control_path.push_back({item.state.x_m,item.state.y_m,item.state.yaw_rad});
      request.measured_to_control_elapsed_sec.push_back(item.source_sec-current.now_sec);
    }
    auto row=describe(request);row["prospective_wire_acceleration"]=acceleration;
    row["meaning"]="Hypothetical input at captured now, existing nominal delays/native model; prospective input is not claimed already published. Original progress reference anchor retained; native builder projects changed world pose.";
    out["prospective_input_prefix"].push_back(row);
  }
  YAML::Emitter emitter;emitter.SetDoublePrecision(17);emitter<<out;
  std::ofstream(argv[2])<<emitter.c_str()<<'\n';std::cout<<emitter.c_str()<<'\n';return 0;
}
