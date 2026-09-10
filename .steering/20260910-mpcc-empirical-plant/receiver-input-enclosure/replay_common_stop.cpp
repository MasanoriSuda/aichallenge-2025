#define main preserved_certified_plan_driver_main
#include "../replay_certified_plan.cpp"
#undef main
#include "footprint_enclosure.hpp"
#include <chrono>
#include <random>

namespace vehicle=m::mpcc_vehicle_model;
namespace rec=m::recovery_footprint;
using Clock=std::chrono::steady_clock;

std::vector<vehicle::PublishedCommand> nominal_stop_commands(retained::Request r,const YAML::Node&root){
 const auto&p=r.plan->execution_artifact->vehicle_model;
 const auto prefix=vehicle::predict_prospective_publication(r.publication_prefix->observation,
   {r.now_sec,r.minimum_acceleration_mps2,static_cast<float>(r.previous_published_steering_rad*p.steering_wire_gain)},p);
 if(!prefix)throw std::runtime_error("nominal Stop prefix unavailable");
 const auto&s=prefix->control_origin;bool projected=false;
 for(const auto&knot:knots(root["source"]["wall_course_frame_knots"])){
   const double original=knot.progress_m+std::cos(knot.heading_rad)*(r.control_pose.x_m-knot.x_m)+std::sin(knot.heading_rad)*(r.control_pose.y_m-knot.y_m);
   if(std::abs(original-r.control_origin_physical_progress_m)>1e-9)continue;
   if(projected)throw std::runtime_error("ambiguous nominal projection");projected=true;
   r.control_origin_physical_progress_m=knot.progress_m+std::cos(knot.heading_rad)*(s.x_m-knot.x_m)+std::sin(knot.heading_rad)*(s.y_m-knot.y_m);
   break;
 }
 if(!projected)throw std::runtime_error("nominal projection unavailable");
 r.publication_prefix=prefix;r.control_pose={s.x_m,s.y_m,s.yaw_rad};
 r.current_speed_mps=prefix->current.forward_velocity_mps;r.control_origin_speed_mps=s.forward_velocity_mps;
 r.current_time_steering_rad=prefix->current.tire_steering_rad;r.current_steering_rad=s.desired_steering_rad;
 r.current_response_steering_rad=s.tire_steering_rad;r.current_lateral_velocity_mps=s.lateral_velocity_mps;r.current_yaw_rate_radps=s.yaw_rate_radps;
 r.measured_to_control_path.clear();r.measured_to_control_elapsed_sec.clear();
 for(const auto&sample:prefix->current_to_control){r.measured_to_control_path.push_back({sample.state.x_m,sample.state.y_m,sample.state.yaw_rad});r.measured_to_control_elapsed_sec.push_back(sample.source_sec-r.now_sec);}
 const auto stop=retained::evaluate_stop_successor(r);
 if(stop.actuation_samples.empty())return {};
 std::vector<vehicle::PublishedCommand> commands{prefix->proposed_packet};size_t previous=0;
 for(const auto&sample:stop.actuation_samples){
   if(sample.command_interval_index==previous)continue;
   previous=sample.command_interval_index;
   commands.push_back({r.now_sec+previous*r.plan->execution_artifact->publication_interval_sec,
     static_cast<float>(r.minimum_acceleration_mps2),static_cast<float>(sample.end_steering_rad*p.steering_wire_gain)});
 }
 return commands;
}

YAML::Node common_stop(const retained::Request&r,const vehicle::Parameters&p,
 double hard_wall_clearance,double pending_horizon,
 const std::vector<vehicle::PublishedCommand>&future){
 const auto begin=Clock::now();
 if(!r.publication_prefix)throw std::runtime_error("missing original observation");
 const auto & observation=r.publication_prefix->observation;
 const auto prediction=vehicle::predict_prospective_publication(observation,
   {r.now_sec,r.minimum_acceleration_mps2,
    static_cast<float>(r.previous_published_steering_rad*p.steering_wire_gain)},p);
 if(!prediction)throw std::runtime_error("missing modeled current body");
 const double upper=p.maximum_wire_acceleration_mps2;
 // The enclosing axes are fixed to the initial heading. World-axis boxes
 // destroy forward/lateral correlation merely when the same scene rotates.
 const auto origin=prediction->current;auto initial=origin;
 initial.x_m=0;initial.y_m=0;initial.yaw_rad=0;
 auto state=enclosure::point(initial);
 std::vector<enclosure::Box> population{state};size_t maximum_population=1;
 const auto to_world=[&](const rec::Pose2D&local){return rec::Pose2D{
   origin.x_m+std::cos(origin.yaw_rad)*local.x_m-std::sin(origin.yaw_rad)*local.y_m,
   origin.y_m+std::sin(origin.yaw_rad)*local.x_m+std::cos(origin.yaw_rad)*local.y_m,
   origin.yaw_rad+local.yaw_rad};};
 const auto wall_footprint=m::mpcc_rate_resolved_physical_wall::resolve_clearance_footprint(
   r.current_footprint,hard_wall_clearance);
 if(!wall_footprint || !r.current_wall_grid)throw std::runtime_error("wall context unavailable");
 YAML::Node row;row["pending_horizon_sec"]=pending_horizon;row["input_lower"]=r.minimum_acceleration_mps2;row["input_upper"]=upper;
 row["wire_steering_rad"]=prediction->proposed_packet.wire_steering_rad;
 row["steering_policy"]=future.empty()?"common-held":"common-precomputed-path-tracking";
 row["initial_speed_mps"]=prediction->current.forward_velocity_mps;
 bool wall_clear=true,peer_clear=true,rest=false;double minimum=INFINITY,first_wall=NAN,first_peer=NAN,enclosure_ms=0,proof_ms=0;
 std::vector<vehicle::State> samples(43,initial);std::vector<double> sample_clearances(samples.size(),INFINITY);
 std::mt19937 random(1031);std::uniform_real_distribution<double> unit(0,1);size_t contained=0,steps=0;
 const double peer_offset=r.now_sec-r.obstacles.observed_sec;
 if(peer_offset<0)throw std::runtime_error("future peer observation");
 const auto check=[&](const enclosure::Box&box,double t0,double t1){
   const auto start=Clock::now();const auto wall=enclosure::footprint(box,*wall_footprint);
   const auto cells=rec::sample_footprint(*r.current_wall_grid,wall.extents,to_world(wall.pose));
   if(!cells.valid || cells.out_of_map || !cells.contact_cells.empty()){
     wall_clear=false;if(!std::isfinite(first_wall))first_wall=t0;
   }
   const auto ego=enclosure::footprint(box,r.current_footprint);
   for(const auto&o:r.obstacles.obstacles){
     auto circle=o.circle;
     // The peer's complete existing CA/CV center path over this interval is
     // inside a disk about its midpoint, using its analytic maximum speed.
     const double speed=circle.maximum_speed(peer_offset+t0,peer_offset+t1);
     circle.radius_m=enclosure::up(circle.radius_m+enclosure::up(speed*enclosure::up((t1-t0)/2)));
     const auto clearance=rec::circle_obstacle_clearance_at_time(ego.extents,to_world(ego.pose),circle,peer_offset+(t0+t1)/2);
     if(!clearance)throw std::runtime_error("invalid peer proof");
     minimum=std::min(minimum,*clearance);
     if(*clearance<0){peer_clear=false;if(!std::isfinite(first_peer))first_peer=t0;}
   }
   proof_ms+=std::chrono::duration<double,std::milli>(Clock::now()-start).count();
 };
 check(state,0,0);
 double elapsed=0;
 for(size_t k=0;k<1200;++k){
   double end=std::min(6.,elapsed+.005);
   // Preserve all separate steering-history and hypothesized input boundaries.
   for(const auto&packet:observation.commands){const double t=packet.published_sec+observation.steering_delay_sec-r.now_sec;if(t>elapsed+1e-12 && t<end)end=t;}
   for(const auto&packet:future){const double t=packet.published_sec+observation.steering_delay_sec-r.now_sec;if(t>elapsed+1e-12 && t<end)end=t;}
   for(const double t:{observation.steering_delay_sec,pending_horizon})if(t>elapsed+1e-12 && t<end)end=t;
   const double dt=end-elapsed;if(dt<=0)throw std::runtime_error("invalid interval");
   double steering=observation.commands.front().wire_steering_rad;
   for(const auto&packet:observation.commands)if(packet.published_sec+observation.steering_delay_sec<=r.now_sec+elapsed+1e-12)steering=packet.wire_steering_rad;
   if(elapsed+1e-12>=observation.steering_delay_sec)steering=prediction->proposed_packet.wire_steering_rad;
   for(const auto&packet:future)if(packet.published_sec+observation.steering_delay_sec<=r.now_sec+elapsed+1e-12)steering=packet.wire_steering_rad;
   state[6]=enclosure::I(steering/p.steering_wire_gain);
   for(auto&member:population)member[6]=state[6];
   const auto wire=elapsed+1e-12<pending_horizon?enclosure::I(r.minimum_acceleration_mps2,upper):enclosure::I(r.minimum_acceleration_mps2);
   const auto started=Clock::now();enclosure::Box swept;
   population=enclosure::advance_partitioned(std::move(population),wire,p,dt,&swept);
   maximum_population=std::max(maximum_population,population.size());const auto next=enclosure::joined(population);
   enclosure_ms+=std::chrono::duration<double,std::milli>(Clock::now()-started).count();
   const auto subdivisions=vehicle::integration_steps(dt,p.maximum_step_sec);
   if(subdivisions>1 && row["additional_subdivisions"].size()<10){YAML::Node item;item["elapsed"]=elapsed;item["duration"]=dt;item["count"]=subdivisions;row["additional_subdivisions"].push_back(item);}
   check(swept,elapsed,end);
   for(size_t j=0;j<samples.size();++j){
     auto&s=samples[j];s.desired_steering_rad=steering/p.steering_wire_gain;
     double acceleration=r.minimum_acceleration_mps2;
     if(elapsed+1e-12<pending_horizon){
       if(j<=40)acceleration=elapsed+1e-12<(pending_horizon*j/40)?upper:r.minimum_acceleration_mps2;
       else acceleration=wire.lo+(wire.hi-wire.lo)*(j==41?unit(random):static_cast<double>(k%2));
     }
     const auto predicted=vehicle::advance(s,{acceleration,0},p,dt);if(!predicted)throw std::runtime_error("native sample failed");s=predicted->state;
     const auto values=enclosure::values(s);for(size_t i=0;i<8;++i){if(values[i]<next[i].lo || values[i]>next[i].hi)throw std::runtime_error("native sample outside body enclosure");++contained;}
     for(const auto&o:r.obstacles.obstacles){const auto clearance=rec::circle_obstacle_clearance_at_time(r.current_footprint,to_world({s.x_m,s.y_m,s.yaw_rad}),o.circle,peer_offset+end);if(!clearance)throw std::runtime_error("invalid native endpoint clearance");sample_clearances[j]=std::min(sample_clearances[j],*clearance);}
   }
   state=next;elapsed=end;++steps;
   if(state[3].lo==0 && state[3].hi==0 && state[4].lo==0 && state[4].hi==0 && state[5].lo==0 && state[5].hi==0 && elapsed>=pending_horizon){rest=true;break;}
 }
 row["rest"]=rest;row["duration_sec"]=elapsed;row["steps"]=steps;row["wall_clear"]=wall_clear;row["peer_clear"]=peer_clear;row["minimum_peer_clearance_m"]=minimum;
 row["first_wall_reject_sec"]=first_wall;row["first_peer_reject_sec"]=first_peer;row["numerical_envelope_clear_to_rest"]=rest&&wall_clear&&peer_clear;
 row["enclosure_ms"]=enclosure_ms;row["physical_proof_ms"]=proof_ms;row["elapsed_ms_including_oracles"]=std::chrono::duration<double,std::milli>(Clock::now()-begin).count();row["contained_scalar_samples"]=contained;
 row["native_endpoint_clearances_observation_only"]=sample_clearances;
 row["interval_frame"]="fixed initial heading";row["input_upper_source"]="native plant wire limit";
 row["hybrid_mode_partition"]=true;row["maximum_body_boxes"]=maximum_population;
 return row;
}

int main(int argc,char**argv){
 if(argc!=3 && argc!=4)return 2;const std::filesystem::path path=argv[1];const auto root=YAML::LoadFile(path.string());
 YAML::Node out;out["authority"]=false;out["meaning"]="Diagnostic numerical enclosure of one common float32 held-steering/brake policy, all native-map input values in the declared interval before the hypothetical horizon, then common brake to full nominal rest. Modeled current body is a point, separate historical steering delay preserved. Fixed initial-heading coordinates preserve translation correlation. Endpoint-box interpolation and full peer midpoint motion enclosure retain existing footprint/margins. This is not a receiver bound, model-error bound, publication certificate, or full real-arithmetic proof.";
 if(argc==4){
   if(std::string(argv[3])!="--source-world")return 2;
   std::string detail;const auto recorded=snap::load_recorded_interaction_snapshot(path,&detail);
   if(!recorded)throw std::runtime_error("source world unavailable: "+detail);
   const auto&source=recorded->source;const auto&world=source.replay_world.value();
   const auto&observation=source.request.observation_provenance.value();const auto&p=source.request.vehicle_model;
   if(!world.terminal_stop_contract_available)throw std::runtime_error("source Stop contract unavailable");
   // This Request carries physical input fields only. It has no CertifiedPlan
   // and is never passed to a normal authority or certificate constructor.
   retained::Request r;r.decision_id=source.identity.source_context.decision_id;r.now_sec=observation.now_sec;
   r.minimum_acceleration_mps2=world.terminal_stop_minimum_acceleration_mps2;
   r.previous_published_steering_rad=observation.commands.back().wire_steering_rad/p.steering_wire_gain;
   r.publication_prefix=vehicle::predict_prospective_publication(observation,
     {r.now_sec,r.minimum_acceleration_mps2,observation.commands.back().wire_steering_rad},p);
   r.current_footprint=world.physical_footprint;r.current_wall_grid=source.wall_grid;
   r.obstacles.generation=world.observation_generation;r.obstacles.observed_sec=world.observed_sec;r.obstacles.current=world.current;
   for(const auto&o:world.obstacles)r.obstacles.obstacles.push_back({o.id,o.circle()});
   out["source_world"]["source_sequence"]=source.identity.sequence;
   out["source_world"]["decision"]=r.decision_id;out["source_world"]["now_sec"]=r.now_sec;
   out["source_world"]["interaction_fingerprint"]=recorded->interaction_fingerprint;
   out["source_world"]["certified_plan_present"]=false;
   out["source_world"]["hypotheses"].push_back(common_stop(r,p,world.hard_wall_clearance_m,.25,{}));
 }
 else {
 for(const auto key:{"previous_accepted_revalidation_evidence","revalidation_evidence"}){
   const auto e=root[key];if(!e || !e["request"] || !e["certified_plan_evidence"])continue;
   auto r=mpcc_observation::read_request(e["request"],path.parent_path());r.plan=read_plan(e["certified_plan_evidence"],path);
   out[key]["decision"]=r.decision_id;out[key]["capture_status"]=e["status"];
   if(!r.publication_prefix){out[key]["status"]="unavailable: original observation not captured";continue;}
   const auto future=nominal_stop_commands(r,root);
   out[key]["nominal_steering_command_count"]=future.size();
   for(const auto&packet:future){YAML::Node item;item["published_sec"]=packet.published_sec;item["wire_acceleration_mps2"]=packet.wire_acceleration_mps2;item["wire_steering_rad"]=packet.wire_steering_rad;out[key]["common_precomputed_commands"].push_back(item);}
   for(const double h:{.10,.15,.20,.25}){
     const auto&p=r.plan->execution_artifact->vehicle_model;const double margin=r.plan->physical_snapshot->hard_wall_clearance_m;
     out[key]["hypotheses"].push_back(common_stop(r,p,margin,h,{}));
     if(!future.empty())out[key]["hypotheses"].push_back(common_stop(r,p,margin,h,future));
   }
 }
 }
 YAML::Emitter e;e.SetDoublePrecision(17);e<<out;std::ofstream(argv[2])<<e.c_str()<<'\n';std::cout<<e.c_str()<<'\n';
}
