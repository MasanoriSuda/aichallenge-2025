#define main preserved_certified_plan_driver_main
#include "../replay_certified_plan.cpp"
#undef main
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_applied_program.hpp"
#include "multi_purpose_mpc_ros/detail/mpcc_footprint_enclosure.hpp"
#include <chrono>
#include <random>

namespace vehicle = m::mpcc_vehicle_model;
namespace num = vehicle::numerical;
using Clock = std::chrono::steady_clock;
std::vector<num::I> packet_groups(const vehicle::ObservationProvenance & observation,
  const vehicle::PublishedInputProgram & program, const vehicle::InputApplicationProfile & profile,
  double begin, double end)
{
  std::array<std::optional<num::I>, 3> groups;
  const auto add = [&](double value) {
    const int index = value < 0 ? 0 : value > 0 ? 2 : 1;
    groups[index] = groups[index] ? num::hull(*groups[index], num::I(value)) : num::I(value);
  };
  const double first = num::down(num::down(begin) - profile.acceleration_age_sec);
  const double last = num::up(end);
  for (const auto * packets : {&observation.commands, &program.commands})
    for (const auto & packet : *packets)
      if (packet.published_sec >= first && packet.published_sec <= last) add(packet.wire_acceleration_mps2);
  if (program.repeat_last_until_rest &&
    program.commands.back().published_sec + program.publication_interval_sec <= last)
    add(program.commands.back().wire_acceleration_mps2);
  std::vector<num::I> result;
  for (const auto & group : groups) if (group) result.push_back(*group);
  if (result.empty()) throw std::runtime_error("missing causal packet group");
  return result;
}
std::vector<num::Box> merge_modes(const std::vector<num::Box> & source, const vehicle::Parameters & p)
{
  std::array<std::optional<num::Box>, 5> bins;
  const std::array<double, 5> limits{-INFINITY, -p.sleep_speed_mps, 0, p.sleep_speed_mps, INFINITY};
  const auto merge = [&](size_t index, const num::Box & box) {
    if (!bins[index]) bins[index] = box;
    else for (size_t j=0; j<8; ++j) (*bins[index])[j]=num::hull((*bins[index])[j],box[j]);
  };
  for (const auto & box : source) {
    if (num::at_rest(box)) {merge(0,box);continue;}
    for (size_t j=0;j<4;++j) {
      auto part=box;part[3]={std::max(box[3].lo,limits[j]),std::min(box[3].hi,limits[j+1])};
      if(part[3].lo<=part[3].hi) merge(j+1,part);
    }
  }
  std::vector<num::Box> result;for(const auto & bin:bins)if(bin)result.push_back(*bin);return result;
}
int main(int argc,char **argv)
{
  if(argc!=3)return 2;
  const std::filesystem::path path=argv[1];const auto root=YAML::LoadFile(path.string());
  const auto e=root["revalidation_evidence"];
  auto request=mpcc_observation::read_request(e["request"],path.parent_path());
  request.plan=read_plan(e["certified_plan_evidence"],path);
  auto nominal_request=request;nominal_request.applied_program_required=false;nominal_request.input_application_profile.reset();
  const auto nominal=retained::evaluate(nominal_request);
  if(!nominal.proof)throw std::runtime_error("nominal source proof unavailable");
  const auto & execution=*request.plan->execution_artifact;
  const auto & model=execution.vehicle_model;
  const auto & observation=request.publication_prefix->observation;
  const auto & profile=*request.input_application_profile;
  const auto prepared=m::mpcc_stop_input_program::prepare({execution.identity,request.decision_id,
    request.control_origin_sec,execution.publication_interval_sec,request.publication_prefix->proposed_packet,
    model.steering_wire_gain,request.minimum_acceleration_mps2,request.maximum_acceleration_mps2,
    execution.maximum_abs_steering_rad,execution.maximum_abs_steering_rate_radps,
    execution.physical_global_tolerance,nominal.proof->terminal_stop_actuation_samples});
  if(!prepared.prepared)throw std::runtime_error("common program unavailable");
  const auto & program=prepared.prepared->program;
  const auto prediction=vehicle::predict_applied_inputs_to_rest(observation,program,profile,model);
  if(!prediction.tube)throw std::runtime_error("original tube unavailable");
  const auto & original=*prediction.tube;
  auto initial=observation.initial.state;initial.x_m=initial.y_m=initial.yaw_rad=0;
  std::vector<num::Box> population{num::point(initial)};
  std::vector<vehicle::State> oracles(512,initial);
  std::mt19937_64 generator(20260911);std::uniform_real_distribution<double> unit(0,1);
  double numerical_ms=0,minimum_u=INFINITY,native_minimum_u=INFINITY,first_state_reject=NAN,minimum_peer=INFINITY;
  size_t samples=0,scalar_checks=0,max_partitions=0;bool walls_clear=true,peers_clear=true;
  const auto wall_footprint=physical::resolve_clearance_footprint(request.current_footprint,
    request.plan->physical_snapshot->hard_wall_clearance_m);
  if(!wall_footprint)throw std::runtime_error("invalid original footprint");
  const auto to_world=[&](const m::recovery_footprint::Pose2D & p) {
    const auto & o=observation.initial.state;const double c=std::cos(o.yaw_rad),s=std::sin(o.yaw_rad);
    return m::recovery_footprint::Pose2D{o.x_m+c*p.x_m-s*p.y_m,o.y_m+s*p.x_m+c*p.y_m,o.yaw_rad+p.yaw_rad};
  };
  YAML::Node output;output["authority"]=false;output["decision"]=request.decision_id;
  output["source"]=execution.identity.sequence;output["original_rest_sec"]=original.rest_sec;
  output["physical_global_tolerance"]=execution.physical_global_tolerance;
  output["observation"]=e["request"]["prospective_publication"]["observation"];
  for (const auto & packet:program.commands) {
    YAML::Node row;row.push_back(packet.published_sec);row.push_back(packet.wire_acceleration_mps2);row.push_back(packet.wire_steering_rad);
    output["program_commands"].push_back(row);
  }
  for (const auto & sample:original.source_to_rest) {
    if(sample.begin_sec>=observation.now_sec && sample.swept_body[3].lower < -execution.physical_global_tolerance) {
      output["original_first_u_reject_sec"]=sample.begin_sec;
      for(const auto & value:sample.swept_body) {YAML::Node row;row.push_back(value.lower);row.push_back(value.upper);output["original_rejected_swept_body"].push_back(row);}
      break;
    }
  }
  for(const auto & sample:original.source_to_rest) {
    const auto started=Clock::now();
    const auto groups=packet_groups(observation,program,profile,sample.begin_sec,sample.end_sec);
    const num::I desired=num::I(sample.inputs.wire_steering_rad.lower,sample.inputs.wire_steering_rad.upper)/model.steering_wire_gain;
    for(auto & box:population)box[6]=desired;
    num::Box swept=num::joined(population);std::vector<num::Box> next;
    for(const auto acceleration:groups) {
      num::Box arm_swept;
      auto arm=num::advance_partitioned(population,acceleration,model,sample.duration_sec,&arm_swept);
      for(size_t i=0;i<8;++i)swept[i]=num::hull(swept[i],arm_swept[i]);
      next.insert(next.end(),arm.begin(),arm.end());
    }
    population=merge_modes(next,model);const auto endpoint=num::joined(population);
    max_partitions=std::max(max_partitions,population.size());
    numerical_ms+=std::chrono::duration<double,std::milli>(Clock::now()-started).count();
    for(size_t arm=0;arm<oracles.size();++arm) {
      const auto acceleration=groups[arm%groups.size()];
      const double af=arm<4?double(arm%2):unit(generator), sf=arm<4?double(arm/2):unit(generator);
      auto & state=oracles[arm];state.desired_steering_rad=(sample.inputs.wire_steering_rad.lower+
        sf*(sample.inputs.wire_steering_rad.upper-sample.inputs.wire_steering_rad.lower))/model.steering_wire_gain;
      const auto moved=vehicle::advance(state,{acceleration.lo+af*(acceleration.hi-acceleration.lo),0},model,sample.duration_sec);
      if(!moved)throw std::runtime_error("native input arm unavailable");state=moved->state;
      const auto values=num::values(state);
      for(size_t j=0;j<8;++j) {
        if(values[j]<endpoint[j].lo||values[j]>endpoint[j].hi)throw std::runtime_error("packet-group native oracle outside enclosure");
        ++scalar_checks;
      }
      if(sample.begin_sec>=observation.now_sec)native_minimum_u=std::min(native_minimum_u,state.forward_velocity_mps);
    }
    if(sample.begin_sec<observation.now_sec)continue;
    ++samples;minimum_u=std::min(minimum_u,swept[3].lo);
    const double tolerance=std::max(1e-9,execution.physical_global_tolerance);
    const bool state_ok=swept[3].lo>=-tolerance&&
      std::max(std::abs(swept[6].lo),std::abs(swept[6].hi))<=execution.maximum_abs_steering_rad+tolerance&&
      std::max(std::abs(swept[7].lo),std::abs(swept[7].hi))<=model.maximum_wire_steering_rad*model.tire_grip+tolerance;
    if(!state_ok&&!std::isfinite(first_state_reject))first_state_reject=sample.begin_sec;
    const auto wall=num::footprint(swept,*wall_footprint);
    const auto cells=m::recovery_footprint::sample_footprint(*request.current_wall_grid,wall.extents,to_world(wall.pose));
    walls_clear=walls_clear&&cells.valid&&!cells.out_of_map&&cells.contact_cells.empty();
    const auto ego=num::footprint(swept,request.current_footprint);
    for(const auto & obstacle:request.obstacles.obstacles) {
      auto circle=obstacle.circle;const double t0=sample.begin_sec-request.obstacles.observed_sec,t1=sample.end_sec-request.obstacles.observed_sec;
      circle.radius_m=num::up(circle.radius_m+num::up(circle.maximum_speed(t0,t1)*num::up((sample.end_sec-sample.begin_sec)/2)));
      const auto clearance=m::recovery_footprint::circle_obstacle_clearance_at_time(ego.extents,to_world(ego.pose),circle,(t0+t1)/2);
      if(!clearance)throw std::runtime_error("invalid peer");minimum_peer=std::min(minimum_peer,*clearance);peers_clear=peers_clear&&*clearance>=0;
    }
  }
  output["meaning"]="Diagnostic numerical representation comparison only. Same source/program/model/profile/wall/full peers; split hulls of actual causal acceleration packet values by sign, retain steering interval. Follow not claimed.";
  output["state_bounds_clear"]=!std::isfinite(first_state_reject);output["first_state_reject_sec"]=first_state_reject;
  output["minimum_u_lower_mps"]=minimum_u;output["native_minimum_u_mps"]=native_minimum_u;
  output["wall_clear"]=walls_clear;output["peer_clear"]=peers_clear;output["minimum_peer_clearance_m"]=minimum_peer;
  output["full_rest"]=num::at_rest(num::joined(population));output["numerical_ms"]=numerical_ms;
  output["samples"]=samples;output["maximum_partitions"]=max_partitions;output["native_scalar_checks"]=scalar_checks;
  YAML::Emitter emitter;emitter.SetDoublePrecision(17);emitter<<output;std::ofstream(argv[2])<<emitter.c_str()<<'\n';
}
