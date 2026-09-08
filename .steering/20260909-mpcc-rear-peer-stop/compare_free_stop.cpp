// Offline geometry hypothesis: the proof's rear peer also constrains the QP.
#include "multi_purpose_mpc_ros/mpcc_architecture_comparison.hpp"
#include "mpcc_architecture_comparison.cpp"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <limits>
int main(int argc,char **argv) {
  if(argc!=3)return 2;
  namespace m=multi_purpose_mpc_ros;
  namespace c=m::mpcc_architecture_comparison;
  namespace a=m::mpcc_architecture_snapshot;
  namespace d=m::mpcc_rate_resolved_dynamic_obstacle;
  std::string detail;
  auto record=a::load_recorded_interaction_snapshot(argv[1],&detail);
  if(!record)throw std::runtime_error(detail);
  YAML::Node report;
  report["source_fingerprint"]=record->interaction_fingerprint;
  auto source=record->source;
  const auto &world=*source.replay_world;
  if(world.obstacles.size()!=1)throw std::runtime_error("bounded single rear-peer comparison only");
  const auto &peer=world.obstacles.front();
  source.dynamic_obstacle_refinement_active=true;
  source.dynamic_obstacle_pass_side_sign=0;
  auto &context=source.identity.source_context;
  context.dynamic_obstacle_constraint_active=true;
  context.dynamic_obstacle_id=peer.id;
  context.dynamic_obstacle_generation=world.observation_generation;
  context.dynamic_obstacle_side_sign=0;
  context.fingerprint=m::mpcc_execution_contract::problem_context_fingerprint(context);
  double elapsed=source.control_prediction_origin_sec-world.observed_sec;
  for(std::size_t i=0;i<source.request.inputs.size();++i) {
    elapsed+=source.request.inputs[i].stage_dt_sec;
    const auto frame=m::mpc_stage_geometry::sample_course_frame(source.wall_course_frame_knots,
      source.course_progress_origin_m+source.wall_reference_progress_m[i+1]);
    if(!frame)throw std::runtime_error("immutable reference unavailable");
    const auto relative=m::mpcc_execution_contract::project_planar_pose_to_frenet(
      {peer.x_m+peer.velocity_x_mps*elapsed,peer.y_m+peer.velocity_y_mps*elapsed,frame->heading_rad},
      {frame->x_m,frame->y_m,frame->heading_rad});
    if(!relative)throw std::runtime_error("peer projection unavailable");
    source.dynamic_obstacle_stages.push_back({true,
      source.wall_reference_progress_m[i+1]+relative->lag_m,relative->lateral_m,
      world.physical_footprint.front_extent_m+world.physical_footprint.margin_m+peer.radius_m,
      world.physical_footprint.left_extent_m+world.physical_footprint.margin_m+peer.radius_m});
  }
  // Every comparison has a new semantic identity; the tactical front target
  // remains absent. The immutable Cartesian world drives the actual planes.
  for (int stop_mode : {2}) {
    const bool stop_intent=true;
    auto candidate=source;
    if(stop_mode==1) {
      m::mpcc_rate_resolved_shadow::SolverContext solver;
      const auto stop=m::mpcc_rate_resolved_stop_control_lattice::build_current_world_maximum_braking_candidate(
        source,solver.physical_constraint_tolerance());
      if(!stop.accepted())throw std::runtime_error(stop.detail);
      candidate=stop.candidate;
    }
    if(stop_mode==2) {
      // Architectural hypothesis: solve the full physical controls through
      // rest, instead of prescribing maximum braking at every future stage.
      // Source physical speed/acceleration/steering/geometry constraints and
      // horizon stay fixed; all peers and rest are still mandatory proofs.
      candidate.execution_prefix_steps=candidate.request.horizon_steps;
      auto &terminal=candidate.request.states.back();
      terminal.reference[m::mpcc_rate_resolved::kVelocityIndex]=0.0;
      terminal.lower[m::mpcc_rate_resolved::kVelocityIndex]=0.0;
      terminal.upper[m::mpcc_rate_resolved::kVelocityIndex]=0.0;
    }
    for (auto topology : {d::LongitudinalTopology::Automatic,d::LongitudinalTopology::StayAhead}) {
      candidate.dynamic_obstacle_longitudinal_topology=topology;
      for(std::size_t sqp_depth : {0U,3U}) {
        const auto fp=a::fingerprint_interaction_snapshot(candidate);
        if(!fp)throw std::runtime_error("candidate not complete");
        const auto arm=c::evaluate_arm(c::Arm::PersistentA,candidate,record->interaction_fingerprint,fp,
          c::resolve_audit_terminal_successor(candidate),-1,-1,nullptr,false,std::nullopt,sqp_depth,
          stop_intent ? c::TerminalStopLateralAuditMode::SolvedStopTrajectory : c::TerminalStopLateralAuditMode::NormalPathProfile, nullptr, true);
        YAML::Node row;
        row["stop"]=stop_intent;row["mode"]="free-controls-through-rest";row["terminal_speed_mps"]=arm.terminal_velocity_mps;row["acceleration_controls"]=arm.solved_acceleration_mps2;row["steering_rate_controls"]=arm.solved_steering_rate_radps;row["control_durations"]=arm.solved_control_duration_sec;row["topology"]=d::to_string(topology);row["sqp_depth"]=sqp_depth;
        row["stage"]=c::to_string(arm.stage);row["solver"]=m::mpcc_rate_resolved_shadow::to_string(arm.solver_outcome);
        row["bundle"]=arm.bundle.has_value();row["candidate_fingerprint"]=fp;row["detail"]=arm.detail;
        row["authority"]=false;report["arms"].push_back(row);std::cout<<row<<'\n';
      }
    }
  }
  YAML::Emitter e;e.SetDoublePrecision(std::numeric_limits<double>::max_digits10);
  e<<report;std::ofstream(argv[2])<<e.c_str()<<'\n';
}
