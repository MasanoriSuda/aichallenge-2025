// Offline geometry hypothesis: the proof's rear peer also constrains the QP.
#include "multi_purpose_mpc_ros/mpcc_architecture_comparison.hpp"
#include "comparison_capture.cpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_retained_revalidation.hpp"
#include "artifact_capture.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_production_adapter.hpp"
std::shared_ptr<const multi_purpose_mpc_ros::mpcc_rate_resolved_execution_artifact::ExecutionArtifact> captured_execution;
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
  const auto & world=*record->source.replay_world;
  if(world.obstacles.size()!=1)throw std::runtime_error("bounded one-peer timing comparison only");
  const auto & peer=world.obstacles.front();
  for(int clock_mode : {0,1}) for(int stop_mode : {1,2}) {
    const bool stop_intent=true;
    auto candidate=record->source;
    double time_horizon=candidate.request.horizon_steps*candidate.request.maximum_stage_dt_sec;
    if(clock_mode==0) {
      const double distance=candidate.wall_course_frame_knots.back().progress_m-candidate.course_progress_origin_m;
      time_horizon=std::min(time_horizon,2.0*distance/candidate.request.initial_state[3]);
    }
    const double stage_dt=time_horizon/candidate.request.horizon_steps;
    if(stage_dt<candidate.request.minimum_stage_dt_sec || stage_dt>candidate.request.maximum_stage_dt_sec)
      throw std::runtime_error("timing outside source physical dt domain");
    for(auto & input:candidate.request.inputs) input.stage_dt_sec=stage_dt;
    candidate.execution_prefix_steps=candidate.request.horizon_steps;
    if(stop_mode==1) {
      m::mpcc_rate_resolved_shadow::SolverContext solver;
      const auto stop=m::mpcc_rate_resolved_stop_control_lattice::build_current_world_maximum_braking_candidate(
        candidate,solver.physical_constraint_tolerance());
      if(!stop.accepted())throw std::runtime_error(stop.detail);
      candidate=stop.candidate;
    } else {
      auto &terminal=candidate.request.states.back();
      terminal.reference[m::mpcc_rate_resolved::kVelocityIndex]=0.0;
      terminal.lower[m::mpcc_rate_resolved::kVelocityIndex]=0.0;
      terminal.upper[m::mpcc_rate_resolved::kVelocityIndex]=0.0;
    }
    // All observed physical peers in this scene (one), using the new stage
    // clock and the original observation origin. No future observations.
    candidate.dynamic_obstacle_refinement_active=true;
    candidate.dynamic_obstacle_pass_side_sign=0;
    candidate.dynamic_obstacle_stages.clear();
    auto &context=candidate.identity.source_context;
    context.dynamic_obstacle_constraint_active=true;
    context.dynamic_obstacle_id=peer.id;
    context.dynamic_obstacle_generation=world.observation_generation;
    context.dynamic_obstacle_side_sign=0;
    double elapsed=candidate.control_prediction_origin_sec-world.observed_sec;
    for(std::size_t i=0;i<candidate.request.inputs.size();++i) {
      elapsed+=candidate.request.inputs[i].stage_dt_sec;
      const auto frame=m::mpc_stage_geometry::sample_course_frame(candidate.wall_course_frame_knots,
        candidate.course_progress_origin_m+candidate.wall_reference_progress_m[i+1]);
      if(!frame)throw std::runtime_error("immutable reference unavailable");
      const auto relative=m::mpcc_execution_contract::project_planar_pose_to_frenet(
        {peer.x_m+peer.velocity_x_mps*elapsed,peer.y_m+peer.velocity_y_mps*elapsed,frame->heading_rad},
        {frame->x_m,frame->y_m,frame->heading_rad});
      if(!relative)throw std::runtime_error("peer projection unavailable");
      candidate.dynamic_obstacle_stages.push_back({true,
        candidate.wall_reference_progress_m[i+1]+relative->lag_m,relative->lateral_m,
        world.physical_footprint.front_extent_m+world.physical_footprint.margin_m+peer.radius_m,
        world.physical_footprint.left_extent_m+world.physical_footprint.margin_m+peer.radius_m});
    }
    // Explicit terminal meaning plus exact clock in context, distinct from
    // original normal request. This is still observation-only; no async use.
    context.bounds_schema_id+=stop_mode==1 ? "/offline-maximum-law-rest-v1" : "/offline-free-rest-v1";
    std::ostringstream clock_identity;clock_identity<<std::hexfloat<<stage_dt;
    context.cost_schema_id+="/offline-uniform-clock/"+clock_identity.str();
    context.fingerprint=m::mpcc_execution_contract::problem_context_fingerprint(context);
    for (auto topology : {d::LongitudinalTopology::Automatic}) {
      candidate.dynamic_obstacle_longitudinal_topology=topology;
      for(std::size_t sqp_depth : {0U,3U}) {
        const auto arm_id=std::to_string(clock_mode)+"-"+std::to_string(stop_mode)+"-"+std::to_string(sqp_depth);
        captured_execution.reset();
        const auto fp=a::fingerprint_interaction_snapshot(candidate);
        if(!fp)throw std::runtime_error("candidate not complete");
        const auto arm=c::evaluate_arm(c::Arm::PersistentA,candidate,record->interaction_fingerprint,fp,
          c::resolve_audit_terminal_successor(candidate),-1,-1,nullptr,false,std::nullopt,sqp_depth,
          stop_intent ? c::TerminalStopLateralAuditMode::SolvedStopTrajectory : c::TerminalStopLateralAuditMode::NormalPathProfile, nullptr, true);
        YAML::Node row;
        row["stop"]=stop_intent;row["mode"]=stop_mode==1 ? "maximum-braking-law" : "free-controls-through-rest";row["clock_mode"]=clock_mode==0 ? "geometry-constant-deceleration" : "source-maximum-stage-time";row["horizon_sec"]=time_horizon;row["arm_id"]=arm_id;row["terminal_speed_mps"]=arm.terminal_velocity_mps;row["acceleration_controls"]=arm.solved_acceleration_mps2;row["steering_rate_controls"]=arm.solved_steering_rate_radps;row["control_durations"]=arm.solved_control_duration_sec;row["topology"]=d::to_string(topology);row["sqp_depth"]=sqp_depth;
        row["stage"]=c::to_string(arm.stage);row["solver"]=m::mpcc_rate_resolved_shadow::to_string(arm.solver_outcome);
        row["bundle"]=arm.bundle.has_value();row["candidate_fingerprint"]=fp;row["detail"]=arm.detail;

        if (arm.bundle && captured_execution) {
          const auto & execution=*captured_execution;
          capture_execution(execution,(arm_id+"-artifact.yaml").c_str());
          const auto captured=a::record_proof_failure(candidate,a::PipelineStage::PhysicalProof,"offline-candidate-observation","Source capture only; actual proof outcome is in report.yaml. No publication.","source-"+arm_id);
          row["source_capture"]=captured.snapshot_file.string();
          const auto physical=c::wall_snapshot(candidate,*candidate.replay_world,arm.bundle->exact_trajectory);
          const auto wall=m::mpcc_rate_resolved_physical_wall::evaluate(physical);
          const auto plan=m::mpcc_rate_resolved_certified_plan::build(captured_execution,physical,wall,std::make_shared<const m::mpcc_rate_resolved_shadow::Snapshot>(candidate));
          if(!plan.plan)throw std::runtime_error("candidate plan construction rejected");
          namespace retained=m::mpcc_rate_resolved_retained_revalidation;
          retained::Request request;
          request.plan=plan.plan;
          request.decision_id=candidate.identity.source_context.decision_id;
          request.now_sec=candidate.identity.snapshot_sec;
          request.control_origin_sec=candidate.control_prediction_origin_sec;
          request.execution_clock={retained::ExecutionClockKind::TimeAlignedCandidate,0.0,0.0};
          request.current_intent=candidate.identity.source_context.intent;
          request.control_origin_physical_progress_m=candidate.course_progress_origin_m+candidate.request.initial_state[1]+candidate.request.initial_state[4];
          request.path_length_m=1000.0;request.circular=true;request.progress_continuity_tolerance_m=1.5;
          request.measured_to_control_path=world.control_prefix;
          request.measured_to_control_elapsed_sec=world.control_prefix_elapsed_sec;
          request.control_pose=world.control_prefix.back();
          request.current_wall_grid=candidate.wall_grid;request.current_footprint=world.physical_footprint;
          request.obstacles={world.observation_generation,world.observed_sec,{},world.current};
          for(const auto & peer:world.obstacles){retained::DynamicObstacle o;o.id=peer.id;o.circle={peer.x_m,peer.y_m,peer.velocity_x_mps,peer.velocity_y_mps,peer.radius_m};request.obstacles.obstacles.push_back(o);}
          request.current_speed_mps=8.718348824128569;
          request.control_origin_speed_mps=candidate.request.initial_state[3];
          request.current_time_steering_rad=0.092016;
          request.current_steering_rad=candidate.request.current_steering_rad;
          request.current_response_steering_rad=candidate.request.current_response_steering_rad;
          request.previous_published_steering_rad=request.current_steering_rad;
          request.previous_published_command_age_sec=0.02;
          request.stop_lateral_policy=world.terminal_stop_lateral_policy;
          request.minimum_acceleration_mps2=world.terminal_stop_minimum_acceleration_mps2;
          request.maximum_acceleration_mps2=1.37;
          const auto joined=retained::evaluate(request);
          row["join"]=retained::to_string(joined.reason);
          row["join_wall_scope"]=retained::to_string(joined.static_wall_scope);
          row["join_dynamic_scope"]=retained::to_string(joined.dynamic_obstacle_scope);
          row["join_continuation_scope"]=m::mpcc_rate_resolved_physical_adapter::to_string(joined.continuation_scope);
          row["join_wall_clear"]=joined.continuation_path_clearance.valid && joined.continuation_path_clearance.clear;
          row["join_terminal_dynamic_min"]=joined.terminal_stop_minimum_dynamic_clearance_m;
          row["join_terminal_wall_clear"]=joined.terminal_stop_path_clearance.valid && joined.terminal_stop_path_clearance.clear;
          if(joined.proof) {
            row["continuation_terminal_speed"]=joined.proof->continuation_trajectory.velocity_mps.back();
            row["solved_suffix_used"]=joined.terminal_stop_uses_solved_suffix;
            const auto command=m::mpcc_rate_resolved_production_adapter::build(joined);
            row["command_adapter_available"]=command.authority.has_value();
            if(command.authority)row["first_acceleration_mps2"]=command.authority->command.acceleration_mps2;
          }
        }
        row["authority"]=false;report["arms"].push_back(row);std::cout<<row<<'\n';
      }
    }
  }
  YAML::Emitter e;e.SetDoublePrecision(std::numeric_limits<double>::max_digits10);
  e<<report;std::ofstream(argv[2])<<e.c_str()<<'\n';
}
