#include "revalidation_input.hpp"
#include "multi_purpose_mpc_ros/mpcc_vehicle_model_yaml.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_production_adapter.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_stop_successor_bundle.hpp"
#include "mpcc_architecture_snapshot.cpp"
#include <iostream>
#include <iomanip>
namespace m=multi_purpose_mpc_ros;
namespace snap=m::mpcc_architecture_snapshot;
namespace retained=m::mpcc_rate_resolved_retained_revalidation;
namespace artifact=m::mpcc_rate_resolved_execution_artifact;
namespace physical=m::mpcc_rate_resolved_physical_wall;
namespace certified=m::mpcc_rate_resolved_certified_plan;
namespace bundle=m::mpcc_rate_resolved_stop_successor_bundle;
namespace contract=m::mpcc_execution_contract;
std::vector<m::mpc_stage_geometry::CourseFrameKnot> knots(const YAML::Node & n) {
 std::vector<m::mpc_stage_geometry::CourseFrameKnot> out;
 for(const auto & x:n)out.push_back({x["progress_m"].as<double>(),x["x_m"].as<double>(),x["y_m"].as<double>(),x["heading_rad"].as<double>(),x["waypoint"].as<int>()});
 return out;
}
m::recovery_footprint::Pose2D pose(const YAML::Node & n){return {n["x_m"].as<double>(),n["y_m"].as<double>(),n["yaw_rad"].as<double>()};}
std::shared_ptr<const certified::CertifiedPlan> read_plan(const YAML::Node & evidence,const std::filesystem::path & path, const YAML::Node & source_node = YAML::Node()) {
 if(evidence["schema"].as<std::string>()!="mpcc-certified-plan-observation/v1" || evidence["status"].as<std::string>()!="present")throw std::runtime_error("missing exact certified plan");
 const auto node=evidence["artifact"],c=node["problem_context"];
 artifact::ExecutionArtifact value;
 auto & ctx=value.identity.source_context;
 ctx.intent=*snap::parse_intent(c["intent"].as<std::string>());
 ctx.formulation=*snap::parse_formulation(c["formulation"].as<std::string>());
#define CONTEXT(name,type) ctx.name=c[#name].as<type>();
 CONTEXT(decision_id,std::uint64_t)
 CONTEXT(intent_generation,std::uint64_t)
 CONTEXT(observation_generation,std::uint64_t)
 CONTEXT(stage_geometry_id,std::uint64_t)
 CONTEXT(target_obstacle_generation,std::uint64_t)
 CONTEXT(dynamic_obstacle_generation,std::uint64_t)
 CONTEXT(fingerprint,std::uint64_t)
 CONTEXT(vehicle_model_fingerprint,std::uint64_t)
 CONTEXT(execution_side_sign,int)
 CONTEXT(dynamic_obstacle_side_sign,int)
 CONTEXT(horizon_steps,int)
 CONTEXT(dynamic_obstacle_constraint_active,bool)
 CONTEXT(target_id,std::string)
 CONTEXT(dynamic_obstacle_id,std::string)
 CONTEXT(state_schema_id,std::string)
 CONTEXT(input_schema_id,std::string)
 CONTEXT(bounds_schema_id,std::string)
 CONTEXT(cost_schema_id,std::string)
#undef CONTEXT
 ctx.applied_program_fingerprint=c["applied_program_fingerprint"].as<std::uint64_t>(0U);
 value.identity.sequence=node["source_sequence"].as<std::uint64_t>();value.identity.snapshot_sec=node["source_snapshot_sec"].as<double>();
 if(ctx.fingerprint!=node["source_problem_fingerprint"].as<std::uint64_t>() || !contract::problem_context_complete(ctx))throw std::runtime_error("original context fingerprint invalid");
  value.prediction_origin_sec = node["prediction_origin_sec"].as<double>();
  value.publication_interval_sec = node["publication_interval_sec"].as<double>();
  value.completed_sec = node["completed_sec"].as<double>();
  value.course_progress_origin_m = node["course_progress_origin_m"].as<double>();
  value.semantic_initial_steering_rad = node["semantic_initial_steering_rad"].as<double>();
  value.semantic_initial_response_steering_rad = node["semantic_initial_response_steering_rad"].as<double>();
  value.wheelbase_m = node["wheelbase_m"].as<double>();
  value.vehicle_model = *m::mpcc_vehicle_model::decode_parameters(node["vehicle_model"]);
  value.terminal_body_rest_required = node["terminal_body_rest_required"].as<bool>();
  value.minimum_frenet_denominator = node["minimum_frenet_denominator"].as<double>();
  value.maximum_abs_steering_rad = node["maximum_abs_steering_rad"].as<double>();
  value.maximum_abs_steering_rate_radps = node["maximum_abs_steering_rate_radps"].as<double>();
  value.physical_global_tolerance = node["physical_global_tolerance"].as<double>();
  value.maximum_constraint_violation = node["maximum_constraint_violation"].as<double>();
  value.maximum_normalized_constraint_violation = node["maximum_normalized_constraint_violation"].as<double>();
  value.terminal_intent_contract.active = node["terminal_intent_contract"]["active"].as<bool>();
  value.terminal_intent_contract.lateral_reference_m = node["terminal_intent_contract"]["lateral_reference_m"].as<double>();
  value.terminal_intent_contract.lateral_tolerance_m = node["terminal_intent_contract"]["lateral_tolerance_m"].as<double>();
  value.terminal_intent_contract.heading_reference_rad = node["terminal_intent_contract"]["heading_reference_rad"].as<double>();
  value.terminal_intent_contract.heading_tolerance_rad = node["terminal_intent_contract"]["heading_tolerance_rad"].as<double>();
  value.terminal_intent_certificate.active = node["terminal_intent_certificate"]["active"].as<bool>();
  value.terminal_intent_certificate.solved_horizon_steps = node["terminal_intent_certificate"]["solved_horizon_steps"].as<std::size_t>();
  value.terminal_intent_certificate.solved_lateral_m = node["terminal_intent_certificate"]["solved_lateral_m"].as<double>();
  value.terminal_intent_certificate.solved_heading_rad = node["terminal_intent_certificate"]["solved_heading_rad"].as<double>();
  for (const auto & item : node["predicted_states"]) {
    m::mpcc_rate_resolved_execution_artifact::PredictedState entry;
    entry.lateral_m = item["lateral_m"].as<double>();
    entry.lag_m = item["lag_m"].as<double>();
    entry.heading_offset_rad = item["heading_offset_rad"].as<double>();
    entry.velocity_mps = item["velocity_mps"].as<double>();
    entry.progress_m = item["progress_m"].as<double>();
    entry.steering_rad = item["steering_rad"].as<double>();
    entry.response_steering_rad = item["response_steering_rad"].as<double>();
    entry.lateral_velocity_mps = item["lateral_velocity_mps"].as<double>();
    entry.yaw_rate_radps = item["yaw_rate_radps"].as<double>();
    value.predicted_states.push_back(entry);
  }
  for (const auto & item : node["control_stages"]) {
    m::mpcc_rate_resolved_execution_artifact::ControlStage entry;
    entry.acceleration_mps2 = item["acceleration_mps2"].as<double>();
    entry.steering_rate_radps = item["steering_rate_radps"].as<double>();
    entry.virtual_progress_speed_mps = item["virtual_progress_speed_mps"].as<double>();
    entry.duration_sec = item["duration_sec"].as<double>();
    entry.virtual_progress_lower_mps = item["virtual_progress_lower_mps"].as<double>();
    entry.virtual_progress_upper_mps = item["virtual_progress_upper_mps"].as<double>();
    entry.acceleration_lower_mps2 = item["acceleration_lower_mps2"].as<double>();
    entry.acceleration_upper_mps2 = item["acceleration_upper_mps2"].as<double>();
    entry.path_curvature_radpm = item["path_curvature_radpm"].as<double>();
    value.control_stages.push_back(entry);
  }
  const auto initial = node["semantic_initial_state"];
  value.semantic_initial_state = m::mpcc_rate_resolved_execution_artifact::PredictedState{
    initial["lateral_m"].as<double>(), initial["lag_m"].as<double>(),
    initial["heading_offset_rad"].as<double>(), initial["velocity_mps"].as<double>(),
    initial["progress_m"].as<double>(), initial["steering_rad"].as<double>(),
    initial["response_steering_rad"].as<double>(),
    initial["lateral_velocity_mps"].as<double>(), initial["yaw_rate_radps"].as<double>()};
  value.nominal_path_distance_m = node["nominal_path_distance_m"].as<std::vector<double>>();
  value.lateral_lower_m = node["lateral_lower_m"].as<std::vector<double>>();
  value.lateral_upper_m = node["lateral_upper_m"].as<std::vector<double>>();

 value.course_frame={std::make_shared<const std::vector<m::mpc_stage_geometry::CourseFrameKnot>>(knots(node["course_frame_knots"])),value.course_progress_origin_m};
 if(node["applied_stop_program"]) {
  const auto provenance=m::mpcc_vehicle_model::decode_applied_program_provenance(node["applied_stop_program"],value.vehicle_model);
  if(!provenance)throw std::runtime_error("invalid captured applied Stop program");
  value.applied_stop_program=std::make_shared<const m::mpcc_vehicle_model::AppliedProgramProvenance>(*provenance);
 }
 if(artifact::validate(value)!=artifact::RejectReason::None)throw std::runtime_error("original artifact invalid");
 physical::Snapshot p;const auto n=evidence["physical_snapshot"],i=n["identity"];
 p.identity.artifact=value.identity;
 if(i["artifact_sequence"].as<std::uint64_t>()!=value.identity.sequence || i["artifact_problem_fingerprint"].as<std::uint64_t>()!=ctx.fingerprint || i["artifact_snapshot_sec"].as<double>()!=value.identity.snapshot_sec)throw std::runtime_error("physical identity substitution");
 p.identity.pose_snapshot_id=i["pose_snapshot_id"].as<std::uint64_t>();p.identity.course_frame_window_id=i["course_frame_window_id"].as<std::uint64_t>();p.identity.captured_sec=i["captured_sec"].as<double>();
 p.current_pose=pose(n["current_pose"]);for(const auto & x:n["control_prefix"])p.control_prefix.push_back(pose(x));
 p.course_frame_knots=knots(n["course_frame_knots"]);
 const auto f=n["footprint"];p.footprint={f["front_extent_m"].as<double>(),f["rear_extent_m"].as<double>(),f["left_extent_m"].as<double>(),f["right_extent_m"].as<double>(),f["margin_m"].as<double>()};
 const auto t=n["trajectory"];
#define TRAJECTORY(name,type) p.trajectory.name=t[#name].as<type>();
 TRAJECTORY(elapsed_time_sec,std::vector<double>)
 TRAJECTORY(path_distance_m,std::vector<double>)
 TRAJECTORY(lateral_m,std::vector<double>)
 TRAJECTORY(lag_m,std::vector<double>)
 TRAJECTORY(heading_offset_rad,std::vector<double>)
 TRAJECTORY(velocity_mps,std::vector<double>)
 TRAJECTORY(progress_m,std::vector<double>)
 TRAJECTORY(lateral_lower_m,std::vector<double>)
 TRAJECTORY(lateral_upper_m,std::vector<double>)
 TRAJECTORY(progress_origin_m,double)
 TRAJECTORY(minimum_lateral_bound_reserve_m,double)
 TRAJECTORY(progress_regression_tolerance_m,double)
 TRAJECTORY(velocity_lower_bound_tolerance_mps,double)
 TRAJECTORY(stationary_velocity_tolerance_mps,double)
 TRAJECTORY(lateral_bound_tolerance_m,double)
 TRAJECTORY(stationary_path_suffix_allowed,bool)
#undef TRAJECTORY
 const auto g=n["terminal_stop_course_geometry"];
 p.terminal_stop_course_geometry={g["progress_m"].as<std::vector<double>>(),g["curvature_radpm"].as<std::vector<double>>(),g["lateral_lower_m"].as<std::vector<double>>(),g["lateral_upper_m"].as<std::vector<double>>()};
 p.hard_wall_clearance_m=n["hard_wall_clearance_m"].as<double>();p.bound_tolerance_m=n["bound_tolerance_m"].as<double>();p.swept_step_m=n["swept_step_m"].as<double>();
 p.wall_grid=*snap::load_wall_grid(n["wall_grid"],path);p.wall_grid_fingerprint=n["wall_grid"]["fingerprint"].as<std::uint64_t>();
 if(p.wall_grid_fingerprint!=m::recovery_footprint::occupancy_grid_fingerprint(*p.wall_grid))throw std::runtime_error("grid fingerprint mismatch");
 const auto native=m::mpcc_rate_resolved_physical_adapter::build(value,ctx.intent,ctx.stage_geometry_id);
 if(!native.exact_trajectory)throw std::runtime_error("original native replay failed");
 const auto proof=physical::evaluate(p);
 if(physical::to_string(proof.outcome)!=evidence["physical_proof"]["outcome"].as<std::string>())throw std::runtime_error("original wall proof mismatch");
 std::shared_ptr<const m::mpcc_rate_resolved_shadow::Snapshot> source;
 if(source_node && source_node.IsMap()) {
  YAML::Node wrapped; wrapped["source"]=source_node;
  const auto loaded=snap::load_source_snapshot(wrapped,path);
  if(!loaded)throw std::runtime_error("original solver source cannot reload");
  source=std::make_shared<const m::mpcc_rate_resolved_shadow::Snapshot>(*loaded);
 }
 const auto built=certified::build(std::make_shared<const artifact::ExecutionArtifact>(value),p,proof,source);
 if(!built.plan)throw std::runtime_error("original certified plan cannot reload");
 return built.plan;
}
int main(int argc,char **argv) {
 if(argc!=3)return 2;
 const std::filesystem::path path=argv[1];const auto root=YAML::LoadFile(path.string());
 YAML::Node output;output["authority"]=false;output["solver_invocations"]=0;output["input"]=path.string();
 for(const auto & key:{"previous_accepted_revalidation_evidence","revalidation_evidence"}) {
  const auto n=root[key];auto request=mpcc_observation::read_request(n["request"],path.parent_path());
  request.plan=read_plan(n["certified_plan_evidence"],path,n["inspected_source"]);
  const auto replay=retained::evaluate(request);const auto stop=retained::evaluate_stop_successor(request);
  auto row=output[key];row["decision"]=request.decision_id;row["source"]=request.plan->execution_artifact->identity.sequence;
  row["control_speed"]=request.control_origin_speed_mps;row["control_vy"]=request.current_lateral_velocity_mps;row["control_yaw_rate"]=request.current_yaw_rate_radps;
  row["retained"]=retained::to_string(replay.reason);row["stop"]=retained::to_string(stop.reason);
  row["stop_wall_clear"]=stop.successor_path_clearance.valid && stop.successor_path_clearance.clear;row["stop_dynamic_clear"]=stop.dynamic_clearance.valid && stop.dynamic_clearance.clear;
  row["stop_samples"]=stop.actuation_samples.size();
  const auto built=bundle::build(request,stop,10000U+request.decision_id);
  row["bundle"]=bundle::to_string(built.reason);row["bundle_detail"]=bundle::to_string(built.actuation_detail);
  if(built.plan) {
   auto current=request;current.plan=built.plan;current.execution_clock={retained::ExecutionClockKind::TimeAlignedCandidate,NAN,NAN};
   const auto joined=retained::evaluate(current);row["joined"]=retained::to_string(joined.reason);row["complete_rest_suffix"]=joined.terminal_stop_uses_solved_suffix;
   const auto production=m::mpcc_rate_resolved_production_adapter::build(joined);
   row["production"]=m::mpcc_rate_resolved_production_adapter::to_string(production.reason);
   const auto & art=*built.plan->execution_artifact;
   const auto native=m::mpcc_rate_resolved_physical_adapter::build(art,art.identity.source_context.intent,art.identity.source_context.stage_geometry_id);
   row["native"]=m::mpcc_rate_resolved_physical_adapter::to_string(native.reason);
   if(production.authority){row["speed"]=production.authority->command.predicted_speed_mps;row["acceleration"]=production.authority->command.acceleration_mps2;}
  }
 }
 YAML::Emitter e;e.SetDoublePrecision(17);e<<output;std::ofstream(argv[2])<<e.c_str()<<'\n';std::cout<<e.c_str()<<'\n';return 0;
}
