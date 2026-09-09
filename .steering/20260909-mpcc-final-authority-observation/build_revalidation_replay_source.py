"""Reuse the reviewed exact artifact decoder, changing its role to inspected evidence."""
from pathlib import Path
s=Path(__file__).resolve().parent
text=(s.parent/'20260909-mpcc-stop-proof-provenance/replay_original_328.cpp').read_text()
text='#include "revalidation_input.hpp"\n'+text
text=text.replace('if (argc != 3) return 2;','if (argc != 4) return 2;')
text=text.replace('mpcc-published-execution-evidence/v1','mpcc-inspected-execution-artifact/v1')
old='  output["trajectory_role"] = node["publication"]["source_kind"].as<std::string>() == "exact-executed" ? "exact-executed-source" : "bundle-source-only-not-the-published-bundle";'
assert text.count(old)==1
text=text.replace(old,'  output["trajectory_role"] = "inspected-execution-not-publication";')
text=text.replace('  output["publication"] = node["publication"];\n','')
old='    row["publication_aligned_time_sec"] = node["publication"]["publication_control_origin_sec"].as<double>() + t - node["publication"]["publication_artifact_elapsed_sec"].as<double>();\n'
assert text.count(old)==1;text=text.replace(old,'')
anchor='    if (!plan.plan) throw std::runtime_error("original published artifact cannot be certified");'
assert text.count(anchor)==1
text=text.replace(anchor,anchor+r'''
    const auto failure=m::mpcc_architecture_snapshot::load_recorded_interaction_snapshot(argv[3],&detail);
    if(!failure)throw std::runtime_error(detail);
    const auto failure_node=YAML::LoadFile(argv[3]);
    const auto observation=failure_node["revalidation_evidence"];
    if(observation["status"].as<std::string>()!="present" || observation["inspected_plan_status"].as<std::string>()!="present")
      throw std::runtime_error("complete current request and inspected plan required; no substitution");
    auto request=mpcc_observation::read_request(observation["request"],std::filesystem::path(argv[3]).parent_path());
    if(request.decision_id!=failure->source.identity.source_context.decision_id ||
      request.now_sec!=failure->source.identity.snapshot_sec || request.control_origin_sec!=failure->source.control_prediction_origin_sec)
      throw std::runtime_error("failure decision/time association mismatch");
    request.plan=plan.plan;
    const auto current=retained::evaluate(request);
    const auto stop=retained::evaluate_stop_successor(request);
    output["failure_world_fingerprint"]=failure->interaction_fingerprint;
    output["failure_outcome"]=failure_node["failure_outcome"];
    output["failure_detail"]=failure_node["failure_detail"];
    auto n=output["current_revalidation"];
    n["decision_id"]=request.decision_id;n["inspected_sequence"]=value.identity.sequence;
    n["reason"]=retained::to_string(current.reason);n["proof_available"]=current.proof.has_value();
    n["clock_kind"]=retained::to_string(current.execution_clock_kind);n["cursor_elapsed_sec"]=current.cursor_elapsed_sec;
    n["physical_progress_delta_m"]=current.progress_difference_m;n["pose_error_m"]=current.control_pose_error_m;n["yaw_error_rad"]=current.control_yaw_error_rad;
    n["current_speed_mps"]=request.current_speed_mps;n["control_speed_mps"]=request.control_origin_speed_mps;n["expected_speed_mps"]=current.expected_speed_mps;
    n["physical_steering_now_rad"]=request.current_time_steering_rad;n["previous_steering_rad"]=request.previous_published_steering_rad;
    n["previous_command_age_sec"]=request.previous_published_command_age_sec;n["expected_steering_rad"]=current.expected_steering_rad;
    n["steering_delta_rad"]=current.steering_difference_rad;n["steering_lower_rad"]=current.reachable_steering_lower_rad;
    n["steering_upper_rad"]=current.reachable_steering_upper_rad;n["steering_duration_sec"]=current.steering_reachability_duration_sec;
    n["continuation_model"]=m::mpcc_rate_resolved_physical_adapter::to_string(current.continuation_reason);
    n["continuation_exact"]=m::race_mpcc_foundation::exact_physical_execution_trajectory_reason_name(current.continuation_exact_reason);
    n["continuation_scope"]=m::mpcc_rate_resolved_physical_adapter::to_string(current.continuation_scope);
    n["continuation_wall_clear"]=current.continuation_path_clearance.valid && current.continuation_path_clearance.clear;
    n["continuation_wall_reject_index"]=current.continuation_path_clearance.rejected_path_index;
    n["terminal_attempted"]=current.terminal_stop_attempted;n["terminal_certified"]=current.terminal_stop_certified;
    n["terminal_uses_solved_suffix"]=current.terminal_stop_uses_solved_suffix;
    n["terminal_reference_attempts"]=current.terminal_stop_reference_attempts;
    n["terminal_wall_valid"]=current.terminal_stop_path_clearance.valid;n["terminal_wall_clear"]=current.terminal_stop_path_clearance.clear;
    n["terminal_wall_reject_index"]=current.terminal_stop_path_clearance.rejected_path_index;
    n["terminal_dynamic_min_m"]=current.terminal_stop_minimum_dynamic_clearance_m;
    n["terminal_dynamic_blocker"]=current.terminal_stop_blocking_obstacle_id;
    n["independent_stop"]=retained::to_string(stop.reason);
    n["independent_stop_wall_clear"]=stop.successor_path_clearance.valid && stop.successor_path_clearance.clear;
    n["independent_stop_dynamic_clear"]=stop.dynamic_clearance.valid && stop.dynamic_clearance.clear;
    n["independent_stop_dynamic_min_m"]=stop.dynamic_clearance.minimum_clearance_m;
    output["authority"]=false;
''')
text=text.replace('std::cout << output["original_execution"]','std::cout << output["current_revalidation"]')
(s/'replay_revalidation.cpp').write_text(text)
