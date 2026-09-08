#include <yaml-cpp/yaml.h>
#include <fstream>
#include <limits>
void capture_execution(const multi_purpose_mpc_ros::mpcc_rate_resolved_execution_artifact::ExecutionArtifact & a,const char *path) {
  YAML::Node n;n["schema"]="offline-complete-stop-artifact/v1";n["published"]=false;
  n["source_sequence"]=a.identity.sequence;n["source_problem_fingerprint"]=a.identity.source_context.fingerprint;n["source_snapshot_sec"]=a.identity.snapshot_sec;
  n["prediction_origin_sec"]=a.prediction_origin_sec;
  n["publication_interval_sec"]=a.publication_interval_sec;
  n["completed_sec"]=a.completed_sec;
  n["course_progress_origin_m"]=a.course_progress_origin_m;
  n["semantic_initial_steering_rad"]=a.semantic_initial_steering_rad;
  n["semantic_initial_response_steering_rad"]=a.semantic_initial_response_steering_rad;
  n["wheelbase_m"]=a.wheelbase_m;
  n["yaw_response_gain"]=a.yaw_response_gain;
  n["yaw_response_time_constant_sec"]=a.yaw_response_time_constant_sec;
  n["minimum_frenet_denominator"]=a.minimum_frenet_denominator;
  n["maximum_abs_steering_rad"]=a.maximum_abs_steering_rad;
  n["maximum_abs_steering_rate_radps"]=a.maximum_abs_steering_rate_radps;
  n["physical_global_tolerance"]=a.physical_global_tolerance;
  n["maximum_constraint_violation"]=a.maximum_constraint_violation;
  n["maximum_normalized_constraint_violation"]=a.maximum_normalized_constraint_violation;
  auto state=[](const auto &s){YAML::Node n;n["lateral_m"]=s.lateral_m;n["lag_m"]=s.lag_m;n["heading_offset_rad"]=s.heading_offset_rad;n["velocity_mps"]=s.velocity_mps;n["progress_m"]=s.progress_m;n["steering_rad"]=s.steering_rad;n["response_steering_rad"]=s.response_steering_rad;return n;};
  n["semantic_initial_state"]=state(*a.semantic_initial_state);for(const auto &s:a.predicted_states)n["predicted_states"].push_back(state(s));
  for(const auto &c:a.control_stages){YAML::Node s;s["acceleration_mps2"]=c.acceleration_mps2;s["steering_rate_radps"]=c.steering_rate_radps;s["virtual_progress_speed_mps"]=c.virtual_progress_speed_mps;s["duration_sec"]=c.duration_sec;s["virtual_progress_lower_mps"]=c.virtual_progress_lower_mps;s["virtual_progress_upper_mps"]=c.virtual_progress_upper_mps;s["acceleration_lower_mps2"]=c.acceleration_lower_mps2;s["acceleration_upper_mps2"]=c.acceleration_upper_mps2;s["path_curvature_radpm"]=c.path_curvature_radpm;n["control_stages"].push_back(s);}
  n["nominal_path_distance_m"]=a.nominal_path_distance_m;
  n["lateral_lower_m"]=a.lateral_lower_m;
  n["lateral_upper_m"]=a.lateral_upper_m;
  n["terminal_intent_contract"]["active"]=a.terminal_intent_contract.active;
  n["terminal_intent_contract"]["lateral_reference_m"]=a.terminal_intent_contract.lateral_reference_m;
  n["terminal_intent_contract"]["lateral_tolerance_m"]=a.terminal_intent_contract.lateral_tolerance_m;
  n["terminal_intent_contract"]["heading_reference_rad"]=a.terminal_intent_contract.heading_reference_rad;
  n["terminal_intent_contract"]["heading_tolerance_rad"]=a.terminal_intent_contract.heading_tolerance_rad;
  n["terminal_intent_certificate"]["active"]=a.terminal_intent_certificate.active;
  n["terminal_intent_certificate"]["solved_horizon_steps"]=a.terminal_intent_certificate.solved_horizon_steps;
  n["terminal_intent_certificate"]["solved_lateral_m"]=a.terminal_intent_certificate.solved_lateral_m;
  n["terminal_intent_certificate"]["solved_heading_rad"]=a.terminal_intent_certificate.solved_heading_rad;
  n["course_frame"]["progress_origin_m"]=a.course_frame.progress_origin_m;n["course_frame"]["knots"]= "exact source capture owns all immutable knots";
  YAML::Emitter e;e.SetDoublePrecision(std::numeric_limits<double>::max_digits10);e<<n;std::ofstream(path)<<e.c_str()<<'\n';
}
