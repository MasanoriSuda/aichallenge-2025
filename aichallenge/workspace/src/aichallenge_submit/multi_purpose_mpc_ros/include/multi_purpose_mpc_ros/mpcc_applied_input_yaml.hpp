#pragma once
#include "multi_purpose_mpc_ros/mpcc_applied_input_prediction.hpp"
#include <yaml-cpp/yaml.h>

namespace multi_purpose_mpc_ros::mpcc_vehicle_model {
inline YAML::Node encode_observation_provenance(
    const mpcc_vehicle_model::ObservationProvenance &v) {
  YAML::Node node;
  node["pose_source_sec"] = v.initial.source_sec;
  node["velocity_source_sec"] = v.velocity_source_sec;
  node["yaw_rate_source_sec"] = v.yaw_rate_source_sec;
  node["tire_source_sec"] = v.tire_source_sec;
  node["now_sec"] = v.now_sec;
  node["control_origin_sec"] = v.control_origin_sec;
  node["nominal_acceleration_application_delay_sec"] = v.acceleration_delay_sec;
  node["nominal_steering_application_delay_sec"] = v.steering_delay_sec;
  const auto &state = v.initial.state;
  node["initial_x_y_yaw_u_vy_r_desired_tire"] =
      std::vector<double>{state.x_m,
                          state.y_m,
                          state.yaw_rad,
                          state.forward_velocity_mps,
                          state.lateral_velocity_mps,
                          state.yaw_rate_radps,
                          state.desired_steering_rad,
                          state.tire_steering_rad};
  YAML::Node history(YAML::NodeType::Sequence);
  for (const auto &command : v.commands) {
    history.push_back(std::vector<double>{command.published_sec,
                                          command.wire_acceleration_mps2,
                                          command.wire_steering_rad});
  }
  node["published_time_wire_acceleration_wire_steering"] = history;
  return node;
}

inline std::optional<mpcc_vehicle_model::ObservationProvenance>
decode_observation_provenance(const YAML::Node &node) {
  if (!node.IsMap())
    return std::nullopt;
  mpcc_vehicle_model::ObservationProvenance v;
  v.initial.source_sec = node["pose_source_sec"].as<double>();
  v.velocity_source_sec = node["velocity_source_sec"].as<double>();
  v.yaw_rate_source_sec = node["yaw_rate_source_sec"].as<double>();
  v.tire_source_sec = node["tire_source_sec"].as<double>();
  v.now_sec = node["now_sec"].as<double>();
  v.control_origin_sec = node["control_origin_sec"].as<double>();
  v.acceleration_delay_sec =
      node["nominal_acceleration_application_delay_sec"].as<double>();
  v.steering_delay_sec =
      node["nominal_steering_application_delay_sec"].as<double>();
  const auto state =
      node["initial_x_y_yaw_u_vy_r_desired_tire"].as<std::vector<double>>();
  if (state.size() != 8U)
    return std::nullopt;
  v.initial.state = {state[0], state[1], state[2], state[3],
                     state[4], state[5], state[6], state[7]};
  const auto history = node["published_time_wire_acceleration_wire_steering"];
  if (!history.IsSequence())
    return std::nullopt;
  for (const auto &item : history) {
    if (!item.IsSequence() || item.size() != 3U)
      return std::nullopt;
    v.commands.push_back(
        {item[0].as<double>(), item[1].as<double>(), item[2].as<double>()});
  }
  return mpcc_vehicle_model::valid(v) ? std::optional{std::move(v)}
                                      : std::nullopt;
}

inline YAML::Node
encode_input_application_profile(const InputApplicationProfile &profile) {
  YAML::Node node;
  node["profile_id"] = profile.profile_id;
  node["acceleration_age_sec"] = profile.acceleration_age_sec;
  node["steering_receipt_age_sec"] = profile.steering_receipt_age_sec;
  node["steering_mechanical_delay_sec"] = profile.steering_mechanical_delay_sec;
  return node;
}
inline std::optional<InputApplicationProfile>
decode_input_application_profile(const YAML::Node &node) {
  InputApplicationProfile profile{
      node["profile_id"].as<std::string>(),
      node["acceleration_age_sec"].as<double>(),
      node["steering_receipt_age_sec"].as<double>(),
      node["steering_mechanical_delay_sec"].as<double>()};
  return valid(profile) ? std::optional{profile} : std::nullopt;
}
inline YAML::Node encode_input_program(const PublishedInputProgram &program) {
  YAML::Node node;
  node["publication_interval_sec"] = program.publication_interval_sec;
  node["repeat_last_until_rest"] = program.repeat_last_until_rest;
  node["published_time_wire_acceleration_wire_steering"] =
      YAML::Node(YAML::NodeType::Sequence);
  for (const auto &packet : program.commands)
    node["published_time_wire_acceleration_wire_steering"].push_back(
        std::vector<double>{packet.published_sec, packet.wire_acceleration_mps2,
                            packet.wire_steering_rad});
  return node;
}
inline std::optional<PublishedInputProgram>
decode_input_program(const YAML::Node &node) {
  PublishedInputProgram program{node["publication_interval_sec"].as<double>(),
                                {},
                                node["repeat_last_until_rest"].as<bool>()};
  for (const auto &packet :
       node["published_time_wire_acceleration_wire_steering"]) {
    if (packet.size() != 3)
      return std::nullopt;
    program.commands.push_back({packet[0].as<double>(), packet[1].as<double>(),
                                packet[2].as<double>()});
  }
  if (program.commands.empty() ||
      !valid(program, program.commands.front().published_sec))
    return std::nullopt;
  return program;
}
inline YAML::Node
encode_applied_program_provenance(const AppliedProgramProvenance &value) {
  YAML::Node node;
  node["schema"] = value.forward_velocity_ceiling_mps ?
    "applied-stop-provenance-v2" : "applied-stop-provenance-v1";
  if (value.forward_velocity_ceiling_mps)
    node["forward_velocity_ceiling_mps"] = *value.forward_velocity_ceiling_mps;
  node["nominal_solution_id"] = value.nominal_solution_id;
  node["nominal_problem_fingerprint"] = value.nominal_problem_fingerprint;
  node["proved_rest_sec"] = value.proved_rest_sec;
  node["observation"] = encode_observation_provenance(value.observation);
  node["profile"] = encode_input_application_profile(value.profile);
  node["program"] = encode_input_program(value.program);
  return node;
}
inline std::optional<AppliedProgramProvenance>
decode_applied_program_provenance(const YAML::Node &node,
                                  const Parameters &parameters) {
  const auto schema = node["schema"].as<std::string>();
  if (schema != "applied-stop-provenance-v1" && schema != "applied-stop-provenance-v2")
    return std::nullopt;
  if ((schema == "applied-stop-provenance-v2") !=
      static_cast<bool>(node["forward_velocity_ceiling_mps"])) return std::nullopt;
  const auto observation = decode_observation_provenance(node["observation"]);
  const auto profile = decode_input_application_profile(node["profile"]);
  const auto program = decode_input_program(node["program"]);
  if (!observation || !profile || !program)
    return std::nullopt;
  AppliedProgramProvenance value{
      node["nominal_solution_id"].as<std::uint64_t>(),
      node["nominal_problem_fingerprint"].as<std::uint64_t>(),
      *observation,
      *profile,
      *program,
      node["proved_rest_sec"].as<double>(), std::nullopt};
  if (schema == "applied-stop-provenance-v2")
    value.forward_velocity_ceiling_mps = node["forward_velocity_ceiling_mps"].as<double>();
  return applied_program_provenance_fingerprint(value, parameters) != 0
             ? std::optional{value}
             : std::nullopt;
}
} // namespace multi_purpose_mpc_ros::mpcc_vehicle_model
