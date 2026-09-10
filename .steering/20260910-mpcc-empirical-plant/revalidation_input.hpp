#pragma once
#include "multi_purpose_mpc_ros/mpcc_rate_resolved_retained_revalidation.hpp"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <stdexcept>
namespace mpcc_observation {
namespace m=multi_purpose_mpc_ros;
inline m::mpcc_rate_resolved_retained_revalidation::Request read_request(
  const YAML::Node & n,const std::filesystem::path & directory) {
  namespace retained=m::mpcc_rate_resolved_retained_revalidation;
  retained::Request r;
  r.decision_id=n["decision_id"].as<std::uint64_t>();
  r.now_sec=n["now_sec"].as<double>();r.control_origin_sec=n["control_origin_sec"].as<double>();
  const auto intent=n["current_intent"].as<std::string>();
  bool intent_found=false;
  for(const auto value:{m::mpcc_execution_contract::ControlIntent::Track,m::mpcc_execution_contract::ControlIntent::Cruise,
    m::mpcc_execution_contract::ControlIntent::Follow,m::mpcc_execution_contract::ControlIntent::Hold,
    m::mpcc_execution_contract::ControlIntent::Stop,m::mpcc_execution_contract::ControlIntent::ShiftOut,
    m::mpcc_execution_contract::ControlIntent::Pass,m::mpcc_execution_contract::ControlIntent::Return,
    m::mpcc_execution_contract::ControlIntent::Rejoin}) {
    if(intent==m::mpcc_execution_contract::to_string(value)){r.current_intent=value;intent_found=true;break;}
  }
  if(!intent_found)throw std::runtime_error("unknown captured intent");
  const auto clock=n["execution_clock"];const auto kind=clock["kind"].as<std::string>();
  if(kind=="published-plan")r.execution_clock.kind=retained::ExecutionClockKind::PublishedPlan;
  else if(kind=="time-aligned-candidate")r.execution_clock.kind=retained::ExecutionClockKind::TimeAlignedCandidate;
  else if(kind=="bootstrap-candidate")r.execution_clock.kind=retained::ExecutionClockKind::BootstrapCandidate;
  else if(kind!="unknown")throw std::runtime_error("unknown captured clock");
  r.execution_clock.first_published_control_origin_sec=clock["first_published_control_origin_sec"].as<double>();
  r.execution_clock.first_published_artifact_elapsed_sec=clock["first_published_artifact_elapsed_sec"].as<double>();
  r.control_origin_physical_progress_m=n["control_origin_physical_progress_m"].as<double>();
  r.path_length_m=n["path_length_m"].as<double>();r.progress_continuity_tolerance_m=n["progress_continuity_tolerance_m"].as<double>();
  r.circular=n["circular"].as<bool>();
  r.current_speed_mps=n["current_speed_mps"].as<double>();r.control_origin_speed_mps=n["control_origin_speed_mps"].as<double>();
  r.current_time_steering_rad=n["current_time_steering_rad"].as<double>();r.current_steering_rad=n["current_steering_rad"].as<double>();
  r.current_response_steering_rad=n["current_response_steering_rad"].as<double>();
  r.current_lateral_velocity_mps=n["current_lateral_velocity_mps"].as<double>();
  r.current_yaw_rate_radps=n["current_yaw_rate_radps"].as<double>();
  r.previous_published_steering_rad=n["previous_published_steering_rad"].as<double>();
  r.previous_published_command_age_sec=n["previous_published_command_age_sec"].as<double>();
  r.minimum_acceleration_mps2=n["minimum_acceleration_mps2"].as<double>();r.maximum_acceleration_mps2=n["maximum_acceleration_mps2"].as<double>();
  const auto pose=[](const YAML::Node & p){return m::recovery_footprint::Pose2D{p["x_m"].as<double>(),p["y_m"].as<double>(),p["yaw_rad"].as<double>()};};
  r.control_pose=pose(n["control_pose"]);
  for(const auto & p:n["measured_to_control_path"])r.measured_to_control_path.push_back(pose(p));
  r.measured_to_control_elapsed_sec=n["measured_to_control_elapsed_sec"].as<std::vector<double>>();
  const auto footprint=n["current_footprint"];
  r.current_footprint={footprint["front_extent_m"].as<double>(),footprint["rear_extent_m"].as<double>(),
    footprint["left_extent_m"].as<double>(),footprint["right_extent_m"].as<double>(),footprint["margin_m"].as<double>()};
  const auto policy=n["stop_lateral_policy"];
  r.stop_lateral_policy.wheelbase_m=policy["wheelbase_m"].as<double>();
  r.stop_lateral_policy.maximum_abs_steering_rad=policy["maximum_abs_steering_rad"].as<double>();
  r.stop_lateral_policy.maximum_abs_steering_rate_radps=policy["maximum_abs_steering_rate_radps"].as<double>();
  r.stop_lateral_policy.maximum_lateral_acceleration_mps2=policy["maximum_lateral_acceleration_mps2"].as<double>();
  r.stop_lateral_policy.steering_command_gain=policy["steering_command_gain"].as<double>();
  r.stop_lateral_policy.lateral_gain=policy["lateral_gain"].as<double>();r.stop_lateral_policy.heading_gain=policy["heading_gain"].as<double>();
  const auto peers=n["obstacles"];
  r.obstacles={peers["generation"].as<std::uint64_t>(),peers["observed_sec"].as<double>(),{},peers["current"].as<bool>()};
  for(const auto & p:peers["obstacles"])r.obstacles.obstacles.push_back({p["id"].as<std::string>(),
    {p["x_m"].as<double>(),p["y_m"].as<double>(),p["velocity_x_mps"].as<double>(),p["velocity_y_mps"].as<double>(),p["radius_m"].as<double>()}});
  if(n["follow_target_available"].as<bool>()) {
    const auto f=n["follow_target"];retained::FollowTargetObservation target;
    target.target_id=f["target_id"].as<std::string>();target.observation_generation=f["observation_generation"].as<std::uint64_t>();
    target.observed_sec=f["observed_sec"].as<double>();target.current_target_gap_m=f["current_target_gap_m"].as<double>();
    target.hard_gap_m=f["hard_gap_m"].as<double>();target.target_speed_mps=f["target_speed_mps"].as<double>();
    target.elapsed_time_sec=f["elapsed_time_sec"].as<std::vector<double>>();
    target.target_progress_from_current_origin_m=f["target_progress_from_current_origin_m"].as<std::vector<double>>();
    target.current=f["current"].as<bool>();r.follow_target=target;
  }
  const auto g=n["current_wall_grid"];
  if(g["available"].as<bool>()) {
    auto grid=std::make_shared<m::recovery_footprint::OccupancyGrid>();
    grid->width=g["width"].as<std::size_t>();grid->height=g["height"].as<std::size_t>();grid->resolution_m=g["resolution_m"].as<double>();
    grid->origin_x_m=g["origin_x_m"].as<double>();grid->origin_y_m=g["origin_y_m"].as<double>();
    const auto axis=g["y_axis"].as<std::string>();
    if(axis!="row-zero-at-minimum-y" && axis!="row-zero-at-maximum-y")throw std::runtime_error("unknown grid axis");
    grid->y_axis=axis=="row-zero-at-minimum-y" ? m::recovery_footprint::YAxisConvention::RowZeroAtMinimumY : m::recovery_footprint::YAxisConvention::RowZeroAtMaximumY;
    const auto payload=g["payload"].as<std::string>();
    if(std::filesystem::path(payload).filename()!=payload)throw std::runtime_error("nonlocal grid payload");
    std::ifstream input(directory/payload,std::ios::binary);std::int8_t byte;
    while(input.read(reinterpret_cast<char *>(&byte),sizeof(byte)))grid->cells.push_back(static_cast<m::recovery_footprint::CellState>(byte));
    if(!input.eof() || grid->cells.size()!=g["cell_count"].as<std::size_t>() || !grid->valid())throw std::runtime_error("invalid captured grid payload");
    r.current_wall_grid=grid;
  }
  return r;
}
}
