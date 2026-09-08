// Offline audit only. Own and reseal a target tube from the recorded peer world.
#include "multi_purpose_mpc_ros/mpcc_architecture_snapshot.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved.hpp"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
namespace m = multi_purpose_mpc_ros;
namespace model = m::mpcc_rate_resolved;
int main(int argc, char **argv) {
  if (argc != 3) return 2;
  std::string detail;
  auto record = m::mpcc_architecture_snapshot::load_recorded_interaction_snapshot(argv[1], &detail);
  if (!record) throw std::runtime_error(detail);
  auto & source = record->source;
  const auto root = YAML::LoadFile(argv[1]);
  const auto out = std::filesystem::path(argv[2]);
  std::filesystem::create_directories(out);
  YAML::Node report;
  report["original_interaction_fingerprint"] = record->interaction_fingerprint;
  report["scope"] = "observation-only target producer comparison; no runtime authority";
  const auto & request = source.request;
  source.request.course_frame = {
    std::make_shared<const std::vector<m::mpc_stage_geometry::CourseFrameKnot>>(
      source.wall_course_frame_knots), source.course_progress_origin_m};
  YAML::Node saved_primal;
  if (root["production_outcome"] && root["production_outcome"]["result"])
    saved_primal = root["production_outcome"]["result"]["primal"];
  if (saved_primal.IsSequence()) {
    const int offset = model::kStateDimension * (request.horizon_steps + 1);
    for (const bool semantic : {false, true}) {
      model::LinearizationRequest transition;
      transition.reference_lateral_m = semantic ? request.initial_state[0] : saved_primal[0].as<double>();
      transition.reference_lag_m = semantic ? request.initial_state[1] : saved_primal[1].as<double>();
      transition.reference_heading_rad = semantic ? request.initial_state[2] : saved_primal[2].as<double>();
      transition.reference_velocity_mps = semantic ? request.initial_state[3] : saved_primal[3].as<double>();
      transition.reference_progress_m = semantic ? request.initial_state[4] : saved_primal[4].as<double>();
      transition.reference_steering_rad = semantic ? request.current_steering_rad : saved_primal[5].as<double>();
      transition.reference_response_steering_rad = semantic ? request.current_response_steering_rad : saved_primal[6].as<double>();
      transition.reference_acceleration_mps2 = saved_primal[offset].as<double>();
      transition.reference_steering_rate_radps = saved_primal[offset+1].as<double>();
      transition.reference_virtual_progress_speed_mps = saved_primal[offset+2].as<double>();
      transition.reference_path_curvature_radpm = request.inputs.front().path_curvature_radpm;
      transition.wheelbase_m = request.wheelbase_m;
      transition.yaw_response_gain = request.yaw_response_gain;
      transition.yaw_response_time_constant_sec = request.yaw_response_time_constant_sec;
      transition.minimum_frenet_denominator = request.minimum_frenet_denominator;
      const double dt = request.inputs.front().stage_dt_sec;
      transition.stage_dt_sec = dt / std::max(1., std::ceil(dt / model::kMaximumPhysicalIntegrationStepSec));
      transition.minimum_stage_dt_sec = transition.stage_dt_sec;
      transition.maximum_stage_dt_sec = transition.stage_dt_sec;
      transition.course_frame = request.course_frame;
      const std::string mode = semantic ? "semantic_initial" : "raw_primal_initial";
      report[mode]["progress_m"] = transition.reference_progress_m;
      report[mode]["course_sample_available"] = static_cast<bool>(m::mpc_stage_geometry::sample_course_frame(
        source.wall_course_frame_knots, source.course_progress_origin_m + transition.reference_progress_m));
      report[mode]["first_physical_step_available"] = static_cast<bool>(model::evaluate_temporal_frenet_transition(transition));
    }
  }
  if (!source.replay_world || source.replay_world->obstacles.size() != 1)
    throw std::runtime_error("audit requires the sealed two-vehicle scene");
  const auto & world = *source.replay_world;
  const auto & peer = world.obstacles.front();
  source.dynamic_obstacle_stages.clear();
  double elapsed = source.control_prediction_origin_sec - world.observed_sec;
  // The native physical predicate uses constant velocity, not acceleration.
  for (const auto & input : request.inputs) {
    elapsed += input.stage_dt_sec;
    const double x = peer.x_m + peer.velocity_x_mps * elapsed;
    const double y = peer.y_m + peer.velocity_y_mps * elapsed;
    double best = std::numeric_limits<double>::infinity(), progress = 0., lateral = 0.;
    for (std::size_t i = 1; i < source.wall_course_frame_knots.size(); ++i) {
      const auto & a = source.wall_course_frame_knots[i-1];
      const auto & b = source.wall_course_frame_knots[i];
      const double dx = b.x_m-a.x_m, dy = b.y_m-a.y_m;
      const double fraction = std::clamp(((x-a.x_m)*dx+(y-a.y_m)*dy)/(dx*dx+dy*dy),0.,1.);
      const double distance = std::hypot(x-a.x_m-fraction*dx,y-a.y_m-fraction*dy);
      if (distance >= best) continue;
      best = distance;
      progress = a.progress_m + fraction*(b.progress_m-a.progress_m);
      const auto frame = m::mpc_stage_geometry::sample_course_frame(source.wall_course_frame_knots, progress);
      if (!frame) throw std::runtime_error("projection frame unavailable");
      lateral = -std::sin(frame->heading_rad)*(x-frame->x_m)+std::cos(frame->heading_rad)*(y-frame->y_m);
    }
    source.dynamic_obstacle_stages.push_back({true, progress-source.course_progress_origin_m, lateral,
      world.physical_footprint.front_extent_m+world.physical_footprint.margin_m+peer.radius_m,
      std::max(world.physical_footprint.left_extent_m,world.physical_footprint.right_extent_m)+world.physical_footprint.margin_m+peer.radius_m});
  }
  source.dynamic_obstacle_refinement_active = true;
  auto & context = source.identity.source_context;
  context.dynamic_obstacle_constraint_active = true;
  context.dynamic_obstacle_id = peer.id;
  context.dynamic_obstacle_generation = world.observation_generation;
  context = m::mpcc_execution_contract::seal_problem_context(context);
  const auto saved = m::mpcc_architecture_snapshot::record_proof_failure(source,
    m::mpcc_architecture_snapshot::PipelineStage::PhysicalProof,
    "world-target-tube-audit", "same world/hard bounds; Cartesian constant-velocity peer tube", out/"candidate");
  if (saved.status != m::mpcc_architecture_snapshot::RecordStatus::Written) throw std::runtime_error(saved.detail);
  report["candidate_snapshot"] = saved.snapshot_file.string();
  report["candidate_interaction_fingerprint"] = m::mpcc_architecture_snapshot::fingerprint_interaction_snapshot(source);
  YAML::Emitter emitter;
  emitter.SetDoublePrecision(std::numeric_limits<double>::max_digits10);
  emitter << report;
  std::ofstream(out/"report.yaml") << emitter.c_str() << '\n';
  std::cout << emitter.c_str() << '\n';
}
