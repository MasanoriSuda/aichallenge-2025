// Offline diagnostic only: reseal changed model inputs; never reuse an old certificate.
#include "multi_purpose_mpc_ros/mpcc_architecture_snapshot.hpp"
#include "multi_purpose_mpc_ros/mpcc_rate_resolved.hpp"
#include <yaml-cpp/yaml.h>
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
  const auto root = YAML::LoadFile(argv[1]);
  const auto output = std::filesystem::path(argv[2]);
  std::filesystem::create_directories(output);
  YAML::Node report;
  report["input"] = argv[1];
  report["original_interaction_fingerprint"] = record->interaction_fingerprint;
  report["original_problem_fingerprint"] = record->source.identity.source_context.fingerprint;
  report["original_state_schema"] = record->source.identity.source_context.state_schema_id;
  report["model"] = model::kCoordinateStateSchema;
  report["scope"] = "new model conditional rollout and resealed solve input; no certificate or publication";
  record->source.identity.source_context.state_schema_id = model::kCoordinateStateSchema;
  record->source.identity.source_context = m::mpcc_execution_contract::seal_problem_context(
    record->source.identity.source_context);
  record->source.request.course_frame = {
    std::make_shared<const std::vector<m::mpc_stage_geometry::CourseFrameKnot>>(
      record->source.wall_course_frame_knots), record->source.course_progress_origin_m};
  record->interaction_fingerprint = m::mpcc_architecture_snapshot::fingerprint_interaction_snapshot(record->source);
  // Old exact QP and its warm start belong to the old coordinate law.
  record->recorded_qp.reset();
  record->assembly_request.reset();
  report["candidate_interaction_fingerprint"] = record->interaction_fingerprint;
  report["candidate_problem_fingerprint"] = record->source.identity.source_context.fingerprint;
  const auto saved = m::mpcc_architecture_snapshot::record_proof_failure(
    record->source, m::mpcc_architecture_snapshot::PipelineStage::PhysicalProof,
    "coordinate-model-v3-offline-input", "changed coordinate model only; original controls are not certified", output / "candidate");
  if (saved.status != m::mpcc_architecture_snapshot::RecordStatus::Written) throw std::runtime_error(saved.detail);
  report["candidate_snapshot"] = saved.snapshot_file.string();
  const auto evidence = root["execution_evidence"];
  if (evidence) {
    const auto initial = evidence["predicted_states"][0];
    model::StateVector state;
    state << initial["lateral_m"].as<double>(), initial["lag_m"].as<double>(),
      initial["heading_offset_rad"].as<double>(), initial["velocity_mps"].as<double>(),
      initial["progress_m"].as<double>(), initial["steering_rad"].as<double>(),
      initial["response_steering_rad"].as<double>();
    model::LinearizationRequest request;
    request.wheelbase_m = evidence["wheelbase_m"].as<double>();
    request.yaw_response_gain = evidence["yaw_response_gain"].as<double>();
    request.yaw_response_time_constant_sec = evidence["yaw_response_time_constant_sec"].as<double>();
    request.minimum_frenet_denominator = evidence["minimum_frenet_denominator"].as<double>();
    request.minimum_stage_dt_sec = 1e-9;
    request.course_frame = record->source.request.course_frame;
    request.course_frame.progress_origin_m = evidence["course_progress_origin_m"].as<double>();
    double elapsed = 0.;
    auto append = [&]() {
      const auto frame = m::mpc_stage_geometry::sample_course_frame(
        *request.course_frame.knots, request.course_frame.progress_origin_m + state[4], 1e-9);
      if (!frame) throw std::runtime_error("course unavailable");
      YAML::Node point;
      point["elapsed_sec"] = elapsed;
      point["x_m"] = frame->x_m + std::cos(frame->heading_rad) * state[1] - std::sin(frame->heading_rad) * state[0];
      point["y_m"] = frame->y_m + std::sin(frame->heading_rad) * state[1] + std::cos(frame->heading_rad) * state[0];
      point["yaw_rad"] = frame->heading_rad + state[2];
      point["velocity_mps"] = state[3];
      report["points"].push_back(point);
    };
    append();
    for (const auto & control : evidence["control_stages"]) {
      const double duration = control["duration_sec"].as<double>();
      const auto count = static_cast<std::size_t>(std::max(1., std::ceil(duration / model::kMaximumPhysicalIntegrationStepSec)));
      request.stage_dt_sec = duration / static_cast<double>(count);
      request.reference_acceleration_mps2 = control["acceleration_mps2"].as<double>();
      request.reference_steering_rate_radps = control["steering_rate_radps"].as<double>();
      request.reference_virtual_progress_speed_mps = control["virtual_progress_speed_mps"].as<double>();
      request.reference_path_curvature_radpm = control["path_curvature_radpm"].as<double>();
      for (std::size_t step = 0; step < count; ++step) {
        request.reference_lateral_m = state[0]; request.reference_lag_m = state[1];
        request.reference_heading_rad = state[2]; request.reference_velocity_mps = state[3];
        request.reference_progress_m = state[4]; request.reference_steering_rad = state[5];
        request.reference_response_steering_rad = state[6];
        const auto next = model::evaluate_temporal_frenet_transition(request);
        if (!next) throw std::runtime_error("conditional transition rejected");
        state = next->next_state;
        elapsed += request.stage_dt_sec;
        append();
      }
    }
  }
  YAML::Emitter emitter;
  emitter.SetDoublePrecision(std::numeric_limits<double>::max_digits10);
  emitter << report;
  std::ofstream(output / "model-input-and-rollout.yaml") << emitter.c_str() << '\n';
  std::cout << saved.snapshot_file << ", conditional points=" << report["points"].size() << '\n';
}
