#define main preserved_certified_plan_driver_main
#include "../replay_certified_plan.cpp"
#undef main
#include "mpcc_architecture_comparison.cpp"
namespace comparison = m::mpcc_architecture_comparison;
namespace maneuver = m::mpcc_stateless_maneuver;

int main(int argc, char **argv)
{
  if (argc != 2) return 2;
  std::string detail;
  const auto recorded = snap::load_recorded_interaction_snapshot(argv[1], &detail);
  if (!recorded) throw std::runtime_error(detail);
  const auto & source = recorded->source;
  const auto fingerprint = recorded->interaction_fingerprint;
  std::cout << "authority=false source=" << fingerprint
    << " scope=A/B and fixed coarse C/D schedules; no exhaustive feasibility claim" << std::endl;
  const auto emit = [](const comparison::ArmResult & r) {
    std::cout << "arm=" << comparison::to_string(r.arm)
      << " stage=" << comparison::to_string(r.stage)
      << " candidate=" << r.candidate_fingerprint
      << " solve_ms=" << r.solver_compute_ms
      << " continuation_ms=" << r.continuation_compute_ms
      << " transition=" << r.lattice_transition_stage
      << " ahead=" << r.lattice_ahead_stage
      << " bundle=" << r.bundle.has_value()
      << " detail=" << r.detail << std::endl;
  };
  emit(comparison::evaluate_arm(comparison::Arm::PersistentA, source, fingerprint,
    fingerprint, comparison::resolve_audit_terminal_successor(source)));
  for (const int side : {1,-1}) {
    const auto arm = side == 1 ? comparison::Arm::StatelessLeftB : comparison::Arm::StatelessRightB;
    const auto built = maneuver::build(source, fingerprint, side);
    if (!built.seed) {
      emit(comparison::rejected_arm(arm, comparison::Stage::CandidateRejected, fingerprint, built.detail));
    } else {
      emit(comparison::evaluate_arm(arm, built.seed->solver_snapshot, fingerprint,
        built.seed->candidate_fingerprint, comparison::resolve_audit_terminal_successor(built.seed->solver_snapshot)));
    }
  }
  const int n = source.request.horizon_steps;
  for (const int side : {1,-1}) {
    for (const auto pair : {std::pair{0,n/2},std::pair{n/4,3*n/4},std::pair{n/2,n}}) {
      const auto arm = side == 1 ? comparison::Arm::RoughLeftC : comparison::Arm::RoughRightC;
      const auto built = maneuver::build_lattice(source, fingerprint, side, pair.first, pair.second);
      if (!built.seed) {
        emit(comparison::rejected_arm(arm, comparison::Stage::CandidateRejected, fingerprint, built.detail));
      } else {
        emit(comparison::evaluate_arm(arm, built.seed->solver_snapshot, fingerprint,
          built.seed->candidate_fingerprint, comparison::resolve_audit_terminal_successor(built.seed->solver_snapshot),
          pair.first, pair.second));
      }
      emit(comparison::evaluate_offline_continuation(
        side == 1 ? comparison::Arm::OfflineLeftD : comparison::Arm::OfflineRightD,
        source, fingerprint, side, pair.first, pair.second));
    }
  }
}
