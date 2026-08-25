"""Source-level deletion gates for the single-authority MPCC migration."""

from pathlib import Path
import re


SOURCE = (
    Path(__file__).resolve().parents[1] / "src" / "mpc_controller_cpp.cpp"
).read_text(encoding="utf-8")
MPCC_PROGRESS_HEADER = (
    Path(__file__).resolve().parents[1]
    / "include"
    / "multi_purpose_mpc_ros"
    / "mpcc_progress.hpp"
).read_text(encoding="utf-8")
MPCC_PROGRESS_SOURCE = (
    Path(__file__).resolve().parents[1] / "src" / "mpcc_progress.cpp"
).read_text(encoding="utf-8")
RACE_MPCC_FOUNDATION_HEADER = (
    Path(__file__).resolve().parents[1]
    / "include"
    / "multi_purpose_mpc_ros"
    / "race_mpcc_foundation.hpp"
).read_text(encoding="utf-8")
RACE_MPCC_FOUNDATION_SOURCE = (
    Path(__file__).resolve().parents[1] / "src" / "race_mpcc_foundation.cpp"
).read_text(encoding="utf-8")
V2X_OVERTAKE_HEADER = (
    Path(__file__).resolve().parents[1]
    / "include"
    / "multi_purpose_mpc_ros"
    / "v2x_overtake_core.hpp"
).read_text(encoding="utf-8")
V2X_OVERTAKE_SOURCE = (
    Path(__file__).resolve().parents[1] / "src" / "v2x_overtake_core.cpp"
).read_text(encoding="utf-8")
OVERTAKE_ORCHESTRATOR_HEADER = (
    Path(__file__).resolve().parents[1]
    / "include"
    / "multi_purpose_mpc_ros"
    / "overtake_execution_orchestrator.hpp"
).read_text(encoding="utf-8")
OVERTAKE_ORCHESTRATOR_SOURCE = (
    Path(__file__).resolve().parents[1]
    / "src"
    / "overtake_execution_orchestrator.cpp"
).read_text(encoding="utf-8")
MPCC_EXECUTION_CONTRACT_HEADER = (
    Path(__file__).resolve().parents[1]
    / "include"
    / "multi_purpose_mpc_ros"
    / "mpcc_execution_contract.hpp"
).read_text(encoding="utf-8")
MPCC_EXECUTION_CONTRACT_SOURCE = (
    Path(__file__).resolve().parents[1]
    / "src"
    / "mpcc_execution_contract.cpp"
).read_text(encoding="utf-8")
CONFIG = (
    Path(__file__).resolve().parents[1] / "config" / "config.yaml"
).read_text(encoding="utf-8")
CLOUD_CONFIG = (
    Path(__file__).resolve().parents[1] / "config" / "config_for_cloud.yaml"
).read_text(encoding="utf-8")
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LEGACY_BOOST_SURFACE = "\n".join(
    path.read_text(encoding="utf-8")
    for path in (
        PACKAGE_ROOT / "src" / "mpc_controller_cpp.cpp",
        PACKAGE_ROOT / "multi_purpose_mpc_ros" / "mpc_controller.py",
        PACKAGE_ROOT / "multi_purpose_mpc_ros" / "path_constraints_provider.py",
        PACKAGE_ROOT / "launch" / "mpc_controller.launch.py",
        PACKAGE_ROOT / "launch" / "mpc_simulation.launch.py",
        PACKAGE_ROOT / "CMakeLists.txt",
        PACKAGE_ROOT.parent / "aichallenge_submit_launch" / "launch" / "control" / "mpc.launch.xml",
        PACKAGE_ROOT.parent / "multi_purpose_mpc_ros_msgs" / "CMakeLists.txt",
    )
)


def test_canonical_normal_owner_has_no_runtime_migration_availability_switch() -> None:
    """The sole normal owner cannot be disabled after legacy authority deletion."""

    production = "\n".join(
        (
            SOURCE,
            RACE_MPCC_FOUNDATION_HEADER,
            RACE_MPCC_FOUNDATION_SOURCE,
            MPCC_PROGRESS_HEADER,
            MPCC_PROGRESS_SOURCE,
            CONFIG,
            CLOUD_CONFIG,
        )
    )
    forbidden = (
        "progress_contouring_mpcc_enabled",
        "progress_contouring_mpcc_overtake_only",
        "progress_contouring_extended_dynamics_enabled",
        "ProgressMpccDisabled",
        "MigrationBoundaryInactive",
        "ExtendedDynamicsDisabled",
        "overtake_only_boundary",
    )
    assert not [token for token in forbidden if token in production]


def test_uncertified_normal_failover_authorities_are_physically_deleted() -> None:
    """Solver failure may stop or recover, but cannot select another normal controller."""

    production = "\n".join(
        (
            SOURCE,
            V2X_OVERTAKE_HEADER,
            V2X_OVERTAKE_SOURCE,
            OVERTAKE_ORCHESTRATOR_HEADER,
            OVERTAKE_ORCHESTRATOR_SOURCE,
            MPCC_EXECUTION_CONTRACT_HEADER,
            MPCC_EXECUTION_CONTRACT_SOURCE,
            CONFIG,
            CLOUD_CONFIG,
        )
    )
    forbidden = (
        "LegacyNormalBypass",
        "legacy-normal-bypass",
        "SolverCrawl",
        "SolverBoundedContinuation",
        "solver_failure_crawl_enabled",
        "solver_failure_crawl_speed_mps",
        "resolve_solver_failure_crawl",
        "resolve_solver_failure_continuation",
        "resolve_dynamic_obstacle_lateral_escape_qualification_hold",
        "qualification_hold_used",
    )
    assert not [token for token in forbidden if token in production]


def test_legacy_boost_normal_authority_is_physically_deleted() -> None:
    """The 2025 boost relay must not bypass the canonical final command owner."""

    forbidden = (
        "use_boost_acceleration",
        "USE_BUG_ACC",
        "use_bug_acc_",
        "bug_acc_enabled",
        "AckermannControlBoostCommand",
        "boost_commander",
        "/boost_commander/command",
    )
    assert not [token for token in forbidden if token in LEGACY_BOOST_SURFACE]


def test_retired_low_speed_direct_authority_is_physically_deleted() -> None:
    """An unreachable normal authority must not remain representable in Slice 6."""

    production = "\n".join(
        (
            SOURCE,
            V2X_OVERTAKE_HEADER,
            V2X_OVERTAKE_SOURCE,
            OVERTAKE_ORCHESTRATOR_HEADER,
            OVERTAKE_ORCHESTRATOR_SOURCE,
            MPCC_EXECUTION_CONTRACT_HEADER,
            MPCC_EXECUTION_CONTRACT_SOURCE,
        )
    )
    forbidden = (
        "LowSpeedDirect",
        "LowSpeedWallStop",
        "low_speed_shift_control(",
        "low_speed_shift_control_active_",
        "low_speed_shift_control_was_active_",
        "low_speed_direct_control_active",
        "low_speed_direct_steering_bounds",
        "low_speed_direct_wall_stop_active",
        "low_speed_direct_active",
        "low_speed_wall_stop_active",
        "low_speed_avoidance_pass_control_velocity",
        "low_speed_avoidance_rejoin_control_velocity",
        "low_speed_avoidance_shift_lateral_tolerance",
        "low_speed_avoidance_shift_heading_tolerance",
        "low_speed_avoidance_shift_clear_hold_sec",
        "low_speed_avoidance_max_lateral_accel",
    )
    assert not [token for token in forbidden if token in production]
    for config in (CONFIG, CLOUD_CONFIG):
        assert not [token for token in forbidden if token in config]


def test_overtake_worker_fresh_chain_uses_sealed_snapshot_context() -> None:
    """A tactical clone must not re-derive authority omitted from its snapshot."""

    worker_start = SOURCE.index("evaluate_overtake_canonical_worker_fresh(")
    worker_end = SOURCE.index(
        "void evaluate_overtake_canonical_retained_shadow(", worker_start
    )
    worker = SOURCE[worker_start:worker_end]

    assert "make_overtake_canonical_shadow_result(problem, snapshot_context)" in worker
    assert (
        "evaluate_overtake_canonical_fresh_shadow(\n"
        "      problem, extended_problem.value(), outcome.result.value(), now_sec,\n"
        "      execution_context)"
    ) in worker
    assert "make_problem_context(\n      problem" not in worker
    assert (
        "const auto execution_context = seal_problem_context_for_problem(\n"
        "      problem, snapshot_context, extended_problem->N)"
    ) in worker

    fresh_start = SOURCE.index(
        "OvertakeCanonicalFreshShadowResult\n"
        "  evaluate_overtake_canonical_fresh_shadow("
    )
    fresh_end = SOURCE.index(
        "evaluate_overtake_canonical_worker_fresh(", fresh_start
    )
    fresh = SOURCE[fresh_start:fresh_end]

    assert "const auto context = make_problem_context(" not in fresh
    assert "const auto & context = snapshot_context;" in fresh


def test_follow_qp_keeps_planning_and_physical_gap_contracts_separate() -> None:
    """Nominal feasibility must not consume the physical hard-gap boundary."""

    assert (
        "legacy.follow_longitudinal_contract.planning_gap_m" in SOURCE
    )
    assert (
        "legacy.follow_longitudinal_contract.target_progress_m[\n"
        "          static_cast<std::size_t>(state)] -\n"
        "          legacy.follow_longitudinal_contract.planning_gap_m"
    ) in SOURCE
    retained_source = (
        PACKAGE_ROOT / "src/mpcc_rate_resolved_retained_revalidation.cpp"
    ).read_text(encoding="utf-8")
    assert "follow_target->hard_gap_m" in retained_source
    assert "state.progress_m + state.lag_m" in retained_source
    assert "Reason::FollowInitialHardGapViolation" in retained_source
    assert "Reason::FollowStageGapViolation" in retained_source


def test_overtake_wall_proof_uses_exact_five_state_trajectory() -> None:
    """Wall proof must retain lag, heading and solved progress until certified."""

    branch_start = SOURCE.index("evaluate_extended_mpcc_branch(")
    branch_end = SOURCE.index(
        "evaluate_isolated_extended_mpcc_branch(", branch_start
    )
    branch = SOURCE[branch_start:branch_end]

    proof_input = branch.index("build_exact_extended_wall_proof_input(")
    physical_proof = branch.index("solved_mpcc_execution_path_wall_safe(")
    assert proof_input < physical_proof
    assert "exact_wall_proof->aligned" in branch
    assert "physical_execution_certificate_exact_trajectory" in branch
    assert "convert_extended_solution_to_legacy(" not in branch[:physical_proof]

    helper_start = SOURCE.index("build_exact_extended_wall_proof_input(")
    helper_end = SOURCE.index(
        "executed_extended_progress_solution_wall_safe(", helper_start
    )
    helper = SOURCE[helper_start:helper_end]
    assert "extract_extended_execution_trajectory(" in helper
    assert "exact.lag_m = extracted->lag_m" in helper
    assert "exact.heading_offset_rad = extracted->heading_offset_rad" in helper
    assert "exact.progress_m = extracted->progress_m" in helper


def test_overtake_entry_speed_proof_uses_selected_exact_trajectory() -> None:
    """Final prefix admission must not fall back to legacy rollout speed."""

    admission_start = SOURCE.index(
        "const auto certified_execution_minimum_speed = []"
    )
    admission_end = SOURCE.index(
        "if (async_shadow_enabled)", admission_start
    )
    admission = SOURCE[admission_start:admission_end]

    assert "mission.physical_execution_certificate_valid" in admission
    assert "exact_physical_execution_trajectory_complete(exact)" in admission
    assert "exact.velocity_mps.begin(), exact.velocity_mps.end()" in admission
    assert (
        "request.certified_execution_minimum_speed_mps =\n"
        "          certified_execution_minimum_speed(mission);"
    ) in admission


def test_bounded_overtake_prefix_has_one_horizon_owner_end_to_end() -> None:
    """Every pre-entry consumer must use the horizon built into the QP."""

    branch_start = SOURCE.index("evaluate_extended_mpcc_branch(")
    branch_end = SOURCE.index(
        "evaluate_isolated_extended_mpcc_branch(", branch_start
    )
    branch = SOURCE[branch_start:branch_end]
    built = branch.index("const auto extended = build_extended_progress_problem(")
    consumers = branch[built:]

    assert "const int effective_horizon = extended->N;" in consumers
    assert (
        "active_extended_branch_horizon_size_ = effective_horizon;" in consumers
    )
    assert "outcome.result->constraint_tolerance, effective_horizon" in consumers
    assert "const int nx_N = nx * (effective_horizon + 1);" in consumers
    assert "tracking_wp_id, effective_horizon," in consumers
    assert "for (int stage = 1; stage < effective_horizon + 1; ++stage)" in consumers
    assert "const int terminal = nx * effective_horizon;" in consumers
    assert (
        "legacy, std::move(prospective_context), effective_horizon)" in consumers
    )

    helper_start = SOURCE.index("build_exact_extended_wall_proof_input(")
    helper_end = SOURCE.index(
        "executed_extended_progress_solution_wall_safe(", helper_start
    )
    helper = SOURCE[helper_start:helper_end]
    assert "const int effective_horizon = extended_problem.N;" in helper
    assert "extract_extended_execution_trajectory(\n      primal, effective_horizon" in helper
    assert "for (int stage = 0; stage < effective_horizon; ++stage)" in helper

    entry_start = SOURCE.index(
        "bool revalidate_overtake_entry_execution_certificate("
    )
    entry_end = SOURCE.index(
        "bool dynamic_margin_escape_solution_wall_safe(", entry_start
    )
    entry = SOURCE[entry_start:entry_end]
    assert "exact_physical_execution_trajectory_complete(" in entry
    assert "exact.lag_m" in entry
    assert "exact.heading_offset_rad" in entry
    assert "exact.progress_m" in entry
    assert "exact five-state certificate accepted" in entry
    assert "physical_execution_certificate_source_sec = now_sec" not in entry
    assert (
        "physical_execution_certificate_source_course_progress_m = model->s"
        not in entry
    )


def test_canonical_overtake_uses_production_wall_clearance_contract() -> None:
    """Fresh and retained candidates must not certify a zero-margin shadow."""

    fresh_start = SOURCE.index(
        "OvertakeCanonicalFreshShadowResult\n"
        "  evaluate_overtake_canonical_fresh_shadow("
    )
    fresh_end = SOURCE.index(
        "evaluate_overtake_canonical_worker_fresh(", fresh_start
    )
    fresh = SOURCE[fresh_start:fresh_end]
    physical_proof = fresh.index("solved_mpcc_execution_path_wall_safe(")
    physical_proof_end = fresh.index(
        "SolvedExecutionWallValidationScope::SweptFromCurrentPose",
        physical_proof,
    )
    proof_call = fresh[physical_proof:physical_proof_end]
    assert "problem.progress_execution_required_wall_clearance_m" in proof_call
    assert "upper_bound, 0.0" not in proof_call

    retained_start = SOURCE.index(
        "void evaluate_overtake_canonical_retained_shadow("
    )
    retained_end = SOURCE.index(
        "void invalidate_overtake_canonical_async_context()", retained_start
    )
    retained = SOURCE[retained_start:retained_end]
    assert (
        "proof_request.required_wall_clearance_m =\n"
        "      problem.progress_execution_required_wall_clearance_m"
    ) in retained


def test_overtake_intents_use_the_rate_resolved_normal_owner() -> None:
    """ShiftOut/Pass/Return must use the same six-state owner as racing."""

    control_start = SOURCE.index("MpcControlCycleResult get_control(")
    control_end = SOURCE.index(
        "std::pair<std::vector<double>, std::vector<double>> update_prediction(",
        control_start,
    )
    overtake_start = SOURCE.index(
        "rate_resolved_artifact::supports_intent(control_intent)", control_start
    )
    overtake_end = SOURCE.index(
        "if (unresolved_dynamic_wait_canonical_scope())", overtake_start
    )
    overtake = SOURCE[overtake_start:overtake_end]
    assert (
        "return rate_resolved_normal_production_control(\n"
        "          problem, now_sec, control_intent);"
    ) in overtake
    assert "canonical_overtake_production_control(" not in SOURCE[control_start:control_end]
    assert "canonical_normal_control(" not in overtake
    assert "solve_problem(" not in overtake

    owner_start = SOURCE.index("rate_resolved_normal_production_control(")
    owner_end = SOURCE.index("MpcControlCycleResult get_control(", owner_start)
    owner = SOURCE[owner_start:owner_end]
    consume = owner.index("evaluate_rate_resolved_track_cruise_retained_shadow(")
    resolve = owner.index("rate_resolved_track_cruise_control(")
    bind = owner.index("bind_rate_resolved_track_cruise_submission(")
    submit = owner.index("submit_rate_resolved_track_cruise_shadow(")
    assert consume < resolve < bind < submit
    assert "VelocityProgress5State" not in owner


def test_follow_uses_the_shared_rate_resolved_normal_owner() -> None:
    """Follow may change intent, but it may not change normal formulation."""

    artifact_source = (
        PACKAGE_ROOT / "src/mpcc_rate_resolved_execution_artifact.cpp"
    ).read_text(encoding="utf-8")
    supports_start = artifact_source.index("bool supports_intent(")
    supports_end = artifact_source.index("bool identity_valid(", supports_start)
    supports = artifact_source[supports_start:supports_end]
    assert "ControlIntent::Follow" in supports

    control_start = SOURCE.index("MpcControlCycleResult get_control(")
    control_end = SOURCE.index(
        "std::pair<std::vector<double>, std::vector<double>> update_prediction(",
        control_start,
    )
    dispatch = SOURCE[control_start:control_end]
    assert "rate_resolved_artifact::supports_intent(control_intent)" in dispatch
    assert "evaluate_follow_async_shadow(" not in dispatch
    assert "evaluate_follow_transition_admission(" not in dispatch


def test_five_state_follow_owner_is_physically_deleted() -> None:
    """The retired Follow worker/store/solver must not remain reconnectable."""

    retired_symbols = (
        "follow_canonical_lifecycle_",
        "follow_canonical_async_mailbox_",
        "follow_canonical_async_worker_",
        "evaluate_follow_async_shadow(",
        "evaluate_follow_transition_admission(",
        "submit_follow_canonical_async(",
        "invalidate_follow_canonical_async_context(",
        "record_follow_canonical_async_status(",
    )
    assert not [symbol for symbol in retired_symbols if symbol in SOURCE]


def test_rate_resolved_normal_snapshot_is_submitted_after_output_commit() -> None:
    """Every six-state intent must inherit the command committed this cycle."""

    builder_start = SOURCE.index(
        "build_rate_resolved_track_cruise_submission_draft("
    )
    builder_end = SOURCE.index(
        "bind_rate_resolved_track_cruise_submission(", builder_start
    )
    builder = SOURCE[builder_start:builder_end]
    assert "submit_rate_resolved_track_cruise_shadow(" not in builder
    assert "build_extended_progress_problem(" in builder
    assert "solve_extended_progress_problem(" not in builder

    owner_start = SOURCE.index("rate_resolved_normal_production_control(")
    owner_end = SOURCE.index("MpcControlCycleResult get_control(", owner_start)
    owner = SOURCE[owner_start:owner_end]

    consume = owner.index("evaluate_rate_resolved_track_cruise_retained_shadow(")
    resolve = owner.index("rate_resolved_track_cruise_control(")
    bind = owner.index("bind_rate_resolved_track_cruise_submission(")
    submit = owner.index("submit_rate_resolved_track_cruise_shadow(")
    assert consume < resolve < bind < submit


def test_rate_resolved_intent_transition_admits_the_same_six_state_producer() -> None:
    """An intent change cannot publish a solver result before live revalidation."""

    admission_start = SOURCE.index(
        "evaluate_rate_resolved_transition_admission("
    )
    admission_end = SOURCE.index(
        "RateResolvedRetainedShadowEvaluation\n"
        "  evaluate_rate_resolved_track_cruise_retained_shadow(",
        admission_start,
    )
    admission = SOURCE[admission_start:admission_end]
    assert "bind_rate_resolved_track_cruise_submission(draft, previous_steering)" in admission
    assert "build_rate_resolved_submission_snapshot(" in admission
    assert "evaluate_rate_resolved_pipeline(" in admission
    assert "rate_resolved_track_cruise_certified_plan_store_->snapshot()" in admission
    assert "VelocityProgress5State" not in admission
    assert "canonical_normal_control(" not in admission
    assert "canonical_normal_emergency_stop(" not in admission
    assert "safe_failure_control(" not in admission

    owner_start = SOURCE.index("rate_resolved_normal_production_control(")
    owner_end = SOURCE.index("MpcControlCycleResult get_control(", owner_start)
    owner = SOURCE[owner_start:owner_end]
    first_revalidation = owner.index(
        "evaluate_rate_resolved_track_cruise_retained_shadow("
    )
    admission_call = owner.index("evaluate_rate_resolved_transition_admission(")
    second_revalidation = owner.index(
        "evaluate_rate_resolved_track_cruise_retained_shadow(",
        first_revalidation + 1,
    )
    publication = owner.index("rate_resolved_track_cruise_control(")
    assert first_revalidation < admission_call < second_revalidation < publication
    assert "!retained.production_authority.has_value()" in owner
    assert "ControlIntent::Unknown" in owner
    assert "last_published_canonical_intent_ != intent" in owner
    assert "retained.reason == rate_resolved_retained::Reason::MissingPlan" not in owner
    assert "retained.reason == rate_resolved_retained::Reason::IntentMismatch" not in owner
    assert "if (admission.certified)" in owner


def test_rate_resolved_track_cruise_uses_explicit_control_time_origin() -> None:
    """Latency-predicted state, cursor, and dynamic prefix share one clock."""

    assert (
        "draft.control_prediction_origin_sec =\n"
        "      now_sec + execution_prediction_delay_sec_;"
        in SOURCE
    )
    assert (
        "snapshot.control_prediction_origin_sec =\n"
        "      bound_submission.control_prediction_origin_sec;"
        in SOURCE
    )
    assert "request.control_origin_sec = now_sec + control_path->duration_sec;" in SOURCE
    assert "request.measured_to_control_elapsed_sec = control_path->elapsed_sec;" in SOURCE


def test_rate_resolved_track_cruise_has_its_own_six_state_problem_identity() -> None:
    """A six-state artifact must not inherit the five-state fingerprint."""

    builder_start = SOURCE.index(
        "build_rate_resolved_track_cruise_submission_draft("
    )
    builder_end = SOURCE.index(
        "bind_rate_resolved_track_cruise_submission(", builder_start
    )
    builder = SOURCE[builder_start:builder_end]
    assert (
        "draft.source_context = make_problem_context(\n"
        "      problem,\n"
        "      mpcc_contract::Formulation::VelocitySteeringProgress6State);"
        in builder
    )
    assert "VelocityProgress5State" not in builder


def test_dynamic_escape_materializes_one_canonical_overtake_identity() -> None:
    """Validated Dynamic Escape may not lose target/attempt/side at MPCC entry."""

    assert "resolve_canonical_execution_identity(" in SOURCE
    assert (
        "progress_contouring_active && canonical_execution_identity.active"
        in SOURCE
    )
    assert (
        "canonical_execution_identity.target_id" in SOURCE
        and "canonical_execution_identity.generation" in SOURCE
        and "canonical_execution_identity.side_sign" in SOURCE
    )
    assert (
        "behavior_output.dynamic_obstacle_lateral_escape_attempt_id"
        in SOURCE
    )
    assert (
        "behavior_output.dynamic_obstacle_lateral_escape_execution_path_validated"
        in SOURCE
    )


def test_follow_transition_admission_uses_the_same_canonical_producer() -> None:
    """Intent elevation must be atomic with a current executable six-state plan."""

    admission_start = SOURCE.index("evaluate_rate_resolved_transition_admission(")
    admission_end = SOURCE.index(
        "evaluate_rate_resolved_track_cruise_retained_shadow(", admission_start
    )
    admission = SOURCE[admission_start:admission_end]
    assert "build_rate_resolved_submission_snapshot(" in admission
    assert "evaluate_rate_resolved_pipeline(" in admission
    assert "rate_resolved_track_cruise_certified_plan_store_" in admission
    assert "VelocityProgress5State" not in admission
    assert "legacy-mpc" not in admission

    owner_start = SOURCE.index("rate_resolved_normal_production_control(")
    owner_end = SOURCE.index("MpcControlCycleResult get_control(", owner_start)
    owner = SOURCE[owner_start:owner_end]
    admission_call = owner.index("evaluate_rate_resolved_transition_admission(")
    retained_join = owner.index(
        "evaluate_rate_resolved_track_cruise_retained_shadow(", admission_call
    )
    assert admission_call < retained_join
    assert "last_published_canonical_intent_" in owner
    assert "ControlIntent::Follow" in owner


def test_shared_rate_resolved_solver_transaction_is_serialized_end_to_end() -> None:
    """Async refresh and transition admission share one serialized solver owner."""

    shadow_header = (
        PACKAGE_ROOT / "include/multi_purpose_mpc_ros/mpcc_rate_resolved_shadow.hpp"
    ).read_text(encoding="utf-8")
    shadow_source = (
        PACKAGE_ROOT / "src/mpcc_rate_resolved_shadow.cpp"
    ).read_text(encoding="utf-8")
    context_start = shadow_header.index("class SolverContext")
    context_end = shadow_header.index("enum class PublishReason", context_start)
    assert "std::mutex mutex_" in shadow_header[context_start:context_end]
    evaluate_start = shadow_source.index("Result SolverContext::evaluate(")
    evaluate_end = shadow_source.index("const char * to_string(", evaluate_start)
    assert "std::lock_guard<std::mutex> lock(mutex_)" in shadow_source[
        evaluate_start:evaluate_end
    ]

    submit_start = SOURCE.index("submit_rate_resolved_track_cruise_shadow(")
    submit_end = SOURCE.index(
        "evaluate_rate_resolved_transition_admission(", submit_start
    )
    admission_end = SOURCE.index(
        "evaluate_rate_resolved_track_cruise_retained_shadow(", submit_end
    )
    assert "rate_resolved_track_cruise_shadow_solver_context_" in SOURCE[
        submit_start:submit_end
    ]
    assert "rate_resolved_track_cruise_shadow_solver_context_" in SOURCE[
        submit_end:admission_end
    ]


def test_overtake_entry_preserves_the_selected_tactical_artifact() -> None:
    """ShiftOut keeps branch identity/proof without installing old authority."""

    branch_start = SOURCE.index("evaluate_extended_mpcc_branch(")
    branch_end = SOURCE.index(
        "evaluate_isolated_extended_mpcc_branch(", branch_start
    )
    branch = SOURCE[branch_start:branch_end]
    solve = branch.index("solve_extended_progress_problem(")
    target_proof = branch.index("build_current_overtake_target_tube(")
    canonical = branch.index("evaluate_overtake_canonical_fresh_shadow(")
    assert solve < target_proof < canonical
    assert "legacy.progress_execution_target_exclusion_certified = true" in branch
    assert "preentry_canonical_plan" in branch

    entry_start = SOURCE.index("const bool fresh_normal_mission_entry =")
    entry_end = SOURCE.index(
        "transition_overtake_line_phase(\n"
        "          direct_pass ? OvertakeLinePhase::Pass",
        entry_start,
    )
    entry = SOURCE[entry_start:entry_end]
    assert "resolve_overtake_preentry_plan(" in entry
    assert "freeze_selected_overtake_mission(" in entry
    assert "prepare_overtake_canonical_async_context(" not in entry
    assert "plan_store.replace(" not in entry

    certificate_start = SOURCE.index(
        "bool revalidate_overtake_entry_execution_certificate("
    )
    certificate_end = SOURCE.index(
        "bool dynamic_margin_escape_solution_wall_safe(", certificate_start
    )
    certificate = SOURCE[certificate_start:certificate_end]
    assert "const canonical_plan::CanonicalExecutionPlan & preentry_plan" in certificate
    assert "sample_retained_progress_advance(" in certificate
    assert "preentry_plan.predicted_states" in certificate
    assert "&plan_time_sec" in certificate
    assert "build_current_overtake_target_tube(" in certificate
    assert "validate_frenet_dp_target_bound_horizon(" in certificate

    retained_start = SOURCE.index(
        "void evaluate_overtake_canonical_retained_shadow("
    )
    retained_end = SOURCE.index(
        "void invalidate_overtake_canonical_async_context()", retained_start
    )
    retained = SOURCE[retained_start:retained_end]
    assert "sample_retained_progress_advance(" in retained
    assert "build_retained_lateral_corridor(" in retained
    assert (
        "corridor.lateral_lower_m = retained_lateral_corridor->lower_m"
        in retained
    )
    assert (
        "corridor.lateral_upper_m = retained_lateral_corridor->upper_m"
        in retained
    )
    assert "problem.progress_state_lower[" not in retained
    assert "problem.progress_state_upper[" not in retained
    assert "&corridor.elapsed_time_sec" in retained
    assert (
        "corridor_path_distance_m.push_back(\n"
        "        problem.progress_stage_geometry"
    ) not in retained
    assert "intersect_overtake_target_tube(" in retained
    assert "stage_corridor_mpc_target_bound_was_active_ ||" not in retained

    fresh_start = SOURCE.index(
        "OvertakeCanonicalFreshShadowResult\n"
        "  evaluate_overtake_canonical_fresh_shadow("
    )
    fresh_end = SOURCE.index(
        "evaluate_overtake_canonical_worker_fresh(", fresh_start
    )
    fresh = SOURCE[fresh_start:fresh_end]
    assert "extraction.lateral_lower_m.push_back(" in fresh
    assert "extraction.lateral_upper_m.push_back(" in fresh


def test_five_state_preentry_artifact_cannot_gate_rate_resolved_actuation() -> None:
    """The tactical five-state artifact is identity/proof, not command authority."""

    async_source = (
        PACKAGE_ROOT / "src/follow_canonical_async.cpp"
    ).read_text(encoding="utf-8")
    resolver_start = async_source.index("resolve_overtake_preentry_plan(")
    resolver_end = async_source.index(
        "const char * to_string(const PublishReason", resolver_start
    )
    resolver = async_source[resolver_start:resolver_end]
    assert "resolve_execution_cursor(" in resolver
    assert "extract_canonical_actuation(" not in resolver
    assert "certify_canonical_steering_continuity(" not in resolver

    entry_start = SOURCE.index("const bool fresh_normal_mission_entry =")
    entry_end = SOURCE.index(
        "transition_overtake_line_phase(\n"
        "          direct_pass ? OvertakeLinePhase::Pass",
        entry_start,
    )
    entry = SOURCE[entry_start:entry_end]
    assert "resolve_overtake_preentry_plan(" in entry
    assert "steering_continuity" not in entry
    assert "prepare_overtake_canonical_async_context(" not in entry
    assert "overtake_canonical_lifecycle_->plan_store.replace(" not in entry


def test_runtime_overtake_replacement_is_a_typed_canonical_artifact() -> None:
    """Runtime Mission replacement may not drop the selected five-state plan."""

    artifact_start = SOURCE.index("struct OvertakeExecutionArtifact")
    artifact_end = SOURCE.index("struct V2XBehaviorOutput", artifact_start)
    artifact = SOURCE[artifact_start:artifact_end]
    assert "OvertakeMissionCandidate mission" in artifact
    assert "CanonicalExecutionPlan> canonical_plan" in artifact
    assert "make_overtake_execution_artifact(" in artifact

    behavior_start = SOURCE.index("struct V2XBehaviorOutput")
    behavior_end = SOURCE.index("struct ExtendedMpccBranchArtifact", behavior_start)
    behavior = SOURCE[behavior_start:behavior_end]
    assert "mpcc_lite_same_side_replan_artifact" in behavior
    assert "mpcc_lite_cross_side_replan_artifact" in behavior
    assert "mpcc_lite_same_side_replan_mission" not in behavior
    assert "mpcc_lite_cross_side_replan_mission" not in behavior

    branch_start = SOURCE.index("evaluate_extended_mpcc_branch(")
    branch_end = SOURCE.index(
        "evaluate_isolated_extended_mpcc_branch(", branch_start
    )
    branch = SOURCE[branch_start:branch_end]
    assert (
        "artifact_identity.source_mission_generation + 1U" in branch
    )
    assert "overtake_line_state_.mission_generation + 1U" not in branch

    selector_start = SOURCE.index(
        "void evaluate_and_select_extended_mpcc_branches("
    )
    selector_end = SOURCE.index(
        "bool submit_mpcc_lite_async_snapshot(", selector_start
    )
    selector = SOURCE[selector_start:selector_end]
    assert "const OvertakeArtifactIdentitySeed & artifact_identity" in selector
    assert "artifact_identity.source_side_sign" in selector
    assert "artifact_identity.source_phase" in selector

    worker_start = SOURCE.index("const auto submission = mpcc_lite_async_worker_->submit_latest(")
    worker_end = SOURCE.index("mpcc_lite_async_last_snapshot_ms_", worker_start)
    worker = SOURCE[worker_start:worker_end]
    assert "const OvertakeArtifactIdentitySeed artifact_identity{" in worker
    assert (
        "evaluate_and_select_extended_mpcc_branches(\n"
        "            result.behavior, horizon_size, now_sec, artifact_identity)"
        in worker
    )

    replace_start = SOURCE.index(
        "bool replace_frozen_overtake_mission_after_dynamic_replan("
    )
    replace_end = SOURCE.index("double mission_completion_reserve_sec() const", replace_start)
    replace = SOURCE[replace_start:replace_end]
    assert "replacement_canonical_plan" in replace
    assert "resolve_overtake_preentry_plan(" in replace
    assert "prospective_generation" in replace
    assert "adopt_overtake_canonical_plan_context(" in replace
    assert "prepare_overtake_canonical_async_context(" not in replace
    assert replace.index("freeze_selected_overtake_mission(") < replace.index(
        "adopt_overtake_canonical_plan_context("
    )
    assert replace.index("adopt_overtake_canonical_plan_context(") < replace.index(
        "transition_overtake_line_phase("
    )

    adoption_start = SOURCE.index(
        "OvertakeCanonicalPlanAdoption adopt_overtake_canonical_plan_context("
    )
    adoption_end = SOURCE.index(
        "bool submit_overtake_canonical_async(", adoption_start
    )
    adoption = SOURCE[adoption_start:adoption_end]
    assert adoption.index("plan_store.replace(") < adoption.index(
        "overtake_canonical_async_context_ = resolution.next"
    )
    assert "plan_store.clear(" not in adoption

    runtime_start = SOURCE.index(
        "behavior_output.mpcc_lite_same_side_replan_artifact.has_value()"
    )
    runtime_end = SOURCE.index(
        "behavior_output.opponent_side_replan_ready", runtime_start
    )
    runtime = SOURCE[runtime_start:runtime_end]
    assert "artifact.canonical_plan" in runtime
    assert "locked_target_observation_generation" in runtime


def test_overtake_preentry_target_prediction_is_an_explicit_snapshot_contract() -> None:
    """Idle-side workers cannot depend on a committed target that does not exist yet."""

    behavior_start = SOURCE.index("struct V2XBehaviorOutput")
    behavior_end = SOURCE.index("struct ExtendedMpccBranchArtifact", behavior_start)
    behavior = SOURCE[behavior_start:behavior_end]
    assert "target_execution_prediction_valid" in behavior
    assert "target_execution_predicted_longitudinal" in behavior
    assert "target_execution_predicted_lateral" in behavior

    selection_start = SOURCE.index("const auto set_target_execution_prediction =")
    selection_end = SOURCE.index(
        "output.overtake_entry_target_speed =", selection_start
    )
    selection = SOURCE[selection_start:selection_end]
    assert "nearest_front_course_lateral" in selection
    assert "nearest_front_lateral_velocity_valid" in selection
    assert "target_execution_prediction_valid" in selection

    tube_start = SOURCE.index("CurrentOvertakeTargetTube build_current_overtake_target_tube(")
    tube_end = SOURCE.index(
        "bool revalidate_overtake_entry_execution_certificate(", tube_start
    )
    tube = SOURCE[tube_start:tube_end]
    assert "committed_target_prediction_available" in tube
    assert "entry_target_prediction_available" in tube
    assert "behavior.target_execution_predicted_lateral" in tube
    assert "behavior.target_execution_predicted_longitudinal" in tube



def test_unresolved_dynamic_wait_cannot_fall_through_to_legacy_normal() -> None:
    """A broken ShiftOut/Pass wait handoff must fail closed, not change authority."""

    scope_start = SOURCE.index("unresolved_dynamic_wait_canonical_scope() const")
    scope_end = SOURCE.index(
        "void invalidate_overtake_canonical_async_context()", scope_start
    )
    scope = SOURCE[scope_start:scope_end]
    assert "authority.request.dynamic_wait_active" in scope
    assert (
        "authority.resolution.action == overtake_orchestrator::Action::DynamicWait"
        in scope
    )

    control_start = SOURCE.index("MpcControlCycleResult get_control(")
    control_end = SOURCE.index(
        "std::pair<std::vector<double>, std::vector<double>> update_prediction(",
        control_start,
    )
    fail_closed = SOURCE.index(
        "if (unresolved_dynamic_wait_canonical_scope())", control_start
    )
    before_old_path = SOURCE[fail_closed:control_end]
    assert "return canonical_normal_emergency_stop(" in before_old_path
    assert "dynamic wait has no executable canonical lateral authority" in before_old_path
    assert "solve_problem(" not in before_old_path


def test_rejoin_uses_isolated_canonical_production_without_legacy_fallthrough() -> None:
    """Qualified Rejoin must publish canonical or Emergency, never legacy normal."""

    activation_start = SOURCE.index(
        "const bool progress_contouring_execution_phase ="
    )
    activation_end = SOURCE.index(
        "const auto track_cruise_shadow_eligibility", activation_start
    )
    activation = SOURCE[activation_start:activation_end]
    assert "OvertakeLinePhase::Recovery" not in activation

    evaluator_start = SOURCE.index("evaluate_rejoin_canonical(")
    evaluator_end = SOURCE.index(
        "resolve_physically_validated_mpcc_execution_trajectory(", evaluator_start
    )
    evaluator = SOURCE[evaluator_start:evaluator_end]
    assert "rejoin_shadow_plan_store_" in evaluator
    assert "rejoin_shadow_warm_start_identity_" in evaluator
    assert "rejoin_shadow_solver_context_" in evaluator
    assert "problem.rejoin_shadow_requested" in evaluator
    assert "track_cruise" not in evaluator
    assert "Rejoin retained policy intentionally unavailable" in evaluator

    control_start = SOURCE.index("MpcControlCycleResult get_control(")
    rejoin_shadow = SOURCE.index(
        "if (control_intent == mpcc_contract::ControlIntent::Rejoin)",
        control_start,
    )
    control_end = SOURCE.index(
        "std::pair<std::vector<double>, std::vector<double>> update_prediction(",
        control_start,
    )
    observation = SOURCE[rejoin_shadow:control_end]
    assert "evaluate_rejoin_canonical(" in observation
    assert "record_rejoin_canonical_telemetry" in observation
    assert "return canonical_normal_control(" in observation
    assert "return canonical_normal_emergency_stop(" in observation
    assert "canonical_result.selected.complete()" in observation
    assert "legacy command" not in observation
    assert "solve_problem(" not in observation

    telemetry_start = SOURCE.index("void record_rejoin_canonical_telemetry(")
    telemetry_end = SOURCE.index("void record_overtake_canonical", telemetry_start)
    telemetry = SOURCE[telemetry_start:telemetry_end]
    assert "production_authority=canonical" in telemetry
    assert "authority=production" in telemetry
    assert "legacy-unchanged" not in telemetry


def test_extended_first_stage_linearization_is_anchored_to_the_execution_state() -> None:
    """The first affine dynamics block must be tangent at the fixed state zero."""

    builder_start = SOURCE.index(
        "std::optional<ExtendedProgressMpcProblem> build_extended_progress_problem("
    )
    builder_end = SOURCE.index(
        "relinearize_extended_progress_wall_bounds(", builder_start
    )
    builder = SOURCE[builder_start:builder_end]
    assert "const bool initial_stage = stage == 0;" in builder
    assert "initial_stage ? initial_frenet_pose->lateral_m" in builder
    assert "initial_stage ? initial_lag_m" in builder
    assert re.search(
        r"initial_stage\s*\?\s*initial_frenet_pose->heading_offset_rad", builder
    )
    assert re.search(
        r"initial_stage\s*\?\s*legacy\.progress_measured_speed_mps", builder
    )
    assert "initial_stage ? previous_curvature" in builder
    assert "legacy.progress_stage_dt_sec[static_cast<std::size_t>(stage)]" in builder


def test_canonical_overtake_wall_certificate_is_not_reinterpreted_downstream() -> None:
    """A certified canonical command has no downstream normal wall owner."""

    assert "const bool active_overtake_wall_monitor_relevant =" not in SOURCE
    assert "resolve_legacy_wall_handoff_authority(" not in SOURCE
    assert "const bool wall_handoff_hold_active =" not in SOURCE
    # The wall proof inside the canonical controller boundary is retained.
    assert "executed_solution_wall_hold_active" in SOURCE
    # Wall telemetry remains observational and cannot modify the command.
    assert "maybe_emit_dynamic_escape_wall_handoff_trace(" in SOURCE


def test_overtake_physical_failure_preserves_stage_and_authority_provenance() -> None:
    """A hard physical rejection must identify both proof and plan producers."""

    horizon_start = SOURCE.index("struct OvertakeLineHorizonEvaluation")
    horizon_end = SOURCE.index(
        "enum class OvertakeRecedingHorizonFailureKind", horizon_start
    )
    horizon = SOURCE[horizon_start:horizon_end]
    assert "physical_failure_cause" in horizon
    assert "physical_failure_path_distance_m" in horizon
    assert "physical_failure_lateral_target_m" in horizon
    assert "physical_failure_wall_lower_m" in horizon
    assert "physical_failure_wall_upper_m" in horizon
    assert "physical_failure_required_lateral_accel_mps2" in horizon

    validation_start = SOURCE.index("const auto record_validation_failure =")
    validation_end = SOURCE.index(
        "if (!validation_accepted) {", validation_start
    )
    validation = SOURCE[validation_start:validation_end]
    assert "physical_validation_attempt_count" in validation
    assert "candidate_speed_mps" in validation
    assert "wall_clearance_m" in validation
    assert "initial_contract" in validation

    trace_start = SOURCE.index("OvertakeLine physical authority failure:")
    trace_end = SOURCE.index(
        "const bool rear_clear_return_handoff", trace_start
    )
    trace = SOURCE[trace_start:trace_end]
    for token in (
        "generation=%lu",
        "cause=%s",
        "stage=%d",
        "wall=[%.3f,%.3f]",
        "initial_contract=%d",
        "attempts=%zu",
        "dp=%d/authority=%d/runtime=%d/side=%d/source_age=%.3f",
        "canonical=%d/plan=%lu/generation=%lu/side=%d/fingerprint=%lu/",
    ):
        assert token in trace


def test_canonical_overtake_demotes_legacy_mission_viability_owner() -> None:
    """A complete reference may feed canonical MPCC but cannot end its Mission."""

    resolution_start = SOURCE.index(
        "const auto receding_viability_authority ="
    )
    transition_start = SOURCE.index(
        "if (!horizon_evaluation.execution_feasible())", resolution_start
    )
    transition_end = SOURCE.index(
        "double previous_target_ey = current_ey;", transition_start
    )
    transition = SOURCE[transition_start:transition_end]

    assert "receding_viability_reference_only" in transition
    assert "OvertakeLine legacy viability demoted:" in transition
    demotion_start = transition.index("if (receding_viability_reference_only)")
    legacy_start = transition.index(
        "should_terminate_recovery_retained_mission", demotion_start
    )
    demotion = transition[demotion_start:legacy_start]
    assert "transition_overtake_line_phase" not in demotion
    assert "enter_dynamic_mission_wait" not in demotion
    assert "arm_overtake_line_side_retry_block" not in demotion
    assert "reset_overtake_line_state" not in demotion

    speed_cap_start = SOURCE.index(
        "horizon_evaluation.static_map_wall_infeasible", resolution_start
    )
    speed_cap_end = SOURCE.index(
        "if (!horizon_evaluation.execution_feasible())", speed_cap_start
    )
    assert "!receding_viability_reference_only" in SOURCE[
        speed_cap_start:speed_cap_end
    ]

    trace_start = SOURCE.index("OvertakeLine physical authority failure:")
    trace_end = SOURCE.index(
        "const bool rear_clear_return_handoff", trace_start
    )
    trace = SOURCE[trace_start:trace_end]
    assert "viability=%s/%s/reference_complete=%d" in trace


def test_runtime_wall_preplanner_cannot_destroy_canonical_mission() -> None:
    """An optional wall-reference producer has no FSM or Recovery authority."""

    assert "RuntimeWallPreplanAction::ExitCurrentMission" not in SOURCE
    prefix_start = SOURCE.index(
        "OvertakeLine runtime wall escape prefix commit rejected:"
    )
    prefix_end = SOURCE.index(
        "const auto transition_action =", prefix_start
    )
    prefix_failure = SOURCE[prefix_start:prefix_end]

    assert "enter_dynamic_mission_wait" not in prefix_failure
    assert "transition_overtake_line_phase" not in prefix_failure
    assert "reset_overtake_line_state" not in prefix_failure
    assert "canonical-reference-only" in prefix_failure


def test_rate_resolved_worker_is_observation_only_but_retained_proof_owns_control() -> None:
    """Only a current-world-qualified retained six-state proof may own control."""

    submit_start = SOURCE.index(
        "bool submit_rate_resolved_track_cruise_shadow("
    )
    evaluator_start = SOURCE.index(
        "CanonicalRejoinCycleResult evaluate_rejoin_canonical("
    )
    transport = SOURCE[submit_start:evaluator_start]
    assert "rate_resolved_track_cruise_shadow_worker_->submit_latest(" in transport
    assert "rate_resolved_track_cruise_shadow_mailbox_->latest_after(" in transport
    assert "authority=shadow, selected=0" in transport
    assert "last_failure_result" in transport
    assert (
        "result->outcome != rate_resolved_shadow::Outcome::Solved" in transport
    )
    failure_trace_start = transport.index(
        "Rate-resolved Track/Cruise shadow failure:"
    )
    failure_trace = transport[failure_trace_start:]
    assert "authority=shadow, selected=0" in failure_trace
    for forbidden in (
        "canonical_normal_control(",
        "canonical_normal_emergency_stop(",
        "track_cruise_shadow_plan_store_.publish(",
        "record_solution_contract(",
        "previous_steering =",
        "current_control =",
    ):
        assert forbidden not in transport

    branch_start = SOURCE.index("rate_resolved_normal_production_control(")
    branch_end = SOURCE.index("MpcControlCycleResult get_control(", branch_start)
    branch = SOURCE[branch_start:branch_end]
    assert "record_rate_resolved_track_cruise_shadow(problem, now_sec, retained);" in branch
    assert "record_rate_resolved_track_cruise_command(" in branch
    assert "auto retained" in branch
    assert "rate_resolved_track_cruise_control(" in branch
    assert "canonical_result.selected.complete()" not in branch
    assert "output = canonical_normal_control(" not in branch
    assert "output = canonical_normal_emergency_stop(" not in branch
    assert "return output;" in branch
    assert "output.control =" not in branch
    assert "output.canonical_normal_command =" not in branch

    shadow_header = (
        Path(__file__).resolve().parents[1]
        / "include"
        / "multi_purpose_mpc_ros"
        / "mpcc_rate_resolved_shadow.hpp"
    ).read_text(encoding="utf-8")
    artifact_header = (
        Path(__file__).resolve().parents[1]
        / "include"
        / "multi_purpose_mpc_ros"
        / "mpcc_rate_resolved_execution_artifact.hpp"
    ).read_text(encoding="utf-8")
    assert (
        "std::shared_ptr<const artifact::ExecutionArtifact> execution_artifact"
        in shadow_header
    )
    for header in (shadow_header, artifact_header):
        assert "CanonicalExecutionPlanStore" not in header
        assert "CanonicalNormalCommand" not in header
        assert "publish_control" not in header


def test_rate_resolved_shadow_replaces_legacy_first_curvature_time_base() -> None:
    """Six-state reachability must not inherit the old one-cycle curvature patch."""

    builder_start = SOURCE.index(
        "std::optional<ExtendedProgressMpcProblem> build_extended_progress_problem("
    )
    builder_end = SOURCE.index(
        "relinearize_extended_progress_wall_bounds(", builder_start
    )
    builder = SOURCE[builder_start:builder_end]
    semantic_start = builder.index("rate_resolved_shadow_request.emplace();")
    semantic_end = builder.index("Eigen::VectorXd q =", semantic_start)
    semantic = builder[semantic_start:semantic_end]
    assert "request.current_steering_rad = previous_steering;" in semantic
    assert "request.maximum_abs_steering_rate_radps" in semantic
    assert "first_curvature_input_lower" in semantic
    assert "first_curvature_input_upper" in semantic
    assert "physical_first_curvature->reachable_lower_radpm" not in semantic
    assert "physical_first_curvature->reachable_upper_radpm" not in semantic


def test_rate_resolved_physical_wall_proof_is_shared_but_cannot_publish() -> None:
    """Async and atomic admission share one proof pipeline with no publisher."""

    proof_start = SOURCE.index(
        "build_rate_resolved_track_cruise_physical_snapshot("
    )
    submit_start = SOURCE.index(
        "bool submit_rate_resolved_track_cruise_shadow(", proof_start
    )
    record_start = SOURCE.index(
        "void record_rate_resolved_track_cruise_shadow(", submit_start
    )
    shared_start = SOURCE.index(
        "RateResolvedPipelineEvaluation evaluate_rate_resolved_pipeline("
    )
    shared_end = SOURCE.index("struct CanonicalCurrentControlPath", shared_start)
    snapshot_builder = SOURCE[proof_start:submit_start]
    pipeline = SOURCE[submit_start:record_start]
    shared_pipeline = SOURCE[shared_start:shared_end]
    assert "snapshot.identity.artifact = solver_snapshot.identity;" in snapshot_builder
    assert "problem.progress_stage_geometry" in snapshot_builder
    assert "build_progress_course_frame_knots(" in snapshot_builder
    assert "fingerprint_control_pose_path(" in snapshot_builder
    assert "fingerprint_course_frame_window(" in snapshot_builder
    assert "rate_resolved_physical::build(" in shared_pipeline
    assert "rate_resolved_physical_wall::evaluate(" in shared_pipeline
    assert "certified_plan_store->certify_and_replace(" in shared_pipeline
    assert shared_pipeline.index(
        "rate_resolved_physical_wall::evaluate("
    ) < shared_pipeline.index(
        "certified_plan_store->certify_and_replace("
    )
    assert "evaluate_rate_resolved_pipeline(" in pipeline
    assert pipeline.count(
        "rate_resolved_track_cruise_shadow_worker_->submit_latest("
    ) == 1
    assert "rate_resolved_track_cruise_physical_wall_worker_" not in SOURCE
    assert "solved_mpcc_execution_path_wall_safe(" not in snapshot_builder
    assert "solved_mpcc_execution_path_wall_safe(" not in pipeline
    for forbidden in (
        "canonical_normal_control(",
        "CanonicalExecutionPlanStore",
        "publish_control_command(",
        "track_cruise_shadow_plan_store_.publish(",
        "record_solution_contract(",
    ):
        assert forbidden not in snapshot_builder
        assert forbidden not in pipeline
        assert forbidden not in shared_pipeline

    adapter_header = (
        Path(__file__).resolve().parents[1]
        / "include"
        / "multi_purpose_mpc_ros"
        / "mpcc_rate_resolved_physical_adapter.hpp"
    ).read_text(encoding="utf-8")
    for forbidden in (
        "CanonicalExecutionPlanStore",
        "CanonicalNormalCommand",
        "publish_control",
        "rclcpp",
    ):
        assert forbidden not in adapter_header

    wall_source = (
        Path(__file__).resolve().parents[1]
        / "src"
        / "mpcc_rate_resolved_physical_wall.cpp"
    ).read_text(encoding="utf-8")
    assert "recovery::sample_footprint(" in wall_source
    assert "recovery::evaluate_clear_footprint_path(" in wall_source
    for forbidden in (
        "CanonicalExecutionPlanStore",
        "CanonicalNormalCommand",
        "publish_control",
        "rclcpp",
    ):
        assert forbidden not in wall_source

    certified_header = (
        Path(__file__).resolve().parents[1]
        / "include"
        / "multi_purpose_mpc_ros"
        / "mpcc_rate_resolved_certified_plan.hpp"
    ).read_text(encoding="utf-8")
    assert "std::shared_ptr<const artifact::ExecutionArtifact>" in certified_header
    assert "physical::Identity physical_identity" in certified_header
    for forbidden in (
        "CanonicalExecutionPlanStore",
        "CanonicalNormalCommand",
        "publish_control",
        "rclcpp",
    ):
        assert forbidden not in certified_header


def test_rate_resolved_retained_current_world_path_is_shadow_only() -> None:
    """Retained proof evaluation may observe the world but cannot publish."""

    evaluate_start = SOURCE.index(
        "evaluate_rate_resolved_track_cruise_retained_shadow("
    )
    record_start = SOURCE.index(
        "void record_rate_resolved_track_cruise_shadow(", evaluate_start
    )
    evaluate = SOURCE[evaluate_start:record_start]
    assert "rate_resolved_track_cruise_certified_plan_store_->snapshot()" in evaluate
    assert "build_canonical_current_control_path()" in evaluate
    assert "gap_planner->dynamic_world_observation(now_sec)" in evaluate
    assert "dynamic_world.vehicles" in evaluate
    assert "rate_resolved_retained::evaluate(request)" in evaluate
    for forbidden in (
        "publish_control_command(",
        "publish_failsafe_command(",
        "canonical_normal_control(",
        "build_canonical_normal_command(",
        "track_cruise_shadow_plan_store_.publish(",
    ):
        assert forbidden not in evaluate

    package = Path(__file__).resolve().parents[1]
    retained_header = (
        package
        / "include"
        / "multi_purpose_mpc_ros"
        / "mpcc_rate_resolved_retained_revalidation.hpp"
    ).read_text(encoding="utf-8")
    retained_source = (
        package / "src" / "mpcc_rate_resolved_retained_revalidation.cpp"
    ).read_text(encoding="utf-8")
    assert "std::optional<Proof> proof" in retained_header
    assert "artifact::extract_actuation(execution, cursor)" in retained_source
    assert "recovery::evaluate_clear_footprint_path(" in retained_source
    assert "recovery::circle_obstacle_clearance_at_time(" in retained_source
    assert "request.current_wall_grid.get() != source.wall_grid.get()" in retained_source
    for text in (retained_header, retained_source):
        for forbidden in (
            "CanonicalNormalCommand",
            "publish_control",
            "rclcpp",
            "AckermannControlCommand",
        ):
            assert forbidden not in text

    snapshot_start = SOURCE.index(
        "DynamicWorldObservation dynamic_world_observation("
    )
    snapshot_end = SOURCE.index(
        "bool has_complete_message(", snapshot_start
    )
    snapshot = SOURCE[snapshot_start:snapshot_end]
    assert "std::lock_guard<std::mutex> lock(mutex_)" in snapshot
    assert "vehicle.observation_generation != observation_generation_" in snapshot
    assert "last_message_vehicle_ids_.size() != last_message_vehicle_count_" in snapshot


def test_racing_follow_and_overtake_have_only_rate_resolved_normal_owner() -> None:
    """Racing, Follow and Overtake must share one normal formulation."""

    branch_start = SOURCE.index("rate_resolved_normal_production_control(")
    branch_end = SOURCE.index("MpcControlCycleResult get_control(", branch_start)
    branch = SOURCE[branch_start:branch_end]
    assert "evaluate_rate_resolved_track_cruise_retained_shadow(" in branch
    assert "rate_resolved_track_cruise_control(" in branch
    assert "build_rate_resolved_track_cruise_submission_draft(" in branch
    assert "bind_rate_resolved_track_cruise_submission(" in branch
    assert "evaluate_canonical_normal_shadow(" not in branch
    assert "canonical_normal_control(" not in branch
    assert "CanonicalNormalShadowMode::TrackCruise" not in branch

    control_start = SOURCE.index("MpcControlCycleResult get_control(")
    control_end = SOURCE.index(
        "std::pair<std::vector<double>, std::vector<double>> update_prediction(",
        control_start,
    )
    dispatch = SOURCE[control_start:control_end]
    assert dispatch.count("return rate_resolved_normal_production_control(") == 1
    assert "rate_resolved_artifact::supports_intent(control_intent)" in dispatch
    assert "canonical_overtake_production_control(" not in dispatch

    assert SOURCE.count(
        "rate_resolved_artifact::request_scope_available("
    ) == 2
    assert "rate_resolved_racing_requested" not in SOURCE
    assert "rate_resolved_overtake_requested" not in SOURCE

    package = Path(__file__).resolve().parents[1]
    adapter_source = (
        package / "src" / "mpcc_rate_resolved_production_adapter.cpp"
    ).read_text(encoding="utf-8")
    assert "rate_resolved_command_candidate::build" not in adapter_source
    assert "candidate::build(retained_result)" in adapter_source
    assert "artifact::supports_intent(source_context.intent)" in adapter_source
    assert "bool track_or_cruise(" not in adapter_source
    assert "resolve_canonical_normal_authority(" in adapter_source
    assert "build_canonical_normal_command(" in adapter_source
    for forbidden in (
        "publish_control_command(",
        "AckermannControlCommand",
        "rclcpp",
        "VelocityProgress5State",
    ):
        assert forbidden not in adapter_source


def test_five_state_track_cruise_owner_is_physically_deleted() -> None:
    """The retired Track/Cruise owner must not remain reconnectable."""

    for retired_symbol in (
        "CanonicalNormalShadowMode",
        "track_cruise_shadow_solver_context_",
        "track_cruise_shadow_plan_store_",
        "track_cruise_shadow_warm_start_identity_",
        "track_cruise_shadow_context_epoch_",
        "track_cruise_shadow_telemetry_window_",
        "last_track_cruise_shadow_telemetry_log_sec_",
        "last_track_cruise_shadow_status_",
    ):
        assert not re.search(
            rf"(?<![A-Za-z0-9_]){re.escape(retired_symbol)}(?![A-Za-z0-9_])",
            SOURCE,
        )
    for retired_function in (
        "evaluate_track_cruise_retained_shadow(",
        "record_track_cruise_shadow_telemetry(",
    ):
        assert retired_function not in SOURCE

    rejoin_start = SOURCE.index("evaluate_rejoin_canonical(")
    rejoin_end = SOURCE.index(
        "resolve_physically_validated_mpcc_execution_trajectory(", rejoin_start
    )
    rejoin = SOURCE[rejoin_start:rejoin_end]
    assert "problem.rejoin_shadow_requested" in rejoin
    assert "rejoin_shadow_plan_store_" in rejoin
    assert "rejoin_shadow_solver_context_" in rejoin
    assert "track_cruise" not in rejoin


def test_get_control_has_no_legacy_normal_fallthrough() -> None:
    """Resolved normal intents must use canonical MPCC or explicit Emergency."""

    control_start = SOURCE.index("MpcControlCycleResult get_control(")
    control_end = SOURCE.index(
        "std::pair<std::vector<double>, std::vector<double>> update_prediction(",
        control_start,
    )
    control = SOURCE[control_start:control_end]
    for retired in (
        "Eigen::VectorXd dec;",
        "solve_problem(",
        "solve_extended_progress_problem(",
        "convert_extended_solution_to_legacy(",
        "Formulation::ProgressContouring3State",
        "Formulation::LegacySpatialMpc3State",
        '"progress-3state"',
        '"legacy-mpc"',
        '"legacy-mpc-solved"',
        '"extended-mpcc-solved"',
    ):
        assert retired not in control
    assert "canonical normal intent has no production owner" in control
    assert "ControlIntent::Track" in control
    assert "ControlIntent::Cruise" in control
    assert "ControlIntent::Rejoin" in control
    assert "return canonical_normal_emergency_stop(" in control

    assert "persistent_osqp::SolveOutcome solve_problem(" not in SOURCE
    assert "persistent_osqp_solver_" not in SOURCE
    assert "last_osqp_solution_" not in SOURCE
    assert "last_osqp_progress_contouring_mode_" not in SOURCE


def test_retired_three_state_normal_representations_are_physically_deleted() -> None:
    """Deleted normal solvers must leave no reconnectable schema or converter."""

    production = "\n".join(
        (
            SOURCE,
            MPCC_PROGRESS_HEADER,
            MPCC_PROGRESS_SOURCE,
            MPCC_EXECUTION_CONTRACT_HEADER,
            MPCC_EXECUTION_CONTRACT_SOURCE,
        )
    )
    forbidden = (
        "LegacySpatialMpc3State",
        "ProgressContouring3State",
        "convert_extended_solution_to_legacy(",
        '"legacy-spatial-mpc-3state"',
        '"progress-contouring-3state"',
        '"legacy-spatial-tracking-v1"',
        '"progress-contouring-v1"',
    )
    assert not [token for token in forbidden if token in production]


def test_retired_extended_formulation_switch_state_is_physically_deleted() -> None:
    """A deleted alternate normal formulation must leave no switch state."""

    for retired_symbol in (
        "ExtendedSolverCircuitBreaker",
        "ExtendedSolverReentryGate",
        "ExtendedModeHandoff",
        "extended_progress_circuit_breaker_",
        "extended_progress_reentry_gate_",
        "extended_mode_handoff_",
        "ExtendedMpccTelemetryWindow",
        "MpcRtiSqpTelemetryWindow",
        "record_extended_mpcc_telemetry(",
        "record_rti_sqp_telemetry(",
        "relinearize_progress_problem(",
    ):
        assert retired_symbol not in SOURCE
        assert retired_symbol not in MPCC_PROGRESS_HEADER
        assert retired_symbol not in V2X_OVERTAKE_HEADER
        assert retired_symbol not in V2X_OVERTAKE_SOURCE

    assert "bool solver_degraded{false};" not in V2X_OVERTAKE_HEADER
    assert "SolverDegraded" not in V2X_OVERTAKE_HEADER
    assert "SolverDegraded" not in V2X_OVERTAKE_SOURCE

    for retired_key in (
        "progress_contouring_extended_failure_cooldown_sec",
        "progress_contouring_extended_reentry_success_cycles",
        "progress_contouring_extended_mode_handoff_sec",
        "v2x_overtake_mpcc_frenet_dp_block_on_extended_solver_degraded",
    ):
        assert retired_key not in SOURCE
        assert retired_key not in MPCC_PROGRESS_HEADER
        assert retired_key not in CONFIG
        assert retired_key not in CLOUD_CONFIG


def test_unproducible_retained_dynamic_escape_path_is_physically_deleted() -> None:
    """A private retained path without an artifact producer must not compile."""

    for retired_symbol in (
        "RetainedDynamicEscapeExecution",
        "RetainedDynamicEscapeControl",
        "pending_dynamic_escape_execution_",
        "retained_dynamic_escape_execution_",
        "dynamic_obstacle_lateral_escape_formulation_lease_until_sec_",
        "dynamic_escape_formulation_lease_was_active_",
        "kDynamicEscapeHandoffLeaseSec",
        "restore_retained_dynamic_escape_execution(",
        "accept_current_dynamic_escape_execution(",
        "RetainedExecutionCursor",
        "DynamicEscapeExecutionLease",
        "retained_solution_available",
        "RetainedSolutionExpired",
        '"retained-stage"',
    ):
        assert retired_symbol not in SOURCE
        assert retired_symbol not in OVERTAKE_ORCHESTRATOR_HEADER
        assert retired_symbol not in OVERTAKE_ORCHESTRATOR_SOURCE


def test_node_level_normal_wall_handoff_owners_are_physically_deleted() -> None:
    """A certified canonical command cannot be reinterpreted at publication."""

    for retired_symbol in (
        "LegacyWallHandoffAuthority",
        "WallHandoffAdmission",
        "WallPathAdmissionGate",
        "DynamicEscapeExitGate",
        "DynamicEscapeExitRequest",
        "DynamicEscapeExitResolution",
        "WallPathAdmissionScope",
        "solver_wall_handoff_admission_gate_",
        "overtake_wall_admission_gate_",
        "dynamic_escape_wall_admission_gate_",
        "dynamic_escape_exit_gate_",
        "wall_handoff_hold_active",
        "wall_handoff_observation_required",
        "populate_wall_admission_observation",
        "SolverWallHandoffHold",
        "OvertakeWallAdmissionHold",
    ):
        assert retired_symbol not in SOURCE
        assert retired_symbol not in OVERTAKE_ORCHESTRATOR_HEADER
        assert retired_symbol not in OVERTAKE_ORCHESTRATOR_SOURCE


def test_canonical_publisher_does_not_postprocess_certified_actuation() -> None:
    """Certified actuation must be made physical before, not after, solving."""

    execution_start = SOURCE.index(
        "const bool canonical_normal_execution_active ="
    )
    execution_end = SOURCE.index(
        "canonical normal command mutated before publication:", execution_start
    )
    execution = SOURCE[execution_start:execution_end]
    assert "if (!canonical_normal_execution_active) {\n      acc = clip(" in execution
    assert "if (!canonical_normal_execution_active) {\n      u[1] = clip(" in execution
    assert (
        "if (!canonical_normal_execution_active || recovery_command_active) {"
        in execution
    )


def test_control_callback_overrun_trace_is_observation_only() -> None:
    """Timing attribution may diagnose a callback but cannot influence it."""

    record_start = SOURCE.index("void record_control_callback_duration(")
    control_start = SOURCE.index("void control()", record_start)
    record = SOURCE[record_start:control_start]
    assert "Control callback overrun detail:" in record
    for region in (
        "pre_mpc_ms",
        "mpc_ms",
        "post_mpc_ms",
        "recovery_ms",
        "publish_ms",
        "unattributed_ms",
    ):
        assert region in record
    assert "observation_only=1" in record
    for forbidden in (
        "publish_control_command(",
        "publish_failsafe_command(",
        "mpc_->get_control(",
        "canonical_normal_control(",
        "track_cruise_shadow_plan_store_.publish(",
    ):
        assert forbidden not in record

    control_end = SOURCE.index("void publish_zero_command()", control_start)
    control = SOURCE[control_start:control_end]
    assert "ControlCallbackTimingObservation callback_timing;" in control
    assert "record_control_callback_duration(steady_now, callback_timing);" in control
    assert 'callback_timing.checkpoint = "complete";' in control


def test_normal_recovery_safety_work_has_one_eligibility_owner() -> None:
    """Map safety may be skipped, but the detector/core update may not be."""

    function_start = SOURCE.index(
        "std::optional<stuck_recovery::CoreOutput> evaluate_stuck_recovery("
    )
    function_end = SOURCE.index(
        "void publish_recovery_gear_request(", function_start
    )
    function = SOURCE[function_start:function_end]
    assert function.count("recovery_safety_evaluation_required(") == 1
    assert (
        "if (recovery_safety_required && recovery_grid_ && "
        "recovery_footprint_.valid())"
    ) in function
    assert (
        "const auto safety = recovery_safety_required ? "
        "evaluate_recovery_safety("
    ) in function
    assert "auto output = stuck_recovery_core_->update(input);" in function
    assert "return output;" in function


def test_rate_resolved_retained_revalidation_uses_artifact_intent_capability() -> None:
    """Artifact validation and current-world proof share one intent owner."""

    retained = (
        PACKAGE_ROOT / "src/mpcc_rate_resolved_retained_revalidation.cpp"
    ).read_text()
    assert "artifact::supports_intent(request.current_intent)" in retained
    assert "bool track_cruise(" not in retained
