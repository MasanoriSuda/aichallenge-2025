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
CMAKE = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
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


def test_follow_current_world_observation_survives_a_new_intent_proposal() -> None:
    """Current Follow owns its projection; prior Follow can rejoin all V2X."""

    helper_start = SOURCE.index("build_current_follow_target_observation(")
    helper_end = SOURCE.index(
        "RateResolvedPreentryAdoptionShadowEvaluation\n", helper_start
    )
    helper = SOURCE[helper_start:helper_end]
    assert "execution.identity.source_context.target_id" in helper
    assert "problem.follow_longitudinal_contract" in helper
    assert "follow.target_id == target_id" in helper
    assert "follow.target_observation_generation" in helper
    assert "current_world.obstacles.obstacles" in helper
    assert "selected_target_provenance(" not in helper
    assert "last_v2x_behavior_output_" not in helper
    assert "build_follow_target_observation(" in helper

    evaluate_start = SOURCE.index(
        "RateResolvedRetainedShadowEvaluation evaluate_rate_resolved_track_cruise_plan("
    )
    evaluate_end = SOURCE.index(
        "evaluate_rate_resolved_track_cruise_retained_shadow(", evaluate_start
    )
    evaluate = SOURCE[evaluate_start:evaluate_end]
    assert "build_current_follow_target_observation(" in evaluate
    assert "problem.follow_longitudinal_contract.valid" not in evaluate


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
        "          std::move(problem), now_sec, control_intent);"
    ) in overtake
    assert "canonical_overtake_production_control(" not in SOURCE[control_start:control_end]
    assert "canonical_normal_control(" not in overtake
    assert "solve_problem(" not in overtake

    owner_start = SOURCE.index("rate_resolved_normal_production_control(")
    owner_end = SOURCE.index("MpcControlCycleResult get_control(", owner_start)
    owner = SOURCE[owner_start:owner_end]
    consume = owner.index("evaluate_rate_resolved_track_cruise_retained_shadow(")
    resolve = owner.index("rate_resolved_track_cruise_control(")
    stage = owner.index("pending_rate_resolved_publication_successor_ =")
    assert consume < resolve < stage
    assert "bind_rate_resolved_track_cruise_submission(" not in owner
    assert "submit_rate_resolved_track_cruise_shadow(" not in owner
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


def test_rejoin_uses_the_shared_rate_resolved_normal_owner() -> None:
    """Recovery-line Rejoin may change intent, never normal formulation."""

    artifact_source = (
        PACKAGE_ROOT / "src/mpcc_rate_resolved_execution_artifact.cpp"
    ).read_text(encoding="utf-8")
    supports_start = artifact_source.index("bool supports_intent(")
    supports_end = artifact_source.index("bool identity_valid(", supports_start)
    supports = artifact_source[supports_start:supports_end]
    assert "ControlIntent::Rejoin" in supports

    control_start = SOURCE.index("MpcControlCycleResult get_control(")
    control_end = SOURCE.index(
        "std::pair<std::vector<double>, std::vector<double>> update_prediction(",
        control_start,
    )
    dispatch = SOURCE[control_start:control_end]
    assert "rate_resolved_artifact::supports_intent(control_intent)" in dispatch
    assert "evaluate_rejoin_canonical(" not in dispatch
    assert "record_rejoin_canonical_telemetry(" not in dispatch
    assert "VelocityProgress5State" not in dispatch


def test_targetless_normal_intents_do_not_borrow_stale_target_provenance() -> None:
    """Track/Cruise/Rejoin identity must not inherit a prior pass target."""

    context_start = SOURCE.index("mpcc_contract::MpccProblemContext make_problem_context(")
    context_end = SOURCE.index(
        "mpcc_contract::MpccProblemContext seal_problem_context_for_problem(",
        context_start,
    )
    context = SOURCE[context_start:context_end]
    assert "const bool target_required =" in context
    assert "canonical_normal_intent_requires_target(context.intent)" in context
    assert "if (target_required && last_overtake_authority_trace_.has_value())" in context
    assert re.search(r"if\s*\(\s*target_required\s*&&\s*provenance\.valid", context)


def test_five_state_rejoin_owner_is_physically_deleted() -> None:
    """The retired Rejoin solver/store/telemetry must not remain reconnectable."""

    retired_symbols = (
        "CanonicalRejoinCycleResult",
        "CanonicalRejoinTelemetryWindow",
        "evaluate_rejoin_canonical(",
        "record_rejoin_canonical_telemetry(",
        "rejoin_shadow_solver_context_",
        "rejoin_shadow_plan_store_",
        "rejoin_shadow_warm_start_identity_",
        "rejoin_shadow_context_epoch_",
        "rejoin_canonical_telemetry_window_",
        "last_rejoin_canonical_status_",
        "last_rejoin_canonical_telemetry_log_sec_",
    )
    assert not [symbol for symbol in retired_symbols if symbol in SOURCE]


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


def test_rate_resolved_normal_snapshot_separates_command_and_response_states() -> None:
    """The integrated steering state is command; measured motion initializes response."""

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
    stage = owner.index("pending_rate_resolved_publication_successor_ =")
    assert consume < resolve < stage
    assert "bind_rate_resolved_track_cruise_submission(" not in owner
    assert "submit_rate_resolved_track_cruise_shadow(" not in owner

    assert "#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>" in SOURCE
    assert "steering_status_sub_ = create_subscription<SteeringReport>(" in SOURCE
    assert "current_physical_steering_state_" in SOURCE
    update_start = SOURCE.index(
        "update_physical_steering_state_for_execution_contract("
    )
    update_end = SOURCE.index("void update_response_steering_state", update_start)
    update = SOURCE[update_start:update_end]
    assert "command_control_origin_steering_rad_" in update
    assert "current_physical_steering_state_->committed_steering_rad" in update
    assert "prediction_origin_steering_rad" not in update


def test_rate_resolved_successor_is_bound_only_after_exact_publication() -> None:
    """The next solve must start from this callback's serialized command."""

    owner_start = SOURCE.index("rate_resolved_normal_production_control(")
    owner_end = SOURCE.index("MpcControlCycleResult get_control(", owner_start)
    owner = SOURCE[owner_start:owner_end]
    assert "submit_rate_resolved_track_cruise_shadow(" not in owner
    assert "submit_rate_resolved_preentry_execution_shadow(" not in owner
    assert "pending_rate_resolved_publication_successor_" in owner

    recorder_start = SOURCE.index(
        "record_rate_resolved_publication_successor("
    )
    recorder_end = SOURCE.index(
        "record_canonical_normal_final_command(", recorder_start
    )
    recorder = SOURCE[recorder_start:recorder_end]
    serialized = recorder.index(
        "physical_steering_matches_serialized_actuation("
    )
    bind = recorder.index("bind_rate_resolved_track_cruise_submission(")
    submit = recorder.index("submit_rate_resolved_track_cruise_shadow(")
    assert serialized < bind < submit
    assert "final_physical_steering_rad" in recorder[bind:submit]
    assert "command_control_origin_steering_rad_.value()" not in recorder

    control_start = SOURCE.index("void control()")
    control_end = SOURCE.index("void publish_zero_command()", control_start)
    control = SOURCE[control_start:control_end]
    publish = control.index("const auto published_steering = publish_control_command(")
    successor = control.index("mpc_->record_rate_resolved_publication_successor(")
    assert publish < successor
    assert "u[1]" in control[successor : successor + 300]
    successor_call = control[successor : successor + 400]
    assert (
        "!recovery_command_active &&\n"
        "      (!mpc_fallback_active || canonical_emergency_stop)"
        in successor_call
    )


def test_rate_resolved_intent_transition_admits_the_same_rate_resolved_producer() -> None:
    """An intent change joins the exact certified plan before publication."""

    admission_start = SOURCE.index(
        "evaluate_rate_resolved_transition_admission("
    )
    admission_end = SOURCE.index(
        "RateResolvedRetainedShadowEvaluation\n"
        "  evaluate_rate_resolved_track_cruise_retained_shadow(",
        admission_start,
    )
    admission = SOURCE[admission_start:admission_end]
    assert "command_control_origin_steering_rad_.has_value()" in admission
    assert "current_physical_steering_state_.has_value()" in admission
    assert (
        "bind_rate_resolved_track_cruise_submission(\n"
        "        draft, command_control_origin_steering_rad_.value())"
        in admission
    )
    assert "bind_rate_resolved_track_cruise_submission(draft, previous_steering)" not in admission
    assert "build_rate_resolved_submission_snapshot(" in admission
    assert "evaluate_rate_resolved_pipeline(" in admission
    assert "admission.certified_plan = evaluation.certified_plan.plan" in admission
    assert "evaluate_rate_resolved_track_cruise_plan(" in admission
    assert "admission.current_world_reason = admission.retained.reason" in admission
    assert "admission.current_world_joined" in admission
    assert (
        "rate_resolved_track_cruise_certified_plan_store_->candidate_snapshot()"
        not in admission
    )
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
    publication = owner.index("rate_resolved_track_cruise_control(")
    assert first_revalidation < admission_call < publication
    # A tactical intent change is atomic: first prove the proposed intent,
    # then prove the last actually published intent in the same current world
    # when the proposal cannot be adopted.
    assert owner.count("evaluate_rate_resolved_track_cruise_retained_shadow(") == 2
    assert "!retained.production_authority.has_value()" in owner
    assert "ControlIntent::Unknown" in owner
    assert "last_published_canonical_intent_ != intent" in owner
    assert "retained.reason == rate_resolved_retained::Reason::MissingPlan" not in owner
    assert "retained.reason == rate_resolved_retained::Reason::IntentMismatch" not in owner
    assert "if (admission.current_world_joined)" in owner
    assert "retained = admission.retained" in owner
    assert "const auto previous_intent = last_published_canonical_intent_" in owner
    assert "resolve_atomic_intent_admission(" in owner
    assert "effective_intent = atomic_resolution.effective_intent" in owner
    assert "problem, effective_intent, retained" in owner


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
        "      mpcc_contract::Formulation::VelocitySteeringYawResponseProgress7State);"
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
    assert "admission.certified_plan = evaluation.certified_plan.plan" in admission
    assert "evaluate_rate_resolved_track_cruise_plan(" in admission
    assert "VelocityProgress5State" not in admission
    assert "legacy-mpc" not in admission

    owner_start = SOURCE.index("rate_resolved_normal_production_control(")
    owner_end = SOURCE.index("MpcControlCycleResult get_control(", owner_start)
    owner = SOURCE[owner_start:owner_end]
    admission_call = owner.index("evaluate_rate_resolved_transition_admission(")
    production = owner.index("rate_resolved_track_cruise_control(")
    assert admission_call < production
    assert "retained = admission.retained" in owner
    assert owner.count("evaluate_rate_resolved_track_cruise_retained_shadow(") == 2
    assert "last_published_canonical_intent_" in owner
    assert "ControlIntent::Follow" in owner
    assert "resolve_atomic_intent_admission(" in owner
    assert "effective_intent = atomic_resolution.effective_intent" in owner


def test_last_published_intent_is_a_publication_ledger() -> None:
    """Solver selection may not advance the actually-published intent ledger."""

    owner_start = SOURCE.index("rate_resolved_normal_production_control(")
    owner_end = SOURCE.index("MpcControlCycleResult get_control(", owner_start)
    owner = SOURCE[owner_start:owner_end]
    assert "last_published_canonical_intent_ =" not in owner

    recorder_start = SOURCE.index("record_canonical_normal_final_command(")
    recorder_end = SOURCE.index(
        "const std::optional<overtake_orchestrator::AuthorityTrace> &",
        recorder_start,
    )
    recorder = SOURCE[recorder_start:recorder_end]
    serialized_join = recorder.index(
        "canonical_normal_command_matches_serialized_actuation("
    )
    ledger_update = recorder.index(
        "last_published_canonical_intent_ = pending.command.intent"
    )
    assert serialized_join < ledger_update
    assert recorder.index("mark_executed(") < ledger_update


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


def test_fresh_preentry_uses_six_state_gate_for_shiftout_and_direct_pass() -> None:
    """No fresh entry, including start-grid, may bypass six-state Gate A."""

    entry_start = SOURCE.index("const bool fresh_normal_mission_entry =")
    entry_end = SOURCE.index(
        "transition_overtake_line_phase(\n"
        "          direct_pass ? OvertakeLinePhase::Pass",
        entry_start,
    )
    entry = SOURCE[entry_start:entry_end]
    assert "resolve_overtake_preentry_plan(" not in entry
    assert "five-state-direct-pass-gate-a" not in entry
    assert "six-state-preentry-gate-a" in entry
    assert "six_state_preentry_gate_a_request" in entry
    assert "six_state_direct_pass_gate_a_request" in SOURCE
    assert "ControlIntent::Pass" in SOURCE[
        SOURCE.index("const bool six_state_preentry_gate_a_request =") : entry_start
    ]
    assert "freeze_selected_overtake_mission(fresh_entry_mission" in entry
    assert "behavior_output.overtake_preentry_canonical_plan" not in entry
    assert "steering_continuity" not in entry
    assert "prepare_overtake_canonical_async_context(" not in entry
    assert "overtake_canonical_lifecycle_->plan_store.replace(" not in entry
    assert "!behavior_output.start_grid_breakout_active" not in entry

    request_start = SOURCE.index("const bool behavior_requests_overtake =")
    request_end = SOURCE.index("if (solver_reentry_suppressed", request_start)
    request = SOURCE[request_start:request_end]
    assert "six_state_preentry_gate_a_request ||" in request
    assert "overtake_line_state_.phase != OvertakeLinePhase::Idle" in request

    start_grid_start = SOURCE.index(
        "if (start_grid_corridor_assessment.gap_available) {"
    )
    start_grid_end = SOURCE.index(
        "const bool dynamic_decision_scope =", start_grid_start
    )
    start_grid = SOURCE[start_grid_start:start_grid_end]
    assert "left_assessment = start_grid_corridor_assessment" not in start_grid
    assert "right_assessment = start_grid_corridor_assessment" not in start_grid
    assert "not a normal execution candidate" in start_grid

    mission_collection = SOURCE[
        SOURCE.index("const auto add_global_mission_candidate =") :
        SOURCE.index("const bool global_complete_mission_available =")
    ]
    assert "add_global_mission_candidate(left_assessment);" in mission_collection
    assert "add_global_mission_candidate(right_assessment);" in mission_collection
    assert "if (!start_grid_breakout_attempt)" not in mission_collection


def test_rate_resolved_preentry_gate_shadow_uses_explicit_intent_without_authority() -> None:
    """Prospective Gate A must not inherit the still-live Follow identity."""

    assert "struct RateResolvedPreentryShadowEvaluation" in SOURCE
    shadow_start = SOURCE.index(
        "RateResolvedPreentryShadowEvaluation\n"
        "  evaluate_rate_resolved_preentry_shadow("
    )
    shadow_end = SOURCE.index(
        "mpcc_progress::ExtendedBranchEvaluation evaluate_extended_mpcc_branch(",
        shadow_start,
    )
    shadow = SOURCE[shadow_start:shadow_end]
    assert "const mpcc_contract::ControlIntent prospective_intent" in shadow
    assert "current_control_intent()" not in shadow
    assert "Formulation::VelocitySteeringYawResponseProgress7State" in shadow
    assert "evaluate_rate_resolved_pipeline(" in shadow
    assert "validate_frenet_dp_target_bound_horizon(" in shadow
    assert "rate_resolved_track_cruise_certified_plan_store_" not in shadow
    assert "canonical_normal_command" not in shadow
    assert "bind_rate_resolved_track_cruise_submission(" in shadow
    assert "command_control_origin_steering_rad_.value()" in shadow
    assert "current_physical_steering_state_->committed_steering_rad" not in shadow
    assert "BoundRateResolvedTrackCruiseSubmission bound_submission;" not in shadow

    isolated_start = SOURCE.index(
        "ExtendedMpccBranchArtifact evaluate_isolated_extended_mpcc_branch("
    )
    isolated_end = SOURCE.index(
        "void evaluate_and_select_extended_mpcc_branches(", isolated_start
    )
    isolated = SOURCE[isolated_start:isolated_end]
    assert "rate_resolved_preentry_left_solver_context_" in isolated
    assert "rate_resolved_preentry_right_solver_context_" in isolated

    selection_start = SOURCE.index(
        "void evaluate_and_select_extended_mpcc_branches("
    )
    selection_end = SOURCE.index(
        "bool submit_mpcc_lite_async_snapshot(",
        selection_start,
    )
    selection = SOURCE[selection_start:selection_end]
    assert selection.count("mpcc_progress::select_extended_branch(") == 1
    assert "behavior.rate_resolved_preentry_branch_selection =" in selection
    assert (
        "behavior.extended_mpcc_branch_selection =\n"
        "      behavior.rate_resolved_preentry_branch_selection;"
    ) in selection
    assert "preentry_canonical_plan" not in selection
    assert "authority=shadow,selected=0" in SOURCE
    assert "if (six_left.attempted || six_right.attempted)" in SOURCE

    physical_start = SOURCE.index(
        "std::optional<rate_resolved_physical_wall::Snapshot>\n"
        "  build_rate_resolved_track_cruise_physical_snapshot("
    )
    physical_end = SOURCE.index(
        "std::optional<RateResolvedTrackCruiseSubmissionDraft>",
        physical_start,
    )
    assert "physical_wall_mailbox_" not in SOURCE[physical_start:physical_end]


def test_six_state_preentry_selection_preserves_exact_evidence_without_authority() -> None:
    """Selection evidence must come from the same immutable six-state proof."""

    pipeline_start = SOURCE.index("RateResolvedPipelineEvaluation evaluate_rate_resolved_pipeline(")
    pipeline_end = SOURCE.index("struct CanonicalCurrentControlPath", pipeline_start)
    pipeline = SOURCE[pipeline_start:pipeline_end]
    assert "evaluation.certified_plan = rate_resolved_certified::build(" in pipeline

    shadow_start = SOURCE.index(
        "RateResolvedPreentryShadowEvaluation\n"
        "  evaluate_rate_resolved_preentry_shadow("
    )
    shadow_end = SOURCE.index(
        "mpcc_progress::ExtendedBranchEvaluation evaluate_extended_mpcc_branch(",
        shadow_start,
    )
    shadow = SOURCE[shadow_start:shadow_end]
    assert "result.objective = evaluation.solver.solver.objective_value" in shadow
    assert "result.certified_plan = evaluation.certified_plan.plan" in shadow
    assert "rate_resolved_track_cruise_certified_plan_store_" not in shadow

    selection_start = SOURCE.index("void evaluate_and_select_extended_mpcc_branches(")
    selection_end = SOURCE.index("bool submit_mpcc_lite_async_snapshot(", selection_start)
    selection = SOURCE[selection_start:selection_end]
    assert "behavior.rate_resolved_preentry_branch_selection =" in selection
    assert "rate_resolved_preentry_branch_evaluation(" in selection
    assert (
        "behavior.extended_mpcc_branch_selection =\n"
        "      behavior.rate_resolved_preentry_branch_selection;"
    ) in selection
    assert "preentry_canonical_plan" not in selection
    assert "five_selected=%d" not in SOURCE
    assert "selection_agree=%d" not in SOURCE
    assert "selection_valid=five%d/six%d" not in SOURCE
    assert '"selected=%d,selection_valid=%d,"' in SOURCE
    assert "authority=shadow,selected=0" in SOURCE

    import_start = SOURCE.index("if (accepted_async_tactical_result != nullptr) {")
    import_end = SOURCE.index(
        "if (opponent_side_replan_assessment_requested", import_start
    )
    imported = SOURCE[import_start:import_end]
    assert (
        "output.rate_resolved_preentry_branch_selection =\n"
        "          async_behavior.rate_resolved_preentry_branch_selection;"
    ) in imported


def test_six_state_preentry_adoption_reuses_current_world_proof_without_authority() -> None:
    """Live adoption evidence must reuse, but not own, production proof."""

    helper_start = SOURCE.index(
        "build_rate_resolved_current_world_request("
    )
    helper_end = SOURCE.index(
        "RateResolvedPreentryAdoptionShadowEvaluation\n"
        "  evaluate_rate_resolved_preentry_adoption_shadow(",
        helper_start,
    )
    helper = SOURCE[helper_start:helper_end]
    assert "request.measured_to_control_path = control_path->poses" in helper
    assert "request.current_wall_grid = overtake_static_wall_grid_snapshot_owner_" in helper
    assert "gap_planner->dynamic_world_observation(now_sec)" in helper

    adoption_start = helper_end
    adoption_end = SOURCE.index(
        "RateResolvedRetainedShadowEvaluation\n"
        "  evaluate_rate_resolved_track_cruise_retained_shadow(",
        adoption_start,
    )
    adoption = SOURCE[adoption_start:adoption_end]
    assert "validate_selected_target_provenance(" in adoption
    assert "build_rate_resolved_current_world_request(" in adoption
    assert "rate_resolved_retained::evaluate(request.value())" in adoption
    assert "rate_resolved_track_cruise_certified_plan_store_" not in adoption
    assert "certify_and_replace(" not in adoption
    for evidence in (
        "evaluation.artifact_age_sec",
        "evaluation.cursor_elapsed_sec",
        "evaluation.progress_difference_m",
        "evaluation.steering_difference_rad",
        "evaluation.velocity_difference_mps",
    ):
        assert evidence in adoption

    assert "progress:measured:%.3f/lifted:%.3f/expected:%.3f/" in SOURCE
    assert "steering:now:%.4f/held_projection:%.4f/expected:%.4f/" in SOURCE
    assert "delta_from_now:%.4f/bounds:[%.4f,%.4f]/duration:%.3f" in SOURCE
    assert "velocity:current:%.3f/expected:%.3f/delta:%.3f/" in SOURCE

    production_end = SOURCE.index(
        "void record_rate_resolved_track_cruise_shadow(", adoption_end
    )
    production_retained = SOURCE[adoption_end:production_end]
    assert "evaluate_rate_resolved_track_cruise_plan(" in production_retained

    import_start = SOURCE.index("if (accepted_async_tactical_result != nullptr) {")
    import_end = SOURCE.index(
        "if (opponent_side_replan_assessment_requested", import_start
    )
    imported = SOURCE[import_start:import_end]
    assert "if (!mpcc_lite_async_worker_context_)" in imported
    assert "evaluate_rate_resolved_preentry_adoption_shadow(output, now_sec)" in imported

    assert "apply_mpcc_entry_execution_contract" not in SOURCE
    assert "adoption=%d/%d/%s/%s/target:%s" in SOURCE
    assert "authority=shadow,selected=0" in SOURCE

    geometry_start = SOURCE.index("void set_overtake_static_wall_geometry(")
    geometry_end = SOURCE.index(
        "PhysicalWallEnvelopeCacheTelemetry take_physical_wall_envelope_cache_telemetry()",
        geometry_start,
    )
    geometry = SOURCE[geometry_start:geometry_end]
    assert "overtake_static_wall_grid_fingerprint_ = 0U" in geometry
    assert "recovery_footprint::occupancy_grid_fingerprint(" in geometry
    assert (
        "snapshot.wall_grid_fingerprint = overtake_static_wall_grid_fingerprint_"
        in SOURCE
    )


def test_tactical_async_and_isolated_branches_share_one_owned_snapshot_boundary() -> None:
    """Deep-copy ownership must not drift between tactical execution paths."""

    helper_start = SOURCE.index("struct OwnedTacticalSnapshot")
    helper_end = SOURCE.index("void invalidate_mpcc_lite_async_results()", helper_start)
    helper = SOURCE[helper_start:helper_end]
    assert "std::make_shared<ReferencePath>(*model->reference_path)" in helper
    assert "std::make_shared<BicycleModel>(*model)" in helper
    assert "gap_planner->tactical_snapshot()" in helper
    assert "snapshot.planner->reference_path_snapshot_owner_" in helper

    isolated_start = SOURCE.index(
        "ExtendedMpccBranchArtifact evaluate_isolated_extended_mpcc_branch("
    )
    isolated_end = SOURCE.index("void evaluate_and_select_extended_mpcc_branches(", isolated_start)
    isolated = SOURCE[isolated_start:isolated_end]
    assert "make_owned_tactical_snapshot()" in isolated

    submit_start = SOURCE.index("bool submit_mpcc_lite_async_snapshot(")
    submit_end = SOURCE.index("void reset_after_external_maneuver(", submit_start)
    submit = SOURCE[submit_start:submit_end]
    assert "auto owned_snapshot = make_owned_tactical_snapshot();" in submit
    assert "owned_snapshot = std::move(owned_snapshot.value())" in submit


def test_preentry_causal_execution_pipeline_is_gate_only_and_predecessor_bound() -> None:
    """Tactical output chooses homotopy; causal proof may gate but not publish."""

    selection_start = SOURCE.index("void evaluate_and_select_extended_mpcc_branches(")
    selection_end = SOURCE.index("bool submit_mpcc_lite_async_snapshot(", selection_start)
    selection = SOURCE[selection_start:selection_end]
    assert "rate_resolved_preentry_selected_mission_hint" in selection
    assert "hint.physical_execution_certificate_valid = false" in selection
    assert "hint.physical_execution_certificate_path_distances_m.clear()" in selection
    assert "hint.physical_execution_certificate_lateral_path_m.clear()" in selection
    assert "hint.physical_execution_certificate_exact_trajectory = {}" in selection

    prospective_start = SOURCE.index("build_prospective_extended_branch_problem(")
    prospective_end = SOURCE.index("evaluate_extended_mpcc_branch(", prospective_start)
    prospective = SOURCE[prospective_start:prospective_end]
    assert "freeze_selected_overtake_mission(candidate, now_sec)" in prospective
    assert "init_problem(" in prospective
    assert "build_extended_progress_problem(" in prospective

    draft_start = SOURCE.index("build_rate_resolved_preentry_execution_draft(")
    draft_end = SOURCE.index("void evaluate_and_select_extended_mpcc_branches(", draft_start)
    draft = SOURCE[draft_start:draft_end]
    assert "make_owned_tactical_snapshot()" in draft
    assert "draft.planner_snapshot = std::move(owned_snapshot->planner)" in draft
    assert "draft.decision_id = active_control_decision_id_" in draft
    assert "draft.context_epoch = mpcc_lite_async_context_epoch_" in draft
    assert "draft.snapshot_sec = now_sec" in draft
    assert "build_prospective_extended_branch_problem(" not in draft
    assert "init_problem(" not in draft
    assert "evaluate_v2x_behavior(" not in draft
    assert "solve_extended_progress_problem(" not in draft

    submit_start = SOURCE.index(
        "bool submit_rate_resolved_preentry_execution_shadow("
    )
    submit_end = SOURCE.index("consume_rate_resolved_preentry_execution_shadow(", submit_start)
    submit = SOURCE[submit_start:submit_end]
    queued = submit.index(
        "rate_resolved_preentry_execution_shadow_worker_->submit_latest("
    )
    worker = submit[queued:]
    assert "planner->build_prospective_extended_branch_problem(" in worker
    assert "planner->seal_problem_context_for_problem(" in worker
    assert "bind_rate_resolved_track_cruise_submission(" in submit
    assert "command_control_origin_steering_rad" in submit
    assert "committed_predecessor_steering_rad" not in submit
    # The canonical publication request type is shared, but worker ownership
    # remains an injected private member rather than being constructed here.
    assert "std::make_unique<LatestOnlyWorker>" not in submit
    assert "observation_only_store" in worker
    assert "should_publish_latest_only_result(" in worker
    assert "result.context_epoch" in worker
    assert "result.selected_mission = draft.assessment.selected_mission" in submit
    assert "result.target_provenance = draft.target_provenance" in submit
    assert "mailbox->context_epoch" in worker
    assert "result.sequence != mailbox->latest_submitted_sequence" not in worker
    assert submit.index("submit_latest(") < submit.index(
        "planner->build_prospective_extended_branch_problem("
    )
    assert "certify_and_replace(" not in submit
    assert "publish_control_command(" not in submit

    consume_start = submit_end
    consume_end = SOURCE.index("RateResolvedTransitionAdmissionEvaluation", consume_start)
    consume = SOURCE[consume_start:consume_end]
    assert "build_rate_resolved_current_world_request(" in consume
    assert "rate_resolved_retained::evaluate(" in consume
    assert "resolve_preentry_tactical_identity(" in consume
    assert "tactical_identity.current_world_observation_permitted" in consume
    assert "tactical_identity.tactical_authority_current" in consume
    assert "current_world_joinable && tactical_identity.tactical_authority_current" in consume
    assert "RateResolvedMissionGateAProposal proposal" in consume
    assert "bind_rate_resolved_gate_a_execution_certificate(" in consume
    assert "proposal.mission = std::move(certified_mission)" in consume
    assert "proposal.certified_plan = plan" in consume
    assert "validate_selected_target_provenance(" in consume
    assert (
        "proposal.target_obstacle_generation =\n"
        "        result->target_provenance.observation_generation" in consume
    )
    assert (
        "live_behavior.rate_resolved_mission_gate_a_proposal =" in consume
    )
    assert "authority=gate-a-evidence,selected=%d" in consume
    assert "certify_and_replace(" not in consume
    assert "publish_control_command(" not in consume

    certificate_start = SOURCE.index(
        "bool bind_rate_resolved_gate_a_execution_certificate("
    )
    certificate_end = SOURCE.index(
        "struct V2XBehaviorOutput", certificate_start
    )
    certificate = SOURCE[certificate_start:certificate_end]
    assert "rate_resolved_certified::validate(*plan)" in certificate
    assert "plan->physical_snapshot->trajectory" in certificate
    assert "plan->execution_artifact->course_progress_origin_m" in certificate
    assert "mission.physical_execution_certificate_valid = true" in certificate
    assert "mission.physical_execution_certificate_path_distances_m =" in certificate
    assert "mission.physical_execution_certificate_lateral_path_m =" in certificate

    freeze_start = SOURCE.index("void freeze_selected_overtake_mission(")
    freeze_end = SOURCE.index(
        "bool replace_frozen_overtake_mission_after_dynamic_replan(", freeze_start
    )
    freeze = SOURCE[freeze_start:freeze_end]
    assert "!physical_execution_certificate_available &&" in freeze


def test_five_state_overtake_tactical_gate_is_physically_deleted() -> None:
    """A five-state tactical result cannot gate the six-state Mission."""

    retired_symbols = (
        "preentry_canonical_plan",
        "overtake_preentry_canonical_plan",
        "mpcc_lite_control_last_feasible_entry_plan_",
        "apply_mpcc_entry_execution_contract",
        "revalidate_overtake_entry_execution_certificate",
        "evaluate_overtake_canonical_fresh_shadow",
        "evaluate_overtake_canonical_worker_fresh",
        "overtake_tactical_five_state_lifecycle_",
    )
    for symbol in retired_symbols:
        assert symbol not in SOURCE
    controller_link_start = CMAKE.index("target_link_libraries(mpc_controller_cpp")
    controller_link_end = CMAKE.index(")", controller_link_start)
    controller_links = CMAKE[controller_link_start:controller_link_end]
    for retired_library in (
        "canonical_execution_plan_adapter",
        "canonical_retained_world_revalidation",
        "follow_canonical_async",
    ):
        assert retired_library not in controller_links

    branch_start = SOURCE.index("evaluate_extended_mpcc_branch(")
    branch_end = SOURCE.index(
        "ExtendedMpccBranchArtifact evaluate_isolated_extended_mpcc_branch(",
        branch_start,
    )
    branch = SOURCE[branch_start:branch_end]
    assert "evaluate_rate_resolved_preentry_shadow(" in branch
    assert "solve_extended_progress_problem(" not in branch
    assert "Formulation::VelocityProgress5State" not in branch

    invalidation_start = SOURCE.index("void invalidate_mpcc_lite_async_results()")
    invalidation_end = SOURCE.index("void set_gap_planner(", invalidation_start)
    invalidation = SOURCE[invalidation_start:invalidation_end]
    assert "const auto invalidate_mailbox" in invalidation
    assert "std::scoped_lock lock(" in invalidation
    assert "mailbox.context_epoch = mpcc_lite_async_context_epoch_" in invalidation
    assert "mailbox.latest_result.reset()" in invalidation
    assert (
        "invalidate_mailbox(*rate_resolved_preentry_execution_shadow_mailbox_)"
        in invalidation
    )

    production_start = SOURCE.index("MpcControlCycleResult rate_resolved_normal_production_control(")
    production_end = SOURCE.index("MpcControlCycleResult get_control(", production_start)
    production = SOURCE[production_start:production_end]
    assert "consume_rate_resolved_preentry_execution_shadow(" not in production
    build_position = production.index("build_rate_resolved_preentry_execution_draft(")
    command_position = production.index("rate_resolved_track_cruise_control(")
    stage_position = production.index(
        "pending_rate_resolved_publication_successor_ ="
    )
    assert build_position < command_position < stage_position
    assert "submit_rate_resolved_preentry_execution_shadow(" not in production

    init_start = SOURCE.index("MpcProblem init_problem(")
    init_end = SOURCE.index("void record_solution_contract(", init_start)
    init_problem = SOURCE[init_start:init_end]
    behavior_position = init_problem.index(
        "*behavior_override : evaluate_v2x_behavior("
    )
    consume_position = init_problem.index(
        "consume_rate_resolved_preentry_execution_shadow("
    )
    gate_a_position = init_problem.index(
        "auto overtake_line_output =\n      update_overtake_line("
    )
    assert behavior_position < consume_position < gate_a_position
    consume_guard_start = init_problem.rfind(
        "if (behavior_override == nullptr)", 0, consume_position
    )
    assert consume_guard_start >= 0
    consume_guard = init_problem[consume_guard_start:gate_a_position]
    assert "consume_rate_resolved_preentry_execution_shadow(" in consume_guard

    fsm_start = SOURCE.index("OvertakeLineOutput update_overtake_line(")
    fsm_end = SOURCE.index("bool is_overtake_forbidden_wp(", fsm_start)
    fsm = SOURCE[fsm_start:fsm_end]
    assert "rate_resolved_mission_gate_a_proposal" in fsm
    assert "six_state_preentry_gate_a_request" in fsm
    assert "six_state_direct_pass_gate_a_request" in fsm
    assert "ControlIntent::ShiftOut" in fsm
    assert "ControlIntent::Pass" in fsm
    assert (
        "proposal_source_context->target_obstacle_generation ==\n"
        "            six_state_gate_a_proposal->target_obstacle_generation" in fsm
    )
    assert "publish_control_command(" not in fsm


def test_unreachable_five_state_overtake_owner_is_physically_deleted() -> None:
    """Dead retained selectors may not keep a five-state publisher reconnectable."""

    for retired_symbol in (
        "MpcControlCycleResult canonical_normal_control(",
        "OvertakeCanonicalFreshShadowResult evaluate_overtake_async_shadow(",
        "void evaluate_overtake_canonical_retained_shadow(",
        "bool submit_overtake_canonical_async(",
        "void record_overtake_canonical_async_status(",
        "CanonicalExecutionPlanStore plan_store;",
        "overtake_canonical_async_mailbox_",
        "overtake_canonical_async_worker_",
    ):
        assert retired_symbol not in SOURCE

    emergency_start = SOURCE.index(
        "MpcControlCycleResult canonical_normal_emergency_stop("
    )
    emergency_end = SOURCE.index(
        "MpcControlCycleResult rate_resolved_track_cruise_control(",
        emergency_start,
    )
    emergency = SOURCE[emergency_start:emergency_end]
    assert "Formulation::VelocityProgress5State" not in emergency
    assert "Formulation::Unresolved" in emergency


def test_runtime_overtake_replacement_requires_causal_six_state_gate() -> None:
    """Runtime Mission mutation may not be owned by geometry or five-state plans."""

    assert "struct OvertakeExecutionArtifact" not in SOURCE
    assert "make_overtake_execution_artifact(" not in SOURCE
    assert "resolve_overtake_preentry_plan(" not in SOURCE

    behavior_start = SOURCE.index("struct V2XBehaviorOutput")
    behavior_end = SOURCE.index("struct ExtendedMpccBranchArtifact", behavior_start)
    behavior = SOURCE[behavior_start:behavior_end]
    assert "rate_resolved_mission_gate_a_proposal" in behavior
    assert "mpcc_lite_same_side_replan_artifact" not in behavior
    assert "mpcc_lite_cross_side_replan_artifact" not in behavior

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
    assert "rate_resolved_mission_gate_a_proposal" in replace
    assert "replacement_canonical_plan" not in replace
    assert "resolve_overtake_preentry_plan(" not in replace
    assert "prospective_generation" in replace
    assert "gate_a_proposal.mission" in replace
    assert "gate_a_proposal.certified_plan" in replace
    assert "source_context.intent == replacement_intent" in replace
    assert "source_context.target_id == overtake_line_state_.target_vehicle_id" in replace
    assert "adopt_overtake_canonical_plan_context(" not in replace
    assert "prepare_overtake_canonical_async_context(" not in replace
    assert replace.index("source_context.intent == replacement_intent") < replace.index(
        "freeze_selected_overtake_mission("
    )
    assert replace.index("freeze_selected_overtake_mission(") < replace.index(
        "transition_overtake_line_phase("
    )

    assert SOURCE.count(
        "behavior_output.rate_resolved_mission_gate_a_proposal"
    ) >= 3


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
        "build_canonical_current_control_path() const", tube_start
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
        "std::optional<rate_resolved_physical_wall::Snapshot>", scope_start
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
        r"initial_stage\s*\?\s*legacy\.progress_control_origin_speed_mps", builder
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
    ):
        assert token in trace
    assert "canonical=%d/plan=%lu" not in trace


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
    transport_end = SOURCE.index(
        "resolve_physically_validated_mpcc_execution_trajectory(", submit_start
    )
    transport = SOURCE[submit_start:transport_end]
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
    assert "request.current_steering_rad =\n        command_control_origin_steering_rad_.value_or(" in semantic
    assert "request.current_steering_rad = previous_steering;" not in semantic
    assert "request.maximum_abs_steering_rate_radps" in semantic
    assert "first_curvature_input_lower" in semantic
    assert "first_curvature_input_upper" in semantic
    assert "physical_first_curvature->reachable_lower_radpm" not in semantic
    assert "physical_first_curvature->reachable_upper_radpm" not in semantic


def test_rate_resolved_physical_wall_proof_is_shared_but_cannot_publish() -> None:
    """Async and atomic admission share one proof pipeline with no publisher."""

    proof_start = SOURCE.index(
        "std::optional<rate_resolved_physical_wall::Snapshot>\n"
        "  build_rate_resolved_track_cruise_physical_snapshot("
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
    assert "candidate_snapshot()" in certified_header
    assert "mark_executed(" in certified_header
    assert "Last plan whose command was successfully published" in certified_header
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
    assert (
        "rate_resolved_track_cruise_certified_plan_store_->candidate_snapshot()"
        in evaluate
    )
    assert "evaluate_rate_resolved_track_cruise_plan(" in evaluate
    request_start = SOURCE.index(
        "build_rate_resolved_current_world_request("
    )
    request_end = SOURCE.index(
        "RateResolvedPreentryAdoptionShadowEvaluation\n"
        "  evaluate_rate_resolved_preentry_adoption_shadow(",
        request_start,
    )
    request_builder = SOURCE[request_start:request_end]
    assert "build_canonical_current_control_path()" in request_builder
    assert "gap_planner->dynamic_world_observation(now_sec)" in request_builder
    assert "dynamic_world.vehicles" in request_builder
    assert "command_control_origin_steering_rad_.has_value()" in request_builder
    assert (
        "request.current_steering_rad =\n"
        "      command_control_origin_steering_rad_.value();"
        in request_builder
    )
    assert "request.current_steering_rad = previous_steering;" not in request_builder
    plan_evaluate_start = SOURCE.index(
        "evaluate_rate_resolved_track_cruise_plan(", request_end
    )
    plan_evaluate_end = SOURCE.index(
        "evaluate_rate_resolved_track_cruise_retained_shadow(",
        plan_evaluate_start,
    )
    plan_evaluate = SOURCE[plan_evaluate_start:plan_evaluate_end]
    assert "build_rate_resolved_current_world_request(" in plan_evaluate
    assert "rate_resolved_retained::evaluate(request.value())" in plan_evaluate
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
    assert "source.wall_grid_fingerprint != 0U" in retained_source
    assert "request.current_wall_grid.get() == source.wall_grid.get()" in retained_source
    assert "recovery::occupancy_grid_fingerprint(*request.current_wall_grid)" in retained_source
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
    assert "pending_rate_resolved_publication_successor_ =" in branch
    assert "bind_rate_resolved_track_cruise_submission(" not in branch
    assert "submit_rate_resolved_track_cruise_shadow(" not in branch
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
    assert "evaluate_rejoin_canonical(" not in control
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


def test_retired_five_state_normal_implementation_is_physically_deleted() -> None:
    """The old canonical owner must not remain buildable or reconnectable."""

    retired_files = (
        "canonical_execution_plan.hpp",
        "canonical_execution_plan_adapter.hpp",
        "canonical_retained_revalidation.hpp",
        "canonical_retained_world_revalidation.hpp",
        "follow_canonical_async.hpp",
        "canonical_normal_async.hpp",
        "canonical_execution_plan.cpp",
        "canonical_execution_plan_adapter.cpp",
        "canonical_retained_revalidation.cpp",
        "canonical_retained_world_revalidation.cpp",
        "follow_canonical_async.cpp",
        "test_canonical_execution_plan.cpp",
        "test_canonical_execution_plan_adapter.cpp",
        "test_canonical_retained_revalidation.cpp",
        "test_canonical_retained_world_revalidation.cpp",
        "test_follow_canonical_async.cpp",
    )
    for name in retired_files:
        assert not list(PACKAGE_ROOT.rglob(name)), name

    production = "\n".join(
        (
            SOURCE,
            MPCC_PROGRESS_HEADER,
            MPCC_PROGRESS_SOURCE,
            MPCC_EXECUTION_CONTRACT_HEADER,
            MPCC_EXECUTION_CONTRACT_SOURCE,
            RACE_MPCC_FOUNDATION_HEADER,
            RACE_MPCC_FOUNDATION_SOURCE,
            (
                PACKAGE_ROOT
                / "include"
                / "multi_purpose_mpc_ros"
                / "persistent_osqp.hpp"
            ).read_text(encoding="utf-8"),
            (PACKAGE_ROOT / "src" / "persistent_osqp.cpp").read_text(
                encoding="utf-8"
            ),
            CMAKE,
        )
    )
    forbidden = (
        "VelocityProgress5State",
        '"velocity-progress-5state"',
        "ShadowWarmStartIdentity",
        "resolve_shadow_warm_start(",
        "build_exact_extended_wall_proof_input(",
        "executed_extended_progress_solution_wall_safe(",
        "dynamic_margin_escape_solution_wall_safe(",
        "executed_progress_solution_wall_safe(",
        "make_canonical_shadow_warm_start_identity(",
        "normalize_extended_execution_primal(",
        "evaluate_extended_lateral_constraint_contract(",
        "extract_extended_execution_trajectory(",
        "CertifiedWarmStartStore",
        "canonical_execution_plan",
        "canonical_retained_revalidation",
        "canonical_retained_world_revalidation",
        "follow_canonical_async",
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


def test_certified_candidate_becomes_retained_only_after_exact_publication() -> None:
    """Solver certification alone must not create retained execution evidence."""

    record_start = SOURCE.index("void record_canonical_normal_final_command(")
    record_end = SOURCE.index(
        "last_overtake_authority_trace() const noexcept", record_start
    )
    record = SOURCE[record_start:record_end]
    assert "canonical_normal_command_matches_serialized_actuation(" in record
    assert "pending.promote_to_executed" in record
    assert "mark_executed(" in record
    assert record.index(
        "canonical_normal_command_matches_serialized_actuation("
    ) < record.index("mark_executed(")

    control_start = SOURCE.index("void control()")
    control_end = SOURCE.index("void publish_zero_command()", control_start)
    control = SOURCE[control_start:control_end]
    publish = control.index("const auto published_steering = publish_control_command(")
    record_call = control.index("mpc_->record_canonical_normal_final_command(")
    assert publish < record_call
    assert "published_steering.value()" in control[record_call : record_call + 250]


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


def test_shiftout_rolling_source_is_projected_from_six_state_certified_store() -> None:
    """The deleted five-state primal recorder cannot own ShiftOut refresh."""

    assert "record_solved_mpcc_execution_trajectory(" not in SOURCE
    assert "adopt_rate_resolved_shiftout_execution_source(" in SOURCE
    assert (
        "rate_resolved_track_cruise_certified_plan_store_->snapshot()"
        in SOURCE
    )
    assert "rate_resolved_execution_source::build(" in SOURCE
    assert (
        "adopt_rate_resolved_shiftout_execution_source(now_sec);"
        in SOURCE
    )


def test_latency_wall_proof_reuses_the_canonical_state_prediction_trajectory() -> None:
    """The latency prefix may not be re-integrated with another yaw model."""

    build_start = SOURCE.index(
        "build_canonical_current_control_path() const"
    )
    build_end = SOURCE.index(
        "bool unresolved_dynamic_wait_canonical_scope()", build_start
    )
    build = SOURCE[build_start:build_end]
    assert "canonical_current_control_path_" in build
    assert "predict_constant_turn_rate(" not in build

    control_start = SOURCE.index("void control()")
    control_end = SOURCE.index("void publish_zero_command()", control_start)
    control = SOURCE[control_start:control_end]
    assert "predict_accelerating_yaw_response_trajectory(" in control
    assert "canonical_control_path = std::move(path);" in control
    assert "update_predicted_pose_for_execution_contract(" in control
