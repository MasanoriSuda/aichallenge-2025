#!/usr/bin/env python3
"""Contract tests for the E2E TinyLidarNet deployment artifact."""

import hashlib
from pathlib import Path

import numpy as np
import pytest

from tiny_lidar_net_controller.tiny_lidar_net_controller_core import TinyLidarNetCore
from tiny_lidar_net_controller.model.tinylidarnet import (
    RecurrentSteeringAdapterNp,
    SpatialSteeringAdapterNp,
    SteeringResidualNetNp,
    TinyLidarNetNp,
)


PACKAGE_ROOT = Path(__file__).parents[1]
CHECKPOINT = PACKAGE_ROOT / "ckpt" / "tinylidarnet_weights.npy"
SPATIAL_CHECKPOINT = PACKAGE_ROOT / "ckpt" / "spatial_steering_adapter.npy"
SPATIAL_CHECKPOINT_SHA256 = (
    "f3921c265677761bcf9458c61758d997b94d0b2045e87ebcee37ca94f3ed412c"
)


def _load_core(checkpoint: Path = CHECKPOINT) -> TinyLidarNetCore:
    return TinyLidarNetCore(
        input_dim=750,
        output_dim=2,
        architecture="normal",
        ckpt_path=str(checkpoint),
        acceleration=0.6,
        control_mode="fixed",
        max_range=30.0,
    )


def _write_spatial_shadow_checkpoint(
    tmp_path: Path,
    *,
    mismatch_base: bool = False,
    use_base_steering: bool = False,
    max_abs_delta_rad: float = 1.2,
    correction_rad: float = 0.2,
) -> Path:
    model = SpatialSteeringAdapterNp(
        input_dim=750,
        hidden_dim=128,
        projection_dim=128,
        use_speed=True,
        use_base_steering=use_base_steering,
        max_speed_mps=12.0,
        max_abs_delta_rad=max_abs_delta_rad,
    )
    weights = {key: value.copy() for key, value in model.params.items()}
    base = np.load(CHECKPOINT, allow_pickle=True).item()
    for key, value in base.items():
        weights[f"base_{key}"] = value.copy()
    if mismatch_base:
        weights["base_fc4_bias"][0] += 0.01
    weights["spatial_scale"].fill(1.0)
    weights["direction_head_bias"][:] = (-20.0, -20.0, 20.0)
    magnitude_fraction = correction_rad / max_abs_delta_rad
    weights["magnitude_head_bias"].fill(
        np.log(magnitude_fraction / (1.0 - magnitude_fraction))
    )
    checkpoint = tmp_path / "spatial-shadow.npy"
    np.save(checkpoint, weights)
    return checkpoint


def _write_recurrent_shadow_checkpoint(
    tmp_path: Path,
    *,
    mismatch_base: bool = False,
    correction_rad: float = 0.03,
) -> Path:
    model = RecurrentSteeringAdapterNp()
    weights = {key: value.copy() for key, value in model.params.items()}
    base = np.load(CHECKPOINT, allow_pickle=True).item()
    spatial = np.load(SPATIAL_CHECKPOINT, allow_pickle=True).item()
    for key, value in base.items():
        weights[f"base_{key}"] = value.copy()
    for key, value in spatial.items():
        weights[f"spatial_baseline_{key}"] = value.copy()
    if mismatch_base:
        weights["base_fc4_bias"][0] += 0.01
    weights["spatial_scale"].fill(1.0)
    weights["correction_output_bias"].fill(
        np.arctanh(correction_rad / model.max_abs_correction_rad)
    )
    checkpoint = tmp_path / "recurrent-shadow.npy"
    np.save(checkpoint, weights)
    return checkpoint


def test_base_conditioned_spatial_shadow_loads_with_trained_residual_scale(
    tmp_path: Path,
) -> None:
    shadow_checkpoint = _write_spatial_shadow_checkpoint(
        tmp_path,
        use_base_steering=True,
        max_abs_delta_rad=0.12,
        correction_rad=0.06,
    )
    base = _load_core()
    shadow = TinyLidarNetCore(
        input_dim=750,
        output_dim=2,
        architecture="normal",
        ckpt_path=str(CHECKPOINT),
        acceleration=0.6,
        control_mode="fixed",
        max_range=30.0,
        spatial_shadow_ckpt_path=str(shadow_checkpoint),
        spatial_shadow_use_base_steering=True,
        spatial_shadow_max_abs_delta_rad=0.12,
    )
    scan = np.linspace(1.0, 30.0, 750, dtype=np.float32)

    assert shadow.process(scan, speed_mps=4.0) == pytest.approx(
        base.process(scan), abs=0.0
    )
    assert shadow.last_spatial_shadow_admitted
    assert shadow.last_spatial_shadow_status == "ok"
    assert shadow.last_spatial_shadow_correction_rad == pytest.approx(
        0.06, abs=1e-6
    )


def test_shipped_checkpoint_matches_runtime_model() -> None:
    core = _load_core()
    assert core.loaded_parameter_count == len(core.model.params) == 18
    acceleration, steering = core.process(np.full(750, 30.0, dtype=np.float32))
    assert acceleration == pytest.approx(0.6)
    assert np.isfinite(steering)


def test_forward_speed_governor_is_an_explicit_wheel_speed_consumer() -> None:
    core = TinyLidarNetCore(
        input_dim=750,
        output_dim=2,
        architecture="normal",
        ckpt_path=str(CHECKPOINT),
        acceleration=0.8,
        maximum_forward_speed_mps=4.6,
        control_mode="fixed",
        max_range=30.0,
    )
    scan = np.full(750, 30.0, dtype=np.float32)

    assert core.requires_wheel_speed
    assert core.process(scan, speed_mps=3.0)[0] == pytest.approx(0.8)
    assert core.process(scan, speed_mps=4.4)[0] == pytest.approx(0.2)
    assert core.process(scan, speed_mps=4.7)[0] == pytest.approx(0.0)
    assert core.process(scan, speed_mps=None)[0] == pytest.approx(0.0)
    assert core.last_speed_governor_decision is not None
    assert core.last_speed_governor_decision.reason == "missing-or-stale-speed"


def test_forward_speed_governor_rejects_teacher_mode() -> None:
    with pytest.raises(ValueError, match="only supported by fixed control modes"):
        TinyLidarNetCore(
            input_dim=750,
            output_dim=2,
            architecture="normal",
            ckpt_path=str(CHECKPOINT),
            acceleration=0.8,
            maximum_forward_speed_mps=4.6,
            control_mode="gap_teacher",
            max_range=30.0,
        )


def test_production_backbone_shared_feature_path_is_bit_identical() -> None:
    core = _load_core()
    scans = np.stack(
        (
            np.linspace(0.0, 1.0, 750, dtype=np.float32),
            np.linspace(1.0, 0.0, 750, dtype=np.float32),
        )
    )[:, None, :]
    direct = core.model(scans)
    shared = core.model.forward_from_conv5_features(
        core.model.forward_conv5_features(scans)
    )
    np.testing.assert_array_equal(shared, direct)


def test_shipped_spatial_checkpoint_matches_promoted_contract() -> None:
    assert hashlib.sha256(SPATIAL_CHECKPOINT.read_bytes()).hexdigest() == (
        SPATIAL_CHECKPOINT_SHA256
    )
    core = TinyLidarNetCore(
        input_dim=750,
        output_dim=2,
        architecture="normal",
        ckpt_path=str(CHECKPOINT),
        acceleration=0.6,
        control_mode="fixed_lidar_brake",
        max_range=30.0,
        spatial_shadow_ckpt_path=str(SPATIAL_CHECKPOINT),
        spatial_shadow_expected_sha256=SPATIAL_CHECKPOINT_SHA256,
        spatial_shadow_use_base_steering=True,
        spatial_shadow_max_abs_delta_rad=1.2,
        spatial_authority_enabled=True,
        spatial_authority_max_abs_delta_rad=1.2,
    )
    acceleration, steering = core.process(
        np.full(750, 30.0, dtype=np.float32), speed_mps=3.0
    )
    assert core.spatial_shadow_loaded_parameter_count == 29
    assert core.last_spatial_shadow_status == "ok"
    assert core.last_spatial_authority_applied
    assert np.isfinite(acceleration)
    assert np.isfinite(steering)


def test_recurrent_shadow_preserves_production_output_and_tracks_state(
    tmp_path: Path,
) -> None:
    recurrent_checkpoint = _write_recurrent_shadow_checkpoint(tmp_path)
    production = TinyLidarNetCore(
        input_dim=750,
        output_dim=2,
        architecture="normal",
        ckpt_path=str(CHECKPOINT),
        acceleration=0.6,
        control_mode="fixed_lidar_brake",
        max_range=30.0,
        spatial_shadow_ckpt_path=str(SPATIAL_CHECKPOINT),
        spatial_shadow_expected_sha256=SPATIAL_CHECKPOINT_SHA256,
        spatial_shadow_use_base_steering=True,
        spatial_shadow_max_abs_delta_rad=1.2,
        spatial_authority_enabled=True,
        spatial_authority_max_abs_delta_rad=1.2,
    )
    shadow = TinyLidarNetCore(
        input_dim=750,
        output_dim=2,
        architecture="normal",
        ckpt_path=str(CHECKPOINT),
        acceleration=0.6,
        control_mode="fixed_lidar_brake",
        max_range=30.0,
        spatial_shadow_ckpt_path=str(SPATIAL_CHECKPOINT),
        spatial_shadow_expected_sha256=SPATIAL_CHECKPOINT_SHA256,
        spatial_shadow_use_base_steering=True,
        spatial_shadow_max_abs_delta_rad=1.2,
        spatial_authority_enabled=True,
        spatial_authority_max_abs_delta_rad=1.2,
        recurrent_shadow_ckpt_path=str(recurrent_checkpoint),
    )
    scans = [
        np.linspace(1.0 + index, 30.0, 750, dtype=np.float32)
        for index in range(3)
    ]

    for scan in scans:
        assert shadow.process(scan, speed_mps=3.0) == pytest.approx(
            production.process(scan, speed_mps=3.0), abs=0.0
        )
        assert shadow.last_recurrent_shadow_admitted
        assert shadow.last_recurrent_shadow_status == "ok"
        assert shadow.last_recurrent_shadow_correction_rad == pytest.approx(
            0.03, abs=1e-6
        )
    assert shadow.recurrent_shadow_hidden is not None
    assert shadow.last_recurrent_shadow_hidden_norm >= 0.0

    shadow.process(scans[-1], speed_mps=None)
    assert shadow.recurrent_shadow_hidden is None
    assert shadow.last_recurrent_shadow_status == "missing-or-stale-speed"
    assert shadow.recurrent_shadow_reset_count == 1


def test_recurrent_shadow_rejects_wrong_embedded_base(tmp_path: Path) -> None:
    recurrent_checkpoint = _write_recurrent_shadow_checkpoint(
        tmp_path, mismatch_base=True
    )
    with pytest.raises(ValueError, match="embedded base does not match"):
        TinyLidarNetCore(
            input_dim=750,
            output_dim=2,
            architecture="normal",
            ckpt_path=str(CHECKPOINT),
            acceleration=0.6,
            control_mode="fixed_lidar_brake",
            max_range=30.0,
            spatial_shadow_ckpt_path=str(SPATIAL_CHECKPOINT),
            spatial_shadow_use_base_steering=True,
            spatial_shadow_max_abs_delta_rad=1.2,
            spatial_authority_enabled=True,
            spatial_authority_max_abs_delta_rad=1.2,
            recurrent_shadow_ckpt_path=str(recurrent_checkpoint),
        )


def test_recurrent_authority_is_explicit_bounded_and_falls_back_to_spatial(
    tmp_path: Path,
) -> None:
    recurrent_checkpoint = _write_recurrent_shadow_checkpoint(
        tmp_path, correction_rad=0.03
    )
    common = {
        "input_dim": 750,
        "output_dim": 2,
        "architecture": "normal",
        "ckpt_path": str(CHECKPOINT),
        "acceleration": 0.6,
        "control_mode": "fixed_lidar_brake",
        "max_range": 30.0,
        "spatial_shadow_ckpt_path": str(SPATIAL_CHECKPOINT),
        "spatial_shadow_expected_sha256": SPATIAL_CHECKPOINT_SHA256,
        "spatial_shadow_use_base_steering": True,
        "spatial_shadow_max_abs_delta_rad": 1.2,
        "spatial_authority_enabled": True,
        "spatial_authority_max_abs_delta_rad": 1.2,
        "recurrent_shadow_ckpt_path": str(recurrent_checkpoint),
        "recurrent_shadow_expected_sha256": hashlib.sha256(
            recurrent_checkpoint.read_bytes()
        ).hexdigest(),
    }
    baseline = TinyLidarNetCore(**common)
    authority = TinyLidarNetCore(
        **common,
        recurrent_authority_enabled=True,
        recurrent_authority_max_abs_correction_rad=0.02,
    )
    scan = np.linspace(1.0, 30.0, 750, dtype=np.float32)
    baseline_command = baseline.process(scan, speed_mps=3.0)
    authority_command = authority.process(scan, speed_mps=3.0)

    assert authority_command[0] == pytest.approx(baseline_command[0], abs=0.0)
    assert authority_command[1] == pytest.approx(
        baseline_command[1] + 0.02, abs=1e-6
    )
    assert authority.last_recurrent_authority_applied
    assert authority.last_recurrent_authority_clipped
    assert authority.last_recurrent_authority_correction_rad == pytest.approx(
        0.02, abs=1e-7
    )

    fallback_command = authority.process(scan, speed_mps=None)
    fresh_spatial_command = baseline.process(scan, speed_mps=None)
    assert fallback_command == pytest.approx(fresh_spatial_command, abs=0.0)
    assert not authority.last_recurrent_authority_applied
    assert not authority.last_recurrent_authority_clipped


def test_recurrent_authority_requires_explicit_checkpoint() -> None:
    with pytest.raises(ValueError, match="requires an explicit recurrent"):
        TinyLidarNetCore(
            input_dim=750,
            output_dim=2,
            architecture="normal",
            ckpt_path=str(CHECKPOINT),
            recurrent_authority_enabled=True,
        )


def test_recurrent_authority_requires_checkpoint_identity(tmp_path: Path) -> None:
    recurrent_checkpoint = _write_recurrent_shadow_checkpoint(tmp_path)
    with pytest.raises(
        ValueError, match="requires an explicit recurrent checkpoint SHA256"
    ):
        TinyLidarNetCore(
            input_dim=750,
            output_dim=2,
            architecture="normal",
            ckpt_path=str(CHECKPOINT),
            spatial_shadow_ckpt_path=str(SPATIAL_CHECKPOINT),
            spatial_shadow_expected_sha256=SPATIAL_CHECKPOINT_SHA256,
            spatial_shadow_use_base_steering=True,
            spatial_shadow_max_abs_delta_rad=1.2,
            spatial_authority_enabled=True,
            spatial_authority_max_abs_delta_rad=1.2,
            recurrent_shadow_ckpt_path=str(recurrent_checkpoint),
            recurrent_authority_enabled=True,
        )


def test_recurrent_authority_rejects_bound_above_model(
    tmp_path: Path,
) -> None:
    recurrent_checkpoint = _write_recurrent_shadow_checkpoint(tmp_path)
    with pytest.raises(ValueError, match="no larger than the model"):
        TinyLidarNetCore(
            input_dim=750,
            output_dim=2,
            architecture="normal",
            ckpt_path=str(CHECKPOINT),
            spatial_shadow_ckpt_path=str(SPATIAL_CHECKPOINT),
            spatial_shadow_use_base_steering=True,
            spatial_shadow_max_abs_delta_rad=1.2,
            spatial_authority_enabled=True,
            spatial_authority_max_abs_delta_rad=1.2,
            recurrent_shadow_ckpt_path=str(recurrent_checkpoint),
            recurrent_shadow_expected_sha256=hashlib.sha256(
                recurrent_checkpoint.read_bytes()
            ).hexdigest(),
            recurrent_shadow_max_abs_correction_rad=0.64,
            recurrent_authority_enabled=True,
            recurrent_authority_max_abs_correction_rad=0.65,
        )


def test_missing_weight_is_rejected(tmp_path: Path) -> None:
    weights = np.load(CHECKPOINT, allow_pickle=True).item()
    weights.pop("fc4_bias")
    invalid = tmp_path / "missing.npy"
    np.save(invalid, weights)
    with pytest.raises(ValueError, match="weight key mismatch"):
        _load_core(invalid)


def test_weight_shape_mismatch_is_rejected(tmp_path: Path) -> None:
    weights = np.load(CHECKPOINT, allow_pickle=True).item()
    weights["fc1_weight"] = weights["fc1_weight"][:, :-1]
    invalid = tmp_path / "shape.npy"
    np.save(invalid, weights)
    with pytest.raises(ValueError, match="weight shape mismatch for fc1_weight"):
        _load_core(invalid)


def test_non_finite_weight_is_rejected(tmp_path: Path) -> None:
    weights = np.load(CHECKPOINT, allow_pickle=True).item()
    weights["fc4_bias"] = weights["fc4_bias"].copy()
    weights["fc4_bias"][0] = np.nan
    invalid = tmp_path / "nan.npy"
    np.save(invalid, weights)
    with pytest.raises(ValueError, match="contains non-finite"):
        _load_core(invalid)


def test_empty_scan_is_rejected() -> None:
    with pytest.raises(ValueError, match="non-empty 1D"):
        _load_core().process(np.array([], dtype=np.float32))


def test_random_production_weights_are_forbidden() -> None:
    with pytest.raises(ValueError, match="ckpt_path is required"):
        TinyLidarNetCore(
            input_dim=750,
            output_dim=2,
            architecture="normal",
            ckpt_path="",
        )


def test_fixed_acceleration_outside_command_contract_is_rejected() -> None:
    with pytest.raises(ValueError, match="within"):
        TinyLidarNetCore(
            input_dim=750,
            output_dim=2,
            architecture="normal",
            ckpt_path=str(CHECKPOINT),
            acceleration=1.1,
            control_mode="fixed",
        )


def test_gap_teacher_mode_is_explicit_and_keeps_finite_contract() -> None:
    core = TinyLidarNetCore(
        input_dim=750,
        output_dim=2,
        architecture="normal",
        ckpt_path=str(CHECKPOINT),
        acceleration=0.6,
        control_mode="gap_teacher",
        max_range=30.0,
    )
    acceleration, steering = core.process(
        np.full(750, 30.0, dtype=np.float32)
    )
    assert acceleration == pytest.approx(0.6)
    assert np.isfinite(steering)
    assert core.last_gap_teacher_decision is not None
    assert not core.last_gap_teacher_decision.active


def test_precontact_teacher_mode_is_explicit_and_keeps_finite_contract() -> None:
    core = TinyLidarNetCore(
        input_dim=750,
        output_dim=2,
        architecture="normal",
        ckpt_path=str(CHECKPOINT),
        acceleration=0.6,
        control_mode="precontact_teacher",
        max_range=30.0,
    )
    acceleration, steering = core.process(
        np.full(750, 30.0, dtype=np.float32)
    )
    assert acceleration == pytest.approx(0.6)
    assert np.isfinite(steering)
    assert core.last_gap_teacher_decision is not None
    assert not core.last_gap_teacher_decision.active


def test_speed_committed_teacher_requires_and_uses_wheel_speed() -> None:
    core = TinyLidarNetCore(
        input_dim=750,
        output_dim=2,
        architecture="normal",
        ckpt_path=str(CHECKPOINT),
        acceleration=0.6,
        control_mode="speed_committed_teacher",
        max_range=30.0,
    )
    assert core.requires_wheel_speed
    with pytest.raises(ValueError, match="requires fresh wheel speed"):
        core.process(np.full(750, 30.0, dtype=np.float32))

    acceleration, steering = core.process(
        np.full(750, 30.0, dtype=np.float32), speed_mps=3.0
    )
    assert acceleration == pytest.approx(0.6)
    assert np.isfinite(steering)
    assert core.last_gap_teacher_decision is not None
    assert core.last_gap_teacher_decision.speed_mps == pytest.approx(3.0)


def test_fixed_lidar_brake_preserves_network_steering_and_limits_acceleration() -> None:
    fixed_core = _load_core()
    safe_core = TinyLidarNetCore(
        input_dim=750,
        output_dim=2,
        architecture="normal",
        ckpt_path=str(CHECKPOINT),
        acceleration=0.6,
        control_mode="fixed_lidar_brake",
        max_range=30.0,
    )
    scan = np.full(750, 1.0, dtype=np.float32)
    fixed_acceleration, fixed_steering = fixed_core.process(scan)
    safe_acceleration, safe_steering = safe_core.process(scan)
    assert fixed_acceleration == pytest.approx(0.6)
    assert safe_acceleration == pytest.approx(-1.0)
    assert safe_steering == pytest.approx(fixed_steering)
    assert safe_core.last_longitudinal_safety_decision is not None
    assert safe_core.last_longitudinal_safety_decision.active


def test_lidar_brake_remains_final_owner_after_speed_governor() -> None:
    core = TinyLidarNetCore(
        input_dim=750,
        output_dim=2,
        architecture="normal",
        ckpt_path=str(CHECKPOINT),
        acceleration=0.8,
        maximum_forward_speed_mps=4.6,
        control_mode="fixed_lidar_brake",
        max_range=30.0,
    )

    acceleration, _ = core.process(
        np.full(750, 1.0, dtype=np.float32), speed_mps=4.7
    )

    assert core.last_speed_governor_decision is not None
    assert core.last_speed_governor_decision.acceleration_mps2 == pytest.approx(0.0)
    assert core.last_longitudinal_safety_decision is not None
    assert core.last_longitudinal_safety_decision.active
    assert acceleration == pytest.approx(-1.0)


def test_speed_aware_lidar_brake_preserves_lateral_authority_and_bounds_restart() -> None:
    legacy = TinyLidarNetCore(
        input_dim=750,
        output_dim=2,
        architecture="normal",
        ckpt_path=str(CHECKPOINT),
        acceleration=0.8,
        maximum_forward_speed_mps=4.6,
        control_mode="fixed_lidar_brake",
        max_range=30.0,
    )
    candidate = TinyLidarNetCore(
        input_dim=750,
        output_dim=2,
        architecture="normal",
        ckpt_path=str(CHECKPOINT),
        acceleration=0.8,
        maximum_forward_speed_mps=4.6,
        control_mode="speed_aware_lidar_brake",
        max_range=30.0,
    )
    scan = np.full(750, 1.7, dtype=np.float32)

    legacy_acceleration, legacy_steering = legacy.process(scan, speed_mps=0.0)
    acceleration, steering = candidate.process(scan, speed_mps=0.0)

    assert legacy_acceleration == pytest.approx(0.0)
    assert 0.0 < acceleration < 0.8
    assert steering == pytest.approx(legacy_steering, abs=0.0)
    assert candidate.last_longitudinal_safety_decision is not None
    assert candidate.last_longitudinal_safety_decision.reason == "safe-speed-limit"


def test_speed_aware_lidar_brake_is_fail_safe_without_fresh_speed() -> None:
    core = TinyLidarNetCore(
        input_dim=750,
        output_dim=2,
        architecture="normal",
        ckpt_path=str(CHECKPOINT),
        acceleration=0.8,
        control_mode="speed_aware_lidar_brake",
        max_range=30.0,
    )
    acceleration, _ = core.process(
        np.full(750, 2.0, dtype=np.float32), speed_mps=None
    )
    assert core.requires_wheel_speed
    assert acceleration == pytest.approx(0.0)
    assert core.last_longitudinal_safety_decision is not None
    assert (
        core.last_longitudinal_safety_decision.reason
        == "missing-or-stale-speed"
    )


def test_unknown_control_mode_is_rejected() -> None:
    with pytest.raises(ValueError, match="control_mode"):
        TinyLidarNetCore(
            input_dim=750,
            output_dim=2,
            architecture="normal",
            ckpt_path=str(CHECKPOINT),
            acceleration=0.6,
            control_mode="gap-ish",
            max_range=30.0,
        )


def test_numpy_runtime_matches_reference_forward_pass() -> None:
    weights = np.load(CHECKPOINT, allow_pickle=True).item()
    model = TinyLidarNetNp(input_dim=750, output_dim=2)
    model.params.update(weights)
    sample = np.linspace(0.0, 1.0, 750, dtype=np.float32)[None, None, :]
    output = model(sample)
    assert output.shape == (1, 2)
    assert np.all(np.isfinite(output))


def test_optional_residual_composes_with_frozen_base(tmp_path: Path) -> None:
    residual_model = SteeringResidualNetNp(input_dim=750)
    weights = {key: value.copy() for key, value in residual_model.params.items()}
    weights["correction_head_bias"][0] = np.arctanh(0.2 / 1.28)
    weights["gate_head_bias"][0] = 20.0
    residual_checkpoint = tmp_path / "residual.npy"
    np.save(residual_checkpoint, weights)

    base = _load_core()
    composed = TinyLidarNetCore(
        input_dim=750,
        output_dim=2,
        architecture="normal",
        ckpt_path=str(CHECKPOINT),
        residual_ckpt_path=str(residual_checkpoint),
        acceleration=0.6,
        control_mode="fixed",
        max_range=30.0,
    )
    scan = np.full(750, 30.0, dtype=np.float32)
    _, base_steering = base.process(scan)
    _, composed_steering = composed.process(scan)
    assert composed.residual_loaded_parameter_count == 14
    assert composed.last_residual_correction_rad == pytest.approx(0.2, abs=1e-6)
    assert composed.last_residual_gate_probability > 0.999
    assert composed_steering == pytest.approx(base_steering + 0.2, abs=1e-6)


def test_partial_residual_checkpoint_is_rejected(tmp_path: Path) -> None:
    residual_model = SteeringResidualNetNp(input_dim=750)
    weights = {key: value.copy() for key, value in residual_model.params.items()}
    weights.pop("gate_head_bias")
    residual_checkpoint = tmp_path / "partial-residual.npy"
    np.save(residual_checkpoint, weights)
    with pytest.raises(ValueError, match="weight key mismatch"):
        TinyLidarNetCore(
            input_dim=750,
            output_dim=2,
            architecture="normal",
            ckpt_path=str(CHECKPOINT),
            residual_ckpt_path=str(residual_checkpoint),
        )


def test_scan_delta_residual_checkpoint_and_history_are_explicit(
    tmp_path: Path,
) -> None:
    residual_model = SteeringResidualNetNp(input_dim=750, input_channels=2)
    residual_checkpoint = tmp_path / "scan-delta-residual.npy"
    np.save(
        residual_checkpoint,
        {key: value.copy() for key, value in residual_model.params.items()},
    )
    core = TinyLidarNetCore(
        input_dim=750,
        output_dim=2,
        architecture="normal",
        ckpt_path=str(CHECKPOINT),
        residual_ckpt_path=str(residual_checkpoint),
        residual_architecture="scan_delta",
        acceleration=0.6,
        control_mode="fixed",
        max_range=30.0,
    )
    first = core._compose_residual_input(np.full(750, 0.2, dtype=np.float32))
    second = core._compose_residual_input(np.full(750, 0.4, dtype=np.float32))
    assert first.shape == (1, 2, 750)
    np.testing.assert_allclose(first[:, 1], 0.0)
    np.testing.assert_allclose(second[:, 0], 0.4)
    np.testing.assert_allclose(second[:, 1], 0.2)
    core.reset_residual_history()
    reset = core._compose_residual_input(np.full(750, 0.4, dtype=np.float32))
    np.testing.assert_allclose(reset[:, 1], 0.0)


def test_residual_architecture_mismatch_is_rejected(tmp_path: Path) -> None:
    stateless = SteeringResidualNetNp(input_dim=750, input_channels=1)
    residual_checkpoint = tmp_path / "stateless-residual.npy"
    np.save(residual_checkpoint, stateless.params)
    with pytest.raises(ValueError, match="weight shape mismatch for conv1_weight"):
        TinyLidarNetCore(
            input_dim=750,
            output_dim=2,
            architecture="normal",
            ckpt_path=str(CHECKPOINT),
            residual_ckpt_path=str(residual_checkpoint),
            residual_architecture="scan_delta",
        )


def test_unknown_residual_architecture_is_rejected() -> None:
    with pytest.raises(ValueError, match="residual_architecture"):
        TinyLidarNetCore(
            input_dim=750,
            output_dim=2,
            architecture="normal",
            ckpt_path=str(CHECKPOINT),
            residual_architecture="maybe-temporal",
        )


def test_spatial_shadow_observes_but_never_changes_production_command(
    tmp_path: Path,
) -> None:
    shadow_checkpoint = _write_spatial_shadow_checkpoint(tmp_path)
    base = _load_core()
    shadow = TinyLidarNetCore(
        input_dim=750,
        output_dim=2,
        architecture="normal",
        ckpt_path=str(CHECKPOINT),
        acceleration=0.6,
        control_mode="fixed",
        max_range=30.0,
        spatial_shadow_ckpt_path=str(shadow_checkpoint),
    )
    scan = np.linspace(1.0, 30.0, 750, dtype=np.float32)

    base_command = base.process(scan)
    shadow_command = shadow.process(scan, speed_mps=4.0)

    assert shadow.spatial_shadow_loaded_parameter_count == 29
    assert shadow_command == pytest.approx(base_command, abs=0.0)
    assert shadow.last_spatial_shadow_admitted
    assert shadow.last_spatial_shadow_status == "ok"
    assert shadow.last_spatial_shadow_correction_rad == pytest.approx(
        0.2, abs=1e-6
    )
    np.testing.assert_allclose(
        shadow.last_spatial_shadow_direction_probabilities,
        np.asarray([0.0, 0.0, 1.0], dtype=np.float32),
        atol=1e-6,
    )


def test_spatial_shadow_missing_speed_skips_without_zero_substitution(
    tmp_path: Path,
) -> None:
    shadow_checkpoint = _write_spatial_shadow_checkpoint(tmp_path)
    base = _load_core()
    shadow = TinyLidarNetCore(
        input_dim=750,
        output_dim=2,
        architecture="normal",
        ckpt_path=str(CHECKPOINT),
        acceleration=0.6,
        control_mode="fixed",
        max_range=30.0,
        spatial_shadow_ckpt_path=str(shadow_checkpoint),
    )
    scan = np.full(750, 15.0, dtype=np.float32)

    assert shadow.process(scan) == pytest.approx(base.process(scan), abs=0.0)
    assert not shadow.last_spatial_shadow_admitted
    assert shadow.last_spatial_shadow_status == "missing-or-stale-speed"


def test_spatial_shadow_embedded_base_mismatch_is_rejected(
    tmp_path: Path,
) -> None:
    shadow_checkpoint = _write_spatial_shadow_checkpoint(
        tmp_path, mismatch_base=True
    )
    with pytest.raises(ValueError, match="embedded base does not match"):
        TinyLidarNetCore(
            input_dim=750,
            output_dim=2,
            architecture="normal",
            ckpt_path=str(CHECKPOINT),
            spatial_shadow_ckpt_path=str(shadow_checkpoint),
        )


def test_spatial_shadow_expected_sha256_accepts_exact_artifact(
    tmp_path: Path,
) -> None:
    shadow_checkpoint = _write_spatial_shadow_checkpoint(tmp_path)
    expected = hashlib.sha256(shadow_checkpoint.read_bytes()).hexdigest()
    core = TinyLidarNetCore(
        input_dim=750,
        output_dim=2,
        architecture="normal",
        ckpt_path=str(CHECKPOINT),
        spatial_shadow_ckpt_path=str(shadow_checkpoint),
        spatial_shadow_expected_sha256=expected,
    )
    assert core.spatial_shadow_loaded_parameter_count == 29


def test_spatial_shadow_expected_sha256_rejects_wrong_artifact(
    tmp_path: Path,
) -> None:
    shadow_checkpoint = _write_spatial_shadow_checkpoint(tmp_path)
    with pytest.raises(ValueError, match="SHA256 mismatch"):
        TinyLidarNetCore(
            input_dim=750,
            output_dim=2,
            architecture="normal",
            ckpt_path=str(CHECKPOINT),
            spatial_shadow_ckpt_path=str(shadow_checkpoint),
            spatial_shadow_expected_sha256="0" * 64,
        )


def test_spatial_shadow_expected_sha256_requires_checkpoint() -> None:
    with pytest.raises(ValueError, match="requires a spatial shadow checkpoint"):
        TinyLidarNetCore(
            input_dim=750,
            output_dim=2,
            architecture="normal",
            ckpt_path=str(CHECKPOINT),
            spatial_shadow_expected_sha256="0" * 64,
        )


def test_spatial_shadow_inference_failure_isolated_from_production(
    tmp_path: Path,
) -> None:
    shadow_checkpoint = _write_spatial_shadow_checkpoint(tmp_path)
    base = _load_core()
    shadow = TinyLidarNetCore(
        input_dim=750,
        output_dim=2,
        architecture="normal",
        ckpt_path=str(CHECKPOINT),
        acceleration=0.6,
        control_mode="fixed",
        max_range=30.0,
        spatial_shadow_ckpt_path=str(shadow_checkpoint),
    )
    shadow.spatial_shadow_model.params["spatial_scale"].fill(0.0)
    scan = np.full(750, 20.0, dtype=np.float32)

    assert shadow.process(scan, speed_mps=3.0) == pytest.approx(
        base.process(scan), abs=0.0
    )
    assert not shadow.last_spatial_shadow_admitted
    assert shadow.last_spatial_shadow_status.startswith(
        "inference-error:ValueError:"
    )


def test_spatial_authority_is_explicit_bounded_and_uses_fresh_inference(
    tmp_path: Path,
) -> None:
    shadow_checkpoint = _write_spatial_shadow_checkpoint(tmp_path)
    base = _load_core()
    authority = TinyLidarNetCore(
        input_dim=750,
        output_dim=2,
        architecture="normal",
        ckpt_path=str(CHECKPOINT),
        acceleration=0.6,
        control_mode="fixed",
        max_range=30.0,
        spatial_shadow_ckpt_path=str(shadow_checkpoint),
        spatial_authority_enabled=True,
        spatial_authority_max_abs_delta_rad=0.12,
    )
    scan = np.linspace(1.0, 30.0, 750, dtype=np.float32)
    base_command = base.process(scan)

    authority_command = authority.process(scan, speed_mps=4.0)
    assert authority_command[0] == pytest.approx(base_command[0], abs=0.0)
    assert authority_command[1] == pytest.approx(
        base_command[1] + 0.12, abs=1e-6
    )
    assert authority.last_spatial_authority_applied
    assert authority.last_spatial_authority_clipped
    assert authority.last_spatial_authority_correction_rad == pytest.approx(
        0.12, abs=1e-7
    )

    missing_speed_command = authority.process(scan)
    assert missing_speed_command == pytest.approx(base_command, abs=0.0)
    assert not authority.last_spatial_authority_applied
    assert not authority.last_spatial_authority_clipped


def test_spatial_authority_requires_checkpoint() -> None:
    with pytest.raises(ValueError, match="requires an explicit"):
        TinyLidarNetCore(
            input_dim=750,
            output_dim=2,
            architecture="normal",
            ckpt_path=str(CHECKPOINT),
            spatial_authority_enabled=True,
        )


def test_spatial_authority_rejects_second_learned_steering_owner(
    tmp_path: Path,
) -> None:
    shadow_checkpoint = _write_spatial_shadow_checkpoint(tmp_path)
    residual_model = SteeringResidualNetNp(input_dim=750)
    residual_checkpoint = tmp_path / "residual.npy"
    np.save(residual_checkpoint, residual_model.params)
    with pytest.raises(ValueError, match="cannot own steering together"):
        TinyLidarNetCore(
            input_dim=750,
            output_dim=2,
            architecture="normal",
            ckpt_path=str(CHECKPOINT),
            residual_ckpt_path=str(residual_checkpoint),
            spatial_shadow_ckpt_path=str(shadow_checkpoint),
            spatial_authority_enabled=True,
        )
