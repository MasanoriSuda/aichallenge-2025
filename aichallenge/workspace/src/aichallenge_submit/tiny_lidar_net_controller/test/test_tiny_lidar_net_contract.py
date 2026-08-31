#!/usr/bin/env python3
"""Contract tests for the E2E TinyLidarNet deployment artifact."""

from pathlib import Path

import numpy as np
import pytest

from tiny_lidar_net_controller.tiny_lidar_net_controller_core import TinyLidarNetCore
from tiny_lidar_net_controller.model.tinylidarnet import TinyLidarNetNp


PACKAGE_ROOT = Path(__file__).parents[1]
CHECKPOINT = PACKAGE_ROOT / "ckpt" / "tinylidarnet_weights.npy"


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


def test_shipped_checkpoint_matches_runtime_model() -> None:
    core = _load_core()
    assert core.loaded_parameter_count == len(core.model.params) == 18
    acceleration, steering = core.process(np.full(750, 30.0, dtype=np.float32))
    assert acceleration == pytest.approx(0.6)
    assert np.isfinite(steering)


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
