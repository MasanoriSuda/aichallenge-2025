import importlib.util
from pathlib import Path

import numpy as np
import pytest


MODULE_PATH = Path(__file__).resolve().parents[1] / "probe_e2e_action_separability.py"
SPEC = importlib.util.spec_from_file_location("probe_e2e_action_separability", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_action_classes_use_material_correction_sign():
    labels = MODULE.action_classes(
        np.asarray([-0.2, 0.01, 0.3]),
        np.asarray([0.0, 0.0, 0.1]),
        material_delta_rad=0.02,
    )

    assert labels.tolist() == [0, 1, 2]


def test_temporal_features_reset_history_at_sequence_start():
    current = np.asarray([[1.0], [2.0], [4.0]], dtype=np.float32)
    speed = np.asarray([1.0, 2.0, 3.0], dtype=np.float32)

    features = MODULE.temporal_features(current, speed, lags=(1, 2), max_speed_mps=10.0)

    assert features.shape == (3, 6)
    assert features[0].tolist() == pytest.approx([1.0, 0.0, 0.0, 0.1, 0.0, 0.0])
    assert features[2].tolist() == pytest.approx([4.0, 2.0, 3.0, 0.3, 0.1, 0.2])


def test_probe_metrics_separate_material_and_anchor_errors():
    report = MODULE.probe_metrics(
        np.asarray([0, 2, 0, 2]),
        np.asarray([0, 1, 2, 2]),
    )

    assert report["accuracy"] == pytest.approx(0.5)
    assert report["material_sign_accuracy"] == pytest.approx(2.0 / 3.0)
    assert report["anchor_false_material_fraction"] == pytest.approx(1.0)


def test_standardization_uses_train_only_statistics():
    train = np.asarray([[0.0], [2.0]], dtype=np.float32)
    validation = np.asarray([[3.0]], dtype=np.float32)

    normalized_train, normalized_validation, _ = MODULE.standardize_from_train(
        train, validation
    )

    assert normalized_train[:, 0].tolist() == pytest.approx([-1.0, 1.0])
    assert normalized_validation[0, 0] == pytest.approx(2.0)


def test_static_spatial_no_speed_does_not_invent_odometry_input():
    projected = np.asarray([[1.0, 2.0], [3.0, 4.0]], dtype=np.float32)
    compact = np.zeros((2, 10), dtype=np.float32)

    features = MODULE.compose_probe_features(
        "static_conv5_no_speed",
        projected,
        compact,
        np.asarray([1.0, 9.0], dtype=np.float32),
    )

    assert np.array_equal(features, projected)


def test_spatial_history_resets_each_sequence_without_speed_columns():
    current = np.asarray([[1.0], [2.0], [4.0]], dtype=np.float32)

    features = MODULE.spatial_history_features(current, lags=(1, 2))

    assert features.shape == (3, 3)
    assert features[0].tolist() == pytest.approx([1.0, 0.0, 0.0])
    assert features[2].tolist() == pytest.approx([4.0, 2.0, 3.0])


def test_static_base_conditioning_appends_embedded_base_steering():
    projected = np.asarray([[1.0, 2.0], [3.0, 4.0]], dtype=np.float32)
    compact = np.zeros((2, 10), dtype=np.float32)

    features = MODULE.compose_probe_features(
        "static_conv5_base",
        projected,
        compact,
        np.asarray([3.0, 6.0], dtype=np.float32),
        base_steering_rad=np.asarray([-0.2, 0.4], dtype=np.float32),
    )

    assert features.shape == (2, 4)
    assert features[0].tolist() == pytest.approx([1.0, 2.0, 0.25, -0.2])
    assert features[1].tolist() == pytest.approx([3.0, 4.0, 0.5, 0.4])


def test_temporal_base_conditioning_is_causal_and_tracks_base_delta():
    projected = np.asarray([[1.0], [2.0], [4.0]], dtype=np.float32)
    compact = np.zeros((3, 10), dtype=np.float32)

    features = MODULE.compose_probe_features(
        "temporal_conv5_base",
        projected,
        compact,
        np.asarray([1.0, 2.0, 4.0], dtype=np.float32),
        base_steering_rad=np.asarray([0.1, 0.2, -0.3], dtype=np.float32),
    )

    assert features.shape == (3, 9)
    assert features[0].tolist() == pytest.approx(
        [1.0, 0.1, 0.0, 0.0, 0.0, 0.0, 1.0 / 12.0, 0.0, 0.0]
    )
    assert features[2].tolist() == pytest.approx(
        [
            4.0,
            -0.3,
            2.0,
            -0.5,
            3.0,
            -0.4,
            4.0 / 12.0,
            2.0 / 12.0,
            3.0 / 12.0,
        ]
    )


def test_base_conditioned_probe_rejects_missing_base_input():
    with pytest.raises(ValueError, match="requires base steering"):
        MODULE.compose_probe_features(
            "static_conv5_base",
            np.zeros((2, 1), dtype=np.float32),
            np.zeros((2, 10), dtype=np.float32),
            np.zeros(2, dtype=np.float32),
        )


def test_static_raw_normalizes_physical_scan_once_and_appends_speed():
    projected = np.zeros((2, 2), dtype=np.float32)
    compact = np.zeros((2, 10), dtype=np.float32)
    scans = np.vstack(
        (
            np.full(750, 15.0, dtype=np.float32),
            np.full(750, 30.0, dtype=np.float32),
        )
    )

    features = MODULE.compose_probe_features(
        "static_raw",
        projected,
        compact,
        np.asarray([3.0, 6.0], dtype=np.float32),
        scans,
    )

    assert features.shape == (2, 751)
    assert features[0, :750].tolist() == pytest.approx([0.5] * 750)
    assert features[1, :750].tolist() == pytest.approx([1.0] * 750)
    assert features[:, -1].tolist() == pytest.approx([0.25, 0.5])


def test_static_geometry_preserves_binned_min_mean_speed_and_base():
    projected = np.zeros((2, 2), dtype=np.float32)
    compact = np.zeros((2, 10), dtype=np.float32)
    scans = np.vstack(
        (
            np.full(750, 3.0, dtype=np.float32),
            np.full(750, 15.0, dtype=np.float32),
        )
    )

    features = MODULE.compose_probe_features(
        "static_geometry_base",
        projected,
        compact,
        np.asarray([6.0, 12.0], dtype=np.float32),
        scans,
        np.asarray([-0.2, 0.3], dtype=np.float32),
    )

    assert features.shape == (2, 102)
    assert np.allclose(features[0, :100], 0.1)
    assert features[0, 100:].tolist() == pytest.approx([0.5, -0.2])
    assert np.allclose(features[1, :100], 0.5)
    assert features[1, 100:].tolist() == pytest.approx([1.0, 0.3])


def test_temporal_raw_history_is_causal_and_resets_at_sequence_start():
    projected = np.zeros((3, 1), dtype=np.float32)
    compact = np.zeros((3, 10), dtype=np.float32)
    scans = np.vstack(
        [np.full(750, value, dtype=np.float32) for value in (3.0, 6.0, 12.0)]
    )

    features = MODULE.compose_probe_features(
        "temporal_raw",
        projected,
        compact,
        np.asarray([1.0, 2.0, 4.0], dtype=np.float32),
        scans,
    )

    assert features.shape == (3, 2253)
    assert np.allclose(features[0, 750:2250], 0.0)
    assert features[2, 0] == pytest.approx(0.4)
    assert features[2, 750] == pytest.approx(0.2)
    assert features[2, 1500] == pytest.approx(0.3)
    assert features[2, -3:].tolist() == pytest.approx(
        [4.0 / 12.0, 2.0 / 12.0, 3.0 / 12.0]
    )


def test_sequence_balanced_probe_weights_give_each_run_equal_mass():
    sequences = [
        MODULE.ProbeSequence(
            sequence_id="short",
            source_bag="short",
            features=np.zeros((2, 1), dtype=np.float32),
            labels=np.ones(2, dtype=np.int64),
        ),
        MODULE.ProbeSequence(
            sequence_id="long",
            source_bag="long",
            features=np.zeros((8, 1), dtype=np.float32),
            labels=np.ones(8, dtype=np.int64),
        ),
    ]

    weights = MODULE.probe_training_sample_weights(sequences, "sequence")

    assert np.sum(weights[:2]) == pytest.approx(0.5)
    assert np.sum(weights[2:]) == pytest.approx(0.5)
    assert MODULE.probe_training_sample_weights(sequences, "sample") is None
