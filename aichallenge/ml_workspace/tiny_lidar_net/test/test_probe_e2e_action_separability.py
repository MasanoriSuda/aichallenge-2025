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
