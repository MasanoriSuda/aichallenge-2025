import importlib.util
from pathlib import Path
import sys

import numpy as np
import pytest


MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / "audit_spatial_label_observability.py"
)
SPEC = importlib.util.spec_from_file_location(
    "audit_spatial_label_observability", MODULE_PATH
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_deterministic_subsample_includes_both_boundaries():
    selected = MODULE.deterministic_subsample_indices(10, 4)

    assert selected.tolist() == [0, 3, 6, 9]


def test_nearest_distances_return_exact_neighbour_identity():
    reference = np.asarray([[0.0, 0.0], [2.0, 0.0], [5.0, 0.0]])
    query = np.asarray([[1.8, 0.0], [4.0, 0.0]])

    indices, distances = MODULE.nearest_distances(query, reference, 1)

    assert indices.tolist() == [1, 2]
    assert distances.tolist() == pytest.approx([0.2, 1.0])


def test_cross_sequence_baseline_excludes_adjacent_same_run_samples():
    features = np.asarray([[0.0], [0.1], [2.0], [2.1]], dtype=np.float32)
    identities = np.asarray([0, 0, 1, 1], dtype=np.int32)

    distances = MODULE.cross_sequence_nearest_distances(
        features, identities, maximum_queries=4, batch_size=2
    )

    assert distances.tolist() == pytest.approx([2.0, 1.9, 1.9, 2.0])


def test_physical_geometry_preserves_near_range_speed_and_base():
    scans = np.vstack((np.full(750, 3.0), np.full(750, 15.0))).astype(
        np.float32
    )
    features = MODULE.physical_geometry_features(
        scans,
        np.asarray([6.0, 12.0], dtype=np.float32),
        np.asarray([-0.2, 0.3], dtype=np.float32),
        max_speed_mps=12.0,
    )

    assert features.shape == (2, 102)
    assert np.allclose(features[0, :100], 0.1)
    assert features[0, 100:].tolist() == pytest.approx([0.5, -0.2])
    assert np.allclose(features[1, :100], 0.5)
    assert features[1, 100:].tolist() == pytest.approx([1.0, 0.3])


def test_conflict_summary_uses_normal_cross_run_scale():
    report = MODULE.conflict_summary(
        np.asarray([0.5, 1.5, 3.0]),
        np.asarray([1.0, 2.0, 3.0, 4.0]),
    )

    assert report["within_normal_cross_run_p50_fraction"] == pytest.approx(2 / 3)
    assert report["within_normal_cross_run_p95_fraction"] == pytest.approx(1.0)
