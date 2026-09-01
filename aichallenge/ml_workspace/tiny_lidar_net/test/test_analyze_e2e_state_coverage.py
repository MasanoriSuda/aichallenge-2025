import importlib.util
from pathlib import Path

import numpy as np
import pytest


MODULE_PATH = Path(__file__).resolve().parents[1] / "analyze_e2e_state_coverage.py"
SPEC = importlib.util.spec_from_file_location("analyze_e2e_state_coverage", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def scans(values):
    return np.repeat(np.asarray(values, dtype=np.float32)[:, None], 750, axis=1)


def test_geometry_features_preserve_near_and_average_range():
    source = scans([3.0, 15.0])
    features = MODULE.geometry_features(source, bins=50)

    assert features.shape == (2, 100)
    assert np.allclose(features[0], 0.1)
    assert np.allclose(features[1], 0.5)


def test_nearest_indices_are_sorted_by_distance():
    reference = np.asarray([[0.0], [2.0], [5.0]], dtype=np.float32)
    query = np.asarray([[1.5]], dtype=np.float32)

    indices, distances = MODULE.nearest_indices(query, reference, 2)

    assert indices.tolist() == [[1, 0]]
    assert distances[0].tolist() == pytest.approx([0.5, 1.5])


def test_cross_sequence_baseline_excludes_same_run_neighbours():
    features = np.asarray([[0.0], [0.1], [2.0], [2.1]], dtype=np.float32)
    sequence_index = np.asarray([0, 0, 1, 1], dtype=np.int32)

    distances = MODULE.cross_sequence_baseline(features, sequence_index, 4)

    assert distances.tolist() == pytest.approx([2.0, 1.9, 1.9, 2.0])


def test_action_ambiguity_requires_opposite_material_actions():
    neighbour_indices = np.asarray([[0, 1, 2], [0, 2, 3]])
    neighbour_distances = np.asarray([[0.1, 0.2, 0.3], [0.1, 0.2, 0.3]])
    teacher = np.asarray([0.2, -0.3, 0.01, 0.4])

    report = MODULE.action_ambiguity(
        neighbour_indices,
        neighbour_distances,
        teacher,
        distance_limit=0.25,
        action_threshold_rad=0.05,
    )

    assert report["ambiguous_fraction"] == pytest.approx(0.5)
    assert report["usable_distinct_sequence_neighbours_mean"] == pytest.approx(2.0)


def test_deterministic_subsample_includes_boundaries():
    selected = MODULE.deterministic_subsample_indices(100, 5)

    assert selected.tolist() == [0, 24, 49, 74, 99]
