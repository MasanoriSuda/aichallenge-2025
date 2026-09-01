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


def test_sample_corpus_preserves_original_sequence_local_indices():
    class Sequence:
        def __init__(self):
            self.sequence_id = "run-a"
            self.metadata = {"source": {"bag": "/output/run-a/d1/bag"}}
            self.scans = np.arange(7500, dtype=np.float32).reshape(10, 750)
            self.speeds = np.arange(10, dtype=np.float32)
            self.base_steers = np.zeros(10, dtype=np.float32)
            self.steers = np.arange(10, dtype=np.float32) / 100.0

        def __len__(self):
            return len(self.scans)

    class Source:
        datasets = [Sequence()]

    sampled = MODULE.sample_corpus(Source(), "teacher", 4)

    assert sampled.sample_index.tolist() == [0, 3, 6, 9]
    assert sampled.sequence_lengths == (10,)
    assert sampled.source_bags == ("/output/run-a/d1/bag",)
    assert sampled.corrections_rad.tolist() == pytest.approx([0.0, 0.03, 0.06, 0.09])


def test_per_sequence_conflict_reports_causal_tail_separately():
    corpus = MODULE.SampledCorpus(
        name="teacher",
        scans_m=np.zeros((4, 750), dtype=np.float32),
        speeds_mps=np.zeros(4, dtype=np.float32),
        corrections_rad=np.asarray([0.1, 0.2, -0.1, -0.2], dtype=np.float32),
        sample_index=np.asarray([0, 9, 0, 9], dtype=np.int64),
        sequence_index=np.asarray([0, 0, 1, 1], dtype=np.int32),
        sequence_ids=("run-a", "run-b"),
        sequence_lengths=(10, 10),
        source_bags=("/a", "/b"),
    )

    report = MODULE.per_sequence_conflict_summary(
        np.asarray([0.5, 2.5, 1.5, 3.5]),
        np.arange(4),
        corpus,
        np.asarray([1.0, 2.0, 3.0, 4.0]),
        tail_samples=2,
    )

    assert report[0]["queries"]["within_normal_cross_run_p50_fraction"] == 1.0
    assert report[0]["tail_queries"]["samples"] == 1
    assert report[0]["tail_queries"]["within_normal_cross_run_p50_fraction"] == 1.0
    assert report[1]["queries"]["within_normal_cross_run_p50_fraction"] == 0.5
    assert report[1]["tail_queries"]["within_normal_cross_run_p95_fraction"] == 1.0


def test_reason_counts_applies_aligned_conflict_mask():
    reasons = np.asarray(["front-clear", "gap-selected", "gap-selected"])

    report = MODULE.reason_counts(reasons, np.asarray([False, True, True]))

    assert report == {"gap-selected": 2}
