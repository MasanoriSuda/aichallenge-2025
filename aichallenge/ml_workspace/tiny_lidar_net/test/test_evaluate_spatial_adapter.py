from pathlib import Path
from types import SimpleNamespace
import importlib.util

import numpy as np
import pytest


MODULE_PATH = Path(__file__).resolve().parents[1] / "evaluate_spatial_adapter.py"
SPEC = importlib.util.spec_from_file_location("evaluate_spatial_adapter", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class Sequence:
    def __init__(self, source_bag: str, samples: int = 5):
        self.sequence_id = source_bag.replace("/", "-")
        self.metadata = {"source": {"bag": source_bag}}
        self.samples = list(range(samples))

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, index):
        return self.samples[index]


def test_select_unique_source_sequence_requires_one_immutable_run():
    sequences = [Sequence("/output/run-a/d1"), Sequence("/output/run-b/d1")]

    assert (
        MODULE.select_unique_source_sequence(sequences, "run-b").metadata["source"][
            "bag"
        ]
        == "/output/run-b/d1"
    )
    assert MODULE.select_unique_source_sequence(sequences, "") is None
    with pytest.raises(ValueError, match="matched 0 sequences"):
        MODULE.select_unique_source_sequence(sequences, "missing")
    with pytest.raises(ValueError, match="matched 2 sequences"):
        MODULE.select_unique_source_sequence(sequences, "output")


def test_causal_tail_never_crosses_sequence_boundary():
    sequence = Sequence("/output/run-a/d1", samples=5)

    assert list(MODULE.causal_tail(sequence, 3)) == [2, 3, 4]
    assert list(MODULE.causal_tail(sequence, 20)) == [0, 1, 2, 3, 4]
    with pytest.raises(ValueError, match="positive"):
        MODULE.causal_tail(sequence, 0)


def test_runtime_bounded_metrics_use_exact_authority_clip():
    report = MODULE.runtime_bounded_metrics(
        np.asarray([-0.4, 0.01, 0.5], dtype=np.float32),
        np.asarray([-0.2, 0.0, 0.3], dtype=np.float32),
        material_delta_rad=0.02,
        authority_bound_rad=0.12,
    )

    assert report["authority_bound_rad"] == pytest.approx(0.12)
    assert report["clipped_fraction"] == pytest.approx(2.0 / 3.0)
    assert report["material_sign_accuracy"] == pytest.approx(1.0)
    assert report["residual"]["all"]["mae_rad"] == pytest.approx(
        (0.08 + 0.01 + 0.18) / 3.0
    )
    assert report["oracle"]["material_mae_rad"] == pytest.approx(0.13)
    assert report["oracle"][
        "maximum_material_mae_improvement_fraction"
    ] == pytest.approx(0.48)
    assert report["oracle"]["attainable_improvement_utilization"] == pytest.approx(
        1.0
    )


def test_runtime_bounded_metrics_reject_invalid_contract():
    with pytest.raises(ValueError, match="positive"):
        MODULE.runtime_bounded_metrics(
            np.asarray([0.1]), np.asarray([0.2]), 0.02, 0.0
        )
    with pytest.raises(ValueError, match="aligned"):
        MODULE.runtime_bounded_metrics(
            np.asarray([0.1]), np.asarray([0.2, 0.3]), 0.02, 0.12
        )
