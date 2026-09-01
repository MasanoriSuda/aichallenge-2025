import importlib.util
from pathlib import Path
import sys

import numpy as np
import pytest
import torch


MODULE_PATH = (
    Path(__file__).resolve().parents[1] / "probe_e2e_causal_geometry.py"
)
SPEC = importlib.util.spec_from_file_location(
    "probe_e2e_causal_geometry", MODULE_PATH
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def make_sequence(identity: str, offset: float, length: int = 10):
    features = np.full((length, 752), offset, dtype=np.float32)
    labels = np.arange(length, dtype=np.int64) % 3
    return MODULE.TemporalProbeSequence(
        sequence_id=identity,
        source_bag=f"/{identity}",
        features=features,
        labels=labels,
        is_normal=False,
    )


def test_temporal_feature_contract_normalizes_physical_inputs_once():
    scans = np.vstack(
        (np.full(750, 3.0, dtype=np.float32), np.full(750, 15.0, dtype=np.float32))
    )
    features = MODULE.compose_temporal_features(
        scans,
        np.asarray([6.0, 12.0], dtype=np.float32),
        np.asarray([-0.2, 0.3], dtype=np.float32),
    )

    assert features.shape == (2, 752)
    assert np.allclose(features[0, :750], 0.1)
    assert np.allclose(features[1, :750], 0.5)
    assert np.allclose(features[:, 750:], [[0.5, -0.2], [1.0, 0.3]])


def test_temporal_chunks_never_cross_sequence_boundaries():
    first = make_sequence("first", 1.0)
    second = make_sequence("second", 2.0)
    chunks = MODULE.TemporalProbeChunkDataset(
        [first, second], chunk_length=4, stride=3
    )

    assert chunks.records == [
        (0, 0),
        (0, 3),
        (0, 6),
        (1, 0),
        (1, 3),
        (1, 6),
    ]
    for index in range(len(chunks)):
        features, labels = chunks[index]
        assert features.shape == (4, 752)
        assert labels.shape == (4,)
        assert np.unique(features[:, 0]).size == 1


def test_causal_probe_emits_logits_and_reusable_hidden_state():
    model = MODULE.CausalGeometryActionProbe()
    first, hidden = model(torch.zeros(2, 5, 752))
    second, next_hidden = model(torch.zeros(2, 3, 752), hidden)

    assert first.shape == (2, 5, 3)
    assert second.shape == (2, 3, 3)
    assert hidden.shape == next_hidden.shape == (1, 2, 64)
    assert torch.isfinite(first).all()
    assert torch.isfinite(second).all()


def test_temporal_chunk_rejects_sequence_shorter_than_context():
    with pytest.raises(ValueError, match="too short"):
        MODULE.TemporalProbeChunkDataset(
            [make_sequence("short", 1.0, length=3)],
            chunk_length=4,
            stride=2,
        )
