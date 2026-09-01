from types import SimpleNamespace

import numpy as np
import pytest

from train_recurrent_policy import outcome_run_balanced_chunk_weights


class ChunkGroup:
    def __init__(self, run_id: str | None, count: int):
        certificate = {} if run_id is None else {"source_run_id": run_id}
        self.sequence = SimpleNamespace(
            metadata={"outcome_certificate": certificate},
        )
        self.count = count

    def __len__(self) -> int:
        return self.count


def test_outcome_run_balancing_groups_correlated_sequences() -> None:
    chunks = SimpleNamespace(
        datasets=[
            ChunkGroup("single", 2),
            ChunkGroup("peer", 3),
            ChunkGroup("peer", 5),
        ]
    )
    weights, counts = outcome_run_balanced_chunk_weights(chunks)
    values = weights.numpy()

    assert counts == {"peer": 8, "single": 2}
    assert np.sum(values[:2]) == pytest.approx(0.5)
    assert np.sum(values[2:]) == pytest.approx(0.5)
    assert np.sum(values[2:5]) == pytest.approx(3.0 / 16.0)
    assert np.sum(values[5:]) == pytest.approx(5.0 / 16.0)


@pytest.mark.parametrize("run_id", [None, "", "  "])
def test_outcome_run_balancing_rejects_missing_identity(run_id) -> None:
    chunks = SimpleNamespace(datasets=[ChunkGroup(run_id, 2)])
    with pytest.raises(ValueError, match="certified source_run_id"):
        outcome_run_balanced_chunk_weights(chunks)
