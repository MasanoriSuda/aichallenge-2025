import numpy as np
import pytest

from lib.speed_sync import synchronize_latest_preceding


def test_causal_speed_sync_never_selects_a_future_sample():
    queries = np.array([5, 10, 15, 25], dtype=np.int64)
    samples = np.array([10, 20], dtype=np.int64)

    indices, ages, valid = synchronize_latest_preceding(queries, samples)

    assert indices.tolist() == [-1, 0, 0, 1]
    assert valid.tolist() == [False, True, True, True]
    assert ages[1:].tolist() == [0, 5, 5]
    assert ages[0] == np.iinfo(np.int64).max


def test_causal_speed_sync_rejects_non_monotonic_sample_time():
    with pytest.raises(ValueError, match="strictly increasing"):
        synchronize_latest_preceding(
            np.array([10, 20]), np.array([10, 10])
        )
