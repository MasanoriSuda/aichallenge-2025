import threading
import time

import numpy as np

from tiny_lidar_net_controller.recurrent_shadow_executor import (
    LatestWinsRecurrentShadowExecutor,
)
from tiny_lidar_net_controller.tiny_lidar_net_controller_core import (
    RecurrentShadowEvaluation,
    RecurrentShadowSample,
)


def _sample(value: float) -> RecurrentShadowSample:
    features = np.asarray([[value]], dtype=np.float32)
    features.setflags(write=False)
    return RecurrentShadowSample(
        conv5_features=features,
        speed_mps=3.0,
        raw_base_steering_rad=0.0,
        spatial_correction_rad=0.0,
        production_spatial_steering_rad=0.0,
    )


def _evaluation(value: float) -> RecurrentShadowEvaluation:
    hidden = np.asarray([[value]], dtype=np.float32)
    return RecurrentShadowEvaluation(
        correction_rad=value,
        raw_correction_rad=value,
        steering_rad=value,
        hidden_norm=abs(value),
        next_hidden=hidden,
    )


def test_latest_wins_replaces_only_pending_work() -> None:
    release = threading.Event()
    started = threading.Event()

    def evaluate(sample, hidden):
        value = float(sample.conv5_features[0, 0])
        if value == 1.0:
            started.set()
            assert release.wait(timeout=2.0)
        return _evaluation(value)

    executor = LatestWinsRecurrentShadowExecutor(
        evaluate, max_result_age_sec=2.0
    )
    try:
        assert executor.submit(1, _sample(1.0)) == 0
        assert started.wait(timeout=1.0)
        assert executor.submit(2, _sample(2.0)) == 0
        assert executor.submit(3, _sample(3.0)) == 1
        release.set()

        deadline = time.monotonic() + 2.0
        completions = []
        while len(completions) < 2 and time.monotonic() < deadline:
            completions.extend(executor.drain_completed())
            time.sleep(0.01)

        assert [result.sequence for result in completions] == [1, 3]
        assert all(result.status == "ok" for result in completions)
        assert executor.stats() == {
            "submitted": 3,
            "completed": 2,
            "dropped": 1,
            "stale": 0,
            "errors": 0,
            "resets": 0,
        }
    finally:
        release.set()
        executor.close()


def test_reset_supersedes_running_result_without_reusing_hidden() -> None:
    release = threading.Event()
    started = threading.Event()
    observed_hidden = []

    def evaluate(sample, hidden):
        observed_hidden.append(hidden)
        started.set()
        assert release.wait(timeout=2.0)
        return _evaluation(1.0)

    executor = LatestWinsRecurrentShadowExecutor(
        evaluate, max_result_age_sec=2.0
    )
    try:
        executor.submit(1, _sample(1.0))
        assert started.wait(timeout=1.0)
        executor.reset_history()
        release.set()

        deadline = time.monotonic() + 2.0
        completions = []
        while not completions and time.monotonic() < deadline:
            completions.extend(executor.drain_completed())
            time.sleep(0.01)

        assert observed_hidden == [None]
        assert completions[0].status == "reset-superseded"
        assert completions[0].evaluation is None
        assert executor.stats()["stale"] == 1
    finally:
        release.set()
        executor.close()
