"""Bounded observation-only execution for recurrent steering experiments."""

from collections import deque
from dataclasses import dataclass
import threading
import time
from typing import Callable, Deque, Optional

import numpy as np

from tiny_lidar_net_controller.tiny_lidar_net_controller_core import (
    RecurrentShadowEvaluation,
    RecurrentShadowSample,
)


@dataclass(frozen=True)
class RecurrentShadowCompletion:
    sequence: int
    status: str
    evaluation: Optional[RecurrentShadowEvaluation]
    inference_ms: float
    age_ms: float


@dataclass(frozen=True)
class _WorkItem:
    sequence: int
    generation: int
    submitted_monotonic: float
    sample: RecurrentShadowSample


class LatestWinsRecurrentShadowExecutor:
    """Run at most one recurrent item with at most one pending successor.

    This executor is diagnostic-only.  It owns recurrent hidden state and
    exposes completed results to the ROS thread, but it cannot publish or
    modify a production command.
    """

    def __init__(
        self,
        evaluator: Callable[
            [RecurrentShadowSample, Optional[np.ndarray]],
            RecurrentShadowEvaluation,
        ],
        *,
        max_result_age_sec: float,
    ) -> None:
        if not np.isfinite(max_result_age_sec) or max_result_age_sec <= 0.0:
            raise ValueError("max_result_age_sec must be finite and positive")
        self._evaluator = evaluator
        self._max_result_age_sec = float(max_result_age_sec)
        self._condition = threading.Condition()
        self._pending: Optional[_WorkItem] = None
        self._completed: Deque[RecurrentShadowCompletion] = deque()
        self._hidden: Optional[np.ndarray] = None
        self._generation = 0
        self._closed = False
        self._submitted_count = 0
        self._completed_count = 0
        self._dropped_count = 0
        self._stale_count = 0
        self._error_count = 0
        self._reset_count = 0
        self._thread = threading.Thread(
            target=self._run,
            name="recurrent-shadow-worker",
            daemon=True,
        )
        self._thread.start()

    def submit(self, sequence: int, sample: RecurrentShadowSample) -> int:
        """Submit newest work and return the number of replaced pending items."""
        with self._condition:
            if self._closed:
                raise RuntimeError("recurrent shadow executor is closed")
            dropped = int(self._pending is not None)
            if dropped:
                self._dropped_count += 1
            self._pending = _WorkItem(
                sequence=int(sequence),
                generation=self._generation,
                submitted_monotonic=time.monotonic(),
                sample=sample,
            )
            self._submitted_count += 1
            self._condition.notify()
            return dropped

    def drain_completed(self) -> list[RecurrentShadowCompletion]:
        with self._condition:
            results = list(self._completed)
            self._completed.clear()
            return results

    def reset_history(self) -> None:
        """Invalidate temporal state at an explicit controller reset boundary."""
        with self._condition:
            self._generation += 1
            if self._hidden is not None:
                self._reset_count += 1
            self._hidden = None
            if self._pending is not None:
                self._pending = None
                self._dropped_count += 1

    def stats(self) -> dict[str, int]:
        with self._condition:
            return {
                "submitted": self._submitted_count,
                "completed": self._completed_count,
                "dropped": self._dropped_count,
                "stale": self._stale_count,
                "errors": self._error_count,
                "resets": self._reset_count,
            }

    def close(self, timeout_sec: float = 1.0) -> None:
        with self._condition:
            if self._closed:
                return
            self._closed = True
            if self._pending is not None:
                self._pending = None
                self._dropped_count += 1
            self._condition.notify_all()
        self._thread.join(timeout=max(0.0, float(timeout_sec)))

    def _run(self) -> None:
        while True:
            with self._condition:
                while self._pending is None and not self._closed:
                    self._condition.wait()
                if self._pending is None and self._closed:
                    return
                item = self._pending
                self._pending = None
                hidden = (
                    None if self._hidden is None else self._hidden.copy()
                )

            inference_start = time.monotonic()
            evaluation = None
            status = "ok"
            try:
                evaluation = self._evaluator(item.sample, hidden)
            except Exception as exc:  # diagnostic failure must stay local
                status = f"inference-error:{type(exc).__name__}:{exc}"
            completed_monotonic = time.monotonic()
            inference_ms = (completed_monotonic - inference_start) * 1000.0
            age_ms = (
                completed_monotonic - item.submitted_monotonic
            ) * 1000.0

            with self._condition:
                if item.generation != self._generation:
                    status = "reset-superseded"
                    evaluation = None
                    self._stale_count += 1
                elif status.startswith("inference-error:"):
                    if self._hidden is not None:
                        self._reset_count += 1
                    self._hidden = None
                    self._error_count += 1
                elif age_ms > self._max_result_age_sec * 1000.0:
                    status = "stale-result"
                    evaluation = None
                    self._stale_count += 1
                else:
                    self._hidden = evaluation.next_hidden.copy()
                self._completed_count += 1
                self._completed.append(RecurrentShadowCompletion(
                    sequence=item.sequence,
                    status=status,
                    evaluation=evaluation,
                    inference_ms=inference_ms,
                    age_ms=age_ms,
                ))
