"""Subprocess client for resource-isolated recurrent shadow inference."""

from dataclasses import dataclass
import os
import select
import subprocess
import sys
from typing import Optional

import numpy as np

from tiny_lidar_net_controller.recurrent_shadow_protocol import (
    receive_message,
    send_message,
)
from tiny_lidar_net_controller.tiny_lidar_net_controller_core import (
    RecurrentShadowEvaluation,
    RecurrentShadowSample,
)


@dataclass(frozen=True)
class RecurrentWorkerIdentity:
    pid: int
    openblas_threads: str
    sha256: str
    artifact_contract: str
    loaded_parameter_count: int


class RecurrentShadowSubprocessEvaluator:
    """Evaluate immutable recurrent samples in a fresh one-thread process."""

    def __init__(
        self,
        *,
        checkpoint_path: str,
        expected_sha256: str,
        expected_runtime_config: dict,
        response_timeout_sec: float,
    ) -> None:
        if (
            not np.isfinite(response_timeout_sec)
            or response_timeout_sec <= 0.0
        ):
            raise ValueError("response_timeout_sec must be finite and positive")
        self._response_timeout_sec = float(response_timeout_sec)
        self._closed = False
        self._sequence = 0
        environment = os.environ.copy()
        environment["OPENBLAS_NUM_THREADS"] = "1"
        self._process = subprocess.Popen(
            [
                sys.executable,
                "-m",
                "tiny_lidar_net_controller.recurrent_shadow_worker",
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=None,
            bufsize=0,
            env=environment,
        )
        if self._process.stdin is None or self._process.stdout is None:
            self.close()
            raise RuntimeError("failed to create recurrent worker pipes")
        self._writer = self._process.stdin
        self._reader = self._process.stdout
        try:
            send_message(self._writer, {
                "op": "initialize",
                "checkpoint_path": str(checkpoint_path),
                "expected_sha256": str(expected_sha256),
                "expected_runtime_config": dict(expected_runtime_config),
            })
            reply = self._receive_with_timeout(
                max(5.0, self._response_timeout_sec)
            )
            status = str(reply.get("status", "missing-status"))
            if status != "ready":
                raise RuntimeError(status)
            self.identity = RecurrentWorkerIdentity(
                pid=int(reply["pid"]),
                openblas_threads=str(reply["openblas_threads"]),
                sha256=str(reply["sha256"]),
                artifact_contract=str(reply["artifact_contract"]),
                loaded_parameter_count=int(reply["loaded_parameter_count"]),
            )
        except Exception:
            self.close()
            raise

    def _receive_with_timeout(self, timeout_sec: Optional[float] = None):
        ready, _, _ = select.select(
            [self._reader],
            [],
            [],
            self._response_timeout_sec
            if timeout_sec is None
            else float(timeout_sec),
        )
        if not ready:
            raise TimeoutError("recurrent worker response timed out")
        return receive_message(self._reader)

    def __call__(
        self,
        sample: RecurrentShadowSample,
        hidden: Optional[np.ndarray],
    ) -> RecurrentShadowEvaluation:
        if self._closed:
            raise RuntimeError("recurrent worker is closed")
        self._sequence += 1
        sequence = self._sequence
        try:
            send_message(self._writer, {
                "op": "evaluate",
                "sequence": sequence,
                "sample": {
                    "conv5_features": sample.conv5_features,
                    "speed_mps": sample.speed_mps,
                    "production_spatial_steering_rad": (
                        sample.production_spatial_steering_rad
                    ),
                },
                "hidden": hidden,
            })
            reply = self._receive_with_timeout()
            status = str(reply.get("status", "missing-status"))
            if status != "ok":
                raise RuntimeError(status)
            if int(reply.get("sequence", -1)) != sequence:
                raise RuntimeError(
                    "recurrent worker returned a mismatched sequence"
                )
        except Exception:
            # A timed-out request may still produce a late pipe reply.  Poison
            # this private channel instead of allowing the next sample to
            # consume a result with the wrong temporal identity.
            self.close()
            raise
        value = reply["evaluation"]
        return RecurrentShadowEvaluation(
            correction_rad=float(value["correction_rad"]),
            raw_correction_rad=float(value["raw_correction_rad"]),
            steering_rad=float(value["steering_rad"]),
            hidden_norm=float(value["hidden_norm"]),
            next_hidden=np.asarray(value["next_hidden"], dtype=np.float32),
        )

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        process = getattr(self, "_process", None)
        writer = getattr(self, "_writer", None)
        if process is None:
            return
        if process.poll() is None and writer is not None:
            try:
                send_message(writer, {"op": "close"})
            except (BrokenPipeError, OSError):
                pass
        try:
            process.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=1.0)
        for stream_name in ("_writer", "_reader"):
            stream = getattr(self, stream_name, None)
            if stream is not None:
                stream.close()
