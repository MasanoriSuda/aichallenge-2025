"""Standalone one-thread NumPy evaluator for recurrent shadow inference."""

import os

# This module is launched in a fresh interpreter.  Set the budget before any
# package importing NumPy is loaded.
os.environ["OPENBLAS_NUM_THREADS"] = "1"

import sys

import numpy as np

from tiny_lidar_net_controller.recurrent_runtime import load_recurrent_runtime
from tiny_lidar_net_controller.recurrent_shadow_protocol import (
    receive_message,
    send_message,
)


def _evaluate(model, request: dict) -> dict:
    sample = request["sample"]
    conv5 = np.asarray(sample["conv5_features"], dtype=np.float32)
    speed = float(sample["speed_mps"])
    production_steering = float(sample["production_spatial_steering_rad"])
    hidden_value = request.get("hidden")
    hidden = (
        None
        if hidden_value is None
        else np.asarray(hidden_value, dtype=np.float32)
    )
    correction, raw_correction, next_hidden = (
        model.forward_correction_from_conv5_features(
            conv5,
            np.asarray([speed], dtype=np.float32),
            hidden,
        )
    )
    steering = np.clip(production_steering + correction, -1.0, 1.0)
    next_hidden = np.array(next_hidden, dtype=np.float32, copy=True)
    return {
        "correction_rad": float(correction[0]),
        "raw_correction_rad": float(raw_correction[0]),
        "steering_rad": float(steering[0]),
        "hidden_norm": float(np.linalg.norm(next_hidden)),
        "next_hidden": next_hidden,
    }


def main() -> int:
    reader = sys.stdin.buffer
    writer = sys.stdout.buffer
    try:
        initialize = receive_message(reader)
        if initialize.get("op") != "initialize":
            raise ValueError("first recurrent worker message must initialize")
        loaded = load_recurrent_runtime(
            str(initialize["checkpoint_path"]),
            str(initialize["expected_sha256"]),
            legacy_runtime_config=initialize.get("legacy_runtime_config"),
        )
        expected_config = initialize.get("expected_runtime_config")
        if expected_config is not None and loaded.runtime_config != expected_config:
            raise ValueError(
                "recurrent worker runtime config mismatch: "
                f"expected={expected_config}, actual={loaded.runtime_config}"
            )
        send_message(writer, {
            "status": "ready",
            "pid": os.getpid(),
            "openblas_threads": os.environ["OPENBLAS_NUM_THREADS"],
            "sha256": loaded.sha256,
            "artifact_contract": loaded.artifact_contract,
            "runtime_config": loaded.runtime_config,
            "loaded_parameter_count": loaded.loaded_parameter_count,
        })
    except Exception as exc:
        send_message(writer, {
            "status": f"initialization-error:{type(exc).__name__}:{exc}"
        })
        return 2

    while True:
        try:
            request = receive_message(reader)
        except EOFError:
            return 0
        if request.get("op") == "close":
            return 0
        if request.get("op") != "evaluate":
            send_message(writer, {
                "status": "protocol-error:unsupported-operation"
            })
            continue
        try:
            evaluation = _evaluate(loaded.model, request)
            send_message(writer, {
                "status": "ok",
                "sequence": int(request["sequence"]),
                "evaluation": evaluation,
            })
        except Exception as exc:
            send_message(writer, {
                "status": f"inference-error:{type(exc).__name__}:{exc}",
                "sequence": int(request.get("sequence", -1)),
                "evaluation": None,
            })


if __name__ == "__main__":
    raise SystemExit(main())
