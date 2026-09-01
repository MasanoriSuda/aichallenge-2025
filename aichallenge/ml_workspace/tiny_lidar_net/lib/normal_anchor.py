"""Speed-synchronized production-normal sequences for zero residual anchors."""

import json
from pathlib import Path
from typing import Union

import numpy as np
from torch.utils.data import Dataset


NORMAL_ANCHOR_SCHEMA_VERSION = 1
NORMAL_ANCHOR_LABEL_SOURCE = "frozen_production_zero_residual_recurrent"
NORMAL_ANCHOR_REQUIRED_ARRAYS = (
    "scans.npy",
    "speeds.npy",
    "scan_timestamps_ns.npy",
    "speed_timestamps_ns.npy",
    "speed_sync_deltas_sec.npy",
)


class NormalAnchorSequenceDataset(Dataset):
    """One ordered normal run whose only valid target is correction zero."""

    def __init__(
        self,
        sequence_dir: Union[str, Path],
        expected_split: str,
        expected_input_dim: int = 750,
        max_scan_range_m: float = 30.0,
        max_speed_sync_delta_sec: float = 0.05,
    ):
        self.sequence_dir = Path(sequence_dir)
        try:
            self.metadata = json.loads(
                (self.sequence_dir / "metadata.json").read_text(encoding="utf-8")
            )
        except (OSError, json.JSONDecodeError) as exc:
            raise ValueError(
                f"invalid normal-anchor metadata in {self.sequence_dir}: {exc}"
            ) from exc
        self.sequence_id = str(self.metadata.get("sequence_id", ""))
        self.split = self.metadata.get("split")
        if self.metadata.get("schema_version") != NORMAL_ANCHOR_SCHEMA_VERSION:
            raise ValueError("unsupported normal-anchor schema")
        if self.metadata.get("label_source") != NORMAL_ANCHOR_LABEL_SOURCE:
            raise ValueError("unexpected normal-anchor label source")
        if self.sequence_id != self.sequence_dir.name:
            raise ValueError("normal-anchor sequence identity mismatch")
        if self.split != expected_split:
            raise ValueError(
                f"normal-anchor split mismatch: expected={expected_split}, "
                f"actual={self.split}"
            )
        if self.metadata.get("scan_shape") != [expected_input_dim]:
            raise ValueError("normal-anchor scan shape metadata mismatch")
        if self.metadata.get("scan_unit") != "m":
            raise ValueError("normal-anchor scans must use metres")
        if not np.isclose(
            self.metadata.get("max_scan_range_m", np.nan), max_scan_range_m
        ):
            raise ValueError("normal-anchor range contract mismatch")
        recorded_sync = self.metadata.get("max_speed_sync_delta_sec")
        if not isinstance(recorded_sync, (int, float)) or (
            recorded_sync > max_speed_sync_delta_sec + 1e-12
        ):
            raise ValueError("normal-anchor speed sync contract too loose")
        missing = [
            name
            for name in NORMAL_ANCHOR_REQUIRED_ARRAYS
            if not (self.sequence_dir / name).is_file()
        ]
        if missing:
            raise FileNotFoundError(
                f"missing normal-anchor arrays in {self.sequence_dir}: {missing}"
            )
        self.scans = np.load(self.sequence_dir / "scans.npy", allow_pickle=False)
        self.speeds = np.load(self.sequence_dir / "speeds.npy", allow_pickle=False)
        self.scan_timestamps_ns = np.load(
            self.sequence_dir / "scan_timestamps_ns.npy", allow_pickle=False
        )
        self.speed_timestamps_ns = np.load(
            self.sequence_dir / "speed_timestamps_ns.npy", allow_pickle=False
        )
        self.speed_sync_deltas_sec = np.load(
            self.sequence_dir / "speed_sync_deltas_sec.npy", allow_pickle=False
        )
        self._validate_arrays(expected_input_dim, max_scan_range_m)
        self.steers = np.zeros(len(self.scans), dtype=np.float32)
        self.base_steers = np.zeros(len(self.scans), dtype=np.float32)

    def _validate_arrays(self, input_dim: int, max_range_m: float) -> None:
        if self.scans.ndim != 2 or self.scans.shape[1] != input_dim:
            raise ValueError("invalid normal-anchor scan array")
        one_dimensional = (
            self.speeds,
            self.scan_timestamps_ns,
            self.speed_timestamps_ns,
            self.speed_sync_deltas_sec,
        )
        if any(values.ndim != 1 for values in one_dimensional):
            raise ValueError("normal-anchor state arrays must be one-dimensional")
        lengths = {len(self.scans), *(len(values) for values in one_dimensional)}
        if lengths != {len(self.scans)} or len(self.scans) == 0:
            raise ValueError("normal-anchor array length mismatch")
        if not np.all(np.isfinite(self.scans)) or not np.all(
            np.isfinite(self.speeds)
        ):
            raise ValueError("normal-anchor states must be finite")
        if np.any(self.scans < 0.0) or np.any(self.scans > max_range_m):
            raise ValueError("normal-anchor scans outside physical range")
        if np.any(self.speeds < 0.0):
            raise ValueError("normal-anchor speed must be non-negative")
        if np.any(np.diff(self.scan_timestamps_ns) <= 0):
            raise ValueError("normal-anchor scan timestamps not increasing")
        if np.any(self.speed_sync_deltas_sec < 0.0) or np.any(
            self.speed_sync_deltas_sec
            > float(self.metadata["max_speed_sync_delta_sec"]) + 1e-12
        ):
            raise ValueError("normal-anchor synchronization violation")

    def __len__(self) -> int:
        return len(self.scans)

    def __getitem__(self, index: int):
        zero = np.float32(0.0)
        return self.scans[index], np.float32(self.speeds[index]), zero, zero


class MultiSeqNormalAnchorDataset:
    def __init__(self, root: Union[str, Path], expected_split: str):
        path = Path(root)
        sequence_dirs = (
            sorted(item for item in path.iterdir() if item.is_dir())
            if path.is_dir()
            else []
        )
        if not sequence_dirs:
            raise RuntimeError(f"no normal-anchor sequences found in {path}")
        self.datasets = [
            NormalAnchorSequenceDataset(item, expected_split)
            for item in sequence_dirs
        ]
        self.sequence_ids = [item.sequence_id for item in self.datasets]
        if len(self.sequence_ids) != len(set(self.sequence_ids)):
            raise ValueError("duplicate normal-anchor sequence identities")
