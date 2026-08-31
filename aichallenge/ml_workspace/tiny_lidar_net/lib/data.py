"""Auditable TinyLidarNet dataset loading and split-contract validation."""

import json
import logging
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple, Union

import numpy as np
from torch.utils.data import ConcatDataset, Dataset


logger = logging.getLogger(__name__)

DATASET_SCHEMA_VERSION = 1
DEFAULT_ALLOWED_LABEL_SOURCES = (
    "mpc",
    "mpcc",
    "human",
    "lidar_gap_teacher",
    "lidar_gap_teacher_dagger",
)
REQUIRED_ARRAY_FILES = (
    "scans.npy",
    "steers.npy",
    "accelerations.npy",
    "delta_times.npy",
    "scan_timestamps_ns.npy",
    "control_timestamps_ns.npy",
)


def _load_json_object(path: Path) -> dict:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        raise FileNotFoundError(f"Missing required dataset metadata: {path}") from None
    except json.JSONDecodeError as exc:
        raise ValueError(f"Invalid JSON in {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ValueError(f"Dataset metadata must be a JSON object: {path}")
    return value


def assert_disjoint_sequence_ids(
    train_sequence_ids: Iterable[str], val_sequence_ids: Iterable[str]
) -> None:
    """Reject run leakage across train and validation before training starts."""
    overlap = sorted(set(train_sequence_ids) & set(val_sequence_ids))
    if overlap:
        raise ValueError(
            "Train/validation sequence leakage detected: " + ", ".join(overlap)
        )


class ScanControlSequenceDataset(Dataset):
    """One synchronized and provenance-checked LiDAR/control sequence."""

    def __init__(
        self,
        seq_dir: Union[str, Path],
        max_range: float = 30.0,
        expected_input_dim: int = 750,
        allowed_label_sources: Sequence[str] = DEFAULT_ALLOWED_LABEL_SOURCES,
        require_metadata: bool = True,
        max_sync_delta_sec: float = 0.05,
        expected_split: Optional[str] = None,
    ):
        self.seq_dir = Path(seq_dir)
        self.max_range = float(max_range)
        self.expected_input_dim = int(expected_input_dim)

        if not np.isfinite(self.max_range) or self.max_range <= 0.0:
            raise ValueError(f"max_range must be finite and positive: {max_range}")
        if self.expected_input_dim <= 0:
            raise ValueError(
                f"expected_input_dim must be positive: {expected_input_dim}"
            )
        if not np.isfinite(max_sync_delta_sec) or max_sync_delta_sec < 0.0:
            raise ValueError(
                "max_sync_delta_sec must be finite and non-negative: "
                f"{max_sync_delta_sec}"
            )

        metadata_path = self.seq_dir / "metadata.json"
        self.metadata = _load_json_object(metadata_path) if require_metadata else {}
        self.sequence_id = str(self.metadata.get("sequence_id", self.seq_dir.name))
        self.split = self.metadata.get("split")
        self.label_source = self.metadata.get("label_source")

        if require_metadata:
            self._validate_metadata(
                allowed_label_sources=allowed_label_sources,
                expected_split=expected_split,
                max_sync_delta_sec=max_sync_delta_sec,
            )

        missing = [
            name for name in REQUIRED_ARRAY_FILES if not (self.seq_dir / name).is_file()
        ]
        if missing:
            raise FileNotFoundError(
                f"Missing required dataset arrays in {self.seq_dir}: {missing}"
            )

        self.scans = np.load(self.seq_dir / "scans.npy", allow_pickle=False)
        self.steers = np.load(self.seq_dir / "steers.npy", allow_pickle=False)
        self.accels = np.load(
            self.seq_dir / "accelerations.npy", allow_pickle=False
        )
        self.delta_times = np.load(
            self.seq_dir / "delta_times.npy", allow_pickle=False
        )
        self.scan_timestamps_ns = np.load(
            self.seq_dir / "scan_timestamps_ns.npy", allow_pickle=False
        )
        self.control_timestamps_ns = np.load(
            self.seq_dir / "control_timestamps_ns.npy", allow_pickle=False
        )

        self._validate_arrays(max_sync_delta_sec=max_sync_delta_sec)
        self.scans = (self.scans / self.max_range).astype(np.float32, copy=False)

    def _validate_metadata(
        self,
        allowed_label_sources: Sequence[str],
        expected_split: Optional[str],
        max_sync_delta_sec: float,
    ) -> None:
        schema_version = self.metadata.get("schema_version")
        if schema_version != DATASET_SCHEMA_VERSION:
            raise ValueError(
                f"Unsupported dataset schema in {self.seq_dir}: "
                f"expected={DATASET_SCHEMA_VERSION}, actual={schema_version}"
            )
        if self.sequence_id != self.seq_dir.name:
            raise ValueError(
                f"Sequence identity mismatch in {self.seq_dir}: "
                f"metadata={self.sequence_id}, directory={self.seq_dir.name}"
            )
        if self.split not in {"train", "val"}:
            raise ValueError(f"Invalid dataset split in {self.seq_dir}: {self.split}")
        if expected_split is not None and self.split != expected_split:
            raise ValueError(
                f"Dataset split mismatch in {self.seq_dir}: "
                f"expected={expected_split}, actual={self.split}"
            )

        allowed = set(allowed_label_sources)
        if not allowed:
            raise ValueError("allowed_label_sources must not be empty")
        if self.label_source not in allowed:
            raise ValueError(
                f"Disallowed label source in {self.seq_dir}: "
                f"actual={self.label_source}, allowed={sorted(allowed)}"
            )

        scan_shape = self.metadata.get("scan_shape")
        if scan_shape != [self.expected_input_dim]:
            raise ValueError(
                f"Metadata scan shape mismatch in {self.seq_dir}: "
                f"expected={[self.expected_input_dim]}, actual={scan_shape}"
            )
        recorded_max_range = self.metadata.get("max_scan_range_m")
        if not isinstance(recorded_max_range, (int, float)) or not np.isclose(
            recorded_max_range, self.max_range
        ):
            raise ValueError(
                f"LiDAR range contract mismatch in {self.seq_dir}: "
                f"expected={self.max_range}, actual={recorded_max_range}"
            )
        recorded_sync_limit = self.metadata.get("max_sync_delta_sec")
        if not isinstance(recorded_sync_limit, (int, float)) or (
            recorded_sync_limit > max_sync_delta_sec + 1e-12
        ):
            raise ValueError(
                f"Extraction sync contract is too loose in {self.seq_dir}: "
                f"trainer_limit={max_sync_delta_sec}, "
                f"extractor_limit={recorded_sync_limit}"
            )

    def _validate_arrays(self, max_sync_delta_sec: float) -> None:
        if self.scans.ndim != 2 or self.scans.shape[1] != self.expected_input_dim:
            raise ValueError(
                f"Scan shape mismatch in {self.seq_dir}: "
                f"expected=(N, {self.expected_input_dim}), actual={self.scans.shape}"
            )
        one_dimensional = {
            "steers": self.steers,
            "accelerations": self.accels,
            "delta_times": self.delta_times,
            "scan_timestamps_ns": self.scan_timestamps_ns,
            "control_timestamps_ns": self.control_timestamps_ns,
        }
        for name, values in one_dimensional.items():
            if values.ndim != 1:
                raise ValueError(
                    f"{name} must be one-dimensional in {self.seq_dir}: "
                    f"actual={values.shape}"
                )

        n_samples = len(self.scans)
        if n_samples == 0:
            raise ValueError(f"Empty sequence: {self.seq_dir}")
        lengths = {"scans": n_samples}
        lengths.update({name: len(values) for name, values in one_dimensional.items()})
        if len(set(lengths.values())) != 1:
            raise ValueError(f"Dataset length mismatch in {self.seq_dir}: {lengths}")

        for name, values in (
            ("scans", self.scans),
            ("steers", self.steers),
            ("accelerations", self.accels),
            ("delta_times", self.delta_times),
        ):
            if not np.all(np.isfinite(values)):
                raise ValueError(f"Non-finite {name} values in {self.seq_dir}")
        if np.any(self.scans < 0.0) or np.any(self.scans > self.max_range):
            raise ValueError(
                f"LiDAR values outside [0, {self.max_range}] in {self.seq_dir}"
            )
        if np.any(self.delta_times < 0.0) or np.any(
            self.delta_times > max_sync_delta_sec + 1e-12
        ):
            raise ValueError(
                f"Synchronization delta exceeds {max_sync_delta_sec}s in {self.seq_dir}: "
                f"max={float(np.max(self.delta_times))}"
            )
        if not np.issubdtype(
            self.scan_timestamps_ns.dtype, np.integer
        ) or not np.issubdtype(self.control_timestamps_ns.dtype, np.integer):
            raise ValueError(f"Timestamps must use integer nanoseconds in {self.seq_dir}")
        if np.any(np.diff(self.scan_timestamps_ns) < 0):
            raise ValueError(f"Scan timestamps are not monotonic in {self.seq_dir}")

        counts = self.metadata.get("counts", {})
        if counts and counts.get("accepted_samples") != n_samples:
            raise ValueError(
                f"Metadata sample count mismatch in {self.seq_dir}: "
                f"metadata={counts.get('accepted_samples')}, arrays={n_samples}"
            )

    def __len__(self) -> int:
        return len(self.scans)

    def __getitem__(self, idx: int) -> Tuple[np.ndarray, np.ndarray]:
        scan = self.scans[idx].astype(np.float32, copy=False)
        target = np.array(
            [np.float32(self.accels[idx]), np.float32(self.steers[idx])],
            dtype=np.float32,
        )
        return scan, target


class MultiSeqConcatDataset(ConcatDataset):
    """Concatenate sequences only after every child satisfies one contract."""

    def __init__(
        self,
        dataset_root: Union[str, Path],
        max_range: float = 30.0,
        include: Optional[List[str]] = None,
        exclude: Optional[List[str]] = None,
        expected_input_dim: int = 750,
        allowed_label_sources: Sequence[str] = DEFAULT_ALLOWED_LABEL_SOURCES,
        require_metadata: bool = True,
        max_sync_delta_sec: float = 0.05,
        expected_split: Optional[str] = None,
    ):
        dataset_root = Path(dataset_root)
        if not dataset_root.is_dir():
            raise FileNotFoundError(f"Dataset root does not exist: {dataset_root}")

        target_seq_dirs = []
        for path in sorted(item for item in dataset_root.iterdir() if item.is_dir()):
            if include and not any(token in path.name for token in include):
                continue
            if exclude and any(token in path.name for token in exclude):
                continue
            target_seq_dirs.append(path)

        if not target_seq_dirs:
            raise RuntimeError(
                f"No sequence directories found in {dataset_root} with provided filters."
            )

        datasets = [
            ScanControlSequenceDataset(
                seq_dir,
                max_range=max_range,
                expected_input_dim=expected_input_dim,
                allowed_label_sources=allowed_label_sources,
                require_metadata=require_metadata,
                max_sync_delta_sec=max_sync_delta_sec,
                expected_split=expected_split,
            )
            for seq_dir in target_seq_dirs
        ]
        self.sequence_ids = [dataset.sequence_id for dataset in datasets]
        if len(self.sequence_ids) != len(set(self.sequence_ids)):
            raise ValueError(
                f"Duplicate sequence identities within {dataset_root}: "
                f"{self.sequence_ids}"
            )

        super().__init__(datasets)
        logger.info(
            "Loaded %d validated sequences from %s. Total samples: %d",
            len(datasets),
            dataset_root,
            len(self),
        )
