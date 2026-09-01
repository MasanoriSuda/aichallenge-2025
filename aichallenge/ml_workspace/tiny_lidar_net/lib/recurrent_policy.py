"""Sequence contracts and model for an offline recurrent direct policy."""

import json
from pathlib import Path
from typing import Optional, Sequence, Union

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import ConcatDataset, Dataset


RECURRENT_DATASET_SCHEMA_VERSION = 1
RECURRENT_REQUIRED_ARRAYS = (
    "scans.npy",
    "speeds.npy",
    "steers.npy",
    "base_steers.npy",
    "scan_timestamps_ns.npy",
    "speed_timestamps_ns.npy",
    "speed_sync_deltas_sec.npy",
)


def _read_metadata(path: Path) -> dict:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        raise FileNotFoundError(f"missing recurrent metadata: {path}") from None
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid recurrent metadata {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ValueError(f"recurrent metadata must be an object: {path}")
    return value


class RecurrentPolicySequenceDataset(Dataset):
    """One ordered run with LiDAR, ego speed and direct steering labels."""

    def __init__(
        self,
        seq_dir: Union[str, Path],
        expected_split: Optional[str] = None,
        expected_input_dim: int = 750,
        max_scan_range_m: float = 30.0,
        max_speed_sync_delta_sec: float = 0.05,
    ):
        self.seq_dir = Path(seq_dir)
        self.metadata = _read_metadata(self.seq_dir / "metadata.json")
        self.sequence_id = str(self.metadata.get("sequence_id", ""))
        self.split = self.metadata.get("split")
        if self.metadata.get("schema_version") != RECURRENT_DATASET_SCHEMA_VERSION:
            raise ValueError(f"unsupported recurrent schema in {self.seq_dir}")
        if self.sequence_id != self.seq_dir.name:
            raise ValueError(f"recurrent sequence identity mismatch in {self.seq_dir}")
        if self.split not in {"train", "val"}:
            raise ValueError(f"invalid recurrent split in {self.seq_dir}")
        if expected_split is not None and self.split != expected_split:
            raise ValueError(
                f"recurrent split mismatch: expected={expected_split}, "
                f"actual={self.split}"
            )
        if self.metadata.get("label_source") != (
            "lidar_precontact_teacher_recurrent_direct"
        ):
            raise ValueError(f"unexpected recurrent label source in {self.seq_dir}")
        if self.metadata.get("scan_shape") != [expected_input_dim]:
            raise ValueError(f"recurrent scan shape metadata mismatch in {self.seq_dir}")
        if not np.isclose(
            self.metadata.get("max_scan_range_m", np.nan), max_scan_range_m
        ):
            raise ValueError(f"recurrent range contract mismatch in {self.seq_dir}")
        recorded_sync = self.metadata.get("max_speed_sync_delta_sec")
        if not isinstance(recorded_sync, (int, float)) or (
            recorded_sync > max_speed_sync_delta_sec + 1e-12
        ):
            raise ValueError(f"recurrent speed sync contract too loose in {self.seq_dir}")

        missing = [
            name for name in RECURRENT_REQUIRED_ARRAYS
            if not (self.seq_dir / name).is_file()
        ]
        if missing:
            raise FileNotFoundError(
                f"missing recurrent arrays in {self.seq_dir}: {missing}"
            )
        self.scans = np.load(self.seq_dir / "scans.npy", allow_pickle=False)
        self.speeds = np.load(self.seq_dir / "speeds.npy", allow_pickle=False)
        self.steers = np.load(self.seq_dir / "steers.npy", allow_pickle=False)
        self.base_steers = np.load(
            self.seq_dir / "base_steers.npy", allow_pickle=False
        )
        self.scan_timestamps_ns = np.load(
            self.seq_dir / "scan_timestamps_ns.npy", allow_pickle=False
        )
        self.speed_timestamps_ns = np.load(
            self.seq_dir / "speed_timestamps_ns.npy", allow_pickle=False
        )
        self.speed_sync_deltas_sec = np.load(
            self.seq_dir / "speed_sync_deltas_sec.npy", allow_pickle=False
        )
        self._validate_arrays(expected_input_dim, max_scan_range_m)

    def _validate_arrays(self, input_dim: int, max_range_m: float) -> None:
        if self.scans.ndim != 2 or self.scans.shape[1] != input_dim:
            raise ValueError(f"invalid recurrent scans in {self.seq_dir}")
        one_dimensional = (
            self.speeds,
            self.steers,
            self.base_steers,
            self.scan_timestamps_ns,
            self.speed_timestamps_ns,
            self.speed_sync_deltas_sec,
        )
        if any(values.ndim != 1 for values in one_dimensional):
            raise ValueError(f"recurrent arrays must be one-dimensional in {self.seq_dir}")
        lengths = {len(self.scans), *(len(values) for values in one_dimensional)}
        if lengths != {len(self.scans)} or len(self.scans) == 0:
            raise ValueError(f"recurrent array length mismatch in {self.seq_dir}")
        for values in (self.scans, self.speeds, self.steers, self.base_steers):
            if not np.all(np.isfinite(values)):
                raise ValueError(f"non-finite recurrent data in {self.seq_dir}")
        if np.any(self.scans < 0.0) or np.any(self.scans > max_range_m):
            raise ValueError(f"recurrent scans outside range in {self.seq_dir}")
        if np.any(self.speeds < 0.0):
            raise ValueError(f"recurrent speed must be non-negative in {self.seq_dir}")
        if np.any(np.diff(self.scan_timestamps_ns) <= 0):
            raise ValueError(f"recurrent scan timestamps not increasing in {self.seq_dir}")
        if np.any(self.speed_sync_deltas_sec < 0.0) or np.any(
            self.speed_sync_deltas_sec
            > float(self.metadata["max_speed_sync_delta_sec"]) + 1e-12
        ):
            raise ValueError(f"recurrent speed sync delta violation in {self.seq_dir}")

    def __len__(self) -> int:
        return len(self.scans)

    def __getitem__(self, idx: int):
        return (
            self.scans[idx],
            np.float32(self.speeds[idx]),
            np.float32(self.steers[idx]),
            np.float32(self.base_steers[idx]),
        )


class MultiSeqRecurrentPolicyDataset:
    """Load every run under one split without flattening temporal identity."""

    def __init__(
        self,
        dataset_root: Union[str, Path],
        expected_split: str,
        include: Optional[Sequence[str]] = None,
    ):
        root = Path(dataset_root)
        sequence_dirs = [
            path for path in sorted(root.iterdir())
            if path.is_dir()
            and (not include or any(token in path.name for token in include))
        ] if root.is_dir() else []
        if not sequence_dirs:
            raise RuntimeError(f"no recurrent sequences found in {root}")
        self.datasets = [
            RecurrentPolicySequenceDataset(path, expected_split=expected_split)
            for path in sequence_dirs
        ]
        self.sequence_ids = [dataset.sequence_id for dataset in self.datasets]
        if len(self.sequence_ids) != len(set(self.sequence_ids)):
            raise ValueError("duplicate recurrent sequence identities")


class RecurrentPolicyChunkDataset(Dataset):
    """Fixed temporal chunks from one run; no item can cross a run boundary."""

    def __init__(
        self,
        sequence: RecurrentPolicySequenceDataset,
        chunk_length: int,
        stride: int,
    ):
        if chunk_length <= 1 or stride <= 0:
            raise ValueError("invalid recurrent chunk configuration")
        if len(sequence) < chunk_length:
            raise ValueError(
                f"sequence {sequence.sequence_id} shorter than chunk length"
            )
        self.sequence = sequence
        self.chunk_length = int(chunk_length)
        self.starts = list(range(0, len(sequence) - chunk_length + 1, stride))
        final_start = len(sequence) - chunk_length
        if self.starts[-1] != final_start:
            self.starts.append(final_start)

    def __len__(self) -> int:
        return len(self.starts)

    def __getitem__(self, idx: int):
        start = self.starts[idx]
        stop = start + self.chunk_length
        return (
            self.sequence.scans[start:stop],
            self.sequence.speeds[start:stop, None],
            self.sequence.steers[start:stop],
            self.sequence.base_steers[start:stop],
        )


def recurrent_chunk_dataset(
    sequences: MultiSeqRecurrentPolicyDataset,
    chunk_length: int,
    stride: int,
) -> ConcatDataset:
    return ConcatDataset([
        RecurrentPolicyChunkDataset(sequence, chunk_length, stride)
        for sequence in sequences.datasets
    ])


class RecurrentDirectSteeringPolicy(nn.Module):
    """Per-beam pressure tokens and ego speed with a compact GRU state."""

    def __init__(
        self,
        input_dim: int = 750,
        speed_embedding_dim: int = 64,
        hidden_dim: int = 512,
        max_speed_mps: float = 12.0,
        max_abs_steering_rad: float = 1.0,
    ):
        super().__init__()
        if min(input_dim, speed_embedding_dim, hidden_dim) <= 0:
            raise ValueError("recurrent dimensions must be positive")
        if max_speed_mps <= 0.0 or max_abs_steering_rad <= 0.0:
            raise ValueError("recurrent physical scales must be positive")
        self.input_dim = int(input_dim)
        self.max_speed_mps = float(max_speed_mps)
        self.max_abs_steering_rad = float(max_abs_steering_rad)
        initial_k = 0.1
        inverse_softplus = np.log(np.expm1(initial_k))
        self.pressure_log_k = nn.Parameter(
            torch.full((self.input_dim,), float(inverse_softplus))
        )
        self.speed_mlp = nn.Sequential(
            nn.Linear(1, speed_embedding_dim),
            nn.ReLU(),
        )
        self.gru = nn.GRU(
            input_size=self.input_dim + speed_embedding_dim,
            hidden_size=hidden_dim,
            num_layers=1,
            batch_first=True,
            bidirectional=False,
        )
        self.decoder = nn.Sequential(
            nn.Linear(hidden_dim, 128),
            nn.ReLU(),
            nn.Linear(128, 1),
        )
        self._initialize_weights()

    def _initialize_weights(self) -> None:
        for module in self.modules():
            if isinstance(module, nn.Linear):
                nn.init.xavier_uniform_(module.weight)
                nn.init.zeros_(module.bias)
        for name, parameter in self.gru.named_parameters():
            if "weight" in name:
                nn.init.xavier_uniform_(parameter)
            elif "bias" in name:
                nn.init.zeros_(parameter)

    def pressure_tokens(self, scans_m: torch.Tensor) -> torch.Tensor:
        k = F.softplus(self.pressure_log_k).view(1, 1, -1)
        return 2.0 * (1.0 - torch.sigmoid(k * scans_m))

    def forward(
        self,
        scans_m: torch.Tensor,
        speeds_mps: torch.Tensor,
        hidden: Optional[torch.Tensor] = None,
    ) -> tuple[torch.Tensor, torch.Tensor]:
        if scans_m.ndim != 3 or scans_m.shape[-1] != self.input_dim:
            raise ValueError("recurrent scans must have shape (batch, time, input_dim)")
        if speeds_mps.shape != scans_m.shape[:2] + (1,):
            raise ValueError("recurrent speeds must have shape (batch, time, 1)")
        pressure = self.pressure_tokens(scans_m)
        speed = torch.clamp(speeds_mps / self.max_speed_mps, 0.0, 1.5)
        features = torch.cat((pressure, self.speed_mlp(speed)), dim=-1)
        recurrent, next_hidden = self.gru(features, hidden)
        steering = (
            torch.tanh(self.decoder(recurrent)).squeeze(-1)
            * self.max_abs_steering_rad
        )
        return steering, next_hidden


def direct_policy_metrics(
    predictions: np.ndarray,
    targets: np.ndarray,
    base_predictions: np.ndarray,
    material_delta_rad: float,
) -> dict:
    predicted = np.asarray(predictions, dtype=np.float64)
    target = np.asarray(targets, dtype=np.float64)
    base = np.asarray(base_predictions, dtype=np.float64)
    if not (predicted.shape == target.shape == base.shape) or predicted.ndim != 1:
        raise ValueError("direct-policy metric arrays must be equal 1D arrays")
    material = np.abs(target - base) >= material_delta_rad

    def summarize(mask: np.ndarray) -> dict:
        if not np.any(mask):
            return {"samples": 0, "candidate_mae_rad": None, "base_mae_rad": None,
                    "improvement_fraction": None}
        candidate_mae = float(np.mean(np.abs(predicted[mask] - target[mask])))
        base_mae = float(np.mean(np.abs(base[mask] - target[mask])))
        improvement = None if base_mae == 0.0 else 1.0 - candidate_mae / base_mae
        return {
            "samples": int(np.count_nonzero(mask)),
            "candidate_mae_rad": candidate_mae,
            "base_mae_rad": base_mae,
            "improvement_fraction": improvement,
        }

    return {
        "all": summarize(np.ones(target.shape, dtype=bool)),
        "anchor": summarize(~material),
        "material": summarize(material),
    }


def weighted_direct_policy_smooth_l1(
    predictions: torch.Tensor,
    targets: torch.Tensor,
    base_predictions: torch.Tensor,
    material_delta_rad: float,
    material_weight: float,
    burn_in_steps: int = 0,
) -> torch.Tensor:
    """Train direct steering while preserving rare successor actions."""
    if not (
        predictions.shape == targets.shape == base_predictions.shape
        and predictions.ndim == 2
    ):
        raise ValueError("direct-policy training tensors must be equal 2D tensors")
    if material_delta_rad <= 0.0 or material_weight < 1.0:
        raise ValueError("invalid direct-policy material weighting")
    if burn_in_steps < 0 or burn_in_steps >= predictions.shape[1]:
        raise ValueError("burn-in must leave at least one supervised timestep")
    predictions = predictions[:, burn_in_steps:]
    targets = targets[:, burn_in_steps:]
    base_predictions = base_predictions[:, burn_in_steps:]
    material = torch.abs(targets - base_predictions) >= material_delta_rad
    weights = torch.where(
        material,
        torch.as_tensor(
            material_weight, dtype=predictions.dtype, device=predictions.device
        ),
        torch.ones((), dtype=predictions.dtype, device=predictions.device),
    )
    losses = F.smooth_l1_loss(predictions, targets, reduction="none")
    return torch.sum(losses * weights) / torch.sum(weights)
