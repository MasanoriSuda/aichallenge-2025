"""Sequence contracts and model for an offline recurrent direct policy."""

import json
from pathlib import Path
from typing import Optional, Sequence, Union

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import ConcatDataset, Dataset

from lib.model import TinyLidarNet
from lib.spatial_adapter import FrozenTinyLidarSpatialResidual


RECURRENT_DATASET_SCHEMA_VERSION = 2
RECURRENT_LABEL_SOURCES = frozenset(
    {
        "lidar_precontact_teacher_recurrent_direct",
        "lidar_speed_committed_teacher_recurrent_direct",
    }
)
RECURRENT_REQUIRED_ARRAYS = (
    "scans.npy",
    "speeds.npy",
    "steers.npy",
    "base_steers.npy",
    "scan_timestamps_ns.npy",
    "speed_timestamps_ns.npy",
    "speed_sync_deltas_sec.npy",
)
RECURRENT_ADAPTER_SPATIAL_FEATURES = ("compact_fc3", "projected_conv5")
RECURRENT_ADAPTER_SPATIAL_NORMALIZATIONS = ("none", "fixed_train_statistics")
RECURRENT_ADAPTER_CORRECTION_HEADS = ("direct", "signed_expert")


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
        if self.metadata.get("label_source") not in RECURRENT_LABEL_SOURCES:
            raise ValueError(f"unexpected recurrent label source in {self.seq_dir}")
        self.label_source = self.metadata["label_source"]
        if self.metadata.get("scan_shape") != [expected_input_dim]:
            raise ValueError(f"recurrent scan shape metadata mismatch in {self.seq_dir}")
        if self.metadata.get("scan_unit") != "m":
            raise ValueError(f"recurrent scan unit must be metres in {self.seq_dir}")
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


class FrozenTinyLidarRecurrentAdapter(nn.Module):
    """Preserve an admitted base policy and learn only temporal corrections."""

    def __init__(
        self,
        input_dim: int = 750,
        speed_embedding_dim: int = 32,
        hidden_dim: int = 128,
        max_scan_range_m: float = 30.0,
        max_speed_mps: float = 12.0,
        max_abs_correction_rad: float = 0.64,
        max_abs_steering_rad: float = 1.0,
        include_pressure_tokens: bool = False,
        spatial_features: str = "compact_fc3",
        spatial_projection_dim: int = 128,
        spatial_projection_seed: int = 2026,
        spatial_normalization: str = "none",
        use_speed: bool = True,
        frozen_spatial_baseline_config: Optional[dict] = None,
        correction_head: str = "direct",
    ):
        super().__init__()
        if min(input_dim, speed_embedding_dim, hidden_dim) <= 0:
            raise ValueError("recurrent adapter dimensions must be positive")
        if min(
            max_scan_range_m,
            max_speed_mps,
            max_abs_correction_rad,
            max_abs_steering_rad,
        ) <= 0.0:
            raise ValueError("recurrent adapter physical scales must be positive")
        if spatial_features not in RECURRENT_ADAPTER_SPATIAL_FEATURES:
            raise ValueError(f"unsupported recurrent spatial features: {spatial_features}")
        if spatial_normalization not in RECURRENT_ADAPTER_SPATIAL_NORMALIZATIONS:
            raise ValueError(
                f"unsupported recurrent spatial normalization: {spatial_normalization}"
            )
        if spatial_projection_dim <= 0:
            raise ValueError("recurrent spatial projection dimension must be positive")
        if spatial_features == "compact_fc3" and spatial_normalization != "none":
            raise ValueError("legacy compact recurrent features require no normalization")
        if spatial_features == "projected_conv5" and include_pressure_tokens:
            raise ValueError("projected conv5 must not duplicate raw pressure tokens")
        if correction_head not in RECURRENT_ADAPTER_CORRECTION_HEADS:
            raise ValueError(f"unsupported recurrent correction head: {correction_head}")
        self.input_dim = int(input_dim)
        self.max_scan_range_m = float(max_scan_range_m)
        self.max_speed_mps = float(max_speed_mps)
        self.max_abs_correction_rad = float(max_abs_correction_rad)
        self.max_abs_steering_rad = float(max_abs_steering_rad)
        self.include_pressure_tokens = bool(include_pressure_tokens)
        self.spatial_features = spatial_features
        self.spatial_projection_dim = int(spatial_projection_dim)
        self.spatial_projection_seed = int(spatial_projection_seed)
        self.spatial_normalization = spatial_normalization
        self.use_speed = bool(use_speed)
        self.correction_head = correction_head
        self.base = TinyLidarNet(input_dim=self.input_dim, output_dim=2)
        for parameter in self.base.parameters():
            parameter.requires_grad_(False)
        self.frozen_spatial_baseline_config = (
            None
            if frozen_spatial_baseline_config is None
            else dict(frozen_spatial_baseline_config)
        )
        if self.frozen_spatial_baseline_config is not None:
            if self.frozen_spatial_baseline_config.get("input_dim") != self.input_dim:
                raise ValueError("frozen spatial baseline input dimension mismatch")
            if not self.frozen_spatial_baseline_config.get("use_speed", False):
                raise ValueError("frozen production spatial baseline must use speed")
            self.spatial_baseline = FrozenTinyLidarSpatialResidual(
                **self.frozen_spatial_baseline_config
            )
            for parameter in self.spatial_baseline.parameters():
                parameter.requires_grad_(False)
        else:
            self.spatial_baseline = None
        if self.use_speed:
            self.speed_mlp = nn.Sequential(
                nn.Linear(1, speed_embedding_dim),
                nn.ReLU(),
            )
        else:
            self.speed_mlp = None
        if self.spatial_features == "projected_conv5":
            with torch.no_grad():
                dummy = torch.zeros(1, 1, self.input_dim)
                conv5 = self._extract_conv5(dummy).flatten(start_dim=1)
                self.conv5_dim = int(conv5.shape[1])
            generator = np.random.default_rng(self.spatial_projection_seed)
            projection = (
                generator.standard_normal(
                    (self.conv5_dim, self.spatial_projection_dim),
                    dtype=np.float32,
                )
                / np.sqrt(float(self.spatial_projection_dim))
            ).astype(np.float32)
            self.register_buffer(
                "spatial_projection", torch.from_numpy(projection)
            )
            self.register_buffer(
                "spatial_mean", torch.zeros(self.spatial_projection_dim)
            )
            self.register_buffer(
                "spatial_scale", torch.ones(self.spatial_projection_dim)
            )
            recurrent_spatial_dim = self.spatial_projection_dim
        else:
            self.conv5_dim = int(self.base.flatten_dim)
            self.register_buffer("spatial_projection", None)
            self.register_buffer("spatial_mean", None)
            self.register_buffer("spatial_scale", None)
            recurrent_spatial_dim = 10
        if self.include_pressure_tokens:
            initial_k = 0.1
            inverse_softplus = np.log(np.expm1(initial_k))
            self.pressure_log_k = nn.Parameter(
                torch.full((self.input_dim,), float(inverse_softplus))
            )
        else:
            self.register_parameter("pressure_log_k", None)
        self.gru = nn.GRU(
            input_size=(
                recurrent_spatial_dim
                + (speed_embedding_dim if self.use_speed else 0)
                + (self.input_dim if self.include_pressure_tokens else 0)
            ),
            hidden_size=hidden_dim,
            num_layers=1,
            batch_first=True,
            bidirectional=False,
        )
        self.correction_hidden = nn.Linear(hidden_dim, 64)
        if self.correction_head == "direct":
            self.correction_output = nn.Linear(64, 1)
            self.direction_output = None
            self.magnitude_output = None
        else:
            self.correction_output = None
            self.direction_output = nn.Linear(64, 3)
            self.magnitude_output = nn.Linear(64, 2)
        self._initialize_adapter()

    def _initialize_adapter(self) -> None:
        modules = [self.correction_hidden]
        if self.speed_mlp is not None:
            modules.append(self.speed_mlp[0])
        for module in modules:
            nn.init.xavier_uniform_(module.weight)
            nn.init.zeros_(module.bias)
        for name, parameter in self.gru.named_parameters():
            if "weight" in name:
                nn.init.xavier_uniform_(parameter)
            elif "bias" in name:
                nn.init.zeros_(parameter)
        # Exact zero makes the composed candidate identical to the admitted base.
        if self.correction_head == "direct":
            nn.init.zeros_(self.correction_output.weight)
            nn.init.zeros_(self.correction_output.bias)
        else:
            nn.init.zeros_(self.direction_output.weight)
            nn.init.zeros_(self.direction_output.bias)
            # Neutral wins the initial categorical decode deterministically.
            with torch.no_grad():
                self.direction_output.bias[1] = 1.0
            nn.init.zeros_(self.magnitude_output.weight)
            nn.init.zeros_(self.magnitude_output.bias)

    def _extract_conv5(self, normalized: torch.Tensor) -> torch.Tensor:
        features = F.relu(self.base.conv1(normalized))
        features = F.relu(self.base.conv2(features))
        features = F.relu(self.base.conv3(features))
        features = F.relu(self.base.conv4(features))
        return F.relu(self.base.conv5(features))

    def _base_features_and_steering(
        self, scans_m: torch.Tensor, speeds_mps: Optional[torch.Tensor] = None
    ) -> tuple[torch.Tensor, torch.Tensor]:
        batch, time, _ = scans_m.shape
        normalized = torch.clamp(
            scans_m / self.max_scan_range_m, 0.0, 1.0
        ).reshape(batch * time, 1, self.input_dim)
        # Frozen features need no activation tape during adapter training.
        with torch.no_grad():
            conv5 = torch.flatten(self._extract_conv5(normalized), start_dim=1)
            compact = F.relu(self.base.fc1(conv5))
            compact = F.relu(self.base.fc2(compact))
            compact = F.relu(self.base.fc3(compact))
            base_output = torch.tanh(self.base.fc4(compact))[:, 1]
            if self.spatial_baseline is not None:
                if speeds_mps is None or speeds_mps.shape != scans_m.shape[:2] + (1,):
                    raise ValueError(
                        "frozen production spatial baseline requires causal speed"
                    )
                spatial_residual = self.spatial_baseline(
                    scans_m.reshape(batch * time, self.input_dim),
                    speeds_mps.reshape(batch * time),
                )
                base_output = torch.clamp(
                    base_output + spatial_residual,
                    -self.max_abs_steering_rad,
                    self.max_abs_steering_rad,
                )
            if self.spatial_features == "projected_conv5":
                features = conv5 @ self.spatial_projection
                if self.spatial_normalization == "fixed_train_statistics":
                    features = (features - self.spatial_mean) / self.spatial_scale
            else:
                features = compact
        return (
            features.reshape(batch, time, -1),
            base_output.reshape(batch, time),
        )

    def projected_spatial_features(self, scans_m: torch.Tensor) -> torch.Tensor:
        """Return unnormalized frozen projected-conv5 features for statistics."""
        if self.spatial_features != "projected_conv5":
            raise RuntimeError("projected spatial features are disabled")
        if scans_m.ndim not in {2, 3} or scans_m.shape[-1] != self.input_dim:
            raise ValueError("adapter scans must end with input_dim")
        leading = scans_m.shape[:-1]
        normalized = torch.clamp(
            scans_m / self.max_scan_range_m, 0.0, 1.0
        ).reshape(-1, 1, self.input_dim)
        with torch.no_grad():
            conv5 = torch.flatten(self._extract_conv5(normalized), start_dim=1)
            projected = conv5 @ self.spatial_projection
        return projected.reshape(*leading, self.spatial_projection_dim)

    def set_spatial_statistics(
        self, mean: torch.Tensor, scale: torch.Tensor
    ) -> None:
        if (
            self.spatial_features != "projected_conv5"
            or self.spatial_normalization != "fixed_train_statistics"
        ):
            raise ValueError("fixed projected statistics are not enabled")
        expected = (self.spatial_projection_dim,)
        if mean.shape != expected or scale.shape != expected:
            raise ValueError("recurrent spatial statistics dimension mismatch")
        if not torch.isfinite(mean).all() or not torch.isfinite(scale).all():
            raise ValueError("recurrent spatial statistics must be finite")
        if torch.any(scale <= 0.0):
            raise ValueError("recurrent spatial scale must be positive")
        self.spatial_mean.copy_(mean.to(self.spatial_mean))
        self.spatial_scale.copy_(scale.to(self.spatial_scale))

    def base_steering(
        self,
        scans_m: torch.Tensor,
        speeds_mps: Optional[torch.Tensor] = None,
    ) -> torch.Tensor:
        if scans_m.ndim != 3 or scans_m.shape[-1] != self.input_dim:
            raise ValueError("adapter scans must have shape (batch, time, input_dim)")
        _, steering = self._base_features_and_steering(scans_m, speeds_mps)
        return steering

    def pressure_tokens(self, scans_m: torch.Tensor) -> torch.Tensor:
        if not self.include_pressure_tokens or self.pressure_log_k is None:
            raise RuntimeError("adapter pressure tokens are disabled")
        k = F.softplus(self.pressure_log_k).view(1, 1, -1)
        return 2.0 * (1.0 - torch.sigmoid(k * scans_m))

    def forward_correction_components(
        self,
        scans_m: torch.Tensor,
        speeds_mps: torch.Tensor,
        hidden: Optional[torch.Tensor] = None,
    ) -> tuple[
        torch.Tensor,
        torch.Tensor,
        torch.Tensor,
        Optional[torch.Tensor],
        Optional[torch.Tensor],
        torch.Tensor,
    ]:
        if scans_m.ndim != 3 or scans_m.shape[-1] != self.input_dim:
            raise ValueError("adapter scans must have shape (batch, time, input_dim)")
        if speeds_mps.shape != scans_m.shape[:2] + (1,):
            raise ValueError("adapter speeds must have shape (batch, time, 1)")
        features, base_steering = self._base_features_and_steering(
            scans_m, speeds_mps
        )
        recurrent_inputs = [features]
        if self.use_speed:
            speed = torch.clamp(speeds_mps / self.max_speed_mps, 0.0, 1.5)
            recurrent_inputs.append(self.speed_mlp(speed))
        if self.include_pressure_tokens:
            recurrent_inputs.append(self.pressure_tokens(scans_m))
        recurrent, next_hidden = self.gru(
            torch.cat(recurrent_inputs, dim=-1), hidden
        )
        correction_features = F.relu(self.correction_hidden(recurrent))
        magnitudes = None
        direction_logits = None
        if self.correction_head == "direct":
            correction = (
                torch.tanh(self.correction_output(correction_features)).squeeze(-1)
                * self.max_abs_correction_rad
            )
        else:
            direction_logits = self.direction_output(correction_features)
            magnitudes = (
                torch.sigmoid(self.magnitude_output(correction_features))
                * self.max_abs_correction_rad
            )
            classes = torch.argmax(direction_logits, dim=-1)
            correction = torch.where(
                classes == 0,
                -magnitudes[..., 0],
                torch.where(
                    classes == 2,
                    magnitudes[..., 1],
                    torch.zeros_like(magnitudes[..., 0]),
                ),
            )
        steering = torch.clamp(
            base_steering + correction,
            -self.max_abs_steering_rad,
            self.max_abs_steering_rad,
        )
        return (
            steering,
            correction,
            base_steering,
            magnitudes,
            direction_logits,
            next_hidden,
        )

    def forward_components(
        self,
        scans_m: torch.Tensor,
        speeds_mps: torch.Tensor,
        hidden: Optional[torch.Tensor] = None,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        steering, correction, base, _, _, next_hidden = (
            self.forward_correction_components(scans_m, speeds_mps, hidden)
        )
        return steering, correction, base, next_hidden

    def forward(
        self,
        scans_m: torch.Tensor,
        speeds_mps: torch.Tensor,
        hidden: Optional[torch.Tensor] = None,
    ) -> tuple[torch.Tensor, torch.Tensor]:
        steering, _, _, next_hidden = self.forward_components(
            scans_m, speeds_mps, hidden
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
