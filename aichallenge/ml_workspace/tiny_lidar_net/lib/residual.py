"""Contracts and model for a separately learned steering residual."""

import json
from pathlib import Path
from typing import Optional, Sequence, Union

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import ConcatDataset, Dataset

from lib.data import ScanControlSequenceDataset


RESIDUAL_TARGET_FILES = (
    "base_steers.npy",
    "reference_steers.npy",
    "steering_deltas.npy",
)
RESIDUAL_INPUT_MODES = ("stateless", "scan_delta")
RESIDUAL_HEAD_ARCHITECTURES = ("binary_gate", "signed_mixture")


def residual_input_channels(input_mode: str) -> int:
    if input_mode not in RESIDUAL_INPUT_MODES:
        raise ValueError(
            f"unsupported residual input mode: {input_mode}; "
            f"expected one of {RESIDUAL_INPUT_MODES}"
        )
    return 1 if input_mode == "stateless" else 2


def compose_residual_input(
    current_scan: np.ndarray,
    previous_scan: np.ndarray,
    input_mode: str,
) -> np.ndarray:
    """Build the exact offline/runtime temporal feature contract."""
    current = np.asarray(current_scan, dtype=np.float32)
    previous = np.asarray(previous_scan, dtype=np.float32)
    if current.shape != previous.shape or current.ndim != 1:
        raise ValueError("residual scan inputs must be equal one-dimensional arrays")
    if input_mode == "stateless":
        return current
    residual_input_channels(input_mode)
    return np.stack((current, current - previous)).astype(np.float32, copy=False)


class SteeringResidualSequenceDataset(ScanControlSequenceDataset):
    """One run whose successor/reference teacher identity is explicit."""

    def __init__(
        self,
        seq_dir: Union[str, Path],
        max_range: float = 30.0,
        expected_input_dim: int = 750,
        expected_split: Optional[str] = None,
        material_delta_rad: float = 0.02,
        input_mode: str = "stateless",
    ):
        super().__init__(
            seq_dir,
            max_range=max_range,
            expected_input_dim=expected_input_dim,
            allowed_label_sources=("lidar_precontact_teacher_dagger",),
            require_metadata=True,
            max_sync_delta_sec=0.0,
            expected_split=expected_split,
        )
        if not np.isfinite(material_delta_rad) or material_delta_rad <= 0.0:
            raise ValueError("material_delta_rad must be finite and positive")
        residual_input_channels(input_mode)
        self.input_mode = input_mode
        missing = [
            name for name in RESIDUAL_TARGET_FILES
            if not (self.seq_dir / name).is_file()
        ]
        if missing:
            raise FileNotFoundError(
                f"Missing runtime residual target arrays in {self.seq_dir}: {missing}"
            )

        self.base_steers = np.load(
            self.seq_dir / "base_steers.npy", allow_pickle=False
        )
        self.reference_steers = np.load(
            self.seq_dir / "reference_steers.npy", allow_pickle=False
        )
        self.steering_deltas = np.load(
            self.seq_dir / "steering_deltas.npy", allow_pickle=False
        )
        expected_shape = self.steers.shape
        for name, values in (
            ("base_steers", self.base_steers),
            ("reference_steers", self.reference_steers),
            ("steering_deltas", self.steering_deltas),
        ):
            if values.shape != expected_shape:
                raise ValueError(
                    f"{name} shape mismatch in {self.seq_dir}: "
                    f"expected={expected_shape}, actual={values.shape}"
                )
            if not np.all(np.isfinite(values)):
                raise ValueError(f"Non-finite {name} in {self.seq_dir}")
        if not np.allclose(
            self.steers - self.base_steers,
            self.steering_deltas,
            rtol=1e-6,
            atol=1e-7,
        ):
            raise ValueError(
                f"Successor-base residual identity mismatch in {self.seq_dir}"
            )

        residual_contract = (
            self.metadata.get("relabeling", {})
            .get("residual_target")
        )
        if not isinstance(residual_contract, dict):
            raise ValueError(
                f"Missing residual_target metadata in {self.seq_dir}"
            )
        expected_contract = {
            "target_definition": "successor_steering_minus_base_steering",
            "runtime_composition": "base_steering_plus_learned_residual",
            "successor_teacher": "LidarPrecontactTeacher",
            "base_policy": "frozen_production_tiny_lidar_net",
        }
        for key, expected in expected_contract.items():
            if residual_contract.get(key) != expected:
                raise ValueError(
                    f"Residual provenance mismatch for {key} in {self.seq_dir}: "
                    f"expected={expected}, actual={residual_contract.get(key)}"
                )
        recorded_threshold = residual_contract.get("material_delta_rad")
        if not isinstance(recorded_threshold, (int, float)) or not np.isclose(
            recorded_threshold, material_delta_rad
        ):
            raise ValueError(
                f"Material residual threshold mismatch in {self.seq_dir}: "
                f"expected={material_delta_rad}, actual={recorded_threshold}"
            )

    def __getitem__(self, idx: int):
        scan, _ = super().__getitem__(idx)
        previous_scan = scan if idx == 0 else self.scans[idx - 1]
        model_input = compose_residual_input(scan, previous_scan, self.input_mode)
        return model_input, np.float32(self.steering_deltas[idx])


class MultiSeqResidualDataset(ConcatDataset):
    """Concatenate paired residual sequences after run-level validation."""

    def __init__(
        self,
        dataset_root: Union[str, Path],
        expected_split: str,
        max_range: float = 30.0,
        expected_input_dim: int = 750,
        material_delta_rad: float = 0.02,
        include: Optional[Sequence[str]] = None,
        input_mode: str = "stateless",
    ):
        root = Path(dataset_root)
        if not root.is_dir():
            raise FileNotFoundError(f"Residual dataset root does not exist: {root}")
        sequence_dirs = [
            path
            for path in sorted(root.iterdir())
            if path.is_dir()
            and (not include or any(token in path.name for token in include))
        ]
        if not sequence_dirs:
            raise RuntimeError(f"No residual sequence directories found in {root}")
        datasets = [
            SteeringResidualSequenceDataset(
                path,
                max_range=max_range,
                expected_input_dim=expected_input_dim,
                expected_split=expected_split,
                material_delta_rad=material_delta_rad,
                input_mode=input_mode,
            )
            for path in sequence_dirs
        ]
        self.sequence_ids = [dataset.sequence_id for dataset in datasets]
        if len(self.sequence_ids) != len(set(self.sequence_ids)):
            raise ValueError(f"Duplicate residual sequence IDs: {self.sequence_ids}")
        super().__init__(datasets)


class ResidualInputSequenceView(Dataset):
    """Apply temporal input construction without crossing one run boundary."""

    def __init__(self, dataset: Dataset, input_mode: str):
        residual_input_channels(input_mode)
        self.dataset = dataset
        self.input_mode = input_mode

    def __len__(self) -> int:
        return len(self.dataset)

    def __getitem__(self, idx: int):
        current_scan, target = self.dataset[idx]
        previous_scan = (
            current_scan if idx == 0 else self.dataset[idx - 1][0]
        )
        return (
            compose_residual_input(current_scan, previous_scan, self.input_mode),
            target,
        )


def adapt_concat_dataset_input(dataset: ConcatDataset, input_mode: str):
    """Adapt every child independently so history cannot leak across runs."""
    residual_input_channels(input_mode)
    if input_mode == "stateless":
        return dataset
    return ConcatDataset(
        [ResidualInputSequenceView(child, input_mode) for child in dataset.datasets]
    )


def sequence_balanced_sample_weights(
    sequence_lengths: Sequence[int],
) -> torch.Tensor:
    """Give every closed-loop sequence equal probability mass.

    DAgger appends short causal prefixes to much longer successful runs.  A
    plain shuffled concatenation therefore makes a run's influence depend on
    recording duration instead of evidence identity.  Equal total mass per
    sequence preserves all samples while preventing a new failure prefix from
    disappearing inside the aggregate dataset.
    """
    lengths = [int(length) for length in sequence_lengths]
    if not lengths or any(length <= 0 for length in lengths):
        raise ValueError("sequence lengths must be non-empty and positive")
    weights = np.concatenate(
        [np.full(length, 1.0 / length, dtype=np.float64) for length in lengths]
    )
    weights /= np.sum(weights)
    return torch.from_numpy(weights)


class SteeringResidualNet(nn.Module):
    """A bounded LiDAR steering correction with an exact zero initial policy."""

    def __init__(
        self,
        input_dim: int = 750,
        max_abs_delta_rad: float = 1.28,
        input_channels: int = 1,
    ):
        super().__init__()
        if input_dim <= 0:
            raise ValueError("input_dim must be positive")
        if not np.isfinite(max_abs_delta_rad) or max_abs_delta_rad <= 0.0:
            raise ValueError("max_abs_delta_rad must be finite and positive")
        if input_channels not in (1, 2):
            raise ValueError("input_channels must be 1 or 2")
        self.max_abs_delta_rad = float(max_abs_delta_rad)
        self.input_channels = int(input_channels)
        self.conv1 = nn.Conv1d(self.input_channels, 16, kernel_size=10, stride=4)
        self.conv2 = nn.Conv1d(16, 24, kernel_size=8, stride=4)
        self.conv3 = nn.Conv1d(24, 32, kernel_size=4, stride=2)
        with torch.no_grad():
            dummy = torch.zeros(1, self.input_channels, input_dim)
            encoded = self.conv3(self.conv2(self.conv1(dummy)))
            flatten_dim = encoded.flatten(start_dim=1).shape[1]
        self.fc1 = nn.Linear(flatten_dim, 64)
        self.fc2 = nn.Linear(64, 16)
        self.correction_head = nn.Linear(16, 1)
        self.gate_head = nn.Linear(16, 1)
        self._initialize_weights()

    def _initialize_weights(self) -> None:
        for layer in (self.conv1, self.conv2, self.conv3, self.fc1, self.fc2):
            nn.init.kaiming_normal_(layer.weight, mode="fan_out", nonlinearity="relu")
            nn.init.zeros_(layer.bias)
        # Before any residual training, composition with the frozen production
        # base is exactly the production policy for every possible scan.
        nn.init.zeros_(self.correction_head.weight)
        nn.init.zeros_(self.correction_head.bias)
        nn.init.zeros_(self.gate_head.weight)
        nn.init.zeros_(self.gate_head.bias)

    def _encode(self, scans: torch.Tensor) -> torch.Tensor:
        features = F.relu(self.conv1(scans))
        features = F.relu(self.conv2(features))
        features = F.relu(self.conv3(features))
        features = torch.flatten(features, start_dim=1)
        features = F.relu(self.fc1(features))
        return F.relu(self.fc2(features))

    def forward_components(
        self, scans: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """Return composed residual, ungated correction and gate logit."""
        features = self._encode(scans)
        correction = (
            torch.tanh(self.correction_head(features)).squeeze(-1)
            * self.max_abs_delta_rad
        )
        gate_logit = self.gate_head(features).squeeze(-1)
        residual = torch.sigmoid(gate_logit) * correction
        return residual, correction, gate_logit

    def forward(self, scans: torch.Tensor) -> torch.Tensor:
        residual, _, _ = self.forward_components(scans)
        return residual


class SignedMixtureSteeringResidualNet(nn.Module):
    """Three-way residual which does not regress opposing actions together."""

    def __init__(
        self,
        input_dim: int = 750,
        max_abs_delta_rad: float = 1.28,
        input_channels: int = 1,
    ):
        super().__init__()
        if input_dim <= 0:
            raise ValueError("input_dim must be positive")
        if not np.isfinite(max_abs_delta_rad) or max_abs_delta_rad <= 0.0:
            raise ValueError("max_abs_delta_rad must be finite and positive")
        if input_channels not in (1, 2):
            raise ValueError("input_channels must be 1 or 2")
        self.max_abs_delta_rad = float(max_abs_delta_rad)
        self.input_channels = int(input_channels)
        self.conv1 = nn.Conv1d(self.input_channels, 16, kernel_size=10, stride=4)
        self.conv2 = nn.Conv1d(16, 24, kernel_size=8, stride=4)
        self.conv3 = nn.Conv1d(24, 32, kernel_size=4, stride=2)
        with torch.no_grad():
            dummy = torch.zeros(1, self.input_channels, input_dim)
            encoded = self.conv3(self.conv2(self.conv1(dummy)))
            flatten_dim = encoded.flatten(start_dim=1).shape[1]
        self.fc1 = nn.Linear(flatten_dim, 64)
        self.fc2 = nn.Linear(64, 16)
        self.direction_head = nn.Linear(16, 3)
        self.magnitude_head = nn.Linear(16, 2)
        self._initialize_weights()

    def _initialize_weights(self) -> None:
        for layer in (self.conv1, self.conv2, self.conv3, self.fc1, self.fc2):
            nn.init.kaiming_normal_(layer.weight, mode="fan_out", nonlinearity="relu")
            nn.init.zeros_(layer.bias)
        # Equal negative/positive probabilities and magnitudes cancel exactly,
        # preserving the frozen base policy before training.
        nn.init.zeros_(self.direction_head.weight)
        nn.init.zeros_(self.direction_head.bias)
        nn.init.zeros_(self.magnitude_head.weight)
        nn.init.zeros_(self.magnitude_head.bias)

    def _encode(self, scans: torch.Tensor) -> torch.Tensor:
        features = F.relu(self.conv1(scans))
        features = F.relu(self.conv2(features))
        features = F.relu(self.conv3(features))
        features = torch.flatten(features, start_dim=1)
        features = F.relu(self.fc1(features))
        return F.relu(self.fc2(features))

    def forward_components(
        self, scans: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        features = self._encode(scans)
        direction_logits = self.direction_head(features)
        direction_probabilities = torch.softmax(direction_logits, dim=-1)
        magnitudes = torch.sigmoid(self.magnitude_head(features)) * self.max_abs_delta_rad
        residual = (
            direction_probabilities[:, 2] * magnitudes[:, 1]
            - direction_probabilities[:, 0] * magnitudes[:, 0]
        )
        return residual, magnitudes, direction_logits, direction_probabilities

    def forward(self, scans: torch.Tensor) -> torch.Tensor:
        residual, _, _, _ = self.forward_components(scans)
        return residual


def signed_direction_targets(
    targets: torch.Tensor, material_delta_rad: float
) -> torch.Tensor:
    """Map correction targets to negative/anchor/positive class IDs."""
    if material_delta_rad <= 0.0:
        raise ValueError("material_delta_rad must be positive")
    classes = torch.ones_like(targets, dtype=torch.long)
    classes = torch.where(targets <= -material_delta_rad, 0, classes)
    return torch.where(targets >= material_delta_rad, 2, classes)


def signed_mixture_training_loss(
    residual: torch.Tensor,
    magnitudes: torch.Tensor,
    direction_logits: torch.Tensor,
    targets: torch.Tensor,
    material_delta_rad: float,
    material_weight: float,
    direction_class_weights: torch.Tensor,
    direction_loss_weight: float,
    anchor_leakage_weight: float,
) -> torch.Tensor:
    """Train direction separately so opposite homotopies are not averaged."""
    if residual.shape != targets.shape:
        raise ValueError("signed mixture residual and targets must match")
    if magnitudes.shape != (targets.shape[0], 2):
        raise ValueError("signed mixture magnitudes must have shape (batch, 2)")
    if direction_logits.shape != (targets.shape[0], 3):
        raise ValueError("signed mixture logits must have shape (batch, 3)")
    if direction_class_weights.shape != (3,):
        raise ValueError("direction_class_weights must have shape (3,)")
    if direction_loss_weight < 0.0 or anchor_leakage_weight < 0.0:
        raise ValueError("signed mixture loss weights must be non-negative")

    classes = signed_direction_targets(targets, material_delta_rad)
    composed_loss = weighted_residual_smooth_l1(
        residual, targets, material_delta_rad, material_weight
    )
    direction_loss = F.cross_entropy(
        direction_logits,
        classes,
        weight=direction_class_weights.to(
            device=direction_logits.device, dtype=direction_logits.dtype
        ),
    )
    material = classes != 1
    if torch.any(material):
        selected_magnitudes = torch.where(
            classes[material] == 0,
            magnitudes[material, 0],
            magnitudes[material, 1],
        )
        magnitude_loss = F.smooth_l1_loss(
            selected_magnitudes, torch.abs(targets[material])
        )
    else:
        magnitude_loss = torch.zeros((), device=targets.device, dtype=targets.dtype)
    anchors = ~material
    if torch.any(anchors):
        anchor_loss = torch.mean(torch.abs(residual[anchors]))
    else:
        anchor_loss = torch.zeros((), device=targets.device, dtype=targets.dtype)
    return (
        composed_loss
        + magnitude_loss
        + direction_loss_weight * direction_loss
        + anchor_leakage_weight * anchor_loss
    )


def weighted_residual_smooth_l1(
    predictions: torch.Tensor,
    targets: torch.Tensor,
    material_delta_rad: float,
    material_weight: float,
) -> torch.Tensor:
    """Keep zero anchors while giving sparse material corrections bounded weight."""
    if predictions.shape != targets.shape:
        raise ValueError(
            f"residual prediction/target shape mismatch: {predictions.shape} vs {targets.shape}"
        )
    if material_delta_rad <= 0.0 or material_weight < 1.0:
        raise ValueError("invalid material residual loss configuration")
    per_sample = F.smooth_l1_loss(predictions, targets, reduction="none")
    weights = torch.where(
        torch.abs(targets) >= material_delta_rad,
        torch.as_tensor(material_weight, dtype=per_sample.dtype, device=per_sample.device),
        torch.ones((), dtype=per_sample.dtype, device=per_sample.device),
    )
    # Normalize by total weight so learning-rate meaning does not depend on the
    # positive fraction in a batch.
    return torch.sum(per_sample * weights) / torch.sum(weights)


def residual_training_loss(
    residual: torch.Tensor,
    correction: torch.Tensor,
    gate_logits: torch.Tensor,
    targets: torch.Tensor,
    material_delta_rad: float,
    material_weight: float,
    gate_loss_weight: float,
    anchor_leakage_weight: float,
) -> torch.Tensor:
    """Train correction magnitude and learned activation without normal leakage."""
    if not (
        residual.shape == correction.shape == gate_logits.shape == targets.shape
    ):
        raise ValueError("residual training tensors must have identical shapes")
    if gate_loss_weight < 0.0 or anchor_leakage_weight < 0.0:
        raise ValueError("residual auxiliary loss weights must be non-negative")
    material = torch.abs(targets) >= material_delta_rad
    composed_loss = weighted_residual_smooth_l1(
        residual,
        targets,
        material_delta_rad,
        material_weight,
    )
    gate_targets = material.to(dtype=gate_logits.dtype)
    positive_weight = torch.as_tensor(
        material_weight, dtype=gate_logits.dtype, device=gate_logits.device
    )
    gate_loss = F.binary_cross_entropy_with_logits(
        gate_logits,
        gate_targets,
        pos_weight=positive_weight,
    )
    if torch.any(material):
        correction_loss = F.smooth_l1_loss(
            correction[material], targets[material]
        )
    else:
        correction_loss = torch.zeros((), device=targets.device, dtype=targets.dtype)
    anchors = ~material
    if torch.any(anchors):
        anchor_loss = torch.mean(torch.abs(residual[anchors]))
    else:
        anchor_loss = torch.zeros((), device=targets.device, dtype=targets.dtype)
    return (
        composed_loss
        + correction_loss
        + gate_loss_weight * gate_loss
        + anchor_leakage_weight * anchor_loss
    )


def residual_metrics(
    predictions: np.ndarray,
    targets: np.ndarray,
    material_delta_rad: float,
) -> dict:
    """Report correction and anchor quality separately."""
    predicted = np.asarray(predictions, dtype=np.float64).reshape(-1)
    target = np.asarray(targets, dtype=np.float64).reshape(-1)
    if predicted.shape != target.shape or predicted.size == 0:
        raise ValueError("residual metrics require equal non-empty arrays")
    if not np.all(np.isfinite(predicted)) or not np.all(np.isfinite(target)):
        raise ValueError("residual metrics require finite arrays")

    def summarize(mask: np.ndarray) -> dict:
        if not np.any(mask):
            return {
                "samples": 0,
                "mae_rad": None,
                "rmse_rad": None,
                "p95_rad": None,
                "mean_prediction_rad": None,
                "mean_target_rad": None,
                "p05_prediction_rad": None,
                "p95_prediction_rad": None,
                "p05_target_rad": None,
                "p95_target_rad": None,
            }
        errors = np.abs(predicted[mask] - target[mask])
        selected_predictions = predicted[mask]
        selected_targets = target[mask]
        return {
            "samples": int(np.count_nonzero(mask)),
            "mae_rad": float(np.mean(errors)),
            "rmse_rad": float(np.sqrt(np.mean(np.square(errors)))),
            "p95_rad": float(np.quantile(errors, 0.95)),
            "mean_prediction_rad": float(np.mean(selected_predictions)),
            "mean_target_rad": float(np.mean(selected_targets)),
            "p05_prediction_rad": float(np.quantile(selected_predictions, 0.05)),
            "p95_prediction_rad": float(np.quantile(selected_predictions, 0.95)),
            "p05_target_rad": float(np.quantile(selected_targets, 0.05)),
            "p95_target_rad": float(np.quantile(selected_targets, 0.95)),
        }

    material = np.abs(target) >= material_delta_rad
    zero_baseline_material_mae = (
        float(np.mean(np.abs(target[material]))) if np.any(material) else None
    )
    result = {
        "all": summarize(np.ones(target.shape, dtype=bool)),
        "material": summarize(material),
        "anchor": summarize(~material),
        "zero_baseline_material_mae_rad": zero_baseline_material_mae,
    }
    material_mae = result["material"]["mae_rad"]
    result["material_mae_improvement_fraction"] = (
        None
        if material_mae is None or not zero_baseline_material_mae
        else float(1.0 - material_mae / zero_baseline_material_mae)
    )
    return result


def save_numpy_state(model: nn.Module, path: Path) -> None:
    """Save a strict runtime-style NumPy state dictionary."""
    state = {
        key.replace(".", "_"): value.detach().cpu().numpy()
        for key, value in model.state_dict().items()
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    np.save(path, state)


def write_json(path: Path, value: dict) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")
