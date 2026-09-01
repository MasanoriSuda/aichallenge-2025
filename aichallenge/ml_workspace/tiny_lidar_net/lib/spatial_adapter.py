"""Frozen TinyLidarNet spatial features with a bounded signed correction head."""

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

from lib.model import TinyLidarNet


SPATIAL_NORMALIZATION_MODES = ("layer_norm", "fixed_train_statistics")


class FrozenTinyLidarSpatialResidual(nn.Module):
    """Learn a correction without changing the admitted base representation.

    The complete conv5 spatial map is retained.  The admitted base network is
    part of the artifact for immutable provenance, but all of its parameters
    are frozen and feature extraction runs without an activation tape.
    """

    def __init__(
        self,
        input_dim: int = 750,
        hidden_dim: int = 128,
        max_scan_range_m: float = 30.0,
        max_abs_delta_rad: float = 1.2,
        use_speed: bool = False,
        max_speed_mps: float = 12.0,
        spatial_normalization: str = "layer_norm",
    ):
        super().__init__()
        if input_dim <= 0 or hidden_dim < 2:
            raise ValueError("spatial adapter dimensions must be positive")
        if not np.isfinite(max_scan_range_m) or max_scan_range_m <= 0.0:
            raise ValueError("maximum scan range must be finite and positive")
        if not np.isfinite(max_abs_delta_rad) or max_abs_delta_rad <= 0.0:
            raise ValueError("maximum correction must be finite and positive")
        if not np.isfinite(max_speed_mps) or max_speed_mps <= 0.0:
            raise ValueError("maximum speed must be finite and positive")
        if spatial_normalization not in SPATIAL_NORMALIZATION_MODES:
            raise ValueError(
                f"unsupported spatial normalization: {spatial_normalization}"
            )
        self.input_dim = int(input_dim)
        self.max_scan_range_m = float(max_scan_range_m)
        self.max_abs_delta_rad = float(max_abs_delta_rad)
        self.use_speed = bool(use_speed)
        self.max_speed_mps = float(max_speed_mps)
        self.spatial_normalization = spatial_normalization
        self.base = TinyLidarNet(input_dim=self.input_dim, output_dim=2)
        for parameter in self.base.parameters():
            parameter.requires_grad_(False)
        with torch.no_grad():
            dummy = torch.zeros(1, 1, self.input_dim)
            spatial = self._extract_conv5(dummy)
            self.spatial_dim = int(spatial.flatten(start_dim=1).shape[1])
        if self.spatial_normalization == "layer_norm":
            self.spatial_norm = nn.LayerNorm(self.spatial_dim)
        else:
            self.spatial_norm = nn.Identity()
            self.register_buffer("spatial_mean", torch.zeros(self.spatial_dim))
            self.register_buffer("spatial_scale", torch.ones(self.spatial_dim))
        adapter_input_dim = self.spatial_dim + int(self.use_speed)
        self.spatial_head = nn.Sequential(
            nn.Linear(adapter_input_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim // 2),
            nn.ReLU(),
        )
        self.direction_head = nn.Linear(hidden_dim // 2, 3)
        self.magnitude_head = nn.Linear(hidden_dim // 2, 2)
        self._initialize_adapter()

    def _initialize_adapter(self) -> None:
        for module in self.spatial_head:
            if isinstance(module, nn.Linear):
                nn.init.kaiming_normal_(
                    module.weight, mode="fan_out", nonlinearity="relu"
                )
                nn.init.zeros_(module.bias)
        if isinstance(self.spatial_norm, nn.LayerNorm):
            nn.init.ones_(self.spatial_norm.weight)
            nn.init.zeros_(self.spatial_norm.bias)
        # Equal left/right probabilities and magnitudes cancel exactly.  The
        # untrained composition therefore preserves candidate3 for every scan.
        nn.init.zeros_(self.direction_head.weight)
        nn.init.zeros_(self.direction_head.bias)
        nn.init.zeros_(self.magnitude_head.weight)
        nn.init.zeros_(self.magnitude_head.bias)

    def _extract_conv5(self, normalized_scans: torch.Tensor) -> torch.Tensor:
        values = F.relu(self.base.conv1(normalized_scans))
        values = F.relu(self.base.conv2(values))
        values = F.relu(self.base.conv3(values))
        values = F.relu(self.base.conv4(values))
        return F.relu(self.base.conv5(values))

    def spatial_features(self, scans_m: torch.Tensor) -> torch.Tensor:
        if scans_m.ndim != 2 or scans_m.shape[1] != self.input_dim:
            raise ValueError("spatial adapter scans must have shape (batch, input_dim)")
        if not torch.isfinite(scans_m).all():
            raise ValueError("spatial adapter scans must be finite")
        normalized = torch.clamp(
            scans_m / self.max_scan_range_m, 0.0, 1.0
        ).unsqueeze(1)
        with torch.no_grad():
            spatial = self._extract_conv5(normalized).flatten(start_dim=1)
        return spatial

    def set_spatial_statistics(
        self, mean: torch.Tensor, scale: torch.Tensor
    ) -> None:
        if self.spatial_normalization != "fixed_train_statistics":
            raise ValueError("fixed statistics require fixed normalization mode")
        if mean.shape != (self.spatial_dim,) or scale.shape != (self.spatial_dim,):
            raise ValueError("spatial statistics dimension mismatch")
        if not torch.isfinite(mean).all() or not torch.isfinite(scale).all():
            raise ValueError("spatial statistics must be finite")
        if torch.any(scale <= 0.0):
            raise ValueError("spatial feature scale must be positive")
        self.spatial_mean.copy_(mean.to(self.spatial_mean))
        self.spatial_scale.copy_(scale.to(self.spatial_scale))

    def normalized_spatial_features(self, scans_m: torch.Tensor) -> torch.Tensor:
        spatial = self.spatial_features(scans_m)
        if self.spatial_normalization == "fixed_train_statistics":
            return (spatial - self.spatial_mean) / self.spatial_scale
        return self.spatial_norm(spatial)

    def _adapter_features(
        self, scans_m: torch.Tensor, speeds_mps: torch.Tensor | None
    ) -> torch.Tensor:
        spatial = self.normalized_spatial_features(scans_m)
        if not self.use_speed:
            return spatial
        if speeds_mps is None or speeds_mps.shape != (len(scans_m),):
            raise ValueError("speed-enabled adapter requires one speed per scan")
        if not torch.isfinite(speeds_mps).all() or torch.any(speeds_mps < 0.0):
            raise ValueError("spatial adapter speed must be finite and non-negative")
        normalized_speed = torch.clamp(
            speeds_mps / self.max_speed_mps, 0.0, 1.5
        ).unsqueeze(1)
        return torch.cat((spatial, normalized_speed), dim=1)

    def forward_components(
        self,
        scans_m: torch.Tensor,
        speeds_mps: torch.Tensor | None = None,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        hidden = self.spatial_head(self._adapter_features(scans_m, speeds_mps))
        direction_logits = self.direction_head(hidden)
        direction_probabilities = torch.softmax(direction_logits, dim=-1)
        magnitudes = (
            torch.sigmoid(self.magnitude_head(hidden)) * self.max_abs_delta_rad
        )
        residual = (
            direction_probabilities[:, 2] * magnitudes[:, 1]
            - direction_probabilities[:, 0] * magnitudes[:, 0]
        )
        return residual, magnitudes, direction_logits, direction_probabilities

    def forward(
        self, scans_m: torch.Tensor, speeds_mps: torch.Tensor | None = None
    ) -> torch.Tensor:
        residual, _, _, _ = self.forward_components(scans_m, speeds_mps)
        return residual
