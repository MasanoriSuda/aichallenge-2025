from dataclasses import dataclass
import hashlib
import logging
from typing import Callable, Optional, Tuple

import numpy as np

from tiny_lidar_net_controller.gap_teacher import (
    GapTeacherConfig,
    GapTeacherDecision,
    LidarGapTeacher,
    LidarLongitudinalSafety,
    LidarPrecontactTeacher,
    LongitudinalSafetyDecision,
)
from tiny_lidar_net_controller.speed_committed_teacher import (
    LidarSpeedCommittedTeacher,
    SpeedCommittedTeacherConfig,
)
from tiny_lidar_net_controller.speed_governor import (
    ForwardSpeedGovernor,
    ForwardSpeedGovernorDecision,
)
from tiny_lidar_net_controller.longitudinal_safety import (
    LidarSpeedAwareLongitudinalSafety,
    SpeedAwareLongitudinalSafetyConfig,
)
from tiny_lidar_net_controller.recurrent_runtime import load_recurrent_runtime
from tiny_lidar_net_controller.model.tinylidarnet import (
    SpatialSteeringAdapterNp,
    SteeringResidualNetNp,
    TinyLidarNetNp,
    TinyLidarNetSmallNp,
)


@dataclass(frozen=True)
class RecurrentShadowSample:
    """Immutable input captured from one admitted production command."""

    conv5_features: np.ndarray
    speed_mps: float
    raw_base_steering_rad: float
    spatial_correction_rad: float
    production_spatial_steering_rad: float


@dataclass(frozen=True)
class RecurrentShadowEvaluation:
    """One recurrent diagnostic result and its successor hidden state."""

    correction_rad: float
    raw_correction_rad: float
    steering_rad: float
    hidden_norm: float
    next_hidden: np.ndarray


class TinyLidarNetCore:
    """Core logic for the TinyLidarNet autonomous driving controller.

    This class manages the neural network model lifecycle, including initialization,
    weight loading, input preprocessing (cleaning, resizing, normalizing), and
    inference execution. It is designed to be framework-agnostic.

    Attributes:
        input_dim (int): Dimension of the input vector expected by the model.
        output_dim (int): Dimension of the output vector (acceleration, steering).
        architecture (str): Model architecture type ('large' or 'small').
        acceleration (float): Fixed acceleration value used in 'fixed' control mode.
        control_mode (str): Control strategy ('ai', 'fixed', production
            'fixed_lidar_brake', or a teacher-only mode).
        max_range (float): Maximum LiDAR range used for normalization and clipping.
        model (object): The instantiated neural network model.
        logger (logging.Logger): Logger instance.
    """

    def __init__(
        self,
        input_dim: int = 1080,
        output_dim: int = 2,
        architecture: str = 'large',
        ckpt_path: str = '',
        acceleration: float = 0.1,
        maximum_forward_speed_mps: float = 0.0,
        control_mode: str = 'ai',
        max_range: float = 30.0,
        gap_teacher_config: Optional[GapTeacherConfig] = None,
        speed_committed_teacher_config: Optional[
            SpeedCommittedTeacherConfig
        ] = None,
        speed_aware_longitudinal_safety_config: Optional[
            SpeedAwareLongitudinalSafetyConfig
        ] = None,
        residual_ckpt_path: str = '',
        residual_max_abs_delta_rad: float = 1.28,
        residual_architecture: str = 'stateless',
        spatial_shadow_ckpt_path: str = '',
        spatial_shadow_expected_sha256: str = '',
        spatial_shadow_hidden_dim: int = 128,
        spatial_shadow_projection_dim: int = 128,
        spatial_shadow_use_speed: bool = True,
        spatial_shadow_use_base_steering: bool = False,
        spatial_shadow_max_speed_mps: float = 12.0,
        spatial_shadow_max_abs_delta_rad: float = 1.2,
        spatial_authority_enabled: bool = False,
        spatial_authority_max_abs_delta_rad: float = 0.12,
        recurrent_shadow_ckpt_path: str = '',
        recurrent_shadow_expected_sha256: str = '',
        recurrent_shadow_hidden_dim: int = 64,
        recurrent_shadow_projection_dim: int = 128,
        recurrent_shadow_use_speed: bool = False,
        recurrent_shadow_speed_embedding_dim: int = 16,
        recurrent_shadow_max_speed_mps: float = 12.0,
        recurrent_shadow_max_abs_correction_rad: float = 0.64,
        recurrent_shadow_correction_deadband_rad: float = 0.02,
        recurrent_authority_enabled: bool = False,
        recurrent_authority_max_abs_correction_rad: float = 0.24,
    ):
        """Initializes the TinyLidarNetCore with specified parameters.

        Args:
            input_dim (int, optional): The number of LiDAR points expected by the model.
                Defaults to 1080.
            output_dim (int, optional): The number of output control values.
                Defaults to 2.
            architecture (str, optional): The model architecture to use ('large' or 'small').
                Defaults to 'large'.
            ckpt_path (str, optional): Path to the numpy weight file (.npy or .npz).
                Defaults to ''.
            acceleration (float, optional): The constant acceleration value to apply
                when control_mode is set to 'fixed'. Defaults to 0.1.
            control_mode (str, optional): The control mode to determine output behavior.
                'ai' uses model output for both acceleration and steering.
                'fixed' uses the fixed acceleration value and model output for steering.
                'gap_teacher' blends model steering toward a LiDAR opening for
                teacher-data collection only.
                Defaults to 'ai'.
            max_range (float, optional): The maximum range value for normalization.
                Values exceeding this will be clipped, and infinity will be replaced
                by this value. Defaults to 30.0.
        """
        self.input_dim = input_dim
        self.output_dim = output_dim
        self.architecture = architecture
        self.acceleration = acceleration
        self.maximum_forward_speed_mps = float(maximum_forward_speed_mps)
        self.control_mode = control_mode.lower()
        self.max_range = max_range
        self.logger = logging.getLogger(__name__)
        self.loaded_parameter_count = 0
        self.residual_loaded_parameter_count = 0
        self.residual_model = None
        self.last_residual_correction_rad = 0.0
        self.last_residual_gate_probability = 0.0
        self.residual_architecture = residual_architecture.lower()
        self.previous_residual_scan: Optional[np.ndarray] = None
        self.last_gap_teacher_decision: Optional[GapTeacherDecision] = None
        self.last_longitudinal_safety_decision: Optional[
            LongitudinalSafetyDecision
        ] = None
        self.last_speed_governor_decision: Optional[
            ForwardSpeedGovernorDecision
        ] = None
        self.spatial_shadow_model = None
        self.spatial_shadow_loaded_parameter_count = 0
        self.last_spatial_shadow_correction_rad = 0.0
        self.last_spatial_shadow_direction_probabilities = np.zeros(
            3, dtype=np.float32
        )
        self.last_spatial_shadow_admitted = False
        self.last_spatial_shadow_status = "disabled"
        self.spatial_authority_enabled = bool(spatial_authority_enabled)
        self.spatial_authority_max_abs_delta_rad = float(
            spatial_authority_max_abs_delta_rad
        )
        self.last_spatial_authority_correction_rad = 0.0
        self.last_spatial_authority_applied = False
        self.last_spatial_authority_clipped = False
        self.recurrent_shadow_model = None
        self.recurrent_shadow_loaded_parameter_count = 0
        self.recurrent_shadow_artifact_contract = "disabled"
        self.recurrent_shadow_runtime_config = None
        self.recurrent_shadow_hidden = None
        self.last_recurrent_shadow_correction_rad = 0.0
        self.last_recurrent_shadow_raw_correction_rad = 0.0
        self.last_recurrent_shadow_steering_rad = 0.0
        self.last_recurrent_shadow_hidden_norm = 0.0
        self.last_recurrent_shadow_admitted = False
        self.last_recurrent_shadow_status = "disabled"
        self.last_recurrent_shadow_sample = None
        self.last_recurrent_shadow_sample_status = "disabled"
        self.recurrent_shadow_reset_count = 0
        self.recurrent_authority_enabled = bool(recurrent_authority_enabled)
        self.recurrent_authority_max_abs_correction_rad = float(
            recurrent_authority_max_abs_correction_rad
        )
        self.last_recurrent_authority_correction_rad = 0.0
        self.last_recurrent_authority_applied = False
        self.last_recurrent_authority_clipped = False
        self._recurrent_authority_evaluator: Optional[
            Callable[
                [RecurrentShadowSample, Optional[np.ndarray]],
                RecurrentShadowEvaluation,
            ]
        ] = None

        if not isinstance(self.input_dim, int) or self.input_dim <= 0:
            raise ValueError("input_dim must be a positive integer")
        if self.output_dim != 2:
            raise ValueError("output_dim must be 2 with [acceleration, steering] semantics")
        if self.architecture not in {"normal", "large", "small"}:
            raise ValueError(
                "architecture must be one of: normal, large, small"
            )
        if self.control_mode not in {
            "ai",
            "fixed",
            "fixed_lidar_brake",
            "speed_aware_lidar_brake",
            "gap_teacher",
            "precontact_teacher",
            "speed_committed_teacher",
        }:
            raise ValueError(
                "control_mode must be one of: ai, fixed, fixed_lidar_brake, "
                "speed_aware_lidar_brake, "
                "gap_teacher, precontact_teacher, speed_committed_teacher"
            )
        if not np.isfinite(self.max_range) or self.max_range <= 0.0:
            raise ValueError("max_range must be finite and positive")
        if not np.isfinite(self.acceleration) or not -1.0 <= self.acceleration <= 1.0:
            raise ValueError("acceleration must be finite and within [-1.0, 1.0]")
        if (
            not np.isfinite(self.maximum_forward_speed_mps)
            or self.maximum_forward_speed_mps < 0.0
        ):
            raise ValueError(
                "maximum_forward_speed_mps must be finite and non-negative"
            )
        if self.maximum_forward_speed_mps > 0.0 and self.control_mode not in {
            "fixed",
            "fixed_lidar_brake",
            "speed_aware_lidar_brake",
        }:
            raise ValueError(
                "maximum_forward_speed_mps is only supported by fixed control modes"
            )
        if (
            not np.isfinite(residual_max_abs_delta_rad)
            or residual_max_abs_delta_rad <= 0.0
        ):
            raise ValueError(
                "residual_max_abs_delta_rad must be finite and positive"
            )
        if self.residual_architecture not in {"stateless", "scan_delta"}:
            raise ValueError(
                "residual_architecture must be one of: stateless, scan_delta"
            )
        if (
            not isinstance(spatial_shadow_hidden_dim, int)
            or spatial_shadow_hidden_dim < 2
            or not isinstance(spatial_shadow_projection_dim, int)
            or spatial_shadow_projection_dim <= 0
        ):
            raise ValueError("spatial shadow dimensions must be positive integers")
        if (
            not np.isfinite(spatial_shadow_max_speed_mps)
            or spatial_shadow_max_speed_mps <= 0.0
            or not np.isfinite(spatial_shadow_max_abs_delta_rad)
            or spatial_shadow_max_abs_delta_rad <= 0.0
        ):
            raise ValueError("spatial shadow bounds must be finite and positive")
        if (
            not np.isfinite(self.spatial_authority_max_abs_delta_rad)
            or self.spatial_authority_max_abs_delta_rad <= 0.0
            or self.spatial_authority_max_abs_delta_rad
            > spatial_shadow_max_abs_delta_rad
        ):
            raise ValueError(
                "spatial authority bound must be finite, positive and no "
                "larger than the model correction bound"
            )
        if self.spatial_authority_enabled and not spatial_shadow_ckpt_path:
            raise ValueError(
                "spatial authority requires an explicit spatial shadow checkpoint"
            )
        if self.spatial_authority_enabled and residual_ckpt_path:
            raise ValueError(
                "legacy residual and spatial authority cannot own steering together"
            )
        if spatial_shadow_expected_sha256 and not spatial_shadow_ckpt_path:
            raise ValueError(
                "spatial shadow expected SHA256 requires a spatial shadow checkpoint"
            )
        if (
            not isinstance(recurrent_shadow_hidden_dim, int)
            or recurrent_shadow_hidden_dim <= 0
            or not isinstance(recurrent_shadow_projection_dim, int)
            or recurrent_shadow_projection_dim <= 0
            or not isinstance(recurrent_shadow_speed_embedding_dim, int)
            or recurrent_shadow_speed_embedding_dim <= 0
        ):
            raise ValueError("recurrent shadow dimensions must be positive integers")
        if (
            not np.isfinite(recurrent_shadow_max_speed_mps)
            or recurrent_shadow_max_speed_mps <= 0.0
            or not np.isfinite(recurrent_shadow_max_abs_correction_rad)
            or recurrent_shadow_max_abs_correction_rad <= 0.0
            or not np.isfinite(recurrent_shadow_correction_deadband_rad)
            or recurrent_shadow_correction_deadband_rad < 0.0
            or recurrent_shadow_correction_deadband_rad
            > recurrent_shadow_max_abs_correction_rad
        ):
            raise ValueError("recurrent shadow scales or deadband are invalid")
        if recurrent_shadow_expected_sha256 and not recurrent_shadow_ckpt_path:
            raise ValueError(
                "recurrent shadow expected SHA256 requires a recurrent checkpoint"
            )
        if recurrent_shadow_ckpt_path and not spatial_shadow_ckpt_path:
            raise ValueError(
                "recurrent shadow requires the frozen production spatial checkpoint"
            )
        if recurrent_shadow_ckpt_path and not self.spatial_authority_enabled:
            raise ValueError(
                "recurrent shadow requires the packaged spatial production authority"
            )
        if (
            not np.isfinite(self.recurrent_authority_max_abs_correction_rad)
            or self.recurrent_authority_max_abs_correction_rad <= 0.0
            or self.recurrent_authority_max_abs_correction_rad
            > recurrent_shadow_max_abs_correction_rad
        ):
            raise ValueError(
                "recurrent authority bound must be finite, positive and no "
                "larger than the model correction bound"
            )
        if self.recurrent_authority_enabled and not recurrent_shadow_ckpt_path:
            raise ValueError(
                "recurrent authority requires an explicit recurrent checkpoint"
            )
        if self.recurrent_authority_enabled and not recurrent_shadow_expected_sha256:
            raise ValueError(
                "recurrent authority requires an explicit recurrent checkpoint "
                "SHA256"
            )

        if self.architecture == 'small':
            self.model = TinyLidarNetSmallNp(input_dim=self.input_dim, output_dim=self.output_dim)
        else:
            self.model = TinyLidarNetNp(input_dim=self.input_dim, output_dim=self.output_dim)

        self.gap_teacher = None
        self.gap_teacher_requires_speed = False
        self.longitudinal_safety = None
        self.longitudinal_safety_requires_speed = False
        self.speed_governor = (
            ForwardSpeedGovernor(self.maximum_forward_speed_mps)
            if self.maximum_forward_speed_mps > 0.0
            else None
        )
        if self.control_mode == "gap_teacher":
            self.gap_teacher = LidarGapTeacher(
                gap_teacher_config or GapTeacherConfig()
            )
        elif self.control_mode == "precontact_teacher":
            self.gap_teacher = LidarPrecontactTeacher(
                gap_teacher_config or GapTeacherConfig()
            )
        elif self.control_mode == "speed_committed_teacher":
            self.gap_teacher = LidarSpeedCommittedTeacher(
                gap_teacher_config or GapTeacherConfig(),
                speed_committed_teacher_config,
            )
            self.gap_teacher_requires_speed = True
        elif self.control_mode == "fixed_lidar_brake":
            self.longitudinal_safety = LidarLongitudinalSafety(
                gap_teacher_config or GapTeacherConfig()
            )
        elif self.control_mode == "speed_aware_lidar_brake":
            self.longitudinal_safety = LidarSpeedAwareLongitudinalSafety(
                gap_teacher_config or GapTeacherConfig(),
                speed_aware_longitudinal_safety_config,
            )
            self.longitudinal_safety_requires_speed = True

        if not ckpt_path:
            raise ValueError("ckpt_path is required; random production weights are forbidden")
        self._load_weights(ckpt_path)
        if residual_ckpt_path:
            self.residual_model = SteeringResidualNetNp(
                input_dim=self.input_dim,
                max_abs_delta_rad=residual_max_abs_delta_rad,
                input_channels=(
                    1 if self.residual_architecture == "stateless" else 2
                ),
            )
            self.residual_loaded_parameter_count = self._load_model_weights(
                self.residual_model,
                residual_ckpt_path,
                "steering residual",
            )

        if spatial_shadow_ckpt_path:
            self._verify_file_sha256(
                spatial_shadow_ckpt_path,
                spatial_shadow_expected_sha256,
                "spatial steering shadow",
            )
            self.spatial_shadow_model = SpatialSteeringAdapterNp(
                input_dim=self.input_dim,
                hidden_dim=spatial_shadow_hidden_dim,
                projection_dim=spatial_shadow_projection_dim,
                use_speed=spatial_shadow_use_speed,
                use_base_steering=spatial_shadow_use_base_steering,
                max_speed_mps=spatial_shadow_max_speed_mps,
                max_abs_delta_rad=spatial_shadow_max_abs_delta_rad,
            )
            self.spatial_shadow_loaded_parameter_count = self._load_model_weights(
                self.spatial_shadow_model,
                spatial_shadow_ckpt_path,
                "spatial steering shadow",
            )
            embedded_base = self.spatial_shadow_model.embedded_base_parameters()
            for key, production_value in self.model.params.items():
                embedded_value = embedded_base.get(key)
                if embedded_value is None or not np.array_equal(
                    embedded_value, production_value
                ):
                    raise ValueError(
                        "spatial shadow embedded base does not match production "
                        f"parameter: {key}"
                    )
            self.last_spatial_shadow_status = "waiting-speed"

        if recurrent_shadow_ckpt_path:
            configured_recurrent = {
                "input_dim": self.input_dim,
                "hidden_dim": recurrent_shadow_hidden_dim,
                "projection_dim": recurrent_shadow_projection_dim,
                "use_speed": recurrent_shadow_use_speed,
                "speed_embedding_dim": recurrent_shadow_speed_embedding_dim,
                "max_speed_mps": recurrent_shadow_max_speed_mps,
                "max_abs_correction_rad": (
                    recurrent_shadow_max_abs_correction_rad
                ),
                "max_abs_steering_rad": 1.0,
                "correction_deadband_rad": (
                    recurrent_shadow_correction_deadband_rad
                ),
                "spatial_baseline_hidden_dim": spatial_shadow_hidden_dim,
                "spatial_baseline_projection_dim": (
                    spatial_shadow_projection_dim
                ),
                "spatial_baseline_max_speed_mps": (
                    spatial_shadow_max_speed_mps
                ),
                "spatial_baseline_max_abs_delta_rad": (
                    spatial_shadow_max_abs_delta_rad
                ),
            }
            loaded_recurrent = load_recurrent_runtime(
                recurrent_shadow_ckpt_path,
                recurrent_shadow_expected_sha256,
                legacy_runtime_config=configured_recurrent,
            )
            recurrent_config = loaded_recurrent.runtime_config
            if recurrent_config["input_dim"] != self.input_dim:
                raise ValueError(
                    "recurrent artifact input dimension does not match production: "
                    f"artifact={recurrent_config['input_dim']}, "
                    f"production={self.input_dim}"
                )
            if recurrent_config["max_abs_steering_rad"] != 1.0:
                raise ValueError(
                    "recurrent artifact steering scale must match the runtime "
                    "command contract"
                )
            self.recurrent_shadow_runtime_config = dict(recurrent_config)
            self.recurrent_shadow_artifact_contract = (
                loaded_recurrent.artifact_contract
            )
            self.recurrent_shadow_model = loaded_recurrent.model
            self.recurrent_shadow_loaded_parameter_count = (
                loaded_recurrent.loaded_parameter_count
            )
            for key, production_value in self.model.params.items():
                recurrent_value = (
                    self.recurrent_shadow_model.embedded_base_parameters().get(key)
                )
                if recurrent_value is None or not np.array_equal(
                    recurrent_value, production_value
                ):
                    raise ValueError(
                        "recurrent shadow embedded base does not match production "
                        f"parameter: {key}"
                    )
            recurrent_spatial = (
                self.recurrent_shadow_model.embedded_spatial_parameters()
            )
            for key, production_value in self.spatial_shadow_model.params.items():
                recurrent_value = recurrent_spatial.get(key)
                if recurrent_value is None or not np.array_equal(
                    recurrent_value, production_value
                ):
                    raise ValueError(
                        "recurrent shadow embedded spatial baseline does not "
                        f"match production parameter: {key}"
                    )
            self.last_recurrent_shadow_status = "waiting-speed"

        self.requires_wheel_speed = bool(
            self.spatial_shadow_model is not None
            or self.recurrent_shadow_model is not None
            or self.gap_teacher_requires_speed
            or self.longitudinal_safety_requires_speed
            or self.speed_governor is not None
        )

    @staticmethod
    def _verify_file_sha256(path: str, expected_sha256: str, label: str) -> None:
        """Reject a promoted artifact when its immutable identity changed."""
        expected = expected_sha256.strip().lower()
        if not expected:
            return
        if len(expected) != 64 or any(c not in "0123456789abcdef" for c in expected):
            raise ValueError(f"{label} expected SHA256 must be 64 hexadecimal characters")
        digest = hashlib.sha256()
        with open(path, "rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
        actual = digest.hexdigest()
        if actual != expected:
            raise ValueError(
                f"{label} SHA256 mismatch: expected {expected}, got {actual}"
            )

    def bind_recurrent_authority_evaluator(
        self,
        evaluator: Callable[
            [RecurrentShadowSample, Optional[np.ndarray]],
            RecurrentShadowEvaluation,
        ],
    ) -> None:
        """Bind the sole current-sample evaluator used by recurrent authority.

        Observation may still use the local model or the asynchronous shadow
        executor.  Steering authority is intentionally different: production
        must bind its resource-isolated evaluator before the first callback,
        so the historical in-process authority route cannot be selected by an
        experiment flag alone.
        """
        if not self.recurrent_authority_enabled:
            raise ValueError(
                "recurrent authority evaluator requires enabled authority"
            )
        if not callable(evaluator):
            raise TypeError("recurrent authority evaluator must be callable")
        if self._recurrent_authority_evaluator is not None:
            raise ValueError("recurrent authority evaluator is already bound")
        self._recurrent_authority_evaluator = evaluator

    def process(
        self,
        ranges: np.ndarray,
        speed_mps: Optional[float] = None,
        *,
        defer_recurrent_shadow: bool = False,
    ) -> Tuple[float, float]:
        """Runs the complete inference pipeline on raw LiDAR data.

        This method handles data cleaning (NaN/Inf removal), resizing, normalization,
        and model inference.

        Args:
            ranges (np.ndarray): A 1D numpy array containing raw LiDAR range data.

        Returns:
            Tuple[float, float]: A tuple containing (acceleration, steering_angle).
                Values are clipped between -1.0 and 1.0.
        """
        if defer_recurrent_shadow and self.recurrent_authority_enabled:
            raise ValueError(
                "recurrent authority cannot use deferred shadow execution"
            )

        ranges = np.asarray(ranges, dtype=np.float32)
        if ranges.ndim != 1 or ranges.size == 0:
            raise ValueError("ranges must be a non-empty 1D array")

        # 1. Preprocess (Clean -> Resize -> Normalize). Keep metre ranges for
        # the explicitly teacher-only gap policy; the network sees normalized data.
        physical_ranges = self._clean_and_resize_ranges(ranges)
        processed_ranges = physical_ranges / self.max_range

        # Prepare input tensor: (1, 1, input_dim)
        x = np.expand_dims(np.expand_dims(processed_ranges, axis=0), axis=1)

        # 2. Inference.  The production Conv5 backbone is the canonical
        # feature owner.  Optional adapters embed byte-identical copies that
        # were verified at load time, so reuse the canonical features instead
        # of evaluating the same CNN several times in the control callback.
        shared_conv5 = None
        if isinstance(self.model, TinyLidarNetNp):
            shared_conv5 = self.model.forward_conv5_features(x)
            model_outputs = self.model.forward_from_conv5_features(shared_conv5)
        else:
            model_outputs = self.model(x)
        outputs = np.asarray(model_outputs[0], dtype=np.float32)
        if outputs.shape != (2,):
            raise ValueError(f"model output must have shape (2,), got {outputs.shape}")
        if not np.all(np.isfinite(outputs)):
            raise ValueError("model output contains non-finite values")

        # 3. Post-process
        if self.control_mode == "ai":
            accel = float(np.clip(outputs[0], -1.0, 1.0))
        else:
            accel = self.acceleration

        steer = float(np.clip(outputs[1], -1.0, 1.0))
        raw_base_steer = steer

        self.last_spatial_shadow_admitted = False
        self.last_spatial_shadow_correction_rad = 0.0
        self.last_spatial_shadow_direction_probabilities.fill(0.0)
        self.last_spatial_authority_correction_rad = 0.0
        self.last_spatial_authority_applied = False
        self.last_spatial_authority_clipped = False
        self.last_recurrent_shadow_sample = None
        self.last_recurrent_shadow_sample_status = "disabled"
        if not defer_recurrent_shadow:
            self.last_recurrent_shadow_correction_rad = 0.0
            self.last_recurrent_shadow_raw_correction_rad = 0.0
            self.last_recurrent_shadow_steering_rad = 0.0
            self.last_recurrent_shadow_admitted = False
        self.last_recurrent_authority_correction_rad = 0.0
        self.last_recurrent_authority_applied = False
        self.last_recurrent_authority_clipped = False
        if self.spatial_shadow_model is not None:
            if speed_mps is None:
                self.last_spatial_shadow_status = "missing-or-stale-speed"
            else:
                try:
                    speed = float(speed_mps)
                    if not np.isfinite(speed) or speed < 0.0:
                        raise ValueError(
                            "spatial shadow speed must be finite and non-negative"
                        )
                    (
                        residual,
                        _,
                        _,
                        direction_probabilities,
                    ) = self.spatial_shadow_model.forward_components_from_spatial_features(
                        shared_conv5,
                        np.asarray([speed], dtype=np.float32),
                        np.asarray([raw_base_steer], dtype=np.float32),
                    )
                    self.last_spatial_shadow_correction_rad = float(residual[0])
                    self.last_spatial_shadow_direction_probabilities = (
                        direction_probabilities[0].astype(np.float32, copy=True)
                    )
                    self.last_spatial_shadow_admitted = True
                    self.last_spatial_shadow_status = "ok"
                    if self.spatial_authority_enabled:
                        applied = float(np.clip(
                            self.last_spatial_shadow_correction_rad,
                            -self.spatial_authority_max_abs_delta_rad,
                            self.spatial_authority_max_abs_delta_rad,
                        ))
                        self.last_spatial_authority_correction_rad = applied
                        self.last_spatial_authority_applied = True
                        self.last_spatial_authority_clipped = not np.isclose(
                            applied,
                            self.last_spatial_shadow_correction_rad,
                            rtol=0.0,
                            atol=1e-7,
                        )
                        steer = float(np.clip(steer + applied, -1.0, 1.0))
                except Exception as exc:
                    # Shadow execution is diagnostic and must never override a
                    # valid production command.
                    self.last_spatial_shadow_status = (
                        f"inference-error:{type(exc).__name__}:{exc}"
                    )

        if self.recurrent_shadow_model is not None:
            try:
                sample = self._make_recurrent_shadow_sample(
                    shared_conv5=shared_conv5,
                    speed_mps=speed_mps,
                    raw_base_steer=raw_base_steer,
                    production_spatial_steer=steer,
                )
                self.last_recurrent_shadow_sample = sample
                self.last_recurrent_shadow_sample_status = "ok"
                if not defer_recurrent_shadow:
                    evaluator = self.evaluate_recurrent_shadow_sample
                    if self.recurrent_authority_enabled:
                        evaluator = self._recurrent_authority_evaluator
                        if evaluator is None:
                            raise RuntimeError(
                                "recurrent authority evaluator is not bound"
                            )
                    evaluation = evaluator(
                        sample, self.recurrent_shadow_hidden
                    )
                    self.record_recurrent_shadow_evaluation(
                        evaluation, retain_hidden=True
                    )
                    if self.recurrent_authority_enabled:
                        applied = float(np.clip(
                            self.last_recurrent_shadow_correction_rad,
                            -self.recurrent_authority_max_abs_correction_rad,
                            self.recurrent_authority_max_abs_correction_rad,
                        ))
                        self.last_recurrent_authority_correction_rad = applied
                        self.last_recurrent_authority_applied = True
                        self.last_recurrent_authority_clipped = not np.isclose(
                            applied,
                            self.last_recurrent_shadow_correction_rad,
                            rtol=0.0,
                            atol=1e-7,
                        )
                        steer = float(np.clip(steer + applied, -1.0, 1.0))
            except Exception as exc:
                status = f"inference-error:{type(exc).__name__}:{exc}"
                if speed_mps is None:
                    status = "missing-or-stale-speed"
                self.last_recurrent_shadow_sample_status = status
                if not defer_recurrent_shadow:
                    self.last_recurrent_shadow_status = status
                    self.reset_recurrent_history()

        self.last_residual_correction_rad = 0.0
        self.last_residual_gate_probability = 0.0
        if self.residual_model is not None:
            residual_input = self._compose_residual_input(processed_ranges)
            residual, _, gate_logit = self.residual_model.forward_components(
                residual_input
            )
            residual_value = float(residual[0, 0])
            gate_value = float(
                1.0 / (1.0 + np.exp(-np.clip(gate_logit[0, 0], -60.0, 60.0)))
            )
            if not np.isfinite(residual_value) or not np.isfinite(gate_value):
                raise ValueError("residual model output contains non-finite values")
            self.last_residual_correction_rad = residual_value
            self.last_residual_gate_probability = gate_value
            steer = float(np.clip(steer + residual_value, -1.0, 1.0))

        self.last_gap_teacher_decision = None
        self.last_longitudinal_safety_decision = None
        self.last_speed_governor_decision = None
        if self.speed_governor is not None:
            governor_decision = self.speed_governor.decide(speed_mps, accel)
            self.last_speed_governor_decision = governor_decision
            accel = governor_decision.acceleration_mps2
        if self.gap_teacher is not None:
            if self.gap_teacher_requires_speed:
                decision = self.gap_teacher.decide(
                    physical_ranges, steer, accel, speed_mps
                )
            else:
                decision = self.gap_teacher.decide(physical_ranges, steer, accel)
            self.last_gap_teacher_decision = decision
            accel = decision.acceleration_mps2
            steer = decision.steering_rad
        elif self.longitudinal_safety is not None:
            if self.longitudinal_safety_requires_speed:
                safety_decision = self.longitudinal_safety.decide(
                    physical_ranges, accel, speed_mps
                )
            else:
                safety_decision = self.longitudinal_safety.decide(
                    physical_ranges, accel
                )
            self.last_longitudinal_safety_decision = safety_decision
            accel = safety_decision.acceleration_mps2

        return accel, steer

    def _make_recurrent_shadow_sample(
        self,
        *,
        shared_conv5: np.ndarray,
        speed_mps: Optional[float],
        raw_base_steer: float,
        production_spatial_steer: float,
    ) -> RecurrentShadowSample:
        """Capture recurrent input only after the production baseline is valid."""
        if speed_mps is None:
            raise ValueError("recurrent shadow speed is missing or stale")
        speed = float(speed_mps)
        if not np.isfinite(speed) or speed < 0.0:
            raise ValueError(
                "recurrent shadow speed must be finite and non-negative"
            )
        if not self.last_spatial_shadow_admitted:
            raise ValueError(
                "recurrent shadow requires an admitted production spatial result"
            )
        expected_production_base = float(np.clip(
            raw_base_steer + self.last_spatial_shadow_correction_rad,
            -1.0,
            1.0,
        ))
        if not np.isclose(
            expected_production_base,
            production_spatial_steer,
            rtol=2e-5,
            atol=2e-5,
        ):
            raise ValueError(
                "recurrent shadow production baseline mismatch: "
                f"embedded={expected_production_base:.8f}, "
                f"published_base={production_spatial_steer:.8f}"
            )
        conv5 = np.array(shared_conv5, dtype=np.float32, copy=True)
        if conv5.ndim != 2 or not np.all(np.isfinite(conv5)):
            raise ValueError("recurrent shadow Conv5 snapshot is invalid")
        conv5.setflags(write=False)
        return RecurrentShadowSample(
            conv5_features=conv5,
            speed_mps=speed,
            raw_base_steering_rad=float(raw_base_steer),
            spatial_correction_rad=float(
                self.last_spatial_shadow_correction_rad
            ),
            production_spatial_steering_rad=float(production_spatial_steer),
        )

    def evaluate_recurrent_shadow_sample(
        self,
        sample: RecurrentShadowSample,
        hidden: Optional[np.ndarray],
    ) -> RecurrentShadowEvaluation:
        """Evaluate one immutable sample without mutating production state."""
        if self.recurrent_shadow_model is None:
            raise ValueError("recurrent shadow model is disabled")
        correction, raw_correction, next_hidden = (
            self.recurrent_shadow_model.forward_correction_from_conv5_features(
                sample.conv5_features,
                np.asarray([sample.speed_mps], dtype=np.float32),
                hidden,
            )
        )
        recurrent_steering = np.clip(
            sample.production_spatial_steering_rad + correction,
            -1.0,
            1.0,
        )
        next_hidden_copy = np.array(next_hidden, dtype=np.float32, copy=True)
        return RecurrentShadowEvaluation(
            correction_rad=float(correction[0]),
            raw_correction_rad=float(raw_correction[0]),
            steering_rad=float(recurrent_steering[0]),
            hidden_norm=float(np.linalg.norm(next_hidden_copy)),
            next_hidden=next_hidden_copy,
        )

    def record_recurrent_shadow_evaluation(
        self,
        evaluation: RecurrentShadowEvaluation,
        *,
        retain_hidden: bool,
    ) -> None:
        """Record a completed evaluation; async callers keep hidden ownership."""
        if retain_hidden:
            self.recurrent_shadow_hidden = evaluation.next_hidden.copy()
        self.last_recurrent_shadow_hidden_norm = evaluation.hidden_norm
        self.last_recurrent_shadow_correction_rad = evaluation.correction_rad
        self.last_recurrent_shadow_raw_correction_rad = (
            evaluation.raw_correction_rad
        )
        self.last_recurrent_shadow_steering_rad = evaluation.steering_rad
        self.last_recurrent_shadow_admitted = True
        self.last_recurrent_shadow_status = "ok"

    def reset_residual_history(self) -> None:
        """Prevent temporal context from crossing a runtime reset boundary."""
        self.previous_residual_scan = None
        self.reset_recurrent_history()
        if self.gap_teacher is not None and hasattr(self.gap_teacher, "reset"):
            self.gap_teacher.reset()

    def reset_recurrent_history(self) -> None:
        """Drop recurrent state without changing the production controller."""
        if self.recurrent_shadow_hidden is not None:
            self.recurrent_shadow_reset_count += 1
        self.recurrent_shadow_hidden = None
        self.last_recurrent_shadow_hidden_norm = 0.0

    def _compose_residual_input(self, processed_ranges: np.ndarray) -> np.ndarray:
        current = np.asarray(processed_ranges, dtype=np.float32)
        if self.residual_architecture == "stateless":
            model_input = current[np.newaxis, np.newaxis, :]
        else:
            previous = (
                current
                if self.previous_residual_scan is None
                else self.previous_residual_scan
            )
            model_input = np.stack((current, current - previous))[np.newaxis, :, :]
        self.previous_residual_scan = current.copy()
        return model_input.astype(np.float32, copy=False)

    def _load_weights(self, path: str) -> None:
        """Loads model weights from a file into the model parameters.

        Args:
            path (str): Path to the .npy or .npz weight file.

        Raises:
            ValueError: If the weight file format is unsupported.
            IOError: If the file cannot be read.
        """
        self.loaded_parameter_count = self._load_model_weights(
            self.model, path, "base TinyLidarNet"
        )

    @staticmethod
    def _read_normalized_weights(path: str) -> dict:
        weights = np.load(path, allow_pickle=True)
        if isinstance(weights, np.lib.npyio.NpzFile):
            weight_dict = dict(weights.items())
            weights.close()
        elif isinstance(weights, np.ndarray) and weights.dtype == object:
            weight_dict = weights.item()
        elif isinstance(weights, dict):
            weight_dict = weights
        else:
            raise ValueError(f"Unsupported weight format type: {type(weights)}")

        normalized_weights = {}
        for key, value in weight_dict.items():
            key_norm = key.replace('.', '_')
            if key_norm in normalized_weights:
                raise ValueError(f"duplicate normalized parameter key: {key_norm}")
            normalized_weights[key_norm] = np.asarray(value)
        return normalized_weights

    def _load_model_weights(
        self, model, path: str, label: str, normalized_weights=None
    ) -> int:
        """Strictly load one NumPy model without accepting partial parameters."""
        try:
            if normalized_weights is None:
                normalized_weights = self._read_normalized_weights(path)

            expected_keys = set(model.params)
            provided_keys = set(normalized_weights)
            missing = sorted(expected_keys - provided_keys)
            unexpected = sorted(provided_keys - expected_keys)
            if missing or unexpected:
                raise ValueError(
                    f"weight key mismatch: missing={missing}, unexpected={unexpected}"
                )

            validated_weights = {}
            for key, expected in model.params.items():
                value = normalized_weights[key]
                if value.shape != expected.shape:
                    raise ValueError(
                        f"weight shape mismatch for {key}: "
                        f"expected={expected.shape}, actual={value.shape}"
                    )
                if not np.issubdtype(value.dtype, np.number):
                    raise ValueError(f"weight {key} must be numeric, got {value.dtype}")
                value = value.astype(np.float32, copy=False)
                if not np.all(np.isfinite(value)):
                    raise ValueError(f"weight {key} contains non-finite values")
                validated_weights[key] = value

            model.params.update(validated_weights)
            self.logger.info(
                "Successfully loaded %d validated %s parameters from %s",
                len(validated_weights),
                label,
                path,
            )
            return len(validated_weights)

        except Exception as e:
            self.logger.error(f"Failed to load weights from {path}: {e}")
            raise e

    def _clean_and_resize_ranges(self, ranges: np.ndarray) -> np.ndarray:
        """Clean and resize LiDAR ranges while preserving metres.

        This method performs the following operations:
        1. Replaces NaNs with 0.0.
        2. Replaces infinite values with `self.max_range`.
        3. Clips all values to the range [0.0, `self.max_range`].
        4. Resizes the array to match `self.input_dim` via interpolation or padding.

        Args:
            ranges (np.ndarray): Source LiDAR range data.

        Returns:
            np.ndarray: Physical ranges in metres with shape (self.input_dim,).
        """
        # Work on a copy to avoid side effects on the input array
        ranges = ranges.copy()
        
        # Handle invalid values
        ranges[np.isnan(ranges)] = 0.0
        ranges[np.isinf(ranges)] = self.max_range
        
        # Clip to ensure data is within the expected range
        ranges = np.clip(ranges, 0.0, self.max_range)

        # Resize input if necessary
        current_len = len(ranges)
        if current_len > self.input_dim:
            idx = np.linspace(0, current_len - 1, self.input_dim, dtype=int)
            ranges = ranges[idx]
        elif current_len < self.input_dim:
            ranges = np.pad(ranges, (0, self.input_dim - current_len), 'constant')

        return ranges.astype(np.float32, copy=False)

    def _preprocess_ranges(self, ranges: np.ndarray) -> np.ndarray:
        """Backward-compatible normalized preprocessing entry point."""
        return self._clean_and_resize_ranges(ranges) / self.max_range
