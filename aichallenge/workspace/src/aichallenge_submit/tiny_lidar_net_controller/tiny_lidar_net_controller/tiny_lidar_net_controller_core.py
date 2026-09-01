import logging
import numpy as np
from typing import Optional, Tuple

from tiny_lidar_net_controller.gap_teacher import (
    GapTeacherConfig,
    GapTeacherDecision,
    LidarGapTeacher,
    LidarLongitudinalSafety,
    LidarPrecontactTeacher,
    LongitudinalSafetyDecision,
)
from tiny_lidar_net_controller.model.tinylidarnet import (
    SpatialSteeringAdapterNp,
    SteeringResidualNetNp,
    TinyLidarNetNp,
    TinyLidarNetSmallNp,
)


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
        control_mode: str = 'ai',
        max_range: float = 30.0,
        gap_teacher_config: Optional[GapTeacherConfig] = None,
        residual_ckpt_path: str = '',
        residual_max_abs_delta_rad: float = 1.28,
        residual_architecture: str = 'stateless',
        spatial_shadow_ckpt_path: str = '',
        spatial_shadow_hidden_dim: int = 128,
        spatial_shadow_projection_dim: int = 128,
        spatial_shadow_use_speed: bool = True,
        spatial_shadow_max_speed_mps: float = 12.0,
        spatial_shadow_max_abs_delta_rad: float = 1.2,
        spatial_authority_enabled: bool = False,
        spatial_authority_max_abs_delta_rad: float = 0.12,
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
            "gap_teacher",
            "precontact_teacher",
        }:
            raise ValueError(
                "control_mode must be one of: ai, fixed, fixed_lidar_brake, "
                "gap_teacher, precontact_teacher"
            )
        if not np.isfinite(self.max_range) or self.max_range <= 0.0:
            raise ValueError("max_range must be finite and positive")
        if not np.isfinite(self.acceleration) or not -1.0 <= self.acceleration <= 1.0:
            raise ValueError("acceleration must be finite and within [-1.0, 1.0]")
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

        if self.architecture == 'small':
            self.model = TinyLidarNetSmallNp(input_dim=self.input_dim, output_dim=self.output_dim)
        else:
            self.model = TinyLidarNetNp(input_dim=self.input_dim, output_dim=self.output_dim)

        self.gap_teacher = None
        self.longitudinal_safety = None
        if self.control_mode == "gap_teacher":
            self.gap_teacher = LidarGapTeacher(
                gap_teacher_config or GapTeacherConfig()
            )
        elif self.control_mode == "precontact_teacher":
            self.gap_teacher = LidarPrecontactTeacher(
                gap_teacher_config or GapTeacherConfig()
            )
        elif self.control_mode == "fixed_lidar_brake":
            self.longitudinal_safety = LidarLongitudinalSafety(
                gap_teacher_config or GapTeacherConfig()
            )

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
            self.spatial_shadow_model = SpatialSteeringAdapterNp(
                input_dim=self.input_dim,
                hidden_dim=spatial_shadow_hidden_dim,
                projection_dim=spatial_shadow_projection_dim,
                use_speed=spatial_shadow_use_speed,
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

    def process(
        self, ranges: np.ndarray, speed_mps: Optional[float] = None
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
        ranges = np.asarray(ranges, dtype=np.float32)
        if ranges.ndim != 1 or ranges.size == 0:
            raise ValueError("ranges must be a non-empty 1D array")

        # 1. Preprocess (Clean -> Resize -> Normalize). Keep metre ranges for
        # the explicitly teacher-only gap policy; the network sees normalized data.
        physical_ranges = self._clean_and_resize_ranges(ranges)
        processed_ranges = physical_ranges / self.max_range

        # Prepare input tensor: (1, 1, input_dim)
        x = np.expand_dims(np.expand_dims(processed_ranges, axis=0), axis=1)

        # 2. Inference
        outputs = np.asarray(self.model(x)[0], dtype=np.float32)
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

        self.last_spatial_shadow_admitted = False
        self.last_spatial_shadow_correction_rad = 0.0
        self.last_spatial_shadow_direction_probabilities.fill(0.0)
        self.last_spatial_authority_correction_rad = 0.0
        self.last_spatial_authority_applied = False
        self.last_spatial_authority_clipped = False
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
                    ) = self.spatial_shadow_model.forward_components(
                        x, np.asarray([speed], dtype=np.float32)
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
        if self.gap_teacher is not None:
            decision = self.gap_teacher.decide(physical_ranges, steer, accel)
            self.last_gap_teacher_decision = decision
            accel = decision.acceleration_mps2
            steer = decision.steering_rad
        elif self.longitudinal_safety is not None:
            safety_decision = self.longitudinal_safety.decide(
                physical_ranges, accel
            )
            self.last_longitudinal_safety_decision = safety_decision
            accel = safety_decision.acceleration_mps2

        return accel, steer

    def reset_residual_history(self) -> None:
        """Prevent temporal context from crossing a runtime reset boundary."""
        self.previous_residual_scan = None

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

    def _load_model_weights(self, model, path: str, label: str) -> int:
        """Strictly load one NumPy model without accepting partial parameters."""
        try:
            weights = np.load(path, allow_pickle=True)

            if isinstance(weights, np.lib.npyio.NpzFile):
                weight_dict = dict(weights.items())
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
