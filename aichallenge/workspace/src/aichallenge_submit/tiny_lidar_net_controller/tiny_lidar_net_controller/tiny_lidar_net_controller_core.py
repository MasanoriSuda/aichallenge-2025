import logging
import numpy as np
from typing import Tuple

from tiny_lidar_net_controller.model.tinylidarnet import (
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
        control_mode (str): Control strategy ('ai' or 'fixed').
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
        max_range: float = 30.0
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

        if not isinstance(self.input_dim, int) or self.input_dim <= 0:
            raise ValueError("input_dim must be a positive integer")
        if self.output_dim != 2:
            raise ValueError("output_dim must be 2 with [acceleration, steering] semantics")
        if self.architecture not in {"normal", "large", "small"}:
            raise ValueError(
                "architecture must be one of: normal, large, small"
            )
        if self.control_mode not in {"ai", "fixed"}:
            raise ValueError("control_mode must be either 'ai' or 'fixed'")
        if not np.isfinite(self.max_range) or self.max_range <= 0.0:
            raise ValueError("max_range must be finite and positive")
        if not np.isfinite(self.acceleration) or not -1.0 <= self.acceleration <= 1.0:
            raise ValueError("acceleration must be finite and within [-1.0, 1.0]")

        if self.architecture == 'small':
            self.model = TinyLidarNetSmallNp(input_dim=self.input_dim, output_dim=self.output_dim)
        else:
            self.model = TinyLidarNetNp(input_dim=self.input_dim, output_dim=self.output_dim)

        if not ckpt_path:
            raise ValueError("ckpt_path is required; random production weights are forbidden")
        self._load_weights(ckpt_path)

    def process(self, ranges: np.ndarray) -> Tuple[float, float]:
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

        # 1. Preprocess (Clean -> Resize -> Normalize)
        processed_ranges = self._preprocess_ranges(ranges)

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

        return accel, steer

    def _load_weights(self, path: str) -> None:
        """Loads model weights from a file into the model parameters.

        Args:
            path (str): Path to the .npy or .npz weight file.

        Raises:
            ValueError: If the weight file format is unsupported.
            IOError: If the file cannot be read.
        """
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

            expected_keys = set(self.model.params)
            provided_keys = set(normalized_weights)
            missing = sorted(expected_keys - provided_keys)
            unexpected = sorted(provided_keys - expected_keys)
            if missing or unexpected:
                raise ValueError(
                    f"weight key mismatch: missing={missing}, unexpected={unexpected}"
                )

            validated_weights = {}
            for key, expected in self.model.params.items():
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

            self.model.params.update(validated_weights)
            self.loaded_parameter_count = len(validated_weights)
            self.logger.info(
                "Successfully loaded %d validated parameters from %s",
                self.loaded_parameter_count,
                path,
            )

        except Exception as e:
            self.logger.error(f"Failed to load weights from {path}: {e}")
            raise e

    def _preprocess_ranges(self, ranges: np.ndarray) -> np.ndarray:
        """Cleans, resizes, and normalizes LiDAR ranges.

        This method performs the following operations:
        1. Replaces NaNs with 0.0.
        2. Replaces infinite values with `self.max_range`.
        3. Clips all values to the range [0.0, `self.max_range`].
        4. Resizes the array to match `self.input_dim` via interpolation or padding.
        5. Normalizes the data by dividing by `self.max_range`.

        Args:
            ranges (np.ndarray): Source LiDAR range data.

        Returns:
            np.ndarray: Processed data array of shape (self.input_dim,).
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

        # Normalize
        return ranges / self.max_range 
