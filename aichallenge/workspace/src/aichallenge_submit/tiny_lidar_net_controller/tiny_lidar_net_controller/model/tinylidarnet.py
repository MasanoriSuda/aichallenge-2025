import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np

# Assuming these are imported from your local module as per the original code
from . import (
    conv1d,
    linear,
    relu,
    sigmoid,
    softmax,
    tanh,
    flatten,
    kaiming_normal_init,
    zeros_init,
)

# ============================================================
# PyTorch Models
# ============================================================

class TinyLidarNet(nn.Module):
    """Standard CNN model for LiDAR data (Conv5 + FC4).

    This model processes 1D LiDAR scan data through 5 convolutional layers
    followed by 4 fully connected layers.

    Attributes:
        conv1 (nn.Conv1d): First convolutional layer.
        conv2 (nn.Conv1d): Second convolutional layer.
        conv3 (nn.Conv1d): Third convolutional layer.
        conv4 (nn.Conv1d): Fourth convolutional layer.
        conv5 (nn.Conv1d): Fifth convolutional layer.
        fc1 (nn.Linear): First fully connected layer.
        fc2 (nn.Linear): Second fully connected layer.
        fc3 (nn.Linear): Third fully connected layer.
        fc4 (nn.Linear): Output fully connected layer.
    """

    def __init__(self, input_dim=1080, output_dim=2):
        """Initializes TinyLidarNet.

        Args:
            input_dim (int): The size of the input LiDAR scan array. Defaults to 1080.
            output_dim (int): The size of the output prediction. Defaults to 2.
        """
        super().__init__()

        # --- Convolutional Layers ---
        self.conv1 = nn.Conv1d(1, 24, kernel_size=10, stride=4)
        self.conv2 = nn.Conv1d(24, 36, kernel_size=8, stride=4)
        self.conv3 = nn.Conv1d(36, 48, kernel_size=4, stride=2)
        self.conv4 = nn.Conv1d(48, 64, kernel_size=3)
        self.conv5 = nn.Conv1d(64, 64, kernel_size=3)

        # --- Calculate Flatten Dimension ---
        with torch.no_grad():
            dummy_input = torch.zeros(1, 1, input_dim)
            x = self.conv5(self.conv4(self.conv3(self.conv2(self.conv1(dummy_input)))))
            flatten_dim = x.view(1, -1).shape[1]

        # --- Fully Connected Layers ---
        self.fc1 = nn.Linear(flatten_dim, 100)
        self.fc2 = nn.Linear(100, 50)
        self.fc3 = nn.Linear(50, 10)
        self.fc4 = nn.Linear(10, output_dim)

        self._initialize_weights()

    def _initialize_weights(self):
        """Initializes weights using Kaiming Normal (He) initialization."""
        for m in self.modules():
            if isinstance(m, (nn.Conv1d, nn.Linear)):
                nn.init.kaiming_normal_(m.weight, mode='fan_out', nonlinearity='relu')
                if m.bias is not None:
                    nn.init.constant_(m.bias, 0)

    def forward(self, x):
        """Defines the computation performed at every call.

        Args:
            x (torch.Tensor): Input tensor of shape (batch_size, 1, input_dim).

        Returns:
            torch.Tensor: Output tensor of shape (batch_size, output_dim) with Tanh activation.
        """
        x = F.relu(self.conv1(x))
        x = F.relu(self.conv2(x))
        x = F.relu(self.conv3(x))
        x = F.relu(self.conv4(x))
        x = F.relu(self.conv5(x))
        x = x.view(x.size(0), -1)
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = F.relu(self.fc3(x))
        return torch.tanh(self.fc4(x))


class TinyLidarNetSmall(nn.Module):
    """Lightweight CNN model for LiDAR data (Conv3 + FC3).

    This model is a smaller version of TinyLidarNet, processing data through
    3 convolutional layers and 3 fully connected layers.
    """

    def __init__(self, input_dim=1080, output_dim=2):
        """Initializes TinyLidarNetSmall.

        Args:
            input_dim (int): The size of the input LiDAR scan array. Defaults to 1080.
            output_dim (int): The size of the output prediction. Defaults to 2.
        """
        super().__init__()

        self.conv1 = nn.Conv1d(1, 24, kernel_size=10, stride=4)
        self.conv2 = nn.Conv1d(24, 36, kernel_size=8, stride=4)
        self.conv3 = nn.Conv1d(36, 48, kernel_size=4, stride=2)

        with torch.no_grad():
            dummy_input = torch.zeros(1, 1, input_dim)
            x = self.conv3(self.conv2(self.conv1(dummy_input)))
            flatten_dim = x.view(1, -1).shape[1]

        self.fc1 = nn.Linear(flatten_dim, 100)
        self.fc2 = nn.Linear(100, 50)
        self.fc3 = nn.Linear(50, output_dim)

        self._initialize_weights()

    def _initialize_weights(self):
        """Initializes weights using Kaiming Normal (He) initialization."""
        for m in self.modules():
            if isinstance(m, (nn.Conv1d, nn.Linear)):
                nn.init.kaiming_normal_(m.weight, mode='fan_out', nonlinearity='relu')
                if m.bias is not None:
                    nn.init.constant_(m.bias, 0)

    def forward(self, x):
        """Defines the computation performed at every call.

        Args:
            x (torch.Tensor): Input tensor of shape (batch_size, 1, input_dim).

        Returns:
            torch.Tensor: Output tensor of shape (batch_size, output_dim) with Tanh activation.
        """
        x = F.relu(self.conv1(x))
        x = F.relu(self.conv2(x))
        x = F.relu(self.conv3(x))
        x = x.view(x.size(0), -1)
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        return torch.tanh(self.fc3(x))


# ============================================================
# NumPy Inference Models (Exact Naming Match with PyTorch)
# ============================================================

class TinyLidarNetNp:
    """NumPy implementation of TinyLidarNet (Conv5 + FC4).

    This class provides a pure NumPy inference implementation that matches the
    architecture of the PyTorch `TinyLidarNet` class.

    Attributes:
        params (dict): Stores weights and biases for all layers.
        strides (dict): Stores stride values for convolutional layers.
        shapes (dict): Stores parameter shapes for initialization.
    """

    def __init__(self, input_dim=1080, output_dim=2):
        """Initializes TinyLidarNetNp.

        Args:
            input_dim (int): The size of the input LiDAR scan array. Defaults to 1080.
            output_dim (int): The size of the output prediction. Defaults to 2.
        """
        self.input_dim = input_dim
        self.output_dim = output_dim
        self.params = {}

        # Stride definitions
        self.strides = {'conv1': 4, 'conv2': 4, 'conv3': 2, 'conv4': 1, 'conv5': 1}

        # Shape definitions matching PyTorch
        self.shapes = {
            'conv1_weight': (24, 1, 10),  'conv1_bias': (24,),
            'conv2_weight': (36, 24, 8),  'conv2_bias': (36,),
            'conv3_weight': (48, 36, 4),  'conv3_bias': (48,),
            'conv4_weight': (64, 48, 3),  'conv4_bias': (64,),
            'conv5_weight': (64, 64, 3),  'conv5_bias': (64,),
        }

        flatten_dim = self._get_conv_output_dim()
        self.shapes.update({
            'fc1_weight': (100, flatten_dim), 'fc1_bias': (100,),
            'fc2_weight': (50, 100),          'fc2_bias': (50,),
            'fc3_weight': (10, 50),           'fc3_bias': (10,),
            'fc4_weight': (output_dim, 10),   'fc4_bias': (output_dim,),
        })

        self._initialize_weights()

    def _get_conv_output_dim(self):
        """Calculates the flattened dimension after the last convolution layer."""
        l = self.input_dim
        for i in range(1, 6):
            k = self.shapes[f'conv{i}_weight'][2]
            s = self.strides[f'conv{i}']
            l = (l - k) // s + 1
        c = self.shapes['conv5_weight'][0]
        return c * l

    def _initialize_weights(self):
        """Initializes weights using Kaiming Normal (fan_out) and biases to zero."""
        for name, shape in self.shapes.items():
            if name.endswith('_weight'):
                fan_out = shape[0] * (shape[2] if 'conv' in name else 1)
                self.params[name] = kaiming_normal_init(shape, fan_out)
            elif name.endswith('_bias'):
                self.params[name] = zeros_init(shape)

    def forward_conv5_features(self, x):
        """Return the flattened Conv5 features for a validated scan batch.

        Spatial and recurrent adapters embed this exact frozen backbone.  The
        public feature boundary lets the runtime evaluate that backbone once
        without changing the numerical order of the production network.
        """
        values = np.asarray(x, dtype=np.float32)
        if values.ndim != 3 or values.shape[1:] != (1, self.input_dim):
            raise ValueError(
                "TinyLidarNet input must have shape "
                f"(batch, 1, {self.input_dim})"
            )
        if not np.all(np.isfinite(values)):
            raise ValueError("TinyLidarNet input must be finite")
        for index in range(1, 6):
            values = relu(conv1d(
                values,
                self.params[f'conv{index}_weight'],
                self.params[f'conv{index}_bias'],
                self.strides[f'conv{index}'],
            ))
        return flatten(values)

    def forward_from_conv5_features(self, conv5_features):
        """Evaluate the production head from precomputed Conv5 features."""
        values = np.asarray(conv5_features, dtype=np.float32)
        expected_dim = self.shapes['fc1_weight'][1]
        if values.ndim != 2 or values.shape[1] != expected_dim:
            raise ValueError(
                "TinyLidarNet Conv5 features must have shape "
                f"(batch, {expected_dim})"
            )
        if not np.all(np.isfinite(values)):
            raise ValueError("TinyLidarNet Conv5 features must be finite")
        values = relu(linear(values, self.params['fc1_weight'], self.params['fc1_bias']))
        values = relu(linear(values, self.params['fc2_weight'], self.params['fc2_bias']))
        values = relu(linear(values, self.params['fc3_weight'], self.params['fc3_bias']))
        return tanh(linear(values, self.params['fc4_weight'], self.params['fc4_bias']))

    def __call__(self, x):
        """Performs the forward pass of the model.

        Args:
            x (np.ndarray): Input array of shape (batch_size, 1, input_dim).

        Returns:
            np.ndarray: Output array of shape (batch_size, output_dim).
        """
        return self.forward_from_conv5_features(self.forward_conv5_features(x))


class TinyLidarNetSmallNp:
    """NumPy implementation of TinyLidarNetSmall (Conv3 + FC3).

    This class provides a pure NumPy inference implementation that matches the
    architecture of the PyTorch `TinyLidarNetSmall` class.
    """

    def __init__(self, input_dim=1080, output_dim=2):
        """Initializes TinyLidarNetSmallNp.

        Args:
            input_dim (int): The size of the input LiDAR scan array. Defaults to 1080.
            output_dim (int): The size of the output prediction. Defaults to 2.
        """
        self.input_dim = input_dim
        self.output_dim = output_dim
        self.params = {}
        self.strides = {'conv1': 4, 'conv2': 4, 'conv3': 2}

        self.shapes = {
            'conv1_weight': (24, 1, 10),  'conv1_bias': (24,),
            'conv2_weight': (36, 24, 8),  'conv2_bias': (36,),
            'conv3_weight': (48, 36, 4),  'conv3_bias': (48,),
        }

        flatten_dim = self._get_conv_output_dim()
        self.shapes.update({
            'fc1_weight': (100, flatten_dim), 'fc1_bias': (100,),
            'fc2_weight': (50, 100),          'fc2_bias': (50,),
            'fc3_weight': (output_dim, 50),   'fc3_bias': (output_dim,),
        })

        self._initialize_weights()

    def _get_conv_output_dim(self):
        """Calculates the flattened dimension after the last convolution layer."""
        l = self.input_dim
        for i in range(1, 4):
            k = self.shapes[f'conv{i}_weight'][2]
            s = self.strides[f'conv{i}']
            l = (l - k) // s + 1
        c = self.shapes['conv3_weight'][0]
        return c * l

    def _initialize_weights(self):
        """Initializes weights using Kaiming Normal (fan_out) and biases to zero."""
        for name, shape in self.shapes.items():
            if name.endswith('_weight'):
                fan_out = shape[0] * (shape[2] if 'conv' in name else 1)
                self.params[name] = kaiming_normal_init(shape, fan_out)
            elif name.endswith('_bias'):
                self.params[name] = zeros_init(shape)

    def __call__(self, x):
        """Performs the forward pass of the model.

        Args:
            x (np.ndarray): Input array of shape (batch_size, 1, input_dim).

        Returns:
            np.ndarray: Output array of shape (batch_size, output_dim).
        """
        x = relu(conv1d(x, self.params['conv1_weight'], self.params['conv1_bias'], self.strides['conv1']))
        x = relu(conv1d(x, self.params['conv2_weight'], self.params['conv2_bias'], self.strides['conv2']))
        x = relu(conv1d(x, self.params['conv3_weight'], self.params['conv3_bias'], self.strides['conv3']))
        x = flatten(x)
        x = relu(linear(x, self.params['fc1_weight'], self.params['fc1_bias']))
        x = relu(linear(x, self.params['fc2_weight'], self.params['fc2_bias']))
        return tanh(linear(x, self.params['fc3_weight'], self.params['fc3_bias']))


class SteeringResidualNetNp:
    """NumPy inference counterpart of the offline two-head residual model."""

    def __init__(self, input_dim=750, max_abs_delta_rad=1.28, input_channels=1):
        if input_dim <= 0:
            raise ValueError("input_dim must be positive")
        if not np.isfinite(max_abs_delta_rad) or max_abs_delta_rad <= 0.0:
            raise ValueError("max_abs_delta_rad must be finite and positive")
        if input_channels not in (1, 2):
            raise ValueError("input_channels must be 1 or 2")
        self.input_dim = input_dim
        self.input_channels = int(input_channels)
        self.max_abs_delta_rad = float(max_abs_delta_rad)
        self.params = {}
        self.strides = {'conv1': 4, 'conv2': 4, 'conv3': 2}
        self.shapes = {
            'conv1_weight': (16, self.input_channels, 10), 'conv1_bias': (16,),
            'conv2_weight': (24, 16, 8), 'conv2_bias': (24,),
            'conv3_weight': (32, 24, 4), 'conv3_bias': (32,),
        }
        length = input_dim
        for index in range(1, 4):
            kernel = self.shapes[f'conv{index}_weight'][2]
            length = (length - kernel) // self.strides[f'conv{index}'] + 1
        flatten_dim = 32 * length
        self.shapes.update({
            'fc1_weight': (64, flatten_dim), 'fc1_bias': (64,),
            'fc2_weight': (16, 64), 'fc2_bias': (16,),
            'correction_head_weight': (1, 16), 'correction_head_bias': (1,),
            'gate_head_weight': (1, 16), 'gate_head_bias': (1,),
        })
        for name, shape in self.shapes.items():
            self.params[name] = zeros_init(shape)

    def forward_components(self, x):
        x = relu(conv1d(
            x, self.params['conv1_weight'], self.params['conv1_bias'],
            self.strides['conv1']))
        x = relu(conv1d(
            x, self.params['conv2_weight'], self.params['conv2_bias'],
            self.strides['conv2']))
        x = relu(conv1d(
            x, self.params['conv3_weight'], self.params['conv3_bias'],
            self.strides['conv3']))
        x = flatten(x)
        x = relu(linear(x, self.params['fc1_weight'], self.params['fc1_bias']))
        x = relu(linear(x, self.params['fc2_weight'], self.params['fc2_bias']))
        correction = tanh(linear(
            x,
            self.params['correction_head_weight'],
            self.params['correction_head_bias'],
        )) * self.max_abs_delta_rad
        gate_logit = linear(
            x, self.params['gate_head_weight'], self.params['gate_head_bias']
        )
        gate = 1.0 / (1.0 + np.exp(-np.clip(gate_logit, -60.0, 60.0)))
        return gate * correction, correction, gate_logit

    def __call__(self, x):
        residual, _, _ = self.forward_components(x)
        return residual


class SpatialSteeringAdapterNp:
    """NumPy shadow counterpart of the offline signed spatial adapter.

    The checkpoint contains a frozen TinyLidarNet, a deterministic spatial
    projection, fixed train statistics and the correction head.  The embedded
    base is checked by :class:`TinyLidarNetCore` before shadow inference is
    enabled; this model never owns the published command.
    """

    def __init__(
        self,
        input_dim=750,
        hidden_dim=128,
        projection_dim=128,
        use_speed=True,
        use_base_steering=False,
        max_speed_mps=12.0,
        max_abs_delta_rad=1.2,
    ):
        if input_dim <= 0 or hidden_dim < 2 or projection_dim <= 0:
            raise ValueError("spatial adapter dimensions must be positive")
        if not np.isfinite(max_speed_mps) or max_speed_mps <= 0.0:
            raise ValueError("spatial adapter maximum speed must be positive")
        if not np.isfinite(max_abs_delta_rad) or max_abs_delta_rad <= 0.0:
            raise ValueError("spatial adapter correction bound must be positive")
        self.input_dim = int(input_dim)
        self.hidden_dim = int(hidden_dim)
        self.projection_dim = int(projection_dim)
        self.use_speed = bool(use_speed)
        self.use_base_steering = bool(use_base_steering)
        self.max_speed_mps = float(max_speed_mps)
        self.max_abs_delta_rad = float(max_abs_delta_rad)
        self.base = TinyLidarNetNp(input_dim=self.input_dim, output_dim=2)
        self.strides = self.base.strides
        self.spatial_dim = self.base.shapes['fc1_weight'][1]
        adapter_input_dim = (
            self.projection_dim
            + int(self.use_speed)
            + int(self.use_base_steering)
        )
        self.shapes = {
            'spatial_projection': (self.spatial_dim, self.projection_dim),
            'spatial_mean': (self.projection_dim,),
            'spatial_scale': (self.projection_dim,),
            **{
                f'base_{key}': shape
                for key, shape in self.base.shapes.items()
            },
            'spatial_head_0_weight': (self.hidden_dim, adapter_input_dim),
            'spatial_head_0_bias': (self.hidden_dim,),
            'spatial_head_2_weight': (self.hidden_dim // 2, self.hidden_dim),
            'spatial_head_2_bias': (self.hidden_dim // 2,),
            'direction_head_weight': (3, self.hidden_dim // 2),
            'direction_head_bias': (3,),
            'magnitude_head_weight': (2, self.hidden_dim // 2),
            'magnitude_head_bias': (2,),
        }
        self.params = {
            name: zeros_init(shape) for name, shape in self.shapes.items()
        }

    def embedded_base_parameters(self):
        return {
            key.removeprefix('base_'): value
            for key, value in self.params.items()
            if key.startswith('base_')
        }

    def _spatial_features(self, normalized_scans):
        values = np.asarray(normalized_scans, dtype=np.float32)
        if values.ndim != 3 or values.shape[1:] != (1, self.input_dim):
            raise ValueError(
                "spatial adapter scan input must have shape "
                f"(batch, 1, {self.input_dim})"
            )
        if not np.all(np.isfinite(values)):
            raise ValueError("spatial adapter scan input must be finite")
        values = np.clip(values, 0.0, 1.0)
        for index in range(1, 6):
            values = relu(conv1d(
                values,
                self.params[f'base_conv{index}_weight'],
                self.params[f'base_conv{index}_bias'],
                self.strides[f'conv{index}'],
            ))
        return flatten(values)

    def forward_components_from_spatial_features(
        self, spatial_features, speeds_mps=None, base_steering=None
    ):
        """Evaluate the adapter head from a verified shared Conv5 tensor."""
        spatial = np.asarray(spatial_features, dtype=np.float32)
        if spatial.ndim != 2 or spatial.shape[1] != self.spatial_dim:
            raise ValueError(
                "spatial adapter features must have shape "
                f"(batch, {self.spatial_dim})"
            )
        if not np.all(np.isfinite(spatial)):
            raise ValueError("spatial adapter features must be finite")
        projected = spatial @ self.params['spatial_projection']
        scale = self.params['spatial_scale']
        if not np.all(np.isfinite(scale)) or np.any(scale <= 0.0):
            raise ValueError("spatial adapter scale must be finite and positive")
        features = [(projected - self.params['spatial_mean']) / scale]
        if self.use_speed:
            speed = np.asarray(speeds_mps, dtype=np.float32)
            if speed.shape != (len(spatial),):
                raise ValueError("speed-enabled shadow requires one speed per scan")
            if not np.all(np.isfinite(speed)) or np.any(speed < 0.0):
                raise ValueError("spatial shadow speed must be finite and non-negative")
            normalized_speed = np.clip(
                speed / self.max_speed_mps, 0.0, 1.5
            )[:, None]
            features.append(normalized_speed)
        if self.use_base_steering:
            if base_steering is None:
                base_hidden = relu(linear(
                    spatial,
                    self.params['base_fc1_weight'],
                    self.params['base_fc1_bias'],
                ))
                base_hidden = relu(linear(
                    base_hidden,
                    self.params['base_fc2_weight'],
                    self.params['base_fc2_bias'],
                ))
                base_hidden = relu(linear(
                    base_hidden,
                    self.params['base_fc3_weight'],
                    self.params['base_fc3_bias'],
                ))
                base_output = tanh(linear(
                    base_hidden,
                    self.params['base_fc4_weight'],
                    self.params['base_fc4_bias'],
                ))
                base_values = base_output[:, 1]
            else:
                base_values = np.asarray(base_steering, dtype=np.float32)
                if base_values.shape != (len(spatial),):
                    raise ValueError(
                        "spatial adapter base steering must have one value per scan"
                    )
                if not np.all(np.isfinite(base_values)):
                    raise ValueError("spatial adapter base steering must be finite")
            features.append(base_values[:, None])
        features = np.concatenate(features, axis=1)
        hidden = relu(linear(
            features,
            self.params['spatial_head_0_weight'],
            self.params['spatial_head_0_bias'],
        ))
        hidden = relu(linear(
            hidden,
            self.params['spatial_head_2_weight'],
            self.params['spatial_head_2_bias'],
        ))
        direction_logits = linear(
            hidden,
            self.params['direction_head_weight'],
            self.params['direction_head_bias'],
        )
        direction_probabilities = softmax(direction_logits)
        magnitudes = sigmoid(linear(
            hidden,
            self.params['magnitude_head_weight'],
            self.params['magnitude_head_bias'],
        )) * self.max_abs_delta_rad
        residual = (
            direction_probabilities[:, 2] * magnitudes[:, 1]
            - direction_probabilities[:, 0] * magnitudes[:, 0]
        )
        if not all(
            np.all(np.isfinite(value))
            for value in (residual, magnitudes, direction_probabilities)
        ):
            raise ValueError("spatial shadow output contains non-finite values")
        return residual, magnitudes, direction_logits, direction_probabilities

    def forward_components(self, normalized_scans, speeds_mps=None):
        return self.forward_components_from_spatial_features(
            self._spatial_features(normalized_scans), speeds_mps
        )

    def __call__(self, normalized_scans, speeds_mps=None):
        residual, _, _, _ = self.forward_components(
            normalized_scans, speeds_mps
        )
        return residual


class RecurrentSteeringAdapterNp:
    """NumPy shadow counterpart of the projected-conv5 recurrent adapter.

    The model owns no ROS authority.  It mirrors PyTorch's one-layer GRU
    equations exactly and retains hidden state only when its caller explicitly
    supplies the previous finite state.
    """

    ARTIFACT_SCHEMA_VERSION = 1
    ARTIFACT_METADATA_PREFIX = '__recurrent_runtime_'
    ARTIFACT_CONFIG_TYPES = {
        'input_dim': int,
        'hidden_dim': int,
        'projection_dim': int,
        'use_speed': bool,
        'speed_embedding_dim': int,
        'max_speed_mps': float,
        'max_abs_correction_rad': float,
        'max_abs_steering_rad': float,
        'correction_deadband_rad': float,
        'spatial_baseline_hidden_dim': int,
        'spatial_baseline_projection_dim': int,
        'spatial_baseline_max_speed_mps': float,
        'spatial_baseline_max_abs_delta_rad': float,
    }

    @classmethod
    def artifact_metadata(cls, runtime_config):
        """Encode the construction contract alongside immutable weights."""
        missing = sorted(set(cls.ARTIFACT_CONFIG_TYPES) - set(runtime_config))
        if missing:
            raise ValueError(
                f"recurrent runtime config is incomplete: missing={missing}"
            )
        metadata = {
            f'{cls.ARTIFACT_METADATA_PREFIX}schema_version': np.asarray(
                cls.ARTIFACT_SCHEMA_VERSION, dtype=np.int64
            )
        }
        for name, value_type in cls.ARTIFACT_CONFIG_TYPES.items():
            value = runtime_config[name]
            if value_type is bool:
                encoded = np.asarray(int(bool(value)), dtype=np.int64)
            elif value_type is int:
                encoded = np.asarray(int(value), dtype=np.int64)
            else:
                encoded = np.asarray(float(value), dtype=np.float64)
            metadata[f'{cls.ARTIFACT_METADATA_PREFIX}{name}'] = encoded
        return metadata

    @classmethod
    def split_artifact(cls, weights):
        """Return a self-described runtime config and its strict weight set.

        Older diagnostic artifacts without metadata remain readable through the
        caller's explicit configuration.  A partially described artifact is
        rejected rather than silently mixing two architecture contracts.
        """
        metadata = {
            key: value
            for key, value in weights.items()
            if key.startswith(cls.ARTIFACT_METADATA_PREFIX)
        }
        parameters = {
            key: value
            for key, value in weights.items()
            if not key.startswith(cls.ARTIFACT_METADATA_PREFIX)
        }
        if not metadata:
            return None, parameters

        expected_keys = {
            f'{cls.ARTIFACT_METADATA_PREFIX}schema_version',
            *(
                f'{cls.ARTIFACT_METADATA_PREFIX}{name}'
                for name in cls.ARTIFACT_CONFIG_TYPES
            ),
        }
        provided_keys = set(metadata)
        if provided_keys != expected_keys:
            raise ValueError(
                "recurrent artifact metadata mismatch: "
                f"missing={sorted(expected_keys - provided_keys)}, "
                f"unexpected={sorted(provided_keys - expected_keys)}"
            )

        def scalar(name):
            key = f'{cls.ARTIFACT_METADATA_PREFIX}{name}'
            value = np.asarray(metadata[key])
            if value.shape != () or not np.issubdtype(value.dtype, np.number):
                raise ValueError(
                    f"recurrent artifact metadata {name} must be numeric scalar"
                )
            numeric = value.item()
            if not np.isfinite(numeric):
                raise ValueError(
                    f"recurrent artifact metadata {name} must be finite"
                )
            return numeric

        schema_version = int(scalar('schema_version'))
        if schema_version != cls.ARTIFACT_SCHEMA_VERSION:
            raise ValueError(
                "unsupported recurrent artifact schema: "
                f"{schema_version}"
            )
        config = {}
        for name, value_type in cls.ARTIFACT_CONFIG_TYPES.items():
            value = scalar(name)
            if value_type is bool:
                if value not in (0, 1):
                    raise ValueError(
                        f"recurrent artifact metadata {name} must be 0 or 1"
                    )
                config[name] = bool(value)
            elif value_type is int:
                if int(value) != value:
                    raise ValueError(
                        f"recurrent artifact metadata {name} must be integral"
                    )
                config[name] = int(value)
            else:
                config[name] = float(value)
        return config, parameters

    def __init__(
        self,
        input_dim=750,
        hidden_dim=64,
        projection_dim=128,
        use_speed=False,
        speed_embedding_dim=16,
        max_speed_mps=12.0,
        max_abs_correction_rad=0.64,
        max_abs_steering_rad=1.0,
        correction_deadband_rad=0.02,
        spatial_baseline_hidden_dim=128,
        spatial_baseline_projection_dim=128,
        spatial_baseline_max_speed_mps=12.0,
        spatial_baseline_max_abs_delta_rad=1.2,
    ):
        positive_ints = (
            input_dim,
            hidden_dim,
            projection_dim,
            speed_embedding_dim,
            spatial_baseline_hidden_dim,
            spatial_baseline_projection_dim,
        )
        if any(not isinstance(value, int) or value <= 0 for value in positive_ints):
            raise ValueError("recurrent adapter dimensions must be positive integers")
        positive_floats = (
            max_speed_mps,
            max_abs_correction_rad,
            max_abs_steering_rad,
            spatial_baseline_max_speed_mps,
            spatial_baseline_max_abs_delta_rad,
        )
        if any(not np.isfinite(value) or value <= 0.0 for value in positive_floats):
            raise ValueError("recurrent adapter scales must be finite and positive")
        if (
            not np.isfinite(correction_deadband_rad)
            or correction_deadband_rad < 0.0
            or correction_deadband_rad > max_abs_correction_rad
        ):
            raise ValueError("recurrent correction deadband is invalid")

        self.input_dim = int(input_dim)
        self.hidden_dim = int(hidden_dim)
        self.projection_dim = int(projection_dim)
        self.use_speed = bool(use_speed)
        self.speed_embedding_dim = int(speed_embedding_dim)
        self.max_speed_mps = float(max_speed_mps)
        self.max_abs_correction_rad = float(max_abs_correction_rad)
        self.max_abs_steering_rad = float(max_abs_steering_rad)
        self.correction_deadband_rad = float(correction_deadband_rad)
        self.base = TinyLidarNetNp(input_dim=self.input_dim, output_dim=2)
        self.strides = self.base.strides
        self.spatial_dim = self.base.shapes['fc1_weight'][1]
        self.spatial_baseline = SpatialSteeringAdapterNp(
            input_dim=self.input_dim,
            hidden_dim=spatial_baseline_hidden_dim,
            projection_dim=spatial_baseline_projection_dim,
            use_speed=True,
            use_base_steering=True,
            max_speed_mps=spatial_baseline_max_speed_mps,
            max_abs_delta_rad=spatial_baseline_max_abs_delta_rad,
        )
        self.spatial_baseline_max_speed_mps = float(
            spatial_baseline_max_speed_mps
        )
        self.spatial_baseline_max_abs_delta_rad = float(
            spatial_baseline_max_abs_delta_rad
        )
        recurrent_input_dim = self.projection_dim + (
            self.speed_embedding_dim if self.use_speed else 0
        )
        self.shapes = {
            'spatial_projection': (self.spatial_dim, self.projection_dim),
            'spatial_mean': (self.projection_dim,),
            'spatial_scale': (self.projection_dim,),
            **{f'base_{key}': shape for key, shape in self.base.shapes.items()},
            **{
                f'spatial_baseline_{key}': shape
                for key, shape in self.spatial_baseline.shapes.items()
            },
            'gru_weight_ih_l0': (3 * self.hidden_dim, recurrent_input_dim),
            'gru_weight_hh_l0': (3 * self.hidden_dim, self.hidden_dim),
            'gru_bias_ih_l0': (3 * self.hidden_dim,),
            'gru_bias_hh_l0': (3 * self.hidden_dim,),
            'correction_hidden_weight': (64, self.hidden_dim),
            'correction_hidden_bias': (64,),
            'correction_output_weight': (1, 64),
            'correction_output_bias': (1,),
        }
        if self.use_speed:
            self.shapes.update({
                'speed_mlp_0_weight': (self.speed_embedding_dim, 1),
                'speed_mlp_0_bias': (self.speed_embedding_dim,),
            })
        self.params = {
            name: zeros_init(shape) for name, shape in self.shapes.items()
        }

    def embedded_base_parameters(self):
        return {
            key.removeprefix('base_'): value
            for key, value in self.params.items()
            if key.startswith('base_')
        }

    def embedded_spatial_parameters(self):
        prefix = 'spatial_baseline_'
        return {
            key.removeprefix(prefix): value
            for key, value in self.params.items()
            if key.startswith(prefix)
        }

    def _conv5_features(self, normalized_scans, prefix):
        values = np.asarray(normalized_scans, dtype=np.float32)
        if values.ndim != 3 or values.shape[1:] != (1, self.input_dim):
            raise ValueError(
                "recurrent scan input must have shape "
                f"(batch, 1, {self.input_dim})"
            )
        if not np.all(np.isfinite(values)):
            raise ValueError("recurrent scan input must be finite")
        values = np.clip(values, 0.0, 1.0)
        for index in range(1, 6):
            values = relu(conv1d(
                values,
                self.params[f'{prefix}conv{index}_weight'],
                self.params[f'{prefix}conv{index}_bias'],
                self.strides[f'conv{index}'],
            ))
        return flatten(values)

    def _base_steering(self, conv5_features, prefix):
        hidden = conv5_features
        for index in range(1, 4):
            hidden = relu(linear(
                hidden,
                self.params[f'{prefix}fc{index}_weight'],
                self.params[f'{prefix}fc{index}_bias'],
            ))
        return tanh(linear(
            hidden,
            self.params[f'{prefix}fc4_weight'],
            self.params[f'{prefix}fc4_bias'],
        ))[:, 1]

    def _spatial_baseline_residual(self, normalized_scans, speeds_mps):
        prefix = 'spatial_baseline_'
        conv5 = self._conv5_features(normalized_scans, f'{prefix}base_')
        projected = conv5 @ self.params[f'{prefix}spatial_projection']
        scale = self.params[f'{prefix}spatial_scale']
        if not np.all(np.isfinite(scale)) or np.any(scale <= 0.0):
            raise ValueError("recurrent spatial baseline scale is invalid")
        normalized_speed = np.clip(
            speeds_mps / self.spatial_baseline_max_speed_mps, 0.0, 1.5
        )[:, None]
        embedded_steering = self._base_steering(conv5, f'{prefix}base_')[:, None]
        features = np.concatenate(
            (
                (projected - self.params[f'{prefix}spatial_mean']) / scale,
                normalized_speed,
                embedded_steering,
            ),
            axis=1,
        )
        hidden = relu(linear(
            features,
            self.params[f'{prefix}spatial_head_0_weight'],
            self.params[f'{prefix}spatial_head_0_bias'],
        ))
        hidden = relu(linear(
            hidden,
            self.params[f'{prefix}spatial_head_2_weight'],
            self.params[f'{prefix}spatial_head_2_bias'],
        ))
        direction = softmax(linear(
            hidden,
            self.params[f'{prefix}direction_head_weight'],
            self.params[f'{prefix}direction_head_bias'],
        ))
        magnitude = sigmoid(linear(
            hidden,
            self.params[f'{prefix}magnitude_head_weight'],
            self.params[f'{prefix}magnitude_head_bias'],
        )) * self.spatial_baseline_max_abs_delta_rad
        return direction[:, 2] * magnitude[:, 1] - direction[:, 0] * magnitude[:, 0]

    @staticmethod
    def _stable_sigmoid(values):
        return 1.0 / (1.0 + np.exp(-np.clip(values, -60.0, 60.0)))

    def _gru_step(self, features, hidden):
        values = np.asarray(features, dtype=np.float32)
        if values.ndim != 2:
            raise ValueError("recurrent features must be two-dimensional")
        if hidden is None:
            previous = np.zeros(
                (len(values), self.hidden_dim), dtype=np.float32
            )
        else:
            previous = np.asarray(hidden, dtype=np.float32)
            if previous.shape != (len(values), self.hidden_dim):
                raise ValueError("recurrent hidden state shape mismatch")
            if not np.all(np.isfinite(previous)):
                raise ValueError("recurrent hidden state must be finite")
        input_gates = linear(
            values,
            self.params['gru_weight_ih_l0'],
            self.params['gru_bias_ih_l0'],
        )
        hidden_gates = linear(
            previous,
            self.params['gru_weight_hh_l0'],
            self.params['gru_bias_hh_l0'],
        )
        input_reset, input_update, input_new = np.split(input_gates, 3, axis=1)
        hidden_reset, hidden_update, hidden_new = np.split(hidden_gates, 3, axis=1)
        reset = self._stable_sigmoid(input_reset + hidden_reset)
        update = self._stable_sigmoid(input_update + hidden_update)
        new = np.tanh(input_new + reset * hidden_new)
        next_hidden = (1.0 - update) * new + update * previous
        return next_hidden.astype(np.float32, copy=False)

    def forward_correction_from_conv5_features(
        self, conv5_features, speeds_mps, hidden=None
    ):
        """Evaluate only the recurrent correction from a shared backbone.

        The embedded production baselines remain in the artifact and are
        checked exactly when the core loads it.  They are intentionally not
        recomputed here, so diagnostic shadow work cannot repeat the frozen
        production CNN on every scan.
        """
        conv5 = np.asarray(conv5_features, dtype=np.float32)
        speed = np.asarray(speeds_mps, dtype=np.float32)
        if conv5.ndim != 2 or conv5.shape[1] != self.spatial_dim:
            raise ValueError(
                "recurrent Conv5 features must have shape "
                f"(batch, {self.spatial_dim})"
            )
        if not np.all(np.isfinite(conv5)):
            raise ValueError("recurrent Conv5 features must be finite")
        if speed.shape != (len(conv5),):
            raise ValueError("recurrent shadow requires one speed per scan")
        if not np.all(np.isfinite(speed)) or np.any(speed < 0.0):
            raise ValueError("recurrent shadow speed must be finite and non-negative")

        projected = conv5 @ self.params['spatial_projection']
        scale = self.params['spatial_scale']
        if not np.all(np.isfinite(scale)) or np.any(scale <= 0.0):
            raise ValueError("recurrent spatial scale is invalid")
        features = [(projected - self.params['spatial_mean']) / scale]
        if self.use_speed:
            normalized_speed = np.clip(
                speed / self.max_speed_mps, 0.0, 1.5
            )[:, None]
            features.append(relu(linear(
                normalized_speed,
                self.params['speed_mlp_0_weight'],
                self.params['speed_mlp_0_bias'],
            )))
        next_hidden = self._gru_step(np.concatenate(features, axis=1), hidden)
        correction_features = relu(linear(
            next_hidden,
            self.params['correction_hidden_weight'],
            self.params['correction_hidden_bias'],
        ))
        raw_correction = tanh(linear(
            correction_features,
            self.params['correction_output_weight'],
            self.params['correction_output_bias'],
        ))[:, 0] * self.max_abs_correction_rad
        applied_correction = np.where(
            np.abs(raw_correction) >= self.correction_deadband_rad,
            raw_correction,
            0.0,
        )
        outputs = (applied_correction, raw_correction, next_hidden)
        if not all(np.all(np.isfinite(value)) for value in outputs):
            raise ValueError("recurrent shadow output contains non-finite values")
        return outputs

    def forward_components(self, normalized_scans, speeds_mps, hidden=None):
        scans = np.asarray(normalized_scans, dtype=np.float32)
        speed = np.asarray(speeds_mps, dtype=np.float32)
        if speed.shape != (len(scans),):
            raise ValueError("recurrent shadow requires one speed per scan")
        if not np.all(np.isfinite(speed)) or np.any(speed < 0.0):
            raise ValueError("recurrent shadow speed must be finite and non-negative")

        conv5 = self._conv5_features(scans, 'base_')
        applied_correction, raw_correction, next_hidden = (
            self.forward_correction_from_conv5_features(conv5, speed, hidden)
        )
        raw_base = self._base_steering(conv5, 'base_')
        spatial_residual = self._spatial_baseline_residual(scans, speed)
        production_base = np.clip(
            raw_base + spatial_residual,
            -self.max_abs_steering_rad,
            self.max_abs_steering_rad,
        )
        steering = np.clip(
            production_base + applied_correction,
            -self.max_abs_steering_rad,
            self.max_abs_steering_rad,
        )
        outputs = (
            steering,
            applied_correction,
            raw_correction,
            production_base,
            next_hidden,
        )
        if not all(np.all(np.isfinite(value)) for value in outputs):
            raise ValueError("recurrent shadow output contains non-finite values")
        return outputs

    def __call__(self, normalized_scans, speeds_mps, hidden=None):
        steering, _, _, _, next_hidden = self.forward_components(
            normalized_scans, speeds_mps, hidden
        )
        return steering, next_hidden
