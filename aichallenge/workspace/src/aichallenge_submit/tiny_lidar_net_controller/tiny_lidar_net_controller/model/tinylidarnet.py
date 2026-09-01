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
        x = relu(conv1d(x, self.params['conv4_weight'], self.params['conv4_bias'], self.strides['conv4']))
        x = relu(conv1d(x, self.params['conv5_weight'], self.params['conv5_bias'], self.strides['conv5']))
        x = flatten(x)
        x = relu(linear(x, self.params['fc1_weight'], self.params['fc1_bias']))
        x = relu(linear(x, self.params['fc2_weight'], self.params['fc2_bias']))
        x = relu(linear(x, self.params['fc3_weight'], self.params['fc3_bias']))
        return tanh(linear(x, self.params['fc4_weight'], self.params['fc4_bias']))


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
        self.max_speed_mps = float(max_speed_mps)
        self.max_abs_delta_rad = float(max_abs_delta_rad)
        self.base = TinyLidarNetNp(input_dim=self.input_dim, output_dim=2)
        self.strides = self.base.strides
        self.spatial_dim = self.base.shapes['fc1_weight'][1]
        adapter_input_dim = self.projection_dim + int(self.use_speed)
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

    def forward_components(self, normalized_scans, speeds_mps=None):
        spatial = self._spatial_features(normalized_scans)
        projected = spatial @ self.params['spatial_projection']
        scale = self.params['spatial_scale']
        if not np.all(np.isfinite(scale)) or np.any(scale <= 0.0):
            raise ValueError("spatial adapter scale must be finite and positive")
        features = (projected - self.params['spatial_mean']) / scale
        if self.use_speed:
            speed = np.asarray(speeds_mps, dtype=np.float32)
            if speed.shape != (len(features),):
                raise ValueError("speed-enabled shadow requires one speed per scan")
            if not np.all(np.isfinite(speed)) or np.any(speed < 0.0):
                raise ValueError("spatial shadow speed must be finite and non-negative")
            normalized_speed = np.clip(
                speed / self.max_speed_mps, 0.0, 1.5
            )[:, None]
            features = np.concatenate((features, normalized_speed), axis=1)
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

    def __call__(self, normalized_scans, speeds_mps=None):
        residual, _, _, _ = self.forward_components(
            normalized_scans, speeds_mps
        )
        return residual
