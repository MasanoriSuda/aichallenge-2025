"""Pure occupancy-grid wall-clearance checks for the Kaleidoscope editor.

The module intentionally has no ROS 2, Tk, NumPy, or OpenCV dependency.  It
does not modify trajectories or map files; adjustment results are detached
candidates that callers must validate and explicitly apply.
"""

from __future__ import annotations

from bisect import bisect_left
from bisect import bisect_right
from collections import deque
from concurrent.futures import CancelledError
from dataclasses import dataclass
from dataclasses import field
from enum import Enum
import hashlib
import math
from pathlib import Path
import re
from typing import Callable, Iterable, Optional, Sequence, Tuple


Point = Tuple[float, float]
Cell = Tuple[int, int]
_EPSILON = 1e-9


class CellState(str, Enum):
    FREE = "free"
    OCCUPIED = "occupied"
    UNKNOWN = "unknown"
    OUTSIDE = "outside"


class Severity(str, Enum):
    ERROR = "error"
    WARNING = "warning"
    INFO = "info"


class AdjustmentStatus(str, Enum):
    NOT_NEEDED = "not_needed"
    FEASIBLE = "feasible"
    INFEASIBLE = "infeasible"


@dataclass(frozen=True)
class MapLoadOptions:
    unknown_is_occupied: bool = True
    # Compatibility name retained for the initial steering design.  Runtime
    # parity removes enclosed OCCUPIED components smaller than this value.
    fill_free_holes_below_cells: int = 5
    runtime_binary_parity: bool = True

    def __post_init__(self) -> None:
        if isinstance(self.fill_free_holes_below_cells, bool):
            raise ValueError("map hole threshold must be an integer")
        if int(self.fill_free_holes_below_cells) != self.fill_free_holes_below_cells:
            raise ValueError("map hole threshold must be an integer")
        if self.fill_free_holes_below_cells < 0:
            raise ValueError("map hole threshold must be non-negative")


@dataclass(frozen=True)
class OccupancyGridSpec:
    yaml_path: Path
    image_path: Path
    resolution_m: float
    origin_x_m: float
    origin_y_m: float
    origin_yaw_rad: float
    negate: bool
    occupied_thresh: float
    free_thresh: float
    unknown_is_occupied: bool
    signature: str


@dataclass(frozen=True)
class OccupancyGrid:
    spec: OccupancyGridSpec
    width: int
    height: int
    cells: Tuple[CellState, ...]
    pixels: Tuple[int, ...]
    max_value: int
    occupied_columns_by_row: Tuple[Tuple[int, ...], ...] = field(
        init=False,
        repr=False,
        compare=False,
    )
    unknown_columns_by_row: Tuple[Tuple[int, ...], ...] = field(
        init=False,
        repr=False,
        compare=False,
    )

    def __post_init__(self) -> None:
        expected = self.width * self.height
        if self.width <= 0 or self.height <= 0:
            raise ValueError("occupancy grid dimensions must be positive")
        if len(self.cells) != expected or len(self.pixels) != expected:
            raise ValueError("occupancy grid raster size does not match dimensions")
        occupied: list[list[int]] = [[] for _ in range(self.height)]
        unknown: list[list[int]] = [[] for _ in range(self.height)]
        for index, state in enumerate(self.cells):
            row, column = divmod(index, self.width)
            if state is CellState.OCCUPIED:
                occupied[row].append(column)
            elif state is CellState.UNKNOWN:
                unknown[row].append(column)
        object.__setattr__(
            self,
            "occupied_columns_by_row",
            tuple(tuple(columns) for columns in occupied),
        )
        object.__setattr__(
            self,
            "unknown_columns_by_row",
            tuple(tuple(columns) for columns in unknown),
        )

    def state(self, row: int, column: int) -> CellState:
        if row < 0 or column < 0 or row >= self.height or column >= self.width:
            return CellState.OUTSIDE
        return self.cells[row * self.width + column]

    def world_to_local(self, x_m: float, y_m: float) -> Point:
        dx = x_m - self.spec.origin_x_m
        dy = y_m - self.spec.origin_y_m
        cosine = math.cos(self.spec.origin_yaw_rad)
        sine = math.sin(self.spec.origin_yaw_rad)
        return cosine * dx + sine * dy, -sine * dx + cosine * dy

    def local_to_world(self, x_m: float, y_m: float) -> Point:
        cosine = math.cos(self.spec.origin_yaw_rad)
        sine = math.sin(self.spec.origin_yaw_rad)
        return (
            self.spec.origin_x_m + cosine * x_m - sine * y_m,
            self.spec.origin_y_m + sine * x_m + cosine * y_m,
        )

    def world_to_cell(self, x_m: float, y_m: float) -> Optional[Cell]:
        local_x, local_y = self.world_to_local(x_m, y_m)
        column = math.floor(local_x / self.spec.resolution_m + 0.5)
        map_y = math.floor(local_y / self.spec.resolution_m + 0.5)
        if column < 0 or map_y < 0 or column >= self.width or map_y >= self.height:
            return None
        return self.height - 1 - map_y, column

    def cell_center_world(self, row: int, column: int) -> Point:
        if self.state(row, column) is CellState.OUTSIDE:
            raise ValueError("cell is outside occupancy grid")
        map_y = self.height - 1 - row
        return self.local_to_world(
            column * self.spec.resolution_m,
            map_y * self.spec.resolution_m,
        )

    def state_at_world(self, x_m: float, y_m: float) -> CellState:
        cell = self.world_to_cell(x_m, y_m)
        return CellState.OUTSIDE if cell is None else self.state(*cell)


@dataclass(frozen=True)
class VehicleFootprintSpec:
    reference_point: str
    wheel_base_m: float
    front_overhang_m: float
    rear_overhang_m: float
    wheel_tread_m: float
    left_overhang_m: float
    right_overhang_m: float
    margin_front_m: float = 0.0
    margin_rear_m: float = 0.0
    margin_left_m: float = 0.0
    margin_right_m: float = 0.0

    def __post_init__(self) -> None:
        if self.reference_point != "rear_axle":
            raise ValueError("only rear_axle trajectory reference is supported")
        values = {
            "wheel_base_m": self.wheel_base_m,
            "front_overhang_m": self.front_overhang_m,
            "rear_overhang_m": self.rear_overhang_m,
            "wheel_tread_m": self.wheel_tread_m,
            "left_overhang_m": self.left_overhang_m,
            "right_overhang_m": self.right_overhang_m,
            "margin_front_m": self.margin_front_m,
            "margin_rear_m": self.margin_rear_m,
            "margin_left_m": self.margin_left_m,
            "margin_right_m": self.margin_right_m,
        }
        for name, value in values.items():
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")
        if self.wheel_base_m <= 0.0:
            raise ValueError("wheel_base_m must be positive")
        if self.wheel_tread_m <= 0.0:
            raise ValueError("wheel_tread_m must be positive")

    @classmethod
    def from_extents(
        cls,
        front_m: float,
        rear_m: float,
        left_m: float,
        right_m: float,
        *,
        reference_point: str = "rear_axle",
        margin_front_m: float = 0.0,
        margin_rear_m: float = 0.0,
        margin_left_m: float = 0.0,
        margin_right_m: float = 0.0,
    ) -> "VehicleFootprintSpec":
        for name, value in (
            ("front_m", front_m),
            ("rear_m", rear_m),
            ("left_m", left_m),
            ("right_m", right_m),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        half_tread = min(left_m, right_m)
        return cls(
            reference_point=reference_point,
            wheel_base_m=front_m,
            front_overhang_m=0.0,
            rear_overhang_m=rear_m,
            wheel_tread_m=2.0 * half_tread,
            left_overhang_m=left_m - half_tread,
            right_overhang_m=right_m - half_tread,
            margin_front_m=margin_front_m,
            margin_rear_m=margin_rear_m,
            margin_left_m=margin_left_m,
            margin_right_m=margin_right_m,
        )

    @property
    def front_extent_m(self) -> float:
        return self.wheel_base_m + self.front_overhang_m

    @property
    def rear_extent_m(self) -> float:
        return self.rear_overhang_m

    @property
    def left_extent_m(self) -> float:
        return 0.5 * self.wheel_tread_m + self.left_overhang_m

    @property
    def right_extent_m(self) -> float:
        return 0.5 * self.wheel_tread_m + self.right_overhang_m

    @property
    def maximum_corner_radius_m(self) -> float:
        return max(
            math.hypot(self.front_extent_m + self.margin_front_m, self.left_extent_m + self.margin_left_m),
            math.hypot(self.front_extent_m + self.margin_front_m, self.right_extent_m + self.margin_right_m),
            math.hypot(self.rear_extent_m + self.margin_rear_m, self.left_extent_m + self.margin_left_m),
            math.hypot(self.rear_extent_m + self.margin_rear_m, self.right_extent_m + self.margin_right_m),
        )


@dataclass(frozen=True)
class Pose2D:
    x_m: float
    y_m: float
    yaw_rad: float
    s_m: Optional[float] = None
    curvature_radpm: Optional[float] = None

    def __post_init__(self) -> None:
        for name, value in (
            ("x_m", self.x_m),
            ("y_m", self.y_m),
            ("yaw_rad", self.yaw_rad),
        ):
            if not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
        for name, value in (
            ("s_m", self.s_m),
            ("curvature_radpm", self.curvature_radpm),
        ):
            if value is not None and not math.isfinite(value):
                raise ValueError(f"{name} must be finite when provided")


@dataclass(frozen=True)
class ValidationOptions:
    circular: bool = False
    sweep_step_m: Optional[float] = None
    include_sweep: bool = True
    calculate_clearance: bool = True

    def __post_init__(self) -> None:
        if self.sweep_step_m is not None and (
            not math.isfinite(self.sweep_step_m) or self.sweep_step_m <= 0.0
        ):
            raise ValueError("sweep_step_m must be finite and positive")


@dataclass(frozen=True)
class ClearanceIssue:
    code: str
    severity: Severity
    message: str
    point_index: Optional[int] = None
    segment_index: Optional[int] = None
    s_m: Optional[float] = None
    clearance_m: Optional[float] = None
    required_margin_m: float = 0.0
    grid_cell: Optional[Cell] = None


@dataclass(frozen=True)
class ClearanceReport:
    map_signature: str
    source_revision: int
    vehicle: VehicleFootprintSpec
    minimum_clearance_m: Optional[float]
    conservative_minimum_clearance_m: Optional[float]
    measurement_resolution_m: float
    colliding_point_count: int
    colliding_segment_count: int
    unknown_contact_count: int
    outside_map_count: int
    issues: Tuple[ClearanceIssue, ...]
    is_safe: bool


@dataclass(frozen=True)
class AdjustmentParameters:
    max_lateral_shift_m: float
    sampling_step_m: float
    displacement_weight: float = 1.0
    smoothness_weight: float = 1.0
    curvature_weight: float = 1.0
    circular: bool = False
    sweep_step_m: Optional[float] = None
    max_abs_curvature_radpm: Optional[float] = 0.70

    def __post_init__(self) -> None:
        for name, value in (
            ("max_lateral_shift_m", self.max_lateral_shift_m),
            ("sampling_step_m", self.sampling_step_m),
            ("displacement_weight", self.displacement_weight),
            ("smoothness_weight", self.smoothness_weight),
            ("curvature_weight", self.curvature_weight),
        ):
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")
        if self.sampling_step_m <= 0.0:
            raise ValueError("sampling_step_m must be positive")
        if self.max_lateral_shift_m > 0.0 and self.sampling_step_m > 2.0 * self.max_lateral_shift_m:
            raise ValueError("sampling_step_m is too large for max_lateral_shift_m")
        if self.sweep_step_m is not None and (
            not math.isfinite(self.sweep_step_m) or self.sweep_step_m <= 0.0
        ):
            raise ValueError("sweep_step_m must be finite and positive")
        if self.max_abs_curvature_radpm is not None and (
            not math.isfinite(self.max_abs_curvature_radpm)
            or self.max_abs_curvature_radpm <= 0.0
        ):
            raise ValueError(
                "max_abs_curvature_radpm must be finite and positive when provided"
            )


@dataclass(frozen=True)
class ClearanceCandidate:
    source_revision: int
    map_signature: str
    vehicle: VehicleFootprintSpec
    parameters: AdjustmentParameters
    poses: Tuple[Pose2D, ...]
    offsets: Tuple[float, ...]
    before_report: ClearanceReport
    after_report: ClearanceReport
    max_shift_m: float


@dataclass(frozen=True)
class AdjustmentResult:
    status: AdjustmentStatus
    before_report: ClearanceReport
    candidate: Optional[ClearanceCandidate] = None
    attempted_report: Optional[ClearanceReport] = None
    infeasible_point_indices: Tuple[int, ...] = ()
    issues: Tuple[ClearanceIssue, ...] = ()


def _finite_float(raw: object, name: str) -> float:
    try:
        value = float(raw)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be numeric") from error
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


def _parse_simple_yaml(path: Path) -> dict[str, object]:
    """Parse the scalar/list subset used by map and ROS parameter YAML files."""

    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError as error:
        raise ValueError(f"could not read YAML {path}: {error}") from error
    result: dict[str, object] = {}
    active_list: Optional[str] = None
    for line_number, raw_line in enumerate(lines, start=1):
        line = raw_line.split("#", 1)[0].strip()
        if not line:
            continue
        if line.startswith("-"):
            if active_list is None:
                continue
            values = result.setdefault(active_list, [])
            assert isinstance(values, list)
            values.append(line[1:].strip())
            continue
        match = re.match(r"^([A-Za-z0-9_/*.-]+)\s*:\s*(.*)$", line)
        if match is None:
            continue
        key, raw_value = match.groups()
        active_list = None
        if not raw_value:
            active_list = key
            result[key] = []
            continue
        if raw_value.startswith("[") and raw_value.endswith("]"):
            result[key] = [part.strip() for part in raw_value[1:-1].split(",")]
        else:
            result[key] = raw_value.strip().strip("\"'")
    return result


def _next_ascii_token(data: bytes, index: int) -> Tuple[bytes, int]:
    length = len(data)
    while index < length:
        if data[index] == ord("#"):
            newline = data.find(b"\n", index)
            index = length if newline < 0 else newline + 1
        elif data[index] in b" \t\r\n\v\f":
            index += 1
        else:
            break
    if index >= length:
        raise ValueError("unexpected end of PGM header")
    start = index
    while index < length and data[index] not in b" \t\r\n\v\f#":
        index += 1
    return data[start:index], index


def _read_pgm(path: Path) -> Tuple[int, int, int, Tuple[int, ...]]:
    try:
        data = path.read_bytes()
    except OSError as error:
        raise ValueError(f"could not read PGM {path}: {error}") from error
    magic, index = _next_ascii_token(data, 0)
    width_token, index = _next_ascii_token(data, index)
    height_token, index = _next_ascii_token(data, index)
    max_token, index = _next_ascii_token(data, index)
    if magic not in {b"P2", b"P5"}:
        raise ValueError("PGM must use P2 or P5 encoding")
    try:
        width = int(width_token)
        height = int(height_token)
        max_value = int(max_token)
    except ValueError as error:
        raise ValueError("PGM dimensions/max value must be integers") from error
    if width <= 0 or height <= 0 or not 0 < max_value <= 65535:
        raise ValueError("PGM dimensions/max value are out of range")
    count = width * height
    if magic == b"P2":
        pixels = []
        for _ in range(count):
            token, index = _next_ascii_token(data, index)
            try:
                pixels.append(int(token))
            except ValueError as error:
                raise ValueError("P2 PGM contains a non-integer pixel") from error
        try:
            _next_ascii_token(data, index)
        except ValueError:
            pass
        else:
            raise ValueError("PGM has more pixels than declared")
    else:
        if index >= len(data) or data[index] not in b" \t\r\n\v\f":
            raise ValueError("P5 PGM header is missing raster separator")
        if data[index:index + 2] == b"\r\n":
            index += 2
        else:
            index += 1
        bytes_per_pixel = 1 if max_value < 256 else 2
        expected_bytes = count * bytes_per_pixel
        raster = data[index:index + expected_bytes]
        if len(raster) != expected_bytes:
            raise ValueError("P5 PGM raster is shorter than declared")
        if len(data) != index + expected_bytes:
            trailing = data[index + expected_bytes:]
            if trailing.strip(b" \t\r\n\v\f"):
                raise ValueError("P5 PGM contains trailing non-whitespace data")
        if bytes_per_pixel == 1:
            pixels = list(raster)
        else:
            pixels = [
                (raster[offset] << 8) | raster[offset + 1]
                for offset in range(0, len(raster), 2)
            ]
    if any(pixel < 0 or pixel > max_value for pixel in pixels):
        raise ValueError("PGM pixel is outside max-value range")
    return width, height, max_value, tuple(pixels)


def _remove_small_occupied_islands(
    cells: list[CellState], width: int, height: int, threshold: int
) -> None:
    if threshold <= 0:
        return
    visited = bytearray(width * height)
    neighbours = tuple(
        (dr, dc)
        for dr in (-1, 0, 1)
        for dc in (-1, 0, 1)
        if dr != 0 or dc != 0
    )
    for start in range(width * height):
        if visited[start] or cells[start] is not CellState.OCCUPIED:
            continue
        visited[start] = 1
        queue = deque([start])
        component: list[int] = []
        touches_border = False
        while queue:
            current = queue.pop()
            component.append(current)
            row, column = divmod(current, width)
            if row in {0, height - 1} or column in {0, width - 1}:
                touches_border = True
            for dr, dc in neighbours:
                nr, nc = row + dr, column + dc
                if nr < 0 or nc < 0 or nr >= height or nc >= width:
                    continue
                neighbour = nr * width + nc
                if visited[neighbour] or cells[neighbour] is not CellState.OCCUPIED:
                    continue
                visited[neighbour] = 1
                queue.append(neighbour)
        if not touches_border and len(component) < threshold:
            for cell_index in component:
                cells[cell_index] = CellState.FREE


def load_occupancy_grid(
    path: Path, options: MapLoadOptions = MapLoadOptions()
) -> OccupancyGrid:
    yaml_path = Path(path).expanduser().resolve(strict=False)
    document = _parse_simple_yaml(yaml_path)
    required = ("image", "resolution", "origin", "negate", "occupied_thresh", "free_thresh")
    missing = [name for name in required if name not in document]
    if missing:
        raise ValueError(f"map YAML is missing: {', '.join(missing)}")
    image_path = Path(str(document["image"]))
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    image_path = image_path.resolve(strict=False)
    resolution = _finite_float(document["resolution"], "resolution")
    if resolution <= 0.0:
        raise ValueError("map resolution must be positive")
    raw_origin = document["origin"]
    if not isinstance(raw_origin, list) or len(raw_origin) != 3:
        raise ValueError("map origin must contain x, y, yaw")
    origin = tuple(_finite_float(value, "origin") for value in raw_origin)
    occupied_thresh = _finite_float(document["occupied_thresh"], "occupied_thresh")
    free_thresh = _finite_float(document["free_thresh"], "free_thresh")
    if not 0.0 <= free_thresh < occupied_thresh <= 1.0:
        raise ValueError("map thresholds must satisfy 0 <= free < occupied <= 1")
    raw_negate = str(document["negate"]).strip().lower()
    if raw_negate not in {"0", "1", "false", "true"}:
        raise ValueError("map negate must be 0/1 or false/true")
    negate = raw_negate in {"1", "true"}
    width, height, max_value, pixels = _read_pgm(image_path)
    # OpenCV runtime normalizes by the maximum value present in the image,
    # rather than solely trusting the PGM header max value.
    normalization_max = max(pixels) if max(pixels) > 1 else 1
    cells: list[CellState] = []
    for pixel in pixels:
        normalized = pixel / normalization_max
        occupancy = normalized if negate else 1.0 - normalized
        if options.runtime_binary_parity and normalized < occupied_thresh:
            # The current C++ MPC treats every pixel below occupied_thresh as
            # a wall, independent of negate/free_thresh.  This branch prevents
            # unknown=free from becoming less conservative than runtime.
            cells.append(CellState.OCCUPIED)
        elif occupancy >= occupied_thresh and not options.runtime_binary_parity:
            cells.append(CellState.OCCUPIED)
        elif occupancy <= free_thresh:
            cells.append(CellState.FREE)
        else:
            cells.append(CellState.UNKNOWN)
    _remove_small_occupied_islands(
        cells,
        width,
        height,
        options.fill_free_holes_below_cells,
    )
    digest = hashlib.sha256()
    digest.update(yaml_path.read_bytes())
    digest.update(b"\0")
    digest.update(image_path.read_bytes())
    digest.update(
        (
            f"|unknown={int(options.unknown_is_occupied)}"
            f"|holes={options.fill_free_holes_below_cells}"
            f"|runtime={int(options.runtime_binary_parity)}"
        ).encode()
    )
    spec = OccupancyGridSpec(
        yaml_path=yaml_path,
        image_path=image_path,
        resolution_m=resolution,
        origin_x_m=origin[0],
        origin_y_m=origin[1],
        origin_yaw_rad=origin[2],
        negate=negate,
        occupied_thresh=occupied_thresh,
        free_thresh=free_thresh,
        unknown_is_occupied=options.unknown_is_occupied,
        signature=digest.hexdigest(),
    )
    return OccupancyGrid(
        spec=spec,
        width=width,
        height=height,
        cells=tuple(cells),
        pixels=pixels,
        max_value=max_value,
    )


def load_vehicle_footprint(
    path: Path,
    *,
    reference_point: str = "rear_axle",
    margin_front_m: float = 0.0,
    margin_rear_m: float = 0.0,
    margin_left_m: float = 0.0,
    margin_right_m: float = 0.0,
) -> VehicleFootprintSpec:
    document = _parse_simple_yaml(Path(path).expanduser())
    names = (
        "wheel_base",
        "front_overhang",
        "rear_overhang",
        "wheel_tread",
        "left_overhang",
        "right_overhang",
    )
    missing = [name for name in names if name not in document]
    if missing:
        raise ValueError(f"vehicle YAML is missing: {', '.join(missing)}")
    values = {name: _finite_float(document[name], name) for name in names}
    return VehicleFootprintSpec(
        reference_point=reference_point,
        wheel_base_m=values["wheel_base"],
        front_overhang_m=values["front_overhang"],
        rear_overhang_m=values["rear_overhang"],
        wheel_tread_m=values["wheel_tread"],
        left_overhang_m=values["left_overhang"],
        right_overhang_m=values["right_overhang"],
        margin_front_m=margin_front_m,
        margin_rear_m=margin_rear_m,
        margin_left_m=margin_left_m,
        margin_right_m=margin_right_m,
    )


def footprint_polygon(
    pose: Pose2D,
    vehicle: VehicleFootprintSpec,
    *,
    include_margin: bool = True,
) -> Tuple[Point, ...]:
    front = vehicle.front_extent_m + (vehicle.margin_front_m if include_margin else 0.0)
    rear = vehicle.rear_extent_m + (vehicle.margin_rear_m if include_margin else 0.0)
    left = vehicle.left_extent_m + (vehicle.margin_left_m if include_margin else 0.0)
    right = vehicle.right_extent_m + (vehicle.margin_right_m if include_margin else 0.0)
    cosine = math.cos(pose.yaw_rad)
    sine = math.sin(pose.yaw_rad)
    local = ((front, left), (-rear, left), (-rear, -right), (front, -right))
    return tuple(
        (
            pose.x_m + cosine * x_value - sine * y_value,
            pose.y_m + sine * x_value + cosine * y_value,
        )
        for x_value, y_value in local
    )


def _point_in_polygon(point: Point, polygon: Sequence[Point]) -> bool:
    x_value, y_value = point
    inside = False
    previous = polygon[-1]
    for current in polygon:
        x1, y1 = previous
        x2, y2 = current
        cross = (x_value - x1) * (y2 - y1) - (y_value - y1) * (x2 - x1)
        if abs(cross) <= _EPSILON and (
            min(x1, x2) - _EPSILON <= x_value <= max(x1, x2) + _EPSILON
            and min(y1, y2) - _EPSILON <= y_value <= max(y1, y2) + _EPSILON
        ):
            return True
        if (y1 > y_value) != (y2 > y_value):
            intersection = x1 + (y_value - y1) * (x2 - x1) / (y2 - y1)
            if x_value <= intersection + _EPSILON:
                inside = not inside
        previous = current
    return inside


def _orientation(a: Point, b: Point, c: Point) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _segments_intersect(a: Point, b: Point, c: Point, d: Point) -> bool:
    o1 = _orientation(a, b, c)
    o2 = _orientation(a, b, d)
    o3 = _orientation(c, d, a)
    o4 = _orientation(c, d, b)
    if ((o1 > _EPSILON and o2 < -_EPSILON) or (o1 < -_EPSILON and o2 > _EPSILON)) and (
        (o3 > _EPSILON and o4 < -_EPSILON) or (o3 < -_EPSILON and o4 > _EPSILON)
    ):
        return True
    for value, first, second, point in (
        (o1, a, b, c),
        (o2, a, b, d),
        (o3, c, d, a),
        (o4, c, d, b),
    ):
        if abs(value) <= _EPSILON and (
            min(first[0], second[0]) - _EPSILON <= point[0] <= max(first[0], second[0]) + _EPSILON
            and min(first[1], second[1]) - _EPSILON <= point[1] <= max(first[1], second[1]) + _EPSILON
        ):
            return True
    return False


def _polygon_intersects_rectangle(
    polygon: Sequence[Point], left: float, right: float, bottom: float, top: float
) -> bool:
    if not polygon:
        return False
    polygon_min_x = min(point[0] for point in polygon)
    polygon_max_x = max(point[0] for point in polygon)
    polygon_min_y = min(point[1] for point in polygon)
    polygon_max_y = max(point[1] for point in polygon)
    if (
        polygon_max_x < left - _EPSILON
        or polygon_min_x > right + _EPSILON
        or polygon_max_y < bottom - _EPSILON
        or polygon_min_y > top + _EPSILON
    ):
        return False

    center_x = 0.5 * (left + right)
    center_y = 0.5 * (bottom + top)
    half_width = 0.5 * (right - left)
    half_height = 0.5 * (top - bottom)
    previous = polygon[-1]
    for current in polygon:
        edge_x = current[0] - previous[0]
        edge_y = current[1] - previous[1]
        axis_x = -edge_y
        axis_y = edge_x
        if abs(axis_x) <= _EPSILON and abs(axis_y) <= _EPSILON:
            previous = current
            continue
        projections = [
            point[0] * axis_x + point[1] * axis_y for point in polygon
        ]
        rectangle_center = center_x * axis_x + center_y * axis_y
        rectangle_radius = (
            half_width * abs(axis_x) + half_height * abs(axis_y)
        )
        if (
            max(projections) < rectangle_center - rectangle_radius - _EPSILON
            or min(projections) > rectangle_center + rectangle_radius + _EPSILON
        ):
            return False
        previous = current
    return True


def _local_polygon(grid: OccupancyGrid, polygon: Sequence[Point]) -> Tuple[Point, ...]:
    return tuple(grid.world_to_local(*point) for point in polygon)


def _polygon_contacts(
    grid: OccupancyGrid,
    world_polygon: Sequence[Point],
    *,
    inflation_m: float = 0.0,
) -> Tuple[set[Cell], set[Cell], bool]:
    if not math.isfinite(inflation_m) or inflation_m < 0.0:
        raise ValueError("polygon inflation must be finite and non-negative")
    polygon = _local_polygon(grid, world_polygon)
    resolution = grid.spec.resolution_m
    min_x = min(point[0] for point in polygon)
    max_x = max(point[0] for point in polygon)
    min_y = min(point[1] for point in polygon)
    max_y = max(point[1] for point in polygon)
    outside = (
        min_x - inflation_m < -0.5 * resolution - _EPSILON
        or min_y - inflation_m < -0.5 * resolution - _EPSILON
        or max_x + inflation_m > (grid.width - 0.5) * resolution + _EPSILON
        or max_y + inflation_m > (grid.height - 0.5) * resolution + _EPSILON
    )
    min_column = max(0, math.floor((min_x - inflation_m) / resolution - 0.5))
    max_column = min(
        grid.width - 1,
        math.ceil((max_x + inflation_m) / resolution + 0.5),
    )
    min_map_y = max(0, math.floor((min_y - inflation_m) / resolution - 0.5))
    max_map_y = min(
        grid.height - 1,
        math.ceil((max_y + inflation_m) / resolution + 0.5),
    )
    occupied: set[Cell] = set()
    unknown: set[Cell] = set()
    for map_y in range(min_map_y, max_map_y + 1):
        row = grid.height - 1 - map_y
        bottom = (map_y - 0.5) * resolution
        top = (map_y + 0.5) * resolution
        for columns, contacts in (
            (grid.occupied_columns_by_row[row], occupied),
            (grid.unknown_columns_by_row[row], unknown),
        ):
            start = bisect_left(columns, min_column)
            end = bisect_right(columns, max_column)
            for position in range(start, end):
                column = columns[position]
                left = (column - 0.5) * resolution
                right = (column + 0.5) * resolution
                if _polygon_intersects_rectangle(
                    polygon,
                    left - inflation_m,
                    right + inflation_m,
                    bottom - inflation_m,
                    top + inflation_m,
                ):
                    contacts.add((row, column))
    return occupied, unknown, outside


def _unsafe_chebyshev_distance(grid: OccupancyGrid) -> Tuple[int, ...]:
    """Return a conservative cell-distance lower bound to unsafe map cells."""

    distances = [-1] * (grid.width * grid.height)
    queue: deque[int] = deque()
    for index, state in enumerate(grid.cells):
        unsafe = state is CellState.OCCUPIED or (
            state is CellState.UNKNOWN and grid.spec.unknown_is_occupied
        )
        if unsafe:
            distances[index] = 0
            queue.append(index)
    neighbours = tuple(
        (dr, dc)
        for dr in (-1, 0, 1)
        for dc in (-1, 0, 1)
        if dr != 0 or dc != 0
    )
    while queue:
        current = queue.popleft()
        row, column = divmod(current, grid.width)
        next_distance = distances[current] + 1
        for dr, dc in neighbours:
            nr, nc = row + dr, column + dc
            if nr < 0 or nc < 0 or nr >= grid.height or nc >= grid.width:
                continue
            neighbour = nr * grid.width + nc
            if distances[neighbour] >= 0:
                continue
            distances[neighbour] = next_distance
            queue.append(neighbour)
    return tuple(distances)


def _conservative_field_clearance(
    grid: OccupancyGrid,
    world_polygon: Sequence[Point],
    distance_steps: Sequence[int],
    *,
    inflation_m: float = 0.0,
) -> Optional[float]:
    polygon = _local_polygon(grid, world_polygon)
    resolution = grid.spec.resolution_m
    min_map_y = max(
        0,
        math.floor(min(point[1] for point in polygon) / resolution - 0.5),
    )
    max_map_y = min(
        grid.height - 1,
        math.ceil(max(point[1] for point in polygon) / resolution + 0.5),
    )
    minimum_steps: Optional[int] = None
    edges = tuple(zip(polygon, (*polygon[1:], polygon[0])))
    # For each image row, project the convex polygon portion inside that
    # cell-height strip onto X.  The resulting column interval is a tight
    # superset of polygon-covered cells, retaining a conservative lower bound
    # without running a polygon/rectangle SAT for every free cell.
    for map_y in range(min_map_y, max_map_y + 1):
        bottom = (map_y - 0.5) * resolution
        top = (map_y + 0.5) * resolution
        x_values = [
            x
            for x, y in polygon
            if bottom - _EPSILON <= y <= top + _EPSILON
        ]
        for first, second in edges:
            dy = second[1] - first[1]
            if abs(dy) <= _EPSILON:
                continue
            for boundary in (bottom, top):
                factor = (boundary - first[1]) / dy
                if -_EPSILON <= factor <= 1.0 + _EPSILON:
                    x_values.append(
                        first[0] + factor * (second[0] - first[0])
                    )
        if not x_values:
            continue
        min_column = max(
            0,
            math.ceil(min(x_values) / resolution - 0.5 - _EPSILON),
        )
        max_column = min(
            grid.width - 1,
            math.floor(max(x_values) / resolution + 0.5 + _EPSILON),
        )
        row = grid.height - 1 - map_y
        base = row * grid.width
        for column in range(min_column, max_column + 1):
            steps = distance_steps[base + column]
            if steps >= 0 and (minimum_steps is None or steps < minimum_steps):
                minimum_steps = steps
                if minimum_steps == 0:
                    break
        if minimum_steps == 0:
            break
    if minimum_steps is None:
        return None
    # Chebyshev steps never exceed Euclidean center distance.  Subtract both
    # cell half-diagonals and the swept-arc dilation to retain a lower bound.
    return max(
        0.0,
        minimum_steps * grid.spec.resolution_m
        - math.sqrt(2.0) * grid.spec.resolution_m
        - inflation_m,
    )


def _distance_point_segment(point: Point, start: Point, end: Point) -> float:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    denominator = dx * dx + dy * dy
    if denominator <= _EPSILON:
        return math.hypot(point[0] - start[0], point[1] - start[1])
    factor = max(0.0, min(1.0, ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / denominator))
    return math.hypot(point[0] - (start[0] + factor * dx), point[1] - (start[1] + factor * dy))


def _polygon_rectangle_distance(
    polygon: Sequence[Point], left: float, right: float, bottom: float, top: float
) -> float:
    if _polygon_intersects_rectangle(polygon, left, right, bottom, top):
        return 0.0
    rectangle = ((left, bottom), (right, bottom), (right, top), (left, top))
    polygon_edges = tuple(zip(polygon, (*polygon[1:], polygon[0])))
    rectangle_edges = tuple(zip(rectangle, (*rectangle[1:], rectangle[0])))
    polygon_to_rectangle = min(
        min(_distance_point_segment(point, start, end) for start, end in rectangle_edges)
        for point in polygon
    )
    rectangle_to_polygon = min(
        min(_distance_point_segment(point, start, end) for start, end in polygon_edges)
        for point in rectangle
    )
    return min(polygon_to_rectangle, rectangle_to_polygon)


def _minimum_clearance(
    grid: OccupancyGrid,
    world_polygon: Sequence[Point],
    *,
    maximum_search_m: float = 5.0,
    inflation_m: float = 0.0,
) -> Optional[float]:
    polygon = _local_polygon(grid, world_polygon)
    resolution = grid.spec.resolution_m
    min_column = math.floor(min(point[0] for point in polygon) / resolution)
    max_column = math.ceil(max(point[0] for point in polygon) / resolution)
    min_map_y = math.floor(min(point[1] for point in polygon) / resolution)
    max_map_y = math.ceil(max(point[1] for point in polygon) / resolution)
    maximum_ring = max(1, math.ceil(maximum_search_m / resolution))
    best = math.inf
    for ring in range(maximum_ring + 1):
        low_column = min_column - ring
        high_column = max_column + ring
        low_y = min_map_y - ring
        high_y = max_map_y + ring
        candidates: set[Tuple[int, int]] = set()
        if ring == 0:
            candidates.update(
                (map_y, column)
                for map_y in range(low_y, high_y + 1)
                for column in range(low_column, high_column + 1)
            )
        else:
            for column in range(low_column, high_column + 1):
                candidates.add((low_y, column))
                candidates.add((high_y, column))
            for map_y in range(low_y + 1, high_y):
                candidates.add((map_y, low_column))
                candidates.add((map_y, high_column))
        for map_y, column in candidates:
            if column < 0 or map_y < 0 or column >= grid.width or map_y >= grid.height:
                continue
            row = grid.height - 1 - map_y
            state = grid.state(row, column)
            unsafe = state is CellState.OCCUPIED or (
                state is CellState.UNKNOWN and grid.spec.unknown_is_occupied
            )
            if not unsafe:
                continue
            distance = _polygon_rectangle_distance(
                polygon,
                (column - 0.5) * resolution,
                (column + 0.5) * resolution,
                (map_y - 0.5) * resolution,
                (map_y + 0.5) * resolution,
            )
            best = min(best, max(0.0, distance - inflation_m))
        if math.isfinite(best) and ring * resolution > (
            best + inflation_m + math.sqrt(2.0) * resolution
        ):
            break
    return best if math.isfinite(best) else None


@dataclass(frozen=True)
class _PoseEvaluation:
    issues: Tuple[ClearanceIssue, ...]
    minimum_clearance_m: Optional[float]
    collides: bool
    unknown_contact: bool
    outside: bool


def _convex_hull(points: Iterable[Point]) -> Tuple[Point, ...]:
    unique = sorted(set(points))
    if len(unique) <= 2:
        return tuple(unique)

    def cross(origin: Point, first: Point, second: Point) -> float:
        return (
            (first[0] - origin[0]) * (second[1] - origin[1])
            - (first[1] - origin[1]) * (second[0] - origin[0])
        )

    lower: list[Point] = []
    for point in unique:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0.0:
            lower.pop()
        lower.append(point)
    upper: list[Point] = []
    for point in reversed(unique):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0.0:
            upper.pop()
        upper.append(point)
    return tuple(lower[:-1] + upper[:-1])


def _corner_radius(vehicle: VehicleFootprintSpec, include_margin: bool) -> float:
    front = vehicle.front_extent_m + (
        vehicle.margin_front_m if include_margin else 0.0
    )
    rear = vehicle.rear_extent_m + (
        vehicle.margin_rear_m if include_margin else 0.0
    )
    left = vehicle.left_extent_m + (
        vehicle.margin_left_m if include_margin else 0.0
    )
    right = vehicle.right_extent_m + (
        vehicle.margin_right_m if include_margin else 0.0
    )
    return max(
        math.hypot(front, left),
        math.hypot(front, right),
        math.hypot(rear, left),
        math.hypot(rear, right),
    )


def _has_clearance_margin(vehicle: VehicleFootprintSpec) -> bool:
    return any(
        margin > _EPSILON
        for margin in (
            vehicle.margin_front_m,
            vehicle.margin_rear_m,
            vehicle.margin_left_m,
            vehicle.margin_right_m,
        )
    )


def _evaluate_swept_interval(
    grid: OccupancyGrid,
    first: Pose2D,
    second: Pose2D,
    vehicle: VehicleFootprintSpec,
    *,
    segment_index: int,
    calculate_clearance: bool = True,
    distance_steps: Optional[Sequence[int]] = None,
) -> _PoseEvaluation:
    body = _convex_hull(
        (
            *footprint_polygon(first, vehicle, include_margin=False),
            *footprint_polygon(second, vehicle, include_margin=False),
        )
    )
    has_margin = _has_clearance_margin(vehicle)
    envelope = (
        _convex_hull(
            (
                *footprint_polygon(first, vehicle, include_margin=True),
                *footprint_polygon(second, vehicle, include_margin=True),
            )
        )
        if has_margin
        else body
    )
    yaw_delta = abs(_wrap_angle(second.yaw_rad - first.yaw_rad))
    # The translated corner chord is contained in the endpoint convex hull.
    # This rotation bound dilates that hull enough to contain the corner arc.
    body_inflation = 2.0 * _corner_radius(vehicle, False) * math.sin(0.5 * yaw_delta)
    envelope_inflation = (
        2.0 * _corner_radius(vehicle, True) * math.sin(0.5 * yaw_delta)
        if has_margin
        else body_inflation
    )
    body_occupied, body_unknown, body_outside = _polygon_contacts(
        grid,
        body,
        inflation_m=body_inflation,
    )
    if has_margin:
        envelope_occupied, envelope_unknown, envelope_outside = _polygon_contacts(
            grid,
            envelope,
            inflation_m=envelope_inflation,
        )
    else:
        envelope_occupied = body_occupied
        envelope_unknown = body_unknown
        envelope_outside = body_outside
    issues: list[ClearanceIssue] = []
    if body_occupied:
        issues.append(
            ClearanceIssue(
                code="SWEPT_FOOTPRINT_COLLISION",
                severity=Severity.ERROR,
                message="conservative swept vehicle body intersects an occupied map cell",
                segment_index=segment_index,
                clearance_m=0.0,
                grid_cell=min(body_occupied),
            )
        )
    if envelope_unknown:
        issues.append(
            ClearanceIssue(
                code="UNKNOWN_CELL_CONTACT",
                severity=(
                    Severity.ERROR
                    if grid.spec.unknown_is_occupied
                    else Severity.WARNING
                ),
                message=(
                    "conservative swept envelope intersects an unknown cell treated as occupied"
                    if grid.spec.unknown_is_occupied
                    else "conservative swept envelope intersects an unknown cell explicitly treated as free"
                ),
                segment_index=segment_index,
                grid_cell=min(envelope_unknown),
            )
        )
    outside = body_outside or envelope_outside
    if outside:
        issues.append(
            ClearanceIssue(
                code="FOOTPRINT_OUTSIDE_MAP",
                severity=Severity.ERROR,
                message="conservative swept envelope extends outside the occupancy grid",
                segment_index=segment_index,
            )
        )
    margin_only = envelope_occupied - body_occupied
    if margin_only:
        issues.append(
            ClearanceIssue(
                code="CLEARANCE_MARGIN_VIOLATION",
                severity=Severity.ERROR,
                message="conservative swept body is clear but its requested margin intersects a wall",
                segment_index=segment_index,
                required_margin_m=max(
                    vehicle.margin_front_m,
                    vehicle.margin_rear_m,
                    vehicle.margin_left_m,
                    vehicle.margin_right_m,
                ),
                grid_cell=min(margin_only),
            )
        )
    body_is_unsafe = bool(body_occupied) or (
        bool(body_unknown) and grid.spec.unknown_is_occupied
    )
    minimum = None
    if calculate_clearance:
        if body_is_unsafe:
            minimum = 0.0
        elif distance_steps is not None:
            minimum = _conservative_field_clearance(
                grid,
                body,
                distance_steps,
                inflation_m=body_inflation,
            )
        else:
            minimum = _minimum_clearance(
                grid,
                body,
                inflation_m=body_inflation,
            )
    return _PoseEvaluation(
        issues=tuple(issues),
        minimum_clearance_m=minimum,
        collides=bool(body_occupied or margin_only),
        unknown_contact=bool(envelope_unknown),
        outside=outside,
    )


def _evaluate_pose(
    grid: OccupancyGrid,
    pose: Pose2D,
    vehicle: VehicleFootprintSpec,
    *,
    point_index: Optional[int],
    segment_index: Optional[int],
    swept: bool,
    calculate_clearance: bool,
) -> _PoseEvaluation:
    body = footprint_polygon(pose, vehicle, include_margin=False)
    has_margin = _has_clearance_margin(vehicle)
    envelope = (
        footprint_polygon(pose, vehicle, include_margin=True)
        if has_margin
        else body
    )
    body_occupied, body_unknown, body_outside = _polygon_contacts(grid, body)
    if has_margin:
        envelope_occupied, envelope_unknown, envelope_outside = _polygon_contacts(
            grid, envelope
        )
    else:
        envelope_occupied = body_occupied
        envelope_unknown = body_unknown
        envelope_outside = body_outside
    issues: list[ClearanceIssue] = []
    location = {
        "point_index": point_index,
        "segment_index": segment_index,
        "s_m": pose.s_m,
    }
    collision_cells = body_occupied
    if collision_cells:
        issues.append(
            ClearanceIssue(
                code="SWEPT_FOOTPRINT_COLLISION" if swept else "FOOTPRINT_COLLISION",
                severity=Severity.ERROR,
                message="vehicle body intersects an occupied map cell",
                clearance_m=0.0,
                grid_cell=min(collision_cells),
                **location,
            )
        )
    unknown_cells = envelope_unknown
    if unknown_cells:
        issues.append(
            ClearanceIssue(
                code="UNKNOWN_CELL_CONTACT",
                severity=Severity.ERROR if grid.spec.unknown_is_occupied else Severity.WARNING,
                message=(
                    "vehicle envelope intersects an unknown cell treated as occupied"
                    if grid.spec.unknown_is_occupied
                    else "vehicle envelope intersects an unknown cell explicitly treated as free"
                ),
                grid_cell=min(unknown_cells),
                **location,
            )
        )
    outside = body_outside or envelope_outside
    if outside:
        issues.append(
            ClearanceIssue(
                code="FOOTPRINT_OUTSIDE_MAP",
                severity=Severity.ERROR,
                message="vehicle envelope extends outside the occupancy grid",
                **location,
            )
        )
    margin_only = envelope_occupied - body_occupied
    if margin_only:
        issues.append(
            ClearanceIssue(
                code="CLEARANCE_MARGIN_VIOLATION",
                severity=Severity.ERROR,
                message="vehicle body is clear but the requested margin intersects a wall",
                required_margin_m=max(
                    vehicle.margin_front_m,
                    vehicle.margin_rear_m,
                    vehicle.margin_left_m,
                    vehicle.margin_right_m,
                ),
                grid_cell=min(margin_only),
                **location,
            )
        )
    body_is_unsafe = bool(body_occupied) or (
        bool(body_unknown) and grid.spec.unknown_is_occupied
    )
    minimum = (
        0.0
        if calculate_clearance and body_is_unsafe
        else _minimum_clearance(grid, body) if calculate_clearance else None
    )
    return _PoseEvaluation(
        issues=tuple(issues),
        minimum_clearance_m=minimum,
        collides=bool(collision_cells or margin_only),
        unknown_contact=bool(unknown_cells),
        outside=outside,
    )


def _wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _interpolate_pose(first: Pose2D, second: Pose2D, factor: float) -> Pose2D:
    yaw = first.yaw_rad + factor * _wrap_angle(second.yaw_rad - first.yaw_rad)
    s_m = None
    if first.s_m is not None and second.s_m is not None:
        s_m = first.s_m + factor * (second.s_m - first.s_m)
    return Pose2D(
        x_m=first.x_m + factor * (second.x_m - first.x_m),
        y_m=first.y_m + factor * (second.y_m - first.y_m),
        yaw_rad=_wrap_angle(yaw),
        s_m=s_m,
    )


def _segment_pairs(poses: Sequence[Pose2D], circular: bool) -> Tuple[Tuple[int, int], ...]:
    pairs = [(index, index + 1) for index in range(len(poses) - 1)]
    if circular and len(poses) > 2:
        duplicate_pose = (
            math.hypot(
                poses[-1].x_m - poses[0].x_m,
                poses[-1].y_m - poses[0].y_m,
            )
            <= 1e-6
            and abs(_wrap_angle(poses[-1].yaw_rad - poses[0].yaw_rad)) <= 1e-6
        )
        if not duplicate_pose:
            pairs.append((len(poses) - 1, 0))
    return tuple(pairs)


def _sweep_subdivisions(
    first: Pose2D,
    second: Pose2D,
    vehicle: VehicleFootprintSpec,
    step_m: float,
) -> int:
    translation = math.hypot(second.x_m - first.x_m, second.y_m - first.y_m)
    rotation_motion = vehicle.maximum_corner_radius_m * abs(
        _wrap_angle(second.yaw_rad - first.yaw_rad)
    )
    # Translation and rotation can move a corner in the same direction.
    return max(1, math.ceil((translation + rotation_motion) / step_m))


def validate_clearance(
    grid: OccupancyGrid,
    poses: Sequence[Pose2D],
    vehicle: VehicleFootprintSpec,
    options: ValidationOptions = ValidationOptions(),
    *,
    source_revision: int = 0,
) -> ClearanceReport:
    if len(poses) < 2:
        raise ValueError("clearance validation requires at least two poses")
    issues: list[ClearanceIssue] = []
    minimum_values: list[float] = []
    colliding_points: set[int] = set()
    colliding_segments: set[int] = set()
    unknown_contacts = 0
    outside_contacts = 0
    sweep_conservative_values: list[float] = []
    for index, pose in enumerate(poses):
        evaluation = _evaluate_pose(
            grid,
            pose,
            vehicle,
            point_index=index,
            segment_index=None,
            swept=False,
            calculate_clearance=options.calculate_clearance,
        )
        issues.extend(evaluation.issues)
        if evaluation.minimum_clearance_m is not None:
            minimum_values.append(evaluation.minimum_clearance_m)
        if evaluation.collides:
            colliding_points.add(index)
        if evaluation.unknown_contact:
            unknown_contacts += 1
        if evaluation.outside:
            outside_contacts += 1
    if options.include_sweep:
        distance_steps = (
            _unsafe_chebyshev_distance(grid)
            if options.calculate_clearance
            else None
        )
        step_m = options.sweep_step_m or max(0.01, 0.5 * grid.spec.resolution_m)
        for first_index, second_index in _segment_pairs(poses, options.circular):
            first = poses[first_index]
            second = poses[second_index]
            subdivisions = _sweep_subdivisions(first, second, vehicle, step_m)
            segment_issues: list[ClearanceIssue] = []
            seen_issue_codes: set[str] = set()
            segment_collides = False
            segment_unknown = False
            segment_outside = False
            for subdivision in range(subdivisions):
                interval_first = _interpolate_pose(
                    first,
                    second,
                    subdivision / subdivisions,
                )
                interval_second = _interpolate_pose(
                    first,
                    second,
                    (subdivision + 1) / subdivisions,
                )
                evaluation = _evaluate_swept_interval(
                    grid,
                    interval_first,
                    interval_second,
                    vehicle,
                    segment_index=first_index,
                    calculate_clearance=options.calculate_clearance,
                    distance_steps=distance_steps,
                )
                if evaluation.minimum_clearance_m is not None:
                    sweep_conservative_values.append(
                        evaluation.minimum_clearance_m
                    )
                for issue in evaluation.issues:
                    if issue.code not in seen_issue_codes:
                        seen_issue_codes.add(issue.code)
                        segment_issues.append(issue)
                segment_collides = segment_collides or evaluation.collides
                segment_unknown = segment_unknown or evaluation.unknown_contact
                segment_outside = segment_outside or evaluation.outside
            issues.extend(segment_issues)
            if segment_collides:
                colliding_segments.add(first_index)
            if segment_unknown:
                unknown_contacts += 1
            if segment_outside:
                outside_contacts += 1
    minimum = min(minimum_values) if minimum_values else None
    conservative_candidates = list(sweep_conservative_values)
    if minimum is not None:
        conservative_candidates.append(
            max(0.0, minimum - grid.spec.resolution_m / math.sqrt(2.0))
        )
    conservative = (
        min(conservative_candidates) if conservative_candidates else None
    )
    issues.sort(
        key=lambda issue: (
            0 if issue.severity is Severity.ERROR else 1,
            issue.point_index if issue.point_index is not None else 1 << 60,
            issue.segment_index if issue.segment_index is not None else 1 << 60,
            issue.code,
        )
    )
    return ClearanceReport(
        map_signature=grid.spec.signature,
        source_revision=source_revision,
        vehicle=vehicle,
        minimum_clearance_m=minimum,
        conservative_minimum_clearance_m=conservative,
        measurement_resolution_m=grid.spec.resolution_m,
        colliding_point_count=len(colliding_points),
        colliding_segment_count=len(colliding_segments),
        unknown_contact_count=unknown_contacts,
        outside_map_count=outside_contacts,
        issues=tuple(issues),
        is_safe=not any(issue.severity is Severity.ERROR for issue in issues),
    )


def _pose_is_safe(
    grid: OccupancyGrid, pose: Pose2D, vehicle: VehicleFootprintSpec
) -> bool:
    evaluation = _evaluate_pose(
        grid,
        pose,
        vehicle,
        point_index=None,
        segment_index=None,
        swept=False,
        calculate_clearance=False,
    )
    return not any(issue.severity is Severity.ERROR for issue in evaluation.issues)


def _transition_is_safe(
    grid: OccupancyGrid,
    first: Pose2D,
    second: Pose2D,
    vehicle: VehicleFootprintSpec,
    step_m: float,
) -> bool:
    subdivisions = _sweep_subdivisions(first, second, vehicle, step_m)
    coarse = _evaluate_swept_interval(
        grid,
        first,
        second,
        vehicle,
        segment_index=0,
        calculate_clearance=False,
    )
    if not any(issue.severity is Severity.ERROR for issue in coarse.issues):
        # The whole-segment conservative hull contains every refined sweep;
        # a clear broad phase therefore proves the transition safe in one test.
        return True
    if subdivisions == 1:
        return False
    for subdivision in range(subdivisions):
        interval_first = _interpolate_pose(
            first,
            second,
            subdivision / subdivisions,
        )
        interval_second = _interpolate_pose(
            first,
            second,
            (subdivision + 1) / subdivisions,
        )
        evaluation = _evaluate_swept_interval(
            grid,
            interval_first,
            interval_second,
            vehicle,
            segment_index=0,
            calculate_clearance=False,
        )
        if any(issue.severity is Severity.ERROR for issue in evaluation.issues):
            return False
    return True


def _offset_values(parameters: AdjustmentParameters) -> Tuple[float, ...]:
    if parameters.max_lateral_shift_m <= _EPSILON:
        return (0.0,)
    count = math.floor(parameters.max_lateral_shift_m / parameters.sampling_step_m + _EPSILON)
    values = {
        0.0,
        -parameters.max_lateral_shift_m,
        parameters.max_lateral_shift_m,
    }
    values.update(
        index * parameters.sampling_step_m
        for index in range(-count, count + 1)
    )
    return tuple(sorted(value for value in values if abs(value) <= parameters.max_lateral_shift_m + _EPSILON))


def _offset_pose(pose: Pose2D, offset_m: float) -> Pose2D:
    return Pose2D(
        x_m=pose.x_m - math.sin(pose.yaw_rad) * offset_m,
        y_m=pose.y_m + math.cos(pose.yaw_rad) * offset_m,
        yaw_rad=pose.yaw_rad,
        s_m=pose.s_m,
        curvature_radpm=pose.curvature_radpm,
    )


def _rank_offset_paths(
    feasible: Sequence[Tuple[float, ...]],
    parameters: AdjustmentParameters,
    *,
    transition_safe: Optional[Callable[[int, float, float], bool]] = None,
    beam_per_endpoint: int = 3,
    maximum_paths: int = 12,
) -> Tuple[Tuple[float, ...], ...]:
    if not feasible or any(not values for values in feasible):
        return ()
    starts = (
        tuple(
            sorted(
                feasible[0],
                key=lambda value: (abs(value), value),
            )[:beam_per_endpoint]
        )
        if parameters.circular
        else (None,)
    )
    completed: list[Tuple[float, Tuple[float, ...]]] = []
    for fixed_start in starts:
        initial_values = (
            (fixed_start,) if fixed_start is not None else feasible[0]
        )
        states: dict[float, list[Tuple[float, Tuple[float, ...]]]] = {
            value: [(parameters.displacement_weight * value * value, (value,))]
            for value in initial_values
        }
        for index, candidates in enumerate(feasible[1:], start=1):
            next_states: dict[float, list[Tuple[float, Tuple[float, ...]]]] = {}
            for current in candidates:
                ranked_for_endpoint: list[
                    Tuple[float, Tuple[float, ...], float]
                ] = []
                for previous, previous_states in states.items():
                    for cost, path in previous_states:
                        second_difference = (
                            current - 2.0 * previous + path[-2]
                            if len(path) >= 2
                            else 0.0
                        )
                        candidate_cost = (
                            cost
                            + parameters.displacement_weight * current * current
                            + parameters.smoothness_weight * (current - previous) ** 2
                            + parameters.curvature_weight * second_difference**2
                        )
                        ranked_for_endpoint.append(
                            (candidate_cost, (*path, current), previous)
                        )
                ranked_for_endpoint.sort(
                    key=lambda item: (
                        item[0],
                        tuple(abs(value) for value in item[1]),
                        item[1],
                    )
                )
                candidates_for_endpoint: list[
                    Tuple[float, Tuple[float, ...]]
                ] = []
                for cost, path, previous in ranked_for_endpoint:
                    # Transition safety depends only on the endpoint offsets.
                    # Evaluate it lazily in objective order and stop once the
                    # exact beam quota is filled; checking paths that would be
                    # pruned made real-course adjustment needlessly quadratic.
                    if transition_safe is not None and not transition_safe(
                        index - 1,
                        previous,
                        current,
                    ):
                        continue
                    candidates_for_endpoint.append((cost, path))
                    if len(candidates_for_endpoint) >= beam_per_endpoint:
                        break
                if candidates_for_endpoint:
                    next_states[current] = candidates_for_endpoint
            states = next_states
            if not states:
                break
        for last, endpoint_states in states.items():
            for cost, path in endpoint_states:
                total = cost
                if parameters.circular:
                    assert fixed_start is not None
                    if transition_safe is not None and not transition_safe(
                        len(feasible) - 1,
                        last,
                        fixed_start,
                    ):
                        continue
                    total += parameters.smoothness_weight * (last - fixed_start) ** 2
                    if len(path) >= 3:
                        total += parameters.curvature_weight * (
                            fixed_start - 2.0 * last + path[-2]
                        ) ** 2
                        total += parameters.curvature_weight * (
                            path[1] - 2.0 * fixed_start + last
                        ) ** 2
                completed.append((total, path))
    completed.sort(
        key=lambda item: (
            item[0],
            tuple(abs(value) for value in item[1]),
            item[1],
        )
    )
    ranked: list[Tuple[float, ...]] = []
    seen: set[Tuple[float, ...]] = set()
    for _cost, path in completed:
        if path in seen:
            continue
        seen.add(path)
        ranked.append(path)
        if len(ranked) >= maximum_paths:
            break
    return tuple(ranked)


def _poses_from_points(points: Sequence[Point], circular: bool) -> Tuple[Pose2D, ...]:
    count = len(points)
    if count < 2:
        raise ValueError("adjusted path requires at least two points")
    duplicate = circular and math.hypot(points[-1][0] - points[0][0], points[-1][1] - points[0][1]) <= 1e-6
    unique_count = count - 1 if duplicate else count
    s_values = [0.0] * count
    for index in range(1, count):
        s_values[index] = s_values[index - 1] + math.hypot(
            points[index][0] - points[index - 1][0],
            points[index][1] - points[index - 1][1],
        )
    yaws = [0.0] * count
    curvatures = [0.0] * count
    for index in range(unique_count):
        previous = (index - 1) % unique_count if circular else max(0, index - 1)
        following = (index + 1) % unique_count if circular else min(unique_count - 1, index + 1)
        if previous == index:
            dx = points[following][0] - points[index][0]
            dy = points[following][1] - points[index][1]
        elif following == index:
            dx = points[index][0] - points[previous][0]
            dy = points[index][1] - points[previous][1]
        else:
            dx = points[following][0] - points[previous][0]
            dy = points[following][1] - points[previous][1]
        if math.hypot(dx, dy) > _EPSILON:
            yaws[index] = math.atan2(dy, dx)
        first_length = math.hypot(
            points[index][0] - points[previous][0],
            points[index][1] - points[previous][1],
        )
        second_length = math.hypot(
            points[following][0] - points[index][0],
            points[following][1] - points[index][1],
        )
        if first_length > 1e-6 and second_length > 1e-6:
            first_heading = math.atan2(
                points[index][1] - points[previous][1],
                points[index][0] - points[previous][0],
            )
            second_heading = math.atan2(
                points[following][1] - points[index][1],
                points[following][0] - points[index][0],
            )
            curvatures[index] = _wrap_angle(second_heading - first_heading) / (
                0.5 * (first_length + second_length)
            )
    if duplicate:
        yaws[-1] = yaws[0]
        curvatures[-1] = curvatures[0]
    return tuple(
        Pose2D(
            x_m=point[0],
            y_m=point[1],
            yaw_rad=yaws[index],
            s_m=s_values[index],
            curvature_radpm=curvatures[index],
        )
        for index, point in enumerate(points)
    )


def adjust_clearance(
    grid: OccupancyGrid,
    poses: Sequence[Pose2D],
    vehicle: VehicleFootprintSpec,
    parameters: AdjustmentParameters,
    *,
    source_revision: int = 0,
    cancel_requested: Optional[Callable[[], bool]] = None,
) -> AdjustmentResult:
    def check_cancelled() -> None:
        if cancel_requested is not None and cancel_requested():
            raise CancelledError("clearance adjustment cancelled")

    check_cancelled()
    validation_options = ValidationOptions(
        circular=parameters.circular,
        sweep_step_m=parameters.sweep_step_m,
        include_sweep=True,
    )
    safety_options = ValidationOptions(
        circular=parameters.circular,
        sweep_step_m=parameters.sweep_step_m,
        include_sweep=True,
        calculate_clearance=False,
    )
    before = validate_clearance(
        grid,
        poses,
        vehicle,
        validation_options,
        source_revision=source_revision,
    )
    check_cancelled()
    if before.is_safe:
        return AdjustmentResult(
            status=AdjustmentStatus.NOT_NEEDED,
            before_report=before,
        )
    offsets = _offset_values(parameters)
    duplicate_endpoint = (
        parameters.circular
        and len(poses) > 2
        and math.hypot(
            poses[-1].x_m - poses[0].x_m,
            poses[-1].y_m - poses[0].y_m,
        )
        <= 1e-6
    )
    optimization_poses = poses[:-1] if duplicate_endpoint else poses
    feasible: list[Tuple[float, ...]] = []
    offset_poses: list[dict[float, Pose2D]] = []
    infeasible_indices: list[int] = []
    for index, pose in enumerate(optimization_poses):
        check_cancelled()
        pose_candidates = {
            offset: _offset_pose(pose, offset)
            for offset in offsets
        }
        candidates = tuple(
            offset
            for offset, offset_pose in pose_candidates.items()
            if _pose_is_safe(grid, offset_pose, vehicle)
        )
        feasible.append(candidates)
        offset_poses.append(pose_candidates)
        if not candidates:
            infeasible_indices.append(index)
    step_m = parameters.sweep_step_m or max(
        0.01,
        0.5 * grid.spec.resolution_m,
    )
    transition_cache: dict[Tuple[int, float, float], bool] = {}

    def transition_safe(index: int, first_offset: float, second_offset: float) -> bool:
        check_cancelled()
        key = (index, first_offset, second_offset)
        if key not in transition_cache:
            second_index = (index + 1) % len(optimization_poses)
            transition_cache[key] = _transition_is_safe(
                grid,
                offset_poses[index][first_offset],
                offset_poses[second_index][second_offset],
                vehicle,
                step_m,
            )
        return transition_cache[key]

    ranked_paths = _rank_offset_paths(
        feasible,
        parameters,
        transition_safe=transition_safe,
    )
    if not ranked_paths:
        issue = ClearanceIssue(
            code="CLEARANCE_ADJUSTMENT_INFEASIBLE",
            severity=Severity.ERROR,
            message=(
                "no point- and transition-safe lateral offset path exists within "
                "the selected maximum shift"
            ),
            point_index=infeasible_indices[0] if infeasible_indices else None,
        )
        return AdjustmentResult(
            status=AdjustmentStatus.INFEASIBLE,
            before_report=before,
            infeasible_point_indices=tuple(infeasible_indices),
            issues=(issue,),
        )
    attempted_report: Optional[ClearanceReport] = None
    curvature_rejections = 0
    for selected_core in ranked_paths:
        check_cancelled()
        selected = (
            (*selected_core, selected_core[0])
            if duplicate_endpoint
            else selected_core
        )
        adjusted_points = tuple(
            (
                pose.x_m - math.sin(pose.yaw_rad) * offset,
                pose.y_m + math.cos(pose.yaw_rad) * offset,
            )
            for pose, offset in zip(poses, selected)
        )
        adjusted_poses = _poses_from_points(adjusted_points, parameters.circular)
        maximum_curvature = max(
            abs(pose.curvature_radpm or 0.0) for pose in adjusted_poses
        )
        if (
            parameters.max_abs_curvature_radpm is not None
            and maximum_curvature > parameters.max_abs_curvature_radpm + 1e-9
        ):
            curvature_rejections += 1
            continue
        attempted_report = validate_clearance(
            grid,
            adjusted_poses,
            vehicle,
            safety_options,
            source_revision=source_revision,
        )
        check_cancelled()
        if not attempted_report.is_safe:
            continue
        attempted_report = validate_clearance(
            grid,
            adjusted_poses,
            vehicle,
            validation_options,
            source_revision=source_revision,
        )
        check_cancelled()
        if not attempted_report.is_safe:
            continue
        candidate = ClearanceCandidate(
            source_revision=source_revision,
            map_signature=grid.spec.signature,
            vehicle=vehicle,
            parameters=parameters,
            poses=adjusted_poses,
            offsets=tuple(selected),
            before_report=before,
            after_report=attempted_report,
            max_shift_m=max(abs(value) for value in selected),
        )
        return AdjustmentResult(
            status=AdjustmentStatus.FEASIBLE,
            before_report=before,
            candidate=candidate,
            attempted_report=attempted_report,
        )

    if curvature_rejections == len(ranked_paths):
        code = "CLEARANCE_ADJUSTMENT_CURVATURE_LIMIT"
        message = (
            "all transition-safe offset paths exceed the configured absolute "
            "curvature limit"
        )
    else:
        code = "CLEARANCE_ADJUSTMENT_SWEEP_INFEASIBLE"
        message = (
            "offset paths exist, but regenerated heading/sweep validation did "
            "not find a safe candidate"
        )
    issue = ClearanceIssue(
        code=code,
        severity=Severity.ERROR,
        message=message,
    )
    return AdjustmentResult(
        status=AdjustmentStatus.INFEASIBLE,
        before_report=before,
        attempted_report=attempted_report,
        issues=(issue,),
    )


__all__ = [
    "AdjustmentParameters",
    "AdjustmentResult",
    "AdjustmentStatus",
    "CellState",
    "ClearanceCandidate",
    "ClearanceIssue",
    "ClearanceReport",
    "MapLoadOptions",
    "OccupancyGrid",
    "OccupancyGridSpec",
    "Pose2D",
    "Severity",
    "ValidationOptions",
    "VehicleFootprintSpec",
    "adjust_clearance",
    "footprint_polygon",
    "load_occupancy_grid",
    "load_vehicle_footprint",
    "validate_clearance",
]
