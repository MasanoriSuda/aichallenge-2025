"""Tk settings and report dialogs for Kaleidoscope clearance checks.

The numerical clearance implementation deliberately does not depend on this
module.  Pure helpers in this file make the user-entered settings testable
without creating a Tk root window.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from pathlib import Path
import tkinter as tk
from tkinter import filedialog
from tkinter import messagebox
from tkinter import ttk
from typing import Callable, Mapping, Optional, Sequence

import yaml


PROVISIONAL_REFERENCE_POINT = "rear_axle"
PROVISIONAL_NOTICE = (
    "Provisional rear-axle preset from vehicle_info YAML. Verify the trajectory "
    "pose reference and AWSIM collider before treating this as physical clearance."
)
DEFAULT_MARGIN_M = 0.0
DEFAULT_SWEEP_STEP_M = 0.05
DEFAULT_MAX_LATERAL_SHIFT_M = 0.50
DEFAULT_OFFSET_STEP_M = 0.05
DEFAULT_SMOOTHNESS_WEIGHT = 1.0
DEFAULT_MAX_ABS_CURVATURE_RADPM = 0.70


class UnknownCellPolicy(str, Enum):
    """How occupancy-grid unknown cells participate in a clearance check."""

    OCCUPIED = "occupied"
    FREE = "free"


@dataclass(frozen=True)
class ProvisionalVehicleExtents:
    """Rear-axle-oriented extents derived from a vehicle-info YAML file."""

    source_path: Path
    wheel_base_m: float
    front_overhang_m: float
    rear_overhang_m: float
    wheel_tread_m: float
    left_overhang_m: float
    right_overhang_m: float
    front_extent_m: float
    rear_extent_m: float
    left_extent_m: float
    right_extent_m: float


@dataclass(frozen=True)
class ClearanceDialogConfig:
    """Explicit, immutable settings collected by the clearance dialog."""

    map_yaml_path: Path
    vehicle_yaml_path: Optional[Path]
    reference_point: str
    wheel_base_m: float
    front_overhang_m: float
    rear_overhang_m: float
    wheel_tread_m: float
    left_overhang_m: float
    right_overhang_m: float
    front_extent_m: float
    rear_extent_m: float
    left_extent_m: float
    right_extent_m: float
    margin_front_m: float
    margin_rear_m: float
    margin_left_m: float
    margin_right_m: float
    unknown_policy: UnknownCellPolicy
    sweep_step_m: float
    max_lateral_shift_m: float
    offset_step_m: float
    smoothness_weight: float
    max_abs_curvature_radpm: float

    @property
    def body_length_m(self) -> float:
        return self.front_extent_m + self.rear_extent_m

    @property
    def body_width_m(self) -> float:
        return self.left_extent_m + self.right_extent_m

    @property
    def envelope_length_m(self) -> float:
        return self.body_length_m + self.margin_front_m + self.margin_rear_m

    @property
    def envelope_width_m(self) -> float:
        return self.body_width_m + self.margin_left_m + self.margin_right_m

    @property
    def unknown_is_occupied(self) -> bool:
        return self.unknown_policy is UnknownCellPolicy.OCCUPIED

    def vehicle_footprint_kwargs(self) -> dict[str, object]:
        """Return exact kwargs for trajectory_clearance.VehicleFootprintSpec."""

        return {
            "reference_point": self.reference_point,
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

    def adjustment_parameter_kwargs(
        self,
        *,
        circular: bool,
        displacement_weight: float = 1.0,
        curvature_weight: float = 1.0,
    ) -> dict[str, object]:
        """Return kwargs matching the clearance-core AdjustmentParameters API."""

        return {
            "max_lateral_shift_m": self.max_lateral_shift_m,
            "sampling_step_m": self.offset_step_m,
            "displacement_weight": displacement_weight,
            "smoothness_weight": self.smoothness_weight,
            "curvature_weight": curvature_weight,
            "circular": bool(circular),
            "sweep_step_m": self.sweep_step_m,
            "max_abs_curvature_radpm": self.max_abs_curvature_radpm,
        }


def _finite_number(
    raw_value: object,
    name: str,
    *,
    minimum: float = 0.0,
    strictly_positive: bool = False,
) -> float:
    if isinstance(raw_value, bool):
        raise ValueError(f"{name} must be a number")
    try:
        value = float(raw_value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be a number") from error
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    if strictly_positive and value <= minimum:
        raise ValueError(f"{name} must be greater than {minimum:g}")
    if not strictly_positive and value < minimum:
        raise ValueError(f"{name} must be at least {minimum:g}")
    return value


def _path_value(raw_value: object, name: str, *, required: bool) -> Optional[Path]:
    text = str(raw_value).strip() if raw_value is not None else ""
    if not text:
        if required:
            raise ValueError(f"{name} is required")
        return None
    return Path(text).expanduser()


def _ros_parameters(document: object) -> Mapping[str, object]:
    if not isinstance(document, Mapping):
        raise ValueError("vehicle YAML root must be a mapping")
    direct = document.get("ros__parameters")
    if isinstance(direct, Mapping):
        return direct
    wildcard = document.get("/**")
    if isinstance(wildcard, Mapping):
        parameters = wildcard.get("ros__parameters")
        if isinstance(parameters, Mapping):
            return parameters
    raise ValueError("vehicle YAML must contain ros__parameters")


def load_provisional_vehicle_extents(path: Path) -> ProvisionalVehicleExtents:
    """Load current repository vehicle values and derive rear-axle extents.

    Loading this preset does not establish that the trajectory actually uses a
    rear-axle reference.  The GUI always labels the result as provisional.
    """

    source_path = Path(path).expanduser()
    try:
        with source_path.open("r", encoding="utf-8") as stream:
            parameters = _ros_parameters(yaml.safe_load(stream))
    except OSError as error:
        raise ValueError(f"could not read vehicle YAML: {error}") from error
    except yaml.YAMLError as error:
        raise ValueError(f"invalid vehicle YAML: {error}") from error

    values: dict[str, float] = {}
    for name in (
        "wheel_base",
        "front_overhang",
        "rear_overhang",
        "wheel_tread",
        "left_overhang",
        "right_overhang",
    ):
        if name not in parameters:
            raise ValueError(f"vehicle YAML is missing ros__parameters.{name}")
        values[name] = _finite_number(
            parameters[name],
            name,
            strictly_positive=name in {"wheel_base", "wheel_tread"},
        )

    return ProvisionalVehicleExtents(
        source_path=source_path,
        wheel_base_m=values["wheel_base"],
        front_overhang_m=values["front_overhang"],
        rear_overhang_m=values["rear_overhang"],
        wheel_tread_m=values["wheel_tread"],
        left_overhang_m=values["left_overhang"],
        right_overhang_m=values["right_overhang"],
        front_extent_m=values["wheel_base"] + values["front_overhang"],
        rear_extent_m=values["rear_overhang"],
        left_extent_m=values["wheel_tread"] / 2.0 + values["left_overhang"],
        right_extent_m=values["wheel_tread"] / 2.0 + values["right_overhang"],
    )


def find_repository_vehicle_yaml() -> Optional[Path]:
    """Find the installed or source-tree vehicle-info YAML without writing."""

    try:
        from ament_index_python.packages import get_package_share_directory

        installed = (
            Path(get_package_share_directory("racing_kart_description"))
            / "config"
            / "vehicle_info.param.yaml"
        )
        if installed.is_file():
            return installed
    except Exception:  # noqa: BLE001 - source-tree fallback is intentional
        pass

    for parent in Path(__file__).resolve().parents:
        candidate = (
            parent
            / "racing_kart_description"
            / "config"
            / "vehicle_info.param.yaml"
        )
        if candidate.is_file():
            return candidate
    return None


def parse_clearance_dialog_config(
    *,
    map_yaml_path: object,
    vehicle_yaml_path: object,
    wheel_base_m: object,
    wheel_tread_m: object,
    front_extent_m: object,
    rear_extent_m: object,
    left_extent_m: object,
    right_extent_m: object,
    margin_front_m: object,
    margin_rear_m: object,
    margin_left_m: object,
    margin_right_m: object,
    unknown_policy: object,
    sweep_step_m: object,
    max_lateral_shift_m: object,
    offset_step_m: object,
    smoothness_weight: object,
    max_abs_curvature_radpm: object = DEFAULT_MAX_ABS_CURVATURE_RADPM,
    require_existing_paths: bool = False,
) -> ClearanceDialogConfig:
    """Parse text-friendly dialog values into a validated immutable config."""

    map_path = _path_value(map_yaml_path, "occupancy-grid YAML path", required=True)
    assert map_path is not None
    vehicle_path = _path_value(
        vehicle_yaml_path, "vehicle-info YAML path", required=False
    )
    try:
        policy = (
            unknown_policy
            if isinstance(unknown_policy, UnknownCellPolicy)
            else UnknownCellPolicy(str(unknown_policy).strip().lower())
        )
    except ValueError as error:
        choices = ", ".join(policy.value for policy in UnknownCellPolicy)
        raise ValueError(f"unknown policy must be one of: {choices}") from error

    wheel_base = _finite_number(
        wheel_base_m, "wheel base", strictly_positive=True
    )
    wheel_tread = _finite_number(
        wheel_tread_m, "wheel tread", strictly_positive=True
    )
    front_extent = _finite_number(front_extent_m, "front extent")
    rear_extent = _finite_number(rear_extent_m, "rear extent")
    left_extent = _finite_number(left_extent_m, "left extent")
    right_extent = _finite_number(right_extent_m, "right extent")
    front_overhang = front_extent - wheel_base
    left_overhang = left_extent - wheel_tread / 2.0
    right_overhang = right_extent - wheel_tread / 2.0
    for name, value in (
        ("front extent minus wheel base", front_overhang),
        ("left extent minus half wheel tread", left_overhang),
        ("right extent minus half wheel tread", right_overhang),
    ):
        if value < -1e-12:
            raise ValueError(f"{name} must be non-negative")

    config = ClearanceDialogConfig(
        map_yaml_path=map_path,
        vehicle_yaml_path=vehicle_path,
        reference_point=PROVISIONAL_REFERENCE_POINT,
        wheel_base_m=wheel_base,
        front_overhang_m=max(0.0, front_overhang),
        rear_overhang_m=rear_extent,
        wheel_tread_m=wheel_tread,
        left_overhang_m=max(0.0, left_overhang),
        right_overhang_m=max(0.0, right_overhang),
        front_extent_m=front_extent,
        rear_extent_m=rear_extent,
        left_extent_m=left_extent,
        right_extent_m=right_extent,
        margin_front_m=_finite_number(margin_front_m, "front margin"),
        margin_rear_m=_finite_number(margin_rear_m, "rear margin"),
        margin_left_m=_finite_number(margin_left_m, "left margin"),
        margin_right_m=_finite_number(margin_right_m, "right margin"),
        unknown_policy=policy,
        sweep_step_m=_finite_number(
            sweep_step_m, "sweep step", strictly_positive=True
        ),
        max_lateral_shift_m=_finite_number(
            max_lateral_shift_m, "maximum lateral shift"
        ),
        offset_step_m=_finite_number(
            offset_step_m, "offset step", strictly_positive=True
        ),
        smoothness_weight=_finite_number(smoothness_weight, "smoothness weight"),
        max_abs_curvature_radpm=_finite_number(
            max_abs_curvature_radpm,
            "maximum absolute curvature",
            strictly_positive=True,
        ),
    )
    validate_clearance_dialog_config(
        config, require_existing_paths=require_existing_paths
    )
    return config


def validate_clearance_dialog_config(
    config: ClearanceDialogConfig,
    *,
    require_existing_paths: bool = False,
) -> None:
    """Validate cross-field constraints for an already constructed config."""

    if config.reference_point != PROVISIONAL_REFERENCE_POINT:
        raise ValueError("only the provisional rear_axle reference is supported")
    if config.body_length_m <= 0.0:
        raise ValueError("front and rear extents must define a positive length")
    if config.body_width_m <= 0.0:
        raise ValueError("left and right extents must define a positive width")
    if not math.isclose(
        config.front_extent_m,
        config.wheel_base_m + config.front_overhang_m,
        rel_tol=0.0,
        abs_tol=1e-9,
    ):
        raise ValueError("front extent is inconsistent with wheel base and overhang")
    if not math.isclose(
        config.left_extent_m,
        config.wheel_tread_m / 2.0 + config.left_overhang_m,
        rel_tol=0.0,
        abs_tol=1e-9,
    ) or not math.isclose(
        config.right_extent_m,
        config.wheel_tread_m / 2.0 + config.right_overhang_m,
        rel_tol=0.0,
        abs_tol=1e-9,
    ):
        raise ValueError("side extents are inconsistent with wheel tread and overhangs")
    if config.max_lateral_shift_m > 0.0 and (
        config.offset_step_m > 2.0 * config.max_lateral_shift_m
    ):
        raise ValueError(
            "offset step must not exceed twice the maximum lateral shift"
        )
    if require_existing_paths:
        if not config.map_yaml_path.is_file():
            raise ValueError(
                f"occupancy-grid YAML does not exist: {config.map_yaml_path}"
            )
        if config.vehicle_yaml_path is not None and not config.vehicle_yaml_path.is_file():
            raise ValueError(
                f"vehicle-info YAML does not exist: {config.vehicle_yaml_path}"
            )


def provisional_default_config(
    map_yaml_path: Path,
    vehicle_yaml_path: Optional[Path] = None,
) -> ClearanceDialogConfig:
    """Build explicit defaults from vehicle_info YAML; margins stay zero."""

    source = vehicle_yaml_path or find_repository_vehicle_yaml()
    if source is None:
        raise ValueError("vehicle_info.param.yaml could not be located")
    extents = load_provisional_vehicle_extents(source)
    return parse_clearance_dialog_config(
        map_yaml_path=map_yaml_path,
        vehicle_yaml_path=source,
        wheel_base_m=extents.wheel_base_m,
        wheel_tread_m=extents.wheel_tread_m,
        front_extent_m=extents.front_extent_m,
        rear_extent_m=extents.rear_extent_m,
        left_extent_m=extents.left_extent_m,
        right_extent_m=extents.right_extent_m,
        margin_front_m=DEFAULT_MARGIN_M,
        margin_rear_m=DEFAULT_MARGIN_M,
        margin_left_m=DEFAULT_MARGIN_M,
        margin_right_m=DEFAULT_MARGIN_M,
        unknown_policy=UnknownCellPolicy.OCCUPIED,
        sweep_step_m=DEFAULT_SWEEP_STEP_M,
        max_lateral_shift_m=DEFAULT_MAX_LATERAL_SHIFT_M,
        offset_step_m=DEFAULT_OFFSET_STEP_M,
        smoothness_weight=DEFAULT_SMOOTHNESS_WEIGHT,
        max_abs_curvature_radpm=DEFAULT_MAX_ABS_CURVATURE_RADPM,
    )


class ClearanceSettingsDialog(tk.Toplevel):
    """Modal editor for map, vehicle envelope, margins, and search settings."""

    _NUMERIC_FIELDS = (
        ("wheel_base_m", "Wheel base", "m"),
        ("wheel_tread_m", "Wheel tread", "m"),
        ("front_extent_m", "Front extent", "m"),
        ("rear_extent_m", "Rear extent", "m"),
        ("left_extent_m", "Left extent", "m"),
        ("right_extent_m", "Right extent", "m"),
        ("margin_front_m", "Front margin", "m"),
        ("margin_rear_m", "Rear margin", "m"),
        ("margin_left_m", "Left margin", "m"),
        ("margin_right_m", "Right margin", "m"),
        ("sweep_step_m", "Sweep interpolation step", "m"),
        ("max_lateral_shift_m", "Maximum lateral shift", "m"),
        ("offset_step_m", "Lateral offset step", "m"),
        ("smoothness_weight", "Smoothness weight", ""),
        ("max_abs_curvature_radpm", "Maximum absolute curvature", "rad/m"),
    )

    def __init__(
        self,
        parent: tk.Misc,
        *,
        initial: Optional[ClearanceDialogConfig] = None,
        map_yaml_path: Optional[Path] = None,
        vehicle_yaml_path: Optional[Path] = None,
    ) -> None:
        super().__init__(parent)
        self.title("Vehicle / Margin Settings")
        self.geometry("820x680")
        self.minsize(720, 560)
        self.transient(parent)
        self.result: Optional[ClearanceDialogConfig] = None

        selected_vehicle = (
            initial.vehicle_yaml_path
            if initial is not None
            else vehicle_yaml_path or find_repository_vehicle_yaml()
        )
        self.map_path = tk.StringVar(
            value=str(initial.map_yaml_path if initial is not None else map_yaml_path or "")
        )
        self.vehicle_path = tk.StringVar(
            value="" if selected_vehicle is None else str(selected_vehicle)
        )
        self.unknown_policy = tk.StringVar(
            value=(
                initial.unknown_policy.value
                if initial is not None
                else UnknownCellPolicy.OCCUPIED.value
            )
        )
        self.values: dict[str, tk.StringVar] = {}
        if initial is not None:
            for name, _label, _unit in self._NUMERIC_FIELDS:
                self.values[name] = tk.StringVar(value=f"{getattr(initial, name):.9g}")
        else:
            defaults = {
                "margin_front_m": DEFAULT_MARGIN_M,
                "margin_rear_m": DEFAULT_MARGIN_M,
                "margin_left_m": DEFAULT_MARGIN_M,
                "margin_right_m": DEFAULT_MARGIN_M,
                "sweep_step_m": DEFAULT_SWEEP_STEP_M,
                "max_lateral_shift_m": DEFAULT_MAX_LATERAL_SHIFT_M,
                "offset_step_m": DEFAULT_OFFSET_STEP_M,
                "smoothness_weight": DEFAULT_SMOOTHNESS_WEIGHT,
                "max_abs_curvature_radpm": DEFAULT_MAX_ABS_CURVATURE_RADPM,
            }
            for name, _label, _unit in self._NUMERIC_FIELDS:
                self.values[name] = tk.StringVar(
                    value="" if name not in defaults else f"{defaults[name]:.9g}"
                )

        outer = ttk.Frame(self, padding=12)
        outer.pack(fill=tk.BOTH, expand=True)
        outer.columnconfigure(1, weight=1)
        ttk.Label(
            outer,
            text=PROVISIONAL_NOTICE,
            foreground="#a85d00",
            wraplength=760,
            justify=tk.LEFT,
        ).grid(row=0, column=0, columnspan=4, sticky="ew", pady=(0, 10))

        ttk.Label(outer, text="Occupancy-grid YAML").grid(row=1, column=0, sticky="w")
        ttk.Entry(outer, textvariable=self.map_path).grid(
            row=1, column=1, columnspan=2, sticky="ew", padx=(8, 6)
        )
        ttk.Button(outer, text="Browse...", command=self._browse_map).grid(
            row=1, column=3, sticky="e"
        )
        ttk.Label(outer, text="Vehicle-info YAML (optional after loading)").grid(
            row=2, column=0, sticky="w", pady=(6, 0)
        )
        ttk.Entry(outer, textvariable=self.vehicle_path).grid(
            row=2, column=1, columnspan=2, sticky="ew", padx=(8, 6), pady=(6, 0)
        )
        vehicle_buttons = ttk.Frame(outer)
        vehicle_buttons.grid(row=2, column=3, sticky="e", pady=(6, 0))
        ttk.Button(vehicle_buttons, text="Browse...", command=self._browse_vehicle).pack(
            side=tk.LEFT
        )
        ttk.Button(vehicle_buttons, text="Reload", command=self._reload_vehicle).pack(
            side=tk.LEFT, padx=(4, 0)
        )
        ttk.Label(
            outer,
            text="Reference point: rear axle (provisional, fixed for this phase)",
        ).grid(row=3, column=0, columnspan=4, sticky="w", pady=(8, 6))

        for row, (name, label, unit) in enumerate(self._NUMERIC_FIELDS, start=4):
            ttk.Label(outer, text=label).grid(row=row, column=0, sticky="w", pady=2)
            ttk.Entry(outer, textvariable=self.values[name], width=16).grid(
                row=row, column=1, sticky="w", padx=(8, 4), pady=2
            )
            ttk.Label(outer, text=unit).grid(row=row, column=2, sticky="w", pady=2)

        policy_row = 4 + len(self._NUMERIC_FIELDS)
        ttk.Label(outer, text="Unknown cells").grid(
            row=policy_row, column=0, sticky="w", pady=(6, 0)
        )
        ttk.Combobox(
            outer,
            textvariable=self.unknown_policy,
            values=tuple(policy.value for policy in UnknownCellPolicy),
            state="readonly",
            width=16,
        ).grid(row=policy_row, column=1, sticky="w", padx=(8, 4), pady=(6, 0))
        ttk.Label(
            outer,
            text="occupied is the safety-side default",
            foreground="#a85d00",
        ).grid(row=policy_row, column=2, columnspan=2, sticky="w", pady=(6, 0))

        self.dimension_summary = tk.StringVar(value="")
        ttk.Label(outer, textvariable=self.dimension_summary).grid(
            row=policy_row + 1, column=0, columnspan=4, sticky="w", pady=(8, 0)
        )
        for variable in self.values.values():
            variable.trace_add("write", lambda *_args: self._update_dimensions())

        buttons = ttk.Frame(outer)
        buttons.grid(row=policy_row + 2, column=0, columnspan=4, sticky="e", pady=(14, 0))
        ttk.Button(buttons, text="Cancel", command=self._cancel).pack(
            side=tk.RIGHT, padx=(6, 0)
        )
        ttk.Button(buttons, text="Apply Settings", command=self._accept).pack(
            side=tk.RIGHT
        )

        self.protocol("WM_DELETE_WINDOW", self._cancel)
        self.bind("<Escape>", lambda _event: self._cancel())
        self.grab_set()
        if initial is None and selected_vehicle is not None:
            self.after_idle(self._reload_vehicle)
        self.after_idle(self._update_dimensions)

    def _browse_map(self) -> None:
        selected = filedialog.askopenfilename(
            parent=self,
            title="Select occupancy-grid YAML",
            filetypes=(("YAML", "*.yaml *.yml"), ("All files", "*")),
        )
        if selected:
            self.map_path.set(selected)

    def _browse_vehicle(self) -> None:
        selected = filedialog.askopenfilename(
            parent=self,
            title="Select vehicle-info YAML",
            filetypes=(("YAML", "*.yaml *.yml"), ("All files", "*")),
        )
        if selected:
            self.vehicle_path.set(selected)
            self._reload_vehicle()

    def _reload_vehicle(self) -> None:
        try:
            path = _path_value(self.vehicle_path.get(), "vehicle-info YAML path", required=True)
            assert path is not None
            extents = load_provisional_vehicle_extents(path)
        except ValueError as error:
            messagebox.showerror("Vehicle YAML", str(error), parent=self)
            return
        for name in (
            "wheel_base_m",
            "wheel_tread_m",
            "front_extent_m",
            "rear_extent_m",
            "left_extent_m",
            "right_extent_m",
        ):
            self.values[name].set(f"{getattr(extents, name):.9g}")
        self._update_dimensions()

    def _raw_values(self) -> dict[str, object]:
        return {
            "map_yaml_path": self.map_path.get(),
            "vehicle_yaml_path": self.vehicle_path.get(),
            **{name: variable.get() for name, variable in self.values.items()},
            "unknown_policy": self.unknown_policy.get(),
        }

    def _update_dimensions(self) -> None:
        try:
            front = float(self.values["front_extent_m"].get())
            rear = float(self.values["rear_extent_m"].get())
            left = float(self.values["left_extent_m"].get())
            right = float(self.values["right_extent_m"].get())
            margins = [
                float(self.values[name].get())
                for name in (
                    "margin_front_m",
                    "margin_rear_m",
                    "margin_left_m",
                    "margin_right_m",
                )
            ]
            if not all(math.isfinite(value) for value in (front, rear, left, right, *margins)):
                raise ValueError
            self.dimension_summary.set(
                f"Body: {front + rear:.3f} x {left + right:.3f} m | "
                f"with margins: {front + rear + margins[0] + margins[1]:.3f} x "
                f"{left + right + margins[2] + margins[3]:.3f} m"
            )
        except (TypeError, ValueError):
            self.dimension_summary.set("Body/envelope size: enter valid finite values")

    def _accept(self) -> None:
        try:
            self.result = parse_clearance_dialog_config(
                **self._raw_values(), require_existing_paths=True
            )
        except ValueError as error:
            self.result = None
            messagebox.showerror("Invalid Clearance settings", str(error), parent=self)
            return
        if self.result.unknown_policy is UnknownCellPolicy.FREE:
            proceed = messagebox.askyesno(
                "Unsafe unknown-cell policy",
                "Unknown cells will be treated as free. Continue with this explicit override?",
                parent=self,
            )
            if not proceed:
                self.result = None
                return
        self.destroy()

    def _cancel(self) -> None:
        self.result = None
        self.destroy()


def ask_clearance_settings(
    parent: tk.Misc,
    *,
    initial: Optional[ClearanceDialogConfig] = None,
    map_yaml_path: Optional[Path] = None,
    vehicle_yaml_path: Optional[Path] = None,
) -> Optional[ClearanceDialogConfig]:
    dialog = ClearanceSettingsDialog(
        parent,
        initial=initial,
        map_yaml_path=map_yaml_path,
        vehicle_yaml_path=vehicle_yaml_path,
    )
    parent.wait_window(dialog)
    return dialog.result


def _field(value: object, names: Sequence[str], default: object = None) -> object:
    for name in names:
        if isinstance(value, Mapping) and name in value:
            return value[name]
        if hasattr(value, name):
            return getattr(value, name)
    return default


def _text(value: object) -> str:
    if value is None:
        return ""
    enum_value = getattr(value, "value", value)
    return str(enum_value)


def _number_text(value: object) -> str:
    if value is None:
        return ""
    try:
        number = float(value)
    except (TypeError, ValueError):
        return _text(value)
    return f"{number:.6g}" if math.isfinite(number) else _text(value)


def clearance_issue_row(issue: object) -> tuple[str, ...]:
    """Normalize a core issue/dataclass/mapping into report Treeview cells."""

    grid_cell = _field(issue, ("grid_cell", "cell"))
    return (
        _text(_field(issue, ("severity",), "")),
        _text(_field(issue, ("code",), "")),
        _text(_field(issue, ("point_index",), "")),
        _text(_field(issue, ("segment_index",), "")),
        _number_text(_field(issue, ("s_m",), None)),
        _number_text(
            _field(issue, ("clearance_m", "raw_clearance_m", "minimum_clearance_m"))
        ),
        _number_text(_field(issue, ("required_margin_m", "margin_m"))),
        _text(grid_cell),
        _text(_field(issue, ("message",), "")),
    )


def clearance_report_summary(report: object) -> str:
    """Create a compact summary while tolerating evolving core field names."""

    safe = bool(_field(report, ("is_safe", "safe"), False))
    issues = tuple(_field(report, ("issues",), ()) or ())
    raw = _number_text(
        _field(report, ("minimum_clearance_m", "minimum_raw_clearance_m"))
    )
    conservative = _number_text(
        _field(report, ("conservative_minimum_clearance_m", "minimum_conservative_clearance_m"))
    )
    parts = [f"status={'SAFE' if safe else 'UNSAFE'}", f"issues={len(issues)}"]
    if raw:
        parts.append(f"minimum raw clearance={raw} m")
    if conservative:
        parts.append(f"minimum conservative clearance={conservative} m")
    resolution = _number_text(
        _field(report, ("measurement_resolution_m", "resolution_m"))
    )
    if resolution:
        parts.append(f"map resolution={resolution} m")
    for names, label in (
        (("colliding_point_count",), "colliding points"),
        (("colliding_segment_count",), "colliding segments"),
        (("unknown_contact_count",), "unknown contacts"),
        (("outside_map_count",), "outside-map"),
    ):
        value = _field(report, names)
        if value is not None:
            parts.append(f"{label}={value}")
    return " | ".join(parts)


class ClearanceReportDialog(tk.Toplevel):
    """Non-destructive structured issue list with main-canvas navigation."""

    def __init__(
        self,
        parent: tk.Misc,
        *,
        report: object,
        on_center_issue: Optional[Callable[[object], None]] = None,
    ) -> None:
        super().__init__(parent)
        self.title("Clearance Report")
        self.geometry("1120x520")
        self.minsize(820, 360)
        self.transient(parent)
        self.report = report
        self.on_center_issue = on_center_issue
        self.issue_map: dict[str, object] = {}

        body = ttk.Frame(self, padding=10)
        body.pack(fill=tk.BOTH, expand=True)
        ttk.Label(
            body,
            text=clearance_report_summary(report),
            wraplength=1080,
            justify=tk.LEFT,
        ).pack(fill=tk.X, anchor="w", pady=(0, 8))

        tree_frame = ttk.Frame(body)
        tree_frame.pack(fill=tk.BOTH, expand=True)
        columns = (
            "severity",
            "code",
            "point",
            "segment",
            "s_m",
            "clearance",
            "required_margin",
            "cell",
            "message",
        )
        self.issue_tree = ttk.Treeview(tree_frame, columns=columns, show="headings")
        widths = (75, 210, 60, 65, 85, 90, 100, 90, 380)
        for column, width in zip(columns, widths):
            self.issue_tree.heading(column, text=column)
            self.issue_tree.column(
                column, width=width, stretch=column == "message", anchor="w"
            )
        vertical = ttk.Scrollbar(
            tree_frame, orient=tk.VERTICAL, command=self.issue_tree.yview
        )
        horizontal = ttk.Scrollbar(
            tree_frame, orient=tk.HORIZONTAL, command=self.issue_tree.xview
        )
        self.issue_tree.configure(
            yscrollcommand=vertical.set, xscrollcommand=horizontal.set
        )
        self.issue_tree.grid(row=0, column=0, sticky="nsew")
        vertical.grid(row=0, column=1, sticky="ns")
        horizontal.grid(row=1, column=0, sticky="ew")
        tree_frame.rowconfigure(0, weight=1)
        tree_frame.columnconfigure(0, weight=1)

        for issue in tuple(_field(report, ("issues",), ()) or ()):
            item = self.issue_tree.insert("", tk.END, values=clearance_issue_row(issue))
            self.issue_map[item] = issue

        self.detail = tk.StringVar(value="Select an issue to inspect or center it.")
        ttk.Label(body, textvariable=self.detail, wraplength=1080, justify=tk.LEFT).pack(
            fill=tk.X, anchor="w", pady=(8, 0)
        )
        buttons = ttk.Frame(body)
        buttons.pack(fill=tk.X, pady=(8, 0))
        ttk.Button(buttons, text="Close", command=self.destroy).pack(side=tk.RIGHT)
        self.center_button = ttk.Button(
            buttons, text="Center Issue", command=self._center_selected
        )
        self.center_button.pack(side=tk.RIGHT, padx=(0, 8))
        self.center_button.state(["disabled"])

        self.issue_tree.bind("<<TreeviewSelect>>", self._on_select)
        self.issue_tree.bind("<Double-1>", lambda _event: self._center_selected())
        self.bind("<Escape>", lambda _event: self.destroy())

    def _selected_issue(self) -> Optional[object]:
        selected = self.issue_tree.selection()
        return self.issue_map.get(selected[0]) if selected else None

    def _on_select(self, _event: tk.Event) -> None:
        issue = self._selected_issue()
        if issue is None:
            self.detail.set("Select an issue to inspect or center it.")
            self.center_button.state(["disabled"])
            return
        self.detail.set(_text(_field(issue, ("message",), "")))
        if self.on_center_issue is None:
            self.center_button.state(["disabled"])
        else:
            self.center_button.state(["!disabled"])

    def _center_selected(self) -> None:
        issue = self._selected_issue()
        if issue is not None and self.on_center_issue is not None:
            self.on_center_issue(issue)


def show_clearance_report(
    parent: tk.Misc,
    *,
    report: object,
    on_center_issue: Optional[Callable[[object], None]] = None,
) -> ClearanceReportDialog:
    """Open a modeless report so its callback can navigate the main canvas."""

    return ClearanceReportDialog(
        parent, report=report, on_center_issue=on_center_issue
    )
