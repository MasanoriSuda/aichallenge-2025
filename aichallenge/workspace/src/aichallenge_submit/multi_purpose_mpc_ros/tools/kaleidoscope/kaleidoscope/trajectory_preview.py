"""Tk dialogs and Canvas rendering for trajectory candidates.

All numeric work lives in the pure processing/plot modules.  This module only
collects explicit user parameters and renders immutable comparison models.
"""

from __future__ import annotations

from dataclasses import fields
import math
import threading
import tkinter as tk
from tkinter import messagebox
from tkinter import ttk
from typing import Callable, Mapping, Optional, Sequence, Tuple, TypeVar

from .trajectory_contract import ValidationIssue
from .trajectory_contract import ValidationReport
from .trajectory_plot import ComparisonPlotData
from .trajectory_plot import ScalarPlotSeries
from .trajectory_processing import MetadataMode
from .trajectory_processing import NormalizeOptions
from .trajectory_speed import SpeedProfileParameters


LOCAL_PRESET_NAME = "AI Challenge 2026 Candidate - Safe"
LOCAL_RESOLUTION_M = 0.25
LOCAL_A_MAX_MPS2 = 1.0
LOCAL_HORIZON_DISTANCE_M = 16.0

_BEFORE_COLOR = "#758195"
_CANDIDATE_COLOR = "#ff9f43"
_SELECTION_COLOR = "#ffdf6b"
_MAX_CANVAS_SAMPLES = 5_000
_TaskResult = TypeVar("_TaskResult")


def run_candidate_task(
    parent: tk.Misc,
    *,
    title: str,
    task: Callable[..., _TaskResult],
    cancellable: bool = False,
) -> _TaskResult:
    """Run pure candidate work off the Tk thread while keeping UI responsive."""

    dialog = tk.Toplevel(parent)
    dialog.title(title)
    dialog.resizable(False, False)
    dialog.transient(parent)
    body = ttk.Frame(dialog, padding=14)
    body.pack(fill=tk.BOTH, expand=True)
    ttk.Label(body, text=f"{title} ...").pack(anchor="w")
    progress = ttk.Progressbar(body, mode="indeterminate", length=320)
    progress.pack(fill=tk.X, pady=(10, 0))
    progress.start(12)
    dialog.grab_set()

    completed = threading.Event()
    cancel_requested = threading.Event()
    outcome: dict[str, object] = {}

    def request_cancel() -> None:
        cancel_requested.set()
        if cancel_button is not None:
            cancel_button.configure(state=tk.DISABLED, text="Cancelling ...")

    cancel_button: Optional[ttk.Button] = None
    if cancellable:
        cancel_button = ttk.Button(body, text="Cancel", command=request_cancel)
        cancel_button.pack(anchor="e", pady=(10, 0))
        dialog.protocol("WM_DELETE_WINDOW", request_cancel)
    else:
        dialog.protocol("WM_DELETE_WINDOW", lambda: None)

    def worker() -> None:
        try:
            outcome["value"] = (
                task(cancel_requested) if cancellable else task()
            )
        except BaseException as error:  # noqa: BLE001
            outcome["error"] = error
        finally:
            completed.set()

    def poll() -> None:
        if completed.is_set():
            progress.stop()
            dialog.destroy()
            return
        dialog.after(25, poll)

    threading.Thread(target=worker, daemon=True).start()
    dialog.after(25, poll)
    parent.wait_window(dialog)
    if "error" in outcome:
        raise outcome["error"]  # type: ignore[misc]
    return outcome["value"]  # type: ignore[return-value]


class NormalizeOptionsDialog(tk.Toplevel):
    """Modal explicit-option dialog for one normalization candidate."""

    def __init__(
        self,
        parent: tk.Misc,
        *,
        circular: bool,
        speed_dirty: bool,
    ) -> None:
        super().__init__(parent)
        self.title("Normalize Geometry")
        self.resizable(False, False)
        self.transient(parent)
        self.result: Optional[NormalizeOptions] = None

        self.circular = circular
        self.remove_closure = tk.BooleanVar(value=True)
        self.remove_degenerate = tk.BooleanVar(value=True)
        self.resample = tk.BooleanVar(value=True)
        self.resolution = tk.StringVar(value=f"{LOCAL_RESOLUTION_M:g}")
        self.metadata_mode = tk.StringVar(
            value=(
                MetadataMode.RECOMPUTE.value
                if speed_dirty
                else MetadataMode.INTERPOLATE.value
            )
        )

        body = ttk.Frame(self, padding=12)
        body.pack(fill=tk.BOTH, expand=True)
        ttk.Label(
            body,
            text=f"{LOCAL_PRESET_NAME} (local candidate; not an official value)",
        ).grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 8))
        ttk.Label(
            body,
            text=(
                f"Topology: {'circular' if circular else 'open'} | "
                f"MPC horizon hint: {LOCAL_HORIZON_DISTANCE_M:g} m (read-only)"
            ),
        ).grid(row=1, column=0, columnspan=3, sticky="w", pady=(0, 8))

        ttk.Checkbutton(
            body,
            text="Remove legacy duplicate endpoint",
            variable=self.remove_closure,
        ).grid(row=2, column=0, columnspan=3, sticky="w")
        ttk.Checkbutton(
            body,
            text="Remove consecutive degenerate points (<= 1e-6 m)",
            variable=self.remove_degenerate,
        ).grid(row=3, column=0, columnspan=3, sticky="w")
        ttk.Checkbutton(
            body,
            text="Uniform linear resampling",
            variable=self.resample,
        ).grid(row=4, column=0, columnspan=3, sticky="w")
        ttk.Label(body, text="Resolution [m]").grid(
            row=5, column=0, sticky="w", padx=(24, 8)
        )
        ttk.Entry(body, textvariable=self.resolution, width=12).grid(
            row=5, column=1, sticky="w"
        )

        ttk.Label(body, text="Velocity metadata").grid(
            row=6, column=0, sticky="nw", pady=(8, 0)
        )
        preserve = ttk.Radiobutton(
            body,
            text="Preserve exact text (only unchanged point count/topology)",
            variable=self.metadata_mode,
            value=MetadataMode.PRESERVE.value,
        )
        preserve.grid(row=6, column=1, columnspan=2, sticky="w", pady=(8, 0))
        ttk.Radiobutton(
            body,
            text="Interpolate vx/ax along arc length",
            variable=self.metadata_mode,
            value=MetadataMode.INTERPOLATE.value,
        ).grid(row=7, column=1, columnspan=2, sticky="w")
        ttk.Radiobutton(
            body,
            text=(
                "Defer vx/ax: use interpolated placeholders and require "
                "Recompute Speed Profile"
            ),
            variable=self.metadata_mode,
            value=MetadataMode.RECOMPUTE.value,
        ).grid(row=8, column=1, columnspan=2, sticky="w")
        if speed_dirty:
            preserve.state(["disabled"])
            ttk.Label(
                body,
                text=(
                    "Current speed metadata is stale; choose interpolation or "
                    "defer to Recompute Speed Profile."
                ),
                foreground="#a85d00",
            ).grid(row=9, column=0, columnspan=3, sticky="w", pady=(4, 0))

        ttk.Checkbutton(
            body,
            text="Periodic spline (pending comparison evidence)",
            state="disabled",
        ).grid(row=10, column=0, columnspan=3, sticky="w", pady=(8, 0))
        ttk.Checkbutton(
            body,
            text="Curvature smoothing (pending displacement/lane checks)",
            state="disabled",
        ).grid(row=11, column=0, columnspan=3, sticky="w")

        buttons = ttk.Frame(body)
        buttons.grid(row=12, column=0, columnspan=3, sticky="e", pady=(12, 0))
        ttk.Button(buttons, text="Cancel", command=self._cancel).pack(
            side=tk.RIGHT, padx=(6, 0)
        )
        ttk.Button(buttons, text="Create Candidate", command=self._accept).pack(
            side=tk.RIGHT
        )

        self.protocol("WM_DELETE_WINDOW", self._cancel)
        self.bind("<Escape>", lambda _event: self._cancel())
        self.bind("<Return>", lambda _event: self._accept())
        self.grab_set()
        self.after_idle(self._center_on_parent)

    def _center_on_parent(self) -> None:
        self.update_idletasks()
        parent = self.master
        x = parent.winfo_rootx() + max(0, (parent.winfo_width() - self.winfo_width()) // 2)
        y = parent.winfo_rooty() + max(0, (parent.winfo_height() - self.winfo_height()) // 2)
        self.geometry(f"+{x}+{y}")

    def _accept(self) -> None:
        try:
            resolution = float(self.resolution.get())
            if not math.isfinite(resolution) or resolution <= 0.0:
                raise ValueError("resolution must be finite and positive")
            mode = MetadataMode(self.metadata_mode.get())
        except (TypeError, ValueError) as error:
            messagebox.showerror("Invalid Normalize settings", str(error), parent=self)
            return
        self.result = NormalizeOptions(
            circular=self.circular,
            metadata_mode=mode,
            remove_closure_duplicate=bool(self.remove_closure.get()),
            remove_degenerate_points=bool(self.remove_degenerate.get()),
            resample=bool(self.resample.get()),
            resolution_m=resolution,
        )
        self.destroy()

    def _cancel(self) -> None:
        self.result = None
        self.destroy()


def ask_normalize_options(
    parent: tk.Misc,
    *,
    circular: bool,
    speed_dirty: bool,
) -> Optional[NormalizeOptions]:
    dialog = NormalizeOptionsDialog(
        parent,
        circular=circular,
        speed_dirty=speed_dirty,
    )
    parent.wait_window(dialog)
    return dialog.result


class SpeedProfileDialog(tk.Toplevel):
    """Modal parameter dialog for the offline speed-profile candidate."""

    def __init__(
        self,
        parent: tk.Misc,
        *,
        circular: bool,
        defaults: Mapping[str, float],
    ) -> None:
        super().__init__(parent)
        self.title("Recompute Speed Profile")
        self.resizable(False, False)
        self.transient(parent)
        self.result: Optional[SpeedProfileParameters] = None
        self.values = {
            name: tk.StringVar(value=f"{defaults[name]:.8g}")
            for name in (
                "v_max_mps",
                "a_max_mps2",
                "a_min_mps2",
                "ay_max_mps2",
                "minimum_speed_mps",
            )
        }
        self.tolerance = tk.StringVar(value="1e-9")
        self.max_iterations = tk.StringVar(value="1000")

        body = ttk.Frame(self, padding=12)
        body.pack(fill=tk.BOTH, expand=True)
        ttk.Label(
            body,
            text=f"{LOCAL_PRESET_NAME} (local candidate; values remain editable)",
        ).grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 6))
        ttk.Label(
            body,
            text=(
                f"Topology: {'circular' if circular else 'open'} | "
                f"horizon hint: {LOCAL_HORIZON_DISTANCE_M:g} m (read-only)"
            ),
        ).grid(row=1, column=0, columnspan=3, sticky="w", pady=(0, 8))

        labels = (
            ("v_max_mps", "v_max", "m/s"),
            ("a_max_mps2", "a_max", "m/s^2"),
            ("a_min_mps2", "a_min", "m/s^2"),
            ("ay_max_mps2", "ay_max", "m/s^2"),
            ("minimum_speed_mps", "minimum_speed", "m/s"),
        )
        for row, (name, label, unit) in enumerate(labels, start=2):
            ttk.Label(body, text=label).grid(row=row, column=0, sticky="w", padx=(0, 8))
            ttk.Entry(body, textvariable=self.values[name], width=14).grid(
                row=row, column=1, sticky="w"
            )
            ttk.Label(body, text=unit).grid(row=row, column=2, sticky="w", padx=(6, 0))

        ttk.Label(body, text="convergence tolerance").grid(
            row=7, column=0, sticky="w", padx=(0, 8)
        )
        ttk.Entry(body, textvariable=self.tolerance, width=14).grid(
            row=7, column=1, sticky="w"
        )
        ttk.Label(body, text="m/s").grid(row=7, column=2, sticky="w", padx=(6, 0))
        ttk.Label(body, text="maximum iterations").grid(
            row=8, column=0, sticky="w", padx=(0, 8)
        )
        ttk.Entry(body, textvariable=self.max_iterations, width=14).grid(
            row=8, column=1, sticky="w"
        )
        ttk.Label(body, text="epsilon=1e-9 (internal curvature guard)").grid(
            row=9, column=0, columnspan=3, sticky="w", pady=(4, 0)
        )

        ttk.Label(
            body,
            text=(
                "Offline profile only; current C++ MPC runtime consumption is pending.\n"
                "a_max=1.0 is a local Candidate value, not a confirmed 2026 rule.\n"
                "Initial v_max/a_min/ay_max come from this CSV and are unverified."
            ),
            foreground="#a85d00",
        ).grid(row=10, column=0, columnspan=3, sticky="w", pady=(8, 0))

        buttons = ttk.Frame(body)
        buttons.grid(row=11, column=0, columnspan=3, sticky="e", pady=(12, 0))
        ttk.Button(buttons, text="Cancel", command=self._cancel).pack(
            side=tk.RIGHT, padx=(6, 0)
        )
        ttk.Button(buttons, text="Create Candidate", command=self._accept).pack(
            side=tk.RIGHT
        )

        self.protocol("WM_DELETE_WINDOW", self._cancel)
        self.bind("<Escape>", lambda _event: self._cancel())
        self.bind("<Return>", lambda _event: self._accept())
        self.grab_set()
        self.after_idle(self._center_on_parent)

    def _center_on_parent(self) -> None:
        self.update_idletasks()
        parent = self.master
        x = parent.winfo_rootx() + max(0, (parent.winfo_width() - self.winfo_width()) // 2)
        y = parent.winfo_rooty() + max(0, (parent.winfo_height() - self.winfo_height()) // 2)
        self.geometry(f"+{x}+{y}")

    def _accept(self) -> None:
        try:
            values = {name: float(variable.get()) for name, variable in self.values.items()}
            tolerance = float(self.tolerance.get())
            max_iterations = int(self.max_iterations.get())
            for name, value in (*values.items(), ("tolerance", tolerance)):
                if not math.isfinite(value):
                    raise ValueError(f"{name} must be finite")
            if values["v_max_mps"] <= 0.0:
                raise ValueError("v_max must be positive")
            if values["a_max_mps2"] <= 0.0:
                raise ValueError("a_max must be positive")
            if values["a_min_mps2"] >= 0.0:
                raise ValueError("a_min must be negative")
            if values["ay_max_mps2"] <= 0.0:
                raise ValueError("ay_max must be positive")
            if values["minimum_speed_mps"] < 0.0:
                raise ValueError("minimum_speed must be non-negative")
            if tolerance <= 0.0:
                raise ValueError("convergence tolerance must be positive")
            if max_iterations <= 0:
                raise ValueError("maximum iterations must be positive")
            self.result = SpeedProfileParameters(
                **values,
                tolerance=tolerance,
                max_iterations=max_iterations,
            )
        except (TypeError, ValueError) as error:
            messagebox.showerror("Invalid Speed settings", str(error), parent=self)
            self.result = None
            return
        self.destroy()

    def _cancel(self) -> None:
        self.result = None
        self.destroy()


def ask_speed_parameters(
    parent: tk.Misc,
    *,
    circular: bool,
    defaults: Mapping[str, float],
) -> Optional[SpeedProfileParameters]:
    dialog = SpeedProfileDialog(
        parent,
        circular=circular,
        defaults=defaults,
    )
    parent.wait_window(dialog)
    return dialog.result


class _ComparisonCanvas(tk.Canvas):
    def __init__(
        self,
        parent: tk.Misc,
        comparison: ComparisonPlotData,
        series_key: str,
        on_select: Callable[[str, int], None],
    ) -> None:
        super().__init__(parent, background="#12161d", highlightthickness=0)
        self.comparison = comparison
        self.series_key = series_key
        self.on_select = on_select
        self.selection: Optional[Tuple[int, int]] = None
        self.screen_points: list[Tuple[float, float, str, int]] = []
        self.bind("<Configure>", lambda _event: self.redraw())
        self.bind("<Button-1>", self._on_click)

    def set_selection(self, before_index: int, candidate_index: int) -> None:
        self.selection = (before_index, candidate_index)
        self.redraw()

    @staticmethod
    def _expanded_range(values: Sequence[float]) -> Tuple[float, float]:
        minimum = min(values)
        maximum = max(values)
        if math.isclose(minimum, maximum, rel_tol=0.0, abs_tol=1e-12):
            padding = max(1.0, abs(minimum) * 0.1)
            return minimum - padding, maximum + padding
        padding = (maximum - minimum) * 0.05
        return minimum - padding, maximum + padding

    def _raw_series(self, role: str) -> Tuple[list[float], list[float], list[int], str]:
        plot = self.comparison.before if role == "before" else self.comparison.candidate
        if self.series_key == "xy":
            return (
                [point.x_m for point in plot.xy.points],
                [point.y_m for point in plot.xy.points],
                [point.point_index for point in plot.xy.points],
                "XY path [m]",
            )
        series: ScalarPlotSeries = plot.scalar_series(self.series_key)
        if not series.available:
            return [], [], [], series.unavailable_reason or "Unavailable"
        return (
            [sample.s_m for sample in series.samples],
            [sample.value for sample in series.samples],
            [sample.point_index for sample in series.samples],
            f"{series.y_label} [{series.y_unit}] vs s [m]",
        )

    def redraw(self) -> None:
        self.delete("all")
        self.screen_points.clear()
        width = max(self.winfo_width(), 200)
        height = max(self.winfo_height(), 160)
        left, right, top, bottom = 70.0, width - 20.0, 28.0, height - 45.0

        before_x, before_y, before_indices, title = self._raw_series("before")
        candidate_x, candidate_y, candidate_indices, candidate_title = self._raw_series(
            "candidate"
        )
        if not before_x and not candidate_x:
            self.create_text(
                width / 2,
                height / 2,
                text=candidate_title or title,
                fill="#c5ccd8",
            )
            return
        all_x = before_x + candidate_x
        all_y = before_y + candidate_y
        x_min, x_max = self._expanded_range(all_x)
        y_min, y_max = self._expanded_range(all_y)

        if self.series_key == "xy":
            common_scale = min(
                (right - left) / (x_max - x_min),
                (bottom - top) / (y_max - y_min),
            )
            x_center = 0.5 * (x_min + x_max)
            y_center = 0.5 * (y_min + y_max)
            screen_center_x = 0.5 * (left + right)
            screen_center_y = 0.5 * (top + bottom)

            def screen(x_value: float, y_value: float) -> Tuple[float, float]:
                return (
                    screen_center_x + (x_value - x_center) * common_scale,
                    screen_center_y - (y_value - y_center) * common_scale,
                )

        else:

            def screen(x_value: float, y_value: float) -> Tuple[float, float]:
                sx = left + (x_value - x_min) * (right - left) / (x_max - x_min)
                sy = bottom - (y_value - y_min) * (bottom - top) / (y_max - y_min)
                return sx, sy

        self.create_line(left, bottom, right, bottom, fill="#6d7787")
        self.create_line(left, top, left, bottom, fill="#6d7787")
        self.create_text(left, 12, text=title, fill="#e3e8ef", anchor="w")
        self.create_text(left, height - 18, text=f"{x_min:.5g}", fill="#aeb7c5")
        self.create_text(right, height - 18, text=f"{x_max:.5g}", fill="#aeb7c5")
        self.create_text(8, bottom, text=f"{y_min:.5g}", fill="#aeb7c5", anchor="w")
        self.create_text(8, top, text=f"{y_max:.5g}", fill="#aeb7c5", anchor="w")

        for role, xs, ys, indices, color in (
            ("before", before_x, before_y, before_indices, _BEFORE_COLOR),
            ("candidate", candidate_x, candidate_y, candidate_indices, _CANDIDATE_COLOR),
        ):
            coords: list[float] = []
            role_points: list[Tuple[float, float]] = []
            step = max(1, int(math.ceil(len(xs) / _MAX_CANVAS_SAMPLES)))
            positions = list(range(0, len(xs), step))
            if positions and positions[-1] != len(xs) - 1:
                positions.append(len(xs) - 1)
            if self.selection is not None:
                selected_index = (
                    self.selection[0] if role == "before" else self.selection[1]
                )
                positions.extend(
                    position
                    for position, point_index in enumerate(indices)
                    if point_index == selected_index
                )
                positions = sorted(set(positions))
            for position in positions:
                x_value = xs[position]
                y_value = ys[position]
                point_index = indices[position]
                sx, sy = screen(x_value, y_value)
                coords.extend((sx, sy))
                role_points.append((sx, sy))
                self.screen_points.append((sx, sy, role, point_index))
            if self.series_key == "xy":
                plot = (
                    self.comparison.before
                    if role == "before"
                    else self.comparison.candidate
                )
                if plot.circular and role_points:
                    coords.extend(role_points[0])
            if len(coords) >= 4:
                self.create_line(*coords, fill=color, width=2)

        self.create_line(right - 155, top + 5, right - 125, top + 5, fill=_BEFORE_COLOR, width=2)
        self.create_text(right - 120, top + 5, text="Before", fill="#d4d9e1", anchor="w")
        self.create_line(right - 72, top + 5, right - 42, top + 5, fill=_CANDIDATE_COLOR, width=2)
        self.create_text(right - 37, top + 5, text="Candidate", fill="#d4d9e1", anchor="w")

        if self.selection is not None:
            selected = {"before": self.selection[0], "candidate": self.selection[1]}
            for sx, sy, role, point_index in self.screen_points:
                if selected[role] == point_index:
                    self.create_oval(
                        sx - 5,
                        sy - 5,
                        sx + 5,
                        sy + 5,
                        outline=_SELECTION_COLOR,
                        width=2,
                    )

    def _on_click(self, event: tk.Event) -> None:
        if not self.screen_points:
            return
        sx, sy, role, index = min(
            self.screen_points,
            key=lambda item: (item[0] - event.x) ** 2 + (item[1] - event.y) ** 2,
        )
        if (sx - event.x) ** 2 + (sy - event.y) ** 2 <= 20.0 ** 2:
            self.on_select(role, index)


class CandidatePreviewDialog(tk.Toplevel):
    """Modal seven-view before/candidate preview with synchronized selection."""

    def __init__(
        self,
        parent: tk.Misc,
        *,
        comparison: ComparisonPlotData,
        validation: ValidationReport,
        transformation: object,
        operation: str,
        parameters: Mapping[str, object],
    ) -> None:
        super().__init__(parent)
        self.title(f"Candidate Preview - {operation}")
        self.geometry("1180x850")
        self.transient(parent)
        self.applied = False
        self.comparison = comparison
        self.validation = validation
        self.canvases: list[_ComparisonCanvas] = []
        self._syncing_issue = False

        summary = comparison.summary
        header = ttk.Frame(self, padding=8)
        header.pack(side=tk.TOP, fill=tk.X)
        ttk.Label(
            header,
            text=(
                f"{operation}: points {summary.before_point_count} -> "
                f"{summary.candidate_point_count}, length "
                f"{summary.before_path_length_m:.3f} -> "
                f"{summary.candidate_path_length_m:.3f} m, "
                f"max same-s interpolated displacement "
                f"{summary.max_displacement_m:.4f} m"
            ),
        ).pack(anchor="w")
        ttk.Label(
            header,
            text=self._report_text(transformation, parameters),
            wraplength=1140,
            justify=tk.LEFT,
        ).pack(anchor="w")
        self.selection_text = tk.StringVar(value="Click a graph or issue to synchronize selection")
        ttk.Label(header, textvariable=self.selection_text).pack(anchor="w")

        metric_columns = ("metric", "before", "candidate", "difference")
        metric_tree = ttk.Treeview(
            self,
            columns=metric_columns,
            show="headings",
            height=8,
        )
        for column, width in zip(metric_columns, (250, 280, 280, 280)):
            metric_tree.heading(column, text=column)
            metric_tree.column(column, width=width, stretch=True)
        for values in self._summary_rows(comparison, validation):
            metric_tree.insert("", tk.END, values=values)
        metric_tree.pack(side=tk.TOP, fill=tk.X, padx=8, pady=(0, 4))

        notebook = ttk.Notebook(self)
        notebook.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=8)
        for key, label in (
            ("xy", "XY"),
            ("spacing", "Spacing"),
            ("psi", "Psi"),
            ("kappa", "Kappa"),
            ("velocity", "Velocity"),
            ("acceleration", "Acceleration"),
            ("lateral_acceleration", "Lateral Accel"),
        ):
            frame = ttk.Frame(notebook)
            notebook.add(frame, text=label)
            canvas = _ComparisonCanvas(frame, comparison, key, self._select)
            canvas.pack(fill=tk.BOTH, expand=True)
            self.canvases.append(canvas)

        issue_frame = ttk.Frame(self, padding=(8, 4))
        issue_frame.pack(side=tk.TOP, fill=tk.X)
        columns = ("severity", "code", "line", "s_m", "message")
        self.issue_tree = ttk.Treeview(
            issue_frame,
            columns=columns,
            show="headings",
            height=5,
        )
        for column, width in zip(columns, (70, 210, 55, 90, 620)):
            self.issue_tree.heading(column, text=column)
            self.issue_tree.column(column, width=width, stretch=column == "message")
        self.issue_map: dict[str, ValidationIssue] = {}
        for issue in validation.issues:
            item = self.issue_tree.insert(
                "",
                tk.END,
                values=(
                    issue.severity.value,
                    issue.code,
                    issue.line_number or "",
                    "" if issue.s_m is None else f"{issue.s_m:.6g}",
                    issue.message,
                ),
            )
            self.issue_map[item] = issue
        self.issue_tree.bind("<<TreeviewSelect>>", self._on_issue)
        self.issue_tree.pack(fill=tk.X, expand=True)

        buttons = ttk.Frame(self, padding=8)
        buttons.pack(side=tk.BOTTOM, fill=tk.X)
        ttk.Button(buttons, text="Discard", command=self._discard).pack(side=tk.RIGHT)
        apply_button = ttk.Button(buttons, text="Apply Candidate", command=self._apply)
        apply_button.pack(side=tk.RIGHT, padx=(0, 8))
        if not validation.is_valid:
            apply_button.state(["disabled"])

        self.protocol("WM_DELETE_WINDOW", self._discard)
        self.bind("<Escape>", lambda _event: self._discard())
        self.grab_set()

    @staticmethod
    def _report_text(transformation: object, parameters: Mapping[str, object]) -> str:
        facts = []
        if hasattr(transformation, "__dataclass_fields__"):
            for field in fields(transformation):
                if field.name == "retained_source_indices":
                    facts.append(
                        "retained_source_count="
                        f"{len(getattr(transformation, field.name))}"
                    )
                    continue
                if field.name in {
                    "removed_closure_indices",
                    "removed_degenerate_indices",
                    "output_min_spacing_m",
                    "output_max_spacing_m",
                    "output_closing_spacing_m",
                }:
                    value = getattr(transformation, field.name)
                    facts.append(f"{field.name}={value}")
        facts.extend(f"{key}={value}" for key, value in parameters.items())
        return " | ".join(facts)

    @staticmethod
    def _summary_rows(
        comparison: ComparisonPlotData,
        validation: ValidationReport,
    ) -> list[Tuple[str, str, str, str]]:
        summary = comparison.summary

        def number(value: Optional[float], unit: str = "") -> str:
            if value is None:
                return "--"
            return f"{value:.7g}{(' ' + unit) if unit else ''}"

        def range_text(key: str) -> Tuple[str, str]:
            series = summary.series(key)

            def one(extrema: object) -> str:
                minimum = getattr(extrema, "minimum")
                maximum = getattr(extrema, "maximum")
                maximum_absolute = getattr(extrema, "maximum_absolute")
                if minimum is None or maximum is None:
                    return "unavailable"
                return (
                    f"min={minimum:.7g}, max={maximum:.7g}, "
                    f"max|.|={maximum_absolute:.7g} {series.unit}"
                )

            return one(series.before), one(series.candidate)

        spacing_before, spacing_candidate = range_text("spacing")
        spacing_before += (
            ", mean="
            + number(comparison.before.metrics.mean_spacing_m, "m")
        )
        spacing_candidate += (
            ", mean="
            + number(comparison.candidate.metrics.mean_spacing_m, "m")
        )
        rows: list[Tuple[str, str, str, str]] = [
            (
                "Point count (raw / normalized)",
                f"{summary.before_point_count} / {summary.before_normalized_point_count}",
                f"{summary.candidate_point_count} / {summary.candidate_normalized_point_count}",
                f"raw delta={summary.point_count_delta:+d}",
            ),
            (
                "Path length",
                number(summary.before_path_length_m, "m"),
                number(summary.candidate_path_length_m, "m"),
                number(summary.path_length_delta_m, "m"),
            ),
            (
                "Waypoint spacing",
                spacing_before,
                spacing_candidate,
                "includes circular seam",
            ),
        ]
        for key, label in (
            ("kappa", "Curvature"),
            ("velocity", "Velocity"),
            ("acceleration", "Longitudinal acceleration"),
            ("lateral_acceleration", "Lateral acceleration"),
        ):
            before_value, candidate_value = range_text(key)
            rows.append((label, before_value, candidate_value, ""))
        rows.extend(
            [
                (
                    "Maximum mapped XY displacement",
                    "--",
                    number(summary.max_displacement_m, "m"),
                    "same-s polyline interpolation, symmetric",
                ),
                (
                    "Candidate validation",
                    "--",
                    (
                        f"errors={validation.error_count}, "
                        f"warnings={validation.warning_count}, "
                        f"info={validation.info_count}"
                    ),
                    "Apply disabled when errors > 0",
                ),
            ]
        )
        return rows

    def _select(self, role: str, point_index: int) -> None:
        try:
            mapping = (
                self.comparison.selection_from_before(point_index)
                if role == "before"
                else self.comparison.selection_from_candidate(point_index)
            )
        except (IndexError, ValueError):
            return
        for canvas in self.canvases:
            canvas.set_selection(mapping.before_index, mapping.candidate_index)
        matching_items = []
        for item, issue in self.issue_map.items():
            issue_index = issue.point_index
            if issue_index is None and issue.segment_index is not None:
                issue_index = issue.segment_index
            if issue_index == mapping.candidate_index:
                matching_items.append(item)
        self._syncing_issue = True
        try:
            if matching_items:
                item = matching_items[0]
                if self.issue_tree.selection() != (item,):
                    self.issue_tree.selection_set(item)
                self.issue_tree.focus(item)
                self.issue_tree.see(item)
            else:
                self.issue_tree.selection_remove(*self.issue_tree.selection())
        finally:
            self._syncing_issue = False
        self.selection_text.set(
            f"Before index={mapping.before_index}, s={mapping.before_s_m:.4f} m | "
            f"Candidate index={mapping.candidate_index}, "
            f"s={mapping.candidate_s_m:.4f} m"
        )

    def _on_issue(self, _event: tk.Event) -> None:
        if self._syncing_issue:
            return
        selected = self.issue_tree.selection()
        if not selected:
            return
        issue = self.issue_map.get(selected[0])
        if issue is None:
            return
        index = issue.point_index
        if index is None and issue.segment_index is not None:
            index = issue.segment_index
        if index is not None:
            self._select("candidate", index)

    def _apply(self) -> None:
        self.applied = True
        self.destroy()

    def _discard(self) -> None:
        self.applied = False
        self.destroy()


def preview_candidate(
    parent: tk.Misc,
    *,
    comparison: ComparisonPlotData,
    validation: ValidationReport,
    transformation: object,
    operation: str,
    parameters: Mapping[str, object],
) -> bool:
    dialog = CandidatePreviewDialog(
        parent,
        comparison=comparison,
        validation=validation,
        transformation=transformation,
        operation=operation,
        parameters=parameters,
    )
    parent.wait_window(dialog)
    return dialog.applied
