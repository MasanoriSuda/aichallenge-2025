import copy
import csv
import math
from pathlib import Path

import pytest

from multi_purpose_mpc_ros.tools import trajectory_contract as contract
from multi_purpose_mpc_ros.tools import trajectory_editor as editor
from multi_purpose_mpc_ros.tools.trajectory_processing import NormalizeOptions
from multi_purpose_mpc_ros.tools.trajectory_speed import SpeedProfileParameters


def _write_rows(
    path: Path,
    fieldnames: tuple[str, ...],
    rows: list[dict[str, str]],
) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def _mpc_row(s: float, x: float, y: float) -> dict[str, str]:
    return {
        "s_m": str(s),
        "x_m": str(x),
        "y_m": str(y),
        "psi_rad": "0",
        "kappa_radpm": "0",
        "vx_mps": "2",
        "ax_mps2": "0",
    }


def _pure_pursuit_row(x: float, y: float, speed: str) -> dict[str, str]:
    return {
        "x": str(x),
        "y": str(y),
        "z": "0.125",
        "x_quat": "0",
        "y_quat": "0",
        "z_quat": "0.25",
        "w_quat": "0.9682458366",
        "speed": speed,
    }


def test_load_accepts_runtime_header_whitespace_and_bom(tmp_path: Path) -> None:
    source = tmp_path / "trajectory.csv"
    source.write_text(
        "  \ufeff s_m , x_m , y_m , psi_rad , kappa_radpm , vx_mps , ax_mps2\n"
        "0,0,0,0,0,1,0\n"
        "1,1,0,0,0,1,0\n",
        encoding="utf-8",
    )

    data = editor.load_trajectory(source)

    assert data.fieldnames == list(contract.MPC_COLUMNS)
    assert data.points == [(0.0, 0.0), (1.0, 0.0)]


def test_editor_startup_path_retains_raw_blank_row_validation(tmp_path: Path) -> None:
    source = tmp_path / "trajectory.csv"
    source.write_text(
        ",".join(contract.MPC_COLUMNS)
        + "\n0,0,0,0,0,1,0\n\n1,1,0,0,0,1,0\n",
        encoding="utf-8",
    )

    _data, _topology, report = editor._load_editor_trajectory(
        source,
        circular=False,
    )

    assert not report.is_valid
    assert "BLANK_DATA_ROW" in {issue.code for issue in report.issues}


def test_compatibility_loader_rejects_nonfinite_xy_before_gui_fit(tmp_path: Path) -> None:
    source = tmp_path / "trajectory.csv"
    source.write_text(
        ",".join(contract.MPC_COLUMNS)
        + "\n0,nan,0,0,0,1,0\n1,1,0,0,0,1,0\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="Non-finite"):
        editor.load_trajectory(source)


def test_mpc_save_canonicalizes_order_and_revalidates(tmp_path: Path) -> None:
    source = tmp_path / "source.csv"
    output = tmp_path / "edited.csv"
    reversed_columns = tuple(reversed(contract.MPC_COLUMNS))
    _write_rows(
        source,
        reversed_columns,
        [_mpc_row(0, 0, 0), _mpc_row(1, 1, 0)],
    )
    data = editor.load_trajectory(source)

    report = editor.save_trajectory(data, output, circular=False)

    assert report.is_valid
    assert output.read_text(encoding="utf-8").splitlines()[0] == ",".join(
        contract.MPC_COLUMNS
    )
    assert data.path == output
    assert data.fieldnames == list(contract.MPC_COLUMNS)
    assert contract.validate_csv_file(output, circular=False).is_valid


def test_failed_save_keeps_document_and_existing_target_unchanged(
    tmp_path: Path,
) -> None:
    source = tmp_path / "source.csv"
    target = tmp_path / "target.csv"
    _write_rows(
        source,
        contract.MPC_COLUMNS,
        [_mpc_row(0, 0, 0), _mpc_row(1, 1, 0)],
    )
    target.write_bytes(b"existing target\n")
    data = editor.load_trajectory(source)
    data.rows[0]["vx_mps"] = "nan"
    before = copy.deepcopy(data)

    with pytest.raises(ValueError, match="trajectory validation failed"):
        editor.save_trajectory(data, target, circular=False)

    assert data == before
    assert target.read_bytes() == b"existing target\n"


def test_pure_pursuit_save_keeps_eight_columns_and_metadata(tmp_path: Path) -> None:
    source = tmp_path / "pure.csv"
    output = tmp_path / "pure_edited.csv"
    rows = [
        _pure_pursuit_row(0, 0, "3.25"),
        _pure_pursuit_row(1, 0, "4.5"),
    ]
    _write_rows(source, contract.PURE_PURSUIT_COLUMNS, rows)
    data = editor.load_trajectory(source)
    data.points[1] = (1.25, 0.5)

    editor.save_trajectory(data, output, recompute=False, circular=False)

    with output.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        output_rows = list(reader)
        assert tuple(reader.fieldnames or ()) == contract.PURE_PURSUIT_COLUMNS
    assert output_rows[0]["speed"] == "3.25"
    assert output_rows[1]["speed"] == "4.5"
    assert output_rows[1]["z_quat"] == "0.25"
    assert float(output_rows[1]["x"]) == pytest.approx(1.25)
    assert float(output_rows[1]["y"]) == pytest.approx(0.5)


def test_unique_endpoint_circular_recompute_wraps_geometry(tmp_path: Path) -> None:
    source = tmp_path / "triangle.csv"
    _write_rows(
        source,
        contract.MPC_COLUMNS,
        [_mpc_row(0, 0, 0), _mpc_row(1, 1, 0), _mpc_row(2, 0, 1)],
    )
    circular = editor.load_trajectory(source)
    open_path = copy.deepcopy(circular)

    editor.recompute_geometry(circular, circular=True)
    editor.recompute_geometry(open_path, circular=False)

    assert float(circular.rows[0]["psi_rad"]) == pytest.approx(
        -math.pi / 4.0, abs=1e-6
    )
    assert float(open_path.rows[0]["psi_rad"]) == pytest.approx(0.0, abs=1e-9)


class _Value:
    def __init__(self, value: object) -> None:
        self.value = value

    def get(self) -> object:
        return self.value

    def set(self, value: object) -> None:
        self.value = value


def _headless_editor(data: editor.TrajectoryData) -> editor.TrajectoryEditor:
    instance = editor.TrajectoryEditor.__new__(editor.TrajectoryEditor)
    instance.trajectory = data
    instance.loaded_original = copy.deepcopy(data)
    instance.original_trajectory = instance.loaded_original
    instance.loaded_original_circular = True
    instance.circular = _Value(True)
    instance.influence_radius_points = _Value(1)
    instance.last_operation = "edited"
    instance.candidate = None
    instance.world_to_screen = lambda point: point
    return instance


class _CanvasSize:
    def __init__(self, width: int, height: int) -> None:
        self.width = width
        self.height = height

    def winfo_width(self) -> int:
        return self.width

    def winfo_height(self) -> int:
        return self.height


def _prepare_headless_save(
    instance: editor.TrajectoryEditor,
    report: contract.ValidationReport,
) -> None:
    instance.geometry_dirty = False
    instance.speed_dirty = False
    instance.dirty = True
    instance.revision = 4
    instance.undo_stack = [object()]
    instance.validation_report = report
    instance.validation_revision = instance.revision
    instance._clear_validation = lambda _reason: None
    instance.validate_current = lambda: report
    instance.redraw = lambda: None
    instance._set_status = lambda _extra="": None


def test_unique_circular_seam_is_selectable_and_influence_wraps(
    tmp_path: Path,
) -> None:
    source = tmp_path / "triangle.csv"
    _write_rows(
        source,
        contract.MPC_COLUMNS,
        [_mpc_row(0, 0, 0), _mpc_row(2, 2, 0), _mpc_row(4, 2, 2)],
    )
    instance = _headless_editor(editor.load_trajectory(source))

    assert instance._nearest_segment(1.0, 1.0) == 2
    assert instance._influenced_indices(0) == {1, 2}

    before = list(instance.trajectory.points)
    instance._apply_influenced_delta(before, selected_index=0, dx=1.0, dy=0.0)
    assert instance.trajectory.points[0][0] == pytest.approx(1.0)
    assert instance.trajectory.points[-1][0] > before[-1][0]
    assert instance.trajectory.points[-1] != instance.trajectory.points[0]


def test_validate_current_preserves_working_undo_dirty_revision_and_file(
    tmp_path: Path,
) -> None:
    source = tmp_path / "line.csv"
    _write_rows(
        source,
        contract.MPC_COLUMNS,
        [_mpc_row(0, 0, 0), _mpc_row(1, 1, 0)],
    )
    instance = _headless_editor(editor.load_trajectory(source))
    instance.circular = _Value(False)
    instance.dirty = True
    instance.geometry_dirty = False
    instance.speed_dirty = False
    instance.revision = 12
    instance.undo_stack = ["sentinel"]
    instance._show_validation_report = lambda report, *_args, **_kwargs: setattr(
        instance, "validation_report", report
    )
    instance._set_status = lambda _extra="": None
    before_data = copy.deepcopy(instance.trajectory)
    before_state = (
        list(instance.undo_stack),
        instance.dirty,
        instance.geometry_dirty,
        instance.speed_dirty,
        instance.revision,
        source.read_bytes(),
        source.stat().st_mtime_ns,
    )

    report = instance.validate_current()

    assert report.is_valid
    assert instance.trajectory == before_data
    assert (
        list(instance.undo_stack),
        instance.dirty,
        instance.geometry_dirty,
        instance.speed_dirty,
        instance.revision,
        source.read_bytes(),
        source.stat().st_mtime_ns,
    ) == before_state


def test_undo_restores_dirty_state_but_revision_remains_monotonic(
    tmp_path: Path,
) -> None:
    source = tmp_path / "line.csv"
    _write_rows(
        source,
        contract.MPC_COLUMNS,
        [_mpc_row(0, 0, 0), _mpc_row(1, 1, 0)],
    )
    instance = _headless_editor(editor.load_trajectory(source))
    instance.selected_index = 0
    instance.dirty = False
    instance.geometry_dirty = False
    instance.speed_dirty = False
    instance.revision = 7
    instance.undo_stack = []
    instance._clear_validation = lambda _reason: None
    instance.redraw = lambda: None
    instance._set_status = lambda _extra="": None

    instance._push_undo()
    instance.trajectory.points[0] = (9.0, 9.0)
    instance.dirty = True
    instance.geometry_dirty = True
    instance.revision += 1
    instance.undo()

    assert instance.trajectory.points[0] == (0.0, 0.0)
    assert not instance.dirty
    assert not instance.geometry_dirty
    assert instance.revision == 9


def test_topology_change_marks_geometry_and_speed_stale(tmp_path: Path) -> None:
    source = tmp_path / "line.csv"
    _write_rows(
        source,
        contract.MPC_COLUMNS,
        [_mpc_row(0, 0, 0), _mpc_row(1, 1, 0), _mpc_row(2, 2, 0)],
    )
    instance = _headless_editor(editor.load_trajectory(source))
    instance.dirty = False
    instance.geometry_dirty = False
    instance.speed_dirty = False
    instance.revision = 0
    instance._clear_validation = lambda _reason: None
    instance.validation_summary = _Value("")
    instance.validation_summary.set = lambda value: setattr(
        instance.validation_summary, "value", value
    )
    instance.redraw = lambda: None
    instance._set_status = lambda _extra="": None

    instance._on_circular_changed()

    assert instance.dirty
    assert instance.geometry_dirty
    assert instance.speed_dirty
    assert instance.revision == 1


def test_mpc_geometry_change_always_marks_speed_metadata_stale(
    tmp_path: Path,
) -> None:
    source = tmp_path / "line.csv"
    _write_rows(
        source,
        contract.MPC_COLUMNS,
        [_mpc_row(0, 0, 0), _mpc_row(1, 1, 0)],
    )
    instance = _headless_editor(editor.load_trajectory(source))
    instance.dirty = False
    instance.geometry_dirty = False
    instance.speed_dirty = False
    instance.revision = 0
    instance._clear_validation = lambda _reason: None

    instance._mark_modified(geometry_dirty=True)

    assert instance.geometry_dirty
    assert instance.speed_dirty
    assert instance.revision == 1


def test_invalid_geometry_recompute_keeps_working_and_undo_unchanged(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    source = tmp_path / "duplicate.csv"
    _write_rows(
        source,
        contract.MPC_COLUMNS,
        [_mpc_row(0, 0, 0), _mpc_row(1, 0, 0)],
    )
    instance = _headless_editor(editor.load_trajectory(source))
    instance.circular = _Value(False)
    instance.selected_index = None
    instance.dirty = False
    instance.geometry_dirty = False
    instance.speed_dirty = False
    instance.revision = 3
    instance.undo_stack = ["sentinel"]
    instance._show_validation_report = lambda *_args, **_kwargs: None
    instance._set_status = lambda _extra="": None
    monkeypatch.setattr(editor.messagebox, "showerror", lambda *_args, **_kwargs: None)
    before = copy.deepcopy(instance.trajectory)

    instance.recompute_derived_geometry()

    assert instance.trajectory == before
    assert instance.undo_stack == ["sentinel"]
    assert not instance.dirty
    assert not instance.geometry_dirty
    assert not instance.speed_dirty
    assert instance.revision == 3


def test_only_normalization_repairable_mpc_errors_may_open_for_repair(
    tmp_path: Path,
) -> None:
    duplicate = tmp_path / "duplicate.csv"
    malformed = tmp_path / "malformed.csv"
    _write_rows(
        duplicate,
        contract.MPC_COLUMNS,
        [_mpc_row(0, 0, 0), _mpc_row(1, 0, 0), _mpc_row(2, 1, 0)],
    )
    _write_rows(
        malformed,
        contract.MPC_COLUMNS,
        [_mpc_row(0, 0, 0), _mpc_row(1, 1, 0)],
    )
    malformed_data = editor.load_trajectory(malformed)
    malformed_data.rows[1]["vx_mps"] = "nan"
    derived_data = editor.load_trajectory(malformed)
    derived_data.rows[0]["s_m"] = "nan"

    duplicate_data = editor.load_trajectory(duplicate)
    duplicate_report = editor.validate_trajectory_data(
        duplicate_data,
        circular=False,
    )
    malformed_report = editor.validate_trajectory_data(
        malformed_data,
        circular=False,
    )
    derived_report = editor.validate_trajectory_data(
        derived_data,
        circular=False,
    )

    assert editor._is_normalization_repairable(
        duplicate_data,
        duplicate_report,
    )
    assert not editor._is_normalization_repairable(
        malformed_data,
        malformed_report,
    )
    assert editor._is_normalization_repairable(
        derived_data,
        derived_report,
    )


def test_overwrite_cancel_does_not_call_serializer_or_change_target(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    source = tmp_path / "source.csv"
    target = tmp_path / "existing.csv"
    _write_rows(
        source,
        contract.MPC_COLUMNS,
        [_mpc_row(0, 0, 0), _mpc_row(1, 1, 0)],
    )
    target.write_bytes(b"existing\n")
    instance = _headless_editor(editor.load_trajectory(source))
    report = editor.validate_trajectory_data(instance.trajectory, circular=False)
    _prepare_headless_save(instance, report)
    monkeypatch.setattr(editor.messagebox, "askyesno", lambda *_args, **_kwargs: False)

    def serializer_must_not_run(*_args: object, **_kwargs: object) -> None:
        raise AssertionError("serializer was called after overwrite cancellation")

    monkeypatch.setattr(editor, "save_trajectory", serializer_must_not_run)

    assert not instance._save_to_path(target)
    assert target.read_bytes() == b"existing\n"
    assert instance.trajectory.path == source


def test_stale_geometry_blocks_save_before_validation_or_temp_creation(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    source = tmp_path / "source.csv"
    target = tmp_path / "must_not_exist.csv"
    _write_rows(
        source,
        contract.MPC_COLUMNS,
        [_mpc_row(0, 0, 0), _mpc_row(1, 1, 0)],
    )
    instance = _headless_editor(editor.load_trajectory(source))
    instance.geometry_dirty = True
    instance.speed_dirty = False
    instance._set_status = lambda _extra="": None
    instance.validate_current = lambda: (_ for _ in ()).throw(
        AssertionError("validation ran for a stale document")
    )
    monkeypatch.setattr(editor.messagebox, "showerror", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(
        editor,
        "save_trajectory",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("serializer ran for a stale document")
        ),
    )

    assert not instance._save_to_path(target)
    assert not target.exists()


def test_successful_ui_save_updates_state_only_after_atomic_write(
    tmp_path: Path,
) -> None:
    source = tmp_path / "source.csv"
    target = tmp_path / "edited.csv"
    _write_rows(
        source,
        contract.MPC_COLUMNS,
        [_mpc_row(0, 0, 0), _mpc_row(1, 1, 0)],
    )
    instance = _headless_editor(editor.load_trajectory(source))
    instance.circular = _Value(False)
    report = editor.validate_trajectory_data(instance.trajectory, circular=False)
    _prepare_headless_save(instance, report)

    assert instance._save_to_path(target)
    assert contract.validate_csv_file(target, circular=False).is_valid
    assert instance.trajectory.path == target
    assert not instance.dirty
    assert not instance.geometry_dirty
    assert not instance.speed_dirty
    assert not instance.undo_stack
    assert instance.revision == 5


def test_successful_save_keeps_loaded_original_session_snapshot(
    tmp_path: Path,
) -> None:
    source = tmp_path / "source.csv"
    target = tmp_path / "edited.csv"
    _write_rows(
        source,
        contract.MPC_COLUMNS,
        [_mpc_row(0, 0, 0), _mpc_row(1, 1, 0)],
    )
    instance = _headless_editor(editor.load_trajectory(source))
    instance.circular = _Value(False)
    original_signature = editor._trajectory_content_signature(instance.loaded_original)
    instance.trajectory.rows[0]["vx_mps"] = "1.5"
    report = editor.validate_trajectory_data(instance.trajectory, circular=False)
    _prepare_headless_save(instance, report)

    assert instance._save_to_path(target)
    assert instance.trajectory.rows[0]["vx_mps"] == "1.5"
    assert editor._trajectory_content_signature(instance.loaded_original) == original_signature
    assert instance.original_trajectory is instance.loaded_original
    assert instance.loaded_original.path == source


def test_scroll_navigation_preserves_world_screen_round_trip_and_zoom_anchor(
    tmp_path: Path,
) -> None:
    source = tmp_path / "line.csv"
    _write_rows(
        source,
        contract.MPC_COLUMNS,
        [_mpc_row(0, 0, 0), _mpc_row(100, 100, 50)],
    )
    instance = _headless_editor(editor.load_trajectory(source))
    instance.world_to_screen = editor.TrajectoryEditor.world_to_screen.__get__(instance)
    instance.canvas = _CanvasSize(400, 300)
    instance.rails = []
    instance.show_original = _Value(True)
    instance.show_working = _Value(True)
    instance.show_candidate = _Value(True)
    instance.center_x = 50.0
    instance.center_y = 25.0
    instance.scale = 10.0
    instance.redraw = lambda: None

    instance._set_scroll_fraction("x", 1.0)
    instance._set_scroll_fraction("y", 1.0)
    world = instance.screen_to_world(125.0, 90.0)
    assert instance.world_to_screen(world) == pytest.approx((125.0, 90.0))

    anchor_before = instance.screen_to_world(260.0, 175.0)
    instance._zoom_at(260.0, 175.0, 1.15)
    assert instance.screen_to_world(260.0, 175.0) == pytest.approx(anchor_before)


def test_layer_toggle_is_view_only(tmp_path: Path) -> None:
    source = tmp_path / "line.csv"
    _write_rows(
        source,
        contract.MPC_COLUMNS,
        [_mpc_row(0, 0, 0), _mpc_row(1, 1, 0)],
    )
    instance = _headless_editor(editor.load_trajectory(source))
    instance.show_original = _Value(False)
    instance.dirty = True
    instance.revision = 11
    instance.undo_stack = ["sentinel"]
    instance.redraw = lambda: None
    instance._set_status = lambda _extra="": None
    before = copy.deepcopy(instance.trajectory)

    instance._on_layer_visibility_changed()

    assert instance.trajectory == before
    assert instance.dirty
    assert instance.revision == 11
    assert instance.undo_stack == ["sentinel"]


def test_original_difference_uses_arc_length_when_point_counts_differ() -> None:
    original = [(0.0, 0.0), (2.0, 0.0)]
    unchanged_resample = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    changed = [(0.0, 0.0), (1.0, 0.2), (2.0, 0.0)]

    same = editor.build_original_difference(original, unchanged_resample)
    difference = editor.build_original_difference(original, changed)

    assert same.maximum_displacement_m == pytest.approx(0.0)
    assert same.working_point_count - same.original_point_count == 1
    assert difference.maximum_displacement_m == pytest.approx(0.2009780947)
    assert len(difference.changed_ranges_m) == 1
    assert difference.changed_ranges_m[0] == pytest.approx((1.0198039, 1.0198039))
    assert difference.changed_indices == (1,)


def test_click_reveals_hidden_working_layer_before_editing(tmp_path: Path) -> None:
    source = tmp_path / "line.csv"
    _write_rows(
        source,
        contract.MPC_COLUMNS,
        [_mpc_row(0, 0, 0), _mpc_row(1, 1, 0)],
    )
    instance = _headless_editor(editor.load_trajectory(source))
    instance.show_working = _Value(False)
    instance.selected_index = None
    instance.dirty = False
    instance.focus_set = lambda: None
    instance.redraw = lambda: None
    instance._set_status = lambda _extra="": None
    event = type("Event", (), {"state": 0, "x": 0, "y": 0})()

    instance._on_left_down(event)

    assert instance.show_working.get() is True
    assert instance.selected_index is None
    assert not instance.dirty


def test_cli_topology_flags_are_explicit_and_mutually_exclusive() -> None:
    common = ["--trajectory", "trajectory.csv", "--osm", "map.osm"]

    circular = editor.parse_args([*common, "--circular"])
    open_path = editor.parse_args([*common, "--open"])
    inferred = editor.parse_args(common)

    assert circular.circular is True and circular.circular_explicit
    assert open_path.circular is False and open_path.circular_explicit
    assert inferred.circular is None and not inferred.circular_explicit
    with pytest.raises(SystemExit):
        editor.parse_args([*common, "--circular", "--open"])


def test_builtin_mpc_preset_is_a_local_circular_default() -> None:
    args = editor.parse_args([])
    assert args.circular is True
    assert not args.circular_explicit


def test_save_as_name_is_non_destructive_by_default() -> None:
    assert (
        editor.TrajectoryEditor._edited_filename(Path("traj_mincurv.csv"))
        == "traj_mincurv_edited.csv"
    )
    assert (
        editor.TrajectoryEditor._edited_filename(Path("traj_mincurv_edited.csv"))
        == "traj_mincurv_edited.csv"
    )


@pytest.mark.parametrize(
    ("source", "operation", "expected"),
    [
        ("traj.csv", "normalize_geometry", "traj_normalized.csv"),
        ("traj_edited.csv", "normalize_geometry", "traj_normalized.csv"),
        ("traj_normalized.csv", "recompute_speed_profile", "traj_speed_profiled.csv"),
        ("traj_clearance_adjusted.csv", "adjust_clearance", "traj_clearance_adjusted.csv"),
        ("traj_speed_profiled.csv", "edited", "traj_edited.csv"),
    ],
)
def test_operation_specific_save_as_suffix(
    source: str,
    operation: str,
    expected: str,
) -> None:
    assert editor.TrajectoryEditor._suggested_filename(
        Path(source), operation
    ) == expected


def _candidate_for(
    data: editor.TrajectoryData,
    *,
    source_revision: int,
    operation: str = "recompute_speed_profile",
) -> editor.EditorCandidate:
    candidate_data = copy.deepcopy(data)
    candidate_data.rows[0]["vx_mps"] = "1.5"
    report = editor.validate_trajectory_data(candidate_data, circular=False)
    assert report.is_valid
    return editor.EditorCandidate(
        source_revision=source_revision,
        operation=operation,
        trajectory=candidate_data,
        validation=report,
        transformation={},
        parameters={},
        suggested_suffix="_speed_profiled",
        geometry_dirty=False,
        speed_dirty=False,
    )


def _prepare_headless_candidate(
    instance: editor.TrajectoryEditor,
    *,
    revision: int,
) -> None:
    instance.circular = _Value(False)
    instance.selected_index = 0
    instance.dirty = False
    instance.geometry_dirty = False
    instance.speed_dirty = False
    instance.revision = revision
    instance.undo_stack = []
    instance.validation_report = None
    instance.validation_revision = None
    instance._clear_validation = lambda _reason: None
    instance._show_validation_report = lambda report, *_args, **_kwargs: setattr(
        instance, "validation_report", report
    )
    instance.redraw = lambda: None
    instance._set_status = lambda _extra="": None


def test_apply_candidate_is_one_undoable_revision(tmp_path: Path) -> None:
    source = tmp_path / "source.csv"
    _write_rows(
        source,
        contract.MPC_COLUMNS,
        [_mpc_row(0, 0, 0), _mpc_row(1, 1, 0)],
    )
    instance = _headless_editor(editor.load_trajectory(source))
    _prepare_headless_candidate(instance, revision=8)
    before = copy.deepcopy(instance.trajectory)
    candidate = _candidate_for(instance.trajectory, source_revision=8)
    instance.candidate = candidate

    assert instance._apply_candidate(candidate)
    assert instance.revision == 9
    assert instance.dirty
    assert instance.last_operation == "recompute_speed_profile"
    assert instance.trajectory.rows[0]["vx_mps"] == "1.5"
    assert instance.original_trajectory == before
    assert len(instance.undo_stack) == 1

    instance.undo()
    assert instance.trajectory.rows == before.rows
    assert instance.trajectory.points == before.points
    assert instance.revision == 10
    assert not instance.dirty
    assert instance.last_operation == "edited"


def test_speed_only_candidate_preserves_current_safe_clearance(
    tmp_path: Path,
) -> None:
    source = tmp_path / "source.csv"
    _write_rows(
        source,
        contract.MPC_COLUMNS,
        [_mpc_row(0, 0, 0), _mpc_row(1, 1, 0)],
    )
    instance = _headless_editor(editor.load_trajectory(source))
    _prepare_headless_candidate(instance, revision=8)
    vehicle = editor.VehicleFootprintSpec.from_extents(
        front_m=1.0,
        rear_m=0.5,
        left_m=0.5,
        right_m=0.5,
    )
    safe_report = editor.ClearanceReport(
        map_signature="map-v1",
        source_revision=8,
        vehicle=vehicle,
        minimum_clearance_m=0.4,
        conservative_minimum_clearance_m=0.3,
        measurement_resolution_m=0.1,
        colliding_point_count=0,
        colliding_segment_count=0,
        unknown_contact_count=0,
        outside_map_count=0,
        issues=(),
        is_safe=True,
    )
    instance.clearance_config = object()
    instance.clearance_grid = None
    instance.clearance_report = safe_report
    instance.clearance_revision = 8
    instance.clearance_state = "safe"
    instance.clearance_selected_issue = None
    instance.clearance_report_window = None
    instance.clearance_summary = _Value("")
    candidate = _candidate_for(instance.trajectory, source_revision=8)
    instance.candidate = candidate

    assert instance._apply_candidate(candidate)
    assert instance.clearance_report is safe_report
    assert instance.clearance_revision == 9
    assert instance.clearance_state == "safe"


def test_clearance_pose_adapter_uses_strict_mpc_geometry(tmp_path: Path) -> None:
    source = tmp_path / "source.csv"
    _write_rows(
        source,
        contract.MPC_COLUMNS,
        [_mpc_row(0, 1, 2), _mpc_row(3, 4, 5)],
    )
    data = editor.load_trajectory(source)
    data.rows[0]["psi_rad"] = "0.25"
    data.rows[0]["kappa_radpm"] = "-0.1"

    poses = editor.TrajectoryEditor._trajectory_clearance_poses(data)

    assert poses[0].x_m == pytest.approx(1.0)
    assert poses[0].y_m == pytest.approx(2.0)
    assert poses[0].yaw_rad == pytest.approx(0.25)
    assert poses[0].s_m == pytest.approx(0.0)
    assert poses[0].curvature_radpm == pytest.approx(-0.1)


def test_candidate_safety_guard_rejects_without_mutation(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    source = tmp_path / "source.csv"
    _write_rows(
        source,
        contract.MPC_COLUMNS,
        [_mpc_row(0, 0, 0), _mpc_row(1, 1, 0)],
    )
    instance = _headless_editor(editor.load_trajectory(source))
    _prepare_headless_candidate(instance, revision=8)
    candidate = _candidate_for(instance.trajectory, source_revision=8)
    candidate.apply_guard = lambda _data: (False, "synthetic unsafe", None)
    instance.candidate = candidate
    before = copy.deepcopy(instance.trajectory)
    monkeypatch.setattr(editor.messagebox, "showerror", lambda *_args, **_kwargs: None)

    assert not instance._apply_candidate(candidate)
    assert instance.trajectory == before
    assert instance.revision == 8
    assert not instance.undo_stack


def test_current_unsafe_clearance_report_blocks_save(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    source = tmp_path / "source.csv"
    target = tmp_path / "must_not_exist.csv"
    _write_rows(
        source,
        contract.MPC_COLUMNS,
        [_mpc_row(0, 0, 0), _mpc_row(1, 1, 0)],
    )
    instance = _headless_editor(editor.load_trajectory(source))
    report = editor.validate_trajectory_data(instance.trajectory, circular=False)
    _prepare_headless_save(instance, report)
    instance.clearance_report = type("UnsafeReport", (), {"is_safe": False})()
    instance.clearance_revision = instance.revision
    monkeypatch.setattr(editor.messagebox, "showerror", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(
        editor,
        "save_trajectory",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("serializer ran for unsafe clearance")
        ),
    )

    assert not instance._save_to_path(target)
    assert not target.exists()


def test_failed_clearance_attempt_invalidates_previous_safe_report(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    source = tmp_path / "source.csv"
    _write_rows(
        source,
        contract.MPC_COLUMNS,
        [_mpc_row(0, 0, 0), _mpc_row(1, 1, 0)],
    )
    instance = _headless_editor(editor.load_trajectory(source))
    instance.revision = 3
    instance.clearance_config = object()
    instance.clearance_report = type(
        "SafeReport", (), {"is_safe": True, "map_signature": "old"}
    )()
    instance.clearance_revision = 3
    instance.clearance_state = "safe"
    instance.clearance_summary = _Value("safe")
    instance.clearance_report_window = None
    instance._clearance_precheck = lambda: True
    instance._ensure_clearance_config = lambda: instance.clearance_config
    instance._validation_options = lambda _config: object()
    instance._load_clearance_context = lambda *_args: (_ for _ in ()).throw(
        ValueError("map missing")
    )
    instance._set_status = lambda _extra="": None
    monkeypatch.setattr(editor.messagebox, "showerror", lambda *_args, **_kwargs: None)

    assert instance.validate_clearance_action() is None
    assert instance.clearance_report is None
    assert instance.clearance_state == "failed"
    assert instance.clearance_revision is None


def test_safe_clearance_map_signature_is_rechecked_before_save(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    source = tmp_path / "source.csv"
    target = tmp_path / "must_not_exist.csv"
    _write_rows(
        source,
        contract.MPC_COLUMNS,
        [_mpc_row(0, 0, 0), _mpc_row(1, 1, 0)],
    )
    instance = _headless_editor(editor.load_trajectory(source))
    report = editor.validate_trajectory_data(instance.trajectory, circular=False)
    _prepare_headless_save(instance, report)
    instance.clearance_config = type(
        "Config",
        (),
        {
            "map_yaml_path": tmp_path / "map.yaml",
            "unknown_is_occupied": True,
        },
    )()
    instance.clearance_report = type(
        "SafeReport", (), {"is_safe": True, "map_signature": "old"}
    )()
    instance.clearance_revision = instance.revision
    instance.clearance_state = "safe"
    instance.clearance_summary = _Value("safe")
    instance.clearance_report_window = None
    changed_grid = type(
        "Grid", (), {"spec": type("Spec", (), {"signature": "new"})()}
    )()
    monkeypatch.setattr(editor, "load_occupancy_grid", lambda *_args, **_kwargs: changed_grid)
    monkeypatch.setattr(editor.messagebox, "showerror", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(
        editor,
        "save_trajectory",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("serializer ran after map signature changed")
        ),
    )

    assert not instance._save_to_path(target)
    assert instance.clearance_state == "stale"
    assert not target.exists()


def test_stale_candidate_is_rejected_without_mutation(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    source = tmp_path / "source.csv"
    _write_rows(
        source,
        contract.MPC_COLUMNS,
        [_mpc_row(0, 0, 0), _mpc_row(1, 1, 0)],
    )
    instance = _headless_editor(editor.load_trajectory(source))
    _prepare_headless_candidate(instance, revision=5)
    candidate = _candidate_for(instance.trajectory, source_revision=4)
    instance.candidate = candidate
    before = copy.deepcopy(instance.trajectory)
    monkeypatch.setattr(editor.messagebox, "showerror", lambda *_args, **_kwargs: None)

    assert not instance._apply_candidate(candidate)
    assert instance.trajectory == before
    assert instance.revision == 5
    assert not instance.undo_stack
    assert instance.candidate is None


def test_mutated_candidate_is_rejected_after_cached_validation(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    source = tmp_path / "source.csv"
    _write_rows(
        source,
        contract.MPC_COLUMNS,
        [_mpc_row(0, 0, 0), _mpc_row(1, 1, 0)],
    )
    instance = _headless_editor(editor.load_trajectory(source))
    _prepare_headless_candidate(instance, revision=6)
    candidate = _candidate_for(instance.trajectory, source_revision=6)
    instance.candidate = candidate
    candidate.trajectory.rows[0]["s_m"] = "nan"
    before = copy.deepcopy(instance.trajectory)
    monkeypatch.setattr(editor.messagebox, "showerror", lambda *_args, **_kwargs: None)

    assert not instance._apply_candidate(candidate)
    assert instance.trajectory == before
    assert instance.revision == 6
    assert not instance.undo_stack
    assert instance.candidate is None


def test_discarded_preview_leaves_working_revision_and_undo_unchanged(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    source = tmp_path / "source.csv"
    _write_rows(
        source,
        contract.MPC_COLUMNS,
        [_mpc_row(0, 0, 0), _mpc_row(1, 1, 0)],
    )
    instance = _headless_editor(editor.load_trajectory(source))
    _prepare_headless_candidate(instance, revision=3)
    candidate = _candidate_for(instance.trajectory, source_revision=3)
    before = copy.deepcopy(instance.trajectory)
    monkeypatch.setattr(
        instance,
        "_preview_baseline",
        lambda: (
            copy.deepcopy(instance.trajectory),
            editor.validate_trajectory_data(instance.trajectory, circular=False),
        ),
    )
    monkeypatch.setattr(
        editor,
        "build_comparison_plot",
        lambda *_args, **_kwargs: object(),
    )
    monkeypatch.setattr(editor, "preview_candidate", lambda *_args, **_kwargs: False)

    assert not instance._present_candidate(candidate)
    assert instance.trajectory == before
    assert instance.revision == 3
    assert not instance.undo_stack
    assert instance.candidate is None


def test_invalid_candidate_can_be_previewed_but_not_applied(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    source = tmp_path / "source.csv"
    _write_rows(
        source,
        contract.MPC_COLUMNS,
        [_mpc_row(0, 0, 0), _mpc_row(1, 1, 0)],
    )
    instance = _headless_editor(editor.load_trajectory(source))
    _prepare_headless_candidate(instance, revision=2)
    invalid_data = copy.deepcopy(instance.trajectory)
    invalid_data.points[1] = invalid_data.points[0]
    invalid_data.rows[1]["x_m"] = invalid_data.rows[0]["x_m"]
    invalid_data.rows[1]["y_m"] = invalid_data.rows[0]["y_m"]
    invalid_report = editor.validate_trajectory_data(invalid_data, circular=False)
    assert not invalid_report.is_valid
    candidate = editor.EditorCandidate(
        source_revision=2,
        operation="normalize_geometry",
        trajectory=invalid_data,
        validation=invalid_report,
        transformation={},
        parameters={},
        suggested_suffix="_normalized",
        geometry_dirty=False,
        speed_dirty=False,
    )
    previewed = []
    monkeypatch.setattr(
        editor,
        "preview_candidate",
        lambda *_args, **kwargs: previewed.append(kwargs["validation"]) or False,
    )

    assert not instance._present_candidate(candidate)
    assert previewed == [invalid_report]
    assert instance.revision == 2
    assert not instance.undo_stack
    assert instance.candidate is None


def test_combined_derived_number_and_duplicate_errors_reach_normalize_preview(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    source = tmp_path / "repairable.csv"
    _write_rows(
        source,
        contract.MPC_COLUMNS,
        [
            _mpc_row(0, 0, 0),
            _mpc_row(1, 1, 0),
            _mpc_row(2, 1, 0),
            _mpc_row(3, 2, 0),
        ],
    )
    instance = _headless_editor(editor.load_trajectory(source))
    instance.trajectory.rows[0]["s_m"] = "nan"
    _prepare_headless_candidate(instance, revision=7)
    instance.geometry_dirty = True
    instance.speed_dirty = True
    result = editor.normalize_geometry(
        instance.trajectory,
        NormalizeOptions(
            circular=False,
            metadata_mode="interpolate",
            resample=False,
        ),
        source_revision=7,
    )
    candidate = editor.EditorCandidate(
        source_revision=7,
        operation=result.operation,
        trajectory=result.dataset,
        validation=result.validation,
        transformation=result.transformation,
        parameters=dict(result.parameters),
        suggested_suffix="_normalized",
        geometry_dirty=False,
        speed_dirty=False,
    )
    previewed = []
    monkeypatch.setattr(
        editor,
        "preview_candidate",
        lambda *_args, **kwargs: previewed.append(kwargs["comparison"]) or False,
    )

    assert not instance._present_candidate(candidate)
    assert len(previewed) == 1
    assert previewed[0].summary.before_point_count == 4
    assert previewed[0].summary.candidate_point_count == 3
    assert instance.revision == 7


def test_speed_command_builds_revision_bound_candidate_without_mutating_working(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    source = tmp_path / "source.csv"
    _write_rows(
        source,
        contract.MPC_COLUMNS,
        [
            _mpc_row(0, 0, 0),
            _mpc_row(1, 1, 0),
            _mpc_row(2, 2, 0),
        ],
    )
    instance = _headless_editor(editor.load_trajectory(source))
    _prepare_headless_candidate(instance, revision=9)
    before = copy.deepcopy(instance.trajectory)
    parameters = SpeedProfileParameters(3.0, 1.0, -1.0, 2.0)
    monkeypatch.setattr(
        editor,
        "ask_speed_parameters",
        lambda *_args, **_kwargs: parameters,
    )
    captured: list[editor.EditorCandidate] = []
    instance._present_candidate = lambda candidate: captured.append(candidate) or False

    instance.recompute_speed_candidate()

    assert instance.trajectory == before
    assert len(captured) == 1
    assert captured[0].source_revision == 9
    assert captured[0].operation == "recompute_speed_profile"
    assert captured[0].validation.is_valid
    assert not captured[0].speed_dirty


def test_speed_command_contains_unexpected_core_failure(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    source = tmp_path / "source.csv"
    _write_rows(
        source,
        contract.MPC_COLUMNS,
        [
            _mpc_row(0, 0, 0),
            _mpc_row(1, 1, 0),
            _mpc_row(2, 2, 0),
        ],
    )
    instance = _headless_editor(editor.load_trajectory(source))
    _prepare_headless_candidate(instance, revision=4)
    before = copy.deepcopy(instance.trajectory)
    monkeypatch.setattr(
        editor,
        "ask_speed_parameters",
        lambda *_args, **_kwargs: SpeedProfileParameters(3.0, 1.0, -1.0, 2.0),
    )
    monkeypatch.setattr(
        editor,
        "recompute_speed_profile",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(RuntimeError("boom")),
    )
    monkeypatch.setattr(editor.messagebox, "showerror", lambda *_args, **_kwargs: None)

    instance.recompute_speed_candidate()

    assert instance.trajectory == before
    assert instance.revision == 4
    assert not instance.undo_stack
    assert instance.candidate is None


def test_v2x_editor_compatibility_exports_remain_available() -> None:
    assert editor.Point is not None
    assert callable(editor._default_paths)
    assert callable(editor.load_osm_rails)
    assert callable(editor.load_trajectory)
