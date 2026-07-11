import copy
import math
from pathlib import Path
import stat
import sys

import pytest

from multi_purpose_mpc_ros.tools import trajectory_contract as contract


def mpc_row(
    s: object,
    x: object,
    y: object,
    psi: object = 0.0,
    kappa: object = 0.0,
    velocity: object = 1.0,
    acceleration: object = 0.0,
) -> dict:
    return {
        "s_m": str(s),
        "x_m": str(x),
        "y_m": str(y),
        "psi_rad": str(psi),
        "kappa_radpm": str(kappa),
        "vx_mps": str(velocity),
        "ax_mps2": str(acceleration),
    }


def pure_pursuit_row(x: object, y: object, speed: object = 1.0) -> dict:
    return {
        "x": str(x),
        "y": str(y),
        "z": "0",
        "x_quat": "0",
        "y_quat": "0",
        "z_quat": "0",
        "w_quat": "1",
        "speed": str(speed),
    }


def issue_codes(report: contract.ValidationReport) -> set[str]:
    return {issue.code for issue in report.issues}


def test_mpc_accepts_arbitrary_input_order_and_reports_metrics() -> None:
    fieldnames = list(reversed(contract.MPC_COLUMNS))
    rows = [
        mpc_row(0, 0, 0, psi=math.radians(179), velocity=2),
        mpc_row(1, 1, 0, psi=math.radians(-179), velocity=3),
    ]

    report = contract.validate_trajectory(fieldnames, rows, "mpc", circular=False)

    assert report.is_valid
    assert report.metrics.point_count == 2
    assert report.metrics.total_distance_m == pytest.approx(1.0)
    assert report.metrics.max_abs_psi_difference_rad == pytest.approx(
        math.radians(2), abs=1e-12
    )


@pytest.mark.parametrize(
    ("fieldnames", "expected_code"),
    [
        (contract.MPC_COLUMNS[:-1], "MISSING_HEADER"),
        ((*contract.MPC_COLUMNS, "extra"), "EXTRA_HEADER"),
        ((*contract.MPC_COLUMNS[:-1], "vx_mps"), "DUPLICATE_HEADER"),
        (("", *contract.MPC_COLUMNS[1:]), "EMPTY_HEADER"),
    ],
)
def test_mpc_rejects_invalid_headers(fieldnames: tuple, expected_code: str) -> None:
    report = contract.validate_trajectory(fieldnames, [], "mpc", circular=False)
    assert not report.is_valid
    assert expected_code in issue_codes(report)


@pytest.mark.parametrize(
    ("value", "message_fragment"),
    [
        ("", "empty"),
        ("1suffix", "partially"),
        ("1_0", "partially"),
        ("\u00a01", "partially"),
        ("nan", "finite"),
        ("+inf", "finite"),
        ("1e309", "finite"),
        ("1e-9999", "outside double range"),
    ],
)
def test_mpc_rejects_invalid_numbers(value: str, message_fragment: str) -> None:
    rows = [mpc_row(0, value, 0), mpc_row(1, 1, 0)]
    report = contract.validate_trajectory(
        contract.MPC_COLUMNS, rows, "mpc", circular=False
    )
    issue = next(issue for issue in report.issues if issue.code == "INVALID_NUMBER")
    assert message_fragment in issue.message
    assert issue.line_number == 2
    assert issue.column == "x_m"


def test_subnormal_double_is_rejected_like_cpp_stod() -> None:
    subnormal = math.nextafter(sys.float_info.min, 0.0)
    rejected = contract.validate_trajectory(
        contract.MPC_COLUMNS,
        [mpc_row(0, 0, 0, velocity=subnormal), mpc_row(1, 1, 0)],
        "mpc",
        circular=False,
    )
    accepted = contract.validate_trajectory(
        contract.MPC_COLUMNS,
        [mpc_row(0, 0, 0, velocity=sys.float_info.min), mpc_row(1, 1, 0)],
        "mpc",
        circular=False,
    )
    assert "INVALID_NUMBER" in issue_codes(rejected)
    assert accepted.is_valid


def test_file_parser_retains_bom_whitespace_crlf_and_physical_line_numbers(
    tmp_path: Path,
) -> None:
    csv_path = tmp_path / "trajectory.csv"
    csv_path.write_bytes(
        (
            "  \ufeff  s_m , x_m , y_m , psi_rad , kappa_radpm , vx_mps , ax_mps2 \r\n"
            "0, 0, 0, 0, 0, 1, 0\r\n"
            "\r\n"
            "1, 1, 0, 0, 0, 1, 0\r\n"
        ).encode("utf-8")
    )

    report = contract.validate_csv_file(csv_path, circular=False)

    assert "BLANK_DATA_ROW" in issue_codes(report)
    blank_issue = next(issue for issue in report.issues if issue.code == "BLANK_DATA_ROW")
    assert blank_issue.line_number == 3
    assert report.metrics.point_count == 2


@pytest.mark.parametrize(
    ("data_line", "expected_code"),
    [
        ("0,0,0,0,0,1", "INVALID_NUMBER"),
        ("0,0,0,0,0,1,0,extra", "EXTRA_ROW_FIELDS"),
        ('0,"0",0,0,0,1,0', "QUOTED_CSV_UNSUPPORTED"),
    ],
)
def test_file_parser_rejects_short_long_and_quoted_rows(
    tmp_path: Path, data_line: str, expected_code: str
) -> None:
    csv_path = tmp_path / "trajectory.csv"
    csv_path.write_text(
        ",".join(contract.MPC_COLUMNS) + "\n" + data_line + "\n",
        encoding="utf-8",
    )
    report = contract.validate_csv_file(csv_path, circular=False)
    assert expected_code in issue_codes(report)


def test_non_increasing_s_and_degenerate_segments_are_distinct_errors() -> None:
    rows = [
        mpc_row(0, 0, 0),
        mpc_row(0, 1, 0),
        mpc_row(2, 1 + contract.MIN_SEGMENT_LENGTH_M, 0),
    ]
    report = contract.validate_trajectory(
        contract.MPC_COLUMNS, rows, "mpc", circular=False
    )
    assert "NON_INCREASING_S" in issue_codes(report)
    assert "DEGENERATE_SEGMENT" in issue_codes(report)


def test_exact_internal_duplicate_has_distinct_issue_code() -> None:
    rows = [mpc_row(0, 0, 0), mpc_row(1, 1, 0), mpc_row(2, 1, 0)]
    report = contract.validate_trajectory(
        contract.MPC_COLUMNS, rows, "mpc", circular=False
    )
    assert "DUPLICATE_POINT" in issue_codes(report)


@pytest.mark.parametrize(
    ("spacing", "valid"),
    [
        (math.nextafter(contract.MIN_SEGMENT_LENGTH_M, 0.0), False),
        (contract.MIN_SEGMENT_LENGTH_M, False),
        (math.nextafter(contract.MIN_SEGMENT_LENGTH_M, math.inf), True),
    ],
)
def test_minimum_segment_length_boundary(spacing: float, valid: bool) -> None:
    rows = [mpc_row(0, 0, 0), mpc_row(1, spacing, 0)]
    report = contract.validate_trajectory(
        contract.MPC_COLUMNS, rows, "mpc", circular=False
    )
    assert ("DEGENERATE_SEGMENT" not in issue_codes(report)) is valid


@pytest.mark.parametrize(
    ("endpoint_distance", "duplicate"),
    [
        (math.nextafter(contract.CLOSURE_TOLERANCE_M, 0.0), True),
        (contract.CLOSURE_TOLERANCE_M, True),
        (math.nextafter(contract.CLOSURE_TOLERANCE_M, math.inf), False),
    ],
)
def test_closure_duplicate_tolerance_boundary(
    endpoint_distance: float, duplicate: bool
) -> None:
    rows = [
        mpc_row(0, 0, 0),
        mpc_row(1, 1, 0),
        mpc_row(2, 0, 1),
        mpc_row(3, endpoint_distance, 0),
    ]
    report = contract.validate_trajectory(
        contract.MPC_COLUMNS, rows, "mpc", circular=True
    )
    assert report.metrics.duplicate_endpoint is duplicate


def test_circular_duplicate_endpoint_uses_runtime_tolerance_and_excludes_zero_seam() -> None:
    rows = [
        mpc_row(0, 0, 0),
        mpc_row(1, 1, 0),
        mpc_row(2, 0, 1),
        mpc_row(3, contract.CLOSURE_TOLERANCE_M, 0),
    ]

    report = contract.validate_trajectory(
        contract.MPC_COLUMNS, rows, "mpc", circular=True
    )

    assert report.is_valid
    assert report.warning_count == 1
    assert report.metrics.duplicate_endpoint
    assert report.metrics.normalized_point_count == 3
    assert report.metrics.closure_distance_m == pytest.approx(
        contract.CLOSURE_TOLERANCE_M
    )
    assert report.metrics.closing_edge_spacing_m == pytest.approx(1.0)
    assert report.metrics.min_spacing_m > contract.MIN_SEGMENT_LENGTH_M


def test_legacy_duplicate_endpoint_still_validates_raw_last_segment_like_cpp() -> None:
    rows = [
        mpc_row(0, 0, 0),
        mpc_row(1, 1, 0),
        mpc_row(2, 0.0005, 0),
        mpc_row(3, 0.0005005, 0),
    ]
    report = contract.validate_trajectory(
        contract.MPC_COLUMNS, rows, "mpc", circular=True
    )
    assert report.metrics.duplicate_endpoint
    assert "DEGENERATE_SEGMENT" in issue_codes(report)
    assert not report.is_valid


def test_circular_unique_closing_edge_is_valid_and_counted_once() -> None:
    rows = [mpc_row(0, 0, 0), mpc_row(1, 1, 0), mpc_row(2, 0, 1)]
    report = contract.validate_trajectory(
        contract.MPC_COLUMNS, rows, "mpc", circular=True
    )
    assert report.is_valid
    assert not report.metrics.duplicate_endpoint
    assert report.metrics.total_distance_m == pytest.approx(2 + math.sqrt(2))
    assert report.metrics.closing_edge_spacing_m == pytest.approx(1.0)


def test_derived_lateral_acceleration_overflow_is_structured_error() -> None:
    rows = [
        mpc_row(0, 0, 0, kappa=1e308, velocity=1e308),
        mpc_row(1, 1, 0, kappa=0, velocity=1),
    ]
    report = contract.validate_trajectory(
        contract.MPC_COLUMNS, rows, "mpc", circular=False
    )
    assert "NONFINITE_LATERAL_ACCELERATION" in issue_codes(report)
    assert report.metrics.max_lateral_acceleration_mps2 == 0.0


def test_lateral_acceleration_avoids_false_double_intermediate_overflow() -> None:
    zero_curvature = contract.validate_trajectory(
        contract.MPC_COLUMNS,
        [mpc_row(0, 0, 0, kappa=0, velocity=1e308), mpc_row(1, 1, 0)],
        "mpc",
        circular=False,
    )
    scaled_curvature = contract.validate_trajectory(
        contract.MPC_COLUMNS,
        [mpc_row(0, 0, 0, kappa=1e-100, velocity=1e200), mpc_row(1, 1, 0)],
        "mpc",
        circular=False,
    )
    assert zero_curvature.is_valid
    assert zero_curvature.metrics.max_lateral_acceleration_mps2 == 0.0
    assert scaled_curvature.is_valid
    assert scaled_curvature.metrics.max_lateral_acceleration_mps2 == pytest.approx(1e300)


@pytest.mark.parametrize("column", ["psi_rad", "kappa_radpm"])
def test_open_path_rejects_nonfinite_cpp_closure_difference(column: str) -> None:
    rows = [mpc_row(0, 0, 0), mpc_row(1, 1, 0), mpc_row(2, 2, 0)]
    rows[0][column] = "-1e308"
    rows[-1][column] = "1e308"
    report = contract.validate_trajectory(
        contract.MPC_COLUMNS, rows, "mpc", circular=False
    )
    expected = (
        "NONFINITE_CLOSURE_PSI_DIFFERENCE"
        if column == "psi_rad"
        else "NONFINITE_CLOSURE_KAPPA_DIFFERENCE"
    )
    assert expected in issue_codes(report)


def test_optional_limits_are_not_invented_and_are_enforced_when_present() -> None:
    rows = [mpc_row(0, 0, 0, kappa=0.5, velocity=3, acceleration=2), mpc_row(1, 1, 0)]
    unconstrained = contract.validate_trajectory(
        contract.MPC_COLUMNS, rows, "mpc", circular=False
    )
    assert unconstrained.is_valid

    constrained = contract.validate_trajectory(
        contract.MPC_COLUMNS,
        rows,
        "mpc",
        circular=False,
        limits=contract.ValidationLimits(
            v_max_mps=2.0,
            a_max_mps2=1.0,
            a_min_mps2=-1.0,
            ay_max_mps2=4.0,
        ),
    )
    assert {"V_MAX_EXCEEDED", "A_MAX_EXCEEDED", "AY_MAX_EXCEEDED"}.issubset(
        issue_codes(constrained)
    )


def test_pure_pursuit_uses_its_own_eight_column_contract() -> None:
    rows = [pure_pursuit_row(0, 0), pure_pursuit_row(1, 0)]
    report = contract.validate_trajectory(
        contract.PURE_PURSUIT_COLUMNS,
        rows,
        "pure_pursuit",
        circular=False,
    )
    assert report.is_valid
    assert report.metrics.min_velocity_mps == 1.0
    assert report.metrics.max_abs_kappa_radpm is None


def test_validation_is_non_mutating_and_deterministic() -> None:
    fieldnames = list(contract.MPC_COLUMNS)
    rows = [mpc_row(0, 0, 0), mpc_row(1, 1, 0)]
    original_fieldnames = copy.deepcopy(fieldnames)
    original_rows = copy.deepcopy(rows)

    first = contract.validate_trajectory(fieldnames, rows, "mpc", circular=False)
    second = contract.validate_trajectory(fieldnames, rows, "mpc", circular=False)

    assert fieldnames == original_fieldnames
    assert rows == original_rows
    assert first == second


def test_atomic_write_preserves_existing_mode_and_canonical_content(tmp_path: Path) -> None:
    target = tmp_path / "trajectory.csv"
    target.write_text("old\n", encoding="utf-8")
    target.chmod(0o664)
    rows = [mpc_row(0, 0, 0), mpc_row(1, 1, 0)]

    contract.atomic_write_csv(target, contract.MPC_COLUMNS, rows)

    assert target.read_text(encoding="utf-8").splitlines()[0] == ",".join(
        contract.MPC_COLUMNS
    )
    assert stat.S_IMODE(target.stat().st_mode) == 0o664
    assert not list(tmp_path.glob(".trajectory.csv.*.tmp"))


def test_atomic_write_validation_failure_leaves_target_and_cleans_temp(
    tmp_path: Path,
) -> None:
    target = tmp_path / "trajectory.csv"
    target.write_text("original\n", encoding="utf-8")
    target.chmod(0o664)
    before = (target.read_bytes(), stat.S_IMODE(target.stat().st_mode))

    def fail_validation(_path: Path) -> None:
        raise ValueError("candidate rejected")

    with pytest.raises(ValueError, match="candidate rejected"):
        contract.atomic_write_csv(
            target,
            contract.MPC_COLUMNS,
            [mpc_row(0, 0, 0), mpc_row(1, 1, 0)],
            validate_path=fail_validation,
        )

    assert (target.read_bytes(), stat.S_IMODE(target.stat().st_mode)) == before
    assert not list(tmp_path.glob(".trajectory.csv.*.tmp"))


def test_atomic_write_replace_failure_leaves_target_and_cleans_temp(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    target = tmp_path / "trajectory.csv"
    target.write_text("original\n", encoding="utf-8")

    def fail_replace(_source: Path, _target: Path) -> None:
        raise OSError("replace failed")

    monkeypatch.setattr(contract.os, "replace", fail_replace)
    with pytest.raises(OSError, match="replace failed"):
        contract.atomic_write_csv(
            target,
            contract.MPC_COLUMNS,
            [mpc_row(0, 0, 0), mpc_row(1, 1, 0)],
        )

    assert target.read_text(encoding="utf-8") == "original\n"
    assert not list(tmp_path.glob(".trajectory.csv.*.tmp"))


def test_atomic_write_refuses_regular_and_dangling_symlinks(tmp_path: Path) -> None:
    referent = tmp_path / "referent.csv"
    referent.write_text("original\n", encoding="utf-8")
    link = tmp_path / "trajectory.csv"
    link.symlink_to(referent.name)

    with pytest.raises(ValueError, match="symlink"):
        contract.atomic_write_csv(
            link,
            contract.MPC_COLUMNS,
            [mpc_row(0, 0, 0), mpc_row(1, 1, 0)],
        )
    assert link.is_symlink()
    assert referent.read_text(encoding="utf-8") == "original\n"

    dangling = tmp_path / "dangling.csv"
    dangling.symlink_to("missing.csv")
    with pytest.raises(ValueError, match="symlink"):
        contract.atomic_write_csv(
            dangling,
            contract.MPC_COLUMNS,
            [mpc_row(0, 0, 0), mpc_row(1, 1, 0)],
        )
    assert dangling.is_symlink()
