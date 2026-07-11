import copy
import math
from pathlib import Path

import pytest

from multi_purpose_mpc_ros.tools import trajectory_contract as contract
from multi_purpose_mpc_ros.tools import trajectory_speed as speed


def _row(
    index: int,
    x: float,
    y: float,
    *,
    kappa: float = 0.0,
    velocity: float = 99.0,
) -> dict[str, str]:
    return {
        "s_m": str(index),
        "x_m": str(x),
        "y_m": str(y),
        "psi_rad": "0",
        "kappa_radpm": str(kappa),
        "vx_mps": str(velocity),
        "ax_mps2": "99",
    }


def _data(rows: list[dict[str, str]]) -> contract.TrajectoryData:
    return contract.TrajectoryData(
        path=Path("source.csv"),
        fieldnames=list(contract.MPC_COLUMNS),
        rows=rows,
        points=[(float(row["x_m"]), float(row["y_m"])) for row in rows],
        x_column="x_m",
        y_column="y_m",
        format_name="mpc",
    )


def _parameters(**changes: object) -> speed.SpeedProfileParameters:
    values = {
        "v_max_mps": 8.0,
        "a_max_mps2": 1.0,
        "a_min_mps2": -2.0,
        "ay_max_mps2": 2.0,
        "minimum_speed_mps": 0.0,
        "epsilon": 1e-9,
        "tolerance": 1e-10,
        "max_iterations": 100,
    }
    values.update(changes)
    return speed.SpeedProfileParameters(**values)  # type: ignore[arg-type]


def _codes(result: speed.SpeedProfileResult) -> set[str]:
    return {issue.code for issue in result.report.issues}


@pytest.mark.parametrize(
    ("changes", "column"),
    [
        ({"v_max_mps": 0.0}, "v_max_mps"),
        ({"v_max_mps": math.nan}, "v_max_mps"),
        ({"a_max_mps2": 0.0}, "a_max_mps2"),
        ({"a_min_mps2": 0.0}, "a_min_mps2"),
        ({"ay_max_mps2": math.inf}, "ay_max_mps2"),
        ({"minimum_speed_mps": -0.1}, "minimum_speed_mps"),
        ({"epsilon": 0.0}, "epsilon"),
        ({"tolerance": 0.0}, "tolerance"),
        ({"max_iterations": 0}, "max_iterations"),
        ({"max_iterations": 1.5}, "max_iterations"),
    ],
)
def test_invalid_parameter_contract_is_reported(
    changes: dict[str, object], column: str
) -> None:
    data = _data([_row(0, 0, 0), _row(1, 1, 0)])
    result = speed.recompute_speed_profile(
        data,
        circular=False,
        parameters=_parameters(**changes),
        source_revision=4,
    )

    assert result.candidate is None
    assert result.iterations == 0
    assert any(
        issue.code == "INVALID_SPEED_PARAMETER" and issue.column == column
        for issue in result.report.issues
    )


def test_circular_and_revision_are_strictly_validated() -> None:
    data = _data([_row(0, 0, 0), _row(1, 1, 0)])
    invalid_topology = speed.recompute_speed_profile(
        data,
        circular=1,  # type: ignore[arg-type]
        parameters=_parameters(),
        source_revision=0,
    )
    invalid_revision = speed.recompute_speed_profile(
        data,
        circular=False,
        parameters=_parameters(),
        source_revision=-1,
    )
    assert invalid_topology.candidate is None
    assert invalid_revision.candidate is None
    assert any(issue.column == "circular" for issue in invalid_topology.report.issues)
    assert any(
        issue.column == "source_revision" for issue in invalid_revision.report.issues
    )


def test_mpc_only_and_invalid_source_are_rejected() -> None:
    pure = contract.TrajectoryData(
        path=Path("pure.csv"),
        fieldnames=list(contract.PURE_PURSUIT_COLUMNS),
        rows=[
            {
                "x": "0",
                "y": "0",
                "z": "0",
                "x_quat": "0",
                "y_quat": "0",
                "z_quat": "0",
                "w_quat": "1",
                "speed": "1",
            },
            {
                "x": "1",
                "y": "0",
                "z": "0",
                "x_quat": "0",
                "y_quat": "0",
                "z_quat": "0",
                "w_quat": "1",
                "speed": "1",
            },
        ],
        points=[(0.0, 0.0), (1.0, 0.0)],
        x_column="x",
        y_column="y",
        format_name="pure_pursuit",
    )
    invalid = _data([_row(0, 0, 0), _row(1, 1, 0)])
    invalid.rows[0]["kappa_radpm"] = "nan"

    pure_result = speed.recompute_speed_profile(
        pure, circular=False, parameters=_parameters(), source_revision=0
    )
    invalid_result = speed.recompute_speed_profile(
        invalid, circular=False, parameters=_parameters(), source_revision=0
    )

    assert pure_result.candidate is None
    assert "UNSUPPORTED_SPEED_PROFILE_FORMAT" in _codes(pure_result)
    assert invalid_result.candidate is None
    assert "INVALID_NUMBER" in _codes(invalid_result)


def test_straight_open_profile_is_non_mutating_and_revision_bound() -> None:
    data = _data([_row(0, 0, 0), _row(1, 1, 0), _row(2, 2, 0)])
    before = copy.deepcopy(data)

    result = speed.recompute_speed_profile(
        data, circular=False, parameters=_parameters(v_max_mps=5.0), source_revision=12
    )

    assert data == before
    assert result.source_revision == 12
    assert result.candidate is not None
    assert result.candidate is not data
    assert result.report.is_valid
    assert [float(row["vx_mps"]) for row in result.candidate.rows] == [5.0] * 3
    assert [float(row["ax_mps2"]) for row in result.candidate.rows] == [0.0] * 3
    assert result.candidate.rows[-1]["ax_mps2"] == "0.0"


def test_constant_curvature_applies_lateral_acceleration_cap() -> None:
    data = _data(
        [_row(index, float(index), 0.0, kappa=0.5) for index in range(4)]
    )
    result = speed.recompute_speed_profile(
        data,
        circular=False,
        parameters=_parameters(v_max_mps=9.0, ay_max_mps2=2.0),
        source_revision=0,
    )

    assert result.candidate is not None
    velocities = [float(row["vx_mps"]) for row in result.candidate.rows]
    assert velocities == pytest.approx([2.0] * 4)
    assert result.report.metrics.max_lateral_acceleration_mps2 == pytest.approx(2.0)


def test_compound_curvature_satisfies_all_open_edge_constraints() -> None:
    data = _data(
        [
            _row(0, 0, 0, kappa=0.0),
            _row(1, 1, 0, kappa=1.0),
            _row(2, 2, 0, kappa=0.0),
            _row(3, 3, 0, kappa=0.0),
        ]
    )
    parameters = _parameters(
        v_max_mps=10.0,
        a_max_mps2=1.0,
        a_min_mps2=-2.0,
        ay_max_mps2=1.0,
    )
    result = speed.recompute_speed_profile(
        data, circular=False, parameters=parameters, source_revision=1
    )

    assert result.candidate is not None
    velocities = [float(row["vx_mps"]) for row in result.candidate.rows]
    assert velocities == pytest.approx(
        [math.sqrt(5.0), 1.0, math.sqrt(3.0), math.sqrt(5.0)]
    )
    accelerations = [float(row["ax_mps2"]) for row in result.candidate.rows]
    assert accelerations == pytest.approx([-2.0, 1.0, 1.0, 0.0])
    assert result.report.is_valid


def test_circular_relaxation_includes_seam_and_converges() -> None:
    data = _data(
        [
            _row(0, 0, 0, kappa=1.0),
            _row(1, 1, 0),
            _row(2, 1, 1),
            _row(3, 0, 1),
        ]
    )
    parameters = _parameters(
        v_max_mps=10.0,
        a_max_mps2=1.0,
        a_min_mps2=-1.0,
        ay_max_mps2=1.0,
    )
    result = speed.recompute_speed_profile(
        data, circular=True, parameters=parameters, source_revision=2
    )

    assert result.candidate is not None
    assert result.iterations >= 1
    velocities = [float(row["vx_mps"]) for row in result.candidate.rows]
    assert velocities[0] == pytest.approx(1.0)
    assert velocities[1] <= math.sqrt(3.0) + parameters.tolerance
    assert velocities[-1] <= math.sqrt(3.0) + parameters.tolerance
    seam_ax = float(result.candidate.rows[-1]["ax_mps2"])
    assert parameters.a_min_mps2 - parameters.tolerance <= seam_ax
    assert seam_ax <= parameters.a_max_mps2 + parameters.tolerance
    assert result.report.is_valid


def test_duplicate_circular_endpoint_mirrors_first_profile_values() -> None:
    rows = [
        _row(0, 0, 0, kappa=1.0),
        _row(1, 1, 0),
        _row(2, 1, 1),
        _row(3, 0, 1),
        _row(4, 0, 0, kappa=99.0),
    ]
    data = _data(rows)
    before = copy.deepcopy(data)
    result = speed.recompute_speed_profile(
        data, circular=True, parameters=_parameters(), source_revision=3
    )

    assert data == before
    assert result.candidate is not None
    assert result.report.metrics.duplicate_endpoint
    assert result.candidate.rows[-1]["vx_mps"] == result.candidate.rows[0]["vx_mps"]
    assert result.candidate.rows[-1]["ax_mps2"] == result.candidate.rows[0]["ax_mps2"]


def test_minimum_speed_conflict_is_infeasible_without_candidate() -> None:
    data = _data([_row(0, 0, 0, kappa=1.0), _row(1, 1, 0, kappa=1.0)])
    result = speed.recompute_speed_profile(
        data,
        circular=False,
        parameters=_parameters(ay_max_mps2=1.0, minimum_speed_mps=1.01),
        source_revision=5,
    )

    assert result.candidate is None
    assert result.iterations == 0
    assert "MINIMUM_SPEED_INFEASIBLE" in _codes(result)


def test_iteration_limit_reports_nonconvergence() -> None:
    data = _data(
        [
            _row(0, 0, 0),
            _row(1, 1, 0),
            _row(2, 2, 0),
            _row(3, 3, 0),
            _row(4, 4, 0, kappa=1.0),
            _row(5, 5, 0),
        ]
    )
    result = speed.recompute_speed_profile(
        data,
        circular=False,
        parameters=_parameters(
            v_max_mps=20.0,
            ay_max_mps2=1.0,
            a_max_mps2=0.1,
            a_min_mps2=-0.1,
            max_iterations=1,
        ),
        source_revision=6,
    )

    assert result.candidate is None
    assert result.iterations == 1
    assert "SPEED_PROFILE_NONCONVERGENCE" in _codes(result)


def test_same_input_produces_deterministic_candidate_and_report() -> None:
    data = _data(
        [_row(index, float(index), 0.0, kappa=index * 0.05) for index in range(6)]
    )
    parameters = _parameters()
    first = speed.recompute_speed_profile(
        data, circular=False, parameters=parameters, source_revision=9
    )
    second = speed.recompute_speed_profile(
        data, circular=False, parameters=parameters, source_revision=9
    )

    assert first == second
    assert first.candidate is not None
