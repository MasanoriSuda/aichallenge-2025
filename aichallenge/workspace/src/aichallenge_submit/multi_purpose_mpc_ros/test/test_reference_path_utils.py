from pathlib import Path

import pytest

from multi_purpose_mpc_ros.core.utils import load_ref_path


HEADER = "s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2\n"


def write_csv(tmp_path: Path, rows: str) -> Path:
    csv_path = tmp_path / "trajectory.csv"
    csv_path.write_text(HEADER + rows, encoding="utf-8")
    return csv_path


def test_circular_loader_removes_closure_record_as_all_columns(tmp_path: Path) -> None:
    csv_path = write_csv(
        tmp_path,
        "0,0,0,0.1,0.2,1,2\n"
        "1,1,0,1.1,1.2,3,4\n"
        "2,0,1,2.1,2.2,5,6\n"
        "3,0.0005,0,9.1,9.2,7,8\n",
    )

    x, y, psi, kappa = load_ref_path(str(csv_path), circular=True)

    assert x == [0.0, 1.0, 0.0]
    assert y == [0.0, 0.0, 1.0]
    assert psi == [0.1, 1.1, 2.1]
    assert kappa == [0.2, 1.2, 2.2]


def test_non_circular_loader_keeps_matching_endpoint(tmp_path: Path) -> None:
    csv_path = write_csv(
        tmp_path,
        "0,0,0,0,0,1,0\n"
        "1,1,0,0,0,1,0\n"
        "2,0,1,0,0,1,0\n"
        "3,0,0,0,0,1,0\n",
    )

    x, _, _, _ = load_ref_path(str(csv_path), circular=False)

    assert x == [0.0, 1.0, 0.0, 0.0]


def test_circular_loader_rejects_too_few_unique_points(tmp_path: Path) -> None:
    csv_path = write_csv(
        tmp_path,
        "0,0,0,0,0,1,0\n"
        "1,1,0,0,0,1,0\n"
        "2,0,0,0,0,1,0\n",
    )

    with pytest.raises(ValueError, match="at least 3 unique points"):
        load_ref_path(str(csv_path), circular=True)
