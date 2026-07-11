#!/usr/bin/env python3
"""Exercise Python Editor output against the built C++ runtime validator."""

import argparse
import csv
from pathlib import Path
import subprocess
import tempfile

from multi_purpose_mpc_ros.tools import trajectory_contract as contract
from multi_purpose_mpc_ros.tools import trajectory_editor as editor


def _row(
    s: object,
    x: object,
    y: object,
    *,
    psi: object = 0,
    kappa: object = 0,
    velocity: object = 1,
) -> dict[str, str]:
    return {
        "s_m": str(s),
        "x_m": str(x),
        "y_m": str(y),
        "psi_rad": str(psi),
        "kappa_radpm": str(kappa),
        "vx_mps": str(velocity),
        "ax_mps2": "0",
    }


def _write(path: Path, rows: list[dict[str, str]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=contract.MPC_COLUMNS,
            lineterminator="\n",
        )
        writer.writeheader()
        writer.writerows(rows)


def _run_validator(validator: Path, path: Path, *options: str) -> subprocess.CompletedProcess:
    return subprocess.run(
        [str(validator), str(path), *options],
        check=False,
        capture_output=True,
        text=True,
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--validator", required=True, type=Path)
    args = parser.parse_args()

    with tempfile.TemporaryDirectory(prefix="trajectory_cpp_contract_") as directory:
        root = Path(directory)

        source = root / "source.csv"
        output = root / "editor_output.csv"
        _write(source, [_row(0, 0, 0), _row(1, 1, 0)])
        data = editor.load_trajectory(source)
        editor.save_trajectory(data, output, circular=False)
        accepted = _run_validator(args.validator, output)
        if accepted.returncode != 0:
            raise RuntimeError(
                "C++ validator rejected Editor output:\n"
                + accepted.stdout
                + accepted.stderr
            )

        reject_cases = {
            "raw_last_degenerate.csv": (
                [
                    _row(0, 0, 0),
                    _row(1, 1, 0),
                    _row(2, 0.0005, 0),
                    _row(3, 0.0005005, 0),
                ],
                ("--circular",),
            ),
            "subnormal.csv": (
                [_row(0, 0, 0, velocity="1e-323"), _row(1, 1, 0)],
                (),
            ),
            "closure_overflow.csv": (
                [
                    _row(0, 0, 0, psi="-1e308"),
                    _row(1, 1, 0),
                    _row(2, 2, 0, psi="1e308"),
                ],
                (),
            ),
        }
        for filename, (rows, options) in reject_cases.items():
            path = root / filename
            _write(path, rows)
            python_report = contract.validate_csv_file(
                path,
                circular="--circular" in options,
            )
            cpp_result = _run_validator(args.validator, path, *options)
            if python_report.is_valid or cpp_result.returncode == 0:
                raise RuntimeError(
                    f"Python/C++ reject parity failed for {filename}: "
                    f"python_valid={python_report.is_valid}, "
                    f"cpp_exit={cpp_result.returncode}"
                )


if __name__ == "__main__":
    main()
