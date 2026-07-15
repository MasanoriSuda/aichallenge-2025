#!/usr/bin/env python3
"""Focused negative tests for Phase 0 result-contract validation."""

from __future__ import annotations

import copy
import importlib.util
import json
import sys
from pathlib import Path
from typing import Callable


REPO_ROOT = Path(__file__).resolve().parents[3]
ORACLE_PATH = Path(__file__).with_name("phase0_contract_oracle.py")
SUMMARY_PATH = "aichallenge/result-summary.json"
DETAILS_PATH = ".steering/20260714-mpc-refactor/fixtures/d1-result-details-v3.json"

spec = importlib.util.spec_from_file_location("phase0_contract_oracle", ORACLE_PATH)
if spec is None or spec.loader is None:
    raise RuntimeError(f"cannot load {ORACLE_PATH}")
oracle_module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = oracle_module
spec.loader.exec_module(oracle_module)

BASE_SUMMARY = json.loads((REPO_ROOT / SUMMARY_PATH).read_text(encoding="utf-8"))
BASE_DETAILS = json.loads((REPO_ROOT / DETAILS_PATH).read_text(encoding="utf-8"))


class FixtureOracle(oracle_module.Oracle):
    def __init__(self, summary: dict[str, object], details: dict[str, object]) -> None:
        super().__init__(REPO_ROOT)
        self._summary = summary
        self._details = details

    def read(self, relative: str) -> str:
        if relative == SUMMARY_PATH:
            return json.dumps(self._summary)
        if relative == DETAILS_PATH:
            return json.dumps(self._details)
        return super().read(relative)


def result_status(
    mutate_summary: Callable[[dict[str, object]], None] | None = None,
    mutate_details: Callable[[dict[str, object]], None] | None = None,
) -> str:
    summary = copy.deepcopy(BASE_SUMMARY)
    details = copy.deepcopy(BASE_DETAILS)
    if mutate_summary is not None:
        mutate_summary(summary)
    if mutate_details is not None:
        mutate_details(details)
    oracle = FixtureOracle(summary, details)
    oracle.check_result_schema()
    return oracle.checks[-1].status


def main() -> int:
    cases: list[tuple[str, str]] = []
    cases.append(("baseline", result_status()))
    cases.append(
        (
            "empty_vehicles",
            result_status(lambda value: value.__setitem__("vehicles", [])),
        )
    )

    def duplicate_vehicle_number(value: dict[str, object]) -> None:
        vehicles = value["vehicles"]
        assert isinstance(vehicles, list)
        vehicles[1]["vehicle_number"] = vehicles[0]["vehicle_number"]

    cases.append(("duplicate_vehicle_number", result_status(duplicate_vehicle_number)))

    def mismatched_legacy(value: dict[str, object]) -> None:
        value["num_laps"] = 999

    cases.append(("mismatched_legacy", result_status(mismatched_legacy)))

    def mismatched_detail_identity(value: dict[str, object]) -> None:
        value["vehicle_number"] = 2
        value["vehicle_name"] = "GoKart2"

    cases.append(
        ("mismatched_detail_filename_identity", result_status(None, mismatched_detail_identity))
    )

    def mismatched_penalty_total(value: dict[str, object]) -> None:
        by_kind = value["penalty_by_kind"]
        assert isinstance(by_kind, dict)
        by_kind["wall"]["total_seconds"] += 1.0

    cases.append(("mismatched_penalty_total", result_status(None, mismatched_penalty_total)))

    failures = 0
    for name, status in cases:
        expected = oracle_module.PASS if name == "baseline" else oracle_module.RED
        if status != expected:
            failures += 1
            print(f"[FAIL] {name}: expected={expected} actual={status}")
        else:
            print(f"[PASS] {name}: {status}")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
