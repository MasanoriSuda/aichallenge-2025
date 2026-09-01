#!/usr/bin/env python3
"""Combine E2E motion, AWSIM race results, and runtime provenance.

This is deliberately stricter than ``analyze_e2e_run.py``.  A motion-only
report can prove that a kart did not stall, but it cannot prove Finish, laps,
or penalties.  This analyzer refuses to call such a run successful.
"""

import argparse
import hashlib
import json
from pathlib import Path
import re
import sys
from typing import Any, Iterable, Optional


SCHEMA_VERSION = 1
MOTION_SCHEMA_VERSION = 2
DETAIL_SCHEMA_VERSION = "v3"
SUMMARY_SCHEMA_VERSION = "v2"
DOMAIN_PATTERN = re.compile(r"^d([1-9][0-9]*)$")
LAUNCH_VALUE_PATTERN = re.compile(
    r"-\s+(tiny_lidar_ckpt_path|tiny_lidar_control_mode):\s*(\S+)"
)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"failed to read JSON {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ValueError(f"JSON root must be an object: {path}")
    return value


def parse_domains(run_dir: Path, requested: Optional[str]) -> list[int]:
    if requested:
        domains = []
        for item in requested.split(","):
            token = item.strip().lower()
            if token.startswith("d"):
                token = token[1:]
            if not token.isdigit() or int(token) <= 0:
                raise ValueError(f"invalid domain: {item!r}")
            domains.append(int(token))
        if len(set(domains)) != len(domains):
            raise ValueError("domains must not contain duplicates")
        return sorted(domains)

    domains = []
    for child in run_dir.iterdir():
        match = DOMAIN_PATTERN.match(child.name) if child.is_dir() else None
        if match:
            domains.append(int(match.group(1)))
    if not domains:
        raise ValueError(f"no dN domain directories found under {run_dir}")
    return sorted(domains)


def result_detail_path(run_dir: Path, domain: int) -> Optional[Path]:
    name = f"d{domain}-result-details.json"
    candidates = (run_dir / name, run_dir / f"d{domain}" / name)
    return next((path for path in candidates if path.is_file()), None)


def parse_runtime_provenance(log_path: Path) -> dict[str, Any]:
    values: dict[str, set[str]] = {
        "tiny_lidar_ckpt_path": set(),
        "tiny_lidar_control_mode": set(),
    }
    try:
        lines = log_path.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError as exc:
        raise ValueError(f"failed to read runtime log {log_path}: {exc}") from exc
    for line in lines:
        match = LAUNCH_VALUE_PATTERN.search(line)
        if match:
            values[match.group(1)].add(match.group(2))

    conflicts = {
        key: sorted(found) for key, found in values.items() if len(found) > 1
    }
    return {
        "checkpoint_path": next(iter(values["tiny_lidar_ckpt_path"]), None),
        "control_mode": next(iter(values["tiny_lidar_control_mode"]), None),
        "conflicts": conflicts,
    }


def validate_motion(value: dict[str, Any]) -> list[str]:
    reasons = []
    if value.get("schema_version") != MOTION_SCHEMA_VERSION:
        reasons.append("motion-schema-mismatch")
    admission = value.get("admission")
    if not isinstance(admission, dict) or admission.get("result") != "pass":
        reasons.append("motion-admission-failed")
    return reasons


def validate_detail(
    value: dict[str, Any], domain: int, max_penalty_count: int
) -> list[str]:
    reasons = []
    if value.get("schema_version") != DETAIL_SCHEMA_VERSION:
        reasons.append("result-detail-schema-mismatch")
    if value.get("vehicle_number") != domain:
        reasons.append("result-detail-domain-mismatch")
    lap_count = value.get("lap_count")
    required_laps = value.get("required_laps")
    if not isinstance(lap_count, int) or not isinstance(required_laps, int):
        reasons.append("result-detail-lap-contract-invalid")
    elif not value.get("finished") or lap_count < required_laps:
        reasons.append("race-not-finished")
    penalty_count = value.get("penalty_count")
    if not isinstance(penalty_count, int) or penalty_count < 0:
        reasons.append("result-detail-penalty-contract-invalid")
    elif penalty_count > max_penalty_count:
        reasons.append("penalty-limit-exceeded")
    events = value.get("penalty_events")
    if isinstance(penalty_count, int) and (
        not isinstance(events, list) or len(events) != penalty_count
    ):
        reasons.append("result-detail-penalty-count-mismatch")
    return reasons


def summary_vehicles(summary: dict[str, Any], domain: int) -> list[dict[str, Any]]:
    vehicles = summary.get("vehicles")
    if not isinstance(vehicles, list):
        return []
    return [
        item
        for item in vehicles
        if isinstance(item, dict) and item.get("vehicle_number") == domain
    ]


def analyze_domain(
    run_dir: Path,
    domain: int,
    summary: dict[str, Any],
    max_penalty_count: int,
    expected_control_mode: Optional[str],
    expected_checkpoint_path: Optional[str],
) -> dict[str, Any]:
    domain_dir = run_dir / f"d{domain}"
    motion_path = domain_dir / "e2e-run-analysis.json"
    detail_path = result_detail_path(run_dir, domain)
    log_path = domain_dir / "autoware.log"
    missing = []
    for label, path in (
        ("motion-analysis", motion_path),
        ("result-detail", detail_path),
        ("autoware-log", log_path),
    ):
        if path is None or not path.is_file():
            missing.append(label)
    if missing:
        return {
            "domain": domain,
            "status": "incomplete",
            "reasons": [f"missing-{label}" for label in missing],
        }

    reasons = []
    try:
        motion = load_json(motion_path)
        detail = load_json(detail_path)
        provenance = parse_runtime_provenance(log_path)
    except ValueError as exc:
        return {
            "domain": domain,
            "status": "fail",
            "reasons": [str(exc)],
        }

    reasons.extend(validate_motion(motion))
    reasons.extend(validate_detail(detail, domain, max_penalty_count))
    summary_candidates = summary_vehicles(summary, domain)
    summary_matches = [
        vehicle
        for vehicle in summary_candidates
        if all(
            vehicle.get(key) == detail.get(key)
            for key in ("finished", "lap_count")
        )
    ]
    if not summary_candidates:
        reasons.append("result-summary-domain-missing")
    elif not summary_matches:
        reasons.append("result-summary-race-mismatch")

    if provenance["conflicts"]:
        reasons.append("runtime-provenance-conflict")
    if provenance["control_mode"] is None:
        reasons.append("runtime-control-mode-missing")
    elif (
        expected_control_mode is not None
        and provenance["control_mode"] != expected_control_mode
    ):
        reasons.append("runtime-control-mode-mismatch")
    if provenance["checkpoint_path"] is None:
        reasons.append("runtime-checkpoint-path-missing")
    elif (
        expected_checkpoint_path is not None
        and provenance["checkpoint_path"] != expected_checkpoint_path
    ):
        reasons.append("runtime-checkpoint-path-mismatch")

    return {
        "domain": domain,
        "status": "fail" if reasons else "pass",
        "reasons": reasons,
        "artifacts": {
            "motion_analysis": str(motion_path),
            "result_detail": str(detail_path),
            "autoware_log": str(log_path),
            "motion_sha256": sha256_file(motion_path),
            "result_detail_sha256": sha256_file(detail_path),
        },
        "runtime": provenance,
        "summary_crosscheck": {
            "domain_entries": len(summary_candidates),
            "matching_race_entries": len(summary_matches),
            # Bundled AWSIM reports runtime NPCs with vehicle_number=1 as well.
            # The per-domain v3 detail remains the identity authority.
            "identity_ambiguous": len(summary_candidates) > 1,
        },
        "race": {
            "finished": detail.get("finished"),
            "lap_count": detail.get("lap_count"),
            "required_laps": detail.get("required_laps"),
            "min_lap_time": detail.get("min_lap_time"),
            "avg_lap_time": detail.get("avg_lap_time"),
            "total_lap_time": detail.get("total_lap_time"),
            "penalty_count": detail.get("penalty_count"),
            "penalty_total_seconds": detail.get("penalty_total_seconds"),
            "penalty_by_kind": detail.get("penalty_by_kind"),
        },
        "motion": motion.get("motion"),
    }


def analyze_competition(
    run_dir: Path,
    domains: Iterable[int],
    max_penalty_count: int = 0,
    expected_control_mode: Optional[str] = None,
    expected_checkpoint_path: Optional[str] = None,
    checkpoint_file: Optional[Path] = None,
    expected_checkpoint_sha256: Optional[str] = None,
) -> dict[str, Any]:
    summary_path = run_dir / "result-summary.json"
    if not summary_path.is_file():
        return {
            "schema_version": SCHEMA_VERSION,
            "run_dir": str(run_dir),
            "status": "incomplete",
            "reasons": ["missing-result-summary"],
            "domains": [],
        }
    try:
        summary = load_json(summary_path)
    except ValueError as exc:
        return {
            "schema_version": SCHEMA_VERSION,
            "run_dir": str(run_dir),
            "status": "fail",
            "reasons": [str(exc)],
            "domains": [],
        }

    global_reasons = []
    if summary.get("schema_version") != SUMMARY_SCHEMA_VERSION:
        global_reasons.append("result-summary-schema-mismatch")

    checkpoint = None
    if checkpoint_file is not None:
        if not checkpoint_file.is_file():
            global_reasons.append("checkpoint-file-missing")
        else:
            checkpoint_hash = sha256_file(checkpoint_file)
            checkpoint = {
                "path": str(checkpoint_file),
                "sha256": checkpoint_hash,
            }
            if (
                expected_checkpoint_sha256 is not None
                and checkpoint_hash != expected_checkpoint_sha256.lower()
            ):
                global_reasons.append("checkpoint-sha256-mismatch")
    elif expected_checkpoint_sha256 is not None:
        global_reasons.append("checkpoint-file-required-for-sha256")

    domain_results = [
        analyze_domain(
            run_dir,
            domain,
            summary,
            max_penalty_count,
            expected_control_mode,
            expected_checkpoint_path,
        )
        for domain in domains
    ]
    if any(item["status"] == "incomplete" for item in domain_results):
        status = "incomplete"
    elif global_reasons or any(item["status"] == "fail" for item in domain_results):
        status = "fail"
    else:
        status = "pass"
    return {
        "schema_version": SCHEMA_VERSION,
        "run_dir": str(run_dir),
        "status": status,
        "reasons": global_reasons,
        "thresholds": {"max_penalty_count": max_penalty_count},
        "expected_runtime": {
            "control_mode": expected_control_mode,
            "checkpoint_path": expected_checkpoint_path,
            "checkpoint_sha256": expected_checkpoint_sha256,
        },
        "checkpoint_artifact": checkpoint,
        "artifacts": {
            "result_summary": str(summary_path),
            "result_summary_sha256": sha256_file(summary_path),
        },
        "domains": domain_results,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--domains", help="Comma-separated domain IDs; default discovers dN")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--max-penalty-count", type=int, default=0)
    parser.add_argument("--expected-control-mode")
    parser.add_argument("--expected-checkpoint-path")
    parser.add_argument("--checkpoint-file", type=Path)
    parser.add_argument("--expected-checkpoint-sha256")
    parser.add_argument(
        "--fail-on-rejection",
        action="store_true",
        help="Exit 2 for fail and 3 for incomplete instead of always succeeding",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.max_penalty_count < 0:
        raise SystemExit("--max-penalty-count must be non-negative")
    run_dir = args.run_dir.expanduser().resolve()
    try:
        domains = parse_domains(run_dir, args.domains)
        result = analyze_competition(
            run_dir,
            domains,
            max_penalty_count=args.max_penalty_count,
            expected_control_mode=args.expected_control_mode,
            expected_checkpoint_path=args.expected_checkpoint_path,
            checkpoint_file=(
                None
                if args.checkpoint_file is None
                else args.checkpoint_file.expanduser().resolve()
            ),
            expected_checkpoint_sha256=(
                None
                if args.expected_checkpoint_sha256 is None
                else args.expected_checkpoint_sha256.lower()
            ),
        )
    except (OSError, ValueError) as exc:
        print(f"e2e-competition-analysis: error: {exc}", file=sys.stderr)
        return 1
    rendered = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    print(rendered, end="")
    if not args.fail_on_rejection:
        return 0
    if result["status"] == "incomplete":
        return 3
    return 2 if result["status"] == "fail" else 0


if __name__ == "__main__":
    sys.exit(main())
