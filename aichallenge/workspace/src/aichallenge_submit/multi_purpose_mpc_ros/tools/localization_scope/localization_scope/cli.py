"""Command-line interface for Localization Scope."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import sys
from typing import Sequence

from .analysis import analyze_run
from .analysis import compare_runs
from .bag_reader import BagReadError
from .bag_reader import read_bag
from .metadata import MetadataError
from .metadata import load_metadata
from .metadata import validate_metadata
from .metadata import write_template
from .models import RunAnalysis
from .report import write_comparison_report
from .report import write_catalog
from .report import write_single_report
from .repository import build_input_manifest
from .repository import find_repository_root
from .repository import resolve_inputs
from .trajectory import TrajectoryError
from .trajectory import load_trajectory_csv


@dataclass(frozen=True)
class RunInput:
    bag: Path
    metadata: Path | None


def _find_metadata(start: Path) -> Path | None:
    candidates = []
    current = start if start.is_dir() else start.parent
    for directory in (current, *list(current.parents)[:3]):
        candidate = directory / "run-metadata.json"
        if candidate.is_file():
            candidates.append(candidate)
    return candidates[0] if candidates else None


def _resolve_run(path: Path, metadata_override: Path | None) -> RunInput:
    resolved = path.expanduser().resolve()
    if resolved.is_file():
        if resolved.suffix.lower() not in {".mcap", ".db3"}:
            raise FileNotFoundError(f"run input is not a supported bag file: {resolved}")
        return RunInput(resolved, metadata_override or _find_metadata(resolved))

    if not resolved.is_dir():
        raise FileNotFoundError(f"run input does not exist: {resolved}")
    metadata = metadata_override or _find_metadata(resolved)
    direct_links = [
        resolved / "rosbag2_autoware.mcap",
        resolved / "rosbag2_autoware_0.mcap",
    ]
    for candidate in direct_links:
        if candidate.is_file():
            return RunInput(candidate, metadata)
    if (resolved / "metadata.yaml").is_file() and (
        any(resolved.glob("*.mcap")) or any(resolved.glob("*.db3"))
    ):
        return RunInput(resolved, metadata)
    bag_files = sorted(
        [*resolved.glob("*.mcap"), *resolved.glob("*.db3")],
        key=lambda item: str(item),
    )
    if len(bag_files) == 1:
        return RunInput(bag_files[0], metadata)
    recursive = sorted(
        [*resolved.glob("**/*.mcap"), *resolved.glob("**/*.db3")],
        key=lambda item: str(item),
    )
    if len(recursive) == 1:
        return RunInput(recursive[0], metadata)
    if not recursive:
        raise FileNotFoundError(f"no .mcap or .db3 bag found under {resolved}")
    raise FileNotFoundError(
        f"multiple bags found under {resolved}; specify one bag or vehicle directory"
    )


def _analyze_input(
    run_path: Path,
    *,
    metadata_path: Path | None,
    trajectory_override: Path | None,
    repository_root: Path,
) -> RunAnalysis:
    run = _resolve_run(run_path, metadata_path)
    metadata, metadata_warnings = load_metadata(run.metadata)
    if trajectory_override is not None:
        metadata["trajectory"]["path"] = str(trajectory_override)
    metadata["topics"]["runtime_trajectory"] = metadata["trajectory"]["topic"]
    resolved_inputs = resolve_inputs(repository_root, metadata)
    trajectory = load_trajectory_csv(resolved_inputs["trajectory"])
    bag_data = read_bag(run.bag, metadata["topics"])
    bag_data.warnings[:0] = metadata_warnings
    manifest = build_input_manifest(repository_root, metadata, resolved_inputs)
    return analyze_run(bag_data, metadata, manifest, trajectory)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="localization-scope",
        description="Offline localization report for Automotive AI Challenge rosbag runs.",
    )
    parser.add_argument(
        "--repository-root",
        type=Path,
        help="AI Challenge repository root; normally discovered automatically.",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    init = subparsers.add_parser("init", help="write a run-metadata.json template")
    init.add_argument(
        "output",
        nargs="?",
        type=Path,
        default=Path("run-metadata.json"),
    )
    init.add_argument("--force", action="store_true", help="overwrite an existing template")

    validate = subparsers.add_parser("validate", help="validate run metadata")
    validate.add_argument("metadata", type=Path)

    report = subparsers.add_parser("report", help="generate a single-run report")
    report.add_argument("run", type=Path, help="bag file, bag directory, or run directory")
    report.add_argument("--metadata", type=Path)
    report.add_argument("--trajectory", type=Path)
    report.add_argument(
        "--output-dir", type=Path, default=Path("localization-scope-report")
    )

    compare = subparsers.add_parser("compare", help="compare baseline and candidate runs")
    compare.add_argument("baseline", type=Path)
    compare.add_argument("candidate", type=Path)
    compare.add_argument("--baseline-metadata", type=Path)
    compare.add_argument("--candidate-metadata", type=Path)
    compare.add_argument("--baseline-trajectory", type=Path)
    compare.add_argument("--candidate-trajectory", type=Path)
    compare.add_argument(
        "--output-dir", type=Path, default=Path("localization-scope-comparison")
    )

    catalog = subparsers.add_parser(
        "catalog",
        help="scan run metadata and build a browser single/vs selector",
    )
    catalog.add_argument(
        "runs_root",
        type=Path,
        help="directory containing one run-metadata.json per run",
    )
    catalog.add_argument(
        "--output-dir", type=Path, default=Path("localization-scope-catalog")
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)
    try:
        if args.command == "init":
            write_template(args.output, overwrite=args.force)
            print(args.output.resolve())
            return 0
        if args.command == "validate":
            metadata, warnings = load_metadata(args.metadata)
            validate_metadata(metadata)
            print(f"valid: {args.metadata}")
            for warning in warnings:
                print(f"warning: {warning}")
            return 0

        root = (
            args.repository_root.expanduser().resolve()
            if args.repository_root
            else find_repository_root()
        )
        if args.command == "report":
            analysis = _analyze_input(
                args.run,
                metadata_path=args.metadata,
                trajectory_override=args.trajectory,
                repository_root=root,
            )
            manifest_path, summary_path, report_path = write_single_report(
                analysis, args.output_dir
            )
            print(f"manifest: {manifest_path.resolve()}")
            print(f"summary: {summary_path.resolve()}")
            print(f"report: {report_path.resolve()}")
            return 0
        if args.command == "compare":
            baseline = _analyze_input(
                args.baseline,
                metadata_path=args.baseline_metadata,
                trajectory_override=args.baseline_trajectory,
                repository_root=root,
            )
            candidate = _analyze_input(
                args.candidate,
                metadata_path=args.candidate_metadata,
                trajectory_override=args.candidate_trajectory,
                repository_root=root,
            )
            comparison = compare_runs(baseline, candidate)
            summary_path, report_path = write_comparison_report(
                baseline, candidate, comparison, args.output_dir
            )
            print(f"summary: {summary_path.resolve()}")
            print(f"report: {report_path.resolve()}")
            return 0
        if args.command == "catalog":
            runs_root = args.runs_root.expanduser().resolve()
            metadata_files = sorted(runs_root.glob("**/run-metadata.json"))
            if not metadata_files:
                raise FileNotFoundError(
                    f"no run-metadata.json found under {runs_root}"
                )
            analyses = [
                _analyze_input(
                    metadata_path.parent,
                    metadata_path=metadata_path,
                    trajectory_override=None,
                    repository_root=root,
                )
                for metadata_path in metadata_files
            ]
            comparisons = {
                f"{left}:{right}": compare_runs(
                    analyses[left], analyses[right]
                )
                for left in range(len(analyses))
                for right in range(len(analyses))
                if left != right
            }
            index_path, catalog_path = write_catalog(
                analyses, comparisons, args.output_dir
            )
            print(f"index: {index_path.resolve()}")
            print(f"catalog: {catalog_path.resolve()}")
            return 0
        parser.error(f"unknown command: {args.command}")
    except (
        BagReadError,
        FileExistsError,
        FileNotFoundError,
        MetadataError,
        TrajectoryError,
    ) as error:
        print(f"localization-scope: error: {error}", file=sys.stderr)
        return 2
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
