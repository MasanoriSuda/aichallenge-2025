"""Command-line interface for offline architecture evidence."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path
from typing import Sequence

from .classification import classify_comparison
from .manifest import ManifestError
from .manifest import load_snapshot_manifest
from .manifest import verify_snapshot_payloads
from .registry import RegistryError
from .registry import load_registry


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="mpcc-architecture-audit",
        description="Validate frozen MPCC evidence and classify A-D comparisons.",
    )
    commands = parser.add_subparsers(dest="command", required=True)

    snapshot = commands.add_parser("validate-snapshot")
    snapshot.add_argument("manifest", type=Path)
    snapshot.add_argument("--verify-payloads", action="store_true")

    registry = commands.add_parser("validate-registry")
    registry.add_argument("registry", type=Path)

    classify = commands.add_parser("classify")
    classify.add_argument("comparison", type=Path)
    classify.add_argument("--output", type=Path)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    try:
        if args.command == "validate-snapshot":
            document = load_snapshot_manifest(args.manifest)
            if args.verify_payloads:
                verify_snapshot_payloads(args.manifest, document)
            print(f"valid: {args.manifest}")
            print(f"replay_ready: {str(document['replay_ready']).lower()}")
            return 0
        if args.command == "validate-registry":
            document = load_registry(args.registry)
            print(f"valid: {args.registry}")
            print(f"snapshots: {len(document['snapshots'])}")
            print(f"experiments: {len(document['experiments'])}")
            return 0
        if args.command == "classify":
            document = json.loads(args.comparison.read_text(encoding="utf-8"))
            result = classify_comparison(document)
            rendered = json.dumps(asdict(result), indent=2, sort_keys=True) + "\n"
            if args.output is None:
                print(rendered, end="")
            else:
                args.output.parent.mkdir(parents=True, exist_ok=True)
                args.output.write_text(rendered, encoding="utf-8")
                print(args.output.resolve())
            return 0
    except (ManifestError, RegistryError, ValueError, OSError, json.JSONDecodeError) as error:
        parser.error(str(error))
    return 2
