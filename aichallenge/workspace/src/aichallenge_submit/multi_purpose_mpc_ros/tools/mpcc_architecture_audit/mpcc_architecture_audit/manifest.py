"""Immutable failure-snapshot manifest validation."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import re
from typing import Any
from typing import Mapping


class ManifestError(ValueError):
    """Raised when snapshot evidence is incomplete or internally inconsistent."""


REQUIRED_REPLAY_PAYLOADS = (
    "state",
    "reference",
    "wall_grid",
    "peers",
    "tactical",
    "problem",
    "warm_start",
    "physical_certificate",
)

_HEX40 = re.compile(r"^[0-9a-f]{40}$")
_HEX64 = re.compile(r"^[0-9a-f]{64}$")


def _require(mapping: Mapping[str, Any], key: str, expected: type) -> Any:
    value = mapping.get(key)
    if not isinstance(value, expected):
        raise ManifestError(f"{key} must be {expected.__name__}")
    return value


def _validate_payload(name: str, raw: object) -> None:
    if not isinstance(raw, Mapping):
        raise ManifestError(f"payload {name} must be an object")
    path = raw.get("path")
    sha256 = raw.get("sha256")
    size_bytes = raw.get("size_bytes")
    if not isinstance(path, str) or not path or Path(path).is_absolute():
        raise ManifestError(f"payload {name} path must be non-empty and relative")
    if not isinstance(sha256, str) or _HEX64.fullmatch(sha256) is None:
        raise ManifestError(f"payload {name} sha256 must be 64 lowercase hex digits")
    if not isinstance(size_bytes, int) or isinstance(size_bytes, bool) or size_bytes < 0:
        raise ManifestError(f"payload {name} size_bytes must be a non-negative integer")


def validate_snapshot_manifest(document: Mapping[str, Any]) -> None:
    if document.get("schema_version") != 1:
        raise ManifestError("schema_version must be 1")
    for key in ("snapshot_id", "run_id", "failure_family"):
        if not _require(document, key, str).strip():
            raise ManifestError(f"{key} must not be empty")
    commit = _require(document, "baseline_commit", str)
    if _HEX40.fullmatch(commit) is None:
        raise ManifestError("baseline_commit must be a full lowercase git SHA")
    domain = document.get("domain_id")
    decision = document.get("decision_id")
    if not isinstance(domain, int) or isinstance(domain, bool) or domain < 0:
        raise ManifestError("domain_id must be a non-negative integer")
    if not isinstance(decision, int) or isinstance(decision, bool) or decision < 0:
        raise ManifestError("decision_id must be a non-negative integer")
    replay_ready = document.get("replay_ready")
    if not isinstance(replay_ready, bool):
        raise ManifestError("replay_ready must be boolean")
    payloads = _require(document, "payloads", dict)
    for name, raw in payloads.items():
        if not isinstance(name, str) or not name:
            raise ManifestError("payload names must be non-empty strings")
        _validate_payload(name, raw)
    if replay_ready:
        for name in REQUIRED_REPLAY_PAYLOADS:
            if name not in payloads:
                raise ManifestError(f"missing replay payload: {name}")
    else:
        reason = document.get("incomplete_reason")
        if not isinstance(reason, str) or not reason.strip():
            raise ManifestError("incomplete snapshot requires incomplete_reason")


def load_snapshot_manifest(path: Path) -> dict[str, Any]:
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ManifestError(f"cannot read snapshot manifest {path}: {error}") from error
    if not isinstance(document, dict):
        raise ManifestError("snapshot manifest root must be an object")
    validate_snapshot_manifest(document)
    return document


def verify_snapshot_payloads(path: Path, document: Mapping[str, Any]) -> None:
    for name, raw in document["payloads"].items():
        payload_path = (path.parent / raw["path"]).resolve()
        try:
            payload_path.relative_to(path.parent.resolve())
        except ValueError as error:
            raise ManifestError(f"payload {name} escapes manifest directory") from error
        if not payload_path.is_file():
            raise ManifestError(f"payload {name} does not exist: {payload_path}")
        content = payload_path.read_bytes()
        if len(content) != raw["size_bytes"]:
            raise ManifestError(f"payload {name} size mismatch")
        if hashlib.sha256(content).hexdigest() != raw["sha256"]:
            raise ManifestError(f"payload {name} sha256 mismatch")
