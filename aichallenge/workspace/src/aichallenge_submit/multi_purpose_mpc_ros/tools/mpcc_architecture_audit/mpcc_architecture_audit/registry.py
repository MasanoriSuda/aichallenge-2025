"""Central experiment-registry validation."""

from __future__ import annotations

import json
from pathlib import Path
import re
from typing import Any
from typing import Mapping


class RegistryError(ValueError):
    """Raised when experiment history is not self-consistent."""


_HEX40 = re.compile(r"^[0-9a-f]{40}$")
_RESULTS = {"accepted", "rejected", "inconclusive"}


def _nonempty_string(value: object, field: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise RegistryError(f"{field} must be a non-empty string")
    return value


def validate_registry(document: Mapping[str, Any]) -> None:
    if document.get("schema_version") != 1:
        raise RegistryError("schema_version must be 1")
    snapshots = document.get("snapshots")
    experiments = document.get("experiments")
    if not isinstance(snapshots, list) or not isinstance(experiments, list):
        raise RegistryError("snapshots and experiments must be arrays")

    snapshot_ids: set[str] = set()
    for index, snapshot in enumerate(snapshots):
        if not isinstance(snapshot, Mapping):
            raise RegistryError(f"snapshot {index} must be an object")
        snapshot_id = _nonempty_string(snapshot.get("snapshot_id"), "snapshot_id")
        if snapshot_id in snapshot_ids:
            raise RegistryError(f"duplicate snapshot: {snapshot_id}")
        snapshot_ids.add(snapshot_id)
        _nonempty_string(snapshot.get("manifest"), "manifest")
        if not isinstance(snapshot.get("replay_ready"), bool):
            raise RegistryError(f"snapshot {snapshot_id} replay_ready must be boolean")

    experiment_ids: set[str] = set()
    for index, experiment in enumerate(experiments):
        if not isinstance(experiment, Mapping):
            raise RegistryError(f"experiment {index} must be an object")
        experiment_id = _nonempty_string(
            experiment.get("experiment_id"), "experiment_id"
        )
        if experiment_id in experiment_ids:
            raise RegistryError(f"duplicate experiment: {experiment_id}")
        experiment_ids.add(experiment_id)
        commit = _nonempty_string(experiment.get("baseline_commit"), "baseline_commit")
        if _HEX40.fullmatch(commit) is None:
            raise RegistryError(f"experiment {experiment_id} has invalid baseline_commit")
        references = experiment.get("snapshot_ids")
        if not isinstance(references, list):
            raise RegistryError(f"experiment {experiment_id} snapshot_ids must be an array")
        for snapshot_id in references:
            if snapshot_id not in snapshot_ids:
                raise RegistryError(
                    f"experiment {experiment_id} references unknown snapshot: {snapshot_id}"
                )
        for field in (
            "hypothesis",
            "changed_dimension",
            "reason",
            "production_impact",
            "revisit_condition",
        ):
            _nonempty_string(experiment.get(field), field)
        if experiment.get("result") not in _RESULTS:
            raise RegistryError(f"experiment {experiment_id} has invalid result")
        deleted_code = experiment.get("deleted_code")
        if not isinstance(deleted_code, list) or not all(
            isinstance(item, str) for item in deleted_code
        ):
            raise RegistryError(f"experiment {experiment_id} deleted_code must be strings")


def load_registry(path: Path) -> dict[str, Any]:
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise RegistryError(f"cannot read registry {path}: {error}") from error
    if not isinstance(document, dict):
        raise RegistryError("registry root must be an object")
    validate_registry(document)
    return document
