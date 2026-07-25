"""Repository layout discovery and reproducibility records."""

from __future__ import annotations

import hashlib
from pathlib import Path
import subprocess
from typing import Any


REPO_MARKER = Path(
    "aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros"
)


def find_repository_root(start: Path | None = None) -> Path:
    """Find the AI Challenge repository root without accepting moved layouts."""

    current = (start or Path(__file__)).resolve()
    if current.is_file():
        current = current.parent
    for candidate in (current, *current.parents):
        if (candidate / REPO_MARKER).is_dir():
            return candidate
    raise FileNotFoundError(
        "AI Challenge repository layout was not found; keep localization_scope under "
        "multi_purpose_mpc_ros/tools/localization_scope"
    )


def _resolve_user_path(root: Path, raw: str | None) -> Path | None:
    if not raw:
        return None
    path = Path(raw).expanduser()
    if not path.is_absolute():
        path = root / path
    return path.resolve()


def default_inputs(root: Path) -> dict[str, Any]:
    package = root / REPO_MARKER
    submit = package.parent
    return {
        "trajectory": package / "env/final_ver3/traj_mincurv.csv",
        "configs": {
            "mpc": package / "config/config.yaml",
            "localization": [
                submit / "imu_corrector/config/imu_corrector.param.yaml",
                submit / "imu_gnss_poser/config/imu_gnss_poser.param.yaml",
                submit
                / "aichallenge_submit_launch/config/vehicle_velocity_converter.param.yaml",
            ],
            "launch": submit / "aichallenge_submit_launch/launch/reference.launch.xml",
        },
    }


def resolve_inputs(root: Path, metadata: dict[str, Any]) -> dict[str, Any]:
    """Resolve trajectory and configuration paths from metadata plus defaults."""

    defaults = default_inputs(root)
    trajectory = _resolve_user_path(root, metadata["trajectory"].get("path"))
    if trajectory is None:
        trajectory = defaults["trajectory"]

    config_meta = metadata["configs"]
    mpc = _resolve_user_path(root, config_meta.get("mpc")) or defaults["configs"]["mpc"]
    launch = (
        _resolve_user_path(root, config_meta.get("launch"))
        or defaults["configs"]["launch"]
    )
    localization_raw = config_meta.get("localization") or defaults["configs"]["localization"]
    localization = [
        resolved
        for raw in localization_raw
        if (resolved := _resolve_user_path(root, str(raw))) is not None
    ]
    return {
        "trajectory": trajectory,
        "configs": {"mpc": mpc, "localization": localization, "launch": launch},
    }


def sha256_file(path: Path) -> str | None:
    if not path.is_file():
        return None
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def file_record(path: Path, root: Path) -> dict[str, Any]:
    try:
        display_path = str(path.relative_to(root))
    except ValueError:
        display_path = str(path)
    return {
        "path": display_path,
        "exists": path.is_file(),
        "sha256": sha256_file(path),
    }


def git_record(root: Path) -> dict[str, Any]:
    """Return Git provenance without mutating the checkout."""

    try:
        commit = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=root,
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
        status = subprocess.run(
            ["git", "status", "--porcelain"],
            cwd=root,
            check=True,
            capture_output=True,
            text=True,
        ).stdout
        return {"commit": commit, "dirty": bool(status.strip())}
    except (OSError, subprocess.CalledProcessError):
        return {"commit": None, "dirty": None}


def build_input_manifest(
    root: Path, metadata: dict[str, Any], resolved: dict[str, Any]
) -> dict[str, Any]:
    configs = resolved["configs"]
    return {
        "repository": git_record(root),
        "trajectory": {
            "requested_path": metadata["trajectory"].get("path"),
            "runtime_topic": metadata["trajectory"].get("topic"),
            **file_record(resolved["trajectory"], root),
        },
        "configs": {
            "mpc": file_record(configs["mpc"], root),
            "localization": [file_record(path, root) for path in configs["localization"]],
            "launch": file_record(configs["launch"], root),
            "overrides": metadata["configs"].get("overrides", {}),
        },
    }
