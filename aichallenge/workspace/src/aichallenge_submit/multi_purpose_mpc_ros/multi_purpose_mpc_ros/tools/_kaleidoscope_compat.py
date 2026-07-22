"""Load the extracted Kaleidoscope package from install or source layouts."""

from __future__ import annotations

import importlib
from pathlib import Path
import sys
from types import ModuleType


def load_module(module_name: str) -> ModuleType:
    """Return a canonical Kaleidoscope module for a legacy import path."""

    qualified_name = f"kaleidoscope.{module_name}"
    try:
        return importlib.import_module(qualified_name)
    except ModuleNotFoundError as error:
        if error.name not in {"kaleidoscope", qualified_name}:
            raise

    source_root = Path(__file__).resolve().parents[2] / "tools" / "kaleidoscope"
    if not source_root.is_dir():
        raise ModuleNotFoundError(
            f"Kaleidoscope source directory was not found: {source_root}"
        )
    source_root_text = str(source_root)
    if source_root_text not in sys.path:
        sys.path.insert(0, source_root_text)
    return importlib.import_module(qualified_name)
