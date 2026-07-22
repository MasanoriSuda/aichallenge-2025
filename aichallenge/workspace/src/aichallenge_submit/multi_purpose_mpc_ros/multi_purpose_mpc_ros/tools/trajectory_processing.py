"""Compatibility alias for :mod:`kaleidoscope.trajectory_processing`."""

import sys

from ._kaleidoscope_compat import load_module

sys.modules[__name__] = load_module("trajectory_processing")
