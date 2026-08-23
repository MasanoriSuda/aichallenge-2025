"""Source-level deletion gates for the single-authority MPCC migration."""

from pathlib import Path
import re


SOURCE = (
    Path(__file__).resolve().parents[1] / "src" / "mpc_controller_cpp.cpp"
).read_text(encoding="utf-8")


def test_low_speed_direct_cannot_be_activated_as_production_authority() -> None:
    """The retired Gate2 bypass must not regain normal command ownership."""

    assert not re.search(r"low_speed_shift_control_active_\s*=\s*true", SOURCE)
    assert "return low_speed_shift_control(" not in SOURCE


def test_low_speed_direct_implementation_has_no_call_site() -> None:
    """Keep compatibility code unreachable until its final Slice 6 deletion."""

    assert len(re.findall(r"(?<![A-Za-z0-9_])low_speed_shift_control\(", SOURCE)) == 1
