#!/usr/bin/env python3
"""Source contract for the E2E controller launch path."""

from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).parents[1]
ENTRY = PACKAGE_ROOT / "launch" / "aichallenge_submit.launch.xml"
REFERENCE = PACKAGE_ROOT / "launch" / "reference.launch.xml"
CONTROL = PACKAGE_ROOT / "launch" / "control" / "tiny_lidar_net.launch.xml"
CONTROLLER_LAUNCH = (
    PACKAGE_ROOT.parent
    / "tiny_lidar_net_controller"
    / "launch"
    / "tiny_lidar_net.launch.xml"
)


def _parse(path: Path) -> ET.Element:
    return ET.parse(path).getroot()


def test_e2e_entry_selects_tiny_lidar_net() -> None:
    entry = _parse(ENTRY)
    arguments = {
        node.get("name"): node.get("default")
        for node in entry.findall("arg")
    }
    assert arguments["control_method"] == "tiny_lidar_net"

    includes = entry.findall("include")
    assert len(includes) == 1
    forwarded = {
        node.get("name"): node.get("value")
        for node in includes[0].findall("arg")
    }
    assert forwarded["control_method"] == "$(var control_method)"
    assert forwarded["tiny_lidar_ckpt_path"] == "$(var tiny_lidar_ckpt_path)"
    assert forwarded["tiny_lidar_residual_ckpt_path"] == (
        "$(var tiny_lidar_residual_ckpt_path)"
    )
    assert forwarded["tiny_lidar_residual_architecture"] == (
        "$(var tiny_lidar_residual_architecture)"
    )
    assert forwarded["tiny_lidar_spatial_shadow_ckpt_path"] == (
        "$(var tiny_lidar_spatial_shadow_ckpt_path)"
    )
    assert forwarded["tiny_lidar_control_mode"] == "$(var tiny_lidar_control_mode)"


def test_reference_retains_explicit_controller_switch() -> None:
    reference = _parse(REFERENCE)
    arguments = {
        node.get("name"): node.get("default")
        for node in reference.findall("arg")
    }
    assert arguments["control_method"] == "mpc"

    tiny_groups = [
        group
        for group in reference.findall("group")
        if group.get("if")
        == "$(eval \"'$(var control_method)' == 'tiny_lidar_net'\")"
    ]
    assert len(tiny_groups) == 1
    tiny_include = tiny_groups[0].find("include")
    assert tiny_include is not None
    tiny_forwarded = {
        node.get("name"): node.get("value")
        for node in tiny_include.findall("arg")
    }
    assert tiny_forwarded["ckpt_path"] == "$(var tiny_lidar_ckpt_path)"
    assert tiny_forwarded["residual_ckpt_path"] == (
        "$(var tiny_lidar_residual_ckpt_path)"
    )
    assert tiny_forwarded["residual_architecture"] == (
        "$(var tiny_lidar_residual_architecture)"
    )
    assert tiny_forwarded["spatial_shadow_ckpt_path"] == (
        "$(var tiny_lidar_spatial_shadow_ckpt_path)"
    )
    assert tiny_forwarded["control_mode"] == "$(var tiny_lidar_control_mode)"


def test_tiny_lidar_net_uses_awsim_lidar_topic_and_final_command_topic() -> None:
    control = _parse(CONTROL)
    arguments = {
        node.get("name"): node.get("default")
        for node in control.findall("arg")
    }
    assert arguments["model_type"] == "tiny_lidar_net"
    assert arguments["control_mode"] == "fixed_lidar_brake"

    include = control.find("include")
    assert include is not None
    forwarded = {
        node.get("name"): node.get("value")
        for node in include.findall("arg")
    }
    assert forwarded["control_cmd_topic"] == "/control/command/control_cmd"
    assert forwarded["control_mode"] == "$(var control_mode)"
    assert forwarded["residual_ckpt_path"] == "$(var residual_ckpt_path)"
    assert forwarded["residual_architecture"] == "$(var residual_architecture)"
    assert forwarded["spatial_shadow_ckpt_path"] == (
        "$(var spatial_shadow_ckpt_path)"
    )

    controller = _parse(CONTROLLER_LAUNCH)
    controller_arguments = {
        node.get("name"): node.get("default")
        for node in controller.findall("arg")
    }
    assert controller_arguments["scan_topic"] == "/sensing/lidar/scan"
    assert controller_arguments["control_mode"] == "fixed_lidar_brake"
    assert controller_arguments["residual_ckpt_path"] == ""
    assert controller_arguments["residual_architecture"] == "stateless"
    assert controller_arguments["spatial_shadow_ckpt_path"] == ""
    assert (
        controller_arguments["control_cmd_topic"]
        == "/control/command/control_cmd"
    )
