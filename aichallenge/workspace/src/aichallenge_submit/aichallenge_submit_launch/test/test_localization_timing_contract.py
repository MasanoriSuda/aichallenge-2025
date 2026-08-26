#!/usr/bin/env python3
"""Source contract for EKF measurement time ownership."""

from pathlib import Path
import xml.etree.ElementTree as ET


LAUNCH_PATH = Path(__file__).parents[1] / "launch" / "reference.launch.xml"


def _root() -> ET.Element:
    return ET.parse(LAUNCH_PATH).getroot()


def _argument_default(root: ET.Element, name: str) -> str:
    matches = [node for node in root.findall("arg") if node.get("name") == name]
    assert len(matches) == 1, f"expected one launch argument named {name}"
    default = matches[0].get("default")
    assert default is not None
    return default


def test_unmeasured_pose_delay_is_not_enabled_by_default() -> None:
    root = _root()
    assert _argument_default(root, "simulation_pose_additional_delay") == "0.0"
    assert _argument_default(root, "vehicle_pose_additional_delay") == "0.0"


def test_selected_pose_delay_remains_an_explicit_ekf_override() -> None:
    root = _root()
    lets = [
        node for node in root.findall("let")
        if node.get("name") == "pose_additional_delay_var"
    ]
    assert len(lets) == 2
    simulation = next(node for node in lets if node.get("if") == "$(var simulation)")
    vehicle = next(node for node in lets if node.get("unless") == "$(var simulation)")
    assert simulation.get("value") == "$(var simulation_pose_additional_delay)"
    assert vehicle.get("value") == "$(var vehicle_pose_additional_delay)"

    ekf_delay_parameters = [
        node
        for node in root.iter("arg")
        if node.get("name") == "pose_additional_delay"
    ]
    assert len(ekf_delay_parameters) == 1
    assert ekf_delay_parameters[0].get("value") == "$(var pose_additional_delay_var)"
