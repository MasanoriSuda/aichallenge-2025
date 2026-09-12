"""Exercise the participant's real MPC XML entry without starting ROS nodes."""

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.frontend import Parser
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters


PACKAGE = Path(__file__).parents[1]


@pytest.mark.parametrize(
    'environment,override,expected',
    [(None, None, 0), ('1', None, 1), ('2', None, 2), ('4', None, 4), ('1', '3', 3)],
)
def test_participant_mpc_entry_receives_declared_vehicle_count(
    monkeypatch, environment, override, expected
):
    if environment is None:
        monkeypatch.delenv('AIC_VEHICLE_COUNT', raising=False)
    else:
        monkeypatch.setenv('AIC_VEHICLE_COUNT', environment)
    # Follow the entry selected by the standard participant launch. Testing the
    # package's standalone Python launch misses this actual production boundary.
    reference = ET.parse(PACKAGE / 'launch/reference.launch.xml').getroot()
    entries = [
        item for item in reference.iter('include')
        if item.get('file', '').endswith('/control/mpc.launch.xml')
    ]
    assert len(entries) == 1
    path = PACKAGE / entries[0].get('file').split(')', 1)[1].lstrip('/')
    entity, parser = Parser.load(str(path))
    description = parser.parse_description(entity)
    context = LaunchContext()
    if override is not None:
        context.launch_configurations['recovery_vehicle_count'] = override
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    nodes = [action for action in description.entities if isinstance(action, Node)]
    assert len(nodes) == 1
    parameters = evaluate_parameters(context, nodes[0]._Node__parameters)
    resolved = {key: value for item in parameters for key, value in item.items()}
    value = resolved['recovery_vehicle_count']
    assert type(value) is int
    assert value == expected
