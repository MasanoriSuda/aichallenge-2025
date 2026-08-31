from pathlib import Path
import os
import re


def find_aichallenge_root() -> Path:
    for candidate in Path(__file__).resolve().parents:
        if (candidate / "run_autoware.bash").is_file():
            return candidate
    raise RuntimeError("aichallenge runtime root was not found")


AICHALLENGE_ROOT = find_aichallenge_root()


def read(relative_path: str) -> str:
    return (AICHALLENGE_ROOT / relative_path).read_text(encoding="utf-8")


def test_runtime_propagates_scenario_vehicle_count_to_launch() -> None:
    runner = read("run_autoware.bash")

    assert 'vehicle_count="${AIC_VEHICLE_COUNT:-1}"' in runner
    assert '"vehicle_count:=${vehicle_count}"' in runner


def test_runtime_propagates_explicit_control_method_to_submit_launch() -> None:
    runner = read("run_autoware.bash")
    system_launch = read(
        "workspace/src/aichallenge_system/"
        "aichallenge_system_launch/launch/aichallenge_system.launch.xml"
    )

    assert 'control_method="${AIC_CONTROL_METHOD:-tiny_lidar_net}"' in runner
    assert 'opts+=("control_method:=${control_method}")' in runner
    assert '<arg name="control_method" default="tiny_lidar_net"/>' in system_launch
    assert '<arg name="control_method" value="$(var control_method)"/>' in system_launch


def test_runtime_propagates_only_an_explicit_tiny_lidar_checkpoint_override() -> None:
    runner = read("run_autoware.bash")
    system_launch = read(
        "workspace/src/aichallenge_system/"
        "aichallenge_system_launch/launch/aichallenge_system.launch.xml"
    )

    assert 'tiny_lidar_ckpt_path="${TINY_LIDAR_CKPT_PATH:-}"' in runner
    assert 'if [[ -n "${tiny_lidar_ckpt_path}" ]]; then' in runner
    assert 'if [[ ! -f "${tiny_lidar_ckpt_path}" ]]; then' in runner
    assert 'opts+=("tiny_lidar_ckpt_path:=${tiny_lidar_ckpt_path}")' in runner
    assert '<arg name="tiny_lidar_ckpt_path" default=' in system_launch
    assert (
        '<arg name="tiny_lidar_ckpt_path" '
        'value="$(var tiny_lidar_ckpt_path)"/>'
        in system_launch
    )


def test_single_vehicle_empty_world_producer_is_simulation_only() -> None:
    system_launch = read(
        "workspace/src/aichallenge_system/"
        "aichallenge_system_launch/launch/aichallenge_system.launch.xml"
    )
    empty_launch = read(
        "workspace/src/aichallenge_system/"
        "aichallenge_system_launch/launch/mode/"
        "single_vehicle_empty_v2x.launch.xml"
    )

    assert '<arg name="vehicle_count" default="1"/>' in system_launch
    assert '<group if="$(var simulation)">' in system_launch
    assert "$(var vehicle_count)==1" in system_launch
    assert "single_vehicle_empty_v2x.launch.xml" in system_launch
    assert 'exec="single_vehicle_empty_v2x_publisher.py"' in empty_launch


def test_empty_world_publisher_emits_a_timestamped_empty_array() -> None:
    publisher_path = (
        AICHALLENGE_ROOT / "workspace/src/aichallenge_system/"
        "aichallenge_system_launch/script/"
        "single_vehicle_empty_v2x_publisher.py"
    )
    publisher = publisher_path.read_text(encoding="utf-8")

    assert os.access(publisher_path, os.X_OK)
    compact_publisher = re.sub(r"\s+", " ", publisher)
    assert (
        'create_publisher( V2XVehiclePositionArray, '
        '"/v2x/vehicle_positions", 1)'
        in compact_publisher
    )
    assert (
        "message.header.stamp = self.get_clock().now().to_msg()"
        in publisher
    )
    assert "message.vehicles = []" in publisher
