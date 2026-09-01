from pathlib import Path
import os
import re


def find_aichallenge_root() -> Path:
    for candidate in Path(__file__).resolve().parents:
        if (candidate / "run_autoware.bash").is_file():
            return candidate
    raise RuntimeError("aichallenge runtime root was not found")


AICHALLENGE_ROOT = find_aichallenge_root()
SPATIAL_CHECKPOINT = (
    "$(find-pkg-share tiny_lidar_net_controller)/ckpt/"
    "spatial_steering_adapter.npy"
)
SPATIAL_SHA256 = (
    "f3921c265677761bcf9458c61758d997b94d0b2045e87ebcee37ca94f3ed412c"
)


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


def test_runtime_validates_and_propagates_tiny_lidar_control_mode() -> None:
    runner = read("run_autoware.bash")
    system_launch = read(
        "workspace/src/aichallenge_system/"
        "aichallenge_system_launch/launch/aichallenge_system.launch.xml"
    )

    assert 'tiny_lidar_control_mode="${TINY_LIDAR_CONTROL_MODE:-}"' in runner
    assert (
        '"fixed"|"fixed_lidar_brake"|"ai"|"gap_teacher"|'
        '"precontact_teacher"|"speed_committed_teacher")' in runner
    )
    assert 'opts+=("tiny_lidar_control_mode:=${tiny_lidar_control_mode}")' in runner
    assert (
        '<arg name="tiny_lidar_control_mode" default="fixed_lidar_brake"/>'
        in system_launch
    )
    assert (
        '<arg name="tiny_lidar_control_mode" '
        'value="$(var tiny_lidar_control_mode)"/>'
        in system_launch
    )


def test_runtime_propagates_only_an_explicit_residual_checkpoint() -> None:
    runner = read("run_autoware.bash")
    system_launch = read(
        "workspace/src/aichallenge_system/"
        "aichallenge_system_launch/launch/aichallenge_system.launch.xml"
    )
    assert (
        'tiny_lidar_residual_ckpt_path="${TINY_LIDAR_RESIDUAL_CKPT_PATH:-}"'
        in runner
    )
    assert 'if [[ ! -f "${tiny_lidar_residual_ckpt_path}" ]]; then' in runner
    assert (
        'opts+=("tiny_lidar_residual_ckpt_path:=${tiny_lidar_residual_ckpt_path}")'
        in runner
    )
    assert (
        'tiny_lidar_residual_architecture='
        '"${TINY_LIDAR_RESIDUAL_ARCHITECTURE:-stateless}"' in runner
    )
    assert (
        'opts+=("tiny_lidar_residual_architecture:'
        '=${tiny_lidar_residual_architecture}")' in runner
    )
    assert '<arg name="tiny_lidar_residual_ckpt_path" default=""/>' in system_launch
    assert (
        '<arg name="tiny_lidar_residual_ckpt_path" '
        'value="$(var tiny_lidar_residual_ckpt_path)"/>'
        in system_launch
    )
    assert (
        '<arg name="tiny_lidar_residual_architecture" default="stateless"/>'
        in system_launch
    )


def test_runtime_preserves_packaged_spatial_default_and_explicit_overrides() -> None:
    runner = read("run_autoware.bash")
    system_launch = read(
        "workspace/src/aichallenge_system/"
        "aichallenge_system_launch/launch/aichallenge_system.launch.xml"
    )
    assert (
        'tiny_lidar_spatial_shadow_ckpt_path='
        '"${TINY_LIDAR_SPATIAL_SHADOW_CKPT_PATH:-}"' in runner
    )
    assert (
        'if [[ ! -f "${tiny_lidar_spatial_shadow_ckpt_path}" ]]; then'
        in runner
    )
    assert (
        'opts+=("tiny_lidar_spatial_shadow_ckpt_path:'
        '=${tiny_lidar_spatial_shadow_ckpt_path}")' in runner
    )
    assert (
        '<arg name="tiny_lidar_spatial_shadow_ckpt_path" '
        f'default="{SPATIAL_CHECKPOINT}"/>' in system_launch
    )
    assert (
        '<arg name="tiny_lidar_spatial_shadow_ckpt_path" '
        'value="$(var tiny_lidar_spatial_shadow_ckpt_path)"/>'
        in system_launch
    )
    assert (
        'tiny_lidar_spatial_shadow_expected_sha256='
        '"${TINY_LIDAR_SPATIAL_SHADOW_EXPECTED_SHA256:-}"' in runner
    )
    assert (
        'TINY_LIDAR_SPATIAL_SHADOW_EXPECTED_SHA256 requires '
        'TINY_LIDAR_SPATIAL_SHADOW_CKPT_PATH' in runner
    )
    assert (
        'opts+=("tiny_lidar_spatial_shadow_expected_sha256:'
        '=${tiny_lidar_spatial_shadow_expected_sha256,,}")' in runner
    )
    assert (
        '<arg name="tiny_lidar_spatial_shadow_expected_sha256" '
        f'default="{SPATIAL_SHA256}"/>' in system_launch
    )
    assert (
        '<arg name="tiny_lidar_spatial_shadow_expected_sha256" '
        'value="$(var tiny_lidar_spatial_shadow_expected_sha256)"/>'
        in system_launch
    )
    assert (
        'tiny_lidar_spatial_shadow_use_base_steering='
        '"${TINY_LIDAR_SPATIAL_SHADOW_USE_BASE_STEERING:-false}"' in runner
    )
    assert (
        'opts+=("tiny_lidar_spatial_shadow_use_base_steering:'
        '=${tiny_lidar_spatial_shadow_use_base_steering}")' in runner
    )
    assert (
        '<arg name="tiny_lidar_spatial_shadow_use_base_steering" '
        'default="true"/>' in system_launch
    )
    assert (
        '<arg name="tiny_lidar_spatial_shadow_use_base_steering" '
        'value="$(var tiny_lidar_spatial_shadow_use_base_steering)"/>'
        in system_launch
    )
    assert (
        'tiny_lidar_spatial_shadow_max_abs_delta_rad='
        '"${TINY_LIDAR_SPATIAL_SHADOW_MAX_ABS_DELTA_RAD:-1.2}"' in runner
    )
    assert '^0+([.]0+)?$' in runner
    assert (
        'opts+=("tiny_lidar_spatial_shadow_max_abs_delta_rad:'
        '=${tiny_lidar_spatial_shadow_max_abs_delta_rad}")' in runner
    )
    assert (
        '<arg name="tiny_lidar_spatial_shadow_max_abs_delta_rad" default="1.2"/>'
        in system_launch
    )
    assert (
        '<arg name="tiny_lidar_spatial_shadow_max_abs_delta_rad" '
        'value="$(var tiny_lidar_spatial_shadow_max_abs_delta_rad)"/>'
        in system_launch
    )


def test_runtime_preserves_spatial_authority_default_and_one_owner() -> None:
    runner = read("run_autoware.bash")
    system_launch = read(
        "workspace/src/aichallenge_system/"
        "aichallenge_system_launch/launch/aichallenge_system.launch.xml"
    )
    assert (
        'tiny_lidar_spatial_authority_enabled='
        '"${TINY_LIDAR_SPATIAL_AUTHORITY_ENABLED:-}"' in runner
    )
    assert 'true|false) ;;' in runner
    assert (
        'spatial authority requires TINY_LIDAR_SPATIAL_SHADOW_CKPT_PATH'
        in runner
    )
    assert (
        'spatial authority cannot be combined with '
        'TINY_LIDAR_RESIDUAL_CKPT_PATH' in runner
    )
    assert (
        'opts+=("tiny_lidar_spatial_authority_enabled:'
        '=${tiny_lidar_spatial_authority_enabled}")' in runner
    )
    assert (
        'tiny_lidar_spatial_authority_max_abs_delta_rad='
        '"${TINY_LIDAR_SPATIAL_AUTHORITY_MAX_ABS_DELTA_RAD:-}"' in runner
    )
    assert (
        'opts+=("tiny_lidar_spatial_authority_max_abs_delta_rad:'
        '=${tiny_lidar_spatial_authority_max_abs_delta_rad}")' in runner
    )
    assert (
        '<arg name="tiny_lidar_spatial_authority_enabled" default="true"/>'
        in system_launch
    )
    assert (
        '<arg name="tiny_lidar_spatial_authority_enabled" '
        'value="$(var tiny_lidar_spatial_authority_enabled)"/>'
        in system_launch
    )
    assert (
        '<arg name="tiny_lidar_spatial_authority_max_abs_delta_rad" '
        'default="1.2"/>' in system_launch
    )
    assert (
        '<arg name="tiny_lidar_spatial_authority_max_abs_delta_rad" '
        'value="$(var tiny_lidar_spatial_authority_max_abs_delta_rad)"/>'
        in system_launch
    )


def test_runtime_recurrent_candidate_is_opt_in_and_authority_is_explicit() -> None:
    runner = read("run_autoware.bash")
    system_launch = read(
        "workspace/src/aichallenge_system/"
        "aichallenge_system_launch/launch/aichallenge_system.launch.xml"
    )

    assert (
        'tiny_lidar_recurrent_shadow_ckpt_path='
        '"${TINY_LIDAR_RECURRENT_SHADOW_CKPT_PATH:-}"' in runner
    )
    assert (
        'if [[ ! -f "${tiny_lidar_recurrent_shadow_ckpt_path}" ]]; then'
        in runner
    )
    assert (
        'opts+=("tiny_lidar_recurrent_shadow_ckpt_path:'
        '=${tiny_lidar_recurrent_shadow_ckpt_path}")' in runner
    )
    assert (
        'TINY_LIDAR_RECURRENT_SHADOW_EXPECTED_SHA256 requires '
        'TINY_LIDAR_RECURRENT_SHADOW_CKPT_PATH' in runner
    )
    assert (
        'opts+=("tiny_lidar_recurrent_shadow_expected_sha256:'
        '=${tiny_lidar_recurrent_shadow_expected_sha256,,}")' in runner
    )
    assert (
        '<arg name="tiny_lidar_recurrent_shadow_ckpt_path" default=""/>'
        in system_launch
    )
    assert (
        '<arg name="tiny_lidar_recurrent_shadow_ckpt_path" '
        'value="$(var tiny_lidar_recurrent_shadow_ckpt_path)"/>'
        in system_launch
    )
    assert (
        '<arg name="tiny_lidar_recurrent_shadow_expected_sha256" '
        'default=""/>' in system_launch
    )
    assert (
        '<arg name="tiny_lidar_recurrent_shadow_expected_sha256" '
        'value="$(var tiny_lidar_recurrent_shadow_expected_sha256)"/>'
        in system_launch
    )
    assert (
        'tiny_lidar_recurrent_authority_enabled='
        '"${TINY_LIDAR_RECURRENT_AUTHORITY_ENABLED:-}"' in runner
    )
    assert (
        'tiny_lidar_recurrent_authority_max_abs_correction_rad='
        '"${TINY_LIDAR_RECURRENT_AUTHORITY_MAX_ABS_CORRECTION_RAD:-}"'
        in runner
    )
    assert (
        "recurrent authority requires TINY_LIDAR_RECURRENT_SHADOW_CKPT_PATH"
        in runner
    )
    assert (
        "recurrent authority requires "
        "TINY_LIDAR_RECURRENT_SHADOW_EXPECTED_SHA256" in runner
    )
    assert (
        'opts+=("tiny_lidar_recurrent_authority_enabled:'
        '=${tiny_lidar_recurrent_authority_enabled}")' in runner
    )
    assert (
        'opts+=("tiny_lidar_recurrent_authority_max_abs_correction_rad:'
        '=${tiny_lidar_recurrent_authority_max_abs_correction_rad}")'
        in runner
    )
    assert (
        '<arg name="tiny_lidar_recurrent_authority_enabled" default="false"/>'
        in system_launch
    )
    assert (
        '<arg name="tiny_lidar_recurrent_authority_enabled" '
        'value="$(var tiny_lidar_recurrent_authority_enabled)"/>'
        in system_launch
    )
    assert (
        '<arg name="tiny_lidar_recurrent_authority_max_abs_correction_rad" '
        'default="0.24"/>' in system_launch
    )
    assert (
        '<arg name="tiny_lidar_recurrent_authority_max_abs_correction_rad" '
        'value="$(var tiny_lidar_recurrent_authority_max_abs_correction_rad)"/>'
        in system_launch
    )


def test_runtime_tiny_lidar_acceleration_is_explicit_and_qualified_by_gate() -> None:
    runner = read("run_autoware.bash")
    system_launch = read(
        "workspace/src/aichallenge_system/"
        "aichallenge_system_launch/launch/aichallenge_system.launch.xml"
    )

    assert 'tiny_lidar_acceleration="${TINY_LIDAR_ACCELERATION:-}"' in runner
    assert (
        "TINY_LIDAR_ACCELERATION is only valid with "
        "AIC_CONTROL_METHOD=tiny_lidar_net" in runner
    )
    assert (
        'opts+=("tiny_lidar_acceleration:=${tiny_lidar_acceleration}")'
        in runner
    )
    assert '<arg name="tiny_lidar_acceleration" default="0.6"/>' in system_launch
    assert (
        '<arg name="tiny_lidar_acceleration" '
        'value="$(var tiny_lidar_acceleration)"/>' in system_launch
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
