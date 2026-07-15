#!/usr/bin/env python3
"""Static Contract/Safety Floor oracle for the MPC Phase 0 baseline.

This intentionally checks only repository facts.  DDS graph, QoS, publisher
counts, topic rates, container identity, and generated artifacts remain
``NEEDS_RUNTIME`` until they are observed from the canonical Docker launch.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable, Sequence


PASS = "PASS"
RED = "RED"
NEEDS_RUNTIME = "NEEDS_RUNTIME"


@dataclass(frozen=True)
class Check:
    check_id: str
    status: str
    summary: str
    evidence: tuple[str, ...]


class Oracle:
    def __init__(self, repo_root: Path) -> None:
        self.root = repo_root.resolve()
        self.checks: list[Check] = []

    def path(self, relative: str) -> Path:
        return self.root / relative

    def read(self, relative: str) -> str:
        path = self.path(relative)
        try:
            return path.read_text(encoding="utf-8")
        except (OSError, UnicodeError) as error:
            raise RuntimeError(f"cannot read {relative}: {error}") from error

    def sha256(self, relative: str) -> str:
        digest = hashlib.sha256()
        with self.path(relative).open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
        return digest.hexdigest()

    def line_evidence(self, relative: str, needle: str) -> str:
        for line_number, line in enumerate(self.read(relative).splitlines(), start=1):
            if needle in line:
                return f"{relative}:{line_number}: {line.strip()}"
        return f"{relative}: missing {needle!r}"

    def add(
        self,
        check_id: str,
        passed: bool,
        summary: str,
        evidence: Iterable[str],
    ) -> None:
        self.checks.append(
            Check(check_id, PASS if passed else RED, summary, tuple(evidence))
        )

    def pending(self, check_id: str, summary: str, evidence: Iterable[str]) -> None:
        self.checks.append(Check(check_id, NEEDS_RUNTIME, summary, tuple(evidence)))

    def check_launch_entry(self) -> None:
        submit_launch = (
            "aichallenge/workspace/src/aichallenge_submit/"
            "aichallenge_submit_launch/launch/aichallenge_submit.launch.xml"
        )
        system_launch = (
            "aichallenge/workspace/src/aichallenge_system/"
            "aichallenge_system_launch/launch/aichallenge_system.launch.xml"
        )
        entry_exists = self.path(submit_launch).is_file()
        submit_text = self.read(submit_launch)
        system_text = self.read(system_launch)
        included = (
            "$(find-pkg-share aichallenge_submit_launch)/launch/"
            "aichallenge_submit.launch.xml"
        ) in system_text
        reference_reachable = (
            "$(find-pkg-share aichallenge_submit_launch)/launch/reference.launch.xml"
            in submit_text
        )
        argument_passthrough = all(
            item in submit_text
            for item in (
                'name="simulation" value="$(var simulation)"',
                'name="use_sim_time" value="$(var use_sim_time)"',
                'name="sensor_model" value="$(var sensor_model)"',
                'name="launch_vehicle_interface" value="$(var launch_vehicle_interface)"',
            )
        )
        self.add(
            "C-ENTRY-LAUNCH",
            entry_exists and included and reference_reachable and argument_passthrough,
            "system -> participant entry -> reference launch is reachable with required arguments",
            (
                f"{submit_launch}: {'exists' if entry_exists else 'missing'}",
                self.line_evidence(system_launch, "aichallenge_submit.launch.xml"),
                self.line_evidence(submit_launch, "reference.launch.xml"),
                f"{submit_launch}: argument_passthrough={argument_passthrough}",
            ),
        )

    def check_control_methods(self) -> None:
        reference_launch = (
            "aichallenge/workspace/src/aichallenge_submit/"
            "aichallenge_submit_launch/launch/reference.launch.xml"
        )
        text = self.read(reference_launch)
        methods = set(
            re.findall(r"'\$\(var control_method\)'\s*==\s*'([^']+)'", text)
        )
        expected = {"mpc", "pure_pursuit", "tiny_lidar_net", "pilot_net", "joycon"}
        default_is_mpc = bool(
            re.search(r'<arg\s+name="control_method"\s+default="mpc"', text)
        )
        control_root = (
            "aichallenge/workspace/src/aichallenge_submit/"
            "aichallenge_submit_launch/launch/control"
        )
        route_files = {
            method: f"{control_root}/{method}.launch.xml" for method in expected
        }
        routes_exist = all(self.path(path).is_file() for path in route_files.values())
        routes_included = all(
            f"/launch/control/{method}.launch.xml" in text for method in expected
        )

        mpc_source = self.read(
            "aichallenge/workspace/src/aichallenge_submit/"
            "multi_purpose_mpc_ros/src/mpc_controller_cpp.cpp"
        )
        pure_launch = self.read(route_files["pure_pursuit"])
        pure_source = self.read(
            "aichallenge/workspace/src/aichallenge_submit/"
            "simple_pure_pursuit/src/simple_pure_pursuit.cpp"
        )
        tiny_launch = self.read(route_files["tiny_lidar_net"])
        tiny_source = self.read(
            "aichallenge/workspace/src/aichallenge_submit/tiny_lidar_net_controller/"
            "tiny_lidar_net_controller/tiny_lidar_net_controller_node.py"
        )
        pilot_launch = self.read(route_files["pilot_net"])
        pilot_source = self.read(
            "aichallenge/workspace/src/aichallenge_submit/pilot_net_controller/"
            "pilot_net_controller/pilot_net_controller_node.py"
        )
        joy_launch = self.read(route_files["joycon"])
        joy_source = self.read(
            "aichallenge/workspace/src/aichallenge_tools/teleop_manager/"
            "src/teleop_manager_node.cpp"
        )
        route_contracts = {
            "mpc": all(
                token in mpc_source
                for token in (
                    "AckermannControlCommand",
                    '"/control/command/control_cmd"',
                    '"/localization/kinematic_state"',
                    '"planning/scenario_planning/trajectory"',
                )
            ),
            "pure_pursuit": all(
                token in pure_launch
                for token in (
                    'from="input/kinematics" to="/localization/kinematic_state"',
                    'from="input/trajectory" to="/planning/scenario_planning/trajectory"',
                    'from="output/control_cmd" to="/control/command/control_cmd"',
                )
            ) and all(
                token in pure_source
                for token in (
                    "create_publisher<AckermannControlCommand>",
                    "create_subscription<Odometry>",
                    "create_subscription<Trajectory>",
                )
            ),
            "tiny_lidar_net": all(
                token in tiny_launch
                for token in (
                    'name="control_cmd_topic" value="/control/command/control_cmd"',
                    "tiny_lidar_net_controller",
                )
            ) and all(
                token in tiny_source
                for token in (
                    "LaserScan, \"/scan\"",
                    "AckermannControlCommand, \"/control/command/control_cmd\"",
                )
            ),
            "pilot_net": all(
                token in pilot_launch
                for token in (
                    'name="control_cmd_topic" value="/control/command/control_cmd"',
                    "pilot_net_controller",
                )
            ) and all(
                token in pilot_source
                for token in (
                    "Image, \"/image_raw\"",
                    "AckermannControlCommand, \"/control/command/control_cmd\"",
                )
            ),
            "joycon": all(
                token in joy_launch
                for token in (
                    'pkg="teleop_manager" exec="teleop_manager_node"',
                    'pkg="joycon_contract_guard" exec="joycon_contract_guard_node"',
                    'from="/ackermann_cmd" to="$(var self_control_topic)"',
                    'from="/control/command/control_cmd" to="$(var vehicle_control_topic)"',
                    'from="/rosbag2_recorder/trigger" to="$(var trigger_topic)"',
                    'from="/awsim/cmd" to="/participant/joycon/boost_request_raw"',
                    'from="/admin/awsim/reset" to="/participant/joycon/reset_ignored"',
                )
            ) and all(
                token in joy_source
                for token in (
                    "create_subscription<sensor_msgs::msg::Joy>",
                    "create_publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>",
                    '"/control/command/control_cmd"',
                )
            ),
        }
        self.add(
            "C-CONTROL-METHODS",
            default_is_mpc
            and methods == expected
            and routes_exist
            and routes_included
            and all(route_contracts.values()),
            "all five control methods exist, consume their contracted input, and converge on control_cmd",
            (
                f"{reference_launch}: default_mpc={default_is_mpc}",
                f"{reference_launch}: methods={','.join(sorted(methods))}",
                f"expected={','.join(sorted(expected))}",
                f"routes_exist={routes_exist}, routes_included={routes_included}",
                *(f"{method}: static_route={route_contracts[method]}" for method in sorted(expected)),
            ),
        )

    def check_participant_io(self) -> None:
        controller = (
            "aichallenge/workspace/src/aichallenge_submit/"
            "multi_purpose_mpc_ros/src/mpc_controller_cpp.cpp"
        )
        reference_launch = (
            "aichallenge/workspace/src/aichallenge_submit/"
            "aichallenge_submit_launch/launch/reference.launch.xml"
        )
        poser = (
            "aichallenge/workspace/src/aichallenge_submit/"
            "imu_gnss_poser/src/imu_gnss_poser_node.cpp"
        )
        gnss_poser = (
            "aichallenge/workspace/src/aichallenge_submit/"
            "racing_kart_gnss_poser/src/gnss_poser_core.cpp"
        )
        trajectory_generator = (
            "aichallenge/workspace/src/aichallenge_submit/"
            "simple_trajectory_generator/src/simple_trajectory_generator.cpp"
        )
        controller_text = self.read(controller)
        launch_text = self.read(reference_launch)
        poser_text = self.read(poser)
        gnss_text = self.read(gnss_poser)
        trajectory_text = self.read(trajectory_generator)
        required_controller = (
            ("autoware_auto_control_msgs/msg/ackermann_control_command.hpp",),
            ('"/control/command/control_cmd"',),
            ('"/localization/kinematic_state"',),
            (
                '"/planning/scenario_planning/trajectory"',
                '"planning/scenario_planning/trajectory"',
            ),
        )
        required_launch = (
            'output_odom_name" value="kinematic_state"',
            'name="simple_trajectory_generator"',
        )
        missing = [
            " or ".join(alternatives)
            for alternatives in required_controller
            if not any(item in controller_text for item in alternatives)
        ]
        missing.extend(item for item in required_launch if item not in launch_text)
        sensor_and_pipeline_contracts = {
            "imu": all(
                item in poser_text
                for item in (
                    "create_subscription<sensor_msgs::msg::Imu>",
                    '"/sensing/imu/imu_raw"',
                )
            ),
            "gnss": all(
                item in launch_text
                for item in (
                    'namespace="gnss"',
                    'name="input_topic_fix" value="nav_sat_fix"',
                )
            ) and "create_subscription<sensor_msgs::msg::NavSatFix>" in gnss_text,
            "velocity": (
                'name="input_vehicle_velocity_topic" value="/vehicle/status/velocity_status"'
                in launch_text
            ),
            "clock": 'name="use_sim_time"' in launch_text,
            "localization": all(
                item in (launch_text + controller_text)
                for item in (
                    'name="output_odom_name" value="kinematic_state"',
                    "nav_msgs::msg::Odometry",
                    '"/localization/kinematic_state"',
                )
            ),
            "trajectory": all(
                item in trajectory_text
                for item in (
                    "autoware_auto_planning_msgs::msg::Trajectory",
                    'create_publisher<Trajectory>("trajectory"',
                )
            ) and all(
                item in launch_text
                for item in (
                    'namespace="planning"',
                    'namespace="scenario_planning"',
                    'name="simple_trajectory_generator"',
                )
            ),
        }
        service_found = all(
            item in poser_text
            for item in (
                'declare_parameter("initial_pose_service", std::string("/set_initial_pose"))',
                "create_service<std_srvs::srv::Trigger>",
            )
        ) and 'exec="imu_gnss_poser_node"' in launch_text
        service_evidence = (
            self.line_evidence(poser, 'std::string("/set_initial_pose")'),
            self.line_evidence(poser, "create_service<std_srvs::srv::Trigger>"),
            self.line_evidence(reference_launch, 'exec="imu_gnss_poser_node"'),
        )
        self.add(
            "C-PARTICIPANT-IO",
            not missing and service_found and all(sensor_and_pipeline_contracts.values()),
            "participant sensor, localization, planning, control, and service connections are present",
            (
                *(f"missing={item}" for item in missing),
                *(
                    f"{name}: static_contract={value}"
                    for name, value in sensor_and_pipeline_contracts.items()
                ),
                self.line_evidence(controller, '"/control/command/control_cmd"'),
                self.line_evidence(controller, '"/localization/kinematic_state"'),
                self.line_evidence(controller, '"planning/scenario_planning/trajectory"'),
                *service_evidence,
            ),
        )

    def check_domain_split(self) -> None:
        evaluation_launch = (
            "aichallenge/workspace/src/aichallenge_system/"
            "aichallenge_system_launch/launch/evaluation.launch.xml"
        )
        env_example = ".env.example"
        compose = "docker-compose.yml"
        launch_text = self.read(evaluation_launch)
        env_text = self.read(env_example)
        compose_text = self.read(compose)
        domain_zero = '<set_env name="ROS_DOMAIN_ID" value="0"/>' in launch_text
        vehicle_domain = '<set_env name="ROS_DOMAIN_ID" value="$(var domain_id)"/>' in launch_text
        default_domain = bool(
            re.search(r"^ROS_DOMAIN_ID\s*=\s*1\s*$", env_text, re.MULTILINE)
        ) and "${ROS_DOMAIN_ID:-1}" in compose_text
        self.add(
            "C-DOMAIN-SPLIT",
            domain_zero and vehicle_domain and default_domain,
            "AWSIM management remains Domain 0 and vehicles remain Domain 1..N",
            (
                self.line_evidence(evaluation_launch, 'ROS_DOMAIN_ID" value="0"'),
                self.line_evidence(evaluation_launch, 'ROS_DOMAIN_ID" value="$(var domain_id)"'),
                self.line_evidence(env_example, "ROS_DOMAIN_ID=1"),
                self.line_evidence(compose, "${ROS_DOMAIN_ID:-1}"),
            ),
        )

    def check_reachable_cross_domain_bridge(self) -> None:
        reference_launch = (
            "aichallenge/workspace/src/aichallenge_submit/"
            "aichallenge_submit_launch/launch/reference.launch.xml"
        )
        bridge = (
            "aichallenge/workspace/src/aichallenge_tools/rl_train/"
            "rl_train_controller/rl_train_controller_node.py"
        )
        submit_wrapper = (
            "aichallenge/workspace/src/aichallenge_submit/"
            "aichallenge_submit_launch/launch/control/rl_train.launch.xml"
        )
        reference_text = self.read(reference_launch)
        bridge_text = self.read(bridge)
        rl_train_reachable = bool(
            re.search(r"'\$\(var control_method\)'\s*==\s*'rl_train'", reference_text)
        )
        bridge_crosses_contract = all(
            item in bridge_text
            for item in ("domain_id=0", "domain_id=1", "/admin/awsim/reset")
        )
        submit_wrapper_exists = self.path(submit_wrapper).is_file()
        self.add(
            "C-NO-REACHABLE-DOMAIN-BRIDGE",
            not ((rl_train_reachable or submit_wrapper_exists) and bridge_crosses_contract),
            "the participant package cannot reach the development-only Domain 1 -> 0 bridge",
            (
                f"{reference_launch}: rl_train_reachable={rl_train_reachable}",
                f"{submit_wrapper}: exists={submit_wrapper_exists}",
                f"{bridge}: crosses_domain_1_to_0_admin_reset={bridge_crosses_contract}",
            ),
        )

    def check_joycon_boost(self) -> None:
        source = (
            "aichallenge/workspace/src/aichallenge_submit/joycon_contract_guard/"
            "joycon_contract_guard/node.py"
        )
        core = (
            "aichallenge/workspace/src/aichallenge_submit/joycon_contract_guard/"
            "joycon_contract_guard/core.py"
        )
        launch = (
            "aichallenge/workspace/src/aichallenge_submit/"
            "aichallenge_submit_launch/launch/control/joycon.launch.xml"
        )
        source_exists = self.path(source).is_file()
        core_exists = self.path(core).is_file()
        source_text = self.read(source) if source_exists else ""
        core_text = self.read(core) if core_exists else ""
        launch_text = self.read(launch)
        official_topic = '"/awsim/status"' in source_text
        forbidden_topic = '"/admin/awsim/status"' in source_text or "'/admin/awsim/status'" in source_text
        legacy_endpoints_isolated = all(
            item in launch_text
            for item in (
                'from="/admin/awsim/status" to="/awsim/status"',
                'from="/awsim/cmd" to="/participant/joycon/boost_request_raw"',
                'from="/admin/awsim/reset" to="/participant/joycon/reset_ignored"',
                'from="/initialpose" to="/participant/joycon/initialpose_ignored"',
            )
        )
        complete_status = all(
            item in core_text
            for item in (
                "len(data) != 7",
                "values[5]",
                "values[6]",
                "all(math.isfinite",
            )
        )
        guarded_publish = all(
            item in (source_text + core_text)
            for item in (
                "_status_valid",
                "_status_timeout_sec",
                "_remaining",
                "_active",
                "_pulse_pending",
                "_remaining_before_pulse",
                "_finish_seen",
                "def try_trigger(",
                "self._remaining < 1.0",
                "self._remaining_before_pulse - 0.5",
                "age > self._status_timeout_sec",
                "if self._active",
                'state == "finish"',
                'state == "spawned"',
                'Float32MultiArray(data=[1.0])',
                'Float32MultiArray(data=[0.0])',
            )
        )
        self.add(
            "C-JOYCON-BOOST",
            official_topic
            and source_exists
            and core_exists
            and not forbidden_topic
            and legacy_endpoints_isolated
            and complete_status
            and guarded_publish,
            "submitted Joycon guard isolates upstream legacy endpoints and enforces the official Boost contract",
            (
                (
                    self.line_evidence(source, '"/awsim/status"')
                    if source_exists
                    else f"{source}: missing"
                ),
                f"{source}: forbidden_admin_status={forbidden_topic}",
                f"{launch}: legacy_endpoints_isolated={legacy_endpoints_isolated}",
                f"{core}: exists={core_exists}, complete_status_validation={complete_status}",
                f"{source}: guarded_publish={guarded_publish}",
            ),
        )

    def check_participant_admin_reset(self) -> None:
        reference_launch = (
            "aichallenge/workspace/src/aichallenge_submit/"
            "aichallenge_submit_launch/launch/reference.launch.xml"
        )
        joycon_launch = (
            "aichallenge/workspace/src/aichallenge_submit/"
            "aichallenge_submit_launch/launch/control/joycon.launch.xml"
        )
        bridge = (
            "aichallenge/workspace/src/aichallenge_tools/rl_train/"
            "rl_train_controller/rl_train_controller_node.py"
        )
        reference_text = self.read(reference_launch)
        methods = set(
            re.findall(r"'\$\(var control_method\)'\s*==\s*'([^']+)'", reference_text)
        )
        reachable_sources: list[str] = []
        if "rl_train" in methods:
            reachable_sources.append(bridge)
        hits = self.occurrences(
            tuple(self.path(relative) for relative in reachable_sources),
            "/admin/awsim/reset",
        )
        joycon_text = self.read(joycon_launch)
        joycon_reset_isolated = (
            "joycon" not in methods
            or 'from="/admin/awsim/reset" to="/participant/joycon/reset_ignored"'
            in joycon_text
        )
        self.add(
            "C-NO-PARTICIPANT-ADMIN-RESET",
            not hits and joycon_reset_isolated,
            "reachable participant controllers do not publish or bridge the Domain 0 reset endpoint",
            (
                f"{reference_launch}: reachable_methods={','.join(sorted(methods))}",
                f"{joycon_launch}: reset_isolated={joycon_reset_isolated}",
                *(hits or ("no reachable /admin/awsim/reset occurrence",)),
            ),
        )

    def check_dev_simulator_management(self) -> None:
        runner = "aichallenge/run_simulator.bash"
        harness = (
            ".steering/20260714-mpc-refactor/tools/"
            "test_run_simulator_supervisor.bash"
        )
        text = self.read(runner)
        harness_text = self.read(harness)
        manager_started = (
            "aichallenge_system_launch awsim_state_manager.launch.xml" in text
            and "ROS_DOMAIN_ID=0" in text
        )
        manager_supervised = all(
            item in text
            for item in (
                "set -m",
                'wait -f "${pid}"',
                'job_group_alive "${pid}"',
                '! kill -0 "${manager_pid}"',
                'stop_job "${simulator_pid}" TERM',
                "awsim_state_manager exited before AWSIM",
            )
        )
        edge_harness_defined = all(
            item in harness_text
            for item in (
                "manager_fast_failure",
                "manager_clean_early_exit",
                "manager_failure_kills_unresponsive_simulator",
                "manager_wrapper_sigkill_is_detected",
                "eval_defaults_to_external_manager_owner",
                'kill -0 -- "-${manager_pid}"',
            )
        )
        self.add(
            "C-DEV-AWSIM-MANAGER",
            manager_started and manager_supervised and edge_harness_defined,
            "the dev/gate runner starts and supervises awsim_state_manager on Domain 0",
            (
                self.line_evidence(
                    runner, "aichallenge_system_launch awsim_state_manager.launch.xml"
                ),
                self.line_evidence(runner, "ROS_DOMAIN_ID=0"),
                f"{runner}: manager_supervised={manager_supervised}",
                f"{harness}: edge_harness_defined={edge_harness_defined}",
            ),
        )

    def check_dev_simulator_working_directory(self) -> None:
        runner = "aichallenge/run_simulator.bash"
        text = self.read(runner)
        changes_to_log_dir = bool(
            re.search(
                r'^cd\s+"\$\{log_dir\}"(?:\s*\|\|\s*exit(?:\s+\d+)?)?\s*$',
                text,
                re.MULTILINE,
            )
        )
        self.add(
            "C-DEV-AWSIM-CWD",
            changes_to_log_dir,
            "the canonical dev/gate AWSIM process writes relative result artifacts in the run directory",
            (self.line_evidence(runner, 'cd "${log_dir}" || exit 1'),),
        )

    def check_admin_contract(self) -> None:
        params = (
            "aichallenge/workspace/src/aichallenge_system/"
            "autostart_orchestrator_py/config/awsim_state_manager.param.yaml"
        )
        manager = (
            "aichallenge/workspace/src/aichallenge_system/"
            "autostart_orchestrator_py/autostart_orchestrator_py/"
            "awsim_state_manager_node.py"
        )
        orchestrator = (
            "aichallenge/workspace/src/aichallenge_system/"
            "autostart_orchestrator_py/autostart_orchestrator_py/"
            "autostart_orchestrator_node.py"
        )
        param_text = self.read(params)
        manager_text = self.read(manager)
        orchestrator_text = self.read(orchestrator)
        make_text = self.read("Makefile")
        required = (
            "admin_state_topic: /admin/awsim/state",
            "admin_start_topic: /admin/awsim/start",
            "admin_start_trigger_state: waitstart,ready",
            "admin_start_enabled: true",
            "admin_start_once: true",
        )
        params_ok = all(item in param_text for item in required)
        manager_owns_start = "admin_start_topic" in manager_text
        orchestrator_forbidden = "/admin/awsim/start" in orchestrator_text
        manager_forbidden = '"/awsim/state"' in manager_text or "'/awsim/state'" in manager_text
        known_states_ok = all(
            f'"{state}"' in manager_text
            for state in (
                "selectmode", "playstart", "ready", "waitstart", "start",
                "lapcomplete", "finish", "finishall", "terminate",
            )
        )
        finish_states_ok = all(
            f'"{state}"' in manager_text
            for state in ("finish", "finishall", "finishedall", "terminate", "terminated")
        )
        manager_types_and_directions = all(
            item in manager_text
            for item in (
                "create_subscription(String, self._admin_state_topic",
                "create_publisher(Bool, self._admin_start_topic",
            )
        )
        manual_operations_ok = all(
            item in make_text
            for item in (
                "/admin/awsim/start std_msgs/msg/Bool",
                "/admin/awsim/reset std_msgs/msg/Empty",
                "ROS_DOMAIN_ID=0",
            )
        )
        self.add(
            "C-ADMIN-OWNERSHIP",
            params_ok
            and manager_owns_start
            and known_states_ok
            and finish_states_ok
            and manager_types_and_directions
            and manual_operations_ok
            and not orchestrator_forbidden
            and not manager_forbidden,
            "admin endpoints, types, states, one-shot start, and ownership match the Domain 0 contract",
            (
                *(self.line_evidence(params, item) for item in required),
                f"{orchestrator}: forbidden_admin_start={orchestrator_forbidden}",
                f"{manager}: forbidden_vehicle_state={manager_forbidden}",
                f"{manager}: known_admin_states={known_states_ok}",
                f"{manager}: finish_states={finish_states_ok}",
                f"{manager}: typed_directions={manager_types_and_directions}",
                f"Makefile: typed_domain0_operations={manual_operations_ok}",
            ),
        )

    def production_files(self, relative_root: str) -> Sequence[Path]:
        suffixes = {".cpp", ".hpp", ".py", ".xml", ".yaml", ".yml", ".bash"}
        files: list[Path] = []
        for path in self.path(relative_root).rglob("*"):
            if not path.is_file() or path.suffix not in suffixes:
                continue
            if any(part in {"test", "tests", "docs", "build", "install", "log"} for part in path.parts):
                continue
            files.append(path)
        return tuple(files)

    def occurrences(self, paths: Sequence[Path], needle: str) -> list[str]:
        hits: list[str] = []
        for path in paths:
            try:
                lines = path.read_text(encoding="utf-8").splitlines()
            except (OSError, UnicodeError):
                continue
            for line_number, line in enumerate(lines, start=1):
                if needle in line:
                    hits.append(f"{path.relative_to(self.root)}:{line_number}: {line.strip()}")
        return hits

    def check_boost_and_gear(self) -> None:
        submit_root = "aichallenge/workspace/src/aichallenge_submit"
        files = self.production_files(submit_root)
        boost_cmd = self.occurrences(files, '"/awsim/cmd"')
        boost_status = self.occurrences(files, '"/awsim/status"')
        gear_cmd = self.occurrences(files, '"/control/command/gear_cmd"')
        gear_status = self.occurrences(files, '"/vehicle/status/gear_status"')
        forbidden_needles = (
            "/awsim/boost_cmd",
            "/admin/awsim/reset",
            "domain_bridge",
            "teleport",
            "respawn",
        )
        forbidden: list[str] = []
        for needle in forbidden_needles:
            hits = self.occurrences(files, needle)
            if needle == "/admin/awsim/reset":
                hits = [
                    hit
                    for hit in hits
                    if not (
                        'from="/admin/awsim/reset"'
                        in hit
                        and 'to="/participant/joycon/reset_ignored"' in hit
                    )
                ]
            forbidden.extend(hits)
        controller = (
            "aichallenge/workspace/src/aichallenge_submit/"
            "multi_purpose_mpc_ros/src/mpc_controller_cpp.cpp"
        )
        controller_text = self.read(controller)
        float_array_type = "std_msgs::msg::Float32MultiArray" in controller_text
        v2x_contract = all(
            item in controller_text
            for item in (
                "v2x_msgs::msg::V2XVehiclePositionArray",
                '"/v2x/vehicle_positions"',
                "create_subscription<V2XVehiclePositionArray>",
            )
        )
        orchestrator = (
            "aichallenge/workspace/src/aichallenge_system/"
            "autostart_orchestrator_py/autostart_orchestrator_py/"
            "autostart_orchestrator_node.py"
        )
        orchestrator_params = (
            "aichallenge/workspace/src/aichallenge_system/"
            "autostart_orchestrator_py/config/autostart_orchestrator.param.yaml"
        )
        orchestrator_text = self.read(orchestrator)
        orchestrator_param_text = self.read(orchestrator_params)
        manager_text = self.read(
            "aichallenge/workspace/src/aichallenge_system/"
            "autostart_orchestrator_py/autostart_orchestrator_py/"
            "awsim_state_manager_node.py"
        )
        vehicle_state_contract = (
            "create_subscription(String, vehicle_state_topic" in orchestrator_text
            and "vehicle_state_topic: /awsim/state" in orchestrator_param_text
            and all(
                f'"{state}"' in manager_text
                for state in ("spawned", "grounded", "ready", "start", "finish")
            )
        )
        control_mode_contract = all(
            item in orchestrator_text
            for item in (
                "self.create_publisher(",
                "Bool, str(self.get_parameter(\"control_mode_request_topic\").value), 1",
            )
        ) and (
            "control_mode_request_topic: /awsim/control_mode_request_topic"
            in orchestrator_param_text
        )
        mpc_does_not_own_control_mode = (
            "/awsim/control_mode_request_topic" not in controller_text
        )
        self.add(
            "C-BOOST-GEAR",
            bool(boost_cmd and boost_status and gear_cmd and gear_status)
            and float_array_type
            and v2x_contract
            and vehicle_state_contract
            and control_mode_contract
            and mpc_does_not_own_control_mode
            and not forbidden,
            "vehicle-domain V2X/state/control-mode/Boost/gear endpoints and ownership match the contract",
            (
                *boost_cmd[:2],
                *boost_status[:2],
                *gear_cmd[:2],
                *gear_status[:2],
                f"{controller}: Float32MultiArray={float_array_type}",
                f"{controller}: V2XVehiclePositionArray={v2x_contract}",
                f"{orchestrator}: vehicle_state={vehicle_state_contract}",
                f"{orchestrator}: control_mode={control_mode_contract}",
                f"{controller}: owns_official_control_mode={not mpc_does_not_own_control_mode}",
                *(f"forbidden={item}" for item in forbidden),
            ),
        )

    def check_submission(self) -> None:
        script = "create_submit_file.bash"
        text = self.read(script)
        path_contract = all(
            item in text
            for item in (
                "tar zcvf submit/aichallenge_submit.tar.gz",
                "-C ./aichallenge/workspace/src aichallenge_submit",
            )
        )
        generated_excludes = all(
            item in text
            for item in (
                "--exclude='*/__pycache__'",
                "--exclude='*/.pytest_cache'",
                "--exclude='*.pyc'",
                "--exclude='*.pyo'",
            )
        )
        self.add(
            "C-SUBMISSION-TAR",
            path_contract and generated_excludes,
            "submission archive stays in the build context with one top-level directory and no Python caches",
            (
                self.line_evidence(script, "tar zcvf"),
                f"path_contract={path_contract}",
                f"generated_excludes={generated_excludes}",
            ),
        )

    def check_result_schema(self) -> None:
        summary_path = "aichallenge/result-summary.json"
        details_path = (
            ".steering/20260714-mpc-refactor/fixtures/"
            "d1-result-details-v3.json"
        )
        summary = json.loads(self.read(summary_path))
        details = json.loads(self.read(details_path))
        summary_required = {
            "schema_version", "session", "vehicles", "laps", "min_time",
            "total_lap_time", "num_laps",
        }
        summary_vehicle_required = {
            "vehicle_number", "vehicle_name", "final_position", "finished", "lap_count",
            "laps", "min_lap_time", "max_lap_time", "avg_lap_time", "total_lap_time",
        }
        details_required = {
            "schema_version", "vehicle_name", "vehicle_number", "finished", "lap_count",
            "required_laps", "session_timeout", "min_lap_time", "avg_lap_time",
            "total_lap_time", "laps", "penalty_count", "penalty_total_seconds",
            "penalty_events", "penalty_by_kind",
        }

        def is_int(value: object) -> bool:
            return type(value) is int

        def is_float(value: object) -> bool:
            return type(value) is float and math.isfinite(value)

        def is_float_list(value: object) -> bool:
            return (
                isinstance(value, list)
                and all(is_float(item) and item >= 0.0 for item in value)
            )

        def close(left: object, right: float) -> bool:
            # AWSIM aggregates float32-derived lap values.  Preserve their
            # semantics while allowing the expected float32 accumulation ULP.
            return is_float(left) and math.isclose(
                left, right, rel_tol=1e-6, abs_tol=1e-5
            )

        def lap_stats_match(
            record: dict[str, object], *, include_max: bool
        ) -> bool:
            laps = record.get("laps")
            if not is_float_list(laps):
                return False
            expected_min = min(laps) if laps else 0.0
            expected_max = max(laps) if laps else 0.0
            expected_total = sum(laps)
            expected_avg = expected_total / len(laps) if laps else 0.0
            checks = (
                close(record.get("min_lap_time"), expected_min),
                close(record.get("avg_lap_time"), expected_avg),
                close(record.get("total_lap_time"), expected_total),
            )
            return all(checks) and (
                not include_max
                or close(record.get("max_lap_time"), expected_max)
            )

        session = summary.get("session")
        vehicles = summary.get("vehicles")
        vehicle_list = vehicles if isinstance(vehicles, list) else []
        total_vehicles = session.get("total_vehicles") if isinstance(session, dict) else None
        vehicle_records_valid = (
            isinstance(vehicles, list)
            and bool(vehicles)
            and is_int(total_vehicles)
            and total_vehicles == len(vehicles)
            and all(
                isinstance(vehicle, dict)
                and summary_vehicle_required <= vehicle.keys()
                and is_int(vehicle.get("vehicle_number"))
                and 1 <= vehicle["vehicle_number"] <= total_vehicles
                and vehicle.get("vehicle_name") == f"GoKart{vehicle['vehicle_number']}"
                and is_int(vehicle.get("final_position"))
                and 1 <= vehicle["final_position"] <= total_vehicles
                and type(vehicle.get("finished")) is bool
                and is_int(vehicle.get("lap_count"))
                and vehicle["lap_count"] >= 0
                and is_float_list(vehicle.get("laps"))
                and vehicle["lap_count"] == len(vehicle["laps"])
                and lap_stats_match(vehicle, include_max=True)
                for vehicle in vehicles
            )
        )
        vehicle_numbers = (
            [vehicle["vehicle_number"] for vehicle in vehicles]
            if vehicle_records_valid
            else []
        )
        positions = (
            [vehicle["final_position"] for vehicle in vehicles]
            if vehicle_records_valid
            else []
        )
        gokart1 = next(
            (
                vehicle
                for vehicle in vehicle_list
                if isinstance(vehicle, dict) and vehicle.get("vehicle_number") == 1
            ),
            None,
        )
        legacy_fields_match = (
            isinstance(gokart1, dict)
            and summary.get("laps") == gokart1.get("laps")
            and summary.get("min_time") == gokart1.get("min_lap_time")
            and summary.get("total_lap_time") == gokart1.get("total_lap_time")
            and summary.get("num_laps") == gokart1.get("lap_count")
        )
        summary_ok = (
            summary.get("schema_version") == "v2"
            and summary_required <= summary.keys()
            and isinstance(session, dict)
            and is_int(session.get("required_laps"))
            and session["required_laps"] >= 0
            and is_float(session.get("timeout"))
            and session["timeout"] >= 0.0
            and vehicle_records_valid
            and sorted(vehicle_numbers) == list(range(1, total_vehicles + 1))
            and positions == sorted(positions)
            and positions[0] == 1
            and is_float_list(summary.get("laps"))
            and is_float(summary.get("min_time"))
            and is_float(summary.get("total_lap_time"))
            and is_int(summary.get("num_laps"))
            and legacy_fields_match
        )

        events = details.get("penalty_events")
        by_kind = details.get("penalty_by_kind")
        event_kinds = ("crash", "wall", "over")
        event_shapes_ok = isinstance(events, list) and all(
            isinstance(event, dict)
            and event.get("kind") in event_kinds
            and is_int(event.get("lap"))
            and event["lap"] >= 1
            and is_float(event.get("race_time"))
            and event["race_time"] >= 0.0
            and is_float(event.get("duration"))
            and event["duration"] >= 0.0
            for event in events
        )
        by_kind_ok = (
            isinstance(events, list)
            and isinstance(by_kind, dict)
            and set(by_kind) == set(event_kinds)
            and all(
                isinstance(by_kind[kind], dict)
                and is_int(by_kind[kind].get("count"))
                and is_float(by_kind[kind].get("total_seconds"))
                and by_kind[kind]["total_seconds"] >= 0.0
                and by_kind[kind]["count"]
                == sum(event.get("kind") == kind for event in events)
                and math.isclose(
                    by_kind[kind]["total_seconds"],
                    sum(
                        event["duration"]
                        for event in events
                        if event.get("kind") == kind
                    ),
                    rel_tol=1e-9,
                    abs_tol=1e-9,
                )
                for kind in event_kinds
            )
        )
        details_filename_match = re.fullmatch(
            r"d([1-9][0-9]*)-result-details-v3\.json", Path(details_path).name
        )
        details_filename_vehicle_number = (
            int(details_filename_match.group(1)) if details_filename_match else None
        )
        details_ok = (
            details.get("schema_version") == "v3"
            and details_required <= details.keys()
            and is_int(details.get("vehicle_number"))
            and details["vehicle_number"] >= 1
            and details["vehicle_number"] == details_filename_vehicle_number
            and details.get("vehicle_name") == f"GoKart{details['vehicle_number']}"
            and type(details.get("finished")) is bool
            and is_int(details.get("lap_count"))
            and details["lap_count"] >= 0
            and is_int(details.get("required_laps"))
            and details["required_laps"] >= 0
            and all(
                is_float(details.get(key))
                for key in (
                    "session_timeout", "min_lap_time", "avg_lap_time",
                    "total_lap_time", "penalty_total_seconds",
                )
            )
            and details["session_timeout"] >= 0.0
            and is_float_list(details.get("laps"))
            and details.get("lap_count") == len(details.get("laps", []))
            and lap_stats_match(details, include_max=False)
            and is_int(details.get("penalty_count"))
            and details["penalty_count"] >= 0
            and event_shapes_ok
            and details.get("penalty_count") == len(events)
            and by_kind_ok
            and details["penalty_total_seconds"] >= 0.0
            and details.get("penalty_total_seconds")
            <= sum(by_kind[kind]["total_seconds"] for kind in event_kinds) + 1e-9
        )
        self.add(
            "C-RESULT-SCHEMA",
            summary_ok and details_ok,
            "repository result fixtures preserve v2/v3 scoring keys, identities, statistics, ordering, and penalties",
            (
                f"{summary_path}: schema={summary.get('schema_version')}, vehicles={len(vehicle_list)}, valid={summary_ok}",
                f"{summary_path}: vehicle_numbers={vehicle_numbers}, positions={positions}, legacy_gokart1={legacy_fields_match}",
                f"{details_path}: schema={details.get('schema_version')}, file_vehicle={details_filename_vehicle_number}, valid={details_ok}",
            ),
        )

    def check_output_and_uid(self) -> None:
        orchestrator = (
            "aichallenge/workspace/src/aichallenge_system/"
            "autostart_orchestrator_py/autostart_orchestrator_py/"
            "autostart_orchestrator_node.py"
        )
        makefile = "Makefile"
        compose = "docker-compose.yml"
        orchestrator_text = self.read(orchestrator)
        make_text = self.read(makefile)
        compose_text = self.read(compose)
        latest_ok = all(
            item in orchestrator_text
            for item in (
                'output_root / "latest"',
                "latest_path.mkdir(parents=True, exist_ok=True)",
                '("result-details.json"',
                '("result-summary.json"',
                '("capture.mp4"',
                '("rosbag2_autoware.mcap"',
                '("motion_analytics.html"',
                'latest_vehicle_dir / "autoware.log"',
            )
        )
        uid_ok = all(
            item in make_text for item in ("HOST_UID ?=", "HOST_GID ?=", "export HOST_UID HOST_GID")
        ) and 'user: "${HOST_UID:-1000}:${HOST_GID:-1000}"' in compose_text
        self.add(
            "C-OUTPUT-OWNERSHIP",
            latest_ok and uid_ok,
            "output/latest layout and host ownership wiring are preserved",
            (
                self.line_evidence(orchestrator, 'output_root / "latest"'),
                self.line_evidence(orchestrator, '("result-details.json"'),
                self.line_evidence(makefile, "export HOST_UID HOST_GID"),
                self.line_evidence(compose, 'user: "${HOST_UID:-1000}:${HOST_GID:-1000}"'),
            ),
        )

    def add_runtime_boundaries(self) -> None:
        dockerfile_text = self.read("Dockerfile")
        upstream_ref_match = re.search(
            r"^ARG AICHALLENGE_UPSTREAM_REF=([0-9a-f]{40})$",
            dockerfile_text,
            re.MULTILINE,
        )
        upstream_ref = upstream_ref_match.group(1) if upstream_ref_match else "UNPINNED"
        upstream_identity_written = "/aichallenge/.upstream-commit" in dockerfile_text
        manifest_relative = (
            ".steering/20260714-mpc-refactor/sealed-eval-manifest.json"
        )
        manifest_path = self.path(manifest_relative)
        if not manifest_path.is_file():
            self.pending(
                "R-SEALED-EVAL-IDENTITY",
                "sealed eval must be built and matched to its pinned upstream plus submitted overlay",
                (
                    f"Dockerfile: upstream_ref={upstream_ref}, identity_file={upstream_identity_written}",
                    f"{manifest_relative}: missing",
                    "build the eval image, verify .upstream-commit/image identity, and extract a "
                    "host-side manifest of the contract files available under /aichallenge",
                    "local system/tools PASS results do not prove the sealed eval implementation",
                ),
            )
        else:
            manifest = json.loads(self.read(manifest_relative))
            source_paths = {
                "joycon_guard_core": (
                    "aichallenge/workspace/src/aichallenge_submit/"
                    "joycon_contract_guard/joycon_contract_guard/core.py"
                ),
                "joycon_guard_node": (
                    "aichallenge/workspace/src/aichallenge_submit/"
                    "joycon_contract_guard/joycon_contract_guard/node.py"
                ),
                "joycon_launch": (
                    "aichallenge/workspace/src/aichallenge_submit/"
                    "aichallenge_submit_launch/launch/control/joycon.launch.xml"
                ),
                "reference_launch": (
                    "aichallenge/workspace/src/aichallenge_submit/"
                    "aichallenge_submit_launch/launch/reference.launch.xml"
                ),
            }
            expected_hashes = {
                name: self.sha256(relative) for name, relative in source_paths.items()
            }
            recorded_hashes = manifest.get("source_sha256", {})
            image_hashes = manifest.get("image_source_sha256", {})
            source_matches = recorded_hashes == expected_hashes
            image_matches = image_hashes == expected_hashes
            submit_tar = "submit/aichallenge_submit.tar.gz"
            recorded_tar_hash = manifest.get("inputs", {}).get("submit_tar_sha256")
            tar_exists = self.path(submit_tar).is_file()
            tar_matches = (
                not tar_exists or self.sha256(submit_tar) == recorded_tar_hash
            )
            image_id = manifest.get("image", {}).get("id", "")
            checks = manifest.get("checks", {})
            required_checks = (
                "build_succeeded",
                "upstream_commit_verified",
                "submit_overlay_hashes_match",
                "guard_tests_passed",
                "launch_show_args_passed",
                "package_prefix_verified",
            )
            checks_pass = all(checks.get(name) is True for name in required_checks)
            identity_matches = (
                manifest.get("schema_version") == "phase0-sealed-eval-manifest-v1"
                and manifest.get("inputs", {}).get("upstream_ref") == upstream_ref
                and manifest.get("inside_image", {}).get("upstream_commit") == upstream_ref
                and upstream_identity_written
                and bool(re.fullmatch(r"sha256:[0-9a-f]{64}", image_id))
            )
            self.add(
                "R-SEALED-EVAL-IDENTITY",
                identity_matches
                and source_matches
                and image_matches
                and tar_matches
                and checks_pass,
                "sealed eval image identity matches the pinned upstream and submitted source overlay",
                (
                    f"{manifest_relative}: image_id={image_id}",
                    f"upstream_ref={upstream_ref}, identity_matches={identity_matches}",
                    f"source_manifest_matches={source_matches}, image_overlay_matches={image_matches}",
                    f"submit_tar_present={tar_exists}, submit_tar_matches={tar_matches}",
                    f"sealed_checks_pass={checks_pass}",
                    "base image, apt/pip/rosdep inputs remain separately mutable; image ID is the "
                    "frozen identity for this build rather than a bit-reproducibility claim",
                ),
            )
        self.pending(
            "R-DDS-GRAPH",
            "Domain 0..4 endpoint/type/direction/QoS/owner require a live canonical graph",
            ("run make dev/dev4 and capture ros2 topic info -v / ros2 node info",),
        )
        self.pending(
            "R-SOLE-PUBLISHER",
            "/control/command/control_cmd sole publisher requires the selected live control_method",
            ("run ros2 topic info -v /control/command/control_cmd",),
        )
        self.pending(
            "R-TOPIC-RATE",
            "control, odometry, trajectory rates require runtime observation",
            ("run ros2 topic hz for the three contracted topics",),
        )
        self.pending(
            "R-ARTIFACTS",
            "generated result JSON, output/latest links, and UID/GID require a completed run",
            ("inspect output/latest after canonical dev/eval",),
        )

    def run(self) -> list[Check]:
        self.check_launch_entry()
        self.check_control_methods()
        self.check_participant_io()
        self.check_domain_split()
        self.check_reachable_cross_domain_bridge()
        self.check_admin_contract()
        self.check_boost_and_gear()
        self.check_joycon_boost()
        self.check_participant_admin_reset()
        self.check_dev_simulator_management()
        self.check_dev_simulator_working_directory()
        self.check_submission()
        self.check_result_schema()
        self.check_output_and_uid()
        self.add_runtime_boundaries()
        return self.checks


def find_repo_root(start: Path) -> Path:
    current = start.resolve()
    for candidate in (current, *current.parents):
        if (candidate / ".git").exists() and (candidate / "AGENTS.md").is_file():
            return candidate
    raise RuntimeError("repository root not found; pass --repo-root")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-root", type=Path)
    parser.add_argument("--json", action="store_true", help="emit machine-readable JSON")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        root = args.repo_root.resolve() if args.repo_root else find_repo_root(Path.cwd())
        checks = Oracle(root).run()
    except (RuntimeError, json.JSONDecodeError) as error:
        print(f"phase0 contract oracle error: {error}", file=sys.stderr)
        return 2

    counts = {status: sum(check.status == status for check in checks) for status in (PASS, RED, NEEDS_RUNTIME)}
    if args.json:
        print(
            json.dumps(
                {
                    "schema_version": "phase0-contract-oracle-v1",
                    "repo_root": str(root),
                    "summary": counts,
                    "checks": [asdict(check) for check in checks],
                },
                ensure_ascii=False,
                indent=2,
            )
        )
    else:
        for check in checks:
            print(f"[{check.status}] {check.check_id}: {check.summary}")
            for evidence in check.evidence:
                print(f"  - {evidence}")
        print(
            f"summary: PASS={counts[PASS]} RED={counts[RED]} "
            f"NEEDS_RUNTIME={counts[NEEDS_RUNTIME]}"
        )
    return 1 if counts[RED] else 0


if __name__ == "__main__":
    raise SystemExit(main())
