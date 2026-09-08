"""Bounded wall-map diagnostic; map and peer geometry acceptance remain open."""

import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time

root = Path(sys.argv[1] if len(sys.argv) > 1 else
            "output/20260908-wall-map-single-r1")
assert re.fullmatch(r"output/[a-z0-9-]+", str(root))
root.mkdir(parents=True, exist_ok=False)
container_root = "/" + str(root)
name = sys.argv[2] if len(sys.argv) > 2 else "wall-map-20260908-r1"
assert re.fullmatch(r"[a-z0-9-]+", name)
package = "aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros"
patch = subprocess.check_output(["git", "diff", "--", package])
(root / "mpcc-source.patch").write_bytes(patch)
manifest = json.loads(Path(".steering/20260908-physical-wall-map-calibration/manifest.json").read_text())
protocol = json.loads(Path(".steering/20260908-physical-wall-map-calibration/protocol.json").read_text())
assert isinstance(protocol["binaries"], dict), "Seal campaign binary hashes before launch"
for item in manifest["files"]:
    preserved = root / "inputs" / item["path"]
    preserved.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(item["path"], preserved)
    assert hashlib.sha256(Path(item["path"]).read_bytes()).hexdigest() == item["sha256"], item["path"]
manifest.update(run_id=root.name, kind="single-six-lap-wall-map-diagnostic",
                working_tree_patch_sha256=hashlib.sha256(patch).hexdigest(),
                termination="six-lap finish or420s diagnostic deadline; geometry acceptance remains open")
(root / "binaries").mkdir()
for binary in manifest["binaries"]:
    source = Path(binary["local_real_path"])
    destination = root / "binaries" / source.name
    shutil.copy2(source, destination)
    binary["sha256"] = hashlib.sha256(destination.read_bytes()).hexdigest()
    assert binary["sha256"] == protocol["binaries"][binary["local_real_path"]], source
    binary["preserved_path"] = str(destination)
manifest["build_log"] = "/tmp/mpcc-20260908-wall-map-build.log"
manifest["tests"] = "output/20260908-physical-wall-map-calibration/package-test-results.log"
# Preserve the changed sensor producer and its launch/calibration independently.
for relative in [
    "aichallenge/workspace/build/racing_kart_gnss_poser/libgnss_poser_node.so",
    "aichallenge/workspace/src/aichallenge_submit/racing_kart_gnss_poser/src/gnss_poser_core.cpp",
    "aichallenge/workspace/src/aichallenge_submit/racing_kart_gnss_poser/launch/gnss_poser.launch.xml",
    "aichallenge/workspace/src/aichallenge_submit/aichallenge_submit_launch/launch/reference.launch.xml",
    "aichallenge/workspace/src/aichallenge_submit/imu_gnss_poser/config/imu_gnss_poser.param.yaml",
    "aichallenge/simulator/AWSIM/AWSIM_Data/StreamingAssets/Vehicle/vehicle.yaml",
]:
    path = Path(relative)
    manifest["files"].append(dict(path=relative, sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    if path.suffix == ".so":
        shutil.copy2(path, root / "binaries" / path.name)
(root / "participant-source.patch").write_bytes(subprocess.check_output(
    ["git", "diff", "--", "aichallenge/workspace/src/aichallenge_submit"]))
manifest["acceptance_scope"] = (
    "Additive local wall coverage and occupied-cell preservation. Full peer enclosure and omitted 3D geometry "
    "remain unresolved even if this diagnostic finishes six laps.")
logdir = container_root + "/d1"
command = ("source /aichallenge/workspace/install/setup.bash && "
           f"export ROS_HOME={logdir}/ros && export ROS_LOG_DIR={logdir}/ros/log && "
           f"mkdir -p {logdir}/ros/log && cd {logdir} && "
           "exec ros2 launch aichallenge_system_launch evaluation.launch.xml "
           "domain_id:=1 vehicle_count:=1 sim_mode:=eval "
           f"log_dir:={logdir} capture:=true rosbag:=true simulation:=true "
           f"use_sim_time:=true run_rviz:=true > {logdir}/autoware.log 2>&1")
manifest["command"] = command
(root / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
Path(f".steering/20260908-physical-wall-map-calibration/{root.name}-manifest.json").write_text(
    json.dumps(manifest, indent=2) + "\n")
env = os.environ.copy()
env.update(LOG_DIR=container_root, ROS_DOMAIN_ID="1", AIC_VEHICLE_COUNT="1", CMD=command)
subprocess.run(["docker", "compose", "run", "-d", "--rm", "--name", name,
                "--no-deps", "autoware-command"], env=env, check=True)
deadline = time.monotonic() + 420
termination = "420s diagnostic deadline"
summary = {}
try:
    while time.monotonic() < deadline:
        log = root / "d1/autoware.log"
        lines = log.read_text(errors="replace").splitlines() if log.exists() else []
        laps = [line for line in lines if "Lap " in line and "completed!" in line]
        callbacks = [line for line in lines if "Control callback runtime:" in line]
        summary = {"line_count": len(lines), "laps": laps[-2:],
                   "latest_callback": callbacks[-1:]}
        result_path = root / "d1/result-summary.json"
        result = None
        if result_path.exists():
            try:
                result = json.loads(result_path.read_text())
            except (json.JSONDecodeError, OSError):
                pass
        print(json.dumps(summary), flush=True)
        if result and result.get("vehicles") and all(v.get("finished") for v in result["vehicles"]):
            termination = "six laps finished"
            summary["result"] = result
            time.sleep(10)
            break
        time.sleep(10)
finally:
    (root / "monitor-summary.json").write_text(
        json.dumps({"termination": termination, "last_observation": summary}, indent=2) + "\n")
    print(termination, flush=True)
    subprocess.run(["docker", "stop", "-t", "30", name], check=True)
