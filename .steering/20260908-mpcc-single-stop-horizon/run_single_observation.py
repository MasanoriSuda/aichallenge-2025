"""One bounded simulation of the diagnostic-only Stop reference comparison."""

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
            "output/20260908-mpcc-normal-path-stop-observation")
assert re.fullmatch(r"output/[a-z0-9-]+", str(root))
root.mkdir(parents=True, exist_ok=False)
container_root = "/" + str(root)
name = sys.argv[2] if len(sys.argv) > 2 else "mpcc-stop-profile-20260908"
assert re.fullmatch(r"[a-z0-9-]+", name)
package = "aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros"
patch = subprocess.check_output(["git", "diff", "--", package])
(root / "mpcc-source.patch").write_bytes(patch)
manifest = json.loads(Path(".steering/20260908-mpcc-single-six-lap/manifest.json").read_text())
manifest.update(run_id=root.name, kind="bounded-single-current-world-stop-counterfactual",
                working_tree_patch_sha256=hashlib.sha256(patch).hexdigest(),
                termination="first accepted moving Cruise counterfactual, six-lap finish, or720s; no observation supplies authority")
(root / "binaries").mkdir()
for binary in manifest["binaries"]:
    source = Path(binary["local_real_path"])
    destination = root / "binaries" / source.name
    shutil.copy2(source, destination)
    binary["sha256"] = hashlib.sha256(destination.read_bytes()).hexdigest()
    binary["preserved_path"] = str(destination)
manifest["build_log"] = "/tmp/mpcc-20260908-stop-horizon-observation-build2.log"
manifest["tests"] = "output/20260908-mpcc-single-six-lap/horizon-package-test-results.log"
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
Path(f".steering/20260908-mpcc-single-stop-horizon/{root.name}-manifest.json").write_text(
    json.dumps(manifest, indent=2) + "\n")
env = os.environ.copy()
env.update(LOG_DIR=container_root, ROS_DOMAIN_ID="1", AIC_VEHICLE_COUNT="1", CMD=command)
subprocess.run(["docker", "compose", "run", "-d", "--rm", "--name", name,
                "--no-deps", "autoware-command"], env=env, check=True)
deadline = time.monotonic() + 720
termination = "720s observation deadline"
summary = {}
try:
    while time.monotonic() < deadline:
        log = root / "d1/autoware.log"
        lines = log.read_text(errors="replace").splitlines() if log.exists() else []
        comparisons = [line for line in lines if "Normal-path Stop counterfactual:" in line]
        summary = {"line_count": len(lines), "counterfactuals": len(comparisons),
                   "latest": comparisons[-1:]}
        accepted = [line for line in comparisons if "intent=cruise" in line
                    and "accepted=1" in line
                    and (m := re.search(r"speed=([\d.]+)", line)) and float(m[1]) > 3.0]
        result_path = root / "d1/result-summary.json"
        result = None
        if result_path.exists():
            try:
                result = json.loads(result_path.read_text())
            except (json.JSONDecodeError, OSError):
                pass
        print(json.dumps(summary), flush=True)
        if accepted:
            summary["accepted_moving_cruise"] = accepted
            termination = "accepted current-world moving Cruise counterfactual observed"
            time.sleep(2)
            break
        if result and result.get("vehicles") and all(v.get("finished") for v in result["vehicles"]):
            termination = "six laps finished before a positive moving Cruise counterfactual"
            summary["result"] = result
            time.sleep(10)
            break
        time.sleep(10)
finally:
    (root / "monitor-summary.json").write_text(
        json.dumps({"termination": termination, "last_observation": summary}, indent=2) + "\n")
    print(termination, flush=True)
    subprocess.run(["docker", "stop", "-t", "30", name], check=True)
