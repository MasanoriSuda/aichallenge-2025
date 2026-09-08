"""Observe the bounded single-vehicle evaluation and stop its named container."""
import json
from pathlib import Path
import subprocess
import sys
import time

root = Path(sys.argv[1])
deadline = time.monotonic() + 720
reason = "720s host deadline reached"
while time.monotonic() < deadline:
    p = root / "d1" / "result-summary.json"
    result = None
    if p.exists():
        try:
            result = json.loads(p.read_text())
        except (json.JSONDecodeError, OSError):
            pass
    log = root / "d1" / "autoware.log"
    lines = log.read_text(errors="replace").splitlines() if log.exists() else []
    summary = {"lines": len(lines), "result_present": result is not None}
    if result:
        summary["vehicles"] = result.get("vehicles", [])
    controls = [s for s in lines if "Control callback runtime:" in s]
    if controls:
        summary["latest_callback"] = controls[-1]
    print(json.dumps(summary), flush=True)
    if result and result.get("vehicles") and all(v.get("finished") for v in result["vehicles"]):
        reason = "all vehicles finished six-lap evaluation"
        time.sleep(15)
        break
    time.sleep(20)
(root / "monitor-summary.json").write_text(json.dumps({"termination": reason, "last_observation": summary}, indent=2) + "\n")
print(reason, flush=True)
result = subprocess.run(["docker", "stop", "-t", "30", "mpcc-single6-20260908"])
sys.exit(result.returncode)
