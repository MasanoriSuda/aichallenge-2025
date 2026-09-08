"""Bounded diagnostic monitor; stop the scoped Compose run on tactical failure."""
import json
from pathlib import Path
import re
import subprocess
import sys
import time

root = Path(sys.argv[1])
deadline = time.monotonic() + 240
termination = "four-minute diagnostic limit"
previous = None
while time.monotonic() < deadline:
    summary = {}
    failed = False
    for domain in ("d1", "d2"):
        p = root / domain / "autoware.log"
        lines = p.read_text(errors="replace").splitlines() if p.exists() else []
        phases = [re.sub(r"\x1b\[[0-9;]*m", "", s) for s in lines
                  if "OvertakeLine:" in s and " -> " in s]
        adoptions = [s for s in lines if "Published stateless sibling Bundle adopted:" in s]
        stop = [s for s in lines if "canonical-certified-terminal-stop" in s]
        summary[domain] = {
            "lines": len(lines), "phases": phases[-4:],
            "sibling_adoptions": len(adoptions), "certified_stop_traces": len(stop),
            "stop_executed_traces": sum("executed-retained" in s for s in stop),
        }
        if any(" -> Recovery," in s for s in phases):
            failed = True
            termination = domain + " entered Recovery during tactical diagnostic"
    encoded = json.dumps(summary, ensure_ascii=False)
    if encoded != previous:
        print(encoded, flush=True)
        previous = encoded
    if failed:
        time.sleep(2)
        break
    time.sleep(20)
(root / "monitor-summary.json").write_text(
    json.dumps({"termination": termination, "last_observation": summary}, indent=2) + "\n")
print(termination, flush=True)
with (root / "down.log").open("w") as log:
    result = subprocess.run(["make", "down"], stdout=log, stderr=subprocess.STDOUT)
print("make down exit=" + str(result.returncode), flush=True)
sys.exit(result.returncode)
