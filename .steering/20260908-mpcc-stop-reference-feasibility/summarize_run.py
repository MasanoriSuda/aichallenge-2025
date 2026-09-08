"""Summarize evidence without interpreting throttled logs as complete tick counts."""
import collections
import json
from pathlib import Path
import re
import sys

root = Path(sys.argv[1])
lines = re.sub(r"\x1b\[[0-9;]*m", "", (root / "d1/autoware.log").read_text(errors="replace")).splitlines()
decisions = []
callbacks = []
references = collections.Counter()
vehicle_state = None
state_transitions = []
for number, line in enumerate(lines, 1):
    state = re.search(r"AWSIM vehicle state observed: state=(\S+)", line)
    if state:
        vehicle_state = state.group(1)
        state_transitions.append(dict(line=number, text=line))
    if "Overtake control decision:" in line:
        def field(pattern):
            match = re.search(pattern, line)
            return match.group(1) if match else None
        decisions.append(dict(line=number, vehicle_state=vehicle_state, time=field(r"\[(\d+\.\d+)\]"),
                              decision=field(r"decision=(\d+)"),
                              intent=field(r"canonical_intent=([^,]+)"),
                              authority=field(r"MPCC execution contract:.*?authority=([^,]+)"),
                              actual=field(r"actual=([^m]+)m/s"),
                              solver=field(r'solver="([^"]+)"')))
    if "Control callback runtime:" in line:
        match = re.search(r"cycles=(\d+), elapsed_ms=[\d.]+/([\d.]+).*overruns=(\d+)", line)
        if match:
            callbacks.append(tuple(map(float, match.groups())))
    match = re.search(r"terminal_reference:([^/]+)/stop_reference_attempts:(\d+)", line)
    if match:
        references["/".join(match.groups())] += 1
overrides = [d for d in decisions if d["authority"] and
             ("emergency" in d["authority"] or "recovery" in d["authority"])]
result = dict(run_id=root.name, decision_log_count=len(decisions),
              vehicle_state_transitions=state_transitions,
              logged_authorities=dict(collections.Counter(d["authority"] for d in decisions)),
              supervisor_override_decisions=overrides,
              moving_override_decisions=[d for d in overrides if d["actual"] is not None and abs(float(d["actual"])) > .1],
              reference_periodic_log_counts=dict(references),
              callback=dict(reported_cycles=int(sum(c[0] for c in callbacks)),
                            max_ms=max((c[1] for c in callbacks), default=None),
                            overruns=int(sum(c[2] for c in callbacks))),
              laps=[dict(line=i, text=l) for i, l in enumerate(lines, 1) if "Lap " in l and "completed!" in l],
              recovery_transitions=[dict(line=i, text=l) for i, l in enumerate(lines, 1)
                                    if (" -> Recovery," in l or ("Stuck recovery:" in l and "action=" in l and "action=NormalControl" not in l))])
for name in ("result-summary.json", "d1-result-details.json"):
    path = root / "d1" / name
    if path.exists():
        result[name] = json.loads(path.read_text())
(root / "log-summary.json").write_text(json.dumps(result, indent=2) + "\n")
print(json.dumps({k: v for k, v in result.items() if k not in ("supervisor_override_decisions", "laps", "result-summary.json")}, indent=2))
