"""Independent diagnostic transcription of the canonical midpoint equations.

The C++ physical oracle must verify the emitted controls before these numbers
are used as causal evidence. This script never supplies production commands.
"""

import json
import math
from pathlib import Path
import sys

import numpy as np
import yaml

snapshot = Path(sys.argv[1])
destination = Path(sys.argv[2])
recorded = yaml.safe_load(snapshot.read_text())
source = recorded["source"]["semantic_request"]
assembly = recorded["assembly_request"]
state = np.array(assembly["initial_state"], dtype=float)
states = [state.copy()]
controls = np.array(assembly["input_reference"], dtype=float).reshape(-1, 3)
for index, control in enumerate(controls):
    duration = source["inputs"][index]["stage_dt_sec"]
    curvature = source["inputs"][index]["path_curvature_radpm"]
    substeps = max(1, math.ceil(duration / 0.01))
    step = duration / substeps
    acceleration, rate, virtual_speed = control
    for _ in range(substeps):
        steering, response = state[5:7]
        tau = source["yaw_response_time_constant_sec"]

        def response_at(time):
            decay = math.exp(-time / tau)
            return steering + (response - steering) * decay + rate * (time - tau * (1 - decay))

        velocity_mid = state[3] + 0.5 * acceleration * step
        heading_rate = source["yaw_response_gain"] * velocity_mid * math.tan(response_at(step / 2)) / source["wheelbase_m"] - curvature * virtual_speed
        heading_mid = state[2] + 0.5 * heading_rate * step
        lateral_rate = velocity_mid * math.sin(heading_mid)
        denominator = 1 - curvature * (state[0] + 0.5 * lateral_rate * step)
        assert denominator >= source["minimum_frenet_denominator"]
        progress_rate = velocity_mid * math.cos(heading_mid) / denominator
        response_next = response_at(step)
        state += np.array([lateral_rate, progress_rate - virtual_speed, heading_rate,
                           acceleration, virtual_speed, rate, 0]) * step
        state[6] = response_next
    states.append(state.copy())

primal = np.concatenate([np.array(states).flatten(), controls.flatten()])
np.savetxt(destination, primal, fmt="%.17g")
qp = recorded["exact_qp"]
values = np.zeros(qp["constraints"]["rows"])
for row, column, value in qp["constraints"]["triplets"]:
    values[row] += value * primal[column]
lower, upper = np.array(qp["lower_bound"]), np.array(qp["upper_bound"])
violations = np.maximum(0, np.maximum(lower - values, values - upper))
worst = np.argsort(violations)[-12:][::-1]
report = {"source": str(snapshot), "primal": str(destination),
          "worst_affine_rows": [{"row": int(row), "value": float(values[row]),
                                  "lower": float(lower[row]), "upper": float(upper[row]),
                                  "violation": float(violations[row])} for row in worst],
          "nonlinear_states": np.array(states).tolist()}
destination.with_suffix(".json").write_text(json.dumps(report, indent=2) + "\n")
print(json.dumps(report["worst_affine_rows"][:5], indent=2))
