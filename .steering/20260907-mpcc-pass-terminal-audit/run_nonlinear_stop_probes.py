"""Run predeclared controls against the immutable physical oracle, offline."""

from pathlib import Path
import subprocess
import sys

root = Path(sys.argv[1])
snapshot = Path(sys.argv[2])
candidate_directory = Path(sys.argv[3])
policy = sys.argv[4]
for candidate in sorted(candidate_directory.glob("candidate-*.txt")):
    with candidate.with_suffix(".log").open("w") as output:
        result = subprocess.run(
            ["ros2", "run", "multi_purpose_mpc_ros", "mpcc_architecture_compare", str(snapshot),
             policy, str(candidate)],
            cwd=root, stdout=output, stderr=subprocess.STDOUT, check=False,
        )
    print(candidate.name, "exit", result.returncode, flush=True)
