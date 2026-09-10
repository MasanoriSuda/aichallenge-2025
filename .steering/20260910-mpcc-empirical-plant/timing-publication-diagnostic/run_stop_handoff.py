"""Replay the explicit published Stop handoff and an independent peer rejection."""
from pathlib import Path
import hashlib
import json
import subprocess
import sys

out = Path('/output/20260910-nine-state-intent-stop-'+sys.argv[1])
out.mkdir(exist_ok=False)
binary = Path('/output/20260910-nine-state-revalidation-tool-r30/replay')
manifest = dict(authority=False, binary=str(binary),
                binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(), runs=[])
for name, domain, decision in [('20260910-nine-state-application-dev2-r1', 'd2', 1294),
                               ('20260910-nine-state-dev2-r12', 'd2', 933)]:
    run = Path('/output')/name
    assert (run/'artifact-restoration.json').exists()
    matches = list((run/domain/'mpcc_architecture_snapshots').glob(f'{decision:012d}-*moving*/snapshot.yaml'))
    assert len(matches) == 1
    snapshot = matches[0]
    command = [str(binary), str(snapshot), str(out/(str(decision)+'.yaml'))]
    with (out/(str(decision)+'.log')).open('w') as log:
        result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, timeout=60)
    manifest['runs'].append(dict(run=str(run), snapshot=str(snapshot),
        snapshot_sha256=hashlib.sha256(snapshot.read_bytes()).hexdigest(),
        command=command, return_code=result.returncode))
    (out/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    result.check_returncode()
print(json.dumps(manifest, indent=2))
