"""Replay one exact captured failure and compare unchanged compiled architectures."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess
import sys

run, snapshot, out = map(Path, sys.argv[1:4])
tool = sys.argv[4]
assert tool.startswith('r') and tool[1:].isdigit()
assert (run / 'artifact-restoration.json').exists(), 'Simulator teardown must finish first'
out.mkdir(exist_ok=False)
replay = Path('/output/20260910-nine-state-revalidation-tool-' + tool + '/replay')
compare = Path('/aichallenge/workspace/build/multi_purpose_mpc_ros/mpcc_architecture_compare')
manifest = {'run': str(run), 'source_commit': json.loads((run / 'manifest.json').read_text())['baseline_commit'],
            'authority': False, 'files': [], 'commands': []}
for p in [snapshot, replay, compare, Path(__file__)]:
    manifest['files'].append(dict(path=str(p), sha256=hashlib.sha256(p.read_bytes()).hexdigest()))
shutil.copy2(compare, out / 'mpcc_architecture_compare')
commands = [('exact-pair', [str(replay), str(snapshot), str(out / 'exact-pair.yaml')]),
            ('architecture', [str(compare), str(snapshot)]),
            ('complete-rest', [str(compare), str(snapshot), '--current-world-complete-rest-only']),
            ('metrics', ['python3', str(Path(__file__).parent.parent / 'analyze_run.py'), str(run), str(out / 'metrics')])]
for name, command in commands:
    with (out / (name + '.log')).open('w') as log:
        result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, timeout=180)
    row = dict(name=name, command=command, return_code=result.returncode)
    manifest['commands'].append(row)
    (out / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(row, flush=True)
