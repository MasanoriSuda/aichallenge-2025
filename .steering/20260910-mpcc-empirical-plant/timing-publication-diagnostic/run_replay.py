"""Sequential same-binary diagnostics; never run alongside a simulator."""
from pathlib import Path
import hashlib
import json
import subprocess

run = Path('/output/20260910-nine-state-dev2-r10')
snapshot = next((run / 'd1/mpcc_architecture_snapshots').glob('000000001668-*/snapshot.yaml'))
out = Path('/output/20260910-nine-state-dev2-r10-causal-r1')
out.mkdir(exist_ok=False)
replay = Path('/output/20260910-nine-state-revalidation-tool-r24/replay')
compare = Path('/aichallenge/workspace/build/multi_purpose_mpc_ros/mpcc_architecture_compare')
commands = [('exact-pair', [str(replay), str(snapshot), str(out / 'exact-pair.yaml')]),
            ('architecture', [str(compare), str(snapshot)]),
            ('complete-rest', [str(compare), str(snapshot), '--current-world-complete-rest-only']),
            ('public-interval', ['python3', str(Path(__file__).with_name('extract_interval.py')),
                                 str(run), str(out / 'public-interval'), 'd1', '28.1', '28.65'])]
manifest = {'source_commit': json.loads((run / 'manifest.json').read_text())['baseline_commit'],
            'snapshot': str(snapshot), 'snapshot_sha256': hashlib.sha256(snapshot.read_bytes()).hexdigest(),
            'authority': False, 'commands': []}
for name, command in commands:
    with (out / (name + '.log')).open('w') as log:
        result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, timeout=180)
    row = dict(name=name, command=command, return_code=result.returncode)
    manifest['commands'].append(row)
    (out / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(row, flush=True)
