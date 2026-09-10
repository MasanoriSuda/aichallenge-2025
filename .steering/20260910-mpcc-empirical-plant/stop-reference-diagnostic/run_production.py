"""Final production replay and sealed earlier-scene reference comparison."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess
import yaml

out = Path('/output/20260910-nine-state-terminal-reference-production-r1')
out.mkdir(exist_ok=False)
production = Path('/output/20260910-nine-state-revalidation-tool-r27/replay')
original = Path('/output/20260910-nine-state-revalidation-tool-r26/replay')
paths = {}
for run, decision in [('r10', '000000001668'), ('r11', '000000002588')]:
    root = Path('/output/20260910-nine-state-dev2-' + run)
    assert (root / 'artifact-restoration.json').exists()
    paths[run] = next((root / 'd1/mpcc_architecture_snapshots').glob(decision + '-*/snapshot.yaml'))
manifest = {'authority': False, 'solver_invocations': 0, 'files': [], 'commands': []}
for p in [production, original, *paths.values(), Path(__file__)]:
    manifest['files'].append(dict(path=str(p), sha256=hashlib.sha256(p.read_bytes()).hexdigest()))
commands = [(run, [str(production), str(path), str(out / (run + '.yaml'))]) for run, path in paths.items()]
commands += [('r10-original-reference-comparison', [str(original), str(paths['r10']), str(out / 'r10-reference')])]
for name, command in commands:
    with (out / (name + '.log')).open('w') as log:
        result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, timeout=120)
    row = dict(name=name, command=command, return_code=result.returncode)
    manifest['commands'].append(row)
    (out / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(row, flush=True)
    result.check_returncode()
result = yaml.safe_load((out / 'r11.yaml').read_text())
assert result['previous_accepted_revalidation_evidence']['reason'] == 'accepted'
assert result['revalidation_evidence']['reason'] == 'accepted'
references = {}
for name, p in [('r11', Path('/output/20260910-nine-state-stop-reference-r2/report.yaml')),
                ('r10', out / 'r10-reference/report.yaml')]:
    report = yaml.safe_load(p.read_text())
    references[name] = {}
    for key in ['previous_accepted_revalidation_evidence', 'revalidation_evidence']:
        variants = {}
        for label, row in report[key].items():
            payload = {'snapshot_sha256': hashlib.sha256(paths[name].read_bytes()).hexdigest(),
                       'decision': row['decision'], 'label': label,
                       'progress': row['reference_progress'], 'lateral': row['reference_lateral']}
            variants[label] = hashlib.sha256(json.dumps(payload, sort_keys=True).encode()).hexdigest()
        references[name][key] = variants
(out / 'diagnostic-reference-identities.json').write_text(json.dumps(references, indent=2) + '\n')
