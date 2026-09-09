"""After teardown, replay exact captured requests without solving or substituting."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess
import yaml

run = Path('/output/20260909-ego-viability-dev2-r1')
assert (run/'monitor-summary.json').exists(), 'Simulator must finish first'
out = Path('/output/20260909-publication-clock-revalidation-r1')
out.mkdir(exist_ok=False)
binary = Path('/output/20260909-publication-clock-revalidation-tool-r1/replay')
b = Path('/aichallenge/workspace/build/multi_purpose_mpc_ros')
records = []
for domain in ['d1', 'd2']:
    for path in sorted((run/domain/'mpcc_architecture_snapshots').glob('*/snapshot.yaml')):
        document = yaml.load(path.read_bytes(), Loader=yaml.CSafeLoader)
        for key in ['revalidation_evidence', 'previous_accepted_revalidation_evidence']:
            observation = document.get(key, {})
            request = observation.get('request', {})
            publication = document.get('publication_bundle', {})
            row = dict(observed_role=key, path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                       world_fingerprint=document['interaction_fingerprint'],
                       outcome=document['failure_outcome'],
                       status=observation.get('status'),
                       inspected_status=observation.get('inspected_plan_status'),
                       publication_status=publication.get('status'),
                       decision=request.get('decision_id'), speed=request.get('current_speed_mps'),
                       inspected_sequence=observation.get('inspected_artifact', {}).get('source_sequence'),
                       published_sequence=publication.get('execution_evidence', {}).get('source_sequence'))
            records.append(row)
            if row['status'] != 'present' or row['inspected_status'] != 'present':
                continue
            destination = out/domain/path.parent.name/key
            destination.mkdir(parents=True, exist_ok=False)
            # Change only the top-level source role for the existing strict loader.
            # Its recorded fingerprint and artifact must validate with zero solves.
            inspected = dict(document)
            inspected['source'] = observation['inspected_source']
            inspected['interaction_fingerprint'] = observation['inspected_interaction_fingerprint']
            inspected['execution_evidence'] = observation['inspected_artifact']
            inspected.pop('publication_bundle', None)
            inspected.pop('revalidation_evidence', None)
            inspected.pop('previous_accepted_revalidation_evidence', None)
            payload = inspected['source']['wall_grid']['payload']
            assert Path(payload).name == payload
            shutil.copy2(path.parent/payload, destination/payload)
            source = destination/'inspected.yaml'
            source.write_text(yaml.safe_dump(inspected, sort_keys=False))
            command = [str(binary), str(source), str(destination/'result.yaml'), str(path), key]
            with (destination/'native.log').open('w') as log:
                result = subprocess.run(command, cwd=b, stdout=log, stderr=subprocess.STDOUT, timeout=120)
            row.update(command=command, return_code=result.returncode, output=str(destination),
                       inspected_source_sha256=hashlib.sha256(source.read_bytes()).hexdigest())
            print(json.dumps(row), flush=True)
            print((destination/'native.log').read_text(), flush=True)
(out/'manifest.json').write_text(json.dumps(dict(run=str(run),
    replay_binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),
    solver_invocations=0, authority=False, records=records), indent=2)+'\n')
