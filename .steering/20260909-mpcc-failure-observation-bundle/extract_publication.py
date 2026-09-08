"""Extract an original publication for the existing zero-solve replay, preserving raw input."""
from pathlib import Path
import hashlib
import json
import shutil
import sys
import yaml

source_file=Path(sys.argv[1]);out=Path(sys.argv[2]);out.mkdir(exist_ok=False)
root=yaml.safe_load(source_file.read_text());bundle=root['publication_bundle']
assert bundle['schema']=='mpcc-authority-loss-publication/v1'
assert bundle['status']=='present',bundle
evidence=bundle['execution_evidence'];publication=evidence['publication']
assert publication['failure_decision_id']==root['source']['problem_context']['decision_id']
assert publication['failure_interaction_fingerprint']==root['interaction_fingerprint']
assert publication['failure_observation_sec']==root['source']['snapshot_sec']
assert publication['failure_control_origin_sec']==root['source']['control_prediction_origin_sec']
assert evidence['source_sequence']==bundle['source']['sequence']
assert evidence['source_problem_fingerprint']==bundle['source']['problem_context']['fingerprint']
published=dict(schema='mpcc-architecture-failure-snapshot/v3' if
    bundle['source']['replay_world'].get('terminal_stop_contract_available',False)
    else 'mpcc-architecture-failure-snapshot/v2',exact_qp_available=False,
    pipeline_stage='physical-proof',failure_outcome='extracted-published-execution',
    failure_detail=root['failure_detail'],source=bundle['source'],
    interaction_fingerprint=bundle['interaction_fingerprint'],execution_evidence=evidence)
grid=published['source']['wall_grid'];payload=None
if grid['available']:
    payload=source_file.parent/grid['payload']
    assert payload.parent.resolve()==source_file.parent.resolve()
    shutil.copy2(payload,out/payload.name)
snapshot=out/'snapshot.yaml';snapshot.write_text(yaml.safe_dump(published,sort_keys=False))
(out/'extraction-manifest.json').write_text(json.dumps(dict(
    source=str(source_file),source_sha256=hashlib.sha256(source_file.read_bytes()).hexdigest(),
    extracted_sha256=hashlib.sha256(snapshot.read_bytes()).hexdigest(),
    source_sequence=evidence['source_sequence'],failure_decision=publication['failure_decision_id'],
    publication_kind=publication['source_kind'],
    caveat='current-world-bundle means source-only evidence; only exact-executed is the executed artifact',
    payload_sha256=hashlib.sha256(payload.read_bytes()).hexdigest() if payload else None),indent=2)+'\n')
print(snapshot)
