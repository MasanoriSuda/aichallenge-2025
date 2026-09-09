"""After teardown, replay recorded source-free physical plans and exact Requests."""
from pathlib import Path
import hashlib,json,subprocess,yaml
run=Path('/output/20260909-stop-viability-dev2-r1')
assert (run/'artifact-restoration.json').exists(), 'teardown/restoration must finish first'
out=Path('/output/20260909-stop-viability-plan-revalidation-r1');out.mkdir(exist_ok=False)
binary=Path('/output/20260909-peer-epochs-plan-tool-r1/replay')
records=[]
for domain in ['d1','d2']:
    for path in sorted((run/domain/'mpcc_architecture_snapshots').glob('*/snapshot.yaml')):
        document=yaml.load(path.read_bytes(),Loader=yaml.CSafeLoader)
        for role in ['publication_bundle','revalidation_evidence','previous_accepted_revalidation_evidence']:
            observation=document.get(role,{})
            physical=observation.get('certified_plan_evidence',{})
            artifact=physical.get('artifact',{})
            row=dict(domain=domain,input=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                role=role,failure_decision=document['source']['problem_context']['decision_id'],
                world_fingerprint=document['interaction_fingerprint'],failure_outcome=document['failure_outcome'],
                observation_status=observation.get('status'),physical_status=physical.get('status'),
                solver_source_status=physical.get('solver_source_status'),sequence=artifact.get('source_sequence'))
            records.append(row)
            if physical.get('status')!='present' or physical.get('solver_source_status')!='absent':continue
            if role!='publication_bundle' and observation.get('status')!='present':continue
            destination=out/domain/path.parent.name/role;destination.mkdir(parents=True,exist_ok=False)
            cmd=[str(binary),str(path),str(destination/'result.yaml'),role]
            with (destination/'native.log').open('w') as log:
                r=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,timeout=120)
            row.update(command=cmd,return_code=r.returncode,output=str(destination))
            print(json.dumps(row),flush=True)
            print((destination/'native.log').read_text(),flush=True)
(out/'manifest.json').write_text(json.dumps(dict(run=str(run),replay_binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),solver_invocations=0,authority=False,records=records),indent=2)+'\n')
assert any('return_code' in row for row in records),'no valid source-free evidence: capture objective not met'
assert all(row.get('return_code',0)==0 for row in records),'one or more exact replays failed'
