"""Diagnostic current-world validation of the actual published normal source.

Copies are explicitly counterfactual input assemblies, not native observations.
The sealed source, artifact and actual publication anchor are kept together.
"""
from pathlib import Path
import copy, hashlib, json, shutil, subprocess, yaml
run=Path('/output/20260909-stop-viability-dev2-r1')
path=next((run/'d1/mpcc_architecture_snapshots').glob('000000000930-*/snapshot.yaml'))
out=Path('/output/20260909-actuation-published-revalidation-r3');out.mkdir(exist_ok=False)
n=yaml.load(path.read_bytes(),Loader=yaml.CSafeLoader);pub=n['publication_bundle']
assert pub['status']=='present'
assert pub['execution_evidence']['source_sequence']==374
assert n['revalidation_evidence']['certified_plan_evidence']['artifact']['source_sequence']==386
clock=pub['certified_plan_evidence']['publication']
assert clock['publication_decision_id']==929 and clock['failure_decision_id']==930
source=copy.deepcopy(n)
source.update(source=pub['source'],interaction_fingerprint=pub['interaction_fingerprint'],execution_evidence=pub['certified_plan_evidence']['artifact'])
for k in ['publication_bundle','revalidation_evidence','previous_accepted_revalidation_evidence']:source.pop(k,None)
failure=copy.deepcopy(n);failure.pop('previous_accepted_revalidation_evidence',None)
x=failure['revalidation_evidence']
x.update(inspected_plan_status='present',inspected_source=pub['source'],inspected_interaction_fingerprint=pub['interaction_fingerprint'],inspected_artifact=pub['certified_plan_evidence']['artifact'],certified_plan_evidence=pub['certified_plan_evidence'])
x['request']['execution_clock']=dict(kind='published-plan',first_published_control_origin_sec=clock['publication_control_origin_sec'],first_published_artifact_elapsed_sec=clock['publication_artifact_elapsed_sec'])
meaning='Diagnostic assembly: actual normal374 artifact/source/929 publication clock in original930 world Request. Original930 inspected386 is preserved separately. No actual374 retained-evaluation trace claim; zero solves, no authority.'
for doc,name in [(source,'source.yaml'),(failure,'diagnostic-current-world.yaml')]:
 doc['diagnostic_context']=meaning
 (out/name).write_text(yaml.safe_dump(doc,sort_keys=False))
for grid in path.parent.glob('*.bin'):shutil.copy2(grid,out/grid.name)
binary=Path('/output/20260909-publication-clock-revalidation-tool-r1/replay')
cmd=[str(binary),str(out/'source.yaml'),str(out/'result.yaml'),str(out/'diagnostic-current-world.yaml'),'revalidation_evidence']
with (out/'native.log').open('w') as log:
 result=subprocess.run(cmd,cwd='/aichallenge/workspace/build/multi_purpose_mpc_ros',stdout=log,stderr=subprocess.STDOUT,timeout=120)
(out/'manifest.json').write_text(json.dumps(dict(meaning=meaning,authority=False,solver_invocations=0,input=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),command=cmd,return_code=result.returncode),indent=2)+'\n')
print((out/'native.log').read_text(),flush=True)
result.check_returncode()
