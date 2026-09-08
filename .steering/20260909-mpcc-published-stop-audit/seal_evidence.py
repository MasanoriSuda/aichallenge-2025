"""Seal the observation slice and all accepted/rejected offline comparisons."""
from pathlib import Path
import hashlib,importlib.util,json,shutil
root=Path.cwd();steering=Path(__file__).resolve().parent
run=Path('output/20260909-published-stop-observation-dev2-r1')
validation=Path('output/20260909-published-stop-observation-validation');validation.mkdir(exist_ok=True)
for p in Path('/tmp').glob('mpcc-published-stop-*.log'):shutil.copy2(p,validation/p.name)
paths=set(p for p in steering.iterdir() if p.suffix in ['.py','.cpp','.yaml'])
for directory in [run,validation,Path('output/20260909-published-stop-architectures'),Path('output/20260909-published-stop-observation-before'),Path('output/20260909-published-stop-observation-after')]:
 paths.update(p for p in directory.rglob('*') if p.is_file())
files=[]
for p in sorted(paths):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
 files.append(dict(path=str(p.relative_to(root)) if p.is_absolute() else str(p),bytes=p.stat().st_size,sha256=h.hexdigest()))
baseline='f3b0338cb18b8ece3a494b7438bf35cac4dd35c1'
evidence=dict(baseline_commit=baseline,scope='observation-only production repair; causal offline comparison; integrated completion remains open',build_packages=25,ctest_groups=60,colcon_records=2346,errors=0,failures=0,skipped=0,run_id=run.name,executed_source=453,failure_domain=2,failure_decision=993,artifacts=files)
(steering/'evidence.json').write_text(json.dumps(evidence,indent=2)+'\n')
p=Path('docs/spec/mpcc-experiment-registry.json');registry=json.loads(p.read_text())
ids=['20260909-published-stop-d2-source-453','20260909-published-stop-d2-world-993']
for identity in ids:
 if not any(x['snapshot_id']==identity for x in registry['snapshots']):
  registry['snapshots'].append(dict(snapshot_id=identity,manifest=str(steering.relative_to(root)/'evidence.json'),replay_ready=True))
arms=[
 ('observation','a previous Stop label incorrectly suppresses failed-proof source capture','observation guard only','accepted','native before misses2cases; after6/6; actual453/993tuple captured in unchanged-control diagnostic','bounded observation guard repaired; no control change'),
 ('dev2','moving published Stop maintains current-world authority','unchanged-control diagnostic after observation repair','rejected','five moving Stop publications followed by Emergency993; no completed race','none; failure retained; user JSON restored'),
 ('exact-artifact','the original executed Stop has a valid physical trajectory','zero-solve native reconstruction and independent Cartesian ODE','accepted','original wall/dynamic/rest pass;484ODEsamples maximum3.487751e-6m position error','none; fixed recorded controls'),
 ('current-world','the captured state reproduces the wall-proof failure','same-native current-world revalidation','accepted','delay/publisher prefix clear; continuation and both terminal references fail wall','none; rounded physical steering and unused non-seam wrap length explicit'),
 ('velocity-hindsight','predicted velocity materially changes terminal viability','future measured velocity only, sensitivity','accepted','1.913733 to1.454301m/s makes terminal proof pass','none; future measurement forbidden as production input'),
 ('committed-acceleration','missing published acceleration history produces the bad predicted state','already published controls at existing declared origins; full path recomputed','accepted','predicts1.522632m/s and passes first terminal proof; original predictor reconstructed within1.14e-9m','none; producer repair proposed, simulator application phase remains distinct'),
 ('architectures','A/B/C/D or a fresh Stop bypasses the observed failure on the same world','sealed world993; existing bounded architectures','rejected','all9normal arms and4Stop arms bundle=0; left QP rejection/right exact wall rejection; no physical infeasibility certificate','none; no margins, tolerances, weights, retries or delay tuning'),
]
for suffix,hypothesis,dimension,result,reason,impact in arms:
 identity='20260909-published-stop-'+suffix
 if not any(x['experiment_id']==identity for x in registry['experiments']):
  registry['experiments'].append(dict(experiment_id=identity,baseline_commit=baseline,snapshot_ids=ids,hypothesis=hypothesis,changed_dimension=dimension,result=result,reason=reason,production_impact=impact,deleted_code=['published_stop_retained observation skip'] if suffix=='observation' else [],revisit_condition='new sealed input or causal producer change; do not repeat unchanged rejected arm'))
mp=Path('aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/tools/mpcc_architecture_audit/mpcc_architecture_audit/registry.py')
spec=importlib.util.spec_from_file_location('registry',mp);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);module.validate_registry(registry)
p.write_text(json.dumps(registry,indent=2)+'\n')
print(json.dumps(dict(artifacts=len(files),snapshots=len(registry['snapshots']),experiments=len(registry['experiments']))))
