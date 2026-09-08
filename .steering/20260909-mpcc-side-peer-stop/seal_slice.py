"""Seal local acceptance and the rejected integrated trial without overwriting history."""
from pathlib import Path
import hashlib
import importlib.util
import json
import shutil

root = Path.cwd()
steering = Path(__file__).resolve().parent
validation = Path('output/20260909-side-peer-validation')
validation.mkdir(exist_ok=True)
for p in Path('/tmp').glob('mpcc-side-peer-*.log'):
    shutil.copy2(p,validation/p.name)
run = Path('output/20260909-side-peer-stop-dev2-r1')
paths = set(steering.glob('*.py')) | set(steering.glob('*.cpp'))
for directory in [validation, run, Path('output/20260909-side-peer-dev2-normal-qp'),
                  Path('output/20260909-side-peer-stop-final-replay'),
                  Path('output/20260909-side-peer-normal-tangent-after'),
                  Path('output/20260909-side-peer-normal-boundary'),
                  Path('output/20260909-side-peer-production-replay'),
                  Path('output/20260909-side-peer-prediction-before'),
                  Path('output/20260909-side-peer-prediction-after')]:
    paths.update(p for p in directory.rglob('*') if p.is_file())
artifacts = []
for p in sorted(paths):
    p = p.resolve()
    digest = hashlib.sha256()
    with p.open('rb') as stream:
        for chunk in iter(lambda:stream.read(1024*1024),b''): digest.update(chunk)
    artifacts.append(dict(path=str(p.relative_to(root)),bytes=p.stat().st_size,sha256=digest.hexdigest()))
baseline = 'b530883997e80cd78aa7e70bcd53bf2b6bcf93fc'
evidence = dict(baseline_commit=baseline,scope='local structural repair accepted; integrated dev2 rejected; full completion open',
                build_packages=25,ctest_groups=60,colcon_records=2346,errors=0,failures=0,skipped=0,
                run_id=run.name,first_failure=dict(domain=2,decision=978,actual_speed_mps=1.58,
                reason='published Stop current-world wall proof failed',missing_executed_source_sequence=442),
                artifacts=artifacts)
(steering/'validation-evidence.json').write_text(json.dumps(evidence,indent=2)+'\n')
registry_path = Path('docs/spec/mpcc-experiment-registry.json')
r = json.loads(registry_path.read_text())
snapshots = [('20260909-side-peer-stop-d2-normal-381',True),
             ('20260909-side-peer-stop-d2-before-stop-976',True),
             ('20260909-side-peer-stop-d2-loss-978',False)]
for name,ready in snapshots:
    if not any(s['snapshot_id']==name for s in r['snapshots']):
        r['snapshots'].append(dict(snapshot_id=name,manifest=str(steering.relative_to(root)/'validation-evidence.json'),replay_ready=ready))
arms = [
    ('production-stop-final',['20260909-peer-envelope-d1-source-396'],'unchanged physical world admits a production zero-objective Stop after producer repair',
     'coordinate and equivalent numerical representation','accepted',
     'native final binary certifies candidate5013267340109276277; full package2346records pass',
     'shared Cartesian plane, side prediction, numerical tangent and seven-intent Stop candidate scope repaired',
     'a new sealed source or regression; this local result does not establish a race pass'),
    ('dev2-integrated',[name for name,_ in snapshots],'full-peer normal and certified Stop maintain current-world authority',
     'sealed structural repair from b5308839','rejected',
     'D2decision978at1.58m/s fails wall proof after Stop442adoption; actual Stop artifact missing; no completed laps/result-details',
     'preserve failure and restore user JSON; no relaxation or unchanged rerun',
     'capture missing executed Stop tuple through an observation-only repair, or a proved structural cause'),
    ('normal-381-kkt',['20260909-side-peer-stop-d2-normal-381'],'normal QP failure is fixed by cold or equilibrated solve',
     'numerical policy and initialization','rejected',
     'all4arms reach4000iterations on identical recorded QP',
     'none; no numerical parameter change',
     'a proved feasible formulation or an equivalent representation with new causal evidence'),
    ('normal-381-affine-certificate',['20260909-side-peer-stop-d2-normal-381'],'the selected affine branch has contradictory physical rows',
     'independent LP and exact rational dual verification','accepted',
     'minimum normalized violation1.111762; nonpositive dual, exact state stationarity, bounded input residual, positive contradiction0.11176197374636564',
     'none; certificate applies to encoded affine branch, not physical-scene infeasibility',
     'a different sealed convex branch/trajectory generation under the same physical world'),
]
for suffix,ids,hypothesis,dimension,result,reason,impact,revisit in arms:
    identity = '20260909-side-peer-'+suffix
    if not any(e['experiment_id']==identity for e in r['experiments']):
        r['experiments'].append(dict(experiment_id=identity,baseline_commit=baseline,snapshot_ids=ids,
            hypothesis=hypothesis,changed_dimension=dimension,result=result,reason=reason,
            production_impact=impact,deleted_code=[],revisit_condition=revisit))
module_path = Path('aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/tools/mpcc_architecture_audit/mpcc_architecture_audit/registry.py')
spec = importlib.util.spec_from_file_location('registry',module_path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
module.validate_registry(r)
registry_path.write_text(json.dumps(r,indent=2)+'\n')
print(json.dumps(dict(artifacts=len(artifacts),snapshots=len(r['snapshots']),experiments=len(r['experiments']))))
