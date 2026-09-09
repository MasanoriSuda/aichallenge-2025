from pathlib import Path
import hashlib,json,shutil,yaml
run=Path('output/20260909-stop-provenance-dev2-r1')
world=next((run/'d1/mpcc_architecture_snapshots').glob('000000000949*/snapshot.yaml'))
y=yaml.load(world.read_text(),Loader=yaml.CSafeLoader);b=y['publication_bundle']
assert b['status']=='present' and b['execution_evidence']['source_sequence']==365
out=Path('output/20260909-final-authority-365-original');out.mkdir(exist_ok=False)
y['source']=b['source'];y['interaction_fingerprint']=b['interaction_fingerprint']
y['execution_evidence']=b['execution_evidence'];del y['publication_bundle']
payload=y['source']['wall_grid']['payload'];assert Path(payload).name==payload
shutil.copy2(world.parent/payload,out/payload)
(out/'snapshot.yaml').write_text(yaml.safe_dump(y,sort_keys=False))
(out/'manifest.json').write_text(json.dumps(dict(world=str(world),world_sha256=hashlib.sha256(world.read_bytes()).hexdigest(),source_sequence=365,source_fingerprint=y['interaction_fingerprint'],solver_invocations=0,limits='Actual normal365in949atomic bundle. No Stop379artifact or951world is inferred.'),indent=2)+'\n')
print(out)
