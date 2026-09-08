"""Revisit only changed physical x0 behavior and a new full-body Stop homotopy."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess

run = Path('/output/20260909-peer-envelope-dev2-r1')
out = Path('/output/20260909-semantic-initial-after')
out.mkdir(exist_ok=False)
program = Path('/aichallenge/workspace/build/multi_purpose_mpc_ros/mpcc_architecture_compare')
source = next(p for p in (run/'d1/mpcc_architecture_snapshots').glob('000000000941-*/snapshot.yaml')
              if 'normal-authority-unavailable' in str(p))
world = next((run/'world-target-comparison-r2/396/candidate').glob('*/snapshot.yaml'))
results = []
for name,path,args in [
    ('original-941',source,[]),
    ('world-396-stop-support',world,['--stop-physical-support-only']),
    ('world-396-physical-sqp',world,['--physical-dynamic-sqp-only']),
]:
    work = out/name
    work.mkdir()
    with (work/'comparison.log').open('w') as log:
        result = subprocess.run([str(program),str(path),*args],cwd=work,
                                stdout=log,stderr=subprocess.STDOUT,timeout=180)
    results.append(dict(name=name,source=str(path),source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                        return_code=result.returncode,arguments=args))
    (out/'results.json').write_text(json.dumps(results,indent=2)+'\n')
    print(json.dumps(results[-1]),flush=True)
