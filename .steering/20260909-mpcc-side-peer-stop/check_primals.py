"""Exact row and nonlinear verification of captured numerical iterates."""
from pathlib import Path
import subprocess
import yaml

out = Path('/output/20260909-side-peer-stop-audit/zero')
qp = next((out/'mpcc_architecture_snapshots').glob('*/snapshot.yaml'))
report = yaml.load((out/'qp-0/report.yaml').read_text(), Loader=yaml.CSafeLoader)
program = '/aichallenge/workspace/build/multi_purpose_mpc_ros/mpcc_architecture_compare'
for name,arm in report['arms'].items():
    values = arm.get('primal', arm.get('rejected_primal'))
    path = out/'qp-0'/f'{name}-primal.txt'
    path.write_text('\n'.join(format(v,'.17g') for v in values)+'\n')
    for flag in ['--external-primal','--external-primal-solved-stop-nonlinear-oracle']:
        with (path.parent/f'{name}{flag}.log').open('w') as log:
            subprocess.run([program,str(qp),flag,str(path)],cwd=path.parent,
                           stdout=log,stderr=subprocess.STDOUT,timeout=120)
