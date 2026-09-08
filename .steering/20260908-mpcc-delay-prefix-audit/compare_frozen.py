"""Run existing comparisons on sealed snapshots; record rejected arms too."""
import json
from pathlib import Path
import subprocess

root = Path('/output/20260908-wall-map-single-r1')
output = root/'offline-comparisons'
output.mkdir(exist_ok=True)
program = '/aichallenge/workspace/build/multi_purpose_mpc_ros/mpcc_architecture_compare'
results = []
for sequence in [10074, 10081, 10699]:
    matches = list((root/'d1/mpcc_architecture_snapshots').glob(f'{sequence:012d}-*/snapshot.yaml'))
    assert len(matches)==1, matches
    for option in ['--warm-start-primal-physical-nonlinear-oracle', '--rejected-primal-physical-nonlinear-oracle', '--wall-restoration-only']:
        destination = output/f'{sequence}-{option.removeprefix("--")}.txt'
        with destination.open('w') as stream:
            process = subprocess.run([program,str(matches[0]),option],cwd=output,stdout=stream,stderr=subprocess.STDOUT,timeout=60)
        record = dict(sequence=sequence,option=option,return_code=process.returncode,output=str(destination))
        results.append(record)
        print(json.dumps(record),flush=True)
(output/'results.json').write_text(json.dumps(results,indent=2)+'\n')
