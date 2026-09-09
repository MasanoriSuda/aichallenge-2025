"""Native actuator and contact-qualified sleep replay of every recorded physics tick.

Private receiver/ground observations are exogenous replay inputs, never live
controller features. Scores do not prove future contact or command arrival.
"""
from pathlib import Path
import hashlib
import json
import subprocess
import numpy as np

OUT=Path('output/20260910-shared-plant-native-replay-r2');OUT.mkdir(exist_ok=False)
BINARY=Path('output/20260910-shared-plant-native-r2/check')


def stats(v):
    v=abs(np.asarray(v))
    return dict(count=len(v),mae=float(v.mean()),maximum=float(v.max())) if len(v) else dict(count=0)


def evaluate(name,identity):
    source=Path(f'output/20260909-force-step-analysis-{name}-r1/physics.npz')
    raw=np.load(source);ix={str(k):i for i,k in enumerate(raw['columns'])};a=raw['values'];a=a[a[:,ix['id']]==identity]
    ground=a[:,[ix[f'w{n}_ground'] for n in range(4)]]
    mask=ground@np.array([1,2,4,8])
    columns=np.column_stack([a[:,ix['fixed']],a[:,ix['dt']],a[:,ix['steer_input_deg']],a[:,ix['steer_deg']],
                             a[:,ix['vehicle_speed']],a[:,ix['wire']],mask])
    inputs=''.join(' '.join(f'{v:.17g}' for v in row)+'\n' for row in columns)
    result=subprocess.run([str(BINARY),'replay'],input=inputs,text=True,capture_output=True,check=True)
    destination=OUT/(name+'-'+str(identity));destination.mkdir()
    (destination/'inputs.txt').write_text(inputs);(destination/'predictions.txt').write_text(result.stdout)
    prediction=np.array([[float(v) for v in line.split()] for line in result.stdout.splitlines()])
    assert len(prediction)==len(a)
    evaluated=a[:,ix['fixed']]-a[0,ix['fixed']]>=.3
    errors=[]
    for n in range(4):
        drive=a[:,[ix[f'w{n}_drive_{c}'] for c in 'xyz']]
        direction=a[:,[ix[f'w{n}_f_{c}'] for c in 'xyz']]
        actual=np.sum(drive*direction,axis=1)
        expected=prediction[:,1]*a[:,ix[f'w{n}_called']]
        errors.extend(expected-actual)
    sleep=a[:,ix['sleep']]
    changes=np.flatnonzero(sleep[1:]!=sleep[:-1])+1
    transitions=[dict(source_sec=float(a[j,ix['fixed']]),sleep=bool(sleep[j]),speed_mps=float(a[j,ix['vehicle_speed']]),
                      wire_mps2=float(a[j,ix['wire']]),grounded_mask=int(mask[j])) for j in changes]
    row=dict(source=str(source),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),records=len(a),
      tire_error_deg=stats(prediction[evaluated,0]-a[evaluated,ix['steer_deg']]),
      wheel_drive_error_mps2=stats(errors),sleep_mismatches=int(np.count_nonzero(prediction[:,2]!=sleep)),
      actual_sleep_ticks=int(sleep.sum()),sleep_transitions=transitions)
    assert row['tire_error_deg']['maximum']<3e-6,row
    assert row['wheel_drive_error_mps2']['maximum']<3e-6,row
    assert row['sleep_mismatches']==0,row
    return row


report=dict(authority=False,meaning=__doc__,binary_sha256=hashlib.sha256(BINARY.read_bytes()).hexdigest(),
 single=evaluate('single',-823441338),dev2={str(i):evaluate('dev2',i) for i in [1594692888,-615963298]})
(OUT/'report.json').write_text(json.dumps(report,indent=2)+'\n')
for name,row in [('single',report['single']),*report['dev2'].items()]:
 print(name,{k:v for k,v in row.items() if k not in ['sleep_transitions','source_sha256']},flush=True)
