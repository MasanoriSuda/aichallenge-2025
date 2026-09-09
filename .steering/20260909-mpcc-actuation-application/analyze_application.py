"""Join direct Unity application records to same-run wire commands only."""
from pathlib import Path
import collections,hashlib,json,re,struct
import numpy as np
run=Path('output/20260909-actuation-observed-dev2-r2')
inputs=Path('output/20260909-actuation-observed-response-r2')
out=Path('output/20260909-actuation-application-analysis-r1');out.mkdir(exist_ok=False)
lines=(run/'unity-player.log').read_text(errors='replace').splitlines();rx=[];applied=[];errors=[]
for line in lines:
 if not line.startswith('MPCC_INPUT_OBS '):continue
 if line.startswith('MPCC_INPUT_OBS error'):errors.append(line);continue
 row={k:v for k,v in re.findall(r'(\w+)=([^ ]+)',line)};row['event']=line.split()[1]
 for k in ['id','seq','source_ns','mono_ticks','ros_ns']:
  if k in row:row[k]=int(row[k])
 for k in ['input','unity','selected_input','selected_unity']:
  if k in row:row[k]=float(row[k])
 (rx if row['event']=='rx' else applied).append(row)
assert not errors,errors
bits=lambda x:struct.pack('f',x)
rows={d:json.loads((inputs/(d+'.json')).read_text()) for d in ['d1','d2']}
keys={d:{(x['stamp_ns'],bits(x['acceleration'])) for x in data if 'wire_steering' in x} for d,data in rows.items()}
lookup={(x['id'],x['seq']):x for x in rx};assert len(lookup)==len(rx)
for x in applied:
 assert x['emergency']=='False','separate immediate emergency route must be classified'
 source=lookup[(x['id'],x['seq'])]
 assert source['source_ns']==x['source_ns'] and bits(source['input'])==bits(x['selected_input'])==bits(x['input'])
 assert x['unity']==x['selected_unity'] and source['mono_ticks']<=x['mono_ticks']
receivers=[]
for rid in sorted({x['id'] for x in rx}):
 unique=collections.Counter();matched=collections.Counter()
 for x in rx:
  if x['id']!=rid:continue
  candidates=[d for d,k in keys.items() if (x['source_ns'],bits(x['input'])) in k]
  matched.update(candidates)
  if len(candidates)==1:unique[candidates[0]]+=1
 assert len(unique)==1 and next(iter(unique.values()))>100
 domain=next(iter(unique));a=[x for x in applied if x['id']==rid];b=[x for x in rx if x['id']==rid];seqs={x['seq'] for x in a}
 delays=[(x['ros_ns']-x['source_ns'])*1e-9 for x in a]
 periods=[(y['ros_ns']-x['ros_ns'])*1e-9 for x,y in zip(a,a[1:])]
 stats=lambda v:dict(min=float(np.min(v)),median=float(np.median(v)),max=float(np.max(v)))
 rec=dict(receiver_id=rid,domain=domain,unique_wire_matches=dict(unique),all_wire_key_matches=dict(matched),received=len(b),applied=len(a),overwritten_before_last_apply=sum(x['seq'] not in seqs for x in b if x['seq']<max(seqs)),unapplied_tail=sum(x['seq']>max(seqs) for x in b),selected_source_to_apply_sec=stats(delays),apply_intervals_sec=stats(periods))
 if domain=='d1':
  # First moving brake publication is the original933 Emergency, not an
  # earlier certified Stop. Preserve later commands only as outcome evidence.
  first=next(x for x in b if x['source_ns']>9_000_000_000 and x['input']<0)
  actual=next(x for x in a if x['source_ns']>=first['source_ns'] and x['input']<0)
  rec['first_brake']=dict(received=first,applied=actual,first_request_to_applied_brake_sec=(actual['ros_ns']-first['source_ns'])*1e-9,overwritten_same_brake_sequences=[x['seq'] for x in b if first['seq']<=x['seq']<actual['seq']])
  # During an already-observed constant applied command, compare actual
  # velocity increment against the canonical wire-a integration. No future
  # signal enters a decision or rescues an earlier proof.
  v=[x for x in rows[domain] if x['topic'].endswith('velocity_status') and 9.3<=x['stamp']<=9.8]
  assert len(v)>5
  pre=[x for x in a if v[0]['stamp_ns']<=x['ros_ns']<=v[-1]['stamp_ns']]
  t=np.array([x['stamp'] for x in v]);speed=np.array([x['v'] for x in v])
  slope,intercept=np.polyfit(t-t[0],speed,1)
  rec['constant_input_scoring']=dict(source_begin_sec=float(t[0]),source_end_sec=float(t[-1]),velocity_samples=len(v),applied_command_min_mps2=min(x['input'] for x in pre),applied_command_max_mps2=max(x['input'] for x in pre),measured_velocity_slope_mps2=float(slope),endpoint_delta_speed_mps=float(speed[-1]-speed[0]),wire_a_delta_speed_mps=float(pre[0]['input']*(t[-1]-t[0])),max_fit_error_mps=float(np.max(np.abs(intercept+slope*(t-t[0])-speed))),meaning='Finite-window empirical plant/model discrepancy, not constant-residual identification across other speeds/steering or a causal repair of world933.')
 receivers.append(rec)
report=dict(run=str(run),authority=False,errors=errors,receivers=receivers,limitations=['Application trace changes simulator logging/timing; no uninstrumented race/performance acceptance.', 'Receiver identity mapped by unique exact source-ns/float32-wire keys; recorder startup misses some receive events.', 'Original normal374/Stop386 run has no application trace; do not transfer this sampling phase.', 'Post-Emergency future motion depends on the Emergency command; cannot score a hypothetical normal continuation as though commands were unchanged.'])
(out/'application-report.json').write_text(json.dumps(report,indent=2)+'\n')
(out/'receiver-events.json').write_text(json.dumps(dict(received=rx,applied=applied),indent=2)+'\n')
(out/'manifest.json').write_text(json.dumps(dict(files={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [run/'unity-player.log',inputs/'d1.json',inputs/'d2.json',Path(__file__)]},authority=False),indent=2)+'\n')
print(json.dumps(report,indent=2))
