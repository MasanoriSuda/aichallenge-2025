"""Summarize a sealed run's active authorities and callback timing, excluding shutdown."""
from pathlib import Path
import json,re,sys
root=Path(sys.argv[1]);monitor=json.load(open(root/'monitor-summary.json'));shutdown=monitor['shutdown_started_wall_sec']
summary={}
for domain in monitor['last_observation']:
 state=None;callbacks=[];overrides=[];stops=[];causal_failures=[];states=[];contracts=0
 for raw in (root/domain/'autoware.log').read_text(errors='replace').splitlines():
  line=re.sub(r'\x1b\[[0-9;]*m','',raw)
  stamp=re.search(r'\[(\d+\.\d+)\]',line)
  if not stamp or float(stamp[1])>=shutdown:continue
  time=float(stamp[1]);transition=re.search(r'AWSIM vehicle state observed: state=(\S+)',line)
  if transition:state=transition[1];states.append(dict(receive_sec=time,state=state))
  if 'missing causal longitudinal response observation' in line:causal_failures.append(dict(receive_sec=time,state=state,line=line))
  if state not in ['ready','start']:continue
  match=re.search(r'Control callback runtime: cycles=(\d+), elapsed_ms=([^/]+)/([^\(]+).*overruns=(\d+)',line)
  if match:callbacks.append(dict(receive_sec=time,cycles=int(match[1]),mean_ms=float(match[2]),max_ms=float(match[3]),overruns=int(match[4])))
  if 'MPCC execution contract:' in line:
   contracts+=1;authority=re.search(r'MPCC execution contract:.*?authority=([^,]+)',line);speed=re.search(r'actual=([^m]+)m/s',line)
   if authority and speed and authority[1] in ['emergency-override','recovery-override'] and abs(float(speed[1]))>.1:overrides.append(line)
  if 'canonical-certified-terminal-stop' in line:stops.append(line)
 details=root/domain/(domain+'-result-details.json')
 if not details.exists():details=root/'finish-results'/(domain+'-result-details.json')
 summary[domain]=dict(states=states,details=json.load(open(details)) if details.exists() else None,active_moving_overrides=overrides,active_contract_traces=contracts,certified_stop_traces=stops,causal_observation_failures=causal_failures,callbacks=dict(windows=len(callbacks),cycles=sum(r['cycles'] for r in callbacks),maximum_ms=max([r['max_ms'] for r in callbacks],default=None),overruns=sum(r['overruns'] for r in callbacks),rows=callbacks))
result=dict(run_id=root.name,shutdown_started_wall_sec=shutdown,domains=summary,limitations='throttled trace/window totals are reported observations; same-run bag timing is checked separately; absent result-details/penalty stays unknown')
(root/'authority-timing-summary.json').write_text(json.dumps(result,indent=2)+'\n')
for domain,value in summary.items():print(domain,{k:v for k,v in value.items() if k not in ['states','active_moving_overrides','certified_stop_traces','causal_observation_failures','callbacks']},'moving_override_count',len(value['active_moving_overrides']),'causal_failure_count',len(value['causal_observation_failures']),'callbacks', {k:v for k,v in value['callbacks'].items() if k!='rows'})
