"""Replay native observer at actual publications, scoring later reports only."""
from pathlib import Path
import json,subprocess,sys
import numpy as np
root=Path(sys.argv[1]);domain=sys.argv[2]
m=json.load(open(root/('d'+domain+'-longitudinal-full-motion.json')))['rows']
events=sorted([(r[1],'O',r) for r in m['/localization/kinematic_state']]+[(r[1],'C',r) for r in m['/control/command/control_cmd']]+[(r[1],'A',r) for r in m['/localization/acceleration']])
out=root/('d'+domain+'-native-longitudinal');out.mkdir(exist_ok=True)
lines=[];actual=[];odometry=None;acceleration=None
for receipt,kind,row in events:
 if kind=='A':acceleration=row;continue
 value=row[5] if kind=='O' else row[3]
 lines.append(f'{kind} {row[0]:.17g} {value:.17g}')
 if kind=='O':odometry=row
 else:
  actual.append(dict(source=row[0],receive=receipt,observed_speed=None if odometry is None else abs(odometry[5]),old_observed_acceleration=None if acceleration is None else acceleration[2]))
(out/'events.txt').write_text('\n'.join(lines)+'\n')
subprocess.run(['/tmp/mpcc-longitudinal-observer-replay',str(out/'events.txt'),str(out/'native.txt')],check=True)
rows=[];truth=m['/vehicle/status/velocity_status'];tt=[r[0] for r in truth];tv=[r[2] for r in truth]
for current,line in zip(actual,(out/'native.txt').read_text().splitlines()):
 stamp,valid,speed,measured,filtered,observed=map(float,line.split());assert stamp==current['source']
 if not valid or current['observed_speed'] is None or current['old_observed_acceleration'] is None or stamp+.13>tt[-1]:continue
 truth_speed=abs(float(np.interp(stamp+.13,tt,tv)))
 old=max(0.,current['observed_speed']+.13*current['old_observed_acceleration'])
 rows.append(dict(**current,native_speed=speed,previous_speed=old,truth_speed=truth_speed,native_error=speed-truth_speed,previous_error=old-truth_speed,filtered_measured_acceleration=measured,filtered_command_acceleration=filtered,observer_stamp=observed))
summary={}
for name,subset in [('all',rows),('moving',[r for r in rows if r['observed_speed']>.1]),('source_time_11_745',[r for r in rows if abs(r['source']-11.744999737)<1e-6])]:
 summary[name]={k:dict(samples=len(subset),mae_mps=float(np.mean([abs(r[k+'_error']) for r in subset])) if subset else None,max_error_mps=max([abs(r[k+'_error']) for r in subset],default=None)) for k in ['native','previous']}
result=dict(scope='native paired observer, recorded receive order; same default0.13s origin and0.9filter; source/receive ordering is bag evidence, not exact controller callback replay',publications=len(actual),scored=len(rows),summary=summary,rows=rows)
(out/'results.json').write_text(json.dumps(result,indent=2)+'\n')
print(root,summary)
