"""Match canonical now-pose against same-run source-to-now motion estimates."""
from pathlib import Path
import hashlib,json,math,yaml
run=Path('output/20260909-stop-viability-dev2-r1')
inputs=Path('output/20260909-stop-viability-runtime-inputs-r1')
results=[]
for domain in ['d1','d2']:
    rows=json.loads((inputs/(domain+'.json')).read_text())
    odom=[r for r in rows if r['topic'].endswith('kinematic_state')]
    for path in sorted((run/domain/'mpcc_architecture_snapshots').glob('*/snapshot.yaml')):
        document=yaml.load(path.read_bytes(),Loader=yaml.CSafeLoader)
        for role in ['revalidation_evidence','previous_accepted_revalidation_evidence']:
            observation=document.get(role,{})
            request=observation.get('request',{})
            if observation.get('status')!='present' or request.get('decision_id',0)<900:continue
            first=request['measured_to_control_path'][0]
            candidates=[]
            for r in odom:
                age=request['now_sec']-r['stamp']
                if not 0<=age<=.5 or abs(r['v'])!=request['current_speed_mps']:continue
                x,y,yaw=r['x'],r['y'],r['yaw'];w=r['yaw_rate'];v=r['v']
                angle=yaw+w*age
                if abs(w)<1e-6:x+=v*age*math.cos(yaw);y+=v*age*math.sin(yaw)
                else:x+=v/w*(math.sin(angle)-math.sin(yaw));y-=v/w*(math.cos(angle)-math.cos(yaw))
                angle=math.atan2(math.sin(angle),math.cos(angle))
                distance=math.hypot(x-first['x_m'],y-first['y_m'])
                yaw_error=math.atan2(math.sin(angle-first['yaw_rad']),math.cos(angle-first['yaw_rad']))
                candidates.append(dict(source=r,age_sec=age,predicted_now=dict(x=x,y=y,yaw=angle),distance_m=distance,yaw_error_rad=yaw_error,raw_pose_distance_m=math.hypot(r['x']-first['x_m'],r['y']-first['y_m'])))
            candidates.sort(key=lambda x:x['distance_m']+abs(x['yaw_error_rad']))
            matches=[c for c in candidates if c['distance_m']<1e-8 and abs(c['yaw_error_rad'])<1e-10]
            row=dict(domain=domain,input=str(path),input_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),role=role,decision=request['decision_id'],now_sec=request['now_sec'],control_sec=request['control_origin_sec'],prefix_duration_sec=request['measured_to_control_elapsed_sec'][-1],exact_candidates=len(matches),best=candidates[0] if candidates else None,meaning='Numerical match to the declared held-twist estimator, not true-motion accuracy or controller receipt identity; ambiguity is explicit.')
            results.append(row);print(json.dumps(row))
(inputs/'runtime-epoch-matches.json').write_text(json.dumps(results,indent=2)+'\n')
assert results and all(r['exact_candidates']>0 for r in results),'some captured now poses do not match the declared estimator'
