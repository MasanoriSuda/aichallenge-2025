"""Current-step force oracle versus next physics state; diagnostic, no fitting."""
from pathlib import Path
import collections
import hashlib
import json
import sys
import numpy as np

run=Path(sys.argv[1]);out=Path(sys.argv[2]);out.mkdir(exist_ok=False)
records=[];keys=None;received=applied=0
with (run/'unity-player.log').open(errors='replace') as stream:
    for line in stream:
        assert not line.startswith(('MPCC_FORCE_ERROR','MPCC_INPUT_OBS error')),line
        received+=line.startswith('MPCC_INPUT_OBS rx ')
        applied+=line.startswith('MPCC_INPUT_OBS apply ')
        if not line.startswith('MPCC_FORCE_OBS '):continue
        row=dict(part.split('=',1) for part in line.split()[1:])
        if keys is None:keys=list(row)
        assert list(row)==keys
        records.append([float(row[key]) for key in keys])
all_data=np.array(records);assert np.isfinite(all_data).all()
indices={key:i for i,key in enumerate(keys)}
def col(data,key):return data[:,indices[key]]
def vec(data,key):return data[:,[indices[key+axis] for axis in 'xyz']]
def rot(data,key):
    q=data[:,[indices[key+axis] for axis in 'xyzw']]
    q=q/np.linalg.norm(q,axis=1)[:,None]
    x,y,z,w=q.T
    return np.stack([1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w),
        2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w),
        2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)],axis=1).reshape(-1,3,3)
def local(rotation,value):return np.einsum('nji,nj->ni',rotation,value)
def stats(value):
    a=np.asarray(value).reshape(-1);absolute=np.abs(a)
    return dict(count=len(a),mean=float(np.mean(a)),mae=float(np.mean(absolute)),
        p95=float(np.percentile(absolute,95)),maximum=float(np.max(absolute))) if len(a) else dict(count=0)

reports=[]
for identity in sorted(set(col(all_data,'id'))):
    data=all_data[col(all_data,'id')==identity]
    assert np.all(np.diff(col(data,'tick'))==1)
    assert np.all(np.abs(np.diff(col(data,'fixed'))-col(data,'dt')[:-1])<1e-9)
    rotation=rot(data,'eq_');mass=col(data,'mass')[:,None]
    v=vec(data,'ev_');w=vec(data,'ew_');body_v=local(rotation,v);body_w=local(rotation,w)
    wheel_f=[];wheel_t=[];grip_f=[];drive_a=[];reconstructed=[]
    for i in range(4):
        key=f'w{i}_';lat=vec(data,key+'lat_');drive=vec(data,key+'drive_')
        f=lat+mass*drive;wheel_f.append(f)
        wheel_t.append(np.cross(vec(data,key+'p_')-vec(data,'com_'),f))
        grip_f.append(lat);drive_a.append(drive)
        side=vec(data,key+'s_');point_v=vec(data,key+'v_');normal=vec(data,key+'n_')
        planar=point_v-np.sum(point_v*normal,axis=1)[:,None]*normal
        expected=-.236*col(data,key+'sprung')/col(data,'dt')*np.sum(planar*side,axis=1)
        reconstructed.append(expected[:,None]*side*col(data,key+'called')[:,None]-lat)
    force=vec(data,'ef_')-vec(data,'bf_');torque=vec(data,'et_')-vec(data,'bt_')
    grounded=np.stack([col(data,f'w{i}_ground') for i in range(4)],axis=1)
    mask=(body_v[:,2]>=.5)&(col(data,'sleep')==0)&(col(data,'gear')==3)
    dt=col(data,'dt')[:-1,None]
    usable=mask[:-1]&mask[1:]
    gravity=vec(data,'gravity_')*col(data,'use_gravity')[:,None]
    # Exact current applied-force observation; contact solver may add other forces later.
    nominal=v[:-1]+dt*(vec(data,'ef_')[:-1]/mass[:-1]+gravity[:-1])
    current=vec(data,'bv_')[1:]
    prediction={'force_no_drag':nominal,
        'force_linear_drag':nominal*(1-dt*col(data,'drag')[:-1,None]),
        'force_inverse_drag':nominal/(1+dt*col(data,'drag')[:-1,None])}
    velocity_errors={name:dict(world_horizontal=stats((p-current)[usable][:,[0,2]]),
        current_body_longitudinal=stats(local(rotation[:-1],p-current)[usable,2]),
        world_vertical=stats((p-current)[usable,1])) for name,p in prediction.items()}
    # Longitudinal derivative uses current rotation and angular velocity only.
    body_acc=local(rotation,vec(data,'ef_')/mass+gravity)-np.cross(body_w,body_v)-col(data,'drag')[:,None]*body_v
    raw=col(data,'wire')
    predicted_long={'wire_equals_net':body_v[:-1,2]+dt[:,0]*raw[:-1],
        'current_force_body_derivative':body_v[:-1,2]+dt[:,0]*body_acc[:-1,2]}
    truth=local(rot(data,'bq_')[1:],current)[:,2]
    long_errors={name:stats((p-truth)[usable]) for name,p in predicted_long.items()}
    # Torque tests report both direct force moment and body momentum, without selecting a model.
    principal_rotation=np.einsum('nij,njk->nik',rotation,rot(data,'iq_'))
    principal_w=local(principal_rotation,w)
    principal_t=local(principal_rotation,vec(data,'et_'))
    inertia=vec(data,'inertia_')
    alpha=np.einsum('nij,nj->ni',principal_rotation,principal_t/inertia)
    w_prediction=(w[:-1]+dt*alpha[:-1])*(1-dt*col(data,'angular_drag')[:-1,None])
    w_truth=vec(data,'bw_')[1:]
    component=dict(drive=local(rotation,np.sum(drive_a,axis=0))[:,2],
        lateral_force=local(rotation,np.sum(grip_f,axis=0)/mass)[:,2],
        gravity=local(rotation,gravity)[:,2],
        rotating_frame=-np.cross(body_w,body_v)[:,2],drag=-col(data,'drag')*body_v[:,2])
    row=dict(vehicle_id=int(identity),records=len(data),moving_intervals=int(usable.sum()),
        first_fixed_sec=float(col(data,'fixed')[0]),last_fixed_sec=float(col(data,'fixed')[-1]),
        dt_sec=sorted(set(col(data,'dt'))),
        grounded_patterns_moving=dict(collections.Counter(''.join(str(int(x)) for x in g) for g in grounded[mask])),
        wheel_vs_accumulated_force_N=stats((np.sum(wheel_f,axis=0)-force)[mask]),
        wheel_vs_accumulated_torque_Nm=stats((np.sum(wheel_t,axis=0)-torque)[mask]),
        reconstructed_lateral_force_N=stats(np.stack(reconstructed,axis=1)[mask]),
        velocity_step_errors_mps=velocity_errors,longitudinal_step_errors_mps=long_errors,
        yaw_velocity_step_error_radps=stats((w_prediction-w_truth)[usable,1]),
        component_acceleration_mps2={name:stats(values[mask]) for name,values in component.items()})
    reports.append(row)
np.savez_compressed(out/'physics.npz',values=all_data,columns=np.array(keys))
report=dict(authority=False,run=str(run),source_log_sha256=hashlib.sha256((run/'unity-player.log').read_bytes()).hexdigest(),
    received=received,applied=applied,vehicles=reports,
    meaning='Current actual force oracle vs next physics state; no parameter fitting or production promotion. Same-step Wheel CIL locals compare with accumulated force. Ground/contact/suspension forces can act after Vehicle.FixedUpdate.',
    limitations=['Observed force/contact/application are simulator-private diagnostics, not available future controller inputs.',
    'Moving score excludes speed<.5 and sleep/non-Drive; all raw records retained.',
    'Derivative and drag comparisons are predeclared diagnostic approximations; next measured state is only a target, not a prediction feature.',
    'Local skid .236 is used solely to independently check captured actual lateral-force locals.'])
(out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
