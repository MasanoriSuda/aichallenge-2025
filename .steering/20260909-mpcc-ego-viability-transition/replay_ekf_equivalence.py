from pathlib import Path
import hashlib,json,os,shutil,subprocess,yaml
s=Path(__file__).resolve().parent;out=Path('/output/20260909-ego-viability-ekf-parity-r1');out.mkdir(exist_ok=False)
shutil.copy2(s/'replay_ekf_equivalence.cpp',out/'replay.cpp')
(out/'CMakeLists.txt').write_text('''cmake_minimum_required(VERSION 3.16)
project(ekf_parity)
find_package(ament_cmake REQUIRED)
find_package(ekf_localizer REQUIRED)
find_package(aichallenge_ekf_localizer REQUIRED)
find_package(yaml-cpp REQUIRED)
foreach(backend IN ITEMS old repaired)
 add_executable(${backend} replay.cpp)
 target_compile_features(${backend} PRIVATE cxx_std_17)
 target_link_libraries(${backend} yaml-cpp dl)
 target_link_options(${backend} PRIVATE -Wl,--export-dynamic)
endforeach()
ament_target_dependencies(old ekf_localizer)
ament_target_dependencies(repaired aichallenge_ekf_localizer)
target_compile_definitions(repaired PRIVATE PARTICIPANT_EKF)
''')
with (out/'build.log').open('w') as log:
 subprocess.run(['cmake','-S',str(out),'-B',str(out/'build')],stdout=log,stderr=subprocess.STDOUT,check=True,timeout=60)
 subprocess.run(['cmake','--build',str(out/'build'),'-j2'],stdout=log,stderr=subprocess.STDOUT,check=True,timeout=120)
for backend in ['old','repaired']:
 with (out/(backend+'.log')).open('w') as log:
  subprocess.run([str(out/'build'/backend),'/output/20260909-ego-viability-ekf-inputs/inputs.json',str(out/(backend+'.yaml'))],env=dict(os.environ,ROS_DOMAIN_ID='224'),stdout=log,stderr=subprocess.STDOUT,check=True,timeout=60)
a=yaml.safe_load((out/'old.yaml').read_text());b=yaml.safe_load((out/'repaired.yaml').read_text());assert len(a)==len(b)>0
errors=[]
for x,y in zip(a,b):
 assert x['stamp_ns']==y['stamp_ns']
 errors.extend(abs(u-v) for key in ['state','covariance'] for u,v in zip(x[key],y[key]))
report=dict(ticks=len(a),compared_values=len(errors),maximum_absolute_difference=max(errors),exact_equal=a==b,meaning='Recorded GNSS/twist inputs with shared reinitialisation and frozen per-callback clock; mathematical parity, not full hidden-state runtime reconstruction or MPCC acceptance',source_sha256=hashlib.sha256((out/'replay.cpp').read_bytes()).hexdigest())
(out/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report),flush=True)
assert a==b,report
