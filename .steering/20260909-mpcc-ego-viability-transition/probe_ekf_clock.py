from pathlib import Path
import hashlib,json,os,shutil,subprocess
s=Path(__file__).resolve().parent
out=Path('/output/20260909-ego-viability-ekf-clock-native-r1');out.mkdir(exist_ok=False)
shutil.copy2(s/'probe_ekf_clock.cpp',out/'probe.cpp')
(out/'CMakeLists.txt').write_text('''cmake_minimum_required(VERSION 3.16)
project(mpcc_ekf_clock_probe)
find_package(ament_cmake REQUIRED)
find_package(ekf_localizer REQUIRED)
add_executable(probe probe.cpp)
target_compile_features(probe PRIVATE cxx_std_17)
ament_target_dependencies(probe ekf_localizer)
target_link_libraries(probe dl)
target_link_options(probe PRIVATE -Wl,--export-dynamic)
''')
with (out/'build.log').open('w') as log:
 subprocess.run(['cmake','-S',str(out),'-B',str(out/'build')],stdout=log,stderr=subprocess.STDOUT,check=True,timeout=60)
 subprocess.run(['cmake','--build',str(out/'build'),'-j2'],stdout=log,stderr=subprocess.STDOUT,check=True,timeout=120)
env=dict(os.environ,ROS_DOMAIN_ID='224')
with (out/'native.log').open('w') as log:r=subprocess.run([str(out/'build/probe')],env=env,stdout=log,stderr=subprocess.STDOUT,timeout=30)
(out/'manifest.json').write_text(json.dumps(dict(return_code=r.returncode,source_sha256=hashlib.sha256((out/'probe.cpp').read_bytes()).hexdigest(),installed_library_sha256=hashlib.sha256(Path('/autoware/install/ekf_localizer/lib/libekf_localizer_lib.so').read_bytes()).hexdigest(),meaning='Actual installed EKF timerCallback, deterministic in-process Clock::now injection; observation-only. Frozen clock control plus two5msmidcallbackclockadvances. Not production or full race attribution.'),indent=2)+'\n')
print((out/'native.log').read_text(),flush=True)
