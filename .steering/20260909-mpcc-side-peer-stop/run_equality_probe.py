"""Compare an objective identically zero on the existing equality feasible set."""
from pathlib import Path
import shlex
import subprocess

package=Path('/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros')
build=Path('/aichallenge/workspace/build/multi_purpose_mpc_ros')
steering=Path('/task-steering/20260909-mpcc-side-peer-stop')
out=Path('/output/20260909-side-peer-stop-audit/equality-comparison')
out.mkdir(exist_ok=False)
source=next((out.parent/'zero/mpcc_architecture_snapshots').glob('*/snapshot.yaml'))
link=shlex.split((build/'CMakeFiles/mpcc_architecture_compare.dir/link.txt').read_text())
command=[link[0],'-std=c++17','-O2','-I'+str(package/'include'),'-I/usr/include/eigen3',
         str(steering/'world_probe.cpp'),'-o',str(out/'probe')]+link[link.index('-o')+2:]
with (out/'build.log').open('w') as log:
    subprocess.run(command,cwd=build,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=90)
for mode in ['original','world']:
    work=out/mode
    work.mkdir()
    with (work/'probe.log').open('w') as log:
        subprocess.run([str(out/'probe'),str(source),str(work/'report.yaml'),mode],cwd=work,
                       stdout=log,stderr=subprocess.STDOUT,check=True,timeout=120)
    print(work/'report.yaml',flush=True)
