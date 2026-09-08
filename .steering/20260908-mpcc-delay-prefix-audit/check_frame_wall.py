"""Run only in the existing Compose development image after the race stops."""
from pathlib import Path
import shlex
import subprocess
import sys

package = Path('/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros')
build = Path('/aichallenge/workspace/build/multi_purpose_mpc_ros')
run = Path(sys.argv[1])
out = run/'executed-replay'
link = shlex.split((build/'CMakeFiles/mpcc_architecture_compare.dir/link.txt').read_text())
command = [link[0], '-std=c++17', '-O2', '-I'+str(package/'include'),
           '-I/usr/include/eigen3', '/task-steering/20260908-mpcc-delay-prefix-audit/check_frame_wall.cpp',
           '-o', str(out/'check_frame_wall')] + link[link.index('-o')+2:]
with (out/'frame-wall-build.log').open('w') as log:
    subprocess.run(command, cwd=build, stdout=log, stderr=subprocess.STDOUT, check=True)
snapshot = next((run/'d1/mpcc_architecture_snapshots').glob('000000006782*/snapshot.yaml'))
with (out/'frame-wall-comparison.json').open('w') as result:
    subprocess.run([str(out/'check_frame_wall'), str(snapshot), str(out/'frame-comparison.json')],
                   stdout=result, check=True)
