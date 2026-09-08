"""Compile a native before-repair observation invariant using existing CMake flags."""
from pathlib import Path
import hashlib
import json
import shlex
import subprocess

out=Path('/output/20260909-failure-bundle-before');out.mkdir(exist_ok=False)
build=Path('/aichallenge/workspace/build/multi_purpose_mpc_ros')
cmake=build/'CMakeFiles/test_mpcc_architecture_snapshot.dir'
flags={}
for line in (cmake/'flags.make').read_text().splitlines():
    if ' = ' in line:
        key,value=line.split(' = ',1);flags[key]=shlex.split(value)
source=Path(__file__).with_suffix('.cpp')
object_file=out/'probe.o'
with (out/'build.log').open('w') as log:
    subprocess.run(['/usr/bin/c++',*flags['CXX_DEFINES'],*flags['CXX_INCLUDES'],
        *flags['CXX_FLAGS'],'-c',str(source),'-o',str(object_file)],cwd=build,
        stdout=log,stderr=subprocess.STDOUT,check=True)
    command=shlex.split((cmake/'link.txt').read_text())
    command=[str(object_file) if value.endswith('test_mpcc_architecture_snapshot.cpp.o') else value for value in command]
    command[command.index('-o')+1]=str(out/'probe')
    subprocess.run(command,cwd=build,stdout=log,stderr=subprocess.STDOUT,check=True)
with (out/'test.log').open('w') as log:
    result=subprocess.run([str(out/'probe'),'--gtest_filter=AuthorityFailureBundleBefore.*'],
        stdout=log,stderr=subprocess.STDOUT)
(out/'manifest.json').write_text(json.dumps(dict(
    expected_result='one failing invariant: second generic source record is Duplicate',
    return_code=result.returncode,source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
    library_sha256=hashlib.sha256((build/'libmulti_purpose_mpc_ros_mpcc_architecture_snapshot.a').read_bytes()).hexdigest()),indent=2)+'\n')
print((out/'test.log').read_text())
assert result.returncode == 1
