from pathlib import Path
import shlex,subprocess,sys,json,hashlib
p=Path('/aichallenge/workspace/src/aichallenge_submit/racing_kart_gnss_poser')
b=Path('/aichallenge/workspace/build/racing_kart_gnss_poser')
out=Path('/output/20260909-exact-join-gnss-heading-'+sys.argv[1]);out.mkdir(exist_ok=False)
flags=(b/'CMakeFiles/test_gnss_covariance.dir/flags.make').read_text()
def flag(name):return shlex.split(next(line.split('=',1)[1] for line in flags.splitlines() if line.startswith(name+' =')))
link=shlex.split((b/'CMakeFiles/test_gnss_covariance.dir/link.txt').read_text())
cmd=[link[0]]+flag('CXX_DEFINES')+flag('CXX_INCLUDES')+flag('CXX_FLAGS')+['-I/opt/ros/humble/src/gtest_vendor/include',str(p/'test/test_gnss_heading.cpp'),'-o',str(out/'test')]+link[link.index('-o')+2:]
with (out/'build.log').open('w') as log:r=subprocess.run(cmd,cwd=b,stdout=log,stderr=subprocess.STDOUT,timeout=120)
print((out/'build.log').read_text(),flush=True);r.check_returncode()
with (out/'native.log').open('w') as log:r=subprocess.run([str(out/'test'),'--gtest_output=xml:'+str(out/'results.xml')],cwd=b,stdout=log,stderr=subprocess.STDOUT,timeout=120)
(out/'manifest.json').write_text(json.dumps(dict(command=cmd,return_code=r.returncode,baseline_commit='466bba5e',source_sha256=hashlib.sha256((p/'src/gnss_poser_core.cpp').read_bytes()).hexdigest(),binary_sha256=hashlib.sha256((b/'libgnss_poser_node.so').read_bytes()).hexdigest()),indent=2)+'\n')
print((out/'native.log').read_text(),flush=True)
