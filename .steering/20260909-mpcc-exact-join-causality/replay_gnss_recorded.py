from pathlib import Path
import shlex,subprocess,json,hashlib
s=Path(__file__).resolve().parent
p=Path('/aichallenge/workspace/src/aichallenge_submit/racing_kart_gnss_poser')
b=Path('/aichallenge/workspace/build/racing_kart_gnss_poser')
out=Path('/output/20260909-exact-join-gnss-replay');out.mkdir(exist_ok=False)
flags=(b/'CMakeFiles/test_gnss_covariance.dir/flags.make').read_text()
def flag(name):return shlex.split(next(line.split('=',1)[1] for line in flags.splitlines() if line.startswith(name+' =')))
link=shlex.split((b/'CMakeFiles/test_gnss_covariance.dir/link.txt').read_text())
cmd=[link[0]]+flag('CXX_DEFINES')+flag('CXX_INCLUDES')+flag('CXX_FLAGS')+[str(s/'replay_gnss_recorded.cpp'),'-o',str(out/'replay')]+link[link.index('-o')+2:]+['-lyaml-cpp']
with (out/'build.log').open('w') as log:r=subprocess.run(cmd,cwd=b,stdout=log,stderr=subprocess.STDOUT,timeout=120)
print((out/'build.log').read_text(),flush=True);r.check_returncode()
results=[]
for domain in ['d1','d2']:
    source=Path('/output/20260909-exact-join-gnss-inputs')/(domain+'.json')
    command=[str(out/'replay'),str(source),str(out/(domain+'.yaml'))]
    with (out/(domain+'.log')).open('w') as log:r=subprocess.run(command,cwd=b,stdout=log,stderr=subprocess.STDOUT,timeout=120)
    print(domain,r.returncode,(out/(domain+'.log')).read_text(),flush=True)
    results.append(dict(domain=domain,command=command,source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),return_code=r.returncode));r.check_returncode()
(out/'manifest.json').write_text(json.dumps(dict(command=cmd,results=results,node_binary_sha256=hashlib.sha256((b/'libgnss_poser_node.so').read_bytes()).hexdigest(),authority=False),indent=2)+'\n')
