from pathlib import Path
import hashlib,json,shlex,subprocess
s=Path(__file__).resolve().parent
p=Path('/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros')
b=Path('/aichallenge/workspace/build/multi_purpose_mpc_ros')
out=Path('/output/20260909-peer-epochs-motion-r1');out.mkdir(exist_ok=False)
link=shlex.split((b/'CMakeFiles/test_v2x_overtake_core.dir/link.txt').read_text())
deps=[x for x in link[link.index('-o')+2:] if 'gtest' not in x]
cmd=[link[0],'-std=c++17','-O2','-I'+str(p/'include'),'-I/usr/include/eigen3',str(s/'replay_motion.cpp'),'-o',str(out/'replay')]+deps+['-lyaml-cpp']
with (out/'build.log').open('w') as log:subprocess.run(cmd,cwd=b,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=120)
for domain in ['d1','d2']:
    with (out/(domain+'.log')).open('w') as log:
        subprocess.run([str(out/'replay'),'/output/20260909-peer-epochs-inputs-r1/'+domain+'.json',str(out/(domain+'.yaml'))],cwd=b,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=120)
(out/'manifest.json').write_text(json.dumps(dict(command=cmd,source_sha256=hashlib.sha256((s/'replay_motion.cpp').read_bytes()).hexdigest(),kernel_source_sha256=hashlib.sha256((p/'src/v2x_overtake_core.cpp').read_bytes()).hexdigest(),authority=False,limits='Actual filter kernel; read-only wrapper replays advancing/identical source samples, not the entire live tracker validity/receipt state. Gains 0.35/0.25 and bound 3.0 are unchanged production config. Match captured output before causal inference.'),indent=2)+'\n')
print('Motion replay complete',flush=True)
