from pathlib import Path
import hashlib,json,shutil,subprocess
out=Path('/output/20260909-ego-viability-ekf-installed');out.mkdir(exist_ok=False)
root=Path('/autoware/install/ekf_localizer')
files=list((root/'include/ekf_localizer').glob('*.hpp'))+[root/'share/ekf_localizer/package.xml',root/'lib/libekf_localizer_lib.so']
for p in files:shutil.copy2(p,out/p.name)
(out/'manifest.json').write_text(json.dumps(dict(source='Installed runtime EKF; no upstream source equivalence assumed',files={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}),indent=2)+'\n')
with (out/'symbols.log').open('w') as log:subprocess.run(['nm','-C',str(root/'lib/libekf_localizer_lib.so')],stdout=log,check=True)
for name,start,end in [('frequency','0xcf230','0xcf650'),('publish','0xcfa90','0xd32b0'),('timer','0xd32b0','0xd6100')]:
 with (out/(name+'.asm')).open('w') as log:subprocess.run(['objdump','-Cd','--start-address='+start,'--stop-address='+end,str(root/'lib/libekf_localizer_lib.so')],stdout=log,check=True)
print('Preserved installed EKF headers and binary',flush=True)
