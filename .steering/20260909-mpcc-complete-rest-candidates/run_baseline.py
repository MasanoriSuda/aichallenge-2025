import subprocess,sys
from pathlib import Path
s=Path('/task-steering/20260909-mpcc-complete-rest-candidates')
for name in ['replay_4264.py','compare_production.py','capture_free_stop.py']:
    r=subprocess.run([sys.executable,str(s/name)],timeout=600)
    print(name, r.returncode, flush=True)
    if r.returncode:sys.exit(r.returncode)
