import subprocess,sys
for name in ['compare_domain.py','capture_domain_free.py']:
 r=subprocess.run([sys.executable,'/task-steering/20260909-mpcc-complete-rest-candidates/'+name],timeout=600)
 print(name,r.returncode,flush=True)
 r.check_returncode()
