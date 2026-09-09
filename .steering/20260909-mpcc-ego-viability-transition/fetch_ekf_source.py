from pathlib import Path
import concurrent.futures,hashlib,json,urllib.request
out=Path('output/20260909-ego-viability-ekf-upstream')
commits=['3fd7b7c542baea2de5963efc049efe51f29ad41f','751f128e7a8a6d224f6c44f235e130444eb81130','1e90380a3570d02e16d40287e0adf51a27cf34d0']
paths=['include/ekf_localizer/ekf_localizer.hpp','src/ekf_localizer.cpp']
expected=hashlib.sha256(Path('output/20260909-ego-viability-ekf-installed/ekf_localizer.hpp').read_bytes()).hexdigest()
def read(item):
 commit,path=item
 url='https://raw.githubusercontent.com/autowarefoundation/autoware_universe/'+commit+'/localization/ekf_localizer/'+path
 b=urllib.request.urlopen(urllib.request.Request(url,headers={'User-Agent':'MPCC-local-causal-audit'}),timeout=30).read()
 p=out/commit/Path(path).name;p.parent.mkdir(exist_ok=True);p.write_bytes(b)
 row=dict(commit=commit,url=url,path=str(p),sha256=hashlib.sha256(b).hexdigest())
 if path.endswith('.hpp'):row['matches_installed_header']=row['sha256']==expected
 return row
with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:rows=list(pool.map(read,[(c,p) for c in commits for p in paths]))
(out/'source-comparison.json').write_text(json.dumps(dict(expected_installed_header_sha256=expected,files=rows),indent=2)+'\n')
for r in rows:print(r,flush=True)
