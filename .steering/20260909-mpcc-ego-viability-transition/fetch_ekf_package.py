from pathlib import Path
import concurrent.futures,hashlib,json,urllib.request
commit='1e90380a3570d02e16d40287e0adf51a27cf34d0'
root=Path('output/20260909-ego-viability-ekf-source-package');root.mkdir(exist_ok=False)
def get(url):return urllib.request.urlopen(urllib.request.Request(url,headers={'User-Agent':'MPCC-local-causal-audit'}),timeout=30).read()
queue=['localization/ekf_localizer'];files=[]
while queue:
 path=queue.pop()
 listing=json.loads(get('https://api.github.com/repos/autowarefoundation/autoware_universe/contents/'+path+'?ref='+commit))
 for item in listing:
  if item['type']=='dir':
   if item['name'] not in ['media','docs']:queue.append(item['path'])
  elif item['type']=='file':files.append(item)
assert len(files)<100,len(files)
def fetch(item):
 b=get(item['download_url']);rel=Path(item['path']).relative_to('localization/ekf_localizer');p=root/rel;p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(b)
 return dict(path=str(rel),url=item['download_url'],sha256=hashlib.sha256(b).hexdigest())
with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:rows=list(pool.map(fetch,files))
(root/'upstream-manifest.json').write_text(json.dumps(dict(commit=commit,files=rows,limitation='Installed ekf_localizer.hpp matches exactly and installed timer clock behaviour is independently reproduced; no claim every compiler artifact is bit-identical.'),indent=2)+'\n')
print('Downloaded official historical package files:',len(rows),flush=True)
