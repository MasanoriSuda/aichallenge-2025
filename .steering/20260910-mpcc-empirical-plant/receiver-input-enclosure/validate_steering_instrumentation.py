"""Compare original CIL with probe-stripped copy, including branch/EH targets."""
from pathlib import Path
import hashlib,json,math
import dnfile
from dncil.cil.body.reader import read_method_body_from_bytes
original=Path('aichallenge/simulator/AWSIM/AWSIM_Data/Managed/Assembly-CSharp.dll')
out=Path('output/20260911-steering-instrumentation-r2')

def decode(path,strip):
 p=dnfile.dnPE(str(path));owners={}
 for t in p.net.mdtables.TypeDef:
  name=str(t.TypeNamespace)+'.'+str(t.TypeName)
  for m in t.MethodList:owners[id(m.row)]=name
  for f in t.FieldList:owners[id(f.row)]=name
 def signature(data):
  pos=0
  def number():
   nonlocal pos
   first=data[pos];pos+=1
   if first<0x80:return first
   if first<0xc0:
    value=((first&0x3f)<<8)|data[pos];pos+=1;return value
   value=((first&0x1f)<<24)|(data[pos]<<16)|(data[pos+1]<<8)|data[pos+2];pos+=3;return value
  def coded():
   value=number();tid=[2,1,27][value&3]
   return rowname(p.net.mdtables.tables[tid].rows[(value>>2)-1])
  def dtype():
   nonlocal pos
   kind=data[pos];pos+=1
   if kind in [0x11,0x12]:return (kind,coded())
   if kind in [0x13,0x1e]:return (kind,number())
   if kind in [0x0f,0x10,0x1d,0x45]:return (kind,dtype())
   if kind in [0x1f,0x20]:return (kind,coded(),dtype())
   if kind==0x15:
    base=dtype();return (kind,base,[dtype() for _ in range(number())])
   if kind==0x14:
    base=dtype();rank=number();sizes=[number() for _ in range(number())];low=[number() for _ in range(number())]
    return (kind,base,rank,sizes,low)
   if kind==0x1b:return (kind,msig())
   if kind==0x41:return (kind,dtype())
   assert kind in list(range(1,15))+[0x16,0x18,0x19,0x1c],('unknown element',kind,data.hex())
   return kind
  def msig():
   nonlocal pos
   call=data[pos];pos+=1
   arity=number() if call&0x10 else 0
   count=number();ret=dtype();return (call,arity,ret,[dtype() for _ in range(count)])
  # TypeSpec starts directly with an element type; the other two callers
  # below explicitly select the signature category.
  value=dtype()
  assert pos==len(data),('unconsumed signature',data.hex(),pos)
  return repr(value)
 def instantiation(data):
  # A MethodSpec signature is genericinst(0x0a), argument count, argument
  # types. Reuse the TypeSpec GENERICINST parser with an existing object
  # class token as a stable dummy constructor, then remove that dummy.
  assert data[0]==0x0a
  # Direct parser framing: GENERICINST, CLASS, TypeDef index1, then count/types.
  return signature(bytes([0x15,0x12,0x04])+data[1:])
 def rowname(row):
  if hasattr(row,'TypeName'):return str(row.TypeNamespace)+'.'+str(row.TypeName)
  if hasattr(row,'Name'):
   return (owners.get(id(row)) or rowname(row.Class.row))+':'+str(row.Name)
  if hasattr(row,'Method'):return rowname(row.Method.row)+':generic:'+instantiation(row.Instantiation.value)
  if hasattr(row,'Signature'):return 'signature:'+signature(row.Signature.value)
  return str(type(row))
 def operand(x):
  if hasattr(x,'value'):
   tid,index=x.value>>24,x.value&0xffffff
   if tid==0x70:return ('string',p.net.user_strings.get(index).value)
   return ('token',rowname(p.net.mdtables.tables[tid].rows[index-1]))
  if isinstance(x,float) and math.isnan(x):return ('float','nan')
  if isinstance(x,(int,float,str)) or x is None:return x
  if isinstance(x,list):return x
  return str(x)
 result={};probes=[]
 for t in p.net.mdtables.TypeDef:
  for index,method in enumerate(t.MethodList):
   m=method.row;key=str(t.TypeNamespace)+'.'+str(t.TypeName)+':'+str(m.Name)+'#'+str(index)
   if not m.Rva:result[key]=None;continue
   b=read_method_body_from_bytes(p.get_data(m.Rva));ins=b.instructions
   remove=set()
   if strip:
    for j,i in enumerate(ins):
     target=operand(i.operand)
     if i.mnemonic=='call' and isinstance(target,tuple) and 'MPCCInputObservation:' in str(target):
      name=target[1].split(':')[-1];count={'Receive':5,'Choose':4,'Apply':8,'Steer':5}[name]
      start=j-count+1
      remove.update(range(start,j+1));probes.append(dict(method=key,name=name,offset=i.offset,instructions=count))
   kept=[i for j,i in enumerate(ins) if j not in remove];loc={i.offset:j for j,i in enumerate(kept)}
   loc[b.header_size+b.code_size]=len(kept)
   def target(x):
    if x not in loc:raise AssertionError((key,'branch/EH into removed probe',x))
    return loc[x]
   rows=[]
   for i in kept:
    op=i.mnemonic
    kind=str(i.opcode.operand_type)
    value=operand(i.operand)
    if 'BrTarget' in kind:
     value=('branch',target(i.operand));op=op.removesuffix('.s')
    if op=='switch':value=('switch',[target(x) for x in i.operand])
    rows.append((op,value))
   eh=[]
   for e in b.exception_handlers:
    eh.append(dict(kind=e.exception_type,try_start=target(e.try_start+b.header_size),try_end=target(e.try_end+b.header_size),handler_start=target(e.handler_start+b.header_size),handler_end=target(e.handler_end+b.header_size),filter_start=target(e.filter_start+b.header_size) if e.filter_start>=0 else -1,catch_type=operand(e.catch_type)))
   result[key]=dict(instructions=rows,exception_handlers=eh)
 return result,probes
old,_=decode(original,False);new,probes=decode(out/'Assembly-CSharp.dll',True)
diff=[]
for key in old.keys()|new.keys():
 if old.get(key)!=new.get(key):
  a=old.get(key);b=new.get(key)
  item=dict(method=key)
  if a and b:
   item['first_differences']=[dict(index=i,old=x,new=y) for i,(x,y) in enumerate(zip(a['instructions'],b['instructions'])) if x!=y][:4]
   item['counts']=[len(a['instructions']),len(b['instructions'])]
   item['eh_equal']=a['exception_handlers']==b['exception_handlers']
  diff.append(item)
report=dict(original=str(original),original_sha256=hashlib.sha256(original.read_bytes()).hexdigest(),instrumented_sha256=hashlib.sha256((out/'Assembly-CSharp.dll').read_bytes()).hexdigest(),methods=len(old),probe_calls=probes,differences=diff,meaning='Static decoded CIL equivalence after removing five observation calls; branch and exception-handler target indices checked. Does not prove identical runtime timing.')
(out/'cil-validation-r3.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(dict(methods=len(old),probes=probes,differences=diff[:10],difference_count=len(diff)),indent=2))
assert not diff
assert [x['name'] for x in probes].count('Apply')==2 and len(probes)==5
