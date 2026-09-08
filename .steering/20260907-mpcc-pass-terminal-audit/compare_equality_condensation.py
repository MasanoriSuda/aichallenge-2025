"""Offline equality-nullspace QP comparison; expand and verify original rows."""
import json
from pathlib import Path
import sys
import numpy as np
import osqp
from scipy.linalg import null_space
from scipy.sparse import coo_matrix,csc_matrix,triu
import yaml

source=Path(sys.argv[1]); root=Path(sys.argv[2])
d=yaml.safe_load(source.read_text())['exact_qp']
def mat(item):
    r,c,v=np.array(item['triplets']).T
    return coo_matrix((v,(r.astype(int),c.astype(int))),shape=(item['rows'],item['columns'])).tocsc()
P=mat(d['quadratic_cost']); P=P+P.T-csc_matrix(np.diag(P.diagonal())) if (P-P.T).nnz else P
A=mat(d['constraints']); q=np.array(d['linear_cost']); l=np.array(d['lower_bound']); u=np.array(d['upper_bound'])
equal=(l==u)&np.isfinite(l)
E=A[equal].toarray(); f=l[equal]
origin=np.linalg.lstsq(E,f,rcond=None)[0]; Z=null_space(E)
assert np.max(np.abs(E@origin-f))<1e-9
R=A[~equal]@Z; shift=A[~equal]@origin
strict_bound=np.minimum(np.where(np.isfinite(l),np.abs(l),np.inf), np.where(np.isfinite(u),np.abs(u),np.inf))
row_scale=np.where(np.isfinite(strict_bound),1/(1+strict_bound),1)[~equal]
R=row_scale[:,None]*R
s=osqp.OSQP(); s.setup(P=triu(csc_matrix(Z.T@P@Z),format='csc'),q=Z.T@(P@origin+q),A=csc_matrix(R),l=row_scale*(l[~equal]-shift),u=row_scale*(u[~equal]-shift),verbose=False,eps_abs=.001,eps_rel=0,scaling=0,max_iter=4000)
a=s.solve(); report={'source':str(source),'osqp_version':osqp.__version__,'full_dimension':A.shape[1],'reduced_dimension':Z.shape[1],'status':a.info.status,'iterations':a.info.iter,'solve_ms':a.info.solve_time*1000,'row_policy':'original physical row tolerance, eps_rel=0, no internal equilibration','scope':'offline comparison; canonical original-row and physical proof required'}
if a.x is not None:
    x=origin+Z@a.x; np.savetxt(root/'equality-condensed-primal.txt',x,fmt='%.17g')
    values=A@x; violations=np.maximum(0,np.maximum(l-values,values-u)); row=int(np.argmax(violations))
    report.update(maximum_row_violation=float(violations[row]),maximum_row=row,equality_residual=float(np.max(np.abs(E@x-f))))
(root/'equality-condensed.json').write_text(json.dumps(report,indent=2)+'\n'); print(json.dumps(report,indent=2))
