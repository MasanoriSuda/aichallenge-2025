"""Offline comparison of one positive objective unit; relative weights unchanged."""
import json
from pathlib import Path
import sys
import numpy as np
import osqp
from scipy.sparse import coo_matrix,diags,triu
import yaml
source=Path(sys.argv[1]);root=Path(sys.argv[2]);record=yaml.safe_load(source.read_text());d=record['exact_qp']
def mat(item):
    r,c,v=np.array(item['triplets']).T
    return coo_matrix((v,(r.astype(int),c.astype(int))),shape=(item['rows'],item['columns'])).tocsc()
P=mat(d['quadratic_cost']);A=mat(d['constraints']);q=np.array(d['linear_cost']);l=np.array(d['lower_bound']);u=np.array(d['upper_bound'])
D=np.array(d['variable_scaling']);Dmat=diags(D)
side=np.minimum(np.where(np.isfinite(l),np.abs(l),np.inf),np.where(np.isfinite(u),np.abs(u),np.inf));S=np.where(np.isfinite(side),1/(1+side),1)
P1=Dmat@P@Dmat;A1=diags(S)@A@Dmat;q1=D*q
unit=max(1.,float(np.max(np.abs(P1.data))),float(np.max(np.abs(q1))))
s=osqp.OSQP();s.setup(P=triu(P1/unit,format='csc'),q=q1/unit,A=A1.tocsc(),l=S*l,u=S*u,verbose=False,eps_abs=.001,eps_rel=0,scaling=0,max_iter=4000)
w=record['warm_start'];s.warm_start(x=np.array(w['primal'])/D,y=np.array(w['dual'])/S/unit)
a=s.solve();x=D*a.x;np.savetxt(root/'objective-units-primal.txt',x,fmt='%.17g')
values=A@x;v=np.maximum(0,np.maximum(l-values,values-u));r=int(np.argmax(v));report={'source':str(source),'osqp_version':osqp.__version__,'objective_unit':unit,'status':a.info.status,'iterations':a.info.iter,'solve_ms':a.info.solve_time*1000,'maximum_row_violation':float(v[r]),'maximum_row':r,'scope':'offline equivalent objective comparison; unchanged physical proof pending'}
(root/'objective-units.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
