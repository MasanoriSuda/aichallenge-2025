"""Independent feasibility check of serialized affine rows; no physical claim."""
import json
from pathlib import Path
import sys
import numpy as np
from scipy.optimize import linprog
from scipy.sparse import coo_matrix, vstack
import yaml

source = Path(sys.argv[1])
out = Path(sys.argv[2])
d = yaml.safe_load(source.read_text())['exact_qp']
r, c, values = np.array(d['constraints']['triplets']).T
A = coo_matrix((values, (r.astype(int), c.astype(int))), shape=(d['constraints']['rows'], d['constraints']['columns'])).tocsr()
l, u = np.array(d['lower_bound']), np.array(d['upper_bound'])
equal = (l == u) & np.isfinite(l)
lo, hi = np.isfinite(l) & ~equal, np.isfinite(u) & ~equal
answer = linprog(np.zeros(A.shape[1]), A_ub=vstack([A[hi], -A[lo]]), b_ub=np.r_[u[hi], -l[lo]], A_eq=A[equal], b_eq=l[equal], bounds=[(None,None)]*A.shape[1], method='highs', options={'time_limit':30})
report = {'source':str(source), 'status':answer.status, 'success':answer.success, 'message':answer.message, 'scope':'exact serialized affine QP only; no nonlinear/physical infeasibility inference'}
if answer.success:
    np.savetxt(out.with_suffix('.txt'), answer.x, fmt='%.17g')
    value=A@answer.x
    report['maximum_row_violation']=float(np.maximum(0,np.maximum(l-value,value-u)).max())
out.write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
