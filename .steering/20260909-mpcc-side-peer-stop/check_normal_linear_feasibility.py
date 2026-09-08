"""Independent LP diagnostic of the saved affine QP, never physical authority."""
from pathlib import Path
import hashlib
import json
import numpy as np
from scipy.optimize import linprog
from scipy.sparse import coo_matrix, hstack, vstack
import yaml

path = next(Path('output/20260909-side-peer-stop-dev2-r1/d2/mpcc_architecture_snapshots').glob('000000000381*/snapshot.yaml'))
data = yaml.load(path.read_text(), Loader=yaml.CSafeLoader)
qp = data['exact_qp']
m = qp['constraints']
t = np.asarray(m['triplets'])
a = coo_matrix((t[:, 2], (t[:, 0].astype(int), t[:, 1].astype(int))),
               shape=(m['rows'], m['columns'])).tocsr()
lower, upper = np.asarray(qp['lower_bound']), np.asarray(qp['upper_bound'])
lo, hi = np.isfinite(lower), np.isfinite(upper)
inequality = vstack([-a[lo], a[hi]]).tocsr()
rhs = np.r_[-lower[lo], upper[hi]]
# Minimize maximum violation measured in the existing physical row tolerance.
# This is diagnostic slack only; no candidate or relaxed row enters MPCC.
tolerance = 1e-3 + 1e-3 * np.abs(rhs)
extended = hstack([inequality, -tolerance[:, None]]).tocsr()
objective = np.r_[np.zeros(a.shape[1]), 1.0]
result = linprog(objective, A_ub=extended, b_ub=rhs,
                 bounds=[(None, None)]*a.shape[1]+[(0, None)], method='highs')
report = dict(source=str(path), source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
              status=int(result.status), message=result.message,
              scope='affine feasibility diagnostic; does not prove physical infeasibility')
if result.success:
    residual = (inequality@result.x[:-1]-rhs)/tolerance
    report.update(minimum_maximum_normalized_violation=float(result.x[-1]),
                  independently_measured_maximum=float(max(0, residual.max())),
                  dual_lower_bound=float(rhs@result.ineqlin.marginals),
                  dual_stationarity_error=float(np.max(np.abs(extended.T@result.ineqlin.marginals+
                    result.lower.marginals+result.upper.marginals-objective))),
                  primal=result.x[:-1].tolist(), dual=result.ineqlin.marginals.tolist())
out = Path('output/20260909-side-peer-dev2-normal-qp/linear-feasibility.json')
out.write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k not in ('primal','dual')}, indent=2))
