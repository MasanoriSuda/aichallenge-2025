"""Exact rational dual certificate for the encoded affine rows, not the world."""
from fractions import Fraction
from pathlib import Path
import json
import math
import yaml

root = Path('output/20260909-side-peer-dev2-normal-qp')
lp = json.loads((root/'linear-feasibility.json').read_text())
data = yaml.load(Path(lp['source']).read_text(), Loader=yaml.CSafeLoader)
q = data['exact_qp']
f = lambda x: Fraction.from_float(float(x))
lower, upper = q['lower_bound'], q['upper_bound']
lo = [i for i,b in enumerate(lower) if math.isfinite(b)]
hi = [i for i,b in enumerate(upper) if math.isfinite(b)]
lo_index = {r:i for i,r in enumerate(lo)}
hi_index = {r:len(lo)+i for i,r in enumerate(hi)}
constraints = lo+hi
signs = [-1]*len(lo)+[1]*len(hi)
# Relax each finite side by its existing physical tolerance. Proving even
# this larger set empty cannot be repaired by OSQP initialization/scaling.
rhs = [f(-lower[i])+f(1e-3+1e-3*abs(lower[i])) for i in lo]
rhs += [f(upper[i])+f(1e-3+1e-3*abs(upper[i])) for i in hi]
y = [f(v) for v in lp['dual']]
assert all(v <= 0 for v in y)
rows = [{} for _ in lower]
for i,j,v in q['constraints']['triplets']:
    rows[i][j] = rows[i].get(j,Fraction(0))+f(v)
residual = [Fraction(0) for _ in q['linear_cost']]
for i,(r,sign) in enumerate(zip(constraints,signs)):
    for j,v in rows[r].items(): residual[j] += y[i]*sign*v
# The dynamics rows have a -I state diagonal and only previous-state/input
# terms. Use their finite lower/upper inequalities to cancel state residuals
# exactly, keeping every multiplier nonpositive. No equality is relaxed away.
state_count = 7*(q['horizon_steps']+1)
for j in reversed(range(state_count)):
    assert rows[j][j] == -1
    assert all(k <= j or k >= state_count for k in rows[j])
    assert lower[j] == upper[j] and math.isfinite(lower[j])
    delta = residual[j]
    if delta >= 0:
        y[lo_index[j]] -= delta
    else:
        y[hi_index[j]] += delta
    for k,v in rows[j].items(): residual[k] += delta*v
assert all(v == 0 for v in residual[:state_count])
assert all(v <= 0 for v in y)
maximum_left = Fraction(0)
assembly = data['assembly_request']
for j,c in enumerate(residual[state_count:]):
    l,u = assembly['input_lower'][j],assembly['input_upper'][j]
    box_row = 2*state_count+j
    assert rows[box_row] == {state_count+j: Fraction(1)}
    assert lower[box_row] == l and upper[box_row] == u
    assert math.isfinite(l) and math.isfinite(u)
    l = f(l)-f(1e-3+1e-3*abs(l))
    u = f(u)+f(1e-3+1e-3*abs(u))
    maximum_left += max(c*l,c*u)
minimum_right = sum((v*b for v,b in zip(y,rhs)),Fraction(0))
margin = minimum_right-maximum_left
checked_residual = [Fraction(0) for _ in residual]
for i,(r,sign) in enumerate(zip(constraints,signs)):
    for j,v in rows[r].items(): checked_residual[j] += y[i]*sign*v
assert checked_residual == residual
report = dict(source=lp['source'],source_sha256=lp['source_sha256'],
              scope='exact rational certificate for the encoded affine QP plus physical row tolerances; no physical-world infeasibility claim',
              certified_infeasible=margin>0, exact_state_stationarity=True,
              all_duals_nonpositive=True, dual_rhs=float(minimum_right),
              maximum_input_residual=float(maximum_left), contradiction_margin=float(margin),
              exact_margin=str(margin))
(root/'affine-infeasibility-certificate.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k!='exact_margin'},indent=2))
assert margin > 0
