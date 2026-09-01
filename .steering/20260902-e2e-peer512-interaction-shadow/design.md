# Design

Use `e2e-peer-audit-student`: domains 1 and 2 remain MPC peers and domain 3 is
the only TinyLidarNet student.  Pass the immutable self-described peer512
artifact and SHA through the existing launch boundary while leaving recurrent
authority false.

The recurrent shadow analyzer gains an explicit `--domain` selector.  It reads
that domain's logs and race detail but still requires the complete strict
competition report to pass, so another participant failure cannot be hidden by
examining only the student.

This Slice measures online execution and material outputs.  It cannot prove
that applying those outputs improves the race; bounded authority remains a
separate A/B only after this shadow Gate passes.

## Decision

The three-vehicle Gate rejects bounded recurrent authority.  The recurrent
artifact itself loaded with the expected immutable identity and produced no
inference exceptions, but synchronous shadow inference did not meet the
runtime contract under peer load.  Observation-only work took 30.91 ms on
average and 151.06 ms at the maximum, while admitted coverage fell to 95.74%
and the recurrent lifecycle reset 401 times.

No timeout, freshness, coverage or reset threshold is relaxed in response.
The next Slice must first isolate authority-disabled recurrent observation
from the production command critical path.  Authority-enabled recurrent
execution remains prohibited until that isolated path is separately
certified.
