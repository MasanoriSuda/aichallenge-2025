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
