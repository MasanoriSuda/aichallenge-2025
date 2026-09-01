# Design

Use the same deterministic `e2e-single` scenario as the successful v3 shadow
run.  Set the v3 shadow checkpoint explicitly and enable the existing bounded
authority flag.  The controller adds the correction only after fresh spatial
inference and clips it to plus/minus `0.12 rad`.

After the run, evaluate motion, Finish/penalties, runtime provenance and
authority statistics with the existing analyzers.  If rejected, freeze the
result and extract a causal pre-failure sequence before changing training or
runtime policy.
