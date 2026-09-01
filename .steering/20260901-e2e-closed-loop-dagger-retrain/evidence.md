# Evidence

## Split contract

The ignored `recurrent_direct_v4_trainonly` view contains ten train sequences:
the nine existing recurrent-v3 train sequences plus only the successful
authority run `/output/20260901-175609/d1/rosbag2_autoware`.  Its four
validation sequences are byte-identical to recurrent v3.

The failed NPC run `/output/20260901-180313/d1/rosbag2_autoware` exists only in
the recurrent-v4 audit validation split.  It therefore affects neither
gradients nor early stopping.

## Candidates

- v2 baseline SHA-256:
  `6ae9d618ea8093b1ff7d212cae760e90c71f84749f986af479681f5f729155d1`
- DAgger v3 SHA-256:
  `3b30f567d9a6bdf5384611ff8dfd759d79c8ed683c34e326e7d940afb2e67a5f`
- frozen base SHA-256:
  `de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa`

The v3 candidate uses the exact v2 architecture and training configuration.
It stopped at epoch 22 using the old validation split.  Its unbounded old-split
material MAE improvement increased from 0.301 to 0.319 and material direction
accuracy from 0.849 to 0.871.

## Same-split audit result

The persisted reports are:

- `output/20260901-e2e-dagger-v2-bounded.json`
- `output/20260901-e2e-dagger-v3-bounded.json`

All values below use the exact plus/minus 0.12 rad authority output after
runtime clipping.

| Scope | Metric | v2 | DAgger v3 |
|---|---|---:|---:|
| aggregate | material sign | 0.929 | 0.936 |
| aggregate | material MAE improvement | 0.255 | 0.270 |
| aggregate | attainable improvement utilization | 0.508 | 0.537 |
| held-out failed prefix | material sign | 0.938 | 0.959 |
| held-out failed prefix | material MAE improvement | 0.273 | 0.286 |
| held-out failed prefix | attainable improvement utilization | 0.603 | 0.632 |
| failed last 200 | material sign | 0.898 | 0.939 |
| failed last 200 | material MAE improvement | 0.322 | 0.332 |
| failed last 200 | attainable improvement utilization | 0.729 | 0.751 |
| independent normal | residual MAE rad | 0.00586 | 0.00670 |

The held-out failure improves despite never entering train or early stopping.
Missing closed-loop coverage is therefore a demonstrated contributor.  The
frozen spatial representation is not the immediate blocker.

## Certificate mismatch found

The previous evaluator accepted the unbounded model output, whose configured
support is plus/minus 1.2 rad.  Runtime authority publishes only the same output
clipped to plus/minus 0.12 rad.  Under the old acceptance threshold, both v2
and v3 pass before clipping and fail after clipping:

- v2 aggregate improvement: 0.305 unbounded, 0.255 runtime-bounded;
- v3 aggregate improvement: 0.332 unbounded, 0.270 runtime-bounded.

The evaluator now reports the exact clipped command, clipping fraction, and the
oracle performance attainable under the same bound.  This prevents an offline
pass from certifying a command the runtime cannot publish.

## Decision

Do not promote v3.  The DAgger hypothesis is supported, but the unchanged head
is trained against an unbounded teacher target and is then clipped by a
different runtime contract.  The next offline experiment must align the model
output support/training target with the existing 0.12 rad authority contract.
Only after that candidate passes the runtime-bounded audit may it return to
shadow and then limited authority.  No launch, runtime weight, authority flag,
or controller parameter changed in this slice.

## Verification

- focused evaluator and representation tests: 13 passed;
- embedded candidate3 identity: passed for both candidates;
- independent production-normal leakage: passed for both candidates;
- audit train/validation identities: disjoint;
- evaluator result at the runtime-bound gate: failed for both candidates, as
  required to prevent premature promotion.
