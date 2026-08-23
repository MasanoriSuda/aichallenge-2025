# Follow effective-gap constraint design

## Causal chain

```text
QP constrains theta only
  -> e_lag may move physical pose forward
  -> theta gap passes while physical longitudinal gap fails
  -> obstacle_clear cannot be certified honestly
  -> Follow canonical authority cannot be promoted
```

## Constraint

For every predicted state `k`:

```text
e_lag[k] + theta[k] <= target_theta[k] - hard_gap
```

Both variables and the bound are in metres, so no new unit scaling is introduced. Existing theta box remains
temporarily unchanged to avoid broadening the feasible set in the same Slice; the new row only closes the
positive-lag hole.

## Provenance

The row block is appended after curvature-rate rows and decoded as `follow-effective-gap`. Non-Follow
problems do not allocate the block, preserving the existing Track/Cruise sparse layout and warm-start identity.

## Post-solve certificate

Telemetry computes:

```text
effective_gap = target_progress - (solved_progress + solved_lag)
```

and records the minimum gap and maximum hard-gap violation. This duplicates the equation as a certificate,
not as a second correction path.
