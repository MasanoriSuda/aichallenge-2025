# Results: async terminal failure snapshot

## Verification

- `make autoware-build`: passed.
- `multi_purpose_mpc_ros`: 2,248 tests, 0 failures.
- Dynamic run: `output/20260831-011806`.

## Dynamic evidence

The control-thread snapshot region fell from the frozen 25.583 / 68.999 ms
tail to 0.002 ms in observed slow cycles.

The worker later emitted:

```text
Rate-resolved terminal proof snapshot recorded asynchronously:
decision=1926, intent=cruise, file=.../snapshot.yaml
```

The corresponding replay file exists under
`output/20260831-011806/d2/mpcc_architecture_snapshots/` and is readable.
Therefore the diagnostic evidence was preserved without blocking normal
authority publication.

The same run completed:

```text
ShiftOut -> Pass -> Return -> Idle
```

for d1 episode 1.

## Remaining measured tail

After removing diagnostic I/O, occasional slow cycles remain entirely inside
one primary current-world retained proof:

- d2 decision 1631: 21.372 ms primary retained, 0.002 ms snapshot capture.
- d1 decision 3330: 29.325 ms primary retained, 0.002 ms snapshot capture.

This is now a separate physical/dynamic proof-runtime concern.  It is no
longer obscured by synchronous filesystem I/O and must be investigated as a
new Slice rather than extending this fix.

## Conclusion

The root cause addressed by this Slice was observability ownership: a
diagnostic writer executed on the control thread.  One bounded latest-only
worker now owns persistence.  No authority, solver, Mission, wall or
clearance behavior changed.
