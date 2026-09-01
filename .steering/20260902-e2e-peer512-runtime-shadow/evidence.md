# Evidence

## Runtime artifact

- source PyTorch SHA-256:
  `10297e9484537d3c63f014050a25162e989f4edc3f7f5359af6a2c0501180e57`
- self-described NumPy SHA-256:
  `b4b292e0223444c84bf85523d31d2c475386e7743416fc9d4eaff31dc7243830`
- runtime contract: schema 1, hidden 512, projection 128, no speed input
- recurrent authority: disabled

The first `v1` launch is invalid model evidence.  It failed before driving
because the old deployment boundary constructed a 64-wide runtime from YAML
before loading the 512-wide artifact.  Commit `deeee111` makes newly converted
artifacts own their complete numerical construction contract and preserves a
clearly logged compatibility path for legacy diagnostics.

## Closed-loop shadow Gate

- run: `output/20260902-e2e-peer512-runtime-shadow-v2`
- Finish: 3 / 3 laps, position 1 / 1
- lap times: `84.543 / 83.968 / 83.944 s`
- total / average: `252.455 / 84.152 s`
- penalties: 0
- post-start stall: 0 s
- distance: 1019.034 m
- mean / maximum speed: `3.802 / 4.461 m/s`
- result-summary SHA-256:
  `78888028621ae8887fa081f05bd138383f169797b11dcccbd32adab24ddda280`
- motion report SHA-256:
  `4e256a7d3d636a31f2337abbaa26e49f9d9a03a40f7e73c2c955d0338321dd57`

The motion and strict competition analyzers both pass.  The previous packaged
single-kart baselines totalled `255.648--256.488 s`; this run is not evidence of
runtime perturbation from the diagnostic model.

## Recurrent runtime Gate

- admitted / scans: `6544 / 6545`
- coverage: `99.9847%`
- inference errors: 0
- non-ok intervals: 0
- authority applications: 0
- average inference: `10.549 ms`
- maximum inference: `79.190 ms`
- minimum reported inference capacity: `57.62 Hz`
- minimum scan rate: `19.94 Hz`
- maximum hidden norm: `11.533`
- non-zero correction intervals: 39 / 65

One scan lacked a fresh wheel-speed sample.  Both the spatial and recurrent
diagnostics skipped that same scan; the recurrent hidden state was reset once
and normal inference resumed in the next interval.  This is the intended
fail-closed freshness behavior, not a hidden-state inference failure.  No stale
LiDAR or published-command stall accompanied it.

## Verification

- controller contract tests: 43 passed
- converter/parity and shadow-analyzer tests in Docker: 6 passed
- shadow-analyzer host tests: 3 passed
- `make autoware-build`: 25 packages completed
- `git diff --check`: passed

## Decision

Admit peer512 to a later multi-vehicle **interaction shadow** Slice.  This Gate
does not grant recurrent steering authority and does not package the artifact.
The remaining question is whether its material corrections align with peer
avoidance online; a single-kart run cannot answer that.
