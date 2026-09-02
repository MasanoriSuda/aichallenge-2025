# Design

## Trust boundary

`audit_e2e_submission_readiness.py` is the final evidence consumer.  It must
therefore validate report contents rather than accepting a self-declared
`status=pass`.

The auditor receives an explicit production runtime contract and checks it
against:

1. `expected_runtime` recorded by the competition analyzer;
2. every evaluated domain's parsed runtime provenance;
3. raw and spatial artifact SHA-256 values recorded by the report;
4. the independently hashed install and source artifacts.

The mixed-peer motion report is also bound to the competition Domain through
its SHA-256.  Each spatial report records the exact competition-report SHA it
consumed, preventing reports from different runs or analyzer invocations from
being combined accidentally.

This makes omission of a strict analyzer argument a rejection instead of an
accidental pass.

## Spatial execution evidence

Reuse `analyze_spatial_shadow_run.py`; do not duplicate log parsing.  Add a
domain selector so a mixed run can inspect the E2E student's domain rather
than assuming Domain 1.

Readiness validates the resulting report directly:

- schema and report status;
- domain identity and checkpoint SHA;
- production competition Gate status;
- runtime spatial configuration;
- coverage >= 0.99;
- error, non-ok, and stale counts are zero;
- authority is enabled in every reported interval and applied at least once.

Single spatial evidence is mandatory for any candidate.  Peer spatial
evidence is required only for multi-vehicle promotion; absence keeps the
classification single-only.

## Artifact identity

The paths passed as `--raw-checkpoint` and `--spatial-adapter` represent the
runtime install artifacts.  New source-artifact arguments prove that the
installed copies are the ones committed in the participant package.  All four
hashes must equal their respective frozen values.

## Compatibility

Only offline evidence tools, their tests, and submission documentation change.
ROS topics, services, launch defaults, packages, checkpoints, and authority
remain unchanged.
