# Validation

## Failure-first evidence

Before implementation, `make autoware-build` failed in
`test_mpcc_execution_contract.cpp` because the following contract did not exist:

- `PhysicalWallCertificateReason`;
- `PhysicalWallCertificateDiagnostic`;
- the stable reason-name function;
- the deterministic diagnostic formatter.

This fixed the test-to-change mapping before the production source was modified.

## Static validation

- Focused `ctest -R test_mpcc_execution_contract`: pass.
- Full package test suite: 33/33 passed in 18.90 s.
- `make autoware-build`: passed after implementation. The first dynamic launch was discarded
  because it overlapped the still-running build container and therefore used the preceding binary.
- Installed build artifact was checked for the new `physical_rejects` diagnostic before the valid
  replay was launched.

After the final swept-stage heading provenance correction:

- `make autoware-build`: pass, 25 packages in 4 min 36 s;
- focused `test_mpcc_execution_contract`: pass in 0.11 s;
- full package suite: 33/33 pass in 17.34 s;
- `git diff --check`: pass.

## Dynamic evidence

Valid run: `output/20260822-135649`

- Single vehicle, four observed waypoint wraparounds.
- 7,579 eligible / solved / physically checked cycles.
- 7,491 physically certified cycles (98.8389%).
- 88 physical rejects:
  - invalid input: 0;
  - lateral bound violation: 0;
  - heading unavailable: 0;
  - wall sample unavailable: 0;
  - hard wall contact: 67;
  - swept current-to-horizon violation: 21.
- 7,491 certified actuation proposals joined; join rejects: 0.
- `selected=1`: 0. Every proposal remained `authority=shadow, selected=0`.
- Production callback: 8,177 cycles, 6.163 ms weighted mean, 29.364 ms maximum, one isolated
  25 ms period overrun.

Status-transition evidence retained ten first-failure records. Nine were direct footprint contact
and one was a first-segment swept violation. Their QP lateral-bound reserves were 0.925 m minimum,
1.143 m median and 1.511 m maximum; nine of ten failed at stage 0 and one at stage 1. Recurrent
failure waypoints included 258--262 and 80--83.

## Causal conclusion

The remaining rejects are not solver-bound tolerance failures. A solution can retain roughly one
metre of centre-coordinate bound reserve while the yawed 2.0 m x 1.45 m kart footprint intersects
the wall grid. The QP and the final certificate therefore use different geometry contracts:

1. the QP admits the vehicle centre inside scalar Frenet bounds;
2. the certificate validates the complete yawed footprint;
3. the swept certificate additionally tests the segment from the actual current pose to stage 0.

The next slice must first derive heading-aware, footprint-safe stage bounds in shadow and compare
them with the existing scalar bounds. It must not suppress the certificate or promote authority.

## Completion

All static and dynamic acceptance criteria pass. The unrelated user modification
`aichallenge/result-summary.json` remains outside this slice and must not be committed.
