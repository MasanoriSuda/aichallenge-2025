# Tasks

- [x] Confirm baseline, user artifacts and prior native/saved-source failures.
- [x] Compare full physical coordinate formulations and choose bounded producer scope.
- [x] Add native regression for stationary/moving body, both curvatures, actual course knots, nu changes and Jacobians.
- [x] Implement one coordinate-consistent physical transition and remove the old law.
- [x] Bind immutable geometry to every solver/artifact/Stop/retained/replay producer and reject old model identity.
- [x] Run focused/package tests, full build and sealed fixed-input comparison.
  - Build25packages and60CTestgroups pass. Fixed-input physical integration agrees; new solves reject wall rows.
  - Two geometry-lineage regressions fail before repair and pass after; normal/continuation/Stop reference binding passes.
  - Final25-package build and60CTestgroups/2335colconrecords pass without C++ compiler warnings.
- [ ] Check measured command/sensor history at the original delay-prefix boundary.
- [x] Run bounded single-vehicle dynamic acceptance with sealed inputs and final authority telemetry.
  - Initial6laps252.253662109375s/penalty0; active moving override0/Recovery0, callbackmax20.390ms/overrun0.
  - Receiptmax39.862633ms/0gaps>50ms; source duplicates coincide with simulator clock/sensor pause.
  - This run precedes the additional geometry-lineage checks; final integrated campaign remains required.
- [ ] Update registry, current status and continue peer geometry/integration/submission phases.
