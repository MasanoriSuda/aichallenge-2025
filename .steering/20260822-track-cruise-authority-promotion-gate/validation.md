# Validation status

## Static status at `5bae30b`

- Build: 25 packages passed.
- Complete package CTest: 35/35 passed.
- Test result: 1,571 tests, 0 errors, 0 failures, 0 skipped.
- Current-intent failure-first tests demonstrated and then rejected Track/Cruise cross-intent
  retained adoption and non-Track/Cruise current requests.
- Canonical plan actuation is read only by Track/Cruise shadow evaluation.
- The final command publisher remains unchanged.
- User-owned `aichallenge/result-summary.json` remains uncommitted and untouched.

## Dynamic status

Blocked on an external simulator run. No AWSIM or Autoware container was running when checked, and
starting the simulator crosses the external GUI-operation boundary defined for this autonomous
goal.

## Required next artifact

One clean Track/Cruise run at the current HEAD with `autoware.log` containing the new periodic line.
Prefer two or more laps so circular seam and warm-start continuity are represented. No opponent is
required for Gate A.
