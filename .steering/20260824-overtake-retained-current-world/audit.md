# Audit record

## Observed fact

The preceding progress-coupled wall Slice produced 794 complete canonical
Overtake selections from 794 candidates that reached physical certification,
with zero hard-wall-contact rejection in the deterministic replay.

## Remaining defect

`evaluate_overtake_canonical_fresh_shadow()` returns a complete
`CanonicalNormalSelection`, but `get_control()` records it and then discards it.
The old extended-to-legacy conversion and three-state solve continue to own the
published command.

## Why production promotion is deferred

The same replay still contained 11 extended-solver failures in 832 attempts and
circuit-breaker skip intervals. Fresh-only promotion would convert those
intervals to repeated emergency stops. The migration rule forbids authority
promotion without dynamic evidence for the retained same-formulation path.

## Hypothesis under test

A recently certified five-state Overtake plan can cover bounded fresh-solve
misses without switching formulation, provided every remaining stage is
re-certified against the current target-dependent corridor and physical wall
map.

## Refutation conditions

- The current corridor cannot be bound to current target provenance.
- Remaining retained stages frequently fall outside the current corridor.
- Pose/progress continuity rejects most candidate windows.
- Retained coverage does not materially reduce uncovered fresh misses.

If any refutation condition holds, production authority will not be promoted;
the next correction must address that upstream contract instead.

## Implemented proof boundary

The Slice adds a typed `OvertakeDynamicCorridorObservation` and fingerprints the
current target identity, current V2X generation/time, target-exclusion evidence,
time knots, and lateral bounds. A retained plan is accepted only when all of the
following identities and physical checks agree:

- exact retained cursor and remaining certified window;
- current ShiftOut/Pass/Return intent and intent generation;
- target ID and current target observation generation;
- measured-to-control pose prefix and control-pose fingerprint;
- current course-frame fingerprint and circular progress branch;
- current measured and expected retained lateral state;
- every remaining retained segment against the current dynamic corridor;
- delay prefix, connector, and every remaining segment against the physical wall map.

Only a complete fresh canonical selection replaces the immutable plan store.
Rejected or partial fresh chains cannot poison it. The retained result is passed
through the existing canonical retained candidate selector, exact actuation
extractor, command adapter, and world-prediction builder. It remains telemetry-only.

## Dynamic evidence

Source bag:

`output/20260823-214300-stop-authority-replay-v2/d1/rosbag2_autoware`

The replay used isolated ROS Domain 91, injected `/awsim/state=Start`, replayed
the six vehicle/V2X/trajectory input topics plus `/clock` at 1.5x, and excluded
the recorded control command. The current controller remained the only command
publisher. Final-code replays are:

- `output/20260824-overtake-retained-current-world-replay-v2/d91/autoware.log`
- `output/20260824-overtake-retained-current-world-replay-v3/d91/autoware.log`
- `output/20260824-overtake-retained-current-world-replay-v4/d91/autoware.log`

The two final typed-outcome runs produced:

| Metric | replay v3 | replay v4 |
|---|---:|---:|
| evaluated | 997 | 1003 |
| eligible Overtake | 951 | 953 |
| fresh canonical selected | 755 | 757 |
| retained canonical selected | 59 | 58 |
| canonical fresh-or-retained coverage | 85.6% | 85.5% |
| cursor rejected | 69 | 94 |
| progress rejected | 44 | 30 |
| course frame unavailable | 8 | 10 |
| initial corridor violation | 4 | 2 |
| stage corridor violation | 2 | 2 |
| no prior plan | 10 | 0 |
| physical hard-wall reject in fresh chain | 2 | 0 |

No retained candidate passed by age alone. Every one of the 117 accepted retained
cycles completed current-world proof, candidate selection, retained authority,
exact actuation, command reconstruction, and prediction construction. Every
uncovered cycle ended in a typed fail-closed outcome.

The replay is not bitwise timing deterministic: fresh solve count and isolated
physical-wall reject count vary between runs. Therefore absolute cycle counts
are not interpreted as vehicle performance. The stable result is that retained
proof covers bounded fresh gaps but leaves roughly 14.5% of eligible Overtake
cycles uncovered.

Final replay v4 measured retained proof evaluation at 0.162 ms average and
0.760 ms maximum over 196 attempts. The 40 Hz callback still recorded 66
overruns in 2991 cycles with a 56.072 ms maximum, but retained proof itself is
too small to explain that tail. Existing extended solver scheduling remains a
separate runtime-quality blocker.

## Causal conclusion

### Root cause repaired by this Slice

A complete five-state Overtake plan previously disappeared after one cycle, so
a bounded fresh solve miss had no same-formulation continuation evidence. The
immutable plan store plus current-world proof repairs that evidence producer.

### Contributing causes exposed

- Fresh/circuit-breaker gaps can outlive the stored plan certificate horizon.
- Current measured progress can leave the retained branch tolerance.
- The current problem's course-frame window can be shorter than the remaining
  retained plan.
- A small number of current/stage corridor checks correctly invalidate the old
  plan after the world changes.

### Existing mask

Production still converts the five-state solution to the legacy representation
and may run the three-state normal fallback. It masks uncovered canonical cycles
but preserves the split authority this migration must remove.

### Detection gap closed

The prior telemetry logged only the final rejection in each one-second window.
Every evaluated cycle now contributes exactly one typed outcome, so coverage and
all rejection classes are aggregatable without increasing log frequency.

## Acceptance decision

Accept this Slice as shadow proof infrastructure. Do not promote Overtake
production authority yet. Coverage of 85.5--85.6% would turn the remaining
14.4--14.5% of eligible cycles into repeated Emergency Stop after deleting the
legacy fallback.

The next Slice must repair the upstream continuity contract, led by certificate
horizon expiry, progress-branch continuity, and course-frame coverage. It must
not lengthen a timeout or loosen progress/corridor tolerances as the primary fix.
After that, rerun this same typed-outcome replay and require full fresh-or-retained
coverage before production promotion and legacy deletion.

## Static validation

- `make autoware-build`: 25 packages passed.
- Focused current-world revalidation: 9/9 tests passed, including three new
  Overtake cases.
- Full `multi_purpose_mpc_ros` suite: 40/40 programs, 1700 tests, zero
  errors/failures/skips.
- Existing missing `build/joycon_contract_guard/package.xml` result warning is
  unrelated.
- No parameter, solver tolerance, timeout, lease, fallback, or production
  authority change was made.
