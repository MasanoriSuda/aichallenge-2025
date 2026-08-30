# Results

## Frozen replay

Source:

`output/20260830-200852/d1/mpcc_architecture_snapshots/000000004017-ee88c9e56718aeeb-shiftout-side-positive-physical-proof-terminal-contingency-unavailable/snapshot.yaml`

The first causal seven-state comparison was not a valid Stop comparison: it
commanded approximately `+1.33 m/s^2` for the first five stages before
braking.  This Slice fixed every future velocity state to the solver-safe
maximum-braking schedule and repeated the unchanged exact physical, wall and
current-world proofs.

## Corrected seven-state result

| Metric | Result |
|---|---:|
| result | accepted |
| solve time | 156.53 ms |
| terminal progress | 5.55934 m |
| terminal velocity | approximately 0 m/s |
| exact minimum lateral reserve | 0.403418 m |
| acceleration range | -2.95960 to approximately 0 m/s^2 |
| acceleration time mean | -1.68981 m/s^2 |
| acceleration stages on solver-safe boundary | 11 / 20 |
| steering-rate range | -0.714215 to +0.714218 rad/s |
| steering-rate time mean | +0.0515101 rad/s |
| steering-rate stages on solver-safe boundary | 8 / 20 |
| steering-rate sign changes | 2 |

The steering sequence is structurally compact:

1. four stages at positive steering-rate boundary;
2. four stages at negative steering-rate boundary, followed by one partial
   negative correction;
3. a near-zero interval;
4. a monotone decaying positive terminal correction.

Acceleration is maximum braking until the vehicle is almost stopped, followed
by one partial braking stage and zero acceleration.  No positive acceleration
is present; the numerical maximum is `8.36e-12 m/s^2`.

## Root-cause classification

The frozen failure is not physical infeasibility and does not require
acceleration before braking.  Under the same world, bounds and proof chain, a
maximum-braking Stop exists when steering rate is solved jointly with the
seven-state dynamics.

The failure is therefore upstream of the exact certificate: production's
fixed-target/path-feedback Stop candidate family cannot express the required
steering sequence.  The earlier normal-path-profile rejection remains valid,
but the previous unconstrained causal Stop result must not be used as evidence
for longitudinal feasibility; this corrected result supersedes it.

The low sign-change count and boundary-dominated controls make a finite
bang-bang control lattice plausible.  They do not yet prove that a lattice
member passes the exact certificates.  The next audit Slice should enumerate a
bounded set of causal steering-rate switch patterns under this same
maximum-braking schedule.  Only if that comparison fails should a fresh
asynchronous seven-state Stop solver be considered for production.

Production authority remains unchanged.

## Verification

- focused build: passed
- physical-adapter test target: passed
- architecture-comparison test target: passed
- corrected frozen replay: accepted with no positive acceleration
- full package CTest: 59 / 59 passed
