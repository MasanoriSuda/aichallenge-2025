# Root-cause audit

## Observed phenomenon

Expected:

```text
certified ShiftOut plan
-> receding execution remains current or is replaced by a matching certified plan
-> ShiftOut / Pass continues
```

Observed:

```text
generation-1 ShiftOut canonical command published
-> initial Frenet-DP source expires at age 0.53 s
-> no legacy solved-execution trajectory is available for DP handoff
-> canonical Overtake worker continues to report current-world physical success
-> receding-horizon validator reports physical failure
-> Mission generation invalidated
-> DynamicWait has no lateral authority
-> Recovery
```

## Authority graph

```text
V2X observation
-> tactical Mission + Frenet-DP profile
-> atomic canonical five-state entry artifact
-> canonical Overtake async producer / retained current-world proof
-> canonical normal command publisher

parallel legacy continuation path:
frozen Mission / DP profile
-> receding-horizon lateral optimizer
-> coupled static-wall and lateral-acceleration revalidation
-> Mission viability / DynamicWait / Recovery
```

The previous Slice removed a later legacy wall command owner. It did not remove
this earlier Mission-viability owner.

## Hypotheses

| Hypothesis | Supporting evidence | Falsifier | Needed observation | Confidence |
|---|---|---|---|---|
| H1: the frozen DP source is not refreshed after entry because its pre-entry result has generation 0 while the Mission is generation 1 | first post-entry tactical result is `mission-generation-mismatch`; DP source expires at exactly 0.53 s | a generation-1 source is produced and accepted before expiry | source/result/Mission generation and promotion result | medium |
| H2: canonical Overtake and receding-horizon validation are duplicate viability owners and disagree on the same world state | canonical telemetry reports 36 current-world/physical accepts while receding validation later invalidates the Mission | exact failure cycle shows canonical plan absent or physically rejected for the same stage | canonical stored plan identity/current proof status at failure | medium-high |
| H3: the vehicle has genuinely reached a future wall or lateral-acceleration infeasibility after moving 4–8 m | failure occurs during a hard curve and the receding validator searches all speed/clearance repairs | exact diagnostic shows valid wall profile and acceleration below the configured limit | failure cause, stage, d(s), bounds, heading and required acceleration | medium |
| H4: source expiry itself directly causes Recovery | timing is close and the bridge has no solved legacy trajectory | the Mission continues for about 1.4 s after expiry, as current log already indicates | separate source-expiry and physical-failure records | low |

## Root/contributor/mask status before instrumentation

- Root cause: **Unknown**. H2 and H3 are not distinguishable with current
  telemetry.
- Contributor: the initial DP source has a 0.5 s absolute lifetime and no
  accepted replacement before expiry.
- Detection gap: the physical-revalidation result drops the decisive profile
  sample and candidate contract.
- Mask: scheduled exact-context hold keeps the old receding evaluation active
  temporarily; DynamicWait then attempts replacement before Recovery.
- Recovery behavior: `FollowPrepare -> Recovery` is downstream safety handling,
  not the root cause.

## Dynamic evidence

Run: `output/20260824-130017/d1/autoware.log`

The decisive episode is episode 3 / Mission generation 1 / target `d2` /
side `+1`.

1. Entry was accepted at `1787544169.907` with a physical certificate and a
   frozen 31-point DP execution profile.
2. Canonical current-world proof was intermittently accepted. At
   `1787544170.921`, 35 canonical horizons were physically accepted and the
   retained plan had zero initial corridor reserve.
3. Before the legacy physical failure, canonical normal authority was already
   unavailable. At `1787544171.618`, the publisher selected the explicit
   emergency override because the retained canonical plan failed
   `initial-corridor-violation`.
4. A generation-1 DP rolling refresh was accepted at `1787544171.589`; it was
   not stale when the failure occurred.
5. At `1787544171.817`, the independent receding validator rejected stage 0.
   Its configured 0.40 m planning contract was infeasible; the recorded
   candidate also required 9.677 m/s2 lateral acceleration at 4.053 m/s.
6. That legacy failure invalidated the whole Mission generation and sent
   `ShiftOut -> FollowPrepare -> Recovery`, despite the canonical emergency
   supervisor already owning the unsafe interval.

The exact failure trace was:

```text
cause=static-map-clearance, stage=0, distance=0.984,
target_ey=1.720, wall=[-4.041,2.820], ay=9.677,
clearance=0.400, attempts=4,
dp=1/authority=1/source_age=0.450,
canonical=1/plan=5996/generation=1/side=1/cursor=1/stage=3
```

The `static-map-clearance` label describes the first configured planning
contract. It must not be read as the terminal physical cause by itself: the
same record shows a simultaneous lateral-acceleration infeasibility after
repair attempts. This is a diagnostic limitation to preserve in the next
observation Slice rather than hide with a threshold change.

## Hypothesis result

- H1: **contributor only**. A generation-1 DP refresh was accepted before the
  decisive failure, so the original generation mismatch is not the root.
- H2: **confirmed as an architectural defect**. Canonical current-world proof
  and the legacy receding validator both own normal-execution viability. They
  reject different contracts and the legacy owner can destroy the Mission
  after canonical has already selected emergency authority.
- H3: **also observed, but not yet reduced to one scalar cause**. The actual
  state was outside the retained plan's initial lateral corridor, while the
  legacy repair path also failed planning clearance/lateral acceleration.
- H4: **falsified**. DP authority was active and its source age was 0.450 s at
  the decisive failure.

## Root-cause classification after instrumentation

- Root architectural defect: Mission viability has two independent owners.
  Canonical MPCC correctly withdraws normal actuation when current-world proof
  fails, but the legacy receding path additionally invalidates tactical state
  and forces Recovery.
- Trigger: measured execution drift reached the boundary of a tight retained
  lateral corridor (`reserve=0.000` shortly before rejection), after which no
  currently certified canonical plan was available.
- Amplifier: the legacy repair path applies its own planning-clearance and
  lateral-acceleration contracts to a different horizon representation.
- Downstream symptom: Mission invalidation, DynamicWait without authority,
  then Recovery.

The next behaviour Slice must retire the legacy Mission-invalidation owner
under canonical Overtake production. It must not relax wall clearance,
lateral acceleration, current-world proof, or emergency supervision.

## Patch history

- `640532a` introduced the aggregate `optimized horizon failed physical
  revalidation` path as part of Mission continuity.
- `dbc872f` introduced strict async Mission-generation lease rejection. That
  contract is safety-correct; it must not be loosened merely to refresh a path.
- `7da5205` deleted the downstream legacy wall handoff owner for canonical
  Overtake and exposed this earlier independent failure.

## Implementation gate result

This observation-only Slice added zero production branches and zero
configuration. The dynamic run confirmed duplicate Mission-viability
ownership and therefore opens a separate behaviour Slice. The observation and
behaviour changes remain separate commits.
