# Current authority graph after Slice 6

Updated: 2026-08-26

## Normal observation-to-command flow

```text
localization + trajectory + steering observation + V2X/world snapshot
  -> typed ControlIntent
  -> immutable six-state submission snapshot
  -> VelocitySteeringProgress6State problem
  -> latest-only async solve
  -> exact physical wall/obstacle proof
  -> current-world fresh or retained proof
  -> canonical production adapter
  -> one normal command publisher
  -> /control/command/control_cmd
```

Track, Cruise, Follow, ShiftOut, Pass, Return and Rejoin use this same normal
formulation.  The old legacy three-state solver, progress three-state solver,
five-state normal owner, lossy converter, pre-entry tactical Gate and migration
availability switches are physically absent.

## Tactical and execution responsibilities

| Stage | Owner | May publish normal control? | Contract |
|---|---|---:|---|
| Target and behavior | V2X behavior/FSM | No | Select intent and target only |
| Left/right candidate construction | tactical worker | No | Immutable geometry/intent snapshot |
| Left/right branch evaluation | prospective six-state worker | No | Same formulation and physical proof as production |
| Overtake Gate A | causal six-state worker | No | Binds current predecessor, world, Mission and exact prefix atomically |
| Fresh/retained execution | certified six-state plan store | Yes | One current-world-certified normal candidate |
| Final normal adapter | rate-resolved production adapter | Yes | Exact certified command only |

The tactical Mission is a soft intent and homotopy supervisor.  It cannot own
a second solver command, physical certificate or fallback.

## Intentional external overrides

```text
explicit emergency safety fault -> typed Emergency Stop command
confirmed stuck/contact/gear recovery -> Recovery command
```

These are supervisors outside normal authority.  They must not reactivate an
MPC, three-state or five-state normal fallback.

## Remaining integration-quality findings

Structural Slice 6 is complete, but race-quality acceptance is not:

- fresh six-state failure can still coincide with
  `retained-proof-unavailable`, producing an explicit Emergency cycle;
- an admitted ShiftOut can later lose its locked target and enter Recovery;
- callback timing tails must be measured over full multi-lap runs;
- Pass and Return require more positive dynamic coverage than ShiftOut.

These are investigated from six-state solver/proof and target-lifecycle
evidence.  They are not addressed by restoring a deleted authority or by
tuning parameters before root-cause evidence.

## Enforcement

`test_single_authority_source_contract.py` rejects restoration of:

- legacy/three-state formulation identities and conversion;
- five-state formulation identity and normal libraries;
- normal fallback publishers and runtime migration switches;
- five-state Overtake tactical Gate, plan cache and certificate revalidator;
- multiple normal dispatch paths.

The post-Slice-6 production graph therefore has exactly one normal
formulation and one publisher path, with Emergency and Recovery as the only
explicit exceptions.
