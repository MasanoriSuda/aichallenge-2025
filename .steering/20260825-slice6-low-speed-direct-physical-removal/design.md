# Design

## Causal audit

The previous authority-retirement Slice removed all paths which could call
`low_speed_shift_control()` or set its active latch. Static inspection now gives:

```text
low_speed_shift_control() definitions = 1
low_speed_shift_control() call sites  = 0
low_speed_shift_control_active_=true  = 0
low_speed_shift_control_was_active_=true producers
  -> only inside the uncalled function
```

Therefore all branches guarded by `low_speed_shift_control_was_active_`, together with their
publisher owner and wall-stop override, are unreachable compatibility infrastructure. Keeping them
does not add safety: it keeps a second normal-command schema representable and obscures which
stopped-vehicle logic is canonical.

## Target flow

```text
stopped/slow V2X vehicle
  -> stopped-vehicle confirmation and side/corridor planner
  -> static-wall preflight
  -> canonical MPCC problem bounds/reference/speed window
  -> certified canonical MPCC command
  -> external Emergency or Recovery only when required
```

## Deletion boundary

Delete:

- direct controller phase/latch/rejoin/retained-pass state;
- direct controller function, accessors and publisher clamps;
- direct wall-stop and final-source branches;
- direct-only core helpers and their tests;
- direct-only configuration values;
- `mpcc_execution_contract::Formulation::LowSpeedDirect`.

Keep:

- `LowSpeedAvoidance` intent and candidate confirmation;
- low-speed gap/local-path generation and static-wall preflight;
- `resolve_low_speed_pass_velocity()` and the live shift-speed setting;
- `resolve_low_speed_shift_steering()` and lateral-acceleration limiting used by solver-failure
  crawl;
- gap-planner target locks, which are planner state rather than a command authority.

## New exceptional paths

None. This Slice only removes unreachable code and obsolete representations.
