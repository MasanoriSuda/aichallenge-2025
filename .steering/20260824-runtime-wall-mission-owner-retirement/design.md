# Design

Delete `ExitCurrentMission` from the runtime wall preplan action domain.

When the action band is reached and neither an accepted same-side replacement,
centerward prefix nor valid Return exists, resolve to `HoldCurrentSide`. This
name means only "do not mutate Mission identity in the preplanner"; it does not
authorize a command. Canonical current-world proof still decides whether a
normal command is publishable, and otherwise the explicit Emergency supervisor
owns output.

The following producer capabilities remain:

- request a fresh same-side candidate;
- atomically replace with a fresh same-side candidate;
- atomically replace with a validated centerward prefix;
- return to the base line after rear-clear.

The following legacy ownership is deleted:

- prefix failure -> Mission invalidation;
- prefix failure -> DynamicWait;
- prefix failure -> line Recovery.

No parameter or timing contract changes.
