# Root-cause audit

## Causal chain

```text
canonical ShiftOut command is certified
-> nominal future wall warning enters action band
-> legacy local centerward prefix is evaluated
-> legacy kinematic preflight rejects lateral acceleration
-> ExitCurrentMission invalidates generation
-> current and in-flight canonical artifacts lose identity
-> DynamicWait has no lateral authority
-> explicit Emergency / later hard wall contact
```

The wall warning is legitimate. The defect is assigning Mission termination to
the availability of one optional legacy reference producer.

## Historical intent

The branch was introduced before canonical Overtake production to prevent a
fixed Mission from being held into the wall when no short escape prefix could
be built. At that time DynamicWait/Recovery was the only fail-closed owner.

Canonical MPCC now revalidates every fresh or retained solution in the current
world and the external supervisor publishes Emergency when no certified normal
solution exists. Retaining the historical exit therefore duplicates safety and
destroys the identity needed for a newly solved alternative.

## Invariant

Runtime wall preplan may produce or recommend a reference. It may not decide
canonical Mission viability. Only an accepted atomic replacement changes the
Mission; otherwise the current identity remains until canonical proof,
supervisor intent completion or an independent hard fault decides its fate.
