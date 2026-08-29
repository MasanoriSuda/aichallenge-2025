# Audit: publisher/stage boundary authority hole

## Observed phenomenon

The repaired current-world Bundle restored normal authority from Stop, but D1
still inserted short Emergency commands during Track, Cruise, Pass and Return.
The earliest repeated invariant violation was not a solver failure:

```text
current=continuation-rejected
continuation=model:invalid-cursor
command=0.00m/s/-3.00m/s2
```

Across `output/20260829-220933/d1/autoware.log`, 143 of 144 logged
`continuation-rejected` transitions had this same model reason.

## Causal chain

1. An asynchronously solved artifact is first published at an arbitrary
   artifact-local cursor.
2. The 25 ms publisher clock then advances independently of the roughly
   100 ms solver stage grid.
3. Near a stage end, the selected stage has less than 25 ms remaining.
4. `build_continuation()` correctly refuses to claim that this command can be
   held for the complete serialization interval.
5. No upstream code selects the next sealed stage and re-proves it from the
   current world.
6. Normal authority becomes unavailable and Emergency braking is serialized.

The Emergency and later Recovery are downstream symptoms.  Changing clearance,
Mission lifecycle or solver tolerance would not repair the missing command
selection operation.

## Existing patch relationship

The previous Slice correctly permits a fully proved latest-state feedback
Bundle.  It exposes this independent failure instead of hiding it behind a
permanent Stop.  The same Bundle principle applies here, but the cause is
publisher/stage phase rather than steering reachability.
