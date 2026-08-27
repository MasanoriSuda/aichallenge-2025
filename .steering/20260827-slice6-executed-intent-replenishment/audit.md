# Audit: executed-intent replenishment

## Baseline failure

The baseline trace in `output/20260827-194608/d1/autoware.log` showed the
first invariant violation before Emergency:

1. ShiftOut artifact sequence 2300 crossed the publisher boundary.
2. Tactical output changed to Follow.
3. Atomic admission retained sequence 2300 for publication, but problem
   assembly and asynchronous submission had already used Follow.
4. No ShiftOut successor was produced while the retained artifact advanced to
   its final cursor stage.
5. Cursor exhaustion left neither proposed nor previous normal authority and
   forced `no-current-world-authority` Emergency.

This establishes that horizon exhaustion was the observation point, while
late intent resolution was the upstream producer fault.

## Implemented invariant

The exact last-published certified artifact may now supply a typed canonical
execution identity before authority and problem assembly when, and only when:

- its immutable artifact and certified plan validate;
- its intent equals the last successfully published canonical intent;
- that intent is ShiftOut, Pass, or Return;
- target, generation, and side form a complete homotopy identity; and
- its cursor is executable at the predicted control origin.

Live OvertakeLine and live DynamicEscape remain higher priority. Cursor
availability is the only retained lifetime; no lease, timeout, or new command
fallback was introduced.

## Static gates

- `make autoware-build`: passed, 25 packages built.
- Full `multi_purpose_mpc_ros` package tests: 47/47 CTest targets passed.
- `colcon test-result --verbose`: 1986 tests, 0 errors, 0 failures, 0 skipped.
- `git diff --check`: passed.

The deterministic contract test pins this ordering:

```text
executed artifact cursor
  -> canonical execution identity
  -> authority phase
  -> resolved problem intent
```

Unit tests cover retained acceptance, live-source precedence, cursor-expired
inactivity, and malformed identity rejection.

## Dynamic gates

### Single car

Run: `output/20260827-204416`

The vehicle entered normal Track/Cruise production and continued beyond 97 s.
No running-session canonical Emergency or authority hole occurred. One shadow
QP reached maximum iterations; it did not own production and did not propagate
to a stop.

### dev2

Run: `output/20260827-204645`

Both vehicles launched and V2X observations were present. The faster vehicle
generated multiple complete tactical missions, but six-state pre-entry
solutions were not adopted into ShiftOut production during this run. The new
retained-ShiftOut branch therefore was not exercised dynamically and direct
dynamic acceptance is not claimed.

The run separately exposed a more general transition defect: at the first
Cruise-to-Follow change, the previous Cruise cursor was unavailable and the
fresh Follow producer had not completed. Decisions 1072 onward temporarily
entered `no-current-world-authority`, including Emergency braking. This is the
next structural Slice; it is not masked by this change.

## Conclusion

The baseline ShiftOut exhaustion causal chain is repaired structurally and
guarded by deterministic tests. Static and single-car gates pass. Dynamic
ShiftOut proof remains pending on a run in which pre-entry production adoption
actually occurs. The separate Cruise/Follow transition hole must be resolved
without weakening cursor or current-world proof.
