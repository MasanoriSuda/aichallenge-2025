# Design

## Causal chain under test

```text
published command N
  -> asynchronous preparation at origin N
  -> commands N+1 ... N+k actually cross the wire
  -> result consumed at a later control origin
```

The existing rejected A arm changes x0 and the previous input while retaining
future state tubes and linearizations from origin N. The B arm instead:

1. resolves the elapsed absolute-time stage suffix;
2. consumes the same stage count from every stage-indexed object;
3. shortens the active first stage rather than restarting it;
4. binds the latest seven-state observation and serialized previous input;
5. relinearizes all surviving dynamics around the suffix-owned tangent;
6. solves the rebuilt QP and constructs a new immutable execution artifact;
7. applies the unchanged exact nonlinear adapter and, during the live A/B,
   the current wall/dynamic/terminal-successor proof.

## Runtime boundary

The solve executes only on a dedicated latest-only observation worker. The
40 Hz callback may capture immutable input and consume aggregate telemetry; it
does not solve, publish, replace the certified Store or mark an artifact
executed.

## Rejected shortcuts

- do not relabel an old problem by changing only x0 or timestamps;
- do not publish the analytical first-steering projection;
- do not hold Stop steering for a guessed solver duration;
- do not accept a suffix because its Mission still exists;
- do not weaken physical constraints to increase the acceptance rate.

## Recorded result

### Offline replay

The same 20-stage preparation was evaluated in two ways:

- changing only the old final QP initial state remained infeasible;
- rebuilding one common-clock suffix accepted the exact physical adapter.

Two representative accepted suffix solves took 8.294 ms and 22.404 ms total,
while corresponding full current-world solves took approximately 46.80 ms and
99.96 ms.  This proves that the exact preparation owner matters and that a
suffix solve can be materially cheaper than a full second solve.

### Live observation-only A/B

Run: `output/20260829-182105`.

| Domain | Results | QP accepted | Physical accepted | Dynamic clear | Current-world authority-ready | Mean / max compute |
|---|---:|---:|---:|---:|---:|---:|
| D1 | 1,469 | 671 | 671 | 241 | 122 | 24.22 / 108.11 ms |
| D2 | 165 | 153 | 153 | 51 | 35 | 27.42 / 77.77 ms |

Production Store, authority selection and publisher were unchanged.  The
observation worker nevertheless rebuilt artifacts which passed the unchanged
QP, nonlinear physical, dynamic and retained current-world proof.  Therefore
the frozen `steering-unreachable` population is not physical infeasibility:
it is primarily a scheduling/lifecycle defect caused by joining an old async
trajectory directly to a newer serialized predecessor.

The unrestricted observation trigger was intentionally adverse: D1 submitted
about 1,300 feedback jobs and continuously occupied the worker.  Its tail cost
exceeds the 25 ms callback period.  It is evidence for a connector, not an
acceptable production schedule.

## Architectural consequence

The temporary live observation wiring is removed at Slice close.  Keeping it
would create a second permanent result owner and a high-rate diagnostic load.
The reusable suffix builder/solver, deterministic regression and offline
replay remain.

The next production Slice must make a single atomic ownership change:

1. a raw async result is preparation evidence, not directly executable when
   it cannot join the serialized predecessor;
2. at most one current-state suffix connection may own that rejected result;
3. only the resulting newly certified artifact may replace the Store
   candidate for that connection attempt;
4. no direct stale adoption and feedback adoption may coexist for the same
   source result;
5. worker admission must be bounded by in-flight ownership and measured cost,
   not a retry lease, grace period or behavior timeout.
