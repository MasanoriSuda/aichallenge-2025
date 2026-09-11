# Exact runtime work after the final-publication capture

Baseline and rollback:3387e684. Current publication window25ms and empirical
receiver profile250ms remain fixed. M4–M6 is not complete.

The earliest failing boundary in dev2-r31 is the final publication bracket,
not the six captured certificates: all six replay Accepted inr207/r208. Moving
D1decision863 advances35msROS with primary19.056ms andRecovery5.502ms;
D2decision883 advances30ms. Late publication remains rejected. The stationary
D1decision602 has a separate double-versus-nanosecond endpoint representation
issue; this performance slice does not change its guard or suppress it.

R32lost sensor updates during startup and was stopped with SIGINT after saving
AWSIM logs. Its cause is unknown; it provides no normal-control timing result.
R33samples main-thread/proc schedstat: representative callbacks spend18–24ms
CPU plus1–10ms runqueue time. R33's first post-publication failure is D1714,
nominal4.279999904 to after4.309999903. It occurs beforeReady; the recordedReady
arrives after shutdown initiation. These measurements describe startup control,
not accepted race running. Per-callback CPU deltas are nearest-sample estimates.

R208rejects the hypothesis that unmeasured request construction/adapter work
explains the live gap: prospective prefix0.010–0.012ms, production adapter
0.061–0.186ms, bundle identity negligible. Revalidation9–13ms, applied input
proof7–8ms, body-only prediction4.4–5.0ms. Two physical_course_progress calls
are constant-time scalar arithmetic. Live/offline proof cost differs; there is
no evidence permitting a cache, scheduling grace or proof omission.

## Bounded same-input alternatives

All diagnostic candidates remain outside production until this slice's gates.
Original arithmetic order, physical kernel, derivatives, margins, receiver
population, complete rest horizon and all current-world checks are retained.

- R209: SSE2 computes the existing scalar down/up adjacency for two endpoints
  together. Non-SSE2 uses the original scalar functions. Six complete body and
  corner tubes are bit-identical; applied proof improves about10%.
- R210: compute the four vertices' common world translation and half-angle sine
  once within one corner map. Six complete tubes are bit-identical; about0.4ms
  less proof work. No value survives the map invocation.
- R212: combined candidate, six tube byte comparisons all equal; original
  elapsed8.91–12.70ms becomes7.82–11.67ms. The independent adjacency oracle
  compares1,000,196pairs /2,000,392endpoints, including arbitraryNaN payloads,
  infinities, signed zeros, subnormals and random binary64 bit patterns.
- R211: Recovery's immutable map producer omitted the existing summed-area
  index, while the normal wall-map copy builds it. On the six captured poses,
  all324comparisons across6directions/3distances/3contact policies preserve
  every feasibility field and full rollout;192accepted/132rejected. The54-case
  batches fall from11.55–12.96ms to7.41–10.97ms. These poses are saved control
  poses, not a reconstruction of each actual Recovery callback. Index creation
  costs4.5–4.7ms once at startup and4,572,160bytes per vehicle. Existing tests
  compare exact cell sets, unknown cells and invalid inputs with/without index.

## Production scope and gates

Change only numerical pair implementation and within-map repeated expressions,
prepare Recovery's existing exact broad phase after all cells are populated,
and expose existing applied/continuation proof timings in overrun observation.
No normal authority added/removed; no physical/profile/parameter/clock changes.
The existing whole-population tests, high-precision Jacobian oracle, earlier
rejected scenes and final-boundary negatives remain required.

- [x] Same-input independent candidates, full tube byte equality, adjacency oracle.
- [x] Recovery candidate positive/negative parity against its original exact scan.
- [x] Buildr64:26packages; testsr53:2465records/63groups,0errors/failures/skips; r213–215production21cases preserve original results.
- [x] Seal2214files for r31–r33/r207–r215;107local document links and pre-commit pass. Reviewed for unchanged arithmetic ordering, immutable cells and no new authority. Local commit prepared.
- [ ] Fixed committed dev2-r34, first failure analysis and actual deadline acceptance.
- [ ] Remaining initial long proofs, timestamp endpoint contract, M4–M6.

[Sealed evidence](exact-runtime-evidence.json) records all attempts and comparisons.

Native validation was performed on this x86_64 host; execution on other
architectures has not been tested. Their original scalar fallback is retained.
