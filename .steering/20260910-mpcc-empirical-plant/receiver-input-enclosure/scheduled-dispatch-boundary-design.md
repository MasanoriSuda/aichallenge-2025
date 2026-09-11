# Scheduled final packet boundary (C5 integration in progress)

Baseline73049f17. Node authority is still synchronous. This boundary passes
native/build/package validation and is not yet connected to a worker or publisher.
It must be promoted with one scheduled dispatcher and retirement of the replaced
synchronous normal/Stop/GateA/previous-intent joins.

Prepared Stop programme retains the original physical steering scalar for every
wire packet. Both arrays are trimmed together for duplicate terminal wire values.
Programme bits, nominal samples, native body model and physical bounds are unchanged.
This retains an immutable serialization witness; it is not a new input law or a
claim that dividing wire by gain is always wrong. An exploratory1million-value
float32 roundtrip search for gain1.435 found no counterexample and proves no
universal property; no numerical fix is inferred from it.

prepare_dispatch uses the exact next suffix index, original ledger cursor and
all actual preceding source identities. It checks one fresh observation/request,
context and remaining world through the existing C4API. The fresh point prefix
must consider the actual next wire payload at fresh.now. The programme packet
keeps its original integer nominal epoch. The returned immutable candidate stores
current dispatch decision separately from source job decision and ledger identity.

At the publisher, matches_before_publication must recheck the unchanged ledger,
source generation, current decision, exact final float32 values, raw window and
steering slew from the previous ledger publication epoch, which includes the
existing causal floor if raw /clock lagged. Existing actuator tolerance
is retained; no new grace/retiming/hold or causal floor is admitted. The matching
after-publication check authenticates one actual ledger send and its source/index,
packet and raw window. Failed post-send checks are violations and cannot erase
already issued commands or make them retrospectively admissible.

Tests cover physical witness retention, current/source decision separation,
1ns clock boundaries, actual short predecessor intervals, stale/extra/missing/wrong
source publications, current world conflict, generation revocation and a real
second suffix send retaining the original immutable proof. These are local API
checks; runtime single-thread ownership, atomic mission adoption, post-send
observation capture and source publication remain C5 integration obligations.


R80build passes26packages/4min36s and r69passes2516records/64groups. Two
new gtest dangling-else warnings were corrected with explicit braces. R272/273
additional test inputs incorrectly placed the appointment1.01 before the original
observation1.05; the factory correctly rejects publication-prefix-unavailable.
R274uses a future1.06 appointment but its0.009rad step fits the0.010001 bound.
R275moves the appointment to1.055; the existing nominal builder already limits
its step to the actual causal-predecessor bound0.005001. Thus the hypothesis of
an unsafe first packet in these examples is falsified, not a production defect
reproduction. The final guard now uses the same ledger epoch as that nominal
builder. The regression checks preservation of this existing protection; it does
not relax a physical acceptance criterion or claim a fix for r44timing.

R276passes all127native tests in796ms with no compiler warnings. Final buildr81passes26packages/28.7s; package r70passes2517records/64groups,
0errors/failures/skips. Actual C5node integration and all M4–M6remain pending.

[Sealed evidence](scheduled-dispatch-boundary-evidence.json).
