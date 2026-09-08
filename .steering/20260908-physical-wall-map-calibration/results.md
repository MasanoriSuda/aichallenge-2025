# Static map calibration passes; dynamic integrated acceptance fails

Before: the known contact coordinate is free, and native Map.data turns a
calibrated singleton into free. Five Python assertions and native regression
fail. After:25Python tests pass; singleton retained;570009native/editor cells
match. The physical wall world already uses raw pixels, so the singleton fix
aligns reference/corridor generation with final proof.4567cells are added,
all old occupied cells retained. Regeneration SHA256matches the applied map:
4958e3f1d977e75d202ef76e099836ca0622ea42e42c5bb2fea92616cb7e9211.
All-height near-vertical projection is conservative and not full3D coverage.
The reference's continuous sweep clears nominal body plus0/0.05/0.25m.

Build:25packages pass.60CTest groups pass;2269test cases from package-local
test_results have0errors/failures/skips. Older reports using the entire build
directory additionally counted the60CTest aggregate records. Native/edit
parity and the actual Map singleton behavior are separately tested.

Controller8af0ea61a7784e0b3ca0c7f6f130c601e677d4713d21b66098813c826bb447bd.
Diagnostic20260908-wall-map-single-r1 finishes6laps253.198868s,penalty0,
but active-race Emergency10699/10700 occurs at7.53/7.55m/s. No Recovery.
This is rejected as integrated acceptance despite finishing. First rejection
is delay-prefix-blocked/collision with published sequence10073; Stop terminal
proof is not reached. Continue at../20260908-mpcc-delay-prefix-audit/.
Finish1788803939.854134 precedes separate Emergency11333/11334.

Control10604messages/265.024996receive sec,mean24.995284ms,
p9526.552033ms,p9927.622485ms,max32.982349ms,0gaps>50ms,
0nonpositive receive gaps. Source has9nonpositive,max70ms,2gaps>50ms.
Callbackmax19.623ms,0overruns (final reported-cycle count in sealed summary).

No wall/certificate threshold is relaxed in response. The geometry producer
repair remains locally supported; the current controller's new-world
execution still needs a causal fix and a fresh acceptance campaign.
