# Measured footprint slice accepted; overall MPCC acceptance open

The prior nominal footprint misses13of33 independent hull vertices in both
configurations. The first repaired width also reveals the combined-distance
coupling: old1.45m minus new ego0.768m shrinks the inferred peer to0.682m.
Both regressions fail before their corresponding repair. All4checks now pass.

`make autoware-build`:25packages pass. Full MPCC package:2323tests,
0errors/0failures/0skips. No margins, weights, prediction time or authority
requirements changed. Controller SHA256:
dd7434c8b250534a8e9d4a41a1a3d1afee255225c8594fc799dfa935f63ba15e.

`20260908-footprint-single-r1`, Domain1, existing six-lap evaluation:
finished=true,6laps249.613098145s,penalty0. No active-race moving Emergency
or Recovery. Moving Emergency11215/11216 occurs after Finish1788802943.418682.
Callbackmax24.708ms,0overruns over13060reported cycles.
Control10483messages/262.013424receive sec:mean24.996511ms,
p9526.548755ms,p9927.395587ms,max31.618595ms,0gaps>50ms,
0nonpositive receive gaps. Source stamps separately contain32nonpositive
gaps,max100ms and4gaps>50ms.

This is one new-binary diagnostic, not an inherited pass or three-repeat
acceptance. Map coverage and full peer-body enclosure remain open. In
particular the inferred peer circle0.768m nominal only accounts for lateral
half width; it must not be described as a complete physical vehicle envelope.
