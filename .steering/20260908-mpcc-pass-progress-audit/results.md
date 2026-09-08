# Pass closes some distance but does not complete

Run4 (`20260908-mpcc-stateless-successor-dev2`) enters certified Pass at
decision1733, then Recovery at3465 because committed longitudinal progress
stalls. No Return and no certified Stop publication are observed. Thirteen
stateless sibling adoptions occur; the source/world3465 is captured after the
tactical transition, not as the exact prior accepted artifact.

Large velocity-estimation/clock bias is not supported: D1 odometry, derived
position speed and commanded speed agree at about4.3–4.45m/s; D2 odometry,
own-position and V2X-position derivatives agree around3.7m/s. D2's configured
cap is intentionally15km/h (20km/h during its initial15s); D1 is40km/h.

The host course projection is diagnostic, distinct from the runtime projector.
In source-time windows30–40,40–50,50–60,60–70s, D1's physical distances are
44.38,43.13,44.21,39.78m and course progress42.96,40.88,39.80,37.50m.
D2's corresponding distances37.15,36.72,37.57,36.94m and course progress
38.39,37.92,38.93,36.54m. D1's extra lateral travel reduces its progress gain.
Same-time course gap falls27.45m at29.1s to16.40m at65s, then17.31m at72.4s.

Live Pass closing reference remains0.50m/s. The old execution-horizon
feasibility value still feeds committed Pass front-cap release, even though
canonical authority treats that old horizon as reference-only. Frozen-Mission
dependent policy predicates also become false after stateless adoption. This
is a candidate reference-owner mismatch, not a proved physical impossibility
or an accepted fix. Do not spoof a frozen plan or relabel stale proof as current.

Next: prioritize the separately observed single-vehicle stop failure. Before
changing Pass speed/reference, make a same-scene counterfactual with the
correct semantic time and target prediction producer. Do not alter the
progress watchdog, speed cap, weights or wall/peer margins to hide failure.
