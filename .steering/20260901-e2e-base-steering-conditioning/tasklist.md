# Tasklist

- [x] Reproduce wheel-v4 NPC seed 2027 failure.
- [x] Replay v3/v4 with wheel and fused speed.
- [x] Compare both candidates with the admitted teacher.
- [x] Identify the missing base-steering conditioning contract.
- [x] Add and test offline base-steering-conditioned model support.
- [x] Train a wheel-speed candidate with the corrected feature contract.
- [x] Pass bounded aggregate, peer, held-out and normal-anchor gates.
- [x] Add runtime loading only after offline qualification.
- [x] Pass single and two-seed NPC closed-loop gates with the successor
  full-range candidate. The bounded v10 candidate failed because
  the `0.12 rad` representation/authority bound could not express the admitted
  teacher's `0.83--0.88 rad` side-clearance correction.
- [x] Promote only the fully qualified full-range successor artifact; v10
  remains rejected.
