# Design

## Evidence path

1. Replay the recorded QP warm and cold to establish deterministic source
   solver behavior.
2. Probe the recorded solution at publication-relevant elapsed times with the
   prepared-suffix comparison.
3. Compare old-origin, time-aligned, reachable bridge, bounded multi-SQP,
   structured/nonlinear interior wall and proof-guided variants.
4. Record which arm first restores an exact physically certified suffix.
5. If the target-free Cruise snapshot remains unclassifiable, extend only the
   offline comparison input contract. Do not connect any new arm to production.

## Exit classification

- Time-aligned succeeds: execution-clock/origin defect.
- Reachable bridge succeeds: candidate-generation defect.
- Multi-SQP succeeds: single-SQP limitation.
- Interior/proof-guided succeeds: wall representation/certificate mismatch.
- All complete physical arms fail: bounded evidence for physical
  infeasibility, not a global proof.
- Live fails while same-snapshot offline succeeds: scheduling/lifecycle
  defect.
