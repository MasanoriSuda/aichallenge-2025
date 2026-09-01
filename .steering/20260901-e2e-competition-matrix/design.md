# Design

Use the production runtime without overrides:

1. deterministic single vehicle (`make e2e-single`)
2. one ego plus two runtime NPCs (`make e2e-npc-single`, seed 2026)
3. four production peers (`make e2e-final`)

For each finalized domain, generate `e2e-run-analysis.json`, then run
`analyze_e2e_competition.py`.  Keep motion, race result, penalties, and runtime
provenance as separate fields in the output so one success signal cannot mask a
failure in another layer.

The peer run is diagnostic.  Existing evidence predicts symmetric lateral
contact failure; reproducing it under the new provenance contract is useful,
but does not justify training until the failed pre-contact state distribution
is isolated.
