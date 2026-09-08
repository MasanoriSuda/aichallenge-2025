# Validation boundary

Production change: adapter tangent selection and two native tests only.
`/tmp/mpcc-rear-peer-tangent-before.log` preserves the expected failing case;
after.log has19passes. Full build25packages and scoped package test2364records
pass. Earlier outcome counts are not reused as this patch's evidence.

Same-world comparison uses the built executable and actual immutable world1629.
Single/dev2 were run sequentially after tests, with no production edits or
heavy replay while simulation ran. The final run rejects; it is not a gate pass.
Read-only rosbag analysis runs after teardown in the same Docker environment.

Validation logs, build commands, instrumented source copies and raw results are
hashed by seal_evidence.py. YAML comparison reports retain all arm outcomes.
Frontmatter/Skills are unchanged in this slice. Pre-commit is unavailable in
this environment; no hook execution is claimed. Python AST, JSON, changed
Markdown local links and `git diff --check` are checked before commit.
