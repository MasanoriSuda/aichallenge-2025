# Design: make DynamicWait a semantic no-transition

## Alternatives

1. Keep the last prefix alive for one more cycle: rejected; this is a grace
   rule and retains the obsolete producer.
2. Generate the optional prefix earlier: rejected; update ordering still owns
   safety semantics and the first cycle can fail again under load.
3. Treat DynamicWait as Cruise until a prefix exists: rejected; this changes
   the admitted Mission intent and creates another ShiftOut-to-Cruise handoff.
4. Resolve the interrupted intent from the already validated canonical
   execution identity, then require ordinary current-world proof: selected.

## Ownership after the change

- DynamicWait owns no lateral command and creates no path authority.
- The optional forward prefix remains candidate/reference provenance only.
- The canonical execution identity owns target, Mission generation, homotopy
  and interrupted ShiftOut/Pass semantics.
- The bounded current-world candidate population, seven-state solve, exact
  wall/opponent proof, terminal successor proof, Store and atomic publisher own
  the actual command.

`AuthorityRequest` carries an explicit
`canonical_execution_identity_active` bit from the resolver. DynamicWait may
preserve ShiftOut or Pass only when this bit is true and target, generation,
side, current canonical phase and origin phase are coherent. The bit does not
grant authority and is never retained by age.

## Atomic deletion

Remove `dynamic_wait_lateral_authority_active`, the
`DynamicWaitPrefix` lateral-owner enum member, the corresponding conflict and
the resolver requirement that the optional prefix own both path and lateral
authority. Replace prefix-dependent intent reasons with canonical-execution
DynamicWait reasons.

## Falsifiers

- a DynamicWait with no canonical execution identity produces normal intent;
- an identity/origin phase mismatch produces normal intent;
- an optional prefix becomes sufficient to publish without current-world
  proof;
- decision 1473's old Unknown-intent signature remains;
- hard DynamicWait Recovery or Emergency fail-closed behavior is weakened.
