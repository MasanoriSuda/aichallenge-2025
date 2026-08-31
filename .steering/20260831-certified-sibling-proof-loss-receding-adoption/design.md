# Design

## Root cause

The sibling adoption boundary mixed two different kinds of evidence:

1. exact current-world physical/certificate evidence, and
2. retained Mission tactics (`selected_homotopy_established`, `no-return`, and
   replacement-count budget).

When the current selected homotopy lost current-world production authority,
the second group vetoed a sibling that had stronger, exact physical proof.
That retained an obsolete path and propagated a local branch failure into
global Stop authority.

## Invariant

For active ShiftOut/Pass, when the selected side has no current-world authority
and the exact opposite side from the same immutable epoch has production
authority, the physical proof owns the replacement decision. Mission history
does not constitute a safety proof and cannot veto it.

## Change

- Remove Mission-history fields from the sibling adoption request and
  publisher token revalidation API.
- Keep selected-current-world availability, active intent, hard-fault,
  production authority, stateless origin, exact epoch identity, live tactical
  identity, and exact opposite-side checks.
- Keep the publisher-bound token and atomic geometry retirement.
- Leave legacy Mission flags available to legacy planning diagnostics; they no
  longer own this certified proof-loss handoff.

## Why this is not a new fallback

The sibling is not a retained command or an emergency approximation. It is a
normal seven-state solution generated from the same immutable current-world
problem and accepted by the existing production certificate chain. This change
only prevents older Mission history from overriding that evidence.

