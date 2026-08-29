# Requirements

## Objective

Determine whether the repeated `invalid-lateral-bounds` exact-proof failures
are caused by the QP's affine interior wall representation rather than Mission
lifecycle, candidate generation, SQP count or physical infeasibility.

## Frozen evidence

- ShiftOut sequence 1266: A/B/C and four D solves all reach exact proof, then
  fail at dense stage 339 by about 2.27 mm over the upper interval;
- Follow sequence 531: the same family fails at dense stage 473 by about
  0.02 mm;
- the canonical stage transition and exact proof already share midpoint
  integration with substeps no longer than 10 ms;
- the QP wall contract uses endpoint boxes plus four affine-interpolated
  interior rows, while exact proof checks true nonlinear lateral state at
  every integration substep.

## Audit-E contract

- keep C's latest x0, semantics, costs, inputs, endpoint boxes, wall intervals,
  obstacle rows, identity and terminal successor;
- use the same shared maximum physical integration step as exact proof;
- at every physical substep inside a stage, linearize the true nonlinear
  intermediate lateral state with respect to that stage's source state/input;
- append those rows to the offline QP without replacing or loosening any
  existing row;
- after every correction, rebuild both endpoint dynamics tangents and the
  nonlinear interior rows around the new primal;
- require the unchanged exact artifact/physical proof for acceptance;
- expose no Store, worker, publisher or command path.

## Prohibited changes

- no clearance, tolerance, cost, steering, speed or solver setting change;
- no production authority or fallback connection;
- no removal of the existing endpoint/swept wall rows;
- no clamping of solved states or proof results;
- no success claim from QP feasibility alone.

## Acceptance

- tests prove the shared integration-step contract;
- every appended row is tangent to a canonical nonlinear intermediate state;
- existing rows and bounds remain bit-for-bit unchanged as a prefix;
- the frozen ShiftOut/Follow probes either pass unchanged exact proof or remain
  explicitly classified;
- full build and package regression pass.
