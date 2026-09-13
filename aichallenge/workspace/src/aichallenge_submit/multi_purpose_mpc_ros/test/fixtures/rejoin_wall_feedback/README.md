# Exact Rejoin wall feedback fixtures

`rejoin537.yaml`: original single-r23 / 226e4f84 / source537 / policy2.
`current_contact527.yaml`: original single-r18 / 9c522a95 / source527 / policy0;
the test uses the existing population builder to obtain its policy2candidate.
Both retain the original snapshot payload and interaction fingerprint.

`wall-grid.rle` losslessly represents their identical row-major int8grid. Header:
first cell value and total cell count; remaining positive run lengths alternate0/1.
Tests decode into a private temporary `wall-grid.bin` for the original loader.
Original binary SHA256: 190563b756797efb857b7e5155017cdbcdcfee615f6975802e373edfac81f13c.
The fixture changes no grid coordinates, unknown cells, footprint, margin, model,
state/input constraints, reference or original solver settings.

`wall_exhaustion622.yaml`: original single-r8 source622/policy1. Its wall99
rejection remains after the maximum three corrections; it cannot execute.
