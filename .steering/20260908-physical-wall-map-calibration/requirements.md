# Local physical wall map calibration

Earliest broken invariant: a reconstructed solid wall/body intersection at
source250.65s lies in the production map free cell. This does not depend on
the later Recovery or Stop failure. The current Lanelet2-derived raster is
not a faithful representation of this local collision mesh.

Add the conservative XY supercover of nondegenerate static faces with
abs(normal.z)<=0.5 to the existing occupancy. Keep every existing occupied
cell. All heights are projected; this can be over-conservative and does not
claim complete3D-world coverage of omitted slopes/other colliders.

The reference-path/corridor Map.data producer drops occupied components smaller
than5cells. The candidate contains one isolated cell. The final physical wall
world already reads raw_normalized_data and retains it; this is a reference/
physical-world mismatch, not loss of that cell in final physical proof. For explicitly calibrated maps,
add preserve_occupied_cells:true metadata to retain authoritative occupancy.
Other maps keep legacy processing unless explicitly opted in. Editor and
native runtime must agree. No thresholds/margins, speeds, geometry extents,
normal authority or certification criteria are changed.

Before change, reproduce missing contact coverage and native small-cell
loss. After change, test contact witnesses, preservation, actual native Map
behavior, editor parity, reference swept clearance, build/package checks,
and new-map dynamic diagnostic. Baseline is a8b968ac plus the sealed
20260908-footprint-single-r1 binaries and participant patch.
