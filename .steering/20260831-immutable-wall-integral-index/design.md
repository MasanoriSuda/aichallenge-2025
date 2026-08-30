# Design: immutable wall integral index

Build a summed-area table of non-free cells after the controller deep-copies
the configured wall grid.  The resulting grid is then exposed only through a
`shared_ptr<const OccupancyGrid>` and shared by tactical snapshots.

For each footprint pose:

1. compute the same conservative AABB as today;
2. query the summed-area table;
3. if the AABB contains zero non-free cells, return clear for that pose;
4. otherwise execute the unchanged occupied/unknown-cell exact intersection.

The index is not part of occupancy identity.  It is an acceleration structure
for the same immutable cells.  Unprepared grids retain the existing exact
fallback, which keeps unit-test and external API behavior compatible.

The prepared grid is shared as const rather than copied into every tactical
snapshot.  This removes both cache reconstruction and derived-index copying
while retaining the same grid fingerprint and cell content.
