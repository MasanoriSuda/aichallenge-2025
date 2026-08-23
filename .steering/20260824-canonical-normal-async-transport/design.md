# Design

The mailbox is not a controller and must not decide behavior. It transports an
immutable plan plus a sealed identity. The identity therefore names the exact
canonical normal intent; validators compare that same intent at submission,
worker result publication and live consumption.

The generic implementation is exposed as `canonical_normal_async`. The old
`follow_canonical_async` namespace remains an alias so the accepted Follow
producer does not change behavior in this refactor Slice. The alias is
temporary migration compatibility, not a second implementation.

Supported normal intents are Track, Cruise, Follow, ShiftOut, Pass and Return.
Stop is an emergency supervisor intent in the current architecture; Unknown is
never executable. Both fail closed.
