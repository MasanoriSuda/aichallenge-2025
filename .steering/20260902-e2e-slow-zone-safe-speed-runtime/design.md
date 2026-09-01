# Design

`speed_aware_lidar_brake` is a separate experimental control mode.  It retains
ML-only lateral authority and the existing pace governor.  Its longitudinal
authority is stateless and is evaluated from the current LiDAR frame and fresh
wheel speed only.

Above the existing 3.0 m slow boundary it returns the incoming pace request
unchanged.  At or below 1.5 m it preserves the existing hard brake.  Between
those boundaries it solves the maximum speed supported by the remaining
clearance and limits acceleration using the safe-speed error.  A zero-speed
vehicle therefore receives a small bounded request when physical clearance is
positive, instead of being an equilibrium of a zero-acceleration plateau.

The old fixed policy remains selectable for an immutable A/B.  No lateral
model, checkpoint or clearance threshold changes in this Slice.
