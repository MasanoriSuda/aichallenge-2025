#include "waypoint.hpp"

Waypoint::Waypoint(double x, double y, double yaw, double v_ref, double kappa)
    : x(x), y(y), yaw(yaw), v_ref(v_ref), kappa(kappa), t(0.0) {}

double Waypoint::getX() const { return x; }
double Waypoint::getY() const { return y; }
double Waypoint::getYaw() const { return yaw; }
double Waypoint::getSpeed() const { return v_ref; }
double Waypoint::getKappa() const { return kappa; }
double Waypoint::getDeltaRef() const { return delta_ref_; }
