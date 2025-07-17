#ifndef WAYPOINT_HPP
#define WAYPOINT_HPP

class Waypoint {
public:
    Waypoint() = default;
    Waypoint(double x, double y, double yaw, double v_ref, double kappa);

    // フィールド
    double x, y;
    double yaw = 0.0;  // 進行方向角（rad）
    double v_ref, kappa;
    double t;  // lateral offset基準の位置？
    double s = 0.0;  // 経路上の累積距離（m）
    double delta_ref_ = 0.0;

    // Getter
    double getX() const;
    double getY() const;
    double getYaw() const;
    double getSpeed() const;
    double getKappa() const;
    double getDeltaRef() const;
};

#endif
