#ifndef TRAJECTORY_POINT_HPP
#define TRAJECTORY_POINT_HPP

struct TrajectoryPoint {
    double x, y, z;
    double x_quat, y_quat, z_quat, w_quat;
    double speed;

    // 🔽 以下を追加！
    double v = 0.0;       // alias for speed（どちらかに統一してもOK）
    double yaw = 0.0;     // クォータニオンから変換
    double kappa = 0.0;   // 曲率（あとで補完 or 差分で計算）
};

#endif  // TRAJECTORY_POINT_HPP
