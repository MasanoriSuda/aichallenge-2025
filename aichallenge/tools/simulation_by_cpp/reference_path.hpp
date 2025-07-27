#pragma once

#include <vector>
#include <memory>
#include <cmath>
#include "waypoint.hpp"  // ← これを忘れずに！

class ReferencePath {
public:
    ReferencePath() = default;
    ~ReferencePath() = default;

    std::vector<std::shared_ptr<Waypoint>> waypoints;

    std::shared_ptr<Waypoint> get_current_waypoint() const {
        if (!waypoints.empty()) {
            return waypoints.front();  // もしくは最初のWayPoint
        } else {
            return std::make_shared<Waypoint>();  // 空のWaypointを返す fallback
        }
    }


    std::shared_ptr<Waypoint> get_waypoint(double s) {
        if (waypoints.empty()) {
            return std::make_shared<Waypoint>();
        }

        if (s >= waypoints.back()->t) {
            return waypoints.back();
        }

        for (size_t i = 0; i < waypoints.size() - 1; ++i) {
            double t0 = waypoints[i]->t;
            double t1 = waypoints[i + 1]->t;
            if (s >= t0 && s < t1) {
                double ratio = (s - t0) / (t1 - t0);
                auto w0 = waypoints[i];
                auto w1 = waypoints[i + 1];

                auto interp = std::make_shared<Waypoint>();
                interp->x = w0->x + ratio * (w1->x - w0->x);
                interp->y = w0->y + ratio * (w1->y - w0->y);
                interp->yaw = w0->yaw + ratio * (w1->yaw - w0->yaw);
                interp->v_ref = w0->v_ref + ratio * (w1->v_ref - w0->v_ref);
                interp->kappa = w0->kappa + ratio * (w1->kappa - w0->kappa);
                interp->delta_ref_ = w0->delta_ref_ + ratio * (w1->delta_ref_ - w0->delta_ref_);  // ← 追加
                interp->t = s;
                return interp;
            }
        }

        return waypoints.back();
    }


    void construct_path(const std::vector<double>& x_list,
                            const std::vector<double>& y_list);

    // set_points はインライン定義をやめて、宣言だけにする
        void set_points(const std::vector<std::shared_ptr<Waypoint>>& wp) {
        // ★ 各点に delta_ref_ を初期化（長さ2.0mのバイシクルモデル想定）
        for (auto& pt : wp) {
            if (std::abs(pt->kappa) > 1e-6) {
                pt->delta_ref_ = std::atan(2.0 * pt->kappa);  // L=2.0m
            } else {
                pt->delta_ref_ = 0.0;
            }
        }

        // ✔️ スムージング処理（オプション）
        for (int i = 1; i < wp.size() - 1; ++i) {
            wp[i]->delta_ref_ =
                0.25 * wp[i - 1]->delta_ref_ +
                0.5  * wp[i]->delta_ref_ +
                0.25 * wp[i + 1]->delta_ref_;
        }

        waypoints = wp;
    }
     void set_speed_profile(double default_speed);  // ← ★追加！

    double get_speed(double s) const;  // ★ const 修正済み

    std::shared_ptr<Waypoint> get_waypoint(double s) const;  // ← const をつける
                            // 必要ならあとで関数を追加
};
