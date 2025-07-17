#pragma once

#include <vector>
#include <memory>
#include "waypoint.hpp"  // ← これを忘れずに！

class ReferencePath {
public:
    ReferencePath() = default;
    ~ReferencePath() = default;

    std::vector<std::shared_ptr<Waypoint>> waypoints;

    void set_points(const std::vector<std::shared_ptr<Waypoint>>& wp) {
        waypoints = wp;
    }

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

    void set_speed_profile(double default_speed);


                            // 必要ならあとで関数を追加
};
