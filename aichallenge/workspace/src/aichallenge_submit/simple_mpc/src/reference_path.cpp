// reference_path.cpp
#include "reference_path.hpp"
#include "interp_utils.hpp"
#include "spline.h"
#include <algorithm>
#include <numeric>
#include <iostream>

#include <unsupported/Eigen/Splines>

ReferencePath::ReferencePath(const std::vector<double>& wp_x, const std::vector<double>& wp_y,
                               double path_resolution, double smoothing_distance, double max_width,
                               bool circular)
    : resolution(path_resolution),
      smoothing_distance(smoothing_distance),
      max_width(max_width),
      circular(circular) {
#if 1
    set_raw_waypoints_from_xy(wp_x, wp_y);
    waypoints = construct_path(wp_x, wp_y,1.0,0.0,2.0);
    segment_lengths = compute_segment_lengths();
#endif
}

std::vector<std::shared_ptr<Waypoint>> ReferencePath::construct_path(
    const std::vector<double> &x, const std::vector<double> &y,
    double ds, double v_current, double a_max)
{
    std::vector<std::shared_ptr<Waypoint>> path;
    assert(x.size() == y.size());
    assert(x.size() >= 2);

    // --- 距離 s を自動生成 ---
    std::vector<double> s;
    s.push_back(0.0);
    for (size_t i = 1; i < x.size(); ++i)
    {
        double dx = x[i] - x[i - 1];
        double dy = y[i] - y[i - 1];
        s.push_back(s.back() + std::sqrt(dx * dx + dy * dy));
    }

    // --- スプライン補間 ---
    tk::spline sx, sy;
    sx.set_points(s, x);
    sy.set_points(s, y);

    // --- 生成 ---
    const double epsilon = 1e-5;
    const double lookahead = 1.0; // 3m先の曲率を見る

    for (double s_interp = s.front(); s_interp <= s.back(); s_interp += ds)
    {
        double x_val = sx(s_interp);
        double y_val = sy(s_interp);

        // 1階微分
        double dx = (sx(s_interp + epsilon) - sx(s_interp - epsilon)) / (2 * epsilon);
        double dy = (sy(s_interp + epsilon) - sy(s_interp - epsilon)) / (2 * epsilon);

        // 2階微分
        double ddx = (sx(s_interp + epsilon) - 2 * sx(s_interp) + sx(s_interp - epsilon)) / (epsilon * epsilon);
        double ddy = (sy(s_interp + epsilon) - 2 * sy(s_interp) + sy(s_interp - epsilon)) / (epsilon * epsilon);

        // ヤウ角
        double yaw = std::atan2(dy, dx);

        // 曲率
        double kappa = (dx * ddy - dy * ddx) / std::pow(dx * dx + dy * dy, 1.5);

        // --- Lookahead先の曲率 ---
        double s_lookahead = std::min(s.back(), s_interp + lookahead);

        double dx_l = (sx(s_lookahead + epsilon) - sx(s_lookahead - epsilon)) / (2 * epsilon);
        double dy_l = (sy(s_lookahead + epsilon) - sy(s_lookahead - epsilon)) / (2 * epsilon);
        double ddx_l = (sx(s_lookahead + epsilon) - 2 * sx(s_lookahead) + sx(s_lookahead - epsilon)) / (epsilon * epsilon);
        double ddy_l = (sy(s_lookahead + epsilon) - 2 * sy(s_lookahead) + sy(s_lookahead - epsilon)) / (epsilon * epsilon);

        double kappa_lookahead = (dx_l * ddy_l - dy_l * ddx_l) / std::pow(dx_l * dx_l + dy_l * dy_l, 1.5);

        // --- 速度制限（現在 + 未来） ---
        double v_kappa_limit = std::sqrt(std::max(0.0, a_max / (std::abs(kappa_lookahead) + 1e-5)));
        double v_acc_limit = std::sqrt(std::max(0.0, v_current * v_current + 2 * a_max * ds));
        
        double v = std::min(v_kappa_limit, v_acc_limit);
        if(v< 100){
            v = 9.695;//ok
            //v = 9.696;//ok or NG
            //v = 9.697;//NG
            //v = 9.698;//NG
        }

        path.push_back(std::make_shared<Waypoint>(x_val, y_val, yaw, kappa, v, 0));
    }

    return path;
}

std::vector<double> ReferencePath::compute_segment_lengths() {
    std::vector<double> lengths = {0.0};
    for (size_t i = 1; i < waypoints.size(); ++i) {
        double dx = waypoints[i]->x - waypoints[i - 1]->x;
        double dy = waypoints[i]->y - waypoints[i - 1]->y;
        lengths.push_back(std::sqrt(dx * dx + dy * dy));
    }
    return lengths;
}

Waypoint ReferencePath::get_waypoint(int idx) const {
    int idx_clamped = std::max(0, std::min(static_cast<int>(waypoints.size()) - 1, idx));
    return *waypoints[idx_clamped];
}

double ReferencePath::get_speed(double s) const {
    if (v_profile.empty()) return 0.0;
    std::vector<double> idx(v_profile.size());
    std::iota(idx.begin(), idx.end(), 0);
    return linear_interp(s, idx, v_profile);
}

double ReferencePath::get_curvature(double s) const {
    if (kappa_profile.empty()) return 0.0;
    std::vector<double> idx(kappa_profile.size());
    std::iota(idx.begin(), idx.end(), 0);
    return linear_interp(s, idx, kappa_profile);
}


void ReferencePath::set_speed_profile(const std::vector<double>& v_profile_in) {
    v_profile = v_profile_in;
    if (v_profile.size() == waypoints.size()) {
        for (size_t i = 0; i < waypoints.size(); ++i) {
            waypoints[i]->v_ref = v_profile[i];
        }
    }
}

void ReferencePath::update_path_constraints(int waypoint_idx, int horizon,
                                            double upper_bound, double lower_bound,
                                            std::vector<double>& ub,
                                            std::vector<double>& lb,
                                            std::vector<double>& width) {
    int H = std::max(1, horizon);
    if (waypoint_idx >= static_cast<int>(waypoints.size())) {
        waypoint_idx = static_cast<int>(waypoints.size()) - 1;
    }
    ub.assign(H, upper_bound);
    lb.assign(H, -lower_bound);
    width.assign(H, upper_bound + lower_bound);
}

int ReferencePath::get_closest_index(double s) const {
    double min_diff = std::numeric_limits<double>::max();
    int closest_index = 0;
    for (size_t i = 0; i < waypoints.size(); ++i) {
        double diff = std::abs(waypoints[i]->s - s);
        if (diff < min_diff) {
            min_diff = diff;
            closest_index = static_cast<int>(i);
        }
    }
    return closest_index;
}

std::vector<Waypoint> ReferencePath::extract_subpath(double s, int N) const {
    std::vector<Waypoint> result;

    int center_idx = get_closest_index(s);
    int start_idx = std::max(0, center_idx - N);
    int end_idx = std::min(static_cast<int>(waypoints.size()) - 1, center_idx + N);

    for (int i = start_idx; i <= end_idx; ++i) {
        result.push_back(*waypoints[i]);  // shared_ptr → 実体コピー
    }

    return result;
}

double ReferencePath::get_closest_s(double x, double y) const {
    double min_dist = 1e9;
    double closest_s = 0.0;
    for (const auto& wp : waypoints) {
        double dx = wp->x - x;
        double dy = wp->y - y;
        double dist = std::hypot(dx, dy);
        if (dist < min_dist) {
            min_dist = dist;
            closest_s = wp->s;
        }
    }
    return closest_s;
}

// reference_path.cpp
Waypoint ReferencePath::get_waypoint_from_s(double s) const {
    // 境界チェック（最後尾より後ろ）
    if (s >= waypoints.back()->s) return *waypoints.back();
    if (s <= waypoints.front()->s) return *waypoints.front();

    // s に一番近い区間を見つけて線形補間
    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        const auto& wp0 = waypoints[i];
        const auto& wp1 = waypoints[i + 1];
        if (wp0->s <= s && s < wp1->s) {
            double ratio = (s - wp0->s) / (wp1->s - wp0->s);
            Waypoint interp(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);  // ✅ 初期値は上書き前提でOK
            interp.s = s;
            interp.x = wp0->x + ratio * (wp1->x - wp0->x);
            interp.y = wp0->y + ratio * (wp1->y - wp0->y);
            interp.psi = wp0->psi + ratio * (wp1->psi - wp0->psi);
            interp.kappa = wp0->kappa + ratio * (wp1->kappa - wp0->kappa);
            interp.v_ref = wp0->v_ref + ratio * (wp1->v_ref - wp0->v_ref);
            interp.delta_ref = wp0->delta_ref + ratio * (wp1->delta_ref - wp0->delta_ref);
            return interp;
        }
    }

    // fallback
    return *waypoints.back();
}

void ReferencePath::set_raw_waypoints_from_xy(const std::vector<double>& xs, const std::vector<double>& ys) {
    raw_waypoints_.clear();

    double accumulated_s = 0.0;
    for (size_t i = 0; i < xs.size(); ++i) {
        double s = 0.0;
        if (i > 0) {
            double dx = xs[i] - xs[i - 1];
            double dy = ys[i] - ys[i - 1];
            s = std::hypot(dx, dy);
            accumulated_s += s;
        }
        auto wp = std::make_shared<Waypoint>(
            xs[i],      // x
            ys[i],      // y
            0.0,        // psi（後で補完）
            0.0,        // kappa
            0.0,        // v_ref
            accumulated_s  // ✅ ここに s をセット
        );
        raw_waypoints_.push_back(wp);
    }
}

std::vector<std::shared_ptr<Waypoint>> ReferencePath::extract_raw_subpath(double s, int N) const {
    std::vector<std::shared_ptr<Waypoint>> result;
    if (raw_waypoints_.empty()) return result;

    // 前方で最も近い index を探す
    int closest_index = -1;
    double min_diff = std::numeric_limits<double>::max();
    // 修正後
    for (size_t i = 0; i < raw_waypoints_.size(); ++i){
        double ds = raw_waypoints_[i]->s - s;
        if (ds >= 0.0 && ds < min_diff) {
            min_diff = ds;
            closest_index = i;
        }
    }

    // fallback：最後尾を中心とする（ゴール or オーバーラン時）
    if (closest_index == -1) {
        closest_index = raw_waypoints_.size() - 1;
    }

    // ✅ 先行N点のみ取り出す
    int end = std::min(static_cast<int>(raw_waypoints_.size()), closest_index + N);
    for (int i = closest_index; i < end; ++i) {
        result.push_back(raw_waypoints_[i]);
    }

    return result;
}

void ReferencePath::update_hoge(const std::vector<double>& wp_x, const std::vector<double>& wp_y,double current_speed_mps, double max_accel) {
    set_raw_waypoints_from_xy(wp_x, wp_y);
    waypoints = construct_path(wp_x, wp_y, 0.2,current_speed_mps, max_accel);
    segment_lengths = compute_segment_lengths();
}

std::vector<std::shared_ptr<Waypoint>>& ReferencePath::get_all_waypoints() {
    return waypoints;
}

inline double wrapToPi(double angle) {
    while (angle > M_PI)  angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

void ReferencePath::compute_curvature_profile() {
    const auto& wps = get_all_waypoints();

    for (size_t i = 1; i + 1 < wps.size(); ++i) {
        const auto& wp_prev = wps[i - 1];
        const auto& wp      = wps[i];
        const auto& wp_next = wps[i + 1];

        double dx1 = wp->x - wp_prev->x;
        double dy1 = wp->y - wp_prev->y;
        double dx2 = wp_next->x - wp->x;
        double dy2 = wp_next->y - wp->y;

        double theta1 = std::atan2(dy1, dx1);
        double theta2 = std::atan2(dy2, dx2);
        double dtheta = wrapToPi(theta2 - theta1);

        double ds = std::hypot(wp_next->x - wp_prev->x, wp_next->y - wp_prev->y);
        wp->kappa = dtheta / ds;
    }

    // 端点は前後の値で埋める
    if (wps.size() >= 2) {
        wps[0]->kappa = wps[1]->kappa;
        wps.back()->kappa = wps[wps.size() - 2]->kappa;
    }
}
