#include "spatial_bicycle_models.hpp"
#include <cmath>
#include <iostream>

// ========== SpatialBicycleModel ==========

SpatialBicycleModel::SpatialBicycleModel(std::shared_ptr<ReferencePath> reference_path, double length, double width, double Ts)
    : reference_path(reference_path), length(length), width(width), Ts(Ts), s(0.0), wp_id(0), eps(1e-12) {
    safety_margin = compute_safety_margin();
    current_waypoint = reference_path->waypoints[wp_id];
}

double SpatialBicycleModel::compute_safety_margin() const {
    return width / std::sqrt(2.0);
}

TemporalState SpatialBicycleModel::s2t(const Waypoint& ref_wp, const SimpleSpatialState& ref_state) {
    double x = ref_wp.x - ref_state.e_y * std::sin(ref_wp.psi);
    double y = ref_wp.y + ref_state.e_y * std::cos(ref_wp.psi);
    double psi = ref_wp.psi + ref_state.e_psi;
    return TemporalState(x, y, psi);
}

SimpleSpatialState SpatialBicycleModel::t2s(const Waypoint& ref_wp, const TemporalState& ref_state) {
    double dx = ref_state.x - ref_wp.x;
    double dy = ref_state.y - ref_wp.y;
    double e_y = std::cos(ref_wp.psi) * dy - std::sin(ref_wp.psi) * dx;
    double e_psi = std::fmod(ref_state.psi - ref_wp.psi + M_PI, 2 * M_PI) - M_PI;
    return SimpleSpatialState(e_y, e_psi, 0.0);
}

void BicycleModel::drive(const Eigen::Vector2d& u) {
    double v = u[0];
    double delta = u[1];

    double x_dot = v * std::cos(this->temporal_state.psi);
    double y_dot = v * std::sin(this->temporal_state.psi);
    double psi_dot = v / this->length * std::tan(delta);

    this->temporal_state.x += x_dot * this->Ts;
    this->temporal_state.y += y_dot * this->Ts;
    this->temporal_state.psi += psi_dot * this->Ts;

    double s_dot = (1.0 / (1.0 - this->spatial_state.e_y * this->current_waypoint->kappa))
                   * v * std::cos(this->spatial_state.e_psi);

    this->s += s_dot * this->Ts;
}

void SpatialBicycleModel::update_current_waypoint() {
    auto& segment_lengths = reference_path->get_segment_lengths();
    std::vector<double> length_cum(segment_lengths.size());
    length_cum[0] = segment_lengths[0];
    for (size_t i = 1; i < segment_lengths.size(); ++i) {
        length_cum[i] = length_cum[i - 1] + segment_lengths[i];
    }
    size_t next_wp = 0;
    while (next_wp < length_cum.size() && length_cum[next_wp] <= s) {
        ++next_wp;
    }
    size_t prev_wp = (next_wp == 0) ? 0 : next_wp - 1;
    if (next_wp >= reference_path->waypoints.size()) next_wp = reference_path->waypoints.size() - 1;
    if (prev_wp >= reference_path->waypoints.size()) prev_wp = reference_path->waypoints.size() - 1;

    if (std::abs(s - length_cum[next_wp]) < std::abs(s - length_cum[prev_wp])) {
        wp_id = next_wp;
    } else {
        wp_id = prev_wp;
    }
    current_waypoint = reference_path->waypoints[wp_id];
}

// ========== BicycleModel ==========

BicycleModel::BicycleModel(std::shared_ptr<ReferencePath> reference_path, double length, double width, double Ts)
    : SpatialBicycleModel(reference_path, length, width, Ts) {
    spatial_state = SimpleSpatialState();
    temporal_state = s2t(*current_waypoint, spatial_state);
    n_states = 3;
}

std::tuple<double, double> BicycleModel::get_temporal_derivatives(
    const SimpleSpatialState& state,
    const Eigen::Vector2d& input,
    double kappa){
    double e_y = state.e_y;
    double e_psi = state.e_psi;
    double v = input[0];
    double delta = input[1];

    double s_dot = 1.0 / (1.0 - (e_y * kappa)) * v * std::cos(e_psi);
    double psi_dot = v / length * std::tan(delta);

    return {s_dot, psi_dot};
}

Eigen::Vector3d BicycleModel::get_spatial_derivatives(const SimpleSpatialState& state,
                                                      const Eigen::Vector2d& input,
                                                      double kappa) {
    double e_y   = state.e_y;
    double e_psi = state.e_psi;

    double v     = input[0];
    double delta = input[1];

    double s_dot = 1.0 / (1.0 - (e_y * kappa)) * v * std::cos(e_psi);
    double psi_dot = v / this->length * std::tan(delta);

    return Eigen::Vector3d{ -v * std::sin(e_psi),
                             psi_dot,
                             s_dot };
}

std::tuple<Eigen::Vector3d, Eigen::Matrix3d, Eigen::Matrix<double, 3, 2>> BicycleModel::linearize(double v_ref, double kappa_ref, double delta_s) {
    Eigen::Vector3d f(0.0, 0.0, 1.0 / v_ref * delta_s);

    Eigen::Matrix3d A;
    A << 1, delta_s, 0,
        -kappa_ref * kappa_ref * delta_s, 1, 0,
        -kappa_ref / v_ref * delta_s, 0, 1;

    Eigen::Matrix<double, 3, 2> B;
    B << 0, 0,
        0, delta_s,
        -1.0 / (v_ref * v_ref) * delta_s, 0;

    return {f, A, B};
}
