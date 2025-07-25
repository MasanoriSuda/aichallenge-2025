// spatial_bicycle_models.cpp
#include "spatial_bicycle_models.hpp"
#include "reference_path.hpp"

#include <iostream>
#include <numeric>
#include <cmath>

// ========= TemporalState =========
TemporalState::TemporalState(double x_, double y_, double psi_)
    : x(x_), y(y_), psi(psi_) {
    members = {"x", "y", "psi"};
}

TemporalState& TemporalState::operator+=(const Eigen::Vector3d& delta) {
    for (size_t i = 0; i < members.size(); ++i) {
        if (members[i] == "x") x += delta[i];
        else if (members[i] == "y") y += delta[i];
        else if (members[i] == "psi") psi += delta[i];
    }
    return *this;
}

// ========= SimpleSpatialState =========
SimpleSpatialState::SimpleSpatialState(double e_y_, double e_psi_, double t_)
    : e_y(e_y_), e_psi(e_psi_), t(t_) {
    members = {"e_y", "e_psi", "t"};
}

std::vector<std::string> SimpleSpatialState::list_states() const {
    return members;
}

double SimpleSpatialState::get(int index) const {
    if (index == 0) return e_y;
    if (index == 1) return e_psi;
    if (index == 2) return t;
    throw std::out_of_range("SimpleSpatialState get index out of range");
}

void SimpleSpatialState::set(int index, double value) {
    if (index == 0) e_y = value;
    else if (index == 1) e_psi = value;
    else if (index == 2) t = value;
    else throw std::out_of_range("SimpleSpatialState set index out of range");
}

int SimpleSpatialState::size() const {
    return 3;
}

SpatialState& SimpleSpatialState::operator+=(const Eigen::VectorXd& delta) {
    if (delta.size() != 3) throw std::invalid_argument("Delta size must be 3");
    e_y += delta[0];
    e_psi += delta[1];
    t += delta[2];
    return *this;
}

Eigen::Vector3d SimpleSpatialState::to_vector() const {
    return Eigen::Vector3d(e_y, e_psi, t);
}

// ========= SpatialBicycleModel =========
SpatialBicycleModel::SpatialBicycleModel(std::shared_ptr<ReferencePath> reference_path_,
                                         double length_, double width_, double Ts_)
    : reference_path(reference_path_), length(length_), width(width_), Ts(Ts_) {
    //eps = 1e-12;
    safety_margin = _compute_safety_margin();
    s = 0.0;
    wp_id = 0;
    current_waypoint = reference_path->waypoints[wp_id];
    spatial_state = nullptr;
    temporal_state = nullptr;
}

double SpatialBicycleModel::_compute_safety_margin() const {
    return width / std::sqrt(2.0);
}

TemporalState SpatialBicycleModel::s2t(const Waypoint& wp, const Eigen::VectorXd& ref_state) const {
    double x = wp.x - ref_state[0] * std::sin(wp.psi);
    double y = wp.y + ref_state[0] * std::cos(wp.psi);
    double psi = wp.psi + ref_state[1];
    return TemporalState(x, y, psi);
}

SimpleSpatialState SpatialBicycleModel::t2s(const Waypoint& wp, const Eigen::VectorXd& ref_state) const {
    double e_y = std::cos(wp.psi) * (ref_state[1] - wp.y) -
                 std::sin(wp.psi) * (ref_state[0] - wp.x);
    double e_psi = ref_state[2] - wp.psi;
    e_psi = std::fmod(e_psi + M_PI, 2.0 * M_PI);
    if (e_psi < 0) e_psi += 2.0 * M_PI;
    e_psi -= M_PI;
    return SimpleSpatialState(e_y, e_psi, 0.0);
}

void SpatialBicycleModel::drive(const Eigen::Vector2d& u) {
    double v = u[0];
    double delta = u[1];

    double x_dot = v * std::cos(temporal_state->psi);
    double y_dot = v * std::sin(temporal_state->psi);
    double psi_dot = v / length * std::tan(delta);

    Eigen::Vector3d derivatives(x_dot, y_dot, psi_dot);
    *temporal_state += derivatives * Ts;

    double s_dot = 1.0 / (1.0 - spatial_state->e_y * current_waypoint->kappa) *
                   v * std::cos(spatial_state->e_psi);
    s += s_dot * Ts;
}

void SpatialBicycleModel::get_current_waypoint() {
    std::vector<double> length_cum(reference_path->segment_lengths.size());
    std::partial_sum(reference_path->segment_lengths.begin(), reference_path->segment_lengths.end(), length_cum.begin());

    auto it = std::find_if(length_cum.begin(), length_cum.end(), [&](double val) { return val > s; });
    int next_wp_id = std::distance(length_cum.begin(), it);
    int prev_wp_id = std::max(0, next_wp_id - 1);

    double s_next = length_cum[next_wp_id];
    double s_prev = length_cum[prev_wp_id];

    if (std::abs(s - s_next) < std::abs(s - s_prev)) {
        wp_id = next_wp_id;
        current_waypoint = reference_path->waypoints[wp_id];
    } else {
        wp_id = prev_wp_id;
        current_waypoint = reference_path->waypoints[wp_id];
    }
}

// ========= BicycleModel =========
BicycleModel::BicycleModel(std::shared_ptr<ReferencePath> reference_path_,
                           double length_, double width_, double Ts_)
    : SpatialBicycleModel(reference_path_, length_, width_, Ts_) {
    spatial_state = std::make_shared<SimpleSpatialState>();
    n_states = spatial_state->size();
    temporal_state = std::make_shared<TemporalState>(s2t(*current_waypoint, spatial_state->to_vector()));
}

std::tuple<double, double> BicycleModel::get_temporal_derivatives(const Eigen::Vector3d& state,
                                                                  const Eigen::Vector2d& input,
                                                                  double kappa) const {
    double e_y = state[0];
    double e_psi = state[1];
    double v = input[0];
    double delta = input[1];

    double s_dot = 1.0 / (1.0 - e_y * kappa) * v * std::cos(e_psi);
    double psi_dot = v / length * std::tan(delta);
    return {s_dot, psi_dot};
}

Eigen::Vector3d BicycleModel::get_spatial_derivatives(const Eigen::Vector3d& state,
                                                      const Eigen::Vector2d& input,
                                                      double kappa) const {
    double e_y = state[0];
    double e_psi = state[1];
    double v = input[0];
    double delta = input[1];

    auto [s_dot, psi_dot] = get_temporal_derivatives(state, input, kappa);

    double d_e_y_d_s = v * std::sin(e_psi) / s_dot;
    double d_e_psi_d_s = psi_dot / s_dot - kappa;
    double d_t_d_s = 1.0 / s_dot;

    return Eigen::Vector3d(d_e_y_d_s, d_e_psi_d_s, d_t_d_s);
}

void BicycleModel::linearize(double v_ref, double kappa_ref, double delta_s,
                             Eigen::Vector3d& f, Eigen::Matrix3d& A,
                             Eigen::Matrix<double, 3, 2>& B) const {
    A.row(0) = Eigen::Vector3d(1.0, delta_s, 0.0);
    A.row(1) = Eigen::Vector3d(-kappa_ref * kappa_ref * delta_s, 1.0, 0.0);
    A.row(2) = Eigen::Vector3d(-kappa_ref / v_ref * delta_s, 0.0, 1.0);

    B.row(0) = Eigen::Vector2d(0.0, 0.0);
    B.row(1) = Eigen::Vector2d(0.0, delta_s);
    B.row(2) = Eigen::Vector2d(-1.0 / (v_ref * v_ref) * delta_s, 0.0);

    f = Eigen::Vector3d(0.0, 0.0, 1.0 / v_ref * delta_s);
}

void BicycleModel::set_pose_from_odom(const OdometryInput& odom) {
    this->temporal_state->x = odom.x;
    this->temporal_state->y = odom.y;
    this->temporal_state->psi = odom.yaw;
    this->temporal_state->v = odom.v;
}