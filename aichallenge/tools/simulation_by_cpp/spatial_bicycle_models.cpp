#include "spatial_bicycle_models.hpp"
#include "reference_path.hpp"   // ← ちゃんと中身を見る必要がある！
#include "waypoint.hpp"         // ← 中身に .x, .y, .yaw などを使ってるので必要！
#include <cmath>
#include <iostream>
#include <Eigen/Dense>


using Eigen::Vector2d;
using Eigen::Vector3d;
using std::cout;
using std::endl;

// --- TemporalState -----------------------------------
TemporalState& TemporalState::operator+=(const Vector3d& delta) {
    x += delta[0];
    y += delta[1];
    yaw += delta[2];
    return *this;
}

// --- SimpleSpatialState -------------------------------

SimpleSpatialState::SimpleSpatialState(double e_y, double e_yaw, double t)
    : e_y(e_y), e_yaw(e_yaw), t(t) {}

Vector3d SimpleSpatialState::toVector() const {
    return Vector3d(e_y, e_yaw, t);
}

SpatialState& SimpleSpatialState::operator+=(const Eigen::VectorXd& delta) {
    e_y += delta[0];
    e_yaw += delta[1];
    t += delta[2];

    return *this;
}

// --- SpatialBicycleModel -------------------------------

SpatialBicycleModel::SpatialBicycleModel(std::shared_ptr<ReferencePath> ref_path, double len, double wid, double Ts_)
    : reference_path(ref_path),
      length(len),
      width(wid),
      Ts(Ts_),
      s(0.0),
      wp_id(0),
      eps(1e-12)
{
    safety_margin = width / std::sqrt(2.0);
    current_waypoint = reference_path->get_current_waypoint();  // ここ重要！
}

// SpatialBicycleModel 内の public に追加
std::shared_ptr<Waypoint> SpatialBicycleModel::get_current_waypoint_ptr() const {
    return current_waypoint;
}


// --- BicycleModel ---------------------------------------

// spatial_bicycle_models.cpp
BicycleModel::BicycleModel(std::shared_ptr<ReferencePath> ref_path, double length, double width, double Ts)
    : SpatialBicycleModel(ref_path, length, width, Ts)
{
    spatial_state = std::make_unique<SimpleSpatialState>(0.0, 0.0, 0.0);
    this->n_states = 3;
    Eigen::Vector3d vec;
    vec << (*spatial_state)[0], (*spatial_state)[1], (*spatial_state)[2];
    temporal_state = std::make_unique<TemporalState>(s2t(*current_waypoint, vec));
}

SimpleSpatialState::~SimpleSpatialState() = default;
BicycleModel::~BicycleModel() = default;

TemporalState BicycleModel::s2t(const Waypoint& wp, const Vector3d& state) {
    double x = wp.x - state[0] * std::sin(wp.yaw);
    double y = wp.y + state[0] * std::cos(wp.yaw);
    double yaw = wp.yaw + state[1];
    return TemporalState(x, y, yaw);
}

SimpleSpatialState BicycleModel::t2s(const Waypoint& wp, const TemporalState& ts) {
    double dx = ts.x - wp.x;
    double dy = ts.y - wp.y;

    double e_y = std::cos(wp.yaw) * dy - std::sin(wp.yaw) * dx;
    double e_yaw = std::fmod(ts.yaw - wp.yaw + M_PI, 2 * M_PI) - M_PI;

    return SimpleSpatialState(e_y, e_yaw, 0.0);
}

void BicycleModel::drive(const Vector2d& u) {
    double v = u[0];
    double delta = u[1];

    // 1. Temporal derivative
    double x_dot = v * std::cos(temporal_state->yaw);
    double y_dot = v * std::sin(temporal_state->yaw);
    double yaw_dot = v / length * std::tan(delta);

    Vector3d delta_state(x_dot, y_dot, yaw_dot);
    (*temporal_state) += delta_state * Ts;

    // 2. Compute s_dot
    double kappa = current_waypoint->kappa;
    double e_y = dynamic_cast<SimpleSpatialState*>(spatial_state.get())->e_y;
    auto s_ptr = dynamic_cast<SimpleSpatialState*>(spatial_state.get());
    double e_yaw = s_ptr->e_yaw;

    double s_dot = 1.0 / (1.0 - e_y * kappa) * v * std::cos(e_yaw);
    s += s_dot * Ts;
}

std::vector<std::string> SimpleSpatialState::list_states() const {
    return members;
}

double& SimpleSpatialState::operator[](size_t index) {
    if (index == 0) return e_y;
    if (index == 1) return e_yaw;
    if (index == 2) return t;
    throw std::out_of_range("SimpleSpatialState index out of range");
}

Eigen::Vector3d BicycleModel::get_spatial_derivatives(
    const SpatialState& state, const Eigen::Vector2d& input, double kappa)
{
    const SimpleSpatialState& simple_state = dynamic_cast<const SimpleSpatialState&>(state);
    double e_y = simple_state.e_y;
    double e_yaw = simple_state.e_yaw;

    double v = input[0];
    double delta = input[1];

    Eigen::Vector3d d_state;
    d_state << v * std::sin(e_yaw),
               v * (std::tan(delta) / length - kappa),
               1.0;
    return d_state;
}

void BicycleModel::linearize(
    double v_ref, double kappa_ref, double delta_s,
    Eigen::Vector3d& f_out,
    Eigen::Matrix3d& A_out,
    Eigen::Matrix<double, 3, 2>& B_out)
{
    double e_psi = 0.0;  // 周囲のcontextでゼロ初期化想定
    double delta = 0.0;  // 同上

    f_out << v_ref * std::sin(e_psi),
             v_ref * (std::tan(delta) / length - kappa_ref),
             1.0;

    A_out.setZero();
    A_out(0, 1) = v_ref * std::cos(e_psi);
    A_out(2, 2) = 0.0;

    B_out.setZero();
    B_out(0, 0) = 0.0;
    B_out(1, 0) = std::tan(delta) / length;
    B_out(1, 1) = v_ref / (length * std::pow(std::cos(delta), 2));
}

void SpatialBicycleModel::update_current_waypoint() {
    while (wp_id + 1 < reference_path->waypoints.size() &&
           s > reference_path->waypoints[wp_id + 1]->s) {
        wp_id++;
        current_waypoint = reference_path->waypoints[wp_id];
    }
}


Eigen::MatrixXd BicycleModel::linearize(const SimpleSpatialState& state, double v, double kappa, double dt, Eigen::MatrixXd& A, Eigen::MatrixXd& B) const {
    A = Eigen::MatrixXd::Identity(3, 3);
    B = Eigen::MatrixXd::Zero(3, 1);

    // 状態: [e_y, e_psi, t]
    // 入力: delta（操舵角）

    A(0, 1) = v * dt;
    A(0, 2) = -v * std::sin(state.e_yaw) * dt;
    A(1, 1) = 1.0;
    A(1, 2) = -kappa * v * std::cos(state.e_yaw) * dt;
    A(2, 2) = 1.0;

    B(0, 0) = 0.0;
    B(1, 0) = v * dt / wheelbase_;  // wheelbase_ はクラスメンバで定義されている前提
    B(2, 0) = 0.0;

    return A;  // Aを返すが、参照渡しなのでA,B両方更新される
}

// どちらか片方だけ残す（多分行175と216両方にある）
void SimpleSpatialState::update(const TemporalState& ts, const Waypoint& wp) {
    double dx = ts.x - wp.x;
    double dy = ts.y - wp.y;

    double yaw_ref = wp.yaw;
    double yaw_diff = ts.yaw - yaw_ref;
    while (yaw_diff > M_PI) yaw_diff -= 2.0 * M_PI;
    while (yaw_diff < -M_PI) yaw_diff += 2.0 * M_PI;
    e_yaw = yaw_diff;

    e_y = -std::sin(yaw_ref) * dx + std::cos(yaw_ref) * dy;

    // 🔥追加！現在の位置を t に反映！
    //t = wp.s;
}


