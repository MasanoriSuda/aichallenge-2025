#pragma once

#include <vector>
#include <string>
#include <cmath>
#include <memory>
#include <Eigen/Dense>
#include "reference_path.hpp"  // ← これがあってもダメ。Waypoint定義が直接必要！
#include "waypoint.hpp"        // ← これを追加！

// 定数定義（πなど）
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ------- TemporalState -------
struct TemporalState {
    double x;
    double y;
    double yaw;

    TemporalState(double x_, double y_, double yaw_)
        : x(x_), y(y_), yaw(yaw_) {}

    TemporalState& operator+=(const Eigen::Vector3d& delta);
};

// ------- SpatialState（抽象クラス） -------
class SpatialState {
public:
    virtual ~SpatialState() {}
    virtual std::vector<std::string> list_states() const = 0;

    virtual double& operator[](size_t index) = 0;
    virtual SpatialState& operator+=(const Eigen::VectorXd& delta) = 0;
};

// ------- SimpleSpatialState -------
class SimpleSpatialState : public SpatialState {
public:
    double e_y;
    double e_yaw;
    double t;
    std::vector<std::string> members;

    SimpleSpatialState(double e_y_=0.0, double e_yawa_=0.0, double t_=0.0);
    virtual ~SimpleSpatialState();  // 追加！
    Eigen::Vector3d toVector() const;

    std::vector<std::string> list_states() const override;
    double& operator[](size_t index) override;
    SpatialState& operator+=(const Eigen::VectorXd& delta) override;
    void update(const TemporalState& ts, const Waypoint& wp);  // ← これ追加！
};

// ------- Forward 宣言 -------
class ReferencePath;
class Waypoint;

// ------- SpatialBicycleModel（抽象） -------
class SpatialBicycleModel {
protected:
    double eps = 1e-12;
    double length;
    double width;
    double safety_margin;
    double s = 0.0;
    double Ts;
    int wp_id = 0;

    std::shared_ptr<ReferencePath> reference_path;
    std::shared_ptr<Waypoint> current_waypoint;

public:
    std::unique_ptr<SpatialState> spatial_state;
    std::unique_ptr<TemporalState> temporal_state;

    SpatialBicycleModel(std::shared_ptr<ReferencePath> ref_path, double len, double wid, double Ts_);

    TemporalState s2t(const Waypoint& wp, const SpatialState& state);
    SimpleSpatialState t2s(const Waypoint& wp, const TemporalState& state);
    void drive(const Eigen::Vector2d& u);
    void get_current_waypoint();
    std::shared_ptr<Waypoint> get_current_waypoint_ptr() const;
    void update_current_waypoint();  // ← これが不足している


    virtual Eigen::Vector3d get_spatial_derivatives(const SpatialState& state, const Eigen::Vector2d& input, double kappa) = 0;
    virtual void linearize(double v_ref, double kappa_ref, double delta_s,
                           Eigen::Vector3d& f_out, Eigen::Matrix3d& A_out, Eigen::Matrix<double, 3, 2>& B_out) = 0;

protected:
    double _compute_safety_margin() const;
};

// ------- BicycleModel -------
class BicycleModel : public SpatialBicycleModel {
private:
    double wheelbase_ = 1.087;  // 車両ホイールベース    
public:
    // spatial_bicycle_models.hpp の class BicycleModel の中に追加
    int n_states;
    BicycleModel(std::shared_ptr<ReferencePath> ref_path, double len, double wid, double Ts);
    virtual ~BicycleModel();  // 追加！

    std::pair<double, double> get_temporal_derivatives(const SpatialState& state, const Eigen::Vector2d& input, double kappa);
    Eigen::Vector3d get_spatial_derivatives(const SpatialState& state, const Eigen::Vector2d& input, double kappa) override;
    void linearize(double v_ref, double kappa_ref, double delta_s,
                   Eigen::Vector3d& f_out, Eigen::Matrix3d& A_out, Eigen::Matrix<double, 3, 2>& B_out) override;
    TemporalState s2t(const Waypoint& wp, const Eigen::Vector3d& state);
    SimpleSpatialState t2s(const Waypoint& wp, const TemporalState& state);
    void drive(const Eigen::Vector2d& u);
    Eigen::MatrixXd linearize(const SimpleSpatialState& state, double v, double kappa, double dt, Eigen::MatrixXd& A, Eigen::MatrixXd& B) const;

};

