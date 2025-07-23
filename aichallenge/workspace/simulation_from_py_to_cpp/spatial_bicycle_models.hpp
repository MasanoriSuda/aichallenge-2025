// BicycleModel.hpp
#pragma once

#include "reference_path.hpp"
#include <Eigen/Dense>
#include <memory>

struct TemporalState {
    double x;
    double y;
    double psi;

    TemporalState() : x(0), y(0), psi(0) {}  // 追加
    TemporalState(double x_, double y_, double psi_) : x(x_), y(y_), psi(psi_) {}
};

struct SimpleSpatialState {
    double e_y;
    double e_psi;
    double t;

    SimpleSpatialState(double e_y_ = 0.0, double e_psi_ = 0.0, double t_ = 0.0)
        : e_y(e_y_), e_psi(e_psi_), t(t_) {}

    Eigen::Vector3d to_vector() const {
        return Eigen::Vector3d(e_y, e_psi, t);
    }
};

class SpatialBicycleModel {
public:
    SpatialBicycleModel(std::shared_ptr<ReferencePath> ref_path, double length, double width, double Ts);
    virtual ~SpatialBicycleModel() = default;

    TemporalState s2t(const Waypoint& ref_wp, const SimpleSpatialState& ref_state);
    SimpleSpatialState t2s(const Waypoint& ref_wp, const TemporalState& ref_state);
    virtual void drive(const Eigen::Vector2d& u) = 0;

    virtual Eigen::Vector3d get_spatial_derivatives(const SimpleSpatialState& state,
                                                    const Eigen::Vector2d& input,
                                                    double kappa) = 0;
    virtual std::tuple<Eigen::Vector3d, Eigen::Matrix3d, Eigen::Matrix<double, 3, 2>>
    linearize(double v_ref, double kappa_ref, double delta_s) = 0;
    
    double eps;
    double length, width, Ts, s;
    double safety_margin;
    std::shared_ptr<ReferencePath> reference_path;
    size_t wp_id;
    std::shared_ptr<Waypoint> current_waypoint;

    TemporalState temporal_state;
    SimpleSpatialState spatial_state;
    int n_states;

    double compute_safety_margin() const;
    void update_current_waypoint();
};

class BicycleModel : public SpatialBicycleModel {
public:
    BicycleModel(std::shared_ptr<ReferencePath> ref_path, double length, double width, double Ts);

    Eigen::Vector3d get_spatial_derivatives(const SimpleSpatialState& state,
                                            const Eigen::Vector2d& input,
                                            double kappa) override;

    std::tuple<Eigen::Vector3d, Eigen::Matrix3d, Eigen::Matrix<double, 3, 2>>
    linearize(double v_ref, double kappa_ref, double delta_s) override;
    std::tuple<double, double> get_temporal_derivatives(
    const SimpleSpatialState& state,
    const Eigen::Vector2d& input,
    double kappa);
    void drive(const Eigen::Vector2d& u) override;
};
