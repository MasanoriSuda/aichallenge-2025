// spatial_bicycle_models.hpp
#pragma once

#include <Eigen/Dense>
#include <memory>
#include <vector>
#include <string>
#include "OdometryInput.hpp"

// Forward declaration
class Waypoint;
class ReferencePath;

class TemporalState {
public:
    double x, y, psi;
    double v;// 追加：速度、将来用拡張
    std::vector<std::string> members;

    TemporalState(double x_ = 0.0, double y_ = 0.0, double psi_ = 0.0);
    TemporalState& operator+=(const Eigen::Vector3d& delta);
    Eigen::VectorXd to_vector() const {
        Eigen::VectorXd vec(3);
        vec << x, y, psi;
        return vec;
    }
};

class SpatialState {
public:
    virtual ~SpatialState() = default;
    virtual std::vector<std::string> list_states() const = 0;
    virtual double get(int index) const = 0;
    virtual void set(int index, double value) = 0;
    virtual int size() const = 0;
    virtual SpatialState& operator+=(const Eigen::VectorXd& delta) = 0;
};

class SimpleSpatialState : public SpatialState {
public:
    double e_y, e_psi, t;
    std::vector<std::string> members;

    SimpleSpatialState(double e_y_ = 0.0, double e_psi_ = 0.0, double t_ = 0.0);

    std::vector<std::string> list_states() const override;
    double get(int index) const override;
    void set(int index, double value) override;
    int size() const override;
    SpatialState& operator+=(const Eigen::VectorXd& delta) override;

    Eigen::Vector3d to_vector() const;
};

class SpatialBicycleModel {
public:
    SpatialBicycleModel(std::shared_ptr<ReferencePath> reference_path,
                        double length, double width, double Ts);
    virtual ~SpatialBicycleModel() = default;

    TemporalState s2t(const Waypoint& reference_waypoint,
                      const Eigen::VectorXd& reference_state) const;

    SimpleSpatialState t2s(const Waypoint& reference_waypoint,
                           const Eigen::VectorXd& reference_state) const;

    void drive(const Eigen::Vector2d& input);
    void get_current_waypoint();

    virtual Eigen::Vector3d get_spatial_derivatives(const Eigen::Vector3d& state,
                                                    const Eigen::Vector2d& input,
                                                    double kappa) const = 0;

    virtual void linearize(double v_ref, double kappa_ref, double delta_s,
                           Eigen::Vector3d& f, Eigen::Matrix3d& A,
                           Eigen::Matrix<double, 3, 2>& B) const = 0;
    // SpatialBicycleModel.hpp 内で public: の下に追加
    double get_length() const { return length; }
    double get_s() const { return s; }
    double get_Ts() const { return Ts; }
    int get_wp_id() const { return wp_id; }
    std::shared_ptr<TemporalState> get_temporal_state() const { return temporal_state; }
    std::shared_ptr<SimpleSpatialState> get_spatial_state() const { return spatial_state; }


public:
    double _compute_safety_margin() const;

    std::shared_ptr<ReferencePath> reference_path;
    double length, width, safety_margin;
    double s, Ts;
    int wp_id;
    std::shared_ptr<Waypoint> current_waypoint;

    std::shared_ptr<SimpleSpatialState> spatial_state;
    std::shared_ptr<TemporalState> temporal_state;
    const double eps = 1e-12;
    int n_states;
};

class BicycleModel : public SpatialBicycleModel {
public:
    BicycleModel(std::shared_ptr<ReferencePath> reference_path,
                 double length, double width, double Ts);

    Eigen::Vector3d get_spatial_derivatives(const Eigen::Vector3d& state,
                                            const Eigen::Vector2d& input,
                                            double kappa) const override;

    void linearize(double v_ref, double kappa_ref, double delta_s,
                   Eigen::Vector3d& f, Eigen::Matrix3d& A,
                   Eigen::Matrix<double, 3, 2>& B) const override;
    void set_pose_from_odom(const OdometryInput& odom);  // Declaration

private:
    std::tuple<double, double> get_temporal_derivatives(const Eigen::Vector3d& state,
                                                        const Eigen::Vector2d& input,
                                                        double kappa) const;
};
