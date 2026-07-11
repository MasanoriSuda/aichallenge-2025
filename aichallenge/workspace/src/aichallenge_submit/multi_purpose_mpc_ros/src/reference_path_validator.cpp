#include "multi_purpose_mpc_ros/path_core.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace path_core = multi_purpose_mpc_ros::path_core;

namespace
{

struct Options
{
  std::string csv_path;
  bool circular{false};
  std::optional<double> resolution_m;
};

Options parse_options(const int argc, char ** argv)
{
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--circular") {
      options.circular = true;
    } else if (argument == "--resolution") {
      if (index + 1 >= argc) {
        throw std::invalid_argument("--resolution requires a value in metres");
      }
      std::size_t consumed = 0;
      const std::string value = argv[++index];
      const double resolution = std::stod(value, &consumed);
      if (consumed != value.size() || !std::isfinite(resolution) || resolution <= 0.0) {
        throw std::invalid_argument("--resolution must be a finite positive number");
      }
      options.resolution_m = resolution;
    } else if (!argument.empty() && argument.front() == '-') {
      throw std::invalid_argument("unknown option: " + argument);
    } else if (options.csv_path.empty()) {
      options.csv_path = argument;
    } else {
      throw std::invalid_argument("multiple CSV paths were provided");
    }
  }
  if (options.csv_path.empty()) {
    throw std::invalid_argument("a trajectory CSV path is required");
  }
  return options;
}

void require_finite_metric(const double value, const std::string & metric)
{
  if (!std::isfinite(value)) {
    throw std::runtime_error("derived metric '" + metric + "' is non-finite");
  }
}

double lateral_acceleration_mps2(
  const path_core::ReferencePathPoint & point, const std::size_t index)
{
  const double abs_kappa = std::abs(point.kappa_radpm);
  if (abs_kappa == 0.0) {
    return 0.0;
  }

  const long double acceleration =
    static_cast<long double>(point.vx_mps) * static_cast<long double>(point.vx_mps) *
    static_cast<long double>(abs_kappa);
  if (!std::isfinite(acceleration) ||
    acceleration > static_cast<long double>(std::numeric_limits<double>::max()))
  {
    throw std::runtime_error(
            "derived metric 'lateral_acceleration_mps2' is non-finite at point " +
            std::to_string(index));
  }
  return static_cast<double>(acceleration);
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const Options options = parse_options(argc, argv);
    auto points = path_core::load_reference_path_csv(options.csv_path);
    const std::size_t raw_point_count = points.size();
    const bool duplicate_endpoint_removed =
      options.circular ? path_core::normalize_circular_endpoint(points, 1e-3) : false;

    const std::size_t segment_count = options.circular ? points.size() : points.size() - 1U;
    double total_distance_m = 0.0;
    double min_spacing_m = std::numeric_limits<double>::infinity();
    double max_spacing_m = 0.0;
    double max_abs_kappa_radpm = 0.0;
    std::optional<double> min_curvature_radius_m;
    double max_wrapped_psi_difference_rad = 0.0;
    double max_lateral_acceleration_mps2 = 0.0;
    double min_velocity_mps = std::numeric_limits<double>::infinity();
    double max_velocity_mps = -std::numeric_limits<double>::infinity();
    double min_acceleration_mps2 = std::numeric_limits<double>::infinity();
    double max_acceleration_mps2 = -std::numeric_limits<double>::infinity();
    std::size_t zero_length_segment_count = 0U;
    std::size_t over_resolution_segment_count = 0U;

    for (std::size_t index = 0; index < points.size(); ++index) {
      const auto & point = points[index];
      const double abs_kappa = std::abs(point.kappa_radpm);
      max_abs_kappa_radpm = std::max(max_abs_kappa_radpm, abs_kappa);
      if (abs_kappa > 1e-12) {
        const double radius_m = 1.0 / abs_kappa;
        require_finite_metric(radius_m, "curvature_radius_m[" + std::to_string(index) + "]");
        min_curvature_radius_m = min_curvature_radius_m.has_value() ?
          std::min(min_curvature_radius_m.value(), radius_m) : radius_m;
      }
      const double lateral_acceleration = lateral_acceleration_mps2(point, index);
      max_lateral_acceleration_mps2 = std::max(
        max_lateral_acceleration_mps2, lateral_acceleration);
      min_velocity_mps = std::min(min_velocity_mps, point.vx_mps);
      max_velocity_mps = std::max(max_velocity_mps, point.vx_mps);
      min_acceleration_mps2 = std::min(min_acceleration_mps2, point.ax_mps2);
      max_acceleration_mps2 = std::max(max_acceleration_mps2, point.ax_mps2);
    }

    for (std::size_t index = 0; index < segment_count; ++index) {
      const std::size_t next_index = (index + 1U) % points.size();
      const double spacing = std::hypot(
        points[next_index].x_m - points[index].x_m,
        points[next_index].y_m - points[index].y_m);
      require_finite_metric(spacing, "spacing_m[" + std::to_string(index) + "]");
      total_distance_m += spacing;
      require_finite_metric(total_distance_m, "total_distance_m");
      min_spacing_m = std::min(min_spacing_m, spacing);
      max_spacing_m = std::max(max_spacing_m, spacing);
      const double psi_difference = points[next_index].psi_rad - points[index].psi_rad;
      require_finite_metric(
        psi_difference, "psi_difference_rad[" + std::to_string(index) + "]");
      const double wrapped_psi_difference = path_core::wrap_to_pi(psi_difference);
      require_finite_metric(
        wrapped_psi_difference,
        "wrapped_psi_difference_rad[" + std::to_string(index) + "]");
      max_wrapped_psi_difference_rad = std::max(
        max_wrapped_psi_difference_rad, std::abs(wrapped_psi_difference));
      if (spacing <= path_core::kMinimumSegmentLengthM) {
        ++zero_length_segment_count;
      }
      if (
        options.resolution_m.has_value() &&
        static_cast<long double>(spacing) >
        static_cast<long double>(options.resolution_m.value()) * 1.05L)
      {
        ++over_resolution_segment_count;
      }
    }

    const double mean_spacing_m = total_distance_m / static_cast<double>(segment_count);
    require_finite_metric(mean_spacing_m, "mean_spacing_m");
    const double closure_position_difference_m = std::hypot(
      points.back().x_m - points.front().x_m,
      points.back().y_m - points.front().y_m);
    require_finite_metric(closure_position_difference_m, "closure_position_difference_m");
    const double closure_psi_delta_rad = points.back().psi_rad - points.front().psi_rad;
    require_finite_metric(closure_psi_delta_rad, "closure_psi_delta_rad");
    const double closure_psi_difference_rad =
      std::abs(path_core::wrap_to_pi(closure_psi_delta_rad));
    require_finite_metric(closure_psi_difference_rad, "closure_psi_difference_rad");
    const double closure_kappa_difference_radpm = std::abs(
      points.back().kappa_radpm - points.front().kappa_radpm);
    require_finite_metric(closure_kappa_difference_radpm, "closure_kappa_difference_radpm");

    std::cout << std::setprecision(10)
              << "csv_path=" << options.csv_path << '\n'
              << "circular=" << (options.circular ? "true" : "false") << '\n'
              << "raw_point_count=" << raw_point_count << '\n'
              << "normalized_point_count=" << points.size() << '\n'
              << "duplicate_endpoint_removed=" <<
      (duplicate_endpoint_removed ? "true" : "false") << '\n'
              << "total_distance_m=" << total_distance_m << '\n'
              << "min_spacing_m=" << min_spacing_m << '\n'
              << "max_spacing_m=" << max_spacing_m << '\n'
              << "mean_spacing_m=" << mean_spacing_m << '\n'
              << "max_abs_kappa_radpm=" << max_abs_kappa_radpm << '\n'
              << "min_curvature_radius_m=";
    if (min_curvature_radius_m.has_value()) {
      std::cout << min_curvature_radius_m.value() << '\n';
    } else {
      std::cout << "unbounded" << '\n';
    }
    std::cout << "max_wrapped_psi_difference_rad=" << max_wrapped_psi_difference_rad << '\n'
              << "closure_position_difference_m=" << closure_position_difference_m << '\n'
              << "closure_psi_difference_rad=" << closure_psi_difference_rad << '\n'
              << "closure_kappa_difference_radpm=" << closure_kappa_difference_radpm << '\n'
              << "min_velocity_mps=" << min_velocity_mps << '\n'
              << "max_velocity_mps=" << max_velocity_mps << '\n'
              << "min_acceleration_mps2=" << min_acceleration_mps2 << '\n'
              << "max_acceleration_mps2=" << max_acceleration_mps2 << '\n'
              << "max_lateral_acceleration_mps2=" << max_lateral_acceleration_mps2 << '\n'
              << "zero_length_segment_count=" << zero_length_segment_count << '\n'
              << "over_resolution_segment_count=" << over_resolution_segment_count << '\n';

    return zero_length_segment_count == 0U && over_resolution_segment_count == 0U ? 0 : 1;
  } catch (const std::exception & error) {
    std::cerr << "reference_path_validator: " << error.what() << '\n';
    return 2;
  }
}
