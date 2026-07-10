#include "multi_purpose_mpc_ros/path_core.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace multi_purpose_mpc_ros::path_core
{
namespace
{

constexpr std::array<const char *, 7> kRequiredColumns{
  "s_m", "x_m", "y_m", "psi_rad", "kappa_radpm", "vx_mps", "ax_mps2"};

std::string trim(std::string value)
{
  const auto is_space = [](const unsigned char character) {
      return std::isspace(character) != 0;
    };
  const auto first = std::find_if_not(value.begin(), value.end(), is_space);
  const auto last = std::find_if_not(value.rbegin(), value.rend(), is_space).base();
  if (first >= last) {
    return {};
  }
  return std::string(first, last);
}

std::vector<std::string> split_csv_row(const std::string & line)
{
  std::vector<std::string> fields;
  std::size_t start = 0;
  while (true) {
    const std::size_t separator = line.find(',', start);
    if (separator == std::string::npos) {
      fields.push_back(line.substr(start));
      break;
    }
    fields.push_back(line.substr(start, separator - start));
    start = separator + 1;
  }
  return fields;
}

[[noreturn]] void throw_csv_error(
  const std::string & path, const std::size_t row, const std::string & column,
  const std::string & value, const std::string & reason)
{
  std::ostringstream message;
  message << path << ": row " << row << ", column '" << column << "', value '" << value <<
    "': " << reason;
  throw std::runtime_error(message.str());
}

double parse_finite_double(
  const std::string & raw_value, const std::string & path, const std::size_t row,
  const std::string & column)
{
  const std::string value = trim(raw_value);
  if (value.empty()) {
    throw_csv_error(path, row, column, raw_value, "numeric value is empty");
  }

  std::size_t consumed = 0;
  double parsed = 0.0;
  try {
    parsed = std::stod(value, &consumed);
  } catch (const std::exception & error) {
    throw_csv_error(
      path, row, column, raw_value, std::string("invalid numeric value: ") + error.what());
  }
  if (consumed != value.size()) {
    throw_csv_error(path, row, column, raw_value, "numeric value was only partially converted");
  }
  if (!std::isfinite(parsed)) {
    throw_csv_error(path, row, column, raw_value, "numeric value must be finite");
  }
  return parsed;
}

}  // namespace

double wrap_to_pi(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

std::vector<ReferencePathPoint> load_reference_path_csv(const std::string & path)
{
  std::ifstream input(path);
  if (!input.is_open()) {
    throw_csv_error(path, 1, "<file>", "<unavailable>", "could not open file");
  }

  std::string header_line;
  if (!std::getline(input, header_line)) {
    throw_csv_error(path, 1, "<header>", "<missing>", "CSV is empty");
  }

  auto headers = split_csv_row(header_line);
  if (headers.empty()) {
    throw_csv_error(path, 1, "<header>", "<missing>", "CSV header is empty");
  }
  headers.front() = trim(std::move(headers.front()));
  if (headers.front().rfind("\xEF\xBB\xBF", 0) == 0) {
    headers.front().erase(0, 3);
    headers.front() = trim(std::move(headers.front()));
  }

  std::unordered_map<std::string, std::size_t> column_indices;
  for (std::size_t index = 0; index < headers.size(); ++index) {
    headers[index] = trim(std::move(headers[index]));
    if (headers[index].empty()) {
      throw_csv_error(path, 1, "<column-" + std::to_string(index + 1) + ">", "", "header is empty");
    }
    if (!column_indices.emplace(headers[index], index).second) {
      throw_csv_error(path, 1, headers[index], headers[index], "duplicate header");
    }
  }

  for (const char * required_column : kRequiredColumns) {
    if (column_indices.count(required_column) == 0U) {
      throw_csv_error(path, 1, required_column, "<missing>", "required header is missing");
    }
  }
  if (headers.size() != kRequiredColumns.size()) {
    const std::size_t diagnostic_index = std::min(kRequiredColumns.size(), headers.size() - 1);
    throw_csv_error(
      path, 1, headers[diagnostic_index], headers[diagnostic_index],
      "strict reference-path CSV must contain exactly 7 columns");
  }

  const auto column = [&column_indices](const char * name) {
      return column_indices.at(name);
    };

  std::vector<ReferencePathPoint> points;
  std::string line;
  std::size_t row = 1;
  while (std::getline(input, line)) {
    ++row;
    if (trim(line).empty()) {
      throw_csv_error(path, row, "<record>", "<blank>", "blank data rows are not allowed");
    }
    const auto fields = split_csv_row(line);
    if (fields.size() != headers.size()) {
      if (fields.size() < headers.size()) {
        const std::string & missing_column = headers[fields.size()];
        throw_csv_error(
          path, row, missing_column, "<missing>", "row has " + std::to_string(fields.size()) +
          " fields; expected " + std::to_string(headers.size()));
      }
      throw_csv_error(
        path, row, "<column-" + std::to_string(headers.size() + 1) + ">",
        fields[headers.size()], "row has " + std::to_string(fields.size()) +
        " fields; expected " + std::to_string(headers.size()));
    }

    ReferencePathPoint point;
    point.s_m = parse_finite_double(fields[column("s_m")], path, row, "s_m");
    point.x_m = parse_finite_double(fields[column("x_m")], path, row, "x_m");
    point.y_m = parse_finite_double(fields[column("y_m")], path, row, "y_m");
    point.psi_rad = parse_finite_double(fields[column("psi_rad")], path, row, "psi_rad");
    point.kappa_radpm =
      parse_finite_double(fields[column("kappa_radpm")], path, row, "kappa_radpm");
    point.vx_mps = parse_finite_double(fields[column("vx_mps")], path, row, "vx_mps");
    point.ax_mps2 = parse_finite_double(fields[column("ax_mps2")], path, row, "ax_mps2");

    if (!points.empty()) {
      if (point.s_m <= points.back().s_m) {
        throw_csv_error(
          path, row, "s_m", fields[column("s_m")],
          "s_m must be strictly increasing; previous value was " +
          std::to_string(points.back().s_m));
      }

      const double segment_length_m = std::hypot(
        point.x_m - points.back().x_m, point.y_m - points.back().y_m);
      if (!std::isfinite(segment_length_m)) {
        throw_csv_error(
          path, row, "<segment-length>", "<non-finite>",
          "derived segment length must be finite");
      }
      if (segment_length_m <= kMinimumSegmentLengthM) {
        throw_csv_error(
          path, row, "<segment-length>", std::to_string(segment_length_m),
          "consecutive points must be more than " +
          std::to_string(kMinimumSegmentLengthM) + " metres apart");
      }
    }
    points.push_back(point);
  }

  if (input.bad()) {
    throw_csv_error(path, row + 1U, "<file>", "<read-error>", "I/O error while reading CSV");
  }

  if (points.size() < 2U) {
    throw_csv_error(
      path, row + 1, "<record>", "<missing>",
      "reference path requires at least 2 data rows");
  }
  return points;
}

bool normalize_circular_endpoint(
  std::vector<ReferencePathPoint> & points, const double tolerance_m)
{
  if (!std::isfinite(tolerance_m) || tolerance_m < 0.0) {
    throw std::invalid_argument("closure duplicate tolerance must be finite and non-negative");
  }
  if (points.empty()) {
    throw std::invalid_argument("circular path requires at least 3 unique points");
  }

  for (std::size_t index = 0; index < points.size(); ++index) {
    if (!std::isfinite(points[index].x_m) || !std::isfinite(points[index].y_m)) {
      throw std::invalid_argument(
              "circular path point " + std::to_string(index) + " has non-finite coordinates");
    }
  }

  const double closure_distance = std::hypot(
    points.back().x_m - points.front().x_m, points.back().y_m - points.front().y_m);
  if (!std::isfinite(closure_distance)) {
    throw std::invalid_argument("circular path closure distance must be finite");
  }
  const bool has_duplicate_endpoint = closure_distance <= tolerance_m;
  const std::size_t normalized_size = points.size() - (has_duplicate_endpoint ? 1U : 0U);
  if (normalized_size < 3U) {
    throw std::invalid_argument("circular path requires at least 3 unique points");
  }

  for (std::size_t index = 1; index < normalized_size; ++index) {
    const double distance = std::hypot(
      points[index].x_m - points[index - 1].x_m,
      points[index].y_m - points[index - 1].y_m);
    if (distance <= kMinimumSegmentLengthM) {
      throw std::invalid_argument(
              "circular path contains a consecutive degenerate edge at point " +
              std::to_string(index));
    }
  }

  if (has_duplicate_endpoint) {
    points.pop_back();
  }
  return has_duplicate_endpoint;
}

std::size_t subdivision_count(const double distance, const double resolution)
{
  if (!std::isfinite(distance) || distance < 0.0) {
    throw std::invalid_argument("subdivision distance must be finite and non-negative");
  }
  if (!std::isfinite(resolution) || resolution <= 0.0) {
    throw std::invalid_argument("subdivision resolution must be finite and positive");
  }

  const long double count = std::max(
    1.0L, std::ceil(
      static_cast<long double>(distance) / static_cast<long double>(resolution)));
  const long double size_t_exclusive_upper_bound =
    std::ldexp(1.0L, std::numeric_limits<std::size_t>::digits);
  if (!std::isfinite(count) || count >= size_t_exclusive_upper_bound) {
    throw std::overflow_error("subdivision count does not fit in std::size_t");
  }
  return static_cast<std::size_t>(count);
}

}  // namespace multi_purpose_mpc_ros::path_core
