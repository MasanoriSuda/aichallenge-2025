#pragma once
#include <optional>
#include "multi_purpose_mpc_ros/detail/mpcc_vehicle_enclosure.hpp"
#include "multi_purpose_mpc_ros/recovery_footprint.hpp"

namespace multi_purpose_mpc_ros::mpcc_vehicle_model::numerical {
namespace recovery = multi_purpose_mpc_ros::recovery_footprint;
struct FootprintBox {
  recovery::Pose2D pose;
  recovery::FootprintExtents extents;
};
inline FootprintBox footprint(const Box &states,
                              const recovery::FootprintExtents &original) {
  if (!original.valid())
    throw std::runtime_error("invalid original footprint");
  FootprintBox out;
  out.pose = {states[0].lo + (states[0].hi - states[0].lo) / 2,
              states[1].lo + (states[1].hi - states[1].lo) / 2,
              states[2].lo + (states[2].hi - states[2].lo) / 2};
  const I c = cosine(I(out.pose.yaw_rad)), sn = sine(I(out.pose.yaw_rad));
  const I dx = states[0] - I(out.pose.x_m), dy = states[1] - I(out.pose.y_m);
  const I forward = c * dx + sn * dy, left = -sn * dx + c * dy;
  const I angle = states[2] - I(out.pose.yaw_rad), ca = cosine(angle),
          sa = sine(angle);
  // Include the original margin before rotating each corner. The returned
  // zero margin represents that already-expanded rectangle, not a relaxation.
  const I front = I(original.front_extent_m) + I(original.margin_m);
  const I rear = -I(original.rear_extent_m) - I(original.margin_m);
  const I lhs = I(original.left_extent_m) + I(original.margin_m);
  const I rhs = -I(original.right_extent_m) - I(original.margin_m);
  for (const I x : {front, rear})
    for (const I y : {lhs, rhs}) {
      const I f = forward + ca * x - sa * y, l = left + sa * x + ca * y;
      out.extents.front_extent_m = std::max(out.extents.front_extent_m, f.hi);
      out.extents.rear_extent_m = std::max(out.extents.rear_extent_m, -f.lo);
      out.extents.left_extent_m = std::max(out.extents.left_extent_m, l.hi);
      out.extents.right_extent_m = std::max(out.extents.right_extent_m, -l.lo);
    }
  if (!out.extents.valid())
    throw std::runtime_error("invalid enclosed footprint");
  return out;
}
// Rigid vertices include the original physical and wall-clearance margin
// before propagation; no rounded-down double sum may shrink that footprint.
inline Box footprint_vertex_offsets(const recovery::FootprintExtents &original) {
  if (!original.valid()) throw std::runtime_error("invalid corner footprint");
  Box offsets;
  size_t i = 0;
  for (const I x : {I(original.front_extent_m) + I(original.margin_m),
                   -I(original.rear_extent_m) - I(original.margin_m)})
    for (const I y : {I(original.left_extent_m) + I(original.margin_m),
                     -I(original.right_extent_m) - I(original.margin_m)}) {
      offsets[i++] = x;
      offsets[i++] = y;
    }
  return offsets;
}

// A rigid rectangle is the convex hull of its four vertices. These additional
// displacement ranges enclose all of them, independently of the body-pose box.
// A contact square excluded by either enclosure cannot intersect the vehicle.
inline std::optional<double> separating_cell_clearance(
    const Box &vertices, const recovery::OccupancyGrid &grid, size_t cell_index,
    const recovery::Point2D &origin) {
  if (!grid.valid() || cell_index >= grid.cells.size() ||
      !std::isfinite(origin.x_m) || !std::isfinite(origin.y_m)) return std::nullopt;
  const auto center = grid.grid_to_world(cell_index / grid.width, cell_index % grid.width);
  if (!center) return std::nullopt;
  double xmin = INFINITY, xmax = -INFINITY, ymin = INFINITY, ymax = -INFINITY;
  for (size_t i = 0; i < vertices.size(); ++i)
    if (!std::isfinite(vertices[i].lo) || !std::isfinite(vertices[i].hi) ||
        vertices[i].lo > vertices[i].hi) return std::nullopt;
  for (size_t i = 0; i < vertices.size(); i += 2) {
    xmin = std::min(xmin, vertices[i].lo);
    xmax = std::max(xmax, vertices[i].hi);
    ymin = std::min(ymin, vertices[i + 1].lo);
    ymax = std::max(ymax, vertices[i + 1].hi);
  }
  // Include the whole occupied/unknown square and sample_footprint's existing
  // 1e-9 contact epsilon. A tangent or unproved direction remains a rejection.
  const I half = I(grid.resolution_m) * I(.5) + I(1e-9);
  const I x = I(center->x_m) - I(origin.x_m);
  const I y = I(center->y_m) - I(origin.y_m);
  const double gap = std::max({(I(xmin) - x - half).lo, (x - half - I(xmax)).lo,
                              (I(ymin) - y - half).lo, (y - half - I(ymax)).lo});
  return std::isfinite(gap) && gap > 0 ? std::optional<double>{gap} : std::nullopt;
}

// Both pose and rigid-vertex boxes contain the same complete response set.
// Project them along the grid axes, original body axes and midpoint vehicle axes. Directions are
// proposals only: separation requires outward-rounded support bounds for every
// footprint vertex and the whole occupied/unknown square, including epsilon.
inline std::optional<double> separating_oriented_cell_clearance(
    const Box &states, const recovery::FootprintExtents &original,
    const Box &vertices, const recovery::OccupancyGrid &grid, size_t cell_index,
    const recovery::Pose2D &origin) {
  if (!original.valid() || !grid.valid() || cell_index >= grid.cells.size() ||
      !std::isfinite(origin.x_m) || !std::isfinite(origin.y_m) ||
      !std::isfinite(origin.yaw_rad)) return std::nullopt;
  for (size_t i = 0; i < 3; ++i)
    if (!std::isfinite(states[i].lo) || !std::isfinite(states[i].hi) ||
        states[i].lo > states[i].hi) return std::nullopt;
  for (const auto &v : vertices)
    if (!std::isfinite(v.lo) || !std::isfinite(v.hi) || v.lo > v.hi)
      return std::nullopt;
  if (const auto gap = separating_cell_clearance(vertices, grid, cell_index,
                                                {origin.x_m, origin.y_m}))
    return gap;
  const auto center = grid.grid_to_world(cell_index / grid.width, cell_index % grid.width);
  if (!center) return std::nullopt;
  const I x = I(center->x_m) - I(origin.x_m), y = I(center->y_m) - I(origin.y_m);
  const I half = I(grid.resolution_m) * I(.5) + I(1e-9);
  const I co = cosine(I(origin.yaw_rad)), so = sine(I(origin.yaw_rad));
  const I c = cosine(states[2]), s = sine(states[2]);
  const auto offsets = footprint_vertex_offsets(original);
  const double middle = origin.yaw_rad + states[2].lo + (states[2].hi - states[2].lo) / 2;
  if (!std::isfinite(middle)) return std::nullopt;
  const double ca = std::cos(middle), sa = std::sin(middle);
  const double c0 = std::cos(origin.yaw_rad), s0 = std::sin(origin.yaw_rad);
  const std::array<std::pair<double, double>, 12> directions{{
    {c0, s0}, {-c0, -s0}, {-s0, c0}, {s0, -c0},
    {ca, sa}, {-ca, -sa}, {-sa, ca}, {sa, -ca},
    {1, 0}, {-1, 0}, {0, 1}, {0, -1}}};
  for (const auto &[nx, ny] : directions) {
    // Combine the frame coefficients before applying the shared XY ranges.
    const I ax = I(nx) * co + I(ny) * so, ay = -I(nx) * so + I(ny) * co;
    const I translation = ax * states[0] + ay * states[1];
    const I forward = ax * c + ay * s, left = -ax * s + ay * c;
    double body_support = -INFINITY, corner_support = -INFINITY;
    for (size_t i = 0; i < 8; i += 2) {
      body_support = std::max(body_support,
        (translation + forward * offsets[i] + left * offsets[i + 1]).hi);
      corner_support = std::max(corner_support,
        (I(nx) * vertices[i] + I(ny) * vertices[i + 1]).hi);
    }
    const I gap = I(nx) * x + I(ny) * y -
      half * (I(std::abs(nx)) + I(std::abs(ny))) - I(std::min(body_support, corner_support));
    if (!std::isfinite(gap.lo) || gap.lo <= 0) continue;
    const double norm = up(std::sqrt((I(nx) * I(nx) + I(ny) * I(ny)).hi));
    if (!std::isfinite(norm) || norm <= 0) continue;
    const double distance = down(gap.lo / norm);
    if (std::isfinite(distance) && distance > 0) return distance;
  }
  return std::nullopt;
}

// A second enclosure can separate a circle from the entire pose population
// even when the enclosing rectangle's unused corners overlap it. The normal
// from that rectangle's nearest point is only a candidate direction. Authority
// requires the outward-rounded projection of every original footprint corner.
// nullopt means this direction did not prove separation; it grants no clearance.
inline std::optional<double> separating_circle_clearance(
    const Box &states, const recovery::FootprintExtents &original,
    const recovery::Pose2D &origin, const FootprintBox &enclosure,
    double peer_x_m, double peer_y_m, double radius_m) {
  if (!original.valid() || !std::isfinite(origin.x_m) ||
      !std::isfinite(origin.y_m) || !std::isfinite(origin.yaw_rad) ||
      !std::isfinite(peer_x_m) || !std::isfinite(peer_y_m) ||
      !std::isfinite(radius_m) || radius_m < 0)
    throw std::runtime_error("invalid circle separation input");
  for (size_t i = 0; i < 3; ++i)
    if (!std::isfinite(states[i].lo) || !std::isfinite(states[i].hi) ||
        states[i].lo > states[i].hi)
      throw std::runtime_error("invalid circle separation pose range");
  const double c0 = std::cos(origin.yaw_rad), s0 = std::sin(origin.yaw_rad);
  const auto &ep = enclosure.pose;
  const auto &ex = enclosure.extents;
  if (!ex.valid() || !std::isfinite(ep.x_m) || !std::isfinite(ep.y_m) ||
      !std::isfinite(ep.yaw_rad))
    throw std::runtime_error("invalid candidate separation direction");
  const double px = origin.x_m + c0 * ep.x_m - s0 * ep.y_m;
  const double py = origin.y_m + s0 * ep.x_m + c0 * ep.y_m;
  const double yaw_mid = origin.yaw_rad + ep.yaw_rad;
  const double c = std::cos(yaw_mid), s = std::sin(yaw_mid);
  const double fx = c * (peer_x_m - px) + s * (peer_y_m - py);
  const double fy = -s * (peer_x_m - px) + c * (peer_y_m - py);
  const double dx = fx - std::clamp(fx, -ex.rear_extent_m, ex.front_extent_m);
  const double dy = fy - std::clamp(fy, -ex.right_extent_m, ex.left_extent_m);
  const double nx = c * dx - s * dy, ny = s * dx + c * dy;
  const double norm = up(std::sqrt((I(nx) * I(nx) + I(ny) * I(ny)).hi));
  if (!std::isfinite(norm) || norm <= 0 || (nx == 0 && ny == 0))
    return std::nullopt;
  // The candidate normal need not be exact or unit length. Only its bounded
  // norm and the following interval projections participate in the proof.
  const I co = cosine(I(origin.yaw_rad)), so = sine(I(origin.yaw_rad));
  const I peer_dx = I(peer_x_m) - I(origin.x_m) - co * states[0] + so * states[1];
  const I peer_dy = I(peer_y_m) - I(origin.y_m) - so * states[0] - co * states[1];
  const I yaw = I(origin.yaw_rad) + states[2];
  const I ca = cosine(yaw), sa = sine(yaw);
  double support = -INFINITY;
  for (const I x : {I(original.front_extent_m) + I(original.margin_m),
                   -I(original.rear_extent_m) - I(original.margin_m)})
    for (const I y : {I(original.left_extent_m) + I(original.margin_m),
                     -I(original.right_extent_m) - I(original.margin_m)})
      support = std::max(support, ((I(nx) * x + I(ny) * y) * ca +
                                  (I(ny) * x - I(nx) * y) * sa).hi);
  const I gap = I(nx) * peer_dx + I(ny) * peer_dy - I(support) - I(radius_m) * I(norm);
  if (!std::isfinite(gap.lo) || gap.lo < 0)
    return std::nullopt;
  return gap.lo == 0 ? 0 : down(gap.lo / norm);
}

inline Box segment_box(Box first, const Box &last) {
  for (size_t i = 0; i < first.size(); ++i)
    first[i] = hull(first[i], last[i]);
  return first;
}
} // namespace multi_purpose_mpc_ros::mpcc_vehicle_model::numerical
