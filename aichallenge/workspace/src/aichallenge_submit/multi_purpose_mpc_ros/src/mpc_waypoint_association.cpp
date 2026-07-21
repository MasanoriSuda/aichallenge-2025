#include <multi_purpose_mpc_ros/mpc_waypoint_association.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace multi_purpose_mpc_ros::mpc_waypoint_association
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kEpsilon = 1.0e-9;

double wrap_to_pi(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

void validate_nonnegative(const double value, const char * name)
{
  if (!std::isfinite(value) || value < 0.0) {
    throw std::invalid_argument(std::string(name) + " must be finite and non-negative");
  }
}

void validate(const std::vector<Waypoint> & waypoints, const Request & request, const Config & config)
{
  if (waypoints.empty()) {
    throw std::invalid_argument("waypoint association requires a non-empty path");
  }
  for (const auto & waypoint : waypoints) {
    if (!std::isfinite(waypoint.x_m) || !std::isfinite(waypoint.y_m) ||
      !std::isfinite(waypoint.heading_rad))
    {
      throw std::invalid_argument("waypoint association path contains a non-finite value");
    }
  }
  if (!std::isfinite(request.x_m) || !std::isfinite(request.y_m) ||
    !std::isfinite(request.yaw_rad) || !std::isfinite(request.speed_mps) ||
    !std::isfinite(request.dt_sec) || request.speed_mps < 0.0 || request.dt_sec < 0.0)
  {
    throw std::invalid_argument("waypoint association request contains an invalid value");
  }
  if (request.previous_valid &&
    (request.previous_index < 0 || request.previous_index >= static_cast<int>(waypoints.size())))
  {
    throw std::out_of_range("previous waypoint index is outside the path");
  }
  validate_nonnegative(config.local_lookbehind_m, "local lookbehind");
  validate_nonnegative(config.local_lookahead_m, "local lookahead");
  validate_nonnegative(config.lost_distance_m, "lost distance");
  validate_nonnegative(config.heading_weight_m_per_rad, "heading weight");
  validate_nonnegative(config.backward_progress_weight, "backward progress weight");
  validate_nonnegative(config.forward_jump_weight, "forward jump weight");
  validate_nonnegative(config.minimum_forward_reach_m, "minimum forward reach");
  validate_nonnegative(config.forward_reach_time_scale, "forward reach time scale");
  if (config.local_lookahead_m <= 0.0 || config.lost_distance_m <= 0.0) {
    throw std::invalid_argument("local lookahead and lost distance must be positive");
  }
}

int adjacent_index(const int index, const int direction, const int count, const bool circular)
{
  const int candidate = index + direction;
  if (circular) {
    return (candidate % count + count) % count;
  }
  return candidate >= 0 && candidate < count ? candidate : -1;
}

double segment_length(const Waypoint & lhs, const Waypoint & rhs)
{
  return std::hypot(lhs.x_m - rhs.x_m, lhs.y_m - rhs.y_m);
}

struct Candidate
{
  int index{};
  double signed_progress_m{};
};

std::vector<Candidate> local_candidates(
  const std::vector<Waypoint> & waypoints, const Request & request, const Config & config)
{
  std::vector<Candidate> candidates;
  std::vector<bool> visited(waypoints.size(), false);
  const int count = static_cast<int>(waypoints.size());
  candidates.push_back(Candidate{request.previous_index, 0.0});
  visited[request.previous_index] = true;

  const auto walk = [&](const int direction, const double maximum_distance_m) {
      int current = request.previous_index;
      double distance_m = 0.0;
      for (int step = 1; step < count; ++step) {
        const int next = adjacent_index(current, direction, count, request.circular);
        if (next < 0 || next == request.previous_index) {
          break;
        }
        distance_m += segment_length(waypoints[current], waypoints[next]);
        if (distance_m > maximum_distance_m + kEpsilon) {
          break;
        }
        if (!visited[next]) {
          candidates.push_back(Candidate{next, direction > 0 ? distance_m : -distance_m});
          visited[next] = true;
        }
        current = next;
      }
    };
  walk(1, config.local_lookahead_m);
  walk(-1, config.local_lookbehind_m);
  return candidates;
}

struct ScoredCandidate
{
  Result result;
  double score{std::numeric_limits<double>::infinity()};
};

ScoredCandidate score_candidate(
  const std::vector<Waypoint> & waypoints, const Request & request, const Config & config,
  const Candidate & candidate, const bool apply_progress_penalty)
{
  const auto & waypoint = waypoints.at(candidate.index);
  const double distance_m = std::hypot(waypoint.x_m - request.x_m, waypoint.y_m - request.y_m);
  const double heading_error_rad = std::abs(wrap_to_pi(waypoint.heading_rad - request.yaw_rad));
  double score = distance_m +
    (config.enabled ? config.heading_weight_m_per_rad * heading_error_rad : 0.0);
  if (apply_progress_penalty) {
    score += config.backward_progress_weight * std::max(0.0, -candidate.signed_progress_m);
    const double reachable_forward_m = std::max(
      config.minimum_forward_reach_m,
      request.speed_mps * request.dt_sec * config.forward_reach_time_scale);
    score += config.forward_jump_weight *
      std::max(0.0, candidate.signed_progress_m - reachable_forward_m);
  }
  return ScoredCandidate{
    Result{candidate.index, !apply_progress_penalty, distance_m, heading_error_rad,
      candidate.signed_progress_m},
    score};
}

ScoredCandidate best_candidate(
  const std::vector<Waypoint> & waypoints, const Request & request, const Config & config,
  const std::vector<Candidate> & candidates, const bool apply_progress_penalty)
{
  ScoredCandidate best;
  for (const auto & candidate : candidates) {
    const auto scored = score_candidate(
      waypoints, request, config, candidate, apply_progress_penalty);
    if (scored.score < best.score) {
      best = scored;
    }
  }
  return best;
}

}  // namespace

Result associate(
  const std::vector<Waypoint> & waypoints,
  const Request & request,
  const Config & config)
{
  validate(waypoints, request, config);
  if (config.enabled && request.previous_valid) {
    const auto local = best_candidate(
      waypoints, request, config, local_candidates(waypoints, request, config), true);
    if (local.result.distance_m <= config.lost_distance_m) {
      return local.result;
    }
  }

  std::vector<Candidate> global;
  global.reserve(waypoints.size());
  for (int index = 0; index < static_cast<int>(waypoints.size()); ++index) {
    global.push_back(Candidate{index, 0.0});
  }
  auto result = best_candidate(waypoints, request, config, global, false).result;
  result.used_global_search = true;
  return result;
}

}  // namespace multi_purpose_mpc_ros::mpc_waypoint_association
