#ifndef MAZE_EXPLORER__FRONTIER_SEARCH_HPP_
#define MAZE_EXPLORER__FRONTIER_SEARCH_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"

namespace maze_explorer
{

struct Candidate
{
  double x;
  double y;
  double path_distance;
  bool short_goal;
};

struct SearchResult
{
  std::vector<Candidate> candidates;
  // Keep the short tier available even when ordinary candidates exist, so a
  // planner can exhaust the ordinary batch before trying short approaches.
  std::vector<Candidate> short_candidates;
  std::size_t frontier_cells = 0;
  bool robot_in_known_free = false;
  bool robot_footprint_free = false;
  std::size_t reachable_cells = 0, frontier_clusters = 0, small_clusters = 0;
  std::size_t excluded_standpoints = 0, too_close_standpoints = 0, unreachable_frontiers = 0;
};

inline bool validMap(const nav_msgs::msg::OccupancyGrid & map)
{
  const auto & info = map.info;
  const auto & p = info.origin.position;
  const auto & q = info.origin.orientation;
  if (info.width == 0 || info.height == 0 ||
    info.width > static_cast<unsigned int>(std::numeric_limits<int>::max()) ||
    info.height > static_cast<unsigned int>(std::numeric_limits<int>::max()) ||
    !std::isfinite(info.resolution) || info.resolution <= 0.0 ||
    !std::isfinite(p.x) || !std::isfinite(p.y) ||
    !std::isfinite(q.x) || !std::isfinite(q.y) ||
    !std::isfinite(q.z) || !std::isfinite(q.w))
  {
    return false;
  }
  const auto cells = static_cast<std::uint64_t>(info.width) * info.height;
  const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
  return cells == map.data.size() && std::isfinite(norm) && norm > 1e-12;
}

namespace detail
{

constexpr std::array<std::pair<int, int>, 4> kDirections4 = {{
  {1, 0}, {-1, 0}, {0, 1}, {0, -1}}};
constexpr std::array<std::pair<int, int>, 8> kDirections8 = {{
  {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}}};

inline double originYaw(const nav_msgs::msg::OccupancyGrid & map)
{
  const auto & q = map.info.origin.orientation;
  const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
  const double x = q.x / norm;
  const double y = q.y / norm;
  const double z = q.z / norm;
  const double w = q.w / norm;
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

inline bool freeCell(const nav_msgs::msg::OccupancyGrid & map, int x, int y)
{
  if (x < 0 || y < 0 ||
    x >= static_cast<int>(map.info.width) || y >= static_cast<int>(map.info.height))
  {
    return false;
  }
  const auto value = map.data[static_cast<std::size_t>(y) * map.info.width + x];
  return value >= 0 && value < 50;
}

// Circle/rectangle intersection also catches cells touched by the circle's edge,
// including cells whose centers lie outside the robot's padded radius.
inline bool footprintFreeLocal(
  const nav_msgs::msg::OccupancyGrid & map, double x, double y, double radius,
  bool allow_unknown = false)
{
  const double resolution = map.info.resolution;
  const double width = map.info.width * resolution;
  const double height = map.info.height * resolution;
  if (!std::isfinite(x) || !std::isfinite(y) ||
    !std::isfinite(radius) || radius < 0.0 ||
    x - radius <= 0.0 || y - radius <= 0.0 ||
    x + radius >= width || y + radius >= height)
  {
    return false;
  }

  // Include the extra low-side cell so exact tangency at a cell edge is checked.
  const int min_x = std::max(0, static_cast<int>(std::floor((x - radius) / resolution)) - 1);
  const int min_y = std::max(0, static_cast<int>(std::floor((y - radius) / resolution)) - 1);
  const int max_x = std::min(
    static_cast<int>(map.info.width) - 1,
    static_cast<int>(std::floor((x + radius) / resolution)));
  const int max_y = std::min(
    static_cast<int>(map.info.height) - 1,
    static_cast<int>(std::floor((y + radius) / resolution)));
  const double radius_squared = radius * radius;
  for (int cy = min_y; cy <= max_y; ++cy) {
    for (int cx = min_x; cx <= max_x; ++cx) {
      const double dx = std::max({cx * resolution - x, 0.0, x - (cx + 1) * resolution});
      const double dy = std::max({cy * resolution - y, 0.0, y - (cy + 1) * resolution});
      if (dx * dx + dy * dy <= radius_squared + 1e-12) {
        const auto value = map.data[static_cast<std::size_t>(cy) * map.info.width + cx];
        if (value >= 50 || (!allow_unknown && value < 0)) {
          return false;
        }
      }
    }
  }
  return true;
}

inline bool goalFootprintFreeLocal(
  const nav_msgs::msg::OccupancyGrid & map, double x, double y,
  double radius, double obstacle_margin)
{
  if (!std::isfinite(obstacle_margin) || obstacle_margin < 0.0 ||
    !footprintFreeLocal(map, x, y, radius))
  {
    return false;
  }
  return obstacle_margin == 0.0 ||
    footprintFreeLocal(map, x, y, radius + obstacle_margin, true);
}

}  // namespace detail

inline bool worldToCell(
  const nav_msgs::msg::OccupancyGrid & map, double wx, double wy, int & cx, int & cy)
{
  cx = -1;
  cy = -1;
  if (!validMap(map) || !std::isfinite(wx) || !std::isfinite(wy)) {
    return false;
  }
  const double yaw = detail::originYaw(map);
  const double dx = wx - map.info.origin.position.x;
  const double dy = wy - map.info.origin.position.y;
  const double x = (std::cos(yaw) * dx + std::sin(yaw) * dy) / map.info.resolution;
  const double y = (-std::sin(yaw) * dx + std::cos(yaw) * dy) / map.info.resolution;
  // Test before converting to int: negative coordinates require floor, and huge
  // finite world coordinates must never overflow a cell index conversion.
  if (!std::isfinite(x) || !std::isfinite(y) ||
    x < 0.0 || y < 0.0 || x >= map.info.width || y >= map.info.height)
  {
    return false;
  }
  cx = static_cast<int>(std::floor(x));
  cy = static_cast<int>(std::floor(y));
  return true;
}

inline bool footprintFree(
  const nav_msgs::msg::OccupancyGrid & map, double wx, double wy, double radius)
{
  if (!validMap(map) || !std::isfinite(wx) || !std::isfinite(wy)) {
    return false;
  }
  const double yaw = detail::originYaw(map);
  const double dx = wx - map.info.origin.position.x;
  const double dy = wy - map.info.origin.position.y;
  return detail::footprintFreeLocal(
    map, std::cos(yaw) * dx + std::sin(yaw) * dy,
    -std::sin(yaw) * dx + std::cos(yaw) * dy, radius);
}

// The body must be fully observed free. The extra clearance applies to actual
// obstacles only: forcing the unknown frontier outside that margin can remove
// the short observation step needed to see around an occluding corner.
inline bool goalFootprintFree(
  const nav_msgs::msg::OccupancyGrid & map, double wx, double wy,
  double radius, double obstacle_margin)
{
  if (!validMap(map) || !std::isfinite(wx) || !std::isfinite(wy)) {
    return false;
  }
  const double yaw = detail::originYaw(map);
  const double dx = wx - map.info.origin.position.x;
  const double dy = wy - map.info.origin.position.y;
  return detail::goalFootprintFreeLocal(
    map, std::cos(yaw) * dx + std::sin(yaw) * dy,
    -std::sin(yaw) * dx + std::cos(yaw) * dy, radius, obstacle_margin);
}

inline bool segmentFootprintFree(
  const nav_msgs::msg::OccupancyGrid & map, double ax, double ay, double bx, double by,
  double radius)
{
  if (!validMap(map) || !std::isfinite(ax) || !std::isfinite(ay) ||
    !std::isfinite(bx) || !std::isfinite(by)) {return false;}
  const double length = std::hypot(bx - ax, by - ay);
  if (length > 2.0 * std::hypot(map.info.width * map.info.resolution,
      map.info.height * map.info.resolution)) {return false;}
  const int count = std::max(1, static_cast<int>(std::ceil(length / (map.info.resolution * 0.25))));
  for (int i = 0; i <= count; ++i) {
    const double t = static_cast<double>(i) / count;
    if (!footprintFree(map, ax + t * (bx - ax), ay + t * (by - ay), radius)) {return false;}
  }
  return true;
}

inline std::size_t knownFreeCells(const nav_msgs::msg::OccupancyGrid & map)
{
  if (!validMap(map)) {
    return 0;
  }
  return static_cast<std::size_t>(std::count_if(
    map.data.begin(), map.data.end(), [](std::int8_t value) {
      return value >= 0 && value < 50;
    }));
}

inline SearchResult findCandidates(
  const nav_msgs::msg::OccupancyGrid & map, double rx, double ry, double padded_radius,
  const std::function<bool(double, double)> & excluded, double obstacle_margin = 0.0)
{
  SearchResult result;
  if (!validMap(map) || !std::isfinite(padded_radius) || padded_radius < 0.0 ||
    !std::isfinite(obstacle_margin) || obstacle_margin < 0.0)
  {
    return result;
  }
  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);
  const std::size_t cells = map.data.size();
  const std::size_t no_cell = std::numeric_limits<std::size_t>::max();
  std::vector<unsigned char> frontier(cells, 0);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      if (!detail::freeCell(map, x, y)) {
        continue;
      }
      for (const auto & direction : detail::kDirections8) {
        const int nx = x + direction.first;
        const int ny = y + direction.second;
        if (nx >= 0 && ny >= 0 && nx < width && ny < height &&
          map.data[static_cast<std::size_t>(ny) * width + nx] < 0)
        {
          frontier[static_cast<std::size_t>(y) * width + x] = 1;
          ++result.frontier_cells;
          break;
        }
      }
    }
  }

  int sx = -1;
  int sy = -1;
  result.robot_in_known_free = worldToCell(map, rx, ry, sx, sy) &&
    detail::freeCell(map, sx, sy);
  if (!result.robot_in_known_free) {
    return result;
  }
  result.robot_footprint_free = footprintFree(map, rx, ry, padded_radius);
  if (!result.robot_footprint_free) {return result;}

  const double yaw = detail::originYaw(map);
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  const double resolution = map.info.resolution;
  std::vector<std::int8_t> safe(cells, -1);
  const auto is_safe = [&](std::size_t cell) {
      if (safe[cell] < 0) {
        const double x = (cell % width + 0.5) * resolution;
        const double y = (cell / width + 0.5) * resolution;
        safe[cell] = detail::goalFootprintFreeLocal(
          map, x, y, padded_radius, obstacle_margin) ? 1 : 0;
      }
      return safe[cell] != 0;
    };
  std::vector<std::size_t> steps(cells, no_cell);
  const std::size_t start = static_cast<std::size_t>(sy) * width + sx;
  // A raw-cell shortest path can hug an inside wall corner. Following that
  // path's parents backward then collapses distant frontiers onto the robot's
  // current cell, even when a route through the corridor center is clear.
  // Build the reachable component using the complete padded body instead.
  std::queue<std::size_t> queue;
  // Connect the actual pose to nearby grid centers with a collision-checked
  // sweep. Rounding a safe pose onto an unsafe center must not erase all goals.
  for (int y = std::max(0, sy - 2); y <= std::min(height - 1, sy + 2); ++y) {
    for (int x = std::max(0, sx - 2); x <= std::min(width - 1, sx + 2); ++x) {
      const double lx = (x + 0.5) * resolution, ly = (y + 0.5) * resolution;
      const double wx = map.info.origin.position.x + cosine * lx - sine * ly;
      const double wy = map.info.origin.position.y + sine * lx + cosine * ly;
      const double distance = std::hypot(wx - rx, wy - ry);
      if (distance > 2.0 * resolution || !segmentFootprintFree(map, rx, ry, wx, wy, padded_radius)) {continue;}
      const auto cell = static_cast<std::size_t>(y) * width + x;
      steps[cell] = cell == start ? 0 : static_cast<std::size_t>(std::ceil(distance / resolution));
      queue.push(cell);
    }
  }
  while (!queue.empty()) {
    const auto current = queue.front();
    queue.pop();
    const int x = static_cast<int>(current % width);
    const int y = static_cast<int>(current / width);
    for (const auto & direction : detail::kDirections4) {
      const int nx = x + direction.first;
      const int ny = y + direction.second;
      if (!detail::freeCell(map, nx, ny)) {
        continue;
      }
      const auto next = static_cast<std::size_t>(ny) * width + nx;
      if (steps[next] != no_cell || !is_safe(next)) {
        continue;
      }
      steps[next] = steps[current] + 1;
      queue.push(next);
    }
  }
  result.reachable_cells = static_cast<std::size_t>(std::count_if(
    steps.begin(), steps.end(), [no_cell](std::size_t s) {return s != no_cell;}));

  // Associate every raw frontier with its nearest safe, reachable standpoint.
  // Multi-source propagation avoids choosing an arbitrary wall-hugging parent
  // chain; navigation distance still comes from the safe component above.
  std::vector<std::size_t> owner(cells, no_cell);
  for (std::size_t cell = 0; cell < cells; ++cell) {
    if (steps[cell] != no_cell && is_safe(cell)) {
      owner[cell] = cell;
      queue.push(cell);
    }
  }
  while (!queue.empty()) {
    const auto current = queue.front();
    queue.pop();
    const int x = static_cast<int>(current % width);
    const int y = static_cast<int>(current / width);
    for (const auto & direction : detail::kDirections4) {
      const int nx = x + direction.first;
      const int ny = y + direction.second;
      if (!detail::freeCell(map, nx, ny)) {
        continue;
      }
      const auto next = static_cast<std::size_t>(ny) * width + nx;
      if (owner[next] != no_cell) {
        continue;
      }
      owner[next] = owner[current];
      queue.push(next);
    }
  }
  std::vector<unsigned char> visited(cells, 0);
  std::vector<unsigned char> used_standpoints(cells, 0);
  std::vector<Candidate> ordinary;
  std::vector<Candidate> short_candidates;
  for (std::size_t seed = 0; seed < cells; ++seed) {
    if (!frontier[seed] || visited[seed]) {
      continue;
    }
    std::vector<std::size_t> cluster;
    queue.push(seed);
    visited[seed] = 1;
    while (!queue.empty()) {
      const auto current = queue.front();
      queue.pop();
      cluster.push_back(current);
      const int x = static_cast<int>(current % width);
      const int y = static_cast<int>(current / width);
      for (const auto & direction : detail::kDirections8) {
        const int nx = x + direction.first;
        const int ny = y + direction.second;
        if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
          continue;
        }
        const auto next = static_cast<std::size_t>(ny) * width + nx;
        if (frontier[next] && !visited[next]) {
          visited[next] = 1;
          queue.push(next);
        }
      }
    }
    ++result.frontier_clusters;
    if (cluster.size() < 5) {
      ++result.small_clusters;
      continue;
    }
    for (const auto frontier_cell : cluster) {
      const auto standpoint = owner[frontier_cell];
      if (standpoint == no_cell) {++result.unreachable_frontiers; continue;}
      if (used_standpoints[standpoint]) {
        continue;
      }
      used_standpoints[standpoint] = 1;
      const double x = (standpoint % width + 0.5) * resolution;
      const double y = (standpoint / width + 0.5) * resolution;
      const double wx = map.info.origin.position.x + cosine * x - sine * y;
      const double wy = map.info.origin.position.y + sine * x + cosine * y;
      if (excluded && excluded(wx, wy)) {
        ++result.excluded_standpoints;
        continue;
      }
      const double path_distance = steps[standpoint] * resolution;
      const double euclidean_distance = std::hypot(wx - rx, wy - ry);
      // Apply both distances before choosing candidates. A long route to a goal
      // across a wall must not qualify when the XY goal checker would stop now.
      if (path_distance + 1e-9 >= 0.35 && euclidean_distance + 1e-9 >= 0.35) {
        ordinary.push_back({wx, wy, path_distance, false});
      } else if (path_distance + 1e-9 >= 0.15 && euclidean_distance + 1e-9 >= 0.15) {
        short_candidates.push_back({wx, wy, path_distance, true});
      } else {++result.too_close_standpoints;}
    }
  }
  // Preserve alternative standpoints from each cluster for planner validation.
  // A failure at one standpoint must not discard the rest of that frontier.
  const auto nearer = [](const Candidate & a, const Candidate & b) {
      if (a.path_distance != b.path_distance) {
        return a.path_distance < b.path_distance;
      }
      if (a.x != b.x) {
        return a.x < b.x;
      }
      return a.y < b.y;
    };
  result.short_candidates = std::move(short_candidates);
  std::sort(result.short_candidates.begin(), result.short_candidates.end(), nearer);
  result.candidates = ordinary.empty() ? result.short_candidates : std::move(ordinary);
  std::sort(result.candidates.begin(), result.candidates.end(), nearer);
  return result;
}

}  // namespace maze_explorer

#endif  // MAZE_EXPLORER__FRONTIER_SEARCH_HPP_
