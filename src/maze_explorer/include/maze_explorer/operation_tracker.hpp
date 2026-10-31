#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace maze_explorer
{
enum class Phase { Idle, Waiting, Planning, Sending, Navigating, Canceling, BackingUp, Spinning, Finished };

// A temporary exclusion is different from a progress-locked short observation.
class GoalExclusions
{
public:
  void remember(double x, double y, double until, double radius, bool locked, std::uint64_t epoch)
  {entries_.push_back({x, y, until, radius, locked, epoch});}
  double blockedUntil(double x, double y, double time, std::uint64_t epoch)
  {
    double until = time;
    for (auto it = entries_.begin(); it != entries_.end();) {
      const bool locked = it->locked && it->epoch == epoch;
      if (!locked && time >= it->until) {it = entries_.erase(it); continue;}
      if (std::hypot(x - it->x, y - it->y) < it->radius) {
        if (locked) {return std::numeric_limits<double>::infinity();}
        until = std::max(until, it->until);
      }
      ++it;
    }
    return until;
  }
private:
  struct Entry {double x, y, until, radius; bool locked; std::uint64_t epoch;};
  std::vector<Entry> entries_;
};

// A canceled operation still owns the motion channel until its terminal result arrives.
class OperationTracker
{
public:
  std::uint64_t begin(Phase phase)
  {
    if (phase_ != Phase::Idle || phase == Phase::Idle || phase == Phase::Waiting ||
      phase == Phase::Finished)
    {
      throw std::logic_error("operation started while another operation still owns control");
    }
    phase_ = phase;
    return ++generation_;
  }
  bool accepts(std::uint64_t generation) const
  {return generation == generation_ && phase_ != Phase::Idle &&
      phase_ != Phase::Waiting && phase_ != Phase::Finished;}
  bool acknowledge(std::uint64_t generation)
  {
    if (!accepts(generation)) {return false;}
    if (phase_ == Phase::Sending) {phase_ = Phase::Navigating;}
    return true;
  }
  bool cancel(std::uint64_t generation)
  {
    if (!accepts(generation)) {return false;}
    phase_ = Phase::Canceling;
    return true;
  }
  bool terminal(std::uint64_t generation)
  {
    if (!accepts(generation)) {return false;}
    phase_ = Phase::Idle;
    return true;
  }
  void finish() {++generation_; phase_ = Phase::Finished;}
  // Waiting owns no action and stays responsive to map/pose improvements.
  // Never release an active/canceling action through this transition.
  bool wait()
  {
    if (phase_ != Phase::Idle) {return false;}
    ++generation_;
    phase_ = Phase::Waiting;
    return true;
  }
  bool resume()
  {
    if (phase_ != Phase::Waiting) {return false;}
    phase_ = Phase::Idle;
    return true;
  }
  Phase phase() const {return phase_;}
  std::uint64_t generation() const {return generation_;}

private:
  Phase phase_{Phase::Idle};
  std::uint64_t generation_{0};
};

class ProgressWatchdog
{
public:
  ProgressWatchdog(double timeout = 30.0, double movement = 0.10)
  : timeout_(timeout), movement_(movement) {}
  void start(double time, double x, double y, double path_length)
  {
    started_ = progressed_ = time;
    x_ = x; y_ = y;
    total_limit_ = std::max(120.0, 3.0 * path_length / 0.12 + 30.0);
  }
  void observe(double time, double x, double y)
  {
    if (std::hypot(x - x_, y - y_) >= movement_) {
      x_ = x; y_ = y; progressed_ = time;
    }
  }
  bool noProgress(double time) const {return time - progressed_ >= timeout_;}
  bool totalExpired(double time) const {return time - started_ >= total_limit_;}

private:
  double timeout_, movement_, started_{0.0}, progressed_{0.0};
  double x_{0.0}, y_{0.0}, total_limit_{120.0};
};

// Driving back to the same observation point after BackUp is motion, but does
// not earn another recovery cycle indefinitely.
class VisitedGoals
{
public:
  bool mark(double x, double y)
  {
    for (const auto & point : points_) {
      if (std::hypot(x - point.first, y - point.second) < 0.15) {return false;}
    }
    points_.emplace_back(x, y);
    return true;
  }
private:
  std::vector<std::pair<double, double>> points_;
};
}  // namespace maze_explorer
