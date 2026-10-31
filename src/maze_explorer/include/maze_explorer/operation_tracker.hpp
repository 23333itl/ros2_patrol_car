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

// Confirmation is based on distinct fresh map observations, never timer ticks.
// Residual clusters need a quiet interval as well as the existing five-map rule.
class CompletionEvidence
{
public:
  explicit CompletionEvidence(double quiet_time = 10.0) : quiet_time_(quiet_time) {}
  bool observe(double time, std::uint64_t revision, bool has_small_frontiers)
  {
    if (maps_ == 0) {started_ = time;}
    if (maps_ == 0 || revision != last_revision_) {++maps_; last_revision_ = revision;}
    residual_ = residual_ || has_small_frontiers;
    return maps_ >= 5 && (!residual_ || time - started_ >= quiet_time_);
  }
  bool active() const {return maps_ > 0;}
  bool hasResidual() const {return residual_;}
  void reset() {maps_ = 0; residual_ = false;}

private:
  double quiet_time_, started_{0.0};
  std::size_t maps_{0};
  std::uint64_t last_revision_{0};
  bool residual_{false};
};

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

// Carry failed-candidate order across bounded search rounds. A timeout must not
// keep restarting at the same nearby goals and starve an untried distant goal.
class GoalAttemptHistory
{
public:
  void record(double x, double y)
  {
    for (auto & entry : entries_) {
      if (std::hypot(x - entry.x, y - entry.y) < 0.025) {
        entry.sequence = ++sequence_;
        return;
      }
    }
    if (entries_.size() >= 512) {
      entries_.erase(std::min_element(entries_.begin(), entries_.end(),
        [](const Entry & a, const Entry & b) {return a.sequence < b.sequence;}));
    }
    entries_.push_back({x, y, ++sequence_});
  }
  std::uint64_t lastAttempt(double x, double y) const
  {
    std::uint64_t last = 0;
    for (const auto & entry : entries_) {
      if (std::hypot(x - entry.x, y - entry.y) < 0.025) {
        last = std::max(last, entry.sequence);
      }
    }
    return last;
  }
  void clear() {entries_.clear(); sequence_ = 0;}

private:
  struct Entry {double x, y; std::uint64_t sequence;};
  std::vector<Entry> entries_;
  std::uint64_t sequence_{0};
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
