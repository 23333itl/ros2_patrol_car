#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/exceptions.hpp"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.hpp"
#include "maze_explorer/frontier_search.hpp"
#include "maze_explorer/operation_tracker.hpp"

using namespace std::chrono_literals;
using Navigate = nav2_msgs::action::NavigateToPose;
using ComputePath = nav2_msgs::action::ComputePathToPose;
using BackUp = nav2_msgs::action::BackUp;
using Spin = nav2_msgs::action::Spin;
using Phase = maze_explorer::Phase;
using Candidate = maze_explorer::Candidate;
using GetState = lifecycle_msgs::srv::GetState;
template<class Action> using Handle = rclcpp_action::ClientGoalHandle<Action>;

class ExplorerNode : public rclcpp::Node
{
public:
  ExplorerNode() : Node("maze_explorer")
  {
    save_path_ = declare_parameter<std::string>("map_save_path", "explored_maze");
    const double radius = declare_parameter("robot_radius", 0.13);
    const double padding = declare_parameter("footprint_padding", 0.0);
    // Nav2 pads each polygon vertex in both x and y, including diagonal vertices.
    // Its padded circle polygon can extend farther than radius + padding.
    padded_radius_ = radius + std::sqrt(2.0) * padding;
    goal_clearance_margin_ = declare_parameter("goal_clearance_margin", 0.025);
    map_timeout_ = declare_parameter("map_timeout", 5.0);
    tf_timeout_ = declare_parameter("tf_timeout", 2.0);
    action_timeout_ = declare_parameter("action_response_timeout", 10.0);
    planning_timeout_ = declare_parameter("planning_timeout", 10.0);
    candidate_search_timeout_ = declare_parameter("candidate_search_timeout", 12.0);
    recovery_scan_timeout_ = declare_parameter("recovery_scan_timeout", 0.5);
    const auto no_progress = declare_parameter("no_progress_timeout", 30.0);
    const auto movement = declare_parameter("required_movement", 0.10);
    if (radius <= 0 || padding < 0 || goal_clearance_margin_ < 0 ||
      padded_radius_ <= 0 || map_timeout_ <= 0 || tf_timeout_ <= 0 ||
      action_timeout_ <= 0 || planning_timeout_ <= 0 || recovery_scan_timeout_ <= 0 ||
      !std::isfinite(candidate_search_timeout_) || candidate_search_timeout_ <= 0 ||
      no_progress <= 0 || movement <= 0)
    {
      throw std::invalid_argument("exploration radii and timeouts must be positive");
    }
    watchdog_ = maze_explorer::ProgressWatchdog(no_progress, movement);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(1).transient_local().reliable(),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr map) {
        if (!maze_explorer::validMap(*map)) {
          empty_maps_ = 0;
          empty_since_ = -1.0;
          RCLCPP_WARN(get_logger(), "忽略无效地图");
          return;
        }
        map_ = map;
        map_received_ = now().seconds();
        ++map_revision_;
        // An intervening uncertain/frontier map breaks the completion streak even
        // if the slower decision timer never samples that particular revision.
        if (empty_maps_ > 0) {
          Pose robot;
          if (!mapFresh() || !pose("map", robot)) {empty_maps_ = 0; empty_since_ = -1.0; return;}
          const auto check = maze_explorer::findCandidates(*map_, robot.x, robot.y,
            padded_radius_, [](double, double) {return true;});
          if (!check.robot_footprint_free || check.actionable_frontier_cells != 0 ||
            check.unresolved_frontier_cells != 0) {empty_maps_ = 0; empty_since_ = -1.0;}
        }
      });
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::SharedPtr scan) {
        scans_.push_back({scan, now().seconds()});
        while (scans_.size() > 16) {scans_.pop_front();}
      });
    nav_client_ = rclcpp_action::create_client<Navigate>(this, "navigate_to_pose");
    plan_client_ = rclcpp_action::create_client<ComputePath>(this, "compute_path_to_pose");
    backup_client_ = rclcpp_action::create_client<BackUp>(this, "backup");
    spin_client_ = rclcpp_action::create_client<Spin>(this, "spin");
    for (const auto * name : {"bt_navigator", "planner_server", "controller_server",
      "behavior_server", "velocity_smoother"})
    {
      lifecycle_.push_back({create_client<GetState>(std::string("/") + name + "/get_state")});
    }
    // ROS-clock timers pause with Gazebo; wall-clock tick counting changes recovery distance.
    timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration(500ms),
      [this]() {onTimer();});
    RCLCPP_INFO(get_logger(), "探索节点已启动：完整半径 %.3fm，地图保存 %s；恢复使用 Nav2 action",
      padded_radius_, save_path_.c_str());
  }

private:
  enum class Kind { Plan, Navigation, Backup, Spin };
  struct Pose {double x{}, y{}, yaw{};};
  struct LifecycleCheck
  {
    rclcpp::Client<GetState>::SharedPtr client;
    bool active{false}, pending{false};
    double checked{-100.0}, requested{-100.0};
    std::int64_t request_id{0};
    std::uint64_t generation{0};
  };

  bool pose(const std::string & frame, Pose & p)
  {
    try {
      const auto tf = tf_buffer_->lookupTransform(frame, "base_footprint", tf2::TimePointZero);
      const double age = now().seconds() - rclcpp::Time(tf.header.stamp).seconds();
      if (age > tf_timeout_ || age < -0.5) {return false;}
      p.x = tf.transform.translation.x;
      p.y = tf.transform.translation.y;
      const auto & q = tf.transform.rotation;
      p.yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.yaw);
    } catch (const tf2::TransformException &) {return false;}
  }

  bool mapFresh() const
  {
    if (!map_) {return false;}
    const double time = now().seconds();
    const double age = time - rclcpp::Time(map_->header.stamp).seconds();
    return time - map_received_ <= map_timeout_ && age <= map_timeout_ && age >= -0.5;
  }

  bool navigationActive()
  {
    const double time = now().seconds();
    bool ready = true;
    for (std::size_t i = 0; i < lifecycle_.size(); ++i) {
      auto & check = lifecycle_[i];
      if (check.pending && time - check.requested >= action_timeout_) {
        check.client->remove_pending_request(check.request_id);
        check.pending = false;
        check.active = false;
        ++check.generation;
      }
      ready = ready && check.active && time - check.checked <= 5.0;
      if (check.pending || time - check.requested < 2.0 || !check.client->service_is_ready()) {
        continue;
      }
      check.pending = true;
      check.requested = time;
      const auto generation = ++check.generation;
      check.request_id = check.client->async_send_request(std::make_shared<GetState::Request>(),
        [this, i, generation](rclcpp::Client<GetState>::SharedFuture result) {
          auto & item = lifecycle_[i];
          if (item.generation != generation) {return;}
          item.pending = false;
          item.checked = now().seconds();
          item.active = result.get()->current_state.id ==
            lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
        }).request_id;
    }
    return ready && nav_client_->action_server_is_ready() && plan_client_->action_server_is_ready();
  }

  double freeArea() const
  {
    return map_ ? maze_explorer::knownFreeCells(*map_) *
      map_->info.resolution * map_->info.resolution : 0.0;
  }

  void updateObservationProgress()
  {
    if (map_revision_ == progress_map_revision_) {return;}
    progress_map_revision_ = map_revision_;
    const double area = freeArea();
    if (area >= progress_area_ + 0.02) {
      ++progress_epoch_;
      progress_area_ = area;
    }
  }

  bool excluded(double x, double y)
  {
    const double time = now().seconds();
    return remembered_.blockedUntil(x, y, time, progress_epoch_) > time;
  }

  void remember(const Candidate & candidate, bool failed, bool no_progress = false)
  {
    remembered_.remember(candidate.x, candidate.y, now().seconds() + (failed ? 30.0 : 10.0),
      failed ? 0.15 : 0.10, no_progress, progress_epoch_);
  }

  geometry_msgs::msg::PoseStamped targetPose(const Candidate & candidate)
  {
    geometry_msgs::msg::PoseStamped target;
    target.header.frame_id = "map";
    target.header.stamp = now();
    target.pose.position.x = candidate.x;
    target.pose.position.y = candidate.y;
    Pose robot;
    const double yaw = pose("map", robot) ?
      std::atan2(candidate.y - robot.y, candidate.x - robot.x) : 0.0;
    target.pose.orientation.z = std::sin(yaw / 2.0);
    target.pose.orientation.w = std::cos(yaw / 2.0);
    return target;
  }

  std::uint64_t begin(Kind kind, Phase phase)
  {
    kind_ = kind;
    op_started_ = now().seconds();
    cancel_sent_ = false;
    cancel_reason_.clear();
    return operation_.begin(phase);
  }

  void tryNextCandidate()
  {
    if (operation_.phase() != Phase::Idle) {return;}
    if (!mapFresh()) {return;}
    const double time = now().seconds();
    if (candidate_search_started_ < 0) {candidate_search_started_ = time;}
    if (time - candidate_search_started_ >= candidate_search_timeout_) {
      noCandidate(); return;
    }
    // Reserve the final third of the shared budget for blind-corner approaches.
    // A long ordinary tier must not consume all opportunities for a short goal.
    if (!short_candidates_.empty() &&
      time - candidate_search_started_ >= candidate_search_timeout_ * 2.0 / 3.0)
    {
      candidates_ = std::move(short_candidates_);
      short_candidates_.clear();
      candidate_index_ = 0;
    }
    if (candidate_index_ >= candidates_.size()) {
      // Keep the short tier for the entire batch, independent of per-point cooldowns.
      if (!short_candidates_.empty()) {
        candidates_ = std::move(short_candidates_);
        short_candidates_.clear();
        candidate_index_ = 0;
      } else {noCandidate(); return;}
    }
    candidate_ = candidates_[candidate_index_++];
    if (excluded(candidate_.x, candidate_.y) ||
      !maze_explorer::goalFootprintFree(*map_, candidate_.x, candidate_.y,
        padded_radius_, goal_clearance_margin_))
    {
      // Do not recurse through a large frontier; continue at the next timer tick.
      retry_candidates_ = true;
      return;
    }
    retry_candidates_ = false;
    ComputePath::Goal goal;
    goal.goal = targetPose(candidate_);
    goal.planner_id = "GridBased";
    goal.use_start = false;
    const auto token = begin(Kind::Plan, Phase::Planning);
    auto options = rclcpp_action::Client<ComputePath>::SendGoalOptions();
    options.goal_response_callback = [this, token](Handle<ComputePath>::SharedPtr handle) {
      if (!operation_.accepts(token)) {
        if (handle) {plan_client_->async_cancel_goal(handle);}
        return;
      }
      if (!handle) {operation_.terminal(token); planFailed(); return;}
      plan_handle_ = handle;
      if (operation_.phase() == Phase::Canceling) {sendCancel();}
    };
    options.result_callback = [this, token](const Handle<ComputePath>::WrappedResult & result) {
      if (!operation_.accepts(token)) {return;}
      const bool canceled = operation_.phase() == Phase::Canceling;
      operation_.terminal(token);
      plan_handle_.reset();
      if (canceled || result.code != rclcpp_action::ResultCode::SUCCEEDED ||
        !result.result || result.result->error_code != ComputePath::Result::NONE ||
        result.result->path.poses.empty() || !mapFresh())
      {
        planFailed(); return;
      }
      const auto & path = result.result->path;
      if (path.header.frame_id != "map") {planFailed(); return;}
      const auto & end = path.poses.back().pose.position;
      if (std::hypot(end.x - candidate_.x, end.y - candidate_.y) > 0.075 ||
        !maze_explorer::goalFootprintFree(*map_, candidate_.x, candidate_.y,
          padded_radius_, goal_clearance_margin_))
      {
        planFailed(); return;
      }
      Pose actual;
      if (!pose("map", actual)) {planFailed(); return;}
      const auto checked = maze_explorer::checkPathFromPose(
        *map_, path, actual.x, actual.y, padded_radius_);
      if (!checked.safe) {
        if (blocked_paths_.size() >= 8) {blocked_paths_.erase(blocked_paths_.begin());}
        blocked_paths_.push_back(path);
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "规划路径的完整车体净空不足：目标(%.2f,%.2f)，段(%.2f,%.2f)->(%.2f,%.2f)",
          candidate_.x, candidate_.y, checked.ax, checked.ay, checked.bx, checked.by);
        if (checked.start_blocked) {
          if (std::hypot(checked.ax - blocked_start_.x, checked.ay - blocked_start_.y) > .05) {
            start_failures_ = 0;
          }
          blocked_start_ = {checked.ax, checked.ay, 0.0};
          ++start_failures_;
        } else {start_failures_ = 0;}
        planFailed();
        if (start_failures_ >= 2 && !waypoint_mode_) {
          retry_candidates_ = false;
          candidate_search_started_ = now().seconds() - candidate_search_timeout_;
          noCandidate();
        }
        return;
      }
      if (checked.joined_at > 0) {
        RCLCPP_INFO(get_logger(), "已从实际位姿安全接入路径第 %zu 点；仅重接起点 0.05m，完整车体检查通过",
          checked.joined_at);
      }
      start_failures_ = 0;
      sendNavigation(checked.length);
    };
    plan_client_->async_send_goal(goal, options);
  }

  void planFailed()
  {
    // A failed representative must not blacklist its entire frontier cluster.
    attempted_goals_.record(candidate_.x, candidate_.y);
    remembered_.remember(candidate_.x, candidate_.y, now().seconds() + 10.0,
      0.025, false, progress_epoch_);
    retry_candidates_ = true;
  }

  void sendNavigation(double length)
  {
    Pose robot;
    if (!mapFresh() || !pose("odom", robot)) {return;}
    candidate_search_started_ = -1.0;
    attempted_goals_.clear();
    const bool intermediate = waypoint_mode_;
    nav_start_ = robot;
    nav_start_area_ = freeArea();
    watchdog_.start(now().seconds(), robot.x, robot.y, length);
    Navigate::Goal goal;
    goal.pose = targetPose(candidate_);
    const auto token = begin(Kind::Navigation, Phase::Sending);
    auto options = rclcpp_action::Client<Navigate>::SendGoalOptions();
    options.goal_response_callback = [this, token](Handle<Navigate>::SharedPtr handle) {
      if (!operation_.accepts(token)) {
        if (handle) {nav_client_->async_cancel_goal(handle);}
        return;
      }
      if (!handle) {
        operation_.terminal(token);
        waypoint_mode_ = false;
        RCLCPP_WARN(get_logger(), "Nav2 拒绝目标；等待生命周期与地图就绪后重选");
        retry_candidates_ = false;
        return;
      }
      nav_handle_ = handle;
      operation_.acknowledge(token);
      if (operation_.phase() == Phase::Canceling) {sendCancel();}
    };
    options.result_callback = [this, token, intermediate](const Handle<Navigate>::WrappedResult & result) {
      if (!operation_.accepts(token)) {return;}
      const bool canceled = operation_.phase() == Phase::Canceling;
      operation_.terminal(token);
      nav_handle_.reset();
      Pose current;
      const bool moved = pose("odom", current) &&
        std::hypot(current.x - nav_start_.x, current.y - nav_start_.y) >= 0.05;
      const bool observed = freeArea() >= nav_start_area_ + 0.01;
      const bool progressed = moved || observed;
      const bool succeeded = result.code == rclcpp_action::ResultCode::SUCCEEDED &&
        result.result && result.result->error_code == Navigate::Result::NONE;
      const bool novel_goal = succeeded && visited_goals_.mark(candidate_.x, candidate_.y);
      if (progressed) {
        ++progress_epoch_;
      }
      if (moved && novel_goal && !intermediate) {
        recovery_used_ = false;
        waypoint_used_ = false;
        blocked_paths_.clear();
        no_candidate_since_ = -1.0;
      }
      remember(candidate_, !succeeded, !progressed);
      if (!succeeded || !progressed) {attempted_goals_.record(candidate_.x, candidate_.y);}
      RCLCPP_INFO(get_logger(), "导航终止：%s，实际进展=%s，目标(%.2f,%.2f)",
        succeeded ? "成功" : "失败/取消", progressed ? "有" : "无", candidate_.x, candidate_.y);
      retry_candidates_ = false;
      waypoint_mode_ = false;
      if (canceled || !succeeded) {
        // Exhaust the remaining verified standpoints before spending recovery
        // or declaring this entire frontier unreachable.
        retry_candidates_ = !intermediate;
      }
    };
    nav_client_->async_send_goal(goal, options);
    writeStatus("exploring", false, intermediate ? "执行一次增加净空的中间站位" : "安全路径验证通过", false);
    RCLCPP_INFO(get_logger(), "探索%s目标(%.2f,%.2f)，已验证路径 %.2fm",
      candidate_.short_goal ? "短距" : "", candidate_.x, candidate_.y, length);
  }

  void requestCancel(const std::string & reason)
  {
    if (operation_.phase() == Phase::Canceling || operation_.phase() == Phase::Idle) {return;}
    cancel_reason_ = reason;
    cancel_started_ = now().seconds();
    operation_.cancel(operation_.generation());
    RCLCPP_WARN(get_logger(), "取消当前 action：%s；收到终止结果前不执行恢复", reason.c_str());
    sendCancel();
  }

  void sendCancel()
  {
    if (cancel_sent_) {return;}
    if (kind_ == Kind::Plan && plan_handle_) {plan_client_->async_cancel_goal(plan_handle_);}
    else if (kind_ == Kind::Navigation && nav_handle_) {nav_client_->async_cancel_goal(nav_handle_);}
    else if (kind_ == Kind::Backup && backup_handle_) {backup_client_->async_cancel_goal(backup_handle_);}
    else if (kind_ == Kind::Spin && spin_handle_) {spin_client_->async_cancel_goal(spin_handle_);}
    else {return;}  // The request can time out before the goal response arrives.
    cancel_sent_ = true;
  }

  bool recoverySensorsReady()
  {
    sensor_reason_ = "尚未收到激光";
    const double time = now().seconds();
    bool newest = true;
    for (auto it = scans_.rbegin(); it != scans_.rend(); ++it) {
      const auto & scan = *it->scan;
      std::string reason;
      const double age = time - rclcpp::Time(scan.header.stamp).seconds();
      if (age < -0.1 || age > recovery_scan_timeout_ ||
        time - it->received > recovery_scan_timeout_) {reason = "激光过期或时钟不一致";}
      else if (scan.ranges.size() < 36 || scan.header.frame_id.empty() ||
        !std::isfinite(scan.angle_min) || !std::isfinite(scan.angle_max) ||
        !std::isfinite(scan.angle_increment) || scan.angle_increment <= 0.0 ||
        !std::isfinite(scan.range_min) || !std::isfinite(scan.range_max) ||
        scan.range_min < 0.0 || scan.range_max <= scan.range_min)
      {reason = "激光元数据无效";}
      else {
        const double span = (scan.ranges.size() - 1) * static_cast<double>(scan.angle_increment);
        if (span < 2.0 * std::acos(-1.0) - 2.0 * scan.angle_increment ||
          std::abs(scan.angle_max - scan.angle_min - span) > 2.0 * scan.angle_increment)
        {reason = "激光未覆盖完整 360°";}
        else {
          for (const auto range : scan.ranges) {
            // +inf means no return; NaN/-inf/out-of-range leave an unseen sector.
            if (std::isnan(range) || range < scan.range_min ||
              (std::isfinite(range) && range > scan.range_max))
            {reason = "激光存在无效测距"; break;}
          }
          if (reason.empty() && !tf_buffer_->canTransform("odom", scan.header.frame_id,
            rclcpp::Time(scan.header.stamp), rclcpp::Duration::from_seconds(0.0)))
          {reason = "扫描时刻的 odom 到激光 TF 尚未就绪";}
        }
      }
      if (reason.empty()) {sensor_reason_.clear(); return true;}
      if (newest) {sensor_reason_ = reason; newest = false;}
    }
    return false;
  }

  void startRecovery()
  {
    if (operation_.phase() != Phase::Idle) {return;}
    if (recovery_used_) {no_candidate_since_ = now().seconds(); return;}
    const bool sensors_ready = recoverySensorsReady();
    const bool nav_ready = navigationActive();
    if (!sensors_ready || !nav_ready) {
      waiting_for_recovery_ = true;
      waitForImprovement("恢复等待：" + (sensors_ready ? std::string("Nav2 未全部 active") : sensor_reason_));
      return;
    }
    waiting_for_recovery_ = false;
    recovery_used_ = true;
    // Nav2 checks the complete swept footprint against the odom-frame live
    // obstacle layer. A displaced global SLAM wall must not veto that check.
    if (!backup_client_->action_server_is_ready()) {startSpin(); return;}
    BackUp::Goal goal;
    goal.target.x = -0.15;
    goal.speed = 0.05;  // BackUp accepts a positive speed magnitude.
    goal.time_allowance = rclcpp::Duration::from_seconds(8.0);
    const auto token = begin(Kind::Backup, Phase::BackingUp);
    auto options = rclcpp_action::Client<BackUp>::SendGoalOptions();
    options.goal_response_callback = [this, token](Handle<BackUp>::SharedPtr handle) {
      if (!operation_.accepts(token)) {
        if (handle) {backup_client_->async_cancel_goal(handle);}
        return;
      }
      if (!handle) {operation_.terminal(token); recoveryFinished(); return;}
      backup_handle_ = handle;
      if (operation_.phase() == Phase::Canceling) {sendCancel();}
    };
    options.result_callback = [this, token](const Handle<BackUp>::WrappedResult & result) {
      if (!operation_.accepts(token)) {return;}
      const bool canceled = operation_.phase() == Phase::Canceling;
      operation_.terminal(token);
      backup_handle_.reset();
      if (!canceled && result.code == rclcpp_action::ResultCode::SUCCEEDED &&
        result.result && result.result->error_code == BackUp::Result::NONE) {startSpin();}
      else {recoveryFinished();}
    };
    backup_client_->async_send_goal(goal, options);
    RCLCPP_INFO(get_logger(), "安全恢复：Nav2 BackUp 0.15m / 0.05m/s");
  }

  void startSpin()
  {
    if (!recoverySensorsReady() || !navigationActive() ||
      !spin_client_->action_server_is_ready()) {recoveryFinished(); return;}
    Spin::Goal goal;
    goal.target_yaw = static_cast<float>(std::acos(-1.0));
    goal.time_allowance = rclcpp::Duration::from_seconds(20.0);
    const auto token = begin(Kind::Spin, Phase::Spinning);
    auto options = rclcpp_action::Client<Spin>::SendGoalOptions();
    options.goal_response_callback = [this, token](Handle<Spin>::SharedPtr handle) {
      if (!operation_.accepts(token)) {
        if (handle) {spin_client_->async_cancel_goal(handle);}
        return;
      }
      if (!handle) {operation_.terminal(token); recoveryFinished(); return;}
      spin_handle_ = handle;
      if (operation_.phase() == Phase::Canceling) {sendCancel();}
    };
    options.result_callback = [this, token](const Handle<Spin>::WrappedResult & result) {
      if (!operation_.terminal(token)) {return;}
      spin_handle_.reset();
      RCLCPP_INFO(get_logger(), "旋转恢复终止，action 状态=%d", static_cast<int>(result.code));
      recoveryFinished();
    };
    spin_client_->async_send_goal(goal, options);
    RCLCPP_INFO(get_logger(), "安全恢复：Nav2 Spin π rad，受统一限速和碰撞检查约束");
  }

  void recoveryFinished()
  {
    waypoint_mode_ = false;
    retry_candidates_ = false;
    candidate_search_started_ = -1.0;
    no_candidate_since_ = now().seconds();
    RCLCPP_INFO(get_logger(), "恢复结束，等待新地图并重新选择前沿");
  }

  bool tryClearanceWaypoint()
  {
    Pose actual;
    if (waypoint_used_ || !mapFresh() || !pose("map", actual)) {return false;}
    auto points = maze_explorer::clearanceWaypoints(
      *map_, actual.x, actual.y, padded_radius_, goal_clearance_margin_);
    points.erase(std::remove_if(points.begin(), points.end(), [this](const auto & c) {
        return excluded(c.x, c.y);
      }), points.end());
    if (points.empty()) {return false;}
    waypoint_used_ = waypoint_mode_ = true;
    if (points.size() > 8) {points.resize(8);}
    candidates_ = std::move(points);
    short_candidates_.clear(); candidate_index_ = 0;
    candidate_search_started_ = now().seconds();
    RCLCPP_INFO(get_logger(), "同一起步段反复失败，检查一次 0.15～0.35m 增加墙边净空的中间站位");
    tryNextCandidate();
    return true;
  }

  bool waitingSceneImproved(const Pose & robot, const maze_explorer::SearchResult & geometry)
  {
    if (waiting_odom_valid_ &&
      std::hypot(robot_odom_.x - waiting_odom_.x, robot_odom_.y - waiting_odom_.y) >= .10)
    {return true;}
    if (!waiting_body_free_ && geometry.robot_footprint_free) {return true;}
    if (map_revision_ == waiting_checked_revision_) {return false;}
    waiting_checked_revision_ = map_revision_;
    for (const auto & path : blocked_paths_) {
      if (maze_explorer::checkPathFromPose(*map_, path, robot.x, robot.y, padded_radius_).safe) {
        return true;
      }
    }
    const auto new_standpoint = [this](const auto & points) {
        for (const auto & c : points) {
          const auto absent = [&](const auto & before) {
              return std::none_of(before.begin(), before.end(), [&](const auto & old) {
                  return std::hypot(c.x - old.x, c.y - old.y) <= .05;
                });
            };
          if (absent(waiting_geometry_.candidates) && absent(waiting_geometry_.short_candidates)) {
            return true;
          }
        }
        return false;
      };
    return new_standpoint(geometry.candidates) || new_standpoint(geometry.short_candidates);
  }

  void noCandidate()
  {
    retry_candidates_ = false;
    const double time = now().seconds();
    if (operation_.phase() == Phase::Waiting) {return;}
    if (no_candidate_since_ < 0) {no_candidate_since_ = time;}
    if (candidate_search_started_ < 0) {candidate_search_started_ = time;}
    const bool search_expired = time - candidate_search_started_ >= candidate_search_timeout_;
    Pose robot;
    if (!search_expired && mapFresh() && pose("map", robot)) {
      const auto geometric = maze_explorer::findCandidates(*map_, robot.x, robot.y,
        padded_radius_, {}, goal_clearance_margin_);
      double retry_at = std::numeric_limits<double>::infinity();
      const auto consider = [&](const auto & candidates) {
          for (const auto & c : candidates) {
            const double until = remembered_.blockedUntil(c.x, c.y, time, progress_epoch_);
            if (until > time && std::isfinite(until)) {retry_at = std::min(retry_at, until);}
          }
        };
      consider(geometric.candidates);
      consider(geometric.short_candidates);
      if (std::isfinite(retry_at) &&
        retry_at <= candidate_search_started_ + candidate_search_timeout_)
      {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
          "安全候选仍在冷却，约 %.1f 秒后可重试；暂不判定停滞", retry_at - time);
        return;
      }
    }
    // Allow map updates and the exact-point planning cooldown to expire before moving.
    if (!search_expired &&
      time - no_candidate_since_ < (recovery_used_ ? 12.0 : 10.0)) {return;}
    if (search_expired) {
      RCLCPP_WARN(get_logger(), "连续选点 %.1f 秒仍无安全路径，结束本轮重试并检查有界恢复",
        time - candidate_search_started_);
    }
    if (!waypoint_mode_ && start_failures_ >= 2 && tryClearanceWaypoint()) {return;}
    waypoint_mode_ = false;
    if (!recovery_used_) {startRecovery();}
    else {waitForImprovement("仍有未知前沿，但安全站位/路径不可达；有界恢复后等待地图或位姿改善");}
  }

  void decide()
  {
    if (!mapFresh()) {
      empty_maps_ = 0; empty_since_ = -1.0;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "等待新鲜 /map；尚不发送探索目标");
      return;
    }
    if (!navigationActive()) {
      empty_maps_ = 0; empty_since_ = -1.0;
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "等待 Nav2 生命周期节点全部 active");
      return;
    }
    Pose robot;
    if (!pose("map", robot) || !pose("odom", robot_odom_)) {
      empty_maps_ = 0; empty_since_ = -1.0;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "等待新鲜 map/odom 到 base_footprint 的 TF");
      return;
    }
    if (retry_candidates_) {tryNextCandidate(); return;}
    const auto result = maze_explorer::findCandidates(*map_, robot.x, robot.y,
      padded_radius_, [this](double x, double y) {return excluded(x, y);},
      goal_clearance_margin_);
    last_search_ = result;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
      "前沿筛选：原始=%zu 簇=%zu 小簇=%zu 可达格=%zu 不可达前沿=%zu 冷却/锁定=%zu 过近=%zu 起点自由=%d 车体净空=%d",
      result.frontier_cells, result.frontier_clusters, result.small_clusters,
      result.reachable_cells, result.unreachable_frontiers, result.excluded_standpoints,
      result.too_close_standpoints, result.robot_in_known_free, result.robot_footprint_free);
    if (!result.robot_footprint_free) {
      empty_maps_ = 0; empty_since_ = -1.0;
      if (operation_.phase() != Phase::Waiting) {noCandidate();}
      else if (waiting_for_recovery_ && !recovery_used_ && recoverySensorsReady()) {
        operation_.resume(); startRecovery();
      }
      return;
    }
    if (result.actionable_frontier_cells == 0 && result.unresolved_frontier_cells == 0) {
      if (last_empty_revision_ != map_revision_) {
        last_empty_revision_ = map_revision_;
        if (empty_maps_ == 0) {empty_since_ = now().seconds();}
        ++empty_maps_;
      }
      if (empty_maps_ >= 5 && now().seconds() - empty_since_ >= 5.0) {
        finish(true, "连续五张新鲜地图且至少 5 秒没有可探索前沿；墙边不可观测碎格单独记录");
      }
      return;
    }
    empty_maps_ = 0;
    empty_since_ = -1.0;
    if (operation_.phase() == Phase::Waiting) {
      if (waiting_for_recovery_ && !recovery_used_ && recoverySensorsReady()) {
        operation_.resume(); startRecovery(); return;
      }
      const auto geometry = maze_explorer::findCandidates(*map_, robot.x, robot.y,
        padded_radius_, {}, goal_clearance_margin_);
      const bool improved = waitingSceneImproved(robot, geometry);
      const auto untried = [this](const auto & points) {
          return std::any_of(points.begin(), points.end(), [&](const auto & c) {
              return !excluded(c.x, c.y) && attempted_goals_.lastAttempt(c.x, c.y) == 0;
            });
        };
      // All those routes share this blocked start. A new destination cannot
      // repair it; do not spend another round on every distant standpoint.
      const bool remaining_routes = start_failures_ < 2 &&
        (untried(geometry.candidates) || untried(geometry.short_candidates));
      if (!improved && !remaining_routes) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 30000,
          "静止等待实际位姿或相关通路改善；冷却到期及相同地图不重新启动选点");
        return;
      }
      operation_.resume();
      candidate_search_started_ = no_candidate_since_ = -1.0;
      if (improved) {start_failures_ = 0; waypoint_used_ = false;}
      RCLCPP_INFO(get_logger(), "%s，开启一次有界检查；安全路径通过后才恢复导航",
        improved ? "位姿或可观测通路改善" : "继续检查本轮尚未尝试的站位，不补充恢复预算");
    }
    candidates_ = result.candidates;
    short_candidates_ = !candidates_.empty() && !candidates_.front().short_goal ?
      result.short_candidates : std::vector<Candidate>{};
    const auto least_recently_tried = [this](const Candidate & a, const Candidate & b) {
        return attempted_goals_.lastAttempt(a.x, a.y) < attempted_goals_.lastAttempt(b.x, b.y);
      };
    // Stable ordering preserves path-distance preference among untried goals.
    std::stable_sort(candidates_.begin(), candidates_.end(), least_recently_tried);
    std::stable_sort(short_candidates_.begin(), short_candidates_.end(), least_recently_tried);
    candidate_index_ = 0;
    if (candidates_.empty()) {noCandidate(); return;}
    RCLCPP_INFO(get_logger(), "前沿格 %zu，安全%s候选 %zu，机器人(%.2f,%.2f)",
      result.frontier_cells, candidates_.front().short_goal ? "短距" : "普通",
      candidates_.size(), robot.x, robot.y);
    tryNextCandidate();
  }

  void onTimer()
  {
    if (operation_.phase() == Phase::Finished) {return;}
    const double time = now().seconds();
    if (last_tick_ >= 0 && time < last_tick_) {
      finish(false, "仿真时钟回退；请重启探索以清除旧目标与地图");
      return;
    }
    last_tick_ = time;
    // Keep lifecycle results fresh during navigation as well as while idle.
    // Otherwise each successful goal is followed by a stale-cache discovery pause.
    navigationActive();
    updateObservationProgress();
    const auto phase = operation_.phase();
    if (phase == Phase::Canceling) {
      sendCancel();
      if (time - cancel_started_ >= action_timeout_) {
        finish(false, "取消未收到终止确认；保留控制权，不启动新的运动");
      }
      return;
    }
    if (phase == Phase::Idle || phase == Phase::Waiting) {
      if (retry_candidates_ || time - last_decision_ >= 2.0) {
        last_decision_ = time;
        decide();
      }
      return;
    }
    if (!mapFresh()) {requestCancel("地图过期"); return;}
    if (phase == Phase::BackingUp || phase == Phase::Spinning) {
      if (!recoverySensorsReady()) {requestCancel("恢复期间：" + sensor_reason_); return;}
      if (!navigationActive()) {requestCancel("恢复期间 Nav2 未全部 active"); return;}
    }
    if (phase == Phase::Planning) {
      if (time - op_started_ >= planning_timeout_) {requestCancel("规划超时"); return;}
      if (candidate_search_started_ >= 0 &&
        time - candidate_search_started_ >= candidate_search_timeout_)
      {requestCancel("连续候选规划达到总重试时限"); return;}
    }
    if (phase == Phase::Sending && time - op_started_ >= action_timeout_) {
      requestCancel("导航目标确认超时"); return;
    }
    if (phase == Phase::Navigating) {
      Pose odom;
      if (!pose("odom", odom)) {requestCancel("里程计 TF 过期"); return;}
      watchdog_.observe(time, odom.x, odom.y);
      if (watchdog_.noProgress(time)) {requestCancel("30 秒无足够平移进展");}
      else if (watchdog_.totalExpired(time)) {requestCancel("单目标总时限到达");}
    }
    if ((phase == Phase::BackingUp || phase == Phase::Spinning) &&
      time - op_started_ >= (phase == Phase::BackingUp ? 10.0 : 25.0))
    {
      requestCancel("恢复 action 超时");
    }
  }

  bool saveMap() const
  {
    if (!map_) {return false;}
    const auto & map = *map_;
    std::ofstream pgm(save_path_ + ".pgm", std::ios::binary);
    if (!pgm) {return false;}
    pgm << "P5\n" << map.info.width << ' ' << map.info.height << "\n255\n";
    for (int y = static_cast<int>(map.info.height) - 1; y >= 0; --y) {
      for (std::uint32_t x = 0; x < map.info.width; ++x) {
        const auto value = map.data[y * map.info.width + x];
        pgm.put(static_cast<char>(value < 0 ? 205 : value >= 50 ? 0 : 254));
      }
    }
    pgm.close();
    if (!pgm) {return false;}
    std::ofstream yaml(save_path_ + ".yaml");
    const auto & q = map.info.origin.orientation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    const auto basename = save_path_.substr(save_path_.find_last_of('/') + 1);
    yaml << "image: " << std::quoted(basename + ".pgm") << "\nresolution: " <<
      map.info.resolution << "\norigin: [" << map.info.origin.position.x << ", " <<
      map.info.origin.position.y << ", " << yaw << "]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n";
    yaml.close();
    return static_cast<bool>(yaml);
  }

  void writeStatus(
    const std::string & state, bool completed, const std::string & reason, bool save)
  {
    const bool saved = save ? saveMap() : false;
    std::ofstream status(save_path_ + ".status.yaml");
    status << "status: " << state << "\ncompleted: " << (completed ? "true" : "false") <<
      "\nreason: " << std::quoted(reason) << "\nresumable: " <<
      (operation_.phase() == Phase::Waiting ? "true" : "false") <<
      "\nmap_saved: " << (saved ? "true" : "false") <<
      "\nraw_frontier_cells: " << last_search_.frontier_cells <<
      "\nactionable_frontier_cells: " << last_search_.actionable_frontier_cells <<
      "\nunresolved_frontier_cells: " << last_search_.unresolved_frontier_cells <<
      "\nresidual_frontier_cells: " << last_search_.residual_frontier_cells <<
      "\nsimulation_time: " << now().seconds() << '\n';
  }

  void waitForImprovement(const std::string & reason)
  {
    if (!operation_.wait()) {return;}
    retry_candidates_ = false;
    waypoint_mode_ = false;
    candidate_search_started_ = -1.0;
    waiting_odom_valid_ = pose("odom", waiting_odom_);
    Pose actual;
    waiting_body_free_ = pose("map", actual) &&
      maze_explorer::footprintFree(*map_, actual.x, actual.y, padded_radius_);
    waiting_geometry_ = pose("map", actual) ? maze_explorer::findCandidates(
      *map_, actual.x, actual.y, padded_radius_, {}, goal_clearance_margin_) : maze_explorer::SearchResult{};
    last_search_ = waiting_geometry_;
    if (pose("map", actual)) {
      blocked_paths_.erase(std::remove_if(blocked_paths_.begin(), blocked_paths_.end(),
        [&](const auto & path) {
          return maze_explorer::checkPathFromPose(*map_, path, actual.x, actual.y, padded_radius_).safe;
        }), blocked_paths_.end());
    }
    waiting_checked_revision_ = map_revision_;
    writeStatus("stalled", false, reason, true);
    RCLCPP_WARN(get_logger(), "探索暂停（未完成，可恢复）：%s；保持监测新地图，不重复强行恢复",
      reason.c_str());
  }

  void finish(bool completed, const std::string & reason)
  {
    if (operation_.phase() == Phase::Finished) {return;}
    // No recovery is dispatched after this point; late goal responses are canceled by callbacks.
    if (operation_.phase() != Phase::Idle && operation_.phase() != Phase::Waiting) {sendCancel();}
    operation_.finish();
    timer_->cancel();
    writeStatus(completed ? "completed" : "stalled", completed, reason, true);
    RCLCPP_INFO(get_logger(), "%s：%s；地图保存结果见 %s.status.yaml",
      completed ? "探索完成" : "探索终止（未完成）", reason.c_str(), save_path_.c_str());
  }

  std::string save_path_;
  double padded_radius_, goal_clearance_margin_, map_timeout_, tf_timeout_;
  double action_timeout_, planning_timeout_, recovery_scan_timeout_;
  double candidate_search_timeout_, candidate_search_started_{-1.0};
  struct ScanSample {sensor_msgs::msg::LaserScan::SharedPtr scan; double received;};
  std::deque<ScanSample> scans_;
  double map_received_{-100.0}, op_started_{0.0}, cancel_started_{0.0};
  double last_tick_{-1.0}, last_decision_{-100.0}, no_candidate_since_{-1.0};
  double nav_start_area_{0.0}, progress_area_{0.0};
  std::string cancel_reason_, sensor_reason_;
  bool cancel_sent_{false}, retry_candidates_{false}, recovery_used_{false};
  bool waypoint_used_{false}, waypoint_mode_{false}, waiting_for_recovery_{false};
  bool waiting_odom_valid_{false}, waiting_body_free_{false};
  Kind kind_{Kind::Plan};
  int empty_maps_{0};
  int start_failures_{0};
  double empty_since_{-1.0};
  std::uint64_t waiting_checked_revision_{0};
  std::uint64_t map_revision_{0}, last_empty_revision_{0}, progress_map_revision_{0}, progress_epoch_{0};
  Pose nav_start_, robot_odom_, blocked_start_, waiting_odom_;
  maze_explorer::SearchResult last_search_, waiting_geometry_;
  std::vector<nav_msgs::msg::Path> blocked_paths_;
  Candidate candidate_{};
  std::vector<Candidate> candidates_;
  std::vector<Candidate> short_candidates_;
  std::size_t candidate_index_{0};
  maze_explorer::GoalExclusions remembered_;
  maze_explorer::GoalAttemptHistory attempted_goals_;
  std::vector<LifecycleCheck> lifecycle_;
  maze_explorer::OperationTracker operation_;
  maze_explorer::ProgressWatchdog watchdog_;
  maze_explorer::VisitedGoals visited_goals_;
  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp_action::Client<Navigate>::SharedPtr nav_client_;
  rclcpp_action::Client<ComputePath>::SharedPtr plan_client_;
  rclcpp_action::Client<BackUp>::SharedPtr backup_client_;
  rclcpp_action::Client<Spin>::SharedPtr spin_client_;
  Handle<Navigate>::SharedPtr nav_handle_;
  Handle<ComputePath>::SharedPtr plan_handle_;
  Handle<BackUp>::SharedPtr backup_handle_;
  Handle<Spin>::SharedPtr spin_handle_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExplorerNode>());
  rclcpp::shutdown();
  return 0;
}
