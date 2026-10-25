#include <chrono>
#include <functional>
#include <memory>
#include <cstdint>
#include <cmath>
#include <vector>
#include <array>
#include <algorithm>
#include <utility>
#include <queue>
#include <limits>
#include <fstream>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp" //用于控制小车自旋
#include "nav2_msgs/srv/clear_entire_costmap.hpp"


using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using namespace std::chrono_literals;

// 8 连通和 4 连通的方向数组
constexpr std::array<std::pair<int,int>,8> kDirs8 = {
  {{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}}
};
constexpr std::array<std::pair<int,int>,4> kDirs4 = {
  {{1,0},{-1,0},{0,1},{0,-1}}
};

//最小前沿距离限制 太小容易导致小车碎步抖动死锁 太大容易无法行动直接丢弃
constexpr double kMinFrontierGoalDistance = 0.4;
constexpr double kRecentGoalRadius = 0.4;
constexpr double kRecentGoalCooldownSeconds = 10.0;
constexpr double kFailedGoalRadius = 0.5;
constexpr double kFailedGoalCooldownSeconds = 30.0;
constexpr int kMaxConsecutiveFailures = 3;
constexpr int kFinishAfterUnreachable = 10;

// 把bfs找到的前沿格聚成簇，小于min_frontier_size丢弃
struct Frontier
{
  double goal_x=0.0;       // 导航目标点的 x (通常用质心)
  double goal_y=0.0;       // 导航目标点的 y
  double centroid_x=0.0;   // 簇的几何中心点 x
  double centroid_y=0.0;   // 簇的几何中心点 y
  double path_dist=0.0;    // 从机器人当前位置到该簇的实际可通行距离 (BFS距离)
};

struct FailedGoal
{
  double x=0.0;
  double y=0.0;
  rclcpp::Time time;
};

// 订阅slam的话题
// 主循环是一个定时器驱动的状态机： 1.等待nav2上线 2.空闲 3.正在导航 4.探索完成
// 看门狗来判断长时间没有任何可以探索的地方 -> 存图

// 可走区域和未知区域的交界 -> 前沿

class ExplorerNode : public rclcpp::Node {
public:
  ExplorerNode() : Node("maze_explorer") {
    clear_costmap_client_ = this->create_client<nav2_msgs::srv::ClearEntireCostmap>(
    "/global_costmap/clear_entirely_global_costmap");
    clear_local_costmap_client_ = this->create_client<nav2_msgs::srv::ClearEntireCostmap>(
    "/local_costmap/clear_entirely_local_costmap");
    finish_after_empty_=5;
    timer_ = create_wall_timer(2s,std::bind(&ExplorerNode::onTimer,this));
    // 订阅 /map 话题，使用 transient_local 确保能收到最新发布的静态地图
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>
    ("/map",rclcpp::QoS(1).transient_local().reliable(),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg){map_=msg;});
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this,"navigate_to_pose");
    //自旋用的速度发布者和定时器
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/model/my_robot/cmd_vel", 10);
    spin_timer_ = create_wall_timer(100ms, std::bind(&ExplorerNode::spinCallback, this));
    spin_timer_->cancel(); // 默认取消
    RCLCPP_INFO(get_logger(),"探索节点已启动");
  }

private:
  // 统计整张地图一共有多少个前沿格子（仅用于打印日志）
  int countFrontiers(const nav_msgs::msg::OccupancyGrid& m) const
  {
    const int w = m.info.width,h = m.info.height;
    int c=0;
    for(int x=0;x<w;x++)
    {
      for(int y=0;y<h;y++)
      {
        if(!isFree(m,x,y))
          continue;
        for(const auto &[dx,dy]:kDirs8)
        {
          const int nx=x+dx,ny=y+dy;
          if(nx<0||nx>=w||ny<0||ny>=h) continue;
          if(m.data[ny*w+nx]<0)
          {
            c++;
            break;
          }
        }
      }
    }
    return c;
  }
  //判断一个区域是否可走
  bool isFree(const nav_msgs::msg::OccupancyGrid& m,int x,int y) const
  {
    const int8_t v = m.data[y*m.info.width+x];
    // -1未知区域 0 可走区域 100障碍物
    return v>=0&&v<50;
  }

  bool isSafeGoalCell(const nav_msgs::msg::OccupancyGrid& m,int x,int y) const
  {
    if(!isFree(m,x,y))
      return false;

    const int w = static_cast<int>(m.info.width);
    const int h = static_cast<int>(m.info.height);
    for(int dx=-1;dx<=1;++dx)
    {
      for(int dy=-1;dy<=1;++dy)
      {
        const int nx=x+dx,ny=y+dy;
        if(nx<0||nx>=w||ny<0||ny>=h) continue;
        // 允许邻接未知格（前沿本身就是自由/未知交界），但不允许邻接障碍。
        if(m.data[ny*w+nx] >= 65) return false;
      }
    }
    return true;
  }

  bool isBlacklisted(double x,double y)
  {
    const auto now_time = now();
    for(auto it=failed_goals_.begin(); it!=failed_goals_.end();)
    {
      if((now_time-it->time).seconds() > kFailedGoalCooldownSeconds)
      {
        it = failed_goals_.erase(it);
        continue;
      }
      if(std::hypot(x-it->x,y-it->y) < kFailedGoalRadius)
        return true;
      ++it;
    }
    return false;
  }

  void recordFailure(double x,double y)
  {
    const auto now_time = now();
    for(auto &goal : failed_goals_)
    {
      if(std::hypot(x-goal.x,y-goal.y) < kFailedGoalRadius)
      {
        goal.time = now_time;
        return;
      }
    }
    failed_goals_.push_back({x,y,now_time});
  }

  void clearCostmaps()
  {
    if(clear_costmap_client_->service_is_ready())
    {
      auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
      clear_costmap_client_->async_send_request(request);
    }
    if(clear_local_costmap_client_->service_is_ready())
    {
      auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
      clear_local_costmap_client_->async_send_request(request);
    }
  }

  bool robotPose(double& x,double& y)
  {
    try
    {
      // 查 TF 获取机器人在 map 坐标系下的当前坐标
      const auto t = tf_buffer_->lookupTransform(
        "map","base_footprint",tf2::TimePointZero,200ms);
      x = t.transform.translation.x;
      y = t.transform.translation.y;
      return true;
    }
    catch(const tf2::TransformException& e)
    {
      return false;
    }
  }
  // 统计可走区域的面积（仅用于打印日志）
  int countFree(const nav_msgs::msg::OccupancyGrid& m) const
  {
    int c=0;
    for(const int8_t v : m.data)
     if(v>=0&&v<50) c++;
    return c;
  }
  // 发送导航目标给 Nav2
  void sendGoal(double x,double y)
  {
    if(!nav_client_->action_server_is_ready())
    {
      RCLCPP_WARN(get_logger(),"Nav2 未就绪");
      return;
    }
    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = now();
    goal.pose.pose.position.x = x;
    goal.pose.pose.position.y = y;
    goal.pose.pose.orientation.w = 1.0;

    auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    options.goal_response_callback =
      [this](const GoalHandleNav::SharedPtr &handle) {
        if (!handle) {
          RCLCPP_WARN(get_logger(), "目标被 Nav2 拒绝");
          state_ = State::kIdle;   // 不恢复就永远卡死
        }
      };
    options.result_callback =
      [this](const GoalHandleNav::WrappedResult& result)
      {
      if(result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(get_logger(),"到达目标");
            consecutive_failures_ = 0;
      }
      else {
        RCLCPP_WARN(get_logger(),"探索失败/被取消，将目标 (%.2f,%.2f) 拉黑 30 秒并清空代价地图",
          last_goal_x_,last_goal_y_);
        recordFailure(last_goal_x_,last_goal_y_);
        clearCostmaps();
        ++consecutive_failures_;
        if(consecutive_failures_ >= kMaxConsecutiveFailures)
        {
          finish("连续导航目标失败，剩余前沿可能不可达");
        }
      }
      state_ = State::kIdle;
      };
    state_ = State::kNavigating;
    last_goal_x_ = x;
    last_goal_y_ = y;
    last_goal_time_ = now();
    unreachable_streak_ = 0;
    nav_client_ ->async_send_goal(goal,options);
    RCLCPP_INFO(get_logger(),"探索（%.2f,%.2f）",x,y);
  }

  void finish(const std::string& reason)
  {
    RCLCPP_INFO(get_logger(),"探索结束：%s",reason.c_str());
    if(saveMap())
    {
      RCLCPP_INFO(get_logger(),"地图已保存：%s.pgm/.yaml",map_save_path_.c_str());
    }
    else
    {
      RCLCPP_ERROR(get_logger(),"地图保存失败");
    }
    timer_->cancel();
  }

  bool saveMap() const
  {
    if(!map_)
      return false;
    const auto& m=*map_;
    const int w = m.info.width,h = m.info.height;
    std::ofstream pgm(map_save_path_+".pgm",std::ios::binary);
    if(!pgm) return false;
    pgm << "P5\n" << w << " " << h << "\n255\n";
    // OccupancyGrid data[0] 是左下角；PGM 第一行是图像最上方 → 行倒序写!
    for (int y = h - 1; y >= 0; --y)
    {
      for (int x = 0; x < w; ++x) {
        const int8_t v = m.data[y * w + x];
        uint8_t pixel = (v < 0) ? 205 : (v >= 50 ? 0 : 254);  // 未知/占据/自由
        pgm.put(static_cast<char>(pixel));
      }
    }
    pgm.close();

    std::ofstream yaml(map_save_path_ + ".yaml");
    if (!yaml) return false;
    yaml << "image: explored_maze.pgm\n"          // 只写文件名,不写路径!
        << "resolution: " << m.info.resolution << "\n"
        << "origin: [" << m.info.origin.position.x << ", "
        << m.info.origin.position.y << ", 0.0]\n"
        << "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n";
    return true;
  }
  // 核心决策逻辑：找出所有前沿簇，选最近的走过去
  void decide()
  {
    if(!map_)
    {
      RCLCPP_INFO(get_logger(),"等待地图");
      return;
    }
    double rx=0.0,ry=0.0;
    if(!robotPose(rx,ry))
    {
      RCLCPP_WARN(get_logger(),"查不到TF");
      return;
    }
    RCLCPP_INFO(get_logger(),"地图：%ux%u 可走区域面积:%d 前沿格数:%d 机器人坐标（%.2f,%.2f）",
      map_->info.width,map_->info.height,countFree(*map_),countFrontiers(*map_),rx,ry);

    const auto &m = *map_;
    const double ox = m.info.origin.position.x;
    const double oy = m.info.origin.position.y;
    const double res = m.info.resolution;

    const auto clusters = findClusters(m,min_frontier_size_);
    if(clusters.empty())
    {
      empty_streak++;
      unreachable_streak_ = 0;
      RCLCPP_INFO(get_logger(),"没有任何前沿簇(连续%d/%d轮)",empty_streak,finish_after_empty_);
      if(empty_streak>=finish_after_empty_)
      {
        finish("地图没有任何未知前沿了");
      }
      return;
    }
    empty_streak = 0;

    int rcx = static_cast<int>((rx-ox)/res);
    int rcy = static_cast<int>((ry-oy)/res);
    const auto dist = bfsDistanceField(m,rcx,rcy);
    std::vector<Frontier> candidates;
    for(const auto& cluster : clusters)
    {
      Frontier f;
      double best_d = std::numeric_limits<double>::max();
      int best_x = -1;
      int best_y = -1;
      for(const auto &[x,y]:cluster)
      {
        if(!isSafeGoalCell(m,x,y))
          continue;
        const double d = static_cast<double>(dist[y*m.info.width+x]);
        if(!std::isfinite(d) || d >= best_d)
          continue;
        best_d = d;
        best_x = x;
        best_y = y;
      }
      if(best_x < 0 || !std::isfinite(best_d))
        continue;
      f.goal_x = ox+(best_x+0.5)*res;
      f.goal_y = oy+(best_y+0.5)*res;
      f.centroid_x = f.goal_x;
      f.centroid_y = f.goal_y;
      f.path_dist = best_d;
      candidates.push_back(f);
    }
    std::vector<Frontier> filtered;
    const auto current_time = now();
    for(const auto& f : candidates)
    {
      // 机器人已经站在这片前沿附近时，重复发送同一目标只会让 Nav2
      // 反复判定“到达目标”，造成原地打转。
      if(f.path_dist < kMinFrontierGoalDistance)
        continue;
      if(isBlacklisted(f.goal_x,f.goal_y))
        continue;
      const double since_last_goal = (current_time - last_goal_time_).seconds();
      if(std::hypot(f.goal_x - last_goal_x_, f.goal_y - last_goal_y_) <
           kRecentGoalRadius &&
         since_last_goal < kRecentGoalCooldownSeconds)
        continue;
      filtered.push_back(f);
    }
    if(filtered.empty())
    {
      if (unreachable_streak_ < 5) {
        RCLCPP_INFO(get_logger(), "没有可达前沿，触发倒车脱困 (%d/5)", unreachable_streak_ + 1);
        state_ = State::kSpinning; // 借用 spinning 状态，但在回调里执行倒车
        spin_ticks_ = 20; // 倒车 2 秒
        spin_timer_->reset();
        ++unreachable_streak_;
        return;
      }

      ++unreachable_streak_;
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "倒车后仍没有可达前沿（连续%d/%d轮），等待地图更新",
        unreachable_streak_,kFinishAfterUnreachable);
      if(unreachable_streak_ >= kFinishAfterUnreachable)
      {
        finish("连续多轮没有可达前沿");
      }
      return;
    }

    const auto best = *std::min_element(filtered.begin(),filtered.end(),
      [](const Frontier& a, const Frontier& b){return a.path_dist<b.path_dist;});

    sendGoal(best.goal_x,best.goal_y);
  }

  // 求小车到地图所有点的距离
  std::vector<float> bfsDistanceField(const nav_msgs::msg::OccupancyGrid& m,int sx,int sy) const
  {
    const int w = m.info.width,h = m.info.height;
    const float inf = std::numeric_limits<float>::infinity();
    std::vector<float> dist(w*h,inf);
    //如果起点被卡在墙里，在附近 3x3 或 5x5 区域内找一个新的起点
    if (!isFree(m, sx, sy)) {
        bool found = false;
        for (int r = 1; r <= 5 && !found; ++r) {
            for (int dx = -r; dx <= r && !found; ++dx) {
                for (int dy = -r; dy <= r && !found; ++dy) {
                    if (isFree(m, sx + dx, sy + dy)) {
                        sx += dx;
                        sy += dy;
                        found = true;
                        RCLCPP_INFO(get_logger(), "起点被占据，BFS 起点自动偏移至安全点");
                    }
                }
            }
        }
        if (!found) return dist; // 如果附近真全是墙，直接返回全 inf
    }
    std::queue<std::pair<int,int>> q;
    dist[sy*w+sx]=0.0f;
    q.emplace(sx,sy);
    while(!q.empty())
    {
      const auto [x,y] = q.front();
      q.pop();
      const float d=dist[y*w+x];
      for(const auto &[dx,dy] : kDirs4)
      {
        const int nx=x+dx,ny=y+dy;
        if(nx<0||nx>=w||ny<0||ny>=h) continue;
        if(dist[ny*w+nx]!=inf||!isFree(m,nx,ny))continue;
        dist[ny*w+nx] = d+m.info.resolution;
        q.emplace(nx,ny);
      }
    }
    return dist;
  }

  std::vector<std::vector<std::pair<int,int>>> findClusters(const nav_msgs::msg::OccupancyGrid& m,int min_size)const
  {
    const int w = m.info.width,h = m.info.height;
    std::vector<char> isfrontier(w*h,0);
    for(int x=0;x<w;x++)
    {
      for(int y=0;y<h;y++)
      {
        if(!isFree(m,x,y))
          continue;
        for(const auto &[dx,dy]:kDirs8)
        {
          const int nx=x+dx,ny=y+dy;
          if(nx<0||nx>=w||ny<0||ny>=h) continue;
          if(m.data[ny*w+nx]<0)
          {
            isfrontier[y*w+x]=1;
            break;
          }
        }
      }
    }

    std::vector<char> visited(w*h,0);
    std::vector<std::vector<std::pair<int,int>>> clusters;
    for(int x=0;x<w;x++)
    {
      for(int y=0;y<h;y++)
      {
        if(!isfrontier[y*w+x] || visited[y*w+x])
          continue;
        std::vector<std::pair<int,int>> cluster;
        std::queue<std::pair<int,int>> q;
        visited[y*w+x]=1;
        q.emplace(x,y);
        while(!q.empty())
        {
          const auto [cx,cy] = q.front();
          q.pop();
          cluster.emplace_back(cx,cy);
          for(const auto &[dx,dy]:kDirs8)
          {
            const int nx=cx+dx,ny=cy+dy;
            if(nx<0||nx>=w||ny<0||ny>=h) continue;
            if(!isfrontier[ny*w+nx]||visited[ny*w+nx]) continue;
            visited[ny*w+nx]=1;
            q.emplace(nx,ny);
          }
        }
        if(static_cast<int>(cluster.size())>=min_size)
        {
          clusters.push_back(cluster);
        }
      }
    }
    return clusters;
  }

  void onTimer()
  {
    // 只有在空闲状态才做决策
    if(state_ != State::kIdle) return;
    decide();
  }

  // 自旋回调函数(执行倒车)
  void spinCallback()
  {
    if (spin_ticks_ > 0) {
      auto cmd = geometry_msgs::msg::Twist();
      cmd.linear.x = -0.15; // 负数代表倒车，速度 0.15 m/s
      cmd.angular.z = 0.0;  // 不转动
      cmd_vel_pub_->publish(cmd);
      spin_ticks_--;
    } else {
      auto cmd = geometry_msgs::msg::Twist(); // 停止
      cmd_vel_pub_->publish(cmd);
      spin_timer_->cancel();
      state_ = State::kIdle;
      last_goal_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      RCLCPP_INFO(get_logger(), "倒车脱困结束，重新规划...");
    }
  }
private:
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_costmap_client_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_local_costmap_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

  //自旋相关变量
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr spin_timer_;
  int spin_ticks_ = 0;
  enum class State { kIdle, kNavigating, kSpinning };
  //enum class State { kIdle,kNavigating };
  State state_ = State::kIdle;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  std::vector<FailedGoal> failed_goals_;
  int min_frontier_size_ = 5;
  int empty_streak = 0;
  int unreachable_streak_ = 0;
  int consecutive_failures_ = 0;
  int finish_after_empty_;
  double last_goal_x_ = std::numeric_limits<double>::infinity();
  double last_goal_y_ = std::numeric_limits<double>::infinity();
  rclcpp::Time last_goal_time_{0, 0, RCL_ROS_TIME};
  std::string map_save_path_="explorer_maze";
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExplorerNode>());
  rclcpp::shutdown();
  return 0;
}
