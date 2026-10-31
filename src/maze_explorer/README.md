# 迷宫探索与建图诊断

默认使用用户实测准确的 `simulation_stable`，直接运行 `ros2 launch maze_explorer explore_maze.launch.py` 即可加载。早期候选的离线结果见 [2026-10-30 验证记录](docs/mapping_validation_20261030.md)，其中默认保留说明属于当时状态。

这轮修改统一了全局/局部地图的小车尺寸、膨胀和碰撞预测，探索终点只要求位置到达；前沿目标先检查站位净空和路径，再交给导航。导航长时间无平移时先取消目标，收到终止结果后才运行 Nav2 恢复动作。这样可以减少墙角不可达目标、无意义终点旋转，以及导航和恢复同时发速度的问题。

前沿搜索先用完整车身净空计算可通行连通区域，再用多源搜索把前沿关联到最近的安全站位。这样不会因为一条原始最短路径贴墙，就把整片前沿的目标全部回退到机器人脚下。普通候选全部规划失败时，保留的短目标仍会继续尝试。

建图偏斜仍需通过真实位姿逐级核对。`record_mapping_diagnostics.py` 只订阅和保存数据，不发布 `/cmd_vel`、导航目标或 TF，也不会替你启动、重置或暂停 Gazebo。

## 构建并确认参数

```bash
cd /home/itl/maze_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select bot_description maze_robot maze_explorer
source install/setup.bash
ros2 launch maze_explorer explore_maze.launch.py
```

另开终端，同样加载工作空间后检查实际运行参数，避免仍在使用旧安装目录或启动覆盖值：

```bash
ros2 param get /controller_server controller_frequency
ros2 param get /controller_server FollowPath.use_collision_detection
ros2 param get /global_costmap/global_costmap robot_radius
ros2 param get /local_costmap/local_costmap robot_radius
ros2 param get /global_costmap/global_costmap inflation_layer.inflation_radius
ros2 param get /local_costmap/local_costmap inflation_layer.inflation_radius
ros2 param get /controller_server exploration_goal_checker.plugin
```

基线应为控制频率 `10 Hz`、碰撞检测 `true`、两张地图半径均为 `0.13 m`、膨胀半径均为 `0.30 m`；探索模式的目标检查器应为 `nav2_controller::PositionGoalChecker`。精确的探索参数名以 `ros2 param list /maze_explorer` 输出和启动日志为准。

Smac 2D 的墙边通行代价使用有效参数 `GridBased.cost_travel_multiplier=3.0`，RPP 的速度前视时间设为 `1.0 s`，降低内角切角的风险。Nav2 给多边形顶点的 x/y 分别加 padding，因此探索器按 `radius + sqrt(2) × padding` 覆盖实际外接圆，该圆必须全部位于已知自由格。观察站位与障碍物另外保持 `0.05 m` 余量，抵御一格地图误差和激光噪声；额外余量的外环允许未知格，避免在盲角处不必要地后退。该余量只用于目标站位，不放大小车物理模型。

探索结果除 `.pgm/.yaml` 外会保存 `.status.yaml`，其中 `completed: false` 表示仍有前沿但停滞；不能将其当作完整地图。导航无进展先取消，终止确认后最多一次安全倒车和旋转；地图边缘增长本身不会无限重置恢复次数。

SLAM 由专用 `lifecycle_manager_slam` 通过服务配置、激活，`bond_timeout=0.0`。这样既避免依赖一次启动状态事件而漏激活，也不启用曾造成误重启的心跳监测。

## 一条命令记录

RViz 中模型抖动而 Gazebo 和建图正常时，先检查显示时间关系。用户反馈以激光为源的 `Approximate` 同步会使部件 TF 断续、模型频闪，故已撤回：`maze_robot/config/rviz.rviz` 恢复 Time 面板 `Synchronization=Off`（`SyncMode: 0`）、15 FPS 和原激光 QoS。不要仅凭车体与轮子的 TF 频率不同就启用同步；指定显示时刻可能超出轮子 TF 的最新时间，导致部件变换查询失败。提高发布上限也不能保证输入消息频率或完整时间覆盖。

重新启动 RViz 后生效；运行中也可在 Time 面板选择 `Synchronization=Off`，并关闭 Experimental。保持 Fixed Frame=`map` 便于观察与地图的配准。若整车仍突跳，可临时改为 `odom` 对照：仅在 `map` 下跳，优先查 `map -> odom` 修正；两者均跳则查 `odom -> base_footprint` 和 TF 发布来源。切换坐标系只是定位故障，不能消除实际定位修正。部件错位需要记录 `/joint_states` 与 TF 的时间戳、发布者及完整变换链后再修改；地图正常也不排除小幅 SLAM 修正。

先启动已有仿真，再在新终端记录。`--world` 必须是 SDF 的 `<world name>`，例如 `maze`；`--robot` 是 Gazebo 顶层模型名，默认 `my_robot`。`--output` 必须是尚不存在的目录，工具拒绝覆盖旧结果。

```bash
source /opt/ros/jazzy/setup.bash
source /home/itl/maze_ws/install/setup.bash
ros2 run maze_explorer record_mapping_diagnostics.py \
  --world maze --robot my_robot --scenario explore \
  --seed 9249201525648114958 --run 1 --duration 600 \
  --output /home/itl/maze_ws/diagnostics/seed9249201525648114958_run1
```

`--duration` 使用仿真秒：Gazebo 暂停时不会消耗记录时长。`--duration 0` 一直记录到 Ctrl+C；`--wall-timeout 900` 可额外设置包括暂停时间在内的真实时间上限。Ctrl+C 会先结束子进程并让 rosbag 写完，再保存 CSV/JSON。仿真时钟倒退时会终止并标记 `incomplete`，防止把重置前后的位姿混在一起。

工具依赖本机可用的 `gz topic --json-output`、ROS 2 Jazzy、`ros2 bag record` 及已有消息包。`--no-bag` 只保存摘要和 Gazebo 原始真值，不能用于完整回放激光；正式验收应保留默认 rosbag。

### 真值来源

当前世界生成器加载 `SceneBroadcaster`，因此 Gazebo 会提供 `/world/maze/dynamic_pose/info`，类型为 `gz.msgs.Pose_V`。该消息来自实体组件中的模型位姿，按模型名提取顶层 `my_robot` 后作为世界坐标真值。工具没有把 `/model/my_robot/odometry` 当真值，后者来自 DiffDrive 轮式里程计。

工具直接保存 `gz topic --echo --json-output` 的消息，保留模型名称与外层仿真时间戳。普通 `Pose_V -> TFMessage` 桥接转换不保留模型名称，而 SceneBroadcaster 的时间戳在外层 header，直接桥接后无法可靠选择机器人并对齐时间。真值不注入 `/tf`，因此不会与 EKF/SLAM 竞争 TF 发布权。

可先只读确认话题：

```bash
gz topic -i -t /world/maze/dynamic_pose/info
gz topic -e --json-output -n 1 -t /world/maze/dynamic_pose/info
```

如果一直等待真值，请检查世界名、模型名、SceneBroadcaster 是否加载，以及工具与仿真是否在相同 Gazebo transport partition 和 ROS domain。模型必须位于世界顶层且名称唯一；不把嵌套模型的局部位姿解释为世界位姿。

官方实现依据：[SceneBroadcaster](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/scene_broadcaster/SceneBroadcaster.cc)、[ROS/Gazebo TF 转换](https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_bridge/src/convert/tf2_msgs.cpp)和[Pose 转 TransformStamped](https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_bridge/src/convert/geometry_msgs.cpp)。

### 输出含义

| 文件 | 用途 |
|---|---|
| `rosbag/` | 激光、IMU、原始/融合里程计、TF、时钟，以及 `/map`、`/plan`、全局/局部代价地图、RPP 前视点和预测碰撞轨迹 |
| `ground_truth.jsonl` | SceneBroadcaster 原始 `Pose_V`，保留名称、实体 ID 和时间戳 |
| `poses.csv` | 真值、IMU 航向、轮式里程计、融合里程计与 SLAM `map -> base_footprint` 位姿 |
| `imu.csv`、`scan.csv` | IMU 角速度/加速度与激光频率、有效点数、扫描周期检查 |
| `timing.csv` | 每条流及每条 TF 的消息时间戳、接收仿真时间、年龄与单调接收时间 |
| `comparison.csv` | 以首次同步样本固定初始 SE(2) 对齐后的相对航向误差、平移误差 |
| `turns.csv` | 从真值自动检测大于 20° 的转弯，比较 IMU、轮式、融合和 SLAM 的转角 |
| `summary.json` | 缺失流、时间戳频率、平均/最大消息年龄、逐级最大偏差 |
| `metadata.json`、`events.csv`、`*.log` | 种子/轮次/场景、启动/重置/超时等记录事件及工具进程日志 |

比较使用时间插值，默认每侧样本距离不超过 `0.15 s`，超出就不计算该项，避免拿旧真值和新里程计比较。TF 查询使用 `map -> base_footprint` 的最新公共时间；在 TF 尚未连通时留空，不猜测位姿。IMU 姿态协方差首项为 `-1` 时不会把不存在的姿态作为航向；角速度仍会保存。

`summary.json` 的 `recorded` 仅表示所需数据流已出现，**不代表建图已经达标**。`incomplete` 会列出缺失流并让命令返回非零状态。静态 TF 的零时间戳不是延迟；SLAM 故意发布未来时间戳时，TF 年龄可以为负。通过 rosbag 回放时应使用隔离的 ROS domain，避免与正在运行的小车数据混合。

## 验证步骤与判断依据

先在可通行区域分别执行直行、左转、右转和四次直角转弯。工具只记录，运动由现有导航流程完成；人工测试时在启动命令加 `explore:=false`，保持单一运动指令来源。每个场景单独记录一份输出，将 `--scenario` 分别设为 `straight`、`left`、`right`、`four-turns`。连续转弯之间至少保留 `0.5 s` 的稳定直行或停止段，使自动分段能区分四次转弯。

根据误差首次出现的级别定位问题：真值与轮式里程计已偏离时，先检查底盘/接触和加速度；IMU 转角异常时，检查传感器时间、坐标系和姿态；融合结果比输入更偏时，再检查 EKF 配置；轮式、IMU 和融合正常但 `map -> base_footprint` 偏离时，重点检查激光时间与扫描匹配。这样可以避免没有依据地重配 EKF 或开启回环。

接着用完全相同的世界文件与种子，从同一出生位姿重新启动三次探索。每次在新启动的仿真中分别记录 `--run 1/2/3`，输出目录独立；`--seed` 只作记录，不会重新生成或重置世界。不要直接在运行中的 Gazebo 重置来拼接三次记录。

每轮检查墙角能否继续推进、30 秒无足够平移后是否取消目标、是否收到终止结果后才恢复，以及仍有前沿但停滞时是否明确保存为未完成。对每份最终地图和生成器真值地图做一次整体刚性对齐，检查平行墙方向差不超过 `2°`、回访同一墙的错位不超过 `0.10 m`。整张地图相对 RViz 网格旋转不作为失败项。该墙面验收需结合保存地图人工或离线分析，当前工具不会仅凭里程计误差宣称墙面达标。

当前仓库中的代码/参数检查与离线测试不能替代上述三次仿真；只有取得各轮 rosbag、`summary.json` 和保存地图后，才能报告运行质量验收结果。

## 建图偏移的分阶段对照

`slam_profile` 在 `slam_params` 基础上覆盖少量参数，启动日志会打印选中的配置。根据用户实测及明确要求，默认已切换为 `simulation_stable`，与显式传入 `slam_profile:=simulation_stable` 的参数完全一致。需要旧配置作对照时可传 `slam_profile:=baseline`。

| profile | 改变及原因 |
|---|---|
| `baseline` | 当前扫描匹配基线，用于同源比较 |
| `baseline_turn` | 仅在基线上启用 `check_min_dist_and_heading_precisely: true`，让原地转向按角度触发扫描 |
| `near_chains` | 使用传感器位姿判断邻近关系，关联半径 `0.30 m`、响应门槛 `0.70`，减少相似走廊的错误关联机会 |
| `heading_prior` | 在上项基础上，角度先验参数 `0.02`、惩罚下限 `0.50`，粗搜索 ±3° / 步长 0.5°，精搜索步长 0.05°，关闭自动扩大搜索；限制对已验证准确的融合航向的额外修正 |
| `heading_prior_turn` | 与 `heading_prior` 参数相同，仅额外启用精确距离/角度筛选；长时验收前显式试用 |
| `simulation_stable` | 默认配置，用户已实测建图准确。强化融合航向先验，保留顺序扫描匹配，以 `1.1` 门槛关闭无先验邻近链；不开回环，避免对称走廊误约束。历史离线回放尚未达到全部几何验收指标 |
| `odometry_only` | 关闭扫描匹配，仅用于判断漂移是否由扫描匹配引入，不作为导航默认值 |

`do_loop_closing: false` 不会关闭 Karto 的邻近扫描链匹配。`angle_variance_penalty` 在本版本内部会平方，调小会加强连续匹配的航向约束；邻近链匹配不使用该惩罚，因而还需要关联距离和响应门槛。`fine_search_angle_offset` 在精匹配实现中作为角度步长使用。实现依据：[slam_toolbox 2.8.5 参数](https://github.com/SteveMacenski/slam_toolbox/blob/2.8.5/src/slam_mapper.cpp)、[Karto 匹配与关联](https://github.com/SteveMacenski/slam_toolbox/blob/2.8.5/lib/karto_sdk/src/Mapper.cpp)。

例如显式选择第二阶段，其他底盘、Nav2、EKF 配置保持现有值：

```bash
ros2 launch maze_explorer explore_maze.launch.py \
  world:=/ros_ws_pool/maze_ws/maze_output/maze.world slam_profile:=heading_prior
ros2 param get /slam_toolbox angle_variance_penalty
ros2 param get /slam_toolbox link_scan_maximum_distance
```

录制至少 600 仿真秒，包含回访同一墙面后，使用一份数据依次比较四种配置。`--domain` 必须是未使用的本地 ROS domain，不能与运行中的仿真相同；工具在检测到已有节点时拒绝启动：

```bash
ros2 run maze_explorer replay_mapping.py compare \
  --recording /ros_ws_pool/maze_ws/diagnostics/seed9249201525648114958_run1 \
  --world /ros_ws_pool/maze_ws/maze_output/maze.world \
  --output /ros_ws_pool/maze_ws/diagnostics/slam_comparison \
  --domain 185 --rate 1.0
```

工具首先产生 `filtered/`：仅保留激光、IMU、原始/融合里程计、录制时钟、静态机器人 TF，以及动态 `odom -> base_footprint`。它在 `/tf` 消息内部剔除旧 `map -> odom`；不回放旧地图、位姿图、导航目标或速度。保留静态 TF 的 transient-local 属性，并在扫描开始前提供激光外参。不会启动 EKF、导航或 Gazebo，不会把评价真值发布到 ROS。

`filter_report.json` 记录实际保留的话题、剔除的 TF 数、仿真时长和输入消息摘要。检测到时钟重置、输出目录已经存在或没有必需的里程计 TF 时会拒绝执行。录制边界可能先收到激光、后收到里程计 TF；工具只排除时间戳落在已录制 TF 覆盖范围之外的边界扫描，并把每个被排除的时间戳写入报告，原始录制保持不变。四个子目录各保存实际运行参数、SLAM 日志、最终地图及：

| 输出 | 判断用途 |
|---|---|
| `slam_poses.csv`、`comparison.csv` | 使用扫描时间戳的 SLAM 位姿，与真值作一次固定初始 SE(2) 对齐；显示误差首次超过 2° / 0.10 m 的时刻 |
| `map_to_odom.csv` | 由同一时刻的 SLAM 位姿和里程计计算修正，避免将 SLAM 故意向未来发布的 TF 时间戳误作扫描时间 |
| `scan_map_residual.csv` | 约每秒一次，记录激光落点到当时已发布占据格的中位数、P95 和最大距离；地图尚未更新也可能产生残差，应结合真值判断 |
| `turns.csv` | 左右转弯前后的角度误差 |
| `wall_overlay.svg` | 固定对齐后的地图边界红点与真实墙面黑线 |
| `summary.json` | 地图墙面方向、残差、分次回访的同墙法向错位、异常修正跳变、丢帧和验收状态 |

墙面评价读取生成器 `maze_walls` 的实际箱体表面，考虑墙厚。连续激光点的方向拟合要求至少 0.40 m 可见墙段和 12 点，剔除墙端 0.10 m。栅格方向另要求可见长度至少 `resolution / tan(2°)`（0.05 m 分辨率时约 1.43 m），避免一个像素台阶被误判成数度倾斜；短段仍保留原始拟合值并标记方向未分辨，使用连续激光验证其方向。所有点仍计入残差与未匹配比例。回访要求同一墙面至少间隔 15 秒没有观测，且每次有足够墙段支持；缺少回访数据记为 `incomplete`，不会当作零误差通过。

几何验收要求单墙方向误差及同向平行墙之间的方向差均 ≤2°、回访错位 ≤0.10 m；栅格长墙与连续激光点的单墙/平行墙方向均需满足阈值，同时检查地图边界 P95 ≤0.10 m、未匹配比例 ≤1%，以及单次修正不跳变超过 2° / 0.10 m。少于 600 秒或发生消息过滤丢帧不能通过，可降低 `--rate` 后用新目录重试。`run` 返回 0 表示几何通过，1 表示未通过/数据不足，2 表示执行错误；`compare` 只有正式候选通过且四组数据完整才返回 0。自动推荐新候选前仍需同一种子三次长时导航回归；本次默认切换依据用户实测和明确要求，不代表历史离线指标已全部通过。当前 profile 默认面向已验证 IMU 航向准确的 Gazebo 仿真。

新版本在线记录器也增加 `map_to_odom.csv`、`scan_map_residual.csv` 和精确扫描时刻的 `slam_pose`，同时录制 `/pose` 与位姿图。在线 `map_to_odom.csv` 保留原 TF 时间戳及接收时刻，与离线按扫描时刻计算的文件含义有明确区别。

完成实跑后，也可以直接评价新版本记录器产生的 rosbag，无需再次启动 ROS 节点或回放。使用同一个墙面评价函数，避免对实跑和回放采用不同标准：

```bash
ros2 run maze_explorer evaluate_mapping_recording.py \
  --recording /ros_ws_pool/maze_ws/diagnostics/seed9249201525648114958_run1 \
  --world /ros_ws_pool/maze_ws/maze_output/maze.world \
  --output /ros_ws_pool/maze_ws/diagnostics/run1_evaluation \
  --launch-log /ros_ws_pool/maze_ws/diagnostics/run1_launch.log \
  --exploration-status /ros_ws_pool/maze_ws/diagnostics/run1_map.status.yaml
```

提供启动日志时，`navigation_acceptance` 另行检查成功目标、最终完成/停滞标记，以及末段 60 秒是否仍有至少 0.10 m 的位置变化。几何或导航任一未通过都返回非零；未提供日志时只评价几何。旧记录没有 `/pose` 时，不猜测扫描时刻位姿，应使用 `replay_mapping.py` 重建。

## 自动回归测试

```bash
source /opt/ros/jazzy/setup.bash
source /home/itl/maze_ws/install/setup.bash
cd /home/itl/maze_ws
colcon test --packages-select maze_explorer --event-handlers console_direct+
colcon test-result --test-result-base build/maze_explorer --verbose
```

测试包含前沿/状态机 C++ 用例、4 个真实 ROS Action 交互场景，以及新增的 11 个回放与诊断用例。回放测试使用实际 MCAP 消息验证 TF 内部过滤、拒绝时钟重置/覆盖、记录并排除缺少里程计 TF 的边界扫描、固定对齐不会掩盖漂移、跨 ±π 航向处理、回访与未匹配点统计，以及 SLAM 位姿消息与里程计消息的接口区别。真实转角地图快照只作为测试输入，运行中的探索器不会读取已知地图或 Gazebo 真值。Action 测试使用独立 ROS domain，验证取消接受后仍等待终止、恢复单飞、五张新地图才完成，以及短暂不确定地图会打断完成计数。ROS 测试需要本地 DDS 套接字权限。


## RViz 跳动与转弯扫描对照

Gazebo 中运动平稳而 RViz 跳动时，先比较同一时刻的 `odom -> base_footprint` 和 SLAM 修正，不能仅凭 RViz 的画面调整底盘摩擦。slam_toolbox 2.8.5 默认关闭 `check_min_dist_and_heading_precisely`，其前置距离筛选可能跳过原地转动的扫描；`minimum_travel_heading` 单独存在不能保证处理转向扫描。`*_turn` 候选保持距离 0.05 m、角度 0.03 rad、时间间隔 0.1 s，仅打开这个筛选开关。依据：[版本源码](https://github.com/SteveMacenski/slam_toolbox/blob/2.8.5/src/slam_toolbox_common.cpp#L702-L757)。

```bash
source /opt/ros/jazzy/setup.bash
source /ros_ws_pool/maze_ws/install/setup.bash
ros2 launch maze_explorer explore_maze.launch.py \
  world:=/ros_ws_pool/maze_ws/maze_output/maze.world slam_profile:=heading_prior_turn
ros2 param get /slam_toolbox check_min_dist_and_heading_precisely
```

默认现为用户实测的 `simulation_stable`，物理模型、EKF、TF 超时保持现有值。该配置输出栅格为 0.025 m，匹配搜索网格为 0.3 m / 0.01 m / 0.1 m；其余覆盖项见上表与 `slam_profiles.yaml`。

另开终端录制长时巡逻（1510 秒为 rosbag 订阅启动预留余量，验收仍要求实际记录至少 1500 秒）：

```bash
ros2 run maze_explorer record_mapping_diagnostics.py \
  --world maze --duration 1510 --seed 1980958899011398951 --run 1 \
  --output /ros_ws_pool/maze_ws/diagnostics/turn_run1
```

记录器额外保存 `/cmd_vel` 与 `/model/my_robot/cmd_vel`（进入 Gazebo 的平滑后命令）、`tf_samples.csv`、`tf_publishers.csv`、`tf_sources.json` 和 `map_timing.csv`。速度消息没有 header，CSV 明确采用接收仿真时间。TF 来源由消息的 publisher GID 与 ROS 发布端点对应，按每一条边统计；`/tf` 上有多个发布者本身不表示冲突。未发现对应端点时保留 GID，不猜测节点身份。

`correction_steps.csv` 由扫描时刻的 SLAM 位姿和同时间里程计推导修正 `C`，比较 `C_new * O(t)` 与 `C_old * O(t)`。不能直接相减 `map -> odom` 的平移分量，否则改变里程计原点就会改变跳变结论。缺少里程计插值边界时打断比较链，正式评价标记 incomplete。超过 0.10 m 或 2° 的更新在 `jump_events/` 保存前后各 5 秒的位姿图和原始时序、命令、TF、地图时间表；`correction_diagnostics.json` 保留所有事件、最大步长和缺失样本数。窗口超出录制边界时只保存已有数据。

`processed_scan_intervals.csv` 保存实际处理扫描的间隔；`turn_scan_coverage.csv` 记录里程计转角超过 20° 的区间及其中的扫描更新数。静止时跳过扫描是正常门限行为，不能把所有长扫描间隔都当作丢帧。地图 `map_walls` 和连续激光 `scan_walls` 分别评价：5 cm 栅格的短墙台阶不能直接当成航向漂移，也不能用栅格分辨率解释持续倾斜或重影。

`wall_direction_timeline.csv` 使用与最终连续墙面评价相同的点和跨度规则，增量拟合每面墙；`first_prefix_direction_violation` 及 `wall_fit_events/` 保留拟合增长过程中首次方向越界的位置和前后 5 秒片段。这可能是随后恢复的早期拟合，明确标为诊断线索；最终验收仍使用全部观测，不因此扩大或缩小阈值。

新一轮同源比较默认四组为 `baseline / baseline_turn / heading_prior / heading_prior_turn`：

```bash
ros2 run maze_explorer replay_mapping.py compare \
  --recording /绝对路径/recording --world /ros_ws_pool/maze_ws/maze_output/maze.world \
  --output /绝对路径/turn_comparison --domain 185 --rate 1
```

旧的邻近链及纯里程计诊断仍可用 `--compare-profiles baseline near_chains heading_prior odometry_only` 选择。正式巡逻回归每次记录至少 1500 仿真秒，同一迷宫种子重复三次；离线评价增加 `--min-duration 1500`，不能用默认的 600 秒回放门槛代替长时验收。每轮保存原始 bag、runtime 参数、导航日志和探索状态。只有转向扫描、几何质量及导航推进全部通过才替换默认；真值仅用于评价。
