# SLAM 建图偏移修改与验证记录（2026-10-30）

已加入两阶段 SLAM 候选、纯里程计对照、隔离回放和实跑评价工具。航向约束明显减小了旋转漂移，但完整验收尚未通过，因此启动默认仍为 `baseline`，没有自动采用候选。现有底盘、Nav2、EKF 和探索器代码保持本轮开始时的状态。

## 修改原因

- `slam_profile` 将候选与基线分开，便于比较和回退。`near_chains` 收紧邻近扫描链关联；`heading_prior` 进一步约束连续扫描匹配对融合航向的修正。实验不能支持“只缩小关联距离就能解决偏移”的判断。
- 回放只提供传感器、时钟、机器人静态 TF 和 `odom -> base_footprint`。在 TF 消息内部排除旧 `map -> odom`，避免旧 SLAM 结果污染比较；真值在回放结束后才读取评价，不输入定位。
- 新诊断记录扫描时间戳位姿、`map -> odom` 修正、扫描/地图残差、转弯误差及回访错位。`PoseWithCovarianceStamped` 与 `Odometry` 的字段区别已有专门回归测试。
- 栅格地图的短墙段不具备足够角分辨率：0.05 m 像素在 0.5 m 墙段上可产生约 5.7° 台阶角。方向统计使用可见长度至少 `resolution / tan(2°)` 的栅格墙段，并另外检查连续激光点的短墙方向。短段原始拟合值和全部位置残差仍保留，阈值没有放宽。

配置入口见 [slam_profiles.yaml](/ros_ws_pool/maze_ws/src/maze_explorer/config/slam_profiles.yaml)，复现方式见 [README](/ros_ws_pool/maze_ws/src/maze_explorer/README.md)。

## 环境与同源回放

使用当前迷宫：10 × 10，单元 0.8 m、墙厚 0.08 m，种子 **1980958899011398951**；ROS 2 Jazzy、slam_toolbox 2.8.5、Gazebo Sim 8。实际运行环境的 OpenGL 可用 NVIDIA RTX 5070 Ti；使用无 GUI 的 Ogre2 激光，深度相机关闭。首轮强制软件 EGL 退出等启动失败不计入有效试验。

第一份自主探索数据约 660 秒，基线在后段停滞，缺少有效回访，仅用于定位旋转误差：

| 配置 | 最大相对航向误差 | 最大固定对齐位置误差 |
|---|---:|---:|
| 基线 | 2.800° | 0.170 m |
| 第一阶段 | 2.800° | 0.170 m |
| 第二阶段 | 0.101° | 0.024 m |
| 关闭扫描匹配，仅诊断 | 0.009° | 0.033 m |

基线最早在仿真 `126.0 s` 超过 0.10 m，真值位置约 `(0.386, 5.006)`；航向最早在 `347.6 s` 超过 2°。这份数据的融合里程计最大航向误差约 0.013°，支持优先约束 SLAM 修正。

随后通过 Nav2 执行固定拐角往返，录制约 620 秒，包含完整往返及再次回访。四组最终回放使用完全相同的 **6198 帧激光**，均处理 **386 帧建图扫描**，消息过滤丢帧均为 **0**。

录制首尾两帧激光 `1.771 s`、`621.600 s` 不在已记录里程计 TF 的 `[1.780, 621.580] s` 覆盖范围，已在共同输入中明确排除并记录。原始 rosbag 未改变；没有根据匹配误差筛除数据。

| 配置 | 最大航向误差 | 最大位置误差 | 最大回访同墙错位 | 该往返场景几何验收 |
|---|---:|---:|---:|---|
| 基线 | 1.600° | 0.044 m | 0.014 m | 通过 |
| 第一阶段 | 2.549° | 0.100 m | 0.050 m | 未通过，平行墙方向差约 2.923° |
| 第二阶段 | 0.291° | 0.048 m | 0.038 m | 通过 |
| 关闭扫描匹配，仅诊断 | 0.004° | 0.080 m | 0.047 m | 通过，不作为导航默认 |

第二阶段在此场景的连续激光墙方向最大误差约 0.188°、平行墙方向差约 0.351°；栅格长墙平行方向差约 1.993°，地图边界 P95 残差约 0.050 m。没有异常修正跳变。

基线也能通过这条往返路线，说明效果依赖路径，不能用单条路线宣称普遍解决。第二阶段并非所有位置误差指标都优于基线。

![最终同源回放比较](/ros_ws_pool/maze_ws/build/mapping_validation_20261030/final_comparison.png)

## 三次自主探索回归

三次均使用同一世界、同一出生点、第二阶段配置，记录约 620 秒。运行中使用独立 ROS domain 和 Gazebo partition。

| 轮次 | 最大航向误差 | 最大位置误差 | 回访错位 | 成功目标 | 运行结果 |
|---|---:|---:|---:|---:|---|
| 1 | 0.055° | 0.041 m | 0.033 m | 22 | 持续推进；连续点平行墙方向差 2.977°，几何未通过 |
| 2 | 0.002° | 0.017 m | 无有效回访 | 3 | 103.5 s 标记停滞、未完成；没有误报探索完成 |
| 3 | 0.174° | 0.073 m | 0.075 m | 22 | 持续推进；连续点平行墙方向差 3.016°，几何未通过 |

轮次 1、3 末段 60 秒的位置变化范围分别约 0.976 m、0.733 m。轮次 2 停滞期间航向误差很小，现有证据不足以将这次停滞归因于旋转漂移。所有实跑均未出现超过 2° / 0.10 m 的单次 `map -> odom` 修正跳变。

**默认不替换。** 第二阶段解决了大部分已测旋转漂移，但局部墙形一致性和探索稳定性尚未全部满足要求。下一步应针对这些失败片段检查平移修正与前沿可达性；本轮没有据此重配 EKF 或放宽导航净空。

## 使用与证据

显式试用候选：

```bash
source /ros_ws_pool/maze_ws/install/setup.bash
ros2 launch maze_explorer explore_maze.launch.py \
  world:=/ros_ws_pool/maze_ws/maze_output/maze.world slam_profile:=heading_prior
```

不传 `slam_profile` 时仍使用基线；`odometry_only` 仅作诊断。

- 最终机器可读汇总：[validation_summary.json](/ros_ws_pool/maze_ws/build/mapping_validation_20261030/validation_summary.json)。`promote_default: false`。
- 共同输入与被排除扫描：[filter_report.json](/ros_ws_pool/maze_ws/build/mapping_validation_20261030/revisit_filtered_final/filter_report.json)。SHA-256 为 `1f2907352103337ea4546eb76c4ee41d164dc1269dff257ba3876d33419d8424`。
- 最终四组结果：[final_comparison_summary.json](/ros_ws_pool/maze_ws/build/mapping_validation_20261030/final_comparison_summary.json)。各组原始运行参数、位姿、地图及日志位于相应 `*_final_replay/`。
- 三次实跑评价位于 `build/mapping_validation_20261030/navigation_1/2/3/evaluation_final/`。原始数据和早期测量版本保留；本报告只采用 `*_final_replay` 和 `evaluation_final`。
- 构建成功；完整已有回归通过。最终测试汇总为 27 项、0 错误、0 失败；其中脚本测试内部新增的 11 项用例全部通过。数据回放与实跑验收结果独立于单元测试，不因测试通过而自动判定地图达标。

较大的 rosbag 和图像保存在工作区 `build/`，未加入源码；清理构建目录前应另行备份验证数据。
