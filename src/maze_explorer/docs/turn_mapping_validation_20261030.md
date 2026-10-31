# RViz 跳动与转弯扫描修正（2026-10-30）

用户确认 Gazebo 中没有对应抖动，本轮因此保持物理模型、底盘、Nav2、EKF、TF 超时及地图分辨率不变。现有日志运行的是 baseline；日志没有保存完整传感器/真值，不能回溯认定截图跳变来自哪一条 TF。

## 修改及原因

- `baseline_turn` 与 `heading_prior_turn` 分别只在相应基线上新增 `check_min_dist_and_heading_precisely: true`。slam_toolbox 2.8.5 默认的前置距离筛选会跳过平移不足的扫描，角度阈值单独配置不能保证原地转动时处理扫描。候选用于验证这一机制，不预先宣称修复完成。
- 跳变量采用扫描时刻的新旧修正作用于同一里程计位姿的差异，即 `C_new O(t)` 与 `C_old O(t)`，不再直接相减 TF 平移。该指标不依赖 map/odom 原点；缺少里程计插值边界时不跨区间猜测。
- 记录器增加速度链路、TF 每条边的 publisher GID/端点身份、地图时间戳；扫描处理间隔与转向扫描覆盖单独保存。异常定位更新保存前后各 5 秒的 CSV 与图表。真值仍仅用于评价。
- 连续激光墙面和栅格墙面分开评价，保持原来的方向/回访/残差阈值，避免用栅格台阶掩盖持续偏移。

## 验证记录

候选默认未替换，正式启动仍为 `baseline`。

- 三个包构建成功；新增指标及消息兼容回归共 18 个 Python 用例通过。C++ 前沿/状态机与 ROS action 回归也通过；主机 `colcon test-result` 为 23 项、0 错误、0 失败（Python 内部 18 项单独报告）。
- 沙箱内 ROS action 测试因 UDP socket / getifaddrs 权限失败，已获放行在主机重跑；这类失败不作为小车定位证据。
- 初次隔离仿真因 Jazzy 回调发布信息为字典，记录器提前退出。已兼容字典与 MessageInfo 并补回归；失败尝试保留在 `build/turn_mapping_validation_20261030/attempt_1/`，不计为有效实跑。
- 用新指标复核历史 navigation_1 约 620 秒数据：最大单次修正位移 0.020 m、航向约 0.050°，无超过 0.10 m / 2° 的修正事件。这是历史数据复核，不是本轮 1500 秒验收。

本轮所有新结果保存在 `build/turn_mapping_validation_20261030/`：controlled 为静止、直行、左右转向和 Nav2 回访；navigation_1/2/3 为相同世界与种子的三次自主探索。运行参数、原始 rosbag、真值、导航日志和探索状态分别保留。当前世界 SHA-256 为 `b64d94d2c45f78cc113aabfc95068a8aee05e82cdd4bcbc85b20df31c5356136`，种子 `1980958899011398951`。

## 同源回放结果

受控录制包含静止、0.16 m 直行/倒回、左右原地转向、多个直角弯和同墙回访，Nav2 完成 39 个目标。四组回放使用相同的 **7191 帧激光**，均为 0 条 Message Filter 丢帧；边界扫描 `[1.88]` 不在已录制 odom TF 覆盖内，共同排除并保留原始数据。输入 SHA-256：`5d3862a53b5915a6f24f1bff084ac724585dbbee57615fa82b8ff649180239d7`。

| 配置 | 左/右原地转向处理扫描 | 最大航向误差 | 最大位置误差 | 栅格平行墙差 | 连续点平行墙差 | 几何验收 |
|---|---:|---:|---:|---:|---:|---|

| baseline | 0 / 0 | 2.999° | 0.157 m | 1.339° | 3.237° | fail |
| baseline_turn | 32 / 32 | 2.999° | 0.188 m | 2.691° | 1.592° | fail |
| heading_prior | 0 / 0 | 0.450° | 0.074 m | 0.745° | 0.999° | pass |
| heading_prior_turn | 32 / 32 | 0.201° | 0.058 m | 2.957° | 0.366° | fail |

这确认了转向门限的影响：未启用的两组在左右原地转动期间均未新增 SLAM 位姿，启用后均各 32 次。但仅启用开关不能消除累计偏航；与航向先验组合后连续墙线改善，栅格平行墙差仍不达标。单条受控路线通过的 heading_prior 也不能代表完整巡逻已通过。

基线最早在仿真 **42.4 s**、真值约 **(0.841, 0.392) m** 出现航向误差 **−2.400°**，当时位置误差约 0.014 m；baseline_turn 在 41.3 s 首次越过 2°。融合里程计保持准确，支持继续关注 SLAM 匹配修正。本次未复现超过 2° / 0.10 m 的单次定位修正，不能据此断言原截图的骤跳时刻已经复现。

首次增长的前后 5 秒数据与图：

- [baseline 42.4 s](/ros_ws_pool/maze_ws/build/turn_mapping_validation_20261030/comparison/baseline/trajectory_events/0001_42.400/poses.png)
- [baseline_turn 41.3 s](/ros_ws_pool/maze_ws/build/turn_mapping_validation_20261030/comparison/baseline_turn/trajectory_events/0001_41.300/poses.png)

图按各来源窗口内首样本归零，用于查看变化；绝对误差使用 comparison.csv 的固定初始刚性对齐。原始位姿、命令、TF 和地图时间表位于图的同目录。

## 三次长时巡逻

三轮均为 heading_prior_turn，同一世界及种子。首份 bag 的订阅启动会少录约 0.6 秒，因此在同一次连续仿真中补录了重叠尾段，再按时钟切片合并为 joined_recording。原始两段、切片时刻、实际时长及计数分别保存在各轮 join_report.json 和总 extension.json。模拟器在补录期间持续运行。

尾段处于停滞后的静止状态，没有新 /pose，尾段记录器因此保留 `incomplete: missing slam_pose`；这不是独立的成功巡逻。合并数据包含此前的扫描时刻位姿、连续里程计/真值，三轮评价的里程计插值缺失均为 0。

| 轮次 | 实际数据时长 | 最大航向误差 | 最大位置误差 | 连续平行墙差 | 回访错位 | 几何 | 导航 |
|---|---:|---:|---:|---:|---:|---|---|
| navigation_1 | 1520.232 s | 0.462° | 0.063 m | 2.154° | 0.080 m | fail | fail |
| navigation_2 | 1520.238 s | 0.307° | 0.052 m | 0.820° | 0.037 m | pass | fail |
| navigation_3 | 1520.219 s | 0.332° | 0.042 m | 0.506° | 0.009 m | pass | fail |

- navigation_1：成功 20 个目标，618.712 s 明确标记停滞、未完成。原因：安全恢复后导航仍无平移进展，保留地图并停止重复试探。
- navigation_2：成功 7 个目标，240.001 s 明确标记停滞、未完成。原因：仍有未知前沿，但安全站位/路径不可达；有界恢复后仍停滞。
- navigation_3：成功 11 个目标，488.508 s 明确标记停滞、未完成。原因：安全恢复后导航仍无平移进展，保留地图并停止重复试探。

navigation_1 的连续平行墙差最终为 2.154°；其逐步拟合最早在 **441.2 s**、真值约 **(1.656, 2.069) m** 越界。对应 [前后 5 秒片段](/ros_ws_pool/maze_ws/build/turn_mapping_validation_20261030/navigation_1/evaluation/wall_fit_events/0001_441.200/poses.png) 和 wall_direction_timeline.csv 已保存。navigation_3 曾在前缀拟合中短暂越界，但最终几何通过，表明前缀越界只能作为诊断线索。

三轮最大单次修正位移不超过 0.023 m、航向不超过 0.300°；map→odom 和 odom→base_footprint 在实际收到的 TF 中各只有一个发布来源。较小的定位误差没有保证探索持续推进，后续仍需针对停滞时的目标净空、路径和碰撞预测另行定位，不能将所有导航失败归因于偏航。

## 默认与复现

**完整验收未通过，promote_default=false，默认仍为 baseline。** 新候选修正了转向扫描进入处理的条件，并改善部分指标，但没有达到“地图几何 + 三次持续巡逻”全部要求；因此不把它当作通用修复，也不直接改变底盘、EKF、栅格分辨率或开启回环。

- [机器可读汇总](/ros_ws_pool/maze_ws/build/turn_mapping_validation_20261030/validation_summary.json)
- [四组对照图](/ros_ws_pool/maze_ws/build/turn_mapping_validation_20261030/comparison.png)
- 各轮 evaluation/summary.json、recording/、tail_recording/、joined_recording/ 和 runtime_params.json 保存评价、原始输入及生效参数。
- 构建、回归及实际参数已核对。原有未提交源码和用户地图/日志保留；新试验写入独立 build 目录。
- 录制、回放、实际参数检查及 1500 秒门槛的用法见 README。build 下数据较大，清理前应备份。任务内 run_trials.py、drive_route.py、extend_trials.py、join_recordings.py 和 evaluate_trials.py 保留了本次执行过程。
