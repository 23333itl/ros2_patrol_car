"""未知迷宫自主探索 + 在线建图一键启动。

与 view_maze.launch.py（已知地图 + AMCL）的区别：
- 不启动 map_server / AMCL，改用 slam_toolbox 在线建图（map -> odom 由它发布）；
- 新增 maze_explorer 节点做 frontier-based 自主探索，探索完成自动存图；
- EKF（轮式里程计 + IMU 融合）照常工作，odom -> base_footprint 由它发布；
- global_costmap 的 static_layer 改为订阅 slam_toolbox 实时发布的 /map。

示例：
  ros2 launch maze_explorer explore_maze.launch.py world:=/path/to/maze.world
  ros2 launch maze_explorer explore_maze.launch.py map_save_path:=/tmp/my_map
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def _read_maze_info(world_path):
    """读取 <name>_info.yaml，返回 (世界名, x, y, yaw)；失败时用默认值。"""
    info_path = os.path.splitext(world_path)[0] + '_info.yaml'
    world_name = os.path.splitext(os.path.basename(world_path))[0]
    x, y, yaw = 0.4, 0.4, 0.0
    with open(info_path, 'r', encoding='utf-8') as f:
        info = yaml.safe_load(f)
    world_name = info.get('name', world_name)
    start = info.get('start', {})
    x = float(start.get('x', x))
    y = float(start.get('y', y))
    yaw = float(start.get('yaw', yaw))
    return world_name, x, y, yaw


def _spawn_robot(context):
    """OpaqueFunction：等 launch 参数求值后，读 _info.yaml 决定出生点位姿。"""
    world_path = LaunchConfiguration('world').perform(context)
    robot_name = LaunchConfiguration('robot_name').perform(context)

    try:
        world_name, x, y, yaw = _read_maze_info(world_path)
        start_msg = f'小车出生点：x={x} y={y} yaw={yaw}，世界名：{world_name}'
    except Exception as exc:
        world_name = os.path.splitext(os.path.basename(world_path))[0]
        x, y, yaw = 0.4, 0.4, 0.0
        start_msg = (f'读取迷宫 _info.yaml 失败（{exc}），'
                     f'使用默认出生点 (0.4, 0.4, 0.0)，世界名：{world_name}')

    return [
        LogInfo(msg=start_msg),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-world', world_name,
                '-name', robot_name,
                '-topic', 'robot_description',
                '-x', str(x), '-y', str(y), '-z', '0.0', '-Y', str(yaw),
            ],
            output='screen',
        ),
    ]


def _ekf_node(context):
    """按 robot_name 改写 EKF 的里程计话题，返回 EKF 节点。

    融合轮式里程计 + IMU，发布 /odometry/filtered 和 odom -> base_footprint TF，
    减小打滑造成的 odom 漂移（gz 侧 DiffDrive 插件不发 TF）。
    """
    robot_name = LaunchConfiguration('robot_name').perform(context)
    ekf_params = LaunchConfiguration('ekf_params').perform(context)

    with open(ekf_params, 'r', encoding='utf-8') as f:
        params = yaml.safe_load(f)
    ekf = params.setdefault('ekf_filter_node', {}).setdefault('ros__parameters', {})
    ekf['odom0'] = f'/model/{robot_name}/odometry'
    tmp = tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False)
    yaml.safe_dump(params, tmp)
    tmp.close()

    return [
        LogInfo(msg=f'EKF 融合里程计话题：{ekf["odom0"]} + /imu'),
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
             parameters=[tmp.name,{'use_sim_time':True}], output='screen'),
    ]


def _slam_nav2_nodes(context):
    """OpaqueFunction：组装 slam_toolbox + Nav2（无 map_server/AMCL）+ 探索节点。

    在 bot_description 的 nav2_params.yaml 基础上改写：
    global_costmap 的 static_layer 订阅 slam_toolbox 实时发布的 /map。
    """
    robot_name = LaunchConfiguration('robot_name').perform(context)
    nav2_params = LaunchConfiguration('nav2_params').perform(context)
    slam_params = LaunchConfiguration('slam_params').perform(context)
    map_save_path = LaunchConfiguration('map_save_path').perform(context)
    use_ekf = LaunchConfiguration('ekf').perform(context).lower() in ('1', 'true', 'yes')
    odom_topic = '/odometry/filtered' if use_ekf else f'/model/{robot_name}/odometry'

    with open(nav2_params, 'r', encoding='utf-8') as f:
        params = yaml.safe_load(f)
    # static_layer 默认订阅 "map"，显式写成 /map 并确认 transient_local
    # （slam_toolbox 以 transient_local + reliable 发布 /map）
    static_layer = params.setdefault('global_costmap', {}).setdefault(
        'global_costmap', {}).setdefault('ros__parameters', {}).setdefault(
        'static_layer', {})
    static_layer['map_topic'] = '/map'
    static_layer['map_subscribe_transient_local'] = True
    # SLAM 模式下地图不断长大，而静态 global_costmap 的尺寸在启动时固定，
    # 前沿目标很容易落到 costmap 边界外，报 "Goal was outside bounds" 而失败；
    # 改成以机器人为中心的滚动窗口，边界永远跟着机器人走。
    gc = params['global_costmap']['global_costmap']['ros__parameters']
    gc['rolling_window'] = True
    gc['width'] = 16   # 覆盖激光 8m 量程，前沿目标一定在窗口内
    gc['height'] = 16
    # 探索模式降速：转快了相邻两帧旋转角太大，扫描匹配容易糊
    vs = params.setdefault('velocity_smoother', {}).setdefault('ros__parameters', {})
    vs['max_velocity'] = [0.4, 0.0, 0.8]
    vs['min_velocity'] = [-0.4, 0.0, -0.8]
    vs['odom_topic'] = odom_topic
    for node_name in ('bt_navigator', 'controller_server'):
        params.setdefault(node_name, {}).setdefault('ros__parameters', {})[
            'odom_topic'] = odom_topic
    mppi = params.setdefault('controller_server', {}).setdefault(
        'ros__parameters', {}).setdefault('FollowPath', {})
    mppi['vx_max'] = 0.4
    mppi['wz_max'] = 0.8
    tmp = tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False)
    yaml.safe_dump(params, tmp)
    tmp.close()
    resolved_params = tmp.name

    return [
        LogInfo(msg=f'探索完成后的地图保存路径：{map_save_path}.pgm/.yaml'),
        # ---- 在线建图：slam_toolbox（生命周期节点，由 lifecycle_manager_slam 激活）----
        Node(package='slam_toolbox', executable='async_slam_toolbox_node',
             name='slam_toolbox', parameters=[slam_params], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_slam',
             parameters=[{'autostart': True,
                          'node_names': ['slam_toolbox'],
                          'bond_timeout': 10.0}],
             output='screen'),
        # ---- 导航核心（无 map_server / AMCL）----
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             parameters=[resolved_params], output='screen'),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             parameters=[resolved_params], output='screen'),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
             parameters=[resolved_params], output='screen'),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             parameters=[resolved_params], output='screen'),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', parameters=[resolved_params],
             remappings=[('cmd_vel_smoothed', ['/model/', robot_name, '/cmd_vel'])],
             output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation',
             parameters=[{'autostart': True,
                          'node_names': ['controller_server', 'planner_server',
                                         'behavior_server', 'bt_navigator',
                                         'velocity_smoother']}],
             output='screen'),
        # ---- 自主探索节点（内部会等 Nav2 action server 就绪再开始）----
        Node(package='maze_explorer', executable='explorer_node', name='maze_explorer',
             parameters=[{'map_save_path': map_save_path,
                          # 卡死自恢复（原地旋转）需要直接发速度指令
                          'cmd_vel_topic': f'/model/{robot_name}/cmd_vel'}],
             output='screen'),
    ]


def generate_launch_description():
    world = LaunchConfiguration('world')
    robot_name = LaunchConfiguration('robot_name')

    bot_share = get_package_share_directory('bot_description')
    maze_share=get_package_share_directory('maze_robot')
    explorer_share = get_package_share_directory('maze_explorer')

    robot_description = Command([
        'xacro ', os.path.join(bot_share, 'bot', 'bot.urdf.xacro'),
        ' lidar_type:=', LaunchConfiguration('lidar_type'),
        ' enable_camera:=', LaunchConfiguration('camera'),
    ])
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )

    home = os.environ['HOME']
    default_world_path = os.path.join(home, "maze_ws", "maze_output", "maze.world")
    world_arg = DeclareLaunchArgument(
            'world',
            default_value=default_world_path,
            description='迷宫SDF世界文件路径'
    )

    return LaunchDescription([
        # DeclareLaunchArgument(
        #     'world',
        #     default_value=PathJoinSubstitution(
        #         [FindPackageShare('maze_robot'), 'generated', 'maze.world']),
        #     description='要打开的 .world 文件路径'),
        world_arg,
        DeclareLaunchArgument(
            'robot_name', default_value='my_robot',
            description='小车在仿真中的实体名（决定 /model/<名>/cmd_vel 等话题名）'),
        DeclareLaunchArgument(
            'nav2_params',
            default_value=os.path.join(maze_share, 'config', 'nav2_params.yaml'),
            description='Nav2 参数文件路径（在 launch 内改写成 SLAM 模式）'),
        DeclareLaunchArgument(
            'slam_params',
            default_value=os.path.join(explorer_share, 'config', 'slam_toolbox.yaml'),
            description='slam_toolbox 参数文件路径'),
        DeclareLaunchArgument(
            'map_save_path', default_value='explored_maze',
            description='探索完成后的地图保存路径（不含扩展名）'),
        DeclareLaunchArgument(
            'ekf', default_value='true',
            description='是否启动 robot_localization EKF（融合里程计 + IMU，发布 odom TF）'),
        DeclareLaunchArgument(
            'ekf_params',
            default_value=os.path.join(maze_share, 'config', 'ekf.yaml'),
            description='EKF 参数文件路径'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='是否启动 RViz'),
        DeclareLaunchArgument(
            'lidar_type', default_value='gpu_lidar',
            description='雷达类型：gpu_lidar=Gazebo Sim 8 可用雷达（默认）；'
                        'ray 不受支持，仅保留参数以兼容旧命令'),
        DeclareLaunchArgument(
            'camera', default_value='false',
            description='是否启用深度相机（当前导航链路不用；需要 GPU/EGL 正常）'),
        DeclareLaunchArgument(
            'gz_gui', default_value='false',
            description='是否显示 Gazebo 图形界面（慢机器上可设 false 只跑物理仿真）'),
        # 独显驱动异常时不要依赖 LIBGL_ALWAYS_SOFTWARE：Ogre2 会按 EGL 设备
        # 直接选硬件，软件渲染开关可能被拒绝。默认 gpu_lidar + camera:=false，
        # 只有 Ogre2 无法初始化时才尝试 software_gl:=true。
        DeclareLaunchArgument(
            'software_gl', default_value='false',
            description='强制 Mesa 软件渲染（仅在没有可用 GPU 且必须用渲染传感器时尝试）'),
        SetEnvironmentVariable(
            'LIBGL_ALWAYS_SOFTWARE', '1',
            condition=IfCondition(LaunchConfiguration('software_gl'))),
        SetEnvironmentVariable(
            'LIBGL_ALWAYS_SOFTWARE', '0',
            condition=UnlessCondition(LaunchConfiguration('software_gl'))),

        # 全局使用仿真时间：Gazebo 里的 TF / odom 时间戳都是仿真时间
        SetParameter(name='use_sim_time', value=True),

        robot_state_publisher,

        # ==================== 2. 启动 Gazebo + 迷宫世界 ====================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
            launch_arguments={'gz_args': [world, ' -r']}.items(),
            condition=IfCondition(LaunchConfiguration('gz_gui'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
            launch_arguments={'gz_args': [world, ' -r -s --headless-rendering']}.items(),
            condition=UnlessCondition(LaunchConfiguration('gz_gui'))),

        # ==================== 3. 延时 spawn 小车 ====================
        TimerAction(period=5.0, actions=[OpaqueFunction(function=_spawn_robot)]),

        # ==================== 4. ROS-Gazebo 话题桥接 ====================
        # 话题名与 bot_description 的插件配置一一对应；
        # 注意 ros_gz_bridge 的 ROS 与 Gazebo 话题名必须一致，不能再 remap。
        # 注意：/tf 不做桥接——odom -> base_footprint 由 EKF 节点在 ROS 侧发布，
        # 传感器坐标系的 TF 由 robot_state_publisher 发布，gz 侧没有需要桥接的 TF。
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                ['/model/', robot_name, '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'],
                ['/model/', robot_name, '/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            ],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/depth_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/depth_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                '/depth_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            ],
            parameters=[{'use_sim_time': True}],
            output='screen',
            condition=IfCondition(LaunchConfiguration('camera')),
        ),

        # ==================== 4.5 EKF：里程计 + IMU 融合 ====================
        # 立即启动（不用等 spawn）：robot_localization 会等话题出现后再开始滤波；
        # 必须在 slam_toolbox 之前就绪，SLAM 依赖它发布的 odom -> base_footprint TF。
        OpaqueFunction(function=_ekf_node, condition=IfCondition(LaunchConfiguration('ekf'))),

        # ==================== 5. 延时启动 SLAM + Nav2 + 探索节点 ====================
        # 等 spawn 和桥接就绪后再拉起导航栈
        TimerAction(
            period=20.0,
            actions=[OpaqueFunction(function=_slam_nav2_nodes)],
        ),

        # ==================== 6. RViz ====================
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(maze_share, 'config', 'rviz.rviz')],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
