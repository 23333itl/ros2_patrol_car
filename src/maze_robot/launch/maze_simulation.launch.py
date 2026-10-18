"""迷宫 + 小车 + Nav2 + RViz 一键启动（教学演示 / 热身任务用）。

- 小车出生点取自迷宫 <name>_info.yaml 的 start 字段；
- Nav2 使用同名的 <name>_map.yaml 地图（map_server + AMCL 定位），
  AMCL 初始位姿自动对齐出生点；
- RViz 加载 bot_description 的导航配置（Fixed Frame = map）。

示例：
  ros2 launch maze_worlds view_maze.launch.py world:=/path/to/exam_01.world
  ros2 launch maze_worlds view_maze.launch.py nav2:=false rviz:=false  # 只看小车
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


def _rewrite_nav2_params(params_path, map_yaml, x, y, yaw, odom_topic):
    """按迷宫改写 Nav2 参数，返回改写后的临时 yaml 路径。

    launch_ros 会把字典参数写成扁平键（如 'initial_pose.x'）的临时文件，
    rcl 的 yaml 解析无法对应到嵌套结构，覆盖会静默失效；
    因此这里直接读写嵌套结构，保证 map_server 的地图和 AMCL 初始位姿生效。
    """
    with open(params_path, 'r', encoding='utf-8') as f:
        params = yaml.safe_load(f)
    params.setdefault('map_server', {}).setdefault('ros__parameters', {})[
        'yaml_filename'] = map_yaml
    amcl = params.setdefault('amcl', {}).setdefault('ros__parameters', {})
    amcl['set_initial_pose'] = True
    amcl['initial_pose'] = {'x': x, 'y': y, 'z': 0.0, 'yaw': yaw}
    # Nav2 统一使用 EKF 融合后的 /odometry/filtered；关闭 EKF 时退回原始里程计
    for node_name in ('bt_navigator', 'controller_server', 'velocity_smoother'):
        params.setdefault(node_name, {}).setdefault('ros__parameters', {})[
            'odom_topic'] = odom_topic
    tmp = tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False)
    yaml.safe_dump(params, tmp)
    tmp.close()
    return tmp.name


def _spawn_robot(context):
    """OpaqueFunction：等 launch 参数求值后，读 _info.yaml 决定出生点位姿。"""
    world_path = LaunchConfiguration('world').perform(context)
    robot_name = LaunchConfiguration('robot_name').perform(context)

    try:
        world_name, x, y, yaw = _read_maze_info(world_path)
        start_msg = f'小车出生点：x={x} y={y} yaw={yaw}，世界名：{world_name}'
    except Exception as exc:  # info 文件缺失或格式错误时退回默认出生点
        # 仍然要 spawn 小车，否则 Gazebo 里只有空世界，后续 TF/导航全部超时
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
                '-topic', 'robot_description',  # 从 /robot_description 话题读取 URDF
                '-x', str(x), '-y', str(y), '-z', '0.0', '-Y', str(yaw),
            ],
            output='screen',
        ),
    ]


def _ekf_node(context):
    """OpaqueFunction：按 robot_name 改写 EKF 的里程计话题，返回 EKF 节点。

    融合轮式里程计 + IMU，发布 /odometry/filtered 和 odom -> base_footprint TF，
    减小打滑造成的 odom 漂移（gz 侧 DiffDrive 插件已不再发 TF）。
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
             parameters=[tmp.name], output='screen'),
    ]


def _nav2_nodes(context):
    """OpaqueFunction：按 world 参数推导地图路径，组装 Nav2 节点组。"""
    world_path = LaunchConfiguration('world').perform(context)
    robot_name = LaunchConfiguration('robot_name').perform(context)
    nav2_params = LaunchConfiguration('nav2_params').perform(context)

    # 生成器的命名约定：<name>.world 旁边有 <name>_map.yaml（占据栅格地图）
    map_yaml = os.path.splitext(world_path)[0] + '_map.yaml'
    if not os.path.isfile(map_yaml):
        return [LogInfo(msg=f'找不到地图 {map_yaml}，不启动 Nav2（请先用生成器输出地图）')]

    try:
        _, x, y, yaw = _read_maze_info(world_path)
    except Exception:
        x, y, yaw = 0.4, 0.4, 0.0

    use_ekf = LaunchConfiguration('ekf').perform(context).lower() in ('1', 'true', 'yes')
    odom_topic = '/odometry/filtered' if use_ekf else f'/model/{robot_name}/odometry'

    # 直接在 Python 里改写参数文件：launch_ros 会把字典参数写成扁平键
    # （'initial_pose.x'）的临时 yaml，rcl 解析不到嵌套结构，覆盖会静默失效，
    # 所以这里手工深拷贝修改后整体写成新的临时 yaml 再下发。
    resolved_params = _rewrite_nav2_params(
        nav2_params, map_yaml, x, y, yaw, odom_topic)

    return [
        LogInfo(msg=(f'Nav2 地图：{map_yaml}，AMCL 初始位姿：({x}, {y}, {yaw})，'
                     f'里程计：{odom_topic}')),
        # ---- 定位：地图服务器 + AMCL ----
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[resolved_params], output='screen'),
        Node(package='nav2_amcl', executable='amcl', name='amcl',
             parameters=[resolved_params], output='screen'),
        # ---- 导航核心 ----
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             parameters=[resolved_params], output='screen'),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             parameters=[resolved_params], output='screen'),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
             parameters=[resolved_params], output='screen'),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             parameters=[resolved_params], output='screen'),
        # 速度平滑：cmd_vel -> cmd_vel_smoothed -> 小车的 cmd_vel 话题
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', parameters=[resolved_params],
             remappings=[('cmd_vel_smoothed', ['/model/', robot_name, '/cmd_vel'])],
             output='screen'),
        # ---- 生命周期管理：自动激活上面的节点 ----
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization',
             parameters=[{'autostart': True, 'node_names': ['map_server', 'amcl']}],
             output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation',
             parameters=[{'autostart': True,
                          'node_names': ['controller_server', 'planner_server',
                                         'behavior_server', 'bt_navigator',
                                         'velocity_smoother']}],
             output='screen'),
    ]


def generate_launch_description():
    world = LaunchConfiguration('world')
    robot_name = LaunchConfiguration('robot_name')
    home = os.environ['HOME']
    default_world_path = os.path.join(home, "maze_ws", "maze_output", "maze.world")

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world_path,
        description='迷宫SDF世界文件路径'
    )
    
    bot_share = get_package_share_directory('bot_description')
    maze_share=get_package_share_directory('maze_robot')

    # ==================== 1. 展开 xacro 并发布 robot_description ====================
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

    return LaunchDescription([
        # DeclareLaunchArgument(
        #     'world',
        #     default_value=PathJoinSubstitution(
        #         [FindPackageShare('maze_robot'), 'generated', 'maze.world']),
        #     description='要打开的 .world 文件路径'),
        DeclareLaunchArgument(
            'robot_name', default_value='my_robot',
            description='小车在仿真中的实体名（决定 /model/<名>/cmd_vel 等话题名）'),
        DeclareLaunchArgument(
            'nav2_params',
            default_value=os.path.join(maze_share, 'config', 'nav2_params.yaml'),
            description='Nav2 参数文件路径'),
        DeclareLaunchArgument(
            'ekf', default_value='true',
            description='是否启动 robot_localization EKF（融合里程计 + IMU，发布 odom TF）'),
        DeclareLaunchArgument(
            'ekf_params',
            default_value=os.path.join(maze_share, 'config', 'ekf.yaml'),
            description='EKF 参数文件路径'),
        DeclareLaunchArgument(
            'nav2', default_value='true',
            description='是否启动 Nav2 导航栈（map_server + AMCL + 导航核心）'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='是否启动 RViz'),
        DeclareLaunchArgument(
            'lidar_type', default_value='ray',
            description='雷达类型：ray=CPU 雷达（无 GPU 也能跑，默认），'
                        'gpu_lidar=GPU 雷达（需正常显卡驱动）'),
        DeclareLaunchArgument(
            'camera', default_value='false',
            description='是否启用深度相机（当前导航链路不用；需要 GPU/EGL 正常）'),
        DeclareLaunchArgument(
            'gz_gui', default_value='false',
            description='是否显示 Gazebo 图形界面（慢机器上可设 false 只跑物理仿真）'),
        # 注意：LIBGL_ALWAYS_SOFTWARE 只影响 Mesa GL，不能让 Ogre2 的 EGL
        # 设备枚举改用软件设备；独显驱动异常时真正可靠的兜底是
        # lidar_type:=ray camera:=false（默认值）。
        DeclareLaunchArgument(
            'software_gl', default_value='false',
            description='强制 Mesa 软件渲染（仅在没有可用 GPU 且必须用渲染传感器时尝试）'),
        world_arg,
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
        # gz_gui=true  → gz sim -r（server + 图形界面）
        # gz_gui=false → gz sim -r -s（仅 server，无界面，省 CPU）
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
        # 必须等 Gazebo Server 把世界加载完，留 5 秒余量
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
                # cmd_vel 只允许 ROS -> Gazebo；odometry 只允许 Gazebo -> ROS。
                # 原来的 @ 是双向桥，既多余又可能把回声写回仿真。
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

        # 相机默认关闭；只有 camera:=true 时才桥接图像话题，避免无谓拷贝
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
        # 必须在 Nav2 之前就绪，AMCL 依赖它发布的 odom -> base_footprint TF。
        OpaqueFunction(function=_ekf_node, condition=IfCondition(LaunchConfiguration('ekf'))),

        # ==================== 5. 延时启动 Nav2 ====================
        # 等 spawn 和桥接就绪后再拉起导航栈（地图加载本身不依赖仿真）
        TimerAction(
            period=8.0,
            actions=[OpaqueFunction(function=_nav2_nodes)],
            condition=IfCondition(LaunchConfiguration('nav2')),
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
