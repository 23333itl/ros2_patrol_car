import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
import launch_ros
from launch.substitutions import Command
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile




def generate_launch_description():
    # bot_description包：模型、world、地图
    pkg_bot = get_package_share_directory('bot_description')
    # my_cpp_pkg包：nav2参数 + rviz配置文件
    pkg_cpp = get_package_share_directory('my_cpp_pkg')

    # -------- Nav2 路径配置 --------
    map_yaml_path = os.path.join(pkg_bot, 'maps', 'my_world_map.yaml')
    nav2_params_path = os.path.join(pkg_cpp, 'config', 'nav2_params.yaml')
    rviz_config_path = os.path.join(pkg_cpp, 'config', 'rviz.rviz')

    # ==================== 1. 处理 XACRO ====================
    # 如果机器人模型是纯 urdf（没有 xacro），把下面改成 .urdf 即可
    xacro_file = os.path.join(pkg_bot, 'bot/bot.urdf.xacro')
    # 用 xacro 命令展开为纯 urdf 字符串
    robot_description = Command(['xacro ', xacro_file])

    # ==================== 2. 发布 Robot State ====================
    # robot_state_publisher 默认订阅 /joint_states，这里 remap 到 relay 修正后的 /joint_states_fixed
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        remappings=[('/joint_states', '/joint_states_fixed')],
        output='screen'
    )

    # ==================== 3. 启动 Gazebo Sim + World ====================
    world_file = pkg_bot + "/worlds/my_world.sdf"

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '4', world_file],
        output='screen'
    )

    # ==================== 4. Spawn 机器人到 Gazebo ====================
    # 用 ros_gz_sim 的 create 命令把 urdf 插入仿真
    # 注意：必须等 Gazebo Server 启动完毕，所以包一层 TimerAction
    spawn_robot = TimerAction(
        period=3.0,  # 延迟 3 秒，等 gz server 就绪
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', 'my_robot',        # 仿真中的实体名
                    '-topic', 'robot_description',  # 从 /robot_description 话题读取 urdf
                    '-x', '0.0',                # 初始位置 X
                    '-y', '0.0',                # 初始位置 Y
                    '-z', '0.0',                # 初始位置 Z
                    '-Y', '0.0',                # 初始偏航角（绕 Z 轴）
                ],
                output='screen'
            )
        ]
    )


    

     # ==================== 5. ROS-Gazebo 话题桥接 ====================
    # Gazebo Sim 的 DiffDrive 插件使用 Gazebo 内部话题：
    #   /model/my_robot/cmd_vel   (ROS -> Gazebo)
    #   /model/my_robot/odometry  (Gazebo -> ROS)
    #   /tf                       (Gazebo -> ROS，插件里已把 tf_topic 设为 /tf)
    #   /clock                    (Gazebo -> ROS，用于时间同步)
    #   /joint_states             (Gazebo -> ROS，轮子关节状态)
    # 注意：ros_gz_bridge 参数里 ROS 和 Gazebo 话题名必须一致；加 remappings 会导致
    # subscriber 创建失败，所以这里直接用相同的话题名
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/my_robot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/my_robot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            #'/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/depth_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/depth_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/depth_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',

        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    odom_tf_node = TimerAction(
    period=4.0,
    actions=[
        Node(
            package='bot_description',
            executable='odom_to_tf.py',
            name='odom_to_tf',
            parameters=[{'use_sim_time': True}],
            output='screen'
        )
    ]
    )

    # ==================== 6. joint_states 时间戳修正 ====================
    # bridge 转发的 /joint_states 是系统时间，relay 成 /joint_states_fixed 并改成仿真时间
    joint_state_relay = Node(
        package='bot_description',
        executable='joint_state_relay.py',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # ==================== 7. 全局使用仿真时间 ====================
    # Gazebo 里的 TF / odom 时间戳是仿真时间，ROS 节点必须开 use_sim_time 才能对上
    use_sim_time = SetParameter(name='use_sim_time', value=True)

    default_rviz_config_path=pkg_cpp+'/config/rviz.rviz'
    # RViz 节点
    rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    arguments=['-d', default_rviz_config_path],
    parameters=[{'use_sim_time': True}],
    output='screen'
    )

    # ==================== 7. Nav2 导航模块 ====================
    # ---- 定位组：map_server + amcl ----
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            ParameterFile(nav2_params_path),
            {'yaml_filename': map_yaml_path}
        ]
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[ParameterFile(nav2_params_path)]
    )

    lifecycle_loc = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'autostart': True},
            {'node_names': ['map_server', 'amcl']}
        ]
    )

    # ---- 导航组 ----
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        # 不再把 /cmd_vel 直接重映射到 Gazebo：控制器要先把速度交给 velocity_smoother
        # 做加速度限幅，再由 smoother 统一发到 /model/my_robot/cmd_vel
        parameters=[ParameterFile(nav2_params_path)]
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[ParameterFile(nav2_params_path)]
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[ParameterFile(nav2_params_path)]
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[ParameterFile(nav2_params_path)]
    )

    # velocity_smoother = Node(
    #     package='nav2_velocity_smoother',
    #     executable='velocity_smoother',
    #     name='velocity_smoother',
    #     output='screen',
    #     parameters=[ParameterFile(nav2_params_path)],
    #     remappings=[
    #         ('/cmd_vel_in', '/cmd_vel'),
    #         ('/cmd_vel_out', '/model/my_robot/cmd_vel'),
    #     ]
    # )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[ParameterFile(nav2_params_path)],
        # 该节点的话题名是 cmd_vel(输入) 和 cmd_vel_smoothed(输出)；
        # 只重映射输出： controller/behavior_server -> /cmd_vel -> smoother
        # -> /model/my_robot/cmd_vel -> Gazebo。
        # （之前写的 /cmd_vel_in、/cmd_vel_out 不存在，是空重映射，导致
        #   behavior_server 的 spin/backup 速度被 smoother 吞掉发不到车轮）
        remappings=[
            ('cmd_vel_smoothed', '/model/my_robot/cmd_vel'),
        ]
    )


    lifecycle_nav = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'autostart': True},
            {'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'velocity_smoother'
            ]}
        ]
    )

    loc_group = TimerAction(
        period=4.0,
        actions=[
            map_server,
            amcl,
            lifecycle_loc
        ]
    )

    # ========== 导航组延迟6s启动 ==========
    nav_group = TimerAction(
        period=6.0,
        actions=[
            controller_server,
            planner_server,
            behavior_server,
            bt_navigator,
            velocity_smoother,
            lifecycle_nav
        ]
    )


    return LaunchDescription([
        use_sim_time,
        robot_state_publisher,
        gazebo,
        spawn_robot,
        ros_gz_bridge,
        joint_state_relay,
        rviz_node,
        odom_tf_node,
        loc_group,
        nav_group,
    ])