import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable

def generate_launch_description():
    pkg_share = get_package_share_directory('bot_description')
    world_file = os.path.join(pkg_share, "worlds", "my_world.sdf")
    gui_config = os.path.join(pkg_share, 'config', 'gui.config')

    # set_gui_config = SetEnvironmentVariable(
    #     name='GZ_GUI_CONFIG_PATH',
    #     value=gui_config
    # )

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', world_file],
        output='screen'
    )

    return LaunchDescription([
        #set_gui_config,
        gazebo
    ])
