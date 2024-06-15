import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    rviz_arg = DeclareLaunchArgument('rviz', default_value='true', description='Enable RViz')

    # include IWR6843.py
    package_dir = get_package_share_directory('ti_mmwave_rospkg')
    iwr6843_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package_dir,'launch','IWR6843.py')),
        launch_arguments={
            "cfg_file": '6843ISK_Tracking.cfg',
            "command_port": "/dev/ttyUSB0",
            "data_port": "/dev/ttyUSB1",
            "rviz": LaunchConfiguration('rviz'),
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(rviz_arg)
    ld.add_action(iwr6843_include)

    return ld
