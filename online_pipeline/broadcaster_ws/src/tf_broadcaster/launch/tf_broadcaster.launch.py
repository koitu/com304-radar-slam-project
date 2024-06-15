from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf_broadcaster',
            executable='odom_broadcaster',
            name='broadcaster1',
            parameters=[
            ]
        ),
    ])
