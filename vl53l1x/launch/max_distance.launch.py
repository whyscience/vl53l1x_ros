from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vl53l1x',
            executable='vl53l1x_node',
            name='vl53l1x',
            parameters=[{
                'timing_budget': 0.14,
            }],
            output='screen',
            emulate_tty=True,
        ),
    ])