from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vl53l1x',
            executable='vl53l1x_node',
            name='vl53l1x',
            parameters=[{
                'mode': 3,
                'i2c_bus': 1,
                'i2c_address': 41,  # 注意：0x29应该转换为十进制的41
                'poll_rate': 100.0,
                'ignore_range_status': False,
                'change_address': False,
                'timing_budget': 0.1,
                'offset': 0.0,
                'frame_id': '',
                'field_of_view': 0.471239,
                'min_range': 0.0,
                'max_range': 4.0,
            }],
            remappings=[],
            output='screen',
            emulate_tty=True,
        ),
    ])
