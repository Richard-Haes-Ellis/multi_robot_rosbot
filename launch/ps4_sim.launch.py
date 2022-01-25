import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description(): 
    
    node_params = os.path.join(
        get_package_share_directory('multi_robot_rosbot'),
        'config',
        'ps4.config.yaml'

    )

    return LaunchDescription([

        Node(
            package='joy_linux', 
            executable='joy_linux_node', 
            name='joy_linux_node',
            remappings=[
            ('/joy', '/joy_1'),
            ],
            parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
            }]),
        Node(
            package='teleop_twist_joy', 
            executable='teleop_node',
            name='teleop_twist_joy_node', 
            parameters=[node_params],
            remappings=[
            ('/joy', '/joy_1'),
            ('/cmd_vel', '/rosbot1/cmd_vel'),
            ],
            )
    ])