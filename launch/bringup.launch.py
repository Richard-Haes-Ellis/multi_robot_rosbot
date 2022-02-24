import launch
import launch_ros
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():


    rosbot_description = get_package_share_directory('multi_robot_rosbot')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rosbot_description, '/launch/gazebo.launch.py']),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rosbot_description, '/launch/spawn_rosbot_1.launch.py']),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rosbot_description, '/launch/spawn_rosbot_2.launch.py']),
        ),

        # Nos relaciona el frame odom de rosbot1 al rosbot2 para unificar el arbol de tf's
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            namespace='rosbot1',
            name='static_transform_publisher_map2rosbotmap',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'rosbot1/map'],
            # arguments=['0', '0', '0', '0', '0', '3.1415926535', 'map', 'rosbot1/odom'],
            parameters=[
        		rosbot_description + '/config/static_tf.yaml'
        	],
        ),

        # Nos relaciona el frame odom de rosbot1 al rosbot2 para unificar el arbol de tf's
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            namespace='rosbot2',
            name='static_transform_publisher_map2rosbotmap',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'rosbot2/map'],
            # arguments=['0', '0', '0', '0', '0', '3.1415926535', 'map', 'rosbot1/odom'],
            parameters=[
        		rosbot_description + '/config/static_tf.yaml'
        	],
        ),


        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rosbot_description, '/launch/mapping_rosbot_1.launch.py']),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rosbot_description, '/launch/mapping_rosbot_2.launch.py']),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rosbot_description, '/launch/ps4_sim.launch.py']),
        ),
        
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([rosbot_description, '/launch/navigation_rosbot_1.launch.py']),
        # ),
        
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([rosbot_description, '/launch/navigation_rosbot_2.launch.py']),
        # ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rosbot_description, '/launch/rviz2.launch.py']),
        ),
        
    ])