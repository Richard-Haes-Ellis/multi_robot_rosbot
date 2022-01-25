from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    rosbot_description_dir = get_package_share_directory('rosbot_description')
    proyecto_rosbot_dir = get_package_share_directory('multi_robot_rosbot')


    return LaunchDescription([

        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true'
        ),

        # Nodo de slam para el robot1
        launch_ros.actions.Node(
        	parameters=[
                proyecto_rosbot_dir + '/config/slam_toolbox_sim.yaml'
        	],
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox_rosbot1',
    
            remappings=[
            ('/scan', '/rosbot1/rp_lidar/out'),
            ('/map', '/rosbot1/map'),
            ],
            output='screen'
        ),

    ])
