from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

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
            name='slam_toolbox_rosbot2',
            # namespace='rosbot2', # No funciona en slam_toolbox
            remappings=[
            ('/scan', '/rosbot2/rp_lidar/out'),
            ('/map', '/rosbot2/map'),
            ('/slam_toolbox/feedback', '/rosbot2/slam_toolbox/feedback'),
            ('/slam_toolbox/update', '/rosbot2/slam_toolbox/update'),
            ('/slam_toolbox/scan_visualization', '/rosbot2/slam_toolbox/scan_visualization'),
            ('/slam_toolbox/graph_visualization', '/rosbot2/slam_toolbox/graph_visualization'),
            # ('/nav_msgs/msg/MapMetaData', '/rosbot2/nav_msgs/msg/MapMetaData'),
            ],
            output='screen'
        ),

    ])
