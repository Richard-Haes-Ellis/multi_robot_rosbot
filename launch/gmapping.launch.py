import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():


    slam_gmapping_launch_dir = get_package_share_directory('slam_gmapping')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')


    rosbot_description = get_package_share_directory('multi_robot_rosbot')
    return LaunchDescription([

        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_gmapping_launch_dir, "launch/slam_gmapping.launch.py")
        ),
        launch_arguments={
            "namespace": 'rosbot1',
            "use_sim_time": use_sim_time,
        }.items(),
        ),
    ])