from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions
import os

  
def generate_launch_description():

    rosbot_description_dir = get_package_share_directory('rosbot_description')
    proyecto_rosbot_dir = get_package_share_directory('multi_robot_rosbot')
    urdf_path = os.path.join(rosbot_description_dir, 'urdf', 'rosbot.urdf')

    return LaunchDescription([

        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name="rviz2",
            arguments=['-d', proyecto_rosbot_dir+"/rviz/mi_configuracion.rviz"],
            output="log"
        ),
    ])
