from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions
import os

  
def generate_launch_description():

    proyecto_rosbot_dir = get_package_share_directory('multi_robot_rosbot')
    urdf_path = os.path.join(proyecto_rosbot_dir, 'urdf', 'rosbot.urdf')

    return LaunchDescription([

        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name="rviz2_1",
            arguments=['-d', proyecto_rosbot_dir+"/rviz/robot1.rviz"],
            remappings=[
            ('/goal_pose', '/rosbot1/goal_pose'),],
            output="log"
        ),
        
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name="rviz2_2",
            remappings=[
            ('/goal_pose', '/rosbot2/goal_pose'),],
            arguments=['-d', proyecto_rosbot_dir+"/rviz/robot2.rviz"],
            output="log"
        ),
    ])
