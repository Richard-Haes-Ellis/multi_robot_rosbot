from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration , Command
import os
import launch
  
def generate_launch_description():

    # world_file_name =  'willow_garage.world'
    # world_file_name =  'solar2.world'
    # world_file_name =  'solar2.world'
    world_file_name =  'solar_simple.world'



    rosbot_description_dir = get_package_share_directory('rosbot_description')

    proyecto_rosbot_dir = get_package_share_directory('multi_robot_rosbot')


    gazebo_ros = get_package_share_directory('gazebo_ros')

    gazebo_client = launch.actions.IncludeLaunchDescription(
    launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
                condition=launch.conditions.IfCondition(launch.substitutions.LaunchConfiguration('gui'))
    )
    
    gazebo_server = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py'))
    )


    return LaunchDescription([

        # # Nos relaciona el frame odom de rosbot2 al rosbot2 para unificar el arbol de tf's
        # launch_ros.actions.Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     output='screen',
        #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'rosbot2_map'],
        #     parameters=[
        # 		rosbot_description_dir + '/config/static_tf.yaml'
        # 	],
        # ),

        # # Nos relaciona el frame odom de rosbot2 al rosbot2 para unificar el arbol de tf's
        # launch_ros.actions.Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     output='screen',
        #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'rosbot1_map'],
        #     parameters=[
        # 		rosbot_description_dir + '/config/static_tf.yaml'
        # 	],
        # ),

        # Nuestro mundo de mierda (Esta parametrixado mediante argumento, pero por defecto usa el willow garage)
        DeclareLaunchArgument(
            'world',
            default_value=[os.path.join(proyecto_rosbot_dir, 'worlds', world_file_name), ''],
            description='SDF world file'),
        
        DeclareLaunchArgument(
            name='gui',
            default_value='true'
        ),
        
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true'
        ),
        
        DeclareLaunchArgument('verbose', default_value='true',
                            description='Set "true" to increase messages written to terminal.'),
        
        DeclareLaunchArgument('gdb', default_value='false',
                            description='Set "true" to run gzserver with gdb'),
        
        DeclareLaunchArgument('state', default_value='true',
                                description='Set "false" not to load "libgazebo_ros_state.so"'),
        gazebo_server,
        gazebo_client,

    ])
