from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration , Command
import os
import launch
  
def generate_launch_description():

    proyecto_rosbot_dir = get_package_share_directory('multi_robot_rosbot')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    urdf_path = os.path.join(proyecto_rosbot_dir, 'urdf', 'rosbot.urdf')

    return LaunchDescription([


        # Nos relaciona el frame odom de rosbot1 al rosbot2 para unificar el arbol de tf's
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            namespace='rosbot1',
            name='static_transform_publisher_map2odom',
            output='screen',
            arguments=['0', '0', '0', '3.1415926535', '0', '0', 'rosbot1/map', 'rosbot1/odom'],
            # arguments=['0', '0', '0', '0', '0', '3.1415926535', 'map', 'rosbot1/odom'],
            parameters=[
        		proyecto_rosbot_dir + '/config/static_tf_sim.yaml'
        	],
        ),

        # Nodo para spawnear rosbot1 en gazebo
        launch_ros.actions.Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            
            # Importante el flag de robot_namespace: Nos renombra los topicos publicados con ese nombre delante, 
            #   las entidades da igual pero se tienen que nombrar diferente en cada robot

            # Por ultimo le pasamos el .sdf del rosbot1: Este archivo es muy parecido al urdf pero es mas para GAZEBO
            #   Tiene definido plugins para los sensores y para la configuracion de los motores (imu, laser, astra, skid_steer_drive)
            #   Lo unico especifico a este archivo es que esta renombrado el base_link y los frames de los 4 motores
            #   ya que esas son las que se usan para el calculo de la odometria y el plugin (mierda de plugin la verdad) no renombra los frames
            #   con el tag <robot_namespace>
            
            arguments=['-spawn_service_timeout', '60','-robot_namespace','rosbot1', '-entity', 'rosbot1', '-x', '0', '-y', '0', '-z', '0.03', '-file', proyecto_rosbot_dir + '/models/rosbot1.sdf']),

        # Nodo que publica las transformadas estáticas de todo el robot
        # Es decir, nos define las tf's desde el base_link a todos nuestro elementos ESTATICOS (imu, laser, astra, sensores etc..)
        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='rosbot1',    # Creo recordar que no hace absolutamente nada :)
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                'frame_prefix':'rosbot1/',  # MARAVIALLA DEL SIGLO, nos renombra los tfs con este prefijo
                # 'robot_description':Command(['xacro',' ', xacro_path, ' robot_namespace:=','rosbot1']) # Pasaaaaaaaaaando
                }],
            arguments=[urdf_path]),

    ])
