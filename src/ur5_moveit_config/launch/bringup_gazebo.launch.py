from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os

# process has died [pid 13572, exit code 255, cmd 
# 'gzserver -slibgazebo_ros_init.so -slibgazebo_ros_factory.so 
# -slibgazebo_ros_force_system.so'].
def generate_launch_description():
    # Set the path to the Gazebo ROS package
    gaz_pkg_share = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
  
    # Set the path to this package.
    desc_pkg_share = FindPackageShare(package='ur5_description').find('ur5_description')

    # Set the path to the world file
    world_file_name = 'empty.world'
    world_path = os.path.join(gaz_pkg_share, 'worlds', world_file_name)
    
    # Set the path to the SDF model files.
    gazebo_models_path = os.path.join(gaz_pkg_share, 'models')
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    
    # Set the path to the RViz configuration settings
    # default_rviz_config_path = os.path.join(pkg_share, 'rviz/rviz_basic_settings.rviz')
    
    # Set the path to the URDF file
    urdf_model_path = os.path.join(desc_pkg_share, 'urdf/rrr_arm.xacro')
  
    cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', world]
    gazebo_node = ExecuteProcess(cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'], output='screen')

    # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        condition=IfCondition(use_simulator),
        launch_arguments={'world': world}.items())

    # Start Gazebo client    
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))

    return LaunchDescription([
        gazebo_node,

        spawn_entity,
    ])