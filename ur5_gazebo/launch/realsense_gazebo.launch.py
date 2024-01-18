#!/usr/bin/env python3
'''
[gzserver-1] gzserver: /usr/include/boost/smart_ptr/shared_ptr.hpp:728: typename boost::detail::sp_member_access<T>::type boost::shared_ptr<T>::operator->() const [with T = gazebo::rendering::Scene; typename boost::detail::sp_member_access<T>::type = gazebo::rendering::Scene*]: Assertion `px != 0' failed.
[ERROR] [gzserver-1]: process has died [pid 5945, exit code -6, cmd 'gzserver /home/anubhav1772/ros2_ws/install/gazebo_sensor_ros2/share/gazebo_sensor_ros2/worlds/realsense/realsense.world -slibgazebo_ros_init.so -slibgazebo_ros_factory.so -slibgazebo_ros_force_system.so'].
[gzclient-2] gzclient: /usr/include/boost/smart_ptr/shared_ptr.hpp:728: typename boost::detail::sp_member_access<T>::type boost::shared_ptr<T>::operator->() const [with T = gazebo::rendering::Camera; typename boost::detail::sp_member_access<T>::type = gazebo::rendering::Camera*]: Assertion `px != 0' failed.
[ERROR] [gzclient-2]: process has died [pid 5947, exit code -6, cmd 'gzclient'].

Fix - https://answers.ros.org/question/358847/cannot-launch-gzclient-on-a-launch-file-results-in-shared_ptr-assertion-error/
            
The solution is to source Gazebo's setup file, i.e.:

. /usr/share/gazebo/setup.sh

This is needed to set some necessary environment variables in case they're going to be overridden, 
which is a common use case. 
'''

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('ur5_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    # world_file_name = 'realsense/realsense.world'
    world_file_name = 'realsense/realsense_setup.world'
    world_path = os.path.join(get_package_share_directory('ur5_gazebo'),
                         'worlds', world_file_name)

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # x_pose = LaunchConfiguration('x_pose', default='-2.0')
    # y_pose = LaunchConfiguration('y_pose', default='-0.5')

    # Set the path to the SDF model files.
    gaz_pkg_share = FindPackageShare(package='realsense2_description').find('realsense2_description')
    gazebo_models_path = os.path.join(gaz_pkg_share, 'meshes')
    # os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] =  os.environ['GAZEBO_MODEL_PATH'] + ':' + gazebo_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  gazebo_models_path

    # gzserver_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
    #     ),
    #     launch_arguments={'world': world}.items()
    # )

    # gzclient_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
    #     )
    # )

    world = LaunchConfiguration('world')

    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=world_path,
        description='Full path to world model file to load')


    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'),
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    # ld.add_action(gzserver_cmd)
    # ld.add_action(gzclient_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(gazebo)
    ld.add_action(robot_state_publisher_cmd)

    return ld
