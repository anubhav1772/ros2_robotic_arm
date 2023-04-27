import os
from launch import LaunchDescription
from launch.actions import TimerAction, DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit

# ros2 topic pub /ur_manipulator/joint_trajectory trajectory_msgs/JointTrajectory '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "base_link"}, joint_names: ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"], points: [{positions: [1.57, 1.57, 0.0, 0.0, 0.0, 0.0], velocities: [], accelerations: [], time_from_start: {sec: 1, nanosec: 0}}]}'


# process has died [pid 13572, exit code 255, cmd 
# 'gzserver -slibgazebo_ros_init.so -slibgazebo_ros_factory.so 
# -slibgazebo_ros_force_system.so'].
# For getting rid of above error, kill gzserver and gzclient.
def generate_launch_description():
    # Set the path to the Gazebo ROS package
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
  
    # Set the path to this package.
    desc_pkg_share = FindPackageShare(package='ur5_description').find('ur5_description')
    # desc_pkg_share = get_package_share_directory('ur5_description')
    gaz_pkg_share = FindPackageShare(package='ur5_gazebo').find('ur5_gazebo')

    # Set the path to the world file
    world_file_name = 'pick_place.world'
    world_path = os.path.join(gaz_pkg_share, 'worlds', world_file_name)
    
    # Set the path to the SDF model files.
    gazebo_models_path = os.path.join(gaz_pkg_share, 'models')
    # os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
    
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] =  os.environ['GAZEBO_MODEL_PATH'] + ':' + gazebo_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  gazebo_models_path


    # if 'GAZEBO_PLUGIN_PATH' in os.environ:
    #     os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + gaz_pkg_share + '/lib'
    # else:
    #     os.environ['GAZEBO_PLUGIN_PATH'] = gaz_pkg_share + '/lib'

    # Set the path to the xacro file
    # urdf_model_path = os.path.join(desc_pkg_share, 'urdf/ur.urdf.xacro')
    xacro_file_path = os.path.join(desc_pkg_share, 'urdf/', 'ur.urdf.xacro')
    # print(xacro_file_path)

    controller_file = os.path.join(gaz_pkg_share, "config", "ur5_controllers.yaml")
    # print(controller_file)

    # print(pkg_gazebo_ros)
    # print(desc_pkg_share)
    # print(gaz_pkg_share)
    # print(world_path)
    # print(gazebo_models_path)
    # print(urdf_model_path)
    
    headless = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_simulator = LaunchConfiguration('use_simulator')
    world = LaunchConfiguration('world')
    # urdf_model = LaunchConfiguration('urdf_model')
    controllers_file = LaunchConfiguration("controllers_file")
    # use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    
    declare_simulator_cmd = DeclareLaunchArgument(
        name='headless',
        default_value='False',
        description='Whether to execute gzclient. If False, then we willnot see gazebo screen.')
    
    declare_use_simulator_cmd = DeclareLaunchArgument(
        name='use_simulator',
        default_value='True',
        description='Whether to start the simulator')
    
    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=world_path,
        description='Full path to world model file to load')
    
    # Declare the launch arguments  
    # declare_urdf_model_path_cmd = DeclareLaunchArgument(
    #     name='urdf_model', 
    #     default_value=urdf_model_path, 
    #     description='Absolute path to robot urdf file')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    # initial_joint_controllers = PathJoinSubstitution(
    #     [FindPackageShare(runtime_config_package), "config", controllers_file]
    # )
    
    # declare_use_fake_hardware = DeclareLaunchArgument(
    #     "use_fake_hardware",
    #     default_value="true",
    #     description="Start robot with fake hardware mirroring command to its states.")

    declare_controllers_file = DeclareLaunchArgument(
            "controllers_file",
            default_value="ur5_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    
    # robot_description_content = Command(
    #     [
    #         PathJoinSubstitution([FindExecutable(name="xacro")]),
    #         " ",
    #         PathJoinSubstitution([desc_pkg_share, urdf_model_path]),
    #         " "
    #     ]
    # )

    # robot_desc = {"robot_description": robot_description_content}
    robot_description_config = xacro.process_file(xacro_file_path, mappings={'name' : 'ur'})
    robot_desc = robot_description_config.toxml()

    # # Gazebo launch
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
    #     )
    # )

    # # Start Gazebo server
    # start_gazebo_server_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
    #     condition=IfCondition(use_simulator),
    #     launch_arguments={'world': world}.items())

    # # Start Gazebo client    
    # start_gazebo_client_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    #     condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'),
        )
    )

    # gazebo_node = ExecuteProcess(cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_factory.so'], output='screen')

    # Launch the robot
    # spawn_entity_cmd = Node(
    #     package='gazebo_ros', 
    #     executable='spawn_entity.py',
    #     name="urdf_spawner",
    #     output="screen",
    #     arguments=['-entity', 'ur', 
    #                 '-topic', '/robot_description',
    #                     '-x', '0.0',
    #                     '-y', '0.0',
    #                     '-z', '0.0',
    #                     '-R', '0.0',
    #                     '-P', '0.0',
    #                     '-Y', '0.0'])
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_desc}],
        output="screen")
    
    # ros2_control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[robot_desc, controller_file],
    #     output={
    #             "stdout": "screen",
    #             "stderr": "screen",
    #         },
    #     )
    # executable is not 'spawner.py' but 'spawner'
    # Check if the spawner file is present in the libexec directory of the 
    # controller_manager package. You can do this by navigating to the libexec 
    # directory (/home/anubhav1772/ros2_ws/install/controller_manager/lib/controller_manager) 
    # and checking if the file is present.
    load_joint_state_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],)

    # load_ur_manipulator_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'configured', 'ur_manipulator_controller'],
    #     output='screen')

    load_ur_manipulator_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ur_manipulator_controller", "-c", "/controller_manager"],)

    spawn_entity_cmd = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        name="urdf_spawner",
        output="screen",
        arguments=['-entity', 'ur', 
                    '-topic', '/robot_description'])
    
    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_ur_manipulator_controller],
            )
        ),
        declare_use_simulator_cmd,
        declare_simulator_cmd,
        declare_use_sim_time_cmd,
        declare_world_cmd,
        gazebo,
        spawn_entity_cmd,
        load_joint_state_controller
    ])

    # ld = LaunchDescription()
    
    # # Declare the launch options
    # ld.add_action(declare_use_simulator_cmd)
    # ld.add_action(declare_simulator_cmd)
    # ld.add_action(declare_use_sim_time_cmd)
    # ld.add_action(declare_world_cmd)
    # # ld.add_action(declare_urdf_model_path_cmd)
    # # ld.add_action(declare_use_fake_hardware)
    # ld.add_action(declare_controllers_file)
    # # ld.add_action(start_gazebo_server_cmd)
    # # ld.add_action(start_gazebo_client_cmd)
    # # ld.add_action(RegisterEventHandler(
    # #     event_handler=OnProcessExit(
    # #         target_action=spawn_entity_cmd,
    # #         on_exit=[load_joint_state_controller],
    # #     )
    # # ))
    # # ld.add_action(RegisterEventHandler(
    # #     event_handler=OnProcessExit(
    # #         target_action=load_joint_state_controller,
    # #         on_exit=[load_ur_manipulator],
    # #     )
    # # ))
    # ld.add_action(gazebo)
    # # ld.add_action(ros2_control_node)
    # ld.add_action(spawn_entity_cmd)
    # ld.add_action(load_joint_state_controller)
    # ld.add_action(load_ur_manipulator)
    # # ld.add_action(
    # #     TimerAction(
    # #         period=0.0,
    # #         actions=[spawn_entity_cmd]))
    # # ld.add_action(
    # #     TimerAction(
    # #         period=0.0,
    # #         actions=[robot_state_publisher]))

    # return ld