import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # Declare launch arguments
    
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    xacro_file_name = 'robot.xacro'

    xacro_path = os.path.join(
        get_package_share_directory('gazebo_simulation'),
        'urdf',
        xacro_file_name)
    assert os.path.exists(xacro_path), "The mini_mec.xacro does not exist in "+str(xacro_path)
    
    robot_description_config = xacro.process_file(xacro_path)
    robot_desc = robot_description_config.toxml()
    
    # with open(urdf_path, 'r') as infp:
       # robot_desc = infp.read()

    world = os.path.join(
        get_package_share_directory('gazebo_simulation'),
        'worlds',
        'Simulation.world')

    use_sim_time = LaunchConfiguration(
        'use_sim_time',
        default='true',)

    x_pos = LaunchConfiguration(
        'x_pos',
        default='0.0',)

    y_pos = LaunchConfiguration(
        'y_pos',
        default='0.0',)

    z_pos = LaunchConfiguration(
        'z_pos',
        default='0.0',)

    # Launch the Gazebo simulation using the gazebo_ros package's launch file
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_gazebo_ros,
                'launch',
                'gzserver.launch.py'
            )
        ),
        launch_arguments={'world': world}.items(),
    )
    
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # Launch the robot_state_publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
                {
                    'use_sim_time': use_sim_time,
                    'robot_description': robot_desc
                }
            ],
        remappings=[('/joint_states', '/wheeltec/joint_states')],
    )

    # Spawn the robot model in Gazebo
    spawn_robot = Node(
        package='wheeltec_simulation',
        executable='Spawn_robot.py',
        output='screen',
        arguments=[robot_desc],
    )

    return LaunchDescription([
        gzserver_cmd,
        gzclient_cmd,
        robot_state_publisher,
        spawn_robot,
    ])
