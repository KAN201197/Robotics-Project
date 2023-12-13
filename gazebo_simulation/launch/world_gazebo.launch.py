from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    world_name = DeclareLaunchArgument(
        'world_name',
        default_value='$(find gazebo_simulation)/worlds/Simulation.world',
        description='Path to the Gazebo world file'
    )

    paused = DeclareLaunchArgument(
        'paused',
        default_value='false',
        description='Start Gazebo in paused mode (true/false)'
    )

    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time (true/false)'
    )

    gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Show the Gazebo GUI (true/false)'
    )

    headless = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo in headless mode (true/false)'
    )

    debug = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Start gzserver with gdb debugging (true/false)'
    )

    # Specify the path to the Gazebo world file
    world_file_name = LaunchConfiguration('world_name')

    # Define a Gazebo launch node
    gazebo_node = Node(
        package='gazebo_ros',
        executable='gzserver',
        name='gazebo',
        namespace='',
        output='screen',
        parameters=[{'world': LaunchConfiguration('world_name')}],
        remappings=[('/gazebo/default/physics/enable_physics', '/gazebo/default/physics/enable_physics')],
        condition=IfCondition(LaunchConfiguration('gui')),  # Conditionally launch with GUI
    )

    # Log information about the launch
    log_info = LogInfo(
        condition=IfCondition(LaunchConfiguration('debug')),
        namespace='',
        text=TextSubstitution(text='Starting Gazebo with debugging enabled.'),
    )

    return LaunchDescription([
        world_name,
        paused,
        use_sim_time,
        gui,
        headless,
        debug,
        gazebo_node,
        log_info,
    ])
