from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    urdf_path = PathJoinSubstitution(
        [FindPackageShare("gazebo_simulation"), "urdf", "robot.xacro"])
    
    world_path = PathJoinSubstitution(
        [FindPackageShare("gazebo_simulation"), "worlds", "Simulation.world"])
    
    world_name = DeclareLaunchArgument(
        'world_name',
        default_value=world_path,
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

    x_pos = DeclareLaunchArgument(
        'x_pos',
        default_value='1.0',
        description='X position of the robot'
    )

    y_pos = DeclareLaunchArgument(
        'y_pos',
        default_value='0.0',
        description='Y position of the robot'
    )

    z_pos = DeclareLaunchArgument(
        'z_pos',
        default_value='0.0',
        description='Z position of the robot'
    )

    roll = DeclareLaunchArgument(
        'roll',
        default_value='0.0',
        description='Roll orientation of the robot'
    )

    pitch = DeclareLaunchArgument(
        'pitch',
        default_value='0.0',
        description='Pitch orientation of the robot'
    )

    yaw = DeclareLaunchArgument(
        'yaw',
        default_value='0.0',
        description='Yaw orientation of the robot'
    )

    # Specify the path to the Gazebo world file
    world_file_name = LaunchConfiguration('world_name')

    # Launch the Gazebo simulation using the equivalent ROS 2 launch action
    gazebo_launch = Node(
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

    # Define the command to generate the robot description
    model = DeclareLaunchArgument(
        'model',
        default_value=urdf_path,
        description='Command to generate the robot description'
    )

    # Launch the robot_state_publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'robot_description': Command(['xacro ', LaunchConfiguration('model')])
                }
            ],
        remappings=[('/joint_states', '/wheeltec/joint_states')],
    )

    # Spawn the robot model in Gazebo
    spawn_urdf = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_urdf',
        output='screen',
        arguments=['-urdf', '-model', 'robot', '-param', 'robot_description',
                   '-x', LaunchConfiguration('x_pos'), '-y', LaunchConfiguration('y_pos'),
                   '-z', LaunchConfiguration('z_pos'), '-R', LaunchConfiguration('roll'),
                   '-P', LaunchConfiguration('pitch'), '-Y', LaunchConfiguration('yaw')],
    )

    return LaunchDescription([
        world_name,
        paused,
        use_sim_time,
        gui,
        headless,
        debug,
        x_pos,
        y_pos,
        z_pos,
        roll,
        pitch,
        yaw,
        gazebo_launch,
        log_info,
        model,
        robot_state_publisher,
        spawn_urdf,
    ])
