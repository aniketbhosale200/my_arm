import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    """
    Launch file to start Gazebo with a 6 DOF robotic arm and gripper in ROS2 Humble using Classic Gazebo.
    Includes robot state publishing, spawning, controller loading, and RViz visualization.
    """

    # Package paths
    pkg_share = FindPackageShare('arm').find('arm')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')

    # Define paths
    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'arm.urdf.xacro'])
    controller_config = PathJoinSubstitution([pkg_share, 'config', 'controllers.yaml'])
    default_world = PathJoinSubstitution([pkg_share, 'worlds', 'empty.world'])
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'config.rviz'])

    # Launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            name='world',
            default_value=default_world,
            description='Path to Gazebo world file'
        ),
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock'
        ),
        DeclareLaunchArgument(
            name='use_gazebo_sim',
            default_value='true',
            description='Pass use_gazebo_sim parameter to XACRO for Gazebo compatibility'
        ),
        DeclareLaunchArgument(
            name='verbose',
            default_value='true',
            description='Enable verbose output for Gazebo'
        ),
        DeclareLaunchArgument(
            name='rviz',
            default_value='true',  # Changed to true to enable RViz by default
            description='Launch RViz for visualization'
        ),
        DeclareLaunchArgument(
            name='rviz_config',
            default_value=rviz_config,
            description='Path to RViz config file'
        )
    ]

    # Process URDF/XACRO
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        xacro_file, ' ',
        'use_gazebo_sim:=', LaunchConfiguration('use_gazebo_sim')
    ])
    robot_description = {'robot_description': robot_description_content}

    # Set Gazebo model path
    gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[PathJoinSubstitution([pkg_share, 'models']), ":", os.environ.get('GAZEBO_MODEL_PATH', '')]
    )

    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gazebo_ros_pkg, 'launch', 'gazebo.launch.py'])
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'verbose': LaunchConfiguration('verbose'),
            'pause': 'false'
        }.items()
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Spawn Entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'arm',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'  # Small offset to prevent ground collision
        ],
        output='screen'
    )

    # ROS2 Control Setup
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_config],
        output='screen'
    )

    # Controller loading with spawner
    controller_names = ['joint_state_broadcaster', 'arm_controller', 'gripper_controller']
    load_controllers = [
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[name, '--controller-manager', '/controller_manager'],
            output='screen'
        ) for name in controller_names
    ]

    # RViz (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='both',  # Changed to 'both' to capture logs and screen output for debugging
    )

    # Event handlers for sequencing
    event_handlers = [
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[LogInfo(msg='Loading joint_state_broadcaster...'), load_controllers[0]]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_controllers[0],
                on_exit=[LogInfo(msg='Loading arm_controller...'), load_controllers[1]]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_controllers[1],
                on_exit=[LogInfo(msg='Loading gripper_controller...'), load_controllers[2]]
            )
        )
    ]

    # Combine all launch actions
    return LaunchDescription(
        declared_arguments + [
            LogInfo(msg='Initializing Gazebo simulation...'),
            gazebo_model_path,
            gazebo_launch,
            robot_state_publisher,
            controller_manager,
            spawn_entity,
            *event_handlers,
            LogInfo(msg='Launching RViz...'),
            rviz_node
        ]
    )
