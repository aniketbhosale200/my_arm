from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    # Load MoveIt config and override URDF from main arm package
    moveit_config = MoveItConfigsBuilder("aarm", package_name="aarm_moveit").to_moveit_configs()
    urdf_path = os.path.join(get_package_share_directory("aarm_moveit"), "config", "aarm.urdf.xacro")
    robot_desc = xacro.process_file(urdf_path).toxml()
    moveit_config.robot_description = {"robot_description": robot_desc}

    # Gazebo launch with explicit world file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])
        ),
        launch_arguments={
            "verbose": "true",
            "pause": "false",
            "world": "/opt/ros/humble/share/gazebo_ros/worlds/empty.world"
        }.items(),
    )

    # Spawn robot
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "/robot_description", "-entity", "aarm"],
        output="screen",
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description, {"use_sim_time": use_sim_time}],
        output="screen",
    )

    # Controller manager with ros2_controllers.yaml from arm_moveit
    controllers_file = PathJoinSubstitution([FindPackageShare("aarm_moveit"), "config", "ros2_controllers.yaml"])
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers_file],
        output="screen",
    )

    # Controller spawners
    controller_spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[ctrl, "--controller-manager", "/controller_manager"],
            output="screen",
        )
        for ctrl in ["joint_state_broadcaster", "arm_controller", "gripper_controller"]
    ]

    # MoveIt move_group node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"trajectory_execution.allowed_execution_duration_scaling": 2.0},
            {"use_sim_time": use_sim_time},
        ],
    )

    # RViz
    rviz_config_file = PathJoinSubstitution([FindPackageShare("aarm_moveit"), "config", "moveit.rviz"])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation time"),
        gazebo_launch,
        spawn_entity,
        robot_state_publisher,
        controller_manager,
        *controller_spawners,
        move_group_node,
        rviz_node,
    ])