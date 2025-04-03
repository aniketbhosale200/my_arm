import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart
import xacro

def generate_launch_description():
    my_robot_dir = get_package_share_directory("aarm")
    xacro_file = os.path.join(my_robot_dir, "urdf", "aarm.urdf.xacro")
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {"robot_description": robot_description_config.toxml()}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": True}],
        output="screen"
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py"])
        ),
        launch_arguments={"verbose": "true", "world": os.path.join(my_robot_dir, "worlds", "empty.world")}.items()
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "/robot_description", "-entity", "aarm"],
        output="screen"
    )

    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    load_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    load_gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    # 🕒 Delay the controllers to make sure Gazebo fully spawns the entity
    delay_controllers = TimerAction(
        period=3.0,  # Wait 3 seconds after spawning
        actions=[load_joint_state_broadcaster, load_arm_controller, load_gripper_controller]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        gazebo,
        spawn_entity,
        delay_controllers  # 🔧 Ensures robot is fully spawned before controllers start
    ])
