import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition

def generate_launch_description():
    gui_arg = DeclareLaunchArgument(
        name="gui",
        default_value="true",
        description="Start RViz"
    )

    # Spawn controllers into Gazebo's controller_manager
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )


    delay_joint_state = RegisterEventHandler(
        OnProcessExit(target_action=arm_controller_spawner, on_exit=[joint_state_broadcaster_spawner])
    )
    delay_gripper = RegisterEventHandler(
        OnProcessExit(target_action=joint_state_broadcaster_spawner, on_exit=[gripper_controller_spawner])
    )

    return LaunchDescription([
        gui_arg,
        arm_controller_spawner,
        delay_joint_state,
        delay_gripper
        
    ])
