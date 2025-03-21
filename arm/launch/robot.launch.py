import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, LogInfo, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
from launch.event_handlers import OnProcessStart
from launch.conditions import IfCondition

def generate_launch_description():
    my_robot_dir = get_package_share_directory("arm")
    xacro_file = "/home/imbatman/arm_ws/src/arm/urdf/arm.urdf.xacro"
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {"robot_description": robot_description_config.toxml()}

    # Declare arguments
    model_arg = DeclareLaunchArgument(name="model", default_value=xacro_file, description="Path to robot Xacro file")
    rviz_config_arg = DeclareLaunchArgument(name="rviz_config", default_value=os.path.join(my_robot_dir, "rviz", "config.rviz"), description="RViz config")
    use_sim_arg = DeclareLaunchArgument(name="use_sim", default_value="true", description="Use sim time")
    world_arg = DeclareLaunchArgument(name="world", default_value=os.path.join(my_robot_dir, "worlds", "empty.world"), description="Gazebo world")

    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": PythonExpression(["'", LaunchConfiguration("use_sim"), "' == 'true'"])}],
        output="screen"
    )

    # Joint state publisher
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[{"use_sim_time": PythonExpression(["'", LaunchConfiguration("use_sim"), "' == 'true'"])}],
        output="screen"
    )

    # Gazebo setup
    gazebo_model_path = SetEnvironmentVariable(name="GAZEBO_MODEL_PATH", value=[os.path.join(my_robot_dir, "models")])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")),
        launch_arguments={"verbose": "true", "world": LaunchConfiguration("world")}.items()
    )

    # Spawn entity
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "/robot_description", "-entity", "arm"],
        output="screen"
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[{"use_sim_time": PythonExpression(["'", LaunchConfiguration("use_sim"), "' == 'true'"])}],
        condition=IfCondition(LaunchConfiguration("use_sim")),
        output="screen"
    )

    # Event handlers
    delay_gazebo = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=robot_state_publisher_node,
            on_start=[gazebo]
        )
    )
    delay_joint_state = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=robot_state_publisher_node,
            on_start=[joint_state_publisher_node]
        )
    )

    # Launch description
    return LaunchDescription([
        model_arg, rviz_config_arg, use_sim_arg, world_arg,
        LogInfo(msg="Starting robot state publisher..."), robot_state_publisher_node,
        delay_joint_state,  # Start joint_state_publisher after robot_state_publisher
        gazebo_model_path,
        delay_gazebo,       # Start Gazebo after robot_state_publisher
        LogInfo(msg="Spawning robot..."), spawn_entity,
        LogInfo(msg="Launching RViz..."), rviz_node
    ])
