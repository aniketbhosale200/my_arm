import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, LogInfo, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    my_robot_dir = get_package_share_directory("arm")
    
    control_params = os.path.join(my_robot_dir, "config", "controllers.yaml")

    # Declare launch arguments
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(my_robot_dir, "urdf", "arm.urdf.xacro"),
        description="Path to robot Xacro file"
    )
    rviz_config_arg = DeclareLaunchArgument(
        name="rviz_config",
        default_value=os.path.join(my_robot_dir, "rviz", "config.rviz"),
        description="RViz config file"
    )
    use_sim_arg = DeclareLaunchArgument(
        name="use_sim",
        default_value="true",
        description="Use simulation time"
    )
    world_arg = DeclareLaunchArgument(
        name="world",
        default_value=os.path.join(my_robot_dir, "worlds", "empty.world"),
        description="Gazebo world file"
    )

    # Process the Xacro file dynamically using a Command substitution.
    robot_description_content = Command(['xacro ', LaunchConfiguration('model')])
    robot_description = {"robot_description": robot_description_content}

    # Robot state publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": LaunchConfiguration("use_sim")}],
        output="screen"
    )

    # Set the Gazebo model path environment variable
    gazebo_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=os.path.join(my_robot_dir, "models")
    )

    # Include the Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("gazebo_ros"),
            "launch",
            "gazebo.launch.py"
        )),
        launch_arguments={"verbose": "true", "world": LaunchConfiguration("world")}.items()
    )

    # Spawn the robot entity in Gazebo
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "/robot_description", "-entity", "arm", "-timeout", "60"],
        output="screen"
    )

    # RViz node to visualize the robot and planning scene
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim")}],
        output="screen"
    )

    # Controller spawners with explicit parameter file
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager", "--controller-manager-timeout", "30"],
        parameters=[control_params],  # Explicitly pass the params file
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_sim"))
    )
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager", "--controller-manager-timeout", "30"],
        parameters=[control_params],  # Explicitly pass the params file
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_sim"))
    )
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager", "--controller-manager-timeout", "30"],
        parameters=[control_params],  # Explicitly pass the params file
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_sim"))
    )

    # Add a delay to ensure Gazebo and spawn_entity are fully initialized
    delay_controllers = ExecuteProcess(
        cmd=['sleep', '5'],
        output='screen'
    )

    return LaunchDescription([
        model_arg,
        rviz_config_arg,
        use_sim_arg,
        world_arg,
        LogInfo(msg="Starting robot state publisher..."),
        robot_state_publisher_node,
        gazebo_model_path,
        gazebo,
        spawn_entity,
        delay_controllers,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
        rviz_node
    ])
