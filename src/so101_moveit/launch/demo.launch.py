from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "port",
            default_value="/dev/ttyACM1",
            description="Serial port for the robot hardware.",
        )
    )
    # Default calibration file path
    default_calibration_file = PathJoinSubstitution(
        [FindPackageShare("so101_moveit"), "config", "follower_arm_calib.json"]
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "calibration_file",
            default_value=default_calibration_file,
            description="Path to motor calibration YAML file (default: motor_calibration.yaml).",
        )
    )

    # Initialize Arguments
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    port = LaunchConfiguration("port")
    calibration_file = LaunchConfiguration("calibration_file")

    moveit_config = (
        MoveItConfigsBuilder("so101_new_calib", package_name="so101_moveit")
        .robot_description(
            mappings={
                "use_fake_hardware": use_fake_hardware,
                "port": port,
                "calibration_file": calibration_file,
            }
        )
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    # RViz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("so101_moveit"), "config", "moveit.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = PathJoinSubstitution(
        [FindPackageShare("so101_moveit"), "config", "ros2_controllers.yaml"]
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="both",
    )

    # Load controllers
    load_controllers = []
    for controller in [
        "arm_controller",
        "gripper_controller",
        "joint_state_broadcaster",
    ]:
        load_controllers.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "-c", "/controller_manager"],
                output="screen",
            )
        )

    return LaunchDescription(
        declared_arguments
        + [
            robot_state_publisher,
            ros2_control_node,
            run_move_group_node,
            rviz_node,
        ]
        + load_controllers
    )
