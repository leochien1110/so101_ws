"""
macOS-compatible demo launch file for SO-101 MoveIt.

This launch file uses joint_state_publisher instead of ros2_control,
which avoids the threading issues that cause ros2_control to crash on macOS.

Use this for testing VR teleop on macOS. For production/real hardware,
use demo.launch.py on a Linux system.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Build MoveIt config (without ros2_control hardware mappings)
    moveit_config = (
        MoveItConfigsBuilder("so101_new_calib", package_name="so101_moveit")
        .robot_description(
            mappings={
                "use_fake_hardware": "true",
                "port": "/dev/null",
                "calibration_file": "",
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

    # Publish TF from robot_state_publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # Use joint_state_publisher instead of ros2_control (macOS compatible)
    # This publishes joint states and allows the TF tree to be complete
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[
            {"source_list": ["/vr_teleop/joint_states"]},  # Listen to VR teleop commands
            {"rate": 50},
        ],
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            joint_state_publisher,
            run_move_group_node,
            rviz_node,
        ]
    )
