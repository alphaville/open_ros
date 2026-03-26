from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="parametric_optimizer_ros2",
            executable="open_node_ros2",
            name="open_node_ros2",
            output="screen",
            parameters=[PathJoinSubstitution([
                FindPackageShare("parametric_optimizer_ros2"),
                "config",
                "open_params.yaml",
            ])],
        )
    ])