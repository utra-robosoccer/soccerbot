from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="game_controller_interface",
                executable="game_controller_interface",
                name="game_controller_interface",
                parameters=[{"team_id": 10}, {"robot_id": 1}],
                output="screen",
            )
        ]
    )
