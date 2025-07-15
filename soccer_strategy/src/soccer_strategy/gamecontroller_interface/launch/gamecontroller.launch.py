from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gamecontroller_interface',
            executable='gamecontroller_interface',
            name='gamecontroller_interface',
            parameters=[
                {'team_id': 10},
                {'robot_id': 1}
            ],
            output='screen'
        )
    ])
