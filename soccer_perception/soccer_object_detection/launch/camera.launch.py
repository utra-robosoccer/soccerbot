from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_model = LaunchConfiguration("robot_model")
    param_file_name = robot_model  # Used directly since we skip the simulation logic for now

    return LaunchDescription(
        [
            # Declare arguments
            DeclareLaunchArgument("robot_model", default_value="bez2"),
            DeclareLaunchArgument("play_rosbag", default_value="false"),
            DeclareLaunchArgument("record_rosbag", default_value="true"),
            DeclareLaunchArgument("simulation", default_value="false"),
            # USB camera node
            Node(
                package="usb_cam",
                executable="usb_cam_node_exe",  # ROS 2 version of usb_cam uses this name
                name="camera",
                output="screen",
                parameters=[
                    {
                        "video_device": "/dev/video0",
                        "image_width": 640,
                        "image_height": 480,
                        "pixel_format": "mjpeg2rgb",
                        "camera_frame_id": "camera",
                        "io_method": "mmap",
                    }
                ],
                respawn=True,
                respawn_delay=30.0,
            ),
            # Ball detector node
            # Node(
            #     package="soccer_object_localization",
            #     executable="detector_fieldline_ros",  # Python files don't include .py extension in ROS 2
            #     name="ball_detector",
            #     output="screen",
            #     parameters=[
            #         PathJoinSubstitution([
            #             FindPackageShare("soccer_object_localization"),
            #             "config",
            #             param_file_name
            #         ])
            #     ]
            # ),
            # Object detector node
            Node(
                package="soccer_object_detection",
                executable="soccer_object_detection",
                name="object_detector",
                output="screen",
                arguments=["--model", PathJoinSubstitution([FindPackageShare("soccer_object_detection"), "models", "half_5.pt"])],
                parameters=[PathJoinSubstitution([FindPackageShare("soccer_object_detection"), "config", param_file_name])],
                remappings=[("/camera/image_raw", "/image_raw")],
            ),
            # # Include foxglove_bridge launch
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(
            #         PathJoinSubstitution([
            #             FindPackageShare("foxglove_bridge"),
            #             "launch",
            #             "foxglove_bridge.launch.py"
            #         ])
            #     ),
            #     launch_arguments={"port": "8765"}.items(),
            # )
        ]
    )
