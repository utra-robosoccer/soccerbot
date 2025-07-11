cmake_minimum_required(VERSION 3.0.2)
project(soccerbot)

find_package(
    catkin
    REQUIRED
    soccer_common
    soccer_msgs
    # soccer_webots
    soccer_trajectories
    soccer_object_detection
    bez1_description
    bez2_description
    soccer_strategy
    soccer_object_localization
    soccer_localization
    soccer_pycontrol
    soccer_firmware_interface
    foxglove_bridge
    robot_pose_ekf
    robot_localization
    map_server
    usb_cam
    imu_complementary_filter
    # webots_ros
)

catkin_package(
    CATKIN_DEPENDS
    soccer_msgs
    soccer_object_detection
    robot_pose_ekf
    robot_localization
    map_server
    usb_cam
    imu_complementary_filter
    # webots_ros
)
