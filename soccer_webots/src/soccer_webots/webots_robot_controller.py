#!/usr/bin/env python3
import os
import math
import time
from visualization_msgs.msg import Marker
from controller import Robot, Node, Field

import rospy
from geometry_msgs.msg import PoseArray, Pose, Point
from sensor_msgs.msg import JointState, Imu, Image, CameraInfo


class RobotController:
    def __init__(self, ):
        """
        The RobotController, a Webots controller that controls a single robot.
        The environment variable WEBOTS_ROBOT_NAME should be set to "amy", "rory", "jack" or "donna" if used with
        4_bots.wbt or to "amy" if used with 1_bot.wbt.

        :param ros_active: Whether ROS messages should be published
        :param robot: The name of the robot to use, currently one of wolfgang, darwin, nao, op3
        :param do_ros_init: Whether to call rospy.init_node (only used when ros_active is True)
        :param external_controller: Whether an external controller is used, necessary for RobotSupervisorController
        :param base_ns: The namespace of this node, can normally be left empty
        """

        self.walkready = [0] * 20
        self.time = 0
        self.robot_node = Robot()
        base_ns = "/robot1"
        # base_ns = "/" + self.robot_node.getName()
        self.motors = []
        self.sensors = []
        self.timestep = int(self.robot_node.getBasicTimeStep())

        self.motor_names = ["left_arm_motor_0", "left_arm_motor_1", "right_arm_motor_0", "right_arm_motor_1",
                            "right_leg_motor_0", "right_leg_motor_1", "right_leg_motor_2", "right_leg_motor_3",
                            "right_leg_motor_4", "right_leg_motor_5", "left_leg_motor_0", "left_leg_motor_1",
                            "left_leg_motor_2", "left_leg_motor_3", "left_leg_motor_4", "left_leg_motor_5",
                            "head_motor_0", "head_motor_1"
                            ]
        self.external_motor_names = self.motor_names
        sensor_postfix = "_sensor"
        accel_name = "imu accelerometer"
        gyro_name = "imu gyro"
        camera_name = "camera"

        self.pressure_sensor_names = ["right_leg_foot_sensor_1", "right_leg_foot_sensor_2", "right_leg_foot_sensor_3",
                                      "right_leg_foot_sensor_4",
                                      "left_leg_foot_sensor_1", "left_leg_foot_sensor_2", "left_leg_foot_sensor_3",
                                      "left_leg_foot_sensor_4"]
        self.pressure_sensors = []
        self.external_pressure_names = self.pressure_sensor_names
        for name in self.pressure_sensor_names:
            sensor = self.robot_node.getDevice(name)
            sensor.enable(self.timestep)
            self.pressure_sensors.append(sensor)

        # self.robot_node = self.supervisor.getFromDef(self.robot_node_name)
        for motor_name in self.motor_names:
            self.motors.append(self.robot_node.getDevice(motor_name))
            self.motors[-1].enableTorqueFeedback(self.timestep)
            self.sensors.append(self.robot_node.getDevice(motor_name + sensor_postfix))
            self.sensors[-1].enable(self.timestep)

        self.accel = self.robot_node.getDevice(accel_name)
        self.accel.enable(self.timestep)
        self.gyro = self.robot_node.getDevice(gyro_name)
        self.gyro.enable(self.timestep)
        self.camera = self.robot_node.getDevice(camera_name)
        self.camera.enable(self.timestep)

        clock_topic = base_ns + "/clock"
        rospy.Subscriber(base_ns + '/all_motor', JointState, self.all_motor_callback)
        self.pub_imu = rospy.Publisher(base_ns + "/imu_raw", Imu, queue_size=1)
        self.pub_js = rospy.Publisher(base_ns + "/joint_states", JointState, queue_size=1)
        self.pub_cam = rospy.Publisher(base_ns + "/camera/image_raw", Image, queue_size=1)
        self.pub_cam_info = rospy.Publisher(base_ns + "/camera/camera_info", CameraInfo, queue_size=1, latch=True)

        self.pressure_sensors_pub = {
            i: rospy.Publisher(base_ns + "/foot_pressure_{}".format(i), Marker, queue_size=10) for i in range(8)}

        # publish camera info once, it will be latched
        self.cam_info = CameraInfo()
        self.cam_info.header.stamp = rospy.Time.from_seconds(self.time)
        self.cam_info.header.frame_id = "camera"
        self.cam_info.height = self.camera.getHeight()
        self.cam_info.width = self.camera.getWidth()
        f_y = self.mat_from_fov_and_resolution(
            self.h_fov_to_v_fov(self.camera.getFov(), self.cam_info.height, self.cam_info.width),
            self.cam_info.height)
        f_x = self.mat_from_fov_and_resolution(self.camera.getFov(), self.cam_info.width)
        self.cam_info.K = [f_x, 0, self.cam_info.width / 2,
                           0, f_y, self.cam_info.height / 2,
                           0, 0, 1]
        self.cam_info.P = [f_x, 0, self.cam_info.width / 2, 0,
                           0, f_y, self.cam_info.height / 2, 0,
                           0, 0, 1, 0]
        self.pub_cam_info.publish(self.cam_info)

    def mat_from_fov_and_resolution(self, fov, res):
        return 0.5 * res * (math.cos((fov / 2)) / math.sin((fov / 2)))

    def h_fov_to_v_fov(self, h_fov, height, width):
        return 2 * math.atan(math.tan(h_fov * 0.5) * (height / width))

    def step_sim(self):
        self.time += self.timestep / 1000
        self.robot_node.step(self.timestep)

    def step(self):
        self.step_sim()
        self.publish_ros()
        # rospy.loginfo("fgdg%f", self.pressure_sensors[0].getValues()[0])

    def publish_ros(self):
        self.publish_camera()
        self.publish_joint_states()
        self.publish_imu()
        self.get_pressure_message()

    def all_motor_callback(self, msg):
        for i, name in enumerate(msg.name):
            motor_index = self.external_motor_names.index(name)
            self.motors[motor_index].setPosition(msg.position[i])

    def publish_camera(self):
        img_msg = Image()
        img_msg.header.stamp = rospy.Time.from_seconds(self.time)
        img_msg.header.frame_id = "camera_base"
        img_msg.height = self.camera.getHeight()
        img_msg.width = self.camera.getWidth()
        img_msg.encoding = "bgra8"
        img_msg.step = 4 * self.camera.getWidth()
        img = self.camera.getImage()
        img_msg.data = img
        self.pub_cam.publish(img_msg)

    def publish_imu(self):
        self.pub_imu.publish(self.get_imu_msg(head=False))

    def get_imu_msg(self, head=False):
        msg = Imu()
        msg.header.stamp = rospy.Time.from_seconds(self.time)

        msg.header.frame_id = "imu_link"

        # change order because webots has different axis

        accel_vels = self.accel.getValues()
        msg.linear_acceleration.x = accel_vels[0]
        msg.linear_acceleration.y = accel_vels[1]
        msg.linear_acceleration.z = accel_vels[2]
        gyro_vels = self.gyro.getValues()
        msg.angular_velocity.x = gyro_vels[0]
        msg.angular_velocity.y = gyro_vels[1]
        msg.angular_velocity.z = gyro_vels[2]

        return msg

    def get_image(self):
        return self.camera.getImage()

    def get_joint_state_msg(self):
        js = JointState()
        js.name = []
        js.header.stamp = rospy.Time.from_seconds(self.time)
        js.position = []
        js.effort = []
        for i in range(len(self.sensors)):
            js.name.append(self.external_motor_names[i])
            value = self.sensors[i].getValue()
            js.position.append(value)
            js.effort.append(self.motors[i].getTorqueFeedback())
        return js

    def publish_joint_states(self):
        self.pub_js.publish(self.get_joint_state_msg())

    def get_pressure_message(self):

        for i in range(8):
            current_time = rospy.Time.from_sec(self.time)

            marker_object = Marker()
            if i < 4:
                marker_object.header.frame_id = "left_foot"
            else:
                marker_object.header.frame_id = "right_foot"

            marker_object.header.stamp = current_time
            marker_object.ns = "Soccer_bot"
            marker_object.id = 3
            marker_object.type = Marker.ARROW
            marker_object.action = Marker.ADD
            if i == 0 or i == 4:
                tip = Point()
                tip.x = 0.05
                tip.y = 0.05
                tip.z = 0
                tail = Point()
                tail.x = 0.05
                tail.y = 0.05
                tail.z = 0.01 * self.pressure_sensors[i].getValues()[2]
            elif i == 1 or i == 5:
                tip = Point()
                tip.x = -0.05
                tip.y = 0.05
                tip.z = 0
                tail = Point()
                tail.x = -0.05
                tail.y = 0.05
                tail.z = 0.01 * self.pressure_sensors[i].getValues()[2]
            elif i == 2 or i == 6:
                tip = Point()
                tip.x = 0.05
                tip.y = -0.05
                tip.z = 0
                tail = Point()
                tail.x = 0.05
                tail.y = -0.05
                tail.z = 0.01 * self.pressure_sensors[i].getValues()[2]
            elif i == 3 or i == 7:
                tip = Point()
                tip.x = -0.05
                tip.y = -0.05
                tip.z = 0
                tail = Point()
                tail.x = -0.05
                tail.y = -0.05
                tail.z = 0.01 * self.pressure_sensors[i].getValues()[2]

            marker_object.points = [tail, tip]

            marker_object.color.r = 0
            marker_object.color.g = 0
            marker_object.color.b = 1
            marker_object.color.a = 1.0

            marker_object.scale.x = 0.01
            marker_object.scale.y = 0.01
            marker_object.scale.z = 0.01

            marker_object.pose.orientation.x = 0
            marker_object.pose.orientation.y = 0
            marker_object.pose.orientation.z = 0
            marker_object.pose.orientation.w = 1

            self.pressure_sensors_pub[i].publish(marker_object)
