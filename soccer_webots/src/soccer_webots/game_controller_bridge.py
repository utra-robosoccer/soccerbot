#!/usr/bin/env python3
import argparse
import os
import sys
import math
import socket
import time

import rospy
import struct
import tf
from rosgraph_msgs.msg import Clock

from sensor_msgs.msg import CameraInfo, Image, Imu, JointState
from std_msgs.msg import Bool, Float64
import messages_pb2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseWithCovarianceStamped


class GameControllerBridge():
    def __init__(self):
        rospy.init_node("game_controller_bridge")
        parser = argparse.ArgumentParser()
        parser.add_argument('--robot_name', help="which robot should be started")

        args, unknown = parser.parse_known_args()
        rospy.set_param("name", args.robot_name)
        self.base_frame = args.robot_name
        self.MIN_FRAME_STEP = 16  # ms
        self.MIN_CONTROL_STEP = 8  # ms
        self.joint_command = [0, 1.5, 0, 1.5, 0, 0, 0.564, -1.176, 0.613, 0, 0, 0, 0.564, -1.176, 0.613, 0, 0, 0]
        self.motor_names = ["left_arm_motor_0 [shoulder]", "left_arm_motor_1", "right_arm_motor_0 [shoulder]",
                            "right_arm_motor_1",
                            "right_leg_motor_0", "right_leg_motor_1 [hip]", "right_leg_motor_2", "right_leg_motor_3",
                            "right_leg_motor_4", "right_leg_motor_5", "left_leg_motor_0", "left_leg_motor_1 [hip]",
                            "left_leg_motor_2", "left_leg_motor_3", "left_leg_motor_4", "left_leg_motor_5",
                            "head_motor_0", "head_motor_1"
                            ]

        self.motor_count = len(self.motor_names)
        self.external_motor_names = \
            ["left_arm_motor_0", "left_arm_motor_1", "right_arm_motor_0", "right_arm_motor_1",
             "right_leg_motor_0", "right_leg_motor_1", "right_leg_motor_2", "right_leg_motor_3",
             "right_leg_motor_4", "right_leg_motor_5", "left_leg_motor_0", "left_leg_motor_1",
             "left_leg_motor_2", "left_leg_motor_3", "left_leg_motor_4", "left_leg_motor_5",
             "head_motor_0", "head_motor_1"
             ]

        self.sensor_names = ["left_arm_motor_0_sensor", "left_arm_motor_1_sensor", "right_arm_motor_0_sensor",
                             "right_arm_motor_1_sensor",
                             "right_leg_motor_0_sensor", "right_leg_motor_1_sensor", "right_leg_motor_2_sensor",
                             "right_leg_motor_3_sensor",
                             "right_leg_motor_4_sensor", "right_leg_motor_5_sensor", "left_leg_motor_0_sensor",
                             "left_leg_motor_1_sensor",
                             "left_leg_motor_2_sensor", "left_leg_motor_3_sensor", "left_leg_motor_4_sensor",
                             "left_leg_motor_5_sensor",
                             "head_motor_0_sensor", "head_motor_1_sensor"
                             ]
        self.regular_sensor_names = ["imu accelerometer", "imu gyro", "imu accelerometer", "camera"

                                     ]
        self.sensor_names.extend(self.regular_sensor_names)
        self.pressure_sensor_names = ["right_leg_foot_sensor_1", "right_leg_foot_sensor_2", "right_leg_foot_sensor_3",
                                      "right_leg_foot_sensor_4",
                                      "left_leg_foot_sensor_1", "left_leg_foot_sensor_2", "left_leg_foot_sensor_3",
                                      "left_leg_foot_sensor_4"]
        self.sensor_names.extend(self.pressure_sensor_names)

        self.create_publishers()
        self.create_subscribers()

        self.addr = os.getenv('ROBOCUP_SIMULATOR_ADDR', '127.0.0.1:10001')

        self.socket = None
        self.first_run = True
        self.published_camera_info = False
        self.amcl_received = False
        rospy.Subscriber("/" + self.base_frame + '/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
        self.run()

    def receive_msg(self):
        msg_size = self.socket.recv(4)
        msg_size = struct.unpack(">L", msg_size)[0]

        data = bytearray()
        while len(data) < msg_size:
            packet = self.socket.recv(msg_size - len(data))
            if not packet:
                return None
            data.extend(packet)
        return data

    def run(self):
        while not rospy.is_shutdown():
            # Parse sensor
            try:
                if self.socket is None:
                    self.socket = self.get_connection(self.addr)

                msg = self.receive_msg()
                self.handle_sensor_measurements_msg(msg)

                sensor_time_steps = None
                if self.first_run:
                    sensor_time_steps = self.get_sensor_time_steps(active=True)
                self.send_actuator_requests(sensor_time_steps)
                self.first_run = False
                if not self.amcl_received:
                    odom_pub = rospy.Publisher("/" + self.base_frame + "/odom", Odometry, queue_size=50)

                    # since all odometry is 6DOF we'll need a quaternion created from yaw
                    odom_quat = tf.transformations.quaternion_from_euler(0, 0, 0)

                    # next, we'll publish the odometry message over ROS
                    odom = Odometry()
                    odom.header.stamp = self.stamp
                    odom.header.frame_id = self.base_frame + "/odom"

                    # set the position
                    odom.pose.pose = Pose(Point(0, 0, 0), Quaternion(*odom_quat))
                    odom.pose.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                                            0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
                    # set the velocity
                    odom.child_frame_id = self.base_frame + "/base_footprint"
                    odom.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
                    odom.twist.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                                             0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                                             0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                                             0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                                             0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                                             0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
                    # publish the message
                    odom_pub.publish(odom)
            except socket.timeout as s:
                rospy.logwarn_throttle(5, s)
                self.socket = None
                time.sleep(1)
            except ConnectionRefusedError as ex:
                rospy.logwarn_throttle(5, ex)
                self.socket = None
                time.sleep(1)
            except ConnectionResetError as ex:
                rospy.logwarn_throttle(5, ex)
                self.socket = None
                time.sleep(1)

        self.close_connection()

    def amcl_callback(self, data):
        self.amcl_received = True

    def create_publishers(self):
        self.pub_clock = rospy.Publisher('/clock', Clock, queue_size=1)
        self.pub_server_time_clock = rospy.Publisher('/server_time_clock', Clock, queue_size=1)
        self.pub_camera = rospy.Publisher('camera/image_raw', Image, queue_size=1)
        self.pub_camera_info = rospy.Publisher('camera/camera_info', CameraInfo, queue_size=1, latch=True)
        self.pub_imu = rospy.Publisher('imu_raw', Imu, queue_size=1)
        self.pressure_sensors_pub = {
            i: rospy.Publisher("foot_contact_{}".format(i), Bool, queue_size=10) for i in range(8)}
        self.pub_joint_states = rospy.Publisher('joint_states', JointState, queue_size=1)

    def create_subscribers(self):
        self.joint_command_subscriber = rospy.Subscriber("joint_command", JointState, self.joint_command_callback)

    def joint_command_callback(self, msg: JointState):
        motor_dictionary = {}
        for i in range(len(msg.name)):
            motor_dictionary[msg.name[i]] = msg.position[i]

        for i in range(len(self.external_motor_names)):
            if self.external_motor_names[i] in motor_dictionary:
                self.joint_command[i] = motor_dictionary[self.external_motor_names[i]]
            else:
                self.joint_command[i] = 0

    def get_connection(self, addr):
        host, port = addr.split(':')
        port = int(port)
        rospy.loginfo_throttle(5, f"Connecting to '{addr}'", logger_name="rc_api")
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((host, port))
        response = sock.recv(8).decode('utf8')
        if response == "Welcome\0":
            rospy.loginfo(f"Successfully connected to '{addr}'", logger_name="rc_api")
            return sock
        elif response == "Refused\0":
            rospy.logerr(f"Connection refused by '{addr}'", logger_name="rc_api")
            sys.exit(1)
        else:
            rospy.logerr(f"Could not connect to '{addr}'\nGot response '{response}'", logger_name="rc_api")
            sys.exit(1)

    def close_connection(self):
        if hasattr(self, 'socket') and self.socket is not None:
            self.socket.close()

    def handle_sensor_measurements_msg(self, msg):
        s_m = messages_pb2.SensorMeasurements()
        s_m.ParseFromString(msg)

        self.handle_time(s_m.time)
        self.handle_real_time(s_m.real_time)
        self.handle_messages(s_m.messages)
        self.handle_imu_data(s_m.accelerometers, s_m.gyros)
        self.handle_bumper_measurements(s_m.bumpers)
        self.handle_camera_measurements(s_m.cameras)
        self.handle_force_measurements(s_m.forces)
        self.handle_force3D_measurements(s_m.force3ds)
        self.handle_force6D_measurements(s_m.force6ds)
        self.handle_position_sensor_measurements(s_m.position_sensors)

    def handle_time(self, time):
        # time stamp at which the measurements were performed expressed in [ms]
        secs = time / 1000
        ros_time = rospy.Time.from_seconds(secs)
        self.stamp = ros_time
        msg = Clock()
        msg.clock.secs = ros_time.secs
        msg.clock.nsecs = ros_time.nsecs

        # Only used for local run no docker
        shared_strategy_source = os.getenv('SHARED_STRATEGY_SOURCE', True)
        if (shared_strategy_source == True and self.base_frame == 'robot1') or shared_strategy_source == False:
            self.pub_clock.publish(msg)


    def handle_real_time(self, time):
        # real unix time stamp at which the measurements were performed in [ms]
        msg = Clock()
        msg.clock.secs = time // 1000
        msg.clock.nsecs = (time % 1000) * 10 ** 6
        # if self.base_frame == 'robot1':
        self.pub_server_time_clock.publish(msg)

    def handle_messages(self, messages):
        for message in messages:
            text = message.text
            if message.message_type == messages_pb2.Message.ERROR_MESSAGE:
                rospy.logerr(f"RECEIVED ERROR: '{text}'", logger_name="rc_api")
            elif message.message_type == messages_pb2.Message.WARNING_MESSAGE:
                rospy.logwarn(f"RECEIVED WARNING: '{text}'", logger_name="rc_api")
            else:
                rospy.logwarn(f"RECEIVED UNKNOWN MESSAGE: '{text}'", logger_name="rc_api")

    def handle_imu_data(self, accelerometers, gyros):
        # IMU
        imu_msg = Imu()
        imu_msg.header.stamp = self.stamp
        imu_msg.header.frame_id = self.base_frame + "/imu_link"
        imu_msg.orientation.w = 1
        imu_accel = imu_gyro = False

        # Extract data from message
        for accelerometer in accelerometers:
            name = accelerometer.name
            value = accelerometer.value
            if name == "imu accelerometer":
                imu_accel = True
                imu_msg.linear_acceleration.x = ((value.X + 32768) / 65535) * (19.62 * 2) - 19.62
                imu_msg.linear_acceleration.y = ((value.Y + 32768) / 65535) * (19.62 * 2) - 19.62
                imu_msg.linear_acceleration.z = ((value.Z + 32768) / 65535) * (19.62 * 2) - 19.62

        for gyro in gyros:
            name = gyro.name
            value = gyro.value
            if name == "imu gyro":
                imu_gyro = True
                imu_msg.angular_velocity.x = ((value.X + 32768) / 65535) * (8.7266 * 2) - 8.7266
                imu_msg.angular_velocity.y = ((value.Y + 32768) / 65535) * (8.7266 * 2) - 8.7266
                imu_msg.angular_velocity.z = ((value.Z + 32768) / 65535) * (8.7266 * 2) - 8.7266

        if imu_accel and imu_gyro:
            self.pub_imu.publish(imu_msg)

    def handle_bumper_measurements(self, bumpers):
        # TODO
        pass
        # for bumper in bumpers:
        #     rospy.logwarn(f"Unknown bumper: '{bumper.name}'", logger_name="rc_api")

    def handle_camera_measurements(self, cameras):
        for camera in cameras:
            name = camera.name
            if name == "camera":
                width = camera.width
                height = camera.height
                quality = camera.quality  # 1 = raw image, 100 = no compression, 0 = high compression
                image = camera.image  # RAW or JPEG encoded data (note: JPEG is not yet implemented)

                if not self.published_camera_info:  # Publish CameraInfo once, it will be latched
                    self.publish_camera_info(height, width)
                    self.published_camera_info = True

                img_msg = Image()
                img_msg.header.stamp = self.stamp
                img_msg.header.frame_id = self.base_frame + "camera"
                img_msg.height = height
                img_msg.width = width
                img_msg.encoding = "bgr8"
                img_msg.step = 3 * width
                img_msg.data = image
                self.pub_camera.publish(img_msg)
            else:
                rospy.logerr(f"Unknown camera: '{name}'", logger_name="rc_api")

    def publish_camera_info(self, height, width):
        camera_info_msg = CameraInfo()
        camera_info_msg.header.stamp = self.stamp
        camera_info_msg.header.frame_id = self.base_frame + "camera"
        camera_info_msg.height = height
        camera_info_msg.width = width
        f_y = self.mat_from_fov_and_resolution(
            self.h_fov_to_v_fov(1.39626, height, width),
            height)
        f_x = self.mat_from_fov_and_resolution(1.39626, width)
        camera_info_msg.K = [f_x, 0, width / 2,
                             0, f_y, height / 2,
                             0, 0, 1]
        camera_info_msg.P = [f_x, 0, width / 2, 0,
                             0, f_y, height / 2, 0,
                             0, 0, 1, 0]
        self.pub_camera_info.publish(camera_info_msg)

    def mat_from_fov_and_resolution(self, fov, res):
        return 0.5 * res * (math.cos((fov / 2)) / math.sin((fov / 2)))

    def h_fov_to_v_fov(self, h_fov, height, width):
        return 2 * math.atan(math.tan(h_fov * 0.5) * (height / width))

    def handle_force_measurements(self, forces):
        for force in forces:
            rospy.logwarn(f"Unknown force measurement: '{force.name}'", logger_name="rc_api")

    def handle_force3D_measurements(self, force3ds):
        for force3d in force3ds:
            rospy.logwarn(f"Unknown force3d measurement: '{force3d.name}'", logger_name="rc_api")

    def handle_force6D_measurements(self, force6ds):
        for force6d in force6ds:
            rospy.logwarn(f"Unknown force6d measurement: '{force6d.name}'", logger_name="rc_api")

    def handle_position_sensor_measurements(self, position_sensors):
        state_msg = JointState()
        state_msg.header.stamp = self.stamp

        for position_sensor in position_sensors:
            state_msg.name.append(position_sensor.name[:len(position_sensor.name) - 7])
            state_msg.position.append(position_sensor.value)
        self.pub_joint_states.publish(state_msg)

    def get_sensor_time_steps(self, active=True):
        sensor_time_steps = []
        for sensor_name in self.sensor_names:
            time_step = self.MIN_CONTROL_STEP
            if sensor_name == "camera":
                time_step = self.MIN_FRAME_STEP
            if not active:
                time_step = 0
            sensor_time_step = messages_pb2.SensorTimeStep()
            sensor_time_step.name = sensor_name
            sensor_time_step.timeStep = time_step
            sensor_time_steps.append(sensor_time_step)
        return sensor_time_steps

    def send_actuator_requests(self, sensor_time_steps=None):
        actuator_requests = messages_pb2.ActuatorRequests()
        if sensor_time_steps is not None:
            actuator_requests.sensor_time_steps.extend(sensor_time_steps)

        for i, name in enumerate(self.motor_names):
            motor_position = messages_pb2.MotorPosition()
            motor_position.name = name
            assert (len(self.joint_command) == len(self.motor_names))
            motor_position.position = self.joint_command[i]
            actuator_requests.motor_positions.append(motor_position)

            if not (name in ["left_arm_motor_0 [shoulder]", "left_arm_motor_1", "right_arm_motor_0 [shoulder]", "right_arm_motor_1"]):
                # print(name)
                # print(not (name in ["left_arm_motor_0", "left_arm_motor_1", "right_arm_motor_0", "right_arm_motor_1"]))
                motor_pid = messages_pb2.MotorPID()
                motor_pid.name = name
                motor_pid.PID.X = 10
                motor_pid.PID.Y = 0.000
                motor_pid.PID.Z = 0.00
                actuator_requests.motor_pids.append(motor_pid)

            else:
                motor_pid = messages_pb2.MotorPID()
                motor_pid.name = name
                motor_pid.PID.X = 15
                motor_pid.PID.Y = 0.00
                motor_pid.PID.Z = 0
                actuator_requests.motor_pids.append(motor_pid)

        msg = actuator_requests.SerializeToString()
        msg_size = struct.pack(">L", len(msg))
        self.socket.send(msg_size + msg)


if __name__ == '__main__':
    GameControllerBridge()
