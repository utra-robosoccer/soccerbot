#!/usr/bin/env python3
import math
import os
import socket
import struct
import time

import rclpy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState
from std_msgs.msg import Bool

# if "ROS_NAMESPACE" not in os.environ:
#     os.environ["ROS_NAMESPACE"] = "/robot1"
from soccer_webots import messages_pb2


class GameControllerBridge:
    def __init__(self):
        self.init_node("game_controller_bridge")
        robot_name = os.environ["ROS_NAMESPACE"].replace("/", "")
        self.base_frame = robot_name
        self.MIN_FRAME_STEP = 16  # ms
        self.MIN_CONTROL_STEP = 8  # ms
        self.joint_command = JointState()
        self.motor_names = [
            "left_shoulder_pitch",
            "left_elbow",
            "right_shoulder_pitch",
            "right_elbow",
            "right_hip_yaw",
            "right_hip_roll",
            "right_hip_pitch",
            "right_knee",
            "right_ankle_pitch",
            "right_ankle_roll",
            "left_hip_yaw",
            "left_hip_roll",
            "left_hip_pitch",
            "left_knee",
            "left_ankle_pitch",
            "left_ankle_roll",
            "head_yaw",
            "head_pitch",
        ]
        self.motor_name_to_proto_name_exception_mapping = {
            "left_shoulder_pitch": "left_shoulder_pitch [shoulder]",
            "right_shoulder_pitch": "right_shoulder_pitch [shoulder]",
            "left_hip_roll": "left_hip_roll [hip]",
            "right_hip_roll": "right_hip_roll [hip]",
        }

        self.motor_count = len(self.motor_names)

        self.sensor_names = [s + "_sensor" for s in self.motor_names]
        self.regular_sensor_names = ["imu accelerometer", "imu gyro", "imu accelerometer", "camera"]
        self.sensor_names.extend(self.regular_sensor_names)
        self.pressure_sensor_names = [
            "right_leg_foot_sensor_1",
            "right_leg_foot_sensor_2",
            "right_leg_foot_sensor_3",
            "right_leg_foot_sensor_4",
            "left_leg_foot_sensor_1",
            "left_leg_foot_sensor_2",
            "left_leg_foot_sensor_3",
            "left_leg_foot_sensor_4",
        ]
        self.sensor_names.extend(self.pressure_sensor_names)

        self.pub_clock = self.create_publisher("/clock", Clock, queue_size=1)
        self.pub_server_time_clock = self.create_publisher("/server_time_clock", Clock, queue_size=1)
        self.pub_camera = self.create_publisher("camera/image_raw", Image, queue_size=1)
        self.pub_camera_info = self.create_publisher("camera/camera_info", CameraInfo, queue_size=1, latch=True)
        self.pub_imu = self.create_publisher("imu_raw", Imu, queue_size=1)
        self.pub_imu_first = 2
        self.pressure_sensors_pub = {i: self.create_publisher("foot_contact_{}".format(i), Bool, queue_size=10) for i in range(8)}
        self.pub_joint_states = self.create_publisher("joint_states", JointState, queue_size=1)
        self.joint_command_create_subscription = self.create_subscription("joint_command", JointState, self.joint_command_callback)

        self.addr = os.getenv("ROBOCUP_SIMULATOR_ADDR", "127.0.0.1:10001")

        self.socket = None
        self.first_run = True
        self.published_camera_info = False
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
        while not self.is_shutdown():
            # Parse sensor
            try:
                if self.socket is None:
                    self.socket = self.get_connection(self.addr)
                    if self.socket is None:
                        time.sleep(8)
                        continue

                msg = self.receive_msg()
                self.handle_sensor_measurements_msg(msg)

                sensor_time_steps = None
                if self.first_run:
                    sensor_time_steps = self.get_sensor_time_steps(active=True)
                self.send_actuator_requests(sensor_time_steps)
                self.first_run = False

            except socket.timeout as s:
                print(s)
                self.socket = None
                time.sleep(6)
            except ConnectionRefusedError as ex:
                print(ex)
                self.socket = None
                time.sleep(6)
            except ConnectionResetError as ex:
                print(ex)
                self.socket = None
                time.sleep(6)
            except socket.gaierror as ex:
                print(ex)
                self.socket = None
                time.sleep(6)

        self.close_connection()

    def joint_command_callback(self, msg: JointState):
        self.joint_command = msg

    def get_connection(self, addr):
        host, port = addr.split(":")
        port = int(port)
        print(f"Connecting to '{addr}'")
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((host, port))
        response = sock.recv(8).decode("utf8")
        if response == "Welcome\0":
            print(f"\033[96mConnected to Simulator at '{addr}'\033[0m")
            return sock
        elif response == "Refused\0":
            print(f"Connection refused by '{addr}'")
            return None
        else:
            print(f"Could not connect to '{addr}'\nGot response '{response}'", logger_name="rc_api")
            return None

    def close_connection(self):
        if hasattr(self, "socket") and self.socket is not None:
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
        ros_time = self.Time.from_seconds(secs)
        self.stamp = ros_time
        msg = Clock()
        msg.clock.secs = ros_time.secs
        msg.clock.nsecs = ros_time.nsecs

        self.pub_clock.publish(msg)

    def handle_real_time(self, time):
        # real unix time stamp at which the measurements were performed in [ms]
        msg = Clock()
        msg.clock.secs = time // 1000
        msg.clock.nsecs = (time % 1000) * 10**6
        self.pub_server_time_clock.publish(msg)

    def handle_messages(self, messages):
        for message in messages:
            text = message.text
            if message.message_type == messages_pb2.Message.ERROR_MESSAGE:
                self.get_logger().error(f"RECEIVED ERROR: '{text}'", logger_name="rc_api")
            elif message.message_type == messages_pb2.Message.WARNING_MESSAGE:
                self.logwarn(f"RECEIVED WARNING: '{text}'", logger_name="rc_api")
            else:
                self.logwarn(f"RECEIVED UNKNOWN MESSAGE: '{text}'", logger_name="rc_api")

    def handle_imu_data(self, accelerometers, gyros):
        # IMU
        imu_msg = Imu()
        imu_msg.header.stamp = self.stamp
        imu_msg.header.frame_id = self.base_frame + "/imu_link"
        imu_msg.orientation.w = 1
        imu_accel = imu_gyro = False

        # Data obtained from https://docs.google.com/document/d/1a-wFkxcSyTkWRwb_al38hqDmtMBojVwZ-Ql2d1uYoKo/edit#

        # Extract data from message
        for accelerometer in accelerometers:
            name = accelerometer.name
            value = accelerometer.value
            if name == "imu accelerometer":
                imu_accel = True
                imu_msg.linear_acceleration.x = ((value.X + 32768) / 65535) * (19.62 * 2) - 19.62
                imu_msg.linear_acceleration.y = ((value.Y + 32768) / 65535) * (19.62 * 2) - 19.62
                imu_msg.linear_acceleration.z = ((value.Z + 32768) / 65535) * (19.62 * 2) - 19.62

                # fmt: off
                imu_msg.linear_acceleration_covariance = [4.06 ** 2, 0, 0,
                                                          0, 4.06 ** 2, 0,
                                                          0, 0, 4.06 ** 2]
                # fmt: on

        for gyro in gyros:
            name = gyro.name
            value = gyro.value
            if name == "imu gyro":
                imu_gyro = True
                imu_msg.angular_velocity.x = ((value.X + 32768) / 65535) * (8.7266 * 2) - 8.7266
                imu_msg.angular_velocity.y = ((value.Y + 32768) / 65535) * (8.7266 * 2) - 8.7266
                imu_msg.angular_velocity.z = ((value.Z + 32768) / 65535) * (8.7266 * 2) - 8.7266

                # fmt: off
                imu_msg.angular_velocity_covariance = [0.402 ** 2, 0, 0,
                                                       0, 0.402 ** 2, 0,
                                                       0, 0, 0.402 ** 2]

                # fmt: on

        if self.pub_imu_first > 0:
            if imu_msg.linear_acceleration.z > 10 or imu_msg.linear_acceleration.z < 8:
                self.pub_imu_first = 2
            else:
                self.pub_imu_first -= 1
            return

        if imu_accel and imu_gyro:
            self.pub_imu.publish(imu_msg)

    def handle_bumper_measurements(self, bumpers):
        # TODO
        pass
        # for bumper in bumpers:
        #     self.logwarn(f"Unknown bumper: '{bumper.name}'", logger_name="rc_api")

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
                img_msg.header.frame_id = self.base_frame + "/camera"
                img_msg.height = height
                img_msg.width = width
                img_msg.encoding = "bgr8"
                img_msg.step = 3 * width
                img_msg.data = image
                self.pub_camera.publish(img_msg)
            else:
                self.get_logger().error(f"Unknown camera: '{name}'", logger_name="rc_api")

    def publish_camera_info(self, height, width):
        camera_info_msg = CameraInfo()
        camera_info_msg.header.stamp = self.stamp
        camera_info_msg.header.frame_id = self.base_frame + "camera"
        camera_info_msg.height = height
        camera_info_msg.width = width
        f_y = self.mat_from_fov_and_resolution(self.h_fov_to_v_fov(1.39626, height, width), height)
        f_x = self.mat_from_fov_and_resolution(1.39626, width)
        camera_info_msg.K = [f_x, 0, width / 2, 0, f_y, height / 2, 0, 0, 1]
        camera_info_msg.P = [f_x, 0, width / 2, 0, 0, f_y, height / 2, 0, 0, 0, 1, 0]
        self.pub_camera_info.publish(camera_info_msg)

    def mat_from_fov_and_resolution(self, fov, res):
        return 0.5 * res * (math.cos((fov / 2)) / math.sin((fov / 2)))

    def h_fov_to_v_fov(self, h_fov, height, width):
        return 2 * math.atan(math.tan(h_fov * 0.5) * (height / width))

    def handle_force_measurements(self, forces):
        for force in forces:
            self.logwarn(f"Unknown force measurement: '{force.name}'", logger_name="rc_api")

    def handle_force3D_measurements(self, force3ds):
        for force3d in force3ds:
            self.logwarn(f"Unknown force3d measurement: '{force3d.name}'", logger_name="rc_api")

    def handle_force6D_measurements(self, force6ds):
        for force6d in force6ds:
            self.logwarn(f"Unknown force6d measurement: '{force6d.name}'", logger_name="rc_api")

    def handle_position_sensor_measurements(self, position_sensors):
        state_msg = JointState()
        state_msg.header.stamp = self.stamp

        for position_sensor in position_sensors:
            state_msg.name.append(position_sensor.name[: len(position_sensor.name) - 7])
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

        for name, position in zip(self.joint_command.name, self.joint_command.position):
            motor_position = messages_pb2.MotorPosition()
            name = self.motor_name_to_proto_name_exception_mapping.get(name, name)
            motor_position.name = name
            motor_position.position = position
            actuator_requests.motor_positions.append(motor_position)

            motor_pid = messages_pb2.MotorPID()
            motor_pid.name = name
            if "arm" in name:
                motor_pid.PID.X = 15
                motor_pid.PID.Y = 0.000
                motor_pid.PID.Z = 0.00
            else:
                motor_pid.PID.X = 10
                motor_pid.PID.Y = 0.000
                motor_pid.PID.Z = 0.00
            actuator_requests.motor_pids.append(motor_pid)

        msg = actuator_requests.SerializeToString()
        msg_size = struct.pack(">L", len(msg))
        self.socket.send(msg_size + msg)


if __name__ == "__main__":
    GameControllerBridge()
