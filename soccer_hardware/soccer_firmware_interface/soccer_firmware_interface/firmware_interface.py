import math
import os
import threading
import time

import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import scipy
import serial
import yaml
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu, JointState


class FirmwareInterface(Node):
    def __init__(self, mtp:str="", mmp:str="" ):
        Node.__init__(self, "firmware_interface")

        self.joint_command_create_subscription = self.create_subscription(JointState,"joint_command",  self.joint_command_callback, qos_profile=10)
        self.joint_state_create_publisher = self.create_publisher(JointState,"joint_states", qos_profile=10)
        self.imu_create_publisher = self.create_publisher(Imu, "imu_raw", qos_profile=10)
        self.serial = None

        self.declare_parameters(
            namespace="", parameters=[("motor_types",mtp), ("motor_mapping",mmp)]
        )

        motor_types_path = self.get_parameter('motor_types').get_parameter_value().string_value
        motor_mapping_path = self.get_parameter('motor_mapping').get_parameter_value().string_value

        self.motor_types = {}
        self.motor_mapping = {}
        self.motor_id_to_name_dict = {}

        with open(motor_types_path) as f:
            self.motor_types = yaml.safe_load(f)

        with open(motor_mapping_path) as f:
            self.motor_mapping = yaml.safe_load(f)

        self.motor_id_to_name_dict = {self.motor_mapping[m]["id"]: m for m in self.motor_mapping}

        self.last_motor_publish_time = self.get_clock().now()
        self.last_motor_publish_time_real = self.get_clock().now()

        # self._IMU_FILT_B = np.array(
        #     [0.030738841, 0.048424201, 0.083829062, 0.11125669, 0.13424691, 0.14013315, 0.13424691, 0.11125669, 0.083829062, 0.048424201, 0.030738841]
        # )  # from embedded code
        # self._imu_filt_zi = np.zeros((len(self._IMU_FILT_B) - 1, 3))

        # Start the thread
        self._lock = threading.Lock()
        self._read_lock = threading.Lock()

        serial_thread = threading.Thread(target=self.firmware_update_loop)
        serial_thread.start()


        # self._timer = self.create_timer(1 / 200, self.firmware_update_loop)

    def reconnect_serial_port(self):
        if self.serial is None:
            # todo: loop through ACMs see which one connects
            # for debug assume ACM0 is STLINK and ACM1 is our PCB
            for i in range(0, 10):
                self.get_logger().info( f"Trying connection to /dev/ttyACM{i}",  throttle_duration_sec=1)
                if os.path.exists(f"/dev/ttyACM{i}"):
                    self.serial = serial.Serial(f"/dev/ttyACM{i}")
                    self.get_logger().info( f"connected to: /dev/ttyACM{i}", throttle_duration_sec=1)
                    return
            self.get_logger().error("Unable to connect to any serial port /dev/ttyACM0-10", throttle_duration_sec=1)



    def firmware_update_loop(self):
        while rclpy.ok():
            with self._read_lock:
                try:

                    self.reconnect_serial_port()

                    # data format
                    # FF
                    # (Raw data motor 0)            2 bytes
                    # ...
                    # (Raw data motor 17)           2 bytes
                    # Acceleration X
                    # Acceleration Y
                    # Acceleration Z
                    # Gyro X
                    # Gyro Y
                    # Gyro Z
                    # IMU yaw (rad)                 2 bytes
                    # IMU pitch (rad)               2 bytes
                    # IMU roll (rad)                2 bytes
                    # (optional CRC)   1 byte
                    # data_l = self.serial.read()
                    # data_h = self.serial.read()
                    # angle = data_l[0] | (data_h[0] << 8)
                    # print(data_h[0], data_l[0], angle)

                    data = self.serial.read(size=2 + 2 * 20 + 12)
                    # s = time.time()
                    # data = self.serial.read(size=2 + 2 * 20 + 12)
                    # print(1.0 / (time.time() - s))
                    self.pub_joint_state(data)

                    self.pub_imu(data)
                except Exception as ex:
                    self.get_logger().error( f"Lost connection to serial port {type(ex)} {ex}, retrying...", throttle_duration_sec=1)
                    pass



    def pub_joint_state(self, data):
        # Publish the Joint State
        j = JointState()
        j.header.stamp = self.get_clock().now().to_msg()
        for i in range(20):
            val = data[i * 2 + 2] | data[i * 2 + 3] << 8

            motor_name = self.motor_id_to_name_dict[i]
            motor_type = self.motor_mapping[motor_name]["type"]
            motor_angle_zero = self.motor_mapping[motor_name]["angle_zero"]
            max_angle_bytes = self.motor_types[motor_type]["max_angle_bytes"]
            max_angle_radians = self.motor_types[motor_type]["max_angle_degrees"] / 180 * math.pi

            motor_angle_zero_radian = motor_angle_zero / 180 * math.pi

            motor_angle_radian = (val / max_angle_bytes) * max_angle_radians - motor_angle_zero_radian
            flipped = "flipped" in self.motor_mapping[motor_name] and self.motor_mapping[motor_name][
                "flipped"] == "true"
            if flipped:
                motor_angle_radian = -motor_angle_radian
            j.name.append(motor_name)

            j.position.append(motor_angle_radian)
        self.joint_state_create_publisher.publish(j)

    def pub_imu(self, data):
        imu = Imu()

        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = "imu_link"
        imu_data = data[2 + 2 * 20: 2 + 2 * 20 + 12]

        # https://www.mouser.com/datasheet/2/783/BST_BMI088_DS001-1509549.pdf
        ACC_RANGE = 32768.0 / 2.0 / 1.5  # page 27 datasheet bmi088
        IMU_GY_RANGE = 32768.0 / (1000.0 * math.pi / 180)  # page 39 datasheet bmi088
        # ACC_RANGE = 32768.0 / 2.0  # page 22 datasheet bmi085
        # IMU_GY_RANGE = 32768.0 / 1000.0  # page 27 datasheet bmi085
        G = 9.81

        ax = int.from_bytes(imu_data[0:2], byteorder="big", signed=True) / ACC_RANGE * G
        ay = int.from_bytes(imu_data[2:4], byteorder="big", signed=True) / ACC_RANGE * G
        az = int.from_bytes(imu_data[4:6], byteorder="big", signed=True) / ACC_RANGE * G

        # ax, ay = ay, ax  # flip pitch and roll

        imu.linear_acceleration.x = az
        imu.linear_acceleration.y = ax
        imu.linear_acceleration.z = ay

        vx = int.from_bytes(imu_data[6:8], byteorder="big", signed=True) / IMU_GY_RANGE
        vy = int.from_bytes(imu_data[8:10], byteorder="big", signed=True) / IMU_GY_RANGE
        vz = int.from_bytes(imu_data[10:12], byteorder="big", signed=True) / IMU_GY_RANGE

        # vx, vy = vy, vx  # flip pitch and roll

        # print(f"a {ax} {ay} {az}")
        # print(f"v {vx:10.3f} {vy:10.3f} {vz:10.3f}")

        imu.angular_velocity.x = vz
        imu.angular_velocity.y = -vx
        imu.angular_velocity.z = vy
        # print(imu)
        self.imu_create_publisher.publish(imu)

    def joint_command_callback(self, joint_state: JointState):
        with self._lock:

            try:
                # if self.last_motor_publish_time is not None:
                #     time_diff_message_in = joint_state.header.stamp - self.last_motor_publish_time
                #     time_diff_real = self.get_clock().now() - self.last_motor_publish_time_real
                #
                #     time_lag = self.get_clock().now() - joint_state.header.stamp
                #     if time_lag > time_diff_message_in:
                #         print(f"Message Skipped Time Lag {time_lag}")
                #         self.last_motor_publish_time = joint_state.header.stamp
                #         return
                #
                #     if time_diff_real < time_diff_message_in:
                #         self.sleep(time_diff_message_in - time_diff_real)

                t1 = time.time()

                self.reconnect_serial_port()


                # [0xff, 0xff, angle1_lo, angle1_hi, angle2_lo ..., crc_lo, crc_hi]
                bytes_to_write = [0x0] * (2 + 20 * 2)
                bytes_to_write[0] = 0xFF
                bytes_to_write[1] = 0xFF

                for name, angle in zip(joint_state.name, joint_state.position):
                    id = self.motor_mapping[name]["id"]
                    type = self.motor_mapping[name]["type"]
                    angle_zero = self.motor_mapping[name]["angle_zero"] / 180 * math.pi
                    angle_max = self.motor_mapping[name]["angle_max"] / 180 * math.pi
                    angle_min = self.motor_mapping[name]["angle_min"] / 180 * math.pi
                    ang_zero_deg = self.motor_mapping[name]["angle_zero"]

                    flipped = "flipped" in self.motor_mapping[name] and self.motor_mapping[name]["flipped"] == "true"
                    if flipped:
                        angle = -angle

                    angle_final = max(min(angle + angle_zero, angle_max), angle_min)

                    max_angle_bytes = self.motor_types[type]["max_angle_bytes"]
                    max_angle_radians = self.motor_types[type]["max_angle_degrees"] / 180 * math.pi
                    assert angle_final >= 0
                    assert angle_final <= max_angle_radians

                    angle_final_bytes = round(angle_final / max_angle_radians * max_angle_bytes)
                    angle_final_bytes_1 = angle_final_bytes & 0xFF
                    angle_final_bytes_2 = (angle_final_bytes >> 8) & 0xFF

                    assert id < 20
                    bytes_to_write[2 + id * 2] = angle_final_bytes_1
                    bytes_to_write[2 + id * 2 + 1] = angle_final_bytes_2
                    pass

                t2 = time.time()

                self.serial.write(bytes_to_write)
                self.last_motor_publish_time_real = self.get_clock().now()
                self.last_motor_publish_time = joint_state.header.stamp
                t3 = time.time()
                self.get_logger().info(
                    f"Time Lag : {(self.get_clock().now().to_msg().nanosec - joint_state.header.stamp.nanosec)}  Bytes Written: {bytes_to_write} Time Take {t2-t1} {t3-t1}"
                ,throttle_duration_sec=1)

            except Exception as ex:
                self.get_logger().error( f"Lost connection to serial port {ex}, retrying...", throttle_duration_sec=1)
                self.serial = None
                pass



def main():
    rclpy.init()
    node = FirmwareInterface()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
