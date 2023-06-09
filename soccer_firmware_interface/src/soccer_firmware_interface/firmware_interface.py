import math
import threading

import rospy
import serial
from sensor_msgs.msg import Imu, JointState


class FirmwareInterface:
    def __init__(self):
        self.joint_command_subscriber = rospy.Subscriber("joint_command", JointState, self.joint_command_callback)
        self.joint_state_publisher = rospy.Publisher("joint_state", JointState)
        self.imu_publisher = rospy.Publisher("imu_raw", Imu)
        self.serial = None
        self.i = 0
        self.motor_mapping = rospy.get_param("motor_mapping")
        self.motor_id_to_name_dict = {self.motor_mapping[m]["id"]: m for m in self.motor_mapping}

        self.motor_types = rospy.get_param("motor_types")

        # Start the thread
        serial_thread = threading.Thread(target=self.firmware_update_loop)
        serial_thread.start()

        pass

    def reconnect_serial_port(self):
        if self.serial is None:
            # todo: loop through ACMs see which one connects
            self.i = (self.i + 1) % 1 + 1
            self.serial = serial.Serial(f"/dev/ttyACM{self.i}")

    def firmware_update_loop(self):
        while not rospy.is_shutdown():
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
                data_l = self.serial.read()
                data_h = self.serial.read()
                angle = data_l[0] | (data_h[0] << 8)
                print(data_h[0], data_l[0], angle)
                continue

                # Publish the Joint State
                data = []
                j = JointState()
                j.header.stamp = rospy.Time.now()
                for i in range(18):
                    val = data[i * 2 + 2] | data[i * 2 + 3] << 8

                    motor_name = self.motor_id_to_name_dict[i]
                    # TODO Nam calculation val to radians
                    position_radians = 0
                    j.name.append(motor_name)
                    j.position.append(position_radians)
                self.joint_state_publisher.publish(j)

                imu = Imu()
                imu.header.stamp = rospy.Time.now()
                imu.linear_acceleration.x = 0  # TODO nam fill out
                imu.linear_acceleration.y = 0
                imu.linear_acceleration.z = 0
                imu.angular_velocity.x = 0
                imu.angular_velocity.y = 0
                imu.angular_velocity.z = 0
                imu.orientation.x = 0
                imu.orientation.y = 0
                imu.orientation.z = 0
                imu.orientation.w = 0
                self.imu_publisher.publish(imu)

            except Exception as ex:
                rospy.logerr_throttle(10, f"Lost connection to serial port {ex}, retrying...")
                pass

    def joint_command_callback(self, joint_state: JointState):
        try:
            self.reconnect_serial_port()

            bytes_to_write = [0xFF] * (18 * 2 + 2)

            for name, angle in zip(joint_state.name, joint_state.position):
                id = self.motor_mapping[name]["id"]
                type = self.motor_mapping[name]["type"]
                angle_zero = self.motor_mapping[name]["angle_zero"] / 180 * math.pi
                angle_max = self.motor_mapping[name]["angle_max"] / 180 * math.pi
                angle_min = self.motor_mapping[name]["angle_min"] / 180 * math.pi

                angle_final = max(min(angle + angle_zero, angle_max), angle_min)

                max_angle_bytes = self.motor_types[type]["max_angle_bytes"]
                max_angle_radians = self.motor_types[type]["max_angle_degrees"] / 180 * math.pi
                assert angle_final >= 0
                assert angle_final <= max_angle_radians

                angle_final_bytes = round(angle_final / max_angle_radians * max_angle_bytes)
                angle_final_bytes_2 = (angle_final_bytes >> 8) & 0xFF
                angle_final_bytes_1 = angle_final_bytes & 0xFF

                assert id < 18
                bytes_to_write[2 + id * 2] = angle_final_bytes_1
                bytes_to_write[2 + id * 2 + 1] = angle_final_bytes_2
                pass

            self.serial.write(bytes_to_write)

        except Exception as ex:
            rospy.logerr_throttle(10, f"Lost connection to serial port {ex}, retrying...")
            pass
