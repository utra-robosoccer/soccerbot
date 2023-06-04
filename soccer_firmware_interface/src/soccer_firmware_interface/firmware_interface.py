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

        self.motor_mapping = rospy.get_param("motor_mapping")
        self.motor_types = rospy.get_param("motor_types")

        # Start the thread
        serial_thread = threading.Thread(target=self.firmware_update_loop)
        serial_thread.start()

        pass

    def reconnect_serial_port(self):
        if self.serial is None:
            # todo: loop through ACMs see which one connects
            self.serial = serial.Serial("/dev/ttyACM1")

    def firmware_update_loop(self):
        while not rospy.is_shutdown():
            try:
                self.reconnect_serial_port()

                data = self.serial.read()
                # print(data)

                # TODO imu_publisher.publish(imu)
                # TODO joint_state_publisher.publish(joint_state)

            except Exception as ex:
                # TODO retry connection
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
            # todo: properly catch error, if unplug/replug cable should still work
            print(ex)
            pass
