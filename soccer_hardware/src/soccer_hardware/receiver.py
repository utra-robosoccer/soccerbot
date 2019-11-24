# receiver.py

import time
import struct
from transformations import *

try:
    from soccer_msgs.msg import RobotState
    from sensor_msgs.msg import Imu
    from geometry_msgs.msg import Vector3
except ImportError:
    pass


class Receiver:
    def __init__(self, ser, ros_is_on):
        self.ser = ser
        self.ros_is_on = ros_is_on
        self.num_receptions = 0

    def change_port(self, ser):
        self.ser = ser

    def decode(self, raw):
        """ Decodes raw bytes received from the microcontroller. As per the agreed
            upon protocol, the first 4 bytes are for a header while the remaining
            80 bytes contain floats for each motor.
        """
        motors = list()
        imu = list()

        for i in range(12):
            # Here, we only unpack for 12 motors since that's all we have connected
            # in our current setup
            motors.append(struct.unpack('<f', raw[4 + i * 4:8 + i * 4])[0])
        for i in range(6):
            # Unpack IMU Data
            imu.append(struct.unpack('<f', raw[52 + i * 4: 56 + i * 4])[0])
        return (motors, imu)

    def receive_packet_from_mcu(self):
        """
        Receives 80 bytes of the MCU provided that there is a valid 4-byte
        header attached to the front. Returns the list of data interpreted as
        32-bit floats.
        """

        receive_succeeded = False

        totalBytesRead = 0
        startSeqCount = 0
        buff = bytes(''.encode())

        timeout = 0.01  # 10 ms timeout
        time_start = time.time()
        time_curr = time_start

        num_bytes_available = 0
        while True:
            # First, we wait until we have received some data, or until the timeout
            # has elapsed
            while (num_bytes_available == 0) and (time_curr - time_start < timeout):
                time.sleep(0.001)
                time_curr = time.time()
                num_bytes_available = self.ser.in_waiting

            if (num_bytes_available == 0) and (time_curr - time_start >= timeout):
                break
            else:
                # If we receive some data, we process it here then go back to
                # waiting for more
                rawData = self.ser.read(num_bytes_available)
                for i in range(num_bytes_available):
                    if startSeqCount == 4:
                        buff = buff + rawData[i:i + 1]
                        totalBytesRead = totalBytesRead + 1

                        if totalBytesRead == 84:
                            # If we get here, we have received a full packet
                            receive_succeeded = True
                            break
                    else:
                        if struct.unpack('<B', rawData[i:i + 1])[0] == 0xFF:
                            startSeqCount = startSeqCount + 1
                        else:
                            startSeqCount = 0
                num_bytes_available = 0
                if receive_succeeded:
                    break
        return receive_succeeded, buff

    def publish_sensor_data(self):
        # IMU FEEDBACK
        imu = Imu()
        vec1 = Vector3(-self.received_imu[2][0], self.received_imu[1][0], self.received_imu[0][0])
        imu.angular_velocity = vec1
        vec2 = Vector3(self.received_imu[5][0], self.received_imu[4][0], self.received_imu[3][0])
        imu.linear_acceleration = vec2
        self.pub.publish(imu)

        # MOTOR FEEDBACK
        # Convert motor array from the embedded coordinate system to that
        # used by controls
        ctrlAngleArray = mcuToCtrlAngles(self.received_angles)

        robotState = RobotState()
        for i in range(12):
            robotState.joint_angles[i] = ctrlAngleArray[i][0]

        # Convert motor array from the embedded order and sign convention
        # to that used by controls
        m = getCtrlToMcuAngleMap()
        robotState.joint_angles[0:12] = np.linalg.inv(m).dot(robotState.joint_angles[0:18])[0:12]

        self.pub2.publish(robotState)

    def receive(self):
        (receive_succeeded, buff) = self.receive_packet_from_mcu()
        if receive_succeeded:
            # If our reception was successful, we update the class variables
            # for the received angles and received IMU data. Otherwise, we
            # just send back the last values we got successfully
            (recvAngles, recvIMUData) = self.decode(buff)

            angleArray = np.array(recvAngles)
            self.received_angles = angleArray[:, np.newaxis]
            self.received_imu = np.array(recvIMUData).reshape((6, 1))
            self.num_receptions = self.num_receptions + 1

            # Only publish to ROS on successful receive
            if self.ros_is_on == True:
                self.publish_sensor_data()
