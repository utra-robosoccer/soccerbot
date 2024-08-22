import rospy
from rospy import ROSException
from sensor_msgs.msg import JointState
from soccer_pycontrol.model.motor_control import MotorControl


class MotorControlROS(MotorControl):
    def __init__(self, motor_names):
        self.motor_names = motor_names
        self.numb_of_motors = len(self.motor_names)
        # Todo make it numpy and add getter and setters
        self.configuration = [0.0] * self.numb_of_motors
        self.configuration_offset = [0.0] * self.numb_of_motors
        # TODO should separate config to current and target
        self.motor_names = motor_names
        # TODo fix namespace
        self.pub_all_motor = rospy.Publisher("/robot1/joint_command", JointState, queue_size=1)

    def set_motor(self) -> None:
        """
        Send the robot angles based on self.configuration + self.configuration_offset to ROS
        """
        js = JointState()
        js.name = []
        js.header.stamp = rospy.Time.now()
        js.position = []
        js.effort = []
        angles = self.get_angles()
        for i, n in enumerate(self.motor_names):
            js.name.append(n)
            js.position.append(angles[i])
        try:
            rospy.loginfo_once("Started Publishing Motors")
            self.pub_all_motor.publish(js)
        except rospy.exceptions.ROSException as ex:
            print(ex)
            exit(0)
