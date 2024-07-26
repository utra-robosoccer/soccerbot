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

        self.pub_all_motor = rospy.Publisher("joint_command", JointState, queue_size=1)

    def updateRobotConfiguration(self) -> None:
        """
        Reads the joint_states message and resets all the positions of all the joints
        """

        self.configuration_offset = [0] * self.numb_of_motors
        try:
            joint_state = rospy.wait_for_message("joint_states", JointState, timeout=3)
            indexes = [joint_state.name.index(motor_name) for motor_name in self.motor_names]
            self.configuration[0 : self.numb_of_motors] = [joint_state.position[i] for i in indexes]
        except (ROSException, KeyError, AttributeError) as ex:
            rospy.logerr(ex)
        except ValueError as ex:
            print(ex)
            rospy.logerr("Not all joint states are reported, cable disconnect?")
            rospy.logerr("Joint States")
            rospy.logerr(joint_state)
            rospy.logerr("Motor Names")
            print(self.motor_names)
            self.configuration[0 : self.numb_of_motors] = [0] * self.numb_of_motors

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
