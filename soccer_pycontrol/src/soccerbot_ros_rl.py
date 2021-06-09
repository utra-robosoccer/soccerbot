import rospy
import numpy as np
import pybullet as pb
from sensor_msgs.msg import JointState, Imu

from soccerbot import Joints
from soccerbot_ros import SoccerbotRos

class SoccerbotRosRl(SoccerbotRos):

    def __init__(self, pose, useFixedBase=False):
        super().__init__(pose, useFixedBase)
        self.velocity_configuration = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.jointState = None

    def ready(self):
        """
        Sets the robot's joint angles for the robot to standing pose.
        :return: None
        """
        # Used later to calculate inverse kinematics
        position = self.pose.get_position()
        position[2] = self.hip_to_torso[2, 3] + self.walking_hip_height
        self.pose.set_position(position)

        # Set initial posit
        self.configuration[Joints.RIGHT_LEG_1] = 0.0
        self.configuration[Joints.RIGHT_LEG_2] = 0.05
        self.configuration[Joints.RIGHT_LEG_3] = 0.4
        self.configuration[Joints.RIGHT_LEG_4] = -0.8
        self.configuration[Joints.RIGHT_LEG_5] = 0.4
        self.configuration[Joints.RIGHT_LEG_6] = -0.05

        self.configuration[Joints.LEFT_LEG_1] = 0.0
        self.configuration[Joints.LEFT_LEG_2] = 0.05
        self.configuration[Joints.LEFT_LEG_3] = 0.4
        self.configuration[Joints.LEFT_LEG_4] = -0.8
        self.configuration[Joints.LEFT_LEG_5] = 0.4
        self.configuration[Joints.LEFT_LEG_6] = -0.05

        self.configuration[Joints.LEFT_ARM_1] = -0.5
        self.configuration[Joints.LEFT_ARM_2] = 2.8
        self.configuration[Joints.RIGHT_ARM_1] = -0.5
        self.configuration[Joints.RIGHT_ARM_2] = 2.8


        pb.setJointMotorControlArray(bodyIndex=self.body,
                                     controlMode=pb.POSITION_CONTROL,
                                     jointIndices=list(range(0, 20, 1)),
                                     targetPositions=self.get_angles(),
                                     forces=self.max_forces)
        # Publish angles using position control
        super().publishAngles()

    def jointStateCallback(self, jointState: JointState):
        self.jointState = jointState
        pass

    def imu_callback(self, imu: Imu):
        self.imu_msg = imu

    def getObservationVector(self):
        positions = self.jointState.position # Array of floats
        velocities = self.jointState.velocity # Array of floats
        imu_angvel = [self.imu_msg.angular_velocity.x, self.imu_msg.angular_velocity.y, self.imu_msg.angular_velocity.z] # IMU angular velocity
        imu_linacc = [self.imu_msg.linear_acceleration.x, self.imu_msg.linear_acceleration.y, self.imu_msg.linear_acceleration.z] # IMU linear acceleration
        imu_orientation = [self.imu_msg.orientation.x, self.imu_msg.orientation.y, self.imu_msg.orientation.z, self.imu_msg.orientation.w] # IMU orientation
        feet_pressure_sensors = self.foot_pressure_values
        orientation_vector = self.getDirectionVector()

        # Concatenate vector
        return np.concatenate((positions, velocities, imu_angvel, imu_linacc, imu_orientation, orientation_vector, feet_pressure_sensors))

    def getDirectionVector(self):
        current = self.pose
        end = self.goal_position

        distance_unit_vec = current.get_position()[0:2] - end.get_position()[0:2]
        distance_unit_vec /= np.linalg.norm(distance_unit_vec)
        mat = current[0:3, 0:3]
        d2_vect = np.array([mat[0], mat[3]], dtype=np.float32)
        d2_vect /= np.linalg.norm(d2_vect)
        cos = np.dot(d2_vect, distance_unit_vec)
        sin = np.linalg.norm(np.cross(distance_unit_vec, d2_vect))
        vec = np.array([cos, sin], dtype=np.float32)
        return vec

    def setGoal(self, finishPosition):
        """
        Returns the trajectories for the robot's feet and crotch. The coordinates x,y will be used only.
        :param finishPosition: #TODO
        :return: #TODO
        """
        finishPositionCoordinate = finishPosition.get_position()
        finishPositionCoordinate[2] = self.hip_to_torso[2, 3] + self.walking_hip_height
        finishPosition.set_position(finishPositionCoordinate)
        self.goal_position = finishPosition

    def publishAngles(self):
        js = JointState()
        js.name = []
        js.header.stamp = rospy.Time.now()
        js.position = []
        js.effort = []
        for i, n in enumerate(self.motor_names):
            js.name.append(n)
            js.velocity.append(self.velocity_configuration[i]) # Velocity control only
        self.pub_all_motor.publish(js)
