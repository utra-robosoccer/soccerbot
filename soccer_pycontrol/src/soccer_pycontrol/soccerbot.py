import math
from copy import deepcopy
from os.path import expanduser
from typing import Union

import matplotlib.pyplot as plt
import numpy as np
import pybullet as pb
import rospy
import scipy
from rospy import ROSException
from sensor_msgs.msg import JointState

from soccer_common.pid import PID
from soccer_common.transformation import Transformation
from soccer_pycontrol.calibration import adjust_navigation_transform
from soccer_pycontrol.joints import Joints
from soccer_pycontrol.links import Links
from soccer_pycontrol.path_robot import PathRobot
from soccer_pycontrol.utils import wrapToPi


class Soccerbot:
    """
    The main class for soccerbot, which receives and sends information to pybullet, inherited by ROS
    """

    def __init__(self, pose, useFixedBase=False, useCalibration=True):
        """
        Initialization function for soccerbot. Does a series of calculations based on the URDF file for standing, walking poses

        :param pose: The position to initialize the robot, Z doesn't matter here as it will be set automatically
        :param useFixedBase: Whether to fix the base_link in the air for movement testing
        :param useCalibration: Whether to use calibration for walking path calculations
        """

        #: Height of the robot's torso (center between two arms) while walking
        self.walking_torso_height = rospy.get_param("walking_torso_height", 0.315)

        #: Dimensions of the foot collision box #TODO get it from URDF
        self.foot_box = rospy.get_param("foot_box", [0.09, 0.07, 0.01474])

        #: Transformations from the right foots joint position to the center of the collision box of the foot (https://docs.google.com/presentation/d/10DKYteySkw8dYXDMqL2Klby-Kq4FlJRnc4XUZyJcKsw/edit#slide=id.g163c1c67b73_0_0)
        self.right_foot_joint_center_to_collision_box_center = rospy.get_param(
            "right_foot_joint_center_to_collision_box_center", [0.00385, 0.00401, -0.00737]
        )

        #: Ready Pose angle for arm 1
        self.arm_0_center = rospy.get_param("arm_0_center", -0.45)

        #: Ready Pose angle for arm 2
        self.arm_1_center = rospy.get_param("arm_0_center", np.pi * 0.8)

        self.useCalibration = useCalibration

        home = expanduser("~")
        self.body = pb.loadURDF(
            home
            + f"/catkin_ws/src/soccerbot/{rospy.get_param('~robot_model', 'bez1')}_description/urdf/{rospy.get_param('~robot_model', 'bez1')}.urdf",
            useFixedBase=useFixedBase,
            flags=pb.URDF_USE_INERTIA_FROM_FILE | (pb.URDF_MERGE_FIXED_LINKS if rospy.get_param("merge_fixed_links", False) else 0),
            basePosition=[pose.position[0], pose.position[1], pose.position[2]],
            baseOrientation=pose.quaternion,
        )
        self.pybullet_offset = pb.getBasePositionAndOrientation(self.body)[0][:2] + (0,)  # pb.getLinkState(self.body, Links.TORSO)[4:6]
        self.motor_names = [pb.getJointInfo(self.body, i)[1].decode("utf-8") for i in range(18)]

        # IMU Stuff
        self.prev_lin_vel = [0, 0, 0]
        self.time_step_sim = 1.0 / 240

        self.foot_center_to_floor = -self.right_foot_joint_center_to_collision_box_center[2] + self.foot_box[2]

        # Calculate Constants
        H34 = self.get_link_transformation(Links.RIGHT_LEG_4, Links.RIGHT_LEG_3)
        H45 = self.get_link_transformation(Links.RIGHT_LEG_5, Links.RIGHT_LEG_4)
        self.DH = np.array(
            [
                [0, -np.pi / 2, 0, 0],
                [0, np.pi / 2, 0, 0],
                [H34[2, 3], 0, 0, 0],
                [H45[2, 3], 0, 0, 0],
                [0, np.pi / 2, 0, 0],
                [0, 0, 0, 0],
            ]
        )
        self.torso_to_right_hip = self.get_link_transformation(Links.TORSO, Links.RIGHT_LEG_1)
        self.right_hip_to_left_hip = self.get_link_transformation(Links.LEFT_LEG_1, Links.RIGHT_LEG_1)
        self.hip_to_torso = abs(self.get_link_transformation(Links.RIGHT_LEG_1, Links.TORSO)[2, 3])

        pitch_correction = Transformation([0, 0, 0], euler=[0, rospy.get_param("torso_offset_pitch_ready", 0.0), 0])

        self.right_foot_init_position = self.get_link_transformation(Links.TORSO, Links.RIGHT_LEG_6)
        self.right_foot_init_position[2, 3] = -self.walking_torso_height + self.foot_center_to_floor
        self.right_foot_init_position[0, 3] -= rospy.get_param("torso_offset_x_ready", 0.0)
        self.right_foot_init_position = pitch_correction @ self.right_foot_init_position

        self.left_foot_init_position = self.get_link_transformation(Links.TORSO, Links.LEFT_LEG_6)
        self.left_foot_init_position[2, 3] = -self.walking_torso_height + self.foot_center_to_floor
        self.left_foot_init_position[0, 3] -= rospy.get_param("torso_offset_x_ready", 0.0)
        self.left_foot_init_position = pitch_correction @ self.left_foot_init_position

        self.setPose(pose)

        pitch_correction = Transformation([0, 0, 0], euler=[0, rospy.get_param("torso_offset_pitch", 0.0), 0])
        self.torso_offset = Transformation([rospy.get_param("torso_offset_x", 0), 0, 0]) @ pitch_correction
        self.robot_path: Union[PathRobot, None] = None
        self.robot_odom_path: Union[PathRobot, None] = None

        self.configuration = [0.0] * len(Joints)  #: The 18x1 float array motor angle configuration for the robot's 18 motors
        self.configuration_offset = [0.0] * len(Joints)  #: The offset for the 18x1 motor angle configurations
        self.max_forces = []
        for i in range(0, 18):
            self.max_forces.append(pb.getJointInfo(self.body, i)[10] or rospy.get_param("max_force", 6))

        pb.setJointMotorControlArray(
            bodyIndex=self.body,
            controlMode=pb.POSITION_CONTROL,
            jointIndices=list(range(0, 18, 1)),
            targetPositions=self.get_angles(),
            forces=self.max_forces,
        )

        self.current_step_time = 0

        # For head rotation
        self.head_step = 0.0

        #: PID values to adjust the torso's front and back movement while standing, getting ready to walk, and post walk
        self.standing_pid = PID(
            Kp=rospy.get_param("standing_Kp", 0.15),
            Kd=rospy.get_param("standing_Kd", 0.0),
            Ki=rospy.get_param("standing_Ki", 0.001),
            setpoint=rospy.get_param("standing_setpoint", -0.01),
            output_limits=(-1.57, 1.57),
        )

        #: PID values to adjust the torso's front and back movement while walking
        self.walking_pid = PID(
            Kp=rospy.get_param("walking_Kp", 0.8),
            Kd=rospy.get_param("walking_Kd", 0.0),
            Ki=rospy.get_param("walking_Ki", 0.0005),
            setpoint=rospy.get_param("walking_setpoint", -0.01),
            output_limits=(-1.57, 1.57),
        )

    def get_angles(self):
        """
        Function for getting all the angles, combines the configuration with the configuration offset

        :return: All 18 angles of the robot in an array formation
        """
        angles = [wrapToPi(a + b) for a, b in zip(self.configuration, self.configuration_offset)]
        return angles

    def get_link_transformation(self, link1, link2):
        """
        Gives the H-trasnform between two links
        :param link1: Starting link
        :param link2: Ending link
        :return: H-transform from starting link to the ending link
        """
        if link1 == Links.TORSO:
            link1world = ((0, 0, 0), (0, 0, 0, 1))
        else:
            link1world = pb.getLinkState(self.body, link1)[4:6]

        if link2 == Links.TORSO:
            link2world = ((0, 0, 0), (0, 0, 0, 1))
        else:
            link2world = pb.getLinkState(self.body, link2)[4:6]

        link1worldrev = pb.invertTransform(link1world[0], link1world[1])

        final_transformation = pb.multiplyTransforms(link2world[0], link2world[1], link1worldrev[0], link1worldrev[1])
        return Transformation(np.round(list(final_transformation[0]), 5), np.round(list(final_transformation[1]), 5))

    def ready(self) -> None:
        """
        Sets the robot's joint angles for the robot to standing pose.
        """

        # hands
        configuration = [0.0] * len(Joints)
        configuration[Joints.RIGHT_ARM_1] = self.arm_0_center
        configuration[Joints.LEFT_ARM_1] = self.arm_0_center
        configuration[Joints.RIGHT_ARM_2] = self.arm_1_center
        configuration[Joints.LEFT_ARM_2] = self.arm_1_center

        # right leg
        thetas = self.inverseKinematicsRightFoot(np.copy(self.right_foot_init_position))
        configuration[Links.RIGHT_LEG_1 : Links.RIGHT_LEG_6 + 1] = thetas[0:6]

        # left leg
        thetas = self.inverseKinematicsLeftFoot(np.copy(self.left_foot_init_position))
        configuration[Links.LEFT_LEG_1 : Links.LEFT_LEG_6 + 1] = thetas[0:6]

        # head
        configuration[Joints.HEAD_1] = 0
        configuration[Joints.HEAD_2] = 0

        # Slowly ease into the ready position
        previous_configuration = self.configuration
        try:
            joint_state = rospy.wait_for_message("joint_states", JointState, timeout=3)
            indexes = [joint_state.name.index(motor_name) for motor_name in self.motor_names]
            previous_configuration = [joint_state.position[i] for i in indexes]
        except (ROSException, AttributeError) as ex:
            rospy.logerr(ex)
        except ValueError as ex:
            print(ex)
            rospy.logerr("Not all joint states are reported, cable disconnect?")
            rospy.logerr("Joint States")
            rospy.logerr(joint_state)
            rospy.logerr("Motor Names")
            print(self.motor_names)
            previous_configuration = [0] * len(Joints)

        for r in np.arange(0, 1.00, 0.040):
            rospy.loginfo_throttle(1, "Going into ready position")
            self.configuration[0:18] = (
                np.array(np.array(configuration[0:18]) - np.array(previous_configuration[0:18])) * r + np.array(previous_configuration[0:18])
            ).tolist()
            self.publishAngles()
            if pb.isConnected():
                pb.setJointMotorControlArray(
                    bodyIndex=self.body,
                    controlMode=pb.POSITION_CONTROL,
                    jointIndices=list(range(0, 18, 1)),
                    targetPositions=self.get_angles(),
                    forces=self.max_forces,
                )
                pb.stepSimulation()
            rospy.sleep(rospy.get_param("get_ready_rate", 0.02))

        self.configuration_offset = [0] * len(Joints)

    def setWalkingTorsoHeight(self, pose: Transformation) -> Transformation:
        """
        Takes a 2D pose and sets the height of the pose to the height of the torso
        https://docs.google.com/presentation/d/10DKYteySkw8dYXDMqL2Klby-Kq4FlJRnc4XUZyJcKsw/edit#slide=id.g163c1c67b73_0_0

        :param position: 2D position of the robot's torso in a 3D transformation format
        """

        p = pose
        position = p.position
        position[2] = self.walking_torso_height
        p.position = position
        return p

    def inverseKinematicsRightFoot(self, transformation):
        """
        # Does the inverse kinematics calculation for the right foot

        :param transformation: The 3D transformation from the torso center to the foot center
        :return: 6x1 Motor angles for the right foot
        """
        transformation[0:3, 3] = transformation[0:3, 3] - self.torso_to_right_hip[0:3, 3]
        invconf = scipy.linalg.inv(transformation)
        d3 = self.DH[2, 0]
        d4 = self.DH[3, 0]

        Xd = invconf[0, 3]
        Yd = invconf[1, 3]
        Zd = invconf[2, 3]

        if np.linalg.norm([Xd, Yd, Zd]) > (d3 + d4):
            print(
                "IK Position Unreachable: Desired Distance: "
                + Transformation(np.linalg.norm([Xd, Yd, Zd]))
                + ", Limited Distance: "
                + Transformation(d3 + d4)
            )
        assert np.linalg.norm([Xd, Yd, Zd]) <= (d3 + d4)

        theta6 = -np.arctan2(Yd, Zd)
        tmp1 = Zd / np.cos(theta6)
        tmp2 = Xd
        D = (((((tmp1**2) + (tmp2**2)) - ((d3**2) + (d4**2))) / 2) / d3) / d4
        tmp3 = np.arctan2(D, -np.sqrt(1 - (D**2)))

        tmpX = tmp3 - (np.pi / 2)
        if tmpX < 0:
            tmpX = tmpX + (2 * np.pi)
        theta4 = -(np.unwrap([tmpX])[0])

        assert theta4 < 4.6

        alp = np.arctan2(tmp1, tmp2)
        beta = np.arctan2(-d3 * np.cos(tmp3), d4 + (d3 * np.sin(tmp3)))
        theta5 = np.pi / 2 - (alp - beta)

        H34 = Transformation(dh=[self.DH[3, 0], self.DH[3, 1], self.DH[3, 2], theta4])
        H45 = Transformation(dh=[self.DH[4, 0], self.DH[4, 1], self.DH[4, 2], theta5])
        H56 = Transformation(dh=[self.DH[5, 0], self.DH[5, 1], self.DH[5, 2], theta6])
        H36 = np.matmul(H34, np.matmul(H45, H56))
        final_rotation = Transformation(euler=[0, np.pi / 2, np.pi])
        H03 = np.matmul(np.matmul(transformation, final_rotation), scipy.linalg.inv(H36))
        assert np.linalg.norm(H03[0:3, 3]) - d3 < 0.03

        angles = Transformation(rotation_matrix=scipy.linalg.inv(H03[0:3, 0:3])).orientation_euler
        theta3 = np.pi / 2 - angles[0]
        theta1 = -angles[1]
        theta2 = angles[2] + np.pi / 2

        return [theta1, theta2, theta3, theta4, theta5, theta6]

    def inverseKinematicsLeftFoot(self, transformation):
        """
        Inverse kinematic function for the left foot. Works due to symmetry between left and right foot.

        :param transformation: The 3D transformation from the torso center to the foot center
        :return: Motor angles for the left foot
        """
        transformation[0:3, 3] = transformation[0:3, 3] + self.right_hip_to_left_hip[0:3, 3]
        [theta1, theta2, theta3, theta4, theta5, theta6] = self.inverseKinematicsRightFoot(transformation)
        return [-theta1, -theta2, theta3, theta4, theta5, -theta6]

    def setPose(self, pose: Transformation):
        """
        Teleports the robot to the desired pose

        :param pose: 3D position in pybullet
        """

        self.pose = self.setWalkingTorsoHeight(pose)

        # Remove the roll and yaw from the pose
        [r, p, y] = pose.orientation_euler
        self.pose.orientation_euler = [r, 0, 0]
        if pb.isConnected():
            pb.resetBasePositionAndOrientation(self.body, self.pose.position, self.pose.quaternion)

    def createPathToGoal(self, endPose: Transformation) -> PathRobot:
        """
        Creates a path from the robot's current location to the goal location

        :param endPose: 3D transformation
        :return: Robot path
        """
        startPose = self.setWalkingTorsoHeight(self.pose)
        endPose = self.setWalkingTorsoHeight(endPose)

        # Remove the roll and yaw from the designated position
        [r, p, y] = endPose.orientation_euler
        endPose.orientation_euler = [r, 0, 0]

        # Add calibration
        if self.useCalibration:
            endPoseCalibrated = adjust_navigation_transform(startPose, endPose)
        else:
            endPoseCalibrated = endPose

        print(
            f"\033[92mEnd Pose Calibrated: Position (xyz) [{endPoseCalibrated.position[0]:.3f} {endPoseCalibrated.position[1]:.3f} {endPoseCalibrated.position[2]:.3f}], "
            f"Orientation (xyzw) [{endPoseCalibrated.quaternion[0]:.3f} {endPoseCalibrated.quaternion[1]:.3f} {endPoseCalibrated.quaternion[2]:.3f} {endPoseCalibrated.quaternion[3]:.3f}]\033[0m"
        )

        self.robot_path = PathRobot(startPose, endPoseCalibrated, self.foot_center_to_floor)
        self.robot_odom_path = PathRobot(startPose, endPose, self.foot_center_to_floor)

        # obj.rate = rateControl(1 / obj.robot_path.step_size); -- from findPath
        self.rate = 1 / self.robot_path.step_precision
        self.period = self.robot_path.step_precision

        self.current_step_time = 0
        return self.robot_path

    def stepPath(self, t):
        """
        Updates the configuration for the robot for the next position t based on the current path

        :param t: Timestep relative to the time of the first path, where t=0 is the beginning of the path
        """

        assert t <= self.robot_path.duration()

        # Get Torso position (Average Time: 0.0007538795471191406)
        torso_position = self.robot_path.torsoPosition(t) @ self.torso_offset

        # Get foot position at time (Average Time: 0.0004878044128417969)
        [right_foot_position, left_foot_position] = self.robot_path.footPosition(t)

        # Calcualate the feet position relative from torso (Average Time: 0.000133514404296875)
        torso_to_left_foot = scipy.linalg.lstsq(torso_position, left_foot_position, lapack_driver="gelsy")[0]
        torso_to_right_foot = scipy.linalg.lstsq(torso_position, right_foot_position, lapack_driver="gelsy")[0]

        # Inverse kinematics for both feet (Average Time: 0.0015840530395507812)
        thetas = self.inverseKinematicsRightFoot(torso_to_right_foot)
        self.configuration[Links.RIGHT_LEG_1 : Links.RIGHT_LEG_6 + 1] = thetas[0:6]

        thetas = self.inverseKinematicsLeftFoot(torso_to_left_foot)
        self.configuration[Links.LEFT_LEG_1 : Links.LEFT_LEG_6 + 1] = thetas[0:6]

        self.pose = torso_position

    def plot_angles(self):
        """
        Creates a plot of all the angles
        """

        angles = []
        iterator = np.linspace(
            0,
            self.robot_path.duration(),
            num=math.ceil(self.robot_path.duration() / self.robot_path.step_precision) + 1,
        )
        plot_angles = np.zeros((len(iterator), 18))
        i = 0
        for t in iterator:
            self.stepPath(t)
            angles.append((t, self.get_angles().copy()))
            plot_angles[i] = np.array(self.get_angles().copy())
            i = i + 1
        fig = plt.figure(3, tight_layout=True)

        # Left Leg
        plt.subplot(311)
        plt.plot(iterator, plot_angles[:, Joints.LEFT_LEG_1], label="LEFT_LEG_1")
        plt.plot(iterator, plot_angles[:, Joints.LEFT_LEG_2], label="LEFT_LEG_2")
        plt.plot(iterator, plot_angles[:, Joints.LEFT_LEG_3], label="LEFT_LEG_3")
        plt.plot(iterator, plot_angles[:, Joints.LEFT_LEG_4], label="LEFT_LEG_4")
        plt.plot(iterator, plot_angles[:, Joints.LEFT_LEG_5], label="LEFT_LEG_5")
        plt.plot(iterator, plot_angles[:, Joints.LEFT_LEG_6], label="LEFT_LEG_6")
        plt.title("Left Foot")
        plt.xlabel("time (t)")
        plt.ylabel("Angles")
        plt.legend()
        plt.grid(b=True, which="both", axis="both")

        # Right Leg
        plt.subplot(312)
        plt.plot(iterator, plot_angles[:, Joints.RIGHT_LEG_1], label="RIGHT_LEG_1")
        plt.plot(iterator, plot_angles[:, Joints.RIGHT_LEG_2], label="RIGHT_LEG_2")
        plt.plot(iterator, plot_angles[:, Joints.RIGHT_LEG_3], label="RIGHT_LEG_3")
        plt.plot(iterator, plot_angles[:, Joints.RIGHT_LEG_4], label="RIGHT_LEG_4")
        plt.plot(iterator, plot_angles[:, Joints.RIGHT_LEG_5], label="RIGHT_LEG_5")
        plt.plot(iterator, plot_angles[:, Joints.RIGHT_LEG_6], label="RIGHT_LEG_6")
        plt.title("Right Foot")
        plt.xlabel("time (t)")
        plt.ylabel("Angles")
        plt.legend()
        plt.grid(b=True, which="both", axis="both")

        # Head & Arms
        plt.subplot(313)
        plt.plot(iterator, plot_angles[:, Joints.HEAD_1], label="HEAD_1")
        plt.plot(iterator, plot_angles[:, Joints.HEAD_2], label="HEAD_2")
        plt.plot(iterator, plot_angles[:, Joints.RIGHT_ARM_1], label="RIGHT_ARM_1")
        plt.plot(iterator, plot_angles[:, Joints.RIGHT_ARM_2], label="RIGHT_ARM_2")
        plt.plot(iterator, plot_angles[:, Joints.LEFT_ARM_1], label="LEFT_ARM_1")
        plt.plot(iterator, plot_angles[:, Joints.LEFT_ARM_2], label="LEFT_ARM_2")
        plt.title("Head & Arms")
        plt.xlabel("time (t)")
        plt.ylabel("Angles")
        plt.legend()
        plt.grid(b=True, which="both", axis="both")

        fig.canvas.draw()
        plt.show()

    def get_imu_raw(self, verbose=False):
        """
        Simulates the IMU at the IMU link location.
        TODO: Add noise model, make the refresh rate vary (currently in sync with the PyBullet time steps)

        :param verbose: Optional - Set to True to print the linear acceleration and angular velocity
        :return: concatenated 3-axes values for linear acceleration and angular velocity
        """

        quart_link, lin_vel, ang_vel = pb.getBasePositionAndOrientation(self.body)[1:2] + pb.getBaseVelocity(self.body)
        # [lin_vel, ang_vel] = p.getLinkState(bodyUniqueId=self.soccerbotUid, linkIndex=Links.HEAD_1, computeLinkVelocity=1)[6:8]
        # print(p.getLinkStates(bodyUniqueId=self.soccerbotUid, linkIndices=range(0,18,1), computeLinkVelocity=1))
        # p.getBaseVelocity(self.soccerbotUid)
        lin_vel = np.array(lin_vel, dtype=np.float32)
        self.gravity = [0, 0, -9.81]
        lin_acc = (lin_vel - self.prev_lin_vel) / self.time_step_sim
        lin_acc -= self.gravity
        rot_mat = np.array(pb.getMatrixFromQuaternion(quart_link), dtype=np.float32).reshape((3, 3))
        lin_acc = np.matmul(rot_mat, lin_acc)
        ang_vel = np.array(ang_vel, dtype=np.float32)
        self.prev_lin_vel = lin_vel
        if verbose:
            print(f"lin_acc = {lin_acc}", end="\t\t")
            print(f"ang_vel = {ang_vel}")
        return np.concatenate((lin_acc, ang_vel))

    def get_imu(self):
        """
        Simulates the IMU at the IMU link location.
        TODO: Add noise model, make the refresh rate vary (currently in sync with the PyBullet time steps)

        :return: concatenated 3-axes values for linear acceleration and angular velocity
        """
        if rospy.get_param("merge_fixed_links", False):
            [quat_pos, quat_orientation] = pb.getBasePositionAndOrientation(self.body)[0:2]
        else:
            [quat_pos, quat_orientation] = pb.getLinkState(self.body, linkIndex=Links.IMU, computeLinkVelocity=1)[4:6]

        return Transformation(quat_pos, quat_orientation)

    def get_foot_pressure_sensors(self, floor):
        """
        Checks if 4 corners of the each feet are in contact with ground #TODO fix docstring

        | Indices for looking from above on the feet plates
        |   Left         Right
        | 4-------5    0-------1
        | |   ^   |    |   ^   |      ^
        | |   |   |    |   |   |      | forward direction
        | |       |    |       |
        | 6-------7    2-------3

        :param floor: PyBullet body id of the plane the robot is walking on.
        :return: boolean array of 8 contact points on both feet, True: that point is touching the ground False: otherwise
        """
        locations = [False] * 8
        right_pts = pb.getContactPoints(bodyA=self.body, bodyB=floor, linkIndexA=Links.RIGHT_LEG_6)
        left_pts = pb.getContactPoints(bodyA=self.body, bodyB=floor, linkIndexA=Links.LEFT_LEG_6)
        right_center = np.array(pb.getLinkState(self.body, linkIndex=Links.RIGHT_LEG_6)[4])
        left_center = np.array(pb.getLinkState(self.body, linkIndex=Links.LEFT_LEG_6)[4])
        right_tr = Transformation(quaternion=pb.getLinkState(self.body, linkIndex=Links.RIGHT_LEG_6)[5]).rotation_matrix
        left_tr = Transformation(quaternion=pb.getLinkState(self.body, linkIndex=Links.LEFT_LEG_6)[5]).rotation_matrix
        for point in right_pts:
            index = np.signbit(np.matmul(right_tr, point[5] - right_center))[0:2]
            locations[index[1] + index[0] * 2] = True
        for point in left_pts:
            index = np.signbit(np.matmul(left_tr, point[5] - left_center))[0:2]
            locations[index[1] + (index[0] * 2) + 4] = True
        return locations

    def apply_imu_feedback(self, pose: Transformation):
        """
        Adds IMU feedback while the robot is moving to the arms

        :param pose: Pose of the torso
        :return: The value for the walking_pid controller
        """

        if pose is None:
            return

        [_, pitch, _] = pose.orientation_euler
        F = self.walking_pid.update(pitch)
        self.configuration_offset[Joints.LEFT_LEG_3] = F
        self.configuration_offset[Joints.RIGHT_LEG_3] = F
        return F

    def apply_imu_feedback_standing(self, pose: Transformation):
        """
        Adds IMU feedback while the robot is standing or getting ready to the arms

        :param pose: Pose of the torso
        :return: The value for the walking_pid controller
        """

        if pose is None:
            return
        [roll, pitch, yaw] = pose.orientation_euler
        F = self.standing_pid.update(pitch)
        self.configuration_offset[Joints.LEFT_LEG_5] = F
        self.configuration_offset[Joints.RIGHT_LEG_5] = F
        return pitch

    def reset_imus(self):
        """
        Reset the walking and standing PID values
        """

        self.walking_pid.reset()
        self.standing_pid.reset()

    def apply_head_rotation(self):
        """
        Does head rotation for the robot, robot will try to face the ball if it is ready, otherwise rotate around
        if its relocalizing or finding the ball
        """
        pass

    def apply_foot_pressure_sensor_feedback(self, floor):
        """
        Add foot pressure sensor feedback (Currently not implemented)

        :param floor: The floor object
        """

        foot_pressure_values = self.get_foot_pressure_sensors(floor)

        motor_forces = deepcopy(self.max_forces)
        if foot_pressure_values is None:
            return motor_forces

        # if (foot_pressure_values[0] and foot_pressure_values[1]) or (
        #         foot_pressure_values[2] and foot_pressure_values[3]):  # Right foot on the ground
        #     motor_forces[Joints.RIGHT_LEG_6] = 0.75
        # if (foot_pressure_values[4] and foot_pressure_values[5]) or (
        #         foot_pressure_values[6] and foot_pressure_values[7]):  # Right foot on the ground
        #     motor_forces[Joints.LEFT_LEG_6] = 0.75

        # Synchronise walking speed

        return motor_forces

    def publishAngles(self):
        """
        Publishes angles to ros
        """
