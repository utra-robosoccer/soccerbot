import os
import enum

import rospy

from soccer_common.transformation import Transformation as tr
import matplotlib.pyplot as plt
from soccer_pycontrol.path_robot import PathRobot
import math
from os.path import expanduser
from copy import deepcopy
import numpy as np
import pybullet as pb
from soccer_common.pid import PID
from calibration import Calibration

class Joints(enum.IntEnum):
    LEFT_ARM_1 = 0
    LEFT_ARM_2 = 1
    RIGHT_ARM_1 = 2
    RIGHT_ARM_2 = 3
    LEFT_LEG_1 = 4
    LEFT_LEG_2 = 5
    LEFT_LEG_3 = 6
    LEFT_LEG_4 = 7
    LEFT_LEG_5 = 8
    LEFT_LEG_6 = 9
    RIGHT_LEG_1 = 10
    RIGHT_LEG_2 = 11
    RIGHT_LEG_3 = 12
    RIGHT_LEG_4 = 13
    RIGHT_LEG_5 = 14
    RIGHT_LEG_6 = 15
    HEAD_1 = 16
    HEAD_2 = 17
    HEAD_CAMERA = 18
    IMU = 19


class Links(enum.IntEnum):
    TORSO = -1
    LEFT_ARM_1 = 0
    LEFT_ARM_2 = 1
    RIGHT_ARM_1 = 2
    RIGHT_ARM_2 = 3
    LEFT_LEG_1 = 4
    LEFT_LEG_2 = 5
    LEFT_LEG_3 = 6
    LEFT_LEG_4 = 7
    LEFT_LEG_5 = 8
    LEFT_LEG_6 = 9
    RIGHT_LEG_1 = 10
    RIGHT_LEG_2 = 11
    RIGHT_LEG_3 = 12
    RIGHT_LEG_4 = 13
    RIGHT_LEG_5 = 14
    RIGHT_LEG_6 = 15
    HEAD_1 = 16
    HEAD_2 = 17
    HEAD_CAMERA = 18
    IMU = 19


class Soccerbot:
    standing_hip_height = 0.36  # Hardcoded for now, todo calculate this
    walking_hip_height = 0.165
    foot_box = [0.09, 0.07, 0.01474]
    right_collision_center = [0.00385, 0.00401, -0.00737]
    pybullet_offset = [0.0082498, -0.0017440, -0.0522479]
    arm_0_center = -0.45
    arm_1_center = np.pi * 0.8

    def __init__(self, pose, useFixedBase=False, useCalibration=True):
        self.useCalibration = useCalibration

        home = expanduser("~")
        self.body = pb.loadURDF(home + "/catkin_ws/src/soccerbot/soccer_description/models/soccerbot_stl.urdf",
                                useFixedBase=useFixedBase,
                                flags=pb.URDF_USE_INERTIA_FROM_FILE,
                                basePosition=[pose.get_position()[0], pose.get_position()[1],
                                              Soccerbot.standing_hip_height],
                                baseOrientation=pose.get_orientation())

        # IMU Stuff
        self.prev_lin_vel = [0, 0, 0]
        self.time_step_sim = 1. / 240

        self.foot_center_to_floor = -self.right_collision_center[2] + self.foot_box[2]

        # Calculate Constants
        H34 = self.get_link_transformation(Links.RIGHT_LEG_4, Links.RIGHT_LEG_3)
        H45 = self.get_link_transformation(Links.RIGHT_LEG_5, Links.RIGHT_LEG_4)
        self.DH = np.array([[0, -np.pi / 2, 0, 0],
                            [0, np.pi / 2, 0, 0],
                            [H34[2, 3], 0, 0, 0],
                            [H45[2, 3], 0, 0, 0],
                            [0, np.pi / 2, 0, 0],
                            [0, 0, 0, 0]])
        self.torso_to_right_hip = self.get_link_transformation(Links.TORSO, Links.RIGHT_LEG_1)
        self.right_hip_to_left_hip = self.get_link_transformation(Links.LEFT_LEG_1, Links.RIGHT_LEG_1)
        self.hip_to_torso = self.get_link_transformation(Links.RIGHT_LEG_1, Links.TORSO)

        self.right_foot_init_position = self.get_link_transformation(Links.TORSO, Links.RIGHT_LEG_6)
        self.right_foot_init_position[2, 3] = -(
                self.hip_to_torso[2, 3] + self.walking_hip_height) + self.foot_center_to_floor

        self.left_foot_init_position = self.get_link_transformation(Links.TORSO, Links.LEFT_LEG_6)
        self.left_foot_init_position[2, 3] = -(
                self.hip_to_torso[2, 3] + self.walking_hip_height) + self.foot_center_to_floor

        self.setPose(pose)
        self.torso_offset = tr()
        self.robot_path : PathRobot = None
        self.robot_odom_path: PathRobot = None

        self.configuration = [0.0] * len(Joints)
        self.configuration_offset = [0.0] * len(Joints)
        self.max_forces = []
        for i in range(0, 20):
            self.max_forces.append(pb.getJointInfo(self.body, i)[10])

        pb.setJointMotorControlArray(bodyIndex=self.body,
                                     controlMode=pb.POSITION_CONTROL,
                                     jointIndices=list(range(0, 20, 1)),
                                     targetPositions=self.get_angles(),
                                     forces=self.max_forces)

        self.current_step_time = 0

        # For head rotation
        self.head_step = 0.0

        self.calibration = Calibration()
        # self.calibration.adjust_navigation_goal(np.array([[0, 0]]))

    def get_angles(self):
        """
        Function for getting all the feet angles (for now?)
        :return: All 12 angles in the dictionary form???
        """
        angles = [a + b for a, b in zip(self.configuration, self.configuration_offset)]
        return angles

    def get_link_transformation(self, link1, link2):
        """
        Gives the H-trasnform between two links
        :param link1: Starting link
        :param link2: Ending link
        :return: H-transform from starting link to the ending link
        """
        if link1 == Links.TORSO:
            link1world = pb.getBasePositionAndOrientation(self.body)
            link1world = (tuple(np.subtract(link1world[0], tuple(self.pybullet_offset))), (0, 0, 0, 1))
        else:
            link1world = pb.getLinkState(self.body, link1)[4:6]

        if link2 == Links.TORSO:
            link2world = pb.getBasePositionAndOrientation(self.body)
            link2world = (tuple(np.subtract(link2world[0], tuple(self.pybullet_offset))), (0, 0, 0, 1))
        else:
            link2world = pb.getLinkState(self.body, link2)[4:6]

        link1worldrev = pb.invertTransform(link1world[0], link1world[1])
        link2worldrev = pb.invertTransform(link2world[0], link2world[1])

        final_transformation = pb.multiplyTransforms(link2world[0], link2world[1], link1worldrev[0],
                                                     link1worldrev[1])
        return tr(np.round(list(final_transformation[0]), 5), np.round(list(final_transformation[1]), 5))


    def ready(self):
        """
        Sets the robot's joint angles for the robot to standing pose.
        :return: None
        """
        # Used later to calculate inverse kinematics
        position = self.pose.get_position()
        position[2] = self.hip_to_torso[2, 3] + self.walking_hip_height
        self.pose.set_position(position)

        # hands
        configuration = [0.0] * len(Joints)
        configuration[Joints.RIGHT_ARM_1] = Soccerbot.arm_0_center
        configuration[Joints.LEFT_ARM_1] = Soccerbot.arm_0_center
        configuration[Joints.RIGHT_ARM_2] = Soccerbot.arm_1_center
        configuration[Joints.LEFT_ARM_2] = Soccerbot.arm_1_center

        # right leg
        thetas = self.inverseKinematicsRightFoot(np.copy(self.right_foot_init_position))
        configuration[Links.RIGHT_LEG_1:Links.RIGHT_LEG_6 + 1] = thetas[0:6]

        # left leg
        thetas = self.inverseKinematicsLeftFoot(np.copy(self.left_foot_init_position))
        configuration[Links.LEFT_LEG_1:Links.LEFT_LEG_6 + 1] = thetas[0:6]

        # head
        configuration[Joints.HEAD_1] = 0
        configuration[Joints.HEAD_2] = 0

        # Slowly ease into the ready position
        if sum(self.configuration[Links.LEFT_LEG_6:Links.RIGHT_LEG_6 + 1]) == 0:
            for r in np.arange(0, 1, 0.040):
                rospy.loginfo_throttle(1, "Going into ready position")
                self.configuration = (np.array(configuration) * r).tolist()
                self.publishAngles()
                rospy.sleep(0.020)

        self.configuration_offset = [0] * len(Joints)

        pb.setJointMotorControlArray(bodyIndex=self.body,
                                     controlMode=pb.POSITION_CONTROL,
                                     jointIndices=list(range(0, 20, 1)),
                                     targetPositions=self.get_angles(),
                                     forces=self.max_forces)

    def inverseKinematicsRightFoot(self, transformation):
        """
        #TODO
        :param transformation: #TODO
        :return: Motor angles for the right foot
        """
        transformation[0:3, 3] = transformation[0:3, 3] - self.torso_to_right_hip[0:3, 3]
        invconf = np.linalg.inv(transformation)

        d3 = self.DH[2, 0]
        d4 = self.DH[3, 0]

        Xd = invconf[0, 3]
        Yd = invconf[1, 3]
        Zd = invconf[2, 3]

        if (np.linalg.norm([Xd, Yd, Zd]) > (d3 + d4)):
            print("IK Position Unreachable: Desired Distance: " + str(
                np.linalg.norm([Xd, Yd, Zd])) + ", Limited Distance: " + str(d3 + d4))
        assert (np.linalg.norm([Xd, Yd, Zd]) <= (d3 + d4))

        theta6 = -np.arctan2(Yd, Zd)
        tmp1 = Zd / np.cos(theta6)
        tmp2 = Xd
        D = ((((((tmp1 ** 2) + (tmp2 ** 2)) - ((d3 ** 2) + (d4 ** 2))) / 2) / d3) / d4)
        tmp3 = np.arctan2(D, -np.sqrt(1 - (D ** 2)))

        tmpX = (tmp3 - (np.pi / 2))
        if tmpX < 0:
            tmpX = tmpX + (2 * np.pi)
        theta4 = -(np.unwrap([tmpX])[0])

        assert (theta4 < 4.6)

        alp = np.arctan2(tmp1, tmp2)
        beta = np.arctan2(-d3 * np.cos(tmp3), d4 + (d3 * np.sin(tmp3)))
        theta5 = np.pi / 2 - (alp - beta)

        H34 = tr.get_transform_from_dh(self.DH[3, 0], self.DH[3, 1], self.DH[3, 2], theta4)
        H45 = tr.get_transform_from_dh(self.DH[4, 0], self.DH[4, 1], self.DH[4, 2], theta5)
        H56 = tr.get_transform_from_dh(self.DH[5, 0], self.DH[5, 1], self.DH[5, 2], theta6)
        H36 = np.matmul(H34, np.matmul(H45, H56))
        final_rotation = tr.get_transform_from_euler([0, np.pi / 2, np.pi])
        H03 = np.matmul(np.matmul(transformation, final_rotation), np.linalg.inv(H36))
        assert (np.linalg.norm(H03[0:3, 3]) - d3 < 0.03)

        angles = tr.get_euler_from_rotation_matrix(np.linalg.inv(H03[0:3, 0:3]), orientation='ZYX')
        theta3 = np.pi / 2 - angles[0]
        theta1 = -angles[1]
        theta2 = (angles[2] + np.pi / 2)

        return [theta1, theta2, theta3, theta4, theta5, theta6]

    def inverseKinematicsLeftFoot(self, transformation):
        """
        Inverse kinematic function for the left foot. Works due to symmetry between left and right foot.
        :param transformation:
        :return: Motor angles for the left foot
        """
        transformation[0: 3, 3] = transformation[0: 3, 3] + self.right_hip_to_left_hip[0: 3, 3]
        [theta1, theta2, theta3, theta4, theta5, theta6] = self.inverseKinematicsRightFoot(transformation)
        return [-theta1, -theta2, theta3, theta4, theta5, -theta6]

    def setPose(self, pose: tr):
        try:
            last_hip_height = self.pose.get_position()[2]
        except:
            self.pose = pose
            last_hip_height = Soccerbot.standing_hip_height
        self.pose.set_position([pose.get_position()[0], pose.get_position()[1], last_hip_height])

        # Remove the roll and yaw from the pose
        [r, p, y] = pose.get_orientation_euler()
        q_new = tr.get_quaternion_from_euler([r, 0, 0])
        self.pose.set_orientation(q_new)
        pb.resetBasePositionAndOrientation(self.body, self.pose.get_position(), self.pose.get_orientation())

    def addTorsoHeight(self, position: tr):
        positionCoordinate = position.get_position()
        positionCoordinate[2] = self.hip_to_torso[2, 3] + self.walking_hip_height
        position.set_position(positionCoordinate)

    def createPathToGoal(self, finishPosition: tr):
        """
        Returns the trajectories for the robot's feet and crotch. The coordinates x,y will be used only.
        :param finishPosition: #TODO
        :return: #TODO
        """
        self.addTorsoHeight(finishPosition)

        # Remove the roll and yaw from the designated position
        [r, p, y] = finishPosition.get_orientation_euler()
        q_new = tr.get_quaternion_from_euler([r, 0, 0])
        finishPosition.set_orientation(q_new)

        # Add calibration
        if self.useCalibration:
            finishPositionCalibrated = self.calibration.adjust_navigation_transform(self.pose, finishPosition)
        else:
            finishPositionCalibrated = finishPosition

        print(f"\033[92mEnd Pose Calibrated: Position (xyz) [{finishPositionCalibrated.get_position()[0]:.3f} {finishPositionCalibrated.get_position()[1]:.3f} {finishPositionCalibrated.get_position()[2]:.3f}], "
              f"Orientation (xyzw) [{finishPositionCalibrated.get_orientation()[0]:.3f} {finishPositionCalibrated.get_orientation()[1]:.3f} {finishPositionCalibrated.get_orientation()[2]:.3f} {finishPositionCalibrated.get_orientation()[3]:.3f}]\033[0m")

        self.robot_path = PathRobot(self.pose, finishPositionCalibrated, self.foot_center_to_floor)
        self.robot_odom_path = PathRobot(self.pose, finishPosition, self.foot_center_to_floor)

        # obj.rate = rateControl(1 / obj.robot_path.step_size); -- from findPath
        self.rate = 1 / self.robot_path.step_precision
        self.period = self.robot_path.step_precision

        self.current_step_time = 0
        return self.robot_path

    def stepPath(self, t, verbose=False):

        assert (t <= self.robot_path.duration())
        crotch_position = self.robot_path.crotchPosition(t) @ self.torso_offset

        [right_foot_position, left_foot_position] = self.robot_path.footPosition(t)
        torso_to_left_foot = np.matmul(np.linalg.inv(crotch_position),
                                       left_foot_position)  # crotch_position \ left_foot_position;
        torso_to_right_foot = np.matmul(np.linalg.inv(crotch_position),
                                        right_foot_position)  # crotch_position \ right_foot_position;

        if verbose:
            print("------------------- feet angles -------------------")
        thetas = self.inverseKinematicsRightFoot(torso_to_right_foot)
        if verbose:
            print("Right foot: " + str([format(theta, '.3f') for theta in thetas]).replace("'", ""))
        self.configuration[Links.RIGHT_LEG_1:Links.RIGHT_LEG_6 + 1] = thetas[0:6]
        thetas = self.inverseKinematicsLeftFoot(torso_to_left_foot)
        if verbose:
            print("Left foot: " + str([format(theta, '.3f') for theta in thetas]).replace("'", ""))
            print("--------------------------------------------------")
        self.configuration[Links.LEFT_LEG_1:Links.LEFT_LEG_6 + 1] = thetas[0:6]
        self.pose = crotch_position

    def calculate_angles(self, show=True):
        angles = []
        iterator = np.linspace(0, self.robot_path.duration(),
                               num=math.ceil(self.robot_path.duration() / self.robot_path.step_precision) + 1)
        if show:
            plot_angles = np.zeros((len(iterator), 20))
        i = 0
        for t in iterator:
            self.stepPath(t)
            angles.append((t, self.get_angles().copy()))
            if show:
                plot_angles[i] = np.array(self.get_angles().copy())
            i = i + 1
        if show:
            fig = plt.figure(3, tight_layout=True)

            # Left Leg
            plt.subplot(311)
            plt.plot(iterator, plot_angles[:, Joints.LEFT_LEG_1], label='LEFT_LEG_1')
            plt.plot(iterator, plot_angles[:, Joints.LEFT_LEG_2], label='LEFT_LEG_2')
            plt.plot(iterator, plot_angles[:, Joints.LEFT_LEG_3], label='LEFT_LEG_3')
            plt.plot(iterator, plot_angles[:, Joints.LEFT_LEG_4], label='LEFT_LEG_4')
            plt.plot(iterator, plot_angles[:, Joints.LEFT_LEG_5], label='LEFT_LEG_5')
            plt.plot(iterator, plot_angles[:, Joints.LEFT_LEG_6], label='LEFT_LEG_6')
            plt.title('Left Foot')
            plt.xlabel('time (t)')
            plt.ylabel('Angles')
            plt.legend()
            plt.grid(b=True, which='both', axis='both')

            # Right Leg
            plt.subplot(312)
            plt.plot(iterator, plot_angles[:, Joints.RIGHT_LEG_1], label='RIGHT_LEG_1')
            plt.plot(iterator, plot_angles[:, Joints.RIGHT_LEG_2], label='RIGHT_LEG_2')
            plt.plot(iterator, plot_angles[:, Joints.RIGHT_LEG_3], label='RIGHT_LEG_3')
            plt.plot(iterator, plot_angles[:, Joints.RIGHT_LEG_4], label='RIGHT_LEG_4')
            plt.plot(iterator, plot_angles[:, Joints.RIGHT_LEG_5], label='RIGHT_LEG_5')
            plt.plot(iterator, plot_angles[:, Joints.RIGHT_LEG_6], label='RIGHT_LEG_6')
            plt.title('Right Foot')
            plt.xlabel('time (t)')
            plt.ylabel('Angles')
            plt.legend()
            plt.grid(b=True, which='both', axis='both')

            # Head & Arms
            plt.subplot(313)
            plt.plot(iterator, plot_angles[:, Joints.HEAD_1], label='HEAD_1')
            plt.plot(iterator, plot_angles[:, Joints.HEAD_2], label='HEAD_2')
            plt.plot(iterator, plot_angles[:, Joints.RIGHT_ARM_1], label='RIGHT_ARM_1')
            plt.plot(iterator, plot_angles[:, Joints.RIGHT_ARM_2], label='RIGHT_ARM_2')
            plt.plot(iterator, plot_angles[:, Joints.LEFT_ARM_1], label='LEFT_ARM_1')
            plt.plot(iterator, plot_angles[:, Joints.LEFT_ARM_2], label='LEFT_ARM_2')
            plt.title('Head & Arms')
            plt.xlabel('time (t)')
            plt.ylabel('Angles')
            plt.legend()
            plt.grid(b=True, which='both', axis='both')

            fig.canvas.draw()
            plt.show()

    def get_imu_raw(self, verbose=False):
        """
        Simulates the IMU at the IMU link location.
        TODO: Add noise model, make the refresh rate vary (currently in sync with the PyBullet time steps)
        :param verbose: Optional - Set to True to print the linear acceleration and angular velocity
        :return: concatenated 3-axes values for linear acceleration and angular velocity
        """
        [quart_link, lin_vel, ang_vel] = pb.getLinkState(self.body, linkIndex=Links.IMU,
                                                         computeLinkVelocity=1)[5:8]
        # [lin_vel, ang_vel] = p.getLinkState(bodyUniqueId=self.soccerbotUid, linkIndex=Links.HEAD_1, computeLinkVelocity=1)[6:8]
        # print(p.getLinkStates(bodyUniqueId=self.soccerbotUid, linkIndices=range(0,20,1), computeLinkVelocity=1))
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
            print(f'lin_acc = {lin_acc}', end="\t\t")
            print(f'ang_vel = {ang_vel}')
        return np.concatenate((lin_acc, ang_vel))

    def get_imu(self):
        """
        Simulates the IMU at the IMU link location.
        TODO: Add noise model, make the refresh rate vary (currently in sync with the PyBullet time steps)
        :param verbose: Optional - Set to True to print the linear acceleration and angular velocity
        :return: concatenated 3-axes values for linear acceleration and angular velocity
        """
        [quat_pos, quat_orientation] = pb.getLinkState(self.body, linkIndex=Links.IMU,
                                                       computeLinkVelocity=1)[4:6]

        return tr(quat_pos, quat_orientation)

    def get_foot_pressure_sensors(self, floor):
        """
        Checks if 4 corners of the each feet are in contact with ground

        Indicies for looking from above on the feet plates:
          Left         Right
        4-------5    0-------1
        |   ^   |    |   ^   |      ^
        |   |   |    |   |   |      | : forward direction
        |       |    |       |
        6-------7    2-------3

        :param floor: PyBullet body id of the plane the robot is walking on.
        :return: boolean array of 8 contact points on both feet, True: that point is touching the ground False: otherwise
        """
        locations = [False] * 8
        right_pts = pb.getContactPoints(bodyA=self.body, bodyB=floor, linkIndexA=Links.RIGHT_LEG_6)
        left_pts = pb.getContactPoints(bodyA=self.body, bodyB=floor, linkIndexA=Links.LEFT_LEG_6)
        right_center = np.array(pb.getLinkState(self.body, linkIndex=Links.RIGHT_LEG_6)[4])
        left_center = np.array(pb.getLinkState(self.body, linkIndex=Links.LEFT_LEG_6)[4])
        right_tr = tr.get_rotation_matrix_from_transformation(
            tr(quaternion=pb.getLinkState(self.body, linkIndex=Links.RIGHT_LEG_6)[5]))
        left_tr = tr.get_rotation_matrix_from_transformation(
            tr(quaternion=pb.getLinkState(self.body, linkIndex=Links.LEFT_LEG_6)[5]))
        for point in right_pts:
            index = np.signbit(np.matmul(right_tr, point[5] - right_center))[0:2]
            locations[index[1] + index[0] * 2] = True
        for point in left_pts:
            index = np.signbit(np.matmul(left_tr, point[5] - left_center))[0:2]
            locations[index[1] + (index[0] * 2) + 4] = True
        return locations

    walking_pid = PID(Kp=0.8, Kd=0.0, Ki=0.0005, setpoint=-0.01, output_limits=(-1.57, 1.57))
    def apply_imu_feedback(self, t: float, pose: tr):
        if pose is None:
            return

        [roll, pitch, yaw] = pose.get_orientation_euler()
        F = self.walking_pid.update(pitch)
        self.configuration_offset[Joints.LEFT_ARM_1] = 5 * F
        self.configuration_offset[Joints.RIGHT_ARM_1] = 5 * F
        return F

    standing_pid = PID(Kp=0.15, Kd=0.0, Ki=0.001, setpoint=-0.01, output_limits=(-1.57, 1.57))
    def apply_imu_feedback_standing(self, pose: tr):
        if pose is None:
            return
        [roll, pitch, yaw] = pose.get_orientation_euler()
        F = self.standing_pid.update(pitch)
        self.configuration_offset[Joints.LEFT_LEG_5] = F
        self.configuration_offset[Joints.RIGHT_LEG_5] = F
        return pitch

    def reset_imus(self):
        self.walking_pid.reset()
        self.standing_pid.reset()

    def apply_head_rotation(self):
        pass

    def apply_foot_pressure_sensor_feedback(self, floor):
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
        pass
