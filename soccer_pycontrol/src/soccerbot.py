import pybullet as pb
import numpy as np
import enum
from transformation import Transformation as tr
import matplotlib.pyplot as plt
from robotpath import Robotpath
import math
from scipy.spatial.transform import Rotation as R

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

class Soccerbot:
    hip_height = 0.165 #0.165
    foot_box = [0.09, 0.07, 0.01474]
    right_collision_center = [0.00385, 0.00401, -0.00737]
    pybullet_offset = [0.0082498, -0.0017440, -0.0522479]


    def get_link_transformation(self, link1, link2):
        """
        Gives the H-trasnform between two links
        :param link1: Starting link
        :param link2: Ending link
        :return: H-transform from starting link to the ending link
        """
        if link1 == Links.TORSO:
            link1world = pb.getBasePositionAndOrientation(self.body)
            link1world = (tuple(np.subtract(link1world[0], tuple(self.pybullet_offset))), (0,0,0,1))
        else:
            link1world = pb.getLinkState(self.body, link1)[4:6]

        if link2 == Links.TORSO:
            link2world = pb.getBasePositionAndOrientation(self.body)
            link2world = (tuple(np.subtract(link2world[0], tuple(self.pybullet_offset))), (0,0,0,1))
        else:
            link2world = pb.getLinkState(self.body, link2)[4:6]

        link1worldrev = pb.invertTransform(link1world[0], link1world[1])
        link2worldrev = pb.invertTransform(link2world[0], link2world[1])

        final_transformation = pb.multiplyTransforms(link2world[0], link2world[1], link1worldrev[0], link1worldrev[1])
        return tr(np.round(list(final_transformation[0]), 5), np.round(list(final_transformation[1]), 5))

    def __init__(self, position, useFixedBase=False):
        """
        Contsructor for the soccerbot. Loads the robot into the pybullet simulation.
        :param position: [x y yaw]
        :param useFixedBase: If true, it will fix the base link in space, thus preventing the robot falling. For testing purpose.
        """
        self.body = pb.loadURDF("../../soccer_description/models/soccerbot_stl.urdf", useFixedBase=useFixedBase, flags=pb.URDF_USE_INERTIA_FROM_FILE,  basePosition=[position[0],position[0],0.36], baseOrientation=[0.,0.,0.,1.]) #|pb.URDF_USE_SELF_COLLISION|pb.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT)
        self.foot_center_to_floor = -self.right_collision_center[2] + self.foot_box[2]

        # DH table
        #pb.resetBasePositionAndOrientation(self.body, (0,0,0.5), (0,0,0,1))
        H34 = self.get_link_transformation(Links.RIGHT_LEG_4, Links.RIGHT_LEG_3)
        H45 = self.get_link_transformation(Links.RIGHT_LEG_5, Links.RIGHT_LEG_4)
        self.DH = np.array([[0, -np.pi / 2, 0, 0],
                            [0, np.pi / 2, 0, 0],
                            [H34[2,3], 0, 0, 0],
                            [H45[2,3], 0, 0, 0],
                            [0, np.pi / 2, 0, 0],
                            [0, 0, 0, 0]])

        self.configuration = [0] * 18

        hip_position = [position[0], position[1], 0.5] # Hardcoded z for now
        self.pose = tr.get_transform_from_euler([0, 0, position[2]])
        self.pose.set_position(hip_position)

        self.torso_offset = tr()
        self.rpy_current = [0, 0, 0] #TODO: is this the right init?
        self.robot_path = None

        pb.setJointMotorControlArray(bodyIndex=self.body, controlMode=pb.PD_CONTROL, jointIndices=[0] * 18, targetPositions=[0] * 18)


    def stand(self):
        """
        Sets the robot's joint angles for the robot to standing pose.
        :return: None
        """
        # Used later to calculate inverse kinematics
        self.torso_to_right_hip = self.get_link_transformation(Links.TORSO, Links.RIGHT_LEG_1)

        # hands
        self.configuration[Joints.RIGHT_ARM_1] = 1.0 * np.pi
        self.configuration[Joints.LEFT_ARM_1] = 1.0 * np.pi

        # Robot Position
        hip_to_torso = self.get_link_transformation(Links.RIGHT_LEG_1, Links.TORSO)
        hip_position = [position[0], position[1], hip_to_torso[2, 3] + self.hip_height]
        self.pose = tr.get_transform_from_euler([0, 0, position[2]])
        self.pose.set_position(hip_position)
        self.update_pose(position)

        # right leg
        right_foot_position = self.get_link_transformation(Links.TORSO, Links.RIGHT_LEG_6)
        right_foot_position[2,3] = -self.pose.get_position()[2] + self.foot_center_to_floor
        thetas = self.inverseKinematicsRightFoot(right_foot_position)

        self.configuration[Links.RIGHT_LEG_1:Links.RIGHT_LEG_6+1] = thetas[0:6]

        # left leg
        left_foot_position = self.get_link_transformation(Links.TORSO, Links.LEFT_LEG_6)
        left_foot_position[2,3] = -self.pose.get_position()[2] + self.foot_center_to_floor
        thetas = self.inverseKinematicsLeftFoot(left_foot_position)

        pb_offset = tr.get_transform_from_euler([0, -0.06, 0])
        pb_offset.set_position([0, 0,  - self.foot_box[2]])
        final_pose = pb_offset @ self.pose
        pb.resetBasePositionAndOrientation(self.body, final_pose.get_position(), final_pose.get_orientation())
        positions = self.configuration

        links = list(range(0,18,1))
        pb.setJointMotorControlArray(bodyIndex=self.body, controlMode=pb.POSITION_CONTROL, jointIndices=links, targetPositions=positions)

        self.configuration[Links.LEFT_LEG_1:Links.LEFT_LEG_6 + 1] = thetas[0:6]

    def get_angles(self):
        """
        Function for getting all the feet angles (for now?) #TODO
        :return: All 12 angles in the dictionary form??? #TODO
        """
        return self.configuration

    def inverseKinematicsRightFoot(self, transformation):
        """
        #TODO
        :param transformation: #TODO
        :return: Motor angles for the right foot
        """
        transformation[0:3,3] = transformation[0:3,3] - self.torso_to_right_hip[0:3,3]
        invconf = np.linalg.inv(transformation)

        d3 = self.DH[2, 0]
        d4 = self.DH[3, 0]

        Xd = invconf[0, 3]
        Yd = invconf[1, 3]
        Zd = invconf[2, 3]

        if (np.linalg.norm([Xd, Yd, Zd]) > (d3 + d4)):
            print("IK Position Unreachable: Desired Distance: " + str(np.linalg.norm([Xd, Yd, Zd])) + ", Limited Distance: " + str(d3 + d4))
        assert(np.linalg.norm([Xd, Yd, Zd]) <= (d3 + d4))

        theta6 = -np.arctan2(Yd, Zd)
        tmp1 = Zd / np.cos(theta6)
        tmp2 = Xd
        D = ((((((tmp1 ** 2) + (tmp2 ** 2)) - (( d3 ** 2 ) + (d4 ** 2))) / 2 ) / d3 ) / d4 )
        tmp3 = np.arctan2(D, -np.sqrt(1 - (D ** 2)))

        tmpX = (tmp3 - (np.pi / 2))
        if tmpX < 0:
            tmpX = tmpX + (2 * np.pi)
        theta4 = -(np.unwrap([tmpX])[0])

        assert(theta4 < 4.6)

        alp = np.arctan2(tmp1, tmp2)
        beta = np.arctan2(-d3 * np.cos(tmp3), d4 + (d3 * np.sin(tmp3)))
        theta5 = np.pi / 2 - (alp - beta)

        H34 = tr.get_transform_from_dh(self.DH[3,0], self.DH[3,1], self.DH[3,2], theta4)
        H45 = tr.get_transform_from_dh(self.DH[4,0], self.DH[4,1], self.DH[4,2], theta5)
        H56 = tr.get_transform_from_dh(self.DH[5,0], self.DH[5,1], self.DH[5,2], theta6)
        H36 = np.matmul(H34, np.matmul(H45, H56))
        final_rotation = tr.get_transform_from_euler([0,np.pi/2,np.pi])
        H03 = np.matmul(np.matmul(transformation, final_rotation), np.linalg.inv(H36))
        assert (np.linalg.norm(H03[0:3, 3]) - d3 < 0.03)

        angles = tr.get_euler_from_rotation_matrix(np.linalg.inv(H03[0:3,0:3]), orientation='ZYX')
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
        right_hip_to_left_hip = self.get_link_transformation(Links.LEFT_LEG_1, Links.RIGHT_LEG_1)
        transformation[0: 3, 3] = transformation[0: 3, 3] + right_hip_to_left_hip[0: 3, 3]
        [theta1, theta2, theta3, theta4, theta5, theta6] = self.inverseKinematicsRightFoot(transformation)
        return [-theta1, -theta2, theta3, theta4, theta5, -theta6]

    def getPath(self, finishPosition, show=True):
        """
        Returns the trajectories for the robot's feet and crotch. The coordinates x,y will be used only.
        :param finishPosition: #TODO
        :return: #TODO
        """
        crotch = self.pose.get_position()
        finishPositionCoordinate = finishPosition.get_position()
        finishPositionCoordinate[2] = crotch[2]
        finishPosition.set_position(finishPositionCoordinate)

        self.robot_path = Robotpath(self.pose, finishPosition, self.foot_center_to_floor)
        if show:
            self.robot_path.show()
            self.robot_path.showTimingDiagram()
        # obj.rate = rateControl(1 / obj.robot_path.step_size); -- from findPath
        self.rate = 1 / self.robot_path.step_size
        self.period = self.robot_path.step_size

        self.current_step_time = 0
        return self.robot_path


    def stepPath(self, t):

        assert(t <= self.robot_path.duration())
        crotch_position = self.robot_path.crotchPosition(t) @ self.torso_offset

        [right_foot_position, left_foot_position] = self.robot_path.footPosition(t)
        torso_to_left_foot = np.matmul(np.linalg.inv(crotch_position), left_foot_position) # crotch_position \ left_foot_position;
        torso_to_right_foot = np.matmul(np.linalg.inv(crotch_position), right_foot_position) # crotch_position \ right_foot_position;

        print("--------------------------------------------------")
        thetas = self.inverseKinematicsRightFoot(torso_to_right_foot)
        print("Right foot: " + str([format(theta, '.3f') for theta in thetas ]).replace("'", ""))
        self.configuration[Links.RIGHT_LEG_1:Links.RIGHT_LEG_6+1] = thetas[0:6]
        thetas = self.inverseKinematicsLeftFoot(torso_to_left_foot)
        print("Left foot: " + str([format(theta, '.3f') for theta in thetas ]).replace("'", ""))
        self.configuration[Links.LEFT_LEG_1:Links.LEFT_LEG_6+1] = thetas[0:6]
        self.pose = crotch_position

    def applyRPYFeedback(self, rpy):
        f_off = (rpy[1] - self.rpy_current[1]) * 0.1 #TODO: where's rpy_current???
        fb_off = f_off * 0.15
        self.torso_offset = tr.get_transform_from_euler([0, f_off, 0]) # eul2tform([0, f_off, 0])
        self.torso_offset[0, 3] = -fb_off

    def calculate_angles(self, show=True):
        self.angles = []
        iterator = np.linspace(0, self.robot_path.duration(), num=math.ceil(self.robot_path.duration() / self.robot_path.step_size) + 1)
        if show:
            plot_angles = np.zeros((len(iterator), 18))
        i = 0
        for t in iterator:
            self.stepPath(t)
            self.angles.append((t, self.get_angles().copy()))
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
