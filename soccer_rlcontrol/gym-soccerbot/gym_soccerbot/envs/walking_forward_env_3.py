import gym
from gym import spaces
from gym.utils import seeding

import os
import pybullet as pb
import pybullet_data
import math
import numpy as np
import enum
from pybullet_utils import bullet_client as bc
import logging

import gym_soccerbot

logger = logging.getLogger(__name__)

class RollingAvg:
    def __init__(self, window, threshold, angvel_coeff):
        self.window = window
        self.threshold = threshold
        self.angvel_coeff = angvel_coeff
        self.x_buffer = np.zeros((self.window,))
        self.y_buffer = np.zeros((self.window,))
        self.za_buffer = np.zeros((self.window,))
        self.count_down = self.window

    def update(self, new_x, new_y, new_za):
        self.x_buffer = np.roll(self.x_buffer, 1)
        self.y_buffer = np.roll(self.y_buffer, 1)
        self.za_buffer = np.roll(self.y_buffer, 1)
        self.x_buffer[0] = new_x ** 2
        self.y_buffer[0] = new_y ** 2
        self.za_buffer[0] = new_za ** 2
        if self.count_down != 0:
            self.count_down -= 1

    def get_stats(self):
        return np.average(self.x_buffer), np.average(self.y_buffer), np.average(self.za_buffer)

    def is_moving(self):
        if self.count_down > 0:
            return True

        stats = self.get_stats()
        if (stats[0] + stats[1] + (stats[2] * self.angvel_coeff)) < self.threshold:
            #  logger.info(f'x_agg: {stats[0]:.3f}, y_agg:{stats[1]:.3f}, za_agg: {stats[2]:.3f}')
            # print(f'x_agg: {stats[0]:.3f}, y_agg:{stats[1]:.3f}, za_agg: {stats[2]:.3f}')
            return False
        else:
            return True


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


class WalkingForwardV3(gym.Env):
    metadata = {'render.modes': ['human', 'rgb_array'], 'video.frames_per_second': 50}

    DTYPE = np.float32

    _POSE_DIM = 3
    _IMU_DIM = 6
    _FEET_DIM = 8
    _JOINT_DIM = 16


    #### Joint Limits HARD CODE
    _joint_limit_high = np.zeros(_JOINT_DIM)

    _joint_limit_high[Joints.RIGHT_LEG_1] = 0.2
    _joint_limit_high[Joints.RIGHT_LEG_2] = 0.2
    _joint_limit_high[Joints.RIGHT_LEG_3] = 0.67
    _joint_limit_high[Joints.RIGHT_LEG_4] = 0.05
    _joint_limit_high[Joints.RIGHT_LEG_5] = 0.5
    _joint_limit_high[Joints.RIGHT_LEG_6] = 0.15
    _joint_limit_high[Joints.LEFT_LEG_1] = 0.2
    _joint_limit_high[Joints.LEFT_LEG_2] = 0.2
    _joint_limit_high[Joints.LEFT_LEG_3] = 0.67
    _joint_limit_high[Joints.LEFT_LEG_4] = 0.05
    _joint_limit_high[Joints.LEFT_LEG_5] = 0.5
    _joint_limit_high[Joints.LEFT_LEG_6] = 0.15
    _joint_limit_high[Joints.RIGHT_ARM_1] = 0.95
    _joint_limit_high[Joints.RIGHT_ARM_2] = 0.8
    _joint_limit_high[Joints.LEFT_ARM_1] = 0.95
    _joint_limit_high[Joints.LEFT_ARM_2] = 0.8

    _joint_limit_high *= (np.pi)

    _joint_limit_low = np.zeros(_JOINT_DIM)

    _joint_limit_low[Joints.RIGHT_LEG_1] = 0.3
    _joint_limit_low[Joints.RIGHT_LEG_2] = 0.1
    _joint_limit_low[Joints.RIGHT_LEG_3] = 0.2
    _joint_limit_low[Joints.RIGHT_LEG_4] = 0.45
    _joint_limit_low[Joints.RIGHT_LEG_5] = 0.12
    _joint_limit_low[Joints.RIGHT_LEG_6] = 0.1
    _joint_limit_low[Joints.LEFT_LEG_1] = 0.3
    _joint_limit_low[Joints.LEFT_LEG_2] = 0.1
    _joint_limit_low[Joints.LEFT_LEG_3] = 0.2
    _joint_limit_low[Joints.LEFT_LEG_4] = 0.45
    _joint_limit_low[Joints.LEFT_LEG_5] = 0.12
    _joint_limit_low[Joints.LEFT_LEG_6] = 0.1
    _joint_limit_low[Joints.RIGHT_ARM_1] = 0.4
    _joint_limit_low[Joints.RIGHT_ARM_2] = 0.0
    _joint_limit_low[Joints.LEFT_ARM_1] = 0.4
    _joint_limit_low[Joints.LEFT_ARM_2] = 0.0

    _joint_limit_low *= (-np.pi)

    _AX_12_force = 1.5
    _MX_28_force = 2.5
    #### End of Joint Limits HARD CODE
    @classmethod
    def joint_limit_high_val(cls):
        return cls._joint_limit_high.copy()

    @classmethod
    def joint_limit_low_val(cls):
        return cls._joint_limit_low.copy()

    _STANDING_HEIGHT = 0.29 # 0.32
    _GRAVITY_VECTOR = [0, 0, -9.81]
    _CLOSENESS = 0.05 # in meters presumably

    # Action Space
    action_space = spaces.Box(low=_joint_limit_low, high=_joint_limit_high, dtype=DTYPE)

    # Observation Space
    _OBSERVATION_DIM = _JOINT_DIM + _IMU_DIM + _POSE_DIM + _FEET_DIM
    imu_limit = np.array([100.] * _IMU_DIM)
    pose_limit = np.array([3.] * _POSE_DIM)
    feet_limit = np.array([1.6] * _FEET_DIM)
    joint_limit = np.array([np.pi] * _JOINT_DIM)

    observation_limit_high = np.concatenate((joint_limit, imu_limit, pose_limit, feet_limit))
    observation_limit_low = np.concatenate((-joint_limit, -imu_limit, -pose_limit, -feet_limit))
    observation_space = spaces.Box(low=observation_limit_low, high=observation_limit_high, dtype=DTYPE)

    # Reward
    reward_limit_low = -1
    reward_limit_high = 1
    reward_range = [float(reward_limit_low), float(reward_limit_high)]

    # MISC
    _render_height = 200
    _render_width = 320
    def __init__(self, renders=False, warm_up=False, goal=[1, 0], seed=42):
        # start the bullet physics server
        self._renders = renders
        self._physics_client_id = -1

        self.goal_xy = goal
        self.WARM_UP = warm_up

        self.seed(seed=seed)
        self.reset()
        # self.st = RollingAvg(256, 0.01, 0.01)

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def _imu(self):
        p = self._p
        [quart_link, lin_vel, ang_vel] = p.getLinkState(bodyUniqueId=self.soccerbotUid, linkIndex=Links.IMU, computeLinkVelocity=1)[5:8]
        lin_vel = np.array(lin_vel, dtype=self.DTYPE)
        lin_acc = (lin_vel - self.prev_lin_vel) / self.timeStep
        lin_acc -= self._GRAVITY_VECTOR
        rot_mat = np.array(pb.getMatrixFromQuaternion(quart_link), dtype=self.DTYPE).reshape((3,3))
        lin_acc = np.matmul(rot_mat, lin_acc)
        ang_vel = np.array(ang_vel, dtype=self.DTYPE)
        self.prev_lin_vel = lin_vel
        return np.clip(np.concatenate((lin_acc, ang_vel)), -self.imu_limit, self.imu_limit)

    def _global_pos(self):
        p = self._p
        pos, _ = p.getBasePositionAndOrientation(self.soccerbotUid)
        return np.array(pos, dtype=self.DTYPE)

    def _feet(self):
        """
        Checks if 4 corners of the each feet are in contact with ground

        Indicies for looking from above on the feet plates:
          Left         Right
        4-------5    0-------1
        |   ^   |    |   ^   |      ^
        |   |   |    |   |   |      | : forward direction
        |       |    |       |
        6-------7    2-------3

        :return: int array of 8 contact points on both feet, 1: that point is touching the ground -1: otherwise
        """
        locations = [-1.] * self._FEET_DIM
        right_pts = pb.getContactPoints(bodyA=self.soccerbotUid, bodyB=self.planeUid, linkIndexA=Links.RIGHT_LEG_6)
        left_pts = pb.getContactPoints(bodyA=self.soccerbotUid, bodyB=self.planeUid, linkIndexA=Links.LEFT_LEG_6)
        right_center = np.array(pb.getLinkState(bodyUniqueId=self.soccerbotUid, linkIndex=Links.RIGHT_LEG_6)[4])
        left_center = np.array(pb.getLinkState(bodyUniqueId=self.soccerbotUid, linkIndex=Links.LEFT_LEG_6)[4])
        right_tr = np.array(pb.getMatrixFromQuaternion(
            pb.getLinkState(bodyUniqueId=self.soccerbotUid, linkIndex=Links.RIGHT_LEG_6)[5])
                                , dtype=self.DTYPE).reshape((3,3))
        left_tr = np.array(pb.getMatrixFromQuaternion(
            pb.getLinkState(bodyUniqueId=self.soccerbotUid, linkIndex=Links.LEFT_LEG_6)[5])
                                , dtype=self.DTYPE).reshape((3,3))
        for point in right_pts:
            index = np.signbit(np.matmul(right_tr, point[5] - right_center))[0:2]
            locations[index[1] + index[0] * 2] = 1.
        for point in left_pts:
            index = np.signbit(np.matmul(left_tr, point[5] - left_center))[0:2]
            locations[index[1] + (index[0] * 2) + 4] = 1.
        return np.array(locations)

    @classmethod
    def _standing_poses(cls, np_random=None, scale=0.6):
        standing_poses = [0.] * (cls._JOINT_DIM + 2)
        standing_poses[Joints.RIGHT_LEG_1] = 0.0
        standing_poses[Joints.RIGHT_LEG_2] = 0.05
        standing_poses[Joints.RIGHT_LEG_3] = 0.4
        standing_poses[Joints.RIGHT_LEG_4] = -0.8
        standing_poses[Joints.RIGHT_LEG_5] = 0.4
        standing_poses[Joints.RIGHT_LEG_6] = -0.05

        standing_poses[Joints.LEFT_LEG_1] = 0.0
        standing_poses[Joints.LEFT_LEG_2] = 0.05
        standing_poses[Joints.LEFT_LEG_3] = 0.4
        standing_poses[Joints.LEFT_LEG_4] = -0.8
        standing_poses[Joints.LEFT_LEG_5] = 0.4
        standing_poses[Joints.LEFT_LEG_6] = -0.05

        standing_poses[Joints.HEAD_1] = 0.0
        standing_poses[Joints.HEAD_2] = 0.0

        standing_poses[Joints.LEFT_ARM_1] = -0.5
        standing_poses[Joints.LEFT_ARM_2] = 2.8
        standing_poses[Joints.RIGHT_ARM_1] = -0.5
        standing_poses[Joints.RIGHT_ARM_2] = 2.8

        if np_random is not None:
            standing_poses[0:cls._JOINT_DIM] = standing_poses[0:cls._JOINT_DIM] + \
                    scale * np_random.uniform(-0., 1., np.size(standing_poses[0:cls._JOINT_DIM])) * \
                                               (cls._joint_limit_high - standing_poses[0:cls._JOINT_DIM]) + \
                    scale * np_random.uniform(-0., 1., np.size(standing_poses[0:cls._JOINT_DIM])) * \
                                               (cls._joint_limit_low - standing_poses[0:cls._JOINT_DIM])
            standing_poses[0:cls._JOINT_DIM] = np.clip(standing_poses[0:cls._JOINT_DIM],
                                                       cls._joint_limit_low,
                                                       cls._joint_limit_high)
            return standing_poses
        return np.array(standing_poses)

    @classmethod
    def _standing_poses2(cls, np_random=None, scale=0.6):
        hardcoded_angles = \
                [2.827433388230814, 0, 2.827433388230814, 0, 0.0016725139646804887, 0.0960802594363035,
                 0.5793527147847758, -1.2298418591359193, 0.6509027637698601, -0.06231839374272367,
                 0.00030337575088856816, -0.10382957319901398, 0.578543334661491, -1.2276210689082445,
                 0.6495736803093827, 0.06684757628616049, 0, 0]
        standing_poses = np.array(hardcoded_angles)
        if np_random is not None:
            standing_poses[0:cls._JOINT_DIM] = standing_poses[0:cls._JOINT_DIM] + \
                    scale * np_random.uniform(-0., 1., np.size(standing_poses[0:cls._JOINT_DIM])) * \
                                               (cls._joint_limit_high - standing_poses[0:cls._JOINT_DIM]) + \
                    scale * np_random.uniform(-0., 1., np.size(standing_poses[0:cls._JOINT_DIM])) * \
                                               (cls._joint_limit_low - standing_poses[0:cls._JOINT_DIM])
            standing_poses[0:cls._JOINT_DIM] = np.clip(standing_poses[0:cls._JOINT_DIM],
                                                       cls._joint_limit_low,
                                                       cls._joint_limit_high)
        return standing_poses


    def step(self, action):
        p = self._p

        # Collect Sensor Data
        feet = self._feet()
        imu = self._imu()

        # CLIP ACTIONS
        # action = np.clip(action, self._joint_limit_low, self._joint_limit_high)
        # MX-28s
        p.setJointMotorControlArray(bodyIndex=self.soccerbotUid,
                                    controlMode=pb.POSITION_CONTROL,
                                    jointIndices=list(range(Joints.LEFT_LEG_1, Joints.HEAD_1, 1)),
                                    targetPositions=action[Joints.LEFT_LEG_1:Joints.HEAD_1],
                                    # targetVelocities=[2*(5/6)*np.pi]*self._JOINT_DIM,
                                    # positionGains=[4]*self._JOINT_DIM,
                                    # velocityGains=[0]*self._JOINT_DIM,
                                    forces=[self._MX_28_force] * (Joints.HEAD_1 - Joints.LEFT_LEG_1))
        # AX-12s
        p.setJointMotorControlArray(bodyIndex=self.soccerbotUid,
                                    controlMode=pb.POSITION_CONTROL,
                                    jointIndices=list(range(Joints.LEFT_ARM_1, Joints.LEFT_LEG_1, 1)),
                                    targetPositions=action[Joints.LEFT_ARM_1:Joints.LEFT_LEG_1],
                                    # targetVelocities=[2*(5/6)*np.pi]*self._JOINT_DIM,
                                    # positionGains=[4]*self._JOINT_DIM,
                                    # velocityGains=[0]*self._JOINT_DIM,
                                    forces=[self._AX_12_force] * (Joints.LEFT_LEG_1 - Joints.LEFT_ARM_1))

        # 120Hz - Step Simulation
        p.stepSimulation()
        p.stepSimulation()

        joint_states = p.getJointStates(self.soccerbotUid, list(range(0, self._JOINT_DIM, 1)))
        joints_pos = np.array([state[0] for state in joint_states], dtype=self.DTYPE)

        # Construct Observation
        observation = np.concatenate((joints_pos, imu, self._global_pos(), feet))

        ## Calculate Reward, Done, Info
        # Calculate Velocity direction field
        [lin_vel, _] = p.getBaseVelocity(self.soccerbotUid)
        lin_vel = np.array(lin_vel, dtype=self.DTYPE)[0:2]
        distance_unit_vec = (self.goal_xy - self._global_pos()[0:2]) \
                            / np.linalg.norm(self.goal_xy - self._global_pos()[0:2])
        velocity_reward = np.dot(distance_unit_vec, lin_vel)
        info = dict(end_cond="None")
        # Fall
        if self._global_pos()[2] < 0.22: #HARDCODE (self._STANDING_HEIGHT / 2): # check z component
            done = True
            reward = -1
            info['end_cond'] = "Robot Fell"
        # Close to the Goal
        elif np.linalg.norm(self._global_pos()[0:2] - self.goal_xy) < self._CLOSENESS:
            done = True
            reward = 1e-1
            info['end_cond'] = "Goal Reached"
        # Out of Bound
        elif np.linalg.norm(self._global_pos()[0:2] - self.goal_xy) > (2 *np.linalg.norm(self.goal_xy)): # out of bound
            done = True
            reward = -1
            info['end_cond'] = "Robot Out"
        # Normal case
        else:
            done = False
            reward = velocity_reward
        return observation, reward, done, info

    def reset(self):
        if self._physics_client_id < 0:
            if self._renders:
                self._p = bc.BulletClient(connection_mode=pb.GUI)
                self._p.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 0)
            else:
                self._p = bc.BulletClient()
                # self._p.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 0)
            self._physics_client_id = self._p._client

            p = self._p
            p.resetSimulation()

            urdfBotPath = gym_soccerbot.getModelPath()
            self.soccerbotUid = p.loadURDF(urdfBotPath,
                                           useFixedBase=False,
                                           useMaximalCoordinates=False,
                                           basePosition=[0, 0, self._STANDING_HEIGHT],
                                           baseOrientation=[0., 0., 0., 1.],
                                           flags=pb.URDF_USE_INERTIA_FROM_FILE
                                                 | pb.URDF_USE_MATERIAL_COLORS_FROM_MTL
                                                 | pb.URDF_USE_SELF_COLLISION
                                                 | pb.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS
                                           )
            # load ramp
            urdfRootPath = pybullet_data.getDataPath()
            self.planeUid = p.loadURDF(os.path.join(urdfRootPath, "plane_implicit.urdf"),
                                       useMaximalCoordinates=True,
                                       basePosition=[0, 0, 0])

            # TODO change dynamics ...
            # for i in range(p.getNumJoints(bodyUniqueId=self.soccerbotUid)):
                # print(p.getJointInfo(bodyUniqueId=self.soccerbotUid, jointIndex=i)[1])
            p.changeDynamics(self.planeUid, linkIndex=-1, lateralFriction=0.9, spinningFriction=0.9, rollingFriction=0)
            # p.changeDynamics(self.soccerbotUid, linkIndex=Links.IMU, mass=0.01, localInertiaDiagonal=[7e-7, 7e-7, 7e-7])
            # p.changeDynamics(self.soccerbotUid, linkIndex=Links.IMU, mass=0., localInertiaDiagonal=[0., 0., 0.])
            '''
            p.changeDynamics(bodyUniqueId=self.soccerbotUid, linkIndex=Links.RIGHT_LEG_6,
                             frictionAnchor=1, lateralFriction=1,
                             rollingFriction=1, spinningFriction=1)
            p.changeDynamics(bodyUniqueId=self.soccerbotUid, linkIndex=Links.RIGHT_LEG_6,
                             frictionAnchor=1, lateralFriction=1,
                             rollingFriction=1, spinningFriction=1)
            '''
            # Simulation Physics General Settings
            self.timeStep = 1./240
            p.setTimeStep(self.timeStep)
            p.setGravity(*self._GRAVITY_VECTOR)
            p.setRealTimeSimulation(0)  # To manually step simulation later

        p = self._p

        # Bring robot back to origin
        p.resetBasePositionAndOrientation(self.soccerbotUid, [0, 0, self._STANDING_HEIGHT], [0., 0., 0., 1.])
        p.resetBaseVelocity(self.soccerbotUid, [0, 0, 0], [0, 0, 0])
        self.prev_lin_vel = np.array([0, 0, 0])

        #p.resetJointStates(self.soccerbotUid, list(range(0, 18, 1)), 0)
        #pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 1)
        #standing_poses = [0] * (self._JOINT_DIM + 2)
        standing_poses = self._standing_poses(self.np_random)

        # MX-28s:
        for i in range(Joints.LEFT_LEG_1, Joints.HEAD_1):
            p.changeDynamics(self.soccerbotUid, i,
                               jointLowerLimit=-self.joint_limit[i], jointUpperLimit=self.joint_limit[i],
                               jointLimitForce=self._MX_28_force)
            p.resetJointState(self.soccerbotUid, i, standing_poses[i])
        # AX-12s:
        for i in range(Joints.LEFT_ARM_1, Joints.LEFT_LEG_1):
            p.changeDynamics(self.soccerbotUid, i,
                               jointLowerLimit=-self.joint_limit[i], jointUpperLimit=self.joint_limit[i],
                               jointLimitForce=self._AX_12_force)
            p.resetJointState(self.soccerbotUid, i, standing_poses[i])
        p.changeDynamics(self.soccerbotUid, Joints.HEAD_1,
                           jointLowerLimit=-np.pi, jointUpperLimit=np.pi,
                           jointLimitForce=self._AX_12_force)
        p.resetJointState(self.soccerbotUid, Joints.HEAD_1, standing_poses[Joints.HEAD_1])
        p.changeDynamics(self.soccerbotUid, Joints.HEAD_2,
                           jointLowerLimit=-np.pi, jointUpperLimit=np.pi,
                           jointLimitForce=self._AX_12_force)
        p.resetJointState(self.soccerbotUid, Joints.HEAD_2, standing_poses[Joints.HEAD_2])


        # WARM UP SIMULATION
        if self.WARM_UP:
            warm_up = self.np_random.randint(0, 21)
            for _ in range(warm_up):
                p.stepSimulation()
                p.stepSimulation()

        # Get Observation
        # self.st = RollingAvg(256, 0.01, 0.01)
        self.prev_lin_vel = np.array([0, 0, 0])
        feet = self._feet()
        imu = self._imu()
        joint_states = p.getJointStates(self.soccerbotUid, list(range(0, self._JOINT_DIM, 1)))
        joints_pos = np.array([state[0] for state in joint_states], dtype=self.DTYPE)
        start_pos = np.array([0, 0, self._STANDING_HEIGHT], dtype=self.DTYPE)
        observation = np.concatenate((joints_pos, imu, start_pos, feet))

        if self._renders:
            pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 1)
        return observation

    def render(self, mode='human'):
        if mode == "human":
            self._renders = True
        if mode != "rgb_array":
            return np.array([])
        base_pos, orn = self._p.getBasePositionAndOrientation(self.soccerbotUid)
        base_pos = np.asarray(base_pos)
        # TODO tune parameters
        # track the position
        base_pos[1] += 0.3
        rpy = self._p.getEulerFromQuaternion(orn)  # rpy, in radians
        rpy = 180 / np.pi * np.asarray(rpy)  # convert rpy in degrees

        self._cam_dist = 3
        self._cam_pitch = 0.3
        self._cam_yaw = 0
        if True: # not (self._p is None):
            view_matrix = self._p.computeViewMatrixFromYawPitchRoll(
                cameraTargetPosition=base_pos,
                distance=self._cam_dist,
                yaw=self._cam_yaw,
                pitch=self._cam_pitch,
                roll=0,
                upAxisIndex=1)
            proj_matrix = self._p.computeProjectionMatrixFOV(fov=60,
                                                             aspect=float(self._render_width) / self._render_height,
                                                             nearVal=0.1,
                                                             farVal=100.0)
            (_, _, px, _, _) = self._p.getCameraImage(
                width=self._render_width,
                height=self._render_height,
                renderer=self._p.ER_BULLET_HARDWARE_OPENGL,
                viewMatrix=view_matrix,
                projectionMatrix=proj_matrix)
            '''
            self._p.resetDebugVisualizerCamera(
              cameraDistance=2 * self._cam_dist,
              cameraYaw=self._cam_yaw,
              cameraPitch=self._cam_pitch,
              cameraTargetPosition=base_pos
            )
            '''
            pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 1)
        else:
            px = np.array([[[255, 255, 255, 255]] * self._render_width] * self._render_height, dtype=np.uint8)
        rgb_array = np.array(px, dtype=np.uint8)
        rgb_array = np.reshape(np.array(px), (self._render_height, self._render_width, -1))
        rgb_array = rgb_array[:, :, :3]
        return rgb_array

    def close(self):
        if self._physics_client_id >= 0:
            self._p.disconnect()
        self._physics_client_id = -1
