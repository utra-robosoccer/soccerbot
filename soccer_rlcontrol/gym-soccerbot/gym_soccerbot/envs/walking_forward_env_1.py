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


class WalkingForwardV1(gym.Env):
    metadata = {'render.modes': ['human', 'rgb_array'], 'video.frames_per_second': 50}


    def __init__(self, renders=False, dtype=np.float64, warm_up=True, reward_scale=1e2, goal=[1, 0]):
        self.dtype = dtype
        self.reward_scale = reward_scale

        # start the bullet physics server
        self._renders = renders
        self._render_height = 200
        self._render_width = 320
        self._physics_client_id = -1

        self.prev_lin_vel = np.array([0, 0, 0])
        self.gravity = [0, 0, -9.81]
        self.STANDING_HEIGHT = 0.35 #0.32
        self.goal_xy = goal

        self.WARM_UP = warm_up

        # TODO Change action space
        self.JOINT_DIM = 16
        self.joint_limit_high = self._joint_limit_high()
        self.joint_limit_low = self._joint_limit_low()

        self.action_space = spaces.Box(low=self.joint_limit_low, high=self.joint_limit_high, dtype=self.dtype) #, shape=(1, self.JOINT_DIM)

        # TODO Change observation space

        self.POSE_DIM = 3
        self.IMU_DIM = 6
        self.FEET_DIM = 8
        self.observation_dim = self.JOINT_DIM + self.IMU_DIM + self.POSE_DIM + self.FEET_DIM

        self.imu_limit = np.array([100.] * self.IMU_DIM)
        self.pose_limit = np.array([3.] * self.POSE_DIM)
        self.feet_limit = np.array([1.6] * self.FEET_DIM)
        self.joint_limit = np.array([np.pi] * self.JOINT_DIM)

        self.observation_limit_high = np.concatenate((self.joint_limit, self.imu_limit, self.pose_limit, self.feet_limit))
        self.observation_limit_low = np.concatenate((-self.joint_limit, -self.imu_limit, -self.pose_limit, -self.feet_limit))
        self.observation_space = spaces.Box(low=self.observation_limit_low, high=self.observation_limit_high, dtype=self.dtype) #shape=(1, observation_dim),

        self.seed()
        self.reset()
        self.viewer = None
        self._configure()
        # self.st = RollingAvg(256, 0.01, 0.01)

    def do_render(self, rend):
        self._renders = bool(rend)

    def _configure(self, display=None):
        self.display = display

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def _imu(self):
        p = self._p
        [quart_link, lin_vel, ang_vel] = p.getLinkState(bodyUniqueId=self.soccerbotUid, linkIndex=Links.IMU, computeLinkVelocity=1)[5:8]
        # [lin_vel, ang_vel] = p.getLinkState(bodyUniqueId=self.soccerbotUid, linkIndex=Links.HEAD_1, computeLinkVelocity=1)[6:8]
        # print(p.getLinkStates(bodyUniqueId=self.soccerbotUid, linkIndices=range(0,18,1), computeLinkVelocity=1))
        # lin_vel = [0, 0, 0]
        # ang_vel = [0, 0, 0]
        #p.getBaseVelocity(self.soccerbotUid)
        self.st.update(lin_vel[0], lin_vel[1], ang_vel[2])
        lin_vel = np.array(lin_vel, dtype = np.float32)

        lin_acc = (lin_vel - self.prev_lin_vel) / self.timeStep
        lin_acc -= self.gravity
        rot_mat = np.array(pb.getMatrixFromQuaternion(quart_link), dtype=self.dtype).reshape((3,3))
        lin_acc = np.matmul(rot_mat, lin_acc)
        ang_vel = np.array(ang_vel, dtype=self.dtype)
        self.prev_lin_vel = lin_vel
        #print(f'lin_acc = {lin_acc}', end="\t\t")
        #print(f'lin_acc = {lin_acc}')
        #print(f'ang_vel = {ang_vel}')
        return np.clip(np.concatenate((lin_acc, ang_vel)), -self.imu_limit, self.imu_limit)

    def _global_pos(self):
        p = self._p
        pos, _ = p.getBasePositionAndOrientation(self.soccerbotUid)
        return np.array(pos, dtype=self.dtype)

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
        locations = [-1.] * self.FEET_DIM
        right_pts = pb.getContactPoints(bodyA=self.soccerbotUid, bodyB=self.planeUid, linkIndexA=Links.RIGHT_LEG_6)
        left_pts = pb.getContactPoints(bodyA=self.soccerbotUid, bodyB=self.planeUid, linkIndexA=Links.LEFT_LEG_6)
        right_center = np.array(pb.getLinkState(bodyUniqueId=self.soccerbotUid, linkIndex=Links.RIGHT_LEG_6)[4])
        left_center = np.array(pb.getLinkState(bodyUniqueId=self.soccerbotUid, linkIndex=Links.LEFT_LEG_6)[4])
        right_tr = np.array(pb.getMatrixFromQuaternion(
            pb.getLinkState(bodyUniqueId=self.soccerbotUid, linkIndex=Links.RIGHT_LEG_6)[5])
                                , dtype=self.dtype).reshape((3,3))
        left_tr = np.array(pb.getMatrixFromQuaternion(
            pb.getLinkState(bodyUniqueId=self.soccerbotUid, linkIndex=Links.LEFT_LEG_6)[5])
                                , dtype=self.dtype).reshape((3,3))
        for point in right_pts:
            index = np.signbit(np.matmul(right_tr, point[5] - right_center))[0:2]
            locations[index[1] + index[0] * 2] = 1.
        for point in left_pts:
            index = np.signbit(np.matmul(left_tr, point[5] - left_center))[0:2]
            locations[index[1] + (index[0] * 2) + 4] = 1.
        return np.array(locations)

    def _joint_limit_high(self):
        joint_limit_high = np.array([0] * self.JOINT_DIM)

        joint_limit_high[Joints.RIGHT_LEG_1] = 0.2
        joint_limit_high[Joints.RIGHT_LEG_2] = 0.2
        joint_limit_high[Joints.RIGHT_LEG_3] = 0.67
        joint_limit_high[Joints.RIGHT_LEG_4] = 0.05
        joint_limit_high[Joints.RIGHT_LEG_5] = 0.5
        joint_limit_high[Joints.RIGHT_LEG_6] = 0.15

        joint_limit_high[Joints.LEFT_LEG_1] = 0.2
        joint_limit_high[Joints.LEFT_LEG_2] = 0.2
        joint_limit_high[Joints.LEFT_LEG_3] = 0.67
        joint_limit_high[Joints.LEFT_LEG_4] = 0.05
        joint_limit_high[Joints.LEFT_LEG_5] = 0.5
        joint_limit_high[Joints.LEFT_LEG_6] = 0.15

        joint_limit_high[Joints.LEFT_ARM_1] = 0.95
        joint_limit_high[Joints.LEFT_ARM_2] = 0.8

        joint_limit_high[Joints.RIGHT_ARM_1] = 0.95
        joint_limit_high[Joints.RIGHT_ARM_2] = 0.8

        return joint_limit_high * np.pi

    def _joint_limit_low(self):
        joint_limit_low = np.array([0] * self.JOINT_DIM)

        joint_limit_low[Joints.RIGHT_LEG_1] = 0.3
        joint_limit_low[Joints.RIGHT_LEG_2] = 0.1
        joint_limit_low[Joints.RIGHT_LEG_3] = 0.2
        joint_limit_low[Joints.RIGHT_LEG_4] = 0.45
        joint_limit_low[Joints.RIGHT_LEG_5] = 0.12
        joint_limit_low[Joints.RIGHT_LEG_6] = 0.1

        joint_limit_low[Joints.LEFT_LEG_1] = 0.3
        joint_limit_low[Joints.LEFT_LEG_2] = 0.1
        joint_limit_low[Joints.LEFT_LEG_3] = 0.2
        joint_limit_low[Joints.LEFT_LEG_4] = 0.45
        joint_limit_low[Joints.LEFT_LEG_5] = 0.12
        joint_limit_low[Joints.LEFT_LEG_6] = 0.1

        joint_limit_low[Joints.LEFT_ARM_1] = 0.4
        joint_limit_low[Joints.LEFT_ARM_2] = 0.0

        joint_limit_low[Joints.RIGHT_ARM_1] = 0.4
        joint_limit_low[Joints.RIGHT_ARM_2] = 0.0

        return joint_limit_low * (-np.pi)


    def _standing_poses(self, random=False):
        standing_poses = [None] * (self.JOINT_DIM + 2)
        standing_poses[Joints.RIGHT_LEG_1] = 0.0
        standing_poses[Joints.RIGHT_LEG_2] = 0.05
        standing_poses[Joints.RIGHT_LEG_3] = 0.0
        standing_poses[Joints.RIGHT_LEG_4] = 0.0
        standing_poses[Joints.RIGHT_LEG_5] = 0.0
        standing_poses[Joints.RIGHT_LEG_6] = -0.05

        standing_poses[Joints.LEFT_LEG_1] = 0.0
        standing_poses[Joints.LEFT_LEG_2] = 0.05
        standing_poses[Joints.LEFT_LEG_3] = 0.0
        standing_poses[Joints.LEFT_LEG_4] = 0.0
        standing_poses[Joints.LEFT_LEG_5] = 0.0
        standing_poses[Joints.LEFT_LEG_6] = -0.05

        standing_poses[Joints.HEAD_1] = 0.0
        standing_poses[Joints.HEAD_2] = 0.0

        standing_poses[Joints.LEFT_ARM_1] = -0.5
        standing_poses[Joints.LEFT_ARM_2] = 2.8
        standing_poses[Joints.RIGHT_ARM_1] = -0.5
        standing_poses[Joints.RIGHT_ARM_2] = 2.8

        if random:
            standing_poses[0:self.JOINT_DIM] = standing_poses[0:self.JOINT_DIM] + \
                    0.4 * self.np_random.uniform(-0., 1., np.size(standing_poses[0:self.JOINT_DIM])) * (self._joint_limit_high() - standing_poses[0:self.JOINT_DIM]) + \
                    0.4 * self.np_random.uniform(-0., 1., np.size(standing_poses[0:self.JOINT_DIM])) * (self._joint_limit_low() - standing_poses[0:self.JOINT_DIM])
            standing_poses[0:self.JOINT_DIM] = np.clip(standing_poses[0:self.JOINT_DIM], self.joint_limit_low, self.joint_limit_high)
        return standing_poses


    def step(self, action):
        p = self._p

        feet = self._feet()
        imu = self._imu()

        # CLIP ACTIONS
        action = np.clip(action, self.joint_limit_low, self.joint_limit_high)

        p.setJointMotorControlArray(bodyIndex=self.soccerbotUid,
                                    controlMode=pb.POSITION_CONTROL,
                                    jointIndices=list(range(0, self.JOINT_DIM, 1)),
                                    targetPositions=action,
                                    # targetVelocities=[2*(5/6)*np.pi]*self.JOINT_DIM,
                                    # positionGains=[4]*self.JOINT_DIM,
                                    # velocityGains=[0]*self.JOINT_DIM,
                                    forces=[2.3]*self.JOINT_DIM)
        # 120Hz
        p.stepSimulation()
        p.stepSimulation()

        joint_states = p.getJointStates(self.soccerbotUid, list(range(0, self.JOINT_DIM, 1)))
        joints_pos = np.array([state[0] for state in joint_states], dtype=self.dtype)

        # TODO Observation
        observation = np.concatenate((joints_pos, imu, self._global_pos(), feet))

        # TODO calculate done
        # TODO calculate reward

        [lin_vel, _] = p.getBaseVelocity(self.soccerbotUid)
        lin_vel = np.array(lin_vel, dtype=self.dtype)[0:2]
        distance_unit_vec = (self.goal_xy - self._global_pos()[0:2]) \
                            / np.linalg.norm(self.goal_xy - self._global_pos()[0:2])
        velocity_reward = 1000 * np.dot(distance_unit_vec, lin_vel)

        #time_penalty = -1
        info = dict(end_cond="None")
        # Fall
        if self._global_pos()[2] < 0.22: #HARDCODE (self.STANDING_HEIGHT / 2): # check z component
            done = True
            reward = -1e4
            info['end_cond'] = "Robot Fell"
        else:
            # Close to the Goal
            if np.linalg.norm(self._global_pos()[0:2] - self.goal_xy) < 0.05:
                done = True
                #reward = 1 / np.linalg.norm(self._global_pos()[0:2] - self.goal_xy)
                reward = 1e3
                info['end_cond'] = "Goal Reached"
            # Out of Bound
            elif np.linalg.norm(self._global_pos()[0:2] - self.goal_xy) > (2 *np.linalg.norm(self.goal_xy)): # out of bound
                done = True
                reward = -1e4
                info['end_cond'] = "Robot Out"
            # Not moving for a long time
            elif not self.st.is_moving():
                done = True
                reward = -1e4
                info['end_cond'] = "Not moving"
            # Normal case
            else:
                done = False
                reward = velocity_reward
                # reward = time_penalty + velocity_reward
                #print(f'x = {self._global_pos()[0]}, y = {self._global_pos()[1]}')
        reward /= self.reward_scale
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
            # load ramp

            urdfBotPath = gym_soccerbot.getModelPath()
            self.soccerbotUid = p.loadURDF(urdfBotPath,
                                           useFixedBase=False,
                                           useMaximalCoordinates=False,
                                           basePosition=[0, 0, self.STANDING_HEIGHT],
                                           baseOrientation=[0., 0., 0., 1.],
                                           flags=pb.URDF_USE_INERTIA_FROM_FILE
                                                 | pb.URDF_USE_MATERIAL_COLORS_FROM_MTL
                                                 | pb.URDF_USE_SELF_COLLISION
                                                 | pb.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS
                                           )

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
            # TODO change timestep ...
            self.timeStep = 1./240
            p.setTimeStep(self.timeStep)

            p.setGravity(0, 0, -9.81)
            self.gravity = [0, 0, -9.81]
            p.setRealTimeSimulation(0)  # To manually step simulation later
        p = self._p

        # TODO reset joint state
        p.resetBasePositionAndOrientation(self.soccerbotUid, [0, 0, self.STANDING_HEIGHT], [0., 0., 0., 1.])
        p.resetBaseVelocity(self.soccerbotUid, [0, 0, 0], [0, 0, 0])
        self.prev_lin_vel = np.array([0, 0, 0])

        #p.resetJointStates(self.soccerbotUid, list(range(0, 18, 1)), 0)
        #pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 1)
        #standing_poses = [0] * (self.JOINT_DIM + 2)
        standing_poses = self._standing_poses(random=True)
        for i in range(self.JOINT_DIM + 2):
            p.resetJointState(self.soccerbotUid, i, standing_poses[i])

        # WARM UP SIMULATION
        if self.WARM_UP:
            warm_up = self.np_random.random_integers(0, 10)
        for _ in range(warm_up):
            p.stepSimulation()
            p.stepSimulation()

        # TODO set state???

        # TODO get observation
        self.st = RollingAvg(256, 0.01, 0.01)
        feet = self._feet()
        imu = self._imu()
        joint_states = p.getJointStates(self.soccerbotUid, list(range(0, self.JOINT_DIM, 1)))
        joints_pos = np.array([state[0] for state in joint_states], dtype=self.dtype)
        start_pos = np.array([0, 0, self.STANDING_HEIGHT], dtype=self.dtype)
        observation = np.concatenate((joints_pos, imu, start_pos, feet))

        # Roll Stats

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
        if not (self._p is None):
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

            self._p.resetDebugVisualizerCamera(
              cameraDistance=2 * self._cam_dist,
              cameraYaw=self._cam_yaw,
              cameraPitch=self._cam_pitch,
              cameraTargetPosition=base_pos
            )
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
