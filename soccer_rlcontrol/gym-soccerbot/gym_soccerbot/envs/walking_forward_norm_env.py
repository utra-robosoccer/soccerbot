import numpy as np
from gym import spaces
from gym_soccerbot.envs.walking_forward_env import WalkingForward
from gym_soccerbot.envs.walking_forward_env_1 import WalkingForwardV1


class WalkingForwardNorm(WalkingForwardV1):
    def __init__(self, renders=False, reward_normal_factor=1e4):
        super().__init__(renders)
        self.joint_limit_high = self._joint_limit_high()
        self.joint_limit_low = self._joint_limit_low()
        self.reward_normal_factor = reward_normal_factor
        self.reward_range = [float(-2), float(2)]
        self.observation_space = spaces.Box(low=-1., high=1., shape=(self.observation_dim, ) , dtype=np.float32)
        self.action_space = spaces.Box(low=-1., high=1., shape=(self.JOINT_DIM, ), dtype=np.float32)

    def step(self, action):
        action = self.unnormalize(action, self.joint_limit_low, self.joint_limit_high)
        # assert np.logical_and.reduce(np.less_equal(action, self.joint_limit_high)), "Joint action max limit exceeded"
        # assert np.logical_and.reduce(np.greater_equal(action, self.joint_limit_low)), "Joint action min limit exceeded"
        action = np.clip(action, self.joint_limit_low, self.joint_limit_high)
        observation, reward, done, info = super().step(action)
        observation = self.normalize(observation, self.observation_limit_low, self.observation_limit_high)
        reward = reward / float(self.reward_normal_factor)
        return observation, reward, done, info

    def reset(self):
        observation = super().reset()
        observation = self.normalize(observation, self.observation_limit_low, self.observation_limit_high)
        return observation

    @staticmethod
    def normalize(actual, low_end, high_end):
        """
        Normalizes to [-1, 1]
        :param actual: to-be-normalized value
        :param low_end: s.e.
        :param high_end: s.e.
        :return: normalized value
        """
        val = actual - low_end
        val = 2 * val / (high_end - low_end)
        val = val - 1
        return val

    @staticmethod
    def unnormalize(norm, low_end, high_end):
        """
        Unnormalizes from [-1, 1]
        :param norm: to-be-unnormalized value
        :param low_end: s.e.
        :param high_end: s.e.
        :return: actual value
        """
        val = norm + 1
        val = (val / 2) * (high_end - low_end)
        val = val + low_end
        return val






