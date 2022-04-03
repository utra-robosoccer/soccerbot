import numpy as np
from gym import spaces, Env
from gym.envs.registration import spec, load
from time import sleep


class WalkingForwardNormAgn(Env):
    def __init__(self, reward_plus_range=1e1,
                 action_plus_range=1e1, observation_plus_range=1e1, env_name=None, **kwargs):
        if type(env_name).__name__ == 'str':
            env_spec = spec(env_name)
        elif env_name is None:
            raise ValueError('Error: No envs has been passed.')
        self.env = env_spec.make(**kwargs)
        # assert 0, type(env)


        if "dtype" in kwargs.keys():
            self.dtype = eval(kwargs["dtype"])
        else:
            self.dtype = np.float32

        # Observation Space
        self.observation_plus_range = observation_plus_range
        self.observation_space = spaces.Box(low=-self.observation_plus_range, high=self.observation_plus_range,
                                            shape=self.env.observation_space.shape , dtype=self.dtype)
        # Action Space
        self.action_plus_range = action_plus_range
        self.action_space = spaces.Box(low=-self.action_plus_range, high=self.action_plus_range,
                                       shape=self.env.action_space.shape, dtype=self.dtype)
        # Reward
        self.reward_plus_range = self.dtype(reward_plus_range)
        self.reward_range = [-reward_plus_range, reward_plus_range]

    def step(self, action):
        action = self.denormalize(action,
                                  self.env.action_space.low, self.env.action_space.high,
                                  self.action_plus_range)
        # assert np.logical_and.reduce(np.less_equal(action, self.joint_limit_high)), "Joint action max limit exceeded"
        # assert np.logical_and.reduce(np.greater_equal(action, self.joint_limit_low)), "Joint action min limit exceeded"
        # action = np.clip(action, self.env.joint_limit_low, self.env.joint_limit_high)
        observation, reward, done, info = self.env.step(action)
        observation = self.normalize(observation,
                                     self.env.observation_limit_low, self.env.observation_limit_high,
                                     self.observation_plus_range)
        observation = np.clip(observation, -self.observation_plus_range, self.observation_plus_range)
        # reward = reward / float(self.reward_normal_factor)
        reward = self.normalize(reward,
                                self.env.reward_limit_low, self.env.reward_limit_high,
                                self.reward_plus_range)
        reward = np.clip(reward, -self.reward_plus_range, self.reward_plus_range)
        return observation, reward, done, info

    def reset(self):
        observation = self.env.reset()
        observation = self.normalize(observation,
                                     self.env.observation_limit_low, self.env.observation_limit_high,
                                     self.observation_plus_range)
        observation = np.clip(observation, -self.observation_plus_range, self.observation_plus_range)
        return observation

    def render(self, **kwargs):
        sleep(0.0041)
        return self.env.render(**kwargs)

    def normalize(self, actual, low_end, high_end, scale):
        """
        Normalizes to [-1, 1]
        :param actual: to-be-normalized value
        :param low_end: s.e.
        :param high_end: s.e.
        :return: normalized value
        """
        val = self.dtype(actual - low_end)
        val = 2 * val / (high_end - low_end)
        val = val - 1
        val *= self.dtype(scale)
        return self.dtype(val)

    def denormalize(self, norm, low_end, high_end, scale):
        """
        Unnormalizes from [-1, 1]
        :param norm: to-be-unnormalized value
        :param low_end: s.e.
        :param high_end: s.e.
        :return: actual value
        """
        val = norm / self.dtype(scale)
        val += 1
        val = (val / 2) * (high_end - low_end)
        val = val + low_end
        return self.dtype(val)
