#!/usr/bin/env python3
import os
import time
from unittest import TestCase

import gym
import matplotlib.pyplot as plt
import numpy as np
from pyvirtualdisplay import Display
from soccer_rlcontrol.envs import *

env = gym.make("AcrobotWalker-v0", render_mode="human")
rng = np.random.RandomState(0)

bin_size = 15
bins = [
    np.linspace(env.observation_space.low[0], env.observation_space.high[0], bin_size),
    np.linspace(env.observation_space.low[1], env.observation_space.high[1], bin_size),
    np.linspace(env.observation_space.low[2], env.observation_space.high[2], bin_size),
    np.linspace(env.observation_space.low[3], env.observation_space.high[3], bin_size),
    np.array([0, 1]),  # Left or right foot
]
action_bin_size = 15
action_bins = np.linspace(env.action_space.low[0], env.action_space.high[0], action_bin_size)

Q = np.random.uniform(low=-1, high=1, size=([bin_size] * 4 + [2] + [action_bin_size]))


def obs2state(state):
    state = state[0:5]
    index = []
    for i in range(len(state) - 1):
        index.append(np.digitize(state[i], bins[i]) - 1)
    index.append(int(state[4] % 2))
    return tuple(index)


def test_policy(policy_fun):
    obs = env.reset()
    s = obs2state(obs)
    a = policy_fun(s)

    prev_screen = env.render(mode="rgb_array")
    plt.imshow(prev_screen)

    i = 0
    while True:
        obs_next, r, done, info = env.step(action_bins[a])
        s_next = obs2state(obs_next)
        a_next = policy_fun(s_next)

        screen = env.render(mode="rgb_array")
        plt.imshow(screen)

        obs, s, a = obs_next, s_next, a_next
        i = i + 1
        if done:
            print("Acrobot lasted for " + str(i) + " steps")
            break
    env.close()
    return i


def run_policy(policy_fun):
    obs = env.reset()
    s = obs2state(obs)
    a = policy_fun(s)

    while True:
        obs_next, r, done, info = env.step(action_bins[a])
        s_next = obs2state(obs_next)
        a_next = policy_fun(s_next)
        yield s, a, r, s_next, a_next, done
        obs, s, a = obs_next, s_next, a_next
        if done:
            break

    env.close()


class Test(TestCase):
    def setUp(self) -> None:
        super().setUpClass()

    def test_acrobot_basic(self):
        env = gym.make("AcrobotWalker-v0", render_mode="human")
        env.reset()
        for i in range(1, 45):
            (obs, reward, terminal, info) = env.step(0.005)
            env.render()
        for i in range(1, 1000):
            (obs, reward, terminal, info) = env.step(-0.055)
            if terminal:
                break
            env.render()

    def test_acrobot_sarsa(self):
        def train(alpha=0.15, eps=0.2, gamma=0.995, episodes=50000):
            print("Starting training with alpha={} eps={} gamma={} episodes={}".format(alpha, eps, gamma, episodes))
            score = 0
            for episode_i in range(episodes):
                policy_fun = lambda s: rng.choice((np.argmax(Q[s]), rng.randint(action_bin_size)), p=(1 - eps, eps))  # eps chance of a random pick

                for step_i, (s, a, r, s_, a_, done) in enumerate(run_policy(policy_fun)):
                    if not done:
                        Q[s][a] = Q[s][a] + alpha * (r + gamma * np.max(Q[s_]) - Q[s][a])
                        score += r
                if episode_i % 200 == 0:
                    print("Episode " + str(episode_i) + ": Average score (steps) for last 200 episodes: " + str(score / 200))
                    test_policy(lambda s: np.argmax(Q[s]))

                    dir_path = os.path.dirname(os.path.realpath(__file__))
                    np.save(dir_path + f"/data/Q{episode_i}", Q)
                    score = 0

        train()

    def test_q_policy(self):
        print("Running Tests")
        dir_path = os.path.dirname(os.path.realpath(__file__))
        Q = np.load(dir_path + "/data/Q31200.npy")

        total_score = 0
        for i in range(0, 20):
            total_score += test_policy(lambda s: np.argmax(Q[s]))
        print("Average number of steps lasted: " + str(total_score / 20))
        pass

    def test_visualize_q(self):
        import matplotlib.pyplot as plt

        plt.ion()
        for i in range(0, 25600, 200):
            dir_path = os.path.dirname(os.path.realpath(__file__))
            Q = np.load(dir_path + f"/data/Q{i}.npy")

            vis = np.argmax(np.average(np.average(Q, 2), 2)[:, :, 0, :], 2)
            plt.imshow(vis, cmap="hot", interpolation="nearest")
            plt.draw()
            plt.pause(0.1)
