import time
from unittest import TestCase

import gym
import matplotlib.pyplot
import matplotlib.pyplot as plt
import numpy
import numpy as np
from pyvirtualdisplay import Display
from soccer_rlcontrol.envs import *


class Test(TestCase):
    def setUp(self) -> None:
        super().setUpClass()

    def test_acrobot_basic(self):
        env = gym.make("AcrobotWalker-v0", render_mode="human")
        env.reset()
        for i in range(1, 1000):
            time.sleep(0.01)
            env.step(0)
            env.render()

    def test_acrobot_sarsa(self):
        env = gym.make("AcrobotWalker-v0", render_mode="numerical")
        rng = np.random.RandomState(0)

        bin_size = 30
        bins = [
            np.linspace(env.observation_space.low[0], env.observation_space.high[0], bin_size),
            np.linspace(env.observation_space.low[1], env.observation_space.high[1], bin_size),
            np.linspace(env.observation_space.low[2], env.observation_space.high[2], bin_size),
            np.linspace(env.observation_space.low[3], env.observation_space.high[3], bin_size),
            np.array([env.observation_space.low[4], env.observation_space.low[4]]),
        ]

        Q = np.random.uniform(low=-1, high=1, size=([bin_size] * 4 + [2] + [30]))

        def obs2state(state):
            index = []
            for i in range(len(state)):
                index.append(np.digitize(state[i], bins[i]) - 1)
            return tuple(index)

        def test_policy(policy_fun):
            obs = env.reset()
            s = obs2state(obs)
            a = policy_fun(s)

            display = Display(visible=1, size=(600, 400))
            display.start()
            prev_screen = env.render(mode="rgb_array")
            plt.imshow(prev_screen)

            i = 0
            while True:
                obs_next, r, done, info = env.step(a)
                s_next = obs2state(obs_next)
                a_next = policy_fun(s_next)

                screen = env.render(mode="rgb_array")
                plt.imshow(screen)

                obs, s, a = obs_next, s_next, a_next
                i = i + 1
                if done:
                    print("Pole lasted for " + str(i) + " steps")
                    break
            env.close()
            return i

        def run_policy(policy_fun):
            obs = env.reset()
            s = obs2state(obs)
            a = policy_fun(s)

            while True:
                obs_next, r, done, info = env.step(a)
                s_next = obs2state(obs_next)
                a_next = policy_fun(s_next)
                yield s, a, r, s_next, a_next, done
                obs, s, a = obs_next, s_next, a_next
                if done:
                    break

            env.close()

        def train(alpha=0.15, eps=0.2, gamma=0.995, episodes=10000):
            print("Starting training with alpha={} eps={} gamma={} episodes={}".format(alpha, eps, gamma, episodes))
            score = 0
            for episode_i in range(episodes):
                policy_fun = lambda s: rng.choice((np.argmax(Q[s]), rng.randint(2)), p=(1 - eps, eps))  # eps chance of a random pick

                for step_i, (s, a, r, s_, a_, done) in enumerate(run_policy(policy_fun)):
                    if not done:
                        Q[s][a] = Q[s][a] + alpha * (r + gamma * np.max(Q[s_]) - Q[s][a])
                        score += r
                if episode_i % 200 == 0:
                    print("Episode " + str(episode_i) + ": Average score (steps) for last 200 episodes: " + str(score / 200))
                    score = 0

        train()
        print("Running Tests")
        total_score = 0
        for i in range(0, 20):
            total_score += test_policy(lambda s: np.argmax(Q[s]))
        print("Average number of steps lasted: " + str(total_score / 20))
