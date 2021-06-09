import pytest
import gym
import numpy as np
from time import sleep
import json
import socket

UDP_IP = "127.0.0.1"
UDP_PORT = 9870

# env_name = "gym_soccerbot:walk-forward-random-v1"
env_name = 'gym_soccerbot:walk-forward-velocity-v1'
env_id = 'gym_soccerbot:walk-forward-norm-v1'
# env_id = "gym_soccerbot:walk-forward-v2"
'''
def test_make_env():
    gym.make(env_id, renders=False, env_name=env_name)
    assert True

def test_reset():
    print("----------------------------------- test reset")
    env = gym.make(env_id, renders=False, env_name=env_name)
    obs = env.reset()
    print(f'obs: {obs}')
    env.close()
    assert True

def test_step():
    print("----------------------------------- test step")
    env = gym.make(env_id, renders=False, env_name=env_name)
    obs = env.reset()
    act = np.random.uniform(-np.pi, np.pi, env.env._JOINT_DIM)
    observation, reward, done, info = env.step(act)
    print(f'obs: {obs}')
    print(f'reward: {reward}')
    print(f'done: {done}')
    print(f'info: {info}')
    env.close()
    assert True

def test_episode():
    print("----------------------------------- test episode")
    env = gym.make(env_id, renders=False, env_name=env_name)
    env.reset()
    done = False
    i = 0
    while not done:
        act = np.random.uniform(-np.pi, np.pi, env.action_space.shape)
        observation, reward, done, info = env.step(act)
        i += 1
        if i == 4096:
            break
    print(f'observation: {observation}')
    print(f'reward: {reward}')
    print(f'done: {done}')
    print(f'info: {info}')
    env.close()
    assert True

def test_back_to_back_episode():
    print("----------------------------------- test_back_to_back_episode")
    env = gym.make(env_id, renders=False, env_name=env_name)
    env.reset()
    done = False
    i = 0
    while not done:
        act = np.random.uniform(-np.pi, np.pi, env.action_space.shape)
        observation, reward, done, info = env.step(act)
        i += 1
        if i == 4096:
            break
    print(f'observation: {observation}')
    print(f'reward: {reward}')
    print(f'done: {done}')
    print(f'info: {info}')
    print(f'steps: {i}')
    env.reset()
    done = False
    i = 0
    while not done:
        act = np.random.uniform(-np.pi, np.pi, env.action_space.shape)
        observation, reward, done, info = env.step(act)
        i += 1
        if i == 4096:
            break
    print(f'observation: {observation}')
    print(f'reward: {reward}')
    print(f'done: {done}')
    print(f'info: {info}')
    print(f'steps: {i}')
    env.close()
    assert True
'''
def test_episode_render():
    print("----------------------------------- test_episode_render")
    env = gym.make(env_id, renders=True, env_name=env_name, seed=1)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    obs = env.reset()
    i = 0
    message = dict(timestamp=i * (1. / 120.), data=list(obs[:12]))
    message["data"] = [float(x) for x in message["data"]]
    raw = json.dumps(message)
    sock.sendto(raw.encode('utf-8'), (UDP_IP, UDP_PORT))
    i += 1
    done = False
    cum_reward = 0
    MAX = 4096
    act = np.zeros(env.action_space.shape)
    while not done:
        # act *= -1.
        # act = np.random.uniform(-np.pi, np.pi, env.action_space.shape)
        observation, reward, done, info = env.step(act)
        cum_reward += reward
        print(f'------------reward: {reward:.5f}')
        message = dict(timestamp=i * (1. / 120.), data=list(observation[:12]))
        message["data"] = [float(x) for x in message["data"]]
        raw = json.dumps(message)
        sock.sendto(raw.encode('utf-8'), (UDP_IP, UDP_PORT))
        sleep(0.041)
        i += 1
        # if i == MAX:
            # break
    # print(f'observation: {observation}')
    # print(f'reward: {reward}')
    print(f'done: {done}')
    print(f'info: {info}')
    print(f'cumulative reward: {cum_reward}')
    env.close()
    assert True