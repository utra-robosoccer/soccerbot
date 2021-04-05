from gym.envs.registration import register
from os.path import dirname as up

register(
    id='walk-forward-v0',
    entry_point='gym_soccerbot.envs:WalkingForward',
)

register(
    id='walk-forward-norm-v0',
    entry_point='gym_soccerbot.envs:WalkingForwardNorm',
)

register(
    id='walk-forward-v1',
    entry_point='gym_soccerbot.envs:WalkingForwardV1',
)

import os

def getModelPath():
  respath = os.path.join(up(up(os.path.dirname(__file__))), "soccer_description/models/soccerbot_stl.urdf")
  return respath
