from gym.envs.registration import register

register(
    id='walk-omni-v0',
    entry_point='gym_soccerbot.envs:WalkingOmni',
)

register(
    id='walk-forward-norm-v1',
    entry_point='gym_soccerbot.envs:WalkingForwardNormAgn',
)

import os
from os.path import dirname as up

def getModelPath():
  respath = os.path.join(up(up(up(os.path.dirname(__file__)))), "soccer_description/models/soccerbot_stl.urdf")
  return respath
