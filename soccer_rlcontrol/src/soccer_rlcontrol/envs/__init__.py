from gym.envs.registration import load_env_plugins as _load_env_plugins
from gym.envs.registration import make, register, registry, spec
from soccer_rlcontrol.envs.acrobot_walker import AcrobotWalkerEnv

# Hook to load plugins from entry points
_load_env_plugins()

register(
    id="AcrobotWalker-v0",
    entry_point="soccer_rlcontrol.envs:AcrobotWalkerEnv",
    max_episode_steps=500,
    reward_threshold=195.0,
)
