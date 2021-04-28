import ray.rllib.agents.es as es
from time import sleep
import gym
import ray
from ray import tune


checkpoint_path = "./demos/es-april26/checkpoint-1180"


if __name__ == '__main__':
    ### Setup
    ray.init(local_mode=False)
    trainer, trainer_class = es.ESTrainer, es
    config = trainer_class.DEFAULT_CONFIG.copy()
    config["framework"] = "torch"
    config["env_config"] = {"env_name": "gym_soccerbot:walk-forward-velocity-v1"}
    config["num_workers"] = 1
    agent = trainer(env="gym_soccerbot:walk-forward-norm-v1", config=config)
    agent.load_checkpoint(checkpoint_path)
    env_id = "walk-forward-norm-v1"
    env = gym.make(env_id, renders=True, env_name="gym_soccerbot:walk-forward-velocity-v1", goal=(2, 0))


    ### run forever
    while True:
        # run until episode ends
        ### observation vector:
        #   16 joint angles (legs and arms) in range (-pi, pi), refer to Joints Enum in ./gym-soccerbot/gym_soccerbot/envs/walking_forward_env_5.py
        #   3 accelerometer x,y,z in range (-2 * 9.81, 2 * 9.81)
        #   3 gyro x,y,z in range (-500, 500)
        #   3 gloabal position of torso x,y,z - start walking from 0,0 - range (-3, 3)
        #   8 feet bumper sensors, refer to _feet() in ./gym-soccerbot/gym_soccerbot/envs/walking_forward_env_5.py
        observation = [] # Expect 18 + 
        
        ### normalize & unnormalize the vectors for the model - make use of the model
        observation = env.normalize(observation, env.env.observation_limit_low, env.env.observation_limit_high, env.observation_plus_range)
        action = agent.compute_action(obs)
        action = env.denormalize(action, env.env.action_space.low, env.env.action_space.high, env.action_plus_range)
        
        
        
        ### action vector:
        #   16 angular velocities, refer to Joints Enum in ./gym-soccerbot/gym_soccerbot/envs/walking_forward_env_5.py
        #   AX12 and MX28 max velocities can be applied
        send(action)
        sleep(0.041) # wait for the next observation vector?

    ray.shutdown()
