import ray.rllib.agents.es as es
from time import sleep
import gym
import ray
from ray import tune


checkpoint_path = "./demos/es-april26/checkpoint-1180"

if __name__ == '__main__':
    ray.init(local_mode=False)
    trainer, trainer_class = es.ESTrainer, es
    # load
    config = trainer_class.DEFAULT_CONFIG.copy()
    config["framework"] = "torch"
    config["env_config"] = {"env_name": "gym_soccerbot:walk-forward-velocity-v1"}
    config["num_workers"] = 1
    agent = trainer(env="gym_soccerbot:walk-forward-norm-v1", config=config)
    agent.load_checkpoint(checkpoint_path)


    # instantiate env class
    env_id = "walk-forward-norm-v1"
    env = gym.make(env_id, renders=True, env_name="gym_soccerbot:walk-forward-velocity-v1", goal=(2, 0))

    # run until episode ends
    while True:
        episode_reward = 0
        done = False
        obs = env.reset()
        i = 0
        while not done:
            action = agent.compute_action(obs)
            obs, reward, done, info = env.step(action)
            episode_reward += reward
            i += 1
        print(f'episode_reward: {episode_reward:.3f}, episode_len: {i}, info: {info}')

    ray.shutdown()
