from ray.rllib.utils.framework import try_import_tf
tf1, tf, tfv = try_import_tf()
from time import sleep
import gym
import ray
from ray import tune
import ray.rllib.agents.ars as ars
import cProfile, pstats, io
from pstats import SortKey

checkpoint_path = "/home/robosoccer/shah_ws/robust/soccerbot/soccer_rlcontrol/results/humanoid-ars/ARS_gym_soccerbot:walk-forward-norm-v1_f29f6_00000_0_2021-06-05_18-39-54/checkpoint_015000/checkpoint-15000"

if __name__ == '__main__':
    physical_devices = tf.config.list_physical_devices('GPU')
    tf.config.experimental.set_memory_growth(physical_devices[0], True)
    ray.init(local_mode=True)
    trainer, trainer_class = ars.ARSTrainer, ars
    # load
    config = trainer_class.DEFAULT_CONFIG.copy()
    config["framework"] = "tf"
    config["eager_tracing"] = False
    config["env_config"] = {"env_name": "gym_soccerbot:walk-omni-v0"}
    config["num_workers"] = 1
    config["model"] = {"fcnet_hiddens": [128, 128]}
    config["num_gpus"] = 0
    agent = trainer(env="gym_soccerbot:walk-forward-norm-v1", config=config)
    agent.load_checkpoint(checkpoint_path)
    #agent.restore(checkpoint_path)
    #agent.get_policy().get_weights()
    #agent.get_policy().export_model("/home/shahryar/PycharmProjects/DeepRL/weights")


    # instantiate env class
    env_id = "walk-forward-norm-v1"
    env = gym.make(env_id, renders=True, env_name="gym_soccerbot:walk-omni-v0", goal=[2, 0])

    #pr = cProfile.Profile()
    #pr.enable()
    # ... do something ...
    # run until episode ends
    # for j in range(100):
    while True:
        episode_reward = 0
        done = False
        obs = env.reset()
        i = 0
        while not done:
            action = agent.compute_action(obs)
            obs, reward, done, info = env.step(action)
            # print(f'Z: {env.env._global_pos()[2]:.3f}')
            sleep(0.0041)
            sleep(0.0041)
            episode_reward += reward
            i += 1
            # print(f'step: {i}')
        # print(f'episode_reward: {episode_reward:.3f}, episode_len: {i}, info: {info}')
    pr.disable()
    s = io.StringIO()
    sortby = SortKey.CUMULATIVE
    ps = pstats.Stats(pr, stream=s).sort_stats(sortby)
    ps.print_stats()
    print(s.getvalue())
    env.close()
    ray.shutdown()
