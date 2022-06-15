# Installation

Make sure you are in the same directory as the README

```shell script
sudo apt-get install python3-tk
python3 -m pip install --user virtualenv
virtualenv -p python3 venv
. venv/bin/activate
pip install -r requirements.txt
```

Then, install the gym-soccerbot package

```shell script
cd gym-soccerbot
pip install -e .
```

# IDE Setup (Optional)

Change your project python intepretor to the virtual environment that you just created.

Mark the folder soccer_rlcontrol/src as a root directory.
In Pycharm IDE change the intepretor to the virtual environment that has been created
`Settings` > `Project` > `Project Intepretor` > Click on the Gear > `Add` > `Virtualenv Environment` > Existing Intepreter and enter the following:

```shell script
/path/to/workspace/soccer_rlcontrol/venv/bin/python
```

Mark the `soccer_rlcontrol/` as a source root directory by right clicking the `soccer_rlcontrol/` > mark as > sources root

# Training

In a terminal with the activated virtualenv,

```shell script
. venv/bin/activate
```

Use one of the yaml config files (preferably one of the non-server ones to avoid the risk of crashing your computer/VM) in the `rllib_configs` directory to start the training:

```shell script
rllib train -f rllib_configs/your_config.yaml
```

## Notes on some Yaml file parameters:

- `timesteps_total` How many simulation steps the training will end after.
- `num_gpus` If you have no compatible nVidia nor Radeon, set this to 0. For Radeon GPUs, look into ROCm-based Tensorflow and PyTorch.
- `num_workers` Roughly corresponds to the number of processes you'd like to simultaneously run.
- `num_cpus_per_worker` Set to 0 so that workers aren't bound to a CPU logical core. Note that there exists a driver process that when running with tune (command above included), takes up a CPU logical core by default.
- `num_envs_per_worker` Each worker process can spawn more than just 1 simulation environment. Ultimately having too many environments per worker will hurt the over performance and/or the computer will run out of RAM (although at a lower rate compared to adding workers).
- `*batch*` If you are using a GPU, your VRAM might limit your batch sizes. If you are running out try reducing the parameters involving the `*batch*` keyword, especially `sgd_minibatch_size`.

# Viewing Results

To visually observer the trained agent, consider one of the checkpoints of your desire in the following directory format:

```shell script
./results/name-of-the-yaml-file/RLalgorithm_UsedGymEnironment_ID_DateOfTraining/checkpoint_n/checkpoint-n
```

Use the following command in the terminal while in the virtualenv created earlier:

```shell script
rllib rollout --run <algo> --env <env> --steps <num_steps> --config '{"num_envs_per_worker": 0, "num_workers": 0, "env_config": {"renders": true}}' <checkpoint_path>
```

Where:

- `<algo>` Is the algorithm the training done with. Look at the `yaml` file used and check for the `run` field value. Replace the argument with the field value from the `yaml` file.
- `<env>` Is the Gym environment agent is trained on. Similar to the procedure for `<algo>`, look for the `env` field value in the `yaml` file.
- `<num_steps>` Defines how many simulation steps the simulation will be running for.
- `<checkpoint_path>` Is the path mentioned. Replace it with the path discussed above.

## View a (somewhat) trained agent out of the box!

### Way 1

Simply run the following with virtualenv activated inside the `soccer_rlcontrol` directory!

```shell script
python esview.py
```

You should see something similar to [this](https://youtu.be/Xkdkml3NZ2Y).
The agent was trained using `humanoid-ppo-server.yaml` on Intel Core i7 5820k with 16GB RAM and GTX 1050Ti with +100M samples (~0.5 day).

### Way 2

Simply run the following with virtualenv activated inside the `soccer_rlcontrol` directory!

```shell script
rllib rollout --run PPO --env gym_soccerbot:walk-forward-norm-v1 --steps 1000 --config '{"num_gpus": 0, "num_envs_per_worker": 0, "num_workers": 0, "env_config": {"renders": true, "env_name": "gym_soccerbot:walk-forward-v3", "slow": true}}' ./demos/ppo-april11/checkpoint_840/checkpoint-840
```

You should see something similar to [this](https://youtu.be/VV2-pMmjHFw).
The agent was trained using `humanoid-ppo-server.yaml` on AMD Threadripper 1950X with 64GB RAM and RTX 2080Ti with +800M samples (~1 day).

## Running in webots

Run the following commands to start webots

```
roslaunch soccerbot soccerbot_multi.launch
```

Wait for webots to load then run to get in proper position

```
rostopic pub robot1/command std_msgs/String 'data: test'
```

Then run the following command to start the walking engine

```
python walking_engine.py
```

# Refrences

## Open AI Gym

The framework used to develop custom environments for our reinforcement learning tasks.
Visit Open AI Gym Github repository [here](https://github.com/openai/gym).

## RLLib

We use RLLib as a scalable set of reinforcement learning.
Visit RLLib Github repository [here](https://github.com/ray-project/ray).

## PyBullet

Free physics simulation of our choice for reinforcement learning tasks.
Visit PyBullet Github repository [here](https://github.com/bulletphysics/bullet3).
