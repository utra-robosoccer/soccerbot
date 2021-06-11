from soccerbot_controller_ros import *
from soccerbot_ros_rl import SoccerbotRosRl

import gym
from ray.rllib.utils.framework import try_import_tf

tf1, tf, tfv = try_import_tf()
import ray
import ray.rllib.agents.ars as ars
import numpy as np
import pybullet as pb
env_id = "walk-forward-norm-v1"
checkpoint_path = "/home/shahryar/hdd/catkin_ws/src/soccerbot/soccer_pycontrol/src/soccer_rlcontrol/results/humanoid-ars" \
                  "/ARS_gym_soccerbot:walk-forward-norm-v1_f29f6_00000_0_2021-06-05_18-39-54/checkpoint_015000" \
                  "/checkpoint-15000"
#"./soccer_rlcontrol/results/humanoid-ars/ARS_gym_soccerbot:walk-forward-norm-v1_f29f6_00000_0_2021-06-05_18-39-54/checkpoint_015000/checkpoint-15000"


class SoccerbotControllerRosRl(SoccerbotControllerRos):

    def __init__(self):

        self.soccerbot = SoccerbotRosRl(Transformation(), usePybullet=False)
        self.position_subscriber = rospy.Subscriber("goal", PoseStamped, self.goal_callback)
        self.robot_position_subscriber = rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped,
                                                          self.robot_pose_callback)
        self.terminate_walk_subscriber = rospy.Subscriber("terminate_walking", Empty, self.terminate_walk_callback)
        self.completed_walk_publisher = rospy.Publisher("completed_walking", Empty, queue_size=1)
        self.goal = PoseStamped()
        self.new_goal = self.goal
        self.terminate_walk = False

        # Run the shahryar env here
        # TODO change goal later?
        # physical_devices = tf.config.list_physical_devices('GPU')
        # tf.config.experimental.set_memory_growth(physical_devices[0], True)
        print("RAY INIT START")
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
        self.agent = trainer(env="gym_soccerbot:walk-forward-norm-v1", config=config)
        self.agent.load_checkpoint(checkpoint_path)
        self.env = gym.make(env_id, renders=True, env_name="gym_soccerbot:walk-omni-v0", goal=[10, 0])
        print("RAY INIT END")

    def robot_pose_callback(self, pose):
        self.robot_pose = pose
        pass

    def terminate_walk_callback(self, val):
        self.terminate_walk = True

    def pose_to_transformation(self, pose):
        t = Transformation([pose.position.x, pose.position.y, pose.position.z],
                           [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        return t

    def wait(self, steps):
        rospy.sleep(SoccerbotController.PYBULLET_STEP * steps)

    def run(self):
        r = rospy.Rate(120)

        completed = True
        while not rospy.is_shutdown():
            self.soccerbot.setPose(self.pose_to_transformation(self.robot_pose.pose.pose))
            if self.new_goal != self.goal:
                print("Recieved New Goal")
                self.wait(200)

                self.goal = self.new_goal
                self.soccerbot.ready()  # TODO Cancel walking
                self.soccerbot.reset_head()
                self.soccerbot.publishAngles()
                print("Getting ready")
                self.wait(150)
                self.soccerbot.setGoal(self.pose_to_transformation(self.goal.pose))
                completed = False

            if not completed:
                observation_vector = self.soccerbot.getObservationVector()
                joint_angles = observation_vector[:16]
                observation_vector = self.env.normalize(observation_vector, self.env.env.observation_limit_low,
                                                        self.env.env.observation_limit_high,
                                                        self.env.observation_plus_range)
                # clip
                observation_vector = np.clip(observation_vector, -10.0, 10.0)
                action_vector = self.agent.compute_action(observation_vector)
                action_vector = self.env.denormalize(action_vector, self.env.env.action_space.low,
                                                     self.env.env.action_space.high,
                                                     self.env.action_plus_range)
                action_vector = self.soccerbot.motor_control(action_vector, joint_angles, self.env.env)
                self.soccerbot.velocity_configuration = list(np.concatenate((action_vector, np.array([0, 0]))))  # 16+2

            if self.terminate_walk:
                self.completed = True
                self.terminate_walk = False

            # Here do a check if the walking is completed
            # # TODO walk is completed
            # if False:
            #     print("Completed Walk")
            #     e = Empty()
            #     self.completed_walk_publisher.publish(e)
            #
            # # TODO when walk is not running apply head rotation
            # if True:
            #     self.soccerbot.apply_head_rotation()

            self.soccerbot.publishAngularVelocities()
            r.sleep()
