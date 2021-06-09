from soccerbot_controller_ros import *
from soccerbot_ros_rl import SoccerbotRosRl
import gym
from ray.rllib.utils.framework import try_import_tf
tf1, tf, tfv = try_import_tf()
import ray
import ray.rllib.agents.ars as ars
import numpy as np

env_id = "walk-forward-norm-v1"
checkpoint_path = "./soccer_rlcontrol/results/humanoid-ars/ARS_gym_soccerbot:walk-forward-norm-v1_f29f6_00000_0_2021-06-05_18-39-54/checkpoint_015000/checkpoint-15000"

class SoccerbotControllerRosRl(SoccerbotControllerRos):

    def __init__(self):

        self.soccerbot = SoccerbotRosRl(Transformation(), useFixedBase=False)
        self.ramp = Ramp("plane.urdf", (0, 0, 0), (0, 0, 0), lateralFriction=0.9, spinningFriction=0.9, rollingFriction=0.0)

        self.position_subscriber = rospy.Subscriber("goal", PoseStamped, self.goal_callback)
        self.robot_position_subscriber = rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.robot_pose_callback)
        self.terminate_walk_subscriber = rospy.Subscriber("terminate_walking", Empty, self.terminate_walk_callback)
        self.completed_walk_publisher = rospy.Publisher("completed_walking", Empty, queue_size=1)
        self.goal = PoseStamped()
        self.new_goal = self.goal
        self.terminate_walk = False

        # Run the shahryar env here
        # TODO change goal later?
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
        self.agent = trainer(env="gym_soccerbot:walk-forward-norm-v1", config=config)
        self.agent.load_checkpoint(checkpoint_path)
        env = gym.make(env_id, renders=True, env_name="gym_soccerbot:walk-omni-v0", goal=[10, 0])

    def run(self):
        r = rospy.Rate(120)

        while not rospy.is_shutdown():
            if self.new_goal != self.goal:
                print("Recieved New Goal")
                self.soccerbot.setPose(self.pose_to_transformation(self.robot_pose.pose.pose))
                self.wait(200)

                self.goal = self.new_goal
                self.soccerbot.ready() # TODO Cancel walking
                self.soccerbot.reset_head()
                self.soccerbot.publishAngles()
                print("Getting ready")
                self.wait(150)


                # Reset robot position and goal
                self.soccerbot.setGoal(self.pose_to_transformation(self.goal.pose))
                self.soccerbot.publishPath()
                t = 0

            if self.terminate_walk:
                # TODO terminate walk
                self.terminate_walk = False

            observation_vector = self.soccerbot.getObservationVector()

            ## RL code here
            action_vector = self.agent.compute_action(observation_vector)

            self.soccerbot.velocity_configuration(np.concatenate((action_vector, [0, 0]))) #16+2

            # TODO walk is completed
            if True:
                print("Completed Walk")
                e = Empty()
                self.completed_walk_publisher.publish(e)

            # TODO when walk is not running apply head rotation
            if True:
                self.soccerbot.apply_head_rotation()

            self.soccerbot.publishAngles()
            r.sleep()
