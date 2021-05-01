import ray.rllib.agents.es as es
from time import sleep
import gym
import ray
from ray import tune
import rospy
from sensor_msgs.msg import JointState, Imu, Image, CameraInfo
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Int32
import numpy as np
from visualization_msgs.msg import Marker

checkpoint_path = "./demos/es-april26/checkpoint-1180"

js_position = np.zeros((16,))
imu_values = np.zeros((6,))
robot_pose = np.zeros((3,))
feet_values = np.zeros((8,))


def feet_callback_1(msg):
    global feet_values
    if msg.points[0].z == 0.0:
        feet_values[0] = -1.0
    else:
        feet_values[0] = 1.0


def feet_callback_2(msg):
    global feet_values
    if msg.points[0].z == 0.0:
        feet_values[1] = -1.0
    else:
        feet_values[1] = 1.0


def feet_callback_3(msg):
    global feet_values
    if msg.points[0].z == 0.0:
        feet_values[3] = -1.0
    else:
        feet_values[3] = 1.0


def feet_callback_4(msg):
    global feet_values
    if msg.points[0].z == 0.0:
        feet_values[2] = -1.0
    else:
        feet_values[2] = 1.0


def feet_callback_5(msg):
    global feet_values
    if msg.points[0].z == 0.0:
        feet_values[4] = -1.0
    else:
        feet_values[4] = 1.0


def feet_callback_6(msg):
    global feet_values
    if msg.points[0].z == 0.0:
        feet_values[5] = -1.0
    else:
        feet_values[5] = 1.0


def feet_callback_7(msg):
    global feet_values
    if msg.points[0].z == 0.0:
        feet_values[7] = -1.0
    else:
        feet_values[7] = 1.0


def feet_callback_8(msg):
    global feet_values
    if msg.points[0].z == 0.0:
        feet_values[6] = -1.0
    else:
        feet_values[6] = 1.0


def js_callback(msg):
    global js_position
    js_position[:] = msg.position[:-2]
    for i in range(len(js_position)):
        if js_position[i] < -3.14:
            js_position[i] = -3.14
        elif js_position[i] > 3.14:
            js_position[i] = 3.14



def imu_callback(msg):
    global imu_values
    imu_values[0] = msg.linear_acceleration.x * (19.62 / 32768)
    imu_values[1] = msg.linear_acceleration.y * (19.62 / 32768)
    imu_values[2] = msg.linear_acceleration.z * (19.62 / 32768)
    imu_values[3] = msg.angular_velocity.x * (500 / 32768)
    imu_values[4] = msg.angular_velocity.y * (500 / 32768)
    imu_values[5] = msg.angular_velocity.z * (500 / 32768)


def pose_callback(msg):
    global robot_pose
    robot_pose[0] = msg.pose.pose.position.x + 0.9736
    robot_pose[1] = msg.pose.pose.position.y + 0.0045
    robot_pose[2] = 0.3


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
    sub_js = rospy.Subscriber("/robot1/joint_states", JointState, js_callback)
    sub_imu = rospy.Subscriber("/robot1/imu_data", Imu, imu_callback)
    sub_pose = rospy.Subscriber("/robot1/amcl_pose", PoseWithCovarianceStamped, pose_callback)
    feet_sub_1 = rospy.Subscriber("/robot1/foot_pressure_0", Marker, feet_callback_1)
    feet_sub_2 = rospy.Subscriber("/robot1/foot_pressure_1", Marker, feet_callback_2)
    feet_sub_3 = rospy.Subscriber("/robot1/foot_pressure_2", Marker, feet_callback_3)
    feet_sub_4 = rospy.Subscriber("/robot1/foot_pressure_3", Marker, feet_callback_4)
    feet_sub_5 = rospy.Subscriber("/robot1/foot_pressure_4", Marker, feet_callback_5)
    feet_sub_6 = rospy.Subscriber("/robot1/foot_pressure_5", Marker, feet_callback_6)
    feet_sub_7 = rospy.Subscriber("/robot1/foot_pressure_6", Marker, feet_callback_7)
    feet_sub_8 = rospy.Subscriber("/robot1/foot_pressure_7", Marker, feet_callback_8)

    rospy.init_node("soccer_control_rl")
    ### run forever
    while not rospy.is_shutdown():

        # run until episode ends
        ### observation vector:
        #   16 joint angles (legs and arms) in range (-pi, pi), refer to Joints Enum in ./gym-soccerbot/gym_soccerbot/envs/walking_forward_env_5.py
        #   3 accelerometer x,y,z in range (-2 * 9.81, 2 * 9.81)
        #   3 gyro x,y,z in range (-500, 500)
        #   3 global position of torso x,y,z - start walking from 0,0 - range (-3, 3)
        #   8 feet bumper sensors, refer to _feet() in ./gym-soccerbot/gym_soccerbot/envs/walking_forward_env_5.py

        observation = np.concatenate((js_position, imu_values, robot_pose, feet_values))  # Expect 18 +
        #print(observation)
        ### normalize & unnormalize the vectors for the model - make use of the model
        observation = env.normalize(observation, env.env.observation_limit_low, env.env.observation_limit_high,
                                    env.observation_plus_range)
        action = agent.compute_action(observation)
        action = env.denormalize(action, env.env.action_space.low, env.env.action_space.high, env.action_plus_range)

        ### action vector:
        #   16 angular velocities, refer to Joints Enum in ./gym-soccerbot/gym_soccerbot/envs/walking_forward_env_5.py
        #   AX12 and MX28 max velocities can be applied

        pub_all_motor = rospy.Publisher("/robot1/all_motor", JointState, queue_size=10)
        motor_names = ["left_arm_motor_0", "left_arm_motor_1", "right_arm_motor_0", "right_arm_motor_1",
                       "left_leg_motor_0", "left_leg_motor_1",
                       "left_leg_motor_2", "left_leg_motor_3", "left_leg_motor_4", "left_leg_motor_5",
                       "right_leg_motor_0", "right_leg_motor_1", "right_leg_motor_2", "right_leg_motor_3",
                       "right_leg_motor_4", "right_leg_motor_5"
                       ]
        js = JointState()
        js.name = []
        js.header.stamp = rospy.Time.now()  # rospy.Time.from_seconds(self.time)
        js.velocity = []
        js.effort = []
        for i, n in enumerate(motor_names):
            js.name.append(n)
            js.velocity.append(action[i])
        pub_all_motor.publish(js)

        sleep(0.041)  # wait for the next observation vector?

    ray.shutdown()
