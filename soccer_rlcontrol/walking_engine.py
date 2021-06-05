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

checkpoint_path = "./demos/checkpoint-14450"
import enum



class Joints(enum.IntEnum):
    LEFT_ARM_1 = 0
    LEFT_ARM_2 = 1
    RIGHT_ARM_1 = 2
    RIGHT_ARM_2 = 3
    LEFT_LEG_1 = 4
    LEFT_LEG_2 = 5
    LEFT_LEG_3 = 6
    LEFT_LEG_4 = 7
    LEFT_LEG_5 = 8
    LEFT_LEG_6 = 9
    RIGHT_LEG_1 = 10
    RIGHT_LEG_2 = 11
    RIGHT_LEG_3 = 12
    RIGHT_LEG_4 = 13
    RIGHT_LEG_5 = 14
    RIGHT_LEG_6 = 15
    HEAD_1 = 16
    HEAD_2 = 17
    HEAD_CAMERA = 18
    IMU = 19


_MX_28_velocity = 2 * np.pi
#### Joint Limits HARD CODE
_joint_limit_high = np.zeros(16)

_joint_limit_high[Joints.RIGHT_LEG_1] = 0.2
_joint_limit_high[Joints.RIGHT_LEG_2] = 0.2
_joint_limit_high[Joints.RIGHT_LEG_3] = 0.67
_joint_limit_high[Joints.RIGHT_LEG_4] = 0.05
_joint_limit_high[Joints.RIGHT_LEG_5] = 0.5
_joint_limit_high[Joints.RIGHT_LEG_6] = 0.15
_joint_limit_high[Joints.LEFT_LEG_1] = 0.2
_joint_limit_high[Joints.LEFT_LEG_2] = 0.2
_joint_limit_high[Joints.LEFT_LEG_3] = 0.67
_joint_limit_high[Joints.LEFT_LEG_4] = 0.05
_joint_limit_high[Joints.LEFT_LEG_5] = 0.5
_joint_limit_high[Joints.LEFT_LEG_6] = 0.15
_joint_limit_high[Joints.RIGHT_ARM_1] = 0.95
_joint_limit_high[Joints.RIGHT_ARM_2] = 0.8
_joint_limit_high[Joints.LEFT_ARM_1] = 0.95
_joint_limit_high[Joints.LEFT_ARM_2] = 0.8

_joint_limit_high *= (np.pi)

_joint_limit_low = np.zeros(16)

_joint_limit_low[Joints.RIGHT_LEG_1] = 0.3
_joint_limit_low[Joints.RIGHT_LEG_2] = 0.1
_joint_limit_low[Joints.RIGHT_LEG_3] = 0.2
_joint_limit_low[Joints.RIGHT_LEG_4] = 0.45
_joint_limit_low[Joints.RIGHT_LEG_5] = 0.12
_joint_limit_low[Joints.RIGHT_LEG_6] = 0.1
_joint_limit_low[Joints.LEFT_LEG_1] = 0.3
_joint_limit_low[Joints.LEFT_LEG_2] = 0.1
_joint_limit_low[Joints.LEFT_LEG_3] = 0.2
_joint_limit_low[Joints.LEFT_LEG_4] = 0.45
_joint_limit_low[Joints.LEFT_LEG_5] = 0.12
_joint_limit_low[Joints.LEFT_LEG_6] = 0.1
_joint_limit_low[Joints.RIGHT_ARM_1] = 0.4
_joint_limit_low[Joints.RIGHT_ARM_2] = 0.0
_joint_limit_low[Joints.LEFT_ARM_1] = 0.4
_joint_limit_low[Joints.LEFT_ARM_2] = 0.0

_joint_limit_low *= (-np.pi)


js_position = np.zeros((16,))
js_position_old = np.zeros((16,))
js_velocity = np.zeros((16,))
imu_values = np.zeros((6,))
robot_orn = np.zeros((4,))
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
    global js_position_old
    global js_velocity
    js_position[:] = msg.position[:-2]
    for i in range(len(js_position)):
        if js_position[i] < -3.14:
            js_position[i] = -3.14
        elif js_position[i] > 3.14:
            js_position[i] = 3.14
    js_velocity = (js_position - js_position_old) / 0.0082
    js_position_old = js_position


def imu_callback(msg):
    global imu_values
    imu_values[0] = msg.linear_acceleration.x * (19.62 / 32768)
    imu_values[1] = msg.linear_acceleration.y * (19.62 / 32768)
    imu_values[2] = msg.linear_acceleration.z * (19.62 / 32768)
    imu_values[3] = msg.angular_velocity.x * (8.7266 / 32768)
    imu_values[4] = msg.angular_velocity.y * (8.7266 / 32768)
    imu_values[5] = msg.angular_velocity.z * (8.7266 / 32768)


def pose_callback(msg):
    global robot_orn
    robot_orn[0] = msg.pose.pose.orientation.x
    robot_orn[1] = msg.pose.pose.orientation.y
    robot_orn[2] = msg.pose.pose.orientation.z
    robot_orn[3] = msg.pose.pose.orientation.w


if __name__ == '__main__':
    ### Setup
    ray.init(local_mode=False)
    trainer, trainer_class = es.ESTrainer, es
    config = trainer_class.DEFAULT_CONFIG.copy()
    config["framework"] = "tf"
    config["env_config"] = {"env_name": "gym_soccerbot:walk-omni-v0"}
    config["num_workers"] = 1
    agent = trainer(env="gym_soccerbot:walk-forward-norm-v1", config=config)
    agent.load_checkpoint(checkpoint_path)
    env_id = "walk-forward-norm-v1"
    env = gym.make(env_id, renders=True, env_name="gym_soccerbot:walk-omni-v0", goal=(2, 0))
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
        #   16 joint angles (legs and arms) in range (-pi, pi), refer to Joints Enum in ./gym-soccerbot/gym_soccerbot/envs/walking_forward_env_6.py
        #   16 joint velocities (legs and arms) in range (-MAX_VELOCITY * 2, MAX_VELOCITY * 2), refer to Joints Enum in ./gym-soccerbot/gym_soccerbot/envs/walking_forward_env_6.py
        #   3 accelerometer x,y,z in range (-2 * 9.81, 2 * 9.81)
        #   3 gyro x,y,z in range (-500, 500)
        #   4 global orientation quaternion - x y z w - torso link
        #   1 height of the torso - range (0.1, 0.4)
        #   8 feet bumper sensors, refer to _feet() in ./gym-soccerbot/gym_soccerbot/envs/walking_forward_env_6.py
        height = np.zeros((1,))
        height[0] = 0.28
        observation = np.concatenate((js_position, js_velocity, imu_values, robot_orn, height, feet_values))  # Expect 18 +
        #print(observation)
        ### normalize & unnormalize the vectors for the model - make use of the model
        observation = env.normalize(observation, env.env.observation_limit_low, env.env.observation_limit_high,
                                    env.observation_plus_range)
        action = agent.compute_action(observation)
        action = env.denormalize(action, env.env.action_space.low, env.env.action_space.high, env.action_plus_range)

        ### action vector:
        #   16 angular velocities, refer to Joints Enum in ./gym-soccerbot/gym_soccerbot/envs/walking_forward_env_5.py
        #   AX12 and MX28 max velocities can be applied

        for i in range(16):
            joint_cur_pos = js_position[i]
            velocity = action[i]
            velocity = velocity if joint_cur_pos < _joint_limit_high[i] else -_MX_28_velocity
            velocity = velocity if joint_cur_pos > _joint_limit_low[i] else _MX_28_velocity
            action[i] = velocity

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
