import time
from os.path import expanduser

import numpy as np
import placo
from ischedule import run_loop, schedule
from placo_utils.visualization import point_viz, robot_frame_viz, robot_viz

robot_model = "bez1"
model_filename = expanduser("~") + f"/catkin_ws/src/soccerbot/soccer_description/{robot_model}_description/urdf/robot.urdf"

robot = placo.HumanoidRobot(model_filename)

robot.set_T_world_frame("left_foot", np.eye(4))
robot.update_kinematics()

solver = placo.KinematicsSolver(robot)
T_world_left = placo.flatten_on_floor(robot.get_T_world_left())
T_world_right = placo.flatten_on_floor(robot.get_T_world_right())
# Retrieving initial position of the feet, com and trunk orientation
# T_world_left = robot.get_T_world_frame("left_foot")
# T_world_right = robot.get_T_world_frame("right_foot")
robot.get_T_world_trunk()
# Creating the viewer
viz = robot_viz(robot)
T_world_trunk = robot.get_T_world_trunk()
# Trunk
com_task = solver.add_com_task(T_world_trunk[:3, 3])
com_task.configure("com", "hard", 1.0)

trunk_orientation_task = solver.add_orientation_task("trunk", np.eye(3))
trunk_orientation_task.configure("trunk_orientation", "soft", 1.0)

# Keep left and right foot on the floor
left_foot_task = solver.add_frame_task("left_foot", T_world_left)
left_foot_task.configure("left_foot", "soft", 1.0, 1.0)

right_foot_task = solver.add_frame_task("right_foot", T_world_right)
right_foot_task.configure("right_foot", "soft", 1.0, 1.0)

head_task = solver.add_joints_task()
elbow = 2.512
shoulder_pitch = -0.45
head_task.set_joints(
    {
        "left_shoulder_pitch": shoulder_pitch,
        "left_elbow": elbow,
        # "right_shoulder_roll": -shoulder_roll,
        "right_shoulder_pitch": shoulder_pitch,
        "right_elbow": elbow,
        "head_pitch": 0.0,
        "head_yaw": 0.0,
    }
)
head_task.configure("joints", "soft", 1.0)


# Regularization task
posture_regularization_task = solver.add_joints_task()
posture_regularization_task.set_joints({dof: 0.0 for dof in robot.joint_names()})
posture_regularization_task.configure("reg", "soft", 1e-5)

solver.enable_joint_limits(True)
solver.enable_velocity_limits(True)

t = 0
dt = 0.01
last = 0
solver.dt = dt
start_t = time.time()
robot.update_kinematics()


@schedule(interval=dt)
def loop():
    global t

    # Updating the com target with lateral sinusoidal trajectory
    com_target = T_world_trunk[:3, 3].copy()
    com_target[1] = 0.21
    com_target[2] = 0.21  # T_world_trunk[2,3] - 0.1
    com_task.target_world = com_target

    solver.solve(True)
    robot.update_kinematics()

    viz.display(robot.state.q)
    robot_frame_viz(robot, "trunk")
    point_viz("com", robot.com_world(), radius=0.025, color=0xAAAAAA)

    t += dt


run_loop()
