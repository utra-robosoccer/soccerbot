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
# Creating the viewer
T_world_trunk = robot.get_T_world_frame("trunk")
viz = robot_viz(robot)
# Trunk
# com_task = solver.add_com_task(T_world_trunk[:3, 3])
# com_task.configure("com", "hard", 1.0)


# T_world_trunk[2, 3] = 0.35
trunk_task = solver.add_frame_task("trunk", T_world_trunk)
trunk_task.configure("trunk_task", "hard", 1e3, 1e3)

# Keep left and right foot on the floor
left_foot_task = solver.add_frame_task("left_foot", T_world_left)
left_foot_task.configure("left_foot", "soft", 1.0, 1.0)

right_foot_task = solver.add_frame_task("right_foot", T_world_right)
right_foot_task.configure("right_foot", "soft", 1.0, 1.0)

# T_world_right_forearm = robot.get_T_world_frame("right_forearm")
# right_hand_task = solver.add_frame_task("right_forearm", T_world_right_forearm)
# right_hand_task.configure("right_forearm", "soft", 1.0, 1.0)

# Look at ball
look_at_ball = solver.add_axisalign_task("camera", np.array([1.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0]))
look_at_ball.configure("look_ball", "soft", 1.0)

# right_forearm
# head_task = solver.add_joints_task()
# elbow = 2.512
# shoulder_pitch = -0.45
# head_task.set_joints(
#     {
#         # "left_shoulder_pitch": shoulder_pitch,
#         # "left_elbow": elbow,
#         # "right_shoulder_roll": -shoulder_roll,
#         # "right_shoulder_pitch": shoulder_pitch,
#         # "right_elbow": elbow,
#         "head_pitch": 0.0,
#         "head_yaw": 0.0,
#     }
# )
# head_task.configure("joints", "soft", 1.0)
# x, y, z = T_world_right_forearm[:3, 3].copy()
# right_hand_y_traj = placo.CubicSpline()
# right_hand_y_traj.add_point(0.0, y, 0.0)
# right_hand_y_traj.add_point(0.5, y - 0.5, 0.0)
# right_hand_y_traj.add_point(1.0, y, 0.0)
# right_hand_y_traj.add_point(2.0, y, 0.0)
#
# right_hand_x_traj = placo.CubicSpline()
# right_hand_x_traj.add_point(0.0, x + 5, 0.0)
# right_hand_x_traj.add_point(0.5, x + 5, 0.0)
# right_hand_x_traj.add_point(1.0, x + 5, 0.0)
# right_hand_x_traj.add_point(2.0, x + 5, 0.0)
#
# right_hand_z_traj = placo.CubicSpline()
# right_hand_z_traj.add_point(0.0, z + 0.3, 0.0)
# right_hand_z_traj.add_point(0.5, z + 0.3, 0.0)
# right_hand_z_traj.add_point(1.0, z + 0.3, 0.0)
# right_hand_z_traj.add_point(2.0, z + 0.1, 0.0)

# Regularization task
# posture_regularization_task = solver.add_joints_task()
# posture_regularization_task.set_joints({dof: 0.0 for dof in robot.joint_names()})
# posture_regularization_task.configure("reg", "soft", 1e-5)

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
    # com_target = T_world_trunk[:3, 3].copy()
    # com_target[1] = 0.21
    # com_target[2] = 0.21  # T_world_trunk[2,3] - 0.1
    # com_task.target_world = com_target
    # Updating the target
    # t_mod = t % 2.0
    # target = right_hand_task.position().target_world
    # # target[0] = right_hand_x_traj.pos(t_mod)
    # # target[1] = right_hand_y_traj.pos(t_mod)
    # # target[2] = right_hand_z_traj.pos(t_mod)
    # right_hand_task.position().target_world = target

    ball = np.array([0.5 + np.cos(t) * 0.25, np.sin(t) * 0.7, 0.0])
    camera_pos = robot.get_T_world_frame("camera")[:3, 3]
    look_at_ball.targetAxis_world = ball - camera_pos

    solver.solve(True)
    robot.update_kinematics()

    viz.display(robot.state.q)
    robot_frame_viz(robot, "trunk")
    point_viz("com", robot.com_world(), radius=0.025, color=0xAAAAAA)

    t += dt


run_loop()
