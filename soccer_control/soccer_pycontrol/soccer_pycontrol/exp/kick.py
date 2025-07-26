import time
from os.path import expanduser

import numpy as np
import placo
from ischedule import run_loop, schedule
from placo_utils.visualization import point_viz, robot_frame_viz, robot_viz

"""
Sigmaban humanoid is moving its legs while looking at a moving ball.
"""

# Loading the robot
robot_model = "assembly"
model_filename = expanduser("~") + f"/ros2_ws/src/soccerbot/soccer_description/{robot_model}_description/urdf/robot.urdf"
robot = placo.HumanoidRobot(model_filename)

# Placing the left foot in world origin
robot.set_T_world_frame("left_foot", np.eye(4))
robot.update_kinematics()

solver = placo.KinematicsSolver(robot)

# Retrieving initial position of the feet, com and trunk orientation
T_world_left = robot.get_T_world_frame("left_foot")
T_world_right = robot.get_T_world_frame("right_foot")

# Creating the viewer
viz = robot_viz(robot)

# Trunk
com_task = solver.add_com_task(np.array([0.0, 0.0, 0.23]))
com_task.configure("com", "soft", 1.0)

trunk_orientation_task = solver.add_orientation_task("trunk", np.eye(3))
trunk_orientation_task.configure("trunk_orientation", "soft", 1.0)

# Keep left and right foot on the floor
left_foot_task = solver.add_frame_task("left_foot", T_world_left)
left_foot_task.configure("left_foot", "soft", 1.0, 1.0)

right_foot_task = solver.add_frame_task("right_foot", T_world_right)
right_foot_task.configure("right_foot", "soft", 1.0, 1.0)

# Look at ball
look_at_ball = solver.add_axisalign_task("camera", np.array([1.0, 0.0, 0.0]), np.array([0.0, 0.0, 1.0]))
look_at_ball.configure("look_ball", "soft", 1.0)

# Creating a very basic lateral swing and foot rise trajectory
left_foot_z_traj = placo.CubicSpline()
left_foot_z_traj.add_point(0.0, 0.0, 0.0)
left_foot_z_traj.add_point(0.5, 0.05, 0.0)
left_foot_z_traj.add_point(1.0, 0.05, 0.0)
left_foot_z_traj.add_point(1.5, 0.0, 0.0)

left_foot_x_traj = placo.CubicSpline()
left_foot_x_traj.add_point(0.0, 0.0, 0.0)
left_foot_x_traj.add_point(0.5, -0.05, 0.0)
left_foot_x_traj.add_point(1.0, 0.15, 0.0)
left_foot_x_traj.add_point(1.5, 0.0, 0.0)

left_foot_y_traj = placo.CubicSpline()
left_foot_y_traj.add_point(0.0, 0.0, 0.0)
left_foot_y_traj.add_point(0.5, 0.0, 0.0)
left_foot_y_traj.add_point(1.0, 0.0, 0.0)
left_foot_y_traj.add_point(1.5, 0.0, 0.0)

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

    # Updating the target
    t_mod = t % 2.0
    target = left_foot_task.position().target_world
    target[0] = left_foot_x_traj.pos(t_mod)
    target[1] = left_foot_y_traj.pos(t_mod)
    target[2] = left_foot_z_traj.pos(t_mod)
    left_foot_task.position().target_world = target

    # Updating the com target with lateral sinusoidal trajectory
    com_task.target_world = np.array([0.0, -0.07, 0.23])

    # Looking at ball
    ball = np.array([0.2, 0.0, 0.0])
    camera_pos = robot.get_T_world_frame("camera")[:3, 3]
    look_at_ball.targetAxis_world = ball - camera_pos

    solver.solve(True)
    robot.update_kinematics()

    viz.display(robot.state.q)
    robot_frame_viz(robot, "trunk")
    robot_frame_viz(robot, "camera")
    point_viz("com", robot.com_world(), radius=0.025, color=0xAAAAAA)
    point_viz("ball", ball, radius=0.05, color=0xDDDDDD)

    t += dt


run_loop()
