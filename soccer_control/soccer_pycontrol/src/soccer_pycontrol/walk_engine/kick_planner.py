import time
from os.path import expanduser
from typing import List

import numpy as np
import placo
import pybullet as p
from placo_utils.visualization import (
    footsteps_viz,
    frame_viz,
    line_viz,
    point_viz,
    robot_frame_viz,
    robot_viz,
)


class KickPlanner:
    def __init__(self, robot_model: str, parameters: dict, funct_time, debug: bool = True):
        self.funct_time = funct_time
        self.DT = parameters["control_frequency"]

        self.debug = debug
        self.robot_model = robot_model
        model_filename = expanduser("~") + f"/catkin_ws/src/soccerbot/soccer_description/{robot_model}_description/urdf/robot.urdf"

        self.start_t = self.funct_time()
        # TODO is there too many global var
        self.robot = placo.HumanoidRobot(model_filename)
        self.t = 0

        self.solver = None
        if self.debug:
            # Starting Meshcat viewer
            self.viz = robot_viz(self.robot)
            self.last_display = self.funct_time()

    # TODO can this be in init
    def setup_tasks(self):
        # Creating the kinematics solver
        self.robot.set_T_world_frame("left_foot", np.eye(4))
        self.robot.update_kinematics()

        self.solver = placo.KinematicsSolver(self.robot)
        self.solver.enable_velocity_limits(True)
        self.solver.enable_joint_limits(True)
        self.solver.dt = self.DT

        T_world_left = self.robot.get_T_world_frame("left_foot")
        T_world_right = self.robot.get_T_world_frame("right_foot")

        self.com_task = self.solver.add_com_task(np.array([0.0, 0.0, 0.23]))
        self.com_task.configure("com", "hard", 1.0)

        trunk_orientation_task = self.solver.add_orientation_task("trunk", np.eye(3))
        trunk_orientation_task.configure("trunk_orientation", "soft", 1.0)

        # Keep left and right foot on the floor
        self.left_foot_task = self.solver.add_frame_task("left_foot", T_world_left)
        self.left_foot_task.configure("left_foot", "soft", 1.0, 1.0)

        self.right_foot_task = self.solver.add_frame_task("right_foot", T_world_right)
        self.right_foot_task.configure("right_foot", "soft", 1.0, 1.0)

        # Creating a very basic lateral swing and foot rise trajectory
        ti = [0.1, 0.2, 0.3, 0.4]
        self.left_foot_z_traj = placo.CubicSpline()
        self.left_foot_z_traj.add_point(ti[0], 0.0, 0.0)
        self.left_foot_z_traj.add_point(ti[1], 0.05, 0.0)
        self.left_foot_z_traj.add_point(ti[2], 0.075, 0.0)
        self.left_foot_z_traj.add_point(ti[3], 0.0, 0.0)

        self.left_foot_x_traj = placo.CubicSpline()
        self.left_foot_x_traj.add_point(ti[0], 0.0, 0.0)
        self.left_foot_x_traj.add_point(ti[1], -0.05, 0.0)
        self.left_foot_x_traj.add_point(ti[2], 0.5, 0.0)
        self.left_foot_x_traj.add_point(ti[3], 0.0, 0.0)

        self.left_foot_y_traj = placo.CubicSpline()
        self.left_foot_y_traj.add_point(ti[0], 0.0, 0.0)
        self.left_foot_y_traj.add_point(ti[1], 0.0, 0.0)
        self.left_foot_y_traj.add_point(ti[2], 0.0, 0.0)
        self.left_foot_y_traj.add_point(ti[3], 0.0, 0.0)

        self.com_z_traj = placo.CubicSpline()
        self.com_z_traj.add_point(ti[0], 0.23, 0.0)
        self.com_z_traj.add_point(ti[1], 0.20, 0.0)
        self.com_z_traj.add_point(ti[2], 0.17, 0.0)
        self.com_z_traj.add_point(ti[3], 0.23, 0.0)

        self.com_x_traj = placo.CubicSpline()
        self.com_x_traj.add_point(ti[0], 0.0, 0.0)
        self.com_x_traj.add_point(ti[1], 0, 0.0)
        self.com_x_traj.add_point(ti[2], 0, 0.0)
        self.com_x_traj.add_point(ti[3], 0.0, 0.0)

        self.com_y_traj = placo.CubicSpline()
        self.com_y_traj.add_point(ti[0], -0.06, 0.0)
        self.com_y_traj.add_point(ti[1], -0.08, 0.0)
        self.com_y_traj.add_point(ti[2], -0.08, 0.0)
        self.com_y_traj.add_point(ti[3], -0.04, 0.0)

        # Regularization task
        posture_regularization_task = self.solver.add_joints_task()
        posture_regularization_task.set_joints({dof: 0.0 for dof in self.robot.joint_names()})
        posture_regularization_task.configure("reg", "soft", 1e-5)

        # Creating a joint task to assign DoF values for upper body
        elbow = 2.512  # -50 * np.pi / 180
        shoulder_roll = 0 * np.pi / 180
        shoulder_pitch = -0.45  # 20 * np.pi / 180
        joints_task = self.solver.add_joints_task()
        if self.robot_model == "bez2" or self.robot_model == "assembly":
            joints_task.set_joints(
                {
                    # "left_shoulder_roll": shoulder_roll,
                    # "left_shoulder_pitch": shoulder_pitch,
                    "left_elbow": elbow,
                    # "right_shoulder_roll": -shoulder_roll,
                    # "right_shoulder_pitch": shoulder_pitch,
                    "right_elbow": elbow,
                    # "head_pitch": 0.0,
                    # "head_yaw": 0.0,
                }
            )
        else:
            joints_task.set_joints(
                {
                    # "left_shoulder_roll": shoulder_roll,
                    # "left_shoulder_pitch": shoulder_pitch,
                    "left_elbow": elbow,
                    # "right_shoulder_roll": -shoulder_roll,
                    # "right_shoulder_pitch": shoulder_pitch,
                    "right_elbow": elbow,
                    # "head_pitch": 0.0,
                    # "head_yaw": 0.0,
                }
            )
        joints_task.configure("joints", "soft", 1.0)

        self.look_at_ball = self.solver.add_axisalign_task("camera", np.array([1.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0]))
        self.look_at_ball.configure("look_ball", "soft", 1)  # TODO replace with a function that remove_task

    def head_movement(self, target: List[float]):
        # TODO clean up and add a cone or it breaks walking
        ball = np.array(target)
        camera_pos = self.robot.get_T_world_frame("camera")[:3, 3]
        ball[2] -= camera_pos[2]
        self.look_at_ball.targetAxis_world = ball

    def plan_kick(self, t: float):
        t_mod = t % 2.0
        target = self.left_foot_task.position().target_world
        target[0] = self.left_foot_x_traj.pos(t_mod)
        target[1] = self.left_foot_y_traj.pos(t_mod)
        target[2] = self.left_foot_z_traj.pos(t_mod)
        self.left_foot_task.position().target_world = target

        # Updating the com target with lateral sinusoidal trajectory
        target2 = self.com_task.target_world
        target2[0] = self.com_x_traj.pos(t_mod)
        target2[1] = self.com_y_traj.pos(t_mod)
        target2[2] = self.com_z_traj.pos(t_mod)
        self.com_task.target_world = target2

        # Looking at ball
        ball = [0.2, 0.0, 0.0]
        self.head_movement(ball)

        self.solver.solve(True)
        self.robot.update_kinematics()
        self.update_viz(ball)

    def step(self, t: float):
        # Spin-lock until the next tick
        t += self.DT
        # while self.funct_time() < self.start_t + t:
        #     time.sleep(1e-3)

        return t

    def update_viz(self, ball):
        if self.debug:
            self.viz.display(self.robot.state.q)
            robot_frame_viz(self.robot, "trunk")
            robot_frame_viz(self.robot, "camera")
            point_viz("com", self.robot.com_world(), radius=0.025, color=0xAAAAAA)
            point_viz("ball", ball, radius=0.05, color=0xDDDDDD)


# if __name__ == "__main__":
#     walk = FootStepPlanner("bez1", time.time, debug=True)
