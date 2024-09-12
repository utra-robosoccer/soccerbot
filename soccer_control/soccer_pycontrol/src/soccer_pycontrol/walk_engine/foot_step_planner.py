import time
from os.path import expanduser
from typing import List

from soccer_pycontrol.model.bez import Bez
import numpy as np
import placo
from placo_utils.visualization import footsteps_viz, frame_viz, line_viz, robot_viz


class FootStepPlanner:
    def __init__(self, robot_model: str, parameters: dict, funct_time, debug: bool = True):
        self.funct_time = funct_time
        self.DT = parameters["control_frequency"]

        self.debug = debug
        self.robot_model = robot_model
        model_filename = expanduser("~") + f"/catkin_ws/src/soccerbot/soccer_description/{robot_model}_description/urdf/robot.urdf"
        self.parameters = self.walk_parameters(parameters)

        self.last_replan = 0
        self.start_t = self.funct_time()
        # TODO is there too many global var
        print(model_filename)
        self.robot = placo.HumanoidRobot(model_filename)

        self.walk_pattern = placo.WalkPatternGenerator(self.robot, self.parameters)
        self.repetitive_footsteps_planner = placo.FootstepsPlannerRepetitive(self.parameters)
        self.trajectory = None
        self.tasks = None
        self.solver = None

        if self.debug:
            # Starting Meshcat viewer
            self.viz = robot_viz(self.robot)
            self.last_display = self.funct_time()

    @staticmethod
    def walk_parameters(parameters: dict):
        placo_parameters = placo.HumanoidParameters()

        # Timing parameters
        placo_parameters.single_support_duration = parameters["single_support_duration"]
        placo_parameters.single_support_timesteps = parameters["single_support_timesteps"]
        placo_parameters.double_support_ratio = parameters["double_support_ratio"]
        placo_parameters.startend_double_support_ratio = parameters["startend_double_support_ratio"]
        placo_parameters.planned_timesteps = parameters["planned_timesteps"]
        placo_parameters.replan_timesteps = parameters["replan_timesteps"]

        # Posture parameters
        placo_parameters.walk_com_height = parameters["walk_com_height"]
        placo_parameters.walk_foot_height = parameters["walk_foot_height"]
        placo_parameters.walk_trunk_pitch = parameters["walk_trunk_pitch"]
        placo_parameters.walk_foot_rise_ratio = parameters["walk_foot_rise_ratio"]

        # Feet parameters
        placo_parameters.foot_length = parameters["foot_length"]
        placo_parameters.foot_width = parameters["foot_width"]
        placo_parameters.feet_spacing = parameters["feet_spacing"]
        placo_parameters.zmp_margin = parameters["zmp_margin"]
        placo_parameters.foot_zmp_target_x = parameters["foot_zmp_target_x"]
        placo_parameters.foot_zmp_target_y = parameters["foot_zmp_target_y"]

        # Limit parameters
        placo_parameters.walk_max_dtheta = parameters["walk_max_dtheta"]
        placo_parameters.walk_max_dy = parameters["walk_max_dy"]
        placo_parameters.walk_max_dx_forward = parameters["walk_max_dx_forward"]
        placo_parameters.walk_max_dx_backward = parameters["walk_max_dx_backward"]
        return placo_parameters

    # TODO can this be in init
    def setup_tasks(self):
        # Creating the kinematics solver
        self.solver = placo.KinematicsSolver(self.robot)
        self.solver.enable_velocity_limits(True)
        self.solver.dt = self.DT

        # Creating the walk QP tasks
        self.tasks = placo.WalkTasks()
        self.tasks.initialize_tasks(self.solver, self.robot)

        # Creating a joint task to assign DoF values for upper body
        elbow = -50 * np.pi / 180
        shoulder_roll = 0 * np.pi / 180
        shoulder_pitch = 20 * np.pi / 180
        joints_task = self.solver.add_joints_task()
        if self.robot_model == "bez2":
            joints_task.set_joints(
                {
                    "left_shoulder_roll": shoulder_roll,
                    "left_shoulder_pitch": shoulder_pitch,
                    "left_elbow": elbow,
                    "right_shoulder_roll": -shoulder_roll,
                    "right_shoulder_pitch": shoulder_pitch,
                    "right_elbow": elbow,
                    "head_pitch": 0.0,
                    "head_yaw": 0.0,
                }
            )
        else:
            joints_task.set_joints(
                {
                    # "left_shoulder_roll": shoulder_roll,
                    "left_shoulder_pitch": shoulder_pitch,
                    "left_elbow": elbow,
                    # "right_shoulder_roll": -shoulder_roll,
                    "right_shoulder_pitch": shoulder_pitch,
                    "right_elbow": elbow,
                    "head_pitch": 0.0,
                    "head_yaw": 0.0,
                }
            )
        joints_task.configure("joints", "soft", 1.0)

        # Placing the robot in the initial position
        print("Placing the robot in the initial position...")
        self.tasks.reach_initial_pose(
            np.eye(4),
            self.parameters.feet_spacing,
            self.parameters.walk_com_height,
            self.parameters.walk_trunk_pitch,
        )
        print("Initial position reached")

    def configure_planner(self, d_x: float = 0.0, d_y: float = 0.0, d_theta: float = 0.0, nb_steps: int = 10):
        # Configure the FootstepsPlanner
        self.repetitive_footsteps_planner.configure(d_x, d_y, d_theta, nb_steps)

    def setup_footsteps(self):
        # Planning footsteps
        T_world_left = placo.flatten_on_floor(self.robot.get_T_world_left())
        T_world_right = placo.flatten_on_floor(self.robot.get_T_world_right())
        footsteps = self.repetitive_footsteps_planner.plan(placo.HumanoidRobot_Side.left, T_world_left, T_world_right)

        supports = placo.FootstepsPlanner.make_supports(footsteps, True, self.parameters.has_double_support(), True)

        # Creating the pattern generator and making an initial plan
        self.trajectory = self.walk_pattern.plan(supports, self.robot.com_world(), 0.0)

    def setup_walk(self, d_x: float = 0.0, d_y: float = 0.0, d_theta: float = 0.0, nb_steps: int = 10):
        self.setup_tasks()

        self.configure_planner(d_x, d_y, d_theta, nb_steps)

        self.setup_footsteps()

        self.last_display = self.funct_time()
        self.last_replan = 0
        self.start_t = self.funct_time()

    def walk_loop(
        self,
        t: float,
    ):
        # Updating the QP tasks from planned trajectory
        self.tasks.update_tasks_from_trajectory(self.trajectory, t)

        # Invoking the IK QP solver
        self.robot.update_kinematics()
        qd_sol = self.solver.solve(True)

        # Ensuring the robot is kinematically placed on the floor on the proper foot to avoid integration drifts
        if not self.trajectory.support_is_both(t):
            self.robot.update_support_side(str(self.trajectory.support_side(t)))
            self.robot.ensure_on_floor()

        # If enough time elapsed and we can replan, do the replanning
        if t - self.last_replan > self.parameters.replan_timesteps * self.parameters.dt() and self.walk_pattern.can_replan_supports(
            self.trajectory, t
        ):

            self.last_replan = t
            # Replanning footsteps from current trajectory
            supports = self.walk_pattern.replan_supports(self.repetitive_footsteps_planner, self.trajectory, t)

            # Replanning CoM trajectory, yielding a new trajectory we can switch to
            self.trajectory = self.walk_pattern.replan(supports, self.trajectory, t)

            self.update_viz(supports, self.trajectory)

        # During the warmup phase, the robot is enforced to stay in the initial position
        self.update_meshcat(t)

    def step(self, t: float):
        # Spin-lock until the next tick
        t += self.DT
        # while self.funct_time() < self.start_t + t:
        #     time.sleep(1e-3)

        return t

    def update_viz(self, supports: List[placo.Supports], trajectory: placo.WalkTrajectory):
        if self.debug:
            # Drawing footsteps
            footsteps_viz(supports)

            # Drawing planned CoM trajectory on the ground
            coms = [[*trajectory.get_p_world_CoM(t)[:2], 0.0] for t in np.linspace(trajectory.t_start, trajectory.t_end, 100)]
            line_viz("CoM_trajectory", np.array(coms), 0xFFAA00)

    def update_meshcat(self, t: float):
        # Updating meshcat display periodically
        if self.debug:
            if self.funct_time() - self.last_display > 0.03:
                self.last_display = self.funct_time()
                self.viz.display(self.robot.state.q)

                frame_viz("left_foot_target", self.trajectory.get_T_world_left(t))
                frame_viz("right_foot_target", self.trajectory.get_T_world_right(t))

                T_world_trunk = np.eye(4)
                T_world_trunk[:3, :3] = self.trajectory.get_R_world_trunk(t)
                T_world_trunk[:3, 3] = self.trajectory.get_p_world_CoM(t)
                frame_viz("trunk_target", T_world_trunk)


if __name__ == "__main__":
    def walk(d_x: float = 0.0, d_y: float = 0.0, d_theta: float = 0.0, nb_steps: int = 10, t_goal: float = 10):
        bez = Bez(robot_model="bez3")
        planner = FootStepPlanner(bez.robot_model, bez.parameters, time.time)
        planner.setup_walk(d_x, d_y, d_theta, nb_steps)
        t = 0
        t0 = time.time()
        while True:
            t = planner.step(t) # time.time() - t0)
            planner.walk_loop(t)
            # print(t - (time.time() - t0))
    walk(d_x=0.08, t_goal=30)
