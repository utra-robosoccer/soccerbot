import time
from os.path import expanduser
from typing import List

import numpy as np
import placo
from placo_utils.visualization import footsteps_viz, frame_viz, line_viz, robot_viz


class WalkPlaco:
    def __init__(self, debug: bool = True):

        self.DT = 0.005

        self.debug = debug

        model_filename = expanduser("~") + "/catkin_ws/src/soccerbot/soccer_description/bez1_description/urdf/robot.urdf"

        self.last_replan = 0
        self.start_t = time.time()
        # TODO is there too many global var
        self.robot = placo.HumanoidRobot(model_filename)
        self.parameters = self.walk_parameters()
        self.walk_pattern = placo.WalkPatternGenerator(self.robot, self.parameters)
        self.repetitive_footsteps_planner = placo.FootstepsPlannerRepetitive(self.parameters)
        self.trajectory = None
        self.tasks = None
        self.solver = None

        if self.debug:
            # Starting Meshcat viewer
            self.viz = robot_viz(self.robot)
            self.last_display = time.time()

    @staticmethod
    def walk_parameters():
        # Walk parameters - if double_support_ratio is not set to 0, should be greater than replan_frequency
        parameters = placo.HumanoidParameters()

        # Timing parameters
        parameters.single_support_duration = 0.25  # Duration of single support phase [s]
        parameters.single_support_timesteps = 10  # Number of planning timesteps per single support phase
        parameters.double_support_ratio = 0.0  # Ratio of double support (0.0 to 1.0)
        parameters.startend_double_support_ratio = 2.0  # Ratio duration of supports for starting and stopping walk
        parameters.planned_timesteps = 48  # Number of timesteps planned ahead
        parameters.replan_timesteps = 10  # Replanning each n timesteps    # 50

        # Posture parameters
        parameters.walk_com_height = 0.21  # Constant height for the CoM [m]
        parameters.walk_foot_height = 0.04  # Height of foot rising while walking [m]
        parameters.walk_trunk_pitch = 0.0  # Trunk pitch angle [rad]
        parameters.walk_foot_rise_ratio = 0.2  # Time ratio for the foot swing plateau (0.0 to 1.0)

        # Feet parameters
        parameters.foot_length = 0.1200  # Foot length [m]
        parameters.foot_width = 0.072  # Foot width [m]
        parameters.feet_spacing = 0.122  # Lateral feet spacing [m]
        parameters.zmp_margin = 0.02  # ZMP margin [m]
        parameters.foot_zmp_target_x = 0.0  # Reference target ZMP position in the foot [m]
        parameters.foot_zmp_target_y = 0.0  # Reference target ZMP position in the foot [m]

        # Limit parameters
        parameters.walk_max_dtheta = 1  # Maximum dtheta per step [rad]
        parameters.walk_max_dy = 0.04  # Maximum dy per step [m]
        parameters.walk_max_dx_forward = 0.08  # Maximum dx per step forward [m]
        parameters.walk_max_dx_backward = 0.03  # Maximum dx per step backward [m]
        return parameters

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

        # return solver, tasks

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

        self.last_display = time.time()
        self.last_replan = 0
        self.start_t = time.time()

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
        while time.time() < self.start_t + t:
            time.sleep(1e-3)

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
            if time.time() - self.last_display > 0.03:
                self.last_display = time.time()
                self.viz.display(self.robot.state.q)

                frame_viz("left_foot_target", self.trajectory.get_T_world_left(t))
                frame_viz("right_foot_target", self.trajectory.get_T_world_right(t))

                T_world_trunk = np.eye(4)
                T_world_trunk[:3, :3] = self.trajectory.get_R_world_trunk(t)
                T_world_trunk[:3, 3] = self.trajectory.get_p_world_CoM(t)
                frame_viz("trunk_target", T_world_trunk)


if __name__ == "__main__":
    walk = WalkPlaco(debug=True)
