import pybullet as pb
from soccer_pycontrol.links import Links
from soccer_pycontrol.soccerbot2.pybullet.pybullet_env import PybulletEnv

from soccer_common.transformation import Transformation


class TrajFollowing:
    """
    The 2D Navigator class, has a running loop that reads commands by the user and outputs actions to the soccerbot
    class.
    Doesn't require ROS and used for unit tests. All functions called here should be related to pybullet simulation
    """

    # PYBULLET_STEP = rospy.get_param("control_frequency", 0.01)

    def __init__(self, env: PybulletEnv):
        """
        Initialize the Navigator

        :param display: Whether or not to show the pybullet visualization, turned off for quick unit tests
        :param useCalibration: Whether or not to use movement calibration files located in config/robot_model.yaml, which adjusts the calibration to the movement given
        """
        self.env = env

        self.terminate_walk = False
        self.prepare_walk_time = 2  # rospy.get_param("prepare_walk_time", 2)

        self.t = 0

    def ready(self) -> None:
        """
        Puts the robot into a ready pose to begin walking
        """
        self.env.motor_control.set_target_angles(self.env.ik_actions.ready())
        self.env.motor_control.set_motor()

    def getPose(self):
        """
        Get the 3D pose of the robot

        :return: The 3D pose of the robot
        """
        [position, quaternion] = pb.getLinkState(self.env.body, linkIndex=Links.LEFT_LEG_6)[4:6]
        return Transformation(position=position, quaternion=quaternion).pos_theta

    def setGoal(self, goal: Transformation) -> None:
        """
        Set the goal of the robot, will create the path to the goal that will be executed in the run() loop

        :param goal: The 3D location goal for the robot
        """
        self.env.step_planner.createPathToGoal(goal)

    def run(self, single_trajectory=False) -> bool:
        """
        The main run loop for the navigator, executes goals given through setGoal and then stops

        :param single_trajectory: If set to true, then the software will exit after a single trajectory is completed
        :return: True if the robot succeeds navigating to the goal, False if it doesn't reach the goal and falls
        """
        logging_id = pb.startStateLogging(pb.STATE_LOGGING_GENERIC_ROBOT, "/tmp/simulation_record.bullet", physicsClientId=self.env.world.client_id)

        if self.env.step_planner.robot_path.duration() == 0:
            pb.stopStateLogging(logging_id)
            return True

        self.t = -self.prepare_walk_time
        stable_count = 20
        self.env.pid.reset_imus()
        self.env.pid.reset_roll_feedback_parameters(self.env.step_planner.robot_path)

        while self.t <= self.env.step_planner.robot_path.duration():
            if self.t < 0:
                F, pitch = self.env.pid.apply_imu_feedback_standing(self.env.sensors.get_imu())
                self.env.motor_control.set_leg_joint_5_target_angle(F)
                self.env.motor_control.set_motor()
                if abs(pitch - self.env.pid.standing_pid.setpoint) < 0.025:
                    stable_count = stable_count - 1
                    if stable_count == 0:
                        self.t = 0
                else:
                    stable_count = 5
            else:
                if self.env.step_planner.current_step_time <= self.t <= self.env.step_planner.robot_path.duration():
                    imu = self.env.sensors.get_imu()
                    t_offset = self.env.pid.apply_phase_difference_roll_feedback(self.t, imu, self.env.step_planner.robot_path)
                    torso_to_right_foot, torso_to_left_foot = self.env.step_planner.stepPath(self.t)
                    r_theta = self.env.ik_actions.ik.ik_right_foot(torso_to_right_foot)
                    l_theta = self.env.ik_actions.ik.ik_left_foot(torso_to_left_foot)
                    self.env.motor_control.set_right_leg_target_angles(r_theta[0:6])
                    self.env.motor_control.set_left_leg_target_angles(l_theta[0:6])
                    F = self.env.pid.apply_imu_feedback(imu)
                    self.env.motor_control.set_leg_joint_3_target_angle(F)
                    # F, pitch = self.env.pid.apply_imu_feedback_standing(self.env.sensors.get_imu())
                    # self.env.motor_control.set_leg_joint_5_target_angle(F)

                    self.env.motor_control.set_motor()
                    self.env.step_planner.current_step_time = (
                        self.env.step_planner.current_step_time + self.env.step_planner.robot_path.step_precision
                    )

            angle_threshold = 1.25  # in radian
            [_, pitch, _] = self.env.sensors.get_imu().orientation_euler
            if pitch > angle_threshold:
                print("Fallen Back")
                pb.stopStateLogging(logging_id)
                return False

            elif pitch < -angle_threshold:
                print("Fallen Front")
                pb.stopStateLogging(logging_id)
                return False

            self.env.step()
            self.t = self.t + 0.01
        pb.stopStateLogging(logging_id)
        return True
