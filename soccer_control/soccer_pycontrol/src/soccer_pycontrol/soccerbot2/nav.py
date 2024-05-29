import pybullet as pb
from soccer_pycontrol.links import Links
from soccer_pycontrol.soccerbot2.pybullet.pybullet_env import PybulletEnv

from soccer_common.transformation import Transformation


class Nav:
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

    def set_goal(self, goal: Transformation) -> None:
        """
        Set the goal of the robot, will create the path to the goal that will be executed in the run() loop

        :param goal: The 3D location goal for the robot
        """
        self.env.step_planner.create_path_to_goal(goal)

    def stand(self, timer: float) -> None:
        while self.t < timer:
            [_, pitch, roll] = self.env.sensors.get_euler_angles()
            pb.applyExternalForce(self.env.model.body, Links.TORSO, [5, 0, 0], [0, 0, 0], pb.LINK_FRAME)
            self.stabilize_stand(pitch, roll)
            self.env.step()
            self.t = self.t + 0.01

    def walk(self) -> bool:
        """
        The main run loop for the navigator, executes goals given through setGoal and then stops

        :return: True if the robot succeeds navigating to the goal, False if it doesn't reach the goal and falls
        """

        self.t = -self.prepare_walk_time
        stable_count = 20
        self.env.pid.reset_imus()

        while True:
            [_, pitch, roll] = self.env.sensors.get_euler_angles()

            if self.t <= self.env.step_planner.robot_path.duration():

                torso_to_right_foot, torso_to_left_foot = self.env.step_planner.get_next_step(self.t)
                r_theta = self.env.ik_actions.get_right_leg_angles(torso_to_right_foot)
                l_theta = self.env.ik_actions.get_left_leg_angles(torso_to_left_foot)
                self.env.motor_control.set_right_leg_target_angles(r_theta[0:6])
                self.env.motor_control.set_left_leg_target_angles(l_theta[0:6])

                F = self.env.pid.walking_pitch_pid.update(pitch)
                self.env.motor_control.set_leg_joint_3_target_angle(F)

                F = self.env.pid.walking_roll_pid.update(roll)
                self.env.motor_control.set_leg_joint_2_target_angle(F)

                self.env.motor_control.set_motor()

            else:
                self.stabilize_stand(pitch, roll)
                if abs(pitch - self.env.pid.standing_pitch_pid.setpoint) < 0.025 and abs(roll - self.env.pid.standing_roll_pid.setpoint) < 0.025:
                    stable_count -= 1
                    if stable_count == 0:
                        if self.t < 0:
                            self.t = 0
                            stable_count = 20
                        else:
                            break
                else:
                    stable_count = 5

            if self.fallen(pitch):
                return False

            self.env.step()
            self.t = self.t + 0.01
        return True

    def stabilize_stand(self, pitch: float, roll: float) -> None:
        error_pitch = self.env.pid.standing_pitch_pid.update(pitch)
        self.env.motor_control.set_leg_joint_3_target_angle(-error_pitch)
        print(error_pitch)
        # error_roll = self.env.pid.standing_roll_pid.update(roll)
        # self.env.motor_control.set_leg_joint_2_target_angle(error_roll)

        self.env.motor_control.set_motor()

    @staticmethod
    def fallen(pitch: float) -> bool:
        angle_threshold = 1.25  # in radian
        if pitch > angle_threshold:
            print("Fallen Back")
            return True

        elif pitch < -angle_threshold:
            print("Fallen Front")
            return True
