from copy import deepcopy
from typing import Union

import scipy

# from soccer_pycontrol.exp.calibration import adjust_navigation_transform
from soccer_pycontrol.links import Links
from soccer_pycontrol.path.path_robot import PathRobot
from soccer_pycontrol.soccerbot.handle_urdf import HandleURDF
from soccer_pycontrol.soccerbot.pybullet_env import PybulletEnv
from soccer_pycontrol.soccerbot.pybullet_world import PybulletWorld

from soccer_common import Transformation


class FootStepPlanner:
    def __init__(self, urdf: HandleURDF, use_calibration: bool = False):
        # TODO lots of duplicates with inversekinematics need to fix it
        self.robot_path: Union[PathRobot, None] = None
        self.use_calibration = use_calibration
        self.urdf = urdf

        pitch_correction = Transformation([0, 0, 0], euler=[0, 0.0, 0])
        # Transformation([0, 0, 0], euler=[0, rospy.get_param("torso_offset_pitch", 0.0), 0])
        self.torso_offset = Transformation([0, 0, 0]) @ pitch_correction
        # Transformation([rospy.get_param("torso_offset_x", 0), 0, 0]) @ pitch_correction

        # Odom pose at start of path, reset everytime a new path is created
        # Odom pose, always starts at (0,0) and is the odometry of the robot's movement.
        # All odom paths start from odom pose
        self.odom_pose_start_path = Transformation()
        self.odom_pose = Transformation()

        self.current_step_time = 0

    def createPathToGoal(self, endPose: Transformation) -> PathRobot:
        """
        Creates a path from the robot's current location to the goal location

        :param endPose: 3D transformation
        :return: Robot path
        """
        # TODO why is this here? should be in an integrator
        startPose = self.urdf.set_pose.set_walking_torso_height(self.urdf.pose)
        endPose = self.urdf.set_pose.set_walking_torso_height(endPose)

        # Remove the roll and yaw from the designated position

        [r, p, y] = endPose.orientation_euler
        endPose.orientation_euler = [r, 0, 0]  # TODO this should remove roll & pitch

        # Add calibration
        # TODO add when fixed
        # if self.use_calibration:
        #     endPoseCalibrated = adjust_navigation_transform(startPose, endPose)
        # else:
        endPoseCalibrated = endPose

        # print( f"\033[92mEnd Pose Calibrated: Position (xyz) [{endPoseCalibrated.position[0]:.3f} {
        # endPoseCalibrated.position[1]:.3f} {endPoseCalibrated.position[2]:.3f}], " f"Orientation (xyzw) [{
        # endPoseCalibrated.quaternion[0]:.3f} {endPoseCalibrated.quaternion[1]:.3f} {endPoseCalibrated.quaternion[
        # 2]:.3f} {endPoseCalibrated.quaternion[3]:.3f}]\033[0m" )

        self.robot_path = PathRobot(startPose, endPoseCalibrated, self.urdf.ik_data.foot_center_to_floor)

        # obj.rate = rateControl(1 / obj.robot_path.step_size); -- from findPath
        self.rate = 1 / self.robot_path.step_precision
        # TODO this edits the rate for the controller need to figure out how to use it
        self.period = self.robot_path.step_precision  # todo IS THIS USED

        self.current_step_time = 0

        self.odom_pose_start_path = deepcopy(self.odom_pose)
        # TODO add unit test
        return self.robot_path

    def stepPath(self, t):
        """
        Updates the configuration for the robot for the next position t based on the current path

        :param t: Timestep relative to the time of the first path, where t=0 is the beginning of the path
        """

        assert t <= self.robot_path.duration()

        # Get Torso position (Average Time: 0.0007538795471191406)
        torso_position = self.robot_path.torsoPosition(t) @ self.torso_offset

        # Get foot position at time (Average Time: 0.0004878044128417969)
        [right_foot_position, left_foot_position] = self.robot_path.footPosition(t)

        # Calcualate the feet position relative from torso (Average Time: 0.000133514404296875)
        torso_to_left_foot = scipy.linalg.lstsq(torso_position, left_foot_position, lapack_driver="gelsy")[0]
        torso_to_right_foot = scipy.linalg.lstsq(torso_position, right_foot_position, lapack_driver="gelsy")[0]

        self.urdf.pose = torso_position
        # Inverse kinematics for both feet (Average Time: 0.0015840530395507812)
        # right_thetas = self.inverseKinematicsRightFoot(torso_to_right_foot)
        # self.configuration[Links.RIGHT_LEG_1: Links.RIGHT_LEG_6 + 1] = thetas[0:6]
        #
        # left_thetas = self.inverseKinematicsLeftFoot(torso_to_left_foot)
        # self.configuration[Links.LEFT_LEG_1: Links.LEFT_LEG_6 + 1] = thetas[0:6]

        # TODO add unit test
        return torso_to_right_foot, torso_to_left_foot


if __name__ == "__main__":
    world = PybulletWorld(path="")
    model = HandleURDF(fixed_base=True)
    p = PybulletEnv(model, world, real_time=True, rate=100)
    fp = FootStepPlanner(p.handle_urdf)
    p.wait(50)
    p.motor_control.set_target_angles(p.ik_actions.ready())
    fp.createPathToGoal(Transformation([5, 0, 0], [0, 0, 0, 1]))
    # p.wmodelait(100)

    pitches = []
    times = []
    t = 0

    while t <= fp.robot_path.duration():
        if fp.current_step_time <= t <= fp.robot_path.duration():
            torso_to_right_foot, torso_to_left_foot = fp.stepPath(t)
            r_theta = p.ik_actions.ik.ik_right_foot(torso_to_right_foot)
            l_theta = p.ik_actions.ik.ik_left_foot(torso_to_left_foot)
            p.motor_control.set_right_leg_target_angles(r_theta[0:6])
            p.motor_control.set_left_leg_target_angles(l_theta[0:6])
            # pitch = self.walker.soccerbot.get_imu().orientation_euler[1]
            # f = self.walker.soccerbot.apply_imu_feedback(t, self.walker.soccerbot.get_imu())
            fp.current_step_time = fp.current_step_time + 0.01  # fp.robot_path.step_precision
            times.append(t)
            # pitches.append((pitch, f))
        p.step()
        t = t + 0.01

    p.wait(100)
