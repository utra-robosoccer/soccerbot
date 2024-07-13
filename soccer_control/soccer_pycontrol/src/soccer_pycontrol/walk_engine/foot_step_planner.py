from copy import deepcopy
from typing import Tuple, Union

import scipy

# from soccer_pycontrol.exp.calibration import adjust_navigation_transform
from soccer_pycontrol.path.path_robot import PathRobot

from soccer_common import Transformation


class FootStepPlanner:
    """
    Class to interface with path for robot foot steps.
    """

    def __init__(
        self,
        use_calibration: bool = False,
        sim: str = "_sim",
        robot_model: str = "bez1",
        torso_offset_pitch: float = 0.0,
        torso_offset_x: float = 0.0,
        walking_torso_height: float = 0.40,
        foot_center_to_floor: float = 0.0221,
    ):
        self.robot_model = robot_model
        self.sim = sim

        self.robot_path: Union[PathRobot, None] = None
        self.use_calibration = use_calibration

        self.walking_torso_height = walking_torso_height
        self.foot_center_to_floor = foot_center_to_floor
        pitch_correction = Transformation([0, 0, 0], euler=[0, torso_offset_pitch, 0])
        self.torso_offset = Transformation([torso_offset_x, 0, 0]) @ pitch_correction

        # Odom pose at start of path, reset everytime a new path is created
        # Odom pose, always starts at (0,0) and is the odometry of the robot's movement.
        # All odom paths start from odom pose
        self.odom_pose_start_path = Transformation()
        self.odom_pose = Transformation()

        self.current_step_time = 0

    def create_path_to_goal(self, start_pose: Transformation, end_pose: Transformation) -> PathRobot:
        """
        Creates a path from the robot's current location to the goal location

        :param end_pose: 3D transformation
        :return: Robot path
        """
        # TODO is this the best place for it?
        start_pose = Transformation(position=[start_pose.position[0], start_pose.position[1], self.walking_torso_height])
        end_pose.position = [end_pose.position[0], end_pose.position[1], self.walking_torso_height]

        # Remove the roll and pitch from the designated position
        [y, _, _] = end_pose.orientation_euler
        end_pose.orientation_euler = [y, 0, 0]

        # Add calibration
        # TODO add when fixed
        # if self.use_calibration:
        #     end_pose_calibrated = adjust_navigation_transform(start_pose, end_pose)
        # else:
        end_pose_calibrated = end_pose

        # print( f"\033[92mEnd Pose Calibrated: Position (xyz) [{end_pose_calibrated.position[0]:.3f} {
        # end_pose_calibrated.position[1]:.3f} {end_pose_calibrated.position[2]:.3f}], " f"Orientation (xyzw) [{
        # end_pose_calibrated.quaternion[0]:.3f} {end_pose_calibrated.quaternion[1]:.3f} {end_pose_calibrated.quaternion[
        # 2]:.3f} {end_pose_calibrated.quaternion[3]:.3f}]\033[0m" )

        self.robot_path = PathRobot(
            start_pose, end_pose_calibrated, foot_center_to_floor=self.foot_center_to_floor, sim=self.sim, robot_model=self.robot_model
        )

        # TODO this edits the rate for the controller need to figure out how to use it
        self.current_step_time = 0

        self.odom_pose_start_path = deepcopy(self.odom_pose)
        # TODO add unit test
        return self.robot_path

    def get_next_step(self, t: float) -> Tuple[Transformation, Transformation]:
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

        # TODO is this needed ?
        # self.urdf.pose = torso_position

        # Inverse kinematics for both feet (Average Time: 0.0015840530395507812)

        # TODO add unit test
        return torso_to_right_foot, torso_to_left_foot
