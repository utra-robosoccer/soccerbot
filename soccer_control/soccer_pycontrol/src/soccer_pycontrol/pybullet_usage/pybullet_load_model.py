from os.path import expanduser

import pybullet as pb

from soccer_common import Transformation


class LoadModel:  # TODO Maybe rename to body
    """
    Interfaces with pybullet to load a pybullet model and set pose.
    """

    # TODO dont know if i like this file
    def __init__(self, robot_model: str, walking_torso_height: float, pose: Transformation, fixed_base: bool):
        self.pose = pose
        self.robot_model = robot_model

        urdf_model_path = expanduser("~") + f"/catkin_ws/src/soccerbot/soccer_description/{robot_model}" f"_description/urdf/{robot_model}.urdf"
        self.body = self.load_urdf_pybullet(urdf_model_path, fixed_base)
        self.walking_torso_height = walking_torso_height

        if not fixed_base and (self.pose == Transformation()).all():  # TODO need way to have custom height
            self.set_pose(pose)

    def load_urdf_pybullet(self, urdf_model_path: str, fixed_base: bool):  # -> pb.loadURDF:
        # TODO read from yaml? Also maybe put in world
        body = pb.loadURDF(
            urdf_model_path,
            useFixedBase=fixed_base,
            flags=pb.URDF_USE_INERTIA_FROM_FILE | 0,
            basePosition=self.pose.position,
            baseOrientation=self.pose.quaternion,
        )
        return body

    # Pose
    def set_pose(self, pose: Transformation = Transformation()) -> None:
        """
        Teleports the robot to the desired pose

        :param pose: 3D position in pybullet
        """
        self.pose.position = (self.pose.position[0], self.pose.position[1], self.walking_torso_height)

        [y, _, _] = pose.orientation_euler

        self.pose.orientation_euler = [y, 0, 0]
        if pb.isConnected():
            pb.resetBasePositionAndOrientation(self.body, self.pose.position, self.pose.quaternion)
