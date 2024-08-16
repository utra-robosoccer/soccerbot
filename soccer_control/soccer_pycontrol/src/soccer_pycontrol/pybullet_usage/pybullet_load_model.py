import pybullet as pb

from soccer_common import Transformation


class LoadModel:  # TODO Maybe rename to body
    """
    Interfaces with pybullet to load a pybullet model and set pose.
    """

    # TODO dont know if i like this file
    def __init__(self, urdf_model_path: str, walking_torso_height: float, pose: Transformation, fixed_base: bool):
        self.pose = pose

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
    # TODO still dont fully like these solutions
    def set_walking_torso_height(self, pose: Transformation) -> Transformation:
        """
        Takes a 2D pose and sets the height of the pose to the height of the torso
        https://docs.google.com/presentation/d/10DKYteySkw8dYXDMqL2Klby-Kq4FlJRnc4XUZyJcKsw/edit#slide=id.g163c1c67b73_0_0
        """
        # if pose.position[2] < self.walking_torso_height:
        pose.position = (pose.position[0], pose.position[1], self.walking_torso_height)

        return pose

    def set_pose(self, pose: Transformation = Transformation()) -> None:
        """
        Teleports the robot to the desired pose

        :param pose: 3D position in pybullet
        """
        self.pose = self.set_walking_torso_height(pose)
        self.pose.position = (self.pose.position[0], self.pose.position[1], self.walking_torso_height + 0.03)

        [y, _, _] = pose.orientation_euler
        r = 0
        if pb.getNumJoints(self.body) > 20:
            r = -0.64
        self.pose.orientation_euler = [y, 0, r]  # TODO need to fix this
        if pb.isConnected():
            pb.resetBasePositionAndOrientation(self.body, self.pose.position, self.pose.quaternion)
