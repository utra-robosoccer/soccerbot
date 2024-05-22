import pybullet as pb

from soccer_common import Transformation


class SetPose:
    def __init__(self, body: pb.loadURDF, pose: Transformation = Transformation(), fixed_base: bool = False, walking_torso_height: float = 0.315):
        self.pose = pose
        self.body = body
        self.walking_torso_height = walking_torso_height

        if not fixed_base:
            self.set_pose(pose)

    # Pose
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

        # Remove the roll and yaw from the pose TODO why also your code and comment is wrong
        [r, p, y] = pose.orientation_euler
        self.pose.orientation_euler = [r, 0, 0]
        if pb.isConnected():
            pb.resetBasePositionAndOrientation(self.body, self.pose.position, self.pose.quaternion)
