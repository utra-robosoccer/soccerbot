from os.path import expanduser

import numpy as np
import pybullet as pb
from soccer_pycontrol.links import Links
from soccer_pycontrol.soccerbot.ik_data import IKData
from soccer_pycontrol.soccerbot.set_pose import SetPose

from soccer_common import Transformation


class HandleURDF:  # TODO Maybe rename to body
    def __init__(self, pose: Transformation = Transformation(), robot_model: str = "bez1", fixed_base: bool = False):

        self.pose = pose
        self.robot_model = robot_model
        self.fixed_base = fixed_base

        self.body = self.load_urdf()

        # TODO put in ik_data maybe?
        #: Height of the robot's torso (center between two arms) while walking
        self.walking_torso_height = 0.315  # rospy.get_param("walking_torso_height", 0.315)

        #: Dimensions of the foot collision box #TODO get it from URDF also what do they mean
        self.foot_box = [0.09, 0.07, 0.01474]
        #  rospy.get_param("foot_box", [0.09, 0.07, 0.01474])

        # : Transformations from the right foots joint position to the center of the collision box of the foot
        # https://docs.google.com/presentation/d/10DKYteySkw8dYXDMqL2Klby-Kq4FlJRnc4XUZyJcKsw/edit#slide=id.g163c1c67b73_0_0
        self.right_foot_joint_center_to_collision_box_center = [0.00385, 0.00401, -0.00737]
        # rospy.get_param("right_foot_joint_center_to_collision_box_center", [0.00385, 0.00401, -0.00737])

        self.ik_data = self.load_ik_data()
        # TODO not sure if i like this setup
        self.set_pose = SetPose(self.body, self.pose, self.fixed_base, self.walking_torso_height)

    def load_urdf(self) -> pb.loadURDF:
        # TODO read from yaml?
        home = expanduser("~")
        body = pb.loadURDF(
            home + f"/catkin_ws/src/soccerbot/soccer_description/{self.robot_model}" f"_description/urdf/{self.robot_model}.urdf",
            useFixedBase=self.fixed_base,
            flags=pb.URDF_USE_INERTIA_FROM_FILE | 0,
            basePosition=self.pose.position,
            baseOrientation=self.pose.quaternion,
        )
        return body

    def get_link_transformation(self, link1, link2):
        """
        Gives the H-trasnform between two links
        :param link1: Starting link
        :param link2: Ending link
        :return: H-transform from starting link to the ending link
        """
        # TODO compare with calc fwd kinematics
        if link1 == Links.TORSO:
            link1world = ((0, 0, 0), (0, 0, 0, 1))
        else:
            link1world = pb.getLinkState(self.body, link1)[4:6]

        if link2 == Links.TORSO:
            link2world = ((0, 0, 0), (0, 0, 0, 1))
        else:
            link2world = pb.getLinkState(self.body, link2)[4:6]

        link1worldrev = pb.invertTransform(link1world[0], link1world[1])

        final_transformation = pb.multiplyTransforms(link2world[0], link2world[1], link1worldrev[0], link1worldrev[1])
        return Transformation(np.round(list(final_transformation[0]), 5), np.round(list(final_transformation[1]), 5))

    def load_ik_data(self):
        ik_data = IKData(
            thigh_length=self.get_link_transformation(Links.RIGHT_LEG_4, Links.RIGHT_LEG_3).position[2],
            tibia_length=self.get_link_transformation(Links.RIGHT_LEG_5, Links.RIGHT_LEG_4).position[2],
            torso_to_right_hip=self.get_link_transformation(Links.TORSO, Links.RIGHT_LEG_1),
            right_hip_to_left_hip=self.get_link_transformation(Links.LEFT_LEG_1, Links.RIGHT_LEG_1),
            right_foot_init_position=self.get_link_transformation(Links.TORSO, Links.RIGHT_LEG_6),
            left_foot_init_position=self.get_link_transformation(Links.TORSO, Links.LEFT_LEG_6),
            foot_center_to_floor=-self.right_foot_joint_center_to_collision_box_center[2] + self.foot_box[2],
            # TODO what does this do exactly and why do we need it
            walking_torso_height=self.walking_torso_height,
        )
        return ik_data
