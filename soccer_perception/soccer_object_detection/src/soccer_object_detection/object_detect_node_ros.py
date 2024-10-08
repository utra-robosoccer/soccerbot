#!/usr/bin/env python3

import os
from os.path import expanduser

import numpy as np
import tf
from rospy.impl.tcpros_base import DEFAULT_BUFF_SIZE
from soccer_object_detection.camera.camera_calculations_ros import CameraCalculationsRos
from soccer_object_detection.object_detect_node import ObjectDetectionNode, bcolors

from soccer_common import Transformation

if "ROS_NAMESPACE" not in os.environ:
    os.environ["ROS_NAMESPACE"] = "/robot1"

from argparse import ArgumentParser

import rospy
import torch
from cv_bridge import CvBridge
from rospy import ROSException
from sensor_msgs.msg import Image

from soccer_msgs.msg import BoundingBoxes

# TODO should be somewhere else


class ObjectDetectionNodeRos(ObjectDetectionNode):
    """
    Ros bridge
    input: 480x640x4 bgra8 -> output: 3x200x150
    """

    def __init__(self, model_path):
        super(ObjectDetectionNodeRos, self).__init__(model_path)
        self.cover_horizon_up_threshold = rospy.get_param("cover_horizon_up_threshold", 30)
        self.CONFIDENCE_THRESHOLD = rospy.get_param("~ball_confidence_threshold", 0.75)

        if torch.cuda.is_available():
            rospy.loginfo(f"{bcolors.OKGREEN}Using CUDA for object detection{bcolors.ENDC}")

        self.robot_name = rospy.get_namespace()[1:-1]  # remove '/'
        self.camera = CameraCalculationsRos(self.robot_name)
        self.camera.reset_position()

        # Params
        self.br = CvBridge()
        self.br2 = tf.TransformBroadcaster()

        # ROS
        self.pub_detection = rospy.Publisher("/robot1/detection_image", Image, queue_size=1, latch=True)
        self.pub_boundingbox = rospy.Publisher("/robot1/object_bounding_boxes", BoundingBoxes, queue_size=1, latch=True)
        self.image_subscriber = rospy.Subscriber(
            "/robot1/camera/image_raw", Image, self.callback, queue_size=1, buff_size=DEFAULT_BUFF_SIZE * 64
        )  # Large buff size (https://answers.ros.org/question/220502/image-subscriber-lag-despite-queue-1/)

    #     self.game_state_subscriber = rospy.Subscriber("gamestate", GameState, self.game_state_callback)
    #     self.game_state = GameState()
    #
    # def game_state_callback(self, game_state: GameState):
    #     self.game_state = game_state

    def callback(self, msg: Image):
        # webots: 480x640x4pixels
        # TODo should be in strategy
        # if self.robot_state.status not in [
        #     RobotState.STATUS_LOCALIZING,
        #     RobotState.STATUS_READY,
        #     RobotState.ROLE_UNASSIGNED,
        # ]:
        #     return
        #
        # if self.game_state.gameState != GameState.GAMESTATE_PLAYING:
        #     return
        rospy.loginfo_once("Object Detection Receiving image")
        # width x height x channels (bgra8)
        image = self.br.imgmsg_to_cv2(msg)
        self.camera.reset_position(timestamp=msg.header.stamp)  # msg->image

        detection_image, bbs_msg = self.get_model_output(image)

        bbs_msg.header = msg.header
        try:
            if self.pub_detection.get_num_connections() > 0:
                self.pub_detection.publish(self.br.cv2_to_imgmsg(detection_image, encoding="bgr8"))

            if self.pub_boundingbox.get_num_connections() > 0 and len(bbs_msg.bounding_boxes) > 0:
                self.pub_boundingbox.publish(bbs_msg)
            # TODO
            for box in bbs_msg.bounding_boxes:
                if box.Class == "0":
                    boundingBoxes = [[box.xmin, box.ymin], [box.xmax, box.ymax]]
                    ball_pose = self.camera.calculate_ball_from_bounding_boxes(0.07, boundingBoxes)
                    self.br2.sendTransform(
                        ball_pose.position,
                        ball_pose.quaternion,
                        msg.header.stamp,
                        "robot1" + "/ball",
                        "robot1" + "/camera",
                    )
                    pos = [box.xbase, box.ybase]
                    floor_coordinate_robot = self.camera.find_floor_coordinate(pos)
                    world_to_obstacle = Transformation(position=floor_coordinate_robot)
                    camera_to_obstacle = np.linalg.inv(self.camera.pose) @ world_to_obstacle
                    self.br2.sendTransform(
                        camera_to_obstacle.position,
                        camera_to_obstacle.quaternion,
                        msg.header.stamp,
                        "robot1" + "/ball_2",
                        "robot1" + "/camera",
                    )

                elif box.Class == "1":
                    pos = [box.xbase, box.ybase]

                    floor_coordinate_robot = self.camera.find_floor_coordinate(pos)
                    world_to_obstacle = Transformation(position=floor_coordinate_robot)
                    camera_to_obstacle = np.linalg.inv(self.camera.pose) @ world_to_obstacle
                    self.br2.sendTransform(
                        camera_to_obstacle.position,
                        camera_to_obstacle.quaternion,
                        msg.header.stamp,
                        "robot1" + "/Goal_pose",
                        "robot1" + "/camera",
                    )

        except ROSException as re:
            print(re)
            exit(0)


if __name__ == "__main__":
    src_path = expanduser("~") + "/catkin_ws/src/soccerbot/soccer_perception/"

    model_path = src_path + "soccer_object_detection/models/half_5.pt"

    parser = ArgumentParser()
    parser.add_argument("--model", dest="model_path", default=model_path, help="pytorch model")
    parser.add_argument("--num-feat", dest="num_feat", default=10, help="specify model size of the neural network")
    args, unknown = parser.parse_known_args()
    rospy.init_node("object_detector")
    my_node = ObjectDetectionNodeRos(args.model_path)

    try:
        rospy.spin()
    except ROSException as rx:
        exit(0)
