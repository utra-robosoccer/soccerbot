#!/usr/bin/env python3

import os
from os.path import expanduser

import rclpy

# import tf
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

# from self.impl.tcpros_base import DEFAULT_BUFF_SIZE
from soccer_object_detection.camera.camera_calculations_ros import CameraCalculationsRos
from soccer_object_detection.object_detect_node import ObjectDetectionNode, bcolors
from std_msgs.msg import Float32MultiArray

if "ROS_NAMESPACE" not in os.environ:
    os.environ["ROS_NAMESPACE"] = "/robot1"

from argparse import ArgumentParser

import torch
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data

# from rospy import ROSException
from sensor_msgs.msg import Image

from soccer_msgs.msg import BoundingBoxes

# TODO should be somewhere else


class ObjectDetectionNodeRos(ObjectDetectionNode, Node):
    """
    Ros bridge
    input: 480x640x4 bgra8 -> output: 3x200x150
    """

    def __init__(self, model_path):
        Node.__init__(self, "object_detector")
        ObjectDetectionNode.__init__(self, model_path)
        self.declare_parameter("cover_horizon_up_threshold", 30)
        self.cover_horizon_up_threshold = self.get_parameter("cover_horizon_up_threshold").value
        self.declare_parameter("ball_confidence_threshold", 0.75)
        self.CONFIDENCE_THRESHOLD = self.get_parameter("ball_confidence_threshold").value

        if torch.cuda.is_available():
            self.get_logger().info(f"{bcolors.OKGREEN}Using CUDA for object detection{bcolors.ENDC}")

        self.robot_name = self.get_namespace().strip("/")  # remove '/'
        self.camera = CameraCalculationsRos(self, self.robot_name)
        self.camera.reset_position()

        # Params
        self.br = CvBridge()
        # self.br2 = tf.TransformBroadcaster()

        # ROS
        self.pub_ball = self.create_publisher(PoseStamped, f"{self.robot_name}/ball", 10)
        self.pub_ball_pixel = self.create_publisher(Float32MultiArray, f"{self.robot_name}/ball_pixel", 10)
        self.pub_detection = self.create_publisher(Image, f"{self.robot_name}/detection_image", 10)
        self.pub_boundingbox = self.create_publisher(BoundingBoxes, f"{self.robot_name}/object_bounding_boxes", 10)

        self.image_create_subscription = self.create_subscription(
            Image, "/camera/image_raw", self.callback, qos_profile_sensor_data
        )  # Large buff size (https://answers.ros.org/question/220502/image-create_subscription-lag-despite-queue-1/)

    #     self.game_state_create_subscription = self.create_subscription("gamestate", GameState, self.game_state_callback)
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
        # self.get_logger().info("Object Detection Receiving image")
        # width x height x channels (bgra8)
        image = self.br.imgmsg_to_cv2(msg)
        self.camera.reset_position(timestamp=msg.header.stamp)  # msg->image

        detection_image, bbs_msg = self.get_model_output(image)

        bbs_msg.header = msg.header

        if len(bbs_msg.bounding_boxes) > 0:
            self.pub_boundingbox.publish(bbs_msg)

        self.pub_detection.publish(self.br.cv2_to_imgmsg(detection_image, encoding="bgr8"))

        for box in bbs_msg.bounding_boxes:
            if box.data == "0":
                boundingBoxes = [[box.xmin, box.ymin], [box.xmax, box.ymax]]

                ball_pixel = Float32MultiArray()
                ball_pixel.data = [(box.xmin + box.xmax) / 2.0, (box.ymin + box.ymax) / 2.0]
                self.pub_ball_pixel.publish(ball_pixel)
                # print(detect.camera.calculate_ball_from_bounding_boxes(boundingBoxes).position)
                ball_pos = self.camera.calculate_ball_from_bounding_boxes(boundingBoxes)
                # print(
                #     f"floor pos: {ball_pos.position} "
                # )
                pose_msg = PoseStamped()
                pose_msg.header.stamp = msg.header.stamp
                pose_msg.header.frame_id = "base_link"  # msg.header.frame_id
                pose_msg.pose = ball_pos.pose
                # pose_msg.pose.position.z = 0.04
                # pose_msg.pose.position.y = -pose_msg.pose.position.y
                self.pub_ball.publish(pose_msg)

            # TODO
            # for box in bbs_msg.bounding_boxes:
            #     if box.Class == "0":
            #         boundingBoxes = [[box.xmin, box.ymin], [box.xmax, box.ymax]]
            #         ball_pose = self.camera.calculate_ball_from_bounding_boxes(boundingBoxes)
            #         self.br2.sendTransform(
            #             ball_pose.position,
            #             ball_pose.quaternion,
            #             msg.header.stamp,
            #             "robot1" + "/ball",
            #             "robot1" + "/camera",
            #         )
            #         pos = [box.xbase, box.ybase]
            #         floor_coordinate_robot = self.camera.find_floor_coordinate(pos)
            #         world_to_obstacle = Transformation(position=floor_coordinate_robot)
            #         camera_to_obstacle = np.linalg.inv(self.camera.pose) @ world_to_obstacle
            #         self.br2.sendTransform(
            #             camera_to_obstacle.position,
            #             camera_to_obstacle.quaternion,
            #             msg.header.stamp,
            #             "robot1" + "/ball_2",
            #             "robot1" + "/camera",
            #         )
            #
            #     elif box.Class == "1":
            #         pos = [box.xbase, box.ybase]
            #
            #         floor_coordinate_robot = self.camera.find_floor_coordinate(pos)
            #         world_to_obstacle = Transformation(position=floor_coordinate_robot)
            #         camera_to_obstacle = np.linalg.inv(self.camera.pose) @ world_to_obstacle
            #         self.br2.sendTransform(
            #             camera_to_obstacle.position,
            #             camera_to_obstacle.quaternion,
            #             msg.header.stamp,
            #             "robot1" + "/Goal_pose",
            #             "robot1" + "/camera",
            #         )


def main(args=None):
    rclpy.init(args=args)

    model_path_default = expanduser("~") + "/ros2_ws/src/soccerbot/soccer_perception/soccer_object_detection/models/half_5.pt"

    parser = ArgumentParser()
    parser.add_argument("--model", dest="model_path", default=model_path_default)
    args, _ = parser.parse_known_args()

    node = ObjectDetectionNodeRos(args.model_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
