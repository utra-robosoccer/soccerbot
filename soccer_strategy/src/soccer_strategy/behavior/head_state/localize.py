import os
import math
import time

from soccer_object_detection.object_detect_node import Label, ObjectDetectionNode

import numpy as np
import cv2
from soccer_pycontrol.model.bez import BezStatusEnum

from soccer_strategy.behavior import Behavior

from soccer_perception.soccer_object_detection.src.soccer_object_detection.camera import camera_calculations

PLOT = True


class Localize(Behavior):
    def __init__(self):
        super().__init__()
        self._start_time = time.time()
        # make head move left to right
        self._sway_frequency = 0.8
        self._sway_amplitude = 2 #0.9

    def action(self) -> None:
        self.bez.status = BezStatusEnum.LOCALIZE

    def run_algorithim(self) -> None:

        nets = []
        if not self.bez.found_home_side:
            # find net
            # TODO: need some sort of find_goal_distance() function
            nets = self._find_nets()
            # print("Net Position: ", nets[0])

            # using the net positions determine x, y
            # x1 = find_goal_distance(net1)
            # x2 = find_goal_distance(net2)
            # Length_of_field = 9
            # theta1 = math.acos((x1**2 + x2**2 - Length_of_field**2)/2*x1*x2)
            # theta2 = (180 - theta1)/2
            # y_pos = (x1 * math.sin(theta2))
            # x_pos = Length_of_field / 2 - math.sqrt(x1 ** 2 + y_pos ** 2)
            # set home_side with compass


    def _find_nets(self):
        """
        Detect goalposts and returns their positions for .
        This is a helper function only used within this class.
        """
        # move head side to side
        elapsed_time = time.time() - self._start_time
        head_yaw = math.sin(elapsed_time * self._sway_frequency) * self._sway_amplitude
        np_head_angles = np.array([head_yaw, 0.1])
        self.bez.motor_control.configuration["head_yaw":"head_pitch"] = np_head_angles
        print("Localize sets head to:", np_head_angles)
        self.bez.motor_control.set_motor()

        img = self.bez.sensors.get_camera_image()
        img = cv2.resize(img, (640, 480))
        dimg, bbs_msg = self.detect.get_model_output(img)

        net_positions = []
        for box in bbs_msg.bounding_boxes:
            # ToDO: after finding 1 net, rotate head and body in a way that faces away from the current post
            if box.data == "0":  # GOALPOST id = 1, was just testing with ball
                self.detect.camera.pose = self.bez.sensors.get_pose(link=2)
                print("Head Yaw: ", self.bez.sensors.get_pose(link=2).orientation_euler[0])
                bounding_box = [[box.xmin, box.ymin], [box.xmax, box.ymax]]
                position = self.detect.camera.calculate_ball_from_bounding_boxes_cam_frame(bounding_box).position
                net_positions.append(position)

        return True

    def ready_to_switch_to(self) -> bool:
        return True

