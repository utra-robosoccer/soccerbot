import os
import math
import time

from soccer_object_detection.object_detect_node import Label, ObjectDetectionNode

import numpy as np

import cv2
from soccer_pycontrol.model.bez import BezStatusEnum

from soccer_strategy.behavior import Behavior
from soccer_strategy.behavior.head_state.track_ball import TrackBall

PLOT = True

class FindBall(Behavior):
    def __init__(self):
        super().__init__()
        self._start_time = time.time()
        self._sway_frequency = 0.8
        self._sway_amplitude = 0.9
        # self._target_goal = target_goal

    def action(self) -> None:
        self.bez.status = BezStatusEnum.FIND_BALL

    def run_algorithim(self) -> None:
        # self.bez.motor_control.set_motor()
        print("FindBall")

        elapsed_time = time.time() - self._start_time
        head_yaw = math.sin(elapsed_time * self._sway_frequency) * self._sway_amplitude

        np_head_angles = np.array([head_yaw, 0.1])

        # self.bez.motor_control.set_head_target_angles(np_head_angles)

        self.bez.motor_control.configuration["head_yaw":"head_pitch"] = np_head_angles

        # print("FindBall sets head to:", np_head_angles)
        self.bez.motor_control.set_motor()

        img = self.bez.sensors.get_camera_image()
        img = cv2.resize(img, dsize=(640, 480))
        # detect.camera.pose.orientation_euler = [0, np.pi / 8, 0]
        dimg, bbs_msg = self.detect.get_model_output(img)
        for box in bbs_msg.bounding_boxes:
            if box.data == "0":
                self.detect.camera.pose = self.bez.sensors.get_pose(link=2)
                boundingBoxes = [[box.xmin, box.ymin], [box.xmax, box.ymax]]
                print(self.detect.camera.calculate_ball_from_bounding_boxes(boundingBoxes).position)
                self.context.transition_to(TrackBall())
                self.bez.status = BezStatusEnum.WALK

        # if "DISPLAY" in os.environ and PLOT:
        #     cv2.imshow("CVT Color2", dimg)
        #     cv2.waitKey(1)

    def ready_to_switch_to(self) -> bool:
        return True
