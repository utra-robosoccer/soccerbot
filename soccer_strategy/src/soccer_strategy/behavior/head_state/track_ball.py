import math
import os
import time

import cv2
import numpy as np
from soccer_object_detection.object_detect_node import Label, ObjectDetectionNode
from soccer_pycontrol.model.bez import BezStatusEnum

from soccer_strategy.behavior import Behavior

PLOT = True


class TrackBall(Behavior):
    def __init__(self):
        super().__init__()

    def action(self) -> None:
        self.bez.status = BezStatusEnum.WALK

    def run_algorithim(self) -> None:

        start = time.time()
        img = self.bez.sensors.get_camera_image()
        img = cv2.resize(img, dsize=(640, 480))

        dimg, bbs_msg = self.detect.get_model_output(img)
        for box in bbs_msg.bounding_boxes:
            if box.data == "0":  # class "0" == ball
                print("ball found")
                self.detect.camera.pose = self.bez.sensors.get_pose(link=2)
                boundingBoxes = [[box.xmin, box.ymin], [box.xmax, box.ymax]]

                tr = self.detect.camera.calculate_ball_from_bounding_boxes_cam_frame(boundingBoxes)
                ball_pos = np.array(tr.position)

                print(ball_pos)

                dx, dy, dz = ball_pos
                yaw = np.arctan2(dy, dx)
                pitch = -np.arctan2(dz, np.sqrt(dx**2 + dy**2))
                print("Yaw:", yaw, "Pitch:", pitch)

                self.bez.motor_control.configuration["head_yaw"] = yaw
                self.bez.motor_control.configuration["head_pitch"] = pitch
                self.bez.motor_control.set_motor()

        if "DISPLAY" in os.environ and PLOT:
            cv2.imshow("CVT Color2", dimg)
            cv2.waitKey(1)
        # end = time.time()

        # print(f"Perception time per step: {1000 * (end - start):.2f} ms")

    def ready_to_switch_to(self) -> bool:
        return True
