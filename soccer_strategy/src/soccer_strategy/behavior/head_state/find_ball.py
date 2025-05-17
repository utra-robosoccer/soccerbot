import os

import cv2
from soccer_pycontrol.model.bez import BezStatusEnum

from soccer_strategy.behavior import Behavior

PLOT = True


class FindBall(Behavior):
    def action(self) -> None:
        self.bez.status = BezStatusEnum.FIND_BALL

    def run_algorithim(self) -> None:
        img = self.bez.sensors.get_camera_image()
        img = cv2.resize(img, dsize=(640, 480))
        # detect.camera.pose.orientation_euler = [0, np.pi / 8, 0]
        dimg, bbs_msg = self.detect.get_model_output(img)
        for box in bbs_msg.bounding_boxes:
            if box.Class == "0":
                self.detect.camera.pose = self.bez.sensors.get_pose(link=2)
                boundingBoxes = [[box.xmin, box.ymin], [box.xmax, box.ymax]]
                print(self.detect.camera.calculate_ball_from_bounding_boxes(boundingBoxes).position)

        if "DISPLAY" in os.environ and PLOT:
            cv2.imshow("CVT Color2", dimg)
            cv2.waitKey(1)

    def ready_to_switch_to(self) -> bool:
        return True
