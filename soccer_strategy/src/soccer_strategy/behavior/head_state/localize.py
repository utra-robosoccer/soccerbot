import os
import math
import time

from sympy.codegen.ast import continue_

from soccer_object_detection.object_detect_node import Label, ObjectDetectionNode

import numpy as np
import cv2
from soccer_pycontrol.model.bez import BezStatusEnum
from soccer_common import Transformation

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
        # self.head_scanner = HeadScanner(max_yaw=self._sway_amplitude, min_yaw=-self._sway_amplitude, step_size=0.4)
        self._rotating = False
        self._rotate_goal = None

    def action(self) -> None:
        self.bez.status = BezStatusEnum.LOCALIZE

    def run_algorithim(self) -> None:

        # pose = self.bez.sensors.get_pose()
        # pos = pose.position
        # target_goal = Transformation(pos, euler=[2.2, 0, 0])
        # self.nav.walk(target_goal)
        # finished = self.rotate_in_place(20)
        #
        # if finished:
        #     nets = []
        #     # Initial Entrance into the game

        if not self.bez.found_home_side:

            # Find nets
            self._find_nets()
            nets = self.bez.head_scanner.goalpost_distances
            print("Net Position: ", nets[0])

            # if the distance of current x1 is bigger than x2, switch
            if nets[0][0] > nets[1][0]:
                temp = nets[0]
                nets[0] = nets[1]
                nets[1] = temp

            # Since no homeside has been set yet, we set the homeside
            self.bez.found_home_side = True
            self.bez.home_side_yaw_positive = (nets[0][1] > 0)

            # using the net positions determine x, y
            x1 = nets[0][0]
            x2 = nets[1][0]
            Length_of_field = 9
            theta1 = math.acos((x1**2 + x2**2 - Length_of_field**2)/2*x1*x2)
            theta2 = (180 - theta1)/2
            y_pos = (x1 * math.sin(theta2))
            x_pos = Length_of_field / 2 - math.sqrt(x1 ** 2 + y_pos ** 2)

            self.bez.location = [x_pos, y_pos]




    def _find_nets(self):
        """
        Detect goalposts and returns their positions for .
        This is a helper function only used within this class.
        """

        img = self.bez.sensors.get_camera_image()
        img = cv2.resize(img, (640, 480))
        dimg, bbs_msg = self.detect.get_model_output(img)

        net_positions = []

        # problem with this is it freezes if there is no bounding boxes detected (like in simulation when it looks away from ball)
        for box in bbs_msg.bounding_boxes:

            # if a Net is detected
            if box.data == "1":  # GOALPOST id = 1, was just testing with ball
                yaw = self.bez.head_scanner.update(True)
                np_head_angles = np.array([yaw, 0.1])
                self.bez.motor_control.configuration["head_yaw":"head_pitch"] = np_head_angles
                print("Localize sets head to:", np_head_angles)
                self.bez.motor_control.set_motor()
                # self.bez.motor_control.set_single_motor("head_yaw", yaw)
                self.detect.camera.pose = self.bez.sensors.get_pose(link=2)
                print("Head Yaw: ", self.bez.sensors.get_pose(link=2).orientation_euler[0])

                bounding_box = [[box.xmin, box.ymin], [box.xmax, box.ymax]]
                position = self.detect.camera.calculate_ball_from_bounding_boxes_cam_frame(bounding_box).position

                # Calculate distance to net
                # TODO: need some sort of find_goal_distance() function
                distance_to_net = 0

                # Save total Yaw
                yaw_total = self.bez.sensors.get_pose(link=2).orientation_euler[0] + self.bez.sensors.get_pose().orientation_euler[0]

                # Save the distance to this net and the yaw in head_scanner object
                self.bez.head_scanner.goalpost_distances.append([distance_to_net, yaw_total])

            # net not detected
            else:
                yaw = self.bez.head_scanner.update(False)
                np_head_angles = np.array([yaw, 0.1])
                self.bez.motor_control.configuration["head_yaw":"head_pitch"] = np_head_angles
                print("Localize sets head to:", np_head_angles)
                self.bez.motor_control.set_motor()
                # self.bez.motor_control.set_single_motor("head_yaw", yaw)

                # Rotate body once max or min head yaw is reached by 45 degrees
                from soccer_strategy.behavior.state.rotate import Rotate

                if self.bez.head_scanner.max_yaw_reached:
                    self.context.transition_to(Rotate(False), np.pi / 4)
                elif self.bez.head_scanner.min_yaw_reached:
                    self.context.transition_to(Rotate(True), np.pi / 4)

        # Once both nets are found
        if self.bez.head_scanner.state == "DONE":
            return True

        return False

    def rotate_in_place(self, yaw_delta: float):
        """
        Starts or continues rotating the robot in place by yaw_delta radians.
        Should be called from inside run_algorithim().
        """
        if not self._rotating:
            current_pose = self.bez.sensors.get_pose()
            new_yaw = current_pose.orientation_euler[0] + yaw_delta
            self._rotate_goal = Transformation(position=current_pose.position, euler=[new_yaw, 0, 0])
            self.nav.reset_walk()
            self._rotating = True

        self.nav.walk(self._rotate_goal)

        if not self.nav.enable_walking:
            self._rotating = False
            self.nav.wait(200)  # Optional pause after rotation
            print("Finished rotating in place.")
            return True  # Rotation is complete

        return False  # Still rotating

    def ready_to_switch_to(self) -> bool:
        return True

