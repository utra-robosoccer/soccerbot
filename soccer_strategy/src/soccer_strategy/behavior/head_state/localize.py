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


class HeadScanner:
    def __init__(self, max_yaw=np.pi / 2, min_yaw=-np.pi / 2, step_size=0.01):
        self.max_yaw = max_yaw
        self.min_yaw = min_yaw
        self.step_size = step_size
        self.yaw = 0.0  # current angle
        self.state = "LOOKING_RIGHT"
        self.goalposts_found = 0
        self.max_yaw_reached = False
        self.min_yaw_reached = False


    def update(self, sees_goal: bool) -> float:
        """
        Update head angle based on goalpost detection.
        :param sees_goal: whether goalpost is visible in current frame
        :return: new head yaw angle
        """

        if sees_goal:
            self.goalposts_found += 1
            if self.goalposts_found == 1:
                self.state = "LOOKING_LEFT"
            elif self.goalposts_found == 2:
                self.state = "DONE"

        if self.state == "LOOKING_RIGHT":
            self.yaw = min(self.yaw + self.step_size, self.max_yaw)
            self.max_yaw_reached = (self.yaw + self.step_size >= self.max_yaw)
        elif self.state == "LOOKING_LEFT":
            self.yaw = max(self.yaw - self.step_size, self.min_yaw)
            self.min_yaw_reached = (self.yaw - self.step_size <= self.min_yaw)
        elif self.state == "DONE":
            pass  # Stop moving

        return self.yaw

    def reset(self):
        self.yaw = 0.0
        self.state = "LOOKING_RIGHT"
        self.goalposts_found = 0

class Localize(Behavior):
    def __init__(self):
        super().__init__()
        self._start_time = time.time()
        # make head move left to right
        self._sway_frequency = 0.8
        self._sway_amplitude = 2 #0.9
        self.head_scanner = HeadScanner(max_yaw=self._sway_amplitude, min_yaw=-self._sway_amplitude, step_size=0.4)
        self._rotating = False
        self._rotate_goal = None

    def action(self) -> None:
        self.bez.status = BezStatusEnum.LOCALIZE

    def run_algorithim(self) -> None:

        # pose = self.bez.sensors.get_pose()
        # pos = pose.position
        # target_goal = Transformation(pos, euler=[2.2, 0, 0])
        # self.nav.walk(target_goal)
        finished = self.rotate_in_place(2)

        if finished:
            nets = []
            # Initial Entrance into the game
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
        # elapsed_time = time.time() - self._start_time
        # head_yaw = math.sin(elapsed_time * self._sway_frequency) * self._sway_amplitude
        # np_head_angles = np.array([head_yaw, 0.1])
        # self.bez.motor_control.configuration["head_yaw":"head_pitch"] = np_head_angles
        # print("Localize sets head to:", np_head_angles)
        # self.bez.motor_control.set_motor()

        img = self.bez.sensors.get_camera_image()
        img = cv2.resize(img, (640, 480))
        dimg, bbs_msg = self.detect.get_model_output(img)

        net_positions = []
        # problem with this is it freezes if there is no boundingboxes detected (like in simulation when it looks away from ball)
        for box in bbs_msg.bounding_boxes:
            # ToDO: after finding 1 net, rotate head and body in a way that faces away from the current post
            if box.data == "1":  # GOALPOST id = 1, was just testing with ball
                yaw = self.head_scanner.update(True)
                np_head_angles = np.array([yaw, 0.1])
                self.bez.motor_control.configuration["head_yaw":"head_pitch"] = np_head_angles
                print("Localize sets head to:", np_head_angles)
                self.bez.motor_control.set_motor()
                # self.bez.motor_control.set_single_motor("head_yaw", yaw)
                self.detect.camera.pose = self.bez.sensors.get_pose(link=2)
                print("Head Yaw: ", self.bez.sensors.get_pose(link=2).orientation_euler[0])
                bounding_box = [[box.xmin, box.ymin], [box.xmax, box.ymax]]
                position = self.detect.camera.calculate_ball_from_bounding_boxes_cam_frame(bounding_box).position
                net_positions.append(position)
            else:
                yaw = self.head_scanner.update(False)
                np_head_angles = np.array([yaw, 0.1])
                self.bez.motor_control.configuration["head_yaw":"head_pitch"] = np_head_angles
                print("Localize sets head to:", np_head_angles)
                self.bez.motor_control.set_motor()
                # self.bez.motor_control.set_single_motor("head_yaw", yaw)

                # Rotate body once max or min head yaw is reached
                pose = self.bez.sensors.get_pose()
                pos = pose.position
                target_goal = Transformation(pos, euler=[yaw, 0, 0])
                if self.head_scanner.max_yaw_reached:
                    yaw += self.head_scanner.step_size
                    self.nav.walk(target_goal)

                elif self.head_scanner.min_yaw_reached:
                    yaw -= self.head_scanner.step_size
                    self.nav.walk(target_goal)



        return True

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

