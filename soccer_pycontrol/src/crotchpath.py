import functools

import numpy as np
from footpath import Footpath
from transformation import Transformation as tr
import math

class Crotchpath(Footpath):

    crotch_zdiff_sway = 0.000
    crotch_sidediff_sway = -0.03
    crotch_sidediff_sway_decay = 5
    crotch_thetadiff_sway = [0., 0., 0.] # [0, 0, 0.08]

    first_step_left = 0

    def __init__(self, start_transform, end_transform, foot_center_to_floor):
        super().__init__(start_transform, end_transform, foot_center_to_floor)

        # Calculate the foot for the first step (based on destination)
        axang_angle, axang_vector = tr.get_axis_angle_from_quaternion(self.start_transform.get_orientation())
        theta1 = axang_angle
        diff_transform = np.matmul(end_transform, np.linalg.inv(start_transform))
        theta2 = np.arctan2(diff_transform[1, 3], diff_transform[0, 3])
        if (theta2 - theta1) % (2 * np.pi) > np.pi:
            self.first_step_left = 0
        else:
            self.first_step_left = 1

    @functools.lru_cache
    def crotchPosition(self, t):
        [step_num, right_foot_ratio, left_foot_ratio] = self.footHeightRatio(t, 1)
        [right_foot_action, _] = self.whatIsTheFootDoing(step_num)
        if len(right_foot_action) == 2:
            ratio = right_foot_ratio
        else:
            ratio = left_foot_ratio

        # Base position for the torso
        if step_num == 0:
            _from = self.getBodyStep(0)
            _to = self.getBodyStep(1)
            body_movement_ratio = ratio / 2
        elif step_num == self.num_steps() - 1:
            _from = self.getBodyStep(step_num - 1)
            _to = self.getBodyStep(step_num)
            body_movement_ratio = (ratio / 2) + (1 / 2)
        else:
            if (ratio < 0.5):
                _from = self.getBodyStep(step_num - 1)
                _to = self.getBodyStep(step_num)
                body_movement_ratio = ratio + 0.5
            else:
                _from = self.getBodyStep(step_num)
                _to = self.getBodyStep(step_num + 1)
                body_movement_ratio = ratio - 0.5

        position = self.parabolicPath(_from, _to, 0.0, 0.0, 0.0, body_movement_ratio)

        # Vertical Sway (sinusoidal wave)
        [_, right_foot_ratio, _] = self.footHeightRatio(t, 3)
        if len(right_foot_action) == 2:  # Left foot moving, lean right
            ratio = right_foot_ratio
        else:
            ratio = left_foot_ratio

        if t < self.half_step_time():
            zdiff = self.crotch_zdiff_sway * (1 - np.cos(ratio * np.pi))
        elif t > self.duration() - self.half_step_time():
            zdiff = self.crotch_zdiff_sway * (1 - np.cos((ratio * np.pi) + np.pi))
        else:
            zdiff = self.crotch_zdiff_sway * (1 - np.cos((ratio * 2 * np.pi) + np.pi))

        # Horizontal Sway (exponential decay)
        [_, right_foot_ratio, left_foot_ratio] = self.footHeightRatio(t, 3)
        if len(right_foot_action) == 2:
            ratio = right_foot_ratio
            is_right_foot = -1
        else:
            ratio = left_foot_ratio
            is_right_foot = 1

        r = -4 * (ratio ** 2) + 4 * ratio
        ydiff = r * self.crotch_sidediff_sway * is_right_foot
        thetadiff = ydiff / self.crotch_sidediff_sway * np.array(self.crotch_thetadiff_sway)

        H = tr.get_transform_from_euler(thetadiff) # H = eul2tform(thetadiff)
        H.set_position([-0.005, ydiff, zdiff])

        position = np.matmul(position, H)

        return position

    def show(self, fig=None):
        # Draw the crotch position
        i = 0
        iterator = np.linspace(0, self.duration(), num=math.ceil(self.duration() / self.step_size) + 1)
        tfInterp = np.zeros((4, 4, len(iterator)))
        for t in iterator:
            tfInterp[:, :, i] = self.crotchPosition(t)
            i = i + 1
        self.show_tf(fig, tfInterp, len(iterator))

