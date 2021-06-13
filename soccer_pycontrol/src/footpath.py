import numpy as np
import math
import enum
from path import Path
from transformation import Transformation as tr
import matplotlib.pyplot as plt


class PostPreSetting(enum.IntEnum):
    POST_AND_PRE = 0
    ONLY_POST_AND_PRE_ON_LAST_ONES = 1
    ONLY_POST = 2
    NO_POST_NOR_PRE = 3

# TODO: where is first_step_left????
class Footpath(Path):
    half_to_full_step_time_ratio = 0.7 # Duration difference between half and full step

    foot_separation = 0.044 # seperation between feet and center
    step_height = 0.065 # height of step
    step_outwardness = 0.015
    step_rotation = 0.05

    def __init__(self, start_transform, end_transform, foot_center_to_floor):
        super().__init__(start_transform, end_transform)
        self.foot_center_to_floor = foot_center_to_floor

    def half_step_time(self):
        return self.full_step_time() * self.half_to_full_step_time_ratio

    def num_steps(self):
        return self.bodyStepCount() + 1

    def full_step_time(self):
        total_step_time = self.duration()
        return total_step_time / (2 * self.half_to_full_step_time_ratio + (self.num_steps() - 2))

    def footHeightRatio(self, t, post_pre_settings=0):
        full_step_time = self.full_step_time()
        half_step_time = self.half_step_time()

        post_step_time = self.post_footstep_ratio * full_step_time
        pre_step_time = self.pre_footstep_ratio * full_step_time
        if post_pre_settings == PostPreSetting.ONLY_POST_AND_PRE_ON_LAST_ONES:
            if t < half_step_time:
                pre_step_time = 0
            elif t > (self.duration() - half_step_time):
                post_step_time = 0
            else:
                post_step_time = 0
                pre_step_time = 0
        elif post_pre_settings == PostPreSetting.ONLY_POST:
            pre_step_time = 0
            post_step_time = -post_step_time
        elif post_pre_settings == PostPreSetting.NO_POST_NOR_PRE:
            post_step_time = 0
            pre_step_time = 0

        last_foot_same = self.num_steps() % 2
        step_num = -1

        # First foot
        if t < half_step_time:
            if t < post_step_time:
                first_foot_step_ratio = 0
            elif t > (half_step_time - pre_step_time):
                first_foot_step_ratio = 1
            else:
                first_foot_step_ratio = (t - post_step_time) / (half_step_time - post_step_time - pre_step_time)
        elif last_foot_same and (t > self.duration() - half_step_time):
            adjusted_step_time = t - (self.duration() - half_step_time)
            if adjusted_step_time < post_step_time:
                first_foot_step_ratio = 0
            elif adjusted_step_time > (half_step_time - pre_step_time):
                first_foot_step_ratio = 1
            else:
                first_foot_step_ratio = (adjusted_step_time - post_step_time) / (
                        half_step_time - post_step_time - pre_step_time)
        else:
            adjusted_step_time = t - half_step_time

            # fix in matlab function rounds to nearest integer towards 0
            if(adjusted_step_time / full_step_time) >= 0:
                step_num = np.floor(adjusted_step_time / full_step_time) # fix function in matlab
            else:
                step_num = np.ceil(adjusted_step_time / full_step_time)  # fix function in matlab

            adjusted_step_time = adjusted_step_time - step_num * full_step_time
            if (step_num % 2) == 0:
                first_foot_step_ratio = 0
            else:
                if adjusted_step_time < post_step_time:
                    first_foot_step_ratio = 0
                elif adjusted_step_time > (full_step_time - pre_step_time):
                    first_foot_step_ratio = 1
                else:
                    first_foot_step_ratio = (adjusted_step_time - post_step_time) / (
                            full_step_time - post_step_time - pre_step_time)

        # Second foot
        if t < half_step_time:
            second_foot_step_ratio = 0
        elif (not last_foot_same) and (t > (self.duration() - half_step_time)):
            adjusted_step_time = t - (self.duration() - half_step_time)
            if adjusted_step_time < post_step_time:
                second_foot_step_ratio = 0
            elif adjusted_step_time > (half_step_time - pre_step_time):
                second_foot_step_ratio = 1
            else:
                second_foot_step_ratio = (adjusted_step_time - post_step_time) / (
                        half_step_time - post_step_time - pre_step_time)
        else:
            adjusted_step_time = t - half_step_time

            # fix in matlab function rounds to nearest integer towards 0
            if(adjusted_step_time / full_step_time) >=0:
                step_num = int(np.floor(adjusted_step_time / full_step_time)) # fix function in matlab
            else:
                step_num = int(np.ceil(adjusted_step_time / full_step_time))  # fix function in matlab

            adjusted_step_time = adjusted_step_time - step_num * full_step_time
            if (step_num % 2) == 1:
                second_foot_step_ratio = 0
            else:
                if adjusted_step_time < post_step_time:
                    second_foot_step_ratio = 0
                elif adjusted_step_time > (full_step_time - pre_step_time):
                    second_foot_step_ratio = 1
                else:
                    second_foot_step_ratio = (adjusted_step_time - post_step_time) / (
                            full_step_time - post_step_time - pre_step_time)

        # Which foot is first?
        assert (first_foot_step_ratio <= 1)
        assert (second_foot_step_ratio <= 1)

        if self.first_step_left:
            right_foot_step_ratio = first_foot_step_ratio
            left_foot_step_ratio = second_foot_step_ratio
        else:
            right_foot_step_ratio = second_foot_step_ratio
            left_foot_step_ratio = first_foot_step_ratio

        step_num = step_num + 1

        return [step_num, right_foot_step_ratio, left_foot_step_ratio]

    def right_foot_position_at_step(self, n):
        bodystep = self.getBodyStep(n)

        bodypos = bodystep.get_position()
        transformToLeftFoot = tr([0, -self.foot_separation, -bodypos[2] + self.foot_center_to_floor])
        return np.matmul(bodystep, transformToLeftFoot)

    def left_foot_position_at_step(self, n):
        bodystep = self.getBodyStep(n)

        bodypos = bodystep.get_position()
        transformToRightFoot = tr([0, self.foot_separation, -bodypos[2] + self.foot_center_to_floor])
        return np.matmul(bodystep, transformToRightFoot)

    def whatIsTheFootDoing(self, step_num):
        if step_num == 0:
            if self.first_step_left:
                right_foot_action = [0, 1]  # Go from body position 0 to 1
                left_foot_action = [0]  # Stay put at position 0
            else:
                right_foot_action = [0]
                left_foot_action = [0, 1]

        elif step_num == (self.num_steps() - 1):
            if self.first_step_left ^ ((self.num_steps() % 2) == 0): # xor
                right_foot_action = [self.num_steps() - 2, self.num_steps() - 1]
                left_foot_action = [self.num_steps() - 1]
            else:
                left_foot_action = [self.num_steps() - 2, self.num_steps() - 1]
                right_foot_action = [self.num_steps() - 1]

        else:
            if self.first_step_left:
                if (step_num % 2) == 0:  # Left foot moving
                    left_foot_action = [step_num]
                    right_foot_action = [step_num - 1, step_num + 1]
                else:
                    left_foot_action = [step_num - 1, step_num + 1]
                    right_foot_action = [step_num]
            else:
                if (step_num % 2) == 0:  # Left foot moving
                    right_foot_action = [step_num]
                    left_foot_action = [step_num - 1, step_num + 1]
                else:
                    right_foot_action = [step_num - 1, step_num + 1]
                    left_foot_action = [step_num]

        return [right_foot_action, left_foot_action]

    def footPosition(self, t):

        [step_num, right_foot_step_ratio, left_foot_step_ratio] = self.footHeightRatio(t)
        [right_foot_action, left_foot_action] = self.whatIsTheFootDoing(step_num)
        if right_foot_step_ratio != 0 and right_foot_step_ratio != 1:
            assert (len(right_foot_action) == 2)
        if left_foot_step_ratio != 0 and left_foot_step_ratio != 1:
            assert (len(left_foot_action) == 2)
        # assert ((len(right_foot_action) == 2) == (right_foot_step_ratio != 0 and right_foot_step_ratio != 1))


        # Left foot
        if len(right_foot_action) == 1:
            right_foot_position = self.right_foot_position_at_step(right_foot_action[0])
        else:
            _from = self.right_foot_position_at_step(right_foot_action[0])
            _to = self.right_foot_position_at_step(right_foot_action[1])

            right_foot_position = self.parabolicPath(_from, _to, self.step_height, -self.step_outwardness, -self.step_rotation, right_foot_step_ratio)


        # Right foot
        if len(left_foot_action) == 1:
            left_foot_position = self.left_foot_position_at_step(left_foot_action[0])
        else:
            _from = self.left_foot_position_at_step(left_foot_action[0])
            _to = self.left_foot_position_at_step(left_foot_action[1])

            left_foot_position = self.parabolicPath(_from, _to, self.step_height, self.step_outwardness, self.step_rotation, left_foot_step_ratio)

        return [right_foot_position, left_foot_position]

    def parabolicPath(self, startTransform, endTransform, zdiff, sidediff, rotdiff, ratio):
        """
        http://mathworld.wolfram.com/ParabolicSegment.html
        """

        step_time = self.bodyStepTime()
        distance_between_step = tr.get_distance(startTransform, endTransform)
        assert (distance_between_step != 0.0)
        height_per_step = np.linalg.norm([zdiff, sidediff])

        h = height_per_step
        a = distance_between_step / 2

        # Using Newton Approximation Method
        # https://math.stackexchange.com/questions/3129154/divide-a-parabola-in-segments-of-equal-length
        L = distance_between_step
        aa = 4 * h / L

        f = lambda x: x * np.sqrt(1 + (x ** 2)) + np.arcsinh(x)  # f = @(x) x * sqrt(1+x^2) + asinh(x);
        s = ratio
        J = lambda X: 2 * np.sqrt(1 + (X ** 2))  # J = @(X) 2 * sqrt(1+X^2);
        r = lambda X: f(X) - (1 - (2 * s)) * f(aa)  # r = @(X) f(X) - (1-2*s)*f(aa);

        X = 0
        while np.abs(r(X)) > 0.0001:
            X = X - r(X) / J(X)

        if aa == 0:
            dist = ratio * L
        else:
            dist = 0.5 * (1 - X / aa) * L

        # Calculate intermediate transform
        position_time = dist / distance_between_step * step_time
        if position_time < 0:
            position_time = 0

        ratio = position_time / step_time
        if ratio < 0:
            ratio = 0
        elif ratio > 1:
            ratio = 1

        # Interpolate between the two H-transforms
        t1 = tr.transformation_weighted_average(startTransform, endTransform, ratio)

        x = (-a) + dist
        y = h * (1 - (x ** 2) / (a ** 2))

        zdelta = np.cos(np.arctan2(sidediff, zdiff)) * y
        ydelta = np.sin(np.arctan2(sidediff, zdiff)) * y
        if rotdiff != 0:
            thetadelta = y / height_per_step * rotdiff
        else:
            thetadelta = 0

        t2 = tr(position=[0, ydelta, zdelta], quaternion=tr.get_quaternion_from_axis_angle(vector = [1, 0, 0], angle = thetadelta))
        position = np.matmul(t1, t2)
        return position

    def show(self, fig=None):
        """
        Draws the feet positions
        :param fig: Shared figure object
        :return: None
        """
        i = 0
        # for t = 0:obj.step_size:obj.duration
        # TODO: make a generator?
        iterator = np.linspace(0, self.duration(), num=math.ceil(self.duration() / self.step_size)+1)
        tfInterp_l = np.zeros((4, 4, len(iterator)))
        tfInterp_r = np.zeros((4, 4, len(iterator)))
        for t in iterator:
            [lfp, rfp] = self.footPosition(t)
            tfInterp_l[:,:,i] = lfp
            tfInterp_r[:,:,i] = rfp
            i = i + 1
        self.show_tf(fig, tfInterp_l, len(iterator))
        self.show_tf(fig, tfInterp_r, len(iterator))

    @staticmethod
    def show_tf(fig=None, tf_array=None, length=0):
        """
        Helper function to draw the H-transforms equivalent to plotTransforms function in Matlab
        :param fig: Shared figure object
        :param tf_array: Array of transforms of size (4,4,n)
        :param length: The 3rd dimension of the array, n
        :return: None
        """
        if fig is None:
            fig = plt.figure()
        ax = fig.gca(projection='3d')
        tfInterp_axis = np.zeros((3, length))
        axes = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        colors = ['r', 'g', 'b']
        ax = fig.gca(projection='3d')
        for j in range(len(axes)):
            for i in range(np.size(tf_array, 2)):
                tfInterp_axis[:, i] = np.matmul(tf_array[0:3, 0:3, i], axes[j]).ravel()
            tfInterp_axis = tfInterp_axis * 0.01
            ax.quiver(tf_array[0, 3, :], tf_array[1, 3, :], tf_array[2, 3, :], tfInterp_axis[0, :],
                      tfInterp_axis[1, :], tfInterp_axis[2, :], color=colors[j], normalize=True, length=0.01,
                      arrow_length_ratio=.2)
        fig.canvas.draw()
        plt.show(block=False)