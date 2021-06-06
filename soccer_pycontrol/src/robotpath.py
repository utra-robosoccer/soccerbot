from crotchpath import Crotchpath
from path import Path
from footpath import Footpath
import matplotlib as plt
import matplotlib.pyplot as plt
import numpy as np
import math

class Robotpath(Crotchpath):
    def __init__(self, start_transform, end_transform, foot_center_to_floor):
        super().__init__(start_transform, end_transform, foot_center_to_floor)

    def show(self):
        fig = plt.figure()
        Path.show(self, fig)
        Footpath.show(self, fig)
        Crotchpath.show(self, fig)
        plt.show() # to interact with the path graph uncomment this

    def showTimingDiagram(self):
        times = np.linspace(0, self.duration(), num=math.ceil(self.duration() / self.step_size) + 1)
        i = 0
        step_num = np.zeros(len(times))
        right_foot_step_ratio = np.zeros(len(times))
        left_foot_step_ratio = np.zeros(len(times))
        right_foot_body_pose = np.zeros(len(times))
        left_foot_body_pose = np.zeros(len(times))

        for t in times:
            [step_num[i], right_foot_step_ratio[i], left_foot_step_ratio[i]] = self.footHeightRatio(t)
            [right_foot_action, left_foot_action] = self.whatIsTheFootDoing(step_num[i])
            if len(right_foot_action) == 1:
                right_foot_body_pose[i] = right_foot_action[0]
            else:
                right_foot_body_pose[i] = (right_foot_action[1] - right_foot_action[0]) * right_foot_step_ratio[i] + right_foot_action[0]

            if len(left_foot_action) == 1:
                left_foot_body_pose[i] = left_foot_action[0]
            else:
                left_foot_body_pose[i] = (left_foot_action[1] - left_foot_action[0]) * left_foot_step_ratio[i] + left_foot_action[0]

            i = i + 1

        fig = plt.figure(2, tight_layout=True)
        # Foot Step ratio
        plt.subplot(321)
        plt.plot(times, right_foot_step_ratio, label='Left')
        plt.plot(times, left_foot_step_ratio, label='Right')
        plt.title('Foot step Ratio')
        plt.xlabel('time (t)')
        plt.ylabel('Ratio')
        plt.legend()
        plt.grid(b=True, which='both', axis='both')

        # Foot step
        plt.subplot(323)
        plt.plot(times, step_num, label='Step Num')
        plt.plot(times, right_foot_body_pose, label='Right')
        plt.plot(times, left_foot_body_pose, label='Left')
        plt.title('Foot body pose')
        plt.xlabel('time (t)')
        plt.ylabel('Body Pose')
        plt.legend()
        plt.grid(b=True, which='both', axis='both')

        # Music????
        times = np.linspace(0, self.duration(), num=math.ceil(self.duration() / self.step_size) + 1)
        lfp = np.zeros((4,4,len(times)))
        rfp = np.zeros((4, 4, len(times)))
        crp = np.zeros((4,4,len(times)))
        diff_right_foot = np.zeros((4,4,len(times)))
        diff_left_foot = np.zeros((4,4,len(times)))
        i = 0
        for t in times:
            [lfp[:,:, i], rfp[:,:, i]] = self.footPosition(t)
            crp[:,:, i] = self.crotchPosition(t)
            diff_right_foot[:,:, i] = np.matmul(lfp[:,:, i], np.linalg.inv(crp[:,:, i])) # lfp[:,:, i] / crp[:,:, i]
            diff_left_foot[:,:, i] = np.matmul(rfp[:,:, i], np.linalg.inv(crp[:,:, i])) # rfp[:,:, i] / crp[:,:, i]
            i = i + 1

        plt.subplot(322)
        plt.plot(times, lfp[0, 3, :].ravel(), label='Left')
        plt.plot(times, rfp[0, 3, :].ravel(), label='Right')
        plt.plot(times, crp[0, 3, :].ravel(), label='Crotch')
        plt.title('X position of left, right and body')
        plt.xlabel('time (t)')
        plt.ylabel('Torso to feet (x)')
        plt.legend()
        plt.grid(b=True, which='both', axis='both')

        plt.subplot(325)
        plt.plot(times, np.linalg.norm(diff_right_foot[0: 3, 3, :], axis=0), label='Left')
        plt.plot(times, np.linalg.norm(diff_left_foot[0: 3, 3, :], axis=0), label='Right')
        plt.title('Absolute distance between torso and foot')
        plt.xlabel('time (t)')
        plt.ylabel('Torso to feet (abs)')
        plt.grid(b=True, which='both', axis='both')
        plt.legend()

        plt.subplot(324)
        plt.plot(times, diff_right_foot[1, 3, :].ravel(), label='Right')
        plt.plot(times, diff_left_foot[1, 3, :].ravel(), label='Left')
        plt.title('Diff between feet and body')
        plt.xlabel('time (t)')
        plt.ylabel('Torso to feet (y)')
        plt.grid(b=True, which='both', axis='both')
        plt.legend()

        plt.subplot(326)
        plt.plot(times, crp[2, 3, :].ravel(), label='Crotch')
        plt.plot(times, lfp[2, 3, :].ravel(), label='Left')
        plt.plot(times, rfp[2, 3, :].ravel(), label='Right')
        plt.title('Z position of left, right and body')
        plt.xlabel('time (t)')
        plt.ylabel('Torso to feet (z)')
        plt.grid(b=True, which='both', axis='both')
        plt.legend()
        #plt.draw()
        fig.canvas.draw()
        plt.show(block=False)
        #plt.show()


