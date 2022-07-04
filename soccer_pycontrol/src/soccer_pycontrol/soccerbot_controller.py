import time
from time import sleep

import numpy as np
import pybullet as pb
import pybullet_data

from soccer_common.transformation import Transformation
from soccer_pycontrol.ramp import Ramp
from soccer_pycontrol.soccerbot import Soccerbot


class SoccerbotController:
    PYBULLET_STEP = 0.01

    def __init__(self, display=True, useCalibration=True):
        self.display = display
        if display:
            self.client_id = pb.connect(pb.GUI)
        else:
            self.client_id = pb.connect(pb.DIRECT)
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
        pb.resetDebugVisualizerCamera(cameraDistance=0.5, cameraYaw=0, cameraPitch=0, cameraTargetPosition=[0, 0, 0.25])
        pb.setGravity(0, 0, -9.81)
        self.ramp = Ramp("plane.urdf", (0, 0, 0), (0, 0, 0), lateralFriction=0.9, spinningFriction=0.9, rollingFriction=0.0)

        self.soccerbot = Soccerbot(Transformation(), useFixedBase=False, useCalibration=useCalibration)
        self.terminate_walk = False

    def __del__(self):
        if hasattr(self, "client_id") and pb.isConnected(self.client_id):
            pb.disconnect(self.client_id)

    def ready(self):
        self.soccerbot.ready()

    def setPose(self, pose: Transformation):
        self.soccerbot.setPose(pose)

    def getPose(self):
        return np.array(
            [
                pb.getBasePositionAndOrientation(self.soccerbot.body)[0][0],
                pb.getBasePositionAndOrientation(self.soccerbot.body)[0][1],
                Transformation.get_euler_from_quaternion(pb.getBasePositionAndOrientation(self.soccerbot.body)[1])[0],
            ]
        )

    def setGoal(self, goal: Transformation):
        self.soccerbot.createPathToGoal(goal)
        # self.soccerbot.robot_path.show()

    def wait(self, steps):
        for i in range(steps):
            if self.display:
                time.sleep(SoccerbotController.PYBULLET_STEP)
            pb.stepSimulation()

    def run(self, single_trajectory=False):
        if self.soccerbot.robot_path.duration() == 0:
            return True

        t = -5
        stable_count = 20

        while t <= self.soccerbot.robot_path.duration():
            if t < 0:
                pitch = self.soccerbot.apply_imu_feedback_standing(self.soccerbot.get_imu())
                if abs(pitch - self.soccerbot.standing_pid.setpoint) < 0.025:
                    stable_count = stable_count - 1
                    if stable_count == 0:
                        t = 0
                else:
                    stable_count = 5
            else:
                if self.soccerbot.current_step_time <= t <= self.soccerbot.robot_path.duration():
                    self.soccerbot.stepPath(t, verbose=False)
                    self.soccerbot.apply_imu_feedback(t, self.soccerbot.get_imu())
                    self.soccerbot.current_step_time = self.soccerbot.current_step_time + self.soccerbot.robot_path.step_precision

            angle_threshold = 1.25  # in radian
            [roll, pitch, yaw] = self.soccerbot.get_imu().get_orientation_euler()
            if pitch > angle_threshold:
                print("Fallen Back")
                return False

            elif pitch < -angle_threshold:
                print("Fallen Front")
                return False

            pb.setJointMotorControlArray(
                bodyIndex=self.soccerbot.body,
                controlMode=pb.POSITION_CONTROL,
                jointIndices=list(range(0, 18, 1)),
                targetPositions=self.soccerbot.get_angles(),
            )
            pb.stepSimulation()
            t = t + SoccerbotController.PYBULLET_STEP
            if self.display:
                sleep(SoccerbotController.PYBULLET_STEP)
        return True

    def updateGoal(self):
        pass
