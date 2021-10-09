import os
from soccer_geometry.transformation import Transformation
from time import sleep
from soccer_pycontrol.soccerbot import Soccerbot
from soccer_pycontrol.ramp import Ramp
import rospy
import time
if os.getenv('ENABLE_PYBULLET', False):
    import pybullet as pb
    import pybullet_data


class SoccerbotController:
    PYBULLET_STEP = 0.004

    def __init__(self):
        if os.getenv('ENABLE_PYBULLET', False):
            if os.getenv('COMPETITION', False):
                pb.connect(pb.DIRECT)
            else:
                pb.connect(pb.GUI)
            pb.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
            pb.resetDebugVisualizerCamera(cameraDistance=0.5, cameraYaw=0, cameraPitch=0,
                                          cameraTargetPosition=[0, 0, 0.25])
            pb.setGravity(0, 0, -9.81)
            self.ramp = Ramp("plane.urdf", (0, 0, 0), (0, 0, 0), lateralFriction=0.9, spinningFriction=0.9,
                             rollingFriction=0.0)

        self.soccerbot = Soccerbot(Transformation(), useFixedBase=False)

    def ready(self):
        self.soccerbot.ready()

    def setPose(self, pose: Transformation):
        self.soccerbot.setPose(pose)

    def setGoal(self, goal: Transformation):
        self.soccerbot.setGoal(goal)

    def wait(self, steps):
        for i in range(steps):
            time.sleep(SoccerbotController.PYBULLET_STEP)
            if os.getenv('ENABLE_PYBULLET', False):
                pb.stepSimulation()

    def run(self, stop_on_completed_trajectory=False):
        if self.soccerbot.robot_path.duration() == 0:
            return

        t = 0
        while t <= self.soccerbot.robot_path.duration():
            if self.soccerbot.current_step_time <= t <= self.soccerbot.robot_path.duration():
                self.soccerbot.stepPath(t, verbose=False)
                self.soccerbot.apply_imu_feedback(t, self.soccerbot.get_imu())
                forces = self.soccerbot.apply_foot_pressure_sensor_feedback(self.ramp.plane)
                if os.getenv('ENABLE_PYBULLET', False):
                    pb.setJointMotorControlArray(bodyIndex=self.soccerbot.body, controlMode=pb.POSITION_CONTROL,
                                             jointIndices=list(range(0, 20, 1)),
                                             targetPositions=self.soccerbot.get_angles(),
                                             forces=forces
                                             )
                self.soccerbot.current_step_time = self.soccerbot.current_step_time + self.soccerbot.robot_path.step_size
            if os.getenv('ENABLE_PYBULLET', False):
                pb.stepSimulation()
            t = t + SoccerbotController.PYBULLET_STEP
            sleep(SoccerbotController.PYBULLET_STEP)

    def updateGoal(self):
        pass