import pybullet as pb
import pybullet_data
from transformation import Transformation
from time import sleep
from soccerbot import Soccerbot
from ramp import Ramp
import matplotlib as plt
import rospy


class SoccerbotController:
    PYBULLET_STEP = 0.004

    def __init__(self):
        pb.connect(pb.GUI)
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
        pb.resetDebugVisualizerCamera(cameraDistance=0.5, cameraYaw=0, cameraPitch=0, cameraTargetPosition=[0, 0, 0.25])
        pb.setGravity(0, 0, -9.81)

        self.soccerbot = Soccerbot(Transformation(), useFixedBase=False)
        self.ramp = Ramp("plane.urdf", (0, 0, 0), (0, 0, 0), lateralFriction=0.9, spinningFriction=0.9, rollingFriction=0.0)

    def wait(self, steps):
        for i in range(steps):
            rospy.sleep(SoccerbotController.PYBULLET_STEP)
            pb.stepSimulation()

    def run(self):
        if self.soccerbot.robot_path.duration() == 0:
            return

        t = 0
        while t <= self.soccerbot.robot_path.duration():
            if self.soccerbot.current_step_time <= t <= self.soccerbot.robot_path.duration():
                self.soccerbot.stepPath(t, verbose=True)
                forces = self.soccerbot.get_motor_forces(self.ramp.plane)
                self.soccerbot.get_imu(verbose=True)
                pb.setJointMotorControlArray(bodyIndex=self.soccerbot.body, controlMode=pb.POSITION_CONTROL,
                                             jointIndices=list(range(0, 18, 1)),
                                             targetPositions=self.soccerbot.configuration,
                                             forces=forces
                                             )
                self.soccerbot.current_step_time = self.soccerbot.current_step_time + self.soccerbot.robot_path.step_size

            pb.stepSimulation()
            t = t + SoccerbotController.PYBULLET_STEP
            sleep(SoccerbotController.PYBULLET_STEP)
