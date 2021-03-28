import pybullet as pb
import pybullet_data
from transformation import Transformation
from time import sleep

from soccerbot import Soccerbot
from ramp import Ramp
import time
import matplotlib as plt

PYBULLET_STEP = 0.004


# Set the destination!
GOAL_X = 0.
GOAL_Y = 1.


def wait(steps):
    for i in range(steps):
        sleep(PYBULLET_STEP)
        pb.stepSimulation()


if __name__ == '__main__':
    plt.use('tkagg')

    pb.connect(pb.GUI)
    pb.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
    pb.resetDebugVisualizerCamera(cameraDistance=0.5, cameraYaw=0, cameraPitch=0, cameraTargetPosition=[0, 0, 0.25])
    pb.setGravity(0, 0, -9.81)

    soccerbot = Soccerbot([0, 0, 0], useFixedBase=False)
    ramp = Ramp("plane.urdf", (0, 0, 0), (0, 0, 0), lateralFriction=0.9, spinningFriction=0.9, rollingFriction=0.0)

    # Move to the standing position
    wait(100)
    soccerbot.ready()

    # Start walking
    wait(1000)
    soccerbot.getPath(Transformation([GOAL_X, GOAL_Y, 0]), show=False)
    # soccerbot.calculate_angles(show=True)
    t = 0
    while True:
        if soccerbot.current_step_time <= t <= soccerbot.robot_path.duration():
            soccerbot.stepPath(t, verbose=True)
            pb.setJointMotorControlArray(bodyIndex=soccerbot.body, controlMode=pb.POSITION_CONTROL, jointIndices=list(range(0, 18, 1)), targetPositions=soccerbot.configuration)
            soccerbot.current_step_time = soccerbot.current_step_time + soccerbot.robot_path.step_size

        pb.stepSimulation()
        soccerbot.get_imu(verbose=True)
        print(f'right foot: {soccerbot.get_feet(ramp.plane)[:4]} \t left foot: {soccerbot.get_feet(ramp.plane)[4:]}')
        t = t + PYBULLET_STEP
        sleep(PYBULLET_STEP)