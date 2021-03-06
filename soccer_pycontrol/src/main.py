import pybullet as pb
import pybullet_data
from transformation import Transformation
from time import sleep

from soccerbot import Soccerbot
from ramp import Ramp
import time
import matplotlib as plt

PYBULLET_STEP = 0.004

def wait(steps):
    for i in range(steps):
        sleep(PYBULLET_STEP)
        pb.stepSimulation()

def main():
    plt.use('tkagg')

    pb.connect(pb.GUI)
    pb.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
    pb.resetDebugVisualizerCamera(cameraDistance=0.5, cameraYaw=0, cameraPitch=0, cameraTargetPosition=[0, 0, 0.1])
    pb.setGravity(0, 0, -9.81)

    soccerbot = Soccerbot([0, 0, 0], useFixedBase=False)
    ramp = Ramp("plane.urdf", (0, 0, 0), (0, 0, 0))
    wait(1000)




    # Move to the standing position

    soccerbot.stand()
    wait(1000)


    soccerbot.getPath(Transformation([0.3, 0, 0]), show=False)
    # soccerbot.calculate_angles(show=True)

    wait(100)

    t = 0
    while True:
        if soccerbot.current_step_time <= t <= soccerbot.robot_path.duration():
            soccerbot.stepPath(t)
            pb.setJointMotorControlArray(bodyIndex=soccerbot.body, controlMode=pb.POSITION_CONTROL, jointIndices=list(range(0, 18, 1)), targetPositions=soccerbot.configuration)
            soccerbot.current_step_time = soccerbot.current_step_time + soccerbot.robot_path.step_size

        pb.stepSimulation()
        t = t + PYBULLET_STEP
        sleep(PYBULLET_STEP)

if __name__ == '__main__':
    main()