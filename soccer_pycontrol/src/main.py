import pybullet as pb
import pybullet_data
from transformation import Transformation
from time import sleep

from soccerbot import Soccerbot
from ramp import Ramp
import time
import matplotlib as plt

def main():
    plt.use('tkagg')

    pb.connect(pb.GUI)
    pb.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
    pb.resetDebugVisualizerCamera(cameraDistance=0.5, cameraYaw=0, cameraPitch=0, cameraTargetPosition=[0, 0, 0.1])
    pb.setGravity(0, 0, -9.81)

    soccerbot = Soccerbot([0, 0, 0], useFixedBase=True)
    ramp = Ramp("plane.urdf", (0, 0, 0), (0, 0, 0))
    t1 = time.perf_counter()
    soccerbot.stand()
    soccerbot.getPath(Transformation([0.3, 0, 0]), show=False)
    t2 = time.perf_counter()
    print("duration: " + str(t2 - t1))
    # soccerbot.calculate_angles(show=False)

    pb_step = 0.004

    for i in range(100):
        sleep(pb_step)
        pb.stepSimulation()
    print("Robot Stabilized")


    for i in range(100):
        sleep(pb_step)
        pb.stepSimulation()
    print("Robot ready for movement")

    t = 0
    while True:
        if t >= soccerbot.current_step_time and t <= soccerbot.robot_path.duration():
            soccerbot.stepPath(t)
            pb.setJointMotorControlArray(bodyIndex=soccerbot.body, controlMode=pb.POSITION_CONTROL, jointIndices=list(range(0, 18, 1)), targetPositions=soccerbot.configuration, targetVelocities= [1] * 18)
            soccerbot.current_step_time = soccerbot.current_step_time + soccerbot.robot_path.step_size

        pb.stepSimulation()
        t = t + pb_step
        sleep(pb_step)

if __name__ == '__main__':
    main()