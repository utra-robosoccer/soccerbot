import pybullet as pb
import pybullet_data
from time import sleep

from robot.soccerbot import Robot
from robot.ramp import Ramp

if __name__ == '__main__':

    # Pybullet Setup
    pb.connect(pb.GUI)
    pb.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
    ramp = Ramp("plane.urdf", (0, 0, 0), (0, 0, 0))
    myrobot = Robot()
    pb.setGravity(0, 0, -10)

    # Step through simulation
    while (1):
        sleep(0.01)
        pos, orn = pb.getBasePositionAndOrientation(myrobot.body)
        pb.stepSimulation()