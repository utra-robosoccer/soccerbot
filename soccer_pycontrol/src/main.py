import pybullet as p
import pybullet_data
from time import sleep

from robot.soccerbot import Robot
from robot.environment import Ramp

if __name__ == '__main__':

    # Pybullet Setup
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
    ramp = Ramp("plane.urdf", (0, 0, 0), (0, 0, 0))
    myrobot = Robot()
    p.setGravity(0, 0, -10)

    # Step through simulation
    while (1):
        sleep(0.01)
        pos, orn = p.getBasePositionAndOrientation(myrobot.body)
        myrobot.stabilize()
        p.stepSimulation()