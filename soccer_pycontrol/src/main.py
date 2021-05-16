import os
if "ROS_NAMESPACE" not in os.environ:
    os.environ["ROS_NAMESPACE"] = "/robot1"
import pybullet as pb
import pybullet_data
from transformation import Transformation
from time import sleep
from soccerbot import Soccerbot
from ramp import Ramp
import matplotlib as plt
import rospy

PYBULLET_STEP = 0.008

def wait(steps):
    for i in range(steps):
        sleep(PYBULLET_STEP)
        pb.stepSimulation()


if __name__ == '__main__':
    rospy.init_node("soccer_control")
    rospy.get_namespace()
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
    soccerbot.publishAngles()

    # Start walking
    wait(1000)
    soccerbot.getPath(Transformation([1, 0, 0]), show=False)
    # soccerbot.calculate_angles(show=True)
    t = 0

    while not rospy.is_shutdown():
        if soccerbot.current_step_time <= t <= soccerbot.robot_path.duration():
            soccerbot.stepPath(t, verbose=True)
            pb.setJointMotorControlArray(bodyIndex=soccerbot.body, controlMode=pb.POSITION_CONTROL, jointIndices=list(range(0, 18, 1)), targetPositions=soccerbot.configuration)
            soccerbot.current_step_time = soccerbot.current_step_time + soccerbot.robot_path.step_size

        pb.stepSimulation()
        soccerbot.get_imu(verbose=True)
        print(f'right foot: {soccerbot.get_feet(ramp.plane)[:4]} \t left foot: {soccerbot.get_feet(ramp.plane)[4:]}')
        soccerbot.publishAngles()
        t = t + PYBULLET_STEP
        sleep(PYBULLET_STEP)


