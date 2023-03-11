import time

import pybullet as pb
import pybullet_data
import rospy

from soccer_common.transformation import Transformation
from soccer_pycontrol.links import Links
from soccer_pycontrol.ramp import Ramp
from soccer_pycontrol.soccerbot import Soccerbot


class Navigator:
    """
    The 2D Navigator class, has a running loop that reads commands by the user and outputs actions to the soccerbot
    class.
    Doesn't require ROS and used for unit tests. All functions called here should be related to pybullet simulation
    """

    PYBULLET_STEP = rospy.get_param("control_frequency", 0.01)

    def __init__(self, real_time=True, display=True, useCalibration=True):
        """
        Initialize the Navigator

        :param display: Whether or not to show the pybullet visualization, turned off for quick unit tests
        :param useCalibration: Whether or not to use movement calibration files located in config/robot_model.yaml, which adjusts the calibration to the movement given
        """
        self.display = display
        self.real_time = real_time
        assert pb.isConnected() == 0
        if display:
            self.client_id = pb.connect(pb.GUI)
        else:
            self.client_id = pb.connect(pb.DIRECT)
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
        pb.resetDebugVisualizerCamera(cameraDistance=0.5, cameraYaw=0, cameraPitch=0, cameraTargetPosition=[0, 0, 0.25])
        pb.setGravity(0, 0, -9.81)
        pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0)
        self.ramp = Ramp("plane.urdf", (0, 0, 0), (0, 0, 0), lateralFriction=0.9, spinningFriction=0.9, rollingFriction=0.0)

        self.soccerbot = Soccerbot(Transformation(), useFixedBase=False, useCalibration=useCalibration)
        self.terminate_walk = False

        self.t = 0

    def close(self):
        if pb.isConnected(self.client_id):
            pb.disconnect(self.client_id)
        assert pb.isConnected() == 0

    def ready(self) -> None:
        """
        Puts the robot into a ready pose to begin walking
        """
        self.soccerbot.ready()

    def setPose(self, pose: Transformation) -> None:
        """
        Relocate the robot at the certain pose

        :param pose: 3D pose of the robot
        """
        self.soccerbot.setPose(pose)

    def getPose(self):
        """
        Get the 3D pose of the robot

        :return: The 3D pose of the robot
        """
        [position, quaternion] = pb.getLinkState(self.soccerbot.body, linkIndex=Links.LEFT_LEG_6)[4:6]
        return Transformation(position=position, quaternion=quaternion).pos_theta

    def setGoal(self, goal: Transformation) -> None:
        """
        Set the goal of the robot, will create the path to the goal that will be executed in the run() loop

        :param goal: The 3D location goal for the robot
        """
        self.soccerbot.createPathToGoal(goal)

    def wait(self, steps) -> None:
        """
        Make the robot wait for a few steps

        :param steps: Defined by Navigator.PYBULLET_STEP, which is usually 0.01
        """
        for i in range(steps):
            if self.real_time:
                time.sleep(Navigator.PYBULLET_STEP)
            pb.stepSimulation()

    def run(self, single_trajectory=False) -> bool:
        """
        The main run loop for the navigator, executes goals given through setGoal and then stops

        :param single_trajectory: If set to true, then the software will exit after a single trajectory is completed
        :return: True if the robot succeeds navigating to the goal, False if it doesn't reach the goal and falls
        """

        if self.soccerbot.robot_path.duration() == 0:
            return True

        self.t = -2
        stable_count = 20

        while self.t <= self.soccerbot.robot_path.duration():
            if self.t < 0:
                pitch = self.soccerbot.apply_imu_feedback_standing(self.soccerbot.get_imu())
                if abs(pitch - self.soccerbot.standing_pid.setpoint) < 0.025:
                    stable_count = stable_count - 1
                    if stable_count == 0:
                        t = 0
                else:
                    stable_count = 5
            else:
                if self.soccerbot.current_step_time <= self.t <= self.soccerbot.robot_path.duration():
                    self.soccerbot.stepPath(self.t)
                    self.soccerbot.apply_imu_feedback(self.soccerbot.get_imu())
                    self.soccerbot.current_step_time = self.soccerbot.current_step_time + self.soccerbot.robot_path.step_precision

            angle_threshold = 1.25  # in radian
            [roll, pitch, yaw] = self.soccerbot.get_imu().orientation_euler
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
            self.t = self.t + Navigator.PYBULLET_STEP
            if self.real_time:
                time.sleep(Navigator.PYBULLET_STEP)
        return True
