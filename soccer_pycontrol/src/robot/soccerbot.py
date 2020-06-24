import pybullet as pb

class Robot:

    def __init__(self):

        # initialize robot
        self.body = pb.loadURDF("../../soccer_description/models/soccerbot_stl.urdf")
        self.imu = -1



        pb.calculateInverseKinematics()
        self.joints = {}
        pb.getJointInfo(self.body, i)

        self.left_subtree
        self.motors = []
        # create a list of joints and find the IMU
        for i in range(pb.getNumJoints(self.body)):
            self.joints.append(i)
            jointInfo = pb.getJointInfo(self.body, i)
            if jointInfo[1].decode('ascii') == "torso_imu":
                self.imu = jointInfo[0]

        if self.imu == -1:
            raise AttributeError("Could not find robot's imu sensor from joint list")
