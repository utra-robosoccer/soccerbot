import os
import rospy
if os.getenv('ENABLE_PYBULLET', False):
    import pybullet as p


class Ramp:

    def __init__(self, path, position, orientation, lateralFriction, spinningFriction, rollingFriction):
        self.orientation = orientation
        self.position = position
        self.path = path
        if os.getenv('ENABLE_PYBULLET', False):
            self.plane = p.loadURDF(self.path, basePosition=self.position,
                                    baseOrientation=p.getQuaternionFromEuler(self.orientation))
            p.changeDynamics(self.plane, linkIndex=-1, lateralFriction=lateralFriction,
                             spinningFriction=spinningFriction, rollingFriction=rollingFriction)

    def setOrientation(self, orientation):
        if os.getenv('ENABLE_PYBULLET', False):
            p.removeBody(self.plane)
        self.__init__(self.path, self.position, orientation)

    def setPosition(self, position):
        if os.getenv('ENABLE_PYBULLET', False):
            p.removeBody(self.plane)
        self.__init__(self.path, position, self.orientation)
