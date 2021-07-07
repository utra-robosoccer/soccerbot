import os
import rospy
if rospy.get_param('ENABLE_PYBULLET'):
    import pybullet as p


class Ramp:

    def __init__(self, path, position, orientation, lateralFriction, spinningFriction, rollingFriction):
        self.orientation = orientation
        self.position = position
        self.path = path
        if rospy.get_param('ENABLE_PYBULLET'):
            self.plane = p.loadURDF(self.path, basePosition=self.position,
                                    baseOrientation=p.getQuaternionFromEuler(self.orientation))
            p.changeDynamics(self.plane, linkIndex=-1, lateralFriction=lateralFriction,
                             spinningFriction=spinningFriction, rollingFriction=rollingFriction)

    def setOrientation(self, orientation):
        if rospy.get_param('ENABLE_PYBULLET'):
            p.removeBody(self.plane)
        self.__init__(self.path, self.position, orientation)

    def setPosition(self, position):
        if rospy.get_param('ENABLE_PYBULLET'):
            p.removeBody(self.plane)
        self.__init__(self.path, position, self.orientation)
