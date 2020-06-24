import pybullet as p

class Ramp:

    def __init__(self, path, position, orientation):
        self.orientation = orientation
        self.position = position
        self.path = path
        self.plane = p.loadURDF(self.path, basePosition=self.position,
                                baseOrientation=p.getQuaternionFromEuler(self.orientation))

    def setOrientation(self, orientation):
        p.removeBody(self.plane)
        self.__init__(self.path, self.position, orientation)

    def setPosition(self, position):
        p.removeBody(self.plane)
        self.__init__(self.path, position, self.orientation)