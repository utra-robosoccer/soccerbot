import pybullet as p


class Ramp:
    """
    Ramp object for the robot to walk on
    """

    def __init__(self, path, position, orientation, lateralFriction, spinningFriction, rollingFriction):
        """
        Initializes the ramp

        :param path: File path for the ramp object
        :param position: Position of the ramp
        :param orientation: Orientation of the ramp
        :param lateralFriction: lateralFriction
        :param spinningFriction: spinningFriction
        :param rollingFriction: rollingFriction
        """
        self.orientation = orientation
        self.position = position
        self.path = path
        self.plane = p.loadURDF(self.path, basePosition=self.position, baseOrientation=p.getQuaternionFromEuler(self.orientation))
        p.changeDynamics(
            self.plane,
            linkIndex=-1,
            lateralFriction=lateralFriction,
            spinningFriction=spinningFriction,
            rollingFriction=rollingFriction,
        )
