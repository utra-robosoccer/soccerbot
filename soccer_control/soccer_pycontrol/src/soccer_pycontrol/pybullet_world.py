import pybullet as pb
import pybullet_data


class PybulletWorld:
    """
    Class for interacting with pybullet world
    """

    def __init__(
        self,
        path: str = "plane.urdf",
        position: tuple = (0, 0, 0),
        orientation: tuple = (0, 0, 0),
        lateral_friction: float = 0.9,
        spinning_friction: float = 0.9,
        rolling_friction: float = 0.9,
        display: bool = True,
    ):
        """
        Initializes the ramp


        """

        assert pb.isConnected() == 0
        if display:
            self.client_id = pb.connect(pb.GUI)
        else:
            self.client_id = pb.connect(pb.DIRECT)

        pb.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
        pb.resetDebugVisualizerCamera(cameraDistance=1.0, cameraYaw=90, cameraPitch=0, cameraTargetPosition=[0, 0, 0.25])
        pb.setGravity(0, 0, -9.81)
        pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0)

        self.orientation = orientation
        self.position = position
        self.path = path
        self.plane = pb.loadURDF(self.path, basePosition=self.position, baseOrientation=pb.getQuaternionFromEuler(self.orientation))

        pb.changeDynamics(
            self.plane,
            linkIndex=-1,
            lateralFriction=lateral_friction,
            spinningFriction=spinning_friction,
            rollingFriction=rolling_friction,
        )

    def close(self):
        if pb.isConnected(self.client_id):
            pb.disconnect(self.client_id)
        assert pb.isConnected() == 0
