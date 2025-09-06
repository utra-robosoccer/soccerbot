import unittest
import matplotlib.pyplot as plt
from soccer_pycontrol.model.bez import Bez  # Import the Bez class
from soccer_pycontrol.mujoco.simulator import Simulator  # Import the MuJoCo Simulator class
from soccer_common import Transformation


class TestSensors(unittest.TestCase):

    def setUp(self):
        """
        Set up the MuJoCo environment and the robot with sensors.
        """
        self.world = Simulator(scene_name="scene_assembly.xml")

        self.robot_model = "assembly"
        self.bez = Bez(robot_model=self.robot_model, pose=Transformation(), simulator=self.world)

    def tearDown(self):
        """
        Close the MuJoCo environment.
        """
        # MuJoCo doesn't need explicit closing like PyBullet
        del self.bez
        del self.world

    def test_camera_image_capture(self):
        """
        Test to capture an image from the robot's camera using the Sensors class.
        """
        image = self.bez.sensors.get_camera_image()

        # self.assertEqual(image.shape, (480, 640, 3), "Captured image does not have the correct dimensions.")

        # Display the captured image
        plt.imshow(image)
        plt.title('Captured Camera Image')
        plt.axis('off')
        plt.show()

if __name__ == "__main__":
    unittest.main()
