import unittest
import matplotlib.pyplot as plt
from soccer_pycontrol.model.bez import Bez  # Import the Bez class
from soccer_pycontrol.pybullet_usage.pybullet_world import PybulletWorld  # Import the PybulletWorld class
from soccer_common import Transformation


class TestSensors(unittest.TestCase):

    def setUp(self):
        """
        Set up the PyBullet environment and the robot with sensors.
        """
        self.world = PybulletWorld(
            camera_yaw=90,
            real_time=True,  # Change to False if you don't want real time simulation
            rate=200,
        )

        self.robot_model = "bez2"
        self.bez = Bez(robot_model=self.robot_model, pose=Transformation())

    def tearDown(self):
        """
        Close the PyBullet environment.
        """
        self.world.close()
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