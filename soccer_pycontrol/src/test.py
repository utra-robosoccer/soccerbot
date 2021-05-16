from unittest import TestCase
import soccerbot_controller
from soccer_pycontrol.src.transformation import Transformation


class Test(TestCase):

    def test_walk_1(self):
        walker = soccerbot_controller.SoccerbotController()
        walker.soccerbot.setPose(Transformation([-1.102478309290906, -0.0054166177606295605, 0], [-0.1608068628859014, 0, 0, 0.9869858929330221]))
        walker.wait(100)
        walker.soccerbot.ready()
        walker.wait(1000)
        walker.soccerbot.setGoal(Transformation([1, 0, 0], [0, 0, 0.009834356403026491, 0.9999516415477991]), show=False)
        # walker.soccerbot.robot_path.show()
        # walker.soccerbot.calculate_angles(show=True)
        walker.run()