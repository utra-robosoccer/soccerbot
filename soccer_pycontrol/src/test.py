from unittest import TestCase
import soccerbot_controller
from soccer_pycontrol.src.transformation import Transformation


class Test(TestCase):

    def test_walk_1(self):
        walker = soccerbot_controller.SoccerbotController()
        walker.soccerbot.setPose(Transformation([-0.7384, -0.008, 0], [0.00000, 0, 0, 1]))
        walker.wait(100)
        walker.soccerbot.ready()
        walker.wait(1000)
        walker.soccerbot.setGoal(Transformation([0.0198, -0.0199, 0], [0.00000, 0, 0, 1]), show=False)
        walker.run()