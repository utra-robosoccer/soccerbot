from unittest import TestCase

import numpy as np

import soccerbot_controller
from soccer_pycontrol.src.transformation import Transformation


class Test(TestCase):

    def test_walk_1(self):
        walker = soccerbot_controller.SoccerbotController()
        walker.soccerbot.setPose(Transformation([0, 0, 0], [0, 0, 0, 1]))
        walker.wait(100)
        walker.soccerbot.ready()
        walker.wait(1000)
        walker.soccerbot.setGoal(Transformation([5, 0, 0], [0, 0, 0, 1]))
        # walker.soccerbot.robot_path.show()
        walker.run()

    def test_walk_2(self):
        walker = soccerbot_controller.SoccerbotController()
        walker.soccerbot.setPose(Transformation([-0.7384, -0.008, 0], [0.00000, 0, 0, 1]))
        walker.wait(100)
        walker.soccerbot.ready()
        walker.wait(1000)
        walker.soccerbot.setGoal(Transformation([0.0198, -0.0199, 0], [0.00000, 0, 0, 1]))
        # walker.soccerbot.robot_path.show()
        walker.run()

    def test_walk_side(self):
        walker = soccerbot_controller.SoccerbotController()
        walker.soccerbot.setPose(Transformation([0, 0, 0], [0.00000, 0, 0, 1]))
        walker.wait(100)
        walker.soccerbot.ready()
        walker.wait(1000)
        walker.soccerbot.setGoal(Transformation([0, -1, 0], [0.00000, 0, 0, 1]))
        # walker.soccerbot.robot_path.show()
        walker.run()

    def test_walk_backward(self):
        walker = soccerbot_controller.SoccerbotController()
        walker.soccerbot.setPose(Transformation([0, 0, 0], [0.00000, 0, 0, 1]))
        walker.wait(100)
        walker.soccerbot.ready()
        walker.wait(1000)
        walker.soccerbot.setGoal(Transformation([-1, 0, 0], [0.00000, 0, 0, 1]))
        walker.run()

    def test_turn_in_place(self):
        walker = soccerbot_controller.SoccerbotController()
        walker.soccerbot.setPose(Transformation([0, 0, 0], [0.00000, 0, 0, 1]))
        walker.wait(100)
        walker.soccerbot.ready()
        walker.wait(1000)
        goal = Transformation.get_transform_from_euler([np.pi, 0, 0])
        walker.soccerbot.setGoal(goal)
        # walker.soccerbot.robot_path.show()
        walker.run()

    def test_small_movement_1(self):
        walker = soccerbot_controller.SoccerbotController()
        walker.soccerbot.setPose(Transformation([0, 0, 0], [0.00000, 0, 0, 1]))
        walker.wait(100)
        walker.soccerbot.ready()
        walker.wait(1000)
        goal = Transformation.get_transform_from_euler([np.pi, 0, 0])
        goal.set_position([0.3, 0, 0])
        walker.soccerbot.setGoal(goal)
        walker.soccerbot.robot_path.show()
        walker.run()

    def test_small_movement_2(self):
        walker = soccerbot_controller.SoccerbotController()
        walker.soccerbot.setPose(Transformation([0, 0, 0], [0.00000, 0, 0, 1]))
        walker.wait(100)
        walker.soccerbot.ready()
        walker.wait(1000)
        goal = Transformation.get_transform_from_euler([np.pi, 0, 0])
        goal.set_position([-0.3, 0, 0])
        walker.soccerbot.setGoal(goal)
        walker.soccerbot.robot_path.show()
        walker.run()

    def test_small_movement_3(self):
        walker = soccerbot_controller.SoccerbotController()
        walker.soccerbot.setPose(Transformation([0, 0, 0], [0.00000, 0, 0, 1]))
        walker.wait(100)
        walker.soccerbot.ready()
        walker.wait(1000)
        goal = Transformation.get_transform_from_euler([-np.pi/2, 0, 0])
        goal.set_position([-0.3, -0.3, 0])
        walker.soccerbot.setGoal(goal)
        walker.soccerbot.robot_path.show()
        walker.run()

    def test_do_nothing(self):
        walker = soccerbot_controller.SoccerbotController()
        walker.soccerbot.setPose(Transformation([0, 0, 0], [0.00000, 0, 0, 1]))
        walker.wait(100)
        walker.soccerbot.ready()
        walker.wait(1000)
        goal = Transformation.get_transform_from_euler([0, 0, 0])
        walker.soccerbot.setGoal(goal)
        # walker.soccerbot.robot_path.show()
        walker.run()