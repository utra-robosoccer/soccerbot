import unittest

import numpy as np
import pytest
from soccer_pycontrol.soccerbot2.foot_step_planner import FootStepPlanner
from soccer_pycontrol.soccerbot2.handle_urdf import HandleURDF
from soccer_pycontrol.soccerbot2.pybullet.pybullet_env import PybulletEnv
from soccer_pycontrol.soccerbot2.pybullet.pybullet_world import PybulletWorld

from soccer_common import Transformation


class TestPybullet(unittest.TestCase):
    def test_imu(self):
        world = PybulletWorld()
        model = HandleURDF()
        p = PybulletEnv(model, world, real_time=True, rate=100)
        p.wait(100)
        print(p.sensors.get_imu().orientation_euler)
        # TODO add more

    def test_foot_sensor(self):
        world = PybulletWorld()
        model = HandleURDF()
        p = PybulletEnv(model, world, real_time=True, rate=100)
        p.wait(100)
        print(p.sensors.get_foot_pressure_sensors(p.world.plane))
        # TODO add more

    def test_foot_step_planner_fixed(self):
        world = PybulletWorld(path="")
        model = HandleURDF(fixed_base=True)
        p = PybulletEnv(model, world, real_time=True, rate=100)
        fp = FootStepPlanner(p.robot_path, p.handle_urdf)
        p.wait(50)
        p.motor_control.set_target_angles(p.ik_actions.ready())
        p.motor_control.set_motor()
        p.wait(50)
        fp.createPathToGoal(Transformation([1, 0, 0], [0, 0, 0, 1]))
        # p.wmodelait(100)

        pitches = []
        times = []
        t = 0

        while t <= fp.robot_path.duration():
            if fp.current_step_time <= t <= fp.robot_path.duration():
                torso_to_right_foot, torso_to_left_foot = fp.stepPath(t)
                r_theta = p.ik_actions.ik.ik_right_foot(torso_to_right_foot)
                l_theta = p.ik_actions.ik.ik_left_foot(torso_to_left_foot)
                p.motor_control.set_right_leg_target_angles(r_theta[0:6])
                p.motor_control.set_left_leg_target_angles(l_theta[0:6])

                # pitch = self.walker.soccerbot2.get_imu().orientation_euler[1]
                # f = self.walker.soccerbot2.apply_imu_feedback(t, self.walker.soccerbot2.get_imu())
                p.motor_control.set_motor()
                fp.current_step_time = fp.current_step_time + fp.robot_path.step_precision
                times.append(t)
                # pitches.append((pitch, f))
            p.step()
            t = t + 0.01

        p.wait(100)

    def test_foot_step_planner_plane(self):
        world = PybulletWorld()
        model = HandleURDF()
        p = PybulletEnv(model, world, real_time=True, rate=100)

        p.wait(50)
        # p.handle_urdf.set_pose.set_pose(Transformation([0.0, 0, 0], [0, 0, 0, 1]))
        p.motor_control.set_target_angles(p.ik_actions.ready())
        p.motor_control.set_motor()
        p.wait(50)
        p.step_planner.createPathToGoal(Transformation([1, 1, 0], [0, 0, 0, 1]))

        p.pid.reset_imus()
        p.pid.reset_roll_feedback_parameters(p.step_planner.robot_path)
        times = []
        t = 0
        # TODO make a navigator class
        while t <= p.step_planner.robot_path.duration():
            if p.step_planner.current_step_time <= t <= p.step_planner.robot_path.duration():
                imu = p.sensors.get_imu()
                t_offset = p.pid.apply_phase_difference_roll_feedback(t, imu, p.step_planner.robot_path)
                torso_to_right_foot, torso_to_left_foot = p.step_planner.stepPath(t_offset)
                r_theta = p.ik_actions.ik.ik_right_foot(torso_to_right_foot)
                l_theta = p.ik_actions.ik.ik_left_foot(torso_to_left_foot)
                p.motor_control.set_right_leg_target_angles(r_theta[0:6])
                p.motor_control.set_left_leg_target_angles(l_theta[0:6])
                F = p.pid.apply_imu_feedback(imu)
                p.motor_control.set_leg_joint_3_target_angle(F)

                p.motor_control.set_motor()
                p.step_planner.current_step_time = (
                    p.step_planner.current_step_time + p.step_planner.robot_path.step_precision
                )  # TODO needs to be 0.02 or much more unstable
                times.append(t)
            p.step()
            t = t + 0.01

        p.wait(100)


@pytest.mark.parametrize("robot_model", ["bez1", "bez2"])
def test_ready(robot_model: str):
    """
    Case 1: Standard case
    :return: None
    """
    world = PybulletWorld()
    model = HandleURDF(robot_model=robot_model, pose=Transformation())
    p = PybulletEnv(model, world, real_time=True, rate=250)
    p.wait(100)
    p.motor_control.set_target_angles(p.ik_actions.ready())
    p.motor_control.set_motor()
    # TODO do i need this
    # self.motor_control.configuration_offset = [0] * len(Joints)

    p.wait_motor()
    p.wait(100)

    p.world.close()
    # TODO doesnt work with more then 1 robot model weird


@pytest.mark.parametrize("sweep_name", ["x", "y", "z"])
@pytest.mark.parametrize("h", [0.0, 0.05, 0.1])
@pytest.mark.parametrize("robot_model", ["bez1", "bez2"])
def test_sweep(sweep_name: str, h: float, robot_model: str):
    """
    Case 1: Standard case
    :return: None
    """
    world = PybulletWorld(path="")
    model = HandleURDF(fixed_base=True, robot_model=robot_model)
    p = PybulletEnv(model, world, real_time=True, rate=1000)

    steps = 50
    x = np.zeros(steps)

    def sweep(target_pose: np.ndarray, step: int) -> None:
        for i in range(step):
            p.motor_control.set_right_leg_target_angles(target_pose[i][0:6])
            p.motor_control.set_motor()
            p.wait_motor()

    if sweep_name == "x":
        x, _, _ = p.ik_actions.x_sweep(h)
    elif sweep_name == "y":
        x, _, _ = p.ik_actions.y_sweep(h)
    elif sweep_name == "z":
        x, _, _ = p.ik_actions.z_sweep()

    sweep(x, steps)
    p.wait(steps)

    p.world.close()
