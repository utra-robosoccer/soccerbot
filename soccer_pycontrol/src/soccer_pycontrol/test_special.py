import glob
import os

if "ROS_NAMESPACE" not in os.environ:
    os.environ["ROS_NAMESPACE"] = "/robot1"

import math
from unittest import TestCase

import matplotlib.pyplot as plt
import numpy as np
from path import Path

from soccer_common.transformation import Transformation
from soccer_msgs.msg import RobotState
from soccer_pycontrol.soccerbot import Soccerbot
from soccer_pycontrol.soccerbot_controller import SoccerbotController
from soccer_pycontrol.soccerbot_controller_ros import SoccerbotControllerRos

RUN_IN_ROS = True
if RUN_IN_ROS:
    import rospy
    import soccerbot_controller_ros
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from std_msgs.msg import String


class TestSpecial(TestCase):
    def setUp(self) -> None:
        if RUN_IN_ROS:
            rospy.init_node("soccer_control")
            os.system("/bin/bash -c 'source /opt/ros/noetic/setup.bash && rosnode kill /robot1/soccer_strategy'")
            os.system("/bin/bash -c 'source /opt/ros/noetic/setup.bash && rosnode kill /robot1/soccer_pycontrol'")
            os.system("/bin/bash -c 'source /opt/ros/noetic/setup.bash && rosnode kill /robot1/soccer_trajectories'")
            self.walker = SoccerbotControllerRos()
        else:
            self.walker = SoccerbotController()

    def test_imu_feedback(self):
        import pybullet as pb

        self.walker.setPose(Transformation([0, 0, 0], [0, 0, 0, 1]))
        self.walker.ready()
        self.walker.wait(100)
        self.walker.setGoal(Transformation([2, 0, 0], [0, 0, 0, 1]))

        pitches = []
        times = []
        t = 0
        while t <= self.walker.soccerbot.robot_path.duration():
            if self.walker.soccerbot.current_step_time <= t <= self.walker.soccerbot.robot_path.duration():
                self.walker.soccerbot.stepPath(t, verbose=True)
                pitch = self.walker.soccerbot.get_imu().get_orientation_euler()[1]
                pitches.append(pitch)
                times.append(t)
                self.walker.soccerbot.apply_imu_feedback(self.walker.soccerbot.get_imu())

                forces = self.walker.soccerbot.apply_foot_pressure_sensor_feedback(self.walker.ramp.plane)
                pb.setJointMotorControlArray(
                    bodyIndex=self.walker.soccerbot.body,
                    controlMode=pb.POSITION_CONTROL,
                    jointIndices=list(range(0, 20, 1)),
                    targetPositions=self.walker.soccerbot.get_angles(),
                    forces=forces,
                )
                self.walker.soccerbot.current_step_time = self.walker.soccerbot.current_step_time + self.walker.soccerbot.robot_path.step_precision

            pb.stepSimulation()
            t = t + self.walker.PYBULLET_STEP

        plt.plot(times, pitches)
        plt.xlabel("Time (t)")
        plt.ylabel("Forward pitch of robot in radians")
        plt.grid(which="minor")
        plt.show()

    def test_imu_feedback_webots(self):
        import pybullet as pb
        from soccerbot_controller import SoccerbotController

        self.walker.setPose(Transformation([0, 0, 0], [0, 0, 0, 1]))
        self.walker.ready()
        self.walker.wait(100)

        self.walker.soccerbot.ready()  # TODO Cancel walking
        self.walker.soccerbot.reset_head()
        self.walker.soccerbot.publishAngles()
        print("Getting ready")
        self.walker.wait(150)

        # Reset robot position and goal
        self.walker.soccerbot.createPathToGoal(Transformation([0.5, 0, 0], [0, 0, 0, 1]))

        pitches = []
        times = []
        t = 0
        r = rospy.Rate(1 / SoccerbotController.PYBULLET_STEP)

        while t <= self.walker.soccerbot.robot_path.duration():
            if self.walker.soccerbot.current_step_time <= t <= self.walker.soccerbot.robot_path.duration():
                self.walker.soccerbot.stepPath(t, verbose=True)
                pitch = self.walker.soccerbot.get_imu().get_orientation_euler()[1]
                f = self.walker.soccerbot.apply_imu_feedback(t, self.walker.soccerbot.get_imu())

                times.append(t)
                pitches.append((pitch, f))

                forces = self.walker.soccerbot.apply_foot_pressure_sensor_feedback(self.walker.ramp.plane)
                pb.setJointMotorControlArray(
                    bodyIndex=self.walker.soccerbot.body,
                    controlMode=pb.POSITION_CONTROL,
                    jointIndices=list(range(0, 20, 1)),
                    targetPositions=self.walker.soccerbot.get_angles(),
                    forces=forces,
                )
                self.walker.soccerbot.current_step_time = self.walker.soccerbot.current_step_time + self.walker.soccerbot.robot_path.step_precision

            pb.stepSimulation()
            self.walker.soccerbot.publishAngles()
            t = t + self.walker.PYBULLET_STEP
            r.sleep()

        plt.plot(times, pitches)
        plt.xlabel("Time (t)")
        plt.ylabel("Forward pitch of robot in radians")
        plt.grid(which="minor")
        plt.show()

    def test_foot_pressure_synchronization(self):
        import pybullet as pb

        fig, axs = plt.subplots(2)

        self.walker.setPose(Transformation([0, 0, 0], [0, 0, 0, 1]))
        self.walker.ready()
        self.walker.wait(100)
        self.walker.setGoal(Transformation([1, 0, 0], [0, 0, 0, 1]))

        times = np.linspace(
            0,
            self.walker.soccerbot.robot_path.duration(),
            num=math.ceil(self.walker.soccerbot.robot_path.duration() / self.walker.soccerbot.robot_path.step_precision) + 1,
        )
        lfp = np.zeros((4, 4, len(times)))
        rfp = np.zeros((4, 4, len(times)))
        i = 0
        for t in times:
            [lfp[:, :, i], rfp[:, :, i]] = self.walker.soccerbot.robot_path.footPosition(t)
            i = i + 1

        right = rfp[2, 3, :].ravel()
        left = lfp[2, 3, :].ravel()
        right = 0.1 - right
        left = left - 0.1
        axs[1].plot(times, left, label="Left")
        axs[1].plot(times, right, label="Right")
        axs[1].grid(b=True, which="both", axis="both")
        axs[1].legend()

        t = 0
        scatter_pts_x = []
        scatter_pts_y = []
        scatter_pts_x_1 = []
        scatter_pts_y_1 = []

        while t <= self.walker.soccerbot.robot_path.duration():
            if self.walker.soccerbot.current_step_time <= t <= self.walker.soccerbot.robot_path.duration():
                self.walker.soccerbot.stepPath(t, verbose=True)
                self.walker.soccerbot.apply_imu_feedback(self.walker.soccerbot.get_imu())
                sensors = self.walker.soccerbot.get_foot_pressure_sensors(self.walker.ramp.plane)
                for i in range(len(sensors)):
                    if sensors[i] == True:
                        scatter_pts_x.append(t)
                        scatter_pts_y.append(i)
                if np.sum(sensors[0:4]) >= 2:
                    scatter_pts_x_1.append(t)
                    scatter_pts_y_1.append(-0.1)
                if np.sum(sensors[4:8]) >= 2:
                    scatter_pts_x_1.append(t)
                    scatter_pts_y_1.append(0.1)

                forces = self.walker.soccerbot.apply_foot_pressure_sensor_feedback(self.walker.ramp.plane)
                pb.setJointMotorControlArray(
                    bodyIndex=self.walker.soccerbot.body,
                    controlMode=pb.POSITION_CONTROL,
                    jointIndices=list(range(0, 20, 1)),
                    targetPositions=self.walker.soccerbot.get_angles(),
                    forces=forces,
                )
                self.walker.soccerbot.current_step_time = self.walker.soccerbot.current_step_time + self.walker.soccerbot.robot_path.step_precision

            pb.stepSimulation()
            t = t + self.walker.PYBULLET_STEP

        axs[0].scatter(scatter_pts_x, scatter_pts_y, s=3)
        axs[1].scatter(scatter_pts_x_1, scatter_pts_y_1, s=3)
        plt.show()

    def amcl_pose_callback(self, amcl_pose):
        self.amcl_pose = amcl_pose
        pass

    def test_terminate_walk(self):
        import rospy

        self.walker.setPose(Transformation([0.5, 0, 0], [0, 0, 0, 1]))
        self.walker.ready()
        self.walker.wait(100)
        self.walker.setGoal(Transformation([2.0, 1.0, 0], [0, 0, 0, 1]))

        def send_terminate_walk(_):
            self.walker.terminate_walk = True
            pass

        self.send_terminate_walk = rospy.Timer(rospy.Duration(5), send_terminate_walk, oneshot=True)
        self.walker.run()

    def test_dynamic_walking_1(self):
        import rospy

        self.walker.setPose(Transformation([0.5, 0, 0], [0, 0, 0, 1]))
        self.walker.ready()
        self.walker.wait(100)
        self.walker.setGoal(Transformation([1.5, 0, 0], [0, 0, 0, 1]))

        def send_alternative_trajectory(_):
            self.walker.setGoal(Transformation([2.5, 0.5, 0], [0, 0, 0, 1]))
            pass

        self.send_alternative_trajectory = rospy.Timer(rospy.Duration(5), send_alternative_trajectory, oneshot=True)
        self.walker.run()

    def test_path_combination_basic(self):
        plt.figure()

        height = 0.321
        start_transform = Transformation([0.5, 0, height], [0, 0, 0, 1])
        end_transform = Transformation([0.9, 0, height], [0, 0, 0, 1])

        path = Path(start_transform, end_transform)
        path.show()

        t = 2
        end_transform_new = Transformation([1.3, 0, height], [0, 0, 0, 1])
        path.dynamicallyUpdateGoalPosition(t, end_transform_new)
        path.show()

        plt.show()

    def test_path_combination(self):

        height = 0.321
        start_transform = Transformation([0.5, 0, height], [0, 0, 0, 1])
        end_transform = Transformation([1.5, 0, height], [0, 0, 0, 1])

        path = Path(start_transform, end_transform)
        # path.show()
        t = 3
        end_transform_new = Transformation([2.0, 0.5, height], [0, 0, 0, 1])
        path.dynamicallyUpdateGoalPosition(t, end_transform_new)
        # path.show()
        t = 6
        end_transform_new = Transformation([2.0, -0.5, height], [0, 0, 0, 1])
        path.dynamicallyUpdateGoalPosition(t, end_transform_new)
        # path.show()
        t = 9
        end_transform_new = Transformation([1.0, -0.5, height], [0, 0, 0, 1])
        path.dynamicallyUpdateGoalPosition(t, end_transform_new)
        plt.figure()
        path.show()
        plt.show()

        self.walker.setPose(Transformation([0.5, 0, 0], [0, 0, 0, 1]))
        self.walker.ready()
        self.walker.wait(100)
        self.walker.soccerbot.createPathToGoal(Transformation([2, 0, 0], [0, 0, 0, 1]))
        self.walker.soccerbot.robot_path.path_sections = path.path_sections
        # self.walker.soccerbot.robot_path.show()
        self.walker.run()
        pass

    def test_path_combination_2(self):
        import rospy

        self.walker.setPose(Transformation([0.5, 0, 0], [0, 0, 0, 1]))
        self.walker.ready()
        self.walker.wait(100)
        self.walker.setGoal(Transformation([1.5, 0.5, 0], [0, 0, 0, 1]))

        def send_alternative_trajectory(_):
            self.walker.setGoal(Transformation([1.5, -0.5, 0], [0, 0, 0, 1]))
            pass

        self.send_alternative_trajectory = rospy.Timer(rospy.Duration(5), send_alternative_trajectory, oneshot=True)
        self.walker.run()

    def test_path_combination_long_to_short(self):
        import rospy

        self.walker.setPose(Transformation([0.5, 0, 0], [0, 0, 0, 1]))
        self.walker.ready()
        self.walker.wait(100)
        self.walker.setGoal(Transformation([1.0, 0, 0], [0, 0, 0, 1]))

        def send_alternative_trajectory(_):
            self.walker.setGoal(Transformation([1, 0.1, 0], [0, 0, 0, 1]))
            pass

        self.send_alternative_trajectory = rospy.Timer(rospy.Duration(3), send_alternative_trajectory, oneshot=True)
        self.walker.run()

    def test_camera_rotation(self):
        import rospy

        self.walker.setPose(Transformation([0.5, 0.5, 0], [0, 0, 0, 1]))
        self.walker.soccerbot.robot_state.status = RobotState.STATUS_READY
        self.state_publisher = rospy.Publisher("/robot1/state", RobotState)
        s = RobotState()
        s.status = RobotState.STATUS_READY
        for i in range(1, 10):
            self.state_publisher.publish(s)
            rospy.sleep(0.1)

        self.walker.run()
