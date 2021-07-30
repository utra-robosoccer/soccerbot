import math
from unittest import TestCase

import numpy as np
import matplotlib.pyplot as plt
import os


if "ROS_NAMESPACE" not in os.environ:
    os.environ["ROS_NAMESPACE"] = "/robot1"

import soccerbot_controller

from transformation import Transformation

RUN_IN_ROS = True
if RUN_IN_ROS:
    import rospy
    import soccerbot_controller_ros
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseWithCovarianceStamped


class Test(TestCase):

    def setUp(self) -> None:
        if RUN_IN_ROS:
            rospy.init_node("soccer_control")
            self.walker = soccerbot_controller_ros.SoccerbotControllerRos()
        else:
            self.walker = soccerbot_controller.SoccerbotController()


    def test_walk_1(self):
        self.walker.setPose(Transformation([0.5, 0, 0], [0, 0, 0, 1]))
        self.walker.ready()
        self.walker.wait(100)
        self.walker.setGoal(Transformation([2, 0, 0], [0, 0, 0, 1]))
        # self.walker.soccerbot.robot_path.show()
        self.walker.run()

    def test_walk_2(self):
        self.walker.setPose(Transformation([-0.7384, -0.008, 0], [0.00000, 0, 0, 1]))
        self.walker.ready()
        self.walker.wait(100)
        self.walker.setGoal(Transformation([0.0198, -0.0199, 0], [0.00000, 0, 0, 1]))
        # self.walker.soccerbot.robot_path.show()
        self.walker.run()

    def test_walk_3(self):
        self.walker.setPose(Transformation([-2.404, -1.0135, 0], [0, 0, -0.9979391070307153, 0.064168050139]))
        self.walker.ready()
        self.walker.wait(100)
        self.walker.setGoal(Transformation([-2.26, -1.27, 0], [0, 0, 0.997836202477347, 0.06574886330262358]))
        # self.walker.soccerbot.robot_path.show()
        self.walker.run()

    def test_walk_4(self):
        self.walker.setPose(Transformation([0.3275415, 0.2841, 0.321], [ 0.04060593, 0.0120126, 0.86708929, -0.4963497]))
        self.walker.ready()
        self.walker.wait(100)
        self.walker.setGoal(Transformation([-0.12015226, -0.19813691, 0.321], [0, 0, 0.95993011, -0.28023953]))
        # self.walker.soccerbot.robot_path.show()
        self.walker.run()

    def test_walk_5(self):
        self.walker.setPose(Transformation([0.716, -0.4188, 0.0], [0.0149, -0.085, 0.9685, 0.2483]))
        self.walker.ready()
        self.walker.wait(100)
        self.walker.setGoal(Transformation([0.0859, -0.016, 0.0], [0, 0, 0.998, 0.0176]))
        # self.walker.soccerbot.robot_path.show()
        self.walker.run()

    def test_walk_6(self):
        self.walker.setPose(Transformation([2.008, -0.646, 0.0], [0.0149, -0.0474, 0.99985, -0.0072]))
        self.walker.ready()
        self.walker.wait(100)
        self.walker.setGoal(Transformation([0.00736, 0.0356, 0.0], [0, 0, 0.998, 0.0176]))
        # self.walker.soccerbot.robot_path.show()
        self.walker.run()

    def test_walk_7(self):
        self.walker.setPose(Transformation([2.082603318747387, 0.04499586647232634, 0.0], [0.07888602209666294, -0.03018659995378454, 0.9054426772657052, 0.41597995490997813]))
        self.walker.ready()
        self.walker.wait(100)
        self.walker.setGoal(Transformation([2.5901226468203067, 0.7938447967981127, 0.0], [0, 0, -0.9987013856398979, 0.050946465244882694]))
        # self.walker.soccerbot.robot_path.show()
        self.walker.run()

    def test_walk_side(self):
        self.walker.setPose(Transformation([0, 0, 0], [0.00000, 0, 0, 1]))
        self.walker.ready()
        self.walker.wait(100)
        self.walker.setGoal(Transformation([0, -1, 0], [0.00000, 0, 0, 1]))
        # self.walker.soccerbot.robot_path.show()
        self.walker.run()

    def test_walk_backward(self):
        self.walker.setPose(Transformation([0, 0, 0], [0.00000, 0, 0, 1]))
        self.walker.ready()
        self.walker.wait(100)
        self.walker.setGoal(Transformation([-1, 0.3, 0], [0.00000, 0, 0, 1]))
        self.walker.run()

    def test_turn_in_place(self):
        self.walker.setPose(Transformation([0, 0, 0], [0.00000, 0, 0, 1]))
        self.walker.ready()
        self.walker.wait(100)

        goal = Transformation.get_transform_from_euler([np.pi, 0, 0])
        self.walker.setGoal(goal)
        self.walker.run()

    def test_small_movement_0(self):
        self.walker.setPose(Transformation([0, 0, 0], [0.00000, 0, 0, 1]))
        self.walker.ready()
        self.walker.wait(100)
        goal = Transformation.get_transform_from_euler([np.pi/5, 0, 0])
        goal.set_position([0.05, 0.05, 0])
        self.walker.setGoal(goal)
        # self.walker.soccerbot.robot_path.show()
        self.walker.run()

    def test_small_movement_1(self):
        self.walker.setPose(Transformation([0, 0, 0], [0.00000, 0, 0, 1]))
        self.walker.ready()
        self.walker.wait(100)
        goal = Transformation.get_transform_from_euler([np.pi, 0, 0])
        goal.set_position([0.15, 0.05, 0])
        self.walker.setGoal(goal)
        # self.walker.soccerbot.robot_path.show()
        self.walker.run()

    def test_small_movement_2(self):
        self.walker.setPose(Transformation([0, 0, 0], [0.00000, 0, 0, 1]))
        self.walker.ready()
        self.walker.wait(100)
        goal = Transformation.get_transform_from_euler([np.pi, 0, 0])
        goal.set_position([-0.3, 0, 0])
        self.walker.setGoal(goal)
        self.walker.soccerbot.robot_path.show()
        self.walker.run()

    def test_small_movement_3(self):
        self.walker.setPose(Transformation([0, 0, 0], [0.00000, 0, 0, 1]))
        self.walker.ready()
        self.walker.wait(100)
        goal = Transformation.get_transform_from_euler([-np.pi/2, 0, 0])
        goal.set_position([-0.2, -0.2, 0])
        self.walker.setGoal(goal)
        self.walker.soccerbot.robot_path.show()
        self.walker.run()

    def test_small_movement_4(self):
        self.walker.setPose(Transformation([0.2489, -0.163, 0.0], [0.0284, -0.003, 0.9939, 0.01986]))
        self.walker.ready()
        self.walker.wait(100)
        self.walker.setGoal(Transformation([0.0503, 0.06323, 0], [0, 0, 1, 0]))
        # self.walker.soccerbot.robot_path.show()
        self.walker.run()

    def test_small_movement_5(self):
        self.walker.setPose(Transformation([0.3096807057334623, 0.09374110438873018, 0.0], [0.03189331238935847, -0.0065516868290173, 0.9990119776602083, 0.03024831426656374]))
        self.walker.ready()
        self.walker.wait(100)
        self.walker.setGoal(Transformation([0.14076394628045208, -0.034574636811865296, 0], [0, 0, -0.9999956132297835, -0.002962013029887055]))
        # self.walker.soccerbot.robot_path.show()
        self.walker.run()

    def test_do_nothing(self):
        self.walker.setPose(Transformation([0, 0, 0], [0.00000, 0, 0, 1]))
        self.walker.ready()
        self.walker.wait(100)
        goal = Transformation.get_transform_from_euler([0, 0, 0])
        self.walker.setGoal(goal)
        # self.walker.soccerbot.robot_path.show()
        self.walker.run()

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
                pb.setJointMotorControlArray(bodyIndex=self.walker.soccerbot.body, controlMode=pb.POSITION_CONTROL,
                                             jointIndices=list(range(0, 20, 1)),
                                             targetPositions=self.walker.soccerbot.get_angles(),
                                             forces=forces
                                             )
                self.walker.soccerbot.current_step_time = self.walker.soccerbot.current_step_time + self.walker.soccerbot.robot_path.step_size

            pb.stepSimulation()
            t = t + self.walker.PYBULLET_STEP

        plt.plot(times, pitches)
        plt.xlabel("Time (t)")
        plt.ylabel("Forward pitch of robot in radians")
        plt.grid(which='minor')
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
        r = rospy.Rate(1/SoccerbotController.PYBULLET_STEP)

        while t <= self.walker.soccerbot.robot_path.duration():
            if self.walker.soccerbot.current_step_time <= t <= self.walker.soccerbot.robot_path.duration():
                self.walker.soccerbot.stepPath(t, verbose=True)
                pitch = self.walker.soccerbot.get_imu().get_orientation_euler()[1]
                f = self.walker.soccerbot.apply_imu_feedback(t, self.walker.soccerbot.get_imu())

                times.append(t)
                pitches.append((pitch, f))

                forces = self.walker.soccerbot.apply_foot_pressure_sensor_feedback(self.walker.ramp.plane)
                pb.setJointMotorControlArray(bodyIndex=self.walker.soccerbot.body, controlMode=pb.POSITION_CONTROL,
                                             jointIndices=list(range(0, 20, 1)),
                                             targetPositions=self.walker.soccerbot.get_angles(),
                                             forces=forces
                                             )
                self.walker.soccerbot.current_step_time = self.walker.soccerbot.current_step_time + self.walker.soccerbot.robot_path.step_size

            pb.stepSimulation()
            self.walker.soccerbot.publishAngles()
            t = t + self.walker.PYBULLET_STEP
            r.sleep()

        plt.plot(times, pitches)
        plt.xlabel("Time (t)")
        plt.ylabel("Forward pitch of robot in radians")
        plt.grid(which='minor')
        plt.show()

    def test_foot_pressure_synchronization(self):
        import pybullet as pb
        fig, axs = plt.subplots(2)

        self.walker.setPose(Transformation([0, 0, 0], [0, 0, 0, 1]))
        self.walker.ready()
        self.walker.wait(100)
        self.walker.setGoal(Transformation([1, 0, 0], [0, 0, 0, 1]))

        times = np.linspace(0, self.walker.soccerbot.robot_path.duration(), num=math.ceil(self.walker.soccerbot.robot_path.duration() / self.walker.soccerbot.robot_path.step_size) + 1)
        lfp = np.zeros((4, 4, len(times)))
        rfp = np.zeros((4, 4, len(times)))
        i = 0
        for t in times:
            [lfp[:,:, i], rfp[:,:, i]] = self.walker.soccerbot.robot_path.footPosition(t)
            i = i + 1

        right = rfp[2, 3, :].ravel()
        left = lfp[2, 3, :].ravel()
        right = 0.1 - right
        left = left - 0.1
        axs[1].plot(times, left, label='Left')
        axs[1].plot(times, right, label='Right')
        axs[1].grid(b=True, which='both', axis='both')
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
                pb.setJointMotorControlArray(bodyIndex=self.walker.soccerbot.body, controlMode=pb.POSITION_CONTROL,
                                             jointIndices=list(range(0, 20, 1)),
                                             targetPositions=self.walker.soccerbot.get_angles(),
                                             forces=forces
                                             )
                self.walker.soccerbot.current_step_time = self.walker.soccerbot.current_step_time + self.walker.soccerbot.robot_path.step_size

            pb.stepSimulation()
            t = t + self.walker.PYBULLET_STEP

        axs[0].scatter(scatter_pts_x, scatter_pts_y, s=3)
        axs[1].scatter(scatter_pts_x_1, scatter_pts_y_1, s=3)
        plt.show()

    def test_calculate_accuracy_matrix(self):

        x_range = np.arange(-1, 1, 0.2)
        y_range = np.arange(-1, 1, 0.2)
        ang_range = np.arange(-np.pi, np.pi, np.pi/6)

        x, y, ang = np.meshgrid(x_range, y_range, ang_range)

        fake_localization_pose_subscriber = rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.amcl_pose_callback)

        for i in x_range:
            for j in y_range:
                for k in ang_range:
                    b = String()
                    b.data = ""
                    self.resetPublisher.publish(b)

                    self.walker.setPose(Transformation([0, 0, 0], [0, 0, 0, 1]))
                    self.walker.ready()
                    t = Transformation.get_transform_from_euler([k, 0, 0])
                    t.set_position([i, j, 0])
                    self.walker.setGoal(t)
                    # self.walker.updateGoal()
                    # self.walker.soccerbot.robot_path.show()
                    self.walker.wait(100)
                    self.walker.run(True)

        pass

    def amcl_pose_callback(self, amcl_pose):
        self.amcl_pose = amcl_pose
        pass

    def test_dynamic_walking_1(self):
        import rospy

        self.walker.setPose(Transformation([0.5, 0, 0], [0, 0, 0, 1]))
        self.walker.ready()
        self.walker.wait(100)
        self.walker.setGoal(Transformation([1.5, 0, 0], [0, 0, 0, 1]))

        def send_alternative_trajectory(_):
            self.walker.setGoal(Transformation([2.0, 0, 0], [0, 0, 0, 1]))
            pass
        self.send_alternative_trajectory = rospy.Timer(rospy.Duration(3), send_alternative_trajectory, oneshot=True)
        self.walker.run()



