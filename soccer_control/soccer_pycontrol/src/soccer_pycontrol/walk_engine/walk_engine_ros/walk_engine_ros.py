import copy
import os
import time

import rospy
import tf
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rospy import ROSInterruptException
from soccer_pycontrol.model.model_ros.bez_ros import BezROS
from soccer_pycontrol.walk_engine.walk_engine import WalkEngine
from soccer_pycontrol.walk_engine.walk_engine_ros.foot_step_planner_ros import (
    FootStepPlannerROS,
)
from soccer_pycontrol.walk_engine.walk_engine_ros.stabilize_ros import StabilizeROS
from std_msgs.msg import Empty

from soccer_common import Transformation
from soccer_msgs.msg import RobotState

# TODO clean up and remove unused


class WalkEngineROS(WalkEngine):
    def __init__(self, bez: BezROS):
        self.PYBULLET_STEP = 0.005  # rospy.get_param("control_frequency", 0.005)
        self.bez = bez
        self.step_planner = FootStepPlannerROS(
            walking_torso_height=self.bez.data.walking_torso_height,
            foot_center_to_floor=self.bez.data.foot_center_to_floor,
            torso_offset_pitch=self.bez.data.torso_offset_pitch,
            torso_offset_x=self.bez.data.torso_offset_x,
        )  # TODO should this be past or should it get through rosparam

        self.pid = StabilizeROS()

        self.t = 0

        self.position_subscriber = rospy.Subscriber("goal", PoseStamped, self.goal_callback)
        self.goal = PoseStamped()

    def wait(self, steps: int):
        for i in range(steps):
            rospy.sleep(self.PYBULLET_STEP)

    def goal_callback(self, pose: PoseStamped) -> None:
        """
        Callback function for when a new goal arrives. It creates the path in the callback function and dynamically
        updates the current path if it exists (currently not working). Note the path computation occurs here
        instead of run because the computation will disturb the main thread. Main thread should still be running

        :param pose: The pose sent by the strategy for the robot to go to
        """

        self.goal = pose

        if self.step_planner.robot_path is None:
            self.pid.reset_imus()
            self.bez.ready()

            self.step_planner.create_path_to_goal(Transformation(pose=self.goal.pose))
            self.t = -self.prepare_walk_time

            self.step_planner.publishPath()

    def run(self, single_trajectory=True):
        # set time settings
        self.t = 0
        r = rospy.Rate(1 / self.PYBULLET_STEP)
        stable_count = 5

        self.bez.ready()
        self.bez.motor_control.set_motor()
        self.pid.reset_imus()
        time_now = 0

        while not rospy.is_shutdown():
            # New goal added
            # TODO fix this layer it is a mess and needs global loc

            if self.bez.sensors.imu_ready:
                [_, pitch, roll] = self.bez.sensors.get_euler_angles()
                # path in progress
                if self.step_planner.robot_path is not None and 0 <= self.t <= self.step_planner.robot_path.duration():

                    # IMU feedback while walking (Average Time: 0.00017305118281667)
                    t_adj = self.t
                    if self.bez.sensors.imu_ready:
                        # TODO needs to be fixed
                        pass
                        # self.stabilize_walk(pitch, roll)

                    torso_to_right_foot, torso_to_left_foot = self.step_planner.get_next_step(t_adj)
                    self.bez.find_joint_angles(torso_to_right_foot, torso_to_left_foot)
                    self.step_planner.current_step_time = self.t

            # Stabilize
            # if self.t < 0 or (
            #     self.bez.robot_state.status == RobotState.STATUS_WALKING
            #     and (self.step_planner.robot_path is None or self.t > self.step_planner.robot_path.duration())
            # ):
            #     if self.bez.sensors.imu_ready:
            #         stable_count = self.update_stable_count(pitch, roll, stable_count)
            #         if stable_count == 0:  # TODO dont really like this format
            #             t = 0  # TODO need to fix later
            #         self.stabilize_stand(pitch, roll)

            if self.bez.sensors.imu_ready:
                [_, pitch, roll] = self.bez.sensors.get_euler_angles()
                self.bez.fallen(pitch)

            self.bez.motor_control.set_motor()

            self.t = self.t + self.PYBULLET_STEP

            r.sleep()
