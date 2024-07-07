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


class WalkEngineROS(WalkEngine):
    def __init__(self, bez: BezROS):
        self.PYBULLET_STEP = rospy.get_param("control_frequency", 0.01)
        self.bez = bez
        self.step_planner = FootStepPlannerROS(
            walking_torso_height=self.bez.data.walking_torso_height, foot_center_to_floor=self.bez.data.foot_center_to_floor
        )  # TODO should this be past or should it get through rosparam

        self.pid = StabilizeROS()

        self.terminate_walk = False
        self.prepare_walk_time = rospy.get_param("prepare_walk_time", 2)

        self.t = 0

        self.odom_publisher = rospy.Publisher("odom", Odometry, queue_size=1)
        self.br = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()
        self.completed_walk_publisher = rospy.Publisher("action_complete", Empty, queue_size=1)
        self.position_subscriber = rospy.Subscriber("goal", PoseStamped, self.goal_callback)
        self.goal = PoseStamped()
        self.new_goal = self.goal
        self.terminated = None

    def publishOdometry(self, time: rospy.Time):
        """
        Send the odometry of the robot to be used in localization by ROS
        """

        o = Odometry()
        o.header.stamp = time
        o.header.frame_id = os.environ["ROS_NAMESPACE"][1:] + "/odom"
        o.child_frame_id = os.environ["ROS_NAMESPACE"][1:] + "/base_link"
        o.pose.pose = self.step_planner.odom_pose.pose
        o.pose.pose.position.z = 0

        # fmt: off
        o.pose.covariance = [1e-4, 0, 0, 0, 0, 0,
                             0, 1e-4, 0, 0, 0, 0,
                             0, 0, 1e-18, 0, 0, 0,
                             0, 0, 0, 1e-18, 0, 0,
                             0, 0, 0, 0, 1e-18, 0,
                             0, 0, 0, 0, 0, 1e-2]
        # fmt: on
        self.odom_publisher.publish(o)

        # TODO this seems to be static  self.pose.position[2] = 0.34
        # Publish the height of the center of the torso of the robot (used for camera vision calculations and odometry)
        height = 0.34 + self.bez.data.foot_box[2] / 2 + self.bez.data.cleats_offset
        self.br.sendTransform(
            (0, 0, height),
            (0, 0, 0, 1),
            time,
            os.environ["ROS_NAMESPACE"] + "/torso",
            os.environ["ROS_NAMESPACE"] + "/base_footprint",
        )
        pass

    def setGoal(self, goal: Transformation):
        self.goal_callback(goal.pose_stamped)

    def goal_callback(self, pose: PoseStamped) -> None:
        """
        Callback function for when a new goal arrives. It creates the path in the callback function and dynamically
        updates the current path if it exists (currently not working). Note the path computation occurs here
        instead of run because the computation will disturb the main thread. Main thread should still be running

        :param pose: The pose sent by the strategy for the robot to go to
        """
        # TODO should be in footstepplanner
        # Update existing path
        #
        if self.step_planner.robot_path is not None:
            print("Updating New Goal")
            start = time.time()
            goal_position = Transformation(pose=pose.pose)
            self.bez.set_walking_torso_height(goal_position)
            self.new_path = copy.deepcopy(self.step_planner.robot_path)

            try:
                self.t_new_path = self.new_path.dynamicallyUpdateGoalPosition(self.t, goal_position)
            except Exception as ex:
                print(ex)
                return

            end = time.time()

            self.step_planner.publishPath(self.new_path)
            print("New Goal Updated, Time Taken: ", end - start)
            pass
        self.new_goal = pose

    def run(self, single_trajectory=False):
        # set time settings
        self.t = 0
        r = rospy.Rate(1 / self.PYBULLET_STEP)
        stable_count = 5

        if single_trajectory:
            self.bez.robot_state.status = RobotState.STATUS_WALKING

        # wait
        while self.bez.robot_state.status == RobotState.STATUS_DISCONNECTED:
            try:
                r.sleep()
            except ROSInterruptException:
                exit(0)

        self.bez.ready()
        self.pid.reset_imus()
        self.bez.motor_control.updateRobotConfiguration()
        time_now = 0

        while not rospy.is_shutdown():
            time_start = time.time()

            # Always publish odometry no matter what state
            self.publishOdometry(r.last_time)

            # Always apply head rotation
            self.bez.apply_head_rotation()

            # Check if not in a walking state
            if self.bez.robot_state.status in [
                RobotState.STATUS_DISCONNECTED,
                RobotState.STATUS_FALLEN_FRONT,
                RobotState.STATUS_FALLEN_BACK,
                RobotState.STATUS_FALLEN_SIDE,
                RobotState.STATUS_GETTING_BACK_UP,
                RobotState.STATUS_KICKING,
                RobotState.STATUS_PENALIZED,
            ]:
                self.step_planner.robot_path = None
                self.goal = self.new_goal
                self.pid.reset_imus()
                self.bez.motor_control.updateRobotConfiguration()
                r.sleep()
                continue
            # CHeck if walk terminated
            if self.bez.robot_state.status in [RobotState.STATUS_TERMINATING_WALK]:
                if not self.terminated:
                    rospy.loginfo("Terminating Walk at time " + str(self.t))
                    if self.step_planner.robot_path is not None:
                        self.step_planner.robot_path.terminateWalk(self.t)
                    self.terminated = True

                self.goal = self.new_goal
                self.pid.reset_imus()
                self.bez.motor_control.updateRobotConfiguration()
                r.sleep()
                continue

            # New goal added
            if self.new_goal != self.goal and self.step_planner.robot_path is None:
                if not single_trajectory:
                    pose_updated = self.bez.update_robot_pose()
                    if not pose_updated:
                        rospy.loginfo_throttle(1, "Unable to get Robot Pose")
                        r.sleep()
                        continue

                rospy.loginfo("Received New Goal")
                time_now = rospy.Time.now()

                # Minimum goal movement tolerance check
                if (self.new_goal.pose.position.x - self.bez.robot_pose.pose.position.x) ** 2 + (
                    self.new_goal.pose.position.y - self.bez.robot_pose.pose.position.y
                ) ** 2 < 0.03**2:
                    self.new_goal.pose.position.x = self.bez.robot_pose.pose.position.x
                    self.new_goal.pose.position.y = self.bez.robot_pose.pose.position.y

                self.goal = self.new_goal
                self.pid.reset_imus()
                self.bez.ready()
                self.bez.setPose(Transformation(pose=self.bez.robot_pose.pose))

                def print_pose(name: str, pose: Pose):
                    print(
                        f"\033[92m{name}: Position (xyz) [{pose.position.x:.3f} {pose.position.y:.3f} {pose.position.z:.3f}], Orientation (xyzw) [{pose.orientation.x:.3f} {pose.orientation.y:.3f} {pose.orientation.z:.3f} {pose.orientation.w:.3f}]\033[0m"
                    )

                print_pose("Start Pose", self.bez.robot_pose.pose)
                print_pose("End Pose", self.goal.pose)
                self.step_planner.create_path_to_goal(Transformation(pose=self.goal.pose))
                # self.pid_stab.reset_roll_feedback_parameters()
                self.t = -self.prepare_walk_time

                # self.soccerbot.robot_path.show()
                self.step_planner.publishPath()
                self.terminated = False

            # Existing goal updated
            if self.new_goal != self.goal and self.step_planner.robot_path is not None and self.t > self.t_new_path:
                print("Updating Existing Goal and Path" + str(self.new_goal))
                self.goal = self.new_goal
                self.step_planner.robot_path = self.new_path

            [_, pitch, roll] = self.bez.sensors.get_euler_angles()
            # path in progress
            if self.step_planner.robot_path is not None and 0 <= self.t <= self.step_planner.robot_path.duration():

                # IMU feedback while walking (Average Time: 0.00017305118281667)
                t_adj = self.t
                if self.bez.sensors.imu_ready:
                    # TODO needs to be fixed
                    self.stabilize_walk(pitch, roll)
                    # t_adj = self.soccerbot.apply_phase_difference_roll_feedback(self.t, imu_pose)

                self.step_planner.get_next_step(t_adj)

                self.step_planner.current_step_time = self.t

            # Walk completed
            if (
                self.step_planner.robot_path is not None
                and self.t <= self.step_planner.robot_path.duration() < self.t + self.PYBULLET_STEP
                and not self.terminated
            ):
                walk_time = rospy.Time.now().secs + (rospy.Time.now().nsecs / 100000000) - (time_now.secs + (time_now.nsecs / 100000000))
                rospy.loginfo(f"\033[92mCompleted Walk, Took: {walk_time} \033[0m")
                e = Empty()
                self.completed_walk_publisher.publish(e)

            # update path if completed TODO why here
            if self.step_planner.robot_path is None or self.t > self.step_planner.robot_path.duration():
                self.step_planner.robot_path = None
                pass

            # Stabilize
            if self.t < 0 or (
                self.bez.robot_state.status == RobotState.STATUS_WALKING
                and (self.step_planner.robot_path is None or self.t > self.step_planner.robot_path.duration())
            ):
                if self.bez.sensors.imu_ready:
                    stable_count = self.update_stable_count(pitch, roll, stable_count)
                    if stable_count < 0:  # TODO dont really like this format
                        break
                    self.stabilize_stand(pitch, roll)

            if single_trajectory:
                if self.step_planner.robot_path is None:
                    return True

                if self.bez.sensors.imu_ready:
                    if self.bez.fallen(pitch):
                        return False
            # Publishes angles to robot (Average Time: 0.00041992547082119)
            # self.soccerbot.robot_path.show()
            self.bez.motor_control.set_motor()

            time_end = time.time()
            if time_end - time_start > self.PYBULLET_STEP * 1.2:
                rospy.logerr_throttle(
                    10,
                    f"Step Delta took longer than expected {time_end - time_start}. Control Frequency {self.PYBULLET_STEP}",
                )

            self.t = self.t + self.PYBULLET_STEP

            r.sleep()
