import copy
import os

import rospy
import tf
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from rospy import ROSInterruptException
from soccer_pycontrol.old.navigator import *
from soccer_pycontrol.old.soccerbot_ros import SoccerbotRos
from soccer_pycontrol.path.path_section import PathSection
from std_msgs.msg import Bool, Empty

from soccer_msgs.msg import RobotState


class NavigatorRos(Navigator):
    """
    The Navigator class, receives commands from ROS and executes them in real life and in webots simulation. For functions
    common to both ROS and pybullet simulation, see :class:`Navigator`
    """

    def __init__(self, useCalibration=False):
        """
        Initializes The NavigatorRos class

        :param useCalibration: Whether or not to use movement calibration files located in config/robot_model.yaml, which adjusts the calibration to the movement given
        """
        self.time_now = 0
        print("hehr: ", type(pb))
        self.client_id = pb.connect(pb.DIRECT)
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
        pb.resetDebugVisualizerCamera(cameraDistance=0.5, cameraYaw=0, cameraPitch=0, cameraTargetPosition=[0, 0, 0.25])
        pb.setGravity(0, 0, -9.81)
        self.soccerbot = SoccerbotRos(Transformation(), useFixedBase=False, useCalibration=useCalibration)
        pb.disconnect(self.client_id)

        self.position_subscriber = rospy.Subscriber("goal", PoseStamped, self.goal_callback)
        self.completed_walk_publisher = rospy.Publisher("action_complete", Empty, queue_size=1)
        self.goal = PoseStamped()
        self.robot_pose: PoseStamped = None
        self.new_goal = self.goal
        self.terminated = None

        self.prepare_walk_time = rospy.get_param("prepare_walk_time", 2)

        self.tf_listener = tf.TransformListener()

        self.t = 0
        self.r = rospy.Rate(1 / Navigator.PYBULLET_STEP)

    def update_robot_pose(self, footprint_name="/base_footprint") -> bool:
        """
        Function to update the location of the robot based on odometry. Called before movement to make sure the starting
        position is correct

        :param footprint_name:
        :return: True if the position is updated, otherwise False
        """
        try:
            (trans, rot) = self.tf_listener.lookupTransform("world", os.environ["ROS_NAMESPACE"] + footprint_name, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print(e)
            return False

        self.robot_pose = Transformation(position=trans, quaternion=rot).pose_stamped
        return True

    def setPose(self, pose: Transformation):
        [r, p, y] = pose.orientation_euler
        pose.orientation_euler = [r, 0, 0]

        resetPublisher = rospy.Publisher("/robot1/reset_robot", PoseStamped, queue_size=1, latch=True)
        initialPosePublisher = rospy.Publisher("initialpose", PoseWithCovarianceStamped, queue_size=1, latch=True)
        pose_stamped = pose.pose_stamped
        resetPublisher.publish(pose_stamped)
        self.robot_pose = pose_stamped

        rospy.sleep(0.2)

        p = PoseWithCovarianceStamped()
        p.header.frame_id = "world"
        p.header.stamp = rospy.Time.now()
        p.pose.pose = pose_stamped.pose
        initialPosePublisher.publish(p)

        rospy.sleep(0.2)

    def getPose(self, footprint_name="/base_footprint_gt"):
        try:
            (trans, rot) = self.tf_listener.lookupTransform("world", os.environ["ROS_NAMESPACE"] + footprint_name, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print(e)
            return False

        return Transformation(position=trans, quaternion=rot).pos_theta

    def setGoal(self, goal: Transformation):
        self.goal_callback(goal.pose_stamped)

    def wait(self, steps: int):
        for i in range(steps):
            rospy.sleep(Navigator.PYBULLET_STEP)

    def goal_callback(self, pose: PoseStamped) -> None:
        """
        Callback function for when a new goal arrives. It creates the path in the callback function and dynamically
        updates the current path if it exists (currently not working). Note the path computation occurs here
        instead of run because the computation will disturb the main thread. Main thread should still be running

        :param pose: The pose sent by the strategy for the robot to go to
        """

        # Update existing path
        #
        if self.soccerbot.robot_path is not None:
            print("Updating New Goal")
            start = time.time()
            goal_position = Transformation(pose=pose.pose)
            self.soccerbot.setWalkingTorsoHeight(goal_position)
            self.new_path = copy.deepcopy(self.soccerbot.robot_path)

            try:
                self.t_new_path = self.new_path.dynamicallyUpdateGoalPosition(self.t, goal_position)
            except Exception as ex:
                print(ex)
                return

            end = time.time()

            self.soccerbot.publishPath(self.new_path)
            print("New Goal Updated, Time Taken: ", end - start)
            pass
        self.new_goal = pose

    # def setup(self):

    def wait_disconnected(self):
        while self.soccerbot.robot_state.status == RobotState.STATUS_DISCONNECTED:
            try:
                self.r.sleep()
            except ROSInterruptException:
                exit(0)

    def check_state(self):
        not_walk = False
        if self.soccerbot.robot_state.status in [
            RobotState.STATUS_DISCONNECTED,
            RobotState.STATUS_FALLEN_FRONT,
            RobotState.STATUS_FALLEN_BACK,
            RobotState.STATUS_FALLEN_SIDE,
            RobotState.STATUS_GETTING_BACK_UP,
            RobotState.STATUS_KICKING,
            RobotState.STATUS_PENALIZED,
        ]:
            self.soccerbot.robot_path = None
            self.goal = self.new_goal
            self.soccerbot.reset_imus()
            self.soccerbot.updateRobotConfiguration()
            not_walk = True
        return not_walk

    def check_term(self):
        walk_term = False
        if self.soccerbot.robot_state.status in [RobotState.STATUS_TERMINATING_WALK]:
            if not self.terminated:
                rospy.loginfo("Terminating Walk at time " + str(self.t))
                if self.soccerbot.robot_path is not None:
                    self.soccerbot.robot_path.terminateWalk(self.t)
                self.terminated = True

            self.goal = self.new_goal
            self.soccerbot.reset_imus()
            self.soccerbot.updateRobotConfiguration()
            walk_term = True
        return walk_term

    def run(self, single_trajectory=False):
        self.t = 0
        stable_count = 5

        if single_trajectory:
            self.soccerbot.robot_state.status = RobotState.STATUS_WALKING

        self.wait_disconnected()

        self.soccerbot.ready()
        self.soccerbot.reset_imus()
        self.soccerbot.updateRobotConfiguration()
        self.time_now = 0

        while not rospy.is_shutdown():
            time_start = time.time()

            # Always publish odometry no matter what state
            self.soccerbot.publishOdometry(self.r.last_time)

            # Always apply head rotation
            self.soccerbot.apply_head_rotation()

            not_walk = self.check_state()
            if not_walk:
                self.r.sleep()
                continue

            walk_term = self.check_term()
            if walk_term:
                self.r.sleep()
                continue

            # New goal added
            pose_updated = self.new_goal_added()
            if not single_trajectory:
                if not pose_updated:
                    rospy.loginfo_throttle(1, "Unable to get Robot Pose")
                    self.r.sleep()
                    continue

            self.update_goal()

            self.walk_loop()

            self.walk_completed()

            if self.soccerbot.robot_path is None or self.t > self.soccerbot.robot_path.duration():
                self.soccerbot.robot_path = None
                pass

            if self.t < 0:
                if self.soccerbot.imu_ready:
                    pitch = self.soccerbot.apply_imu_feedback_standing(self.soccerbot.get_imu())
                    rospy.loginfo_throttle(
                        0.3,
                        "Performing prewalk stabilization, distance to desired pitch: " + str(pitch - self.soccerbot.standing_pid.setpoint),
                    )
                    if abs(pitch - self.soccerbot.standing_pid.setpoint) < 0.025:
                        stable_count = stable_count - 1
                        if stable_count == 0:
                            t = 0
                    else:
                        stable_count = 5
            elif self.soccerbot.robot_state.status == RobotState.STATUS_WALKING and (
                self.soccerbot.robot_path is None or self.t > self.soccerbot.robot_path.duration()
            ):
                rospy.loginfo_throttle_identical(1, "Performing post stabilization")
                if self.soccerbot.imu_ready:
                    self.soccerbot.apply_imu_feedback_standing(self.soccerbot.get_imu())
                    pass

            if single_trajectory:
                if self.soccerbot.robot_path is None:
                    return True

                if self.soccerbot.imu_ready:
                    q = self.soccerbot.imu_msg.orientation
                    [roll, pitch, yaw] = Transformation.get_euler_from_quaternion([q.w, q.x, q.y, q.z])
                    self.fallen(pitch)

            # Publishes angles to robot (Average Time: 0.00041992547082119)
            self.soccerbot.publishAngles()

            time_end = time.time()
            if time_end - time_start > Navigator.PYBULLET_STEP * 1.2:
                rospy.logerr_throttle(
                    10,
                    f"Step Delta took longer than expected {time_end - time_start}. Control Frequency {self.PYBULLET_STEP}",
                )

            self.t = self.t + Navigator.PYBULLET_STEP

            self.r.sleep()

    def new_goal_added(self):
        if self.new_goal != self.goal and self.soccerbot.robot_path is None:
            pose_updated = self.update_robot_pose()
            if not pose_updated:
                return pose_updated

            rospy.loginfo("Received New Goal")
            self.time_now = rospy.Time.now()

            self.check_tol_new_goal()

            self.goal = self.new_goal
            self.soccerbot.reset_imus()
            self.soccerbot.ready()
            self.soccerbot.setPose(Transformation(pose=self.robot_pose.pose))

            def print_pose(name: str, pose: Pose):
                print(
                    f"\033[92m{name}: Position (xyz) [{pose.position.x:.3f} {pose.position.y:.3f} {pose.position.z:.3f}], Orientation (xyzw) [{pose.orientation.x:.3f} {pose.orientation.y:.3f} {pose.orientation.z:.3f} {pose.orientation.w:.3f}]\033[0m"
                )

            print_pose("Start Pose", self.robot_pose.pose)
            print_pose("End Pose", self.goal.pose)
            self.soccerbot.createPathToGoal(Transformation(pose=self.goal.pose))
            self.soccerbot.reset_roll_feedback_parameters()
            self.t = -self.prepare_walk_time

            # self.soccerbot.robot_path.show()
            self.soccerbot.publishPath()
            self.terminated = False
        return True

    def check_tol_new_goal(self):
        # Minimum goal movement tolerance check
        if (self.new_goal.pose.position.x - self.robot_pose.pose.position.x) ** 2 + (
            self.new_goal.pose.position.y - self.robot_pose.pose.position.y
        ) ** 2 < 0.03**2:
            self.new_goal.pose.position.x = self.robot_pose.pose.position.x
            self.new_goal.pose.position.y = self.robot_pose.pose.position.y

    def update_goal(self):
        # Existing goal updated
        if self.new_goal != self.goal and self.soccerbot.robot_path is not None and self.t > self.t_new_path:
            print("Updating Existing Goal and Path" + str(self.new_goal))
            self.goal = self.new_goal
            self.soccerbot.robot_path = self.new_path

    def walk_loop(self):
        if self.soccerbot.robot_path is not None and 0 <= self.t <= self.soccerbot.robot_path.duration():

            # IMU feedback while walking (Average Time: 0.00017305118281667)
            t_adj = self.t
            if self.soccerbot.imu_ready:
                imu_pose = self.soccerbot.get_imu()
                self.soccerbot.apply_imu_feedback(imu_pose)
                t_adj = self.soccerbot.apply_phase_difference_roll_feedback(self.t, imu_pose)

            self.soccerbot.stepPath(t_adj)

            self.soccerbot.current_step_time = self.t

    def walk_completed(self):
        # Walk completed
        if (
            self.soccerbot.robot_path is not None
            and self.t <= self.soccerbot.robot_path.duration() < self.t + Navigator.PYBULLET_STEP
            and not self.terminated
        ):
            walk_time = rospy.Time.now().secs + (rospy.Time.now().nsecs / 100000000) - (self.time_now.secs + (self.time_now.nsecs / 100000000))
            rospy.loginfo(f"\033[92mCompleted Walk, Took: {walk_time} \033[0m")
            e = Empty()
            self.completed_walk_publisher.publish(e)

    @staticmethod
    def fallen(pitch: float) -> bool:
        angle_threshold = 1.25  # in radian
        if pitch > angle_threshold:
            print("Fallen Back")
            return True

        elif pitch < -angle_threshold:
            print("Fallen Front")
            return True
